//! Embedded CPython hosts for standalone Cerulion node cdylibs.
//!
//! A process may host one interpreter at a time. The host assumes GIL
//! serialization, no Python-created threads or fork. During initialization,
//! the host installs no Python signal handlers and never changes the process's
//! SIGINT disposition. Build one node cdylib per CPython minor:
//! the resulting library has a `libpython` `NEEDED` entry and is not an abi3
//! extension. This host is intended for deterministic graph ticks, not kHz
//! Python loops. Optional ABI exports not implemented by this crate are
//! omitted from the macro expansion. A leaked memoryview of a discarded loan
//! pins one publisher pool slot until Python releases it.
#![deny(unsafe_op_in_unsafe_fn)]

use cerulion_core::codegen::{parse_rosmsg, MessageSchema};
use cerulion_core::dynamic::{FrameEncoder, SchemaSet};
use cerulion_core::graph::node::{AnyPublisher, NodeContext};
use cerulion_core::transport::publisher::RawShmLoan;
use cerulion_core::transport::subscriber::RawInputView;
use cerulion_core::wire::WireHeader;
use pyo3::exceptions::{PyBufferError, PyRuntimeError, PyValueError};
use pyo3::ffi;
use pyo3::prelude::*;
use std::cell::{Cell, RefCell};
use std::collections::{HashMap, HashSet};
#[cfg(all(not(Py_LIMITED_API), not(cerulion_pynode_limited)))]
use std::ffi::CString;
use std::rc::Rc;
#[cfg(all(not(Py_LIMITED_API), not(cerulion_pynode_limited)))]
use std::sync::{Once, OnceLock};

pub use cerulion_core;

fn builtin_schemas() -> Vec<MessageSchema> {
    native_ros2_messages::BUILTIN_MSGS
        .iter()
        .filter_map(|&(package, name, text)| {
            parse_rosmsg(text, name, Some(package))
                .map_err(|error| {
                    tracing::warn!(
                        package,
                        name,
                        error = %error,
                        "failed to parse built-in ROS 2 schema for Python node host"
                    );
                })
                .ok()
        })
        .collect()
}

/// The workspace whose `schemas/` a node resolves against: `CERULION_WORKSPACE`
/// when set, else the workspace that owns the node's baked source directory
/// (`<workspace>/nodes/<type>`, the first `sys_path` entry), else the cwd.
fn node_workspace(ctx: &NodeContext, sys_path: &[&str]) -> Result<std::path::PathBuf, String> {
    let explicit = ctx.env_str("CERULION_WORKSPACE", "");
    if !explicit.is_empty() {
        return Ok(explicit.into());
    }
    if let Some(root) = sys_path
        .first()
        .map(std::path::Path::new)
        .and_then(std::path::Path::parent)
        .and_then(std::path::Path::parent)
        .filter(|root| root.join("nodes").is_dir())
    {
        return Ok(root.to_path_buf());
    }
    std::env::current_dir().map_err(|error| error.to_string())
}

fn node_schemas(workspace: &std::path::Path) -> Result<SchemaSet, String> {
    let (workspace_set, warnings) =
        SchemaSet::from_workspace_dir(workspace).map_err(|error| error.to_string())?;
    for warning in warnings {
        tracing::warn!(warning = %warning, "workspace schema warning for Python node host");
    }
    let mut schemas = builtin_schemas();
    for schema in workspace_set.schemas() {
        let qualified_name = schema.qualified_name();
        schemas.retain(|builtin| builtin.qualified_name() != qualified_name);
        schemas.push(schema.clone());
    }
    SchemaSet::from_schemas(schemas)
        .map(|(schemas, _)| schemas)
        .map_err(|error| error.to_string())
}

fn python_error(error: PyErr) -> String {
    Python::attach(|py| {
        let traceback = py.import("traceback").and_then(|module| {
            module.call_method1(
                "format_exception",
                (error.get_type(py), error.value(py), error.traceback(py)),
            )
        });
        match traceback {
            Ok(value) => value
                .extract::<Vec<String>>()
                .map(|parts| parts.concat())
                .unwrap_or_else(|_| error.to_string()),
            Err(_) => error.to_string(),
        }
    })
}

#[pyclass(unsendable)]
struct HostCtx {
    ptr: Rc<Cell<*mut NodeContext>>,
}

#[pymethods]
impl HostCtx {
    fn now_ns(&self) -> PyResult<u64> {
        let ptr = self.ptr.get();
        if ptr.is_null() {
            return Err(PyRuntimeError::new_err("node context is no longer alive"));
        }
        // SAFETY: Host nulls the pointer in shutdown and Drop before releasing
        // the boxed NodeContext; the GIL serializes all access.
        Ok(unsafe { (*ptr).clock().now_ns() })
    }

    #[pyo3(signature = (name, default=None))]
    fn env(&self, name: &str, default: Option<String>) -> PyResult<Option<String>> {
        let ptr = self.ptr.get();
        if ptr.is_null() {
            return Err(PyRuntimeError::new_err("node context is no longer alive"));
        }
        // SAFETY: Host nulls the pointer in shutdown and Drop before releasing
        // the boxed NodeContext; the GIL serializes all access.
        Ok(unsafe { (*ptr).env_opt(name) }.or(default))
    }

    fn request_shutdown(&self) -> PyResult<()> {
        let ptr = self.ptr.get();
        if ptr.is_null() {
            return Err(PyRuntimeError::new_err("node context is no longer alive"));
        }
        // SAFETY: Host nulls the pointer in shutdown and Drop before releasing
        // the boxed NodeContext; the GIL serializes all access.
        unsafe { (*ptr).request_shutdown() };
        Ok(())
    }
}

/// Owns the inbound sample for a frame. A held view (the subscriber re-serving
/// its last sample on a tick with no new data) cannot detach from its
/// subscriber, so it is copied once per distinct held frame and the copy is
/// shared by every later tick that re-serves the same frame.
enum FrameBacking {
    Sample(RawInputView<'static>),
    Copied(Rc<[u8]>),
}

#[pyclass(unsendable)]
struct HostFrame {
    backing: FrameBacking,
    schema_hash: u64,
    released: Cell<bool>,
    exports: Cell<usize>,
}

impl HostFrame {
    fn bytes(&self) -> &[u8] {
        match &self.backing {
            FrameBacking::Sample(view) => view,
            FrameBacking::Copied(bytes) => bytes,
        }
    }
}

#[pymethods]
impl HostFrame {
    unsafe fn __getbuffer__(
        slf: Bound<'_, Self>,
        view: *mut ffi::Py_buffer,
        flags: std::ffi::c_int,
    ) -> PyResult<()> {
        let this = slf.borrow();
        if this.released.get() {
            return Err(PyValueError::new_err("input frame was released"));
        }
        let rc = unsafe {
            // SAFETY: `view` is supplied by CPython; the pointer and length
            // come from `bytes()`, backing owned by this HostFrame, which is
            // kept alive by the exporter until `__releasebuffer__`.
            ffi::PyBuffer_FillInfo(
                view,
                slf.as_ptr(),
                this.bytes().as_ptr() as *mut std::ffi::c_void,
                this.bytes().len() as ffi::Py_ssize_t,
                1,
                flags,
            )
        };
        if rc != 0 {
            return Err(PyErr::take(slf.py())
                .unwrap_or_else(|| PyBufferError::new_err("frame buffer export failed")));
        }
        this.exports.set(this.exports.get() + 1);
        Ok(())
    }

    unsafe fn __releasebuffer__(slf: Bound<'_, Self>, _view: *mut ffi::Py_buffer) {
        let this = slf.borrow();
        this.exports.set(this.exports.get().saturating_sub(1));
    }

    #[getter]
    fn schema_hash(&self) -> PyResult<u64> {
        if self.released.get() {
            return Err(PyValueError::new_err("input frame was released"));
        }
        Ok(self.schema_hash)
    }

    #[getter]
    fn released(&self) -> bool {
        self.released.get()
    }

    fn _check_alive(&self, py: Python<'_>) -> PyResult<()> {
        if !self.released.get() {
            return Ok(());
        }
        let exception = py
            .import("cerulion._native")?
            .getattr("ReleasedFrame")?
            .call1(("loan is closed",))?;
        Err(PyErr::from_value(exception))
    }
}

#[derive(Clone)]
struct LoanMeta {
    schema_name: String,
    schema_hash: u64,
    max_slice_len: Option<u32>,
    wire_fixed_size: Option<u32>,
}

struct NodeMetadata {
    input_names: Vec<String>,
    input_hashes: HashMap<String, u64>,
    outputs: HashMap<String, LoanMeta>,
}

#[pyclass(unsendable)]
struct HostLoan {
    loan: Option<RawShmLoan>,
    variable_entries: Option<Vec<(String, usize, usize)>>,
    exports: Cell<usize>,
    closed: Cell<bool>,
}

#[pymethods]
impl HostLoan {
    unsafe fn __getbuffer__(
        slf: Bound<'_, Self>,
        view: *mut ffi::Py_buffer,
        flags: std::ffi::c_int,
    ) -> PyResult<()> {
        let mut this = slf.borrow_mut();
        if this.closed.get() {
            return Err(PyValueError::new_err("loan already discarded"));
        }
        let Some(loan) = this.loan.as_mut() else {
            return Err(PyValueError::new_err("loan already discarded"));
        };
        let body = &mut loan.bytes_mut()[WireHeader::SIZE..];
        let rc = unsafe {
            // SAFETY: `view` is supplied by CPython; the loan remains retained
            // until the last export is released, even after discard marks it
            // closed.
            ffi::PyBuffer_FillInfo(
                view,
                slf.as_ptr(),
                body.as_mut_ptr().cast(),
                body.len() as ffi::Py_ssize_t,
                0,
                flags,
            )
        };
        if rc != 0 {
            return Err(PyErr::take(slf.py())
                .unwrap_or_else(|| PyBufferError::new_err("loan buffer export failed")));
        }
        this.exports.set(this.exports.get() + 1);
        Ok(())
    }

    unsafe fn __releasebuffer__(slf: Bound<'_, Self>, _view: *mut ffi::Py_buffer) {
        let mut this = slf.borrow_mut();
        let exports = this.exports.get().saturating_sub(1);
        this.exports.set(exports);
        if exports == 0 && this.closed.get() {
            this.loan = None;
        }
    }

    fn variable_entries(&self) -> Option<Vec<(String, usize, usize)>> {
        self.variable_entries.clone()
    }

    #[getter]
    fn exports(&self) -> usize {
        self.exports.get()
    }

    fn discard(&mut self) {
        self.closed.set(true);
        if self.exports.get() == 0 {
            self.loan = None;
        }
    }

    fn _check_alive(&self, py: Python<'_>) -> PyResult<()> {
        if !self.closed.get() && self.loan.is_some() {
            return Ok(());
        }
        let exception = py
            .import("cerulion._native")?
            .getattr("ReleasedFrame")?
            .call1(("loan is closed",))?;
        Err(PyErr::from_value(exception))
    }
}

struct HostTickShared {
    publishers: Cell<*mut indexmap::IndexMap<String, AnyPublisher>>,
    outputs: HashMap<String, LoanMeta>,
    schemas: SchemaSet,
    active: Cell<bool>,
    loans: RefCell<Vec<Py<HostLoan>>>,
}

#[pyclass(unsendable)]
struct HostTick {
    state: Rc<HostTickShared>,
}

#[pymethods]
impl HostTick {
    fn loan(&self, py: Python<'_>, name: &str, var_lens: Vec<usize>) -> PyResult<Py<HostLoan>> {
        let state = self.state.as_ref();
        if !state.active.get() {
            return Err(PyRuntimeError::new_err("tick is no longer active"));
        }
        let publishers_ptr = state.publishers.get();
        if publishers_ptr.is_null() {
            return Err(PyRuntimeError::new_err("tick is no longer active"));
        }
        let meta = state
            .outputs
            .get(name)
            .ok_or_else(|| PyValueError::new_err(format!("unknown output '{name}'")))?;
        // SAFETY: The active flag and null check above are set before the
        // Host releases its mutable publisher borrow. The unsendable pyclass
        // prevents concurrent Python access while this pointer is used.
        let publishers = unsafe { &mut *publishers_ptr };
        let publisher = publishers
            .get_mut(name)
            .ok_or_else(|| PyValueError::new_err(format!("unknown output '{name}'")))?;
        let layout = state.schemas.layout(&meta.schema_name).ok_or_else(|| {
            PyValueError::new_err(format!("schema {:?} is not available", meta.schema_name))
        })?;
        let encoder =
            FrameEncoder::new(layout).map_err(|error| PyValueError::new_err(error.to_string()))?;
        let total = encoder
            .required_len(&var_lens)
            .map_err(|error| PyValueError::new_err(error.to_string()))?;
        if let Some(max) = meta.max_slice_len {
            if total > max as usize {
                return Err(PyValueError::new_err(format!(
                    "output '{name}' frame of {total} bytes exceeds max_slice_len_default {max}"
                )));
            }
        }
        let loan = match publisher {
            AnyPublisher::Ipc(publisher) => publisher
                .loan_raw_uninit(total)
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?,
        };
        let mut loan = loan;
        for byte in loan.bytes_uninit_mut() {
            byte.write(0);
        }
        let mut loan = unsafe {
            // SAFETY: every byte of the exact-size raw loan was initialized.
            loan.assume_init()
        };
        let entries = {
            let bytes = loan.bytes_mut();
            encoder
                .begin(bytes, &var_lens, 0)
                .map_err(|error| PyValueError::new_err(error.to_string()))?;
            layout
                .variable_fields
                .iter()
                .enumerate()
                .map(|(index, field)| -> Result<_, PyErr> {
                    let offset = WireHeader::SIZE + layout.fixed_size + index * 8;
                    let payload = &bytes[WireHeader::SIZE..];
                    let start = offset - WireHeader::SIZE;
                    let entry = payload.get(start..start + 8).ok_or_else(|| {
                        PyValueError::new_err("encoder produced a truncated variable entry")
                    })?;
                    let off = u32::from_le_bytes(entry[..4].try_into().map_err(|_| {
                        PyValueError::new_err("encoder produced an invalid offset entry")
                    })?);
                    let len = u32::from_le_bytes(entry[4..].try_into().map_err(|_| {
                        PyValueError::new_err("encoder produced an invalid length entry")
                    })?);
                    Ok((field.name.clone(), off as usize, len as usize))
                })
                .collect::<PyResult<Vec<_>>>()?
        };
        let result = Py::new(
            py,
            HostLoan {
                loan: Some(loan),
                variable_entries: Some(entries),
                exports: Cell::new(0),
                closed: Cell::new(false),
            },
        )?;
        state.loans.borrow_mut().push(result.clone_ref(py));
        Ok(result)
    }
}

fn install_host_module(py: Python<'_>) -> PyResult<()> {
    let module = PyModule::new(py, "cerulion_pynode._host")?;
    module.add_class::<HostCtx>()?;
    module.add_class::<HostTick>()?;
    module.add_class::<HostFrame>()?;
    py.import("sys")?
        .getattr("modules")?
        .set_item("cerulion_pynode._host", module)?;
    Ok(())
}

#[cfg(all(not(Py_LIMITED_API), not(cerulion_pynode_limited)))]
unsafe extern "C" {
    fn PyConfig_InitIsolatedConfig(config: *mut ffi::PyConfig);
    fn Py_InitializeFromConfig(config: *const ffi::PyConfig) -> ffi::PyStatus;
    fn PyConfig_Clear(config: *mut ffi::PyConfig);
    fn PyStatus_Exception(status: ffi::PyStatus) -> std::ffi::c_int;
}

#[cfg(all(not(Py_LIMITED_API), not(cerulion_pynode_limited)))]
fn initialize_python() -> Result<(), String> {
    static LIB: Once = Once::new();
    static LIB_ERROR: OnceLock<Option<String>> = OnceLock::new();
    LIB.call_once(|| {
        let soname = env!("CERULION_LIBPYTHON_SONAME");
        let fallback = env!("CERULION_LIBPYTHON_FALLBACK");
        let result = match (CString::new(soname), CString::new(fallback)) {
            (Ok(soname), Ok(fallback)) => unsafe {
                // SAFETY: Loading the process's configured libpython globally is
                // required before PyO3 calls into the embedded interpreter.
                let first = libc::dlopen(soname.as_ptr(), libc::RTLD_NOW | libc::RTLD_GLOBAL);
                if first.is_null() {
                    libc::dlopen(fallback.as_ptr(), libc::RTLD_NOW | libc::RTLD_GLOBAL)
                } else {
                    first
                }
            },
            _ => std::ptr::null_mut(),
        };
        if result.is_null() {
            let _ = LIB_ERROR.set(Some(format!("unable to load {soname} or {fallback}")));
        } else {
            let _ = LIB_ERROR.set(None);
        }
    });
    if let Some(error) = LIB_ERROR.get().and_then(Clone::clone) {
        return Err(error);
    }
    // SAFETY: Py_IsInitialized is a process-global CPython query and requires
    // no borrowed pointers.
    if unsafe { pyo3::ffi::Py_IsInitialized() } == 0 {
        unsafe {
            let mut config = std::mem::zeroed();
            // SAFETY: config is a fresh PyConfig allocated on this stack.
            PyConfig_InitIsolatedConfig(&mut config);
            config.install_signal_handlers = 0;
            config.use_hash_seed = 1;
            config.hash_seed = 0;
            config.parse_argv = 0;
            config.site_import = 0;
            let status = Py_InitializeFromConfig(&config);
            PyConfig_Clear(&mut config);
            if PyStatus_Exception(status) != 0 {
                return Err("Py_InitializeFromConfig failed".to_string());
            }
            // Initialization leaves this thread holding the GIL; release it so
            // any scheduler thread can attach for later ticks.
            pyo3::ffi::PyEval_SaveThread();
        }
    }
    static INIT_LOG: Once = Once::new();
    Python::attach(|py| -> Result<(), String> {
        INIT_LOG.call_once(|| {
            if let Ok(sys) = py.import("sys") {
                if let Ok(prefix) = sys
                    .getattr("prefix")
                    .and_then(|value| value.extract::<String>())
                {
                    tracing::info!(prefix = %prefix, "embedded CPython initialised");
                }
            }
        });
        Ok(())
    })?;
    Ok(())
}

/// This cfg is emitted by `build.rs` when workspace-wide feature unification
/// causes PyO3 to target abi3. Standalone node workspaces use the full API.
#[cfg(any(Py_LIMITED_API, cerulion_pynode_limited))]
fn initialize_python() -> Result<(), String> {
    Err(
        "cerulion_pynode built against the limited API; build the node crate from its own workspace"
            .to_string(),
    )
}

pub struct Host {
    ctx: Box<NodeContext>,
    ctx_ptr: Rc<Cell<*mut NodeContext>>,
    runtime: Py<PyAny>,
    schemas: SchemaSet,
    input_names: Vec<String>,
    input_hashes: HashMap<String, u64>,
    outputs: HashMap<String, LoanMeta>,
    next_sequences: HashMap<String, u32>,
    held_copies: HashMap<String, Rc<[u8]>>,
    warned_threads: bool,
}

// SAFETY: Hosts are only entered while CPython's process-wide GIL is held.
// The ABI map may move a Host between callers, but no Host state is accessed
// concurrently; the embedded interpreter and NodeContext remain serialized by
// that GIL invariant.
unsafe impl Send for Host {}

impl Drop for Host {
    fn drop(&mut self) {
        self.ctx_ptr.set(std::ptr::null_mut());
    }
}

impl Host {
    fn metadata(info: &[u8], value: &Bound<'_, PyAny>) -> Result<NodeMetadata, String> {
        let info = info.strip_suffix(&[0]).unwrap_or(info);
        let document: serde_json::Value = serde_json::from_slice(info)
            .map_err(|error| format!("invalid node metadata: {error}"))?;
        let inputs = document
            .get("inputs")
            .and_then(serde_json::Value::as_array)
            .ok_or_else(|| "node metadata is missing inputs".to_string())?;
        let outputs = document
            .get("outputs")
            .and_then(serde_json::Value::as_array)
            .ok_or_else(|| "node metadata is missing outputs".to_string())?;
        let mut input_names = Vec::with_capacity(inputs.len());
        let mut input_hashes = HashMap::with_capacity(inputs.len());
        for input in inputs {
            let name = input
                .get("name")
                .and_then(serde_json::Value::as_str)
                .ok_or_else(|| "input metadata is missing name".to_string())?;
            let hash = input
                .get("schema_hash")
                .and_then(serde_json::Value::as_u64)
                .ok_or_else(|| format!("input metadata {name:?} is missing schema_hash"))?;
            input_names.push(name.to_string());
            input_hashes.insert(name.to_string(), hash);
        }
        let mut output_meta = HashMap::with_capacity(outputs.len());
        for output in outputs {
            let name = output
                .get("name")
                .and_then(serde_json::Value::as_str)
                .ok_or_else(|| "output metadata is missing name".to_string())?;
            let schema_hash = output
                .get("schema_hash")
                .and_then(serde_json::Value::as_u64)
                .ok_or_else(|| format!("output metadata {name:?} is missing schema_hash"))?;
            let max_slice_len = match output.get("max_slice_len_default") {
                Some(value) if !value.is_null() => {
                    let value = value.as_u64().ok_or_else(|| {
                        format!("output metadata {name:?} has invalid max_slice_len_default")
                    })?;
                    Some(u32::try_from(value).map_err(|_| {
                        format!("output metadata {name:?} has oversized max_slice_len_default")
                    })?)
                }
                _ => None,
            };
            let wire_fixed_size = match output.get("wire_fixed_size") {
                Some(value) if !value.is_null() => {
                    let value = value.as_u64().ok_or_else(|| {
                        format!("output metadata {name:?} has invalid wire_fixed_size")
                    })?;
                    Some(u32::try_from(value).map_err(|_| {
                        format!("output metadata {name:?} has oversized wire_fixed_size")
                    })?)
                }
                _ => None,
            };
            output_meta.insert(
                name.to_string(),
                LoanMeta {
                    schema_name: String::new(),
                    schema_hash,
                    max_slice_len,
                    wire_fixed_size,
                },
            );
        }
        let ports = value.getattr("__cerulion_ports__").map_err(python_error)?;
        let class_inputs = Self::class_port_names(&ports, "inputs")?;
        let class_outputs = Self::class_port_names(&ports, "outputs")?;
        let info_inputs = input_names.clone();
        let info_outputs = outputs
            .iter()
            .filter_map(|output| output.get("name").and_then(serde_json::Value::as_str))
            .map(str::to_owned)
            .collect::<Vec<_>>();
        if info_inputs != class_inputs || info_outputs != class_outputs {
            return Err(format!(
                "node metadata does not match the Python declaration: info inputs {:?} outputs {:?}, class inputs {:?} outputs {:?}",
                info_inputs, info_outputs, class_inputs, class_outputs
            ));
        }
        let output_ports = ports
            .get_item("outputs")
            .map_err(python_error)?
            .extract::<Vec<Bound<'_, PyAny>>>()
            .map_err(python_error)?;
        for port_name in class_outputs {
            let mut matching = None;
            for port in &output_ports {
                let name = port
                    .getattr("name")
                    .and_then(|name| name.extract::<String>())
                    .map_err(python_error)?;
                if name == port_name {
                    matching = Some(port.clone());
                    break;
                }
            }
            let port = matching
                .ok_or_else(|| format!("Python declaration is missing output {port_name:?}"))?;
            let schema_name = port
                .getattr("schema")
                .and_then(|name| name.extract::<String>())
                .map_err(python_error)?;
            if let Some(meta) = output_meta.get_mut(&port_name) {
                meta.schema_name = schema_name;
            }
        }
        Ok(NodeMetadata {
            input_names,
            input_hashes,
            outputs: output_meta,
        })
    }

    /// The compiled INFO is the declaration with each port's `schema` name
    /// removed; any other difference (policy, depth, timing, backpressure,
    /// sizes) means the scheduler would run a stale compiled shape.
    fn validate_declaration_matches_info(info: &[u8], declaration: &str) -> Result<(), String> {
        let info = info.strip_suffix(&[0]).unwrap_or(info);
        let compiled: serde_json::Value = serde_json::from_slice(info)
            .map_err(|error| format!("invalid node metadata: {error}"))?;
        let mut current: serde_json::Value = serde_json::from_str(declaration)
            .map_err(|error| format!("invalid Python declaration metadata: {error}"))?;
        for section in ["inputs", "outputs"] {
            if let Some(ports) = current
                .get_mut(section)
                .and_then(serde_json::Value::as_array_mut)
            {
                for port in ports
                    .iter_mut()
                    .filter_map(serde_json::Value::as_object_mut)
                {
                    port.remove("schema");
                }
            }
        }
        if compiled == current {
            return Ok(());
        }
        let empty = serde_json::Map::new();
        let compiled_keys = compiled.as_object().unwrap_or(&empty);
        let current_keys = current.as_object().unwrap_or(&empty);
        let key = compiled_keys
            .keys()
            .chain(current_keys.keys())
            .find(|key| compiled_keys.get(*key) != current_keys.get(*key))
            .map_or("<document>", String::as_str);
        let show = |value: Option<&serde_json::Value>| {
            value.map_or_else(|| "absent".to_string(), serde_json::Value::to_string)
        };
        Err(format!(
            "node metadata is stale: compiled {key} {}, declaration {}; run `cerulion node build`",
            show(compiled_keys.get(key)),
            show(current_keys.get(key)),
        ))
    }

    fn validate_declaration_metadata(
        metadata: &NodeMetadata,
        declaration: &str,
    ) -> Result<(), String> {
        let document: serde_json::Value = serde_json::from_str(declaration)
            .map_err(|error| format!("invalid Python declaration metadata: {error}"))?;
        let inputs = document
            .get("inputs")
            .and_then(serde_json::Value::as_array)
            .ok_or_else(|| "Python declaration metadata is missing inputs".to_string())?;
        let outputs = document
            .get("outputs")
            .and_then(serde_json::Value::as_array)
            .ok_or_else(|| "Python declaration metadata is missing outputs".to_string())?;

        for input in inputs {
            let name = input
                .get("name")
                .and_then(serde_json::Value::as_str)
                .ok_or_else(|| "Python declaration input is missing name".to_string())?;
            let declaration_hash = input
                .get("schema_hash")
                .and_then(serde_json::Value::as_u64)
                .ok_or_else(|| {
                    format!("Python declaration input {name:?} is missing schema_hash")
                })?;
            let info_hash = metadata
                .input_hashes
                .get(name)
                .ok_or_else(|| format!("Python declaration is missing input {name:?} in INFO"))?;
            if *info_hash != declaration_hash {
                return Err(format!(
                    "node metadata is stale for input '{name}': INFO schema_hash {info_hash}, declaration {declaration_hash}; run `cerulion node build`"
                ));
            }
        }

        for output in outputs {
            let name = output
                .get("name")
                .and_then(serde_json::Value::as_str)
                .ok_or_else(|| "Python declaration output is missing name".to_string())?;
            let meta = metadata
                .outputs
                .get(name)
                .ok_or_else(|| format!("Python declaration is missing output {name:?} in INFO"))?;
            let declaration_hash = output
                .get("schema_hash")
                .and_then(serde_json::Value::as_u64)
                .ok_or_else(|| {
                    format!("Python declaration output {name:?} is missing schema_hash")
                })?;
            if meta.schema_hash != declaration_hash {
                return Err(format!(
                    "node metadata is stale for output '{name}': INFO schema_hash {}, declaration {declaration_hash}; run `cerulion node build`",
                    meta.schema_hash
                ));
            }
            let declaration_max_slice = match output.get("max_slice_len_default") {
                Some(value) if !value.is_null() => {
                    let value = value.as_u64().ok_or_else(|| {
                        format!(
                            "Python declaration output {name:?} has invalid max_slice_len_default"
                        )
                    })?;
                    Some(u32::try_from(value).map_err(|_| {
                        format!(
                            "Python declaration output {name:?} has oversized max_slice_len_default"
                        )
                    })?)
                }
                _ => None,
            };
            if meta.max_slice_len != declaration_max_slice {
                return Err(format!(
                    "node metadata is stale for output '{name}': INFO max_slice_len_default {:?}, declaration {:?}; run `cerulion node build`",
                    meta.max_slice_len, declaration_max_slice
                ));
            }
            let declaration_wire_fixed = match output.get("wire_fixed_size") {
                Some(value) if !value.is_null() => {
                    let value = value.as_u64().ok_or_else(|| {
                        format!("Python declaration output {name:?} has invalid wire_fixed_size")
                    })?;
                    Some(u32::try_from(value).map_err(|_| {
                        format!("Python declaration output {name:?} has oversized wire_fixed_size")
                    })?)
                }
                _ => None,
            };
            if meta.wire_fixed_size != declaration_wire_fixed {
                return Err(format!(
                    "node metadata is stale for output '{name}': INFO wire_fixed_size {:?}, declaration {:?}; run `cerulion node build`",
                    meta.wire_fixed_size, declaration_wire_fixed
                ));
            }
        }
        Ok(())
    }

    fn class_port_names(ports: &Bound<'_, PyAny>, key: &str) -> Result<Vec<String>, String> {
        ports
            .get_item(key)
            .map_err(python_error)?
            .extract::<Vec<Bound<'_, PyAny>>>()
            .map_err(python_error)?
            .into_iter()
            .map(|port| {
                port.getattr("name")
                    .and_then(|name| name.extract::<String>())
                    .map_err(python_error)
            })
            .collect()
    }

    fn warn_extra_threads(py: Python<'_>, warned: &mut bool) {
        if *warned {
            return;
        }
        let count = py
            .import("threading")
            .and_then(|module| module.call_method0("active_count"))
            .and_then(|value| value.extract::<usize>());
        if let Ok(count) = count {
            if count <= 1 {
                return;
            }
            tracing::warn!(
                threads = count,
                "embedded Python node created additional threads"
            );
            *warned = true;
        }
    }

    pub fn init(
        mut ctx: Box<NodeContext>,
        module_name: &str,
        sys_path: &[&str],
        info: &[u8],
    ) -> Result<Self, String> {
        initialize_python()?;
        Python::attach(|py| -> Result<Self, String> {
            install_host_module(py).map_err(python_error)?;
            let path = py
                .import("sys")
                .map_err(python_error)?
                .getattr("path")
                .map_err(python_error)?;
            let prefixes = std::env::var("CERULION_PY_PATH")
                .ok()
                .into_iter()
                .flat_map(|value| value.split(':').map(str::to_owned).collect::<Vec<_>>())
                .chain(sys_path.iter().map(|value| (*value).to_owned()));
            for entry in prefixes.rev() {
                if !entry.is_empty() && path.call_method1("insert", (0, entry)).is_err() {
                    return Err("failed to update Python sys.path".to_string());
                }
            }
            py.import("sys")
                .and_then(|sys| sys.getattr("modules"))
                .and_then(|modules| modules.call_method1("pop", (module_name, py.None())))
                .map_err(python_error)?;
            let module = py.import(module_name).map_err(python_error)?;
            let mut classes = Vec::new();
            for (name, value) in module.dict().iter() {
                if value.hasattr("__cerulion_ports__").map_err(python_error)? {
                    classes.push((name, value));
                }
            }
            if classes.len() != 1 {
                return Err(format!(
                    "module {module_name:?} must contain exactly one class with __cerulion_ports__"
                ));
            }
            let metadata = Self::metadata(info, &classes[0].1)?;
            let workspace = node_workspace(&ctx, sys_path)?;
            let schemas = node_schemas(&workspace)?;
            let ctx_ptr = Rc::new(Cell::new((&mut *ctx) as *mut NodeContext));
            let host_ctx = Py::new(
                py,
                HostCtx {
                    ptr: ctx_ptr.clone(),
                },
            )
            .map_err(python_error)?;
            let runtime_cls = py
                .import("cerulion._node")
                .and_then(|m| m.getattr("_Runtime"))
                .map_err(python_error)?;
            let runtime = runtime_cls
                .call1((
                    classes[0].1.clone(),
                    host_ctx,
                    py.None(),
                    workspace.to_string_lossy().into_owned(),
                ))
                .map_err(python_error)?;
            let declaration = classes[0]
                .1
                .getattr("__cerulion_info__")
                .and_then(|info| info.call1((runtime.getattr("schemas")?,)))
                .and_then(|info| info.extract::<String>())
                .map_err(python_error)?;
            Self::validate_declaration_metadata(&metadata, &declaration)?;
            Self::validate_declaration_matches_info(info, &declaration)?;
            let runtime = runtime.unbind();
            let outputs = metadata.outputs;
            let mut host = Self {
                ctx,
                ctx_ptr,
                runtime,
                schemas,
                input_names: metadata.input_names,
                input_hashes: metadata.input_hashes,
                outputs,
                next_sequences: HashMap::new(),
                held_copies: HashMap::new(),
                warned_threads: false,
            };
            Self::warn_extra_threads(py, &mut host.warned_threads);
            Ok(host)
        })
    }

    /// Run one Python tick and commit its touched outputs.
    ///
    /// # Output commit contract
    ///
    /// Declaration, loan-shape, and frame-size validation happens before any
    /// output is published. Transport failures are not transactional: already
    /// committed outputs remain published, unsent loans are discarded, and
    /// the error reports the exact commit count. An output is committed when
    /// `send_raw_loan` sends it to SHM; a notification failure after send
    /// still counts that output as committed.
    pub fn tick(&mut self) -> Result<(), String> {
        let clock = self.ctx.clock().clone();
        let (publishers, subscribers) = self.ctx.split_publishers_subscribers_mut();
        let publishers_ptr = publishers as *mut _;
        let mut views = Vec::new();
        for (name, subscriber) in subscribers.iter_mut() {
            if let Some(&schema_hash) = self.input_hashes.get(name) {
                if let Some(view) = subscriber
                    .view_raw_expecting(schema_hash)
                    .map_err(|error| error.to_string())?
                {
                    let backing = match view.into_owned() {
                        Ok(view) => FrameBacking::Sample(view),
                        Err(held) => {
                            let cached = self
                                .held_copies
                                .get(name)
                                .filter(|copy| copy.as_ref() == &held[..]);
                            let copy = match cached {
                                Some(copy) => Rc::clone(copy),
                                None => {
                                    // hot-path-alloc-ok: one copy per distinct held frame.
                                    let copy: Rc<[u8]> = Rc::from(&held[..]);
                                    self.held_copies.insert(name.clone(), Rc::clone(&copy));
                                    copy
                                }
                            };
                            FrameBacking::Copied(copy)
                        }
                    };
                    views.push((name.clone(), backing));
                }
            }
        }
        Python::attach(|py| {
            let mut frames = Vec::with_capacity(self.input_names.len());
            let mut frame_objects = Vec::new();
            for name in &self.input_names {
                let Some(index) = views.iter().position(|(view_name, _)| view_name == name) else {
                    frames.push(None::<Py<HostFrame>>);
                    continue;
                };
                let (_, backing) = views.remove(index);
                let bytes = match &backing {
                    FrameBacking::Sample(view) => view.as_ref(),
                    FrameBacking::Copied(bytes) => bytes,
                };
                let schema_hash = WireHeader::read_from_buf(bytes)
                    .ok_or_else(|| "input frame has an invalid wire header".to_string())?
                    .schema_hash;
                let frame = Py::new(
                    py,
                    HostFrame {
                        backing,
                        schema_hash,
                        released: Cell::new(false),
                        exports: Cell::new(0),
                    },
                )
                .map_err(python_error)?;
                frames.push(Some(frame.clone_ref(py)));
                frame_objects.push((name.clone(), frame));
            }
            let state = Rc::new(HostTickShared {
                publishers: Cell::new(publishers_ptr),
                outputs: self.outputs.clone(),
                schemas: self.schemas.clone(),
                active: Cell::new(true),
                loans: RefCell::new(Vec::new()),
            });
            let tick = Py::new(
                py,
                HostTick {
                    state: state.clone(),
                },
            )
            .map_err(python_error)?;
            let call_result = (|| -> Result<Vec<(String, Py<HostLoan>)>, String> {
                self.runtime
                    .call_method1(py, "begin_tick", (tick.clone_ref(py), frames))
                    .map_err(python_error)?;
                self.runtime
                    .call_method0(py, "run_tick")
                    .map_err(python_error)?;
                self.runtime
                    .call_method0(py, "end_tick")
                    .and_then(|value| value.extract(py))
                    .map_err(python_error)
            })();
            state.active.set(false);
            state.publishers.set(std::ptr::null_mut());
            let (touched, tick_error) = match call_result {
                Ok(touched) => (touched, None),
                Err(error) => {
                    let mut message = error;
                    if let Err(cleanup_error) = self
                        .runtime
                        .call_method0(py, "abort_tick")
                        .map(|_| ())
                        .map_err(python_error)
                    {
                        message.push_str(&format!("\nfailed tick cleanup failed: {cleanup_error}"));
                    }
                    (Vec::new(), Some(message))
                }
            };
            let retained_input = frame_objects.iter().find_map(|(name, frame)| {
                let frame_ref = frame.bind(py).borrow();
                frame_ref.released.set(true);
                (frame_ref.exports.get() > 0).then(|| name.clone())
            });
            if tick_error.is_some() || retained_input.is_some() {
                for loan in state.loans.borrow().iter() {
                    loan.bind(py).borrow_mut().discard();
                }
                let had_tick_error = tick_error.is_some();
                let mut message = tick_error.unwrap_or_else(|| {
                    format!(
                        "retained view of input '{}' escaped tick(); NumPy/memoryview views of inputs are tick-scoped",
                        retained_input.as_deref().unwrap_or("<unknown>")
                    )
                });
                if had_tick_error && retained_input.is_some() {
                    message.push_str("\ninput view retained across a failed tick");
                }
                return Err(message);
            }
            let preflight = (|| -> Result<(), String> {
                let mut names = HashSet::new();
                for (name, loan_object) in &touched {
                    if !names.insert(name) {
                        return Err(format!("output '{name}' was touched more than once"));
                    }
                    let mut loan = loan_object.bind(py).borrow_mut();
                    if loan.exports.get() != 0 {
                        return Err(format!(
                            "output loan '{name}' retained a buffer export at tick end"
                        ));
                    }
                    let variable_entries = loan.variable_entries.clone();
                    let raw = loan
                        .loan
                        .as_mut()
                        .ok_or_else(|| format!("output loan '{name}' was already discarded"))?;
                    let body_len = raw.bytes_mut().len().saturating_sub(WireHeader::SIZE);
                    if let Some(entries) = variable_entries {
                        for (field, offset, length) in entries {
                            let end = offset.checked_add(length).ok_or_else(|| {
                                format!("output loan '{name}' variable entry '{field}' overflows")
                            })?;
                            if end > body_len {
                                return Err(format!(
                                    "output loan '{name}' variable entry '{field}' exceeds loan body"
                                ));
                            }
                        }
                    }
                    WireHeader::read_from_buf(raw.bytes_mut()).ok_or_else(|| {
                        format!("output loan '{name}' has an invalid wire header")
                    })?;
                    if !self.outputs.contains_key(name.as_str()) {
                        return Err(format!("unknown output '{name}'"));
                    }
                    if publishers.get(name.as_str()).is_none() {
                        return Err(format!("unknown output publisher '{name}'"));
                    }
                }
                Ok(())
            })();
            if let Err(error) = preflight {
                for remaining in state.loans.borrow().iter() {
                    remaining.bind(py).borrow_mut().discard();
                }
                return Err(error);
            }
            let total_outputs = touched.len();
            let mut committed = 0usize;
            for (name, loan_object) in touched {
                let result = (|| -> Result<(), String> {
                    let mut loan = loan_object.bind(py).borrow_mut();
                    let mut raw_loan = loan
                        .loan
                        .take()
                        .ok_or_else(|| format!("output loan '{name}' was already discarded"))?;
                    loan.closed.set(true);
                    let meta = self
                        .outputs
                        .get(&name)
                        .ok_or_else(|| format!("unknown output '{name}'"))?;
                    // SAFETY: the split context borrow keeps the publishers map
                    // alive for the duration of this tick.
                    let publisher = publishers
                        .get_mut(&name)
                        .ok_or_else(|| format!("unknown output publisher '{name}'"))?;
                    // A restored replay seeds the publisher; the first frame
                    // continues the recorded stream's numbering, as a Rust node's does.
                    let initial = match publisher {
                        AnyPublisher::Ipc(publisher) => publisher.initial_sequence(),
                    };
                    let sequence = self.next_sequences.entry(name.clone()).or_insert(initial);
                    let bytes = raw_loan.bytes_mut();
                    let mut header = WireHeader::read_from_buf(bytes).ok_or_else(|| {
                        format!("output loan '{name}' has an invalid wire header")
                    })?;
                    header.schema_hash = meta.schema_hash;
                    header.sequence = *sequence;
                    header.timestamp_ns = clock.now_ns();
                    header.write_to_buf(bytes);
                    match publisher {
                        AnyPublisher::Ipc(publisher) => {
                            publisher
                                .send_raw_loan(raw_loan)
                                .map_err(|error| error.to_string())?;
                            *sequence = sequence.wrapping_add(1);
                            committed += 1;
                            publisher.check_subscriber_events();
                            publisher
                                .notify_sent_sample()
                                .map_err(|error| error.to_string())?;
                        }
                    }
                    Ok(())
                })();
                if let Err(error) = result {
                    for remaining in state.loans.borrow().iter() {
                        remaining.bind(py).borrow_mut().discard();
                    }
                    return Err(format!(
                        "output '{name}' publish failed after {committed} of {total_outputs} outputs were committed: {error}"
                    ));
                }
            }
            Self::warn_extra_threads(py, &mut self.warned_threads);
            Ok(())
        })
    }

    pub fn pump_history(&mut self) -> Result<(), String> {
        self.ctx.pump_history();
        Ok(())
    }

    pub fn shutdown(&mut self) -> Result<(), String> {
        let result = Python::attach(|py| {
            self.runtime
                .call_method0(py, "shutdown")
                .map_err(python_error)?;
            Ok(())
        });
        self.ctx_ptr.set(std::ptr::null_mut());
        result
    }
}

#[macro_export]
macro_rules! export_node {
    (module: $module:literal, sys_path: [$($path:expr),* $(,)?], info: $info:ident) => {
        static NODES: ::std::sync::Mutex<
            ::std::option::Option<::std::collections::HashMap<u64, $crate::Host>>,
        > = ::std::sync::Mutex::new(::std::option::Option::None);
        static NEXT_HANDLE: ::std::sync::atomic::AtomicU64 =
            ::std::sync::atomic::AtomicU64::new(1);
        ::std::thread_local! {
            static LAST_ERROR: ::std::cell::RefCell<::std::option::Option<::std::ffi::CString>> =
                const { ::std::cell::RefCell::new(::std::option::Option::None) };
        }
        fn __set_error(message: ::std::string::String) {
            LAST_ERROR.with(|slot| {
                *slot.borrow_mut() = ::std::ffi::CString::new(message.replace('\0', "\\0")).ok();
            });
        }
        #[no_mangle]
        pub extern "C" fn cerulion_abi_version() -> u32 {
            $crate::cerulion_core::CERULION_ABI_VERSION
        }
        #[no_mangle]
        pub extern "C" fn cerulion_rustc_fingerprint() -> *const ::std::ffi::c_char {
            $crate::cerulion_core::rustc_fingerprint_cstr()
        }
        #[no_mangle]
        pub extern "C" fn cerulion_node_info() -> *const ::std::ffi::c_char {
            $info.as_ptr() as *const ::std::ffi::c_char
        }
        #[no_mangle]
        pub extern "C" fn cerulion_take_last_error() -> *mut ::std::ffi::c_char {
            LAST_ERROR.with(|slot| slot.borrow_mut().take().map_or(::std::ptr::null_mut(), |v| v.into_raw()))
        }
        #[no_mangle]
        pub unsafe extern "C" fn cerulion_free_error(ptr: *mut ::std::ffi::c_char) {
            if !ptr.is_null() {
                // SAFETY: pointer was returned by cerulion_take_last_error.
                let _ = ::std::ffi::CString::from_raw(ptr);
            }
        }
        #[no_mangle]
        pub extern "C" fn cerulion_node_init(ctx_ptr: *mut u8) -> u64 {
            let ptr = ctx_ptr.cast::<$crate::cerulion_core::graph::node::NodeContext>();
            let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                if ptr.is_null() {
                    __set_error("cerulion_node_init: NodeContext pointer was null".into());
                    return 0;
                }
                // SAFETY: ABI transfers ownership of the boxed context.
                let ctx = unsafe { ::std::boxed::Box::from_raw(ptr) };
                let rust_log = ctx.env_str("RUST_LOG", "");
                $crate::cerulion_core::graph::node::install_cdylib_stderr_tracing(
                    if rust_log.is_empty() { None } else { Some(rust_log.as_str()) },
                );
                let iox2_log = ctx.env_str("IOX2_LOG_LEVEL", "");
                $crate::cerulion_core::iceoryx_logger::init_iceoryx_log_level(
                    if iox2_log.is_empty() { None } else { Some(iox2_log.as_str()) },
                );
                match $crate::Host::init(ctx, $module, &[$($path),*], $info) {
                    Ok(host) => {
                        let handle = NEXT_HANDLE.fetch_add(1, ::std::sync::atomic::Ordering::Relaxed);
                        let mut nodes = match NODES.lock() {
                            Ok(nodes) => nodes,
                            Err(_) => {
                                __set_error("mutex poisoned".into());
                                return 0;
                            }
                        };
                        nodes.get_or_insert_with(::std::collections::HashMap::new).insert(handle, host);
                        handle
                    }
                    Err(error) => { __set_error(error); 0 }
                }
            }));
            result.unwrap_or_else(|_| { __set_error("cerulion_node_init: panic caught by catch_unwind".into()); 0 })
        }
        #[no_mangle]
        pub extern "C" fn cerulion_node_tick(handle: u64) -> i32 {
            let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                let mut nodes = match NODES.lock() { Ok(v) => v, Err(_) => { __set_error("mutex poisoned".into()); return 3; } };
                match nodes.as_mut().and_then(|v| v.get_mut(&handle)) {
                    Some(host) => match host.tick() { Ok(()) => 0, Err(error) => { __set_error(error); 1 } },
                    None => { __set_error(format!("handle {handle} not found")); 4 }
                }
            }));
            result.unwrap_or_else(|_| { __set_error("cerulion_node_tick: panic caught by catch_unwind".into()); 2 })
        }
        #[no_mangle]
        pub extern "C" fn cerulion_node_pump_history(handle: u64) -> i32 {
            let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                let mut nodes = match NODES.lock() { Ok(v) => v, Err(_) => { __set_error("mutex poisoned".into()); return 3; } };
                match nodes.as_mut().and_then(|v| v.get_mut(&handle)) {
                    Some(host) => match host.pump_history() { Ok(()) => 0, Err(error) => { __set_error(error); 1 } },
                    None => { __set_error(format!("handle {handle} not found")); 4 }
                }
            }));
            result.unwrap_or_else(|_| { __set_error("cerulion_node_pump_history: panic caught by catch_unwind".into()); 2 })
        }
        #[no_mangle]
        pub extern "C" fn cerulion_node_shutdown(handle: u64) -> i32 {
            let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                let mut nodes = match NODES.lock() { Ok(v) => v, Err(_) => { __set_error("mutex poisoned".into()); return 3; } };
                match nodes.as_mut().and_then(|v| v.remove(&handle)) {
                    Some(mut host) => match host.shutdown() { Ok(()) => 0, Err(error) => { __set_error(error); 1 } },
                    None => { __set_error(format!("handle {handle} not found")); 4 }
                }
            }));
            result.unwrap_or_else(|_| { __set_error("cerulion_node_shutdown: panic caught by catch_unwind".into()); 2 })
        }
    };
}

#[cfg(test)]
mod tests {
    use super::Host;

    const INFO: &[u8] =
        br#"{"inputs":[],"outputs":[{"name":"out","schema_hash":7}],"policy":{"period_ms":10}}"#;

    #[test]
    fn declaration_matching_info_apart_from_schema_names_is_accepted() {
        let declaration = r#"{"inputs":[],"outputs":[{"name":"out","schema":"Point","schema_hash":7}],"policy":{"period_ms":10}}"#;
        assert_eq!(
            Host::validate_declaration_matches_info(INFO, declaration),
            Ok(())
        );
    }

    #[test]
    fn nul_terminated_info_is_accepted() {
        let info = [INFO, b"\0"].concat();
        let declaration = r#"{"inputs":[],"outputs":[{"name":"out","schema":"Point","schema_hash":7}],"policy":{"period_ms":10}}"#;
        assert_eq!(
            Host::validate_declaration_matches_info(&info, declaration),
            Ok(())
        );
    }

    #[test]
    fn changed_policy_is_reported_as_stale_metadata() {
        let declaration = r#"{"inputs":[],"outputs":[{"name":"out","schema":"Point","schema_hash":7}],"policy":{"period_ms":20}}"#;
        assert_eq!(
            Host::validate_declaration_matches_info(INFO, declaration),
            Err("node metadata is stale: compiled policy {\"period_ms\":10}, declaration {\"period_ms\":20}; run `cerulion node build`".to_string())
        );
    }

    #[test]
    fn key_missing_from_declaration_is_reported_as_absent() {
        let declaration =
            r#"{"inputs":[],"outputs":[{"name":"out","schema":"Point","schema_hash":7}]}"#;
        assert_eq!(
            Host::validate_declaration_matches_info(INFO, declaration),
            Err("node metadata is stale: compiled policy {\"period_ms\":10}, declaration absent; run `cerulion node build`".to_string())
        );
    }

    #[test]
    fn invalid_info_json_is_an_error() {
        assert!(Host::validate_declaration_matches_info(b"{", "{}")
            .unwrap_err()
            .starts_with("invalid node metadata"));
    }
}
