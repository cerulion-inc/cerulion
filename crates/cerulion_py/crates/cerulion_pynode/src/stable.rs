//! Python nodes behind the same checked stable boundary as generated Rust nodes.
use crate::{backend::Context, Host};
use cerulion_core::codegen::layout::WireLayout;
use cerulion_core::error::{TransportError, TransportResult};
use cerulion_core::graph::node::DylibNodeEntry;
use cerulion_core::graph::node_plugin_abi::{HostClient, NodeContext, PluginNode};
use cerulion_core::node_abi::{ExternalSourceV1, LoanRequestV1};
use cerulion_core::state::StateSink;
use std::marker::PhantomData;
use std::sync::Mutex;

#[doc(hidden)]
pub trait Definition: Send {
    fn module() -> &'static str;
    fn paths() -> &'static [&'static str];
    fn info() -> &'static [u8];
    fn instances() -> &'static Mutex<usize>;
}
#[doc(hidden)]
pub struct PythonNode<D: Definition> {
    host: Host,
    client: HostClient,
    _definition: PhantomData<D>,
}
fn node_error(reason: String) -> TransportError {
    TransportError::NodeError {
        node_id: "Python node".into(),
        reason,
    }
}
impl<D: Definition> PluginNode for PythonNode<D> {
    fn create(client: HostClient, params: &[u8]) -> TransportResult<Self> {
        if !params.is_empty() {
            let values: serde_json::Value =
                serde_json::from_slice(params).map_err(|e| node_error(e.to_string()))?;
            if !values.is_object() {
                return Err(node_error(
                    "Python node params must be a JSON object".into(),
                ));
            }
        }
        let text = std::str::from_utf8(D::info().strip_suffix(&[0]).unwrap_or(D::info()))
            .map_err(|e| node_error(e.to_string()))?;
        let info =
            DylibNodeEntry::parse_info_json_labeled(text, "Python node").map_err(node_error)?;
        let context = NodeContext::new(HostClient::share(&client), &info)?;
        let rust_log = context.env_str("RUST_LOG", "");
        cerulion_core::graph::node::install_cdylib_stderr_tracing(if rust_log.is_empty() {
            None
        } else {
            Some(&rust_log)
        });
        let iox_log = context.env_str("IOX2_LOG_LEVEL", "");
        cerulion_core::iceoryx_logger::init_iceoryx_log_level(if iox_log.is_empty() {
            None
        } else {
            Some(&iox_log)
        });
        // Recovering a poisoned counter is safe: only successful init changes
        // it, and every failed/panicking initialization leaves it unchanged.
        let mut count = D::instances().lock().unwrap_or_else(|e| e.into_inner());
        let next_count = count
            .checked_add(1)
            .ok_or_else(|| node_error("Python instance count exhausted".into()))?;
        let host = Host::init_backend(
            Box::new(Context::Plugin(context, HostClient::share(&client))),
            D::module(),
            D::paths(),
            D::info(),
            *count == 0,
            Some(params),
        )
        .map_err(node_error)?;
        *count = next_count;
        Ok(Self {
            host,
            client,
            _definition: PhantomData,
        })
    }
    fn tick(&mut self) -> TransportResult<()> {
        let result = self.host.tick().map_err(node_error);
        self.client.take_failure().map_or(result, Err)
    }
    fn shutdown(&mut self) -> TransportResult<()> {
        let result = self.host.shutdown().map_err(node_error);
        self.client.take_failure().map_or(result, Err)
    }
    fn state_shape() -> u64 {
        0
    }
    fn capture(&self, _: &mut dyn StateSink) -> TransportResult<()> {
        Err(node_error(
            "Python nodes restart from init; state capture is unavailable".into(),
        ))
    }
    fn restore(&mut self, _: &[u8]) -> TransportResult<()> {
        Err(node_error(
            "Python nodes restart from init; state restore is unavailable".into(),
        ))
    }
    fn inline_safe(&self) -> bool {
        false
    }
    fn probe(&self) -> bool {
        false
    }
    fn external_source(&mut self) -> TransportResult<ExternalSourceV1> {
        Err(node_error(
            "Python node does not declare an external source".into(),
        ))
    }
}
impl<D: Definition> Drop for PythonNode<D> {
    fn drop(&mut self) {
        let mut count = D::instances().lock().unwrap_or_else(|e| e.into_inner());
        *count = count.saturating_sub(1);
    }
}

pub(crate) fn loan_request(layout: &WireLayout, hash: u64) -> Result<LoanRequestV1, String> {
    let fixed = u32::try_from(layout.fixed_size).map_err(|_| "wire fixed size exceeds u32")?;
    let count =
        u32::try_from(layout.variable_fields.len()).map_err(|_| "variable count exceeds u32")?;
    let minimum = fixed
        .checked_add(count.checked_mul(8).ok_or("variable table exceeds u32")?)
        .and_then(|n| n.checked_add(40))
        .ok_or("wire prefix exceeds u32")?;
    if layout.schema_hash != hash {
        return Err("dynamic layout schema hash differs from declared output".into());
    }
    Ok(LoanRequestV1 {
        schema_hash: hash,
        minimum_capacity: minimum,
        wire_fixed_size: fixed,
        variable_count: count,
        alignment: cerulion_core::shm_runtime::FRAME_ALIGN as u32,
    })
}

/// Cold wire facts are resolved by the same canonical layout hash as init.
#[doc(hidden)]
pub fn metadata<D: Definition>() -> Result<std::ffi::CString, String> {
    let workspace = std::env::var_os("CERULION_WORKSPACE")
        .map(std::path::PathBuf::from)
        .or_else(|| D::paths().first().map(std::path::PathBuf::from))
        .unwrap_or(std::env::current_dir().map_err(|e| e.to_string())?);
    let schemas = crate::node_schemas(&workspace)?;
    let bytes = D::info().strip_suffix(&[0]).unwrap_or(D::info());
    let mut info: serde_json::Value = serde_json::from_slice(bytes).map_err(|e| e.to_string())?;
    info.as_object_mut()
        .ok_or("metadata is not an object")?
        .insert("retain_process".into(), true.into());
    for section in ["inputs", "outputs"] {
        let ports = info
            .get_mut(section)
            .and_then(serde_json::Value::as_array_mut)
            .ok_or_else(|| format!("metadata has no {section}"))?;
        for port in ports {
            let hash = port
                .get("schema_hash")
                .and_then(serde_json::Value::as_u64)
                .ok_or("metadata port has no schema hash")?;
            let layout = schemas
                .layout_for_hash(hash)
                .ok_or_else(|| format!("metadata schema hash {hash} is unavailable"))?;
            let facts = loan_request(layout, hash)?;
            let port = port
                .as_object_mut()
                .ok_or("metadata port is not an object")?;
            port.insert("wire_fixed_size".into(), facts.wire_fixed_size.into());
            port.insert("variable_count".into(), facts.variable_count.into());
            port.insert("alignment".into(), facts.alignment.into());
        }
    }
    std::ffi::CString::new(serde_json::to_vec(&info).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())
}

#[doc(hidden)]
#[macro_export]
macro_rules! export_stable_node {
    (module: $module:literal, sys_path: [$($path:expr),* $(,)?], info: $info:ident) => {
        struct __CerPythonDefinition;
        impl $crate::stable::Definition for __CerPythonDefinition {
            fn module() -> &'static str { $module }
            fn paths() -> &'static [&'static str] { &[$($path),*] }
            fn info() -> &'static [u8] { $info }
            fn instances() -> &'static ::std::sync::Mutex<usize> {
                static COUNT: ::std::sync::Mutex<usize> = ::std::sync::Mutex::new(0); &COUNT
            }
        }
        fn __cer_python_nodes() -> &'static $crate::cerulion_core::graph::node_plugin_abi::NodeExports<$crate::stable::PythonNode<__CerPythonDefinition>> {
            static NODES: ::std::sync::OnceLock<$crate::cerulion_core::graph::node_plugin_abi::NodeExports<$crate::stable::PythonNode<__CerPythonDefinition>>> = ::std::sync::OnceLock::new();
            NODES.get_or_init(Default::default)
        }
        unsafe extern "C" fn __cer_python_create(host: *const $crate::cerulion_core::node_abi::HostApiV1, context: $crate::cerulion_core::node_abi::ContextHandle, params: $crate::cerulion_core::node_abi::Bytes, output: *mut $crate::cerulion_core::node_abi::NodeHandle) -> $crate::cerulion_core::node_abi::Status {
            // SAFETY (U06/U12): immutable host table and synchronous borrowed
            // caller storage; the shared export adapter checks all descriptors.
            unsafe { __cer_python_nodes().create(host, context, params, output, 0) }
        }
        unsafe extern "C" fn __cer_python_tick(handle: $crate::cerulion_core::node_abi::NodeHandle) -> $crate::cerulion_core::node_abi::Status { __cer_python_nodes().tick(handle) }
        unsafe extern "C" fn __cer_python_shutdown(handle: $crate::cerulion_core::node_abi::NodeHandle) -> $crate::cerulion_core::node_abi::Status { __cer_python_nodes().shutdown(handle) }
        unsafe extern "C" fn __cer_python_destroy(handle: $crate::cerulion_core::node_abi::NodeHandle) -> $crate::cerulion_core::node_abi::Status { __cer_python_nodes().destroy(handle) }
        unsafe extern "C" fn __cer_python_error(token: u64, buffer: $crate::cerulion_core::node_abi::MutableBytes, required: *mut u64) -> $crate::cerulion_core::node_abi::Status {
            // SAFETY (U06/U12): shared adapter validates bounds and copies
            // owner-local diagnostics only during this synchronous call.
            unsafe { __cer_python_nodes().copy_error(token, buffer, required) }
        }
        unsafe extern "C" fn __cer_python_extension(_: u64, _: u32, _: *mut $crate::cerulion_core::node_abi::ExtensionDescriptorV1) -> $crate::cerulion_core::node_abi::Status {
            $crate::cerulion_core::node_abi::Status { code: $crate::cerulion_core::node_abi::status_code::UNSUPPORTED, ..Default::default() }
        }
        fn __cer_python_api() -> *const $crate::cerulion_core::node_abi::NodeApiV1 {
            static API: ::std::sync::OnceLock<&'static $crate::cerulion_core::node_abi::NodeApiV1> = ::std::sync::OnceLock::new();
            static METADATA: ::std::sync::OnceLock<::std::ffi::CString> = ::std::sync::OnceLock::new();
            match ::std::panic::catch_unwind(|| {
                *API.get_or_init(|| {
                    let metadata = METADATA.get_or_init(|| $crate::stable::metadata::<__CerPythonDefinition>().expect("Python metadata schemas must be available"));
                    // hot-path-alloc-ok: one immutable table per loaded Python library.
                    ::std::boxed::Box::leak(::std::boxed::Box::new($crate::cerulion_core::node_abi::NodeApiV1 {
                        header: $crate::cerulion_core::node_abi::TableHeaderV1::for_type::<$crate::cerulion_core::node_abi::NodeApiV1>(),
                        metadata: $crate::cerulion_core::node_abi::Bytes { ptr: metadata.as_bytes().as_ptr(), len: metadata.as_bytes().len() as u64 },
                        required_host_capabilities: 0, capabilities: 0,
                        create: __cer_python_create, tick: __cer_python_tick, shutdown: __cer_python_shutdown, destroy: __cer_python_destroy,
                        copy_error: __cer_python_error, query_extension: __cer_python_extension, state: ::std::ptr::null(), external: ::std::ptr::null(),
                    }))
                }) as *const _
            }) { Ok(api) => api, Err(_) => ::std::ptr::null() }
        }
        $crate::cerulion_core::node_abi::export_node_api!(__cer_python_api);
    };
}
