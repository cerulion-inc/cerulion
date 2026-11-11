// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-port stable fixture for lifecycle, rejection and logging tests.
// A node is a library loaded by the runtime. Use tracing for diagnostics.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::graph::node_plugin_abi::{HostClient, NodeContext, NodeExports, PluginNode};
use cerulion_core::node_abi as abi;

struct TestNodeState {
    context: Option<NodeContext>,
    tick_count: u64,
    tick_fail: bool,
}

impl PluginNode for TestNodeState {
    fn create(client: HostClient, _params: &[u8]) -> cerulion_core::TransportResult<Self> {
        let info = cerulion_core::graph::node::DylibNodeEntry::parse_info_json_labeled(
            std::str::from_utf8(&INFO_BYTES[..INFO_BYTES.len() - 1]).map_err(|error| {
                cerulion_core::TransportError::GraphError {
                    reason: error.to_string(),
                }
            })?,
            "raw node",
        )
        .map_err(|reason| cerulion_core::TransportError::GraphError { reason })?;
        let context = NodeContext::new(client.clone(), &info)?;
        let rust_log = context.env_opt("RUST_LOG");
        cerulion_core::graph::node::install_cdylib_stderr_tracing(rust_log.as_deref());
        let iox_log = context.env_str("IOX2_LOG_LEVEL", "");
        cerulion_core::iceoryx_logger::init_iceoryx_log_level(if iox_log.is_empty() {
            None
        } else {
            Some(&iox_log)
        });
        // Machine-readable probe reports this library's actual linked log level.
        #[allow(clippy::print_stderr)]
        if std::env::var_os("CER_CDYLIB_IOX2_PROBE").is_some() {
            eprintln!(
                "RAWFFI_IOX2_LEVEL={}",
                cerulion_core::iceoryx_logger::current_iox2_log_level()
            );
        }
        if let Some(error) = client.take_failure() {
            return Err(error);
        }
        Ok(Self {
            context: Some(context),
            tick_count: 0,
            tick_fail: std::env::var_os("CER_TICK_FAIL").is_some(),
        })
    }
    fn tick(&mut self) -> cerulion_core::TransportResult<()> {
        if self.tick_fail {
            return Err(cerulion_core::TransportError::NodeError {
                node_id: "raw fixture".into(),
                reason: "untyped fixture tick failure".into(),
            });
        }
        self.tick_count = self.tick_count.wrapping_add(1);
        Ok(())
    }
    fn shutdown(&mut self) -> cerulion_core::TransportResult<()> {
        self.context = None;
        Ok(())
    }
    fn state_shape() -> u64 {
        0
    }
    fn capture(
        &self,
        _sink: &mut dyn cerulion_core::state::StateSink,
    ) -> cerulion_core::TransportResult<()> {
        Ok(())
    }
    fn restore(&mut self, payload: &[u8]) -> cerulion_core::TransportResult<()> {
        if payload.is_empty() {
            Ok(())
        } else {
            Err(cerulion_core::TransportError::GraphError {
                reason: "raw template declares no restorable state".into(),
            })
        }
    }
    fn inline_safe(&self) -> bool {
        true
    }
    fn probe(&self) -> bool {
        true
    }
    fn external_source(&mut self) -> cerulion_core::TransportResult<abi::ExternalSourceV1> {
        Ok(abi::ExternalSourceV1 {
            fd: -1,
            kind: abi::external::HOST_DRIVEN,
            flags: 0,
        })
    }
}

static NODES: std::sync::OnceLock<NodeExports<TestNodeState>> = std::sync::OnceLock::new();
fn nodes() -> &'static NodeExports<TestNodeState> {
    NODES.get_or_init(Default::default)
}

unsafe extern "C" fn create(
    host: *const abi::HostApiV1,
    context: abi::ContextHandle,
    params: abi::Bytes,
    output: *mut abi::NodeHandle,
) -> abi::Status {
    // SAFETY: the host lends its immutable checked table, parameters and exclusive
    // output for this call. NodeExports validates them before any node effect.
    unsafe { nodes().create(host, context, params, output, 0) }
}
unsafe extern "C" fn tick(handle: abi::NodeHandle) -> abi::Status {
    nodes().tick(handle)
}
unsafe extern "C" fn shutdown(handle: abi::NodeHandle) -> abi::Status {
    nodes().shutdown(handle)
}
unsafe extern "C" fn destroy(handle: abi::NodeHandle) -> abi::Status {
    nodes().destroy(handle)
}
unsafe extern "C" fn copy_error(
    token: u64,
    target: abi::MutableBytes,
    required: *mut u64,
) -> abi::Status {
    // SAFETY: caller owns bounded writable storage and the output scalar for this
    // synchronous call. NodeExports checks bounds and copies node-owned text.
    unsafe { nodes().copy_error(token, target, required) }
}
unsafe extern "C" fn query_extension(
    _id: u64,
    _version: u32,
    _output: *mut abi::ExtensionDescriptorV1,
) -> abi::Status {
    abi::Status {
        code: abi::status_code::UNSUPPORTED,
        ..Default::default()
    }
}
unsafe extern "C" fn state_shape(output: *mut u64) -> abi::Status {
    // SAFETY: exclusive host scalar is borrowed synchronously; NodeExports
    // checks alignment before writing the node's declared shape.
    unsafe { nodes().shape(output) }
}
unsafe extern "C" fn capture_state(
    handle: abi::NodeHandle,
    sink: *const abi::StateSinkV1,
    capacity: u64,
) -> abi::Status {
    // SAFETY: the checked host sink and its context remain live only for this
    // synchronous capture, including every borrowed encoder byte span.
    unsafe { nodes().capture(handle, sink, capacity) }
}
unsafe extern "C" fn restore_state(handle: abi::NodeHandle, payload: abi::Bytes) -> abi::Status {
    // SAFETY: initialized host bytes stay borrowed for this call; NodeExports
    // validates the span before the node-local decoder reads it.
    unsafe { nodes().restore(handle, payload) }
}
unsafe extern "C" fn inline_safe(handle: abi::NodeHandle, output: *mut u32) -> abi::Status {
    // SAFETY: caller owns the aligned exclusive result slot through this query.
    unsafe { nodes().inline_safe(handle, output) }
}
unsafe extern "C" fn probe(handle: abi::NodeHandle) -> abi::Status {
    nodes().probe(handle)
}
static STATE: abi::StateApiV1 = abi::StateApiV1 {
    header: abi::TableHeaderV1::for_type::<abi::StateApiV1>(),
    shape: state_shape,
    capture: capture_state,
    restore: restore_state,
    inline_safe,
    probe,
};
static API: std::sync::OnceLock<&'static abi::NodeApiV1> = std::sync::OnceLock::new();
static FAULT_API: std::sync::OnceLock<&'static abi::NodeApiV1> = std::sync::OnceLock::new();
static METADATA: std::sync::OnceLock<std::ffi::CString> = std::sync::OnceLock::new();
fn node_api() -> *const abi::NodeApiV1 {
    let fault = std::env::var_os("CER_ABI_FAULT").is_some();
    let slot = if fault { &FAULT_API } else { &API };
    *slot.get_or_init(|| {
        let metadata = METADATA.get_or_init(__cer_raw_wire_metadata).as_bytes();
        Box::leak(Box::new(abi::NodeApiV1 {
            header: {
                let mut header = abi::TableHeaderV1::for_type::<abi::NodeApiV1>();
                if fault {
                    header.boundary_major = 9999;
                }
                header
            },
            metadata: abi::Bytes {
                ptr: metadata.as_ptr(),
                len: metadata.len() as u64,
            },
            required_host_capabilities: abi::capability::STATE,
            capabilities: abi::capability::STATE,
            create,
            tick,
            shutdown,
            destroy,
            copy_error,
            query_extension,
            state: &STATE,
            external: std::ptr::null(),
        }))
    }) as *const _
}
cerulion_core::node_abi::export_node_api!(node_api);

/// Diagnostic provenance is optional and is never a stable-load requirement.
#[no_mangle]
pub extern "C" fn cerulion_rustc_fingerprint() -> *const std::ffi::c_char {
    match std::env::var("CER_RUSTC_FAULT").as_deref() {
        Ok("null") => std::ptr::null(),
        Ok("1") => c"diagnostic-fault".as_ptr(),
        _ => cerulion_core::rustc_fingerprint_cstr(),
    }
}
// CERULION:INFO_START
static INFO_BYTES: &[u8] = b"{\"inputs\":[],\"outputs\":[]}\0";

#[no_mangle]
pub extern "C" fn cerulion_node_info() -> *const std::ffi::c_char {
    // SAFETY: INFO_BYTES is a static &[u8] with a trailing null byte.
    // Casting *const u8 to *const c_char is safe: identical layout, valid and 'static.
    INFO_BYTES.as_ptr() as *const std::ffi::c_char
}

fn __cer_raw_wire_metadata() -> std::ffi::CString {
    let base = std::ffi::CStr::from_bytes_with_nul(INFO_BYTES)
        .expect("static metadata has one trailing NUL");
    cerulion_core::graph::node_plugin_abi::raw_metadata(base, &[], &[])
        .expect("raw schema facts match declared ports")
}
// CERULION:INFO_END
