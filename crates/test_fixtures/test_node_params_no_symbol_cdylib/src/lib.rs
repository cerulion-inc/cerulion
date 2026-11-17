// SPDX-License-Identifier: AGPL-3.0-only
//! Stable metadata and parameter fixture. Legacy diagnostic metadata may vary;
//! the checked table retains the immutable document used by inspection.
// A node is a library loaded by the runtime. Use tracing for diagnostics.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::graph::node_plugin_abi::{HostClient, NodeContext, NodeExports, PluginNode};
use cerulion_core::node_abi as abi;

struct MetadataFixture {
    context: Option<NodeContext>,
    tick_count: u64,
}

impl PluginNode for MetadataFixture {
    fn create(client: HostClient, params: &[u8]) -> cerulion_core::TransportResult<Self> {
        let info = cerulion_core::graph::node::DylibNodeEntry::parse_info_json_labeled(
            std::str::from_utf8(&INFO_BYTES[..INFO_BYTES.len() - 1]).map_err(|error| {
                cerulion_core::TransportError::GraphError {
                    reason: error.to_string(),
                }
            })?,
            "raw node",
        )
        .map_err(|reason| cerulion_core::TransportError::GraphError { reason })?;
        let values = cerulion_core::graph::params::params_from_json(if params.is_empty() {
            "{}"
        } else {
            std::str::from_utf8(params).map_err(|error| {
                cerulion_core::TransportError::NodeError {
                    node_id: "metadata fixture".into(),
                    reason: error.to_string(),
                }
            })?
        })
        .map_err(|reason| cerulion_core::TransportError::NodeError {
            node_id: "metadata fixture".into(),
            reason,
        })?;
        cerulion_core::graph::params::resolve_params("metadata fixture", info.params(), &values)
            .map_err(|reason| cerulion_core::TransportError::NodeError {
                node_id: "metadata fixture".into(),
                reason: reason.to_string(),
            })?;
        let context = NodeContext::new(client.clone(), &info)?;
        let rust_log = context.env_opt("RUST_LOG");
        cerulion_core::graph::node::install_cdylib_stderr_tracing(rust_log.as_deref());
        if let Some(error) = client.take_failure() {
            return Err(error);
        }
        Ok(Self {
            context: Some(context),
            tick_count: 0,
        })
    }
    fn tick(&mut self) -> cerulion_core::TransportResult<()> {
        // Replace this body with application logic over the local context.
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

static NODES: std::sync::OnceLock<NodeExports<MetadataFixture>> = std::sync::OnceLock::new();
fn nodes() -> &'static NodeExports<MetadataFixture> {
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
unsafe extern "C" fn external_source(
    handle: abi::NodeHandle,
    output: *mut abi::ExternalSourceV1,
) -> abi::Status {
    // SAFETY: caller owns the aligned output scalar through this synchronous
    // query; NodeExports checks it before invoking the local node.
    unsafe { nodes().external(handle, output) }
}
static EXTERNAL: abi::ExternalApiV1 = abi::ExternalApiV1 {
    header: abi::TableHeaderV1::for_type::<abi::ExternalApiV1>(),
    describe: external_source,
};
static API: std::sync::OnceLock<&'static abi::NodeApiV1> = std::sync::OnceLock::new();
static METADATA: std::sync::OnceLock<std::ffi::CString> = std::sync::OnceLock::new();
fn node_api() -> *const abi::NodeApiV1 {
    *API.get_or_init(|| {
        let metadata = METADATA
            .get_or_init(|| {
                let selected = if std::env::var_os("CER_INFO_MALFORMED").is_some() {
                    MALFORMED_INFO
                } else if std::env::var_os("CER_INFO_UNKNOWN_KEY").is_some() {
                    UNKNOWN_INFO
                } else if std::env::var_os("CER_INFO_FIRST_READ_PLAIN").is_some() {
                    PLAIN_INFO
                } else {
                    INFO_BYTES
                };
                std::ffi::CString::new(&selected[..selected.len() - 1]).expect("static metadata")
            })
            .as_bytes();
        Box::leak(Box::new(abi::NodeApiV1 {
            header: abi::TableHeaderV1::for_type::<abi::NodeApiV1>(),
            metadata: abi::Bytes {
                ptr: metadata.as_ptr(),
                len: metadata.len() as u64,
            },
            required_host_capabilities: 0,
            capabilities: abi::capability::EXTERNAL,
            create,
            tick,
            shutdown,
            destroy,
            copy_error,
            query_extension,
            state: std::ptr::null(),
            external: &EXTERNAL,
        }))
    }) as *const _
}
cerulion_core::node_abi::export_node_api!(node_api);
static INFO_BYTES: &[u8] = b"{\"inputs\":[],\"outputs\":[],\"params\":[{\"name\":\"gain\",\"type\":\"f64\",\"default\":1.0,\"min\":0.0,\"max\":10.0}]}\0";
static PLAIN_INFO: &[u8] = b"{\"inputs\":[],\"outputs\":[]}\0";
static UNKNOWN_INFO: &[u8] = b"{\"inputs\":[],\"outputs\":[],\"polciy\":\"x\"}\0";
static MALFORMED_INFO: &[u8] = b"{not json\0";
static INFO_CALLS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
#[no_mangle]
pub extern "C" fn cerulion_node_info() -> *const std::ffi::c_char {
    let first = INFO_CALLS.fetch_add(1, std::sync::atomic::Ordering::Relaxed) == 0;
    let bytes = if first { PLAIN_INFO } else { INFO_BYTES };
    bytes.as_ptr().cast()
}
