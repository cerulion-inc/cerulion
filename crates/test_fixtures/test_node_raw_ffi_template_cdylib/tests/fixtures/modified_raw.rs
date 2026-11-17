// SPDX-License-Identifier: AGPL-3.0-only
// Compiled delivery fixture. Its INFO block is pinned to node create followed
// by two node modify calls; tick publishes hand-written vectors on all ports.
// A node is a library loaded by the runtime. Use tracing for diagnostics.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::graph::node_plugin_abi::{HostClient, NodeContext, NodeExports, PluginNode};
use cerulion_core::node_abi as abi;

struct RawFfiTemplateState {
    context: Option<NodeContext>,
    client: HostClient,
    tick_count: u64,
}

impl PluginNode for RawFfiTemplateState {
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
        cerulion_core::tracing::warn!(
            "raw node template contains no application logic; implement tick before running"
        );
        if let Some(error) = client.take_failure() {
            return Err(error);
        }
        Ok(Self {
            context: Some(context),
            client,
            tick_count: 0,
        })
    }
    fn flush_deferred(&self) -> cerulion_core::TransportResult<()> {
        self.client.flush_deferred()
    }
    fn tick(&mut self) -> cerulion_core::TransportResult<()> {
        use cerulion_core::{message::ShmMessage, wire::WireHeader};
        use native_ros2_messages::geometry_msgs::Vector3;
        let context = self.context.as_mut().expect("initialized raw fixture");
        let timestamp = context.clock().now_ns();
        for (name, values) in [
            ("original", [1.0_f64, 2.0, 3.0]),
            ("first", [4.0_f64, 5.0, 6.0]),
            ("second", [7.0_f64, 8.0, 9.0]),
        ] {
            let publisher = context.publisher_mut(name).expect("bound typed output");
            let mut frame = publisher.loan_raw_frame(HostClient::request::<Vector3>(), 64)?;
            let bytes = frame.bytes_mut();
            let mut header = WireHeader::new(Vector3::SCHEMA_HASH, 0, timestamp);
            header.total_size = 64;
            header.offset_table_offset = 64;
            header.write_to_buf(bytes);
            for (index, value) in values.into_iter().enumerate() {
                bytes[40 + index * 8..48 + index * 8].copy_from_slice(&value.to_le_bytes());
            }
            frame.commit()?;
        }
        self.tick_count = self.tick_count.wrapping_add(1);
        Ok(())
    }
    fn shutdown(&mut self) -> cerulion_core::TransportResult<()> {
        if let Some(context) = &self.context {
            context.shutdown_signal().request();
        }
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

static NODES: std::sync::OnceLock<NodeExports<RawFfiTemplateState>> = std::sync::OnceLock::new();
fn nodes() -> &'static NodeExports<RawFfiTemplateState> {
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
static METADATA: std::sync::OnceLock<std::ffi::CString> = std::sync::OnceLock::new();
fn node_api() -> *const abi::NodeApiV1 {
    *API.get_or_init(|| {
        let metadata = METADATA.get_or_init(__cer_raw_wire_metadata).as_bytes();
        Box::leak(Box::new(abi::NodeApiV1 {
            header: abi::TableHeaderV1::for_type::<abi::NodeApiV1>(),
            metadata: abi::Bytes {
                ptr: metadata.as_ptr(),
                len: metadata.len() as u64,
            },
            required_host_capabilities: abi::capability::STATE,
            capabilities: abi::capability::STATE | abi::capability::EXTERNAL,
            create,
            tick,
            shutdown,
            destroy,
            copy_error,
            query_extension,
            state: &STATE,
            external: &EXTERNAL,
        }))
    }) as *const _
}
cerulion_core::node_abi::export_node_api!(node_api);
// CERULION:INFO_START
static INFO_BYTES: &[u8] = b"{\"inputs\":[],\"outputs\":[\"original\",\"first\",\"second\"],\"policy\":{\"period_ms\":1}}\0";

// CERULION:PORT_SCHEMAS {"outputs":{"first":"geometry_msgs/Vector3","original":"geometry_msgs/Vector3","second":"geometry_msgs/Vector3"}}
#[no_mangle]
pub extern "C" fn cerulion_node_info() -> *const std::ffi::c_char {
    // SAFETY: INFO_BYTES is a static &[u8] with a trailing null byte.
    // Casting *const u8 to *const c_char is safe: identical layout, valid and 'static.
    INFO_BYTES.as_ptr() as *const std::ffi::c_char
}

fn __cer_raw_wire_metadata() -> std::ffi::CString {
    let base = std::ffi::CStr::from_bytes_with_nul(INFO_BYTES)
        .expect("static metadata has one trailing NUL");
    cerulion_core::graph::node_plugin_abi::raw_metadata(
        base,
        &[],
        &[
            cerulion_core::graph::node_plugin_abi::HostClient::request::<
                native_ros2_messages::geometry_msgs::Vector3,
            >(),
            cerulion_core::graph::node_plugin_abi::HostClient::request::<
                native_ros2_messages::geometry_msgs::Vector3,
            >(),
            cerulion_core::graph::node_plugin_abi::HostClient::request::<
                native_ros2_messages::geometry_msgs::Vector3,
            >(),
        ],
    )
    .expect("raw schema facts match declared ports")
}
// CERULION:INFO_END
