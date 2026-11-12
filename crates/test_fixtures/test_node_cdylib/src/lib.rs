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
    retained: Option<RetainedOutput>,
}

impl PluginNode for TestNodeState {
    fn create(client: HostClient, _params: &[u8]) -> cerulion_core::TransportResult<Self> {
        let cold = COLD_HOST.with(std::cell::Cell::take);
        let info = cerulion_core::graph::node::DylibNodeEntry::parse_info_json_labeled(
            std::str::from_utf8(if retain_output_mode() {
                RETAIN_METADATA.get_or_init(retain_metadata).as_bytes()
            } else {
                &INFO_BYTES[..INFO_BYTES.len() - 1]
            })
            .map_err(|error| cerulion_core::TransportError::GraphError {
                reason: error.to_string(),
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
            retained: if retain_output_mode() {
                Some(RetainedOutput::new(cold.expect("checked create context"))?)
            } else {
                None
            },
        })
    }
    fn tick(&mut self) -> cerulion_core::TransportResult<()> {
        if self.tick_fail {
            return Err(cerulion_core::TransportError::NodeError {
                node_id: "raw fixture".into(),
                reason: "untyped fixture tick failure".into(),
            });
        }
        if let Some(retained) = &mut self.retained {
            retained.tick()?;
        }
        self.tick_count = self.tick_count.wrapping_add(1);
        Ok(())
    }
    fn shutdown(&mut self) -> cerulion_core::TransportResult<()> {
        if let Some(retained) = &mut self.retained {
            retained.release()?;
        }
        self.context = None;
        Ok(())
    }
    fn state_shape() -> u64 {
        0
    }
    fn capture(
        &self,
        _: &mut dyn cerulion_core::state::StateSink,
    ) -> cerulion_core::TransportResult<()> {
        Ok(())
    }
    fn restore(&mut self, _: &[u8]) -> cerulion_core::TransportResult<()> {
        Ok(())
    }
    fn inline_safe(&self) -> bool {
        false
    }
    fn probe(&self) -> bool {
        false
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
    // SAFETY: host retains immutable descriptor/code/context for the instance;
    // the maintained checker validates its complete identity before caching it.
    let api = match unsafe { abi::read_host_api(host) } {
        Ok(api) => api,
        Err(_) => {
            return abi::Status {
                code: abi::status_code::INVALID_ARGUMENT,
                ..Default::default()
            }
        }
    };
    let previous = COLD_HOST.with(|slot| slot.replace(Some((api, context))));
    // SAFETY: caller's initialized parameter span and exclusive result stay live
    // through the maintained registry's checked, panic-contained create.
    let result = unsafe { nodes().create(host, context, params, output, 0) };
    COLD_HOST.with(|slot| slot.set(previous));
    result
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
static API: [std::sync::OnceLock<&'static abi::NodeApiV1>; 6] =
    [const { std::sync::OnceLock::new() }; 6];
static METADATA: [std::sync::OnceLock<std::ffi::CString>; 4] =
    [const { std::sync::OnceLock::new() }; 4];
fn node_api() -> *const abi::NodeApiV1 {
    let fault = std::env::var_os("CER_ABI_FAULT").is_some();
    let variant = if retain_output_mode() {
        4
    } else {
        match std::env::var("CER_INLINE_SAFE_METADATA").as_deref() {
            Ok("true") => 1,
            Ok("false") => 2,
            Ok("invalid") => 3,
            _ => 0,
        }
    };
    *API[if fault { 5 } else { variant }].get_or_init(|| {
        let metadata = if variant == 4 {
            RETAIN_METADATA.get_or_init(retain_metadata).as_bytes()
        } else {
            METADATA[variant]
                .get_or_init(|| {
                    let base = __cer_raw_wire_metadata();
                    let mut value: serde_json::Value =
                        serde_json::from_slice(base.to_bytes()).expect("generated metadata");
                    let fields = value.as_object_mut().expect("metadata object");
                    match variant {
                        0 => {
                            fields.remove("inline_safe");
                        }
                        1 => {
                            fields.insert("inline_safe".into(), true.into());
                        }
                        2 => {
                            fields.insert("inline_safe".into(), false.into());
                        }
                        _ => {
                            fields.insert("inline_safe".into(), "true".into());
                        }
                    }
                    std::ffi::CString::new(serde_json::to_vec(&value).expect("metadata bytes"))
                        .expect("no embedded NUL")
                })
                .as_bytes()
        };
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
            required_host_capabilities: 0,
            capabilities: 0,
            create,
            tick,
            shutdown,
            destroy,
            copy_error,
            query_extension,
            state: std::ptr::null(),
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

fn retain_output_mode() -> bool {
    std::env::var_os("CER_RETAIN_OUTPUT").is_some()
}
static RETAIN_METADATA: std::sync::OnceLock<std::ffi::CString> = std::sync::OnceLock::new();
fn retain_metadata() -> std::ffi::CString {
    use cerulion_core::message::ShmMessage;
    let hash = native_ros2_messages::geometry_msgs::Vector3::SCHEMA_HASH;
    std::ffi::CString::new(format!(r#"{{"inputs":[{{"name":"inp","trigger":true,"schema_hash":{hash},"wire_fixed_size":24,"variable_count":0,"alignment":8}}],"outputs":[{{"name":"out","schema_hash":{hash},"wire_fixed_size":24,"variable_count":0,"alignment":8}}],"policy":{{"data_trigger":{{"input_name":"inp"}}}}}}"#)).expect("valid metadata")
}
thread_local! {
    static COLD_HOST: std::cell::Cell<Option<(abi::HostApiV1, abi::ContextHandle)>> = const { std::cell::Cell::new(None) };
}
struct RetainedOutput {
    api: abi::HostApiV1,
    context: abi::ContextHandle,
    input: abi::PortHandle,
    output: abi::PortHandle,
    pending: Option<abi::LeaseHandle>,
    ticks: u64,
}
fn accepted(result: abi::Status) -> cerulion_core::TransportResult<()> {
    if result.code == abi::status_code::OK {
        Ok(())
    } else {
        Err(cerulion_core::TransportError::NodeError {
            node_id: "retained output fixture".into(),
            reason: format!("host callback refused with status {}", result.code),
        })
    }
}
impl RetainedOutput {
    fn new(
        (api, context): (abi::HostApiV1, abi::ContextHandle),
    ) -> cerulion_core::TransportResult<Self> {
        let mut ports = [abi::PortHandle::default(); 2];
        for (index, direction) in [abi::direction::INPUT, abi::direction::OUTPUT]
            .into_iter()
            .enumerate()
        {
            let mut binding = abi::PortBindingV1 {
                port: abi::PortHandle::default(),
                topic: abi::Bytes {
                    ptr: std::ptr::null(),
                    len: 0,
                },
                configured_capacity: 0,
                read_mode: 0,
            };
            // SAFETY: checked cached table/code/context remain host-owned and live;
            // admitted create lends exclusive initialized output for this call.
            accepted(unsafe { (api.bind_port)(context, 0, direction, &mut binding) })?;
            ports[index] = binding.port;
        }
        Ok(Self {
            api,
            context,
            input: ports[0],
            output: ports[1],
            pending: None,
            ticks: 0,
        })
    }
    fn tick(&mut self) -> cerulion_core::TransportResult<()> {
        use cerulion_core::message::ShmMessage;
        if let Some(lease) = self.pending.take() {
            // SAFETY: scalar lease belongs to this context, retained from the
            // previous admitted tick; host owns its unchanged slot until finish.
            accepted(unsafe { (self.api.finish_output)(self.context, lease, 64) })?;
        }
        let mut read = abi::ReadSlotV1 {
            data: std::ptr::null(),
            lease: abi::LeaseHandle::default(),
            schema_hash: 0,
            size: 0,
            alignment: 0,
        };
        // SAFETY: admitted tick, checked input handle and exclusive initialized
        // output; host retains the mapped frame until our terminal release below.
        accepted(unsafe {
            (self.api.receive_input)(self.context, self.input, abi::read_mode::ONE, &mut read)
        })?;
        let mut write = abi::WriteSlotV1 {
            data: std::ptr::null_mut(),
            lease: abi::LeaseHandle::default(),
            schema_hash: 0,
            capacity: 0,
            configured_capacity: 0,
            alignment: 0,
            prefix_size: 0,
        };
        let request = abi::LoanRequestV1 {
            schema_hash: native_ros2_messages::geometry_msgs::Vector3::SCHEMA_HASH,
            minimum_capacity: 64,
            wire_fixed_size: 24,
            variable_count: 0,
            alignment: 8,
        };
        // SAFETY: checked output handle/descriptor, admitted exclusive tick; the
        // initialized result is borrowed until its scalar lease is consumed.
        let loan =
            unsafe { (self.api.loan_output)(self.context, self.output, request, &mut write) };
        if loan.code == abi::status_code::OK
            && !read.data.is_null()
            && !write.data.is_null()
            && read.size >= 64
            && write.capacity >= 64
        {
            // SAFETY: distinct producer/read and node/output segments, validated
            // fixed prefixes and live leases; 24 payload bytes stay within both.
            unsafe {
                std::slice::from_raw_parts_mut(write.data.add(40), 24)
                    .copy_from_slice(std::slice::from_raw_parts(read.data.add(40), 24));
            }
            self.pending = Some(write.lease);
        }
        // SAFETY: admitted terminal release of exactly the live read lease; no
        // reference to its mapping survives the synchronous payload transform.
        accepted(unsafe { (self.api.release_input)(self.context, read.lease) })?;
        accepted(loan)?;
        if self.pending.is_none() {
            // SAFETY: an admitted successful loan minted this scalar lease;
            // malformed bounds refuse the frame and return it to its host owner.
            accepted(unsafe {
                (self.api.discard_output)(self.context, write.lease, abi::discard_reason::INVARIANT)
            })?;
            return Err(cerulion_core::TransportError::NodeError {
                node_id: "retained output fixture".into(),
                reason: "host returned a malformed fixed slot".into(),
            });
        }
        self.ticks += 1;
        if self.ticks > 1 {
            if let Some(lease) = self.pending.take() {
                // SAFETY: just-loaned host-owned slot initialized above, admitted
                // terminal commit; sequence is assigned once by the host.
                accepted(unsafe { (self.api.finish_output)(self.context, lease, 64) })?;
            }
        }
        Ok(())
    }
    fn release(&mut self) -> cerulion_core::TransportResult<()> {
        if let Some(lease) = self.pending.take() {
            // SAFETY: admitted shutdown releases its retained slot once through
            // its host owner; no node allocation or mapping is freed across C.
            accepted(unsafe {
                (self.api.discard_output)(self.context, lease, abi::discard_reason::TICK_FAILED)
            })?;
        }
        Ok(())
    }
}
