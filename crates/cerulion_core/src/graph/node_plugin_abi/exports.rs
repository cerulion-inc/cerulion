// SPDX-License-Identifier: AGPL-3.0-only
//! Node-owned lifecycle and diagnostic storage behind the generated C exports.

use std::mem::align_of;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::Mutex;

use crate::error::TransportResult;
use crate::node_abi::*;
use crate::state::StateSink;

use super::{registry::CallError, HostClient, NodeRegistry};

/// Generated local node implementation. No Rust trait object crosses C.
pub trait PluginNode: Sized + Send {
    fn create(client: HostClient, params: &[u8]) -> TransportResult<Self>;
    fn tick(&mut self) -> TransportResult<()>;
    fn shutdown(&mut self) -> TransportResult<()>;
    fn state_shape() -> u64;
    fn capture(&self, sink: &mut dyn StateSink) -> TransportResult<()>;
    fn restore(&mut self, payload: &[u8]) -> TransportResult<()>;
    fn inline_safe(&self) -> bool;
    fn probe(&self) -> bool;
    fn external_source(&mut self) -> TransportResult<ExternalSourceV1>;
}

// Pending cold diagnostics have a fixed budget; an uncooperative caller that
// never copies/consumes them receives BUSY rather than growing node storage.
const ERROR_CAPACITY: usize = 64;

struct Errors {
    next: u64,
    values: [Option<(u64, String)>; ERROR_CAPACITY],
}

/// Static per-library storage instantiated by the generated export module.
pub struct NodeExports<T> {
    registry: NodeRegistry<T>,
    errors: Mutex<Errors>,
}

impl<T> Default for NodeExports<T> {
    fn default() -> Self {
        Self {
            registry: NodeRegistry::default(),
            errors: Mutex::new(Errors {
                next: 1,
                values: std::array::from_fn(|_| None),
            }),
        }
    }
}

impl<T: PluginNode> NodeExports<T> {
    // hot-path-alloc-ok-fn: only returned errors allocate owner-local diagnostic text and tokens.
    fn error(&self, error: CallError) -> Status {
        match error {
            CallError::InvalidHandle => return super::status(status_code::INVALID_HANDLE),
            CallError::InvalidArgument => return super::status(status_code::INVALID_ARGUMENT),
            CallError::Unsupported => return super::status(status_code::UNSUPPORTED),
            CallError::Busy => return super::status(status_code::BUSY),
            _ => {}
        }
        let Ok(mut errors) = self.errors.lock() else {
            return super::status(status_code::PANICKED);
        };
        let Some(index) = errors.values.iter().position(Option::is_none) else {
            return super::status(status_code::BUSY);
        };
        let Some(next) = errors.next.checked_add(1) else {
            return super::status(status_code::BUSY);
        };
        let (code, kind, text) = match error {
            CallError::InvalidHandle => return super::status(status_code::INVALID_HANDLE),
            CallError::InvalidArgument => return super::status(status_code::INVALID_ARGUMENT),
            CallError::Unsupported => return super::status(status_code::UNSUPPORTED),
            CallError::Busy => return super::status(status_code::BUSY),
            CallError::Panic => (
                status_code::PANICKED,
                0,
                "node panic caught inside the execution gate".into(),
            ),
            CallError::Node(error) => (
                if matches!(error, crate::error::TransportError::NodeTickPanicked { .. }) {
                    status_code::PANICKED
                } else {
                    status_code::FAILED
                },
                error.node_error_kind().map_or(0, |kind| kind.code()),
                error.to_string(),
            ),
        };
        let token = errors.next;
        errors.next = next;
        errors.values[index] = Some((token, text));
        Status {
            code,
            kind,
            error: token,
        }
    }

    fn guarded(&self, call: impl FnOnce() -> Result<(), CallError>) -> Status {
        self.guarded_status(|| match catch_unwind(AssertUnwindSafe(call)) {
            Ok(Ok(())) => super::status(status_code::OK),
            Ok(Err(error)) => self.error(error),
            Err(_) => self.error(CallError::Panic),
        })
    }

    fn guarded_status(&self, call: impl FnOnce() -> Status) -> Status {
        catch_unwind(AssertUnwindSafe(call))
            .unwrap_or_else(|_| super::status(status_code::PANICKED))
    }

    /// Create a node from checked host callbacks and borrowed params.
    ///
    /// # Safety
    /// Host table lifetime follows HostClient::connect. Params are immutable
    /// initialized bytes for this call; output is a live aligned NodeHandle slot.
    /// The host retains the context through every node/helper call and destroy.
    // hot-path-alloc-ok-fn: create builds and registers one cold node instance.
    pub unsafe fn create(
        &self,
        host: *const HostApiV1,
        context: ContextHandle,
        params: Bytes,
        output: *mut NodeHandle,
        required: u64,
    ) -> Status {
        if !valid_output(output) {
            return super::status(status_code::INVALID_ARGUMENT);
        }
        // SAFETY (U12): caller's aligned writable output is live for this call;
        // initialize the invalid sentinel before any node/transport side effect.
        unsafe {
            output.write(NodeHandle::default());
        }
        self.guarded(|| {
            // SAFETY (U05): caller's immutable table/context lifetime is the
            // documented create contract; connect validates full shape first.
            let client =
                unsafe { HostClient::connect(host, context, required) }.map_err(|status| {
                    if status.code == status_code::UNSUPPORTED {
                        CallError::Unsupported
                    } else {
                        CallError::InvalidArgument
                    }
                })?;
            // SAFETY (U12): host params are caller-owned, immutable and live
            // through create. They are parsed/copied locally, never retained.
            let params = unsafe { super::bytes::read(params.ptr, params.len, 1) }
                .map_err(|_| CallError::InvalidArgument)?;
            // Create panic is caught before acquiring registry locks.
            let value = T::create(client.clone(), params).map_err(CallError::Node)?;
            if let Some(error) = client.take_failure() {
                return Err(CallError::Node(error));
            }
            let handle = self.registry.insert(value)?;
            // SAFETY (U12): valid_output checked aligned non-null storage;
            // caller retains it until return, and the handle is not an address.
            unsafe {
                output.write(NodeHandle { value: handle });
            }
            Ok(())
        })
    }

    pub fn tick(&self, handle: NodeHandle) -> Status {
        self.guarded(|| self.registry.call(handle.value, T::tick))
    }

    pub fn shutdown(&self, handle: NodeHandle) -> Status {
        self.guarded(|| self.registry.call(handle.value, T::shutdown))
    }

    pub fn destroy(&self, handle: NodeHandle) -> Status {
        self.guarded(|| self.registry.destroy(handle.value))
    }

    /// Copy and consume a diagnostic only when the complete text fits.
    ///
    /// # Safety
    /// Destination is initialized writable caller storage, and required is an
    /// aligned live u64 output; both are retained by the caller for this call.
    pub unsafe fn copy_error(
        &self,
        token: u64,
        destination: MutableBytes,
        required: *mut u64,
    ) -> Status {
        if !valid_output(required) {
            return super::status(status_code::INVALID_ARGUMENT);
        }
        self.guarded_status(|| {
            let Ok(mut errors) = self.errors.lock() else {
                return super::status(status_code::PANICKED);
            };
            let Some(index) = errors
                .values
                .iter()
                .position(|entry| entry.as_ref().is_some_and(|(id, _)| *id == token))
            else {
                return super::status(status_code::INVALID_HANDLE);
            };
            let text = errors.values[index]
                .as_ref()
                .expect("matching token is occupied")
                .1
                .as_bytes();
            // SAFETY (U12): checked caller-owned writable scalar output.
            unsafe {
                required.write(text.len() as u64);
            }
            if destination.len < text.len() as u64 {
                return Status {
                    code: status_code::BUFFER_TOO_SMALL,
                    error: token,
                    kind: 0,
                };
            }
            // SAFETY (U09/U12): caller lends initialized exclusive storage
            // for this call only; check null/length before writing anything.
            let Ok(bytes) = (unsafe { super::bytes::write(destination.ptr, destination.len, 1) })
            else {
                return super::status(status_code::INVALID_ARGUMENT);
            };
            bytes[..text.len()].copy_from_slice(text);
            errors.values[index] = None;
            super::status(status_code::OK)
        })
    }

    /// Write this node type's canonical state shape to caller-owned storage.
    ///
    /// # Safety
    /// output is a live initialized aligned u64 slot for this call.
    pub unsafe fn shape(&self, output: *mut u64) -> Status {
        if !valid_output(output) {
            return super::status(status_code::INVALID_ARGUMENT);
        }
        self.guarded(|| {
            let shape = T::state_shape();
            // SAFETY (U12): caller-owned output was checked non-null/aligned.
            unsafe {
                output.write(shape);
            }
            Ok(())
        })
    }

    /// Capture canonical bytes through a synchronous C sink under the node gate.
    ///
    /// # Safety
    /// sink follows CallbackSink::new's borrowed record/context lifetime rules.
    pub unsafe fn capture(
        &self,
        handle: NodeHandle,
        sink: *const StateSinkV1,
        capacity: u64,
    ) -> Status {
        self.guarded_status(|| {
            // SAFETY (U12/U07): live caller-owned sink for this entire call;
            // new validates its pointer/callback before any state is captured.
            let mut sink = match unsafe { super::state::CallbackSink::new(sink, capacity) } {
                Ok(sink) => sink,
                Err(_) => return super::status(status_code::INVALID_ARGUMENT),
            };
            let result = self
                .registry
                .call(handle.value, |node| node.capture(&mut sink));
            match sink.failure() {
                Some(status) if status.code == status_code::PANICKED => {
                    self.error(CallError::Panic)
                }
                Some(status) if status.code == status_code::BUFFER_TOO_SMALL => {
                    super::status(status_code::BUFFER_TOO_SMALL)
                }
                Some(_) => self.error(CallError::Node(super::argument_error(
                    "state sink refused the capture",
                ))),
                None => match result {
                    Ok(()) => super::status(status_code::OK),
                    Err(error) => self.error(error),
                },
            }
        })
    }

    /// Restore borrowed canonical bytes under the node gate.
    ///
    /// # Safety
    /// Payload is immutable initialized caller storage retained until return.
    pub unsafe fn restore(&self, handle: NodeHandle, payload: Bytes) -> Status {
        self.guarded(|| {
            // SAFETY (U12): host owns/live-retains the immutable payload; the
            // local state cursor borrows it only through this synchronous call.
            let bytes = unsafe { super::bytes::read(payload.ptr, payload.len, 1) }
                .map_err(|_| CallError::InvalidArgument)?;
            self.registry.call(handle.value, |node| node.restore(bytes))
        })
    }

    /// Write inline carrier eligibility under the instance gate.
    ///
    /// # Safety
    /// output is a live aligned initialized u32 slot for this call.
    pub unsafe fn inline_safe(&self, handle: NodeHandle, output: *mut u32) -> Status {
        if !valid_output(output) {
            return super::status(status_code::INVALID_ARGUMENT);
        }
        self.guarded(|| {
            self.registry.call(handle.value, |node| {
                // SAFETY (U12): the caller owns this checked scalar output.
                unsafe {
                    output.write(u32::from(node.inline_safe()));
                }
                Ok(())
            })
        })
    }

    pub fn probe(&self, handle: NodeHandle) -> Status {
        match catch_unwind(AssertUnwindSafe(|| {
            self.registry.call(handle.value, |node| Ok(node.probe()))
        })) {
            Ok(Ok(true)) => super::status(status_code::OK),
            Ok(Ok(false)) => super::status(status_code::BUSY),
            Ok(Err(error)) => self.error(error),
            Err(_) => self.error(CallError::Panic),
        }
    }

    /// Return one primitive source descriptor, never a device/closure owner.
    ///
    /// # Safety
    /// output is a live aligned ExternalSourceV1 slot for this synchronous call.
    pub unsafe fn external(&self, handle: NodeHandle, output: *mut ExternalSourceV1) -> Status {
        if !valid_output(output) {
            return super::status(status_code::INVALID_ARGUMENT);
        }
        self.guarded(|| {
            self.registry.call(handle.value, |node| {
                let source = node.external_source()?;
                // SAFETY (U12): caller owns the checked complete scalar descriptor.
                unsafe {
                    output.write(source);
                }
                Ok(())
            })
        })
    }
}

fn valid_output<T>(output: *mut T) -> bool {
    !output.is_null() && output.addr().is_multiple_of(align_of::<T>())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::{NodeErrorKind, TransportError};
    use crate::state::BoundedSink;

    struct CounterNode {
        value: u64,
    }

    impl PluginNode for CounterNode {
        fn create(_: HostClient, _: &[u8]) -> TransportResult<Self> {
            Ok(Self { value: 7 })
        }
        fn tick(&mut self) -> TransportResult<()> {
            self.value += 1;
            Ok(())
        }
        fn shutdown(&mut self) -> TransportResult<()> {
            self.value = 0;
            Ok(())
        }
        fn state_shape() -> u64 {
            0x1234
        }
        fn capture(&self, sink: &mut dyn StateSink) -> TransportResult<()> {
            sink.write(&self.value.to_le_bytes())
                .map_err(|_| super::super::argument_error("state sink is full"))
        }
        fn restore(&mut self, payload: &[u8]) -> TransportResult<()> {
            let bytes: [u8; 8] = payload.try_into().map_err(|_| {
                super::super::argument_error("counter state must contain eight bytes")
            })?;
            self.value = u64::from_le_bytes(bytes);
            Ok(())
        }
        fn inline_safe(&self) -> bool {
            true
        }
        fn probe(&self) -> bool {
            true
        }
        fn external_source(&mut self) -> TransportResult<ExternalSourceV1> {
            Ok(ExternalSourceV1 {
                fd: -1,
                kind: external::HOST_DRIVEN,
                flags: 0,
            })
        }
    }

    fn installed() -> (NodeExports<CounterNode>, NodeHandle) {
        let exports = NodeExports::default();
        let value = exports.registry.insert(CounterNode { value: 7 }).unwrap();
        (exports, NodeHandle { value })
    }

    #[test]
    fn callbacks_capture_restore_and_destroy_match_literal_state_oracles() {
        let (exports, handle) = installed();
        assert_eq!(exports.tick(handle).code, status_code::OK);
        let mut storage = [0u8; 8];
        let mut sink = BoundedSink::new(&mut storage);
        let mut bridge = super::super::state::SinkBridge::new(&mut sink);
        let descriptor = bridge.descriptor();
        // SAFETY: synchronous bridge and caller-owned state remain live.
        assert_eq!(
            unsafe { exports.capture(handle, &descriptor, 8) }.code,
            status_code::OK
        );
        assert_eq!(sink.written(), &[8, 0, 0, 0, 0, 0, 0, 0]);
        let restored = [41, 0, 0, 0, 0, 0, 0, 0];
        // SAFETY: initialized oracle bytes are borrowed only through restore.
        assert_eq!(
            unsafe { exports.restore(handle, super::super::client::bytes_span(&restored)) }.code,
            status_code::OK
        );
        assert_eq!(exports.tick(handle).code, status_code::OK);
        assert_eq!(
            exports
                .registry
                .call(handle.value, |node| Ok(node.value))
                .unwrap(),
            42
        );
        let mut shape = 0;
        let mut inline = 0;
        let mut source = ExternalSourceV1 { fd: -1, kind: 0, flags: 0 };
        // SAFETY: aligned caller-owned scalar outputs remain live.
        unsafe {
            assert_eq!(exports.shape(&mut shape).code, status_code::OK);
            assert_eq!(
                exports.inline_safe(handle, &mut inline).code,
                status_code::OK
            );
            assert_eq!(exports.external(handle, &mut source).code, status_code::OK);
        }
        assert_eq!(
            (shape, inline, source.fd, source.kind),
            (0x1234, 1, -1, external::HOST_DRIVEN)
        );
        assert_eq!(exports.probe(handle).code, status_code::OK);
        assert_eq!(exports.shutdown(handle).code, status_code::OK);
        assert_eq!(
            exports
                .registry
                .call(handle.value, |node| Ok(node.value))
                .unwrap(),
            0
        );
        assert_eq!(exports.destroy(handle).code, status_code::OK);
        assert_eq!(exports.tick(handle).code, status_code::INVALID_HANDLE);
        assert_eq!(exports.destroy(handle).code, status_code::INVALID_HANDLE);
    }

    #[test]
    fn diagnostic_query_short_retry_complete_copy_and_stale_token_are_distinct() {
        let (exports, _) = installed();
        let status = exports.error(CallError::Panic);
        assert_eq!(status.code, status_code::PANICKED);
        assert_eq!(status.error, 1);
        let oracle = b"node panic caught inside the execution gate";
        let mut required = 0;
        // SAFETY: empty span requires no storage; required is a live output.
        let query = unsafe {
            exports.copy_error(
                status.error,
                MutableBytes {
                    ptr: std::ptr::null_mut(),
                    len: 0,
                },
                &mut required,
            )
        };
        assert_eq!(
            (query.code, query.error, required),
            (status_code::BUFFER_TOO_SMALL, 1, oracle.len() as u64)
        );
        let mut short = [0xa5; 3];
        // SAFETY: live exclusive caller buffer.
        let retry = unsafe {
            exports.copy_error(
                status.error,
                super::super::client::mutable_bytes(&mut short),
                &mut required,
            )
        };
        assert_eq!(retry.code, status_code::BUFFER_TOO_SMALL);
        assert_eq!(short, [0xa5; 3]);
        let mut text = [0u8; 43];
        assert_eq!(text.len(), oracle.len());
        // SAFETY: complete live caller-owned destination.
        assert_eq!(
            unsafe {
                exports.copy_error(
                    status.error,
                    super::super::client::mutable_bytes(&mut text),
                    &mut required,
                )
            }
            .code,
            status_code::OK
        );
        assert_eq!(&text, oracle);
        // SAFETY: empty destination and aligned output remain valid.
        assert_eq!(
            unsafe {
                exports.copy_error(
                    status.error,
                    MutableBytes {
                        ptr: std::ptr::null_mut(),
                        len: 0,
                    },
                    &mut required,
                )
            }
            .code,
            status_code::INVALID_HANDLE
        );
        assert_eq!(exports.error(CallError::Panic).error, 2);
    }

    #[test]
    fn diagnostic_budget_and_generation_exhaustion_do_not_grow_or_wrap() {
        let (exports, _) = installed();
        for token in 1..=ERROR_CAPACITY as u64 {
            assert_eq!(exports.error(CallError::Panic).error, token);
        }
        assert_eq!(exports.error(CallError::Panic).code, status_code::BUSY);
        assert_eq!(
            exports
                .errors
                .lock()
                .unwrap()
                .values
                .iter()
                .flatten()
                .count(),
            ERROR_CAPACITY
        );
        let (exhausted, _) = installed();
        exhausted.errors.lock().unwrap().next = u64::MAX;
        assert_eq!(exhausted.error(CallError::Panic).code, status_code::BUSY);
        assert!(exhausted
            .errors
            .lock()
            .unwrap()
            .values
            .iter()
            .all(Option::is_none));
    }

    #[test]
    fn invalid_create_table_and_spans_are_refused_before_node_effects() {
        let (exports, handle) = installed();
        let mut output = NodeHandle { value: 999 };
        // SAFETY: null table is deliberately refused without dereference;
        // initialized aligned output and empty params are valid.
        let status = unsafe {
            exports.create(
                std::ptr::null(),
                ContextHandle { value: 1 },
                Bytes {
                    ptr: std::ptr::null(),
                    len: 0,
                },
                &mut output,
                0,
            )
        };
        assert_eq!(
            (status.code, output.value),
            (status_code::INVALID_ARGUMENT, 0)
        );
        let header = TableHeaderV1 {
            size_bytes: std::mem::size_of::<TableHeaderV1>() as u32,
            ..TableHeaderV1::for_type::<HostApiV1>()
        };
        // SAFETY: only the live fixed header is read; its short size forbids
        // full-table interpretation or any function-pointer invocation.
        let status = unsafe {
            exports.create(
                (&header as *const TableHeaderV1).cast(),
                ContextHandle { value: 1 },
                Bytes {
                    ptr: std::ptr::null(),
                    len: 0,
                },
                &mut output,
                0,
            )
        };
        assert_eq!(status.code, status_code::INVALID_ARGUMENT);
        // SAFETY: malformed null span is refused before access or restore.
        assert_eq!(
            unsafe {
                exports.restore(
                    handle,
                    Bytes {
                        ptr: std::ptr::null(),
                        len: 8,
                    },
                )
            }
            .code,
            status_code::INVALID_ARGUMENT
        );
        // SAFETY: null outputs/sink are rejected before access.
        unsafe {
            assert_eq!(
                exports.shape(std::ptr::null_mut()).code,
                status_code::INVALID_ARGUMENT
            );
            assert_eq!(
                exports.inline_safe(handle, std::ptr::null_mut()).code,
                status_code::INVALID_ARGUMENT
            );
            assert_eq!(
                exports.external(handle, std::ptr::null_mut()).code,
                status_code::INVALID_ARGUMENT
            );
            assert_eq!(
                exports.capture(handle, std::ptr::null(), 8).code,
                status_code::INVALID_ARGUMENT
            );
            assert_eq!(
                exports
                    .copy_error(
                        0,
                        MutableBytes {
                            ptr: std::ptr::null_mut(),
                            len: 0
                        },
                        std::ptr::null_mut()
                    )
                    .code,
                status_code::INVALID_ARGUMENT
            );
        }
        assert_eq!(
            exports
                .registry
                .call(handle.value, |node| Ok(node.value))
                .unwrap(),
            7
        );
    }

    #[test]
    fn sink_capacity_refusal_does_not_deliver_partial_counter_state() {
        let (exports, handle) = installed();
        let mut storage = [0xa5; 8];
        let mut sink = BoundedSink::new(&mut storage);
        let mut bridge = super::super::state::SinkBridge::new(&mut sink);
        let descriptor = bridge.descriptor();
        // SAFETY: synchronous sink descriptor is live for capture.
        assert_eq!(
            unsafe { exports.capture(handle, &descriptor, 7) }.code,
            status_code::BUFFER_TOO_SMALL
        );
        assert!(sink.written().is_empty());
        assert_eq!(storage, [0xa5; 8]);
    }

    #[test]
    fn typed_user_failure_and_caught_panic_preserve_boundary_class() {
        let (exports, handle) = installed();
        let error = exports.guarded(|| {
            exports.registry.call(handle.value, |_| {
                Err(TransportError::NodeFailed {
                    node_id: "counter".into(),
                    kind: NodeErrorKind::Logic,
                    reason: "retry oracle".into(),
                })
            })
        });
        assert_eq!(
            (error.code, error.kind),
            (status_code::FAILED, NodeErrorKind::Logic.code())
        );
        let panic = exports.guarded(|| {
            exports
                .registry
                .call::<()>(handle.value, |_| panic!("counter panic oracle"))
        });
        assert_eq!(panic.code, status_code::PANICKED);
        assert_eq!(exports.tick(handle).code, status_code::OK);
        let writer = exports.error(CallError::Node(TransportError::NodeTickPanicked {
            node_id: "writer".into(),
            reason: "writer panic oracle".into(),
        }));
        assert_eq!(writer.code, status_code::PANICKED);
        assert_eq!(
            exports.error(CallError::Unsupported).code,
            status_code::UNSUPPORTED
        );
        let busy = exports.guarded(|| {
            exports.registry.call(handle.value, |_| {
                assert_eq!(exports.tick(handle).code, status_code::BUSY);
                Ok(())
            })
        });
        assert_eq!(busy.code, status_code::OK);
    }

    #[test]
    fn independent_runs_deliver_the_same_hand_written_counter_state() {
        for _ in 0..2 {
            let (exports, handle) = installed();
            assert_eq!(exports.tick(handle).code, status_code::OK);
            assert_eq!(exports.tick(handle).code, status_code::OK);
            let mut storage = [0u8; 8];
            let mut sink = BoundedSink::new(&mut storage);
            let mut bridge = super::super::state::SinkBridge::new(&mut sink);
            let descriptor = bridge.descriptor();
            // SAFETY: live synchronous sink and storage.
            assert_eq!(
                unsafe { exports.capture(handle, &descriptor, 8) }.code,
                status_code::OK
            );
            assert_eq!(sink.written(), &[9, 0, 0, 0, 0, 0, 0, 0]);
        }
    }
}
