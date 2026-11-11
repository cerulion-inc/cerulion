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
    /// Complete deferred host releases before an admitted user callback.
    fn flush_deferred(&self) -> TransportResult<()> {
        Ok(())
    }
    fn tick(&mut self) -> TransportResult<()>;
    fn shutdown(&mut self) -> TransportResult<()>;
    fn state_shape() -> u64;
    fn capture(&self, sink: &mut dyn StateSink) -> TransportResult<()>;
    fn restore(&mut self, payload: &[u8]) -> TransportResult<()>;
    fn inline_safe(&self) -> bool;
    fn probe(&self) -> bool;
    fn external_source(&mut self) -> TransportResult<ExternalSourceV1>;
}

// Each live instance has one replacing diagnostic, provisioned at create.
// Failed creates share one replacing library slot. Tokens never name memory
// and are not reused after replacement, consumption or instance destruction.
struct DiagnosticSlot {
    handle: u64,
    pending: Option<(u64, String)>,
    retired: bool,
}
struct Errors {
    next: u64,
    library: Option<(u64, String)>,
    instances: Vec<DiagnosticSlot>,
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
                library: None,
                // hot-path-alloc-ok: empty slots are initialized only during cold library construction.
                instances: Vec::new(),
            }),
        }
    }
}
impl<T: PluginNode> NodeExports<T> {
    // hot-path-alloc-ok-fn: provision one local diagnostic slot per cold instance creation.
    fn register_diagnostic(&self, handle: u64) -> Result<(), CallError> {
        let mut errors = self.errors.lock().map_err(|_| CallError::Panic)?;
        errors.instances.push(DiagnosticSlot {
            handle,
            pending: None,
            retired: false,
        });
        Ok(())
    }
    #[cfg(test)]
    fn error(&self, error: CallError) -> Status {
        self.error_for(0, error)
    }
    // hot-path-alloc-ok-fn: full owner-local text is formatted only after a returned failure.
    fn error_for(&self, owner: u64, error: CallError) -> Status {
        let Ok(mut errors) = self.errors.lock() else {
            return super::status(status_code::PANICKED);
        };
        let index = if owner == 0 {
            None
        } else {
            let Some(index) = errors
                .instances
                .iter()
                .position(|slot| slot.handle == owner)
            else {
                return super::status(match error {
                    CallError::InvalidArgument => status_code::INVALID_ARGUMENT,
                    CallError::Unsupported => status_code::UNSUPPORTED,
                    CallError::InvalidState => status_code::INVALID_STATE,
                    _ => status_code::INVALID_HANDLE,
                });
            };
            Some(index)
        };
        // Even a scalar-only new error replaces the prior pending diagnostic.
        match index {
            None => errors.library = None,
            Some(i) => errors.instances[i].pending = None,
        }
        match error {
            CallError::InvalidHandle => return super::status(status_code::INVALID_HANDLE),
            CallError::InvalidArgument => return super::status(status_code::INVALID_ARGUMENT),
            CallError::Unsupported => return super::status(status_code::UNSUPPORTED),
            CallError::InvalidState => return super::status(status_code::INVALID_STATE),
            _ => {}
        }
        let Some(next) = errors.next.checked_add(1) else {
            return super::status(status_code::INVALID_STATE);
        };
        let (code, kind, text) = match error {
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
                super::diagnostic_reason(&error).into_owned(),
            ),
            _ => unreachable!("scalar refusal returned before diagnostic formatting"),
        };
        let token = errors.next;
        errors.next = next;
        match index {
            None => errors.library = Some((token, text)),
            Some(i) => errors.instances[i].pending = Some((token, text)),
        }
        Status {
            code,
            kind,
            error: token,
        }
    }
    fn clear_instance_diagnostic(&self, owner: u64) {
        if let Ok(mut errors) = self.errors.lock() {
            if let Some(slot) = errors
                .instances
                .iter_mut()
                .find(|slot| slot.handle == owner)
            {
                slot.pending = None;
            }
        }
    }
    fn guarded(&self, call: impl FnOnce() -> Result<(), CallError>) -> Status {
        self.guarded_for(0, call)
    }
    fn guarded_node(
        &self,
        handle: NodeHandle,
        call: impl FnOnce() -> Result<(), CallError>,
    ) -> Status {
        self.guarded_for(handle.value, call)
    }
    fn guarded_for(&self, owner: u64, call: impl FnOnce() -> Result<(), CallError>) -> Status {
        self.guarded_status(|| match catch_unwind(AssertUnwindSafe(call)) {
            Ok(Ok(())) => super::status(status_code::OK),
            Ok(Err(error)) => self.error_for(owner, error),
            Err(payload) => {
                super::contain_panic_payload(payload);
                self.error_for(owner, CallError::Panic)
            }
        })
    }
    fn guarded_status(&self, call: impl FnOnce() -> Status) -> Status {
        catch_unwind(AssertUnwindSafe(call)).unwrap_or_else(|payload| {
            super::contain_panic_payload(payload);
            super::status(status_code::PANICKED)
        })
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
            return self.error_for(0, CallError::InvalidArgument);
        }
        // SAFETY (U12): caller's aligned writable output is live for this call;
        // initialize the invalid sentinel before any node/transport side effect.
        let output = unsafe { &mut *output };
        *output = NodeHandle::default();
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
            if let Err(error) = self.register_diagnostic(handle) {
                let _ = self.registry.destroy(handle);
                return Err(error);
            }
            // The caller-owned output borrow stays exclusive through create.
            *output = NodeHandle { value: handle };
            Ok(())
        })
    }

    fn call_node<R>(
        &self,
        handle: u64,
        call: impl FnOnce(&mut T) -> TransportResult<R>,
    ) -> Result<R, CallError> {
        self.registry.call(handle, |node| {
            node.flush_deferred()?;
            call(node)
        })
    }

    pub fn tick(&self, handle: NodeHandle) -> Status {
        self.guarded_node(handle, || self.call_node(handle.value, T::tick))
    }

    pub fn shutdown(&self, handle: NodeHandle) -> Status {
        self.guarded_node(handle, || self.call_node(handle.value, T::shutdown))
    }

    pub fn destroy(&self, handle: NodeHandle) -> Status {
        let invalidated = std::cell::Cell::new(false);
        let status = self.guarded_node(handle, || {
            self.registry.destroy_with(
                handle.value,
                |node| node.flush_deferred(),
                || invalidated.set(true),
            )
        });
        if let Ok(mut errors) = self.errors.lock() {
            if status.code == status_code::OK {
                errors.instances.retain(|slot| slot.handle != handle.value);
            } else if status.code == status_code::PANICKED && invalidated.get() {
                if let Some(slot) = errors
                    .instances
                    .iter_mut()
                    .find(|slot| slot.handle == handle.value)
                {
                    slot.retired = true;
                }
            }
        }
        status
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
            let index = if errors.library.as_ref().is_some_and(|(id, _)| *id == token) {
                None
            } else {
                let Some(index) = errors
                    .instances
                    .iter()
                    .position(|slot| slot.pending.as_ref().is_some_and(|(id, _)| *id == token))
                else {
                    return super::status(status_code::INVALID_HANDLE);
                };
                Some(index)
            };
            let text = match index {
                None => errors.library.as_ref(),
                Some(i) => errors.instances[i].pending.as_ref(),
            }
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
            match index {
                None => errors.library = None,
                Some(i) if errors.instances[i].retired => {
                    errors.instances.remove(i);
                }
                Some(i) => errors.instances[i].pending = None,
            }
            super::status(status_code::OK)
        })
    }

    /// Write this node type's canonical state shape to caller-owned storage.
    ///
    /// # Safety
    /// output is a live initialized aligned u64 slot for this call.
    pub unsafe fn shape(&self, output: *mut u64) -> Status {
        if !valid_output(output) {
            return self.error_for(0, CallError::InvalidArgument);
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
                Err(_) => return self.error_for(handle.value, CallError::InvalidArgument),
            };
            let result = self.call_node(handle.value, |node| node.capture(&mut sink));
            match sink.failure() {
                Some(status) if status.code == status_code::PANICKED => {
                    self.error_for(handle.value, CallError::Panic)
                }
                Some(status) if status.code == status_code::BUFFER_TOO_SMALL => {
                    self.clear_instance_diagnostic(handle.value);
                    super::status(status_code::BUFFER_TOO_SMALL)
                }
                Some(_) => self.error_for(
                    handle.value,
                    CallError::Node(super::argument_error("state sink refused the capture")),
                ),
                None => match result {
                    Ok(()) => super::status(status_code::OK),
                    Err(error) => self.error_for(handle.value, error),
                },
            }
        })
    }

    /// Restore borrowed canonical bytes under the node gate.
    ///
    /// # Safety
    /// Payload is immutable initialized caller storage retained until return.
    pub unsafe fn restore(&self, handle: NodeHandle, payload: Bytes) -> Status {
        self.guarded_node(handle, || {
            // SAFETY (U12): host owns/live-retains the immutable payload; the
            // local state cursor borrows it only through this synchronous call.
            let bytes = unsafe { super::bytes::read(payload.ptr, payload.len, 1) }
                .map_err(|_| CallError::InvalidArgument)?;
            self.call_node(handle.value, |node| node.restore(bytes))
        })
    }

    /// Write inline carrier eligibility under the instance gate.
    ///
    /// # Safety
    /// output is a live aligned initialized u32 slot for this call.
    pub unsafe fn inline_safe(&self, handle: NodeHandle, output: *mut u32) -> Status {
        if !valid_output(output) {
            return self.error_for(handle.value, CallError::InvalidArgument);
        }
        self.guarded_node(handle, || {
            self.call_node(handle.value, |node| {
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
            self.call_node(handle.value, |node| Ok(node.probe()))
        })) {
            Ok(Ok(true)) => super::status(status_code::OK),
            Ok(Ok(false)) => self.error_for(handle.value, CallError::InvalidState),
            Ok(Err(error)) => self.error_for(handle.value, error),
            Err(payload) => {
                super::contain_panic_payload(payload);
                self.error_for(handle.value, CallError::Panic)
            }
        }
    }

    /// Return one primitive source descriptor, never a device/closure owner.
    ///
    /// # Safety
    /// output is a live aligned ExternalSourceV1 slot for this synchronous call.
    pub unsafe fn external(&self, handle: NodeHandle, output: *mut ExternalSourceV1) -> Status {
        if !valid_output(output) {
            return self.error_for(handle.value, CallError::InvalidArgument);
        }
        self.guarded_node(handle, || {
            self.call_node(handle.value, |node| {
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
        fn create(_: HostClient, params: &[u8]) -> TransportResult<Self> {
            if params == b"fail" {
                return Err(crate::error::NodeError::Logic("create probe".into())
                    .into_transport("user_init"));
            }
            Ok(Self { value: 7 })
        }
        fn tick(&mut self) -> TransportResult<()> {
            if self.value == 998 {
                return Err(crate::error::NodeError::Logic("tick probe".into())
                    .into_transport("zero_copy_tick"));
            }
            if self.value == 999 {
                return Err(super::super::argument_error("counter refuses 999"));
            }
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
            self.value != 0
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
        exports.register_diagnostic(value).unwrap();
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
        let mut source = ExternalSourceV1 {
            fd: -1,
            kind: 0,
            flags: 0,
        };
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
        assert_eq!(exports.probe(handle).code, status_code::INVALID_STATE);
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
    fn repeated_instance_and_create_errors_replace_tokens_without_growing_storage() {
        let (exports, handle) = installed();
        exports
            .registry
            .call(handle.value, |node| {
                node.value = 999;
                Ok(())
            })
            .unwrap();
        let mut previous = 0;
        for _ in 0..100 {
            let status = exports.tick(handle);
            assert_eq!(status.code, status_code::FAILED);
            assert_ne!(status.error, previous);
            if previous != 0 {
                let mut required = 0;
                // SAFETY: null empty destination and aligned required output are live.
                assert_eq!(
                    unsafe {
                        exports.copy_error(
                            previous,
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
            }
            previous = status.error;
        }
        for _ in 0..100 {
            assert_eq!(exports.error(CallError::Panic).code, status_code::PANICKED);
        }
        let errors = exports.errors.lock().unwrap();
        assert_eq!(errors.instances.len(), 1);
        assert_eq!(
            errors
                .instances
                .iter()
                .filter(|slot| slot.pending.is_some())
                .count(),
            1
        );
        assert!(errors.library.is_some());
        drop(errors);
        exports.errors.lock().unwrap().next = u64::MAX;
        assert_eq!(
            exports.error_for(handle.value, CallError::Panic).code,
            status_code::INVALID_STATE
        );
        assert!(exports.errors.lock().unwrap().instances[0]
            .pending
            .is_none());
        assert_eq!(exports.destroy(handle).code, status_code::OK);
        assert!(exports.errors.lock().unwrap().instances.is_empty());
    }

    fn copied_reason(exports: &NodeExports<CounterNode>, token: u64) -> String {
        let mut bytes = [0u8; 128];
        let mut required = 0;
        // SAFETY: exclusive initialized bytes and aligned required scalar
        // remain live through the checked synchronous diagnostic copy.
        let status = unsafe {
            exports.copy_error(
                token,
                super::super::client::mutable_bytes(&mut bytes),
                &mut required,
            )
        };
        assert_eq!(status.code, status_code::OK);
        String::from_utf8(bytes[..required as usize].to_vec()).unwrap()
    }

    #[test]
    fn complete_empty_reason_copy_consumes_once_without_rewriting_text() {
        let (exports, handle) = installed();
        let status = exports.error_for(
            handle.value,
            CallError::Node(TransportError::NodeError {
                node_id: "empty reason".into(),
                reason: String::new(),
            }),
        );
        assert_eq!(copied_reason(&exports, status.error), "");
        let mut required = 1;
        // SAFETY: empty null span is never dereferenced; required is aligned local storage.
        let stale = unsafe {
            exports.copy_error(
                status.error,
                MutableBytes {
                    ptr: std::ptr::null_mut(),
                    len: 0,
                },
                &mut required,
            )
        };
        assert_eq!(stale.code, status_code::INVALID_HANDLE);
    }

    #[test]
    fn typed_rust_tick_error_copy_preserves_the_original_reason_exactly() {
        let (exports, handle) = installed();
        exports
            .registry
            .call(handle.value, |node| {
                node.value = 998;
                Ok(())
            })
            .unwrap();
        let status = exports.tick(handle);
        assert_eq!((status.code, status.kind), (status_code::FAILED, 1));
        assert_eq!(
            copied_reason(&exports, status.error),
            "Logic error: tick probe"
        );
    }

    #[test]
    fn typed_rust_create_error_copy_preserves_the_original_reason_exactly() {
        use crate::graph::node::{NodeContext, ShutdownSignal};
        use crate::graph::node_host_abi::{
            register_context, remove_context, HostContext, HOST_API,
        };
        let environment = std::sync::Arc::new(std::collections::HashMap::new());
        let context = NodeContext::with_runtime_env(
            Default::default(),
            Default::default(),
            std::sync::Arc::new(crate::clock::VirtualClock::new()),
            ShutdownSignal::new(),
            environment.clone(),
        );
        let context = ContextHandle {
            value: register_context(HostContext::new(context, Vec::new(), environment, 1).unwrap())
                .unwrap(),
        };
        let exports = NodeExports::<CounterNode>::default();
        let mut output = NodeHandle { value: 99 };
        // SAFETY: actual immutable process table and registered host context
        // remain live; params/output are initialized caller-owned allocations.
        let status = unsafe {
            exports.create(
                &HOST_API,
                context,
                super::super::client::bytes_span(b"fail"),
                &mut output,
                0,
            )
        };
        assert_eq!(
            (status.code, status.kind, output.value),
            (status_code::FAILED, 1, 0)
        );
        assert_eq!(
            copied_reason(&exports, status.error),
            "Logic error: create probe"
        );
        remove_context(context.value).unwrap();
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
                assert_eq!(exports.tick(handle).code, status_code::INVALID_STATE);
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

    #[test]
    fn sink_panic_is_caught_and_next_capture_remains_callable() {
        struct PanicSink;
        impl StateSink for PanicSink {
            fn write(&mut self, _: &[u8]) -> Result<(), crate::state::SinkFull> {
                panic!("sink panic oracle")
            }
        }
        let (exports, handle) = installed();
        let mut sink = PanicSink;
        let mut bridge = super::super::state::SinkBridge::new(&mut sink);
        let descriptor = bridge.descriptor();
        // SAFETY: node-owned synchronous bridge remains live; it catches the
        // panicking Rust sink before returning through its extern C callback.
        assert_eq!(
            unsafe { exports.capture(handle, &descriptor, 8) }.code,
            status_code::PANICKED
        );
        assert_eq!(exports.tick(handle).code, status_code::OK);
    }
}
