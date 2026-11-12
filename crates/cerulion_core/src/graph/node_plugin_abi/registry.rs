// SPDX-License-Identifier: AGPL-3.0-only
//! Node-owned handles and execution gates. No handle encodes an address.

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::{Arc, Mutex, TryLockError};

use crate::error::{TransportError, TransportResult};

/// Failure while entering or executing a node-owned instance.
#[derive(Debug)]
pub enum CallError {
    /// Zero, unknown or already destroyed handle.
    InvalidHandle,
    /// Invalid table, span or caller-owned output storage.
    InvalidArgument,
    /// A required host capability is missing.
    Unsupported,
    /// Another operation owns the instance gate, or the handle counter is full.
    InvalidState,
    /// Node code unwound; the unwind was caught inside the execution gate.
    Panic(&'static str),
    /// The canonical state encoder unwound inside its own guard.
    EncoderPanic,
    /// The canonical state decoder unwound inside its own guard.
    DecoderPanic,
    /// A host state sink refused a write, including a caught host sink panic.
    StateSinkFull(String),
    /// A canonical state codec refused with its complete native diagnostic.
    StateCodec(String),
    /// A poisoned registry or instance mutex prevented admission.
    Poisoned {
        operation: &'static str,
        mutex: &'static str,
    },
    /// A typed user/transport error returned normally.
    Node(TransportError),
}

impl CallError {
    pub(super) fn at_operation(self, operation: &'static str) -> Self {
        match self {
            Self::Panic(_) => Self::Panic(operation),
            Self::Poisoned { mutex, .. } => Self::Poisoned { operation, mutex },
            error => error,
        }
    }
}

struct Row<T> {
    handle: u64,
    instance: Arc<Mutex<Option<T>>>,
}

/// Registry allocated by the node library, never by the host.
///
/// Handle values increase without reuse. Destroy invalidates the row before
/// dropping the value, while another caller that already obtained the row sees
/// `None`. Independent instance gates allow independent nodes to execute together.
pub struct NodeRegistry<T> {
    rows: Mutex<Vec<Row<T>>>,
    next: Mutex<u64>,
}

impl<T> Default for NodeRegistry<T> {
    // hot-path-alloc-ok-fn: bootstrap initializes the empty node-owned registry once.
    fn default() -> Self {
        Self {
            rows: Mutex::new(Vec::new()),
            next: Mutex::new(1),
        }
    }
}

impl<T> NodeRegistry<T> {
    /// Insert initialized node-local state and mint a nonzero handle.
    // hot-path-alloc-ok-fn: node create allocates node-owned instance storage once.
    pub fn insert(&self, value: T) -> Result<u64, CallError> {
        let mut rows = self.rows.lock().map_err(|_| CallError::Poisoned {
            operation: "",
            mutex: "NODES",
        })?;
        let mut next = self.next.lock().map_err(|_| CallError::Poisoned {
            operation: "",
            mutex: "NODES",
        })?;
        let following = next.checked_add(1).ok_or(CallError::InvalidState)?;
        let handle = *next;
        rows.push(Row {
            handle,
            instance: Arc::new(Mutex::new(Some(value))),
        });
        *next = following;
        Ok(handle)
    }

    fn instance(&self, handle: u64) -> Result<Arc<Mutex<Option<T>>>, CallError> {
        if handle == 0 {
            return Err(CallError::InvalidHandle);
        }
        let rows = self.rows.lock().map_err(|_| CallError::Poisoned {
            operation: "",
            mutex: "NODES",
        })?;
        rows.iter()
            .find(|row| row.handle == handle)
            .map(|row| Arc::clone(&row.instance))
            .ok_or(CallError::InvalidHandle)
    }

    /// Execute a callback with one instance exclusively borrowed.
    ///
    /// The catch is inside the gate, so a user panic cannot poison its mutex.
    /// A callback cannot retain the mutable borrow or overlap destruction.
    pub fn call<R>(
        &self,
        handle: u64,
        call: impl FnOnce(&mut T) -> TransportResult<R>,
    ) -> Result<R, CallError> {
        self.call_with(handle, |node| call(node).map_err(CallError::Node))
    }

    pub(super) fn call_with<R>(
        &self,
        handle: u64,
        call: impl FnOnce(&mut T) -> Result<R, CallError>,
    ) -> Result<R, CallError> {
        let instance = self.instance(handle)?;
        let mut gate = instance.try_lock().map_err(|error| match error {
            TryLockError::WouldBlock => CallError::InvalidState,
            TryLockError::Poisoned(_) => CallError::Poisoned {
                operation: "",
                mutex: "node instance",
            },
        })?;
        let value = gate.as_mut().ok_or(CallError::InvalidHandle)?;
        catch_unwind(AssertUnwindSafe(|| call(value))).map_err(|payload| {
            super::contain_panic_payload(payload);
            CallError::Panic("")
        })?
    }

    /// Invalidate an instance and drop its state with its own compiler/allocator.
    /// A caught destructor panic still leaves the handle permanently invalid.
    pub fn destroy(&self, handle: u64) -> Result<(), CallError> {
        self.destroy_with(handle, |_| Ok(()), || ())
    }
    pub(super) fn destroy_with(
        &self,
        handle: u64,
        before: impl FnOnce(&mut T) -> TransportResult<()>,
        invalidated: impl FnOnce(),
    ) -> Result<(), CallError> {
        let instance = self.instance(handle)?;
        let mut gate = instance.try_lock().map_err(|error| match error {
            TryLockError::WouldBlock => CallError::InvalidState,
            TryLockError::Poisoned(_) => CallError::Poisoned {
                operation: "",
                mutex: "node instance",
            },
        })?;
        let value = gate.as_mut().ok_or(CallError::InvalidHandle)?;
        catch_unwind(AssertUnwindSafe(|| before(value)))
            .map_err(|payload| {
                super::contain_panic_payload(payload);
                CallError::Panic("")
            })
            .and_then(|result| result.map_err(CallError::Node))?;
        let value = gate.take().ok_or(CallError::InvalidHandle)?;
        let mut rows = self.rows.lock().map_err(|_| CallError::Poisoned {
            operation: "",
            mutex: "NODES",
        })?;
        rows.retain(|row| row.handle != handle);
        drop(rows);
        invalidated();
        catch_unwind(AssertUnwindSafe(|| drop(value))).map_err(|payload| {
            super::contain_panic_payload(payload);
            CallError::Panic("")
        })?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn poisoned_instance_is_distinct_from_a_caught_execution_panic() {
        let registry = NodeRegistry::default();
        let handle = registry.insert(7_u64).unwrap();
        let instance = registry.instance(handle).unwrap();
        let poison = catch_unwind(AssertUnwindSafe(|| {
            let _guard = instance.lock().unwrap();
            panic!("instance mutex poison oracle");
        }));
        assert!(poison.is_err());
        let error = registry.call(handle, |value| Ok(*value)).unwrap_err();
        assert!(matches!(
            error.at_operation("cerulion_node_tick"),
            CallError::Poisoned {
                operation: "cerulion_node_tick",
                mutex: "node instance"
            }
        ));
    }

    #[test]
    fn deferred_pre_destroy_refusal_preserves_the_live_instance_and_handle() {
        let registry = NodeRegistry::default();
        let handle = registry.insert(17_u64).unwrap();
        let invalidated = std::cell::Cell::new(false);
        assert!(matches!(
            registry.destroy_with(
                handle,
                |_| Err(super::super::argument_error("release refused")),
                || invalidated.set(true)
            ),
            Err(CallError::Node(_))
        ));
        assert!(!invalidated.get());
        assert_eq!(registry.call(handle, |value| Ok(*value)).unwrap(), 17);
        assert!(matches!(
            registry.destroy_with(
                handle,
                |_| panic!("deferred flush panic oracle"),
                || invalidated.set(true)
            ),
            Err(CallError::Panic(""))
        ));
        assert!(!invalidated.get());
        assert_eq!(registry.call(handle, |value| Ok(*value)).unwrap(), 17);
        registry
            .destroy_with(handle, |_| Ok(()), || invalidated.set(true))
            .unwrap();
        assert!(invalidated.get());
        assert!(matches!(
            registry.call(handle, |_| Ok(())),
            Err(CallError::InvalidHandle)
        ));
    }

    #[test]
    fn handles_are_nonzero_distinct_and_never_reused_after_destroy() {
        let registry = NodeRegistry::default();
        let first = registry.insert(11_u64).unwrap();
        let second = registry.insert(22_u64).unwrap();
        assert_eq!((first, second), (1, 2));
        assert_eq!(registry.call(first, |value| Ok(*value)).unwrap(), 11);
        registry.destroy(first).unwrap();
        assert!(matches!(
            registry.call(first, |_| Ok(())),
            Err(CallError::InvalidHandle)
        ));
        assert!(matches!(
            registry.destroy(first),
            Err(CallError::InvalidHandle)
        ));
        assert!(matches!(
            registry.call(0, |_| Ok(())),
            Err(CallError::InvalidHandle)
        ));
        assert_eq!(registry.insert(33).unwrap(), 3);
        assert_eq!(registry.call(second, |value| Ok(*value)).unwrap(), 22);
    }

    #[test]
    fn callback_panic_is_caught_inside_the_gate_and_next_call_remains_valid() {
        let registry = NodeRegistry::default();
        let handle = registry.insert(0_u64).unwrap();
        assert!(matches!(
            registry.call::<()>(handle, |value| {
                *value = 7;
                panic!("node callback oracle");
            }),
            Err(CallError::Panic(""))
        ));
        assert_eq!(registry.call(handle, |value| Ok(*value)).unwrap(), 7);
        registry.destroy(handle).unwrap();
    }

    #[test]
    fn reentrant_call_is_busy_and_independent_instance_is_callable() {
        let registry = NodeRegistry::default();
        let first = registry.insert(11_u64).unwrap();
        let second = registry.insert(22_u64).unwrap();
        registry
            .call(first, |_| {
                assert!(matches!(
                    registry.call(first, |_| Ok(())),
                    Err(CallError::InvalidState)
                ));
                assert!(matches!(
                    registry.destroy(first),
                    Err(CallError::InvalidState)
                ));
                assert_eq!(registry.call(second, |value| Ok(*value)).unwrap(), 22);
                Ok(())
            })
            .unwrap();
    }

    #[test]
    fn exhausted_handle_counter_does_not_insert_or_wrap() {
        let registry = NodeRegistry::default();
        *registry.next.lock().unwrap() = u64::MAX;
        assert!(matches!(
            registry.insert(7_u64),
            Err(CallError::InvalidState)
        ));
        assert!(registry.rows.lock().unwrap().is_empty());
        assert_eq!(*registry.next.lock().unwrap(), u64::MAX);
    }

    #[test]
    fn destructor_panic_invalidates_the_handle_and_preserves_other_instances() {
        struct Drops {
            panic: bool,
        }
        impl Drop for Drops {
            fn drop(&mut self) {
                assert!(!self.panic, "destructor oracle");
            }
        }
        let registry = NodeRegistry::default();
        let first = registry.insert(Drops { panic: true }).unwrap();
        let second = registry.insert(Drops { panic: false }).unwrap();
        assert!(matches!(registry.destroy(first), Err(CallError::Panic(""))));
        assert!(matches!(
            registry.call(first, |_| Ok(())),
            Err(CallError::InvalidHandle)
        ));
        registry.call(second, |_| Ok(())).unwrap();
        registry.destroy(second).unwrap();
    }
}

#[cfg(test)]
mod panic_payload_test {
    use super::*;
    struct DestructingPayload;
    impl Drop for DestructingPayload {
        fn drop(&mut self) {
            panic!("panic payload destructor");
        }
    }
    #[test]
    fn hostile_panic_payload_does_not_poison_the_instance_or_escape() {
        let registry = NodeRegistry::default();
        let handle = registry.insert(41u64).unwrap();
        assert!(matches!(
            registry.call::<()>(handle, |_| {
                std::panic::panic_any(DestructingPayload);
            }),
            Err(CallError::Panic(""))
        ));
        assert_eq!(registry.call(handle, |value| Ok(*value)).unwrap(), 41);
        registry.destroy(handle).unwrap();
    }
}
