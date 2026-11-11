// SPDX-License-Identifier: AGPL-3.0-only
//! Local typed writers over host-owned output leases.

use std::marker::PhantomData;
use std::ops::{Deref, DerefMut};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::Arc;

use crate::error::{TransportError, TransportResult};
use crate::message::ShmMessage;
use crate::wire::{MaxPayloadCapacity, WireHeader};

use super::{bytes, AnyPublisher, HostClient};
use crate::node_abi::{discard_reason, WriteSlotV1};

/// A host lease; only the host releases its actual transport sample.
struct WriteLease<'loan> {
    client: HostClient,
    publisher: &'loan mut AnyPublisher,
    slot: WriteSlotV1,
    active: bool,
    discard_reason: u32,
}

impl WriteLease<'_> {
    fn finish(&mut self, size: u32) {
        if self.active {
            // A terminal callback consumes this lease even when sending fails.
            self.active = false;
            self.client.finish_output(self.slot.lease, size);
        }
    }
}

impl Drop for WriteLease<'_> {
    fn drop(&mut self) {
        if self.active {
            self.active = false;
            self.client.discard_output(
                self.slot.lease,
                if self.discard_reason == discard_reason::SPILL_RELEASE {
                    discard_reason::SPILL_RELEASE
                } else if std::thread::panicking() {
                    discard_reason::TICK_PANICKED
                } else {
                    self.discard_reason
                },
            );
        }
    }
}

/// Node-local counterpart of the native output proxy.
///
/// It owns a local schema writer and an integer lease, never a host publisher
/// or sample. Publishing is deferred until the common tick rewrite arms it.
pub struct OutputProxy<'loan, T: ShmMessage + 'loan> {
    // Drop the writer before the lease even on panic. A hand-written schema's
    // writer destructor is permitted to inspect its still-live leased bytes.
    writer: Option<T::Writer<'loan>>,
    lease: WriteLease<'loan>,
    armed: bool,
    _not_send: PhantomData<*mut ()>,
}

impl<'loan, T: ShmMessage + 'loan> OutputProxy<'loan, T> {
    pub(super) fn loan(publisher: &'loan mut AnyPublisher) -> TransportResult<Self> {
        let _: () = T::_SHM_INVARIANTS;
        let client = HostClient::share(&publisher.client);
        let slot = client.loan_output::<T>(publisher.handle)?;
        let mut lease = WriteLease {
            client,
            publisher,
            slot,
            active: true,
            discard_reason: discard_reason::INVARIANT,
        };
        let required = crate::wire::frame_prefix_size(T::WIRE_FIXED_SIZE, T::VARIABLE_FIELD_COUNT)
            .expect("ShmMessage invariants prove the prefix fits");
        if slot.schema_hash != T::SCHEMA_HASH {
            return Err(TransportError::SchemaMismatch {
                topic: lease.publisher.topic().into(),
                expected_hash: T::SCHEMA_HASH,
                actual_hash: slot.schema_hash,
            });
        }
        if (slot.capacity as usize) < required
            || slot.prefix_size as usize != required
            || slot.configured_capacity != lease.publisher.maximum.get()
            || slot.capacity > slot.configured_capacity
            || slot.alignment < crate::shm_runtime::FRAME_ALIGN as u32
        {
            return Err(super::argument_error(
                "host output slot capacity, prefix or alignment is invalid",
            ));
        }
        // SAFETY (U09): the host granted an exclusive initialized slot. The
        // lease remains live until after the writer is destroyed, including
        // every constructor-error, tick-error and panic path.
        let frame = unsafe {
            bytes::write(
                slot.data,
                slot.capacity.into(),
                crate::shm_runtime::FRAME_ALIGN,
            )
        }
        .map_err(|error| super::span_error(lease.publisher.topic(), error))?;
        let maximum = MaxPayloadCapacity::from_max_slice_len(lease.publisher.maximum);
        let writer = T::build_writer(
            &mut frame[WireHeader::SIZE..],
            maximum,
            Arc::clone(&lease.publisher.topic),
        );
        lease.discard_reason = discard_reason::TICK_FAILED;
        Ok(Self {
            writer: Some(writer),
            lease,
            armed: false,
            _not_send: PhantomData,
        })
    }

    /// Defer publication until successful completion of the tick.
    #[doc(hidden)]
    pub fn __cer_defer_publish(&mut self) {
        self.armed = false;
    }

    /// Arm the current lease for publication.
    #[doc(hidden)]
    pub fn __cer_arm_publish(&mut self) {
        self.armed = true;
    }

    fn finalize(&mut self) {
        let Some(mut writer) = self.writer.take() else {
            return;
        };
        if !self.armed || std::thread::panicking() || self.lease.client.has_failure() {
            drop(writer);
            return;
        }
        if T::flush_staged_nested(&mut writer).is_err() {
            self.lease.discard_reason = discard_reason::NESTED_FAILED;
            drop(writer);
            return;
        }
        if !T::all_variables_written(&writer) {
            self.lease.discard_reason = discard_reason::MISSING_VARIABLE;
            drop(writer);
            return;
        }
        let Some(total) = WireHeader::SIZE
            .checked_add(T::payload_wire_size(&writer))
            .and_then(|total| u32::try_from(total).ok())
        else {
            self.lease.discard_reason = discard_reason::INVARIANT;
            drop(writer);
            return;
        };
        if total > self.lease.publisher.maximum.get() {
            self.lease.discard_reason = discard_reason::INVARIANT;
            drop(writer);
            return;
        }
        if T::has_overflow(&writer) {
            let Some(payload) = T::overflow_view_bytes(&writer) else {
                self.lease.discard_reason = discard_reason::INVARIANT;
                drop(writer);
                return;
            };
            if payload.len() != total as usize - WireHeader::SIZE {
                self.lease.discard_reason = discard_reason::INVARIANT;
                drop(writer);
                return;
            }
            // Explicit copied exception: the host synchronously copies the
            // node-owned spill and preserves the original loan timestamp. The
            // original slot stays leased until the local writer has dropped.
            self.lease.discard_reason = discard_reason::SPILL_RELEASE;
            self.lease
                .client
                .submit_spill(self.lease.slot.lease, payload);
            drop(writer);
        } else {
            if total > self.lease.slot.capacity {
                self.lease.discard_reason = discard_reason::INVARIANT;
                drop(writer);
                return;
            }
            // The writer borrow ends before the host stamps sequence/step and
            // commits. Loan-time timestamp and source order stay host-owned.
            drop(writer);
            self.lease.finish(total);
        }
    }
}

impl<'loan, T: ShmMessage + 'loan> Deref for OutputProxy<'loan, T> {
    type Target = T::Writer<'loan>;
    fn deref(&self) -> &Self::Target {
        self.writer.as_ref().expect("live output writer")
    }
}

impl<T: ShmMessage> DerefMut for OutputProxy<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.writer.as_mut().expect("live output writer")
    }
}

impl<T: ShmMessage> Drop for OutputProxy<'_, T> {
    fn drop(&mut self) {
        // Drop cannot unwind across the boundary or cause a second panic while
        // a user tick is unwinding. The lease's own drop still returns its slot.
        if let Err(payload) = catch_unwind(AssertUnwindSafe(|| self.finalize())) {
            super::contain_panic_payload(payload);
            if self.lease.discard_reason != discard_reason::SPILL_RELEASE {
                self.lease.discard_reason = discard_reason::TICK_PANICKED;
            }
            self.lease
                .client
                .record_failure(TransportError::NodeTickPanicked {
                    node_id: "output".into(),
                    reason: "node output writer panicked".into(),
                });
        }
    }
}

/// Lazy first-write loan with the common rewriter's success gate.
pub struct LazyOutput<'loan, T: ShmMessage + 'loan> {
    publisher: Option<&'loan mut AnyPublisher>,
    proxy: Option<OutputProxy<'loan, T>>,
}

impl<'loan, T: ShmMessage + 'loan> LazyOutput<'loan, T> {
    pub fn new(publisher: &'loan mut AnyPublisher) -> Self {
        Self {
            publisher: Some(publisher),
            proxy: None,
        }
    }

    #[doc(hidden)]
    pub fn __cer_loan(&mut self) -> TransportResult<&mut OutputProxy<'loan, T>> {
        if self.proxy.is_none() {
            if let Some(publisher) = self.publisher.take() {
                self.proxy = Some(OutputProxy::loan(publisher)?);
            }
        }
        self.proxy
            .as_mut()
            .ok_or_else(|| TransportError::NodeError {
                node_id: "lazy_output".into(),
                reason: "output publisher unavailable after a failed loan".into(),
            })
    }

    #[doc(hidden)]
    pub fn __cer_arm_if_loaned(&mut self) {
        if let Some(proxy) = &mut self.proxy {
            proxy.__cer_arm_publish();
        }
    }

    #[doc(hidden)]
    pub fn __cer_trace_if_unloaned(&self) {
        if let Some(publisher) = &self.publisher {
            publisher.client.diagnostic(
                publisher.handle,
                5,
                b"output untouched; no loan requested",
            );
        }
    }
}
