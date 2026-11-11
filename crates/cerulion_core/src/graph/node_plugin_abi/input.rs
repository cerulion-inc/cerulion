// SPDX-License-Identifier: AGPL-3.0-only
//! Local reader borrows; sample, head and history ownership stay on the host.

use std::marker::PhantomData;
use std::ops::Deref;

use crate::error::{TransportError, TransportResult};
use crate::message::ShmMessage;
use crate::node_abi::ReadSlotV1;
use crate::transport::input_view::InputView as NativeInputView;

use super::{bytes, AnySubscriber, HostClient};

/// Host-held immutable sample, released only after all local views are gone.
pub(super) struct ReadLease {
    client: HostClient,
    slot: ReadSlotV1,
}

impl ReadLease {
    pub(super) fn new(client: HostClient, slot: ReadSlotV1) -> Self {
        Self { client, slot }
    }

    fn native_view<'a, T: ShmMessage + 'a>(
        &self,
        topic: &str,
    ) -> TransportResult<NativeInputView<'a, T>> {
        if self.slot.alignment < crate::shm_runtime::FRAME_ALIGN as u32 {
            return Err(TransportError::Deserialization {
                topic: "node input".into(),
                reason: "host read slot alignment is below the wire alignment".into(),
            });
        }
        if self.slot.schema_hash != T::SCHEMA_HASH {
            return Err(TransportError::SchemaMismatch {
                topic: topic.into(),
                expected_hash: T::SCHEMA_HASH,
                actual_hash: self.slot.schema_hash,
            });
        }
        // ScheduledBytes owns only a local byte borrow, never a host sample.
        // The same constructor and validators serve the native receive path.
        // SAFETY (U08): the caller keeps the guard live through the callback,
        // or borrows a host batch frozen for the entire fire. Native accessor
        // borrows cannot outlive that scope; no sample owner crosses C.
        let raw = unsafe {
            bytes::read(
                self.slot.data,
                self.slot.size.into(),
                crate::shm_runtime::FRAME_ALIGN,
            )
        }
        .map_err(|error| super::span_error(topic, error))?;
        NativeInputView::from_borrowed(topic, raw)
    }

    fn view<'a, T: ShmMessage + 'a>(&'a self, topic: &str) -> TransportResult<InputView<'a, T>> {
        Ok(InputView {
            view: self.native_view::<T>(topic)?,
            _lease: None,
        })
    }

    fn batch_view<'a, T: ShmMessage + 'a>(
        self,
        topic: &'a str,
    ) -> TransportResult<InputView<'a, T>> {
        // Accumulated frames belong to the host's frozen fire batch, just as
        // native Accumulated::iter borrows AccumulateBuffer. The borrowed
        // accumulator topic ties 'a to this fire; releasing an indexed lease
        // never mutates or clears that frozen batch during the node call.
        Ok(InputView {
            view: self.native_view::<T>(topic)?,
            _lease: Some(self),
        })
    }
}

/// A node-local typed reader and the integer lease keeping its backing live.
pub struct InputView<'sample, T: ShmMessage + 'sample> {
    view: NativeInputView<'sample, T>,
    _lease: Option<ReadLease>,
}

impl<T: ShmMessage> InputView<'_, T> {
    pub fn wire_header(&self) -> crate::wire::WireHeader {
        self.view.wire_header()
    }
    pub fn wire_timestamp_ns(&self) -> u64 {
        self.view.wire_timestamp_ns()
    }
}

impl<'sample, T: ShmMessage + 'sample> Deref for InputView<'sample, T> {
    type Target = T::Reader<'sample>;
    fn deref(&self) -> &Self::Target {
        &self.view
    }
}

impl Drop for ReadLease {
    fn drop(&mut self) {
        self.client.release_read(self.slot.lease);
    }
}

impl AnySubscriber {
    /// Serve the host-selected frame and release its local borrow at return.
    pub fn try_view<T: ShmMessage, R>(
        &mut self,
        callback: impl FnOnce(InputView<'_, T>) -> R,
    ) -> TransportResult<Option<R>> {
        let Some(slot) = self.client.receive_input(self.handle, self.read_mode)? else {
            return Ok(None);
        };
        let lease = ReadLease::new(HostClient::share(&self.client), slot);
        let view = lease.view::<T>(self.topic())?;
        // Keep the owner OUTSIDE the callback's view. Fixed readers are copyable
        // references: dropping a view early must not release bytes still read
        // inside that callback. This matches native try_view's sample owner.
        Ok(Some(callback(view)))
    }
}

/// Frames borrowed from the host's accumulate buffers for one fire.
/// No frame is copied into the node and no accumulator state crosses the ABI.
pub struct Accumulated<'a, T: ShmMessage> {
    topic: &'a str,
    client: Option<HostClient>,
    port: crate::node_abi::PortHandle,
    count: usize,
    dropped: u64,
    dropped_total: u64,
    _schema: PhantomData<fn() -> T>,
}

impl<'a, T: ShmMessage + 'a> Accumulated<'a, T> {
    pub(super) fn new(
        topic: &'a str,
        client: Option<HostClient>,
        port: crate::node_abi::PortHandle,
        count: usize,
        dropped: u64,
        dropped_total: u64,
    ) -> Self {
        Self {
            topic,
            client,
            port,
            count,
            dropped,
            dropped_total,
            _schema: PhantomData,
        }
    }
    pub fn len(&self) -> usize {
        self.count
    }
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }
    pub fn dropped(&self) -> u64 {
        self.dropped
    }
    pub fn dropped_total(&self) -> u64 {
        self.dropped_total
    }

    pub fn iter(&self) -> impl Iterator<Item = InputView<'a, T>> + 'a {
        let topic = self.topic;
        let client = self.client.as_ref().map(HostClient::share);
        let port = self.port;
        (0..self.count).map_while(move |index| {
            let client = client.as_ref()?;
            let result = client
                .accumulate_frame(port, index as u32)
                .and_then(|slot| {
                    ReadLease::new(HostClient::share(client), slot).batch_view::<T>(topic)
                });
            match result {
                Ok(view) => Some(view),
                Err(error) => {
                    // The author iteration contract yields views, not Results.
                    // A host/validation failure is latched and returned by the
                    // generated tick adapter after the user body, never hidden.
                    client.record_failure(error);
                    None
                }
            }
        })
    }
}
