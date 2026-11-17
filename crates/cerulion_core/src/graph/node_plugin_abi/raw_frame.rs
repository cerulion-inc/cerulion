// SPDX-License-Identifier: AGPL-3.0-only
//! Dynamic-schema loans and reads; all transport owners remain on the host.

use super::{bytes, AnyPublisher, AnySubscriber, HostClient};
use crate::error::TransportResult;
use crate::node_abi::{discard_reason, LoanRequestV1, ReadSlotV1, WriteSlotV1};
use crate::wire::WireHeader;
use std::marker::PhantomData;
use std::rc::Rc;

/// Immutable full-frame borrow pinned by a host-owned lease.
pub struct RawFrameRead {
    client: HostClient,
    slot: ReadSlotV1,
    length: usize,
    hash: u64,
    _not_send: PhantomData<Rc<()>>,
}
impl RawFrameRead {
    pub fn schema_hash(&self) -> u64 {
        self.hash
    }
    pub fn bytes(&self) -> &[u8] {
        // SAFETY (U08): creation validated this immutable span and the lease
        // pins it until this owner drops; returned borrows cannot outlive self.
        unsafe {
            bytes::read(
                self.slot.data,
                self.length as u64,
                crate::shm_runtime::FRAME_ALIGN,
            )
        }
        .expect("validated host read span remains leased")
    }
}
impl Drop for RawFrameRead {
    fn drop(&mut self) {
        self.client.release_read(self.slot.lease);
    }
}
impl AnySubscriber {
    pub fn read_raw_frame(&mut self, hash: u64) -> TransportResult<Option<RawFrameRead>> {
        let Some(slot) = self.client.receive_input(self.handle, self.read_mode)? else {
            return Ok(None);
        };
        let mut frame = RawFrameRead {
            client: HostClient::share(&self.client),
            slot,
            length: slot.size as usize,
            hash,
            _not_send: PhantomData,
        };
        if slot.alignment < crate::shm_runtime::FRAME_ALIGN as u32 || slot.schema_hash != hash {
            return Err(super::argument_error(
                "raw input descriptor has invalid alignment or schema",
            ));
        }
        // SAFETY (U08): the live lease retains the entire immutable slot,
        // including all validation failure paths; no byte pointer escapes it.
        let raw =
            unsafe { bytes::read(slot.data, slot.size.into(), crate::shm_runtime::FRAME_ALIGN) }
                .map_err(|error| super::span_error(self.topic(), error))?;
        let (_, payload) =
            crate::transport::input_view::validate_wire_frame_raw(self.topic(), raw, Some(hash))?;
        frame.length = payload.end;
        Ok(Some(frame))
    }
}

/// Exclusive full-frame loan. Dropping it discards; commit consumes the lease.
/// Buffer exports must keep this local owner until the last export ends.
pub struct RawFrameLoan {
    client: HostClient,
    slot: WriteSlotV1,
    size: u32,
    spill: Option<Vec<u8>>,
    active: bool,
    reason: u32,
    _not_send: PhantomData<Rc<()>>,
}
impl RawFrameLoan {
    pub fn bytes_mut(&mut self) -> &mut [u8] {
        if let Some(ref mut spill) = self.spill {
            return spill;
        }
        // SAFETY (U09): creation validated the exclusive initialized slot;
        // its live lease pins it, and the mutable borrow is tied to this owner.
        unsafe {
            bytes::write(
                self.slot.data,
                self.size.into(),
                crate::shm_runtime::FRAME_ALIGN,
            )
        }
        .expect("validated exclusive host loan remains live")
    }
    pub fn commit(mut self) -> TransportResult<()> {
        self.active = false;
        if let Some(spill) = &self.spill {
            // The explicit adaptive exception borrows only payload bytes while
            // this Vec and the original lease both remain live synchronously.
            self.client
                .submit_spill(self.slot.lease, &spill[WireHeader::SIZE..]);
            self.client
                .discard_output(self.slot.lease, discard_reason::SPILL_RELEASE);
        } else {
            self.client.finish_output(self.slot.lease, self.size);
        }
        self.client.take_failure().map_or(Ok(()), Err)
    }
}
impl Drop for RawFrameLoan {
    fn drop(&mut self) {
        if self.active {
            self.active = false;
            self.client.discard_output(
                self.slot.lease,
                if std::thread::panicking() {
                    discard_reason::TICK_PANICKED
                } else {
                    self.reason
                },
            );
        }
    }
}
impl AnyPublisher {
    pub fn loan_raw_frame(
        &mut self,
        request: LoanRequestV1,
        total: usize,
    ) -> TransportResult<RawFrameLoan> {
        let size =
            u32::try_from(total).map_err(|_| super::argument_error("raw frame exceeds u32"))?;
        if total < request.minimum_capacity as usize || size > self.maximum.get() {
            return Err(super::argument_error("raw frame exceeds configured bounds"));
        }
        let slot = self.client.loan_frame(self.handle, request)?;
        let mut loan = RawFrameLoan {
            client: HostClient::share(&self.client),
            slot,
            size,
            spill: None,
            active: true,
            reason: discard_reason::INVARIANT,
            _not_send: PhantomData,
        };
        if slot.schema_hash != request.schema_hash
            || slot.capacity < request.minimum_capacity
            || slot.prefix_size != request.minimum_capacity
            || slot.configured_capacity != self.maximum.get()
            || slot.capacity > slot.configured_capacity
            || slot.alignment < crate::shm_runtime::FRAME_ALIGN as u32
        {
            return Err(super::argument_error(
                "raw output descriptor has invalid schema, prefix, capacity or alignment",
            ));
        }
        // SAFETY (U09): the exclusive lease remains live across validation and
        // any adaptive spill allocation; a rejected descriptor is discarded.
        unsafe {
            bytes::write(
                slot.data,
                slot.capacity.into(),
                crate::shm_runtime::FRAME_ALIGN,
            )
        }
        .map_err(|error| super::span_error(self.topic(), error))?;
        if size > slot.capacity {
            // hot-path-alloc-ok: explicit adaptive spill exception, submitted synchronously to the host.
            loan.spill = Some(vec![0; total]);
        } else {
            loan.bytes_mut().fill(0);
        }
        loan.reason = discard_reason::TICK_FAILED;
        Ok(loan)
    }
}
