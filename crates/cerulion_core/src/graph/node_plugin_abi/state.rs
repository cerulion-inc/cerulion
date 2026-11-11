// SPDX-License-Identifier: AGPL-3.0-only
//! Synchronous state sinks, with allocator and trait ownership kept local.

use std::mem::{align_of, offset_of};
use std::panic::{catch_unwind, AssertUnwindSafe};

use crate::error::TransportResult;
use crate::node_abi::{status_code, Bytes, StateSinkV1, Status};
use crate::state::{SinkFull, StateSink};

use super::client::bytes_span;

/// Adapter over a borrowed C sink, never over a foreign Rust trait object.
pub struct CallbackSink {
    descriptor: StateSinkV1,
    remaining: u64,
    failed: Option<Status>,
}

impl CallbackSink {
    /// Borrow a caller-owned synchronous sink for one state capture.
    ///
    /// # Safety
    /// `sink` points to a live initialized StateSinkV1 for this call. Its
    /// callback/context remain valid, contain their own panics and never retain
    /// the node-owned byte chunks. The caller serializes mutable sink access.
    pub unsafe fn new(sink: *const StateSinkV1, capacity: u64) -> TransportResult<Self> {
        if sink.is_null() || !sink.addr().is_multiple_of(align_of::<StateSinkV1>()) {
            return Err(super::argument_error("state sink pointer is invalid"));
        }
        // SAFETY (U12): fixed caller-owned record is live/aligned. Check the fn
        // address before constructing a typed record with a required fn pointer.
        let address = unsafe {
            sink.cast::<u8>()
                .add(offset_of!(StateSinkV1, write))
                .cast::<usize>()
                .read()
        };
        if address == 0 {
            return Err(super::argument_error("state sink callback is null"));
        }
        // SAFETY (U12): the caller owns a complete immutable descriptor, and
        // its callback representation was checked. No context cast is made.
        let descriptor = unsafe { sink.read() };
        Ok(Self {
            descriptor,
            remaining: capacity,
            failed: None,
        })
    }

    pub fn failure(&self) -> Option<Status> {
        self.failed
    }
}

impl StateSink for CallbackSink {
    fn write(&mut self, bytes: &[u8]) -> Result<(), SinkFull> {
        if self.failed.is_some() || bytes.len() as u64 > self.remaining {
            self.failed.get_or_insert(Status {
                code: status_code::BUFFER_TOO_SMALL,
                ..Status::default()
            });
            return Err(SinkFull);
        }
        // SAFETY (U07): new checked the immutable callback descriptor; the
        // context remains caller-owned and bytes are borrowed only until this
        // synchronous callback returns. The host contains its own Rust panics.
        let status = unsafe { (self.descriptor.write)(self.descriptor.context, bytes_span(bytes)) };
        if status.code != status_code::OK {
            self.failed = Some(status);
            return Err(SinkFull);
        }
        self.remaining -= bytes.len() as u64;
        Ok(())
    }

    fn remaining_hint(&self) -> Option<usize> {
        Some(usize::try_from(self.remaining).unwrap_or(usize::MAX))
    }

    fn refuse(&mut self) {
        self.failed.get_or_insert(Status {
            code: status_code::BUFFER_TOO_SMALL,
            ..Status::default()
        });
    }
}

/// Opaque context for host accumulator capture to return bytes to a local sink.
pub(super) struct SinkBridge<'a> {
    sink: &'a mut dyn StateSink,
    failed: Option<Status>,
}

impl<'a> SinkBridge<'a> {
    pub(super) fn new(sink: &'a mut dyn StateSink) -> Self {
        Self { sink, failed: None }
    }

    pub(super) fn descriptor(&mut self) -> StateSinkV1 {
        StateSinkV1 {
            context: (self as *mut Self).cast(),
            write: bridge_write,
        }
    }

    pub(super) fn check(&self) -> TransportResult<()> {
        match self.failed {
            None => Ok(()),
            Some(status) if status.code == status_code::PANICKED => {
                Err(crate::error::TransportError::NodeTickPanicked {
                    node_id: "state".into(),
                    reason: "state sink panicked".into(),
                })
            }
            Some(_) => Err(super::argument_error("state sink is full")),
        }
    }
}

unsafe extern "C" fn bridge_write(context: *mut u8, chunk: Bytes) -> Status {
    let result = catch_unwind(AssertUnwindSafe(|| {
        if context.is_null() || !context.addr().is_multiple_of(align_of::<SinkBridge<'_>>()) {
            return super::status(status_code::INVALID_ARGUMENT);
        }
        // SAFETY (U10): descriptor() supplied this exact node-owned bridge.
        // The host calls synchronously/serially and does not retain its pointer.
        // The outer capture keeps the bridge and its mutable sink borrow live.
        let bridge = unsafe { &mut *context.cast::<SinkBridge<'_>>() };
        if let Some(status) = bridge.failed {
            return status;
        }
        // SAFETY (U12): host chunk is initialized and immutable for this
        // synchronous call. It is never retained or freed by the node sink.
        let Ok(bytes) = (unsafe { super::bytes::read(chunk.ptr, chunk.len, 1) }) else {
            let status = super::status(status_code::INVALID_ARGUMENT);
            bridge.failed = Some(status);
            return status;
        };
        let result = catch_unwind(AssertUnwindSafe(|| bridge.sink.write(bytes)));
        let status = match result {
            Ok(Ok(())) => super::status(status_code::OK),
            Ok(Err(_)) => super::status(status_code::BUFFER_TOO_SMALL),
            Err(_) => super::status(status_code::PANICKED),
        };
        if status.code != status_code::OK {
            bridge.failed = Some(status);
        }
        status
    }));
    result.unwrap_or_else(|_| super::status(status_code::PANICKED))
}
