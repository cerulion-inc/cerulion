// SPDX-License-Identifier: AGPL-3.0-only
//! Checked, cached calls to the host's immutable C table.

use std::mem::{align_of, offset_of};
use std::sync::{Arc, Mutex};

use crate::error::{NodeErrorKind, TransportError, TransportResult};
use crate::message::ShmMessage;
use crate::node_abi::*;
use crate::wire::WireHeader;

struct ClientInner {
    api: HostApiV1,
    context: ContextHandle,
    failure: Mutex<Option<TransportError>>,
}

/// Node-owned cache of checked C function pointers and a host-minted handle.
/// Local Arc ownership never crosses the boundary.
#[derive(Clone)]
pub struct HostClient(Arc<ClientInner>);

impl std::fmt::Debug for HostClient {
    fn fmt(&self, out: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        out.debug_struct("HostClient")
            .field("context", &self.0.context)
            .finish_non_exhaustive()
    }
}

// SAFETY (U07): connect validates every cached signature/table and context;
// each caller supplies live, correctly aligned output storage or byte spans
// borrowed for this synchronous call. Host callbacks validate handle membership
// and contain their own Rust panics; no host Rust object is ever dereferenced.
macro_rules! host_call {
    ($client:expr, $function:ident $(, $argument:expr)* $(,)?) => {{
        unsafe { ($client.0.api.$function)($client.0.context $(, $argument)*) }
    }};
}

impl HostClient {
    /// Share the node-owned callback cache without allocating or copying data.
    pub fn share(client: &Self) -> Self {
        Self(Arc::clone(&client.0))
    }
    /// Validate and copy the host table once at cold create.
    ///
    /// # Safety
    /// `api` points to immutable host-owned storage covering at least the fixed
    /// header, and the complete table when the header advertises its exact size.
    /// Every non-null callback has its checked signature and stays callable for
    /// the context lifetime; the host keeps this context alive through all node
    /// calls and any retained external helper. Trusted native code supplies it.
    // hot-path-alloc-ok-fn: create checks/caches host callbacks and allocates a local error cell once.
    pub unsafe fn connect(
        api: *const HostApiV1,
        context: ContextHandle,
        required: u64,
    ) -> Result<Self, Status> {
        if api.is_null()
            || !api.addr().is_multiple_of(align_of::<HostApiV1>())
            || context.value == 0
        {
            return Err(super::status(status_code::INVALID_ARGUMENT));
        }
        // SAFETY (U05): the caller guarantees a live immutable fixed header;
        // non-null/alignment are checked before reading it. No full table yet.
        let header = unsafe { api.cast::<TableHeaderV1>().read() };
        validate_header::<HostApiV1>(&header)
            .map_err(|_| super::status(status_code::INVALID_ARGUMENT))?;
        // Read required function addresses as integers BEFORE forming a typed
        // HostApi value: a null Rust function pointer is an invalid value.
        let offsets = [
            offset_of!(HostApiV1, bind_port),
            offset_of!(HostApiV1, loan_output),
            offset_of!(HostApiV1, finish_output),
            offset_of!(HostApiV1, submit_spill),
            offset_of!(HostApiV1, discard_output),
            offset_of!(HostApiV1, receive_input),
            offset_of!(HostApiV1, release_input),
            offset_of!(HostApiV1, input_head),
            offset_of!(HostApiV1, set_snapshot_inputs),
            offset_of!(HostApiV1, snapshot_inputs),
            offset_of!(HostApiV1, pump_history),
            offset_of!(HostApiV1, clock),
            offset_of!(HostApiV1, request_shutdown),
            offset_of!(HostApiV1, environment),
            offset_of!(HostApiV1, poll_event),
            offset_of!(HostApiV1, publisher_counters),
            offset_of!(HostApiV1, copy_error),
            offset_of!(HostApiV1, diagnostic),
            offset_of!(HostApiV1, accumulate_count),
            offset_of!(HostApiV1, accumulate_frame),
            offset_of!(HostApiV1, mark_served),
            offset_of!(HostApiV1, capture_accumulate),
            offset_of!(HostApiV1, query_extension),
            offset_of!(HostApiV1, restore_accumulate),
            offset_of!(HostApiV1, create_raw_publisher),
            offset_of!(HostApiV1, finish_raw_output),
            offset_of!(HostApiV1, register_raw_egress),
            offset_of!(HostApiV1, destroy_raw_publisher),
        ];
        for offset in offsets {
            // SAFETY (U05): complete byte size/identity were checked; fn fields
            // have pointer-sized initialized representations on this target.
            let address = unsafe { api.cast::<u8>().add(offset).cast::<usize>().read() };
            if address == 0 {
                return Err(super::status(status_code::INVALID_ARGUMENT));
            }
        }
        // SAFETY (U05): complete shape and every fn representation checked;
        // host backing remains immutable. Copy only the C records/callbacks.
        let api = unsafe { api.read() };
        negotiate(required, api.capabilities)
            .map_err(|_| super::status(status_code::UNSUPPORTED))?;
        Ok(Self(Arc::new(ClientInner {
            api,
            context,
            failure: Mutex::new(None),
        })))
    }

    pub(super) fn check(&self, status: Status) -> TransportResult<()> {
        if status.code == status_code::OK {
            return Ok(());
        }
        Err(self.status_error(status))
    }

    // hot-path-alloc-ok-fn: formats/copies diagnostic text only after a returned boundary failure.
    fn status_error(&self, status: Status) -> TransportError {
        let reason = self.error_text(status.error).unwrap_or_else(|| {
            format!(
                "host operation failed with status {} and error token {}",
                status.code, status.error
            )
        });
        if status.code == status_code::PANICKED {
            return TransportError::NodeTickPanicked {
                node_id: "host".into(),
                reason,
            };
        }
        match NodeErrorKind::from_code(status.kind) {
            Some(kind) => TransportError::NodeFailed {
                node_id: "host".into(),
                kind,
                reason,
            },
            None => TransportError::NodeError {
                node_id: "host".into(),
                reason,
            },
        }
    }

    // hot-path-alloc-ok-fn: diagnostic text is copied only after an error, never on successful delivery.
    fn error_text(&self, token: u64) -> Option<String> {
        if token == 0 {
            return None;
        }
        let mut required = 0;
        let status = host_call!(
            self,
            copy_error,
            token,
            MutableBytes {
                ptr: std::ptr::null_mut(),
                len: 0
            },
            &mut required
        );
        if !matches!(status.code, status_code::OK | status_code::BUFFER_TOO_SMALL) {
            return None;
        }
        let len = usize::try_from(required)
            .ok()
            .filter(|len| *len <= isize::MAX as usize)?;
        let mut storage = vec![0_u8; len];
        let status = host_call!(
            self,
            copy_error,
            token,
            mutable_bytes(&mut storage),
            &mut required
        );
        if status.code != status_code::OK || required > storage.len() as u64 {
            return None;
        }
        storage.truncate(required as usize);
        String::from_utf8(storage).ok()
    }

    pub fn record_failure(&self, error: TransportError) {
        if let Ok(mut failure) = self.0.failure.lock() {
            if failure.is_none() {
                *failure = Some(error);
            }
        }
    }

    pub fn take_failure(&self) -> Option<TransportError> {
        self.0
            .failure
            .lock()
            .ok()
            .and_then(|mut failure| failure.take())
    }

    /// Whether a prior host service failed during the current node call.
    pub fn has_failure(&self) -> bool {
        self.0
            .failure
            .lock()
            .map_or(true, |failure| failure.is_some())
    }

    pub(super) fn bind_port(&self, index: u32, direction: u32) -> TransportResult<PortBindingV1> {
        let mut binding = PortBindingV1 {
            port: PortHandle::default(),
            topic: bytes_span(&[]),
            configured_capacity: 0,
            read_mode: 0,
        };
        self.check(host_call!(self, bind_port, index, direction, &mut binding))?;
        if binding.port.value == 0 {
            return Err(super::argument_error("host bound an invalid port handle"));
        }
        Ok(binding)
    }

    pub(super) fn loan_output<T: ShmMessage>(
        &self,
        port: PortHandle,
    ) -> TransportResult<WriteSlotV1> {
        self.loan_frame(port, Self::request::<T>())
    }

    pub(super) fn loan_frame(
        &self,
        port: PortHandle,
        request: LoanRequestV1,
    ) -> TransportResult<WriteSlotV1> {
        let mut slot = WriteSlotV1 {
            data: std::ptr::null_mut(),
            lease: LeaseHandle::default(),
            schema_hash: 0,
            capacity: 0,
            configured_capacity: 0,
            alignment: 0,
            prefix_size: 0,
        };
        self.check(host_call!(self, loan_output, port, request, &mut slot))?;
        if slot.lease.value == 0 {
            return Err(super::argument_error("host loan returned an invalid lease"));
        }
        Ok(slot)
    }

    /// Shared compile-checked wire facts for cold metadata and adaptive loans.
    pub fn request<T: ShmMessage>() -> LoanRequestV1 {
        let _: () = T::_SHM_INVARIANTS;
        LoanRequestV1 {
            schema_hash: T::SCHEMA_HASH,
            minimum_capacity: (WireHeader::SIZE + T::WIRE_FIXED_SIZE + 8 * T::VARIABLE_FIELD_COUNT)
                as u32,
            wire_fixed_size: T::WIRE_FIXED_SIZE as u32,
            variable_count: T::VARIABLE_FIELD_COUNT as u32,
            alignment: crate::shm_runtime::FRAME_ALIGN as u32,
        }
    }

    pub(super) fn finish_output(&self, lease: LeaseHandle, size: u32) {
        let status = host_call!(self, finish_output, lease, size);
        if let Err(error) = self.check(status) {
            self.record_failure(error);
        }
    }

    pub(super) fn discard_output(&self, lease: LeaseHandle, reason: u32) {
        let status = host_call!(self, discard_output, lease, reason);
        if let Err(error) = self.check(status) {
            self.record_failure(error);
        }
    }

    pub(super) fn submit_spill(&self, lease: LeaseHandle, payload: &[u8]) {
        let Some(size) = payload
            .len()
            .checked_add(WireHeader::SIZE)
            .and_then(|len| u32::try_from(len).ok())
        else {
            self.record_failure(super::argument_error("spill size is not a wire-frame size"));
            return;
        };
        let status = host_call!(self, submit_spill, lease, bytes_span(payload), size);
        if let Err(error) = self.check(status) {
            self.record_failure(error);
        }
    }

    pub(super) fn create_raw(
        &self,
        request: RawPublisherRequestV1,
    ) -> TransportResult<PortBindingV1> {
        let mut binding = PortBindingV1 {
            port: PortHandle::default(),
            topic: bytes_span(&[]),
            configured_capacity: 0,
            read_mode: 0,
        };
        self.check(host_call!(
            self,
            create_raw_publisher,
            request,
            &mut binding
        ))?;
        if binding.port.value == 0 {
            return Err(super::argument_error(
                "host raw publisher handle is invalid",
            ));
        }
        Ok(binding)
    }
    pub(super) fn finish_raw(&self, lease: LeaseHandle, size: u32) -> TransportResult<usize> {
        let mut recipients = 0;
        self.check(host_call!(
            self,
            finish_raw_output,
            lease,
            size,
            &mut recipients
        ))?;
        usize::try_from(recipients)
            .map_err(|_| super::argument_error("raw recipient count exceeds usize"))
    }
    pub(super) fn register_raw(&self, port: PortHandle) -> TransportResult<()> {
        // Explicitly fallible: network registration failure must leave local ingress usable.
        self.check(host_call!(self, register_raw_egress, port))
    }
    pub(super) fn destroy_raw(&self, port: PortHandle) {
        if let Err(error) = self.check(host_call!(self, destroy_raw_publisher, port)) {
            self.record_failure(error);
        }
    }

    pub(super) fn receive_input(
        &self,
        port: PortHandle,
        mode: u32,
    ) -> TransportResult<Option<ReadSlotV1>> {
        let mut slot = empty_read_slot();
        let status = host_call!(self, receive_input, port, mode, &mut slot);
        if status.code == status_code::EMPTY {
            return Ok(None);
        }
        self.check(status)?;
        if slot.lease.value == 0 {
            return Err(super::argument_error("host read returned an invalid lease"));
        }
        Ok(Some(slot))
    }

    pub(super) fn accumulate_frame(
        &self,
        port: PortHandle,
        index: u32,
    ) -> TransportResult<ReadSlotV1> {
        let mut slot = empty_read_slot();
        self.check(host_call!(self, accumulate_frame, port, index, &mut slot))?;
        if slot.lease.value == 0 {
            return Err(super::argument_error(
                "host accumulation returned an invalid lease",
            ));
        }
        if let Err(error) = self.check(host_call!(self, mark_served, slot.lease)) {
            self.release_read(slot.lease);
            return Err(error);
        }
        Ok(slot)
    }

    pub(super) fn release_read(&self, lease: LeaseHandle) {
        let status = host_call!(self, release_input, lease);
        if let Err(error) = self.check(status) {
            self.record_failure(error);
        }
    }

    fn clock(&self, source: u32) -> Option<u64> {
        let mut value = ClockValueV1::default();
        let status = host_call!(self, clock, source, &mut value);
        if let Err(error) = self.check(status) {
            self.record_failure(error);
            return None;
        }
        match (value.present, value.reserved) {
            (0, 0) => None,
            (1, 0) => Some(value.timestamp_ns),
            _ => {
                self.record_failure(super::argument_error("host clock presence is invalid"));
                None
            }
        }
    }

    pub fn clock_now(&self) -> u64 {
        self.clock(clock_source::ACTIVE).unwrap_or_else(|| {
            self.record_failure(super::argument_error("active clock is absent"));
            0
        })
    }
    pub fn clock_real(&self) -> u64 {
        self.clock(clock_source::REAL).unwrap_or_else(|| {
            self.record_failure(super::argument_error("real clock is absent"));
            0
        })
    }
    pub fn clock_virtual(&self) -> Option<u64> {
        self.clock(clock_source::VIRTUAL)
    }
    pub fn clock_external(&self) -> Option<u64> {
        self.clock(clock_source::EXTERNAL)
    }

    pub fn request_shutdown(&self) {
        let status = host_call!(self, request_shutdown);
        if let Err(error) = self.check(status) {
            self.record_failure(error);
        }
    }

    /// Emit a bounded borrowed diagnostic. Zero port is node-wide; levels are
    /// 1 error, 2 warn, 3 info, 4 debug and 5 trace.
    pub fn diagnostic(&self, port: PortHandle, level: u32, text: &[u8]) {
        let status = host_call!(self, diagnostic, port, level, bytes_span(text));
        if let Err(error) = self.check(status) {
            self.record_failure(error);
        }
    }

    // hot-path-alloc-ok-fn: preserves author String environment lookup, used during node init.
    pub(super) fn environment(&self, key: &str) -> Option<String> {
        let mut required = 0;
        let status = host_call!(
            self,
            environment,
            bytes_span(key.as_bytes()),
            mutable_bytes(&mut []),
            &mut required
        );
        if status.code == status_code::EMPTY {
            return None;
        }
        if !matches!(status.code, status_code::OK | status_code::BUFFER_TOO_SMALL) {
            self.record_failure(self.status_error(status));
            return None;
        }
        let Some(len) = usize::try_from(required)
            .ok()
            .filter(|len| *len <= isize::MAX as usize)
        else {
            self.record_failure(super::argument_error(
                "environment value exceeds slice bounds",
            ));
            return None;
        };
        let mut storage = vec![0_u8; len];
        let status = host_call!(
            self,
            environment,
            bytes_span(key.as_bytes()),
            mutable_bytes(&mut storage),
            &mut required
        );
        if let Err(error) = self.check(status) {
            self.record_failure(error);
            return None;
        }
        if required > storage.len() as u64 {
            self.record_failure(super::argument_error("environment buffer length changed"));
            return None;
        }
        storage.truncate(required as usize);
        match String::from_utf8(storage) {
            Ok(value) => Some(value),
            Err(_) => {
                self.record_failure(super::argument_error("environment value is not UTF-8"));
                None
            }
        }
    }

    pub(super) fn poll_event(
        &self,
        port: PortHandle,
        kind: u32,
    ) -> TransportResult<Option<EventV1>> {
        let mut event = EventV1::default();
        let status = host_call!(self, poll_event, port, kind, &mut event);
        if status.code == status_code::EMPTY {
            return Ok(None);
        }
        self.check(status)?;
        if event.port != port || event.kind != kind {
            return Err(super::argument_error("host event port or kind is invalid"));
        }
        Ok(Some(event))
    }

    pub(super) fn accumulate_count(&self, port: PortHandle) -> TransportResult<AccumulateInfoV1> {
        let mut info = AccumulateInfoV1::default();
        self.check(host_call!(self, accumulate_count, port, &mut info))?;
        if info.reserved != 0 {
            return Err(super::argument_error(
                "accumulate reserved value is nonzero",
            ));
        }
        Ok(info)
    }

    pub(super) fn capture_accumulate(
        &self,
        port: PortHandle,
        sink: &StateSinkV1,
    ) -> TransportResult<()> {
        self.check(host_call!(self, capture_accumulate, port, sink))
    }

    pub(super) fn restore_accumulate(
        &self,
        port: PortHandle,
        payload: &[u8],
    ) -> TransportResult<()> {
        self.check(host_call!(
            self,
            restore_accumulate,
            port,
            bytes_span(payload)
        ))
    }
}

pub(super) fn bytes_span(bytes: &[u8]) -> Bytes {
    Bytes {
        ptr: bytes.as_ptr(),
        len: bytes.len() as u64,
    }
}
pub(super) fn mutable_bytes(bytes: &mut [u8]) -> MutableBytes {
    MutableBytes {
        ptr: bytes.as_mut_ptr(),
        len: bytes.len() as u64,
    }
}
fn empty_read_slot() -> ReadSlotV1 {
    ReadSlotV1 {
        data: std::ptr::null(),
        lease: LeaseHandle::default(),
        schema_hash: 0,
        size: 0,
        alignment: 0,
    }
}
