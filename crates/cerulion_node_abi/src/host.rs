// SPDX-License-Identifier: AGPL-3.0-only
use crate::*;

/// Host process-lifetime services. Handles are context-scoped and generation
/// checked. All calls contain panics before returning across C. Output spans are
/// borrowed only for the call; no owner allocation is freed on the other side.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy)]
pub struct HostApiV1 {
    /// Complete immutable table identity.
    pub header: TableHeaderV1,
    /// Services available for negotiation.
    pub capabilities: u64,
    /// Resolve declaration-order port index and verify direction at cold init.
    pub bind_port: unsafe extern "C" fn(ContextHandle, u32, u32, *mut PortHandle) -> Status,
    /// Loan adaptive capacity, initialize prefix, retain native loan on host.
    pub loan_output:
        unsafe extern "C" fn(ContextHandle, PortHandle, LoanRequestV1, *mut WriteSlotV1) -> Status,
    /// Commit initialized frame bytes in an existing loan. Consumes lease once.
    pub finish_output: unsafe extern "C" fn(ContextHandle, LeaseHandle, u32) -> Status,
    /// Copy a node-owned initialized overflow PAYLOAD into a fresh host loan.
    /// Header/timestamp come from old lease; size includes header. The caller
    /// retains spill allocation until return. Consumes old lease on every result.
    pub finish_spill: unsafe extern "C" fn(ContextHandle, LeaseHandle, Bytes, u32) -> Status,
    /// Discard without consuming wire sequence; reason drives host counters.
    pub discard_output: unsafe extern "C" fn(ContextHandle, LeaseHandle, u32) -> Status,
    /// Receive through the existing queue/frozen read path and retain frame.
    pub receive_input:
        unsafe extern "C" fn(ContextHandle, PortHandle, u32, *mut ReadSlotV1) -> Status,
    /// End an immutable input borrow; consumes lease once.
    pub release_input: unsafe extern "C" fn(ContextHandle, LeaseHandle) -> Status,
    /// Drain/refill/peek/promote/skip through existing host read decisions.
    pub input_head:
        unsafe extern "C" fn(ContextHandle, PortHandle, u32, *mut HeadOutcomeV1) -> Status,
    /// Freeze non-trigger inputs; JSON names are copied only during cold setup.
    pub set_snapshot_inputs: unsafe extern "C" fn(ContextHandle, Bytes) -> Status,
    /// Freeze the configured inputs at the existing step boundary.
    pub snapshot_inputs: unsafe extern "C" fn(ContextHandle) -> Status,
    /// Perform existing late-joiner/history maintenance.
    pub pump_history: unsafe extern "C" fn(ContextHandle) -> Status,
    /// Read active/virtual/external/explicit-real clock without transferring Clock.
    pub clock: unsafe extern "C" fn(ContextHandle, u32, *mut ClockValueV1) -> Status,
    /// Idempotent request, observable independently of tick execution.
    pub request_shutdown: unsafe extern "C" fn(ContextHandle) -> Status,
    /// Copy frozen environment value; insufficient buffer retains the value.
    /// Empty status means absent key; required length is returned in all cases.
    pub environment: unsafe extern "C" fn(ContextHandle, Bytes, MutableBytes, *mut u64) -> Status,
    /// Poll an existing event kind, in host declaration order.
    pub poll_event: unsafe extern "C" fn(ContextHandle, u32, *mut EventV1) -> Status,
    /// Host-owned observable counters, with no formatted hot-path text.
    pub publisher_counters:
        unsafe extern "C" fn(ContextHandle, PortHandle, *mut PublisherCountersV1) -> Status,
    /// Copy diagnostic for token; short-buffer refusal does not consume it.
    pub copy_error: unsafe extern "C" fn(ContextHandle, u64, MutableBytes, *mut u64) -> Status,
    /// Report a structured node-side diagnostic; host suppresses repeated regimes.
    pub diagnostic: unsafe extern "C" fn(ContextHandle, PortHandle, u32, Bytes) -> Status,
    /// Bounded accumulation count and drops; no AccumulatorSet crosses.
    pub accumulate_count:
        unsafe extern "C" fn(ContextHandle, PortHandle, *mut u32, *mut u64) -> Status,
    /// Borrow an indexed accumulated frame; consumes no queue frame.
    pub accumulate_frame:
        unsafe extern "C" fn(ContextHandle, PortHandle, u32, *mut ReadSlotV1) -> Status,
    /// Record that a borrowed accumulated frame was served, once per lease.
    pub mark_served: unsafe extern "C" fn(ContextHandle, LeaseHandle) -> Status,
    /// Capture canonical accumulator bytes into a synchronous borrowed sink.
    pub capture_accumulate:
        unsafe extern "C" fn(ContextHandle, PortHandle, *const StateSinkV1) -> Status,
    /// Query a separately versioned service; unknown IDs return unsupported.
    pub query_extension:
        unsafe extern "C" fn(ContextHandle, u64, u32, *mut ExtensionDescriptorV1) -> Status,
    /// Restore canonical accumulator bytes, validated before mutation.
    pub restore_accumulate: unsafe extern "C" fn(ContextHandle, PortHandle, Bytes) -> Status,
}
