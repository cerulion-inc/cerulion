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
    /// Resolve direction-scoped declaration-order index at cold init. Topic borrows
    /// context-lifetime immutable storage; node copies it only during create.
    pub bind_port: unsafe extern "C" fn(ContextHandle, u32, u32, *mut PortBindingV1) -> Status,
    /// Loan adaptive capacity, initialize prefix, retain native loan on host.
    pub loan_output:
        unsafe extern "C" fn(ContextHandle, PortHandle, LoanRequestV1, *mut WriteSlotV1) -> Status,
    /// Commit initialized frame bytes in an existing loan. Consumes lease once.
    pub finish_output: unsafe extern "C" fn(ContextHandle, LeaseHandle, u32) -> Status,
    /// Copy a node-owned initialized overflow PAYLOAD into a fresh host loan.
    /// Header/timestamp come from old lease; size includes header. The caller
    /// retains spill allocation until return. Original loan stays live on EVERY
    /// result until discard_output(SPILL_RELEASE) after writer destruction.
    /// Records one attempt per lease; repeated submission cannot publish twice.
    pub submit_spill: unsafe extern "C" fn(ContextHandle, LeaseHandle, Bytes, u32) -> Status,
    /// Consume original loan without consuming wire sequence. SPILL_RELEASE only
    /// releases an attempted spill and records no node discard; other reasons
    /// drive existing counters/latches. Never call while its writer is live.
    pub discard_output: unsafe extern "C" fn(ContextHandle, LeaseHandle, u32) -> Status,
    /// Receive through the existing queue/frozen read path and retain frame.
    /// Requested read_mode must match the cold binding classification.
    pub receive_input:
        unsafe extern "C" fn(ContextHandle, PortHandle, u32, *mut ReadSlotV1) -> Status,
    /// End an immutable input borrow; consumes lease once.
    pub release_input: unsafe extern "C" fn(ContextHandle, LeaseHandle) -> Status,
    /// Apply head_op with explicit boundary/refill site through existing decisions.
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
    /// Poll only the named port and event_kind; empty poll consumes no other event.
    pub poll_event: unsafe extern "C" fn(ContextHandle, PortHandle, u32, *mut EventV1) -> Status,
    /// Host-owned observable counters, with no formatted hot-path text.
    pub publisher_counters:
        unsafe extern "C" fn(ContextHandle, PortHandle, *mut PublisherCountersV1) -> Status,
    /// Copy diagnostic for token; short-buffer refusal does not consume it.
    pub copy_error: unsafe extern "C" fn(ContextHandle, u64, MutableBytes, *mut u64) -> Status,
    /// Report a structured node-side diagnostic at diagnostic_level. Zero port
    /// selects a distinct node-wide latch; named ports retain their own regimes.
    pub diagnostic: unsafe extern "C" fn(ContextHandle, PortHandle, u32, Bytes) -> Status,
    /// Hand pending batch to this fire once, returning both drop observations.
    /// Repeated calls during the fire return the same frozen batch; subsequent
    /// fires without new collection return zero count and per-fire drops.
    pub accumulate_count:
        unsafe extern "C" fn(ContextHandle, PortHandle, *mut AccumulateInfoV1) -> Status,
    /// Repeatedly borrow indexed frames of this fire batch; consumes no queue frame.
    pub accumulate_frame:
        unsafe extern "C" fn(ContextHandle, PortHandle, u32, *mut ReadSlotV1) -> Status,
    /// Record read outcomes once per distinct accumulated host frame per fire.
    /// Repeated iteration leases must not duplicate sequence/trace observations.
    pub mark_served: unsafe extern "C" fn(ContextHandle, LeaseHandle) -> Status,
    /// Capture exactly one native per-port record, without the outer input count.
    /// Encoding is specified in the node interface documentation. Sink is a
    /// frozen concrete borrowed record, not a table; validate pointer/alignment.
    pub capture_accumulate:
        unsafe extern "C" fn(ContextHandle, PortHandle, *const StateSinkV1) -> Status,
    /// Query a separately versioned service; unknown IDs return unsupported.
    pub query_extension:
        unsafe extern "C" fn(ContextHandle, u64, u32, *mut ExtensionDescriptorV1) -> Status,
    /// Restore exactly one matching per-port record; reject trailing bytes, wrong
    /// name/schema/capacity/counters before mutation. Caller retains byte storage.
    pub restore_accumulate: unsafe extern "C" fn(ContextHandle, PortHandle, Bytes) -> Status,
    /// Cold create of a bounded host-owned raw-ingress publisher. All topic
    /// bytes are borrowed only for this call; returned topic borrows immutable
    /// context-lifetime storage. Exhausted publisher budget returns BUSY.
    pub create_raw_publisher:
        unsafe extern "C" fn(ContextHandle, RawPublisherRequestV1, *mut PortBindingV1) -> Status,
    /// Consume a raw write lease once, including on failure. Preserve the complete
    /// caller-supplied header, schema and initialized length; never stamp another
    /// sequence/time. Return actual recipients; every error arm writes zero.
    pub finish_raw_output:
        unsafe extern "C" fn(ContextHandle, LeaseHandle, u32, *mut u64) -> Status,
    /// Announce the bound topic/schema for dynamic egress through the existing
    /// local control channel. A refusal leaves local ingress usable.
    pub register_raw_egress: unsafe extern "C" fn(ContextHandle, PortHandle) -> Status,
    /// Cold destroy; outstanding leases return BUSY. Handles are invalidated
    /// without reuse; context teardown drops any remaining native raw ports.
    pub destroy_raw_publisher: unsafe extern "C" fn(ContextHandle, PortHandle) -> Status,
}
