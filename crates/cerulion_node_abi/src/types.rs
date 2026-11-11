// SPDX-License-Identifier: AGPL-3.0-only
use stabby::IStable;

/// Host-owned node context; zero is invalid. Never encodes a pointer.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ContextHandle {
    /// Owner-local identifier.
    pub value: u64,
}

/// Node-owned instance; zero is invalid. Never encodes a pointer.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NodeHandle {
    /// Owner-local identifier.
    pub value: u64,
}

/// Host-owned port scoped to a context; zero is invalid. Never encodes a pointer.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PortHandle {
    /// Owner-local identifier.
    pub value: u64,
}

/// Host-owned sample lease with a reuse generation; zero is invalid. Never encodes a pointer.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LeaseHandle {
    /// Owner-local identifier.
    pub value: u64,
}

/// Immutable bytes borrowed for one synchronous call, or library-lifetime
/// metadata when stored in NodeApiV1. Null is permitted only for an empty span.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Bytes {
    /// Owner retains allocation and release responsibility.
    pub ptr: *const u8,
    /// Initialized bytes, bounded by the actual allocation and isize::MAX.
    pub len: u64,
}

/// Caller-owned writable storage; callee retains no pointer after returning.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct MutableBytes {
    /// Exclusive writable allocation.
    pub ptr: *mut u8,
    /// Available bytes, bounded by the actual allocation and isize::MAX.
    pub len: u64,
}

/// Scalar result; the error token names an owner-local diagnostic, never memory.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Status {
    /// Diagnostic token, scoped to the context or instance that returned it.
    pub error: u64,
    /// Operation result from status_code.
    pub code: u32,
    /// Existing NodeErrorKind wire code, or zero for an untyped failure.
    pub kind: u32,
}

/// Frozen operation result codes. Empty is meaningful only for read/event polls.
pub mod status_code {
    /// Operation completed.
    pub const OK: u32 = 0;
    /// No input frame or event exists.
    pub const EMPTY: u32 = 1;
    /// Node or transport failure.
    pub const FAILED: u32 = 2;
    /// Panic contained on its originating side.
    pub const PANICKED: u32 = 3;
    /// Zero, stale, foreign or wrong-direction handle.
    pub const INVALID_HANDLE: u32 = 4;
    /// Malformed arguments, frame or descriptor.
    pub const INVALID_ARGUMENT: u32 = 5;
    /// Borrow budget exhausted or instance busy.
    pub const BUSY: u32 = 6;
    /// Required capability unavailable.
    pub const UNSUPPORTED: u32 = 7;
    /// State sink or caller buffer refused the write; pending data is retained.
    pub const BUFFER_TOO_SMALL: u32 = 8;
}

/// Every table starts with this header; its full type is immutable within major 1.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TableHeaderV1 {
    /// Full Stabby report identity including target facts and every field.
    pub layout_id: u64,
    /// Exact table size; prefixes are never accepted as complete tables.
    pub size_bytes: u32,
    /// Boundary major, independent of the runtime release version.
    pub boundary_major: u32,
}

impl TableHeaderV1 {
    /// Construct a header for the complete checked table type.
    pub const fn for_type<T: IStable>() -> Self {
        Self {
            layout_id: T::ID,
            size_bytes: core::mem::size_of::<T>() as u32,
            boundary_major: crate::BOUNDARY_MAJOR,
        }
    }
}

/// Node-local schema facts needed to initialize a host-owned loan prefix.
/// Host verifies schema and all lengths against the wired port before loaning.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LoanRequestV1 {
    /// Generated message schema hash.
    pub schema_hash: u64,
    /// Minimum complete frame capacity, including header.
    pub minimum_capacity: u32,
    /// Generated fixed payload bytes, excluding header.
    pub wire_fixed_size: u32,
    /// Generated offset-table entry count.
    pub variable_count: u32,
    /// Required frame alignment.
    pub alignment: u32,
}

/// A host-owned initialized output frame, exclusively borrowed until finish.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct WriteSlotV1 {
    /// Real shared-memory frame base, including the existing wire header.
    pub data: *mut u8,
    /// Host lease retaining the native loan.
    pub lease: LeaseHandle,
    /// Port schema identity.
    pub schema_hash: u64,
    /// Actual adaptive loan capacity, including header.
    pub capacity: u32,
    /// Configured frame ceiling, which may exceed adaptive capacity.
    pub configured_capacity: u32,
    /// Required frame alignment.
    pub alignment: u32,
    /// Header, fixed fields and offset-table prefix bytes initialized by host.
    pub prefix_size: u32,
}

/// A host-owned immutable frame, borrowed until release.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ReadSlotV1 {
    /// Real shared-memory or host-held bounded accumulator frame base.
    pub data: *const u8,
    /// Host lease retaining the frame owner.
    pub lease: LeaseHandle,
    /// Validated schema identity.
    pub schema_hash: u64,
    /// Complete initialized wire-frame bytes.
    pub size: u32,
    /// Guaranteed frame alignment.
    pub alignment: u32,
}

/// Explicit output discard classifications; host counters/latches are authority.
pub mod discard_reason {
    /// User tick returned an error or did not complete.
    pub const TICK_FAILED: u32 = 1;
    /// User tick panicked.
    pub const TICK_PANICKED: u32 = 2;
    /// A declared variable field was not written.
    pub const MISSING_VARIABLE: u32 = 3;
    /// Nested-field completion or flush failed.
    pub const NESTED_FAILED: u32 = 4;
    /// Replay suppressed this output.
    pub const REPLAY_SUPPRESSED: u32 = 5;
}

/// Port direction tags.
pub mod direction {
    /// Subscriber input.
    pub const INPUT: u32 = 0;
    /// Publisher output.
    pub const OUTPUT: u32 = 1;
}

/// Input read modes; the host remains the only queue/read-outcome authority.
pub mod read_mode {
    /// Drain according to the existing latest-value rule.
    pub const LATEST: u32 = 0;
    /// Consume one admitted queue frame.
    pub const ONE: u32 = 1;
    /// Serve the frozen non-trigger frame without an additional queue read.
    pub const HELD: u32 = 2;
}

/// Synchronization head operations.
pub mod head_op {
    /// Inspect without consuming.
    pub const PEEK: u32 = 0;
    /// Promote the staged head.
    pub const PROMOTE: u32 = 1;
    /// Skip the staged head.
    pub const SKIP: u32 = 2;
    /// Drain trigger frames using existing policy.
    pub const DRAIN: u32 = 3;
    /// Refill at most one head.
    pub const REFILL: u32 = 4;
}

/// Bounded read/head observations; presence is explicit, never `Option<T>`.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct HeadOutcomeV1 {
    /// Number of popped frames.
    pub popped: u64,
    /// Observed wire timestamp if has_timestamp is one.
    pub timestamp_ns: u64,
    /// Zero or one.
    pub has_timestamp: u32,
    /// Owner-defined existing read/head outcome kind.
    pub kind: u32,
}

/// Source-specific clock access, retaining explicit real-time escape semantics.
pub mod clock_source {
    /// Injected active graph Clock.
    pub const ACTIVE: u32 = 0;
    /// VirtualClock only.
    pub const VIRTUAL: u32 = 1;
    /// ExternalClock only.
    pub const EXTERNAL: u32 = 2;
    /// Existing explicit real_ns escape; never an implicit execution default.
    pub const REAL: u32 = 3;
}

/// Clock result with explicit source presence.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ClockValueV1 {
    /// Timestamp for the requested source.
    pub timestamp_ns: u64,
    /// Zero or one.
    pub present: u32,
    /// Always zero; reserved storage cannot gain meaning within this type.
    pub reserved: u32,
}

/// Scalar QoS/backpressure observation, ordered by the host's existing stores.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct EventV1 {
    /// Port responsible for the event.
    pub port: PortHandle,
    /// Graph-clock timestamp.
    pub timestamp_ns: u64,
    /// Cumulative observed count.
    pub total: u64,
    /// Regime-specific count.
    pub in_regime: u64,
    /// Frames dropped.
    pub dropped: u64,
    /// Effective buffer capacity.
    pub capacity: u64,
    /// Existing event kind.
    pub kind: u32,
    /// Existing event policy/state tag.
    pub detail: u32,
}

/// Snapshot of observable publisher counters.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct PublisherCountersV1 {
    /// Committed frames, independent of replay's sequence origin.
    pub committed: u64,
    /// Discarded outputs.
    pub discarded: u64,
    /// Send failures after sequence commitment.
    pub send_failures: u64,
}

/// Capability bits refer to complete immutable extension tables, never versions.
pub mod capability {
    /// Canonical state capture/restore.
    pub const STATE: u64 = 1;
    /// External ingress source description.
    pub const EXTERNAL: u64 = 2;
    /// Bounded accumulator frame and state services.
    pub const ACCUMULATE: u64 = 4;
    /// All currently known optional capabilities.
    pub const KNOWN: u64 = STATE | EXTERNAL | ACCUMULATE;
}

/// Cold discovery of an independently versioned immutable extension table.
/// Validate full size and layout identity before converting the byte pointer.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ExtensionDescriptorV1 {
    /// Owner's immutable table storage, valid for its library/process lifetime.
    pub table: *const u8,
    /// Extension identifier; unknown optional IDs are ignored without casting.
    pub id: u64,
    /// Complete Stabby layout identity of the concrete extension.
    pub layout_id: u64,
    /// Complete table bytes.
    pub size_bytes: u32,
    /// Exact immutable extension version.
    pub version: u32,
}
