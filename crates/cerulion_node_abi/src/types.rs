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
/// metadata when stored in NodeApiV1, or context-lifetime topics in PortBindingV1.
/// Null is permitted only for an empty span.
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

/// A host-owned initialized output frame, exclusively borrowed until its
/// terminal finish/discard. Spill submission alone never ends this borrow.
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
    /// Release original spill loan after writer destruction, without discard accounting.
    pub const SPILL_RELEASE: u32 = 6;
    /// Malformed writer/slot contract; records an invariant drop.
    pub const INVARIANT: u32 = 7;
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

/// Cold binding facts; topic storage remains immutable for the context lifetime.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PortBindingV1 {
    /// Context-scoped port identifier.
    pub port: PortHandle,
    /// Resolved UTF-8 topic; node copies it into node-owned storage at create.
    pub topic: Bytes,
    /// Complete frame ceiling for outputs, zero for inputs.
    pub configured_capacity: u32,
    /// Exact host input classification from read_mode; ignored for outputs.
    pub read_mode: u32,
}

/// Synchronization operations retain the caller's boundary/refill read site.
pub mod head_op {
    /// Fill or re-offer a head at the level boundary.
    pub const FILL_BOUNDARY: u32 = 0;
    /// Fill between fires; never re-offer an unread frozen head.
    pub const FILL_REFILL: u32 = 1;
    /// Non-consuming arrival probe behind the head or on a latest input.
    pub const PROBE_NEXT: u32 = 2;
    /// Stage the next frame at the boundary read site.
    pub const PEEK_BOUNDARY: u32 = 3;
    /// Stage the next frame at the between-fire read site.
    pub const PEEK_REFILL: u32 = 4;
    /// Discard/refill the head at the boundary read site.
    pub const ADVANCE_BOUNDARY: u32 = 5;
    /// Discard/refill the head at the between-fire read site.
    pub const ADVANCE_REFILL: u32 = 6;
    /// Serve a restored head as no live frame.
    pub const VOID: u32 = 7;
    /// Existing trigger drain at the level boundary.
    pub const DRAIN: u32 = 8;
    /// Existing trigger refill between fires.
    pub const REFILL: u32 = 9;
}

/// Exact scalar answers to native synchronization and trigger drain operations.
pub mod head_kind {
    /// No head/frame exists; no timestamp.
    pub const NOTHING: u32 = 0;
    /// Probe found another frame; stamp unknown.
    pub const PRESENT: u32 = 1;
    /// Peek returned a staged frame's timestamp.
    pub const STAMP: u32 = 2;
    /// Fill/advance produced a head, or drain returned its latest timestamp.
    pub const HEAD: u32 = 3;
    /// Failed operation; never evidence enabling alignment descent.
    pub const FAILED: u32 = 4;
    /// Empty drain/refill deliberately withheld by the replay read plan.
    pub const WITHHELD: u32 = 5;
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
    /// Frozen head_kind answer; timestamp presence is one only for STAMP/HEAD.
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

/// Existing author event kinds, polled only for the specified port.
pub mod event_kind {
    /// Backpressure regime observation.
    pub const BACKPRESSURE: u32 = 1;
    /// Input watchdog miss.
    pub const EXPECT_WITHIN: u32 = 2;
    /// Output watchdog miss.
    pub const PROMISE_WITHIN: u32 = 3;
    /// Publisher liveliness transition.
    pub const LIVELINESS: u32 = 4;
}

/// Kind-scoped event policy, state and cause tags.
pub mod event_detail {
    /// Backpressure drops the oldest frame.
    pub const DROP_OLDEST: u32 = 0;
    /// Backpressure blocks.
    pub const BLOCK: u32 = 1;
    /// Backpressure samples at policy_value milliseconds.
    pub const SAMPLE: u32 = 2;
    /// Liveliness state is alive.
    pub const ALIVE: u32 = 1;
    /// Liveliness state is lost.
    pub const LOST: u32 = 2;
    /// Liveliness cause is publisher connected.
    pub const CONNECTED: u32 = 1;
    /// Liveliness cause is publisher disconnected.
    pub const DISCONNECTED: u32 = 2;
}

/// Scalar QoS/backpressure observation, ordered by the host's existing stores.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct EventV1 {
    /// Port responsible for the event.
    pub port: PortHandle,
    /// Original regime_started_at_ns, missed_at_ns or changed_at_ns.
    pub timestamp_ns: u64,
    /// Cumulative observed count.
    pub total: u64,
    /// Backpressure regime-specific count; zero for other kinds.
    pub in_regime: u64,
    /// Backpressure frames dropped; zero for other kinds.
    pub dropped: u64,
    /// Backpressure buffer_capacity or liveliness publisher_count; otherwise zero.
    pub capacity: u64,
    /// Frozen event_kind tag.
    pub kind: u32,
    /// Kind-scoped backpressure policy or liveliness state from event_detail.
    pub detail: u32,
    /// Configured expect/promise window in milliseconds; otherwise zero.
    pub window_ms: u64,
    /// Original watchdog elapsed milliseconds; otherwise zero.
    pub elapsed_ms: u64,
    /// Backpressure Sample(N)'s interval in milliseconds; otherwise zero.
    pub policy_value: u64,
    /// Liveliness cause from event_detail; otherwise zero.
    pub cause: u32,
}

/// Frozen fire batch observations. Repeated queries in one fire return the same batch.
#[stabby::stabby]
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AccumulateInfoV1 {
    /// Frame count handed to this fire.
    pub count: u32,
    /// Always zero.
    pub reserved: u32,
    /// Frames dropped for this fire.
    pub dropped: u64,
    /// Lifetime dropped total.
    pub dropped_total: u64,
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
