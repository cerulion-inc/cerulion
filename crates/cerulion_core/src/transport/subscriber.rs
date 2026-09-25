// SPDX-License-Identifier: AGPL-3.0-only
//! Event-based zero-copy message subscriber over iceoryx2 shared memory.
//!
//! # Subscribe Flow
//!
//! ```text
//! 1. listener.timed_wait_one(timeout)   → block until event or timeout
//! 2. subscriber.receive() in loop       → drain ALL available samples
//! 3. For each sample:
//!    a. read_from_buf() → owned WireHeader (32 bytes, alignment-safe)
//!    b. &raw[32..] → zero-copy payload slice into shared memory
//!    c. invoke callback with ReceivedMessage
//! ```
//!
//! # Drain-on-Timeout
//!
//! We ALWAYS drain after wake — even on timeout. This handles the send→notify race:
//! if a publisher calls `send()` then `notify()`, the data may arrive before the
//! notification. Draining ensures no data loss (Principle #6).
//!
//! # Bidirectional Events
//!
//! The subscriber creates a Notifier to signal the publisher:
//! - `SubscriberConnected` on creation (triggers history delivery)
//! - `SubscriberDisconnected` on Drop (graceful cleanup)

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use iceoryx2::identifiers::UniquePublisherId;
use iceoryx2::port::listener::Listener;
use iceoryx2::port::notifier::Notifier;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::sample::Sample;

use crate::error::{TransportError, TransportResult};
use crate::graph::node::BackpressurePolicy;
use crate::message::ShmMessage;
use crate::read_outcome::{ReadOutcomeKind, ReadOutcomeStage, ReadSiteRole};
use crate::scheduler::{record_backpressure_event_n, BackpressureCounters, BackpressureEvent};
use crate::wire::WireHeader;

use super::events::PubSubEvent;
use super::input_view::InputView;
use super::shm_sample::SampleHandle;
use super::CerService;

/// `sample(N)`: subscriber-side read-gate state. Accepts at most one
/// read per `interval` ms, keyed off each sample's WIRE timestamp (publish
/// time, baked into the frame) — NOT wall-clock-at-drain, so decimation is
/// replay-deterministic (Principle #7). Zero-copy: rejecting a too-soon
/// sample just drops it (releasing its SHM slot); nothing is buffered.
struct SampleGate {
    /// Minimum spacing between accepted reads (ns); `interval_ms * 1e6`.
    interval_ns: u64,
    /// `interval_ms` retained for the structured warn/counter routing.
    interval_ms: u64,
    /// Wire timestamp of the last ACCEPTED sample, or `None` until the
    /// first accept.
    last_accepted_ns: Option<u64>,
    /// Per-(consumer, input) counters — a decimated read bumps
    /// `sampled_count`, visible via the consumer's `NodeHandle`.
    counters: Arc<BackpressureCounters>,
    /// Consumer node id (structured-warn attribution).
    node_id: Arc<str>,
    /// Consumer input field name (structured-warn attribution).
    input: Arc<str>,
    /// Edge-trigger latch for the `#[on_event]`
    /// `BackpressureEvent`. `true` after every accept; the FIRST decimate in a regime
    /// queues one [`BackpressureEvent`] and disarms — subsequent decimates
    /// in the same regime are silent until the next accept rearms (decision
    /// J: one event per regime).
    armed: bool,
    /// Wire timestamp when the current decimation regime started.
    regime_started_ns: u64,
    /// Decimation count at regime OPEN (always 1 — the first decimate).
    /// Events fire only at regime open, which is the only point this is
    /// read; decimations while disarmed bump `sampled_count` but no
    /// per-regime state (deliberate: a disarmed `+= 1` would be dead
    /// state no event could ever surface).
    regime_count: u64,
    /// Configured subscriber buffer capacity (carried on the event for
    /// user recovery decisions).
    buffer_capacity: usize,
}

/// `drop_oldest`: detect iceoryx2's silent oldest-eviction (queue
/// overflow reclaim) on this subscriber via per-publisher wire-sequence
/// gaps, so `drop_oldest_count` is real and an
/// `#[on_event(input = "...")]` handler (with a `BackpressureEvent` parameter) on a `drop_oldest` input actually fires.
/// Installed on every input the runtime wires as `drop_oldest` (the default
/// policy). Zero-copy: the detector reads the 4-byte wire `sequence` from
/// frames already in the drain path plus iceoryx2's own
/// [`Sample::origin`](iceoryx2::sample::Sample) publisher id (no wire-format
/// change, no extra FFI).
///
/// **Exact per publisher stream**: the wire sequence is
/// per-publisher monotonic, and the origin id pins each gap to one
/// publisher's stream, so each stream's missing-slot count is its exact
/// eviction count — summed across streams per drain, with no cap and no
/// saturation. Multi-publisher topics count exactly per id; a publisher
/// restart simply starts a new stream (its first observation
/// baseline-establishes, uncounted — the stream's prior history is
/// unknowable). Same-id BACKWARD sequences — history replay to a late
/// joiner — are duplicate re-delivery, never counted (see
/// [`GapClass::Backward`]); corrupt or errored drains reset ALL baselines
/// and may under-report one interval.
struct DropOldestProbe {
    /// Per-publisher baselines: (origin id, highest wire sequence
    /// consumed, drain counter when the stream was last SEEN). Empty until
    /// the first drain (and re-emptied by a corrupt drain — or an ERRORED
    /// drain that actually popped frames: both consume frames the probe
    /// never observed; an error before the first pop keeps the still-valid
    /// baselines). Per stream, the gap between its sequence and the next
    /// drain's lowest still-queued sequence is exactly how many samples
    /// iceoryx2 evicted from that stream.
    ///
    /// Pre-sized to `max_baselines` (find-or-insert by linear scan — the
    /// publisher count is small). At capacity, inserting a new stream
    /// evicts the not-seen-this-drain entry with the OLDEST last-seen
    /// stamp: dead streams stop refreshing forever while a live-but-quiet
    /// stream refreshes on every appearance, so the victim is the dead
    /// stream except when a dead stream out-published a live one's entire
    /// quiet period (then the displaced live stream re-establishes,
    /// uncounted for one window — a safe under-report, breadcrumbed). If
    /// every entry was seen this drain, the set grows instead of
    /// miscounting (dead publishers' still-queued frames can transiently
    /// exceed `max_publishers` distinct ids in one drain).
    baselines: Vec<(UniquePublisherId, u32, u64)>,
    /// Baseline capacity = the topic's iceoryx2 `max_publishers`
    /// (concurrent publishers are bounded by it, so a new id implies a
    /// dead slot exists — though not WHICH baseline entry is the dead one;
    /// see the eviction-stamp rationale on `baselines`).
    max_baselines: usize,
    /// Monotonic per-probe counter of drains with ≥1 readable frame;
    /// stamps `baselines` entries for the eviction-victim choice.
    drain_counter: u64,
    /// Per-(consumer, input) counters — a detected eviction bumps
    /// `drop_oldest_count` by the per-drain evicted total.
    counters: Arc<BackpressureCounters>,
    /// Consumer node id (structured-warn attribution + event).
    node_id: Arc<str>,
    /// Consumer input field name.
    input: Arc<str>,
    /// Edge-trigger latch: a drain with NO eviction rearms; the first
    /// eviction after that queues one [`BackpressureEvent`] and disarms
    /// (one event per eviction regime, mirroring the sample gate).
    armed: bool,
    /// Count of consecutive non-empty drains containing
    /// [`GapClass::Backward`] (duplicate re-delivery) streams; reset on
    /// any non-empty drain with no backward stream (empty drains carry no
    /// information and leave it untouched). No onset warn (a single replay
    /// drain is routine — every late joiner on a history-enabled topic
    /// produces one); warns every [`ANOMALY_REWARN_EVERY`] drains under
    /// SUSTAINED re-delivery, where the replaying stream's counting stays
    /// intermittent (other streams keep counting exactly).
    backward_run: u64,
    /// Wire timestamp of the drain that opened the current eviction regime
    /// (the opening drain's high-water frame).
    regime_started_ns: u64,
    /// Evicted-sample count of the regime-OPENING drain. Events fire only
    /// at regime open, which is the only point this is read; later
    /// evictions while disarmed bump `drop_oldest_count` but no per-regime
    /// state (deliberate: a disarmed `+=` would be dead state no event
    /// could ever surface).
    regime_count: u64,
    /// Subscriber buffer capacity (carried on the event for user recovery
    /// decisions).
    buffer_capacity: usize,
}

/// `block`: consumer-side event probe. `block` loses NO data — the
/// producer's pre-fire defer already holds the producer back and bumps
/// `block_fires_deferred_count`. This probe fires the CONSUMER's
/// `#[on_event(input = "...")]` `BackpressureEvent` handler when its own queue was at the defer line on a
/// drain (it caused/sustained the defer), as a flow-control signal with
/// `dropped == 0`. It READS (never bumps) the shared counter for
/// `count_total`, so there's no double-count with the producer side.
struct BlockProbe {
    /// Shared mirror of this consumer's iceoryx2 queue depth, sampled
    /// pre-drain. Each drain decrements it (saturating). It is a
    /// process-local heap word for a co-located edge, a mapped SHM page when
    /// the producer lives in another process — the same operations either way
    /// ([`crate::credit::CreditWord`]), so a split edge's drain frees credit
    /// the peer's pre-fire gate can see.
    outstanding: crate::credit::CreditWord,
    /// Defer line = the declared `#[input(depth = N)]` — the input's real
    /// iceoryx2 queue (no clamp). At/above it the
    /// producer is being deferred on this consumer's behalf.
    threshold: u64,
    /// Shared counters — `block_fires_deferred_count` is maintained by the
    /// PRODUCER pre-fire; the event only reads it. (No `node_id`: this probe
    /// never warns/bumps — the producer's defer already does both — so it
    /// needs no attribution beyond `input`.)
    counters: Arc<BackpressureCounters>,
    input: Arc<str>,
    /// Edge-trigger latch: a below-threshold drain rearms; the first
    /// at-threshold drain after that queues one event and disarms.
    armed: bool,
    /// Wire timestamp of the regime-opening drain's high-water frame
    /// (0 when that drain had no readable wire timestamp).
    regime_started_ns: u64,
    /// Always 1 — set at regime open, the only point an event reads it
    /// (see the sibling probes' field docs for the rationale).
    regime_count: u64,
    buffer_capacity: usize,
}

/// The single backpressure probe installed on an input.
/// An input carries exactly ONE policy (topology-enforced upstream in
/// `graph/topology.rs`), so at most one probe can ever be live — this enum
/// makes two-at-once UNREPRESENTABLE (the former three sibling `Option`
/// fields permitted it, pinned only by debug-build asserts at registration
/// and per receive). All three variants queue into the single
/// `pending_backpressure_event` slot, which the type now proves has one
/// writer per input. What stays representable is registering twice
/// (silently REPLACING live probe state); `assert_no_probe_installed`
/// pins that loudly at the mutation point, in ALL builds.
enum BackpressureProbe {
    /// `sample(N)` subscriber read-gate (see [`SampleGate`]): `try_view`
    /// consults it AFTER drain-to-latest — a latest sample whose wire
    /// timestamp is `< N` ms after the last accepted one is decimated
    /// (dropped, `sampled_count` bumped, `Ok(None)` returned).
    Sample(SampleGate),
    /// `drop_oldest` silent-eviction detector (see [`DropOldestProbe`]):
    /// installed on inputs the runtime wires as `drop_oldest` (the default
    /// policy); compares per-publisher wire sequences after each drain to
    /// spot what iceoryx2 reclaimed on overflow.
    DropOldest(DropOldestProbe),
    /// `block` consumer-side event probe (see [`BlockProbe`]): installed on
    /// `block` consumers on all-`block` topics; holds the shared
    /// `outstanding` mirror (each drain decrements it, saturating) plus the
    /// threshold + edge state to fire the consumer's event at the defer
    /// line.
    Block(BlockProbe),
}

/// `sample(N)`: read the wire `timestamp_ns` (publish clock) from a
/// frame's leading [`WireHeader`] without a full parse — bytes `[24..32]`,
/// little-endian. Returns `None` for an undersized frame (shorter than a
/// `WireHeader`); the caller must NOT decimate such a frame on the sample
/// gate — it is a corrupt/truncated frame that the normal receive path
/// surfaces as a loud `Deserialization` error, so we fall through to it
/// rather than masking the corruption as a routine sample-drop.
#[inline]
fn wire_timestamp_ns(raw: &[u8]) -> Option<u64> {
    if raw.len() >= WireHeader::SIZE {
        let bytes = raw.get(24..32)?.try_into().ok()?;
        Some(u64::from_le_bytes(bytes))
    } else {
        None
    }
}

/// Read the wire `sequence` (bytes `[20..24]`) from a raw frame, or `None`
/// if the frame is too small to hold a full `WireHeader`. Used by the
/// `drop_oldest` detector to spot iceoryx2 evictions via sequence
/// gaps. Mirrors [`wire_timestamp_ns`].
#[inline]
fn wire_sequence(raw: &[u8]) -> Option<u32> {
    if raw.len() >= WireHeader::SIZE {
        let bytes = raw.get(20..24)?.try_into().ok()?;
        Some(u32::from_le_bytes(bytes))
    } else {
        None
    }
}

/// The PRODUCER TOKEN for one served sample — a 64-bit FNV-1a
/// over iceoryx2's `UniquePublisherId::value()` little-endian bytes.
///
/// The raw id is run-random by construction (`UniqueSystemId` mints it from
/// the process id and a creation time), so it is meaningless ACROSS runs and
/// is never compared raw: each side resolves its own run's ids through its
/// own manifest table before anything is compared. What
/// the token buys over the raw `u128` is that it fits the kind-6 aux slot;
/// the collapse to 64 bits is why the offline resolver must report a
/// COLLISION rather than pick a winner.
#[inline]
fn producer_token(id: UniquePublisherId) -> u64 {
    crate::wire::fnv1a_hash(&id.value().to_le_bytes())
}

/// Forward/backward decision threshold for same-publisher sequence gaps: a
/// wrapping gap of at most half the `u32` range reads as forward motion
/// (eviction candidate); anything larger means the drain's oldest sequence
/// is BEHIND the stream's baseline — duplicate re-delivery, not eviction.
/// See [`GapClass::Backward`].
const BACKWARD_GAP_THRESHOLD: u32 = u32::MAX / 2;

/// Under SUSTAINED duplicate re-delivery (every drain contains a
/// [`GapClass::Backward`] stream — e.g. late-joiner churn on a
/// history-enabled topic replaying to every subscriber), re-emit the
/// intermittency warn every this-many consecutive backward drains. A
/// single debug breadcrumb would scroll out of the log buffer while
/// counting stays intermittent for hours — sustained blindness, which the
/// periodic re-warn exists to break.
const ANOMALY_REWARN_EVERY: u64 = 64;

/// Per-drain, per-publisher-stream observation captured
/// during the drain loop. One entry per distinct `sample.origin()` id seen
/// in the drain; lives in the subscriber's reusable scratch (taken before
/// the drain, restored after — capacity retained, zero-alloc steady state).
#[derive(Debug, Clone, Copy)]
struct StreamObservation {
    /// Origin publisher id of this stream.
    id: UniquePublisherId,
    /// First-drained (oldest in queue order) wire sequence of this stream —
    /// the gap between the stream's baseline and this is its eviction
    /// count. Under history replay a LATER re-delivered frame may carry a
    /// lower sequence; it never updates this field.
    first_seq: u32,
    /// HIGH-WATER wire sequence of this stream within the drain (NOT
    /// drain-order last: a trailing replayed frame, wrapping-backward of
    /// `first_seq`, never becomes the high-water — otherwise the baseline
    /// would regress and the next drain would count this drain's consumed
    /// live frames as evicted).
    newest_seq: u32,
    /// Wire timestamp of the `newest_seq` frame (regime stamps must not
    /// come from a trailing replayed frame's stale clock).
    newest_ts: u64,
}

/// Classification of one stream's wire-sequence gap between its baseline
/// (highest sequence consumed on a previous drain) and the LOWEST sequence
/// of that stream still in the queue on this drain.
///
/// **Exact**: per-publisher sequences are monotonic and dense, and the
/// per-id baseline lookup pins both sides of the gap to one publisher's
/// stream, so every sample in the open interval `(baseline, first)` was
/// published and never reached us — on a connected subscriber the only way
/// to miss them is queue eviction. The samples a drain then skips via
/// "latest wins" are NOT evictions: they are contiguous with
/// `first..=newest` and *were* delivered to the queue.
///
/// - Contiguous / re-saw (raw forward gap ≤ 1) → [`GapClass::Clean`].
/// - Raw forward gap ≥ 2 with gap ≤ [`BACKWARD_GAP_THRESHOLD`] →
///   `Exact(gap − 1)` — **unbounded**: a consumer lagging by many buffers'
///   worth between drains reports the true loss, not a saturated floor.
/// - Backward (wrapping gap > the threshold) → [`GapClass::Backward`]:
///   duplicate re-delivery, never counted. A live publisher's sequence is
///   monotonic for its whole life, so a backward jump within one stream is
///   re-delivery. Native iceoryx2 history delivers a late
///   joiner the retained frames with their ORIGINAL stale sequences (by SHM
///   offset, before any newer live frame), so a late joiner that already saw
///   a high live sequence and then drains older retained frames observes a
///   backward jump within the one publisher stream. Counting the wrapped gap
///   would fabricate a ~2³² eviction count the moment a late joiner attaches
///   to a history-enabled topic.
///
/// `u32` wraparound at `MAX` stays exact: a genuine `MAX → 0` rollover
/// eviction yields a small wrapped forward gap (≤ threshold), while a
/// replayed stale frame yields a near-2³² wrapping gap → `Backward`.
#[derive(Debug, PartialEq, Eq)]
enum GapClass {
    /// No eviction in this stream (contiguous sequence or re-saw).
    Clean,
    /// Exactly `n` samples evicted from this stream.
    Exact(u32),
    /// This stream's oldest drained sequence is BEHIND its baseline —
    /// duplicate re-delivery (history replay), never counted. The caller
    /// keeps the stream's live high-water baseline (advancing it only if
    /// the drain also contained newer live frames) and tracks SUSTAINED
    /// re-delivery via `backward_run` (warn every
    /// [`ANOMALY_REWARN_EVERY`] drains).
    Backward,
}

/// Classify one stream's gap (see [`GapClass`]). Pure — unit-tested
/// directly; the per-id baseline lookup in `try_view` guarantees both
/// sequences belong to the same publisher stream.
fn classify_gap(base_seq: u32, first_seq: u32) -> GapClass {
    let gap = first_seq.wrapping_sub(base_seq);
    if gap > BACKWARD_GAP_THRESHOLD {
        return GapClass::Backward;
    }
    let evicted = gap.saturating_sub(1); // samples strictly between base and first
    if evicted == 0 {
        GapClass::Clean // gap 0 (re-saw) or gap 1 (contiguous)
    } else {
        GapClass::Exact(evicted)
    }
}

/// An inbound iceoryx2 sample (slice payload, no user header).
type InboundSample = Sample<CerService, [u8], ()>;

/// A validated, zero-copy raw input frame.
///
/// A live sample owns its inbound slot through this view. A held sample
/// borrows the subscriber's held slot, mirroring typed `try_view`: held
/// serves do not record a service cursor and remain available for re-serve.
pub struct RawInputView<'a> {
    inner: RawInputViewInner<'a>,
}

enum RawInputViewInner<'a> {
    Owned(InboundSample, usize),
    Held(&'a [u8]),
}

impl RawInputView<'_> {
    /// Detaches the view from the subscriber borrow when it owns its sample.
    ///
    /// `Owned` views pin their SHM sample and consume borrow budget for as long
    /// as the returned value lives; a `Held` view borrows the subscriber's
    /// held sample and is returned unchanged in the error arm.
    pub fn into_owned(self) -> Result<RawInputView<'static>, Self> {
        match self.inner {
            RawInputViewInner::Owned(sample, len) => Ok(RawInputView {
                inner: RawInputViewInner::Owned(sample, len),
            }),
            RawInputViewInner::Held(bytes) => Err(RawInputView {
                inner: RawInputViewInner::Held(bytes),
            }),
        }
    }
}

impl std::ops::Deref for RawInputView<'_> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        match &self.inner {
            RawInputViewInner::Owned(sample, frame_len) => &sample.payload()[..*frame_len],
            RawInputViewInner::Held(bytes) => bytes,
        }
    }
}

impl AsRef<[u8]> for RawInputView<'_> {
    fn as_ref(&self) -> &[u8] {
        self
    }
}

/// An owned, zero-copy inbound sample held beyond the receive callback
/// (the `bagd` recorder tap).
///
/// Wraps a single iceoryx2 inbound sample so the recorder can drain a batch
/// of frames off a [`CerulionSubscriber`] and hold them past the drain call
/// (e.g. queue them for an off-thread `writev`), reading each frame's bytes
/// zero-copy straight out of shared memory.
///
/// # This is the RECORDING tap and the rmw loaned take, not the node hot path
///
/// Produced by [`DataOnlySubscriber::drain_owned`] (the recorder tap)
/// and by [`CerulionSubscriber::try_receive_one_owned`] (the rmw
/// loaned-take path, which holds the sample across the C ABI until rclcpp
/// returns the loan). It is entirely independent of the backpressure
/// machinery and the step-boundary snapshot (`frozen` /
/// `held_sample`) — draining owned samples runs NO probe accounting and does
/// NOT reset any watchdog. Use it for recording/introspection/rmw loans, never
/// as a node's input read path (that stays [`CerulionSubscriber::try_view`]).
///
/// # Cost of holding one (SHM back-pressure)
///
/// Each live `OwnedInboundSample` PINS one publisher-pool slot in shared memory
/// (the publisher cannot reclaim that slot until this drops) AND consumes one
/// unit of the service's `subscriber_max_borrowed_samples` budget on this
/// subscriber's connection(s). Holding more than that budget makes the next
/// `receive()` fail with `ExceedsMaxBorrows` (see
/// [`DataOnlySubscriber::drain_owned`]). Drop it to release both the pool slot
/// and the borrow unit.
pub struct OwnedInboundSample {
    /// The owned iceoryx2 sample; its `payload()` borrows the SHM slot for as
    /// long as this value lives.
    sample: InboundSample,
    /// Number of leading bytes of `sample.payload()` that make up the actual
    /// published wire frame (`WireHeader::total_size`), computed once at
    /// construction. iceoryx2 loans slots sized by the publisher's adaptive
    /// sizer / `max_slice_len`, which can exceed the frame, leaving trailing
    /// capacity bytes past the frame ("subscribers slice on `total_size`" —
    /// see `publisher.rs`). `frame_len` is that slice bound so [`Self::payload`]
    /// returns exactly the bytes the publisher wrote. Falls back to the full
    /// slot length for a frame with no parseable / out-of-bounds header (a
    /// corrupt frame is exposed whole rather than silently hidden).
    frame_len: usize,
}

impl OwnedInboundSample {
    /// Wrap an owned inbound sample, computing its wire-frame length once
    /// (alignment-safe header read; SHM slots are not guaranteed 8-byte
    /// aligned — see `wire.rs`).
    fn new(sample: InboundSample) -> Self {
        let raw = sample.payload();
        let frame_len = match WireHeader::read_from_buf(raw) {
            Some(header) => {
                let total = header.total_size as usize;
                // Only trust a header whose total_size names a slice WITHIN the
                // loaned slot and at least the header itself; otherwise expose
                // the whole slot (hide nothing from the recorder).
                if (WireHeader::SIZE..=raw.len()).contains(&total) {
                    total
                } else {
                    raw.len()
                }
            }
            None => raw.len(),
        };
        Self { sample, frame_len }
    }

    /// The full published wire frame (32-byte [`WireHeader`] + body), exactly
    /// the bytes the publisher wrote — zero-copy, borrowing the SHM slot.
    ///
    /// Trailing loan capacity past `WireHeader::total_size` is excluded (the
    /// publisher may loan a slot larger than the frame), so the returned slice
    /// is the recordable frame and nothing more. For a frame with an
    /// unparseable / out-of-bounds header the whole loaned slot is returned
    /// (a corrupt frame is never silently truncated to hidden).
    pub fn payload(&self) -> &[u8] {
        &self.sample.payload()[..self.frame_len]
    }

    /// The parsed [`WireHeader`] at the front of the frame, or `None` when the
    /// frame is shorter than a header / unparseable.
    ///
    /// Returned by value (32 bytes on the stack) via the alignment-safe
    /// [`WireHeader::read_from_buf`]: iceoryx2 SHM slots are not guaranteed
    /// 8-byte aligned, so a borrowed [`WireHeader::from_bytes`] view would
    /// spuriously return `None` on an unaligned slot. This matches how every
    /// other SHM read path in the transport parses the header.
    pub fn wire_header(&self) -> Option<WireHeader> {
        WireHeader::read_from_buf(self.sample.payload())
    }

    /// The PUBLISHER IDENTITY of this frame — iceoryx2's
    /// `UniquePublisherId` for the port that committed it, raw — so the
    /// recorder can stamp record-time producer LABELS and tell one
    /// `multi_publisher_topics` topic's interleaved streams apart.
    ///
    /// This is sample METADATA: no wire parse, no byte of the payload is
    /// touched, and nothing is allocated (this file is inside the
    /// hot-path allocation lint's scan set, and the accessor is
    /// allocation-free).
    ///
    /// The id is RUN-RANDOM by construction (`UniqueSystemId` mints it from
    /// the process id and a creation time), so it is meaningless ACROSS runs
    /// and is never compared raw: each side resolves its own run's ids
    /// through that run's per-rank manifest `publishers` table first — the
    /// producer-token precedent, and the same caveat `producer_token` above
    /// carries. Returned as the raw `u128` so a caller (`cerulion_bagd`)
    /// needs no iceoryx2 type import.
    pub fn origin(&self) -> u128 {
        self.sample.origin().value()
    }
}

// Compile-time Send pin: `OwnedInboundSample` MUST stay `Send` so the
// recorder can hand a drained frame to an off-thread writer. `InboundSample`
// (`Sample<ipc_threadsafe::Service, [u8], ()>`) is `Send` because
// `ipc_threadsafe`'s `ArcThreadSafetyPolicy = MutexProtected` is `Send + Sync`
// (iceoryx2 0.9.1 `sample.rs`). If a future change erases that `Send`, this
// const closure stops type-checking and the file fails to compile.
const _: fn() = || {
    fn assert_send<T: Send>() {}
    assert_send::<OwnedInboundSample>();
};

/// A **data-only** subscriber for the record/replay capture tap — a
/// zero-copy SHM reader with NO event `Listener` and NO `Notifier`.
///
/// # Why it exists (the storm, killed structurally)
///
/// The recorder (`bagd`) and the replay engine's capture taps drain SHM samples
/// exclusively via a POLLING loop ([`Self::drain_owned`]); they never wait on an
/// event listener. A [`CerulionSubscriber`], however, MANDATES a `Listener` (it
/// bundles a data receiver AND a WaitSet event listener), and iceoryx2's notifier
/// sends an 8-byte `SentSample` datagram to EVERY connected listener on every
/// publish. A tap that never drains that listener lets its `AF_UNIX SOCK_DGRAM`
/// socket fill; once full, every publisher notify pays iceoryx2's
/// `FailedToDeliverSignal` path (~12 µs + a ~2 KB warn dump per notify) — the
/// notify warn storm (10–150 MB/s of stderr, enough to fill a disk
/// during a long replay).
///
/// A `DataOnlySubscriber` opens ONLY the data pub/sub service (never the topic's
/// event service), so it registers NO listener connection: the notifier's
/// per-listener send loop finds ZERO tap connections → ZERO sends → ZERO failed
/// sends, STRUCTURALLY, at DEFAULT sysctls, under any drain stall or publish
/// rate. There is no queue, no ceiling, and no `net.unix.max_dgram_qlen` to hit.
/// This makes a drain-listener, a receive-buffer deepening and a
/// sender-buffer sweep all unnecessary — each of those would only be
/// treating the symptom of a listener the tap never needed.
///
/// # No late-joiner history
///
/// A `DataOnlySubscriber` has NO notifier, so it never sends the
/// `SubscriberConnected` event a publisher uses to replay its iceoryx2 history
/// ring to a late joiner. A tap therefore sees ONLY frames published from the
/// moment it attaches onward — any frame emitted BEFORE it attached (including
/// whatever is in a publisher's history ring at attach time) is invisible to it.
/// For a recorder (`cerulion_bagd`) that means pre-attach frames are absent from
/// the bag; `bagd_cli_run` warns about this once per run for ATTACH-mode taps
/// only (exact-mode taps — the `graph run --record` subprocess shape — attach
/// before any node publishes, so they stay silent).
///
/// # Bonus: notify elision stays armed
///
/// With the tap invisible to the event service, a same-process producer's
/// `number_of_listeners()` equals the count of REAL same-process consumers, so
/// the notify-elision gate (`publisher.rs`) stays ARMED during record
/// runs — the producer elides the notify to its real in-step consumers too, so a
/// pure `graph run --record` issues zero `sendto` at all.
///
/// # Compile-time unattachability (type-design)
///
/// This type deliberately exposes NO `Listener` and no accessor that yields one,
/// and it is a DISTINCT type from [`CerulionSubscriber`] — so it CANNOT be passed
/// to `WaitSet::attach_notification` (which takes a `Listener`). A data-only tap
/// being WaitSet-attached is therefore *unrepresentable*, not merely discouraged
/// — the compile-time-prevention discipline (make the pathological state
/// unrepresentable rather than guard it at runtime).
///
/// The proof pins the DURABLE root property — a `DataOnlySubscriber` is a
/// distinct type that cannot stand in for the ONLY subscriber a WaitSet source
/// accepts ([`CerulionSubscriber`], which owns the `Listener`). It is
/// deliberately anchored to the two permanent public types so the proof stays
/// meaningful regardless of which wait helpers exist — the example keeps proving
/// unattachability against surface that will outlive them. A future change that
/// gave `DataOnlySubscriber`
/// a coercion to `CerulionSubscriber` (the prerequisite for attachability) would
/// make this COMPILE and break the doctest:
///
/// ```compile_fail
/// use cerulion_core::transport::subscriber::{CerulionSubscriber, DataOnlySubscriber};
/// fn cannot_stand_in_for_a_waitset_source(tap: &DataOnlySubscriber) {
///     // ERROR[E0308]: expected `&CerulionSubscriber`, found `&DataOnlySubscriber`.
///     // A WaitSet notification source is a `Listener`, owned only by
///     // `CerulionSubscriber`; a data-only tap is a distinct listener-less type
///     // with no coercion to it, so it can never be attached.
///     let _wrong: &CerulionSubscriber = tap;
/// }
/// ```
///
/// # Safety (Principles #6 + #7)
///
/// Removing a wake-only listener changes nothing about which frames are
/// recorded/replayed or their bytes (the tap never used the wake — it polls the
/// SHM data queue, provisioned deep at [`super::RECORDING_TAP_BUFFER_DEPTH`]).
/// No same-process consumer loses a wake (the ≤250 ms live heartbeat + the
/// doorbell backstops are untouched).
pub struct DataOnlySubscriber {
    topic: String,
    subscriber: Subscriber<CerService, [u8], ()>,
    /// The tap's own handle on the topic's data service, kept so
    /// [`Self::publisher_count`] is a dynamic-config read rather than a fresh
    /// `.open()`.
    ///
    /// Holding it costs nothing new — `subscriber` already keeps the service
    /// alive for the tap's whole life — and it is what makes the single-writer
    /// probe cheap enough to run on EVERY drain, which is the cadence the rate
    /// estimate's trust decision needs.
    data_service:
        iceoryx2::service::port_factory::publish_subscribe::PortFactory<CerService, [u8], ()>,
    /// The topic's effective `subscriber_max_borrowed_samples` (read
    /// from the data service's static config at creation — the per-service truth).
    max_borrowed_samples: usize,
    /// Test seam (mirrors [`CerulionSubscriber::fault_inject_receive_after`]):
    /// see [`Self::drain_owned`]. Cfg-gated (unlike `CerulionSubscriber`'s
    /// unconditional field): `DataOnlySubscriber` never crosses an FFI / cdylib
    /// boundary — it is a purely host-side recorder/replay type — so the
    /// struct-layout-stability rule does not apply, and the field is genuinely
    /// write-only in production (its reads are test-gated). Gating it keeps
    /// `dead_code = deny` satisfied without an `allow`.
    #[cfg(any(test, feature = "test-helpers"))]
    fault_inject_receive_after: Option<u32>,
}

impl DataOnlySubscriber {
    /// Assemble a data-only subscriber. Called by
    /// [`TransportManager::create_data_only_subscriber`](super::TransportManager::create_data_only_subscriber);
    /// not intended for direct use. Unlike [`CerulionSubscriber::new`] it sends NO
    /// `SubscriberConnected` event (it has no notifier) — the tap reads live
    /// frames forward and never requests late-joiner history.
    pub(crate) fn new(
        topic: String,
        subscriber: Subscriber<CerService, [u8], ()>,
        max_borrowed_samples: usize,
        data_service: iceoryx2::service::port_factory::publish_subscribe::PortFactory<
            CerService,
            [u8],
            (),
        >,
    ) -> Self {
        Self {
            topic,
            subscriber,
            max_borrowed_samples,
            data_service,
            #[cfg(any(test, feature = "test-helpers"))]
            fault_inject_receive_after: None,
        }
    }

    /// The receive-queue depth this tap was ACTUALLY created
    /// with — the mode gate's observable (Principle #3).
    ///
    /// The depth a caller ASKS for and the depth it GETS are different facts: a
    /// request is clamped into the service's legal range, and the ceiling that
    /// clamps it belongs to the producer. Without this accessor "the `--record`
    /// tap stayed ceiling-deep" is a claim no test can check, and the whole
    /// contract split would rest on reading the code.
    pub fn buffer_size(&self) -> usize {
        self.subscriber.buffer_size()
    }

    /// What ONE queued chunk of this topic costs in the
    /// producer's SHM pool RIGHT NOW —
    /// [`iceoryx2_slot_bytes`](super::iceoryx2_slot_bytes) of the WIDEST
    /// currently-attached publisher's slice.
    ///
    /// # LIVE, never a creation-time cache — and that is a correctness requirement
    ///
    /// A value cached when the tap was created would be wrong, because a topic's
    /// widest publisher CHANGES: a tap opened while only a narrow publisher
    /// existed would keep quoting the narrow slot after a wide one joins, so the
    /// pinned-bytes figure under-reports by the ratio of the two slices. The
    /// slice is a per-PUBLISHER property with no static-config mirror, so a
    /// cached copy can only ever be a snapshot of one moment's publisher set.
    ///
    /// The read is the same class [`Self::publisher_count`] already
    /// does on every drain — a dynamic-config walk on the service handle this
    /// tap already holds, no `.open()`, no new port, no allocation — which is
    /// what makes it affordable per drain rather than per creation.
    ///
    /// `None` when NO publisher is attached: the correct reading is "this tap
    /// cannot price its own occupancy", never a zero. A reader summing pinned
    /// bytes must report the unpriced taps rather than treating them as free.
    pub fn live_slot_bytes(&self) -> Option<usize> {
        self.live_max_slice_len().map(super::iceoryx2_slot_bytes)
    }

    /// The WIDEST `max_slice_len` among the publishers
    /// attached to this topic right now, or `None` if there are none.
    ///
    /// The MAXIMUM, because a tap can be handed a chunk from any attached
    /// publisher and the budget must be priced against the biggest slot it
    /// could be given. `PublisherDetails::max_slice_len`
    /// (`iceoryx2-0.9.1/src/service/dynamic_config/publish_subscribe.rs:62`) is
    /// the only place iceoryx2 exposes it — a service's STATIC config does not
    /// carry the slice at all.
    pub fn live_max_slice_len(&self) -> Option<usize> {
        use iceoryx2::prelude::{CallbackProgression, PortFactory as _};
        let mut widest: Option<usize> = None;
        self.data_service
            .dynamic_config()
            .list_publishers(|details| {
                widest = Some(widest.map_or(details.max_slice_len, |w: usize| {
                    w.max(details.max_slice_len)
                }));
                CallbackProgression::Continue
            });
        widest
    }

    /// How many publishers are attached to this topic's data service
    /// RIGHT NOW — the single-writer evidence the rate estimate's sequence basis
    /// requires.
    ///
    /// The wire `sequence` is a PER-PUBLISHER commit counter, and
    /// [`DrainObservation::newest_sequence`](super::liveness::DrainObservation::newest_sequence)
    /// is the MAX across a batch, so on a topic with two writers that maximum hops
    /// between unrelated counters and its delta is not a frame count at all. Only
    /// a topic with exactly ONE publisher can be measured that way, so the rate
    /// window asks on every drain and falls back to the labelled FLOOR whenever the
    /// answer is anything else.
    ///
    /// A `PortFactory` read against the service's dynamic config — no `.open()`,
    /// no allocation. Graph-owned topics provision `max_publishers = 1`, so on the
    /// overwhelming majority of topics this reads `1` forever.
    pub fn publisher_count(&self) -> u32 {
        use iceoryx2::prelude::PortFactory as _;
        self.data_service.dynamic_config().number_of_publishers() as u32
    }

    /// Returns the topic name.
    pub fn topic(&self) -> &str {
        &self.topic
    }

    /// The topic's effective iceoryx2 `subscriber_max_borrowed_samples`
    /// budget — the maximum number of samples this subscriber may hold borrowed
    /// at once, read from the data service's static config at creation (the
    /// per-service truth, so it reflects per-topic overrides). The `bagd`
    /// recorder uses it to bound how many owned frames it holds at once (see
    /// [`Self::drain_owned`]).
    pub fn max_borrowed_samples(&self) -> usize {
        self.max_borrowed_samples
    }

    /// Whether this tap's receive queue still holds samples — the
    /// NON-CONSUMING emptiness observation, distinct from inferring emptiness
    /// from how many frames a [`Self::drain_owned`] came back with.
    ///
    /// A drain that FILLED its `max` proves nothing about the remainder, and a
    /// caller that takes exactly one budget per pass (the network gateway's egress
    /// drain) therefore cannot tell "the queue is now empty" from "there is more"
    /// by arithmetic at all — at `max == 1` the arithmetic answer is unreachable.
    /// This asks the port instead. It is non-consuming BY DESIGN: draining one
    /// extra frame would answer the same question but would also change WHAT the
    /// caller forwards and when, which an emptiness query must not do.
    ///
    /// Cheap, but not free — it runs iceoryx2's `update_connections()` first (the
    /// same work any `receive()` does), so call it only when the arithmetic is
    /// genuinely ambiguous.
    pub fn has_samples(&self) -> TransportResult<bool> {
        self.subscriber
            .has_samples()
            .map_err(|e| TransportError::Receive {
                topic: self.topic.clone(),
                reason: format!("has_samples: {e}"),
            })
    }

    /// The record/replay tap: drain up to `max` samples off the SHM
    /// queue, pushing each as an [`OwnedInboundSample`] onto `out`, and return the
    /// number pushed.
    ///
    /// Borrow-budget contract: pass `max` ≤ remaining budget or iceoryx2 fails
    /// with `ExceedsMaxBorrows`; FIFO / no-loss; `max == 0` → `Ok(0)`; frames
    /// already pushed are KEPT on a mid-drain `receive()` error. Runs NO
    /// backpressure/probe accounting and resets no watchdog — a raw recording
    /// tap, independent of the snapshot machinery.
    /// The frames read zero-copy straight out of shared memory.
    #[must_use = "drain_owned result must be checked (partial fills are kept in `out` on Err)"]
    pub fn drain_owned(
        &mut self,
        max: usize,
        out: &mut Vec<OwnedInboundSample>,
    ) -> TransportResult<usize> {
        if max == 0 {
            return Ok(0);
        }
        let mut pushed = 0usize;
        while pushed < max {
            // Test seam (compiled out of production builds): reuses the
            // `fault_inject_receive_after` countdown — decremented per drained
            // sample; when it reaches 0 the next receive fails, leaving the
            // partial fill in `out` (the kept-on-Err contract).
            #[cfg(any(test, feature = "test-helpers"))]
            if self.fault_inject_receive_after == Some(0) {
                self.fault_inject_receive_after = None;
                return Err(TransportError::Receive {
                    topic: self.topic.clone(),
                    reason: "fault injection: drain_owned receive failure (test seam)".to_string(),
                });
            }
            let received = self
                .subscriber
                .receive()
                .map_err(|e| TransportError::Receive {
                    topic: self.topic.clone(),
                    reason: format!("{}", e),
                })?;
            match received {
                Some(sample) => {
                    // Plain prose, NOT an annotation: this lint deliberately does
                    // not match `.push(` (25 sites, all pushes into cleared-and-
                    // reused buffers — see the script header), so a marker here
                    // arms nothing and only lands in its unmatched banner.
                    //
                    // Cold recording tap — `out` is a caller-owned, pre-reserved
                    // batch buffer (`bagd`/replay reserve once), NOT the
                    // per-message node hot path.
                    out.push(OwnedInboundSample::new(sample));
                    pushed += 1;
                    #[cfg(any(test, feature = "test-helpers"))]
                    if let Some(n) = self.fault_inject_receive_after {
                        self.fault_inject_receive_after = Some(n.saturating_sub(1));
                    }
                }
                None => break,
            }
        }
        Ok(pushed)
    }

    /// Test seam (compiled out of production builds): arm the
    /// [`Self::drain_owned`] receive fault after `after` successful drains.
    /// Mirrors [`CerulionSubscriber::fault_inject_receive_after_for_test`].
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn fault_inject_receive_after_for_test(&mut self, after: u32) {
        self.fault_inject_receive_after = Some(after);
    }
}

// Compile-time Send pin: `DataOnlySubscriber` MUST stay `Send` so the
// recorder can own it inside a thread-driven `Recorder` (same reasoning as
// `OwnedInboundSample` / `CerulionSubscriber`). If a future change erases that
// `Send`, this const closure stops type-checking and the file fails to compile.
const _: fn() = || {
    fn assert_send<T: Send>() {}
    assert_send::<DataOnlySubscriber>();
};

/// A drained-and-accounted sample frozen at a step boundary for a
/// non-trigger latest-value input. Produced by
/// `snapshot_latest`, served by the following `try_view`(s). Four states
/// because the drain's accounting already ran and must NOT re-run:
/// re-draining would double-count backpressure. An ERRORED drain already
/// mutated accounting (popped frames decremented the block mirror, reset
/// drop_oldest baselines), so the error is STORED and replayed at body
/// read — never re-driven.
///
/// # SERVE-MANY within one step
///
/// A serve does NOT clear the slot. The slot is CLEARED-AND-RECAPTURED at its
/// two capture sites — [`CerulionSubscriber::snapshot_latest`] (the non-trigger
/// step-boundary snapshot) and [`CerulionSubscriber::drain_for_trigger`] (the
/// trigger boundary drain / burst refill) — which is what makes "the step" the
/// slot's lifetime rather than "the first read".
///
/// It exists because a `Data`-trigger node bursts `fire_count` fires inside ONE
/// step ([`crate::scheduler::Scheduler::tick_data_burst`]) while its plain
/// non-trigger `#[input]` context is snapshotted ONCE per level pass. Serve-once
/// meant the first fire CONSUMED the frozen context, so fires 2..k found the slot
/// empty and FELL TO THE LIVE ARM below — and that fall-through is the defect in
/// both of its outcomes:
///
/// * on a QUIET context topic (the motivating shape — a data-triggered
///   controller with a slow `/map` or config input) the live drain found the
///   queue empty and returned `Ok(None)`, so the tick chain collapsed at the
///   macro's declaration-ordered `try_view` and the burst was capped at one
///   message per step;
/// * on a NON-quiet one it really POPPED — leaking a mid-step read into the
///   middle of a burst, so the fires of ONE step disagreed about their context
///   and record/replay could diverge (Principle #7). The Period catch-up shape
///   is the measured instance: a same-level producer publishes during the step,
///   fire 1 could not see it and fire 2 served it
///   (`read_outcome_capture_iox2_test`'s arm (g) recorded exactly that, and was
///   renegotiated with this decision).
///
/// Re-serving is CORRECT because a non-trigger input is latest-value by
/// contract: every fire of one step reads the identical bytes, and the frame the
/// live drain would steal stays queued for the next boundary to serve.
///
/// TWO variants are still consumed by their serve, and each for a reason that is
/// not about the step:
///
/// * [`FrozenSlot::Sample`] — the FIFO head of a `ConsumeMode::EachFifo` TRIGGER
///   input. Taking it is the ONLY signal `drain_for_trigger` has that a tick
///   really read the frame it froze (its re-offer guard reads `Some(Sample)` as
///   "unserved"). Leaving it would re-offer a served head forever and the burst
///   would never advance. The trigger's own per-fire value is refreshed BETWEEN
///   fires by [`CerulionSubscriber::refill_for_trigger`], so it needs no
///   re-serve. (`snapshot_latest` never leaves a `Sample` in the slot — it moves
///   the sample into `held_sample` and freezes `Held` — so this arm is exactly
///   the trigger/direct path.)
/// * [`FrozenSlot::Err`] — [`TransportError`] is not `Clone`, so the error is
///   MOVED out and can be replayed exactly once. Documented residual: a later
///   read in the same step therefore falls to the live arm. It is bounded to the
///   drain-error path, which already mutated accounting and is exceptional.
enum FrozenSlot {
    /// A delivered sample survived the drain + sample(N) gate.
    ///
    /// CONSUMED by its serve — see the type docs (the FIFO head-served signal).
    Sample(InboundSample),
    /// Serve the subscriber's HELD sample (`held_sample`, set on a
    /// prior delivery) WITHOUT re-draining — a non-trigger latest-value input
    /// replaying its last value on a step with no new arrival. A pure read:
    /// no accounting (the drain already accounted when the sample was first
    /// delivered). `try_view` serves it via `SampleHandle::InboundRef`.
    ///
    /// SURVIVES its serve — a pure marker over `held_sample`, so
    /// every fire of the step reads the identical bytes for free.
    Held,
    /// The drain ran and yielded no deliverable sample (empty queue, or a
    /// sample(N)-decimated frame). Body read returns `Ok(None)`.
    ///
    /// SURVIVES its serve — every fire of the step observes the same
    /// "nothing to read", instead of the second one live-draining a frame that
    /// landed mid-step.
    Empty,
    /// The drain errored mid-pop (accounting already mutated). Body read
    /// returns this error once.
    ///
    /// CONSUMED by its serve — `TransportError` is not `Clone` (see the type
    /// docs).
    ///
    /// Known RESIDUAL, stated here because this is the arm that carries it:
    /// once fire `j` of a burst has taken the error, fires `j+1..k` find the
    /// slot empty and fall to the LIVE arm, i.e. exactly the mid-step read
    /// serve-many removes everywhere else. It is bounded to the drain-error
    /// path — which already mutated accounting and is exceptional — and the
    /// alternative (re-serving one error to every fire of the step) is not
    /// available without making `TransportError` cloneable.
    Err(TransportError),
}

/// The result of one drain-and-account pass. `slot` is the surviving
/// sample (the frozen value); `popped` is the number of frames this
/// drain consumed (= the trigger arrival count — `signal_data` is called once
/// per popped frame); `latest_ts` is the wire timestamp_ns of the surviving
/// sample (= the `signal_input_received` watchdog reset value), None if Empty/Err.
struct DrainOutcome {
    slot: FrozenSlot,
    popped: u64,
    latest_ts: Option<u64>,
    /// True iff this drain's surviving frame was DECIMATED by the
    /// `sample(N)` gate (the one path that yields `Empty` with `popped > 0`).
    /// Capture-only metadata — the read-outcome classification needs to tell
    /// a decimated Empty from a genuinely-empty queue; nothing on the
    /// serve/accounting path reads it.
    decimated: bool,
}

/// A received message with zero-copy payload access.
///
/// The `header` is an owned copy (32 bytes on stack, from alignment-safe deserialization).
/// The `payload` is a reference directly into iceoryx2 shared memory — true zero-copy.
///
/// # Lifetime
///
/// The `'a` lifetime is tied to the iceoryx2 `Sample`. Once the callback returns,
/// the sample is dropped and the shared memory slot is freed.
pub struct ReceivedMessage<'a> {
    header: WireHeader,
    payload: &'a [u8],
}

impl<'a> ReceivedMessage<'a> {
    /// Create a new received message from header and payload.
    pub fn new(header: WireHeader, payload: &'a [u8]) -> Self {
        Self { header, payload }
    }

    /// Returns the wire header.
    pub fn header(&self) -> &WireHeader {
        &self.header
    }

    /// Returns the payload bytes (zero-copy reference into shared memory).
    pub fn payload(&self) -> &'a [u8] {
        self.payload
    }
}

/// Testing-only: a process-wide STICKY fault on the
/// listener notification drain. While armed, [`CerulionSubscriber::drain_event_notifications`]
/// returns `Err` WITHOUT draining and every internal stale-event drain is
/// skipped — precisely the hazard shape the rmw wait must survive: a
/// listener fd that stays readable while its drain fails, so a
/// level-triggered `poll(2)` wakes instantly on every block.
///
/// A PROCESS-GLOBAL static rather than a per-subscriber field, deliberately:
/// `CerulionSubscriber`'s layout is ABI-pinned (`abi_layout_pins` — it rides
/// inside `NodeContext` across the cdylib boundary), so a new field, even a
/// test seam, would be a `CERULION_ABI_VERSION` bump. The setter is
/// test-gated, so production never stores `true`; the load is one relaxed
/// atomic read per drain (the same class of cost as the
/// `fault_inject_receive_after` cell read on the receive path).
static FAULT_INJECT_DRAIN_EVENTS_ERR: AtomicBool = AtomicBool::new(false);

/// Arm / disarm the sticky listener-drain fault (see
/// [`FAULT_INJECT_DRAIN_EVENTS_ERR`]). Process-wide: every subscriber in
/// the process fails its drain while armed — pair with a disarm-on-drop
/// guard in the test.
#[cfg(any(test, feature = "test-helpers"))]
pub fn fault_inject_drain_events_err_for_test(armed: bool) {
    FAULT_INJECT_DRAIN_EVENTS_ERR.store(armed, Ordering::Release);
}

/// Event-based zero-copy subscriber for a single topic.
///
/// Uses iceoryx2's event service (Listener) for notification-based wakeup,
/// avoiding busy-polling. Each `wait_for_message` call blocks until an event
/// arrives or the timeout expires, then drains all available samples.
///
/// On creation, sends `SubscriberConnected` to notify publishers.
/// On Drop, sends `SubscriberDisconnected` for graceful cleanup.
pub struct CerulionSubscriber {
    topic: String,
    subscriber: Subscriber<CerService, [u8], ()>,
    listener: Listener<CerService>,
    notifier: Notifier<CerService>,
    /// This input's single backpressure probe (see
    /// [`BackpressureProbe`] for the per-variant semantics). `Some` on
    /// every graph-wired input — the runtime always installs one
    /// (`drop_oldest` is the default policy); `None` = a subscriber not
    /// wired as a node input (CLI introspection, direct transport users,
    /// or the runtime's internal data-trigger drain subscriber): no
    /// enforcement, the zero-copy direct path.
    probe: Option<BackpressureProbe>,
    /// The single edge-triggered [`BackpressureEvent`]
    /// pending for `#[on_event]` / `try_take_backpressure_event`.
    /// Set by whichever probe variant is installed — `sample(N)` first
    /// decimate, `drop_oldest` first eviction, `block` first at-threshold
    /// drain of a regime — cleared on take. Metadata only (NOT the SHM
    /// data path).
    pending_backpressure_event: Option<BackpressureEvent>,
    /// Fire-once fault-injection hook
    /// (testing-only) for the receive loops in
    /// `drain_to_latest_with_accounting` (reached via `try_view` /
    /// `snapshot_latest`) AND `drain_samples`
    /// (reached via `try_receive` / `wait_for_message` / the trigger-drain
    /// `try_receive_timestamps`), so the partial-batch staging posture is
    /// test-reachable. Mirrors
    /// `CerulionPublisher::fault_inject_publish_raw_after`:
    /// the FIELD is unconditional for FFI struct-layout stability
    /// (never `#[cfg]`-gate a field — the SIGSEGV lesson); the
    /// SETTER is test-gated, so the field stays `None` in production.
    /// `Cell` because `drain_samples` runs under `&self` (the `try_receive`
    /// family is a shared-borrow API) — single-threaded interior mutability,
    /// same as the drain paths' other `Cell` scratch. `Some(n)`: `n`
    /// `receive()` calls proceed normally (successful pops AND empty-queue
    /// `Ok(None)` calls both count), then the next one errors once and the
    /// hook clears — so `Some(0)` errors the very next call, `Some(1)` the
    /// second.
    fault_inject_receive_after: std::cell::Cell<Option<u32>>,
    /// The topic's EFFECTIVE iceoryx2
    /// `subscriber_max_borrowed_samples` — the borrow budget this
    /// subscriber's takes are really bounded by, read from the data
    /// service's static config at creation (the `DataOnlySubscriber`
    /// pattern; the iceoryx2 `Subscriber` port does not expose it). The
    /// rmw's adopt-take refusal diagnostics report THIS, not the
    /// requested create-leg floor, so the operator's remedy math is right
    /// against a pre-existing smaller service.
    max_borrowed_samples: usize,
    /// The topic's iceoryx2 `max_publishers` (read from
    /// the data service's static config at creation — the per-service
    /// truth, so it stays correct when later chunks set explicit per-topic
    /// values). Bounds the per-id baseline/scratch pre-sizing.
    max_publishers: usize,
    /// Reusable per-drain scratch for the per-publisher
    /// capture ([`StreamObservation`] per distinct origin id). Taken
    /// (`mem::take`) before each drain so the drain closures can fill it
    /// while `record_block_drained` borrows `&self`, and restored after —
    /// capacity is retained across drains (zero-alloc steady state).
    /// Reserved at probe registration for `2 × max_publishers` (live
    /// streams are bounded by `max_publishers`; recently-dead publishers'
    /// still-queued frames can briefly add more ids in one drain).
    drain_scratch: Vec<StreamObservation>,
    /// Shared `last_data_ns` anchor for this input's
    /// `#[input(expect_within_ms = N)]` watchdog. `Some` only when the
    /// graph runtime wired a watchdog on this input; the same
    /// `Arc<AtomicU64>` is held by the scheduler's `input_expect_within`
    /// tracker. On every sample DELIVERED to the node (in
    /// `drain_to_latest_with_accounting`, before `FrozenSlot::Sample` is
    /// returned), we write the sample's wire `timestamp_ns` here
    /// so the scheduler's next `step()` sees fresh data and does not
    /// false-count a miss. This covers NON-trigger inputs (the node reads
    /// them in its tick body); trigger inputs are additionally reset
    /// same-step by `drain_level`. Field is unconditional (FFI
    /// struct-layout stability — never `#[cfg]`-gate a field; the
    /// SIGSEGV lesson); `None` in production for un-watched inputs.
    expect_within_last_data_ns: Option<Arc<AtomicU64>>,
    /// This input is a PER-SET Sync TRIGGER — the align pass owns
    /// its pops, and its `expect_within` watchdog is deliberately a
    /// PRODUCER-LIVENESS surface: a raw ARRIVAL resets it, a
    /// `sample(N)`-DECIMATED arrival included. See the decimate arm in
    /// [`Self::drain_with_accounting_impl`] for why this is a marker rather
    /// than a widening of the delivered-sample rule.
    per_set_sync_trigger: bool,
    /// A step-boundary snapshot for a non-trigger
    /// latest-value input, and — since the trigger-drain unification — the boundary-drained FIFO head
    /// of a data-TRIGGER input. `Some` between the capture and the body's
    /// `try_view`, which serves it without re-draining (the accounting-once
    /// contract). `None` only for the probe-less DIRECT path (no capture site
    /// runs, so `try_view` drains live) and before a wired input's first
    /// capture.
    ///
    /// A serve does NOT clear this — the slot's lifetime is the
    /// STEP, so a `Data` burst's every fire reads the same frozen context. It
    /// is cleared-and-recaptured at the two capture sites (`snapshot_latest`,
    /// `drain_for_trigger`); the `Sample` and `Err` states are the two
    /// exceptions, consumed by their serve for reasons stated on
    /// [`FrozenSlot`].
    frozen: Option<FrozenSlot>,
    /// The SECOND slot of a two-slot FIFO buffer sitting in front of
    /// the iceoryx2 queue. Frames flow `queue → (frozen | next_head) →
    /// served/skipped`, strictly in order.
    ///
    /// It exists because the per-set Sync matcher has to learn the STAMP of the
    /// frame behind an input's head to decide whether advancing past that head
    /// would tighten the set's span — and a stamp can only be learned by
    /// POPPING. With one slot, the pop that learned the stamp would have to
    /// overwrite the head it was comparing against, i.e. destroy a frame that is
    /// still a member of an arrived complete set. This slot is where that popped
    /// frame is RETAINED instead.
    ///
    /// # The three structural rules
    ///
    /// * **R-pop** — the queue is popped into `next_head` ONLY while `frozen`
    ///   holds a [`FrozenSlot::Sample`] head, and popped into `frozen` ONLY
    ///   while `next_head` is Vacant. Never both in one call. `Sample`
    ///   specifically, not merely Occupied: that is what makes the states
    ///   `(Empty, Occupied)` and `(Err, Occupied)` UNCONSTRUCTIBLE, so the pair
    ///   is total over six reachable states rather than eight.
    /// * **R-promote** — whenever a trigger drain finds `frozen == None` and
    ///   `next_head` Occupied, it PROMOTES `next_head → frozen` and answers
    ///   `(1, promoted_ts)` without touching the queue. That branch lives inside
    ///   [`Self::drain_for_trigger`], so it serves BOTH drain sites (the next
    ///   step's boundary drain and the same-step burst refill) and needs no new
    ///   FFI — it rides the existing drain entry points.
    /// * **R-order** — `next_head` has exactly TWO exits, both in order:
    ///   promotion (the drain sites) and promote-serve ([`Self::try_view`]'s
    ///   live arm). It is never dropped silently and never bypassed by a queue
    ///   pop on any `&mut` serve path. With R-pop that gives the invariant
    ///   `stamp(frozen) <= stamp(next_head)` whenever both are Occupied
    ///   (single-writer monotone stamps; a `multi_publisher_topics` topic mixes
    ///   clocks and is outside it).
    ///
    /// # Why the borrow peak stays 2 — no provisioning change
    ///
    /// The pop-one drain holds a SINGLE transient (`drain_with_accounting_impl`
    /// keeps one `latest`) and moves it straight into a slot, so it never holds
    /// a `latest` local AND a receive transient at once the way the
    /// drain-to-LATEST loop does. The peak is therefore `frozen`(1) + one of
    /// {`next_head`, the transient}(1) = 2, which fits
    /// `ICEORYX2_DEFAULT_BORROWED_SAMPLES` (2)
    /// exactly: no `subscriber_max_borrowed_samples` raise, and no degrade on an
    /// external topic whose service somebody else created. The promote-serve
    /// adds no borrow either — it MOVES a sample that is already held.
    ///
    /// # Enforcement boundary
    ///
    /// R-order is unconstructible through any `&mut` serve path — the boundary
    /// drain, the burst refill, and `try_view`. It is NOT enforced against the
    /// `&self` [`Self::try_receive`] / [`Self::try_receive_one`] /
    /// [`Self::wait_for_message`] family, which also pops the queue and is
    /// deliberately NOT amended here. Such a read can pop PAST a staged
    /// `next_head` and invert this input's delivery order — but it is reachable
    /// only through a path [`Self::mark_unified_bound`] already warns is the
    /// wrong one ("the pre-step drain already consumed this queue"), whose
    /// existing loss mode on a unified input is frame loss. So the claim is:
    /// unconstructible through any `&mut` serve path, constructible only through
    /// a read the system already calls a misuse, loudly, on first use.
    next_head: Option<InboundSample>,
    /// Under the per-set backpressure contract: how many of this input's frames
    /// the matcher is currently HOLDING in the two slots above — 0, 1 or 2 —
    /// and therefore how much of its `block` occupancy the iceoryx2 queue no
    /// longer accounts for.
    ///
    /// `block`'s contract is "at most `depth` UNSERVED frames per edge", and
    /// the producer enforces it against the `outstanding` mirror, which a pop
    /// decrements ([`Self::record_block_drained`]). On every OTHER read path a
    /// pop and the node's read of that frame happen inside one `try_view`, so
    /// pop-time and serve-time coincide and the mirror IS occupancy. Under
    /// per-set Sync they come apart: the align pass pops a frame into
    /// [`Self::frozen`] (and a second into [`Self::next_head`]) where it can
    /// sit UNSERVED for many steps — a node whose partner has gone quiet holds
    /// its head indefinitely — while the mirror has already forgotten it. The
    /// producer would then run `depth + 2` frames ahead of the consumer while
    /// the declared contract says `depth`.
    ///
    /// So a frame ENTERING a slot re-credits the mirror by 1 and a frame
    /// LEAVING one (served, skipped, death-discarded, or dropped with the
    /// subscriber) debits it — the decrement MOVES from pop time to slot-exit
    /// time for exactly the frames that sit in a slot. Everything that does
    /// not land in a slot (decimated, junk-skipped, `Empty`, `Err`, a
    /// drain-to-latest drop) keeps the plain pop-time decrement: those frames
    /// have left the world.
    ///
    /// It is maintained by ONE method, [`Self::reconcile_block_slot_debt`],
    /// which RE-DERIVES it from the slots rather than counting `+1`/`-1` at
    /// each site: the invariant `debt == (frozen is Sample) + (next_head is
    /// Some)` is then the implementation instead of an assertion a new call
    /// site has to remember. Nonzero only on a `block`-probed
    /// [`ConsumeMode::EachFifo`] input, which today means a per-set Sync
    /// trigger (a `Data` trigger unifies only under `DropOldest`, and a
    /// `block` non-trigger input is excluded from the step-boundary snapshot).
    block_slot_debt: u8,
    /// Testing-only, fire-once: make the next
    /// [`Self::sync_next_arrived`] report a transport error.
    ///
    /// The gate's probe is the one place where `false` is not merely an answer
    /// but a PASS WITNESS — it says "this input has no second arrived frame",
    /// which is what PERMITS a descent. An error swallowed into `false` there
    /// is therefore not a lost diagnostic, it is a FABRICATED FACT that lets a
    /// descent run into an unprobed backlog and destroy arrived complete sets.
    /// Nothing can make a healthy iceoryx2 subscriber's `has_samples()` fail on
    /// demand, so that property is untestable without this seam.
    ///
    /// UNCONDITIONAL field, `#[cfg]`-gated SETTER: gating the field would
    /// differ this struct's layout between the host (which has the test
    /// feature through the dev-dependency self-ref) and a cdylib (which does
    /// not), and `CerulionSubscriber` crosses the FFI inside `NodeContext` —
    /// the SIGSEGV class. Always `false` in production, one branch.
    fault_inject_sync_next_arrived_err: bool,
    /// The last-DELIVERED sample for a non-trigger latest-value
    /// input, HELD across steps so the per-step snapshot can replay it when
    /// no new sample arrives (wait-for-first-delivery + hold-after). `None`
    /// before the first delivery (node waits) and for trigger / direct-path
    /// subscribers (which never hold). Borrowed by `try_view`'s `Held` arm
    /// via `SampleHandle::InboundRef`, so it must outlive the view — it lives
    /// here, in the subscriber. Pinning one sample raises the per-connection
    /// drain peak to 3 (held + latest + receive-transient); that headroom is
    /// budgeted by `subscriber_max_borrowed_samples = 3` on snapshot-source
    /// topics. Dropped on subscriber Drop (releases the SHM borrow).
    ///
    /// # Invariant (forward-only, NOT iff)
    ///
    /// `frozen == Some(FrozenSlot::Held) ⟹ held_sample.is_some()` — the
    /// implication is ONE-WAY: an `Empty` / `Err` frozen slot can coexist with
    /// a `Some` held value (a held value survives an Empty drain and a transient
    /// Err drain). (`frozen == Sample` happens ONLY for trigger / direct-path
    /// inputs, which never set `held_sample`, so a borrowed `Sample` frozen slot
    /// can NEVER coexist with a `Some` hold.) The field is MONOTONIC: it only
    /// ever goes
    /// `None → Some` (set on the first delivery, REPLACED by a newer sample,
    /// never reset to `None` before Drop). That monotonicity is exactly WHY the
    /// `try_view` and `view_raw` `Held` arms' missing-sample errors are
    /// invariant diagnostics — `Held` is only ever written by
    /// `snapshot_latest` AFTER `held_sample` is `Some`, and nothing clears it.
    /// There is no "un-hold":
    /// once delivered, the subscriber pins one SHM sample for life (intended; a
    /// liveliness-loss release is future scope).
    held_sample: Option<InboundSample>,
    /// Silent-failure guard: `Some((node_id, input_name))`
    /// when the graph runtime bound this input's data-trigger drain onto THIS
    /// body subscriber (`DrainSource::Unified`). Under that binding the
    /// pre-step drain consumes the queue into the frozen slot, which serves
    /// ONLY [`Self::try_view`] — a tick-body [`Self::try_receive`] observes an
    /// EMPTY queue and every frame is effectively lost to that read. The flag
    /// arms the warn-once enforcement in `try_receive` (the runtime rule:
    /// loud over silent). `None` (the default) for Separate-bound / opted-out
    /// / forced-Separate / non-node-input subscribers — the check is one
    /// never-taken branch, zero hot-path cost. Field is unconditional (never
    /// `#[cfg]`-gate a field — the FFI struct-layout lesson).
    unified_bound: Option<(Arc<str>, Arc<str>)>,
    /// Warn-once latch for the `unified_bound` `try_receive` misuse warn
    /// (`try_receive` takes `&self`, hence atomic). Latched on first warn so a
    /// per-tick `try_receive` does not warn per-step.
    unified_receive_warned: AtomicBool,
    /// The per-edge READ-OUTCOME stage — `Some` on every
    /// GRAPH-WIRED subscriber (body inputs AND the runtime's Separate/Sync
    /// trigger-drain subscribers), set at wiring time; `None` for direct /
    /// introspection subscribers, which are never part of the read log. The
    /// stage is SHARED (`Arc`) with the runtime/scheduler, which arms it only
    /// when a recording installs the trace ring and drains it at the
    /// level-end merge — so on a non-recording run every capture site is one
    /// `Option` test + one relaxed-class atomic load (the `unified_bound`
    /// cost class), with no allocation and no header parse beyond what the
    /// drain already does. Field is unconditional (never `#[cfg]`-gate a
    /// field — the FFI struct-layout lesson); adding it changed the
    /// FFI-crossed `NodeContext` layout, which is the v12 ABI bump.
    read_stage: Option<Arc<ReadOutcomeStage>>,
    /// Is this input's topic `multi_publisher_topics`-listed?
    ///
    /// `false` on every edge of every shipped graph and mp fixture (the opt-in
    /// list is empty there), and set at graph WIRING time by the runtime — the
    /// one site that holds the graph config. It gates the PRODUCER ANNOTATION
    /// (`ReadOutcomeKind::Producer`): the wire `sequence` is a PER-PUBLISHER
    /// counter, so on a topic with two writers a served seq names no
    /// publisher and the read log cannot say WHOSE frame was read — which is
    /// the whole reason the annotation exists. It is a per-subscriber bit
    /// rather than a per-read config lookup because it also gates
    /// `sample.origin()`, an iceoryx2 call the drain path makes only inside
    /// the eviction-probe branch otherwise: a single-publisher
    /// edge must pay one bool and nothing else.
    ///
    /// Field is unconditional (never `#[cfg]`-gate a field — the FFI
    /// struct-layout lesson).
    multi_publisher_edge: bool,
    /// How this subscriber's node-body read consumes the queue.
    ///
    /// `Latest` (the default): drain-to-latest — the newest sample is served
    /// and older queued samples are discarded. Correct for latest-value
    /// CONTEXT inputs (state samples where newer fully supersedes older) and
    /// for Sync-aligned trigger inputs (alignment serves the freshest frame
    /// in the window).
    ///
    /// `EachFifo`: pop exactly ONE sample per read, in FIFO arrival order —
    /// the per-message delivery contract for data-trigger inputs ("fire on
    /// each message arriving"). Set at graph WIRING time for every
    /// Data-policy trigger input (both Unified and Separate drain
    /// disciplines); the scheduler pairs it with one fire per pending frame
    /// (carrying the remainder across steps), so a burst of N frames yields
    /// N ticks each observing exactly one frame, in order.
    consume_mode: ConsumeMode,
    /// Consecutive BOUNDARY RE-OFFERS of the same unserved frozen
    /// FIFO head (see [`Self::snapshot_latest_for_trigger`]). Reset to 0 the
    /// moment either trigger drain actually pops — i.e. the moment a tick took
    /// the head. Only ever non-zero for an `EachFifo` input on the Unified
    /// drain discipline, which is the only place a head can be re-offered.
    ///
    /// The two sides are deliberately ASYMMETRIC: only a
    /// [`TriggerDrainSite::Boundary`] re-offer INCREMENTS it (a boundary runs
    /// exactly once per step, which is what makes the threshold a wall-clock
    /// bound rather than a function of burst depth), while EITHER site's pop
    /// RESETS it (a [`TriggerDrainSite::Refill`] that gets past the guard is
    /// proof the fire just run consumed the head).
    ///
    /// A re-offer is normal and expected while a fire is DEFERRED (a
    /// `throttle_ms` cap, a `block` gate). It is ALSO what a COLLAPSED read
    /// chain looks like — the node keeps firing but its tick never reaches
    /// this input's read — and **the subscriber cannot tell the two
    /// apart**: no fire signal reaches it, so from here a deferred head and an
    /// unread one are the same held frame re-offered on the same cadence. The
    /// streak is therefore a DEFER-DEPTH heuristic, not a diagnosis, and the
    /// warn it raises names both causes with the evidence that separates them.
    ///
    /// What the streak IS good for: the collapsed case is otherwise silent AND
    /// self-concealing. Each boundary re-offer mints a fresh signal whenever
    /// the node has none pending, so the arrival backlog never stays empty and
    /// the `expect_within_ms` backlog guard holds that input's
    /// liveliness watchdog suppressed for as long as it lasts — the producer
    /// could have died an hour ago. A long DEFER conceals the same watchdog
    /// for the same reason, which is why the line is worth raising in both
    /// cases even though it can only name a candidate.
    ///
    /// The cost of the heuristic: `throttle_ms` has a LOWER bound only
    /// (`cerulion_macros`'s validator rejects 0 and nothing else), and a
    /// `block` defer's length is set by downstream drain rather than by any
    /// declaration — which the runtime itself calls a legitimate steady state.
    /// So a node declaring a cap at or above the threshold, or one gated by a
    /// slow consumer, trips this once per defer cycle on a perfectly healthy
    /// graph. It stays once-per-regime with a recovery line at the next fire,
    /// so the volume is bounded at a pair per cycle rather than per boundary.
    held_head_reoffers: u64,
    /// Latch for the held-head `warn!` — one loud line per held-head regime,
    /// not one per boundary (the log-flood class), plus one
    /// recovery `info!` when the head is finally served.
    held_head_warned: bool,
    /// This input's SERVICE CURSOR — the wire `sequence` of the last
    /// frame this subscriber actually SERVED to the node's tick, so a mid-run
    /// anchor can state which pre-anchor frames a resumed replay must NOT
    /// re-inject.
    ///
    /// `Some` only for a per-message FIFO trigger input the graph runtime wired
    /// one onto ([`Self::register_service_cursor`]); `None` — the default — for
    /// every other subscriber, where the store is one never-taken branch.
    ///
    /// # The encoding is part of the ABI, not an implementation detail
    ///
    /// The value is `sequence as u64 + 1`, with **0 meaning "nothing served"**.
    /// `WireHeader::sequence` is a `u32` that is gap-free FROM ZERO at commit,
    /// so sequence 0 is a real frame and a bare `0` sentinel would
    /// make "nothing read yet" and "read frame 0" the same number — and the two
    /// take OPPOSITE reader rules (re-inject the first frame vs skip it), so the
    /// wrong branch starves the first resumed step. The `+ 1` cannot overflow:
    /// `u32::MAX as u64 + 1` is exactly representable.
    ///
    /// This field is written by code STATICALLY LINKED INTO EVERY CDYLIB (a
    /// node's own `try_view`), so the encoding is a contract between the host
    /// and every `.so` it loads, and changing it is a `CERULION_ABI_VERSION`
    /// bump. The field itself is unconditional for FFI struct-layout stability
    /// (never `#[cfg]`-gate a field — the SIGSEGV lesson).
    served_sequence: Option<Arc<AtomicU64>>,
}

/// Consecutive re-offers of one unserved FIFO head that constitute
/// a HELD-HEAD regime worth a loud line.
///
/// A boundary runs once per step, so 1024 boundaries is ~1 s at a 1 kHz loop —
/// deep enough that ordinary control-loop defers do not reach it (the shipping
/// `test_node_macro_qos_cdylib` shape, `throttle_ms = 5`, re-offers 4 times per
/// fire) while a tick that genuinely never reads its trigger is named within
/// seconds.
///
/// It is a THRESHOLD ON DEFER DEPTH and nothing stronger, because nothing
/// bounds a defer: `throttle_ms` is validated only against 0, and a `block`
/// gate lasts as long as its downstream queue stays full. A `throttle_ms` at or
/// above ~1025 ms on a 1 kHz graph, or a slow `block` consumer, WILL trip this
/// on a healthy node — so the warn names both candidates and the evidence that
/// tells them apart rather than asserting a wiring defect. Raising it would
/// only move which legitimate defers are affected while delaying the diagnosis
/// of the case that never recovers.
pub const HELD_HEAD_WARN_BOUNDARIES: u64 = 1024;

/// Queue-consumption mode for a node-body input read — see
/// [`CerulionSubscriber::consume_mode`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ConsumeMode {
    /// Drain the queue, serve the newest sample, discard the rest.
    Latest,
    /// Pop exactly one sample per read, FIFO order — per-message delivery.
    EachFifo,
}

/// WHICH site is asking a data-trigger input for a frame.
///
/// The two sites run the SAME drain and differ on exactly one thing: what an
/// UNSERVED frozen head means. At the level boundary it is a frame still owed a
/// fire, so it is RE-OFFERED; inside the Data burst loop's between-fires refill
/// it is proof the fire just run did NOT consume it, so the correct answer is
/// "nothing new" and the burst ends there.
///
/// DECLARED by the caller, never inferred from state — the
/// [`crate::transport::notify_delivery_latch::ListenerCountTiming`] discipline:
/// inferring which site is asking is precisely the silent inversion this type
/// exists to prevent, and the only state-side discriminator available (the
/// head's wire timestamp) is unusable, since two frames published in one step
/// under a `VirtualClock` legitimately share a stamp.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TriggerDrainSite {
    /// `drain_level`'s once-per-level-boundary drain, BEFORE decide.
    Boundary,
    /// `Scheduler::tick_data_burst`'s refill, BETWEEN two fires of one step.
    Refill,
}

impl CerulionSubscriber {
    /// Create a new subscriber.
    ///
    /// Sends `SubscriberConnected` event to notify any existing publishers,
    /// enabling them to deliver history to this late-joiner.
    ///
    /// Called by `TransportManager::create_subscriber()`. Not intended for direct use.
    pub(crate) fn new(
        topic: String,
        subscriber: Subscriber<CerService, [u8], ()>,
        listener: Listener<CerService>,
        notifier: Notifier<CerService>,
        max_publishers: usize,
        max_borrowed_samples: usize,
    ) -> Self {
        // Notify publishers that a new subscriber has connected
        let _ = notifier.notify_with_custom_event_id(PubSubEvent::SubscriberConnected.into());
        tracing::debug!(topic = %topic, "subscriber connected, sent SubscriberConnected event");

        Self {
            topic,
            subscriber,
            listener,
            notifier,
            probe: None,
            pending_backpressure_event: None,
            fault_inject_receive_after: std::cell::Cell::new(None),
            max_publishers,
            max_borrowed_samples,
            // hot-path-alloc-ok: constructor — once per subscriber. Empty (no
            // heap reservation) until a sample/drop_oldest probe is installed at
            // graph-build time, which `reserve`s it up front (see below).
            drain_scratch: Vec::new(),
            expect_within_last_data_ns: None,
            per_set_sync_trigger: false,
            frozen: None,
            next_head: None,
            block_slot_debt: 0,
            fault_inject_sync_next_arrived_err: false,
            held_sample: None,
            unified_bound: None,
            unified_receive_warned: AtomicBool::new(false),
            read_stage: None,
            multi_publisher_edge: false,
            consume_mode: ConsumeMode::Latest,
            held_head_reoffers: 0,
            held_head_warned: false,
            served_sequence: None,
        }
    }

    /// The topic's EFFECTIVE iceoryx2
    /// `subscriber_max_borrowed_samples` — see the field doc. What a take
    /// past it fails with (`ExceedsMaxBorrows`), which can be SMALLER than
    /// any create-leg floor the caller requested when the service already
    /// existed.
    pub fn max_borrowed_samples(&self) -> usize {
        self.max_borrowed_samples
    }

    /// Install this input's shared read-outcome stage.
    /// Called by the graph runtime at WIRING time (every graph-wired
    /// subscriber — body inputs and the Separate/Sync trigger-drain
    /// subscribers), before the context moves into the node entry. The
    /// runtime retains a clone: arming (recording only) and the level-end
    /// merge both happen through that side.
    pub(crate) fn set_read_outcome_stage(&mut self, stage: Arc<ReadOutcomeStage>) {
        self.read_stage = Some(stage);
    }

    /// Is read-outcome capture LIVE on this subscriber — a stage is
    /// wired AND a recording armed it? Checked BEFORE any capture-only work
    /// (the extra served-seq header parse included), so a non-recording run
    /// pays exactly this branch pair per drain.
    #[inline]
    fn read_capture_armed(&self) -> bool {
        self.read_stage.as_ref().is_some_and(|s| s.is_armed())
    }

    /// Mark this input's topic as `multi_publisher_topics`-listed
    /// — the ONLY edges whose reads carry a
    /// [`ReadOutcomeKind::Producer`] annotation. Set by the graph runtime at
    /// WIRING time (the one site that has the graph config), so the decision
    /// is a per-subscriber BIT rather than a per-read lookup: a
    /// single-publisher edge's hot path pays one already-loaded bool and its
    /// read log is byte-identical to one without the annotation.
    pub(crate) fn mark_multi_publisher_edge(&mut self) {
        self.multi_publisher_edge = true;
    }

    /// Should THIS read carry a producer annotation? Both
    /// conjuncts are load-bearing: the armed check keeps a
    /// non-recording run free of any origin read at all, and the
    /// multi-publisher bit keeps `sample.origin()` — an iceoryx2 call the hot
    /// path does not otherwise make outside the eviction-probe branch — off
    /// every single-publisher edge, which is every edge on every shipped
    /// graph and mp fixture.
    #[inline]
    fn capture_producer_token(&self) -> bool {
        self.multi_publisher_edge && self.read_capture_armed()
    }

    /// Stage one read outcome (capture-armed callers only — the
    /// caller has already checked [`Self::read_capture_armed`]).
    ///
    /// `role` is the CALL-SITE role, passed as a compile-time
    /// constant by every mint arm. It is never derived from the stage — one
    /// stage legitimately holds both roles' records under the unified
    /// discipline — and never inferred from state.
    #[inline]
    fn stage_read_outcome(
        &self,
        kind: ReadOutcomeKind,
        served_seq: Option<u32>,
        popped: u64,
        role: ReadSiteRole,
    ) {
        if let Some(stage) = &self.read_stage {
            stage.record(kind, served_seq, popped, role);
        }
    }

    /// Stage one read outcome PRECEDED by its producer
    /// annotation (callers that have already checked
    /// [`Self::capture_producer_token`]). The pair is admitted atomically at
    /// the staging rim — see `ReadOutcomeStage::record_with_producer`.
    #[inline]
    fn stage_read_outcome_with_producer(
        &self,
        kind: ReadOutcomeKind,
        served_seq: Option<u32>,
        popped: u64,
        token: u64,
        role: ReadSiteRole,
    ) {
        if let Some(stage) = &self.read_stage {
            stage.record_with_producer(kind, served_seq, popped, token, role);
        }
    }

    /// Mark this subscriber as a per-message FIFO consumer (a Data-policy
    /// trigger input). Called by the graph runtime at WIRING time — the one
    /// site that knows which inputs are data-trigger edges. After this, the
    /// node-body read ([`Self::try_view`] with no frozen slot, and the
    /// boundary [`Self::snapshot_latest_for_trigger`]) pops exactly ONE
    /// frame per call in FIFO order instead of draining to the latest.
    pub(crate) fn mark_fifo_consume(&mut self) {
        self.consume_mode = ConsumeMode::EachFifo;
    }

    /// Declare this input a PER-SET Sync TRIGGER. Called by the graph
    /// runtime at WIRING time beside [`Self::mark_fifo_consume`], on the
    /// per-set arm only.
    ///
    /// It cannot be INFERRED from `ConsumeMode::EachFifo`: serve-many put a Data
    /// node's per-message trigger on the same mode, and the two want OPPOSITE
    /// answers about a decimated arrival — the Data path keeps the pinned
    /// delivered-sample rule (`expect_within_iox2_test`'s sample-gate case),
    /// while a per-set Sync trigger resets on the raw arrival.
    pub(crate) fn mark_per_set_sync_trigger(&mut self) {
        self.per_set_sync_trigger = true;
    }

    /// Install this input's SERVICE CURSOR (see
    /// [`Self::served_sequence`]). Called by the graph runtime at WIRING time,
    /// beside [`Self::mark_fifo_consume`], so exactly the per-message FIFO
    /// inputs carry one; the same `Arc` is held by the runtime, which reads it
    /// at the anchor boundary.
    pub(crate) fn register_service_cursor(&mut self, cursor: Arc<AtomicU64>) {
        self.served_sequence = Some(cursor);
    }

    /// Record that `raw` was SERVED to the node.
    ///
    /// Called only after a frame has really been handed to user code — never at
    /// pop. A frame popped into the frozen slot and not served is re-offered at
    /// the next boundary, and a rebuilt subscriber has no frozen slot, so
    /// advancing at pop would tell a resume to skip a frame it must re-inject.
    ///
    /// Cost: one predictable branch (`None` for every subscriber that is not a
    /// per-message FIFO trigger input), then one header parse and one `Release`
    /// store. No allocation, no transport call.
    #[inline]
    fn record_service_cursor(&self, raw: &[u8]) {
        let Some(cursor) = self.served_sequence.as_ref() else {
            return;
        };
        if let Some(header) = WireHeader::read_from_buf(raw) {
            cursor.store(u64::from(header.sequence) + 1, Ordering::Release);
        }
    }

    /// Mark this subscriber as the body half of a
    /// `DrainSource::Unified` data-trigger binding. Called by the graph
    /// runtime at WIRING time (the Unified arm only — never for
    /// forced-Separate or `with_unified_drain(false)` opted-out inputs), so
    /// the queue-draining read paths ([`Self::try_receive`] /
    /// [`Self::try_receive_one`] / [`Self::try_receive_one_owned`] /
    /// [`Self::wait_for_message`]) can warn once when a tick body reads the
    /// input through the wrong path.
    pub(crate) fn mark_unified_bound(&mut self, node_id: Arc<str>, input: Arc<str>) {
        self.unified_bound = Some((node_id, input));
    }

    /// The warn-once body for a queue-draining read on a unified-bound input.
    /// The sibling read paths (`try_receive` / `try_receive_one` /
    /// `try_receive_one_owned` / `wait_for_message`) share the SAME loss mode (the pre-step drain
    /// already consumed the queue) and the SAME latch, so exactly ONE warn
    /// fires per input regardless of which misuse method is hit first;
    /// `method` names the actual call site in the message. Kept out-of-line
    /// (`#[cold]`) so each hot read path pays exactly one pointer-sized
    /// `Option` test when the input is not unified-bound.
    #[cold]
    #[inline(never)]
    fn warn_unified_receive_misuse(&self, method: &'static str) {
        // `swap` latches: only the FIRST caller observes `false` and warns.
        if self.unified_receive_warned.swap(true, Ordering::Relaxed) {
            return;
        }
        if let Some((node_id, input)) = &self.unified_bound {
            tracing::warn!(
                topic = %self.topic,
                node_id = %node_id,
                input = %input,
                method = %method,
                "this read method (see the `method` field) was called on a \
                 UNIFIED-drained trigger input: the pre-step drain already consumed this \
                 queue into the frozen slot, which serves ONLY try_view, so the call \
                 observes an empty queue (frames are effectively lost to this read). \
                 Read via try_view, or opt out of unification with \
                 with_unified_drain(false). Warning once per input."
            );
        }
    }

    /// Test-only setter for the fire-once receive fault (see the
    /// `fault_inject_receive_after` field doc). `after = 0` errors the very
    /// next `receive()` call inside `drain_to_latest_with_accounting` OR
    /// `drain_samples` (whichever drain runs next).
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn fault_inject_receive_after_for_test(&mut self, after: u32) {
        self.fault_inject_receive_after.set(Some(after));
    }

    /// `sample(N)`: install the subscriber read-gate. Called by the
    /// graph runtime for `sample(N)` inputs. `interval_ms` is the minimum
    /// spacing between accepted reads.
    pub(crate) fn register_sample_gate(
        &mut self,
        interval_ms: u64,
        counters: Arc<BackpressureCounters>,
        node_id: Arc<str>,
        input: Arc<str>,
        buffer_capacity: usize,
    ) {
        self.assert_no_probe_installed();
        self.probe = Some(BackpressureProbe::Sample(SampleGate {
            interval_ns: interval_ms.saturating_mul(1_000_000),
            interval_ms,
            last_accepted_ns: None,
            counters,
            node_id,
            input,
            armed: true,
            regime_started_ns: 0,
            regime_count: 0,
            buffer_capacity,
        }));
    }

    /// `drop_oldest`: install the silent-eviction detector. Called by
    /// the graph runtime for `drop_oldest` inputs (the default policy), so
    /// `drop_oldest_count` is real and an `#[on_event(input = "...")]` handler (with a `BackpressureEvent` parameter) on a
    /// `drop_oldest` input fires when iceoryx2 reclaims the oldest on overflow.
    /// Per-id baselines + the drain scratch are pre-sized here from the
    /// topic's `max_publishers` so the hot path never allocates.
    pub(crate) fn register_drop_oldest_probe(
        &mut self,
        counters: Arc<BackpressureCounters>,
        node_id: Arc<str>,
        input: Arc<str>,
        buffer_capacity: usize,
    ) {
        self.assert_no_probe_installed();
        // hot-path-alloc-ok: probe install runs once per input at graph-build
        // time, never on the receive hot path. Reserving drain_scratch +
        // baselines up front is exactly what keeps the drain loop alloc-free.
        self.drain_scratch.reserve(self.max_publishers * 2);
        self.probe = Some(BackpressureProbe::DropOldest(DropOldestProbe {
            // hot-path-alloc-ok: probe install (graph-build time), not the drain
            // hot path — reserved to capacity so the drain loop never grows it.
            baselines: Vec::with_capacity(self.max_publishers),
            max_baselines: self.max_publishers,
            drain_counter: 0,
            counters,
            node_id,
            input,
            armed: true,
            backward_run: 0,
            regime_started_ns: 0,
            regime_count: 0,
            buffer_capacity,
        }));
    }

    /// Test-only public wrapper over `register_drop_oldest_probe` —
    /// integration tests exercise the eviction detector against real
    /// iceoryx2 publishers, including the restart / multi-publisher
    /// identity anomalies that `GraphRuntime` wiring deliberately cannot
    /// express (topology validation rejects double-producer topics).
    /// Gated like [`NodeContext::for_tests`](crate::graph::node::NodeContext)
    /// so it cannot be used in production builds.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn register_drop_oldest_probe_for_test(
        &mut self,
        counters: Arc<BackpressureCounters>,
        node_id: Arc<str>,
        input: Arc<str>,
        buffer_capacity: usize,
    ) {
        self.register_drop_oldest_probe(counters, node_id, input, buffer_capacity);
    }

    /// Test-only public wrapper over `register_sample_gate` — integration
    /// tests exercise the read-gate (and the at-most-one-probe wiring
    /// assert, which all three `register_*` methods share) against real
    /// iceoryx2. Gated like [`Self::register_drop_oldest_probe_for_test`].
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn register_sample_gate_for_test(
        &mut self,
        interval_ms: u64,
        counters: Arc<BackpressureCounters>,
        node_id: Arc<str>,
        input: Arc<str>,
        buffer_capacity: usize,
    ) {
        self.register_sample_gate(interval_ms, counters, node_id, input, buffer_capacity);
    }

    /// Test-only public wrapper over `register_block_probe` — same gating
    /// rationale as [`Self::register_sample_gate_for_test`].
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn register_block_probe_for_test(
        &mut self,
        outstanding: crate::credit::CreditWord,
        threshold: u64,
        counters: Arc<BackpressureCounters>,
        input: Arc<str>,
        buffer_capacity: usize,
    ) {
        self.register_block_probe(outstanding, threshold, counters, input, buffer_capacity);
    }

    /// Test-only public wrapper over [`Self::register_expect_within`] — lets an
    /// integration test install the `expect_within` shared anchor and observe
    /// that `snapshot_latest` / `try_view` writes the delivered sample's wire
    /// timestamp into it EXACTLY ONCE (at snapshot time, not on the frozen-slot
    /// view). Gated like [`Self::register_drop_oldest_probe_for_test`].
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn register_expect_within_anchor_for_test(&mut self, anchor: Arc<AtomicU64>) {
        self.register_expect_within(anchor);
    }

    /// Take the edge-triggered [`BackpressureEvent`] queued
    /// by this subscriber's backpressure probe, if any. All three policies
    /// queue events: `sample(N)` on the first decimate of a regime,
    /// `drop_oldest` on the first detected eviction of a regime
    /// (`dropped > 0`), and `block` on the first at-threshold drain of a
    /// regime (`dropped == 0` — lossless flow-control signal). One event per
    /// regime (the next accept / clean / below-threshold drain rearms). The
    /// `#[on_event]` macro drains this after each tick; users may
    /// also poll it directly via `ctx.take_backpressure_event("input_name")`.
    /// Returns `None` for a subscriber with no probe or no pending event.
    /// Metadata only — no data path, no copy.
    #[must_use = "BackpressureEvent is a user-visible data-loss signal; discarding it silently loses observability"]
    pub fn try_take_backpressure_event(&mut self) -> Option<BackpressureEvent> {
        self.pending_backpressure_event.take()
    }

    /// Wire this input's `#[input(expect_within_ms = N)]`
    /// watchdog anchor. The graph runtime mints the `Arc<AtomicU64>`, hands
    /// the same handle to the scheduler's `input_expect_within` tracker, and
    /// installs it here so every sample DELIVERED to the node (in
    /// [`Self::drain_to_latest_with_accounting`], before `FrozenSlot::Sample`
    /// is returned) writes the sample's wire
    /// `timestamp_ns` into it — resetting the watchdog window without a
    /// scheduler round-trip. Independent of the backpressure probe (an input
    /// may have a watchdog AND any backpressure policy). No-op unless wired.
    pub(crate) fn register_expect_within(&mut self, last_data_ns: Arc<AtomicU64>) {
        self.expect_within_last_data_ns = Some(last_data_ns);
    }

    /// `block`: install the consumer-side probe. Called by the graph
    /// runtime for `block` consumers on all-`block` topics. The `outstanding`
    /// `Arc` is the same one registered on the producer's publisher (publish
    /// increment) + its pre-fire check (fullness read); `counters` is the same
    /// set the producer bumps on defer (the event only reads it).
    pub(crate) fn register_block_probe(
        &mut self,
        outstanding: crate::credit::CreditWord,
        threshold: u64,
        counters: Arc<BackpressureCounters>,
        input: Arc<str>,
        buffer_capacity: usize,
    ) {
        self.assert_no_probe_installed();
        // The block arm stamps its regime from the drain scratch's
        // high-water timestamp — reserve it here too (block topics are
        // single-producer in-graph, but the capture is shared machinery).
        // hot-path-alloc-ok: probe install runs once per input at graph-build
        // time, never on the receive hot path — the exact twin of the reserve in
        // `register_drop_oldest_probe`. Reserving to capacity here is what keeps
        // the drain loop alloc-free.
        self.drain_scratch.reserve(self.max_publishers * 2);
        self.probe = Some(BackpressureProbe::Block(BlockProbe {
            outstanding,
            threshold,
            counters,
            input,
            armed: true,
            regime_started_ns: 0,
            regime_count: 0,
            buffer_capacity,
        }));
    }

    /// An input carries exactly ONE backpressure policy, so at most
    /// one probe may ever be installed — and never twice. Two-at-once is
    /// unrepresentable since the [`BackpressureProbe`] collapse;
    /// what stays representable is registering twice, which would silently
    /// REPLACE live probe state — for `block`, that orphans the shared
    /// `outstanding` mirror (drains stop decrementing it while the producer
    /// keeps incrementing), so the producer defers FOREVER while the
    /// per-defer warn keeps mis-reporting routine "no data lost" flow
    /// control, never the orphaned mirror. Registration is cold-path wiring
    /// code (once per input at graph build), so this is a hard `assert!` in
    /// ALL builds: a wiring bug fails loudly at build time, never as a
    /// permanent stall masquerading as routine backpressure.
    fn assert_no_probe_installed(&self) {
        assert!(
            self.probe.is_none(),
            "a backpressure probe is already installed on this input — at most one \
             probe per input, installed at most once"
        );
    }

    /// The `block` probe, when this input is wired as `block`.
    #[inline]
    fn block_probe(&self) -> Option<&BlockProbe> {
        match &self.probe {
            Some(BackpressureProbe::Block(p)) => Some(p),
            _ => None,
        }
    }

    /// Mutable [`Self::block_probe`].
    #[inline]
    fn block_probe_mut(&mut self) -> Option<&mut BlockProbe> {
        match &mut self.probe {
            Some(BackpressureProbe::Block(p)) => Some(p),
            _ => None,
        }
    }

    /// The `drop_oldest` eviction detector, when this input is wired as
    /// `drop_oldest`.
    #[inline]
    fn drop_oldest_probe_mut(&mut self) -> Option<&mut DropOldestProbe> {
        match &mut self.probe {
            Some(BackpressureProbe::DropOldest(p)) => Some(p),
            _ => None,
        }
    }

    /// The `sample(N)` read-gate, when this input is wired as `sample(N)`.
    #[inline]
    fn sample_gate_mut(&mut self) -> Option<&mut SampleGate> {
        match &mut self.probe {
            Some(BackpressureProbe::Sample(g)) => Some(g),
            _ => None,
        }
    }

    /// `block`: decrement the outstanding mirror by the number of
    /// samples just removed from the iceoryx2 queue (saturating — the
    /// mirror must never underflow). No-op unless `block` is wired.
    #[inline]
    fn record_block_drained(&self, removed: u64) {
        if removed == 0 {
            return;
        }
        if let Some(counter) = self.block_probe().map(|p| &p.outstanding) {
            // Saturating: an interleaving non-drain removal (history
            // replay on a late joiner) could otherwise drive the mirror
            // below zero. Clamping keeps "outstanding == 0" the floor.
            //
            // `record_drained` IS that saturating `fetch_update`,
            // and it additionally RINGS the word's wake epoch. Ringing is not
            // decoration on a split edge: freeing credit is precisely the event
            // a credit-blocked producer in another process parks for, and a
            // decrement that did not ring would leave it asleep until its next
            // slice — the wedge the wake word exists to prevent. On a LOCAL
            // word nobody can be parked, so the ring is a `wake_seq` bump plus
            // a `parked`-mask read and no syscall.
            counter.record_drained(removed);
        }
    }

    /// Re-state this input's `block` occupancy from the SLOTS, after
    /// anything that could have changed which frames the matcher is holding.
    ///
    /// The invariant is `block_slot_debt == (frozen is Sample) + (next_head is
    /// Some)`, and this method establishes it by RE-DERIVING the left side from
    /// the right rather than by applying a `+1`/`-1` at each of the five sites
    /// that move a frame between the queue, the slots and the node. That choice
    /// is the whole safety argument: a hand-placed pair can be dropped at one
    /// site (the producer then either runs ahead of its declared `depth` or —
    /// far worse — LIVELOCKS behind a mirror nothing ever gives back), whereas a
    /// re-derivation is idempotent, can be called anywhere, and is correct at a
    /// site nobody thought of. What a missing CALL costs is bounded and
    /// self-healing: the debt is restated at the next call on that input.
    ///
    /// The mirror moves in the SAME direction the frames did:
    ///
    /// * a frame that entered a slot was already decremented at its pop, so the
    ///   mirror is credited back by 1 — occupancy did not fall, it moved;
    /// * a frame that left a slot really has been consumed, so the mirror is
    ///   debited (saturating — `outstanding == 0` is the floor, and a lost
    ///   decrement must never be able to wrap it to `u64::MAX` and defer the
    ///   producer forever).
    ///
    /// Every credit can only push `outstanding` UP, i.e. defer the producer
    /// EARLIER, so the failure direction is conservative: Principle #6 tightens
    /// rather than loosens.
    ///
    /// MEASURED REDUNDANCY, recorded because it is easy to mistake for coverage.
    /// The call at the boundary fill is TOTAL on its own: it runs every level
    /// boundary and re-derives from the slots, so it repairs whatever any other
    /// site missed before the producer next reads the mirror (its `decide` runs
    /// at an earlier level of the FOLLOWING step). Deleting the serve exit, the
    /// discard exits, or both leaves every end-to-end arm green and fails
    /// only the subscriber walk, which observes
    /// between transitions. So the other four calls do not buy correctness at a
    /// boundary; they buy the invariant holding CONTINUOUSLY, which is what the
    /// debug assert rests on and what any mid-step reader of `outstanding` sees
    /// (the consumer's own block event samples it pre-drain). Deleting the
    /// boundary ENTRY call is a different matter and is observable end to end:
    /// on a starved input no other site runs at all, so nothing repairs it and
    /// the producer publishes `depth + 1`.
    ///
    /// A no-op unless a `block` probe is installed AND the input is per-message
    /// FIFO. Both guards are load-bearing rather than defensive: `sample(N)` and
    /// `drop_oldest` inputs carry no mirror to restate, and a
    /// [`ConsumeMode::Latest`] input's `frozen` slot is the step-boundary
    /// snapshot, whose accounting contract is pinned elsewhere and is not this
    /// feature's to change.
    fn reconcile_block_slot_debt(&mut self) {
        if self.consume_mode != ConsumeMode::EachFifo || self.block_probe().is_none() {
            debug_assert_eq!(
                self.block_slot_debt, 0,
                "block slot debt is only ever taken on a block-probed EachFifo input"
            );
            return;
        }
        let held = u8::from(matches!(self.frozen, Some(FrozenSlot::Sample(_))))
            + u8::from(self.next_head.is_some());
        let debt = self.block_slot_debt;
        if held == debt {
            return;
        }
        if let Some(probe) = self.block_probe() {
            if held > debt {
                // The credit-BACK direction — the same
                // `fetch_add(Release)` this site wrote inline, through the
                // word's own API so a mapped edge moves the SHARED page.
                probe.outstanding.record_published_n(u64::from(held - debt));
            } else {
                // The RELEASE direction really frees a slot, so it goes through
                // `record_drained` and therefore RINGS the wake word — a
                // decrement that freed room without ringing would leave a
                // cross-process producer parked on room that already exists.
                probe.outstanding.record_drained(u64::from(debt - held));
            }
        }
        self.block_slot_debt = held;
    }

    /// Testing-only public wrappers over the four per-set head ops
    /// and the FIFO consume mode, so an INTEGRATION test can drive one input
    /// through every slot transition by hand.
    ///
    /// They exist because the walk they enable cannot live in this module: it
    /// has to `try_view` a real message type, every one of which lives in
    /// `native_ros2_messages` — a crate that DEPENDS on this one, so under
    /// `cfg(test)` its `ShmMessage` impls are against a different build of
    /// `cerulion_core` and do not satisfy `try_view`'s bound. Same gating
    /// rationale as [`Self::register_block_probe_for_test`].
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn mark_fifo_consume_for_test(&mut self) {
        self.mark_fifo_consume();
    }

    /// Test-only wrapper over `sync_peek_next_stamp`. See
    /// [`Self::mark_fifo_consume_for_test`].
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn sync_peek_next_stamp_for_test(&mut self) -> TransportResult<Option<u64>> {
        self.sync_peek_next_stamp()
    }

    /// Test-only wrapper over `sync_discard_head`. See
    /// [`Self::mark_fifo_consume_for_test`].
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn sync_discard_head_for_test(&mut self) -> TransportResult<Option<u64>> {
        self.sync_discard_head()
    }

    /// Test-only wrapper over `sync_void_head`. See
    /// [`Self::mark_fifo_consume_for_test`].
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn sync_void_head_for_test(&mut self) {
        self.sync_void_head();
    }

    /// Testing-only: this input's current [`Self::block_slot_debt`],
    /// the observable the slot-walk oracles assert the invariant on directly,
    /// rather than inferring it from the producer's publish count.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn block_slot_debt_for_test(&self) -> u8 {
        self.block_slot_debt
    }

    /// Wait for messages and invoke callback for each one received.
    ///
    /// Blocks until a data event (`SentSample` or `SentHistory`) arrives or `timeout`
    /// expires, then drains all available samples. Non-data events (e.g.,
    /// `SubscriberConnected` from self) are silently skipped to avoid spurious wakeups.
    ///
    /// # Drain-on-Timeout
    ///
    /// Even on timeout, we drain the queue. This handles the send→notify race where
    /// data arrives before the notification (Principle #6: no data loss).
    ///
    /// # Returns
    ///
    /// `Ok(count)` — number of messages delivered (0 on timeout with empty queue).
    ///
    /// **Backpressure-probe caveat:** [`Self::try_view`] is the
    /// sole probe-aware read path. Draining an input the runtime wired with
    /// a `drop_oldest` probe through this method consumes frames the probe
    /// never observes, desynchronizing its sequence baseline — eviction
    /// counts after such a drain are unreliable until the baseline
    /// re-establishes. Mixing read paths on one probed input is a caller
    /// error.
    #[must_use = "receive result must be checked"]
    pub fn wait_for_message<F>(&self, timeout: Duration, mut callback: F) -> TransportResult<usize>
    where
        F: FnMut(ReceivedMessage<'_>),
    {
        // Same misuse warn as `try_receive`: a
        // unified-bound input's queue was consumed by the pre-step drain, so
        // this blocking read sees nothing until timeout. Shared latch: one
        // warn per input across all four queue-draining read paths.
        if self.unified_bound.is_some() {
            self.warn_unified_receive_misuse("wait_for_message");
        }
        // Drain any stale events from the listener before waiting.
        // This prevents spurious wakeups from previously queued events
        // (e.g., SentSample from a prior publish cycle, SubscriberConnected
        // from our own constructor, etc.)
        self.drain_stale_events();

        // Also drain any already-available samples (send→notify race from prior publishes)
        let pre_count = self.drain_samples(&mut callback, ReadSiteRole::Body)?;
        if pre_count > 0 {
            return Ok(pre_count);
        }

        // Track remaining time for re-waits after spurious/non-data events
        let deadline = std::time::Instant::now() + timeout;

        loop {
            let remaining = deadline.saturating_duration_since(std::time::Instant::now());
            if remaining.is_zero() {
                // Timeout reached — drain any pending samples and return
                return self.drain_samples(&mut callback, ReadSiteRole::Body);
            }

            // Block on event listener (or remaining timeout)
            let event =
                self.listener
                    .timed_wait_one(remaining)
                    .map_err(|e| TransportError::Receive {
                        topic: self.topic.clone(),
                        reason: format!("{}", e),
                    })?;

            match event {
                Some(event_id) => {
                    let parsed = PubSubEvent::try_from(event_id);
                    match parsed {
                        Ok(PubSubEvent::SentSample) | Ok(PubSubEvent::SentHistory) => {
                            // Data event — drain samples and return
                            return self.drain_samples(&mut callback, ReadSiteRole::Body);
                        }
                        _ => {
                            // Non-data event. Drain to check for any pending data
                            // (send→notify race), but only return if we got data.
                            let count = self.drain_samples(&mut callback, ReadSiteRole::Body)?;
                            if count > 0 {
                                return Ok(count);
                            }
                            continue;
                        }
                    }
                }
                None => {
                    // Timeout — drain any pending samples (send→notify race)
                    return self.drain_samples(&mut callback, ReadSiteRole::Body);
                }
            }
        }
    }

    /// Try to receive messages without blocking.
    ///
    /// Drains stale events from the listener (preventing socket buffer overflow),
    /// then drains all available samples from the subscriber queue.
    /// Returns immediately with `Ok(0)` if no messages are available.
    ///
    /// **Backpressure-probe caveat:** see
    /// [`Self::wait_for_message`] — draining a probe-wired input here
    /// bypasses the eviction detector and desynchronizes its baseline.
    #[must_use = "receive result must be checked"]
    pub fn try_receive<F>(&self, mut callback: F) -> TransportResult<usize>
    where
        F: FnMut(ReceivedMessage<'_>),
    {
        // A unified-bound trigger input's queue is
        // consumed by the pre-step drain — this read path silently loses every
        // frame. Warn ONCE, loudly (the check is one branch when not bound).
        if self.unified_bound.is_some() {
            self.warn_unified_receive_misuse("try_receive");
        }
        self.drain_stale_events();
        self.drain_samples(&mut callback, ReadSiteRole::Body)
    }

    /// [`Self::try_receive`], performed by the SCHEDULER on the
    /// node's behalf — the Separate data-trigger / Sync trigger drain
    /// (`TriggerSubscriber::try_receive_timestamps`, its ONLY caller).
    ///
    /// # Why this exists rather than a role argument on `try_receive`
    ///
    /// `try_receive` is `pub` and reached by node code, by the TUI, by the
    /// service layer and by demo nodes; the trigger drain reached the SAME
    /// function, so the drain and the body were indistinguishable INSIDE it —
    /// which is precisely the site ambiguity the role bits exist to remove
    /// (`replay_rederive::StandDownReason::AmbiguousReadSite` is that
    /// ambiguity, offline). A sibling entry point keeps the role a
    /// COMPILE-TIME CONSTANT at every mint, so no site infers it and no caller
    /// can pass the wrong one by omission: everything that does not name this
    /// function is a body read, which is the correct default.
    ///
    /// Behaviourally IDENTICAL to `try_receive` — same stale drain, same batch,
    /// same frames, same order. The ONLY difference is two bits in the recorded
    /// read log, and a non-recording run does not even reach the staging
    /// branch.
    ///
    /// It carries NO unified-bound misuse warn, unlike its three siblings, and
    /// that is a reachability fact rather than an omission: `unified_bound` is
    /// set by [`Self::mark_unified_bound`] on the node's BODY subscriber and
    /// only on the `DrainSource::Unified` arm, while this function's only
    /// caller is `TriggerSubscriber::try_receive_timestamps` on a
    /// `TriggerSubscriber::Ipc` — a subscriber the runtime MINTS separately at
    /// the two `Separate`/`Sync` wiring sites and never marks. (The Unified arm
    /// pushes a `ListenerOnly`, which has no data queue to drain and takes the
    /// loud desync arm in that caller instead.) A warn here could not fire, so
    /// it was deleted rather than kept as a comment claiming otherwise.
    #[must_use = "receive result must be checked"]
    pub(crate) fn try_receive_for_drain<F>(&self, mut callback: F) -> TransportResult<usize>
    where
        F: FnMut(ReceivedMessage<'_>),
    {
        self.drain_stale_events();
        self.drain_samples(&mut callback, ReadSiteRole::Drain)
    }

    /// Drain all stale events from the listener without blocking.
    ///
    /// This clears any queued event notifications (e.g., SentSample from a prior
    /// publish, SubscriberConnected from our constructor) so `timed_wait_one` will
    /// only wake on genuinely new events.
    fn drain_stale_events(&self) {
        // The consuming read paths never blocked on the listener's fd, so a
        // drain error there costs at most one stale wake later — they
        // swallow it. The fd-BLOCKING caller goes through
        // `drain_event_notifications`, which surfaces the error instead.
        let _ = self.try_drain_stale_events();
    }

    /// The fallible core of the notification drain: `Ok(())` once the
    /// listener's event queue is empty, `Err` the moment `try_wait_one`
    /// fails — WITHOUT retrying, because a listener whose drain fails while
    /// its fd stays readable is the hazard a blocking caller must know
    /// about (a swallowed error here turns a level-triggered
    /// `poll(2)` wait into a busy-spin until its deadline).
    fn try_drain_stale_events(&self) -> TransportResult<()> {
        if FAULT_INJECT_DRAIN_EVENTS_ERR.load(Ordering::Relaxed) {
            // Test seam: report the failure AND leave the queue undrained,
            // which is exactly what a real persistent `try_wait_one` error
            // does to the fd.
            return Err(TransportError::Receive {
                // hot-path-alloc-ok: test-seam error arm, never taken in
                // production (the setter is cfg-gated).
                topic: self.topic.clone(),
                reason: "fault-injected listener drain failure (test seam)".to_string(),
            });
        }
        loop {
            match self.listener.try_wait_one() {
                Ok(Some(_)) => continue,
                Ok(None) => return Ok(()),
                Err(e) => {
                    return Err(TransportError::Receive {
                        // hot-path-alloc-ok: cold error arm — a listener
                        // whose try_wait_one fails is already off the
                        // healthy path.
                        topic: self.topic.clone(),
                        reason: format!("listener try_wait_one: {e:?}"),
                    });
                }
            }
        }
    }

    /// Testing-only: drain all currently
    /// queued events and return their parsed `PubSubEvent` variants.
    /// Used to verify that `SentHistory` was actually notified after
    /// `deliver_history` drives native history delivery on a
    /// `SubscriberConnected` (the wake that lets a late
    /// joiner on a quiescent publisher drain the natively-delivered frames).
    ///
    /// Returns events in the order received. Calls
    /// `drain_stale_events`-style `try_wait_one` repeatedly until
    /// the listener queue is empty.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn test_drain_events(&self) -> Vec<super::events::PubSubEvent> {
        // hot-path-alloc-ok: cfg-gated test helper (test / test-helpers
        // feature only) — never compiled into the production hot path
        let mut events = Vec::new();
        while let Ok(Some(event_id)) = self.listener.try_wait_one() {
            if let Ok(parsed) = super::events::PubSubEvent::try_from(event_id) {
                events.push(parsed);
            }
        }
        events
    }

    /// Non-consuming readiness probe: true when at least one sample is
    /// queued (backs `rmw_wait`'s readiness check).
    pub fn has_pending_sample(&self) -> bool {
        self.subscriber.has_samples().unwrap_or(false)
    }

    /// The Result-carrying twin of [`Self::has_pending_sample`] — the
    /// SAME non-consuming question, with the iceoryx2 error SURFACED instead of
    /// swallowed.
    ///
    /// The shipped probe answers `self.subscriber.has_samples().unwrap_or(false)`.
    /// That swallow is fail-SAFE for its rmw-readiness caller, where `false`
    /// means "do not claim this subscription is ready" and a lost error costs at
    /// most one delayed wake. It is INVERTED for the per-set Sync matcher, where
    /// `false` is the descent GATE's PASS witness: the gate permits descent only
    /// while at least one trigger input provably has NO second arrived frame, so
    /// a swallowed error would VOUCH FOR SCARCITY on an input nothing actually
    /// checked, pass the gate on fabricated evidence, and let the descent walk
    /// into an unprobed backlog — destroying arrived, complete, in-window sets,
    /// which is precisely the class the gate exists to prevent.
    ///
    /// So the matcher's align driver calls THIS, and maps the `Err` through its
    /// position-aware fail-closed policy (an argmin probe resolves to "nothing
    /// to descend to" ⇒ fire greedily; a GATE probe resolves to "not scarce" ⇒
    /// that input REFUSES the gate). Neither answer can enable a descent.
    ///
    /// [`Self::has_pending_sample`] and its three existing callers are
    /// deliberately UNCHANGED — this is an additional question, not a
    /// replacement for theirs.
    pub(crate) fn has_pending_sample_checked(&mut self) -> TransportResult<bool> {
        if self.fault_inject_sync_next_arrived_err {
            self.fault_inject_sync_next_arrived_err = false;
            return Err(TransportError::Receive {
                topic: self.topic.clone(),
                reason: "fault-injected probe failure (test seam)".to_string(),
            });
        }
        self.subscriber
            .has_samples()
            .map_err(|e| TransportError::Receive {
                topic: self.topic.clone(),
                reason: format!("has_samples: {e}"),
            })
    }

    /// Snapshot of this subscriber's event-listener fd, for a
    /// `poll(2)`-based blocking wait (`crate::wake::FdWakeSet`).
    ///
    /// This is what lets `rmw_wait` park on N subscriptions' EXISTING
    /// listeners — the same listeners every publish already notifies —
    /// without minting a second wake source per topic (which would cost an
    /// event-port slot and a `sendto` per publish) and without holding any
    /// entity mutex across the block: an fd is a `Copy` integer, so the
    /// caller locks the entity, snapshots, UNLOCKS, and blocks on the
    /// snapshot. This is the narrowest surface that makes that possible —
    /// narrower than exposing the `Listener` itself, whose borrow would pin
    /// the entity lock for the whole block.
    ///
    /// # Caller contract (the `native_handle` lifetime rule, restated)
    ///
    /// The value is a NON-OWNING snapshot: watch it for readability
    /// (POLLIN) only — never `read`/`write`/`close` it (the listener's own
    /// drain path is [`Self::drain_event_notifications`]) — and treat it as
    /// dead once this subscriber can have dropped. `poll(2)` degrades a
    /// violated lifetime to `POLLNVAL` (where iceoryx2's select-backed
    /// WaitSet would abort the process on EBADF), but the contract stands.
    ///
    /// Unix-only, like the `crate::wake` machinery that consumes it: the
    /// return type is `RawFd`, so an ungated accessor breaks every
    /// non-Unix build of this crate even though nothing there calls it.
    #[cfg(unix)]
    pub fn event_listener_fd(&self) -> std::os::unix::io::RawFd {
        use iceoryx2_bb_posix::file_descriptor::FileDescriptorBased;
        // SAFETY (native_handle): the value is returned as a plain integer
        // whose lifetime/no-close contract is documented above; this call
        // itself stores nothing and closes nothing.
        unsafe { self.listener.file_descriptor().native_handle() }
    }

    /// Drain this subscriber's event-NOTIFICATION queue
    /// (nonblocking) — the public face of the drain every consuming read
    /// path already performs.
    ///
    /// A blocked-wait loop calls this each iteration BEFORE probing
    /// [`Self::has_pending_sample`]: notifications the drain removes
    /// describe samples that were committed BEFORE the notify was sent, so
    /// the probe still sees them — while a notification left in place
    /// (a connection-lifecycle event, a coalesced notify for an
    /// already-taken sample) would re-fire a level-triggered fd wait
    /// forever (the spin class). Notification-only: the SHM
    /// MESSAGE queue is untouched (same firewall as the WaitSet reactor's
    /// callback drain).
    ///
    /// # Errors
    ///
    /// `Err` means the queue could NOT be drained (iceoryx2's
    /// `try_wait_one` failed) — and the fd may still be readable. A caller
    /// that blocks on [`Self::event_listener_fd`] MUST treat that as
    /// "stop blocking on this fd for now" and fall back to timeout pacing
    /// (`rmw_wait` marks its wait set degraded for the call); polling a
    /// readable fd nobody can drain is an unbounded spin. The error is
    /// surfaced rather than swallowed for exactly that reason.
    pub fn drain_event_notifications(&self) -> TransportResult<()> {
        self.try_drain_stale_events()
    }

    /// Receive AT MOST ONE sample (the rmw `take` contract
    /// is one message per call — the drain-everything `try_receive`
    /// would silently discard the rest of the queue).
    ///
    /// DELIBERATELY outside the read-outcome log. This is the rmw
    /// `take` / services read path — those reads are not recorded in graph
    /// bags today, so the read log does not invent records for them (the
    /// three graph entry points are `drain_to_latest_with_accounting`'s
    /// callers and `drain_samples`).
    ///
    /// Returns `Ok(true)` iff a valid sample was delivered to the
    /// callback. Malformed frames are skipped with a warning and
    /// reception continues to the next sample (a corrupt frame must not
    /// wedge the queue).
    pub fn try_receive_one<F>(&self, mut callback: F) -> TransportResult<bool>
    where
        F: FnMut(ReceivedMessage<'_>),
    {
        // Same misuse warn as `try_receive`:
        // shared latch, one warn per input across all four queue-draining
        // read paths.
        if self.unified_bound.is_some() {
            self.warn_unified_receive_misuse("try_receive_one");
        }
        self.drain_stale_events();
        loop {
            let Some(sample) = self
                .subscriber
                .receive()
                .map_err(|e| TransportError::Receive {
                    topic: self.topic.clone(),
                    reason: format!("{}", e),
                })?
            else {
                return Ok(false);
            };
            let raw = sample.payload();
            if raw.len() < WireHeader::SIZE {
                tracing::warn!(
                    topic = %self.topic,
                    size_bytes = raw.len(),
                    "received undersized message, skipping"
                );
                continue;
            }
            let Some(header) = WireHeader::read_from_buf(raw) else {
                // Match the sibling branches (and the docstring's "skipped
                // with a warning" promise) — never skip a malformed frame
                // silently.
                tracing::warn!(
                    topic = %self.topic,
                    size_bytes = raw.len(),
                    "wire header read failed, skipping"
                );
                continue;
            };
            let total_size = header.total_size as usize;
            if total_size < WireHeader::SIZE || total_size > raw.len() {
                tracing::warn!(
                    topic = %self.topic,
                    total_size,
                    frame_len = raw.len(),
                    "wire header total_size out of bounds, skipping"
                );
                continue;
            }
            let payload = &raw[WireHeader::SIZE..total_size];
            callback(ReceivedMessage::new(header, payload));
            // Same serve point as `drain_samples` — one frame, really
            // delivered.
            self.record_service_cursor(raw);
            return Ok(true);
        }
    }

    /// Receive AT MOST ONE sample as an OWNED zero-copy sample the
    /// caller may hold indefinitely — the rmw loaned-take read path
    /// (`rmw_take_loaned_message` holds the sample across the C ABI until
    /// `rmw_return_loaned_message_from_subscription` drops it).
    ///
    /// The SAME iceoryx2 queue receive as [`Self::try_receive_one`] — the
    /// The one-read-path decision: this is NOT a bypass read over the queue
    /// plane, it is the same `receive()`, the same stale-event drain, the same
    /// malformed-frame skip-with-warn validation and the same served-cursor
    /// accounting; the ONLY difference is ownership. A frame that parses is
    /// returned instead of lent to a callback, so its SHM slot stays borrowed
    /// until the caller drops the returned [`OwnedInboundSample`].
    ///
    /// # Holding cost (SHM back-pressure)
    ///
    /// Each live [`OwnedInboundSample`] pins one publisher-pool slot AND one
    /// unit of the service's `subscriber_max_borrowed_samples` budget on this
    /// subscriber's connection (see the type's docs). Holding the whole budget
    /// makes the NEXT receive on this subscriber fail with iceoryx2's
    /// `ExceedsMaxBorrows` — loud and bounded, never a silent loss and never a
    /// hang; dropping one held sample recovers. The sample is `Send` (pinned
    /// by the compile-time assert below [`OwnedInboundSample`]), so a caller
    /// may release it from another thread.
    ///
    /// # No reclaim while held
    ///
    /// iceoryx2's borrow accounting keeps the sample's chunk out of the
    /// publisher pool's free list until the `Sample` drops (the pool is sized
    /// with a `subscriber_max_borrowed_samples` term per subscriber slot for
    /// exactly this), so the bytes behind [`OwnedInboundSample::payload`] can
    /// never be overwritten by a later publish while the sample is held.
    ///
    /// Like [`Self::try_receive_one`], DELIBERATELY outside the
    /// read-outcome log (rmw takes are not recorded in graph bags today).
    #[must_use = "receive result must be checked"]
    pub fn try_receive_one_owned(&self) -> TransportResult<Option<OwnedInboundSample>> {
        // Same misuse warn as `try_receive_one`:
        // shared latch, one warn per input across the queue-draining read
        // paths.
        if self.unified_bound.is_some() {
            self.warn_unified_receive_misuse("try_receive_one_owned");
        }
        self.drain_stale_events();
        loop {
            let Some(sample) = self
                .subscriber
                .receive()
                .map_err(|e| TransportError::Receive {
                    topic: self.topic.clone(),
                    reason: format!("{}", e),
                })?
            else {
                return Ok(None);
            };
            let raw = sample.payload();
            if raw.len() < WireHeader::SIZE {
                tracing::warn!(
                    topic = %self.topic,
                    size_bytes = raw.len(),
                    "received undersized message, skipping"
                );
                continue;
            }
            let Some(header) = WireHeader::read_from_buf(raw) else {
                tracing::warn!(
                    topic = %self.topic,
                    size_bytes = raw.len(),
                    "wire header read failed, skipping"
                );
                continue;
            };
            let total_size = header.total_size as usize;
            if total_size < WireHeader::SIZE || total_size > raw.len() {
                tracing::warn!(
                    topic = %self.topic,
                    total_size,
                    frame_len = raw.len(),
                    "wire header total_size out of bounds, skipping"
                );
                continue;
            }
            // Same serve point as the callback twin — one frame,
            // really delivered (ownership does not change what "served" means).
            self.record_service_cursor(raw);
            return Ok(Some(OwnedInboundSample::new(sample)));
        }
    }

    /// Drain all available samples, invoking `callback` for each.
    ///
    /// Loops `receive()` straight off the iceoryx2 queue (zero-copy:
    /// each sample's payload is viewed in shared memory).
    ///
    /// `role` is the CALL-SITE role of whoever asked for the
    /// drain. This one body is genuinely shared by BOTH roles — a node body
    /// reaching it through [`Self::try_receive`] / [`Self::wait_for_message`],
    /// and the scheduler's Separate/Sync trigger drain reaching it through
    /// [`Self::try_receive_for_drain`] — which is exactly the ambiguity the
    /// role bits exist to remove, so it is a parameter rather than anything
    /// this function could work out for itself.
    fn drain_samples<F>(&self, callback: &mut F, role: ReadSiteRole) -> TransportResult<usize>
    where
        F: FnMut(ReceivedMessage<'_>),
    {
        let sub = &self.subscriber;
        let topic = &self.topic;
        let mut count = 0usize;
        // The LEGACY batch-drain read outcome — this is the
        // `TriggerSubscriber::try_receive_timestamps` path (Separate
        // data-trigger + Sync drains) and the accumulate-all
        // `with_unified_drain(false)` tick-body `try_receive`. One
        // DrainedBatch record per NON-EMPTY drain: served-seq = the NEWEST
        // delivered frame's sequence (drain order — "latest wins"), popped =
        // the delivered count (malformed frames are skipped by
        // `deliver_raw_frame` and not counted, matching the timestamps the
        // trigger path forwards). Armed-only; the seq parse is gated on it.
        // rmw `take` / services route through `try_receive_one`, which is
        // DELIBERATELY out of the read log's scope (not recorded in graph
        // bags today) — see that method.
        let capture = self.read_capture_armed();
        // On a `multi_publisher_topics` edge, ALSO track the
        // origin of the frame whose sequence this batch reports (the newest
        // DELIVERED one) — the record says which producer's frame the batch
        // served. Gated on the same bit as every other producer capture, so a
        // single-publisher edge never calls `sample.origin()` here.
        let annotate = self.capture_producer_token();
        let mut newest_seq: Option<u32> = None;
        let mut newest_origin: Option<UniquePublisherId> = None;
        // The `block` mirror MUST be decremented for every popped sample even
        // if `receive()` errors mid-drain — `drain_with_block_accounting`
        // guarantees that, then propagates the error below (AFTER the staging,
        // see the next comment).
        let drain_result = drain_with_block_accounting(
            || {
                // The SAME fire-once test fault as
                // `drain_to_latest_with_accounting`'s receive loop — a `None`
                // `Cell` read in production, so one branch on the hot path.
                // This is what makes the partial-batch staging posture below
                // (stage what was delivered BEFORE the Err propagates)
                // test-reachable.
                if let Some(remaining) = self.fault_inject_receive_after.get() {
                    if remaining == 0 {
                        self.fault_inject_receive_after.set(None);
                        return Err(TransportError::Receive {
                            topic: topic.clone(),
                            reason: "fault-injected receive error (test hook)".to_string(),
                        });
                    }
                    self.fault_inject_receive_after.set(Some(remaining - 1));
                }
                sub.receive().map_err(|e| TransportError::Receive {
                    topic: topic.clone(),
                    reason: format!("{}", e),
                })
            },
            // Every received sample is removed from the iceoryx2 queue
            // (whether or not it survives `deliver_raw_frame`'s filter) — the
            // `block` mirror tracks queue REMOVALS, not deliveries.
            |sample| {
                if deliver_raw_frame(topic, sample.payload(), callback) {
                    count += 1;
                    if capture {
                        if let Some(seq) = wire_sequence(sample.payload()) {
                            newest_seq = Some(seq);
                            if annotate {
                                newest_origin = Some(sample.origin());
                            }
                        }
                    }
                    // A hand-written / forced-Separate node body that
                    // reads through this path really did SERVE the frame, so
                    // its cursor must advance here too — otherwise its input
                    // reads "nothing served" at the anchor while frames were
                    // flowing, and a resume re-injects a band the node had
                    // already consumed.
                    self.record_service_cursor(sample.payload());
                }
            },
            |removed| self.record_block_drained(removed),
        );
        // Stage the PARTIAL batch BEFORE propagating
        // a mid-drain Err — the frames delivered so far really reached the
        // callback (the count/newest_seq above tracked them), so omitting the
        // record would make a faithful replay re-derive a batch the recording
        // never wrote (a false read-log divergence at exactly the fault
        // moment). An all-or-nothing Err (nothing delivered) still records
        // nothing, matching every other drain site's Err posture.
        if capture && count > 0 {
            match newest_origin {
                Some(origin) => self.stage_read_outcome_with_producer(
                    ReadOutcomeKind::DrainedBatch,
                    newest_seq,
                    count as u64,
                    producer_token(origin),
                    role,
                ),
                None => self.stage_read_outcome(
                    ReadOutcomeKind::DrainedBatch,
                    newest_seq,
                    count as u64,
                    role,
                ),
            }
        }
        drain_result?;
        Ok(count)
    }

    /// Receive a single SHM-backed sample as `InputView<T>` and pass it to
    /// `f`.
    ///
    /// Drains the iceoryx2 receive queue keeping only the latest sample
    /// (matches the existing "latest wins" semantic).
    /// Validates the WireHeader's `schema_hash` against `T::SCHEMA_HASH`,
    /// constructs `T::Reader<'_>` over the SHM bytes, and runs `f` with
    /// an `InputView` that holds the iceoryx2 sample alive for the
    /// closure's scope.
    ///
    /// Serve-many: while a step-boundary snapshot is frozen
    /// (`FrozenSlot::Held` / `Empty`), the slot is served IN PLACE on
    /// every call until the next capture — a repeated `try_view` returns
    /// the identical frozen answer, it does NOT drain the queue. Do not
    /// write a drain-until-`None` loop over a snapshotted subscriber: an
    /// `Ok(Some(..))` from a `Held` slot never becomes `Ok(None)` within
    /// the step. (`Sample` — the FIFO trigger head — and `Err` are still
    /// consumed on first serve; see the `FrozenSlot` docs for the rule.)
    ///
    /// Returns:
    /// - `Ok(Some(R))` — a sample was received, `f` ran, returns its value.
    /// - `Ok(None)` — no samples available; `f` was not called.
    /// - `Err(SchemaMismatch)` — a sample was received but its schema
    ///   hash didn't match `T::SCHEMA_HASH`.
    /// - `Err(Receive)` — the iceoryx2 subscriber returned an error
    ///   mid-drain. Frames popped before the error are consumed and
    ///   dropped (latest-wins semantics; the `block` mirror is still
    ///   decremented for them, and the `drop_oldest` baseline resets so
    ///   they are never miscounted as evictions — an error before the
    ///   first pop consumed nothing and keeps the baseline).
    /// - `Err(Deserialization)` — the sample was undersized or its
    ///   WireHeader could not be parsed.
    #[must_use = "try_view result must be checked"]
    pub fn try_view<T: ShmMessage, R>(
        &mut self,
        f: impl FnOnce(InputView<'_, T>) -> R,
    ) -> TransportResult<Option<R>> {
        let slot = self.select_slot();
        match slot {
            FrozenSlot::Sample(sample) => {
                // The ONE serve point both drain disciplines reach —
                // Unified serves the boundary's frozen slot, Separate pops live
                // into the same arm — so the service cursor has one write site
                // and one meaning, and `CERULION_DRAIN_DISCIPLINE=separate`
                // yields the identical cursor.
                //
                // Advanced only on `Ok`: `build_inbound_view` rejects a
                // schema-mismatched / undersized / out-of-bounds frame, which
                // the tick never sees. Recording it as served would tell a
                // resume to skip a frame nothing read.
                let out = build_inbound_view::<T, R>(&self.topic, &sample, f)?;
                self.record_service_cursor(sample.payload());
                Ok(Some(out))
            }
            // NB: this arm records NO cursor, and it is unreachable TWICE over,
            // which is why that is not a gap:
            //
            // 1. The serve-many arm above RETURNS on `Held`, so no `Held` slot
            //    reaches this match at all.
            // 2. Even if one did, `held_sample` is written only by
            //    `snapshot_latest` (the non-trigger, latest-value path), and
            //    this type's own invariant states that a `Sample` frozen slot
            //    happens ONLY for trigger / direct-path inputs, which never
            //    hold — so a `Held` input never carries a cursor to write.
            //
            // Storing here would cost an atomic per non-trigger input per fire
            // for a value no reader can use, and — under serve-many — would do
            // it once per fire of a burst for one frame.
            FrozenSlot::Held => {
                let sample = self
                    .held_sample
                    .as_ref()
                    .ok_or_else(|| TransportError::Internal {
                        reason: format!(
                            "subscriber {} selected a held slot without a held sample",
                            self.topic
                        ),
                    })?;
                build_inbound_view::<T, R>(&self.topic, sample, f).map(Some)
            }
            FrozenSlot::Empty => Ok(None),
            FrozenSlot::Err(e) => Err(e),
        }
    }

    /// Return the next complete wire frame without schema validation.
    ///
    /// Slot selection, read-outcome staging, FIFO promotion, and block-slot
    /// reconciliation are identical to [`Self::try_view`]. Live samples are
    /// retained in an owned view until dropped; held samples borrow the held
    /// slot and do not advance the service cursor.
    #[must_use = "raw input view must be checked"]
    pub fn view_raw(&mut self) -> TransportResult<Option<RawInputView<'_>>> {
        let slot = self.select_slot();
        match slot {
            FrozenSlot::Held => {
                let sample = self
                    .held_sample
                    .as_ref()
                    .ok_or_else(|| TransportError::Internal {
                        reason: format!(
                            "subscriber {} selected a held slot without a held sample",
                            self.topic
                        ),
                    })?;
                let frame_len = validate_raw_frame(&self.topic, sample.payload())?;
                Ok(Some(RawInputView {
                    inner: RawInputViewInner::Held(&sample.payload()[..frame_len]),
                }))
            }
            FrozenSlot::Empty => Ok(None),
            FrozenSlot::Sample(sample) => {
                let frame_len = validate_raw_frame(&self.topic, sample.payload())?;
                self.record_service_cursor(sample.payload());
                Ok(Some(RawInputView {
                    inner: RawInputViewInner::Owned(sample, frame_len),
                }))
            }
            FrozenSlot::Err(error) => Err(error),
        }
    }

    /// Callback convenience wrapper around [`Self::view_raw`].
    #[must_use = "raw input result must be checked"]
    pub fn try_view_raw<R>(&mut self, f: impl FnOnce(&[u8]) -> R) -> TransportResult<Option<R>> {
        self.view_raw().map(|view| view.map(|view| f(&view)))
    }

    /// Select the next slot using the same frozen/held/live discipline as `try_view`.
    ///
    /// Raw and typed consumers must share this path so read-outcome staging,
    /// FIFO promotion, and block-slot reconciliation remain identical.
    fn select_slot(&mut self) -> FrozenSlot {
        // Accounting-once: if a step-boundary snapshot was
        // taken for this input, serve it WITHOUT re-draining (the drain already
        // ran + accounted at snapshot time). Otherwise drain live now (trigger
        // inputs + the probe-less direct path).
        //
        // SERVE-MANY: the two slot states that own nothing are served
        // IN PLACE, so every fire of a `Data` burst reads the identical frozen
        // context (see the `FrozenSlot` type docs for the whole rule, and for
        // why `Sample`/`Err` are still taken). Serving in place runs NO
        // accounting — a `Held` serve is a borrow of `held_sample` and an
        // `Empty` serve reads nothing — so the accounting-once contract is
        // unchanged however many fires read it.
        match self.frozen.as_ref() {
            Some(FrozenSlot::Held) => return FrozenSlot::Held,
            Some(FrozenSlot::Empty) => return FrozenSlot::Empty,
            // `Sample` / `Err` (consumed below) and `None` (the live arm).
            _ => {}
        }
        let slot = match self.frozen.take() {
            // Serving a FROZEN slot records NOTHING — the read
            // outcome was already staged when the snapshot/trigger drain ran
            // (the accounting-once contract extends to the read log: one
            // record per read, whichever half serves it). With serve-many that is
            // now true of every re-serve too, which is what keeps the log at
            // one record per CONSUMED frame rather than one per fire.
            //
            // Reachable for `Sample` / `Err` only — the serve-many arm above
            // returned for `Held` / `Empty`. A SECOND `try_view` of the same
            // tick therefore still falls to the live arm below on those two
            // and records that second read as it happened.
            Some(slot) => slot,
            // The live drain returns a richer `DrainOutcome`;
            // `try_view` only needs the surviving slot (popped/latest_ts are
            // consumed by the trigger path, not the body read).
            //
            // Per-message FIFO (52125241e): an `EachFifo` (data-trigger)
            // input with no frozen slot pops exactly ONE frame in arrival
            // order — the Separate-discipline tick-body read. Latest-value
            // context inputs keep the drain-to-latest semantic.
            None => {
                // R-pop′: the next FIFO frame IS `next_head`'s.
                // Promote-serve it rather than receiving from the queue, which
                // would invert this input's delivery order — the staged frame
                // is OLDER than whatever the queue would hand back.
                //
                // Records nothing and re-accounts nothing: both were staged
                // when the descent's `NeedStamp` popped it, mirroring the
                // frozen-slot rule directly above (one record per CONSUMED
                // frame, whichever half finally serves it).
                //
                // WHAT THE SILENCE COSTS AN OFFLINE READER, and
                // why it is still the right call:
                //
                // The silence is correct about POPS (the frame's one pop was
                // accounted at the `Peek`, and re-recording it here creates
                // a false finding from the other end — MEASURED). What it
                // cannot express is SLOT STATE: this arm and the frozen-slot
                // serve above BOTH record nothing, so a bag reader cannot tell
                // "the peeked frame is still parked in `next_head`" from "the
                // body consumed it". Its consumers would be
                // `cerulion_cli_engine`'s `verify_fifo`, `verify_conservation`
                // and `replay_inject`'s cursor join, and nothing is currently
                // WRONG because of it — every pop is accounted exactly once, so
                // both sums are right and the gap blocks only a claim nobody
                // makes yet. `verify_sync` is out of its blast radius entirely:
                // the peek stamp is a candidate in its supply
                // model, which over-approximates by construction.
                //
                // The ONE record that would close it is a body-site `Served` at
                // `popped: 0` for this promote-serve, mirroring
                // `sync_discard_head`'s `Drain`-at-`popped: 0` promotion.
                // It is a RECORDING-side change,
                // so it needs the same PAIRED-rollout treatment the format-5
                // peek/head mark got, and it is not minted here.
                //
                // `(None, Occupied)` is reachable only POST-FIRE — a tick took
                // the head while a staged next was still pending — so on every
                // other read this costs one `Option` test on the live arm.
                let promoted = match self.consume_mode {
                    ConsumeMode::EachFifo => self.next_head.take(),
                    ConsumeMode::Latest => None,
                };
                match promoted {
                    // Yields the slot so it FALLS THROUGH to the shared serve
                    // below rather than returning here — the service
                    // cursor must advance for a promote-serve exactly as for
                    // any other served frame, and routing every served frame
                    // through the ONE serve point is what makes that structural
                    // rather than a rule two sites have to remember.
                    Some(sample) => FrozenSlot::Sample(sample),
                    None => {
                        let outcome = match self.consume_mode {
                            ConsumeMode::Latest => self.drain_to_latest_with_accounting(),
                            ConsumeMode::EachFifo => self.drain_one_with_accounting(),
                        };
                        // The LIVE body read (Separate-discipline
                        // trigger bodies, no-snapshot closure inputs, direct
                        // staged reads). Armed-only, checked BEFORE the
                        // served-seq parse; an Err drain is out of the
                        // read-log contract (the TraceEntry posture) and
                        // records nothing.
                        //
                        // The classification is CONSUME-MODE-BLIND on purpose:
                        // a FIFO pop-one that yields a frame is still "a fresh
                        // frame was served to this body read" — `Served`, with
                        // `popped` exactly 1 (junk-skipped frames excluded by
                        // the drain) — and a `sample(N)`-decimated pop is still
                        // `Decimated`. The kind vocabulary distinguishes SITE
                        // ROLE (body read vs trigger/batch drain), never queue
                        // discipline; replay runs this same match, so
                        // RECORD==REPLAY holds by construction.
                        //
                        // CLASSIFICATION ASYMMETRY vs `snapshot_latest` —
                        // INTENTIONAL: an Empty drain here
                        // records `NoFrame` even when a held sample
                        // exists, because `try_view`'s serve below really
                        // returns `Ok(None)` on Empty — only the SNAPSHOT path
                        // upgrades Empty→Held and serves the held value. The
                        // record mirrors what the read SERVED, never what it
                        // could have served.
                        if self.read_capture_armed() {
                            match &outcome.slot {
                                // On a `multi_publisher_topics`
                                // edge the SERVED frame's producer is annotated
                                // ahead of the read (the served seq alone names
                                // no publisher there — `sequence` is
                                // per-publisher). Every other edge takes the
                                // plain arm and is byte-identical to 2c0.
                                // `try_view` IS the node body's
                                // own read — every arm here is a BODY-site
                                // record.
                                FrozenSlot::Sample(s) if self.capture_producer_token() => self
                                    .stage_read_outcome_with_producer(
                                        ReadOutcomeKind::Served,
                                        wire_sequence(s.payload()),
                                        outcome.popped,
                                        producer_token(s.origin()),
                                        ReadSiteRole::Body,
                                    ),
                                FrozenSlot::Sample(s) => self.stage_read_outcome(
                                    ReadOutcomeKind::Served,
                                    wire_sequence(s.payload()),
                                    outcome.popped,
                                    ReadSiteRole::Body,
                                ),
                                FrozenSlot::Empty if outcome.decimated => self.stage_read_outcome(
                                    ReadOutcomeKind::Decimated,
                                    None,
                                    outcome.popped,
                                    ReadSiteRole::Body,
                                ),
                                FrozenSlot::Empty => self.stage_read_outcome(
                                    ReadOutcomeKind::NoFrame,
                                    None,
                                    outcome.popped,
                                    ReadSiteRole::Body,
                                ),
                                FrozenSlot::Err(_) | FrozenSlot::Held => {}
                            }
                        }
                        outcome.slot
                    }
                }
            }
        };
        // SLOT EXIT: the slot (or the staged next) has just been TAKEN,
        // so whatever this read is about to serve is no longer occupancy the
        // matcher holds. This is the one exit both the frozen serve and the
        // R-pop' promote-serve funnel through — the same property the serve-many
        // service cursor leans on — and it is deliberately BEFORE the serve, so
        // a frame the view rejects (`build_inbound_view`'s `?`) still releases
        // its slot: it left the queue and it is not coming back, whatever the
        // node made of it. The LIVE arm above pops and serves inside this one
        // call and never touched a slot, so re-deriving finds nothing to
        // release and the pop-time decrement stands unchanged.
        self.reconcile_block_slot_debt();
        slot
    }

    /// The single drain-and-account implementation. Drains
    /// the iceoryx2 queue to the latest sample, running ALL backpressure
    /// accounting EXACTLY ONCE (block mirror, drop_oldest baselines, sample(N)
    /// gate, expect_within anchor). Fully non-generic — the message type `T` is
    /// needed only at `build_inbound_view`, which the caller does AFTER this.
    /// Returns the surviving sample / Empty / Err as a `FrozenSlot`, plus the
    /// unified trigger accounting (`popped` = frames consumed this drain,
    /// `latest_ts` = the surviving sample's wire timestamp, None on Empty/Err).
    fn drain_to_latest_with_accounting(&mut self) -> DrainOutcome {
        self.drain_with_accounting_impl(false)
    }

    /// Pop exactly ONE sample (FIFO head) with the same accounting-once pass
    /// — the per-message read for `ConsumeMode::EachFifo` trigger inputs.
    /// Identical accounting to the latest-wins drain (block mirror, eviction
    /// baselines, `sample(N)` gate, `expect_within` anchor), but the receive
    /// loop stops after the first popped sample, so queued older frames are
    /// SERVED on later reads instead of discarded. A `sample(N)`-decimated
    /// pop returns `Empty` (popped = 1, decimation counted); the next read
    /// pops the next frame.
    fn drain_one_with_accounting(&mut self) -> DrainOutcome {
        self.drain_with_accounting_impl(true)
    }

    /// The single drain-and-account implementation behind BOTH consume modes
    /// (`limit_one = false` ⇒ drain-to-latest; `true` ⇒ pop-one FIFO).
    fn drain_with_accounting_impl(&mut self, limit_one: bool) -> DrainOutcome {
        self.drain_stale_events();

        // Drain to the latest message ("latest wins": drop older
        // messages in favour of the most recent — predictable for control
        // loops). Reads straight off the iceoryx2 queue and views the
        // `Sample` zero-copy.
        // `block`: sample the outstanding mirror BEFORE the drain
        // decrements it — that is this consumer's queue depth on arrival, i.e.
        // whether the producer was being deferred on its behalf.
        let pre_drain_outstanding = self.block_probe().map(|p| p.outstanding.outstanding());
        let sub = &self.subscriber;
        let topic = &self.topic;
        let mut latest: Option<InboundSample> = None;
        // Per-publisher-stream capture. One
        // `StreamObservation` per distinct origin id seen this drain
        // (oldest surviving seq + high-water seq + its timestamp), filled
        // only when a probe needs it. `mem::take` lets the drain closures
        // own it while `record_block_drained` borrows `&self`; restored on
        // every exit so capacity is retained (zero-alloc steady state).
        let probing = matches!(
            &self.probe,
            Some(BackpressureProbe::DropOldest(_) | BackpressureProbe::Block(_))
        );
        let mut scratch = std::mem::take(&mut self.drain_scratch);
        scratch.clear();
        // Frames actually popped by this drain — read after an ERRORED
        // drain to decide whether the baselines must reset (an error on the
        // very first `receive()` call consumed nothing, so the baselines
        // are still valid). `Cell` because the recording closure and this
        // function both need it, immutably.
        let removed_in_drain = std::cell::Cell::new(0u64);
        // If ANY drained frame is undersized (no
        // readable wire sequence), the per-stream firsts may not be the
        // true oldest → gaps would inflate. Track it and skip eviction
        // detection this drain (matches the sample(N) corrupt-frame
        // precedent).
        let mut any_undersized = false;
        // "Latest wins": keep only the newest sample, dropping older ones.
        // The `block` mirror decrements by ALL popped samples, and
        // `drain_with_block_accounting` guarantees that even if `receive()`
        // errors mid-drain (bypassing the decrement would permanently
        // inflate the mirror — the same deadlock guarded in `drain_samples`).
        let mut fault_inject = self.fault_inject_receive_after.get();
        // FIFO pop-one: the per-sample closure flips this after the first
        // pop; the receive closure then reports the queue as drained, so
        // exactly one sample leaves the queue per call and older frames
        // survive for later reads (per-message delivery). `Cell` because the
        // receive and per-sample closures both capture it immutably.
        let stop_after_first = std::cell::Cell::new(false);
        // Corrupt (undersized) frames skipped by the FIFO pop — they are
        // real queue removals (block-mirror-accounted) but must not mint a
        // fire, so the returned `popped` (the trigger's signal count)
        // excludes them. Always 0 on the latest-wins drain.
        let junk_skipped = std::cell::Cell::new(0u64);
        let drain_result = drain_with_block_accounting(
            || {
                if limit_one && stop_after_first.get() {
                    return Ok(None);
                }
                // Test-only fire-once receive fault (see the
                // `fault_inject_receive_after` field doc); `None` in
                // production, so this is a single branch on the hot path.
                if let Some(remaining) = fault_inject {
                    if remaining == 0 {
                        fault_inject = None;
                        return Err(TransportError::Receive {
                            topic: topic.clone(),
                            reason: "fault-injected receive error (test hook)".to_string(),
                        });
                    }
                    fault_inject = Some(remaining - 1);
                }
                sub.receive().map_err(|e| TransportError::Receive {
                    topic: topic.clone(),
                    reason: format!("{}", e),
                })
            },
            |sample| {
                if probing {
                    match wire_sequence(sample.payload()) {
                        Some(seq) => {
                            let id = sample.origin();
                            match scratch.iter_mut().find(|o| o.id == id) {
                                None => scratch.push(StreamObservation {
                                    id,
                                    first_seq: seq,
                                    newest_seq: seq,
                                    newest_ts: wire_timestamp_ns(sample.payload()).unwrap_or(0),
                                }),
                                Some(obs) => {
                                    // High-water per stream, NOT drain-order
                                    // last: a re-delivered stale frame
                                    // (history replay) AFTER live ones must
                                    // not regress the stream's baseline —
                                    // the next drain would count this
                                    // drain's consumed live frames as
                                    // evicted (fabrication). Frames BEHIND
                                    // the stream's first (wrapping gap >
                                    // threshold) never become the
                                    // high-water; `newest_ts` tracks the
                                    // same frame so regime timestamps are
                                    // never a replayed frame's stale clock.
                                    let fwd = seq.wrapping_sub(obs.first_seq);
                                    let newest_fwd = obs.newest_seq.wrapping_sub(obs.first_seq);
                                    if fwd <= BACKWARD_GAP_THRESHOLD && fwd >= newest_fwd {
                                        obs.newest_seq = seq;
                                        obs.newest_ts =
                                            wire_timestamp_ns(sample.payload()).unwrap_or(0);
                                    }
                                }
                            }
                        }
                        None => any_undersized = true,
                    }
                }
                // FIFO pop-one: a corrupt (undersized) frame must not WEDGE
                // the queue or occupy a fire — skip it and keep popping to
                // the next parseable frame (the same contract
                // `try_receive_one` documents for the per-message read
                // path). The removal is still block-mirror-accounted, and
                // when an eviction probe is installed the `probing` branch
                // above has already flagged `any_undersized` so the
                // baselines reset; the skipped frame simply never becomes
                // the served slot. The
                // latest-wins drain keeps its pre-existing behavior (an
                // undersized latest surfaces as a loud Deserialization error
                // from the read).
                if limit_one && sample.payload().len() < WireHeader::SIZE {
                    junk_skipped.set(junk_skipped.get() + 1);
                    return;
                }
                latest = Some(sample);
                stop_after_first.set(true);
            },
            |removed| {
                removed_in_drain.set(removed);
                self.record_block_drained(removed);
            },
        );
        self.fault_inject_receive_after.set(fault_inject);
        if let Err(e) = drain_result {
            if removed_in_drain.get() > 0 {
                // An errored drain already consumed an
                // unknowable set of frames (prior `receive()` calls popped
                // before the error surfaced) that the probe never observed —
                // keeping the old baselines would count those CONSUMED
                // frames as evictions on the next successful drain
                // (fabrication). An errored drain that consumed frames is a
                // no-information drain: reset all baselines, mirroring the
                // corrupt-drain contract (one interval may under-report).
                // An error on the very FIRST `receive()` call consumed
                // nothing — the baselines are still valid and are kept.
                if let Some(probe) = self.drop_oldest_probe_mut() {
                    probe.baselines.clear();
                }
            }
            scratch.clear();
            self.drain_scratch = scratch;
            // An errored drain has no surviving sample (latest_ts None);
            // `popped` is still the frames consumed before the error surfaced.
            return DrainOutcome {
                slot: FrozenSlot::Err(e),
                popped: removed_in_drain.get().saturating_sub(junk_skipped.get()),
                latest_ts: None,
                decimated: false,
            };
        }
        // `drop_oldest` (EXACT PER PUBLISHER STREAM):
        // count iceoryx2 evictions (silent oldest reclaim on overflow) from
        // each stream's wire-sequence gap below its oldest drained frame,
        // summed across the streams seen this drain. Only installed on
        // `drop_oldest` inputs (mutually exclusive with the sample gate).
        // Per stream (see `classify_gap`): `Exact(n)` adds to the drain's
        // evicted total; `Backward` (history replay) is never counted; a
        // first-sighting (or restarted — new port id) stream
        // baseline-establishes, uncounted. A corrupt drain resets ALL
        // baselines. The accessor's `&mut self` borrow ends with the owned
        // `queued` value, before the `pending_backpressure_event` write.
        //
        // NOTE: `try_view` is the SOLE probe-aware read path. Draining a
        // `drop_oldest`-probed input through `wait_for_message` /
        // `try_receive` consumes frames the probe never observes,
        // desynchronizing the baselines — eviction counts after a bypassed
        // drain are unreliable. (The `block` mirror IS maintained on those
        // paths via `record_block_drained`; the sample gate is unaffected.)
        //
        // Regime stamps (drop_oldest Exact + block) use the drain's
        // high-water timestamp across streams — never a trailing replayed
        // frame's stale clock. 0 when the drain had no
        // readable wire timestamp (documented on
        // `BackpressureEvent::regime_started_at_ns`).
        let drain_high_water_ts = scratch.iter().map(|o| o.newest_ts).max().unwrap_or(0);
        if any_undersized {
            // A corrupt (undersized) frame makes
            // every positional inference untrustworthy — a stream's `first`
            // may not be its true oldest readable frame, and its high-water
            // may sit BEHIND frames this drain actually consumed (an
            // undersized frame occupies an unknowable sequence slot).
            // Advancing baselines would convert those consumed frames into
            // next-drain "evictions" (fabrication), so ALL baselines reset:
            // the next drain re-establishes them and one interval goes
            // uncounted (a safe under-report, the documented direction).
            // Runs regardless of whether any frame had a readable sequence,
            // so an ALL-undersized drain leaves the same trail and the same
            // baseline state. Event/regime state untouched: a corrupt drain
            // carries no information about eviction regimes.
            tracing::debug!(
                topic = %self.topic,
                "undersized frame in drain (corrupt/truncated) — surfaced as a loud \
                 Deserialization error by the latest-wins read"
            );
            if let Some(probe) = self.drop_oldest_probe_mut() {
                probe.baselines.clear();
                tracing::debug!(
                    node_id = %probe.node_id,
                    input = %probe.input,
                    "drop_oldest eviction detection skipped this drain (undersized \
                     frame); baselines reset — the next drain re-establishes them, so \
                     `backpressure_drop_oldest_count` may under-report one interval"
                );
            }
        } else if !scratch.is_empty() {
            let queued = self.drop_oldest_probe_mut().and_then(|probe| {
                let mut evicted_total: u64 = 0;
                let mut any_backward = false;
                probe.drain_counter += 1;
                let now_drain = probe.drain_counter;
                for obs in &scratch {
                    match probe
                        .baselines
                        .iter()
                        .position(|(bid, _, _)| *bid == obs.id)
                    {
                        None => {
                            // First sighting of this publisher stream (a new
                            // publisher, or a restarted one — a restarted
                            // port gets a NEW UniquePublisherId): its prior
                            // history is unknowable, so it
                            // baseline-establishes uncounted. At capacity,
                            // evict the not-seen-this-drain entry with the
                            // OLDEST last-seen stamp (dead streams never
                            // refresh; a live-but-quiet stream refreshes on
                            // every appearance — see the `baselines` field
                            // doc for the residual displacement window).
                            // Deterministic: stamps are per-probe counters,
                            // ties broken by lowest index.
                            tracing::debug!(
                                node_id = %probe.node_id,
                                input = %probe.input,
                                first_seq = obs.first_seq,
                                "new publisher stream observed — baseline established \
                                 (its pre-observation history is not countable)"
                            );
                            if probe.baselines.len() >= probe.max_baselines {
                                let victim = probe
                                    .baselines
                                    .iter()
                                    .enumerate()
                                    .filter(|(_, (bid, _, _))| {
                                        !scratch.iter().any(|o| o.id == *bid)
                                    })
                                    .min_by_key(|(i, (_, _, seen))| (*seen, *i))
                                    .map(|(i, _)| i);
                                if let Some(evict) = victim {
                                    let (evicted_id, _, last_seen) = probe.baselines[evict];
                                    // The displaced stream's window goes
                                    // uncounted if it was live — leave the
                                    // trail a "my count is low" report
                                    // needs.
                                    tracing::debug!(
                                        node_id = %probe.node_id,
                                        input = %probe.input,
                                        evicted_stream = %evicted_id,
                                        last_seen_drain = last_seen,
                                        current_drain = now_drain,
                                        "baseline capacity reached — evicting the \
                                         longest-unseen stream's baseline; if that \
                                         stream is alive-but-quiet, its next window \
                                         re-establishes uncounted"
                                    );
                                    probe.baselines.remove(evict);
                                } else {
                                    // Every tracked stream appeared in this
                                    // drain (dead publishers' still-queued
                                    // frames can transiently exceed
                                    // max_publishers distinct ids); grow
                                    // rather than miscount.
                                    tracing::debug!(
                                        node_id = %probe.node_id,
                                        input = %probe.input,
                                        "more publisher streams in one drain than \
                                         max_publishers — growing the baseline set"
                                    );
                                }
                            }
                            probe.baselines.push((obs.id, obs.newest_seq, now_drain));
                        }
                        Some(bi) => {
                            // Stamp the stream as seen this drain whatever
                            // its gap classifies as — the eviction victim
                            // choice keys off recency of APPEARANCE, not of
                            // baseline advancement.
                            probe.baselines[bi].2 = now_drain;
                            let base_seq = probe.baselines[bi].1;
                            match classify_gap(base_seq, obs.first_seq) {
                                GapClass::Clean => {
                                    probe.baselines[bi].1 = obs.newest_seq;
                                }
                                GapClass::Exact(n) => {
                                    // Sequence evidence for the count:
                                    // the canonical warn in
                                    // `record_backpressure_event_n` carries
                                    // only the total — this breadcrumb lets
                                    // a fabrication report be falsified
                                    // from logs alone.
                                    tracing::debug!(
                                        node_id = %probe.node_id,
                                        input = %probe.input,
                                        evicted = n,
                                        base_seq,
                                        first_seq = obs.first_seq,
                                        newest_seq = obs.newest_seq,
                                        "drop_oldest eviction evidence (per stream)"
                                    );
                                    evicted_total += u64::from(n);
                                    probe.baselines[bi].1 = obs.newest_seq;
                                }
                                GapClass::Backward => {
                                    // Duplicate re-delivery (history replay
                                    // to a late joiner) — not eviction. Keep
                                    // the stream's live high-water baseline;
                                    // advance it ONLY if this drain also
                                    // contained frames FORWARD of it
                                    // (replay-then-live in one window) —
                                    // otherwise the next drain would count
                                    // this drain's consumed live frames as
                                    // evicted. CAVEAT: when the
                                    // forward advance happens, evictions
                                    // between the old baseline and the
                                    // advanced high-water are NOT counted —
                                    // an overlap drain under-reports that
                                    // window (the safe direction).
                                    any_backward = true;
                                    let fwd = obs.newest_seq.wrapping_sub(base_seq);
                                    if fwd != 0 && fwd <= BACKWARD_GAP_THRESHOLD {
                                        probe.baselines[bi].1 = obs.newest_seq;
                                    }
                                    tracing::debug!(
                                        node_id = %probe.node_id,
                                        input = %probe.input,
                                        "drop_oldest drain contained re-delivered frames \
                                         (history replay to a late joiner) — duplicates \
                                         are never counted as evictions; evictions hidden \
                                         under an overlapping replay window are \
                                         under-reported"
                                    );
                                }
                            }
                        }
                    }
                }
                // Sustained re-delivery (e.g. late-joiner churn on a
                // history-enabled topic — every attach replays to every
                // subscriber) keeps counting intermittent with only
                // debug-level breadcrumbs; surface it at warn level on a
                // cadence. No onset warn: a single replay
                // drain is routine.
                if any_backward {
                    probe.backward_run += 1;
                    if probe.backward_run % ANOMALY_REWARN_EVERY == 0 {
                        tracing::warn!(
                            node_id = %probe.node_id,
                            input = %probe.input,
                            backward_drains_in_regime = probe.backward_run,
                            "drop_oldest eviction counting is intermittent — this \
                             input keeps receiving re-delivered (backward-sequence) \
                             frames: sustained late-joiner churn on a history-enabled \
                             topic, or a stalled consumer; \
                             `backpressure_drop_oldest_count` may under-report the \
                             replaying stream while re-delivery persists (other \
                             streams keep counting exactly)"
                        );
                    }
                } else {
                    probe.backward_run = 0;
                }
                if evicted_total > 0 {
                    let fire = probe.armed;
                    if fire {
                        // Regime state is written ONLY at regime open —
                        // events fire only here, so disarmed evictions bump
                        // the counter (below) but no per-regime state.
                        probe.regime_started_ns = drain_high_water_ts;
                        probe.regime_count = evicted_total;
                        probe.armed = false;
                    }
                    // `emit_warn = fire` — the counter bumps by
                    // `evicted_total` on EVERY evicting drain (unconditional,
                    // Principle #3), but the warn fires once per regime, aligned
                    // with the `BackpressureEvent` edge-trigger below (`fire` is
                    // `probe.armed`, set true only at regime open). Before the latch
                    // this warned per drain — 91% of a healthy nav2 record.log.
                    record_backpressure_event_n(
                        &probe.node_id,
                        &probe.input,
                        BackpressurePolicy::DropOldest,
                        &probe.counters,
                        evicted_total,
                        fire,
                    );
                    fire.then(|| BackpressureEvent {
                        policy: BackpressurePolicy::DropOldest,
                        input_name: Arc::clone(&probe.input),
                        count_total: probe.counters.drop_oldest_count.load(Ordering::Acquire),
                        count_in_regime: probe.regime_count,
                        dropped: probe.regime_count,
                        regime_started_at_ns: probe.regime_started_ns,
                        buffer_capacity: probe.buffer_capacity,
                    })
                } else {
                    // No eviction this drain. A drain with NO backward
                    // streams is clean and rearms the edge-trigger; a
                    // backward (replay) drain carries no regime
                    // information and leaves `armed` untouched.
                    if !any_backward {
                        probe.armed = true;
                    }
                    None
                }
            });
            if let Some(ev) = queued {
                self.pending_backpressure_event = Some(ev);
            }
        }
        // `block`: fire the consumer's event if its queue was at the
        // defer threshold on arrival (it caused/sustained the producer defer).
        // Lossless — `dropped = 0`; the count_total READS the producer-
        // maintained counter (no double-count). Mutually exclusive with the
        // sample gate / drop_oldest probe (one policy per input — now
        // type-enforced by [`BackpressureProbe`]). The accessor borrow ends
        // with the owned `queued` value, before the event write.
        if let Some(pre) = pre_drain_outstanding {
            let queued = self.block_probe_mut().and_then(|probe| {
                if pre >= probe.threshold {
                    let fire = probe.armed;
                    if fire {
                        // High-water frame's timestamp (same rationale
                        // as the drop_oldest arm); regime state written
                        // only at regime open.
                        probe.regime_started_ns = drain_high_water_ts;
                        probe.regime_count = 1;
                        probe.armed = false;
                    }
                    fire.then(|| BackpressureEvent {
                        policy: BackpressurePolicy::Block,
                        input_name: Arc::clone(&probe.input),
                        count_total: probe
                            .counters
                            .block_fires_deferred_count
                            .load(Ordering::Acquire),
                        count_in_regime: probe.regime_count,
                        dropped: 0,
                        regime_started_at_ns: probe.regime_started_ns,
                        buffer_capacity: probe.buffer_capacity,
                    })
                } else {
                    probe.armed = true; // dropped below threshold → rearm
                    None
                }
            });
            if let Some(ev) = queued {
                self.pending_backpressure_event = Some(ev);
            }
        }
        // Restore the per-drain scratch (capacity retained) before the
        // sample gate — the gate's decimate path returns early.
        scratch.clear();
        self.drain_scratch = scratch;
        // `sample(N)`: read-gate the latest sample by its WIRE
        // timestamp. Accept iff it's >= N ms after the last accepted read;
        // otherwise decimate (drop it, bump `sampled_count`, return None).
        // Keyed off the publish-time wire clock so it is replay-deterministic.
        if let Some(gate) = self.sample_gate_mut() {
            if let Some(sample) = &latest {
                // An undersized frame has no readable wire
                // timestamp. Do NOT decimate it (that would mask a corrupt
                // frame as a routine sample-drop). Fall through to
                // `build_inbound_view` below, which surfaces a loud
                // `Deserialization` error — the same path the non-sample
                // input takes. `None` ⇒ skip the gate entirely for this
                // frame; the gate state (last_accepted_ns / armed) is left
                // untouched.
                if let Some(ts) = wire_timestamp_ns(sample.payload()) {
                    let accept = match gate.last_accepted_ns {
                        None => true,
                        Some(last) => ts.saturating_sub(last) >= gate.interval_ns,
                    };
                    if accept {
                        gate.last_accepted_ns = Some(ts);
                        // A successful accept rearms the edge-trigger; the
                        // next decimation opens a fresh regime (and is the
                        // only writer of the regime fields).
                        gate.armed = true;
                    } else {
                        // Decimate. Edge-trigger: the FIRST decimate of a regime
                        // queues one BackpressureEvent; subsequent ones are silent
                        // (counter still bumps) until the next accept rearms.
                        let fire_event = gate.armed;
                        if fire_event {
                            // Regime state written only at regime open
                            // (events fire only here).
                            gate.regime_started_ns = ts;
                            gate.regime_count = 1;
                            gate.armed = false;
                        }
                        // Pass the same edge-trigger (`fire_event`,
                        // = `gate.armed` at regime open) that gates the
                        // `BackpressureEvent` above. A fast producer into a
                        // `sample(N)` input decimates on nearly every frame; an
                        // unconditional-`true` warn would flood ~990/s at 1kHz
                        // into a 100ms window — the exact healthy-steady-state
                        // flood class also suppressed for `drop_oldest`. The warn
                        // fires once per decimation regime; sustained decimations
                        // log at `debug!`; the `sampled_count` bump stays
                        // unconditional (Principle #3).
                        record_backpressure_event_n(
                            &gate.node_id,
                            &gate.input,
                            BackpressurePolicy::Sample(gate.interval_ms),
                            &gate.counters,
                            1,
                            fire_event,
                        );
                        let queued = fire_event.then(|| BackpressureEvent {
                            policy: BackpressurePolicy::Sample(gate.interval_ms),
                            input_name: Arc::clone(&gate.input),
                            // sampled_count was just bumped by record_* above, so
                            // this matches NodeHandle::backpressure_sampled_count.
                            count_total: gate.counters.sampled_count.load(Ordering::Acquire),
                            count_in_regime: gate.regime_count,
                            // sample(N) decimation IS data loss: every regime drop
                            // is a dropped message.
                            dropped: gate.regime_count,
                            regime_started_at_ns: gate.regime_started_ns,
                            buffer_capacity: gate.buffer_capacity,
                        });
                        // The gate borrow ends here; set the (disjoint) event slot.
                        if let Some(ev) = queued {
                            self.pending_backpressure_event = Some(ev);
                        }
                        // On a PER-SET
                        // Sync TRIGGER a raw ARRIVAL resets the
                        // `expect_within_ms` watchdog — a decimated arrival
                        // INCLUDED. The watchdog is a PRODUCER-liveness
                        // surface; `sample(N)` is a CONSUMER read policy, so a
                        // healthy producer in a decimation regime must not
                        // page the operator. It also removes a silent
                        // by-policy fork: the SAME
                        // `#[input(trigger, backpressure = sample(N),
                        // expect_within_ms = M)]` line on a DATA node keeps
                        // its watchdog reset per raw arrival, because that
                        // input's Separate drain calls
                        // `signal_input_received` per drained timestamp.
                        //
                        // SCOPED to the marker, deliberately: the
                        // delivered-sample-only rule above is a PINNED
                        // contract for every other input
                        // (`expect_within_iox2_test`'s sample-gate case), and
                        // widening it here would flip that pin for Data
                        // triggers and non-trigger body reads too. `ts` is the
                        // frame's own wire stamp — the same quantity the
                        // delivered path stores, so the anchor stays
                        // arrival-authored and the scheduler's
                        // single-writer-class invariant is untouched.
                        if self.per_set_sync_trigger {
                            if let Some(anchor) = &self.expect_within_last_data_ns {
                                anchor.store(ts, Ordering::Release);
                            }
                        }
                        // The `latest` sample drops here → its SHM slot is released.
                        // A decimated frame delivers no sample, so
                        // latest_ts is None; `popped` still reflects the drain.
                        return DrainOutcome {
                            slot: FrozenSlot::Empty,
                            popped: removed_in_drain.get().saturating_sub(junk_skipped.get()),
                            latest_ts: None,
                            decimated: true,
                        };
                    }
                }
            }
        }
        match latest {
            Some(sample) => {
                // A sample is being delivered to the node —
                // reset this input's `expect_within_ms` watchdog to the
                // sample's wire (publish-clock) timestamp so the scheduler's
                // next `step()` sees fresh data. Deterministic (wire ts is
                // data). No-op for un-watched inputs (`None`) and for
                // undersized frames with no readable wire timestamp. Read the
                // payload BEFORE `build_inbound_view` consumes the sample.
                //
                // CONTRACT: this reset is reached ONLY
                // for a DELIVERED sample. The `sample(N)` decimate path above
                // `return`s `Ok(None)` BEFORE this point, so a decimated frame
                // does NOT reset the watchdog — by design: the node never
                // observed that data, so it should not count as "fresh data
                // arrived". An input that declares BOTH `sample(N)` and
                // `expect_within_ms = M` with `N > M` will therefore trip the
                // watchdog (accepted frames are ≥ N ms apart > the M-ms
                // window). Pinned e2e by `expect_within_iox2_test`'s
                // sample-gate case; keep this ordering (gate decimate returns
                // first) if either block is refactored.
                if let Some(anchor) = &self.expect_within_last_data_ns {
                    if let Some(ts) = wire_timestamp_ns(sample.payload()) {
                        anchor.store(ts, Ordering::Release);
                    }
                }
                // Capture the surviving sample's wire timestamp BEFORE
                // it is moved into the FrozenSlot — the `signal_input_received`
                // watchdog reset value the runtime threads through the trigger.
                let latest_ts = wire_timestamp_ns(sample.payload());
                DrainOutcome {
                    slot: FrozenSlot::Sample(sample),
                    popped: removed_in_drain.get().saturating_sub(junk_skipped.get()),
                    latest_ts,
                    decimated: false,
                }
            }
            None => DrainOutcome {
                slot: FrozenSlot::Empty,
                popped: removed_in_drain.get().saturating_sub(junk_skipped.get()),
                latest_ts: None,
                decimated: false,
            },
        }
    }

    /// Freeze this input's latest sample at a step
    /// boundary. Drains + accounts ONCE here; the following `try_view`(s) serve
    /// the frozen slot without re-draining. Called by the runtime's snapshot
    /// pass for non-trigger latest-value inputs only.
    ///
    /// This is one of the slot's two CAPTURE SITES, and a capture is
    /// the ONLY thing that ends the previous slot — a serve does not (see
    /// the `FrozenSlot` type docs; it is private, so this is deliberately not
    /// an intra-doc link). The `frozen` write below is unconditional, so a step
    /// that snapshots always supersedes the previous step's slot whether or not
    /// any fire read it.
    ///
    /// HOLD the last-delivered sample across steps. `held_sample`
    /// survives an Empty drain so a step with no new arrival REPLAYS it. That
    /// keeps it borrowed ACROSS the drain, raising the per-connection peak to
    /// held(1) + latest(1) + receive-transient(1) = 3; snapshot-source topics
    /// provision `subscriber_max_borrowed_samples = 3` for exactly this.
    pub fn snapshot_latest(&mut self) {
        // HOLD the last-delivered sample across steps. We do NOT
        // clear `held_sample` before the drain — it must survive an Empty
        // drain to be replayed. That keeps it borrowed ACROSS the drain, so
        // the per-connection peak is held(1) + latest(1) + receive-transient(1)
        // = 3; snapshot-source topics provision subscriber_max_borrowed_samples
        // = 3 for exactly this. `frozen` for a non-trigger input is only ever
        // Held/Empty/Err after this fn (never a borrowed Sample), so overwriting
        // it below leaks no borrow.
        let outcome = self.drain_to_latest_with_accounting();
        // The NON-TRIGGER read outcome — classified HERE (not in the
        // drain) because only this caller knows the Empty→Held/Empty
        // upgrade. Exactly ONE record per snapshot; the body's later
        // `try_view` serves the frozen slot and records nothing (the
        // accounting-once contract — see `try_view`). Order of the arms:
        // Decimated wins over Held (the drain really popped + dropped a fresh
        // frame; the body still observes the held value per the hold rule). An Err
        // drain records nothing (out of the read-log contract, the TraceEntry
        // posture). Armed-only — checked before the served-seq parse.
        //
        // CLASSIFICATION ASYMMETRY vs `try_view`'s live arm — INTENTIONAL:
        // Empty upgrades to `Held` HERE because the frozen
        // slot really serves the held value to the body; `try_view`'s live
        // Empty serves `Ok(None)` and so correctly records `NoFrame`. Each
        // record mirrors its own path's serve.
        if self.read_capture_armed() {
            // The producer annotation rides every read that
            // SERVES a frame on a `multi_publisher_topics` edge — the fresh
            // one AND the held replay, since a held frame was
            // produced by some specific publisher too and the read log must
            // be able to say which.
            let annotate = self.capture_producer_token();
            // The step-boundary snapshot is taken FOR the node
            // body and served to it by the tick's `try_view` — the value the
            // record describes is what BODY code observes, so every arm here
            // is a BODY-site record. (The scheduler's own trigger drain is
            // `drain_for_trigger`, which stamps `Drain`.)
            match &outcome.slot {
                FrozenSlot::Sample(s) if annotate => self.stage_read_outcome_with_producer(
                    ReadOutcomeKind::Served,
                    wire_sequence(s.payload()),
                    outcome.popped,
                    producer_token(s.origin()),
                    ReadSiteRole::Body,
                ),
                FrozenSlot::Sample(s) => self.stage_read_outcome(
                    ReadOutcomeKind::Served,
                    wire_sequence(s.payload()),
                    outcome.popped,
                    ReadSiteRole::Body,
                ),
                // MISSING_PRODUCER_TAG: a DECIMATED
                // snapshot still SERVES. The gate dropped the fresh frame, so
                // `outcome.slot` is Empty — but the assignment below leaves
                // `frozen` at `Held` whenever `held_sample` is `Some`, i.e.
                // the body's `try_view` this step reads the HELD frame. The
                // record stays `Decimated` (the gate event is what happened,
                // and `resolve_seq_slot` resolves its served-seq slot to the
                // last ACCEPT, which IS that held frame's sequence), but on a
                // `multi_publisher_topics` edge it must still name WHO
                // produced the value the body observes — exactly as the
                // `Held` arm below does, and for the same reason: a
                // per-publisher sequence names no publisher on its own.
                // Without this, the ONE read path that serves a frame under a
                // non-`Held` kind is the one read the log cannot attribute.
                //
                // The other two `Decimated` sites are deliberately NOT
                // annotated, because neither serves anything: `try_view`'s
                // live arm returns `Ok(None)` on Empty, and
                // `drain_for_trigger`'s Empty means no fire.
                FrozenSlot::Empty if outcome.decimated => match &self.held_sample {
                    Some(held) if annotate => self.stage_read_outcome_with_producer(
                        ReadOutcomeKind::Decimated,
                        None,
                        outcome.popped,
                        producer_token(held.origin()),
                        ReadSiteRole::Body,
                    ),
                    _ => self.stage_read_outcome(
                        ReadOutcomeKind::Decimated,
                        None,
                        outcome.popped,
                        ReadSiteRole::Body,
                    ),
                },
                FrozenSlot::Empty => match &self.held_sample {
                    Some(held) if annotate => self.stage_read_outcome_with_producer(
                        ReadOutcomeKind::Held,
                        wire_sequence(held.payload()),
                        outcome.popped,
                        producer_token(held.origin()),
                        ReadSiteRole::Body,
                    ),
                    Some(held) => self.stage_read_outcome(
                        ReadOutcomeKind::Held,
                        wire_sequence(held.payload()),
                        outcome.popped,
                        ReadSiteRole::Body,
                    ),
                    None => self.stage_read_outcome(
                        ReadOutcomeKind::NoFrame,
                        None,
                        outcome.popped,
                        ReadSiteRole::Body,
                    ),
                },
                FrozenSlot::Err(_) | FrozenSlot::Held => {}
            }
        }
        match outcome.slot {
            FrozenSlot::Sample(s) => {
                // New sample this step: update the held value (drops the prior
                // held → releases its borrow) and serve it.
                self.held_sample = Some(s);
                self.frozen = Some(FrozenSlot::Held);
            }
            FrozenSlot::Empty => {
                // No new sample. After first delivery, REPLAY the held value.
                // Before first delivery, stay Empty so the node
                // WAITS (the macro's no-op-on-None gate) — never a fabricated
                // default on genuinely-absent data. NOTE: a replay goes through
                // the drain's Empty arm, which does NOT reset the expect_within
                // watchdog — correct: a held value is not fresh data, so a
                // genuinely-stopped producer still trips the watchdog.
                self.frozen = Some(if self.held_sample.is_some() {
                    FrozenSlot::Held
                } else {
                    FrozenSlot::Empty
                });
            }
            FrozenSlot::Err(e) => {
                // The drain already mutated accounting; replay the error once.
                // "Once" holds only if the firing node READS this input THIS
                // step (consuming the frozen Err via `try_view`); an UNREAD Err
                // is simply superseded by the next `snapshot_latest` (Empty →
                // Held replay, or a fresh Sample), consistent with latest-value
                // semantics — a node that skips a step doesn't accumulate stale
                // errors. A PERSISTENT drain error is surfaced DIRECTLY: `frozen`
                // stays `Err`, so each step's `try_view` returns `Err` (the held
                // value is NOT served on an erroring step). The `expect_within`
                // watchdog also trips (not reset on a non-Sample drain).
                // `held_sample` is untouched — a transient drain error must not
                // discard the last good held value — so the held value RESUMES
                // serving once the drain returns Empty (replay) or a fresh Sample
                // again.
                self.frozen = Some(FrozenSlot::Err(e));
            }
            // `Held` is produced ONLY by this fn's field assignment above — the
            // drain returns Sample/Empty/Err exclusively, so this is statically
            // impossible.
            FrozenSlot::Held => unreachable!("drain_to_latest_with_accounting never yields Held"),
        }
    }

    /// Drain this (data-trigger) input's body subscriber ONCE at the level
    /// boundary — the unified replacement for the separate trigger-drain subscriber.
    /// Runs the SAME single drain-and-account pass as `snapshot_latest` (accounting
    /// once), freezes the surviving sample so the node's later `try_view` serves it
    /// WITHOUT a second receive, and returns (popped, latest_ts) for the runtime to
    /// drive `signal_data` (once per popped frame) + `signal_input_received` (the
    /// watchdog reset to latest_ts). Clear-then-capture like `snapshot_latest`.
    /// # The role this mints is the CALLER's promise
    ///
    /// Both drain entry points stamp
    /// [`crate::read_outcome::ReadSiteRole::Drain`] as a compile-time constant,
    /// and offline the wire role is BELIEVED over the record's kind — so
    /// calling either of them from NODE CODE records a drain-site read that
    /// never happened, which `replay_rederive` then folds into
    /// `sync_input_timestamps`, counts toward the FIFO order and conserves
    /// `block` credit against. **They are scheduler-only.** The reachable
    /// wrappers on `AnySubscriber` (the type `NodeContext::subscriber_mut`
    /// hands a tick) are `pub(crate)` for exactly that reason; these two stay
    /// `pub` because `cerulion_core`'s integration tests drive a
    /// `CerulionSubscriber` directly (`sync_per_set_backpressure_iox2_test`),
    /// which is an out-of-crate caller — so the contract is stated rather than
    /// enforced here.
    pub fn snapshot_latest_for_trigger(&mut self) -> (u64, Option<u64>) {
        self.drain_for_trigger(TriggerDrainSite::Boundary)
    }

    /// The Data burst loop's BETWEEN-FIRES refill of this data-trigger
    /// input — the same one-receive-per-served-frame drain as
    /// [`Self::snapshot_latest_for_trigger`], asking a DIFFERENT question.
    ///
    /// The boundary drain asks "is there a frame to fire on?", and an unserved
    /// frozen head IS one, so it RE-OFFERS the head (Principle #6: the signal a
    /// deferred/collapsed fire consumed must be re-minted, or the node wedges).
    /// The refill asks "did the fire I just ran consume the head, and is there
    /// ANOTHER frame behind it?" — so a still-frozen head is the answer NO
    /// (`(0, None)`), and the loop's existing `popped == 0` break ends the burst
    /// with the head intact for the next boundary to re-offer.
    ///
    /// Without that split the two questions shared one answer and the refill
    /// counted a RE-OFFER as a fresh pop: a tick that fires but never reaches
    /// this input's `try_view` (the pre-first-delivery collapse — an
    /// earlier-declared non-trigger `#[input]` with no delivery yet short-circuits
    /// the macro's declaration-ordered `try_view` chain) leaves the head frozen,
    /// so the loop re-fired the SAME frame until the per-step cap, then set
    /// `data_backlog_hint` and had the live loop come straight back for more of
    /// the same — ~64 no-op fires per step at the 1 ms floor, 64 `TraceEntry`s
    /// for one frame, and a panicking tick reaching `MAX_CONSECUTIVE_PANICS`
    /// inside ONE step instead of three.
    ///
    /// The discriminator is the CALL SITE, declared by the caller — never a
    /// timestamp comparison, which cannot work: two frames published in one step
    /// under a `VirtualClock` legitimately share a stamp.
    /// # The role this mints is the CALLER's promise
    ///
    /// Both drain entry points stamp
    /// [`crate::read_outcome::ReadSiteRole::Drain`] as a compile-time constant,
    /// and offline the wire role is BELIEVED over the record's kind — so
    /// calling either of them from NODE CODE records a drain-site read that
    /// never happened, which `replay_rederive` then folds into
    /// `sync_input_timestamps`, counts toward the FIFO order and conserves
    /// `block` credit against. **They are scheduler-only.** The reachable
    /// wrappers on `AnySubscriber` (the type `NodeContext::subscriber_mut`
    /// hands a tick) are `pub(crate)` for exactly that reason; these two stay
    /// `pub` because `cerulion_core`'s integration tests drive a
    /// `CerulionSubscriber` directly (`sync_per_set_backpressure_iox2_test`),
    /// which is an out-of-crate caller — so the contract is stated rather than
    /// enforced here.
    pub fn refill_for_trigger(&mut self) -> (u64, Option<u64>) {
        self.drain_for_trigger(TriggerDrainSite::Refill)
    }

    // ==================================================================
    // The four transport ops the per-set Sync matcher DEMANDS.
    //
    // The matcher is a PURE function over `(heads, next_info, inputs,
    // window)` that cannot touch transport at all; it asks for facts by
    // returning a VERDICT, and the align driver performs the op and
    // re-runs it. That is what keeps the matcher oracle-testable with
    // hand-built inputs, keeps every FFI crossing demand-driven, and
    // leaves a hand-driven pure-scheduler embedder (which has no probe
    // channel, so every `next_info` reads `Unknown → None`) on clean
    // greedy membership.
    //
    // These four are the whole transport surface those verdicts need:
    //
    // * `NeedNext(i)`   → `sync_next_arrived`     (non-consuming probe)
    // * `NeedStamp(i)`  → `sync_peek_next_stamp`  (pop into `next_head`)
    // * `Advance(i)` / `DiscardTie(..)`
    //                   → `sync_discard_head`     (drop head, refill)
    // * the restore-boundary `Void`
    //                   → `sync_void_head`        (serve "no frame")
    //
    // PROMOTION is deliberately NOT one of them: R-promote lives inside
    // `drain_for_trigger` and R-pop′ inside `try_view`, so both ride
    // entry points that already exist and neither needs a new symbol.
    // ==================================================================

    /// `NeedNext`: the NON-CONSUMING gate/argmin probe — "has a second
    /// frame ARRIVED on this input?", asked without moving the data plane.
    ///
    /// A frame already staged in [`Self::next_head`] HAS arrived; it is simply
    /// no longer in the queue, so the queue probe cannot see it. Reporting it as
    /// absent would make the descent gate read a SCARCE partner where there is
    /// none — the one reading that lets a descent proceed — so the staged slot
    /// is answered first and the queue is consulted only when it is Vacant.
    ///
    /// The queue half goes through [`Self::has_pending_sample_checked`], never
    /// the error-swallowing [`Self::has_pending_sample`]: see that method for
    /// why `false` is the gate's PASS witness and therefore the one answer a
    /// lost error must never be able to fabricate.
    pub(crate) fn sync_next_arrived(&mut self) -> TransportResult<bool> {
        if self.next_head.is_some() {
            return Ok(true);
        }
        // The CHECKED twin, never `has_pending_sample`: that one is
        // `has_samples().unwrap_or(false)`, and `false` is this call site's
        // PASS WITNESS.
        self.has_pending_sample_checked()
    }

    /// Testing-only: arm a fire-once probe failure. See
    /// [`Self::fault_inject_sync_next_arrived_err`].
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn fault_inject_sync_next_arrived_err(&mut self) {
        self.fault_inject_sync_next_arrived_err = true;
    }

    /// `NeedStamp`: pop this input's next frame into
    /// [`Self::next_head`] and report its wire stamp — the only way a stamp can
    /// be learned, since nothing but a pop reveals one.
    ///
    /// IDEMPOTENT: a second peek on an input that already has a staged next
    /// returns the RETAINED stamp and pops NOTHING. That is what makes the
    /// matcher's per-pass scratch safe to re-read mid-walk, and it is why a
    /// stale "peek below the head" is impossible here rather than merely
    /// avoided — by R-order the staged stamp is `>=` the head's.
    ///
    /// Returns:
    ///
    /// * `Ok(Some(ts))` — a frame is staged and carries `ts`.
    /// * `Ok(None)` — nothing VALID exists to descend to. THREE reachable ways
    ///   to get here, one sound answer: a junk-only queue (the pop-one drain
    ///   pops PAST an undersized frame and finds nothing behind it), a
    ///   `sample(N)` decimation, and a genuinely empty queue. The driver maps
    ///   it to `next[i] := None`, i.e. the matcher fires greedily. (A staged
    ///   frame whose header is unreadable would answer `None` too, but that is
    ///   DEFENSIVE ONLY: `drain_one_with_accounting` skips any payload shorter
    ///   than a `WireHeader` as junk, so every `Sample` it yields carries a
    ///   readable stamp.)
    /// * `Err(e)` — the pop itself errored. `next_head` cannot hold an error and
    ///   there is NO body reader on this path (completeness gates the only read,
    ///   and an `Err`-frozen head never fills), so the store-and-replay rule the
    ///   `Data` path uses has nobody to replay to. The driver maps it to the
    ///   mutating-op `Failed` policy: TERMINATE the align pass for this boundary
    ///   and fire the current tuple, which is complete and in-window at any
    ///   `NeedStamp` (window death already ran). The accounting the drain
    ///   already mutated STANDS — accounting-once.
    ///
    /// OUT OF CONTRACT if the head is not a [`FrozenSlot::Sample`]: R-pop pops
    /// into `next_head` only while a real head is held, so reaching here
    /// otherwise means the matcher and the slots have desynced. It refuses to
    /// pop, says so loudly, and answers `Ok(None)` — the DESCENT-REFUSING
    /// answer, so a desync can never be what enables a descent.
    pub(crate) fn sync_peek_next_stamp(&mut self) -> TransportResult<Option<u64>> {
        // Idempotent peek: the retained stamp, no pop.
        if let Some(staged) = &self.next_head {
            return Ok(wire_timestamp_ns(staged.payload()));
        }
        // R-pop: a pop into `next_head` requires a `Sample` head to compare
        // against. `Sample` specifically — an `Empty` or `Err` head is what
        // makes `(Empty, Occupied)` / `(Err, Occupied)` unconstructible, and
        // staging behind one would put a frame in a slot nothing promotes from.
        if !matches!(self.frozen, Some(FrozenSlot::Sample(_))) {
            let (node_id, input) = self.bound_identity();
            debug_assert!(
                false,
                "NeedStamp on {node_id}/{input}: no Sample head to stage behind"
            );
            tracing::warn!(
                topic = %self.topic,
                node_id = %node_id,
                input = %input,
                "a per-set Sync `NeedStamp` reached this input with no \
                 `Sample` head held, which R-pop forbids — the matcher's view of \
                 the slots has desynced from the slots themselves. Refusing to pop \
                 (a pop here would stage a frame behind nothing, and nothing would \
                 ever promote it) and answering `no next frame`, which DISABLES the \
                 descent for this pass rather than enabling one on evidence that \
                 was never gathered"
            );
            return Ok(None);
        }
        // Pop EXACTLY one, in FIFO order, with the same accounting-once pass
        // every other read on this input runs.
        let outcome = self.drain_one_with_accounting();
        match outcome.slot {
            FrozenSlot::Sample(sample) => {
                let ts = wire_timestamp_ns(sample.payload());
                // Staged EXACTLY as `drain_for_trigger` stages its
                // own pop — one record per CONSUMED frame, at the moment it
                // leaves the queue. Its later PROMOTION mints a second record
                // (see `sync_discard_head`), and that one carries `popped: 0`
                // precisely because THIS record already accounted for the pop.
                if self.read_capture_armed() {
                    // On a `multi_publisher_topics` edge the
                    // served seq alone names no publisher (`sequence` is
                    // per-publisher), so a resume or a `--verify` resim cannot
                    // place this frame without its producer. The descent's pop
                    // is a CONSUMED frame like any other and carries the same
                    // annotation the boundary drain does.
                    // With the format-5 role definition: the R-pop is the SCHEDULER reading
                    // on the node's behalf (driven from `NodeEntry`'s
                    // `sync_head_op` seam, never from tick code) — but the frame
                    // it takes is PARKED in `next_head`, not fired on, so it is
                    // a `Peek` and not a `Drain`. That distinction is the whole
                    // point: `verify_sync` folds its input stamps
                    // last-write-wins, and this record's stamp is NEWER than the
                    // head's, so folding it judged the schedule against a frame
                    // the matcher looked at and did not align on.
                    if self.capture_producer_token() {
                        self.stage_read_outcome_with_producer(
                            ReadOutcomeKind::DrainedBatch,
                            wire_sequence(sample.payload()),
                            outcome.popped,
                            producer_token(sample.origin()),
                            ReadSiteRole::Peek,
                        );
                    } else {
                        self.stage_read_outcome(
                            ReadOutcomeKind::DrainedBatch,
                            wire_sequence(sample.payload()),
                            outcome.popped,
                            ReadSiteRole::Peek,
                        );
                    }
                }
                self.next_head = Some(sample);
                // SLOT ENTRY: the descent has taken a frame OUT of the
                // queue to learn its stamp and is holding it. Its pop already
                // decremented the mirror; re-derive so the producer keeps
                // counting it as unserved.
                self.reconcile_block_slot_debt();
                Ok(ts)
            }
            FrozenSlot::Empty => {
                // With the format-5 role definition: `Drain`, NOT `Peek`, and the arm above
                // is where the difference lives. `Peek` marks a record that
                // NAMES A PARKED FRAME — the thing `verify_sync` must exclude
                // from its head fold. A decimated pop parks nothing (`next_head`
                // stays empty and the caller answers `None`), so there is no
                // head to distinguish it from; it is an ordinary consuming
                // scheduler read whose frames left the queue. Keeping it
                // `Drain` is also what makes `Peek` + `Decimated` an
                // UNPRODUCIBLE pairing replay can convict on.
                if self.read_capture_armed() && outcome.decimated {
                    self.stage_read_outcome(
                        ReadOutcomeKind::Decimated,
                        None,
                        outcome.popped,
                        ReadSiteRole::Drain,
                    );
                }
                Ok(None)
            }
            FrozenSlot::Err(e) => Err(e),
            // No drain yields `Held` (it is written only by `snapshot_latest`),
            // so this is unreachable. Answered rather than `unreachable!`d: a
            // diagnostic slot state must never be able to abort the path that
            // observes it, and `None` is the descent-refusing answer.
            FrozenSlot::Held => Ok(None),
        }
    }

    /// `Advance` / `DiscardTie`: DROP this input's head and refill from
    /// the staged next — or, if nothing is staged, from the queue.
    ///
    /// The two verdicts share this one op because the transport does the same
    /// thing for both; what differs is WHY, and that is the caller's to record.
    /// The subscriber cannot tell an `Advance` (a nearer arrived member of the
    /// same stream was chosen — PASSED-OVER, the feature working) from a
    /// `DiscardTie` (a partner ran more than a window past this frame, so it is
    /// provably in no set — UNMATCHABLE, worth a loud regime), so it counts
    /// NEITHER and the matcher's driver counts the one it knows.
    ///
    /// Returns the new head's stamp: `Ok(Some(ts))` for a real frame,
    /// `Ok(None)` when the refill found nothing valid (empty / decimated /
    /// errored). It never returns `Err`: this op has ALREADY
    /// discarded a frame by the time a refill could fail, and telling its caller
    /// to terminate the pass on top of that would strand the input mid-advance.
    /// A failed refill FREEZES the `Err` exactly as [`Self::drain_for_trigger`]
    /// does, so the next boundary's drain replaces it and the input self-heals
    /// in one boundary (the `Err` carried no frame, so nothing is lost).
    pub(crate) fn sync_discard_head(&mut self) -> TransportResult<Option<u64>> {
        // The head really does leave — drop it (releasing its SHM borrow) and
        // close any held-head regime, exactly as a served head would. The
        // re-offer streak is a "nobody is reading this" heuristic, and a head
        // the matcher skipped is a head that will never be read.
        self.frozen = None;
        self.note_head_served();
        // R-promote's sibling: the staged next IS the FIFO-correct new head,
        // and it is already ACCOUNTED FOR (the `NeedStamp` pop decremented the
        // mirror), so this hand-off touches the block accounting not at all.
        if let Some(promoted) = self.next_head.take() {
            let ts = wire_timestamp_ns(promoted.payload());
            // With the format-5 role definition: the promotion DOES record, and it is the
            // other half of the peek/head mark. The peek's record names this
            // frame as PARKED; nothing else on the wire ever said it became the
            // HEAD, so `verify_sync` could not recover which frame the matcher
            // aligned on and had to decline the whole step. A `Drain`-role
            // record at the SAME sequence says "the head advanced to this
            // frame", and `popped: 0` says "nothing left the queue here" — the
            // frame was popped at its peek, and double-counting it would inflate
            // the FIFO pop sum and the `block` credit both drainers conserve.
            if self.read_capture_armed() {
                // Same annotation rule as every other record on this input: on a
                // `multi_publisher_topics` edge the sequence alone names no
                // publisher, and a promotion is addressed by sequence like the
                // rest.
                if self.capture_producer_token() {
                    self.stage_read_outcome_with_producer(
                        ReadOutcomeKind::DrainedBatch,
                        wire_sequence(promoted.payload()),
                        0,
                        producer_token(promoted.origin()),
                        ReadSiteRole::Drain,
                    );
                } else {
                    self.stage_read_outcome(
                        ReadOutcomeKind::DrainedBatch,
                        wire_sequence(promoted.payload()),
                        0,
                        ReadSiteRole::Drain,
                    );
                }
            }
            self.frozen = Some(FrozenSlot::Sample(promoted));
            // SLOT EXIT (the skipped head only): the promotion itself
            // moves a frame BETWEEN slots and changes nothing, so the net here
            // is exactly the one frame this op discarded.
            self.reconcile_block_slot_debt();
            return Ok(ts);
        }
        // Nothing staged (an `Advance` the gate reached without a `NeedStamp`,
        // or a `DiscardTie`): refill from the queue, with the ordinary
        // accounting-once pass and the ordinary read-log record.
        let outcome = self.drain_one_with_accounting();
        if self.read_capture_armed() {
            match &outcome.slot {
                // Same rule as the boundary drain — an
                // `Advance`/`DiscardTie` refill is a CONSUMED frame, and on a
                // multi-publisher edge its producer is what makes the record
                // placeable.
                // An `Advance`/`DiscardTie` refill is the
                // SCHEDULER's matcher taking a frame off the queue — a
                // DRAIN-site record.
                FrozenSlot::Sample(s) if self.capture_producer_token() => self
                    .stage_read_outcome_with_producer(
                        ReadOutcomeKind::DrainedBatch,
                        wire_sequence(s.payload()),
                        outcome.popped,
                        producer_token(s.origin()),
                        ReadSiteRole::Drain,
                    ),
                FrozenSlot::Sample(s) => self.stage_read_outcome(
                    ReadOutcomeKind::DrainedBatch,
                    wire_sequence(s.payload()),
                    outcome.popped,
                    ReadSiteRole::Drain,
                ),
                FrozenSlot::Empty if outcome.decimated => self.stage_read_outcome(
                    ReadOutcomeKind::Decimated,
                    None,
                    outcome.popped,
                    ReadSiteRole::Drain,
                ),
                FrozenSlot::Empty | FrozenSlot::Err(_) | FrozenSlot::Held => {}
            }
        }
        let ts = outcome.latest_ts;
        self.frozen = Some(outcome.slot);
        // The skipped head EXITED and a refill may have ENTERED — one
        // re-derivation settles both, whichever way they net out.
        self.reconcile_block_slot_debt();
        Ok(ts)
    }

    /// `Void`: serve a RESTORED (unbacked) head's read as "no frame",
    /// in place.
    ///
    /// A head restored from a bag names a STAMP but is backed by no live frame —
    /// the framework section is stamp-only, so restore mints a `Filled` head
    /// with nothing behind it. The scheduler's fire for that set is REAL and
    /// appears in the trace (a fire records even when the tick collapses), so
    /// the read has to answer something, and the only correct answer is
    /// `Ok(None)`.
    ///
    /// Freezing [`FrozenSlot::Empty`] is what makes it answer that WITHOUT
    /// draining: the live queue's next frame belongs to the NEXT set by the
    /// post-restore FIFO rule, and serving it here would consume a later set's
    /// member to satisfy a frame that no longer exists. The slot self-heals at
    /// the following boundary — the trigger drain's re-offer guard matches
    /// `Some(Sample)` ONLY, so an `Empty` head falls straight through it and the
    /// next drain pops normally.
    ///
    /// [`Self::next_head`] is deliberately untouched: descent is DISABLED for
    /// any boundary at which a head is unbacked, so no `NeedStamp` runs and the
    /// slot is Vacant by construction at a restore boundary. Leaving it alone
    /// keeps R-order total — a staged frame is never dropped silently, not even
    /// here.
    pub(crate) fn sync_void_head(&mut self) {
        self.frozen = Some(FrozenSlot::Empty);
        // A restore boundary reaches here with an UNBACKED head, so
        // there is normally nothing to release — but a slot state is a fact and
        // this is the one write that can overwrite a `Sample` without serving
        // it, so it restates the debt rather than assuming its own precondition.
        self.reconcile_block_slot_debt();
    }

    /// The ONE drain-and-account body both trigger sites share, so the pop path
    /// (and its read-outcome record) cannot drift between them.
    fn drain_for_trigger(&mut self, site: TriggerDrainSite) -> (u64, Option<u64>) {
        // Per-message FIFO (the data-trigger default): an UNSERVED frozen
        // head must never be overwritten. Two ways a fire can fail to
        // consume the head this boundary froze, unified by ONE rule:
        //
        // 1. The scheduler DEFERRED the decided fire (`throttle_ms`, the
        //    block pre-fire gate, the panic circuit breaker) — the pending
        //    count survives the defer, and the retried fire will take the
        //    head. (Measured pre-guard: a throttled consumer observed
        //    [1, 5] of a 6-frame burst — the next boundary popped frame 2
        //    over the unserved frame.)
        // 2. The fire RAN but its tick never reached this input's
        //    `try_view` — the macro's generated tick nests one `try_view`
        //    per input in DECLARATION order, and an earlier non-trigger
        //    context input with no delivery yet collapses the chain (the
        //    pre-first-delivery WAIT), so the trigger's read never
        //    runs. Here the pending count WAS consumed, and a guard that
        //    mints no new signal would wedge the node PERMANENTLY (no signal ⇒
        //    no fire ⇒ no read ⇒ no signal).
        //
        // So the guard holds the head, pops NOTHING, and — at the BOUNDARY —
        // RE-OFFERS it: `(1, head_ts)` re-mints one signal so a fire (or the
        // retried deferred one) keeps coming until a tick actually takes the
        // head — at which point the next boundary resumes popping. Case-1
        // re-offers inflate the pending count while the defer lasts; that is
        // bounded by the scheduler's saturation clamp, matches the pre-FIFO
        // boundary-drain signal cadence, and the excess self-drains as
        // no-op fires. The queue keeps every later frame and the block
        // mirror stays high for them (the producer stays paced).
        //
        // A REFILL asks the other question, "did the fire
        // I just ran consume the head?" — so it must NOT re-offer. A re-offer
        // counted as a fresh pop re-fires the SAME frame up to the per-step
        // cap; see [`Self::refill_for_trigger`].
        //
        // `expect_within` under a LONG defer (handled at the
        // scheduler seam): the re-offer carries the head's ORIGINAL wire
        // timestamp, and the watchdog measures elapsed-since-anchor — so
        // re-storing an old stamp does not stop the window growing, and newer
        // arrivals sit unobserved in the queue while the head is held. A
        // defer longer than `expect_within_ms` therefore lapses windows on an
        // input whose data is genuinely available. The BOUNDARY re-offer ALSO
        // mints a fresh arrival signal, and that is what the backlog rule keys on: a
        // lapsed window whose input still carries unconsumed signalled
        // arrivals is reported as BACKLOG
        // (`NodeHandle::expect_within_backlogged_count`) instead of counted as
        // a missed deadline, and no `ExpectWithinEvent` is emitted for it. See
        // `Scheduler::run_qos_windows`.
        //
        // The case the backlog rule CANNOT distinguish is case 2 above — a tick that
        // NEVER reaches this input's read re-offers the same head forever, so
        // the backlog never empties and the liveliness watchdog stays
        // suppressed even after the producer dies. That is what the streak
        // counter below is for: it is the only place in the system that can
        // tell "deferred, will be served" from "nobody is ever going to read
        // this".
        if self.consume_mode == ConsumeMode::EachFifo {
            // The timestamp is read BEFORE the streak bump so the `&self.frozen`
            // borrow ends first (`note_head_reoffered` needs `&mut self`).
            let held_head_ts = match &self.frozen {
                Some(FrozenSlot::Sample(sample)) => Some(wire_timestamp_ns(sample.payload())),
                _ => None,
            };
            if let Some(head_ts) = held_head_ts {
                // The held-head RE-OFFER records NOTHING in the
                // read log — no frame leaves the queue here, and the held
                // head's own `DrainedBatch` record was already staged at the
                // boundary that originally popped+froze it (accounting-once:
                // one record per CONSUMED frame, never one per re-offer — a
                // throttled defer must not mint N duplicate records for one
                // frame). Replay runs this same guard, so the log stays
                // symmetric.
                return match site {
                    TriggerDrainSite::Boundary => {
                        // The streak counts BOUNDARY re-offers
                        // ONLY, and the site split is what keeps that true. A
                        // boundary runs exactly once per step, which is the whole
                        // derivation of `HELD_HEAD_WARN_BOUNDARIES` (~1 s at a
                        // 1 kHz loop); a REFILL runs zero-or-more times per step
                        // depending on how many fires the burst served, so
                        // counting it too would make the threshold a function of
                        // burst depth rather than of wall time. A refill that
                        // finds the head still frozen is the SAME observation the
                        // boundary already counted this step, not a new one.
                        self.note_head_reoffered();
                        (1, head_ts)
                    }
                    // "Nothing new behind the head you are still holding."
                    TriggerDrainSite::Refill => (0, None),
                };
            }
        }
        // Reaching here means this drain is free to POP, i.e. any head a
        // previous drain froze was taken by a tick (or there was none). TRUE AT
        // EITHER SITE and deliberately reset at both: the guard above is the
        // only thing standing between a frozen head and this line, so a REFILL
        // reaching it proves the fire that just ran DID consume the head —
        // which is exactly the recovery condition, observed one step earlier
        // than the next boundary would see it.
        self.note_head_served();
        self.frozen = None;
        // A frame the descent staged in `next_head` is the
        // FIFO-correct answer, so PROMOTE it rather than popping the queue —
        // which would hand back a NEWER frame and invert this input's delivery
        // order.
        //
        // Reaching here proves `frozen` is None: the re-offer guard above
        // returned on `Some(Sample)`, and R-pop makes `(Empty, Occupied)` /
        // `(Err, Occupied)` unconstructible, so an Occupied `next_head` implies
        // the head was just served, skipped or death-discarded. That is true at
        // BOTH sites, which is exactly why the branch lives here rather than in
        // either caller: the next step's Boundary drain and the same-step burst
        // Refill both need it, and putting it on the shared body means it needs
        // no new FFI symbol — it rides the drain entry points that already
        // exist.
        //
        // Records NOTHING in the read log, and with the format-5 role definition that is
        // a DECISION rather than the accounting-once side effect it used
        // to be, because its sibling in `sync_discard_head` now DOES record.
        //
        // The two promotions are not interchangeable, and the difference is
        // WHEN they run. `sync_discard_head`'s is driven by the align pass, so
        // it lands strictly BEFORE the fire and the frame it installs really is
        // the head the scheduler decided on. THIS one is reached only when
        // `frozen` is None, i.e. after a tick TOOK the head — which is the
        // post-fire burst refill, in the SAME step.
        //
        // A pop-sum argument for the
        // silence ("a second record here would DOUBLE
        // the pop") is FALSE for the shape the sibling site already
        // mints. `sync_discard_head` records its own promotion as a
        // `Drain`-role `DrainedBatch` with `popped: 0`, and every pop-summing
        // consumer treats that shape as POP-NEUTRAL:
        // `replay_engine.rs`'s read-outcome classifier reads a Drain-role
        // zero-pop `DrainedBatch` as `RecordedReadBody::HandOff` (:8806), and
        // the injection planner's `fold_edge` simply `continue`s past a
        // hand-off without folding it into any pop sum; `verify_fifo` and
        // `verify_conservation` add its `popped == 0` to nothing either. A
        // `stage_read_outcome(DrainedBatch, seq, 0, Drain)` HERE would fold
        // identically — pop-neutral, not a double count — so recording would
        // not break any pop-summing consumer.
        //
        // The TRUE state is simpler and less flattering: this is an ACCEPTED
        // RESIDUAL, not a load-bearing silence. Leaving it unrecorded means a
        // step whose ONLY evidence an input's head is complete-and-unfired
        // (arm C's "owed fire" claim) is exactly an R-promoted head is
        // invisible to the rederivation verifier's NARROW map — the hole
        // `replay_rederive::is_sync_head_record`'s doc names. That can only
        // SUPPRESS arm C (a missed conviction), never manufacture a false
        // one, which is the direction the rule may safely err in — and is
        // why nobody has closed it. Pinned by
        // `a_non_improving_peek_is_ignored_by_the_fold_while_a_drain_is_not`
        // (the second leg of which stands the node down on the crafted record's
        // stamp regression) and by `verify_fifo`'s pop arithmetic.
        //
        // Nothing is lost by the silence. A `next_head` cannot survive a step
        // in which its input's head was NOT served (the re-offer guard above
        // returns while `frozen` is `Some(Sample)`, so this branch is
        // unreachable then), and when it IS served the burst refill promotes in
        // the same step — so the head at the NEXT fire is re-recorded by that
        // step's own boundary drain or by the matcher's ops, never left to this
        // one to announce.
        if let Some(promoted) = self.next_head.take() {
            let ts = wire_timestamp_ns(promoted.payload());
            self.frozen = Some(FrozenSlot::Sample(promoted));
            // Reaching here means the previous head was SERVED (the
            // re-offer guard above returns while it is not), so this restates
            // the debt after that exit; the promotion is slot-to-slot.
            self.reconcile_block_slot_debt();
            return (1, ts);
        }
        // Freeze the FIFO HEAD — one frame per boundary drain, older frames
        // stay queued for the following steps' drains (the scheduler fires
        // once per served frame). Latest mode keeps the drain-to-latest
        // freeze.
        let outcome = match self.consume_mode {
            ConsumeMode::Latest => self.drain_to_latest_with_accounting(),
            ConsumeMode::EachFifo => self.drain_one_with_accounting(),
        };
        // The UNIFIED trigger-drain read outcome — a DrainedBatch
        // (served-seq = the surviving/newest frame, popped = the batch), or a
        // Decimated when the sample gate dropped the survivor. A SILENT drain
        // (Empty, popped == 0 — every level pass on a quiet input) records
        // NOTHING, which is what keeps staging bounded on non-firing nodes;
        // an Err drain is out of the read-log contract. Exactly ONE record
        // per drain: the tick's later `try_view` serves the frozen slot
        // without re-draining (accounting-once — see `try_view`).
        //
        // Per-message FIFO (52125241e): an `EachFifo` boundary drain pops
        // exactly ONE frame, so its record is `DrainedBatch` with `popped` =
        // 1 (junk-skipped frames excluded by the drain) and `served_seq` =
        // the frozen head. Kept as `DrainedBatch`, not a new kind: it IS a
        // drained batch of one at the same SITE ROLE (the trigger boundary
        // drain), the vocabulary distinguishes site role rather than queue
        // discipline, and replay stages through this same match — so
        // RECORD==REPLAY symmetry holds with no wire change.
        if self.read_capture_armed() {
            match &outcome.slot {
                // The boundary drain's SURVIVING frame carries
                // its producer on a `multi_publisher_topics` edge.
                //
                // The trigger drain is the SCHEDULER reading on
                // the node's behalf — a DRAIN-site record — even though under
                // the UNIFIED discipline it runs on the node's own BODY
                // subscriber and lands in a `Body`-role STAGE. That divergence
                // is exactly why the wire role is the CALL SITE's and not the
                // stage's.
                FrozenSlot::Sample(s) if self.capture_producer_token() => self
                    .stage_read_outcome_with_producer(
                        ReadOutcomeKind::DrainedBatch,
                        wire_sequence(s.payload()),
                        outcome.popped,
                        producer_token(s.origin()),
                        ReadSiteRole::Drain,
                    ),
                FrozenSlot::Sample(s) => self.stage_read_outcome(
                    ReadOutcomeKind::DrainedBatch,
                    wire_sequence(s.payload()),
                    outcome.popped,
                    ReadSiteRole::Drain,
                ),
                FrozenSlot::Empty if outcome.decimated => self.stage_read_outcome(
                    ReadOutcomeKind::Decimated,
                    None,
                    outcome.popped,
                    ReadSiteRole::Drain,
                ),
                FrozenSlot::Empty | FrozenSlot::Err(_) | FrozenSlot::Held => {}
            }
        }
        let (popped, latest_ts) = (outcome.popped, outcome.latest_ts);
        self.frozen = Some(outcome.slot);
        // SLOT ENTRY: a `Sample` frozen here is held UNSERVED until a
        // tick reads it, so it stays the producer's occupancy. `Empty` / `Err`
        // hold no frame and re-derive to nothing.
        self.reconcile_block_slot_debt();
        (popped, latest_ts)
    }

    /// One more boundary went by with the SAME unserved FIFO head.
    /// Emits ONE loud line per held-head regime once the streak reaches
    /// [`HELD_HEAD_WARN_BOUNDARIES`] — never one per boundary.
    ///
    /// `warn!`, not `debug!`, and deliberately louder than the scheduler-side
    /// backlog report: a defer is the node behaving as its own declared rate
    /// cap says, whereas a head nobody ever reads is a wiring defect that
    /// ALSO disables this input's producer-liveliness watchdog for as long as
    /// it lasts. In a cdylib node this line is emitted by the cdylib's own
    /// statically-linked `cerulion_core`, so it reaches the operator through
    /// the node-side stderr subscriber.
    fn note_head_reoffered(&mut self) {
        self.held_head_reoffers = self.held_head_reoffers.saturating_add(1);
        if self.held_head_reoffers >= HELD_HEAD_WARN_BOUNDARIES && !self.held_head_warned {
            self.held_head_warned = true;
            let (node_id, input) = self.bound_identity();
            tracing::warn!(
                topic = %self.topic,
                node_id = %node_id,
                input = %input,
                reoffers = self.held_head_reoffers,
                "held FIFO head re-offered on this input for \
                 {HELD_HEAD_WARN_BOUNDARIES} consecutive level \
                 boundaries without any tick reading it. TWO causes reach this \
                 line and they need different remedies. (1) A LONG DEFER: the \
                 node's own declared `throttle_ms`, or a `block` gate held by a \
                 full downstream queue, has deferred the fire for longer than \
                 this many steps — the node is behaving as configured and the \
                 head WILL be served. (2) A COLLAPSED READ CHAIN: the tick RUNS \
                 but never reaches this input's read (a context input declared \
                 BEFORE it with no delivery yet short-circuits the macro's \
                 declaration-ordered chain — see the pre-first-delivery \
                 WAIT), so nothing will ever take the head. TELL THEM APART by \
                 this node's declaration: if it declares a `throttle_ms` at or \
                 above this many steps, or consumes a `block`-gated topic, \
                 suspect (1) and expect a recovery line at its next fire; if it \
                 declares neither, or no recovery line follows, it is (2). \
                 `NodeHandle::expect_within_backlogged_count` climbs under both, \
                 while `backpressure_block_fires_deferred_count` and \
                 `throttle` defers are non-zero only under (1). EITHER WAY, \
                 while this lasts every boundary re-signals the same arrival, so \
                 the input's `expect_within_ms` liveliness watchdog is held as \
                 BACKLOG and cannot report a dead producer"
            );
        }
    }

    /// The `(node_id, input)` this subscriber is bound to, for the
    /// held-head lines.
    ///
    /// `unified_bound` is set at exactly and only the Unified wiring site, and
    /// the streak is documented as reachable ONLY on that discipline — so the
    /// `None` arm is unreachable through production wiring. It renders an
    /// explicit marker rather than a plausible-looking name: a fabricated
    /// `node_id` an operator could grep for and never find is worse than a
    /// line that says the binding was absent.
    fn bound_identity(&self) -> (&str, &str) {
        match &self.unified_bound {
            Some((node_id, input)) => (node_id, input),
            None => ("<unbound>", "<unbound>"),
        }
    }

    /// A trigger drain is free to pop, so any previously frozen head
    /// was served. Closes a held-head regime (one recovery line, only if the
    /// warn actually fired) and re-arms it. Called from BOTH drain sites — see
    /// the comment at the call site for why a refill reaching it is proof of
    /// service rather than a spurious reset.
    fn note_head_served(&mut self) {
        if self.held_head_warned {
            let (node_id, input) = self.bound_identity();
            tracing::info!(
                topic = %self.topic,
                node_id = %node_id,
                input = %input,
                reoffers = self.held_head_reoffers,
                "held FIFO head served — a tick finally took it; the input's \
                 `expect_within_ms` watchdog is live again. This line closes \
                 BOTH regimes the warn names: a DEFER that ended AND a \
                 collapsed read chain that healed (the missing context \
                 delivered) reach this same recovery, so the CAUSE is told \
                 apart by the node's declaration (see the warn), never by \
                 this line's presence"
            );
            self.held_head_warned = false;
        }
        self.held_head_reoffers = 0;
    }

    /// Spin-poll the iceoryx2 receive queue until a sample with
    /// `WireHeader::sequence >= expected_seq` arrives, then run `f` over
    /// it.
    ///
    /// This bypasses the listener-based notification path entirely:
    ///
    /// - **No `Listener::wait_for_message`** — no kernel boundary, no
    ///   `pthread_cond_wait` / `sem_wait` round-trip, no semaphore signal
    ///   from the publisher's `notify()`.
    /// - **No `drain_stale_events`** — the listener is never read.
    /// - **No drain-keep-latest** — the first sample whose sequence
    ///   matches is returned. Older samples in the queue are skipped
    ///   (and dropped immediately) so the caller sees `expected_seq`'s
    ///   payload, not whatever happened to be at the head.
    ///
    /// The hot loop is just `subscriber.receive()` (a lock-free queue
    /// pop) followed by `std::hint::spin_loop()` on no-data. This matches
    /// iceoryx2's own published latency-benchmark style.
    ///
    /// # When to use
    ///
    /// For applications that need sub-microsecond receive latency and
    /// can dedicate a CPU core to spinning. **Spinning consumes 100% of
    /// a core** while waiting — use [`Self::try_view`] for general-purpose
    /// receive that yields when no sample is available.
    ///
    /// # Returns
    ///
    /// - `Ok(Some(R))` — a sample with `sequence >= expected_seq` was
    ///   received within `timeout`; `f`'s return value is forwarded.
    /// - `Ok(None)` — `timeout` elapsed without a matching sample. `f`
    ///   was not called.
    /// - `Err(SchemaMismatch)` — a sample arrived but its schema hash
    ///   didn't match `T::SCHEMA_HASH`.
    /// - `Err(Receive)` — the iceoryx2 subscriber returned an error.
    /// - `Err(Deserialization)` — a sample was undersized or its
    ///   `WireHeader` could not be parsed.
    ///
    /// # Determinism
    ///
    /// Because spin returns the *first* sample with `sequence >= expected_seq`,
    /// callers must keep their own sequence counter and increment it
    /// monotonically — request-response benchmarks pass `seq + 1` each
    /// round-trip and get back exactly the expected reply.
    #[must_use = "spin_view_until_seq result must be checked"]
    pub fn spin_view_until_seq<T: ShmMessage, R>(
        &mut self,
        expected_seq: u32,
        timeout: Duration,
        f: impl FnOnce(InputView<'_, T>) -> R,
    ) -> TransportResult<Option<R>> {
        // The spin path is the raw, sub-microsecond benchmark receive.
        let sub = &self.subscriber;
        let deadline = Instant::now() + timeout;
        loop {
            match sub.receive() {
                Ok(Some(sample)) => {
                    // Validate WireHeader.
                    let raw = sample.payload();
                    if raw.len() < WireHeader::SIZE {
                        return Err(TransportError::Deserialization {
                            topic: self.topic.clone(),
                            reason: format!(
                                "undersized message: {} bytes, need at least {}",
                                raw.len(),
                                WireHeader::SIZE,
                            ),
                        });
                    }
                    let header = WireHeader::read_from_buf(raw).ok_or_else(|| {
                        TransportError::Deserialization {
                            topic: self.topic.clone(),
                            reason: "failed to parse WireHeader from received message".to_string(),
                        }
                    })?;
                    if header.schema_hash != T::SCHEMA_HASH {
                        return Err(TransportError::SchemaMismatch {
                            topic: self.topic.clone(),
                            expected_hash: T::SCHEMA_HASH,
                            actual_hash: header.schema_hash,
                        });
                    }
                    // Skip stale samples (sequence < expected_seq). Drop
                    // the iceoryx2 sample to release the SHM slot, then
                    // continue the spin.
                    if header.sequence < expected_seq {
                        drop(sample);
                        if Instant::now() > deadline {
                            return Ok(None);
                        }
                        std::hint::spin_loop();
                        continue;
                    }
                    // See drain_samples for the
                    // rationale; this is the spin-loop variant of the
                    // same total_size validation pattern.
                    let total_size = header.total_size as usize;
                    if total_size < WireHeader::SIZE || total_size > raw.len() {
                        return Err(TransportError::Deserialization {
                            topic: self.topic.clone(),
                            reason: format!(
                                "wire header total_size {} out of bounds (frame {} bytes, header {} bytes)",
                                total_size,
                                raw.len(),
                                WireHeader::SIZE,
                            ),
                        });
                    }
                    let payload_ptr = raw.as_ptr();
                    let payload_len = total_size - WireHeader::SIZE;
                    // SAFETY: `raw` is a valid slice into the iceoryx2
                    // SHM region owned by `sample`. We move the sample
                    // into SampleHandle::Inbound below — same scope,
                    // same lifetime — so the reader's borrow stays alive
                    // and aliasing-free for `f`'s scope. The bound check
                    // above guarantees `payload_len` and the offset stay
                    // within `raw`.
                    let payload_slice: &[u8] = unsafe {
                        std::slice::from_raw_parts(payload_ptr.add(WireHeader::SIZE), payload_len)
                    };
                    let reader = T::build_reader(payload_slice);
                    let handle = SampleHandle::Inbound {
                        sample,
                        _phantom: std::marker::PhantomData,
                    };
                    let view = InputView::new(handle, reader);
                    return Ok(Some(f(view)));
                }
                Ok(None) => {
                    if Instant::now() > deadline {
                        return Ok(None);
                    }
                    std::hint::spin_loop();
                }
                Err(e) => {
                    return Err(TransportError::Receive {
                        topic: self.topic.clone(),
                        reason: format!("{}", e),
                    });
                }
            }
        }
    }

    /// Returns the topic name.
    pub fn topic(&self) -> &str {
        &self.topic
    }

    /// Production path: borrow this subscriber's iceoryx2
    /// event `Listener` so the WaitSet reactor can attach it as a
    /// notification source (the data-trigger drain in `drain_level`
    /// reads via `try_receive` off the SHM MESSAGE queue, NOT a `Listener`
    /// wait — so a WaitSet attachment of the same `Listener` does not
    /// conflict with the deterministic firing path). Rooted by the
    /// production `GraphRuntime::run_live` live loop (and the still-gated
    /// `run_waitset_reactor_once_for_test` seam). The reactor records which
    /// sources fired and clears the `Listener`'s EVENT-notification queue;
    /// it never reads/consumes the data sample and never fires the scheduler.
    pub(crate) fn listener(&self) -> &Listener<CerService> {
        &self.listener
    }
}

/// Validates a raw frame's `WireHeader` bounds and returns its total length.
fn validate_raw_frame(topic: &str, raw: &[u8]) -> TransportResult<usize> {
    if raw.len() < WireHeader::SIZE {
        return Err(TransportError::Deserialization {
            topic: topic.to_string(),
            reason: format!(
                "undersized message: {} bytes, need at least {}",
                raw.len(),
                WireHeader::SIZE
            ),
        });
    }
    let header = WireHeader::read_from_buf(raw).ok_or_else(|| TransportError::Deserialization {
        topic: topic.to_string(),
        reason: "failed to parse WireHeader from received message".to_string(),
    })?;
    let total_size = header.total_size as usize;
    if total_size < WireHeader::SIZE || total_size > raw.len() {
        return Err(TransportError::Deserialization {
            topic: topic.to_string(),
            reason: format!(
                "invalid total_size {total_size} for {}-byte message",
                raw.len()
            ),
        });
    }
    Ok(total_size)
}

/// Build an `InputView` from an owned inbound iceoryx2 sample,
/// validating its `WireHeader`. Used by the `try_view` receive path.
/// Returns `Err` on undersized / schema-mismatch / out-of-bounds
/// frames; otherwise runs `f` over the SHM-backed view.
fn build_inbound_view<T: ShmMessage, R>(
    topic: &str,
    sample: &InboundSample,
    f: impl FnOnce(InputView<'_, T>) -> R,
) -> TransportResult<R> {
    let raw = sample.payload();
    if raw.len() < WireHeader::SIZE {
        return Err(TransportError::Deserialization {
            topic: topic.to_string(),
            reason: format!(
                "undersized message: {} bytes, need at least {}",
                raw.len(),
                WireHeader::SIZE,
            ),
        });
    }
    let header = WireHeader::read_from_buf(raw).ok_or_else(|| TransportError::Deserialization {
        topic: topic.to_string(),
        reason: "failed to parse WireHeader from received message".to_string(),
    })?;
    if header.schema_hash != T::SCHEMA_HASH {
        return Err(TransportError::SchemaMismatch {
            topic: topic.to_string(),
            expected_hash: T::SCHEMA_HASH,
            actual_hash: header.schema_hash,
        });
    }
    // Explicit lower-bound check (no silent
    // `.max(WireHeader::SIZE)` coercion of a malformed sub-header size).
    let total_size = header.total_size as usize;
    if total_size < WireHeader::SIZE || total_size > raw.len() {
        return Err(TransportError::Deserialization {
            topic: topic.to_string(),
            reason: format!(
                "wire header total_size {} out of bounds (frame {} bytes, header {} bytes)",
                total_size,
                raw.len(),
                WireHeader::SIZE,
            ),
        });
    }
    let payload_ptr = raw.as_ptr();
    let payload_len = total_size - WireHeader::SIZE;
    // SAFETY: `raw` is a valid slice into the iceoryx2 SHM region owned by
    // `sample`, which is BORROWED — it is owned by the CALLER and guaranteed
    // to outlive this view (the caller holds it for `f`'s entire scope). We
    // build SampleHandle::InboundRef from the same `&sample` immediately
    // below, so the reader's borrow and the handle both point at the same
    // SHM slot and stay alive and aliasing-free for `f`'s entire scope. The
    // bound check above keeps `payload_ptr.add(WireHeader::SIZE)` +
    // `payload_len` within the original `raw` slice.
    let payload_slice: &[u8] =
        unsafe { std::slice::from_raw_parts(payload_ptr.add(WireHeader::SIZE), payload_len) };
    let reader = T::build_reader(payload_slice);
    let handle = SampleHandle::InboundRef { sample };
    let view = InputView::new(handle, reader);
    Ok(f(view))
}

/// Validate a raw wire frame and deliver it to `callback` as a
/// `ReceivedMessage`. Returns `true` if delivered, `false` if skipped
/// (a `warn` was emitted). Used by the `drain_samples` receive path.
fn deliver_raw_frame<F>(topic: &str, raw: &[u8], callback: &mut F) -> bool
where
    F: FnMut(ReceivedMessage<'_>),
{
    if raw.len() < WireHeader::SIZE {
        tracing::warn!(
            topic = %topic,
            size_bytes = raw.len(),
            "received undersized message, skipping"
        );
        return false;
    }
    let header = match WireHeader::read_from_buf(raw) {
        Some(h) => h,
        None => {
            // Never skip a malformed frame silently — mirror the
            // undersized/out-of-bounds branches + the sibling
            // try_receive_one.
            tracing::warn!(
                topic = %topic,
                size_bytes = raw.len(),
                "wire header read failed, skipping"
            );
            return false;
        }
    };
    // Validate total_size against the
    // header lower bound and the raw frame upper bound; clamp the payload
    // slice so trailing-pad bytes don't leak into user-visible data.
    let total_size = header.total_size as usize;
    if total_size < WireHeader::SIZE || total_size > raw.len() {
        tracing::warn!(
            topic = %topic,
            total_size,
            frame_len = raw.len(),
            "wire header total_size out of bounds, skipping"
        );
        return false;
    }
    let payload = &raw[WireHeader::SIZE..total_size];
    callback(ReceivedMessage::new(header, payload));
    true
}

/// Drive a `receive()` loop to exhaustion (or the first error) while keeping
/// the `block` outstanding mirror correct on EVERY exit path.
///
/// `record_drained` is invoked with the number of samples popped from the
/// queue when the internal RAII guard drops — which is on **every** exit
/// path: normal return, the `?` early-return when `recv()` errors mid-drain,
/// AND panic-unwind (e.g. a user `on_sample` callback panicking — note
/// `drain_samples` delivers through a user closure). Every popped sample is
/// already off the iceoryx2 queue, so its decrement must not be stranded:
/// otherwise `block_outstanding` is permanently inflated and the producer's
/// pre-fire defer wedges forever (a deadlock, not a dropped frame). The
/// receive error, if any, is propagated only after the guard has recorded.
/// `record_block_drained` is saturating and a no-op on 0, so firing from
/// Drop is always safe.
///
/// This is pure control flow extracted from `drain_samples` / `try_view` so
/// the always-account invariant is unit-testable without a live iceoryx2
/// subscriber (see `tests`). Both call sites route through it, so a
/// regression in the accounting fails the unit suite loud — the test that
/// the "decrement bypassed on error" class asks
/// for, plus the panic path.
fn drain_with_block_accounting<S, E>(
    mut recv: impl FnMut() -> Result<Option<S>, E>,
    mut on_sample: impl FnMut(S),
    record_drained: impl FnOnce(u64),
) -> Result<(), E> {
    // RAII accountant: records the popped count from `Drop`, so it fires on
    // normal return, `?` early-return, and panic-unwind alike. This is the
    // "set state for the duration of a scope → release it in Drop" pattern;
    // a post-loop call (no guard) would leak the count on unwind.
    struct Accountant<G: FnOnce(u64)> {
        removed: u64,
        record: Option<G>,
    }
    impl<G: FnOnce(u64)> Drop for Accountant<G> {
        fn drop(&mut self) {
            if let Some(record) = self.record.take() {
                record(self.removed);
            }
        }
    }
    let mut acct = Accountant {
        removed: 0,
        record: Some(record_drained),
    };
    while let Some(sample) = recv()? {
        // Each pop removes a sample from the iceoryx2 queue — the `block`
        // mirror tracks REMOVALS regardless of what `on_sample` does with the
        // sample (deliver, drop-as-stale, view-latest).
        acct.removed += 1;
        on_sample(sample);
    }
    Ok(())
}

impl Drop for CerulionSubscriber {
    fn drop(&mut self) {
        // A subscriber going away takes its held members with it, so
        // give the producer back every slot this input still owed. Hygiene
        // rather than a live path — a graph tearing down normally drops the
        // producer too — but a mirror is SHARED state and leaving it inflated
        // is the one failure mode that outlives the object holding it.
        self.frozen = None;
        self.next_head = None;
        self.reconcile_block_slot_debt();
        // Best-effort notification — Drop must not panic
        let _ = self
            .notifier
            .notify_with_custom_event_id(PubSubEvent::SubscriberDisconnected.into());
        tracing::debug!(topic = %self.topic, "subscriber disconnected, sent SubscriberDisconnected event");
    }
}

// ---------------------------------------------------------------------------
// This module's half of the ABI LAYOUT PIN (see `crate::abi_layout`).
//
// `abi_pin_struct!` expands to an exhaustive destructuring pattern with no `..`
// rest pattern, so adding or removing a field of one of these structs is a
// COMPILE ERROR naming the struct and the field; it also measures
// size/align/`offset_of!`, which `crate::abi_layout` compares against the
// snapshot table keyed to `CERULION_ABI_VERSION`. `abi_pin_enum!` does the
// same for a variant set (an enum carries no stable field offsets).
// ---------------------------------------------------------------------------
#[cfg(test)]
pub(crate) fn abi_layout_pins() -> Vec<crate::abi_layout::MeasuredStruct> {
    use crate::abi_layout::{abi_pin_enum, abi_pin_struct};
    vec![
        abi_pin_struct!(CerulionSubscriber {
            topic,
            subscriber,
            listener,
            notifier,
            probe,
            pending_backpressure_event,
            fault_inject_receive_after,
            // ABI v18: the topic's effective iceoryx2
            // `subscriber_max_borrowed_samples`, cached at creation for the
            // rmw's adopt-take refusal diagnostics. A `usize` that moves
            // every later field — NOT an additive change: `NodeContext`
            // owns this struct through `AnySubscriber`, so a v17 cdylib
            // would index it at stale offsets (the v18 lib.rs paragraph).
            max_borrowed_samples,
            max_publishers,
            drain_scratch,
            expect_within_last_data_ns,
            per_set_sync_trigger,
            frozen,
            next_head,
            block_slot_debt,
            fault_inject_sync_next_arrived_err,
            held_sample,
            unified_bound,
            unified_receive_warned,
            read_stage,
            multi_publisher_edge,
            consume_mode,
            held_head_reoffers,
            held_head_warned,
            served_sequence
        }),
        abi_pin_struct!(SampleGate {
            interval_ns,
            interval_ms,
            last_accepted_ns,
            counters,
            node_id,
            input,
            armed,
            regime_started_ns,
            regime_count,
            buffer_capacity
        }),
        abi_pin_struct!(DropOldestProbe {
            baselines,
            max_baselines,
            drain_counter,
            counters,
            node_id,
            input,
            armed,
            backward_run,
            regime_started_ns,
            regime_count,
            buffer_capacity
        }),
        abi_pin_struct!(BlockProbe {
            outstanding,
            threshold,
            counters,
            input,
            armed,
            regime_started_ns,
            regime_count,
            buffer_capacity
        }),
        abi_pin_struct!(StreamObservation {
            id,
            first_seq,
            newest_seq,
            newest_ts
        }),
        abi_pin_enum!(BackpressureProbe {
            BackpressureProbe::Sample(_),
            BackpressureProbe::DropOldest(_),
            BackpressureProbe::Block(_)
        }),
        abi_pin_enum!(FrozenSlot {
            FrozenSlot::Sample(_),
            FrozenSlot::Held,
            FrozenSlot::Empty,
            FrozenSlot::Err(_)
        }),
        abi_pin_enum!(ConsumeMode {
            ConsumeMode::Latest,
            ConsumeMode::EachFifo
        }),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The capacity a HAND-MINTED drain stage in this module stands
    /// in for. A wired stage's capacity is DERIVED from its edge's graph facts;
    /// these arms mint stages directly (there is no graph here), so the shape is
    /// DECLARED — a single-publisher trigger DRAIN at the default consumer
    /// depth, firing at the within-step burst clamp.
    fn hand_wired_drain_sizing() -> crate::read_outcome::ReadStageSizing {
        crate::read_outcome::ReadStageSizing {
            depth: crate::graph::topology::DEFAULT_CONSUMER_DEPTH as u32,
            annotated: crate::read_outcome::ProducerAnnotation::Plain,
            sync_shape: crate::read_outcome::SyncTriggerShape::Ordinary,
            // Routed through the ONE mint: a DRAIN stage is never frozen, so
            // the resolver hands back the node's own fire burst.
            burst: crate::graph::runtime::read_stage_burst_bound(
                crate::read_outcome::ReadStageRole::Drain,
                "inp",
                &[],
                false,
                crate::read_outcome::FireBurstBound::Fires(
                    crate::scheduler::DATA_PENDING_CARRY_CLAMP as u32,
                ),
            ),
        }
    }

    /// The GATE'S PROBE MUST NOT SWALLOW A TRANSPORT ERROR.
    ///
    /// `sync_next_arrived` is the matcher's non-consuming "has another frame
    /// arrived?" question, and at the GATE its `false` is not merely an answer —
    /// it is the PASS WITNESS that PERMITS a descent. The shipped
    /// `has_pending_sample` is `has_samples().unwrap_or(false)`, which is
    /// fail-SAFE for its rmw-readiness caller and INVERTED here: a swallowed
    /// iceoryx2 error would vouch for scarcity on an input nobody verified, the
    /// gate would pass, and descent would run into an unprobed backlog and
    /// destroy arrived complete in-window sets — the violation the whole
    /// R-Fail policy exists to prevent.
    ///
    /// So the probe rides the CHECKED twin and the error must SURFACE, where
    /// the driver maps it through R-Fail (at the gate: `Present`, i.e. this
    /// input REFUSES the gate). Nothing can make a healthy subscriber's
    /// `has_samples()` fail on demand, which is why the fault seam exists.
    ///
    /// Both halves in one body: the armed call must be `Err`, and the CONTROL
    /// immediately after must be a normal `Ok` — without it, "returns Err" is
    /// satisfied by a probe that is simply broken.
    #[test]
    fn the_sync_gate_probe_surfaces_a_transport_error_rather_than_reading_it_as_no_frame() {
        use crate::transport::{TransportConfig, TransportManager};
        use crate::wire::MaxSliceLen;

        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "probe_fault".into(),
                clock: Arc::new(crate::clock::RealClock),
                subscriber_buffer_size: 8,
                network: None,
            },
            crate::testing::iceoryx_test_config(),
        )
        .expect("per-test SHM transport");

        let topic = "probe_fault";
        let _publisher = mgr
            .create_publisher(topic, MaxSliceLen::try_new(1024).expect("len"), 0)
            .expect("publisher");
        let mut sub = mgr.create_subscriber(topic).expect("subscriber");

        sub.fault_inject_sync_next_arrived_err();
        let armed = sub.sync_next_arrived();
        assert!(
            armed.is_err(),
            "a failing probe must SURFACE its error: at the gate, `Ok(false)` is \
             the PASS WITNESS, so swallowing the failure fabricates scarcity on \
             an unverified input and lets descent destroy arrived complete sets. \
             Got {armed:?}"
        );

        // CONTROL, same subscriber, immediately after: the fault is fire-once
        // and an empty queue answers normally. Without this, the assertion above
        // is satisfied by a probe that always errors.
        assert!(
            !sub.sync_next_arrived().expect("the fault is fire-once"),
            "an empty queue is a genuine `false` — the correct answer the gate is \
             entitled to act on"
        );
    }

    // Deadlock guard: the `block` outstanding
    // mirror MUST be decremented for every popped sample on EVERY exit path
    // of the receive loop. A mid-drain `receive()` error that skipped the
    // decrement would permanently inflate `block_outstanding` and wedge the
    // producer's pre-fire defer forever. `drain_samples` + `try_view` both
    // route through `drain_with_block_accounting`, so these pin the invariant
    // for both — revert the helper to a record-after-`?` order and all three
    // fail loud.

    #[test]
    fn block_accounting_records_count_when_receive_errors_after_some_pops() {
        let mut next = 0u32;
        let mut delivered = 0u32;
        let mut recorded: Option<u64> = None;
        // recv: Ok(Some), Ok(Some), then Err — error AFTER 2 samples popped.
        let outcome: Result<(), &str> = drain_with_block_accounting(
            || {
                next += 1;
                match next {
                    1 | 2 => Ok(Some(next)),
                    _ => Err("receive failed mid-drain"),
                }
            },
            |_sample| delivered += 1,
            |removed| recorded = Some(removed),
        );
        // The error propagates...
        assert_eq!(outcome, Err("receive failed mid-drain"));
        // ...but the mirror was STILL decremented by the 2 already popped.
        assert_eq!(
            recorded,
            Some(2),
            "must account for samples popped before the error"
        );
        assert_eq!(delivered, 2);
    }

    #[test]
    fn block_accounting_records_full_count_on_clean_drain() {
        let mut next = 0u32;
        let mut recorded: Option<u64> = None;
        let outcome: Result<(), &str> = drain_with_block_accounting(
            || {
                next += 1;
                if next <= 3 {
                    Ok(Some(()))
                } else {
                    Ok(None)
                }
            },
            |_sample| {},
            |removed| recorded = Some(removed),
        );
        assert_eq!(outcome, Ok(()));
        assert_eq!(recorded, Some(3));
    }

    #[test]
    fn block_accounting_records_zero_when_first_receive_errors() {
        let mut recorded: Option<u64> = None;
        // Err on the very first receive → nothing popped → record 0 (no
        // spurious decrement that would underflow the saturating mirror).
        let outcome: Result<(), &str> = drain_with_block_accounting(
            || Err("immediate failure"),
            |_sample: ()| {},
            |removed| recorded = Some(removed),
        );
        assert_eq!(outcome, Err("immediate failure"));
        assert_eq!(recorded, Some(0));
    }

    #[test]
    fn block_accounting_records_count_when_on_sample_panics_mid_drain() {
        use std::cell::Cell;
        use std::panic::{catch_unwind, AssertUnwindSafe};
        // The deadlock the helper guards is most realistically reached via a
        // panicking user callback: `drain_samples`'s `on_sample` runs the
        // user's `callback`. If a panic skipped the decrement, the popped
        // samples (already off the queue) would inflate `block_outstanding`
        // forever. The RAII guard must fire on unwind — this pins it.
        let recorded: Cell<Option<u64>> = Cell::new(None);
        let mut next = 0u32;
        let panicked = catch_unwind(AssertUnwindSafe(|| {
            let _: Result<(), &str> = drain_with_block_accounting(
                || {
                    next += 1;
                    Ok(Some(next))
                },
                |sample| {
                    if sample == 3 {
                        panic!("user callback panicked on the 3rd sample");
                    }
                },
                |removed| recorded.set(Some(removed)),
            );
        }));
        assert!(panicked.is_err(), "the panic must propagate");
        // All 3 samples were popped (the 3rd triggered the panic in
        // on_sample, but recv had already removed it from the queue), so the
        // guard must have recorded 3 from Drop during unwind.
        assert_eq!(
            recorded.get(),
            Some(3),
            "the Drop guard must record popped count even on panic-unwind"
        );
    }

    #[test]
    fn block_accounting_count_is_removals_not_deliveries() {
        // The `block` mirror tracks queue REMOVALS, not what `on_sample`
        // accepts: `drain_samples` drops undersized frames (deliver_raw_frame
        // returns false) yet still removed them from the queue. Pin that the
        // recorded count == pops, independent of a caller-side delivered tally.
        let mut next = 0u32;
        let mut delivered = 0u32;
        let mut recorded: Option<u64> = None;
        let outcome: Result<(), &str> = drain_with_block_accounting(
            || {
                next += 1;
                if next <= 4 {
                    Ok(Some(next))
                } else {
                    Ok(None)
                }
            },
            // Simulate the undersized-frame filter: only "deliver" even
            // samples. removed (4) must diverge from delivered (2).
            |sample| {
                if sample % 2 == 0 {
                    delivered += 1;
                }
            },
            |removed| recorded = Some(removed),
        );
        assert_eq!(outcome, Ok(()));
        assert_eq!(recorded, Some(4), "mirror decrements by pops (removals)");
        assert_eq!(delivered, 2, "caller delivered only the even samples");
    }

    /// A plausible concern, REFUTED WITH EVIDENCE.
    ///
    /// The concern: the legacy-`Sync` drain requests
    /// `max(depth, subscriber_buffer_size)` slots while its `Drain` stage is
    /// sized from the DECLARED `depth`, so a topic another consumer provisioned
    /// deeper can drain more records than the stage holds and lose read-log
    /// records.
    ///
    /// It cannot: a `Drain` stage's record population is not a function of the
    /// batch SIZE. `drain_samples` stages ONE `DrainedBatch` per non-empty
    /// drain — `if capture && count > 0 { .. stage once .. }` — and the number
    /// of frames popped rides that single record as its `popped` FIELD. A
    /// deeper queue therefore yields a bigger `popped`, never more records.
    ///
    /// This drives exactly the shape the concern describes: a subscriber
    /// buffer of 16 against a stage sized from a DECLARED depth of 2 (whose
    /// derived capacity is far below the 16 frames queued), drained in one
    /// pass. If the concern were right this would truncate.
    #[test]
    fn a_drain_stage_holds_a_batch_deeper_than_its_declared_depth() {
        use crate::read_outcome::{
            ProducerAnnotation, ReadOutcomeKind, ReadOutcomeStage, ReadStageRole, ReadStageSizing,
            StagedReadOutcome, SyncTriggerShape,
        };
        use crate::transport::{TransportConfig, TransportManager};
        use crate::wire::MaxSliceLen;

        // The provisioned queue is 16 — what a deeper co-consumer forces via
        // `depth.max(sub_buf)` at the legacy-Sync drain site.
        const PROVISIONED: usize = 16;
        // The DECLARED depth this edge's stage is sized from (the
        // declaration is the sizing contract).
        const DECLARED_DEPTH: u32 = 2;

        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "deep_batch".into(),
                clock: Arc::new(crate::clock::RealClock),
                subscriber_buffer_size: PROVISIONED,
                network: None,
            },
            crate::testing::iceoryx_test_config(),
        )
        .expect("per-test SHM transport");

        let frame = |seq: u32| -> Vec<u8> {
            let mut header = crate::wire::WireHeader::new(0xC1487, seq, 2_000 + u64::from(seq));
            let total = crate::wire::WireHeader::SIZE + 8;
            header.total_size = total as u32;
            let mut buf = vec![0u8; total]; // hot-path-alloc-ok: test-only frame builder.
            header.write_to_buf(&mut buf);
            buf
        };

        let topic = "deep_batch";
        let mut publisher = mgr
            .create_publisher(topic, MaxSliceLen::try_new(1024).expect("len"), 0)
            .expect("publisher");
        let mut sub = mgr.create_subscriber(topic).expect("subscriber");

        let sizing = ReadStageSizing {
            depth: DECLARED_DEPTH,
            annotated: ProducerAnnotation::Plain,
            sync_shape: SyncTriggerShape::Ordinary,
            burst: crate::graph::runtime::read_stage_burst_bound(
                ReadStageRole::Drain,
                "inp",
                &[],
                false,
                crate::read_outcome::FireBurstBound::Fires(1),
            ),
        };
        // The premise, asserted rather than assumed: the derived capacity is
        // genuinely SMALLER than the batch about to be drained. Without this the
        // test could pass on a stage that happened to be big enough.
        let derived = crate::read_outcome::derive_stage_capacity(ReadStageRole::Drain, sizing);
        assert!(
            (derived as usize) < PROVISIONED,
            "premise: the declared-depth capacity ({derived}) must be smaller than the \
             {PROVISIONED} frames queued, or this proves nothing"
        );

        let stage = Arc::new(ReadOutcomeStage::new(0, ReadStageRole::Drain, sizing));
        stage.arm();
        sub.set_read_outcome_stage(Arc::clone(&stage));

        for seq in 0..PROVISIONED as u32 {
            publisher.publish_raw(&frame(seq)).expect("publish");
        }
        let mut delivered = 0usize;
        sub.try_receive_for_drain(|_msg| delivered += 1)
            .expect("the drain succeeds");
        assert_eq!(
            delivered, PROVISIONED,
            "the whole queue drained in one pass"
        );

        let mut staged = Vec::new(); // hot-path-alloc-ok: test-only collector.
        stage.drain_into(|r| {
            staged.push(r);
            true
        });
        assert_eq!(
            staged,
            // hot-path-alloc-ok: test-only hand oracle.
            vec![StagedReadOutcome {
                kind: ReadOutcomeKind::DrainedBatch,
                served_seq: PROVISIONED as u64 - 1,
                popped: PROVISIONED as u32,
                token: None,
                role: ReadSiteRole::Drain,
                run_count: 1,
            }],
            "ONE record for the whole batch, with the frame count as `popped` — \
             which is why a queue provisioned deeper than the declaration cannot \
             overflow a stage sized from that declaration"
        );
        assert_eq!(
            stage.dropped(),
            0,
            "nothing was dropped: one record never approaches the rim"
        );
    }

    /// A mid-drain receive fault in `drain_samples`
    /// stages the PARTIAL batch BEFORE the Err propagates — the frames
    /// delivered before the fault really reached the callback, so omitting
    /// the record would make a faithful replay re-derive a batch the
    /// recording never wrote (a false read-log divergence at exactly the
    /// fault moment). Real iceoryx2 over a per-test isolated SHM root
    /// (`iceoryx_test_config` — parallel-safe); frames are hand-built wire
    /// bytes via `publish_raw` (seqs 0/1/2), the stage is wired + armed
    /// directly (the crate-internal seam the runtime uses).
    ///
    /// ARM: publish 3, inject the fault after 2 receives → `try_receive`
    /// propagates the Err AND the stage carries exactly
    /// `DrainedBatch(seq 1, popped 2)` (the 2 delivered frames; frame 2 was
    /// never popped). CONTROL: inject at 0 → Err with NOTHING staged (an
    /// all-or-nothing fault records no batch — every drain site's Err
    /// posture).
    ///
    /// What breaks this test: moving `drain_result?`
    /// above the partial-batch staging in `drain_samples` fails the ARM
    /// (Err returns before anything is staged → empty stage).
    #[test]
    fn drain_samples_stages_the_partial_batch_before_a_mid_drain_receive_fault() {
        use crate::read_outcome::{
            ReadOutcomeKind, ReadOutcomeStage, ReadStageRole, StagedReadOutcome,
        };
        use crate::transport::{TransportConfig, TransportManager};
        use crate::wire::MaxSliceLen;

        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "partial_batch".into(),
                clock: Arc::new(crate::clock::RealClock),
                subscriber_buffer_size: 8,
                network: None,
            },
            crate::testing::iceoryx_test_config(),
        )
        .expect("per-test SHM transport");

        // A hand-built wire frame: 32-byte header + 8 payload bytes, the
        // caller-stamped `sequence` the capture parses back.
        let frame = |seq: u32| -> Vec<u8> {
            let mut header = crate::wire::WireHeader::new(0xC1289, seq, 1_000 + u64::from(seq));
            let total = crate::wire::WireHeader::SIZE + 8;
            header.total_size = total as u32;
            let mut buf = vec![0u8; total]; // hot-path-alloc-ok: test-only frame builder.
            header.write_to_buf(&mut buf);
            buf
        };

        // ARM: 3 frames queued, fault after 2 receives.
        let topic = "partial_batch";
        let mut publisher = mgr
            .create_publisher(topic, MaxSliceLen::try_new(1024).expect("len"), 0)
            .expect("publisher");
        let mut sub = mgr.create_subscriber(topic).expect("subscriber");
        // The capacity is DERIVED per edge, so a hand-minted stage
        // declares the shape it stands in for — a single-publisher trigger
        // DRAIN at the default consumer depth.
        let stage = Arc::new(ReadOutcomeStage::new(
            0,
            ReadStageRole::Drain,
            hand_wired_drain_sizing(),
        ));
        stage.arm();
        sub.set_read_outcome_stage(Arc::clone(&stage));
        for seq in 0..3u32 {
            publisher.publish_raw(&frame(seq)).expect("publish");
        }
        sub.fault_inject_receive_after_for_test(2);
        let mut delivered = 0usize;
        let err = sub.try_receive(|_msg| delivered += 1);
        assert!(
            matches!(err, Err(TransportError::Receive { .. })),
            "the mid-drain fault must PROPAGATE: {err:?}"
        );
        assert_eq!(
            delivered, 2,
            "frames 0 and 1 were delivered before the fault"
        );
        let mut staged = Vec::new(); // hot-path-alloc-ok: test-only collector.
        stage.drain_into(|r| {
            staged.push(r);
            true
        });
        assert_eq!(
            staged,
            // hot-path-alloc-ok: test-only hand oracle.
            vec![StagedReadOutcome {
                kind: ReadOutcomeKind::DrainedBatch,
                served_seq: 1,
                popped: 2,
                token: None,
                // This test drives `drain_samples` through the
                // BODY entry point (`try_receive`), so the staged record
                // carries the BODY call-site role.
                role: ReadSiteRole::Body,
                run_count: 1,
            }],
            "the PARTIAL batch is staged before the Err propagates (newest \
             delivered seq 1, popped 2)"
        );

        // CONTROL: fault on the very first receive — nothing delivered,
        // nothing staged.
        let topic2 = "partial_batch_ctl";
        let mut publisher2 = mgr
            .create_publisher(topic2, MaxSliceLen::try_new(1024).expect("len"), 0)
            .expect("publisher");
        let mut sub2 = mgr.create_subscriber(topic2).expect("subscriber");
        let stage2 = Arc::new(ReadOutcomeStage::new(
            0,
            ReadStageRole::Drain,
            hand_wired_drain_sizing(),
        ));
        stage2.arm();
        sub2.set_read_outcome_stage(Arc::clone(&stage2));
        for seq in 0..3u32 {
            publisher2.publish_raw(&frame(seq)).expect("publish");
        }
        sub2.fault_inject_receive_after_for_test(0);
        let mut delivered2 = 0usize;
        let err2 = sub2.try_receive(|_msg| delivered2 += 1);
        assert!(
            matches!(err2, Err(TransportError::Receive { .. })),
            "the immediate fault must propagate: {err2:?}"
        );
        assert_eq!(delivered2, 0, "nothing was delivered");
        let mut staged2 = Vec::new(); // hot-path-alloc-ok: test-only collector.
        stage2.drain_into(|r| {
            staged2.push(r);
            true
        });
        assert!(
            staged2.is_empty(),
            "an all-or-nothing Err stages NO batch record: {staged2:?}"
        );
    }

    // The drop_oldest detector: `classify_gap` counts iceoryx2's
    // silent oldest-eviction from one publisher stream's wire-sequence
    // gap (the per-id baseline lookup in `try_view` pins both sequences
    // to one stream). The hard cases are (a) NOT mistaking our
    // own latest-wins read-skips for evictions, (b) u32 wraparound,
    // (c) huge gaps staying EXACT (no buffer-capped `Saturated`
    // floor), and (d) backward jumps (history replay) → never
    // counted. The real-iceoryx2 end-to-end proof (real `sample.origin()`
    // streams from real publishers) lives in
    // `tests/backpressure_event_iox2_test.rs`.

    #[test]
    fn classify_gap_contiguous_is_clean() {
        // last consumed 5, oldest surviving 6 → nothing evicted
        assert_eq!(classify_gap(5, 6), GapClass::Clean);
    }

    #[test]
    fn classify_gap_re_saw_last_is_clean() {
        assert_eq!(classify_gap(5, 5), GapClass::Clean);
    }

    #[test]
    fn classify_gap_single_gap() {
        // last 5, oldest surviving 7 → seq 6 was evicted
        assert_eq!(classify_gap(5, 7), GapClass::Exact(1));
    }

    #[test]
    fn classify_gap_multi_gap() {
        // last 5, oldest surviving 10 → 6,7,8,9 evicted
        assert_eq!(classify_gap(5, 10), GapClass::Exact(4));
    }

    #[test]
    fn classify_gap_huge_gap_is_exact_not_capped() {
        // The per-stream pinning makes a gap of many buffers'
        // worth the TRUE loss — there is no saturation cap. (The pre-epoch
        // detector returned `Saturated` here and under-reported sustained
        // overload.)
        assert_eq!(classify_gap(0, 100_001), GapClass::Exact(100_000));
    }

    #[test]
    fn classify_gap_handles_u32_wrap_contiguous() {
        // publisher wrapped MAX → 0 with no gap
        assert_eq!(classify_gap(u32::MAX, 0), GapClass::Clean);
    }

    #[test]
    fn classify_gap_handles_u32_wrap_with_eviction() {
        // seq …MAX-1, MAX, 0, 1; last consumed MAX-1, oldest surviving 1 →
        // MAX and 0 evicted (2)
        assert_eq!(classify_gap(u32::MAX - 1, 1), GapClass::Exact(2));
    }

    #[test]
    fn classify_gap_backward_is_backward_not_exact() {
        // Native history delivers retained frames with
        // their ORIGINAL stale sequences (by SHM offset,
        // same publisher port). Baseline 100, retained frame 3 drained after:
        // the wrapping gap is ~2^32 — a naive detector would return
        // Exact(4_294_967_198) and fabricate a catastrophic loss count the
        // moment a late joiner attached. It must classify as Backward.
        assert_eq!(classify_gap(100, 3), GapClass::Backward);
    }

    #[test]
    fn classify_gap_forward_gap_at_threshold_is_exact() {
        // Boundary pin: a wrapping gap of exactly BACKWARD_GAP_THRESHOLD is
        // still forward motion → Exact(threshold - 1).
        assert_eq!(
            classify_gap(0, BACKWARD_GAP_THRESHOLD),
            GapClass::Exact(BACKWARD_GAP_THRESHOLD - 1)
        );
    }

    #[test]
    fn classify_gap_just_over_threshold_is_backward() {
        // Boundary pin: one past the threshold flips to Backward — guards
        // against an off-by-one (>= vs >) silently reclassifying the
        // extreme-forward case.
        assert_eq!(
            classify_gap(0, BACKWARD_GAP_THRESHOLD + 1),
            GapClass::Backward
        );
    }

    #[test]
    fn raw_frame_validation_accepts_hand_written_wire_frame() {
        let mut frame = [0_u8; WireHeader::SIZE];
        frame[8..12].copy_from_slice(&(WireHeader::SIZE as u32).to_le_bytes());
        assert_eq!(
            validate_raw_frame("/raw", &frame).unwrap(),
            WireHeader::SIZE
        );
    }

    #[test]
    fn raw_frame_validation_rejects_undersized_and_out_of_bounds_frames() {
        assert!(matches!(
            validate_raw_frame("/raw", &[0; WireHeader::SIZE - 1]),
            Err(TransportError::Deserialization { .. })
        ));
        let mut frame = [0_u8; WireHeader::SIZE];
        frame[8..12].copy_from_slice(&((WireHeader::SIZE as u32) + 1).to_le_bytes());
        assert!(matches!(
            validate_raw_frame("/raw", &frame),
            Err(TransportError::Deserialization { .. })
        ));
    }

    fn raw_view_frame(sequence: u32) -> Vec<u8> {
        let mut frame = vec![
            0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, // schema hash
            0x22, 0x00, 0x00, 0x00, // total size
            0x20, 0x00, 0x00, 0x00, // offset table offset
            0x00, 0x00, 0x00, 0x00, // offset table count
            0x00, 0x00, 0x00, 0x00, // sequence
            0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // timestamp
            0xAA, 0xBB,
        ];
        frame[20..24].copy_from_slice(&sequence.to_le_bytes());
        frame
    }

    #[test]
    fn view_raw_reads_a_live_frame_with_hand_written_bytes() {
        use crate::transport::{TransportConfig, TransportManager};
        use crate::wire::MaxSliceLen;

        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "view_raw_live".into(),
                clock: Arc::new(crate::clock::RealClock),
                subscriber_buffer_size: 4,
                network: None,
            },
            crate::testing::iceoryx_test_config(),
        )
        .expect("transport");
        let topic = "view_raw/live";
        let mut publisher = mgr
            .create_publisher(topic, MaxSliceLen::try_new(128).expect("length"), 0)
            .expect("publisher");
        let mut subscriber = mgr.create_subscriber(topic).expect("subscriber");
        let expected = raw_view_frame(7);
        publisher.publish_raw(&expected).expect("publish");

        let view = subscriber.view_raw().expect("view").expect("sample");
        assert_eq!(&*view, expected.as_slice());
        let owned = match view.into_owned() {
            Ok(owned) => owned,
            Err(_) => panic!("live sample is owned"),
        };
        assert_eq!(&*owned, expected.as_slice());
    }

    #[test]
    fn view_raw_reoffers_a_held_frame_without_advancing_the_cursor() {
        use crate::transport::{TransportConfig, TransportManager};
        use crate::wire::MaxSliceLen;

        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "view_raw_held".into(),
                clock: Arc::new(crate::clock::RealClock),
                subscriber_buffer_size: 4,
                network: None,
            },
            crate::testing::iceoryx_test_config(),
        )
        .expect("transport");
        let topic = "view_raw/held";
        let mut publisher = mgr
            .create_publisher(topic, MaxSliceLen::try_new(128).expect("length"), 0)
            .expect("publisher");
        let mut subscriber = mgr.create_subscriber(topic).expect("subscriber");
        let expected = raw_view_frame(3);
        publisher.publish_raw(&expected).expect("publish");
        subscriber.snapshot_latest();

        let first = subscriber.view_raw().expect("first view").expect("sample");
        assert_eq!(&*first, expected.as_slice());
        let held = match first.into_owned() {
            Ok(_) => panic!("held sample must remain borrowed"),
            Err(held) => held,
        };
        assert_eq!(&*held, expected.as_slice());
        let second = subscriber.view_raw().expect("second view").expect("sample");
        assert_eq!(&*second, expected.as_slice());
    }

    #[test]
    fn view_raw_rejects_short_and_malformed_live_frames() {
        use crate::transport::{TransportConfig, TransportManager};
        use crate::wire::MaxSliceLen;

        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "view_raw_short".into(),
                clock: Arc::new(crate::clock::RealClock),
                subscriber_buffer_size: 4,
                network: None,
            },
            crate::testing::iceoryx_test_config(),
        )
        .expect("transport");
        let topic = "view_raw/short";
        let mut publisher = mgr
            .create_publisher(topic, MaxSliceLen::try_new(128).expect("length"), 0)
            .expect("publisher");
        let mut subscriber = mgr.create_subscriber(topic).expect("subscriber");
        publisher
            .publish_raw(&[0_u8; WireHeader::SIZE - 1])
            .expect("publish");

        assert!(matches!(
            subscriber.view_raw(),
            Err(TransportError::Deserialization { .. })
        ));
        let mut malformed = raw_view_frame(8);
        malformed[8..12].copy_from_slice(&100_u32.to_le_bytes());
        publisher
            .publish_raw(&malformed)
            .expect("publish malformed");
        assert!(matches!(
            subscriber.view_raw(),
            Err(TransportError::Deserialization { .. })
        ));
    }

    #[test]
    fn view_raw_then_try_view_raw_consumes_each_live_frame_once() {
        use crate::transport::{TransportConfig, TransportManager};
        use crate::wire::MaxSliceLen;

        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "view_raw_accounting".into(),
                clock: Arc::new(crate::clock::RealClock),
                subscriber_buffer_size: 4,
                network: None,
            },
            crate::testing::iceoryx_test_config(),
        )
        .expect("transport");
        let topic = "view_raw/accounting";
        let mut publisher = mgr
            .create_publisher(topic, MaxSliceLen::try_new(128).expect("length"), 0)
            .expect("publisher");
        let mut subscriber = mgr.create_subscriber(topic).expect("subscriber");
        let first = raw_view_frame(1);
        let second = raw_view_frame(2);
        publisher.publish_raw(&first).expect("publish first");
        assert_eq!(
            subscriber.view_raw().expect("first view").as_deref(),
            Some(first.as_slice())
        );
        assert!(subscriber
            .try_view_raw(|_| ())
            .expect("empty view")
            .is_none());
        publisher.publish_raw(&second).expect("publish second");
        assert_eq!(
            subscriber
                .try_view_raw(|bytes| bytes.to_vec())
                .expect("second view"),
            Some(second)
        );
    }
}
