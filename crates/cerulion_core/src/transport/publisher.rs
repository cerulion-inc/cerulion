// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-copy message publisher over iceoryx2 shared memory.
//!
//! # Publish flow
//!
//! `loan_proxy::<T>()` loans a fixed-size SHM slot, stamps the WireHeader,
//! and returns an `OutputProxy<T>` derefing to `T::Writer<'_>`. The proxy
//! writes fields directly into the loaned buffer; on drop it finalises
//! the header, calls `send()`, and notifies subscribers with `SentSample`.

use std::mem::MaybeUninit;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;

use iceoryx2::port::listener::Listener;
use iceoryx2::port::notifier::Notifier;
use iceoryx2::port::publisher::Publisher;
// The `PortFactory` TRAIT (not the `event::PortFactory`
// struct) supplies `.dynamic_config()` on the retained event-service factory
// so the notify-elision gate can read the live `number_of_listeners()`.
use iceoryx2::service::port_factory::PortFactory as _;
// `Publisher::update_connections` (native history delivery
// to a freshly-connected late joiner) is a method of this trait — bring it
// into scope so `deliver_history` can call it.
use iceoryx2::port::update_connections::UpdateConnections;

use crate::clock::Clock;
use crate::error::{TransportError, TransportResult};
use crate::message::ShmMessage;
use crate::trace::{PublishTrace, PublishTraceEntry};
use crate::wire::{MaxSliceLen, WireHeader};

use super::adaptive_sizer::AdaptiveSizer;
use super::events::PubSubEvent;
use super::notify_delivery_latch::{
    ListenerCountTiming, NotifyDeliveryAction, NotifyDeliveryLatch,
};
use super::output_discard_latch::{DiscardLogLevel, OutputDiscardLatch};
use super::output_proxy::OutputProxy;
use super::shm_sample::{ProxyPublisher, SampleHandle};
use super::CerService;
use std::sync::Mutex;

#[cfg(any(test, feature = "test-helpers"))]
thread_local! {
    /// Test-only: when set, the NEXT `deliver_history` treats its
    /// `update_connections` as failed (suppressing the SentHistory wake), so the
    /// Err-path gate is testable without a real transport failure. Auto-clears.
    static FORCE_NEXT_UPDATE_CONNECTIONS_FAIL: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// How ONE notify's `listeners` count reaches
/// [`CerulionPublisher::record_notify_delivery`].
///
/// The seam takes this instead of an `Option<usize>` so that a CALLER-SUPPLIED
/// count carries the [`ListenerCountTiming`] whoever read it DECLARED, never
/// one INFERRED from the presence of a count. That inference is precisely the silent
/// inversion `ListenerCountTiming` exists to prevent: a caller that read its
/// count AFTER notifying (a future site, or a refactor that moves the elision
/// gate's read below the notify to shave the gate cost) would have handed over
/// a `Some(n)` and been labelled `BeforeNotify`, re-opening the false-positive
/// `warn!` + permanent counter bump on every attach race — with no compile
/// error. With this type such a caller writes `AfterNotify` and gets
/// `AfterNotify`.
enum NotifyListenerCount {
    /// The CALLER already read the count and states which side of ITS notify
    /// that read landed on. Always classified — the cost gate can save nothing
    /// on a read that is already paid for.
    CallerRead(usize, ListenerCountTiming),
    /// No caller count: `record_notify_delivery` reads it itself, under the
    /// cost gate, and stamps that read [`ListenerCountTiming::AfterNotify`] —
    /// the one timing this seam ASSIGNS rather than receives. It rests on a
    /// convention, not on the type system: every caller invokes the seam only
    /// once its own notify has returned, so a read taken inside the seam is
    /// after that notify. A future site calling `record_notify_delivery(t,
    /// ReadHere)` BEFORE notifying would be mislabelled with no compile error —
    /// but that direction fails SAFE (a real shortfall is delayed by one
    /// classified notify; a false one is never counted), which is the whole
    /// reason a no-count arm is allowed to exist. Any site whose ordering is
    /// not that must hand a `CallerRead(n, timing)`.
    ReadHere,
}

/// Calls to [`CerulionPublisher::check_subscriber_events`] that keep draining
/// the publisher's own event listener after the topic's live listener count
/// moves.
///
/// Exactly one would race. The count rises when a late joiner's `Listener` is
/// created, and its `SubscriberConnected` notify lands after that, so a single
/// gated drain can fall in the window between the two, see nothing, and never
/// look again. Staying armed across a run of calls closes that window: on the
/// quiescent-publisher path the runtime's `pump_history` cadence spends this
/// budget over as many steps, and on a publishing path a drain that observes a
/// transition clears the arming immediately.
///
/// The bound is what stops the opposite failure. A listener that attaches or
/// dies WITHOUT ever sending a transition — a foreign observer, a consumer that
/// crashed — would otherwise arm every publish for the life of the process.
/// While armed the cost is what every publish paid before this gate existed, so
/// the budget is set well past any plausible create-then-notify gap rather than
/// trimmed: it is spent only when the listener count actually moves.
const SELF_DRAIN_ARMED_CALLS: u8 = 64;

/// Zero-copy publisher for a single topic.
///
/// Holds an iceoryx2 data publisher, an event notifier (to signal subscribers),
/// and an event listener (to hear from subscribers). Late-joiner history is
/// NATIVE: the iceoryx2 SERVICE is created with
/// `history_size(N)` (at the creator/precreate site in `mod.rs`), so the
/// publisher port iceoryx2 hands back already owns a history queue of size N
/// that retains the last N sent SHM frames by offset (zero-copy) and
/// auto-delivers them to a late subscriber on `update_connections()`. There is
/// no Cerulion-side history buffer and no per-frame copy on the publish hot
/// path — `deliver_history` just drives iceoryx2's native delivery + wakes the
/// late joiner with a `SentHistory` event.
///
/// # Thread Safety
///
/// `publish()` takes `&mut self` — the publisher is designed for single-threaded use
/// within a node. The `AtomicU32` sequence counter remains atomic for observable reads.
pub struct CerulionPublisher {
    /// Topic name as `Arc<str>`. It was once a `String`;
    /// `Arc<str>` lets `loan_proxy` hand the writer
    /// an owned-but-cheaply-cloned topic reference WITHOUT the
    /// raw-pointer-reborrow `unsafe` block that the borrowed variant
    /// required. Allocation happens once at publisher construction;
    /// every loan does an atomic refcount bump.
    ///
    /// Error sites construct `String` via `self.topic.to_string()` —
    /// same allocation behavior as the prior `self.topic.clone()` on
    /// `String`.
    topic: Arc<str>,
    publisher: Publisher<CerService, [u8], ()>,
    notifier: Notifier<CerService>,
    listener: Listener<CerService>,
    /// Live listener count on this topic's event service as of the last
    /// [`Self::check_subscriber_events`] — the cheap gate on the per-publish
    /// self drain. `usize::MAX` until the first call, so the first publish
    /// always drains.
    last_listener_count: usize,
    /// Armed self drains still owed. A change in `last_listener_count` sets it
    /// to [`SELF_DRAIN_ARMED_CALLS`]; a drain that observes a subscriber
    /// transition clears it early. While zero, a publish costs one relaxed load
    /// instead of a listener drain.
    self_drains_armed: u8,
    sequence: AtomicU32,
    /// The value `sequence` was CONSTRUCTED with — 0 on every
    /// live path, and the recorded stream's next sequence on a restored replay.
    ///
    /// Retained because `sequence` alone stops meaning "frames committed" the
    /// moment it starts somewhere other than 0, and the producer reconciliation
    /// reads exactly that quantity. Keeping the origin lets
    /// [`Self::committed_frames`] answer it without the seed, while
    /// [`Self::sequence`] keeps its unchanged meaning — both observable,
    /// neither conflated (Principle #3).
    initial_sequence: u32,
    clock: Arc<dyn Clock>,
    /// Maximum buffer size for this publisher (header + payload).
    /// `MaxSliceLen` — encodes the wire-format ceiling
    /// AND the floor `>= WireHeader::SIZE` at construction.
    max_slice_len: MaxSliceLen,
    /// Native iceoryx2 history depth this
    /// publisher REQUESTED (`.history_size(N)` at the creator/precreate site
    /// in `mod.rs`). 0 = no late-joiner history requested (VOLATILE). Backs
    /// [`Self::has_history`], which the rmw bridge reads to distinguish
    /// TRANSIENT_LOCAL publishers from VOLATILE ones.
    ///
    /// This is the ASK. What the publisher actually got is
    /// [`Self::provisioned_history_size`] — on an OPEN of a deeper existing
    /// service the two differ, and the difference is observable to a late
    /// joiner.
    history_size: usize,
    /// The history depth the opened SERVICE really provides — see
    /// [`CerulionPublisherConfig::provisioned_history_size`] for why this is
    /// not always [`Self::history_size`] and how it is measured.
    provisioned_history_size: usize,
    /// Adaptive loan sizing: sliding-window estimator over recent
    /// publish payload sizes. The publisher loans
    /// `sizer.next_loan_size(min_required, max_slice_len)` instead of
    /// always `max_slice_len`. Updated via `record_payload_size` from
    /// `OutputProxy::Drop` on successful publish. See
    /// `adaptive_sizer.rs` for the convergence + determinism
    /// guarantees.
    sizer: AdaptiveSizer,
    /// Regime A: optional metadata-only publish trace.
    /// When attached, every successful publish (via `publish_raw`)
    /// pushes a 32-byte `PublishTraceEntry` parsed from the
    /// `WireHeader`. Default `None` — no overhead on the hot path
    /// when tracing is disabled.
    trace: Option<Arc<Mutex<PublishTrace>>>,
    /// Testing-only: **fire-once**
    /// fault-injection for `publish_raw`. When `Some(n)`, the next
    /// n calls succeed and the (n+1)th call returns
    /// `TransportError::LoanCapacity`. After firing the field is
    /// cleared — subsequent calls succeed.
    /// Used to exercise the `deliver_history` truncation path on
    /// the iceoryx2 backend without requiring real iceoryx2 pool
    /// exhaustion (which is hard to force deterministically).
    ///
    /// This field is deliberately not `#[cfg(...)]`-gated:
    /// gating a struct field on a feature differs the struct's layout
    /// between callers with the feature enabled (host integration tests
    /// via `cerulion_core/.../dev-dependencies` self-ref) and callers
    /// without (cdylibs). Passing the publisher across the FFI
    /// boundary would then SIGSEGV in the cdylib's Drop chain. Keeping
    /// the field unconditional (always `Option::None` in production)
    /// costs 8 bytes per CerulionPublisher (4-byte tag + 4-byte u32
    /// payload; `Option<u32>` is NOT niche-optimized) and removes the
    /// layout mismatch. Setter stays test-gated.
    fault_inject_publish_raw_after: Option<u32>,
    /// **fire-once** fault-injection for `send_overflow_frame`. Mirror of
    /// `fault_inject_publish_raw_after` but on the overflow re-loan
    /// path. Lets tests deterministically exercise the Drop's
    /// "overflow re-loan failed → frame DROPPED" branch without
    /// requiring real iceoryx2 pool exhaustion.
    fault_inject_send_overflow_frame_after: Option<u32>,
    /// Fire-once fault-injection for
    /// `send_raw_loan` — the flatten-into-loan send path (rmw_publish's
    /// VOLATILE branch) had no hook, so its send-failure → caller-error
    /// branch was untested. When `true`, the next `send_raw_loan` drops
    /// the loan (releasing the pool slot) and returns
    /// `TransportError::Publish` instead of `sample.send()`. Production:
    /// a single bool, one branch, always `false`. Setter is test-gated.
    fault_inject_send_raw_loan: bool,
    /// "Loud failure" signal:
    /// counter of frames LOST during `OutputProxy::Drop`'s overflow
    /// re-loan path (incremented when `send_overflow_frame` returns Err).
    /// Loud-failure complement to the `tracing::error!` in Drop —
    /// metrics-attachable signal vs grep-only log line.
    frames_dropped_overflow: std::sync::atomic::AtomicU64,
    /// Counter of frames LOST due to invariant violations — distinct
    /// failure mode from `frames_dropped_overflow`.
    /// Bumped from documented-unreachable Drop branches:
    /// - Contract violation: `T::has_overflow == true` but
    ///   `T::overflow_view_bytes == None` (hand-written `impl ShmMessage`
    ///   bug).
    /// - Small buffer at overflow drop: `original_bytes.len() <
    ///   WireHeader::SIZE` (would indicate `loan_proxy` failed to stamp
    ///   the 32-byte header).
    /// - Small buffer at steady-state drop: loaned `buf.len() < 12`
    ///   (would indicate a publisher returned an undersized sample).
    ///
    /// All three are defense-in-depth: production never reaches them,
    /// but a future refactor breaking the relevant invariant would
    /// drop frames silently. This counter is the operator's canary
    /// for "something corrupted is happening in Drop".
    frames_dropped_invariant_violation: std::sync::atomic::AtomicU64,
    /// Count of frames LOST at the STEADY-STATE commit-then-fail arms
    /// of `OutputProxy::Drop` — the wire `sequence` was already CONSUMED
    /// (`commit_sequence`, burning the number) but the frame reached NO
    /// subscriber queue because either `take_outbound()` returned `None` (the
    /// outbound iceoryx2 sample was missing at drop) or `sample.send()` errored.
    ///
    /// This was the ONE counter-less silent-loss hole in the commit
    /// path: unlike the overflow arm (which bumps
    /// [`Self::frames_dropped_overflow`]) and the invariant arms (which bump
    /// [`Self::frames_dropped_invariant_violation`]), the steady-state fail arms
    /// were pure `tracing::error!` + `return` — a committed frame vanished with
    /// ZERO metric and ZERO bag-side signal (the recorder's gap-detector evasion
    /// is downstream of this, but this counter is the send-side half of the
    /// reconciliation identity). `> 0` here explains a bagd gap that
    /// `record_health.json` reports as `frames_lost = 0`: the frame died on the
    /// PRODUCER side before it ever reached bagd's tap, so bagd's forward-only
    /// gap detector never saw a surviving newer seq to reveal the hole (the
    /// failed frame was the topic's highest committed seq). When the failed
    /// frame is NOT the tail (a mid-stream drop under the storm — a later seq
    /// still succeeds), bagd DOES see the hole and counts it as `frames_lost`,
    /// so that frame is counted on BOTH sides: this term and bagd's
    /// `frames_lost` OVERLAP for a mid-stream drop, which is why the
    /// reconciliation identity is an upper bound (`<=`), exact only for
    /// tail-only drops (see [`crate::graph::node::PublisherReconStat`]).
    ///
    /// **Operator action:** a nonzero value with a matching bagd gap ⇒ the loss
    /// is send-side (iceoryx2 `send()` failure / missing outbound sample under
    /// SHM or wakeup-socket pressure), not receive-side reclaim — investigate
    /// the publisher, not the recorder tap.
    frames_dropped_send_fail: std::sync::atomic::AtomicU64,
    /// `block`: one shared outstanding counter per
    /// `block`-policy consumer of this publisher's topic. Each is the
    /// deterministic mirror of that consumer's iceoryx2 queue depth —
    /// incremented here on every successful publish (the frame enters
    /// every subscriber's queue), decremented on the consumer's
    /// [`CerulionSubscriber`] drain. The producer's scheduler pre-fire
    /// check reads these to defer the moment any consumer's queue is
    /// full. Empty unless the graph runtime wired this publisher's topic
    /// as all-`block` (mixed topics degrade to native drop_oldest and
    /// install no counter). The frame is never buffered or copied here —
    /// this is a plain atomic mirror, not a data path.
    block_outstanding: Vec<crate::credit::CreditWord>,
    /// Shared `last_publish_ns` anchor for this output's
    /// `#[output(promise_within_ms = N)]` watchdog. `Some` only when the
    /// graph runtime wired a watchdog on this output; the same
    /// `Arc<AtomicU64>` is held by the scheduler's `output_promise_within`
    /// tracker. Every successful send writes `clock.now_ns()` here so the
    /// scheduler's next `step()` sees a fresh publish and does not
    /// false-count a miss. Deterministic — `clock` is the same source the
    /// scheduler reads. Field is unconditional (FFI struct-layout
    /// stability — never `#[cfg]`-gate a field; the SIGSEGV
    /// lesson); `None` in production for un-watched outputs.
    promise_within_last_publish_ns: Option<Arc<std::sync::atomic::AtomicU64>>,
    /// Producer-owned SHM doorbell for this publisher's topic. `Some`
    /// only after `enable_doorbell` runs at graph build under an active doorbell
    /// policy; `None` in production otherwise, so the publish path
    /// pays nothing. When `Some`, every successful send rings it (a `Release`
    /// store) so a consumer parked in a shallow CPU monitor-wait
    /// ([`crate::monitor_wait`]) on the same line wakes the instant data is
    /// published. FIREWALL: a WAKE SIGNAL ONLY — the data is already in the
    /// iceoryx2 SHM queue by the time the ring happens and is still read there
    /// by the consumer's `step()`; ringing changes only WHEN the consumer wakes,
    /// never WHAT it reads/fires. Field is unconditional (FFI struct-layout
    /// stability — never `#[cfg]`-gate a field; the SIGSEGV lesson).
    doorbell: Option<crate::doorbell::Doorbell>,
    /// Notify elision: the topic's EVENT-service factory,
    /// retained so the publish hot path can read the LIVE
    /// `dynamic_config().number_of_listeners()` for the self-healing elision
    /// gate. The `notifier`/`listener` above were minted from this same
    /// service; holding the factory adds only a cheap shared-service-state
    /// refcount (no extra SHM). It is the ONLY handle exposing the live
    /// listener count — the `Notifier` port does not. Unconditional (FFI
    /// struct-layout stability — never `#[cfg]`-gate a field; the
    /// SIGSEGV lesson).
    event_service: iceoryx2::service::port_factory::event::PortFactory<CerService>,
    /// The EXPECTED in-process listener count on this topic's
    /// event service — the listeners the graph runtime PROVED it owns (this
    /// publisher's own listener + every in-graph body/trigger/sync listener on
    /// the topic). `Some` only when the runtime armed elision (an active
    /// `CERULION_NOTIFY_ELISION` on a graph-owned topic); `None` for every
    /// non-graph publisher (service layer, rmw, raw `create_publisher`) AND
    /// when the kill-switch is off — those NEVER elide (byte-identical to the
    /// always-notify behavior). The `Arc<AtomicUsize>` is SHARED
    /// with the runtime's build loop, which `fetch_add`s it at each listener
    /// create; by the time the graph runs (post-build) it holds the final
    /// count, so the notify path reads a stable value. See
    /// [`Self::notify_sent_sample`]. UNDER-counting is safe (loses the win,
    /// never elides while a listener needs the wake); the runtime NEVER
    /// over-counts (it increments only at literal listener-create sites).
    notify_elision_expected: Option<Arc<std::sync::atomic::AtomicUsize>>,
    /// Count of SentSample notifies ELIDED by the gate — the
    /// engagement observable (proves the win is real).
    /// SHARED `Arc` so the runtime exposes it to tests
    /// (`notify_elided_count_for_test`); `None` when elision is not armed.
    notify_elided_count: Option<Arc<std::sync::atomic::AtomicU64>>,
    /// Last gate decision (`true` = elided), so the transition
    /// breadcrumb fires only when the gate FLIPS (a foreign listener attaches /
    /// detaches), never per publish (hot path — no alloc, no log). Relaxed —
    /// diagnostics only, never read by the scheduler (determinism firewall).
    notify_gate_last_elided: AtomicBool,
    /// When `true`, [`Self::publish_raw`] fires the `SentSample` wake
    /// itself (through [`Self::notify_sent_sample`], so the elision gate
    /// still applies). Set ONLY on a publisher minted by
    /// [`crate::transport::TransportManager::create_ingress_publisher`] — the
    /// raw-frame INGRESS path (the DDS-bridge `RawIngressRoute`, the network
    /// re-inject `IngressInjector`), whose consumers are FOREIGN/cross-process
    /// and MUST be woken. Folding the wake INTO `publish_raw` makes the notify
    /// STRUCTURAL for every ingress caller (no "remember to notify after
    /// publish_raw" convention, which is the convention that CAUSED the
    /// robot-local `topic hz` starvation on raw-routed topics). `false` for
    /// every non-ingress publisher (graph outputs, service, rmw), which either
    /// notify through `OutputProxy::Drop` or call `notify_sent_sample` on their
    /// own schedule — so this never double-notifies them.
    notify_on_publish_raw: bool,
    /// **This publisher holds a COMMITTED frame no listener has been
    /// told about.** Set by the elided arm of
    /// [`Self::notify_sent_sample`] and by [`Self::publish_raw`] on a publisher
    /// that is not `notify_on_publish_raw`-armed. CLEARED by a `SentSample`
    /// notify (the per-publish path or the boundary
    /// [`Self::resweep_notify_elision`]), because a notify wakes every listener
    /// on the topic's event service and therefore announces everything committed
    /// before it.
    ///
    /// Two scope statements, both stated rather than implied:
    ///
    /// * **NOT every un-announcing publish path sets it.**
    ///   [`Self::send_raw_loan`] commits a frame and rings nothing, and does not
    ///   record the debt. That is INERT today — its only callers are rmw
    ///   publishers, which `arm_notify_elision` never touches (its sole caller
    ///   is the graph runtime's `build_with_scheduler`), and it is not on
    ///   `AnyPublisher` — so the resweep returns on its first line for every
    ///   publisher that can reach it. Named here rather than fixed by adding a
    ///   store no reachable path can observe; the day an elision-armed publisher
    ///   gains that call, the store belongs there too.
    /// * **CLEARED only by a `SentSample`.** `deliver_history`'s `SentHistory`
    ///   notify is a real notify that leaves the debt standing. The direction is
    ///   safe: at worst the next boundary announces a frame the history wake
    ///   already covered — ONE extra announcement, never a missed one — and the
    ///   two events mean different things (a late joiner's backlog vs a live
    ///   commit), so collapsing them would make the debt lie about which.
    ///
    /// It is the boundary resweep's whole TRIGGER, and that is what prevents the
    /// boundary-notify storm described next.
    /// A `notified_since_boundary` flag — set on a real notify, consumed
    /// by the resweep as a one-window SKIP — would make the resweep's condition
    /// "a silent pass observed a foreign listener" rather than "an un-announced
    /// frame exists". The runtime runs the boundary on EVERY `live_step`
    /// iteration (via `pump_history_all`, not a 250 ms timer), so an armed
    /// publisher with a foreign listener attached would fire a real `SentSample`
    /// notify on every pass it did not publish: MEASURED 198 notifies for 2
    /// frames, and — because that notify also reaches the topic's OWN in-graph
    /// consumer listener on the live WaitSet — the graph's live loop would wake itself
    /// and FREE-RUN (MEASURED 9/s → 136/s with a live producer, 47/s → 1568/s silent).
    /// Keyed on un-announced state instead, a silent pass fires NOTHING and the
    /// steady-state bill is one notify per publish.
    ///
    /// **The late-attach heal is per un-announced FRAME, not per LISTENER — a
    /// real narrowing, measured.** The debt is spent by the FIRST announcing
    /// boundary, so a SECOND wake source attaching after that boundary is never
    /// resweep-woken for the frame its own tap already holds (a resweep that
    /// fired on every silent pass would happen to catch that listener
    /// too). Consequences, stated at their real size: the frame is NOT lost — it
    /// is in the second tap's SHM queue and a vizd-class consumer drains it on
    /// its timeout fallback, i.e. within one poll interval; the flagship
    /// single-viewer path is unaffected, because that viewer attaches before its
    /// own first announcing boundary; and the extra latency is bounded by the
    /// consumer's own fallback, never unbounded. Pinned by
    /// `notify_elision_resweep_iox2_test::a_second_wake_attaching_after_the_announcement_rides_the_timeout_not_the_boundary`,
    /// so it is a stated contract rather than an accident.
    ///
    /// # Ordering
    ///
    /// `Relaxed` on both halves of the contract:
    ///
    /// * DETERMINISM (Principle #7) — diagnostics-adjacent, never read by the
    ///   scheduler; it changes only WHEN a consumer wakes, never what fires.
    /// * NO LOST WAKE (Principle #6) — the store is published by the per-node
    ///   `Mutex` the runtime already holds, not by this atomic's ordering. Every
    ///   publish runs inside a node tick under that lock, and the boundary
    ///   (`pump_history_all` → `NodeContext::pump_history`) takes the SAME lock
    ///   on the same thread that joined the level's rayon fires, so the
    ///   lock's release→acquire edge happens-before the boundary's read. A debt
    ///   recorded by a publish can therefore never be invisible to the boundary
    ///   that follows it.
    ///
    /// Alloc-free (a bare `AtomicBool`).
    unannounced_publish: AtomicBool,
    /// Count of notifies FIRED by the boundary resweep
    /// (NOT the skips). SHARED `Arc` so the runtime exposes it to tests
    /// (`notify_resweep_count_for_test`) — a steady producer + foreign listener
    /// must show ~0 boundary notifies (the per-publish path did the waking).
    /// **The debt-keyed trigger pins the other half**: a producer holding an UN-ANNOUNCED
    /// frame shows ONE, not one per boundary pass — the count is per debt, so
    /// this counter is also the storm oracle (`N` silent passes ⇒ still 1).
    /// `None` when elision is not armed (armed together with the elided counter).
    resweep_notify_count: Option<Arc<std::sync::atomic::AtomicU64>>,
    /// Flood-suppression latch + log-level-independent counter for notifies
    /// that could NOT be delivered to every live listener (a consumer whose
    /// process died without deregistering, so its doorbell has no reader while
    /// its registration stands). Always armed — see [`NotifyDeliveryLatch`] for
    /// why this signal must exist on OUR side once `IOX2_LOG_LEVEL=error`
    /// correctly filters iceoryx2's own complaint, and for the condition the
    /// latch was built for that iceoryx2 0.10 made unreachable. Read via
    /// [`Self::notify_undelivered_count`].
    notify_delivery_latch: NotifyDeliveryLatch,
    /// SHARED mirror of [`notify_delivery_latch`](Self::notify_delivery_latch)'s
    /// `total_undelivered`, so the count is observable EXTERNALLY
    /// (Principle #3) — the exact `output_discard_shared` shape.
    ///
    /// The latch lives inline on this per-port publisher and is only reachable
    /// through `CerulionPublisher` / `AnyPublisher`, i.e. from the node's OWN
    /// tick code. An off-thread operator holds a `NodeHandle`, which cannot
    /// reach the publisher. The graph runtime mints one `Arc<AtomicU64>` per
    /// output, installs it here (via
    /// [`register_notify_undelivered_count`](Self::register_notify_undelivered_count))
    /// and registers the SAME `Arc` into the node's `NodeHandle` per-output map,
    /// so `NodeHandle::notify_undelivered_count(output)` reads the very atomic
    /// this publisher writes. `None` for every non-graph publisher (they never
    /// call the setter), so the wiring is graph-only and the raw / service / rmw
    /// publishers pay nothing.
    notify_undelivered_shared: Option<Arc<std::sync::atomic::AtomicU64>>,
    /// RECORD side: this publisher's node's shared publish-DISCARD
    /// signal. `Some` after `GraphRuntime` calls [`Self::set_discard_signal`]
    /// with the Arc it also handed to the node's `ScheduledNode`. Bumped on every
    /// pre-commit discard — the all-defer path in `OutputProxy::Drop`
    /// (via the proxy's `ProxyPublisher`) AND `loan_proxy`'s loan-FAILURE path
    /// (which never constructs a proxy, so the counter is the ONLY discard signal
    /// for a single-output node whose sole loan fails under pressure).
    /// `None` for every non-graph publisher (service layer, rmw, raw
    /// `create_publisher`) — they never mark. Field is unconditional (FFI
    /// struct-layout stability — never `#[cfg]`-gate a field; the SIGSEGV
    /// lesson). Bumps are atomic `fetch_add` (no alloc on the hot path).
    discard_signal: Option<Arc<std::sync::atomic::AtomicU32>>,
    /// REPLAY side: this publisher's node's shared publish-SUPPRESS
    /// flag. `Some` after `GraphRuntime` calls [`Self::set_replay_suppress`].
    /// During replay the scheduler sets it TRUE for a marked fire's callback
    /// duration; `OutputProxy::Drop` reads it FIRST (before `commit_sequence`) and
    /// returns without publishing — the byte-identical mirror of the live
    /// discard, so no phantom sequence is burned. `None` (never suppress) for
    /// every non-graph publisher and on the live/record path. Unconditional field
    /// (FFI struct-layout stability). Atomic `load` only (no alloc).
    replay_suppress: Option<Arc<std::sync::atomic::AtomicBool>>,
    /// TEST SEAM: when `true`, the next `loan_proxy` call FAILS its loan
    /// (as under real SHM pressure) — bumping `discard_signal` and returning
    /// `Err(LoanCapacity)` — then clears itself. Lets a test force the pre-commit
    /// discard path deterministically (SHM pressure is nondeterministic). Always
    /// `false` in production; setter is test-gated. Unconditional field (FFI
    /// struct-layout stability — same rationale as `fault_inject_publish_raw_after`).
    fault_inject_loan_fail_next: bool,
    /// Per-port flood-suppression latch for `OutputProxy::Drop`'s
    /// loud discard-error path (an incomplete-output tick — a missing declared
    /// variable field, or a failed staged nested-field flush — skips publish).
    /// FIRST discard of a regime logs at `error!` (the loud-by-design
    /// first signal), SUSTAINED discards downgrade to `debug!` with a running
    /// suppressed count, and a subsequent COMPLETE publish reports recovery at
    /// `info!` once + re-arms. Owned inline (not `Option`) — the publisher is
    /// per-(node, output port), so the latch is naturally per-port and the
    /// happy-path `on_complete` check is a single branch (no lock, no alloc,
    /// no map lookup — hot-path discipline). See
    /// [`crate::transport::output_discard_latch`]. Unconditional field (FFI
    /// struct-layout stability — never `#[cfg]`-gate a field; the
    /// SIGSEGV lesson).
    discard_latch: OutputDiscardLatch,
    /// SHARED mirror of [`discard_latch`](Self::discard_latch)'s
    /// unconditional `total_discards`, so the count is observable EXTERNALLY
    /// (Principle #3). The `discard_latch` lives inline on this per-port
    /// publisher and is only reachable through `CerulionPublisher` /
    /// `AnyPublisher` — i.e. by the node's own tick code. An off-thread OPERATOR
    /// reads a `NodeHandle`, which cannot reach the publisher. The graph runtime
    /// mints one `Arc<AtomicU64>` per output, installs it here (via
    /// [`register_output_discard_count`](Self::register_output_discard_count)),
    /// and registers the SAME `Arc` into the node's `NodeHandle` per-output
    /// discard map — exactly the promise_within anchor-sharing shape. Every
    /// `record_output_discard` stores the latch total into it. `None` for every
    /// non-graph publisher (they never call the setter), so the observability
    /// wiring is graph-only and the raw / service / rmw publishers pay nothing.
    output_discard_shared: Option<Arc<std::sync::atomic::AtomicU64>>,
}

/// Configuration for creating a `CerulionPublisher`.
///
/// Bundles all parameters needed by `CerulionPublisher::new()`.
pub(crate) struct CerulionPublisherConfig {
    pub topic: Arc<str>,
    pub publisher: Publisher<CerService, [u8], ()>,
    pub notifier: Notifier<CerService>,
    pub listener: Listener<CerService>,
    pub clock: Arc<dyn Clock>,
    pub max_slice_len: MaxSliceLen,
    /// Native iceoryx2 history depth this publisher REQUESTED
    /// (0 = VOLATILE / no late-joiner history). Mirrored onto the
    /// publisher to back [`CerulionPublisher::has_history`].
    pub history_size: usize,
    /// The history depth the OPENED SERVICE actually provides — which is
    /// not always [`Self::history_size`], and the difference is observable.
    ///
    /// iceoryx2's open-time verification on `.history_size(N)` is AT-LEAST
    /// (existing < required fails), so a request of 5 attaches to a service
    /// already created at 16 — and the port iceoryx2 hands back is sized
    /// from the SERVICE's static config, not from the request
    /// (`iceoryx2-0.9.1/src/port/publisher.rs`:
    /// `Queue::new(static_config.history_size)`). Such a publisher really
    /// retains 16, MEASURED by a late joiner in
    /// `rmw_transient_local_ceiling_test.rs`.
    ///
    /// Read back from `static_config()` at construction, so it is right for
    /// a CREATE (where it equals the request) and for an OPEN (where it may
    /// exceed it) alike.
    pub provisioned_history_size: usize,
    /// The topic's EVENT-service factory (the same one the
    /// `notifier`/`listener` were minted from), retained on the publisher so
    /// the notify-elision gate can read the live `number_of_listeners()`.
    pub event_service: iceoryx2::service::port_factory::event::PortFactory<CerService>,
    /// The wire `sequence` this publisher's FIRST frame
    /// carries. **0 on every live path** — the only caller that sets anything
    /// else is a RESTORED replay, which must continue the recorded stream's
    /// numbering rather than restart it.
    ///
    /// It is a CONSTRUCTION parameter, not a setter, and that is the whole
    /// safety argument: a live counter can never be moved, because there is no
    /// path that moves one. (A setter would also be unreachable where it
    /// matters — a cdylib node owns its `NodeContext` from `init()` onward, so
    /// the host cannot reach its publishers at all; the reconciliation work hit that wall and
    /// solved it by logging from inside the cdylib rather than by adding an
    /// FFI export.) Seeding at creation reaches every entry kind — in-process,
    /// macro and cdylib alike — with no FFI and no trait surface.
    pub initial_sequence: u32,
}

impl CerulionPublisher {
    /// Create a new publisher.
    ///
    /// Called by `TransportManager::create_publisher()`. Not intended for direct use.
    pub(crate) fn new(config: CerulionPublisherConfig) -> Self {
        // No `debug_assert!(max_slice_len <= u32::MAX
        // as usize, …)` is needed here as defense-in-depth. With
        // `max_slice_len: u32` the invariant is a type-system
        // property — the wire format's `WireHeader::total_size: u32`
        // cannot be exceeded by construction.
        Self {
            topic: config.topic,
            publisher: config.publisher,
            notifier: config.notifier,
            listener: config.listener,
            // `usize::MAX` can never equal a real listener count, so the first
            // publish always drains and establishes the baseline.
            last_listener_count: usize::MAX,
            self_drains_armed: 0,
            sequence: AtomicU32::new(config.initial_sequence),
            initial_sequence: config.initial_sequence,
            clock: config.clock,
            max_slice_len: config.max_slice_len,
            history_size: config.history_size,
            provisioned_history_size: config.provisioned_history_size,
            sizer: AdaptiveSizer::default(),
            trace: None,
            fault_inject_publish_raw_after: None,
            fault_inject_send_overflow_frame_after: None,
            fault_inject_send_raw_loan: false,
            frames_dropped_overflow: std::sync::atomic::AtomicU64::new(0),
            frames_dropped_invariant_violation: std::sync::atomic::AtomicU64::new(0),
            frames_dropped_send_fail: std::sync::atomic::AtomicU64::new(0),
            // hot-path-alloc-ok: constructor — once per publisher, never on the
            // publish hot path. Empty (no heap reservation) until a `block`
            // consumer registers its outstanding mirror at graph-build time.
            block_outstanding: Vec::new(),
            promise_within_last_publish_ns: None,
            doorbell: None,
            event_service: config.event_service,
            // Elision is OFF until the graph runtime arms it
            // (only for graph-owned topics under an active CERULION_NOTIFY_ELISION).
            notify_elision_expected: None,
            notify_elided_count: None,
            notify_gate_last_elided: AtomicBool::new(false),
            // OFF until create_ingress_publisher arms it (raw-frame
            // ingress publishers wake their foreign consumers from publish_raw).
            notify_on_publish_raw: false,
            // No publish has happened yet, so nothing is un-announced.
            unannounced_publish: AtomicBool::new(false),
            // Armed together with the elided counter.
            resweep_notify_count: None,
            // Always on — a saturated listener is a degraded wake path
            // on ANY publisher, so the detector is never opt-in.
            notify_delivery_latch: NotifyDeliveryLatch::new(),
            // Unwired until GraphRuntime installs the shared anchor.
            notify_undelivered_shared: None,
            // Unwired until GraphRuntime installs the node's shared Arcs.
            discard_signal: None,
            replay_suppress: None,
            fault_inject_loan_fail_next: false,
            // Healthy (no discard regime open) until the first
            // incomplete-output drop on this port.
            discard_latch: OutputDiscardLatch::new(),
            // Unwired until GraphRuntime installs the node's
            // per-output NodeHandle-shared discard mirror.
            output_discard_shared: None,
        }
    }

    /// Record an incomplete-output discard on this port and get the
    /// level to log it at (first of a regime → `Error`, repeats → `Debug` with
    /// a running suppressed count). Called by `OutputProxy::Drop` at BOTH loud
    /// discard sites (the missing-variable-field gate and the staged
    /// nested-flush failure) so a persistently broken node floods once, not per
    /// publish. Takes `&mut self` — the publisher is per-port and single-writer,
    /// so no lock is needed.
    #[inline]
    pub(crate) fn record_output_discard(&mut self) -> DiscardLogLevel {
        let level = self.discard_latch.on_discard();
        // Mirror the latch's unconditional running total into the
        // NodeHandle-shared anchor so an off-thread operator can observe the
        // per-output discard count (Principle #3). No-op for non-graph
        // publishers (`None`). `Release` pairs with the reader's `Acquire`.
        if let Some(shared) = &self.output_discard_shared {
            shared.store(self.discard_latch.total_discards(), Ordering::Release);
        }
        level
    }

    /// Record a COMPLETE publish on this port. Returns
    /// `Some(total_suppressed)` exactly when it ends a discard regime (the
    /// caller logs recovery at `info!` once); `None` on the steady-state healthy
    /// path (a single branch — no alloc, no lock). Called by `OutputProxy::Drop`
    /// after a successful `send()` (steady-state and overflow paths).
    #[inline]
    pub(crate) fn record_output_complete(&mut self) -> Option<u64> {
        self.discard_latch.on_complete()
    }

    /// Install this publisher's node's shared publish-DISCARD signal
    /// (the RECORD-side counter `OutputProxy::Drop` and `loan_proxy` bump on a
    /// pre-commit discard). Called once per publisher by `GraphRuntime::build`
    /// with the SAME Arc it hands the node's `ScheduledNode` (so
    /// `fire_node_into`'s delta observes these bumps). No-op surface for every
    /// non-graph publisher (they never call this ⇒ stay `None` ⇒ never mark).
    pub(crate) fn set_discard_signal(&mut self, signal: Arc<std::sync::atomic::AtomicU32>) {
        self.discard_signal = Some(signal);
    }

    /// Install this publisher's node's shared REPLAY-suppress flag (read
    /// by `OutputProxy::Drop` before `commit_sequence`). Installed alongside
    /// `set_discard_signal` by `GraphRuntime::build`.
    pub(crate) fn set_replay_suppress(&mut self, flag: Arc<std::sync::atomic::AtomicBool>) {
        self.replay_suppress = Some(flag);
    }

    /// Bump the node's discard signal — called on every pre-commit
    /// output discard (loan failure here in `loan_proxy`, and via the proxy in
    /// `OutputProxy::Drop`'s all-defer path). No-op (`None`) for a
    /// non-graph publisher. `Release` so `fire_node_into`'s `Acquire` load of the
    /// post-callback value observes it.
    #[inline]
    pub(crate) fn bump_discard_signal(&self) {
        if let Some(sig) = &self.discard_signal {
            sig.fetch_add(1, Ordering::Release);
        }
    }

    /// Whether replay is currently suppressing this fire's publishes
    /// (the scheduler set the shared flag for the firing node's callback
    /// duration). `false` for a non-graph publisher (`None`) and on the
    /// live/record path. `Acquire` pairs with the scheduler's `Release` store.
    #[inline]
    pub(crate) fn replay_suppress_active(&self) -> bool {
        self.replay_suppress
            .as_ref()
            .is_some_and(|f| f.load(Ordering::Acquire))
    }

    /// TEST SEAM: arm a one-shot loan failure on the next `loan_proxy`
    /// (deterministically forces the pre-commit discard path — SHM pressure is
    /// nondeterministic). Test-gated; production never arms it.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn force_discard_next_fire(&mut self) {
        self.fault_inject_loan_fail_next = true;
    }

    /// Arm the self-healing notify-elision gate. The graph
    /// runtime calls this once per graph-owned publisher when
    /// `CERULION_NOTIFY_ELISION` is active, handing (1) the SHARED `expected`
    /// in-process listener count (live-incremented by the build loop, final by
    /// run time) and (2) the SHARED `elided` counter it exposes to tests. After
    /// this, every [`Self::notify_sent_sample`] whose topic's LIVE listener
    /// count equals `expected` SKIPS the iceoryx2 notify (the doorbell still
    /// rings). No-op surface for every non-graph publisher (service layer, rmw,
    /// raw `create_publisher`) — they never call this, so `expected` stays
    /// `None` and they notify exactly as before.
    pub(crate) fn arm_notify_elision(
        &mut self,
        expected: Arc<std::sync::atomic::AtomicUsize>,
        elided: Arc<std::sync::atomic::AtomicU64>,
        resweep: Arc<std::sync::atomic::AtomicU64>,
    ) {
        self.notify_elision_expected = Some(expected);
        self.notify_elided_count = Some(elided);
        // The boundary-resweep fire counter is armed
        // in lockstep with elision — a publisher that can elide can also resweep.
        self.resweep_notify_count = Some(resweep);
    }

    /// Arm the raw-INGRESS wake — after this, every [`Self::publish_raw`]
    /// fires the `SentSample` notify itself (through [`Self::notify_sent_sample`],
    /// so the elision gate still applies and an unarmed ingress publisher
    /// always wakes its foreign consumers). Called ONLY by
    /// [`crate::transport::TransportManager::create_ingress_publisher`], so the
    /// notify is STRUCTURAL for the DDS-bridge `RawIngressRoute` + the network
    /// re-inject `IngressInjector` and can never be forgotten (the defect this closes:
    /// a `RawIngressRoute` publishing raw frames with no wake leaves an event-driven
    /// `topic hz` on a raw-routed topic blocked forever).
    pub(crate) fn arm_publish_raw_notify(&mut self) {
        self.notify_on_publish_raw = true;
    }

    /// `block`: register one consumer's outstanding counter. The
    /// graph runtime calls this once per `block` consumer of this
    /// publisher's topic (all-`block` topics only). The same `Arc` is
    /// shared into that consumer's [`CerulionSubscriber`] (drain
    /// decrement) and the producer's pre-fire check (fullness read).
    pub(crate) fn register_block_outstanding(&mut self, counter: crate::credit::CreditWord) {
        self.block_outstanding.push(counter);
    }

    /// Wire this output's `#[output(promise_within_ms = N)]`
    /// watchdog anchor. The graph runtime mints the `Arc<AtomicU64>`, hands
    /// the same handle to the scheduler's `output_promise_within` tracker,
    /// and installs it here so every successful send writes the publish time
    /// into it (see [`Self::record_promise_within_published`]). No-op unless
    /// the runtime wired a watchdog on this output.
    pub(crate) fn register_promise_within(
        &mut self,
        last_publish_ns: Arc<std::sync::atomic::AtomicU64>,
    ) {
        self.promise_within_last_publish_ns = Some(last_publish_ns);
    }

    /// Install this output's NodeHandle-shared discard-count
    /// mirror. The graph runtime mints the `Arc<AtomicU64>`, registers the same
    /// handle into the node's `NodeHandle` per-output discard map, and installs
    /// it here so every [`record_output_discard`](Self::record_output_discard)
    /// stores the latch's running total into it — making the per-output discard
    /// count observable off-thread (Principle #3). No-op unless the runtime wired
    /// it (non-graph publishers never call this). Seeds the anchor with the
    /// latch's current total so a mid-life registration is not lossy.
    pub(crate) fn register_output_discard_count(
        &mut self,
        shared: Arc<std::sync::atomic::AtomicU64>,
    ) {
        shared.store(self.discard_latch.total_discards(), Ordering::Release);
        self.output_discard_shared = Some(shared);
    }

    /// Open this publisher's producer-OWNED SHM doorbell (rings on each
    /// `notify_sent_sample`). Keyed by `self.topic` so the consumer's
    /// `DoorbellRegistry` (same `ns`) maps the same `/cer_db_<ns>_<hash>` page.
    /// `open_owned` ⇒ this publisher `shm_unlink`s the name on Drop. Called once
    /// per publisher at graph build when the doorbell policy is active.
    /// Non-fatal on error (the consumer then wakes on the ≤100µs timer backstop).
    ///
    /// `pub` since the parity work on the event-driven rmw wait: the rmw
    /// arms it at `rmw_create_publisher` under
    /// `crate::doorbell::default_namespace()` — the SAME producer-side ring
    /// the graph build arms, at a second call site rather than a second
    /// mechanism. (Off Linux the doorbell is the no-op stub; arming is
    /// harmless.)
    pub fn enable_doorbell(&mut self, ns: &str) {
        match crate::doorbell::Doorbell::open_owned(ns, &self.topic) {
            Ok(db) => self.doorbell = Some(db),
            Err(e) => tracing::warn!(
                topic = %self.topic, error = ?e,
                "failed to open producer doorbell; consumers wake on the timer backstop only"
            ),
        }
    }

    /// Like [`Self::enable_doorbell`], but the bell is opened UNOWNED —
    /// created if absent, NEVER unlinked on drop. For producers that can
    /// share one topic name (the rmw: ROS topics are provisioned at TWO
    /// publishers — the `/rosout` shape, two processes' publishers on one
    /// topic), where an OWNED bell would let the first publisher to die
    /// unlink the name under the survivor: the survivor keeps ringing its
    /// old inode while a wait set created afterwards maps a fresh page and
    /// never hears it. A per-process refcount would not help — each
    /// process's last publisher would still unlink — so the bell simply
    /// outlives individual publishers. Residual, stated: the page (one
    /// cache line under `/cer_db_<ns>_<hash>`) can outlive every publisher
    /// on the machine until the next creator re-uses the name — bounded by the
    /// topic count; and because a re-created publisher JOINS the same page,
    /// no consumer goes stale on a re-create.
    pub fn enable_doorbell_shared(&mut self, ns: &str) {
        match crate::doorbell::Doorbell::open_unowned(ns, &self.topic) {
            Ok(db) => self.doorbell = Some(db),
            Err(e) => tracing::warn!(
                topic = %self.topic, error = ?e,
                "failed to open shared producer doorbell; consumers wake on the fd/timer backstop only"
            ),
        }
    }

    /// Test-only public wrapper over `Self::register_promise_within` so
    /// integration tests (separate crate) can wire a `promise_within` anchor
    /// onto a `TestTransport`-minted publisher and assert that a raw publish
    /// path (`publish_raw`) resets it. Gated behind `test-helpers` — NOT part
    /// of the production surface (production wires this from
    /// `GraphRuntime::build`).
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn register_promise_within_for_test(
        &mut self,
        last_publish_ns: Arc<std::sync::atomic::AtomicU64>,
    ) {
        self.register_promise_within(last_publish_ns);
    }

    /// Test-only public wrapper over `Self::send_overflow_frame` so an
    /// integration test can pin the `send_overflow_frame`
    /// `record_promise_within_published` call site (the variable-slice overflow
    /// re-send) directly — it is
    /// byte-identical to the `publish_raw` site but only reachable via a real
    /// loan spill in production. Gated behind `test-helpers`.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn send_overflow_frame_for_test(
        &mut self,
        header_bytes: &[u8],
        payload: &[u8],
    ) -> TransportResult<()> {
        self.send_overflow_frame(header_bytes, payload)
    }

    /// Record that a frame was just published — reset the
    /// `promise_within_ms` watchdog window to the current clock so the
    /// scheduler's next `step()` does not count a miss. A no-op (`None`) unless
    /// the output declares `promise_within_ms`. `Release` so the scheduler's
    /// `Acquire` load sees the write. Reads `clock.now_ns()` — the same
    /// deterministic source the scheduler uses.
    ///
    /// Called from every successful-send site, alongside
    /// [`Self::record_block_published`] — there are **FOUR** direct callers, and
    /// a new send path MUST call this too or its frames will silently fail to
    /// reset the watchdog:
    ///   1. `publish_raw` (raw-FFI re-publish path) — pinned by
    ///      `promise_within_iox2_test::publish_raw_resets_promise_within`
    ///      (via `register_promise_within_for_test`).
    ///   2. `send_overflow_frame` (variable-slice loan spill) — byte-identical
    ///      to (1), pinned by
    ///      `promise_within_iox2_test::send_overflow_frame_resets_promise_within`
    ///      through `send_overflow_frame_for_test` (a real spill fixture is
    ///      heavier, and the one-line call is the same).
    ///   3. `send_raw_loan` (the flatten-into-loan publish path
    ///      — the rmw bridge's publish, including its loaned-message borrow
    ///      window) — pinned by
    ///      `promise_within_iox2_test::send_raw_loan_resets_promise_within_and_bumps_block_outstanding`.
    ///   4. `OutputProxy::drop` (the macro node's output publish) — it sends its
    ///      own `SampleMut` and calls this itself (`output_proxy.rs`); it does
    ///      NOT route through (3). Covered e2e by this file's graph arms.
    ///
    /// The count and the attribution were both wrong until this was rewritten:
    /// the doc said THREE callers and named (3) as the path `OutputProxy::drop`
    /// publishes through, "covered e2e by `promise_within_iox2_test`". (4) is a
    /// separate site, so that attribution credited (3) with (4)'s coverage and
    /// (3) had no test caller at all — which is how the 4th send path stayed
    /// untested. Keep this list and the real call sites in step; a reader who
    /// trusts a stale count is exactly the reader who skips the gap.
    pub(crate) fn record_promise_within_published(&self) {
        if let Some(anchor) = &self.promise_within_last_publish_ns {
            anchor.store(self.clock.now_ns(), Ordering::Release);
        }
    }

    /// Test-only public wrapper over `Self::register_block_outstanding` so
    /// integration tests (separate crate) can register a `block` outstanding
    /// mirror on a `TestTransport`-minted publisher. Gated behind
    /// `test-helpers` — NOT part of the production surface (production wires
    /// this from `GraphRuntime::build`).
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn register_block_outstanding_for_test(&mut self, counter: crate::credit::CreditWord) {
        self.register_block_outstanding(counter);
    }

    /// Test-only: force the next `deliver_history` to treat `update_connections`
    /// as failed (so the SentHistory-suppression gate is exercisable without a real
    /// transport failure). Auto-clears after one `deliver_history`.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn force_next_update_connections_fail_for_test(&self) {
        FORCE_NEXT_UPDATE_CONNECTIONS_FAIL.with(|f| f.set(true));
    }

    /// Test-only: the LIVE number of event `Listener`s connected to this
    /// publisher's topic (its event service `dynamic_config().number_of_listeners()`
    /// — the exact value the notify-elision gate reads). Backs the event-level ZERO
    /// proof that a [`super::subscriber::DataOnlySubscriber`] capture tap registers
    /// no listener (this count stays at the same-process consumer count when a
    /// data-only tap attaches, and rises by one when a listener-full tap does).
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn event_listener_count_for_test(&self) -> usize {
        self.event_service.dynamic_config().number_of_listeners()
    }

    /// `block`: record that one frame was published to this topic
    /// — bump every registered consumer's outstanding counter (each
    /// subscriber's queue just gained one sample). Called on every
    /// successful send (steady-state + overflow re-send). A no-op (empty
    /// vec) unless the topic was wired all-`block`. `Release` so the
    /// producer's pre-fire `Acquire` load sees a consistent depth.
    pub(crate) fn record_block_published(&self) {
        for counter in &self.block_outstanding {
            // `record_published` IS the `fetch_add(1, Release)`
            // of the block credit — on whichever word backs the
            // edge, so a split edge's producer bumps the SHARED page and its
            // peer consumer's drain sees the same count.
            counter.record_published();
        }
    }

    /// Count of frames
    /// LOST during `OutputProxy::Drop`'s overflow re-loan path. Operators
    /// should monitor this — a non-zero value means the iceoryx2 pool
    /// was exhausted at re-loan time and the frame is gone. Producer
    /// nodes have no in-band signal; this counter is the only metrics
    /// surface.
    ///
    /// **Operator action:** scale iceoryx2 SHM pool / investigate
    /// publisher loan-rate / check for stuck subscribers.
    pub fn frames_dropped_overflow(&self) -> u64 {
        self.frames_dropped_overflow
            .load(std::sync::atomic::Ordering::Relaxed)
    }

    /// Increment the overflow-drop counter.
    /// Called by `OutputProxy::Drop` when `send_overflow_frame` Err's.
    pub(crate) fn record_dropped_overflow_frame(&self) {
        self.frames_dropped_overflow
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    }

    /// Count of frames lost due
    /// to invariant violations in Drop's defensive branches (see
    /// field doc on `frames_dropped_invariant_violation` for the
    /// three specific paths). A non-zero value is a code-bug signal,
    /// NOT a resource-exhaustion signal.
    ///
    /// **Operator action:** file a bug; production should never reach
    /// these branches. Distinct from `frames_dropped_overflow` so
    /// operators can disambiguate "iceoryx2 pool issue" (overflow
    /// counter) from "code corruption" (this counter).
    pub fn frames_dropped_invariant_violation(&self) -> u64 {
        self.frames_dropped_invariant_violation
            .load(std::sync::atomic::Ordering::Relaxed)
    }

    /// Increment the
    /// invariant-violation drop counter. Called from `OutputProxy::Drop`'s
    /// three documented-unreachable defensive branches.
    pub(crate) fn record_invariant_violation_drop(&self) {
        self.frames_dropped_invariant_violation
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    }

    /// Count of frames LOST at the steady-state commit-then-fail arms
    /// of `OutputProxy::Drop` (missing outbound sample / `send()` error after
    /// the sequence was consumed). See the field doc on
    /// [`Self::frames_dropped_send_fail`] for the reconciliation semantics — a
    /// nonzero value paired with a bagd `frames_lost = 0` gap localizes the loss
    /// to the SEND side.
    ///
    /// **Operator action:** investigate the publisher / iceoryx2 SHM pressure,
    /// NOT the recorder tap.
    pub fn frames_dropped_send_fail(&self) -> u64 {
        self.frames_dropped_send_fail
            .load(std::sync::atomic::Ordering::Relaxed)
    }

    /// Increment the steady-state send-fail drop counter. Called by
    /// `OutputProxy::Drop` on the two counter-less commit-then-fail arms
    /// (missing outbound sample; `send()` Err). Relaxed — a metrics tally, not
    /// a synchronization point (the loud `tracing::error!` at each call site is
    /// the operator's live signal; this is the machine-readable tally).
    pub(crate) fn record_send_fail_drop(&self) {
        self.frames_dropped_send_fail
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    }

    /// Testing-only: arm fault injection on
    /// `send_overflow_frame`. Next `n` calls succeed; (n+1)th returns
    /// `LoanCapacity`. After firing, the field is cleared.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn fault_inject_send_overflow_frame_after(&mut self, n: u32) {
        self.fault_inject_send_overflow_frame_after = Some(n);
    }

    /// Testing-only: arm **fire-once**
    /// fault injection on `publish_raw`. Next `n` calls succeed; the
    /// `(n+1)`th call returns `TransportError::LoanCapacity`. Lets tests
    /// exercise the `publish_raw` loan-failure path (raw-FFI re-publish)
    /// without real iceoryx2 pool exhaustion.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn fault_inject_publish_raw_after(&mut self, n: u32) {
        self.fault_inject_publish_raw_after = Some(n);
    }

    /// Testing-only: arm fire-once fault on
    /// `send_raw_loan`. The next call drops the loan (releasing its pool
    /// slot) and returns `TransportError::Publish` instead of sending.
    /// After firing, the flag clears.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn fault_inject_send_raw_loan(&mut self) {
        self.fault_inject_send_raw_loan = true;
    }

    /// The history depth this publisher's service ACTUALLY
    /// provides — the number of past frames a late joiner really receives,
    /// which the rmw bridge reports as an endpoint's QoS `depth`.
    ///
    /// Not the same as the request whenever the publisher OPENED a service
    /// someone else created deeper: iceoryx2's open verification is
    /// at-least, and the port it returns is sized from the service's static
    /// config. Reporting the ask there would understate what late joiners
    /// receive (Principle #3).
    ///
    /// [`Self::has_history`] deliberately still keys on the REQUEST: it
    /// gates the SentHistory wake, and changing what that gate means is a
    /// transport-behaviour question rather than a reporting one.
    pub fn provisioned_history_size(&self) -> usize {
        self.provisioned_history_size
    }

    /// `true` when this publisher's service was created with
    /// native iceoryx2 history (`history_size > 0`). The rmw bridge reads
    /// this to distinguish TRANSIENT_LOCAL publishers (which retain frames
    /// for late joiners) from VOLATILE ones. Late-joiner delivery itself is
    /// native + zero-copy (`update_connections` in `deliver_history`);
    /// there is no Cerulion-side ring to push to.
    pub fn has_history(&self) -> bool {
        self.history_size > 0
    }

    /// Regime A: attach a metadata-only publish trace.
    /// Every successful publish via `publish_raw` will push a
    /// `PublishTraceEntry` parsed from the WireHeader. Cheap hot-path
    /// addition (parse 24 bytes + push to ring buffer) when attached;
    /// zero overhead when `None` (default).
    pub fn attach_trace(&mut self, trace: Arc<Mutex<PublishTrace>>) {
        self.trace = Some(trace);
    }

    /// Regime A: returns true if a trace is attached.
    pub fn has_trace(&self) -> bool {
        self.trace.is_some()
    }

    /// Record a publish tick's actual payload size into the sliding
    /// window.
    ///
    /// Called from two sites:
    /// 1. `OutputProxy::Drop` steady-state path, after `send()` succeeds.
    /// 2. `OutputProxy::Drop` overflow Err arm (convergence-on-failure):
    ///    the growth happened — record it so the next tick's adaptive loan
    ///    accommodates the size, even though THIS tick's frame was lost.
    ///    Without this, a single overflow that races with SHM exhaustion
    ///    perpetuates: the sizer doesn't learn → next loan is still too
    ///    small → spills again.
    /// 3. `send_overflow_frame` itself on its Ok path. The Err arm there
    ///    returns without recording; site 2 above is the symmetric
    ///    Err-arm record.
    ///
    /// `size` is the wire frame's `total_size` (header + payload).
    /// Aborted ticks (missing variable fields, `PayloadTooLarge` before
    /// any setter ran) are NOT recorded — the demand never materialised
    /// into an attempted publish. See `AdaptiveSizer` docs for the
    /// convergence policy.
    pub(crate) fn record_payload_size(&mut self, size: u32) {
        self.sizer.record(size);
    }

    /// Return the next adaptive loan size for the given schema's
    /// `min_required` floor.
    ///
    /// Cold publishers (sliding window not yet warm) return
    /// `max_slice_len`; warm publishers return
    /// `min(recent_max × 1.5, max_slice_len).max(min_required)`. Used
    /// by `loan_proxy` to pick a smaller-than-`max_slice_len` slot
    /// when the recent payload sizes warrant it.
    fn adaptive_loan_size(&self, min_required: u32) -> u32 {
        // The sizer is u32 in/out; MaxSliceLen unwraps
        // via `.get()` for the u32 value.
        self.sizer
            .next_loan_size(min_required, self.max_slice_len.get())
    }

    /// Test introspection: the current adaptive loan size the publisher
    /// would return on the next `loan_proxy::<T>()` call for a schema
    /// with the given `min_required`. Production code uses
    /// `adaptive_loan_size`; tests use this accessor to verify the
    /// sliding window converged.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn adaptive_loan_size_for_min_required(&self, min_required: u32) -> u32 {
        self.adaptive_loan_size(min_required)
    }

    /// Test introspection: whether the sliding window has filled.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn sizer_warm(&self) -> bool {
        self.sizer.warm()
    }

    /// Test introspection: the `recent_max` over the sliding window.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn sizer_recent_max(&self) -> u32 {
        self.sizer.recent_max()
    }

    /// Loan a SHM-backed `OutputProxy<T>` for zero-copy publishing.
    ///
    /// The returned proxy `Deref`s to `T::Writer<'_>` (the SHM-backed
    /// `<Name>Shm` type for the schema). User code writes fields directly
    /// — fixed fields land in the `#[repr(C)]` overlay, variable fields
    /// land in the loaned SHM region with offset-table bookkeeping. On
    /// drop the proxy publishes the wire frame to all subscribers.
    ///
    /// # Loan size
    ///
    /// Always `max_slice_len`. By design every loan is the same size
    /// regardless of payload, so replay is byte-identical across cold and
    /// warm caches.
    ///
    /// # Errors
    ///
    /// - [`TransportError::MaxSliceLenRequired`] if `T` is a variable schema
    ///   (`VARIABLE_FIELD_COUNT > 0`) but the publisher was created with a
    ///   buffer too small to host the WireHeader + fixed section + offset
    ///   table.
    /// - [`TransportError::LoanCapacity`] if iceoryx2's sample pool is
    ///   exhausted.
    /// - [`TransportError::Loan`] for other iceoryx2 loan failures.
    pub fn loan_proxy<T: ShmMessage>(&mut self) -> TransportResult<OutputProxy<'_, T>> {
        // Force monomorphization-time
        // const-eval of `_SHM_INVARIANTS` (asserts WireHeader::SIZE +
        // T::WIRE_FIXED_SIZE + 8 * T::VARIABLE_FIELD_COUNT <= u32::MAX).
        // A pathological hand-written `impl ShmMessage` fails to
        // COMPILE at this call site, not panic at runtime.
        let _: () = <T as ShmMessage>::_SHM_INVARIANTS;

        // Drain pending subscriber events (e.g. SubscriberConnected → deliver
        // history). On every loan path it keeps history delivery + late-joiner
        // notifications wired.
        self.check_subscriber_events();

        // `_SHM_INVARIANTS` above proves the sum fits in u32; the
        // `as u32` cast is provably non-truncating. Optimizer
        // constant-folds for any concrete T.
        let min_required: u32 =
            (WireHeader::SIZE + T::WIRE_FIXED_SIZE + 8 * T::VARIABLE_FIELD_COUNT) as u32;
        if self.max_slice_len.get() < min_required {
            return Err(if T::VARIABLE_FIELD_COUNT > 0 {
                TransportError::MaxSliceLenRequired {
                    topic: self.topic.to_string(),
                }
            } else {
                // `needed`/`available` carry byte counts as `usize`
                // for compatibility with the error variant's shape.
                TransportError::BufferTooSmall {
                    topic: self.topic.to_string(),
                    needed: min_required as usize,
                    available: self.max_slice_len.get() as usize,
                }
            });
        }

        // 1. Loan the adaptive slot size — `min(recent_max × 1.5,
        //    max_slice_len).max(min_required)` once the sliding
        //    window is warm; `max_slice_len` while cold (adaptive
        //    sizing). For graphs whose payloads are much
        //    smaller than the configured ceiling, this drops
        //    steady-state SHM reservation 100×+. The overflow spill
        //    closes the convergence-on-growth gap: payload spikes
        //    past the adaptive loan trigger an in-tick spill to a
        //    heap fallback (Vec<u64>, 8-byte aligned), and
        //    `OutputProxy::Drop` re-loans a fresh sample sized to fit.
        //    The hard ceiling fires as `PayloadTooLarge` only when
        //    the actual required size exceeds `max_slice_len`.
        // `loan_size` is `u32` (sizer in/out); iceoryx2's
        // `loan_slice_uninit` takes `usize` — widen at the boundary
        // (lossless).
        // TEST SEAM: a one-shot forced loan failure (SHM pressure is
        // nondeterministic). Fires the SAME discard-signal bump + `Err` the real
        // loan-failure arm below does, so a test can deterministically record a
        // marked (discarded) fire. The READ is unconditional (always `false` in
        // production — the setter `force_discard_next_fire` is test-gated),
        // matching the `fault_inject_publish_raw_after` pattern: a `#[cfg]`-gated
        // read would trip `dead_code` in a plain build.
        if self.fault_inject_loan_fail_next {
            self.fault_inject_loan_fail_next = false;
            self.bump_discard_signal();
            return Err(TransportError::LoanCapacity {
                topic: self.topic.to_string(),
            });
        }

        let loan_size: u32 = self.adaptive_loan_size(min_required);
        let mut sample = match self.publisher.loan_slice_uninit(loan_size as usize) {
            Ok(sample) => sample,
            Err(e) => {
                // A pre-commit loan FAILURE (SHM pressure) IS a live
                // discard — but no `OutputProxy` is ever constructed, so its
                // Drop cannot signal. Bump the node's discard signal HERE so
                // this fire records the marker even for a single-output node
                // whose sole loan fails (a loaded humanoid graph does this). Burns no
                // sequence (never reached `commit_sequence`).
                self.bump_discard_signal();
                // hot-path-alloc-ok: loan-FAILURE error path only — formats the
                // iceoryx2 error to classify it; never runs on a successful loan.
                //
                // iceoryx2 0.9.1's `LoanError` Display renders
                // `LoanError::ExceedsMaxLoans`; a match on the
                // older `ExceedsMaxLoanedSamples` spelling is DEAD, and maps a
                // loan-budget exhaustion to the generic `Loan` variant
                // instead of `LoanCapacity`. (`ExceedsMaxLoanSize` — capital
                // `S` — cannot false-match the lowercase-`s` plural.)
                let msg = format!("{}", e);
                return Err(
                    if msg.contains("ExceedsMaxLoans") || msg.contains("OutOfMemory") {
                        TransportError::LoanCapacity {
                            topic: self.topic.to_string(),
                        }
                    } else {
                        TransportError::Loan {
                            topic: self.topic.to_string(),
                            reason: msg,
                        }
                    },
                );
            }
        };

        // 2. Initialise the *header + fixed section + offset table* prefix
        //    to zero. The variable payload region (everything past the
        //    offset table) is left uninitialised — it gets overwritten by
        //    user writes for the bytes the wire frame actually uses, and
        //    bytes past `WireHeader::total_size` are never observed by
        //    well-behaved subscribers (they slice on `total_size`).
        //
        //    Soundness:
        //    - `iceoryx2::SampleMutUninit::assume_init` is a `transmute_copy`
        //      that reads no bytes — the compile-time contract is that
        //      every byte the consumer touches is initialised.
        //    - Subscribers slice on `WireHeader::total_size` (computed by
        //      `OutputProxy::Drop` from the writer cursor), so they only
        //      read inside the prefix we zero-init plus the bytes the user
        //      explicitly wrote.
        //    - `&[u8]` of `MaybeUninit<u8>`-backed memory is sound for
        //      every bit pattern (`u8` has no invalid bit patterns); we
        //      additionally guarantee no consumer reads those untouched
        //      bytes by clamping reads to `total_size`.
        //
        //    Determinism:
        //    - The prefix length is statically known per schema (fixed
        //      schemas: `WireHeader::SIZE + WIRE_FIXED_SIZE`; variable
        //      schemas: `WireHeader::SIZE + WIRE_FIXED_SIZE + 8 *
        //      VARIABLE_FIELD_COUNT`). It does *not* scale with
        //      `max_slice_len`, so a 4 MB image loan pays the same init
        //      cost as a 48-byte Twist loan.
        //    - For fixed schemas this also guarantees that any field the
        //      user forgot to write is observed as zero (deterministic),
        //      preserving Replay = Live.
        //    - For variable schemas the offset-table bytes are zero so
        //      `from_bytes`/`from_bytes_mut` see "no field written yet"
        //      — but `<Name>Shm::from_bytes_mut` re-zeroes them anyway,
        //      so this branch is belt-and-suspenders for that case.
        let prefix_len = WireHeader::SIZE + T::WIRE_FIXED_SIZE + 8 * T::VARIABLE_FIELD_COUNT;
        let uninit_buf: &mut [MaybeUninit<u8>] = sample.payload_mut();
        for slot in uninit_buf.iter_mut().take(prefix_len) {
            slot.write(0u8);
        }

        // 3. Promote to an initialised SampleMut. SAFETY: see the
        //    soundness argument above — the prefix is initialised, the
        //    tail is `MaybeUninit<u8>` but never observed.
        let mut sample = unsafe { sample.assume_init() };

        // 4. Stamp the WireHeader at bytes [0..32]. `total_size` is set
        //    provisionally to the worst case (max_slice_len); Drop will
        //    rewrite it once the writer cursor is final. Timestamp is
        //    captured at loan time so all subscribers see the
        //    publisher-side time, not drop-side. (Under the macro
        //    lazy-loan the loan — and thus this stamp — now happens at the
        //    port's first write, not tick-start. Replay-safe: the gating
        //    clock does not advance mid-step in deterministic runs, so every
        //    write in a tick reads the same `now_ns()`.)
        //
        //    `sequence` is PROVISIONAL here — the current counter
        //    value WITHOUT incrementing. The counter is consumed
        //    (`fetch_add`) only on the COMMIT paths in `OutputProxy::Drop`
        //    (`commit_sequence`), which rewrite this header field alongside
        //    `total_size`. A loan that never commits (the discard /
        //    early-exit class) therefore burns NO sequence — published
        //    streams stay gap-free and bagd's / the drop_oldest detector's
        //    wire-seq gap accounting counts only REAL losses. The
        //    provisional value is never observed: iceoryx2 delivers only
        //    sent samples, and every send site stamps the committed value
        //    first. Reading `load` here (vs a non-atomic shadow) is safe
        //    from torn interleavings because the proxy exclusively borrows
        //    this publisher (`&'loan mut`) for its whole lifetime — no
        //    second loan or commit can run concurrently on this port.
        let seq = self.sequence.load(Ordering::Relaxed);
        let header = WireHeader {
            schema_hash: T::SCHEMA_HASH,
            // Provisional `total_size` — `OutputProxy::Drop` rewrites
            // this from the actual writer cursor before send. This is
            // now the adaptive loan size rather than
            // the static `max_slice_len`, so the provisional
            // header reads correctly even for an immediate Drop on a
            // partial-write tick.
            //
            // `_SHM_INVARIANTS` (forced at top
            // of fn) proves both `WireHeader::SIZE + WIRE_FIXED_SIZE`
            // and `VARIABLE_FIELD_COUNT` fit in u32 for any T that
            // compiled to this call site. The `as u32` casts are now
            // provably non-truncating; optimizer constant-folds.
            total_size: loan_size,
            offset_table_offset: (WireHeader::SIZE + T::WIRE_FIXED_SIZE) as u32,
            offset_table_count: T::VARIABLE_FIELD_COUNT as u32,
            sequence: seq,
            timestamp_ns: self.clock.now_ns(),
        };
        let payload = sample.payload_mut();
        header.write_to_buf(&mut payload[..WireHeader::SIZE]);

        // 5. Construct the SHM-backed writer over the post-header bytes.
        //    `T::build_writer` matches the schema-specific `from_bytes_mut`:
        //    fixed schemas get `&mut <Name>Shm`, variable schemas get
        //    `<Name>Shm<'_>` with cursor positioned past the offset table.
        //
        //    We need to construct the writer with a borrow that lives for
        //    the proxy's `'_` lifetime, but the SampleMut owns the bytes.
        //    To bridge, we extend the lifetime via raw pointer reborrow —
        //    the SampleMut is moved into SampleHandle::Outbound below and
        //    held for exactly the same lifetime, so the borrow is sound.
        let payload_ptr = payload.as_mut_ptr();
        let payload_len = payload.len();
        // SAFETY: `payload` is a valid mutable slice into the SHM region
        // owned by `sample`. We stash the SampleMut into the SampleHandle
        // immediately below (same proxy lifetime), so the slice stays
        // alive — and exclusively borrowed — for the proxy's full
        // lifetime. The writer's lifetime tracks that of the proxy via
        // `T::Writer<'loan>`.
        let post_header: &mut [u8] = unsafe {
            std::slice::from_raw_parts_mut(
                payload_ptr.add(WireHeader::SIZE),
                payload_len - WireHeader::SIZE,
            )
        };
        // Pass `max_capacity` (the post-header payload region
        // size) and `topic` so variable-schema setters can detect
        // overflow (spill to heap when current loan is too small but
        // ceiling would fit) and attribute `PayloadTooLarge` errors to
        // the originating topic.
        // `MaxPayloadCapacity::from_max_slice_len` is the typed
        // derivation — `MaxSliceLen`'s `>= WireHeader::SIZE` floor is
        // preserved, so the underlying subtraction is lossless.
        let max_capacity = crate::wire::MaxPayloadCapacity::from_max_slice_len(self.max_slice_len);

        // `Arc::clone` on the publisher's topic.
        // Cheap (atomic refcount bump, no allocation) AND borrow-checker-
        // safe (Arc<str> is owned, not borrowed; doesn't conflict with
        // `ProxyPublisher::Iceoryx2(self)` below). This replaces a prior
        // raw-pointer reborrow `unsafe` block that detached an `&str`
        // from `&self.topic: String`.
        let writer = T::build_writer(post_header, max_capacity, Arc::clone(&self.topic));

        // 6. Park the SampleMut inside the SampleHandle so its lifetime is
        //    bound to the proxy. Drop will `take()` it back to call send().
        let handle = SampleHandle::Outbound {
            sample: Some(sample),
            _phantom: std::marker::PhantomData,
        };

        Ok(OutputProxy::new(
            handle,
            writer,
            ProxyPublisher::Iceoryx2(self),
        ))
    }

    /// Best-effort `SentSample` notification — exposed for `OutputProxy::Drop`,
    /// the service layer, and the rmw bridge (which publish via
    /// `publish_raw` / `send_raw_loan` and must wake event-driven subscribers).
    ///
    /// Returns the underlying iceoryx2 result so the caller can log the
    /// specific error code; the data is already in shared memory by the
    /// time this is called, so notification failures don't drop messages.
    pub fn notify_sent_sample(
        &self,
    ) -> Result<usize, iceoryx2::port::notifier::NotifierNotifyError> {
        // Ring this topic's SHM doorbell AFTER the data is already in
        // the iceoryx2 SHM queue (this post-send hook is called by
        // `OutputProxy::Drop`, `send_overflow_frame`, the service layer,
        // and the rmw bridge). `None` unless `enable_doorbell` ran at build,
        // so production pays nothing. A consumer parked in a shallow
        // CPU monitor-wait on the same line wakes the instant of this store.
        // FIREWALL: a wake signal only — never the data path (the sample is
        // already enqueued; the consumer still reads it from SHM in `step()`).
        if let Some(db) = &self.doorbell {
            db.ring();
        }
        // Self-healing NOTIFY ELISION. For a graph-owned topic
        // whose consumers are all SAME-process + level-gated, the scheduler
        // already drains every consumer at its DAG level within the SAME step
        // the producer fired — so the per-publish iceoryx2 event-notify only
        // ever produces a SPURIOUS next-step WaitSet/park wake (the exact stale
        // wake `unified_stale_wake_park_test` pins). Skip it while the topic's
        // LIVE listener count equals the count the runtime PROVED it owns; the
        // instant any FOREIGN listener attaches (a `topic echo`/`hz`, another
        // process's subscriber, an mp sibling) the live count rises above
        // `expected` and notifies resume within ONE publish — the re-check IS
        // the self-heal (a cheap SHM `len()` read, no alloc, no syscall).
        //
        // FIREWALL: eliding a notify changes only WHEN a consumer next wakes,
        // never WHAT it reads/fires (the sample is already committed to the SHM
        // queue by the send() above), and the ≤250ms live heartbeat is the
        // always-present wake backstop — no wedge is possible. The DOORBELL
        // (rung above) still carries the monitor-wait park's data wake, so the
        // park path is unaffected by elision.
        // The LIVE listener count, read AT MOST ONCE per call and ONLY
        // when a branch actually needs it. Two branches can:
        //
        //  * the elision gate below — armed publishers only, and it has
        //    pays this read on every armed publish (a latency
        //    WIN, not an added cost);
        //  * the delivery latch after the notify — but only for a notify
        //    whose outcome differs from the last CLASSIFIED one, or while the
        //    latch is degraded or holds an unconfirmed shortfall (see
        //    `NotifyDeliveryLatch::needs_classification`).
        //
        // So a publisher with elision UNARMED (the shape that never paid this
        // read before the delivery latch existed) pays nothing extra in its steady state.
        //
        // The two branches read the count on DIFFERENT sides of the notify, and
        // the latch is told which (`ListenerCountTiming`) because each side has
        // its own race:
        //
        //  * ARMED — the gate's read happens BEFORE the notify and is REUSED by
        //    the latch (`BeforeNotify`, DECLARED at the read below), so a
        //    listener attaching in between can only raise `triggered`, never
        //    fabricate an undelivered one. A shortfall is believed immediately.
        //  * UNARMED — no count is read here at all; `record_notify_delivery`
        //    reads it itself, which is necessarily AFTER the notify
        //    (`AfterNotify`), so a listener that ATTACHES in the notify→read
        //    window raises `listeners` without raising `triggered` and LOOKS
        //    like a shortfall. That is not a corner: every raw/service/rmw
        //    publisher and every netd/gateway mirror re-inject publisher is
        //    unarmed, and their consumers attach constantly. The latch
        //    therefore requires such a shortfall to PERSIST across two
        //    classified notifies before it warns or counts.
        let mut listener_count = NotifyListenerCount::ReadHere;
        if let Some(expected) = &self.notify_elision_expected {
            let expected_n = expected.load(Ordering::Relaxed);
            let live_n = self.event_service.dynamic_config().number_of_listeners();
            // This read is BEFORE the notify below — stated, not inferred.
            listener_count =
                NotifyListenerCount::CallerRead(live_n, ListenerCountTiming::BeforeNotify);
            let elide = live_n == expected_n;
            // Transition-only breadcrumb (never per publish — hot path). A plain
            // Relaxed load + conditional store: NO write on the steady state, NO
            // allocation (Display-deferred `%topic` + Copy fields).
            if self.notify_gate_last_elided.load(Ordering::Relaxed) != elide {
                self.notify_gate_last_elided.store(elide, Ordering::Relaxed);
                tracing::debug!(
                    topic = %self.topic,
                    live = live_n,
                    expected = expected_n,
                    elide,
                    "notify-elision gate transition (live vs expected in-process listeners)"
                );
            }
            if elide {
                if let Some(c) = &self.notify_elided_count {
                    c.fetch_add(1, Ordering::Relaxed);
                }
                // The frame is COMMITTED and NOT announced. That is the
                // whole state the boundary resweep exists to heal — a listener
                // that attaches after this point must still be told the retained
                // frame is there. Set here and nowhere else on this path: an
                // elided publish is precisely an un-announced one.
                self.unannounced_publish.store(true, Ordering::Relaxed);
                // Elided: the doorbell already rang and the sample is in the SHM
                // queue. Report 0 listeners notified (truthful).
                return Ok(0);
            }
        }
        // A REAL notify is about to run, and a notify reaches EVERY
        // listener on the topic's event service — so everything committed before
        // it is announced. Clear the debt unconditionally: a
        // notify that FAILS is a delivery failure the latch reports, not
        // a reason to make the boundary retry it at live-loop rate.
        self.unannounced_publish.store(false, Ordering::Relaxed);
        let result = self
            .notifier
            .notify_with_custom_event_id(PubSubEvent::SentSample.into());
        // An `Err` reached NOBODY, so it is classified as `triggered =
        // 0` — the field docs promise the counter is bumped on EVERY
        // undelivered notify, and silently skipping the error arm would make a
        // wholly-rejected notify the one failure the signal cannot see.
        self.record_notify_delivery(*result.as_ref().unwrap_or(&0), listener_count);
        result
    }

    /// Feed ONE notify outcome to the delivery latch and emit the
    /// mapped `tracing` event.
    ///
    /// `triggered` is the number of listeners the notify actually reached (`0`
    /// for an `Err`, which reached none). `listeners` says where the topic's
    /// live listener count comes from — see [`NotifyListenerCount`], whose
    /// [`CallerRead`](NotifyListenerCount::CallerRead) arm carries the caller's
    /// OWN [`ListenerCountTiming`] rather than letting this seam infer it.
    ///
    /// The `needs_classification` skip is therefore consulted ONLY on the
    /// [`ReadHere`](NotifyListenerCount::ReadHere) path. It exists solely to
    /// avoid a shared-memory read, so a caller that already paid that read
    /// saves nothing by skipping — and skipping there would silently drop a
    /// real class of detection (a listener registered but NEVER once reached,
    /// whose `triggered` never moves, so the gate would suppress it forever) at
    /// exactly the two sites best placed to see it: the armed per-publish path
    /// of every graph publisher, and the boundary resweep, which is the only
    /// observer of a quiescent producer's foreign listener.
    ///
    /// **The debt-keyed trigger narrowed what the resweep OBSERVES, and the narrowing is real.**
    /// It now fires once per un-announced frame instead of once per silent pass,
    /// so a genuinely quiescent producer's saturated foreign listener is
    /// classified ONCE (on the boundary that announces the last debt) and
    /// `notify_undelivered_count` stops growing thereafter, rather than
    /// climbing for as long as the listener stays attached. What SURVIVES is the
    /// part that matters: that one classification arrives via
    /// [`CallerRead`](NotifyListenerCount::CallerRead) +
    /// [`BeforeNotify`](ListenerCountTiming::BeforeNotify), so it is believed
    /// immediately and the latch's loud regime head still fires. What is lost is
    /// the running COUNT on a producer that has stopped publishing — an
    /// acceptable trade, since the alternative was a notify syscall per
    /// live-loop pass whose only product was that counter.
    ///
    /// Shared by [`Self::notify_sent_sample`] and
    /// [`Self::resweep_notify_elision`] so BOTH notify sites feed one latch —
    /// a boundary notify that cannot reach a saturated listener is the same
    /// degraded wake path as a per-publish one, and must not be invisible.
    fn record_notify_delivery(&self, triggered: usize, listeners: NotifyListenerCount) {
        let (live_n, timing) = match listeners {
            // Read already paid by the caller, which DECLARED its side of the
            // notify: always classify (the gate saves nothing here).
            NotifyListenerCount::CallerRead(n, timing) => (n, timing),
            // The read would be NEW, so the cost gate applies — and when it
            // does happen it lands AFTER the notify (structural: every caller
            // invokes this seam once its notify has already returned).
            NotifyListenerCount::ReadHere => {
                if !self.notify_delivery_latch.needs_classification(triggered) {
                    return;
                }
                (
                    self.event_service.dynamic_config().number_of_listeners(),
                    ListenerCountTiming::AfterNotify,
                )
            }
        };
        let action = self
            .notify_delivery_latch
            .on_notify(triggered, live_n, timing);
        // Mirror the latch's running CONFIRMED total into the
        // NodeHandle-shared anchor so an off-thread operator can observe it
        // (Principle #3). No-op for non-graph publishers (`None`). `Release`
        // pairs with the reader's `Acquire`. Stored on every classification, so
        // a Recovered/None verdict republishes the (unchanged) total too — the
        // anchor can never lag the latch.
        if let Some(shared) = &self.notify_undelivered_shared {
            shared.store(
                self.notify_delivery_latch.total_undelivered(),
                Ordering::Release,
            );
        }
        match action {
            NotifyDeliveryAction::None => {}
            NotifyDeliveryAction::Warn { undelivered } => {
                tracing::warn!(
                    topic = %self.topic,
                    undelivered,
                    triggered,
                    listeners = live_n,
                    "SentSample notify reached fewer listeners than this topic has. \
                     Either a listener's event socket is FULL (nobody is draining that \
                     consumer, so it now wakes only on the heartbeat — drain it, e.g. a \
                     subscriber that never receives), or a listener's process DIED and its \
                     registration has not been reaped yet (harmless; `cerulion clean` / the \
                     next dead-node sweep removes it). Sustained repeats are downgraded to \
                     debug; the total is queryable via notify_undelivered_count()"
                );
            }
            NotifyDeliveryAction::Debug {
                undelivered,
                suppressed,
            } => {
                tracing::debug!(
                    topic = %self.topic,
                    undelivered,
                    suppressed,
                    listeners = live_n,
                    "SentSample notify still undeliverable (suppressed)"
                );
            }
            NotifyDeliveryAction::Recovered { suppressed } => {
                tracing::info!(
                    topic = %self.topic,
                    suppressed,
                    listeners = live_n,
                    "SentSample notify delivery recovered — every listener reachable \
                     again"
                );
            }
        }
    }

    /// Install this output's NodeHandle-shared undelivered-notify
    /// mirror. The graph runtime mints the `Arc<AtomicU64>`, registers the same
    /// handle into the node's `NodeHandle` per-output map, and installs it here
    /// so every classified notify stores the latch's running total into it —
    /// making the count observable off-thread (Principle #3). No-op unless the
    /// runtime wired it (non-graph publishers never call this). Seeds the anchor
    /// with the latch's current total so a mid-life registration is not lossy.
    /// Mirrors [`Self::register_output_discard_count`].
    pub(crate) fn register_notify_undelivered_count(
        &mut self,
        shared: Arc<std::sync::atomic::AtomicU64>,
    ) {
        shared.store(
            self.notify_delivery_latch.total_undelivered(),
            Ordering::Release,
        );
        self.notify_undelivered_shared = Some(shared);
    }

    /// Running total of listener-notifies on this topic that could not
    /// be delivered, across all regimes and independent of log level.
    ///
    /// Nonzero means at least one listener on this topic did not receive a
    /// notify it should have: either it is not being drained (its event socket
    /// filled, so it fell off the event-driven wake path and now only wakes on
    /// the live loop's ≤250 ms heartbeat), or its process died and its
    /// registration has not been reaped yet.
    ///
    /// Zero is the STEADY-STATE expectation on a healthy graph — with one
    /// accepted transient exception. The three classes that could make a
    /// healthy publisher report nonzero, and what each actually does:
    ///
    /// * an ELIDED notify triggers nothing but is not a delivery
    ///   failure — NEVER counted: the gate returns before the latch sees it;
    /// * a listener ATTACHING between an elision-unarmed publisher's notify and
    ///   its listener-count read momentarily looks like a shortfall — not
    ///   counted while the latch is HEALTHY: the persistence rule makes the
    ///   first such observation arm a suspicion only (nothing logged, nothing
    ///   counted), and a single attach race resolves healthy on the very next
    ///   classified notify. Two things scope that "not counted": inside an
    ///   ALREADY-OPEN degraded regime the persistence guard is skipped (the
    ///   shortfall condition is established), so an attach race landing there
    ///   is counted on its first observation; and a SECOND attach landing in
    ///   the very next notify→read window confirms the armed suspicion, so a
    ///   burst of attaching consumers can bump the counter a few times;
    /// * a listener DEREGISTERING between an ARMED publisher's count read and
    ///   its notify — the ONE accepted false positive: it IS counted, exactly
    ///   once, and then the latch re-arms SILENTLY. There is deliberately NO
    ///   `Recovered` line for it: a count-once regime leaves `suppressed == 0`,
    ///   and the latch does not announce recovery from a lone warn (the one
    ///   `warn!` already told the whole story: the `OutputDiscardLatch` rule), so the
    ///   counter simply STAYS at its bumped value with nothing following. It is
    ///   accepted rather than suppressed because that count was read BEFORE the
    ///   notify, where the persistence rule would buy nothing (an attaching
    ///   listener cannot fabricate a shortfall there) and would delay every
    ///   REAL detection by a notify to hide a self-correcting blip.
    ///
    /// So the operator reading is: a count that is 0, or that ticked up a few
    /// times while consumers were attaching/detaching and then stopped, is
    /// healthy — a LINGERING nonzero value is the normal resting state after
    /// such a blip, NOT an unresolved condition, and no recovery line is owed.
    /// A count that KEEPS GROWING is the real signal (a listener nobody drains,
    /// or a dead registration). The sibling [`NotifyDeliveryLatch`] docs carry
    /// the same accounting from the latch's side.
    ///
    /// Node-local (reachable from the node's own tick code via
    /// [`AnyPublisher::notify_undelivered_count`](crate::graph::node::AnyPublisher::notify_undelivered_count)).
    /// The off-thread OPERATOR surface is the per-output
    /// `NodeHandle::notify_undelivered_count` accessor, which the graph runtime
    /// wires to the SAME count via the crate-private
    /// `register_notify_undelivered_count`.
    pub fn notify_undelivered_count(&self) -> u64 {
        self.notify_delivery_latch.total_undelivered()
    }

    /// The LIVE-LOOP BOUNDARY re-check for the notify-elision
    /// self-heal. Returns the number of listeners a boundary notify triggered
    /// (0 when it did nothing).
    ///
    /// # The gap this closes
    ///
    /// The self-heal — resume notifies the instant a FOREIGN listener
    /// attaches — is evaluated ONLY inside [`Self::notify_sent_sample`], i.e.
    /// ONLY when the producer publishes through the `OutputProxy` path. That is
    /// airtight while the producer publishes on a steady cadence, but it leaves
    /// a real hole on two shapes seen on a live `graph run attach`:
    ///
    /// 1. **The producer goes quiescent / the live loop stalls.** Once the gate
    ///    last elided, a foreign `topic echo`/`hz` (or an event-paced network
    ///    consumer) that attaches afterward is never re-detected until the NEXT
    ///    publish — which may never come. The subscriber blocks on the topic's
    ///    event listener forever (the elided publisher never rings it).
    /// 2. **The producer publishes off the gate** (e.g. a raw wire frame via
    ///    [`Self::publish_raw`], which by contract does not notify). Those
    ///    publishes deliver data to the subscriber's SHM queue but never
    ///    re-evaluate the elision gate, so a listener-driven consumer is not
    ///    woken to drain them.
    ///
    /// Both leave a foreign LISTENER-full subscriber stuck: the frames are in
    /// shared memory (delivered by iceoryx2's `send`, which auto-updates its
    /// subscriber connections) but no `SentSample` wake ever fires.
    ///
    /// # The remedy: re-check at a boundary that always runs
    ///
    /// The graph runtime calls this on every live-loop iteration (via
    /// `pump_history_all`, the same ≤250 ms cadence that already services
    /// quiescent publishers' late-joiner history), so a foreign listener gets
    /// notifies flowing again within one heartbeat REGARDLESS of the producer's
    /// publish cadence — a structural boundary, not a timer.
    ///
    /// # The CONTRACT: announce a DEBT, never observe a silent pass
    ///
    /// It fires **at most one notify per un-announced frame**, and only while a
    /// foreign listener is present. Both conjuncts are required:
    ///
    /// * [`Self::unannounced_publish`] — a frame was committed to the SHM queue
    ///   with nothing rung (the elided arm, or an off-gate
    ///   [`Self::publish_raw`]). Firing CLEARS it, so the wake happens once and
    ///   the next silent pass fires nothing.
    /// * `number_of_listeners() > expected` — somebody FOREIGN is waiting. With
    ///   only the in-graph level-gated consumers attached (`live == expected`)
    ///   nothing fires, so the elision latency win is untouched, and a
    ///   DATA-ONLY tap registers no listener at all (the
    ///   data-only-keeps-elision-armed invariant holds).
    ///
    /// A trigger of the SECOND conjunct plus "the per-publish
    /// path did not notify since the last boundary" reads as "a silent
    /// pass observed a foreign listener" — true on every quiet iteration of a
    /// ~1 kHz live loop. MEASURED on an armed graph publisher with one vizd wake
    /// attached: **198 notifies for 2 frames**, and 302 for 47 at 30 Hz (6.4x the
    /// frame rate). Worse, the notify reaches the topic's OWN in-graph consumer
    /// listener on the live WaitSet, so the graph wakes ITSELF and free-runs —
    /// 9/s → 136/s measured, the `unified_stale_wake_park` class re-created by a
    /// viewer. Keyed on the debt, a quiescent producer costs ZERO.
    ///
    /// Cost model: with a foreign listener attached the per-publish
    /// gate never elides, so every publish notifies directly and the boundary
    /// adds nothing — ONE `sendto` per frame. The boundary's own notify is
    /// reachable only on the two shapes above, at most once each.
    ///
    /// FIREWALL: like the per-publish path, a boundary notify changes only WHEN
    /// a consumer next wakes, never WHAT it reads (Principle #7) — the sample is
    /// already in the SHM queue.
    ///
    /// No-op (returns 0) unless elision is armed on this publisher.
    pub(crate) fn resweep_notify_elision(&mut self) -> usize {
        let Some(expected) = &self.notify_elision_expected else {
            return 0;
        };
        let expected_n = expected.load(Ordering::Relaxed);
        let live_n = self.event_service.dynamic_config().number_of_listeners();
        // Only a FOREIGN listener (live above the runtime's PROVED-owned count)
        // needs a wake the elided per-publish path withheld. `live == expected`
        // ⇒ elision is correctly engaged (only in-graph level-gated consumers,
        // which drain in-step) — no notify. `live < expected` ⇒ an owned
        // listener is transiently gone; the per-publish path never elides there
        // either, so nothing to heal at the boundary.
        if live_n <= expected_n {
            return 0;
        }
        // The TRIGGER — is there anything to announce? The runtime runs
        // this boundary on EVERY `live_step` iteration (via `pump_history_all`),
        // NOT on a 250 ms timer, so the condition has to be about the PUBLISHER's
        // state, never about the pass. A frame committed without a notify (an
        // elided publish, or an off-gate `publish_raw`) sets the debt; a
        // `SentSample` notify clears it (see the field doc for why
        // `deliver_history`'s `SentHistory` deliberately does not). `swap` takes
        // the debt and clears it in one Relaxed RMW — this boundary is the
        // announcement.
        //
        // Ordering is load-bearing: the `live_n <= expected_n` return ABOVE must
        // NOT consume the debt. An elided publish with no foreign listener yet is
        // exactly the late-attach shape — the debt has to survive until a listener
        // shows up, which is the whole guarantee this function exists for.
        if !self.unannounced_publish.swap(false, Ordering::Relaxed) {
            return 0;
        }
        // Transition-only breadcrumb (shares the per-publish gate's latch so the
        // two paths never double-log a steady state). A boundary re-arm means
        // the producer went quiescent / published off the gate while a foreign
        // listener was live — the exact hole this resweep exists for.
        if self.notify_gate_last_elided.swap(false, Ordering::Relaxed) {
            tracing::debug!(
                topic = %self.topic,
                live = live_n,
                expected = expected_n,
                "notify-elision re-armed at the live-loop boundary \
                 (foreign listener detected off the publish path)"
            );
        }
        // Wake the foreign listener to drain whatever is queued (the sample is
        // already committed to SHM; this is a wake only). Best-effort — a notify
        // failure never loses data (Principle #6).
        let triggered = self
            .notifier
            .notify_with_custom_event_id(PubSubEvent::SentSample.into())
            .unwrap_or(0);
        // The boundary notify is a REAL notify, so it feeds the SAME
        // delivery latch as the per-publish path. The debt-keyed trigger defines WHEN this is
        // reached: a FOREIGN listener is present AND this publisher holds an
        // un-announced frame — a debt only an elided publish or an off-gate
        // `publish_raw` can create — so a saturated (or dead-but-unreaped)
        // foreign listener on a producer that never notifies for itself is
        // observed here and nowhere else. It is not observed on EVERY
        // silent pass, which is the point: see `record_notify_delivery`'s doc
        // for what that narrows about the signal. `live_n` was read just above —
        // reuse it rather than re-reading the SHM dynamic config, and DECLARE
        // that it was read before this notify.
        self.record_notify_delivery(
            triggered,
            NotifyListenerCount::CallerRead(live_n, ListenerCountTiming::BeforeNotify),
        );
        // Count the boundary FIRE (not the skips) for the runtime pin.
        if let Some(c) = &self.resweep_notify_count {
            c.fetch_add(1, Ordering::Relaxed);
        }
        triggered
    }

    /// Re-loan a fresh iceoryx2 sample sized to fit, memcpy the
    /// caller-supplied wire frame (header + post-header payload) into it,
    /// and send. Called by `OutputProxy::Drop` when the writer spilled
    /// to a heap fallback buffer.
    ///
    /// Distinct from `publish_raw` in two ways:
    /// 1. Updates the adaptive sliding window on success (recording overflow
    ///    ticks closes the convergence-on-growth gap).
    /// 2. Best-effort notifies `SentSample` after the send so subscribers
    ///    waiting on the event channel wake.
    ///
    /// History on the overflow path: NATIVE. The
    /// `sample.send()` above feeds iceoryx2's native publisher history queue
    /// (sized at service creation) by SHM offset — zero-copy, no Cerulion
    /// buffer — so overflow ticks are retained for late joiners exactly like
    /// steady-state ticks (Replay = Live, Principle #7). The only remaining
    /// post-send fan-out here is the optional zenoh network bridge.
    ///
    /// `header_bytes` MUST be exactly `WireHeader::SIZE` bytes; the
    /// caller is responsible for having patched `total_size` to the
    /// final wire size before calling.
    #[must_use = "overflow re-loan result must be checked"]
    pub(crate) fn send_overflow_frame(
        &mut self,
        header_bytes: &[u8],
        payload: &[u8],
    ) -> TransportResult<()> {
        // Fault-injection hook (testing-only, fire-once).
        // After firing, field is cleared — subsequent calls succeed.
        if let Some(remaining) = self.fault_inject_send_overflow_frame_after {
            if remaining == 0 {
                self.fault_inject_send_overflow_frame_after = None;
                return Err(TransportError::LoanCapacity {
                    topic: self.topic.to_string(),
                });
            }
            self.fault_inject_send_overflow_frame_after = Some(remaining - 1);
        }
        debug_assert_eq!(
            header_bytes.len(),
            WireHeader::SIZE,
            "send_overflow_frame: header_bytes must be exactly WireHeader::SIZE"
        );
        let total_size = WireHeader::SIZE + payload.len();
        // Debug-assert that the total_size cast is non-truncating. It is
        // bounded by max_capacity:u32 upstream, but the assert pins the
        // invariant against future changes.
        debug_assert!(
            total_size <= u32::MAX as usize,
            "send_overflow_frame: total_size {} exceeds u32::MAX",
            total_size
        );
        let mut sample = self.publisher.loan_slice_uninit(total_size).map_err(|e| {
            // hot-path-alloc-ok: loan-FAILURE error path only (overflow re-loan);
            // formats the iceoryx2 error to classify it, never on success.
            // Match iceoryx2 0.9.1's real `LoanError::ExceedsMaxLoans`
            // Display (the older `ExceedsMaxLoanedSamples` spelling was dead —
            // see the `loan_proxy` twin).
            let msg = format!("{}", e);
            if msg.contains("ExceedsMaxLoans") || msg.contains("OutOfMemory") {
                TransportError::LoanCapacity {
                    topic: self.topic.to_string(),
                }
            } else {
                TransportError::Loan {
                    topic: self.topic.to_string(),
                    reason: msg,
                }
            }
        })?;
        let uninit_buf: &mut [MaybeUninit<u8>] = sample.payload_mut();
        for (i, &b) in header_bytes.iter().enumerate() {
            uninit_buf[i].write(b);
        }
        for (i, &b) in payload.iter().enumerate() {
            uninit_buf[WireHeader::SIZE + i].write(b);
        }
        // SAFETY: total_size bytes were initialised above (the loop covered
        // [0..WireHeader::SIZE) for the header and [WireHeader::SIZE..total_size)
        // for the payload).
        let sample = unsafe { sample.assume_init() };
        sample.send().map_err(|e| TransportError::Publish {
            topic: self.topic.to_string(),
            reason: format!("{}", e),
        })?;

        // `block`: the overflow re-send is mutually exclusive with
        // the steady-state send (OutputProxy::drop `return`s after the
        // overflow arm), so exactly one of them bumps per published frame.
        self.record_block_published();
        // A frame went out — reset the output's
        // `promise_within_ms` watchdog (no-op unless one is wired).
        self.record_promise_within_published();

        // Convergence policy: overflow ticks ALSO record.
        // Recording only successful steady-state publishes would leave a
        // convergence-on-growth gap.
        self.sizer.record(total_size as u32);

        // Best-effort SentSample notification (matches the steady-state path).
        if let Err(e) = self.notify_sent_sample() {
            tracing::trace!(
                topic = %self.topic,
                error = %e,
                "send_overflow_frame: SentSample notification failed (data already delivered)"
            );
        }

        // Native iceoryx2 history retained the SHM frame on
        // `send()` above (zero-copy, by offset) — no Cerulion-side copy. Also,
        // there is no post-send network fan-out here — a graph process is
        // network-free; the separate gateway taps produced topics for egress.

        Ok(())
    }

    // `loan_raw_zeroed` was DELETED. Its only
    // caller was `rmw_borrow_loaned_message`, and its premise was wrong
    // twice over: a zero-filled slot is NOT a constructed ROS message
    // (rosidl defaults ≠ zeros — a loaned Quaternion published untouched
    // read `w == 0.0` where every other rmw serves `w == 1.0`), and the
    // whole-slot memset was the only O(payload) cost on the loan lane.
    // The rmw borrow path now takes `loan_raw_uninit` and runs the
    // typesupport's `init_function` (the message's real construction),
    // with padding zeroed at publish from the layout's pad ranges.

    /// Loan an UNINITIALIZED exact-size raw SHM slot (the
    /// flatten-into-loan path; later also the rmw loaned-message
    /// borrow path). The slot is NOT zero-filled — skipping the memset
    /// is the whole point for multi-megabyte variable payloads the caller is
    /// about to overwrite end-to-end anyway.
    ///
    /// # Invariant (caller contract)
    ///
    /// The caller MUST initialize every byte of
    /// [`RawShmLoanUninit::bytes_uninit_mut`] before calling
    /// [`RawShmLoanUninit::assume_init`], and MUST drop the loan (never send)
    /// if its fill fails partway. The rmw bridge satisfies this via
    /// `flatten_into_uninit`'s bounds-checked cursor: `Ok(n)` proves bytes
    /// `[0, n)` were written and `n == len`; any `Err` drops the loan. No
    /// uninitialized SHM byte can therefore ever be published (Principle #7).
    ///
    /// # Exact-size dependency
    ///
    /// `bytes_uninit_mut()` is expected to return a slice of EXACTLY `len`
    /// bytes — the rmw cursor's `require_full` proof ties to that slice
    /// length. iceoryx2's `loan_slice_uninit(len)` returns exactly `len` for a
    /// `u8` payload today; if a future backend ever rounded the slice up, the
    /// caller's `written == frame_size` re-check would fire and the publish
    /// would fail LOUDLY (RMW_RET_ERROR) — never ship a short/garbage frame.
    /// Load-bearing but fail-safe.
    pub fn loan_raw_uninit(&mut self, len: usize) -> TransportResult<RawShmLoanUninit> {
        let sample = self
            .publisher
            .loan_slice_uninit(len)
            .map_err(|e| TransportError::Loan {
                topic: self.topic.to_string(),
                reason: format!("{}", e),
            })?;
        Ok(RawShmLoanUninit { sample })
    }

    /// Send a raw loan (finalized by the caller — header + payload bytes
    /// already written in place). Returns the recipient count.
    #[must_use = "publish result must be checked for loan/send errors"]
    pub fn send_raw_loan(&mut self, mut loan: RawShmLoan) -> TransportResult<usize> {
        // Fire-once fault hook (testing-only; field is always `false` in
        // production — one branch). Returning here drops `loan`; the inner
        // `SampleMut`'s drop glue releases the SHM slot WITHOUT sending —
        // exactly the error-path contract (no partial/garbage frame on the
        // wire).
        if self.fault_inject_send_raw_loan {
            self.fault_inject_send_raw_loan = false;
            return Err(TransportError::Publish {
                topic: self.topic.to_string(),
                reason: "fault-injected send_raw_loan failure".to_string(),
            });
        }
        let sample = loan
            .sample
            .take()
            .expect("RawShmLoan sent twice (impossible: send consumes)");
        let recipients = sample.send().map_err(|e| TransportError::Publish {
            topic: self.topic.to_string(),
            reason: format!("{}", e),
        })?;
        // `block` invariant: `send_raw_loan` is a 4th
        // send path, and every frame entering subscriber queues MUST bump the
        // outstanding mirror, else a future `block` topic publishing via the
        // flatten-into-loan path would silently overflow (Principle #6).
        // No-op (empty `block_outstanding` vec) unless the topic is all-block.
        self.record_block_published();
        // A frame went out — reset the output's `promise_within_ms` watchdog
        // (no-op unless one is wired), symmetric with `publish_raw`.
        self.record_promise_within_published();
        Ok(recipients)
    }

    /// Publish raw wire bytes (header + payload) directly.
    ///
    /// Re-publishes a caller-supplied wire frame through the normal publisher
    /// port so subscribers receive it via the standard `drain_samples()` path.
    /// Used by raw-FFI nodes that stamp their own headers. (History replay no
    /// longer flows through here: late-joiner delivery is
    /// native/zero-copy via the private `deliver_history` path.)
    ///
    /// **Wire-sequence contract:** the `drop_oldest` eviction
    /// detector reads the frame's wire `sequence` and treats same-publisher
    /// sequences as monotonic; a same-id BACKWARD sequence is classified as
    /// duplicate re-delivery (exactly what history replay produces) and is
    /// never counted as eviction. Callers stamping their own headers must
    /// keep per-publisher sequences monotonically increasing — non-monotonic
    /// custom stamping silently degrades eviction counting on subscribers
    /// (a constant sequence reads as "no evictions, ever").
    ///
    /// Also used by the service layer (request / response frames).
    /// Returns the number of subscribers the frame was delivered to —
    /// callers that require a recipient (the service layer's `send_response`)
    /// can warn loudly on 0 instead of responding into the void.
    #[must_use = "publish result must be checked for loan/send errors"]
    pub fn publish_raw(&mut self, data: &[u8]) -> TransportResult<usize> {
        // Fire-once fault-injection hook (testing-only).
        // Exercises the `publish_raw` loan-failure path (raw-FFI re-publish)
        // without forcing real iceoryx2 pool exhaustion. After firing, the
        // field is cleared.
        //
        // The `#[cfg(...)]` gate on the check + field was removed
        // for FFI layout stability (see field doc).
        // Setter is still test-gated; field stays `None` in production.
        if let Some(remaining) = self.fault_inject_publish_raw_after {
            if remaining == 0 {
                self.fault_inject_publish_raw_after = None;
                return Err(TransportError::LoanCapacity {
                    topic: self.topic.to_string(),
                });
            }
            self.fault_inject_publish_raw_after = Some(remaining - 1);
        }

        let total_size = data.len();

        let mut sample =
            self.publisher
                .loan_slice_uninit(total_size)
                .map_err(|e| TransportError::Loan {
                    topic: self.topic.to_string(),
                    reason: format!("{}", e),
                })?;

        let uninit_buf: &mut [MaybeUninit<u8>] = sample.payload_mut();
        for (i, &byte) in data.iter().enumerate() {
            uninit_buf[i].write(byte);
        }

        // SAFETY: the loan was sized to exactly `data.len()` (`total_size`)
        // and the loop above writes every byte of `uninit_buf` from `data`.
        // Every byte of the payload is therefore initialised before
        // `assume_init`, satisfying iceoryx2's contract that the consumer
        // observes only initialised memory.
        let sample = unsafe { sample.assume_init() };
        let recipients = sample.send().map_err(|e| TransportError::Publish {
            topic: self.topic.to_string(),
            reason: format!("{}", e),
        })?;

        // `block`: every frame that
        // enters every subscriber's queue MUST bump the outstanding mirror —
        // including frames sent via `publish_raw`. Without this, a `block`
        // producer publishing through `ctx.publisher_mut("out").publish_raw(..)`
        // would enqueue frames the mirror never counts, so the scheduler
        // pre-fire would never observe a full queue → no defer → iceoryx2
        // overflow → silent data loss (violates Principle #6). This is the
        // symmetric one-increment-per-published-frame partner of the
        // `record_block_drained` decrement on the consumer's drain. (Native
        // history delivery does not flow through
        // `publish_raw`, so history-replayed frames are NOT counted by this
        // mirror; the graph build warns against `block` + `history_size > 0`
        // for exactly this reason. This counts live sends + genuine raw-FFI
        // re-publishes only.)
        // No-op (empty `block_outstanding` vec) unless the topic is all-block.
        self.record_block_published();
        // A frame went out — reset the output's
        // `promise_within_ms` watchdog (no-op unless one is wired).
        self.record_promise_within_published();

        // Regime A: record the publish into the trace
        // (when attached). Parsing 24 bytes of WireHeader is cheap
        // relative to the iceoryx2 send that just completed.
        if let Some(trace) = &self.trace {
            if let Some(entry) = parse_trace_entry_from_wire(&self.topic, data) {
                // Poison-safe: even if a previous holder panicked, we
                // still want to keep recording.
                let mut t = trace.lock().unwrap_or_else(|e| e.into_inner());
                t.record(entry);
            }
        }

        // A raw-INGRESS publisher wakes its FOREIGN consumers itself —
        // the wake is part of the publish, not a caller convention. Routed
        // through `notify_sent_sample` so the elision gate still applies
        // (an unarmed ingress publisher always notifies — correct: its consumers
        // are cross-process `topic hz`/gateway/mirror readers, never in-graph
        // level-gated). Best-effort: the frame is already in the SHM queue, so a
        // notify failure loses only the wake, never data (Principle #6). `false`
        // for graph outputs / service / rmw — they notify on their own schedule,
        // so this never double-notifies.
        if self.notify_on_publish_raw {
            // Service any subscriber transition before notifying, the same
            // call the typed loan path makes. iceoryx2's
            // `notify_with_custom_event_id` passes `skip_self_deliver = false`,
            // so a notify is delivered to EVERY listener on the topic's event
            // service, this publisher's own included (every
            // `CerulionPublisher` owns one, to hear `SubscriberConnected`).
            //
            // This call used to drain that listener on EVERY raw publish, and
            // under iceoryx2 0.9.1 it had to: an undrained listener filled its
            // own `AF_UNIX SOCK_DGRAM` socket within a few hundred publishes,
            // after which every notify failed and was logged once per publish
            // (~90 raw ingress routes on an attached robot, ~2500 lines/s,
            // ~5 MB/s, enough to fill a root disk). 0.10 removed that failure
            // at the source, so `check_subscriber_events` is now gated on the
            // topic's live listener count and does nothing at all on a steady
            // topic — see its own doc for the gate and for the race it arms
            // across.
            //
            // `check_subscriber_events` rather than a bare listener drain
            // because it is the ONE primitive that also CLASSIFIES what it
            // pulled, so there is one path to reason about instead of a second
            // bespoke one. Be precise about its `SubscriberConnected` arm here:
            // the only publishers reaching this branch are
            // `create_ingress_publisher`'s (the sole `arm_publish_raw_notify`
            // caller) and those are built with `history_size = 0`, so the
            // `deliver_history` it drives is a no-op TODAY. It is not dead
            // weight — it is what keeps this call correct if an ingress
            // publisher ever requests history.
            self.check_subscriber_events();
            let _ = self.notify_sent_sample();
        } else {
            // This publisher does NOT wake anybody from `publish_raw` (a
            // graph output, the service layer, rmw — they notify on their own
            // schedule), so the frame is now in the SHM queue with nothing rung.
            // That is the SECOND shape the boundary resweep exists for
            // (the first being an elided notify), and the resweep now needs the
            // publisher to SAY so rather than inferring it from a silent pass.
            //
            // Inert unless elision is armed — the resweep returns on its first
            // line otherwise — so this costs an unarmed publisher one Relaxed
            // store and nothing else. The `notify_elision_resweep_iox2_test`
            // off-gate arm is the pin: its producer publishes exactly this way.
            self.unannounced_publish.store(true, Ordering::Relaxed);
        }

        Ok(recipients)
    }

    /// Non-blocking check for subscriber events.
    ///
    /// Drains all pending events from the listener. When a `SubscriberConnected`
    /// event is found, drives iceoryx2's NATIVE history delivery to the new
    /// subscriber (see `deliver_history`).
    ///
    /// Public for transport integrators: the service layer and the
    /// rmw bridge publish via `publish_raw` / `send_raw_loan` + notify and
    /// must drain their own listener to service late joiners.
    pub fn check_subscriber_events(&mut self) {
        // iceoryx2 0.10: gate the drain on the topic's LIVE listener count.
        //
        // This function's only action is `deliver_history()` on a
        // `SubscriberConnected`, which happens when a subscriber attaches and
        // at no other time — yet it ran a full listener drain on EVERY
        // `loan_proxy` and every `publish_raw`. Under iceoryx2 0.9.1 the
        // unconditional drain had a second job: an undrained listener filled
        // its datagram socket, after which every later notify from any
        // publisher on the topic failed and logged. 0.10 removed that hazard at
        // the source, so the drain is now paying two `recvmsg` calls, two
        // sequentially consistent atomic operations and a counting-bitset walk
        // per publish to ask a question whose answer is almost always no.
        //
        // The gate is `number_of_listeners()` on the event service the
        // publisher already holds — `self.listeners.len()` on a shared-memory
        // container, one relaxed load. A `CerulionSubscriber` bundles a
        // listener, so any late joiner owed history moves that count. A
        // listener-less `DataOnlySubscriber` does not move it and does not need
        // to: iceoryx2's own `update_connections()` inside the next `send()`
        // flushes retained history into a new tap.
        //
        // A count change ARMS several drains rather than one, because the
        // count rises when the subscriber's listener is created and the
        // `SubscriberConnected` notify lands after it — a single gated drain
        // could fall in that window and see nothing. Staying armed until a
        // transition is actually observed closes that race; the bounded count
        // is what stops a listener that attaches or dies WITHOUT an event (a
        // foreign observer, a crashed consumer) from arming every later publish
        // forever. While disarmed the cost is the one load.
        let live = self.event_service.dynamic_config().number_of_listeners();
        if live != self.last_listener_count {
            self.last_listener_count = live;
            self.self_drains_armed = SELF_DRAIN_ARMED_CALLS;
        }
        if self.self_drains_armed == 0 {
            return;
        }
        self.self_drains_armed -= 1;
        // One `try_wait` empties the queue. Its callback borrows `self.listener`
        // for the whole call while `deliver_history()` below needs `&mut self`,
        // so the callback only FLAGS what it saw and the act happens after the
        // drain returns.
        let mut connected = false;
        let mut disconnected = false;
        // A listener error is still "stop polling": nothing is flagged.
        let _ = self
            .listener
            .try_wait(|activation| match PubSubEvent::try_from(activation.id) {
                Ok(PubSubEvent::SubscriberConnected) => connected = true,
                Ok(PubSubEvent::SubscriberDisconnected) => disconnected = true,
                _ => {}
            });
        if connected || disconnected {
            // The transition this arming was waiting for: stop draining on
            // every publish until the listener count moves again.
            self.self_drains_armed = 0;
        }
        if connected {
            tracing::debug!(topic = %self.topic, "subscriber connected");
            // ONE `deliver_history()` per drain, where 0.9.1 ran one per queued
            // `SubscriberConnected`. `deliver_history` drives iceoryx2's
            // `update_connections()`, which services EVERY newly-connected
            // subscriber in one call, so N connects in one batch still get their
            // history from one call.
            self.deliver_history();
        }
        if disconnected {
            tracing::debug!(topic = %self.topic, "subscriber disconnected");
        }
    }

    /// Drain pending subscriber events (delivering native history to any
    /// freshly-connected late joiner) WITHOUT publishing. The runtime calls this
    /// on a cadence so a quiescent publisher — one that filled history then
    /// stopped sending — still services late joiners (the publish path's
    /// `check_subscriber_events` would otherwise be the only driver). A no-op
    /// when no `SubscriberConnected` is pending (a single non-blocking listener
    /// drain); never fatal (see `deliver_history`).
    pub fn pump_history(&mut self) {
        self.check_subscriber_events();
    }

    /// Drive iceoryx2's NATIVE history delivery to a freshly-connected
    /// subscriber, then fire `SentHistory` to wake it — but only when the
    /// connection refresh succeeded (`Ok`) AND the publisher
    /// actually retains history (`history_size > 0`; a `history_size == 0` /
    /// VOLATILE service delivered nothing, so the wake is
    /// suppressed — see the body for both gates).
    ///
    /// # Why this exists
    ///
    /// iceoryx2 native history is zero-copy — the SHM frames sent so far are
    /// retained by offset in the publisher port's history queue (sized at
    /// service creation via `history_size(N)`) and delivered to a late joiner
    /// automatically by `update_connections()` (and on every subsequent
    /// `send()`). The catch: that delivery is SILENT into the subscriber's
    /// DATA queue — iceoryx2 emits no completion event. Cerulion's
    /// `wait_for_message` blocks on the EVENT channel, so without a Cerulion
    /// wake a late joiner would not drain the now-populated queue until some
    /// unrelated event. `deliver_history` runs from `check_subscriber_events`,
    /// which the publisher drains both on the publish path (`loan_proxy`/`send`)
    /// AND on the runtime's `pump_history` cadence, so
    /// even a PERMANENTLY quiescent publisher (fills history, never sends again)
    /// services a late joiner. On that drain we:
    ///
    /// 1. Call `self.publisher.update_connections()` — iceoryx2 establishes the
    ///    new connection and delivers `min(history.len(), subscriber.buffer_size)`
    ///    most-recent frames into its data queue (per-consumer truncation;
    ///    the intended semantics). Zero-copy: borrow-refcounted SHM offsets.
    /// 2. Fire `SentHistory` via the notifier — but ONLY when step 1 returned
    ///    `Ok` AND `history_size > 0` — so the late joiner's `wait_for_message`
    ///    wakes and drains the delivered frames.
    ///
    /// History size 0 (no history requested — VOLATILE durability in rmw terms)
    /// still ESTABLISHES the connection (step 1 returns `Ok`, delivering
    /// nothing), but step 2 is SUPPRESSED: with zero retained frames a
    /// `SentHistory` wake would be spurious (empty data queue), and the
    /// hard rule is `SentHistory` only on a non-empty delivery. The
    /// late joiner is unaffected — it receives only post-match samples (the DDS
    /// VOLATILE contract), woken by `SentSample` on the next live publish.
    ///
    /// `update_connections` returns `Err` ONLY on a TOTAL connection-establishment
    /// failure → ZERO frames delivered (verified against iceoryx2 0.9.1: the
    /// per-frame history push runs only after `Connection::new` succeeds; a
    /// per-frame drop inside an ESTABLISHED connection still returns `Ok`). So on
    /// `Err` we SUPPRESS the `SentHistory` wake: nothing
    /// landed in the subscriber's queue, so a wake would be spurious. Non-fatal —
    /// the retained SHM frames stay in the publisher's history and deliver when
    /// the connection next establishes (the next subscriber-set change re-attempts
    /// the still-unconnected slot), which fires `SentHistory` then.
    ///
    /// # Test seam
    ///
    /// Under `cfg(test)` / `feature = "test-helpers"`, a test can arm
    /// [`Self::force_next_update_connections_fail_for_test`] to drive the
    /// Err-path gate (SentHistory suppression) WITHOUT a real
    /// `update_connections` failure — the Cerulion API can't induce one. The
    /// forced flag ORs into the real gate (so removing the real `return` is
    /// still caught by the negative test) but is NOT logged as an error (only a
    /// genuine `Err` logs).
    fn deliver_history(&mut self) {
        // Step 1: drive native delivery of the retained history frames into
        // the freshly-connected subscriber's data queue (zero-copy by SHM
        // offset). On a quiescent publisher this is the ONLY thing that
        // populates the late joiner before the next `send()`.
        let uc = self.publisher.update_connections();
        // Test-only: a test may force the suppression gate without a real
        // transport failure. Reads-and-clears (fire-once). Production compiles
        // `forced` to a constant `false`, so the gate below is byte-identical to
        // the original `if let Err(e) = uc { … return; }`.
        #[cfg(any(test, feature = "test-helpers"))]
        let forced = FORCE_NEXT_UPDATE_CONNECTIONS_FAIL.with(|f| f.replace(false));
        #[cfg(not(any(test, feature = "test-helpers")))]
        let forced = false;
        if uc.is_err() || forced {
            // Log ONLY a genuine transport failure — a forced (test) failure is
            // not a real error and must not emit the production error! line.
            if let Err(e) = uc {
                tracing::error!(
                    topic = %self.topic,
                    error = ?e,
                    "iceoryx2 update_connections failed on SubscriberConnected; native \
                     history delivery to the late joiner is deferred until the connection \
                     next establishes (live delivery unaffected)"
                );
            }
            // `Err` == a TOTAL connection-establishment failure == ZERO frames
            // delivered, so SUPPRESS the SentHistory wake: there is nothing in the
            // subscriber's queue to drain and a wake would be spurious. The frames
            // remain in the publisher's history for the next establishment to
            // deliver + wake. A partial-but-nonempty delivery returns `Ok`, so this
            // never suppresses a real wake (see the fn doc).
            return;
        }

        // Hard rule: SentHistory fires ONLY on a non-empty
        // delivery. A `history_size == 0` service (VOLATILE durability, in rmw
        // terms) retained nothing, so `update_connections` established the
        // connection but delivered ZERO frames — a SentHistory wake here would be
        // spurious (the late joiner's data queue is empty; it would re-block
        // immediately). Suppress it. The connection is live; the late joiner
        // receives only post-match samples (the DDS VOLATILE contract), woken by
        // `SentSample` on the next real publish — no frame is lost, nothing
        // stalls (the subscriber wait-loop drains on any wake, and `rmw_wait`
        // polls the data queue, never this event).
        //
        // The gate is `history_size > 0`, a STATIC publisher property — NOT a
        // "delivered zero frames this time" heuristic. SentHistory is genuinely
        // load-bearing for `history_size > 0` late joiners on a quiescent
        // publisher (native iceoryx2 delivery emits no completion event of its
        // own), so suppression must NEVER reach that path.
        if self.history_size == 0 {
            return;
        }

        // Step 2: wake the late joiner so its `wait_for_message` drains the
        // (now natively-delivered) history frames. Reached only on a confirmed
        // `Ok` connection refresh with `history_size > 0`. Best-effort — the data
        // is already in the subscriber's queue; a lost wake only extends its
        // latency to the next event on the topic.
        if let Err(e) = self
            .notifier
            .notify_with_custom_event_id(PubSubEvent::SentHistory.into())
        {
            tracing::error!(
                topic = %self.topic,
                error = ?e,
                "failed to notify SentHistory after native history delivery; \
                 late joiner may block waiting to drain delivered frames"
            );
        }
    }

    /// Returns the topic name.
    pub fn topic(&self) -> &str {
        &self.topic
    }

    /// This publisher's iceoryx2
    /// `UniquePublisherId::value()` — the raw id a served sample's
    /// `Sample::origin()` reports, before the read log hashes it into a
    /// 64-bit producer token.
    ///
    /// RUN-RANDOM by construction (`UniqueSystemId` mints from pid +
    /// creation time), which is exactly why the recording stamps it into the
    /// trace-ring manifest's publisher section: an id means nothing across
    /// runs, so each side translates its OWN run's ids to `(node, output)`
    /// before anything is compared.
    pub fn publisher_id(&self) -> u128 {
        self.publisher.id().value()
    }

    /// Returns the current sequence number (the next value to be assigned
    /// at COMMIT).
    ///
    /// The counter advances at commit (`OutputProxy::Drop`'s send
    /// paths, via the crate-private `Self::commit_sequence`) — NOT at loan.
    /// A loaned-but-
    /// discarded proxy leaves this value unchanged, so "next value" here
    /// means "the sequence the next COMMITTED frame will carry".
    pub fn sequence(&self) -> u32 {
        self.sequence.load(Ordering::Relaxed)
    }

    /// The wire sequence this publisher STARTED at.
    ///
    /// **0 on every live path.** Nonzero only on a restored replay, which seeds
    /// the counter so its first frame continues the recorded stream's numbering
    /// ([`crate::transport::TransportManager::set_replay_sequence_seeds`]).
    pub fn initial_sequence(&self) -> u32 {
        self.initial_sequence
    }

    /// The number of frames THIS publisher has committed — the quantity the
    /// reconciliation identity's producer term means.
    ///
    /// `sequence()` is the next number to assign, which equals the committed
    /// count only when the counter started at 0. It always did until
    /// restored replay gained a seed, after which the two diverge by
    /// exactly the seed: one frame committed by a publisher seeded at 4242
    /// leaves `sequence()` at 4243, and reporting THAT as the producer term
    /// mints 4242 frames of phantom loss against bag-side counts that only ever
    /// saw one.
    ///
    /// This is the ONE definition of the subtraction — the two reconciliation
    /// log sites read it rather than each re-deriving it. `wrapping_sub`
    /// mirrors the counter's own `wrapping` advance: a `u32` wire sequence
    /// really does wrap (~49.7 days at 1 kHz), and past the wrap the DIFFERENCE
    /// stays correct while either raw value alone does not.
    pub fn committed_frames(&self) -> u32 {
        self.sequence().wrapping_sub(self.initial_sequence)
    }

    /// Consume the next wire sequence number — called ONLY by the
    /// `OutputProxy::Drop` COMMIT paths (steady-state send + overflow
    /// re-loan send), which rewrite the header's `sequence` field alongside
    /// `total_size` before handing the frame to iceoryx2.
    ///
    /// Stamping at commit (not loan) means a tick that loans but never
    /// publishes (the collapse / early-exit / discard class)
    /// burns no sequence: published streams are gap-free, so bagd's
    /// wire-seq gap detector and the subscriber's `drop_oldest` eviction
    /// detector count only REAL losses, never the phantom ones loan-time
    /// stamping produced. A send that FAILS after this consume does burn its
    /// number: that frame was genuinely attempted and lost (loud `error!` at
    /// the send site), so the resulting gap is real.
    #[inline]
    pub(crate) fn commit_sequence(&self) -> u32 {
        self.sequence.fetch_add(1, Ordering::Relaxed)
    }

    /// Returns the maximum buffer size for this publisher (header + payload).
    /// `MaxSliceLen` — encodes both wire-format bounds.
    pub fn max_slice_len(&self) -> MaxSliceLen {
        self.max_slice_len
    }

    /// UNCONDITIONAL running total of output DISCARDS on this port (an
    /// incomplete-output tick — a missing declared variable field, or a failed
    /// staged nested-field flush — that skipped publish). Bumped on EVERY
    /// discard regardless of the flood-latch's log level, and NEVER reset on
    /// recovery, so a persistently-broken node whose loud head `error!` scrolled
    /// away and whose sustained discards are `debug!`-suppressed is still
    /// countable (Principle #3 queryability). Symmetric with the unconditional
    /// `NodeHandle::backpressure_*_count` counters. Reads the per-port
    /// [`OutputDiscardLatch`].
    ///
    /// This is the NODE-LOCAL surface (reachable from the node's own tick code
    /// via [`AnyPublisher::output_discard_count`](crate::graph::node::AnyPublisher::output_discard_count)).
    /// The OFF-THREAD OPERATOR reads the same count through the per-output
    /// `NodeHandle::output_discard_count` accessor, which the graph runtime wires
    /// to a shared mirror of this counter.
    pub fn output_discard_count(&self) -> u64 {
        self.discard_latch.total_discards()
    }
}

impl Drop for CerulionPublisher {
    fn drop(&mut self) {
        // Best-effort notification — Drop must not panic
        let _ = self
            .notifier
            .notify_with_custom_event_id(PubSubEvent::PublisherDisconnected.into());
        tracing::debug!(topic = %self.topic, "publisher dropped, sent PublisherDisconnected");
    }
}

/// Regime A: parse the `WireHeader` fields needed for a
/// `PublishTraceEntry` (schema_hash, sequence, timestamp_ns) from the
/// publish-bytes prefix. Returns `None` if the slice is shorter than
/// `WireHeader::SIZE` (32 B) — defensive against malformed publishes.
///
/// Alignment-safe: reads field bytes via `to_le_bytes`-style extraction,
/// no `unsafe` and no alignment requirement on the input slice. (A loaned
/// slot's `[u8]` payload IS 8-aligned on iceoryx2 0.9.1 — the per-sample
/// header is 40 B @ align 8 (`IOX2_SAMPLE_HEADER_BYTES`), so the payload
/// starts at chunk+40 of an 8-aligned chunk — but that is DE FACTO, not a
/// declared iceoryx2 contract: the rmw loan paths ASSERT it fail-closed
/// before handing out a typed pointer, and this parser simply does not
/// depend on it.)
pub(crate) fn parse_trace_entry_from_wire(topic: &str, data: &[u8]) -> Option<PublishTraceEntry> {
    if data.len() < WireHeader::SIZE {
        return None;
    }
    // WireHeader layout (matches wire.rs):
    //   schema_hash:       [0..8]   u64 LE
    //   total_size:        [8..12]
    //   offset_table_offset: [12..16]
    //   offset_table_count: [16..20]
    //   sequence:          [20..24] u32 LE
    //   timestamp_ns:      [24..32] u64 LE
    let schema_hash = u64::from_le_bytes(data[0..8].try_into().ok()?);
    let sequence = u32::from_le_bytes(data[20..24].try_into().ok()?);
    let timestamp_ns = u64::from_le_bytes(data[24..32].try_into().ok()?);
    Some(PublishTraceEntry {
        topic: Arc::from(topic),
        sequence,
        publish_time_ns: timestamp_ns,
        schema_hash,
    })
}

/// A self-contained raw SHM loan.
///
/// Holds the iceoryx2 sample alive while the rmw layer hands its payload
/// pointer across the FFI boundary. Dropping without sending releases the
/// slot back to the pool (the rmw "return loaned message" cancel path).
pub struct RawShmLoan {
    /// `Some` until sent; `Option` so `send_raw_loan` can consume the
    /// sample out of a by-value loan.
    sample: Option<iceoryx2::sample_mut::SampleMut<CerService, [u8], ()>>,
}

impl RawShmLoan {
    /// Mutable view of the loaned bytes.
    pub fn bytes_mut(&mut self) -> &mut [u8] {
        self.sample
            .as_mut()
            .expect("RawShmLoan accessed after send")
            .payload_mut()
    }

    /// Stable BASE address of the loaned bytes (frame start, including the
    /// WireHeader region). NOTE: the rmw layer keys pending loans by the
    /// PAYLOAD pointer it hands rclcpp (`base + WireHeader::SIZE`), not by
    /// this base address (the two differ by exactly the
    /// header size).
    pub fn as_ptr(&self) -> *const u8 {
        self.sample
            .as_ref()
            .expect("RawShmLoan accessed after send")
            .payload()
            .as_ptr()
    }
}

/// An UNINITIALIZED exact-size raw SHM loan (flatten-into-loan).
/// Produced by [`CerulionPublisher::loan_raw_uninit`]; see that method's
/// invariant: the holder must initialize EVERY byte before
/// [`Self::assume_init`], and drop (never send) on a partial fill. Dropping
/// releases the slot back to the pool without sending.
#[must_use = "an unsent loan holds a pool slot; fill + assume_init + send it, or drop it to release"]
pub struct RawShmLoanUninit {
    sample: iceoryx2::sample_mut_uninit::SampleMutUninit<CerService, [MaybeUninit<u8>], ()>,
}

impl RawShmLoanUninit {
    /// Mutable view of the (uninitialized) loaned bytes.
    pub fn bytes_uninit_mut(&mut self) -> &mut [MaybeUninit<u8>] {
        self.sample.payload_mut()
    }

    /// Promote to an initialized [`RawShmLoan`] (sendable via
    /// [`CerulionPublisher::send_raw_loan`]).
    ///
    /// # Safety
    /// Every byte of [`Self::bytes_uninit_mut`] must have been written since
    /// the loan was taken (iceoryx2's `assume_init` contract — consumers must
    /// observe only initialized memory). Callers should derive this from a
    /// checked encoder whose `Ok` value proves written-length == slot length
    /// (see the rmw `FrameCursor` proof chain at its `assume_init` call
    /// site), never from unchecked bookkeeping.
    #[must_use = "the promoted loan must be sent via send_raw_loan (or dropped to release the slot)"]
    pub unsafe fn assume_init(self) -> RawShmLoan {
        RawShmLoan {
            sample: Some(self.sample.assume_init()),
        }
    }

    /// Promote a loan whose WRITTEN REGIONS form a complete wire frame while
    /// the slot's remaining bytes ship as the mapped shared memory already
    /// held them (the rmw borrow-window publish, whose
    /// gap-frame layout deliberately leaves dead bytes inside `total_size`
    /// and slack beyond it).
    ///
    /// Distinct from [`Self::assume_init`], whose contract requires EVERY
    /// byte written and which callers must keep preferring wherever a
    /// checked encoder can prove full coverage. This variant exists for the
    /// one caller shape that cannot: a frame assembled around storage a
    /// third party (the borrow-window fill) placed in the slot, where the
    /// un-written remainder is intentional gap/slack.
    ///
    /// # Safety
    ///
    /// - The slot's backing is an iceoryx2 POSIX shared-memory mapping:
    ///   every byte has an OS-defined value (a zero-filled fresh page, or a
    ///   previously-published frame in a recycled pool slot). It is never
    ///   memory the Rust abstract machine considers uninitialized — it did
    ///   not come from the Rust allocator and is never deallocated under
    ///   Rust's rules — so reading it as `u8` is defined; the
    ///   `MaybeUninit` typing on this API is the DISCIPLINE layer that
    ///   forces callers to state which case they are.
    /// - The caller must have written a parseable [`WireHeader`] at
    ///   `[0, 32)` whose `total_size` is at most the loan length, and every
    ///   byte range a READER interprets (the wire head and every
    ///   offset-table-referenced range) must have been written — either by
    ///   the caller or by the in-slot fill it is adopting. Consumers slice
    ///   the frame on `total_size` (`OwnedInboundSample`), so bytes beyond
    ///   it are never served; un-written bytes inside it are the frame's
    ///   documented dead-gap bytes and are never interpreted by any reader
    ///   of a legal offset table.
    #[must_use = "the promoted loan must be sent via send_raw_loan (or dropped to release the slot)"]
    pub unsafe fn assume_init_shm_defined(self) -> RawShmLoan {
        RawShmLoan {
            sample: Some(self.sample.assume_init()),
        }
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
    use crate::abi_layout::abi_pin_struct;
    vec![abi_pin_struct!(CerulionPublisher {
        topic,
        publisher,
        notifier,
        listener,
        last_listener_count,
        self_drains_armed,
        sequence,
        initial_sequence,
        clock,
        max_slice_len,
        history_size,
        provisioned_history_size,
        sizer,
        trace,
        fault_inject_publish_raw_after,
        fault_inject_send_overflow_frame_after,
        fault_inject_send_raw_loan,
        frames_dropped_overflow,
        frames_dropped_invariant_violation,
        frames_dropped_send_fail,
        block_outstanding,
        promise_within_last_publish_ns,
        doorbell,
        event_service,
        notify_elision_expected,
        notify_elided_count,
        notify_gate_last_elided,
        notify_on_publish_raw,
        unannounced_publish,
        resweep_notify_count,
        notify_delivery_latch,
        notify_undelivered_shared,
        discard_signal,
        replay_suppress,
        fault_inject_loan_fail_next,
        discard_latch,
        output_discard_shared
    })]
}

#[cfg(test)]
mod notify_elision_gate_tests {
    //! Pin that the elision gate
    //! actually SKIPS the iceoryx2 notify — not merely that the elided
    //! COUNTER moves. Every integration test asserts the counter + delivery,
    //! but the counter `fetch_add` and the `return Ok(0)` sit in the same
    //! `if elide` block, so the counter is a proxy for the DECISION, not the
    //! SKIP: deleting the `return Ok(0)` (decide-but-still-notify) would still
    //! pass all four. This test is the one that catches it, at the notify
    //! boundary.
    //!
    //! The truthful skip-observable (VERIFIED against iceoryx2 0.9.1
    //! `Notifier::__internal_notify`): `notify_with_custom_event_id` returns
    //! `Ok(number_of_triggered_listeners)` — the count of listeners actually
    //! notified (disconnected connections are pruned, per-connection failures
    //! warn and do NOT count). The publisher's OWN event listener (minted by
    //! `finish_publisher` for SubscriberConnected/etc.) is itself a listener
    //! on the topic's event service, so a REAL notify here returns `Ok(>=1)`
    //! even with no other attachers — while the elided path returns `Ok(0)`.
    //! `Ok(0)` vs `Ok(n>=1)` is therefore a direct read of "did the iceoryx2
    //! notify run", independent of the counter.
    //!
    //! Real iceoryx2 event service over a per-test isolated SHM root
    //! (`iceoryx_test_config` — parallel-safe, no `#[serial]` needed).

    use super::*;
    use crate::transport::{TransportConfig, TransportManager};
    use std::sync::atomic::{AtomicU64, AtomicUsize};

    /// Drain a raw event listener, returning one entry per NOTIFY.
    ///
    /// iceoryx2 0.10 calls the drain callback once per DISTINCT event id,
    /// carrying how many times that id was activated since the last drain,
    /// where 0.9.1's socket queue held one datagram per notify and was popped
    /// one at a time. Expanding by `count` keeps these oracles counting
    /// notifies, which is what they are about; reading the number of CALLBACK
    /// invocations instead would silently collapse a burst to one.
    fn drain_notifies(listener: &Listener<CerService>) -> Vec<iceoryx2::prelude::EventId> {
        let mut seen = Vec::new();
        let _ = listener.try_wait(|activation| {
            for _ in 0..activation.count {
                seen.push(activation.id);
            }
        });
        seen
    }

    #[test]
    fn elision_gate_skips_the_real_notify_and_self_heals_at_the_boundary() {
        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "notify_elision_gate_test".into(),
                clock: Arc::new(crate::clock::RealClock),
                subscriber_buffer_size: 4,
                network: None,
            },
            crate::testing::iceoryx_test_config(),
        )
        .expect("per-test SHM transport");

        let topic = "neg/out";
        let mut publisher = mgr
            .create_publisher(
                topic,
                MaxSliceLen::try_new(1024).expect("1024 >= WireHeader::SIZE"),
                0,
            )
            .expect("create publisher (creator path — makes the event service)");

        // ANTI-TAUTOLOGY CONTROL (unarmed): the publisher's own listener is
        // the event service's only listener, and an UNARMED publisher always
        // notifies — the real notify reaches it. Proves the apparatus: a
        // notify that RUNS returns >= 1 here, so the Ok(0)s below can only
        // come from the skip.
        let unarmed = publisher
            .notify_sent_sample()
            .expect("unarmed notify must succeed");
        assert_eq!(
            unarmed, 1,
            "unarmed publisher notifies its own event listener (iceoryx2 returns \
             the number of listeners triggered)"
        );

        // Arm the gate: expected == the ONE listener this test proved exists
        // (the publisher's own). live == expected ⇒ elide.
        let elided = Arc::new(AtomicU64::new(0));
        publisher.arm_notify_elision(
            Arc::new(AtomicUsize::new(1)),
            Arc::clone(&elided),
            Arc::new(AtomicU64::new(0)),
        );

        // PHASE 1 (gated): the notify is SKIPPED — Ok(0), not Ok(1). Without
        // the `return Ok(0)`, the real notify runs and returns Ok(1).
        for i in 1..=3u64 {
            let n = publisher
                .notify_sent_sample()
                .expect("elided notify returns Ok");
            assert_eq!(
                n, 0,
                "gated (live == expected): the iceoryx2 notify must be SKIPPED — \
                 Ok(0) triggered listeners, not Ok(1) (decide-but-still-notify)"
            );
            assert_eq!(
                elided.load(Ordering::Relaxed),
                i,
                "the elided counter moves in lockstep with each skipped notify"
            );
        }

        // PHASE 2 (foreign listener attached): live = 2 != expected 1 ⇒ the
        // gate OPENS at this very notify — the real notify runs and triggers
        // BOTH listeners (own + foreign). The counter must NOT move.
        let foreign = mgr
            .create_trigger_listener_for_test(topic, mgr.default_topic_config())
            .expect("attach a foreign listener on the topic's event service");
        let n = publisher
            .notify_sent_sample()
            .expect("resumed notify returns Ok");
        assert!(
            n >= 1,
            "foreign listener present: notifies must RESUME (self-heal at the \
             notify boundary) — got Ok({n})"
        );
        assert_eq!(
            n, 2,
            "the resumed notify triggers BOTH live listeners (own + foreign) — \
             a full real notify, not a partial one"
        );
        assert_eq!(
            elided.load(Ordering::Relaxed),
            3,
            "the elided counter must NOT move while the gate is open (decision \
             tied to the skip, not to publish volume)"
        );

        // PHASE 3 (foreign detached): live back to 1 == expected ⇒ the skip
        // resumes — Ok(0) and the counter moves again.
        drop(foreign);
        let n = publisher
            .notify_sent_sample()
            .expect("re-elided notify returns Ok");
        assert_eq!(
            n, 0,
            "foreign listener dropped: live == expected again ⇒ the notify is \
             skipped again (re-elide)"
        );
        assert_eq!(
            elided.load(Ordering::Relaxed),
            4,
            "the elided counter resumes with the skip"
        );
    }

    /// The boundary re-check (`resweep_notify_elision`)
    /// fires a REAL notify to a FOREIGN listener OFF the publish path — the remedy
    /// for a quiescent producer / a producer publishing off the gate
    /// (`publish_raw`) leaving a `topic hz` blocked on the event listener
    /// forever — **exactly once per un-announced frame, never once per pass**.
    ///
    /// The two halves are pinned against each other in one body because they are
    /// one contract: a wake that never fires starves a `topic hz`, and a wake
    /// that fires on every silent boundary is the notify storm (198 notifies
    /// for 2 frames, and a live loop woken by its own boundary notify).
    ///
    /// What breaks this test: reverting the body of `resweep_notify_elision` to a
    /// bare `0` (never notify) fails the heal arms (the foreign listener never
    /// receives the `SentSample`); a "did not notify since the last boundary"
    /// trigger fails the SILENT-PASS arm (the resweep
    /// fire count climbs with the passes instead of staying at the frame count).
    /// Truthful observables: the foreign listener's own drained events, and
    /// the shared `resweep` fire counter the runtime exposes to tests.
    #[test]
    fn resweep_announces_each_unannounced_frame_once_and_a_silent_pass_never() {
        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "resweep_gate_test".into(),
                clock: Arc::new(crate::clock::RealClock),
                subscriber_buffer_size: 4,
                network: None,
            },
            crate::testing::iceoryx_test_config(),
        )
        .expect("per-test SHM transport");

        let topic = "resweep/out";
        let mut publisher = mgr
            .create_publisher(
                topic,
                MaxSliceLen::try_new(1024).expect("1024 >= WireHeader::SIZE"),
                0,
            )
            .expect("create publisher (creator path — makes the event service)");

        // Arm elision: expected == the ONE proved-owned listener (the
        // publisher's own). No foreign listener yet.
        let elided = Arc::new(AtomicU64::new(0));
        let fired = Arc::new(AtomicU64::new(0));
        publisher.arm_notify_elision(
            Arc::new(AtomicUsize::new(1)),
            Arc::clone(&elided),
            Arc::clone(&fired),
        );

        // CONTROL (no foreign listener, nothing published): the boundary sweep is
        // a NO-OP on both conjuncts. Returns 0, and the elided counter does NOT
        // move (the sweep is not a publish).
        assert_eq!(
            publisher.resweep_notify_elision(),
            0,
            "no foreign listener (live == expected): the boundary sweep must NOT notify"
        );
        assert_eq!(
            elided.load(Ordering::Relaxed),
            0,
            "the boundary sweep never touches the elided counter (it is not a publish)"
        );

        // A publish ELIDES (live == expected — only the publisher's own
        // listener). That is the debt: a frame in the SHM queue nobody was told
        // about. It must SURVIVE boundaries taken before a listener exists —
        // otherwise the late-attach guarantee is gone.
        assert_eq!(
            publisher
                .notify_sent_sample()
                .expect("post-send hook runs while gated"),
            0,
            "gated publish elides its notify (0 listeners reached)"
        );
        assert_eq!(elided.load(Ordering::Relaxed), 1, "one publish elided");
        for _ in 0..4 {
            assert_eq!(
                publisher.resweep_notify_elision(),
                0,
                "nobody foreign is waiting yet — the boundary must not fire, and must \
                 NOT spend the debt"
            );
        }

        // Attach a FOREIGN listener (a `topic hz`-shaped attacher). Drain the
        // events its own attach queued so the assert below reads ONLY the sweep's
        // notify.
        let foreign = mgr
            .create_trigger_listener_for_test(topic, mgr.default_topic_config())
            .expect("attach a foreign listener on the topic's event service");
        drain_notifies(&foreign);

        // HEADLINE (the LATE-ATTACH heal): the debt survived, the listener is
        // here, so the boundary announces it. The exact wake the per-publish path
        // CANNOT do — that publish already happened and elided.
        let triggered = publisher.resweep_notify_elision();
        assert!(
            triggered >= 1,
            "foreign listener attached after an elided publish: the boundary sweep MUST \
             fire the announcement (got {triggered})"
        );

        // The foreign listener actually received the SentSample event — the wake
        // that unblocks a `topic hz`. (A no-op resweep leaves this
        // `None`.)
        let seen = drain_notifies(&foreign);
        assert_eq!(
            seen,
            vec![crate::transport::events::PubSubEvent::SentSample.into()],
            "the boundary sweep's notify must reach the foreign listener, exactly once, \
             as a SentSample event (a data wake, not history)"
        );

        // The sweep is NOT a publish — the elided counter is still untouched.
        assert_eq!(
            elided.load(Ordering::Relaxed),
            1,
            "the boundary sweep fires a notify but is not a publish (elided unchanged)"
        );

        // **THE SILENT-PASS ARM.** The debt is SPENT. The runtime takes this boundary
        // on every `live_step` — ~1 kHz — and a producer with nothing new to say
        // must cost NOTHING. Without the debt gate every one of these passes fires a real
        // `notify_with_custom_event_id`, which is how 2 frames become 198 notifies
        // and how the notify reaches the topic's own in-graph consumer listener
        // and free-runs the graph's live loop.
        const SILENT_PASSES: usize = 200;
        let fired_after_heal = fired.load(Ordering::Relaxed);
        for _ in 0..SILENT_PASSES {
            assert_eq!(
                publisher.resweep_notify_elision(),
                0,
                "a silent boundary pass announces NOTHING — the debt was spent by the heal"
            );
        }
        assert_eq!(
            fired.load(Ordering::Relaxed),
            fired_after_heal,
            "{SILENT_PASSES} silent passes must fire ZERO boundary notifies (the \
             trigger is an un-announced frame, never a quiet pass)"
        );
        assert!(
            drain_notifies(&foreign).is_empty(),
            "and the foreign listener received nothing across all of them"
        );

        // A NEW un-announced frame re-arms the heal exactly once. Here via the
        // OTHER debt source: an off-gate `publish_raw` (this publisher is not
        // `notify_on_publish_raw`-armed — the graph-output shape), the
        // case-2 producer.
        // hot-path-alloc-ok: test-only fixture frame, built once outside any
        // measured window (this whole module is `#[cfg(test)]`).
        let mut frame = vec![0u8; WireHeader::SIZE];
        WireHeader::new(0xDEAD_BEEF, 7, 42).write_to_buf(&mut frame);
        publisher.publish_raw(&frame).expect("off-gate raw publish");
        assert!(
            publisher.resweep_notify_elision() >= 1,
            "a NEW un-announced frame (off-gate publish_raw) must be announced"
        );
        drain_notifies(&foreign);
        assert_eq!(
            publisher.resweep_notify_elision(),
            0,
            "and only once — the second boundary has nothing left to announce"
        );

        // The no-redundant-notify guard, restated on the debt: the per-publish path
        // notifies while a foreign listener is present (live > expected ⇒ no
        // elision), which CLEARS the debt — so the boundary adds nothing. This is
        // the steady state the cost model documents as one `sendto` per frame.
        let n = publisher
            .notify_sent_sample()
            .expect("per-publish notify (foreign present)");
        assert!(
            n >= 1,
            "per-publish path notifies while foreign present (got {n})"
        );
        drain_notifies(&foreign); // drain that wake
        assert_eq!(
            publisher.resweep_notify_elision(),
            0,
            "GUARD: the per-publish path already announced this frame ⇒ the boundary \
             sweep must SKIP (no redundant second notify)"
        );
        assert!(
            drain_notifies(&foreign).is_empty(),
            "the skipped boundary sweep fired NO notify to the foreign listener"
        );

        // Foreign detaches → back to live == expected → the sweep is a no-op again
        // even with a fresh debt outstanding (nobody is waiting to be told).
        drop(foreign);
        assert_eq!(
            publisher
                .notify_sent_sample()
                .expect("post-send hook runs while gated"),
            0,
            "with the foreign listener gone the gate re-elides"
        );
        assert_eq!(
            publisher.resweep_notify_elision(),
            0,
            "foreign gone (live == expected): the boundary sweep is a no-op again"
        );
    }

    /// An UNARMED publisher's boundary sweep is a strict no-op (returns
    /// 0, no notify) — the kill-switch (`CERULION_NOTIFY_ELISION=off`) and every
    /// non-graph publisher pay ZERO for the boundary re-check.
    #[test]
    fn resweep_is_a_noop_when_elision_is_not_armed() {
        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "resweep_unarmed_test".into(),
                clock: Arc::new(crate::clock::RealClock),
                subscriber_buffer_size: 4,
                network: None,
            },
            crate::testing::iceoryx_test_config(),
        )
        .expect("per-test SHM transport");

        let mut publisher = mgr
            .create_publisher(
                "resweep_unarmed/out",
                MaxSliceLen::try_new(1024).expect("1024 >= WireHeader::SIZE"),
                0,
            )
            .expect("create publisher");

        // NOT armed. Even attaching a foreign listener leaves the sweep inert —
        // no `notify_elision_expected` ⇒ no boundary notify at all.
        let foreign = mgr
            .create_trigger_listener_for_test("resweep_unarmed/out", mgr.default_topic_config())
            .expect("attach a foreign listener");
        drain_notifies(&foreign);
        assert_eq!(
            publisher.resweep_notify_elision(),
            0,
            "an unarmed publisher's boundary sweep is a strict no-op"
        );
        assert!(
            drain_notifies(&foreign).is_empty(),
            "the unarmed sweep fired NO notify to the foreign listener"
        );
        drop(foreign);
    }
}

#[cfg(test)]
mod notify_delivery_wiring_tests {
    //! Pins for the production seam that feeds the
    //! [`NotifyDeliveryLatch`] — `CerulionPublisher::record_notify_delivery`,
    //! the one function both notify sites ([`CerulionPublisher::notify_sent_sample`]
    //! and [`CerulionPublisher::resweep_notify_elision`]) call.
    //!
    //! The pure state machine is oracle-tested in
    //! [`super::super::notify_delivery_latch`]; what CANNOT be pinned there is
    //! the seam's own three decisions:
    //!
    //! 1. a caller-supplied (already-paid, pre-notify) count is ALWAYS
    //!    classified — the cost gate is consulted only when the read would be
    //!    NEW;
    //! 2. a count this seam reads ITSELF lands AFTER the notify and is therefore
    //!    reported as [`ListenerCountTiming::AfterNotify`], which is what makes
    //!    an attach race harmless; and
    //! 3. the timing is whatever the CALLER DECLARED — a supplied count does
    //!    not by itself mean `BeforeNotify`, so a future caller that reads its
    //!    count after notifying gets the persistence rule, not an immediate
    //!    `warn!` (the silent inversion [`NotifyListenerCount`] exists to stop).
    //!
    //! Real iceoryx2 event services over per-test isolated SHM roots
    //! (`iceoryx_test_config` — parallel-safe, no `#[serial]`).
    //!
    //! # Scope
    //!
    //! Three of these tests call `record_notify_delivery` DIRECTLY. Two of them
    //! hand-supply the listener count (`NotifyListenerCount::CallerRead`); the
    //! third — `self_read_count_absorbs_a_single_shortfall_then_confirms_a_persistent_one`,
    //! the `AfterNotify` vehicle — hand-supplies only `triggered` and passes
    //! `ReadHere` precisely so the seam reads the REAL live count itself. The
    //! hand-supplied count is deliberate, not a shortcut: the
    //! condition under test is "a listener is REGISTERED on the topic but the
    //! notify never reaches it, while `triggered` does not move" — a peer whose
    //! process died with its dynamic-config entry unreaped, or one already
    //! unreachable when it joined. Over real in-process iceoryx2 a freshly
    //! attached listener is always immediately reachable (pinned by
    //! `notify_elision_gate_tests::elision_gate_skips_the_real_notify_and_self_heals_at_the_boundary`,
    //! PHASE 2: attach + notify returns `Ok(2)`), so `triggered` rises in
    //! lockstep with `listeners` and the condition cannot be fabricated without
    //! killing a peer process. Everything else in these tests is real: a real
    //! publisher, its real event service, its real latch, and the real
    //! production function. The counts fed in are exactly the ones the armed
    //! elision gate / boundary resweep hand over on a live robot.

    use super::*;
    use crate::transport::{TransportConfig, TransportManager};
    use std::sync::atomic::{AtomicU64, AtomicUsize};

    fn test_manager(name: &str) -> Arc<TransportManager> {
        TransportManager::init_for_test(
            TransportConfig {
                node_name: name.into(),
                clock: Arc::new(crate::clock::RealClock),
                subscriber_buffer_size: 4,
                network: None,
            },
            crate::testing::iceoryx_test_config(),
        )
        .expect("per-test SHM transport")
    }

    fn test_publisher(mgr: &TransportManager, topic: &str) -> CerulionPublisher {
        mgr.create_publisher(
            topic,
            MaxSliceLen::try_new(1024).expect("1024 >= WireHeader::SIZE"),
            0,
        )
        .expect("create publisher (creator path — makes the event service)")
    }

    /// THE CONFIRMED-FINDING PIN: when the caller already paid the
    /// `number_of_listeners()` read, the notify is classified even though
    /// `triggered` is IDENTICAL to the last classified one.
    ///
    /// This is the class an unconditional cost gate swallows for zero saving: a
    /// listener registered on the topic that the notify never reaches, while
    /// `triggered` sits still — so `needs_classification` says "same as last
    /// time, healthy" forever and the operator counter never moves. Both
    /// callers that hand a count in are exactly the ones best placed to see it:
    /// the elision gate (armed on every graph publisher) and the
    /// boundary resweep (the ONLY observer of a quiescent producer's foreign
    /// listener).
    ///
    /// Restoring the unconditional
    /// `if !needs_classification(triggered) { return; }` at the top of
    /// `record_notify_delivery` leaves the count at 0 here and fails this test,
    /// while every counter-based test elsewhere stays green (they all move
    /// `triggered`).
    #[test]
    fn known_listener_count_is_always_classified_even_when_triggered_never_moves() {
        let mgr = test_manager("known_count_test");
        let publisher = test_publisher(&mgr, "ndw_known/out");

        // Seed a healthy classification through the REAL notify path: the
        // publisher's own listener is the topic's only listener, so this
        // notify reaches 1 of 1 and records `last_classified = 1`.
        let triggered = publisher.notify_sent_sample().expect("seed notify");
        assert_eq!(
            triggered, 1,
            "the publisher's own listener is the only one, and it is reachable"
        );
        assert_eq!(
            publisher.notify_undelivered_count(),
            0,
            "a fully delivered notify counts nothing"
        );
        // The gate would skip an identical repeat — that is the whole point.
        assert!(
            !publisher
                .notify_delivery_latch
                .needs_classification(triggered),
            "precondition: the cost gate WOULD skip this notify if asked"
        );

        // A second listener is now registered but unreachable (the dead-peer /
        // never-reached class), so the caller's pre-notify read sees 2 while the
        // notify still reaches the same 1 as every notify before it.
        publisher.record_notify_delivery(
            1,
            NotifyListenerCount::CallerRead(2, ListenerCountTiming::BeforeNotify),
        );
        assert_eq!(
            publisher.notify_undelivered_count(),
            1,
            "an already-paid listener count must be USED, not discarded: the gate \
             exists only to avoid the SHM read, and there is nothing to save here"
        );
        assert!(
            publisher.notify_delivery_latch.is_degraded(),
            "the regime opens at the first observation on a caller-supplied count \
             (an attaching listener cannot inflate a count read BEFORE the notify)"
        );

        // The regime then behaves normally: repeats counted, recovery closes it
        // and never resets the total.
        publisher.record_notify_delivery(
            1,
            NotifyListenerCount::CallerRead(2, ListenerCountTiming::BeforeNotify),
        );
        assert_eq!(publisher.notify_undelivered_count(), 2);
        publisher.record_notify_delivery(
            2,
            NotifyListenerCount::CallerRead(2, ListenerCountTiming::BeforeNotify),
        );
        assert!(!publisher.notify_delivery_latch.is_degraded());
        assert_eq!(
            publisher.notify_undelivered_count(),
            2,
            "recovery must never reset the operator total"
        );
    }

    /// THE SUSTAINED-PLAUSIBLE PIN (the other direction): a count this seam
    /// reads ITSELF lands AFTER the notify, so a listener that ATTACHES in that
    /// window fabricates a shortfall on a perfectly healthy publisher. One such
    /// observation must cost nothing — no `warn!`, no counter bump — and must
    /// resolve silently on the next notify.
    ///
    /// Every elision-UNARMED publisher takes this path: the raw / service / rmw
    /// publishers and the netd + gateway mirror re-inject publishers, whose
    /// consumers (vizd, `topic echo`, `topic hz`) attach and detach constantly.
    ///
    /// Reporting this path as `ListenerCountTiming::BeforeNotify`
    /// makes the first observation count and fails this test.
    #[test]
    fn self_read_count_absorbs_a_single_shortfall_then_confirms_a_persistent_one() {
        let mgr = test_manager("self_read_test");
        let publisher = test_publisher(&mgr, "ndw_selfread/out");

        // --- ARM A (attach race): one apparent shortfall, then healthy.
        // The seam reads the REAL live count (1 — the publisher's own listener),
        // and we report a notify that reached 0 of it.
        publisher.record_notify_delivery(0, NotifyListenerCount::ReadHere);
        assert_eq!(
            publisher.notify_undelivered_count(),
            0,
            "a single self-read shortfall is a suspicion, not a verdict — a \
             listener attaching in the notify→read window produces exactly this"
        );
        assert!(
            !publisher.notify_delivery_latch.is_degraded(),
            "no regime may open on one self-read observation"
        );
        // Resolved healthy on the next notify (1 of 1) — nothing counted, ever.
        publisher.record_notify_delivery(1, NotifyListenerCount::ReadHere);
        assert_eq!(publisher.notify_undelivered_count(), 0);
        assert!(!publisher.notify_delivery_latch.has_pending_shortfall());

        // --- ARM B (real saturation): the shortfall REPEATS, so it is believed.
        publisher.record_notify_delivery(0, NotifyListenerCount::ReadHere);
        assert_eq!(
            publisher.notify_undelivered_count(),
            0,
            "first observation of the new regime is still only a suspicion"
        );
        // The suspicion must force classification even though `triggered` is
        // unchanged and the latch is healthy — otherwise a saturated listener at
        // a steady publish rate is invisible forever.
        assert!(
            publisher.notify_delivery_latch.needs_classification(0),
            "an armed suspicion forces the next notify to be classified"
        );
        publisher.record_notify_delivery(0, NotifyListenerCount::ReadHere);
        assert_eq!(
            publisher.notify_undelivered_count(),
            1,
            "a shortfall that survives a second classified notify is real and counts"
        );
        assert!(publisher.notify_delivery_latch.is_degraded());
    }

    /// THE NO-INFERENCE PIN: the timing is whatever the CALLER DECLARED, never
    /// derived from the fact that a count was supplied.
    ///
    /// The two tests above pin the two arms as they exist in production today
    /// (`CallerRead(_, BeforeNotify)` counts at once; `ReadHere` absorbs the
    /// first shortfall) — but they would BOTH still pass if the seam went back
    /// to inferring the timing from whether a count was present, because the
    /// two current callers happen to read before notifying. This test drives the
    /// combination that inference cannot express: a caller-SUPPLIED count that
    /// was read AFTER its notify. It must get the persistence rule.
    ///
    /// That combination is not hypothetical bookkeeping — it is the exact shape
    /// a third notify site, or a refactor moving the elision gate's read below
    /// the notify, would produce. Under an `Option<usize>` seam it silently
    /// becomes `BeforeNotify`: a loud `warn!` plus a permanent counter bump on
    /// every attach race of the publisher, with no compile error and no failing
    /// test.
    ///
    /// What breaks this test (either direction): map `CallerRead(n, _) =>
    /// BeforeNotify` and arm A counts 1 instead of 0 here; map it to
    /// `AfterNotify` and
    /// `known_listener_count_is_always_classified_even_when_triggered_never_moves`
    /// fails instead.
    #[test]
    fn declared_timing_governs_classification_not_the_presence_of_a_count() {
        let mgr = test_manager("declared_timing_test");
        let publisher = test_publisher(&mgr, "ndw_declared/out");

        // Seed a healthy classification through the REAL notify path (reaches
        // the publisher's own listener, 1 of 1).
        assert_eq!(publisher.notify_sent_sample().expect("seed notify"), 1);
        assert_eq!(publisher.notify_undelivered_count(), 0);

        // ARM A: a SUPPLIED count declared as read AFTER the notify. A listener
        // attaching in that window inflates `listeners` without moving
        // `triggered`, so one observation must be absorbed as a suspicion.
        publisher.record_notify_delivery(
            1,
            NotifyListenerCount::CallerRead(2, ListenerCountTiming::AfterNotify),
        );
        assert_eq!(
            publisher.notify_undelivered_count(),
            0,
            "a supplied count declared AfterNotify must ride the persistence \
             rule — the timing comes from the caller, not from Option-ness"
        );
        assert!(
            !publisher.notify_delivery_latch.is_degraded(),
            "no regime may open on one AfterNotify observation, supplied or not"
        );
        assert!(
            publisher.notify_delivery_latch.has_pending_shortfall(),
            "the observation must still ARM the suspicion (absorbed, not dropped)"
        );

        // ARM B: it persists across a second classified notify ⇒ believed.
        publisher.record_notify_delivery(
            1,
            NotifyListenerCount::CallerRead(2, ListenerCountTiming::AfterNotify),
        );
        assert_eq!(
            publisher.notify_undelivered_count(),
            1,
            "a declared-AfterNotify shortfall that repeats is real and counts"
        );
        assert!(publisher.notify_delivery_latch.is_degraded());
    }

    /// The boundary resweep FEEDS the latch — end to end over real
    /// transport, with NO per-publish notify involved at any point.
    ///
    /// SCOPE: this pins that the resweep classifies its notify AT ALL.
    /// It does NOT discriminate WHICH count the resweep passes: once the
    /// foreign listener's socket is full every sweep falls short, so a variant
    /// passing `ReadHere` instead of its already-read count would merely defer
    /// the count by one sweep inside a 20_000-iteration loop and still pass.
    /// The already-paid-count decision is pinned by
    /// `known_listener_count_is_always_classified_even_when_triggered_never_moves`,
    /// and the declared-timing decision by
    /// `declared_timing_governs_classification_not_the_presence_of_a_count`.
    ///
    /// This is the site the resweep's own docs call the one that matters most
    /// for the signal: it runs only when a FOREIGN listener is present and the
    /// producer holds an UN-ANNOUNCED frame, so a saturated foreign listener on
    /// a producer that never notifies for itself is observable HERE AND NOWHERE
    /// ELSE.
    ///
    /// The debt-keyed trigger changed WHAT this loop must do, not what it pins: the resweep
    /// announces a DEBT, so each round has to create one. It does that the way
    /// the shape actually arises in production — an off-gate `publish_raw` on a
    /// publisher that is not `notify_on_publish_raw`-armed (the graph-output /
    /// DDS-bridge raw-route shape, the resweep docs' case 2). A round that merely swept
    /// again would now correctly do nothing.
    ///
    /// Truthful observable: the publisher's own listener is drained every round
    /// (so it can never be the one missing), the foreign listener never is, and
    /// the count moves only after the foreign listener's `AF_UNIX SOCK_DGRAM`
    /// socket fills.
    #[test]
    fn boundary_resweep_feeds_the_delivery_latch_on_a_quiescent_producer() {
        let mgr = test_manager("resweep_latch_test");
        let topic = "ndw_resweep/out";
        let mut publisher = test_publisher(&mgr, topic);

        // Arm elision: expected == the ONE proved-owned listener (our own).
        publisher.arm_notify_elision(
            Arc::new(AtomicUsize::new(1)),
            Arc::new(AtomicU64::new(0)),
            Arc::new(AtomicU64::new(0)),
        );

        // A foreign listener (a `topic hz`-shaped attacher) that NOBODY drains.
        let foreign = mgr
            .create_trigger_listener_for_test(topic, mgr.default_topic_config())
            .expect("attach a foreign listener on the topic's event service");

        assert_eq!(
            publisher.notify_undelivered_count(),
            0,
            "precondition: nothing undelivered before the sweeps start"
        );

        // Drive ONLY the boundary sweep — the per-publish notify path never runs
        // (an off-gate `publish_raw` on this publisher notifies nobody; it only
        // records the debt). Each sweep therefore announces that debt (live 2 >
        // expected 1); our own listener is drained every round, the foreign one
        // is not.
        // hot-path-alloc-ok: test-only fixture frame, built once outside any
        // measured window (this whole module is `#[cfg(test)]`).
        let mut frame = vec![0u8; WireHeader::SIZE];
        WireHeader::new(0xFEED_FACE, 0, 0).write_to_buf(&mut frame);
        let mut swept = 0_usize;
        let mut saturated = false;
        for _ in 0..20_000 {
            publisher.check_subscriber_events();
            publisher
                .publish_raw(&frame)
                .expect("off-gate raw publish (creates the un-announced debt)");
            let triggered = publisher.resweep_notify_elision();
            swept += 1;
            assert!(
                triggered >= 1,
                "the boundary sweep must fire a real notify while a foreign \
                 listener is present and a frame is un-announced"
            );
            if publisher.notify_undelivered_count() > 0 {
                saturated = true;
                break;
            }
        }

        assert!(
            saturated,
            "after {swept} boundary sweeps with the foreign listener never drained, \
             its event socket must be full and the resweep's notify must start \
             falling short — got 0, so either the platform's socket is unexpectedly \
             unbounded or the resweep no longer feeds the delivery latch"
        );
        assert!(
            publisher.notify_delivery_latch.is_degraded(),
            "the resweep's shortfall opens a real regime, not a silent count"
        );
        drop(foreign);
    }
}
