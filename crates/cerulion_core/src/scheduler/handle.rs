// SPDX-License-Identifier: AGPL-3.0-only
//! Observable handles and configuration for scheduled nodes.
//!
//! Provides `NodeConfig` for adding nodes and `NodeHandle` for lock-free
//! observation of node state (Principle #3: Observable state).

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, RwLock};

use indexmap::IndexMap;

use crate::graph::node::BackpressurePolicy;

use super::trigger::TriggerPolicy;

/// Per-input backpressure counters.
///
/// One instance per (node, input) pair. Three atomics correspond to the
/// three `BackpressurePolicy` variants; a fourth, `block_defer_regimes_count`,
/// counts `block` defer REGIMES (state transitions, not steps). Counter
/// semantics:
///
/// * `drop_oldest_count` — number of times the subscriber's per-input
///   buffer was at capacity AND policy was `DropOldest`, causing the
///   oldest queued message to be evicted to make room for the new one.
/// * `block_fires_deferred_count` — number of times a `step()` call
///   invoked the pre-fire defer path for this input. NO DATA WAS LOST
///   — the producer's tick was skipped (or, for Period, the catch-up
///   was deferred). Note: this is "number of step()s that deferred,"
///   not "number of producer tick instances skipped" — a Period
///   producer at interval=10ms that's deferred over a 50ms step
///   bumps this counter by 1, NOT 5 (the 5 missed fires are
///   batched into a single defer decision per step).
/// * `sampled_count` — number of times a message was dropped at the
///   subscriber's input buffer because it arrived within N ms of the
///   last accepted message (the `Sample(N)` policy).
/// * `block_defer_regimes_count` — number of times the producer-side
///   `block` defer edge went from re-armed to firing, i.e. how many loud
///   regime-opening warns it owed. Bumped at the TRANSITION, never at the
///   emission (see the field's own doc).
///
/// The three policy counters are independent: bumping one does not affect
/// the others. `block_defer_regimes_count` is not: every regime open is
/// also a deferred step, so at quiescence it is `<=`
/// `block_fires_deferred_count`. Like the per-node QoS miss counters, but
/// indexed per-input (by input field name) rather than per-node.
///
/// **Field-visibility note:** the four atomic counters are `pub` so
/// external holders of an `Arc<BackpressureCounters>` (e.g. from
/// `Scheduler::register_backpressure_input_with`) can read them lock-free.
/// Every bump routes through one of TWO sanctioned writers: the canonical
/// `record_backpressure_event_n` (and its single-event wrapper
/// `record_backpressure_event`), the single point that translates a
/// `BackpressurePolicy` variant into the right event-counter bump +
/// structured `tracing::warn!`; and `Self::open_block_regime` for
/// `block_defer_regimes_count`, which `GraphRuntime`'s producer-side block
/// edge calls at the arm→fire transition — deliberately OUTSIDE the emission
/// path, so the count stays what the loud regime-opening warns are compared
/// against where the sustained `debug!` repeats are compiled out. Direct
/// external `fetch_add` would skip the warn or the transition semantics;
/// reads should use the `NodeHandle::backpressure_*_count(input)` accessors.
/// Adding a new field here requires care: per `#[non_exhaustive]`,
/// downstream constructors must initialize via `..Default::default()`.
#[derive(Default)]
#[non_exhaustive]
pub struct BackpressureCounters {
    pub drop_oldest_count: AtomicU64,
    pub block_fires_deferred_count: AtomicU64,
    pub sampled_count: AtomicU64,
    /// Number of `block` defer REGIMES the producer-side edge opened for this
    /// input: bumped when the edge goes from re-armed to firing — at the state
    /// transition, never at the log emission — so it is what the regime-opening
    /// WARN count is compared against where the sustained `debug!` repeats are
    /// compiled out (`release_max_level_info`). Two atomics, no transaction: a
    /// concurrent reader may transiently see this one ahead of
    /// `block_fires_deferred_count`; at quiescence it is `<=` that count.
    pub block_defer_regimes_count: AtomicU64,
}

impl BackpressureCounters {
    /// Construct a fresh counter set with all fields at zero.
    pub fn new() -> Self {
        Self::default()
    }

    /// The sanctioned writer for `block_defer_regimes_count`: called by the
    /// producer-side block edge exactly when it goes from re-armed to firing
    /// (`armed.swap(false)` returned `true`), before and independent of the
    /// warn it then owes — never from a log path.
    pub(crate) fn open_block_regime(&self) {
        self.block_defer_regimes_count
            .fetch_add(1, Ordering::AcqRel);
    }
}

/// Structured event surfaced to `#[on_event(input = "...")]` user
/// callbacks (or manually via `ctx.take_backpressure_event("input_name")`).
/// One event per regime (edge-triggered via the per-channel `armed`
/// latch — the first event after the policy goes quiet fires; subsequent
/// events in the same regime are silent until it clears and rearms).
///
/// Fires for ANY backpressure policy that triggers — branch on `policy`
/// (and read `dropped`) to tell them apart:
/// - `Sample(N)`: a read-gate decimation. `dropped` = messages decimated.
/// - `DropOldest`: iceoryx2 evicted the oldest sample(s) on queue overflow,
///   detected by the subscriber via a wire-sequence gap. `dropped` =
///   messages evicted.
/// - `Block`: the consumer's queue reached the defer threshold, so the
///   producer was deferred on its behalf. **Lossless — `dropped == 0`** (a
///   flow-control signal, not a data-loss one).
///
/// Carries enough context for a user callback to act without re-querying
/// counters. `count_total` matches the value
/// `NodeHandle::backpressure_<variant>_count(input)` would return AT THE
/// TIME the event was queued.
///
/// `#[non_exhaustive]` so future fields (e.g. publisher node-id,
/// source topic, wall-clock timestamp) can be added without breaking
/// downstream matchers. External construction is not supported —
/// instances are produced by the transport layer at the channel push
/// point.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct BackpressureEvent {
    /// Which policy fired.
    pub policy: BackpressurePolicy,
    /// Input field name (matches `#[input(...)]` field identifier).
    /// `Arc<str>` for cheap clone on dispatch.
    pub input_name: Arc<str>,
    /// Cumulative drop/defer count on this input AT EVENT TIME.
    /// Matches `NodeHandle::backpressure_<variant>_count(input)`.
    pub count_total: u64,
    /// Trigger occurrences in the regime-OPENING drain (events fire once
    /// per regime, at open — there is no mid-regime re-emit). Per policy:
    /// `Sample(N)` = 1 (the first decimation), `DropOldest` = samples
    /// evicted by the opening drain (so `== dropped`), `Block` = 1 (NOT a
    /// loss — for `Block`, `dropped == 0`). Read `dropped` for the
    /// data-loss count.
    pub count_in_regime: u64,
    /// Number of messages this event represents as LOST. For `sample(N)`
    /// and `drop_oldest` this equals the messages dropped (decimated /
    /// evicted) in the current regime; for `block` it is always `0` —
    /// block defers the producer and loses nothing, so its event is a
    /// flow-control signal, not a data-loss one. Read this instead of
    /// inferring loss from `policy`.
    pub dropped: u64,
    /// Clock timestamp (ns) when the current regime started. For the
    /// `Sample` policy this matches the publish time of the sampled
    /// message; for `DropOldest` and `Block` it is the regime-opening
    /// drain's high-water frame's wire timestamp (on a multi-publisher
    /// topic, whichever stream's clock wins the max — NOT monotonic
    /// across regimes when publisher clocks skew). `0` when the opening
    /// drain carried no readable wire timestamp (corrupt/undersized
    /// frames).
    pub regime_started_at_ns: u64,
    /// Configured buffer capacity for this input. Useful for sizing
    /// recovery decisions in user callbacks.
    pub buffer_capacity: usize,
}

/// Reactable QoS watchdog event for a per-input
/// `#[input(expect_within_ms = N)]` window that elapsed without fresh
/// data. The counter twin (`NodeHandle::expect_within_missed_count`)
/// bumps on EVERY missed window; this EVENT is **edge-triggered** —
/// it fires once per silence regime (the first miss after data goes
/// quiet) and rearms when a real arrival resets the window. Drain it
/// manually via [`crate::graph::node::NodeContext::take_expect_within_event`]
/// (the permanent escape hatch the `#[on_event]` macro
/// calls).
///
/// Every field is derived from the scheduler clock or wire data — no
/// `Instant`/wall-clock read — so an event captured under replay is
/// bit-identical to the live run (Principle #7).
///
/// `#[non_exhaustive]` so future fields can be added without breaking
/// downstream matchers; external construction is not supported (the
/// scheduler mints instances at the miss site).
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct ExpectWithinEvent {
    /// Input field name (matches the `#[input(...)]` field identifier).
    /// `Arc<str>` for cheap clone on dispatch.
    pub input_name: Arc<str>,
    /// The configured window in ms (`#[input(expect_within_ms = N)]`).
    pub expect_within_ms: u64,
    /// Milliseconds elapsed since the last data on this input, AT THE
    /// step that detected the miss. Always `> expect_within_ms`.
    pub elapsed_ms: u64,
    /// Cumulative `expect_within_missed` count for the node AT EVENT TIME
    /// (matches `NodeHandle::expect_within_missed_count` at the moment the
    /// event was queued).
    pub count_total: u64,
    /// Scheduler-clock timestamp (ns) of the `step()` that detected the
    /// miss (the advanced `current_time`). Deterministic — never a wall
    /// read.
    pub missed_at_ns: u64,
}

/// Reactable QoS watchdog event for a per-output
/// `#[output(promise_within_ms = N)]` window that elapsed without a
/// publish. The output twin of [`ExpectWithinEvent`] — same
/// edge-triggered (once-per-regime, rearm-on-publish) semantics, same
/// determinism guarantee, drained via
/// [`crate::graph::node::NodeContext::take_promise_within_event`].
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct PromiseWithinEvent {
    /// Output field name (matches the `#[output(...)]` field identifier).
    pub output_name: Arc<str>,
    /// The configured window in ms (`#[output(promise_within_ms = N)]`).
    pub promise_within_ms: u64,
    /// Milliseconds elapsed since the last publish on this output, AT THE
    /// step that detected the miss. Always `> promise_within_ms`.
    pub elapsed_ms: u64,
    /// Cumulative `promise_within_missed` count for the node AT EVENT TIME.
    pub count_total: u64,
    /// Scheduler-clock timestamp (ns) of the `step()` that detected the
    /// miss. Deterministic.
    pub missed_at_ns: u64,
}

/// Liveliness transition state for a per-input
/// `#[on_event(input = "...")]` handler bound to a [`LivelinessEvent`].
///
/// `Alive` means a publisher is (re)connected on the input's topic;
/// `Lost` means the last publisher disconnected. The pair is the
/// edge — the event fires on the TRANSITION, not on every step.
///
/// `#[non_exhaustive]` so future states (e.g. a degraded/partial
/// variant) can be added without breaking downstream matchers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum LivelinessState {
    /// At least one publisher is connected on the input's topic.
    Alive,
    /// The last publisher on the input's topic disconnected.
    Lost,
}

/// Why a [`LivelinessEvent`] fired. Minimal for
/// now — the real crash-detection producer (which will distinguish
/// crash vs graceful shutdown vs timeout) is not implemented yet; for
/// now only the connect/disconnect edges exist.
///
/// `#[non_exhaustive]` so the richer causes can be added later without
/// breaking downstream matchers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum LivelinessCause {
    /// A publisher on the input's topic disconnected (drives a
    /// [`LivelinessState::Lost`] transition).
    PublisherDisconnected,
    /// A publisher on the input's topic (re)connected (drives a
    /// [`LivelinessState::Alive`] transition).
    PublisherConnected,
}

/// Reactable liveliness event for a per-input
/// `#[input(...)]` port whose publisher set changed (a publisher
/// connected or the last one disconnected). Input-scoped — a handler
/// binds it via `#[on_event(input = "...")]` with a [`LivelinessEvent`]
/// parameter, type-routed to
/// [`crate::graph::node::NodeContext::take_liveliness_event`] (the
/// permanent escape hatch the `#[on_event]` macro calls).
///
/// Every field is derived from the scheduler clock or data — no
/// `Instant`/wall-clock read — so an event captured under replay is
/// bit-identical to the live run (Principle #7).
///
/// `#[non_exhaustive]` so future fields can be added without breaking
/// downstream matchers; external construction is not supported (the
/// scheduler mints instances at the transition site).
///
/// # Dispatch & observability limitations
///
/// A `LivelinessEvent` handler dispatches on the consuming node's tick
/// Ok-path (like every `#[on_event]` handler), so a purely data-triggered
/// node — which stops ticking once its input goes silent — will NOT fire its
/// `Lost` handler until it next ticks. The always-on observable for a
/// disconnect is the
/// [`NodeHandle::publisher_disconnects_observed_count`](crate::scheduler::NodeHandle::publisher_disconnects_observed_count)
/// counter, which the runtime sweep bumps regardless of whether the node
/// ticks. Use a `period`/`external` trigger (or another live input) if you
/// need the handler itself to fire on a disconnect.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct LivelinessEvent {
    /// Input field name (matches the `#[input(...)]` field identifier).
    /// `Arc<str>` for cheap clone on dispatch.
    pub input_name: Arc<str>,
    /// The liveliness state AFTER the transition this event represents.
    pub state: LivelinessState,
    /// Why the transition happened (connect vs disconnect).
    pub cause: LivelinessCause,
    /// Cumulative count of liveliness transitions on this input AT EVENT
    /// TIME.
    pub count_total: u64,
    /// Scheduler-clock timestamp (ns) of the `step()` that detected the
    /// change (the advanced `current_time`). Deterministic — never a wall
    /// read.
    pub changed_at_ns: u64,
    /// Number of live publishers on this input's topic at detection. Carries
    /// the raw live publisher count; with multiple publishers on the topic,
    /// `Lost` fires only when the LAST one leaves (count → 0).
    pub publisher_count: usize,
}

impl LivelinessEvent {
    /// Test-only: mint a `LivelinessEvent` for the e2e
    /// `#[on_event]` dispatch tests. `LivelinessEvent` is `#[non_exhaustive]`,
    /// so an integration test (a separate crate) cannot use a struct literal —
    /// this constructor is the seam, mirroring how the scheduler will mint
    /// instances at the real transition site. Not part of the
    /// user API.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn new_for_test(
        input_name: Arc<str>,
        state: LivelinessState,
        cause: LivelinessCause,
        count_total: u64,
        changed_at_ns: u64,
        publisher_count: usize,
    ) -> Self {
        Self {
            input_name,
            state,
            cause,
            count_total,
            changed_at_ns,
            publisher_count,
        }
    }
}

/// Per-node store for the edge-triggered QoS
/// watchdog events (`ExpectWithinEvent` / `PromiseWithinEvent`) and the
/// liveliness-transition events (`LivelinessEvent`). One
/// `Arc<QosEventStore>` is minted per node at graph build and shared
/// between the scheduler's `ScheduledNode` (which `push_*`es on a
/// watchdog miss inside `step()`) and the node's `NodeContext` (which
/// `take_*`es on demand from the tick body). The liveliness slot's
/// production `push_liveliness` site (the `step()` transition detector)
/// is not implemented yet; until then it is fed only by the scheduler
/// test seam.
///
/// Off the hot path entirely: the `Mutex` guards are acquired only on a
/// watchdog miss (`push_*`, in `step()`) or a manual drain (`take_*`,
/// in the node tick) — never on the zero-copy receive/publish path. The
/// scheduler drives `step()` single-threaded, so the locks are
/// uncontended; `Mutex` (over `RefCell`) is required because the store
/// is held inside the `Send` `NodeContext` and `Send` `ScheduledNode`
/// (mirrors the `Arc<RwLock<..>>` on the backpressure counters).
///
/// Each side keeps at most one pending event per port (edge-triggered,
/// one-per-regime). If a new regime's event is pushed before the
/// previous one is drained, the newer event replaces it — newest wins
/// (mirrors the single-slot `pending_backpressure_event` overwrite).
#[derive(Default)]
pub(crate) struct QosEventStore {
    expect: Mutex<IndexMap<Arc<str>, ExpectWithinEvent>>,
    promise: Mutex<IndexMap<Arc<str>, PromiseWithinEvent>>,
    liveliness: Mutex<IndexMap<Arc<str>, LivelinessEvent>>,
}

impl QosEventStore {
    /// Queue an edge-triggered input-watchdog event, keyed by input name
    /// (newest-wins on an undrained slot). Called from `step()` on a miss.
    pub(crate) fn push_expect(&self, event: ExpectWithinEvent) {
        if let Ok(mut map) = self.expect.lock() {
            map.insert(Arc::clone(&event.input_name), event);
        }
    }

    /// Drain the pending input-watchdog event for `name`, if any.
    pub(crate) fn take_expect(&self, name: &str) -> Option<ExpectWithinEvent> {
        self.expect.lock().ok()?.swap_remove(name)
    }

    /// Queue an edge-triggered output-watchdog event, keyed by output
    /// name (newest-wins). Called from `step()` on a miss.
    pub(crate) fn push_promise(&self, event: PromiseWithinEvent) {
        if let Ok(mut map) = self.promise.lock() {
            map.insert(Arc::clone(&event.output_name), event);
        }
    }

    /// Drain the pending output-watchdog event for `name`, if any.
    pub(crate) fn take_promise(&self, name: &str) -> Option<PromiseWithinEvent> {
        self.promise.lock().ok()?.swap_remove(name)
    }

    /// Queue an edge-triggered liveliness event, keyed by
    /// input name (newest-wins on an undrained slot). Mirrors
    /// `push_expect`/`push_promise`. The runtime wires the real
    /// producer: `GraphRuntime::liveliness_sweep` (the runtime's
    /// publisher-count poll) mints + pushes a `LivelinessEvent` here on every
    /// detected `Alive`/`Lost` transition, in addition to the scheduler test
    /// seam (`push_liveliness_event_for_test`). Ungated accordingly.
    pub(crate) fn push_liveliness(&self, event: LivelinessEvent) {
        if let Ok(mut map) = self.liveliness.lock() {
            map.insert(Arc::clone(&event.input_name), event);
        }
    }

    /// Drain the pending liveliness event for `name`, if any.
    pub(crate) fn take_liveliness(&self, name: &str) -> Option<LivelinessEvent> {
        self.liveliness.lock().ok()?.swap_remove(name)
    }
}

/// Configuration for adding a node to the scheduler.
pub struct NodeConfig {
    pub id: String,
    pub policy: TriggerPolicy,
    pub callback: Box<dyn FnMut() + Send>,
}

/// Observable handle to a scheduled node (Principle #3).
///
/// All reads are lock-free via atomics. Safe to read from any thread
/// while the scheduler is running. Uses `Acquire` ordering on loads to
/// ensure visibility of stores from `step()` (which uses `Release`).
#[derive(Clone)]
pub struct NodeHandle {
    id: String,
    fire_count: Arc<AtomicU64>,
    last_fire_ns: Arc<AtomicU64>,
    panic_count: Arc<AtomicU64>,
    pending_data_count: Arc<AtomicU64>,
    /// Subscriber-side miss counter: per-input
    /// `#[input(expect_within_ms = N)]` exceeded by elapsed time without
    /// new data. The scheduler ticks this when `now - last_data_on_input >
    /// deadline`.
    expect_within_missed: Arc<AtomicU64>,
    /// Publisher-side miss counter: per-output
    /// `#[output(promise_within_ms = N)]` exceeded by elapsed time without
    /// a publish on that output. Per the design, this is observed by
    /// the publisher itself (which tracks last_publish_time_ns per
    /// output); for now exposed via the same NodeHandle counter so
    /// downstream observers see one canonical "publisher missed
    /// commitment" signal.
    promise_within_missed: Arc<AtomicU64>,
    /// Tick-budget miss counter: tick execution time exceeded
    /// `#[cerulion_node(tick_within_ms = N)]`. The scheduler wraps
    /// the tick callback with timing and increments this when the
    /// elapsed wall time exceeds N ms.
    tick_within_missed: Arc<AtomicU64>,
    /// `expect_within_ms` windows that lapsed on the node's FIFO
    /// TRIGGER input while that input still carried unconsumed signalled
    /// arrivals — BACKLOG, not silence. Disjoint from
    /// `expect_within_missed`: a lapsed window lands in exactly one bucket.
    expect_within_backlogged: Arc<AtomicU64>,
    /// The gating-clock stamp of the fire this node is CURRENTLY
    /// executing, or `0` when it is not inside a tick.
    ///
    /// The Principle-#3 in-process surface for the one condition
    /// `tick_within_missed` above structurally cannot see: that counter is bumped
    /// AFTER the callback returns, so a tick that never returns increments
    /// nothing and is reported by nobody. Read
    /// [`in_tick_since_ns`](Self::in_tick_since_ns), which decodes the sentinel.
    in_tick_since_ns: Arc<AtomicU64>,
    /// Per-input backpressure counters, indexed by
    /// input field name. Registered exactly-once at graph-build time —
    /// `Scheduler::register_backpressure_input_with` inserts an entry for
    /// EVERY graph-wired input (the default `drop_oldest` registers too,
    /// via the runtime's else arm — its eviction detector is real).
    /// Reads via the `backpressure_*_count(input_name)` accessors below
    /// return 0 for an unknown input name OR for a registered input with
    /// no events recorded yet — a 0 means "nothing counted", never "no
    /// policy configured".
    ///
    /// `RwLock` guards the index-map mutation at registration time
    /// (typically only during graph build); reads are lock-free on the
    /// inner `Arc<BackpressureCounters>` atomics after the lookup.
    /// Insertions stop after graph build, so contention is negligible.
    backpressure: Arc<RwLock<IndexMap<Arc<str>, Arc<BackpressureCounters>>>>,
    /// Per-node count of publisher-loss transitions
    /// observed on this node's inputs by the runtime liveliness sweep
    /// (every `Lost` transition — a publisher disconnecting or crashing
    /// are indistinguishable via `number_of_publishers()`). Bumped by
    /// `GraphRuntime::liveliness_sweep` on each `Alive → Lost` edge; the
    /// `Alive` edge does NOT bump it. Per-node (no input arg), mirroring
    /// `expect_within_missed`.
    publisher_disconnects_observed: Arc<AtomicU64>,
    /// Per-node count of `signal_*` calls the
    /// scheduler REJECTED for this node because the call did not match the
    /// node's trigger policy (a wiring desync). Bumped in the wrong-policy
    /// `Err` arms of `Scheduler::signal_data` and `Scheduler::signal_sync_input`
    /// (the latter also counts a not-declared sync input). `signal_input_received`
    /// has no such arm (it no-ops for unconfigured inputs), and the
    /// `NodeNotFound` `?` paths cannot attribute to a node, so neither bumps
    /// this. Per-node (no input arg), mirroring `panic_count`.
    ///
    /// This is a RAW per-call count, **not** latched. A persistent desync
    /// climbs it at the input data rate — e.g. `GraphRuntime`'s sync drain
    /// calls `signal_sync_input` once per drained message, so a stuck keyspace
    /// mismatch increments this every message even while the runtime's own
    /// `warn!` is suppressed to one-per-regime by its per-binding
    /// `signal_warn_suppressed` latch. That latch is a *log*-flood guard owned
    /// by the runtime; the counter is one cheap atomic add (no flood to
    /// suppress) and the scheduler cannot see the runtime's latch state, so the
    /// two intentionally diverge. Read the value as a desync *rate/volume*, NOT
    /// as a count of distinct desync regimes. Reads via
    /// `Self::signal_failed_count`.
    signal_failed: Arc<AtomicU64>,
    /// Per-OUTPUT count of incomplete-output DISCARDS (a tick
    /// releasing its loan without writing all declared variable fields, or a
    /// failed staged nested-field flush — a publish that was skipped), indexed by
    /// output field name. Each entry ALIASES the `Arc<AtomicU64>` the graph
    /// runtime also installed on the port's `CerulionPublisher`
    /// (`register_output_discard_count`), which stores the per-port
    /// `OutputDiscardLatch::total_discards` into it on every discard. This makes
    /// the discard count observable off-thread (Principle #3) — the latch itself
    /// lives on the publisher, reachable only through the node's own
    /// `AnyPublisher`. Registered per output at graph build via
    /// `Scheduler::register_output_discard`. Reads via
    /// `Self::output_discard_count(output_name)` return 0 for an unknown output
    /// OR a registered output with no discard yet — a 0 means "nothing
    /// discarded", never "no such port". Mirrors the per-input `backpressure`
    /// map's `RwLock`-guarded registration + lock-free atomic reads.
    output_discards: Arc<RwLock<IndexMap<Arc<str>, Arc<AtomicU64>>>>,
    /// Per-OUTPUT count of `SentSample` notifies that did not reach
    /// every live listener on that output's topic, indexed by output field name.
    /// Each entry ALIASES the `Arc<AtomicU64>` the graph runtime also installed
    /// on the port's `CerulionPublisher` (`register_notify_undelivered_count`),
    /// which stores the per-port `NotifyDeliveryLatch::total_undelivered` into it
    /// on every classified notify. Exactly the `output_discards` shape above,
    /// for the notify-delivery signal: the latch lives on the publisher and is otherwise
    /// reachable only from the node's own tick code, so without this an operator
    /// could not see a degraded wake path at all (Principle #3). Registered per
    /// output at graph build via `Scheduler::register_notify_undelivered`. Reads
    /// via `Self::notify_undelivered_count(output_name)` return 0 for an unknown
    /// output OR a registered output with nothing undelivered.
    notify_undelivered: Arc<RwLock<IndexMap<Arc<str>, Arc<AtomicU64>>>>,
    /// The per-set Sync observability bundle. One `Arc` rather than
    /// five constructor arguments, on the `BackpressureCounters` precedent —
    /// and one bundle rather than a scattering, because the two skip counters
    /// only MEAN anything read together (see [`SyncCounters`]).
    ///
    /// Every node carries one; on a non-Sync node it stays at its defaults, so
    /// a reader never has to ask whether the accessor applies before calling
    /// it (a `0` from a Period node is accurate — nothing was skipped).
    sync: Arc<SyncCounters>,
}

/// Per-set Sync alignment observability (Principle #3).
///
/// # The two skip classes are SEPARATE on purpose
///
/// Per-set Sync skips frames for two reasons that look identical from the data
/// plane and could not be more different to an operator:
///
/// * **UNMATCHABLE** ([`Self::unmatched_discards`]) — a frame provably in NO
///   set, because a partner ran more than `sync_window_ms` past it. Something
///   IS wrong: widen the window, fix a stalled or skewed producer, or raise the
///   fast input's `depth` above the fast:slow rate ratio. On a healthy VIO node
///   this is 0.
/// * **PASSED-OVER** ([`Self::closer_skips`]) — a nearer arrived member of the
///   same stream was chosen for the set. This is the FEATURE WORKING, and on a
///   200/20 Hz pair it runs at roughly 180 per second.
///
/// Merging them into one "skipped" number would make the healthy rate drown the
/// diagnostic one, so a real fault would read as a slightly larger number on a
/// counter that is always large. It is also why only the UNMATCHABLE class is
/// loud in the log (a flood-latched head naming node/input/topic/stamps/window
/// and the `depth` remedy) while PASSED-OVER is counter + `debug!` only — a
/// loud head at 180 lines per second is the disk-fill class.
///
/// Neither routes through `record_backpressure_event_n`: an alignment skip is
/// not backpressure, and fabricating `BackpressureEvent`s for it would fire
/// user `#[on_event]` handlers for a condition they did not subscribe to.
#[derive(Default)]
pub struct SyncCounters {
    /// Per trigger input (keyed by the RESOLVED SOURCE TOPIC, the space
    /// `TriggerPolicy::Sync::inputs` uses): frames a nearer arrived member of
    /// the same stream displaced. Unconditional, never reset.
    pub closer_skips: RwLock<IndexMap<Arc<str>, Arc<AtomicU64>>>,
    /// Per trigger input: frames discarded as provably unmatchable (window
    /// death). Unconditional, never reset.
    pub unmatched_discards: RwLock<IndexMap<Arc<str>, Arc<AtomicU64>>>,
    /// Per trigger input: `signal_sync_input` offers that were a counted no-op
    /// because the head was already `Filled` at a different stamp.
    ///
    /// PER-SET path only: the bump is gated on this node's head ops being
    /// installed, and a node without them keeps the legacy latest-wins insert
    /// (the second offer REPLACES the first, so nothing is refused and this
    /// stays 0). A caller on the per-set path is using the API CORRECTLY — the
    /// head simply holds the older frame — so this is `Ok(())` plus a count,
    /// never an `Err` and never a `signal_failed` bump. The counter exists so
    /// "my node is not firing and I do not know why" has an answer that is not
    /// a log grep.
    pub head_refusals: RwLock<IndexMap<Arc<str>, Arc<AtomicU64>>>,
    /// Per trigger input: heads DISCARDED by a wire-stamp EPOCH RESET — frames
    /// thrown away because a partner input's publisher clock restarted and this
    /// input's held frame belongs to the epoch that ended.
    ///
    /// Its own counter rather than a fold into `unmatched_discards`, because it
    /// is a different diagnosis with a different remedy. An UNMATCHABLE frame
    /// means "a partner ran past you — widen the window, fix a skewed producer,
    /// raise `depth`"; an epoch-reset discard means "a producer REBOOTED and the
    /// node re-based itself onto its new clock", which needs nothing done at
    /// all. Unconditional, never reset.
    pub epoch_reset_discards: RwLock<IndexMap<Arc<str>, Arc<AtomicU64>>>,
    /// Does a COMPLETE aligned set sit unfired in the frozen slots right now?
    ///
    /// The Sync twin of `data_backlog_hint`, exposed because it is the one
    /// question an operator watching a stalled fusion node actually has. Read
    /// by `Scheduler::ns_until_next_fire` to hold the live loop's wake window
    /// at its floor until the backlog clears — it changes WHEN the loop wakes,
    /// never WHAT fires (Principle #7).
    pub backlog_pending: AtomicBool,
}

impl SyncCounters {
    /// Register (or fetch) the per-input counter triple for `input`.
    ///
    /// Idempotent: called once per declared trigger input at `add_node`, so the
    /// maps are complete before any read and a `0` always means "nothing
    /// happened", never "not registered yet".
    pub(crate) fn register_input(&self, input: &str) {
        for map in [
            &self.closer_skips,
            &self.unmatched_discards,
            &self.head_refusals,
            &self.epoch_reset_discards,
        ] {
            let mut guard = match map.write() {
                Ok(g) => g,
                // A poisoned diagnostic lock must never wedge the path it
                // observes; the cost is one unregistered counter reading 0.
                Err(poisoned) => poisoned.into_inner(),
            };
            guard
                .entry(Arc::from(input))
                .or_insert_with(|| Arc::new(AtomicU64::new(0)));
        }
    }

    /// Bump one per-input counter by `n`, registering the input if the
    /// scheduler never did (a hand-built node, or a wiring desync — a lost
    /// count is worse than a late registration).
    pub(crate) fn bump(
        map: &RwLock<IndexMap<Arc<str>, Arc<AtomicU64>>>,
        input: &str,
        n: u64,
    ) -> u64 {
        if n == 0 {
            return 0;
        }
        let existing = map
            .read()
            .map(|g| g.get(input).map(Arc::clone))
            .unwrap_or(None);
        let counter = match existing {
            Some(c) => c,
            None => {
                let mut guard = match map.write() {
                    Ok(g) => g,
                    Err(poisoned) => poisoned.into_inner(),
                };
                Arc::clone(
                    guard
                        .entry(Arc::from(input))
                        .or_insert_with(|| Arc::new(AtomicU64::new(0))),
                )
            }
        };
        counter.fetch_add(n, Ordering::Release) + n
    }

    /// Read one per-input counter. An unknown input and a registered input with
    /// nothing counted both read 0 — a 0 means "nothing happened", never "no
    /// such input".
    fn read(map: &RwLock<IndexMap<Arc<str>, Arc<AtomicU64>>>, input: &str) -> u64 {
        map.read()
            .map(|g| g.get(input).map_or(0, |c| c.load(Ordering::Acquire)))
            .unwrap_or(0)
    }
}

impl NodeHandle {
    /// Create a new handle with shared atomic counters.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        id: String,
        fire_count: Arc<AtomicU64>,
        last_fire_ns: Arc<AtomicU64>,
        panic_count: Arc<AtomicU64>,
        pending_data_count: Arc<AtomicU64>,
        expect_within_missed: Arc<AtomicU64>,
        promise_within_missed: Arc<AtomicU64>,
        tick_within_missed: Arc<AtomicU64>,
        expect_within_backlogged: Arc<AtomicU64>,
        in_tick_since_ns: Arc<AtomicU64>,
        backpressure: Arc<RwLock<IndexMap<Arc<str>, Arc<BackpressureCounters>>>>,
        publisher_disconnects_observed: Arc<AtomicU64>,
        signal_failed: Arc<AtomicU64>,
        output_discards: Arc<RwLock<IndexMap<Arc<str>, Arc<AtomicU64>>>>,
        notify_undelivered: Arc<RwLock<IndexMap<Arc<str>, Arc<AtomicU64>>>>,
        sync: Arc<SyncCounters>,
    ) -> Self {
        Self {
            id,
            fire_count,
            last_fire_ns,
            panic_count,
            pending_data_count,
            expect_within_missed,
            promise_within_missed,
            tick_within_missed,
            expect_within_backlogged,
            in_tick_since_ns,
            backpressure,
            publisher_disconnects_observed,
            signal_failed,
            output_discards,
            notify_undelivered,
            sync,
        }
    }

    /// Returns the node ID.
    pub fn id(&self) -> &str {
        &self.id
    }

    /// Frames on `input` that a NEARER arrived member of the same
    /// stream displaced — the per-set matcher's descent working.
    ///
    /// `input` is the RESOLVED SOURCE TOPIC (the space `TriggerPolicy::Sync`
    /// declares), not the macro field name. Expect this to be LARGE and
    /// growing on a healthy mismatched-rate node; read it beside
    /// [`Self::sync_unmatched_discard_count`], which is the one that means
    /// something is wrong.
    pub fn sync_closer_skip_count(&self, input: &str) -> u64 {
        SyncCounters::read(&self.sync.closer_skips, input)
    }

    /// Heads on `input` discarded by a wire-stamp EPOCH RESET (a
    /// partner's publisher clock restarted, so this input's held frame belonged
    /// to the epoch that ended). See [`SyncCounters::epoch_reset_discards`].
    pub fn sync_epoch_reset_discards(&self, input: &str) -> u64 {
        SyncCounters::read(&self.sync.epoch_reset_discards, input)
    }

    /// Frames on `input` discarded as provably in NO set — a partner
    /// ran more than `sync_window_ms` past them.
    ///
    /// 0 on a healthy node. A climbing count means the window is too tight, a
    /// producer is stalled or skewed, or the fast input's `depth` is below the
    /// fast:slow rate ratio (in which case `drop_oldest` evicts the frames the
    /// matcher needed BEFORE it ever sees them, and the slow frame then dies on
    /// a perfectly healthy graph).
    pub fn sync_unmatched_discard_count(&self, input: &str) -> u64 {
        SyncCounters::read(&self.sync.unmatched_discards, input)
    }

    /// `signal_sync_input` offers on `input` that were a counted
    /// no-op because the head already held a different stamp.
    ///
    /// Not a failure — see [`SyncCounters::head_refusals`]. It counts on the
    /// PER-SET path only (head ops installed); a node without them takes the
    /// latest-wins insert, refuses no offer, and reads 0 here forever.
    pub fn sync_head_refusals(&self, input: &str) -> u64 {
        SyncCounters::read(&self.sync.head_refusals, input)
    }

    /// Does a COMPLETE aligned set sit unfired right now?
    ///
    /// `true` after a burst stopped at its per-step cap, at a pre-fire defer,
    /// or at the panic breaker with a set still aligned in the frozen slots —
    /// the state that reports due-NOW through `ns_until_next_fire`.
    pub fn sync_backlog_pending(&self) -> bool {
        self.sync.backlog_pending.load(Ordering::Acquire)
    }

    /// Returns the total number of times this node has fired (including panicked fires).
    pub fn fire_count(&self) -> u64 {
        self.fire_count.load(Ordering::Acquire)
    }

    /// Hand out a clone of this node's fire counter for an
    /// observer thread (the online profiler's harvesting thread reads it to
    /// build the per-node fire-target gate that feeds `harvest_costs` —
    /// The uniform `--fires` override or the warm-up-derived
    /// per-node targets).
    ///
    /// The returned `Arc<AtomicU64>` ALIASES the same atomic `step()` bumps on
    /// every fire, so an observer sees the LIVE count. It is:
    ///
    /// * **observer-safe** — a shared `Arc` read via atomic loads; safe to poll
    ///   from any thread while the scheduler is running (like every other
    ///   `NodeHandle` read).
    /// * **monotonic** — only ever incremented (one bump per fire, INCLUDING
    ///   panicked/`Err` fires — see [`fire_count`](Self::fire_count)), never
    ///   reset for the node's lifetime.
    /// * **ring-eviction-immune** — independent of the bounded trace ring, so it
    ///   retains the TRUE lifetime fire total even after old `TraceEntry`s are
    ///   evicted. A profiler must NOT infer the fire count by counting trace
    ///   entries (the ring caps them); it reads this counter instead.
    pub fn fire_counter(&self) -> Arc<AtomicU64> {
        Arc::clone(&self.fire_count)
    }

    /// Returns the timestamp (ns) of the last fire, or 0 if never fired.
    pub fn last_fire_ns(&self) -> u64 {
        self.last_fire_ns.load(Ordering::Acquire)
    }

    /// Returns the number of callback panics.
    pub fn panic_count(&self) -> u64 {
        self.panic_count.load(Ordering::Acquire)
    }

    /// Returns the number of `signal_*()` calls the scheduler rejected for
    /// this node because the call did not match the node's trigger policy
    /// (a wiring-desync diagnostic). Bumped by the wrong-policy `Err` arms
    /// of `Scheduler::signal_data` / `Scheduler::signal_sync_input` (the
    /// latter also counts a not-declared sync input). 0 means no desync was
    /// ever observed for this node.
    ///
    /// RAW per-call count — a persistent desync climbs it at the input data
    /// rate (it is NOT latched like the runtime's per-regime `warn!`). Read it
    /// as desync rate/volume, not as a number of distinct desync episodes.
    pub fn signal_failed_count(&self) -> u64 {
        self.signal_failed.load(Ordering::Acquire)
    }

    /// Returns the number of pending (signalled, not-yet-fired) data
    /// arrivals — the node's fire BACKLOG under per-message FIFO firing.
    ///
    /// Each `signal_data()` adds one pending arrival (saturating at
    /// `MAX_CONSUMER_DEPTH` = 64, the deepest input queue that can exist —
    /// signals beyond what a queue can retain describe frames the queue has
    /// already evicted, and an unbounded carry would fire no-op ticks
    /// indefinitely after a burst). Each fire consumes exactly ONE pending
    /// arrival and serves the queue's FIFO head, so a burst of N arrivals
    /// yields N fires across N steps and this count is the remaining
    /// backlog, visible mid-drain.
    pub fn pending_data_count(&self) -> u64 {
        self.pending_data_count.load(Ordering::Acquire)
    }

    /// Count of `#[input(expect_within_ms = N)]`
    /// violations — N ms elapsed without new data on an
    /// `expect_within_ms`-declared input. Subscriber-side observation.
    ///
    /// A window that lapses on the node's per-message FIFO
    /// TRIGGER input while unconsumed arrivals are still signalled on it is
    /// BACKLOG, not silence, and is counted by
    /// [`expect_within_backlogged_count`](Self::expect_within_backlogged_count)
    /// instead. So this counter answers "was this input's producer quiet?",
    /// which is what the knob is documented to mean — it no longer climbs
    /// because the node's own `throttle_ms` / `block` gate held the fire.
    pub fn expect_within_missed_count(&self) -> u64 {
        self.expect_within_missed.load(Ordering::Acquire)
    }

    /// Count of `#[input(expect_within_ms = N)]` windows that
    /// lapsed on this node's per-message FIFO TRIGGER input **while that
    /// input still carried unconsumed signalled arrivals** — the node was
    /// BEHIND, not starved, so the window is reported here instead of in
    /// [`expect_within_missed_count`](Self::expect_within_missed_count).
    ///
    /// The two buckets are disjoint (a lapsed window lands in exactly one)
    /// and neither is ever reset, so `missed + backlogged` is the total
    /// number of lapsed windows over the node's lifetime.
    ///
    /// A non-zero value means the node could not keep up with its own
    /// trigger input for at least one full window — normal and expected
    /// under a declared `throttle_ms` cap or a `block` gate, and worth
    /// investigating otherwise (a tick that never reaches the input's read
    /// keeps it climbing forever; the transport logs a held-head `warn!`
    /// for that case).
    ///
    /// It is NOT a producer-liveness signal: while it is climbing, the
    /// `expect_within_ms` liveliness detector on that input is deliberately
    /// quiet (no miss counted, no `ExpectWithinEvent` emitted), and a
    /// producer that dies mid-backlog is detected only once the backlog
    /// drains, plus at most one more window.
    pub fn expect_within_backlogged_count(&self) -> u64 {
        self.expect_within_backlogged.load(Ordering::Acquire)
    }

    /// Count of `#[output(promise_within_ms = N)]`
    /// violations — N ms elapsed without a publish on a
    /// `promise_within_ms`-declared output. Publisher-side observation.
    pub fn promise_within_missed_count(&self) -> u64 {
        self.promise_within_missed.load(Ordering::Acquire)
    }

    /// Count of
    /// `#[cerulion_node(tick_within_ms = N)]` violations — `tick`
    /// callback took longer than N ms to complete. Execution-time
    /// observation.
    pub fn tick_within_missed_count(&self) -> u64 {
        self.tick_within_missed.load(Ordering::Acquire)
    }

    /// The gating-clock stamp of the fire this node is CURRENTLY
    /// inside, or `None` when it is not inside a tick.
    ///
    /// # Why this is not `tick_within_missed_count` with extra steps
    ///
    /// That counter is bumped from the elapsed read AFTER the callback returns,
    /// so it describes only ticks that CAME BACK. A tick that never returns is
    /// timed by nobody, counted by nothing, and (once the flow-mode work deletes
    /// the level barrier's boundary timeout) noticed by nothing either. This is
    /// the observable that says "node X went into a tick and is still in it";
    /// what to DO about that is the supervisor's job, and it reads a
    /// cross-process page rather than this handle, because the observer is a
    /// different process.
    ///
    /// # Read it as a MARKER, not as an age
    ///
    /// The stamp is on the node's own GATING clock, which under the
    /// multi-process default starts near zero per worker while any observer
    /// outside that process is on wall time — so subtracting it from another
    /// clock's `now` compares two unrelated number lines (the liveness
    /// clock-domain lesson). Within the node's own process it is exactly the
    /// `fire_time_ns` its `TraceEntry` carries, floored at 1 ns because `0` is
    /// the not-in-a-tick sentinel.
    pub fn in_tick_since_ns(&self) -> Option<u64> {
        match self.in_tick_since_ns.load(Ordering::Acquire) {
            0 => None,
            stamp => Some(stamp),
        }
    }

    /// Per-node count of publisher-loss transitions
    /// observed on this node's inputs by the runtime liveliness sweep
    /// (every `Lost` transition — a publisher disconnecting or crashing
    /// are indistinguishable via `number_of_publishers()`). The `Alive`
    /// edge does NOT bump this. Live-only observation: the OBSERVATION
    /// (when the count drops) is not bit-reproducible from a free re-run
    /// — replay reproduces it via the recorded trace.
    pub fn publisher_disconnects_observed_count(&self) -> u64 {
        self.publisher_disconnects_observed.load(Ordering::Acquire)
    }

    /// Per-input `drop_oldest` eviction count for `input_name` —
    /// **EXACT per publisher stream** (multi-publisher topics and restarts
    /// included).
    ///
    /// `drop_oldest` is enforced by iceoryx2's own queue, which silently
    /// reclaims the oldest sample on overflow. Cerulion counts those
    /// evictions on the subscriber at drain time from each stream's
    /// wire-sequence gap, keyed by iceoryx2's `sample.origin()` id — the
    /// missing per-stream sequence slots (gap − 1, summed across streams)
    /// ARE the eviction count, with no cap (a consumer lagging by many
    /// buffers' worth reports the true loss). The same evictions fire the
    /// input's `#[on_event(input = "...")]` `BackpressureEvent` handler (edge-triggered: once per
    /// eviction regime, while this counter accumulates every eviction).
    ///
    /// Cases that deliberately do NOT count (all in the safe,
    /// under-report direction — the counter never fabricates):
    /// - **A stream's first observation** (a brand-new publisher, or a
    ///   restarted one — a restarted port mints a new origin id): its
    ///   pre-observation history is unknowable, so it baseline-establishes
    ///   uncounted and counts exactly from then on.
    /// - **History replay** to a late joiner (BACKWARD sequences within a
    ///   stream): recognized as duplicate re-delivery, never counted. When
    ///   a replay overlaps live frames in one drain, evictions hidden
    ///   under the replayed window are skipped.
    /// - **Corrupt (undersized) frames / errored drains that consumed
    ///   frames**: all baselines reset and one interval may under-report.
    ///   (An errored drain that popped nothing keeps its baselines.)
    /// - **Baseline displacement under publisher churn**: per-stream
    ///   baselines are capped at the topic's `max_publishers`; a stream
    ///   displaced by the capacity eviction (longest-unseen victim)
    ///   re-establishes uncounted for one window.
    ///
    /// Returns `0` if the input is unknown or no eviction has been counted.
    pub fn backpressure_drop_oldest_count(&self, input_name: &str) -> u64 {
        self.backpressure_load(input_name, |c| c.drop_oldest_count.load(Ordering::Acquire))
    }

    /// Per-input `Block` deferred-fire count for `input_name`.
    ///
    /// Returns the number of times the producer's tick was deferred by
    /// the scheduler because this subscriber's buffer was full AND the
    /// policy was `Block` AND no non-Block subscribers existed on the
    /// topic (sole-Block-consumer-full or all-Block-buffers-full case).
    /// NO DATA WAS LOST in these events. Returns 0 if the input is
    /// unknown.
    pub fn backpressure_block_fires_deferred_count(&self, input_name: &str) -> u64 {
        self.backpressure_load(input_name, |c| {
            c.block_fires_deferred_count.load(Ordering::Acquire)
        })
    }

    /// Per-input count of `block` defer REGIMES for `input_name` — how many
    /// times the producer-side defer edge went from re-armed to firing, i.e.
    /// how many loud regime-opening warns it owed. Bumped at the transition,
    /// never at the emission, so a test can require `warns == regimes`
    /// exactly, in release too. Returns 0 if the input is unknown.
    pub fn backpressure_block_defer_regimes_count(&self, input_name: &str) -> u64 {
        self.backpressure_load(input_name, |c| {
            c.block_defer_regimes_count.load(Ordering::Acquire)
        })
    }

    /// Per-input `Sample(N)` event count for `input_name`.
    ///
    /// Returns the number of times an incoming message was dropped
    /// because it arrived within N ms of the last accepted message on
    /// this input. Returns 0 if the input is unknown.
    pub fn backpressure_sampled_count(&self, input_name: &str) -> u64 {
        self.backpressure_load(input_name, |c| c.sampled_count.load(Ordering::Acquire))
    }

    /// Shared lookup-and-load helper for the per-input backpressure
    /// accessors. Lock-free on the inner atomic load — only the
    /// `RwLock::read` is taken to look up the input's `Arc`.
    fn backpressure_load<F: Fn(&BackpressureCounters) -> u64>(
        &self,
        input_name: &str,
        f: F,
    ) -> u64 {
        let guard = self.backpressure.read().unwrap_or_else(|e| e.into_inner());
        guard.get(input_name).map_or(0, |c| f(c.as_ref()))
    }

    /// Per-OUTPUT count of incomplete-output discards on
    /// `output_name` — a tick that released its loan without writing all
    /// declared variable fields (or a failed staged nested-field flush), skipping
    /// publish. This is the off-thread OPERATOR surface (Principle #3) for the
    /// per-port `OutputDiscardLatch::total_discards`, which lives on the
    /// `CerulionPublisher` and is otherwise reachable only from the node's own
    /// tick code (via `AnyPublisher::output_discard_count`). The graph runtime
    /// installs the same `Arc<AtomicU64>` on both the publisher and this map, so
    /// the two views are one count.
    ///
    /// Returns 0 for an unknown output OR a registered output with no discard yet
    /// — a 0 means "nothing discarded", never "no such port". Lock-free on the
    /// inner atomic load; only the `RwLock::read` is taken for the lookup.
    /// Registered per output at graph build via
    /// [`Scheduler::register_output_discard`](crate::scheduler::Scheduler::register_output_discard).
    pub fn output_discard_count(&self, output_name: &str) -> u64 {
        let guard = self
            .output_discards
            .read()
            .unwrap_or_else(|e| e.into_inner());
        guard
            .get(output_name)
            .map_or(0, |a| a.load(Ordering::Acquire))
    }

    /// Per-OUTPUT count of `SentSample` notifies on `output_name`'s
    /// topic that did not reach every live listener — either a consumer whose
    /// event socket filled because nobody drains it (it has fallen off the
    /// event-driven wake path and now only wakes on the live loop's ≤250 ms
    /// heartbeat) or a listener whose process died before its registration was
    /// reaped.
    ///
    /// This is the off-thread OPERATOR surface (Principle #3) for the per-port
    /// `NotifyDeliveryLatch::total_undelivered`, which lives on the
    /// `CerulionPublisher` and is otherwise reachable only from the node's own
    /// tick code (via `AnyPublisher::notify_undelivered_count`). The graph
    /// runtime installs the same `Arc<AtomicU64>` on both the publisher and this
    /// map, so the two views are one count. It matters especially because
    /// the default iceoryx2 log level correctly silences iceoryx2's own per-publish warning about this
    /// condition — without this accessor a sustained degraded wake path is
    /// invisible to anything but a `debug!`-level log.
    ///
    /// Returns 0 for an unknown output OR a registered output with nothing
    /// CONFIRMED undelivered — a 0 never means "no such port". It is not quite
    /// "everything reachable" either: the count is deliberately CONFIRMED-only,
    /// so on an elision-unarmed publisher (raw / service / rmw / netd + gateway
    /// mirror re-injectors) the FIRST shortfall of a regime is an armed
    /// suspicion that reads 0 until the next classified notify confirms it —
    /// the price of never counting an attach race as a failure. A SUSTAINED
    /// degraded wake path confirms on its second notify and then grows;
    /// see `NotifyDeliveryLatch`'s module docs for the full accounting.
    /// Lock-free on the inner atomic load; only the `RwLock::read` is taken for
    /// the lookup. Registered per output at graph build via
    /// [`Scheduler::register_notify_undelivered`](crate::scheduler::Scheduler::register_notify_undelivered).
    pub fn notify_undelivered_count(&self, output_name: &str) -> u64 {
        let guard = self
            .notify_undelivered
            .read()
            .unwrap_or_else(|e| e.into_inner());
        guard
            .get(output_name)
            .map_or(0, |a| a.load(Ordering::Acquire))
    }
}

/// A trace entry for replay comparison.
///
/// Uses `Arc<str>` for node_id to avoid per-fire string allocation.
/// Cloning an `Arc<str>` is a cheap reference count bump.
///
/// `duration_ns` is the full-tick WALL-clock elapsed time (payload-fill
/// INCLUDED — "Mode B"), populated ONLY when duration recording is enabled
/// (see [`Scheduler::set_record_tick_durations`]); it is 0 otherwise. The
/// write is gated on the recording flag SPECIFICALLY: a node with a
/// `tick_within_ms` budget shares the same `Instant` for its budget check,
/// but its `duration_ns` is still 0 unless B-dur recording is on — so the
/// field means exactly "B-dur recording was on for this fire." It is
/// telemetry-only and is DELIBERATELY EXCLUDED from `PartialEq`/`Eq`: wall
/// time varies run-to-run and is non-replayable (Principle #7), so two trace
/// entries that fired the same node at the same logical time compare EQUAL
/// regardless of how long the tick took. This keeps every determinism /
/// byte-identity trace assertion valid even with durations on.
///
/// [`Scheduler::set_record_tick_durations`]: crate::scheduler::Scheduler::set_record_tick_durations
#[derive(Debug, Clone)]
pub struct TraceEntry {
    pub node_id: Arc<str>,
    /// 0-based index of the logical `step()` call that produced this fire — the
    /// PRIMARY sort key of the cross-process trace merge `(step, global_level,
    /// process_rank, seq)`. In the barrier model every process increments it in
    /// lockstep (one bump per `begin_step`), so it ALIGNS across processes. This
    /// cross-process alignment holds ONLY under lockstep stepping — the barrier
    /// model (which gates DAG level advance across processes) guarantees every
    /// process calls `step()` the same number of times, so a given `step` value
    /// names the SAME logical step everywhere; outside that model the merge has
    /// no way to detect a step-range desync (see `merge_partition_traces`'s
    /// caller-owns-lockstep contract). Making `step` the PRIMARY key is also what
    /// lets `global_level` — not the misleading sub-step `fire_time_ns` — order
    /// fires WITHIN a step: a `Period` node's catch-up burst fires at sub-step
    /// `fire_time_ns` values BEFORE the step's `current_time_ns`, so a
    /// `fire_time_ns`-keyed sort would float those higher-level burst fires ahead
    /// of an earlier level's fire in the same step. Grouping by `step` first keeps
    /// every fire of one logical step together so the secondary key
    /// (`global_level`) decides their order; `step` itself is constant within a
    /// step. Like `global_level` (and UNLIKE the wall-time `duration_ns`) this is
    /// replay-DETERMINISTIC, so it is INCLUDED in `PartialEq`/`Eq`.
    pub step: u64,
    pub fire_time_ns: u64,
    /// Deterministic DAG level the node fired at — the GLOBAL level (spans
    /// processes in a partitioned cross-process run, so per-process traces can
    /// be merged back into the single global fire sequence). The flat
    /// `Scheduler::step` path has no levelization and stamps `0`. UNLIKE
    /// `duration_ns` (wall time), this is replay-DETERMINISTIC — derived from
    /// graph topology — so it is INCLUDED in `PartialEq`/`Eq`.
    pub global_level: usize,
    /// Full-tick wall-clock duration in ns (Mode B). `0` when recording is
    /// off; when recording is ON, `0` still occurs for a sub-clock-resolution
    /// or genuinely ~0ns tick — so `0` does NOT distinguish "off" from
    /// "measured ~0". A non-zero value, however, DOES imply recording was on
    /// (the write is gated on the recording flag). A `None` encoding was
    /// deliberately not used to keep `TraceEntry` literal-constructible across
    /// tests. EXCLUDED from `PartialEq`/`Eq` (non-replayable wall time).
    pub duration_ns: u64,
    /// This fire committed **zero** of its loaned outputs (a pre-commit
    /// loan/borrow failure under SHM pressure, or the all-defer discard).
    /// Read from the per-node discard-signal DELTA around the tick callback in
    /// `fire_node_into`; `push_fire` folds it into the ring record as
    /// `TRACE_DISCARD_BIT` (see the note on `discarded`'s sibling in
    /// `scheduler::ScheduledNode` — deliberately not an intra-doc link) so
    /// replay can suppress the fire's
    /// outputs (the byte-identical mirror of the live discard — no phantom seq).
    /// EXCLUDED from `PartialEq`/`Eq` (a data-frame annotation, exactly like
    /// `duration_ns`): the fire *schedule* — the exit-6 divergence key — must not
    /// trip on it, and the flat-vs-level / rayon byte-identity assertions must
    /// stay green.
    pub discarded: bool,
}

// `PartialEq`/`Eq` are HAND-WRITTEN to exclude `duration_ns` AND
// `discarded` (both non-replayable data-frame annotations — Principle #7) but
// INCLUDE `step` + `global_level` (both replay-deterministic — the logical-step
// counter and the DAG topology). If a `Hash` impl is ever needed (e.g. to key a
// `HashSet<TraceEntry>`), HAND-WRITE it over `(node_id, step, fire_time_ns,
// global_level)` to match this `eq` — do NOT `#[derive(Hash)]`, which would hash
// `duration_ns`/`discarded` too and break the `k1 == k2 ⇒ hash(k1) == hash(k2)`
// contract.
impl PartialEq for TraceEntry {
    fn eq(&self, other: &Self) -> bool {
        self.node_id == other.node_id
            && self.step == other.step
            && self.fire_time_ns == other.fire_time_ns
            && self.global_level == other.global_level
    }
}

impl Eq for TraceEntry {}

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
    vec![abi_pin_struct!(QosEventStore {
        expect,
        promise,
        liveliness
    })]
}
