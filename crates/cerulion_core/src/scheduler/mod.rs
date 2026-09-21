// SPDX-License-Identifier: AGPL-3.0-only
//! Deterministic scheduler with simulated time for replay-identical execution.
//!
//! Runtime internals, not node-author API. A node declares WHEN it fires on
//! its macro (`#[cerulion_node(period_ms = ..)]`, `#[input(trigger)]`, ...);
//! the graph runtime turns that into the policies below and drives this
//! scheduler. Node code never constructs or steps a `Scheduler`.
//!
//! # Design
//!
//! The scheduler evaluates trigger policies for each node in insertion order
//! (via `IndexMap`) at each `step()` call. Time advances before evaluation,
//! so `step(10ms)` means "10ms has passed, fire everything due."
//!
//! # Trigger Policies
//!
//! | Policy | Description |
//! |--------|-------------|
//! | `Period` | Fire every interval (e.g., 10ms for 100Hz camera) |
//! | `Data` | Fire when `signal_data()` is called |
//! | `Sync` | Fire once per complete set: a message on EVERY trigger input, within a timestamp window when one is set |
//! | `External` | Fire only via explicit `trigger_external()` call |
//!
//! # Determinism Guarantees
//!
//! - Same inputs → same trace (Principle #7: Replay = Live)
//! - Insertion order = execution order (Principle #5: Graph is source of truth)
//! - Integer nanosecond arithmetic throughout (no floating-point drift)

pub mod catchup_clamp;
// The clamp's WIRING pins (all three `decide_node` call
// sites + the once-per-step read). In-crate because two of the three sites are
// `pub(crate)` level-executor seams no integration test can reach.
#[cfg(test)]
mod catchup_clamp_wiring_tests;
pub mod handle;
// The NODE-DEATH ledger both mint sites write.
pub mod node_death;
/// The per-set Sync matcher (the pure verdict engine replacing
/// `check_sync`).
pub mod sync_match;
pub mod trace_merge;
pub mod trigger;
// Inert for now: the wake-origin taxonomy the WaitSet reactor will consume.
// `#[cfg(test)]`-gated until that wiring lands so its zero-caller
// `pub(crate)` item doesn't trip `dead_code = "deny"`.
#[cfg(test)]
mod wake;

pub use handle::{
    BackpressureCounters, BackpressureEvent, ExpectWithinEvent, LivelinessCause, LivelinessEvent,
    LivelinessState, NodeConfig, NodeHandle, PromiseWithinEvent, TraceEntry,
};
// Crate-internal: the shared per-node QoS-event store
// between `ScheduledNode` (writer) and `NodeContext` (drainer).
pub(crate) use handle::QosEventStore;
pub use node_death::{NodeDeath, NodeDeathCause, NodeDeathLedger};
// Cross-process deterministic trace merge.
pub use trace_merge::{merge_partition_traces, ProcessTrace};
pub use trigger::TriggerPolicy;

use std::collections::{HashSet, VecDeque};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, RwLock};
use std::time::Duration;

use indexmap::IndexMap;
use rayon::prelude::*;

use crate::clock::{Clock, RealClock, VirtualClock};
use crate::error::{TransportError, TransportResult};
use crate::graph::node::BackpressurePolicy;
// The repo's ONE shared flood-suppression state machine, reused for
// the `expect_within_ms` BACKLOG report (loud head / suppressed repeats /
// decade re-announcement / recovery) instead of hand-writing a sixth copy.
use crate::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};

use self::sync_match::{next_sync_step, NextInfo, ProbeSite, SyncHead, SyncStep};

/// The per-STEP fire cap for a `Data` node — how many queued frames one step
/// may serve — and, equivalently, the upper bound on the pending-arrival count
/// it carries to the next step.
///
/// DERIVED from [`crate::graph::topology::MAX_CONSUMER_DEPTH`], the deepest
/// input queue that can exist. Two things follow from that one number:
///
/// * **Per-step work is bounded.** A step can never fire a Data node more times
///   than its queue could have retained frames, so one step's work is bounded
///   no matter how far behind the loop fell.
/// * **Nothing servable is dropped.** Signals beyond what the queue retains
///   describe frames it has already evicted (counted by the drop_oldest probe),
///   so clamping them bounds the collapsed no-op fire tail without ever
///   discarding a frame the subscriber could still serve.
///
/// This is a CAP, not a quota. A burst up to the cap drains WITHIN
/// the step (see `FireKind::Data`); only the remainder carries, and a carried
/// remainder reports due-NOW through [`Scheduler::ns_until_next_fire`], so the
/// live loop keeps its 1 ms floor until it clears.
///
/// It is a DERIVATION rather than a literal mirrored under a
/// `const _: () = assert!(..)`. A derivation cannot drift, so such an assert
/// would compare one expression with itself and could never fail; the LITERAL
/// that still needs a guard is `MAX_CONSUMER_DEPTH`'s own (the
/// `cerulion_macros` mirror), and that guard lives where the literal does, in
/// `topology::tests::max_consumer_depth_pinned_at_64`.
///
/// # It is also the SYNC BURST CEILING, and that is why it is `pub`
///
/// `decide_node`'s Sync arm stamps this very value into
/// `FireKind::Sync`'s `max_sets`, so `tick_sync_burst` cannot fire more sets in
/// one step than this, under EITHER delivery discipline. The replay
/// verifier needs that bound to tell a legal per-set burst from a recorded fire
/// count no schedule can produce, and it RE-EXPORTS this constant
/// (`cerulion_cli_engine::replay_rederive::SYNC_BURST_MAX_SETS`) rather than
/// re-deriving it from `MAX_CONSUMER_DEPTH`. Re-deriving would restate the
/// semantic tie ("the Sync burst cap IS the consumer-depth ceiling") in a second
/// crate, which is the two-copies class; a re-export cannot drift.
pub const DATA_PENDING_CARRY_CLAMP: u64 = crate::graph::topology::MAX_CONSUMER_DEPTH as u64;

/// Per-port QoS watchdog tracker (one per tracked
/// `expect_within_ms` input / `promise_within_ms` output).
///
/// **Single-writer-class anchor.**
/// `last_event_ns` is the **shared** `Arc<AtomicU64>` holding the
/// wire/clock timestamp of the last REAL arrival/publish. It is written
/// ONLY by the off-scheduler reset sites — this port's subscriber
/// (`try_view` on a delivered sample), `signal_input_received` (trigger
/// inputs), and on the output side `signal_output_published` /
/// `record_promise_within_published`. The scheduler NEVER writes it; it
/// only *reads* it to detect a fresh arrival. The one-miss-per-window
/// counter cadence lives in the scheduler-local `window_start_ns`
/// instead, so the scheduler never has to author a value into the shared
/// anchor.
///
/// That single-writer-class invariant is what makes arrival detection
/// **unambiguous**: a load that differs from `last_arrival_ns` is ALWAYS
/// a real arrival, because no other producer of values exists. (The
/// earlier design had the scheduler advance the anchor on a miss, which
/// could alias a real arrival whose wire-ts happened to equal a
/// scheduler-written miss timestamp — under `VirtualClock` that
/// collision was reachable and silently suppressed one regime's event.
/// Removing the scheduler write eliminates that edge entirely.)
///
/// `armed` implements the **edge-trigger** for the reactable event (NOT
/// the counter — the counter bumps every window unconditionally): the
/// first miss of a silence regime fires the event and disarms; a real
/// arrival rearms it.
struct WatchdogTracker {
    within_ns: u64,
    /// Shared anchor — last real arrival/publish ts. Arrival-only-written
    /// (the scheduler never stores into it).
    last_event_ns: Arc<AtomicU64>,
    /// Edge-trigger latch (see type doc).
    armed: bool,
    /// Scheduler-local start of the current miss window. Set to the
    /// arrival ts on a detected arrival; advanced to `new_time` on a miss
    /// (the one-miss-per-window cadence — kept local so the scheduler
    /// never writes the shared anchor).
    window_start_ns: u64,
    /// The shared-anchor value the scheduler last observed. A differing
    /// load ⇒ a real arrival landed since the last step (unambiguous —
    /// the scheduler never writes the anchor).
    last_arrival_ns: u64,
    /// The RESOLVED SOURCE TOPIC of this watched input, when it is
    /// a PER-SET Sync trigger — `None` for every other input.
    ///
    /// TWO KEY SPACES, and this field is the bridge. The scheduler keys a
    /// Sync node's heads by resolved source TOPIC (the space
    /// `TriggerPolicy::Sync` and `signal_sync_input` have always used) while
    /// `input_expect_within` is keyed by the macro FIELD NAME (the space
    /// `NodeHandle::expect_within_missed_count` reads, i.e. the one the USER
    /// spells). Nothing else in the scheduler knows both, which is exactly how
    /// `note_sync_set_arrivals` shipped INERT: it looked heads up by topic in
    /// a field-keyed map and every lookup missed.
    ///
    /// Declared by the caller of [`Scheduler::set_expect_within`] — the
    /// runtime reads it off `sync_input_bindings`, the single source of truth
    /// for which input is wired to which topic — never inferred here, exactly
    /// as `fifo_trigger` is.
    sync_topic: Option<Arc<str>>,
    /// This watched input is the node's per-message FIFO TRIGGER
    /// input — the one whose arrivals drive `pending_data_count`. Declared
    /// by the caller of [`Scheduler::set_expect_within`] (the runtime reads
    /// it off `data_trigger_bindings`, the single source of truth for
    /// "which input signals this node"), never inferred here.
    ///
    /// Only a marked input can have its window SUPPRESSED by a signalled
    /// backlog: a node's non-trigger latest-value inputs share the same
    /// node-level `pending_data_count` but are fed by unrelated producers,
    /// and the held-context staleness detector lives on exactly
    /// those — suppressing them would silence the feature.
    fifo_trigger: bool,
    /// Whether the PREVIOUS evaluation of this tracker saw a
    /// signalled backlog on a `fifo_trigger` input. The falling edge
    /// (backlog → no backlog) re-anchors the window to the drain instant,
    /// so the first post-drain window is measured from when the node
    /// caught up rather than from the stamp of the frame it was only just
    /// served (which is, by construction, the OLDEST surviving one) — the
    /// drain-out edge, otherwise one spurious miss per drained burst.
    prev_backlog: bool,
    /// Did the CURRENT backlog episode actually SUPPRESS a window?
    ///
    /// Set by the suppressed branch, cleared at the drain-out edge, and the
    /// re-anchor's second condition. Without it the edge fires on any
    /// `pending > 0 → 0` transition — which is the perfectly ordinary "one
    /// frame delivered, next step silent" shape, since `run_qos_windows` runs
    /// post-drain/pre-decide and so sees `pending == 1` on EVERY step a frame
    /// arrived. That re-anchored a window whose anchor was the FRESH stamp of
    /// the one frame just served, widening every Data trigger input's effective
    /// threshold to `within_ms + one step` unconditionally, on graphs that were
    /// never behind at all.
    ///
    /// The correction the edge exists for only applies when the node WAS
    /// behind: the anchor then holds the OLDEST surviving frame's stamp, which
    /// under-states when the node caught up. "Was behind" is exactly "a window
    /// lapsed while the backlog was signalled", which is what this records — so
    /// the edge now fires on precisely the episodes its own rationale
    /// describes, and a node keeping up pays nothing.
    backlog_suppressed_window: bool,
    /// Which CAUSE opened the currently-open suppression regime —
    /// `true` for a Sync HELD MEMBER, `false` for a Data BACKLOG.
    ///
    /// The two causes have their own `info!` heads because they have different
    /// remedies, and the CLOSE has to name the same one its head did or the
    /// pair cannot be grepped. It cannot re-derive the cause: by the time the
    /// regime closes, the condition that opened it is false by definition. So
    /// the open records it here and the close reads it.
    suppression_was_held_member: bool,
    /// Flood-suppression state for the BACKLOG report (the repo's
    /// shared machine — see [`FailureRegimeLatch`]). The per-window line is
    /// `debug!`; this drives the once-per-regime `info!` head, the decade
    /// re-announcement, and the closing line carrying the suppressed count.
    /// INFO, not WARN: a node that is behind its producer because of its own
    /// declared `throttle_ms` / `block` gate is behaving as designed.
    backlog_latch: FailureRegimeLatch,
}

/// The per-node hook that lets a Data node's fire loop pull the NEXT
/// queued frame off a `DrainSource::Unified` trigger input, between fires,
/// within one step.
///
/// It exists because the Unified boundary drain can only pop ONE frame (it
/// freezes that frame into the subscriber's slot for the tick's `try_view`, and
/// there is exactly one slot), so it can only ever signal ONE arrival — while
/// the queue behind it may hold many. On the `Separate` path the boundary drain
/// signals one arrival per queued frame, so `pending_data_count` already
/// describes the whole burst and no hook is installed.
///
/// `drain` returns what
/// [`crate::graph::node::NodeEntry::refill_trigger_input`] returns — `(popped,
/// latest_ts)` — because that IS what it calls: the same
/// one-receive-per-served-frame path the unified trigger drain uses, through the same node
/// lock, for cdylib and in-process nodes alike. `popped == 0` means "nothing
/// more this step" and the loop stops.
///
/// It is the REFILL entry point, not the boundary drain, and the difference is
/// load-bearing: the boundary RE-OFFERS a head the tick never read (Principle
/// #6 — a fire that was deferred, or whose tick collapsed before reaching the
/// input's `try_view`, must not lose its signal), while here that same state
/// means the fire just run did NOT consume the head, so the correct answer is
/// `(0, None)`. Counting the re-offer as a fresh frame re-fires the node on ONE
/// frame up to [`DATA_PENDING_CARRY_CLAMP`] times per step — see
/// [`crate::transport::subscriber::CerulionSubscriber::refill_for_trigger`].
///
/// `input` is carried so the fire loop can advance the input's `expect_within`
/// watchdog anchor to each refilled frame's wire timestamp, exactly as the
/// boundary drain does for the frame it popped.
#[derive(Clone)]
struct TriggerRefill {
    input: Arc<str>,
    drain: Arc<dyn Fn() -> (u64, Option<u64>) + Send + Sync>,
}

/// The transport ops the per-set Sync matcher demands through its
/// verdicts.
///
/// Deliberately ONE multiplexed op rather than four hooks, mirroring the ONE
/// FFI symbol (`cerulion_node_sync_head_op`) it crosses on a cdylib: the four
/// are transitions of ONE state machine, and four separate hooks would admit
/// partial-capability nodes (advance-without-void) the degrade logic would then
/// have to enumerate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyncHeadOp {
    /// Fill this input's head at a LEVEL BOUNDARY. A head the tick never read
    /// is RE-OFFERED here (Principle #6 — a deferred fire, or a tick that
    /// collapsed before reaching this input's read, must not lose its signal).
    FillBoundary,
    /// Fill this input's head BETWEEN two fires of one step. A still-frozen
    /// head answers "nothing" here, because the fire that just ran did NOT
    /// consume it — re-offering it would fire the node again on one frame.
    FillRefill,
    /// The NON-CONSUMING "has another frame arrived behind the head?" probe.
    ProbeNext,
    /// Pop the next frame into the staged slot and report its stamp.
    PeekNext,
    /// Pop past the head (the caller counts the skip) and refill from the
    /// staged next, or failing that from the queue.
    Advance,
    /// Serve a RESTORED head's read as "no frame" — the head names a stamp but
    /// no live frame, and a live queued frame belongs to the NEXT set.
    Void,
}

/// What a [`SyncHeadOp`] answered.
///
/// [`Self::Failed`] is a first-class answer rather than an error, because the
/// failed-probe policy that interprets it is POSITION-AWARE: the same failure means
/// `None` at the argmin (fire greedy — sound) and `Present` at the gate (this
/// input REFUSES the gate). Collapsing it into an `Err` would force one
/// mapping on both sites, and mapping a gate failure to `None` lets a probe
/// failure VOUCH FOR SCARCITY on an unverified input — destroying arrived
/// complete in-window sets, which must never happen.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyncOpAnswer {
    /// A fill or an advance produced a head at this stamp.
    Head(u64),
    /// A fill produced no head / a probe found no second frame / a peek found
    /// nothing valid behind the head. All one sound answer at every site:
    /// there is nothing there.
    Nothing,
    /// A probe found a second arrived frame (stamp not yet known).
    Present,
    /// A peek popped the next frame; here is its stamp.
    Stamp(u64),
    /// The op FAILED (a poisoned cdylib `NODES` mutex, an FFI error code, a
    /// caught panic, an iceoryx2 error). Never evidence that ENABLES descent.
    Failed,
}

/// Which alignment site is running — the same discriminator
/// [`crate::transport::subscriber::CerulionSubscriber`]'s trigger drain takes,
/// DECLARED by the caller and never inferred.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyncAlignSite {
    /// Once per level boundary, before decide.
    Boundary,
    /// Between two fires of one step's burst.
    Refill,
}

/// Did an alignment pass end with a complete, in-window set in the heads?
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlignOutcome {
    /// A set is aligned and may fire.
    Complete,
    /// No set. Do not fire.
    Incomplete,
}

/// A Sync node's transport seam — the ops, and the DECLARATION-ORDER
/// input list they are indexed by.
///
/// Installed once per Sync node at graph build by
/// [`Scheduler::set_sync_ops`], which REFUSES a list that disagrees with the
/// node's declared `TriggerPolicy::Sync::inputs`: the matcher's gate scan, its
/// tie-break and the whole `next_info` scratch are indexed by declaration
/// position, so two lists silently disagreeing would make the verdict describe
/// one input while the op moved another.
///
/// A node with NO ops installed is a pure-scheduler embedder's (there are no
/// transport slots, so there is nothing to probe or advance): it keeps the
/// classic hand-driven semantics through [`Scheduler::signal_sync_input`], and
/// the matcher runs with every `next_info` at `None`, which disables descent
/// and leaves completeness + window death exactly as they were.
#[derive(Clone)]
struct SyncOps {
    inputs: Vec<Arc<str>>,
    op: SyncOpFn,
}

/// The shared shape of a Sync node's op dispatcher — aliased so the field, the
/// setter and any future holder all name ONE type rather than three
/// hand-copied `dyn Fn` spellings.
type SyncOpFn = Arc<dyn Fn(&str, SyncHeadOp) -> SyncOpAnswer + Send + Sync>;

/// Internal state for a scheduled node.
struct ScheduledNode {
    policy: TriggerPolicy,
    callback: Box<dyn FnMut() + Send>,

    // Interned node ID for cheap TraceEntry clones
    node_id: Arc<str>,

    // Timing state (managed by scheduler)
    next_fire_ns: Option<u64>,
    pending_data_count: u64,
    /// Per trigger-input HEAD — the frame this input contributes to
    /// the set currently being formed, keyed by the RESOLVED SOURCE TOPIC.
    ///
    /// Replaces `sync_input_timestamps: IndexMap<String, u64>`. The bare stamp
    /// could not say "this head was consumed by a fire", so the earlier
    /// code expressed that by CLEARING the whole map on every fire — which is
    /// exactly what destroyed the rest of a burst. A consumed head is now an
    /// `Emitted` TOMBSTONE, which is what tells the next align pass to re-drain
    /// that input and what lets a k-set burst serve k sets.
    sync_heads: IndexMap<String, SyncHead>,

    /// This Sync node's transport seam, or `None` for a hand-driven
    /// pure-scheduler node. Installed once at graph build by
    /// [`Scheduler::set_sync_ops`].
    sync_ops: Option<SyncOps>,

    /// Does a COMPLETE aligned set sit unfired in the frozen slots?
    ///
    /// The Sync twin of [`Self::data_backlog_hint`], and its setting rule is
    /// deliberately BROADER than `refilled_unfired`'s: it is set whenever a
    /// complete aligned set is left unfired at the end of the burst loop —
    /// the per-step cap, a mid-burst defer, a FIRST-fire defer on a set the
    /// BOUNDARY aligned, and a breaker break all qualify.
    ///
    /// Data can afford the narrow rule because its defer path hands signalled
    /// arrivals back to `pending_data_count`, which itself feeds the due-NOW
    /// arm. Sync has no pending-count analogue, so without the broader rule a
    /// throttled Sync node's first deferred set reports no due-NOW and waits
    /// for an unrelated publish or the 250 ms liveliness cap where Data
    /// recovers at the 1 ms floor.
    ///
    /// Read ONLY by [`Scheduler::ns_until_next_fire`] and mirrored into
    /// `sync_counters.backlog_pending` for operators — never by
    /// `step`/`decide_node`/`fire_node`, so it changes WHEN the loop wakes and
    /// never WHAT fires (Principle #7).
    sync_backlog_hint: bool,

    /// The per-set Sync observability bundle, shared with
    /// [`NodeHandle`].
    sync_counters: Arc<crate::scheduler::handle::SyncCounters>,

    /// The UNMATCHABLE (window-death) flood latch, one per trigger
    /// input. A skew regime discards a frame per boundary, so an unlatched
    /// loud head is the disk-fill class; PASSED-OVER skips are never
    /// latched because they are never loud (counter + `debug!` only).
    sync_unmatched_latches: IndexMap<Arc<str>, FailureRegimeLatch>,

    /// The align-op FAILURE flood latch. Node-level rather than
    /// per-input because the conditions that produce it are node-level (a
    /// poisoned cdylib `NODES` mutex, a caught panic in the FFI shim), and a
    /// per-input split would report one fault k times. A PERMANENTLY failing
    /// op costs greedy-or-wait membership every boundary — loudly, and never a
    /// spin or a wrong set.
    sync_op_failure_latch: FailureRegimeLatch,
    /// Did any align op FAIL during the pass now running?
    ///
    /// The recovery half of the flood latch needs a healthy TRANSITION to fire
    /// on, and an align pass that ran to its end without a failure is exactly
    /// that. Reset at the top of each pass, set by `report_sync_op_failure`,
    /// read by `align_node_sync_heads` once the pass returns.
    sync_op_failed_this_pass: bool,
    /// Per trigger input, the highest wire stamp this node has ever
    /// popped on it — the reference a wire-stamp EPOCH RESET is detected
    /// against.
    ///
    /// Within ONE run a single writer's stamps are monotonically non-decreasing
    /// (one writer, one clock), so a fresh pop
    /// that lands FAR below this input's own high-water is structural evidence
    /// that the publisher's clock restarted, not that a frame arrived late.
    sync_stamp_high_water: IndexMap<String, u64>,
    /// Flood suppression for the epoch-reset announcement. A reboot
    /// can cross both inputs' queues, and the reset can legitimately fire more
    /// than once while the deeper queue drains its last old-epoch frames.
    sync_epoch_latch: FailureRegimeLatch,
    /// The newest stamp of the CURRENT clock epoch, once a reset has
    /// established one.
    ///
    /// A reset cannot be a one-shot purge of the heads that happen to be filled
    /// at that instant: the partner's OLD-epoch frames are still QUEUED, and its
    /// head is refilled from that queue on the very next boundary. Without a
    /// band to test them against, the first straggler becomes the immortal
    /// maximum all over again and the wedge simply resumes one boundary later —
    /// measured, on the arm below, before this field existed.
    ///
    /// So the reset also re-bases every other input (it REMOVES their
    /// high-water, and "no high-water" is exactly "awaiting re-base"), and a
    /// re-based input's fills are tested against this band until one lands
    /// inside it. Frames above `floor + window` are old-epoch stragglers,
    /// discarded and counted; the first in-band frame re-establishes that
    /// input's high-water and ends its purge. An input that was NEVER re-based
    /// keeps its high-water and is never purged, which is what stops a
    /// legitimately-ahead partner from being mistaken for a dead epoch.
    sync_epoch_floor: Option<u64>,

    /// Reusable align-pass scratch (the zero-alloc scratch pattern: cleared and
    /// refilled, capacity retained, so a steady-state boundary allocates
    /// nothing). Taken with `mem::take` for the pass and written back, which is
    /// also what keeps the loop free of whole-node borrows.
    sync_scratch_heads: Vec<Option<SyncHead>>,
    sync_scratch_next: Vec<NextInfo>,
    sync_scratch_advances: Vec<u32>,

    /// This Data node's `DrainSource::Unified` refill hook, or `None`
    /// for a `Separate` binding / a non-Data node. Installed once at graph build
    /// by [`Scheduler::set_trigger_refill`]; see [`TriggerRefill`].
    trigger_refill: Option<TriggerRefill>,

    /// Does this Data node's Unified trigger input hold a frame that
    /// has been popped and frozen but NOT yet fired?
    ///
    /// Set by the Data fire loop when it stops at the per-step fire cap
    /// ([`DATA_PENDING_CARRY_CLAMP`]) with a refilled frame in hand; cleared by
    /// the next step's `decide_node`. Read ONLY by
    /// [`Scheduler::ns_until_next_fire`] — the live loop's wake sizing — never
    /// by `step`/`decide_node`/`fire_node`, so it changes WHEN the loop wakes
    /// and never WHAT fires (Principle #7).
    ///
    /// It is needed because `pending_data_count` cannot describe that frame: on
    /// the Unified path a frozen-but-unfired head is re-OFFERED by the next
    /// boundary drain (which mints its signal then), so counting it as pending
    /// now would fire the node twice for one frame. This is the one bit that
    /// says "there is more, and nothing has signalled it yet".
    data_backlog_hint: bool,

    /// This node's `throttle_ms` producer rate cap, in ns.
    ///
    /// The GATE stays in the graph's pre-fire closure — this is not a second
    /// copy of the decision, and `decide_node`/`fire_node` never read it. It is
    /// here for exactly one reason: [`Scheduler::ns_until_next_fire`] sizes the
    /// live loop's wait, and a throttle-deferred node is due-NOW by every
    /// signal the scheduler can see (`pending_data_count > 0`), so without the
    /// deadline the loop reports `Some(0)` and POLLS at its 1 ms floor for the
    /// whole throttle window — which on a 100 ms throttle is a hundred wakeups
    /// that can each only decide to defer again.
    ///
    /// The inputs the deadline is computed from — `fire_count` and
    /// `last_fire_ns` — are the scheduler's OWN canonical state, the same two
    /// the closure reads back through the node handle, and both sides call the
    /// same free function [`crate::graph::runtime::throttle_defers`]. So there is one
    /// rule and one source of truth; what is duplicated is the WINDOW LENGTH,
    /// a static declaration that cannot drift within a run.
    ///
    /// `None` for every node without `throttle_ms`, which is almost all of
    /// them — and `None` contributes nothing, so an untouched graph's wake
    /// sizing is byte-identical.
    throttle_ns: Option<u64>,

    /// How many times a TRACE-DRIVEN fire burst
    /// ([`FireKind::Replay`]) asked this node's [`TriggerRefill`] hook for the
    /// next FIFO frame and got NOTHING.
    ///
    /// The recording says a frame was consumed there, so an empty refill means
    /// the replay's injected input stream is short of what the recording held.
    /// The burst FIRES anyway — the plan is authoritative for the fire SCHEDULE,
    /// and skipping the fire would turn an input shortfall into a fabricated
    /// fire-schedule divergence — so the node re-reads its held head and the
    /// symptom surfaces as a frame-content divergence. This counter is what
    /// names the real cause (Principle #3: observable, log-level independent);
    /// read via [`Scheduler::replay_refill_shortfalls`]. Zero on every live path
    /// (nothing but the Replay arm touches it).
    replay_refill_shortfalls: u64,

    /// This node's installed INTRA-STEP pauses for the current
    /// replayed step — see [`NodeReplayPauses`] for the contract and for why
    /// they live here rather than in a slot table beside the fire plan.
    replay_pauses: NodeReplayPauses,

    /// The engine's intra-step injection callback, cloned in from
    /// [`Scheduler::set_replay_injection_hook`] (and inherited by a node added
    /// afterwards — the `record_tick_durations` pattern). `None` on every live
    /// path: nothing but a replay engine ever installs one, and with none
    /// installed a reached pause delivers nothing and stays UNCONSUMED rather
    /// than silently counting itself served.
    replay_injection_hook: Option<ReplayInjectionHook>,

    // Observable state (shared with NodeHandle)
    fire_count: Arc<AtomicU64>,
    last_fire_ns: Arc<AtomicU64>,
    panic_count: Arc<AtomicU64>,
    pending_data_count_shared: Arc<AtomicU64>,

    // Three QoS miss counters.
    expect_within_missed: Arc<AtomicU64>,
    promise_within_missed: Arc<AtomicU64>,
    tick_within_missed: Arc<AtomicU64>,

    /// Per-node count of `expect_within_ms` windows that lapsed on
    /// a FIFO TRIGGER input while that input still carried unconsumed
    /// signalled arrivals. Such a window is BACKLOG, not silence, so it is
    /// counted HERE instead of in `expect_within_missed` — the two buckets
    /// are disjoint and neither is ever reset. Shared with
    /// `NodeHandle::expect_within_backlogged_count`.
    expect_within_backlogged: Arc<AtomicU64>,

    /// This node's IN-TICK marker — the gating-clock stamp of the
    /// fire currently executing, or `0` when the node is not inside a tick.
    ///
    /// It exists because `tick_within_missed` above measures only ticks that
    /// RETURN: [`Scheduler::fire_node_into`] reads `elapsed()` AFTER the callback,
    /// so a tick that never returns is timed by nobody and counted by nothing. The
    /// only thing that used to notice one was the level barrier's boundary
    /// timeout, which the flow-mode work deletes.
    ///
    /// Written at exactly two sites in `fire_node_into` — the stamp immediately
    /// after `tick_start` is captured, `0` beside the post-callback `elapsed_ns`
    /// read. The second site is reached even on a CAUGHT PANIC (the panic is
    /// contained by the `catch_unwind` above it), which is the same invariant
    /// the replay-suppress clear rests on a few lines earlier. Two atomic
    /// stores per fire: the discard-verdict feature set the cost precedent (it added an
    /// atomic load, a conditional store and a second load to the same body).
    ///
    /// Shared with [`NodeHandle::in_tick_since_ns`](crate::scheduler::NodeHandle::in_tick_since_ns),
    /// which decodes the sentinel so `0` never leaks to a caller (Principle #3).
    in_tick_since_ns: Arc<AtomicU64>,

    /// This node's cross-process wedge marker and its SLOT in the
    /// rank's page, installed together by [`Scheduler::set_wedge_page`].
    ///
    /// `None` — the monolith, every test, every bare scheduler — so the two
    /// `fetch_add`s are never reached and an un-installed page costs nothing
    /// beyond the two stores above. Held as ONE `Option` rather than two so a
    /// marker can never exist without the slot it must be indexed with.
    ///
    /// A `dyn` handle for the reason [`Scheduler::catchup_arm`] is one: the
    /// concrete `wedge_page::MappedWedgePage` is
    /// `#[cfg(unix)]`, and naming it here would cfg-gate this field, this fire
    /// body and every construction site (pinned by `cfg_audit_test`).
    wedge: Option<(Arc<dyn WedgeMarker>, usize)>,

    /// Per-node count of publisher-loss transitions the
    /// runtime liveliness sweep observed on this node's inputs. Shared with
    /// `NodeHandle::publisher_disconnects_observed`; bumped by the runtime
    /// (`GraphRuntime::liveliness_sweep`) via the clone handed out by
    /// `Scheduler::publisher_disconnects_counter`, NOT by `step()`.
    publisher_disconnects_observed: Arc<AtomicU64>,

    /// Per-node count of `signal_*()` desync
    /// rejections. Shared with `NodeHandle::signal_failed`; bumped in the
    /// wrong-policy `Err` arms of [`Self::signal_data`] /
    /// [`Self::signal_sync_input`], NOT by `step()` (off the firing path).
    signal_failed: Arc<AtomicU64>,

    /// Per-input `expect_within_ms` tracking
    /// (value = [`WatchdogTracker`]). When a step's current_time exceeds
    /// the window start + within_ns, `expect_within_missed` increments.
    ///
    /// The tracker's `last_event_ns` is a **shared**
    /// `Arc<AtomicU64>` minted by `GraphRuntime::build` and ALSO held by
    /// this input's [`CerulionSubscriber`] (non-trigger body reads write
    /// it on delivery) and the runtime's per-level trigger drain
    /// (`drain_level`; trigger inputs reset it same-step via
    /// [`Self::signal_input_received`]).
    ///
    /// The anchor is **arrival-only-written**
    /// — `step()` only READS it (to detect a fresh arrival) and keeps the
    /// one-miss-per-window cadence in the tracker's scheduler-local
    /// `window_start_ns`, so the scheduler never authors a value into the
    /// shared anchor. That single-writer-class invariant makes arrival
    /// detection unambiguous and eliminates the equal-timestamp
    /// missed-rearm edge. Deterministic: every stored value is a
    /// wire/clock timestamp (data), not a wall read. The tracker also
    /// carries the edge-trigger latch for the reactable
    /// [`ExpectWithinEvent`].
    input_expect_within: IndexMap<String, WatchdogTracker>,

    /// Per-output `promise_within_ms` tracking
    /// (value = [`WatchdogTracker`]). Symmetric to `input_expect_within`.
    /// `signal_output_published` / the publisher's
    /// `record_promise_within_published` write the shared anchor on every
    /// successful send; `step()` increments `promise_within_missed` when
    /// current_time exceeds the configured interval since the last publish.
    ///
    /// Same arrival-only-anchor design as
    /// the input side — the scheduler only reads the anchor and keeps the
    /// cadence in `window_start_ns`. Carries the edge-trigger latch for
    /// the reactable [`PromiseWithinEvent`].
    output_promise_within: IndexMap<String, WatchdogTracker>,

    /// Per-node `tick_within_ms` budget (ns).
    /// `Some(d)` means `fire_node` measures the tick callback's
    /// elapsed time and increments `tick_within_missed` when
    /// elapsed > d.
    tick_within_ns: Option<u64>,

    /// B-dur: when true, `fire_node_into` records the full-tick
    /// wall elapsed into the emitted `TraceEntry::duration_ns` (Mode B,
    /// telemetry-only). Default false (zero hot-path tax). Set en masse via
    /// [`Scheduler::set_record_tick_durations`].
    record_durations: bool,

    /// Shared per-node store for the edge-triggered
    /// QoS watchdog events. `Some` once `register_qos_event_store` has run
    /// (the runtime calls it for every node that has any `expect_within`
    /// / `promise_within` window). `step()` `push_*`es into it on a miss;
    /// the node's `NodeContext` holds the same `Arc` and `take_*`es from
    /// the tick body. `None` ⇒ no QoS windows ⇒ no event ever fires.
    /// `tick_within_ms` is intentionally absent — it stays counter-only
    /// (no reactable event) until the record-execution-time clock model
    /// lands.
    qos_events: Option<Arc<QosEventStore>>,

    /// Per-input backpressure counters. Populated by
    /// `register_backpressure_input_with` at graph build time (the
    /// idempotent `register_backpressure_input` is a test-only sibling).
    /// Shared with `NodeHandle::backpressure` via `Arc<RwLock>`
    /// so observers see the same atomic state as the producer-side
    /// counter bumps. See `BackpressureCounters` docs for variant
    /// semantics.
    backpressure: Arc<RwLock<IndexMap<Arc<str>, Arc<BackpressureCounters>>>>,

    /// Per-OUTPUT discard-count mirrors, shared with
    /// `NodeHandle::output_discards` via `Arc<RwLock>`. Populated by
    /// `register_output_discard` at graph build (once per publisher-owning
    /// output). Each entry ALIASES the `Arc<AtomicU64>` the runtime also installs
    /// on the port's `CerulionPublisher`, so `NodeHandle::output_discard_count`
    /// observers see the same count the publisher's `OutputDiscardLatch` stores.
    /// The scheduler `step()` never reads this — it is a pure observability seam,
    /// wired at build and read off-thread.
    output_discards: Arc<RwLock<IndexMap<Arc<str>, Arc<AtomicU64>>>>,

    /// Per-OUTPUT undelivered-notify mirrors, shared with
    /// `NodeHandle::notify_undelivered` via `Arc<RwLock>`. Populated by
    /// `register_notify_undelivered` at graph build (once per publisher-owning
    /// output). Each entry ALIASES the `Arc<AtomicU64>` the runtime also installs
    /// on the port's `CerulionPublisher`, so `NodeHandle::notify_undelivered_count`
    /// observers see the same count the publisher's `NotifyDeliveryLatch` stores.
    /// The scheduler `step()` never reads this — it is a pure observability seam,
    /// wired at build and read off-thread.
    notify_undelivered: Arc<RwLock<IndexMap<Arc<str>, Arc<AtomicU64>>>>,

    /// Optional pre-fire predicate for the Block
    /// defer-the-fire model. When `Some(f)` and `f()` returns true,
    /// `evaluate_node` skips this node's fire for this step
    /// (no callback invocation, no fire_count bump, no trace entry).
    /// The closure is responsible for bumping the appropriate
    /// per-input `block_fires_deferred_count` counter and emitting
    /// the structured warn — the scheduler does NOT do this on its
    /// own because counter attribution requires knowledge of which
    /// downstream subscriber's input is the cause.
    ///
    /// Typically wired by `GraphRuntime` after graph build: the
    /// closure captures Arc references to the node's publishers and
    /// consults the publisher's backpressure-defer predicate
    /// (which encapsulates the graceful-degradation logic per the
    /// consumer-mix rule).
    ///
    /// `Send` + `Sync` because the scheduler may be run from
    /// arbitrary threads. `'static` because the closure outlives
    /// the function that constructed it.
    /// The closure receives the step's already-advanced
    /// `current_time_ns` (the same instant the trigger evaluation sees),
    /// so a `throttle_ms` time-gate is deterministic and consistent with
    /// the rest of the step. The `block` predicate ignores the argument
    /// (it reads its outstanding-counter atomics).
    pre_fire_check: Option<Arc<dyn Fn(u64) -> bool + Send + Sync>>,

    // Circuit breaker state
    consecutive_panics: u32,
    disabled: bool,

    // External trigger flag
    external_triggered: bool,

    /// Reusable per-node parallel-fire trace fragment. The within-level
    /// fire passes `mem::take` it, `clear()` it, fire this node's `TraceEntry`s
    /// into it, then write it back — so once its capacity stabilizes the fragment
    /// is zero-alloc at steady state. It is drained in DECISION (pos) order at the
    /// level merge (`tick_decided_parallel` pass 3) so the merged trace stays
    /// byte-identical to the serial `tick_decided` order. `Vec<TraceEntry>: Send`
    /// keeps `ScheduledNode` `Send` (the rayon wide path moves disjoint
    /// `&mut ScheduledNode` to workers).
    trace_fragment: Vec<TraceEntry>,

    /// RECORD side: the per-node publish-DISCARD signal, bumped by
    /// this node's publishers (`OutputProxy::Drop`'s all-defer discard,
    /// and `CerulionPublisher::loan_proxy`'s pre-commit loan-failure path) every
    /// time a loaned output is released WITHOUT a `commit_sequence`. Shared with
    /// every publisher of this node via the runtime's `set_node_discard_signals`
    /// (each publisher holds a clone). `fire_node_into` snapshots it before the
    /// tick callback and reads it after: a non-zero DELTA means "this fire
    /// committed 0 outputs" → the emitted `TraceEntry::discarded` is set →
    /// `push_fire` folds it into the ring record as `TRACE_DISCARD_BIT`
    /// (deliberately NOT an intra-doc link: this field is PORTABLE while
    /// `trace_ring` is `#[cfg(unix)]`, so a link here is unresolvable on a
    /// non-unix target and fails the `RUSTDOCFLAGS=-D warnings` docs gate —
    /// pinned by `cfg_audit_test`). Minted with a fresh default in
    /// `add_node` (a bare scheduler with no runtime never marks — the default
    /// Arc is shared with no publisher, so the delta is always 0); the runtime
    /// OVERWRITES it with the shared Arc. Atomic reads only on the fire path (no
    /// alloc — `step_zero_alloc_test` stays green).
    discard_signal: Arc<std::sync::atomic::AtomicU32>,

    /// REPLAY side: the per-node publish-SUPPRESS flag, set by
    /// `fire_node_into` for a marked fire's callback duration (and cleared after,
    /// even on tick Err/panic). Shared with every publisher of this node via the
    /// runtime's `set_node_discard_signals`; `OutputProxy::Drop` reads it FIRST
    /// (before `commit_sequence`) and returns without publishing — the
    /// byte-identical mirror of the live discard. Untouched on the RECORD/live
    /// path (the queue below is empty, so the flag is never set).
    replay_suppress: Arc<std::sync::atomic::AtomicBool>,

    /// REPLAY side: this node's recorded per-fire discard bools for the
    /// CURRENT step, in fire order (Period catch-up ⇒ multiple fires per step, so
    /// this is a queue, not a bool). Installed once per step by
    /// [`Scheduler::set_replay_discards`] (the replay engine reads each fire's
    /// `is_discarded()` into it); `fire_node_into` pops the front each time this
    /// node fires and, when `true`, drives `replay_suppress`. EMPTY on the
    /// live/record path (never installed), so `pop_front()` is a cheap no-op
    /// there. A leftover after a step is a fire-schedule divergence already
    /// surfaced by exit 6 (defensive; the next `set_replay_discards` clears it).
    replay_discard_queue: std::collections::VecDeque<bool>,

    /// This node's read-outcome stages — one
    /// [`crate::read_outcome::ReadOutcomeStage`] per graph-wired input read
    /// path (body inputs in wiring order, then any Separate/Sync
    /// trigger-drain subscribers), installed by the runtime's
    /// `set_node_read_stages` (the `discard_signal` pattern: the SAME `Arc`s
    /// the runtime shared into the subscribers). Drained by
    /// [`Scheduler::merge_read_outcomes`] on the step thread at the level-end
    /// merge, into the recording trace ring as kind-6 records. EMPTY for a
    /// bare scheduler (no runtime) and for every node until the runtime
    /// registers — and the stages inside are DISARMED until a recording
    /// installs the trace ring, so a non-recording run does no stage work at
    /// all.
    read_stages: Vec<Arc<crate::read_outcome::ReadOutcomeStage>>,

    /// The shared NODE-DEATH ledger this node reports its
    /// disable TRANSITION into.
    ///
    /// The `discard_signal` pattern exactly, and for the same three reasons:
    /// minted with a fresh default in [`Scheduler::add_node`] (so a bare
    /// scheduler with no runtime writes into a ledger nothing shares and nothing
    /// drains), OVERWRITTEN by the runtime with the ONE instance it also hands
    /// every tick callback, and touched on the fire path only through an
    /// `Arc` — never through `Scheduler` state, because
    /// [`Scheduler::fire_node_into`] runs on a rayon worker on the wide path.
    ///
    /// The write is on the DISABLE arm alone, which is inside the caught-panic
    /// branch, so `step_zero_alloc_test`'s graphs — none of which panic — never
    /// reach it. That is the same structural argument the three cold-path
    /// annotations on the panic-payload renders in `fire_node_into` rest on.
    /// (Spelled in prose rather than by quoting the marker: the hot-path allocation lint
    /// reads a comment block above a line, and a doc block quoting the marker
    /// verbatim registers as an annotation on the field declaration.)
    node_death: Arc<crate::scheduler::node_death::NodeDeathLedger>,
}

impl ScheduledNode {
    /// The ns until this node's `throttle_ms` window expires,
    /// or `None` if it has no rate cap or is not currently deferred by one.
    ///
    /// The DEFER decision is [`crate::graph::runtime::throttle_defers`] — the same free
    /// function the pre-fire closure calls — so this cannot disagree with the
    /// gate about WHETHER the node is throttled; it only adds by how much.
    ///
    /// Both inputs are this scheduler's own canonical observables, read through
    /// the shared atomics the node handle exposes, so there is no second source
    /// of truth for either.
    ///
    /// Saturating: a `last_fire_ns` ahead of `now_ns` (a clock the caller moved
    /// backwards) yields a zero remainder rather than an enormous one, so the
    /// worst case is the polling behaviour this exists to replace, never a node
    /// parked into the far future.
    fn throttle_remaining_ns(&self, now_ns: u64) -> Option<u64> {
        let throttle_ns = self.throttle_ns?;
        let prior_fires = self.fire_count.load(Ordering::Relaxed);
        let last_fire_ns = self.last_fire_ns.load(Ordering::Relaxed);
        if !crate::graph::runtime::throttle_defers(prior_fires, now_ns, last_fire_ns, throttle_ns) {
            return None;
        }
        Some(
            last_fire_ns
                .saturating_add(throttle_ns)
                .saturating_sub(now_ns),
        )
    }
}

/// Clock storage: a `VirtualClock` (advanceable) or any read-only
/// `Arc<dyn Clock>`.
///
/// We store the `VirtualClock` separately so `step()` can call `advance()`
/// without trait downcasting. The `Real` arm holds any read-only clock whose
/// time the scheduler observes but never advances — `RealClock` (live kernel
/// monotonic time) OR an `ExternalClock` driven by an external master (e.g. a
/// sim's published `/clock`), both reached via `with_clock`.
///
/// The `Barrier` arm holds a `VirtualClock` EXACTLY like
/// `Virtual`, but the INTENT differs — it is the deterministic LIVE gating
/// clock. `Virtual` means "advanced by a polled/replay `step(fixed_delta)`";
/// `Barrier` means "advanced by the LIVE loop's handed logical quantum (the
/// cross-process barrier-handed quantum, or the single-process graph-derived
/// quantum) — NEVER wall elapsed, NEVER a no-op." The default live loop advances
/// the gating clock by WALL-clock elapsed (run-dependent → `fire_time_ns`
/// non-deterministic); the deterministic-live path instead advances it by a
/// fixed logical quantum.
///
/// IMPORTANT — what actually chooses the advance source: `ClockInner` is PRIVATE
/// with NO accessor, so the runtime CANNOT read which arm is in play. The advance
/// source is selected entirely by the runtime's `GraphRuntime.live_gating_quantum`
/// and the `step_live(gating, wall)` split (`Some(quantum)` advances the gating
/// clock by the quantum; `None` advances by wall elapsed). The `Barrier` arm is
/// therefore a DOCUMENTARY / intent marker — and a forward hook for the
/// cross-process spine-dep (e), which may branch on it — NOT a flag the live loop
/// reads to pick its delta. Its one load-bearing job is to keep a
/// deterministic-live `VirtualClock` out of the no-op `Real` arm: a `VirtualClock`
/// is placed in `Barrier` ONLY via the dedicated
/// [`Scheduler::with_barrier_clock`] constructor — NEVER upcast to `dyn Clock`
/// (that would be the `Real` arm and lose the advanceability).
enum ClockInner {
    Virtual(Arc<VirtualClock>),
    Barrier(Arc<VirtualClock>),
    Real(Arc<dyn Clock>),
}

impl ClockInner {
    /// Advance the per-step gating clock by `delta_ns` and return the new time.
    ///
    /// The `Virtual` arm is the GATING clock (Period nodes gate
    /// against it), so it advances by a run-INDEPENDENT logical quantum — the
    /// fixed polled delta OR a replay-bag RECORDED execution duration — NEVER a
    /// wall-derived or max-of-peers value (that would break replay = live,
    /// Principle #7). The dedicated `advance_by_recorded` call names that
    /// contract at the seam; see [`VirtualClock::advance_by_recorded`] for the
    /// canonical text. The `Real` arm is the no-op live path.
    fn advance(&self, delta_ns: u64) -> u64 {
        match self {
            ClockInner::Virtual(c) => c.advance_by_recorded(delta_ns),
            // Mechanically identical to the `Virtual`
            // arm. The contract on what `delta_ns` means here is enforced by the
            // CALLER (the live loop): it hands the LIVE logical quantum,
            // NOT wall elapsed. So the `Barrier` gating clock advances
            // deterministically and is NEVER a no-op (unlike `Real`).
            ClockInner::Barrier(c) => c.advance_by_recorded(delta_ns),
            // Read-only clocks (RealClock / ExternalClock) are not advanced by
            // the scheduler — their time is driven externally (the kernel
            // monotonic clock, or an external master). Return current time.
            ClockInner::Real(c) => c.now_ns(),
        }
    }

    fn now_ns(&self) -> u64 {
        match self {
            ClockInner::Virtual(c) => c.now_ns(),
            ClockInner::Barrier(c) => c.now_ns(),
            ClockInner::Real(c) => c.now_ns(),
        }
    }
}

/// The FIRE part of a `decide_node` evaluation, deferred
/// so the runtime can interleave a fire-gated input snapshot BETWEEN deciding
/// which nodes fire (`decide_fires`) and actually firing them (`tick_decided`).
///
/// `decide_node` performs all the state mutations the fused `evaluate_node`
/// would (advancing `next_fire_ns`, clearing `pending_data_count` /
/// `sync_input_timestamps` / `external_triggered`) and returns the recipe for
/// the deferred fire. `tick_node` consumes it. Splitting decide from tick is
/// byte-identical to the fused form because nothing between the two reads the
/// node's scheduler state (levelization guarantees no same-level node triggers
/// another).
///
/// `pub(crate)` to match `FireDecision::kind`'s visibility (it is a
/// crate-internal handoff between the scheduler and the runtime's level loop).
///
/// `Debug` for tracing / test assertions on the crate-internal handoff (zero
/// runtime cost — derived, never on the hot fire path).
///
/// `FireKind` is now `Copy` (all fields are `Copy`) — the prior
/// `Period { fire_times: Vec<u64> }` variant heap-allocated one `Vec` per
/// Period decide on the moat path. With the `Vec` gone, moving a `FireDecision`
/// (which is NOT `Copy` — it owns an `Arc<str>` node id) is alloc-free. The
/// catch-up burst is now described ARITHMETICALLY (first fire + count +
/// interval), and `tick_node[_collect]` reconstructs the exact same `fire_time`
/// sequence via a running add (byte-identical to the old collected `Vec`).
#[derive(Debug, Clone, Copy)]
pub(crate) enum FireKind {
    /// Sync / External: a single fire at the step's `current_time_ns`.
    Single { fire_time_ns: u64 },
    /// Data: a (possibly multi-fire) BURST at the step's
    /// `current_time_ns`, one fire per queued frame, all sharing that one fire
    /// time (the `Period` catch-up precedent: a burst served inside one step is
    /// served AT that step, not at invented sub-step times).
    ///
    /// `fire_count` is the number of arrivals the scheduler had already been
    /// SIGNALLED about and has authorised for this step — `min(pending, cap)`.
    /// It is a FLOOR on the burst, not a ceiling: on the `DrainSource::Unified`
    /// path the boundary drain can only pop and freeze ONE frame, so it signals
    /// one arrival however many are queued, and the true burst length is
    /// UNKNOWABLE up front (the emptiness probe is a bool; counting would mean
    /// popping, and only one frame can be held frozen at a time). `tick_node`
    /// therefore REFILLS between fires through the node's [`TriggerRefill`]
    /// hook, which pops the next frame exactly as the boundary drain did.
    ///
    /// Why this is not `Single`: with one fire per step a data-trigger consumer
    /// is throughput-capped at one frame per scheduler step. Behind an in-graph
    /// `period_ms = 1` producer the live loop steps at ~1 kHz, so the first
    /// catch-up burst that publishes k > 1 frames in one step puts the consumer
    /// k frames behind PERMANENTLY — one in, one out, forever — until the input
    /// queue fills and evicts, after which every frame it serves is a whole
    /// queue-depth old. That is measurable: it took the CLI e2e graph-latency
    /// gate from ~25 µs p50 to 8.75 ms p50 with max ≈ depth × period.
    Data { fire_time_ns: u64, fire_count: u32 },

    /// A per-set Sync burst — up to `max_sets` COMPLETE aligned sets
    /// served in ONE step, one fire each, in set order.
    ///
    /// `max_sets` is a CEILING, not a count, and that asymmetry with
    /// [`Self::Data`] is forced rather than chosen: `fire_count` is known
    /// because each arrival was SIGNALLED, while sets cannot be counted without
    /// popping — the matcher only learns a set exists by forming it. So the
    /// burst loop re-aligns between fires and stops when the next alignment is
    /// incomplete, and this number only bounds how many it may serve before the
    /// remainder carries (through `sync_backlog_hint` + due-NOW) to a later
    /// step. It is `DATA_PENDING_CARRY_CLAMP` — the SAME constant Data's cap
    /// derives from, deliberately not a third one.
    Sync { fire_time_ns: u64, max_sets: u32 },
    /// Period: a (possibly multi-fire) catch-up burst, described arithmetically
    /// (zero-alloc). The fire times are exactly `first_fire_ns,
    /// first_fire_ns + interval_ns, …, first_fire_ns + (fire_count-1) *
    /// interval_ns` — IDENTICAL to the sequence the old `decide_node` collected
    /// into `fire_times_buf` (it pushed `*next_fire` then `*next_fire +=
    /// interval_ns` for `fire_count` iterations). `tick_node[_collect]`
    /// reconstructs them with the SAME running add (not an `i * interval`
    /// multiply) so overflow semantics match the original byte-for-byte. The
    /// per-fire `pre_fire_check` re-check (the block-overflow clamp) is
    /// re-evaluated in `tick_node` per reconstructed `fire_time`, exactly as
    /// the fused form did — so the catch-up re-check counter side-effect runs
    /// in `tick`, not `decide`.
    Period {
        first_fire_ns: u64,
        fire_count: u32,
        interval_ns: u64,
        current_time_ns: u64,
    },
    /// A TRACE-DRIVEN fire — the recording says this node fired
    /// `fire_count` times this step, starting at `first_fire_ns` and stepping by
    /// `interval_ns`. Synthesized by [`Scheduler::take_planned_fire`] from an
    /// installed [`ReplayFirePlan`] INSTEAD of evaluating the node's trigger.
    ///
    /// # Why this is not a synthetic `Period`
    ///
    /// A `Period` decision re-enters `tick_node`'s per-catch-up
    /// `run_pre_fire_check` re-check, which has a COUNTER SIDE-EFFECT (the block
    /// gate's `block_fires_deferred_count` bump + its once-per-regime warn) and a
    /// `next_fire_ns` REWIND on defer. A trace-driven fire must pay neither: the
    /// recording already states the fire happened, so re-deriving a clamp against
    /// live queue occupancy can only ever CONTRADICT it (and, cross-rank, against
    /// occupancy the recording never captured). Re-encoding a replay
    /// fire as `Period` would also mean a `Data` or `Sync` node's replayed fire
    /// silently acquiring Period's catch-up semantics.
    ///
    /// # The arithmetic descriptor is TOTAL over producible traces
    ///
    /// Per (node, step) the recorded fire times are ALWAYS an arithmetic
    /// progression: a `Period` catch-up burst advances by a running add of one
    /// interval, a `Data` burst stamps every fire with the step's
    /// `current_time_ns` (stride 0), and `Single` is one fire. So one
    /// `(first, count, interval)` triple describes every shape the live
    /// scheduler can record — the same argument [`FireKind::Period`] makes for
    /// its own arithmetic form, and the reason a plan entry is per (node, step)
    /// rather than a list of instants.
    Replay {
        first_fire_ns: u64,
        fire_count: u32,
        interval_ns: u64,
    },
}

/// A decided fire awaiting `tick_decided`. Carries the
/// node's interned id (one atomic refcount bump, reused for the trace) and its
/// stable insertion index (valid for the remainder of this step — the
/// scheduler's `IndexMap` is not mutated between `decide_fires` and
/// `tick_decided`) so `tick_decided` re-fetches the node by O(1) index.
///
/// `Debug` for tracing / test assertions on the crate-internal handoff (zero
/// runtime cost — derived, never on the hot fire path).
#[derive(Debug)]
pub(crate) struct FireDecision {
    pub(crate) node_id: Arc<str>,
    /// The node's stable insertion index into the scheduler's `IndexMap`. It is
    /// a CALLER OBLIGATION that this idx stays valid: it is correct ONLY within
    /// the SAME `step()` call that produced this decision — the scheduler's
    /// `IndexMap` must NOT be mutated (no `add_node` / removal / reorder)
    /// between [`Scheduler::decide_fires`] and the consuming
    /// [`Scheduler::tick_decided`] / [`Scheduler::tick_decided_parallel`].
    /// BOTH consumers re-fetch the node by this O(1) index, so both rely on
    /// this invariant for correctness; a mutation in between would silently
    /// fire the wrong node (`tick_decided_parallel` `debug_assert!`s the
    /// idx/IndexMap stay in sync, but only catches a count mismatch, not a
    /// same-count reorder).
    pub(crate) idx: usize,
    pub(crate) kind: FireKind,
}

/// The index-keyed scratch entry [`Scheduler::tick_decided_parallel`]
/// fills (one per firing node, addressed by its stable insertion `idx`) so the
/// within-level fire passes carry no per-step `HashMap`. It replaces the old
/// `decided_by_idx: HashMap<usize, (pos, Arc<str>, FireKind)>`: `decision_slots`
/// is a `Vec<Option<DecisionSlot>>` sized to `nodes.len()` and reused (cleared +
/// `resize_with`) every level, so steady-state stepping allocates zero times.
///
/// `serial` records whether the node is in the level executor's `serial_ids`
/// (a `!performs_input_snapshot()` node that reads live and so must fire on the
/// calling thread BEFORE any parallel producer publishes — invariant B). It is
/// resolved once here (one `HashSet` lookup per firing node) so neither fire
/// pass re-consults `serial_ids`.
///
/// `Send + Sync` (`Arc<str>` is `Sync`, `FireKind` is `Copy`, `bool` is `Sync`)
/// so a shared `&Vec<Option<DecisionSlot>>` can be captured by the rayon
/// `for_each` closure on the wide path (invariant G). The `const _: fn()`
/// Send/Sync pin below enforces this so a future non-`Send`/`Sync` field fails
/// HERE, not as a confusing rayon call-site error.
///
/// `Debug` so the orphan / idx-desync guards (and any future diagnostics) can
/// `?`-print a slot, matching its sibling `FireDecision` / `FireKind`.
#[derive(Debug)]
struct DecisionSlot {
    /// The id `decide_fires` minted this fire for. Kept ALONGSIDE the node's own
    /// `node_id` ON PURPOSE: it is the independent reorder oracle for the
    /// same-count idx/reorder `debug_assert_eq!` in every fire pass (a slot at
    /// position `i` must still match the node at insertion idx `i`). Dropping it
    /// would delete that guard. `Arc::clone` to fill it is a refcount bump.
    node_id: Arc<str>,
    kind: FireKind,
    serial: bool,
}

/// Compile-time pin (invariant G): `DecisionSlot` must stay `Send +
/// Sync` (a shared `&Vec<Option<DecisionSlot>>` crosses into rayon workers) and
/// `ScheduledNode` must stay `Send` (`par_values_mut` hands each worker a
/// disjoint `&mut ScheduledNode`). A future field that breaks either bound fails
/// to compile HERE with a clear pointer, not as an opaque trait-bound error at
/// the `pool.install` call site.
const _: fn() = || {
    fn assert_send_sync<T: Send + Sync>() {}
    fn assert_send<T: Send>() {}
    assert_send_sync::<DecisionSlot>();
    assert_send::<ScheduledNode>();
};

/// A sink for trace entries produced by a single node fire. Lets the
/// ONE fire body ([`Scheduler::fire_node_into`]) serve BOTH the serial path
/// (push directly into the scheduler's `VecDeque` trace with ring-buffer
/// truncation — zero alloc) and the rayon parallel path (collect into an owned
/// `Vec` fragment merged after the join). Byte-identical: both apply the SAME
/// per-entry truncation cadence at their respective merge points (per-entry in
/// the serial `RingTraceSink`; once, after the decision-ordered merge, for the
/// parallel `Vec` fragments).
trait TraceSink {
    fn push_entry(&mut self, entry: TraceEntry);
}

/// Parallel-path sink: a plain owned fragment (truncation applied later, once,
/// at the decision-ordered merge in [`Scheduler::tick_decided_parallel`]).
impl TraceSink for Vec<TraceEntry> {
    fn push_entry(&mut self, entry: TraceEntry) {
        self.push(entry);
    }
}

/// The scheduler-side trace-ring hook — a minted
/// [`crate::trace_ring::TraceRingProducer`] plus the node-id → manifest-index
/// mapping DERIVED FROM THE SAME node-id table the ring owner's manifest was
/// encoded from (both handed in together by
/// [`Scheduler::set_trace_ring_producer`], so record `node_idx` and the bagged
/// manifest can never disagree).
///
/// SPSC contract: the producer is only ever touched through `&mut Scheduler`
/// — the serial [`RingTraceSink::push_entry`] choke point, which runs on the
/// step-calling thread only (the serial fire funnel, and the parallel pass 3
/// fragment merge after the rayon join returns). Rayon workers fire into
/// per-node `trace_fragment` Vec sinks and never see this.
#[cfg(unix)]
struct TraceRingHook {
    producer: crate::trace_ring::TraceRingProducer,
    /// node_id → manifest index (the manifest table's order).
    node_idx: std::collections::HashMap<String, u32>,
    /// Node ids already warned about as missing from the manifest (a
    /// manifest/scheduler desync — loud once PER DISTINCT node, never
    /// per-fire loud). Bounded by the scheduler's node count.
    warned_unmapped: std::collections::HashSet<String>,
    /// FIRE records DROPPED because their node id was missing from the
    /// manifest. Exposed via [`Scheduler::trace_ring_unmapped_count`] so the
    /// recording CLI can make a desynced (incomplete) bag VISIBLY wrong at
    /// run exit instead of quietly finalizing it. Fire records only (the counters
    /// are split by record kind): kind-6 read-outcome drops ride
    /// `unmapped_read_outcomes` below, so this counter's callers report
    /// fire-record loss exactly rather than a conflated total.
    unmapped_dropped: u64,
    /// Kind-6 read-outcome records dropped for the
    /// same manifest-desync reason, counted separately from the fire counter
    /// above. Exposed via [`Scheduler::trace_ring_unmapped_read_outcomes`];
    /// the run-exit report prints the two distinctly (fire loss = a
    /// scheduler-trace hole, read-outcome loss = a read-log hole).
    unmapped_read_outcomes: u64,
}

/// Which record kind an unmapped-node drop was —
/// selects the desync counter the shared [`TraceRingHook::resolve_node_idx`]
/// bumps, so fire-record loss is never conflated with read-outcome loss. The
/// once-per-distinct-node warn latch stays SHARED (one desynced node id is one
/// bug however many record kinds it drops).
#[cfg(unix)]
#[derive(Clone, Copy)]
enum UnmappedRecordKind {
    Fire,
    ReadOutcome,
}

#[cfg(unix)]
impl TraceRingHook {
    /// Push one fire record. WAIT-FREE + zero-alloc on the mapped path: a
    /// `HashMap` lookup by `&str`, a 40-byte stack encode, and the wait-free
    /// [`crate::trace_ring::TraceRingProducer::push`] (no alloc, no lock, no
    /// syscall — pinned by `shm_ring_zero_alloc_test`). Overrun accounting
    /// lives inside the ring (`records_lost`), so there is no error path here.
    /// The UNMAPPED arm (a manifest/scheduler desync — a bug, never a normal
    /// state) is cold: it counts the dropped record and warns once per
    /// distinct node id (the first sighting allocates for the latch set).
    fn push_fire(&mut self, entry: &TraceEntry) {
        let Some(idx) = self.resolve_node_idx(entry.node_id.as_ref(), UnmappedRecordKind::Fire)
        else {
            return;
        };
        self.producer.push(&crate::trace_ring::TraceRingRecord {
            step: entry.step,
            fire_time_ns: entry.fire_time_ns,
            duration_ns: entry.duration_ns,
            node_idx: idx,
            global_level: entry.global_level as u32,
            record_type: crate::trace_ring::RECORD_TYPE_FIRE,
            // The rank subfield stays 0 on the ring (bagd co-stamps the
            // ring's header rank in, preserving this bit); the DISCARD bit is set
            // here iff this fire committed 0 of its loaned outputs. Set ONLY on
            // FIRE records (this is `push_fire`; `push_step_boundary` writes 0).
            reserved: if entry.discarded {
                crate::trace_ring::TRACE_DISCARD_BIT
            } else {
                0
            },
        });
    }

    /// Push the per-step
    /// [`crate::trace_ring::RECORD_TYPE_STEP_BOUNDARY`] record — `step` is
    /// the 0-based step index, `advanced_ns` the step's post-advance clock
    /// (what `begin_step` returns). `node_idx`/`global_level`/`duration_ns`
    /// are 0 by contract. WAIT-FREE + zero-alloc (a 40-byte stack encode +
    /// the wait-free ring push) — one extra push per step on the recording
    /// path, nothing on the recording-OFF path (the hook is `None`).
    fn push_step_boundary(&mut self, step: u64, advanced_ns: u64) {
        self.producer.push(&crate::trace_ring::TraceRingRecord {
            step,
            fire_time_ns: advanced_ns,
            duration_ns: 0,
            node_idx: 0,
            global_level: 0,
            record_type: crate::trace_ring::RECORD_TYPE_STEP_BOUNDARY,
            reserved: 0,
        });
    }

    /// Push one kind-6 READ-OUTCOME record (the staged form
    /// → the `TraceRingRecord::read_outcome` packing, ONE assembly site).
    /// Same wait-free + zero-alloc profile as [`Self::push_fire`]; the
    /// UNMAPPED-node arm shares its desync accounting/warn.
    ///
    /// Returns whether the record was really WRITTEN.
    /// The UNMAPPED-node arm drops it, and the OVERFLOW MARKER's debt is
    /// retired only on a confirmed hand-over (see
    /// `ReadOutcomeStage::drain_into`), so "did this land?" has to cross back.
    fn push_read_outcome(
        &mut self,
        step: u64,
        node_id: &str,
        input_idx: u16,
        rec: &crate::read_outcome::StagedReadOutcome,
    ) -> bool {
        let Some(idx) = self.resolve_node_idx(node_id, UnmappedRecordKind::ReadOutcome) else {
            return false;
        };
        // The two ANNOTATION kinds carry a DIFFERENT aux word
        // from a read (a per-window drop count / the full 64-bit producer
        // token), so each has its own assembly site in `trace_ring` and the
        // dispatch happens HERE — a `read_outcome` call with an annotation
        // kind would silently write the read packing.
        //
        // The CALL-SITE role rides `rec` and is forwarded down
        // EVERY arm — the staging site is what knows it, and this dispatch
        // must not have an opinion about it (a `Truncated` marker's role is
        // its overflowed STAGE's, which only the stage could supply).
        let record = match rec.kind {
            crate::read_outcome::ReadOutcomeKind::Truncated => {
                crate::trace_ring::TraceRingRecord::read_outcome_truncated(
                    step, idx, input_idx, rec.popped, rec.role,
                )
            }
            crate::read_outcome::ReadOutcomeKind::Producer => {
                crate::trace_ring::TraceRingRecord::read_outcome_producer(
                    step,
                    idx,
                    input_idx,
                    rec.served_wire_seq(),
                    // A `Producer` record always carries its token (it is
                    // built in ONE place); 0 is the never-taken defensive
                    // arm, and resolves to `foreign(0000000000000000)`
                    // offline rather than to a wrong publisher.
                    rec.token.unwrap_or(0),
                    rec.role,
                )
            }
            _ => crate::trace_ring::TraceRingRecord::read_outcome(
                step,
                idx,
                input_idx,
                rec.kind,
                rec.served_wire_seq(),
                // Popped and the fold count travel as ONE value —
                // they are the two halves of the aux word. A record the stage
                // never folded carries 1, which is what an older reader's
                // structurally-zero high bits also decode to.
                crate::trace_ring::ReadRun::folded(rec.popped, rec.run_count),
                rec.role,
            ),
        };
        self.producer.push(&record);
        true
    }

    /// The shared node-id → manifest-index resolution for every record kind
    /// this hook pushes. The UNMAPPED arm (a manifest/scheduler desync — a
    /// bug, never a normal state) is cold: it counts the dropped record on
    /// the caller-declared per-kind counter (`kind` — fires and
    /// read outcomes are reported distinctly at run exit) and warns once per
    /// distinct node id (the first sighting allocates for the latch set).
    fn resolve_node_idx(&mut self, node_id: &str, kind: UnmappedRecordKind) -> Option<u32> {
        if let Some(&idx) = self.node_idx.get(node_id) {
            return Some(idx);
        }
        match kind {
            UnmappedRecordKind::Fire => self.unmapped_dropped += 1,
            UnmappedRecordKind::ReadOutcome => self.unmapped_read_outcomes += 1,
        }
        if !self.warned_unmapped.contains(node_id) {
            // hot-path-alloc-ok: cold: guarded by the `warned_unmapped.contains` check
            // immediately above — one allocation per DISTINCT unmapped node id, once, then
            // never again (and an unmapped id is a manifest/scheduler desync, never a normal
            // state). The enclosing fn IS hot — it resolves every pushed record — so this is
            // the LINE form deliberately.
            self.warned_unmapped.insert(node_id.to_string());
            tracing::warn!(
                node_id = %node_id,
                "trace ring: node id missing from the recording manifest — its \
                 trace records (fires + read outcomes) are NOT bagged \
                 (manifest/scheduler desync; report this). Warning once per node; \
                 the run continues and the per-kind drop counts are reported at \
                 run exit."
            );
        }
        None
    }
}

/// Non-unix stub: the trace ring is POSIX SHM (`trace_ring` is
/// `#![cfg(unix)]`), so on other targets the hook type exists only to keep
/// the `Option<&mut TraceRingHook>` plumbing signature-identical — it is
/// never constructed (no setter exists), so the slot is always `None`.
#[cfg(not(unix))]
struct TraceRingHook;

#[cfg(not(unix))]
impl TraceRingHook {
    fn push_fire(&mut self, _entry: &TraceEntry) {}
    fn push_step_boundary(&mut self, _step: u64, _advanced_ns: u64) {}
    fn push_read_outcome(
        &mut self,
        _step: u64,
        _node_id: &str,
        _input_idx: u16,
        _rec: &crate::read_outcome::StagedReadOutcome,
    ) -> bool {
        false
    }
}

/// Serial-path sink: push straight into the shared ring buffer, applying the
/// `max_trace_entries` pop-front-then-push-back truncation per entry — exactly
/// the earlier inline `fire_node` tail (byte-identical), with no local Vec.
/// Also bumps the scheduler's monotonic `entries_appended` counter per append
/// (the cap-immune fire signal — see `Scheduler::entries_appended`).
///
/// Additionally the SINGLE trace-ring choke point — when a recording
/// hook is installed, EVERY appended entry is pushed to the SPSC trace ring
/// FIRST, before (and never gated by) the in-memory `max_trace_entries`
/// eviction: the bag's trace is complete by contract even when the in-memory
/// trace is capped. Both append paths funnel here (the serial fire funnel
/// directly; the parallel pass 3 merge constructs this sink too).
struct RingTraceSink<'a> {
    trace: &'a mut VecDeque<TraceEntry>,
    max_trace_entries: Option<usize>,
    entries_appended: &'a mut u64,
    ring: Option<&'a mut TraceRingHook>,
}

impl TraceSink for RingTraceSink<'_> {
    fn push_entry(&mut self, entry: TraceEntry) {
        // Ring push FIRST — ungated by the in-memory cap below.
        if let Some(hook) = self.ring.as_mut() {
            hook.push_fire(&entry);
        }
        if let Some(limit) = self.max_trace_entries {
            if self.trace.len() >= limit {
                self.trace.pop_front();
            }
        }
        self.trace.push_back(entry);
        *self.entries_appended += 1;
    }
}

/// One level-end-merged read outcome, as
/// collected by the scheduler's IN-MEMORY read-outcome sink — the replay
/// engine's collection seam. The recording path pushes the same merged stream
/// into the SPSC trace ring as kind-6 records; replay has no POSIX-SHM ring
/// (and needs none), so the level-end merge (`merge_read_outcomes` — private)
/// generalizes its push target exactly the way the private `TraceSink` trait
/// splits Vec-vs-Ring for fires. Offline-path allocation discipline applies
/// (one owned entry per merged record) — the sink is never installed on a
/// live/recording run.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MergedReadOutcome {
    /// The 0-based logical step the read happened on (the same stamp a
    /// recorded kind-6 record carries).
    pub step: u64,
    /// The consuming node's id.
    pub node_id: String,
    /// The input's index into its node's wiring-order input table
    /// (`GraphRuntime::read_log_input_names` resolves it to a name).
    pub input_idx: u16,
    /// The staged outcome (kind + served wire sequence + popped count), in
    /// the same wire terms a kind-6 record encodes.
    pub outcome: crate::read_outcome::StagedReadOutcome,
}

/// ONE node's trace-driven fires for ONE step — the caller-facing
/// half of [`Scheduler::set_replay_fire_plan`].
///
/// `node_id` is borrowed (the engine holds the recording's own strings), and the
/// three numbers are exactly `FireKind::Replay`'s: the first recorded fire
/// instant, how many fires the recording holds for this node in this step, and
/// the stride between them (`0` for a `Data` burst or a single fire — see
/// `FireKind::Replay` for why one triple is total over producible traces).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReplayFire<'a> {
    /// The node the recording says fired.
    pub node_id: &'a str,
    /// The first recorded `fire_time_ns` for this node in this step.
    pub first_fire_ns: u64,
    /// How many fires the recording holds. Zero is a caller bug, not "no fire":
    /// a node that did not fire simply carries no entry.
    pub fire_count: u32,
    /// The stride between consecutive recorded fire times (`0` when they are
    /// all equal, i.e. every shape but a `Period` catch-up burst).
    pub interval_ns: u64,
}

/// The engine's INTRA-STEP injection callback — invoked between
/// two fires of a node's trace-driven burst, with `(node_id, after_fire)`.
///
/// `after_fire` is 1-BASED and counts COMPLETED fires: `1` means "the node's
/// first fire of this step has returned, and the recording holds a foreign
/// frame published at that instant". Together the pair is the whole key: the
/// planner's slots are keyed by exactly `(producer_node, after_fire)`, so the
/// hook needs nothing else to find the frames it owes.
///
/// # What the hook may do
///
/// Publish, through the engine's REAL injector publisher — never a bypass. It
/// runs with the scheduler `&mut`-borrowed, so it must NOT re-enter the
/// scheduler or the runtime; the signature is what makes that structural
/// (it is handed a name and a count, not a handle). Errors are the hook's own
/// to latch and the caller's to check after `step()` — the
/// `GraphRuntime::set_replay_discards` / `replay_discards_drained` precedent.
///
/// # A panic is CAUGHT, never an unwind out of the step
///
/// The hook runs inside a node's burst, so an unguarded panic would unwind past
/// the node tick two frames away and out of `step()` with no counter and no
/// report. The node-tick `catch_unwind` does NOT cover it — that frame has
/// already returned by the time the pause is consulted — hence the LOCAL catch
/// at the pause seam: the burst continues, the slot's cursor does NOT advance
/// (so it surfaces in [`Scheduler::unconsumed_replay_pauses`], a slot whose
/// frames may have been injected only in part — the hook can panic MID-publish),
/// and the panic is counted by [`Scheduler::replay_hook_panics`]. A hook that
/// can fail should still report its own failure through its own latch — the
/// catch is a floor, not a substitute.
///
/// # What the ENGINE owes this seam
///
/// Both signals are the engine's to read, and the replay engine DOES: it reads
/// [`Scheduler::unconsumed_replay_pauses`] after every step and
/// [`Scheduler::replay_hook_panics`] + [`Scheduler::replay_pause_mismatches`]
/// at the pass epilogue, and a non-zero value REFUSES the pass as an internal
/// error (exit 5) naming the counter — never a verdict over the candidate.
/// Either one means the ENGINE'S OWN injector lost frames the recording holds,
/// so any downstream byte divergence is the harness's, not the candidate's
/// (`cerulion_cli_engine::replay_engine`, the `IntraStepSeamCounters` gate in
/// `run_rank_pass`).
///
/// `Send + Sync` because a wide level's fires can run on the rayon fire pool
/// (see `Scheduler::tick_decided_parallel`, which forces the SERIAL rest walk
/// while pauses are armed so the ORDER stays deterministic, but the bound is
/// what makes the type sound either way).
pub type ReplayInjectionHook = Arc<dyn Fn(&str, u32) + Send + Sync>;

/// ONE intra-step pause — "after this node's `after_fire`-th fire
/// of this step, hand control to the injection hook".
///
/// The caller-facing half of [`Scheduler::set_replay_intra_step_pauses`], and
/// the mirror of [`ReplayFire`]: `node_id` is borrowed (the engine holds the
/// recording's own strings) and the count is copied out at install.
///
/// # Why this exists
///
/// A recorded serve order like `[local fire 1, FOREIGN frame, local fire 2]`
/// on one shared topic cannot be reproduced by a before-step injection bucket:
/// everything the bucket holds lands in the consumer FIFO ahead of BOTH local
/// fires. The pause is the vocabulary for "inject after the k-th local fire".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IntraStepPause<'a> {
    /// The node whose burst pauses.
    pub node_id: &'a str,
    /// How many of that node's fires must have COMPLETED first. 1-based; `0` is
    /// refused at install (a "pause before any fire" is the ordinary before-step
    /// bucket, which already exists).
    pub after_fire: u32,
}

/// One node's installed intra-step pauses, and the bookkeeping the
/// burst consumes them with.
///
/// # Why this lives on the NODE and not in a `Vec<Option<_>>` beside the fire plan
///
/// [`ReplayFirePlan`] is consulted in `take_planned_fire`, which has `&mut
/// self`. The pauses are consumed in [`Scheduler::tick_replay_burst`], which is
/// an ASSOCIATED fn over a single `&mut ScheduledNode` — the shape it must have,
/// because it is reached from SIX call sites across three tick paths (the flat
/// `evaluate_one`, the level `tick_decided`, the block-fused
/// `evaluate_nodes_fused`, `evaluate_node`, and both `tick_node_into` sites).
/// Threading a scheduler-owned slot table down all six would need a new
/// parameter on four already-`too_many_arguments` signatures AND an
/// interior-mutable cursor (the parallel path holds the plan by `&`) — and
/// missing any one site would make the seam INERT on exactly the shape it is
/// for (`evaluate_nodes_fused` serves the `block`-involved nodes).
///
/// Storing it per node keeps the reuse property the slot table exists for —
/// `after_fires` is CLEARED, never dropped, so its capacity is reused install
/// over install and a paused step allocates nothing at steady state — while
/// making every tick path carry the seam by construction. It follows
/// `trigger_refill` / `replay_refill_shortfalls`, which are per-node for the
/// same reason.
#[derive(Debug, Default)]
struct NodeReplayPauses {
    /// SORTED ASCENDING and duplicate-free (both enforced at install), so the
    /// per-fire consult is one indexed compare — no scan, no allocation.
    after_fires: Vec<u32>,
    /// How many entries the burst has REACHED and delivered this step. Entries
    /// at or beyond it when the step ends are reported by
    /// [`Scheduler::unconsumed_replay_pauses`].
    cursor: usize,
    /// The 0-based logical step `after_fires` was installed for. A burst running
    /// at a DIFFERENT step delivers NOTHING (the stale-plan rule
    /// [`ReplayFirePlan`] states: never a fall-through).
    step: u64,
    /// UNCONDITIONAL count of bursts that consulted a STALE list (Principle #3:
    /// the counter is the truth, not the log). Never reset by a re-install.
    mismatches: u64,
    /// Once-per-install latch for the stale-list `error!`: the consult runs per
    /// FIRE, so an unlatched log would flood a stale-plan burst.
    mismatch_reported: bool,
    /// UNCONDITIONAL count of injection-hook invocations that PANICKED (the
    /// `mismatches` rule: the counter is the truth, not the log). Never reset by
    /// a re-install. A panicking hook's slot is left UNCONSUMED, so this and
    /// [`Scheduler::unconsumed_replay_pauses`] name the same loss twice — once
    /// with its cause, once with its position.
    ///
    /// Deliberately WITHOUT a `mismatch_reported`-style log latch: unlike the
    /// stale-list arm (which is re-decided on every fire of the burst), a panic
    /// PARKS the cursor, so the head entry can never match a later, strictly
    /// larger `completed` — the hook is entered AT MOST ONCE per (node,
    /// install) and there is no flood to suppress. See
    /// `Scheduler::consume_intra_step_pause`, where that bound is argued at the
    /// log site.
    hook_panics: u64,
}

impl NodeReplayPauses {
    /// Retire the previous install's entries, keeping the `Vec`'s capacity (the
    /// zero-alloc property) and the cumulative `mismatches` / `hook_panics`
    /// counters.
    fn retire(&mut self) {
        self.after_fires.clear();
        self.cursor = 0;
        self.mismatch_reported = false;
    }
}

/// One installed plan entry (the scheduler's own copy of a
/// [`ReplayFire`], keyed by the node's stable insertion `idx`).
#[derive(Debug, Clone, Copy)]
struct PlannedFire {
    first_fire_ns: u64,
    fire_count: u32,
    interval_ns: u64,
    /// Set when a decide seam actually synthesized this fire. A plan entry that
    /// finishes the step UNCONSUMED is a fire the recording holds and this run
    /// did NOT perform — reported by [`Scheduler::unconsumed_replay_fires`]
    /// rather than silently dropped (the never-fire class).
    consumed: bool,
}

/// The installed trace-driven fire plan for ONE step.
///
/// `slots` is indexed by the node's stable `IndexMap` insertion index — the SAME
/// `idx` [`FireDecision::idx`] carries — so a lookup is O(1) and the buffer is
/// re-used step over step (cleared + `resize_with`, never re-allocated: the
/// `decision_slots` precedent, and what keeps `step_zero_alloc_test`'s contract
/// true of a plan-driven step too).
#[derive(Debug, Default)]
struct ReplayFirePlan {
    /// The 0-based logical step this plan describes. A decide seam running at a
    /// DIFFERENT step is a caller ordering bug and fires NOTHING (never a
    /// fall-through to live deciding, which would fabricate fires the recording
    /// does not hold).
    step: u64,
    slots: Vec<Option<PlannedFire>>,
    /// Once-per-plan latch for the step-mismatch report: the seams run per
    /// level, so an unlatched `error!` would flood a stale-plan run.
    mismatch_reported: bool,
    /// UNCONDITIONAL count of decide seams that ran against a stale plan
    /// (Principle #3: the counter is the truth, not the log).
    mismatches: u64,
}

/// Deterministic scheduler for node execution.
///
/// Nodes are evaluated in insertion order (`IndexMap`) at each `step()` call.
/// Use `VirtualClock` for deterministic testing and replay.
pub struct Scheduler {
    clock: ClockInner,
    nodes: IndexMap<String, ScheduledNode>,
    trace: VecDeque<TraceEntry>,
    max_trace_entries: Option<usize>,
    /// Reusable scratch buffer that [`Self::decide_fires`] fills with
    /// this level's fire decisions (cleared, not reallocated, per call). The
    /// runtime borrows it via [`Self::take_decisions`] / hands it back via
    /// [`Self::return_decisions`] so its heap capacity is reused every
    /// level/step — eliminating the per-level `Vec<FireDecision>` allocation on
    /// the moat path.
    fire_decisions: Vec<FireDecision>,
    /// Reusable index-keyed scratch for the multi-fire
    /// [`Self::tick_decided_parallel`] passes (sized to `nodes.len()`, addressed
    /// by each firing node's stable insertion `idx`). Cleared + `resize_with`'d
    /// (not reallocated) per multi-fire level, so it eliminates the old
    /// per-level `HashMap<usize, …>` allocation. `None` at an index = that node
    /// did not fire this level; `Some(slot)` carries the node id, fire kind, and
    /// resolved serial-gating flag. Untouched by the `decisions.len() <= 1`
    /// fast-path (the moat chain never sizes it).
    decision_slots: Vec<Option<DecisionSlot>>,
    /// B-dur: master gate for per-node tick-duration recording.
    /// Applied to new nodes in `add_node` and to all existing nodes by
    /// [`Self::set_record_tick_durations`]. Default false.
    record_tick_durations: bool,
    /// Monotonic count of `begin_step` calls — i.e. ONE
    /// past the 0-based index of the logical step currently executing. Bumped
    /// EXACTLY ONCE at the top of [`Self::begin_step`], the SINGLE per-step seam
    /// both the flat [`Self::step`] and the `GraphRuntime::step` level executor
    /// route through. The step index stamped onto every `TraceEntry.step` is
    /// [`Self::current_step`] (`== this − 1`), the cross-process trace merge's
    /// primary sort key. DELIBERATELY NOT reset by [`Self::clear_trace`]: it is
    /// an ABSOLUTE logical-step counter, so warmup-then-clear offsets the
    /// monolith and every partition IDENTICALLY and the cross-process firewall
    /// still holds. Default 0 (no step has begun yet).
    steps_begun: u64,
    /// MONOTONIC count of `TraceEntry`s APPENDED to
    /// `trace` over the scheduler's lifetime — incremented once per append at
    /// both trace-append choke points (`RingTraceSink::push_entry` on the
    /// serial path; the pass 3 fragment merge in `tick_decided_parallel` on
    /// the parallel path), INDEPENDENT of ring-buffer eviction. This is the
    /// cap-immune "did the last step fire?" signal for the live loop's
    /// spin-then-block (`GraphRuntime::live_step`): once a capped ring is FULL
    /// `trace.len()` is pinned at the cap (every firing step pops+pushes), so
    /// a before/after length delta reads 0 forever — silently collapsing the
    /// spin budget on exactly the long-running graphs the production cap
    /// targets. DELIBERATELY NOT reset by [`Self::clear_trace`] (mirrors the
    /// `steps_begun` precedent above): it is an absolute append counter and
    /// callers only ever difference it. Default 0.
    entries_appended: u64,
    /// MONOTONIC count of the levels whose REST the
    /// [`RestWalk::InsertionOrder`] constraint ALONE took off the rayon path —
    /// bumped in [`Self::tick_decided_parallel`] exactly when
    /// [`Self::rest_driver`] returns [`RestDriver::SerialByConstraint`]. Three
    /// causes narrow a REST, ranked: the size/pool gate (`< THRESHOLD` fires or
    /// a single-thread pool), an ARMED replay-pause seam, and
    /// this constraint — and ONLY the third is counted, because a level the
    /// first two already made serial is one the constraint did not change.
    /// So the counter reads exactly "how often did the shared-topic constraint
    /// change the routing" (Principle #3: the routing is observable, not
    /// inferred from timing). Default 0; never reset.
    ///
    /// It is a TEST / host-side observable, not the production one: the
    /// `GraphRuntime` accessor that reaches it is `cfg`-gated, so a shipping
    /// robot cannot read it. The production evidence that a level's routing
    /// changed is the build-time `info!` `GraphRuntime` emits per (level,
    /// qualifying topic).
    forced_serial_rest_walks: u64,
    /// The optional recording trace-ring hook (see [`TraceRingHook`]).
    /// `None` (the default, and the only state on non-unix targets) = today's
    /// behavior: the ring arm in [`RingTraceSink::push_entry`] is one
    /// never-taken branch — zero hot-path change. Installed once by
    /// `set_trace_ring_producer` before step 0 of a recording run.
    trace_ring: Option<TraceRingHook>,
    /// The attached checkpoint arm, read ONCE per step by
    /// [`Self::begin_step`] to derive [`Self::catchup_cap_override`]. `None`
    /// (the default) = no recorder is attached to this run, and the whole clamp
    /// is one never-taken branch per step — the same constraint-5 argument
    /// `crate::state_arm` makes for its own boundary check (prose, NOT an
    /// intra-doc link: this module is portable and that one is `#[cfg(unix)]`,
    /// so a link breaks the non-unix docs gate — pinned by
    /// `cfg_audit_test`). Installed by `GraphRuntime::attach_state_arm`.
    catchup_arm: Option<Arc<dyn catchup_clamp::CatchupArm>>,
    /// THIS step's `max_catchup` override for `Period`
    /// nodes that declared none — refreshed at the top of [`Self::begin_step`]
    /// from ONE reading of [`Self::catchup_arm`], then READ (never re-derived)
    /// by every `decide_node` of the step.
    ///
    /// Once per step is a CORRECTNESS rule, not an optimisation: a recorder can
    /// arm or disarm the word at any instant, so re-reading it per node would
    /// let the flip land BETWEEN two nodes of one step and hand them different
    /// caps — a fire set that depends on where a wall instant fell inside a
    /// step, which replay cannot reproduce (Principle #7).
    catchup_cap_override: Option<u32>,
    /// The IN-MEMORY read-outcome sink, the
    /// replay engine's collection seam for the level-end merge (see
    /// [`MergedReadOutcome`]). `None` (the default, and the only state on a
    /// live/recording run) keeps [`Self::merge_read_outcomes`] on its
    /// early-out when no trace ring is installed either — zero behavior
    /// change. Installed by [`Self::set_read_outcome_memory_sink`]; drained by
    /// [`Self::take_merged_read_outcomes`] (per replayed step).
    read_outcome_sink: Option<Vec<MergedReadOutcome>>,
    /// The installed TRACE-DRIVEN fire plan, or `None` on every
    /// live and every non-replay path — where the whole feature is ONE
    /// never-taken branch at the head of each decide seam (the `catchup_arm`
    /// precedent). Installed per step by [`Self::set_replay_fire_plan`];
    /// `Some(_)` means this scheduler is REPLAY-DRIVEN and its triggers are no
    /// longer consulted for the fire decision.
    replay_plan: Option<ReplayFirePlan>,
    /// The engine's INTRA-STEP injection callback — the source of
    /// truth for [`ScheduledNode::replay_injection_hook`], kept here so a node
    /// added AFTER the install inherits it (the `record_tick_durations`
    /// pattern). `None` on every live path.
    replay_injection_hook: Option<ReplayInjectionHook>,
    /// The node insertion indices the CURRENT
    /// [`Self::set_replay_intra_step_pauses`] install wrote pauses to, ASCENDING.
    ///
    /// It is the retire list (the next install clears exactly these, so an
    /// install is O(pauses) rather than O(nodes)) and the read order for
    /// [`Self::unconsumed_replay_pauses`]. It is deliberately NOT the parallel
    /// path's armed predicate — that reads [`Self::replay_pauses_armed`], so the
    /// determinism guard does not lean on this set's hand-maintained invariant.
    /// Cleared + re-extended, never re-allocated — the `decision_slots` reuse
    /// rule.
    replay_paused_nodes: Vec<usize>,
    /// Has any [`Self::set_replay_intra_step_pauses`] been
    /// installed (Principle #3 — distinguishes "no pauses this step" from "this
    /// scheduler is not pause-driven at all"). Cleared by
    /// [`Self::clear_replay_intra_step_pauses`].
    replay_pauses_armed: bool,
    /// This rank's wedge page, for the per-STEP companion word.
    ///
    /// Held on the scheduler as well as per node because the two halves answer
    /// different questions: the per-node seq pairs catch a tick that never
    /// returns, and this catches a rank whose loop stopped BETWEEN ticks (where
    /// nothing is inside a tick, so no pair can dwell). Bumped once in
    /// [`Self::begin_step`].
    wedge_page: Option<Arc<dyn WedgeMarker>>,
}

/// What the scheduler needs of a cross-process wedge page.
///
/// A trait rather than the concrete `MappedWedgePage` for the reason
/// [`catchup_clamp::CatchupArm`] is one: the mapping type is `#[cfg(unix)]`, and
/// naming it in the scheduler's private per-node record would cfg-gate the fire
/// body itself. It also keeps the scheduler free of any POSIX-SHM dependency, so
/// the whole marker mechanism is drivable from a test double with no segment at all.
///
/// Every method is called from the fire path, so every implementation must be
/// wait-free and allocation-free — the production one is two `fetch_add`s.
pub trait WedgeMarker: Send + Sync {
    /// Mark `slot` as having ENTERED a tick.
    fn enter(&self, slot: usize);
    /// Mark `slot` as having RETURNED from its tick.
    fn exit(&self, slot: usize);
    /// Bump this rank's step counter — the companion for a rank wedged OUTSIDE
    /// any tick.
    fn advance_step(&self);
}

/// What a fresh pop turned out to be — see `Scheduler::set_sync_head`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum HeadFill {
    /// An ordinary frame; the head now holds it.
    Filled,
    /// The stamp jumped BACKWARD past the alignment window — within one run only
    /// a publisher clock RESTART can do that. The node must re-base onto the new
    /// epoch, because no exit the matcher has can evict the stale MAXIMUM the
    /// restart leaves on the other inputs.
    EpochReset,
    /// A straggler from the epoch that ended, popped from a queue that still
    /// held it. Nothing was filled; the caller discards it.
    StaleEpoch,
}

/// Which gating-epoch entry point a refusal
/// belongs to, so the refusal text names it. The immediate placement carries
/// the epoch it was asked to place; the deferred (live-anchor) arm carries
/// none — its epoch is read from `real_ns()` only when `run_live` takes its
/// anchor — and rendering it as `place_gating_epoch(0)` told an operator about
/// a placement of an epoch nobody passed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EpochPlacement {
    /// `GraphRuntime::place_gating_epoch(epoch_ns)`.
    Now(u64),
    /// `GraphRuntime::place_gating_epoch_at_live_anchor()`.
    AtLiveAnchor,
}

impl std::fmt::Display for EpochPlacement {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Now(epoch_ns) => write!(f, "place_gating_epoch({epoch_ns})"),
            Self::AtLiveAnchor => f.write_str("place_gating_epoch_at_live_anchor()"),
        }
    }
}

/// How [`Scheduler::tick_decided_parallel`] may walk a level's REST
/// (the non-serial-gated fires of PASS 2). The build computes one per DAG
/// level (its `LevelPlan::rest_walk`); the scheduler only honours it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RestWalk {
    /// The size/pool gate alone decides: on a multi-thread pool a REST of
    /// `>= PARALLEL_FIRE_THRESHOLD` fans across rayon, and anything smaller
    /// walks serially. The default for every level without the constraint
    /// below.
    Unconstrained,
    /// The REST MUST walk serially in insertion order on the calling thread
    /// whatever its size. CONSTRAINT, not a knob: the level holds two or more
    /// non-serial-gated producers of ONE `multi_publisher_topics` topic, whose
    /// ticks publish into ONE shared iceoryx2 FIFO — under rayon their
    /// interleave is a scheduling artifact the read log and the recorded bag
    /// inherit while the fire trace cannot see it (PASS 3 merges in decision
    /// order), so replay could re-fire them into a different order on healthy
    /// code (Principle #7). Insertion order is the one order every driver and
    /// every run agree on — and it IS graph (declaration) order: the
    /// scheduler's `IndexMap` is filled by `add_node` in `config.nodes` order,
    /// so "insertion" here never means anything a YAML author cannot read off
    /// the file.
    InsertionOrder,
}

/// The one verdict on which driver walks a level's
/// REST in PASS 2 of [`Scheduler::tick_decided_parallel`], and WHY — produced
/// by the pure [`Scheduler::rest_driver`] and matched EXHAUSTIVELY at the
/// routing site.
///
/// An enum rather than three booleans because the three narrowing causes are
/// RANKED, and the rank is load-bearing for the observable: the constraint
/// counter credits a level only when the constraint ALONE narrowed it, which
/// used to be a `serial_rest && !narrow` expression whose precedence lived in a
/// comment. Here the precedence is the fn's `if` order and the "alone" is a
/// variant. A fourth narrowing input must add a variant AND a rank, and the
/// PASS 2 `match` refuses to compile until it is classified.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RestDriver {
    /// Fan the REST across the rayon pool (`par_values_mut().for_each`).
    Parallel,
    /// Serial by the size/pool gate: fewer than `PARALLEL_FIRE_THRESHOLD`
    /// REST fires, or a single-thread pool. A pure perf/alloc knob.
    SerialBySize,
    /// Serial because intra-step replay pauses are ARMED: the
    /// hook-call order across nodes is the serve order of injected frames.
    SerialByReplayPauses,
    /// Serial because the level carries [`RestWalk::InsertionOrder`] and
    /// nothing above it narrowed the REST already — the ONLY variant that
    /// bumps `forced_serial_rest_walks`.
    SerialByConstraint,
}

impl Scheduler {
    /// Create a scheduler with `RealClock`.
    // hot-path-alloc-ok-fn: cold: Scheduler CONSTRUCTION
    pub fn new() -> Self {
        Self {
            clock: ClockInner::Real(Arc::new(RealClock)),
            nodes: IndexMap::new(),
            trace: VecDeque::new(),
            max_trace_entries: None,
            fire_decisions: Vec::new(),
            decision_slots: Vec::new(),
            record_tick_durations: false,
            steps_begun: 0,
            entries_appended: 0,
            forced_serial_rest_walks: 0,
            trace_ring: None,
            catchup_arm: None,
            catchup_cap_override: None,
            read_outcome_sink: None,
            replay_plan: None,
            replay_injection_hook: None,
            replay_paused_nodes: Vec::new(),
            replay_pauses_armed: false,
            wedge_page: None,
        }
    }

    /// Create a scheduler with a `VirtualClock` for deterministic execution.
    // hot-path-alloc-ok-fn: cold: Scheduler CONSTRUCTION
    pub fn with_virtual_clock(clock: Arc<VirtualClock>) -> Self {
        Self {
            clock: ClockInner::Virtual(clock),
            nodes: IndexMap::new(),
            trace: VecDeque::new(),
            max_trace_entries: None,
            fire_decisions: Vec::new(),
            decision_slots: Vec::new(),
            record_tick_durations: false,
            steps_begun: 0,
            entries_appended: 0,
            forced_serial_rest_walks: 0,
            trace_ring: None,
            catchup_arm: None,
            catchup_cap_override: None,
            read_outcome_sink: None,
            replay_plan: None,
            replay_injection_hook: None,
            replay_paused_nodes: Vec::new(),
            replay_pauses_armed: false,
            wedge_page: None,
        }
    }

    /// Create a scheduler whose gating clock is advanced
    /// by the LIVE loop's handed logical quantum (deterministic), NOT by a polled
    /// `step()` delta and NOT by wall elapsed. Used by the deterministic-live
    /// build path. Selecting the `Barrier` arm BY INTENT here is what
    /// keeps a `VirtualClock` from ever being upcast to `dyn Clock` (the `Real`
    /// arm, whose advance is a no-op). The advance SOURCE itself is NOT read off
    /// this arm — `ClockInner` is private with no accessor — it is chosen by the
    /// runtime's `GraphRuntime.live_gating_quantum` + the `step_live(gating, wall)`
    /// split (see the `ClockInner::Barrier` doc). This arm is the documentary
    /// intent marker (and an (e) forward hook) that GUARANTEES the advanceability
    /// the quantum path needs. Mechanically the clock advances exactly like the
    /// `Virtual` arm (`advance_by_recorded`); only the intent differs.
    // hot-path-alloc-ok-fn: cold: Scheduler CONSTRUCTION
    pub fn with_barrier_clock(clock: Arc<VirtualClock>) -> Self {
        Self {
            clock: ClockInner::Barrier(clock),
            nodes: IndexMap::new(),
            trace: VecDeque::new(),
            max_trace_entries: None,
            fire_decisions: Vec::new(),
            decision_slots: Vec::new(),
            record_tick_durations: false,
            steps_begun: 0,
            entries_appended: 0,
            forced_serial_rest_walks: 0,
            trace_ring: None,
            catchup_arm: None,
            catchup_cap_override: None,
            read_outcome_sink: None,
            replay_plan: None,
            replay_injection_hook: None,
            replay_paused_nodes: Vec::new(),
            replay_pauses_armed: false,
            wedge_page: None,
        }
    }

    /// Create a scheduler with any `Clock` implementation.
    ///
    /// Note: `step()` can only advance time for `VirtualClock`. Use
    /// `with_virtual_clock()` for deterministic stepping. An `ExternalClock`
    /// (external time master, fed via `ExternalClock::set_external`) is driven
    /// through this path — the scheduler reads its time but never advances it.
    // hot-path-alloc-ok-fn: cold: Scheduler CONSTRUCTION
    pub fn with_clock(clock: Arc<dyn Clock>) -> Self {
        Self {
            clock: ClockInner::Real(clock),
            nodes: IndexMap::new(),
            trace: VecDeque::new(),
            max_trace_entries: None,
            fire_decisions: Vec::new(),
            decision_slots: Vec::new(),
            record_tick_durations: false,
            steps_begun: 0,
            entries_appended: 0,
            forced_serial_rest_walks: 0,
            trace_ring: None,
            catchup_arm: None,
            catchup_cap_override: None,
            read_outcome_sink: None,
            replay_plan: None,
            replay_injection_hook: None,
            replay_paused_nodes: Vec::new(),
            replay_pauses_armed: false,
            wedge_page: None,
        }
    }

    /// Nanoseconds until the soonest pending `Period` fire,
    /// or `None` if no enabled Period node has a pending deadline and no
    /// enabled Data node carries a FIFO backlog (which reports due-NOW). PURE
    /// READ — reads only `policy` + `next_fire_ns` + `pending_data_count` +
    /// `data_backlog_hint` + the disabled flag; performs no
    /// `advance`/`fire`/trace/mutation. Called ONLY from the live loop
    /// (`GraphRuntime::run_live`) to size the WaitSet timeout so a `period_ms`
    /// node fires on time between data events. NEVER called from
    /// `step`/`evaluate_node`/`fire_node`, so it is invisible to the replay
    /// firewall (replay_test drives the scheduler directly and never calls it).
    ///
    /// A Data node's backlog is normally NOT visible here at all,
    /// because a step now drains it — the fire loop serves up to
    /// `DATA_PENDING_CARRY_CLAMP` queued frames in one step. Only a burst
    /// DEEPER than that cap leaves anything behind, and it does so in one of two
    /// shapes: signalled-but-unfired arrivals (`pending_data_count`, the shape a
    /// `Separate` binding produces, and the shape a mid-burst `block`/throttle
    /// defer produces on either path), or a frame the Unified refill popped and
    /// froze at the cap without firing (`data_backlog_hint`, which
    /// `pending_data_count` deliberately cannot describe — see that field).
    /// Either way the node is due NOW, and `Some(0)` clamps to the loop's 1 ms
    /// floor: the remainder drains at the poll cadence, never a busy-spin.
    pub fn ns_until_next_fire(&self, now_ns: u64) -> Option<u64> {
        self.nodes
            .values()
            .filter(|n| !n.disabled)
            .filter_map(|n| {
                let due = match &n.policy {
                    TriggerPolicy::Period { .. } => {
                        n.next_fire_ns.map(|nf| nf.saturating_sub(now_ns))
                    }
                    TriggerPolicy::Data if n.pending_data_count > 0 || n.data_backlog_hint => {
                        Some(0)
                    }
                    // A Sync node holding a COMPLETE aligned set it has
                    // not fired is due NOW, exactly as a Data node holding a
                    // popped-but-unfired frame is. Without this arm a carried set
                    // waits for an unrelated publish to wake the loop, or for the
                    // 250 ms liveliness cap, where Data recovers at the 1 ms floor.
                    TriggerPolicy::Sync { .. } if n.sync_backlog_hint => Some(0),
                    _ => None,
                }?;
                // A node that is DUE but THROTTLE-DEFERRED
                // cannot run until its rate cap expires, so its real deadline is
                // the LATER of the two. Without this the arms above report
                // `Some(0)` for the whole throttle window — the node is due by
                // every signal the scheduler can see — and the loop polls at its
                // 1 ms floor, deciding to defer again each time.
                //
                // `max`, not "replace": the throttle bounds when the node MAY
                // fire, it never brings a later deadline forward. A Period node
                // cannot reach this arm anyway (`throttle_ms` and `period_ms`
                // are mutually exclusive at the macro), so in practice this
                // lifts a `Some(0)` to the remaining window.
                //
                // BLOCK-deferred nodes are deliberately NOT covered: block has
                // its own wake channel (the credit word), and a deadline the
                // loop polls toward would race the wake it now has.
                Some(match n.throttle_remaining_ns(now_ns) {
                    Some(remaining) => due.max(remaining),
                    None => due,
                })
            })
            .min()
    }

    /// The gating clock's current time (ns) — the domain every `next_fire_ns`
    /// deadline read by [`Self::ns_until_next_fire`] lives in. PURE READ: the
    /// live loop anchors its wall-paced wait to this so a controlled clock
    /// (which starts at 0, not at boot) is never compared against
    /// `CLOCK_MONOTONIC` directly.
    pub fn gating_now_ns(&self) -> u64 {
        self.clock.now_ns()
    }

    /// The tightest timing requirement the graph *declares*
    /// (in ns), or `None` if it declares none (purely data-driven /
    /// latency-tolerant).
    ///
    /// Used at startup to derive the live-path CPU C-state cap — a graph with
    /// tight timing wants a hot core (shallow C-states), while a relaxed graph
    /// can idle deep for power. The MIN is taken across, for every node:
    ///
    /// - the `Period { interval }` cadence (the *configured* interval, NOT the
    ///   per-fire `next_fire_ns` deadline that [`Self::ns_until_next_fire`]
    ///   reads),
    /// - the bounded `Sync { window: Some(d) }` pairing window (a declared
    ///   latency requirement, symmetric to `expect_within`; UNBOUNDED sync —
    ///   `window: None` — is latency-tolerant and contributes nothing),
    /// - the node-level `tick_within_ms` budget,
    /// - each input's `expect_within_ms` watchdog window,
    /// - each output's `promise_within_ms` watchdog window.
    ///
    /// Nodes/fields without a value are skipped.
    ///
    /// Unlike [`Self::ns_until_next_fire`], this deliberately does NOT filter
    /// on `!disabled`: this is a STARTUP-time cap derived from the graph's
    /// *configured* tightness, so a node disabled mid-run (e.g. by the
    /// repeated-panic circuit breaker) must not silently loosen the cap.
    ///
    /// PURE READ — touches no firing/trace/`next_fire_ns` state, so it is
    /// invisible to the Replay=Live firewall (Principle #7).
    pub fn tightest_timing_ns(&self) -> Option<u64> {
        self.nodes
            .values()
            .flat_map(|n| {
                let policy_timing = match &n.policy {
                    TriggerPolicy::Period { interval, .. } => Some(interval.as_nanos() as u64),
                    // A bounded sync window is a declared latency requirement;
                    // unbounded sync (`window: None`) is latency-tolerant.
                    TriggerPolicy::Sync {
                        window: Some(d), ..
                    } => Some(d.as_nanos() as u64),
                    _ => None,
                };
                let expect = n.input_expect_within.values().map(|w| w.within_ns);
                let promise = n.output_promise_within.values().map(|w| w.within_ns);
                policy_timing
                    .into_iter()
                    .chain(n.tick_within_ns)
                    .chain(expect)
                    .chain(promise)
            })
            .min()
    }

    /// Set maximum trace entries (ring buffer). Default: unbounded.
    ///
    /// When set, the trace operates as a ring buffer that drops the oldest
    /// entries when full. When `None` (default), trace grows unbounded.
    /// Use this for long-running sessions to prevent OOM.
    pub fn with_trace_limit(mut self, limit: usize) -> Self {
        self.max_trace_entries = Some(limit);
        self
    }

    /// Set the trace ring-buffer cap AFTER construction — the post-build twin
    /// of [`Self::with_trace_limit`] (sets `max_trace_entries = Some(limit)`;
    /// behaviorally identical, pinned by `set_trace_limit_matches_builder`).
    /// The capped ring keeps the NEWEST entries. Production CLI paths apply a
    /// default cap so long-running graphs don't grow their trace unbounded;
    /// test builds never cap (replay / trace-compare tests need full traces).
    pub fn set_trace_limit(&mut self, limit: usize) {
        self.max_trace_entries = Some(limit);
    }

    /// Add a node to the scheduler. Returns a handle for observing state.
    ///
    /// Nodes fire in the order they are added (IndexMap insertion order).
    // hot-path-alloc-ok-fn: cold: registers ONE node at graph BUILD
    pub fn add_node(&mut self, config: NodeConfig) -> TransportResult<NodeHandle> {
        if self.nodes.contains_key(&config.id) {
            return Err(TransportError::DuplicateNode { node_id: config.id });
        }

        // Validate trigger policy. Clippy 1.95's
        // `collapsible_match` flags the nested `if cond { return Err }`
        // pattern; collapsing the guard into the match arm itself
        // satisfies it.
        match &config.policy {
            TriggerPolicy::Period { interval, .. } if interval.is_zero() => {
                return Err(TransportError::SchedulerError {
                    reason: "Period interval must be > 0".to_string(),
                });
            }
            TriggerPolicy::Sync { inputs, .. } if inputs.is_empty() => {
                // Name the CAUSE and the FIX. The graph
                // runtime builds `Sync.inputs` from the node's
                // `#[input(trigger)]`-marked WIRED inputs only, so an empty
                // set means zero inputs carry the trigger mark — the node
                // could NEVER fire (the silent-starvation class,
                // rejected loudly instead).
                return Err(TransportError::SchedulerError {
                    reason: format!(
                        "node '{}': Sync trigger set is empty — sync aligns ONLY \
                         `#[input(trigger)]`-marked wired inputs, and none \
                         resolved, so the node could never fire. Mark >=2 wired inputs \
                         `#[input(trigger)]` (a plain `#[input]` is a latest-value read \
                         that never gates the fire), or use a different policy \
                         (`period_ms` / `external` / a single `#[input(trigger)]` for \
                         DataTrigger).",
                        config.id
                    ),
                });
            }
            _ => {}
        }

        let fire_count = Arc::new(AtomicU64::new(0));
        let last_fire_ns = Arc::new(AtomicU64::new(0));
        let panic_count = Arc::new(AtomicU64::new(0));
        let pending_data_count_shared = Arc::new(AtomicU64::new(0));
        // Three independent QoS miss counters.
        let expect_within_missed = Arc::new(AtomicU64::new(0));
        let promise_within_missed = Arc::new(AtomicU64::new(0));
        let tick_within_missed = Arc::new(AtomicU64::new(0));
        // The disjoint BACKLOG bucket beside `expect_within_missed`.
        let expect_within_backlogged = Arc::new(AtomicU64::new(0));
        // The IN-TICK marker. `0` is the not-in-a-tick sentinel, so a
        // fresh node reads "not in a tick" before it has ever fired.
        let in_tick_since_ns = Arc::new(AtomicU64::new(0));
        // Per-input backpressure counters, lazily registered.
        let backpressure: Arc<RwLock<IndexMap<Arc<str>, Arc<BackpressureCounters>>>> =
            Arc::new(RwLock::new(IndexMap::new()));
        // Per-output discard-count mirrors, registered per
        // publisher-owning output at graph build (observability-only).
        let output_discards: Arc<RwLock<IndexMap<Arc<str>, Arc<AtomicU64>>>> =
            Arc::new(RwLock::new(IndexMap::new()));
        // Per-output undelivered-notify mirrors, registered per
        // publisher-owning output at graph build (observability-only).
        let notify_undelivered: Arc<RwLock<IndexMap<Arc<str>, Arc<AtomicU64>>>> =
            Arc::new(RwLock::new(IndexMap::new()));
        // Per-node publisher-disconnect counter, bumped by
        // the runtime liveliness sweep (not by `step()`).
        let publisher_disconnects_observed = Arc::new(AtomicU64::new(0));
        // Per-node signal_*() desync-rejection counter,
        // bumped in the wrong-policy Err arms of signal_data/signal_sync_input.
        let signal_failed = Arc::new(AtomicU64::new(0));
        // Fresh per-node discard signal + replay-suppress flag. A bare
        // scheduler (no GraphRuntime) never marks a discard — these defaults are
        // shared with NO publisher, so `fire_node_into`'s delta is always 0.
        // `GraphRuntime` OVERWRITES both with Arcs it also injects into every one
        // of this node's publishers (`set_node_discard_signals`).
        let discard_signal = Arc::new(std::sync::atomic::AtomicU32::new(0));
        let replay_suppress = Arc::new(std::sync::atomic::AtomicBool::new(false));

        // Register every DECLARED trigger input's counters up front,
        // so a later read of 0 means "nothing happened" rather than "not
        // registered yet" — the contract the per-input backpressure and
        // output-discard maps already keep. A non-Sync node carries the bundle
        // at its defaults, so an accessor never has to be guarded by a policy
        // check at the call site.
        let sync_counters = Arc::new(crate::scheduler::handle::SyncCounters::default());
        let mut sync_unmatched_latches: IndexMap<Arc<str>, FailureRegimeLatch> = IndexMap::new();
        if let TriggerPolicy::Sync { inputs, .. } = &config.policy {
            for input in inputs {
                sync_counters.register_input(input);
                sync_unmatched_latches.insert(Arc::from(input.as_str()), FailureRegimeLatch::new());
            }
        }

        let handle = NodeHandle::new(
            config.id.clone(),
            Arc::clone(&fire_count),
            Arc::clone(&last_fire_ns),
            Arc::clone(&panic_count),
            Arc::clone(&pending_data_count_shared),
            Arc::clone(&expect_within_missed),
            Arc::clone(&promise_within_missed),
            Arc::clone(&tick_within_missed),
            Arc::clone(&expect_within_backlogged),
            Arc::clone(&in_tick_since_ns),
            Arc::clone(&backpressure),
            Arc::clone(&publisher_disconnects_observed),
            Arc::clone(&signal_failed),
            Arc::clone(&output_discards),
            Arc::clone(&notify_undelivered),
            Arc::clone(&sync_counters),
        );

        // Initialize next_fire_ns for Period nodes.
        //
        // Baseline the first fire RELATIVE to the clock's
        // current time, NOT absolute-from-0. Under `VirtualClock` the clock
        // reads 0 at build (no `step`/`advance` has run yet when nodes are
        // wired), so this is `0 + interval = interval` — byte-identical to the
        // prior absolute init (the replay firewall confirms it). Under the
        // RealClock live default (`build_live`) the clock reads wall time, so
        // without this a fresh Period node's `next_fire_ns` (≈ interval, e.g.
        // 10 ms) would be eons behind wall-now and `evaluate_node`'s catch-up
        // loop would try to fire up to `max_catchup` (default `u32::MAX`) times
        // on the very first `step` — an effective hang. Mirrors the existing
        // clock-relative advance in `reset_node`.
        let next_fire_ns = match &config.policy {
            TriggerPolicy::Period { interval, .. } => Some(
                self.clock
                    .now_ns()
                    .saturating_add(interval.as_nanos() as u64),
            ),
            _ => None,
        };

        let node_id_arc: Arc<str> = Arc::from(config.id.as_str());

        let node = ScheduledNode {
            policy: config.policy,
            callback: config.callback,
            node_id: Arc::clone(&node_id_arc),
            next_fire_ns,
            pending_data_count: 0,
            sync_heads: IndexMap::new(),
            // Installed later (graph build) for TRANSPORT-backed Sync
            // nodes only; a pure-scheduler embedder keeps the hand-driven
            // `signal_sync_input` semantics with no ops at all.
            sync_ops: None,
            sync_backlog_hint: false,
            sync_counters,
            sync_unmatched_latches,
            sync_op_failure_latch: FailureRegimeLatch::new(),
            sync_op_failed_this_pass: false,
            sync_stamp_high_water: IndexMap::new(),
            sync_epoch_latch: FailureRegimeLatch::new(),
            sync_epoch_floor: None,
            sync_scratch_heads: Vec::new(),
            sync_scratch_next: Vec::new(),
            sync_scratch_advances: Vec::new(),
            // Installed later (graph build) for Unified data-trigger
            // bindings only; the hint starts clear.
            trigger_refill: None,
            data_backlog_hint: false,
            // Installed at graph build for nodes that
            // declare `throttle_ms`; absent means the node has no rate cap.
            throttle_ns: None,
            // Only a trace-driven replay burst ever bumps this.
            replay_refill_shortfalls: 0,
            // No pauses until a replay engine installs some.
            replay_pauses: NodeReplayPauses::default(),
            // Inherit whatever hook is installed NOW, so a node
            // added after `set_replay_injection_hook` is not silently
            // pause-deaf (the `record_tick_durations` rule).
            replay_injection_hook: self.replay_injection_hook.clone(),
            fire_count,
            last_fire_ns,
            panic_count,
            pending_data_count_shared,
            expect_within_missed,
            promise_within_missed,
            tick_within_missed,
            expect_within_backlogged,
            in_tick_since_ns,
            // No page until a multi-process worker installs one.
            wedge: None,
            publisher_disconnects_observed,
            signal_failed,
            input_expect_within: IndexMap::new(),
            output_promise_within: IndexMap::new(),
            tick_within_ns: None,
            record_durations: self.record_tick_durations,
            qos_events: None,
            backpressure,
            output_discards,
            notify_undelivered,
            pre_fire_check: None,
            consecutive_panics: 0,
            disabled: false,
            external_triggered: false,
            // Starts empty; the within-level fire passes grow it once,
            // then reuse its capacity (mem::take / clear / write-back).
            trace_fragment: Vec::new(),
            // Defaults; the runtime overwrites discard_signal +
            // replay_suppress with the Arcs it shares into the publishers.
            discard_signal,
            replay_suppress,
            replay_discard_queue: std::collections::VecDeque::new(),
            // Empty until the runtime registers the node's wired
            // stages (`set_node_read_stages`); a bare scheduler never does.
            read_stages: Vec::new(),
            // A fresh UNARMED ledger, on the
            // `discard_signal` rule — unshared and unarmed means the disable
            // edge below records nothing at all until a runtime installs the
            // shared one AND a live loop arms it.
            node_death: Arc::new(crate::scheduler::node_death::NodeDeathLedger::new()),
        };

        self.nodes.insert(config.id, node);
        Ok(handle)
    }

    /// Remove a node from the scheduler.
    ///
    /// Also RE-DERIVES the intra-step pause seam's index list.
    /// `shift_remove` re-points every LATER insertion index, so a
    /// `replay_paused_nodes` entry minted before the removal can end up
    /// ALIASING a different, in-range node — which makes
    /// [`Self::unconsumed_replay_pauses`] silently under-report (it reads the
    /// wrong node's list) and lets the next install's retire loop clear an
    /// innocent node's pauses. Rebuilding the list from the map is O(nodes) on a
    /// build/teardown path and keeps the surviving nodes' installed pauses both
    /// reportable and retirable, which clearing it would not.
    // hot-path-alloc-ok-fn: cold: node removal — a build/teardown edit, never a step
    pub fn remove_node(&mut self, id: &str) -> TransportResult<()> {
        let Some((removed_idx, _, _)) = self.nodes.shift_remove_full(id) else {
            return Err(TransportError::NodeNotFound {
                node_id: id.to_string(),
            });
        };
        // The installed fire plan is keyed by node INDEX, and
        // `shift_remove` moved every later node down by one. A plan left as it
        // was would pair each of those nodes with its PREDECESSOR's slot — the
        // removed node's slot would fire the node after it, and the last
        // node's slot would name no node at all (`unconsumed_replay_fires`
        // filters it out, so the loss would be SILENT). The slot moves with the
        // node it describes, exactly as the pause index below is re-derived;
        // the removed node's own slot goes with it.
        if let Some(plan) = self.replay_plan.as_mut() {
            if removed_idx < plan.slots.len() {
                plan.slots.remove(removed_idx);
            }
        }
        // Cleared + re-extended (the `decision_slots` reuse rule), ASCENDING by
        // construction, and naming EXACTLY the nodes that still hold entries —
        // which is the invariant `set_replay_intra_step_pauses`'s
        // `debug_assert!` states. The removed node's own list went with it.
        self.replay_paused_nodes.clear();
        self.replay_paused_nodes.extend(
            self.nodes
                .values()
                .enumerate()
                .filter(|(_, node)| !node.replay_pauses.after_fires.is_empty())
                .map(|(idx, _)| idx),
        );
        Ok(())
    }

    /// Reset a disabled node's circuit breaker, allowing it to fire again.
    ///
    /// Clears the `disabled` flag, resets the consecutive panic counter, and
    /// for Period nodes, advances `next_fire_ns` past the current time so
    /// the node doesn't catch up on fires missed while disabled.
    // hot-path-alloc-ok-fn: cold: node reset — a build/teardown edit, never a step
    pub fn reset_node(&mut self, node_id: &str) -> TransportResult<()> {
        let current_time = match &self.clock {
            ClockInner::Virtual(c) => c.now_ns(),
            ClockInner::Barrier(c) => c.now_ns(),
            ClockInner::Real(c) => c.now_ns(),
        };

        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                node_id: node_id.to_string(),
            })?;
        node.disabled = false;
        node.consecutive_panics = 0;

        // For Period nodes, skip ahead to avoid catch-up burst.
        //
        // This goes through the ONE implementation of the advance
        // (`catchup_clamp::period_advance`), at a cap of ZERO — "carry the
        // deadline strictly past `now` and mint NO fires", which is exactly what
        // this wants. It was a `while` loop, the third spelling of that rule,
        // and it carried both of the failure modes the O(1) form exists to fix:
        // a `period_ms` of 0 spins forever (the deadline never moves), and a
        // deadline far below a wall-faithful clock is ~1e12 iterations of a plain
        // `+=` that also wraps rather than clamping. A `reset_node` on a
        // circuit-broken node is a diagnostic action on a live graph and must
        // not hang it.
        if let TriggerPolicy::Period { interval, .. } = &node.policy {
            let interval_ns = interval.as_nanos() as u64;
            if let Some(next_fire) = node.next_fire_ns.as_mut() {
                *next_fire =
                    catchup_clamp::period_advance(*next_fire, current_time, interval_ns, 0)
                        .next_fire_ns;
            }
        }

        tracing::info!(node_id = %node_id, "circuit breaker reset");
        Ok(())
    }

    /// Restore path: declare the step this scheduler RESUMES at.
    ///
    /// `steps_begun` counts from 0, so a graph resumed from an anchor at step
    /// `S` would number its first step 0 while the recording numbers it
    /// `S + 1` — and every trace record it emits would carry the wrong step.
    /// The requirement, directly: the gating clock and `steps_begun`
    /// are restored, or the resume is a new execution ORIGIN rather than a
    /// replay-grade one.
    ///
    /// `first_step` is the first step the graph will EXECUTE, and the value is
    /// read off the recording's own first `STEP_BOUNDARY` — the same number the
    /// restore's rendezvous is derived from, so the two cannot disagree.
    ///
    /// Must be called BEFORE the first `step()`; after one it would renumber a
    /// run already in progress.
    ///
    /// # A FROM-START replay can begin at a step > 0
    ///
    /// It is NOT "zero on every from-start path, where it is the
    /// value the counter already holds", though that holds for
    /// a mid-run resume. Under free-run, per-rank replay
    /// executes ONE rank at a time and each rank's recorded boundary stream is
    /// its OWN: a rank that first appears at global step 400 replays FROM ITS
    /// START at 400, so the engine sets that here and the rank's trace records
    /// carry the recording's step numbers rather than a second origin at 0.
    /// Zero remains the value a LOCKSTEP from-start replay (and every live path)
    /// leaves untouched.
    pub fn set_resume_step(&mut self, first_step: u64) {
        self.steps_begun = first_step;
    }

    /// Install the TRACE-DRIVEN fire plan for ONE logical step.
    ///
    /// While a plan is installed this scheduler is REPLAY-DRIVEN: every decide
    /// seam (`decide_fires` and `evaluate_nodes_fused`)
    /// synthesizes the recorded fires INSTEAD of evaluating triggers. That is
    /// the rule — "fires are TRACE-DRIVEN; re-derivation is the
    /// VERIFIER's job, never the fire driver" — and it is forced rather than
    /// chosen: a cross-process `block` producer's pre-fire gate reads cross-rank
    /// occupancy at decision instants the recording captures NOTHING about, so
    /// an independently re-derived schedule can legally differ from the recorded
    /// one and would yield spurious fire-schedule divergences on exactly the
    /// block-paced graphs this arc ships for.
    ///
    /// # What is still evaluated
    ///
    /// The node's `pre_fire_check` — the composed throttle + block gate — is
    /// STILL consulted before a planned fire. `throttle_ms` is rank-local and
    /// re-derivable from the recorded clock, so it reproduces the recording
    /// exactly; the BLOCK half is what must not gate here, and the runtime
    /// bypasses that disjunct under replay (see
    /// `GraphRuntime::set_block_gate_replay_bypass`). A `disabled` node (the
    /// panic circuit breaker) is likewise still honoured: forcing a panicking
    /// candidate to keep firing would bury the node failure the replay exists to
    /// report.
    ///
    /// # Errors
    ///
    /// * an unknown `node_id` — a fire the recording holds that this graph
    ///   cannot perform, refused LOUDLY rather than dropped (the
    ///   never-fire class); the engine filters the trace to this rank's nodes.
    /// * `fire_count == 0` — a node that did not fire carries no entry, so a
    ///   zero-count entry is a caller bug, not a way to say "no fire".
    /// * two entries for one node — per (node, step) the recorded fire times are
    ///   always ONE arithmetic progression (see `FireKind::Replay`), so a
    ///   duplicate means the caller split a burst it should have folded.
    ///
    /// On any of these the plan is installed ARMED and EMPTY: the step then
    /// fires nothing (never a fall-through to live deciding), and the `Err` is
    /// the caller's signal to abort the replay.
    ///
    /// # Allocation
    ///
    /// The slot table is cleared and `resize_with`'d to the node count, never
    /// re-allocated after the first call — so a plan-driven step allocates
    /// nothing at steady state, exactly as the un-planned one does.
    pub fn set_replay_fire_plan(
        &mut self,
        step: u64,
        fires: &[ReplayFire<'_>],
    ) -> TransportResult<()> {
        let node_count = self.nodes.len();
        let mut plan = self.replay_plan.take().unwrap_or_default();
        plan.step = step;
        plan.mismatch_reported = false;
        plan.slots.clear();
        plan.slots.resize_with(node_count, || None);

        let mut outcome = Ok(());
        for fire in fires {
            if fire.fire_count == 0 {
                outcome = Err(TransportError::GraphError {
                    reason: format!(
                        "replay fire plan (step {step}): node '{}' carries fire_count 0 — \
                         a node that did not fire carries no entry at all",
                        fire.node_id
                    ),
                });
                break;
            }
            let Some(idx) = self.nodes.get_index_of(fire.node_id) else {
                outcome = Err(TransportError::NodeNotFound {
                    // hot-path-alloc-ok: cold: plan-install ERROR arm — a malformed replay
                    // plan refuses at install, before any step runs.
                    node_id: fire.node_id.to_string(),
                });
                break;
            };
            if plan.slots[idx].is_some() {
                outcome = Err(TransportError::GraphError {
                    reason: format!(
                        "replay fire plan (step {step}): node '{}' named twice — fold a \
                         node's fires for one step into ONE entry (first + count + interval)",
                        fire.node_id
                    ),
                });
                break;
            }
            plan.slots[idx] = Some(PlannedFire {
                first_fire_ns: fire.first_fire_ns,
                fire_count: fire.fire_count,
                interval_ns: fire.interval_ns,
                consumed: false,
            });
        }
        if outcome.is_err() {
            // ARMED but EMPTY: a refused plan must not leave a PARTIAL fire set
            // armed (that would silently under-fire the step), and must not
            // disarm either (that would silently fall back to live deciding).
            plan.slots.clear();
            plan.slots.resize_with(node_count, || None);
        }
        self.replay_plan = Some(plan);
        outcome
    }

    /// Drop the trace-driven fire plan — the scheduler decides
    /// from its triggers again. A no-op when none is installed.
    pub fn clear_replay_fire_plan(&mut self) {
        self.replay_plan = None;
    }

    /// Is a trace-driven fire plan installed (Principle #3)?
    ///
    /// `true` means the decide seams are plan-driven — including for a step the
    /// plan does not describe, where the correct answer is "no fires", never
    /// "decide freely".
    pub fn is_replay_driven(&self) -> bool {
        self.replay_plan.is_some()
    }

    /// The nodes whose planned fires the installed plan still
    /// holds UNCONSUMED — fires the recording carries that no decide seam
    /// performed.
    ///
    /// Read after a step. A non-empty answer is a real divergence with a real
    /// cause: a `disabled` node (the panic breaker opened), a `throttle_ms` gate
    /// that deferred where the recording did not, or a node in no level list of
    /// this rank's runtime. Empty on every clean plan-driven step; empty when no
    /// plan is installed.
    // hot-path-alloc-ok-fn: cold: replay-only post-step OBSERVER — read once per
    // replayed step by the verifier epilogue, never on the live path.
    pub fn unconsumed_replay_fires(&self) -> Vec<&str> {
        let Some(plan) = self.replay_plan.as_ref() else {
            return Vec::new();
        };
        plan.slots
            .iter()
            .enumerate()
            .filter(|(_, slot)| matches!(slot, Some(planned) if !planned.consumed))
            .filter_map(|(idx, _)| self.nodes.get_index(idx).map(|(id, _)| id.as_str()))
            .collect()
    }

    /// How many decide seams ran against a plan installed for a
    /// DIFFERENT step (Principle #3 — the log is latched once, this is not).
    ///
    /// Non-zero means the engine's install/step pairing slipped; those seams
    /// fired nothing.
    pub fn replay_plan_mismatches(&self) -> u64 {
        self.replay_plan.as_ref().map_or(0, |plan| plan.mismatches)
    }

    /// How many times this node's trace-driven burst asked its
    /// refill hook for the next FIFO frame and got nothing — see
    /// `ScheduledNode::replay_refill_shortfalls`. `None` for an unknown id.
    pub fn replay_refill_shortfalls(&self, node_id: &str) -> Option<u64> {
        self.nodes.get(node_id).map(|n| n.replay_refill_shortfalls)
    }

    /// Install the engine's INTRA-STEP injection callback — the
    /// one thing a paused burst can hand control to. See
    /// [`ReplayInjectionHook`] for what it may do.
    ///
    /// Installed ONCE per replay (the `set_pre_fire_check` shape), not per step:
    /// the per-step vocabulary is [`Self::set_replay_intra_step_pauses`]. Cloned
    /// into every node now and inherited by any node added later, so a hook and
    /// a graph cannot be assembled in the wrong order.
    ///
    /// A second call REPLACES the first everywhere (no stacking — two hooks
    /// would inject one slot's frames twice).
    // hot-path-alloc-ok-fn: cold: installs the replay injection hook ONCE, before any step
    pub fn set_replay_injection_hook(&mut self, hook: ReplayInjectionHook) {
        for node in self.nodes.values_mut() {
            node.replay_injection_hook = Some(Arc::clone(&hook));
        }
        self.replay_injection_hook = Some(hook);
    }

    /// Drop the intra-step injection callback (the symmetric half
    /// of [`Self::set_replay_injection_hook`]).
    ///
    /// Installed pauses are deliberately LEFT ALONE: with no hook a reached
    /// pause delivers nothing and stays UNCONSUMED, so the loss shows up in
    /// [`Self::unconsumed_replay_pauses`] instead of being laundered into a
    /// silently-served slot. Use [`Self::clear_replay_intra_step_pauses`] to
    /// disarm the pauses themselves.
    // hot-path-alloc-ok-fn: cold: replay teardown — never on a live path
    pub fn clear_replay_injection_hook(&mut self) {
        for node in self.nodes.values_mut() {
            node.replay_injection_hook = None;
        }
        self.replay_injection_hook = None;
    }

    /// Is an intra-step injection callback installed (Principle #3
    /// — a seam nothing can read is a seam nothing can audit)?
    pub fn has_replay_injection_hook(&self) -> bool {
        self.replay_injection_hook.is_some()
    }

    /// Install the INTRA-STEP PAUSES for the step this scheduler is
    /// about to execute — "after node N's k-th fire, hand control to the
    /// injection hook".
    ///
    /// # Why a pause exists at all
    ///
    /// A before-step injection bucket puts every foreign frame in the consumer
    /// FIFO ahead of ALL of this step's local fires, so a recorded serve order
    /// of `[local, FOREIGN, local]` on one shared topic is unreproducible by
    /// construction. The pause is the sub-step slot that makes it reproducible;
    /// the frames still ride the engine's REAL injector publisher (never a
    /// pre-loaded bypass), which is what keeps the drain accounting and the
    /// `BackpressureEvent` dispatch real.
    ///
    /// # Pairing
    ///
    /// Call once per step, BEFORE `step()`, alongside
    /// [`Self::set_replay_fire_plan`] — the two are independent installs of one
    /// step's replay description, and the pause list carries its own step tag so
    /// a pairing slip cannot deliver silently. Passing an EMPTY slice is how a
    /// step with no pauses is declared; it leaves the seam armed and delivers
    /// nothing.
    ///
    /// Only a `FireKind::Replay` burst consults pauses, so this is inert unless
    /// a fire plan drives the step — and structurally unreachable on every live
    /// path (see `Scheduler::tick_replay_burst`).
    ///
    /// # Errors
    ///
    /// * a non-empty list with no [`Self::set_replay_injection_hook`] installed
    ///   — pauses with nothing to hand control to are a wiring bug, refused at
    ///   install rather than surfacing later as silently-undelivered frames.
    /// * `after_fire == 0` — the count is 1-based and means COMPLETED fires;
    ///   "before any fire" is the ordinary before-step bucket, not a pause.
    /// * an unknown `node_id` — the mirror of `set_replay_fire_plan`'s: a pause
    ///   this graph cannot perform is refused LOUDLY, never dropped.
    /// * the same `(node_id, after_fire)` twice — two injections at ONE slot are
    ///   ONE slot carrying more frames; a duplicate means the caller split a
    ///   slot it should have folded.
    ///
    /// On any of these the seam is left ARMED and EMPTY: the step then pauses
    /// nowhere (never a partial delivery), and the `Err` is the caller's signal
    /// to abort the replay.
    ///
    /// # What the ENGINE owes this seam
    ///
    /// After every step, before calling the run a clean pass, consult
    /// [`Self::replay_hook_panics`] and [`Self::unconsumed_replay_pauses`] — a
    /// non-empty answer from either means the engine's OWN injector lost frames
    /// the recording holds, so a downstream byte divergence is the harness's
    /// rather than the candidate's.
    ///
    /// # Allocation
    ///
    /// Each touched node's list is CLEARED (capacity retained), never dropped,
    /// and the retire list is cleared + re-extended — so a paused step allocates
    /// nothing at steady state, exactly as an unpaused one does.
    // hot-path-alloc-ok-fn: cold: replay-only per-step install, before the step runs
    pub fn set_replay_intra_step_pauses(
        &mut self,
        step: u64,
        pauses: &[IntraStepPause<'_>],
    ) -> TransportResult<()> {
        // The no-hook refusal is DECIDED here but not returned here: all four
        // error arms leave through the ONE shared exit below, so every one of
        // them satisfies the "ARMED and EMPTY" contract this fn documents. An
        // early `return` would skip both the retire loop and the arming flag,
        // leaving the PREVIOUS step's pauses installed (and, from a virgin
        // scheduler, the seam UNARMED) — reachable by install →
        // `clear_replay_injection_hook` (which deliberately leaves pauses
        // armed) → refused re-install.
        let mut outcome = if !pauses.is_empty() && self.replay_injection_hook.is_none() {
            Err(TransportError::GraphError {
                reason: format!(
                    "replay intra-step pauses (step {step}): {} pause(s) installed with NO \
                     injection hook — call set_replay_injection_hook() first, or the pauses \
                     would stop the burst with nothing to hand control to",
                    pauses.len()
                ),
            })
        } else {
            Ok(())
        };

        // Retire the PREVIOUS install by touching only the nodes it wrote to.
        // `mem::take` hands us the buffer so the loop below can borrow
        // `self.nodes` mutably; it goes back at the end with its capacity.
        let mut touched = std::mem::take(&mut self.replay_paused_nodes);
        for &idx in touched.iter() {
            if let Some((_, node)) = self.nodes.get_index_mut(idx) {
                node.replay_pauses.retire();
            }
        }
        touched.clear();
        self.replay_pauses_armed = true;

        for pause in pauses {
            if outcome.is_err() {
                // The no-hook refusal decided above: install NOTHING, exactly as
                // the three arms below do on their own first error.
                break;
            }
            if pause.after_fire == 0 {
                outcome = Err(TransportError::GraphError {
                    reason: format!(
                        "replay intra-step pauses (step {step}): node '{}' carries after_fire 0 \
                         — the count is 1-based and means COMPLETED fires; an injection before \
                         any fire is the ordinary before-step bucket",
                        pause.node_id
                    ),
                });
                break;
            }
            let Some(idx) = self.nodes.get_index_of(pause.node_id) else {
                outcome = Err(TransportError::NodeNotFound {
                    // hot-path-alloc-ok: cold: pause-install ERROR arm — a malformed pause
                    // list refuses at install, before any step runs.
                    node_id: pause.node_id.to_string(),
                });
                break;
            };
            // SAFETY: `idx` came from `get_index_of` on this same map, which is
            // not mutated inside this loop.
            let entry = &mut self
                .nodes
                .get_index_mut(idx)
                .expect("idx in range")
                .1
                .replay_pauses;
            if entry.after_fires.is_empty() {
                entry.step = step;
                entry.cursor = 0;
                // Belt-and-braces: only a node with a non-empty list can set
                // this (the consult returns before it otherwise), and every such
                // node is in the previous `touched` set, which `retire()` above
                // already cleared. It is kept so an edit to either loop cannot
                // carry a stale latch into a fresh install.
                entry.mismatch_reported = false;
                touched.push(idx);
            }
            entry.after_fires.push(pause.after_fire);
        }

        if outcome.is_ok() {
            // ASCENDING so the per-fire consult is one indexed compare, and so
            // `unconsumed_replay_pauses` reads in a stable insertion order.
            touched.sort_unstable();
            for &idx in touched.iter() {
                let (id, node) = self.nodes.get_index_mut(idx).expect("idx in range");
                node.replay_pauses.after_fires.sort_unstable();
                if let Some(dup) = node
                    .replay_pauses
                    .after_fires
                    .windows(2)
                    .find(|w| w[0] == w[1])
                    .map(|w| w[0])
                {
                    outcome = Err(TransportError::GraphError {
                        reason: format!(
                            "replay intra-step pauses (step {step}): node '{id}' names \
                             after_fire {dup} twice — fold two injections at ONE slot into \
                             ONE pause carrying both frames"
                        ),
                    });
                    break;
                }
            }
        }

        if outcome.is_err() {
            // ARMED but EMPTY, for the reason `set_replay_fire_plan` states: a
            // refused list must not leave a PARTIAL set armed (that would
            // deliver some of a step's injections), and must not disarm either
            // (that would silently fall back to before-step-only delivery).
            for &idx in touched.iter() {
                if let Some((_, node)) = self.nodes.get_index_mut(idx) {
                    node.replay_pauses.retire();
                }
            }
            touched.clear();
        }
        // `touched` is the ONLY index `unconsumed_replay_pauses` and the next
        // install's retire loop ever walk, so a node holding entries that is
        // NOT in it is a permanently-unreportable, never-retired pause list.
        // Nothing in this fn can produce that today (an entry is pushed the
        // moment a node's list goes from empty to non-empty); the assert is what
        // keeps a future edit from breaking it silently.
        debug_assert!(
            self.nodes.values().enumerate().all(|(idx, node)| node
                .replay_pauses
                .after_fires
                .is_empty()
                || touched.contains(&idx)),
            "every node holding installed pauses must be in the touched set — \
             otherwise its entries are neither reportable nor retirable"
        );
        self.replay_paused_nodes = touched;
        outcome
    }

    /// Disarm the intra-step pause seam entirely (the mirror of
    /// [`Self::clear_replay_fire_plan`]). Retires every installed pause; a no-op
    /// when none are armed.
    // hot-path-alloc-ok-fn: cold: replay teardown — never on a live path
    pub fn clear_replay_intra_step_pauses(&mut self) {
        let touched = std::mem::take(&mut self.replay_paused_nodes);
        for &idx in touched.iter() {
            if let Some((_, node)) = self.nodes.get_index_mut(idx) {
                node.replay_pauses.retire();
            }
        }
        self.replay_paused_nodes = touched;
        self.replay_paused_nodes.clear();
        self.replay_pauses_armed = false;
    }

    /// Are intra-step pauses armed (Principle #3)?
    ///
    /// `true` after any [`Self::set_replay_intra_step_pauses`], including one
    /// that installed an EMPTY list — "no pauses this step" and "this scheduler
    /// is not pause-driven" are different facts.
    pub fn is_replay_pause_armed(&self) -> bool {
        self.replay_pauses_armed
    }

    /// The installed pauses this step's bursts never CONSUMED, as
    /// `(node_id, after_fire)` in node insertion order then ascending count.
    ///
    /// Read after a step. A non-empty answer is a real divergence with a real
    /// cause, and the seam's whole reason for existing rather than dropping
    /// them (the never-fire class):
    ///
    /// * the burst was shorter than the recording's (see
    ///   [`Self::unconsumed_replay_fires`], which names why),
    /// * the node's planned fire was never performed at all,
    /// * the list was installed for a DIFFERENT step (see
    ///   [`Self::replay_pause_mismatches`]),
    /// * the injection hook was cleared out from under it,
    /// * or the injection hook PANICKED at the slot (see
    ///   [`Self::replay_hook_panics`], which says WHY while this says WHERE).
    ///
    /// Every one of those means foreign frames the recording holds were not
    /// injected where it held them. Only the first four mean the slot was never
    /// REACHED; the panic arm is reached and then refuses to record itself as
    /// served, which is why the report is keyed on CONSUMPTION.
    ///
    /// Empty on every clean paused step; empty when no pauses are armed.
    // hot-path-alloc-ok-fn: cold: replay-only post-step OBSERVER — read once per
    // replayed step by the verifier epilogue, never on the live path.
    pub fn unconsumed_replay_pauses(&self) -> Vec<(&str, u32)> {
        let mut out = Vec::new();
        for &idx in &self.replay_paused_nodes {
            let Some((id, node)) = self.nodes.get_index(idx) else {
                // Unreachable by construction: the only mutation of the node map
                // that can invalidate an index is `remove_node`, which
                // RE-DERIVES this list from the map for exactly that reason.
                // Kept because the alternative failure is silent — dropping the
                // entry would under-report the divergence this fn exists to
                // name — so a future map mutation that forgets the re-derive is
                // loud instead: the `tick_decided_parallel` orphan guard's rule,
                // applied to the pause report. (An index that survives a
                // removal IN RANGE aliases a different node and cannot be
                // detected here at all, which is why the fix is at the mutation
                // site rather than in this guard.)
                tracing::error!(
                    idx,
                    node_count = self.nodes.len(),
                    "unconsumed_replay_pauses: a paused node index no longer names a node \
                     — the node was removed between the pause install and this read, and \
                     its unconsumed pauses CANNOT be reported"
                );
                continue;
            };
            let pauses = &node.replay_pauses;
            let from = pauses.cursor.min(pauses.after_fires.len());
            out.extend(
                pauses.after_fires[from..]
                    .iter()
                    .map(|&after| (id.as_str(), after)),
            );
        }
        out
    }

    /// How many trace-driven fires consulted a pause list installed
    /// for a DIFFERENT step — the pause twin of
    /// [`Self::replay_plan_mismatches`], and unconditional for the same reason
    /// (the log is latched once per install; this is not).
    ///
    /// Non-zero means the engine's install/step pairing slipped; those fires
    /// delivered nothing, and the pauses they skipped are reported by
    /// [`Self::unconsumed_replay_pauses`]. Cumulative over the scheduler's life
    /// — never reset by a re-install.
    pub fn replay_pause_mismatches(&self) -> u64 {
        self.nodes
            .values()
            .map(|n| n.replay_pauses.mismatches)
            .sum()
    }

    /// How many intra-step pauses handed control to an injection
    /// hook that PANICKED.
    ///
    /// The hook is engine code running inside a node's burst, so an unguarded
    /// panic would unwind out of `step()` entirely — no counter, no report, no
    /// verdict. It is caught instead: the burst continues and the slot's cursor
    /// does NOT advance, so the pause is also reported by
    /// [`Self::unconsumed_replay_pauses`], which says WHERE the frames were
    /// lost while this says WHY. Cumulative over the scheduler's life — never
    /// reset by a re-install.
    ///
    /// The two reports do NOT count the same thing, and the difference is the
    /// consequence: a parked cursor also makes every LATER slot for that node in
    /// the same step unreachable, so ONE panic costs the node's whole remaining
    /// pause list. `unconsumed_replay_pauses` reports all of them; this counts
    /// the one invocation that failed.
    ///
    /// Non-zero means the replay engine's own injector failed; the run's
    /// verdict cannot be trusted as a candidate divergence — and the engine
    /// acts on it: the pass is REFUSED at exit 5 (see [`ReplayInjectionHook`],
    /// "What the ENGINE owes this seam").
    pub fn replay_hook_panics(&self) -> u64 {
        self.nodes
            .values()
            .map(|n| n.replay_pauses.hook_panics)
            .sum()
    }

    /// Synthesize this node's TRACE-DRIVEN fire for the current
    /// step, or `None` if the plan holds none for it.
    ///
    /// The ONE plan-consultation body both decide seams share, so the level path
    /// (`decide_fires`) and the block-fused path (`evaluate_nodes_fused`) cannot
    /// drift — which matters because the fused path serves exactly the
    /// `block`-involved nodes the block-gate demotion is about, and leaving them
    /// live-decided would make the whole feature inert on its own headline shape.
    ///
    /// Performs the bookkeeping a live decide would have performed for the
    /// trigger that fired, so the node's OBSERVABLE scheduler state does not
    /// drift while the plan drives it:
    ///
    /// * `Data` — clears the backlog hint and consumes up to `fire_count`
    ///   signalled arrivals (the injected reads keep signalling; leaving them
    ///   would saturate the count and mis-report `node_framework_state`).
    /// * `Sync` — clears the alignment timestamps, as a live fire does.
    /// * `External` — consumes the external mark.
    /// * `Period` — `next_fire_ns` is deliberately LEFT ALONE. The plan is
    ///   authoritative for the schedule; the initial phase is the engine's
    ///   (`restore_period_schedule`), and re-deriving an advance here would put a
    ///   second, disagreeing schedule beside the recorded one.
    ///
    /// `new_time` is the ADVANCED step clock — the same value the live decide
    /// hands `decide_node`, and the only value the pre-fire gate may be
    /// evaluated at. It is NOT the fire's timestamp: a trace-driven fire is
    /// stamped with `planned.first_fire_ns`, which is what the RECORDING says
    /// and what the burst reconstruction walks up from. See the gate call
    /// below for why the two must not be conflated.
    fn take_planned_fire(&mut self, idx: usize, id: &str, new_time: u64) -> Option<FireKind> {
        let step = self.current_step();
        let plan = self.replay_plan.as_mut()?;
        if plan.step != step {
            plan.mismatches = plan.mismatches.saturating_add(1);
            if !plan.mismatch_reported {
                plan.mismatch_reported = true;
                tracing::error!(
                    plan_step = plan.step,
                    step,
                    "replay fire plan is installed for a DIFFERENT step — this step fires \
                     NOTHING (a stale plan must never fall through to live deciding). \
                     Install one plan per step before stepping."
                );
            }
            return None;
        }
        let planned = (*plan.slots.get(idx)?)?;
        let node = self.nodes.get_index_mut(idx)?.1;
        if node.disabled {
            // The panic circuit breaker. A trace-driven fire never re-opens it:
            // the recording's fire is unperformable here, and that is the node
            // failure the replay verdict exists to report.
            return None;
        }
        // hot-path-alloc-ok: not a heap allocation: `pre_fire_check` is
        // `Option<Arc<dyn Fn>>`, so the clone is a refcount bump (the live twin in
        // `tick_data_burst` carries the same note) — and this fn runs only under an
        // installed REPLAY plan.
        if let Some(check) = node.pre_fire_check.clone() {
            // Evaluated at the ADVANCED STEP CLOCK, exactly
            // as live does (`decide_node`, and the Period catch-up loop's own
            // per-fire re-check, both pass `current_time_ns` — never the fire's
            // instant). The design keeps the throttle live under replay, so its
            // INPUTS have to match live's or it is a different rule wearing the
            // same name.
            //
            // It used to be handed `planned.first_fire_ns`. For every trigger
            // that stamps its fire with the step clock (Data / Sync / External)
            // those two values are equal, so the swap is behaviour-preserving
            // there; the shape where they differ is a Period CATCH-UP burst,
            // whose `first_fire_ns` is the earliest un-fired INTERVAL DEADLINE
            // and sits up to `fire_count * interval - 1` ns BELOW the step
            // clock. Fed to `throttle_defers` that reads as "the node fired more
            // recently than it did", so the gate could defer a burst live
            // allowed and the fire would be reported as an unconsumed
            // divergence the candidate never caused.
            //
            // SCOPE: `throttle_ms` + `period_ms` is a `#[cerulion_node]`
            // COMPILE error, and the throttle is the only disjunct that reads
            // this argument (the block half is bypassed under replay), so the
            // wrong clock is unreachable for any macro-authored node. It is
            // reachable through a raw-FFI cdylib's info JSON, which carries
            // `policy` and `throttle_ms` as independent fields with no
            // runtime-side rejection — and it also disagreed with the offline
            // re-deriver, which answers the same `throttle_defers` rule at
            // `step.clock_ns` (`replay_rederive::verify_throttle`). One rule,
            // two consumers, one clock.
            if Self::run_pre_fire_check(id, &check, new_time) {
                tracing::trace!(
                    node_id = %id,
                    "pre-fire check deferred a TRACE-DRIVEN fire (throttle is re-derived \
                     under replay; the block disjunct is bypassed by the runtime)"
                );
                return None;
            }
        }
        match &node.policy {
            TriggerPolicy::Data => {
                node.data_backlog_hint = false;
                let consumed = node.pending_data_count.min(u64::from(planned.fire_count));
                node.pending_data_count -= consumed;
                node.pending_data_count_shared
                    .store(node.pending_data_count, Ordering::Release);
            }
            // The per-set analogue of the earlier
            // `sync_input_timestamps.clear()` this replaced. A fire SPENDS its
            // set's members, and under per-set that is a TOMBSTONE rather than
            // a removal — an `Emitted` head is precisely what tells the next
            // alignment to re-drain that input, which a cleared map used to do
            // by being empty. Same consumption, and it goes through the one
            // helper every other fire path uses so the latch recovery it
            // carries cannot drift away from it.
            TriggerPolicy::Sync { .. } => Self::mark_sync_heads_emitted(node),
            TriggerPolicy::External => node.external_triggered = false,
            TriggerPolicy::Period { .. } => {}
        }
        // Re-borrow the plan to mark the entry consumed: the `node` borrow above
        // ends here, and the two fields are disjoint.
        if let Some(plan) = self.replay_plan.as_mut() {
            if let Some(slot) = plan.slots[idx].as_mut() {
                slot.consumed = true;
            }
        }
        Some(FireKind::Replay {
            first_fire_ns: planned.first_fire_ns,
            fire_count: planned.fire_count,
            interval_ns: planned.interval_ns,
        })
    }

    /// Restore path: set a `Period` node's NEXT fire instant.
    ///
    /// `add_node` baselines `next_fire_ns` at `clock.now_ns() + interval`,
    /// which is correct for a graph starting at its clock's origin and wrong
    /// for one RESUMED mid-run: the node's real baseline is whatever it was
    /// when the anchor was taken, and a build-time baseline makes the first
    /// step either burst through every interval since the origin or skip the
    /// fire the recording shows.
    ///
    /// That baseline belongs to the framework section (`next_fire_ns` is
    /// named there beside `pending_data_count`), which nothing captures yet.
    /// Until it does, the replay engine supplies the value from the ONE other
    /// place the recording states it — the scheduler trace's first recorded
    /// fire for the node — and this is the seam it sets it through.
    ///
    /// A NON-`Period` node is left alone (its trigger carries no timing
    /// state), and an unknown id is a no-op: the caller is reading a recording
    /// that may legitimately describe nodes this graph does not contain, and
    /// `restore_node_states` already reports that class.
    ///
    /// # A node the recording does not name is RE-PHASED, never left as built
    ///
    /// `baselines` carries only the nodes whose first recorded fire the caller
    /// could read, and a `Period` node whose next fire falls AFTER the recorded
    /// suffix has no fire in it to read — a 30 ms node resumed into a 10 ms
    /// window, the ordinary shape of any graph mixing rates. Leaving such a
    /// node alone is not neutral: `add_node` baselined it at
    /// `clock.now_ns() + interval` with the clock still at its ORIGIN, so the
    /// deadline sits far BELOW the anchor instant the caller then places the
    /// clock at, and the first resumed step fires it — an unrecorded fire on a
    /// graph nobody changed, and under `max_catchup` a whole burst of them.
    ///
    /// So every un-named `Period` node is advanced past `now` in whole
    /// intervals — through the SAME shared advance
    /// ([`catchup_clamp::period_advance`]) that [`Self::reset_node`] uses and
    /// that the live scheduler performs on every fire. That is what makes it
    /// replay-exact rather than a guess: the advance PRESERVES the node's
    /// phase, and the phase a fresh build establishes is the phase the recorded
    /// run had (both start from `add_node` on a clock at its origin, with the
    /// same declared interval, and every later mutation — the fire loop, the
    /// `max_catchup` skip-ahead, `reset_node` — only ever adds whole
    /// intervals). The result is therefore the earliest un-fired deadline of
    /// the recorded schedule at the anchor instant, which is what the live run
    /// held there.
    ///
    /// # The advance is O(1) ARITHMETIC, never a loop
    ///
    /// The FORWARD arm does not spell that arithmetic at
    /// all — it calls [`catchup_clamp::period_advance`] at a cap of ZERO, which
    /// is that function's "carry the deadline strictly past `now`, mint no
    /// fires" reading and the same code the live `Period` decide arm runs. The
    /// reasoning below is why the O(1) form is the right answer and is kept
    /// because it is the argument the extraction rests on. The BACKWARD arm is
    /// still spelled here: nothing else in the system pulls a deadline back.
    ///
    /// The advance used to be `while *next_fire <= now { *next_fire += i }`,
    /// which is O((now − next_fire) / interval) ITERATIONS — fine when `now` is
    /// an anchor a few seconds above a build-time deadline, and catastrophic
    /// when it is not. Free-run replay made it not: a FREE-RUN bag's per-rank
    /// clocks are wall-faithful, so a from-start free-run replay PLACES the clock
    /// on the first recorded boundary (~1.7e18 ns since the epoch) and then
    /// calls this — and a 1 ms node the recording never fired inside its window
    /// is exactly the un-named case, i.e. ~1.7e12 iterations, which is a HANG,
    /// not a slow path. It is computed in one step instead: the smallest whole
    /// number of intervals that carries the deadline strictly past `now`.
    ///
    /// The result is IDENTICAL to the loop's, by construction — `steps` is
    /// `floor((now − next_fire) / interval) + 1`, so `next_fire + steps *
    /// interval` is the first value above `now` that is congruent to
    /// `next_fire` modulo the interval, which is what "advance in whole
    /// intervals until past now" means. Phase is therefore preserved exactly as
    /// the paragraph above requires. Saturating throughout, so an interval or a
    /// gap near `u64::MAX` clamps rather than wrapping into the past.
    ///
    /// Ordering: the caller must place the clock at the anchor instant BEFORE
    /// calling this, or "past now" means past the origin and every un-named
    /// node keeps the build-time deadline this exists to replace.
    ///
    /// # The re-phase runs in BOTH directions
    ///
    /// The advance above answers "the deadline is BELOW the placed clock". Under
    /// free-run's SEQUENTIAL PER-RANK replay the opposite arrives too, and it is not
    /// a corner: one runtime is built per rank, one at a time, on the ONE clock
    /// the engine carries — so rank R+1's `add_node` baselines its deadlines at
    /// `rank R's last boundary + interval`, and the engine then places the clock
    /// at rank R+1's OWN first recorded boundary, which is EARLIER. A deadline
    /// sitting a whole recorded run above the clock is not "already correct": it
    /// is a node that will not fire for the length of the previous rank's
    /// window, which on a plan-driven replay would show up as a rank that
    /// produced nothing.
    ///
    /// So a deadline more than one interval ABOVE `now` is pulled back onto the
    /// same congruence class, by the same whole-interval arithmetic and with the
    /// same phase guarantee. Both arms compute the ONE value `V` with
    /// `next_fire ≡ V (mod interval)` and `now < V <= now + interval` — the
    /// earliest un-fired deadline of that phase at the placed instant — so the
    /// function is now total over an ARBITRARY prior clock rather than only over
    /// one below the deadline. The forward arm is byte-unchanged, and no
    /// earlier caller can reach the new one: the mid-run resume and the
    /// from-start free-run replay both build at the clock's ORIGIN, where an
    /// un-named node's deadline is exactly one interval up and the pull-back is
    /// a no-op by construction.
    pub fn restore_period_schedule(&mut self, baselines: &std::collections::BTreeMap<String, u64>) {
        let now = self.clock.now_ns();
        for (node_id, node) in self.nodes.iter_mut() {
            let TriggerPolicy::Period { interval, .. } = &node.policy else {
                continue;
            };
            if let Some(next_fire_ns) = baselines.get(node_id.as_str()) {
                node.next_fire_ns = Some(*next_fire_ns);
                continue;
            }
            let interval_ns = interval.as_nanos() as u64;
            if interval_ns == 0 {
                continue; // a zero interval cannot be advanced; leave it as built.
            }
            if let Some(next_fire) = node.next_fire_ns.as_mut() {
                if *next_fire <= now {
                    // The FORWARD arm is exactly
                    // `catchup_clamp::period_advance` at a cap of ZERO (carry the
                    // deadline strictly past `now`, mint no fires), so it is
                    // delegated rather than re-derived — the two were the same
                    // arithmetic written twice, which is the class the extraction
                    // exists to close. The BACKWARD arm below has no counterpart
                    // there (nothing else in the system pulls a deadline back)
                    // and stays here.
                    *next_fire =
                        catchup_clamp::period_advance(*next_fire, now, interval_ns, 0).next_fire_ns;
                } else {
                    // More than one interval ABOVE the placed
                    // clock ⇒ pull back onto the same congruence class. `ahead`
                    // is > 0 here, so `ahead - 1` never underflows, and
                    // `(ahead - 1) / interval` is 0 for `ahead <= interval` —
                    // which is why a deadline already inside one interval is
                    // left exactly as it was (a no-op).
                    let ahead = *next_fire - now; // `>` above ⇒ never underflows.
                    let steps_back = (ahead - 1) / interval_ns;
                    *next_fire = next_fire.saturating_sub(steps_back.saturating_mul(interval_ns));
                }
            }
        }
    }

    /// Place the CONTROLLED (`Barrier`) gating clock at
    /// `epoch_ns` BEFORE the first step, re-baselining everything
    /// [`Self::add_node`] / the QoS registrations anchored to the clock's
    /// build-time value — so the schedule is exactly what a build at the epoch
    /// would have produced. The scheduler-side half of
    /// [`crate::graph::GraphRuntime::place_gating_epoch`], which holds the
    /// rationale; this is the mechanism.
    ///
    /// Three things are re-based, each to the rule that produced it at build:
    ///
    /// * the clock itself — `set(epoch_ns)`;
    /// * every `Period` node's `next_fire_ns` → `epoch + interval`, the
    ///   `add_node` baseline (`now + interval`) re-read at the epoch. Deliberately
    ///   NOT [`Self::restore_period_schedule`]'s forward arm (which carries a
    ///   deadline past `now` on the ORIGIN's phase): a build at the epoch has no
    ///   phase to preserve, and mirroring the baseline rule keeps the free-run
    ///   rank's first fire where the non-record arm's would be — one interval
    ///   after its clock started;
    /// * every `expect_within` / `promise_within` tracker's `window_start_ns` →
    ///   `epoch`, the registration seed re-read at the epoch. The shared anchor
    ///   and `last_arrival_ns` are NOT touched: the scheduler never writes the
    ///   anchor (that is what makes an arrival unambiguous — see
    ///   [`Self::set_expect_within`]), and leaving `last_arrival_ns` equal to
    ///   it means the first REAL arrival still re-arms the window exactly as it
    ///   would on a fresh build. Without this re-seed the first
    ///   `run_qos_windows` reads an elapsed of `epoch − 0` and counts a phantom
    ///   miss on every guarded port.
    ///
    /// `throttle_ms`'s `last_fire_ns` (0) is left alone on purpose: `epoch − 0`
    /// exceeds every window, which is "never fired" — the same answer a fresh
    /// build gives. `sample(N)` keys on wire stamps, which are minted from this
    /// same clock AFTER placement. Liveliness and external-silence ride the
    /// dedicated wall-health clock and never see the gating clock at all.
    ///
    /// # Refusals
    ///
    /// * not a `Barrier` clock (`with_clock` / a `Real` clock): there is no
    ///   controlled clock to place — the runtime already refuses this arm by
    ///   its `live_gating_quantum`, and this is the scheduler's own check;
    /// * `steps_begun > 0`: an epoch is an origin; moving it under values
    ///   already stamped is the cross-domain skew the epoch exists to prevent.
    // hot-path-alloc-ok-fn: cold: loop-entry placement, before the first step
    pub(crate) fn place_gating_epoch(&mut self, epoch_ns: u64) -> TransportResult<()> {
        let clock = self.epoch_placeable_clock(EpochPlacement::Now(epoch_ns))?;
        clock.set(epoch_ns);
        for node in self.nodes.values_mut() {
            if let TriggerPolicy::Period { interval, .. } = &node.policy {
                node.next_fire_ns = Some(epoch_ns.saturating_add(interval.as_nanos() as u64));
            }
            for tracker in node
                .input_expect_within
                .values_mut()
                .chain(node.output_promise_within.values_mut())
            {
                tracker.window_start_ns = epoch_ns;
            }
        }
        Ok(())
    }

    /// The two scheduler-level refusals of
    /// [`Self::place_gating_epoch`], checkable WITHOUT placing — the deferred
    /// (live-anchor) placement runs them at arm time so a mis-built rank is
    /// refused before it waits for GO.
    pub(crate) fn epoch_placeable(&self) -> TransportResult<()> {
        self.epoch_placeable_clock(EpochPlacement::AtLiveAnchor)
            .map(|_| ())
    }

    /// How many logical steps have begun — `0`
    /// until the first `begin_step`. The runtime's polled seam reads it to say
    /// ONCE that an armed-for-`run_live` epoch is being stepped past.
    pub(crate) fn steps_begun(&self) -> u64 {
        self.steps_begun
    }

    // hot-path-alloc-ok-fn: cold: the refusal text of a build-time placement check
    fn epoch_placeable_clock(
        &self,
        placement: EpochPlacement,
    ) -> TransportResult<Arc<VirtualClock>> {
        if self.steps_begun > 0 {
            return Err(TransportError::GraphError {
                reason: format!(
                    "{placement}: the scheduler has already begun {} step(s) — an epoch is a \
                     clock ORIGIN and must be placed before the first step, never under a \
                     stream that has already stamped values",
                    self.steps_begun
                ),
            });
        }
        match &self.clock {
            ClockInner::Barrier(c) => Ok(Arc::clone(c)),
            ClockInner::Virtual(_) | ClockInner::Real(_) => Err(TransportError::GraphError {
                reason: format!(
                    "{placement}: this scheduler has no CONTROLLED (deterministic-live \
                     `Barrier`) gating clock to place — only a `build_live_deterministic*` \
                     runtime carries one"
                ),
            }),
        }
    }

    /// Read ONE node's framework section, the
    /// scheduler's own plain-data view of it, as of right now.
    ///
    /// The capture carrier calls this at a due step boundary, where "right
    /// now" is the state AFTER the step completed, which is what an anchor at
    /// `S` means. Nothing here reads a clock or mutates anything: the
    /// three fields are copied out and the caller owns what happens next.
    ///
    /// `None` for an id this scheduler does not hold — a carrier walking a
    /// recording's node list against a live graph is the ordinary reason, and
    /// `restore_node_states` already reports that class.
    ///
    /// # The two counters are the SCHEDULER's, not the transport's
    ///
    /// `pending_data_count` is arrivals SIGNALLED into the trigger and not yet
    /// consumed by a fire — not the depth of the input's transport queue. The
    /// two coincide only while nothing defers the fire, and they are different
    /// facts: the queue is what a restore must re-feed, the counter is what the
    /// scheduler itself was holding. This states the second, because the second
    /// is the one a rebuilt `Scheduler` cannot recover.
    ///
    /// # This fills THREE of the section's four fields
    ///
    /// `NodeFrameworkState` is the CARRIER's composite view (see its own doc).
    /// The fourth, `input_service`, is a TRANSPORT fact: the wire `sequence` of
    /// the last frame each per-message FIFO input's tick actually read — which
    /// the scheduler does not hold and cannot invent, so this leaves it `None`
    /// and the graph runtime's capture site fills it in from the ports. A
    /// caller that uses this value verbatim therefore states "no cursor", which
    /// is the accurate reading of a scheduler asked on its own.
    // hot-path-alloc-ok-fn: cold: introspection read for the state-capture plane
    pub fn node_framework_state(
        &self,
        node_id: &str,
    ) -> Option<crate::state_restore::NodeFrameworkState> {
        let node = self.nodes.get(node_id)?;
        Some(crate::state_restore::NodeFrameworkState {
            next_fire_ns: node.next_fire_ns,
            pending_data_count: node.pending_data_count,
            // The EMITTED-TOMBSTONE CAPTURE RULE. Export ONLY
            // `Filled` heads (backed or unbacked: both carry a meaningful
            // stamp); `Emitted` tombstones are SKIPPED.
            //
            // The framework section is a stamp-only `BTreeMap<String, u64>`
            // with no state channel, so `Emitted` is inexpressible in it and
            // the writer must choose. Skipping reproduces today's
            // cleared-on-fire capture BYTE-FOR-BYTE on the checkpoint-after-a-
            // fired-step shape, which is what keeps the "bytes identical"
            // claim true and the read floor unchanged.
            //
            // The natural export-everything implementation is WRONG, and not
            // subtly: a fired step's tombstones would serialise as bare stamps,
            // restore would mint `Filled { backed: false }` for BOTH members of
            // the consumed set, completeness would hold over two unbacked
            // heads, and the resumed run would fire ONE PHANTOM SET the
            // original continuation never fired. `Void` serves and the body
            // collapses, but the fire is REAL in the trace (fire_count records
            // even on a collapsed tick) — a fire-sequence divergence on the
            // COMMON case, any node that fired in the checkpoint step.
            //
            // RESIDUAL, stated: a checkpoint landing between a fire and the
            // next align captures nothing for the fired heads, so the resumed
            // node waits for a fresh complete set — exactly like today's
            // post-clear capture.
            sync_input_timestamps: node
                .sync_heads
                .iter()
                .filter(|(_, head)| head.is_filled())
                .map(|(input, head)| (input.clone(), head.ts))
                .collect(),
            input_service: None,
        })
    }

    /// Restore `Sync` nodes' per-input arrival timestamps from
    /// a recording's framework sections.
    ///
    /// The worked example is the contract: a `sync_window_ms = 50`
    /// fusion node checkpointed with `/cam` arrived and `/lidar` not must
    /// resume with `/cam` STILL arrived, so the next `/lidar` completes the
    /// alignment. A rebuilt scheduler starts with an empty map, so without
    /// this the node waits for a second `/cam` the recording never sent and
    /// fires later than the trace it is judged against.
    ///
    /// # It restores the SCHEDULE, and it cannot restore the DATA
    ///
    /// MEASURED in `replay_engine_test`'s `a_half_satisfied_sync_alignment_*`
    /// pair: with the alignment restored the node fires exactly where the
    /// recording says it did, and its generated body then collapses on the
    /// unserved input (a body waits rather than fabricating a value),
    /// so it publishes nothing. A trigger input's held FRAME is transport
    /// state; no anchor carries it.
    ///
    /// That is strictly closer to the recording than the alternative — one
    /// divergence class instead of two, and the remaining one names the real
    /// cause instead of reporting a schedule that never diverged — but it is
    /// not a clean resume, and the gap closes only when the recording's
    /// pre-anchor frames can be re-served to the node.
    ///
    /// # What it deliberately does not do
    ///
    /// A node the map does not name is left ALONE rather than cleared. An
    /// absent entry means the recording states nothing about that node — a
    /// v1 bag states nothing about any of them — and clearing on that
    /// basis would turn "no information" into the positive claim "no input had
    /// arrived", which is the silent-inference class this repo refuses. The
    /// build-time value is an empty map anyway, so leaving it alone is also
    /// exactly the v1 behaviour.
    ///
    /// A NON-`Sync` node is skipped for the same reason
    /// [`Self::restore_period_schedule`] skips a non-`Period` one: its trigger
    /// carries no such state, and writing into it would leave a value nothing
    /// ever reads and nothing ever clears.
    ///
    /// Must be called BEFORE the first `step()`, like every other restore seam.
    // hot-path-alloc-ok-fn: cold: state RESTORE, once at run start
    pub fn restore_sync_input_timestamps(
        &mut self,
        per_node: &std::collections::BTreeMap<String, std::collections::BTreeMap<String, u64>>,
    ) {
        for (node_id, node) in self.nodes.iter_mut() {
            if !matches!(node.policy, TriggerPolicy::Sync { .. }) {
                continue;
            }
            let Some(recorded) = per_node.get(node_id.as_str()) else {
                continue;
            };
            // Every restored entry mints an UNBACKED head — a real
            // stamp with no live frame behind it. That is what disables descent
            // for the restoring boundary (there is nothing to probe past, and a
            // live queued frame belongs to the NEXT set) and what makes the
            // head's read serve `Void` in place instead of stealing that frame.
            node.sync_heads.clear();
            for (input, ts) in recorded {
                node.sync_heads
                    .insert(input.clone(), SyncHead::restored(*ts));
            }
        }
    }

    /// Configure a per-input expected interval
    /// for `node_id`. After this call, every `step()` checks the
    /// elapsed time since the last data on `input_name`; if it
    /// exceeds `within_ms`, the node's
    /// `expect_within_missed_count` increments and the tracker
    /// resets to the current step time so the same gap isn't
    /// re-counted.
    ///
    /// `last_data_ns` is the **caller-supplied** shared
    /// `Arc<AtomicU64>` the runtime also wires into the input's subscriber
    /// (and writes from `drain_level` for trigger inputs). Minted by
    /// the caller — not here — because the runtime must hand the same
    /// `Arc` to the subscriber BEFORE it is moved into the node's
    /// `NodeContext`, which happens before this node exists in the
    /// scheduler. Initialize it to the current clock so a freshly-wired
    /// input is not instantly "missed" on the first step.
    ///
    /// `fifo_trigger` DECLARES whether this input is the node's
    /// per-message FIFO trigger — the input whose arrivals drive the node's
    /// `pending_data_count`. It is a required parameter rather than a
    /// companion setter or an inference so every caller has to state it: the
    /// scheduler cannot tell a Data node's trigger input from its
    /// latest-value context inputs (they share ONE node-level pending count),
    /// and getting it wrong is silent in both directions — `false` re-opens
    /// the backlog false positive this flag exists to close, `true` on a
    /// context input silences the held-context staleness detector.
    /// `GraphRuntime::build` derives it from `data_trigger_bindings`, the
    /// single source of truth for which input signals which node; a bare
    /// scheduler (tests, embedders driving `signal_data` by hand) states it
    /// directly.
    ///
    /// `sync_trigger_topic` is the same kind of declaration for the
    /// PER-SET Sync path — `Some(resolved topic)` iff this input is a per-set
    /// Sync trigger. Required rather than inferred for the `fifo_trigger`
    /// reason: the scheduler holds two key spaces (field name here, resolved
    /// topic in `sync_heads`) and only the runtime knows the pairing.
    // hot-path-alloc-ok-fn: cold: per-input QoS registration at graph BUILD
    pub fn set_expect_within(
        &mut self,
        node_id: &str,
        input_name: &str,
        within_ms: u64,
        last_data_ns: Arc<AtomicU64>,
        fifo_trigger: bool,
        sync_trigger_topic: Option<Arc<str>>,
    ) -> TransportResult<()> {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                node_id: node_id.to_string(),
            })?;
        let within_ns = within_ms.saturating_mul(1_000_000);
        // Seed the edge-trigger latch. `armed = true`
        // so the FIRST miss of the first regime fires an event. Seed the
        // scheduler-local window + last-observed-arrival to the anchor's
        // current value so the seed is not mistaken for a real arrival on
        // the first step and a freshly-wired input isn't instantly missed.
        let seed = last_data_ns.load(Ordering::Acquire);
        node.input_expect_within.insert(
            input_name.to_string(),
            WatchdogTracker {
                within_ns,
                last_event_ns: last_data_ns,
                armed: true,
                window_start_ns: seed,
                last_arrival_ns: seed,
                sync_topic: sync_trigger_topic,
                fifo_trigger,
                prev_backlog: false,
                backlog_suppressed_window: false,
                suppression_was_held_member: false,
                backlog_latch: FailureRegimeLatch::new(),
            },
        );
        Ok(())
    }

    /// Configure a per-node tick execution
    /// deadline. After this call, every `fire_node` measures the
    /// tick callback's wall-clock duration; if it exceeds
    /// `within_ms`, the node's `tick_within_missed_count`
    /// increments.
    // hot-path-alloc-ok-fn: cold: per-node QoS registration at graph BUILD
    pub fn set_tick_within(&mut self, node_id: &str, within_ms: u64) -> TransportResult<()> {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                node_id: node_id.to_string(),
            })?;
        node.tick_within_ns = Some(within_ms.saturating_mul(1_000_000));
        Ok(())
    }

    /// B-dur: enable/disable full-tick wall-duration recording
    /// for ALL nodes (existing and future). When enabled, each fire records
    /// its tick's wall-clock elapsed into `TraceEntry::duration_ns` (Mode B,
    /// telemetry-only — EXCLUDED from determinism equality). Off by default
    /// so the hot path pays zero `Instant` cost unless a consumer opts in.
    pub fn set_record_tick_durations(&mut self, on: bool) {
        self.record_tick_durations = on;
        for node in self.nodes.values_mut() {
            node.record_durations = on;
        }
    }

    /// Test-only: whether the master tick-duration recording gate is on
    /// (B-dur). Lets the GraphRuntime passthrough be pinned deterministically.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn record_tick_durations_enabled(&self) -> bool {
        self.record_tick_durations
    }

    /// Configure a per-output committed
    /// interval for `node_id`. Symmetric to `set_expect_within`.
    /// After this call, every `step()` checks the elapsed time
    /// since the last `signal_output_published` on `output_name`;
    /// if it exceeds `within_ms`, the node's
    /// `promise_within_missed_count` increments + a
    /// `tracing::warn!` fires + the tracker resets to current_time.
    ///
    /// `last_publish_ns` is the **caller-supplied**
    /// shared `Arc<AtomicU64>` the runtime also wires into the output's
    /// publisher (which writes it on every successful send). Minted by
    /// the caller for the same ordering reason as `set_expect_within`.
    // hot-path-alloc-ok-fn: cold: per-output QoS registration at graph BUILD
    pub fn set_promise_within(
        &mut self,
        node_id: &str,
        output_name: &str,
        within_ms: u64,
        last_publish_ns: Arc<AtomicU64>,
    ) -> TransportResult<()> {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                node_id: node_id.to_string(),
            })?;
        let within_ns = within_ms.saturating_mul(1_000_000);
        // Edge-trigger latch, symmetric to
        // `set_expect_within`.
        let seed = last_publish_ns.load(Ordering::Acquire);
        node.output_promise_within.insert(
            output_name.to_string(),
            WatchdogTracker {
                within_ns,
                last_event_ns: last_publish_ns,
                armed: true,
                window_start_ns: seed,
                last_arrival_ns: seed,
                // OUTPUT side — an output window can never
                // be suppressed by an input backlog nor by a Sync head, so the
                // tracker's backlog fields are inert here by construction.
                sync_topic: None,
                fifo_trigger: false,
                prev_backlog: false,
                backlog_suppressed_window: false,
                suppression_was_held_member: false,
                backlog_latch: FailureRegimeLatch::new(),
            },
        );
        Ok(())
    }

    /// Install the shared per-node QoS-event store so
    /// `step()` can `push_*` edge-triggered [`ExpectWithinEvent`] /
    /// [`PromiseWithinEvent`]s on a watchdog miss. The runtime mints ONE
    /// `Arc<QosEventStore>` per node, clones it into the node's
    /// `NodeContext` (the drain side) BEFORE `init()` moves the context,
    /// and hands the same `Arc` here (the emit side). Without this, the
    /// counter still bumps on every miss but no reactable event is queued
    /// — the `set_expect_within` / `set_promise_within` signatures stay
    /// store-free on purpose so scheduler-direct callers that only assert
    /// the counter (e.g. `deadline_qos_test`) are unaffected.
    // hot-path-alloc-ok-fn: cold: per-node event-store registration at graph BUILD
    pub(crate) fn register_qos_event_store(
        &mut self,
        node_id: &str,
        store: Arc<QosEventStore>,
    ) -> TransportResult<()> {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                node_id: node_id.to_string(),
            })?;
        node.qos_events = Some(store);
        Ok(())
    }

    /// Test-only: mint + register a fresh
    /// `QosEventStore` for `node_id` so scheduler-direct tests can exercise
    /// the edge-triggered emission without standing up a `GraphRuntime` +
    /// `NodeContext`. Production wiring goes through
    /// `Self::register_qos_event_store` with the runtime-minted store.
    /// `QosEventStore` is `pub(crate)`, so external tests cannot construct
    /// one themselves — this + the `take_*` accessors below are the seam.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn install_qos_event_store_for_test(&mut self, node_id: &str) -> TransportResult<()> {
        self.register_qos_event_store(node_id, Arc::new(QosEventStore::default()))
    }

    /// Test-only: drain the pending edge-triggered
    /// [`ExpectWithinEvent`] for `node_id`/`input_name`, mirroring the
    /// production `NodeContext::take_expect_within_event` drain. Returns
    /// `None` if no store is installed, the node is unknown, or no event is
    /// pending.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn take_expect_within_event(
        &mut self,
        node_id: &str,
        input_name: &str,
    ) -> Option<ExpectWithinEvent> {
        self.nodes
            .get(node_id)?
            .qos_events
            .as_ref()?
            .take_expect(input_name)
    }

    /// Test-only: output twin of
    /// [`Self::take_expect_within_event`].
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn take_promise_within_event(
        &mut self,
        node_id: &str,
        output_name: &str,
    ) -> Option<PromiseWithinEvent> {
        self.nodes
            .get(node_id)?
            .qos_events
            .as_ref()?
            .take_promise(output_name)
    }

    /// Test-only: inject an edge-triggered
    /// [`LivelinessEvent`] into `node_id`'s `QosEventStore`, so e2e tests can
    /// exercise the `#[on_event]` `LivelinessEvent` dispatch path without a
    /// real publisher transition. Mirrors the
    /// production `step()` push site, which mints + queues the event on a real
    /// liveliness transition. No-op if no store is installed or the node is
    /// unknown.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn push_liveliness_event_for_test(&mut self, node_id: &str, event: LivelinessEvent) {
        if let Some(node) = self.nodes.get(node_id) {
            if let Some(store) = node.qos_events.as_ref() {
                store.push_liveliness(event);
            }
        }
    }

    /// Test-only: drain the pending edge-triggered
    /// [`LivelinessEvent`] for `node_id`/`input_name`, mirroring the production
    /// `NodeContext::take_liveliness_event` drain. Returns `None` if no store
    /// is installed, the node is unknown, or no event is pending.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn take_liveliness_event(
        &mut self,
        node_id: &str,
        input_name: &str,
    ) -> Option<LivelinessEvent> {
        self.nodes
            .get(node_id)?
            .qos_events
            .as_ref()?
            .take_liveliness(input_name)
    }

    /// Publisher-side hook — called by an
    /// output publisher after a successful publish to record the
    /// last-publish time on `output_name`. Symmetric to
    /// `signal_input_received`. Safe to call on outputs without a
    /// configured deadline (no-op).
    pub fn signal_output_published(
        &mut self,
        node_id: &str,
        output_name: &str,
    ) -> TransportResult<()> {
        let now_ns = self.clock.now_ns();
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                // hot-path-alloc-ok: cold: error-struct field on the NodeNotFound arm; the closure
                // runs only when the id is absent (a build/wiring bug), never on a successful
                // signal
                node_id: node_id.to_string(),
            })?;
        if let Some(entry) = node.output_promise_within.get(output_name) {
            entry.last_event_ns.store(now_ns, Ordering::Release);
        }
        Ok(())
    }

    /// Register an input for backpressure-event tracking on
    /// `node_id`. Idempotent — registering the same input twice does
    /// not reset counters. Must be called at graph build time before
    /// any `signal_backpressure_event` for the input fires; without
    /// registration, `NodeHandle::backpressure_*_count(input)` returns
    /// 0 (the unregistered-input semantic).
    ///
    /// Returns the `Arc<BackpressureCounters>` for the input so the
    /// transport-layer enforcement path can bump counters lock-free
    /// without going through the scheduler API on every event.
    ///
    /// This idempotent variant has no production
    /// caller — `GraphRuntime::build` only uses
    /// [`Scheduler::register_backpressure_input_with`] (which supplies a
    /// caller-minted `Arc`). It is retained ONLY for `backpressure_counters_test`,
    /// which exercises the idempotent `or_insert_with` semantic directly, so
    /// it is gated behind `#[cfg(any(test, feature = "test-helpers"))]` to
    /// shrink the public surface to a single production registration path.
    #[cfg(any(test, feature = "test-helpers"))]
    // hot-path-alloc-ok-fn: cold: per-input backpressure registration at graph BUILD
    pub fn register_backpressure_input(
        &mut self,
        node_id: &str,
        input_name: &str,
    ) -> TransportResult<Arc<BackpressureCounters>> {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                node_id: node_id.to_string(),
            })?;
        let mut guard = node.backpressure.write().unwrap_or_else(|e| e.into_inner());
        let key: Arc<str> = Arc::from(input_name);
        let counters = guard
            .entry(Arc::clone(&key))
            .or_insert_with(|| Arc::new(BackpressureCounters::new()));
        Ok(Arc::clone(counters))
    }

    /// Register an input for backpressure-event tracking on
    /// `node_id` using a **caller-supplied** counters handle. Used by the
    /// graph runtime, which creates the `Arc<BackpressureCounters>` early
    /// (to wire it into the subscriber's `BackpressureBuffer` BEFORE the
    /// subscriber is moved into the node's `NodeContext`) and then shares
    /// that same `Arc` into the scheduler here — AFTER `add_node` — so
    /// `NodeHandle::backpressure_*_count(input)` reads the very atomics the
    /// transport path bumps. Replaces any prior registration for the input.
    // hot-path-alloc-ok-fn: cold: per-input backpressure registration at graph BUILD
    pub fn register_backpressure_input_with(
        &mut self,
        node_id: &str,
        input_name: &str,
        counters: Arc<BackpressureCounters>,
    ) -> TransportResult<()> {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                node_id: node_id.to_string(),
            })?;
        let mut guard = node.backpressure.write().unwrap_or_else(|e| e.into_inner());
        // `insert` replaces any prior `Arc` for this
        // input, which would swap the counters out from under a holder that
        // already cached the old `Arc` (e.g. a subscriber's SampleGate). The
        // production path (`GraphRuntime::build`) registers each input exactly
        // once, so a pre-existing key signals a wiring bug. Assert it loudly
        // in debug builds rather than silently replace.
        debug_assert!(
            !guard.contains_key(input_name),
            "register_backpressure_input_with called twice for ({node_id}, {input_name}); \
             each input must be registered exactly once or shared Arc holders desync"
        );
        guard.insert(Arc::from(input_name), counters);
        Ok(())
    }

    /// Register `output_name`'s discard-count mirror on `node_id`
    /// using a **caller-supplied** `Arc<AtomicU64>`. The graph runtime mints the
    /// `Arc`, installs it on the port's `CerulionPublisher`
    /// (`register_output_discard_count`, which stores the latch total into it on
    /// every discard), and shares the SAME `Arc` here — AFTER `add_node` — so
    /// `NodeHandle::output_discard_count(output)` reads the very atomic the
    /// publisher writes. Mirrors [`Self::register_backpressure_input_with`]. Registered
    /// exactly once per publisher-owning output; a pre-existing key signals a
    /// wiring bug (loud in debug builds).
    // hot-path-alloc-ok-fn: cold: per-output discard-counter registration at graph BUILD
    pub fn register_output_discard(
        &mut self,
        node_id: &str,
        output_name: &str,
        shared: Arc<AtomicU64>,
    ) -> TransportResult<()> {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                node_id: node_id.to_string(),
            })?;
        let mut guard = node
            .output_discards
            .write()
            .unwrap_or_else(|e| e.into_inner());
        debug_assert!(
            !guard.contains_key(output_name),
            "register_output_discard called twice for ({node_id}, {output_name}); \
             each output must be registered exactly once or shared Arc holders desync"
        );
        guard.insert(Arc::from(output_name), shared);
        Ok(())
    }

    /// Register `output_name`'s undelivered-notify mirror on `node_id`
    /// using a **caller-supplied** `Arc<AtomicU64>`. The graph runtime mints the
    /// `Arc`, installs it on the port's `CerulionPublisher`
    /// (`register_notify_undelivered_count`, which stores the latch total into it
    /// on every classified notify), and shares the SAME `Arc` here — AFTER
    /// `add_node` — so `NodeHandle::notify_undelivered_count(output)` reads the
    /// very atomic the publisher writes. Mirrors
    /// [`Self::register_output_discard`]. Registered exactly once per
    /// publisher-owning output; a pre-existing key signals a wiring bug (loud in
    /// debug builds).
    // hot-path-alloc-ok-fn: cold: per-output notify-counter registration at graph BUILD
    pub fn register_notify_undelivered(
        &mut self,
        node_id: &str,
        output_name: &str,
        shared: Arc<AtomicU64>,
    ) -> TransportResult<()> {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                node_id: node_id.to_string(),
            })?;
        let mut guard = node
            .notify_undelivered
            .write()
            .unwrap_or_else(|e| e.into_inner());
        debug_assert!(
            !guard.contains_key(output_name),
            "register_notify_undelivered called twice for ({node_id}, {output_name}); \
             each output must be registered exactly once or shared Arc holders desync"
        );
        guard.insert(Arc::from(output_name), shared);
        Ok(())
    }

    /// Declare `node_id`'s `throttle_ms` rate cap (in ns) so
    /// [`Self::ns_until_next_fire`] can report its DEADLINE instead of `Some(0)`.
    ///
    /// Paired with [`Self::set_pre_fire_check`] at the graph build: the closure
    /// owns the DECISION, this owns the WAKE SIZING, and both derive it from
    /// this scheduler's own `fire_count`/`last_fire_ns` through the same
    /// [`crate::graph::runtime::throttle_defers`] rule. Calling one without the other is
    /// not unsafe — it costs a throttled node the 1 ms polling it had before —
    /// which is why they are installed in the same loop.
    ///
    /// `0` is rejected: the macro already refuses `throttle_ms = 0`, and a zero
    /// here would silently mean "no cap" while reading as one.
    // hot-path-alloc-ok-fn: cold: a graph-BUILD setter. Both allocations are on
    // REFUSAL paths that end the build — the `format!` for a zero window and the
    // `to_string` naming an unknown node — and neither is reachable once the
    // graph is running. Mirrors the annotation on the sibling setters above.
    pub fn set_throttle_ns(&mut self, node_id: &str, throttle_ns: u64) -> TransportResult<()> {
        if throttle_ns == 0 {
            return Err(TransportError::GraphError {
                reason: format!(
                    "node '{node_id}': a throttle window of 0 ns is not a rate cap.                      `throttle_ms` must be > 0 (the macro refuses 0 at compile time),                      so reaching this means a caller computed the window rather than                      reading the declaration"
                ),
            });
        }
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                node_id: node_id.to_string(),
            })?;
        node.throttle_ns = Some(throttle_ns);
        Ok(())
    }

    /// Install (or replace) the Block defer-the-fire
    /// pre-fire predicate for `node_id`. The closure is invoked at the
    /// top of every `evaluate_node` call BEFORE any trigger policy
    /// evaluation. If it returns `true`, the node's tick is skipped
    /// for this step (no callback invocation, no fire_count bump, no
    /// trace entry, no panic_count change).
    ///
    /// **One-publish-per-tick assumption**: the
    /// `block` outstanding mirror is incremented once per published frame
    /// and the pre-fire defers when `outstanding >= threshold`. A node that
    /// publishes MORE than once per tick can drive the mirror past the
    /// threshold within a single un-deferred tick (the pre-fire only gates
    /// ENTRY to a tick, not mid-tick publishes). Multi-publish-per-tick is a
    /// documented future limitation; the Period catch-up burst — the other
    /// multi-publish-per-step path — is clamped by re-checking the pre-fire
    /// before each catch-up fire (see `evaluate_node`).
    ///
    /// The closure is responsible for bumping the appropriate
    /// per-input `block_fires_deferred_count` counter via the cached
    /// `Arc<BackpressureCounters>` from `register_backpressure_input_with`
    /// — see `record_backpressure_event` for the shared bump+warn
    /// helper. The scheduler does not bump counters on its own
    /// because attribution is per-downstream-subscriber, which the
    /// scheduler does not directly track.
    ///
    /// Typically wired by `GraphRuntime` after graph build:
    /// `set_pre_fire_check` is called once per producer node with a
    /// closure that captures Arc refs to the producer's publishers
    /// and consults each publisher's downstream consumer-mix state.
    ///
    /// Returns `Err(NodeNotFound)` if `node_id` is not in the scheduler.
    ///
    /// **Panic safety**: the installed closure is wrapped in
    /// `std::panic::catch_unwind` inside `evaluate_node`. A panic in
    /// the closure is logged via `tracing::error!` and treated as "no
    /// defer" (the conservative choice — defer-on-panic could lock the
    /// scheduler if the closure is permanently buggy). The scheduler
    /// does not increment any counter for the panic, and there is no
    /// circuit breaker — a perpetually-panicking closure emits one
    /// error log per `step()`. Inspect the closure for a bug rather
    /// than tolerate the spam.
    ///
    /// **Period catch-up**: when the closure
    /// returns `true` at the top of `evaluate_node` for a
    /// `TriggerPolicy::Period` node, `next_fire_ns` is NOT advanced — the
    /// defer happens BEFORE the Period match arm runs. On resume (predicate
    /// returns false at the top), the catch-up loop fires missed intervals
    /// — but it RE-CHECKS this predicate before EACH catch-up fire and
    /// BREAKs (rewinding `next_fire_ns` to the first un-fired interval) the
    /// moment a downstream block consumer's queue fills again. The resume
    /// burst is therefore bounded by BOTH `Period.max_catchup` AND the live
    /// queue depth, so it can never overflow a block consumer's queue or
    /// desync the outstanding mirror. Users wiring Block backpressure to
    /// high-frequency Period producers MAY still set `max_catchup: Some(N)`
    /// to additionally bound latency, but it is no longer required for
    /// data-loss safety.
    ///
    /// **Calling cadence**: for `Sync`/`External` this closure is invoked
    /// exactly ONCE per `evaluate_node` per `step()`. For `Period` it is
    /// invoked once at the top of `evaluate_node` PLUS once before each
    /// catch-up fire (the catch-up re-check above), so on a multi-interval catch-up
    /// step it runs up to `1 + catchup_count` times. `Data` now has
    /// the same shape — once at the top, plus once before each fire of the
    /// step's burst, which is what makes `throttle_ms` mean "at most one fire
    /// per step" and stops a `block` producer mid-burst the instant a
    /// downstream queue fills. Each invocation that
    /// defers bumps `block_fires_deferred_count` once — this is intentional
    /// (each deferred catch-up attempt is a distinct defer event). Closures
    /// should consult their predicate exactly once per invocation; a closure
    /// that itself calls a counter-bumping predicate multiple times within a
    /// single invocation would over-count.
    // hot-path-alloc-ok-fn: cold: installs the pre-fire check ONCE at graph BUILD
    pub fn set_pre_fire_check<F>(&mut self, node_id: &str, check: F) -> TransportResult<()>
    where
        F: Fn(u64) -> bool + Send + Sync + 'static,
    {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                node_id: node_id.to_string(),
            })?;
        node.pre_fire_check = Some(Arc::new(check));
        Ok(())
    }

    /// Install the `DrainSource::Unified` REFILL hook for a `Data`
    /// node's trigger input — the thing that lets one step serve a whole
    /// queued burst instead of one frame.
    ///
    /// `drain` must pop AT MOST ONE frame off `input`'s body subscriber and
    /// freeze it for the tick's read — i.e. it must be
    /// [`crate::graph::node::NodeEntry::refill_trigger_input`], through the node
    /// lock — and return that call's `(popped, latest_ts)` verbatim. It is
    /// called ONLY between fires of a Data burst, never speculatively: the loop
    /// asks for a frame only when it is about to fire one.
    ///
    /// It must be the REFILL entry point and NOT the boundary drain: the
    /// boundary re-offers an unserved frozen head, and a hook that reported that
    /// re-offer as a fresh frame would re-fire the node on ONE frame up to
    /// `DATA_PENDING_CARRY_CLAMP` times per step (see this module's
    /// `TriggerRefill`).
    ///
    /// Installed once per Unified binding at graph build. A `Separate` binding
    /// installs NOTHING: its boundary drain signals one arrival per queued
    /// frame, so `pending_data_count` already describes the burst.
    ///
    /// Returns `Err(NodeNotFound)` if `node_id` is not in the scheduler, and
    /// `Err(SchedulerError)` for a non-`Data` node (a data-trigger binding on a
    /// non-Data policy is a wiring desync, and installing a hook nothing would
    /// ever call would hide it).
    // hot-path-alloc-ok-fn: cold: installs one Unified binding's refill hook ONCE at graph BUILD
    pub fn set_trigger_refill<F>(
        &mut self,
        node_id: &str,
        input: &str,
        drain: F,
    ) -> TransportResult<()>
    where
        F: Fn() -> (u64, Option<u64>) + Send + Sync + 'static,
    {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                node_id: node_id.to_string(),
            })?;
        if !matches!(node.policy, TriggerPolicy::Data) {
            return Err(TransportError::SchedulerError {
                reason: format!(
                    "set_trigger_refill() not valid for node '{}' with policy {:?} — \
                     only a Data node has a trigger input to refill",
                    node_id, node.policy
                ),
            });
        }
        node.trigger_refill = Some(TriggerRefill {
            input: Arc::from(input),
            drain: Arc::new(drain),
        });
        Ok(())
    }

    /// Scheduler-side hook used by the transport layer to
    /// record a backpressure enforcement event on `(node_id, input_name)`.
    /// Bumps the per-variant counter, emits a structured
    /// `tracing::warn!`, and is idempotent if the input is not
    /// registered (no-ops on unknown inputs — matches the
    /// "unregistered = 0 counter" contract).
    ///
    /// For high-volume hot paths the transport layer SHOULD cache the
    /// `Arc<BackpressureCounters>` returned by `register_backpressure_input_with`
    /// and bump counters directly — every production site does (the block
    /// defer path holds its cached `Arc` and calls
    /// `record_backpressure_event_n` directly). This API is the
    /// manual/test event-injection seam; it has no production callers.
    pub fn signal_backpressure_event(
        &self,
        node_id: &str,
        input_name: &str,
        policy: BackpressurePolicy,
    ) -> TransportResult<()> {
        let node = self
            .nodes
            .get(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                // hot-path-alloc-ok: cold: error-struct field on the NodeNotFound arm; the closure
                // runs only when the id is absent (a build/wiring bug), never on a successful
                // signal
                node_id: node_id.to_string(),
            })?;
        let guard = node.backpressure.read().unwrap_or_else(|e| e.into_inner());
        if let Some(counters) = guard.get(input_name) {
            record_backpressure_event(node_id, input_name, policy, counters);
        }
        Ok(())
    }

    /// Get a handle to an existing node.
    pub fn node_handle(&self, id: &str) -> Option<NodeHandle> {
        self.nodes.get(id).map(|node| {
            NodeHandle::new(
                // hot-path-alloc-ok: cold: mints an operator-facing NodeHandle; an introspection
                // read, not a step
                id.to_string(),
                Arc::clone(&node.fire_count),
                Arc::clone(&node.last_fire_ns),
                Arc::clone(&node.panic_count),
                Arc::clone(&node.pending_data_count_shared),
                Arc::clone(&node.expect_within_missed),
                Arc::clone(&node.promise_within_missed),
                Arc::clone(&node.tick_within_missed),
                Arc::clone(&node.expect_within_backlogged),
                Arc::clone(&node.in_tick_since_ns),
                Arc::clone(&node.backpressure),
                Arc::clone(&node.publisher_disconnects_observed),
                Arc::clone(&node.signal_failed),
                Arc::clone(&node.output_discards),
                Arc::clone(&node.notify_undelivered),
                Arc::clone(&node.sync_counters),
            )
        })
    }

    /// Hand the runtime liveliness sweep a clone of
    /// `node_id`'s per-node publisher-disconnect counter `Arc` so it can
    /// bump it lock-free on every `Lost` transition it observes on one of
    /// the node's inputs. Mirrors how `register_backpressure_input_with`
    /// shares a counter `Arc` with the transport path: the same atomics the
    /// `NodeHandle::publisher_disconnects_observed_count` accessor reads.
    /// `None` for an unknown node.
    pub fn publisher_disconnects_counter(&self, node_id: &str) -> Option<Arc<AtomicU64>> {
        self.nodes
            .get(node_id)
            .map(|node| Arc::clone(&node.publisher_disconnects_observed))
    }

    /// Install the shared per-node discard-signal + replay-suppress
    /// Arcs onto `node_id`'s `ScheduledNode`, REPLACING the fresh defaults
    /// `add_node` minted. `GraphRuntime` calls this once per node with the SAME
    /// two Arcs it also injects into every one of that node's publishers, so the
    /// RECORD-side delta (publisher bumps `discard`, `fire_node_into` reads it)
    /// and the REPLAY-side gate (`fire_node_into` sets `suppress`,
    /// `OutputProxy::Drop` reads it) both observe the shared state. `None`-safe:
    /// an unknown node id is a wiring desync — loud `Err`, never a silent drop.
    // hot-path-alloc-ok-fn: cold: installs a node's discard signals ONCE at graph BUILD
    pub(crate) fn set_node_discard_signals(
        &mut self,
        node_id: &str,
        discard: Arc<std::sync::atomic::AtomicU32>,
        suppress: Arc<std::sync::atomic::AtomicBool>,
    ) -> TransportResult<()> {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                node_id: node_id.to_string(),
            })?;
        node.discard_signal = discard;
        node.replay_suppress = suppress;
        Ok(())
    }

    /// Install the shared NODE-DEATH ledger onto
    /// `node_id`'s `ScheduledNode`, REPLACING the fresh default `add_node`
    /// minted — the `set_node_discard_signals` seam, same shape, same reason.
    ///
    /// `GraphRuntime` calls this once per node with the ONE ledger it also hands
    /// to every tick callback, so the scheduler's DISABLE edge and the runtime's
    /// POISON transition write the SAME place — sharing one cap, one drain and
    /// one ordering. They are NOT deduplicated against each other: the ledger's
    /// key is `(node, cause)`, so a node that is both disabled and poisoned
    /// yields two records, and the recorder's gate coalesces them on the shared
    /// subject. `None`-safe: an unknown node id is a wiring desync — loud `Err`,
    /// never a silent drop.
    ///
    /// NOT `pub`, for the same reason its sibling is not: the production caller
    /// is the runtime, and a test that needs it goes through
    /// [`Self::set_node_death_ledger_for_test`].
    // hot-path-alloc-ok-fn: cold: installs a node's death ledger ONCE at graph BUILD
    pub(crate) fn set_node_death_ledger(
        &mut self,
        node_id: &str,
        ledger: Arc<crate::scheduler::node_death::NodeDeathLedger>,
    ) -> TransportResult<()> {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                node_id: node_id.to_string(),
            })?;
        node.node_death = ledger;
        Ok(())
    }

    /// TEST SEAM: install a node-death ledger on a BARE scheduler.
    ///
    /// The scheduler's own disable edge is unreachable through a `GraphRuntime`
    /// (the entry mutex poisons on the first panic — see
    /// [`node_death`](crate::scheduler::node_death)), so the only way to drive it
    /// at all is a bare `Scheduler`, and the only way to OBSERVE it is a ledger
    /// the test holds.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn set_node_death_ledger_for_test(
        &mut self,
        node_id: &str,
        ledger: Arc<crate::scheduler::node_death::NodeDeathLedger>,
    ) -> TransportResult<()> {
        self.set_node_death_ledger(node_id, ledger)
    }

    /// Install `node_id`'s read-outcome stages — the SAME
    /// `Arc`s the runtime shared into that node's subscribers at wiring time
    /// (the `set_node_discard_signals` pattern), so a subscriber-staged read
    /// outcome is drained by [`Self::merge_read_outcomes`] over shared state.
    /// Called once per node at graph build; the stages stay DISARMED (inert)
    /// until a recording installs the trace ring. `None`-safe: an unknown
    /// node id is a wiring desync — loud `Err`, never a silent drop.
    // hot-path-alloc-ok-fn: cold: installs a node's read-outcome stages ONCE at graph BUILD
    // (the `set_node_discard_signals` sibling above, same seam, same reason)
    pub(crate) fn set_node_read_stages(
        &mut self,
        node_id: &str,
        stages: Vec<Arc<crate::read_outcome::ReadOutcomeStage>>,
    ) -> TransportResult<()> {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                node_id: node_id.to_string(),
            })?;
        node.read_stages = stages;
        Ok(())
    }

    /// TEST SEAM: install a node's discard-signal + replay-suppress Arcs
    /// onto a bare scheduler (the production runtime wires these in
    /// `GraphRuntime::build`). Lets a scheduler-level test drive the record-side
    /// marker (`fire_node_into` delta → `push_fire` bit) and the replay-side
    /// suppress flag without a full graph build.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn set_node_discard_signals_for_test(
        &mut self,
        node_id: &str,
        discard: Arc<std::sync::atomic::AtomicU32>,
        suppress: Arc<std::sync::atomic::AtomicBool>,
    ) -> TransportResult<()> {
        self.set_node_discard_signals(node_id, discard, suppress)
    }

    /// REPLAY side: install this step's recorded per-fire discard verdicts,
    /// keyed by node id. Called ONCE per step by the replay engine BEFORE
    /// `step()` — every node's queue is CLEARED first (a leftover from a divergent
    /// previous step never leaks forward), then the map's entries are installed.
    /// A node absent from the map fires with an empty queue (never suppressed).
    /// `fire_node_into` pops one bool per fire, in recorded order.
    pub fn set_replay_discards(
        &mut self,
        discards: std::collections::HashMap<Arc<str>, std::collections::VecDeque<bool>>,
    ) {
        for node in self.nodes.values_mut() {
            node.replay_discard_queue.clear();
        }
        for (id, queue) in discards {
            if let Some(node) = self.nodes.get_mut(id.as_ref()) {
                node.replay_discard_queue = queue;
            }
        }
    }

    /// REPLAY side, defensive: true iff every node's replay-discard queue is
    /// drained. A leftover means the recording listed MORE fires for some node
    /// this step than replay produced — a fire-schedule divergence already
    /// surfaced by the exit-6 structural comparator, so the replay engine only
    /// breadcrumbs on this (never a hard failure — a genuine divergence must
    /// reach its exit-6 verdict, not panic).
    pub fn replay_discards_drained(&self) -> bool {
        self.nodes
            .values()
            .all(|n| n.replay_discard_queue.is_empty())
    }

    /// Returns the number of scheduled nodes.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Interned ids of every node whose trigger policy is
    /// [`TriggerPolicy::External`], in insertion order (the graph's declaration
    /// order — `IndexMap`). The live loop's `collect_external_sources` iterates
    /// these ONCE at `run_live` entry to query each external-policy node's
    /// [`crate::graph::node::ExternalSource`]. Read-only; no behavior change.
    // hot-path-alloc-ok-fn: cold: its own doc records the contract — the live loop's
    // `collect_external_sources` iterates this ONCE at `run_live` entry
    pub(crate) fn external_policy_node_ids(&self) -> Vec<Arc<str>> {
        self.nodes
            .values()
            .filter(|n| matches!(n.policy, TriggerPolicy::External))
            .map(|n| Arc::clone(&n.node_id))
            .collect()
    }

    /// Count of [`TriggerPolicy::External`] nodes — a conservative
    /// upper bound on the live loop's external WaitSet attachments, passed to
    /// [`crate::graph::waitset::check_waitset_attachment_capacity`] at build
    /// (which runs BEFORE `collect_external_sources`, so a node returning
    /// `HostDriven` / `None` may never materialize a binding — the count is a
    /// safe over-estimate, never an under-estimate).
    pub(crate) fn external_policy_node_count(&self) -> usize {
        self.nodes
            .values()
            .filter(|n| matches!(n.policy, TriggerPolicy::External))
            .count()
    }

    /// Advance simulated time by `delta` and evaluate all trigger policies.
    ///
    /// For `VirtualClock`, this advances the clock then evaluates triggers.
    /// For `RealClock`, this evaluates triggers at the current wall time
    /// (the `delta` is ignored — time reads from the kernel clock).
    ///
    /// # Execution Order
    ///
    /// Nodes are evaluated in insertion order (IndexMap). Within a single step,
    /// a Period node may fire multiple times if delta > interval (catch-up).
    pub fn step(&mut self, delta: Duration) {
        // This flat wrapper stays BYTE-IDENTICAL to its
        // pre-split form — advance the clock ONCE, then evaluate every node
        // in insertion order. The extracted `begin_step` / `evaluate_one`
        // pieces let `GraphRuntime::step` interleave per-DAG-level drains
        // between fires; calling `step` directly (the `scheduler_test.rs`
        // callers) replays the exact original order.
        let new_time = self.begin_step(delta);
        for i in 0..self.nodes.len() {
            self.evaluate_one(i, new_time);
        }
    }

    /// Advance the simulated clock ONCE for this step and
    /// return the new logical time. Extracted verbatim from the head of the
    /// pre-split `step` so the level executor (`GraphRuntime::step`) can advance
    /// the clock a single time per tick, then evaluate DAG levels in order with
    /// drains interleaved — all node evaluations in one tick share this one
    /// `new_time` (the determinism contract: the clock advances once per tick).
    ///
    /// `begin_step` is the SINGLE per-step clock-advance seam,
    /// shared by the flat `Scheduler::step` and the `GraphRuntime::step` level
    /// executor (and the live path via `live_step -> step`). On the gating
    /// (polled/replay) path `delta` MUST be a run-INDEPENDENT logical quantum —
    /// the poll-loop's fixed delta OR a replay-bag recorded duration — NOT a
    /// wall-derived or max-of-peers value, so replay is bit-for-bit identical to
    /// the polled gating run (Principle #7). The advance routes through
    /// [`VirtualClock::advance_by_recorded`] via `ClockInner::advance`; see it
    /// for the canonical contract.
    pub(crate) fn begin_step(&mut self, delta: Duration) -> u64 {
        // Bump the absolute logical-step counter ONCE per
        // step HERE — `begin_step` is the single seam both the flat
        // `Scheduler::step` and the `GraphRuntime::step` level executor share, so
        // this is the one place that names every step consistently across paths.
        // The 0-based index stamped onto this step's `TraceEntry`s is
        // `current_step()` (== `steps_begun − 1`). Saturating against the
        // (unreachable in practice) overflow of a u64 step count.
        self.steps_begun = self.steps_begun.saturating_add(1);
        // Bump this rank's step-progress companion. Placed at the
        // step's OPENING, beside the counter it mirrors, so a rank that wedges
        // ANYWHERE inside a step — including inside a tick, where the per-node
        // pairs also catch it — stops advancing it. Advancing at the step's END
        // would report a rank that never finishes a step as making no progress
        // AND one that never starts a step identically, which is the same verdict
        // for the same fault and buys nothing; advancing at the open keeps the two
        // words independent, which is what lets the supervisor say WHICH of the
        // two conditions it saw. `None` unless a worker installed a page.
        if let Some(page) = self.wedge_page.as_ref() {
            page.advance_step();
        }
        // Refresh THIS step's catch-up clamp from ONE
        // reading of the arm, immediately after the step index is named and
        // before any node is evaluated. Every `decide_node` of this step then
        // READS the derived value, so the whole step runs under one cap
        // whatever a recorder does to the word mid-step (see the field docs).
        // Detached (the default) this is a `None` test on a field already in
        // cache — no load of the SHM word, no arithmetic.
        self.catchup_cap_override = self.catchup_arm.as_ref().and_then(|arm| {
            catchup_clamp::derive_catchup_cap_override(
                arm.onset(),
                self.steps_begun.saturating_sub(1),
            )
        });
        let delta_ns = delta.as_nanos() as u64;
        let new_time = self.clock.advance(delta_ns);
        // When recording, push ONE StepBoundary
        // record per scheduler step — EVERY step, no-fire steps INCLUDED (the
        // boundary is the only record such a step produces; replay needs the
        // full clock progression, so eliding empty steps is FORBIDDEN by the
        // addendum). `fire_time_ns` is the step's ADVANCED clock (the value
        // this function returns), `step` is the 0-based `current_step()`
        // index. Boundaries are RING-ONLY: they never enter the in-memory
        // `TraceEntry` trace, so the recording-ON/OFF byte-identity firewall
        // and the `max_trace_entries` cap semantics are untouched by
        // construction. SPSC holds: `begin_step` runs on the step-calling
        // thread only.
        if let Some(hook) = self.trace_ring.as_mut() {
            hook.push_step_boundary(self.steps_begun.saturating_sub(1), new_time);
        }
        new_time
    }

    /// The 0-based index of the logical step currently
    /// executing — the value stamped onto every `TraceEntry.step` produced this
    /// step (the cross-process trace merge's primary sort key). Valid AFTER
    /// [`Self::begin_step`] has run for the step; `saturating_sub` yields `0`
    /// before the first step rather than underflowing. Read by the flat path
    /// (`evaluate_one`) and the runtime path (`GraphRuntime::step`, ONCE before
    /// the per-level loop — `step` is per-step, never per-level).
    pub(crate) fn current_step(&self) -> u64 {
        // A pre-first-`begin_step()` read is a caller bug: the returned `0` would
        // alias real step 0. Catch it loudly in debug; the `saturating_sub` below
        // stays the release-safe fallback (yields `0` rather than underflowing).
        debug_assert!(
            self.steps_begun > 0,
            "current_step() read before the first begin_step()"
        );
        self.steps_begun.saturating_sub(1)
    }

    /// Install the checkpoint arm whose onset gates this
    /// scheduler's `Period` catch-up clamp.
    ///
    /// Takes effect at the NEXT `begin_step` — never mid-step, which is
    /// the whole point of the once-per-step read. Idempotent-by-replacement: a
    /// second attach simply supersedes the first (a re-attaching recorder
    /// creates a new word, so the old handle must not linger).
    pub fn attach_catchup_arm(&mut self, arm: Arc<dyn catchup_clamp::CatchupArm>) {
        self.catchup_arm = Some(arm);
    }

    /// Install this rank's wedge page and bind each node to its SLOT.
    ///
    /// `slots` maps node id → slot index, and the mapping is the SUPERVISOR's:
    /// the process that will READ the page decides which slot means which node,
    /// exactly as the supervisor computes the mid-level barrier flags
    /// rather than letting each worker re-derive them. Two derivations of one
    /// ordering is how the supervisor comes to name node B while node A is
    /// wedged — a wrong answer from the feature whose whole job is to name the
    /// offender.
    ///
    /// Registers ONLY nodes this scheduler holds. A desync is reported LOUDLY in
    /// BOTH directions — the page is then still installed for the nodes that DID
    /// match, because a partial alarm beats none and the report says which nodes it
    /// cannot cover:
    ///
    /// - a slot naming a node that is not here (the map is AHEAD of this worker);
    /// - a node this scheduler holds that the map OMITS (the map is BEHIND it).
    ///
    /// The second direction is the one that goes silent if nobody looks for it: an
    /// unmapped node simply never marks, so its slot is one the supervisor never
    /// reads and the omission looks exactly like a rank with fewer nodes. The
    /// forward direction at least leaves an unmoving slot the supervisor watches.
    ///
    /// Install ONCE, before step 0. A second install is REFUSED (the
    /// [`Self::set_trace_ring_producer`] rule): the supervisor's observer is
    /// already accumulating dwell against the first page's counters, and a swap
    /// would silently re-anchor every node's pair mid-run.
    ///
    /// Returns whether the page was INSTALLED, so a caller cannot report a binding
    /// this refused (a refused re-install returns `false` while the first page
    /// stays in force).
    // hot-path-alloc-ok-fn: cold: installs the wedge page ONCE, before step 0 (its own
    // doc REFUSES a second install); the id lists it builds are the desync-report arms
    pub fn set_wedge_page(
        &mut self,
        page: Arc<dyn WedgeMarker>,
        slots: &indexmap::IndexMap<String, usize>,
    ) -> bool {
        if self.wedge_page.is_some() {
            tracing::error!(
                "a wedge page is already installed — REFUSING the re-install \
                 and keeping the first (the contract is install ONCE, before step 0; a \
                 mid-run swap would re-anchor every node's seq pair under an observer \
                 already accumulating dwell against the old one)"
            );
            return false;
        }
        let mut bound = 0usize;
        let mut unknown: Vec<&str> = Vec::new();
        for (node_id, slot) in slots {
            match self.nodes.get_mut(node_id.as_str()) {
                Some(node) => {
                    node.wedge = Some((Arc::clone(&page), *slot));
                    bound += 1;
                }
                None => unknown.push(node_id.as_str()),
            }
        }
        if !unknown.is_empty() {
            tracing::error!(
                unknown_nodes = ?unknown,
                bound,
                "the wedge-page slot map names node(s) this worker does not \
                 hold — they get NO wedge alarm (their supervisor will see their slots \
                 never move). This is a supervisor/worker plan desync; the page is still \
                 installed for the nodes that matched"
            );
        }
        if bound != self.nodes.len() {
            let unwatched: Vec<&str> = self
                .nodes
                .iter()
                .filter(|(_, n)| n.wedge.is_none())
                .map(|(id, _)| id.as_str())
                .collect();
            tracing::error!(
                unwatched_nodes = ?unwatched,
                bound,
                held = self.nodes.len(),
                "the wedge-page slot map OMITS node(s) this worker holds — \
                 they get NO wedge alarm, and unlike an unknown slot the omission is \
                 INVISIBLE to the supervisor (an unmapped node never marks, so there is \
                 no unmoving slot to notice). This is a supervisor/worker plan desync; \
                 the page is still installed for the nodes that matched"
            );
        }
        self.wedge_page = Some(page);
        true
    }

    /// Principle #3: the `max_catchup` override THIS step
    /// is running under — `None` when nothing is clamped.
    ///
    /// Observable independently of execution, because the alternative is
    /// inferring the clamp from a fire count, which is exactly the quantity the
    /// clamp changes. Valid AFTER `begin_step` has run for the step.
    pub fn catchup_cap_override(&self) -> Option<u32> {
        self.catchup_cap_override
    }

    /// Evaluate the single node at insertion index `idx`
    /// against the already-advanced `new_time`. This is the EXACT per-node body
    /// of the pre-split `step` loop (the `input_expect_within` window check, the
    /// `output_promise_within` window check, then `evaluate_node`), used by the
    /// flat `step` path. The two QoS window loops are factored
    /// into `run_qos_windows` (the level path's `decide_fires` shares them) and
    /// `evaluate_node` into `decide_node ∘ tick_node`; this fused `evaluate_one`
    /// stays observably identical (same counter cadence, same trace, same
    /// order). Preserves the field-disjoint borrow pattern: the `Arc::clone`
    /// handles are hoisted before the `iter_mut()` borrows so each loop body
    /// touches no other `node.*` field, and `evaluate_node` takes `&mut self.trace`
    /// (a disjoint field from `self.nodes`).
    fn evaluate_one(&mut self, idx: usize, new_time: u64) {
        // The QoS windows run for EVERY node every step
        // (their cadence is independent of whether the node fires), so they are
        // factored into `run_qos_windows`. Re-fetch the node afterwards: the
        // borrow patterns are disjoint, but the helper takes `&mut self` so the
        // node borrow from `run_qos_windows` ends before this re-fetch.
        self.run_qos_windows(idx, new_time);
        let max_trace = self.max_trace_entries;
        // The logical step index for this fire, read
        // BEFORE the `&mut self.nodes` borrow below (constant for the whole
        // `step()` call). Same value on every node this step.
        let step = self.current_step();
        // Copied out BEFORE the `self.nodes` borrow below —
        // one value for the whole step, derived in `begin_step`.
        let cap_override = self.catchup_cap_override;
        // `idx` comes from a `get_index_of` /
        // a `0..self.nodes.len()` loop, so it is always in range — pin that
        // invariant loud in debug before the unwrap below.
        debug_assert!(idx < self.nodes.len(), "evaluate_one: idx out of range");
        let node_id = Arc::clone(&self.nodes[idx].node_id);
        // The flat path honours an installed TRACE-DRIVEN plan for
        // the same reason the level paths do — "a plan-driven scheduler never
        // consults a trigger" is a property of the SCHEDULER, not of one caller,
        // and a seam that quietly kept deciding would be a second, disagreeing
        // fire source. One never-taken branch when no plan is installed.
        if self.replay_plan.is_some() {
            if let Some(kind) = self.take_planned_fire(idx, &node_id, new_time) {
                let (_, node) = self.nodes.get_index_mut(idx).unwrap();
                Self::tick_node(
                    &node_id,
                    node,
                    kind,
                    0,
                    step,
                    &mut self.trace,
                    max_trace,
                    &mut self.entries_appended,
                    self.trace_ring.as_mut(),
                );
            }
            return;
        }
        let (_, node) = self.nodes.get_index_mut(idx).unwrap();
        // The flat `Scheduler::step` path has NO
        // levelization, so every fire is stamped `global_level = 0` (the
        // sentinel) but carries the REAL `step` index. The level executor
        // (`GraphRuntime::step` -> `run_level`) threads the real global level
        // through the `tick_decided*` / `evaluate_nodes_fused` seams instead.
        Self::evaluate_node(
            &node_id,
            node,
            new_time,
            0,
            step,
            &mut self.trace,
            max_trace,
            &mut self.entries_appended,
            self.trace_ring.as_mut(),
            cap_override,
        );
    }

    /// Run the two per-port QoS watchdog windows
    /// (`expect_within_ms` inputs, `promise_within_ms` outputs) for the node at
    /// insertion index `idx` against the already-advanced `new_time`. Extracted
    /// VERBATIM from the head of the pre-split `evaluate_one` so both the flat
    /// `evaluate_one` and the level `decide_fires` paths run the windows
    /// identically (same counter cadence, same edge-triggered event emission)
    /// regardless of whether the node fires this step. Preserves the
    /// field-disjoint borrow pattern: the `Arc::clone` handles are hoisted
    /// before the `iter_mut()` borrows so each loop body touches no other
    /// `node.*` field.
    fn run_qos_windows(&mut self, idx: usize, new_time: u64) {
        debug_assert!(idx < self.nodes.len(), "run_qos_windows: idx out of range");
        // SAFETY: `idx` is in range (debug_asserted above); callers pass a
        // `get_index_of` / `0..self.nodes.len()` index, so `get_index_mut` is Some.
        let (_, node) = self.nodes.get_index_mut(idx).unwrap();
        let node_id = Arc::clone(&node.node_id);
        // Check per-input `expect_within_ms`
        // windows BEFORE evaluating triggers. For each tracked input,
        // if `new_time - window_start_ns > within_ns`, increment
        // `expect_within_missed` (EVERY window — the counter cadence
        // is unchanged) and emit a structured `tracing::warn!`. The
        // miss advances ONLY the scheduler-local `window_start_ns`;
        // the shared anchor is arrival-only-written (the scheduler
        // never touches it).
        //
        // ALSO queue a reactable
        // `ExpectWithinEvent`, but EDGE-TRIGGERED — once per silence
        // regime. `armed` gates emission. A real arrival is detected
        // by the shared anchor differing from `last_arrival_ns` —
        // UNAMBIGUOUS because arrivals are the anchor's sole writers
        // (no aliasing with a scheduler-authored value) — and rearms
        // the latch + resets the window to the data's timestamp.
        // Hoist the per-node counter + store clones BEFORE the
        // `iter_mut()` borrow so the loop body touches no other
        // `node.*` field.
        let expect_missed = Arc::clone(&node.expect_within_missed);
        // This node's signalled-but-unfired arrival BACKLOG, read
        // ONCE before the `iter_mut()` borrow below. A window that lapses on
        // the node's FIFO trigger input while that input still carries
        // unconsumed arrivals is BACKLOG, not silence: the producer's frames
        // are queued and the node has simply not been allowed to serve them
        // (its own `throttle_ms` / `block` gate deferred the fire, or a
        // collapsed tick never reached the read). Counting such a window as
        // an `expect_within_ms` MISS inverts the knob's meaning — it is
        // documented as a producer-liveness detector — and, at a defer longer
        // than the window, produced a per-window `warn!` on a perfectly
        // healthy graph (the log-flood class).
        //
        // The counter is taken as a plain `&AtomicU64` rather than an
        // `Arc::clone`: cloning would be an atomic RMW on the strong count
        // (plus a matching decrement at drop) for EVERY node on EVERY step,
        // including nodes with no watched inputs, on a path that carries a
        // zero-alloc regression gate. A disjoint-field borrow against
        // `node.input_expect_within.iter_mut()` compiles under NLL and costs
        // nothing.
        let pending_data = node.pending_data_count;
        let data_backlog = pending_data > 0;
        // `signal_data` REJECTS any non-Data policy (bumping `signal_failed`
        // and returning `Err`), and the only writers of `pending_data_count`
        // are that signal and the Data arm of `decide_node` — so a non-zero
        // pending count already IMPLIES `TriggerPolicy::Data`. Pinned as an
        // invariant here rather than added as a `matches!` conjunct to the
        // guard below: a conjunct no input can ever falsify is a branch no
        // test can kill.
        debug_assert!(
            !data_backlog || matches!(node.policy, TriggerPolicy::Data),
            "pending_data_count > 0 on a non-Data node — signal_data must \
             reject non-Data policies (see Scheduler::signal_data)"
        );
        let expect_backlogged: &AtomicU64 = &node.expect_within_backlogged;
        // The Sync analogue of `data_backlog`.
        //
        // A per-set Sync trigger input anchors its watchdog when a frame is
        // POPPED. Once its head is `Filled` the align pass SKIPS it (PASS 1
        // leaves an already-filled head alone — it is already this set's
        // member), so nothing pops and nothing anchors. On a healthy node that
        // is invisible: arrivals keep the anchor fresh, no window ever lapses,
        // and this branch is never reached. It bites when the node CANNOT
        // consume the member — a starved PARTNER, or the node's own
        // `throttle_ms` / `block` gate deferring a complete set — and then the
        // FLOWING input accrues `expect_within_missed_count` and a per-window
        // `warn!` while the real fault is elsewhere. That is the Data-backlog
        // inversion verbatim ("the knob is documented as a producer-liveness
        // detector"), reached by a different route.
        //
        // The rule is exactly the Data arm's, restated for a head: a window that
        // lapses on an input whose MEMBER is already held is not silence. It is
        // reported (debug), counted into the disjoint `expect_within_backlogged`
        // bucket, and NOT counted as a missed deadline.
        //
        // Deliberately NOT gated on the alignment being INCOMPLETE. That
        // conjunct was written to stop the suppression swallowing healthy
        // windows — `run_qos_windows` runs in the DECIDE phase, after the
        // boundary align has filled the heads and BEFORE the fire, so on a
        // healthy step every head IS `Filled` at this instant. MEASURED: it
        // defends nothing, because this branch is only reached once a window
        // has ALREADY lapsed, and a window can only lapse when no arrival has
        // anchored — which on a flowing input does not happen. What the
        // conjunct DID exclude is the one shape where suppression is most
        // clearly right: a node holding a COMPLETE set behind its own declared
        // rate cap, i.e. the Data arm's own scenario with a Sync trigger instead of
        // a Data one.
        //
        // It is per-INPUT, not node-level, and that is what keeps the starved
        // sibling loud: its head is not filled, so it is not suppressed.
        let sync_heads: &IndexMap<String, SyncHead> = &node.sync_heads;
        // hot-path-alloc-ok: not a heap allocation: cloning an `Arc`/`Option<Arc>` is a refcount
        // bump (`qos_events` is an `Option<Arc<QosEventStore>>`)
        let qos = node.qos_events.clone();
        for (input_name, tracker) in node.input_expect_within.iter_mut() {
            let arrival = tracker.last_event_ns.load(Ordering::Acquire);
            if arrival != tracker.last_arrival_ns {
                // The shared anchor changed, and the scheduler NEVER
                // writes it → a real arrival landed → reset the window
                // to the data's own timestamp and rearm the event latch.
                // Unambiguous: no aliasing with a scheduler-authored
                // value (the equal-ts edge of the prior design is gone).
                tracker.last_arrival_ns = arrival;
                tracker.window_start_ns = arrival;
                tracker.armed = true;
            }
            // Only the node's FIFO TRIGGER input can be suppressed
            // by the node-level backlog (see `WatchdogTracker::fifo_trigger`).
            //
            // The Sync arm is per-INPUT rather than node-level — a
            // starved partner must still trip, which is the whole point — so
            // it asks THIS input's head, translated through `sync_topic`.
            let sync_member_held = tracker.sync_topic.as_ref().is_some_and(|topic| {
                sync_heads
                    .get(topic.as_ref())
                    .is_some_and(SyncHead::is_filled)
            });
            let backlogged_now = (data_backlog && tracker.fifo_trigger) || sync_member_held;
            if tracker.prev_backlog && !backlogged_now {
                // DRAIN-OUT EDGE, but ONLY for an episode that actually
                // suppressed a window (see `backlog_suppressed_window`). The
                // anchor then holds the stamp of the frame the node was only
                // just served, which under FIFO is by construction the OLDEST
                // surviving one — so measuring the next window from it counts
                // one spurious miss per drained burst on a producer that never
                // stopped. Re-anchor on the drain instant instead. (A
                // transition step has no arrival: an arrival signals, and a
                // signal makes the backlog non-empty at this very check, which
                // is not a transition.)
                //
                // A node that KEPT UP reaches this edge too — every ordinary
                // "frame delivered, next step silent" pair does — and there the
                // anchor is the fresh stamp of the frame just served, which is
                // already the right answer; re-anchoring it would silently
                // widen that input's threshold by one step for no reason.
                if tracker.backlog_suppressed_window {
                    tracker.window_start_ns = new_time;
                }
                tracker.backlog_suppressed_window = false;
                if let Some(suppressed) = tracker.backlog_latch.on_success() {
                    // The close names the SAME cause its head did.
                    // A shared umbrella noun was tried and is wrong twice over:
                    // an operator cannot pair an open with a close that calls
                    // the regime something else, and it silently broke the
                    // `expect_within_fifo_iox2_test` pin on the Data wording —
                    // which is the wording this arm restores verbatim.
                    if tracker.suppression_was_held_member {
                        tracing::info!(
                            node_id = %node_id,
                            input = %input_name,
                            suppressed_windows = suppressed,
                            total_backlogged = expect_backlogged.load(Ordering::Acquire),
                            "expect_within HELD-MEMBER regime closed — this input's \
                             member has been served; `expect_within_ms` windows are \
                             counted as misses again"
                        );
                    } else {
                        tracing::info!(
                            node_id = %node_id,
                            input = %input_name,
                            suppressed_windows = suppressed,
                            total_backlogged = expect_backlogged.load(Ordering::Acquire),
                            "expect_within backlog regime closed — this input's signalled \
                             arrivals have all been served; `expect_within_ms` \
                             windows are counted as misses again"
                        );
                    }
                    tracker.suppression_was_held_member = false;
                }
            }
            tracker.prev_backlog = backlogged_now;
            let elapsed_ns = new_time.saturating_sub(tracker.window_start_ns);
            if elapsed_ns > tracker.within_ns {
                if backlogged_now {
                    // Report it, never count it, and deliberately do NOT emit
                    // the reactive `ExpectWithinEvent`: an `#[on_event]`
                    // handler that fails over to a backup input on staleness
                    // must not fire while unconsumed frames are queued on this
                    // very input. `armed` is left untouched — a backlog is not
                    // a regime boundary — so the first genuinely-silent window
                    // after the backlog drains still fires exactly one event
                    // (re-arm comes only from a real anchor move).
                    // This episode really did cost the node a window, so the
                    // drain-out edge above has a correction to make.
                    tracker.backlog_suppressed_window = true;
                    // Remember WHICH cause this regime is, so the
                    // close can name what the head named.
                    tracker.suppression_was_held_member = sync_member_held;
                    let total = expect_backlogged.fetch_add(1, Ordering::Release) + 1;
                    //
                    // The two causes get DIFFERENT words and
                    // DIFFERENT fields. `signalled_unserved` is a Data
                    // quantity — a Sync node's `pending_data_count` is
                    // structurally 0 — so emitting it on the Sync arm would
                    // print a positive "nothing is queued" beside a
                    // suppression that happened precisely because something
                    // IS held. Each arm describes only what ITS state did.
                    let decision = tracker.backlog_latch.on_failure();
                    if sync_member_held {
                        match decision {
                            RegimeDecision::Loud => tracing::info!(
                                node_id = %node_id,
                                input = %input_name,
                                expect_within_ms = tracker.within_ns / 1_000_000,
                                elapsed_ms = elapsed_ns / 1_000_000,
                                "expect_within HELD-MEMBER regime opened — this input's \
                                 frame is already the aligned set's member and the set \
                                 is waiting on a SIBLING input, so its windows are \
                                 reported (debug) and NOT counted as missed deadlines \
                                 while this lasts. The starved sibling is the fault; \
                                 look for ITS `expect_within` misses"
                            ),
                            RegimeDecision::StillFailing { total, suppressed } => tracing::info!(
                                node_id = %node_id,
                                input = %input_name,
                                total_failures = total,
                                suppressed = suppressed,
                                "expect_within held-member regime still open"
                            ),
                            RegimeDecision::Suppressed { .. } => {}
                        }
                    } else {
                        match decision {
                            RegimeDecision::Loud => tracing::info!(
                                node_id = %node_id,
                                input = %input_name,
                                expect_within_ms = tracker.within_ns / 1_000_000,
                                elapsed_ms = elapsed_ns / 1_000_000,
                                signalled_unserved = pending_data,
                                "expect_within backlog regime opened — `expect_within_ms` \
                                 windows on this input are elapsing while \
                                 signalled arrivals sit unserved on it; they are \
                                 reported (debug) and NOT counted as missed \
                                 deadlines while this lasts"
                            ),
                            RegimeDecision::StillFailing { total, suppressed } => tracing::info!(
                                node_id = %node_id,
                                input = %input_name,
                                total_failures = total,
                                suppressed = suppressed,
                                signalled_unserved = pending_data,
                                "expect_within backlog regime still open"
                            ),
                            RegimeDecision::Suppressed { .. } => {}
                        }
                    }
                    // EVERY suppressed window is reported, so a suppression
                    // is never silent even when the regime head has scrolled
                    // away; `debug!` because a node behind its own declared
                    // rate cap is behaving as designed, and the running total
                    // stays queryable via
                    // `NodeHandle::expect_within_backlogged_count`
                    // (Principle #3). The wording states only what is
                    // OBSERVED: signalled arrivals this node has not served.
                    // It says nothing about the producer — a head re-offered
                    // by a collapsed tick keeps the count above zero long
                    // after a producer died (see the held-head warn in
                    // `CerulionSubscriber::snapshot_latest_for_trigger`).
                    if sync_member_held {
                        tracing::debug!(
                            node_id = %node_id,
                            input = %input_name,
                            expect_within_ms = tracker.within_ns / 1_000_000,
                            elapsed_ms = elapsed_ns / 1_000_000,
                            total_backlogged = total,
                            "expect_within held-member window — this input's frame is the \
                             aligned set's member and the set is incomplete; the window \
                             elapsed without the node firing"
                        );
                    } else {
                        tracing::debug!(
                            node_id = %node_id,
                            input = %input_name,
                            expect_within_ms = tracker.within_ns / 1_000_000,
                            elapsed_ms = elapsed_ns / 1_000_000,
                            signalled_unserved = pending_data,
                            total_backlogged = total,
                            "expect_within backlog window — signalled arrivals sit unserved \
                             on this input; the window elapsed without the node \
                             serving one"
                        );
                    }
                    // Same one-report-per-window cadence as the miss path
                    // below: without this the branch re-evaluates true on
                    // every step and the per-window diagnostic becomes a
                    // per-step flood.
                    tracker.window_start_ns = new_time;
                    continue;
                }
                let count_total = expect_missed.fetch_add(1, Ordering::Release) + 1;
                tracing::warn!(
                    node_id = %node_id,
                    input = %input_name,
                    expect_within_ms = tracker.within_ns / 1_000_000,
                    elapsed_ms = elapsed_ns / 1_000_000,
                    "input `expect_within_ms` exceeded (no fresh data within the expected \
                     window) — `expect_within_missed` counter incremented"
                );
                if tracker.armed {
                    if let Some(store) = &qos {
                        store.push_expect(ExpectWithinEvent {
                            input_name: Arc::from(input_name.as_str()),
                            expect_within_ms: tracker.within_ns / 1_000_000,
                            elapsed_ms: elapsed_ns / 1_000_000,
                            count_total,
                            missed_at_ns: new_time,
                        });
                    }
                    tracker.armed = false;
                }
                // Advance ONLY the scheduler-local window (one miss per
                // window of silence). The shared anchor is untouched —
                // arrivals are its sole writers — so the next real
                // arrival is always detected by the load-vs-last_arrival
                // check above.
                tracker.window_start_ns = new_time;
            }
        }
        // Symmetric to `expect_within_ms`:
        // check per-output `promise_within_ms` windows.
        // `signal_output_published` (and the publisher's
        // `record_promise_within_published`) update the shared anchor;
        // `step()` bumps the miss counter + warn (every window) and
        // queues an edge-triggered `PromiseWithinEvent` (once per
        // regime) when elapsed > the promised interval.
        let promise_missed = Arc::clone(&node.promise_within_missed);
        for (output_name, tracker) in node.output_promise_within.iter_mut() {
            let arrival = tracker.last_event_ns.load(Ordering::Acquire);
            if arrival != tracker.last_arrival_ns {
                // A real publish landed (arrival-only-written anchor).
                tracker.last_arrival_ns = arrival;
                tracker.window_start_ns = arrival;
                tracker.armed = true;
            }
            let elapsed_ns = new_time.saturating_sub(tracker.window_start_ns);
            if elapsed_ns > tracker.within_ns {
                let count_total = promise_missed.fetch_add(1, Ordering::Release) + 1;
                tracing::warn!(
                    node_id = %node_id,
                    output = %output_name,
                    promise_within_ms = tracker.within_ns / 1_000_000,
                    elapsed_ms = elapsed_ns / 1_000_000,
                    "output `promise_within_ms` exceeded (no publish within the promised \
                     window) — `promise_within_missed` counter incremented"
                );
                if tracker.armed {
                    if let Some(store) = &qos {
                        store.push_promise(PromiseWithinEvent {
                            output_name: Arc::from(output_name.as_str()),
                            promise_within_ms: tracker.within_ns / 1_000_000,
                            elapsed_ms: elapsed_ns / 1_000_000,
                            count_total,
                            missed_at_ns: new_time,
                        });
                    }
                    tracker.armed = false;
                }
                // Scheduler-local window advance only — anchor untouched.
                tracker.window_start_ns = new_time;
            }
        }
    }

    /// DECIDE which level nodes fire this step, recording a
    /// `FireDecision` per firing node WITHOUT yet invoking its callback. The
    /// runtime (`GraphRuntime::step`) calls this per DAG level (after draining
    /// the level's trigger inputs), then performs a fire-gated input snapshot of
    /// the decided nodes' non-trigger inputs, then calls [`Self::tick_decided`]
    /// to actually fire them. Splitting decide from tick lets the snapshot land
    /// AFTER the fire-set is known (so only firing nodes snapshot) but BEFORE
    /// any callback runs (so a frozen latest-value input is observed by the
    /// tick).
    ///
    /// Runs the per-node QoS windows here (via `run_qos_windows`) — they fire
    /// for EVERY level node every step, matching the flat `evaluate_one`
    /// cadence. The decide itself performs all of `decide_node`'s state
    /// mutations (the byte-identity contract: decide-all-then-tick-all within a
    /// level equals fused per-node firing because no same-level node triggers
    /// another — levelization guarantees it).
    ///
    /// A node id not present in the scheduler is a levelization/registration
    /// desync (p1 completeness should make every level node present): it
    /// `debug_assert!(false)` panics in debug and `tracing::error!` + skips in
    /// release — it is NEVER silently dropped. (This is the same loud desync
    /// guard the removed `evaluate_nodes` carried.)
    ///
    /// Fills the reusable `fire_decisions` scratch buffer (cleared,
    /// not reallocated, each call) instead of returning a fresh `Vec` — the
    /// runtime borrows it via [`Self::take_decisions`] and returns it via
    /// [`Self::return_decisions`] so the capacity is reused level over level.
    pub(crate) fn decide_fires(&mut self, node_ids: &[String], new_time: u64) {
        // THIS step's clamp, read once (in `begin_step`) and
        // hoisted here before the `self.nodes` borrows — every node of every
        // level in the step decides under the same value.
        let cap_override = self.catchup_cap_override;
        self.fire_decisions.clear();
        // A TRACE-DRIVEN step never consults a trigger — the
        // recording IS the fire schedule. One never-taken branch on every live
        // and non-replay path (`replay_plan` is `None` there).
        if self.replay_plan.is_some() {
            self.decide_fires_from_plan(node_ids, new_time);
            return;
        }
        for id in node_ids {
            if let Some(idx) = self.nodes.get_index_of(id.as_str()) {
                self.run_qos_windows(idx, new_time);
                // SAFETY: `idx` from `get_index_of` two lines above; `run_qos_windows`
                // does not restructure the map.
                let (_, node) = self.nodes.get_index_mut(idx).unwrap();
                let node_id = Arc::clone(&node.node_id);
                // The `node` borrow of `self.nodes` ends here — `decide_node`
                // returns owned `kind` / `node_id` — so pushing into the
                // disjoint `self.fire_decisions` field compiles under NLL.
                if let Some(kind) = Self::decide_node(&node_id, node, new_time, cap_override) {
                    self.fire_decisions
                        .push(FireDecision { node_id, idx, kind });
                }
            } else {
                // A level node id absent from the
                // scheduler is a levelization/registration desync — every level
                // node was added via `add_node` at build (p1 completeness). Keep
                // the defensive skip, but make it LOUD: fail in debug, log in
                // release. Silently dropping it would make the node never fire.
                debug_assert!(
                    false,
                    "decide_fires: node id {id} not in scheduler — levelization/registration desync"
                );
                tracing::error!(
                    node_id = %id,
                    "level node absent from scheduler — not evaluated"
                );
            }
        }
    }

    /// The TRACE-DRIVEN half of [`Self::decide_fires`] — fill the
    /// decision buffer from the installed plan instead of from the nodes'
    /// triggers.
    ///
    /// The QoS windows run in their OWN pass first, for EVERY level node, so the
    /// per-port `expect_within` / `promise_within` / `tick_within` cadence is
    /// the live one (they are rank-local and re-derivable; skipping them would
    /// make a replayed run's QoS counters differ from the recorded run's for a
    /// reason that has nothing to do with the candidate). The one difference
    /// from the live path is ORDER — all windows, then all decisions, rather
    /// than interleaved per node — which no node can observe: a window's effects
    /// are its own node's counters, and decide-all-then-tick-all already means
    /// no node's tick has run when any other node decides.
    ///
    /// The desync guard is the SAME loud one `decide_fires` carries, and it runs
    /// in the decision pass only (once per absent id, not twice).
    fn decide_fires_from_plan(&mut self, node_ids: &[String], new_time: u64) {
        for id in node_ids {
            if let Some(idx) = self.nodes.get_index_of(id.as_str()) {
                self.run_qos_windows(idx, new_time);
            }
        }
        for id in node_ids {
            let Some(idx) = self.nodes.get_index_of(id.as_str()) else {
                debug_assert!(
                    false,
                    "decide_fires_from_plan: node id {id} not in scheduler — levelization/registration desync"
                );
                tracing::error!(
                    node_id = %id,
                    "level node absent from scheduler — not evaluated"
                );
                continue;
            };
            if let Some(kind) = self.take_planned_fire(idx, id, new_time) {
                let node_id = Arc::clone(&self.nodes[idx].node_id);
                self.fire_decisions
                    .push(FireDecision { node_id, idx, kind });
            }
        }
    }

    /// Lend the filled decision buffer to the runtime (which iterates
    /// it for the fire-gated snapshot, then drains it via
    /// `tick_decided[_parallel]`), then [`Self::return_decisions`] gives the
    /// now-empty buffer back so its capacity is reused next level/step (zero
    /// per-step alloc). `std::mem::take` swaps in an empty `Vec` (no alloc) for
    /// the duration of the borrow.
    pub(crate) fn take_decisions(&mut self) -> Vec<FireDecision> {
        std::mem::take(&mut self.fire_decisions)
    }

    /// Return the (now-empty-but-capacity-retaining) decision buffer
    /// after `tick_decided[_parallel]` has drained it, so the next
    /// `decide_fires` reuses the allocation. Pairs with [`Self::take_decisions`].
    pub(crate) fn return_decisions(&mut self, decisions: Vec<FireDecision>) {
        self.fire_decisions = decisions;
    }

    /// FIRE the nodes a prior [`Self::decide_fires`]
    /// decided, in decision order. Re-fetches each node by its stable insertion
    /// `idx` (the `IndexMap` is not mutated between decide and tick within a
    /// step). The deferred fire (`FireKind`) carries everything `tick_node`
    /// needs; the Period catch-up `pre_fire_check` re-check (the block-overflow
    /// clamp) re-runs inside `tick_node`, exactly as the fused form did.
    ///
    /// DRAINS the caller's reusable `decisions` buffer in place
    /// (`drain(..)`) instead of consuming it by value — after the call the
    /// `Vec` is empty but retains its capacity, so the runtime can hand it back
    /// to the scheduler (`return_decisions`) for reuse next level/step.
    pub(crate) fn tick_decided(
        &mut self,
        decisions: &mut Vec<FireDecision>,
        global_level: usize,
        step: u64,
    ) {
        let max_trace = self.max_trace_entries;
        for d in decisions.drain(..) {
            // Turn a silent wrong-node fetch into a loud failure: if a future
            // `shift_remove` ever ran mid-step, `idx` would point at a different
            // node. The id at `idx` must still equal the one decide minted it for.
            debug_assert_eq!(
                self.nodes.get_index(d.idx).map(|(k, _)| k.as_str()),
                Some(d.node_id.as_ref()),
                "tick_decided: idx/node_id desync — scheduler IndexMap mutated between decide_fires and tick_decided"
            );
            // SAFETY: `idx` was minted by `get_index_of` in `decide_fires` and the
            // IndexMap is not mutated within a step, so the index is valid.
            let (_, node) = self.nodes.get_index_mut(d.idx).unwrap();
            Self::tick_node(
                &d.node_id,
                node,
                d.kind,
                global_level,
                step,
                &mut self.trace,
                max_trace,
                &mut self.entries_appended,
                self.trace_ring.as_mut(),
            );
        }
    }

    /// Evaluate `node_ids` (in the given GRAPH order) with the
    /// decide and tick FUSED per node — each node's `decide_node` →
    /// (fire-gated input snapshot via `snapshot`) → `tick_node` runs to
    /// completion BEFORE the next id's `decide_node`. This is the seam the
    /// level executor uses for the **block-involved** subset only.
    ///
    /// # Why this exists
    ///
    /// The level executor runs DECIDE-all (`decide_fires`) then
    /// TICK-all (`tick_decided_parallel`). For ≥2 same-level in-graph
    /// producers of one ALL-`block` `multi_publisher_topics` topic, that pair
    /// shares ONE `outstanding` mirror. Because every producer's `decide_node`
    /// reads `outstanding` (via its `pre_fire_check`) at the LEVEL BOUNDARY —
    /// before ANY same-level producer publishes (publishes happen in tick) — at
    /// `outstanding == depth-1` BOTH producers decide to fire, BOTH publish,
    /// and `outstanding` overshoots to `depth+1`: iceoryx2 evicts the oldest →
    /// silent DATA LOSS on a topic the user declared lossless. Serializing only
    /// the block-involved TICK does NOT fix it — the over-decide already
    /// happened at the level boundary.
    ///
    /// Fusing decide+tick fixes it: producer1 decides+ticks (PUBLISHES →
    /// `outstanding` bumps) BEFORE producer2's `decide_node` runs, so
    /// producer2's `pre_fire_check` now observes the bumped `outstanding` and
    /// defers correctly. No change to the `outstanding` mechanism
    /// (publisher send-path bump, subscriber drain decrement) — this only
    /// changes the ORDER in which the block subset decides vs ticks.
    ///
    /// Composition: this reuses the existing `run_qos_windows` (so the per-port
    /// QoS cadence is inherited unchanged — every listed node runs its windows
    /// every step, identical to `decide_fires`), `decide_node` (so the Period
    /// catch-up clamp + counter side-effects are inherited unchanged) and
    /// `tick_node`. External publishers are unaffected (they never appear in
    /// `node_ids`). A node id not present in the scheduler is a
    /// levelization/registration desync handled with the SAME loud guard
    /// `decide_fires` carries (`debug_assert!(false)` + `tracing::error!`,
    /// never a silent drop).
    ///
    /// `snapshot` is invoked for a firing block node id AFTER its `decide_node`
    /// returns `Some(_)` and BEFORE its `tick_node`, so a block consumer with a
    /// plain (non-block) snapshotted input still freezes that input's prior
    /// value before its own tick — same fire-gated semantics as the non-block
    /// level path.
    pub(crate) fn evaluate_nodes_fused(
        &mut self,
        node_ids: &[String],
        new_time: u64,
        global_level: usize,
        step: u64,
        snapshot: &mut dyn FnMut(&str),
    ) {
        let max_trace = self.max_trace_entries;
        // THIS step's clamp (see `decide_fires`).
        let cap_override = self.catchup_cap_override;
        for id in node_ids {
            let Some(idx) = self.nodes.get_index_of(id.as_str()) else {
                // Same loud desync guard as `decide_fires`: a level node id
                // absent from the scheduler is a levelization/registration
                // desync — never silently dropped (that would make the node
                // never fire).
                debug_assert!(
                    false,
                    "evaluate_nodes_fused: node id {id} not in scheduler — levelization/registration desync"
                );
                tracing::error!(
                    node_id = %id,
                    "level (fused-block) node absent from scheduler — not evaluated"
                );
                continue;
            };
            // QoS windows run for EVERY listed node every step (cadence
            // independent of firing) — identical to `decide_fires`. Re-fetch the
            // node afterwards: `run_qos_windows` takes `&mut self`, so its node
            // borrow ends before the decide re-fetch.
            self.run_qos_windows(idx, new_time);
            // The block-involved subset is EXACTLY the shape the
            // trace-driven decision exists for (the cross-rank pre-fire
            // argument), so the plan is consulted HERE too — through the same
            // `take_planned_fire` body the level path uses, so the two cannot
            // drift. One never-taken branch when no plan is installed.
            let node_id = Arc::clone(&self.nodes[idx].node_id);
            let decided = if self.replay_plan.is_some() {
                self.take_planned_fire(idx, &node_id, new_time)
            } else {
                // SAFETY: `idx` from `get_index_of` above; `run_qos_windows` does
                // not restructure the map.
                let (_, node) = self.nodes.get_index_mut(idx).unwrap();
                Self::decide_node(&node_id, node, new_time, cap_override)
            };
            if let Some(kind) = decided {
                // Freeze this firing block node's own non-trigger inputs BEFORE
                // its tick (fire-gated snapshot), then tick it — so its PUBLISH
                // lands before the NEXT same-level block producer's
                // `decide_node` reads `outstanding`. This ordering is what keeps it correct.
                snapshot(&node_id);
                // Re-fetch by idx: `snapshot` is an opaque callback that does
                // not touch the scheduler's IndexMap (it locks node entries in
                // the runtime), so `idx` is still valid.
                debug_assert_eq!(
                    self.nodes.get_index(idx).map(|(k, _)| k.as_str()),
                    Some(node_id.as_ref()),
                    "evaluate_nodes_fused: idx/node_id desync — scheduler IndexMap mutated across the snapshot callback"
                );
                let (_, node) = self.nodes.get_index_mut(idx).unwrap();
                Self::tick_node(
                    &node_id,
                    node,
                    kind,
                    global_level,
                    step,
                    &mut self.trace,
                    max_trace,
                    &mut self.entries_appended,
                    self.trace_ring.as_mut(),
                );
            }
        }
    }

    /// Convenience: step by milliseconds.
    pub fn step_ms(&mut self, ms: u64) {
        self.step(Duration::from_millis(ms));
    }

    /// Signal that data has arrived for a `Data` triggered node.
    ///
    /// Adds one pending arrival, **saturating at `MAX_CONSUMER_DEPTH`
    /// (64)** — the deepest input queue that can exist, so signals beyond
    /// what any queue can retain (describing frames already evicted) do not
    /// accrue an unbounded tail of no-op fires. Each `step()` consumes one
    /// pending arrival per fire (per-message FIFO firing); a signalled
    /// backlog within the saturation bound is never lost.
    /// This node's signalled-but-unfired arrival count, read WITHOUT
    /// the `&mut` borrow `signal_data` needs.
    ///
    /// The ONE consumer is `drain_level`'s Unified arm, which uses it to decide
    /// whether a popped frame needs a fresh signal at all — see the mint gate
    /// there. An unknown node reads 0, which is the same answer `signal_data`
    /// would give it a `NodeNotFound` for: the gate's job is to skip a
    /// REDUNDANT mint, and there is nothing redundant about a node the
    /// scheduler has never heard of.
    pub(crate) fn pending_data_count(&self, node_id: &str) -> u64 {
        self.nodes
            .get(node_id)
            .map(|n| n.pending_data_count)
            .unwrap_or(0)
    }

    pub fn signal_data(&mut self, node_id: &str) -> TransportResult<()> {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                // hot-path-alloc-ok: cold: error-struct field on the NodeNotFound arm; the closure
                // runs only when the id is absent (a build/wiring bug), never on a successful
                // signal
                node_id: node_id.to_string(),
            })?;

        match &node.policy {
            TriggerPolicy::Data => {
                // Clamp the pending-arrival carry at the deepest possible
                // input queue (MAX_CONSUMER_DEPTH). Signals can outrun what
                // the body queue retains (frames evicted under drop_oldest
                // between signal and fire), and each phantom signal costs a
                // collapsed no-op fire — the clamp bounds that tail while
                // never under-signaling a retainable frame (the queue holds
                // at most MAX_CONSUMER_DEPTH). Evicted frames are already
                // counted by the drop_oldest probe; the clamp is bookkeeping,
                // not loss.
                node.pending_data_count =
                    (node.pending_data_count + 1).min(DATA_PENDING_CARRY_CLAMP);
                node.pending_data_count_shared
                    .store(node.pending_data_count, Ordering::Release);
                Ok(())
            }
            _ => {
                node.signal_failed.fetch_add(1, Ordering::Release);
                Err(TransportError::SchedulerError {
                    reason: format!(
                        "signal_data() not valid for node '{}' with policy {:?}",
                        node_id, node.policy
                    ),
                })
            }
        }
    }

    /// Notify the scheduler that fresh data
    /// arrived on `input_name` for `node_id` at `timestamp_ns`. Used
    /// purely for the per-input `#[input(expect_within_ms = N)]` tracker —
    /// distinct from `signal_data` which also drives the firing
    /// pipeline. Safe to call on nodes without an input-deadline
    /// configured (no-op).
    pub fn signal_input_received(
        &mut self,
        node_id: &str,
        input_name: &str,
        timestamp_ns: u64,
    ) -> TransportResult<()> {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                // hot-path-alloc-ok: cold: error-struct field on the NodeNotFound arm; the closure
                // runs only when the id is absent (a build/wiring bug), never on a successful
                // signal
                node_id: node_id.to_string(),
            })?;
        if let Some(entry) = node.input_expect_within.get(input_name) {
            entry.last_event_ns.store(timestamp_ns, Ordering::Release);
        }
        Ok(())
    }

    /// Signal that data has arrived on a specific input for a Sync triggered node.
    pub fn signal_sync_input(
        &mut self,
        node_id: &str,
        input_name: &str,
        timestamp_ns: u64,
    ) -> TransportResult<()> {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                // hot-path-alloc-ok: cold: error-struct field on the NodeNotFound arm; the closure
                // runs only when the id is absent (a build/wiring bug), never on a successful
                // signal
                node_id: node_id.to_string(),
            })?;

        // The head MODEL is per-set machinery, and a node with no ops
        // installed is not on the per-set path at all — see the fill-if-empty
        // block below for why the two must not share one rule.
        let per_set = node.sync_ops.is_some();

        match &node.policy {
            TriggerPolicy::Sync { inputs, .. } => {
                if !inputs.iter().any(|i| i == input_name) {
                    node.signal_failed.fetch_add(1, Ordering::Release);
                    return Err(TransportError::SchedulerError {
                        reason: format!(
                            "input '{}' not declared for sync node '{}'",
                            input_name, node_id
                        ),
                    });
                }
                // FILL-IF-EMPTY on the PER-SET path, LATEST-WINS
                // everywhere else — and the split is not a convenience.
                //
                // Per-set: earlier this was a plain `insert`, i.e.
                // latest-wins: a second arrival before a fire silently REPLACED
                // the first stamp, which is precisely how N arrivals collapsed
                // to one fire on the freshest members. A head is now a MEMBER of
                // the set being formed, so it is written once and released by a
                // fire (`Emitted`) or by the align pass's own death discard
                // (`DiscardTie` → `Advance` → refill / `mark_head_spent`).
                //
                // A DIFFERENT stamp offered to a `Filled` head is a counted
                // no-op returning `Ok(())` — NOT an `Err`, and NOT a
                // `signal_failed` bump. A hand-driven embedder signalling
                // every arrival is using the API CORRECTLY; the head simply
                // holds the older frame, which IS the semantic. Reporting it
                // as a failure would make correct use look broken and would
                // bump a counter whose documented meaning is "wiring desync".
                //
                // NO OPS INSTALLED — the DEGRADED transport-backed node
                // (`CERULION_DRAIN_DISCIPLINE=separate`, a raw-FFI cdylib, a
                // `from_names` closure) and the pure-scheduler embedder — keeps
                // the earlier LATEST-WINS insert, for two reasons that are
                // each sufficient on their own:
                //
                //  1. WITHOUT IT THE NODE WEDGES, PERMANENTLY. `align_sync_heads`
                //     returns `Incomplete` immediately when `sync_ops` is `None`,
                //     so nothing on this path can ever execute a `DiscardTie`:
                //     `decide_node` re-derives the verdict and DROPS it (a death
                //     must not fire — the tuple is out of window). A fill-if-empty
                //     head pair that lands out-of-window is therefore never
                //     released — no fire, no align pass, no retraction — and the
                //     node is silent FOREVER, where earlier the next arrival
                //     overwrote the stale stamp and it self-healed. MEASURED: one
                //     out-of-window pair followed by five perfectly aligned pairs
                //     produced ZERO fires.
                //  2. IT IS THE ONLY COHERENT SEMANTIC HERE. The degrade warn
                //     promises this node "fires once per complete alignment and
                //     its tick reads the FRESHEST frame on each trigger". Under
                //     latest-wins the head stamp IS the freshest frame's stamp —
                //     the same frame the body reads — so the window test judges
                //     the frames actually delivered. Fill-if-empty would hold the
                //     OLDEST unfired stamp while the body read the newest, so on
                //     any mismatched-rate node (the headline 200/20 Hz VIO shape)
                //     the window test would be applied to stamps no fire ever
                //     serves, starving the cadence on a healthy graph.
                let filled_head = node
                    .sync_heads
                    .get(input_name)
                    .filter(|head| head.is_filled())
                    .copied();
                match filled_head {
                    Some(head) if per_set => {
                        if head.ts != timestamp_ns {
                            crate::scheduler::handle::SyncCounters::bump(
                                &node.sync_counters.head_refusals,
                                input_name,
                                1,
                            );
                            tracing::debug!(
                                node_id = %node_id,
                                sync_input = %input_name,
                                held_ts = head.ts,
                                offered_ts = timestamp_ns,
                                "sync head already filled; offer counted and ignored \
                                 (the head holds the older frame — that is the per-set semantic)"
                            );
                        }
                    }
                    _ => {
                        node.sync_heads
                            // hot-path-alloc-known: one `String` per SYNC-input signal, carried
                            // across from the `sync_input_timestamps` insert this replaced.
                            // The map key is the input NAME, which never changes after the first
                            // arrival, so `get_mut`-then-insert-on-miss would make the steady state
                            // allocation-free; it is a per-signal allocation today. Recorded rather
                            // than claimed cold
                            .insert(input_name.to_string(), SyncHead::filled(timestamp_ns));
                    }
                }
                Ok(())
            }
            _ => {
                node.signal_failed.fetch_add(1, Ordering::Release);
                Err(TransportError::SchedulerError {
                    reason: format!(
                        "signal_sync_input() not valid for node '{}' with policy {:?}",
                        node_id, node.policy
                    ),
                })
            }
        }
    }

    /// Install a Sync node's TRANSPORT SEAM — the ops the matcher
    /// demands, and the declaration-order input list they are indexed by.
    ///
    /// Called once per Sync node at graph build. A node with no ops installed
    /// keeps the classic hand-driven semantics (completeness + window death,
    /// descent disabled), which is what leaves every pure-scheduler embedder
    /// and every `scheduler_test.rs` Sync arm on their existing contract.
    ///
    /// REFUSES a list that disagrees with the node's declared trigger set,
    /// because every index in the matcher's verdicts — the gate scan, the
    /// tie-break, the whole `next_info` scratch — is a DECLARATION POSITION. Two
    /// lists silently disagreeing would make a verdict describe one input while
    /// the op moved another: a wrong-set bug with no loud failure anywhere,
    /// which is the class this repo refuses to ship.
    pub(crate) fn set_sync_ops<F>(
        &mut self,
        node_id: &str,
        inputs: &[String],
        op: F,
    ) -> TransportResult<()>
    where
        F: Fn(&str, SyncHeadOp) -> SyncOpAnswer + Send + Sync + 'static,
    {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                // hot-path-alloc-ok: error path — the node is missing, which is a
                // wiring bug surfaced once at registration, never a steady state
                node_id: node_id.to_string(),
            })?;
        let TriggerPolicy::Sync {
            inputs: declared, ..
        } = &node.policy
        else {
            return Err(TransportError::SchedulerError {
                reason: format!(
                    "set_sync_ops() not valid for node '{node_id}' with policy {:?} — \
                     only a Sync node has heads to align",
                    node.policy
                ),
            });
        };
        if declared.as_slice() != inputs {
            return Err(TransportError::SchedulerError {
                reason: format!(
                    "set_sync_ops() input list disagrees with node '{node_id}''s declared \
                     trigger set (declared {declared:?}, installed {inputs:?}) — the matcher \
                     indexes every verdict by DECLARATION POSITION, so two lists would make a \
                     verdict describe one input while the op moved another"
                ),
            });
        }
        node.sync_ops = Some(SyncOps {
            // hot-path-alloc-ok: cold: ONCE per Sync node at graph BUILD. The
            // resulting `Arc<str>` slice is what every later align pass indexes,
            // which is precisely why it is interned here and not per pass
            inputs: inputs.iter().map(|i| Arc::from(i.as_str())).collect(),
            op: Arc::new(op),
        });
        Ok(())
    }

    /// Run ONE alignment pass over a Sync node's heads, driving the
    /// matcher's verdicts against transport.
    ///
    /// Called at two sites, DECLARED not inferred: once per level boundary
    /// (before decide), and again between two fires of one step's burst. The
    /// site chooses the fill op, and the difference is load-bearing — a
    /// boundary RE-OFFERS a head the tick never read (Principle #6), while at a
    /// refill that same state means the fire that just ran did NOT consume the
    /// head, so re-offering it would fire the node again on one frame.
    ///
    /// Returns `Incomplete` for a node the caller should not fire. A node with
    /// no ops installed (a pure-scheduler embedder) always reports `Incomplete`
    /// here and is fired by `decide_node` alone — which is what keeps the
    /// hand-driven semantics at exactly one fire per complete set per step.
    pub(crate) fn align_sync_heads(
        &mut self,
        node_id: &str,
        site: SyncAlignSite,
    ) -> TransportResult<AlignOutcome> {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                // hot-path-alloc-ok: error path — a miss here is a caller bug, and
                // the align pass proper takes the `&mut ScheduledNode` overload
                node_id: node_id.to_string(),
            })?;
        Ok(Self::align_node_sync_heads(node_id, node, site))
    }

    /// The align pass over a node the caller already holds.
    ///
    /// Takes the three scratch buffers out with `mem::take` for the duration
    /// and writes them back: it keeps a steady-state boundary allocation-free
    /// (the zero-alloc scratch pattern) AND it is what lets the loop hold `&mut node`
    /// while reading its own scratch, which a borrow of `node.sync_scratch_*`
    /// through the same reference would otherwise fight.
    /// Write `ts` into this input's head, WITHOUT allocating a key when the entry
    /// already exists.
    ///
    /// The three align-pass fill sites used to `insert(input.to_string(), ..)`,
    /// which allocates a `String` on every write even when the key is present —
    /// a per-alignment allocation on the hot path, and one the widened allocation
    /// lint correctly flags. It is avoidable rather than merely recordable: the
    /// key is the input NAME, and `mark_head_spent` leaves an `Emitted` TOMBSTONE
    /// instead of removing the entry, so after an input's FIRST fill the key is
    /// present for the life of the node. `get_mut` therefore hits on every write
    /// but the first.
    #[inline]
    fn set_sync_head(
        node: &mut ScheduledNode,
        input: &str,
        ts: u64,
        window_ns: Option<u64>,
    ) -> HeadFill {
        // Is this fresh pop evidence that the publisher's clock
        // RESTARTED? Within one run a single writer's stamps are monotonically
        // non-decreasing, so a stamp far below this input's own high-water
        // cannot be a late arrival — it is a new epoch (a rebooted robot behind
        // a netd mirror re-injecting origin headers verbatim, a restarted worker
        // whose `VirtualClock` begins again at 0, a rebooted `ros2 attach`
        // bridge). This is the `max_stamp_ns` rule read on the
        // scheduler's side of the same evidence.
        //
        // THE TOLERANCE IS THE ALIGNMENT WINDOW, and it is not arbitrary. A
        // regression of at most `W` is exactly the stamp spread the matcher is
        // built to absorb, and — the load-bearing half — a head that sits at
        // most `W` above its partners can still be SERVED, so it can never
        // become the immortal maximum this reset exists to evict. Only a
        // regression LARGER than the window can wedge, so only that one is
        // called an epoch. An unbounded-window node needs no reset at all: with
        // no death check its tuple always fires on completeness, so no head can
        // be stranded above a partner.
        let regressed = match (node.sync_stamp_high_water.get(input), window_ns) {
            (Some(&high_water), Some(window_ns)) => ts.saturating_add(window_ns) < high_water,
            _ => false,
        };
        // A RE-BASED input (its high-water was removed by a reset) is still
        // draining the epoch that ended. Anything above the band is one of its
        // stragglers — the frame that would otherwise become the next immortal
        // maximum.
        if !regressed && node.sync_stamp_high_water.get(input).is_none() {
            if let (Some(floor), Some(window_ns)) = (node.sync_epoch_floor, window_ns) {
                if ts > floor.saturating_add(window_ns) {
                    return HeadFill::StaleEpoch;
                }
            }
        }
        if let Some(floor) = node.sync_epoch_floor.as_mut() {
            // The band tracks the newest stamp of the CURRENT epoch, so a
            // re-based input's stragglers stay outside it while real traffic
            // keeps it moving. A regression RE-SEATS it rather than raising it.
            *floor = if regressed { ts } else { (*floor).max(ts) };
        }
        match node.sync_stamp_high_water.get_mut(input) {
            Some(high_water) => {
                // A REGRESSION re-bases this input outright: the old epoch's
                // high-water is meaningless now, and keeping it would make every
                // subsequent pop look like a fresh restart (measured — the node
                // reset on every boundary and never filled a head).
                *high_water = if regressed { ts } else { (*high_water).max(ts) };
            }
            None => {
                node.sync_stamp_high_water
                    // hot-path-alloc-ok: cold: ONE `String` per input per node, at
                    // its FIRST pop only — `get_mut` takes every later write
                    .insert(input.to_string(), ts);
            }
        }
        // The frame is GOOD either way — a regression is evidence about the
        // CLOCK, not about this frame, which is the first of the new epoch and
        // is exactly what the node should align on next. So it is filled here
        // and the verdict only tells the caller to re-base the OTHER inputs.
        let verdict = if regressed {
            HeadFill::EpochReset
        } else {
            HeadFill::Filled
        };
        if let Some(head) = node.sync_heads.get_mut(input) {
            *head = SyncHead::filled(ts);
            return verdict;
        }
        node.sync_heads
            // hot-path-alloc-ok: cold: ONE `String` per input per node, at its FIRST
            // fill only — the `get_mut` above takes every subsequent write, because
            // a spent head is tombstoned rather than removed
            .insert(input.to_string(), SyncHead::filled(ts));
        verdict
    }

    /// Discard one old-epoch straggler — a frame from the clock epoch
    /// that ended, popped after the reset from a queue that still held it.
    fn discard_stale_epoch_head(node: &mut ScheduledNode, ops: &SyncOps, input: &Arc<str>) {
        (ops.op)(input, SyncHeadOp::Void);
        crate::scheduler::handle::SyncCounters::bump(
            &node.sync_counters.epoch_reset_discards,
            input,
            1,
        );
        Self::mark_head_spent(node, input.as_ref());
    }

    /// A publisher clock restarted — re-base this node onto the new
    /// epoch by DISCARDING every other input's held head.
    ///
    /// # Why the matcher cannot do this itself
    ///
    /// A held head has exactly three exits, and every one of them is blocked
    /// here. Serving needs `span <= W`, which a stale head by definition
    /// prevents; `Advance` targets the ARGMIN; and `DiscardTie` discards the
    /// LO tie-set. All three evict from the BOTTOM, so a head that is the
    /// tuple's MAXIMUM and more than a window above every future partner frame
    /// can never leave — and after a clock restart the OTHER input's retained
    /// old-epoch head is exactly that. Left alone the node never fires again:
    /// every new-epoch frame is popped and counted UNMATCHABLE on the HEALTHY
    /// input, the stalled input's own `expect_within` watchdog is suppressed
    /// because its member is "held", and under `block` its producer defers
    /// forever. The earlier latest-wins path self-healed on this exact
    /// stimulus, so this is a REGRESSION against legacy, not merely a gap.
    ///
    /// Evicting from the top is therefore not something the matcher can decide:
    /// nothing in `(heads, window)` distinguishes "a partner is ahead" from "a
    /// partner is in a dead epoch". Only the popped STREAM carries that, so the
    /// align driver acts on it and the matcher stays a pure function.
    ///
    /// # Why voiding is always safe
    ///
    /// `Void` discards the held frame and leaves the slot empty, so the next
    /// boundary drains normally. It can therefore only ever COST frames, never
    /// fabricate a set — the rule "a complete in-window arrived set is never
    /// destroyed" is preserved in the only sense available, because a set
    /// spanning two clock epochs was never in-window to begin with. Every
    /// discarded head is counted per input, so the cost is observable
    /// (Principle #3) rather than silent.
    fn apply_sync_epoch_reset(
        id: &str,
        node: &mut ScheduledNode,
        ops: &SyncOps,
        trigger_input: &str,
        new_ts: u64,
    ) {
        let mut discarded = 0u64;
        for input in ops.inputs.iter() {
            if input.as_ref() == trigger_input {
                continue;
            }
            let filled = node
                .sync_heads
                .get(input.as_ref())
                .is_some_and(SyncHead::is_filled);
            if !filled {
                continue;
            }
            (ops.op)(input, SyncHeadOp::Void);
            crate::scheduler::handle::SyncCounters::bump(
                &node.sync_counters.epoch_reset_discards,
                input,
                1,
            );
            Self::mark_head_spent(node, input.as_ref());
            discarded += 1;
        }
        // Re-base EVERY other input, filled head or not: its queue may still
        // hold old-epoch frames that have not been popped yet, and those are
        // exactly what would become the next immortal maximum. Removing the
        // high-water marks the input as awaiting re-base; the band below is
        // what its next fills are tested against until one lands inside it.
        for input in ops.inputs.iter() {
            if input.as_ref() != trigger_input {
                node.sync_stamp_high_water.shift_remove(input.as_ref());
            }
        }
        node.sync_epoch_floor = Some(new_ts);
        match node.sync_epoch_latch.on_failure() {
            RegimeDecision::Loud => tracing::warn!(
                node_id = %id,
                sync_input = %trigger_input,
                new_stamp_ns = new_ts,
                discarded_heads = discarded,
                "sync wire-stamp EPOCH RESET — this input's stamps jumped BACKWARD by \
                 more than the alignment window, which within one run only a \
                 publisher clock RESTART can do (a rebooted robot, a restarted \
                 worker, a re-attached bridge). The other inputs' held frames belong \
                 to the epoch that just ended and can never align with anything \
                 again, so they are discarded (counted per input as \
                 `sync_epoch_reset_discards`) and the node re-bases onto the new \
                 clock. Nothing is wrong with your producers and there is nothing \
                 to fix; without this the node would never fire again"
            ),
            RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
                node_id = %id,
                sync_input = %trigger_input,
                total_failures = total,
                suppressed,
                "sync wire-stamp epoch reset still recurring"
            ),
            RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                node_id = %id,
                sync_input = %trigger_input,
                suppressed,
                "sync wire-stamp epoch reset (suppressed)"
            ),
        }
    }

    fn align_node_sync_heads(
        id: &str,
        node: &mut ScheduledNode,
        site: SyncAlignSite,
    ) -> AlignOutcome {
        let TriggerPolicy::Sync { window, .. } = &node.policy else {
            // Not a Sync node: nothing to align. Never an error — the runtime
            // walks its Sync bindings, so reaching here at all is a desync, and
            // a diagnostic path must not abort the loop it observes.
            debug_assert!(
                false,
                "align_sync_heads on a non-Sync node — sync-binding/policy wiring desync"
            );
            return AlignOutcome::Incomplete;
        };
        let window = *window;
        // Cheap: two `Arc` clones. Cloning releases the borrow on `node.policy`
        // so the loop below can mutate the head map and the counters.
        // hot-path-alloc-ok: not a heap allocation: `SyncOps` is two `Arc`s, so the
        // clone is a pair of refcount bumps (the `qos_events.clone()` precedent)
        let Some(ops) = node.sync_ops.clone() else {
            return AlignOutcome::Incomplete;
        };

        let mut heads = std::mem::take(&mut node.sync_scratch_heads);
        let mut next = std::mem::take(&mut node.sync_scratch_next);
        let mut advances = std::mem::take(&mut node.sync_scratch_advances);

        let outcome = Self::align_sync_pass(
            id,
            node,
            site,
            &ops,
            window,
            &mut heads,
            &mut next,
            &mut advances,
        );

        heads.clear();
        next.clear();
        advances.clear();
        node.sync_scratch_heads = heads;
        node.sync_scratch_next = next;
        node.sync_scratch_advances = advances;

        // The FIRST-FIRE DEFER, and why the hint is raised HERE
        // rather than only in the burst.
        //
        // `decide_node`'s `pre_fire_check` gate sits ABOVE the policy match, so
        // a deferred node (a `throttle_ms` cap, a `block` consumer at its
        // threshold) never reaches the Sync arm and never reaches the burst
        // either. A hint set only by the burst would therefore be FALSE for a
        // set the BOUNDARY aligned and a gate deferred before fire 1 — the node
        // owes a fire, reports nothing due, and waits for an unrelated publish
        // or the 250 ms liveliness cap, where a Data node recovers at the 1 ms
        // floor.
        //
        // Raising it at a COMPLETE boundary alignment covers that: on the
        // ordinary path `decide_node`'s Sync arm clears it a moment later and
        // the burst re-raises it only if a set really is left over, so the
        // steady state is unchanged; on the deferred path the Sync arm never
        // runs, and the raised hint is exactly the truth.
        //
        // Refill-site alignments are the burst's own business and do not raise
        // it here — `tick_sync_burst` owns that decision, because only it knows
        // whether the set it just aligned was then fired.
        if site == SyncAlignSite::Boundary && outcome == AlignOutcome::Complete {
            node.sync_backlog_hint = true;
            node.sync_counters
                .backlog_pending
                .store(true, Ordering::Release);
        }
        // The RECOVERY half of the op-failure latch. A pass that ran
        // to its end with every op answering is the healthy transition the
        // shared machine's contract is written against — without it the
        // "once per regime" head is once per PROCESS, the recovery `info!` the
        // doc promises can never be emitted, and a node that fails, heals, and
        // fails again reports the second regime at `debug!` where nothing sees
        // it. Same shape every other consumer of the latch uses.
        if !node.sync_op_failed_this_pass {
            if let Some(suppressed) = node.sync_op_failure_latch.on_success() {
                tracing::info!(
                    node_id = %id,
                    suppressed_failures = suppressed,
                    "sync alignment ops RECOVERED — every op answered this pass; \
                     descent is available again on this node"
                );
            }
        }
        outcome
    }

    /// The align pass proper. Split out so the scratch buffers can
    /// be owned by the caller for the duration.
    #[allow(clippy::too_many_arguments)]
    fn align_sync_pass(
        id: &str,
        node: &mut ScheduledNode,
        site: SyncAlignSite,
        ops: &SyncOps,
        window: Option<Duration>,
        heads: &mut Vec<Option<SyncHead>>,
        next: &mut Vec<NextInfo>,
        advances: &mut Vec<u32>,
    ) -> AlignOutcome {
        let n = ops.inputs.len();
        next.clear();
        next.resize(n, NextInfo::Unknown);
        advances.clear();
        advances.resize(n, 0);

        // PASS 1 — FILL. Every input whose head is not `Filled` (served,
        // skipped, or never delivered) asks transport for one. An input whose
        // head IS filled is left alone: it is already this set's member.
        node.sync_op_failed_this_pass = false;

        // The epoch-reset tolerance, derived from the node's OWN
        // alignment window (see `set_sync_head`). `None` for an unbounded-window
        // node, which cannot strand a head above its partners and so needs no
        // reset at all.
        let epoch_window_ns = window.map(|w| w.as_nanos() as u64);

        let fill_op = match site {
            SyncAlignSite::Boundary => SyncHeadOp::FillBoundary,
            SyncAlignSite::Refill => SyncHeadOp::FillRefill,
        };
        for input in &ops.inputs {
            let already = node
                .sync_heads
                .get(input.as_ref())
                .is_some_and(SyncHead::is_filled);
            if already {
                continue;
            }
            match (ops.op)(input, fill_op) {
                SyncOpAnswer::Head(ts) => {
                    match Self::set_sync_head(node, input, ts, epoch_window_ns) {
                        HeadFill::Filled => {}
                        HeadFill::StaleEpoch => {
                            // A frame from the epoch that ended, popped from a queue
                            // that still held it. Discard it and leave the head empty:
                            // the tuple is incomplete this pass and the next boundary
                            // pops the frame behind it.
                            Self::discard_stale_epoch_head(node, ops, input);
                        }
                        HeadFill::EpochReset => {
                            Self::apply_sync_epoch_reset(id, node, ops, input, ts);
                            return AlignOutcome::Incomplete;
                        }
                    }
                }
                SyncOpAnswer::Nothing => {
                    Self::mark_head_spent(node, input.as_ref());
                }
                answer => {
                    // A fill is a MUTATING op (it pops), so a failure's side
                    // effects are unknown: TERMINATE the pass. The tuple is not
                    // complete here, so the correct answer is Wait — never a
                    // fire on a set that cannot be proven.
                    debug_assert!(
                        answer == SyncOpAnswer::Failed,
                        "a fill op answered {answer:?}, which only a probe or a peek can \
                         produce — sync op wiring desync"
                    );
                    Self::report_sync_op_failure(id, node, input, "fill");
                    Self::mark_head_spent(node, input.as_ref());
                    return AlignOutcome::Incomplete;
                }
            }
        }

        // PASS 2 — the verdict loop.
        //
        // The DESIGN bound is the per-input advance clamp below. This total is
        // the belt-and-braces bound on a DESYNC (an op that reports success
        // while changing nothing would otherwise spin): six ops per advance is
        // generous against the three the protocol issues.
        let op_budget = n
            .saturating_mul(DATA_PENDING_CARRY_CLAMP as usize)
            .saturating_mul(6)
            .saturating_add(16);
        let mut ops_run = 0usize;

        loop {
            // COUNTED AT THE TOP, deliberately. At the bottom this check is
            // bypassed by any `continue` in a match arm below — and a driver
            // that retries a failing op instead of terminating is exactly the
            // shape this bound exists to catch, so a guard it can skip is no
            // guard at all. (Measured: with the check at the bottom, a variant
            // that `continue`d on a failed mutating op HUNG the suite instead
            // of failing it.)
            ops_run += 1;
            if ops_run > op_budget {
                tracing::error!(
                    node_id = %id,
                    ops_run,
                    op_budget,
                    "sync alignment exceeded its op budget — an op is reporting success \
                     without making progress (wiring desync); abandoning this boundary's \
                     alignment rather than spinning"
                );
                return AlignOutcome::Incomplete;
            }

            heads.clear();
            for input in &ops.inputs {
                heads.push(node.sync_heads.get(input.as_ref()).copied());
            }

            match next_sync_step(heads, next, window) {
                SyncStep::Wait => return AlignOutcome::Incomplete,
                SyncStep::Fire => {
                    // A RESTORED head names a stamp but no live frame. The fire
                    // is REAL in the trace, so its read must collapse to "no
                    // frame" IN PLACE rather than draining a live queued frame
                    // that belongs to the NEXT set.
                    for (i, head) in heads.iter().enumerate() {
                        if head.is_some_and(|h| h.is_unbacked()) {
                            let answer = (ops.op)(&ops.inputs[i], SyncHeadOp::Void);
                            if answer == SyncOpAnswer::Failed {
                                Self::report_sync_op_failure(id, node, &ops.inputs[i], "void");
                            }
                        }
                    }
                    return AlignOutcome::Complete;
                }
                SyncStep::NeedNext(i, probe_site) => {
                    let answer = (ops.op)(&ops.inputs[i], SyncHeadOp::ProbeNext);
                    next[i] = match answer {
                        SyncOpAnswer::Present => NextInfo::Present,
                        SyncOpAnswer::Nothing => NextInfo::None,
                        _ => {
                            debug_assert!(
                                answer == SyncOpAnswer::Failed,
                                "a probe answered {answer:?} — sync op wiring desync"
                            );
                            Self::report_sync_op_failure(id, node, &ops.inputs[i], "probe");
                            // R-FAIL, POSITION-AWARE: a failure resolves to the
                            // DESCENT-DISABLING answer for its site, so a probe
                            // failure can never be the evidence that enables a
                            // descent into an unprobed backlog.
                            match probe_site {
                                ProbeSite::Argmin => NextInfo::None,
                                ProbeSite::Gate => NextInfo::Present,
                            }
                        }
                    };
                }
                SyncStep::NeedStamp(i) => {
                    let answer = (ops.op)(&ops.inputs[i], SyncHeadOp::PeekNext);
                    match answer {
                        SyncOpAnswer::Stamp(ts) => next[i] = NextInfo::Stamp(ts),
                        SyncOpAnswer::Nothing => next[i] = NextInfo::None,
                        _ => {
                            debug_assert!(
                                answer == SyncOpAnswer::Failed,
                                "a peek answered {answer:?} — sync op wiring desync"
                            );
                            Self::report_sync_op_failure(id, node, &ops.inputs[i], "peek");
                            // MUTATING op: terminate. Death already ran, so the
                            // current tuple is complete AND in-window — firing
                            // it is sound greedy membership.
                            return AlignOutcome::Complete;
                        }
                    }
                }
                SyncStep::Advance(i) => {
                    if advances[i] >= DATA_PENDING_CARRY_CLAMP as u32 {
                        // Clamp exhausted. Stop descending and fire the current
                        // tuple: complete and in-window, so the fallback is
                        // sound greedy membership — nothing carries falsely and
                        // the walk resumes from fresher heads next boundary.
                        tracing::debug!(
                            node_id = %id,
                            sync_input = %ops.inputs[i],
                            clamp = DATA_PENDING_CARRY_CLAMP,
                            "sync descent hit its per-input advance clamp; firing the \
                             current in-window tuple greedily"
                        );
                        return AlignOutcome::Complete;
                    }
                    let answer = (ops.op)(&ops.inputs[i], SyncHeadOp::Advance);
                    match answer {
                        SyncOpAnswer::Head(_) | SyncOpAnswer::Nothing => {
                            advances[i] += 1;
                            // PASSED-OVER: a nearer arrived member of the same
                            // stream was chosen. Counter + `debug!` only — a
                            // loud head here runs at ~180 lines/s on a healthy
                            // 200/20 Hz node (the disk-fill class).
                            let total = crate::scheduler::handle::SyncCounters::bump(
                                &node.sync_counters.closer_skips,
                                &ops.inputs[i],
                                1,
                            );
                            tracing::debug!(
                                node_id = %id,
                                sync_input = %ops.inputs[i],
                                total_passed_over = total,
                                "sync alignment passed over a frame for a nearer arrived member"
                            );
                            match answer {
                                SyncOpAnswer::Head(ts) => {
                                    // A refill can itself be the frame that crosses the
                                    // epoch boundary — on the HEALTHY input, whose frames
                                    // this very loop has been discarding.
                                    // hot-path-alloc-ok: not a heap allocation: an `Arc<str>` clone is a refcount bump
                                    let input = ops.inputs[i].clone();
                                    match Self::set_sync_head(node, &input, ts, epoch_window_ns) {
                                        HeadFill::Filled => {}
                                        HeadFill::StaleEpoch => {
                                            Self::discard_stale_epoch_head(node, ops, &input);
                                        }
                                        HeadFill::EpochReset => {
                                            Self::apply_sync_epoch_reset(id, node, ops, &input, ts);
                                            return AlignOutcome::Incomplete;
                                        }
                                    }
                                }
                                _ => {
                                    Self::mark_head_spent(node, ops.inputs[i].as_ref());
                                }
                            }
                            // The staged frame just became the head; whether
                            // ANOTHER sits behind it is genuinely unknown, so
                            // this ONE input is re-probed. That is a per-advance
                            // re-probe, not a mid-walk re-probe of the world.
                            next[i] = NextInfo::Unknown;
                        }
                        _ => {
                            debug_assert!(
                                answer == SyncOpAnswer::Failed,
                                "an advance answered {answer:?} — sync op wiring desync"
                            );
                            Self::report_sync_op_failure(id, node, &ops.inputs[i], "advance");
                            return AlignOutcome::Complete;
                        }
                    }
                }
                SyncStep::DiscardTie(indices) => {
                    let lo = heads.iter().flatten().map(|h| h.ts).min().unwrap_or(0);
                    let hi = heads.iter().flatten().map(|h| h.ts).max().unwrap_or(0);
                    for i in indices {
                        let head_ts = heads[i].map_or(0, |h| h.ts);
                        let answer = (ops.op)(&ops.inputs[i], SyncHeadOp::Advance);
                        match answer {
                            SyncOpAnswer::Head(_) | SyncOpAnswer::Nothing => {
                                Self::report_sync_unmatched(
                                    id,
                                    node,
                                    &ops.inputs[i],
                                    head_ts,
                                    lo,
                                    hi,
                                    window,
                                );
                                match answer {
                                    SyncOpAnswer::Head(ts) => {
                                        // A refill can itself be the frame that crosses the
                                        // epoch boundary — on the HEALTHY input, whose frames
                                        // this very loop has been discarding.
                                        // hot-path-alloc-ok: not a heap allocation: an `Arc<str>` clone is a refcount bump
                                        let input = ops.inputs[i].clone();
                                        match Self::set_sync_head(node, &input, ts, epoch_window_ns)
                                        {
                                            HeadFill::Filled => {}
                                            HeadFill::StaleEpoch => {
                                                Self::discard_stale_epoch_head(node, ops, &input);
                                            }
                                            HeadFill::EpochReset => {
                                                Self::apply_sync_epoch_reset(
                                                    id, node, ops, &input, ts,
                                                );
                                                return AlignOutcome::Incomplete;
                                            }
                                        }
                                    }
                                    _ => {
                                        Self::mark_head_spent(node, ops.inputs[i].as_ref());
                                    }
                                }
                                next[i] = NextInfo::Unknown;
                            }
                            _ => {
                                debug_assert!(
                                    answer == SyncOpAnswer::Failed,
                                    "a discard answered {answer:?} — sync op wiring desync"
                                );
                                Self::report_sync_op_failure(id, node, &ops.inputs[i], "discard");
                                // The tuple is OUT of window; firing it would
                                // violate the window the user declared, so this
                                // one waits and is retried next boundary.
                                return AlignOutcome::Incomplete;
                            }
                        }
                    }
                }
            }
        }
    }

    /// This input's head is SPENT and nothing refilled it.
    ///
    /// Leaves an `Emitted` TOMBSTONE rather than removing the entry. Both read
    /// as "not Filled" for completeness, so the fire decision is identical —
    /// but only the tombstone keeps the input's key in the map, which is what
    /// the capture rule has to filter (an exported tombstone restores as an
    /// unbacked `Filled` head and mints a PHANTOM set) and what keeps the head
    /// map a stable walk over the declared trigger set. An entry that was never
    /// filled has no stamp to tombstone and simply stays absent.
    fn mark_head_spent(node: &mut ScheduledNode, input: &str) {
        if let Some(head) = node.sync_heads.get_mut(input) {
            head.state = sync_match::HeadState::Emitted;
        }
    }

    /// Report a FAILED align op, flood-latched.
    ///
    /// Loud first-of-regime, `debug!` repeats carrying the running suppressed
    /// count, a loud re-announcement at each DECADE of the running total, and
    /// one recovery `info!` — the repo's ONE shared policy
    /// ([`FailureRegimeLatch`]), because a permanently poisoned op fails on
    /// EVERY boundary at the node's data rate.
    fn report_sync_op_failure(id: &str, node: &mut ScheduledNode, input: &Arc<str>, op: &str) {
        node.sync_op_failed_this_pass = true;
        match node.sync_op_failure_latch.on_failure() {
            RegimeDecision::Loud => tracing::error!(
                node_id = %id,
                sync_input = %input,
                op = %op,
                "sync alignment op FAILED; this boundary falls back to greedy-or-wait \
                 membership (never a wrong set). Descent stays disabled on this node \
                 while the failure persists"
            ),
            RegimeDecision::StillFailing { total, suppressed } => tracing::error!(
                node_id = %id,
                sync_input = %input,
                op = %op,
                total_failures = total,
                suppressed,
                "sync alignment op still FAILING"
            ),
            RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                node_id = %id,
                sync_input = %input,
                op = %op,
                suppressed,
                "sync alignment op failed (suppressed)"
            ),
        }
    }

    /// Report an UNMATCHABLE discard — a frame provably in NO set.
    ///
    /// LOUD (flood-latched), unlike its PASSED-OVER sibling, because this one
    /// means something is wrong and the operator has three things to check. The
    /// `depth` remedy is named in the head for a reason that is easy to miss:
    /// when the fast:slow rate ratio exceeds the fast input's declared `depth`,
    /// `drop_oldest` evicts the intervening frames UPSTREAM of the matcher, so
    /// the fast head lands NEWER than the slow head and the slow frame dies on
    /// a perfectly healthy graph.
    fn report_sync_unmatched(
        id: &str,
        node: &mut ScheduledNode,
        input: &Arc<str>,
        head_ts: u64,
        lo: u64,
        hi: u64,
        window: Option<Duration>,
    ) {
        crate::scheduler::handle::SyncCounters::bump(
            &node.sync_counters.unmatched_discards,
            input,
            1,
        );
        let window_ns = window.map_or(0, |w| w.as_nanos() as u64);
        let latch = node
            .sync_unmatched_latches
            .entry(Arc::clone(input))
            .or_default();
        match latch.on_failure() {
            RegimeDecision::Loud => tracing::warn!(
                node_id = %id,
                topic = %input,
                head_timestamp_ns = head_ts,
                set_min_ns = lo,
                set_max_ns = hi,
                window_ns,
                "sync discarded an UNMATCHABLE frame: a partner ran past it by more than \
                 the window. Widen `sync_window_ms`, fix a stalled or skewed producer, or \
                 raise the fast input's `depth` above the fast:slow rate ratio (below it, \
                 `drop_oldest` evicts the frames the matcher needed before it sees them)"
            ),
            RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
                node_id = %id,
                topic = %input,
                total_failures = total,
                suppressed,
                window_ns,
                "sync is still discarding UNMATCHABLE frames"
            ),
            RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                node_id = %id,
                topic = %input,
                head_timestamp_ns = head_ts,
                suppressed,
                "sync discarded an unmatchable frame (suppressed)"
            ),
        }
    }

    /// Trigger an External node to fire on the next step.
    pub fn trigger_external(&mut self, node_id: &str) -> TransportResult<()> {
        let node = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| TransportError::NodeNotFound {
                // hot-path-alloc-ok: cold: error-struct field on the NodeNotFound arm; the closure
                // runs only when the id is absent (a build/wiring bug), never on a successful
                // signal
                node_id: node_id.to_string(),
            })?;

        match &node.policy {
            TriggerPolicy::External => {
                node.external_triggered = true;
                Ok(())
            }
            _ => Err(TransportError::SchedulerError {
                reason: format!(
                    "trigger_external() not valid for node '{}' with policy {:?}",
                    node_id, node.policy
                ),
            }),
        }
    }

    /// Returns the execution trace (for replay verification).
    pub fn trace(&mut self) -> &[TraceEntry] {
        self.trace.make_contiguous()
    }

    // NOTE: the `trace_len()` O(1) length
    // accessor was DELETED here — its only caller was `live_step`'s
    // before/after fire detection, which now differences the cap-immune
    // `entries_appended()` below (a capped-FULL ring pins `trace.len()` at the
    // cap, so a length delta read 0 forever once the production
    // `PRODUCTION_TRACE_LIMIT` ring filled — silently collapsing the
    // spin budget). Dead-code policy: zero callers ⇒ delete, not keep.

    /// Monotonic count of `TraceEntry`s ever
    /// APPENDED to the trace — increments once per append at both append
    /// choke points (serial `RingTraceSink::push_entry`; the parallel pass 3
    /// fragment merge), INDEPENDENT of the `max_trace_entries` ring eviction,
    /// and DELIBERATELY NOT reset by [`Self::clear_trace`] (an absolute
    /// counter; callers only difference it — see the field doc). This is the
    /// cap-immune "did the last step fire ≥1 node?" signal the live loop's
    /// spin-then-block differences before/after `step()` (a capped-full ring
    /// pins `trace.len()` at the cap, so a length delta reads 0 forever).
    pub(crate) fn entries_appended(&self) -> u64 {
        self.entries_appended
    }

    /// Install the recording trace-ring hook — a minted SPSC
    /// [`crate::trace_ring::TraceRingProducer`] plus the node-id table the
    /// ring owner's manifest was encoded from (`node_ids`, manifest order —
    /// hand the SAME slice to both, never a rebuilt one, so a record's
    /// `node_idx` and the bagged manifest cannot disagree). Call BEFORE the
    /// first `step()` of the recording run; every trace append thereafter is
    /// mirrored to the ring, UNGATED by `max_trace_entries` (the bag's trace
    /// is complete by contract even when the in-memory trace is capped).
    ///
    /// # Install-once, before step 0
    ///
    /// The hook is installed EXACTLY ONCE, BEFORE the first `step()`. A
    /// second install is REFUSED (the first hook is kept — replacing it would
    /// tear the recording across two rings mid-run) and an install after
    /// step 0 is REFUSED (the ring would silently miss every earlier fire —
    /// an incomplete bag). Both refusals are `tracing::error!`-loud.
    #[cfg(unix)]
    // hot-path-alloc-ok-fn: cold: installs the trace-ring producer ONCE at arm time, building its
    // node-id -> manifest-index map
    pub fn set_trace_ring_producer(
        &mut self,
        producer: crate::trace_ring::TraceRingProducer,
        node_ids: &[String],
    ) {
        if self.trace_ring.is_some() {
            tracing::error!(
                "trace ring producer already installed — REFUSING the re-install and \
                 keeping the first hook (the contract is install ONCE, before step 0; \
                 a mid-run swap would tear the recording across two rings)"
            );
            return;
        }
        if self.steps_begun > 0 {
            tracing::error!(
                steps_begun = self.steps_begun,
                "trace ring producer installed AFTER step 0 — REFUSING (the contract \
                 is install ONCE, before the first step(); installing now would \
                 silently omit every fire already executed from the bag)"
            );
            return;
        }
        let node_idx = node_ids
            .iter()
            .enumerate()
            .map(|(i, id)| (id.clone(), i as u32))
            .collect();
        self.trace_ring = Some(TraceRingHook {
            producer,
            node_idx,
            warned_unmapped: std::collections::HashSet::new(),
            unmapped_dropped: 0,
            unmapped_read_outcomes: 0,
        });
    }

    /// Fire records DROPPED from the trace ring because their node
    /// id was missing from the recording manifest (a manifest/scheduler
    /// desync — the handed `node_ids` table did not cover every scheduler
    /// node). `0` when no hook is installed or no desync occurred. Read by
    /// the recording CLI at run exit: a nonzero count makes a desynced
    /// (incomplete) bag VISIBLY wrong instead of quietly finalized.
    #[cfg(unix)]
    pub fn trace_ring_unmapped_count(&self) -> u64 {
        self.trace_ring.as_ref().map_or(0, |h| h.unmapped_dropped)
    }

    /// Kind-6 read-outcome records dropped from the
    /// trace ring for the same manifest-desync reason — counted separately so
    /// [`Self::trace_ring_unmapped_count`] keeps meaning FIRE records only
    /// (its callers report fire-record loss; read-outcome loss is a read-log
    /// hole, not a scheduler-trace hole, and the run-exit report prints the
    /// two distinctly). `0` when no hook is installed or no desync occurred.
    #[cfg(unix)]
    pub fn trace_ring_unmapped_read_outcomes(&self) -> u64 {
        self.trace_ring
            .as_ref()
            .map_or(0, |h| h.unmapped_read_outcomes)
    }

    /// Whether [`Self::set_trace_ring_producer`] really installed a
    /// hook (its two refusal arms leave the slot as it was). The runtime
    /// consults this before ARMING the read-outcome stages, so a refused
    /// install (after step 0 / double-install of a different ring) never arms
    /// staging that nothing will drain into a ring.
    #[cfg(unix)]
    pub(crate) fn trace_ring_installed(&self) -> bool {
        self.trace_ring.is_some()
    }

    /// The LEVEL-END read-outcome merge — drain the level's
    /// nodes' staged read outcomes ([`crate::read_outcome::ReadOutcomeStage`],
    /// filled by the subscribers' drain sites) into the recording trace ring
    /// as kind-6 records. Runs on the STEP THREAD strictly after the level's
    /// tick pass (rayon scope-joined), which is the SPSC ring's single-writer
    /// contract and the stages' single-reader happens-before edge.
    ///
    /// `other_ids` / `block_ids` are the level's two build-time partitions
    /// (`GraphRuntime`'s per-level `LevelPlan` partition) — walked in
    /// that fixed order, each in level order, stages per node in registration
    /// order (body inputs in wiring order, then trigger-drain stages), so the
    /// merged ring order is deterministic run-to-run. The order is NOT
    /// semantically load-bearing: a kind-6 record carries its step / consumer
    /// / input_idx, and the offline reader keys on those.
    ///
    /// A LEVEL-END sweep (not a per-fired-node one) is deliberate: a unified
    /// trigger drain pops frames in `drain_level` even when the node's fire is
    /// then DEFERRED (`throttle_ms` / `block` pre-fire), so a fire-scoped
    /// merge would strand those records across steps (unbounded staging, or a
    /// wrong-step stamp at the eventual fire). Sweeping the level's nodes each
    /// step keeps every record stamped with the step its READ happened on and
    /// bounds staging at one merge cadence.
    ///
    /// Recording OFF ⇒ the `None` early-out is the whole cost (one branch per
    /// level). Recording ON with quiet stages ⇒ one relaxed load per stage
    /// (`drain_into`'s pending fast path) — no lock, no alloc.
    ///
    /// TWO possible push targets — the recording trace ring
    /// (kind-6 records into the bag) and the in-memory sink (replay's
    /// collection seam, [`MergedReadOutcome`]) — mirroring the [`TraceSink`]
    /// Vec-vs-Ring split for fires. Neither installed (every plain run) ⇒ the
    /// early-out is still the whole cost; no production path ever installs
    /// both (a recording has a ring, a replay has the sink), but pushing to
    /// both is well-defined if a caller ever does.
    pub(crate) fn merge_read_outcomes(&mut self, other_ids: &[String], block_ids: &[String]) {
        // Read the step BEFORE mut-borrowing the sinks (`current_step` borrows
        // all of `self`); `begin_step` has always run by the time a level
        // merges, so the saturation never actually engages.
        let step = self.steps_begun.saturating_sub(1);
        let mut ring = self.trace_ring.as_mut();
        let mut memory = self.read_outcome_sink.as_mut();
        if ring.is_none() && memory.is_none() {
            return;
        }
        for id in other_ids.iter().chain(block_ids.iter()) {
            // A level id absent from `nodes` is impossible by construction
            // (both partitions are built from the scheduler's own node set);
            // skipping is the drain-path-safe posture (the desync would
            // already be loud elsewhere).
            let Some(node) = self.nodes.get(id) else {
                continue;
            };
            for stage in &node.read_stages {
                let input_idx = stage.input_idx();
                stage.drain_into(|rec| {
                    // HANDED OVER = at least one
                    // consumer took it. The ring REFUSES a record whose node
                    // id is missing from the recording manifest, and with no
                    // in-memory sink installed that refusal is the whole
                    // story — which is what the overflow marker's debt has to
                    // key on rather than on the offer.
                    let mut handed_over = false;
                    if let Some(hook) = ring.as_mut() {
                        handed_over |= hook.push_read_outcome(step, id, input_idx, &rec);
                    }
                    if let Some(sink) = memory.as_mut() {
                        sink.push(MergedReadOutcome {
                            step,
                            // hot-path-alloc-ok: REPLAY-only: the in-memory sink is installed
                            // solely by `set_read_outcome_memory_sink` (reached from the replay
                            // engine's `enable_read_outcome_memory_sink`), and this fn's own doc
                            // records that no production path installs both sinks — a live or
                            // recording run takes the `ring` arm above and never reaches here,
                            // while a run with neither installed early-returns before the loop.
                            node_id: id.clone(),
                            input_idx,
                            outcome: rec,
                        });
                        handed_over = true;
                    }
                    handed_over
                });
            }
        }
    }

    /// Install the IN-MEMORY read-outcome sink — the replay
    /// engine's collection seam (no POSIX-SHM ring; see [`MergedReadOutcome`]).
    /// Call before the first `step()`; installing later simply misses the
    /// earlier steps' merges (the read log's install-before-step-0 discipline
    /// is the caller's — replay installs it right after build). Idempotent-ish:
    /// a second install resets the pending buffer (no production caller does).
    // hot-path-alloc-ok-fn: cold: installs the replay engine's collection sink ONCE, before the
    // first `step()`
    pub(crate) fn set_read_outcome_memory_sink(&mut self) {
        self.read_outcome_sink = Some(Vec::new());
    }

    /// Take every read outcome the level-end merges pushed
    /// into the in-memory sink since the last call (merge order — level by
    /// level, node registration order within a level, chronological within a
    /// stage), leaving the sink installed and empty. `Vec::new()` when no sink
    /// is installed (a non-replay run) — allocation-free.
    pub(crate) fn take_merged_read_outcomes(&mut self) -> Vec<MergedReadOutcome> {
        self.read_outcome_sink
            .as_mut()
            .map(std::mem::take)
            .unwrap_or_default()
    }

    /// Clear the execution trace.
    pub fn clear_trace(&mut self) {
        self.trace.clear();
    }

    /// Run a node's installed pre-fire `Block` defer predicate
    /// (if any) under a `catch_unwind`. Returns `true` if the node's tick
    /// should be DEFERRED this step, `false` to allow firing.
    ///
    /// Panic safety: a panicking pre_fire_check would otherwise propagate
    /// through `evaluate_node`, killing `Scheduler::step` entirely (no
    /// circuit-breaker analog to the `fire_node` `MAX_CONSECUTIVE_PANICS`
    /// path). A panic is logged via `tracing::error!` and treated as
    /// "no defer" — the conservative choice: defer-on-panic could lock the
    /// scheduler indefinitely if the closure is permanently buggy.
    ///
    /// The closure receives `current_time_ns` (deterministic — it reads
    /// deterministic atomics + the simulated/real clock value passed in),
    /// so calling it once per missed Period interval inside the catch-up
    /// loop preserves replay determinism.
    fn run_pre_fire_check(
        id: &str,
        check: &Arc<dyn Fn(u64) -> bool + Send + Sync>,
        current_time_ns: u64,
    ) -> bool {
        let result =
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| (check)(current_time_ns)));
        match result {
            Ok(deferred) => deferred,
            Err(panic_info) => {
                let msg = if let Some(s) = panic_info.downcast_ref::<&str>() {
                    // hot-path-alloc-ok: cold: renders a CAUGHT PANIC payload — the branch runs
                    // only when a node callback unwound
                    s.to_string()
                } else if let Some(s) = panic_info.downcast_ref::<String>() {
                    // hot-path-alloc-ok: cold: renders a CAUGHT PANIC payload — the branch runs
                    // only when a node callback unwound
                    s.clone()
                } else {
                    // hot-path-alloc-ok: cold: renders a CAUGHT PANIC payload — the branch runs
                    // only when a node callback unwound
                    "unknown panic".to_string()
                };
                tracing::error!(
                    node_id = %id,
                    error = %msg,
                    "pre-fire check panicked; treating as no-defer to avoid \
                     scheduler lockup. Inspect the installed closure for a bug."
                );
                false
            }
        }
    }

    /// Evaluate a single node's trigger policy.
    ///
    /// This is now the FUSED composition `tick_node ∘
    /// decide_node` — `decide_node` decides whether (and how) the node fires
    /// and mutates its scheduler state, `tick_node` performs the actual fire.
    /// Kept as a single fn so the flat `Scheduler::step` / `evaluate_one` path
    /// stays byte-identical to its pre-split form (same trace, same counter
    /// bumps, same order); the level executor calls `decide_node` / `tick_node`
    /// separately so it can interleave a fire-gated input snapshot between them.
    // 8 args: see `fire_node` — same funnel, same borrow rationale.
    #[allow(clippy::too_many_arguments)]
    fn evaluate_node(
        id: &str,
        node: &mut ScheduledNode,
        current_time_ns: u64,
        global_level: usize,
        step: u64,
        trace: &mut VecDeque<TraceEntry>,
        max_trace_entries: Option<usize>,
        entries_appended: &mut u64,
        ring: Option<&mut TraceRingHook>,
        catchup_cap_override: Option<u32>,
    ) {
        if let Some(kind) = Self::decide_node(id, node, current_time_ns, catchup_cap_override) {
            Self::tick_node(
                id,
                node,
                kind,
                global_level,
                step,
                trace,
                max_trace_entries,
                entries_appended,
                ring,
            );
        }
    }

    /// The DECIDE half of `evaluate_node` — evaluate the
    /// node's trigger policy and mutate its scheduler state (advance
    /// `next_fire_ns`, clear `pending_data_count` / `sync_input_timestamps` /
    /// `external_triggered`), returning the deferred fire recipe ([`FireKind`])
    /// if the node fires this step, or `None` if it does not.
    ///
    /// All state mutations the fused `evaluate_node` performed BEFORE the first
    /// `fire_node` happen HERE — argued byte-identical because nothing between
    /// `decide_node` and `tick_node` (the runtime's fire-gated snapshot) reads
    /// this node's scheduler state. The TOP `pre_fire_check` gate (its counter
    /// side-effect + warn) runs EXACTLY ONCE per node per step, here. The Period
    /// per-catch-up `pre_fire_check` re-check runs later, in `tick_node`.
    ///
    /// `catchup_cap_override` is THIS step's clamp, derived
    /// ONCE in [`Self::begin_step`] and passed down rather than re-read here —
    /// see [`Scheduler::catchup_cap_override`] and
    /// [`catchup_clamp::effective_max_catchup`]. It is `None` on every
    /// un-armed run, where the Period arm is byte-identical to its earlier
    /// form.
    fn decide_node(
        id: &str,
        node: &mut ScheduledNode,
        current_time_ns: u64,
        catchup_cap_override: Option<u32>,
    ) -> Option<FireKind> {
        if node.disabled {
            return None;
        }
        // Block defer-the-fire predicate. If installed
        // AND the closure returns true, skip the entire evaluation —
        // no firing, no trace, no panic count change. The closure has
        // already bumped any relevant counters + emitted the warn.
        // Runs BEFORE trigger evaluation because the producer should
        // not even tick if any downstream sole-Block-consumer is full.
        // This top-of-decide check gates whether we enter the policy
        // match at all. Counter side-effect runs EXACTLY ONCE per node
        // per step (here, in decide).
        //
        // Period-policy catch-up burst note:
        // a `TriggerPolicy::Period` producer that was deferred for several
        // steps accumulates missed fire intervals. The catch-up loop in the
        // Period arm below would, on resume, fire ALL of them in ONE step.
        // For a `block` producer that is a CLAMP point: N publishes into a
        // consumer queue with room for ~1 would overflow iceoryx2 (silent
        // data loss) AND jump the outstanding mirror by N with no matching
        // drain, livelocking the producer forever. To prevent BOTH, the
        // Period arm (in `tick_node`) re-evaluates `run_pre_fire_check`
        // BEFORE EACH `fire_node` and BREAKs (leaving `next_fire_ns` at the
        // first un-fired interval) the moment the predicate defers again — so
        // the catch-up burst is bounded by both `max_catchup` AND the live
        // queue-fullness. The burst is therefore no longer ONLY a latency
        // concern; it is also clamped to prevent overflow.
        // hot-path-alloc-ok: not a heap allocation: cloning an `Arc`/`Option<Arc>` is a refcount
        // bump (`pre_fire_check` is an `Option<Arc<dyn Fn>>`)
        let pre_fire_check = node.pre_fire_check.clone();
        if let Some(check) = &pre_fire_check {
            if Self::run_pre_fire_check(id, check, current_time_ns) {
                tracing::trace!(
                    node_id = %id,
                    "scheduler pre-fire check deferred node tick (Block backpressure); \
                     Period nodes will accumulate missed fires until resume — bound via `max_catchup`"
                );
                return None;
            }
        }
        match &node.policy {
            TriggerPolicy::Period {
                interval,
                max_catchup,
            } => {
                // Describe the catch-up burst ARITHMETICALLY (first
                // fire + count + interval) instead of collecting each interval
                // into a heap `Vec` — eliminating one allocation per Period
                // decide on the moat path. The recorded `first_fire_ns` is the
                // earliest un-fired interval; the `fire_count` intervals are
                // `first_fire_ns + k*interval_ns` for k in 0..fire_count.
                // `next_fire_ns` is advanced by the SAME running add the old
                // collect loop used, so it lands in the identical place
                // (including the `max_catchup` cap-skip), and the
                // reconstruction in `tick_node[_collect]` mirrors the old
                // pushed sequence byte-for-byte.
                let interval_ns = interval.as_nanos() as u64;
                // An EXPLICIT `max_catchup` always wins;
                // the clamp only replaces the unbounded default, and only while
                // a state arm is attached past its agreed onset step.
                let cap = catchup_clamp::effective_max_catchup(*max_catchup, catchup_cap_override);
                let mut fire_count: u32 = 0;
                let mut first_fire_ns: u64 = 0;
                if let Some(next_fire) = node.next_fire_ns.as_mut() {
                    // ONE implementation of the advance
                    // (`catchup_clamp::period_advance`), shared with the replay
                    // verifier's re-derivation. It was a `while` loop here and
                    // an O(1) form there — two spellings of one rule, whose
                    // whole job is to agree — and the loop could not terminate
                    // at all on a zero interval or a wall-faithful anchor. The
                    // O(1) form is adopted; see that function's doc for why the
                    // two are the same answer wherever the loop finished.
                    let adv = catchup_clamp::period_advance(
                        *next_fire,
                        current_time_ns,
                        interval_ns,
                        cap,
                    );
                    fire_count = adv.fire_count;
                    // The earliest un-fired interval (the old loop's first
                    // pushed `*next_fire`, recorded before any advance).
                    // `period_advance` reports 0 when nothing is due, which is
                    // the value this variable already held there.
                    first_fire_ns = adv.first_fire_ns;
                    *next_fire = adv.next_fire_ns;
                }
                if fire_count == 0 {
                    None
                } else {
                    Some(FireKind::Period {
                        first_fire_ns,
                        fire_count,
                        interval_ns,
                        current_time_ns,
                    })
                }
            }
            TriggerPolicy::Data => {
                // The hint describes a frame the LAST step's fire
                // loop popped and froze without firing. This step's boundary
                // drain has already run (drain → decide → tick), so it has
                // re-offered that frame and signalled it; the observation is
                // spent. Clear it here — once per node per step, on the one
                // path that runs whether or not the node fires — so the live
                // loop's wake window is held open by THIS step's evidence and
                // never by a stale bit.
                node.data_backlog_hint = false;
                if node.pending_data_count > 0 {
                    // Per-message FIFO firing: serve up to
                    // `DATA_PENDING_CARRY_CLAMP` queued frames in THIS step,
                    // one fire each, each tick's FIFO pop-one read serving the
                    // next frame in order — "fire on each message arriving".
                    // The pre-FIFO collapse (reset to 0 ⇒ one fire per step,
                    // tick sees only the newest) silently discarded N−1 frames
                    // per burst.
                    //
                    // The count is the arrivals SIGNALLED so far; on a
                    // `DrainSource::Unified` input that is 1 however many frames
                    // are queued, and `tick_node` refills between fires. See
                    // [`FireKind::Data`].
                    //
                    // Anything past the cap CARRIES to the next step and reports
                    // due-NOW through `ns_until_next_fire`, so on the live path
                    // the wake window stays at the loop's 1 ms floor until it
                    // clears — never a busy-spin, and never a per-frame tail.
                    let fire_count = node.pending_data_count.min(DATA_PENDING_CARRY_CLAMP);
                    node.pending_data_count -= fire_count;
                    node.pending_data_count_shared
                        .store(node.pending_data_count, Ordering::Release);
                    Some(FireKind::Data {
                        fire_time_ns: current_time_ns,
                        // `fire_count <= DATA_PENDING_CARRY_CLAMP` (= 64), so
                        // the narrowing is lossless.
                        fire_count: fire_count as u32,
                    })
                } else {
                    None
                }
            }
            TriggerPolicy::Sync { inputs, window } => {
                // Same rule as the Data arm's hint clear, for the
                // same reason — the boundary's align pass has already run, so
                // last step's observation is spent and the live loop's wake
                // window must be held open by THIS step's evidence. Cleared on
                // the not-firing path too; the burst re-sets it.
                node.sync_backlog_hint = false;
                node.sync_counters
                    .backlog_pending
                    .store(false, Ordering::Release);

                // Re-derive the verdict from the heads with EVERY `next_info`
                // at `None`, which disables descent while leaving completeness
                // and window death exactly as they were. On the transport path
                // the align pass already settled the membership, so this is a
                // cheap re-check of what it left; on the HAND-DRIVEN path
                // (`signal_sync_input`, no ops installed) it is the whole
                // decision, and the all-`None` scratch is what keeps that path
                // on its classic semantics rather than issuing ops that do not
                // exist.
                let scratch_heads = &mut node.sync_scratch_heads;
                let scratch_next = &mut node.sync_scratch_next;
                scratch_heads.clear();
                scratch_next.clear();
                for input in inputs {
                    scratch_heads.push(node.sync_heads.get(input).copied());
                    scratch_next.push(NextInfo::None);
                }
                match next_sync_step(scratch_heads, scratch_next, *window) {
                    SyncStep::Fire => Some(FireKind::Sync {
                        fire_time_ns: current_time_ns,
                        max_sets: DATA_PENDING_CARRY_CLAMP as u32,
                    }),
                    // A `DiscardTie` here means the align pass never ran (a
                    // hand-driven node) or a hand signal landed after it: the
                    // tuple is out of window, so it must not fire — byte-for-
                    // byte what `check_sync` answered for the same shape.
                    _ => None,
                }
            }
            TriggerPolicy::External => {
                if node.external_triggered {
                    node.external_triggered = false;
                    Some(FireKind::Single {
                        fire_time_ns: current_time_ns,
                    })
                } else {
                    None
                }
            }
        }
    }

    /// Advance an input's `expect_within` watchdog anchor to a
    /// refilled frame's wire timestamp — the same store
    /// [`Self::signal_input_received`] performs for the frame the boundary drain
    /// popped, applied to the frames the Data burst's refills pop after it.
    ///
    /// Without it, a burst drained entirely inside one step would leave the
    /// anchor at the burst's FIRST frame while the node consumed frames stamped
    /// much later — and since the next step's boundary drain finds the queue
    /// empty (nothing to re-anchor with), a subsequent quiet period would be
    /// measured from a stamp that is a whole burst too old and could report a
    /// deadline miss that did not happen.
    ///
    /// `None` (a drain that popped a frame carrying no timestamp) leaves the
    /// anchor alone — never invented.
    /// Serve up to `max_sets` COMPLETE aligned sets in ONE step, one
    /// fire each, in set order — the Sync twin of [`Self::tick_data_burst`],
    /// mirroring it seam for seam.
    ///
    /// # Why it re-ALIGNS rather than counting down
    ///
    /// Data knows its `fire_count` because every arrival was SIGNALLED. Sets
    /// cannot be counted without popping — the matcher only learns a set exists
    /// by forming it — so this loop asks the alignment between fires and stops
    /// the moment one comes back incomplete. `max_sets` is therefore a ceiling
    /// on how many it may serve before the remainder carries.
    ///
    /// # The four stop conditions, and what each leaves behind
    ///
    /// * **Nothing more** — the refill alignment is incomplete. The ordinary
    ///   end; nothing is owed.
    /// * **The cap** — `fired == max_sets` with a set still aligned. The set
    ///   stays in the heads, `sync_backlog_hint` reports due-NOW, and the next
    ///   step serves it.
    /// * **A pre-fire defer** — including BEFORE fire 1, on a set the BOUNDARY
    ///   aligned. That first-fire case is exactly why the hint's setting rule is
    ///   broader than `refilled_unfired`'s (see [`ScheduledNode::sync_backlog_hint`]).
    /// * **The panic breaker** — re-read after EVERY fire, exactly as the Data
    ///   burst does, because `fire_node_into` may have just opened it.
    ///
    /// # The panic guarantee — MEASURED, and better than designed
    ///
    /// Two shapes, and the loss is bounded at ONE set in both.
    ///
    /// * **A panic that left any member UNTAKEN** (a closure panicking before
    ///   its reads, a collapsed chain, any deferred fire): that input's slot
    ///   stays frozen, the refill answers "nothing", the alignment is
    ///   incomplete and the burst ends — one fire, members intact.
    /// * **A panic AFTER every member was taken** (every macro user-body panic,
    ///   the shipping shape): the members are gone. One might expect
    ///   the burst to then keep aligning FRESH sets into the panicking body
    ///   until `MAX_CONSECUTIVE_PANICS` opened the breaker — up to 3 sets eaten
    ///   inside one step, mirroring `tick_data_burst`. **It does not, and the
    ///   reason is worth knowing:** the tick runs while the runtime holds the
    ///   node's `Mutex`, so the panic POISONS it (a pre-existing property the
    ///   runtime documents at its poison-recovering read sites). Every
    ///   subsequent align op goes through that lock, gets `Err`, and maps to
    ///   `SyncOpAnswer::Failed`, which the failed-probe policy resolves fail-closed — so the
    ///   alignment is incomplete and this loop ends at ONE fire.
    ///
    /// The two burst twins still agree: the Data refill hook carries the same
    /// `Err(_) => (0, None)` arm for the same reason. Pinned by
    /// `sync_per_set_iox2_test::a_post_read_panic_loses_exactly_one_set_and_the_burst_cannot_cascade`.
    fn tick_sync_burst<S: TraceSink>(
        id: &str,
        node: &mut ScheduledNode,
        fire_time_ns: u64,
        max_sets: u32,
        global_level: usize,
        step: u64,
        sink: &mut S,
    ) {
        debug_assert!(
            max_sets > 0,
            "tick_sync_burst: a Sync FireKind must carry max_sets > 0 (decide_node returns \
             None when no set is aligned)"
        );
        // hot-path-alloc-ok: not a heap allocation: cloning an `Option<Arc<..>>` is a
        // refcount bump (the same shape `run_qos_windows` annotates for `qos_events`)
        let pre_fire_check = node.pre_fire_check.clone();
        let mut fired: u32 = 0;
        // `decide_node` only returns `FireKind::Sync` when a complete in-window
        // set sits in the heads, so the loop starts owing one.
        let mut aligned_unfired = true;

        loop {
            if fired >= max_sets {
                break;
            }
            if let Some(check) = &pre_fire_check {
                if Self::run_pre_fire_check(id, check, fire_time_ns) {
                    tracing::trace!(
                        node_id = %id,
                        fired,
                        "pre-fire check deferred a Sync burst; the aligned set stays in the \
                         heads and carries to a later step"
                    );
                    break;
                }
            }
            // There is deliberately NO fire-time watchdog re-anchor
            // here. One shipped and was INERT (it looked heads up by resolved
            // TOPIC in a map keyed by macro FIELD NAME, so every lookup
            // missed), and fixing the key space was then MEASURED to change
            // nothing: EVERY frame that leaves the queue passes the delivered
            // arm of `drain_with_accounting_impl` and anchors there — the
            // align pass's fill, the descent's `NeedStamp` pop into the staged
            // slot, the `Advance`/`DiscardTie` refills alike. A promotion and a
            // boundary re-offer move a frame that was already anchored when it
            // was popped, so a fire-time walk can only ever re-write the same
            // value. The one case it would NOT re-write is a RESTORED unbacked
            // head, which names no live frame at all and whose recorded stamp
            // belongs to another clock epoch — writing that into an
            // arrival-only anchor would be a fabrication, not a fix. So the
            // second writer is gone rather than repaired; the anchors are the
            // pops' business, and a HELD member's window is suppressed by
            // `run_qos_windows` instead of re-anchored.

            Self::fire_node_into(id, node, fire_time_ns, global_level, step, sink);
            fired += 1;

            // The set's members are spent. TOMBSTONE rather than clear: an
            // `Emitted` head is what tells the next alignment to re-drain THIS
            // input, which is the whole difference from the earlier
            // clear-the-map-on-fire that collapsed bursts.
            Self::mark_sync_heads_emitted(node);
            aligned_unfired = false;

            // The panic circuit breaker: `fire_node_into` may have just opened
            // it, and nothing else on this path re-reads the flag.
            if node.disabled {
                break;
            }

            match Self::align_node_sync_heads(id, node, SyncAlignSite::Refill) {
                AlignOutcome::Complete => aligned_unfired = true,
                AlignOutcome::Incomplete => break,
            }
        }

        if aligned_unfired {
            node.sync_backlog_hint = true;
            node.sync_counters
                .backlog_pending
                .store(true, Ordering::Release);
        }
    }

    /// Mark the fired set's members `Emitted`.
    ///
    /// The tombstone is deliberately NOT a removal: it keeps the input's key in
    /// the map (so the align pass walks a stable set) and it is the state the
    /// capture rule reads — `Emitted` heads are NOT written into the framework
    /// section, which is what reproduces today's cleared-on-fire capture
    /// byte-for-byte.
    fn mark_sync_heads_emitted(node: &mut ScheduledNode) {
        // A set just FIRED, so every member input is matching again —
        // the healthy transition the per-input UNMATCHABLE latch needs. Without
        // it the loud head is once per PROCESS: a node that hits a death regime,
        // recovers, and hits another reports the second one at `debug!`, which
        // is invisible at the default level, and the recovery `info!` the site
        // documents can never fire at all.
        // Split borrows so the recovery needs no allocation: this runs once per
        // FIRE, and collecting the member names would be a per-fire heap
        // allocation on the hot path.
        let ScheduledNode {
            sync_heads,
            sync_unmatched_latches,
            ..
        } = node;
        for (input, head) in sync_heads.iter_mut() {
            if !head.is_filled() {
                continue;
            }
            head.state = sync_match::HeadState::Emitted;
            if let Some(latch) = sync_unmatched_latches.get_mut(input.as_str()) {
                if let Some(suppressed) = latch.on_success() {
                    tracing::info!(
                        sync_input = %input,
                        suppressed_discards = suppressed,
                        "sync UNMATCHABLE regime closed — this input's frames are \
                         joining sets again"
                    );
                }
            }
        }
    }

    fn note_trigger_arrival(node: &ScheduledNode, input: &str, timestamp_ns: Option<u64>) {
        let Some(ts) = timestamp_ns else {
            return;
        };
        if let Some(entry) = node.input_expect_within.get(input) {
            entry.last_event_ns.store(ts, Ordering::Release);
        }
    }

    /// The `Data` burst fire loop — ONE implementation shared by the
    /// serial ([`Self::tick_node`]) and parallel ([`Self::tick_node_into`]) tick
    /// paths, which differ only in the [`TraceSink`] they hand it. (The `Period`
    /// catch-up loop is mirrored in both; this one is not, so the two paths
    /// cannot drift.)
    ///
    /// Serves up to [`DATA_PENDING_CARRY_CLAMP`] queued frames in this step, each
    /// its own fire and its own `TraceEntry`, all stamped with the SAME
    /// `fire_time_ns` (the `Period` catch-up precedent).
    ///
    /// Four things stop the loop, and they are different:
    ///
    /// * **Nothing more to serve.** `fire_count` signalled arrivals are
    ///   exhausted and either there is no refill hook (a `Separate` binding: the
    ///   boundary drain signalled every queued frame, so the count WAS the
    ///   burst; or a node that cannot refill — see
    ///   [`crate::graph::node::NodeEntry::refills_trigger_input`]) or the hook
    ///   reports nothing left. Nothing carries.
    ///
    ///   "Nothing left" covers TWO states, and conflating them is the bug this
    ///   loop's refill entry point exists to avoid: the queue really is empty,
    ///   OR the fire just run did not CONSUME the head this input is holding
    ///   (its tick never reached that input's `try_view`). Both mean "do not
    ///   fire again" — the second because the node has already been fired for
    ///   that frame, and the next boundary drain will re-offer it.
    /// * **A pre-fire defer.** `run_pre_fire_check` is re-evaluated BEFORE EACH
    ///   fire, exactly as the `Period` catch-up loop does, so a `throttle_ms`
    ///   node fires at most once per step (after its first fire `last_fire ==
    ///   now`, so the gate defers) and a `block` producer stops the instant a
    ///   downstream queue fills mid-burst. The unfired signalled remainder is
    ///   given back to `pending_data_count` — deferral never loses a fire.
    /// * **The per-step cap.** Bounded work per step regardless of how far
    ///   behind the loop fell.
    /// * **The panic circuit breaker.** `fire_node_into` opens it after
    ///   `MAX_CONSECUTIVE_PANICS` consecutive panics, and the only `disabled`
    ///   gate on this path is in `decide_node` — which ran BEFORE this burst. So
    ///   the flag is re-read after every fire: without that, an
    ///   always-panicking callback runs the REST of the batch, up to
    ///   [`DATA_PENDING_CARRY_CLAMP`] `catch_unwind` cycles and trace entries in
    ///   ONE step, every one of them after the breaker opened. Pre-FIFO the node
    ///   fired once per step, so it stopped after three.
    ///
    /// The refill is skipped while a signalled remainder is still carried past
    /// the cap: those arrivals fire on later steps, and pulling a frame ahead of
    /// them would serve the FIFO out of step with the signals describing it.
    fn tick_data_burst<S: TraceSink>(
        id: &str,
        node: &mut ScheduledNode,
        fire_time_ns: u64,
        fire_count: u32,
        global_level: usize,
        step: u64,
        sink: &mut S,
    ) {
        debug_assert!(
            fire_count > 0,
            "tick_data_burst: Data FireKind must carry fire_count > 0 (decide_node returns None on empty)"
        );
        // hot-path-alloc-ok: not a heap allocation: both are `Option`s over `Arc`s, so each
        // clone is a refcount bump — `pre_fire_check` is `Option<Arc<dyn Fn>>`, and
        // `TriggerRefill`'s derived `Clone` clones its two fields, `Arc<str>` + `Arc<dyn Fn>`.
        // The enclosing fn runs per Data fire, so this is the LINE form deliberately.
        let pre_fire_check = node.pre_fire_check.clone();
        // hot-path-alloc-ok: see the block above — `TriggerRefill`'s derived `Clone` clones two
        // `Arc`s (`Arc<str>` + `Arc<dyn Fn>`), so this is two refcount bumps and no allocation.
        // A second annotation because a code line ends the preceding comment block.
        let refill = node.trigger_refill.clone();
        let cap = DATA_PENDING_CARRY_CLAMP as u32;
        // Signalled arrivals this step still owes a fire for.
        let mut remaining = fire_count;
        let mut fired: u32 = 0;
        // `true` while `remaining` describes a frame a REFILL popped (already
        // popped and frozen in the subscriber's slot) rather than a signalled
        // arrival — the two carry differently when the loop stops.
        let mut refilled_unfired = false;
        loop {
            if remaining == 0 {
                if node.pending_data_count > 0 {
                    break;
                }
                let Some(r) = refill.as_ref() else {
                    break; // Separate binding: the signalled count WAS the burst.
                };
                let (popped, latest_ts) = (r.drain)();
                if popped == 0 {
                    break; // queue empty — the burst is fully served
                }
                Self::note_trigger_arrival(node, &r.input, latest_ts);
                remaining = 1; // the drain's pop-one contract
                refilled_unfired = true;
            }
            if fired >= cap {
                break;
            }
            if let Some(check) = &pre_fire_check {
                if Self::run_pre_fire_check(id, check, fire_time_ns) {
                    tracing::trace!(
                        node_id = %id,
                        fired,
                        "pre-fire check deferred mid Data burst; the unfired remainder \
                         carries to a later step"
                    );
                    break;
                }
            }
            Self::fire_node_into(id, node, fire_time_ns, global_level, step, sink);
            fired += 1;
            remaining -= 1;
            refilled_unfired = false;
            // The panic circuit breaker (see this fn's docs): `fire_node_into`
            // may have just opened it, and nothing else on this path re-reads
            // the flag. The break lands AFTER the bookkeeping, so the fire that
            // opened the breaker is fully accounted for; `refilled_unfired` is
            // false here, so the tail hands the signalled remainder back to
            // `pending_data_count` exactly as a mid-burst defer does. That
            // needs no special case — `decide_node` gates a disabled node, so
            // the carry cannot fire while the breaker is open, and what a
            // re-enabled node does with it is `reset_node`'s business.
            if node.disabled {
                break;
            }
        }
        if refilled_unfired {
            // A frame is popped and frozen but NOT fired. It must NOT become a
            // pending arrival: the next boundary drain RE-OFFERS a held head and
            // mints its signal then, so counting it here would fire the node
            // twice for one frame. The hint is the one thing that tells the live
            // loop to come back promptly for it.
            node.data_backlog_hint = true;
        } else if remaining > 0 {
            // Signalled arrivals a mid-burst defer left unfired — give them back
            // so a later step serves them. Re-clamped for the same reason
            // `signal_data` clamps.
            node.pending_data_count =
                (node.pending_data_count + remaining as u64).min(DATA_PENDING_CARRY_CLAMP);
            node.pending_data_count_shared
                .store(node.pending_data_count, Ordering::Release);
        }
    }

    /// The TRACE-DRIVEN burst — fire this node exactly as many
    /// times as the recording says, at the recorded instants.
    ///
    /// ONE implementation shared by the serial ([`Self::tick_node`]) and
    /// parallel ([`Self::tick_node_into`]) tick paths, which differ only in the
    /// [`TraceSink`] they hand it (the `tick_data_burst` precedent — the two
    /// paths cannot drift).
    ///
    /// # The refill hook is driven EXPLICITLY, by the plan's count
    ///
    /// A live `Data` burst discovers its own length: it fires, then asks the
    /// [`TriggerRefill`] hook whether another frame is queued, and stops when
    /// the hook reports nothing. Here the LENGTH is given — the recording states
    /// it — so the hook is driven once per fire beyond the first, purely to POP
    /// the next FIFO frame into the subscriber's frozen slot. That is what makes
    /// the k-th replayed fire read the k-th recorded frame rather than re-reading
    /// the head, and it is the same `NodeEntry::drain_trigger_input` the live
    /// refill calls (never a bypass — a bypass injection would silently lose the
    /// drain's accounting and its `BackpressureEvent` dispatch).
    ///
    /// A node with no refill hook (`Period`, `Sync`, `External`, or a `Separate`
    /// data binding) has nothing to pop and the loop is a plain k-fire walk.
    ///
    /// # A refill that pops NOTHING still fires
    ///
    /// The plan is authoritative for the fire SCHEDULE. Skipping the fire would
    /// convert an input-injection shortfall into a fabricated fire-schedule
    /// divergence — blaming the candidate's control flow for the harness's
    /// missing frame — so the fire happens, the node re-reads its held head, and
    /// the shortfall is counted on
    /// [`ScheduledNode::replay_refill_shortfalls`] (read via
    /// [`Self::replay_refill_shortfalls`]) so the real cause is nameable.
    ///
    /// The panic circuit breaker is re-read after every fire, exactly as the
    /// `Data` and `Period` bursts do: the only `disabled` gate on this path ran
    /// in the decide seam, before the burst.
    // 8 args: the same funnel + borrow rationale `tick_node` states — the
    // `FireKind::Replay` triple is destructured by the caller so BOTH tick paths
    // hand this one body identical values.
    #[allow(clippy::too_many_arguments)]
    fn tick_replay_burst<S: TraceSink>(
        id: &str,
        node: &mut ScheduledNode,
        first_fire_ns: u64,
        fire_count: u32,
        interval_ns: u64,
        global_level: usize,
        step: u64,
        sink: &mut S,
    ) {
        debug_assert!(
            fire_count > 0,
            "tick_replay_burst: Replay FireKind must carry fire_count > 0 (set_replay_fire_plan refuses 0)"
        );
        // hot-path-alloc-ok: not a heap allocation: `TriggerRefill`'s derived `Clone`
        // clones two `Arc`s (`Arc<str>` + `Arc<dyn Fn>`) — two refcount bumps, mirroring
        // `tick_data_burst`'s live twin — and this fn runs only under a REPLAY plan.
        let refill = node.trigger_refill.clone();
        let mut fire_time = first_fire_ns;
        for i in 0..fire_count {
            if i > 0 {
                if let Some(r) = refill.as_ref() {
                    let (popped, latest_ts) = (r.drain)();
                    if popped == 0 {
                        node.replay_refill_shortfalls =
                            node.replay_refill_shortfalls.saturating_add(1);
                        tracing::debug!(
                            node_id = %id,
                            step,
                            fire_index = i,
                            "trace-driven fire found no frame to refill — the recording holds \
                             a consumed frame this replay's input stream does not; firing \
                             anyway (the fire schedule is the recording's)"
                        );
                    } else {
                        Self::note_trigger_arrival(node, &r.input, latest_ts);
                    }
                }
            }
            Self::fire_node_into(id, node, fire_time, global_level, step, sink);
            // The INTRA-STEP seam. Placed after the fire and its
            // bookkeeping, and BEFORE the `disabled` break, so a slot the
            // recording holds after the burst's LAST fire is still delivered —
            // including the last fire of a burst the panic breaker just ended.
            // The recording published those frames at that instant; withholding
            // them would replace a node failure with an input divergence.
            Self::consume_intra_step_pause(id, node, step, i + 1);
            // Running add, never `i * interval` — the overflow rule the
            // `Period` reconstruction states.
            fire_time = fire_time.saturating_add(interval_ns);
            if node.disabled {
                break;
            }
        }
    }

    /// Hand control to the injection hook if the installed pause
    /// list names this node's `completed`-th fire.
    ///
    /// `completed` is 1-BASED: it is the number of fires of THIS step's burst
    /// that have returned, so calling it with `i + 1` from the burst loop means
    /// "between fire `i` and fire `i + 1`", and with `fire_count` means "after
    /// the last one".
    ///
    /// Allocation-free by construction: the list is sorted ascending and
    /// duplicate-free (both enforced at install) and `completed` walks 1, 2, 3…,
    /// so the whole consult is ONE indexed compare against the cursor entry —
    /// no scan, no clone (the hook is borrowed, not `Arc::clone`d).
    ///
    /// # Never on a live path
    ///
    /// Reached only from [`Self::tick_replay_burst`], which is reached only from
    /// a [`FireKind::Replay`], which only [`Self::take_planned_fire`] mints —
    /// i.e. only under an installed [`ReplayFirePlan`]. That is a STRUCTURAL
    /// exclusion, not a flag: a live scheduler has no way to reach this fn.
    #[inline]
    fn consume_intra_step_pause(id: &str, node: &mut ScheduledNode, step: u64, completed: u32) {
        if node.replay_pauses.after_fires.is_empty() {
            return;
        }
        if node.replay_pauses.step != step {
            // The stale-list rule `ReplayFirePlan` states: deliver NOTHING (a
            // stale plan must never fall through), count it unconditionally,
            // and log it once per install rather than once per fire.
            node.replay_pauses.mismatches = node.replay_pauses.mismatches.saturating_add(1);
            if !node.replay_pauses.mismatch_reported {
                node.replay_pauses.mismatch_reported = true;
                tracing::error!(
                    node_id = %id,
                    pause_step = node.replay_pauses.step,
                    step,
                    "replay intra-step pauses are installed for a DIFFERENT step — this \
                     burst injects NOTHING (a stale list must never deliver). Install one \
                     pause list per step before stepping."
                );
            }
            return;
        }
        let cursor = node.replay_pauses.cursor;
        if node.replay_pauses.after_fires.get(cursor).copied() != Some(completed) {
            return;
        }
        let Some(hook) = node.replay_injection_hook.as_ref() else {
            // A pause with nothing to hand control to. `set_replay_intra_step_pauses`
            // refuses that at install, so this is reachable only by clearing the
            // hook AFTER installing pauses. The cursor deliberately does NOT
            // advance: the slot's frames were never injected, and the accurate
            // report of that is an UNCONSUMED pause, not a served one.
            tracing::error!(
                node_id = %id,
                step,
                after_fire = completed,
                "a trace-driven burst reached an intra-step pause with NO injection hook \
                 installed — the recording's foreign frames for this slot were NOT \
                 injected (reported by unconsumed_replay_pauses)"
            );
            return;
        };
        // The hook is ENGINE code called from inside a node's burst. A panic
        // here would unwind straight out of `step()`, leaving the replay with no
        // counter, no report and no verdict — and the node-tick `catch_unwind`
        // does NOT cover it: that frame has already returned by the time this
        // consult runs, which is why the catch is LOCAL. So it is caught,
        // counted, and reported the way every other loss on this seam is: the
        // cursor does NOT advance, so the slot surfaces in
        // `unconsumed_replay_pauses`.
        //
        // `AssertUnwindSafe` because `ReplayInjectionHook` is a `dyn Fn` and
        // therefore not `RefUnwindSafe`. Unwind safety is a LINT, not a
        // memory-safety property, so the accurate claim is the narrow one: nothing
        // the SCHEDULER owns is left torn — the only state this fn touches is
        // the node's own pause bookkeeping, updated below on both arms. The
        // residual is the hook's OWN captured state, which this frame cannot see
        // and cannot repair; the hook stays installed and IS called again (at
        // later slots and on later steps), which is exactly why a hook that can
        // fail must own its failure through its own latch.
        let outcome =
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| hook(id, completed)));
        if let Err(panic_info) = outcome {
            node.replay_pauses.hook_panics = node.replay_pauses.hook_panics.saturating_add(1);
            let msg = if let Some(s) = panic_info.downcast_ref::<&str>() {
                // hot-path-alloc-ok: cold: renders a CAUGHT PANIC payload — the branch runs
                // only when the replay injection hook unwound
                s.to_string()
            } else if let Some(s) = panic_info.downcast_ref::<String>() {
                // hot-path-alloc-ok: cold: renders a CAUGHT PANIC payload — the branch runs
                // only when the replay injection hook unwound
                s.clone()
            } else {
                // hot-path-alloc-ok: cold: renders a CAUGHT PANIC payload — the branch runs
                // only when the replay injection hook unwound
                "unknown panic".to_string()
            };
            // UNLATCHED, unlike the stale-list arm above, and there is no flood
            // to suppress: the cursor is PARKED by the `return` below, so the
            // head entry can never match a later `completed` (which walks 1, 2,
            // 3… while the list is sorted and duplicate-free), and the list is
            // tagged for ONE step. The hook is therefore entered at most ONCE
            // per (node, install) — a latch here could suppress nothing, and
            // saying it does would misdescribe the seam.
            tracing::error!(
                node_id = %id,
                step,
                after_fire = completed,
                error = %msg,
                "the replay injection hook PANICKED at an intra-step pause — the \
                 recording's foreign frames for this slot may not have been injected (a \
                 panic MID-publish can leave some of them on the wire; counted by \
                 replay_hook_panics, positioned by unconsumed_replay_pauses). The burst \
                 continues; a panicking hook must not unwind out of step()."
            );
            return;
        }
        node.replay_pauses.cursor = cursor + 1;
    }

    /// The TICK half of `evaluate_node` — perform the fire
    /// a prior `decide_node` decided on (its [`FireKind`]). `Single` fires once;
    /// `Period` runs the per-catch-up `fire_node` loop, re-checking
    /// `run_pre_fire_check` BEFORE each fire (the block-overflow clamp) and
    /// re-reading the panic circuit breaker AFTER each one. That per-catch-up
    /// re-check's counter side-effect runs HERE — identical position, count,
    /// and argument (`current_time_ns`) to the pre-split fused form.
    ///
    /// The two breaks carry DIFFERENT `next_fire_ns` semantics and the loop
    /// body says why: a pre-fire defer REWINDS (the catch-up resumes once the
    /// consumer drains), a disabled break does NOT (`reset_node` promises a
    /// re-enabled node does not catch up on fires missed while disabled).
    // 8 args: see `fire_node` — same funnel, same borrow rationale.
    #[allow(clippy::too_many_arguments)]
    fn tick_node(
        id: &str,
        node: &mut ScheduledNode,
        kind: FireKind,
        global_level: usize,
        step: u64,
        trace: &mut VecDeque<TraceEntry>,
        max_trace_entries: Option<usize>,
        entries_appended: &mut u64,
        mut ring: Option<&mut TraceRingHook>,
    ) {
        match kind {
            FireKind::Single { fire_time_ns } => {
                Self::fire_node(
                    id,
                    node,
                    fire_time_ns,
                    global_level,
                    step,
                    trace,
                    max_trace_entries,
                    entries_appended,
                    ring,
                );
            }
            // The Data burst runs through the SHARED loop (see
            // `tick_data_burst`), fed the same `RingTraceSink` `fire_node`
            // builds — so the serial and parallel paths cannot drift.
            FireKind::Data {
                fire_time_ns,
                fire_count,
            } => {
                let mut sink = RingTraceSink {
                    trace,
                    max_trace_entries,
                    entries_appended,
                    ring,
                };
                Self::tick_data_burst(
                    id,
                    node,
                    fire_time_ns,
                    fire_count,
                    global_level,
                    step,
                    &mut sink,
                );
            }
            // The Sync twin, mirroring the Data burst seam for seam.
            FireKind::Sync {
                fire_time_ns,
                max_sets,
            } => {
                let mut sink = RingTraceSink {
                    trace,
                    max_trace_entries,
                    entries_appended,
                    ring,
                };
                Self::tick_sync_burst(
                    id,
                    node,
                    fire_time_ns,
                    max_sets,
                    global_level,
                    step,
                    &mut sink,
                );
            }
            // The trace-driven burst, through the SAME
            // `RingTraceSink` the other bursts build.
            FireKind::Replay {
                first_fire_ns,
                fire_count,
                interval_ns,
            } => {
                let mut sink = RingTraceSink {
                    trace,
                    max_trace_entries,
                    entries_appended,
                    ring,
                };
                Self::tick_replay_burst(
                    id,
                    node,
                    first_fire_ns,
                    fire_count,
                    interval_ns,
                    global_level,
                    step,
                    &mut sink,
                );
            }
            FireKind::Period {
                first_fire_ns,
                fire_count,
                interval_ns,
                current_time_ns,
            } => {
                debug_assert!(
                    fire_count > 0,
                    "tick_node: Period FireKind must carry fire_count > 0 (decide_node returns None on empty)"
                );
                // hot-path-alloc-ok: not a heap allocation: cloning an `Arc`/`Option<Arc>` is a
                // refcount bump (`pre_fire_check` is an `Option<Arc<dyn Fn>>`)
                let pre_fire_check = node.pre_fire_check.clone();
                // Reconstruct the catch-up fire times arithmetically
                // via a RUNNING add (`fire_time += interval_ns`) — the same
                // SEQUENCE the old collected `Vec` carried, and the same one
                // `tick_node_into` reconstructs. Do NOT use an `i*interval`
                // multiply: the two agree on every reachable burst and the
                // running add is the shipped sequence, so a second spelling buys
                // nothing and would have to be proved equal.
                //
                // The OVERFLOW half of that note used to
                // claim byte-parity with `decide_node`'s own `*next_fire +=
                // interval_ns`. That add is gone — `decide_node` advances
                // through `catchup_clamp::period_advance`, which SATURATES — so
                // there is no longer a wrapping counterpart to match. What
                // bounds this loop is `fire_count`, which comes from that same
                // saturating advance, so a burst long enough to overflow the
                // reconstruction would have to start from a deadline the advance
                // had already clamped.
                let mut fire_time = first_fire_ns;
                for _ in 0..fire_count {
                    // Re-check the block pre-fire BEFORE each
                    // catch-up fire. If a downstream block consumer's queue
                    // filled mid-burst, BREAK and resume the remaining catch-up
                    // on a later step once the consumer drains. This keeps the
                    // outstanding mirror in lock-step with actual publishes
                    // (no jump-by-N desync) and prevents iceoryx2 overflow.
                    // The closure bumps `block_fires_deferred_count` on defer
                    // (correct: each deferred catch-up attempt is one defer).
                    if let Some(check) = &pre_fire_check {
                        if Self::run_pre_fire_check(id, check, current_time_ns) {
                            // `fire_time` is the EARLIEST un-fired reconstructed
                            // interval — set `next_fire_ns` directly to it so
                            // the catch-up resumes from here. (Do not
                            // arithmetic-rewind from the
                            // post-collection `next_fire_ns` — the max_catchup
                            // cap-skip loop in `decide_node` may have
                            // over-advanced it past the burst, which would land
                            // next_fire in the stale region and silently DROP
                            // these un-fired intervals.) Any intervals the
                            // `max_catchup` cap already skipped stay dropped
                            // (that is the cap's intent); the un-fired
                            // reconstructed intervals re-fire from here.
                            if let Some(next_fire) = node.next_fire_ns.as_mut() {
                                *next_fire = fire_time;
                            }
                            tracing::trace!(
                                node_id = %id,
                                resume_at_ns = fire_time,
                                "block pre-fire deferred mid Period catch-up; next_fire set to the \
                                 earliest un-fired collected interval (overflow clamp, mirror in sync)"
                            );
                            break;
                        }
                    }
                    Self::fire_node(
                        id,
                        node,
                        fire_time,
                        global_level,
                        step,
                        trace,
                        max_trace_entries,
                        entries_appended,
                        // Reborrow: the catch-up burst fires N times through
                        // the one Option<&mut _> (never moved out).
                        ring.as_deref_mut(),
                    );
                    fire_time += interval_ns;
                    // The panic circuit breaker: `fire_node` may have just
                    // opened it (`MAX_CONSECUTIVE_PANICS` consecutive panics),
                    // and the only `disabled` gate on this path is in
                    // `decide_node`, which ran BEFORE this loop. Without this
                    // re-check an always-panicking callback runs the REST of
                    // the catch-up burst inside ONE step, every fire of it
                    // after the breaker opened.
                    //
                    // DELIBERATELY NO REWIND, in contrast to the block-defer
                    // arm above: that one rewinds `next_fire_ns` to the
                    // earliest un-fired interval so the catch-up RESUMES once
                    // the consumer drains. A disabled node must NOT resume —
                    // `reset_node`'s documented contract is that a re-enabled
                    // node "doesn't catch up on fires missed while disabled" —
                    // so leaving `next_fire_ns` where `decide_node` advanced it
                    // (past the whole burst) IS that no-catch-up behaviour.
                    if node.disabled {
                        break;
                    }
                }
            }
        }
    }

    /// The COLLECT mirror of [`Self::tick_node`]
    /// — fire the decided [`FireKind`] into the CALLER-SUPPLIED `Vec<TraceEntry>`
    /// `sink` (via [`Self::fire_node_into`], reusing the `impl TraceSink for
    /// Vec<TraceEntry>`), instead of pushing into a shared
    /// `&mut VecDeque<TraceEntry>` ring buffer. No `max_trace_entries` param: the
    /// ring-buffer truncation is applied ONCE at the merge site
    /// (`tick_decided_parallel` pass 3), AFTER the per-node fragments are drained
    /// in decision order — so the merged trace is byte-identical to the serial
    /// `tick_node` order. The Period catch-up loop is identical to `tick_node`:
    /// arithmetic running-add reconstruction of the fire times, the per-fire
    /// `run_pre_fire_check` re-check with its `next_fire_ns`
    /// rewind-to-earliest-un-fired + break, and the post-fire
    /// panic-circuit-breaker re-check that breaks WITHOUT rewinding (see
    /// `tick_node` for why the two differ).
    ///
    /// Touches ONLY `&mut node` + the caller's `sink` (per-node-disjoint on the
    /// parallel path — the sink is the node's own reused `trace_fragment`) — no
    /// shared mutable state — so each level node can run this concurrently on a
    /// rayon worker with its own owned `id` / `kind`.
    ///
    /// Writes into the caller's reusable `sink` (the node's
    /// `trace_fragment`, mem::take'd + cleared by [`Self::fire_into_fragment`])
    /// rather than returning a fresh `Vec` — the per-fire allocation the old
    /// `tick_node_collect` paid (`let mut sink = Vec::new()`) is gone; the
    /// fragment's capacity is reused across steps.
    fn tick_node_into(
        id: &str,
        node: &mut ScheduledNode,
        kind: FireKind,
        global_level: usize,
        step: u64,
        sink: &mut Vec<TraceEntry>,
    ) {
        match kind {
            FireKind::Single { fire_time_ns } => {
                Self::fire_node_into(id, node, fire_time_ns, global_level, step, sink);
            }
            // Same shared burst loop as the serial path, over this
            // node's own `Vec` fragment. The refill hook locks the node's mutex
            // exactly as the fire callback does, and the two are strictly
            // SEQUENTIAL here (fire, unlock, refill, unlock, fire …), so a rayon
            // worker holding `&mut ScheduledNode` never holds two locks at once.
            FireKind::Data {
                fire_time_ns,
                fire_count,
            } => {
                Self::tick_data_burst(id, node, fire_time_ns, fire_count, global_level, step, sink);
            }
            // The Sync twin. Same sequential lock discipline — the
            // align pass locks the node's mutex exactly as the fire callback
            // does, and the two never overlap (fire, unlock, align, unlock, …).
            FireKind::Sync {
                fire_time_ns,
                max_sets,
            } => {
                Self::tick_sync_burst(id, node, fire_time_ns, max_sets, global_level, step, sink);
            }
            // Same shared burst loop as the serial path, over this
            // node's own `Vec` fragment.
            FireKind::Replay {
                first_fire_ns,
                fire_count,
                interval_ns,
            } => {
                Self::tick_replay_burst(
                    id,
                    node,
                    first_fire_ns,
                    fire_count,
                    interval_ns,
                    global_level,
                    step,
                    sink,
                );
            }
            FireKind::Period {
                first_fire_ns,
                fire_count,
                interval_ns,
                current_time_ns,
            } => {
                debug_assert!(
                    fire_count > 0,
                    "tick_node_into: Period FireKind must carry fire_count > 0 (decide_node returns None on empty)"
                );
                // hot-path-alloc-ok: not a heap allocation: cloning an `Arc`/`Option<Arc>` is a
                // refcount bump (`pre_fire_check` is an `Option<Arc<dyn Fn>>`)
                let pre_fire_check = node.pre_fire_check.clone();
                // Reconstruct the catch-up fire times arithmetically
                // via a RUNNING add — the same SEQUENCE the old collected `Vec`
                // carried, and the same one `tick_node` reconstructs. Not an
                // `i*interval` multiply, for the reason stated there; the
                // overflow-parity claim that note used to carry is retired with
                // `decide_node`'s wrapping add (see `tick_node`).
                let mut fire_time = first_fire_ns;
                for _ in 0..fire_count {
                    // Mirror of `tick_node`'s catch-up re-check: re-evaluate the
                    // block pre-fire BEFORE each catch-up fire and BREAK (leaving
                    // `next_fire_ns` at the earliest un-fired reconstructed
                    // interval) the moment the predicate defers again. Same
                    // count/position/arg as the serial form.
                    if let Some(check) = &pre_fire_check {
                        if Self::run_pre_fire_check(id, check, current_time_ns) {
                            if let Some(next_fire) = node.next_fire_ns.as_mut() {
                                *next_fire = fire_time;
                            }
                            tracing::trace!(
                                node_id = %id,
                                resume_at_ns = fire_time,
                                "block pre-fire deferred mid Period catch-up; next_fire set to the \
                                 earliest un-fired collected interval (overflow clamp, mirror in sync)"
                            );
                            break;
                        }
                    }
                    Self::fire_node_into(id, node, fire_time, global_level, step, sink);
                    fire_time += interval_ns;
                    // Mirror of `tick_node`'s panic-circuit-breaker re-check:
                    // `fire_node_into` may have just opened it, and the only
                    // `disabled` gate on this path is in `decide_node`, which
                    // ran BEFORE this loop. DELIBERATELY NO REWIND (unlike the
                    // block-defer arm above, which rewinds so the catch-up
                    // RESUMES): `reset_node` promises a re-enabled node does
                    // not catch up on fires missed while disabled, so leaving
                    // `next_fire_ns` where `decide_node` advanced it IS that
                    // behaviour. See `tick_node` for the full rationale.
                    if node.disabled {
                        break;
                    }
                }
            }
        }
    }

    /// Fire ONE decided node into its OWN reusable `trace_fragment`,
    /// keeping the fragment zero-alloc at steady state. `mem::take` swaps the
    /// fragment out of `node` (replacing it with an empty `Vec` — no alloc) so
    /// the `&mut node` and `&mut frag` borrows do not overlap inside
    /// [`Self::tick_node_into`]; `clear()` resets length while retaining
    /// capacity; the fire pushes this node's `TraceEntry`s; then the
    /// capacity-retaining fragment is written back into `node`. The level merge
    /// (`tick_decided_parallel` pass 3) drains it in decision order.
    ///
    /// Shared by the serial-gated PASS 1 and BOTH drivers of PASS 2 (the narrow
    /// serial loop and the rayon-parallel `par_values_mut`) so the
    /// take/clear/fire/write-back lives in ONE place. An associated fn over
    /// `&mut ScheduledNode` + `&DecisionSlot` + two Copy scalars `global_level`
    /// (the deterministic GLOBAL DAG level) and `step` (the 0-based logical-step
    /// index) — both stamped onto every `TraceEntry`, both constant for the
    /// level — captures nothing, so it is trivially `Send` for the rayon closure.
    fn fire_into_fragment(
        node: &mut ScheduledNode,
        slot: &DecisionSlot,
        global_level: usize,
        step: u64,
    ) {
        let mut frag = std::mem::take(&mut node.trace_fragment);
        frag.clear();
        Self::tick_node_into(
            &slot.node_id,
            node,
            slot.kind,
            global_level,
            step,
            &mut frag,
        );
        node.trace_fragment = frag;
    }

    /// The NON-serial-gated REST count at/above which a multi-fire
    /// level's REST fires via rayon (`par_values_mut`) instead of serially on the
    /// calling thread (keyed on the REST count — the nodes that actually fan out
    /// — not the total level width). PURE PERFORMANCE + injector-alloc knob with ZERO
    /// determinism effect: below it the REST fires serially → strictly
    /// zero-alloc (no crossbeam-injector residual); at/above it the REST fans
    /// across the build-time pool, accepting rayon's amortized ~1/63
    /// crossbeam-injector residual for the parallel speed-up on genuinely wide
    /// levels. The merged trace is byte-identical either way (pass 3 drains
    /// `decisions` in decision order regardless of which driver fired the REST).
    pub(crate) const PARALLEL_FIRE_THRESHOLD: usize = 8;

    /// How many levels the shared-topic `RestWalk::InsertionOrder`
    /// constraint alone took off the rayon path (see the field docs). A
    /// wide-path test asserts this against the step count it drove, so a
    /// constraint that went inert fails deterministically rather than by
    /// scheduling luck.
    pub fn forced_serial_rest_walks(&self) -> u64 {
        self.forced_serial_rest_walks
    }

    /// The PURE routing verdict for PASS 2 of [`Self::tick_decided_parallel`]:
    /// which driver walks a level's REST, and why. FIRST MATCH WINS — the `if`
    /// order below IS the precedence the counter's "alone" is defined by:
    ///
    /// 1. size/pool ⇒ [`RestDriver::SerialBySize`];
    /// 2. armed replay pauses ⇒ [`RestDriver::SerialByReplayPauses`];
    /// 3. the shared-topic constraint ⇒ [`RestDriver::SerialByConstraint`];
    /// 4. otherwise ⇒ [`RestDriver::Parallel`].
    ///
    /// No `&self`, no clock, no transport: the verdict is a function of the
    /// four inputs and nothing else, which is what lets the hand-oracle table
    /// in `mod tests` pin every cell.
    pub(crate) fn rest_driver(
        rest_fire_count: usize,
        pool_threads: usize,
        replay_pauses_armed: bool,
        rest_walk: RestWalk,
    ) -> RestDriver {
        // The size/pool gate is a PURE perf/alloc knob (no determinism
        // effect): below the threshold (or a single-thread pool) the REST
        // fires serially — strictly zero-alloc / no crossbeam injector; at or
        // above it the REST fans across the pool. Keyed on the non-serial
        // REST count, so a level whose REST is small never pays rayon
        // dispatch even if it has many serial-gated nodes.
        if rest_fire_count < Self::PARALLEL_FIRE_THRESHOLD || pool_threads <= 1 {
            return RestDriver::SerialBySize;
        }
        // The replay-pause arm, and it is NOT a perf knob: while intra-step
        // pauses are armed, a fire can call out to the engine's injection hook
        // (see `consume_intra_step_pause`), and the ORDER of those calls
        // across nodes IS the serve order of the frames they inject. Rayon's
        // `for_each` fans the REST across workers, so the wide path would make
        // that order nondeterministic — a Replay=Live violation the trace
        // could not even show, because PASS 3 merges in decision order and is
        // byte-identical either way. Forcing the narrow walk (the SAME
        // insertion-order loop the sub-threshold path already takes, and the
        // one whose trace-identity invariant A already covers) makes the hook
        // order deterministic. Zero effect on any live path: nothing but a
        // replay engine ever arms pauses.
        //
        // The input is the ARMED FLAG, not `!replay_paused_nodes.is_empty()`.
        // Strictly conservative — an armed-but-empty install also takes the
        // serial walk, which costs nothing on a replay-only path and changes
        // no trace (invariant A) — and it decouples a DETERMINISM guard from
        // the `replay_paused_nodes` ⟺ non-empty-`after_fires` invariant. That
        // invariant is maintained by hand across `set_replay_intra_step_pauses`
        // (which retires the previous install and re-extends the list),
        // `clear_replay_intra_step_pauses` (which empties both) and
        // `remove_node` (whose `shift_remove` re-points every later index, so
        // it RE-DERIVES the list from the map), so a slip there would silently
        // restore the rayon fan-out; the flag has two mutation sites (the
        // constructors merely initialize it) and cannot drift the same way.
        //
        // Coupling worth naming, because two fixes meet here: a REFUSED
        // install still ARMS the seam (the "armed and empty" contract), so a
        // caller that takes the `Err` and abandons the replay leaves this
        // scheduler on the serial rest walk until
        // `clear_replay_intra_step_pauses`. Accepted: the walk is record-only
        // (invariant A — same fires, same order, same trace), and nothing but
        // a replay engine ever arms pauses, so no live path can reach the
        // state at all.
        //
        // It ranks ABOVE the constraint on purpose: a level that is serial for
        // the replay seam is serial in insertion order already, so the
        // constraint changed nothing there and must not be credited.
        if replay_pauses_armed {
            return RestDriver::SerialByReplayPauses;
        }
        // A level holding >= 2 non-serial-gated producers
        // of ONE `multi_publisher_topics` topic MUST walk its REST in
        // insertion order. Those producers publish into ONE shared iceoryx2
        // FIFO from inside their ticks (`OutputProxy::Drop`), and nothing in
        // the parallel driver orders two rayon workers' publishes — the
        // interleave would be a scheduling artifact the read log and the
        // recorded bag inherit while the fire trace (merged in decision order
        // in PASS 3) cannot see it, so replay could re-fire the pair into a
        // different order on healthy code (Principle #7). The build decides
        // the verdict per level (each level's `LevelPlan::rest_walk`, over the
        // non-block subset — an all-`block` shared topic never reaches this
        // REST: `evaluate_nodes_fused` takes it); this seam only honours it, as a driver
        // choice on the SAME routing seam rather than as PASS 1 membership, so
        // it changes WHEN the REST fires and never which pass a node fires in.
        if rest_walk == RestWalk::InsertionOrder {
            return RestDriver::SerialByConstraint;
        }
        RestDriver::Parallel
    }

    /// FIRE the nodes a prior
    /// [`Self::decide_fires`] decided, WITHIN-LEVEL, then merge their trace
    /// fragments into `self.trace` in decision order (byte-identical to the
    /// serial [`Self::tick_decided`]).
    ///
    /// `serial_ids` lists node ids the level executor must keep OFF the rayon
    /// path because their step-boundary input snapshot is a no-op (cdylib /
    /// closure nodes with non-trigger inputs — `!performs_input_snapshot()`):
    /// without a real freeze, a same-level producer's parallel publish could be
    /// observed mid-level, breaking replay = live. They fire FIRST, on the
    /// CALLING thread (PASS 1), so they read the pre-REST-publish state; the REST
    /// fires in PASS 2.
    ///
    /// `rest_walk` is the build's per-level verdict on HOW PASS 2 may walk the
    /// REST ([`RestWalk`]): `InsertionOrder` forces the serial insertion-order
    /// walk regardless of size (the level carries two producers of
    /// one shared topic; the constraint is stated at the routing site below).
    ///
    /// # Passes (lifetime-free index-keyed scratch — no per-step HashMap)
    ///
    /// 1. Fast-path: a level of ≤1 decided fire keeps the exact 4a serial
    ///    [`Self::tick_decided`] hot path (zero rayon overhead, zero scratch —
    ///    narrow moat graphs never pay the fan-out cost; also keeps
    ///    `tick_decided` non-dead).
    /// 2. For ≥2 fires, fill [`Self::decision_slots`] (reused, sized to
    ///    `nodes.len()`, addressed by each firing node's stable insertion `idx`)
    ///    with one [`DecisionSlot`] per fire. `Arc::clone` of the node id is a
    ///    refcount bump (zero heap); `clear()` + `resize_with` reuses capacity →
    ///    zero alloc at steady state.
    /// 3. PASS 1 — serial-gated nodes fire FIRST on the calling thread (only if
    ///    any were gated), so they observe the pre-REST-publish state (invariant
    ///    B). topology.rs makes non-trigger inputs NON-edges, so a serial-gated
    ///    node CAN be same-level as its producer → this ordering is load-bearing.
    /// 4. PASS 2 — the REST (non-serial-gated). [`Self::rest_driver`] picks the
    ///    driver; three causes make it SERIAL — in insertion order on the
    ///    calling thread, zero injector alloc — ranked first-match-wins: the
    ///    level is narrow (`< THRESHOLD` OR a single-thread pool), intra-step
    ///    replay pauses are ARMED, or `rest_walk` is
    ///    [`RestWalk::InsertionOrder`]. Only the THIRD bumps
    ///    `forced_serial_rest_walks` — it is the one that changed a routing the
    ///    other two would have sent wide. Otherwise
    ///    `pool.install(par_values_mut().enumerate().for_each(..))` fans the
    ///    disjoint `&mut ScheduledNode` across the pool, capturing only the
    ///    shared `&decision_slots` (Sync) — no shared mutable state (invariant G).
    /// 5. PASS 3 — drain `decisions` (already pos-ordered) and, per decision,
    ///    drain that node's `trace_fragment` into `self.trace` applying the
    ///    `max_trace_entries` per-entry ring truncation. Decision-order drain ⇒
    ///    BYTE-IDENTICAL to serial `tick_decided` (invariant A).
    ///
    /// DRAINS the caller's reusable `decisions` buffer in place (the
    /// fast-path delegation to `tick_decided` AND the pass 3 `drain(..)` empty it
    /// while retaining capacity) so the runtime can hand it back via
    /// `return_decisions` for reuse next level/step.
    pub(crate) fn tick_decided_parallel(
        &mut self,
        decisions: &mut Vec<FireDecision>,
        global_level: usize,
        step: u64,
        pool: &rayon::ThreadPool,
        serial_ids: &HashSet<String>,
        rest_walk: RestWalk,
    ) {
        // Fast-path: 0 or 1 decided fire → serial. UNCHANGED — the moat chain
        // (single-fire levels) stays zero-alloc and never touches the scratch /
        // fragment machinery below.
        if decisions.len() <= 1 {
            return self.tick_decided(decisions, global_level, step);
        }

        // Build the lifetime-free index-keyed scratch (NO HashMap). Cleared +
        // resized (not reallocated) → zero alloc once `n` stabilizes. Each
        // `DecisionSlot` carries the node id (refcount bump), fire kind (Copy),
        // and resolved serial-gating flag (one `serial_ids` lookup per fire).
        let n = self.nodes.len();
        self.decision_slots.clear();
        self.decision_slots.resize_with(n, || None);
        let mut any_serial = false;
        // The non-serial-gated REST count — the nodes that would actually fan
        // out to rayon — drives the parallel/serial threshold below (NOT the
        // total `decisions.len()`: serial-gated nodes always fire serially in
        // PASS 1, so a level of mostly-serial-gated nodes should not pay rayon
        // dispatch for a tiny REST).
        let mut rest_fire_count = 0usize;
        // Empty `Vec::new()` allocates nothing; at steady state there are no
        // orphans so this never grows (cold path only).
        // hot-path-alloc-ok: an empty `Vec::new()` allocates nothing, and it only ever grows on the
        // idx/IndexMap-desync branch below — unreachable while the IndexMap is not mutated mid-step
        let mut orphan_ids: Vec<Arc<str>> = Vec::new();
        for d in decisions.iter() {
            if d.idx >= n {
                // Out-of-range idx = idx/IndexMap desync (the IndexMap was
                // mutated between decide_fires and here). Collect for the loud
                // guard below; its fire is dropped (never silently — see (D)).
                orphan_ids.push(Arc::clone(&d.node_id));
                continue;
            }
            let serial = serial_ids.contains(d.node_id.as_ref());
            any_serial |= serial;
            if !serial {
                rest_fire_count += 1;
            }
            self.decision_slots[d.idx] = Some(DecisionSlot {
                node_id: Arc::clone(&d.node_id),
                kind: d.kind,
                serial,
            });
        }

        // ONE verdict, ranked (the size/pool knob, then the armed replay seam,
        // then the shared-topic constraint): see `rest_driver`, where each
        // cause is argued at its own arm.
        let driver = Self::rest_driver(
            rest_fire_count,
            pool.current_num_threads(),
            self.replay_pauses_armed,
            rest_walk,
        );
        if driver == RestDriver::SerialByConstraint {
            // Observable (Principle #3): only a level the constraint ALONE took
            // off the rayon path counts — a level already narrow by size or by
            // an armed replay seam ranks above it in
            // `rest_driver` and lands on a different variant.
            self.forced_serial_rest_walks += 1;
        }

        // (D) Loud orphan guard — symmetry with the `decide_fires` /
        // `evaluate_nodes_fused` desync guards: an out-of-range decided idx is a
        // SILENT dropped fire in release otherwise. Unreachable while the
        // IndexMap is not mutated mid-step (the invariant), but a future mutation
        // must surface loudly (error log) + fail in debug.
        if !orphan_ids.is_empty() {
            // hot-path-alloc-ok: cold: the LOUD orphan guard, reached only when the decided
            // set names a node the IndexMap no longer holds — which the comment above records
            // as unreachable while the map is not mutated mid-step. The enclosing fn IS hot, so
            // this is the LINE form deliberately; the `is_empty` check above is the gate.
            let orphaned: Vec<&str> = orphan_ids.iter().map(|a| a.as_ref()).collect();
            tracing::error!(
                orphaned_node_ids = ?orphaned,
                orphaned = orphaned.len(),
                "tick_decided_parallel: decided node(s) out of range — idx/IndexMap desync \
                 between decide_fires and tick_decided_parallel; their fire is DROPPED"
            );
        }
        debug_assert!(
            orphan_ids.is_empty(),
            "every decided idx must be < nodes.len() — idx/IndexMap desync between decide_fires and tick_decided_parallel"
        );

        // PASS 1 — serial-gated nodes FIRST, on the calling thread, serially.
        // Preserves the "serial nodes fire before any parallel producer
        // publishes" invariant (B). `&self.decision_slots` (shared) + `&mut
        // self.nodes` (the values_mut walk) are disjoint field borrows.
        if any_serial {
            let scratch = &self.decision_slots;
            for (i, node) in self.nodes.values_mut().enumerate() {
                if let Some(slot) = &scratch[i] {
                    if slot.serial {
                        // (E) idx/node_id desync guard (mirrors serial
                        // tick_decided): the node at insertion idx `i` must still
                        // BE the node decide_fires minted this decision for.
                        debug_assert_eq!(
                            node.node_id.as_ref(),
                            slot.node_id.as_ref(),
                            "tick_decided_parallel(serial pass): idx/node_id desync at {i}"
                        );
                        Self::fire_into_fragment(node, slot, global_level, step);
                    }
                }
            }
        }

        // PASS 2 — the REST (non-serial-gated firing nodes). Same fire body as
        // PASS 1 but skipping serial slots (already fired). The driver is the
        // verdict above, matched EXHAUSTIVELY: a new `RestDriver` variant must
        // be classified here before it compiles — never defaulted.
        match driver {
            RestDriver::SerialBySize
            | RestDriver::SerialByReplayPauses
            | RestDriver::SerialByConstraint => {
                // Serial REST: no rayon → zero injector alloc — and the ONLY
                // driver whose publish order IS the insertion order, which is
                // what `RestWalk::InsertionOrder` buys.
                let scratch = &self.decision_slots;
                for (i, node) in self.nodes.values_mut().enumerate() {
                    if let Some(slot) = &scratch[i] {
                        if slot.serial {
                            continue;
                        }
                        debug_assert_eq!(
                            node.node_id.as_ref(),
                            slot.node_id.as_ref(),
                            "tick_decided_parallel({driver:?} serial rest pass): idx/node_id desync at {i}"
                        );
                        Self::fire_into_fragment(node, slot, global_level, step);
                    }
                }
            }
            RestDriver::Parallel => {
                // Wide REST: fan the disjoint `&mut ScheduledNode` across the
                // build-time pool. `par_values_mut().enumerate()` yields one
                // unique `&mut` per node (Send); the `for_each` closure
                // captures ONLY `scratch` (`&Vec<Option<DecisionSlot>>`, Sync)
                // — no shared mutable state (invariant G). `scratch` (shared)
                // + `nodes` (the `&mut` consumed by the parallel iterator) are
                // disjoint field borrows.
                let scratch = &self.decision_slots;
                let nodes = &mut self.nodes;
                pool.install(|| {
                    nodes.par_values_mut().enumerate().for_each(|(i, node)| {
                        if let Some(slot) = &scratch[i] {
                            if slot.serial {
                                return;
                            }
                            debug_assert_eq!(
                                node.node_id.as_ref(),
                                slot.node_id.as_ref(),
                                "tick_decided_parallel(parallel rest pass): idx/node_id desync at {i}"
                            );
                            Self::fire_into_fragment(node, slot, global_level, step);
                        }
                    });
                });
            }
        }

        // PASS 3 — merge into `self.trace` in DECISION (pos) ORDER by draining
        // `decisions` (ALREADY pos-ordered: decide_fires pushes in level/graph
        // order). Apply the SAME per-entry `max_trace_entries` ring truncation
        // the serial `fire_node` / `RingTraceSink` applies, so the merged trace
        // is BYTE-IDENTICAL to serial `tick_decided` (invariant A). `self.nodes`
        // and `self.trace` are disjoint field borrows. `drain(..)` empties
        // `decisions` (retaining capacity) for `return_decisions` reuse (F).
        // The parallel path now funnels through the SAME
        // `RingTraceSink::push_entry` choke point as the serial path (one
        // sink over disjoint field borrows), so the `entries_appended` bump,
        // the `max_trace_entries` per-entry truncation, AND the recording
        // trace-ring push happen at ONE site for both paths. Runs on the
        // step-calling thread, AFTER the rayon join returned (SPSC-safe: the
        // producer is never touched from a worker).
        let mut sink = RingTraceSink {
            trace: &mut self.trace,
            max_trace_entries: self.max_trace_entries,
            entries_appended: &mut self.entries_appended,
            ring: self.trace_ring.as_mut(),
        };
        let nodes = &mut self.nodes;
        for d in decisions.drain(..) {
            if let Some((_, node)) = nodes.get_index_mut(d.idx) {
                for e in node.trace_fragment.drain(..) {
                    sink.push_entry(e);
                }
            }
            // Out-of-range idx already reported by the (D) orphan guard above;
            // skip gracefully (its fragment was never written).
        }
    }

    /// Maximum consecutive panics before a node is disabled.
    const MAX_CONSECUTIVE_PANICS: u32 = 3;

    /// Fire a node's callback and update observable state.
    ///
    /// Uses `catch_unwind` for panic safety. After `MAX_CONSECUTIVE_PANICS`
    /// consecutive panics, the node is disabled and no longer fires.
    ///
    /// Thin wrapper over [`Self::fire_node_into`] for the
    /// SERIAL path — fires the ONE shared fire body straight into `trace` via a
    /// [`RingTraceSink`], applying the `max_trace_entries` ring-buffer
    /// truncation per entry, so the FLAT `Scheduler::step` / `evaluate_one` path
    /// stays BYTE-IDENTICAL to its pre-4b form (same trace contents, same
    /// per-entry pop-front cadence, same order).
    ///
    /// ZERO allocation per serial fire — `fire_node_into` pushes
    /// directly into the shared `VecDeque` through the sink (the old form
    /// collected into a per-fire local `Vec` then drained it; that `Vec` is
    /// gone). The parallel level executor uses the [`Self::tick_node_into`]
    /// `Vec`-sink path instead (firing into each node's reusable
    /// `trace_fragment`), merging all fragments once in decision order at the
    /// pass 3 merge.
    // 8 args: the serial fire funnel threads the trace ring + its cap + the
    // monotonic append counter together; a params struct would obscure the
    // disjoint-field-borrow pattern the callers rely on.
    #[allow(clippy::too_many_arguments)]
    fn fire_node(
        id: &str,
        node: &mut ScheduledNode,
        fire_time_ns: u64,
        global_level: usize,
        step: u64,
        trace: &mut VecDeque<TraceEntry>,
        max_trace_entries: Option<usize>,
        entries_appended: &mut u64,
        ring: Option<&mut TraceRingHook>,
    ) {
        let mut sink = RingTraceSink {
            trace,
            max_trace_entries,
            entries_appended,
            ring,
        };
        Self::fire_node_into(id, node, fire_time_ns, global_level, step, &mut sink);
    }

    /// The ONE fire body — the EXACT fire steps
    /// (catch_unwind, `tick_within_ms` timing, consecutive-panic/disable
    /// bookkeeping, `fire_count` / `last_fire_ns` bumps) parameterised over a
    /// [`TraceSink`] so a SINGLE implementation serves BOTH paths:
    ///
    /// * SERIAL ([`Self::fire_node`]) passes a [`RingTraceSink`] that pushes
    ///   straight into the shared `&mut VecDeque<TraceEntry>` with the
    ///   `max_trace_entries` per-entry truncation — zero alloc, byte-identical
    ///   to the pre-4b inline tail.
    /// * PARALLEL ([`Self::tick_node_into`]) passes a `Vec<TraceEntry>` sink
    ///   (each node's reusable `trace_fragment`) that just appends; the
    ///   `max_trace_entries` truncation is applied ONCE later, at the
    ///   decision-ordered pass 3 merge in `tick_decided_parallel`, so each rayon
    ///   worker fires its disjoint `&mut ScheduledNode` into a per-node fragment
    ///   with NO shared mutable trace state.
    ///
    /// The ONLY shared mutable state `fire_node_into` touches is `sink`
    /// (caller-owned: the shared `VecDeque` on the serial path, a per-node
    /// `Vec` on the parallel path); everything else is on `&mut node`.
    fn fire_node_into<S: TraceSink>(
        id: &str,
        node: &mut ScheduledNode,
        fire_time_ns: u64,
        global_level: usize,
        step: u64,
        sink: &mut S,
    ) {
        // Measure tick callback elapsed wall
        // time when `tick_within_ns` is configured. We capture
        // `Instant::now()` outside `catch_unwind` so the timing
        // wraps the panic path too (a tick that panics still "took"
        // some wall time; the miss counter increments if it
        // exceeded the budget before panicking).
        // B-dur: capture wall start when EITHER the tick_within
        // budget OR duration recording is active — one Instant serves both.
        let tick_start =
            (node.tick_within_ns.is_some() || node.record_durations).then(std::time::Instant::now);

        // ENTRY: mark this node IN A TICK, before anything that can
        // fail to return. Placed HERE — after the `Instant` and BEFORE the
        // `catch_unwind` — because a mark taken after the callback describes only
        // ticks that came back, which is exactly the blindness this alarm exists to
        // remove: `tick_within_ms` already reports those and reports them better.
        //
        // The stamp is FLOORED at 1 ns. `0` is the not-in-a-tick sentinel, so a
        // fire genuinely stamped 0 (a `VirtualClock` at its origin) would read as
        // "not in a tick" while inside one — a silent wrong answer on precisely the
        // class this watches. The cost is that one unreachable-in-practice stamp
        // reports 1 ns instead of 0, and it is the only value the marker cannot
        // represent exactly; the alternative (a second word) doubles the per-fire
        // store count for a distinction nothing reads.
        node.in_tick_since_ns
            .store(fire_time_ns.max(1), Ordering::Release);
        // The CROSS-PROCESS half. `None` on every monolith and every test, so this
        // is a `None` test on a field already in cache — the same shape the
        // replay queue and the catch-up arm use for their detached paths.
        if let Some((page, slot)) = node.wedge.as_ref() {
            page.enter(*slot);
        }

        // REPLAY side: pop this fire's recorded discard verdict (in
        // fire order; Period catch-up pops one per fire). On the live/record
        // path the queue is empty ⇒ `None` ⇒ `false` ⇒ neither store below runs
        // (zero extra atomic ops on the hot path). When `true`, arm the
        // per-node replay-suppress flag so every publisher's `OutputProxy::Drop`
        // returns before `commit_sequence` — the byte-identical mirror of the
        // live discard.
        let replay_suppress_this_fire = node.replay_discard_queue.pop_front().unwrap_or(false);
        if replay_suppress_this_fire {
            node.replay_suppress
                .store(true, std::sync::atomic::Ordering::Release);
        }

        // RECORD side: snapshot the per-node discard signal BEFORE the
        // callback; a non-zero DELTA after means this fire committed 0 of its
        // loaned outputs (a pre-commit loan failure or the all-defer
        // discard). Atomic load only — no alloc.
        let discard_before = node
            .discard_signal
            .load(std::sync::atomic::Ordering::Acquire);

        // Wrap callback in catch_unwind to prevent panics from crashing the scheduler
        let callback = &mut node.callback;
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            (callback)();
        }));

        // REPLAY side: CLEAR the suppress flag now the callback (and
        // every proxy Drop it owned) has returned — BEFORE anything below can
        // early-exit, and it runs even on a caught panic (the panic is contained
        // by the `catch_unwind` above; control always reaches here). A leaked
        // flag would silently suppress the NEXT fire. No-op unless this fire was
        // armed above.
        if replay_suppress_this_fire {
            node.replay_suppress
                .store(false, std::sync::atomic::Ordering::Release);
        }

        // RECORD side: read the delta. The counter only increments, so
        // `!= before` ⇔ "≥1 output discarded this fire". Never a false positive
        // (would need 2^32 discards within one fire).
        let discarded = node
            .discard_signal
            .load(std::sync::atomic::Ordering::Acquire)
            != discard_before;

        // Tick `tick_within_ms` budget check + loud surface.
        //
        // This measures WALL elapsed only (via the
        // `Instant::now()` captured above). It does not additionally
        // report the thread-CPU elapsed (`clock::thread_cpu_ns`) on a
        // miss, so preemption ("10ms wall / 2ms CPU → the OS descheduled
        // me") is not distinguishable from genuinely slow code ("10ms wall
        // / 9.8ms CPU"). The `thread_cpu_ns()` primitive already exists;
        // the dual (wall + thread-CPU) report is not implemented.
        // B-dur: single elapsed read, reused for the budget check + duration_ns.
        let elapsed_ns = tick_start.map(|start| start.elapsed().as_nanos() as u64);

        // EXIT: the tick came back — clear the marker and close the
        // seq pair. Beside the `elapsed_ns` read for the same reason that read is
        // here: this is the first point after the callback that control ALWAYS
        // reaches, caught panic included (the panic is contained by the
        // `catch_unwind` above; the replay-suppress clear twenty lines up
        // rests on exactly this invariant and says so). Above the `match result`
        // below, so the panic arm's early bookkeeping cannot get between the tick
        // returning and the marker clearing.
        //
        // UNCONDITIONAL — not gated on `elapsed_ns`, which is `None` unless the
        // node declared `tick_within_ms` or B-dur recording is on. The whole point
        // is that a node with no declared budget is the one nothing else watches.
        node.in_tick_since_ns.store(0, Ordering::Release);
        if let Some((page, slot)) = node.wedge.as_ref() {
            page.exit(*slot);
        }

        if let (Some(within_ns), Some(elapsed_ns)) = (node.tick_within_ns, elapsed_ns) {
            if elapsed_ns > within_ns {
                node.tick_within_missed.fetch_add(1, Ordering::Release);
                tracing::warn!(
                    node_id = %id,
                    tick_within_ms = within_ns / 1_000_000,
                    elapsed_ms = elapsed_ns / 1_000_000,
                    "tick `tick_within_ms` exceeded (tick took longer than the execution \
                     budget) — `tick_within_missed` counter incremented"
                );
            }
        }

        match result {
            Ok(()) => {
                node.consecutive_panics = 0;
            }
            Err(panic_info) => {
                let msg = if let Some(s) = panic_info.downcast_ref::<&str>() {
                    // hot-path-alloc-ok: cold: renders a CAUGHT PANIC payload — the branch runs
                    // only when a node callback unwound
                    s.to_string()
                } else if let Some(s) = panic_info.downcast_ref::<String>() {
                    // hot-path-alloc-ok: cold: renders a CAUGHT PANIC payload — the branch runs
                    // only when a node callback unwound
                    s.clone()
                } else {
                    // hot-path-alloc-ok: cold: renders a CAUGHT PANIC payload — the branch runs
                    // only when a node callback unwound
                    "unknown panic".to_string()
                };
                tracing::error!(node_id = %id, error = %msg, "node callback panicked");

                node.consecutive_panics += 1;
                node.panic_count.fetch_add(1, Ordering::Release);
                // Node-death ledger mint SITE (a). The guard is `!node.disabled`
                // and NOT the `>=` below, because the `>=` is not self-latching:
                // every later fire of an already-disabled node that somehow
                // reached here would re-satisfy it and mint a second capture for
                // one condition. Guarding on the FLAG makes this fire once per
                // TRANSITION — and a transition really can happen twice, because
                // `reset_node` re-enables (so a reset plus three more panics is a
                // legitimate second death, which `TriggerKind::PanicDisable`'s own
                // doc gets wrong when it says "at most once per node per run").
                if node.consecutive_panics >= Self::MAX_CONSECUTIVE_PANICS && !node.disabled {
                    node.disabled = true;
                    tracing::error!(
                        node_id = %id,
                        consecutive_panics = node.consecutive_panics,
                        "node disabled after repeated panics"
                    );
                    // Touches ONLY the shared `Arc` — never `Scheduler` state —
                    // because the wide fire path drives this on a rayon worker.
                    // A no-op unless a live loop armed the ledger, which is the
                    // replay firewall.
                    node.node_death
                        .record(id, node_death::NodeDeathCause::DisabledAfterPanics);
                }
            }
        }

        node.fire_count.fetch_add(1, Ordering::Release);
        node.last_fire_ns.store(fire_time_ns, Ordering::Release);

        // Hand the entry to the caller's sink.
        // The SERIAL [`RingTraceSink`] pushes it straight into the shared ring
        // buffer applying the `max_trace_entries` truncation per entry; the
        // PARALLEL `Vec` sink just appends, deferring truncation to the
        // post-join merge — so the parallel path stays lock-free.
        sink.push_entry(TraceEntry {
            node_id: Arc::clone(&node.node_id),
            // Stamp the 0-based logical-step index (a Copy
            // scalar constant for the whole `step()`/`GraphRuntime::step` call —
            // `current_step()` on the flat path, threaded down from the runtime's
            // pre-level-loop read on the level path). INCLUDED in `PartialEq`/`Eq`
            // and the cross-process merge's PRIMARY sort key (separates logical
            // steps that `fire_time_ns` cannot — a Period sub-step catch-up burst
            // fires BEFORE the step's `current_time_ns`).
            step,
            fire_time_ns,
            // Stamp the deterministic GLOBAL DAG level the
            // node fired at (a Copy scalar threaded down from the runtime's level
            // loop; `0` on the flat `Scheduler::step` path). INCLUDED in
            // `PartialEq`/`Eq` so per-process traces merge back into the global
            // fire sequence — contrast `duration_ns` (wall time, excluded).
            global_level,
            // B-dur: `duration_ns` is gated on `record_durations` SPECIFICALLY,
            // not on whether `elapsed_ns` was measured. A `tick_within_ms` node
            // shares the same `Instant` for its budget check, so `elapsed_ns`
            // can be `Some` while B-dur recording is off — but the field must
            // mean exactly "B-dur recording was on", so we emit 0 in that case.
            // Keeps the documented "0 unless recording is on" contract crisp.
            duration_ns: if node.record_durations {
                elapsed_ns.unwrap_or(0)
            } else {
                0
            },
            // The discard verdict from the signal delta above.
            // EXCLUDED from `TraceEntry::PartialEq` (a data-frame annotation),
            // folded into the ring record by `push_fire` as the discard bit.
            discarded,
        });
    }

    // `check_sync` is GONE. It answered ONE question — "are all
    // trigger inputs present and within the window?" — and the fire then
    // CLEARED every stamp, which is what collapsed a burst of aligned sets into
    // one fire on the freshest members. Its completeness and window semantics
    // are preserved EXACTLY inside
    // [`crate::scheduler::sync_match::next_sync_step`] (steps 1 and 2, `>` not
    // `>=`, unbounded skipping the span test), which is where they now live
    // alongside the membership choice they were never able to make.
}

/// The Sync FIRE predicate, callable without a [`Scheduler`].
///
/// Two rules, and nothing else:
///
/// 1. every declared trigger input must have a stamp;
/// 2. bounded sync only: `max(stamps) - min(stamps) <= window` — **inclusive**
///    (a node whose spread EQUALS the window fires).
///
/// Unbounded sync (`window_ns == None`) applies rule 1 alone.
///
/// **The spread is over the WIRE STAMPS, never a consumer's clock.** A stamp is
/// the PRODUCING side's gating clock at loan time; a consumer's own `now` only
/// decides WHEN the check runs. Computing the spread against it is a
/// clock-domain error and makes a perfectly aligned pair read as misaligned by
/// the distance between the two domains.
///
/// Stamps arrive through a LOOKUP rather than a concrete map because the two
/// callers hold different containers — the scheduler an `IndexMap` on its
/// decide path, `cerulion_cli_engine::replay_rederive` a `BTreeMap` rebuilt
/// from a bag — and neither may pay a conversion to satisfy the other. Extra
/// stamps the lookup can serve are IGNORED: only `trigger_inputs` is consulted.
#[must_use]
pub fn sync_aligned<F>(stamp_of: F, trigger_inputs: &[String], window_ns: Option<u64>) -> bool
where
    F: Fn(&str) -> Option<u64>,
{
    let mut min_ts = u64::MAX;
    let mut max_ts = 0u64;
    for input in trigger_inputs {
        let Some(ts) = stamp_of(input) else {
            return false;
        };
        min_ts = min_ts.min(ts);
        max_ts = max_ts.max(ts);
    }
    // Unbounded sync: every input present is the whole condition.
    let Some(window_ns) = window_ns else {
        return true;
    };
    // A node with NO declared trigger inputs has no spread to judge; the
    // scheduler refuses such a node at `add_node`, so this is defensive.
    if trigger_inputs.is_empty() {
        return true;
    }
    max_ts.saturating_sub(min_ts) <= window_ns
}

impl Default for Scheduler {
    fn default() -> Self {
        Self::new()
    }
}

/// Bump the appropriate per-policy counter on `counters` and
/// emit a structured `tracing::warn!` describing the event.
///
/// Exposed at module scope so transport-layer enforcement sites (which
/// cache the `Arc<BackpressureCounters>` returned from
/// `Scheduler::register_backpressure_input_with`) can record events without
/// re-entering the scheduler's `RwLock`. The structured fields match
/// the `BackpressureEvent` struct surfaced to `#[on_event]`
/// user callbacks so log readers and callback handlers see the same
/// vocabulary.
///
/// Note: `BlockDeferred` is **not** a data-loss event — the producer's
/// tick was skipped at the scheduler pre-fire predicate, so the message
/// was never produced. The warn message reflects this.
pub fn record_backpressure_event(
    node_id: &str,
    input_name: &str,
    policy: BackpressurePolicy,
    counters: &BackpressureCounters,
) {
    // The single-event convenience path emits the warn per call (`emit_warn =
    // true`). Its ONLY remaining caller is the `signal_backpressure_event` API
    // (the manual/test event-injection seam). Every high-frequency PRODUCTION
    // path — the `drop_oldest` drain, the `sample(N)` decimate (both in
    // subscriber.rs), AND the `block` pre-fire defer (runtime.rs) — calls
    // `record_backpressure_event_n` directly and passes its OWN once-per-regime
    // edge-trigger as `emit_warn`; see
    // [`record_backpressure_event_n`].
    record_backpressure_event_n(node_id, input_name, policy, counters, 1, true);
}

/// Like [`record_backpressure_event`] but bumps the per-policy
/// counter by `n` in one shot. The `drop_oldest` detector observes
/// iceoryx2 evicting a BATCH of `n` samples between two drains (`n`
/// missing sequence slots), so it records `n` at once rather than calling the
/// single-event writer in a loop. The canonical bump + structured warn
/// stay in this one place.
///
/// `emit_warn` gates ONLY the `tracing::warn!` — the counter bump is
/// UNCONDITIONAL (Principle #3: truth is the counter, not the log). The
/// `drop_oldest` drain (subscriber.rs) records an eviction on EVERY drain that
/// evicts, but a healthy overflowing topic would flood the log with one warn
/// per drain (a measured 91% of a healthy nav2 `record.log`). It passes its
/// existing once-per-regime edge-trigger (`fire` = `probe.armed`) as
/// `emit_warn`, so the warn now fires exactly once per eviction regime —
/// aligned with the `BackpressureEvent` already gated by that same edge — while
/// the counter still counts every eviction. Non-batched callers pass `true`
/// (via [`record_backpressure_event`]) to keep their per-event warn.
///
/// `emit_warn == false` is NOT full silence: the sustained
/// event logs at `debug!` (same structured fields + the running `total`),
/// invisible at the default `info` level but present under a `debug` filter, so
/// a permanently-overflowing input is diagnosable without flooding a healthy
/// log. This mirrors the `DrainWarnLatch` loud-first / quiet-sustained shape.
///
/// PRODUCTION CALLERS: all THREE policies now ride their own once-per-regime
/// edge-trigger, passed as `emit_warn`, so each warns loud on the FIRST event of
/// a regime and logs sustained events at `debug!`:
///
/// - `drop_oldest` drain + `sample(N)` decimate (both in subscriber.rs): the
///   probe's `armed` bit, rearmed by an accept / below-eviction drain.
/// - `block` pre-fire defer (runtime.rs): the `BlockDeferEdge::armed` bit,
///   rearmed by a below-threshold observation — the SAME edge the consumer-side
///   `block` `BackpressureEvent` fires on.
///
/// All three are high-frequency healthy-steady-state paths. `block` is designed
/// as lossless backpressure (a slow consumer intentionally throttling a fast
/// producer; see `docs/internals/core-scheduler-graph.md`), so a consumer sitting at its
/// buffer threshold is a LEGITIMATE steady state — a per-deferred-step warn
/// would flood a healthy log exactly like a 1kHz producer into a 100ms `sample`
/// window decimating ~990 frames/s. Riding the once-per-regime edge kills that
/// flood while keeping the counter exact.
///
/// The `emit_warn == false` sustained-`debug!` arm is therefore production-
/// reachable for ALL three policies (not just `drop_oldest`). It is additionally
/// pinned as a direct unit contract in `backpressure_counters_test.rs`.
pub fn record_backpressure_event_n(
    node_id: &str,
    input_name: &str,
    policy: BackpressurePolicy,
    counters: &BackpressureCounters,
    n: u64,
    emit_warn: bool,
) {
    match policy {
        BackpressurePolicy::DropOldest => {
            // fetch_add returns the PREVIOUS value; +n is the running total.
            let total = counters.drop_oldest_count.fetch_add(n, Ordering::Release) + n;
            if emit_warn {
                tracing::warn!(
                    node_id = %node_id,
                    input = %input_name,
                    policy = "drop_oldest",
                    evicted = n,
                    total,
                    "backpressure event: iceoryx2 evicted oldest queued message(s) on \
                     overflow (detected via wire-sequence gap) — \
                     `backpressure_drop_oldest_count` incremented"
                );
            } else {
                // Sustained regime: the once-per-regime warn
                // already fired at regime open. Stay quiet-but-PRESENT at
                // `debug!` (invisible at the default `info` level, so the nav2
                // 91%-of-record.log flood stays dead) rather than fully silent,
                // mirroring the `DrainWarnLatch` loud-first / debug-sustained
                // shape. The counter bump above is unconditional (Principle #3).
                tracing::debug!(
                    node_id = %node_id,
                    input = %input_name,
                    policy = "drop_oldest",
                    evicted = n,
                    total,
                    "backpressure event (sustained; first of regime logged loudly at warn): \
                     iceoryx2 evicted oldest queued message(s) on overflow — \
                     `backpressure_drop_oldest_count` incremented"
                );
            }
        }
        BackpressurePolicy::Block => {
            let total = counters
                .block_fires_deferred_count
                .fetch_add(n, Ordering::Release)
                + n;
            if emit_warn {
                tracing::warn!(
                    node_id = %node_id,
                    input = %input_name,
                    policy = "block",
                    total,
                    "backpressure event: producer's tick deferred (sole-Block-consumer-full \
                     or all-Block-buffers-full) — no data lost, \
                     `backpressure_block_fires_deferred_count` incremented"
                );
            } else {
                tracing::debug!(
                    node_id = %node_id,
                    input = %input_name,
                    policy = "block",
                    total,
                    "backpressure event (sustained; first of regime logged loudly at warn): \
                     producer's tick deferred — no data lost, \
                     `backpressure_block_fires_deferred_count` incremented"
                );
            }
        }
        BackpressurePolicy::Sample(interval_ms) => {
            let total = counters.sampled_count.fetch_add(n, Ordering::Release) + n;
            if emit_warn {
                tracing::warn!(
                    node_id = %node_id,
                    input = %input_name,
                    policy = "sample",
                    interval_ms,
                    total,
                    "backpressure event: dropped message arrived within sample window — \
                     `backpressure_sampled_count` incremented"
                );
            } else {
                tracing::debug!(
                    node_id = %node_id,
                    input = %input_name,
                    policy = "sample",
                    interval_ms,
                    total,
                    "backpressure event (sustained; first of regime logged loudly at warn): \
                     dropped message arrived within sample window — \
                     `backpressure_sampled_count` incremented"
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The capacity a HAND-MINTED stage in this module stands in for.
    ///
    /// A wired stage's capacity is DERIVED from its edge's graph facts, and
    /// these arms mint stages directly (there is no graph here), so the shape
    /// is DECLARED rather than assumed: the ordinary single-publisher,
    /// non-Sync latest-value BODY input at the default consumer depth. Reading
    /// it through `derive_stage_capacity` keeps these arms moving WITH the
    /// derivation instead of against a number frozen in a test.
    fn hand_wired_stage_sizing() -> crate::read_outcome::ReadStageSizing {
        crate::read_outcome::ReadStageSizing {
            depth: crate::graph::topology::DEFAULT_CONSUMER_DEPTH as u32,
            annotated: crate::read_outcome::ProducerAnnotation::Plain,
            sync_shape: crate::read_outcome::SyncTriggerShape::Ordinary,
            // Routed through the ONE mint, exactly as a wiring site would: a
            // FROZEN body input on an entry that really holds its snapshot.
            burst: crate::graph::runtime::read_stage_burst_bound(
                crate::read_outcome::ReadStageRole::Body,
                "ctx",
                &["ctx".to_string()],
                true,
                crate::read_outcome::FireBurstBound::Fires(1),
            ),
        }
    }

    fn noop_callback() -> Box<dyn FnMut() + Send> {
        Box::new(|| {})
    }

    fn add_period(scheduler: &mut Scheduler, id: &str, interval: Duration) {
        scheduler
            .add_node(NodeConfig {
                id: id.to_string(),
                policy: TriggerPolicy::Period {
                    interval,
                    max_catchup: None,
                },
                callback: noop_callback(),
            })
            .unwrap();
    }

    fn add_data(scheduler: &mut Scheduler, id: &str) {
        scheduler
            .add_node(NodeConfig {
                id: id.to_string(),
                policy: TriggerPolicy::Data,
                callback: noop_callback(),
            })
            .unwrap();
    }

    fn add_sync(scheduler: &mut Scheduler, id: &str, inputs: &[&str], window: Option<Duration>) {
        scheduler
            .add_node(NodeConfig {
                id: id.to_string(),
                policy: TriggerPolicy::Sync {
                    inputs: inputs.iter().map(|s| s.to_string()).collect(),
                    window,
                },
                callback: noop_callback(),
            })
            .unwrap();
    }

    /// The manifest-desync drop counters are split
    /// by record kind — `trace_ring_unmapped_count()` keeps meaning FIRE
    /// records only (its callers report fire-record loss), while kind-6
    /// read-outcome drops ride `trace_ring_unmapped_read_outcomes()`. Hand
    /// oracle over a ring whose manifest covers only "alpha" while the
    /// scheduler also runs "beta": beta's fires land on the FIRE counter
    /// exactly, a read outcome merged for beta lands on the READ counter
    /// exactly, and a mapped node's read outcome lands on NEITHER (it
    /// reaches the ring as a kind-6 record — the anti-tautology arm).
    #[cfg(unix)]
    #[test]
    fn unmapped_drop_counters_split_fires_from_read_outcomes() {
        use crate::read_outcome::{ReadOutcomeKind, ReadOutcomeStage, ReadStageRole};

        let tag = format!("split_{}", std::process::id());
        let mut owner =
            crate::trace_ring::TraceRingOwner::create(&tag, 64, 0, &["alpha"]).expect("create");
        let ring_name = owner.name().to_string();
        let producer = owner.producer().expect("mint producer");

        let mut scheduler =
            Scheduler::with_virtual_clock(Arc::new(crate::clock::VirtualClock::new()));
        scheduler.set_trace_ring_producer(producer, &["alpha".to_string()]);
        add_period(&mut scheduler, "alpha", Duration::from_millis(10));
        add_period(&mut scheduler, "beta", Duration::from_millis(10));

        // One armed stage per node (the recording-on state).
        // A stage's capacity is DERIVED from its edge's graph facts,
        // so a test that mints one by hand DECLARES the shape it is standing in
        // for — here the ordinary single-publisher latest-value input.
        let alpha_stage = Arc::new(ReadOutcomeStage::new(
            0,
            ReadStageRole::Body,
            hand_wired_stage_sizing(),
        ));
        alpha_stage.arm();
        scheduler
            .set_node_read_stages("alpha", vec![Arc::clone(&alpha_stage)])
            .expect("wire alpha stage");
        let beta_stage = Arc::new(ReadOutcomeStage::new(
            0,
            ReadStageRole::Body,
            hand_wired_stage_sizing(),
        ));
        beta_stage.arm();
        scheduler
            .set_node_read_stages("beta", vec![Arc::clone(&beta_stage)])
            .expect("wire beta stage");

        const STEPS: u64 = 3;
        for _ in 0..STEPS {
            scheduler.step_ms(10);
        }
        // Beta fired once per step; every fire was dropped unmapped. No read
        // outcome has been merged yet, so the READ counter must not move.
        assert_eq!(
            scheduler.trace_ring_unmapped_count(),
            STEPS,
            "the FIRE counter counts exactly beta's dropped fires"
        );
        assert_eq!(
            scheduler.trace_ring_unmapped_read_outcomes(),
            0,
            "fires must never bump the READ counter"
        );

        // One staged read outcome on the UNMAPPED node: the READ counter
        // moves by exactly 1; the FIRE counter not at all.
        beta_stage.record(
            ReadOutcomeKind::Served,
            Some(7),
            1,
            crate::read_outcome::ReadSiteRole::Body,
        );
        scheduler.merge_read_outcomes(&["beta".to_string()], &[]);
        assert_eq!(
            scheduler.trace_ring_unmapped_count(),
            STEPS,
            "a dropped read outcome must never bump the FIRE counter"
        );
        assert_eq!(
            scheduler.trace_ring_unmapped_read_outcomes(),
            1,
            "the READ counter counts exactly beta's dropped read outcome"
        );

        // A MAPPED node's read outcome bumps NEITHER — it reaches the ring.
        alpha_stage.record(
            ReadOutcomeKind::Served,
            Some(9),
            1,
            crate::read_outcome::ReadSiteRole::Body,
        );
        scheduler.merge_read_outcomes(&["alpha".to_string()], &[]);
        assert_eq!(scheduler.trace_ring_unmapped_count(), STEPS);
        assert_eq!(scheduler.trace_ring_unmapped_read_outcomes(), 1);
        let mut consumer =
            crate::trace_ring::TraceRingConsumer::open(&ring_name).expect("open ring");
        let mut records = Vec::new();
        consumer.drain(&mut records).expect("drain");
        let kind6: Vec<_> = records
            .iter()
            .filter(|r| r.record_type == crate::trace_ring::RECORD_TYPE_READ_OUTCOME)
            .collect();
        assert_eq!(
            kind6.len(),
            1,
            "exactly the mapped node's read outcome reached the ring"
        );
        assert_eq!(kind6[0].node_idx, 0, "resolved through the manifest");
    }

    /// A single `Period(1ms)` node reports its interval as the
    /// tightest timing requirement.
    #[test]
    fn tightest_timing_ns_single_period_reports_interval() {
        let mut scheduler = Scheduler::new();
        add_period(&mut scheduler, "cam", Duration::from_millis(1));
        assert_eq!(scheduler.tightest_timing_ns(), Some(1_000_000));
    }

    /// A purely data-driven graph (no Period, no QoS deadline) has no
    /// timing requirement → `None` (latency-tolerant, may idle deep).
    #[test]
    fn tightest_timing_ns_data_only_is_none() {
        let mut scheduler = Scheduler::new();
        add_data(&mut scheduler, "relay");
        assert_eq!(scheduler.tightest_timing_ns(), None);
    }

    /// An empty graph (zero nodes) has no timing requirement → `None`.
    #[test]
    fn tightest_timing_ns_zero_nodes_is_none() {
        let scheduler = Scheduler::new();
        assert_eq!(scheduler.tightest_timing_ns(), None);
    }

    /// The MIN is taken across every timing source — a QoS
    /// `expect_within(2ms)` window tighter than a `Period(10ms)` wins.
    #[test]
    fn tightest_timing_ns_takes_min_across_period_and_expect_within() {
        let mut scheduler = Scheduler::new();
        add_period(&mut scheduler, "fusion", Duration::from_millis(10));
        scheduler
            .set_expect_within("fusion", "imu", 2, Arc::new(AtomicU64::new(0)), false, None)
            .unwrap();
        assert_eq!(scheduler.tightest_timing_ns(), Some(2_000_000));
    }

    /// A data node carrying only a `tick_within_ms` budget reports
    /// that budget (no Period present).
    #[test]
    fn tightest_timing_ns_tick_within_only() {
        let mut scheduler = Scheduler::new();
        add_data(&mut scheduler, "worker");
        scheduler.set_tick_within("worker", 5).unwrap();
        assert_eq!(scheduler.tightest_timing_ns(), Some(5_000_000));
    }

    /// A data node carrying only an `expect_within_ms` input watchdog
    /// reports that window.
    #[test]
    fn tightest_timing_ns_expect_within_only() {
        let mut scheduler = Scheduler::new();
        add_data(&mut scheduler, "sink");
        scheduler
            .set_expect_within("sink", "in", 3, Arc::new(AtomicU64::new(0)), false, None)
            .unwrap();
        assert_eq!(scheduler.tightest_timing_ns(), Some(3_000_000));
    }

    /// A data node carrying only a `promise_within_ms` output watchdog
    /// reports that window.
    #[test]
    fn tightest_timing_ns_promise_within_only() {
        let mut scheduler = Scheduler::new();
        add_data(&mut scheduler, "src");
        scheduler
            .set_promise_within("src", "out", 4, Arc::new(AtomicU64::new(0)))
            .unwrap();
        assert_eq!(scheduler.tightest_timing_ns(), Some(4_000_000));
    }

    /// With two nodes, the global MIN can come from the second node.
    #[test]
    fn tightest_timing_ns_min_from_second_node() {
        let mut scheduler = Scheduler::new();
        add_period(&mut scheduler, "slow", Duration::from_millis(50));
        add_period(&mut scheduler, "fast", Duration::from_millis(8));
        assert_eq!(scheduler.tightest_timing_ns(), Some(8_000_000));
    }

    /// A bounded `Sync { window: Some(5ms) }` node reports its
    /// pairing window as a declared latency requirement (symmetric to
    /// `expect_within`).
    #[test]
    fn tightest_timing_ns_bounded_sync_window_contributes() {
        let mut scheduler = Scheduler::new();
        add_sync(
            &mut scheduler,
            "fuse",
            &["a", "b"],
            Some(Duration::from_millis(5)),
        );
        assert_eq!(scheduler.tightest_timing_ns(), Some(5_000_000));
    }

    /// An UNBOUNDED `Sync { window: None }` node is
    /// latency-tolerant — it declares no timing requirement → `None`.
    #[test]
    fn tightest_timing_ns_unbounded_sync_is_none() {
        let mut scheduler = Scheduler::new();
        add_sync(&mut scheduler, "fuse", &["a", "b"], None);
        assert_eq!(scheduler.tightest_timing_ns(), None);
    }

    /// A SINGLE node with two `expect_within` inputs (7ms,
    /// 3ms), a `promise_within` output (5ms), and a `Period(10ms)` reports the
    /// MIN across ALL of its own entries — `3ms` — proving the min walks every
    /// `.values()` entry on one node, not just the first.
    #[test]
    fn tightest_timing_ns_min_across_multiple_entries_on_one_node() {
        let mut scheduler = Scheduler::new();
        add_period(&mut scheduler, "fusion", Duration::from_millis(10));
        scheduler
            .set_expect_within(
                "fusion",
                "slow",
                7,
                Arc::new(AtomicU64::new(0)),
                false,
                None,
            )
            .unwrap();
        scheduler
            .set_expect_within(
                "fusion",
                "fast",
                3,
                Arc::new(AtomicU64::new(0)),
                false,
                None,
            )
            .unwrap();
        scheduler
            .set_promise_within("fusion", "out", 5, Arc::new(AtomicU64::new(0)))
            .unwrap();
        assert_eq!(scheduler.tightest_timing_ns(), Some(3_000_000));
    }

    /// A node disabled mid-run STILL contributes its configured
    /// timing — the cap is derived from the graph's configured tightness, not
    /// its live enabled-set (the no-`!disabled`-filter decision).
    #[test]
    fn tightest_timing_ns_disabled_node_still_contributes() {
        let mut scheduler = Scheduler::new();
        add_period(&mut scheduler, "loose", Duration::from_millis(50));
        add_period(&mut scheduler, "tight", Duration::from_millis(1));
        // Disable the tight node directly (simulating the repeated-panic
        // circuit breaker). It must NOT loosen the startup-derived cap.
        scheduler.nodes.get_mut("tight").unwrap().disabled = true;
        assert_eq!(scheduler.tightest_timing_ns(), Some(1_000_000));
    }

    // -----------------------------------------------------------------------
    // The deterministic-LIVE `Barrier` gating
    // clock arm + its `with_barrier_clock` constructor.
    // -----------------------------------------------------------------------

    /// A scheduler built via `with_barrier_clock` selects the `Barrier` arm BY
    /// INTENT and advances its GATING clock by the handed quantum — it is NOT
    /// frozen (unlike the `Real` arm's no-op advance). Handing the quantum N
    /// twice lands `now_ns` at 2N (it ACCUMULATES), and the gating clock drives
    /// `fire_time_ns` exactly as the `Virtual` arm does.
    #[test]
    fn with_barrier_clock_advances_gating_clock_by_handed_quantum() {
        let clock = Arc::new(VirtualClock::new());
        // A second handle onto the SAME gating clock the scheduler advances, so
        // we can read `now_ns` directly (the scheduler's clock field is private).
        let observe = clock.clone();
        let mut scheduler = Scheduler::with_barrier_clock(clock);

        // The constructor placed the clock in the `Barrier` arm — NOT `Virtual`
        // (polled/replay) and NOT `Real` (no-op live). `ClockInner` is private with
        // no accessor, so the live loop does NOT read this arm to pick its advance
        // source: that source is the runtime's `live_gating_quantum` + the
        // `step_live(gating, wall)` split. This arm is a documentary
        // intent marker (and a cross-process forward hook) whose one load-bearing job is to
        // keep this `VirtualClock` advanceable (out of the no-op `Real` arm) — the
        // property we assert next.
        assert!(
            matches!(scheduler.clock, ClockInner::Barrier(_)),
            "with_barrier_clock must select the Barrier arm by intent"
        );

        // A Period node lets us also confirm the gating clock drives `fire_time`.
        add_period(&mut scheduler, "ticker", Duration::from_millis(10));

        // Before any advance the gating clock reads 0.
        assert_eq!(observe.now_ns(), 0);

        // Hand the live quantum (10ms) once: the gating clock advances and the
        // Period node fires AT that gating time.
        scheduler.step(Duration::from_millis(10));
        assert_eq!(observe.now_ns(), 10_000_000);
        assert_eq!(
            scheduler.trace().last().map(|e| e.fire_time_ns),
            Some(10_000_000)
        );

        // Hand it again: the `Barrier` arm is NOT a no-op — the quantum
        // ACCUMULATES to 2N (twice N == 20ms), and `fire_time` tracks it.
        scheduler.step(Duration::from_millis(10));
        assert_eq!(observe.now_ns(), 20_000_000);
        assert_eq!(
            scheduler.trace().last().map(|e| e.fire_time_ns),
            Some(20_000_000)
        );
    }

    /// Contrast pin at the `ClockInner::advance` seam: the `Barrier` arm's
    /// advance is NOT a no-op (it accumulates the handed quantum), whereas the
    /// `Real` arm's advance IS a no-op (it never advances its wrapped clock,
    /// only reads it). Built directly so both arms are exercised through the
    /// exact same `advance` seam, isolating the arm as the only variable.
    #[test]
    fn barrier_arm_advance_is_not_a_noop_unlike_real_arm() {
        const Q: u64 = 7_000_000; // an arbitrary handed quantum, in ns

        // Barrier arm: advances deterministically. Two N-handoffs → 2N.
        let barrier = ClockInner::Barrier(Arc::new(VirtualClock::new()));
        assert_eq!(barrier.now_ns(), 0);
        assert_eq!(
            barrier.advance(Q),
            Q,
            "Barrier advance must apply the quantum"
        );
        assert_eq!(barrier.advance(Q), 2 * Q, "Barrier advance must ACCUMULATE");
        assert_eq!(barrier.now_ns(), 2 * Q);

        // Real arm (over an advanceable `VirtualClock` upcast to `dyn Clock`):
        // the scheduler treats it as read-only, so `advance` is a no-op — it
        // returns the current (unchanged) time and never advances the clock.
        let real: ClockInner = ClockInner::Real(Arc::new(VirtualClock::new()));
        assert_eq!(real.now_ns(), 0);
        assert_eq!(real.advance(Q), 0, "Real advance must be a no-op");
        assert_eq!(real.advance(Q), 0, "Real advance must stay a no-op");
        assert_eq!(real.now_ns(), 0, "Real arm must remain frozen");
    }

    /// [`RestWalk::InsertionOrder`] takes a level OFF the rayon path
    /// ONLY where size + pool would have sent it wide, and the walk it forces
    /// is the insertion-order serial walk — the scheduler-seam half of the
    /// shared-topic ordering constraint (the transport half, the shared FIFO's
    /// OBSERVED interleave over real iceoryx2, is
    /// `read_outcome_capture_iox2_test`'s shared-topic arms).
    ///
    /// Three levels through ONE scheduler, hand oracles: a WIDE level under
    /// `Unconstrained` (the counter must stay 0 — that level really went wide),
    /// the SAME wide level under `InsertionOrder` (counter 1, every fire in
    /// insertion order), then a NARROW level under `InsertionOrder` (counter
    /// UNCHANGED — size already made it serial, so the constraint is not
    /// credited with a walk it did not change). Neutralising `rest_driver`'s
    /// `rest_walk == RestWalk::InsertionOrder` arm fails step 1 (the counter
    /// stays 0) whatever the workers happen to do.
    #[test]
    fn tick_decided_parallel_insertion_order_walk_is_forced_only_where_size_would_go_wide() {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        // One past the threshold: the REST count is wide on its own.
        let width = Scheduler::PARALLEL_FIRE_THRESHOLD + 1;
        let ids: Vec<String> = (0..width).map(|i| format!("n{i:02}")).collect();
        for id in &ids {
            add_period(&mut scheduler, id, Duration::from_millis(10));
        }
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(2)
            .build()
            .expect("build 2-thread fire pool");
        let serial_ids: HashSet<String> = HashSet::new();
        const T_NS: u64 = 10_000_000;

        // Step 0: wide + Unconstrained ⇒ rayon; the constraint
        // counter must not move (an unconditional bump would read 1 here).
        scheduler.decide_fires(&ids, T_NS);
        let mut decisions = scheduler.take_decisions();
        assert_eq!(
            decisions.len(),
            width,
            "every Period node decides at t=10ms"
        );
        scheduler.tick_decided_parallel(
            &mut decisions,
            0,
            0,
            &pool,
            &serial_ids,
            RestWalk::Unconstrained,
        );
        assert_eq!(
            scheduler.forced_serial_rest_walks(),
            0,
            "an Unconstrained wide level is rayon's — the constraint counter must stay 0"
        );

        // Step 1: the SAME wide level + InsertionOrder ⇒ the forced
        // serial walk, counted exactly once.
        scheduler.decide_fires(&ids, 2 * T_NS);
        let mut decisions = scheduler.take_decisions();
        assert_eq!(
            decisions.len(),
            width,
            "every Period node decides at t=20ms"
        );
        scheduler.tick_decided_parallel(
            &mut decisions,
            0,
            1,
            &pool,
            &serial_ids,
            RestWalk::InsertionOrder,
        );
        assert_eq!(
            scheduler.forced_serial_rest_walks(),
            1,
            "a wide level under InsertionOrder is the constraint's doing — counted once"
        );

        // Step 2: a NARROW level (2 decided fires — past the 1-fire
        // fast path, below the threshold) + InsertionOrder ⇒ serial BY SIZE;
        // the constraint changed nothing and must not be credited.
        scheduler.decide_fires(&ids[..2], 3 * T_NS);
        let mut decisions = scheduler.take_decisions();
        assert_eq!(
            decisions.len(),
            2,
            "exactly the two re-decided nodes fire at t=30ms"
        );
        scheduler.tick_decided_parallel(
            &mut decisions,
            0,
            2,
            &pool,
            &serial_ids,
            RestWalk::InsertionOrder,
        );
        assert_eq!(
            scheduler.forced_serial_rest_walks(),
            1,
            "a size-narrow level is serial regardless — the constraint counter must not move"
        );

        // ORACLE (hand-written): every phase fired in insertion order, tagged
        // with its step. The merged trace is decision-ordered on EVERY driver
        // (that is invariant A), so this is the driver-invariant half; the
        // counter above is the driver-sensitive half.
        let fired: Vec<(&str, u64)> = scheduler
            .trace()
            .iter()
            .map(|e| (e.node_id.as_ref(), e.step))
            .collect();
        let mut expected: Vec<(&str, u64)> = Vec::new();
        for id in &ids {
            expected.push((id.as_str(), 0));
        }
        for id in &ids {
            expected.push((id.as_str(), 1));
        }
        for id in &ids[..2] {
            expected.push((id.as_str(), 2));
        }
        assert_eq!(
            fired, expected,
            "each phase fires its decided set in insertion order under its own step"
        );
    }

    /// The pure verdict, every cell of
    /// {below / at the threshold} × {1 / 4 threads} × {pauses armed / not} ×
    /// {Unconstrained / InsertionOrder} against a HAND-WRITTEN expectation, plus
    /// two ABOVE-threshold rows so the size gate is pinned as a `<` rather than
    /// an equality.
    /// The table is the precedence contract in tabular form: size beats
    /// everything, an armed seam beats the constraint, and only the last
    /// column of the last block reaches `Parallel` / `SerialByConstraint`. A
    /// swapped rank, a dropped pool half, or a deleted arm each flips at least
    /// one row.
    #[test]
    fn rest_driver_verdict_table_is_size_then_pauses_then_constraint() {
        use RestDriver::{Parallel, SerialByConstraint, SerialByReplayPauses, SerialBySize};
        use RestWalk::{InsertionOrder, Unconstrained};
        const BELOW: usize = Scheduler::PARALLEL_FIRE_THRESHOLD - 1;
        const AT: usize = Scheduler::PARALLEL_FIRE_THRESHOLD;
        const ABOVE: usize = Scheduler::PARALLEL_FIRE_THRESHOLD + 1;
        // (rest fires, pool threads, pauses armed, rest walk) -> expected.
        let table: [(usize, usize, bool, RestWalk, RestDriver); 18] = [
            // Below the threshold: size wins whatever else is true.
            (BELOW, 1, false, Unconstrained, SerialBySize),
            (BELOW, 1, false, InsertionOrder, SerialBySize),
            (BELOW, 1, true, Unconstrained, SerialBySize),
            (BELOW, 1, true, InsertionOrder, SerialBySize),
            (BELOW, 4, false, Unconstrained, SerialBySize),
            (BELOW, 4, false, InsertionOrder, SerialBySize),
            (BELOW, 4, true, Unconstrained, SerialBySize),
            (BELOW, 4, true, InsertionOrder, SerialBySize),
            // At the threshold on ONE thread: still the pool half of size.
            (AT, 1, false, Unconstrained, SerialBySize),
            (AT, 1, false, InsertionOrder, SerialBySize),
            (AT, 1, true, Unconstrained, SerialBySize),
            (AT, 1, true, InsertionOrder, SerialBySize),
            // Wide on a real pool: the seam outranks the constraint, and only
            // an unarmed, unconstrained level goes to rayon.
            (AT, 4, true, Unconstrained, SerialByReplayPauses),
            (AT, 4, true, InsertionOrder, SerialByReplayPauses),
            (AT, 4, false, Unconstrained, Parallel),
            (AT, 4, false, InsertionOrder, SerialByConstraint),
            // ABOVE the threshold on a real pool: the size gate is a `<`, not an
            // equality. Without these two rows a `<` -> `!=` slip on the size
            // comparison passes every row above (`AT` is the only wide value
            // tested, and `AT != THRESHOLD` is false either way).
            (ABOVE, 4, false, Unconstrained, Parallel),
            (ABOVE, 4, false, InsertionOrder, SerialByConstraint),
        ];
        for (rest, threads, armed, walk, expected) in table {
            assert_eq!(
                Scheduler::rest_driver(rest, threads, armed, walk),
                expected,
                "rest_driver(rest={rest}, threads={threads}, armed={armed}, walk={walk:?})"
            );
        }
    }

    /// The counter's composed rule — a level
    /// is credited to the constraint ONLY when neither the armed replay seam
    /// nor the size/pool gate already narrowed it. The sibling arm above pins
    /// the size half; this one pins the SEAM half and the POOL half of the
    /// size gate, which no other test reached: crediting on
    /// `!narrow_by_size` alone (i.e. ignoring the seam) survived the whole
    /// suite before this arm.
    ///
    /// One scheduler, 9 Period(10) nodes (`THRESHOLD + 1`, wide on its own),
    /// three phases:
    /// 1. pauses ARMED with an EMPTY list (`set_replay_intra_step_pauses(0,
    ///    &[])` — no hook needed, and nothing on the Period fire path can
    ///    consult a pause), `InsertionOrder`, 4-thread pool ⇒ counter 0: the
    ///    seam narrowed it, the constraint did not;
    /// 2. seam CLEARED, same level, same pool ⇒ counter 1;
    /// 3. a 1-thread pool, same level ⇒ counter STILL 1: the pool half of the
    ///    size gate narrowed it.
    ///
    /// Trace oracle: every phase fired its whole set in insertion order under
    /// its own step (the driver-invariant half, invariant A).
    #[test]
    fn tick_decided_parallel_insertion_order_is_not_credited_when_narrow_by_seam_or_pool() {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        let width = Scheduler::PARALLEL_FIRE_THRESHOLD + 1;
        let ids: Vec<String> = (0..width).map(|i| format!("m{i:02}")).collect();
        for id in &ids {
            add_period(&mut scheduler, id, Duration::from_millis(10));
        }
        let pool4 = rayon::ThreadPoolBuilder::new()
            .num_threads(4)
            .build()
            .expect("build 4-thread fire pool");
        let pool1 = rayon::ThreadPoolBuilder::new()
            .num_threads(1)
            .build()
            .expect("build 1-thread fire pool");
        let serial_ids: HashSet<String> = HashSet::new();
        const T_NS: u64 = 10_000_000;

        // Step 0: ARMED seam + InsertionOrder + wide + 4 threads.
        scheduler
            .set_replay_intra_step_pauses(0, &[])
            .expect("an empty pause list arms the seam without a hook");
        assert!(
            scheduler.is_replay_pause_armed(),
            "step 0 precondition: armed"
        );
        scheduler.decide_fires(&ids, T_NS);
        let mut decisions = scheduler.take_decisions();
        assert_eq!(
            decisions.len(),
            width,
            "every Period node decides at t=10ms"
        );
        scheduler.tick_decided_parallel(
            &mut decisions,
            0,
            0,
            &pool4,
            &serial_ids,
            RestWalk::InsertionOrder,
        );
        assert_eq!(
            scheduler.forced_serial_rest_walks(),
            0,
            "an ARMED replay seam already narrows the REST — the constraint must not be credited"
        );

        // Step 1: seam CLEARED — the constraint is now the only
        // narrowing cause and is credited once.
        scheduler.clear_replay_intra_step_pauses();
        assert!(
            !scheduler.is_replay_pause_armed(),
            "step 1 precondition: disarmed"
        );
        scheduler.decide_fires(&ids, 2 * T_NS);
        let mut decisions = scheduler.take_decisions();
        assert_eq!(
            decisions.len(),
            width,
            "every Period node decides at t=20ms"
        );
        scheduler.tick_decided_parallel(
            &mut decisions,
            0,
            1,
            &pool4,
            &serial_ids,
            RestWalk::InsertionOrder,
        );
        assert_eq!(
            scheduler.forced_serial_rest_walks(),
            1,
            "with the seam cleared the constraint alone narrows the wide level — counted once"
        );

        // Step 2: a ONE-thread pool — the pool half of the size gate
        // narrows the same wide level, so the constraint is not credited.
        scheduler.decide_fires(&ids, 3 * T_NS);
        let mut decisions = scheduler.take_decisions();
        assert_eq!(
            decisions.len(),
            width,
            "every Period node decides at t=30ms"
        );
        scheduler.tick_decided_parallel(
            &mut decisions,
            0,
            2,
            &pool1,
            &serial_ids,
            RestWalk::InsertionOrder,
        );
        assert_eq!(
            scheduler.forced_serial_rest_walks(),
            1,
            "a single-thread pool is serial by SIZE — the constraint counter must not move"
        );

        // ORACLE (hand-written): ids × {0, 1, 2}, insertion order within each
        // step.
        let fired: Vec<(&str, u64)> = scheduler
            .trace()
            .iter()
            .map(|e| (e.node_id.as_ref(), e.step))
            .collect();
        let mut expected: Vec<(&str, u64)> = Vec::new();
        for step in 0..3u64 {
            for id in &ids {
                expected.push((id.as_str(), step));
            }
        }
        assert_eq!(
            fired, expected,
            "each phase fires the whole level in insertion order under its own step"
        );
    }

    // -----------------------------------------------------------------------
    // The
    // all-decided-fires-are-serial-gated ⇒ empty PASS 2 REST branch of
    // `tick_decided_parallel`.
    // -----------------------------------------------------------------------

    /// Directly exercise the
    /// "every decided fire is serial-gated ⇒ `rest_fire_count == 0` ⇒ the narrow
    /// PASS 2 REST loop `continue`s every slot (empty REST)" path of
    /// [`Scheduler::tick_decided_parallel`].
    ///
    /// The level executor routes `!performs_input_snapshot()` nodes (cdylib /
    /// closure nodes with non-trigger inputs) into `serial_ids` so they fire on
    /// the calling thread in PASS 1, BEFORE any parallel producer publishes. When
    /// EVERY decided fire is serial-gated, `any_serial == true` and
    /// `rest_fire_count == 0`: PASS 1 fires them all serially, and PASS 2's narrow
    /// loop walks each slot but hits `slot.serial ⇒ continue`, leaving the REST
    /// EMPTY. This is the branch under test (the e2e `rayon_fire_cdylib_serial`
    /// pin drives it through FFI; this unit test pins the scheduler seam directly,
    /// with a HAND-WRITTEN oracle — NOT a two-run self-compare).
    ///
    /// `> 1` decided fires deliberately bypass the `decisions.len() <= 1`
    /// fast-path (which delegates to `tick_decided`), so this genuinely enters the
    /// scratch / PASS 1 / PASS 2 machinery. A 2-thread pool ensures `narrow` is
    /// decided by the `rest_fire_count == 0 < PARALLEL_FIRE_THRESHOLD` gate (not
    /// by a single-thread pool), pinning the all-serial-gated branch itself.
    #[test]
    fn tick_decided_parallel_all_serial_gated_yields_empty_pass2_rest() {
        // Two Period(10ms) nodes on ONE level. `with_virtual_clock` reads 0 at
        // build (no step/advance has run), so each node's `next_fire_ns`
        // initializes to exactly the interval (10ms) — the hand-derived fire time.
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        add_period(&mut scheduler, "id0", Duration::from_millis(10));
        add_period(&mut scheduler, "id1", Duration::from_millis(10));

        // DECIDE both at t = 10ms: each Period node's `next_fire` (10ms) <= now
        // (10ms) ⇒ exactly ONE fire at `first_fire_ns = 10ms` (fire_count 1).
        const T_NS: u64 = 10_000_000;
        scheduler.decide_fires(&["id0".to_string(), "id1".to_string()], T_NS);
        let mut decisions = scheduler.take_decisions();
        assert_eq!(
            decisions.len(),
            2,
            "both Period nodes must decide to fire at t=10ms (len > 1 ⇒ bypasses \
             the `decisions.len() <= 1` fast-path delegating to tick_decided)"
        );

        // `serial_ids` contains BOTH ids ⇒ `any_serial == true` AND
        // `rest_fire_count == 0`. With `rest_fire_count (0) <
        // PARALLEL_FIRE_THRESHOLD` ⇒ `narrow == true`:
        //   PASS 1 fires BOTH serially on the calling thread (any_serial);
        //   PASS 2 (narrow) walks every slot but `continue`s each (slot.serial)
        //          ⇒ the empty PASS 2 REST under test;
        //   PASS 3 drains `decisions` (pos-ordered) ⇒ merges [id0, id1].
        let mut serial_ids: HashSet<String> = HashSet::new();
        serial_ids.insert("id0".to_string());
        serial_ids.insert("id1".to_string());
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(2)
            .build()
            .expect("build 2-thread fire pool");

        const GLOBAL_LEVEL: usize = 3;
        const STEP: u64 = 7;
        scheduler.tick_decided_parallel(
            &mut decisions,
            GLOBAL_LEVEL,
            STEP,
            &pool,
            &serial_ids,
            RestWalk::Unconstrained,
        );

        // The decision buffer is drained in PASS 3 (capacity retained for reuse).
        assert!(
            decisions.is_empty(),
            "tick_decided_parallel must drain the decision buffer (the PASS 3 drain)"
        );

        // ORACLE (hand-written, NOT a self-compare): BOTH nodes fired exactly once,
        // in DECISION order [id0, id1], at the handed `global_level`/`step`, each at
        // the Period first-fire time (10ms). Compare the WHOLE trace against a
        // literal `Vec<TraceEntry>` built by hand. (`duration_ns` is excluded from
        // `TraceEntry`'s PartialEq, but it is 0 on both sides here regardless.)
        let expected = vec![
            TraceEntry {
                node_id: Arc::from("id0"),
                step: STEP,
                fire_time_ns: T_NS,
                global_level: GLOBAL_LEVEL,
                duration_ns: 0,
                discarded: false,
            },
            TraceEntry {
                node_id: Arc::from("id1"),
                step: STEP,
                fire_time_ns: T_NS,
                global_level: GLOBAL_LEVEL,
                duration_ns: 0,
                discarded: false,
            },
        ];
        assert_eq!(
            scheduler.trace(),
            expected.as_slice(),
            "all-serial-gated narrow level must fire [id0, id1] in decision order \
             at global_level {GLOBAL_LEVEL} / step {STEP} / fire_time {T_NS}"
        );

        // Explicit node-id-sequence pin (the headline ordering assertion): the
        // empty PASS 2 REST must not drop or reorder either serial-gated fire.
        let ids: Vec<&str> = scheduler
            .trace()
            .iter()
            .map(|e| e.node_id.as_ref())
            .collect();
        assert_eq!(
            ids,
            ["id0", "id1"],
            "fire sequence must be exactly [id0, id1] (decision order)"
        );
    }

    /// Armed intra-step pauses force the SERIAL rest walk, so the
    /// order the injection hook is called in is deterministic.
    ///
    /// The wide (rayon) PASS 2 rest fans `for_each` across workers. That is a
    /// pure perf choice for the trace — PASS 3 merges in decision order, so the
    /// trace is byte-identical either way, which is exactly why nothing else in
    /// the suite would notice — but the hook calls ARE the serve order of the
    /// frames the engine injects, so fanning them out makes that order
    /// nondeterministic (a Replay=Live violation the trace cannot show).
    ///
    /// The level here is deliberately WIDE: `PARALLEL_FIRE_THRESHOLD` (8)
    /// non-serial-gated fires on a 4-thread pool, i.e. every other disjunct of
    /// `narrow` is false and only the pause disjunct can produce a serial walk.
    /// Two assertions, because either alone is weak on its own: the hook order
    /// must equal INSERTION order (what the narrow walk produces), and every
    /// invocation must have run on the CALLING thread (rayon's `install` lets
    /// the caller participate, so a wide run puts most — never reliably all —
    /// on workers).
    #[test]
    fn armed_intra_step_pauses_force_the_serial_rest_walk_for_deterministic_hook_order() {
        const WIDE: usize = Scheduler::PARALLEL_FIRE_THRESHOLD;
        let clock = Arc::new(crate::clock::VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        // Insert in an order that is NOT the sorted-name order, so an
        // accidental sort cannot masquerade as the insertion-order walk.
        let ids: Vec<String> = (0..WIDE).map(|i| format!("n{}", WIDE - 1 - i)).collect();
        for id in &ids {
            add_period(&mut scheduler, id, Duration::from_millis(10));
        }

        #[allow(clippy::type_complexity)]
        let seen: Arc<std::sync::Mutex<Vec<(String, std::thread::ThreadId)>>> =
            Arc::new(std::sync::Mutex::new(Vec::new()));
        let sink = Arc::clone(&seen);
        scheduler.set_replay_injection_hook(Arc::new(move |id: &str, _after: u32| {
            sink.lock()
                .expect("hook journal not poisoned")
                .push((id.to_string(), std::thread::current().id()));
        }));

        let pauses: Vec<IntraStepPause<'_>> = ids
            .iter()
            .map(|id| IntraStepPause {
                node_id: id.as_str(),
                after_fire: 1,
            })
            .collect();
        scheduler.set_replay_intra_step_pauses(0, &pauses).unwrap();
        let fires: Vec<ReplayFire<'_>> = ids
            .iter()
            .map(|id| ReplayFire {
                node_id: id.as_str(),
                first_fire_ns: 1_000,
                fire_count: 1,
                interval_ns: 0,
            })
            .collect();
        scheduler.set_replay_fire_plan(0, &fires).unwrap();

        // Drive `tick_decided_parallel` directly with NO serial-gated ids and a
        // multi-thread pool: `rest_fire_count == WIDE >= PARALLEL_FIRE_THRESHOLD`
        // and `current_num_threads() > 1`, so only the pause disjunct forces the
        // narrow walk.
        scheduler.begin_step(Duration::from_millis(10));
        let mut decisions: Vec<FireDecision> = Vec::new();
        for (idx, id) in ids.iter().enumerate() {
            let kind = scheduler
                .take_planned_fire(idx, id, 10_000_000)
                .expect("the plan holds a fire for every node");
            decisions.push(FireDecision {
                idx,
                node_id: Arc::from(id.as_str()),
                kind,
            });
        }
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(4)
            .build()
            .expect("build 4-thread fire pool");
        let caller = std::thread::current().id();
        scheduler.tick_decided_parallel(
            &mut decisions,
            0,
            0,
            &pool,
            &HashSet::new(),
            RestWalk::Unconstrained,
        );

        let observed = seen.lock().expect("hook journal not poisoned").clone();
        assert_eq!(
            observed
                .iter()
                .map(|(id, _)| id.as_str())
                .collect::<Vec<_>>(),
            ids.iter().map(String::as_str).collect::<Vec<_>>(),
            "the hook must be called in node INSERTION order — the serial rest \
             walk's order, and the only deterministic one"
        );
        assert!(
            observed.iter().all(|(_, tid)| *tid == caller),
            "every hook call must run on the calling thread: a rayon fan-out \
             would put some on workers, which is what makes the injection order \
             nondeterministic"
        );
        assert!(scheduler.unconsumed_replay_pauses().is_empty());
    }

    // ========================================================================
    // The production trace cap (`set_trace_limit`).
    // ========================================================================

    /// `set_trace_limit(4)` post-build caps the ring to the NEWEST 4 entries:
    /// a Period(1ms) node stepped 10×1ms fires 10 times (fire_count is the
    /// full truth) while the trace retains only the last 4 CONSECUTIVE steps
    /// ending at the max — the newest, not the oldest / a sample.
    #[test]
    fn set_trace_limit_post_build_caps_ring_to_newest() {
        // VirtualClock: `step(1ms)` advances simulated time deterministically,
        // so the Period(1ms) node fires exactly once per step.
        let mut scheduler = Scheduler::with_virtual_clock(Arc::new(VirtualClock::new()));
        scheduler.set_trace_limit(4);
        let handle = scheduler
            .add_node(NodeConfig {
                id: "n".to_string(),
                policy: TriggerPolicy::Period {
                    interval: Duration::from_millis(1),
                    max_catchup: None,
                },
                callback: noop_callback(),
            })
            .unwrap();
        for _ in 0..10 {
            scheduler.step(Duration::from_millis(1));
        }
        assert_eq!(handle.fire_count(), 10, "all 10 fires happened");
        let steps: Vec<u64> = scheduler.trace().iter().map(|e| e.step).collect();
        assert_eq!(steps.len(), 4, "ring capped at 4");
        let max = *steps.last().expect("non-empty");
        assert_eq!(
            steps,
            vec![max - 3, max - 2, max - 1, max],
            "retained entries must be the NEWEST 4 consecutive fires"
        );

        // CAP-IMMUNE append counter: all 10 appends are counted even though 6
        // were evicted from the capped ring (the live loop's fire signal
        // differences this, so a full ring cannot zero it).
        assert_eq!(
            scheduler.entries_appended(),
            10,
            "entries_appended counts every APPEND, independent of eviction"
        );

        // Monotonic across clear_trace (mirrors the steps_begun precedent):
        // clearing the trace must NOT reset the append counter — a live loop
        // differencing it across a clear would otherwise read a bogus 0 delta.
        scheduler.clear_trace();
        assert_eq!(scheduler.trace().len(), 0, "trace cleared");
        assert_eq!(
            scheduler.entries_appended(),
            10,
            "entries_appended survives clear_trace (monotonic append counter)"
        );
    }

    /// The post-build setter behaves IDENTICALLY to the pre-build
    /// `with_trace_limit` builder: same node, same stepping, byte-equal traces
    /// (`TraceEntry`'s `PartialEq` covers node_id/step/fire_time/global_level;
    /// wall-clock `duration_ns` is excluded by design).
    #[test]
    fn set_trace_limit_matches_builder() {
        let run = |mut s: Scheduler| {
            s.add_node(NodeConfig {
                id: "n".to_string(),
                policy: TriggerPolicy::Period {
                    interval: Duration::from_millis(1),
                    max_catchup: None,
                },
                callback: noop_callback(),
            })
            .unwrap();
            for _ in 0..10 {
                s.step(Duration::from_millis(1));
            }
            s.trace().to_vec()
        };
        let builder_trace =
            run(Scheduler::with_virtual_clock(Arc::new(VirtualClock::new())).with_trace_limit(4));
        let mut setter_sched = Scheduler::with_virtual_clock(Arc::new(VirtualClock::new()));
        setter_sched.set_trace_limit(4);
        let setter_trace = run(setter_sched);
        assert_eq!(builder_trace.len(), 4, "both capped to 4");
        assert_eq!(
            builder_trace, setter_trace,
            "post-build set_trace_limit must equal the with_trace_limit builder"
        );
    }
    /// The COLLECT mirror's Period catch-up
    /// burst must stop at the panic circuit breaker too.
    ///
    /// `tick_node_into` is the mirror the rayon level executor drives
    /// (`fire_into_fragment`), so its break is reachable from NO integration
    /// test: the bare-`Scheduler` seam `scheduler_test` uses routes through
    /// `tick_node`, and through `GraphRuntime` the disable path cannot be
    /// reached at all — the runtime's tick callback holds the node entry's
    /// `Mutex` across `tick()`, so the FIRST panic poisons it and every later
    /// fire returns without re-entering the body (`panic_count` sticks at 1,
    /// `consecutive_panics` resets, `MAX_CONSECUTIVE_PANICS` is never met; see
    /// `rayon_fire_iox2_test`'s test 5). With
    /// this break deleted, `scheduler_test` (74 arms) and
    /// `rayon_fire_iox2_test` (10 arms) are both fully green.
    ///
    /// So the mirror is pinned HERE, driving the private fn directly with a
    /// hand-built `FireKind::Period` — the only vantage that can see it, and
    /// the reason the two loops cannot silently drift apart.
    #[test]
    fn the_collect_mirror_period_burst_stops_at_the_panic_circuit_breaker() {
        // `Scheduler::MAX_CONSECUTIVE_PANICS` restated as a HAND oracle.
        const PANIC_LIMIT: u64 = 3;
        // Comfortably past the limit, so a burst that ignores the flag is
        // unmistakable rather than off by one.
        const BURST: u32 = 10;
        const INTERVAL_NS: u64 = 10_000_000;

        let mut scheduler = Scheduler::with_virtual_clock(Arc::new(VirtualClock::new()));
        let calls = Arc::new(AtomicU64::new(0));
        let calls_cb = Arc::clone(&calls);
        scheduler
            .add_node(NodeConfig {
                id: "panicker".to_string(),
                policy: TriggerPolicy::Period {
                    interval: Duration::from_millis(10),
                    max_catchup: None,
                },
                callback: Box::new(move || {
                    calls_cb.fetch_add(1, Ordering::Relaxed);
                    panic!("intentional test panic");
                }),
            })
            .unwrap();

        let mut sink: Vec<TraceEntry> = Vec::new();
        let node = scheduler.nodes.get_mut("panicker").expect("node present");
        Scheduler::tick_node_into(
            "panicker",
            node,
            FireKind::Period {
                first_fire_ns: INTERVAL_NS,
                fire_count: BURST,
                interval_ns: INTERVAL_NS,
                current_time_ns: INTERVAL_NS * BURST as u64,
            },
            0,
            0,
            &mut sink,
        );

        assert_eq!(
            calls.load(Ordering::Relaxed),
            PANIC_LIMIT,
            "the collect mirror must stop the instant the breaker opens — one \
             that re-reads nothing runs all {BURST} due intervals"
        );
        assert_eq!(
            sink.len() as u64,
            PANIC_LIMIT,
            "one trace entry per fire, so a burst that ran on past the breaker \
             writes {BURST} entries for a node the breaker disabled"
        );
        assert!(
            node.disabled,
            "the breaker really opened — without this the counts above would \
             be pinning a burst that merely ran out of intervals"
        );
        // NO REWIND on the disabled break: `decide_node` would have advanced
        // `next_fire_ns` past the whole burst, and a disabled node must not
        // resume (`reset_node`'s no-catch-up contract). The hand-built
        // `FireKind` above bypasses `decide_node`, so what is pinned here is
        // that the break itself leaves the field ALONE — the block-defer arm
        // directly above it would have rewound to the earliest un-fired
        // interval.
        assert_eq!(
            node.next_fire_ns,
            Some(INTERVAL_NS),
            "the disabled break must not rewind `next_fire_ns` (the block-defer \
             arm does; this one must not)"
        );
    }

    /// The scheduler-level refusals of
    /// `place_gating_epoch` are pinned at THIS layer, not only through the
    /// runtime (which refuses first on its own quantum check). A `VirtualClock`
    /// scheduler has no CONTROLLED `Barrier` clock to place — `Err(GraphError)`
    /// naming it, the clock UNTOUCHED (a placement that moved the clock before
    /// refusing would be the exact skew the primitive exists to prevent) —
    /// and `epoch_placeable` says the same thing without placing.
    #[test]
    fn place_gating_epoch_refuses_a_non_controlled_clock_and_leaves_it_untouched() {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_clock(clock.clone());
        add_period(&mut scheduler, "p", Duration::from_millis(4));
        let err = scheduler
            .place_gating_epoch(1_000_000)
            .expect_err("a VirtualClock scheduler has no controlled gating clock");
        assert!(
            matches!(err, TransportError::GraphError { .. }),
            "the scheduler refuses with its own GraphError class, got {err:?}"
        );
        assert!(
            err.to_string().contains("no CONTROLLED"),
            "the refusal names the missing controlled clock: {err}"
        );
        assert_eq!(clock.now_ns(), 0, "a refused placement moves nothing");
        let err = scheduler
            .epoch_placeable()
            .expect_err("the placeable check refuses the same clock");
        assert!(err.to_string().contains("no CONTROLLED"), "{err}");
        assert_eq!(clock.now_ns(), 0);
    }
}

/// The per-set Sync ALIGN DRIVER, driven against a hand-written
/// transport double.
///
/// # Why a double rather than real iceoryx2
///
/// Two of these arms exist to pin what happens when a transport op FAILS, and a
/// real transport cannot be asked to fail on one named input while succeeding on
/// another — which is exactly the asymmetric regime the position-aware failed-probe rule
/// exists for. A DI double is the sanctioned shape here (Principle #13 forbids
/// fabricating DATA and presenting it as a measurement; it does not forbid a
/// test double behind a real seam). The double sits behind the SHIPPED
/// `set_sync_ops` seam, so the driver, the matcher, the counters and the burst
/// under test are all production code; the transport-backed halves live in
/// `crates/cerulion_core/tests/sync_per_set_iox2_test.rs`.
#[cfg(test)]
mod align_driver_tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::Mutex;

    /// One input's queue, modelling the two-slot lifecycle the real subscriber
    /// implements: a head (`frozen`), a staged next (`next_head`), and the
    /// iceoryx2 queue behind them.
    #[derive(Default)]
    struct FakeInput {
        queue: VecDeque<u64>,
        head: Option<u64>,
        staged: Option<u64>,
    }

    impl FakeInput {
        fn with(frames: &[u64]) -> Self {
            Self {
                queue: frames.iter().copied().collect(),
                ..Default::default()
            }
        }

        /// R-promote then queue-pop: the staged next is ALWAYS preferred, which
        /// is the in-order half of the contract.
        fn refill(&mut self) -> Option<u64> {
            self.head = self.staged.take().or_else(|| self.queue.pop_front());
            self.head
        }
    }

    /// The double.
    ///
    /// **The failure model mirrors the real FFI SPLIT, and that is load-bearing.**
    /// `FillBoundary` / `FillRefill` do NOT cross `cerulion_node_sync_head_op` —
    /// they ride the pre-existing drain symbols — so a fixture whose
    /// `sync_head_op` fails still FILLS its heads. A double that failed the
    /// fills too would never reach the verdict loop at all: completeness would
    /// fail first, no gate would ever be scanned, and the arms that exist to
    /// pin the gate's failure POSITION would silently test nothing. (Measured:
    /// with fills failing, a variant that maps a failed GATE probe to `None`
    /// passes the whole suite.)
    struct FakeTransport {
        inputs: IndexMap<String, FakeInput>,
        /// Fail `sync_head_op` for this ONE input (the asymmetric regime: an
        /// input-name wiring/drift bug). `None` = no input filter.
        fail_head_ops_for: Option<String>,
        /// Fail `sync_head_op` for EVERY input (the poisoned-`NODES` regime).
        fail_all_head_ops: bool,
        /// Fail only these op KINDS, whichever input. Empty = every kind.
        fail_only_kinds: Vec<SyncHeadOp>,
        ops_run: u64,
        /// Ops that only a DESCENT issues — the discriminator for the restore
        /// arm, where a probe is not merely wasteful but a steal (it would take
        /// a live queued frame that belongs to the NEXT set).
        descent_ops: u64,
    }

    impl FakeTransport {
        fn should_fail(&self, input: &str, op: SyncHeadOp) -> bool {
            // A fill rides the drain symbols, not `sync_head_op`.
            if matches!(op, SyncHeadOp::FillBoundary | SyncHeadOp::FillRefill) {
                return false;
            }
            let input_matches =
                self.fail_all_head_ops || self.fail_head_ops_for.as_deref() == Some(input);
            let kind_matches =
                self.fail_only_kinds.is_empty() || self.fail_only_kinds.contains(&op);
            input_matches && kind_matches
        }

        fn run(&mut self, input: &str, op: SyncHeadOp) -> SyncOpAnswer {
            self.ops_run += 1;
            if self.should_fail(input, op) {
                return SyncOpAnswer::Failed;
            }
            let Some(state) = self.inputs.get_mut(input) else {
                return SyncOpAnswer::Failed;
            };
            if matches!(
                op,
                SyncHeadOp::ProbeNext | SyncHeadOp::PeekNext | SyncHeadOp::Advance
            ) {
                self.descent_ops += 1;
            }
            match op {
                // The head is consumed by every fire in these arms (the macro
                // shape), so a fill always advances.
                SyncHeadOp::FillBoundary | SyncHeadOp::FillRefill => match state.refill() {
                    Some(ts) => SyncOpAnswer::Head(ts),
                    None => SyncOpAnswer::Nothing,
                },
                SyncHeadOp::ProbeNext => {
                    if state.staged.is_some() || !state.queue.is_empty() {
                        SyncOpAnswer::Present
                    } else {
                        SyncOpAnswer::Nothing
                    }
                }
                SyncHeadOp::PeekNext => {
                    if state.staged.is_none() {
                        state.staged = state.queue.pop_front();
                    }
                    match state.staged {
                        Some(ts) => SyncOpAnswer::Stamp(ts),
                        None => SyncOpAnswer::Nothing,
                    }
                }
                SyncHeadOp::Advance => match state.refill() {
                    Some(ts) => SyncOpAnswer::Head(ts),
                    None => SyncOpAnswer::Nothing,
                },
                SyncHeadOp::Void => SyncOpAnswer::Nothing,
            }
        }
    }

    struct Harness {
        scheduler: Scheduler,
        transport: Arc<Mutex<FakeTransport>>,
        fires: Arc<AtomicU64>,
    }

    /// Build a 2-input bounded-Sync node whose ops run against the double.
    fn harness(
        a: &[u64],
        b: &[u64],
        window_ms: Option<u64>,
        fail_head_ops_for: Option<&str>,
        fail_all_head_ops: bool,
    ) -> Harness {
        harness_with(a, b, window_ms, fail_head_ops_for, fail_all_head_ops, &[])
    }

    fn harness_with(
        a: &[u64],
        b: &[u64],
        window_ms: Option<u64>,
        fail_head_ops_for: Option<&str>,
        fail_all_head_ops: bool,
        fail_only_kinds: &[SyncHeadOp],
    ) -> Harness {
        let mut inputs = IndexMap::new();
        inputs.insert("a".to_string(), FakeInput::with(a));
        inputs.insert("b".to_string(), FakeInput::with(b));
        let transport = Arc::new(Mutex::new(FakeTransport {
            inputs,
            fail_head_ops_for: fail_head_ops_for.map(str::to_string),
            fail_all_head_ops,
            fail_only_kinds: fail_only_kinds.to_vec(),
            ops_run: 0,
            descent_ops: 0,
        }));

        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        let fires = Arc::new(AtomicU64::new(0));
        let fires_cb = Arc::clone(&fires);
        scheduler
            .add_node(NodeConfig {
                id: "fuse".to_string(),
                policy: TriggerPolicy::Sync {
                    inputs: vec!["a".to_string(), "b".to_string()],
                    window: window_ms.map(Duration::from_millis),
                },
                callback: Box::new(move || {
                    fires_cb.fetch_add(1, Ordering::Relaxed);
                }),
            })
            .expect("add sync node");

        let ops_transport = Arc::clone(&transport);
        scheduler
            .set_sync_ops(
                "fuse",
                &["a".to_string(), "b".to_string()],
                move |input, op| {
                    ops_transport
                        .lock()
                        .expect("fake transport poisoned")
                        .run(input, op)
                },
            )
            .expect("install sync ops");

        Harness {
            scheduler,
            transport,
            fires,
        }
    }

    impl Harness {
        /// One boundary: align, then step — exactly the runtime's order.
        fn boundary(&mut self) {
            let _ = self
                .scheduler
                .align_sync_heads("fuse", SyncAlignSite::Boundary);
            self.scheduler.step_ms(1);
        }

        fn skips(&self, input: &str) -> u64 {
            self.scheduler
                .node_handle("fuse")
                .expect("handle")
                .sync_closer_skip_count(input)
        }

        fn unmatched(&self, input: &str) -> u64 {
            self.scheduler
                .node_handle("fuse")
                .expect("handle")
                .sync_unmatched_discard_count(input)
        }

        fn head_refusals(&self, input: &str) -> u64 {
            self.scheduler
                .node_handle("fuse")
                .expect("handle")
                .sync_head_refusals(input)
        }

        fn signal_failed(&self) -> u64 {
            self.scheduler
                .node_handle("fuse")
                .expect("handle")
                .signal_failed_count()
        }

        fn ops_run(&self) -> u64 {
            self.transport
                .lock()
                .expect("fake transport poisoned")
                .ops_run
        }

        fn descent_ops(&self) -> u64 {
            self.transport
                .lock()
                .expect("fake transport poisoned")
                .descent_ops
        }
    }

    /// `sync_head_refusals` COUNTS an offer that landed on an occupied head — and
    /// counting is all it does.
    ///
    /// The counter is scoped to the PER-SET path, which is why this harness installs
    /// the head ops: the bump reads `sync_ops.is_some()`, and a node WITHOUT the seam
    /// keeps the legacy latest-wins insert, where the second offer REPLACES the first
    /// and nothing is refused. A caller on the per-set path is using the API
    /// CORRECTLY — the head is a MEMBER of the set being formed, written once and
    /// released by a fire or by the align pass's own unmatchable discard (bounded
    /// mode; a `window: None` node has no discard path, so only a fire releases a
    /// head) — so a second offer at a different stamp simply does not displace it.
    /// Reporting that as an `Err` would make correct use look broken, and bumping
    /// `signal_failed` would fire a counter whose documented meaning is "wiring
    /// desync" — so the ONLY signal is this counter, which is why it needs an oracle
    /// rather than a reader's trust.
    ///
    /// Hand oracle throughout: the exact count after each offer, the two shapes that
    /// must NOT count, and the two counters that must not move.
    #[test]
    fn a_second_offer_on_a_filled_head_is_counted_and_is_not_a_failure() {
        let mut h = harness(&[], &[], Some(50), None, false);

        // The FIRST offer fills an empty head: nothing refused yet.
        assert!(h.scheduler.signal_sync_input("fuse", "a", 100).is_ok());
        assert_eq!(
            h.head_refusals("a"),
            0,
            "filling an empty head is not a refusal"
        );

        // A DIFFERENT stamp on the now-filled head: counted, and still `Ok(())`.
        assert!(
            h.scheduler.signal_sync_input("fuse", "a", 200).is_ok(),
            "a refused offer is Ok(()) — correct use, not an error"
        );
        assert_eq!(h.head_refusals("a"), 1);

        // It keeps counting, one per refused offer.
        assert!(h.scheduler.signal_sync_input("fuse", "a", 300).is_ok());
        assert_eq!(h.head_refusals("a"), 2);

        // CONTROL 1 — the SAME stamp is not a refusal. Without this arm a reporter
        // that counted every offer onto a filled head would pass every assert above.
        assert!(h.scheduler.signal_sync_input("fuse", "a", 100).is_ok());
        assert_eq!(
            h.head_refusals("a"),
            2,
            "re-offering the stamp the head already holds displaces nothing and \
             refuses nothing"
        );

        // CONTROL 2 — per INPUT, not per node: `b` has been offered nothing, and its
        // own first offer fills its own empty head.
        assert_eq!(h.head_refusals("b"), 0);
        assert!(h.scheduler.signal_sync_input("fuse", "b", 100).is_ok());
        assert_eq!(h.head_refusals("b"), 0, "b filled its own empty head");
        assert_eq!(
            h.head_refusals("a"),
            2,
            "b's traffic does not touch a's count"
        );

        // The counter is the WHOLE signal: nothing else moved. `signal_failed` means
        // "wiring desync" and a refused offer is not one; the skip/discard counters
        // describe the MATCHER, which a signal never runs.
        assert_eq!(
            h.signal_failed(),
            0,
            "a refused offer is not a signal failure"
        );
        assert_eq!(h.skips("a"), 0);
        assert_eq!(h.unmatched("a"), 0);
    }

    /// The NEGATIVE half, and the one the documented guidance rests on: a Sync
    /// node WITHOUT the head ops never refuses an offer, so `sync_head_refusals`
    /// reads 0 there no matter how many frames are offered.
    ///
    /// This is the path an out-of-crate embedder is on — the ops installer is
    /// crate-private, so a `Scheduler` built by hand can never reach the per-set
    /// branch — and it is also the path the runtime itself signals (a node WITH
    /// the seam is driven by the align pass instead). The semantics are the
    /// OPPOSITE of the per-set branch, which is exactly why documenting the
    /// counter as an embedder's diagnostic was wrong: here the second offer
    /// REPLACES the first, so the node fires on the frame offered LAST.
    ///
    /// Hand oracle: the head's own stamp after each offer, plus the three
    /// counters that must stay 0.
    #[test]
    fn a_node_without_head_ops_replaces_the_head_and_never_counts_a_refusal() {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        scheduler
            .add_node(NodeConfig {
                id: "fuse".to_string(),
                policy: TriggerPolicy::Sync {
                    inputs: vec!["a".to_string(), "b".to_string()],
                    window: Some(Duration::from_millis(50)),
                },
                callback: Box::new(|| {}),
            })
            .expect("add sync node");
        // DELIBERATELY no `set_sync_ops`: `sync_ops` stays `None`, which is what
        // sends every offer below down the latest-wins arm.

        let head_ts = |s: &Scheduler| -> Option<u64> {
            s.nodes
                .get("fuse")
                .expect("node")
                .sync_heads
                .get("a")
                .filter(|h| h.is_filled())
                .map(|h| h.ts)
        };
        let refusals = |s: &Scheduler| -> u64 {
            s.node_handle("fuse")
                .expect("handle")
                .sync_head_refusals("a")
        };

        assert!(scheduler.signal_sync_input("fuse", "a", 100).is_ok());
        assert_eq!(head_ts(&scheduler), Some(100), "the empty head takes 100");
        assert_eq!(refusals(&scheduler), 0);

        // A DIFFERENT stamp onto the FILLED head — the exact shape the per-set
        // branch counts. Here it REPLACES, and counts nothing.
        assert!(scheduler.signal_sync_input("fuse", "a", 200).is_ok());
        assert_eq!(
            head_ts(&scheduler),
            Some(200),
            "latest-wins: the second offer displaced the first"
        );
        assert_eq!(
            refusals(&scheduler),
            0,
            "no seam, no refusal — the counter cannot move on this path"
        );

        assert!(scheduler.signal_sync_input("fuse", "a", 300).is_ok());
        assert_eq!(head_ts(&scheduler), Some(300));
        assert_eq!(refusals(&scheduler), 0);

        let handle = scheduler.node_handle("fuse").expect("handle");
        assert_eq!(
            handle.signal_failed_count(),
            0,
            "a latest-wins insert is not a signal failure either"
        );
        assert_eq!(handle.sync_closer_skip_count("a"), 0);
        assert_eq!(handle.sync_unmatched_discard_count("a"), 0);
    }

    /// The install-time refusal: a list that disagrees with the node's declared
    /// trigger set is REFUSED, because every verdict the matcher issues is a
    /// DECLARATION POSITION and two lists would make a verdict describe one
    /// input while the op moved another.
    #[test]
    fn set_sync_ops_refuses_an_input_list_that_disagrees_with_the_declared_set() {
        let mut h = harness(&[], &[], Some(50), None, false);
        let err = h
            .scheduler
            .set_sync_ops("fuse", &["b".to_string(), "a".to_string()], |_, _| {
                SyncOpAnswer::Nothing
            })
            .expect_err("a reordered list must be refused");
        let msg = format!("{err}");
        assert!(
            msg.contains("DECLARATION POSITION"),
            "the refusal must name WHY order matters; got: {msg}"
        );
    }

    /// The TOTAL failure regime (a poisoned cdylib `NODES` mutex:
    /// every `sync_head_op` returns a failure code).
    ///
    /// The fills still work — they ride the drain symbols, not
    /// `cerulion_node_sync_head_op` — so the heads fill and a complete
    /// in-window tuple forms. The ARGMIN's own probe is then the FIRST head op
    /// issued, it fails, and the failed-probe policy resolves it to `None` at that site: nothing
    /// to descend to, so the node FIRES GREEDY. Sound membership, loudly
    /// degraded, never a spin and never a wrong set.
    ///
    /// The gate is never scanned in this regime — which is exactly why the
    /// gate-position test below has to exist separately.
    #[test]
    fn a_total_op_failure_regime_fires_greedy_and_terminates_bounded() {
        let mut h = harness(&[1_000_000, 2_000_000], &[10_000_000], Some(50), None, true);
        h.boundary();
        assert_eq!(
            h.fires.load(Ordering::Relaxed),
            1,
            "a complete in-window tuple still fires: the argmin's failed probe \
             means `nothing to descend to`, which is greedy membership"
        );
        assert!(
            h.ops_run() <= 8,
            "a failed op TERMINATES the pass rather than being re-issued; got \
             {} ops on one boundary, which is a spin",
            h.ops_run()
        );
        assert_eq!(
            h.skips("a"),
            0,
            "a failure never advances a frame — descent is disabled, not \
             fabricated"
        );
    }

    /// A failed MUTATING op TERMINATES the pass instead of being re-issued.
    ///
    /// Reaching the `NeedStamp` failure needs a pass that gets PAST the gate,
    /// so only the PEEK is failed here: the argmin probes healthy (`Present`),
    /// `b` is genuinely scarce so the gate passes on real evidence, and the pop
    /// that follows fails. The tuple is complete and in-window at that point
    /// (death already ran), so firing it is sound greedy membership.
    ///
    /// A driver that retried instead would re-issue the same failing op against
    /// an unchanged scratch entry until the pass budget ran out, and answer
    /// `Incomplete` — a node that stops firing entirely because one op is
    /// broken.
    #[test]
    fn a_failed_mutating_op_terminates_the_pass_instead_of_retrying() {
        let mut h = harness_with(
            &[0, 1_000_000, 2_000_000],
            &[9_000_000],
            None,
            None,
            true,
            &[SyncHeadOp::PeekNext],
        );
        h.boundary();
        assert_eq!(
            h.fires.load(Ordering::Relaxed),
            1,
            "the current tuple is complete and in-window, so the terminated \
             pass fires it greedily"
        );
        assert!(
            h.ops_run() <= 8,
            "the failing peek is issued ONCE; got {} ops, which is a retry loop",
            h.ops_run()
        );
    }

    /// A burst that ends with a COMPLETE aligned set still in the heads reports
    /// DUE-NOW — the mid-burst defer, which is the burst's own half of the hint
    /// (`a_set_deferred_before_its_first_fire_still_reports_due_now` covers the boundary's half, and the two are set at different
    /// sites, so one arm cannot cover both).
    #[test]
    fn a_burst_that_stops_with_a_set_still_aligned_reports_due_now() {
        let mut h = harness(
            &[1_000_000, 2_000_000],
            &[1_100_000, 2_100_000],
            Some(50),
            None,
            false,
        );
        // Defer from the THIRD consultation on: once in `decide_node`, once
        // before fire 1, then defer before fire 2 — so the burst stops with the
        // second set aligned and unfired.
        let calls = Arc::new(AtomicU64::new(0));
        let calls_check = Arc::clone(&calls);
        h.scheduler
            .set_pre_fire_check("fuse", move |_now_ns| {
                calls_check.fetch_add(1, Ordering::Relaxed) >= 2
            })
            .expect("install the defer gate");

        h.boundary();
        assert_eq!(
            h.fires.load(Ordering::Relaxed),
            1,
            "exactly one set fires before the gate defers the burst"
        );
        assert_eq!(
            h.scheduler.ns_until_next_fire(0),
            Some(0),
            "the second set is aligned and unfired — DUE NOW. Without the \
             burst raising the hint, the live loop sizes its wake as if the \
             node owed nothing"
        );
    }

    /// THE GATE-POSITION KILL.
    ///
    /// A=[1,2,3], B=[10,20,30], and every op on B FAILS while A's are healthy.
    /// Three complete sets are present among arrived frames, so the
    /// never-destroy-a-complete-set rule demands three fires and ZERO skips.
    ///
    /// Under the position-BLIND policy (a `Failed` probe resolving to `None`
    /// everywhere), B's failed gate probe VOUCHES FOR SCARCITY on an input
    /// nobody verified: the gate passes sticky, descent runs into the unprobed
    /// backlog and eats it into ONE fire `(3,10)` plus two PASSED-OVER skips.
    /// The failed-probe policy resolves a failed GATE probe to `Present` instead — that input
    /// REFUSES the gate — so the backlog is served in order.
    ///
    /// The total-failure test above structurally cannot see this: there the ARGMIN's own probe fails
    /// first and the gate is never scanned at all.
    #[test]
    fn a_failed_gate_probe_refuses_the_gate_rather_than_vouching_for_scarcity() {
        // Only B's HEAD OPS fail; its head still FILLS (fills ride the drain
        // symbols). So three complete arrived sets exist and all three must
        // fire, in order, with nothing skipped.
        let mut h = harness(
            &[1_000_000, 2_000_000, 3_000_000],
            &[10_000_000, 20_000_000, 30_000_000],
            Some(50),
            Some("b"),
            false,
        );
        h.boundary();
        assert_eq!(
            h.fires.load(Ordering::Relaxed),
            3,
            "every gate scan hits B's failed probe, which REFUSES the gate, so \
             the backlog serves in order. Under the position-blind policy B's \
             failure reads as SCARCITY, the gate passes, and the backlog is \
             eaten into ONE fire"
        );
        assert_eq!(
            h.skips("a"),
            0,
            "a failed probe on B must never enable a descent on A — that is a \
             probe failure destroying arrived complete in-window sets, which \
             must never happen"
        );
        assert_eq!(
            h.unmatched("a"),
            0,
            "nothing here is unmatchable — every frame is in a set"
        );
    }

    /// The ANTI-TAUTOLOGY control for the gate-position test: the identical stimulus with NO
    /// failures serves all three sets and skips nothing, so the assertion above
    /// is not satisfied by a driver that simply never descends.
    #[test]
    fn the_same_backlog_with_no_failures_serves_three_sets_and_skips_nothing() {
        let mut h = harness(&[1, 2, 3], &[10, 20, 30], Some(50), None, false);
        h.boundary();
        assert_eq!(
            h.fires.load(Ordering::Relaxed),
            3,
            "three complete arrived sets must yield three fires in ONE step"
        );
        assert_eq!(h.skips("a"), 0, "the gate refuses while both hold a next");
        assert_eq!(h.skips("b"), 0, "likewise on b");
    }

    /// A scarce partner IS descended on — the positive half of the gate, so the
    /// arms above cannot pass a driver that refuses every descent.
    #[test]
    fn a_scarce_partner_lets_the_walk_descend_and_counts_every_passed_over_frame() {
        let mut h = harness(&[0, 1, 2, 3, 4], &[9], None, None, false);
        h.boundary();
        assert_eq!(
            h.fires.load(Ordering::Relaxed),
            1,
            "b is scarce, so exactly one set can form"
        );
        assert_eq!(
            h.skips("a"),
            4,
            "a@0..a@3 were each passed over for a nearer member"
        );
    }

    /// THE EMITTED-TOMBSTONE CAPTURE RULE, both halves in one body.
    ///
    /// Capture exports ONLY `Filled` heads. A head consumed by a fire is an
    /// `Emitted` TOMBSTONE, and the framework section is a stamp-only map with
    /// no state channel, so a tombstone is INEXPRESSIBLE there and the writer
    /// must choose. Skipping reproduces the earlier cleared-on-fire capture
    /// byte-for-byte.
    ///
    /// The natural export-everything implementation is wrong on the COMMON
    /// case: a fired step's two tombstones would serialise as bare stamps,
    /// restore would mint two unbacked `Filled` heads, completeness would hold
    /// over them, and the resumed run would fire ONE PHANTOM SET the original
    /// continuation never fired — a fire-sequence divergence, since a fire is
    /// real in the trace even when its body collapses.
    ///
    /// The PRE-fire half is the anti-tautology: without it, "the section is
    /// empty" is satisfied by a capture that exports nothing at all.
    #[test]
    fn a_capture_taken_after_a_fire_carries_no_heads_while_one_taken_before_it_carries_both() {
        let mut h = harness(&[1_000_000], &[10_000_000], Some(50), None, false);

        // Align but do NOT step: both heads are Filled and the set is owed.
        let _ = h
            .scheduler
            .align_sync_heads("fuse", SyncAlignSite::Boundary);
        let before = h
            .scheduler
            .node_framework_state("fuse")
            .expect("framework state");
        assert_eq!(
            before.sync_input_timestamps.len(),
            2,
            "a capture taken with a set ALIGNED must carry both members, or the \
             emptiness below proves nothing"
        );

        h.scheduler.step_ms(1);
        assert_eq!(h.fires.load(Ordering::Relaxed), 1, "the aligned set fires");

        let after = h
            .scheduler
            .node_framework_state("fuse")
            .expect("framework state");
        assert!(
            after.sync_input_timestamps.is_empty(),
            "a checkpoint taken after a fired step must carry NO heads — the \
             consumed members are tombstones, and exporting them mints a \
             phantom set on resume; got {:?}",
            after.sync_input_timestamps
        );
    }

    /// THE RESTORE BOUNDARY.
    ///
    /// A head restored from a bag names a stamp but NO live frame. Descent is
    /// disabled for the whole boundary: there is nothing to probe past, and a
    /// live queued frame belongs to the NEXT set, so probing would STEAL it.
    ///
    /// The oracle is the descent-op count — zero — which is stronger than the
    /// fire count, because a driver that probed and then declined to advance
    /// would still have popped a frame into the staged slot and reordered that
    /// input's delivery.
    #[test]
    fn a_restored_boundary_issues_no_descent_op_even_with_a_queue_behind_it() {
        // Both inputs have DEEP queues, so an ungated driver has every reason
        // to descend: the restored stamps are 200 ms apart inside a 500 ms
        // window, and nearer frames are queued on both.
        let mut h = harness(
            &[100_000_000, 110_000_000],
            &[300_000_000, 310_000_000],
            Some(500),
            None,
            false,
        );
        let mut per_node = std::collections::BTreeMap::new();
        let mut heads = std::collections::BTreeMap::new();
        heads.insert("a".to_string(), 100_000_000u64);
        heads.insert("b".to_string(), 300_000_000u64);
        per_node.insert("fuse".to_string(), heads);
        h.scheduler.restore_sync_input_timestamps(&per_node);

        let before = h.descent_ops();
        // Measure the BOUNDARY's own align in isolation. The rule is scoped to
        // the boundary that carries an unbacked head: once the restored set has
        // fired, the burst's refill alignment holds LIVE heads and may descend
        // like any other — so stepping first would fold a legitimate descent
        // into the count and make the assertion measure the wrong thing.
        let _ = h
            .scheduler
            .align_sync_heads("fuse", SyncAlignSite::Boundary);
        assert_eq!(
            h.descent_ops(),
            before,
            "a boundary with an unbacked head must issue NO probe, peek or \
             advance — each would steal a queued frame that belongs to the \
             NEXT set"
        );
        h.scheduler.step_ms(1);
        assert_eq!(
            h.fires.load(Ordering::Relaxed),
            3,
            "descent is disabled on the RESTORING boundary, completeness is \
             not: the restored set fires, and the two queued pairs then serve \
             in order within the same step through the burst's own alignments, \
             whose heads are LIVE. The cost of the restore rule is exactly one \
             boundary of greedy membership — not a lost set"
        );
    }

    /// THE FIRST-FIRE DEFER REPORTS DUE-NOW.
    ///
    /// `decide_node`'s `pre_fire_check` gate sits ABOVE the policy match, so a
    /// deferred node reaches neither the Sync arm nor the burst. A hint raised
    /// only by the burst would read FALSE for a set the BOUNDARY aligned and a
    /// `throttle_ms` / `block` gate deferred before fire 1 — the node owes a
    /// fire, reports nothing due, and waits for an unrelated publish or the
    /// 250 ms liveliness cap where a Data node recovers at the 1 ms floor.
    ///
    /// The oracle is `ns_until_next_fire` (what the live loop actually sizes
    /// its wake on) AND the operator-facing mirror, because a hint that moved
    /// without reaching the wake sizing would be invisible where it matters.
    /// The undeferred CONTROL runs the identical stimulus one flag apart: if
    /// the hint were simply raised unconditionally, that control would report
    /// due-NOW with nothing left to fire.
    #[test]
    fn a_set_deferred_before_its_first_fire_still_reports_due_now() {
        let mut h = harness(&[1_000_000], &[10_000_000], Some(50), None, false);
        h.scheduler
            .set_pre_fire_check("fuse", |_now_ns| true)
            .expect("install the defer gate");

        let _ = h
            .scheduler
            .align_sync_heads("fuse", SyncAlignSite::Boundary);
        h.scheduler.step_ms(1);

        assert_eq!(
            h.fires.load(Ordering::Relaxed),
            0,
            "the pre-fire gate deferred the fire — that is the shape under test"
        );
        assert_eq!(
            h.scheduler.ns_until_next_fire(0),
            Some(0),
            "a complete set the boundary aligned and a gate deferred is DUE NOW; \
             without it the live loop sizes its wake as if nothing were owed"
        );
        assert!(
            h.scheduler
                .node_handle("fuse")
                .expect("handle")
                .sync_backlog_pending(),
            "the operator-facing mirror must agree with the wake sizing"
        );

        // CONTROL: the identical stimulus with no gate. The set fires, nothing
        // is left, and the node must NOT report due-NOW — which is what stops
        // the assertions above being satisfied by an always-true hint.
        let mut clean = harness(&[1_000_000], &[10_000_000], Some(50), None, false);
        clean.boundary();
        assert_eq!(
            clean.fires.load(Ordering::Relaxed),
            1,
            "the control fires its one set"
        );
        assert_eq!(
            clean.scheduler.ns_until_next_fire(0),
            None,
            "with the set fired and the queues empty, nothing is owed"
        );
    }

    /// Window DEATH is classified before descent, and on its own counter.
    #[test]
    fn a_window_death_is_counted_unmatchable_and_leaves_the_skip_counter_alone() {
        // Stamps are NANOSECONDS while the window is MILLISECONDS: a@0 sits
        // 200 ms from b, four times the 50 ms window, so it is provably in no
        // set. (Writing these as bare small integers put the whole span inside
        // the window and silently routed the arm through descent instead —
        // caught by running it.)
        let mut h = harness(&[0, 210_000_000], &[200_000_000], Some(50), None, false);
        h.boundary();
        assert_eq!(
            h.fires.load(Ordering::Relaxed),
            1,
            "the in-window pair fires after the unmatchable head is discarded"
        );
        assert_eq!(
            (h.unmatched("a"), h.skips("a")),
            (1, 0),
            "a@0 is provably in no set: UNMATCHABLE, not PASSED-OVER. A \
             descent-first matcher reaches the same set counting (0, 1)"
        );
    }
}
