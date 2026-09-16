//! The PURE rank-local re-derivation VERIFIER.
//!
//! Replay fires are **trace-driven**: the executor does not
//! re-decide what fires, it replays what the recording says fired
//! (re-derivation is the VERIFIER's job, never the fire
//! driver). That decision is forced rather than chosen, and it leaves a hole
//! this module fills: with nothing re-deciding, nothing would notice that a
//! candidate's `period_ms`, `throttle_ms` or `sync_window_ms` DECLARATION moved
//! since the recording. So the structural verdict — "would the schedule the bag
//! records have HAPPENED, given the clock and the reads the bag records?" — is
//! produced HERE, by re-deriving each **rank-local** decider and comparing it
//! POSITIONALLY against the recorded fire stream.
//!
//! # What is re-derived, and what is deliberately NOT
//!
//! The line is drawn at re-derivability, not at convenience:
//!
//! | decider | posture | why |
//! |---|---|---|
//! | `period_ms` schedule | RE-DERIVED | a function of the recorded clock alone |
//! | `throttle_ms` | RE-DERIVED (one direction — see below) | a function of the recorded clock + the node's own last fire |
//! | `sync_window_ms` / `unbounded_sync` | RE-DERIVED | a function of the recorded reads' WIRE stamps |
//! | FIFO pop counts | RE-DERIVED | a function of the read log's `popped` |
//! | the `block` pre-fire gate | **NOT re-derived** | cross-rank occupancy at decision instants the log records NOTHING about |
//!
//! The block edge gets a **conservation assert** instead ([`ConservationFinding`]):
//! per-edge credit conservation must hold under SOME legal interleave. It is
//! LOUD and typed, and it is **never a gate** — [`RederivationReport::is_clean`]
//! and [`RederivationReport::divergence_class`] both ignore it, because a
//! conservation breach is evidence about the recording, not a verdict on the
//! candidate's schedule. Feeding the block gate into re-derivation instead
//! would manufacture spurious divergence on exactly the block-paced graphs
//! MappedCredit ships for — which is the failure the verifier-only posture exists to prevent,
//! and which
//! `a_block_paced_period_producer_stands_down_instead_of_diverging` pins.
//!
//! # No second copy of any decider (the duplicated-rule class)
//!
//! Every rule this module needs is **called**, never restated. The Period
//! catch-up cap resolution is
//! [`cerulion_core::scheduler::catchup_clamp::effective_max_catchup`]; the
//! Period ADVANCE is
//! [`cerulion_core::scheduler::catchup_clamp::period_advance`]; the Sync fire
//! predicate is [`cerulion_core::scheduler::sync_aligned`]; the throttle defer
//! is [`cerulion_core::graph::runtime::throttle_defers`].
//!
//! Three of those four were MIRRORS when this module was written, because the
//! core's own copy was private — `Scheduler::check_sync`, the Period advance
//! inside `Scheduler::decide_node`, and the throttle disjunct captured into the
//! pre-fire closure. Nothing compiled against both spellings, so the two were
//! free to drift on exactly the rules whose job is to agree, and the Period pair
//! DID (`catchup_clamp`'s own doc records where). Each rule has since moved to a
//! callable place in the core and the local functions (`period_step`,
//! `sync_aligned`, `throttle_defers`) are thin delegating wrappers kept for
//! their module-local return shapes and their doc.
//!
//! What the wrappers still buy is the TESTING pairing: each is exposed so an
//! oracle can pin the predicate and the whole-engine verdict against ONE hand
//! oracle in one body. Two assertions over one oracle cannot drift into two
//! different beliefs about one rule — and now they cannot drift into two
//! different IMPLEMENTATIONS of it either.
//!
//! # Coverage — a stand-down is REPORTED, never silent
//!
//! Every place this module declines to judge emits a [`StandDown`] naming the
//! node, the decider and the reason. A verifier that quietly skipped a node
//! would report "compared clean" about a node it never looked at, which is the
//! rule ("a verdicting verifier cannot treat a deterministic-
//! truncation hole as compared clean") applied to every other hole as well.
//!
//! # Vocabulary
//!
//! A schedule mismatch is [`DivergenceClass::FireSchedule`] and NOTHING else —
//! this module mints no phrase of its own. The
//! renderer stays [`crate::replay_engine::divergence_class_phrase`].
//!
//! # Purity
//!
//! No clock, no filesystem, no transport: the inputs are numbers the replay
//! executor reads out of the bag, the output is a decision. That is what lets
//! every branch a healthy robot never reaches be oracle-tested.
//!
//! # Platform gate
//!
//! `#[cfg(unix)]` is INHERITED, not intrinsic: the module is pure and would
//! compile anywhere, but the one vocabulary it is required to slot into
//! ([`DivergenceClass`]) lives in the Unix-gated [`crate::replay_engine`]. A
//! second copy of that enum here is exactly the drift the rider forbids, so the
//! gate rides along. If the vocabulary ever un-gates, so can this.

use std::collections::BTreeMap;
use std::fmt;

use cerulion_core::read_outcome::{ReadOutcomeKind, ReadSiteRole};
use cerulion_core::scheduler::catchup_clamp::effective_max_catchup;

use crate::replay_engine::DivergenceClass;

/// Above this many fires in ONE step the comparison drops to COUNTS and stops
/// materialising per-fire instants.
///
/// A real Period burst is bounded by one step's clock advance; a CORRUPT bag
/// carrying a clock jump of days against a 1 ms interval is not, and a verifier
/// must refuse to allocate on a number it read out of a file. The count
/// comparison is unaffected, so a divergence is still caught — only its
/// rendering thins.
pub const PERIOD_BURST_RENDER_CAP: u32 = 1024;

/// The per-step Sync burst ceiling — `cerulion_core`'s own
/// constant, never a mirror.
///
/// `Scheduler::decide_node`'s Sync arm answers an aligned step with
/// `FireKind::Sync { max_sets: DATA_PENDING_CARRY_CLAMP }` and
/// `tick_sync_burst` stops at it, so a step recording more fires than this is
/// impossible under EITHER delivery discipline (legacy fires exactly once;
/// per-set fires at most this many times).
///
/// A RE-EXPORT rather than a re-derivation from
/// `graph::topology::MAX_CONSUMER_DEPTH`: the tie between the burst cap and the
/// consumer-depth ceiling is the core's own semantic claim, and restating it
/// here would be a second copy of a rule whose whole job is to agree — the
/// duplicated-rule class this module keeps deleting. A re-export cannot drift, so it
/// needs no drift test; the LITERAL is pinned where it lives, by
/// `cerulion_core::graph::topology::tests::max_consumer_depth_pinned_at_64`.
pub const SYNC_BURST_MAX_SETS: u32 = cerulion_core::scheduler::DATA_PENDING_CARRY_CLAMP as u32;

/// How many CANDIDATE stamps `verify_sync` will hold for one
/// trigger input in one step.
///
/// The [`PERIOD_BURST_RENDER_CAP`] precedent, for the same reason: a verifier
/// must not allocate on a number it read out of a file. A step's candidate
/// count is already bounded by the read-log record budget, but a bag is
/// untrusted input and that budget is not this module's to rely on.
///
/// When the cap bites the feasibility arm takes the CONSERVATIVE branch — the
/// transversal is treated as EXISTING and no finding is emitted — because a
/// verifier must not convict because it ran out of room. The SUPPLY arm's
/// carried term is the stored vector's LENGTH, so the cap bounds it as well —
/// which is why EVERY input's carry is cut at its depth ceiling first
/// ([`NodeDecl::trigger_input_capacities`]): a carry that reached this cap would
/// pin arm A at the burst cap and skip arm B for the rest of the run. The cap
/// is therefore a per-STEP condition (this step's own records overflowed the
/// vector) with no state carried across steps.
pub const SYNC_CANDIDATE_CAP: usize = 1024;

/// The most heads a trigger input's CROSS-STEP carry may hold — half the
/// candidate cap, so a carried set can never fill the feasibility vector on
/// its own and the arm-B admission cap stays a per-STEP condition (a carry
/// ceiling clamped AT the cap filled the vector, the first
/// new record of the next step tripped `candidates_capped`, and every later
/// fire was accepted without a feasibility check — an out-of-window Sync fire
/// replayed CLEAN). An input whose physical ceiling (`capacity + 2`) exceeds
/// this room is not truncated: the node STANDS DOWN
/// ([`StandDownReason::QueueExceedsVerifierRoom`]) before its first step is
/// judged — a loud "no claim", never a silent decline, never a cut that drops
/// heads the matcher holds, and never a verdict on the steps that convict
/// before the carry would have been cut.
pub const SYNC_CARRY_ROOM: usize = SYNC_CANDIDATE_CAP / 2;

// Every input's cross-step carry is cut at its physical ceiling — its
// effective capacity (declared depth × publisher connections) + 2 (the
// `verify_sync` carry-forward below) — and that ceiling must fit the carry
// ROOM for the arm-B cap to stay a per-step condition. A single-connection
// input ALWAYS fits: the two constants live in different crates, so this pins
// the claim where a bump to either would fail to compile rather than silently
// turn every single-writer node into a stand-down. (A many-writer bus can
// exceed the room; that node stands down loudly rather than being cut.)
const _: () = assert!(
    cerulion_core::graph::topology::MAX_CONSUMER_DEPTH + 2 <= SYNC_CARRY_ROOM,
    "a single-connection input's depth + 2 ceiling must fit SYNC_CARRY_ROOM"
);

// ---------------------------------------------------------------------------
// Vocabulary
// ---------------------------------------------------------------------------

/// WHICH rule produced a finding or declined to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Decider {
    /// The `period_ms` whole-interval schedule.
    Period,
    /// The `throttle_ms` producer rate cap.
    Throttle,
    /// `sync_window_ms` / `unbounded_sync` alignment.
    Sync,
    /// Per-message FIFO: fires per step against the read log's `popped`.
    FifoPop,
    /// The cross-rank `block` credit edge. **Never re-derived** — it appears
    /// only on [`StandDown`]s and [`ConservationFinding`]s, so a reader of a
    /// stand-down list can tell "we declined to judge this node's schedule
    /// because of a block edge" from "we could not even run the conservation
    /// assert".
    BlockCredit,
    /// The RECORDING's own read log, judged before any rule is.
    ///
    /// Also never re-derived, and for the same reason [`Self::BlockCredit`]
    /// is not: it exists so a stand-down can say "no rule ran, because the
    /// record stream this node's rules would read is not one the recorder can
    /// have written". Naming a real decider there would claim that THAT rule
    /// looked and declined, when the truth is that the evidence was refused
    /// upstream of all of them — including for a node
    /// ([`TriggerDecl::Unmodelled`], a `Period` node) that has no read-driven
    /// rule to decline.
    ReadLog,
}

/// What a RESUMED pass RESTORED into its scheduler before the first step this
/// module judges: the `Sync` heads `Scheduler::restore_sync_input_timestamps`
/// minted from the anchor's framework section, per node, per input.
///
/// The branch this guards is reachable: `resolve_resume` admits a
/// `coordination: free_run` bag with exactly ONE worker rank into the ordinary
/// resume, and that pass is judged HERE (the verifier runs on every free-run
/// pass). A restored head is UNBACKED (a real stamp with no kind-6 record
/// behind it, since the read that filled it happened before the recording's
/// first retained step), so a Sync rule that folded only the records it can
/// see would judge a map MISSING those members and convict a healthy resume on
/// its first step (arm B: no transversal on an input whose head is real). The
/// heads are therefore SEEDED as the matcher's own: the scheduler holds them
/// (`HeadState::Filled { backed: false }` is still a filled head to
/// `align()`), so the verifier starts from them.
///
/// A from-start pass restores nothing ([`Self::none`]) and every rule runs
/// exactly as it does without a resume; the from-start oracles are pinned on
/// that.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(crate) struct PassRestore {
    /// `node id -> (input -> wire stamp)`: what the anchor's framework section
    /// stated as each Sync node's `sync_input_timestamps` at the anchor step.
    sync_heads: BTreeMap<String, BTreeMap<String, u64>>,
}

impl PassRestore {
    /// A pass that restored nothing: every from-start pass, of either mode.
    pub(crate) const fn none() -> Self {
        Self {
            sync_heads: BTreeMap::new(),
        }
    }

    /// The heads a resume plan restored (`ResumePlan::sync_input_timestamps`,
    /// exactly what `Scheduler::restore_sync_input_timestamps` was handed).
    pub(crate) fn from_sync_heads(sync_heads: BTreeMap<String, BTreeMap<String, u64>>) -> Self {
        Self { sync_heads }
    }

    /// The restored heads of ONE node, if the anchor stated any.
    fn sync_heads_for(&self, node_id: &str) -> Option<&BTreeMap<String, u64>> {
        self.sync_heads.get(node_id)
    }
}

/// May THIS BAG's kind-6 role bits be read as roles?
///
/// A named two-state rather than a `bool`, because every consumer in this
/// module takes it and `verify(.., true)` at a call site says nothing about
/// WHICH truth is being asserted — a caller that swapped it for the sibling
/// `block_gated` flag next door would compile and would silently read an
/// archived bag's zeroed bits as declarations. The bool survives at the
/// BOUNDARY (`replay_engine::PassInputs::roles_stamped`, which also
/// drives the injection steering) and is converted ONCE, where the bag's
/// `trace_format` is already the subject.
///
/// [`Self::Declined`] is not "the bits are absent" — it is "this reader may
/// not believe them", which is also the answer for a bag whose bits are right
/// there under a `trace_format` that predates them (the adversarial half of the
/// threshold).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RoleTrust {
    /// `trace_format >= replay_engine::ROLE_STAMPED_MIN_TRACE_FORMAT`: a
    /// record whose role NAMES a site (`Drain` / `Body`) may be steered on.
    Believed,
    /// Every archived bag, and any bag whose stamp forbids reading the bits.
    /// Every rule below takes its pre-roles arm, byte for byte.
    Declined,
}

impl RoleTrust {
    /// Whether roles may be steered on.
    pub fn believed(self) -> bool {
        matches!(self, Self::Believed)
    }
}

impl RoleTrust {
    /// The ONE conversion, from the boundary's `roles_stamped` flag.
    ///
    /// A NAMED constructor rather than `impl From<bool>`, because every
    /// flag threaded beside this one is also a `bool` —
    /// `block_gated`, `suppressible`, `is_last_pass` — and `RoleTrust::from(x)`
    /// reads as a conversion whatever `x` means, so the wrong one compiles and
    /// silently declares an archived bag's zeroed bits trustworthy. The name
    /// says which truth is being asserted.
    pub(crate) fn from_stamped(roles_stamped: bool) -> Self {
        if roles_stamped {
            Self::Believed
        } else {
            Self::Declined
        }
    }
}

impl Decider {
    /// The stable label a finding renders under.
    pub fn label(self) -> &'static str {
        match self {
            Self::Period => "period_ms schedule",
            Self::Throttle => "throttle_ms gate",
            Self::Sync => "sync alignment",
            Self::FifoPop => "FIFO pop count",
            Self::BlockCredit => "block credit",
            Self::ReadLog => "the recorded read log",
        }
    }
}

impl fmt::Display for Decider {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.label())
    }
}

// ---------------------------------------------------------------------------
// Inputs — what the replay executor reads out of the bag
// ---------------------------------------------------------------------------

/// A node's DECLARED trigger, as the replayed graph declares it.
///
/// This is the candidate's side of the comparison: the recording says what
/// fired, the declaration says what should have.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TriggerDecl {
    /// `#[cerulion_node(period_ms = N)]`.
    Period {
        /// The declared interval in nanoseconds.
        interval_ns: u64,
        /// The declared `max_catchup`, verbatim (`None` = undeclared).
        max_catchup: Option<u32>,
        /// The catch-up clamp override in force for this run, if any
        /// — passed straight to
        /// [`effective_max_catchup`], never re-implemented.
        catchup_cap_override: Option<u32>,
    },
    /// `sync_window_ms = N` (`window_ns = Some`) or `unbounded_sync`
    /// (`window_ns = None`) over the node's `#[input(trigger)]` ports.
    Sync {
        /// The declared window, or `None` for `unbounded_sync`.
        window_ns: Option<u64>,
        /// The `#[input(trigger)]`-marked inputs, in declaration order.
        trigger_inputs: Vec<String>,
    },
    /// A data-trigger node.
    Data {
        /// The `#[input(trigger)]`-marked inputs, in declaration order.
        trigger_inputs: Vec<String>,
    },
    /// Anything this module does not model (External, host-driven). No
    /// schedule re-derivation is attempted and no stand-down is emitted —
    /// there is no rule to decline.
    Unmodelled,
}

/// One node's declaration.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NodeDecl {
    /// The graph node id, matching the recorded fires' `node_id`.
    pub node_id: String,
    /// Its trigger policy.
    pub trigger: TriggerDecl,
    /// `throttle_ms` in nanoseconds, if declared. Orthogonal to `trigger` —
    /// it stacks with every policy except `period_ms` (rejected at compile
    /// time), so a `Period` node carrying one is a declaration this module
    /// judges on the Period rule alone.
    pub throttle_ns: Option<u64>,
    /// Does this node wire any input that is NOT
    /// `#[input(trigger)]`-marked?
    ///
    /// It decides whether a Sync fire may be assumed to CONSUME its trigger
    /// heads. The macro's generated tick nests one `try_view` per input in
    /// DECLARATION order and an earlier input yielding nothing collapses the
    /// chain (the pre-first-delivery WAIT —
    /// `transport::subscriber`'s `drain_for_trigger` states it verbatim). A
    /// fire proves every TRIGGER head is `Filled`, so with no non-trigger input
    /// the chain cannot collapse and every trigger head is read; with one, a
    /// head can survive the fire in the subscriber's frozen slot and the NEXT
    /// boundary re-offers it — silently, at the same stamp. So this flag is
    /// exactly "may a spent head come back".
    ///
    /// It WIDENS the supply/candidate carry (`verify_sync`'s "carry forward"
    /// block: a fire on such a node is credited with consuming no head) and
    /// nothing else. A widening can only make the verifier more permissive, which is
    /// why it is not a [`StandDownReason`] — the rule DOES run — but it is
    /// still operator-visible, as a [`CarriedHeadsNote`] in the report.
    pub has_non_trigger_inputs: bool,
    /// Per wired TRIGGER input, the input's EFFECTIVE queue capacity — its
    /// declared depth times the number of publisher connections on the topic
    /// it reads (iceoryx2 keeps ONE queue per publisher connection, each of
    /// the subscriber's declared depth, so a two-writer bus read at depth 10
    /// holds 20 heads) — with the frozen head and the parked `next_head`
    /// added, the physical ceiling on heads the scheduler can hold for that
    /// input at once. It BOUNDS the cross-step carry on every node: a fire on
    /// a [`Self::has_non_trigger_inputs`] node is credited with consuming
    /// nothing, and the ordinary carry grows by pops minus fires on a
    /// fast-input windowed pair, so without a ceiling the carry grows with the
    /// run and both wide-map arms go inert (arm A's supply pins at the burst
    /// cap, arm B is skipped once the candidates cap). A ceiling BELOW the
    /// real capacity is the opposite defect — it drops heads the matcher still
    /// holds and convicts a healthy replay — which is why the multiplier is
    /// part of the value. An input the map does
    /// not name takes `DEFAULT_CONSUMER_DEPTH` (one connection).
    pub trigger_input_capacities: BTreeMap<String, u32>,
}

/// One recorded FIRE (`RECORD_TYPE_FIRE`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecordedFire {
    /// The firing node's id, resolved through the rank manifest.
    pub node_id: String,
    /// The record's `fire_time_ns` — the gating-clock instant of THIS fire.
    /// For a Period catch-up burst that is the interval deadline, not the
    /// step clock.
    pub fire_time_ns: u64,
}

/// One recorded kind-6 READ OUTCOME, resolved through the rank manifest.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecordedRead {
    /// The reading node's id.
    pub node_id: String,
    /// The input port name.
    pub input: String,
    /// The wire outcome kind — [`ReadOutcomeKind`] itself, never a second
    /// vocabulary (the no-second-copy rule).
    pub kind: ReadOutcomeKind,
    /// The served wire SEQUENCE (`None` = the no-frame sentinel).
    ///
    /// `u32`, matching the wire: `trace_ring`'s own doc says "a real wire
    /// sequence is a `u32`, so the two ranges can never collide" with the
    /// `u64::MAX` no-frame sentinel. A `u64` here against the `u32` in
    /// [`crate::replay_inject`] would make the caller narrow twice — silently
    /// TRUNCATING on one side and, on this one, dropping the frame join so a
    /// malformed record surfaces as
    /// [`StandDownReason::MissingWireStamp`] ("a served frame's wire stamp
    /// could not be joined"), which names the wrong cause. The narrowing
    /// happens ONCE, at the demux, and a value the wire cannot have produced is
    /// a corrupt bag rather than a silent degrade.
    pub served_seq: Option<u32>,
    /// Frames this drain consumed. On a [`ReadOutcomeKind::Truncated`] marker
    /// this is REINTERPRETED as the number of records dropped in that merge
    /// window, exactly as the wire defines it.
    pub popped: u32,
    /// The served frame's WIRE timestamp — the PRODUCER's gating clock at loan
    /// time, joined out of the bag's frame stream by `served_seq`.
    ///
    /// It is not in the kind-6 record itself; the caller performs the join,
    /// because only the caller holds the frames. `None` means "no frame served"
    /// or "the join failed", and Sync stands down loudly on the second.
    pub wire_stamp_ns: Option<u64>,
    /// The record's READ-SITE ROLE, verbatim off the wire.
    ///
    /// [`ReadSiteRole::Unstamped`] on every `trace_format` <= 4 bag (the bits
    /// were never written), which is why every consumer here ALSO takes a
    /// `RoleTrust`: whether a role may be BELIEVED is a property of the BAG,
    /// and a reader that keyed on the bits alone would behave identically
    /// today and would silently start believing whatever a future writer put
    /// there.
    pub role: ReadSiteRole,
}

/// One recorded scheduler step on ONE rank.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecordedStep {
    /// The logical step number.
    pub step: u64,
    /// The recorded `RECORD_TYPE_STEP_BOUNDARY` gating-clock target — the
    /// `current_time_ns` every decider on this rank saw at this step.
    pub clock_ns: u64,
    /// Every FIRE record in this step, in recorded order.
    pub fires: Vec<RecordedFire>,
    /// Every kind-6 record in this step.
    pub reads: Vec<RecordedRead>,
}

/// One cross-rank `block` edge, with the publish count the bag's frame stream
/// carries for it.
///
/// `published_frames` comes from the FRAMES, not from the producer's fires: a
/// fire and a publish are not the same event (an output the tick never writes
/// never publishes), so counting fires would make a legitimately
/// silent tick read as a conservation under-run.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BlockEdgeObservation {
    /// The edge's topic.
    pub topic: String,
    /// The producing node — its schedule re-derivation stands down.
    pub producer_node: String,
    /// The consuming node.
    pub consumer_node: String,
    /// The consuming `#[input(backpressure = block)]` port.
    pub consumer_input: String,
    /// The declared `#[input(depth = N)]` ceiling.
    pub depth: u32,
    /// Frames the bag carries on this edge's topic from the producing rank.
    pub published_frames: u64,
}

// ---------------------------------------------------------------------------
// Outputs
// ---------------------------------------------------------------------------

/// What a decider says about ONE (node, step): how many fires, and — when the
/// decider fixes them — at which gating-clock instants.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct StepFires {
    count: u32,
    instants: Vec<u64>,
}

impl StepFires {
    /// No fire at all.
    pub fn none() -> Self {
        Self::default()
    }

    /// A count with no instants — the shape every decider but Period produces.
    pub fn count(count: u32) -> Self {
        Self {
            count,
            instants: Vec::new(),
        }
    }

    /// A fully-determined burst.
    pub fn at(instants: Vec<u64>) -> Self {
        Self {
            count: instants.len() as u32,
            instants,
        }
    }

    /// How many times the node fires at this step.
    ///
    /// The pair is PRIVATE. `instants` is either empty or exactly
    /// `count` long — every constructor above upholds that, and a struct
    /// literal was free to break it, which would have rendered "2 fire(s) at
    /// [t0, t1, t2]" into an operator's divergence finding.
    #[must_use]
    pub fn fire_count(&self) -> u32 {
        self.count
    }

    /// Each fire's gating-clock instant, in order. EMPTY when the decider
    /// constrains only the count (Sync, throttle, FIFO) or when the burst
    /// exceeded [`PERIOD_BURST_RENDER_CAP`].
    #[must_use]
    pub fn instants(&self) -> &[u64] {
        &self.instants
    }
}

impl fmt::Display for StepFires {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.instants.is_empty() {
            write!(f, "{} fire(s)", self.count)
        } else {
            write!(f, "{} fire(s) at {:?}", self.count, self.instants)
        }
    }
}

/// A rank-local decider's re-derivation disagreed with the recorded trace.
///
/// This is a [`DivergenceClass::FireSchedule`] finding and nothing else.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScheduleFinding {
    /// The node whose schedule diverged.
    pub node_id: String,
    /// The step at which the disagreement first appears.
    pub step: u64,
    /// Which rule produced it.
    pub decider: Decider,
    /// What the re-derivation says should have happened.
    pub expected: StepFires,
    /// What the trace records.
    pub recorded: StepFires,
}

impl ScheduleFinding {
    /// The operator-facing class — always [`DivergenceClass::FireSchedule`].
    /// A method rather than a field so no caller can construct a finding
    /// carrying a different class.
    pub fn divergence_class(&self) -> DivergenceClass {
        DivergenceClass::FireSchedule
    }
}

impl fmt::Display for ScheduleFinding {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}: node '{}' step {} {} — re-derived {}, recorded {}",
            self.divergence_class().phrase(),
            self.node_id,
            self.step,
            self.decider,
            self.expected,
            self.recorded,
        )
    }
}

/// How a block edge's credit conservation broke.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConservationBreach {
    /// More frames were drained than were ever published — the consumer popped
    /// something nobody produced.
    UnderRun,
    /// The final outstanding count exceeds the declared depth, so NO legal
    /// interleave of the recorded publishes and drains keeps the edge inside
    /// its ceiling.
    OverRun {
        /// `published - drained`: the outstanding count the edge cannot avoid
        /// reaching.
        outstanding: u64,
    },
}

/// A block edge's credit conservation does not hold under ANY legal interleave.
///
/// LOUD and typed, and deliberately **non-gating** — see
/// [`RederivationReport::is_clean`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConservationFinding {
    /// The edge's topic.
    pub topic: String,
    /// The consuming node.
    pub consumer_node: String,
    /// The consuming input.
    pub consumer_input: String,
    /// The declared depth.
    pub depth: u32,
    /// Frames published on the edge.
    pub published: u64,
    /// Frames drained by the consumer, from the read log.
    pub drained: u64,
    /// Which bound broke.
    pub breach: ConservationBreach,
}

impl fmt::Display for ConservationFinding {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let what = match self.breach {
            ConservationBreach::UnderRun => {
                format!("drained {} > published {}", self.drained, self.published)
            }
            ConservationBreach::OverRun { outstanding } => format!(
                "outstanding {} exceeds depth {} (published {}, drained {})",
                outstanding, self.depth, self.published, self.drained
            ),
        };
        write!(
            f,
            "block credit conservation broken on '{}' -> {}.{}: {} \
             (report-only — never a replay verdict)",
            self.topic, self.consumer_node, self.consumer_input, what,
        )
    }
}

/// Why a decider declined to judge.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StandDownReason {
    /// The node produces onto a cross-rank `block` edge, so its fires are
    /// gated by occupancy the log does not record.
    BlockGated,
    /// A Period node the recording never fired — there is no phase evidence to
    /// anchor the schedule on.
    NoPhaseAnchor,
    /// A Period node declaring a zero interval: the whole-interval advance is
    /// undefined and the live loop would not terminate on it either.
    ZeroInterval,
    /// A [`ReadOutcomeKind::Truncated`] marker sits on an input this decider
    /// reads, so its record stream has a deterministic hole.
    TruncatedReadLog,
    /// A [`ReadOutcomeKind::Decimated`] read sits on a trigger input: frames
    /// were popped and none delivered, so pops do not count fires.
    DecimatedRead,
    /// A trigger input served a frame whose WIRE stamp the caller could not
    /// join, so the alignment spread cannot be computed.
    MissingWireStamp,
    /// The node is gated by something that can only SUPPRESS a fire (a
    /// `throttle_ms` cap, or a block edge), so "aligned but did not fire"
    /// is not evidence of divergence. The opposite direction — "fired while
    /// misaligned" — is still judged.
    SuppressibleFire,
    /// A trigger input's physical queue (its effective
    /// capacity — declared depth × publisher connections — plus the frozen
    /// head and the parked `next_head`) exceeds [`SYNC_CARRY_ROOM`], the most
    /// heads the verifier can carry across a step without its feasibility
    /// arm going inert. The node is NOT judged — decided ONCE, before its
    /// first step (decided at the carry cut, a step that convicted
    /// before reaching the cut still judged it): cutting the carry below the
    /// queue would drop heads the matcher holds and convict a healthy replay;
    /// carrying them all would fill the candidate vector and accept every
    /// later fire unchecked. A many-writer bus read at a deep queue is the
    /// shape; a single-connection input can never reach it (const-asserted).
    QueueExceedsVerifierRoom,
    /// A trigger input's record stream carries reads whose SITE was INFERRED
    /// and whose inference has collapsed, so which stamps the scheduler's
    /// alignment map actually saw is not recoverable.
    ///
    /// **Raised only where the SITE was INFERRED rather than read.** The
    /// usual case is a `trace_format` <= 4 bag: the wire has no role
    /// bit, so this module infers the SITE from the KIND — and `DrainedBatch`
    /// is emitted by the sync drain (`try_receive_timestamps` →
    /// `drain_samples`) AND by a node body that calls
    /// `AnySubscriber::try_receive` on the same input. Those are two
    /// SUBSCRIBERS with two cursors, so their stamps differ, and only the
    /// drain's reached `sync_input_timestamps`. More than one KIND-inferred
    /// drain-site `DrainedBatch` for one `(node, input)` in one step is the
    /// collision, and nothing on such a bag can take it apart.
    ///
    /// It is **also reachable on a format-5 bag**, and that is not a leftover:
    /// [`ReadSiteRole::from_wire`] decodes an UNWRITTEN `0` to
    /// [`ReadSiteRole::Unstamped`], and such a record takes the pre-roles KIND
    /// arm at every consumer — so a stamped bag can carry drain-site reads
    /// whose site nothing on the wire actually named, and their collisions are
    /// exactly as unrecoverable as an archived bag's. (The value `3` is
    /// [`ReadSiteRole::Peek`], so a `0` is the only unnamed value.)
    ///
    /// **What it is NOT:** a collision among
    /// records whose role is EXPLICITLY [`ReadSiteRole::Drain`]. Those are not
    /// ambiguous at all — the wire named their site — and they are also not
    /// impossible: per-set Sync (the DEFAULT) mints several
    /// drain-role `DrainedBatch` records for one `(node, input)` in one step as
    /// its ORDINARY behaviour. See [`Self::ImpossibleReadShape`].
    AmbiguousReadSite,
    /// A ROLE-STAMPED bag carries a `(kind, role)` pair NO MINT
    /// SITE can produce — a `Drain` role on a BODY-ONLY outcome kind
    /// (`Served`, `Held`, `NoFrame`).
    ///
    /// # The shape this does NOT name, and why
    ///
    /// A tempting definition names a different shape: "two or
    /// more DRAIN-role `DrainedBatch` records for one `(node, input)` in one
    /// step", on the premise that *the drain runs exactly once per step per
    /// trigger input*. That premise does not hold under PER-SET Sync, which is
    /// the DEFAULT delivery discipline: ONE healthy `align()` mints
    /// a `FillBoundary` drain (`drain_for_trigger`), PLUS a
    /// `sync_peek_next_stamp` pop, PLUS up to `DATA_PENDING_CARRY_CLAMP`
    /// `sync_discard_head` pops — every one of them a `DrainedBatch` stamped
    /// [`ReadSiteRole::Drain`] on the same `(node, input)` in the same step,
    /// because the per-set matcher IS the scheduler reading on the node's
    /// behalf. So a detector built on that premise brands a HEALTHY per-set Sync
    /// recording "corrupt — re-record the bag", routinely, on exactly the graphs
    /// the feature exists for.
    ///
    /// # What is left, and why it is genuinely unproducible
    ///
    /// The mint-site table in `transport::subscriber` is the whole evidence:
    /// `Served` / `Held` / `NoFrame` are staged ONLY by the node body's own
    /// read paths (`try_view`, `snapshot_latest`), each passing
    /// [`ReadSiteRole::Body`] as a compile-time constant; the drain sites
    /// (`try_receive_for_drain`, `drain_for_trigger`, the two per-set Sync
    /// matcher ops) stage `DrainedBatch` and `Decimated` and nothing else
    /// (`drain_samples` itself stages only `DrainedBatch`, under either role;
    /// a `Body`-role `Decimated` comes from `try_view` / `snapshot_latest`). A
    /// record pairing `Drain` with a body-only kind therefore names a site that
    /// cannot have written it — with the bits BELIEVED, so it is not an
    /// inference that could be wrong. That is CORRUPTION, and it is reported as
    /// such, LOUDLY and PER NODE.
    ///
    /// `Truncated` and `Producer` are deliberately NOT judged: the overflow
    /// marker carries the STAGE's role (a `Body`-role stage genuinely holds
    /// drain-SITE records under the unified discipline) and a `Producer`
    /// annotation carries its paired READ's role, so both roles are legitimate
    /// on both. `Decimated` is minted at both sites.
    ///
    /// Only records whose role is EXPLICITLY [`ReadSiteRole::Drain`] on a
    /// BELIEVED bag can raise this: a corruption claim must rest on bits that
    /// were believed, so an unwritten `0` or the reserved `3` reaches
    /// [`Self::AmbiguousReadSite`]'s kind arm instead.
    ///
    /// REPORT-ONLY, like every other stand-down: the read log has been
    /// report-only since it existed, and making the first verdict-bearing
    /// read-log condition a by-product of a provenance change would be a scope
    /// smuggle. Whether the read log should ever cross into the verdict is a
    /// separate decision that is not yet made.
    ImpossibleReadShape,
    /// A `block` edge's returned credit cannot be attributed to
    /// the SUBSCRIBER PORT the runtime's credit mirror is wired to, so no
    /// correct `drained` count exists for it.
    ///
    /// # What the mirror actually counts
    ///
    /// `BlockDeferEdge::outstanding` (`graph/runtime.rs:958-1010`) is
    /// incremented once per publish and decremented by
    /// `CerulionSubscriber::record_block_drained`
    /// (`transport/subscriber.rs:1879`), which runs on the ONE subscriber the
    /// consumer's `register_block_probe` was installed on
    /// (`graph/runtime.rs:5168`) — the node's own BODY subscriber.
    ///
    /// # The two shapes that reach here, and the one that does not
    ///
    /// This is a NARROW decline, because the graph settles the common case
    /// (see `CreditBasis`). What is left:
    ///
    /// 1. **A pop under a role the wire did not NAME** — an unwritten `0` on a
    ///    bag whose roles are otherwise believed. Nothing
    ///    can place such a pop on a port: not the wire, which declined to say,
    ///    and not the graph, which knows the discipline but not which site this
    ///    record came from. A record that popped NOTHING is exempt — it enters
    ///    no sum, so it cannot skew one.
    /// 2. **A SYNC trigger input whose stream carries pops under BOTH roles.**
    ///    Per-set Sync unifies onto the probed subscriber
    ///    (`graph/runtime.rs:5683`) while legacy Sync keeps a dedicated
    ///    UNPROBED one (`:5791`), and `per_set_capable` (`:5587`) depends on
    ///    symbol presence and an env knob, neither of which is in the bag —
    ///    see `credit_basis` for why a `trace_format` 5 bag really can carry
    ///    either. Summing double-counts under legacy (a false UNDER-run, since
    ///    both ports see the same frames); body-only drops the boundary drain
    ///    under per-set (a false OVER-run). One role alone is the same number
    ///    either way and is counted.
    ///
    /// What does NOT reach here is the shape that matters most: a `block`
    /// DATA-trigger edge always shows Drain-role drain-sub records beside
    /// Body-role probed-sub pops, and declining that would decline every
    /// cross-rank block data-trigger edge there is. `graph/node.rs:777-788`
    /// settles it.
    ///
    /// Raised ONLY on a bag whose roles are BELIEVED. On an archived bag the
    /// pre-roles kind rule is kept verbatim and this is unreachable — that arm
    /// has its own imprecision, recorded at `verify_conservation`.
    UnattributableCredit,
    /// A trigger input's HEAD stamps move BACKWARD, so the
    /// alignment map cannot be carried across the discontinuity.
    ///
    /// Within one publisher epoch a head-naming record's stamp is
    /// non-decreasing: the fill, the refill and the promotion each name a frame
    /// at or after the one they replace. A regression is therefore a wire-stamp
    /// EPOCH RESET (a rebooted robot, a restarted worker, a re-attached bridge
    /// — `Scheduler::apply_sync_epoch_reset`), and a reset VOIDS every other
    /// input's held head with NO record at all (`sync_void_head` stages
    /// nothing). Nothing on the wire says which heads went, so the map this
    /// rule carries is no longer the scheduler's and the correct answer is to
    /// stop judging this node.
    ///
    /// It is also the guard that makes the "aligned but did not fire" direction
    /// sound at all: a `DiscardTie` whose refill found an empty, non-decimated
    /// queue leaves a spent LOW stamp in the map, and only per-input
    /// monotonicity keeps the resulting spread above the window.
    SyncHeadStampRegressed,
}

impl StandDownReason {
    /// The stable label a stand-down renders under.
    pub fn label(&self) -> &'static str {
        match self {
            Self::BlockGated => "a cross-rank block edge gates its fires",
            Self::NoPhaseAnchor => "the recording carries no fire to anchor its phase on",
            Self::ZeroInterval => "the declared interval is zero",
            Self::TruncatedReadLog => "the read log is truncated on an input it reads",
            Self::DecimatedRead => "a trigger input read was decimated",
            Self::MissingWireStamp => "a served frame's wire stamp could not be joined",
            Self::SuppressibleFire => "a suppressing gate can explain a missing fire",
            Self::QueueExceedsVerifierRoom => {
                "a trigger input's queue holds more heads than the verifier can carry across \
                 a step"
            }
            Self::AmbiguousReadSite => {
                "a trigger input's reads cannot be attributed to one read site"
            }
            Self::ImpossibleReadShape => {
                "the recording carries a read shape no recorder can produce \
                 (corrupt recording — re-record the bag)"
            }
            Self::UnattributableCredit => {
                "the edge's reads cannot be attributed to the subscriber the \
                 block credit mirror is wired to"
            }
            Self::SyncHeadStampRegressed => {
                "a trigger input's head stamps move backward across the run"
            }
        }
    }

    /// The STABLE machine token an operator or a CI job keys on.
    ///
    /// A sibling of [`crate::replay_inject::StandDownReason::code`], and for
    /// the same reason: the LABEL is prose that may be reworded, while a
    /// grep-able token must not move. It rides
    /// [`crate::replay_engine::RederivationNote::code`]: a per-node
    /// stand-down does have a token "that belongs to a closed set".
    /// This enum is exactly a closed set, and a CI
    /// job asking "did any bag report `impossible_read_shape` this week?"
    /// would otherwise have to substring-match a sentence.
    pub fn code(&self) -> &'static str {
        match self {
            Self::BlockGated => "block_gated",
            Self::NoPhaseAnchor => "no_phase_anchor",
            Self::ZeroInterval => "zero_interval",
            Self::TruncatedReadLog => "truncated_read_log",
            Self::DecimatedRead => "decimated_read",
            Self::MissingWireStamp => "missing_wire_stamp",
            Self::SuppressibleFire => "suppressible_fire",
            Self::QueueExceedsVerifierRoom => "queue_exceeds_verifier_room",
            Self::AmbiguousReadSite => "ambiguous_read_site",
            Self::ImpossibleReadShape => "impossible_read_shape",
            Self::UnattributableCredit => "unattributable_credit",
            Self::SyncHeadStampRegressed => "sync_head_stamp_regressed",
        }
    }
}

impl fmt::Display for StandDownReason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.label())
    }
}

/// A decider declined to judge, and says so.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StandDown {
    /// The node (for [`Decider::BlockCredit`], the CONSUMING node of the edge).
    pub node_id: String,
    /// Which rule declined.
    pub decider: Decider,
    /// Why.
    pub reason: StandDownReason,
}

impl fmt::Display for StandDown {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "not verified: node '{}' {} — {}",
            self.node_id, self.decider, self.reason
        )
    }
}

/// How one input's `DrainedBatch` records split
/// between the two read SITES, on a bag whose roles are believed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InputReadSites {
    /// The input port.
    pub input: String,
    /// `DrainedBatch` records the wire attributes to the DRAIN site — the ones
    /// every rule in this module counts.
    pub drain_records: u64,
    /// `DrainedBatch` records the wire attributes to the node BODY — the ones
    /// role steering EXCLUDES, and which the pre-roles kind rule would have
    /// counted.
    pub body_records: u64,
    /// Frames the DRAIN-site records above popped.
    ///
    /// The RECORD counts
    /// alone were the wrong scale for the thing the census is evidence about.
    /// `verify_fifo` compares FIRES against a sum of `popped`, so three drain
    /// batches of one frame beside one body batch of a hundred is a stream
    /// whose records read 3-vs-1 "drain-heavy" while the exclusion removed 100
    /// of its 103 pops — the census stayed silent on exactly the shape it
    /// exists to explain. Both scales are carried, and either being body-heavy
    /// emits the note.
    pub drain_popped: u64,
    /// Frames the BODY-site records above popped — the ones role steering
    /// excluded from the pop sums.
    pub body_popped: u64,
}

/// The evidence a finding derived under ROLE
/// STEERING carries about the reads it was derived from.
///
/// Role steering makes the site EXACT, and the price of exactness is that a
/// body-role `DrainedBatch` — an accumulate-all `try_receive` in a node's own
/// tick — does not move the sync stamp map, does not count as a FIFO pop
/// and does not return block credit. That is correct, and it is also a
/// way for a finding to be RIGHT ABOUT THE NUMBERS AND WRONG ABOUT THE CAUSE:
/// on a stream that is mostly body reads, "expected 0 pops, recorded 3 fires"
/// may be describing the EXCLUSION rather than a schedule that diverged.
///
/// So a finding on such a stream carries its own census. It is evidence, never
/// a verdict — the finding stands exactly as it was, and this says what the
/// reader needs in order to interpret it. Emitted ONLY under
/// `RoleTrust::Believed` (there is no split to report otherwise) and ONLY
/// when some relevant input's body records OUTNUMBER its drain records.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReadSiteCensus {
    /// The node the finding names.
    pub node_id: String,
    /// Which rule produced the finding this census belongs to.
    pub decider: Decider,
    /// The per-input split, in the order the inputs were declared.
    pub inputs: Vec<InputReadSites>,
}

impl fmt::Display for ReadSiteCensus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "node '{}' {}: this bag's roles are believed, so only DRAIN-site reads \
             fed the finding above — the drained-batch records on its inputs split \
             drain/body as ",
            self.node_id, self.decider
        )?;
        for (i, e) in self.inputs.iter().enumerate() {
            if i > 0 {
                f.write_str(", ")?;
            }
            write!(
                f,
                "{}=records {}/{}, pops {}/{}",
                e.input, e.drain_records, e.body_records, e.drain_popped, e.body_popped
            )?;
        }
        f.write_str(
            " (a body-heavy stream means the finding may describe the role EXCLUSION \
             rather than a diverged schedule — report-only, the finding stands)",
        )
    }
}

/// This Sync node was judged with CARRIED HEADS, because it wires
/// an input that is not `#[input(trigger)]`-marked.
///
/// The weakening, stated plainly: a Sync fire normally consumes one head per
/// trigger input, so the next step starts from the arrivals it recorded. A node
/// with a non-trigger input can collapse its tick's read chain (the
/// pre-first-delivery WAIT), leaving a fired head unread in the subscriber's
/// frozen slot — and the NEXT boundary silently re-offers it, at the same stamp,
/// with no record at either end. So this node's supply and candidate sets keep
/// the previous step's last stamp alive whether or not the fire consumed it.
///
/// It is NOT a [`StandDown`], and the distinction is exact: a stand-down claims
/// "no rule ran", while here every rule ran — just more permissively, in the
/// direction that can only DROP a finding, never invent one. What it costs is
/// the arm-B finding on a genuinely misaligned step whose only feasible tuple
/// uses a carried head, so an operator reading a CLEAN Sync verdict on such a
/// node should know it was judged on the wider evidence.
///
/// Report-only and never a gate — [`RederivationReport::is_clean`] ignores it,
/// exactly as it ignores [`RederivationReport::conservation`]. Emitted at most
/// once per node per report (`verify_sync` runs once per node).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CarriedHeadsNote {
    /// The Sync node judged with carried heads.
    pub node_id: String,
}

impl fmt::Display for CarriedHeadsNote {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "node '{}' sync alignment: judged with carried heads — this node \
             wires non-trigger inputs, so a fired head can be silently \
             re-offered at the next boundary and the alignment evidence is \
             widened to admit it (report-only; it can only DROP a finding, \
             never invent one)",
            self.node_id
        )
    }
}

/// Everything one rank's re-derivation concluded.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct RederivationReport {
    /// Schedule divergences — the GATING half.
    pub findings: Vec<ScheduleFinding>,
    /// Credit-conservation breaches — loud, and NEVER a gate.
    pub conservation: Vec<ConservationFinding>,
    /// Everything this run declined to judge, with its reason.
    pub stand_downs: Vec<StandDown>,
    /// Per-finding read-site evidence. Never a
    /// gate — [`Self::is_clean`] and [`Self::divergence_class`] both ignore
    /// it, exactly as they ignore [`Self::conservation`].
    pub read_site_census: Vec<ReadSiteCensus>,
    /// The Sync nodes judged on WIDENED evidence because they wire
    /// a non-trigger input. Never a gate — see [`CarriedHeadsNote`].
    pub carried_heads: Vec<CarriedHeadsNote>,
}

impl RederivationReport {
    /// Whether this rank's SCHEDULE verified clean.
    ///
    /// Conservation findings are deliberately excluded: they are evidence
    /// about the recording's credit accounting, not a claim that the
    /// candidate's fire schedule diverged, and the design states them as
    /// "loud on violation, never a gate". Making this read
    /// `findings.is_empty() && conservation.is_empty()` turns a report-only
    /// assert into a replay verdict, which is what
    /// `a_conservation_breach_is_loud_but_never_gates_the_verdict` pins.
    pub fn is_clean(&self) -> bool {
        self.findings.is_empty()
    }

    /// The class this report contributes to the replay verdict, or `None`.
    ///
    /// One class only: this module mints no vocabulary of its own.
    pub fn divergence_class(&self) -> Option<DivergenceClass> {
        (!self.findings.is_empty()).then_some(DivergenceClass::FireSchedule)
    }
}

// ---------------------------------------------------------------------------
// The mirrored predicates — each names the core location it restates
// ---------------------------------------------------------------------------

/// One step of the Period schedule.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PeriodStep {
    /// How many times the node fires at this step.
    pub fire_count: u32,
    /// The earliest un-fired interval — the first fire's gating instant.
    /// Meaningless when `fire_count == 0`.
    ///
    /// Deliberately NOT "0 when `fire_count == 0`": the shared
    /// `catchup_clamp::period_advance` reports 0 only when nothing was DUE, and
    /// a caller passing `cap = 0` gets a real deadline back beside a zero count
    /// (that is the "carry the deadline past `now`, mint no fires" reading two
    /// scheduler call sites use). A reader must key on `fire_count`, never on
    /// this field being zero.
    pub first_fire_ns: u64,
    /// The node's deadline AFTER this step.
    pub next_fire_ns: u64,
}

/// The Period advance, from the ONE place it is implemented:
/// [`cerulion_core::scheduler::catchup_clamp::period_advance`], which is what
/// the LIVE `Scheduler::decide_node` `Period` arm calls too.
///
/// A MIRROR here — the live arm advancing with a `while` loop while this
/// re-derived the same schedule in O(1) — would be the two-copies
/// class on the one rule whose entire job is to agree with itself. The core
/// owns the O(1) form and this delegates; the hand oracles below and
/// `the_period_mirror_and_the_engine_answer_one_hand_oracle` pin the SHARED
/// function, and `catchup_clamp`'s own arms pin it against a transliteration of
/// the `while` loop.
///
/// [`PeriodStep`] is kept as this module's own shape (its sibling predicates
/// return module-local types and its callers read it) and is a field-for-field
/// projection of the core's `PeriodAdvance`.
pub fn period_step(next_fire_ns: u64, now_ns: u64, interval_ns: u64, cap: u32) -> PeriodStep {
    let adv = cerulion_core::scheduler::catchup_clamp::period_advance(
        next_fire_ns,
        now_ns,
        interval_ns,
        cap,
    );
    PeriodStep {
        fire_count: adv.fire_count,
        first_fire_ns: adv.first_fire_ns,
        next_fire_ns: adv.next_fire_ns,
    }
}

/// The `throttle_ms` DEFER predicate, from the ONE place it is implemented:
/// [`cerulion_core::graph::runtime::throttle_defers`], which the live pre-fire
/// closure also calls.
///
/// A MIRROR of a closure captured into the scheduler at build
/// time would be uncallable, so nothing would compile against both and the two would be free to
/// drift (the duplicated-rule class). `<`, not `<=`: a node whose gap EQUALS the
/// throttle is allowed to fire, and
/// `the_throttle_mirror_and_the_engine_answer_one_hand_oracle` pins both sides
/// of the SHARED function.
pub fn throttle_defers(prior_fires: u64, now_ns: u64, last_fire_ns: u64, throttle_ns: u64) -> bool {
    cerulion_core::graph::runtime::throttle_defers(prior_fires, now_ns, last_fire_ns, throttle_ns)
}

/// The Sync FIRE predicate, delegated to [`cerulion_core::scheduler::sync_aligned`]
/// so the verifier's window rule is ONE function. The LIVE matcher
/// (`cerulion_core::scheduler::sync_match::next_sync_step`) does not call it —
/// it MIRRORS the same inclusive `<=` on its own descent — and the two are
/// pinned against each other by `the_window_boundary_is_inclusive_exactly_as_check_sync_was`
/// (core) and `the_sync_window_boundary_is_inclusive_on_both_sides` (here).
/// (There is no `Scheduler::check_sync`.)
///
/// A MIRROR of a private associated function would have nothing compiled
/// against both, so the two would be free to drift on exactly the rule
/// whose job is to agree (the duplicated-rule class). The rule lives at module scope in
/// the core and this delegates; the oracles below pin the SHARED function.
///
/// Two rules, in the core's own order:
///
/// 1. every declared trigger input must have a stamp (the core additionally
///    short-circuits on `timestamps.len() < inputs.len()` BEFORE calling the
///    shared rule, which cannot change the answer for a node whose input names
///    are unique — it is a fast path, not a rule);
/// 2. bounded sync only: `max(stamps) - min(stamps) <= window` — **inclusive**.
///
/// **The spread is over the WIRE STAMPS, never the consumer's clock.** A stamp
/// is the PRODUCING rank's gating clock at loan time; the consumer's clock only
/// decides WHEN the check runs. Computing the spread against the
/// consumer's `now` is the clock-domain error and would make a
/// perfectly aligned pair read as misaligned by the distance between the two
/// domains — which is what
/// `two_producer_clocks_align_on_their_own_stamps_not_the_consumers` pins.
pub fn sync_aligned(
    stamps: &BTreeMap<String, u64>,
    trigger_inputs: &[String],
    window_ns: Option<u64>,
) -> bool {
    // The RULE is `cerulion_core::scheduler::sync_aligned` — see this
    // function's doc for how the live matcher relates to it. What stays here is
    // the container adaptation: this module holds a `BTreeMap` rebuilt from a
    // bag, so the shared rule takes a LOOKUP and pays no conversion.
    cerulion_core::scheduler::sync_aligned(
        |name| stamps.get(name).copied(),
        trigger_inputs,
        window_ns,
    )
}

/// Does SOME choice of one candidate per trigger input fit inside
/// `window_ns`?
///
/// The FEASIBILITY question `verify_sync`'s arm B asks, rather than a
/// last-wins fold. Under per-set Sync a step
/// can carry SEVERAL head-naming records per input — `tick_sync_burst` runs
/// `fire → align(Refill)` in a loop — and the LAST of them describes the set
/// that was NOT fired (the refill that came up `Incomplete`). Judging the step
/// on that tuple reports a divergence on a healthy recording; judging it on
/// whether ANY tuple was legal cannot.
///
/// Strictly MORE permissive than the fold it replaces, because the fold's value
/// is always among the candidates — so it introduces no findings of its own.
///
/// # The sweep
///
/// Every candidate is tried as the MINIMUM of the tuple. For a fixed minimum
/// `lo`, the spread-minimising choice on every input is its SMALLEST candidate
/// `>= lo`, so one pass over the inputs decides that `lo`. The true minimum of
/// any feasible tuple is itself a candidate, so trying each in turn is complete.
///
/// The `<=` comparison is NOT re-implemented: the chosen tuple is handed to
/// [`sync_aligned`], hence to `cerulion_core::scheduler::sync_aligned`, so the
/// VERIFIER's inclusivity is one function. The LIVE matcher
/// (`sync_match::next_sync_step`) mirrors that `<=` rather than calling it, and
/// is pinned against it by `the_window_boundary_is_inclusive_exactly_as_check_sync_was`
/// (core) — `the_sync_window_boundary_is_inclusive_on_both_sides` drives this
/// side's both halves.
///
/// `None` window (`unbounded_sync`) is PRESENCE alone — the same early return
/// the core's rule takes — and so is a node declaring no trigger inputs at all
/// (defensive; the scheduler refuses such a node at `add_node`).
fn in_window_transversal(
    candidates: &BTreeMap<String, Vec<u64>>,
    trigger_inputs: &[String],
    window_ns: Option<u64>,
) -> bool {
    // Rule 1, the core's own: every declared trigger input must have a stamp.
    for input in trigger_inputs {
        match candidates.get(input) {
            Some(v) if !v.is_empty() => {}
            _ => return false,
        }
    }
    // Unbounded sync (and the no-trigger-input degenerate case): presence is
    // the whole condition, exactly as `cerulion_core::scheduler::sync_aligned`
    // has it.
    if window_ns.is_none() || trigger_inputs.is_empty() {
        return true;
    }

    let mut floors: Vec<u64> = trigger_inputs
        .iter()
        .filter_map(|i| candidates.get(i))
        .flatten()
        .copied()
        .collect();
    floors.sort_unstable();
    floors.dedup();

    let mut chosen: BTreeMap<String, u64> = BTreeMap::new();
    for lo in floors {
        chosen.clear();
        let mut complete = true;
        for input in trigger_inputs {
            let Some(v) = candidates.get(input) else {
                complete = false;
                break;
            };
            match v.iter().copied().filter(|s| *s >= lo).min() {
                Some(s) => {
                    chosen.insert(input.clone(), s);
                }
                None => {
                    complete = false;
                    break;
                }
            }
        }
        if complete && sync_aligned(&chosen, trigger_inputs, window_ns) {
            return true;
        }
    }
    false
}

/// The credit-conservation assert for ONE block edge.
///
/// Conservation holds iff SOME legal interleave of the recorded publishes and
/// drains keeps the edge inside its ceiling. Only the TOTALS decide it, and the
/// argument is short:
///
/// * `drained > published` is impossible outright — a drain consumes a frame,
///   so no ordering produces more drains than publishes;
/// * `published - drained > depth` is impossible because at the END every
///   publish has happened, so the final outstanding count is forced;
/// * otherwise a greedy interleave works — publish while `outstanding < depth`,
///   drain when a frame is available. It cannot stall: reaching `outstanding ==
///   depth` with no drains left means `published == drained + depth`, and
///   `published_total - drained_total <= depth` leaves nothing to publish.
pub fn conservation_breach(published: u64, drained: u64, depth: u32) -> Option<ConservationBreach> {
    if drained > published {
        return Some(ConservationBreach::UnderRun);
    }
    let outstanding = published - drained;
    (outstanding > u64::from(depth)).then_some(ConservationBreach::OverRun { outstanding })
}

// ---------------------------------------------------------------------------
// The engine
// ---------------------------------------------------------------------------

/// Re-derive every rank-local decider over ONE rank's recording and compare it
/// positionally against the recorded fire stream.
///
/// Deterministic: nodes are walked in declaration order and steps in recorded
/// order, so the report is a pure function of its inputs.
///
/// At most ONE finding per (node, decider): after a divergence the re-derived
/// state no longer describes the run, so every later step would mismatch too
/// and the flood would bury the first cause. The first mismatch is reported and
/// that decider stops for that node.
///
/// [`RoleTrust`] says whether this bag's kind-6 records carry
/// TRUSTWORTHY read-site roles (`trace_format >=
/// replay_engine::ROLE_STAMPED_MIN_TRACE_FORMAT`). [`RoleTrust::Declined`] —
/// every archived bag — keeps every rule below on its pre-roles arm, byte for
/// byte. It is a PARAMETER rather than something derived from the records
/// because a reader that keyed on the bits alone would behave identically on
/// today's corpus and would silently start believing whatever a future writer
/// put there.
///
/// `restore` is what a RESUMED pass put into its scheduler before its first
/// judged step ([`PassRestore`]). A `coordination: free_run` bag with ONE
/// worker rank resumes and is judged here, and `Scheduler::restore_sync_input_timestamps`
/// mints UNBACKED heads (a stamp with no kind-6 record), which would make the
/// Sync rule fold a map missing their members: so the restored Sync heads seed
/// the Sync rule's maps, and a from-start pass hands [`PassRestore::none`].
pub(crate) fn verify(
    nodes: &[NodeDecl],
    steps: &[RecordedStep],
    block_edges: &[BlockEdgeObservation],
    trust: RoleTrust,
    restore: &PassRestore,
) -> RederivationReport {
    let mut report = RederivationReport::default();

    for node in nodes {
        let block_gated = block_edges.iter().any(|e| e.producer_node == node.node_id);

        // The RECORDING is
        // judged BEFORE any rule is, over ALL of this node's reads.
        //
        // `unproducible_read_shape` is a claim about the recorder, not about
        // Sync or FIFO; asked only where those two rules
        // happen to look, it would go unasked for a `Period` node, an
        // `Unmodelled` node, a block-gated `Data` node (`verify_fifo` returns
        // before its read loop) and, on every node, for every NON-trigger
        // input, since both loops filter on `trigger_inputs`. A bag can carry a
        // corrupt read shape on any of those and nothing would say so.
        //
        // A node whose stream carries one is stood down WHOLE and its trigger
        // rule is not run: every read-driven rule below would be arithmetic
        // over records the recorder cannot have written. `Decider::ReadLog`
        // rather than a real rule, because no rule looked — see its doc.
        // Exactly ONE stand-down per node, so the caller's per-note `warn!`
        // stays bounded however many corrupt records the stream holds.
        //
        // The two in-rule asks (`sync_read_sites_verdict`, `verify_fifo`) are
        // KEPT as belt: this sweep strictly subsumes them for every call that
        // arrives through `verify`, but each is also the local invariant of a
        // helper its own oracle tests drive directly, and a rule that computes
        // over a corrupt stream when called on its own is a worse default than
        // one redundant branch.
        if node_carries_unproducible_read(&node.node_id, steps, trust) {
            report.stand_downs.push(StandDown {
                node_id: node.node_id.clone(),
                decider: Decider::ReadLog,
                reason: StandDownReason::ImpossibleReadShape,
            });
        } else {
            match &node.trigger {
                TriggerDecl::Period {
                    interval_ns,
                    max_catchup,
                    catchup_cap_override,
                } => {
                    verify_period(
                        node,
                        steps,
                        *interval_ns,
                        effective_max_catchup(*max_catchup, *catchup_cap_override),
                        block_gated,
                        &mut report,
                    );
                }
                TriggerDecl::Sync {
                    window_ns,
                    trigger_inputs,
                } => {
                    verify_sync(
                        node,
                        steps,
                        *window_ns,
                        trigger_inputs,
                        block_gated,
                        trust,
                        restore.sync_heads_for(&node.node_id),
                        &mut report,
                    );
                }
                TriggerDecl::Data { trigger_inputs } => {
                    verify_fifo(node, steps, trigger_inputs, block_gated, trust, &mut report);
                }
                TriggerDecl::Unmodelled => {}
            }
        }

        // The throttle gate is judged for EVERY policy, block edge or not. It
        // is one-directional (below), and block can only SUPPRESS a fire, so a
        // recorded fire the throttle forbids is a divergence whether or not the
        // node also carries credit.
        if let Some(throttle_ns) = node.throttle_ns {
            verify_throttle(node, steps, throttle_ns, &mut report);
        }
    }

    verify_conservation(nodes, steps, block_edges, trust, &mut report);
    census_read_sites(nodes, steps, trust, &mut report);
    report
}

/// Attach a [`ReadSiteCensus`] to every
/// finding that role steering could have MISATTRIBUTED.
///
/// Runs LAST, over the report the deciders just produced, so it is a pure
/// annotation pass and cannot change a verdict. TWO finding classes qualify,
/// and they are exactly the two whose arithmetic role steering still changes:
/// the Sync alignment map and the FIFO pop count.
///
/// # Why `Decider::BlockCredit` is NOT one of them any more
///
/// Filtering it to [`ConservationBreach::UnderRun`] on the ground
/// that "excluding drains can manufacture an under-run" gets the direction
/// INVERTED: excluding reads can only
/// LOWER `drained`, which makes `drained > published` (the under-run)
/// strictly HARDER and `published - drained > depth` (the over-run) strictly
/// EASIER — so the one class the census could reach would be the one it filtered
/// OUT, and the class it could actually mis-diagnose would carry no note.
///
/// The filter is not merely flipped, because [`verify_conservation`] does not
/// EXCLUDE anything by role: on a believed bag it sums every pop-bearing
/// record whatever its role, and where the roles cannot be attributed to a
/// subscriber port it declines outright
/// ([`StandDownReason::UnattributableCredit`]) rather than computing a number
/// off a subset. A conservation FINDING on a believed bag is therefore never
/// derived under a role exclusion, and an arm annotating one would have nothing
/// to say. Pinned by an oracle asserting a believed-bag conservation finding
/// carries NO census, so re-adding an arm fails a test rather than passing
/// silently.
///
/// # One note per finding is already one note per node
///
/// There is no dedup and none is needed: [`verify`] emits at most one finding
/// per `(node, decider)` and a node declares exactly one trigger, so at most
/// one of its findings can ever be census-qualifying. Per-finding emission is
/// what makes that bound structural instead of a rule this pass would have to
/// re-state.
fn census_read_sites(
    nodes: &[NodeDecl],
    steps: &[RecordedStep],
    trust: RoleTrust,
    report: &mut RederivationReport,
) {
    if !trust.believed() {
        // Nothing was excluded, so there is no split to report.
        return;
    }
    let mut pending: Vec<(String, Decider, Vec<String>)> = Vec::new();
    for f in &report.findings {
        if !matches!(f.decider, Decider::Sync | Decider::FifoPop) {
            continue;
        }
        let Some(node) = nodes.iter().find(|n| n.node_id == f.node_id) else {
            continue;
        };
        let inputs = match &node.trigger {
            TriggerDecl::Sync { trigger_inputs, .. } | TriggerDecl::Data { trigger_inputs } => {
                trigger_inputs.clone()
            }
            _ => continue,
        };
        pending.push((f.node_id.clone(), f.decider, inputs));
    }
    for (node_id, decider, inputs) in pending {
        let mut split = Vec::new();
        let mut body_heavy = false;
        for input in &inputs {
            let (mut drain_records, mut body_records) = (0u64, 0u64);
            let (mut drain_popped, mut body_popped) = (0u64, 0u64);
            for read in steps
                .iter()
                .flat_map(|s| s.reads.iter())
                .filter(|r| r.node_id == node_id && &r.input == input)
                .filter(|r| r.kind == ReadOutcomeKind::DrainedBatch)
            {
                match read.role {
                    // The format-5 peek/head mark: a `Peek` counts with `Drain`,
                    // because this census reports what role steering EXCLUDED
                    // from the pop-summing rules — and a peek is excluded from
                    // none of them ([`is_drain_site_read`] admits it). Its
                    // exclusion is from the sync HEAD fold alone, which is a
                    // different question and one a body-heavy split cannot
                    // describe.
                    ReadSiteRole::Drain | ReadSiteRole::Peek => {
                        drain_records += 1;
                        drain_popped = drain_popped.saturating_add(u64::from(read.popped));
                    }
                    ReadSiteRole::Body => {
                        body_records += 1;
                        body_popped = body_popped.saturating_add(u64::from(read.popped));
                    }
                    // A fallback record took the KIND arm and WAS counted, so
                    // it is neither excluded nor evidence of exclusion.
                    ReadSiteRole::Unstamped => {}
                }
            }
            // EITHER scale being body-heavy emits the note. The rules
            // this annotates sum `popped`, so the pop scale is the one that
            // matches their arithmetic; the record scale is kept because a
            // many-small-batches stream is body-heavy in a way the pop sum can
            // hide when the batches are of equal size.
            body_heavy |= body_records > drain_records || body_popped > drain_popped;
            split.push(InputReadSites {
                input: input.clone(),
                drain_records,
                body_records,
                drain_popped,
                body_popped,
            });
        }
        if body_heavy {
            report.read_site_census.push(ReadSiteCensus {
                node_id,
                decider,
                inputs: split,
            });
        }
    }
}

/// The fires of one node within one step, as a [`StepFires`].
fn recorded_fires(step: &RecordedStep, node_id: &str, with_instants: bool) -> StepFires {
    let mut count = 0u32;
    let mut instants = Vec::new();
    for fire in &step.fires {
        if fire.node_id == node_id {
            count = count.saturating_add(1);
            if with_instants {
                instants.push(fire.fire_time_ns);
            }
        }
    }
    // Through the constructors, so the empty-or-`count`-long invariant holds by
    // construction on BOTH arms rather than by this loop happening to agree.
    if with_instants {
        StepFires::at(instants)
    } else {
        StepFires::count(count)
    }
}

fn verify_period(
    node: &NodeDecl,
    steps: &[RecordedStep],
    interval_ns: u64,
    cap: u32,
    block_gated: bool,
    report: &mut RederivationReport,
) {
    if block_gated {
        report.stand_downs.push(StandDown {
            node_id: node.node_id.clone(),
            decider: Decider::Period,
            reason: StandDownReason::BlockGated,
        });
        return;
    }
    if interval_ns == 0 {
        report.stand_downs.push(StandDown {
            node_id: node.node_id.clone(),
            decider: Decider::Period,
            reason: StandDownReason::ZeroInterval,
        });
        return;
    }

    // Phase anchor: the FIRST recorded fire. A Period node's `first_fire_ns`
    // IS the pre-advance deadline the live arm read, so the anchor is exact
    // rather than a guess — and it is the one value no amount of clock
    // arithmetic can recover, which is why the first fire itself is not
    // verified. Everything after it is predicted.
    let Some(anchor_idx) = steps
        .iter()
        .position(|s| s.fires.iter().any(|f| f.node_id == node.node_id))
    else {
        report.stand_downs.push(StandDown {
            node_id: node.node_id.clone(),
            decider: Decider::Period,
            reason: StandDownReason::NoPhaseAnchor,
        });
        return;
    };

    let mut next_fire_ns = steps[anchor_idx]
        .fires
        .iter()
        .find(|f| f.node_id == node.node_id)
        .map(|f| f.fire_time_ns)
        .expect("the anchor step contains a fire of this node (position() found one)");

    for step in &steps[anchor_idx..] {
        let derived = period_step(next_fire_ns, step.clock_ns, interval_ns, cap);
        // Gated on the CAP alone, never on `fire_count > 0`: a step where the
        // re-derivation expects NO fire is exactly the step a spurious recorded
        // fire lands on, and dropping instants there would report
        // `1 fire(s)` with no instant — the diagnostic an operator needs most,
        // withheld at the one moment it exists for.
        let render_instants = derived.fire_count <= PERIOD_BURST_RENDER_CAP;
        let expected = if render_instants {
            StepFires::at(
                (0..derived.fire_count)
                    .map(|k| {
                        derived
                            .first_fire_ns
                            .saturating_add(u64::from(k).saturating_mul(interval_ns))
                    })
                    .collect(),
            )
        } else {
            StepFires::count(derived.fire_count)
        };
        let recorded = recorded_fires(step, &node.node_id, render_instants);

        if expected != recorded {
            report.findings.push(ScheduleFinding {
                node_id: node.node_id.clone(),
                step: step.step,
                decider: Decider::Period,
                expected,
                recorded,
            });
            return;
        }
        next_fire_ns = derived.next_fire_ns;
    }
}

fn verify_throttle(
    node: &NodeDecl,
    steps: &[RecordedStep],
    throttle_ns: u64,
    report: &mut RederivationReport,
) {
    // ONE-DIRECTIONAL, and the direction is forced: the gate says when a fire
    // is FORBIDDEN, never when one is required. `throttle_ms` is mutually
    // exclusive with `period_ms` (rejected at compile time), so a throttled
    // node is data/sync/external-triggered and whether it WANTED to fire at a
    // quiet step is not re-derivable from the clock. A recorded fire the gate
    // forbids is therefore the whole of what this rule can catch — and it is
    // the half that matters, because it is what a changed `throttle_ms`
    // declaration produces.
    let mut prior_fires = 0u64;
    let mut last_fire_ns = 0u64;

    for step in steps {
        for fire in step.fires.iter().filter(|f| f.node_id == node.node_id) {
            if throttle_defers(prior_fires, step.clock_ns, last_fire_ns, throttle_ns) {
                report.findings.push(ScheduleFinding {
                    node_id: node.node_id.clone(),
                    step: step.step,
                    decider: Decider::Throttle,
                    expected: StepFires::none(),
                    recorded: StepFires::at(vec![fire.fire_time_ns]),
                });
                return;
            }
            prior_fires = prior_fires.saturating_add(1);
            last_fire_ns = fire.fire_time_ns;
        }
    }
}

/// Does a node's trigger-input record stream carry reads
/// whose SITE was INFERRED and whose inference has collapsed — or, on a bag
/// whose roles are believed, a `(kind, role)` pair no mint site can produce?
///
/// # The AMBIGUITY half — INFERRED sites only
///
/// Where the site is inferred FROM THE KIND, `DrainedBatch` is not exclusively
/// a drain-site kind. It is what `drain_samples` stages, which the sync drain
/// reaches through `try_receive_timestamps`, and which a node BODY reaches
/// through `AnySubscriber::try_receive` (`pub`, reached from
/// `NodeContext::subscriber`; the accumulate-all pattern). Under Sync those are
/// two SUBSCRIBERS with two cursors on one topic, so their stamps differ — and
/// only the drain's ever reached `sync_input_timestamps`.
///
/// There is no record-level discriminator on such a bag, so this does not guess
/// which record belongs to which site. It reports the collision: **two or more
/// KIND-INFERRED drain-site `DrainedBatch` records for one `(node, input)` in
/// one step**. Detecting it ONCE stands the node down for the WHOLE run rather
/// than for that step — a body that reads an input this way does so on every
/// fire, so the signature is evidence about the node's CODE, not about one step.
///
/// The collision needs a KIND-INFERRED record IN IT.
/// "Kind-inferred" covers an UNSTAMPED bag
/// (`trace_format` <= 4) entirely, and on a STAMPED bag it covers exactly the
/// records whose role decoded to [`ReadSiteRole::Unstamped`] — which, since the
/// format-5 role definition filled the two-bit field, means a literal `0`. Those take
/// the pre-roles KIND arm at every consumer ([`is_drain_site_read`]), so their
/// collisions are exactly as unrecoverable as an archived bag's.
///
/// On a format-5 bag a `0` is unproducible by this recorder: every mint site
/// passes a role as a compile-time constant and `Unstamped` is not one a writer
/// may elect, so it can only arrive from a hand-edited bag or a foreign writer.
/// It is still READ rather than convicted — a corruption claim is reserved for a
/// shape whose site the wire NAMED (see [`unproducible_read_shape`]), and an
/// unnamed site is exactly the pre-roles condition this arm already handles.
///
/// A fold of records whose
/// roles are ALL believed is not a collision at all — every one of them was
/// named by the wire.
///
/// RESIDUAL, on kind-inferred reads only: a firing step in which the
/// drain was SILENT and only the body recorded yields exactly one
/// `DrainedBatch` and is indistinguishable, so it still mis-stamps. It is
/// narrow — a Sync fire is normally the drain's own doing — but it is real.
/// **Role stamping CLOSES it for BELIEVED roles**: such a step contributes no
/// DRAIN-role record at all, so the fold is exactly the scheduler's
/// alignment map.
///
/// # The CORRUPTION half — and the premise it must not rest on
///
/// Returning [`ReadSiteVerdict::Corrupt`] for **two or more
/// DRAIN-role `DrainedBatch` records in one step** would rest on the premise that *the
/// sync drain runs exactly once per step per trigger input*. That premise is
/// FALSE under PER-SET Sync, which is the DEFAULT: one healthy
/// `align()` mints a `FillBoundary` drain (`drain_for_trigger`), PLUS a
/// `sync_peek_next_stamp` pop, PLUS up to `DATA_PENDING_CARRY_CLAMP`
/// `sync_discard_head` pops — every one of them a scheduler-site record on
/// the same `(node, input)` in the same step, because the per-set matcher IS
/// the scheduler reading on the node's behalf. Such a detector would brand
/// HEALTHY per-set Sync recordings "corrupt — re-record the bag", routinely.
/// So there is NO such detector: a count of scheduler-site records in
/// one step is evidence about the MATCHER, not about the recorder, and with
/// the format-5 role definition the wire distinguishes the records a descent mints
/// ([`ReadSiteRole::Peek`]) from the ones that name the head, so the fold is
/// exact for ANY drain count and there is nothing to decline.
///
/// What survives is a claim the mint-site table really supports —
/// [`unproducible_read_shape`], per RECORD rather than per step. See its doc
/// for the evidence.
///
/// # ONE site rule
///
/// What counts as a drain-site read is [`is_drain_site_read`] and nothing else —
/// the same predicate `verify_fifo` and `verify_conservation` steer on, and the
/// one `verify_sync`'s narrower [`is_sync_head_record`] is DEFINED in terms of.
/// A SECOND spelling here would leave the detector blind to a role
/// it did not enumerate: a bare `role == Drain` filter counts zero of those
/// records while the shared predicate feeds every one of them into the sync fold,
/// so neither stand-down can fire and the collision goes silent.
///
/// Across a run, a proven corruption anywhere outranks an ambiguity elsewhere —
/// `Corrupt` is evidence about the RECORDING, and finding it later does not
/// make it less true.
fn sync_read_sites_verdict(
    node_id: &str,
    steps: &[RecordedStep],
    trigger_inputs: &[String],
    trust: RoleTrust,
) -> ReadSiteVerdict {
    let mut ambiguous = false;
    for step in steps {
        for input in trigger_inputs {
            let mut drain_site_reads = 0usize;
            let mut any_inferred = false;
            for read in step
                .reads
                .iter()
                .filter(|r| r.node_id == node_id && &r.input == input)
            {
                if unproducible_read_shape(read, trust) {
                    return ReadSiteVerdict::Corrupt;
                }
                if is_drain_site_read(read, trust) {
                    drain_site_reads += 1;
                    any_inferred |= !believed_role(read, trust);
                }
            }
            // A collision needs BOTH: more than one record in the fold, AND at
            // least one whose site nothing on the wire named. All-BELIEVED
            // drain roles are per-set Sync working exactly as designed (a
            // boundary drain plus the matcher's descent pops), so counting
            // those would convict a healthy bag. One INFERRED record beside a
            // believed one is still a collision — the believed one is
            // certainly the drain, the other is folded in and could be
            // anything — which is the "MIXED sets take the weaker
            // reading" rule.
            if drain_site_reads > 1 && any_inferred {
                ambiguous = true;
            }
        }
    }
    if ambiguous {
        ReadSiteVerdict::Ambiguous
    } else {
        ReadSiteVerdict::Attributable
    }
}

/// What [`sync_read_sites_verdict`] concluded about one node's trigger record
/// stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ReadSiteVerdict {
    /// Every read is attributable to one site — judge normally.
    Attributable,
    /// Kind-INFERRED sites collide: an unstamped bag (`trace_format` <= 4), or a
    /// stamped one whose colliding records carry an unwritten (`0`) role.
    Ambiguous,
    /// A believed record pairs a role with a kind that site cannot mint.
    Corrupt,
}

impl ReadSiteVerdict {
    /// The stand-down this verdict raises, or `None` when there is none.
    fn stand_down(self) -> Option<StandDownReason> {
        match self {
            Self::Attributable => None,
            Self::Ambiguous => Some(StandDownReason::AmbiguousReadSite),
            Self::Corrupt => Some(StandDownReason::ImpossibleReadShape),
        }
    }
}

/// Does this record's ROLE name a real site AND is this bag's format allowed to
/// say so?
///
/// The two halves of "may I steer on these bits", asked in one place so no
/// consumer can check the format and forget the reserved value (or the reverse).
fn believed_role(read: &RecordedRead, trust: RoleTrust) -> bool {
    trust.believed() && read.role.is_stamped()
}

/// Does this record pair a BELIEVED role with an
/// outcome kind that site cannot mint?
///
/// The evidence is the mint-site table in `cerulion_core::transport::subscriber`,
/// where every stage call passes its role as a compile-time constant:
///
/// | kind | minted at |
/// |---|---|
/// | `Served` / `Held` / `NoFrame` | the node BODY's read paths ONLY (`try_view`, `snapshot_latest`) — always [`ReadSiteRole::Body`] |
/// | `DrainedBatch` / `Decimated` | BOTH sites (`DrainedBatch` via `drain_samples` under either role; `Decimated` via `try_view` / `snapshot_latest` under `Body` and via the three drain fns under `Drain`) |
/// | `Truncated` | the STAGE, so it carries the stage's role — and a `Body` stage genuinely holds drain-SITE records under the unified discipline |
/// | `Producer` | its PAIRED read's role, whichever that was |
///
/// The format-5 role definition adds a THIRD role with a much narrower mint set.
/// [`ReadSiteRole::Peek`] marks a record that NAMES A FRAME PARKED in the
/// per-set Sync matcher's `next_head`, and exactly one site can produce one:
/// `sync_peek_next_stamp`'s `Sample` arm, which stages `DrainedBatch` — plus,
/// on an annotated edge, the `Producer` token paired with that same read. Every
/// other kind is unproducible under it, and each for its own reason:
///
/// * `Served` / `Held` / `NoFrame` — body-only kinds, as above;
/// * `Decimated` — the peek's OWN decimated arm deliberately keeps `Drain`
///   (a decimated pop parks nothing, so there is no head to distinguish it
///   from), and no other site mints `Peek` at all;
/// * `Truncated` — the marker is minted by the STAGE from a `ReadStageRole`,
///   whose `From` impl is total over two variants and maps to `Body`/`Drain`.
///
/// So the unproducible pairings are `Drain` on a body-only kind, and `Peek` on
/// anything but `DrainedBatch`/`Producer`. Neither is an inference that could
/// be wrong: the format says the bits may be read, the bits name a site, and no
/// code path in the recorder can write those records. CORRUPTION, reported as
/// [`StandDownReason::ImpossibleReadShape`].
///
/// Deliberately NOT judged: `Body` on `DrainedBatch`/`Decimated` (legitimate —
/// the accumulate-all `try_receive` this whole rule exists to fend off),
/// `Drain` on `Truncated`/`Producer` (both roles legitimate, see the table),
/// and every pairing on a record whose role is not believed (a corruption claim
/// must rest on bits we trusted).
fn unproducible_read_shape(read: &RecordedRead, trust: RoleTrust) -> bool {
    if !believed_role(read, trust) {
        return false;
    }
    match read.role {
        ReadSiteRole::Drain => matches!(
            read.kind,
            ReadOutcomeKind::Served | ReadOutcomeKind::Held | ReadOutcomeKind::NoFrame
        ),
        ReadSiteRole::Peek => !matches!(
            read.kind,
            ReadOutcomeKind::DrainedBatch | ReadOutcomeKind::Producer
        ),
        // A body site mints every read kind, and an unbelieved role convicts
        // nothing (guarded above).
        ReadSiteRole::Body | ReadSiteRole::Unstamped => false,
    }
}

/// Does ANY read this node
/// recorded — on ANY input — pair a believed role with a kind that site cannot
/// mint?
///
/// The whole-node scope is the point. [`unproducible_read_shape`] is evidence
/// about the RECORDER, so it must not be scoped to whichever inputs a
/// particular rule happens to fold; see the sweep in [`verify`] for what that
/// scoping would miss.
fn node_carries_unproducible_read(node_id: &str, steps: &[RecordedStep], trust: RoleTrust) -> bool {
    steps
        .iter()
        .flat_map(|s| s.reads.iter())
        .filter(|r| r.node_id == node_id)
        .any(|r| unproducible_read_shape(r, trust))
}

/// Is this recorded read a DRAIN-site read — a read that CONSUMED
/// from the input's queue on the scheduler's behalf, whose pops paced the FIFO
/// and whose frames a `block` edge's credit accounting is conserved against?
///
/// On a bag whose roles are BELIEVED the wire says so, and BOTH scheduler sites
/// answer yes: [`ReadSiteRole::Drain`] and [`ReadSiteRole::Peek`]. A peek
/// really did take a frame out of the queue — its record is the ONLY place that
/// pop is accounted (its later promotion carries `popped: 0`) — so excluding it
/// here would lose a pop from the FIFO count and from `block` credit.
/// Otherwise the KIND is the site vocabulary and `DrainedBatch` is the whole of
/// it — the pre-roles rule, verbatim, which is what keeps every archived bag
/// reading exactly as it did.
///
/// # It is NOT the same question as [`is_sync_head_record`]
///
/// This one asks "did this read pop the queue"; that one asks "does this record
/// NAME THE HEAD the matcher aligned on". A peek answers yes here and no there,
/// which is exactly the distinction the `Peek` role was minted to carry — so
/// the two are separate predicates rather than one, and the narrower is defined
/// in terms of this one so they cannot come to disagree about the KIND rule.
///
/// ONE predicate for the pop-summing consumers (`verify_fifo`,
/// `verify_conservation`, and `sync_read_sites_verdict`'s collision test),
/// because three copies of one site rule is how they drift apart — which they
/// already had to be talked back into agreeing on once.
fn is_drain_site_read(read: &RecordedRead, trust: RoleTrust) -> bool {
    if read.kind != ReadOutcomeKind::DrainedBatch {
        return false;
    }
    if believed_role(read, trust) {
        return matches!(read.role, ReadSiteRole::Drain | ReadSiteRole::Peek);
    }
    true
}

/// The format-5 peek/head mark: does this record name the HEAD `verify_sync` must
/// fold — the frame the per-set Sync matcher was holding when it decided?
///
/// STRICTLY NARROWER than [`is_drain_site_read`], and the narrowing is the
/// whole of the peek/head mark. Every record it admits is a drain-site read;
/// what it additionally excludes, on a BELIEVED bag, is
/// [`ReadSiteRole::Peek`] — a frame the matcher popped to LOOK at and parked in
/// `next_head`. A peek's stamp is NEWER than the head's, and admitting it here
/// moves arm C's NARROW map to a frame the matcher parked, so an aligned and
/// unfired step reads as aligned on nothing and the owed-fire finding VANISHES
/// — pinned by `a_peek_stamp_never_explains_a_missing_fire`.
///
/// What REMAINS admitted is the set of records that say "the head is now this
/// frame": the boundary fill, the `Advance`/`DiscardTie` refill, and
/// `sync_discard_head`'s promotion of an already-peeked frame (a `Drain`-role
/// record at the promoted sequence with `popped: 0`). `drain_for_trigger`'s
/// R-promote — the post-fire refill promoting a parked frame — deliberately
/// records NOTHING (`transport::subscriber` states it as a decision), so the
/// NARROW map is NOT exact for every drain count: a head installed that way is
/// invisible here, which is the hole `verify_sync`'s WIDE set closes by
/// admitting the peek that parked it. A missing narrow head can only SUPPRESS
/// arm C, which is the direction that rule may err in.
///
/// On a bag whose roles cannot be believed this collapses to
/// [`is_drain_site_read`] — the pre-roles kind rule, byte for byte, which is
/// what keeps every archived bag reading as it did (and why such a stream's
/// collisions are still stood down as
/// [`StandDownReason::AmbiguousReadSite`] rather than folded).
fn is_sync_head_record(read: &RecordedRead, trust: RoleTrust) -> bool {
    if !is_drain_site_read(read, trust) {
        return false;
    }
    if believed_role(read, trust) {
        return read.role == ReadSiteRole::Drain;
    }
    true
}

/// Re-derive one Sync node's fire COUNT per step, over a
/// COUNTING + FEASIBILITY model.
///
/// # Why one end-of-step alignment check is not enough
///
/// A simpler model re-derives ONE quantity — "was the last-recorded head stamp
/// per trigger input in-window at the end of this step" — and asks three
/// questions of it. Under per-set Sync (the DEFAULT delivery discipline)
/// that quantity is not the scheduler's head map, and two of the three
/// questions are verdict-bearing (`ScheduleFinding` ⇒ exit 6):
///
/// * `count > 1 && aligned ⇒ expected 1` is FALSE on every healthy burst.
///   `decide_node`'s Sync arm answers with `FireKind::Sync { max_sets }` and
///   `tick_sync_burst` loops `fire → mark_sync_heads_emitted → align(Refill)`,
///   so several arrivals per input in one step legitimately fire several sets.
///   Sync never produces `FireKind::Single`: it answers with
///   `FireKind::Sync { max_sets }`, and `Single` exists only for External.
/// * `count > 0 && !aligned ⇒ expected none` is FALSE whenever the burst's
///   LAST refill comes up `Incomplete` out of window: that fold is last-wins, so
///   it judges the step against exactly the tuple the scheduler did NOT fire.
///
/// # The model
///
/// Two parallel per-input maps, both built by walking the step's reads in
/// recorded order:
///
/// * the **WIDE** set — [`is_drain_site_read`] (`Drain` ∪ `Peek` where roles
///   are believed). It feeds SUPPLY (arm A) and CANDIDATES (arm B). Admitting
///   `Peek` is what makes the silent R-promote free: a peek pops exactly one
///   frame into `next_head`, that frame becomes at most one head, and the peek
///   record accounts for exactly one unit of supply at exactly the right stamp.
///   When the promote is the RECORDED one (`sync_discard_head` mints a
///   `Drain`-at-`popped: 0` beside its peek) that one frame supplies TWO units;
///   the over-count is deliberate and harmless, because A and B are upper
///   bounds and an upper bound that is too high can only decline to convict.
/// * the **NARROW** map — [`is_sync_head_record`] (`Drain` only). It feeds the
///   OWED-FIRE arm (C), where the format-5 peek/head mark stays load-bearing:
///   admitting a peek there re-creates the false finding that mark prevents.
///
/// The three arms, in the order they are asked:
///
/// | arm | the question | evidence | `expected` |
/// |---|---|---|---|
/// | B | can NO in-window set be formed from the arrivals at all? | WIDE candidates | `none` |
/// | A | is the fire count above what the arrivals can supply? | WIDE supply | `count(bound)` |
/// | C | was a set aligned and unfired? | NARROW map | `count(1)` |
///
/// B before A because "no legal set exists at all" is the root cause and "too
/// many sets" the symptom. A and B are OVER-APPROXIMATIONS by construction —
/// they admit every head the recording could have held, including ones the wire
/// cannot see — so neither can false-fire. C keeps the exact model and gains
/// two guards (below).
///
/// # Carry — the only cross-step state
///
/// Per input, per map: `supply = carry_in + records_this_step`, and
/// `supply − k` heads SURVIVE the step, where `k` is the recorded fire count.
/// Heads arrive in per-input chronological record order and a fire consumes
/// the OLDEST, so the survivors are the LAST `supply − k` stamps — which is
/// the only order the recording preserves (across inputs of one node the
/// level-end read merge destroys chronology, so no per-FIRE re-derivation is
/// expressible — reads and fires carry no shared ordinal, and that is the whole
/// argument).
///
/// The WIDE carry keeps EVERY survivor's stamp, not just the
/// last one: a step routinely ends with an input holding a head AND a parked
/// `next_head` — a `NeedStamp` peek whose fire `tick_sync_burst`'s pre-fire
/// check then DEFERRED (`throttle_ms`, a `block` input), or simply the ordinary
/// post-fire shape — and a one-stamp carry kept the PARKED frame while
/// dropping the head the node fired on next step, so arm B could find no
/// tuple and convicted a healthy bag. A one-BIT carry had the matching flaw on
/// supply: two frames held across a boundary counted as one. The NARROW map
/// carries one stamp: it names the head the matcher is believed to HOLD, and
/// it is consumed by the `supply > k` rule without the widening.
///
/// **The `has_non_trigger_inputs` widening applies to the WIDE map ONLY.** On
/// such a node a recorded fire may have consumed NOTHING (the tick chain
/// collapsed on the context input; [`NodeDecl::has_non_trigger_inputs`]), so
/// the carry credits every fire with consuming ZERO heads: the WHOLE supply
/// survives the step (keeping ONE survivor, the most
/// recent stamp, would drop a held head whenever an input held a head AND a
/// parked `next_head` across a collapsed fire, and arm B would convict the next
/// real fire on the missing head). That is an UPPER bound on the heads the
/// input could have held, exactly what arms A and B need. Applying it to the
/// NARROW map would make arm C claim heads the scheduler may not have had,
/// i.e. it would CONVICT on a widening, and a widening must only ever permit.
/// Arm C's own soundness rests on the opposite reading: the silent re-offer
/// makes the NARROW map MISS a head, which can only SUPPRESS the arm.
///
/// # Arm C's stale-OPTIMISTIC sources — two guards, one unreachable by construction
///
/// In a `k == 0` step the only align pass is the boundary one, so the
/// end-of-step NARROW map IS the map `decide_node` saw — unless it has gone
/// stale-OPTIMISTIC, which it can in THREE ways:
///
/// * an EPOCH RESET voids partner heads with no record — caught by per-input
///   head-stamp MONOTONICITY, since a backward jump is what defines a reset
///   ([`StandDownReason::SyncHeadStampRegressed`], the monotonicity guard);
/// * a `DiscardTie` whose advance answered NOTHING leaves the discarded low
///   stamp standing (`mark_head_spent`, an `Emitted` tombstone, no record).
///   Monotonicity alone bounds the damage only while the input stays quiet
///   (partners move UP, so the spread stays above the window); the input's
///   NEXT fill is a real record that REPLACES the stamp, and the phantom's
///   unit of supply then survived a fire that consumed the fill (the
///   arm convicted a healthy run one quiet step later). So the spend is
///   MODELLED: a complete out-of-window NARROW tuple at a step's end drops its
///   minimum-stamp inputs, the exact set a `DiscardTie` advances (the
///   spent-tie guard);
/// * `restore_sync_input_timestamps`' UNBACKED heads have no record at all —
///   they are SEEDED from the pass's [`PassRestore`] instead (the maps start
///   from the heads the scheduler was restored with), so a resumed one-rank
///   free-run pass is judged from the state it really began in. This is a
///   seed rather than a guard; the "g5" numeral once used for a guard here
///   stays retired.
///
/// # Residuals
///
/// * a node the panic circuit breaker DISABLED mid-run stops firing while its
///   heads stay filled, so arm C would call the quiet steps owed fires — loud
///   on the recording side;
/// * an align pass that exhausts its op budget (a loud wiring desync) skips one
///   step's fire, likewise;
/// * [`ReadOutcomeKind::Decimated`] on a trigger input is ignored — a decimated
///   pop installs no head, so it moves none of the three arms (`verify_fifo`
///   declines on it for a different reason: pop counts).
// Seven arguments. Bundling any subset into a struct would break the shape it
// shares with `verify_period` / `verify_fifo` / `verify_throttle` — every one of
// them is `(node, steps, its own declared parameters, the gates, report)` — and
// a per-rule parameter bag for exactly ONE of the four is a worse boundary than
// the argument count it hides. Each argument is a NAMED type or a declared
// value, never a positional bool pair (`RoleTrust` is a two-state enum
// precisely so a swapped call site cannot compile).
#[allow(clippy::too_many_arguments)]
fn verify_sync(
    node: &NodeDecl,
    steps: &[RecordedStep],
    window_ns: Option<u64>,
    trigger_inputs: &[String],
    block_gated: bool,
    trust: RoleTrust,
    restored: Option<&BTreeMap<String, u64>>,
    report: &mut RederivationReport,
) {
    // Asked BEFORE any stamp is folded: an edge carrying both sites' reads has
    // no recoverable alignment map, and a verdict built on one would be a
    // guess. Whole-run scope — see the helper.
    if let Some(reason) =
        sync_read_sites_verdict(&node.node_id, steps, trigger_inputs, trust).stand_down()
    {
        report.stand_downs.push(StandDown {
            node_id: node.node_id.clone(),
            decider: Decider::Sync,
            reason,
        });
        return;
    }
    // Asked BEFORE the first step is
    // judged, not at the carry cut. The cut runs only on a step that passed
    // arms B, A and C, so a check living there let an over-room node be
    // CONVICTED on its first step — a verdict on a node this rule says is
    // unjudgeable (and one the operator could not tell from a sound one).
    // Whole-run scope, like the read-site verdict above: the room is a
    // property of the node's queues, not of any step.
    if queue_exceeds_verifier_room(node, trigger_inputs) {
        report.stand_downs.push(StandDown {
            node_id: node.node_id.clone(),
            decider: Decider::Sync,
            reason: StandDownReason::QueueExceedsVerifierRoom,
        });
        return;
    }
    // A gate that can only SUPPRESS a fire makes "aligned but did not fire"
    // un-judgeable, so that direction stands down while "fired while
    // misaligned" and "fired more sets than the arrivals supply" stay live.
    let suppressible = block_gated || node.throttle_ns.is_some();
    if suppressible {
        report.stand_downs.push(StandDown {
            node_id: node.node_id.clone(),
            decider: Decider::Sync,
            reason: StandDownReason::SuppressibleFire,
        });
    }
    // The carry widening is OPERATOR-VISIBLE: the rule DOES run, so this is a
    // note rather than a stand-down — but a reader of a clean Sync verdict on
    // such a node should know it was judged on wider evidence.
    // Once per node, structurally: `verify_sync` runs once per node. WITHDRAWN
    // (`withdraw_carried_heads`) on the three in-loop stand-downs below, since
    // a node that was not judged cannot have been judged on carried heads;
    // a FINDING keeps it. `SuppressibleFire` above is not a
    // withdrawal: arms A and B still run under it.
    let note_at = report.carried_heads.len();
    if node.has_non_trigger_inputs {
        report.carried_heads.push(CarriedHeadsNote {
            node_id: node.node_id.clone(),
        });
    }
    // WIDE carry: per input, the STAMPS of every head this rule admits the
    // matcher may still hold at the step boundary — the `supply − k` most
    // recent ones (see the fn doc's carry section), each a unit of the next
    // step's supply AND a candidate for its transversal. A `Vec`, not one
    // stamp: a step can end with an input holding a head AND a parked
    // `next_head` (a `NeedStamp` peek whose fire the pre-fire check DEFERRED —
    // the first-fire-defer shape (see
    // `a_head_beside_a_parked_peek_survives_a_deferred_fire_into_the_next_step`
    // below), and the ordinary post-fire
    // shape too), and a carry that kept only the LAST stamp dropped the head
    // the node then fired on, convicting a healthy bag.
    let mut wide_carry: BTreeMap<String, Vec<u64>> = BTreeMap::new();
    // The carry is BOUNDED at each input's physical ceiling — its queue depth
    // plus the frozen head plus the parked `next_head` (`depth + 2`, see the
    // cut below) — on EVERY node, so it can never reach `SYNC_CANDIDATE_CAP`
    // (const-asserted beside that constant). That bound is what makes the
    // in-step candidate cap a per-STEP condition with no cross-step state:
    // a carry cut for ROOM would drop the OLDEST heads, exactly what a slow
    // partner's next arrival aligns with, and every rule tried for
    // remembering that loss (a one-step flag; a per-input mark cleared on an
    // empty carry) was either spent by a quiet step or silenced arm B for the
    // rest of the run on a fast-input windowed pair, whose carry grows by
    // pops minus fires and would have reached the room cap in seconds.
    // A head more than `depth + 2` pops old
    // is a head the scheduler had already dropped, so cutting there loses
    // nothing the matcher holds.
    // NARROW map: the head this rule believes the matcher was holding. Its own
    // carry is the ordinary `supply > k` rule — never the carry widening.
    let mut narrow: BTreeMap<String, u64> = BTreeMap::new();
    // The highest NARROW stamp ever SEEN on each input, which is a different
    // quantity from the live head above and outlives it: the monotonicity guard is a claim
    // about the RECORD STREAM (head-naming stamps are non-decreasing within one
    // publisher epoch), and a head consumed by a fire leaves the map while the
    // stream's floor stands. Keying the guard on the live map would silence it
    // for exactly the run shapes it exists to catch — every fire clears the
    // head it would have compared against.
    let mut narrow_floor: BTreeMap<String, u64> = BTreeMap::new();
    // The heads a RESUMED pass restored
    // before its first judged step ([`PassRestore`]). Each is a head the
    // scheduler really holds (unbacked, but FILLED), so it seeds all three
    // maps exactly as a drain record naming that stamp would have: the NARROW
    // head (arm C's alignment and the narrow supply), the g4 floor (a later
    // head-naming stamp below a restored one is the same epoch reset it is
    // for a recorded one), and the WIDE carry (a candidate for the first
    // step's transversal and one unit of its supply). Only this node's
    // TRIGGER inputs: the section is keyed by the inputs the matcher aligns,
    // and a stray key names nothing this rule folds.
    if let Some(heads) = restored {
        for input in trigger_inputs {
            if let Some(&ts) = heads.get(input) {
                narrow.insert(input.clone(), ts);
                narrow_floor.insert(input.clone(), ts);
                wide_carry.insert(input.clone(), vec![ts]);
            }
        }
    }

    for step in steps {
        // Per-step WIDE evidence. `records` is the SUPPLY scale (a count, never
        // stored) and `candidates` the FEASIBILITY scale (bounded by
        // `SYNC_CANDIDATE_CAP`).
        let mut wide_records: BTreeMap<String, u32> = BTreeMap::new();
        let mut candidates: BTreeMap<String, Vec<u64>> = BTreeMap::new();
        // EVERY wide stamp this step saw on each
        // input, in record order and UNCAPPED — the cross-step carry is cut
        // from the tail of this, because `candidates` stops retaining stamps
        // at `SYNC_CANDIDATE_CAP` and its last entry on an over-cap step is
        // the cap-boundary record's stamp, not the head the matcher really
        // kept. Reading the carry off the capped vector convicted a valid
        // trace (1,025 records on one input, the next step's partner aligning
        // only with record 1,025) with a false Sync finding. Bounded by the
        // step's own read count, which is already in memory.
        let mut step_stamps: BTreeMap<String, Vec<u64>> = BTreeMap::new();
        let mut narrow_records: BTreeMap<String, u32> = BTreeMap::new();
        let mut candidates_capped = false;
        // The NARROW map as it stood BEFORE this step's records — the other
        // half of the narrow supply.
        let narrow_before: BTreeMap<String, u64> = narrow.clone();

        // One admission rule for the feasibility scale, whatever the source:
        // a stamp past the cap is COUNTED (supply) but not RETAINED, and the
        // step's arm B then declines rather than judging a set it cannot see.
        let admit_candidate = |candidates: &mut BTreeMap<String, Vec<u64>>,
                               capped: &mut bool,
                               input: &str,
                               ts: u64| {
            let slot = candidates.entry(input.to_string()).or_default();
            if slot.len() < SYNC_CANDIDATE_CAP {
                slot.push(ts);
            } else {
                *capped = true;
            }
        };

        // Seed the candidate sets with the carried heads, so a step with no
        // arrivals on an input can still form a tuple with the heads it kept.
        for input in trigger_inputs {
            if let Some(carried) = wide_carry.get(input) {
                for &ts in carried {
                    admit_candidate(&mut candidates, &mut candidates_capped, input, ts);
                }
            }
        }

        for read in step
            .reads
            .iter()
            .filter(|r| r.node_id == node.node_id && trigger_inputs.contains(&r.input))
        {
            match read.kind {
                // A deterministic hole in the record stream means neither map
                // is any longer the map the scheduler held.
                ReadOutcomeKind::Truncated => {
                    withdraw_carried_heads(report, note_at);
                    report.stand_downs.push(StandDown {
                        node_id: node.node_id.clone(),
                        decider: Decider::Sync,
                        reason: StandDownReason::TruncatedReadLog,
                    });
                    return;
                }
                // A real ARRIVAL AT THE SYNC DRAIN is what supplies a head. The
                // vocabulary is the SITE (`ReadOutcomeKind::DrainedBatch`'s own
                // doc: it "distinguishes SITE ROLE (trigger drain vs body
                // read), never queue discipline"), and the caller feeds this
                // module BOTH sites' records under one `(node, input)`.
                //
                // `Served` is deliberately NOT here: `sync_input_timestamps` is
                // written off the DRAIN, never by the node body's `try_view`,
                // and under Sync the body reads a SEPARATE subscriber with its
                // own cursor — so a frame landing between the drain
                // and the tick would move the map with a stamp the matcher
                // never saw. `Held` is not an arrival either (the hold replays
                // the frame the map already carries); `NoFrame` / `Decimated`
                // deliver nothing.
                //
                // On a ROLE-STAMPED bag the exclusion is by ROLE
                // rather than by kind, so a BODY `DrainedBatch` (the
                // accumulate-all `try_receive` this rule exists to fend off) is
                // excluded because the wire SAYS it is a body read.
                ReadOutcomeKind::DrainedBatch if is_drain_site_read(read, trust) => {
                    // Asked over the WIDE set, not just the head-naming one: a
                    // stamp-less candidate cannot enter the transversal and a
                    // stamp-less record cannot be placed, so silently dropping
                    // it would LOWER supply and NARROW the candidate set —
                    // both in the convicting direction.
                    let Some(ts) = read.wire_stamp_ns else {
                        withdraw_carried_heads(report, note_at);
                        report.stand_downs.push(StandDown {
                            node_id: node.node_id.clone(),
                            decider: Decider::Sync,
                            reason: StandDownReason::MissingWireStamp,
                        });
                        return;
                    };
                    *wide_records.entry(read.input.clone()).or_insert(0) += 1;
                    step_stamps.entry(read.input.clone()).or_default().push(ts);
                    admit_candidate(&mut candidates, &mut candidates_capped, &read.input, ts);

                    if is_sync_head_record(read, trust) {
                        // The monotonicity guard: within one publisher epoch a
                        // head-naming stamp is non-decreasing, so a backward
                        // jump is an EPOCH RESET — which voids partner heads
                        // with no record at all.
                        if let Some(prev) = narrow_floor.get(&read.input) {
                            if ts < *prev {
                                withdraw_carried_heads(report, note_at);
                                report.stand_downs.push(StandDown {
                                    node_id: node.node_id.clone(),
                                    decider: Decider::Sync,
                                    reason: StandDownReason::SyncHeadStampRegressed,
                                });
                                return;
                            }
                        }
                        narrow_floor.insert(read.input.clone(), ts);
                        *narrow_records.entry(read.input.clone()).or_insert(0) += 1;
                        narrow.insert(read.input.clone(), ts);
                    }
                }
                _ => {}
            }
        }

        let recorded = recorded_fires(step, &node.node_id, false);
        let k = recorded.count;

        // ── arm B: FEASIBILITY ────────────────────────────────────────────
        //
        // Skipped when the candidate cap bit: a verifier must not convict
        // because it ran out of room (the conservative arm is "a transversal
        // exists").
        if k > 0
            && !candidates_capped
            && !in_window_transversal(&candidates, trigger_inputs, window_ns)
        {
            report.findings.push(ScheduleFinding {
                node_id: node.node_id.clone(),
                step: step.step,
                decider: Decider::Sync,
                expected: StepFires::none(),
                recorded,
            });
            return;
        }

        // ── arm A: the SET-SUPPLY ceiling ─────────────────────────────────
        //
        // `k` fires need `k` heads on EVERY trigger input, and the scheduler
        // itself stops at `SYNC_BURST_MAX_SETS`. Both bounds are inequalities
        // over an over-approximated supply, so the arm is sound under either
        // delivery discipline: legacy supplies one arrival per input per step
        // and re-derives the "exactly once" answer, per-set admits the burst.
        //
        // `expected` is an UPPER BOUND rendered as a count; `ScheduleFinding`'s
        // `Display` ("re-derived N fire(s), recorded M fire(s)") reads
        // correctly for it.
        let expected_max = trigger_inputs
            .iter()
            .map(|input| {
                carried_heads(&wide_carry, input)
                    .saturating_add(wide_records.get(input).copied().unwrap_or(0))
            })
            .min()
            .unwrap_or(u32::MAX)
            .min(SYNC_BURST_MAX_SETS);
        if k > expected_max {
            report.findings.push(ScheduleFinding {
                node_id: node.node_id.clone(),
                step: step.step,
                decider: Decider::Sync,
                expected: StepFires::count(expected_max),
                recorded,
            });
            return;
        }

        // ── arm C: the OWED FIRE ──────────────────────────────────────────
        if k == 0 && sync_aligned(&narrow, trigger_inputs, window_ns) && !suppressible {
            report.findings.push(ScheduleFinding {
                node_id: node.node_id.clone(),
                step: step.step,
                decider: Decider::Sync,
                expected: StepFires::count(1),
                recorded,
            });
            return;
        }

        // ── carry forward ─────────────────────────────────────────────────
        //
        // A head survives a step iff supply > fires: a fire consumes ONE head
        // per input, and per-set tombstones only the FILLED heads and re-aligns
        // mid-step, so a post-fire refill legitimately leaves a head standing
        // (a whole-map clear on every fire is the LEGACY discipline, and reads a
        // healthy burst's refill as a phantom). The survivors are the LAST
        // `supply − k` stamps in per-input chronological order — carried-in
        // first, then this step's records — because a fire consumes the
        // OLDEST head and the matcher's parked `next_head` is always its most
        // recent pop. Cut from `step_stamps` (uncapped), never from
        // `candidates` (the capped vector's last
        // entry on an over-cap step is the cap-boundary record, not the head
        // the matcher kept), and bounded at the input's PHYSICAL ceiling
        // (`depth + 2`) on every node — see the cut below.
        for input in trigger_inputs {
            let h = wide_records.get(input).copied().unwrap_or(0);
            let supply = carried_heads(&wide_carry, input).saturating_add(h);
            // `supply >= k` here: arm A returned on the opposite.
            //
            // The carry widening (collapse × carry): on a
            // node with a non-trigger input a recorded fire may have CONSUMED
            // NOTHING — the tick chain collapsed on the context input before it
            // read any trigger head, and every head stays in its frozen slot to
            // be re-offered silently at the next boundary. The recording cannot
            // tell such a fire from a real one, so the only sound carry is to
            // credit a fire on a `has_non_trigger_inputs` node with consuming
            // ZERO heads: every head of this step's supply survives (bounded at
            // `depth + 2`; see the cut below). The WIDE map
            // only (see the fn doc); the NARROW map below is consumed without
            // the widening.
            // The input's PHYSICAL ceiling: it can never hold more heads than
            // its queues (one per publisher connection, each of its declared
            // depth — `trigger_input_capacities`) plus the frozen head plus
            // the parked `next_head`, so a stamp past `capacity + 2` newer
            // pops is a head the scheduler had already dropped — on EVERY
            // node (the first bound covered only the widened branch, whose fires
            // consume nothing and whose carry would otherwise grow with the
            // run; the ordinary branch grows the
            // same way on a fast-input windowed pair, by pops minus fires,
            // and a carry allowed to reach the room cap pinned arm A at the
            // burst cap and skipped arm B for the rest of the run, letting a
            // narrowed window replay CLEAN). The bound keeps the NEWEST
            // heads, which is what the matcher holds under drop-oldest.
            // A ceiling past the carry ROOM is a STAND-DOWN, never a clamp:
            // clamped AT the cap the carry filled the feasibility vector and
            // arm B went inert for the rest of the run; cut BELOW the
            // queue it drops heads the matcher holds.
            // That stand-down is decided ONCE, before the first step is
            // judged (`queue_exceeds_verifier_room`) — deciding it
            // here let a step that convicted before reaching the cut judge a
            // node this rule calls unjudgeable — so every ceiling this loop
            // sees fits the room.
            let ceiling = input_carry_ceiling(node, input);
            debug_assert!(
                ceiling <= SYNC_CARRY_ROOM,
                "an over-room input is stood down before the first step is judged"
            );
            let survivors = if node.has_non_trigger_inputs {
                // A widened fire consumes nothing: every head of the supply
                // survives, up to the ceiling.
                (supply as usize).min(ceiling)
            } else {
                (supply.saturating_sub(k) as usize).min(ceiling)
            };
            // `ceiling <= SYNC_CARRY_ROOM < SYNC_CANDIDATE_CAP` here, so the
            // carry can never seed a capped candidate set on its own.
            let mut stamps = wide_carry.remove(input).unwrap_or_default();
            if let Some(seen) = step_stamps.get(input) {
                stamps.extend_from_slice(seen);
            }
            if stamps.len() > survivors {
                stamps.drain(..stamps.len() - survivors);
            }
            if !stamps.is_empty() {
                wide_carry.insert(input.clone(), stamps);
            }

            // The NARROW map is consumed on the SAME rule and WITHOUT the carry
            // widening (see the fn doc): a head this rule can no longer account
            // for is REMOVED, so a quiet step after a fire is not read as an
            // owed fire on a map the scheduler had already spent.
            let narrow_supply = u32::from(narrow_before.contains_key(input))
                .saturating_add(narrow_records.get(input).copied().unwrap_or(0));
            if narrow_supply <= k {
                narrow.remove(input);
            }
        }

        // The `DiscardTie` SPEND, modelled — arm C's third guard.
        //
        // The scheduler's last align pass of this step ended on something other
        // than `Fire` (a fire would have raised `k`, and an in-window complete
        // tuple with `k == 0` was arm C's business above). When the NARROW
        // tuple is COMPLETE and OUT OF WINDOW at the step's end, that pass
        // reached `SyncStep::DiscardTie` on the inputs at the MINIMUM stamp and
        // advanced each; a refill that answered a frame is a `Drain` record
        // this map already holds, so a min-stamp input still standing here is
        // one whose advance answered NOTHING — `mark_head_spent`, an `Emitted`
        // tombstone with NO record. Left in the map it is a PHANTOM: the
        // `supply > k` rule above credited it on the input's next fired step
        // (carried-in `1` + one fresh fill = `2 > 1`), so the fill's stamp
        // survived a fire that consumed it, and the following quiet step read
        // as an owed fire — `ScheduleFinding`, exit 6, on a recording the
        // scheduler executed correctly. Removal is the safe direction (fewer
        // narrow heads can only SUPPRESS arm C), and it is exact on the
        // silent-spend shape; an epoch reset's silently voided partners land
        // here too, since the resetting stamp is the new maximum.
        if let Some(window) = window_ns {
            let complete = trigger_inputs.iter().all(|i| narrow.contains_key(i));
            if complete {
                let (lo, hi) = trigger_inputs
                    .iter()
                    .filter_map(|i| narrow.get(i).copied())
                    .fold((u64::MAX, 0u64), |(lo, hi), ts| (lo.min(ts), hi.max(ts)));
                if hi.saturating_sub(lo) > window {
                    for input in trigger_inputs {
                        if narrow.get(input) == Some(&lo) {
                            narrow.remove(input);
                        }
                    }
                }
            }
        }
    }
}

/// Withdraw the [`CarriedHeadsNote`] `verify_sync` pushed for a
/// node it then STOOD DOWN on mid-loop.
///
/// The note says "every rule ran, on widened evidence"; a stand-down says "no
/// rule ran". Both in one report about one node is a contradiction, so the
/// three in-loop stand-downs call this before returning. A FINDING keeps the
/// note: a conviction is a judgment, and it was reached on the widened carry.
fn withdraw_carried_heads(report: &mut RederivationReport, note_at: usize) {
    report.carried_heads.truncate(note_at);
}

/// A trigger input's PHYSICAL carry ceiling — its effective
/// capacity (declared depth × publisher connections,
/// [`NodeDecl::trigger_input_capacities`]; an unnamed input reads as one
/// connection at the default depth) plus the frozen head plus the parked
/// `next_head`. The most heads the scheduler can still hold, so the most the
/// verifier may carry across a step for that input.
fn input_carry_ceiling(node: &NodeDecl, input: &str) -> usize {
    let capacity = node
        .trigger_input_capacities
        .get(input)
        .copied()
        .unwrap_or(cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH as u32);
    capacity as usize + 2
}

/// PURE — does ANY trigger input's carry ceiling exceed
/// [`SYNC_CARRY_ROOM`]? Asked by `verify_sync` ONCE per node, before the first
/// step is judged: such a node is stood down
/// ([`StandDownReason::QueueExceedsVerifierRoom`]) rather than judged on the
/// steps that happen to convict before the carry cut would have noticed (a
/// check living at the cut is one a
/// convicting step never reaches).
fn queue_exceeds_verifier_room(node: &NodeDecl, trigger_inputs: &[String]) -> bool {
    trigger_inputs
        .iter()
        .any(|input| input_carry_ceiling(node, input) > SYNC_CARRY_ROOM)
}

/// How many heads `verify_sync`'s WIDE carry admits an input may
/// still hold at a step boundary — the carried-in half of that step's supply.
///
/// A free function rather than a closure over the carry map because the
/// carry-forward loop both reads it and rewrites the map in one iteration.
fn carried_heads(wide_carry: &BTreeMap<String, Vec<u64>>, input: &str) -> u32 {
    wide_carry
        .get(input)
        .map_or(0, |v| u32::try_from(v.len()).unwrap_or(u32::MAX))
}

fn verify_fifo(
    node: &NodeDecl,
    steps: &[RecordedStep],
    trigger_inputs: &[String],
    block_gated: bool,
    trust: RoleTrust,
    report: &mut RederivationReport,
) {
    if block_gated {
        // A block defer suppresses the fire but the boundary drain has already
        // popped, so pops legitimately exceed fires on a credit-paced node.
        report.stand_downs.push(StandDown {
            node_id: node.node_id.clone(),
            decider: Decider::FifoPop,
            reason: StandDownReason::BlockGated,
        });
        return;
    }

    for step in steps {
        let mut popped = 0u32;
        for read in step
            .reads
            .iter()
            .filter(|r| r.node_id == node.node_id && trigger_inputs.contains(&r.input))
        {
            // A `(kind, role)` pair no mint site
            // can produce is evidence about the RECORDING, not about Sync — so
            // it is asked here too, on the same predicate, rather than only
            // where the Sync verdict happens to ask it. A FIFO pop count
            // derived from a stream carrying one would be arithmetic over
            // records the recorder cannot have written.
            if unproducible_read_shape(read, trust) {
                report.stand_downs.push(StandDown {
                    node_id: node.node_id.clone(),
                    decider: Decider::FifoPop,
                    reason: StandDownReason::ImpossibleReadShape,
                });
                return;
            }
            match read.kind {
                ReadOutcomeKind::Truncated => {
                    report.stand_downs.push(StandDown {
                        node_id: node.node_id.clone(),
                        decider: Decider::FifoPop,
                        reason: StandDownReason::TruncatedReadLog,
                    });
                    return;
                }
                ReadOutcomeKind::Decimated => {
                    report.stand_downs.push(StandDown {
                        node_id: node.node_id.clone(),
                        decider: Decider::FifoPop,
                        reason: StandDownReason::DecimatedRead,
                    });
                    return;
                }
                // Per-message FIFO: one pop, one fire. `Served` / `Held` are
                // body reads of an already-popped frame, so counting them would
                // double-count the fire they belong to — and on a ROLE-STAMPED
                // bag so is a BODY `DrainedBatch`, which the role branch
                // excludes exactly.
                ReadOutcomeKind::DrainedBatch if is_drain_site_read(read, trust) => {
                    popped = popped.saturating_add(read.popped)
                }
                _ => {}
            }
        }

        let recorded = recorded_fires(step, &node.node_id, false);
        if recorded.count != popped {
            report.findings.push(ScheduleFinding {
                node_id: node.node_id.clone(),
                step: step.step,
                decider: Decider::FifoPop,
                expected: StepFires::count(popped),
                recorded,
            });
            return;
        }
    }
}

/// Which records on a `block` edge are pops of the subscriber the credit
/// mirror is wired to.
///
/// The wire names the SITE; only the DRAIN DISCIPLINE maps a site to a PORT,
/// and the bag does not record it. But for one shape the GRAPH settles it, and
/// that shape is the primary one this verifier exists for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CreditBasis {
    /// The consumer input is a DATA-trigger input, so the discipline is
    /// SEPARATE **by construction** and only BODY-role pops touch the mirror.
    ///
    /// Two structural facts, neither of them a heuristic:
    ///
    /// 1. `NodeInfo::unifies_data_trigger` (`graph/node.rs:777-788`) unifies a
    ///    data trigger onto the body subscriber ONLY when its backpressure is
    ///    `DropOldest`. A `block` data trigger therefore NEVER unifies, so it
    ///    always gets the dedicated trigger-drain subscriber built at
    ///    `graph/runtime.rs:5540` — which receives a read-outcome STAGE but no
    ///    `register_block_probe` (that call is at `:5168`, on the node's own
    ///    body subscriber). Its pops are real pops of a real port, and that
    ///    port's queue is not what gates the producer.
    /// 2. The `CERULION_DRAIN_DISCIPLINE=separate` seam only ever forces
    ///    Unified BACK to Separate (`graph/runtime.rs:5355`), so it cannot move
    ///    a block data trigger that is already Separate. The env knob is
    ///    invisible in the bag, and here it does not need to be.
    ///
    /// So the Drain-role records on such an edge are evidence about the
    /// UNPROBED port and are ignored — not because they are untrustworthy, but
    /// because they are not about the quantity being re-derived.
    BodyOnly,
    /// Every pop-bearing record on this edge is a pop of the ONE probed
    /// subscriber, whatever role it carries.
    ///
    /// The non-trigger case is structural: a `block` input is excluded from the
    /// step-boundary snapshot (`graph/runtime.rs:11440`), it has no
    /// trigger-drain subscriber at all, and the node body reads it live through
    /// `try_view` on the probed sub. A `Period` / `External` node's input is the
    /// same shape. The per-set Sync case is the unification at
    /// `graph/runtime.rs:5683`: the boundary drain runs on the BODY subscriber,
    /// so its Drain-role pops ARE mirror decrements, and a second `try_view` in
    /// one tick adds a Body-role pop of the same port.
    AllPops,
    /// The graph does not settle it and the record stream does not either.
    Undecidable,
}

/// Which [`CreditBasis`] this edge takes, from the consumer's DECLARED trigger.
///
/// # The Sync fork is genuinely undecidable, and this is where that is said
///
/// A Sync trigger input is per-set (UNIFIED onto the probed body subscriber,
/// `graph/runtime.rs:5683`) or LEGACY (its own dedicated, UNPROBED drain
/// subscriber, `:5791`), and `per_set_capable` (`:5587`) is
/// `supports_sync_head_ops && unifies_trigger_drain && !force_separate_discipline`.
/// None of those three terms is recoverable from a bag:
///
/// * `supports_sync_head_ops` is SYMBOL PRESENCE on the loaded entry
///   (`graph/node.rs:5617` — `self.sync_head_op_ffi.is_some()`), and that export
///   is **additive with no ABI bump** (`graph/node.rs:3917`). A cdylib without
///   it is WARNED, never refused (`graph/runtime.rs:5590-5599`), so a
///   `trace_format` 5 bag really can carry a legacy-Sync node. `ClosureNodeEntry`
///   returns `false` outright (`graph/node.rs:2950`).
/// * `force_separate_discipline` is an env knob that is not in the manifest.
///
/// Summing both roles is exact under per-set and DOUBLE-COUNTS under legacy —
/// iceoryx2 delivers each frame to both ports — which can push `drained` past
/// `published` and manufacture a false UNDER-run. Counting body-only is exact
/// under legacy and drops the boundary drain (most of the credit) under
/// per-set, manufacturing a false OVER-run. Both are reachable, so a Sync edge
/// whose stream shows pops under BOTH roles is [`CreditBasis::Undecidable`].
///
/// It costs little in practice: a per-set Sync body read is served from the
/// FROZEN slot and records NOTHING, so the ordinary per-set stream is
/// drain-only and is still counted exactly. Only the second-`try_view`-in-one-
/// tick shape declines.
fn credit_basis(nodes: &[NodeDecl], edge: &BlockEdgeObservation) -> CreditBasis {
    let Some(node) = nodes.iter().find(|n| n.node_id == edge.consumer_node) else {
        // No declaration ⇒ no discipline ⇒ no basis. Guessing here would be a
        // guess about which PORT the pops came from.
        return CreditBasis::Undecidable;
    };
    match &node.trigger {
        TriggerDecl::Data { trigger_inputs } if trigger_inputs.contains(&edge.consumer_input) => {
            CreditBasis::BodyOnly
        }
        TriggerDecl::Sync { trigger_inputs, .. }
            if trigger_inputs.contains(&edge.consumer_input) =>
        {
            CreditBasis::Undecidable
        }
        // A non-trigger input of ANY node, and every input of a Period /
        // External / Unmodelled node: one probed subscriber, no drain sub.
        _ => CreditBasis::AllPops,
    }
}

/// Re-derive each cross-rank `block` edge's credit conservation from the read
/// log, and report an edge whose recorded numbers no legal interleave explains.
///
/// # `drained` is a count of pops on ONE subscriber — the PROBED one
///
/// The runtime enforces `block` against `BlockDeferEdge::outstanding`
/// (`graph/runtime.rs:958-1010`): the publisher increments it once per send
/// (`register_block_outstanding`, `:5005`) and the consumer decrements it in
/// `record_block_drained` (`transport/subscriber.rs:1879`), reached from
/// `drain_samples` (`:2410`, EVERY role) and from the `try_view` live arm's
/// `drain_to_latest_with_accounting` (`:2847`, which mints `Served` /
/// `Decimated` / `NoFrame` carrying a REAL pop count). All of them run on the
/// single subscriber `register_block_probe` was installed on
/// (`graph/runtime.rs:5168`) — the node's own BODY subscriber. Re-deriving that
/// quantity therefore means summing the pops taken on THAT port, whatever KIND
/// or ROLE the record wears:
///
/// * a body `try_receive` (the accumulate-all pattern) pops it under `Body`;
/// * so does a second `try_view` in one tick, whose live arm records `Served`;
/// * and a UNIFIED per-set Sync boundary drain pops it under `Drain`, because
///   `drain_for_trigger` runs on the body subscriber (`graph/runtime.rs:5683`).
///
/// Restricting the sum to `DrainedBatch` — the pre-roles rule — loses the first
/// two: a non-trigger `block` input is excluded from the step-boundary
/// snapshot (`graph/runtime.rs:11440`), so it is read live by the body and
/// records only `Served`/`NoFrame`, and such an edge re-derives `drained == 0`
/// and reports an OVER-run the moment `published` passes `depth`. Restricting
/// it further to the DRAIN role drops the accumulate-all case too. So every
/// pop-bearing record counts.
///
/// `Producer` is excluded because its `popped` is not one — a producer
/// annotation puts the full 64-bit token in the aux slot, so the low half the
/// popped field reads is part of that token. `Truncated` is excluded because
/// its `popped` is the count of records DROPPED in a merge window.
///
/// # Which records those are is settled by the GRAPH, not by the wire
///
/// The wire names the SITE, not the PORT, and only the DRAIN DISCIPLINE maps
/// one to the other. The bag does not record the discipline — but it does carry
/// the graph, and for the shape this verifier primarily exists for the graph
/// settles it outright. See [`CreditBasis`] and [`credit_basis`]: a `block`
/// DATA-trigger input is SEPARATE by construction (`graph/node.rs:777-788`
/// unifies only `DropOldest`), so only BODY-role pops touch the mirror; a
/// non-trigger input and a `Period`/`External` node's input have no drain
/// subscriber at all, so every pop is the probed port's. Only the Sync fork is
/// genuinely undecidable, and only when its stream carries pops under BOTH
/// roles — that edge declines as
/// [`StandDownReason::UnattributableCredit`] rather than picking a reading.
///
/// This is the difference between a verifier that reports and one that
/// declines: a rule that declines EVERY mixed stream declines — by fact 1 of
/// [`CreditBasis::BodyOnly`] — every cross-rank `block` data-trigger edge
/// there is.
///
/// # The archived arm is the pre-roles rule, verbatim
///
/// On [`RoleTrust::Declined`] nothing above applies: the sum is `DrainedBatch`
/// popped and only that, exactly as before roles existed, so every archived bag
/// re-derives the number it always did. That arm keeps the two imprecisions
/// named above — a `Served`-only edge reads `drained == 0`, and under SEPARATE
/// the count is the UNPROBED port's — and they are left alone deliberately:
/// they cannot be fixed without the port evidence the bag lacks, and an
/// archived bag's numbers must never change.
fn verify_conservation(
    nodes: &[NodeDecl],
    steps: &[RecordedStep],
    block_edges: &[BlockEdgeObservation],
    trust: RoleTrust,
    report: &mut RederivationReport,
) {
    for edge in block_edges {
        // The believed arm's two per-SITE sums, kept apart so their
        // co-occurrence is visible; and the archived arm's pre-roles sum.
        let (mut drain_pops, mut body_pops, mut kind_pops) = (0u64, 0u64, 0u64);
        let mut truncated = false;
        let mut corrupt = false;
        let mut unattributable_role = false;
        for step in steps {
            for read in step
                .reads
                .iter()
                .filter(|r| r.node_id == edge.consumer_node && r.input == edge.consumer_input)
            {
                // A `(kind, role)` pair no mint site can produce is evidence
                // about the RECORDING, and this rule is about to do ARITHMETIC
                // on those very bits — a believed-`Drain` `Served` would be
                // summed as a drain pop. Asked per EDGE because conservation is
                // keyed on an edge, not on the node sweep's scope.
                if unproducible_read_shape(read, trust) {
                    corrupt = true;
                    continue;
                }
                match read.kind {
                    ReadOutcomeKind::Truncated => truncated = true,
                    // Its `popped` slot is part of the producer token.
                    ReadOutcomeKind::Producer => {}
                    ReadOutcomeKind::Served
                    | ReadOutcomeKind::Held
                    | ReadOutcomeKind::NoFrame
                    | ReadOutcomeKind::DrainedBatch
                    | ReadOutcomeKind::Decimated => {
                        if !trust.believed() {
                            if read.kind == ReadOutcomeKind::DrainedBatch {
                                kind_pops = kind_pops.saturating_add(u64::from(read.popped));
                            }
                        } else if read.popped > 0 {
                            // A record that popped NOTHING moved no credit, so
                            // it belongs to neither sum and — the reason this
                            // guard is here rather than on the match — an
                            // unattributable one cannot skew a total it does
                            // not enter. A `NoFrame` under a role nothing named
                            // must not stand a healthy edge down.
                            match read.role {
                                // The format-5 peek/head mark: a `Peek` pop is the
                                // SCHEDULER's, on the SAME port a `Drain` pop
                                // takes — the per-set matcher's R-pop runs on
                                // whichever subscriber `drain_for_trigger` does
                                // — so the two sum together. What separates
                                // them is which record NAMES THE HEAD, which is
                                // `verify_sync`'s question and not this one:
                                // here the quantity is pops on a port.
                                ReadSiteRole::Drain | ReadSiteRole::Peek => {
                                    drain_pops = drain_pops.saturating_add(u64::from(read.popped));
                                }
                                ReadSiteRole::Body => {
                                    body_pops = body_pops.saturating_add(u64::from(read.popped));
                                }
                                // An unwritten `0` on a stamped bag: the wire
                                // named no site, so this pop belongs to no port.
                                ReadSiteRole::Unstamped => unattributable_role = true,
                            }
                        }
                    }
                }
            }
        }

        // Precedence: what is known about the RECORDING outranks what is known
        // about the READ LOG, which outranks what is known about the SITES —
        // each answer is strictly stronger evidence than the next, and a
        // corrupt stream makes the other two questions moot.
        if corrupt {
            report.stand_downs.push(StandDown {
                node_id: edge.consumer_node.clone(),
                decider: Decider::BlockCredit,
                reason: StandDownReason::ImpossibleReadShape,
            });
            continue;
        }

        if truncated {
            // Every sum above is a FLOOR under truncation, and a floor on
            // `drained` makes `published - drained` a CEILING — so it can only
            // manufacture a false OVER-run, never an under-run. Declining is
            // the correct answer either way.
            report.stand_downs.push(StandDown {
                node_id: edge.consumer_node.clone(),
                decider: Decider::BlockCredit,
                reason: StandDownReason::TruncatedReadLog,
            });
            continue;
        }

        let basis = credit_basis(nodes, edge);
        // A role the wire did not NAME cannot be placed on a port by anything —
        // not by the graph, not by the stream. It is the one decline the
        // structural rule below cannot absorb, and it is only ever reachable on
        // a believed bag (an unwritten `0`).
        let role_unnamed = trust.believed() && unattributable_role;
        // …and the Sync fork, which the graph settles for every OTHER shape.
        // Only a stream that actually carries pops under BOTH roles is
        // ambiguous: one role alone is the same number under either discipline.
        let discipline_unknown = trust.believed()
            && basis == CreditBasis::Undecidable
            && drain_pops > 0
            && body_pops > 0;
        if role_unnamed || discipline_unknown {
            report.stand_downs.push(StandDown {
                node_id: edge.consumer_node.clone(),
                decider: Decider::BlockCredit,
                reason: StandDownReason::UnattributableCredit,
            });
            continue;
        }

        let drained = if trust.believed() {
            match basis {
                // The Drain-role records are the UNPROBED trigger-drain sub's;
                // they are real pops of a real port, and not of the one the
                // mirror counts.
                CreditBasis::BodyOnly => body_pops,
                // One probed port, so every pop on it is a decrement — or an
                // undecidable edge whose stream carried only ONE role, where
                // both readings give this same number.
                CreditBasis::AllPops | CreditBasis::Undecidable => {
                    drain_pops.saturating_add(body_pops)
                }
            }
        } else {
            kind_pops
        };

        if let Some(breach) = conservation_breach(edge.published_frames, drained, edge.depth) {
            report.conservation.push(ConservationFinding {
                topic: edge.topic.clone(),
                consumer_node: edge.consumer_node.clone(),
                consumer_input: edge.consumer_input.clone(),
                depth: edge.depth,
                published: edge.published_frames,
                drained,
                breach,
            });
        }
    }
}

// ---------------------------------------------------------------------------
// Oracle tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1_000_000;
    /// A wall-faithful free-run epoch — the shape a per-rank controlled clock
    /// really carries (the `real_ns()` init), not a from-zero toy.
    const EPOCH: u64 = 1_700_000_000_000_000_000;

    /// One hand-written Period oracle row: `(next_fire, now, interval, cap)`
    /// against `(fire_count, first_fire, next_fire)`.
    type PeriodOracleRow = ((u64, u64, u64, u32), (u32, u64, u64));
    /// One hand-written throttle oracle row:
    /// `(prior_fires, now, last_fire, throttle)` against "defers?".
    type ThrottleOracleRow = ((u64, u64, u64, u64), bool);
    /// One hand-written conservation oracle row: `(published, drained, depth)`
    /// against the breach, if any.
    type ConservationOracleRow = ((u64, u64, u32), Option<ConservationBreach>);

    fn node(id: &str, trigger: TriggerDecl) -> NodeDecl {
        NodeDecl {
            node_id: id.to_string(),
            trigger,
            throttle_ns: None,
            // The NARROW default. Every oracle below that does not
            // specifically drive the carry widening declares a node whose
            // every wired input is trigger-marked, which is exactly the shape
            // in which a fire provably CONSUMES its heads — so their numbers
            // mean what they always meant.
            has_non_trigger_inputs: false,
            trigger_input_capacities: BTreeMap::new(),
        }
    }

    fn period(interval_ns: u64, max_catchup: Option<u32>) -> TriggerDecl {
        TriggerDecl::Period {
            interval_ns,
            max_catchup,
            catchup_cap_override: None,
        }
    }

    fn fire(id: &str, at: u64) -> RecordedFire {
        RecordedFire {
            node_id: id.to_string(),
            fire_time_ns: at,
        }
    }

    fn step(step: u64, clock_ns: u64, fires: Vec<RecordedFire>) -> RecordedStep {
        RecordedStep {
            step,
            clock_ns,
            fires,
            reads: Vec::new(),
        }
    }

    fn read(
        node_id: &str,
        input: &str,
        kind: ReadOutcomeKind,
        popped: u32,
        stamp: Option<u64>,
    ) -> RecordedRead {
        RecordedRead {
            node_id: node_id.to_string(),
            input: input.to_string(),
            kind,
            served_seq: Some(1),
            popped,
            wire_stamp_ns: stamp,
            // The pre-roles default. Every oracle below drives
            // `verify` with `roles_stamped = false`, i.e. the archived-bag arm,
            // so these vectors keep meaning exactly what they meant. The
            // ROLE-STAMPED arms build their reads through `read_with_role`.
            role: ReadSiteRole::Unstamped,
        }
    }

    /// A recorded read carrying a REAL read-site role — the
    /// stamped-bag half of every oracle that has one.
    fn read_with_role(
        node_id: &str,
        input: &str,
        kind: ReadOutcomeKind,
        popped: u32,
        stamp: Option<u64>,
        role: ReadSiteRole,
    ) -> RecordedRead {
        RecordedRead {
            role,
            ..read(node_id, input, kind, popped, stamp)
        }
    }

    fn inputs(names: &[&str]) -> Vec<String> {
        names.iter().map(|s| s.to_string()).collect()
    }

    /// [`verify`] for a FROM-START pass, the shape every from-start oracle
    /// below was written for, restored nothing (`PassRestore::none()`), so
    /// each of them reads exactly as it did before the gate came back.
    fn verify_unrestored(
        nodes: &[NodeDecl],
        steps: &[RecordedStep],
        block_edges: &[BlockEdgeObservation],
        trust: RoleTrust,
    ) -> RederivationReport {
        verify(nodes, steps, block_edges, trust, &PassRestore::none())
    }

    // -- the PassRestore seed ------------------------------------------------

    /// A Sync node with ONE restored head and the fusion shape the resume
    /// arms replay: `cam` arrived before the recording's first retained step
    /// (so its head is a restored stamp with NO kind-6 record), `lidar`
    /// arrives on the first resumed step, and the recording says the node
    /// fired there. `(cam, lidar)` is the node's declared trigger pair.
    fn restored_cam_at(stamp: u64) -> PassRestore {
        let mut per_input = BTreeMap::new();
        per_input.insert("cam".to_string(), stamp);
        let mut heads = BTreeMap::new();
        heads.insert("fuse".to_string(), per_input);
        PassRestore::from_sync_heads(heads)
    }

    fn fuse_node() -> NodeDecl {
        node(
            "fuse",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        )
    }

    fn drain_at(input: &str, at: u64) -> RecordedRead {
        read_with_role(
            "fuse",
            input,
            ReadOutcomeKind::DrainedBatch,
            1,
            Some(EPOCH + at),
            ReadSiteRole::Drain,
        )
    }

    /// The first resumed step: the recorded fire the restored head explains.
    fn first_resumed_step_with_lidar_at(at: u64) -> Vec<RecordedStep> {
        vec![RecordedStep {
            step: 7,
            clock_ns: EPOCH + 5 * MS,
            fires: vec![fire("fuse", EPOCH + 5 * MS)],
            reads: vec![drain_at("lidar", at)],
        }]
    }

    #[test]
    fn a_restored_sync_head_seeds_the_first_resumed_steps_transversal() {
        // THE gate, and its mutant, in one body. With the seed, `cam`'s
        // restored 1000 ms head and `lidar`'s recorded 1010 ms arrival form an
        // in-window transversal, and the recorded fire is exactly what the
        // scheduler did: CLEAN. Without it (a from-start pass, or the seed
        // dropped), `cam` has no candidate at all, arm B finds no transversal
        // for a step that fired, and a healthy resume is CONVICTED on its
        // first step: the fold over a map missing its members that the
        // `PassRestore` doc names.
        let n = fuse_node();
        let steps = first_resumed_step_with_lidar_at(1010 * MS);

        let restored = verify(
            std::slice::from_ref(&n),
            &steps,
            &[],
            RoleTrust::Believed,
            &restored_cam_at(EPOCH + 1000 * MS),
        );
        assert!(
            restored.is_clean(),
            "the restored head is a candidate and a unit of supply: {:?}",
            restored.findings
        );
        assert!(
            restored.stand_downs.is_empty(),
            "…and nothing stood down: {:?}",
            restored.stand_downs
        );

        let bare = verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Believed);
        assert_eq!(bare.findings.len(), 1, "{:?}", bare.findings);
        let f = &bare.findings[0];
        assert_eq!(f.node_id, "fuse");
        assert_eq!(f.step, 7);
        assert_eq!(f.decider, Decider::Sync);
        assert_eq!(
            f.expected,
            StepFires::none(),
            "arm B: no transversal exists without the restored head"
        );
    }

    #[test]
    fn a_restored_sync_head_is_the_g4_floor_for_the_stamps_that_follow() {
        // A restored head names a stamp the publisher's epoch had reached, so a
        // later head-naming stamp BELOW it is the same epoch reset guard g4
        // catches between two recorded heads. With the seed the regression is
        // seen and the node stands down; without it the 900 ms stamp is the
        // first floor this rule ever saw, `(900, 905)` align, and the run
        // reads clean: the difference IS the seeded floor.
        let n = fuse_node();
        let steps = vec![RecordedStep {
            step: 7,
            clock_ns: EPOCH + 5 * MS,
            fires: vec![fire("fuse", EPOCH + 5 * MS)],
            reads: vec![drain_at("cam", 900 * MS), drain_at("lidar", 905 * MS)],
        }];

        let restored = verify(
            std::slice::from_ref(&n),
            &steps,
            &[],
            RoleTrust::Believed,
            &restored_cam_at(EPOCH + 1000 * MS),
        );
        assert_eq!(
            restored.stand_downs,
            vec![StandDown {
                node_id: "fuse".to_string(),
                decider: Decider::Sync,
                reason: StandDownReason::SyncHeadStampRegressed,
            }]
        );
        assert!(restored.is_clean(), "a stand-down is not a finding");

        let bare = verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Believed);
        assert!(bare.is_clean() && bare.stand_downs.is_empty(), "{bare:?}");
    }

    #[test]
    fn a_restore_naming_another_node_or_a_non_trigger_input_seeds_nothing() {
        // The seed is keyed by NODE and folded over the node's TRIGGER inputs
        // only: a restore that names a different node, or an input the
        // matcher does not align, leaves the verdict exactly the unrestored
        // one: the same arm-B finding on the same step.
        let n = fuse_node();
        let steps = first_resumed_step_with_lidar_at(1010 * MS);
        let bare = verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Believed);
        assert_eq!(bare.findings.len(), 1);

        let mut other_node = BTreeMap::new();
        other_node.insert("other".to_string(), {
            let mut m = BTreeMap::new();
            m.insert("cam".to_string(), EPOCH + 1000 * MS);
            m
        });
        let mut stray_input = BTreeMap::new();
        stray_input.insert("fuse".to_string(), {
            let mut m = BTreeMap::new();
            m.insert("ctx".to_string(), EPOCH + 1000 * MS);
            m
        });
        for (label, restore) in [
            ("another node", PassRestore::from_sync_heads(other_node)),
            (
                "a non-trigger input",
                PassRestore::from_sync_heads(stray_input),
            ),
            ("nothing", PassRestore::none()),
        ] {
            let report = verify(
                std::slice::from_ref(&n),
                &steps,
                &[],
                RoleTrust::Believed,
                &restore,
            );
            assert_eq!(
                report.findings, bare.findings,
                "a restore naming {label} must change nothing"
            );
        }
        assert_eq!(PassRestore::none(), PassRestore::default());
    }

    // -- Period ------------------------------------------------------------

    /// The MIRROR and the whole ENGINE, driven against ONE hand-written oracle
    /// in one body — the two-copies discipline made checkable: a mirror that
    /// drifted from what the engine actually applies would fail here even if
    /// each half agreed with itself.
    ///
    /// The oracle is hand-computed from the rule
    /// `crates/cerulion_core/src/scheduler/mod.rs`'s Period arm states, never from
    /// this module's own arithmetic.
    #[test]
    fn the_period_mirror_and_the_engine_answer_one_hand_oracle() {
        // (next_fire, now, interval, cap) -> (fire_count, first_fire, next_fire)
        let oracle: &[PeriodOracleRow] = &[
            // Deadline still ahead ⇒ no fire, deadline untouched.
            ((100, 90, 10, u32::MAX), (0, 0, 100)),
            // Exactly due ⇒ one fire, deadline advances one interval.
            ((100, 100, 10, u32::MAX), (1, 100, 110)),
            // 25 ns behind a 10 ns interval ⇒ 3 whole intervals past `now`.
            ((100, 125, 10, u32::MAX), (3, 100, 130)),
            // Same gap, capped at 2 ⇒ 2 fires, but the deadline STILL skips
            // ahead past `now` (the live arm's `>= cap` skip-ahead).
            ((100, 125, 10, 2), (2, 100, 130)),
            // cap == 0 ⇒ zero fires AND the skip-ahead still runs (the `>=`
            // is satisfied at zero, which a `>` version of that comparison
            // would miss). Two SCHEDULER call sites now depend on exactly this
            // reading — `reset_node` and `restore_period_schedule`'s forward arm
            // both advance a deadline with no fires — so it is a production
            // shape, not only a boundary this oracle happens to cover.
            ((100, 125, 10, 0), (0, 100, 130)),
        ];
        for &((next_fire, now, interval, cap), (fc, ff, nf)) in oracle {
            let got = period_step(next_fire, now, interval, cap);
            assert_eq!(
                got,
                PeriodStep {
                    fire_count: fc,
                    first_fire_ns: if fc == 0 && next_fire > now { 0 } else { ff },
                    next_fire_ns: nf,
                },
                "period_step({next_fire}, {now}, {interval}, {cap})"
            );
        }

        // The SAME rule through the engine: a 10 ms node on a 10 ms step grid
        // fires once per step, and the engine must agree with the mirror.
        let nodes = vec![node("ticker", period(10 * MS, None))];
        let steps = vec![
            step(0, EPOCH, vec![fire("ticker", EPOCH)]),
            step(1, EPOCH + 10 * MS, vec![fire("ticker", EPOCH + 10 * MS)]),
            step(2, EPOCH + 20 * MS, vec![fire("ticker", EPOCH + 20 * MS)]),
        ];
        let report = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert!(report.is_clean(), "unexpected: {:?}", report.findings);
        assert_eq!(report.divergence_class(), None);
        assert!(report.stand_downs.is_empty());
    }

    /// A catch-up burst: the step clock jumps 3 intervals, so the recording
    /// carries 3 fires at the 3 interval deadlines — NOT 3 fires at the step
    /// clock. An implementation that stamped the step clock onto each fire
    /// fails on the instants.
    #[test]
    fn a_period_catch_up_burst_matches_its_interval_deadlines() {
        let nodes = vec![node("ticker", period(10 * MS, None))];
        let steps = vec![
            step(0, EPOCH, vec![fire("ticker", EPOCH)]),
            step(
                1,
                EPOCH + 30 * MS,
                vec![
                    fire("ticker", EPOCH + 10 * MS),
                    fire("ticker", EPOCH + 20 * MS),
                    fire("ticker", EPOCH + 30 * MS),
                ],
            ),
        ];
        let report = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert!(report.is_clean(), "unexpected: {:?}", report.findings);
    }

    /// THE anti-tautology arm for every clean Period test above: a fire that
    /// lands one step EARLY is caught, and the finding names node / step /
    /// decider / expected-vs-recorded.
    ///
    /// The grid is deliberately 33 ms, not 1 ms or 30 ms: `WALL_EPOCH_NS`
    /// divides both of those, so a phase-preserving and a phase-DISCARDING
    /// implementation coincide on them and the arm would pass either way.
    #[test]
    fn a_period_fire_one_step_early_is_caught_and_named() {
        let interval = 33 * MS;
        let nodes = vec![node("scanner", period(interval, None))];
        let steps = vec![
            step(0, EPOCH, vec![fire("scanner", EPOCH)]),
            // Step 1's clock has not reached the next deadline, yet the
            // recording fires: one step early.
            step(1, EPOCH + 20 * MS, vec![fire("scanner", EPOCH + interval)]),
            step(2, EPOCH + 40 * MS, vec![fire("scanner", EPOCH + interval)]),
        ];
        let report = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert!(!report.is_clean());
        assert_eq!(
            report.divergence_class(),
            Some(DivergenceClass::FireSchedule)
        );
        assert_eq!(report.findings.len(), 1, "one finding per (node, decider)");
        let f = &report.findings[0];
        assert_eq!(f.node_id, "scanner");
        assert_eq!(f.step, 1);
        assert_eq!(f.decider, Decider::Period);
        assert_eq!(f.expected, StepFires::none());
        assert_eq!(f.recorded, StepFires::at(vec![EPOCH + interval]));
        assert_eq!(f.divergence_class(), DivergenceClass::FireSchedule);
        // The rendering carries the ONE vocabulary, never a bare exit code.
        assert!(f.to_string().contains("fire-schedule divergence"), "{f}");
    }

    /// A Period node the recording never fired has no phase to anchor on, and
    /// says so rather than reporting "clean".
    #[test]
    fn a_period_node_that_never_fired_stands_down_instead_of_reporting_clean() {
        let nodes = vec![node("silent", period(10 * MS, None))];
        let steps = vec![step(0, EPOCH, vec![]), step(1, EPOCH + 10 * MS, vec![])];
        let report = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert!(report.is_clean());
        assert_eq!(
            report.stand_downs,
            vec![StandDown {
                node_id: "silent".to_string(),
                decider: Decider::Period,
                reason: StandDownReason::NoPhaseAnchor,
            }]
        );
    }

    /// A zero interval is refused rather than looped on — the live arm's own
    /// `while *next_fire <= now { *next_fire += 0 }` would never terminate.
    #[test]
    fn a_zero_interval_period_node_stands_down_rather_than_looping() {
        let nodes = vec![node("broken", period(0, None))];
        let steps = vec![step(0, EPOCH, vec![fire("broken", EPOCH)])];
        let report = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert!(report.is_clean());
        assert_eq!(report.stand_downs[0].reason, StandDownReason::ZeroInterval);
    }

    // -- The block fence ---------------------------------------------------

    /// **The block-fence arm.** A block-paced Period producer's fires are gated
    /// by cross-rank occupancy the log records nothing about, so its schedule
    /// STANDS DOWN — it does not diverge.
    ///
    /// The trace here is the real shape: step 2's deadline passed and the
    /// recording carries no fire, because the credit word was exhausted.
    /// Feeding the block edge into re-derivation (i.e. deleting the
    /// `block_gated` stand-down) turns exactly that into a spurious
    /// fire-schedule divergence, which is the failure the decision exists to
    /// prevent.
    #[test]
    fn a_block_paced_period_producer_stands_down_instead_of_diverging() {
        let nodes = vec![node("producer", period(10 * MS, None))];
        let steps = vec![
            step(0, EPOCH, vec![fire("producer", EPOCH)]),
            step(1, EPOCH + 10 * MS, vec![fire("producer", EPOCH + 10 * MS)]),
            // Deadline EPOCH+20ms has passed; credit was exhausted, so no fire.
            step(2, EPOCH + 20 * MS, vec![]),
            step(3, EPOCH + 30 * MS, vec![]),
        ];
        let edges = vec![BlockEdgeObservation {
            topic: "/blk".to_string(),
            producer_node: "producer".to_string(),
            consumer_node: "sink".to_string(),
            consumer_input: "inp".to_string(),
            depth: 4,
            published_frames: 2,
        }];
        let report = verify_unrestored(&nodes, &steps, &edges, RoleTrust::Declined);
        assert!(
            report.is_clean(),
            "a credit-paced producer must not diverge: {:?}",
            report.findings
        );
        assert_eq!(report.divergence_class(), None);
        assert!(report.stand_downs.contains(&StandDown {
            node_id: "producer".to_string(),
            decider: Decider::Period,
            reason: StandDownReason::BlockGated,
        }));
        // ANTI-TAUTOLOGY: the identical trace with NO block edge IS a
        // divergence, so the clean verdict above is the fence doing work and
        // not a fixture that could never diverge.
        let bare = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert!(!bare.is_clean());
        assert_eq!(bare.findings[0].step, 2);
    }

    // -- Throttle ----------------------------------------------------------

    /// The mirror and the engine against ONE hand oracle, with BOTH sides of
    /// the `<` boundary pinned: a gap EQUAL to the throttle is allowed, one
    /// nanosecond under it defers.
    #[test]
    fn the_throttle_mirror_and_the_engine_answer_one_hand_oracle() {
        // (prior_fires, now, last_fire, throttle) -> defers?
        let oracle: &[ThrottleOracleRow] = &[
            // Never fired ⇒ nothing to throttle against.
            ((0, 100, 0, 50), false),
            // Gap 49 < 50 ⇒ defer.
            ((1, 149, 100, 50), true),
            // Gap 50 is NOT < 50 ⇒ allowed. The `<`-vs-`<=` boundary.
            ((1, 150, 100, 50), false),
            // Gap 51 ⇒ allowed.
            ((1, 151, 100, 50), false),
            // A clock that ran backwards saturates to 0 ⇒ defer, never panic.
            ((1, 90, 100, 50), true),
        ];
        for &((prior, now, last, throttle), expect) in oracle {
            assert_eq!(
                throttle_defers(prior, now, last, throttle),
                expect,
                "throttle_defers({prior}, {now}, {last}, {throttle})"
            );
        }

        // The SAME boundary through the engine: fires exactly `throttle_ns`
        // apart are legal, so the report is clean.
        let mut n = node(
            "relay",
            TriggerDecl::Data {
                trigger_inputs: vec![],
            },
        );
        n.throttle_ns = Some(5 * MS);
        n.trigger = TriggerDecl::Unmodelled; // isolate the throttle rule
        let steps = vec![
            step(0, EPOCH, vec![fire("relay", EPOCH)]),
            step(1, EPOCH + 5 * MS, vec![fire("relay", EPOCH + 5 * MS)]),
            step(2, EPOCH + 10 * MS, vec![fire("relay", EPOCH + 10 * MS)]),
        ];
        let report = verify_unrestored(&[n], &steps, &[], RoleTrust::Declined);
        assert!(report.is_clean(), "unexpected: {:?}", report.findings);
    }

    /// A trace whose gaps genuinely respect the cap verifies clean, and the
    /// deferred steps (no fire) are not mistaken for a missing fire.
    #[test]
    fn a_throttled_trace_that_respects_its_cap_verifies_clean() {
        let mut n = node("relay", TriggerDecl::Unmodelled);
        n.throttle_ns = Some(20 * MS);
        let steps = vec![
            step(0, EPOCH, vec![fire("relay", EPOCH)]),
            step(1, EPOCH + 10 * MS, vec![]), // deferred by the throttle
            step(2, EPOCH + 20 * MS, vec![fire("relay", EPOCH + 20 * MS)]),
            step(3, EPOCH + 30 * MS, vec![]), // deferred again
        ];
        let report = verify_unrestored(&[n], &steps, &[], RoleTrust::Declined);
        assert!(report.is_clean(), "unexpected: {:?}", report.findings);
    }

    /// A fire INSIDE the throttle window is caught and named.
    #[test]
    fn a_fire_inside_the_throttle_window_is_caught_and_named() {
        let mut n = node("relay", TriggerDecl::Unmodelled);
        n.throttle_ns = Some(20 * MS);
        let steps = vec![
            step(0, EPOCH, vec![fire("relay", EPOCH)]),
            // 10 ms after the last fire, against a 20 ms cap.
            step(1, EPOCH + 10 * MS, vec![fire("relay", EPOCH + 10 * MS)]),
        ];
        let report = verify_unrestored(&[n], &steps, &[], RoleTrust::Declined);
        assert!(!report.is_clean());
        let f = &report.findings[0];
        assert_eq!(f.node_id, "relay");
        assert_eq!(f.step, 1);
        assert_eq!(f.decider, Decider::Throttle);
        assert_eq!(f.expected, StepFires::none());
        assert_eq!(f.recorded, StepFires::at(vec![EPOCH + 10 * MS]));
        assert_eq!(
            report.divergence_class(),
            Some(DivergenceClass::FireSchedule)
        );
    }

    // -- Sync --------------------------------------------------------------

    /// **The clock-domain arm.** Two trigger inputs stamped by TWO DIFFERENT
    /// producing ranks, 30 ms apart, against a 50 ms window — aligned. The
    /// consumer's own clock sits 500 ms above both, which is what separates the
    /// two implementations: the spread is `max(stamp) - min(stamp)` = 30 ms
    /// (aligned), while any rule reaching for the consumer's `now` — the
    /// cross-clock error — reads ~500 ms and calls a perfectly aligned pair
    /// misaligned.
    #[test]
    fn two_producer_clocks_align_on_their_own_stamps_not_the_consumers() {
        let cam_stamp = EPOCH + 100 * MS; // producing rank A
        let lidar_stamp = EPOCH + 130 * MS; // producing rank B, 30 ms apart
        let consumer_clock = EPOCH + 600 * MS; // ~500 ms above BOTH stamps

        // The mirror, against a hand oracle.
        let mut stamps = BTreeMap::new();
        stamps.insert("cam".to_string(), cam_stamp);
        stamps.insert("lidar".to_string(), lidar_stamp);
        assert!(
            sync_aligned(&stamps, &inputs(&["cam", "lidar"]), Some(50 * MS)),
            "a 30 ms spread is inside a 50 ms window"
        );
        // The discriminator, spelled out: the consumer-clock reading is not
        // merely different, it is over the window by an order of magnitude.
        assert!(consumer_clock - cam_stamp > 50 * MS);

        // The SAME case through the engine.
        let nodes = vec![node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        )];
        let steps = vec![RecordedStep {
            step: 0,
            clock_ns: consumer_clock,
            fires: vec![fire("fusion", consumer_clock)],
            reads: vec![
                read(
                    "fusion",
                    "cam",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(cam_stamp),
                ),
                read(
                    "fusion",
                    "lidar",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(lidar_stamp),
                ),
            ],
        }];
        let report = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert!(
            report.is_clean(),
            "the spread is over the stamps: {:?}",
            report.findings
        );
    }

    /// The window comparison pinned on BOTH sides: `<=` is inclusive, so a
    /// spread EQUAL to the window aligns and one nanosecond more does not.
    #[test]
    fn the_sync_window_boundary_is_inclusive_on_both_sides() {
        let base = EPOCH + 7 * MS;
        let window = 50 * MS;
        for (spread, expect_aligned) in [(window - 1, true), (window, true), (window + 1, false)] {
            let mut stamps = BTreeMap::new();
            stamps.insert("a".to_string(), base);
            stamps.insert("b".to_string(), base + spread);
            assert_eq!(
                sync_aligned(&stamps, &inputs(&["a", "b"]), Some(window)),
                expect_aligned,
                "spread {spread} against window {window}"
            );

            // And through the engine, on the same three spreads: a trace that
            // FIRES is clean iff the spread aligns.
            let nodes = vec![node(
                "fusion",
                TriggerDecl::Sync {
                    window_ns: Some(window),
                    trigger_inputs: inputs(&["a", "b"]),
                },
            )];
            let steps = vec![RecordedStep {
                step: 0,
                clock_ns: base + spread + MS,
                fires: vec![fire("fusion", base + spread + MS)],
                reads: vec![
                    read("fusion", "a", ReadOutcomeKind::DrainedBatch, 1, Some(base)),
                    read(
                        "fusion",
                        "b",
                        ReadOutcomeKind::DrainedBatch,
                        1,
                        Some(base + spread),
                    ),
                ],
            }];
            let report = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
            assert_eq!(
                report.is_clean(),
                expect_aligned,
                "engine verdict at spread {spread}"
            );
        }
    }

    /// A trace claiming a fire while the spread is OUTSIDE the window is caught
    /// and named — the archetype of an edited `sync_window_ms`.
    #[test]
    fn a_sync_fire_outside_the_window_is_caught_and_named() {
        let nodes = vec![node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        )];
        let steps = vec![RecordedStep {
            step: 4,
            clock_ns: EPOCH + 300 * MS,
            fires: vec![fire("fusion", EPOCH + 300 * MS)],
            reads: vec![
                read(
                    "fusion",
                    "cam",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH + 100 * MS),
                ),
                // 200 ms apart against a 50 ms window.
                read(
                    "fusion",
                    "lidar",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH + 300 * MS),
                ),
            ],
        }];
        let report = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert!(!report.is_clean());
        let f = &report.findings[0];
        assert_eq!(f.node_id, "fusion");
        assert_eq!(f.step, 4);
        assert_eq!(f.decider, Decider::Sync);
        assert_eq!(f.expected, StepFires::none());
        assert_eq!(f.recorded, StepFires::count(1));
        assert!(f.to_string().contains("sync alignment"), "{f}");
        assert_eq!(
            report.divergence_class(),
            Some(DivergenceClass::FireSchedule)
        );
    }

    /// `unbounded_sync` (`window_ns: None`) takes `sync_aligned`'s own
    /// `window_ns: None` arm (`Self::sync_aligned` delegates to
    /// `cerulion_core::scheduler::sync_aligned`, the shared rule): all inputs
    /// present is the whole condition, so a spread that a bounded window
    /// would refuse still fires — and a MISSING input still does not.
    #[test]
    fn unbounded_sync_fires_on_presence_alone_and_not_before() {
        let nodes = vec![node(
            "fused",
            TriggerDecl::Sync {
                window_ns: None,
                trigger_inputs: inputs(&["a", "b"]),
            },
        )];
        let steps = vec![
            // Step 0: only `a` has arrived ⇒ must NOT fire.
            RecordedStep {
                step: 0,
                clock_ns: EPOCH,
                fires: vec![],
                reads: vec![read(
                    "fused",
                    "a",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH),
                )],
            },
            // Step 1: `b` arrives a FULL SECOND later — no window to violate.
            RecordedStep {
                step: 1,
                clock_ns: EPOCH + 1_000 * MS,
                fires: vec![fire("fused", EPOCH + 1_000 * MS)],
                reads: vec![read(
                    "fused",
                    "b",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH + 1_000 * MS),
                )],
            },
        ];
        let report = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert!(report.is_clean(), "unexpected: {:?}", report.findings);

        // The same stamps under a BOUNDED 50 ms window are a divergence — the
        // anti-tautology half, proving the unbounded arm is not vacuous.
        let bounded = vec![node(
            "fused",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["a", "b"]),
            },
        )];
        assert!(!verify_unrestored(&bounded, &steps, &[], RoleTrust::Declined,).is_clean());
    }

    /// A Sync input is stamped by its DRAIN, never by the
    /// node body's own read of the same port.
    ///
    /// The wire carries no role bit, so this module is handed BOTH sites'
    /// records under one `(node, input)` and has to tell them apart by KIND —
    /// which is what the kind vocabulary is for. Under Sync the body reads a
    /// SEPARATE subscriber with its own cursor, so a frame landing between the
    /// drain and the tick gives the body read a NEWER stamp; folded into the
    /// alignment map it moves `max(stamp)` and pushes a genuinely aligned pair
    /// outside its window. The step it lands in is always a step the node
    /// FIRED (a body read happens inside a tick), so the damage is precisely a
    /// false "fired while misaligned".
    ///
    /// Restoring `ReadOutcomeKind::Served` to the arrival arm fails
    /// the first half with a `Decider::Sync` finding at step 0.
    #[test]
    fn a_sync_inputs_body_read_does_not_stamp_its_alignment() {
        let cam = EPOCH + 100 * MS;
        let lidar = EPOCH + 130 * MS; // 30 ms apart — inside a 50 ms window
                                      // The frame the tick's own `try_view` served, 300 ms above `lidar`:
                                      // read as an arrival it blows the window wide open.
        let body = EPOCH + 400 * MS;
        let clock = EPOCH + 500 * MS;

        let nodes = vec![node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        )];
        let drain_reads = vec![
            read("fusion", "cam", ReadOutcomeKind::DrainedBatch, 1, Some(cam)),
            read(
                "fusion",
                "lidar",
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(lidar),
            ),
        ];

        let mut with_body = drain_reads.clone();
        with_body.push(read(
            "fusion",
            "cam",
            ReadOutcomeKind::Served,
            1,
            Some(body),
        ));
        with_body.push(read(
            "fusion",
            "lidar",
            ReadOutcomeKind::Served,
            1,
            Some(lidar),
        ));
        let steps = vec![RecordedStep {
            step: 0,
            clock_ns: clock,
            fires: vec![fire("fusion", clock)],
            reads: with_body,
        }];
        let report = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert!(
            report.is_clean(),
            "a body read is not an arrival: {:?}",
            report.findings
        );

        // ANTI-TAUTOLOGY: the SAME stamp arriving at the DRAIN really is an
        // arrival and really does break the window — so the discriminator is
        // the record's KIND, not its value, and this arm is not passing because
        // the stamp was harmless.
        //
        // The drain carries it INSTEAD OF the earlier one rather than beside
        // it: one drain record per input per step is the only shape the sync
        // drain can produce, and two would be the ambiguous
        // shape, which is stood down rather than judged — so writing the arm
        // that way would prove nothing about the kind rule.
        let steps = vec![RecordedStep {
            step: 0,
            clock_ns: clock,
            fires: vec![fire("fusion", clock)],
            reads: vec![
                read(
                    "fusion",
                    "cam",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(body),
                ),
                read(
                    "fusion",
                    "lidar",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(lidar),
                ),
            ],
        }];
        let report = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert!(!report.is_clean());
        assert_eq!(report.findings[0].decider, Decider::Sync);
        assert!(
            report
                .stand_downs
                .iter()
                .all(|s| s.reason != StandDownReason::AmbiguousReadSite),
            "the arm must reach the alignment rule, not the site-ambiguity gate"
        );
    }

    /// ONE arrival per input supplies ONE set, so a step recording TWO fires on
    /// one arrival each is a divergence.
    ///
    /// # The claim, and what it is not
    ///
    /// This arm does NOT read "an ALIGNED Sync step fires EXACTLY ONCE", as if
    /// `Scheduler::evaluate_node` answered `FireKind::Single` and a
    /// `check_sync` cleared the stamp map on the fire. There is no `check_sync`
    /// and Sync does not produce `FireKind::Single` (it exists
    /// only for External): per-set Sync (the DEFAULT) answers with
    /// `FireKind::Sync { max_sets }` and `tick_sync_burst` fires once per
    /// COMPLETE SET, so "exactly once" is false on every healthy burst — and
    /// the discipline that would restore it is not decidable from a bag (there
    /// is no positive LEGACY witness on the wire).
    ///
    /// The claim is the SET-SUPPLY ceiling, and on THIS shape the two
    /// answers coincide exactly: `min_i supply(i)` over one arrival per input
    /// with no carry is 1, so every assertion below survives verbatim —
    /// `expected == count(1)`, `recorded == count(2)`, the suppressible half
    /// (a suppressor takes the count DOWN, so it explains a 0 and never a 2),
    /// and the one-fire anti-tautology.
    ///
    /// The THIRD block is what the ceiling buys: the SAME two fires with TWO
    /// records per input is CLEAN, because two arrivals supply two sets. That
    /// is the per-set burst, and an "exactly once" arm calls it exit 6.
    ///
    /// Deleting the supply ceiling makes the first two blocks read
    /// clean; a `count > 1 && aligned ⇒ expected 1` arm fails the third (and
    /// `a_per_set_burst_of_aligned_sets_is_not_a_schedule_divergence`).
    #[test]
    fn an_aligned_sync_step_that_fired_twice_is_caught() {
        let a = EPOCH + 10 * MS;
        let b = EPOCH + 20 * MS; // aligned inside a 50 ms window
        let clock = EPOCH + 30 * MS;
        let reads = vec![
            read("fusion", "a", ReadOutcomeKind::DrainedBatch, 1, Some(a)),
            read("fusion", "b", ReadOutcomeKind::DrainedBatch, 1, Some(b)),
        ];

        // Plain Sync: two fires where the rule allows one.
        let nodes = vec![node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["a", "b"]),
            },
        )];
        let steps = vec![RecordedStep {
            step: 3,
            clock_ns: clock,
            fires: vec![fire("fusion", clock), fire("fusion", clock)],
            reads: reads.clone(),
        }];
        let report = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert!(!report.is_clean());
        let f = &report.findings[0];
        assert_eq!(f.step, 3);
        assert_eq!(f.decider, Decider::Sync);
        assert_eq!(f.expected, StepFires::count(1));
        assert_eq!(f.recorded, StepFires::count(2));

        // A SUPPRESSIBLE Sync node is caught too: a suppressor can only take
        // the count DOWN to 0, so it explains a 0 and never a 2. Its
        // one-directional stand-down is still recorded.
        let throttled = vec![NodeDecl {
            throttle_ns: Some(5 * MS),
            ..node(
                "fusion",
                TriggerDecl::Sync {
                    window_ns: Some(50 * MS),
                    trigger_inputs: inputs(&["a", "b"]),
                },
            )
        }];
        let report = verify_unrestored(&throttled, &steps, &[], RoleTrust::Declined);
        assert!(
            report
                .findings
                .iter()
                .any(|f| f.decider == Decider::Sync && f.recorded == StepFires::count(2)),
            "suppression cannot explain a SECOND fire: {:?}",
            report.findings
        );

        // ANTI-TAUTOLOGY: the same aligned step firing ONCE is clean, so the
        // arm above pins the COUNT rather than refusing alignment outright.
        let once = vec![RecordedStep {
            step: 3,
            clock_ns: clock,
            fires: vec![fire("fusion", clock)],
            reads: reads.clone(),
        }];
        assert!(verify_unrestored(&nodes, &once, &[], RoleTrust::Declined).is_clean());

        // The SAME two fires with TWO arrivals per input is CLEAN.
        // Two sets were supplied, so `tick_sync_burst` firing twice is the
        // DEFAULT discipline working — not a divergence. This block is the one
        // a `count > 1 && aligned` arm would call exit 6.
        //
        // It runs on a ROLE-STAMPED bag deliberately: on an ARCHIVED one two
        // kind-inferred drain-site records for one `(node, input)` in one step
        // are the AMBIGUITY signature, so the node would not be JUDGED at all
        // and "clean" would be a claim about the site gate rather than about
        // the supply model (an unstamped bag that survives the gate
        // carries at most one drain-site record per input per step, which is
        // exactly the shape the first two blocks drive).
        let drain = |input: &str, stamp: u64| {
            read_with_role(
                "fusion",
                input,
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(stamp),
                ReadSiteRole::Drain,
            )
        };
        let burst = vec![RecordedStep {
            step: 3,
            clock_ns: clock,
            fires: vec![fire("fusion", clock), fire("fusion", clock)],
            reads: vec![
                drain("a", a),
                drain("b", b),
                drain("a", a + 5 * MS),
                drain("b", b + 5 * MS),
            ],
        }];
        let clean = verify_unrestored(&nodes, &burst, &[], RoleTrust::Believed);
        assert!(
            clean.is_clean(),
            "two arrivals per input SUPPLY two sets — the per-set burst: {clean:?}"
        );
        assert!(
            clean.stand_downs.is_empty(),
            "…and the node was really JUDGED, not stood down: {:?}",
            clean.stand_downs
        );
    }

    /// A trigger input carrying reads from BOTH of
    /// its read sites stands the node DOWN — never a guessed alignment.
    ///
    /// `DrainedBatch` is not exclusively a drain-site kind: `drain_samples`
    /// stages it, the sync drain reaches it via `try_receive_timestamps`, and a
    /// node body reaches the SAME function via the `pub`
    /// `AnySubscriber::try_receive` (`NodeContext::subscriber` hands it out).
    /// Under Sync those are two subscribers with two cursors, so a body read
    /// carries a different — usually newer — stamp, and folding it into the
    /// alignment map changes the spread the scheduler actually saw.
    ///
    /// There is no record-level discriminator, so the rule keys on the one
    /// shape the drain alone cannot produce: it runs ONCE per step per trigger
    /// input, so two `DrainedBatch` records for one `(node, input)` in one step
    /// is structural evidence the edge is contaminated.
    ///
    /// Deleting the pre-pass makes the first half report a
    /// `Decider::Sync` FINDING — a divergence on a live-consistent trace.
    #[test]
    fn a_trigger_input_carrying_both_read_sites_stands_the_node_down() {
        let cam = EPOCH + 100 * MS;
        let lidar = EPOCH + 130 * MS; // 30 ms apart — inside a 50 ms window
                                      // The body's own `try_receive` popped a newer frame during the tick.
        let body = EPOCH + 400 * MS;
        let clock = EPOCH + 500 * MS;

        let nodes = vec![node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        )];
        let steps = vec![RecordedStep {
            step: 0,
            clock_ns: clock,
            fires: vec![fire("fusion", clock)],
            reads: vec![
                read("fusion", "cam", ReadOutcomeKind::DrainedBatch, 1, Some(cam)),
                read(
                    "fusion",
                    "lidar",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(lidar),
                ),
                // The SECOND drain-kind record on `cam` in ONE step: the sync
                // drain cannot have produced it.
                read(
                    "fusion",
                    "cam",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(body),
                ),
            ],
        }];
        let report = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert!(
            report.findings.is_empty(),
            "an unrecoverable alignment map must not yield a verdict: {:?}",
            report.findings
        );
        assert!(
            report
                .stand_downs
                .iter()
                .any(|s| s.decider == Decider::Sync
                    && s.reason == StandDownReason::AmbiguousReadSite),
            "…it must SAY it declined: {:?}",
            report.stand_downs
        );

        // WHOLE-RUN scope: a body that reads an input this way does so on every
        // fire, so one sighting disqualifies the node — a LATER step whose
        // stamps are plainly misaligned still yields no finding.
        let mut steps = steps;
        steps.push(RecordedStep {
            step: 1,
            clock_ns: clock + 500 * MS,
            fires: vec![fire("fusion", clock + 500 * MS)],
            reads: vec![
                read("fusion", "cam", ReadOutcomeKind::DrainedBatch, 1, Some(cam)),
                read(
                    "fusion",
                    "lidar",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(cam + 900 * MS),
                ),
            ],
        });
        assert!(
            verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined)
                .findings
                .is_empty(),
            "the stand-down covers the whole run, not one step"
        );
    }

    /// ANTI-TAUTOLOGY for the arm above: the rule keys on the IMPOSSIBLE shape,
    /// not on "a Sync node has drain records".
    ///
    /// Without this, standing every Sync node down would pass that test — and
    /// would silently retire the whole decider.
    #[test]
    fn one_drain_record_per_input_per_step_is_not_ambiguous() {
        let a = EPOCH + 10 * MS;
        let b = EPOCH + 20 * MS;
        let nodes = vec![node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["a", "b"]),
            },
        )];
        // One drain record per input per step, across two steps, PLUS body
        // `Served` reads (which the site rule already excludes) — none of it is
        // the ambiguous shape.
        let steps = vec![
            RecordedStep {
                step: 0,
                clock_ns: EPOCH + 30 * MS,
                fires: vec![fire("fusion", EPOCH + 30 * MS)],
                reads: vec![
                    read("fusion", "a", ReadOutcomeKind::DrainedBatch, 1, Some(a)),
                    read("fusion", "b", ReadOutcomeKind::DrainedBatch, 1, Some(b)),
                    read("fusion", "a", ReadOutcomeKind::Served, 1, Some(a)),
                    read("fusion", "b", ReadOutcomeKind::Served, 1, Some(b)),
                ],
            },
            RecordedStep {
                step: 1,
                clock_ns: EPOCH + 60 * MS,
                fires: vec![fire("fusion", EPOCH + 60 * MS)],
                reads: vec![
                    read(
                        "fusion",
                        "a",
                        ReadOutcomeKind::DrainedBatch,
                        1,
                        Some(a + 40 * MS),
                    ),
                    read(
                        "fusion",
                        "b",
                        ReadOutcomeKind::DrainedBatch,
                        1,
                        Some(b + 40 * MS),
                    ),
                ],
            },
        ];
        let report = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert!(report.is_clean(), "unexpected: {:?}", report.findings);
        assert!(
            !report
                .stand_downs
                .iter()
                .any(|s| s.reason == StandDownReason::AmbiguousReadSite),
            "the ordinary shape must still be JUDGED: {:?}",
            report.stand_downs
        );

        // …and the decider is still live on it: the same edge, misaligned, is
        // still caught. Without this the arm above could pass on a rule that
        // stood every node down for some other reason.
        let mut misaligned = steps;
        misaligned[1].reads[1] = read(
            "fusion",
            "b",
            ReadOutcomeKind::DrainedBatch,
            1,
            Some(b + 900 * MS),
        );
        assert!(!verify_unrestored(&nodes, &misaligned, &[], RoleTrust::Declined,).is_clean());
    }

    /// A trigger input whose wire stamp could not be joined stands the node
    /// down rather than guessing an alignment.
    #[test]
    fn a_sync_input_with_no_joinable_stamp_stands_down() {
        let nodes = vec![node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["a", "b"]),
            },
        )];
        let steps = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![],
            reads: vec![read("fusion", "a", ReadOutcomeKind::DrainedBatch, 1, None)],
        }];
        let report = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert!(report.is_clean());
        assert_eq!(
            report.stand_downs[0].reason,
            StandDownReason::MissingWireStamp
        );
    }

    // -- FIFO pop counts ---------------------------------------------------

    /// Per-message FIFO: one pop, one fire. A step popping three frames fires
    /// three times.
    #[test]
    fn fifo_fires_match_the_read_logs_pop_counts() {
        let nodes = vec![node(
            "sink",
            TriggerDecl::Data {
                trigger_inputs: inputs(&["inp"]),
            },
        )];
        let steps = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![
                fire("sink", EPOCH),
                fire("sink", EPOCH),
                fire("sink", EPOCH),
            ],
            reads: vec![
                read("sink", "inp", ReadOutcomeKind::DrainedBatch, 1, None),
                read("sink", "inp", ReadOutcomeKind::DrainedBatch, 1, None),
                read("sink", "inp", ReadOutcomeKind::DrainedBatch, 1, None),
                // A body read of an already-popped frame must NOT be counted.
                read("sink", "inp", ReadOutcomeKind::Served, 0, None),
            ],
        }];
        let report = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert!(report.is_clean(), "unexpected: {:?}", report.findings);

        // The anti-tautology half: drop one fire and the mismatch is caught.
        let mut short = steps.clone();
        short[0].fires.pop();
        let bad = verify_unrestored(&nodes, &short, &[], RoleTrust::Declined);
        assert!(!bad.is_clean());
        assert_eq!(bad.findings[0].decider, Decider::FifoPop);
        assert_eq!(bad.findings[0].expected, StepFires::count(3));
        assert_eq!(bad.findings[0].recorded, StepFires::count(2));
    }

    /// A truncated read log is a deterministic HOLE, so the pop-count rule
    /// declines rather than reporting the floor as a divergence — the
    /// rule ("a verdicting verifier cannot treat a truncation hole as compared
    /// clean") applied to this decider.
    #[test]
    fn a_truncated_read_log_stands_the_pop_count_down_rather_than_diverging() {
        let nodes = vec![node(
            "sink",
            TriggerDecl::Data {
                trigger_inputs: inputs(&["inp"]),
            },
        )];
        let steps = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("sink", EPOCH), fire("sink", EPOCH)],
            reads: vec![
                read("sink", "inp", ReadOutcomeKind::DrainedBatch, 1, None),
                // The marker says a record that WOULD have followed was
                // dropped at the rim; `popped` is the drop count, not a pop.
                read("sink", "inp", ReadOutcomeKind::Truncated, 1, None),
            ],
        }];
        let report = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert!(
            report.is_clean(),
            "a truncation hole must not read as divergence: {:?}",
            report.findings
        );
        assert_eq!(
            report.stand_downs,
            vec![StandDown {
                node_id: "sink".to_string(),
                decider: Decider::FifoPop,
                reason: StandDownReason::TruncatedReadLog,
            }]
        );
        // ANTI-TAUTOLOGY: the same fires with the marker replaced by a real
        // pop verify clean, so the stand-down is the marker's doing and not a
        // fixture that could never diverge either way.
        let mut healed = steps.clone();
        healed[0].reads[1].kind = ReadOutcomeKind::DrainedBatch;
        let ok = verify_unrestored(&nodes, &healed, &[], RoleTrust::Declined);
        assert!(ok.is_clean());
        assert!(ok.stand_downs.is_empty());
    }

    // -- Block credit conservation ----------------------------------------

    /// Conservation holds when SOME legal interleave exists — pinned on both
    /// sides of each bound by a hand oracle, then through the engine.
    #[test]
    fn conservation_holds_under_some_legal_interleave() {
        // (published, drained, depth) -> breach?
        let oracle: &[ConservationOracleRow] = &[
            ((10, 10, 4), None),
            ((14, 10, 4), None), // outstanding == depth: the inclusive edge
            (
                (15, 10, 4),
                Some(ConservationBreach::OverRun { outstanding: 5 }),
            ),
            ((10, 11, 4), Some(ConservationBreach::UnderRun)),
            ((0, 0, 0), None),
        ];
        for &((published, drained, depth), expect) in oracle {
            assert_eq!(
                conservation_breach(published, drained, depth),
                expect,
                "conservation_breach({published}, {drained}, {depth})"
            );
        }

        let edges = vec![BlockEdgeObservation {
            topic: "/blk".to_string(),
            producer_node: "producer".to_string(),
            consumer_node: "sink".to_string(),
            consumer_input: "inp".to_string(),
            depth: 4,
            published_frames: 6,
        }];
        let steps = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![],
            reads: vec![
                read("sink", "inp", ReadOutcomeKind::DrainedBatch, 3, None),
                read("sink", "inp", ReadOutcomeKind::DrainedBatch, 2, None),
            ],
        }];
        let report = verify_unrestored(&[], &steps, &edges, RoleTrust::Declined);
        assert!(report.conservation.is_empty(), "{:?}", report.conservation);
        assert!(report.is_clean());
    }

    /// **The non-gating arm.** A conservation breach is LOUD — a typed finding
    /// that names the edge and both counts — and it does NOT gate the verdict:
    /// `is_clean()` stays true and `divergence_class()` stays `None`, because
    /// the design states the assert as "loud on violation, never a gate".
    ///
    /// Making conservation a gate (folding it into `is_clean` /
    /// `divergence_class`) fails exactly here.
    #[test]
    fn a_conservation_breach_is_loud_but_never_gates_the_verdict() {
        let edges = vec![BlockEdgeObservation {
            topic: "/blk".to_string(),
            producer_node: "producer".to_string(),
            consumer_node: "sink".to_string(),
            consumer_input: "inp".to_string(),
            depth: 2,
            published_frames: 9,
        }];
        let steps = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![],
            reads: vec![read("sink", "inp", ReadOutcomeKind::DrainedBatch, 3, None)],
        }];
        let report = verify_unrestored(&[], &steps, &edges, RoleTrust::Declined);

        // LOUD.
        assert_eq!(
            report.conservation,
            vec![ConservationFinding {
                topic: "/blk".to_string(),
                consumer_node: "sink".to_string(),
                consumer_input: "inp".to_string(),
                depth: 2,
                published: 9,
                drained: 3,
                breach: ConservationBreach::OverRun { outstanding: 6 },
            }]
        );
        assert!(report.conservation[0]
            .to_string()
            .contains("report-only — never a replay verdict"));

        // NEVER A GATE.
        assert!(
            report.is_clean(),
            "conservation must not gate the schedule verdict"
        );
        assert_eq!(report.divergence_class(), None);
        assert!(report.findings.is_empty());
    }

    /// A truncated consumer read log makes `drained` a FLOOR, which can only
    /// manufacture a false under-run — so the edge stands down instead.
    #[test]
    fn a_truncated_consumer_log_stands_conservation_down() {
        let edges = vec![BlockEdgeObservation {
            topic: "/blk".to_string(),
            producer_node: "producer".to_string(),
            consumer_node: "sink".to_string(),
            consumer_input: "inp".to_string(),
            depth: 2,
            published_frames: 1,
        }];
        let steps = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![],
            reads: vec![
                read("sink", "inp", ReadOutcomeKind::DrainedBatch, 5, None),
                read("sink", "inp", ReadOutcomeKind::Truncated, 2, None),
            ],
        }];
        let report = verify_unrestored(&[], &steps, &edges, RoleTrust::Declined);
        assert!(
            report.conservation.is_empty(),
            "a floor must not read as an under-run: {:?}",
            report.conservation
        );
        assert_eq!(
            report.stand_downs,
            vec![StandDown {
                node_id: "sink".to_string(),
                decider: Decider::BlockCredit,
                reason: StandDownReason::TruncatedReadLog,
            }]
        );
        // ANTI-TAUTOLOGY: without the marker the same counts ARE an under-run.
        let mut healed = steps.clone();
        healed[0].reads.pop();
        assert_eq!(
            verify_unrestored(&[], &healed, &edges, RoleTrust::Declined).conservation[0].breach,
            ConservationBreach::UnderRun
        );
    }

    // -- Composition -------------------------------------------------------

    /// A throttled Sync node: the throttle can SUPPRESS an aligned fire, so
    /// the "aligned but did not fire" direction stands down while "fired while
    /// misaligned" stays judged.
    #[test]
    fn a_suppressible_sync_node_keeps_only_the_direction_it_can_judge() {
        let mut n = node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["a", "b"]),
            },
        );
        n.throttle_ns = Some(500 * MS);

        // Aligned, and the recording does NOT fire — the throttle explains it.
        let quiet = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![],
            reads: vec![
                read("fusion", "a", ReadOutcomeKind::DrainedBatch, 1, Some(EPOCH)),
                read("fusion", "b", ReadOutcomeKind::DrainedBatch, 1, Some(EPOCH)),
            ],
        }];
        let report = verify_unrestored(std::slice::from_ref(&n), &quiet, &[], RoleTrust::Declined);
        assert!(report.is_clean(), "unexpected: {:?}", report.findings);
        assert!(report.stand_downs.contains(&StandDown {
            node_id: "fusion".to_string(),
            decider: Decider::Sync,
            reason: StandDownReason::SuppressibleFire,
        }));

        // MISALIGNED and the recording fires anyway — still caught.
        let loud = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("fusion", EPOCH)],
            reads: vec![
                read("fusion", "a", ReadOutcomeKind::DrainedBatch, 1, Some(EPOCH)),
                read(
                    "fusion",
                    "b",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH + 400 * MS),
                ),
            ],
        }];
        let bad = verify_unrestored(&[n], &loud, &[], RoleTrust::Declined);
        assert!(!bad.is_clean());
        assert_eq!(bad.findings[0].decider, Decider::Sync);
    }

    /// Determinism: the same inputs give the same report, and node declaration
    /// order fixes the finding order.
    #[test]
    fn the_report_is_a_deterministic_function_of_its_inputs() {
        let nodes = vec![
            node("zeta", period(10 * MS, None)),
            node("alpha", period(10 * MS, None)),
        ];
        let steps = vec![
            step(0, EPOCH, vec![fire("zeta", EPOCH), fire("alpha", EPOCH)]),
            // Both diverge at step 1 (neither deadline reached, both fire).
            step(
                1,
                EPOCH + MS,
                vec![
                    fire("zeta", EPOCH + 10 * MS),
                    fire("alpha", EPOCH + 10 * MS),
                ],
            ),
        ];
        let a = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        let b = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert_eq!(a, b);
        assert_eq!(
            a.findings
                .iter()
                .map(|f| f.node_id.as_str())
                .collect::<Vec<_>>(),
            vec!["zeta", "alpha"],
            "declaration order, not alphabetical"
        );
    }

    // =======================================================================
    // The READ-SITE ROLE branch. Pure oracles, beside the rules
    // they judge (no crafted-bag or e2e arms here).
    // =======================================================================

    /// The ONE site predicate all three consumers share, as a hand table.
    ///
    /// The UNSTAMPED rows are the archived-bag rule verbatim (kind IS the site), and
    /// they are what every archived bag flows through; the STAMPED rows are
    /// the exact split the wire now supplies.
    #[test]
    fn the_drain_site_predicate_answers_its_hand_table() {
        let table = [
            // (kind, role, roles_stamped, expected)
            (
                ReadOutcomeKind::DrainedBatch,
                ReadSiteRole::Unstamped,
                false,
                true,
            ),
            (
                ReadOutcomeKind::DrainedBatch,
                ReadSiteRole::Drain,
                true,
                true,
            ),
            // THE row the role bit exists for: a BODY batch (an accumulate-all
            // `try_receive` in a node's own tick) is NOT the drain.
            (
                ReadOutcomeKind::DrainedBatch,
                ReadSiteRole::Body,
                true,
                false,
            ),
            // A stamped bag whose record carries an UNWRITTEN `0` decodes to
            // `Unstamped` and takes the pre-roles arm rather than guessing.
            (
                ReadOutcomeKind::DrainedBatch,
                ReadSiteRole::Unstamped,
                true,
                true,
            ),
            // The format-5 peek/head mark: a PEEK popped the queue on the scheduler's
            // behalf, so it IS a drain-site read here — its `popped` is the only
            // place that pop is accounted, and refusing it loses a frame from
            // the FIFO sum and from `block` credit. What excludes it from the
            // sync fold is `is_sync_head_record`, a different question.
            (
                ReadOutcomeKind::DrainedBatch,
                ReadSiteRole::Peek,
                true,
                true,
            ),
            // A role on an ARCHIVED bag is never believed, whatever the bits
            // happen to say — the gate is the FORMAT.
            (
                ReadOutcomeKind::DrainedBatch,
                ReadSiteRole::Body,
                false,
                true,
            ),
            // Every other kind is a body read under both arms.
            (ReadOutcomeKind::Served, ReadSiteRole::Drain, true, false),
            (ReadOutcomeKind::Held, ReadSiteRole::Drain, true, false),
            (ReadOutcomeKind::Truncated, ReadSiteRole::Drain, true, false),
        ];
        for (kind, role, stamped, expected) in table {
            let r = read_with_role("n", "i", kind, 1, Some(EPOCH), role);
            assert_eq!(
                is_drain_site_read(&r, RoleTrust::from_stamped(stamped)),
                expected,
                "is_drain_site_read({kind:?}, {role:?}, roles_stamped={stamped})"
            );
        }
    }

    /// The detector's THREE verdicts, as a hand table over the two shapes that
    /// discriminate them. The CORRUPT arm must not rest on the
    /// false premise that the sync drain runs once per step.
    ///
    /// * **AMBIGUITY** is a fold carrying more than one drain-site record of
    ///   which at least ONE is KIND-INFERRED. That covers every archived bag,
    ///   and on a stamped bag exactly the records whose role decoded to
    ///   [`ReadSiteRole::Unstamped`] (an unwritten `0`, or the RESERVED `3`).
    /// * **CORRUPTION** is a BELIEVED `Drain` role on a BODY-ONLY kind — a
    ///   `(kind, role)` pair the mint-site table says cannot be written.
    /// * **ATTRIBUTABLE** is everything else, and the headline row is the one
    ///   this table exists for: N drain-role `DrainedBatch` records for one
    ///   `(node, input)` in one step is PER-SET SYNC WORKING (a `FillBoundary`
    ///   drain plus the matcher's `sync_peek_next_stamp` / `sync_discard_head`
    ///   pops), and a once-per-step premise would call it "corrupt — re-record the
    ///   bag".
    ///
    /// The CORRUPT rows drive EVERY body-only kind and EVERY legitimate
    /// pairing, because a table that only drove `Served` would pass an
    /// implementation that hardcoded it.
    #[test]
    fn the_read_site_verdict_separates_ambiguity_from_corruption() {
        let inputs = inputs(&["cam"]);
        let rec = |kind: ReadOutcomeKind, role: ReadSiteRole| {
            read_with_role("fusion", "cam", kind, 1, Some(EPOCH), role)
        };
        let batch = |role: ReadSiteRole| rec(ReadOutcomeKind::DrainedBatch, role);
        let step_of = |step: u64, reads: Vec<RecordedRead>| RecordedStep {
            step,
            clock_ns: EPOCH,
            fires: Vec::new(),
            reads,
        };
        let verdict = |steps: &[RecordedStep], trust| {
            sync_read_sites_verdict("fusion", steps, &inputs, trust)
        };
        let two_batches =
            |a: ReadSiteRole, b: ReadSiteRole| vec![step_of(0, vec![batch(a), batch(b)])];

        // ── AMBIGUITY: the answer on an archived bag.
        let steps = two_batches(ReadSiteRole::Unstamped, ReadSiteRole::Unstamped);
        assert_eq!(
            verdict(&steps, RoleTrust::Declined),
            ReadSiteVerdict::Ambiguous
        );
        assert_eq!(
            verdict(&steps, RoleTrust::Declined).stand_down(),
            Some(StandDownReason::AmbiguousReadSite)
        );
        // …and on a STAMPED bag whose colliding records carry an UNWRITTEN `0`
        // (a hand-edited or foreign-written bag — no mint site can elect it):
        // they are drain-site reads BY KIND, the collision is real, and nothing
        // on the wire named a site.
        assert_eq!(
            verdict(&steps, RoleTrust::Believed),
            ReadSiteVerdict::Ambiguous,
            "a stamped bag whose colliding records carry an unnamed role must \
             report the collision it really has — never nothing"
        );
        // MIXED: one believed `Drain` beside one inferred record is still a
        // collision — the inferred one is in the fold and could be anything.
        assert_eq!(
            verdict(
                &two_batches(ReadSiteRole::Drain, ReadSiteRole::Unstamped),
                RoleTrust::Believed
            ),
            ReadSiteVerdict::Ambiguous,
            "MIXED sets take the weaker reading"
        );

        // ── THE HEADLINE ROW: N drain-role batches in one step
        //    is PER-SET SYNC, not corruption. Driven at 2 and at 5, because the
        //    matcher's descent legitimately mints a whole run of them.
        assert_eq!(
            verdict(
                &two_batches(ReadSiteRole::Drain, ReadSiteRole::Drain),
                RoleTrust::Believed
            ),
            ReadSiteVerdict::Attributable,
            "ONE healthy per-set Sync align() mints a FillBoundary drain PLUS a \
             peek pop PLUS discard-head pops, every one of them a drain-role \
             DrainedBatch on the same (node, input) in the same step"
        );
        let five_drains = vec![step_of(
            0,
            (0..5).map(|_| batch(ReadSiteRole::Drain)).collect(),
        )];
        assert_eq!(
            verdict(&five_drains, RoleTrust::Believed),
            ReadSiteVerdict::Attributable,
            "a deeper descent is more of the same, not more corrupt"
        );

        // ── ATTRIBUTABLE: drain + body — THE residual the roles close. The
        //    identical record stream stands the node down on an archived bag.
        let steps = two_batches(ReadSiteRole::Drain, ReadSiteRole::Body);
        assert_eq!(
            verdict(&steps, RoleTrust::Believed),
            ReadSiteVerdict::Attributable
        );
        assert_eq!(verdict(&steps, RoleTrust::Believed).stand_down(), None);
        assert_eq!(
            verdict(&steps, RoleTrust::Declined),
            ReadSiteVerdict::Ambiguous,
            "the SAME stream on an archived bag still stands down"
        );

        // ── CORRUPTION: a believed `Drain` on a kind only the BODY mints.
        for kind in [
            ReadOutcomeKind::Served,
            ReadOutcomeKind::Held,
            ReadOutcomeKind::NoFrame,
        ] {
            let steps = vec![step_of(0, vec![rec(kind, ReadSiteRole::Drain)])];
            assert_eq!(
                verdict(&steps, RoleTrust::Believed),
                ReadSiteVerdict::Corrupt,
                "{kind:?} is staged ONLY by the node body's own read paths, so a \
                 believed Drain role on it names a site that cannot have written it"
            );
            assert_eq!(
                verdict(&steps, RoleTrust::Believed).stand_down(),
                Some(StandDownReason::ImpossibleReadShape)
            );
            // The SAME record on a bag whose bits are declined proves nothing:
            // a corruption claim must rest on bits that were believed.
            assert_eq!(
                verdict(&steps, RoleTrust::Declined),
                ReadSiteVerdict::Attributable,
                "{kind:?}: the bits were DECLINED, so they cannot convict"
            );
            // And a BODY role on the same kind is what a recorder really writes.
            assert_eq!(
                verdict(
                    &[step_of(0, vec![rec(kind, ReadSiteRole::Body)])],
                    RoleTrust::Believed
                ),
                ReadSiteVerdict::Attributable,
                "{kind:?} at the BODY site is the ordinary shape"
            );
        }
        // The kinds BOTH sites mint, and the two whose role is not a call-site
        // role at all, are never convictable — an over-eager predicate that
        // judged every `Drain` role would fail here.
        for kind in [
            ReadOutcomeKind::DrainedBatch,
            ReadOutcomeKind::Decimated,
            ReadOutcomeKind::Truncated,
            ReadOutcomeKind::Producer,
        ] {
            assert_eq!(
                verdict(
                    &[step_of(0, vec![rec(kind, ReadSiteRole::Drain)])],
                    RoleTrust::Believed
                ),
                ReadSiteVerdict::Attributable,
                "{kind:?} carries a legitimate Drain role"
            );
        }

        // ── With the format-5 role definition: `Peek` has exactly ONE mint —
        //    `sync_peek_next_stamp`'s `Sample` arm, which stages
        //    `DrainedBatch`, plus the `Producer` token paired with that same
        //    read on an annotated edge. Every OTHER kind under it is a
        //    `(kind, role)` pair no recorder can write.
        for kind in [ReadOutcomeKind::DrainedBatch, ReadOutcomeKind::Producer] {
            assert_eq!(
                verdict(
                    &[step_of(0, vec![rec(kind, ReadSiteRole::Peek)])],
                    RoleTrust::Believed
                ),
                ReadSiteVerdict::Attributable,
                "{kind:?} at the PEEK site is what the matcher really writes"
            );
        }
        for kind in [
            ReadOutcomeKind::Served,
            ReadOutcomeKind::Held,
            ReadOutcomeKind::NoFrame,
            // The peek's OWN decimated arm keeps `Drain` (a decimated pop parks
            // nothing), so this pairing is unproducible too — the arm that
            // makes the decision in `sync_peek_next_stamp` checkable offline.
            ReadOutcomeKind::Decimated,
            // A marker carries a STAGE role, and `From<ReadStageRole>` is total
            // over two variants, neither of which is `Peek`.
            ReadOutcomeKind::Truncated,
        ] {
            let steps = vec![step_of(0, vec![rec(kind, ReadSiteRole::Peek)])];
            assert_eq!(
                verdict(&steps, RoleTrust::Believed),
                ReadSiteVerdict::Corrupt,
                "{kind:?} under a believed PEEK role names a site that cannot \
                 have written it"
            );
            assert_eq!(
                verdict(&steps, RoleTrust::Declined),
                ReadSiteVerdict::Attributable,
                "{kind:?}: the bits were DECLINED, so they cannot convict"
            );
        }

        // ── WHOLE-RUN PRECEDENCE: a proven corruption in a LATER step outranks
        //    an earlier ambiguity — `Corrupt` is evidence about the recording,
        //    and meeting it second does not make it less true.
        let steps = vec![
            step_of(
                0,
                vec![
                    batch(ReadSiteRole::Unstamped),
                    batch(ReadSiteRole::Unstamped),
                ],
            ),
            step_of(1, vec![rec(ReadOutcomeKind::Served, ReadSiteRole::Drain)]),
        ];
        assert_eq!(
            verdict(&steps, RoleTrust::Believed),
            ReadSiteVerdict::Corrupt,
            "an early ambiguity must not mask a later PROVEN impossible shape"
        );

        // Both stand-down reasons render their own words (an operator reading
        // one must not be told the other's cause), and each carries its own
        // stable machine token.
        assert!(StandDownReason::ImpossibleReadShape
            .label()
            .contains("corrupt recording"));
        assert_ne!(
            StandDownReason::ImpossibleReadShape.label(),
            StandDownReason::AmbiguousReadSite.label()
        );
        assert_eq!(
            StandDownReason::ImpossibleReadShape.code(),
            "impossible_read_shape"
        );
        assert_eq!(
            StandDownReason::AmbiguousReadSite.code(),
            "ambiguous_read_site"
        );
    }

    /// Every [`StandDownReason`] carries a DISTINCT, stable machine token, and
    /// the token is not the label.
    ///
    /// The tokens are what a CI job greps; the labels are prose that may be
    /// reworded. Two reasons sharing a token would silently merge two
    /// conditions in whatever dashboard reads them, which is the class this
    /// repo has already paid for once (`total_failures=` vs four spellings).
    #[test]
    fn every_stand_down_reason_has_its_own_machine_token() {
        let all = [
            StandDownReason::BlockGated,
            StandDownReason::NoPhaseAnchor,
            StandDownReason::ZeroInterval,
            StandDownReason::TruncatedReadLog,
            StandDownReason::DecimatedRead,
            StandDownReason::MissingWireStamp,
            StandDownReason::SuppressibleFire,
            StandDownReason::QueueExceedsVerifierRoom,
            StandDownReason::AmbiguousReadSite,
            StandDownReason::ImpossibleReadShape,
            StandDownReason::UnattributableCredit,
            StandDownReason::SyncHeadStampRegressed,
        ];
        let codes: std::collections::BTreeSet<&str> = all.iter().map(|r| r.code()).collect();
        assert_eq!(codes.len(), all.len(), "every reason's token is distinct");
        for r in &all {
            assert!(
                !r.code().is_empty() && r.code() != r.label(),
                "{r:?}: the token is a grep key, never the rewordable sentence"
            );
            assert!(
                r.code().chars().all(|c| c.is_ascii_lowercase() || c == '_'),
                "{r:?}: tokens are snake_case ASCII, matching `replay_inject`'s"
            );
        }
    }

    /// The unproducible `(kind, role)` pair is
    /// judged on a DATA node's FIFO stream too, not only where the Sync verdict
    /// happens to ask.
    ///
    /// It exists because deleting `verify_fifo`'s check killed NOTHING —
    /// measured, the whole suite stayed green — which is the shape a rule
    /// written at one of two call sites always has. A pop count derived from a
    /// stream carrying a record the recorder cannot have written is arithmetic
    /// over fiction, and it lands under [`Decider::FifoPop`] rather than
    /// `Sync`, so a reader can tell WHICH rule declined.
    ///
    /// Three claims, because the first alone is satisfied by a `verify_fifo`
    /// that stands every node down: the corrupt stream STANDS DOWN and produces
    /// NO finding (the whole point — an arithmetic verdict on it would be
    /// worse than none), an ARCHIVED reading of the identical stream does NOT
    /// (the bits were declined), and a HEALTHY stream is judged normally.
    #[test]
    fn a_data_nodes_unproducible_read_stands_the_fifo_rule_down() {
        let n = node(
            "sink",
            TriggerDecl::Data {
                trigger_inputs: inputs(&["inp"]),
            },
        );
        let corrupt_stream = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("sink", EPOCH)],
            reads: vec![
                read_with_role(
                    "sink",
                    "inp",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH),
                    ReadSiteRole::Drain,
                ),
                // `Served` is staged ONLY by the node body's own read paths.
                read_with_role(
                    "sink",
                    "inp",
                    ReadOutcomeKind::Served,
                    0,
                    Some(EPOCH),
                    ReadSiteRole::Drain,
                ),
            ],
        }];

        let believed = verify_unrestored(
            std::slice::from_ref(&n),
            &corrupt_stream,
            &[],
            RoleTrust::Believed,
        );
        assert_eq!(
            believed
                .stand_downs
                .iter()
                .map(|s| (s.decider, s.reason.clone()))
                .collect::<Vec<_>>(),
            vec![(Decider::ReadLog, StandDownReason::ImpossibleReadShape)],
            "the RECORDING is judged before any rule is, and exactly once"
        );
        assert!(
            believed.findings.is_empty(),
            "…and produces no arithmetic verdict over records no recorder wrote: {:?}",
            believed.findings
        );

        // ARCHIVED: the same bits, declined. The FIFO rule judges normally (one
        // drain pop, one fire), so the stand-down above really is about the
        // BELIEVED role rather than about the record's shape.
        let archived = verify_unrestored(
            std::slice::from_ref(&n),
            &corrupt_stream,
            &[],
            RoleTrust::Declined,
        );
        assert!(
            archived.stand_downs.is_empty() && archived.findings.is_empty(),
            "an archived bag cannot convict on bits it may not read: {:?} / {:?}",
            archived.stand_downs,
            archived.findings
        );

        // HEALTHY: the same node, a producible stream. Without this the
        // stand-down above is satisfied by a rule that declines on everything.
        let healthy = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("sink", EPOCH)],
            reads: vec![
                read_with_role(
                    "sink",
                    "inp",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH),
                    ReadSiteRole::Drain,
                ),
                read_with_role(
                    "sink",
                    "inp",
                    ReadOutcomeKind::Served,
                    0,
                    Some(EPOCH),
                    ReadSiteRole::Body,
                ),
            ],
        }];
        let ok = verify_unrestored(std::slice::from_ref(&n), &healthy, &[], RoleTrust::Believed);
        assert!(
            ok.stand_downs.is_empty() && ok.is_clean(),
            "a body-role `Served` beside a drain-role batch is the ORDINARY \
             shape: {:?} / {:?}",
            ok.stand_downs,
            ok.findings
        );
    }

    /// A finding derived under ROLE
    /// STEERING on a BODY-HEAVY stream carries the split it was derived from.
    ///
    /// The FIFO shape, because it is the sharpest: role steering excludes the
    /// body batches from the pop count, so the node's `expected 1 pop` against
    /// `recorded 3 fires` is arithmetic the EXCLUSION produced. The finding
    /// stands (it is what the numbers say); the census is what lets a reader
    /// see why it might be describing the exclusion rather than a divergence.
    ///
    /// THREE claims, and the last two are what stop it being a rubber stamp: an
    /// ARCHIVED bag emits none (nothing was excluded), and a DRAIN-HEAVY stream
    /// emits none either (the exclusion cannot explain the finding).
    #[test]
    fn a_finding_on_a_body_heavy_stream_carries_its_read_site_census() {
        let n = node(
            "sink",
            TriggerDecl::Data {
                trigger_inputs: inputs(&["inp"]),
            },
        );
        let batch = |role: ReadSiteRole| {
            read_with_role(
                "sink",
                "inp",
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(EPOCH),
                role,
            )
        };
        // One drain pop, THREE body batches, and three recorded fires: under
        // role steering the pop count is 1 and the fire count is 3.
        let body_heavy = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![
                fire("sink", EPOCH),
                fire("sink", EPOCH),
                fire("sink", EPOCH),
            ],
            reads: vec![
                batch(ReadSiteRole::Drain),
                batch(ReadSiteRole::Body),
                batch(ReadSiteRole::Body),
                batch(ReadSiteRole::Body),
            ],
        }];

        let believed = verify_unrestored(
            std::slice::from_ref(&n),
            &body_heavy,
            &[],
            RoleTrust::Believed,
        );
        assert_eq!(
            believed.findings.len(),
            1,
            "the exclusion really does produce a finding: {:?}",
            believed.findings
        );
        assert_eq!(
            believed.read_site_census,
            vec![ReadSiteCensus {
                node_id: "sink".to_string(),
                decider: Decider::FifoPop,
                inputs: vec![InputReadSites {
                    input: "inp".to_string(),
                    drain_records: 1,
                    body_records: 3,
                    drain_popped: 1,
                    body_popped: 3,
                }],
            }],
            "the finding carries the split it was derived from"
        );
        assert!(
            !believed.is_clean(),
            "the census is EVIDENCE, never a rescue — the finding stands"
        );
        let rendered = believed.read_site_census[0].to_string();
        assert!(
            rendered.contains("inp=records 1/3, pops 1/3") && rendered.contains("report-only"),
            "the note names the split and says it is not a verdict: {rendered}"
        );

        // ARCHIVED: nothing was excluded, so there is no split to report. (The
        // node stands down on the kind-inferred collision instead, which is the
        // pre-roles answer and is why `findings` is empty here.)
        let archived = verify_unrestored(
            std::slice::from_ref(&n),
            &body_heavy,
            &[],
            RoleTrust::Declined,
        );
        assert!(
            archived.read_site_census.is_empty(),
            "an archived bag has no drain/body split: {:?}",
            archived.read_site_census
        );

        // DRAIN-HEAVY: the same finding shape, but the exclusion cannot explain
        // it, so no census. Without this the census would be a rubber stamp on
        // every finding a stamped bag produces.
        let drain_heavy = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("sink", EPOCH), fire("sink", EPOCH)],
            reads: vec![batch(ReadSiteRole::Drain), batch(ReadSiteRole::Body)],
        }];
        let d = verify_unrestored(&[n], &drain_heavy, &[], RoleTrust::Believed);
        assert_eq!(d.findings.len(), 1, "still a finding: {:?}", d.findings);
        assert!(
            d.read_site_census.is_empty(),
            "a drain-heavy stream's finding is not explained by the exclusion: {:?}",
            d.read_site_census
        );
    }

    /// The stamp fold, one flag apart: a BODY `DrainedBatch` carrying a stamp
    /// that would push a genuinely aligned pair outside its window is EXCLUDED
    /// on a stamped bag, and folded in (reporting a divergence on a trace that
    /// is fine) on an unstamped one.
    ///
    /// This is the hazard `verify_sync` documents at length —
    /// "a frame landing between the drain and the tick gives the body read a
    /// NEWER stamp … reporting a divergence on a trace that is fine" — with
    /// the role now able to say so.
    #[test]
    fn a_body_batch_does_not_move_the_sync_stamp_map_on_a_stamped_bag() {
        let nodes = vec![node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        )];
        // cam and lidar are aligned at the DRAIN (10 ms apart). A body read on
        // `cam` carries a stamp 400 ms later — inside the same step, because
        // the body reads its own subscriber with its own cursor.
        let steps = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("fusion", EPOCH)],
            reads: vec![
                read_with_role(
                    "fusion",
                    "cam",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH),
                    ReadSiteRole::Drain,
                ),
                read_with_role(
                    "fusion",
                    "lidar",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH + 10 * MS),
                    ReadSiteRole::Drain,
                ),
                read_with_role(
                    "fusion",
                    "cam",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH + 400 * MS),
                    ReadSiteRole::Body,
                ),
            ],
        }];

        let stamped = verify_unrestored(&nodes, &steps, &[], RoleTrust::Believed);
        assert!(
            stamped.is_clean(),
            "the DRAIN stamps are 10 ms apart — aligned: {:?}",
            stamped.findings
        );
        assert!(
            stamped.stand_downs.is_empty(),
            "a drain+body stream is attributable on a stamped bag: {:?}",
            stamped.stand_downs
        );

        // The SAME stream read as an archived bag: two `DrainedBatch` records
        // for one `(node, input)` in one step is the ambiguity signature, so
        // the node stands down rather than being judged on a stamp map that
        // may not be the scheduler's.
        let archived = verify_unrestored(&nodes, &steps, &[], RoleTrust::Declined);
        assert_eq!(
            archived
                .stand_downs
                .iter()
                .map(|s| s.reason.clone())
                .collect::<Vec<_>>(),
            vec![StandDownReason::AmbiguousReadSite]
        );
    }

    /// FIFO and conservation count the DRAIN's pops, and on a stamped bag a
    /// body batch's pops are excluded exactly. One flag apart, one hand
    /// oracle each.
    #[test]
    fn a_body_batch_is_not_counted_as_a_pop_or_as_returned_credit() {
        // ── FIFO: one drain pop, one fire. A body batch on the same input
        //    would make the expected pop count 2 and the fire count 1.
        let n = node(
            "sink",
            TriggerDecl::Data {
                trigger_inputs: inputs(&["inp"]),
            },
        );
        let steps = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("sink", EPOCH)],
            reads: vec![
                read_with_role(
                    "sink",
                    "inp",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH),
                    ReadSiteRole::Drain,
                ),
                read_with_role(
                    "sink",
                    "inp",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH),
                    ReadSiteRole::Body,
                ),
            ],
        }];
        assert!(
            verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Believed,)
                .is_clean(),
            "one DRAIN pop, one fire"
        );
        let archived =
            verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Declined);
        assert!(
            !archived.is_clean(),
            "counting BOTH batches expects 2 pops against 1 fire"
        );

        // ── Conservation does NOT share that rule, and the difference is the
        //    subject of `a_block_edge_whose_pops_span_two_sites_declines`
        //    below: a body pop on a `block` input really did decrement the
        //    credit mirror, so the pops are not excluded — they are either
        //    summed or declined, depending on whether the stream evidences one
        //    subscriber port or two. Here it is two, so the edge declines.
        let edges = vec![BlockEdgeObservation {
            topic: "/t".to_string(),
            producer_node: "src".to_string(),
            consumer_node: "sink".to_string(),
            consumer_input: "inp".to_string(),
            depth: 1,
            published_frames: 3,
        }];
        let stamped = verify_unrestored(&[], &steps, &edges, RoleTrust::Believed);
        assert!(
            stamped.conservation.is_empty(),
            "a two-site stream yields no credit number at all: {:?}",
            stamped.conservation
        );
        assert_eq!(
            stamped
                .stand_downs
                .iter()
                .map(|s| (s.decider, s.reason.clone()))
                .collect::<Vec<_>>(),
            vec![(Decider::BlockCredit, StandDownReason::UnattributableCredit)],
            "…it declines, and names the reason"
        );
        // ARCHIVED: the pre-roles rule verbatim — `DrainedBatch` popped, both records,
        // so 2 drained against 3 published is 1 outstanding at depth 1: clean.
        let archived = verify_unrestored(&[], &steps, &edges, RoleTrust::Declined);
        assert!(
            archived.conservation.is_empty() && archived.stand_downs.is_empty(),
            "the pre-roles arm counts both batches and finds nothing: {:?} {:?}",
            archived.conservation,
            archived.stand_downs
        );
    }

    // -- The corrupt-shape sweep -------------------------------

    /// [`unproducible_read_shape`] is a claim about the RECORDING, so it
    /// is asked for every node and every input — not only where a read-driven
    /// rule happens to look.
    ///
    /// The three arms are the three holes the per-rule asks left, each verified
    /// against the shipping code rather than imagined: a `Period` node runs no
    /// read loop at all; a block-gated `Data` node returns from `verify_fifo`
    /// BEFORE its read loop; and both read loops filter on
    /// `trigger_inputs.contains`, so a NON-trigger input's records would never be
    /// examined by anything. A corrupt record on any of them would be silent.
    #[test]
    fn a_corrupt_read_shape_stands_a_node_down_whatever_rule_would_have_judged_it() {
        // `Served` under `Drain`: minted by no site in `transport::subscriber`.
        let corrupt = |input: &str| {
            read_with_role(
                "n",
                input,
                ReadOutcomeKind::Served,
                0,
                Some(EPOCH),
                ReadSiteRole::Drain,
            )
        };
        let declined = |report: &RederivationReport| {
            report
                .stand_downs
                .iter()
                .filter(|s| s.reason == StandDownReason::ImpossibleReadShape)
                .map(|s| s.decider)
                .collect::<Vec<_>>()
        };

        // (a) a Period node — no read-driven rule to decline.
        let steps = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("n", EPOCH)],
            reads: vec![corrupt("inp")],
        }];
        let n = node("n", period(10 * MS, None));
        assert_eq!(
            declined(&verify_unrestored(
                std::slice::from_ref(&n),
                &steps,
                &[],
                RoleTrust::Believed,
            )),
            vec![Decider::ReadLog],
            "a Period node's corrupt read is reported"
        );

        // (b) a block-gated Data node — `verify_fifo` returns before its loop.
        let n = node(
            "n",
            TriggerDecl::Data {
                trigger_inputs: inputs(&["inp"]),
            },
        );
        let gating = vec![BlockEdgeObservation {
            topic: "/t".to_string(),
            producer_node: "n".to_string(),
            consumer_node: "other".to_string(),
            consumer_input: "x".to_string(),
            depth: 4,
            published_frames: 0,
        }];
        assert_eq!(
            declined(&verify_unrestored(
                std::slice::from_ref(&n),
                &steps,
                &gating,
                RoleTrust::Believed,
            )),
            vec![Decider::ReadLog],
            "a block-gated Data node's corrupt read is reported"
        );

        // (c) a NON-trigger input — outside every rule's `trigger_inputs` fold.
        let non_trigger = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("n", EPOCH)],
            reads: vec![corrupt("context")],
        }];
        assert_eq!(
            declined(&verify_unrestored(
                std::slice::from_ref(&n),
                &non_trigger,
                &[],
                RoleTrust::Believed,
            )),
            vec![Decider::ReadLog],
            "a corrupt read on a NON-trigger input is reported"
        );

        // ANTI-TAUTOLOGY, both directions: the identical stream on an ARCHIVED
        // bag makes no corruption claim (the bits were not believed), and a
        // HEALTHY believed stream makes none either.
        assert!(
            declined(&verify_unrestored(
                std::slice::from_ref(&n),
                &non_trigger,
                &[],
                RoleTrust::Declined,
            ))
            .is_empty(),
            "an archived bag's identical bits are not a corruption claim"
        );
        let healthy = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("n", EPOCH)],
            reads: vec![read_with_role(
                "n",
                "inp",
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(EPOCH),
                ReadSiteRole::Drain,
            )],
        }];
        assert!(
            declined(&verify_unrestored(
                std::slice::from_ref(&n),
                &healthy,
                &[],
                RoleTrust::Believed,
            ))
            .is_empty(),
            "a healthy believed stream is not corrupt"
        );
    }

    // -- Block credit attribution ------------------------------

    /// The conservation rule mirrors the runtime's DECREMENT SITES, and the
    /// oracle walks every shape a recorder can produce for a `block` edge.
    ///
    /// Each row is `(reads, expected)`, hand-derived from
    /// `transport::subscriber`'s mint-site table and the probe wiring at
    /// `graph/runtime.rs:5168` — never from this module's own arithmetic. The
    /// edge is depth 1 with 3 frames published, so `drained` decides between a
    /// clean edge (>= 2) and an over-run (< 2).
    #[test]
    fn block_credit_is_summed_per_port_or_declined() {
        let edges = vec![BlockEdgeObservation {
            topic: "/t".to_string(),
            producer_node: "src".to_string(),
            consumer_node: "sink".to_string(),
            consumer_input: "inp".to_string(),
            depth: 1,
            published_frames: 3,
        }];
        let r = |kind: ReadOutcomeKind, popped: u32, role: ReadSiteRole| {
            read_with_role("sink", "inp", kind, popped, Some(EPOCH), role)
        };
        // A NON-TRIGGER `block` input: excluded from the step-boundary snapshot
        // (`graph/runtime.rs:11440`) and served live by the body off the ONE
        // probed subscriber, with no trigger-drain sub in existence. Basis
        // `AllPops`, decided by the graph rather than assumed.
        let consumer = node(
            "sink",
            TriggerDecl::Data {
                trigger_inputs: inputs(&["trig"]),
            },
        );
        let run = |reads: Vec<RecordedRead>, trust: RoleTrust| {
            let steps = vec![RecordedStep {
                step: 0,
                clock_ns: EPOCH,
                fires: Vec::new(),
                reads,
            }];
            let report = verify_unrestored(std::slice::from_ref(&consumer), &steps, &edges, trust);
            (
                report.conservation.iter().map(|c| c.drained).next(),
                report.conservation.iter().map(|c| c.breach).next(),
                report
                    .stand_downs
                    .iter()
                    // Scoped to the CREDIT decider: with a real node
                    // declaration in play the whole-node corrupt-shape sweep
                    // fires too (`Decider::ReadLog`), and this matrix is about
                    // what the credit rule concluded.
                    .filter(|s| s.decider == Decider::BlockCredit)
                    .map(|s| s.reason.clone())
                    .collect::<Vec<_>>(),
            )
        };

        use ReadOutcomeKind as K;
        use ReadSiteRole as R;

        // UNIFIED per-set Sync: the boundary drain runs on the BODY (probed)
        // subscriber, so its Drain-role pops ARE the mirror's decrements.
        assert_eq!(
            run(vec![r(K::DrainedBatch, 2, R::Drain)], RoleTrust::Believed),
            (None, None, vec![]),
            "2 of 3 drained at depth 1 is clean"
        );

        // NON-TRIGGER `block` input: excluded from the step-boundary snapshot,
        // so the body reads it live and records `Served` — never a
        // `DrainedBatch`. The pre-roles rule reads `drained == 0` here and
        // reports an over-run on a healthy edge; this is THE headline case.
        assert_eq!(
            run(vec![r(K::Served, 2, R::Body)], RoleTrust::Believed),
            (None, None, vec![]),
            "a Served-only edge's pops are real credit"
        );
        assert_eq!(
            run(vec![r(K::Served, 2, R::Body)], RoleTrust::Declined).1,
            Some(ConservationBreach::OverRun { outstanding: 3 }),
            "…and the ARCHIVED arm still reads 0 drained, byte-unchanged"
        );

        // An accumulate-all `try_receive` in a node body pops the probed port
        // under `Body`, and a junk-skipped drain records `NoFrame` carrying a
        // real pop count. Both are credit.
        assert_eq!(
            run(
                vec![r(K::DrainedBatch, 1, R::Body), r(K::NoFrame, 1, R::Body)],
                RoleTrust::Believed
            ),
            (None, None, vec![]),
            "a body batch and a junk-skipping NoFrame both returned credit"
        );

        // A REAL over-run still reports, with the exact drained count.
        assert_eq!(
            run(vec![r(K::Served, 1, R::Body)], RoleTrust::Believed),
            (
                Some(1),
                Some(ConservationBreach::OverRun { outstanding: 2 }),
                vec![]
            ),
            "1 of 3 drained at depth 1 over-runs"
        );

        // BOTH sites popped on a NON-TRIGGER input: there is no trigger-drain
        // subscriber for such an input at all, so both are pops of the one
        // probed port and the sum is exact. (The Sync fork, where the same
        // shape IS undecidable, is pinned in
        // `block_credit_basis_follows_the_consumers_declared_trigger`.)
        assert_eq!(
            run(
                vec![r(K::DrainedBatch, 2, R::Drain), r(K::Served, 1, R::Body)],
                RoleTrust::Believed
            ),
            (None, None, vec![]),
            "one probed port, so every pop is credit"
        );

        // A pop the wire attributes to NO site (an unwritten 0 / the RESERVED
        // 3) cannot be placed on a port either.
        assert_eq!(
            run(
                vec![r(K::DrainedBatch, 2, R::Unstamped)],
                RoleTrust::Believed
            ),
            (None, None, vec![StandDownReason::UnattributableCredit]),
        );
        // …but an unstamped record that popped NOTHING enters no sum, so it
        // must not stand a healthy edge down.
        assert_eq!(
            run(
                vec![
                    r(K::DrainedBatch, 2, R::Drain),
                    r(K::NoFrame, 0, R::Unstamped)
                ],
                RoleTrust::Believed
            ),
            (None, None, vec![]),
            "a zero-pop unstamped record is not evidence of a second port"
        );

        // A `Producer` annotation's `popped` slot is part of the 64-bit token,
        // so it is not a pop. Counting it here would read 2 + a token fragment.
        assert_eq!(
            run(
                vec![
                    r(K::DrainedBatch, 2, R::Drain),
                    r(K::Producer, 4_000_000, R::Drain)
                ],
                RoleTrust::Believed
            ),
            (None, None, vec![]),
            "a producer annotation is not a pop"
        );

        // PRECEDENCE: corrupt outranks truncated outranks unattributable.
        assert_eq!(
            run(
                vec![
                    r(K::Served, 0, R::Drain),
                    r(K::Truncated, 9, R::Drain),
                    r(K::DrainedBatch, 1, R::Drain),
                    r(K::Served, 1, R::Body),
                ],
                RoleTrust::Believed
            ),
            (None, None, vec![StandDownReason::ImpossibleReadShape]),
        );
        assert_eq!(
            run(
                vec![
                    r(K::Truncated, 9, R::Drain),
                    r(K::DrainedBatch, 1, R::Drain),
                    r(K::Served, 1, R::Body),
                ],
                RoleTrust::Believed
            ),
            (None, None, vec![StandDownReason::TruncatedReadLog]),
        );
    }

    /// Which records count as returned credit
    /// is decided by the CONSUMER'S DECLARED TRIGGER, not by the wire.
    ///
    /// A credit rule that declines every stream carrying pops
    /// under both roles is useless: by `NodeInfo::unifies_data_trigger`
    /// (`graph/node.rs:777-788`, which unifies a data trigger only when its
    /// backpressure is `DropOldest`) a `block` DATA trigger is ALWAYS Separate,
    /// so that shape is EVERY cross-rank block data-trigger edge there is. Such
    /// a verifier declines the case it exists for.
    ///
    /// Each arm is a hand oracle over a depth-1 edge carrying 6 published
    /// frames, so `drained` alone decides between clean, an over-run and an
    /// under-run — and the three bases give three different answers to the same
    /// record stream, which is what makes the basis load-bearing rather than
    /// decorative.
    #[test]
    fn block_credit_basis_follows_the_consumers_declared_trigger() {
        let edges = vec![BlockEdgeObservation {
            topic: "/t".to_string(),
            producer_node: "src".to_string(),
            consumer_node: "sink".to_string(),
            consumer_input: "inp".to_string(),
            depth: 1,
            published_frames: 6,
        }];
        let r = |kind: ReadOutcomeKind, popped: u32, role: ReadSiteRole| {
            read_with_role("sink", "inp", kind, popped, Some(EPOCH), role)
        };
        let run = |decls: &[NodeDecl], reads: Vec<RecordedRead>| {
            let steps = vec![RecordedStep {
                step: 0,
                clock_ns: EPOCH,
                fires: Vec::new(),
                reads,
            }];
            let report = verify_unrestored(decls, &steps, &edges, RoleTrust::Believed);
            (
                report.conservation.iter().map(|c| c.drained).next(),
                report.conservation.iter().map(|c| c.breach).next(),
                report
                    .stand_downs
                    .iter()
                    .filter(|s| s.decider == Decider::BlockCredit)
                    .map(|s| s.reason.clone())
                    .collect::<Vec<_>>(),
            )
        };
        use ReadOutcomeKind as K;
        use ReadSiteRole as R;

        // ── (a) DATA trigger + block ⇒ SEPARATE by construction.
        //
        // The counts are deliberately UNEQUAL — the trigger-drain sub popped 6
        // while the body consumed 4 (a throttled body) — because equal counts
        // cannot tell the three bases apart. Only the BODY pops decremented the
        // mirror, so 6 published − 4 drained = 2 outstanding against depth 1.
        let data = node(
            "sink",
            TriggerDecl::Data {
                trigger_inputs: inputs(&["inp"]),
            },
        );
        let separate_shape = vec![r(K::DrainedBatch, 6, R::Drain), r(K::Served, 4, R::Body)];
        assert_eq!(
            run(std::slice::from_ref(&data), separate_shape.clone()),
            (
                Some(4),
                Some(ConservationBreach::OverRun { outstanding: 2 }),
                vec![]
            ),
            "only the PROBED body subscriber's pops return credit"
        );

        // ── (b) SYNC trigger + block, in the shape per-set really produces.
        //
        // The body read is served from the FROZEN slot and records NOTHING
        // (accounting-once), so an ordinary per-set stream is DRAIN-ONLY — and
        // those pops are on the body subscriber, because per-set unifies onto
        // it (`graph/runtime.rs:5683`). One role, so both readings of the Sync
        // fork give the same number and it is counted exactly.
        let sync = node(
            "sink",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["inp"]),
            },
        );
        assert_eq!(
            run(
                std::slice::from_ref(&sync),
                vec![r(K::DrainedBatch, 6, R::Drain)]
            ),
            (None, None, vec![]),
            "a per-set Sync boundary drain runs on the PROBED sub, so its pops \
             are credit — 6 of 6 drained is clean"
        );

        // …and the shape the wire CANNOT settle: pops under BOTH roles on a
        // Sync trigger input. Per-set (unified, sum = 6, clean) and legacy
        // (separate + unprobed drain sub, body-only = 1, a 5-frame over-run)
        // disagree, and `per_set_capable` (`graph/runtime.rs:5587`) depends on
        // SYMBOL PRESENCE (`graph/node.rs:5617`, an ADDITIVE export with no ABI
        // bump per `graph/node.rs:3917`) and on an env knob — neither of which
        // is in the bag. So it declines rather than pick.
        assert_eq!(
            run(
                std::slice::from_ref(&sync),
                vec![r(K::DrainedBatch, 5, R::Drain), r(K::Served, 1, R::Body)]
            ),
            (None, None, vec![StandDownReason::UnattributableCredit]),
            "the Sync fork is not on the wire, and the two readings differ by 5 \
             frames on this very stream"
        );

        // ── (c) A NON-TRIGGER `block` input has NO drain subscriber at all
        //    (`graph/runtime.rs:11440` excludes it from the snapshot), so the
        //    identical record stream from (a) is ONE port and sums to 10 —
        //    which EXCEEDS what was published and is a visible under-run
        //    report, not a silent wrong number.
        let non_trigger = node(
            "sink",
            TriggerDecl::Data {
                trigger_inputs: inputs(&["other"]),
            },
        );
        assert_eq!(
            run(std::slice::from_ref(&non_trigger), separate_shape.clone()),
            (Some(10), Some(ConservationBreach::UnderRun), vec![]),
            "the SAME records under a different basis give a different answer — \
             which is why the basis is read from the graph and not guessed"
        );

        // ── (d) A pop under a role the wire did not NAME is placeable by
        //    nothing, whatever the basis says.
        assert_eq!(
            run(
                std::slice::from_ref(&data),
                vec![r(K::DrainedBatch, 4, R::Unstamped)]
            ),
            (None, None, vec![StandDownReason::UnattributableCredit]),
        );

        // ── The consumer is not in the declarations at all ⇒ no basis. Guessing
        //    would be a guess about which PORT the pops came from.
        assert_eq!(
            run(&[], separate_shape.clone()),
            (None, None, vec![StandDownReason::UnattributableCredit]),
        );

        // ── (e) The ARCHIVED arm ignores every word above: `DrainedBatch`
        //    popped, and only that, exactly as before roles existed.
        let steps = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: Vec::new(),
            reads: separate_shape,
        }];
        let archived = verify_unrestored(
            std::slice::from_ref(&data),
            &steps,
            &edges,
            RoleTrust::Declined,
        );
        assert_eq!(
            archived
                .conservation
                .iter()
                .map(|c| (c.drained, c.breach))
                .collect::<Vec<_>>(),
            vec![],
            "6 published, 6 drain-batch pops ⇒ 0 outstanding: the pre-roles rule, clean"
        );
        assert!(
            archived
                .stand_downs
                .iter()
                .all(|s| s.decider != Decider::BlockCredit),
            "…and the archived arm never reaches the role decline: {:?}",
            archived.stand_downs
        );
    }

    /// A conservation finding on a BELIEVED bag carries NO
    /// read-site census.
    ///
    /// A census arm annotating one, filtered to `UnderRun`, would name
    /// the direction role exclusion CANNOT manufacture — excluding
    /// reads only LOWERS `drained`, which makes an under-run harder and an
    /// over-run easier. There is no such arm, flipped or otherwise, because
    /// `verify_conservation` excludes nothing by role: it sums every
    /// pop or declines. This is the pin that fails if an arm is added.
    #[test]
    fn a_believed_bag_conservation_finding_carries_no_read_site_census() {
        let edges = vec![BlockEdgeObservation {
            topic: "/t".to_string(),
            producer_node: "src".to_string(),
            consumer_node: "sink".to_string(),
            consumer_input: "inp".to_string(),
            depth: 1,
            published_frames: 5,
        }];
        // A body-HEAVY stream (3 body records to 1 drain) that still yields a
        // real over-run: the shape an annotating arm would have fired on.
        let sink = node(
            "sink",
            TriggerDecl::Data {
                trigger_inputs: inputs(&["inp"]),
            },
        );
        let steps = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: Vec::new(),
            reads: vec![
                read_with_role(
                    "sink",
                    "inp",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH),
                    ReadSiteRole::Body,
                ),
                read_with_role(
                    "sink",
                    "inp",
                    ReadOutcomeKind::Served,
                    1,
                    Some(EPOCH),
                    ReadSiteRole::Body,
                ),
                read_with_role(
                    "sink",
                    "inp",
                    ReadOutcomeKind::Served,
                    1,
                    Some(EPOCH),
                    ReadSiteRole::Body,
                ),
            ],
        }];
        let report = verify_unrestored(
            std::slice::from_ref(&sink),
            &steps,
            &edges,
            RoleTrust::Believed,
        );
        assert_eq!(
            report
                .conservation
                .iter()
                .map(|c| (c.drained, c.breach))
                .collect::<Vec<_>>(),
            vec![(3, ConservationBreach::OverRun { outstanding: 2 })],
            "every body pop is credit, and 3 of 5 at depth 1 still over-runs"
        );
        assert!(
            report
                .read_site_census
                .iter()
                .all(|c| c.decider != Decider::BlockCredit),
            "no census annotates a credit finding: {:?}",
            report.read_site_census
        );
    }

    // -- With the format-5 role definition: the peek/head mark -----------------------------

    /// **The fold is EXACT for a per-set Sync descent, because the wire now
    /// says which record was the head.**
    ///
    /// Without the format-5 peek/head mark a peek record and a head record
    /// are byte-indistinguishable
    /// (both `DrainedBatch` under `Drain`), so a last-wins fold cannot
    /// tell them apart and the correct answer is to judge nothing. With
    /// [`ReadSiteRole::Peek`] on the wire the fold takes each input's stamp
    /// from its last `Drain`-role record — the fill, an `Advance`/`DiscardTie`
    /// refill, or a promotion — and that record IS the head at fire time
    /// however deep the descent went.
    ///
    /// FOUR vectors, and each kills a DIFFERENT variant:
    ///
    /// 1. **NON-IMPROVING PEEK** — the head aligns; the peeked frame is 400 ms
    ///    out, far outside the 50 ms window. This is the shape that produces a
    ///    FALSE `ScheduleFinding` (exit 6, not report-only) under a role-blind
    ///    fold, and it must read
    ///    CLEAN. A fold that takes the last record REGARDLESS of role reports a
    ///    divergence here.
    /// 2. **ONE DISCARD** — a fill, a peek, and the PROMOTION of that peeked
    ///    frame. The head at fire time is the PROMOTED frame, so the fold must
    ///    move to it: the fill is 100 ms from the partner and the promotion is
    ///    10 ms from it, so a fold that ignored the promotion record (which is
    ///    what a recorder without the head mark does — the promotion stages nothing)
    ///    reports a divergence on a recording that is fine.
    /// 3. **CONTROL** — one drain record per input, judged normally.
    /// 4. **ARCHIVED** — the identical descent on a `trace_format` <= 4 bag is
    ///    two or more kind-inferred `DrainedBatch` records, which the pre-roles
    ///    collision detector still stands down as `AmbiguousReadSite`. That arm
    ///    does not read the role bits, which proves the believed-role reading is gated on the
    ///    FORMAT rather than applied to every bag.
    ///
    /// # What vectors 1-2 pin
    ///
    /// Vectors 1 and 2 pass through arm B's FEASIBILITY sweep, which asks
    /// whether SOME in-window tuple exists over ALL the candidates — a question
    /// the peek/head distinction cannot change, because the transversal admits
    /// peeks as candidates outright (vector 1 is feasible at `EPOCH`, vector 2
    /// at `(EPOCH+90, EPOCH+100)`). They are therefore NOT the pin for the peek
    /// EXCLUSION: that job belongs to
    /// `a_peek_stamp_never_explains_a_missing_fire`, which drives arm C — the
    /// one arm reading the NARROW map — and fails when `Peek` is admitted into
    /// [`is_sync_head_record`].
    #[test]
    fn a_per_set_sync_descent_folds_its_head_not_its_peeks() {
        let n = node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        );
        let at = |input: &str, stamp: u64, role: ReadSiteRole, popped: u32| {
            read_with_role(
                "fusion",
                input,
                ReadOutcomeKind::DrainedBatch,
                popped,
                Some(stamp),
                role,
            )
        };
        let reasons = |report: &RederivationReport| {
            report
                .stand_downs
                .iter()
                .map(|s| s.reason.clone())
                .collect::<Vec<_>>()
        };

        // 1. NON-IMPROVING PEEK. `cam`'s head is EPOCH and aligns with
        //    `lidar`; the matcher looked at a frame 400 ms later and fired on
        //    the head anyway.
        let peeked = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("fusion", EPOCH)],
            reads: vec![
                at("cam", EPOCH, ReadSiteRole::Drain, 1),
                at("cam", EPOCH + 400 * MS, ReadSiteRole::Peek, 1),
                at("lidar", EPOCH, ReadSiteRole::Drain, 1),
            ],
        }];
        let report = verify_unrestored(std::slice::from_ref(&n), &peeked, &[], RoleTrust::Believed);
        assert!(
            report.is_clean(),
            "THE HEADLINE: a peeked frame is not the head, so a step the \
             scheduler fired on an aligned pair is CLEAN — folding the peek \
             reports a divergence on a healthy recording: {report:?}"
        );

        // 2. ONE DISCARD: fill(EPOCH) → peek(EPOCH+90ms) → PROMOTION of that
        //    same frame. `lidar` sits at EPOCH+100ms, so only the promotion's
        //    stamp is inside the 50 ms window — the fill is 100 ms away.
        let promoted = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("fusion", EPOCH)],
            reads: vec![
                at("cam", EPOCH, ReadSiteRole::Drain, 1),
                at("cam", EPOCH + 90 * MS, ReadSiteRole::Peek, 1),
                // The promotion: same frame, `popped: 0` — the pop was
                // accounted at the peek.
                at("cam", EPOCH + 90 * MS, ReadSiteRole::Drain, 0),
                at("lidar", EPOCH + 100 * MS, ReadSiteRole::Drain, 1),
            ],
        }];
        assert!(
            verify_unrestored(
                std::slice::from_ref(&n),
                &promoted,
                &[],
                RoleTrust::Believed,
            )
            .is_clean(),
            "the PROMOTION names the head, so the fold moves to it — without \
             that record the fold holds the fill's stamp, 100 ms from the \
             partner, and reports a divergence"
        );

        // 3. CONTROL: one drain record per input per step, judged normally.
        let single = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("fusion", EPOCH)],
            reads: vec![
                at("cam", EPOCH, ReadSiteRole::Drain, 1),
                at("lidar", EPOCH + MS, ReadSiteRole::Drain, 1),
            ],
        }];
        assert!(
            verify_unrestored(std::slice::from_ref(&n), &single, &[], RoleTrust::Believed,)
                .is_clean(),
            "the ordinary shape is unaffected"
        );

        // 4. ARCHIVED: the identical descent read WITHOUT believing the bits is
        //    two kind-inferred drain reads, and the pre-roles detector
        //    still stands them down.
        assert_eq!(
            reasons(&verify_unrestored(
                std::slice::from_ref(&n),
                &peeked,
                &[],
                RoleTrust::Declined,
            )),
            vec![StandDownReason::AmbiguousReadSite],
        );
    }

    /// **THE ANTI-FALSE-EXIT-6 ARM: a per-set burst of ALIGNED sets is not
    /// a schedule divergence.**
    ///
    /// Per-set Sync (the DEFAULT) answers an aligned step
    /// with `FireKind::Sync { max_sets }` and `tick_sync_burst` loops
    /// `fire → mark_sync_heads_emitted → align(Refill)` until the refill comes
    /// up `Incomplete`. So TWO arrivals per input in one step is TWO fires, and
    /// a `count > 1 && aligned ⇒ expected 1` arm brands that a
    /// `ScheduleFinding` — which is `DivergenceClass::FireSchedule`, i.e. EXIT
    /// 6, on a recording that is perfectly healthy.
    ///
    /// The rule is the SET-SUPPLY ceiling: `k` fires need `k` heads on
    /// every input, and each head is a WIDE record (or the carry). Two records
    /// per input supply two sets, so two fires are legal.
    ///
    /// A `count > 1 && aligned ⇒ expected 1` arm fails
    /// this test and `a_recorded_per_set_sync_burst_replays_without_a_schedule_divergence`.
    #[test]
    fn a_per_set_burst_of_aligned_sets_is_not_a_schedule_divergence() {
        let n = node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        );
        let at = |input: &str, stamp: u64, role: ReadSiteRole, popped: u32| {
            read_with_role(
                "fusion",
                input,
                ReadOutcomeKind::DrainedBatch,
                popped,
                Some(stamp),
                role,
            )
        };

        // TWO sets: a boundary fill and a burst refill on each input, all four
        // stamps mutually inside the 50 ms window, TWO fires.
        let two = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("fusion", EPOCH), fire("fusion", EPOCH)],
            reads: vec![
                at("cam", EPOCH, ReadSiteRole::Drain, 1),
                at("lidar", EPOCH + 5 * MS, ReadSiteRole::Drain, 1),
                at("cam", EPOCH + 10 * MS, ReadSiteRole::Drain, 1),
                at("lidar", EPOCH + 15 * MS, ReadSiteRole::Drain, 1),
            ],
        }];
        let report = verify_unrestored(std::slice::from_ref(&n), &two, &[], RoleTrust::Believed);
        assert!(
            report.is_clean(),
            "THE HEADLINE: two arrivals per input SUPPLY two sets, so a per-set \
             burst of two fires is legal — the earlier arm called this \
             exit 6 on a healthy recording: {report:?}"
        );

        // THREE sets, three records per input — the same claim one deeper, so
        // the arm pins a BOUND rather than a hard-coded "2 is fine".
        let three = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![
                fire("fusion", EPOCH),
                fire("fusion", EPOCH),
                fire("fusion", EPOCH),
            ],
            reads: vec![
                at("cam", EPOCH, ReadSiteRole::Drain, 1),
                at("lidar", EPOCH + 2 * MS, ReadSiteRole::Drain, 1),
                at("cam", EPOCH + 4 * MS, ReadSiteRole::Drain, 1),
                at("lidar", EPOCH + 6 * MS, ReadSiteRole::Drain, 1),
                at("cam", EPOCH + 8 * MS, ReadSiteRole::Drain, 1),
                at("lidar", EPOCH + 10 * MS, ReadSiteRole::Drain, 1),
            ],
        }];
        assert!(
            verify_unrestored(std::slice::from_ref(&n), &three, &[], RoleTrust::Believed,)
                .is_clean(),
            "three supplied sets, three fires"
        );

        // THE PEEK-IN-THE-WIDE-SET ARM: one input's ENTIRE supply
        // for its second set is a bare `Peek`.
        //
        // That is the R-promote shape, and it is the reason the WIDE set admits peeks
        // at all: `sync_peek_next_stamp` pops a frame into `next_head` and
        // records the pop as a `Peek`, and `drain_for_trigger`'s R-promote then
        // makes it the head SILENTLY — `subscriber.rs` records nothing there,
        // deliberately and by measurement. So on this input the peek record is
        // the ONLY evidence a second head ever existed, and a supply model that
        // admitted `Drain` alone would count ONE head on `cam` against TWO
        // fires and convict a healthy burst.
        //
        // (The OTHER promote path — `sync_discard_head` — does mint a
        // `Drain`-at-`popped: 0` beside its peek, so that shape supplies TWO
        // units for one frame. The over-count is deliberate and harmless: arms
        // A and B are upper bounds, and an upper bound that is too high can
        // only decline to convict.)
        //
        // `lidar` carries two ordinary fills, so the finding a peek-excluding
        // model produces is attributable to `cam` alone.
        let peek_supplied = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("fusion", EPOCH), fire("fusion", EPOCH)],
            reads: vec![
                at("cam", EPOCH, ReadSiteRole::Drain, 1),
                at("lidar", EPOCH + 2 * MS, ReadSiteRole::Drain, 1),
                // The look-ahead pop. Its silent R-promote leaves no record.
                at("cam", EPOCH + 12 * MS, ReadSiteRole::Peek, 1),
                at("lidar", EPOCH + 14 * MS, ReadSiteRole::Drain, 1),
            ],
        }];
        assert!(
            verify_unrestored(
                std::slice::from_ref(&n),
                &peek_supplied,
                &[],
                RoleTrust::Believed,
            )
            .is_clean(),
            "a peeked frame silently promoted into the head is a set the burst \
             really fired, so it must count toward SUPPLY — excluding `Peek` \
             from the WIDE set convicts this burst"
        );
    }

    /// **The SECOND false-exit-6 class: a burst whose refills end OUT OF
    /// WINDOW is still a legal step.**
    ///
    /// `tick_sync_burst` runs `fire → align(Refill)` in a loop, so the LAST
    /// records of a step describe the set that was NOT fired — the refill that
    /// came up `Incomplete`. A LAST-WINS fold judges the
    /// step against exactly that unfired tuple: here `(1000 ms, 2000 ms)`,
    /// a full second apart against a 50 ms window ⇒ "recorded 1 fire while
    /// misaligned" ⇒ `expected: none` ⇒ exit 6.
    ///
    /// The set the scheduler actually fired on is `(0, 20 ms)`, and it is
    /// right there in the candidates. So the question is FEASIBILITY — does
    /// SOME choice of one candidate per input fit the window — not what the
    /// last record happened to say.
    ///
    /// Replacing the transversal with a last-wins fold fails this test.
    #[test]
    fn a_burst_whose_refills_end_out_of_window_is_still_a_legal_step() {
        let n = node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        );
        let at = |input: &str, stamp: u64| {
            read_with_role(
                "fusion",
                input,
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(stamp),
                ReadSiteRole::Drain,
            )
        };
        let steps = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("fusion", EPOCH)],
            reads: vec![
                // The FIRED set.
                at("cam", EPOCH),
                at("lidar", EPOCH + 20 * MS),
                // The refill that came up Incomplete — a full second apart, so
                // a last-wins fold reads the step as wildly misaligned.
                at("cam", EPOCH + 1000 * MS),
                at("lidar", EPOCH + 2000 * MS),
            ],
        }];
        let report = verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Believed);
        assert!(
            report.is_clean(),
            "the fired set (0, 20 ms) is among the candidates, so the step is \
             FEASIBLE — a last-wins fold judges it against the unfired refill \
             tuple (1000 ms, 2000 ms) and reports exit 6: {report:?}"
        );

        // ANTI-TAUTOLOGY: with NO feasible tuple at all — every candidate pair
        // more than the window apart — the step IS a finding. Without this the
        // arm above is satisfied by a rule that never reports anything.
        let infeasible = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("fusion", EPOCH)],
            reads: vec![
                at("cam", EPOCH),
                at("cam", EPOCH + 1000 * MS),
                at("lidar", EPOCH + 400 * MS),
                at("lidar", EPOCH + 2000 * MS),
            ],
        }];
        let bad = verify_unrestored(
            std::slice::from_ref(&n),
            &infeasible,
            &[],
            RoleTrust::Believed,
        );
        assert!(!bad.is_clean(), "no tuple fits the window: {bad:?}");
        assert_eq!(bad.findings[0].expected, StepFires::none());
        assert_eq!(bad.findings[0].decider, Decider::Sync);
    }

    /// **Arm A's teeth: a step that fired more sets than its arrivals
    /// could supply.**
    ///
    /// One arrival per input with no carry supplies exactly ONE set, so three
    /// fires is arithmetic no schedule can produce. The bound is
    /// `min_i supply(i)`, and this arm is what makes that term load-bearing —
    /// dropping it leaves only the `SYNC_BURST_MAX_SETS` clamp, which 3 fires
    /// clears easily.
    #[test]
    fn a_sync_step_that_fired_more_sets_than_its_arrivals_could_supply_is_caught() {
        let n = node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        );
        let steps = vec![RecordedStep {
            step: 7,
            clock_ns: EPOCH,
            fires: vec![
                fire("fusion", EPOCH),
                fire("fusion", EPOCH),
                fire("fusion", EPOCH),
            ],
            reads: vec![
                read_with_role(
                    "fusion",
                    "cam",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH),
                    ReadSiteRole::Drain,
                ),
                read_with_role(
                    "fusion",
                    "lidar",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH + 10 * MS),
                    ReadSiteRole::Drain,
                ),
            ],
        }];
        let report = verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Believed);
        assert!(!report.is_clean());
        let f = &report.findings[0];
        assert_eq!(f.step, 7);
        assert_eq!(f.decider, Decider::Sync);
        assert_eq!(
            f.expected,
            StepFires::count(1),
            "one arrival per input bounds the step at one set"
        );
        assert_eq!(f.recorded, StepFires::count(3));
        assert!(
            f.to_string()
                .contains("re-derived 1 fire(s), recorded 3 fire(s)"),
            "the UPPER BOUND renders as a count and reads correctly: {f}"
        );
    }

    /// **The scheduler's own burst ceiling, pinned on BOTH sides.**
    ///
    /// `tick_sync_burst` stops at `FireKind::Sync { max_sets }`, and `max_sets`
    /// is `cerulion_core`'s `DATA_PENDING_CARRY_CLAMP`. So a step with supply to
    /// spare is STILL bounded: `SYNC_BURST_MAX_SETS` fires is legal and one more
    /// is not, whatever the arrivals say.
    ///
    /// Both sides, because the ceiling is the half of arm A the supply term
    /// cannot cover — with 100 arrivals per input, `min_i supply` is 100 and
    /// only the clamp can catch 65. Its value comes from the core by
    /// RE-EXPORT, so this arm also pins that the two never drift.
    #[test]
    fn a_sync_burst_beyond_the_scheduler_clamp_is_caught() {
        let n = node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        );
        // 100 arrivals per input, every stamp inside the 50 ms window, so the
        // SUPPLY term is 100 and only the clamp can bound the step.
        let mut reads = Vec::new();
        for i in 0..100u64 {
            for input in ["cam", "lidar"] {
                reads.push(read_with_role(
                    "fusion",
                    input,
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH + i * 100_000),
                    ReadSiteRole::Drain,
                ));
            }
        }
        let at_ceiling = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("fusion", EPOCH); SYNC_BURST_MAX_SETS as usize],
            reads: reads.clone(),
        }];
        assert!(
            verify_unrestored(
                std::slice::from_ref(&n),
                &at_ceiling,
                &[],
                RoleTrust::Believed,
            )
            .is_clean(),
            "exactly `SYNC_BURST_MAX_SETS` sets is what the scheduler allows"
        );

        let over = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("fusion", EPOCH); SYNC_BURST_MAX_SETS as usize + 1],
            reads,
        }];
        let report = verify_unrestored(std::slice::from_ref(&n), &over, &[], RoleTrust::Believed);
        assert!(!report.is_clean());
        assert_eq!(
            report.findings[0].expected,
            StepFires::count(SYNC_BURST_MAX_SETS),
            "the ceiling comes from `cerulion_core`, never a literal here"
        );
        assert_eq!(
            report.findings[0].recorded,
            StepFires::count(SYNC_BURST_MAX_SETS + 1)
        );
    }

    /// **A head CARRIED across a quiet step supplies the next step's set.**
    ///
    /// A step that supplies more heads than it fires leaves one standing, and
    /// the scheduler really does: `align_sync_pass` fills a head and leaves it
    /// `Filled` until a fire tombstones it. So an input that arrived at step 0
    /// and stayed silent at step 1 is still a member of step 1's set.
    ///
    /// ANTI-TAUTOLOGY in the same body: the identical shape with the two stamps
    /// a full window apart IS a finding, so this arm pins the carry rather than
    /// a rule that never reports.
    #[test]
    fn a_carried_head_supplies_the_next_steps_set() {
        let n = node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["a", "b"]),
            },
        );
        let drain = |input: &str, stamp: u64| {
            read_with_role(
                "fusion",
                input,
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(stamp),
                ReadSiteRole::Drain,
            )
        };
        let two_steps = |b_stamp: u64| {
            vec![
                // Step 0: `a` arrives alone, nothing fires (correctly — `b` has
                // no head, so the set is incomplete).
                RecordedStep {
                    step: 0,
                    clock_ns: EPOCH,
                    fires: Vec::new(),
                    reads: vec![drain("a", EPOCH)],
                },
                // Step 1: `b` arrives and the set completes on the head `a`
                // has been holding since step 0.
                RecordedStep {
                    step: 1,
                    clock_ns: EPOCH + 100 * MS,
                    fires: vec![fire("fusion", EPOCH + 100 * MS)],
                    reads: vec![drain("b", b_stamp)],
                },
            ]
        };

        let report = verify_unrestored(
            std::slice::from_ref(&n),
            &two_steps(EPOCH + 10 * MS),
            &[],
            RoleTrust::Believed,
        );
        assert!(
            report.is_clean(),
            "`a`'s head survived step 0 (supply 1 > 0 fires), so step 1's set is \
             `(EPOCH, EPOCH+10ms)` — inside the window: {report:?}"
        );

        // ANTI-TAUTOLOGY: carry the head and the pair STILL does not fit.
        let bad = verify_unrestored(
            std::slice::from_ref(&n),
            &two_steps(EPOCH + 400 * MS),
            &[],
            RoleTrust::Believed,
        );
        assert!(!bad.is_clean(), "400 ms apart against a 50 ms window");
        assert_eq!(bad.findings[0].step, 1);
        assert_eq!(bad.findings[0].expected, StepFires::none());
    }

    /// **A PEEK stamp never explains a missing fire.**
    ///
    /// This is the arm that keeps the format-5 peek/head mark load-bearing
    /// after arms A and B moved onto the WIDE set (which admits
    /// peeks outright). Arm C — "a set was aligned and nothing fired" — is the
    /// one rule still reading the NARROW map, so it is the only place the
    /// distinction can still be pinned.
    ///
    /// `cam`'s HEAD is at `EPOCH` and aligns with `lidar`; the descent looked at
    /// a frame 400 ms out and parked it. Nothing fired, so the step owes a fire.
    ///
    /// Admitting `Peek` into `is_sync_head_record` makes `cam` read
    /// `EPOCH+400ms`, the pair reads unaligned, and the finding VANISHES.
    #[test]
    fn a_peek_stamp_never_explains_a_missing_fire() {
        let n = node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        );
        let at = |input: &str, stamp: u64, role: ReadSiteRole| {
            read_with_role(
                "fusion",
                input,
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(stamp),
                role,
            )
        };
        let steps = vec![RecordedStep {
            step: 2,
            clock_ns: EPOCH,
            fires: Vec::new(),
            reads: vec![
                at("cam", EPOCH, ReadSiteRole::Drain),
                at("cam", EPOCH + 400 * MS, ReadSiteRole::Peek),
                at("lidar", EPOCH, ReadSiteRole::Drain),
            ],
        }];
        let report = verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Believed);
        assert!(!report.is_clean(), "an aligned set owed a fire: {report:?}");
        let f = &report.findings[0];
        assert_eq!(f.step, 2);
        assert_eq!(f.decider, Decider::Sync);
        assert_eq!(f.expected, StepFires::count(1));
        assert_eq!(f.recorded, StepFires::none());
    }

    /// **The monotonicity guard: a head stamp that moves BACKWARD stands the node down.**
    ///
    /// Within one publisher epoch a head-naming stamp is non-decreasing — the
    /// fill, the refill and the promotion each name a frame at or after the one
    /// they replace. A backward jump is therefore an EPOCH RESET, and
    /// `Scheduler::apply_sync_epoch_reset` VOIDS every other input's held head
    /// with no record at all (`sync_void_head` stages nothing). Nothing on the
    /// wire says which heads went, so the map is no longer the scheduler's.
    ///
    /// The guard is keyed on the RECORD STREAM's floor, not on the live head:
    /// every fire consumes the head it would otherwise have compared against,
    /// so a map-keyed guard would be silent on exactly this shape (steps 0 and 1
    /// both fire).
    ///
    /// ANTI-TAUTOLOGY twin: the identical shape with step 2's stamps ASCENDING
    /// is judged normally and produces the arm-C finding.
    #[test]
    fn a_head_stamp_that_moves_backward_stands_the_node_down() {
        let n = node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        );
        let drain = |input: &str, stamp: u64| {
            read_with_role(
                "fusion",
                input,
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(stamp),
                ReadSiteRole::Drain,
            )
        };
        // Steps 0 and 1 fire on aligned pairs, consuming their heads. Step 2's
        // records are the variable: BELOW step 1's stamps (a reset) or above.
        let run = |step2: u64| {
            vec![
                RecordedStep {
                    step: 0,
                    clock_ns: EPOCH,
                    fires: vec![fire("fusion", EPOCH)],
                    reads: vec![
                        drain("cam", EPOCH + 100 * MS),
                        drain("lidar", EPOCH + 100 * MS),
                    ],
                },
                RecordedStep {
                    step: 1,
                    clock_ns: EPOCH + MS,
                    fires: vec![fire("fusion", EPOCH + MS)],
                    reads: vec![
                        drain("cam", EPOCH + 200 * MS),
                        drain("lidar", EPOCH + 200 * MS),
                    ],
                },
                // Aligned and UNFIRED — an arm-C finding, unless the node stood
                // down first.
                RecordedStep {
                    step: 2,
                    clock_ns: EPOCH + 2 * MS,
                    fires: Vec::new(),
                    reads: vec![drain("cam", step2), drain("lidar", step2)],
                },
            ]
        };

        let reset = verify_unrestored(
            std::slice::from_ref(&n),
            &run(EPOCH + 50 * MS),
            &[],
            RoleTrust::Believed,
        );
        assert!(
            reset.stand_downs.contains(&StandDown {
                node_id: "fusion".to_string(),
                decider: Decider::Sync,
                reason: StandDownReason::SyncHeadStampRegressed,
            }),
            "a backward head stamp is an epoch reset: {:?}",
            reset.stand_downs
        );
        assert!(
            reset.findings.is_empty(),
            "and nothing at or after the discontinuity is judged: {:?}",
            reset.findings
        );

        // ANTI-TAUTOLOGY: the same three steps with step 2 ASCENDING are judged,
        // and the step really would have produced a finding.
        let judged = verify_unrestored(
            std::slice::from_ref(&n),
            &run(EPOCH + 300 * MS),
            &[],
            RoleTrust::Believed,
        );
        assert!(
            judged
                .stand_downs
                .iter()
                .all(|s| s.reason != StandDownReason::SyncHeadStampRegressed),
            "ascending stamps are one epoch: {:?}",
            judged.stand_downs
        );
        assert_eq!(judged.findings.len(), 1);
        assert_eq!(judged.findings[0].step, 2);
        assert_eq!(judged.findings[0].expected, StepFires::count(1));
    }

    /// **A node with a NON-TRIGGER input keeps its carry across a fire, and
    /// says so.**
    ///
    /// The macro's generated tick nests one `try_view` per input in DECLARATION
    /// order, and an earlier input yielding nothing collapses the chain (the
    /// pre-first-delivery WAIT). A fire proves every TRIGGER head is
    /// `Filled`, so with no non-trigger input the chain cannot collapse and
    /// every trigger head is read; with one, a head can survive the fire in the
    /// frozen slot and the NEXT boundary re-offers it — silently, at the same
    /// stamp, with no record at either end.
    ///
    /// TWO `NodeDecl`s differing in EXACTLY that flag, over ONE read log. Both
    /// directions are the oracle: the widening is deliberate, and its COST — a
    /// finding it declines to make — is what the operator-visible
    /// [`CarriedHeadsNote`] exists to disclose.
    #[test]
    fn a_node_with_a_non_trigger_input_keeps_its_carry_across_a_fire() {
        let trigger_only = node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        );
        let with_context = NodeDecl {
            has_non_trigger_inputs: true,
            ..trigger_only.clone()
        };
        let drain = |input: &str, stamp: u64| {
            read_with_role(
                "fusion",
                input,
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(stamp),
                ReadSiteRole::Drain,
            )
        };
        let steps = vec![
            // Step 0: an ordinary aligned fire on `(cam, lidar)`.
            RecordedStep {
                step: 0,
                clock_ns: EPOCH,
                fires: vec![fire("fusion", EPOCH)],
                reads: vec![drain("cam", EPOCH), drain("lidar", EPOCH + 5 * MS)],
            },
            // Step 1: only `lidar` arrives, and the node fires again — which is
            // possible ONLY if `cam`'s head came back.
            RecordedStep {
                step: 1,
                clock_ns: EPOCH + 10 * MS,
                fires: vec![fire("fusion", EPOCH + 10 * MS)],
                reads: vec![drain("lidar", EPOCH + 20 * MS)],
            },
        ];

        let widened = verify_unrestored(
            std::slice::from_ref(&with_context),
            &steps,
            &[],
            RoleTrust::Believed,
        );
        assert!(
            widened.is_clean(),
            "the re-offered `cam` head is a candidate, so step 1's set exists: \
             {widened:?}"
        );
        assert_eq!(
            widened.carried_heads,
            vec![CarriedHeadsNote {
                node_id: "fusion".to_string()
            }],
            "and the widening is OPERATOR-VISIBLE, once per node"
        );

        let narrow = verify_unrestored(
            std::slice::from_ref(&trigger_only),
            &steps,
            &[],
            RoleTrust::Believed,
        );
        assert!(
            !narrow.is_clean(),
            "with every input trigger-marked the fire provably CONSUMED `cam`'s \
             head, so step 1 has no `cam` candidate at all"
        );
        assert_eq!(narrow.findings[0].step, 1);
        assert_eq!(narrow.findings[0].expected, StepFires::none());
        assert!(
            narrow.carried_heads.is_empty(),
            "…and nothing was widened, so there is nothing to disclose"
        );
    }

    /// **The widened carry's `depth + 2` cut is a THRESHOLD
    /// pinned on BOTH sides**, not merely a direction. `cam` accumulates `N`
    /// head-naming records in one step (no fires), so the widened carry
    /// (`survivors = supply.min(depth + 2)`) keeps the newest
    /// `min(N, depth + 2)` of them — dropping the OLDEST once `N` exceeds the
    /// cap. A LATER step's `lidar` record supplies exactly `cam`'s oldest
    /// stamp and the node fires: the fire's set exists (CLEAN) iff the
    /// oldest stamp survived the cut.
    ///
    /// `window_ns: Some(1)` (1 ns) makes only an EXACT stamp match count as
    /// in-window, so "the oldest stamp survived" is the ONLY way the
    /// transversal can be found — no coarser alignment can rescue a wrong
    /// answer on either side.
    #[test]
    fn the_widened_carry_cap_is_a_threshold_pinned_on_both_sides() {
        const DEPTH: u32 = 5;
        const CAP: u32 = DEPTH + 2; // 7

        let widened = NodeDecl {
            node_id: "fusion".to_string(),
            trigger: TriggerDecl::Sync {
                window_ns: Some(1),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
            throttle_ns: None,
            has_non_trigger_inputs: true,
            trigger_input_capacities: [("cam".to_string(), DEPTH)].into_iter().collect(),
        };

        let drive = |cam_count: u32| -> RederivationReport {
            // Step 0: `cam` supplies `cam_count` head-naming records, oldest
            // stamp EPOCH, each later one strictly greater by 1 ms — no
            // fires (arm B only runs on `k > 0`, and this step's own
            // candidates are irrelevant to it either way).
            let cam_reads: Vec<RecordedRead> = (0..cam_count)
                .map(|i| {
                    read_with_role(
                        "fusion",
                        "cam",
                        ReadOutcomeKind::DrainedBatch,
                        1,
                        Some(EPOCH + u64::from(i) * MS),
                        ReadSiteRole::Drain,
                    )
                })
                .collect();
            // Step 1: `lidar` supplies ONE record at EXACTLY `cam`'s oldest
            // stamp (EPOCH) and the node fires — in-window with `cam`'s
            // carry iff EPOCH survived the `depth + 2` cut.
            let steps = vec![
                RecordedStep {
                    step: 0,
                    clock_ns: EPOCH,
                    fires: Vec::new(),
                    reads: cam_reads,
                },
                RecordedStep {
                    step: 1,
                    clock_ns: EPOCH + 100 * MS,
                    fires: vec![fire("fusion", EPOCH + 100 * MS)],
                    reads: vec![read_with_role(
                        "fusion",
                        "lidar",
                        ReadOutcomeKind::DrainedBatch,
                        1,
                        Some(EPOCH),
                        ReadSiteRole::Drain,
                    )],
                },
            ];
            verify_unrestored(
                std::slice::from_ref(&widened),
                &steps,
                &[],
                RoleTrust::Believed,
            )
        };

        let at_cap = drive(CAP);
        assert!(
            at_cap.is_clean(),
            "N == depth + 2 ({CAP}): every stamp fits under the cap, \
             including the oldest, so step 1's fire finds a legal \
             (cam=EPOCH, lidar=EPOCH) transversal: {at_cap:?}"
        );

        let over_cap = drive(CAP + 1);
        assert_eq!(
            over_cap.findings.len(),
            1,
            "N == depth + 3 ({}): the oldest stamp is CUT (only the newest \
             {CAP} of {} records survive), so step 1's fire finds no legal \
             transversal and CONVICTS: {:?}",
            CAP + 1,
            CAP + 1,
            over_cap.findings
        );
        assert_eq!(over_cap.findings[0].step, 1);
        assert_eq!(over_cap.findings[0].expected, StepFires::none());
    }

    /// **The collapse × carry shape: a fire that consumed NOTHING
    /// must not cost an input its held head.**
    ///
    /// On a node with a non-trigger input a recorded fire can be a tick chain
    /// that COLLAPSED on the context input before reading any trigger head —
    /// nothing is consumed, every head is re-offered next boundary. Step 0:
    /// `cam` arrives twice (a head `H` and a parked `next_head` `P`, 300 ms
    /// apart) and `lidar` once, in window with `H`; the node "fires" once (the
    /// collapsed tick). Step 1: `lidar` arrives again, in window with `H` and
    /// NOT with `P`, and the node fires for real on `(H, lidar')`.
    ///
    /// A carry that credited the step-0 fire with consuming ONE head and
    /// kept `max(supply − k, 1) = 1` survivor — the LAST stamp, `P` — so step
    /// 1's candidates held `P` alone, arm B found no in-window tuple and
    /// convicted a byte-faithful recording (exit 6). Crediting a fire on such a
    /// node with consuming ZERO heads keeps both `H` and `P`, and the real fire
    /// is feasible on `H`. CONTROL: with every input trigger-marked the fire
    /// provably consumed `H`, so step 1's real fire IS infeasible and the
    /// finding stands — the widening is exactly as wide as the collapse.
    #[test]
    fn a_collapsed_fire_does_not_cost_a_widened_node_its_held_head() {
        let trigger_only = node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        );
        let with_context = NodeDecl {
            has_non_trigger_inputs: true,
            ..trigger_only.clone()
        };
        let drain = |input: &str, stamp: u64| {
            read_with_role(
                "fusion",
                input,
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(stamp),
                ReadSiteRole::Drain,
            )
        };
        let steps = vec![
            RecordedStep {
                step: 0,
                clock_ns: EPOCH,
                fires: vec![fire("fusion", EPOCH)],
                reads: vec![
                    drain("cam", EPOCH),            // H
                    drain("cam", EPOCH + 300 * MS), // P, parked
                    drain("lidar", EPOCH + 5 * MS),
                ],
            },
            RecordedStep {
                step: 1,
                clock_ns: EPOCH + 10 * MS,
                fires: vec![fire("fusion", EPOCH + 10 * MS)],
                reads: vec![drain("lidar", EPOCH + 20 * MS)],
            },
        ];

        let widened = verify_unrestored(
            std::slice::from_ref(&with_context),
            &steps,
            &[],
            RoleTrust::Believed,
        );
        assert!(
            widened.is_clean(),
            "the collapsed fire consumed nothing, so `H` is still a candidate and step 1's \
             real fire on (H, lidar') is feasible: {widened:?}"
        );
        assert_eq!(
            widened.carried_heads,
            vec![CarriedHeadsNote {
                node_id: "fusion".to_string()
            }]
        );

        let narrow = verify_unrestored(
            std::slice::from_ref(&trigger_only),
            &steps,
            &[],
            RoleTrust::Believed,
        );
        assert!(!narrow.is_clean(), "CONTROL: a real fire consumed `H`");
        assert_eq!(narrow.findings[0].step, 1);
        assert_eq!(narrow.findings[0].expected, StepFires::none());
    }

    /// **A node the Sync rule STOOD DOWN on carries no
    /// carried-heads note.**
    ///
    /// The note says "judged on widened evidence"; a stand-down says "not
    /// judged". A `Truncated` marker on a trigger input stands the node down
    /// mid-loop, and the note pushed before the loop must be withdrawn with it
    /// — CONTROL: the same declaration over an intact log carries the note.
    #[test]
    fn a_stood_down_node_carries_no_carried_heads_note() {
        let n = NodeDecl {
            has_non_trigger_inputs: true,
            ..node(
                "fusion",
                TriggerDecl::Sync {
                    window_ns: Some(50 * MS),
                    trigger_inputs: inputs(&["cam", "lidar"]),
                },
            )
        };
        let drain = |input: &str, stamp: u64| {
            read_with_role(
                "fusion",
                input,
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(stamp),
                ReadSiteRole::Drain,
            )
        };
        let truncated = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: Vec::new(),
            reads: vec![
                read_with_role(
                    "fusion",
                    "cam",
                    ReadOutcomeKind::Truncated,
                    0,
                    None,
                    ReadSiteRole::Drain,
                ),
                drain("lidar", EPOCH + 5 * MS),
            ],
        }];
        let stood_down = verify_unrestored(
            std::slice::from_ref(&n),
            &truncated,
            &[],
            RoleTrust::Believed,
        );
        assert_eq!(
            stood_down
                .stand_downs
                .iter()
                .map(|s| s.reason.clone())
                .collect::<Vec<_>>(),
            vec![StandDownReason::TruncatedReadLog]
        );
        assert!(
            stood_down.carried_heads.is_empty(),
            "a node that was not judged was not judged on carried heads: {:?}",
            stood_down.carried_heads
        );

        let intact = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("fusion", EPOCH)],
            reads: vec![drain("cam", EPOCH), drain("lidar", EPOCH + 5 * MS)],
        }];
        let judged = verify_unrestored(std::slice::from_ref(&n), &intact, &[], RoleTrust::Believed);
        assert!(judged.stand_downs.is_empty());
        assert_eq!(
            judged.carried_heads.len(),
            1,
            "CONTROL: judged, and says so"
        );
    }

    /// **A stamp-less WIDE-only (`Peek`) record stands the node
    /// down, exactly like a stamp-less head.**
    ///
    /// The stamp check is asked over the WIDE set: a peek is a unit of SUPPLY,
    /// and dropping one silently lowers supply and narrows the candidates —
    /// both in the CONVICTING direction. A variant that moved the `wire_stamp_ns`
    /// check inside `is_sync_head_record` would pass every head-only arm while
    /// a stamp-less peek vanished from supply; this arm carries one.
    #[test]
    fn a_stamp_less_peek_stands_the_node_down_rather_than_lowering_supply() {
        let n = node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        );
        let steps = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("fusion", EPOCH)],
            reads: vec![
                read_with_role(
                    "fusion",
                    "cam",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH),
                    ReadSiteRole::Drain,
                ),
                // The peek: WIDE-only, and stamp-less.
                read_with_role(
                    "fusion",
                    "cam",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    None,
                    ReadSiteRole::Peek,
                ),
                read_with_role(
                    "fusion",
                    "lidar",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH + 2 * MS),
                    ReadSiteRole::Drain,
                ),
            ],
        }];
        let report = verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Believed);
        assert_eq!(
            report
                .stand_downs
                .iter()
                .map(|s| s.reason.clone())
                .collect::<Vec<_>>(),
            vec![StandDownReason::MissingWireStamp],
            "the stamp-less PEEK is refused, not dropped: {report:?}"
        );
        assert!(report.findings.is_empty());
    }

    /// The FEASIBILITY sweep against a hand-written oracle, apart from the
    /// engine that calls it.
    ///
    /// Six shapes, each a different way the sweep can be wrong:
    /// EMPTY (nothing declared — vacuous, like the core's own rule), ONE INPUT,
    /// a pair that aligns only on a NON-LAST choice (the whole reason a
    /// last-wins fold is wrong), a pair that aligns on NOTHING, the exact
    /// window boundary on BOTH sides (inclusivity, single-sourced through
    /// [`sync_aligned`]), and UNBOUNDED (presence alone).
    #[test]
    fn the_transversal_sweep_answers_one_hand_oracle() {
        let cand = |pairs: &[(&str, &[u64])]| {
            pairs
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_vec()))
                .collect::<BTreeMap<String, Vec<u64>>>()
        };
        let w = Some(50 * MS);

        // EMPTY declaration: no spread to judge — vacuously feasible, exactly
        // as `cerulion_core::scheduler::sync_aligned` answers it.
        assert!(in_window_transversal(&cand(&[]), &[], w));

        // ONE input: any single candidate is a tuple of one, spread 0.
        assert!(in_window_transversal(
            &cand(&[("a", &[EPOCH + 900 * MS])]),
            &inputs(&["a"]),
            w
        ));
        // …and a declared input with NO candidate fails the presence rule.
        assert!(!in_window_transversal(&cand(&[]), &inputs(&["a"]), w));
        assert!(!in_window_transversal(
            &cand(&[("a", &[])]),
            &inputs(&["a"]),
            w
        ));

        // ALIGNS ON A NON-LAST CHOICE: the last candidate of each input is a
        // full second from the other, but the FIRST pair is 20 ms apart. This
        // is the per-set burst's own shape — the refill that came up
        // Incomplete lands last — and it is what a last-wins fold gets wrong.
        assert!(in_window_transversal(
            &cand(&[
                ("cam", &[EPOCH, EPOCH + 1000 * MS]),
                ("lidar", &[EPOCH + 20 * MS, EPOCH + 2000 * MS]),
            ]),
            &inputs(&["cam", "lidar"]),
            w
        ));
        // …and the mirror image, where only the LAST pair fits: the sweep must
        // not be biased toward early candidates either.
        assert!(in_window_transversal(
            &cand(&[
                ("cam", &[EPOCH, EPOCH + 1000 * MS]),
                ("lidar", &[EPOCH + 500 * MS, EPOCH + 1010 * MS]),
            ]),
            &inputs(&["cam", "lidar"]),
            w
        ));

        // ALIGNS ON NOTHING: every cross pair is more than the window apart.
        assert!(!in_window_transversal(
            &cand(&[
                ("cam", &[EPOCH, EPOCH + 1000 * MS]),
                ("lidar", &[EPOCH + 400 * MS, EPOCH + 2000 * MS]),
            ]),
            &inputs(&["cam", "lidar"]),
            w
        ));

        // THE BOUNDARY, both sides — `<=` is inclusive.
        for (spread, feasible) in [(50 * MS - 1, true), (50 * MS, true), (50 * MS + 1, false)] {
            assert_eq!(
                in_window_transversal(
                    &cand(&[("a", &[EPOCH]), ("b", &[EPOCH + spread])]),
                    &inputs(&["a", "b"]),
                    w
                ),
                feasible,
                "spread {spread} against a {} ns window",
                50 * MS
            );
        }

        // UNBOUNDED: presence is the whole condition, however far apart.
        assert!(in_window_transversal(
            &cand(&[("a", &[EPOCH]), ("b", &[EPOCH + 9_000 * MS])]),
            &inputs(&["a", "b"]),
            None
        ));
        assert!(!in_window_transversal(
            &cand(&[("a", &[EPOCH])]),
            &inputs(&["a", "b"]),
            None
        ));
    }

    /// The candidate CAP takes the CONSERVATIVE arm: a verifier must not
    /// convict because it ran out of room.
    ///
    /// A step whose candidates overflow [`SYNC_CANDIDATE_CAP`] on an input
    /// SKIPS the feasibility arm entirely — the transversal is treated as
    /// EXISTING. The SUPPLY arm is unaffected, because it counts records rather
    /// than storing them, and this arm pins both halves in one body: the same
    /// over-cap step with an impossible FIRE COUNT is still caught.
    #[test]
    fn an_over_cap_candidate_set_declines_feasibility_but_keeps_its_supply_bound() {
        let n = node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        );
        // `cam` overflows the cap with stamps spread a full second apart, so no
        // tuple with `lidar` could ever fit the 1 ms window. Feasibility is
        // DECLINED rather than convicting.
        let over = SYNC_CANDIDATE_CAP + 1;
        let mut reads = Vec::new();
        for i in 0..over as u64 {
            reads.push(read_with_role(
                "fusion",
                "cam",
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(EPOCH + i * MS),
                ReadSiteRole::Drain,
            ));
        }
        reads.push(read_with_role(
            "fusion",
            "lidar",
            ReadOutcomeKind::DrainedBatch,
            1,
            Some(EPOCH + 9_000_000 * MS),
            ReadSiteRole::Drain,
        ));

        let one_fire = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("fusion", EPOCH)],
            reads: reads.clone(),
        }];
        assert!(
            verify_unrestored(
                std::slice::from_ref(&n),
                &one_fire,
                &[],
                RoleTrust::Believed,
            )
            .is_clean(),
            "the cap bit, so no feasibility claim is made — a verifier must not \
             convict because it ran out of room"
        );

        // The SUPPLY bound still bites on the same step: `lidar` supplied ONE
        // head, so two fires is arithmetic no schedule can produce.
        let two_fires = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("fusion", EPOCH), fire("fusion", EPOCH)],
            reads,
        }];
        let report = verify_unrestored(
            std::slice::from_ref(&n),
            &two_fires,
            &[],
            RoleTrust::Believed,
        );
        assert_eq!(report.findings.len(), 1, "{report:?}");
        assert_eq!(report.findings[0].expected, StepFires::count(1));
        assert_eq!(report.findings[0].recorded, StepFires::count(2));
    }

    /// Candidate-cap carry: a step whose records
    /// OVERFLOW the candidate cap still carries the REAL surviving head into
    /// the next step, not the stamp of the record that happened to land at the
    /// cap boundary.
    ///
    /// The feasibility vector stops retaining stamps at [`SYNC_CANDIDATE_CAP`],
    /// which is fine for the feasibility arm (it declines rather than
    /// convicts), but not for a cross-step carry read off THAT vector's
    /// last entry — with 1,025 records on `a` that carry is record 1,024's
    /// stamp while the head the matcher really kept is record 1,025's. A
    /// next-step `b` record aligning only with the real head then convicts a
    /// VALID trace with a Sync finding (exit 6) — the exact false-exit-6 class
    /// V exists to eliminate. Read that way the verdict is "step-1 Sync
    /// finding of expected zero versus one recorded fire".
    ///
    /// Driven at BOTH counts: exactly the cap (the carry was already right —
    /// the control that shows the arm is about the overflow, not the cap's
    /// existence) and cap + 1 (the discriminating case).
    #[test]
    fn an_over_cap_step_carries_the_real_surviving_head_not_the_cap_boundary_stamp() {
        const SEC: u64 = 1_000 * MS;
        let n = node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(MS),
                trigger_inputs: inputs(&["a", "b"]),
            },
        );
        for a_records in [SYNC_CANDIDATE_CAP, SYNC_CANDIDATE_CAP + 1] {
            // Step 0: `a` records a SECOND apart (no two within the 1 ms
            // window), `b` exactly as many heads as the burst clamp permits
            // fires — so the step's fire count is the maximum arm A allows,
            // feasibility declines on the over-cap count, and ONLY `a` carries
            // a head out of the step (its supply exceeds the fire count; `b`'s
            // equals it).
            let last_a = EPOCH + (a_records as u64 - 1) * SEC;
            let mut reads = Vec::new();
            for i in 0..a_records as u64 {
                reads.push(read_with_role(
                    "fusion",
                    "a",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH + i * SEC),
                    ReadSiteRole::Drain,
                ));
            }
            for i in 0..SYNC_BURST_MAX_SETS as u64 {
                reads.push(read_with_role(
                    "fusion",
                    "b",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH + i * MS),
                    ReadSiteRole::Drain,
                ));
            }
            let fires: Vec<RecordedFire> = (0..SYNC_BURST_MAX_SETS)
                .map(|_| fire("fusion", EPOCH))
                .collect();
            // Step 1: ONE `b` arrival stamped exactly at `a`'s REAL surviving
            // head, and one recorded fire. Feasible iff the carry is that head.
            let steps = vec![
                RecordedStep {
                    step: 0,
                    clock_ns: EPOCH,
                    fires,
                    reads,
                },
                RecordedStep {
                    step: 1,
                    clock_ns: EPOCH + SEC,
                    fires: vec![fire("fusion", EPOCH + SEC)],
                    reads: vec![read_with_role(
                        "fusion",
                        "b",
                        ReadOutcomeKind::DrainedBatch,
                        1,
                        Some(last_a),
                        ReadSiteRole::Drain,
                    )],
                },
            ];
            let report =
                verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Believed);
            assert!(
                report.is_clean(),
                "{a_records} records on `a` in one step: the carried head must be the LAST \
                 record's stamp ({last_a}), so the next step's `b` aligns with it and the \
                 recorded fire is valid — got {report:?}"
            );
        }
    }

    fn sync_fusion() -> NodeDecl {
        node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(MS),
                trigger_inputs: inputs(&["a", "b"]),
            },
        )
    }

    /// The cross-step carry is cut at the input's PHYSICAL ceiling —
    /// `depth + 2` (queue + frozen head + parked `next_head`) — on every
    /// node, pinned as a THRESHOLD on both sides (without the cut
    /// the ordinary branch carries `supply − k` heads
    /// unbounded, growing by pops minus fires on a fast-input windowed pair
    /// until it hits the room cap and silences arm B for the rest of the run).
    ///
    /// Step 0: `a` carries `depth + 2 + 8` heads a second apart, `b` one head
    /// aligned with `a`'s first, ONE fire — the fire spends `a[0]`, the cut
    /// keeps the NEWEST `depth + 2` (drop-oldest is what the matcher does).
    /// Step 1: `b` arrives stamped exactly at ONE `a` head with one recorded
    /// fire. Aligned with the OLDEST kept head ⇒ feasible ⇒ CLEAN; aligned
    /// with the head one older — one the scheduler had already dropped, since
    /// `depth + 2` newer pops followed it — ⇒ infeasible ⇒ the arm CONVICTS.
    /// Unbounding the ordinary branch (`supply − k` alone) keeps that stale
    /// head and reads the infeasible fire as feasible — the failure this pins.
    #[test]
    fn the_carry_is_cut_at_the_inputs_physical_ceiling_on_every_node() {
        const SEC: u64 = 1_000 * MS;
        let default_depth = cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH as u64;
        // Two capacities: the unnamed default (one connection at the default
        // depth) and a NAMED capacity twice that (the two-writer bus shape —
        // a ceiling read from the declared depth alone
        // dropped the second connection's heads and convicted a healthy
        // replay). A cut that ignored the field would pass the first and fail
        // the second's CLEAN side.
        for (label, capacity, named) in [
            ("default", default_depth, false),
            ("two-connection", 2 * default_depth, true),
        ] {
            let mut n = sync_fusion();
            if named {
                n.trigger_input_capacities =
                    [("a".to_string(), capacity as u32)].into_iter().collect();
            }
            let ceiling = capacity + 2;
            let a_records = ceiling + 8;
            // After the fire spends `a[0]`, the newest `ceiling` heads are
            // `a[a_records - ceiling ..]`; the oldest KEPT head is the first of
            // them, and the one before it is the newest DROPPED head.
            let oldest_kept = a_records - ceiling;
            for (aligned_with, clean) in [(oldest_kept, true), (oldest_kept - 1, false)] {
                let mut reads = Vec::new();
                for i in 0..a_records {
                    reads.push(read_with_role(
                        "fusion",
                        "a",
                        ReadOutcomeKind::DrainedBatch,
                        1,
                        Some(EPOCH + i * SEC),
                        ReadSiteRole::Drain,
                    ));
                }
                reads.push(read_with_role(
                    "fusion",
                    "b",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH),
                    ReadSiteRole::Drain,
                ));
                let steps = vec![
                    RecordedStep {
                        step: 0,
                        clock_ns: EPOCH,
                        fires: vec![fire("fusion", EPOCH)],
                        reads,
                    },
                    RecordedStep {
                        step: 1,
                        clock_ns: EPOCH + a_records * SEC,
                        fires: vec![fire("fusion", EPOCH + a_records * SEC)],
                        reads: vec![read_with_role(
                            "fusion",
                            "b",
                            ReadOutcomeKind::DrainedBatch,
                            1,
                            Some(EPOCH + aligned_with * SEC),
                            ReadSiteRole::Drain,
                        )],
                    },
                ];
                let report =
                    verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Believed);
                assert_eq!(
                report.is_clean(),
                clean,
                "[{label}] a partner aligned with a[{aligned_with}] (ceiling {ceiling}, oldest \
                 kept a[{oldest_kept}]): clean={clean} expected: {:?}",
                report.findings
            );
                if !clean {
                    assert!(
                        report
                            .findings
                            .iter()
                            .any(|f| f.step == 1 && f.decider == Decider::Sync),
                        "[{label}] the conviction is the step-1 feasibility finding: {:?}",
                        report.findings
                    );
                }
            }
        }
    }

    /// The carry ROOM is a THRESHOLD pinned on both sides: a capacity
    /// whose ceiling is EXACTLY the room is judged — the cut keeps every head
    /// and an out-of-window fire on the next step CONVICTS (the carried 512
    /// plus one arrival is well under the candidate cap, so arm B runs) —
    /// while one head past the room STANDS DOWN with
    /// `queue_exceeds_verifier_room` and emits NO finding: the verifier says
    /// it cannot judge, never "clean". A clamp at the candidate cap
    /// would fill the vector at 1024 carried heads and accept the
    /// same fire.
    #[test]
    fn a_queue_past_the_carry_room_stands_down_and_one_at_the_room_is_judged() {
        const SEC: u64 = 1_000 * MS;
        for (capacity, judged) in [
            ((SYNC_CARRY_ROOM - 2) as u32, true),
            ((SYNC_CARRY_ROOM - 1) as u32, false),
        ] {
            let mut n = sync_fusion();
            n.trigger_input_capacities = [("a".to_string(), capacity)].into_iter().collect();
            // Step 0: `a` carries `capacity + 2` heads a second apart (exactly
            // the ceiling, so the cut drops nothing and every head is a
            // candidate next step), `b` one head aligned with `a[0]`, one
            // fire. Step 1: `b` arrives 500 ms from EVERY `a` head — no pair
            // inside the 1 ms window — with one recorded fire.
            let a_records = capacity as u64 + 3;
            let mut reads = Vec::new();
            for i in 0..a_records {
                reads.push(read_with_role(
                    "fusion",
                    "a",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH + i * SEC),
                    ReadSiteRole::Drain,
                ));
            }
            reads.push(read_with_role(
                "fusion",
                "b",
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(EPOCH),
                ReadSiteRole::Drain,
            ));
            let t1 = EPOCH + a_records * SEC;
            let steps = vec![
                RecordedStep {
                    step: 0,
                    clock_ns: EPOCH,
                    fires: vec![fire("fusion", EPOCH)],
                    reads,
                },
                RecordedStep {
                    step: 1,
                    clock_ns: t1,
                    fires: vec![fire("fusion", t1)],
                    reads: vec![read_with_role(
                        "fusion",
                        "b",
                        ReadOutcomeKind::DrainedBatch,
                        1,
                        Some(EPOCH + 500 * MS),
                        ReadSiteRole::Drain,
                    )],
                },
            ];
            let report =
                verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Believed);
            let stood_down = report
                .stand_downs
                .iter()
                .any(|s| s.reason == StandDownReason::QueueExceedsVerifierRoom);
            if judged {
                assert!(
                    !stood_down,
                    "capacity {capacity}: a ceiling AT the room is judged, not stood down"
                );
                assert!(
                    report
                        .findings
                        .iter()
                        .any(|f| f.step == 1 && f.decider == Decider::Sync),
                    "capacity {capacity}: the out-of-window fire convicts — arm B still runs \
                     with {SYNC_CARRY_ROOM} carried heads: {:?}",
                    report.findings
                );
            } else {
                assert!(
                    stood_down,
                    "capacity {capacity}: one head past the room stands the node down: {:?}",
                    report.stand_downs
                );
                assert!(
                    report.is_clean(),
                    "…and emits NO finding (no claim, not a clean claim): {:?}",
                    report.findings
                );
            }
        }
    }

    /// The room stand-down is decided
    /// BEFORE the first step is judged. Decided at the carry cut — which only a
    /// step that passed arms B, A and C reaches — an over-room node whose FIRST
    /// step convicted would be judged after all, and the operator would see an exit-6
    /// verdict on a node the rule calls unjudgeable. The shape: one head on
    /// each input, TWO recorded fires on step 0 — arm A's supply ceiling
    /// convicts it on step 0 under any capacity (the control at `room − 2`
    /// proves the shape convicts); at `room − 1` it must stand down with NO
    /// finding, no carried-heads note, and nothing else in the report.
    #[test]
    fn an_over_room_node_is_stood_down_before_its_first_step_is_judged() {
        for (capacity, judged) in [
            ((SYNC_CARRY_ROOM - 2) as u32, true),
            ((SYNC_CARRY_ROOM - 1) as u32, false),
        ] {
            let mut n = sync_fusion();
            n.trigger_input_capacities = [("a".to_string(), capacity)].into_iter().collect();
            let steps = vec![RecordedStep {
                step: 0,
                clock_ns: EPOCH,
                fires: vec![fire("fusion", EPOCH), fire("fusion", EPOCH)],
                reads: vec![
                    read_with_role(
                        "fusion",
                        "a",
                        ReadOutcomeKind::DrainedBatch,
                        1,
                        Some(EPOCH),
                        ReadSiteRole::Drain,
                    ),
                    read_with_role(
                        "fusion",
                        "b",
                        ReadOutcomeKind::DrainedBatch,
                        1,
                        Some(EPOCH),
                        ReadSiteRole::Drain,
                    ),
                ],
            }];
            let report =
                verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Believed);
            let room_stand_downs = report
                .stand_downs
                .iter()
                .filter(|s| s.reason == StandDownReason::QueueExceedsVerifierRoom)
                .count();
            if judged {
                assert_eq!(
                    room_stand_downs, 0,
                    "capacity {capacity}: under the room, judged"
                );
                assert!(
                    report
                        .findings
                        .iter()
                        .any(|f| f.step == 0 && f.decider == Decider::Sync),
                    "capacity {capacity}: two fires on one head convict on step 0 (the control \
                     that proves the shape convicts before any carry cut): {:?}",
                    report.findings
                );
            } else {
                assert_eq!(
                    room_stand_downs, 1,
                    "capacity {capacity}: one head past the room stands the node down ONCE, \
                     before step 0 is judged: {:?}",
                    report.stand_downs
                );
                assert!(
                    report.is_clean(),
                    "…and the step-0 conviction never happens (no claim): {:?}",
                    report.findings
                );
                assert!(
                    report.carried_heads.is_empty(),
                    "a node that was never judged carries no carried-heads note: {:?}",
                    report.carried_heads
                );
            }
        }
    }

    /// The candidate cap is a THRESHOLD, pinned on both sides: at EXACTLY the
    /// cap the feasibility arm still RUNS (and convicts an infeasible set); one
    /// record past it the arm DECLINES. A `<=` in place of the `<` — or a cap
    /// consulted off by one — moves this boundary, and the over-cap sibling
    /// above cannot see which side it landed on.
    #[test]
    fn the_candidate_cap_is_a_threshold_pinned_on_both_sides() {
        let n = node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        );
        for (cam_records, convicts) in [(SYNC_CANDIDATE_CAP, true), (SYNC_CANDIDATE_CAP + 1, false)]
        {
            let mut reads = Vec::new();
            for i in 0..cam_records as u64 {
                reads.push(read_with_role(
                    "fusion",
                    "cam",
                    ReadOutcomeKind::DrainedBatch,
                    1,
                    Some(EPOCH + i * MS),
                    ReadSiteRole::Drain,
                ));
            }
            // `lidar` a full second past every `cam` stamp: no tuple fits 1 ms.
            reads.push(read_with_role(
                "fusion",
                "lidar",
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(EPOCH + 9_000_000 * MS),
                ReadSiteRole::Drain,
            ));
            let steps = vec![RecordedStep {
                step: 0,
                clock_ns: EPOCH,
                fires: vec![fire("fusion", EPOCH)],
                reads,
            }];
            let report =
                verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Believed);
            if convicts {
                assert_eq!(
                    report.findings.len(),
                    1,
                    "{cam_records} records is AT the cap: feasibility runs and convicts — \
                     {report:?}"
                );
                assert_eq!(report.findings[0].expected, StepFires::none());
                assert_eq!(report.findings[0].recorded, StepFires::count(1));
            } else {
                assert!(
                    report.is_clean(),
                    "{cam_records} records is PAST the cap: feasibility declines — {report:?}"
                );
            }
        }
    }

    /// Arm B under-approximation: a step that ends with an input
    /// holding a head AND a parked `next_head` carries BOTH into the next step,
    /// so the fire the deferred set finally takes is still feasible.
    ///
    /// The shape is the first-fire defer: the boundary align
    /// fills `cam@1000 ms` / `lidar@1010 ms` (in window), the descent peeks
    /// `cam`'s next frame `@5000 ms` into `next_head` (a `Peek` record) and
    /// answers `Fire`, and `tick_sync_burst`'s pre-fire check DEFERS it
    /// (`throttle_ms` — a `block` input is the other door). Nothing fires,
    /// nothing is promoted: `cam` ends the step with 1000 held and 5000 parked.
    /// Next step the throttle clears and the node fires the (1000, 1010) set;
    /// the post-fire refill promotes 5000, either SILENTLY
    /// (`drain_for_trigger`'s R-promote) or with `sync_discard_head`'s
    /// `Drain`-at-`popped: 0` record — both legs are driven, because which one
    /// the recording carries depends on the matcher's path and the verifier
    /// must be right on both.
    ///
    /// A narrow carry (one `bool` + one stamp per
    /// input, the stamp being the step's LAST wide record) fails this: step 1's
    /// candidates are `cam = [5000]`, `lidar = [1010]`, no tuple fits 50 ms,
    /// `ScheduleFinding { expected: none }` — exit 6 on a healthy bag. The
    /// third step is the one-BIT half of the same defect: two frames held
    /// across a boundary count as ONE unit of supply, so after the fire
    /// consumes one, `cam` carries nothing and the fire on the promoted 5000
    /// (paired with `lidar@5010`) is infeasible too.
    #[test]
    fn a_head_beside_a_parked_peek_survives_a_deferred_fire_into_the_next_step() {
        let n = NodeDecl {
            throttle_ns: Some(20 * MS),
            ..node(
                "fuse",
                TriggerDecl::Sync {
                    window_ns: Some(50 * MS),
                    trigger_inputs: inputs(&["cam", "lidar"]),
                },
            )
        };
        let drain = |input: &str, at: u64, popped: u32| {
            read_with_role(
                "fuse",
                input,
                ReadOutcomeKind::DrainedBatch,
                popped,
                Some(EPOCH + at),
                ReadSiteRole::Drain,
            )
        };
        let peek = |input: &str, at: u64| {
            read_with_role(
                "fuse",
                input,
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(EPOCH + at),
                ReadSiteRole::Peek,
            )
        };
        for recorded_promotion in [false, true] {
            let mut step1_reads = Vec::new();
            if recorded_promotion {
                step1_reads.push(drain("cam", 5000 * MS, 0));
            }
            let steps = vec![
                // The deferred step: aligned, peeked, NOT fired.
                RecordedStep {
                    step: 0,
                    clock_ns: EPOCH,
                    fires: vec![],
                    reads: vec![
                        drain("cam", 1000 * MS, 1),
                        drain("lidar", 1010 * MS, 1),
                        peek("cam", 5000 * MS),
                    ],
                },
                // The throttle clears: the (1000, 1010) set fires, the parked
                // 5000 is promoted.
                RecordedStep {
                    step: 1,
                    clock_ns: EPOCH + 5 * MS,
                    fires: vec![fire("fuse", EPOCH + 5 * MS)],
                    reads: step1_reads,
                },
                // The promoted head fires with lidar's next frame — a full
                // throttle interval later, so the throttle rule (judged on
                // every policy) has nothing to say.
                RecordedStep {
                    step: 2,
                    clock_ns: EPOCH + 30 * MS,
                    fires: vec![fire("fuse", EPOCH + 30 * MS)],
                    reads: vec![drain("lidar", 5010 * MS, 1)],
                },
            ];
            let report =
                verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Believed);
            assert!(
                report.is_clean(),
                "recorded_promotion={recorded_promotion}: the head beside the parked peek \
                 is a carried candidate and a unit of supply, so both later fires are \
                 legal — {report:?}"
            );
            // The throttle stands the OWED-FIRE arm down (report-only), which is
            // exactly why arms A and B stay live on this node and had to be right.
            assert!(
                report
                    .stand_downs
                    .iter()
                    .any(|s| s.reason == StandDownReason::SuppressibleFire),
                "{report:?}"
            );
        }
    }

    /// Arm C phantom: a head the scheduler SPENT silently (a
    /// `DiscardTie` whose advance found an empty queue) is not credited as
    /// supply on the input's next fired step, so no phantom survives that fire
    /// to convict the quiet step after it.
    ///
    /// The four-step vector, with the scheduler's own moves beside it:
    ///
    /// | step | records | k | scheduler |
    /// |---|---|---|---|
    /// | 0 | `cam@0` | 0 | cam filled, lidar empty — `Wait` |
    /// | 1 | `lidar@200 ms` | 0 | (0, 200) out of window — `DiscardTie(cam)`, cam's queue empty — SPENT, no record |
    /// | 2 | `cam@210 ms` | 1 | cam refilled, (210, 200) fires — both tombstoned; refills find nothing |
    /// | 3 | `lidar@215 ms` | 0 | cam `Emitted` — `Wait` |
    ///
    /// A narrow carry fails this: step 1 leaves `cam@0`
    /// standing (out of window, harmless by monotonicity while cam stays
    /// quiet), step 2's supply rule reads it as carried-in (`1 + 1 = 2 > 1`)
    /// and KEEPS `cam@210` past the fire that consumed it, and step 3 then reads
    /// `(210, 215)` as an owed fire: `ScheduleFinding { expected: count(1),
    /// recorded: none }` — exit 6 on a recording the scheduler executed
    /// correctly. The robot shape is two sensors at different rates with ONE
    /// late (or lost) frame on the slower input.
    #[test]
    fn a_head_silently_spent_by_a_discard_tie_is_not_credited_on_the_next_fired_step() {
        let n = node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        );
        let drain = |input: &str, at: u64| {
            read_with_role(
                "fusion",
                input,
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(EPOCH + at),
                ReadSiteRole::Drain,
            )
        };
        let steps = vec![
            RecordedStep {
                step: 0,
                clock_ns: EPOCH,
                fires: vec![],
                reads: vec![drain("cam", 0)],
            },
            RecordedStep {
                step: 1,
                clock_ns: EPOCH + 5 * MS,
                fires: vec![],
                reads: vec![drain("lidar", 200 * MS)],
            },
            RecordedStep {
                step: 2,
                clock_ns: EPOCH + 10 * MS,
                fires: vec![fire("fusion", EPOCH + 10 * MS)],
                reads: vec![drain("cam", 210 * MS)],
            },
            RecordedStep {
                step: 3,
                clock_ns: EPOCH + 15 * MS,
                fires: vec![],
                reads: vec![drain("lidar", 215 * MS)],
            },
        ];
        let report = verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Believed);
        assert!(
            report.is_clean(),
            "the spent head must not survive the step-2 fire as a phantom — {report:?}"
        );

        // ANTI-TAUTOLOGY: the spend model drops ONLY the spent input. Make
        // step 2 fire NOTHING — cam's fresh fill (210) beside lidar's retained
        // 200 is then a genuinely aligned-and-unfired set, and arm C convicts.
        let mut owed = steps.clone();
        owed[2].fires.clear();
        owed.truncate(3);
        let report = verify_unrestored(std::slice::from_ref(&n), &owed, &[], RoleTrust::Believed);
        assert_eq!(report.findings.len(), 1, "{report:?}");
        assert_eq!(report.findings[0].step, 2);
        assert_eq!(report.findings[0].expected, StepFires::count(1));
        assert_eq!(report.findings[0].recorded, StepFires::none());
    }

    /// The same spend, reached from a FIRED step: the post-fire refill fills
    /// `cam@300 ms` / `lidar@10 ms`, out of window, and the `DiscardTie` on
    /// lidar finds its queue empty. The step's own carry rule keeps both refill
    /// stamps (`2 > 1` on each input), so without the spend model `lidar@10`
    /// stood as a phantom, was credited on step 1's fire (`1 + 1 = 2 > 1`,
    /// keeping `lidar@305` past the fire that consumed it), and step 2's
    /// `cam@310` then read as an owed fire beside it. The model runs at EVERY
    /// step's end, not only on `k == 0` steps, and this is the arm that says so.
    #[test]
    fn a_discard_tie_spend_after_a_fire_leaves_no_phantom_for_the_next_quiet_step() {
        let n = node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        );
        let drain = |input: &str, at: u64| {
            read_with_role(
                "fusion",
                input,
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(EPOCH + at),
                ReadSiteRole::Drain,
            )
        };
        let steps = vec![
            // Fill (0, 5), fire, refill (300, 10): out of window, lidar spent.
            RecordedStep {
                step: 0,
                clock_ns: EPOCH,
                fires: vec![fire("fusion", EPOCH)],
                reads: vec![
                    drain("cam", 0),
                    drain("lidar", 5 * MS),
                    drain("cam", 300 * MS),
                    drain("lidar", 10 * MS),
                ],
            },
            // lidar's next frame completes (300, 305): fire.
            RecordedStep {
                step: 1,
                clock_ns: EPOCH + 5 * MS,
                fires: vec![fire("fusion", EPOCH + 5 * MS)],
                reads: vec![drain("lidar", 305 * MS)],
            },
            // cam's next frame: lidar is spent, nothing to align — no fire.
            RecordedStep {
                step: 2,
                clock_ns: EPOCH + 10 * MS,
                fires: vec![],
                reads: vec![drain("cam", 310 * MS)],
            },
        ];
        let report = verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Believed);
        assert!(report.is_clean(), "{report:?}");
    }

    /// The two site predicates ask DIFFERENT questions, and this is the table
    /// that says so — the anti-tautology for defining one in terms of the
    /// other.
    ///
    /// A `Peek` is a drain-SITE read (it popped the queue, and its record is
    /// the only place that pop is accounted) and is NOT a head record (the
    /// frame was parked, not fired on). Collapsing the two either way is a real
    /// failure: making `is_drain_site_read` refuse a peek loses a pop from the
    /// FIFO sum and from `block` credit; making `is_sync_head_record` admit one
    /// restores the false `ScheduleFinding`.
    #[test]
    fn the_head_predicate_is_strictly_narrower_than_the_site_predicate() {
        // (role, roles_stamped, is_drain_site, is_sync_head)
        let table = [
            (ReadSiteRole::Drain, true, true, true),
            (ReadSiteRole::Peek, true, true, false),
            (ReadSiteRole::Body, true, false, false),
            // Unbelieved bits take the KIND arm at BOTH predicates, so the two
            // answers coincide — which is what keeps an archived bag reading
            // exactly as it did.
            (ReadSiteRole::Unstamped, true, true, true),
            (ReadSiteRole::Peek, false, true, true),
            (ReadSiteRole::Body, false, true, true),
        ];
        for (role, stamped, site, head) in table {
            let r = read_with_role(
                "n",
                "i",
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(EPOCH),
                role,
            );
            let trust = RoleTrust::from_stamped(stamped);
            assert_eq!(
                is_drain_site_read(&r, trust),
                site,
                "is_drain_site_read({role:?}, roles_stamped={stamped})"
            );
            assert_eq!(
                is_sync_head_record(&r, trust),
                head,
                "is_sync_head_record({role:?}, roles_stamped={stamped})"
            );
            assert!(
                !head || site,
                "the head predicate must be STRICTLY narrower: {role:?}"
            );
        }
        // …and a non-`DrainedBatch` kind is neither, under either arm.
        for kind in [
            ReadOutcomeKind::Served,
            ReadOutcomeKind::Held,
            ReadOutcomeKind::NoFrame,
            ReadOutcomeKind::Decimated,
            ReadOutcomeKind::Truncated,
        ] {
            let r = read_with_role("n", "i", kind, 1, Some(EPOCH), ReadSiteRole::Drain);
            assert!(!is_drain_site_read(&r, RoleTrust::Believed), "{kind:?}");
            assert!(!is_sync_head_record(&r, RoleTrust::Believed), "{kind:?}");
        }
    }

    /// H2(i): the SYNC arm of the read-site census.
    ///
    /// Two inputs, so the census also pins the DECLARATION order a one-input
    /// fixture cannot see. `cam` is body-heavy (1 drain, 3 body) and the pair
    /// is unaligned, so the Sync rule produces a finding derived from the drain
    /// record alone — exactly the misattribution the census exists to explain.
    #[test]
    fn a_body_heavy_sync_finding_carries_its_read_site_census() {
        let n = node(
            "fusion",
            TriggerDecl::Sync {
                window_ns: Some(50 * MS),
                trigger_inputs: inputs(&["cam", "lidar"]),
            },
        );
        let cam = |role: ReadSiteRole, stamp: u64| {
            read_with_role(
                "fusion",
                "cam",
                ReadOutcomeKind::DrainedBatch,
                1,
                Some(stamp),
                role,
            )
        };
        // ONE drain record per input per step,
        // spread over three steps to carry the body records.
        let steps = vec![
            RecordedStep {
                step: 0,
                clock_ns: EPOCH,
                fires: Vec::new(),
                reads: vec![
                    cam(ReadSiteRole::Drain, EPOCH),
                    cam(ReadSiteRole::Body, EPOCH + 400 * MS),
                    read_with_role(
                        "fusion",
                        "lidar",
                        ReadOutcomeKind::DrainedBatch,
                        1,
                        Some(EPOCH + 400 * MS),
                        ReadSiteRole::Drain,
                    ),
                ],
            },
            RecordedStep {
                step: 1,
                clock_ns: EPOCH + MS,
                fires: vec![fire("fusion", EPOCH + MS)],
                reads: vec![cam(ReadSiteRole::Body, EPOCH + 400 * MS)],
            },
            RecordedStep {
                step: 2,
                clock_ns: EPOCH + 2 * MS,
                fires: Vec::new(),
                reads: vec![cam(ReadSiteRole::Body, EPOCH + 400 * MS)],
            },
        ];
        let report = verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Believed);
        assert_eq!(
            report
                .findings
                .iter()
                .map(|f| f.decider)
                .collect::<Vec<_>>(),
            vec![Decider::Sync],
            "the drain-only fold leaves the pair unaligned on a step that fired"
        );
        assert_eq!(
            report.read_site_census,
            vec![ReadSiteCensus {
                node_id: "fusion".to_string(),
                decider: Decider::Sync,
                inputs: vec![
                    InputReadSites {
                        input: "cam".to_string(),
                        drain_records: 1,
                        body_records: 3,
                        drain_popped: 1,
                        body_popped: 3,
                    },
                    InputReadSites {
                        input: "lidar".to_string(),
                        drain_records: 1,
                        body_records: 0,
                        drain_popped: 1,
                        body_popped: 0,
                    },
                ],
            }],
            "the census covers EVERY trigger input, in declaration order"
        );
    }

    /// The census is body-heavy on EITHER scale — records or POPS.
    ///
    /// The rules it annotates sum `popped`, so three drain batches of one frame
    /// beside one body batch of a hundred is a stream whose RECORDS read 3-vs-1
    /// "drain-heavy" while the exclusion removed 100 of its 103 pops. A
    /// record-count-only test cannot see that.
    #[test]
    fn the_census_fires_when_the_pops_are_body_heavy_and_the_records_are_not() {
        let n = node(
            "sink",
            TriggerDecl::Data {
                trigger_inputs: inputs(&["inp"]),
            },
        );
        let batch = |popped: u32, role: ReadSiteRole| {
            read_with_role(
                "sink",
                "inp",
                ReadOutcomeKind::DrainedBatch,
                popped,
                Some(EPOCH),
                role,
            )
        };
        // 3 drain records popping 1 each; 1 body record popping 100. Records
        // 3-vs-1 (drain-heavy) but pops 3-vs-100 (body-heavy). The FIFO rule
        // expects 3 pops against 1 recorded fire ⇒ a finding.
        let steps = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("sink", EPOCH)],
            reads: vec![
                batch(1, ReadSiteRole::Drain),
                batch(1, ReadSiteRole::Drain),
                batch(1, ReadSiteRole::Drain),
                batch(100, ReadSiteRole::Body),
            ],
        }];
        let report = verify_unrestored(std::slice::from_ref(&n), &steps, &[], RoleTrust::Believed);
        assert_eq!(
            report.findings.len(),
            1,
            "the exclusion produces a finding: {:?}",
            report.findings
        );
        assert_eq!(
            report.read_site_census,
            vec![ReadSiteCensus {
                node_id: "sink".to_string(),
                decider: Decider::FifoPop,
                inputs: vec![InputReadSites {
                    input: "inp".to_string(),
                    drain_records: 3,
                    body_records: 1,
                    drain_popped: 3,
                    body_popped: 100,
                }],
            }],
            "record-heavy toward the drain, pop-heavy toward the body"
        );
        let rendered = report.read_site_census[0].to_string();
        assert!(
            rendered.contains("inp=records 3/1, pops 3/100"),
            "both scales render: {rendered}"
        );

        // BOUNDARY: an even split on BOTH scales is not body-heavy.
        let even = vec![RecordedStep {
            step: 0,
            clock_ns: EPOCH,
            fires: vec![fire("sink", EPOCH)],
            // 2 drain pops against 1 recorded fire ⇒ a finding to annotate;
            // records 1/1 and pops 2/2, so neither scale is body-heavy.
            reads: vec![batch(2, ReadSiteRole::Drain), batch(2, ReadSiteRole::Body)],
        }];
        let report = verify_unrestored(std::slice::from_ref(&n), &even, &[], RoleTrust::Believed);
        assert!(
            !report.findings.is_empty(),
            "the finding is still there to annotate"
        );
        assert!(
            report.read_site_census.is_empty(),
            "an even split is not body-heavy: {:?}",
            report.read_site_census
        );
    }
}
