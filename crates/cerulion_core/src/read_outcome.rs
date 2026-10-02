// SPDX-License-Identifier: AGPL-3.0-only
//! Per-edge READ-OUTCOME staging — the capture half of the
//! flow-mode read log.
//!
//! A [`ReadOutcomeStage`] is a small, fixed-capacity buffer shared (`Arc`)
//! between ONE graph-wired subscriber (the writer) and the scheduler's
//! level-end merge (the reader, on the step thread). The subscriber records
//! one [`StagedReadOutcome`] per read at its drain sites
//! (`transport::subscriber` — `snapshot_latest` / `snapshot_latest_for_trigger`
//! / `try_view`'s live arm / `drain_samples`); the merge drains every level's
//! stages, each step, into the recording trace ring as
//! `RECORD_TYPE_READ_OUTCOME` records (kind 6, `trace_format` 3 —
//! deliberately NOT an intra-doc link: this module is PORTABLE while
//! `trace_ring` is `#[cfg(unix)]`, so a link here is unresolvable on a
//! non-unix target and fails the `RUSTDOCFLAGS=-D warnings` docs gate —
//! the unix-only intra-doc-link class).
//!
//! # Why a shared cell and not a subscriber-owned `Vec`
//!
//! The subscriber lives inside its node's `NodeContext`, which `init()` moves
//! into the node entry (and, for a cdylib node, across the FFI) — the runtime
//! cannot reach it again without a new `NodeEntry` method on every entry form
//! (macro codegen included). The `Arc` handle is retained runtime-side at
//! WIRING time (before the context moves), the same pattern as
//! `NodeContext`'s `QosEventStore` / discard signals.
//!
//! # Synchronization
//!
//! There is exactly ONE writer at a time (a subscriber is drained either on
//! the step thread — snapshot phase, `drain_level` — or inside its own node's
//! tick, which the node `Mutex` serializes), and the reader runs on the step
//! thread strictly AFTER the level's rayon join. The interior `Mutex` exists
//! to make that discipline *sound* rather than *assumed*; it is uncontended
//! by construction, so a lock is a bare compare-exchange each way, paid ONLY
//! when a recording has armed the stage (`is_armed` gates every touch, and a
//! recording-off run never arms). This is the one deliberate deviation from
//! the "no locks" staging sketch: the lock-free alternative is an
//! `UnsafeCell` + a hand-argued happens-before, and the house bar for unsafe
//! is higher than an uncontended, recording-only lock.
//!
//! # Bounded by construction
//!
//! `record` never grows the buffer past its armed capacity: an overflowing
//! record is DROPPED and counted (`dropped` — an unconditional running total,
//! surfaced by a loud `warn!` at the next merge and RE-ANNOUNCED at each
//! DECADE of the total — Principle #3, never silent), so a stage that
//! accumulates more reads than one merge drains (a `Period` CATCH-UP burst on
//! an input the step freeze does not cover — a `block` input, or ANY input of
//! a node whose entry cannot freeze — staging one body read per catch-up fire,
//! all inside one level execution)
//! degrades loudly instead of allocating on the hot path. `Vec::push` within
//! capacity does not allocate, which is what keeps the recording-armed steady
//! state zero-alloc (the `step_zero_alloc_test` /
//! `trace_ring_hook_zero_alloc_test` discipline).
//!
//! # Residual: an overflow is MARKED, never silent
//!
//! Each stage's capacity is DERIVED from the graph facts of its own edge
//! ([`derive_stage_capacity`]) and sized for the level-end merge
//! cadence (one merge per step) with headroom over the deepest burst one level
//! execution can serve ([`READ_OUTCOME_STAGE_HEADROOM`]).
//!
//! One population source is bounded by no graph fact: a `Period` node's
//! catch-up burst reading an input that is NOT step-frozen. A step-FROZEN
//! input stages exactly ONE record per STEP however many times its node fires
//! (the boundary snapshot fills the context slot and `try_view` serves it IN
//! PLACE, running no accounting) — but freezing takes TWO things, and the
//! unfrozen set is correspondingly two classes wide:
//!
//! 1. **POLICY** — `build_snapshot_input_names` excludes `block`, because a
//!    `block` input's drain must stay on the consumer's tick to keep the
//!    producer-pacing mirror in lockstep.
//! 2. **CAPABILITY** — the node's ENTRY must be able to freeze
//!    (`NodeEntry::holds_input_snapshot`). A `ClosureNodeEntry` and a raw-FFI
//!    cdylib both inherit the default no-op `snapshot_inputs`, so EVERY plain
//!    input of such a node reads live on every fire, name-set membership
//!    notwithstanding.
//!
//! Such a stage declares
//! [`FireBurstBound::Unbounded`] and is armed at [`READ_OUTCOME_STAGE_MAX`]; a
//! catch-up storm deeper than that ceiling can still stage more reads between
//! merges than the stage holds (the run-length encoding is what
//! closes it). The
//! truncation is DETERMINISTIC on both the record and replay sides (both run
//! the same capacity, the same drop-newest-at-the-rim rule, and the same
//! merge cadence), so the verifier never false-alarms on an
//! overflowed edge.
//!
//! The HOLE is also self-describing: the drain
//! emits one OVERFLOW MARKER record ([`ReadOutcomeKind::Truncated`]) per
//! merge window in which this stage dropped anything, carrying the count
//! dropped IN THAT WINDOW — so an offline reader sees "k records are missing
//! here" instead of a silent gap at exactly the stall moments an operator
//! would want to inspect. A verdicting verifier could not otherwise call a
//! truncated edge "compared clean", which is why the marker is an
//! obligation rather than a nicety.
//!
//! **How replay classifies a TRUNCATED consumed frame's read.** A frame the
//! recording really CONSUMED, whose kind-6 record was dropped at the rim, is
//! read back as a MARKED HOLE, never as silence: the marker sits at the
//! position the dropped records would have occupied, so the offline read log
//! says "this edge consumed frames here whose outcomes were not retained".
//! The frame itself is unaffected (the read served it; only the diagnostic
//! record was lost), and the run-exit drop report
//! (`GraphRuntime::read_outcome_dropped_report`, surfaced beside the
//! trace-ring unmapped count) remains the run-level total.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

/// The SLOTS a stage keeps above the deepest burst one level
/// execution can hand it — the `+ H` term of [`derive_stage_capacity`], and
/// the one term of that derivation that is still a constant rather
/// than a graph fact.
///
/// # Derivation
///
/// Under the within-step burst a Data node fires up to
/// `DATA_PENDING_CARRY_CLAMP` times inside ONE level execution, and that
/// clamp is DEFINED as `MAX_CONSUMER_DEPTH` (with its own const-assert
/// pinning the identity in `scheduler/mod.rs`), so the deepest burst one
/// merge window can be asked to hold is exactly `MAX_CONSUMER_DEPTH`
/// records. At zero headroom the post-cap REFILL — the drain that runs after
/// the burst caps and finds the queue non-empty — stages a `clamp + 1`-th
/// record that drops at the rim, and that record is the HELD HEAD's, whose
/// re-offer records nothing: a frame the run CONSUMED with no kind-6
/// record at all. One slot is what makes the burst-plus-refill fit, so 1 is
/// the floor rather than a round number; more would be free, but the derived
/// capacity is a replay-compat format parameter (see
/// [`derive_stage_capacity`]) and every extra slot is one more byte of skew
/// surface for no behavioural gain.
///
/// It is multiplied by the PAIR factor along with the rest of the
/// bound, because the refill read it pays for is itself a pair on an annotated
/// edge — see [`derive_stage_capacity`].
pub const READ_OUTCOME_STAGE_HEADROOM: usize = 1;

/// The FLOOR of [`derive_stage_capacity`]'s clamp — the smallest
/// capacity any stage is ever armed with.
///
/// The floor is not decorative: an UNANNOTATED depth-0 edge derives an
/// unclamped 2 (`1 * (max(0 + 0, 1) + 1)`), which is the only shape that
/// reaches below it — a depth-1 latest-value input on a once-firing node
/// derives EXACTLY 4 and is clamped by arithmetic, not by this bound. Raising
/// that one shape from 2 to 4 costs 64 bytes and removes a class of off-by-one
/// rims from every arm that drives one. It is EVEN by construction, and so is
/// [`READ_OUTCOME_STAGE_MAX`], because the clamp must not break
/// `record_with_producer`'s atomic-pair admission: on an annotated edge a cap
/// that is odd refuses the last pair from a stage with one free slot, which is
/// a truncation the derivation never accounted for.
pub const READ_OUTCOME_STAGE_MIN: u32 = 4;

/// The CEILING of [`derive_stage_capacity`]'s clamp — 4096 records,
/// i.e. 128 KiB of staged records at `StagedReadOutcome`'s pinned 32-byte size.
///
/// A ceiling is necessary rather than defensive, for two independent reasons:
///
/// 1. **A node may DECLARE an arbitrary catch-up burst.** `max_catchup` is a
///    graph fact, so a node declaring `1_000_000` would otherwise put a
///    user-authored number straight into an allocation size.
/// 2. **A `Period` node that declares nothing is UNBOUNDED.** The state arm
///    clamps an undeclared `max_catchup` to `ARMED_MAX_CATCHUP_DEFAULT`, but
///    the arm's ONSET (`step >= first_anchor_step`) is a RUN fact and arm
///    creation can fail, while this derivation runs before step 0. So the
///    derivation ASSUMES THE WORST ([`FireBurstBound::Unbounded`]) and lands on
///    this ceiling rather than reading the state-arm plane — which keeps the
///    staging plane and the arm plane independent, so no RUN fact (a step
///    index, an arm's onset, a frame count) can move a capacity.
///
///    SCOPED, because "two runs of one graph always arm the same capacity" is
///    not true unconditionally: `sync_shape` is derived from
///    `per_set_capable`, which reads `CERULION_DRAIN_DISCIPLINE`, so the
///    widest shape derives 516 unset and 260 under `=separate`. That seam
///    already forks the RECORD STREAM — it is the verifier's own divergence
///    class 2 — so a bag and a binary that disagree about it are already
///    non-comparable for reasons that have nothing to do with staging.
///
/// The classes that reach the ceiling are a `Period` node's catch-up burst
/// reading an input the freeze does not cover — either a **`block`** input
/// (excluded by POLICY in `build_snapshot_input_names`, because its drain must
/// stay on the consumer's tick to keep the producer-pacing mirror in lockstep)
/// or **any** input of a node whose ENTRY cannot freeze (a `ClosureNodeEntry`
/// or a raw-FFI cdylib: no snapshot symbols, so `try_view` takes its
/// live arm on every fire). A genuinely FROZEN input stages exactly ONE record
/// per step however many times its node fires, so it declares
/// `FireBurstBound::Fires(1)` and never reaches here.
pub const READ_OUTCOME_STAGE_MAX: u32 = 4096;

// The clamp bounds must be ORDERED and EVEN — see each constant's doc. Both
// are const-asserted rather than commented, because `derive_stage_capacity`
// leans on evenness for the atomic-pair admission and on the ordering for the
// clamp to be a clamp at all.
//
// WHY THE TOTAL IS EVEN WHEREVER IT HAS TO BE. An ODD rim is
// reachable only on an UNANNOTATED edge (`pair == 1`), and an unannotated edge
// never calls `record_with_producer`, so the atomic pair never meets an odd
// rim. That is a COUPLING, not a coincidence: all three wiring sites compute
// `annotated` ONCE and drive BOTH the sizing and `mark_multi_publisher_edge`
// from that same value, so the bit that decides `pair == 2` is the same bit
// that decides whether pairs are ever staged. Splitting those two reads is the
// one edit that would make an odd rim reachable by a pair.
const _: () = assert!(
    READ_OUTCOME_STAGE_MIN <= READ_OUTCOME_STAGE_MAX
        && READ_OUTCOME_STAGE_MIN.is_multiple_of(2)
        && READ_OUTCOME_STAGE_MAX.is_multiple_of(2)
        && READ_OUTCOME_STAGE_MIN >= 2,
    "the read-outcome stage clamp bounds must be ordered and EVEN (an odd bound \
     would refuse `record_with_producer`'s atomic pair from a stage with one \
     free slot), and the floor must admit at least one pair"
);
/// Which READ PATH a stage observes. Under the Separate and
/// Sync disciplines ONE input carries TWO stages that deliberately share an
/// `input_idx` (the body subscriber's read + the runtime-owned trigger/sync
/// DRAIN subscriber's batch drain — one input name per index offline), so
/// the run-exit drop report needs this tag to attribute a drop to the read
/// path that actually overflowed.
// `Hash` because a stage is identified by the PAIR
// `(input_idx, role)`, and the replay's adoption map is keyed on it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReadStageRole {
    /// The node body's own subscriber (step-boundary snapshot / `try_view`).
    Body,
    /// A runtime-owned trigger/sync DRAIN subscriber (the second read path
    /// on the same input under the Separate/Sync disciplines).
    Drain,
}

impl ReadStageRole {
    /// The byte this role rides as in the ring manifest's
    /// capacity section.
    ///
    /// A stage is identified by `(input_idx, role)` and NOT by position — an
    /// input under the Separate or legacy-`Sync` discipline contributes TWO
    /// stages that share an index — so the role has to cross the wire.
    #[must_use]
    pub const fn wire(self) -> u8 {
        match self {
            Self::Body => 0,
            Self::Drain => 1,
        }
    }

    /// The inverse of [`Self::wire`]. An UNKNOWN byte is `None` rather than a
    /// guess: a capacity row this binary cannot place is a row it must not
    /// adopt, and the caller stands that edge down instead (the same posture
    /// `ReadSiteRole::from_wire` takes for its own unknown discriminants).
    #[must_use]
    pub const fn from_wire(byte: u8) -> Option<Self> {
        match byte {
            0 => Some(Self::Body),
            1 => Some(Self::Drain),
            _ => None,
        }
    }

    /// The stable label the run-exit drop report and its log lines carry.
    pub fn label(self) -> &'static str {
        match self {
            Self::Body => "body",
            Self::Drain => "drain",
        }
    }
}

/// The SKEW between the two role numberings, pinned so a
/// renumbering of either enum has to come here and read this.
///
/// [`ReadStageRole`] and [`ReadSiteRole`] name overlapping words with DIFFERENT
/// discriminants, because they answer different questions: a stage role has two
/// values and a site role has FOUR (`Unstamped`, `Drain`, `Body`, `Peek`). The
/// one that forces the skew is `Unstamped` — "the bits were never written" —
/// which must be `0` so a format-<= 4 bag decodes to it.
/// The two therefore AGREE on `Drain` (1) and DISAGREE on `Body`, and that is
/// the whole hazard: a `ReadSiteRole::Unstamped` byte fed to
/// [`ReadStageRole::from_wire`] decodes SILENTLY as `Body` rather than being
/// refused, because `0` is valid in both numberings and means opposite things.
///
/// There is no numbering that removes it (one enum needs a zero sentinel and the
/// other does not), so what ships is the guard a renumbering cannot walk past:
/// these asserts state the skew as it is, and any edit to either enum's
/// discriminants fails the build here rather than silently changing which role a
/// manifest byte decodes to. The site role never rides the capacity section —
/// the ONE place a role byte crosses the wire as a `ReadStageRole` is
/// `pack_read_outcome_meta`'s row encoder, which takes the typed value.
const _: () = {
    assert!(ReadStageRole::Drain.wire() == ReadSiteRole::Drain as u8);
    assert!(ReadStageRole::Body.wire() != ReadSiteRole::Body as u8);
    assert!(ReadStageRole::Body.wire() == ReadSiteRole::Unstamped as u8);
    // Total over the stage numbering: exactly two bytes decode, and the third
    // is refused rather than guessed.
    assert!(ReadStageRole::from_wire(0).is_some());
    assert!(ReadStageRole::from_wire(1).is_some());
    assert!(ReadStageRole::from_wire(2).is_none());
};

/// A consumer's per-window fire bound, or the absence of one — the
/// term [`derive_stage_capacity`] charges for a stage's NON-CONSUMING records
/// (a live-arm read of an empty queue, a boundary snapshot).
///
/// It is an enum rather than an `Option<u32>` for the
/// [`crate::transport::notify_delivery_latch::ListenerCountTiming`] reason:
/// `None` reads as "not specified", while the two meanings here — "this stage
/// sees one read per window" and "nothing in the graph bounds this stage" —
/// differ by three orders of magnitude and land on opposite ends of the clamp.
///
/// # What the caller declares, and why it is not always the node's fire count
///
/// The bound is stated PER STAGE, not per node, because a step-FROZEN input
/// stages exactly ONE record per STEP however many times its node fires: the
/// boundary snapshot fills the context slot and `try_view` then serves that
/// slot IN PLACE, running no accounting (`transport::subscriber`'s SERVE-MANY
/// rule). So a latest-value input of a 700-fire `Period` catch-up burst
/// declares `Fires(1)`, not `Fires(700)`.
///
/// The one input class excluded from that freeze by POLICY is `block`
/// (`build_snapshot_input_names`): a `block` input's drain must stay on the
/// consumer's tick to keep the producer-pacing mirror in lockstep, so it reads
/// LIVE on every fire and really does stage one record per fire.
///
/// **Policy is only half of it.** Being in the snapshot NAME SET does not make
/// an input frozen — the node's ENTRY has to actually fill the slot. A
/// `ClosureNodeEntry` and a raw-FFI cdylib both inherit a no-op
/// `snapshot_inputs`, so their inputs sit in the policy set while every read
/// takes `try_view`'s LIVE arm. `graph::runtime::read_stage_burst_bound` is the
/// ONE place that resolves both halves, and it is the only WIRING mint of a
/// [`StageBurstBound`] — it is the sole caller of the crate-private
/// `StageBurstBound::resolved_by_wiring` constructor (this module's own
/// oracles construct one directly, bypassing that resolution).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FireBurstBound {
    /// A bounded population: the Data/Sync within-step burst clamp, an
    /// `External` node's single fire, a `Period` node's DECLARED `max_catchup`,
    /// or `1` for a step-FROZEN input (see the type doc).
    Fires(u32),
    /// No graph fact bounds it — a `Period` node's catch-up burst on a `block`
    /// input, with no declared `max_catchup`. Lands on
    /// [`READ_OUTCOME_STAGE_MAX`]; see that constant for why the derivation
    /// refuses to read the state-arm clamp instead.
    Unbounded,
}

/// Whether an edge's reads carry a [`ReadOutcomeKind::Producer`]
/// annotation — the `pair` factor, as a NAMED fact rather than a bare `bool`
/// sitting next to another bare `bool`.
///
/// It is the value of the wiring loop's own `edge_needs_producer_annotation`
/// (`is_multi_publisher(topic) || provisioning == External`), PASSED IN rather
/// than re-derived here — re-stating half of that predicate is the
/// two-copies class. The same computed value also drives
/// `mark_multi_publisher_edge` at every site, which is what keeps the derived
/// rim EVEN wherever a pair can meet it (see the const assert above).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProducerAnnotation {
    /// Every read on this edge is admitted as an atomic `(Producer, read)`
    /// pair — two records, not one.
    Annotated,
    /// A single-publisher, graph-owned edge: one record per read.
    Plain,
}

/// Whether this stage's input is a PER-SET `Sync` TRIGGER — the
/// `site` factor, named rather than a second adjacent `bool`.
///
/// The two shapes differ in how many READS one CONSUMED FRAME costs, which is
/// independent of how many RECORDS one read costs
/// ([`ProducerAnnotation`]). Adjacent booleans for two independent factors
/// transpose silently and the arithmetic still type-checks; these do not.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyncTriggerShape {
    /// A per-set `Sync` descent: one consumed frame costs a `Peek` record when
    /// the descent PARKS it and a `Drain` record when it PROMOTES it to head.
    PerSetSyncTrigger,
    /// Every other input: one consumed frame costs one read.
    Ordinary,
}

/// A [`FireBurstBound`] that has been RESOLVED for one stage —
/// the freeze question answered by both halves (policy AND entry capability).
///
/// # Why this is a newtype with a private field
///
/// Were [`ReadStageSizing::burst`] a bare `FireBurstBound`, handing it
/// the node's raw fire bound would compile — silently returning every frozen
/// `Period` input to the ceiling (the 80x over-allocation this type exists to
/// prevent) — and hand-writing `Fires(1)` on a `block` input would compile too,
/// under-sizing by three orders of magnitude. Neither is expressible from a
/// WIRING site: the field is private, and the only wiring mint is
/// `graph::runtime::read_stage_burst_bound`, which takes the snapshot name set
/// AND the entry's `holds_input_snapshot` capability and cannot be bypassed by
/// a caller that simply forgot the freeze. (`resolved_by_wiring` is
/// `pub(crate)`, so this module's own oracle vector builds bounds directly —
/// that is a test affordance, not a second wiring path.)
///
/// [`FireBurstBound`] stays public because `node_fire_burst_bound` produces one
/// (a NODE fact) before the per-stage resolution turns it into this.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StageBurstBound(FireBurstBound);

impl StageBurstBound {
    /// The crate-internal constructor. `pub(crate)` and deliberately awkward to
    /// name: the WIRING caller is `graph::runtime::read_stage_burst_bound` (the
    /// two hand-minted transport/scheduler helpers route THROUGH that, not
    /// through here), and this module's own oracle vector builds bounds
    /// directly. A wiring site that wants a bound calls that function; it does
    /// not build one of these.
    #[must_use]
    pub(crate) const fn resolved_by_wiring(bound: FireBurstBound) -> Self {
        Self(bound)
    }

    /// The resolved bound, for the derivation and for oracles.
    #[must_use]
    pub const fn bound(self) -> FireBurstBound {
        self.0
    }
}

/// The facts ONE read-outcome stage's capacity is DERIVED from.
/// Every field is a BUILD-TIME fact the wiring loop already holds; nothing
/// here is read from the transport, the clock, or a run-time word.
///
/// "Build-time" is not the same as "a function of the graph FILE", and the
/// difference is load-bearing in two places: [`Self::burst`] folds in the
/// loaded ENTRY's `holds_input_snapshot` capability (a property of the node
/// BINARY — see [`crate::read_outcome::ReadStageSizing::burst`]), and
/// [`Self::sync_shape`] folds in `per_set_capable`, which reads
/// `CERULION_DRAIN_DISCIPLINE`. Both are fixed before step 0 and neither can
/// move during a run; neither is recoverable from the YAML alone.
///
/// # No `Default`, deliberately
///
/// The [`ReadSiteRole`] precedent: a `Default` makes `..Default::default()`
/// compile at a site whose whole contract is that the CALLER DECLARES each
/// fact, so a wiring site that forgot one would silently under-size a stage —
/// on a bag whose manifest PROMISES its capacity was derived. Every fact is
/// passed in; no site infers one here.
///
/// # The stage ROLE is NOT a field
///
/// As a field it would be restated three times per wiring site — `new`'s
/// `role`, the sizing's `stage_role`, and the burst resolver's argument — with
/// nothing reconciling them. A `Body` stage handed a `Drain` sizing would silently
/// HALVE a per-set `Sync` body.
///
/// Leaving the field out removes the third restatement and, with it, the desync
/// that matters: the sizing cannot disagree with the stage. The role is
/// still WRITTEN twice per site — `ReadOutcomeStage::new` takes it and
/// `graph::runtime::read_stage_burst_bound` needs it to decide whether a
/// `Body`-only freeze applies — but both reads come from ONE `stage_role`
/// local at each of the three wiring sites, so the two cannot drift. Collapsing
/// them to a single mention would mean resolving the burst inside `new`, which
/// would have to take the snapshot name set and the entry capability too; that
/// is a larger change than the desync it would prevent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReadStageSizing {
    /// This input's DECLARED queue depth (`#[input(depth = N)]`, read from the
    /// topology's consumer edge), never the topic ceiling and never
    /// `DEFAULT_CONSUMER_DEPTH`. Clamped by
    /// [`crate::graph::topology::MAX_CONSUMER_DEPTH`] at validation, but this
    /// function saturates rather than trusting that.
    ///
    /// It is the DECLARED depth and not `depth × publisher connections`:
    /// a connection count is not a build-time fact on an
    /// External topic, the `annotated` factor already doubles exactly the edges
    /// a connection bound would multiply, and compounding two conservative
    /// terms is the oversizing this derivation exists to delete.
    pub depth: u32,
    /// Whether this edge's reads carry a producer annotation — see
    /// [`ProducerAnnotation`].
    pub annotated: ProducerAnnotation,
    /// Whether this stage's input is a per-set `Sync` trigger — see
    /// [`SyncTriggerShape`].
    pub sync_shape: SyncTriggerShape,
    /// The bound on this stage's NON-CONSUMING records, RESOLVED by
    /// `graph::runtime::read_stage_burst_bound` — see [`StageBurstBound`] for
    /// why it cannot be built anywhere else.
    pub burst: StageBurstBound,
}

/// PURE: the record capacity one read-outcome stage is armed with.
///
/// Total, saturating, and clamped to `[READ_OUTCOME_STAGE_MIN,
/// READ_OUTCOME_STAGE_MAX]`. It reads nothing, allocates nothing, and is
/// `const` so a test can derive an oracle's rim at compile time rather than
/// restating a number the formula owns.
///
/// ```text
/// pair = 2 on an annotated edge, else 1
///
/// a per-set Sync TRIGGER's BODY stage      ⇒ READ_OUTCOME_STAGE_MAX (unbounded)
/// every other stage:
///     cap = clamp( pair × ( max( depth + burst,  2·depth + 1 ) + H ),
///                  READ_OUTCOME_STAGE_MIN, READ_OUTCOME_STAGE_MAX )
/// ```
///
/// Two things about the terms: there is no numeric `site` factor, and `burst`
/// stays even though identical runs are folded:
///
/// * **`site` is not a factor.** It would be 2 only on the per-set `Sync`
///   body, and that shape is an UNBOUNDED class returning the ceiling
///   directly, so no value of `site` could change any answer. It survives as a
///   PREDICATE — the peek/promote cost is the DERIVATION of why the class is
///   unbounded, not arithmetic pretending to be load-bearing.
/// * **`burst` STAYS.** Run-length encoding does not remove it, because RLE folds
///   CONSECUTIVE BYTE-IDENTICAL records and only a FROZEN input produces those
///   across a catch-up burst. A LIVE input can observe a new frame per fire, so
///   its records differ and the fold cannot collapse them; the sizing assumes
///   that worst case. What RLE changes for a live input is the COMMON-case
///   population, never the bound. See `derive_stage_capacity`'s body.
///
/// # Where each term comes from
///
/// The bound is stated in RECORDS, not reads, and a stage's population splits
/// in two: records that CONSUMED a frame and records that did not. Because the
/// fold stores a run of identical reads as ONE record, this number is not a
/// bound on a step's READS, and the replay gate's step plan, which carries one
/// entry per read, is therefore NOT bounded by it
/// ([`ReadPlanStage::install_step`]).
///
/// **Frame-consuming records ≤ `site × depth`.** The queue yields at most its
/// declared depth in one merge window (one merge per step), and a per-set
/// `Sync` descent spends `site` records on each frame it consumes.
///
/// **Non-consuming records ≤ `burst`.** One live-arm or snapshot record per
/// fire on the stages that are not step-frozen; the frozen ones declare
/// `Fires(1)`. The snapshot's own single record is absorbed because
/// `depth >= 1` on every wired edge.
///
/// **The second arm, `2·site·depth + 1`, is the RUN-LENGTH bound.** Two
/// maximal runs of identical non-consuming records must be separated by a
/// record that differs, and a non-consuming record can only differ from its
/// predecessor by a change of served sequence or kind — both of which require a
/// frame to have been consumed or delivered in between. So the runs are at most
/// `consuming + 1`, and a folded stream totals `site·depth + (site·depth + 1)`.
///
/// The derivation takes the **MAX** of the two arms rather than branching on
/// whether folding is enabled, so a kill switch cannot change the truncation
/// shape — which would otherwise be a silent replay-compat fork. Until the
/// run-length encoding lands, the unfolded arm is the one that actually
/// binds on a bursting edge, which is why the `burst` term is still here.
///
/// # What the derivation reads, stated at its real strength
///
/// Every input is a BUILD-TIME graph fact, with ONE scoped exception: the
/// `sync_shape` a wiring site declares depends on `per_set_capable`, which
/// includes `!force_separate_discipline` — the `CERULION_DRAIN_DISCIPLINE`
/// measurement seam. So the widest shape derives 516 unset and 260 under
/// `=separate`. That is contained rather than alarming: the same knob already
/// forks the RECORD STREAM itself (it changes which subscriber performs each
/// read), which is the replay verifier's divergence class 2 — a bag recorded
/// under one discipline was never comparable against a replay under the other,
/// capacity or no capacity. The accurate claim is therefore "no run-time word
/// OTHER than the drain-discipline measurement seam, which already forks the
/// stream", NOT "never varies run to run".
///
/// # The PAIR factor, and why it multiplies the WHOLE bound
///
/// A read on a `multi_publisher_topics`-listed (or
/// External-provisioned) edge costs TWO records: the
/// [`ReadOutcomeKind::Producer`] annotation and the read it annotates, admitted
/// as an atomic pair by `ReadOutcomeStage::record_with_producer` (crate-private,
/// so named rather than linked — an intra-doc link from a `pub` item to a
/// `pub(crate)` one is the documented `RUSTDOCFLAGS=-D warnings` CI break). A
/// bound written in READS therefore under-counts every annotated edge by
/// exactly a factor of two.
///
/// The factor multiplies the whole bound, [`READ_OUTCOME_STAGE_HEADROOM`]
/// included, because the headroom slot is the post-cap REFILL read's — and on
/// exactly the edges this factor exists for, that read is a pair.
///
/// # The PEEK+PROMOTE factor (`site`)
///
/// The pair factor counts the records ONE read costs. The peek mark
/// changes how many READS one CONSUMED FRAME costs on a per-set `Sync` edge,
/// and that is a second, independent multiplier.
///
/// A per-set `Sync` descent asks an input for the stamp BEHIND its head:
/// `sync_peek_next_stamp` pops exactly one frame to learn it and PARKS the
/// frame. That pop is a consumed frame like any other, so it stages a record —
/// `DrainedBatch` at [`ReadSiteRole::Peek`], `popped >= 1`. When the descent
/// then advances, the parked frame is PROMOTED to head and the promotion stages
/// a SECOND record — `DrainedBatch` at [`ReadSiteRole::Drain`], `popped: 0`,
/// because the pop was already accounted at the peek. Both are minted inside
/// ONE merge window: a descent is a single align pass, entirely between two
/// level-end merges.
///
/// So on an annotated per-set `Sync` edge — the intersection the bound has to
/// survive — one consumed frame costs FOUR records:
///
/// ```text
/// peek:      Producer annotation + DrainedBatch(role = Peek,  popped >= 1)
/// promotion: Producer annotation + DrainedBatch(role = Drain, popped  = 0)
/// ```
///
/// At `MAX_CONSUMER_DEPTH` that shape derives 516 records, which a single
/// global constant of 320 would NOT hold: it would truncate a healthy full-depth
/// descent and emit an OVERFLOW MARKER on an edge doing nothing wrong. That
/// is what a global number cannot express — the same
/// formula sizes an ordinary latest-value input at 4.
#[must_use]
pub const fn derive_stage_capacity(role: ReadStageRole, sizing: ReadStageSizing) -> u32 {
    let pair: u64 = match sizing.annotated {
        ProducerAnnotation::Annotated => 2,
        ProducerAnnotation::Plain => 1,
    };
    // The peek/promote pair mints on the BODY subscriber (per-set `Sync`
    // unifies its trigger drain onto it), so a `Drain` stage cannot hold it.
    //
    // `(Drain, PerSetSyncTrigger)` is UNREACHABLE by construction, and the
    // three reasons are worth stating because the arm below looks like a
    // silent normalisation and is not meant to be one: per-set `Sync` unifies
    // the trigger drain onto the body subscriber, so no drain stage exists on
    // that path at all; the Separate trigger drain is created only on a binding
    // that is NOT per-set-capable; and the legacy-`Sync` drain lives in the
    // `else` arm of `per_set_capable`. So the pair is a WIRING DESYNC, and the
    // `debug_assert!` names it rather than absorbing it. The release answer
    // stays the fail-safe site 1 — a derivation must never abort the build path
    // it sizes — which mirrors `ReadSiteRole::from_wire`'s posture exactly.
    // WHICH SHAPE THIS IS. The peek/promote SITE factor could be a numeric
    // term here (2 on a per-set Body, 1 everywhere else). It is a
    // PREDICATE instead, because its value cannot change any answer: the
    // only shape it is 2 on is the per-set Body, and that shape is
    // unbounded, so multiplying by it and then saturating onto the ceiling
    // gives the ceiling either way. A term nothing can observe is a term that
    // rots — the peek/promote cost survives as the DERIVATION of why this class
    // is unbounded (see above), not as arithmetic pretending to be load-bearing.
    //
    // `(Drain, PerSetSyncTrigger)` is UNREACHABLE by construction, and the
    // three reasons are worth stating because the arm below looks like a silent
    // normalisation and is not meant to be one: per-set `Sync` unifies the
    // trigger drain onto the body subscriber, so no drain stage exists on that
    // path at all; the Separate trigger drain is created only on a binding that
    // is NOT per-set-capable; and the legacy-`Sync` drain lives in the `else`
    // arm of `per_set_capable`. So the pair is a WIRING DESYNC, and the
    // `debug_assert!` names it rather than absorbing it. The release answer is
    // the fail-safe: treat it as the ORDINARY shape, which is what every real
    // `Drain` site passes anyway. A `const fn` cannot pair that with a `warn!`
    // — logging is not available in const context — so debug builds and the
    // whole test suite get the loud arm, exactly as `ReadSiteRole::from_wire`
    // does.
    let per_set_body: bool = match (role, sizing.sync_shape) {
        (ReadStageRole::Body, SyncTriggerShape::PerSetSyncTrigger) => true,
        (ReadStageRole::Body, SyncTriggerShape::Ordinary) => false,
        (ReadStageRole::Drain, _) => {
            debug_assert!(
                matches!(sizing.sync_shape, SyncTriggerShape::Ordinary),
                "a Drain stage cannot be a per-set Sync trigger: per-set unifies the \
                 trigger drain onto the BODY subscriber, so this pair is a wiring desync"
            );
            false
        }
    };
    if per_set_body {
        // The UNBOUNDED class, stated rather than approximated. Every arm below
        // would saturate onto this value anyway; returning it here is what makes
        // the claim legible instead of an accident of the clamp.
        return READ_OUTCOME_STAGE_MAX;
    }
    // Every REMAINING shape consumes at most its declared depth per window at
    // ONE record per consumed frame. An ORDINARY Body can out-run `depth` under
    // a refill, but its node's `burst` term is the same
    // `DATA_PENDING_CARRY_CLAMP`, so `depth + burst >= max_sets` already covers
    // it. A `Drain` stage stages ONE `DrainedBatch` per NON-EMPTY drain whatever
    // the batch popped (`drain_samples`: `if capture && count > 0` stages once,
    // with `popped` as a FIELD), so a queue provisioned deeper than the
    // declaration by another consumer — the legacy-`Sync` site's
    // `depth.max(sub_buf)` — yields a bigger `popped`, never more records. That
    // is pinned by
    // `a_drain_stage_holds_a_batch_deeper_than_its_declared_depth`.
    let consuming = sizing.depth as u64;
    // Run-length encoding folds a maximal run of identical non-consuming
    // records to ONE counted record, which shrinks this arm's COMMON-case
    // population. The term stays in the formula (never behind a `folds_runs`
    // branch) so the fold's kill switch cannot re-shape a truncation.
    //
    // The `burst` term STAYS. "Folding lets burst leave
    // the formula" is an OVER-CLAIM, and the argument for it
    // has a hole worth naming because it is easy to re-make.
    //
    // The argument runs: two maximal runs must be separated by a record that
    // DIFFERS, and a non-consuming record can only differ from its predecessor
    // by a change of `served_seq` or `kind` — "both of which require a frame to
    // have been consumed or delivered in between". The consumption half is
    // bounded by this stage. The DELIVERY half is not: a frame arriving between
    // two catch-up fires comes from ANOTHER node, so the separator is produced
    // outside the bound and `runs <= consuming + 1` does not hold.
    //
    // What the fold actually does, stated precisely: it collapses CONSECUTIVE
    // BYTE-IDENTICAL records. That splits the shapes in two.
    //
    //  * A step-FROZEN input serves the same snapshot on every catch-up fire,
    //    so its burst is a run of identical records and folds to one slot. The
    //    sizing already treats it that way — `read_stage_burst_bound` declares
    //    `Fires(1)` for it — so folding has nothing further to remove here.
    //  * A LIVE (unfrozen) input — a `block` input, or any input on an entry
    //    that cannot freeze — may observe a NEW frame on every catch-up fire.
    //    Each such read carries a different `served_seq`, so the records are
    //    DISTINCT and the fold cannot touch them. Under lockstep a producer's
    //    publishes per step are bounded by its own fires, which for a `Period`
    //    producer is unbounded; under free-run it is unbounded outright. Its
    //    worst case is therefore one DISTINCT record per fire, which is exactly
    //    what `burst` counts.
    //
    // So RLE changes the COMMON-case population of a live input (measured: a
    // 701-fire catch-up burst with no new frames stages 3 records instead of
    // 701) and never the WORST-case bound the stage must be sized for.
    // The rule "assume the worst" governs the sizing, and the ceiling arms
    // for `Period` x `block` and for a cannot-freeze closure node are the pins
    // that say so.
    let unfolded = match sizing.burst.bound() {
        FireBurstBound::Fires(n) => consuming.saturating_add(n as u64),
        FireBurstBound::Unbounded => u64::MAX,
    };
    let folded = consuming.saturating_mul(2).saturating_add(1);
    let arm = if unfolded > folded { unfolded } else { folded };
    let cap = pair.saturating_mul(arm.saturating_add(READ_OUTCOME_STAGE_HEADROOM as u64));
    if cap < READ_OUTCOME_STAGE_MIN as u64 {
        READ_OUTCOME_STAGE_MIN
    } else if cap > READ_OUTCOME_STAGE_MAX as u64 {
        READ_OUTCOME_STAGE_MAX
    } else {
        cap as u32
    }
}

/// The ANNOTATION half of an atomic pair's fold key — `true` only when BOTH
/// records are `Producer` annotations that match field for field.
///
/// The inverse of [`records_fold`], which refuses a `Producer` outright. The two
/// exist separately because they answer different questions: a `Producer` is
/// never a fold target in its own right (a folded pair carries its count on the
/// READ half), but a pair fold has to re-match the annotation it is
/// folding INTO, token included. Destructured for the same reason as its
/// sibling: a field added to [`StagedReadOutcome`] joins this key by
/// construction.
///
/// The `*kind == Producer` conjunct is a REQUIREMENT, not an optimisation — it
/// is what stops this predicate from accepting two matching plain reads and
/// letting `record_with_producer` fold a pair onto something that is not one.
#[inline]
#[must_use]
fn annotation_folds(last: &StagedReadOutcome, incoming: &StagedReadOutcome) -> bool {
    let StagedReadOutcome {
        kind,
        served_seq,
        popped,
        token,
        role,
        run_count: _,
    } = incoming;
    last.kind == *kind
        && *kind == ReadOutcomeKind::Producer
        && last.served_seq == *served_seq
        && last.popped == *popped
        && last.token == *token
        && last.role == *role
}

/// Whether two records may be folded into one counted run.
///
/// The predicate is TOTAL over the record — it compares every field except the
/// count itself — so a field added to [`StagedReadOutcome`] tomorrow becomes
/// part of the fold key by construction rather than being silently ignored.
/// That is the same "a new fact fails the check rather than slipping past it"
/// property the derivation has.
///
/// A `Producer` annotation is never folded on this path: a folded pair carries
/// its count on the READ half, and `record_with_producer` handles
/// that case as a unit.
#[inline]
#[must_use]
fn records_fold(last: &StagedReadOutcome, incoming: &StagedReadOutcome) -> bool {
    if last.kind == ReadOutcomeKind::Producer {
        return false;
    }
    let StagedReadOutcome {
        kind,
        served_seq,
        popped,
        token,
        role,
        run_count: _,
    } = incoming;
    last.kind == *kind
        && last.served_seq == *served_seq
        && last.popped == *popped
        && last.token == *token
        && last.role == *role
}

/// The fold kill switch, `CERULION_READ_LOG_FOLD=off`.
///
/// Read once per process. A run with folding OFF stamps `trace_format` 5 rather
/// than 6, so a bag never claims a format whose encoding it did not use.
///
/// The CAPACITY does not depend on this switch, and the reason is NOT that
/// folding shrinks the worst case: the derivation keeps its
/// `burst` term, because folding is CONSECUTIVE and BYTE-IDENTICAL only. A
/// step-FROZEN input serves the same snapshot every fire, so its records really
/// do fold — and [`derive_stage_capacity`] already sizes that shape at
/// `Fires(1)`. A LIVE or unfrozen input may see a NEW frame every fire, so its
/// records differ and it costs one slot per fire whether or not the switch is
/// on. The rule "assume the worst" governs the SIZING; this switch
/// changes only whether identical runs are COLLAPSED in the common case, which
/// is why it is safe to ship as a switch at all.
///
/// It is also report-shape-only on the REPLAY side, and that is a property the
/// verifier has to maintain rather than inherit: every consumer of a folded
/// record EXPANDS it into the occurrences it stands for (see
/// `replay_engine::expand_run`), so a folded stream and an unfolded one compare
/// occurrence-for-occurrence and this switch cannot move a verdict.
pub fn fold_enabled() -> bool {
    static ENABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ENABLED.get_or_init(|| {
        !matches!(
            std::env::var("CERULION_READ_LOG_FOLD").as_deref(),
            Ok("off")
        )
    })
}

/// The identity of a read-outcome STAGE — the only key
/// adoption uses.
///
/// A stage is `(node, input_idx, role)` and NOT `(node, input_idx)`: an input
/// under the Separate or legacy-`Sync` discipline contributes TWO stages that
/// share an index. Written down as a type because the completeness check and
/// the arming loop must enumerate the SAME set. If the check walked input
/// NAMES and accepted an input when ANY role row existed, while arming keyed on
/// the role too, a table naming only an input's `Body` row would pass the check
/// and its `Drain` stage would then arm at the DERIVED rim in silence. One key
/// type, one accessor (`GraphRuntime::read_outcome_stage_keys`), and the two
/// walks cannot disagree.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct StageKey {
    /// The consuming node's id.
    pub node: String,
    /// The input's index into its node's wiring-order input table.
    pub input_idx: u16,
    /// Which read path the stage observes.
    pub role: ReadStageRole,
}

impl StageKey {
    /// `node[idx]/role` — how a stage is NAMED in an operator-facing refusal.
    #[must_use]
    pub fn label(&self) -> String {
        format!("{}[{}]/{}", self.node, self.input_idx, self.role.label())
    }
}

/// A staging rim a RECORDING declared, validated against this
/// binary's window `[READ_OUTCOME_STAGE_MIN, READ_OUTCOME_STAGE_MAX]`.
///
/// The type exists so the window is checked in ONE place and a rim that fails
/// it cannot be silently repaired into a different one. Both ends are refusals
/// for the same reason: adoption exists so the two sides truncate identically,
/// and a rim this binary changed — up OR down — is a rim the recording never
/// used, which is exactly the non-comparable shape adoption exists to prevent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdoptedRim(u32);

/// What [`AdoptedRim::from_recorded`] refuses, with both bounds so the caller's
/// message can name them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RimOutOfRange {
    /// The rim the bag declared.
    pub declared: u32,
    /// This binary's floor.
    pub min: u32,
    /// This binary's ceiling.
    pub max: u32,
}

impl AdoptedRim {
    /// Validate a rim a bag declared. `Err` is a stand-down, never a clamp.
    pub fn from_recorded(declared: u32) -> Result<Self, RimOutOfRange> {
        if (READ_OUTCOME_STAGE_MIN..=READ_OUTCOME_STAGE_MAX).contains(&declared) {
            Ok(Self(declared))
        } else {
            Err(RimOutOfRange {
                declared,
                min: READ_OUTCOME_STAGE_MIN,
                max: READ_OUTCOME_STAGE_MAX,
            })
        }
    }

    /// The validated rim.
    #[must_use]
    pub const fn get(self) -> u32 {
        self.0
    }
}

/// One stage's DERIVED capacity, with the two facts that identify the
/// stage it belongs to.
///
/// Not `cfg(test)`, deliberately: the manifest writer and the test
/// seam must share ONE row type, so the ordering and keying contract is written
/// down once. See `GraphRuntime::read_outcome_stage_capacities` for the vector's
/// shape — and note that `input_idx` alone does NOT identify a row.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReadStageCapacity {
    /// This input's index into its node's manifest input table (wiring order).
    /// REPEATS across roles: an input under the Separate or legacy-`Sync`
    /// discipline carries two stages that deliberately share it.
    pub input_idx: u16,
    /// Which read path the stage observes. With `input_idx` this is the key.
    pub role: ReadStageRole,
    /// The capacity the stage is armed at.
    pub capacity: u32,
}

/// The CALL-SITE role a kind-6 record carries on the wire — bits
/// 14..16 of its packed `global_level` word (`trace_ring::pack_read_outcome_meta`).
///
/// # Why this is NOT [`ReadStageRole`]
///
/// `ReadStageRole` names which STAGE a record landed in, and the two genuinely
/// differ: under the UNIFIED drain discipline there is no trigger subscriber at
/// all, so `drain_for_trigger` runs on the node's own BODY subscriber and its
/// records land in a `Body`-role stage while the CALLER is the scheduler's
/// boundary drain. Stamping the stage role would reproduce the very ambiguity
/// one discipline over.
///
/// **By design, the wire role is the role of the CALL SITE that performed
/// the read.** Every mint site knows it statically and passes a compile-time
/// constant; no site infers it from state. That is the
/// [`crate::transport::notify_delivery_latch::ListenerCountTiming`] /
/// `TriggerDrainSite` discipline: inferring which site is asking is precisely
/// the silent inversion a declared type exists to prevent.
///
/// # `Unstamped` is not a value a writer elects
///
/// It is what an UNWRITTEN field reads as. Every kind-6 record of a bag whose
/// `trace_format` is <= 4 carries zeroes in these bits (verified: the only
/// packer ever wrote `ReadOutcomeKind` discriminants <= 7, so bits 14-15 are
/// zero in every record that exists), so `0` means "this bag predates roles" —
/// never "this writer did not know".
///
/// # The two-bit field is FULL
///
/// Wire value `3` is [`Self::Peek`], a third SITE, and `trace_format` 5 is DEFINED to carry that
/// meaning: format 5 has exactly one definition, with three roles. A
/// reader on a `trace_format` <= 4 bag still treats EVERY role as unstamped,
/// because those bits were never written there.
///
/// The consequence a future site has to plan for: there is no fourth value
/// left. A fourth SITE needs a wider subfield (three bits, narrowing the kind
/// half to 13), which is a `trace_format` bump and a re-check of every packed
/// oracle — not an additive edit.
///
/// # No `Default`
///
/// It deliberately does NOT derive `Default`, even though `Unstamped` is the
/// obvious candidate. A
/// `Default` on a role is precisely the footgun this module forbids: it makes
/// `..Default::default()` and `unwrap_or_default()` compile at a site whose
/// whole contract is that the CALL SITE declares its role as a compile-time
/// constant, so a mint that forgot to name one would silently record
/// "unstamped" — a record whose site nothing on the wire names, on a bag whose
/// format PROMISES the bits are readable. Nothing constructed it that way, so
/// the derive bought no caller anything and only kept the hole open.
///
/// # Adding a variant: the predicates the compiler will NOT name
///
/// Adopting a new site is not only a `from_wire` edit. The compiler catches the
/// EXHAUSTIVE matches on its own — replay's steering arm
/// (`crates/cerulion_cli_engine/src/replay_engine.rs`), the census
/// (`replay_rederive.rs`) and the ABI variant-set pin
/// (`crate::abi_layout`) — but several predicates test a role by EQUALITY or by
/// NEGATION, so a new variant is absorbed silently, each with a different wrong
/// answer:
///
/// - [`Self::is_stamped`] (below) is `!matches!(Unstamped)`, so a new variant
///   is BELIEVED the moment it exists — every consumer gated on it takes the
///   wire's word for a site it has no rule for.
/// - `replay_rederive::is_drain_site_read` enumerates the CONSUMING sites, so a
///   new consuming site reads as a BODY read: excluded from the FIFO count and
///   from `block` credit conservation, with no stand-down and no diagnostic.
/// - `replay_rederive::is_sync_head_record` enumerates the sites that name the
///   HEAD, so a new head-naming site is silently dropped from the sync fold.
/// - `replay_rederive::unproducible_read_shape` matches each believed role
///   against the kinds its site can mint, so a new site is corruption-checked
///   against nothing until a match arm is written for it.
///
/// This is not hypothetical any more: [`Self::Peek`] was added in exactly this
/// way, and each of those four predicates needed an edit the compiler did not
/// demand. They are recorded because a `match` that fails to compile is a
/// reminder and an `==` is not.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ReadSiteRole {
    /// The bits were never written (a format <= 4 bag). Takes the pre-roles arm
    /// at every consumer.
    Unstamped = 0,
    /// The read was performed by a DRAIN — a read that CONSUMES from the
    /// input's queue on the scheduler's behalf AND whose frame BECAME THE HEAD:
    /// `try_receive_for_drain` (the Separate/Sync trigger drain),
    /// `drain_for_trigger` (the unified boundary drain), the Data burst refill,
    /// the per-set Sync matcher's `Advance`/`DiscardTie` REFILL, and the
    /// matcher's PROMOTION of an already-peeked frame. That enumeration IS the
    /// definition. Its stamps are what reached `sync_input_timestamps`.
    ///
    /// "Performed by the scheduler" would NOT separate the roles, and the
    /// carve-out is not an exception to a rule: the step-boundary snapshot is
    /// ALSO performed by the scheduler and mints [`Self::Body`], because what
    /// it serves is the node body's latest-value read (the seven snapshot mints
    /// in `transport::subscriber`, and the carve-out argued beside them).
    Drain = 1,
    /// The read was performed by NODE CODE inside its own tick.
    Body = 2,
    /// A scheduler read that POPPED a
    /// frame from the queue to LOOK at its stamp. The frame is PARKED as the
    /// per-set Sync matcher's `next_head` — it is NOT the head, and the set the
    /// matcher fired may have been aligned on the frame ahead of it.
    ///
    /// It is a CONSUMING read like [`Self::Drain`] (the frame really left the
    /// queue, and its `popped` count is the only place that pop is accounted),
    /// so FIFO pop counting and `block` credit conservation treat the two
    /// alike. What it is NOT is the HEAD, which is why `verify_sync`'s stamp
    /// fold ignores it: folding a peeked stamp judges the schedule against a
    /// frame the matcher looked at and did not align on.
    ///
    /// # Only `DrainedBatch` (and its paired `Producer` annotation)
    ///
    /// The role marks a record that NAMES A PARKED FRAME, so the ONE mint is
    /// `sync_peek_next_stamp`'s `Sample` arm. Its `Decimated` arm keeps
    /// [`Self::Drain`]: a decimated pop parks nothing, so there is no head to
    /// distinguish it from and nothing for a consumer to exclude. On an
    /// annotated edge the peek stages through `stage_read_outcome_with_producer`
    /// like any other consuming read, so its `Producer` token carries `Peek`
    /// too. Every OTHER pairing is unproducible — see
    /// `replay_rederive::unproducible_read_shape`.
    Peek = 3,
}

impl ReadSiteRole {
    /// The wire value carried in bits 14..16 of the kind-6 meta word.
    #[inline]
    pub fn wire(self) -> u16 {
        self as u16
    }

    /// Decode a wire role. Every value the two-bit field can hold now names a
    /// real variant (`3` became [`Self::Peek`] in the format-5 definition), so the
    /// only value that reads [`Self::Unstamped`] is the `0` a `trace_format`
    /// <= 4 bag carries. The catch-all arm survives for the masking mistake the
    /// `debug_assert!` below names, and answers with the pre-roles arm.
    ///
    /// The argument is the SUBFIELD, already masked out of the meta word by
    /// `trace_ring::read_site_role`, so a value above the two-bit field can
    /// only come from a caller that skipped the mask — which would silently
    /// read as `Unstamped` and make a role-bearing record look archived. The
    /// `debug_assert!` names that mistake instead of absorbing it; the release
    /// answer is unchanged (`Unstamped`, the pre-roles arm), because a decoder
    /// must never abort the path that reads it.
    #[inline]
    pub fn from_wire(wire: u16) -> Self {
        // `0b11` rather than `trace_ring::READ_OUTCOME_ROLE_MASK`: this module
        // is PORTABLE and `trace_ring` is `#[cfg(unix)]`, so the constant is
        // not nameable here. The two are tied by a `const _` layout assert
        // where the mask is declared.
        debug_assert!(
            wire <= 0b11,
            "a read-site role is a TWO-BIT subfield; got {wire}. Mask it out of the \
             meta word with `trace_ring::read_site_role` rather than handing the whole word here"
        );
        match wire {
            1 => Self::Drain,
            2 => Self::Body,
            3 => Self::Peek,
            // 0 = never written. Anything above the two-bit field is a caller
            // that skipped the mask; the pre-roles arm is the safe answer.
            _ => Self::Unstamped,
        }
    }

    /// Whether this role names a real call site (i.e. the bag stamped one).
    #[inline]
    pub fn is_stamped(self) -> bool {
        !matches!(self, Self::Unstamped)
    }

    /// The stable label rendering surfaces carry.
    pub fn label(self) -> &'static str {
        match self {
            Self::Unstamped => "unstamped",
            Self::Drain => "drain",
            Self::Body => "body",
            Self::Peek => "peek",
        }
    }
}

impl From<ReadStageRole> for ReadSiteRole {
    /// The OVERFLOW MARKER's role — "the STAGE that overflowed", diagnostic
    /// only. It is deliberately NOT a
    /// call-site role: a `Body`-role stage genuinely holds drain-SITE records
    /// under the unified discipline. No consumer steers on it — `Truncated`
    /// goes to BOTH injection queues and raises `TruncatedReadLog` under EVERY
    /// role value — so nothing is weakened by the marker's role being the
    /// stage's.
    ///
    /// It is also TOTAL over the stage vocabulary, which is what makes
    /// `Peek` + `Truncated` an unproducible pairing: a stage carries a
    /// `ReadStageRole`, which has exactly two variants and neither maps here to
    /// [`ReadSiteRole::Peek`].
    fn from(role: ReadStageRole) -> Self {
        match role {
            ReadStageRole::Body => Self::Body,
            ReadStageRole::Drain => Self::Drain,
        }
    }
}

/// The outcome of one input read, as staged by the subscriber. The explicit
/// discriminants ARE the kind-6 wire values (`trace_ring::READ_OUTCOME_*`) —
/// pinned by a drift-guard test on unix, where the wire constants live.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum ReadOutcomeKind {
    /// A fresh frame was served to the read.
    Served = 1,
    /// The HELD sample was replayed (no new arrival); `served_seq`
    /// carries the HELD frame's wire sequence.
    Held = 2,
    /// No frame has ever been delivered on this input — the read observed
    /// nothing (wire kind "none").
    NoFrame = 3,
    /// A trigger/batch drain: `served_seq` = the NEWEST sequence in the
    /// batch, `popped` = the batch size. Per-message FIFO (52125241e): an
    /// `EachFifo` (data-trigger) boundary drain pops exactly ONE frame, so
    /// its record is a batch of one (`served_seq` = the FIFO head, `popped`
    /// = 1) — same kind, the vocabulary distinguishes SITE ROLE (trigger
    /// drain vs body read), never queue discipline. The drain-everything
    /// shape survives on the Separate trigger-drain / Sync `drain_samples`
    /// path, where `served_seq` really is the newest of N.
    DrainedBatch = 4,
    /// The `sample(N)` gate popped frames but delivered none; `served_seq` =
    /// the last-ACCEPTED sequence (stage-remembered) or the no-frame
    /// sentinel.
    Decimated = 5,
    /// ANNOTATION — the OVERFLOW MARKER. Not a read at all: it
    /// says "the `popped` records that would have followed this position, in
    /// THIS merge window, were dropped at the rim". Emitted LAST in the
    /// window by `ReadOutcomeStage::drain_into`, one per input whose stage
    /// dropped at least one record since the previous drain (a per-window
    /// DELTA, never the lifetime total — the lifetime total is
    /// [`ReadOutcomeStage::dropped`]). `served_seq` is the no-frame
    /// sentinel.
    Truncated = 6,
    /// ANNOTATION — the PRODUCER TOKEN. Not a read: it names
    /// WHICH publisher produced the frame the NEXT read record on the same
    /// (node, input) served, as a 64-bit token over iceoryx2's
    /// `UniquePublisherId` (see `transport::subscriber`'s minting site). Only
    /// ever emitted on an input whose topic is `multi_publisher_topics`-listed,
    /// so a single-publisher edge's stream is unchanged by it.
    /// `served_seq` REPEATS the annotated read's served sequence (a redundant
    /// join key), and the token rides the FULL 64-bit aux word — see
    /// [`StagedReadOutcome::token`].
    Producer = 7,
}

impl ReadOutcomeKind {
    /// The kind-6 wire value (the low-16 half of the record's packed
    /// `global_level` slot).
    #[inline]
    pub fn wire(self) -> u16 {
        self as u16
    }
}

/// One staged read outcome — POD, already in wire terms: `served_seq` is the
/// raw served-seq slot value (a wire `u32` widened, or `u64::MAX` = no
/// frame), `popped` the drain's consumed-frame count (saturated to `u32`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StagedReadOutcome {
    /// What the read served.
    pub kind: ReadOutcomeKind,
    /// The served frame's wire sequence (`u64::MAX` = no frame).
    pub served_seq: u64,
    /// Frames this drain consumed off the queue. At the batch-drain site
    /// (`drain_samples` → `DrainedBatch`) this is the DELIVERED count: a
    /// malformed frame is popped off the queue but skipped by
    /// `deliver_raw_frame` and NOT counted here (matching the timestamps the
    /// trigger path forwards).
    ///
    /// On a [`ReadOutcomeKind::Truncated`] marker this is REINTERPRETED as
    /// the number of records dropped in the marker's own merge window (a
    /// per-window delta); on a [`ReadOutcomeKind::Producer`] annotation it is
    /// unused (0) and the payload rides [`Self::token`].
    pub popped: u32,
    /// The FULL 64-bit aux word for the ONE annotation kind
    /// that needs more than 32 bits — the producer token on
    /// [`ReadOutcomeKind::Producer`]. `None` for every other kind, whose aux
    /// word is `popped` (packed by `trace_ring::pack_read_outcome_aux`).
    ///
    /// It is a separate field rather than a widened `popped` because the two
    /// mean different things and are read by different consumers: `popped` is
    /// part of the verifier's per-read comparison tuple, while the token is
    /// resolved through a manifest table before it means anything at all.
    pub token: Option<u64>,
    /// The CALL-SITE role (see [`ReadSiteRole`]) — passed as a
    /// compile-time constant by the mint site, carried through the stage, and
    /// packed into the record's meta word by
    /// `trace_ring::pack_read_outcome_meta`.
    ///
    /// It rides the STAGED record rather than being read off the stage,
    /// because ONE stage legitimately holds BOTH roles' records: under the
    /// unified discipline `drain_for_trigger` stages a drain-SITE read into
    /// the node's own body stage.
    pub role: ReadSiteRole,
    /// How many CONSECUTIVE byte-identical occurrences of this
    /// record the stream carries at this position. `1` for an unfolded record;
    /// a fold increments it rather than pushing a second copy.
    ///
    /// The fold key is the WHOLE record — every other field of this struct —
    /// deliberately, so a future field added here becomes part of the key by
    /// construction and FAILS the fold rather than silently collapsing two
    /// records that differ in it.
    ///
    /// On the wire it rides the aux word's high 32 bits (packed by
    /// `trace_ring::pack_read_outcome_aux_run` — named in plain backticks, NOT
    /// linked: this item is PORTABLE and `trace_ring` is `#[cfg(unix)]`-only, so
    /// an intra-doc link from here fails `cargo doc -D warnings` on a non-unix
    /// build. `cfg_audit_test::no_portable_item_references_a_unix_only_module` is
    /// the gate that says so) with `0 == 1`, so a
    /// format-≤5 bag — whose high bits are structurally zero — decodes as all
    /// ones, which is exactly right.
    pub run_count: u32,
}

impl StagedReadOutcome {
    /// The served wire sequence in `Option` form (`None` = the
    /// [`STAGE_NO_FRAME`] sentinel) — the typed shape the kind-6 assembly
    /// site takes, so the sentinel packing lives in exactly one place
    /// (`trace_ring::TraceRingRecord::read_outcome`). Lossless: staging only
    /// ever stores a widened wire `u32` or the sentinel.
    pub fn served_wire_seq(self) -> Option<u32> {
        (self.served_seq != STAGE_NO_FRAME).then_some(self.served_seq as u32)
    }
}

/// The "no frame" sentinel for [`StagedReadOutcome::served_seq`] — kept
/// numerically identical to `trace_ring::READ_OUTCOME_NO_FRAME` (drift-guard
/// pinned) so staging is already in wire terms.
pub const STAGE_NO_FRAME: u64 = u64::MAX;

/// What, if anything, the stage's LAST record may be folded
/// into — the state a run count's two guarantees are made of.
///
/// A run count is a claim about ADJACENCY ("these N reads happened back to
/// back") and about PROVENANCE ("all N are the same kind of read"). Both are
/// properties of what happened BETWEEN two pushes, which no single record can
/// carry, so the stage tracks it:
///
/// * [`FoldAnchor::BareRead`] — the last push was a plain [`ReadOutcomeStage::record`].
///   Only a plain `record` may fold into it.
/// * [`FoldAnchor::PairRead`] — the last push was the READ half of a
///   `(Producer, read)` atomic pair. Only
///   [`ReadOutcomeStage::record_with_producer`] may fold into it, and it must
///   re-match the ANNOTATION too. Without this state a bare read whose fields
///   happen to match would fold into the pair's read half — the read half
///   carries `token: None`, so `records_fold` alone cannot tell the two apart —
///   and an offline reader expanding `Producer x 1 + read x N` would hand the
///   pair's publisher provenance to a read that never had it.
/// * [`FoldAnchor::Broken`] — nothing may fold. Set on a stage that holds no
///   foldable record: a fresh one, and every stage the level-end
///   merge has just DRAINED, so a new window can never fold into the last record
///   of the previous one. (That would hold anyway because the records are gone,
///   which is an accident of the implementation rather than the invariant.) And,
///   crucially, on every DROP at the rim: if the stream was `R, S, R` and `S`
///   was dropped at capacity, folding the second `R` into the first would claim
///   an adjacency the stream did not have, and the reader would materialise the
///   extra position where the dropped record used to be. A drop is described by
///   the window's [`ReadOutcomeKind::Truncated`] marker; it must not also be
///   silently papered over by a run count. **Run counts are adjacency-faithful**
///   is therefore a guarantee, not a caveat.
///
/// `pub(crate)`, not `pub`: the `#[cfg(test)]`
/// `ReadOutcomeStage::fold_anchor_for_test` accessor is itself `pub(crate)`, and
/// clippy's `private_interfaces` is right that a `pub(crate)` signature must not
/// name a module-private type. Widened to match the accessor exactly rather than
/// silenced with an `#[allow]`, which would assert the signature is fine when it
/// is not; the variants stay unreachable outside the crate either way.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub(crate) enum FoldAnchor {
    #[default]
    Broken,
    BareRead,
    PairRead,
}

/// The mutable half of a stage, behind the uncontended `Mutex`.
#[derive(Debug, Default)]
struct StageInner {
    records: Vec<StagedReadOutcome>,
    /// What the last push left foldable — see [`FoldAnchor`].
    fold_anchor: FoldAnchor,
    /// The last sequence this input SERVED (`Served` / `DrainedBatch`) — the
    /// "last accepted" a `Decimated` record reports (on a sample-gated input
    /// every served frame passed the gate, so the two are the same value).
    last_served_seq: Option<u32>,
    /// Records dropped at capacity — NEVER reset (an unconditional total,
    /// Principle #3; read off-thread via [`ReadOutcomeStage::dropped`]).
    dropped: u64,
    /// How much of `dropped` has already been described by an
    /// emitted [`ReadOutcomeKind::Truncated`] marker. The marker carries
    /// `dropped - dropped_reported`, i.e. THIS WINDOW's delta — a marker
    /// carrying the LIFETIME total would re-report every earlier window's
    /// losses at every later drain, so an offline reader summing the markers
    /// would count one dropped record many times over. The lifetime total
    /// stays available, unconditionally, via [`ReadOutcomeStage::dropped`].
    dropped_reported: u64,
    /// Whether the FIRST overflow warn has been announced (the loud head of
    /// the decade ladder below).
    dropped_announced: bool,
    /// The next power-of-ten `dropped` total at which the open overflow
    /// condition RE-ANNOUNCES itself loudly (the decade discipline).
    /// 0 until the head warn fires.
    next_decade: u64,
}

/// The served-seq SLOT one staged record carries, plus the `last_served_seq`
/// bookkeeping the slot rule depends on — factored out of [`ReadOutcomeStage::record`]
/// so [`ReadOutcomeStage::record_with_producer`] cannot grow a second copy of
/// the `Decimated` last-accept rule (the one place this module has ever been
/// wrong is exactly there).
fn resolve_seq_slot(inner: &mut StageInner, kind: ReadOutcomeKind, served_seq: Option<u32>) -> u64 {
    let slot = match kind {
        ReadOutcomeKind::Decimated => inner
            .last_served_seq
            .map(u64::from)
            .unwrap_or(STAGE_NO_FRAME),
        ReadOutcomeKind::NoFrame => STAGE_NO_FRAME,
        // The two ANNOTATION kinds never reach here through a
        // caller-chosen path — `Truncated` is minted by `drain_into` and
        // `Producer` by `record_with_producer`, each building its own record
        // — but the match is exhaustive so a future variant cannot be added
        // without deciding what its served-seq slot means.
        ReadOutcomeKind::Truncated | ReadOutcomeKind::Producer => STAGE_NO_FRAME,
        ReadOutcomeKind::Served | ReadOutcomeKind::Held | ReadOutcomeKind::DrainedBatch => {
            served_seq.map(u64::from).unwrap_or(STAGE_NO_FRAME)
        }
    };
    if matches!(
        kind,
        ReadOutcomeKind::Served | ReadOutcomeKind::DrainedBatch
    ) {
        if let Some(seq) = served_seq {
            inner.last_served_seq = Some(seq);
        }
    }
    slot
}

/// The smallest power of ten strictly greater than `n` — the next decade
/// boundary of the overflow re-announcement ladder.
fn next_decade_above(n: u64) -> u64 {
    let mut d = 10u64;
    while d <= n {
        d = d.saturating_mul(10);
    }
    d
}

/// The shared per-(node, input) staging cell. See the module doc for the
/// sharing/synchronization contract.
#[derive(Debug)]
pub struct ReadOutcomeStage {
    /// This input's index into its node's manifest input table
    /// (`trace_ring::encode_manifest_with_inputs` — wiring order), fixed at
    /// creation.
    input_idx: u16,
    /// Which read path this stage observes (see [`ReadStageRole`]), fixed at
    /// creation — the run-exit drop report's disambiguator for the two
    /// same-`input_idx` stages a Separate/Sync input carries.
    role: ReadStageRole,
    /// This stage's record capacity — the rim `record` drops at and
    /// the reservation `arm` makes. DERIVED at creation
    /// ([`derive_stage_capacity`]); a REPLAY may then adopt the rim its
    /// recording declared via
    /// `arm_at_recorded_capacity`, which is the only writer and runs once,
    /// before step 0.
    ///
    /// `AtomicU32` rather than `u32` purely so that one pre-step adoption can
    /// happen through `&self` (every stage is held behind an `Arc`). Layout is
    /// unchanged — `AtomicU32` is `repr(transparent)` over a `u32` cell, and
    /// the ABI pin on this struct is a FIELD SET, whose names are untouched —
    /// so the cdylib ABI stays 18.
    ///
    /// It is stored rather than read back off `Vec::capacity()` because
    /// `reserve_exact` is documented as permitted to over-allocate, and the
    /// truncation SHAPE (which records survive at the rim) is a replay-compat
    /// parameter: a rim read off the allocator would make it
    /// allocator-dependent. This field is a deterministic function of its
    /// INPUTS — no run fact, no clock, no allocator — and is fixed before
    /// step 0.
    ///
    /// SCOPED, because "the graph" is not the whole input set: one of them is
    /// the loaded ENTRY's `holds_input_snapshot` capability. A cdylib REBUILT
    /// between record and replay — gaining or losing the snapshot
    /// symbols — derives 4 on one side and 4096 on the other from a byte-
    /// identical graph, and the format token matches on both sides, so the
    /// read-log verifier does NOT stand down. That is the correct outcome (the
    /// node really did behave differently, and the divergence is the finding),
    /// but it means this number is a function of the graph AND the binaries it
    /// loads. The per-input manifest capacities are what make such a
    /// capability skew DETECTABLE rather than merely correct.
    capacity: AtomicU32,
    /// Armed by `GraphRuntime::set_trace_ring_producer` (recording runs
    /// only). Disarmed = every `record` is a load+branch no-op.
    armed: AtomicBool,
    /// Staged-record count mirror — lets the merge skip empty stages without
    /// taking the lock (one `Acquire` load per stage per level).
    pending: AtomicU32,
    inner: Mutex<StageInner>,
}

impl ReadOutcomeStage {
    /// A DISARMED stage for `input_idx` (no buffer reservation — a
    /// never-recording run pays one `Arc` per wired input at build and
    /// nothing after). `pub(crate)` with the other mutators: the stage's
    /// lifecycle is owned by the runtime/scheduler wiring; only the POD
    /// record types and the read-side accessors are crate-external surface.
    ///
    /// The capacity is DERIVED HERE from `sizing`, never passed in.
    /// There is no constructor that takes a raw number, so an underived, odd,
    /// below-floor or above-ceiling capacity is unrepresentable — and the stage
    /// ROLE is named exactly once, at this call, instead of being restated in a
    /// sizing record nothing reconciled it against.
    pub(crate) fn new(input_idx: u16, role: ReadStageRole, sizing: ReadStageSizing) -> Self {
        Self {
            input_idx,
            role,
            capacity: AtomicU32::new(derive_stage_capacity(role, sizing)),
            armed: AtomicBool::new(false),
            pending: AtomicU32::new(0),
            inner: Mutex::new(StageInner::default()),
        }
    }

    /// This input's manifest input index.
    #[inline]
    pub fn input_idx(&self) -> u16 {
        self.input_idx
    }

    /// Which read path this stage observes (fixed at creation).
    #[inline]
    pub fn role(&self) -> ReadStageRole {
        self.role
    }

    /// This stage's DERIVED record capacity — the rim `record` drops
    /// at, and what `arm` reserves. Observable (Principle #3) because it is a
    /// replay-compat parameter the manifest stamps per input, and
    /// because an operator reading an OVERFLOW MARKER needs the number the
    /// stage was actually sized to.
    ///
    /// `u32`, the type the field and the manifest row both use, so
    /// there is ONE conversion point — the two `usize` consumers inside this
    /// module (the `Vec` reservation and the rim comparison) cast locally.
    #[inline]
    pub fn capacity(&self) -> u32 {
        self.capacity.load(Ordering::Relaxed)
    }

    /// Records dropped at capacity over this stage's LIFETIME — an
    /// unconditional running total, never reset by a drain or a re-arm
    /// (Principle #3; the run-exit drop report reads this).
    pub fn dropped(&self) -> u64 {
        self.lock_inner().dropped
    }

    /// Arm the stage: reserve its DERIVED capacity ONCE (cold — recording
    /// setup, before step 0) and open the record gate. Reserves only the
    /// SHORTFALL to the capacity target, so a re-arm (the idempotent
    /// double-install path) never grows the buffer past the armed bound.
    ///
    /// The target is the stage's own [`Self::capacity`], derived at
    /// wiring time from this edge's graph facts — not one global number every
    /// stage in the process shared.
    pub(crate) fn arm(&self) {
        // WARM the fold switch here, off the recording path.
        // `fold_enabled()` resolves through a `OnceLock` whose initialiser calls
        // `std::env::var`, which ALLOCATES — and its first caller was `record`,
        // under this stage's lock, on the documented zero-alloc path. Resolving
        // it at arm time makes every later read a plain atomic load. (The
        // steady-state alloc probes cannot see it: they warm up first, so the one
        // allocation lands before the measured window.)
        let _ = fold_enabled();
        let mut inner = self.lock_inner();
        let shortfall = (self.capacity() as usize).saturating_sub(inner.records.capacity());
        inner.records.reserve_exact(shortfall);
        drop(inner);
        self.armed.store(true, Ordering::Release);
    }

    /// TEST SEAM: drive the last staged record's fold count
    /// directly, so the `u32::MAX` saturation boundary is reachable without
    /// four billion real `record` calls.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn set_last_run_count_for_test(&self, n: u32) {
        let mut inner = self.lock_inner();
        if let Some(last) = inner.records.last_mut() {
            last.run_count = n;
        }
    }

    /// Arm this stage at the rim the RECORDING declared, instead of
    /// at the one this binary derived.
    ///
    /// The truncation shape is a format parameter of the recording,
    /// so a replay adopts it rather than re-deriving one — the
    /// `StaticCatchupArm` precedent. Called once per stage, before step 0, from
    /// `GraphRuntime::enable_read_outcome_memory_sink_with_recorded_capacities`.
    ///
    /// Takes an [`AdoptedRim`], so there is no clamp left to write here: a rim
    /// outside this binary's window was REFUSED where the bag was read, loudly
    /// and by name, and never reaches a stage. (A `clamp` here would make
    /// the two ends of the window behave differently for no reason anyone
    /// chose — above the ceiling would stand the verifier down, below the floor would be
    /// silently raised to 4, adopting a truncation shape the bag never
    /// declared.)
    pub fn arm_at_recorded_capacity(&self, recorded: AdoptedRim) {
        self.capacity.store(recorded.get(), Ordering::Relaxed);
        self.arm();
    }

    /// Whether a recording has armed this stage. The subscriber checks this
    /// BEFORE parsing anything (recording off ⇒ no header parse beyond what
    /// already exists on the drain path).
    #[inline]
    pub fn is_armed(&self) -> bool {
        self.armed.load(Ordering::Acquire)
    }

    /// Stage one read outcome. `served_seq` is the served frame's wire
    /// sequence where the kind carries one (`Served` / `Held` /
    /// `DrainedBatch`); for `Decimated` it is IGNORED and the stage's
    /// remembered last-served sequence is reported instead; for `NoFrame` it
    /// is ignored (sentinel). `popped` saturates to `u32` for the packed aux
    /// word. A disarmed stage records nothing; a full stage drops + counts.
    ///
    /// `role` is the CALL-SITE role (see [`ReadSiteRole`]),
    /// DECLARED by the caller as a compile-time constant — never inferred
    /// here, and never read off the stage (one stage holds both roles).
    pub(crate) fn record(
        &self,
        kind: ReadOutcomeKind,
        served_seq: Option<u32>,
        popped: u64,
        role: ReadSiteRole,
    ) {
        if !self.is_armed() {
            return;
        }
        let mut inner = self.lock_inner();
        let seq_slot = resolve_seq_slot(&mut inner, kind, served_seq);
        let incoming = StagedReadOutcome {
            kind,
            served_seq: seq_slot,
            popped: u32::try_from(popped).unwrap_or(u32::MAX),
            token: None,
            role,
            run_count: 1,
        };
        // FOLD a maximal run of byte-identical records.
        //
        // The key is the WHOLE record, not an enumerated field list, so a
        // future field on `StagedReadOutcome` joins the key by construction
        // instead of silently collapsing two records that differ in it. A
        // `Producer` annotation is never the fold target — a folded PAIR
        // carries its count on the READ half — and the count saturates
        // rather than wrapping.
        //
        // A `Period` node's catch-up burst repeats ONE non-consuming record
        // within a single step's staging window, and a maximal run of those
        // costs a single slot however long it is. That is a real saving, and it
        // is NOT a licence to drop `burst` from the derivation: folding is
        // CONSECUTIVE and BYTE-IDENTICAL only, so a burst whose reads differ in
        // any field — a served sequence that advances, a `popped` that changes,
        // the DELIVERED half of the "consumed or delivered" separator —
        // still costs one slot per read. The stage must therefore stay sized for
        // the worst case, which is what
        // `derive_stage_capacity` does.
        // Only a BareRead anchor is foldable here: a `PairRead` belongs to an
        // atomic pair (folding into it would launder the pair's provenance onto
        // a bare read) and a `Broken` one sits across a DROP (folding across it
        // would claim an adjacency the stream did not have). See [`FoldAnchor`].
        if fold_enabled() && inner.fold_anchor == FoldAnchor::BareRead {
            if let Some(last) = inner.records.last_mut() {
                if last.run_count < u32::MAX && records_fold(last, &incoming) {
                    last.run_count += 1;
                    let len = inner.records.len() as u32;
                    drop(inner);
                    self.pending.store(len, Ordering::Release);
                    return;
                }
            }
        }
        if inner.records.len() >= self.capacity() as usize {
            // NEVER grow past the armed reservation (zero-alloc contract);
            // the drop is counted, MARKED at the next merge, and warned.
            // The anchor BREAKS: whatever lands next was not adjacent to the
            // record before this hole.
            inner.dropped += 1;
            inner.fold_anchor = FoldAnchor::Broken;
            return;
        }
        inner.fold_anchor = FoldAnchor::BareRead;
        inner.records.push(incoming);
        let len = inner.records.len() as u32;
        drop(inner);
        self.pending.store(len, Ordering::Release);
    }

    /// Stage one read outcome PRECEDED by its
    /// [`ReadOutcomeKind::Producer`] annotation — the multi-publisher-edge
    /// form of [`Self::record`]. `token` is the 64-bit producer token (see
    /// `ReadOutcomeKind::Producer`); every other parameter means exactly what
    /// it does on `record`.
    ///
    /// # Atomic-pair admission at the rim
    ///
    /// The pair is admitted BOTH or NEITHER (`len + 2 <= CAPACITY`). A split
    /// would leave a WIDOWED token — a `Producer` record whose binding rule
    /// ("the next read on this edge") points at a read that was dropped, so
    /// an offline reader would silently attribute the FOLLOWING window's
    /// first read to the wrong publisher. Refusing both instead costs one
    /// read record and is fully described: the drop is counted and the
    /// window's [`ReadOutcomeKind::Truncated`] marker reports it, which is a
    /// marked hole rather than a wrong answer. Both sides run this same rule
    /// at the same capacity, so the truncation stays deterministic.
    ///
    /// BOTH records carry the caller's `role` — the annotation
    /// and the read it names were produced by the same call site, so a reader
    /// steering the pair by role can never split them.
    pub(crate) fn record_with_producer(
        &self,
        kind: ReadOutcomeKind,
        served_seq: Option<u32>,
        popped: u64,
        token: u64,
        role: ReadSiteRole,
    ) {
        if !self.is_armed() {
            return;
        }
        let mut inner = self.lock_inner();
        let seq_slot = resolve_seq_slot(&mut inner, kind, served_seq);
        // Fold the PAIR. The stage's last two records must be a
        // `(Producer, read)` pair identical to the incoming one; the count then
        // goes on the READ half and the annotation's is implied. A fold
        // consumes no slots, so the atomic-pair rim check below is never
        // reached on a folding path — the pair stays all-or-nothing.
        //
        // Gated on a `PairRead` anchor, so only a pair folds
        // into a pair — and the two candidate records are built and compared
        // through the SAME `records_fold` key the plain path uses, rather than
        // hand-enumerated here. A field added to `StagedReadOutcome` tomorrow
        // joins this key by construction; a hand-enumerated copy would fold
        // two records differing in it.
        let (annotation, read) = self.pair_records(kind, seq_slot, popped, token, role);
        if fold_enabled() && inner.fold_anchor == FoldAnchor::PairRead && inner.records.len() >= 2 {
            let n = inner.records.len();
            let read_matches = inner.records[n - 1].run_count < u32::MAX
                && records_fold(&inner.records[n - 1], &read);
            let annotation_matches = annotation_folds(&inner.records[n - 2], &annotation);
            if read_matches && annotation_matches {
                inner.records[n - 1].run_count += 1;
                let len = inner.records.len() as u32;
                drop(inner);
                self.pending.store(len, Ordering::Release);
                return;
            }
        }
        if inner.records.len() + 2 > self.capacity() as usize {
            // Both or neither — see the doc above. Counted as TWO, because
            // two records really were refused. The anchor BREAKS for the same
            // reason it does on the plain path: the next record is not adjacent
            // to the one before this hole.
            inner.dropped += 2;
            inner.fold_anchor = FoldAnchor::Broken;
            return;
        }
        inner.fold_anchor = FoldAnchor::PairRead;
        inner.records.push(annotation);
        inner.records.push(read);
        let len = inner.records.len() as u32;
        drop(inner);
        self.pending.store(len, Ordering::Release);
    }

    /// The `(annotation, read)` pair `record_with_producer` stages — built in
    /// ONE place so the records it PUSHES and the records it compares a fold
    /// against cannot drift apart.
    fn pair_records(
        &self,
        kind: ReadOutcomeKind,
        seq_slot: u64,
        popped: u64,
        token: u64,
        role: ReadSiteRole,
    ) -> (StagedReadOutcome, StagedReadOutcome) {
        (
            StagedReadOutcome {
                kind: ReadOutcomeKind::Producer,
                // The annotation REPEATS the annotated read's served sequence
                // (the redundant join key) so a reader that lost the binding
                // can still pair them.
                served_seq: seq_slot,
                popped: 0,
                token: Some(token),
                role,
                run_count: 1,
            },
            StagedReadOutcome {
                kind,
                served_seq: seq_slot,
                popped: u32::try_from(popped).unwrap_or(u32::MAX),
                token: None,
                role,
                run_count: 1,
            },
        )
    }

    /// Drain every staged record into `f` in stage order (chronological
    /// within the writer's discipline) and clear the stage. Empty stages are
    /// skipped with one `Acquire` load — no lock. Surfaces the overflow
    /// condition on the merge thread: a loud `warn!` head on the first drain
    /// that observes drops, then RE-ANNOUNCEMENTS only at each DECADE of the
    /// unconditional running total (the decade discipline).
    ///
    /// Deliberately NOT `FailureRegimeLatch` (the house machine): its
    /// per-failure `Loud`/`Suppressed` vocabulary wants a decision at each
    /// FAILURE site, but a stage's failures land on the recording hot path
    /// (`record`), where this module refuses to make logging decisions, and
    /// its recovery re-arm maps to "the drain freed capacity" — which happens
    /// at EVERY merge, so a regime-per-burst mapping would re-open (and
    /// re-announce loudly) once per step on a persistently-overflowing edge.
    /// The minimal decade ladder over the existing counter, evaluated at the
    /// drain cadence, gives the same bounded re-announcement (`log10(total)`
    /// lines, ever) without either distortion.
    /// `f` returns whether the record was HANDED OVER — accepted by at least
    /// one consumer (the trace ring, the in-memory sink) rather than dropped
    /// on the floor. Only the OVERFLOW MARKER's answer is read; a survivor's
    /// is ignored, because a survivor carries no debt (it is either bagged or
    /// it is not, and the run-exit drop report already accounts for the
    /// unmapped-node class).
    pub(crate) fn drain_into(&self, mut f: impl FnMut(StagedReadOutcome) -> bool) {
        if self.pending.load(Ordering::Acquire) == 0 {
            return;
        }
        let mut inner = self.lock_inner();
        for rec in inner.records.drain(..) {
            let _ = f(rec);
        }
        // The fold anchor is BROKEN by the drain.
        //
        // A run never spans two merge windows. That would hold anyway
        // because the records are gone — `records.last_mut()` finds nothing, so
        // nothing can fold. That is an accident of one implementation, not the
        // invariant: the anchor is what the fold CONSULTS, and leaving it on
        // `BareRead` across a drain states that the next window's first record
        // is adjacent to the previous window's last. It would also make
        // `FoldAnchor`'s own "set on an empty stage" false for every window
        // after the first. Reset it where the stage empties.
        //
        // It sits BELOW the `pending == 0` early return, and that is correct
        // rather than incidental: `pending` is stored as
        // `records.len()` by every push and zeroed by this drain, so `pending == 0`
        // holds exactly when the record vector is EMPTY. On that path there is no
        // last record to fold into, so the fold's `records.last_mut()` finds
        // nothing whatever the anchor says — its value is unobservable until the
        // next push sets it. A window that staged nothing therefore needs no
        // reset, and one that staged anything gets it here.
        inner.fold_anchor = FoldAnchor::Broken;
        // The OVERFLOW MARKER, emitted LAST in the window —
        // after every survivor — because that is where the dropped records
        // would have been: `record` drops at the RIM (newest-first), so the
        // survivors are this window's PREFIX and the hole is its tail. One
        // marker per window per input, carrying the window's DELTA (never
        // the lifetime total — see `StageInner::dropped_reported`), and none
        // at all in a window that dropped nothing, so a healthy edge's
        // stream carries no marker at all.
        //
        // # The debt is retired only on a CONFIRMED hand-over
        //
        // `dropped_reported` must not advance the instant the marker is
        // OFFERED. A drain's consumer can REFUSE it — `push_read_outcome`
        // returns without writing when the node id is missing from the
        // recording manifest (a manifest/scheduler desync), and with no
        // in-memory sink installed that is every consumer there is. Retiring
        // on the offer would therefore lose the marker AND the debt in one move: the
        // hole stays in the bag and nothing ever describes it again,
        // because the next window's delta is computed against the count this
        // line just advanced.
        //
        // Retiring on ACCEPTANCE instead means a refused marker's count
        // accumulates into the NEXT window's delta and is re-reported there.
        // RESIDUAL, stated because it is real and not fixable here: the LAST
        // window of a run has no next window, so a marker refused at shutdown
        // is lost. What survives it is the run-exit drop report
        // (`GraphRuntime::read_outcome_dropped_report`), which is
        // unconditional and counts every drop whether or not a marker carried
        // it — so the loss is one self-describing record offline, never a
        // silent count.
        let dropped_this_window = inner.dropped.saturating_sub(inner.dropped_reported);
        if dropped_this_window > 0 {
            let handed_over = f(StagedReadOutcome {
                kind: ReadOutcomeKind::Truncated,
                served_seq: STAGE_NO_FRAME,
                popped: u32::try_from(dropped_this_window).unwrap_or(u32::MAX),
                token: None,
                // The marker is minted BY THE STAGE, so the only
                // role it can carry is the STAGE's own — "the stage that
                // overflowed", DIAGNOSTIC ONLY. It is deliberately not a
                // call-site role (a `Body` stage holds drain-site records
                // under the unified discipline), and no consumer steers on
                // it: a `Truncated` marker goes to BOTH injection queues and
                // raises `TruncatedReadLog` under EVERY role value.
                role: ReadSiteRole::from(self.role),
                // A `Truncated` marker is NEVER folded. It is
                // minted by `drain_into` rather than by `record`, and it
                // carries THIS window's drop delta — two markers from two
                // windows describe different losses even when their fields
                // coincide, so collapsing them would destroy the one fact the
                // marker exists to report.
                run_count: 1,
            });
            if handed_over {
                inner.dropped_reported = inner.dropped;
            }
        }
        if inner.dropped > 0 {
            if !inner.dropped_announced {
                inner.dropped_announced = true;
                inner.next_decade = next_decade_above(inner.dropped);
                // `dropped=` is the head burst (what THIS regime opened with);
                // `total_failures=` is the shared-latch key every flood
                // reporter in the repo logs the running total under (the spellings
                // were unified so that one grep answers "how bad?").
                tracing::warn!(
                    input_idx = self.input_idx,
                    dropped = inner.dropped,
                    total_failures = inner.dropped,
                    capacity = self.capacity(),
                    "read-outcome staging overflowed its fixed capacity — records were \
                     dropped (the read log for this input has holes). Report this: the \
                     capacity is DERIVED from this edge's own graph facts for the \
                     level-end merge cadence. Re-announcing \
                     at each decade of the running total."
                );
            } else if inner.dropped >= inner.next_decade {
                inner.next_decade = next_decade_above(inner.dropped);
                tracing::warn!(
                    input_idx = self.input_idx,
                    total_failures = inner.dropped,
                    capacity = self.capacity(),
                    "read-outcome staging is STILL overflowing — the running dropped \
                     total crossed another decade (the read log for this input keeps \
                     losing records)"
                );
            }
        }
        drop(inner);
        self.pending.store(0, Ordering::Release);
    }

    /// The uncontended lock, poison-tolerant: a stage is a DIAGNOSTIC plane
    /// and must never wedge (or panic-cascade) the step/merge path it
    /// observes — the `failure_regime_latch::lock_regime_latch` posture. The
    /// data is POD records, structurally valid whatever the panicking thread
    /// was doing.
    fn lock_inner(&self) -> std::sync::MutexGuard<'_, StageInner> {
        match self.inner.lock() {
            Ok(g) => g,
            Err(poisoned) => poisoned.into_inner(),
        }
    }
}

// ===========================================================================
// THE ENFORCEMENT SIDE: what the RECORDING says a stage's reads were, and the
// gate that admits exactly those.
//
// Everything below is the twin of the capture side above. The capture stage
// answers "what did this read do"; the plan stage answers "what did the
// recording's read at this position do, and how much of that step's quota is
// unspent". The two share `StageKey`, `derive_stage_capacity` and
// `StagedReadOutcome`, so a field added to the capture record joins the
// enforcement key by construction instead of collapsing a distinction on one
// side only.
// ===========================================================================

/// What ONE recorded read at this stage did, in the THREE shapes a bag can
/// state. The shape is a variant and not a count, because a count admits one
/// state for two different facts: "the read consumed nothing and served
/// nothing" and "the read served a frame and consumed nothing" both write zero.
///
/// # The third shape is already in the tree
///
/// A SERVED read that popped nothing is recorded today as a `DrainedBatch` with
/// `popped == 0`, the hand off a promotion mints, which the engine's injection
/// folder already skips as occupying no position in the pop stream. Collapsing
/// it onto [`Self::Nothing`] would withhold a frame at the step the recording
/// says it was SERVED, mint neither a never-arrives nor a shortfall for it (an
/// unspent none owes nothing, and a settle of 0 against 0 agrees), and let the
/// divergence surface downstream as a byte difference charged to the candidate:
/// a silent wrong verdict. Refusing the whole run on the shape is not the answer
/// either, since that would make every graph carrying a promotion unreplayable.
///
/// # These three are the WHOLE vocabulary
///
/// A future [`ReadOutcomeKind`] that maps to none of them has no admission
/// meaning here, and an unknown kind reaching a verdict is an exit 6 schedule
/// divergence rather than a silent fourth meaning. So adding a kind is a TRACE
/// FORMAT bump, with this enum's arms as the cost: a new kind must state which
/// of the three it is, or declare a fourth here and take the format bump.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DueShape {
    /// The recorded read CONSUMED `pops` frames off the queue.
    Pops {
        /// Frames this read must consume. Never 0, a pop of nothing is one of
        /// the two shapes below.
        pops: u32,
        /// True only for a recorded [`ReadOutcomeKind::Decimated`] read, whose
        /// served sequence names the last ACCEPTED sequence and not what this
        /// read popped, so the COUNT is enforced and the identity is not.
        identity_unstated: bool,
    },
    /// The recorded read served NOTHING and consumed nothing: a recorded none,
    /// or a held read that took no new frame. It is a POSITION a gated consult
    /// really occupied, the recording's drain ran there and got nothing, so
    /// the gate withholds at it and never scans past it.
    Nothing,
    /// The recorded read SERVED its sequence and consumed nothing: the frame
    /// reached the consumer WITHOUT a dequeue, at a site that pops nothing (a
    /// promotion), or the serving site is another stage and this record is
    /// COVERED there.
    ///
    /// Either way it occupies no position in this gate's pop stream, so it is
    /// TRANSPARENT: a consult skips past it to the next real entry, and the
    /// end-of-step sweep never mints for it. Neither withheld nor swept.
    ServedWithoutPop,
}

/// One recorded read at this stage: its shape and the sequence it served.
///
/// The fields are PRIVATE with named constructors, so a pop count of zero on
/// the [`DueShape::Pops`] arm and a `served_seq` on an arm that serves nothing
/// are unconstructible rather than documented as not happening.
///
/// It carries one sequence and not a list because the recording carries one: a
/// batch's record names its NEWEST sequence and its pop count, never the
/// sequences in between, so naming a range here would state an order the bag
/// does not.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DueRead {
    /// Which of the three shapes this read is.
    shape: DueShape,
    /// The sequence the recorded read SERVED (`None` = it served no frame).
    served_seq: Option<u32>,
}

impl DueRead {
    /// A read that consumed `pops` frames, the LAST surviving of which carried
    /// `served_seq`. A `pops` of 0 is NOT this shape and is normalised to
    /// [`Self::nothing`] or [`Self::served_without_pop`] by what it served, so
    /// a caller folding a record cannot mint a pop of nothing.
    #[must_use]
    pub fn pops(pops: u32, served_seq: Option<u32>) -> Self {
        Self::pops_with_identity(pops, served_seq, false)
    }

    /// [`Self::pops`] for a recorded [`ReadOutcomeKind::Decimated`] read: the
    /// count is enforced and the identity is not.
    #[must_use]
    pub fn decimated(pops: u32, last_accepted: Option<u32>) -> Self {
        Self::pops_with_identity(pops, last_accepted, true)
    }

    /// A read that served nothing and consumed nothing.
    #[must_use]
    pub const fn nothing() -> Self {
        Self {
            shape: DueShape::Nothing,
            served_seq: None,
        }
    }

    /// A read that SERVED `served_seq` and consumed nothing (a hand off).
    #[must_use]
    pub const fn served_without_pop(served_seq: Option<u32>) -> Self {
        Self {
            shape: DueShape::ServedWithoutPop,
            served_seq,
        }
    }

    /// Which of the three shapes this read is.
    #[must_use]
    pub fn shape(&self) -> DueShape {
        self.shape
    }

    /// The sequence the recorded read served.
    #[must_use]
    pub fn served_seq(&self) -> Option<u32> {
        self.served_seq
    }

    /// Frames this read must consume: 0 on both pop-less shapes.
    #[must_use]
    pub fn pop_count(&self) -> u32 {
        match self.shape {
            DueShape::Pops { pops, .. } => pops,
            DueShape::Nothing | DueShape::ServedWithoutPop => 0,
        }
    }

    fn pops_with_identity(pops: u32, served_seq: Option<u32>, identity_unstated: bool) -> Self {
        if pops == 0 {
            // A pop of nothing is one of the two pop-less shapes, decided by
            // what the record SERVED. Normalising here is what keeps
            // `DueShape::Pops { pops: 0 }` out of the type entirely.
            return match served_seq {
                Some(_) => Self::served_without_pop(served_seq),
                None => Self::nothing(),
            };
        }
        Self {
            shape: DueShape::Pops {
                pops,
                identity_unstated,
            },
            served_seq,
        }
    }
}

/// One gated stage's admissions for ONE step, in RECORDED ORDER.
///
/// An EMPTY slice is a DECLARATION ("this stage reads nothing this step"), and
/// an armed stage the per-step call OMITS is the same declaration. The two are
/// never observably different because the engine installs an entry for every
/// armed stage at every step.
#[derive(Debug, Clone, Copy)]
pub struct ReadPlanEdge<'a> {
    /// Which stage this step's admissions belong to.
    pub key: &'a StageKey,
    /// This step's reads at this stage, in RECORDED ORDER.
    pub due: &'a [DueRead],
}

/// Why a wired stage can never be GATED, decided from WIRING facts at build
/// time and fixed for the run.
///
/// It is a two-variant enum on the stage rather than a pair of bools because
/// the census renders exactly one reason per stage, and two bools admit a
/// fourth state (both set) nothing would choose between.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateBlocker {
    /// A per-set `Sync` trigger input. Its pops leave the two gated drain
    /// bodies: `sync_peek_next_stamp` and `sync_discard_head` remove frames
    /// from the queue through the matcher's ops, so a gate installed here
    /// would be bypassed rather than enforced.
    PerSetSyncEdge,
    /// A `multi_publisher_topics` edge. The wire sequence is a PER-PUBLISHER
    /// counter, so the due frame is not necessarily the head and a served seq
    /// names no producer.
    MultiPublisherEdge,
}

impl GateBlocker {
    /// The stable token an operator-facing refusal carries.
    #[must_use]
    pub fn code(self) -> &'static str {
        match self {
            Self::PerSetSyncEdge => "per_set_sync_edge",
            Self::MultiPublisherEdge => "multi_publisher_edge",
        }
    }
}

/// What the core refuses to GATE, on WIRING facts alone. Every arm names the
/// stage; the engine renders the input name from its own manifest.
///
/// A bag fact is NOT in this vocabulary: the core cannot see a bag, so no
/// coverage, a truncated stream, a dropped record and every unenforceable
/// record shape are refused by the engine's planner before step 0.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReadPlanRefusal {
    /// No wired stage carries this key.
    UnknownKey(StageKey),
    /// The same key appeared twice in one call.
    DuplicateKey(StageKey),
    /// See [`GateBlocker::PerSetSyncEdge`].
    PerSetSyncEdge(StageKey),
    /// See [`GateBlocker::MultiPublisherEdge`].
    MultiPublisherEdge(StageKey),
    /// The gate's mutex was poisoned by a panic; the stage refuses every later
    /// admission rather than falling back to a live pop.
    GateUnusable(StageKey),
}

impl ReadPlanRefusal {
    /// The refused stage.
    #[must_use]
    pub fn key(&self) -> &StageKey {
        match self {
            Self::UnknownKey(k)
            | Self::DuplicateKey(k)
            | Self::PerSetSyncEdge(k)
            | Self::MultiPublisherEdge(k)
            | Self::GateUnusable(k) => k,
        }
    }

    /// The stable token, one per cause.
    #[must_use]
    pub fn code(&self) -> &'static str {
        match self {
            Self::UnknownKey(_) => "unknown_key",
            Self::DuplicateKey(_) => "duplicate_key",
            Self::PerSetSyncEdge(_) => GateBlocker::PerSetSyncEdge.code(),
            Self::MultiPublisherEdge(_) => GateBlocker::MultiPublisherEdge.code(),
            Self::GateUnusable(_) => "gate_unusable",
        }
    }
}

impl std::fmt::Display for ReadPlanRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "stage {} refuses the read gate ({})",
            self.key().label(),
            self.code()
        )
    }
}

/// One wired stage's answer to "can this edge be gated", read before that
/// rank's first step.
///
/// The census is a read-only walk, so a refusal built on it lands before the rank
/// it refuses has executed anything. The SCOPE is the rank and not the run,
/// because the rows are this RUNTIME's wiring and a replay stands one runtime up
/// per rank in rank order: a recording whose read log cannot cover one of its
/// edges is refused before the first step, while a replay whose wiring differs
/// from the recording's is refused before that rank's first step. A stage this
/// row says cannot be gated is never a refusal at all: it takes today's drain
/// and the report NAMES it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReadEdgeCapability {
    /// The stage this row describes.
    pub key: StageKey,
    /// Whether `ReadPlanStage::arm` would succeed.
    pub enforceable: bool,
    /// Why not, when `enforceable` is false.
    pub reason: Option<ReadPlanRefusal>,
}

/// A recorded read the re-execution did not reproduce.
///
/// Three kinds for what an operator reads as one story, kept apart because
/// they distinguish "the drain never ran" from "the drain ran and the queue
/// was short" from "the drain ran and the head was a different frame", which
/// is the difference between a missing fire, a missing frame and a reordered
/// stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplayReadViolationKind {
    /// A [`DueRead`] with `pops > 0` that no drain consulted this step.
    NeverArrived {
        /// The sequence the recording read at that position.
        sequence: Option<u32>,
    },
    /// A drain ran and delivered fewer frames than the quota.
    PoppedShortfall {
        /// The recorded pop count.
        expected: u32,
        /// What the drain actually delivered.
        admitted: u32,
    },
    /// The surviving frame carried a different sequence.
    SequenceMismatch {
        /// The recorded surviving sequence.
        expected: Option<u32>,
        /// The sequence the replay's surviving frame carried.
        observed: Option<u32>,
    },
}

impl ReplayReadViolationKind {
    /// The stable token the engine renders and keys its JSON on.
    #[must_use]
    pub fn code(self) -> &'static str {
        match self {
            Self::NeverArrived { .. } => "never_arrived",
            Self::PoppedShortfall { .. } => "popped_shortfall",
            Self::SequenceMismatch { .. } => "sequence_mismatch",
        }
    }
}

/// One divergence between the recording's read and the replay's, naming the
/// stage and the step.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReplayReadViolation {
    /// The stage that diverged.
    pub key: StageKey,
    /// The step it diverged at.
    pub step: u64,
    /// How it diverged.
    pub kind: ReplayReadViolationKind,
}

/// What one gate consult answers.
///
/// `Ungated` and `Withhold` are two different facts and not one: `Ungated`
/// means no plan is armed on this edge at all, so the drain takes today's path
/// byte for byte, while `Withhold` means an armed plan's recorded read at this
/// position popped nothing. Collapsing them would make an unarmed edge and a
/// zero-quota step indistinguishable, and the live path could then no longer be
/// proven untouched.
///
/// `#[must_use]`: a consult that drops its answer takes today's drain over a
/// gated edge, and the `Exact` arm's settle is the only place a shortfall or a
/// sequence difference is minted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[must_use]
pub enum GateAnswer {
    /// No armed plan: take today's path.
    Ungated,
    /// An armed plan admits nothing here. The frame, if any, stays in its own
    /// queue in arrival order with its bytes never read.
    Withhold,
    /// Pop exactly this admission.
    Exact(DueRead),
}

/// The enforcement twin of [`ReadOutcomeStage`]: what the RECORDING says this
/// stage's reads were for the step being replayed, and how much of that step's
/// quota is unspent.
///
/// # Why it lives on the subscriber and not on the capture stage
///
/// Hanging the plan off [`ReadOutcomeStage`] needs the same ABI bump (the read
/// outcome chain is in the same FFI-covered closure) and makes `armed` mean two
/// things, one armed by the recording and one by the replay.
#[derive(Debug)]
pub struct ReadPlanStage {
    /// node + `input_idx` + [`ReadStageRole`], the same key the capture twin
    /// is enumerated by, fixed at creation.
    key: StageKey,
    /// The bound on the RETAINED DIVERGENCE LIST, and nothing else, the SAME
    /// number the capture twin derives.
    ///
    /// It does NOT bound the installed step plan. The number is a bound on this
    /// stage's RECORDS within one merge window
    /// ([`derive_stage_capacity`]), while a step plan is the recording's READS
    /// with folded runs expanded, so the two are in different units and the
    /// smaller one was never the larger one's ceiling (see
    /// [`ReadPlanStage::install_step`]).
    rim: u32,
    /// Why this stage can never be gated, from wiring facts (see
    /// [`GateBlocker`]). `None` on a gateable stage.
    blocker: Option<GateBlocker>,
    /// False until `arm` names this key. An unarmed stage is UNGATED for the
    /// whole run.
    armed: AtomicBool,
    /// The step the RUNTIME is executing, written once per step by
    /// `GraphRuntime::step_live` through the clone every plan stage shares.
    ///
    /// The consult reads it rather than taking a step argument because the
    /// drain sites it is called from do not carry one: a subscriber knows its
    /// queue and nothing about the schedule. One shared cell is also what makes
    /// the stale check a single integer compare instead of a per-site
    /// parameter thread.
    now_step: Arc<AtomicU64>,
    /// Uncontended, replay only, and mirroring [`ReadOutcomeStage`]'s own
    /// interior mutability for the same reason: `drain_samples` takes `&self`.
    inner: Mutex<PlanInner>,
}

/// One gated stage's mutable half.
#[derive(Debug, Default)]
struct PlanInner {
    /// The step this quota belongs to.
    step: u64,
    /// A step with no install is a ZERO quota, not a stale one.
    installed: bool,
    /// This step's reads at this stage, in RECORDED ORDER, runs expanded.
    /// Retained across steps (cleared and refilled), so a gated steady state
    /// allocates nothing.
    due: Vec<DueRead>,
    /// The next unspent [`DueRead`].
    cursor: usize,
    /// Frames admitted at this stage this step.
    admitted_this_step: u32,
    /// Frames DELIVERED under a grant over the run: a MONOTONE witness, never
    /// reset, written by `settle` from the drain's own count.
    lifetime_admitted: u64,
    /// CONSULTS the gate refused over the run: the other monotone witness, and
    /// the one the refill classifier reads to tell an enforced empty from an
    /// empty queue. One per refused consult and never one per queued frame, so
    /// a drain site added to a gated body moves it with no behaviour change.
    refused_consults: u64,
    /// Consults the plan held no position for: an armed stage with no install
    /// for the step, and a consult past a NON-EMPTY step plan the drains already
    /// spent. Counted apart from `refused_consults` because a recorded none and
    /// an empty step plan are the enforcement working, while these two are the
    /// plan and the drain disagreeing about how many reads one step holds.
    unplanned_consults: u64,
    /// Retained divergences, bounded by [`ReadPlanStage::rim`].
    violations: Vec<ReplayReadViolation>,
    /// Divergences the rim dropped. Nonzero means the report is INCOMPLETE,
    /// which the engine turns into a refusal rather than a silent truncation.
    violations_dropped: u64,
    /// Consults that found a plan installed for a DIFFERENT step. A harness
    /// fault, counted here and never carried in the violation list, because a
    /// candidate divergence and an install bug are two report classes.
    mismatches: u64,
    /// Set when the mutex was found poisoned. The stage then refuses every
    /// later admission instead of falling back to a live pop.
    unusable: bool,
}

impl ReadPlanStage {
    /// A DISARMED plan stage for `key` at `rim`, sharing the runtime's step
    /// cell. `pub(crate)` with the other mutators, the same rule
    /// [`ReadOutcomeStage::new`] states: the stage's lifecycle is owned by the
    /// runtime wiring, and only the POD types and the read-side accessors are
    /// crate-external surface.
    ///
    /// The rim is PASSED here rather than derived, because the caller derives
    /// it ONCE per edge through [`derive_stage_capacity`] and hands the same
    /// number to both twins: deriving it twice would let the plan and the
    /// capture disagree about a rim that is the same edge's. On this twin the
    /// number bounds the RETAINED DIVERGENCE LIST only; what bounds a step's
    /// plan is the recording, and [`Self::install_step`] states why the rim
    /// cannot.
    pub(crate) fn new(
        key: StageKey,
        rim: u32,
        blocker: Option<GateBlocker>,
        now_step: Arc<AtomicU64>,
    ) -> Self {
        Self {
            key,
            rim,
            blocker,
            armed: AtomicBool::new(false),
            now_step,
            inner: Mutex::new(PlanInner::default()),
        }
    }

    /// This stage's key.
    #[must_use]
    pub fn key(&self) -> &StageKey {
        &self.key
    }

    /// The bound on the retained divergence list. NOT a bound on one step's
    /// plan: see the field's own doc for the two units.
    #[must_use]
    pub fn rim(&self) -> u32 {
        self.rim
    }

    /// Can this edge be gated, and why not. Read BEFORE step 0.
    #[must_use]
    pub fn capability(&self) -> ReadEdgeCapability {
        let reason = self.wiring_refusal();
        ReadEdgeCapability {
            key: self.key.clone(),
            enforceable: reason.is_none(),
            reason,
        }
    }

    /// Arm the gate. An unarmed stage is UNGATED for the whole run.
    pub(crate) fn arm(&self) -> Result<(), ReadPlanRefusal> {
        if let Some(reason) = self.wiring_refusal() {
            return Err(reason);
        }
        self.armed.store(true, Ordering::Release);
        Ok(())
    }

    /// Disarm the gate (the `clear_replay_fire_plan` twin's per-stage half).
    /// The step's quota goes with it, so a re-arm cannot spend a retired one.
    pub(crate) fn disarm(&self) {
        self.armed.store(false, Ordering::Release);
        if let Ok(mut inner) = self.inner.lock() {
            inner.installed = false;
            inner.due.clear();
            inner.cursor = 0;
            inner.admitted_this_step = 0;
            // The FINDINGS go with the plan: a retired run's divergences read
            // back through `take_violations` after a disarm would be reported
            // against the next plan's steps. The two monotone lifetime
            // witnesses stay, they are the run's own totals.
            inner.violations.clear();
            inner.violations_dropped = 0;
            inner.mismatches = 0;
        }
    }

    /// Is the gate armed (Principle #3; false on a virgin runtime).
    #[inline]
    #[must_use]
    pub fn is_armed(&self) -> bool {
        self.armed.load(Ordering::Acquire)
    }

    /// Write this step's quota, retiring the previous step's wholesale.
    ///
    /// # The plan is bounded by the RECORDING, never by the rim
    ///
    /// `due` is this step's recorded reads with folded runs EXPANDED, one entry
    /// per read, because the gate answers one consult per read. The rim bounds
    /// this stage's RECORDS within one merge window, and the fold means those
    /// two counts are not the same number: a catch-up burst whose reads are
    /// byte identical is stored as ONE record carrying the count, and a
    /// `Period` node bursts to the derivation's ceiling, so a step the
    /// recording describes exactly can name more reads than the rim holds.
    /// Refusing that install rejected a replay the recording is a complete
    /// description of, which is why no bound is applied here.
    ///
    /// A FOREIGN or TRUNCATED recording is still refused, by name, where the
    /// bag is read rather than here: an overflow marker that dropped records
    /// (`read_log_record_dropped`), a recorded stage set this build does not
    /// wire (`read_log_stage_set_skew`), an input name that does not resolve or
    /// resolves twice (`read_log_input_name_unresolved`,
    /// `read_log_input_name_duplicated`), every unenforceable record shape
    /// (`read_log_unenforceable_record`), and a recorded rim outside this
    /// binary's window ([`AdoptedRim::from_recorded`]). The engine also expands
    /// runs under its own whole-bag record budget and DECLINES past it, so a
    /// crafted run count cannot make this allocation unbounded.
    ///
    /// `Err(GateUnusable)` when a panic poisoned the gate: it accepts no
    /// install and admits nothing from there on.
    pub(crate) fn install_step(&self, step: u64, due: &[DueRead]) -> Result<(), InstallRefusal> {
        let mut inner = self.lock_plan().map_err(|_| InstallRefusal::GateUnusable)?;
        inner.step = step;
        inner.installed = true;
        inner.due.clear();
        inner.due.extend_from_slice(due);
        inner.cursor = 0;
        inner.admitted_this_step = 0;
        Ok(())
    }

    /// One consult: take `due[cursor]` AS IT STANDS and advance.
    ///
    /// It never scans forward past an entry a drain can spend. That is what
    /// makes the recorded order load bearing: a step whose first recorded read
    /// at this stage served nothing admits nothing at the first consult even
    /// with a frame queued, and the second consult gets the second recorded
    /// read.
    ///
    /// The ONE entry the cursor walks past without answering is
    /// [`DueShape::ServedWithoutPop`]: that recorded read reached its consumer
    /// without a dequeue, so no drain consults for it and it occupies no
    /// position in this gate's pop stream. Withholding at it would hold back
    /// the frame the recording served, and the divergence would then reach a
    /// verdict as a byte difference charged to the candidate.
    #[inline(never)]
    pub(crate) fn admit(&self) -> GateAnswer {
        // An unarmed stage is UNGATED, which is a different fact from a zero
        // quota: the drain takes today's path byte for byte. Answered here so
        // the arm is produced by the one function that can know it, rather than
        // documented as unreachable at a caller that filters on `is_armed`.
        if !self.is_armed() {
            return GateAnswer::Ungated;
        }
        let now = self.now_step.load(Ordering::Acquire);
        let Ok(mut inner) = self.lock_plan() else {
            return GateAnswer::Withhold;
        };
        // The NOT-INSTALLED test comes FIRST, and the order is load bearing. An
        // armed stage the step's install omitted has a ZERO quota for that step;
        // it remembers whatever step number its last install carried, so a step
        // compare placed ahead of this arm would report every omitted edge as a
        // STALE PLAN, a harness fault, where the omission is instead the
        // declaration that this stage reads nothing this step. A stale plan is
        // only meaningful while one IS installed.
        if !inner.installed {
            inner.refused_consults = inner.refused_consults.saturating_add(1);
            inner.unplanned_consults = inner.unplanned_consults.saturating_add(1);
            return GateAnswer::Withhold;
        }
        if inner.step != now {
            inner.mismatches = inner.mismatches.saturating_add(1);
            // Counted as a refused consult too: the refill that follows this
            // withhold finds its queue empty, and the classifier reads this
            // witness to tell an enforced empty from an empty queue. Without it
            // a stale-plan withhold is reported as the recording holding a
            // frame the replay's input stream does not, which names the
            // candidate for a harness fault.
            inner.refused_consults = inner.refused_consults.saturating_add(1);
            return GateAnswer::Withhold;
        }
        loop {
            let Some(due) = inner.due.get(inner.cursor).copied() else {
                inner.refused_consults = inner.refused_consults.saturating_add(1);
                // An EMPTY step plan is a DECLARATION ("this stage reads nothing
                // this step"), and it is the ORDINARY shape: a drain that found
                // its queue empty writes no record at all, so most steps of a
                // gated stage name no read. A consult past a plan that DID hold
                // positions is the other fact: the step's records are spent and a
                // drain asked for one more, which is the plan and the drain
                // disagreeing about how many reads the step holds.
                if !inner.due.is_empty() {
                    inner.unplanned_consults = inner.unplanned_consults.saturating_add(1);
                }
                return GateAnswer::Withhold;
            };
            inner.cursor += 1;
            match due.shape() {
                // Transparent: the recorded read served its frame without a
                // dequeue, so this consult belongs to the next entry.
                DueShape::ServedWithoutPop => continue,
                DueShape::Nothing => {
                    inner.refused_consults = inner.refused_consults.saturating_add(1);
                    return GateAnswer::Withhold;
                }
                DueShape::Pops { pops, .. } => {
                    // The step's QUOTA spent, which is what the next consult
                    // reads. Frames DELIVERED are counted in `settle` from the
                    // drain's own count, so a shortfall is never reported as an
                    // admission.
                    inner.admitted_this_step = inner.admitted_this_step.saturating_add(pops);
                    return GateAnswer::Exact(due);
                }
            }
        }
    }

    /// Compare what the drain DID against the admission it was granted.
    ///
    /// `delivered` is the drain's own pop count, which is the same quantity
    /// [`StagedReadOutcome::popped`] carries: a malformed frame is popped off
    /// the queue and skipped, and neither side counts it. `surviving_seq` is
    /// the wire sequence of the frame the drain kept.
    ///
    /// The enforcement never fabricates a frame, so this leaves the drain's own
    /// answer alone and only mints.
    #[inline(never)]
    pub(crate) fn settle(&self, granted: DueRead, delivered: u64, surviving_seq: Option<u32>) {
        let step = self.now_step.load(Ordering::Acquire);
        // The monotone admitted witness is written HERE and not at the grant, so
        // the number the report carries is frames the drain really delivered.
        // Its own scope: `mint` below takes the same lock, which is not
        // reentrant.
        if let Ok(mut inner) = self.lock_plan() {
            inner.lifetime_admitted = inner.lifetime_admitted.saturating_add(delivered);
        }
        let expected = granted.pop_count();
        let delivered_u32 = u32::try_from(delivered).unwrap_or(u32::MAX);
        if delivered_u32 != expected {
            self.mint(
                step,
                ReplayReadViolationKind::PoppedShortfall {
                    expected,
                    admitted: delivered_u32,
                },
            );
        }
        // A DECIMATED record's served sequence names the last ACCEPTED sequence
        // and not what this read popped, so its count is compared and its
        // identity is not.
        let identity_unstated = matches!(
            granted.shape(),
            DueShape::Pops {
                identity_unstated: true,
                ..
            }
        );
        if !identity_unstated && surviving_seq != granted.served_seq() {
            self.mint(
                step,
                ReplayReadViolationKind::SequenceMismatch {
                    expected: granted.served_seq(),
                    observed: surviving_seq,
                },
            );
        }
    }

    /// End of step: turn every unspent [`DueShape::Pops`] admission into a
    /// [`ReplayReadViolationKind::NeverArrived`], then retire the step.
    ///
    /// A leftover on either POP LESS shape mints nothing and is discarded: a
    /// [`DueShape::Nothing`] entry declares a read that consumed nothing, and a
    /// [`DueShape::ServedWithoutPop`] entry declares one served elsewhere, so a
    /// drain that never ran owes nothing for either. An edge whose producer
    /// sits in a LATER level legitimately reads next step, and its recorded
    /// quota for this step is then empty.
    #[inline(never)]
    pub(crate) fn sweep(&self, step: u64) {
        let Ok(mut inner) = self.lock_plan() else {
            return;
        };
        if inner.installed && inner.step == step {
            let rim = self.rim as usize;
            let key = &self.key;
            // Destructured so the unspent slice and the violation list are two
            // disjoint borrows of one guard; `&mut *inner` would otherwise hold
            // the whole struct for the walk.
            let PlanInner {
                due,
                cursor,
                violations,
                violations_dropped,
                ..
            } = &mut *inner;
            for unspent in &due[(*cursor).min(due.len())..] {
                if unspent.pop_count() == 0 {
                    continue;
                }
                if violations.len() >= rim {
                    *violations_dropped = violations_dropped.saturating_add(1);
                    continue;
                }
                violations.push(ReplayReadViolation {
                    key: key.clone(),
                    step,
                    kind: ReplayReadViolationKind::NeverArrived {
                        sequence: unspent.served_seq(),
                    },
                });
            }
        }
        // Retire the step. An install is what DECLARES a step's quota, so a
        // step the engine installs nothing for reads nothing rather than
        // spending the previous step's.
        inner.installed = false;
        inner.due.clear();
        inner.cursor = 0;
        inner.admitted_this_step = 0;
    }

    /// Drain the retained divergences, so nothing accumulates across the
    /// engine's step loop.
    pub(crate) fn take_violations(&self) -> Vec<ReplayReadViolation> {
        std::mem::take(&mut self.lock_for_report().violations)
    }

    /// Frames DELIVERED under a grant over the run (a monotone witness).
    #[must_use]
    pub fn admitted(&self) -> u64 {
        self.lock_for_report().lifetime_admitted
    }

    /// Consults the gate refused over the run (a monotone witness). One per
    /// refused consult, never one per queued frame.
    #[must_use]
    pub fn refused_consults(&self) -> u64 {
        self.lock_for_report().refused_consults
    }

    /// Consults the installed plan held no position for: no install for the
    /// step, or past the step's last recorded read. A harness fault, reported
    /// beside the verdict and never part of it.
    #[must_use]
    pub fn unplanned_consults(&self) -> u64 {
        self.lock_for_report().unplanned_consults
    }

    /// Frames admitted at this stage in the step currently installed.
    #[must_use]
    pub fn admitted_this_step(&self) -> u32 {
        self.lock_for_report().admitted_this_step
    }

    /// Consults that found a plan installed for a DIFFERENT step (a harness
    /// fault, never a candidate divergence).
    #[must_use]
    pub fn mismatches(&self) -> u64 {
        self.lock_for_report().mismatches
    }

    /// Divergences the rim dropped. Nonzero means the report is incomplete.
    #[must_use]
    pub fn violations_dropped(&self) -> u64 {
        self.lock_for_report().violations_dropped
    }

    /// Did a panic poison this gate. A poisoned gate withholds every later
    /// admission, so a run that ends with one served nothing from this stage
    /// after the panic and no verdict over it means anything.
    #[must_use]
    pub fn is_unusable(&self) -> bool {
        self.lock_for_report().unusable
    }

    /// The wiring refusal, or the poison one, or none.
    fn wiring_refusal(&self) -> Option<ReadPlanRefusal> {
        match self.blocker {
            Some(GateBlocker::PerSetSyncEdge) => {
                return Some(ReadPlanRefusal::PerSetSyncEdge(self.key.clone()))
            }
            Some(GateBlocker::MultiPublisherEdge) => {
                return Some(ReadPlanRefusal::MultiPublisherEdge(self.key.clone()))
            }
            None => {}
        }
        match self.lock_plan() {
            Ok(inner) if inner.unusable => Some(ReadPlanRefusal::GateUnusable(self.key.clone())),
            Ok(_) => None,
            Err(_) => Some(ReadPlanRefusal::GateUnusable(self.key.clone())),
        }
    }

    /// Retain one divergence, or count it against the rim.
    fn mint(&self, step: u64, kind: ReplayReadViolationKind) {
        let rim = self.rim as usize;
        let Ok(mut inner) = self.lock_plan() else {
            return;
        };
        if inner.violations.len() >= rim {
            inner.violations_dropped = inner.violations_dropped.saturating_add(1);
            return;
        }
        inner.violations.push(ReplayReadViolation {
            key: self.key.clone(),
            step,
            kind,
        });
    }

    /// The uncontended lock, poison-INTOLERANT, the one place this type
    /// departs from its capture twin.
    ///
    /// [`ReadOutcomeStage::lock_inner`] recovers a poisoned lock because a
    /// capture stage is a diagnostic plane whose records are POD and
    /// structurally valid whatever the panicking thread was doing. A gate is
    /// not a diagnostic plane: it DECIDES whether a frame leaves a queue, and a
    /// cursor a panic left mid-advance would admit a read the recording did not
    /// hold. So the poison latches, `wiring_refusal` reports
    /// [`ReadPlanRefusal::GateUnusable`], and every later consult withholds.
    /// The REPORT-side lock, poison-TOLERANT, and the one place a poisoned
    /// guard is read rather than refused.
    ///
    /// An admission decision over a cursor a panic left mid-advance would serve
    /// a frame the recording did not read, so `lock_plan` refuses. A report is
    /// the opposite: the findings retained BEFORE the panic and the poison flag
    /// itself are exactly what the engine needs to refuse the run, and
    /// answering `Vec::new()` / `0` there would hand a clean verdict over a
    /// replay that served nothing after the panic.
    fn lock_for_report(&self) -> std::sync::MutexGuard<'_, PlanInner> {
        match self.inner.lock() {
            Ok(g) => g,
            Err(poisoned) => {
                let mut g = poisoned.into_inner();
                g.unusable = true;
                g
            }
        }
    }

    fn lock_plan(&self) -> Result<std::sync::MutexGuard<'_, PlanInner>, GatePoisoned> {
        match self.inner.lock() {
            Ok(g) => Ok(g),
            Err(poisoned) => {
                let mut g = poisoned.into_inner();
                g.unusable = true;
                Err(GatePoisoned)
            }
        }
    }
}

/// Why a step's quota could not be installed on a stage.
///
/// ONE arm, because a poisoned gate is the only thing an install can refuse on:
/// the plan's LENGTH is the recording's own, and a rim that counts records was
/// never a ceiling for a count of reads (see
/// [`ReadPlanStage::install_step`]). An enum rather than a unit type so a
/// second cause, if one is ever found, arrives without changing every caller's
/// shape.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InstallRefusal {
    /// A panic poisoned the gate; it accepts no install and admits nothing.
    GateUnusable,
}

impl InstallRefusal {
    /// The stable token, one per cause.
    #[must_use]
    pub fn code(self) -> &'static str {
        match self {
            Self::GateUnusable => "gate_unusable",
        }
    }
}

/// The gate's mutex was poisoned by a panic. Carries no data: the ONE answer
/// every caller gives is to withhold, and the reason reaches an operator
/// through [`ReadPlanRefusal::GateUnusable`] in the census.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GatePoisoned;

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
impl ReadOutcomeStage {
    /// The fold anchor, for the one arm that must read STATE rather than an outcome.
    ///
    /// It exists because a mutation proved the drain's
    /// `fold_anchor = FoldAnchor::Broken` reset to be OUTPUT-EQUIVALENT: the fold
    /// site is guarded by `records.last_mut()`, which is `None` after any drain, so
    /// a run cannot span a merge window whatever the anchor says — the emptiness
    /// enforces the invariant, and deleting the reset changed no observable
    /// behaviour in the whole suite. That is not an argument for deleting the line.
    /// The anchor is documented as "`Broken` on an empty stage", the fold CONSULTS
    /// it, and leaving it on `BareRead` across a drain makes that documented meaning
    /// FALSE for every window after the first — an observable-state lie of exactly
    /// the kind Principle #3 forbids, and one that becomes a real fold bug the
    /// moment anything reads the anchor before the vector.
    ///
    /// So the oracle reads the state the line exists to keep true. That is what
    /// makes the reset killable instead of unkillable.
    pub(crate) fn fold_anchor_for_test(&self) -> FoldAnchor {
        self.lock_inner().fold_anchor
    }
}

#[cfg(test)]
pub(crate) fn abi_layout_pins() -> Vec<crate::abi_layout::MeasuredStruct> {
    use crate::abi_layout::{abi_pin_enum, abi_pin_struct};
    vec![
        abi_pin_struct!(ReadOutcomeStage {
            input_idx,
            role,
            capacity,
            armed,
            pending,
            inner
        }),
        abi_pin_struct!(StageInner {
            records,
            fold_anchor,
            last_served_seq,
            dropped,
            dropped_reported,
            dropped_announced,
            next_decade
        }),
        abi_pin_struct!(StagedReadOutcome {
            kind,
            served_seq,
            popped,
            token,
            role,
            // The fold count joins the pinned FIELD SET. The macro
            // destructures without a rest pattern, so a field added here that is
            // not listed fails to compile — which is exactly why the count could
            // not be added silently.
            run_count
        }),
        abi_pin_enum!(ReadOutcomeKind { ReadOutcomeKind::Served, ReadOutcomeKind::Held, ReadOutcomeKind::NoFrame, ReadOutcomeKind::DrainedBatch, ReadOutcomeKind::Decimated, ReadOutcomeKind::Truncated, ReadOutcomeKind::Producer }),
        abi_pin_enum!(ReadSiteRole { ReadSiteRole::Unstamped, ReadSiteRole::Drain, ReadSiteRole::Body, ReadSiteRole::Peek }),
        abi_pin_enum!(ReadStageRole { ReadStageRole::Body, ReadStageRole::Drain }),
        // ABI v23: the ENFORCEMENT side. The subscriber owns a
        // `ReadPlanStage` through a crate-owned `Arc`, which `abi_layout`'s
        // covered closure reaches, so the plan's own chain is pinned on the
        // same rule the capture chain above is: the records a cdylib's code can
        // index are pinned by FIELD SET and by offset, and the stage that holds
        // them by field set alone (it embeds a `std::sync::Mutex`).
        abi_pin_struct!(ReadPlanStage {
            key,
            rim,
            blocker,
            armed,
            now_step,
            inner
        }),
        abi_pin_struct!(PlanInner {
            step,
            installed,
            due,
            cursor,
            admitted_this_step,
            lifetime_admitted,
            refused_consults,
            unplanned_consults,
            violations,
            violations_dropped,
            mismatches,
            unusable
        }),
        abi_pin_struct!(DueRead { shape, served_seq }),
        abi_pin_enum!(DueShape {
            DueShape::Pops { .. },
            DueShape::Nothing,
            DueShape::ServedWithoutPop
        }),
        abi_pin_struct!(ReplayReadViolation { key, step, kind }),
        abi_pin_enum!(ReplayReadViolationKind {
            ReplayReadViolationKind::NeverArrived { .. },
            ReplayReadViolationKind::PoppedShortfall { .. },
            ReplayReadViolationKind::SequenceMismatch { .. }
        }),
        abi_pin_enum!(GateBlocker {
            GateBlocker::PerSetSyncEdge,
            GateBlocker::MultiPublisherEdge
        }),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The shape of the WIDEST stage this derivation ships — a
    /// `MAX_CONSUMER_DEPTH`-deep, ANNOTATED, PER-SET `Sync` TRIGGER body stage,
    /// i.e. the intersection the bound has to survive (the pair factor times the
    /// peek+promote factor).
    ///
    /// Every arm below whose subject is the RIM RULE rather than the SIZING
    /// arms its stage from this shape, so those arms move WITH the derivation
    /// instead of against a number frozen in a test — which is the whole point
    /// of deriving the rim per stage instead of reading one global constant.
    const WIDEST_STAGE: ReadStageSizing = ReadStageSizing {
        depth: crate::graph::topology::MAX_CONSUMER_DEPTH as u32,
        annotated: ProducerAnnotation::Annotated,
        sync_shape: SyncTriggerShape::PerSetSyncTrigger,
        burst: StageBurstBound::resolved_by_wiring(FireBurstBound::Fires(
            crate::graph::topology::MAX_CONSUMER_DEPTH as u32,
        )),
    };

    /// The capacity [`WIDEST_STAGE`] derives — 516 under the shipped constants,
    /// where a single global constant of 320 TRUNCATES this shape.
    const RIM_CAP: usize = derive_stage_capacity(ReadStageRole::Body, WIDEST_STAGE) as usize;

    /// A stage sized from [`WIDEST_STAGE`] — the constructor every RIM-RULE arm
    /// uses. The role is named ONCE, here, and the capacity is derived by the
    /// constructor.
    ///
    /// The shape is NORMALISED to the role, because [`WIDEST_STAGE`] is a
    /// per-set `Sync` TRIGGER shape and that pair is a wiring desync the
    /// derivation refuses in debug (per-set unifies the trigger drain onto the
    /// BODY subscriber, so no `Drain` stage can carry it). A `Drain` rim stage
    /// is therefore an ORDINARY shape and derives 260, not [`RIM_CAP`] — which
    /// is why every rim-rule arm that needs the wide rim asks for `Body`.
    fn rim_stage(input_idx: u16, role: ReadStageRole) -> ReadOutcomeStage {
        let sizing = match role {
            ReadStageRole::Body => WIDEST_STAGE,
            ReadStageRole::Drain => ReadStageSizing {
                sync_shape: SyncTriggerShape::Ordinary,
                ..WIDEST_STAGE
            },
        };
        ReadOutcomeStage::new(input_idx, role, sizing)
    }

    // -----------------------------------------------------------------------
    // `derive_stage_capacity`'s ORACLE VECTOR.
    //
    // Every expected value below is HAND-COMPUTED from the design's worked
    // table, never from a re-run of the function's own arithmetic. The rows are
    // per-FACTOR so a factor that goes missing has nowhere to hide: each arm
    // moves exactly ONE term and states both sides.
    // -----------------------------------------------------------------------

    /// A shape builder for the oracle rows — every field is named at every call
    /// site (there is no `Default`, deliberately), so a row reads as the shape
    /// it describes.
    fn shape(
        depth: u32,
        annotated: ProducerAnnotation,
        sync_shape: SyncTriggerShape,
        burst: FireBurstBound,
    ) -> ReadStageSizing {
        ReadStageSizing {
            depth,
            annotated,
            sync_shape,
            // The oracles mint through the resolver's own constructor: the
            // point of the newtype is that a WIRING site cannot skip the
            // freeze, not that a hand oracle cannot state a bound.
            burst: StageBurstBound::resolved_by_wiring(burst),
        }
    }

    /// A plain latest-value input on a `Period` node costs a HANDFUL of slots,
    /// not the 320 a single global constant would hand every stage in the process.
    ///
    /// Its declared burst is `Fires(1)`, not the node's catch-up count: the
    /// input is STEP-FROZEN, so it stages exactly one record per step however
    /// many times the node fires (`read_stage_burst_bound`
    /// is the site that declares it). The second half of this arm states what
    /// the SAME edge would derive if it were NOT frozen — a `block` input at
    /// the same depth — so the freeze's contribution to the number is visible
    /// rather than implied.
    #[test]
    fn a_plain_period_input_costs_a_handful_of_slots() {
        let frozen = shape(
            1,
            ProducerAnnotation::Plain,
            SyncTriggerShape::Ordinary,
            FireBurstBound::Fires(1),
        );
        // `max(1*1 + 1, 2*1*1 + 1) + 1` = 4, which is also the floor.
        assert_eq!(derive_stage_capacity(ReadStageRole::Body, frozen), 4);
        assert_eq!(
            derive_stage_capacity(ReadStageRole::Body, frozen),
            READ_OUTCOME_STAGE_MIN
        );
        // The SAME edge unfrozen (a `block` input) with a bounded burst of 4:
        // `max(1 + 4, 3) + 1` = 6.
        //
        // This row survives the run-length encoding,
        // and the reason is the limit of what folding can do. RLE folds
        // CONSECUTIVE BYTE-IDENTICAL records. A FROZEN input serves the same
        // snapshot on every catch-up fire, so its burst folds to one slot —
        // which is why `read_stage_burst_bound` declares `Fires(1)` for it and
        // why the frozen row above is 4. An UNFROZEN input may observe a NEW
        // frame on each fire, so its records carry DIFFERENT served sequences,
        // the fold cannot touch them, and the stage must still hold one per
        // fire. The difference between 4 and 6 IS the freeze, and it is a
        // sizing difference, not only a stream difference.
        let unfrozen = shape(
            1,
            ProducerAnnotation::Plain,
            SyncTriggerShape::Ordinary,
            FireBurstBound::Fires(4),
        );
        assert_eq!(derive_stage_capacity(ReadStageRole::Body, unfrozen), 6);
        assert_ne!(
            derive_stage_capacity(ReadStageRole::Body, unfrozen),
            derive_stage_capacity(ReadStageRole::Body, frozen),
            "the freeze changes the SIZE, not merely the stream: an unfrozen \
             input's catch-up records are distinct and cannot fold"
        );
    }

    /// The depth term is the input's OWN declared depth, never
    /// `MAX_CONSUMER_DEPTH`. Substituting the ceiling would make every row
    /// below read 130.
    #[test]
    fn the_depth_term_is_the_declared_depth_not_the_ceiling() {
        let at = |depth| {
            derive_stage_capacity(
                ReadStageRole::Body,
                shape(
                    depth,
                    ProducerAnnotation::Plain,
                    SyncTriggerShape::Ordinary,
                    FireBurstBound::Fires(1),
                ),
            )
        };
        // depth 10: max(10 + 1, 21) + 1 = 22.
        assert_eq!(at(10), 22);
        // depth 64: max(64 + 1, 129) + 1 = 130.
        assert_eq!(at(64), 130);
        assert_eq!(
            at(64),
            at(crate::graph::topology::MAX_CONSUMER_DEPTH as u32)
        );
        // And the two really are different, so the arm cannot pass on a
        // formula that ignores depth altogether.
        assert_ne!(at(10), at(64));
    }

    /// The PAIR factor multiplies the WHOLE bound, headroom included — not the
    /// depth term alone. A `Producer` annotation and the read it names are
    /// admitted as an atomic pair, and the post-cap refill read is itself a
    /// pair on exactly these edges.
    #[test]
    fn the_pair_factor_doubles_the_whole_bound_not_only_the_depth_term() {
        let at = |annotated| {
            derive_stage_capacity(
                ReadStageRole::Body,
                shape(
                    64,
                    annotated,
                    SyncTriggerShape::Ordinary,
                    FireBurstBound::Fires(1),
                ),
            )
        };
        assert_eq!(at(ProducerAnnotation::Plain), 130);
        // 2 * (max(64 + 1, 129) + 1) = 260. Doubling the DEPTH TERM only would
        // give max(128 + 1, 257) + 1 = 258 — which is why the expected value
        // is not simply "twice something".
        assert_eq!(at(ProducerAnnotation::Annotated), 260);
        assert_eq!(
            at(ProducerAnnotation::Annotated),
            2 * at(ProducerAnnotation::Plain)
        );
        assert_ne!(at(ProducerAnnotation::Annotated), 258);
    }

    /// The SITE factor is the peek+promote cost: a per-set `Sync` descent
    /// stages a `Peek` record when it PARKS a frame and a `Drain` record when
    /// it PROMOTES that frame to head, so one consumed frame costs two reads
    /// before the pair factor is applied.
    #[test]
    fn the_site_factor_is_the_peek_promote_cost() {
        let at = |sync_shape| {
            derive_stage_capacity(
                ReadStageRole::Body,
                shape(
                    64,
                    ProducerAnnotation::Plain,
                    sync_shape,
                    FireBurstBound::Fires(64),
                ),
            )
        };
        // The ISOLATED toggle — only `per_set_sync_trigger` moves.
        // site is a predicate, so this is max(64 + 64, 129) + 1 = 130.
        assert_eq!(at(SyncTriggerShape::Ordinary), 130);
        // Site 2 does not produce a closed-form number. A
        // per-set Body stage's reachable population is
        // `(max_sets + 1) * max_sets * site` records per step — the advance
        // budget resets per alignment pass and the Sync burst re-aligns after
        // every fire — which saturates the ceiling. The site factor is still
        // the per-advance cost (peek + promote); it is simply not
        // OBSERVABLE on this shape, because the shape is unbounded. The `Drain`
        // and `Ordinary` rows above are where site stays visible.
        assert_eq!(
            at(SyncTriggerShape::PerSetSyncTrigger),
            READ_OUTCOME_STAGE_MAX
        );
        // The annotated twin lands on the same ceiling: once a shape is
        // unbounded the pair factor cannot move it either.
        assert_eq!(
            derive_stage_capacity(
                ReadStageRole::Body,
                shape(
                    64,
                    ProducerAnnotation::Annotated,
                    SyncTriggerShape::PerSetSyncTrigger,
                    FireBurstBound::Fires(64)
                )
            ),
            READ_OUTCOME_STAGE_MAX
        );
    }

    /// An annotated PAIR folds as a UNIT, with the count on the
    /// READ half — the annotation carries a token in its aux, never a count.
    #[test]
    fn an_annotated_pair_folds_as_a_unit_with_the_count_on_the_read_half() {
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        for _ in 0..4 {
            stage.record_with_producer(
                ReadOutcomeKind::NoFrame,
                None,
                0,
                0xABCD,
                ReadSiteRole::Body,
            );
        }
        let mut drained = Vec::new(); // hot-path-alloc-ok: test collector.
        stage.drain_into(|r| {
            drained.push(r);
            true
        });
        assert_eq!(
            drained
                .iter()
                .map(|r| (r.kind, r.run_count))
                .collect::<Vec<_>>(),
            vec![
                (ReadOutcomeKind::Producer, 1),
                (ReadOutcomeKind::NoFrame, 4),
            ],
            "four identical pairs occupy TWO slots: the annotation once, the read \
             carrying the count. The annotation's own count stays 1 because its \
             aux word is the producer TOKEN — a count there would be token bits"
        );
        assert_eq!(drained[0].token, Some(0xABCD));
    }

    // ------------------------------------------------------------------
    // The FOLD RULE, against hand oracles.
    //
    // A stage folds a maximal run of byte-identical records into one carrying
    // a count. These drive `record` directly and read the staged vector back,
    // so the oracle is the record LIST — not a count the code also computes.
    // ------------------------------------------------------------------

    /// A BARE read never folds into the READ half of
    /// an annotated PAIR.
    ///
    /// The pair's read half carries `token: None`, exactly like a bare read, so
    /// `records_fold` alone cannot tell them apart — a matching bare read folded
    /// straight into it, and an offline reader expanding `Producer x 1 +
    /// read x N` then handed the PAIR's publisher provenance to a read that
    /// never had it. The stage tracks what the last push left foldable
    /// (`FoldAnchor`) instead of trying to read it off the record.
    #[test]
    fn a_bare_read_never_folds_into_an_annotated_pairs_read_half() {
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        // A pair, then a BARE read whose every compared field matches the
        // pair's read half.
        stage.record_with_producer(
            ReadOutcomeKind::Served,
            Some(7),
            1,
            0xAB,
            ReadSiteRole::Body,
        );
        stage.record(ReadOutcomeKind::Served, Some(7), 1, ReadSiteRole::Body);
        let mut drained = Vec::new(); // hot-path-alloc-ok: test collector.
        stage.drain_into(|r| {
            drained.push(r);
            true
        });
        assert_eq!(
            drained.len(),
            3,
            "annotation + the pair's read + the BARE read, unfolded: {drained:?}"
        );
        assert_eq!(drained[0].kind, ReadOutcomeKind::Producer);
        assert_eq!(drained[0].token, Some(0xAB));
        assert_eq!(
            (drained[1].run_count, drained[2].run_count),
            (1, 1),
            "neither may claim to stand for the other: {drained:?}"
        );
    }

    /// The other direction: a PAIR still folds into
    /// a pair. Without this the rule above is satisfied by never folding pairs at
    /// all, which would silently delete a shipped saving.
    #[test]
    fn an_annotated_pair_still_folds_into_an_identical_pair() {
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        for _ in 0..3 {
            stage.record_with_producer(
                ReadOutcomeKind::Served,
                Some(7),
                1,
                0xAB,
                ReadSiteRole::Body,
            );
        }
        let mut drained = Vec::new(); // hot-path-alloc-ok: test collector.
        stage.drain_into(|r| {
            drained.push(r);
            true
        });
        assert_eq!(drained.len(), 2, "one pair, counted: {drained:?}");
        assert_eq!(drained[1].run_count, 3, "the count rides the READ half");
        assert_eq!(
            drained[0].run_count, 1,
            "…and never the annotation, whose aux is the token"
        );
    }

    /// A DROP at the rim BREAKS the run.
    ///
    /// The fold is attempted before the capacity check — deliberately, since a
    /// fold consumes no slot — so without the break, at a full stage the record after a dropped one
    /// folds into the record BEFORE it, and the count then claims an
    /// adjacency the stream did not have (`R, S, R` reads back as `R x 2`). An
    /// offline reader materialises the extra position where the hole is.
    #[test]
    fn a_drop_at_the_rim_breaks_the_run_rather_than_claiming_a_false_adjacency() {
        // Armed at the FLOOR, so the rim is reachable in four records.
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm_at_recorded_capacity(
            AdoptedRim::from_recorded(READ_OUTCOME_STAGE_MIN).expect("the floor is in range"),
        );
        // Four DISTINCT records fill the stage…
        stage.record(ReadOutcomeKind::NoFrame, None, 0, ReadSiteRole::Body);
        stage.record(ReadOutcomeKind::Served, Some(1), 1, ReadSiteRole::Body);
        stage.record(ReadOutcomeKind::Served, Some(2), 1, ReadSiteRole::Body);
        stage.record(ReadOutcomeKind::Served, Some(3), 1, ReadSiteRole::Body);
        // …this one is DROPPED (the stage is full)…
        stage.record(ReadOutcomeKind::Served, Some(4), 1, ReadSiteRole::Body);
        // …and this one matches the last SURVIVOR. Folding consumes no slot, so
        // folding it in would make the count say the two `Served(3)` reads were
        // adjacent — the stream between them held a read that was dropped.
        stage.record(ReadOutcomeKind::Served, Some(3), 1, ReadSiteRole::Body);
        assert_eq!(
            stage.dropped(),
            2,
            "BOTH post-rim records are refused: a run count may not smuggle one \
             past a full stage"
        );
        let mut drained = Vec::new(); // hot-path-alloc-ok: test collector.
        stage.drain_into(|r| {
            drained.push(r);
            true
        });
        assert_eq!(
            drained
                .iter()
                .map(|r| (r.served_seq, r.run_count))
                .collect::<Vec<_>>(),
            vec![
                (STAGE_NO_FRAME, 1),
                (1, 1),
                (2, 1),
                (3, 1),
                // …and the `Truncated` MARKER the drain appends, which is what
                // describes the hole the broken run refused to paper over.
                (STAGE_NO_FRAME, 1),
            ],
            "every survivor stands for ONE occurrence — a run count is a claim \
             about ADJACENCY, and a dropped record between two matching reads is \
             a hole the `Truncated` marker describes: {drained:?}"
        );
    }

    /// A run never crosses a
    /// MERGE WINDOW — and that is a RULE rather than a consequence.
    ///
    /// It would hold anyway because `drain_into` empties the record vector, so
    /// `records.last_mut()` finds nothing to fold into. That is an accident of
    /// one implementation: the fold consults the ANCHOR, and leaving the anchor
    /// on `BareRead` across a drain states that the next window's first record is
    /// adjacent to the previous window's last. `drain_into` resets it, and
    /// the second half of this arm is what says so — identical records either
    /// side of a boundary must NOT fold, which a `records`-only implementation
    /// gets right for the wrong reason and an anchor-consulting one gets right
    /// for the right one.
    #[test]
    fn a_run_never_crosses_a_merge_window() {
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        let mut windows: Vec<Vec<u32>> = Vec::new(); // hot-path-alloc-ok: test collector.
        for _ in 0..2 {
            for _ in 0..2 {
                stage.record(ReadOutcomeKind::NoFrame, None, 0, ReadSiteRole::Body);
            }
            let mut w = Vec::new(); // hot-path-alloc-ok: test collector.
            stage.drain_into(|r| {
                w.push(r.run_count);
                true
            });
            windows.push(w);
        }
        assert_eq!(
            windows,
            vec![vec![2], vec![2]],
            "two windows of two, never one window of four"
        );

        // …and the ANCHOR half: records that are byte-identical ACROSS the
        // boundary still land as two records, one per window. (Above, each
        // window's pair folds internally, so a stale anchor would be invisible —
        // the boundary pair has to be the thing measured.)
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        let mut across: Vec<Vec<u32>> = Vec::new(); // hot-path-alloc-ok: test collector.
        for _ in 0..2 {
            stage.record(ReadOutcomeKind::NoFrame, None, 0, ReadSiteRole::Body);
            let mut w = Vec::new(); // hot-path-alloc-ok: test collector.
            stage.drain_into(|r| {
                w.push(r.run_count);
                true
            });
            across.push(w);
        }
        assert_eq!(
            across,
            vec![vec![1], vec![1]],
            "one identical record per window, each standing for ONE occurrence — \
             a run that spanned the boundary would report `[[], [2]]` or `[[1]]`"
        );

        // …and the STATE half. Both oracles above pass with the drain's
        // `fold_anchor = Broken` reset DELETED,
        // because the fold is guarded by `records.last_mut()` and the vector is
        // empty after a drain, so emptiness alone prevents a cross-window run. The
        // reset is therefore output-equivalent, and what it actually buys is that
        // `FoldAnchor`'s documented meaning ("`Broken` on an empty stage") stays
        // TRUE: leaving it on `BareRead` across a drain asserts that the next
        // window's first record is adjacent to the previous window's last, which is
        // a lie about observable state and a real fold bug for any future reader
        // that consults the anchor before the vector. Read the state, not the
        // outcome — otherwise the line is unkillable and the invariant untested.
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        assert_eq!(
            stage.fold_anchor_for_test(),
            FoldAnchor::Broken,
            "a freshly armed stage is empty, so nothing is adjacent to anything"
        );
        stage.record(ReadOutcomeKind::NoFrame, None, 0, ReadSiteRole::Body);
        assert_eq!(
            stage.fold_anchor_for_test(),
            FoldAnchor::BareRead,
            "a staged bare read is what the NEXT one may fold into"
        );
        stage.drain_into(|_| true);
        assert_eq!(
            stage.fold_anchor_for_test(),
            FoldAnchor::Broken,
            "the drain emptied the stage, so the anchor must say so — the next \
             window's first record is adjacent to nothing"
        );
    }

    /// A change in the SERVED SEQUENCE breaks a run.
    /// The most common real separator, and the one a fold key that dropped
    /// `served_seq` would swallow.
    #[test]
    fn a_served_sequence_change_breaks_the_run() {
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        stage.record(ReadOutcomeKind::Served, Some(1), 1, ReadSiteRole::Body);
        stage.record(ReadOutcomeKind::Served, Some(1), 1, ReadSiteRole::Body);
        stage.record(ReadOutcomeKind::Served, Some(2), 1, ReadSiteRole::Body);
        let mut drained = Vec::new(); // hot-path-alloc-ok: test collector.
        stage.drain_into(|r| {
            drained.push((r.served_seq, r.run_count));
            true
        });
        assert_eq!(drained, vec![(1, 2), (2, 1)], "{drained:?}");
    }

    #[test]
    fn a_run_of_identical_reads_folds_to_one_counted_record() {
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        // Five identical non-consuming reads.
        for _ in 0..5 {
            stage.record(ReadOutcomeKind::NoFrame, None, 0, ReadSiteRole::Body);
        }
        let mut drained = Vec::new(); // hot-path-alloc-ok: test collector.
        stage.drain_into(|r| {
            drained.push(r);
            true
        });
        assert_eq!(
            drained.len(),
            1,
            "five identical reads occupy ONE slot: {drained:?}"
        );
        assert_eq!(drained[0].kind, ReadOutcomeKind::NoFrame);
        assert_eq!(
            drained[0].run_count, 5,
            "…carrying the count of what it stands for"
        );
    }

    /// A record that DIFFERS ends the run — the fold key is the whole record,
    /// so a changed `served_seq` starts a new one rather than being absorbed.
    #[test]
    fn a_differing_read_between_two_runs_keeps_them_separate() {
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        stage.record(ReadOutcomeKind::NoFrame, None, 0, ReadSiteRole::Body);
        stage.record(ReadOutcomeKind::NoFrame, None, 0, ReadSiteRole::Body);
        stage.record(ReadOutcomeKind::Served, Some(7), 1, ReadSiteRole::Body);
        stage.record(ReadOutcomeKind::NoFrame, None, 0, ReadSiteRole::Body);
        let mut drained = Vec::new(); // hot-path-alloc-ok: test collector.
        stage.drain_into(|r| {
            drained.push(r);
            true
        });
        let shape: Vec<(ReadOutcomeKind, u32)> =
            drained.iter().map(|r| (r.kind, r.run_count)).collect();
        assert_eq!(
            shape,
            vec![
                (ReadOutcomeKind::NoFrame, 2),
                (ReadOutcomeKind::Served, 1),
                (ReadOutcomeKind::NoFrame, 1),
            ],
            "the run breaks at the differing record and RESTARTS after it — a fold \
             that keyed on `kind` alone would have merged the two NoFrame runs \
             across the Served read between them"
        );
    }

    /// A `Truncated` marker is NEVER folded: it is minted by `drain_into` and
    /// carries THIS window's drop delta, so two markers describe different
    /// losses even when their fields coincide.
    #[test]
    fn a_truncated_marker_is_never_folded() {
        let sizing = shape(
            1,
            ProducerAnnotation::Plain,
            SyncTriggerShape::Ordinary,
            FireBurstBound::Fires(1),
        );
        let cap = derive_stage_capacity(ReadStageRole::Body, sizing) as usize;
        let stage = std::sync::Arc::new(ReadOutcomeStage::new(0, ReadStageRole::Body, sizing));
        stage.arm();
        // Fill past the rim with DISTINCT records so nothing folds, forcing drops.
        for seq in 0..(cap as u32 + 3) {
            stage.record(ReadOutcomeKind::Served, Some(seq), 1, ReadSiteRole::Body);
        }
        let mut drained = Vec::new(); // hot-path-alloc-ok: test collector.
        stage.drain_into(|r| {
            drained.push(r);
            true
        });
        let markers: Vec<&StagedReadOutcome> = drained
            .iter()
            .filter(|r| r.kind == ReadOutcomeKind::Truncated)
            .collect();
        assert_eq!(markers.len(), 1, "one marker per window: {drained:?}");
        assert_eq!(
            markers[0].run_count, 1,
            "a marker stands for ITSELF — folding two windows' markers would \
             destroy the per-window delta the marker exists to report"
        );
    }

    /// The count SATURATES rather than wrapping: at `u32::MAX` the fold stops
    /// and the next identical read takes a fresh slot. Wrapping to 0 would
    /// decode as ONE occurrence (the `0 == 1` convention) and lose 4 billion.
    #[test]
    fn a_run_at_u32_max_stops_folding_rather_than_wrapping() {
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        stage.record(ReadOutcomeKind::NoFrame, None, 0, ReadSiteRole::Body);
        // Drive the staged record to the ceiling directly — 4 billion real
        // calls is not a test.
        stage.set_last_run_count_for_test(u32::MAX);
        stage.record(ReadOutcomeKind::NoFrame, None, 0, ReadSiteRole::Body);
        let mut drained = Vec::new(); // hot-path-alloc-ok: test collector.
        stage.drain_into(|r| {
            drained.push(r);
            true
        });
        assert_eq!(
            drained.iter().map(|r| r.run_count).collect::<Vec<_>>(),
            vec![u32::MAX, 1],
            "the saturated run is closed and a NEW record opens; a wrapping \
             count would read as a single occurrence"
        );
    }

    /// THE REFILL BOUND, and the arm a depth-only formula fails.
    ///
    /// A per-set `Sync` matcher serves up to `max_sets` complete aligned sets
    /// in ONE step, re-aligning between fires, and each set consumes one frame
    /// from every trigger input. So an edge whose queue REFILLS mid-level is
    /// drained `max_sets` times regardless of its declared depth, and each
    /// consumed frame costs `site` (peek + promote) records — `pair`-doubled on
    /// an annotated edge.
    ///
    /// The hand oracle is therefore `max_sets * site * pair` = `64 * 2 * 2` =
    /// **256 records reachable** on the shipping annotated shape. A naive
    /// formula of `2 * (max(2*16 + 64, 2*2*16 + 1) + 1)` = **194** at
    /// depth 16 would truncate a healthy descent by 62 records. The literals
    /// here are hand-derived from the scheduler's contract, never read back
    /// from the function under test.
    #[test]
    fn a_per_set_trigger_is_sized_for_a_refilling_queue_not_for_its_depth() {
        let cap = |depth| {
            derive_stage_capacity(
                ReadStageRole::Body,
                shape(
                    depth,
                    ProducerAnnotation::Annotated,
                    SyncTriggerShape::PerSetSyncTrigger,
                    FireBurstBound::Fires(64),
                ),
            )
        };
        // The reachable descent, computed from the scheduler's own ceiling.
        const MAX_SETS: u32 = crate::scheduler::DATA_PENDING_CARRY_CLAMP as u32;
        const REACHABLE_RECORDS: u32 = MAX_SETS * 2 /* site */ * 2 /* pair */;
        assert_eq!(REACHABLE_RECORDS, 256, "the hand oracle itself");

        // DEPTH-INDEPENDENT: the refill ceiling is the binding term at every
        // declarable depth, so a shallow per-set edge is sized exactly like a
        // full-depth one. This is the assertion a `frames = depth` variant
        // fails — at depth 1 it derives 8, at depth 16 it derives 194.
        for depth in [1u32, 2, 8, 16, 31, 32, 63, 64] {
            assert_eq!(
                cap(depth),
                READ_OUTCOME_STAGE_MAX,
                "a per-set trigger at depth {depth} must be sized for the refill \
                 ceiling, not for its declared depth"
            );
            assert!(
                cap(depth) >= REACHABLE_RECORDS,
                "depth {depth} allocates {} against {REACHABLE_RECORDS} reachable \
                 records — the stage would truncate a healthy full descent",
                cap(depth)
            );
        }

        // The naive shortfall is exactly `site*depth + burst < site*max_sets`
        // ⇔ `2d + 64 < 128` ⇔ `d < 32`. Pin BOTH sides of that boundary so a
        // half-measure that only raises shallow edges is still caught: depth 31 is
        // under-sized by the naive formula (2*31+64 = 126 < 128) and depth 32 is not.
        // `const` blocks: these are compile-time facts about the shipped
        // constants, so a runtime `assert!` on them is a constant assertion
        // (clippy `assertions_on_constants`). Evaluating them at compile time
        // is strictly stronger — the boundary cannot regress even in a build
        // that never runs this test.
        const _: () = assert!(2 * 31 + 64 < REACHABLE_RECORDS / 2);
        const _: () = assert!(2 * 32 + 64 >= REACHABLE_RECORDS / 2);

        // The UNANNOTATED twin isolates the pair factor from the refill term.
        assert_eq!(
            derive_stage_capacity(
                ReadStageRole::Body,
                shape(
                    16,
                    ProducerAnnotation::Plain,
                    SyncTriggerShape::PerSetSyncTrigger,
                    FireBurstBound::Fires(64),
                )
            ),
            READ_OUTCOME_STAGE_MAX,
        );
    }

    /// The refill bound is scoped to the per-set BODY shape, and the two
    /// controls say why it can be: an ORDINARY body costs ONE record per
    /// consumed frame and its node's `burst` term is the same `max_sets`, so
    /// `depth + burst >= max_sets` already covers a refill there; a `Drain`
    /// stage never carries the per-set shape at all.
    #[test]
    fn the_refill_bound_does_not_leak_into_the_shapes_that_do_not_need_it() {
        // Ordinary body at depth 16: max(16 + 64, 33) + 1 = 81. Unchanged by
        // the refill term, and still >= the 64 frames a refill can consume.
        let ordinary = derive_stage_capacity(
            ReadStageRole::Body,
            shape(
                16,
                ProducerAnnotation::Plain,
                SyncTriggerShape::Ordinary,
                FireBurstBound::Fires(64),
            ),
        );
        assert_eq!(ordinary, 81);
        assert!(ordinary >= crate::scheduler::DATA_PENDING_CARRY_CLAMP as u32);

        // Drain at depth 16: site 1, no refill term. max(16 + 64, 33) + 1 = 81.
        assert_eq!(
            derive_stage_capacity(
                ReadStageRole::Drain,
                shape(
                    16,
                    ProducerAnnotation::Plain,
                    SyncTriggerShape::Ordinary,
                    FireBurstBound::Fires(64),
                )
            ),
            81
        );
    }

    /// The peek/promote factor is a BODY-stage property: the peek and the
    /// promotion both mint on the body subscriber, because per-set `Sync`
    /// unifies its trigger drain onto it. A `Drain` stage is therefore sized at
    /// site 1 — and the discriminator is that the SAME depth on a BODY per-set
    /// stage really does double.
    #[test]
    fn a_drain_stage_is_sized_at_site_one() {
        let drain = derive_stage_capacity(
            ReadStageRole::Drain,
            shape(
                64,
                ProducerAnnotation::Plain,
                SyncTriggerShape::Ordinary,
                FireBurstBound::Fires(64),
            ),
        );
        let body = derive_stage_capacity(
            ReadStageRole::Body,
            shape(
                64,
                ProducerAnnotation::Plain,
                SyncTriggerShape::PerSetSyncTrigger,
                FireBurstBound::Fires(64),
            ),
        );
        assert_eq!(drain, 130, "a drain stage is sized at site 1");
        assert_eq!(
            body, READ_OUTCOME_STAGE_MAX,
            "the per-set shape on a BODY stage really does bite — it \
             bites all the way to the ceiling, because the advance budget resets \
             per alignment pass and the burst re-aligns after every fire"
        );
    }

    /// `(Drain, PerSetSyncTrigger)` is UNREACHABLE from any wiring (per-set
    /// unifies the drain onto the body; the Separate drain exists only on a
    /// NOT-per-set-capable binding; the legacy-`Sync` drain is the `else` arm),
    /// so the derivation treats it as a WIRING DESYNC and says so loudly rather
    /// than absorbing it into a silent normalisation.
    ///
    /// Debug-only by construction: `debug_assert!` is what carries the loudness,
    /// and the release build deliberately keeps the FAIL-SAFE site 1 — a
    /// derivation must never abort the build path it sizes. The release answer
    /// is pinned by the arm below, which is compiled only in a release test
    /// build; in the ordinary (debug) suite this arm is the one that runs.
    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "wiring desync")]
    fn a_drain_stage_that_claims_the_per_set_shape_is_a_loud_wiring_desync() {
        let _ = derive_stage_capacity(
            ReadStageRole::Drain,
            shape(
                64,
                ProducerAnnotation::Plain,
                SyncTriggerShape::PerSetSyncTrigger,
                FireBurstBound::Fires(64),
            ),
        );
    }

    /// The RELEASE half of the arm above: the fail-safe answer is site 1, never
    /// an abort and never a silently doubled drain stage.
    #[cfg(not(debug_assertions))]
    #[test]
    fn a_drain_stage_desync_falls_back_to_site_one_in_release() {
        assert_eq!(
            derive_stage_capacity(
                ReadStageRole::Drain,
                shape(
                    64,
                    ProducerAnnotation::Plain,
                    SyncTriggerShape::PerSetSyncTrigger,
                    FireBurstBound::Fires(64),
                ),
            ),
            130
        );
    }

    /// THE ANTI-REGRESSION ORACLE. The worst shipping shape — an annotated
    /// per-set `Sync` trigger at `MAX_CONSUMER_DEPTH` — needs MORE than a
    /// global constant of 320 can hold. At 320 that shape truncates a healthy
    /// full-depth descent and emits an overflow marker on an edge doing
    /// nothing wrong; that under-sizing is half of why there is no global constant.
    #[test]
    fn the_worst_shipping_shape_exceeds_the_interim_constant() {
        let worst = derive_stage_capacity(ReadStageRole::Body, WIDEST_STAGE);
        // The worst shipping shape is an UNBOUNDED class (see
        // `derive_stage_capacity`), so it arms at the ceiling rather than at a
        // closed-form 516. The claim this arm exists for holds all the more:
        // it needs MORE than a global 320.
        assert_eq!(worst, READ_OUTCOME_STAGE_MAX);
        assert!(
            worst > 320,
            "the worst shape derives {worst}, which must EXCEED a global capacity of 320"
        );
        // 320 is not a token this binary compares against —
        // the per-input manifest table carries every stage's rim,
        // and there is no regime token. What matters here is the
        // FACT that an older recorder armed every stage at 320, which a
        // replay ADOPTS wholesale rather than standing down on;
        // the classification is oracle-tested in
        // `replay_engine::classify_recorded_staging`'s own arms.
        for historical in [64u32, 80, 160, 320] {
            assert!(
                worst > historical,
                "the worst shape must exceed every historical global capacity, or a bag \
                 recorded under one could not be adopted without truncating"
            );
        }
    }

    /// An UNBOUNDED burst lands on the ceiling — never on an overflowing
    /// multiply, and never on a number the state-arm plane supplied.
    #[test]
    fn an_unbounded_burst_lands_on_the_ceiling_not_on_a_panic() {
        for depth in [0u32, 1, 64, u32::MAX] {
            for annotated in [ProducerAnnotation::Plain, ProducerAnnotation::Annotated] {
                let cap = derive_stage_capacity(
                    ReadStageRole::Body,
                    shape(
                        depth,
                        annotated,
                        SyncTriggerShape::Ordinary,
                        FireBurstBound::Unbounded,
                    ),
                );
                assert_eq!(
                    cap, READ_OUTCOME_STAGE_MAX,
                    "depth {depth}, annotated {annotated:?}"
                );
            }
        }
    }

    /// A node may DECLARE a catch-up burst larger than any buffer worth
    /// allocating. The declaration is honoured up to the ceiling and clamped
    /// there — a user-authored number never becomes an allocation size.
    #[test]
    fn a_declared_max_catchup_above_the_ceiling_is_clamped() {
        let cap = derive_stage_capacity(
            ReadStageRole::Body,
            shape(
                1,
                ProducerAnnotation::Plain,
                SyncTriggerShape::Ordinary,
                FireBurstBound::Fires(1_000_000),
            ),
        );
        assert_eq!(cap, READ_OUTCOME_STAGE_MAX);
        // Just BELOW the ceiling the declaration is still honoured exactly, so
        // the clamp is a clamp and not a blanket answer.
        let under = derive_stage_capacity(
            ReadStageRole::Body,
            shape(
                1,
                ProducerAnnotation::Plain,
                SyncTriggerShape::Ordinary,
                FireBurstBound::Fires(4000),
            ),
        );
        assert_eq!(under, 4002);
    }

    /// The derivation takes the MAX of the two arms — the burst arm
    /// (`site*depth + burst`) and the run-length arm (`2*site*depth + 1`) —
    /// rather than branching on whether folding is enabled, so a fold kill
    /// switch cannot re-shape a truncation. Both arms are driven here: one
    /// shape where the burst arm dominates and one where the run-length arm
    /// does.
    #[test]
    fn the_derivation_is_the_max_of_both_arms() {
        // Burst arm dominates: 1*1 + 64 = 65 > 2*1*1 + 1 = 3 ⇒ 65 + 1 = 66.
        let burst_dominates = shape(
            1,
            ProducerAnnotation::Plain,
            SyncTriggerShape::Ordinary,
            FireBurstBound::Fires(64),
        );
        assert_eq!(
            derive_stage_capacity(ReadStageRole::Body, burst_dominates),
            66
        );
        // Run-length arm dominates: 64 + 1 = 65 < 2*64 + 1 = 129 ⇒ 129 + 1 = 130.
        let runs_dominate = shape(
            64,
            ProducerAnnotation::Plain,
            SyncTriggerShape::Ordinary,
            FireBurstBound::Fires(1),
        );
        assert_eq!(
            derive_stage_capacity(ReadStageRole::Body, runs_dominate),
            130
        );
        // A `min` would answer 4 (clamped up from 3 + 1) and 66 respectively.
        assert_ne!(
            derive_stage_capacity(ReadStageRole::Body, burst_dominates),
            4
        );
        assert_ne!(
            derive_stage_capacity(ReadStageRole::Body, runs_dominate),
            66
        );
    }

    /// Every capacity an ANNOTATED edge derives admits an atomic
    /// `record_with_producer` pair: it is EVEN and at least 2. An odd capacity
    /// would make the rim refuse the last pair from a stage with one free slot
    /// — a truncation the derivation never accounted for.
    #[test]
    fn every_stage_capacity_admits_an_atomic_producer_pair() {
        for depth in [0u32, 1, 7, 10, 64, 4095, u32::MAX] {
            for sync_shape in [
                SyncTriggerShape::Ordinary,
                SyncTriggerShape::PerSetSyncTrigger,
            ] {
                for burst in [
                    FireBurstBound::Fires(0),
                    FireBurstBound::Fires(1),
                    FireBurstBound::Fires(64),
                    FireBurstBound::Fires(u32::MAX),
                    FireBurstBound::Unbounded,
                ] {
                    let cap = derive_stage_capacity(
                        ReadStageRole::Body,
                        shape(depth, ProducerAnnotation::Annotated, sync_shape, burst),
                    );
                    assert!(
                        cap >= 2 && cap.is_multiple_of(2),
                        "annotated depth {depth} shape {sync_shape:?} burst {burst:?} \
                         derived {cap}"
                    );
                }
            }
        }
    }

    /// A depth outside the validated range CLAMPS rather than wrapping — a
    /// depth of 0 (which validation forbids, so this is the defence-in-depth
    /// arm) lands on the floor, and a depth at `u32::MAX` lands on the ceiling
    /// instead of overflowing an intermediate product.
    ///
    /// Renamed from `..._saturate_rather_than_wrap`: no arm here
    /// reaches an arithmetic saturation POINT — `u32::MAX` at site 2 is ~8.6e9,
    /// comfortably inside `u64` — so the claim this arm actually carries is the
    /// CLAMP. The saturation claim is carried by
    /// `an_unbounded_burst_lands_on_the_ceiling_not_on_a_panic`, whose
    /// `Unbounded` arm really does feed `u64::MAX` through the multiply.
    #[test]
    fn depth_zero_and_depth_above_the_ceiling_clamp_rather_than_wrap() {
        let at = |depth, burst| {
            derive_stage_capacity(
                ReadStageRole::Body,
                shape(
                    depth,
                    ProducerAnnotation::Annotated,
                    SyncTriggerShape::PerSetSyncTrigger,
                    burst,
                ),
            )
        };
        // This row lands ON the floor rather than BELOW it (the
        // annotated pair doubles 2 to exactly 4), so it does NOT exercise the
        // MIN branch — `the_min_floor_is_a_branch_that_is_actually_taken` does.
        //
        // The depth-0 row reads an ORDINARY shape, because on a
        // PER-SET shape `depth` does not drive the consuming term at all —
        // the refill bound does, so depth 0 derives the full 516 and this row
        // would stop being about the clamp. The per-set twin is asserted just
        // below, where that IS the subject.
        let ordinary_at = |depth, burst| {
            derive_stage_capacity(
                ReadStageRole::Body,
                shape(
                    depth,
                    ProducerAnnotation::Annotated,
                    SyncTriggerShape::Ordinary,
                    burst,
                ),
            )
        };
        assert_eq!(
            ordinary_at(0, FireBurstBound::Fires(0)),
            READ_OUTCOME_STAGE_MIN
        );
        // And the per-set twin: a depth-0 per-set trigger is STILL sized for the
        // refill ceiling, because a queue that refills is drained `max_sets`
        // times whatever its declared depth says. Zero is not a special case.
        //
        // This row reaches the ceiling by the UNBOUNDED-class
        // early return, NOT by the clamp — so it is the wrong instrument for
        // the overflow rows below, which are this test's subject. They read the
        // ORDINARY shape, whose arithmetic really does run through
        // `saturating_mul` and land on the clamp. (With the per-set
        // shape they short-circuit, and dropping the clamp survives
        // this test while two sibling arms still kill it.)
        assert_eq!(at(0, FireBurstBound::Fires(0)), READ_OUTCOME_STAGE_MAX);
        assert_eq!(
            ordinary_at(u32::MAX, FireBurstBound::Fires(1)),
            READ_OUTCOME_STAGE_MAX
        );
        assert_eq!(
            ordinary_at(u32::MAX, FireBurstBound::Fires(u32::MAX)),
            READ_OUTCOME_STAGE_MAX
        );
    }

    /// The MIN clamp is a branch that is ACTUALLY TAKEN.
    ///
    /// Every other arm in this module lands on or above the floor — the
    /// annotated depth-0 row derives exactly 4, which is `READ_OUTCOME_STAGE_MIN`
    /// by arithmetic rather than by clamping — so before this arm the `cap <
    /// MIN` branch was dead: deleting it changed no oracle. The discriminator is
    /// an UNANNOTATED depth-0 shape, whose unclamped value is 2.
    #[test]
    fn the_min_floor_is_a_branch_that_is_actually_taken() {
        let sizing = shape(
            0,
            ProducerAnnotation::Plain,
            SyncTriggerShape::Ordinary,
            FireBurstBound::Fires(0),
        );
        // Unclamped: pair 1 * (max(0 + 0, 0 + 1) + 1) = 2 — strictly BELOW the
        // floor, which is what makes the branch observable at all.
        assert_eq!(derive_stage_capacity(ReadStageRole::Body, sizing), 4);
        assert_eq!(
            derive_stage_capacity(ReadStageRole::Body, sizing),
            READ_OUTCOME_STAGE_MIN
        );
        // A CONST assertion, not a runtime one: both operands are constants, so
        // a lowered floor should fail the BUILD rather than one test run (and
        // clippy refuses the runtime spelling for exactly that reason).
        const {
            assert!(
                READ_OUTCOME_STAGE_MIN > 2,
                "the floor must exceed the unclamped 2, or this arm proves nothing"
            )
        };
    }

    /// The DESIGN's worked table, as one hand-written vector — every shipping
    /// shape in one place, so a formula change that moves any of them shows up
    /// as a single readable diff.
    ///
    /// Three rows are still set by the BURST arm rather than the run-length
    /// one (`Data`-trigger at depth 10, and the two `Data`-trigger rows at
    /// depth 16 and 64); the depth-10 row is the clearest, so it carries the
    /// note. That row's value is
    /// still set by the BURST arm (75, where the run-length arm alone would
    /// give 22): the run-length encoding is what retires it, and
    /// until then the burst term is what keeps that stage from truncating.
    #[test]
    fn the_shipping_shapes_derive_their_worked_table_values() {
        use FireBurstBound::{Fires, Unbounded};
        use ProducerAnnotation::{Annotated, Plain};
        use ReadStageRole::{Body, Drain};
        use SyncTriggerShape::{Ordinary, PerSetSyncTrigger};
        // The ROLE is a column, because it is an argument rather than a
        // field: deriving the two DRAIN rows as Body would merely
        // happen to agree (both are site-1 shapes).
        let rows: &[(&str, ReadStageRole, ReadStageSizing, u32)] = &[
            (
                "Period, plain latest-value input, depth 1, single pub",
                Body,
                shape(1, Plain, Ordinary, Fires(1)),
                4,
            ),
            (
                "Period, plain latest-value input, depth 10",
                Body,
                shape(10, Plain, Ordinary, Fires(1)),
                22,
            ),
            (
                "Period + block input, no declared max_catchup (the unbounded class)",
                Body,
                shape(1, Plain, Ordinary, Unbounded),
                READ_OUTCOME_STAGE_MAX,
            ),
            (
                "Data-trigger consumer, depth 10, single pub",
                Body,
                shape(10, Plain, Ordinary, Fires(64)),
                75,
            ),
            // A `sample(N)`-gated input derives the SAME 75. The
            // backpressure policy decimates which frames are SERVED; it changes
            // neither the queue depth nor the fire bound, and a decimated read
            // still stages a record (`Decimated`). So `sample(N)` is not a
            // factor of this derivation, which is why no field carries it.
            (
                "Data-trigger consumer, depth 10, sample(N)-gated (same shape)",
                Body,
                shape(10, Plain, Ordinary, Fires(64)),
                75,
            ),
            (
                "Data-trigger consumer, depth 64, single pub",
                Body,
                shape(64, Plain, Ordinary, Fires(64)),
                130,
            ),
            (
                "Data-trigger consumer, depth 64, multi-publisher",
                Body,
                shape(64, Annotated, Ordinary, Fires(64)),
                260,
            ),
            (
                "Separate trigger DRAIN stage, depth 64, single pub",
                Drain,
                shape(64, Plain, Ordinary, Fires(64)),
                130,
            ),
            (
                // An UNBOUNDED class — the advance budget
                // resets per alignment pass and the burst re-aligns after every
                // fire, so the reachable population saturates the ceiling.
                "Multi-pub per-set Sync TRIGGER, depth 64 (the worst shape)",
                Body,
                shape(64, Annotated, PerSetSyncTrigger, Fires(64)),
                READ_OUTCOME_STAGE_MAX,
            ),
            (
                "Legacy-Sync DRAIN stage, depth 64, multi-pub",
                Drain,
                shape(64, Annotated, Ordinary, Fires(64)),
                260,
            ),
            // An `External`-provisioned absolute source. External
            // is ALWAYS annotated (`edge_needs_producer_annotation` covers
            // `provisioning == External`), fires at most once per step, and its
            // inputs are step-frozen on a Period/External node — so the row is
            // (Annotated, Ordinary, Fires(1)): (max(11, 21) + 1) * 2 = 44.
            (
                "External absolute source, depth 10, annotated, frozen",
                Body,
                shape(10, Annotated, Ordinary, Fires(1)),
                44,
            ),
        ];
        for (name, role, sizing, expected) in rows {
            assert_eq!(
                derive_stage_capacity(*role, *sizing),
                *expected,
                "worked-table row: {name}"
            );
        }
    }

    /// The enum discriminants ARE the kind-6 wire values — drift-guarded
    /// against the `trace_ring` constants where they exist (unix).
    #[cfg(unix)]
    #[test]
    fn kind_wire_values_match_the_trace_ring_constants() {
        use crate::trace_ring::{
            READ_OUTCOME_DECIMATED, READ_OUTCOME_DRAINED_BATCH, READ_OUTCOME_HELD,
            READ_OUTCOME_NONE, READ_OUTCOME_NO_FRAME, READ_OUTCOME_SERVED,
        };
        assert_eq!(ReadOutcomeKind::Served.wire(), READ_OUTCOME_SERVED);
        assert_eq!(ReadOutcomeKind::Held.wire(), READ_OUTCOME_HELD);
        assert_eq!(ReadOutcomeKind::NoFrame.wire(), READ_OUTCOME_NONE);
        assert_eq!(
            ReadOutcomeKind::DrainedBatch.wire(),
            READ_OUTCOME_DRAINED_BATCH
        );
        assert_eq!(ReadOutcomeKind::Decimated.wire(), READ_OUTCOME_DECIMATED);
        assert_eq!(STAGE_NO_FRAME, READ_OUTCOME_NO_FRAME);
    }

    /// The overflow marker carries the OVERFLOWED
    /// STAGE's role, and the `Drain` arm of `From<ReadStageRole>` had no direct
    /// pin anywhere.
    ///
    /// Every other marker oracle in the tree runs over a `Body`-role stage (the
    /// unified trigger edge's), so `impl From<ReadStageRole> for ReadSiteRole`
    /// could have mapped BOTH stage roles to `Body` and nothing would have
    /// failed. Both arms are driven here, in one body, past the same rim.
    ///
    /// It is DIAGNOSTIC — no consumer steers on a marker's role — but a stage
    /// whose role never reached the wire is a fact the run-exit drop report and
    /// the offline reader disagree about.
    #[test]
    fn an_overflow_marker_carries_the_role_of_the_stage_that_overflowed() {
        for stage_role in [ReadStageRole::Body, ReadStageRole::Drain] {
            let stage = rim_stage(0, stage_role);
            stage.arm();
            for seq in 0..(RIM_CAP as u32 + 1) {
                // The record's OWN role is deliberately the opposite of the
                // stage's, so a marker that copied the last RECORD's role
                // instead of the stage's fails here too.
                stage.record(
                    ReadOutcomeKind::Served,
                    Some(seq),
                    1,
                    match stage_role {
                        ReadStageRole::Body => ReadSiteRole::Drain,
                        ReadStageRole::Drain => ReadSiteRole::Body,
                    },
                );
            }
            let mut drained = Vec::new();
            stage.drain_into(|r| {
                drained.push(r);
                true
            });
            let markers: Vec<_> = drained
                .iter()
                .filter(|r| r.kind == ReadOutcomeKind::Truncated)
                .map(|r| r.role)
                .collect();
            assert_eq!(
                markers,
                vec![ReadSiteRole::from(stage_role)],
                "a {stage_role:?} stage's overflow marker carries {:?}",
                ReadSiteRole::from(stage_role)
            );
        }
        // …and the two stage roles really do map to DIFFERENT wire roles, so
        // the loop above cannot pass against a `From` that collapses them.
        assert_ne!(
            ReadSiteRole::from(ReadStageRole::Body),
            ReadSiteRole::from(ReadStageRole::Drain)
        );
        assert_eq!(
            ReadSiteRole::from(ReadStageRole::Drain),
            ReadSiteRole::Drain
        );
        assert_eq!(ReadSiteRole::from(ReadStageRole::Body), ReadSiteRole::Body);
    }

    #[test]
    fn disarmed_stage_records_nothing() {
        let stage = rim_stage(3, ReadStageRole::Body);
        stage.record(ReadOutcomeKind::Served, Some(7), 1, ReadSiteRole::Body);
        let mut drained = Vec::new();
        stage.drain_into(|r| {
            drained.push(r);
            true
        });
        assert!(drained.is_empty(), "a disarmed stage stages nothing");
    }

    /// The overflow marker's DEBT is retired only
    /// on a CONFIRMED hand-over.
    ///
    /// `dropped_reported` must not advance the instant the marker is OFFERED to
    /// the drain closure, because a consumer can REFUSE it: the production one is
    /// `TraceRingHook::push_read_outcome`, which returns without writing when
    /// the node id is missing from the recording manifest, and with no
    /// in-memory sink installed that refusal is every consumer there is.
    /// Retiring on the offer would therefore lose the marker AND the debt in one
    /// move: the hole stays in the bag and nothing ever describes it
    /// again, because the next window's delta is computed against the count the
    /// offer just advanced.
    ///
    /// Hand oracle: overflow, REFUSE the marker, overflow again, ACCEPT — the
    /// second marker must carry the WHOLE accumulated count, not just the
    /// second window's.
    #[test]
    fn a_refused_overflow_marker_keeps_its_debt_for_the_next_window() {
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();

        // Window 1: overflow by 3, and REFUSE everything the drain offers.
        for seq in 0..(RIM_CAP as u32 + 3) {
            stage.record(ReadOutcomeKind::Served, Some(seq), 1, ReadSiteRole::Body);
        }
        let mut refused = Vec::new();
        stage.drain_into(|r| {
            refused.push(r);
            false // the unmapped-node arm: nothing was written
        });
        assert_eq!(
            refused
                .iter()
                .filter(|r| r.kind == ReadOutcomeKind::Truncated)
                .map(|r| r.popped)
                .collect::<Vec<_>>(),
            vec![3],
            "the marker WAS offered, carrying window 1's delta"
        );

        // Window 2: overflow by 2 more, and ACCEPT this time.
        for seq in 0..(RIM_CAP as u32 + 2) {
            stage.record(ReadOutcomeKind::Served, Some(seq), 1, ReadSiteRole::Body);
        }
        let mut accepted = Vec::new();
        stage.drain_into(|r| {
            accepted.push(r);
            true
        });
        assert_eq!(
            accepted
                .iter()
                .filter(|r| r.kind == ReadOutcomeKind::Truncated)
                .map(|r| r.popped)
                .collect::<Vec<_>>(),
            vec![5],
            "the refused window's 3 drops ride the NEXT marker — 3 + 2, never 2. A debt \
             retired on the OFFER would report 2 here and the first three drops would be \
             described by nothing, ever."
        );

        // Window 3: nothing dropped, and the debt is settled — so NO marker at
        // all. The anti-tautology half: without it, "the count accumulates" is
        // satisfied by a stage that emits a marker on every drain forever.
        stage.record(ReadOutcomeKind::Served, Some(0), 1, ReadSiteRole::Body);
        let mut quiet = Vec::new();
        stage.drain_into(|r| {
            quiet.push(r);
            true
        });
        assert!(
            !quiet.iter().any(|r| r.kind == ReadOutcomeKind::Truncated),
            "a window that dropped nothing, on a stage whose debt is settled, emits NO \
             marker — a healthy edge's stream stays byte-identical to a marker-free one: {quiet:?}"
        );
        // …and the UNCONDITIONAL running total is untouched by any of it
        // (Principle #3 — the counter never resets).
        assert_eq!(stage.dropped(), 5);
    }

    /// The capacity is stated in RECORDS, and a
    /// read on a `multi_publisher_topics` edge costs TWO of them.
    ///
    /// A bound written in READS
    /// (`CAPACITY >= MAX_CONSUMER_DEPTH + HEADROOM` = 65) under-counts
    /// every multi-publisher edge by exactly a factor of two: the deepest burst
    /// one merge window can be asked to hold is `MAX_CONSUMER_DEPTH` reads plus
    /// the post-cap refill, and each of those is a `Producer` annotation PLUS
    /// its read. So the floor is `2 * (64 + 1)` = 130, and at capacity 80
    /// a full annotated burst drops 50 records — deterministically, on the
    /// one edge class the annotation exists for.
    ///
    /// SCOPE: this arm pins the PAIR factor alone. `4 * (MAX_CONSUMER_DEPTH +
    /// HEADROOM)` = 260 would be a GLOBAL floor, not the
    /// shipped bound: each stage derives its own, and the widest
    /// shipping shape derives 516. The whole rule is pinned by
    /// `the_widest_shape_still_holds_a_full_four_records_per_frame_descent` and
    /// driven by `a_full_peek_promote_descent_fits_the_capacity_its_own_shape_derives`. The
    /// precondition below is therefore this arm's own (the pair floor), not the
    /// derived sizing rule.
    ///
    /// This drives the pair floor exactly and asserts BOTH sides of the rim:
    /// the burst FITS (every record survives, no overflow marker), and one pair
    /// past the capacity is refused ATOMICALLY (two records, one marker).
    #[test]
    fn a_full_annotated_burst_fits_and_the_rim_refuses_a_pair_atomically() {
        let floor_pairs = crate::graph::topology::MAX_CONSUMER_DEPTH + READ_OUTCOME_STAGE_HEADROOM;
        assert!(
            RIM_CAP >= 2 * floor_pairs,
            "this arm's precondition is the PAIR floor, 2 * (MAX_CONSUMER_DEPTH + HEADROOM) \
             = {}; the widest shape derives {}. (The capacity is DERIVED per stage \
             — this arm drives the pair floor on a stage sized for the widest shape.)",
            2 * floor_pairs,
            RIM_CAP
        );

        // (a) THE FLOOR FITS. `floor_pairs` annotated reads = 2 * floor_pairs
        // records, which is 130 under the shipped constants.
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        for seq in 0..floor_pairs as u32 {
            stage.record_with_producer(
                ReadOutcomeKind::Served,
                Some(seq),
                1,
                0xABCD_0000 + seq as u64,
                ReadSiteRole::Body,
            );
        }
        let mut drained = Vec::new();
        stage.drain_into(|r| {
            drained.push(r);
            true
        });
        assert_eq!(
            drained.len(),
            2 * floor_pairs,
            "a full annotated burst must fit: {floor_pairs} reads at TWO records each"
        );
        assert!(
            !drained.iter().any(|r| r.kind == ReadOutcomeKind::Truncated),
            "nothing was dropped, so no overflow marker may be emitted"
        );
        // The pairing is what the offline binding rule reads: annotation FIRST,
        // then the read it annotates.
        assert_eq!(drained[0].kind, ReadOutcomeKind::Producer);
        assert_eq!(drained[0].token, Some(0xABCD_0000));
        assert_eq!(drained[1].kind, ReadOutcomeKind::Served);
        assert_eq!(drained[1].token, None);

        // (b) THE RIM refuses a pair BOTH-OR-NEITHER. Fill to capacity in
        // pairs, then offer one more.
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        let capacity_pairs = RIM_CAP / 2;
        for seq in 0..capacity_pairs as u32 + 1 {
            stage.record_with_producer(
                ReadOutcomeKind::Served,
                Some(seq),
                1,
                seq as u64,
                ReadSiteRole::Body,
            );
        }
        let mut drained = Vec::new();
        stage.drain_into(|r| {
            drained.push(r);
            true
        });
        let marker: Vec<_> = drained
            .iter()
            .filter(|r| r.kind == ReadOutcomeKind::Truncated)
            .collect();
        assert_eq!(
            marker.len(),
            1,
            "one overflow marker per window that dropped"
        );
        assert_eq!(
            marker[0].popped, 2,
            "the refused PAIR counts as TWO records — a split would leave a WIDOWED token \
             whose binding rule points at a read that was dropped"
        );
        assert_eq!(
            drained.len(),
            RIM_CAP + 1,
            "every admitted record plus the one marker"
        );
    }

    /// The capacity holds a FULL per-set `Sync` DESCENT on a
    /// `multi_publisher_topics` edge — four records per consumed frame, not two.
    ///
    /// # What breaks without this
    ///
    /// The peek mark made the descent legible by minting a SECOND record per
    /// consumed frame: `sync_peek_next_stamp` pops a frame to learn its stamp
    /// and PARKS it (a `Peek`-role `DrainedBatch`, `popped >= 1`), and the
    /// promote arm that later makes it the head mints a `Drain`-role
    /// `DrainedBatch` with `popped: 0`. On a multi-publisher edge each of those
    /// is a PAIR, so one consumed frame costs FOUR records — and a descent is
    /// ONE align pass, so a whole strict-improvement walk of the queue lands
    /// inside a SINGLE merge window.
    ///
    /// At a capacity of 160 that shape truncates on a HEALTHY backlog:
    /// `4 * (MAX_CONSUMER_DEPTH + HEADROOM)` = 260 records against 160 slots,
    /// dropping 100 at the rim and emitting an overflow marker on an edge doing
    /// nothing wrong. That is the exact class the raise from 64 to 160 existed
    /// to prevent, re-opened by the mark. This arm is what FAILS at 160.
    ///
    /// # Shape
    ///
    /// The floor is driven EXACTLY, in the order and with the `popped` values
    /// the production sites use — so a promotion that re-counted its pop, or an
    /// implementation that dropped the promotion record, is visible here and not
    /// only in the count. Everything is symbolic in `MAX_CONSUMER_DEPTH` and
    /// `READ_OUTCOME_STAGE_HEADROOM`; no literal depth appears.
    #[test]
    fn a_full_peek_promote_descent_fits_the_capacity_its_own_shape_derives() {
        // The bound's own terms: the deepest walk one merge window can hold,
        // plus the post-cap refill slot.
        let floor_frames = crate::graph::topology::MAX_CONSUMER_DEPTH + READ_OUTCOME_STAGE_HEADROOM;
        /// Records ONE consumed frame costs on this edge class: the PAIR factor
        /// (a `Producer` annotation plus its read) times the PEEK+PROMOTE
        /// factor (the peek that parks the frame, the promotion that heads it).
        const RECORDS_PER_FRAME: usize = 4;
        let floor_records = RECORDS_PER_FRAME * floor_frames;

        // The capacity is the one THIS SHAPE derives — `WIDEST_STAGE`
        // IS the annotated per-set `Sync` trigger at `MAX_CONSUMER_DEPTH` — so
        // the arm does not ask whether a global number happens to be large
        // enough; it asks whether the derivation sizes its own worst shape.
        //
        // (An identity `assert_eq!(derive_stage_capacity(..WIDEST..), RIM_CAP)`
        // does not belong here: `RIM_CAP` IS that
        // call, so it would assert `x == x`.)
        //
        // RESIDUAL — the identical-pass set is [264, INFINITY), not
        // [260, 264). This stimulus drives `floor_records` = 260 records and
        // the FINAL assert below requires one further frame of headroom
        // (`RIM_CAP >= floor_records + RECORDS_PER_FRAME` = 264), so rims
        // 260..=263 FAIL here and every rim at or above 264 passes
        // identically. 516 is a rim this stimulus cannot reach:
        // driving a stage to 516 needs the fold-era record economics, so the
        // 516 VALUE is pinned by the pure oracles
        // (`the_worst_shipping_shape_exceeds_the_interim_constant`) and this arm
        // pins the ORDER and `popped` accounting of a real descent.
        assert!(
            RIM_CAP >= floor_records,
            "the derived capacity {} must hold a full peek+promote descent: \
             {RECORDS_PER_FRAME} * (MAX_CONSUMER_DEPTH + HEADROOM) = \
             {RECORDS_PER_FRAME} * {floor_frames} = {floor_records} records",
            RIM_CAP
        );

        // Body, not Drain: `WIDEST_STAGE` is a per-set `Sync` TRIGGER shape and
        // a `Drain` stage derives 260 from it, never 516 — minting this one
        // `Drain` would have sized the arm at a rim its own doc does not name.
        // The RECORDS still carry their own call-site roles below.
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        for seq in 0..floor_frames as u32 {
            // (1) The R-pop: the frame LEAVES the queue here, so `popped` is 1
            // and the role is `Peek` — it names a PARKED frame, which is what
            // `verify_sync` must exclude from its head fold.
            stage.record_with_producer(
                ReadOutcomeKind::DrainedBatch,
                Some(seq),
                1,
                0x0BEE_0000 + seq as u64,
                ReadSiteRole::Peek,
            );
            // (2) The promotion: the SAME frame becomes the head. `popped` is 0
            // — the pop was accounted at (1), and double-counting it would
            // inflate the FIFO pop sum and the `block` credit both drainers
            // conserve.
            stage.record_with_producer(
                ReadOutcomeKind::DrainedBatch,
                Some(seq),
                0,
                0x0BEE_0000 + seq as u64,
                ReadSiteRole::Drain,
            );
        }

        let mut drained = Vec::new();
        stage.drain_into(|r| {
            drained.push(r);
            true
        });

        assert!(
            !drained.iter().any(|r| r.kind == ReadOutcomeKind::Truncated),
            "a full-depth descent on a multi-publisher edge is a HEALTHY backlog and must \
             not reach the rim; at a fixed capacity of 160 this drops {} records and \
             marks the hole",
            floor_records.saturating_sub(160)
        );
        assert_eq!(
            stage.dropped(),
            0,
            "…and the unconditional drop total agrees (a marker-free drain that still \
             counted drops would be a silent hole)"
        );
        assert_eq!(
            drained.len(),
            floor_records,
            "{floor_frames} consumed frames at {RECORDS_PER_FRAME} records each — peek pair \
             plus promotion pair"
        );

        // The ORDER and the `popped` values, on the first frame: annotation
        // first (the offline binding rule reads it that way), then the read it
        // annotates, peek before promotion.
        assert_eq!(
            drained[..4]
                .iter()
                .map(|r| (r.kind, r.role, r.popped, r.token))
                .collect::<Vec<_>>(),
            vec![
                (
                    ReadOutcomeKind::Producer,
                    ReadSiteRole::Peek,
                    0,
                    Some(0x0BEE_0000)
                ),
                (ReadOutcomeKind::DrainedBatch, ReadSiteRole::Peek, 1, None),
                (
                    ReadOutcomeKind::Producer,
                    ReadSiteRole::Drain,
                    0,
                    Some(0x0BEE_0000)
                ),
                (ReadOutcomeKind::DrainedBatch, ReadSiteRole::Drain, 0, None),
            ],
            "peek pair then promotion pair, the pop counted exactly once"
        );

        // THE RIM, one frame past the floor is still inside the capacity — so
        // the fit above is not the buffer being exactly full by luck.
        assert!(
            RIM_CAP >= floor_records + RECORDS_PER_FRAME,
            "the DERIVED capacity {} is a rim ABOVE the floor {floor_records}, not the floor \
             itself",
            RIM_CAP
        );
    }

    #[test]
    fn armed_stage_round_trips_records_in_order_and_clears() {
        let stage = rim_stage(1, ReadStageRole::Drain);
        stage.arm();
        stage.record(
            ReadOutcomeKind::DrainedBatch,
            Some(4),
            3,
            ReadSiteRole::Drain,
        );
        stage.record(ReadOutcomeKind::Held, Some(4), 0, ReadSiteRole::Drain);
        stage.record(ReadOutcomeKind::NoFrame, None, 0, ReadSiteRole::Drain);
        let mut drained = Vec::new();
        stage.drain_into(|r| {
            drained.push(r);
            true
        });
        // Hand oracle, never a self-compare.
        assert_eq!(
            drained,
            vec![
                StagedReadOutcome {
                    kind: ReadOutcomeKind::DrainedBatch,
                    served_seq: 4,
                    popped: 3,
                    token: None,
                    role: ReadSiteRole::Drain,
                    run_count: 1,
                },
                StagedReadOutcome {
                    kind: ReadOutcomeKind::Held,
                    served_seq: 4,
                    popped: 0,
                    token: None,
                    role: ReadSiteRole::Drain,
                    run_count: 1,
                },
                StagedReadOutcome {
                    kind: ReadOutcomeKind::NoFrame,
                    served_seq: STAGE_NO_FRAME,
                    popped: 0,
                    token: None,
                    role: ReadSiteRole::Drain,
                    run_count: 1,
                },
            ]
        );
        // The drain cleared the stage.
        let mut second = Vec::new();
        stage.drain_into(|r| {
            second.push(r);
            true
        });
        assert!(second.is_empty(), "drain clears");
    }

    #[test]
    fn decimated_reports_the_last_served_sequence_not_the_dropped_one() {
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        // Nothing ever served: a decimation reports NO_FRAME.
        stage.record(ReadOutcomeKind::Decimated, Some(99), 1, ReadSiteRole::Body);
        // Serve 5, then decimate: the decimation reports 5 (the last ACCEPT),
        // never its own dropped frame's sequence.
        stage.record(ReadOutcomeKind::Served, Some(5), 1, ReadSiteRole::Body);
        stage.record(ReadOutcomeKind::Decimated, Some(6), 1, ReadSiteRole::Body);
        let mut drained = Vec::new();
        stage.drain_into(|r| {
            drained.push(r);
            true
        });
        assert_eq!(
            drained
                .iter()
                .map(|r| (r.kind, r.served_seq))
                .collect::<Vec<_>>(),
            vec![
                (ReadOutcomeKind::Decimated, STAGE_NO_FRAME),
                (ReadOutcomeKind::Served, 5),
                (ReadOutcomeKind::Decimated, 5),
            ]
        );
    }

    /// `last_served_seq` tracks
    /// what the INPUT SERVED, never what the STAGE RETAINED.
    ///
    /// Both entry points resolve the served-seq slot — and therefore advance
    /// `last_served_seq` — BEFORE the capacity check, so a read whose RECORD
    /// is refused at the rim still moves the last-accept bookmark. That is
    /// deliberate and it is the only correct answer: the frame really was
    /// served to the tick body (the read happened; only the diagnostic record
    /// was dropped), so a later `Decimated` naming it is a true statement
    /// about the value the node continues to observe. Naming the older
    /// last-RECORDED sequence instead would be an affirmatively WRONG claim
    /// about the node's observed value — a staging-buffer fact reported as a
    /// data-plane one, which is the Principle #2 violation, not the fix for
    /// it. The HOLE is described separately and explicitly by the window's
    /// [`ReadOutcomeKind::Truncated`] marker.
    ///
    /// Driven through BOTH rims (`record` and `record_with_producer`), since
    /// each has its own capacity arm and its own `dropped` arithmetic.
    #[test]
    fn a_served_frame_whose_record_was_refused_still_advances_the_last_accept() {
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        // Fill EXACTLY to the rim. The last admitted read serves CAPACITY-1.
        for seq in 0..RIM_CAP as u32 {
            stage.record(ReadOutcomeKind::Served, Some(seq), 1, ReadSiteRole::Body);
        }
        // Two more reads REALLY SERVE their frames to the node; only their
        // records are refused — one at each rim.
        stage.record(ReadOutcomeKind::Served, Some(900), 1, ReadSiteRole::Body);
        stage.record_with_producer(
            ReadOutcomeKind::Served,
            Some(901),
            1,
            0xFEED_FACE,
            ReadSiteRole::Body,
        );

        let mut first = Vec::new();
        stage.drain_into(|r| {
            first.push(r);
            true
        });
        // The window is every admitted record plus ONE overflow marker
        // carrying 3 = 1 (plain) + 2 (the atomically-refused pair).
        let marker: Vec<_> = first
            .iter()
            .filter(|r| r.kind == ReadOutcomeKind::Truncated)
            .collect();
        assert_eq!(marker.len(), 1, "one overflow marker per dropping window");
        assert_eq!(
            marker[0].popped, 3,
            "the plain read dropped ONE record; the annotated pair dropped TWO"
        );

        // The drain freed the buffer, so this decimation is admitted — and it
        // must name 901, the last frame the input really SERVED, not
        // CAPACITY-1, the last one whose record survived.
        stage.record(ReadOutcomeKind::Decimated, Some(902), 1, ReadSiteRole::Body);
        let mut second = Vec::new();
        stage.drain_into(|r| {
            second.push(r);
            true
        });
        assert_eq!(
            second
                .iter()
                .map(|r| (r.kind, r.served_seq))
                .collect::<Vec<_>>(),
            vec![(ReadOutcomeKind::Decimated, 901)],
            "a Decimated names the last ACCEPT — the frame the body still observes — \
             even when that read's own record was refused at the rim"
        );
    }

    #[test]
    fn overflow_drops_and_counts_instead_of_growing() {
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        for seq in 0..(RIM_CAP as u32 + 5) {
            stage.record(ReadOutcomeKind::Served, Some(seq), 1, ReadSiteRole::Body);
        }
        let mut drained = Vec::new();
        stage.drain_into(|r| {
            drained.push(r);
            true
        });
        // The drained window is the retained READ records PLUS
        // ONE overflow MARKER — the hole is described rather than silent, so
        // the length is `CAPACITY + 1` while the capacity bound on the
        // BUFFER is unchanged (the marker is emitted at drain, never staged).
        let reads: Vec<StagedReadOutcome> = drained
            .iter()
            .copied()
            .filter(|r| r.kind != ReadOutcomeKind::Truncated)
            .collect();
        assert_eq!(
            reads.len(),
            RIM_CAP,
            "capacity is a hard bound on the RETAINED READ records"
        );
        assert_eq!(
            drained.last().map(|r| (r.kind, r.popped)),
            Some((ReadOutcomeKind::Truncated, 5)),
            "…and the 5 dropped records are MARKED, last in the window"
        );
        // The retained prefix is the OLDEST records (drop-newest at the rim —
        // the merge order stays chronological).
        assert_eq!(reads[0].served_seq, 0);
        assert_eq!(reads[RIM_CAP - 1].served_seq, RIM_CAP as u64 - 1);
        // After the overflow drained, the stage keeps working — and a window
        // that dropped nothing carries NO marker.
        stage.record(ReadOutcomeKind::Served, Some(1000), 1, ReadSiteRole::Body);
        let mut after = Vec::new();
        stage.drain_into(|r| {
            after.push(r);
            true
        });
        assert_eq!(after.len(), 1);
        assert_eq!(after[0].served_seq, 1000);
        assert_eq!(after[0].kind, ReadOutcomeKind::Served);
    }

    /// The decade helper's boundaries, both sides (0→10, 9→10, 10→100,
    /// 99→100, 100→1000) — the ladder's re-announcement points.
    #[test]
    fn next_decade_above_is_the_next_power_of_ten() {
        assert_eq!(next_decade_above(0), 10);
        assert_eq!(next_decade_above(5), 10);
        assert_eq!(next_decade_above(9), 10);
        assert_eq!(next_decade_above(10), 100);
        assert_eq!(next_decade_above(99), 100);
        assert_eq!(next_decade_above(100), 1000);
    }

    /// `dropped()` is an UNCONDITIONAL running total:
    /// it accumulates across drains and is never reset by a healthy drain.
    #[test]
    fn dropped_is_an_unconditional_running_total_across_drains() {
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        for seq in 0..(RIM_CAP as u32 + 5) {
            stage.record(ReadOutcomeKind::Served, Some(seq), 1, ReadSiteRole::Body);
        }
        stage.drain_into(|_| true);
        assert_eq!(stage.dropped(), 5, "first overflow burst: 5 dropped");
        for seq in 0..(RIM_CAP as u32 + 2) {
            stage.record(ReadOutcomeKind::Served, Some(seq), 1, ReadSiteRole::Body);
        }
        stage.drain_into(|_| true);
        assert_eq!(stage.dropped(), 7, "the total ACCUMULATES across drains");
        stage.record(ReadOutcomeKind::Served, Some(1), 1, ReadSiteRole::Body);
        stage.drain_into(|_| true);
        assert_eq!(stage.dropped(), 7, "a healthy drain never resets the total");
    }

    /// The overflow report is a loud head carrying
    /// `dropped=K` + the capacity, then RE-ANNOUNCES only when the running
    /// total crosses a decade — never once per drain (flood discipline), and
    /// never only-once-per-lifetime (a one-shot latch would hide a growing loss).
    #[test]
    #[tracing_test::traced_test]
    fn overflow_warns_once_then_re_announces_only_at_the_decade() {
        let stage = rim_stage(3, ReadStageRole::Body);
        stage.arm();
        let overflow_by = |k: u32| {
            for seq in 0..(RIM_CAP as u32 + k) {
                stage.record(ReadOutcomeKind::Served, Some(seq), 1, ReadSiteRole::Body);
            }
            stage.drain_into(|_| true);
        };
        // Head: overflow by 5 → ONE warn with dropped=5 + the shared-latch
        // `total_failures=` key (one grep answers "how
        // bad?" across every flood reporter) + the capacity.
        overflow_by(5);
        assert_eq!(stage.dropped(), 5);
        let head = "read-outcome staging overflowed";
        let redecade = "STILL overflowing";
        let capacity_field = format!("capacity={RIM_CAP}");
        logs_assert(|lines: &[&str]| {
            let heads: Vec<&&str> = lines.iter().filter(|l| l.contains(head)).collect();
            match heads.as_slice() {
                [line]
                    if line.contains("WARN")
                        && line.contains("dropped=5")
                        && line.contains("total_failures=5")
                        && line.contains(&capacity_field) =>
                {
                    Ok(())
                }
                other => Err(format!(
                    "expected ONE WARN head carrying dropped=5 + total_failures=5 + \
                     {capacity_field}, got {other:?}"
                )),
            }
        });
        assert!(!logs_contain(redecade), "no decade line yet");
        // Below the decade (total 8): a further overflowing drain is SILENT.
        overflow_by(3);
        assert_eq!(stage.dropped(), 8);
        logs_assert(
            |lines: &[&str]| match lines.iter().filter(|l| l.contains(head)).count() {
                1 => Ok(()),
                n => Err(format!("head must not repeat below the decade (got {n})")),
            },
        );
        assert!(!logs_contain(redecade), "8 < 10: still no decade line");
        // Crossing the decade (total 10): exactly ONE re-announcement, at
        // WARN, carrying the shared-latch `total_failures=` key.
        overflow_by(2);
        assert_eq!(stage.dropped(), 10);
        logs_assert(|lines: &[&str]| {
            let hits: Vec<&&str> = lines.iter().filter(|l| l.contains(redecade)).collect();
            match hits.as_slice() {
                [line] if line.contains("WARN") && line.contains("total_failures=10") => Ok(()),
                other => Err(format!(
                    "expected ONE WARN decade re-announcement with total_failures=10, \
                     got {other:?}"
                )),
            }
        });
        // Below the NEXT decade (total 15 < 100): no further line.
        overflow_by(5);
        assert_eq!(stage.dropped(), 15);
        logs_assert(
            |lines: &[&str]| match lines.iter().filter(|l| l.contains(redecade)).count() {
                1 => Ok(()),
                n => Err(format!("one decade line until 100 (got {n})")),
            },
        );
        // The SECOND crossing: the ladder must keep climbing
        // (a re-announcement that parks `next_decade` at `u64::MAX` after the
        // first crossing passes everything above). Total 15 → 100: a second
        // `STILL overflowing` line carrying total_failures=100.
        overflow_by(85);
        assert_eq!(stage.dropped(), 100);
        logs_assert(|lines: &[&str]| {
            let hits: Vec<&&str> = lines.iter().filter(|l| l.contains(redecade)).collect();
            match hits.as_slice() {
                [_, second] if second.contains("WARN") && second.contains("total_failures=100") => {
                    Ok(())
                }
                other => Err(format!(
                    "expected a SECOND WARN decade line with total_failures=100, got {other:?}"
                )),
            }
        });
        // And below the third decade (total 105 < 1000): still exactly two.
        overflow_by(5);
        assert_eq!(stage.dropped(), 105);
        logs_assert(
            |lines: &[&str]| match lines.iter().filter(|l| l.contains(redecade)).count() {
                2 => Ok(()),
                n => Err(format!("two decade lines until 1000 (got {n})")),
            },
        );
    }

    // =======================================================================
    // The two ANNOTATION kinds + the capacity ↔ clamp coupling.
    // =======================================================================

    /// The marker reports THIS WINDOW's losses, never the lifetime total —
    /// the whole reason `dropped_reported` exists. Driven over THREE windows
    /// so a variant reporting the lifetime total is visible as the sum being re-reported.
    #[test]
    fn marker_records_dropped_count_per_window_not_lifetime() {
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        let overflow_by = |k: u32| {
            for seq in 0..(RIM_CAP as u32 + k) {
                stage.record(ReadOutcomeKind::Served, Some(seq), 1, ReadSiteRole::Body);
            }
            let mut drained = Vec::new(); // hot-path-alloc-ok: test collector.
            stage.drain_into(|r| {
                drained.push(r);
                true
            });
            drained
        };
        // Window 1: 5 dropped. The marker is LAST, after every survivor.
        let w1 = overflow_by(5);
        assert_eq!(w1.len(), RIM_CAP + 1, "survivors + 1 marker");
        assert_eq!(
            w1.last().copied(),
            Some(StagedReadOutcome {
                kind: ReadOutcomeKind::Truncated,
                served_seq: STAGE_NO_FRAME,
                popped: 5,
                token: None,
                role: ReadSiteRole::Body,
                run_count: 1,
            }),
            "the marker is the window's LAST record and carries ITS OWN drop count"
        );
        assert!(
            w1[..w1.len() - 1]
                .iter()
                .all(|r| r.kind == ReadOutcomeKind::Served),
            "every survivor is a real read — the marker is appended, never substituted"
        );
        // Window 2: 3 MORE dropped (lifetime 8). The marker says 3, not 8.
        let w2 = overflow_by(3);
        assert_eq!(
            w2.last().map(|r| (r.kind, r.popped)),
            Some((ReadOutcomeKind::Truncated, 3)),
            "the DELTA (3), never the lifetime total (8)"
        );
        assert_eq!(
            stage.dropped(),
            8,
            "the lifetime total is still unconditional"
        );
        // Window 3: nothing dropped ⇒ NO marker at all (a healthy edge's
        // stream carries no marker).
        stage.record(ReadOutcomeKind::Served, Some(999), 1, ReadSiteRole::Body);
        let mut w3 = Vec::new(); // hot-path-alloc-ok: test collector.
        stage.drain_into(|r| {
            w3.push(r);
            true
        });
        assert_eq!(w3.len(), 1, "one read, no marker");
        assert_eq!(w3[0].kind, ReadOutcomeKind::Served);
        assert_eq!(stage.dropped(), 8, "and the lifetime total did not move");
    }

    /// A producer annotation and the read it binds are admitted BOTH or
    /// NEITHER — a rim split would leave a WIDOWED token pointing at a read
    /// that was dropped, i.e. the FOLLOWING window's first read attributed to
    /// the wrong publisher.
    #[test]
    fn producer_and_read_are_admitted_atomically_at_the_rim() {
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        // The pair's sentinel sequence must live OUTSIDE the
        // fill range. A bare `777` would be outside only by
        // accident at a rim of 516. This shape is unbounded, the
        // fill runs to 4095, and a literal 777 would silently be a legitimate
        // FILL record, so the "neither half is admitted" guard would match the fill
        // and fail on correct code. Derive it from the rim instead.
        const PAIR_SEQ: u32 = RIM_CAP as u32 + 777;
        const _: () = assert!(PAIR_SEQ > RIM_CAP as u32 - 1, "sentinel must not collide");
        // Fill to exactly ONE free slot: a pair cannot fit, a lone read can.
        for seq in 0..(RIM_CAP as u32 - 1) {
            stage.record(ReadOutcomeKind::Served, Some(seq), 1, ReadSiteRole::Body);
        }
        stage.record_with_producer(
            ReadOutcomeKind::Served,
            Some(PAIR_SEQ),
            1,
            0xABCD,
            ReadSiteRole::Body,
        );
        let mut drained = Vec::new(); // hot-path-alloc-ok: test collector.
        stage.drain_into(|r| {
            drained.push(r);
            true
        });
        // CAPACITY-1 reads + the marker; the pair went nowhere, and NO half
        // of it is in the stream.
        assert_eq!(drained.len(), RIM_CAP);
        assert!(
            !drained
                .iter()
                .any(|r| r.kind == ReadOutcomeKind::Producer
                    || r.served_seq == u64::from(PAIR_SEQ)),
            "neither half of the pair is admitted: {drained:?}"
        );
        assert_eq!(
            drained.last().map(|r| (r.kind, r.popped)),
            Some((ReadOutcomeKind::Truncated, 2)),
            "BOTH refused records are counted and MARKED"
        );
        assert_eq!(stage.dropped(), 2);

        // With room for both, the pair lands in order: annotation FIRST,
        // then the read it binds, sharing the served sequence (the redundant
        // join key).
        stage.record_with_producer(
            ReadOutcomeKind::Served,
            Some(888),
            3,
            0x1234_5678_9ABC_DEF0,
            ReadSiteRole::Body,
        );
        let mut after = Vec::new(); // hot-path-alloc-ok: test collector.
        stage.drain_into(|r| {
            after.push(r);
            true
        });
        assert_eq!(
            after,
            vec![
                StagedReadOutcome {
                    kind: ReadOutcomeKind::Producer,
                    served_seq: 888,
                    popped: 0,
                    token: Some(0x1234_5678_9ABC_DEF0),
                    role: ReadSiteRole::Body,
                    run_count: 1,
                },
                StagedReadOutcome {
                    kind: ReadOutcomeKind::Served,
                    served_seq: 888,
                    popped: 3,
                    token: None,
                    role: ReadSiteRole::Body,
                    run_count: 1,
                },
            ]
        );
    }

    /// What a global const-assert (`CAPACITY >= 4 * (MAX_CONSUMER_DEPTH
    /// plus HEADROOM)`) would guarantee, stated where a per-stage bound
    /// can actually be expressed.
    ///
    /// A `const _` cannot express a bound that depends on the graph, so the
    /// coupling it would protect — *a `MAX_CONSUMER_DEPTH` bump must not be silently
    /// absorbed* — is preserved by three things instead: this arm (the widest
    /// shape must still hold a full four-records-per-frame descent at whatever
    /// `MAX_CONSUMER_DEPTH` is), `the_depth_term_is_the_declared_depth_not_the_ceiling`
    /// (the depth the formula reads is the EDGE's, never the ceiling), and
    /// `topology::max_consumer_depth_pinned_at_64` (the ceiling itself does not
    /// move without a test edit).
    ///
    /// It also pins the headroom's own floor: a headroom of 0 would re-open the
    /// dropped-held-head hole the derivation names.
    #[test]
    fn the_widest_shape_still_holds_a_full_four_records_per_frame_descent() {
        let depth = crate::graph::topology::MAX_CONSUMER_DEPTH;
        // Read through locals: the point of this arm is to be a RUNTIME
        // mirror a reader of the suite can see, so it must not collapse into
        // a `const { assert!(..) }` (clippy::assertions_on_constants) — the
        // compile-time half is the `const _: () = assert!(..)` at the top of
        // the module, and duplicating it here would prove nothing new.
        let headroom = READ_OUTCOME_STAGE_HEADROOM;
        let capacity = RIM_CAP;
        assert!(
            headroom >= 1,
            "a zero headroom drops the post-cap refill's record — the held head's, whose \
             re-offer records nothing"
        );
        assert!(
            capacity >= 4 * (depth + headroom),
            "capacity {capacity} must hold a full {depth}-deep burst plus \
             {headroom} headroom at FOUR records per consumed frame — the multi-publisher \
             PAIR cost (the bound is stated in RECORDS, and a \
             `Producer` annotation plus its read is two) times the PEEK+PROMOTE \
             cost (a per-set Sync descent records the peek that PARKS a frame and the \
             promotion that makes it the HEAD, both inside one merge window)"
        );
        // The shipped values, so a silent re-tune shows up as a test edit (the
        // derived capacity is a replay-compat FORMAT parameter).
        // This shape is an UNBOUNDED class, so it arms at the ceiling —
        // still far more than a fixed global of 320 could hold, which is the
        // claim this arm makes.
        assert_eq!(capacity, READ_OUTCOME_STAGE_MAX as usize);
        assert_eq!(headroom, 1);
        assert!(
            capacity > 320,
            "the widest shape must derive MORE than a global constant of 320 — that \
             under-sizing is half of why there is no global constant"
        );
    }

    #[test]
    fn popped_saturates_to_u32() {
        let stage = rim_stage(0, ReadStageRole::Body);
        stage.arm();
        stage.record(
            ReadOutcomeKind::DrainedBatch,
            Some(1),
            u64::from(u32::MAX) + 7,
            ReadSiteRole::Body,
        );
        let mut drained = Vec::new();
        stage.drain_into(|r| {
            drained.push(r);
            true
        });
        assert_eq!(drained[0].popped, u32::MAX);
    }

    // =======================================================================
    // THE ENFORCEMENT SIDE. Hand-built `DueRead`s and a hand queue of
    // sequences: no transport, portable, and every oracle is a number written
    // in the arm rather than one read back off the thing under test.
    // =======================================================================

    /// An ORDINARY body stage's sizing, the shape almost every graph-produced
    /// edge is, and the one whose rim the enforcement arms below are sized by.
    const ORDINARY_STAGE: ReadStageSizing = ReadStageSizing {
        depth: 4,
        annotated: ProducerAnnotation::Plain,
        sync_shape: SyncTriggerShape::Ordinary,
        burst: StageBurstBound::resolved_by_wiring(FireBurstBound::Fires(1)),
    };

    /// The rim an [`ORDINARY_STAGE`] body plan gets. Evaluated in a `const`
    /// through the SAME derivation the capture twin uses, so the arms move with
    /// the derivation instead of against a number frozen in a test.
    const ORDINARY_RIM: u32 = derive_stage_capacity(ReadStageRole::Body, ORDINARY_STAGE);

    fn plan_key(node: &str, input_idx: u16, role: ReadStageRole) -> StageKey {
        StageKey {
            node: node.to_string(),
            input_idx,
            role,
        }
    }

    /// An ARMED plan stage at the ordinary rim, plus the step cell a runtime
    /// would own, so an arm can drive the step the way `step_live` does.
    fn armed_plan(blocker: Option<GateBlocker>) -> (Arc<ReadPlanStage>, Arc<AtomicU64>) {
        let now = Arc::new(AtomicU64::new(0));
        let plan = Arc::new(ReadPlanStage::new(
            plan_key("relay", 0, ReadStageRole::Body),
            ORDINARY_RIM,
            blocker,
            Arc::clone(&now),
        ));
        plan.arm().expect("an ordinary body stage arms");
        (plan, now)
    }

    fn at_step(now: &AtomicU64, step: u64) {
        now.store(step, Ordering::Release);
    }

    fn served(pops: u32, seq: u32) -> DueRead {
        DueRead::pops(pops, Some(seq))
    }

    const NOTHING: DueRead = DueRead::nothing();

    /// The recorded read that SERVED its sequence and popped nothing: today's
    /// hand off, tomorrow's fused chain hop.
    const fn hand_off(seq: u32) -> DueRead {
        DueRead::served_without_pop(Some(seq))
    }

    #[test]
    fn a_zero_quota_step_admits_nothing() {
        let (plan, now) = armed_plan(None);
        at_step(&now, 4);
        plan.install_step(4, &[NOTHING]).unwrap();
        assert_eq!(plan.admit(), GateAnswer::Withhold);
        plan.sweep(4);

        at_step(&now, 5);
        plan.install_step(5, &[served(1, 7)]).unwrap();
        assert_eq!(plan.admit(), GateAnswer::Exact(served(1, 7)));
        // The two monotone witnesses, both read after the fact: one consult
        // refused at step 4, and one frame admitted at step 5 once the drain
        // SETTLED it, which is where the admitted witness is written.
        assert_eq!(plan.refused_consults(), 1);
        assert_eq!(plan.admitted(), 0);
        plan.settle(served(1, 7), 1, Some(7));
        assert_eq!(plan.admitted(), 1);

        // An EMPTY step plan is the same declaration, and it is the ORDINARY
        // shape: a drain that found its queue empty writes no record, so most
        // steps of a gated stage name no read at all. It refuses the consult
        // WITHOUT counting it as one the plan held no position for, which is
        // what keeps that harness-fault witness meaningful.
        plan.sweep(5);
        at_step(&now, 6);
        plan.install_step(6, &[]).unwrap();
        assert_eq!(plan.admit(), GateAnswer::Withhold);
        assert_eq!(plan.refused_consults(), 2);
        assert_eq!(plan.unplanned_consults(), 0);
    }

    #[test]
    fn admit_exactly_n_spends_the_quota_once() {
        let (plan, now) = armed_plan(None);
        at_step(&now, 1);
        plan.install_step(1, &[served(3, 9)]).unwrap();
        assert_eq!(plan.admit(), GateAnswer::Exact(served(3, 9)));
        // The quota is SPENT: a second consult in the same step gets nothing,
        // which is what a gate that refilled per call would fail.
        assert_eq!(plan.admit(), GateAnswer::Withhold);
        assert_eq!(plan.admit(), GateAnswer::Withhold);
        // The QUOTA is spent at the grant and the admitted witness is written at
        // the SETTLE, so a drain that was granted three and delivered two moves
        // the two numbers apart.
        assert_eq!(plan.admitted_this_step(), 3);
        plan.settle(served(3, 9), 2, Some(9));
        assert_eq!(plan.admitted(), 2);
        // The two consults past a SPENT non-empty plan are the plan and the
        // drain disagreeing about how many reads the step holds, counted apart
        // from a recorded none and from an empty step plan.
        assert_eq!(plan.refused_consults(), 2);
        assert_eq!(plan.unplanned_consults(), 2);
    }

    /// **An EMPTY step plan withholds, admits nothing and mints no violation.**
    ///
    /// The half of the readless-stage question that is INTENDED, pinned beside
    /// the half that is not (`replay_inject`'s
    /// `a_two_site_stream_over_one_wired_stage_is_refused`): a stage the
    /// recording holds no read for at a step is armed and declares that step
    /// empty, so the gate withholds and NOTHING is charged to the candidate. A
    /// drain that found its queue empty writes no record at all, so most steps
    /// of a gated stage name no read and this is the ordinary path.
    ///
    /// The frame the withhold leaves QUEUED is the subscriber's half
    /// (`drain_with_accounting_impl` answers `DrainOutcome::withheld()`), which
    /// needs a wired transport and is unreachable from this crate: the claim
    /// here is scoped to what the gate itself does.
    #[test]
    fn an_empty_step_plan_withholds_admits_nothing_and_mints_no_violation() {
        let (plan, now) = armed_plan(None);
        at_step(&now, 3);
        plan.install_step(3, &[]).unwrap();
        assert_eq!(plan.admit(), GateAnswer::Withhold);
        assert_eq!(plan.admitted_this_step(), 0, "an empty plan grants nothing");
        plan.sweep(3);
        assert!(
            plan.take_violations().is_empty(),
            "a step the recording named no read for charges the candidate nothing"
        );
        assert_eq!(plan.admitted(), 0);
        // The consult IS counted as refused (the refill classifier reads this
        // witness to tell an enforced empty from an empty queue) and NOT as one
        // the plan held no position for.
        assert_eq!(plan.refused_consults(), 1);
        assert_eq!(plan.unplanned_consults(), 0);
    }

    /// A recorded none admits nothing and mints nothing.
    ///
    /// The arm reads the GATE only. Its earlier name claimed "even with frames
    /// present" over a local `&[u32]` the gate never saw, which asserted
    /// nothing: a real queue holding frames across a withheld consult needs a
    /// wired subscriber, and no arm in this crate pins that half today.
    #[test]
    fn a_recorded_none_admits_nothing_and_mints_nothing() {
        let (plan, now) = armed_plan(None);
        at_step(&now, 12);
        plan.install_step(12, &[NOTHING]).unwrap();
        assert_eq!(plan.admit(), GateAnswer::Withhold);
        // Nothing was minted: a held-back frame is never a violation.
        plan.sweep(12);
        assert!(plan.take_violations().is_empty());
    }

    #[test]
    fn a_sequence_mismatch_names_stage_step_and_both_sequences() {
        let (plan, now) = armed_plan(None);
        at_step(&now, 343);
        plan.install_step(343, &[served(1, 43)]).unwrap();
        let GateAnswer::Exact(granted) = plan.admit() else {
            panic!("an installed nonzero quota admits");
        };
        // The drain popped its one frame and the head carried 42, not 43.
        plan.settle(granted, 1, Some(42));
        let found = plan.take_violations();
        assert_eq!(
            found,
            vec![ReplayReadViolation {
                key: plan_key("relay", 0, ReadStageRole::Body),
                step: 343,
                kind: ReplayReadViolationKind::SequenceMismatch {
                    expected: Some(43),
                    observed: Some(42),
                },
            }]
        );
        // And a MATCHING settle mints nothing, so the arm cannot pass on a
        // gate that mints unconditionally.
        at_step(&now, 344);
        plan.install_step(344, &[served(1, 44)]).unwrap();
        let GateAnswer::Exact(granted) = plan.admit() else {
            panic!("an installed nonzero quota admits");
        };
        plan.settle(granted, 1, Some(44));
        assert!(plan.take_violations().is_empty());
    }

    #[test]
    fn a_short_drain_is_a_popped_shortfall() {
        let (plan, now) = armed_plan(None);
        at_step(&now, 8);
        plan.install_step(8, &[served(3, 9)]).unwrap();
        let GateAnswer::Exact(granted) = plan.admit() else {
            panic!("an installed nonzero quota admits");
        };
        // Two frames delivered against a quota of three, and the survivor is
        // the recorded one, so the ONE finding is the count.
        plan.settle(granted, 2, Some(9));
        assert_eq!(
            plan.take_violations(),
            vec![ReplayReadViolation {
                key: plan_key("relay", 0, ReadStageRole::Body),
                step: 8,
                kind: ReplayReadViolationKind::PoppedShortfall {
                    expected: 3,
                    admitted: 2,
                },
            }]
        );
    }

    #[test]
    fn an_unspent_quota_at_step_end_is_never_arrived() {
        let (plan, now) = armed_plan(None);
        at_step(&now, 21);
        // Three recorded reads: a none, a served one, and a second served one.
        // The step's drains consult ONCE, so the last two go unspent.
        plan.install_step(21, &[NOTHING, served(1, 5), served(2, 6)])
            .unwrap();
        assert_eq!(plan.admit(), GateAnswer::Withhold);
        plan.sweep(21);
        assert_eq!(
            plan.take_violations(),
            vec![
                ReplayReadViolation {
                    key: plan_key("relay", 0, ReadStageRole::Body),
                    step: 21,
                    kind: ReplayReadViolationKind::NeverArrived { sequence: Some(5) },
                },
                ReplayReadViolation {
                    key: plan_key("relay", 0, ReadStageRole::Body),
                    step: 21,
                    kind: ReplayReadViolationKind::NeverArrived { sequence: Some(6) },
                },
            ]
        );
    }

    #[test]
    fn an_unspent_zero_pops_leftover_mints_nothing() {
        let (plan, now) = armed_plan(None);
        at_step(&now, 22);
        plan.install_step(22, &[NOTHING, NOTHING]).unwrap();
        // No drain consulted at all this step.
        plan.sweep(22);
        assert!(plan.take_violations().is_empty());
        // The sweep RETIRED the step, so the next step with no install reads
        // nothing rather than spending step 22's quota.
        at_step(&now, 23);
        assert_eq!(plan.admit(), GateAnswer::Withhold);
    }

    #[test]
    fn a_burst_run_expands_to_run_count_admissions() {
        let (plan, now) = armed_plan(None);
        at_step(&now, 30);
        // A recorded record with `run_count = 3, popped = 1` is expanded engine
        // side into three identical admissions; this arm pins that the gate
        // then admits THREE single-pop reads and not one.
        let expanded = [served(1, 11), served(1, 11), served(1, 11)];
        plan.install_step(30, &expanded).unwrap();
        for _ in 0..3 {
            assert_eq!(plan.admit(), GateAnswer::Exact(served(1, 11)));
            plan.settle(served(1, 11), 1, Some(11));
        }
        assert_eq!(plan.admit(), GateAnswer::Withhold);
        assert_eq!(plan.admitted(), 3);
    }

    #[test]
    fn the_cursor_never_scans_past_a_zero_pops_entry() {
        let (plan, now) = armed_plan(None);
        at_step(&now, 2);
        plan.install_step(2, &[NOTHING, served(1, 7)]).unwrap();
        // The FIRST consult withholds and the SECOND admits. A gate that
        // skipped forward to the next nonzero entry would admit at the first,
        // which is the boundary drain spending a body read's quota.
        assert_eq!(plan.admit(), GateAnswer::Withhold);
        assert_eq!(plan.admit(), GateAnswer::Exact(served(1, 7)));
    }

    #[test]
    fn a_stale_plan_admits_nothing_and_counts_a_mismatch() {
        let (plan, now) = armed_plan(None);
        at_step(&now, 4);
        plan.install_step(4, &[served(1, 7)]).unwrap();
        // The runtime moved to step 5 with no install for it.
        at_step(&now, 5);
        assert_eq!(plan.admit(), GateAnswer::Withhold);
        assert_eq!(plan.mismatches(), 1);
        // A harness fault is NOT a candidate divergence, so nothing is minted.
        assert!(plan.take_violations().is_empty());
        // The refused witness DOES move: the refill that follows this withhold
        // finds its queue empty, and the classifier reads this witness to tell
        // an enforced empty from an empty queue. Left at zero, the stale-plan
        // withhold was reported as the recording holding a frame the replay's
        // input stream did not, which names the candidate for a harness fault.
        // The mismatch counter is what keeps the two apart.
        assert_eq!(plan.refused_consults(), 1);
    }

    #[test]
    fn a_decimated_read_enforces_the_count_and_not_the_identity() {
        let (plan, now) = armed_plan(None);
        let decimated = DueRead::decimated(2, Some(70));
        at_step(&now, 9);
        plan.install_step(9, &[decimated]).unwrap();
        let GateAnswer::Exact(granted) = plan.admit() else {
            panic!("an installed nonzero quota admits");
        };
        // The right COUNT with a different surviving sequence: no finding, because
        // a decimated record's served seq names the last ACCEPTED sequence and
        // not what the read popped.
        plan.settle(granted, 2, Some(99));
        assert!(plan.take_violations().is_empty());

        // A different COUNT still mints: the count is the part of a decimated
        // record the gate does compare, pinned here rather than assumed.
        at_step(&now, 10);
        plan.install_step(10, &[decimated]).unwrap();
        let GateAnswer::Exact(granted) = plan.admit() else {
            panic!("an installed nonzero quota admits");
        };
        plan.settle(granted, 1, Some(99));
        assert_eq!(
            plan.take_violations(),
            vec![ReplayReadViolation {
                key: plan_key("relay", 0, ReadStageRole::Body),
                step: 10,
                kind: ReplayReadViolationKind::PoppedShortfall {
                    expected: 2,
                    admitted: 1,
                },
            }]
        );
    }

    #[test]
    fn an_unarmed_stage_is_ungated_and_an_armed_omitted_stage_is_zero_quota() {
        // HALF ONE: never armed. The live path's whole proof is this answer,
        // `is_armed` is false, so no drain ever reaches a consult at all.
        let now = Arc::new(AtomicU64::new(0));
        let unarmed = ReadPlanStage::new(
            plan_key("relay", 0, ReadStageRole::Body),
            ORDINARY_RIM,
            None,
            Arc::clone(&now),
        );
        assert!(!unarmed.is_armed());
        // The answer itself, not just the flag: an unarmed stage answers
        // UNGATED, which is what tells a drain to take today's path rather than
        // to pop nothing.
        assert_eq!(unarmed.admit(), GateAnswer::Ungated);
        assert_eq!(unarmed.admitted(), 0);
        assert_eq!(unarmed.refused_consults(), 0);
        assert!(unarmed.take_violations().is_empty());

        // HALF TWO: armed, and the step's install OMITS it. The quota is zero
        // for that step, which is a refused pop and not an ungated one.
        let (plan, now) = armed_plan(None);
        at_step(&now, 6);
        assert_eq!(plan.admit(), GateAnswer::Withhold);
        assert_eq!(plan.refused_consults(), 1);
        // And it is counted as UNPLANNED: an armed stage the step's install
        // omitted holds no position for this consult, which is a different fact
        // from a recorded none.
        assert_eq!(plan.unplanned_consults(), 1);
        assert!(plan.is_armed());
    }

    #[test]
    fn the_live_path_stays_the_live_path_while_nothing_is_armed() {
        // The NEGATIVE CONTROL, at the level a pure unit test can reach: a
        // plan wired and never armed answers `Ungated` at the accessor the
        // drain sites branch on, so the drain takes today's code path. The
        // integration half of this control is the graph-level arm and the
        // brief's named live binaries at the tip.
        let now = Arc::new(AtomicU64::new(0));
        let plan = ReadPlanStage::new(
            plan_key("relay", 0, ReadStageRole::Body),
            ORDINARY_RIM,
            None,
            now,
        );
        // Installing a quota on an UNARMED stage cannot make it gate: the
        // subscriber's accessor filters on `is_armed`, so the install is inert.
        plan.install_step(0, &[served(1, 1)]).unwrap();
        assert!(!plan.is_armed());

        // And a disarm retires the quota with the gate, so a re-arm cannot
        // spend a retired step's reads.
        plan.arm().unwrap();
        plan.install_step(0, &[served(1, 1)]).unwrap();
        plan.disarm();
        assert!(!plan.is_armed());
        plan.arm().unwrap();
        assert_eq!(plan.admit(), GateAnswer::Withhold);
    }

    #[test]
    fn the_census_and_arm_refuse_and_name_the_stage() {
        let now = Arc::new(AtomicU64::new(0));
        let key = plan_key("relay", 2, ReadStageRole::Body);
        for (blocker, want) in [
            (
                GateBlocker::PerSetSyncEdge,
                ReadPlanRefusal::PerSetSyncEdge(key.clone()),
            ),
            (
                GateBlocker::MultiPublisherEdge,
                ReadPlanRefusal::MultiPublisherEdge(key.clone()),
            ),
        ] {
            let plan =
                ReadPlanStage::new(key.clone(), ORDINARY_RIM, Some(blocker), Arc::clone(&now));
            let census = plan.capability();
            assert!(!census.enforceable);
            assert_eq!(census.reason.as_ref(), Some(&want));
            assert_eq!(plan.arm(), Err(want));
            assert!(!plan.is_armed());
            // Every refusal NAMES the stage, which is what an operator reads.
            assert_eq!(census.reason.unwrap().key().label(), "relay[2]/body");
        }

        // A gateable stage's census row says so, and its arm succeeds.
        let ok = ReadPlanStage::new(key.clone(), ORDINARY_RIM, None, Arc::clone(&now));
        assert_eq!(
            ok.capability(),
            ReadEdgeCapability {
                key: key.clone(),
                enforceable: true,
                reason: None,
            }
        );
        assert!(ok.arm().is_ok());
    }

    /// THE BOUND ON A STEP PLAN IS THE RECORDING, NOT THE RIM.
    ///
    /// The rim counts this stage's RECORDS in one merge window; a step plan
    /// carries one entry per recorded READ, and the fold stores a run of
    /// identical reads as ONE record. So a plan longer than the rim is what a
    /// catch-up burst looks like, not evidence of a foreign log, and it installs
    /// and is SERVED IN FULL. Refusing it refused replays the recording
    /// describes exactly.
    #[test]
    fn a_step_plan_longer_than_the_derived_rim_installs_and_is_served_in_full() {
        let (plan, now) = armed_plan(None);
        assert_eq!(plan.rim(), ORDINARY_RIM);
        at_step(&now, 0);
        // The rim oracle is the DERIVATION, evaluated in a `const` above, so
        // this arm does not restate the formula's number. One entry PAST it is
        // the shape the old bound refused.
        let over: Vec<DueRead> = (0..=ORDINARY_RIM).map(|i| served(1, i)).collect();
        assert!(
            over.len() > ORDINARY_RIM as usize,
            "PRECONDITION: the plan must be longer than the rim"
        );
        assert_eq!(plan.install_step(0, &over), Ok(()));
        // Every entry, the ones past the rim included, is answered in recorded
        // order: a bound that truncated the tail would withhold at the rim.
        for i in 0..=ORDINARY_RIM {
            assert_eq!(plan.admit(), GateAnswer::Exact(served(1, i)));
            plan.settle(served(1, i), 1, Some(i));
        }
        assert_eq!(plan.admit(), GateAnswer::Withhold);
        assert_eq!(plan.admitted(), u64::from(ORDINARY_RIM) + 1);
    }

    /// A plan sized by a WIDER recording installs on a stage at the FLOOR rim.
    ///
    /// The mismatch direction the enforcement has to survive: the sizing reads
    /// candidate facts (a node's burst bound and its per-set shape), so a
    /// recording whose stage derived the CEILING can be replayed on a build that
    /// derives the FLOOR for the same edge. The recorded reads are then many
    /// times this stage's rim, and refusing that install refused the replay
    /// outright. Written at the floor and the ceiling so the arm states the
    /// widest gap the clamp allows rather than a number picked by hand.
    #[test]
    fn a_plan_sized_by_a_wider_recording_installs_on_a_floor_rim_stage() {
        let now = Arc::new(AtomicU64::new(0));
        let plan = ReadPlanStage::new(
            plan_key("relay", 0, ReadStageRole::Body),
            READ_OUTCOME_STAGE_MIN,
            None,
            Arc::clone(&now),
        );
        plan.arm().expect("an ordinary body stage arms");
        let wide: Vec<DueRead> = (0..=READ_OUTCOME_STAGE_MAX).map(|_| NOTHING).collect();
        assert!(
            wide.len() > READ_OUTCOME_STAGE_MIN as usize,
            "PRECONDITION: the recording's step must outnumber this stage's rim"
        );
        assert_eq!(plan.install_step(0, &wide), Ok(()));
        // A recorded NONE at every position: the gate withholds at each, which is
        // the enforcement working, and no position was lost to a bound.
        assert_eq!(plan.admit(), GateAnswer::Withhold);
        assert_eq!(plan.admitted(), 0);
    }

    /// The rim KEEPS its one job over a plan longer than itself.
    ///
    /// The arm above proves the plan is not bounded; this one proves that did
    /// not take the divergence list's bound with it. Unreachable before the
    /// plan's bound was lifted, which is why it is written beside it.
    #[test]
    fn the_divergence_list_is_still_bounded_by_the_rim_over_a_longer_plan() {
        let (plan, now) = armed_plan(None);
        let rim = ORDINARY_RIM as usize;
        at_step(&now, 1);
        // Twice the rim, every entry UNSPENT: the sweep mints one finding per
        // entry, retains the rim's worth and counts the rest.
        let due: Vec<DueRead> = (0..rim * 2).map(|i| served(1, i as u32)).collect();
        plan.install_step(1, &due).unwrap();
        plan.sweep(1);
        assert_eq!(plan.take_violations().len(), rim);
        assert_eq!(plan.violations_dropped() as usize, due.len() - rim);
    }

    #[test]
    fn the_violation_list_is_bounded_by_the_rim_and_counts_the_overflow() {
        let (plan, now) = armed_plan(None);
        let rim = ORDINARY_RIM as usize;
        at_step(&now, 1);
        // Every entry unspent, one more than the rim can retain: the rim's
        // worth is kept and the remainder is COUNTED, so the engine can refuse
        // rather than report a truncated verdict.
        let due: Vec<DueRead> = (0..rim).map(|i| served(1, i as u32)).collect();
        plan.install_step(1, &due).unwrap();
        plan.sweep(1);
        at_step(&now, 2);
        plan.install_step(2, &[served(1, 999)]).unwrap();
        plan.sweep(2);
        assert_eq!(plan.violations_dropped(), 1);
        assert_eq!(plan.take_violations().len(), rim);
        assert_eq!(plan.violations_dropped(), 1);
    }

    #[test]
    fn a_pop_of_nothing_normalises_by_what_it_served() {
        // The type's whole invariant: `DueShape::Pops { pops: 0 }` is
        // unconstructible, and which POP LESS shape a zero-pop record becomes is
        // decided by whether it SERVED a sequence.
        assert_eq!(DueRead::pops(0, None).shape(), DueShape::Nothing);
        assert_eq!(
            DueRead::pops(0, Some(9)).shape(),
            DueShape::ServedWithoutPop
        );
        assert_eq!(DueRead::pops(0, Some(9)).served_seq(), Some(9));
        assert_eq!(DueRead::pops(0, None).pop_count(), 0);
        assert_eq!(DueRead::pops(0, Some(9)).pop_count(), 0);
        assert_eq!(
            DueRead::pops(2, Some(9)).shape(),
            DueShape::Pops {
                pops: 2,
                identity_unstated: false,
            }
        );
        assert_eq!(DueRead::decimated(2, Some(9)).pop_count(), 2);
    }

    #[test]
    fn a_served_without_pop_record_is_transparent_to_the_gate() {
        let (plan, now) = armed_plan(None);
        at_step(&now, 7);
        // The recording's step: a hand off served sequence 50 without a
        // dequeue, then a drain popped one frame carrying 51. The hand off
        // occupies no position in the pop stream, so the step's ONE drain
        // consult must be answered by the SECOND entry.
        //
        // The old two-meaning shape withheld on the hand off instead, then the
        // sweep discarded the leftover and the settle compared 0 to 0, so the
        // divergence reached a verdict as a byte difference charged to the
        // candidate.
        plan.install_step(7, &[hand_off(50), served(1, 51)])
            .unwrap();
        assert_eq!(plan.admit(), GateAnswer::Exact(served(1, 51)));
        plan.settle(served(1, 51), 1, Some(51));
        plan.sweep(7);
        assert!(plan.take_violations().is_empty());
        // The transparent entry moved neither witness: no pop was refused at
        // it, and none was admitted for it.
        assert_eq!(plan.refused_consults(), 0);
        assert_eq!(plan.admitted(), 1);
    }

    #[test]
    fn an_unspent_served_without_pop_leftover_mints_nothing() {
        let (plan, now) = armed_plan(None);
        at_step(&now, 8);
        // No drain consulted at all, so the hand off is never even walked past.
        // It still mints nothing: the frame it names reached its consumer
        // without a dequeue, so this gate owes it nothing.
        plan.install_step(8, &[hand_off(60)]).unwrap();
        plan.sweep(8);
        assert!(plan.take_violations().is_empty());
        assert_eq!(plan.refused_consults(), 0);
        assert_eq!(plan.admitted(), 0);
    }

    #[test]
    fn a_trailing_served_without_pop_leaves_the_next_consult_withholding() {
        let (plan, now) = armed_plan(None);
        at_step(&now, 9);
        // The cursor walks past the hand off and runs off the end, which is the
        // zero-quota answer and not an admission: a gate that treated a
        // transparent entry as spendable would pop a frame the recording did
        // not.
        plan.install_step(9, &[hand_off(70)]).unwrap();
        assert_eq!(plan.admit(), GateAnswer::Withhold);
        assert_eq!(plan.refused_consults(), 1);
        plan.sweep(9);
        assert!(plan.take_violations().is_empty());
    }

    /// A PANIC inside the gate latches the poison, and every reader says so.
    ///
    /// The fail-safe half (every later consult withholds) had no arm at any
    /// level, and the report half was worse than untested: `take_violations`
    /// answered `Vec::new()` and `violations_dropped` answered 0 on a poisoned
    /// gate, so a panic mid-run discarded the divergences already retained and
    /// the pass could report a clean verdict over a replay that served nothing
    /// from this stage after the panic.
    #[test]
    fn a_poisoned_gate_withholds_and_reports_its_own_findings() {
        let (plan, now) = armed_plan(None);
        at_step(&now, 3);
        plan.install_step(3, &[served(1, 21)]).unwrap();
        let GateAnswer::Exact(granted) = plan.admit() else {
            panic!("an installed nonzero quota admits");
        };
        // One retained finding BEFORE the panic: the drain delivered the one
        // frame the quota named and it carried a DIFFERENT sequence, which mints
        // exactly one violation (a short delivery would mint a second).
        plan.settle(granted, 1, Some(99));

        // POISON the gate: a panic while the plan lock is held. The hook is
        // silenced so the caught panic does not print a backtrace into a
        // passing run's output, and restored immediately after.
        let hook = std::panic::take_hook();
        std::panic::set_hook(Box::new(|_| {}));
        let caught = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _guard = plan.inner.lock().expect("the gate is not poisoned yet");
            panic!("a panic inside the gate");
        }));
        std::panic::set_hook(hook);
        assert!(caught.is_err());

        // The census and the arm both name the stage and the cause.
        let census = plan.capability();
        assert!(!census.enforceable);
        assert_eq!(
            census.reason,
            Some(ReadPlanRefusal::GateUnusable(plan_key(
                "relay",
                0,
                ReadStageRole::Body
            )))
        );
        assert!(plan.is_unusable());
        assert_eq!(
            plan.install_step(4, &[served(1, 22)]),
            Err(InstallRefusal::GateUnusable)
        );

        // Every later consult withholds rather than falling back to a live pop.
        at_step(&now, 4);
        assert_eq!(plan.admit(), GateAnswer::Withhold);
        assert_eq!(plan.admit(), GateAnswer::Withhold);

        // And the finding retained before the panic is still reported, which is
        // what lets the engine refuse the run naming this stage.
        let kept = plan.take_violations();
        assert_eq!(kept.len(), 1);
        assert_eq!(
            kept[0].kind,
            ReplayReadViolationKind::SequenceMismatch {
                expected: Some(21),
                observed: Some(99),
            }
        );
    }
}
