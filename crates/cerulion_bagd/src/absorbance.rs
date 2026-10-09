// SPDX-License-Identifier: AGPL-3.0-only
//! The per-topic ABSORBANCE VERDICT.
//!
//! # Why this exists
//!
//! The always-on Flashback plane's tap queues are BYTE-BUDGETED
//! (`CERULION_FLASHBACK_TAP_BUDGET_MB`, 64 MiB default),
//! so a robot's standing recorder footprint is bounded. What that trade spends
//! is ABSORBANCE: how long a drain stall a tap can ride out before its topic's
//! own SHM queue starts dropping frames. The trade holds on ONE
//! condition, and this module is that condition:
//!
//! > The reporting is the compliance mechanism: the boundary
//! > narrows AND is stated during, not just counted after.
//!
//! So every tapped topic carries a verdict, on every surface, while the run is
//! happening: the absorbance its depth buys at its own measured rate, against
//! the drain-gap tail THIS RUN actually measured.
//!
//! # The arithmetic, and why both formulations are the same comparison
//!
//! * `absorbance_us = depth / rate` — how long the queue covers.
//! * `required_depth = absorbing_depth(rate, q)` — the shipped
//!   [`DrainGapHistogram`] instrument, run backwards: the depth that would have
//!   absorbed every gap up to quantile `q`.
//!
//! The verdict is `absorbance_us >= tail_us`, and it is EXACTLY equivalent to
//! `depth >= required_depth`, because a depth is an integer and
//! `d >= x  <=>  d >= ceil(x)` for integer `d`. Both numbers are reported —
//! the time for a human, the depth because it is the actionable one ("you have
//! 4, you need 27") — and they can never disagree, which is why they are
//! derived from ONE tail rather than computed twice.
//!
//! # Zero new measurement
//!
//! Nothing here measures anything new. The gap distribution is the
//! [`DrainGapHistogram`]'s, already folded once per drive pass; the depth is
//! the recorded `tap_buffer_depth`; and the rate is read off the wire header the
//! drain already parses. This module is SURFACING, not construction.
//!
//! # What the numbers cannot see, stated here so every surface can repeat it
//!
//! 1. **The rate is a WINDOW AVERAGE** — over an UNINTERRUPTED stream, since a
//!    window that spans a silence is discarded (residual 5) and one that spans a
//!    publisher restart is re-anchored. A bursty topic's true absorbance
//!    against its BURST rate is worse than the figure here — a topic averaging
//!    10 Hz that arrives in 100-frame clumps needs a queue sized for the clump,
//!    not for the average.
//! 2. **Pre-baseline loss is invisible without `armed_before_producers`.** A
//!    contiguous prefix dropped before a tap's FIRST drain is absorbed AS the
//!    baseline, so `frames_lost` cannot see it and neither can this verdict
//!    (the `prefix_lost` marker proves it separately, and only on the
//!    `graph run --record` argv). That is why [`LossCountingBasis`] rides the
//!    same documents.
//! 3. **A floor rate makes the absorbance a CEILING.** See
//!    [`DrainRateEstimate::is_floor`].
//! 4. **The tail is a LOOP-CADENCE tail, not a per-tap undrained interval.**
//!    `DrainGapHistogram` folds the gap between consecutive DRIVE PASSES on
//!    every pass, whether or not a given tap was drained in it — and a tap at
//!    `RECORDING_TAP_STAGING_MAX_BYTES` STOPS draining (counted per topic as
//!    `TopicHealth::staging_full_passes`) while the loop keeps ticking. So
//!    during exactly the episodes that lose frames, the histogram reads ~10 ms
//!    while that topic's own queue went seconds undrained, and the verdict is
//!    OPTIMISTIC precisely then. Read a verdict beside that topic's
//!    `staging_full_passes`; a nonzero one means the number here is a floor on
//!    the stall the tap actually saw. (This is an inherited blind spot:
//!    a per-tap drain-suspension-duration histogram does not exist yet,
//!    and until it does no surface can do better.)
//! 5. **A window that spans a silence is DISCARDED, not diluted.** The cost is
//!    that a topic whose PERIOD reaches [`ABSORBANCE_MAX_IDLE_GAP`] never mints
//!    a rate and reports [`AbsorbanceVerdict::NoClaim`] forever — no verdict
//!    rather than a wrong one.

use std::time::{Duration, Instant};

use crate::DrainGapHistogram;

/// The quantile of the measured drain-gap distribution the verdict is taken
/// against.
///
/// `0.999` is the chosen quantile: read
/// `absorbing_depth(rate, 0.999)` per arm — that number, not the ladder, is
/// the loaded floor. A tail quantile rather than a mean because a queue
/// overflows on the ONE long pass: a 117 ms stall every 250 ms is invisible in
/// an average over ~100 passes/s while being exactly the condition that empties
/// a 16-frame queue at 700 Hz.
pub const ABSORBANCE_VERDICT_QUANTILE: f64 = 0.999;

/// The shortest observation window that may mint a rate.
///
/// Below it a tap has not been watched long enough for `frames / span` to mean
/// anything — two frames a millisecond apart would otherwise report a confident
/// 1 kHz on a topic that has published twice. A young tap reports
/// [`AbsorbanceVerdict::NoClaim`], which is the correct answer and is a DIFFERENT
/// statement from "it absorbs".
pub const ABSORBANCE_MIN_RATE_SPAN: Duration = Duration::from_secs(1);

/// How often the recorder re-evaluates every tap's verdict.
///
/// The verdict is a REPORTING artifact over quantities that are already being
/// measured, so its cadence is chosen for the operator watching a live robot
/// rather than for the arithmetic — one second is the same order as the status
/// feed an operator reads it beside. Evaluating per drive pass would run the
/// quantile walk ~100x/s per tap to re-read a distribution that moved by one
/// sample.
pub const ABSORBANCE_EVAL_INTERVAL: Duration = Duration::from_secs(1);

/// How many CONSECUTIVE absorbing evaluations close an open shortfall regime.
///
/// # Why a recovery has to DWELL, when the loud head does not
///
/// The comparison behind the verdict is a queue depth against a HISTOGRAM
/// BUCKET EDGE: `quantile_us` reports the upper edge of the bucket the 0.999
/// reading lands in, and there are eleven of them. A tap whose absorbance sits
/// BETWEEN two edges — the module's own worked example, a stock ingress depth of
/// 16 absorbing 9.6 ms against a ladder centred on the 5-25 ms rungs — flips
/// verdict whenever the reading crosses that boundary, which on a live recorder
/// it does routinely: `rank = ceil(0.999 * total)` walks as `total` grows, and
/// one preempted drive pass moves the reading a whole rung.
///
/// A `FailureRegimeLatch` suppresses REPEATS, never ALTERNATION. Closing the
/// regime on the first absorbing evaluation therefore re-arms the loud head, and
/// a strictly alternating tap emits one full multi-field `warn!` every OTHER
/// evaluation, for ever — no `Suppressed` arm is ever reached and no decade
/// re-announcement can ever fire, because the decade ladder rides `StillFailing`
/// and that needs the regime to still be open. At `ABSORBANCE_EVAL_INTERVAL`
/// that is ~43 k lines a day PER straddling topic on an always-on plane: the
/// disk-fill class, arriving through the very machine that exists to
/// prevent it.
///
/// So a recovery must be SUSTAINED, on the same reasoning as the sibling
/// cap-pressure guard in `lib.rs` ("a per-pass verdict would alternate between
/// 'binding' and 'healed' many times a second"). Three consecutive absorbing
/// evaluations is `3 x ABSORBANCE_EVAL_INTERVAL` = 3 s of clean readings — long
/// enough that a boundary flip cannot buy a re-arm, short enough that a genuine
/// fix is reported while the operator is still watching.
///
/// The asymmetry is deliberate and is the same one every latch in this repo
/// makes: the FIRST failure is loud IMMEDIATELY (a shortfall is losing frames
/// now), while the all-clear waits for evidence.
pub const ABSORBANCE_RECOVERY_DWELL: u32 = 3;

/// A gap this long between two drained frames DISCARDS the rate window.
///
/// DERIVED from the platform's own streaming-recency window rather than chosen:
/// a topic silent longer than `LIVENESS_STREAMING_RECENCY_MS` is not streaming
/// by this system's own definition (the liveness classifier says so on every
/// surface), so an average taken across that silence describes no regime the
/// topic was ever in. Const-asserted strictly greater than
/// [`ABSORBANCE_MIN_RATE_SPAN`], or a window could be discarded before it could
/// ever close.
pub const ABSORBANCE_MAX_IDLE_GAP: Duration =
    Duration::from_millis(cerulion_core::transport::liveness::LIVENESS_STREAMING_RECENCY_MS);

const _: () = assert!(
    ABSORBANCE_MAX_IDLE_GAP.as_millis() > ABSORBANCE_MIN_RATE_SPAN.as_millis(),
    "a rate window must be able to close before an idle gap can discard it"
);

/// A wire-`sequence` step this large is read as BACKWARD, not as a forward jump.
///
/// Sequences are `u32` and wrap, so forward motion is `wrapping_sub` and a wrap
/// is an ordinary small delta. A LARGE wrapping delta is the arithmetic image of
/// a backward step, which on a single-writer topic means the publisher's counter
/// restarted (a new process on the same topic). Half the space is the only
/// non-arbitrary cut: it is precisely "closer going backwards than forwards".
const SEQ_BACKWARD_THRESHOLD: u32 = u32::MAX / 2;

/// What a topic's loss numbers are able to see, carried on
/// the same document as the verdict.
///
/// This is the counting caveat as a stable token rather than as prose,
/// on the `shm_pinned_scope` / `shm_pinned_basis` precedent — a reader asking
/// "is a zero here good news" gets an answer without reading a paragraph.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LossCountingBasis {
    /// The recorder was armed BEFORE the producers (`--armed-before-producers`,
    /// which only `graph run --record` passes), so the recorder can prove a
    /// contiguous head loss and reports it as `TappedTopic::prefix_lost`. Loss
    /// on these taps is counted from the producer's first frame.
    PrefixProven,
    /// The recorder attached to producers that were ALREADY RUNNING. A
    /// contiguous prefix dropped before a tap's first drain is taken AS the
    /// baseline, so it is invisible to `frames_lost`, to `prefix_lost`, and to
    /// the absorbance verdict alike. A zero here means "nothing was counted",
    /// not "nothing was lost".
    PrefixInvisible,
}

impl LossCountingBasis {
    /// The stable token this basis renders as — ONE word for the JSON document
    /// and the log line alike.
    ///
    /// Without it the two surfaces disagreed (`prefix_invisible` in the bag,
    /// `PrefixInvisible` in the `tracing` field's `Debug`), so an operator
    /// grepping the token they saw in one found nothing in the other. The
    /// `AbsorbanceVerdict::as_str` rule, applied to its sibling.
    pub fn as_wire(self) -> &'static str {
        match self {
            LossCountingBasis::PrefixProven => "prefix_proven",
            LossCountingBasis::PrefixInvisible => "prefix_invisible",
        }
    }
}

/// A topic's observed publish rate, in millihertz.
///
/// # Why this is not `cerulion_core`'s `TopicRateEstimate`
///
/// That type is field-for-field identical and means the same thing, and the
/// duplication is DELIBERATE rather than missed: that estimate describes
/// what the ROBOT's liveness observer saw through its own depth-2 tap, and its
/// `is_floor` is clipped by THAT queue. This one describes what the RECORDER's
/// tap saw and is clipped by the recorder's. Merging them would make one value
/// answer two questions, which is how a number ends up meaning whichever plane
/// the reader had in mind. The name says which plane this one is about.
///
/// Millihertz, and the numerator is the publisher's COMMIT SEQUENCE rather than
/// the frames we caught (both on that type's precedent, for the same reason). A
/// frames-based rate is CLIPPED BY THE VERY QUEUE UNDER TEST: a depth-2 tap
/// drained every 10 ms can never observe more than 200 frames/s, so a
/// frames-counting rate would report exactly `depth / gap`, the absorbance
/// boundary itself, and the verdict would read "absorbs" on every topic no
/// matter how much it lost. Sequence deltas count the frames the queue dropped,
/// which is the whole point.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DrainRateEstimate {
    /// The rate, in millihertz (1000 = 1 Hz).
    pub millihertz: u64,
    /// The rate rests on FRAMES OBSERVED rather than on committed sequences, so
    /// it is a FLOOR — and therefore the absorbance derived from it is a
    /// CEILING, i.e. optimistic.
    ///
    /// Set when the sequence line cannot be differenced: a `multi_publisher`
    /// topic (declared or observed — `sequence` is a PER-PUBLISHER counter, so
    /// the batch max hops between unrelated ladders), or
    /// a topic that carried a headerless frame (fabricated seq 0).
    pub is_floor: bool,
}

/// A tap whose CAPACITY exceeds the byte budget
/// its depth was chosen against.
///
/// A queue's depth is pinned at port creation (iceoryx2 fixes
/// `subscriber_max_buffer_size` at CREATE), and the slot price is a per-PUBLISHER
/// property, so a publisher WIDER than the one the tap was sized against can put
/// an already-created queue over its budget with no way to shrink it. See
/// `Recorder::drain_taps` for why the disposition is to REPORT this rather than
/// to re-create the tap.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct OverBudget {
    /// The byte budget this tap's depth was chosen against.
    pub budget_bytes: u64,
    /// `depth x widest-slot-ever-seen` — what the queue can now pin.
    pub capacity_bytes: u64,
    /// The widest slot price ever read on this tap.
    ///
    /// `widest`, not `live`: the price is a STICKY MAXIMUM for the tap's life
    /// (see `TapState::priced_slot_bytes`), so a publisher that has since
    /// detached still counts toward it. `live` would read as "current", which
    /// is the one thing it is not.
    pub widest_slot_bytes: u64,
}

/// The verdict itself.
///
/// FOUR readings, and the two that are not "absorbs" or "short" are the
/// load-bearing ones: an absent verdict and a fabricated healthy one are both
/// worse than a plain "nothing to say".
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AbsorbanceVerdict {
    /// Nothing to compare. No rate could be established (the tap is younger than
    /// [`ABSORBANCE_MIN_RATE_SPAN`], or has seen no frames), or the recorder
    /// measured no drain gaps at all. NEVER a synonym for healthy.
    NoClaim,
    /// `absorbance >= the measured tail`: this tap rode out every drain gap up
    /// to [`ABSORBANCE_VERDICT_QUANTILE`] at this topic's own rate.
    Absorbs,
    /// `absorbance < the measured tail`: this tap is DEMONSTRABLY short of the
    /// stalls this very run measured. The loud line fires on this and only this.
    Short,
    /// The tail landed in the histogram's unbounded OVERFLOW bucket — the
    /// recorder stalled longer than the histogram can measure — AND this tap's
    /// absorbance is above that top edge too, so the two cannot be RANKED.
    ///
    /// Named for what a reader can DO with it rather than for the instrument:
    /// "ladder" is this repo's word for four unrelated things (the discovery
    /// ladder, the schema-resolution ladder, the flood latch's decade ladder, the viz
    /// ladder), and a wire token that has to be translated by every surface
    /// that renders it is a name that needs a footnote.
    ///
    /// Reported everywhere, never skipped, and deliberately NOT loud: a warn
    /// must name a DEMONSTRATED shortfall. Nothing genuinely short escapes into
    /// this arm — a tap whose absorbance is under the top edge is provably below
    /// an overflow tail and is classified [`Short`](Self::Short) from that
    /// floor.
    Unrankable,
}

impl AbsorbanceVerdict {
    /// The stable token this verdict renders as.
    pub fn as_str(self) -> &'static str {
        match self {
            AbsorbanceVerdict::NoClaim => "no_claim",
            AbsorbanceVerdict::Absorbs => "absorbs",
            AbsorbanceVerdict::Short => "short",
            AbsorbanceVerdict::Unrankable => "unrankable",
        }
    }

    /// Every verdict, for a caller that must handle each one — and for the
    /// cross-check that the JSON spelling and [`as_str`](Self::as_str) agree.
    pub const ALL: [AbsorbanceVerdict; 4] = [
        AbsorbanceVerdict::NoClaim,
        AbsorbanceVerdict::Absorbs,
        AbsorbanceVerdict::Short,
        AbsorbanceVerdict::Unrankable,
    ];
}

/// One topic's absorbance verdict — the whole reporting surface for
/// that topic, as one object.
///
/// NESTED rather than flattened into its host document because the numbers only
/// mean anything TOGETHER: a depth without a rate is not a verdict, and a
/// shortfall without the tail it was measured against is a number nobody can
/// check. Additive on every document that carries it — absent on a
/// pre-verdict artifact, which is UNKNOWN and not a passing verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct TopicAbsorbance {
    /// The receive-queue depth this tap was created with (the health row's
    /// `tap_buffer_depth`, repeated here so this object stands alone on surfaces
    /// that carry no other per-topic health).
    pub tap_buffer_depth: u64,
    /// The topic's observed publish rate in millihertz, or `None` when no rate
    /// could be established. `None` is UNKNOWN, never 0 Hz.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rate_mhz: Option<u64>,
    /// The rate is a FLOOR, so [`absorbance_us`](Self::absorbance_us) is a
    /// CEILING — see [`DrainRateEstimate::is_floor`]. Omitted when false.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub rate_is_floor: bool,
    /// `depth / rate` in microseconds: how long this queue covers at this
    /// topic's own WINDOW AVERAGE rate — over an uninterrupted stream, never a
    /// lifetime one. A bursty topic's absorbance against
    /// its burst rate is worse.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub absorbance_us: Option<u64>,
    /// The measured drain-gap at [`ABSORBANCE_VERDICT_QUANTILE`], in
    /// microseconds — the bucket's UPPER edge, so it is an upper bound on every
    /// gap inside it.
    ///
    /// `None` with a [`Unrankable`](AbsorbanceVerdict::Unrankable) or
    /// [`Short`](AbsorbanceVerdict::Short) verdict means the quantile landed in
    /// the unbounded OVERFLOW bucket: the stall is worse than the ladder can
    /// describe and no number can be served for it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub measured_tail_us: Option<u64>,
    /// The depth that WOULD have absorbed that tail at this rate — the shipped
    /// `DrainGapHistogram::absorbing_depth`, i.e. the actionable number. `None`
    /// when the tail is in the overflow bucket.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub required_depth: Option<u64>,
    /// How far short this tap is, in microseconds — EXACT when
    /// [`measured_tail_us`](Self::measured_tail_us) is present, and a LOWER
    /// BOUND (the ladder's top edge minus the absorbance) when the tail
    /// overflowed. The name is true in both cases, which is why it is not
    /// `shortfall_us`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub shortfall_at_least_us: Option<u64>,
    /// The verdict, AS OF THE LAST EVALUATION.
    ///
    /// Read it beside [`short_evaluations`](Self::short_evaluations): nothing
    /// here is sticky, so a tap that was short for twenty minutes and recovered
    /// before shutdown finalizes `Absorbs`.
    pub verdict: AbsorbanceVerdict,
    /// How many DRIVE-LOOP evaluations found this tap SHORT over the whole run.
    ///
    /// The recorder's own throttled evaluation is what feeds the latch this is
    /// read off; the SILENT evaluation that each finalize and each capture close
    /// runs is deliberately not counted (see `AbsorbanceReporting`). So a run
    /// too short to cross one `ABSORBANCE_EVAL_INTERVAL` can finalize `Short`
    /// carrying a zero here, which is accurate: nothing reported it.
    ///
    /// UNCONDITIONAL and never reset by a recovery (Principle #3, and the
    /// `FailureRegimeLatch::total_failures` rule this is read from). The verdict
    /// above is a SNAPSHOT, so without this a run that lost frames to a shallow
    /// tap for most of its life finalizes with a clean row, an empty terminal
    /// roll-up, and no `absorbance:` line in the summary — the whole episode
    /// surviving only as a `warn!` in a log that has scrolled away.
    ///
    /// `0` is omitted, so a tap that was never short is byte-identical to a
    /// row written before this field existed.
    #[serde(default, skip_serializing_if = "crate::is_zero_u64")]
    pub short_evaluations: u64,
    /// The byte budget this tap's depth was chosen against, or `None` when the
    /// tap is CEILING-DEEP (its depth is the topic's own provisioned
    /// `subscriber_buffer_size`).
    ///
    /// It is on the row because it decides which REMEDY is true.
    /// `CERULION_FLASHBACK_TAP_BUDGET_MB` moves a budgeted tap and nothing else,
    /// while a ceiling-deep tap reaches `Short` routinely (the stock ingress
    /// depth of 16 absorbs 9.6 ms at 1.66 kHz, against a ladder centred on the
    /// 5-25 ms rungs) — so a surface that printed the budget knob on every short
    /// row would name something that cannot help, on `cerulion bag record`
    /// every single time (that verb never builds a Flashback plane).
    ///
    /// Additive `Option` — absent means UNKNOWN or ceiling-deep, and
    /// [`remedy`](Self::remedy) treats them the same because the ceiling remedy
    /// is the one that is true in both.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub budget_bytes: Option<u64>,
    /// FIND L54: this tap's capacity exceeds the budget it was sized against
    /// (a wider publisher joined after the queue was created, and a depth cannot
    /// be changed afterwards). Omitted on every tap inside its budget.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub over_budget: Option<OverBudget>,
    /// A BUDGETED tap whose slot width was never priced, so whether it is inside
    /// its budget is UNKNOWN rather than established.
    ///
    /// `over_budget: None` alone cannot say this. It is minted from
    /// `priced_slot_bytes.and_then(..)`, which short-circuits when nothing has
    /// been priced — so a tap proven inside its budget and a tap nobody could
    /// measure render byte-identically, and the second reads as the first. That
    /// is the "a zero means nothing was COUNTED" rule this document already
    /// applies to `shm_pinned_bytes`, dropped on the row beside it.
    ///
    /// Reachable whenever the dynamic-config read has never yielded a live slot
    /// width: a budgeted tap whose publisher has not attached at any pass
    /// boundary yet, or one whose reads have failed. Always `false` on a
    /// ceiling-deep (`--record`) tap, which carries no budget to be inside.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub budget_unpriced: bool,
}

impl TopicAbsorbance {
    /// The rate as the ONE value it came from, re-bundled.
    ///
    /// The two fields are split apart on the wire because an absent rate must
    /// OMIT its key (absent means UNKNOWN, never 0 Hz) and a nested object
    /// cannot be omitted field by field. This puts them back together for a
    /// reader, and it is what [`inconsistency`](Self::inconsistency) checks the
    /// pair against.
    pub fn rate(&self) -> Option<DrainRateEstimate> {
        self.rate_mhz.map(|millihertz| DrainRateEstimate {
            millihertz,
            is_floor: self.rate_is_floor,
        })
    }

    /// The row CONTRADICTS itself, and how.
    ///
    /// Every field here is `pub` and the type is `Deserialize`, so a bag from an
    /// unknown robot — or a hand-edited attachment, or a writer from a version
    /// nobody has — can present a combination [`absorbance_verdict`] cannot
    /// produce. Both renderers key on `verdict` for their counts and read the
    /// numbers independently, so without a check an `Absorbs` row carrying a
    /// shortfall is COUNTED as healthy and its shortfall silently dropped, and a
    /// `Short` row with no shortfall is named with its magnitude nowhere. A
    /// reader that checks first can say "this row disagrees with itself"
    /// instead, which is the true answer and the one this vocabulary exists
    /// to give.
    ///
    /// Also `debug_assert`ed at the end of [`absorbance_verdict`], so the
    /// PRODUCER cannot drift from these rules with only prose to stop it.
    pub fn inconsistency(&self) -> Option<&'static str> {
        self.inconsistency_against(None)
    }

    /// [`inconsistency`](Self::inconsistency), against the LADDER the row was
    /// ranked on.
    ///
    /// # Every field, classified
    ///
    /// One defect can hide behind four fields — a decoded field
    /// nothing asked about (the rate, the absorbance, the
    /// `Unrankable` shape, the required depth), each rendered to an
    /// operator as if a producer had computed it. Those are not four
    /// bugs; they are one, once per field. So the table below covers the
    /// WHOLE struct, and every field is in exactly one class:
    ///
    /// | field | class | what holds it |
    /// |---|---|---|
    /// | `tap_buffer_depth` | INPUT | the free variable the absorbance derivation reads; a wrong depth surfaces there |
    /// | `rate_mhz` | derived-checked | strictly positive wherever a verdict rests on it, and never `Some(0)` at all |
    /// | `rate_is_floor` | shape-checked | a floor LABEL requires the positive rate it labels |
    /// | `absorbance_us` | derived-checked | `depth / rate` exactly, and present iff a rate is |
    /// | `measured_tail_us` | membership-checked | a SERVED quantile is a bucket EDGE of the ladder it was ranked on |
    /// | `required_depth` | derived-checked | `ceil(rate x tail)` where a tail is served, ABSENT where none is |
    /// | `shortfall_at_least_us` | derived-checked | `tail - absorbance` served, `edge + 1 - absorbance` tail-less |
    /// | `verdict` | the discriminant | every arm below is a rule about it |
    /// | `short_evaluations` | UNVALIDATED | a free running total off the reporting latch: every value is producible, and it is independent of every other field (a tap short all run may finalize `Absorbs`) |
    /// | `budget_bytes` | shape-checked | present iff budgeted, and never `Some(0)` — the call site maps a zero budget to `None` |
    /// | `over_budget` | derived-checked | `capacity == widest x depth`, over the row's OWN budget, and present only while it exceeds it |
    /// | `budget_unpriced` | shape-checked | mutually exclusive with an occupancy, and requires a budget to be unpriced against |
    ///
    /// What no reader can check is a writer that moved TWO fields together —
    /// a depth and an absorbance that agree with each other and with nothing
    /// that happened. That is the floor of this vocabulary, not a gap in it: a
    /// row is judged self-consistent, never true.
    ///
    /// # Why the ladder is a parameter rather than the constant
    ///
    /// Some of the rules below are not arithmetic on the row's own numbers.
    /// They are the two halves of ONE partition: inside the ladder-overflow
    /// branch the producer emits `Short` while the absorbance is at most the
    /// ladder's LAST edge and `Unrankable` above it, so each verdict is
    /// unproducible on the other's side of that edge. Taking the first as the
    /// worked example: a
    /// `Short` row carrying NO measured tail is the OVERFLOW arm, which
    /// [`absorbance_verdict`] reaches only when the absorbance is at most the
    /// ladder's LAST edge (above it the tail cannot be bounded from below at
    /// all, and the verdict is [`Unrankable`](AbsorbanceVerdict::Unrankable)).
    /// Checking that needs a ladder, and there are two of them in play:
    ///
    /// * the PRODUCER ranks against the `DrainGapHistogram` it was handed, and
    ///   [`absorbance_verdict`] is `pub` — so a caller passing a histogram with
    ///   a taller ladder would trip the debug assert on a row it legitimately
    ///   produced if the check used this build's constant; and
    /// * a READER holds the recording's OWN `edges_us`, which travels with the
    ///   counts for exactly this reason (`DrainGapHistogram`'s wire shape is
    ///   self-describing so "a bag read years later does not need the constant
    ///   to interpret the array").
    ///
    /// So both pass what they hold. `None` means the caller has no ladder and
    /// falls back to this build's [`DRAIN_GAP_BUCKET_EDGES_US`], which is the
    /// only vocabulary it can speak; an EMPTY slice means the row was ranked
    /// against a histogram with no edges, where the arm cannot be asked at all
    /// and is skipped rather than guessed.
    ///
    /// [`DRAIN_GAP_BUCKET_EDGES_US`]: crate::DRAIN_GAP_BUCKET_EDGES_US
    pub fn inconsistency_against(&self, ladder_edges_us: Option<&[u64]>) -> Option<&'static str> {
        let last_edge_us = ladder_edges_us
            .unwrap_or(crate::DRAIN_GAP_BUCKET_EDGES_US)
            .last()
            .copied();
        // A REPORTED rate is a STRICTLY POSITIVE one. `rate_mhz`'s own contract
        // is that absence means UNKNOWN and never 0 Hz, and
        // `absorbance_verdict_inner` filters `millihertz > 0` BEFORE the row is
        // built — so `None` and `Some(0)` are equally unproducible wherever a
        // rate is REQUIRED, and rejecting only the first left the second as a
        // hole a forged row walks straight through.
        let rate_reported = self.rate_mhz.is_some_and(|mhz| mhz > 0);
        // `rate_is_floor` is minted from the ALREADY-FILTERED rate
        // (`rate.is_some_and(|r| r.is_floor)`), so a floor LABEL over anything
        // but a positive rate is unproducible in BOTH shapes, for that one
        // reason — hence one guard, and a message naming which shape it caught.
        if self.rate_is_floor && !rate_reported {
            return Some(if self.rate_mhz.is_none() {
                "the rate is labelled a FLOOR but no rate is reported"
            } else {
                "the rate is labelled a FLOOR, yet the rate it labels is ZERO"
            });
        }
        // EVERY verdict except `NoClaim` is DERIVED from a rate, so a row that
        // reaches one without reporting a positive rate was written by two
        // different hands. `absorbance_verdict_inner` filters a zero/absent rate
        // FIRST and returns immediately with `NoClaim` — the absorbance is
        // `depth / rate`, so with no denominator there is nothing to compare and
        // nothing to say. There is therefore no producible `Absorbs`, `Short` or
        // `Unrankable` row with `rate_mhz` either absent or zero.
        //
        // Left unchecked, the arithmetic arms below are all SATISFIED by such a
        // row — they compare absorbance against the tail and never ask where the
        // absorbance came from — so `bag info` aggregated it as a real verdict
        // and rendered `rate unknown -> absorbs`, i.e. a health claim about a
        // topic whose rate the recording says it never knew. The `Some(0)` shape
        // is the same defect wearing a number: the row renders `0.000 Hz` and is
        // COUNTED as an absorbing topic.
        //
        // ONE-DIRECTIONAL, deliberately: the converse is producible and sound.
        // A `NoClaim` row may legitimately CARRY a rate — the vacant-histogram
        // arm computes the absorbance, then declines to rank it because the run
        // measured no cadence ("no measurement ran" is not "it held").
        if self.verdict != AbsorbanceVerdict::NoClaim && !rate_reported {
            return Some(if self.rate_mhz.is_none() {
                "the row reaches a verdict without reporting the rate that verdict is \
                 derived from"
            } else {
                "the row reaches a verdict derived from a rate of ZERO, which the \
                 absorbance arithmetic (depth / rate) has no denominator for"
            });
        }
        // …and the ONE cell that rule cannot reach: a `NoClaim` row carrying a
        // ZERO. `NoClaim` is exempt above because it may legitimately carry the
        // rate it DECLINED to rank — but that exemption is for a rate the
        // producer accepted, and a zero never is (`rate_mhz`'s own contract:
        // absent is UNKNOWN, never 0 Hz, and `absorbance_verdict_inner` filters
        // it before the row is built). Left unchecked, a `NoClaim` row renders
        // `0.000 Hz` — a measurement, on the one verdict that exists to say no
        // measurement was made.
        if self.rate_mhz == Some(0) {
            return Some(
                "the row reports a rate of ZERO, a value the producer filters before a row is \
                 built — an unknown rate is ABSENT, never 0 Hz",
            );
        }
        if self.budget_unpriced && self.over_budget.is_some() {
            return Some("the budget occupancy is UNPRICED, yet the row reports one");
        }
        if self.budget_unpriced && self.budget_bytes.is_none() {
            return Some("the budget occupancy is UNPRICED, yet this tap carries no budget");
        }
        // A budget of ZERO is not a budget. The production call site is
        // `(tap.budget_bytes > 0).then_some(..)`, so a budgeted tap always
        // carries a positive number and a ceiling-deep one carries `None` —
        // the two states `depth_mode` and `remedy` are keyed on. `Some(0)`
        // reads as the first and behaves as neither: the row renders
        // `(budgeted)` and sends the operator to
        // `CERULION_FLASHBACK_TAP_BUDGET_MB`, a knob that moves a tap sized
        // against a budget it does not have.
        if self.budget_bytes == Some(0) {
            return Some(
                "the tap is labelled BUDGETED, yet the budget it names is ZERO — a ceiling-deep \
                 tap carries no budget at all rather than an empty one",
            );
        }
        // …and the OCCUPANCY, which is as derived as the absorbance is.
        //
        // `budget_occupancy` mints all three numbers from two inputs: it is
        // `Some` only while `slot x depth > budget`, and then reports exactly
        // that budget, that product, and that slot width. None of the three is
        // free data, and every one of them is RENDERED — `render_line` prints
        // "depth N is OVER BUDGET: priced for B B, now C B", which is
        // provisioning guidance an operator sizes a queue from.
        //
        // The budget agreement is checked against the ROW's own
        // `budget_bytes`, which the production call site passes from the same
        // `tap.budget_bytes` the occupancy was computed from, so a row naming
        // two different budgets was written by two hands. The capacity mirrors
        // the producer's saturating product exactly.
        if let Some(ob) = self.over_budget {
            if Some(ob.budget_bytes) != self.budget_bytes {
                return Some(
                    "the row is OVER BUDGET against a budget that is not the one the row itself \
                     carries",
                );
            }
            if ob.capacity_bytes != ob.widest_slot_bytes.saturating_mul(self.tap_buffer_depth) {
                return Some(
                    "the capacity the row reports OVER BUDGET is not the one its own depth and \
                     widest slot derive (depth x widest slot)",
                );
            }
            if ob.capacity_bytes <= ob.budget_bytes {
                return Some(
                    "the row reports being OVER BUDGET, yet the capacity it names is within the \
                     budget it names",
                );
            }
        }
        match self.verdict {
            AbsorbanceVerdict::NoClaim => {
                if self.measured_tail_us.is_some()
                    || self.required_depth.is_some()
                    || self.shortfall_at_least_us.is_some()
                {
                    return Some("NO CLAIM was made, yet the row carries a comparison");
                }
            }
            AbsorbanceVerdict::Absorbs => {
                if self.shortfall_at_least_us.is_some() {
                    return Some("the tap ABSORBS, yet the row carries a shortfall");
                }
                let (Some(tail), Some(absorbance)) = (self.measured_tail_us, self.absorbance_us)
                else {
                    return Some("the tap ABSORBS, yet nothing was compared");
                };
                // THE ARITHMETIC, checked where both operands are present: the
                // verdict IS `absorbance >= tail`, so a row claiming to absorb
                // while reporting less than the tail it names has had its
                // verdict and its numbers written by two different hands.
                if absorbance < tail {
                    return Some(
                        "the tap ABSORBS, yet its absorbance is under the tail it was ranked \
                         against",
                    );
                }
            }
            AbsorbanceVerdict::Short => {
                if self.shortfall_at_least_us.is_none() {
                    return Some("the tap is SHORT, yet the row says by how much nowhere");
                }
                let Some(absorbance) = self.absorbance_us else {
                    return Some("the tap is SHORT, yet it reports no absorbance");
                };
                // The same arithmetic from the other side. Only checked when a
                // tail is present: a SHORT verdict is also reachable with NO
                // tail at all (the ladder-overflow arm, where the tail is known
                // only to exceed the last edge), and that row is sound.
                //
                // The DEPTH-ZERO row (`absorbance_us = Some(0)`, tail present)
                // does not collide with this, and the reason is load-bearing
                // rather than obvious: `DRAIN_GAP_BUCKET_EDGES_US` starts at
                // 100 us, so a SERVED tail is never 0 and `0 >= tail` is never
                // true. A future ladder with a zero first edge would need this
                // arm revisited.
                if self.measured_tail_us.is_some_and(|tail| absorbance >= tail) {
                    return Some(
                        "the tap is SHORT, yet its absorbance meets the tail it was ranked \
                         against",
                    );
                }
                // …and the MAGNITUDE is the gap itself, not a free number beside
                // it. Where a tail is SERVED the producer emits exactly
                // `tail - absorbance`, on BOTH arms that can reach here: the
                // ordinary short arm computes that difference, and the
                // DEPTH-ZERO arm reports `tail` against an absorbance of 0,
                // which is the same subtraction. So any other value on a
                // tail-carrying row was written by a second hand.
                //
                // # Why EXACT, when the field is named `at_least`
                //
                // The name is earned by the arm this rule does NOT touch: the
                // ladder-OVERFLOW row has no tail, and there `edge + 1 -
                // absorbance` is a genuine conservative bound on a magnitude
                // nobody measured. Where the tail IS measured there is nothing
                // to be conservative about, and the two directions fail
                // differently rather than equally:
                //
                // * a value ABOVE the gap is simply FALSE — the row asserts a
                //   floor its own two numbers refute; while
                // * a value BELOW it is true-but-unproducible, and it
                //   UNDER-REPORTS SEVERITY, which is the direction that misleads.
                //   The shape: `1` us claimed against a
                //   20_000 us gap reads as a rounding error on a row describing
                //   a tap that misses its stall tail by 20 ms.
                //
                // A `<=` rule would admit the second and is the looser
                // alternative that was considered. It is not taken, because this
                // function's contract is "a combination `absorbance_verdict`
                // cannot produce" rather than "a statement that is false", and
                // because the under-reporting direction is the one an operator
                // acts on. THE COST: a foreign producer that deliberately
                // emitted conservative bounds on tail-carrying rows would be
                // flagged here. That cost is bounded and LOUD — the row renders
                // as INCONSISTENT with `absorbance_us`, `measured_tail_us` and
                // `shortfall_at_least_us` all printed, so a reader can do the
                // subtraction and see which convention the writer used — rather
                // than being silently counted with an invented magnitude.
                if let (Some(tail), Some(claimed)) =
                    (self.measured_tail_us, self.shortfall_at_least_us)
                {
                    if claimed != tail.saturating_sub(absorbance) {
                        return Some(
                            "the tap is SHORT, yet the shortfall it claims is not the gap \
                             between its absorbance and the tail it was ranked against",
                        );
                    }
                }
                // …and the SAME arithmetic for the tail-less OVERFLOW row, whose
                // bound is the ladder rather than a served number.
                //
                // That arm is reached only when the absorbance is AT MOST the
                // ladder's last edge (`absorbance_us <= edge`, boundary
                // inclusive and reachable — see `absorbance_verdict_inner`).
                // Above the edge nothing bounds the tail from below, so the
                // producer says `Unrankable` and stays SILENT; a decoded row
                // claiming `Short` up there therefore describes a comparison
                // that was never made, and both renderers would have counted it
                // as a genuine shortfall and printed its invented magnitude.
                //
                // Skipped when the caller holds no rankable ladder — an absence
                // of the vocabulary is not evidence against the row.
                if self.measured_tail_us.is_none()
                    && last_edge_us.is_some_and(|edge| absorbance > edge)
                {
                    return Some(
                        "the tap is SHORT with NO measured tail, yet its absorbance is above \
                         the highest gap the ladder can rank — the overflow arm cannot produce \
                         this row",
                    );
                }
                // …and WITHIN the ladder, the tail-less row's bound is DERIVED,
                // not free: the overflow arm emits exactly
                // `edge + 1 - absorbance` ("the tail is at least `edge + 1`",
                // its own comment), so on a tail-less `Short` row the claimed
                // shortfall is as fixed a function of the row's numbers as the
                // served-tail subtraction above. A claim BELOW the derived bound
                // under-reports severity — the direction an operator acts on —
                // and a claim ABOVE it asserts a floor the row's own numbers do
                // not establish. Same contract as both siblings: "a combination
                // `absorbance_verdict` cannot produce", judged against the
                // ladder the row was ranked on (the recording's own, when the
                // caller holds one). Skipped without a ladder or a claim —
                // absence of the vocabulary is not evidence against the row.
                if self.measured_tail_us.is_none() {
                    if let (Some(edge), Some(claimed)) = (last_edge_us, self.shortfall_at_least_us)
                    {
                        if absorbance <= edge
                            && claimed != edge.saturating_add(1).saturating_sub(absorbance)
                        {
                            return Some(
                                "the tap is SHORT with NO measured tail, yet the shortfall it \
                                 claims is not the ladder-derived bound (one past the last \
                                 edge, minus its absorbance) the overflow arm emits",
                            );
                        }
                    }
                }
            }
            AbsorbanceVerdict::Unrankable => {
                // The producer's image of this row is EXACT, and every field of
                // it is checked here, not only the measured tail. The
                // arm is reached at a single point in `absorbance_verdict_inner`
                // — the `_` of the ladder match inside the OVERFLOW branch — and
                // to get there a row must have passed the positive-rate filter,
                // the depth-zero return and the vacant-histogram return, and
                // then found NO served quantile. So:
                //
                // * `measured_tail_us` is None (the branch is entered on
                //   `quantile_us(..) == None`, and that is the only writer);
                // * `required_depth` is None (both writers sit inside the
                //   served-tail arms, which this branch is the alternative to);
                // * `shortfall_at_least_us` is None — the arm is SILENT by
                //   design, which is the whole distinction between it and the
                //   `Short` overflow row beside it; and
                // * `absorbance_us` is `Some` UNCONDITIONALLY, written before
                //   the vacancy check and therefore before any of the three
                //   verdicts downstream of it.
                //
                // The last one is worth stating because it is easy to get
                // backwards: `Unrankable` makes no absorbance-DEPENDENT claim,
                // and it was read as "so the field is optional here". It is not
                // — the producer computed the absorbance to decide the verdict.
                // A row that omits it is two-handed, and it also silently
                // disarms both the derivation check at the end of this function
                // and the ladder rule below, each of which reads that field.
                if self.measured_tail_us.is_some() {
                    return Some("the pair is UNRANKABLE, yet the row carries a measured tail");
                }
                if self.required_depth.is_some() {
                    return Some(
                        "the pair is UNRANKABLE, yet the row names the depth that would have \
                         absorbed a tail it has no number for",
                    );
                }
                if self.shortfall_at_least_us.is_some() {
                    return Some(
                        "the pair is UNRANKABLE, yet the row claims a shortfall — that is the \
                         SHORT overflow row, which this arm is the alternative to",
                    );
                }
                let Some(absorbance) = self.absorbance_us else {
                    return Some("the pair is UNRANKABLE, yet it reports no absorbance");
                };
                // …and THE LADDER, the exact mirror of the tail-less `Short`
                // rule above. That one refuses a `Short` row whose absorbance is
                // ABOVE the last edge; this refuses an `Unrankable` row whose
                // absorbance is AT OR BELOW it, and the two together partition
                // the overflow branch the way the producer does.
                //
                // `Unrankable`'s own doc states the rule: "a tap whose
                // absorbance is under the top edge is provably below an overflow
                // tail and is classified `Short` from that floor". The edge is
                // reachable and INCLUSIVE on the `Short` side (a gap of exactly
                // the last edge lands in the BOUNDED top bucket, so the overflow
                // bucket holds only gaps strictly greater), which is why this
                // comparison is `<=` and not `<` — an absorbance of exactly the
                // edge is a row the producer emits as `Short`, never here.
                //
                // Left unchecked this was the loosest arm of the four: a decoded
                // row could name any absorbance at all, and `bag info` counted
                // it under "met a stall the histogram cannot measure" — i.e. a
                // topic reported as beyond the instrument's reach while its own
                // numbers say the instrument ranked it fine.
                //
                // Skipped without a rankable ladder, like its sibling. THE
                // COST, and it is the same one: a row genuinely ranked against
                // an EDGE-LESS histogram is `Unrankable` at ANY absorbance (with
                // no last edge the producer takes this arm unconditionally), so
                // a reader that holds no ladder and falls back to this build's
                // would flag it. The production reader does not — it passes the
                // recording's own `edges_us`, which for such a histogram is
                // `Some(&[])`, where the arm is skipped rather than guessed.
                if last_edge_us.is_some_and(|edge| absorbance <= edge) {
                    return Some(
                        "the pair is UNRANKABLE, yet its absorbance is at or below the highest \
                         gap the ladder can rank — a tap down there is provably under an \
                         overflow tail, and the producer ranks it SHORT from that floor",
                    );
                }
            }
        }
        // A POSITIVE rate and an absorbance travel TOGETHER, in both
        // directions. `absorbance_verdict_inner` returns before writing an
        // absorbance while the rate is absent, and writes one UNCONDITIONALLY
        // once past that return — the depth-zero arm stamps `Some(0)` — so each
        // half is unproducible without the other.
        //
        // The verdict arms already name a missing absorbance for the three
        // verdicts that READ one. This covers `NoClaim`, the fourth: it may
        // legitimately carry a rate (the vacant-histogram arm computes the
        // absorbance and then declines to rank it), and a row that carried the
        // rate WITHOUT it rendered `100.000 Hz -> absorbance unknown`, which a
        // reader takes for "the queue could not be measured" rather than for a
        // row two hands wrote.
        if rate_reported != self.absorbance_us.is_some() {
            return Some(if self.absorbance_us.is_none() {
                "the row reports a rate, yet no absorbance was derived from it"
            } else {
                "the row reports an absorbance, yet no rate for it to have been derived from"
            });
        }
        // …and THE TAIL IS A RUNG, not a number.
        //
        // A SERVED quantile is `edges_us.get(i)` — `DrainGapHistogram::
        // quantile_us` returns a bucket's UPPER EDGE and has no other exit that
        // yields a value — so a `measured_tail_us` the ladder does not contain
        // is a measurement the instrument cannot make. Every arm above compares
        // against that number without asking where on the ladder it sits, and
        // the arithmetic is satisfied by any value at all: a forged 37 ms tail
        // (between the 25 ms and 50 ms rungs) buys a `Short` verdict, a
        // shortfall computed from it, and a `needs depth` an operator
        // provisions a queue from.
        //
        // Skipped without a rankable ladder, like its siblings: with `Some(&[])`
        // the row was ranked on a histogram that has no rungs to be on, and
        // absence of the vocabulary is not evidence against the row. (Such a
        // histogram can serve no quantile at all, so a tail-carrying row is
        // already unproducible there — but by an argument this arm does not
        // make, and the empty-ladder skip is this function's stated contract.)
        if let Some(tail) = self.measured_tail_us {
            let ladder = ladder_edges_us.unwrap_or(crate::DRAIN_GAP_BUCKET_EDGES_US);
            if !ladder.is_empty() && !ladder.contains(&tail) {
                return Some(
                    "the measured tail is not a bucket edge of the ladder the row was ranked on, \
                     and a served quantile is always one",
                );
            }
        }
        // …and THE REQUIRED DEPTH, the one number on the row an operator ACTS
        // on — it is what `render_line` prints as `needs depth N`, and what a
        // reader re-provisions a queue from.
        //
        // `required_depth` is as derived as the absorbance is: the producer
        // calls the module's `required_depth` helper at BOTH sites that serve a
        // tail (the ordinary arm and the depth-zero arm), and that helper
        // returns `Some` unconditionally — the shipped
        // `DrainGapHistogram::absorbing_depth` when it answers, and otherwise
        // the exact integer form `ceil(mhz * tail / 1e9)` those two are
        // `debug_assert`ed equal on. So a served tail always mints one, and its
        // value is fixed by the rate and the tail already on the row.
        //
        // The tail-LESS half is the other side of the same fact: the ladder-
        // overflow arm has no tail to require a depth FOR, and never writes one.
        // `NoClaim` and `Unrankable` are already told this by their own arms,
        // which name the sharper contradiction; the row this half newly reaches
        // is the `Short` OVERFLOW row, where a forged depth rendered as
        // provisioning guidance derived from a stall nobody could measure.
        //
        // THE COST, and it is the mirror of the absorbance rule's: the
        // mirrored arithmetic is the INTEGER form, so a release-built producer
        // whose `absorbing_depth` disagreed with it in `f64` would be flagged
        // here. That disagreement is what the producer's own `debug_assert`
        // exists to forbid, and the module's equivalence claim rests on it.
        match (self.measured_tail_us, self.required_depth) {
            (Some(tail), claimed) => {
                // Scoped to a positive rate for the same reason the absorbance
                // derivation is: with no denominator the producer emits neither
                // the tail nor the depth, and a tail-carrying row that reports
                // no rate has already been convicted above.
                if let Some(mhz) = self.rate_mhz.filter(|m| *m > 0) {
                    let derived = (u128::from(mhz) * u128::from(tail))
                        .div_ceil(1_000_000_000u128)
                        .min(u128::from(u64::MAX)) as u64;
                    if claimed != Some(derived) {
                        return Some(
                            "the depth the row says would have absorbed its tail is not the one \
                             its own rate and tail derive (ceil(rate x tail))",
                        );
                    }
                }
            }
            (None, Some(_)) => {
                return Some(
                    "the row names the depth that would have absorbed a tail it has no number \
                     for — the overflow arm requires a depth for nothing",
                );
            }
            (None, None) => {}
        }
        // …and THE DERIVATION ITSELF, which every arm above takes on trust.
        //
        // They all compare the absorbance against something ELSE on the row —
        // the tail, the ladder, the shortfall — and none asks where the
        // absorbance came from. It is not free data: `absorbance_verdict_inner`
        // computes it from this row's OWN depth and rate on both arms that set
        // it (the ordinary `depth * 1e9 / mhz`, and the DEPTH-ZERO arm, which is
        // that same expression at depth 0). So a decoded row could inflate it
        // and buy a verdict its own two operands refute, with every arm
        // satisfied: the shape is depth 3 at 100 Hz claiming 40 ms of
        // absorbance where the arithmetic gives 30, and `bag info` counted the
        // topic as absorbing its stalls.
        //
        // Judged LAST so the verdict-specific arms keep naming the sharper
        // contradiction where a row has both.
        //
        // `tap_buffer_depth` is a REQUIRED field — no `Option`, no serde default
        // — so a decoded row ALWAYS carries the depth this recomputes from; a
        // document missing it fails to deserialize whole and never reaches here.
        // There is therefore no non-`NoClaim` shape to scope this to.
        //
        // Scoped to rows carrying BOTH a positive rate and an absorbance: with
        // no rate the producer emits no absorbance either, and the only two arms
        // that READ an absorbance already refuse a row that reaches them without
        // one, by name. Mirrors the producer's arithmetic exactly, saturation
        // included — `u128` throughout, truncating division, clamped at
        // `u64::MAX` — so an off-by-one microsecond is a mismatch on both sides.
        // The message carries no numbers because it cannot (`&'static str`), and
        // needs none: `render_line` prints the depth, the rate and the
        // absorbance on the same line, so a reader can do the division and see
        // which of the three the writer got wrong.
        if let (Some(mhz), Some(claimed)) = (self.rate_mhz.filter(|m| *m > 0), self.absorbance_us) {
            let derived = (u128::from(self.tap_buffer_depth) * 1_000_000_000u128 / u128::from(mhz))
                .min(u128::from(u64::MAX)) as u64;
            if claimed != derived {
                return Some(
                    "the absorbance the row reports is not the one its own depth and rate \
                     derive (depth / rate)",
                );
            }
        }
        None
    }

    /// Which depth rule set this tap's queue — the stable token every surface
    /// renders and the loud line logs.
    pub fn depth_mode(&self) -> &'static str {
        if self.budget_bytes.is_some() {
            "budgeted"
        } else {
            "ceiling"
        }
    }

    /// The remedy that is TRUE for this tap.
    ///
    /// A budgeted tap's depth moves with `CERULION_FLASHBACK_TAP_BUDGET_MB`; a
    /// ceiling-deep tap's does not, and telling an operator otherwise sends them
    /// to a knob that cannot help. See [`budget_bytes`](Self::budget_bytes).
    pub fn remedy(&self) -> &'static str {
        if self.budget_bytes.is_some() {
            "raise CERULION_FLASHBACK_TAP_BUDGET_MB, or record this topic explicitly \
             (a --record tap stays ceiling-deep)"
        } else {
            "this tap is CEILING-DEEP, so no recorder knob moves it: raise the topic's own \
             subscriber_buffer_size where it is declared, or shorten the recorder's drive-loop \
             stalls"
        }
    }

    /// A compact one-line rendering, shared by every surface so they cannot
    /// drift apart.
    ///
    /// Milliseconds TO TWO DECIMALS, not whole milliseconds. The band the
    /// question lives in is 5-25 ms, but the ladder's four lowest edges are
    /// 100/250/500/1000 MICROseconds and a healthy recorder's pass cadence sits
    /// down there — so a whole-millisecond rendering prints `measured stall tail
    /// 0 ms` on the ordinary case, and a genuinely short sub-millisecond row
    /// prints `absorbs 0 ms vs measured stall tail 0 ms`, leaving `needs depth`
    /// as the only readable number on the line. Two decimals resolve 10 us,
    /// which covers the whole ladder, and one format covers every magnitude
    /// (no unit switching for a reader to notice).
    pub fn render_line(&self) -> String {
        let rate = match self.rate_mhz {
            Some(mhz) => format!(
                "{}.{:03} Hz{}",
                mhz / 1000,
                mhz % 1000,
                if self.rate_is_floor { " (floor)" } else { "" }
            ),
            None => "rate unknown".to_string(),
        };
        let absorbance = match self.absorbance_us {
            Some(us) => format!("absorbs {}", render_ms(us)),
            None => "absorbance unknown".to_string(),
        };
        let against = match (self.verdict, self.measured_tail_us) {
            (AbsorbanceVerdict::NoClaim, _) => "no claim (nothing to compare)".to_string(),
            (_, Some(tail)) => format!(
                "measured stall tail {}{}",
                render_ms(tail),
                match self.required_depth {
                    Some(d) => format!(", needs depth {d}"),
                    None => String::new(),
                }
            ),
            (_, None) => "measured stall tail is WORSE THAN THE LADDER CAN DESCRIBE".to_string(),
        };
        let over = match (self.over_budget, self.budget_unpriced) {
            (Some(ob), _) => format!(
                "  [depth {} is OVER BUDGET: priced for {} B, now {} B]",
                self.tap_buffer_depth, ob.budget_bytes, ob.capacity_bytes
            ),
            // UNKNOWN, said out loud. Silence here would be read as "inside its
            // budget", which is a claim nothing measured.
            (None, true) => concat!(
                "  [budget occupancy UNPRICED: no live slot width was ever read, ",
                "so whether this tap is inside its budget is unknown]"
            )
            .to_string(),
            (None, false) => String::new(),
        };
        let fix = if self.verdict == AbsorbanceVerdict::Short {
            format!("  fix: {}", self.remedy())
        } else {
            String::new()
        };
        format!(
            "depth {} ({}) @ {rate} -> {absorbance} vs {against} [{}]{over}{fix}",
            self.tap_buffer_depth,
            self.depth_mode(),
            self.verdict.as_str()
        )
    }
}

/// Microseconds as milliseconds to two decimals — the ONE time format every
/// absorbance surface renders through, so a `bag info` row and a log line
/// cannot disagree about a number.
fn render_ms(us: u64) -> String {
    format!("{}.{:02} ms", us / 1000, (us % 1000) / 10)
}

/// The PURE verdict function.
///
/// Every input is plain data, so the whole rule is oracle-testable without a
/// recorder, a tap, or a clock.
pub fn absorbance_verdict(
    tap_buffer_depth: u64,
    rate: Option<DrainRateEstimate>,
    gaps: &DrainGapHistogram,
    budget_bytes: Option<u64>,
    over_budget: Option<OverBudget>,
    budget_unpriced: bool,
) -> TopicAbsorbance {
    let out = absorbance_verdict_inner(
        tap_buffer_depth,
        rate,
        gaps,
        budget_bytes,
        over_budget,
        budget_unpriced,
    );
    // The producer's own rules, asserted rather than merely documented — at the
    // ONE exit every path funnels through, so the check is TOTAL. Placed
    // at the end of the body it would sit past three earlier `return out`s (no rate, depth
    // zero, vacant histogram); the row the DEPTH-ZERO arm
    // builds by hand is exactly the kind a producer drifts on, and it would be the
    // one arm the check could not see. Free in release.
    // Against the ladder THIS call ranked on, never this build's constant: the
    // function is `pub` and takes the histogram, so a caller with a taller
    // ladder must not trip the assert on a row it legitimately produced.
    debug_assert!(
        out.inconsistency_against(Some(&gaps.edges_us)).is_none(),
        "absorbance_verdict produced a self-contradicting row: {:?} ({out:?})",
        out.inconsistency_against(Some(&gaps.edges_us))
    );
    out
}

/// What a tap can say about its BUDGET OCCUPANCY: whether it is over, and
/// whether that is even knowable.
///
/// # Why this is a function rather than three lines at the call site
///
/// It was three lines at the call site, and both of its answers were reachable
/// by no test. The state it exists to describe — a budgeted tap whose
/// slot width was never priced — needs a live iceoryx2 service with no attached
/// publisher, which no harness in this crate can stand up, so a variant that
/// answers `(None, false)` for everything would pass the whole suite while
/// rendering an unmeasured tap as one proven inside its budget.
///
/// Pure, so the oracle can drive the states production cannot: the
/// `plan_discovery` / `prove_prefix_loss` split this repo uses wherever a
/// decision is worth testing apart from the machinery that feeds it.
///
/// `priced_slot_bytes` is `None` exactly when nothing has ever read a live slot
/// width for this tap — which for a CEILING-deep tap is always (it is never
/// priced at all), and that is why `budget_bytes == 0` answers "no claim"
/// rather than "unpriced": such a tap carries no budget to be inside.
pub(crate) fn budget_occupancy(
    budget_bytes: u64,
    priced_slot_bytes: Option<u64>,
    tap_buffer_depth: u64,
) -> (Option<OverBudget>, bool) {
    if budget_bytes == 0 {
        // No budget: neither over one nor unable to measure one.
        return (None, false);
    }
    let Some(slot) = priced_slot_bytes else {
        return (None, true);
    };
    let capacity = slot.saturating_mul(tap_buffer_depth);
    let over = (capacity > budget_bytes).then_some(OverBudget {
        budget_bytes,
        capacity_bytes: capacity,
        widest_slot_bytes: slot,
    });
    (over, false)
}

/// The verdict body. Wrapped by [`absorbance_verdict`], which is where the
/// self-consistency assert lives so that EVERY exit is covered by it.
fn absorbance_verdict_inner(
    tap_buffer_depth: u64,
    rate: Option<DrainRateEstimate>,
    gaps: &DrainGapHistogram,
    budget_bytes: Option<u64>,
    over_budget: Option<OverBudget>,
    budget_unpriced: bool,
) -> TopicAbsorbance {
    // A rate that rounds to zero millihertz is not a rate, and it is filtered
    // BEFORE the row is built: `rate_mhz`'s own contract is that `None` means
    // UNKNOWN and never 0 Hz, so writing `Some(0)` and then refusing to divide
    // by it would put exactly the value that field forbids onto the wire.
    // Unreachable from `RateWindow` (which refuses it too) — this function is
    // `pub` and cannot assume its caller.
    let rate = rate.filter(|r| r.millihertz > 0);
    let mut out = TopicAbsorbance {
        tap_buffer_depth,
        rate_mhz: rate.map(|r| r.millihertz),
        rate_is_floor: rate.is_some_and(|r| r.is_floor),
        absorbance_us: None,
        measured_tail_us: None,
        required_depth: None,
        shortfall_at_least_us: None,
        verdict: AbsorbanceVerdict::NoClaim,
        short_evaluations: 0,
        budget_bytes,
        over_budget,
        budget_unpriced,
    };
    // No rate: nothing to compare. NOT a pass — an absent comparison must never
    // render as a healthy one.
    let Some(rate) = rate else {
        return out;
    };
    // A DEPTH-ZERO queue holds nothing, so it absorbs nothing — which is
    // provably below any measured stall, not an absence of information. It is
    // unreachable through iceoryx2 (which floors a subscriber buffer at 1) and
    // is classified rather than waved through because `NoClaim` on the one shape
    // that is certainly short is the wrong direction for a defensive arm.
    if tap_buffer_depth == 0 {
        out.absorbance_us = Some(0);
        if let Some(tail_us) = (!gaps.is_vacant())
            .then(|| gaps.quantile_us(ABSORBANCE_VERDICT_QUANTILE))
            .flatten()
        {
            out.measured_tail_us = Some(tail_us);
            out.required_depth = required_depth(rate.millihertz, tail_us, gaps);
            out.shortfall_at_least_us = Some(tail_us);
            out.verdict = AbsorbanceVerdict::Short;
        }
        return out;
    }
    // absorbance = depth / rate, in microseconds, in integers:
    //   depth / (mhz / 1000) seconds = depth * 1e9 / mhz microseconds.
    let absorbance_us = (u128::from(tap_buffer_depth) * 1_000_000_000u128
        / u128::from(rate.millihertz))
    .min(u128::from(u64::MAX)) as u64;
    out.absorbance_us = Some(absorbance_us);

    // A recorder that measured NO cadence has nothing to rank this against. It
    // is the vacant-histogram arm, and it is a NoClaim rather than an Absorbs:
    // "no measurement ran" is not "it held".
    if gaps.is_vacant() {
        return out;
    }
    // Past the vacancy check, a `None` quantile is UNAMBIGUOUSLY the unbounded
    // overflow bucket — the one signal this surface may never skip.
    match gaps.quantile_us(ABSORBANCE_VERDICT_QUANTILE) {
        Some(tail_us) => {
            out.measured_tail_us = Some(tail_us);
            out.required_depth = required_depth(rate.millihertz, tail_us, gaps);
            if absorbance_us >= tail_us {
                out.verdict = AbsorbanceVerdict::Absorbs;
            } else {
                out.verdict = AbsorbanceVerdict::Short;
                out.shortfall_at_least_us = Some(tail_us - absorbance_us);
            }
        }
        None => {
            // The tail is above the ladder's last edge. That edge is therefore a
            // LOWER BOUND on it, and a tap whose absorbance sits under the edge
            // is provably short even though the tail itself has no number.
            //
            // The comparison is `<=`, not `<`, and the boundary is REACHABLE:
            // `DrainGapHistogram::record` uses `partition_point(|e| *e < gap_us)`,
            // so a gap of exactly the last edge lands in the BOUNDED top bucket
            // and the overflow bucket holds only gaps STRICTLY GREATER than it.
            // An absorbance of exactly the edge is therefore provably below the
            // tail — and with `<` it took the Unrankable arm, which is silent by
            // design, so a demonstrably short tap went unreported. Reachable
            // wherever `depth * 1e9 / mhz == 250_000` (depth 4 at 16 Hz, depth
            // 16 at 64 Hz, depth 25 at 100 Hz, and a band around each).
            match gaps.edges_us.last().copied() {
                Some(edge) if absorbance_us <= edge => {
                    out.verdict = AbsorbanceVerdict::Short;
                    // The tail is at least `edge + 1`, so this stays a true
                    // LOWER bound at the boundary itself (where `edge -
                    // absorbance` would be 0 — true, and useless).
                    out.shortfall_at_least_us =
                        Some((edge.saturating_add(1)).saturating_sub(absorbance_us));
                }
                _ => out.verdict = AbsorbanceVerdict::Unrankable,
            }
        }
    }
    out
}

/// The depth that WOULD have absorbed `tail_us` at `millihertz` — the actionable
/// number, from the SHIPPED instrument.
///
/// `DrainGapHistogram::absorbing_depth` is the value, deliberately, so this
/// module reads the run's own cadence through the same function every other
/// consumer does. Two guards ride it:
///
/// * it takes an `f64` rate while the absorbance is exact `u128`, so the
///   equivalence the module doc claims is cross-checked in a debug build
///   against the integer form (`ceil(mhz * tail_us / 1e9)`); and
/// * it is `Option`, and a `None` here would silently drop the one number an
///   operator can act on — so the integer form is the fallback rather than the
///   absence. Unreachable today (the rate is nonzero and the quantile served a
///   value), which is exactly why it must not be a silent hole.
fn required_depth(millihertz: u64, tail_us: u64, gaps: &DrainGapHistogram) -> Option<u64> {
    let exact = (u128::from(millihertz) * u128::from(tail_us))
        .div_ceil(1_000_000_000u128)
        .min(u128::from(u64::MAX)) as u64;
    let shipped = gaps
        .absorbing_depth(millihertz as f64 / 1000.0, ABSORBANCE_VERDICT_QUANTILE)
        .map(|d| d as u64);
    debug_assert!(
        shipped.is_none_or(|d| d == exact),
        "the shipped absorbing_depth ({shipped:?}) and the exact integer form \
         ({exact}) disagree at {millihertz} mHz over a {tail_us} us tail — the \
         module's equivalence claim rests on them agreeing"
    );
    Some(shipped.unwrap_or(exact))
}

/// The per-tap rate observation window.
///
/// Fed one drained frame at a time from `Recorder::drain_taps`, off the wire
/// header that loop ALREADY parses — so it costs no receive, no parse and no
/// clock read of its own beyond one `Instant` per drive pass.
#[derive(Debug, Clone, Default)]
pub(crate) struct RateWindow {
    /// When the window's first frame was drained (recorder clock).
    anchor_at: Option<Instant>,
    /// When its most recent frame was drained.
    last_at: Option<Instant>,
    /// The most recent wire `sequence` seen, for differencing.
    last_seq: Option<u32>,
    /// Sum of FORWARD sequence deltas since the anchor — frames the publisher
    /// committed, including the ones this tap's queue dropped.
    committed: u64,
    /// Frames actually drained since the anchor (the FLOOR basis's numerator).
    frames: u64,
    /// STICKY: the sequence line cannot be differenced on this topic, so the
    /// estimate falls back to the frames floor. See [`DrainRateEstimate::is_floor`].
    floor: bool,
}

impl RateWindow {
    /// A fresh window for a topic with ONE declared writer — the ordinary case,
    /// and the only one that can mint an exact sequence-based rate.
    pub(crate) fn single_writer() -> Self {
        Self::default()
    }

    /// A fresh window for a topic the graph DECLARED `multi_publisher`.
    ///
    /// Such a topic can never mint an exact rate — `sequence` is a per-publisher
    /// counter — so the window opens on the FLOOR basis rather than discovering
    /// that at the first drain.
    pub(crate) fn multi_publisher() -> Self {
        Self {
            floor: true,
            ..Self::default()
        }
    }

    /// This topic has more than one writer, so its `sequence` is a per-publisher
    /// counter and differencing it compares two unrelated ladders.
    /// STICKY for the tap's life — a topic that was ever plural stays on the
    /// floor basis, because the frames already folded in came from two counters.
    pub(crate) fn note_multi_writer(&mut self) {
        self.floor = true;
    }

    /// Fold one drained frame in. `seq` is `None` for a headerless frame, whose
    /// fabricated `0` would otherwise read as an epoch reset on every arrival.
    pub(crate) fn observe(&mut self, at: Instant, seq: Option<u32>) {
        // A window may not SPAN A SILENCE: the liveness rate estimate's rule,
        // inherited rather than re-derived, and its absence was a
        // confidently-wrong PASS.
        //
        // SCOPE: `at` is the DRIVE-PASS instant, so this measures the
        // DRAIN clock, not the publish clock. A tap that stops being DRAINED for
        // this long — a stalled writer, a staging buffer that stays full, a long
        // discovery hold — has its window discarded on resume and mints no rate
        // for at least `ABSORBANCE_MIN_RATE_SPAN`, so its verdict reads
        // `NoClaim` (and is therefore SILENT) during exactly the episode an
        // operator most wants one. That direction is right — a rate averaged
        // across a stall is a fabrication — but it is a real cost, and the
        // `STAGING FULL` warn beside it is what carries the news meanwhile.
        //
        // A lifetime average with no cap dilutes without bound: MEASURED on a
        // transliteration, a 100 Hz topic that ran for 10 s and then went quiet
        // for an hour reported 277 mHz, at which a depth-8 tap claims it absorbs
        // 28.9 SECONDS of stall against a 25 ms measured tail — `Absorbs`, with
        // `is_floor: false`, on a tap that was demonstrably short while the
        // topic was live. It also generates a FALSE RECOVERY: a genuinely short
        // tap drifts upward as its topic idles, crosses the tail, and the loud
        // regime closes with an "absorbs again" line because the topic stopped.
        //
        // So a gap at or past `ABSORBANCE_MAX_IDLE_GAP` DISCARDS the window and
        // re-anchors here. The cost is real and is the same one the liveness
        // rate estimate pays: a topic whose PERIOD exceeds the cap re-anchors on
        // every frame and never mints a rate at all, so it reports `NoClaim`
        // rather than a diluted average. That is the deliberate trade: a slow topic
        // gets no verdict instead of a wrong one.
        if let Some(last) = self.last_at {
            if at.saturating_duration_since(last) >= ABSORBANCE_MAX_IDLE_GAP {
                self.anchor_at = None;
                self.committed = 0;
                self.frames = 0;
                self.last_seq = None;
            }
        }
        self.frames = self.frames.saturating_add(1);
        if self.anchor_at.is_none() {
            self.anchor_at = Some(at);
        }
        self.last_at = Some(at);
        let Some(s) = seq else {
            // A headerless frame is counted (it is a real frame) but can never be
            // differenced, and its presence means the sequence total is already
            // missing this frame's step — so the sequence basis is retired.
            self.floor = true;
            return;
        };
        match self.last_seq {
            None => {}
            Some(prev) => {
                let delta = s.wrapping_sub(prev);
                if delta > SEQ_BACKWARD_THRESHOLD && !self.floor {
                    // BACKWARD on a SINGLE-WRITER topic: a publisher restart,
                    // i.e. a new counter epoch. Differencing across it would
                    // fabricate a gigantic "committed" count, so the window
                    // RE-ANCHORS on the new epoch (the liveness rate estimate's
                    // discard rule) rather than trying to bridge it.
                    //
                    // The `!self.floor` guard is LOAD-BEARING and its absence was
                    // a real defect (machine-verified on a transliteration): a
                    // FLOOR window does not difference sequences at all — its
                    // numerator is `frames - 1` — while a two-writer topic's
                    // batch maximum hops between unrelated per-publisher ladders,
                    // so a backward delta fires on roughly every other frame. An
                    // unconditional re-anchor therefore reset `frames` and the
                    // anchor forever, `span` never reached
                    // `ABSORBANCE_MIN_RATE_SPAN`, and every `multi_publisher`
                    // topic — `/tf` on any real robot — carried a PERMANENT
                    // `NoClaim` on every surface. MEASURED: two interleaved
                    // 100 Hz writers, 6000 frames over 60 s, `estimate() == None`
                    // against a single-writer control reporting 100 Hz.
                    self.anchor_at = Some(at);
                    self.committed = 0;
                    self.frames = 1;
                } else if delta <= SEQ_BACKWARD_THRESHOLD {
                    // Forward, INCLUDING a large jump: a large forward delta is
                    // exactly the loss this verdict exists to price, so it is
                    // counted rather than filtered. `delta == 0` (a duplicate) is
                    // a no-op, not an epoch change.
                    self.committed = self.committed.saturating_add(u64::from(delta));
                }
            }
        }
        self.last_seq = Some(s);
    }

    /// The rate as of `now`, or `None` when this window has no basis to mint one.
    ///
    /// # Why this takes the EVALUATION instant
    ///
    /// The idle cap in [`RateWindow::observe`] can only fire when a NEXT frame
    /// arrives — and the case it exists for is precisely the one where no next
    /// frame ever comes. Without a check HERE, a topic that streamed and then
    /// stopped kept serving its last window's rate for the rest of the run: the
    /// drive loop's evaluation, the live `/bagd/status` row, and the FINAL
    /// `record_health.json` all carried a confident `Absorbs` or `Short` about a
    /// topic that has not published in hours. The liveness rate estimate's rule
    /// is explicit and is inherited rather than re-derived: a stopped stream
    /// DROPS its rate rather than decaying one.
    ///
    /// So staleness is asked at EVALUATION time, against the same
    /// `ABSORBANCE_MAX_IDLE_GAP` `observe` re-anchors on, and a stale window
    /// answers `None` — which the verdict builder turns into `NoClaim`, i.e. an
    /// UNKNOWN rather than a stale claim. The window is NOT mutated: whether a
    /// silence has ended is not knowable at evaluation time, and the next frame
    /// is what re-anchors it (`observe` runs the identical comparison).
    ///
    /// This is also why the transition cannot mint a false recovery.
    /// `report_absorbance` treats `NoClaim` as an ABSENCE of evidence — it
    /// neither closes the regime nor advances the recovery dwell — and
    /// `bag info` counts such a row under "fell short earlier and can no longer
    /// be ranked" rather than under recovered.
    pub(crate) fn estimate(&self, now: Instant) -> Option<DrainRateEstimate> {
        let (anchor, last) = (self.anchor_at?, self.last_at?);
        if now.saturating_duration_since(last) >= ABSORBANCE_MAX_IDLE_GAP {
            return None;
        }
        // On the SEQUENCE basis the numerator is the committed delta; on the
        // FLOOR basis it is the frames BETWEEN the two endpoints, which is one
        // fewer than the frames observed (N frames span N-1 intervals).
        let numerator = if self.floor {
            self.frames.saturating_sub(1)
        } else {
            self.committed
        };
        rate_from(
            numerator,
            last.saturating_duration_since(anchor),
            self.floor,
        )
    }
}

/// The rate arithmetic, split out of [`RateWindow::estimate`] so the SPAN
/// BOUNDARY, the zero refusals and the u64 saturation are oracle-testable with
/// no `Instant` at all — the `plan_discovery` / `prove_prefix_loss` split this
/// repo uses wherever a decision is worth testing apart from its clock.
///
/// `None` is the correct answer three ways: a window too short to describe, one
/// that accounts for no committed frame, and one whose rate rounds below a
/// millihertz. Each is UNKNOWN, and a fabricated 0 Hz on a plainly-streaming
/// topic is the confident-false this module exists to prevent.
pub(crate) fn rate_from(
    numerator: u64,
    span: Duration,
    is_floor: bool,
) -> Option<DrainRateEstimate> {
    if span < ABSORBANCE_MIN_RATE_SPAN || numerator == 0 {
        return None;
    }
    let span_ns = span.as_nanos().max(1);
    // millihertz = frames / seconds * 1000 = frames * 1e12 / span_ns.
    let mhz =
        (u128::from(numerator) * 1_000_000_000_000u128 / span_ns).min(u128::from(u64::MAX)) as u64;
    if mhz == 0 {
        return None;
    }
    Some(DrainRateEstimate {
        millihertz: mhz,
        is_floor,
    })
}

#[cfg(test)]
impl RateWindow {
    /// The rate AS OF THIS WINDOW'S OWN LAST FRAME — i.e. with no idle elapsed.
    ///
    /// Test-only, and deliberately not a production affordance: it is the one
    /// instant at which staleness cannot apply, so the arms that are about SPAN,
    /// BASIS and RE-ANCHORING go on asserting exactly what they asserted before
    /// the evaluation-time idle check existed, and the arms that are about
    /// staleness are the only ones that choose a `now`.
    fn estimate_at_last_frame(&self) -> Option<DrainRateEstimate> {
        self.estimate(self.last_at?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::DRAIN_GAP_BUCKET_EDGES_US;

    /// A histogram whose every gap is `gap_us`, `n` of them. Hand-built, so no
    /// assertion below is checking the recorder against itself.
    fn gaps_all(gap_us: u64, n: u64) -> DrainGapHistogram {
        let mut h = DrainGapHistogram::new();
        for _ in 0..n {
            h.record(Duration::from_micros(gap_us));
        }
        h
    }

    /// 1 Hz in millihertz, for readability at the call sites.
    const fn hz(v: u64) -> Option<DrainRateEstimate> {
        Some(DrainRateEstimate {
            millihertz: v * 1000,
            is_floor: false,
        })
    }

    /// A CEILING-deep tap's verdict (no budget), the shape most call sites want.
    fn verdict_of(
        depth: u64,
        rate: Option<DrainRateEstimate>,
        gaps: &DrainGapHistogram,
    ) -> TopicAbsorbance {
        absorbance_verdict(depth, rate, gaps, None, None, false)
    }

    #[test]
    fn a_tap_deep_enough_for_the_measured_tail_absorbs() {
        // 100 Hz, depth 4 => 40 ms of absorbance. Every measured gap is 10 ms.
        // HAND ORACLE: 40 ms >= 10 ms, and the depth that covers 10 ms at 100 Hz
        // is ceil(100 * 0.010) = 1.
        let v = verdict_of(4, hz(100), &gaps_all(10_000, 1000));
        assert_eq!(v.verdict, AbsorbanceVerdict::Absorbs);
        assert_eq!(v.absorbance_us, Some(40_000));
        assert_eq!(v.measured_tail_us, Some(10_000));
        assert_eq!(v.required_depth, Some(1));
        assert_eq!(
            v.shortfall_at_least_us, None,
            "an absorbing tap is not short"
        );
    }

    #[test]
    fn a_tap_shallower_than_the_measured_tail_is_short_and_says_by_how_much() {
        // 300 Hz, depth 2 => 6666 us. Gaps are all 10 ms => tail 10_000 us.
        // HAND ORACLE: absorbance 2 * 1e9 / 300_000 = 6666 us; shortfall
        // 10_000 - 6666 = 3334; required depth ceil(300 * 0.010) = 3.
        let v = verdict_of(2, hz(300), &gaps_all(10_000, 1000));
        assert_eq!(v.verdict, AbsorbanceVerdict::Short);
        assert_eq!(v.absorbance_us, Some(6_666));
        assert_eq!(v.measured_tail_us, Some(10_000));
        assert_eq!(v.shortfall_at_least_us, Some(3_334));
        assert_eq!(v.required_depth, Some(3));
        assert!(
            v.tap_buffer_depth < v.required_depth.unwrap(),
            "the two formulations must agree: a SHORT verdict means the depth is \
             under the required depth"
        );
    }

    /// The module's headline claim, swept over EVERY ladder edge and a wide rate
    /// range rather than one convenient pair.
    ///
    /// The two formulations cross an arithmetic boundary — `absorbance_us` is
    /// exact `u128`, `absorbing_depth` goes through `f64` — so "they can never
    /// disagree" is a claim about magnitudes. This is where it is checked.
    #[test]
    fn the_depth_comparison_and_the_time_comparison_are_the_same_verdict() {
        for &edge_us in DRAIN_GAP_BUCKET_EDGES_US {
            let gaps = gaps_all(edge_us, 500);
            for &mhz in &[1_000u64, 7_000, 100_000, 1_666_000, 10_000_000] {
                let rate = Some(DrainRateEstimate {
                    millihertz: mhz,
                    is_floor: false,
                });
                // Straddle the boundary: the depth that exactly covers the tail,
                // and one on each side of it.
                let exact = (u128::from(mhz) * u128::from(edge_us)).div_ceil(1_000_000_000u128);
                for depth in [
                    exact.saturating_sub(1).max(1) as u64,
                    exact.max(1) as u64,
                    (exact + 1) as u64,
                ] {
                    let v = verdict_of(depth, rate, &gaps);
                    let by_time = v.absorbance_us.unwrap() >= v.measured_tail_us.unwrap();
                    let by_depth = depth >= v.required_depth.unwrap();
                    assert_eq!(
                        by_time, by_depth,
                        "edge {edge_us} us, {mhz} mHz, depth {depth}: time says \
                         {by_time}, depth says {by_depth} ({v:?})"
                    );
                    assert_eq!(by_time, v.verdict == AbsorbanceVerdict::Absorbs);
                }
            }
        }
    }

    #[test]
    fn an_overflow_tail_is_rendered_never_skipped() {
        // Every gap is 5 s — far above the ladder's 250 ms top edge, so the
        // quantile lands in the unbounded overflow bucket and `quantile_us`
        // serves nothing.
        let gaps = gaps_all(5_000_000, 100);
        assert_eq!(
            gaps.quantile_us(ABSORBANCE_VERDICT_QUANTILE),
            None,
            "precondition: this fixture's tail really is in the overflow bucket"
        );

        // (a) A tap UNDER the ladder's top edge is provably short of it, and the
        // shortfall is a LOWER bound: (250_000 + 1) - 40_000 = 210_001.
        let shallow = verdict_of(4, hz(100), &gaps);
        assert_eq!(shallow.verdict, AbsorbanceVerdict::Short);
        assert_eq!(shallow.measured_tail_us, None, "the tail has no number");
        assert_eq!(shallow.shortfall_at_least_us, Some(210_001));
        assert_eq!(shallow.required_depth, None);

        // (b) A tap ABOVE the top edge cannot be ranked against an unbounded
        // tail — and that is reported, not silently passed.
        let deep = verdict_of(4096, hz(100), &gaps);
        assert_eq!(deep.verdict, AbsorbanceVerdict::Unrankable);
        assert_ne!(
            deep.verdict,
            AbsorbanceVerdict::Absorbs,
            "an unrankable pair must never render as a passing verdict"
        );
        assert!(deep
            .render_line()
            .contains("WORSE THAN THE LADDER CAN DESCRIBE"));
    }

    /// The overflow boundary is INCLUSIVE, and the boundary is reachable.
    ///
    /// `DrainGapHistogram::record` uses `partition_point(|e| *e < gap_us)`, so a
    /// gap of exactly the last edge lands in the BOUNDED top bucket — the
    /// overflow bucket holds only gaps STRICTLY GREATER. An absorbance of
    /// exactly the edge is therefore provably below an overflow tail, and with a
    /// strict `<` it took the silent `Unrankable` arm: a demonstrably short tap
    /// going unreported, contradicting this module's own claim that nothing
    /// genuinely short escapes there.
    #[test]
    fn an_absorbance_exactly_at_the_ladders_top_edge_is_short_not_unrankable() {
        let gaps = gaps_all(5_000_000, 100);
        let edge = *DRAIN_GAP_BUCKET_EDGES_US.last().unwrap();
        // depth 25 at 100 Hz => 25 * 1e9 / 100_000 = 250_000 us == the edge.
        let v = verdict_of(25, hz(100), &gaps);
        assert_eq!(
            v.absorbance_us,
            Some(edge),
            "precondition: exactly AT the edge"
        );
        assert_eq!(
            v.verdict,
            AbsorbanceVerdict::Short,
            "a tap at the top edge is below an overflow tail by construction: {v:?}"
        );
        assert_eq!(
            v.shortfall_at_least_us,
            Some(1),
            "the tail is at least edge + 1, so the bound stays TRUE (and nonzero) \
             at the boundary itself"
        );
        // One microsecond deeper and it genuinely cannot be ranked.
        let deeper = verdict_of(26, hz(100), &gaps);
        assert_eq!(deeper.verdict, AbsorbanceVerdict::Unrankable);
    }

    #[test]
    fn a_vacant_histogram_makes_no_claim_and_never_a_healthy_one() {
        let v = verdict_of(4, hz(100), &DrainGapHistogram::new());
        assert_eq!(v.verdict, AbsorbanceVerdict::NoClaim);
        assert_eq!(
            v.absorbance_us,
            Some(40_000),
            "what IS known is still reported"
        );
        assert_eq!(v.measured_tail_us, None);
        assert!(v.render_line().contains("no claim"));
    }

    #[test]
    fn a_topic_with_no_rate_makes_no_claim_and_fabricates_no_zero_hz() {
        let gaps = gaps_all(10_000, 100);
        let no_rate = verdict_of(4, None, &gaps);
        assert_eq!(no_rate.verdict, AbsorbanceVerdict::NoClaim);
        assert_eq!(no_rate.rate_mhz, None, "None is UNKNOWN, never 0 Hz");
        assert_eq!(no_rate.absorbance_us, None);

        // A rate that rounds to zero millihertz is not a rate, and the row must
        // not carry it: `rate_mhz`'s own contract is that `None` means UNKNOWN
        // and never 0 Hz, so `Some(0)` is exactly the value that field forbids.
        let zero_rate = verdict_of(
            4,
            Some(DrainRateEstimate {
                millihertz: 0,
                is_floor: false,
            }),
            &gaps,
        );
        assert_eq!(zero_rate.verdict, AbsorbanceVerdict::NoClaim);
        assert_eq!(
            zero_rate.rate_mhz, None,
            "a zero-millihertz rate is filtered BEFORE the row is built"
        );
        assert_eq!(zero_rate.absorbance_us, None);
    }

    /// A queue that holds nothing absorbs NOTHING, which is provably below any
    /// measured stall — not an absence of information.
    ///
    /// Unreachable through iceoryx2 (which floors a subscriber buffer at 1) and
    /// classified anyway: `NoClaim` on the one shape that is certainly short is
    /// the wrong direction for a defensive arm.
    #[test]
    fn a_zero_depth_queue_absorbs_nothing_and_is_short_not_silent() {
        let v = verdict_of(0, hz(100), &gaps_all(10_000, 100));
        assert_eq!(v.verdict, AbsorbanceVerdict::Short);
        assert_eq!(v.absorbance_us, Some(0));
        assert_eq!(v.shortfall_at_least_us, Some(10_000), "the whole tail");
        // ...but with nothing measured there is still nothing to compare.
        let vacant = verdict_of(0, hz(100), &DrainGapHistogram::new());
        assert_eq!(vacant.verdict, AbsorbanceVerdict::NoClaim);
    }

    #[test]
    fn the_over_budget_clause_rides_the_verdict_row() {
        let ob = OverBudget {
            budget_bytes: 4 * 1024 * 1024,
            capacity_bytes: 6 * 1024 * 1024,
            widest_slot_bytes: 1024 * 1024,
        };
        let v = absorbance_verdict(
            6,
            hz(100),
            &gaps_all(10_000, 100),
            Some(4 * 1024 * 1024),
            Some(ob),
            false,
        );
        assert_eq!(v.over_budget, Some(ob));
        let line = v.render_line();
        assert!(line.contains("OVER BUDGET"), "{line}");
        assert!(
            line.contains("6291456"),
            "the capacity must be named: {line}"
        );
        assert!(
            line.contains("4194304"),
            "…and the budget it exceeds: {line}"
        );
    }

    /// The REMEDY on a row is the one that can actually move that tap's depth.
    ///
    /// `CERULION_FLASHBACK_TAP_BUDGET_MB` reaches a BUDGETED (window-only) tap
    /// and nothing else; a ceiling-deep tap's depth is its topic's own
    /// `subscriber_buffer_size`. Naming the budget knob on a ceiling row sends
    /// an operator to a switch that does nothing — and on `cerulion bag record`,
    /// which never builds a Flashback plane, that would be every row.
    #[test]
    fn each_row_names_the_remedy_that_can_move_its_own_depth() {
        let gaps = gaps_all(10_000, 100);
        let budgeted = absorbance_verdict(2, hz(300), &gaps, Some(4 * 1024 * 1024), None, false);
        assert_eq!(budgeted.depth_mode(), "budgeted");
        assert!(
            budgeted
                .remedy()
                .contains("CERULION_FLASHBACK_TAP_BUDGET_MB"),
            "{}",
            budgeted.remedy()
        );

        let ceiling = verdict_of(2, hz(300), &gaps);
        assert_eq!(ceiling.depth_mode(), "ceiling");
        assert!(
            !ceiling
                .remedy()
                .contains("CERULION_FLASHBACK_TAP_BUDGET_MB"),
            "a ceiling-deep tap must not be sent to a knob that cannot move it: {}",
            ceiling.remedy()
        );
        assert!(
            ceiling.remedy().contains("subscriber_buffer_size"),
            "{}",
            ceiling.remedy()
        );
        // Both are SHORT, so the difference is the remedy and nothing else.
        assert_eq!(budgeted.verdict, AbsorbanceVerdict::Short);
        assert_eq!(ceiling.verdict, AbsorbanceVerdict::Short);
        assert!(ceiling.render_line().contains("fix: "));
    }

    #[test]
    fn a_floor_rate_is_labelled_because_it_makes_the_absorbance_optimistic() {
        let v = verdict_of(
            4,
            Some(DrainRateEstimate {
                millihertz: 100_000,
                is_floor: true,
            }),
            &gaps_all(10_000, 100),
        );
        assert!(v.rate_is_floor);
        assert!(v.render_line().contains("(floor)"), "{}", v.render_line());
    }

    /// A row that CONTRADICTS itself is nameable, and the producer never makes
    /// one.
    ///
    /// Every field is `pub` and the type is `Deserialize`, so a bag from an
    /// unknown robot can present a combination `absorbance_verdict` cannot
    /// produce; without a check the renderers count it by its verdict and read
    /// its numbers independently, so an `Absorbs` row carrying a shortfall is
    /// tallied as healthy with the shortfall dropped.
    #[test]
    fn a_self_contradicting_row_is_named_and_a_produced_one_never_is() {
        let sound = verdict_of(2, hz(300), &gaps_all(10_000, 100));
        assert_eq!(sound.inconsistency(), None, "the producer's own output");

        let absorbs_with_shortfall = TopicAbsorbance {
            verdict: AbsorbanceVerdict::Absorbs,
            shortfall_at_least_us: Some(1),
            ..sound
        };
        assert!(absorbs_with_shortfall
            .inconsistency()
            .is_some_and(|r| r.contains("ABSORBS")));

        let short_without_shortfall = TopicAbsorbance {
            shortfall_at_least_us: None,
            ..sound
        };
        assert!(short_without_shortfall
            .inconsistency()
            .is_some_and(|r| r.contains("SHORT")));

        let floor_without_rate = TopicAbsorbance {
            rate_mhz: None,
            rate_is_floor: true,
            ..sound
        };
        assert!(floor_without_rate
            .inconsistency()
            .is_some_and(|r| r.contains("FLOOR")));

        let no_claim_with_comparison = TopicAbsorbance {
            verdict: AbsorbanceVerdict::NoClaim,
            ..sound
        };
        assert!(no_claim_with_comparison
            .inconsistency()
            .is_some_and(|r| r.contains("NO CLAIM")));

        let unrankable_with_tail = TopicAbsorbance {
            verdict: AbsorbanceVerdict::Unrankable,
            shortfall_at_least_us: None,
            ..sound
        };
        assert!(unrankable_with_tail
            .inconsistency()
            .is_some_and(|r| r.contains("UNRANKABLE")));
    }

    /// The wire token and the rendered token are ONE word.
    ///
    /// `as_str` is hand-written beside a `rename_all = "snake_case"` derive, and
    /// nothing else pins them equal — the `MonitorCondition::as_wire` lesson.
    #[test]
    fn the_verdict_token_is_the_same_word_in_json_and_in_a_log_line() {
        for v in AbsorbanceVerdict::ALL {
            assert_eq!(
                serde_json::to_value(v).expect("serialize"),
                serde_json::Value::String(v.as_str().to_string()),
                "{v:?}"
            );
        }
        // ...and the same for the basis token, whose two surfaces are the JSON
        // document and the loud line's structured field.
        for b in [
            LossCountingBasis::PrefixProven,
            LossCountingBasis::PrefixInvisible,
        ] {
            assert_eq!(
                serde_json::to_value(b).expect("serialize"),
                serde_json::Value::String(b.as_wire().to_string()),
                "{b:?}"
            );
        }
    }

    /// Sub-millisecond values are the ORDINARY case on a healthy recorder, and a
    /// whole-millisecond rendering collapses them all to `0 ms`.
    ///
    /// The ladder's four lowest edges are 100/250/500/1000 us, and a depth-1 tap
    /// at 1.66 kHz absorbs 602 us — so before this, a genuinely short kHz row
    /// read `absorbs 0 ms vs measured stall tail 0 ms [short]`, two zeros where
    /// the whole point is a comparison.
    #[test]
    fn a_sub_millisecond_row_renders_two_distinguishable_numbers() {
        let v = verdict_of(1, hz(1_660), &gaps_all(250, 500));
        let line = v.render_line();
        assert!(
            line.contains("0.60 ms"),
            "the absorbance must be legible: {line}"
        );
        assert!(line.contains("0.25 ms"), "…and so must the tail: {line}");
        assert_ne!(
            v.absorbance_us, v.measured_tail_us,
            "precondition: the two numbers really do differ"
        );
    }

    // ---- the rate arithmetic, with no clock ----

    /// The span boundary, pinned on BOTH sides at exactly the minimum.
    #[test]
    fn the_rate_window_span_is_a_threshold_pinned_on_both_sides() {
        let min = ABSORBANCE_MIN_RATE_SPAN;
        assert_eq!(
            rate_from(100, min - Duration::from_nanos(1), false),
            None,
            "one nanosecond short of the minimum mints nothing"
        );
        assert!(
            rate_from(100, min, false).is_some(),
            "and AT the minimum it mints"
        );
    }

    #[test]
    fn the_rate_arithmetic_refuses_what_it_cannot_honestly_answer() {
        let span = Duration::from_secs(10);
        assert_eq!(rate_from(0, span, false), None, "no committed frame");
        // 1 frame in 10 s = 100 mHz.
        assert_eq!(rate_from(1, span, false).map(|r| r.millihertz), Some(100));
        // Slower than a millihertz rounds to zero, which is UNKNOWN, not 0 Hz.
        assert_eq!(rate_from(1, Duration::from_secs(2000), false), None);
        // The floor flag is carried through untouched.
        assert!(rate_from(1, span, true).unwrap().is_floor);
        // And an absurd numerator saturates rather than wrapping.
        assert!(rate_from(u64::MAX, span, false).unwrap().millihertz > 0);
    }

    // ---- RateWindow ----

    /// Drive a window with `n` frames `period` apart, sequences advancing by
    /// `step` (1 = nothing lost; 10 = nine in ten frames dropped).
    fn drive(n: u64, period: Duration, step: u32) -> RateWindow {
        let mut w = RateWindow::single_writer();
        let t0 = Instant::now();
        let mut seq: u32 = 0;
        for i in 0..n {
            w.observe(t0 + period * (i as u32), Some(seq));
            seq = seq.wrapping_add(step);
        }
        w
    }

    #[test]
    fn the_rate_counts_frames_the_queue_dropped_not_frames_we_caught() {
        // THE headline. 100 frames drained 100 ms apart = 10 Hz of DELIVERY, but
        // the sequences advance by 10 each time, so the publisher committed 990
        // frames over 9.9 s = 100 Hz. A frames-counting rate would answer 10 Hz —
        // exactly the "absorbs" reading that hides the loss.
        let w = drive(100, Duration::from_millis(100), 10);
        let est = w
            .estimate_at_last_frame()
            .expect("a 9.9 s window mints a rate");
        assert_eq!(est.millihertz, 100_000, "990 commits / 9.9 s = 100 Hz");
        assert!(!est.is_floor);

        // The frames-basis answer, for contrast — driven through the SAME window
        // by declaring plurality, which is the only thing that changes.
        let mut floored = drive(100, Duration::from_millis(100), 10);
        floored.note_multi_writer();
        let f = floored.estimate_at_last_frame().expect("rate");
        assert_eq!(f.millihertz, 10_000, "99 frames / 9.9 s = 10 Hz");
        assert!(f.is_floor);
        assert!(
            f.millihertz < est.millihertz,
            "the floor basis must UNDER-report, which is what makes it a floor"
        );
    }

    /// A MULTI-WRITER topic still gets a rate, and this is the arm that says so.
    ///
    /// `sequence` is a per-publisher counter, so two writers make the batch
    /// maximum hop between unrelated ladders and a backward delta fires on
    /// roughly every other frame. An epoch re-anchor that did not check the
    /// basis therefore reset the FLOOR window's frames and anchor forever, the
    /// span never reached the minimum, and every `multi_publisher` topic — `/tf`
    /// on any real robot — carried a PERMANENT `NoClaim` on every surface.
    #[test]
    fn a_two_writer_topic_still_mints_a_floor_rate_rather_than_a_permanent_no_claim() {
        let mut w = RateWindow::multi_publisher();
        let t0 = Instant::now();
        // Two 50 Hz writers interleaved on one topic, 10 s, their counters far
        // apart — so every other observation is a large backward hop.
        for i in 0..1000u32 {
            let seq = if i % 2 == 0 { i / 2 } else { 900_000 + i / 2 };
            w.observe(t0 + Duration::from_millis(10) * i, Some(seq));
        }
        let est = w
            .estimate_at_last_frame()
            .expect("a two-writer topic must still report a floor rate");
        assert!(est.is_floor, "and it must be LABELLED a floor: {est:?}");
        // HAND ORACLE: 999 intervals over 9.99 s = 100 Hz of delivery.
        assert_eq!(est.millihertz, 100_000, "{est:?}");
    }

    /// A window may not span a SILENCE.
    ///
    /// Without the cap a lifetime average dilutes without bound: a 100 Hz topic
    /// that ran 10 s and then went quiet for an hour reported 277 mHz, at which
    /// a depth-8 tap claims it absorbs 28.9 SECONDS against a 25 ms tail —
    /// `Absorbs`, un-floored, on a tap that was demonstrably short while the
    /// topic was live.
    #[test]
    fn a_window_that_spans_a_silence_is_discarded_not_served_as_a_diluted_average() {
        let t0 = Instant::now();
        let mut w = RateWindow::single_writer();
        // 100 Hz for 10 s.
        for i in 0..1000u32 {
            w.observe(t0 + Duration::from_millis(10) * i, Some(i));
        }
        let live = w.estimate_at_last_frame().expect("rate");
        assert_eq!(live.millihertz, 100_000, "the live regime, measured");

        // ...then an hour of silence and one frame.
        let after = t0 + Duration::from_secs(3610);
        w.observe(after, Some(1000));
        assert_eq!(
            w.estimate_at_last_frame(),
            None,
            "the window is DISCARDED, not diluted — a single frame after the \
             silence spans no time and mints nothing"
        );

        // And the new regime is measured on its own terms.
        for i in 1..1000u32 {
            w.observe(after + Duration::from_millis(10) * i, Some(1000 + i));
        }
        assert_eq!(
            w.estimate_at_last_frame().expect("rate").millihertz,
            100_000,
            "the post-silence regime is measured, not averaged with the pause"
        );
    }

    /// The cost of that rule, stated as a test rather than left to be
    /// rediscovered: a topic slower than the cap never mints a rate at all.
    #[test]
    fn a_topic_slower_than_the_idle_cap_reports_no_rate_rather_than_a_wrong_one() {
        let t0 = Instant::now();
        let mut w = RateWindow::single_writer();
        let period = ABSORBANCE_MAX_IDLE_GAP + Duration::from_secs(1);
        for i in 0..20u32 {
            w.observe(t0 + period * i, Some(i));
        }
        assert_eq!(
            w.estimate_at_last_frame(),
            None,
            "every frame re-anchors, so no window can close — NoClaim rather \
             than a diluted average"
        );
    }

    #[test]
    fn a_window_shorter_than_the_minimum_span_mints_nothing() {
        let w = drive(10, Duration::from_millis(50), 1); // 450 ms span
        assert_eq!(w.estimate_at_last_frame(), None);
        // …and one frame past the minimum, it does.
        let w = drive(30, Duration::from_millis(50), 1); // 1.45 s span
        assert!(w.estimate_at_last_frame().is_some());
    }

    #[test]
    fn a_single_frame_and_an_empty_window_mint_nothing() {
        assert_eq!(RateWindow::single_writer().estimate_at_last_frame(), None);
        let mut w = RateWindow::single_writer();
        w.observe(Instant::now(), Some(0));
        assert_eq!(w.estimate_at_last_frame(), None, "one frame spans no time");
    }

    #[test]
    fn a_headerless_frame_retires_the_sequence_basis() {
        let mut w = RateWindow::single_writer();
        let t0 = Instant::now();
        for i in 0..30u32 {
            w.observe(t0 + Duration::from_millis(100) * i, Some(i * 10));
        }
        assert!(
            !w.estimate_at_last_frame().unwrap().is_floor,
            "control: still sequence-based"
        );
        w.observe(t0 + Duration::from_millis(3000), None);
        assert!(
            w.estimate_at_last_frame().unwrap().is_floor,
            "a fabricated seq 0 cannot be differenced, so the basis must retire"
        );
    }

    #[test]
    fn a_publisher_restart_re_anchors_instead_of_fabricating_a_giant_commit_count() {
        let mut w = RateWindow::single_writer();
        let t0 = Instant::now();
        // Epoch 1: 20 frames at 10 Hz, sequences 1_000_000..
        for i in 0..20u32 {
            w.observe(t0 + Duration::from_millis(100) * i, Some(1_000_000 + i));
        }
        // Epoch 2: the publisher restarts at 0 and runs 30 frames at 10 Hz.
        for i in 0..30u32 {
            w.observe(
                t0 + Duration::from_millis(2_000 + 100 * u64::from(i)),
                Some(i),
            );
        }
        let est = w.estimate_at_last_frame().expect("rate");
        // HAND ORACLE: bridging the epochs would count ~4.29e9 commits. The
        // re-anchored window sees 29 commits over 2.9 s = 10 Hz.
        assert_eq!(est.millihertz, 10_000, "{est:?}");
    }

    #[test]
    fn a_duplicate_sequence_is_a_no_op_not_an_epoch_change() {
        let mut w = RateWindow::single_writer();
        let t0 = Instant::now();
        for i in 0..20u32 {
            w.observe(t0 + Duration::from_millis(100) * i, Some(i));
        }
        // Same sequence again, 100 ms later.
        w.observe(t0 + Duration::from_millis(2_000), Some(19));
        let est = w.estimate_at_last_frame().expect("rate");
        // 19 commits over 2.0 s — the duplicate advanced the clock but not the
        // commit count, and did NOT reset the anchor.
        assert_eq!(est.millihertz, 9_500, "{est:?}");
    }

    #[test]
    fn a_u32_wrap_is_ordinary_forward_motion() {
        let mut w = RateWindow::single_writer();
        let t0 = Instant::now();
        let mut seq = u32::MAX - 10;
        for i in 0..30u32 {
            w.observe(t0 + Duration::from_millis(100) * i, Some(seq));
            seq = seq.wrapping_add(1);
        }
        let est = w.estimate_at_last_frame().expect("rate");
        assert_eq!(
            est.millihertz, 10_000,
            "wrapping through u32::MAX is +1, not a 4-billion-frame loss: {est:?}"
        );
    }

    /// A BUDGETED tap nobody could price says UNKNOWN, not "inside its budget".
    ///
    /// `over_budget: None` alone cannot carry that: it is minted from
    /// `priced_slot_bytes.and_then(..)`, which short-circuits when nothing was
    /// ever priced — so a tap PROVEN inside its budget and a tap nobody could
    /// measure render byte-identically, and the second reads as the first. This
    /// is the "a zero means nothing was COUNTED" rule the health document
    /// already applies to `shm_pinned_bytes`.
    #[test]
    fn an_unpriced_budgeted_tap_says_unknown_rather_than_looking_healthy() {
        let gaps = gaps_all(10_000, 100);
        let unpriced = absorbance_verdict(6, hz(100), &gaps, Some(4 * 1024 * 1024), None, true);
        assert!(unpriced.budget_unpriced);
        assert_eq!(unpriced.over_budget, None);
        let line = unpriced.render_line();
        assert!(line.contains("UNPRICED"), "{line}");
        assert!(line.contains("unknown"), "{line}");

        // THE CONTRAST, in the same body: a tap that WAS priced and is inside
        // its budget says nothing at all — which is why the line above has to
        // exist, and is the anti-tautology half (a renderer that printed the
        // clause unconditionally passes the assertions above).
        let priced = absorbance_verdict(6, hz(100), &gaps, Some(4 * 1024 * 1024), None, false);
        let line = priced.render_line();
        assert!(!line.contains("UNPRICED"), "{line}");
        assert!(!line.contains("OVER BUDGET"), "{line}");
    }

    /// The two budget facts cannot both hold, and the producer's own
    /// self-check says so.
    #[test]
    fn an_unpriced_row_that_also_reports_an_occupancy_is_self_contradicting() {
        let gaps = gaps_all(10_000, 100);
        let ob = OverBudget {
            budget_bytes: 4 * 1024 * 1024,
            capacity_bytes: 6 * 1024 * 1024,
            widest_slot_bytes: 1024 * 1024,
        };
        let sound = absorbance_verdict(6, hz(100), &gaps, Some(4 * 1024 * 1024), Some(ob), false);
        assert_eq!(sound.inconsistency(), None, "the producer's own output");

        // Hand-built contradictions — a WRITER that drifted from the rules.
        let both = TopicAbsorbance {
            budget_unpriced: true,
            ..sound
        };
        assert!(
            both.inconsistency().is_some_and(|m| m.contains("UNPRICED")),
            "{:?}",
            both.inconsistency()
        );
        let no_budget = TopicAbsorbance {
            budget_unpriced: true,
            budget_bytes: None,
            over_budget: None,
            ..sound
        };
        assert!(
            no_budget
                .inconsistency()
                .is_some_and(|m| m.contains("no budget")),
            "{:?}",
            no_budget.inconsistency()
        );
    }

    /// The three things a tap can say about its budget occupancy, against a HAND
    /// oracle — including the two the harnesses cannot reach.
    #[test]
    fn budget_occupancy_separates_over_inside_and_unknowable() {
        const BUDGET: u64 = 140_000;
        // An arbitrary slot size: `budget_occupancy` is arithmetic over whatever
        // it is handed, and pinning it to the live iceoryx2 slot would make this
        // oracle move every time that header does.
        const SLOT: u64 = 65_576;

        // (1) OVER: 3 x 65_576 = 196_728 > 140_000. Hand-computed.
        let (over, unpriced) = budget_occupancy(BUDGET, Some(SLOT), 3);
        assert!(!unpriced);
        assert_eq!(
            over,
            Some(OverBudget {
                budget_bytes: BUDGET,
                capacity_bytes: 196_728,
                widest_slot_bytes: SLOT,
            })
        );

        // (2) INSIDE, and PROVEN so: 2 x 65_576 = 131_152 <= 140_000.
        assert_eq!(budget_occupancy(BUDGET, Some(SLOT), 2), (None, false));
        // The BOUNDARY, both sides — `capacity > budget` is strict, so a tap
        // exactly at its budget is inside it.
        assert_eq!(budget_occupancy(131_152, Some(SLOT), 2), (None, false));
        assert!(budget_occupancy(131_151, Some(SLOT), 2).0.is_some());

        // (3) UNKNOWABLE: budgeted, never priced. THE state no harness can
        // stand up, and the one a variant answering `(None, false)` renders as (2).
        assert_eq!(budget_occupancy(BUDGET, None, 2), (None, true));

        // (4) NO BUDGET (a ceiling-deep `--record` tap): no claim either way,
        // whether or not anything was priced. Not "unpriced" — there is no
        // budget for it to be unpriced against.
        assert_eq!(budget_occupancy(0, None, 2), (None, false));
        assert_eq!(budget_occupancy(0, Some(SLOT), 4096), (None, false));
    }

    /// A window that has gone QUIET past the cap mints no rate AT EVALUATION
    /// TIME — the idle check cannot wait for a frame that never comes.
    ///
    /// `observe`'s own cap only fires when a NEXT frame arrives, and the case it
    /// exists for is precisely the one where none does. Without this check a
    /// topic that streamed and then stopped kept serving its last window's rate
    /// for the rest of the run, so the live `/bagd/status` row and the FINAL
    /// `record_health.json` both carried a confident `Absorbs` or `Short` about
    /// a topic that had not published in hours. The rate rule: a stopped stream DROPS
    /// its rate rather than decaying one.
    #[test]
    fn a_window_gone_quiet_past_the_cap_mints_no_rate_at_evaluation_time() {
        let t0 = Instant::now();
        let mut w = RateWindow::single_writer();
        // 200 frames over 2 s => 100 Hz, hand-computed.
        for i in 0..200u32 {
            w.observe(t0 + Duration::from_millis(u64::from(i) * 10), Some(i));
        }
        let last = t0 + Duration::from_millis(1_990);

        // PRECONDITION: the window really does mint a rate while it is live, or
        // every `None` below is satisfied by a window that never had one.
        let live = w.estimate(last).expect("a live 100 Hz window mints a rate");
        assert_eq!(live.millihertz, 100_000, "hand oracle: 199 frames / 1.99 s");

        // THE THRESHOLD, both sides — and the window is NOT mutated by asking,
        // so the two questions are independent.
        assert!(
            w.estimate(last + ABSORBANCE_MAX_IDLE_GAP - Duration::from_millis(1))
                .is_some(),
            "one millisecond inside the cap is still live"
        );
        assert_eq!(
            w.estimate(last + ABSORBANCE_MAX_IDLE_GAP),
            None,
            "AT the cap the window is stale — the boundary is reachable"
        );
        assert_eq!(
            w.estimate(last + ABSORBANCE_MAX_IDLE_GAP * 1000),
            None,
            "and it does not come back on its own"
        );
        // Asking did not consume the window: a frame arriving later re-anchors
        // it through `observe`, which runs the identical comparison.
        assert!(
            w.estimate(last).is_some(),
            "the query is READ-ONLY — a stale reading must not destroy the window"
        );
    }

    /// …and the verdict built from a stale window is `NoClaim`, not a stale
    /// `Absorbs`.
    ///
    /// The end-to-end statement of the same defect, one layer up: what an
    /// operator sees is not the rate, it is the verdict, and a `NoClaim` is what
    /// makes the row say "nothing to compare" instead of repeating a claim about
    /// a topic that stopped.
    #[test]
    fn the_verdict_built_from_a_stale_window_makes_no_claim() {
        let t0 = Instant::now();
        let mut w = RateWindow::single_writer();
        for i in 0..200u32 {
            w.observe(t0 + Duration::from_millis(u64::from(i) * 10), Some(i));
        }
        let last = t0 + Duration::from_millis(1_990);
        let gaps = gaps_all(10_000, 100);

        // LIVE: a depth-4 tap at 100 Hz holds 40 ms against a 10 ms tail.
        let live = absorbance_verdict(4, w.estimate(last), &gaps, None, None, false);
        assert_eq!(live.verdict, AbsorbanceVerdict::Absorbs, "{live:?}");
        assert_eq!(live.rate_mhz, Some(100_000));

        // QUIET: the SAME tap, the SAME histogram, evaluated after the cap.
        let stale = absorbance_verdict(
            4,
            w.estimate(last + ABSORBANCE_MAX_IDLE_GAP),
            &gaps,
            None,
            None,
            false,
        );
        assert_eq!(
            stale.verdict,
            AbsorbanceVerdict::NoClaim,
            "a topic that stopped publishing has no verdict, not its old one: {stale:?}"
        );
        assert_eq!(stale.rate_mhz, None);
        assert_eq!(stale.absorbance_us, None);
        assert_eq!(stale.measured_tail_us, None, "and nothing to compare it to");
    }

    /// A row whose VERDICT and whose NUMBERS disagree is named, in both
    /// directions, by the arithmetic rather than by the verdict alone.
    ///
    /// This is the reader's whole defence: `bag info` decodes these rows from a
    /// bag written by an unknown robot, so `verdict` and the numbers beside it
    /// are two independent claims and a reader may not assume they agree.
    #[test]
    fn a_verdict_that_contradicts_its_own_arithmetic_is_named() {
        let gaps = gaps_all(10_000, 100);
        let absorbs = absorbance_verdict(4, hz(100), &gaps, None, None, false);
        assert_eq!(absorbs.verdict, AbsorbanceVerdict::Absorbs);
        assert_eq!(absorbs.inconsistency(), None, "the producer's own output");

        // ABSORBS while reporting less than the tail it names.
        let impossible = TopicAbsorbance {
            absorbance_us: Some(1_000),
            ..absorbs
        };
        assert!(
            impossible
                .inconsistency()
                .is_some_and(|m| m.contains("under the tail")),
            "{:?}",
            impossible.inconsistency()
        );

        // SHORT while its absorbance meets the tail.
        let short = absorbance_verdict(1, hz(1_000), &gaps, None, None, false);
        assert_eq!(short.verdict, AbsorbanceVerdict::Short);
        assert_eq!(short.inconsistency(), None);
        let impossible = TopicAbsorbance {
            absorbance_us: Some(50_000),
            ..short
        };
        assert!(
            impossible
                .inconsistency()
                .is_some_and(|m| m.contains("meets the tail")),
            "{:?}",
            impossible.inconsistency()
        );

        // …but a SHORT row with NO tail is SOUND: the ladder-overflow arm knows
        // only that the tail exceeds the last edge, so there is no number to
        // compare against and this check must not fire on it. (Note: the
        // producer's tail-less claim is the DERIVED bound `edge + 1 -
        // absorbance`, so a faithful fixture carries it — a stale claim here
        // would now be convicted by the derived-bound rule instead of proving
        // the served-tail check stays quiet. Also, dropping the tail drops
        // the REQUIRED DEPTH with it — the overflow arm has no tail to require a
        // depth for and never writes one, so carrying the served row's over
        // would be convicted by that rule instead.)
        let last_edge = *DRAIN_GAP_BUCKET_EDGES_US.last().unwrap();
        let overflow = TopicAbsorbance {
            measured_tail_us: None,
            required_depth: None,
            shortfall_at_least_us: Some(
                last_edge
                    .saturating_add(1)
                    .saturating_sub(short.absorbance_us.unwrap()),
            ),
            ..short
        };
        assert_eq!(
            overflow.inconsistency(),
            None,
            "an overflow tail is unknown, not contradictory"
        );
    }

    /// A verdict is DERIVED from a rate, so a row that reaches
    /// one without reporting a rate is not producible.
    ///
    /// `absorbance_verdict_inner` filters an absent-or-zero rate first and
    /// returns `NoClaim` immediately — the absorbance is `depth / rate`, so with
    /// no denominator there is nothing to compare. Every arithmetic arm in
    /// `inconsistency_against` is nonetheless SATISFIED by such a row (they
    /// compare absorbance against the tail and never ask where the absorbance
    /// came from), which is why `bag info` aggregated it and rendered
    /// `rate unknown -> absorbs`.
    ///
    /// Driven as a 2x2 over (verdict class) x (rate present), because the rule
    /// is ONE-DIRECTIONAL and a symmetric implementation would be wrong in a way
    /// no single-cell test can see.
    #[test]
    fn a_verdict_without_the_rate_it_was_derived_from_is_not_producible() {
        let gaps = gaps_all(10_000, 100);

        // The PRODUCER's own rows, both cells of the sound column.
        let ranked = verdict_of(4, hz(100), &gaps);
        assert_eq!(ranked.verdict, AbsorbanceVerdict::Absorbs);
        assert!(ranked.rate_mhz.is_some());
        assert_eq!(ranked.inconsistency(), None, "the producer's own output");

        // NoClaim WITH a rate is producible and SOUND — the vacant-histogram arm
        // computes the absorbance, then declines to rank it. This is the cell a
        // rule written as "rate present iff verdict is not NoClaim" gets wrong.
        let vacant = verdict_of(4, hz(100), &DrainGapHistogram::new());
        assert_eq!(vacant.verdict, AbsorbanceVerdict::NoClaim);
        assert_eq!(
            (vacant.rate_mhz, vacant.absorbance_us),
            (Some(100_000), Some(40_000)),
            "precondition: this IS the NoClaim-carrying-a-rate shape"
        );
        assert_eq!(
            vacant.inconsistency(),
            None,
            "a NoClaim row may carry the rate it declined to rank"
        );

        // NoClaim WITHOUT a rate: the rate-absent early return. Sound.
        let rateless = verdict_of(4, None, &gaps);
        assert_eq!(
            (rateless.verdict, rateless.rate_mhz),
            (AbsorbanceVerdict::NoClaim, None)
        );
        assert_eq!(rateless.inconsistency(), None);

        // …and the UNPRODUCIBLE cell, once per non-NoClaim verdict, because each
        // reaches the arithmetic arms by a different path and a check placed
        // inside any one arm would miss the other two.
        for verdict in [
            AbsorbanceVerdict::Absorbs,
            AbsorbanceVerdict::Short,
            AbsorbanceVerdict::Unrankable,
        ] {
            let row = TopicAbsorbance {
                verdict,
                rate_mhz: None,
                rate_is_floor: false,
                // Arithmetically self-consistent for whichever verdict it
                // claims, so ONLY the missing rate can convict it — without
                // this the test would pass on the sibling rules.
                measured_tail_us: match verdict {
                    AbsorbanceVerdict::Absorbs => Some(10_000),
                    AbsorbanceVerdict::Short => Some(50_000),
                    _ => None,
                },
                shortfall_at_least_us: match verdict {
                    AbsorbanceVerdict::Short => Some(10_000),
                    _ => None,
                },
                ..ranked
            };
            assert!(
                row.inconsistency()
                    .is_some_and(|m| m.contains("without reporting the rate")),
                "{verdict:?}: {:?}",
                row.inconsistency()
            );
        }
    }

    /// A rate of ZERO is not a rate either, so a verdict
    /// derived from one is as unproducible as a verdict derived from none.
    ///
    /// The sibling above rejected `None` only, and `absorbance_verdict_inner`
    /// filters `millihertz > 0` for the SAME reason — so `Some(0)` is otherwise a hole a
    /// decoded row walks through with every arithmetic arm satisfied: `bag info`
    /// counts a forged `Absorbs` as an absorbing topic while `render_line` prints
    /// it `0.000 Hz`.
    ///
    /// Driven over BOTH gates the zero can reach (the FLOOR label, which is
    /// verdict-independent, and the verdict-derivation rule) and over all three
    /// non-`NoClaim` verdicts, since each reaches the arithmetic by its own path.
    #[test]
    fn a_verdict_derived_from_a_zero_rate_is_not_producible() {
        let gaps = gaps_all(10_000, 100);
        let ranked = verdict_of(4, hz(100), &gaps);
        assert_eq!(ranked.verdict, AbsorbanceVerdict::Absorbs);

        // THE PRODUCER, handed exactly this rate: it emits `NoClaim` carrying NO
        // rate at all. That is the precondition the rule rests on — a zero never
        // survives onto a row — so a decoded row carrying one came from
        // somewhere else.
        let zeroed = verdict_of(
            4,
            Some(DrainRateEstimate {
                millihertz: 0,
                is_floor: true,
            }),
            &gaps,
        );
        assert_eq!(
            (zeroed.verdict, zeroed.rate_mhz, zeroed.rate_is_floor),
            (AbsorbanceVerdict::NoClaim, None, false),
            "precondition: a zero rate is filtered BEFORE the row is built, floor label included"
        );
        assert_eq!(zeroed.inconsistency(), None, "the producer's own output");

        for verdict in [
            AbsorbanceVerdict::Absorbs,
            AbsorbanceVerdict::Short,
            AbsorbanceVerdict::Unrankable,
        ] {
            let row = TopicAbsorbance {
                verdict,
                rate_mhz: Some(0),
                rate_is_floor: false,
                // Arithmetically self-consistent for whichever verdict it
                // claims, so ONLY the zero rate can convict it.
                measured_tail_us: match verdict {
                    AbsorbanceVerdict::Absorbs => Some(10_000),
                    AbsorbanceVerdict::Short => Some(50_000),
                    _ => None,
                },
                shortfall_at_least_us: match verdict {
                    AbsorbanceVerdict::Short => Some(10_000),
                    _ => None,
                },
                ..ranked
            };
            assert!(
                row.inconsistency()
                    .is_some_and(|m| m.contains("rate of ZERO")),
                "{verdict:?}: {:?}",
                row.inconsistency()
            );
        }

        // The FLOOR gate is verdict-INDEPENDENT: a label over a zero is
        // unproducible whatever the row claims, and it convicts before the
        // derivation rule can.
        let floored = TopicAbsorbance {
            rate_mhz: Some(0),
            rate_is_floor: true,
            ..ranked
        };
        assert!(
            floored
                .inconsistency()
                .is_some_and(|m| m.contains("the rate it labels is ZERO")),
            "{:?}",
            floored.inconsistency()
        );
        // …and it is the ZERO that convicts, not the label: the same row on a
        // positive rate is sound (anti-tautology).
        assert_eq!(
            TopicAbsorbance {
                rate_is_floor: true,
                ..ranked
            }
            .inconsistency(),
            None
        );
    }

    /// Where a tail is SERVED, the shortfall IS the gap.
    ///
    /// Both producer arms that emit a tail-carrying `Short` row emit exactly
    /// `tail - absorbance`: the ordinary arm computes that difference, and the
    /// DEPTH-ZERO arm reports `tail` against an absorbance of 0, which is the
    /// same subtraction. Pinned on BOTH sides of the exact value, since the two
    /// directions are wrong in different ways — above the gap is FALSE, below it
    /// UNDER-REPORTS SEVERITY — and a `<=` rule would admit the second.
    ///
    /// The tail-LESS overflow row, where `at_least` is a genuine conservative
    /// bound, is the control: this rule must not touch it.
    #[test]
    fn a_served_tail_makes_the_shortfall_exact_on_both_sides() {
        let gaps = gaps_all(50_000, 100);
        // depth 1 at 1000 Hz => 1 ms of absorbance against a 50 ms tail.
        let short = verdict_of(1, hz(1_000), &gaps);
        assert_eq!(
            (
                short.verdict,
                short.absorbance_us,
                short.measured_tail_us,
                short.shortfall_at_least_us
            ),
            (
                AbsorbanceVerdict::Short,
                Some(1_000),
                Some(50_000),
                Some(49_000)
            ),
            "precondition: the producer emits the gap EXACTLY"
        );
        assert_eq!(short.inconsistency(), None, "the producer's own output");

        // ONE MICROSECOND either side. Neither is producible.
        for claimed in [48_999u64, 49_001] {
            let row = TopicAbsorbance {
                shortfall_at_least_us: Some(claimed),
                ..short
            };
            assert!(
                row.inconsistency()
                    .is_some_and(|m| m.contains("is not the gap")),
                "{claimed}: {:?}",
                row.inconsistency()
            );
        }

        // The shape: a magnitude small enough to read as a
        // rounding error on a row describing a 49 ms miss.
        let fabricated = TopicAbsorbance {
            shortfall_at_least_us: Some(1),
            ..short
        };
        assert!(fabricated.inconsistency().is_some(), "{fabricated:?}");

        // The DEPTH-ZERO arm reaches the same rule by the other route
        // (`shortfall = tail` against `absorbance = 0`) and must stay sound.
        let depth_zero = verdict_of(0, hz(1_000), &gaps);
        assert_eq!(
            (
                depth_zero.verdict,
                depth_zero.absorbance_us,
                depth_zero.shortfall_at_least_us
            ),
            (AbsorbanceVerdict::Short, Some(0), Some(50_000)),
            "precondition: the depth-zero shape"
        );
        assert_eq!(depth_zero.inconsistency(), None);

        // THE CONTROL: the tail-LESS overflow row is where `at_least` is a real
        // conservative bound, and this rule may not reach it.
        let overflow = verdict_of(25, hz(100), &gaps_all(5_000_000, 100));
        assert_eq!(
            (overflow.verdict, overflow.measured_tail_us),
            (AbsorbanceVerdict::Short, None),
            "precondition: the ladder-overflow shape"
        );
        assert_eq!(
            overflow.inconsistency(),
            None,
            "a conservative bound on an unmeasured tail is exactly what `at_least` means"
        );
    }

    /// A tail-less `Short` row is bounded by the LADDER, on
    /// both sides of the edge.
    ///
    /// The sibling test above pins that such a row is not contradictory *per
    /// se*, and that is right — but it is only sound BELOW the edge. The
    /// overflow arm exists because a tail above the ladder's last edge has no
    /// number, and the producer can bound it from below only while the
    /// absorbance sits at most AT that edge (`absorbance_us <= edge`); above it
    /// the pair is `Unrankable` and the producer says nothing. So a decoded row
    /// reading `Short` with no tail and an absorbance above the edge describes a
    /// comparison nobody made — and before this arm existed both renderers
    /// counted it as a genuine shortfall and printed its invented magnitude.
    ///
    /// Driven at the boundary itself rather than at a comfortable distance from
    /// it, because a check written with `>=` instead of `>` would reject the one
    /// row the producer really does emit there.
    ///
    /// A tail-less `Short` row's claimed bound
    /// is the LADDER-DERIVED one, exactly — `edge + 1 - absorbance`, which is
    /// what the overflow arm emits ("the tail is at least `edge + 1`") — pinned
    /// on BOTH sides of the value, with the ladder parameter load-bearing.
    ///
    /// The sibling exactness rule (served tail) pins `tail - absorbance`; this
    /// arm is the tail-LESS analogue the served-tail fix deliberately did not
    /// touch, and the shape it catches is the under-report: a fabricated
    /// `shortfall_at_least_us` far below what the row's own edge and absorbance
    /// establish, rendered as a rounding error on a tap that is off the scale.
    #[test]
    fn a_tail_less_short_rows_claimed_bound_is_the_ladder_derived_one_exactly() {
        let edge = *DRAIN_GAP_BUCKET_EDGES_US.last().unwrap();
        // The producer's own overflow row at the edge: absorbance == edge, so
        // its derived bound is exactly `edge + 1 - edge = 1`.
        let at_edge = verdict_of(25, hz(100), &gaps_all(5_000_000, 100));
        assert_eq!(
            (
                at_edge.verdict,
                at_edge.absorbance_us,
                at_edge.measured_tail_us,
                at_edge.shortfall_at_least_us
            ),
            (AbsorbanceVerdict::Short, Some(edge), None, Some(1)),
            "precondition: the producer's own claim at the edge is exactly 1"
        );
        assert_eq!(
            at_edge.inconsistency(),
            None,
            "the row the producer really emits must stay SOUND"
        );

        // BELOW the derived bound — the shape that matters, the direction that
        // under-reports severity. The producer cannot emit it.
        let under = TopicAbsorbance {
            shortfall_at_least_us: Some(0),
            ..at_edge
        };
        assert!(
            under
                .inconsistency()
                .is_some_and(|m| m.contains("ladder-derived bound")),
            "{:?}",
            under.inconsistency()
        );

        // ABOVE it — a floor the row's own numbers do not establish.
        let over = TopicAbsorbance {
            shortfall_at_least_us: Some(2),
            ..at_edge
        };
        assert!(
            over.inconsistency()
                .is_some_and(|m| m.contains("ladder-derived bound")),
            "{:?}",
            over.inconsistency()
        );

        // THE LADDER IS A PARAMETER, both directions: a row ranked on a TALLER
        // ladder claims THAT ladder's derived bound and is sound on its own
        // terms — and the same row judged on the default vocabulary is flagged,
        // loudly, with the numbers a reader needs to see which ladder it meant.
        let taller: Vec<u64> = DRAIN_GAP_BUCKET_EDGES_US
            .iter()
            .copied()
            .chain(std::iter::once(edge * 4))
            .collect();
        let foreign = TopicAbsorbance {
            shortfall_at_least_us: Some((edge * 4) + 1 - edge),
            ..at_edge
        };
        assert_eq!(
            foreign.inconsistency_against(Some(&taller)),
            None,
            "sound under the ladder it was ranked on"
        );
        assert!(
            foreign
                .inconsistency()
                .is_some_and(|m| m.contains("ladder-derived bound")),
            "{:?}",
            foreign.inconsistency()
        );

        // And WITHOUT a ladder there is no vocabulary to convict on: the same
        // under-claiming row is not this arm's business when the caller holds
        // no edges at all.
        assert_eq!(
            under.inconsistency_against(Some(&[])),
            None,
            "an EMPTY ladder is a correct skip"
        );
    }

    #[test]
    fn a_tail_less_short_row_is_bounded_by_the_ladder_on_both_sides_of_its_edge() {
        let edge = *DRAIN_GAP_BUCKET_EDGES_US.last().unwrap();
        // The PRODUCER's own overflow row, at the edge: every tail overflowed,
        // depth 25 at 100 Hz => exactly `edge`.
        let at_edge = verdict_of(25, hz(100), &gaps_all(5_000_000, 100));
        assert_eq!(
            (
                at_edge.verdict,
                at_edge.absorbance_us,
                at_edge.measured_tail_us
            ),
            (AbsorbanceVerdict::Short, Some(edge), None),
            "precondition: this IS the tail-less overflow row, exactly at the edge"
        );
        assert_eq!(
            at_edge.inconsistency(),
            None,
            "the row the producer really emits at the boundary must stay SOUND"
        );

        // ONE DEPTH STEP further, which the producer cannot reach: above the
        // edge it emits `Unrankable`, so this combination has been hand-made.
        //
        // The absorbance must still FOLLOW from the row's own depth and
        // rate, or the derivation rule convicts it before this arm is reached
        // and the test would pass on the wrong message. At 100 Hz the depth
        // quantum is 10 ms, so the nearest derivable point above the edge is one
        // deeper queue rather than one microsecond. That costs nothing here: the
        // `>` boundary is pinned from the other side by `at_edge` above, whose
        // absorbance IS the edge and which must stay sound — a check written
        // `>=` fails that assertion.
        let past_depth = (edge / 10_000) + 1;
        let past_edge = TopicAbsorbance {
            tap_buffer_depth: past_depth,
            absorbance_us: Some(past_depth * 10_000),
            ..at_edge
        };
        assert!(
            past_edge.absorbance_us.is_some_and(|a| a > edge),
            "precondition: this row's absorbance is above the ladder's last edge"
        );
        assert!(
            past_edge
                .inconsistency()
                .is_some_and(|m| m.contains("above the highest gap the ladder can rank")),
            "{:?}",
            past_edge.inconsistency()
        );

        // The shape that matters: a plausible-looking row
        // whose absorbance is orders of magnitude above anything the ladder can
        // describe. Without this rule `bag info` counts it as a real shortfall.
        // (Depth 500 at 100 Hz DERIVES 5 s of absorbance, so only the
        // ladder arm can convict it — an absorbance that did not follow from the
        // row's own depth and rate would be caught by the derivation rule
        // instead, and this assertion would pass on the wrong message.)
        let far_past = TopicAbsorbance {
            tap_buffer_depth: 500,
            absorbance_us: Some(5_000_000),
            shortfall_at_least_us: Some(1),
            ..at_edge
        };
        assert!(
            far_past
                .inconsistency()
                .is_some_and(|m| m.contains("above the highest gap the ladder can rank")),
            "{:?}",
            far_past.inconsistency()
        );

        // THE LADDER IS A PARAMETER, and it is load-bearing in BOTH directions.
        //
        // A recording carries its own `edges_us`, so a row ranked against a
        // TALLER ladder is sound on its own terms and must not be flagged by a
        // reader that happens to have been built with a shorter one…
        let taller: Vec<u64> = DRAIN_GAP_BUCKET_EDGES_US
            .iter()
            .copied()
            .chain(std::iter::once(edge * 4))
            .collect();
        // Sound "on its own terms" now includes the CLAIM — a row
        // ranked on the taller ladder carries THAT ladder's derived bound.
        let on_taller = TopicAbsorbance {
            shortfall_at_least_us: Some((edge * 4) + 1 - past_edge.absorbance_us.unwrap()),
            ..past_edge
        };
        assert_eq!(
            on_taller.inconsistency_against(Some(&taller)),
            None,
            "a row ranked on a taller ladder is sound against THAT ladder"
        );
        assert!(
            far_past
                .inconsistency_against(Some(&taller))
                .is_some_and(|m| m.contains("above the highest gap the ladder can rank")),
            "…but the taller ladder still has a top, and this row is above it"
        );
        // …and a histogram with NO edges cannot be asked at all, so the arm is
        // skipped rather than guessed. (An absence of the vocabulary is not
        // evidence against the row.)
        assert_eq!(far_past.inconsistency_against(Some(&[])), None);

        // ANTI-TAUTOLOGY: the arm is scoped to the TAIL-LESS row. A `Short` row
        // that names its tail is judged against that tail, whatever the ladder
        // says — so a PRODUCIBLE one stays sound. Depth 20 at 100 Hz covers
        // 200 ms against the ladder's own 250 ms top rung, which needs depth 25.
        let with_tail = TopicAbsorbance {
            tap_buffer_depth: 20,
            measured_tail_us: Some(edge),
            absorbance_us: Some(200_000),
            required_depth: Some(25),
            shortfall_at_least_us: Some(50_000),
            ..at_edge
        };
        assert_eq!(
            with_tail.inconsistency(),
            None,
            "a served tail is the comparison; the ladder bound is only for its absence"
        );

        // …and the SCOPING itself, which that sound row cannot pin: a served
        // tail is always at most the ladder's last edge, and a `Short`
        // verdict puts the absorbance strictly under the tail — so on a
        // PRODUCIBLE row the absorbance can never reach the edge this arm
        // compares against, and deleting the `measured_tail_us.is_none()`
        // conjunct would change nothing a sound row can see.
        //
        // It is pinned on an UNPRODUCIBLE one instead, by the MESSAGE rather
        // than the verdict. This row is contradictory twice over — its tail is
        // no rung of the ladder, and its absorbance is above the top rung — and
        // which contradiction a reader is TOLD about is the whole value of the
        // scoping: "the tail is not a bucket edge" points at the number that is
        // wrong, while "the absorbance is above the highest gap the ladder can
        // rank" describes an overflow row this is not.
        let huge_tail = TopicAbsorbance {
            tap_buffer_depth: 500,
            measured_tail_us: Some(9_000_000),
            absorbance_us: Some(5_000_000),
            required_depth: Some(900),
            shortfall_at_least_us: Some(4_000_000),
            ..at_edge
        };
        assert!(
            huge_tail
                .inconsistency()
                .is_some_and(|m| m.contains("not a bucket edge")),
            "a tail-carrying row must be convicted by its tail, not by an arm about tail-less \
             rows: {:?}",
            huge_tail.inconsistency()
        );
    }

    /// The `Unrankable` arm accepts EXACTLY the producer's
    /// image of that row, and nothing wider.
    ///
    /// Rejecting a measured tail and nothing else would make it the
    /// loosest of the four: a decoded row could omit the absorbance, or carry a
    /// `required_depth`, or claim a shortfall, or name an absorbance the ladder
    /// demonstrably ranks — and `bag info` would count every one of them under "met
    /// a stall the histogram cannot measure", i.e. reported a topic as beyond
    /// the instrument's reach while its own numbers say otherwise.
    ///
    /// The image is read off `absorbance_verdict_inner` rather than guessed. The
    /// arm is one `_` inside the ladder match of the OVERFLOW branch, so a row
    /// reaching it has passed the positive-rate filter, the depth-zero return
    /// and the vacancy return, and found no served quantile: `absorbance_us` is
    /// `Some` (written before the vacancy check, i.e. before EVERY verdict
    /// downstream of it), while the tail, the required depth and the shortfall
    /// are all `None` (their only writers are the served-tail arms and the
    /// `Short` overflow arm, which this is the alternative to).
    ///
    /// Each deviation is driven with everything ELSE producer-true, so the arm
    /// under test is the one that convicts rather than a sibling rule or the
    /// derivation check.
    #[test]
    fn an_unrankable_row_accepts_exactly_the_producers_image() {
        let edge = *DRAIN_GAP_BUCKET_EDGES_US.last().unwrap();
        // The PRODUCER's own unrankable row: every gap is 5 s (overflow), and
        // depth 26 at 100 Hz derives 260 ms — one slot ABOVE the ladder's edge,
        // which is the only side of it this verdict is reachable from.
        let gaps = gaps_all(5_000_000, 100);
        let produced = verdict_of(26, hz(100), &gaps);
        assert_eq!(
            (
                produced.verdict,
                produced.absorbance_us,
                produced.measured_tail_us,
                produced.required_depth,
                produced.shortfall_at_least_us,
            ),
            (
                AbsorbanceVerdict::Unrankable,
                Some(260_000),
                None,
                None,
                None
            ),
            "precondition: THIS is the producer's image of an unrankable row"
        );
        assert_eq!(
            produced.inconsistency(),
            None,
            "the row the producer really emits must stay SOUND"
        );

        // Each field the producer cannot stamp, one at a time.
        let deviations: [(TopicAbsorbance, &str); 4] = [
            (
                TopicAbsorbance {
                    measured_tail_us: Some(300_000),
                    ..produced
                },
                "carries a measured tail",
            ),
            (
                TopicAbsorbance {
                    required_depth: Some(30),
                    ..produced
                },
                "names the depth",
            ),
            (
                TopicAbsorbance {
                    shortfall_at_least_us: Some(1),
                    ..produced
                },
                "claims a shortfall",
            ),
            (
                // Counter-intuitive, and easy to read
                // backwards: `Unrankable` makes no absorbance-DEPENDENT claim,
                // but the producer computed the absorbance to REACH it, so the
                // field is unconditional. Omitting it also disarms the ladder
                // rule below and the derivation check, both of which read it.
                TopicAbsorbance {
                    absorbance_us: None,
                    ..produced
                },
                "reports no absorbance",
            ),
        ];
        for (row, why) in deviations {
            assert!(
                row.inconsistency().is_some_and(|m| m.contains(why)),
                "expected `{why}`, got {:?} ({row:?})",
                row.inconsistency()
            );
        }

        // THE LADDER, pinned on BOTH sides of the edge — the mirror of the
        // tail-less `Short` rule, which refuses the same row from above.
        //
        // AT the edge the producer emits `Short` (the overflow bucket holds only
        // gaps strictly greater than the last edge, so an absorbance of exactly
        // the edge is provably under the tail), so an `Unrankable` claiming it
        // has been hand-made. Depth 25 at 100 Hz derives exactly that, which
        // keeps the derivation check quiet and leaves this arm the only one that
        // can convict — and it is what pins the comparison as `<=` rather than
        // `<`, since `produced` one slot higher must stay sound.
        let at_edge = TopicAbsorbance {
            tap_buffer_depth: 25,
            absorbance_us: Some(edge),
            ..produced
        };
        assert!(
            at_edge
                .inconsistency()
                .is_some_and(|m| m.contains("at or below the highest gap the ladder can rank")),
            "{:?}",
            at_edge.inconsistency()
        );
        // …and WELL below it, the shape an operator would actually be misled by:
        // a shallow tap reported as beyond the instrument's reach.
        let shallow = TopicAbsorbance {
            tap_buffer_depth: 4,
            absorbance_us: Some(40_000),
            ..produced
        };
        assert!(
            shallow
                .inconsistency()
                .is_some_and(|m| m.contains("at or below the highest gap the ladder can rank")),
            "{:?}",
            shallow.inconsistency()
        );

        // THE LADDER IS A PARAMETER, both directions. A row ranked against a
        // SHORTER ladder is sound on its own terms even though this build's
        // vocabulary ranks it…
        let shorter: Vec<u64> = DRAIN_GAP_BUCKET_EDGES_US
            .iter()
            .copied()
            .take_while(|e| *e < 50_000)
            .collect();
        assert!(
            shorter.last().is_some_and(|e| *e < 40_000),
            "precondition: the shorter ladder's top edge is under the shallow row's absorbance"
        );
        assert_eq!(
            shallow.inconsistency_against(Some(&shorter)),
            None,
            "sound under the ladder it was ranked on"
        );
        // …while a TALLER one still has a top, and the producer's own row is
        // below THAT, so on those terms it is the one that could not have been
        // emitted.
        let taller: Vec<u64> = DRAIN_GAP_BUCKET_EDGES_US
            .iter()
            .copied()
            .chain(std::iter::once(edge * 4))
            .collect();
        assert!(
            produced
                .inconsistency_against(Some(&taller))
                .is_some_and(|m| m.contains("at or below the highest gap the ladder can rank")),
            "{:?}",
            produced.inconsistency_against(Some(&taller))
        );
        // …and a histogram with NO edges cannot be asked at all: with no last
        // edge the producer takes this arm unconditionally, so the shallow row
        // is one it really could have emitted.
        assert_eq!(
            shallow.inconsistency_against(Some(&[])),
            None,
            "an EMPTY ladder is a correct skip"
        );
    }

    /// The absorbance must FOLLOW from the depth
    /// and rate the row itself reports.
    ///
    /// Every other arm compares the absorbance against something else on the row
    /// and none asks where it came from — so a decoded row could inflate it and
    /// buy a verdict its own two operands refute, with every arm satisfied.
    /// The shape: depth 3 at 100 Hz claiming 40 ms of
    /// absorbance where `depth / rate` gives 30, and `bag info` counted the topic
    /// as absorbing its stalls.
    ///
    /// The anti-tautology arm is the SAME claimed absorbance from the depth that
    /// really derives it, so the rule is pinned as a check on the DERIVATION and
    /// not on the number.
    #[test]
    fn an_absorbance_that_does_not_follow_from_its_own_depth_and_rate_is_not_producible() {
        // The tail a SERVED quantile can report is always a bucket EDGE, and the
        // shipped ladder steps 25 ms -> 50 ms, so a 35 ms tail (and with
        // it a 40 ms inflated claim) is not a shape the producer can hand
        // back. The depth and rate are kept exactly; the two microsecond
        // figures move onto the ladder the recorder really ranks on.
        let gaps = gaps_all(50_000, 100);

        // The PRODUCER, handed that depth and rate: 3 slots at 100 Hz
        // cover 30 ms, which is under the 50 ms tail, so the true row is SHORT.
        let truth = verdict_of(3, hz(100), &gaps);
        assert_eq!(
            (truth.verdict, truth.absorbance_us, truth.measured_tail_us),
            (AbsorbanceVerdict::Short, Some(30_000), Some(50_000)),
            "precondition: the producer-true absorbance for this pair is 30 ms"
        );
        assert_eq!(truth.inconsistency(), None, "the producer's own output");

        // Run first, as a control: removing the rule is seen to
        // leave this row exactly where it was: 60 ms of absorbance from the
        // depth that DERIVES it, `Absorbs`, sound.
        let derived = verdict_of(6, hz(100), &gaps);
        assert_eq!(
            (derived.verdict, derived.absorbance_us),
            (AbsorbanceVerdict::Absorbs, Some(60_000)),
            "precondition: depth 6 at 100 Hz really does cover 60 ms"
        );
        assert_eq!(
            derived.inconsistency(),
            None,
            "a row whose absorbance follows from its own depth and rate is sound"
        );

        // THE INFLATED ROW: the very same 60 ms, on the same depth and rate,
        // and the verdict that inflation buys. Every OTHER arm is satisfied — an
        // `Absorbs` row with no shortfall whose absorbance clears the tail it
        // names — so without this rule the row is aggregated as a healthy topic.
        // Nothing but the derivation separates it from `derived` above, which is
        // why the two are asserted in this order.
        let forged = TopicAbsorbance {
            verdict: AbsorbanceVerdict::Absorbs,
            absorbance_us: derived.absorbance_us,
            shortfall_at_least_us: None,
            ..truth
        };
        assert!(
            forged
                .inconsistency()
                .is_some_and(|m| m.contains("its own depth and rate")),
            "{:?}",
            forged.inconsistency()
        );
    }

    /// …and the derived value is the producer's arithmetic EXACTLY: truncating
    /// division, clamped, pinned one microsecond either side.
    ///
    /// A rule that rounded the other way (or recomputed in `f64`) would admit a
    /// row off by a microsecond, which is the shape a drifting foreign writer
    /// actually produces. Driven on `NoClaim` rows — the vacant-histogram arm
    /// computes the absorbance and then declines to rank it — so no sibling arm
    /// can convict them and the derivation rule is the only thing under test.
    #[test]
    fn the_derived_absorbance_mirrors_the_producers_truncating_division_and_its_clamp() {
        // 1 slot at 3 Hz: 1e9 / 3000 = 333_333.33…, and the producer TRUNCATES.
        let rate = |millihertz| {
            Some(DrainRateEstimate {
                millihertz,
                is_floor: false,
            })
        };
        let truncated = verdict_of(1, rate(3_000), &DrainGapHistogram::new());
        assert_eq!(
            (truncated.verdict, truncated.absorbance_us),
            (AbsorbanceVerdict::NoClaim, Some(333_333)),
            "precondition: the producer truncates rather than rounding"
        );
        assert_eq!(truncated.inconsistency(), None, "the producer's own output");
        for claimed in [333_332u64, 333_334] {
            let row = TopicAbsorbance {
                absorbance_us: Some(claimed),
                ..truncated
            };
            assert!(
                row.inconsistency()
                    .is_some_and(|m| m.contains("its own depth and rate")),
                "{claimed}: {:?}",
                row.inconsistency()
            );
        }

        // …and the CLAMP, which the mirror must carry too: a depth this deep at
        // a millihertz rate overflows `u64` microseconds, and the producer pins
        // the result at the ceiling rather than wrapping.
        let clamped = verdict_of(u64::MAX, rate(1), &DrainGapHistogram::new());
        assert_eq!(
            clamped.absorbance_us,
            Some(u64::MAX),
            "precondition: the producer saturates at the ceiling"
        );
        assert_eq!(clamped.inconsistency(), None, "the producer's own output");
        assert!(
            TopicAbsorbance {
                absorbance_us: Some(u64::MAX - 1),
                ..clamped
            }
            .inconsistency()
            .is_some_and(|m| m.contains("its own depth and rate")),
            "one microsecond under the ceiling does not follow from these operands"
        );
    }

    /// The REQUIRED DEPTH is derived too, and it
    /// is the one number on the row an operator provisions a queue from.
    ///
    /// `render_line` prints it as `needs depth N`, and without this rule nothing
    /// asks where it came from: a decoded row could name any depth at all
    /// beside a tail and a rate that derive a different one, and both renderers
    /// would print it as guidance. The producer's arithmetic is
    /// `required_depth(mhz, tail, gaps)` — the shipped `absorbing_depth`, which
    /// its own `debug_assert` pins to the exact integer `ceil(mhz * tail /
    /// 1e9)` — and it is called at BOTH sites that serve a tail, so a served
    /// tail always mints one.
    ///
    /// Pinned one step either side of the value, since the two directions
    /// mislead differently: a depth BELOW the derived one under-provisions the
    /// queue an operator then sizes, and one above wastes the budget the tap
    /// was sized against.
    #[test]
    fn the_required_depth_is_the_one_its_own_rate_and_tail_derive() {
        // 100 Hz over a 50 ms tail: ceil(0.100 * 50) = 5 slots. HAND ORACLE.
        let gaps = gaps_all(50_000, 100);
        let short = verdict_of(3, hz(100), &gaps);
        assert_eq!(
            (
                short.verdict,
                short.absorbance_us,
                short.measured_tail_us,
                short.required_depth
            ),
            (
                AbsorbanceVerdict::Short,
                Some(30_000),
                Some(50_000),
                Some(5)
            ),
            "precondition: the producer's own required depth here is 5"
        );
        assert_eq!(short.inconsistency(), None, "the producer's own output");

        for claimed in [4u64, 6] {
            let row = TopicAbsorbance {
                required_depth: Some(claimed),
                ..short
            };
            assert!(
                row.inconsistency()
                    .is_some_and(|m| m.contains("its own rate and tail derive")),
                "{claimed}: {:?}",
                row.inconsistency()
            );
        }
        // …and ABSENT is a deviation too: a served tail always mints one, so a
        // row that names a tail and no depth for it dropped the actionable half.
        assert!(
            TopicAbsorbance {
                required_depth: None,
                ..short
            }
            .inconsistency()
            .is_some_and(|m| m.contains("its own rate and tail derive")),
            "a served tail always mints a required depth"
        );

        // The CEILING arithmetic, at the boundary where truncation and ceiling
        // disagree: 100 Hz over a 25 ms tail is 2.5 slots, and the producer
        // rounds UP (a depth of 2 would not have absorbed it).
        let boundary = verdict_of(1, hz(100), &gaps_all(25_000, 100));
        assert_eq!(
            (boundary.measured_tail_us, boundary.required_depth),
            (Some(25_000), Some(3)),
            "precondition: ceil(2.5) is 3, not 2"
        );
        assert!(
            TopicAbsorbance {
                required_depth: Some(2),
                ..boundary
            }
            .inconsistency()
            .is_some_and(|m| m.contains("its own rate and tail derive")),
            "a truncating mirror would accept the depth that does NOT absorb the tail"
        );

        // …and the ABSORBS arm reaches the same rule by the other route, so the
        // check is not a `Short`-only accident.
        let absorbs = verdict_of(9, hz(100), &gaps);
        assert_eq!(
            (absorbs.verdict, absorbs.required_depth),
            (AbsorbanceVerdict::Absorbs, Some(5))
        );
        assert!(
            TopicAbsorbance {
                required_depth: Some(1),
                ..absorbs
            }
            .inconsistency()
            .is_some_and(|m| m.contains("its own rate and tail derive")),
            "an absorbing row names a required depth too, and it is derived the same way"
        );

        // The DEPTH-ZERO arm is the third writer, and it is the one that builds
        // its row by hand: 1000 Hz over the same 50 ms tail needs 50 slots, and
        // the row reports that beside an absorbance of zero.
        let depth_zero = verdict_of(0, hz(1_000), &gaps);
        assert_eq!(
            (
                depth_zero.verdict,
                depth_zero.absorbance_us,
                depth_zero.required_depth
            ),
            (AbsorbanceVerdict::Short, Some(0), Some(50)),
            "precondition: the depth-zero arm requires a depth too"
        );
        assert_eq!(depth_zero.inconsistency(), None);
    }

    /// …and a row with NO tail requires a depth for NOTHING.
    ///
    /// The ladder-overflow `Short` row is the one this half newly reaches —
    /// `NoClaim` and `Unrankable` are told by their own arms — and it is the
    /// worst place for a forged depth: the row's own text says the stall was
    /// worse than the ladder can describe, so `needs depth N` beside it is
    /// provisioning guidance derived from a measurement that does not exist.
    #[test]
    fn a_tail_less_row_requires_a_depth_for_nothing() {
        // The producer's own overflow row: every gap is 5 s, depth 25 at 100 Hz
        // sits exactly at the ladder's last edge.
        let overflow = verdict_of(25, hz(100), &gaps_all(5_000_000, 100));
        assert_eq!(
            (
                overflow.verdict,
                overflow.measured_tail_us,
                overflow.required_depth
            ),
            (AbsorbanceVerdict::Short, None, None),
            "precondition: the overflow arm stamps no required depth"
        );
        assert_eq!(overflow.inconsistency(), None, "the producer's own output");

        let forged = TopicAbsorbance {
            required_depth: Some(26),
            ..overflow
        };
        assert!(
            forged
                .inconsistency()
                .is_some_and(|m| m.contains("requires a depth for nothing")),
            "{:?}",
            forged.inconsistency()
        );

        // The other two tail-less verdicts keep naming their OWN contradiction,
        // which is sharper — this arm must not take their message away.
        let unrankable = verdict_of(26, hz(100), &gaps_all(5_000_000, 100));
        assert_eq!(unrankable.verdict, AbsorbanceVerdict::Unrankable);
        assert!(
            TopicAbsorbance {
                required_depth: Some(30),
                ..unrankable
            }
            .inconsistency()
            .is_some_and(|m| m.contains("UNRANKABLE")),
            "the unrankable arm names the depth first"
        );
        let no_claim = verdict_of(4, hz(100), &DrainGapHistogram::new());
        assert_eq!(no_claim.verdict, AbsorbanceVerdict::NoClaim);
        assert!(
            TopicAbsorbance {
                required_depth: Some(30),
                ..no_claim
            }
            .inconsistency()
            .is_some_and(|m| m.contains("NO CLAIM")),
            "the no-claim arm names the comparison first"
        );
    }

    /// A SERVED tail is a RUNG of the ladder, never a number
    /// between two.
    ///
    /// `DrainGapHistogram::quantile_us` returns `edges_us.get(i)` and has no
    /// other exit that yields a value, so a tail the ladder does not contain is
    /// a measurement the instrument cannot make. Every arm that reads the tail
    /// compares against it without asking where on the ladder it sits, so a
    /// forged value between two rungs bought a verdict, a shortfall computed
    /// from it, and a required depth — all arithmetically self-consistent.
    ///
    /// Driven at a value BETWEEN two real rungs rather than an absurd one,
    /// because that is the shape a hand edit or a foreign writer produces and
    /// the one every sibling rule waves through.
    #[test]
    fn a_measured_tail_that_is_not_a_rung_of_the_ladder_is_not_producible() {
        let gaps = gaps_all(50_000, 100);
        let short = verdict_of(3, hz(100), &gaps);
        assert_eq!(
            (short.measured_tail_us, short.shortfall_at_least_us),
            (Some(50_000), Some(20_000)),
            "precondition: the producer serves the 50 ms RUNG"
        );
        assert_eq!(short.inconsistency(), None, "the producer's own output");

        // 37 ms: between the 25 ms and 50 ms rungs, with the shortfall and the
        // required depth recomputed FROM it, so every sibling rule is satisfied
        // and only the membership check can convict.
        let forged = TopicAbsorbance {
            measured_tail_us: Some(37_000),
            shortfall_at_least_us: Some(37_000 - 30_000),
            required_depth: Some(4),
            ..short
        };
        assert!(
            forged
                .inconsistency()
                .is_some_and(|m| m.contains("not a bucket edge")),
            "{:?}",
            forged.inconsistency()
        );

        // ANTI-TAUTOLOGY: every rung of the shipped ladder is accepted, with
        // the row's own derived numbers moved onto it. Without this the rule
        // would pass as a check that simply distrusts tails.
        for &rung in DRAIN_GAP_BUCKET_EDGES_US {
            let absorbance = short.absorbance_us.expect("the producer's absorbance");
            let row = TopicAbsorbance {
                measured_tail_us: Some(rung),
                required_depth: Some(rung.div_ceil(10_000)),
                verdict: if absorbance >= rung {
                    AbsorbanceVerdict::Absorbs
                } else {
                    AbsorbanceVerdict::Short
                },
                shortfall_at_least_us: (absorbance < rung).then(|| rung - absorbance),
                ..short
            };
            assert_eq!(
                row.inconsistency(),
                None,
                "{rung} us is a rung of the shipped ladder: {row:?}"
            );
        }

        // THE LADDER IS A PARAMETER, both directions — the same discipline the
        // tail-less arms carry. A recording whose histogram had a rung at 37 ms
        // really could serve that tail…
        let foreign: Vec<u64> = DRAIN_GAP_BUCKET_EDGES_US
            .iter()
            .copied()
            .chain(std::iter::once(37_000))
            .collect();
        assert_eq!(
            forged.inconsistency_against(Some(&foreign)),
            None,
            "sound under the ladder it was ranked on"
        );
        // …and a histogram with NO edges cannot be asked at all, so the arm is
        // skipped rather than guessed.
        assert_eq!(forged.inconsistency_against(Some(&[])), None);
    }

    /// A rate and an absorbance travel together, in BOTH
    /// directions.
    ///
    /// The three verdicts that READ an absorbance already refuse a row without
    /// one. `NoClaim` is the fourth, and it is the one that may legitimately
    /// carry a rate — so a row could claim `100.000 Hz` and render `absorbance
    /// unknown` beside it, which a reader takes for "the queue could not be
    /// measured" rather than for two hands disagreeing. The producer writes the
    /// absorbance UNCONDITIONALLY once past the rate filter (the depth-zero arm
    /// stamps `Some(0)`), and writes nothing at all before it.
    #[test]
    fn a_rate_and_an_absorbance_are_present_together_or_not_at_all() {
        // The two SOUND no-claim shapes, which is why the rule is a
        // biconditional over the pair and not a check on either field.
        let with_rate = verdict_of(4, hz(100), &DrainGapHistogram::new());
        assert_eq!(
            (
                with_rate.verdict,
                with_rate.rate_mhz,
                with_rate.absorbance_us
            ),
            (AbsorbanceVerdict::NoClaim, Some(100_000), Some(40_000)),
            "precondition: a vacant histogram computes the absorbance, then declines to rank it"
        );
        assert_eq!(with_rate.inconsistency(), None);
        let rateless = verdict_of(4, None, &gaps_all(10_000, 100));
        assert_eq!(
            (rateless.rate_mhz, rateless.absorbance_us),
            (None, None),
            "precondition: with no rate the producer returns before writing an absorbance"
        );
        assert_eq!(rateless.inconsistency(), None);

        // Each half alone.
        assert!(
            TopicAbsorbance {
                absorbance_us: None,
                ..with_rate
            }
            .inconsistency()
            .is_some_and(|m| m.contains("no absorbance was derived from it")),
            "a rate with no absorbance beside it"
        );
        assert!(
            TopicAbsorbance {
                absorbance_us: Some(40_000),
                ..rateless
            }
            .inconsistency()
            .is_some_and(|m| m.contains("no rate for it to have been derived from")),
            "an absorbance with no rate behind it"
        );
    }

    /// A rate of ZERO is unproducible on EVERY verdict,
    /// `NoClaim` included.
    ///
    /// The zero-rate rule refuses it wherever a verdict rests on it, and exempts
    /// `NoClaim` because such a row may carry the rate it declined to rank. But
    /// that exemption is for a rate the producer ACCEPTED, and `Some(0)` never
    /// is: `rate_mhz`'s contract is that absence means UNKNOWN, and
    /// `absorbance_verdict_inner` filters the zero before the row is built. The
    /// uncovered cell rendered `0.000 Hz` on the one verdict that exists to say
    /// nothing was measured.
    #[test]
    fn a_no_claim_row_carrying_a_zero_rate_is_not_producible_either() {
        let no_claim = verdict_of(4, hz(100), &DrainGapHistogram::new());
        assert_eq!(no_claim.verdict, AbsorbanceVerdict::NoClaim);

        let zeroed = TopicAbsorbance {
            rate_mhz: Some(0),
            // …with the absorbance dropped too, so the rate-absorbance pairing rule
            // cannot be what convicts it: `Some(0)` is not a positive rate, so
            // the pair "no positive rate, no absorbance" is coherent and only
            // the zero itself is left.
            absorbance_us: None,
            ..no_claim
        };
        assert!(
            zeroed
                .inconsistency()
                .is_some_and(|m| m.contains("a rate of ZERO")),
            "{:?}",
            zeroed.inconsistency()
        );
        // ANTI-TAUTOLOGY: the same row with the rate ABSENT is the legitimate
        // shape and stays sound, so the rule is about the zero and not about
        // a no-claim row's rate field.
        assert_eq!(
            TopicAbsorbance {
                rate_mhz: None,
                absorbance_us: None,
                ..no_claim
            }
            .inconsistency(),
            None
        );
    }

    /// An OVER-BUDGET occupancy is derived from the row's own
    /// depth and slot width, and reports the row's own budget.
    ///
    /// `budget_occupancy` mints all three numbers from two inputs, and every
    /// one of them is RENDERED: `render_line` prints "depth N is OVER BUDGET:
    /// priced for B B, now C B", which is what an operator re-sizes a tap
    /// from. Nothing asked where any of them came from.
    #[test]
    fn an_over_budget_occupancy_follows_from_the_rows_own_depth_and_slot_width() {
        const BUDGET: u64 = 4 * 1024 * 1024;
        const SLOT: u64 = 1024 * 1024;
        let gaps = gaps_all(10_000, 100);
        // The PRODUCER's own pair: 6 slots x 1 MiB = 6 MiB against a 4 MiB
        // budget, exactly what `budget_occupancy(BUDGET, Some(SLOT), 6)` mints.
        let (over, unpriced) = budget_occupancy(BUDGET, Some(SLOT), 6);
        assert!(!unpriced);
        let sound = absorbance_verdict(6, hz(100), &gaps, Some(BUDGET), over, false);
        assert_eq!(sound.inconsistency(), None, "the producer's own output");
        let ob = over.expect("this tap really is over its budget");

        // The CAPACITY, one byte either side of the product.
        for capacity in [ob.capacity_bytes - 1, ob.capacity_bytes + 1] {
            let row = TopicAbsorbance {
                over_budget: Some(OverBudget {
                    capacity_bytes: capacity,
                    ..ob
                }),
                ..sound
            };
            assert!(
                row.inconsistency()
                    .is_some_and(|m| m.contains("its own depth and widest slot")),
                "{capacity}: {:?}",
                row.inconsistency()
            );
        }
        // …reached from the SLOT WIDTH too, since the product has two operands
        // and a wrong one of either buys the same rendered line.
        assert!(
            TopicAbsorbance {
                over_budget: Some(OverBudget {
                    widest_slot_bytes: SLOT * 2,
                    ..ob
                }),
                ..sound
            }
            .inconsistency()
            .is_some_and(|m| m.contains("its own depth and widest slot")),
            "a slot width that does not multiply out to the capacity beside it"
        );

        // The BUDGET the occupancy is against must be the row's own: a tap
        // sized against one budget and reported over another names two.
        assert!(
            TopicAbsorbance {
                over_budget: Some(OverBudget {
                    budget_bytes: BUDGET / 2,
                    ..ob
                }),
                ..sound
            }
            .inconsistency()
            .is_some_and(|m| m.contains("not the one the row itself carries")),
            "the occupancy names a budget the row does not carry"
        );

        // …and the PRESENCE condition: `budget_occupancy` is `Some` only while
        // the capacity EXCEEDS the budget, so an occupancy reporting a capacity
        // within it is a claim the producer never makes. Pinned AT the boundary,
        // which is the reachable side — `capacity > budget` is strict, so a tap
        // exactly at its budget is inside it.
        let inside = OverBudget {
            budget_bytes: SLOT * 6,
            capacity_bytes: SLOT * 6,
            widest_slot_bytes: SLOT,
        };
        assert!(
            TopicAbsorbance {
                budget_bytes: Some(SLOT * 6),
                over_budget: Some(inside),
                ..sound
            }
            .inconsistency()
            .is_some_and(|m| m.contains("within the budget it names")),
            "a capacity exactly at its budget is INSIDE it"
        );
        // …and one byte over is the row the producer really emits.
        assert_eq!(
            TopicAbsorbance {
                budget_bytes: Some(SLOT * 6 - 1),
                over_budget: Some(OverBudget {
                    budget_bytes: SLOT * 6 - 1,
                    ..inside
                }),
                ..sound
            }
            .inconsistency(),
            None,
            "one byte over the budget is over it"
        );
    }

    /// A budget of ZERO is not a budget.
    ///
    /// The production call site is `(tap.budget_bytes > 0).then_some(..)`, so a
    /// budgeted tap carries a positive number and a ceiling-deep one carries
    /// `None` — the two states `depth_mode` and `remedy` are keyed on. `Some(0)`
    /// reads as the first and behaves as neither: the row renders `(budgeted)`
    /// and sends the operator to `CERULION_FLASHBACK_TAP_BUDGET_MB`, which
    /// cannot move a tap that was never sized against a budget.
    #[test]
    fn a_budget_of_zero_is_not_a_budget() {
        let gaps = gaps_all(10_000, 100);
        let ceiling = verdict_of(4, hz(100), &gaps);
        assert_eq!(
            (ceiling.budget_bytes, ceiling.depth_mode()),
            (None, "ceiling"),
            "precondition: a ceiling-deep tap carries NO budget"
        );
        assert_eq!(ceiling.inconsistency(), None);

        let zeroed = TopicAbsorbance {
            budget_bytes: Some(0),
            ..ceiling
        };
        assert!(
            zeroed
                .inconsistency()
                .is_some_and(|m| m.contains("the budget it names is ZERO")),
            "{:?}",
            zeroed.inconsistency()
        );
        // ANTI-TAUTOLOGY: a real budget is accepted, and it is what flips the
        // rendered mode and the remedy.
        let budgeted = TopicAbsorbance {
            budget_bytes: Some(4 * 1024 * 1024),
            ..ceiling
        };
        assert_eq!(budgeted.inconsistency(), None);
        assert_eq!(budgeted.depth_mode(), "budgeted");
    }
}
