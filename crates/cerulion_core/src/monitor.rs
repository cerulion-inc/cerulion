// SPDX-License-Identifier: AGPL-3.0-only
//! Monitors — the PURE per-topic rate/liveness watchdog engine.
//!
//! A monitor is **per topic, standing, unconfigured, and a pure consumer of an
//! observation that already exists.** It samples the same
//! [`TopicLiveness`] both liveness planes already
//! produce, classifies with
//! [`LivenessState`] — **never its own thresholds**
//! — and raises/clears alerts on CONFIRMED transitions.
//!
//! Nothing here measures anything. Every number it reads was already computed by
//! the robot's observer or by the desk's own tap statistics, which is what makes
//! the cost claim ("zero ports, zero subscribers, zero threads, zero netd round
//! trips") a property of the design rather than a promise.
//!
//! # The three conditions, and what falls out for free
//!
//! | Condition | Fires when |
//! |---|---|
//! | [`MonitorCondition::Stalled`] | a row that WAS `Streaming` is now `Idle` or `NoData`, confirmed |
//! | [`MonitorCondition::Silent`] | a row reaches `NoData` having never been `Streaming` |
//! | [`MonitorCondition::RateDeviation`] | a row with a learned baseline `R` measures `< R/2` or `> 2R`, confirmed |
//!
//! "Age exceeds a threshold" is deliberately NOT a fourth condition — it is
//! precisely `Streaming → Idle`, already thresholded once inside
//! [`LivenessState`]. The monitor watches
//! CLASSIFIED transitions; it never re-implements a threshold the substrate owns.
//!
//! Watching transitions **from a Streaming state** rather than absolute states is
//! what makes the liveness observer's largest documented degradation corner cost nothing here: the
//! undatable-`Idle` class (`frames_observed > 0`, `last_frame_age_ms: None` — a
//! flushed backlog, a `/tf_static` one-shot) is never [`MonitorCondition::Stalled`]
//! (it was never `Streaming`) and never [`MonitorCondition::Silent`]
//! (`frames_observed > 0` forbids `NoData`). No special case exists for it because
//! none is needed.
//!
//! # Two absolute evidence rules
//!
//! 1. **Never alert on `Unknown`, and never on an absent liveness payload.**
//!    Absence of information is not a condition. An absent payload is reachable
//!    four ways that have nothing to do with the topic: a pre-liveness robot,
//!    `CERULION_TOPIC_LIVENESS=off`, the observer's tap BUDGET exhausted on a wide
//!    robot, and a WAN/iroh-served catalog. A watchdog that read absence as
//!    trouble would page on all four.
//! 2. **Never alert while discovery has not converged.** A short or empty catalog
//!    proves nothing then. The marker already on the wire is reused; no
//!    second one is invented.
//!
//! Both rules are enforced at ONE place — [`MonitorSample::is_evidential`] — and
//! a non-evidential sample is NEUTRAL, deliberately asymmetric:
//!
//! * it **resets** a not-yet-confirmed qualifying streak (losing sight of a topic
//!   breaks the chain of evidence a confirmation is built from),
//! * it **resets the baseline LEARNING run** for the same reason — the baseline is
//!   the median of *consecutive* trustworthy rates, and a blind sample is a hole
//!   the topic may have restarted behind, and
//! * it **never advances** a clearing streak (losing sight of a topic is not an
//!   all-clear for an alert already raised).
//!
//! All three fall on the same side: never claim more than was observed. What a
//! blind sample does NOT touch is state that was already EARNED — a frozen
//! baseline, the LAST frozen baseline (the stall basis, see below),
//! `ever_streaming`, `ever_floor`, and the cumulative slow-topic counter all
//! survive, because forgetting them would be its own false claim.
//!
//! # The stall BASIS, and why it outlives the re-learn wipe
//!
//! D4 freezes the rate baseline and throws it away whenever ANY condition on the
//! row CLEARS, because a restart may legitimately change the RATE. D7 then reads
//! that same baseline as its only evidence that the row was ever measured fast
//! enough for `Streaming → Idle` to be a fault rather than the recency window.
//! Those two readings collide on a publisher that DIES: its rate going absent is
//! itself what clears a raised [`MonitorCondition::RateDeviation`], the wipe
//! lands, and re-learning needs [`MONITOR_LEARN_SAMPLES`] consecutive trustworthy
//! `Streaming` samples — which a dead publisher can never supply. The row then
//! parks in [`MonitorState::Learning`] FOREVER on exactly the topic an operator
//! most needs paged (MEASURED: 220 `Idle` samples, 88 s, zero stall alerts).
//!
//! So the row keeps ONE more number — the LAST FROZEN baseline — written only
//! where a baseline freezes, superseded only by the next freeze, and dropped only
//! with the row ([`MonitorEngine::forget`]). The stall gate falls
//! through to it when there is no live baseline, and the alert carries it. The
//! wipe, the stall predicate, the stall gate's evidence standard and the wire are all
//! unchanged: eligibility is simply LAST-KNOWN-GOOD rather than un-earned by a
//! clear, on I4's own reasoning (earned state survives).
//!
//! # What a ROW is
//!
//! One row is a topic **on a robot**, keyed by both (see `RowKey`). One desk
//! watches many robots, so `/lowstate` from robot A and `/lowstate` from robot B
//! are two independent rows with two baselines and two alert histories; a genuine
//! LOCAL producer (`robot: None`) is a third. Nothing is ever re-attributed in
//! place — a sample naming a different origin opens its own row, settle window
//! included.
//!
//! # The settle window
//!
//! A sample taken within [`MONITOR_SETTLE_NS`] of a row FIRST APPEARING is
//! discarded whole — it contributes to no baseline, no `ever_streaming` mark and
//! no condition tracker.
//!
//! This is wider than "discard baseline samples", and deliberately so. vizd's own
//! documented residual (`daemon.rs`, `TopicStat::observe_frames`) is that a
//! retained-history flush SPLIT across two poll ticks dates on its second chunk,
//! so a dead-but-latched route can read `Streaming` for up to the recency window
//! once per attach before settling to `Idle`. A baseline-only discard leaves that
//! spurious `Streaming` free to set `ever_streaming`, and the settle to `Idle`
//! then raises a false [`MonitorCondition::Stalled`] on **every attach of a
//! latched topic** — which is the very leak the residual is listed under. The
//! whole sample is therefore withheld, using the substrate's own constant.
//!
//! Cost, stated: an unattached row whose robot-side observation is hours old still
//! pays the window, because the clock is "when this ENGINE first saw the row", not
//! "when the robot's tap attached" — a delay, never a wrong answer.
//!
//! # Confirmation needs BOTH a count and a span
//!
//! [`MONITOR_CONFIRM_SAMPLES`] consecutive qualifying samples **and** a span of
//! [`MONITOR_CONFIRM_MIN_SPAN_NS`]. Samples alone cannot absorb the
//! lull-free-restart residual when the sampler is fast; a span alone flaps when
//! the sampler is slow (the ~2 s piggyback plane). Both, always.
//!
//! That is a PRODUCTION rule keyed on the substrate's own clock. The test oracles
//! in this module are COUNTS and hand-built sample vectors, never walls.
//!
//! # What v1 does NOT do
//!
//! No actions or remediation, no cross-topic correlation, nothing runs on the
//! robot, no push (the agent polls), no user-configurable thresholds, no
//! persistence, no non-topic signals, and no decode-failure / frame-loss
//! conditions. Every constant below is DERIVED from a substrate constant rather
//! than tuned.

use crate::transport::liveness::{
    LIVENESS_STREAMING_RECENCY_MS, LIVENESS_SWEEP_INTERVAL_NS, SUSTAINED_REGRESSION_MIN_SPAN_NS,
};
use crate::{LivenessState, TopicLiveness, TopicRateEstimate};
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Constants — DERIVED, not tuned.
// ---------------------------------------------------------------------------

/// How often one row is sampled: `2 ×` the robot's own liveness sweep interval.
///
/// Sampling at or under [`LIVENESS_SWEEP_INTERVAL_NS`] re-reads ONE observation
/// (the robot's record only advances on its own sweep grid), so a monitor running
/// at the sweep rate would count the same evidence twice and confirm in half the
/// real time. Doubling guarantees a FRESH observation even when the two grids
/// beat against each other.
///
/// Consumed by the sampler, not by the pure engine — the engine is
/// driven by whatever cadence its caller chooses and derives every decision from
/// the sample timestamps it is handed.
pub const MONITOR_SAMPLE_INTERVAL_NS: u64 = 2 * LIVENESS_SWEEP_INTERVAL_NS;

/// Consecutive qualifying samples required to RAISE a condition.
///
/// One half of the confirmation; the other is [`MONITOR_CONFIRM_MIN_SPAN_NS`] and
/// BOTH must be satisfied.
pub const MONITOR_CONFIRM_SAMPLES: u32 = 4;

/// Wall span a qualifying run must cover before it can RAISE — the
/// lull-free-restart residual absorber.
///
/// `SUSTAINED_REGRESSION_MIN_SPAN_NS + LIVENESS_SWEEP_INTERVAL_NS`: a lull-free
/// publisher restart heals within the substrate's sustained span plus one further
/// advancement, and the monitor must not page inside a window the substrate is
/// still recovering in. DERIVED from that constant rather than restated, so the
/// two cannot drift apart.
pub const MONITOR_CONFIRM_MIN_SPAN_NS: u64 =
    SUSTAINED_REGRESSION_MIN_SPAN_NS + LIVENESS_SWEEP_INTERVAL_NS;

// The span must strictly EXCEED the substrate's own healing span, or the monitor could
// confirm a stall inside the window the substrate uses to recover from a restart.
const _: () = assert!(MONITOR_CONFIRM_MIN_SPAN_NS > SUSTAINED_REGRESSION_MIN_SPAN_NS);

/// Consecutive non-qualifying samples required to CLEAR a raised condition.
///
/// Lower than [`MONITOR_CONFIRM_SAMPLES`] on purpose: raising is a claim about the
/// robot ("this is broken") and clearing is a retraction, so the asymmetry favours
/// retracting quickly over holding a stale alert.
pub const MONITOR_CLEAR_SAMPLES: u32 = 2;

/// Below this learned rate a topic is INELIGIBLE for
/// [`MonitorCondition::Stalled`] (D7).
///
/// `2 × 1_000_000 / LIVENESS_STREAMING_RECENCY_MS` millihertz — the rate at which
/// a topic's own period reaches half the substrate's recency window. Slower than
/// that and `Streaming ↔ Idle` oscillates BY DESIGN on a perfectly healthy topic
/// (its next frame lands after the window expires), which no amount of hysteresis
/// can fix, because the flapping is the substrate telling the truth. Such a row
/// gets [`MonitorCondition::Silent`] and an explicit
/// [`IneligibleReason::SlowTopic`] instead of a stall it would never stop raising.
pub const MONITOR_STALL_MIN_MHZ: u64 = 2 * 1_000_000 / LIVENESS_STREAMING_RECENCY_MS;

/// Consecutive `Streaming` samples carrying a trustworthy rate that must be seen
/// before a baseline is FROZEN — and, separately, the number of consecutive
/// `Streaming` samples carrying NO rate at all that classify a row
/// [`IneligibleReason::SlowTopic`].
///
/// The second use is what makes the slow-topic gate reachable on a topic the
/// substrate refuses to rate at all: the rate estimator declines a rate for a period longer
/// than its horizon, so a 0.1 Hz topic never learns a baseline and could never be
/// gated by a baseline test. "Streaming, repeatedly, and still unrated" is exactly
/// that shape, observed rather than assumed.
pub const MONITOR_LEARN_SAMPLES: u32 = 8;

/// Samples within this long of a row FIRST APPEARING are discarded whole.
///
/// The substrate's own recency window, which is the documented bound on the
/// split-history-flush residual (see the module docs). Not a new number.
pub const MONITOR_SETTLE_NS: u64 = LIVENESS_STREAMING_RECENCY_MS * 1_000_000;

/// The rate band's numerator/denominator: a topic is deviating below its baseline
/// at `observed × MONITOR_RATE_BAND < baseline`, and above it at
/// `observed > baseline × MONITOR_RATE_BAND`.
///
/// An OCTAVE (½× .. 2×), and integral because
/// [`TopicLiveness`] is `Eq` and this repo's
/// millihertz discipline forbids a float on this path. The rate estimator's docs state plainly that
/// two windows over the same stream legitimately disagree ("19.8 against 20.1 has
/// found the window, not a bug") and that the observer and demand planes use
/// DIFFERENT windows, so an octave is the smallest band no measurement vantage can
/// trip on a healthy topic. It is the one number an operator might reasonably want
/// tighter, and it is a one-constant change.
pub const MONITOR_RATE_BAND: u64 = 2;

/// How many alert events the daemon's ring retains (the daemon owns the ring; the
/// bound is declared here with the rest of the policy).
pub const MONITOR_ALERT_RING: usize = 128;

// A confirmation is meaningless if it can complete on a single sample.
const _: () = assert!(MONITOR_CONFIRM_SAMPLES > 1);
// Clearing must be able to happen at all, and must not be slower than raising.
const _: () =
    assert!(MONITOR_CLEAR_SAMPLES >= 1 && MONITOR_CLEAR_SAMPLES <= MONITOR_CONFIRM_SAMPLES);
// A band of 1 would call every topic deviating; the octave is the floor.
const _: () = assert!(MONITOR_RATE_BAND >= 2);

// ---------------------------------------------------------------------------
// Vocabulary.
// ---------------------------------------------------------------------------

/// One of the three v1 conditions. Serialized snake_case (`stalled` / `silent` /
/// `rate_deviation`) — the closed set a controller matches on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MonitorCondition {
    /// A row that demonstrably delivered dated frames stopped delivering them.
    Stalled,
    /// A row reached `NoData` having never been `Streaming` — the
    /// registered-but-dead route.
    Silent,
    /// A row with a learned baseline is measuring outside the octave band.
    RateDeviation,
}

impl MonitorCondition {
    /// Every condition, in the order rows report them (declaration order, so a
    /// rendered row is stable run to run).
    pub const ALL: [MonitorCondition; 3] = [
        MonitorCondition::Stalled,
        MonitorCondition::Silent,
        MonitorCondition::RateDeviation,
    ];

    /// The condition's WIRE spelling, for a `tracing` field or any other place a
    /// `&'static str` is needed.
    ///
    /// Exists so the daemon's log lines and its JSON say the SAME word. A log that
    /// spelled a condition differently from the wire would make an operator's grep
    /// disagree with the agent's payload about which fault fired — the
    /// two-copies-of-one-vocabulary class, in the one place a `Serialize` impl
    /// cannot reach. Pinned against the serde spelling by
    /// `the_wire_spelling_helper_agrees_with_serde`.
    pub const fn as_wire(&self) -> &'static str {
        match self {
            MonitorCondition::Stalled => "stalled",
            MonitorCondition::Silent => "silent",
            MonitorCondition::RateDeviation => "rate_deviation",
        }
    }
}

/// A row's overall verdict — what a sidebar colours and what an agent reads first.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MonitorState {
    /// Evidence is flowing and nothing is raised.
    Healthy,
    /// At least one condition is currently raised.
    Alerting,
    /// Evidence is flowing, nothing is raised, and the row could still learn a
    /// rate baseline it has not learned yet.
    ///
    /// A row that can NEVER learn one (its rate basis is untrusted) is
    /// [`Self::Healthy`] with an [`Ineligible`] entry saying what it is not being
    /// told — reporting `Learning` forever would be a claim that something is
    /// still in progress when nothing is.
    ///
    /// The retained stall basis removed the one shape where that claim was flatly FALSE: a row
    /// whose baseline was wiped by a clear and whose publisher then died could
    /// never learn again, yet reported `Learning` with `ineligible: []` — a
    /// positive statement that judging was still in progress on a topic nothing
    /// would ever judge. Such a row now reports on its retained stall basis
    /// instead (`alerting` when the stall confirms, or an explicit
    /// [`IneligibleReason::SlowTopic`] when the basis is below the gate).
    Learning,
    /// No usable evidence: the liveness payload is absent, classifies
    /// [`LivenessState::Unknown`], or discovery has not converged. NEVER conflate
    /// with [`Self::Healthy`].
    Unknown,
}

/// WHY a condition is not being evaluated for a row.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IneligibleReason {
    /// The row's learned rate is below [`MONITOR_STALL_MIN_MHZ`], or it streams
    /// repeatedly while the substrate declines to rate it at all — either way its
    /// `Streaming ↔ Idle` transitions are the recency window, not a fault.
    SlowTopic,
    /// The row has reported a FLOOR-basis rate at least once
    /// ([`TopicRateEstimate::is_floor`]), so its rate basis is a labelled lower
    /// bound rather than a measurement. The rate estimator gives a `multi_publisher_topics`
    /// row (`/tf`) exactly this, a class the lull-free-restart fix explicitly does not heal.
    UntrustedRateBasis,
    /// No usable liveness evidence has ever been admitted for this row, so the
    /// agent is being told nothing — not that the row is healthy.
    NoLivenessEvidence,
}

/// One condition this row is NOT being judged on, and why.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct Ineligible {
    /// The condition being withheld.
    pub condition: MonitorCondition,
    /// Why it is withheld.
    pub reason: IneligibleReason,
}

// ---------------------------------------------------------------------------
// The sample.
// ---------------------------------------------------------------------------

/// ONE observation of ONE topic, from either sample source.
///
/// Both sources produce the same shape on purpose (attached rows are fed
/// from vizd's own tap statistics, unattached rows from the
/// `discover` handler's catalog gather), so the engine has exactly one input
/// vocabulary and neither plane can grow its own classification.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MonitorSample {
    /// The absolute topic this observation is about.
    pub topic: String,
    /// The origin robot, or `None` for a genuine local producer (the attribution
    /// convention).
    pub robot: Option<String>,
    /// The SAMPLER's monotonic clock, in nanoseconds. Every duration the engine
    /// computes is a difference of two of these, so the origin is arbitrary as
    /// long as it is the same origin throughout one engine's life.
    pub observed_at_ns: u64,
    /// What the substrate observed, or `None` for UNKNOWN. `None` is never a
    /// condition (module rule 1).
    pub liveness: Option<TopicLiveness>,
    /// The rate to score, when there is one.
    ///
    /// Defaults to the liveness payload's own [`TopicLiveness::rate_estimate`]
    /// (built that way by [`MonitorSample::new`]), which is what an UNATTACHED row
    /// carries. An ATTACHED row overrides it with the desk's own per-tap
    /// measurement, which is the same quantity measured the same way over a
    /// different window.
    pub rate: Option<TopicRateEstimate>,
    /// Whether the discovery plane had converged when this observation was taken.
    /// `false` suppresses every condition (module rule 2).
    pub discovery_converged: bool,
}

impl MonitorSample {
    /// Build a sample whose rate is the liveness payload's own estimate.
    pub fn new(
        topic: impl Into<String>,
        robot: Option<String>,
        observed_at_ns: u64,
        liveness: Option<TopicLiveness>,
        discovery_converged: bool,
    ) -> Self {
        let rate = liveness.and_then(|l| l.rate_estimate);
        Self {
            topic: topic.into(),
            robot,
            observed_at_ns,
            liveness,
            rate,
            discovery_converged,
        }
    }

    /// Override the rate with a measurement taken elsewhere (the attached-row
    /// plane's own per-tap rate).
    #[must_use]
    pub fn with_rate(mut self, rate: Option<TopicRateEstimate>) -> Self {
        self.rate = rate;
        self
    }

    /// The classified verdict, or `None` when this sample carries no usable
    /// evidence.
    ///
    /// Collapses BOTH absolute evidence rules into one answer: an absent payload,
    /// an [`LivenessState::Unknown`] classification, and an unconverged discovery
    /// plane are indistinguishable to everything downstream, because all three
    /// mean the same thing — nothing was observed that anyone may act on.
    pub fn is_evidential(&self) -> Option<LivenessState> {
        if !self.discovery_converged {
            return None;
        }
        match self.liveness?.state() {
            LivenessState::Unknown => None,
            state => Some(state),
        }
    }
}

// ---------------------------------------------------------------------------
// Pure predicates.
// ---------------------------------------------------------------------------

/// Whether `observed` sits OUTSIDE the octave band around `baseline`.
///
/// Both comparisons are STRICT, so exactly `baseline / 2` and exactly
/// `baseline × 2` are INSIDE the band — a topic sitting precisely on an edge is
/// not paged. The low side is written as a multiplication
/// (`observed × BAND < baseline`) rather than a division so an odd baseline is not
/// silently floored into a slightly wider band, and the high side saturates so a
/// baseline near `u64::MAX` cannot wrap into a false alert.
pub fn rate_deviates(baseline_mhz: u64, observed_mhz: u64) -> bool {
    let low = observed_mhz.saturating_mul(MONITOR_RATE_BAND) < baseline_mhz;
    let high = observed_mhz > baseline_mhz.saturating_mul(MONITOR_RATE_BAND);
    low || high
}

/// The MEDIAN of a learning window — the frozen baseline.
///
/// Median rather than mean so ONE stalled or bursty window inside the learning run
/// cannot drag the baseline the whole band's worth. `None` for an empty window.
/// An even-length window takes the LOWER of the two middle samples, which is a
/// tie-break, not a rounding choice: it keeps the result an observed value rather
/// than an average of two the topic never ran at.
pub fn median_mhz(samples: &[u64]) -> Option<u64> {
    if samples.is_empty() {
        return None;
    }
    let mut sorted: Vec<u64> = samples.to_vec();
    sorted.sort_unstable();
    Some(sorted[(sorted.len() - 1) / 2])
}

/// A per-condition confirmation tracker — the hysteresis, as a pure state machine.
///
/// Its whole job is to make "confirmed" mean the same thing for every condition,
/// so no condition can grow its own private notion of how much evidence is enough.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ConditionTracker {
    raised: bool,
    raised_at_ns: u64,
    qualifying: u32,
    qualifying_since_ns: u64,
    clearing: u32,
}

/// What one [`ConditionTracker::observe`] did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackerTransition {
    /// Nothing changed.
    None,
    /// The condition just became raised.
    Raised,
    /// The condition just became cleared.
    Cleared,
}

impl ConditionTracker {
    /// Feed ONE evidential sample.
    ///
    /// `qualifies` is the condition's own predicate for this sample; `now_ns` is
    /// the sample's timestamp. A sample that is NOT evidential must go to
    /// [`Self::observe_blind`] instead — the two are different, and conflating them
    /// is what would let a lost observation clear a live alert.
    pub fn observe(&mut self, qualifies: bool, now_ns: u64) -> TrackerTransition {
        if self.raised {
            if qualifies {
                self.clearing = 0;
                return TrackerTransition::None;
            }
            self.clearing += 1;
            if self.clearing >= MONITOR_CLEAR_SAMPLES {
                *self = Self::default();
                return TrackerTransition::Cleared;
            }
            return TrackerTransition::None;
        }
        if !qualifies {
            self.qualifying = 0;
            self.qualifying_since_ns = 0;
            return TrackerTransition::None;
        }
        if self.qualifying == 0 {
            self.qualifying_since_ns = now_ns;
        }
        self.qualifying += 1;
        let long_enough =
            now_ns.saturating_sub(self.qualifying_since_ns) >= MONITOR_CONFIRM_MIN_SPAN_NS;
        if self.qualifying >= MONITOR_CONFIRM_SAMPLES && long_enough {
            self.raised = true;
            self.raised_at_ns = now_ns;
            self.qualifying = 0;
            self.qualifying_since_ns = 0;
            self.clearing = 0;
            return TrackerTransition::Raised;
        }
        TrackerTransition::None
    }

    /// Feed a sample that carried NO usable evidence.
    ///
    /// ASYMMETRIC on purpose: it breaks a not-yet-confirmed qualifying run (the
    /// chain of evidence a confirmation is built from really was broken), and it
    /// leaves a RAISED condition exactly as it was (losing sight of a topic is not
    /// an all-clear). Both halves are the conservative direction.
    pub fn observe_blind(&mut self) {
        if !self.raised {
            self.qualifying = 0;
            self.qualifying_since_ns = 0;
        }
    }

    /// Whether the condition is currently raised.
    pub const fn is_raised(&self) -> bool {
        self.raised
    }

    /// When it was raised (the sampler clock), meaningless unless
    /// [`Self::is_raised`].
    pub const fn raised_at_ns(&self) -> u64 {
        self.raised_at_ns
    }
}

// ---------------------------------------------------------------------------
// Eligibility (D7).
// ---------------------------------------------------------------------------

/// Whether a condition is being evaluated for a row right now, and — when it is
/// not — whether that is PERMANENT (so the agent must be told) or merely not-yet.
///
/// The distinction is the whole point. "Withheld forever because this row's rate
/// basis cannot be trusted" is something an agent has to know, or it reads silence
/// as health. "Not judged yet because the baseline is still being learned" is
/// ordinary progress and reporting it as ineligible would cry wolf on every fresh
/// row.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConditionGate {
    /// The condition is being evaluated on every evidential sample.
    Open,
    /// The condition is NOT being evaluated, and this is why — reported to the
    /// agent on the row.
    Closed(IneligibleReason),
    /// The condition is not being evaluated YET (the row is still learning). Not
    /// reported as ineligible: nothing is being permanently withheld.
    ///
    /// "Yet" is a promise, so the gate may only answer this while learning is
    /// genuinely still possible. The stall gate reaches it only for a
    /// row that has NEVER frozen a baseline — once one has been frozen the basis
    /// survives every later wipe, so a row that can no longer learn (its
    /// publisher is gone) is judged on what it earned rather than told a
    /// progress story nothing can finish.
    Learning,
}

impl ConditionGate {
    /// Whether the condition may qualify on this sample.
    pub const fn is_open(&self) -> bool {
        matches!(self, ConditionGate::Open)
    }

    /// The reason to report, if this gate withholds the condition permanently.
    pub const fn reason(&self) -> Option<IneligibleReason> {
        match self {
            ConditionGate::Closed(reason) => Some(*reason),
            _ => None,
        }
    }
}

// ---------------------------------------------------------------------------
// The reported row + alert (also the WIRE types; `cerulion-vizd`'s protocol
// re-exports them rather than declaring a second copy, the one-copy rule).
// ---------------------------------------------------------------------------

/// One watched topic's current, STICKY state — always readable (Principle #3),
/// whether or not anybody was listening when it changed.
///
/// Every optional field is `#[serde(default, skip_serializing_if)]` with `None`
/// meaning UNKNOWN, so nothing here defaults to a positive claim and no protocol
/// version bump is implied on either daemon.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MonitorRow {
    /// The absolute topic.
    pub topic: String,
    /// The origin robot; ABSENT means a genuine local producer.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub robot: Option<String>,
    /// The row's verdict.
    pub state: MonitorState,
    /// The currently-raised conditions, in [`MonitorCondition::ALL`] order.
    ///
    /// ALWAYS serialized, empty array included. This is the field an agent acts
    /// on, and `[]` is a positive "we looked and nothing is raised" — materially
    /// different from a key that is absent because the daemon had nothing to say.
    /// (Contrast [`Self::ineligible`], where absent genuinely means "nothing is
    /// being withheld".)
    pub conditions: Vec<MonitorCondition>,
    /// The FROZEN learned RATE baseline in millihertz, absent while learning.
    ///
    /// This is the number the octave band is scored against, and nothing else.
    /// It is ABSENT after the re-learn wipe — including on a row that is
    /// simultaneously `alerting` with `conditions: ["stalled"]`, a combination
    /// the retained stall basis made reachable: that verdict was judged against
    /// that basis, which rides the ALERT
    /// ([`Alert::baseline_mhz`]) rather than the row. A reader wanting the number
    /// a stall rested on must read it there; reading it here and finding nothing
    /// means the rate band is re-learning, never that the verdict was baseless.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub baseline_mhz: Option<u64>,
    /// The last evidential sample's rate, absent when it carried none.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub observed_mhz: Option<u64>,
    /// Whether [`Self::observed_mhz`] was a labelled FLOOR rather than a
    /// measurement. Absent exactly when `observed_mhz` is.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub observed_is_floor: Option<bool>,
    /// The substrate's own classification of the last evidential sample — the
    /// same closed set every other Cerulion surface uses. Absent for UNKNOWN,
    /// which is the ONE wire encoding of unknown everywhere in this repo.
    #[serde(default, skip_serializing_if = "liveness_state_absent_on_the_wire")]
    pub liveness_state: Option<LivenessState>,
    /// How many EVIDENTIAL samples this row has contributed.
    ///
    /// Samples the engine looked at and learned nothing from (an absent payload,
    /// an `Unknown` classification, an unconverged discovery plane, or anything
    /// inside the settle window) are NOT counted. Paired with
    /// [`Self::last_sample_age_ms`] this is the freshness statement at its
    /// sharpest: `samples: 0` beside `last_sample_age_ms: 0` reads exactly
    /// "we are polling this row and have learned nothing from it".
    pub samples: u64,
    /// How stale this row's evidence is: milliseconds since the last sample was
    /// TAKEN (evidential or not — this is about the freshness of the FEED).
    ///
    /// The freshness field. Coverage of an unattached row is only as current as
    /// the last `discover` the sidebar made, so the agent is told how fresh the
    /// evidence is and can never read silence as health.
    pub last_sample_age_ms: u64,
    /// Conditions this row is NOT being judged on, and why. ABSENT means nothing
    /// is being withheld.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub ineligible: Vec<Ineligible>,
}

/// Keeps ABSENCE the one wire encoding of UNKNOWN (the `cerulion-vizd` protocol's
/// own rule, applied to the type that crosses this boundary).
fn liveness_state_absent_on_the_wire(state: &Option<LivenessState>) -> bool {
    matches!(state, None | Some(LivenessState::Unknown))
}

/// ONE alert transition — a RAISE or a CLEAR. (The design memo calls this an
/// `AlertEvent`; it is the same type the `monitors` verb serves, so it carries one
/// name.)
///
/// A CLEAR is a NEW event with a NEW [`Self::seq`] carrying
/// [`Self::cleared_at_ms`], never a mutation of the raise already in the ring.
/// That is forced by the retention contract: an agent polls, remembers the highest
/// `seq` it has seen, and dedupes client-side — so an in-place mutation of an
/// entry it already read is a change it can never learn about.
///
/// # Clock domain
///
/// `raised_at_ms` / `cleared_at_ms` are the SAMPLER's monotonic clock in
/// milliseconds, NOT a wall-clock epoch and NOT a Unix timestamp. They are
/// comparable to each other (`cleared - raised` is a real duration) and to other
/// alerts from the same daemon; they are meaningless across daemons and must never
/// be rendered as a time of day.
///
/// [`Self::age_ms`] is the RENDERABLE quantity built from them — see its own
/// note. The stamps stay on the wire because ordering and client-side dedupe
/// need them; the age is what a reader can put on screen.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Alert {
    /// Monotonic across this engine's life, over raises AND clears.
    pub seq: u64,
    /// The absolute topic.
    pub topic: String,
    /// The origin robot; ABSENT means a genuine local producer.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub robot: Option<String>,
    /// Which condition.
    pub condition: MonitorCondition,
    /// When the condition was RAISED (see the type's clock-domain note).
    pub raised_at_ms: u64,
    /// When it was CLEARED — present exactly on a clear event, and the
    /// discriminator between the two kinds.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cleared_at_ms: Option<u64>,
    /// How long ago the TRANSITION this entry records happened, stamped by the
    /// SERVING daemon at serve time on its own monotonic clock: `serve_now -
    /// raised_at_ms` for a raise entry, `serve_now - cleared_at_ms` for a clear
    /// one. This is what a reader renders — "stalled 40 s ago" — with no
    /// cross-clock arithmetic of its own, which is the whole point: the stamps
    /// beside it belong to a clock the reader does not share.
    ///
    /// Meaningful only WITHIN ONE RESPONSE — it is a snapshot, not a ticking
    /// value. A client that wants it to tick adds its own time since the
    /// response arrived, which is client-local arithmetic on a client-local
    /// clock and therefore sound; adding it to `raised_at_ms` would not be.
    ///
    /// ABSENT means UNKNOWN, never zero — the same rule every optional field on
    /// this wire follows. A `0` is a positive claim that the transition happened
    /// just now.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub age_ms: Option<u64>,
    /// The rate observed at the transition, when there was one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub observed_mhz: Option<u64>,
    /// The baseline the row was being judged against, when it had one.
    ///
    /// Read BEFORE the re-learn wipe, so a CLEAR names the baseline the alert it
    /// ends was scored against rather than the absence the wipe leaves behind.
    ///
    /// For a [`MonitorCondition::Stalled`] this may be the row's LAST FROZEN
    /// baseline rather than a live one: a stall confirmed after a wipe
    /// is judged on the basis the row EARNED at its last freeze, and that is the
    /// number this field carries — which is why an `alerting` row can serve
    /// `conditions: ["stalled"]` with a `baseline_mhz` of its own that is ABSENT
    /// while the alert's is present. The two fields answer different questions:
    /// [`MonitorRow::baseline_mhz`] is the live RATE baseline, this one is what
    /// the verdict rested on.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub baseline_mhz: Option<u64>,
    /// The substrate's classification at the transition.
    #[serde(default, skip_serializing_if = "liveness_state_absent_on_the_wire")]
    pub liveness_state: Option<LivenessState>,
}

impl Alert {
    /// Stamp [`Self::age_ms`] for a response being served at `now_ms`.
    ///
    /// The rule, in ONE place, so the serving handler is a single call and
    /// cannot arrive at a second answer: the age is measured from the
    /// TRANSITION this entry records — its `cleared_at_ms` when it is a clear,
    /// its `raised_at_ms` when it is a raise — on the SAME monotonic clock those
    /// stamps came from. `now_ms` is an ARGUMENT, never read here: the engine
    /// holds no clock (see the module docs), and the serving daemon is the only
    /// thing that knows when the response is being built.
    ///
    /// `now_ms` BELOW the stamp SATURATES to `Some(0)` rather than wrapping. It
    /// is reachable — the ring is served from a snapshot taken microseconds
    /// earlier, and a caller passing a stale `now_ms` (or a clock whose
    /// resolution rounds the wrong way) would otherwise render a `u64::MAX`-ish
    /// age on a transition that just happened. Zero is the correct answer there:
    /// no time has elapsed that this clock can see.
    #[must_use]
    pub const fn with_age(mut self, now_ms: u64) -> Self {
        let transition_at_ms = match self.cleared_at_ms {
            Some(cleared) => cleared,
            None => self.raised_at_ms,
        };
        self.age_ms = Some(now_ms.saturating_sub(transition_at_ms));
        self
    }
}

// ---------------------------------------------------------------------------
// The engine.
// ---------------------------------------------------------------------------

/// The identity of ONE watched row: a topic **on a robot**.
///
/// Keyed by BOTH, because one desk watches many robots at once and `/lowstate` on
/// robot A is a different stream from `/lowstate` on robot B — the whole reason
/// the attribution machinery exists. Merged under the topic name alone,
/// two robots' samples interleave into ONE baseline, ONE hysteresis run and ONE
/// alert history: mixed baselines, rate deviations scored against another robot's
/// rate, and alerts attributed to whichever robot was sampled last.
///
/// ABSENT and PRESENT are two DISTINCT keys and are never conflated: `robot: None`
/// is a genuine LOCAL producer (the attribution convention), which is a different data
/// source from any remote robot's topic of the same name. That is also what keeps
/// a desk producer reusing a detached remote topic's NAME from inheriting the
/// historical robot's learned state — it gets its own row, settle window included.
///
/// Ordered by TOPIC first so [`MonitorEngine::rows`] still reads as a topic list;
/// same-named topics sit adjacent with the local one first (`None < Some`).
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
struct RowKey {
    topic: String,
    robot: Option<String>,
}

impl RowKey {
    fn new(topic: &str, robot: Option<&str>) -> Self {
        Self {
            topic: topic.to_string(),
            robot: robot.map(str::to_string),
        }
    }
}

/// One watched row's accumulated state. Its IDENTITY lives in the [`RowKey`] it is
/// stored under, so nothing here can be re-attributed in place.
#[derive(Debug, Clone, PartialEq, Eq)]
struct RowState {
    first_seen_ns: u64,
    last_sample_ns: u64,
    samples: u64,
    last_state: Option<LivenessState>,
    last_rate: Option<TopicRateEstimate>,
    ever_streaming: bool,
    ever_floor: bool,
    baseline_mhz: Option<u64>,
    /// The last baseline this row ever FROZE — the D7 stall basis.
    ///
    /// Written at exactly one site (the freeze in
    /// [`MonitorEngine::advance_learning`]), superseded only by the next freeze,
    /// and dropped only with the row. Deliberately NOT touched by the re-learn wipe, by a
    /// blind sample, or by any non-`Streaming`/floor/unrated sample: it records
    /// something the row EARNED, and the wipe's stated purpose ("a restart may
    /// legitimately change the rate") is about the RATE band.
    ///
    /// INVARIANT, and the reason [`Alert::baseline_mhz`]'s `.or()` is a
    /// fall-through rather than a choice: `baseline_mhz.is_some()` implies
    /// `last_frozen_mhz == baseline_mhz`, because the only site that sets a
    /// baseline sets both. So the fall-through can only ever ADD a value where
    /// there was none.
    last_frozen_mhz: Option<u64>,
    learn_window: Vec<u64>,
    streaming_without_rate: u32,
    trackers: [ConditionTracker; MonitorCondition::ALL.len()],
}

impl RowState {
    fn new(now_ns: u64) -> Self {
        Self {
            first_seen_ns: now_ns,
            last_sample_ns: now_ns,
            samples: 0,
            last_state: None,
            last_rate: None,
            ever_streaming: false,
            ever_floor: false,
            baseline_mhz: None,
            last_frozen_mhz: None,
            learn_window: Vec::new(),
            streaming_without_rate: 0,
            trackers: [ConditionTracker::default(); MonitorCondition::ALL.len()],
        }
    }

    fn tracker(&mut self, condition: MonitorCondition) -> &mut ConditionTracker {
        &mut self.trackers[condition as usize]
    }

    /// The STALL gate.
    ///
    /// A stall is judged only on a row whose rate the substrate measured
    /// trustworthily AND fast enough that `Streaming <-> Idle` is a fault rather
    /// than the recency window doing its job. Both of the gate's clauses collapse into
    /// "the learned baseline is at or above [`MONITOR_STALL_MIN_MHZ`]", plus the
    /// one shape a baseline test alone cannot reach: a topic the substrate keeps
    /// classifying `Streaming` while declining to rate it at all is, by the rate estimator's
    /// own horizon, slower than a rate can be measured for — which is precisely
    /// what D7 excludes, and it is OBSERVED here rather than assumed.
    ///
    /// # The fall-through to the retained basis
    ///
    /// D7 asks whether the row was EVER measured trustworthy and fast, and a
    /// freeze is what answers that. The re-learn wipe answers a different question (may
    /// the RATE band re-learn?), so with no live baseline the gate falls through
    /// to [`RowState::last_frozen_mhz`] rather than parking in
    /// [`ConditionGate::Learning`] — which, on a row whose publisher has DIED, is
    /// a park it can never leave.
    ///
    /// # Precedence is load-bearing, in both directions
    ///
    /// * `ever_floor` stays FIRST: a poisoned basis is permanent, and a basis
    ///   frozen before the first floor sample must not launder it.
    /// * the unrated slow-topic COUNTER stays AHEAD of the basis: a row that
    ///   restarts as a topic the substrate declines to rate is slow by the rate estimator's
    ///   own horizon, and closing on that evidence before its first `Idle`
    ///   stretch is what keeps a fast→slow restart from flapping a stall against
    ///   a basis that describes the OLD publisher. The bound is cadence-dependent
    ///   and pinned as such (see
    ///   `a_slow_cadence_fast_to_slow_transition_is_bounded_by_the_counter_at_the_boundary`).
    /// * only then the basis, with the SAME threshold test a live baseline gets:
    ///   a row that last froze BELOW the gate reads `slow_topic`, not `learning`.
    fn stall_gate(&self) -> ConditionGate {
        if self.ever_floor {
            return ConditionGate::Closed(IneligibleReason::UntrustedRateBasis);
        }
        match self.baseline_mhz {
            Some(mhz) if mhz >= MONITOR_STALL_MIN_MHZ => ConditionGate::Open,
            Some(_) => ConditionGate::Closed(IneligibleReason::SlowTopic),
            None if self.streaming_without_rate >= MONITOR_LEARN_SAMPLES => {
                ConditionGate::Closed(IneligibleReason::SlowTopic)
            }
            None => match self.last_frozen_mhz {
                Some(mhz) if mhz >= MONITOR_STALL_MIN_MHZ => ConditionGate::Open,
                Some(_) => ConditionGate::Closed(IneligibleReason::SlowTopic),
                // Nothing was ever earned, so "not yet" is the correct answer.
                None => ConditionGate::Learning,
            },
        }
    }

    /// The RATE-DEVIATION gate.
    ///
    /// Closed only on an untrusted basis: baseline learning and the band BOTH
    /// require `is_floor == false`, and the rate estimator's floor ceiling means a labelled
    /// floor of 10 Hz may be a 500 Hz topic — scoring that against a band would
    /// page on healthy fast topics. A SLOW topic is deliberately NOT excluded
    /// here: a slow topic with a trustworthy baseline can still deviate from it.
    fn rate_gate(&self) -> ConditionGate {
        if self.ever_floor {
            return ConditionGate::Closed(IneligibleReason::UntrustedRateBasis);
        }
        match self.baseline_mhz {
            Some(_) => ConditionGate::Open,
            None => ConditionGate::Learning,
        }
    }

    /// The SILENT gate — always open once there is evidence. A row ineligible for
    /// everything else still gets told when it is a registered-but-dead route,
    /// which is the condition that needs no rate at all.
    fn silent_gate(&self) -> ConditionGate {
        ConditionGate::Open
    }

    fn gate(&self, condition: MonitorCondition) -> ConditionGate {
        match condition {
            MonitorCondition::Stalled => self.stall_gate(),
            MonitorCondition::Silent => self.silent_gate(),
            MonitorCondition::RateDeviation => self.rate_gate(),
        }
    }
}

/// The standing watchdog over every row it has been shown.
///
/// PURE: no clock, no transport, no I/O. It advances only when
/// [`MonitorEngine::observe`] is called, and every decision is a function of the
/// samples it has been handed — so the same sample vector yields byte-identical
/// alerts, `seq` numbering included.
#[derive(Debug, Clone, Default)]
pub struct MonitorEngine {
    rows: std::collections::BTreeMap<RowKey, RowState>,
    next_seq: u64,
}

impl MonitorEngine {
    /// A fresh engine watching nothing.
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed ONE observation; returns every alert TRANSITION it caused, in
    /// [`MonitorCondition::ALL`] order.
    ///
    /// At most one transition per condition, so at most three events — a raise of
    /// one condition and a clear of another can legitimately land on one sample.
    pub fn observe(&mut self, sample: &MonitorSample) -> Vec<Alert> {
        let now = sample.observed_at_ns;
        // Rows are keyed by (topic, robot) — see `RowKey`. A sample naming a
        // DIFFERENT robot for the same topic name is a different data source, so
        // it opens its own row rather than re-attributing this one in place.
        let row = self
            .rows
            .entry(RowKey::new(&sample.topic, sample.robot.as_deref()))
            .or_insert_with(|| RowState::new(now));
        // The FEED's freshness advances even for a sample we discard — that is
        // exactly what makes `samples: 0` beside a small age readable.
        row.last_sample_ns = now;

        // The settle window: the whole sample is withheld (see the module docs).
        if now.saturating_sub(row.first_seen_ns) < MONITOR_SETTLE_NS {
            return Vec::new();
        }

        let Some(state) = sample.is_evidential() else {
            // Nothing was observed. Break every pending run, clear nothing, and
            // report no current classification or rate: none is available.
            for tracker in row.trackers.iter_mut() {
                tracker.observe_blind();
            }
            // The baseline is the median of CONSECUTIVE trustworthy `Streaming`
            // rates, and "consecutive" is load-bearing: a blind sample is a HOLE in
            // the observation stream, and behind that hole the topic may have gone
            // `Idle`, changed rate, or been restarted by a publisher whose new rate
            // has nothing to do with the old one. Medianing across it stitches
            // non-adjacent windows into a number that describes no run the topic
            // ever had — and freezes it, since D4 re-learns only after an alert
            // clears. A stale pre-gap rate frozen as the baseline then pages on the
            // rate the topic is ACTUALLY running at.
            //
            // Only the learning RUN is broken. `streaming_without_rate` is
            // deliberately CUMULATIVE (see `advance_learning`) and must survive —
            // the slow-topic class it exists for is interrupted by exactly this
            // kind of gap — and a baseline already FROZEN is kept, because losing
            // sight of a topic is neither of the re-learn wipe's two triggers: an alert
            // CLEARING (the wipe below) and `forget` dropping the row entirely.
            // The stall basis (`last_frozen_mhz`) survives a gap for the same
            // reason, and survives the wipe as well.
            row.learn_window.clear();
            row.last_state = None;
            row.last_rate = None;
            return Vec::new();
        };

        row.samples += 1;
        row.last_state = Some(state);
        row.last_rate = sample.rate;
        if sample.rate.is_some_and(|r| r.is_floor) {
            // PERMANENT (D7): a row that has ever been served a labelled floor has
            // a rate basis nothing later can retroactively make trustworthy.
            row.ever_floor = true;
        }
        if state == LivenessState::Streaming {
            row.ever_streaming = true;
        }

        Self::advance_learning(row, state, sample.rate);

        let baseline_at_transition = row.baseline_mhz;
        // Read BESIDE the live baseline and BEFORE the wipe, for the same reason:
        // an alert names what it was judged against at the instant of the
        // transition, and a stall confirmed on a wiped row was judged against the
        // basis.
        let last_frozen_at_transition = row.last_frozen_mhz;
        let mut events = Vec::new();
        let mut cleared_any = false;
        for condition in MonitorCondition::ALL {
            let gate = row.gate(condition);
            let qualifies = gate.is_open() && Self::qualifies(row, condition, state, sample.rate);
            let raised_at_ns = row.tracker(condition).raised_at_ns();
            match row.tracker(condition).observe(qualifies, now) {
                TrackerTransition::None => {}
                TrackerTransition::Raised => {
                    let at = row.tracker(condition).raised_at_ns();
                    events.push((condition, at, None));
                }
                TrackerTransition::Cleared => {
                    cleared_any = true;
                    events.push((condition, raised_at_ns, Some(now)));
                }
            }
        }

        if cleared_any {
            // D4: an adaptive baseline silently absorbs the degradation it exists
            // to detect, so the baseline is FROZEN — and re-learned only here,
            // after a raised alert clears, because a restart may legitimately
            // change the rate.
            //
            // What this re-learns is the RATE band, and ONLY the rate band.
            // Stall eligibility was EARNED by a freeze, and
            // `last_frozen_mhz` deliberately survives: a restart changing the
            // rate says nothing about whether this row was ever measured fast
            // enough for `Streaming -> Idle` to be a fault, and a publisher that
            // DIED cannot supply the samples a re-learn needs — so wiping the
            // evidence here used to park the stall gate in `Learning` forever on
            // the one row that most needed paging.
            row.baseline_mhz = None;
            row.learn_window.clear();
        }

        let robot = sample.robot.clone();
        let observed_mhz = sample.rate.map(|r| r.millihertz);
        events
            .into_iter()
            .map(|(condition, raised_at_ns, cleared_at_ns)| {
                let seq = self.next_seq;
                self.next_seq += 1;
                Alert {
                    seq,
                    topic: sample.topic.clone(),
                    robot: robot.clone(),
                    condition,
                    raised_at_ms: raised_at_ns / 1_000_000,
                    cleared_at_ms: cleared_at_ns.map(|ns| ns / 1_000_000),
                    // UNKNOWN at MINT time, and deliberately so: an age is a
                    // statement about how long ago this happened relative to
                    // SERVE time, which is a different instant the engine has no
                    // way to know. The serving handler stamps it with
                    // `Alert::with_age`.
                    age_ms: None,
                    observed_mhz,
                    // The live baseline when there is one, else the basis the
                    // verdict actually rested on. By the invariant on
                    // `RowState::last_frozen_mhz` the two agree whenever both are
                    // present, so this can only ADD a number where the wipe left
                    // none — and it is reachable ONLY for a stall, since a
                    // `rate_deviation` needs a live baseline to score at all and a
                    // `silent` row was never `Streaming` and so never froze one.
                    baseline_mhz: baseline_at_transition.or(last_frozen_at_transition),
                    liveness_state: Some(state),
                }
            })
            .collect()
    }

    /// Advance (or break) the baseline-learning run for one evidential sample.
    ///
    /// The run must be CONSECUTIVE `Streaming` samples carrying a trustworthy
    /// rate: anything else — a non-`Streaming` classification, a labelled floor,
    /// or no rate at all — breaks it, because a baseline stitched together from
    /// non-adjacent windows is not a measurement of anything.
    fn advance_learning(row: &mut RowState, state: LivenessState, rate: Option<TopicRateEstimate>) {
        if state != LivenessState::Streaming {
            row.learn_window.clear();
            return;
        }
        match rate {
            Some(rate) if !rate.is_floor => {
                row.streaming_without_rate = 0;
                if row.baseline_mhz.is_none() {
                    row.learn_window.push(rate.millihertz);
                    if row.learn_window.len() >= MONITOR_LEARN_SAMPLES as usize {
                        row.baseline_mhz = median_mhz(&row.learn_window);
                        // THE one site that writes the stall basis. A
                        // freeze is what earns D7 eligibility, so the basis is
                        // written where eligibility is earned and nowhere else —
                        // which is what makes "it survives the wipe" a property
                        // of the code's shape rather than of a list of places
                        // remembering not to clear it. Superseded here too: a
                        // re-learned SLOW baseline replaces a fast one, because
                        // the last freeze is what describes the row now.
                        row.last_frozen_mhz = row.baseline_mhz;
                        row.learn_window.clear();
                    }
                }
            }
            Some(_floor) => {
                row.learn_window.clear();
                row.streaming_without_rate = 0;
            }
            None => {
                // Streaming and UNRATED: the rate estimator's own uncomfortable band. Counted
                // so the slow-topic gate is reachable on a topic no baseline can
                // ever describe.
                //
                // CUMULATIVE, deliberately not a consecutive run. A topic slower
                // than the recency window OSCILLATES `Streaming <-> Idle` by
                // design — that is the very shape D7 excludes — so a run reset by
                // every `Idle` sample could never reach the threshold on the one
                // class the counter exists for. What it measures is "we have seen
                // this row Streaming this many times and never once been given a
                // rate for it", which is exactly the signature.
                //
                // It is reset ONLY by a trustworthy rate arriving (below), which
                // is proof the substrate CAN rate the row, so a fast topic whose
                // first rate window has not closed yet self-heals rather than
                // being permanently misclassified.
                row.learn_window.clear();
                row.streaming_without_rate = row.streaming_without_rate.saturating_add(1);
            }
        }
    }

    /// One condition's predicate for one evidential sample. The GATE is applied by
    /// the caller, so this is only the condition itself.
    fn qualifies(
        row: &RowState,
        condition: MonitorCondition,
        state: LivenessState,
        rate: Option<TopicRateEstimate>,
    ) -> bool {
        match condition {
            // Watched as a TRANSITION FROM Streaming, which is what keeps the
            // undatable-Idle class out of it for free.
            MonitorCondition::Stalled => {
                row.ever_streaming && matches!(state, LivenessState::Idle | LivenessState::NoData)
            }
            MonitorCondition::Silent => !row.ever_streaming && state == LivenessState::NoData,
            MonitorCondition::RateDeviation => match (row.baseline_mhz, rate) {
                // A PRESENT, trustworthy rate is the only thing that can score.
                // `None` clears (by not qualifying) and never scores: a
                // stopped stream serves no rate, and reading that as 0 Hz would be
                // a confident false alert.
                (Some(baseline), Some(rate)) if !rate.is_floor => {
                    rate_deviates(baseline, rate.millihertz)
                }
                _ => false,
            },
        }
    }

    /// Every watched row's CURRENT state, sorted by topic then by robot.
    ///
    /// One topic NAME can legitimately appear more than once — once per robot
    /// serving it, plus once for a genuine local producer — and those rows are
    /// independent in every respect. They sort adjacent, local first.
    ///
    /// `now_ns` is only used to age [`MonitorRow::last_sample_age_ms`]; it changes
    /// no verdict, so rendering twice at different instants cannot alter what the
    /// engine believes.
    pub fn rows(&self, now_ns: u64) -> Vec<MonitorRow> {
        self.rows
            .iter()
            .map(|(key, row)| {
                let conditions: Vec<MonitorCondition> = MonitorCondition::ALL
                    .into_iter()
                    .filter(|c| row.trackers[*c as usize].is_raised())
                    .collect();
                let ineligible: Vec<Ineligible> = if row.samples == 0 {
                    // Nothing has ever been admitted for this row, so NOTHING is
                    // being judged — say so per condition rather than let an empty
                    // `conditions` read as health.
                    MonitorCondition::ALL
                        .into_iter()
                        .map(|condition| Ineligible {
                            condition,
                            reason: IneligibleReason::NoLivenessEvidence,
                        })
                        .collect()
                } else {
                    MonitorCondition::ALL
                        .into_iter()
                        .filter_map(|condition| {
                            row.gate(condition)
                                .reason()
                                .map(|reason| Ineligible { condition, reason })
                        })
                        .collect()
                };
                let learning = row.samples > 0
                    && MonitorCondition::ALL
                        .into_iter()
                        .any(|c| matches!(row.gate(c), ConditionGate::Learning));
                let state = if !conditions.is_empty() {
                    MonitorState::Alerting
                } else if row.samples == 0 || row.last_state.is_none() {
                    MonitorState::Unknown
                } else if learning {
                    MonitorState::Learning
                } else {
                    MonitorState::Healthy
                };
                MonitorRow {
                    topic: key.topic.clone(),
                    robot: key.robot.clone(),
                    state,
                    conditions,
                    baseline_mhz: row.baseline_mhz,
                    observed_mhz: row.last_rate.map(|r| r.millihertz),
                    observed_is_floor: row.last_rate.map(|r| r.is_floor),
                    liveness_state: row.last_state,
                    samples: row.samples,
                    last_sample_age_ms: now_ns.saturating_sub(row.last_sample_ns) / 1_000_000,
                    ineligible,
                }
            })
            .collect()
    }

    /// The next `seq` this engine will assign — the ring's high-water mark
    /// (Principle #3: observable without reading an alert).
    pub const fn next_seq(&self) -> u64 {
        self.next_seq
    }

    /// How many rows are being watched.
    pub fn watched(&self) -> usize {
        self.rows.len()
    }

    /// Stop watching ONE row — a topic on a robot — and discard everything learned
    /// about it.
    ///
    /// The engine holds one entry per row it has ever been shown, so a caller that
    /// watches a changing topic set (a robot going away, a tap detaching) must be
    /// able to release one — otherwise a long-lived daemon accumulates rows for
    /// topics that no longer exist and reports them as UNKNOWN forever.
    ///
    /// `robot` is REQUIRED, and `None` means the LOCAL row rather than "any robot's":
    /// rows are keyed by `(topic, robot)`, so forgetting `/lowstate` without saying
    /// whose would either drop a robot that is still streaming or leave the one that
    /// went away behind. A caller releasing a whole robot forgets its topics one by
    /// one.
    ///
    /// Returns whether the row existed. Re-showing it later starts a fresh row,
    /// settle window included, which is the correct reading: nothing learned about
    /// the old producer describes the new one — the retained stall basis
    /// included, which is why it lives on the row's own state rather
    /// than in a map beside the rows.
    pub fn forget(&mut self, robot: Option<&str>, topic: &str) -> bool {
        self.rows.remove(&RowKey::new(topic, robot)).is_some()
    }
}

#[cfg(test)]
mod vocabulary_tests {
    use super::*;

    /// The derived constants land on the numbers the design memo states. A hand
    /// oracle, so a substrate constant moving under us is a LOUD failure here
    /// rather than a silent change of behaviour on a robot.
    #[test]
    fn the_constants_are_derived_from_the_substrate_and_land_where_the_memo_says() {
        assert_eq!(MONITOR_SAMPLE_INTERVAL_NS, 400_000_000, "2 x the sweep");
        assert_eq!(MONITOR_CONFIRM_MIN_SPAN_NS, 1_200_000_000, "1s + 200ms");
        assert_eq!(MONITOR_STALL_MIN_MHZ, 400, "0.4 Hz");
        assert_eq!(MONITOR_SETTLE_NS, 5_000_000_000, "the recency window");
        // The relationships, asserted against the substrate rather than restated:
        // if the substrate's healing span moves, the confirmation span moves WITH it.
        assert_eq!(
            MONITOR_CONFIRM_MIN_SPAN_NS,
            SUSTAINED_REGRESSION_MIN_SPAN_NS + LIVENESS_SWEEP_INTERVAL_NS
        );
        assert_eq!(MONITOR_SAMPLE_INTERVAL_NS, 2 * LIVENESS_SWEEP_INTERVAL_NS);
        assert_eq!(MONITOR_SETTLE_NS, LIVENESS_STREAMING_RECENCY_MS * 1_000_000);
    }

    /// [`MonitorCondition::as_wire`] and the serde spelling are the SAME word, for
    /// every variant.
    ///
    /// The helper exists because a `tracing` field needs a `&'static str` while the
    /// wire needs `Serialize`, so the vocabulary is rendered twice — and two
    /// renderings of one vocabulary is exactly how an operator's grep comes to
    /// disagree with the agent's payload. Driven over
    /// [`MonitorCondition::ALL`] rather than a hand list, so a fourth condition
    /// cannot be added with a helper arm nobody checked; the serde side is read out
    /// of a real serialization (with its JSON quotes stripped) rather than
    /// re-spelled here, which is what makes this a cross-check instead of a second
    /// hand copy.
    #[test]
    fn the_wire_spelling_helper_agrees_with_serde() {
        for condition in MonitorCondition::ALL {
            let json = serde_json::to_string(&condition).expect("serializes");
            assert_eq!(
                json,
                format!("\"{}\"", condition.as_wire()),
                "{condition:?} spells differently on the wire than in a log field"
            );
        }
        // Anti-tautology: the loop above is satisfied by an EMPTY `ALL`, and by a
        // helper that returned the Debug spelling for a one-word variant. Pin one
        // multi-word answer literally.
        assert_eq!(MonitorCondition::ALL.len(), 3);
        assert_eq!(MonitorCondition::RateDeviation.as_wire(), "rate_deviation");
    }

    /// A topic AT the stall gate has a period of exactly half the recency window —
    /// the point below which `Streaming <-> Idle` oscillation is the substrate
    /// telling the truth. Pinned as arithmetic so the gate's own justification
    /// cannot rot.
    #[test]
    fn the_stall_gate_sits_where_the_recency_window_stops_oscillating() {
        // 400 mHz => a 2.5 s period => half of the 5 s recency window.
        let period_ms = 1_000 * 1_000 / MONITOR_STALL_MIN_MHZ;
        assert_eq!(period_ms, LIVENESS_STREAMING_RECENCY_MS / 2);
    }

    /// The band is an OCTAVE and it is pinned on BOTH edges: exactly half and
    /// exactly double are INSIDE (not deviating), one millihertz past either edge
    /// is outside. A `<`/`<=` slip fails one side, so both sides are needed.
    #[test]
    fn the_rate_band_is_an_octave_pinned_on_both_edges() {
        const R: u64 = 20_000; // 20 Hz
        assert!(
            !rate_deviates(R, R),
            "the baseline itself is not a deviation"
        );
        assert!(!rate_deviates(R, R / 2), "exactly half is inside");
        assert!(rate_deviates(R, R / 2 - 1), "one mHz below half is outside");
        assert!(!rate_deviates(R, R * 2), "exactly double is inside");
        assert!(
            rate_deviates(R, R * 2 + 1),
            "one mHz above double is outside"
        );
        // A zero observation against a real baseline is the extreme low case.
        assert!(rate_deviates(R, 0));
    }

    /// The low edge is computed by MULTIPLYING the observation, never by dividing
    /// the baseline: an ODD baseline divided by two floors, which would widen the
    /// band by half a millihertz and let a genuinely-halved topic sit inside it.
    /// Driven at an odd baseline where the two formulations disagree.
    #[test]
    fn an_odd_baseline_does_not_widen_the_low_edge_by_flooring() {
        const R: u64 = 21; // R/2 floors to 10, but half of 21 is 10.5
                           // 10 x 2 = 20 < 21 => deviating. A `observed < R / 2` formulation would
                           // read `10 < 10` => false and MISS it.
        assert!(rate_deviates(R, 10));
        assert!(!rate_deviates(R, 11), "11 x 2 = 22 >= 21, inside");
    }

    /// A baseline near the integer ceiling must not WRAP into a false alert.
    ///
    /// Both edges are driven at values where the unchecked arithmetic really does
    /// overflow — `baseline * 2` on the high edge and `observed * 2` on the low
    /// one — so a wrapping implementation inverts each verdict rather than merely
    /// being unprovable. Debug builds would panic on the overflow; RELEASE builds
    /// would wrap silently, which is the shipping hazard.
    #[test]
    fn a_baseline_at_the_integer_ceiling_saturates_rather_than_wrapping() {
        assert!(
            !rate_deviates(u64::MAX, u64::MAX),
            "a topic at its baseline"
        );
        // HIGH edge overflow: 2 x baseline exceeds u64. The observation is BELOW
        // the true 2x bound, so the correct answer is "inside the band".
        let huge = u64::MAX / 2 + 1;
        assert!(
            !rate_deviates(huge, u64::MAX),
            "2 x baseline overflows; wrapping would read this as deviating"
        );
        // LOW edge overflow: 2 x observed exceeds u64, so it cannot be below any
        // baseline.
        assert!(!rate_deviates(u64::MAX, u64::MAX / 2 + 1));
        // …and the genuinely extreme cases still report correctly.
        assert!(rate_deviates(u64::MAX, 1), "still deviating LOW");
        assert!(rate_deviates(1, u64::MAX), "still deviating HIGH");
    }

    /// The median is the MEDIAN, against hand-written vectors — including the one
    /// shape it exists for: a single wild sample inside an otherwise steady run
    /// must not move it. A mean would.
    #[test]
    fn the_baseline_is_a_median_so_one_wild_window_cannot_drag_it() {
        assert_eq!(median_mhz(&[]), None);
        assert_eq!(median_mhz(&[7]), Some(7));
        assert_eq!(median_mhz(&[3, 1, 2]), Some(2), "sorted, middle");
        // Even length takes the LOWER middle: an observed value, not an average.
        assert_eq!(median_mhz(&[10, 20]), Some(10));
        // The motivating shape: seven samples at 20 Hz and one stalled window.
        let window = [20_000, 20_000, 20_000, 0, 20_000, 20_000, 20_000, 20_000];
        assert_eq!(median_mhz(&window), Some(20_000));
        // The mean would have been 17_500 — well inside the band, and wrong.
        let mean: u64 = window.iter().sum::<u64>() / window.len() as u64;
        assert_ne!(
            mean, 20_000,
            "the mean really does move; the median does not"
        );
    }

    /// The wire spellings are the closed sets a controller matches on. Pinned as
    /// literal JSON so a rename is a deliberate, visible act.
    #[test]
    fn the_vocabulary_serializes_to_the_documented_snake_case_names() {
        let cond = |c: MonitorCondition| serde_json::to_string(&c).expect("serializes");
        assert_eq!(cond(MonitorCondition::Stalled), "\"stalled\"");
        assert_eq!(cond(MonitorCondition::Silent), "\"silent\"");
        assert_eq!(cond(MonitorCondition::RateDeviation), "\"rate_deviation\"");

        let state = |s: MonitorState| serde_json::to_string(&s).expect("serializes");
        assert_eq!(state(MonitorState::Healthy), "\"healthy\"");
        assert_eq!(state(MonitorState::Alerting), "\"alerting\"");
        assert_eq!(state(MonitorState::Learning), "\"learning\"");
        assert_eq!(state(MonitorState::Unknown), "\"unknown\"");

        let reason = |r: IneligibleReason| serde_json::to_string(&r).expect("serializes");
        assert_eq!(reason(IneligibleReason::SlowTopic), "\"slow_topic\"");
        assert_eq!(
            reason(IneligibleReason::UntrustedRateBasis),
            "\"untrusted_rate_basis\""
        );
        assert_eq!(
            reason(IneligibleReason::NoLivenessEvidence),
            "\"no_liveness_evidence\""
        );
    }

    fn liveness(age_ms: Option<u64>, observed_for_ms: u64, frames: u64) -> TopicLiveness {
        TopicLiveness {
            last_frame_age_ms: age_ms,
            observed_for_ms,
            frames_observed: frames,
            rate_estimate: None,
        }
    }

    /// Rule 1 and rule 2 collapse to ONE answer, and the arms are independent: an
    /// absent payload, an `Unknown` classification and an unconverged plane each
    /// yield no evidence ON THEIR OWN, while a converged, classifiable sample does.
    #[test]
    fn a_sample_is_evidential_only_when_the_substrate_and_discovery_both_spoke() {
        // Converged + classifiable => the classified verdict, verbatim.
        let live = MonitorSample::new("/t", None, 0, Some(liveness(Some(10), 1_000, 5)), true);
        assert_eq!(live.is_evidential(), Some(LivenessState::Streaming));

        // Rule 1a: absent payload.
        let absent = MonitorSample::new("/t", None, 0, None, true);
        assert_eq!(absent.is_evidential(), None);

        // Rule 1b: present payload that classifies Unknown (nothing seen, and not
        // long enough to call it dead).
        let unknown = MonitorSample::new("/t", None, 0, Some(liveness(None, 1_000, 0)), true);
        assert_eq!(
            unknown.liveness.expect("payload").state(),
            LivenessState::Unknown,
            "precondition: this payload really does classify Unknown"
        );
        assert_eq!(unknown.is_evidential(), None);

        // Rule 2: a perfectly good payload behind an unconverged plane.
        let unconverged =
            MonitorSample::new("/t", None, 0, Some(liveness(Some(10), 1_000, 5)), false);
        assert_eq!(unconverged.is_evidential(), None);
    }

    /// The sample's rate DEFAULTS to the liveness payload's own estimate (the
    /// unattached plane) and can be OVERRIDDEN by the attached plane's
    /// measurement. Both halves in one body so neither can be satisfied alone.
    #[test]
    fn the_sample_rate_defaults_to_the_payload_and_can_be_overridden() {
        let mut payload = liveness(Some(10), 1_000, 5);
        payload.rate_estimate = Some(TopicRateEstimate {
            millihertz: 20_000,
            is_floor: false,
        });
        let sample = MonitorSample::new("/t", None, 0, Some(payload), true);
        assert_eq!(sample.rate.expect("inherited").millihertz, 20_000);

        let overridden = sample.with_rate(Some(TopicRateEstimate {
            millihertz: 30_000,
            is_floor: true,
        }));
        assert_eq!(overridden.rate.expect("override").millihertz, 30_000);
        assert!(overridden.rate.expect("override").is_floor);
        // The payload itself is untouched — the override is the SAMPLE's rate.
        assert_eq!(
            overridden
                .liveness
                .expect("payload")
                .rate_estimate
                .expect("payload rate")
                .millihertz,
            20_000
        );
    }

    /// The tracker needs BOTH the count and the span. Driven twice over the SAME
    /// qualifying run, differing only in how fast the samples arrive — a count-only
    /// rule raises on the fast one, a span-only rule raises on a two-sample slow
    /// one, and the real rule raises on neither.
    #[test]
    fn a_confirmation_needs_both_the_count_and_the_span() {
        // FAST: the count is met at sample 4, but they span only 30 ms.
        let mut fast = ConditionTracker::default();
        for i in 0..MONITOR_CONFIRM_SAMPLES as u64 {
            assert_eq!(
                fast.observe(true, i * 10_000_000),
                TrackerTransition::None,
                "sample {i} is inside the span, so it cannot confirm"
            );
        }
        assert!(!fast.is_raised());
        // …and the very next sample, once the span is covered, raises.
        assert_eq!(
            fast.observe(true, MONITOR_CONFIRM_MIN_SPAN_NS),
            TrackerTransition::Raised
        );

        // SLOW: two samples an hour apart cover the span but not the count.
        let mut slow = ConditionTracker::default();
        assert_eq!(slow.observe(true, 0), TrackerTransition::None);
        assert_eq!(
            slow.observe(true, 3_600_000_000_000),
            TrackerTransition::None,
            "the span is covered, the count is not"
        );
        assert!(!slow.is_raised());
    }

    /// A qualifying run BROKEN by a non-qualifying sample restarts from zero — the
    /// run must be CONSECUTIVE, and the span is measured from the restart, not from
    /// the original start.
    #[test]
    fn a_broken_qualifying_run_restarts_the_count_and_the_span() {
        let mut t = ConditionTracker::default();
        // Three qualifying samples spanning well past the span requirement.
        for i in 0..3u64 {
            t.observe(true, i * MONITOR_CONFIRM_MIN_SPAN_NS);
        }
        // One healthy sample breaks it.
        assert_eq!(
            t.observe(false, 3 * MONITOR_CONFIRM_MIN_SPAN_NS),
            TrackerTransition::None
        );
        // Now four more qualifying samples arrive back to back — the count is met
        // but the span is measured from THIS run's start, so nothing raises.
        for i in 0..MONITOR_CONFIRM_SAMPLES as u64 {
            let at = 4 * MONITOR_CONFIRM_MIN_SPAN_NS + i * 1_000_000;
            assert_eq!(t.observe(true, at), TrackerTransition::None);
        }
        assert!(!t.is_raised());
    }

    /// Raise, then clear after exactly [`MONITOR_CLEAR_SAMPLES`] non-qualifying
    /// samples — pinned on BOTH sides (one short does NOT clear), and a re-arm
    /// afterwards so a cleared tracker is genuinely reusable.
    #[test]
    fn clearing_is_a_threshold_pinned_on_both_sides_and_re_arms() {
        let mut t = ConditionTracker::default();
        for i in 0..MONITOR_CONFIRM_SAMPLES as u64 {
            t.observe(true, i * MONITOR_CONFIRM_MIN_SPAN_NS);
        }
        assert!(t.is_raised());
        let raised_at = t.raised_at_ns();

        let base = 10 * MONITOR_CONFIRM_MIN_SPAN_NS;
        for i in 0..MONITOR_CLEAR_SAMPLES as u64 - 1 {
            assert_eq!(t.observe(false, base + i), TrackerTransition::None);
            assert!(t.is_raised(), "one short of the threshold still holds");
        }
        assert_eq!(
            t.observe(false, base + MONITOR_CLEAR_SAMPLES as u64),
            TrackerTransition::Cleared
        );
        assert!(!t.is_raised());
        assert_eq!(t, ConditionTracker::default(), "a clear re-arms fully");

        // Re-arm: the same tracker can raise again.
        for i in 0..MONITOR_CONFIRM_SAMPLES as u64 {
            t.observe(
                true,
                100 * MONITOR_CONFIRM_MIN_SPAN_NS + i * MONITOR_CONFIRM_MIN_SPAN_NS,
            );
        }
        assert!(t.is_raised());
        assert!(t.raised_at_ns() > raised_at);
    }

    /// A qualifying sample arriving mid-clear RESETS the clearing run — an alert
    /// that keeps re-qualifying is not intermittently cleared.
    #[test]
    fn a_qualifying_sample_resets_a_clearing_run() {
        let mut t = ConditionTracker::default();
        for i in 0..MONITOR_CONFIRM_SAMPLES as u64 {
            t.observe(true, i * MONITOR_CONFIRM_MIN_SPAN_NS);
        }
        assert!(t.is_raised());
        for _ in 0..100 {
            assert_eq!(t.observe(false, 0), TrackerTransition::None);
            assert_eq!(t.observe(true, 0), TrackerTransition::None);
            assert!(
                t.is_raised(),
                "the clearing run never reaches the threshold"
            );
        }
    }

    /// The blind arm is ASYMMETRIC, and both halves are asserted in one body: a
    /// blind sample breaks an unconfirmed run, and a blind sample NEVER clears a
    /// raised condition however many arrive.
    #[test]
    fn a_blind_sample_breaks_a_pending_run_but_never_clears_a_raised_one() {
        // Half 1: it breaks a pending run.
        let mut pending = ConditionTracker::default();
        for i in 0..MONITOR_CONFIRM_SAMPLES as u64 - 1 {
            pending.observe(true, i * MONITOR_CONFIRM_MIN_SPAN_NS);
        }
        pending.observe_blind();
        // The count restarts, so the next four back-to-back samples cannot raise.
        for i in 0..MONITOR_CONFIRM_SAMPLES as u64 {
            pending.observe(true, 100 * MONITOR_CONFIRM_MIN_SPAN_NS + i);
        }
        assert!(
            !pending.is_raised(),
            "the blind sample really broke the run"
        );

        // Half 2: it never clears a raised one.
        let mut raised = ConditionTracker::default();
        for i in 0..MONITOR_CONFIRM_SAMPLES as u64 {
            raised.observe(true, i * MONITOR_CONFIRM_MIN_SPAN_NS);
        }
        assert!(raised.is_raised());
        for _ in 0..1_000 {
            raised.observe_blind();
        }
        assert!(
            raised.is_raised(),
            "losing sight of a topic is not an all-clear"
        );
        // …and a real non-qualifying observation still clears it.
        for _ in 0..MONITOR_CLEAR_SAMPLES {
            raised.observe(false, 0);
        }
        assert!(!raised.is_raised());
    }

    fn raise_at(raised_at_ms: u64) -> Alert {
        Alert {
            seq: 3,
            topic: "/scan".to_string(),
            robot: Some("go2".to_string()),
            condition: MonitorCondition::Stalled,
            raised_at_ms,
            cleared_at_ms: None,
            age_ms: None,
            observed_mhz: None,
            baseline_mhz: Some(20_000),
            liveness_state: Some(LivenessState::Idle),
        }
    }

    /// The age is measured from the TRANSITION the entry records, which is a
    /// DIFFERENT stamp on each kind: a raise ages from `raised_at_ms`, a clear
    /// from `cleared_at_ms`. Both in one body against hand-written values, and
    /// the clear's oracle is chosen so the two rules give different answers —
    /// otherwise "ages from the clear" is satisfied by an implementation that
    /// always reads the raise.
    #[test]
    fn an_alerts_age_is_measured_from_the_transition_it_records() {
        let raise = raise_at(1_000);
        assert_eq!(raise.clone().with_age(41_000).age_ms, Some(40_000));

        let clear = Alert {
            seq: 4,
            cleared_at_ms: Some(30_000),
            ..raise.clone()
        };
        let stamped = clear.clone().with_age(41_000);
        assert_eq!(stamped.age_ms, Some(11_000), "ages from the CLEAR");
        assert_ne!(
            stamped.age_ms,
            Some(40_000),
            "a clear must not report the age of the raise it ends"
        );

        // The stamp touches NOTHING else — a served alert is the minted one plus
        // an age, never a re-derivation of anything beside it.
        assert_eq!(
            Alert {
                age_ms: None,
                ..stamped
            },
            clear
        );
    }

    /// A `now_ms` BELOW the stamp saturates to `Some(0)` rather than wrapping to
    /// a `u64::MAX`-ish age on a transition that just happened. Pinned on BOTH
    /// kinds and on BOTH sides of the boundary, so "saturates" cannot be
    /// satisfied by an implementation that always answers zero.
    #[test]
    fn an_age_below_the_transition_stamp_saturates_to_zero_rather_than_wrapping() {
        let raise = raise_at(5_000);
        assert_eq!(
            raise.clone().with_age(4_999).age_ms,
            Some(0),
            "one ms below"
        );
        assert_eq!(raise.clone().with_age(0).age_ms, Some(0), "far below");
        assert_eq!(raise.clone().with_age(5_000).age_ms, Some(0), "exactly at");
        // The anti-tautology half: one ms ABOVE really does report one ms.
        assert_eq!(raise.clone().with_age(5_001).age_ms, Some(1));

        // The clear kind saturates against ITS OWN stamp, not the raise's.
        let clear = Alert {
            cleared_at_ms: Some(9_000),
            ..raise
        };
        assert_eq!(clear.clone().with_age(8_000).age_ms, Some(0));
        assert_eq!(clear.with_age(9_500).age_ms, Some(500));
    }
}

#[cfg(test)]
mod engine_tests {
    use super::*;
    use crate::transport::liveness::RATE_FLOOR_BASIS_CEILING_MHZ;

    /// How many samples a fresh row spends inside the settle window at the
    /// production cadence. DERIVED, so a constant moving re-derives the harness
    /// instead of silently shortening every warm-up below.
    const SETTLE_SAMPLES: u64 = MONITOR_SETTLE_NS.div_ceil(MONITOR_SAMPLE_INTERVAL_NS);

    const TOPIC: &str = "/probe";
    /// 20 Hz, the shape most of these arms are driven at.
    const NOMINAL_MHZ: u64 = 20_000;
    /// Just under half [`NOMINAL_MHZ`] — the smallest rate that is OUTSIDE the
    /// octave band, so the deviation it raises is the band's own edge case rather
    /// than an obviously-broken number.
    const DEVIATING_MHZ: u64 = NOMINAL_MHZ / 2 - 1;
    // …asserted rather than assumed, since every retained-basis arm's wipe depends on
    // this rate really raising a deviation.
    const _: () = assert!(DEVIATING_MHZ * MONITOR_RATE_BAND < NOMINAL_MHZ);

    // --- hand-built liveness payloads, one per class the classifier can return.

    fn streaming() -> Option<TopicLiveness> {
        Some(TopicLiveness {
            last_frame_age_ms: Some(50),
            observed_for_ms: 60_000,
            frames_observed: 1_000,
            rate_estimate: None,
        })
    }

    /// `Idle` with a DATED age past the recency window — a topic that was
    /// streaming and stopped.
    fn idle_dated() -> Option<TopicLiveness> {
        Some(TopicLiveness {
            last_frame_age_ms: Some(LIVENESS_STREAMING_RECENCY_MS + 1),
            observed_for_ms: 60_000,
            frames_observed: 1_000,
            rate_estimate: None,
        })
    }

    /// `Idle` with NO age but banked frames: the liveness observer's undatable class (a flushed
    /// backlog, a `/tf_static` one-shot).
    fn idle_undatable() -> Option<TopicLiveness> {
        Some(TopicLiveness {
            last_frame_age_ms: None,
            observed_for_ms: 60_000,
            frames_observed: 4,
            rate_estimate: None,
        })
    }

    /// The registered-but-dead route: watched long enough, never a frame.
    fn no_data() -> Option<TopicLiveness> {
        Some(TopicLiveness {
            last_frame_age_ms: None,
            observed_for_ms: 60_000,
            frames_observed: 0,
            rate_estimate: None,
        })
    }

    /// Watched, nothing seen, and NOT long enough to call it dead.
    fn unknown() -> Option<TopicLiveness> {
        Some(TopicLiveness {
            last_frame_age_ms: None,
            observed_for_ms: 1_000,
            frames_observed: 0,
            rate_estimate: None,
        })
    }

    fn rate(millihertz: u64) -> Option<TopicRateEstimate> {
        Some(TopicRateEstimate {
            millihertz,
            is_floor: false,
        })
    }

    fn floor_rate(millihertz: u64) -> Option<TopicRateEstimate> {
        Some(TopicRateEstimate {
            millihertz,
            is_floor: true,
        })
    }

    /// Drives an engine on a synthetic clock. Every assertion in this module is a
    /// COUNT or a hand-written value vector; no arm states a wall.
    struct Driver {
        engine: MonitorEngine,
        now_ns: u64,
        converged: bool,
    }

    impl Driver {
        fn new() -> Self {
            Self {
                engine: MonitorEngine::new(),
                now_ns: 0,
                converged: true,
            }
        }

        /// Feed ONE sample WITHOUT advancing the clock (so two topics can be
        /// observed at the same instant).
        fn observe(
            &mut self,
            topic: &str,
            liveness: Option<TopicLiveness>,
            rate: Option<TopicRateEstimate>,
        ) -> Vec<Alert> {
            self.observe_from(None, topic, liveness, rate)
        }

        /// The same, from a NAMED robot — `None` is the genuine local producer.
        fn observe_from(
            &mut self,
            robot: Option<&str>,
            topic: &str,
            liveness: Option<TopicLiveness>,
            rate: Option<TopicRateEstimate>,
        ) -> Vec<Alert> {
            let sample = MonitorSample::new(
                topic,
                robot.map(str::to_string),
                self.now_ns,
                liveness,
                self.converged,
            )
            .with_rate(rate);
            self.engine.observe(&sample)
        }

        fn advance(&mut self) {
            self.now_ns += MONITOR_SAMPLE_INTERVAL_NS;
        }

        /// `n` samples at the production cadence, returning every alert raised.
        fn run(
            &mut self,
            n: u64,
            topic: &str,
            liveness: Option<TopicLiveness>,
            rate: Option<TopicRateEstimate>,
        ) -> Vec<Alert> {
            let mut out = Vec::new();
            for _ in 0..n {
                out.extend(self.observe(topic, liveness, rate));
                self.advance();
            }
            out
        }

        /// Push one row past its settle window with benign samples.
        fn warm_up(&mut self, topic: &str) -> Vec<Alert> {
            self.run(SETTLE_SAMPLES, topic, streaming(), rate(NOMINAL_MHZ))
        }

        /// Push the row past settle AND teach it a baseline.
        fn warm_up_and_learn(&mut self, topic: &str) -> Vec<Alert> {
            let mut out = self.warm_up(topic);
            out.extend(self.run(
                MONITOR_LEARN_SAMPLES as u64,
                topic,
                streaming(),
                rate(NOMINAL_MHZ),
            ));
            out
        }

        /// Raise `rate_deviation` at `deviating` and clear it by returning to
        /// `back_to` — which is what WIPES the frozen baseline (D4).
        ///
        /// Written as a real flap rather than a poke at the state, because the
        /// wipe is only reachable through a confirmed transition and every
        /// retained-basis arm needs the wiped state to have arrived the way production
        /// arrives at it.
        fn flap(&mut self, topic: &str, deviating: u64, back_to: u64) -> Vec<Alert> {
            let mut out = self.run(
                MONITOR_CONFIRM_SAMPLES as u64,
                topic,
                streaming(),
                rate(deviating),
            );
            out.extend(self.run(
                MONITOR_CLEAR_SAMPLES as u64,
                topic,
                streaming(),
                rate(back_to),
            ));
            out
        }

        /// Feed `idle_dated()` samples until a `stalled` RAISE appears, returning
        /// HOW MANY it took — the quantity two vectors are compared on when the
        /// claim is "the same sample index", which no wall could express.
        ///
        /// `None` if `limit` samples pass without one.
        fn idle_until_stall(&mut self, topic: &str, limit: u64) -> Option<(u64, Alert)> {
            for n in 1..=limit {
                for alert in self.run(1, topic, idle_dated(), None) {
                    if alert.condition == MonitorCondition::Stalled && alert.cleared_at_ms.is_none()
                    {
                        return Some((n, alert));
                    }
                }
            }
            None
        }

        fn row(&self, topic: &str) -> MonitorRow {
            self.row_of(None, topic)
        }

        /// The row for ONE (robot, topic) pair. Since a topic NAME can carry more
        /// than one row, every lookup names the origin it means.
        fn row_of(&self, robot: Option<&str>, topic: &str) -> MonitorRow {
            self.engine
                .rows(self.now_ns)
                .into_iter()
                .find(|r| r.topic == topic && r.robot.as_deref() == robot)
                .unwrap_or_else(|| panic!("no row for {robot:?} {topic}"))
        }
    }

    /// The harness's own derivation, pinned: 13 samples of a 400 ms cadence sit
    /// inside a 5 s settle window. Without this, every warm-up length below is an
    /// unchecked magic number.
    #[test]
    fn the_settle_window_is_thirteen_samples_at_the_production_cadence() {
        assert_eq!(SETTLE_SAMPLES, 13);
        // The last WITHHELD sample and the first ADMITTED one, by arithmetic —
        // at COMPILE time, since both are pure functions of shipped constants.
        const _: () =
            assert!((SETTLE_SAMPLES - 1) * MONITOR_SAMPLE_INTERVAL_NS < MONITOR_SETTLE_NS);
        const _: () = assert!(SETTLE_SAMPLES * MONITOR_SAMPLE_INTERVAL_NS >= MONITOR_SETTLE_NS);
    }

    /// THE headline: a topic that demonstrably streamed and then stopped raises
    /// `stalled` EXACTLY ONCE, and a sibling topic that keeps streaming raises
    /// NOTHING over the identical sample instants.
    ///
    /// The anti-tautology sibling is in the same body deliberately: without it,
    /// "exactly one alert" is satisfied by an engine that alerts once on
    /// everything it is shown.
    #[test]
    fn a_stopped_topic_raises_stalled_once_while_its_streaming_sibling_raises_nothing() {
        let mut d = Driver::new();
        const SIBLING: &str = "/sibling";

        // Both rows settle and learn together, at the same instants.
        for _ in 0..SETTLE_SAMPLES + MONITOR_LEARN_SAMPLES as u64 {
            assert!(d.observe(TOPIC, streaming(), rate(NOMINAL_MHZ)).is_empty());
            assert!(d
                .observe(SIBLING, streaming(), rate(NOMINAL_MHZ))
                .is_empty());
            d.advance();
        }
        assert_eq!(d.row(TOPIC).baseline_mhz, Some(NOMINAL_MHZ), "learned");
        assert_eq!(d.row(TOPIC).state, MonitorState::Healthy);

        // One stops; the other does not.
        let mut raised = Vec::new();
        for _ in 0..MONITOR_CONFIRM_SAMPLES as u64 {
            raised.extend(d.observe(TOPIC, idle_dated(), None));
            assert!(
                d.observe(SIBLING, streaming(), rate(NOMINAL_MHZ))
                    .is_empty(),
                "a healthy sibling must never alert"
            );
            d.advance();
        }

        assert_eq!(raised.len(), 1, "exactly one alert: {raised:?}");
        let alert = &raised[0];
        assert_eq!(alert.condition, MonitorCondition::Stalled);
        assert_eq!(alert.topic, TOPIC);
        assert_eq!(alert.cleared_at_ms, None, "a RAISE carries no clear time");
        assert_eq!(alert.baseline_mhz, Some(NOMINAL_MHZ));
        assert_eq!(alert.liveness_state, Some(LivenessState::Idle));
        assert_eq!(alert.seq, 0, "the first event of this engine's life");

        // STICKY, and silent afterwards: the state stays readable while the
        // transition fires once.
        assert!(d.run(50, TOPIC, idle_dated(), None).is_empty());
        let row = d.row(TOPIC);
        assert_eq!(row.state, MonitorState::Alerting);
        assert_eq!(row.conditions, vec![MonitorCondition::Stalled]);
        let sibling = d.row(SIBLING);
        assert_eq!(sibling.state, MonitorState::Healthy);
        assert!(sibling.conditions.is_empty());
        assert!(sibling.ineligible.is_empty());
    }

    /// A row that reaches `NoData` having NEVER been `Streaming` raises `silent`,
    /// not `stalled`.
    #[test]
    fn a_route_that_was_never_streaming_raises_silent_and_never_stalled() {
        let mut d = Driver::new();
        assert!(d.run(SETTLE_SAMPLES, TOPIC, no_data(), None).is_empty());

        let raised = d.run(MONITOR_CONFIRM_SAMPLES as u64, TOPIC, no_data(), None);
        assert_eq!(raised.len(), 1, "{raised:?}");
        assert_eq!(raised[0].condition, MonitorCondition::Silent);
        assert_eq!(raised[0].liveness_state, Some(LivenessState::NoData));
        assert_eq!(raised[0].baseline_mhz, None, "a dead route has no baseline");

        let row = d.row(TOPIC);
        assert_eq!(row.conditions, vec![MonitorCondition::Silent]);
        assert!(
            !row.conditions.contains(&MonitorCondition::Stalled),
            "it was never Streaming, so it cannot have stalled"
        );
    }

    /// The liveness observer's undatable-`Idle` class (`frames_observed > 0` with no age,
    /// i.e. a flushed backlog or a latched `/tf_static` one-shot) raises NOTHING, and
    /// needs no special case to do it.
    ///
    /// It is not `stalled` (never `Streaming`) and not `silent` (`NoData` requires
    /// zero frames). 200 samples is far past every confirmation threshold.
    #[test]
    fn the_undatable_idle_class_raises_nothing_and_needs_no_special_case() {
        let mut d = Driver::new();
        assert!(d
            .run(SETTLE_SAMPLES, TOPIC, idle_undatable(), None)
            .is_empty());
        let alerts = d.run(200, TOPIC, idle_undatable(), None);
        assert!(alerts.is_empty(), "{alerts:?}");
        let row = d.row(TOPIC);
        assert!(row.conditions.is_empty());
        assert_eq!(row.liveness_state, Some(LivenessState::Idle));
        assert!(row.samples > 0, "the samples really were admitted");
    }

    /// THE arm for the WIDENED settle window: a split history flush at attach
    /// reads `Streaming` for up to the recency window and then settles to the
    /// undatable `Idle` it really is. That must raise NOTHING.
    ///
    /// A settle window that discarded only BASELINE samples — the design memo's
    /// narrower rule — would let the spurious `Streaming` set `ever_streaming`,
    /// and the settle back to `Idle` would then confirm a false `stalled` on every
    /// attach of a latched topic.
    ///
    /// **The flush CARRIES A RATE, and that is load-bearing.** A flush with no
    /// rate would not catch a narrow settle-window rule: with
    /// no rate no baseline is ever learned, so the stall gate stays `Learning` and
    /// blocks the alert for a reason that has nothing to do with the settle
    /// window. A real flush drains real frames whose sequences advance, so
    /// the rate estimator rates it like any other stream — which is what makes the leak
    /// reachable and this arm an oracle rather than a coincidence. The flush is
    /// exactly the residual's documented bound (`SETTLE_SAMPLES`), which is longer
    /// than `MONITOR_LEARN_SAMPLES`, so under the narrow rule the baseline really
    /// does get learned.
    #[test]
    fn a_split_history_flush_at_attach_never_raises_a_false_stall() {
        const _: () = assert!(SETTLE_SAMPLES >= MONITOR_LEARN_SAMPLES as u64);
        let mut d = Driver::new();
        // The flush: `Streaming` at a healthy, trustworthy rate for the whole
        // settle window and not one sample longer.
        assert!(d
            .run(SETTLE_SAMPLES, TOPIC, streaming(), rate(NOMINAL_MHZ))
            .is_empty());
        // Nothing was admitted, so nothing was learned FROM the flush — the
        // discriminator against the narrow rule, which would have a baseline here.
        let after_flush = d.row(TOPIC);
        assert_eq!(after_flush.samples, 0, "the flush contributed nothing");
        assert_eq!(after_flush.baseline_mhz, None, "…including no baseline");

        // …then the truth, for far longer than any confirmation needs.
        let alerts = d.run(200, TOPIC, idle_undatable(), None);
        assert!(
            alerts.is_empty(),
            "a latched topic's attach flush must not page: {alerts:?}"
        );
        assert!(d.row(TOPIC).conditions.is_empty());
    }

    /// The lull-free restart: the substrate serves `Idle` carrying the DEAD
    /// run's growing age for about a second before the epoch reset fires. The
    /// monitor must not page inside that window.
    ///
    /// Driven at the SWEEP cadence rather than the sample cadence, so the sample
    /// COUNT threshold is crossed and the SPAN is the only thing holding the alert
    /// back — which is what makes this the span's oracle rather than a second copy
    /// of the count's.
    #[test]
    fn a_lull_free_restart_is_absorbed_by_the_confirmation_span() {
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);

        const FAST_SAMPLES: u64 = 5;
        // Preconditions, so this arm cannot pass by driving too few samples or by
        // spanning too long. Checked at COMPILE time: they are pure functions of
        // the shipped constants, so a constant moving must break the build here
        // rather than quietly turn this arm vacuous.
        const _: () = assert!(FAST_SAMPLES > MONITOR_CONFIRM_SAMPLES as u64);
        const _: () =
            assert!(FAST_SAMPLES * LIVENESS_SWEEP_INTERVAL_NS < MONITOR_CONFIRM_MIN_SPAN_NS);

        let mut alerts = Vec::new();
        for _ in 0..FAST_SAMPLES {
            alerts.extend(d.observe(TOPIC, idle_dated(), None));
            d.now_ns += LIVENESS_SWEEP_INTERVAL_NS;
        }
        assert!(
            alerts.is_empty(),
            "the count was met but the span was not: {alerts:?}"
        );

        // The restart completes and the row is healthy again.
        assert!(d.run(10, TOPIC, streaming(), rate(NOMINAL_MHZ)).is_empty());
        assert_eq!(d.row(TOPIC).state, MonitorState::Healthy);
    }

    /// D7: a row that has EVER been served a labelled FLOOR rate is ineligible for
    /// `stalled` AND `rate_deviation`, says so per condition, and still gets
    /// `silent`.
    ///
    /// This is the rate estimator's `multi_publisher_topics` shape (`/tf`), the class the
    /// lull-free-restart fix explicitly does not heal.
    #[test]
    fn a_floor_basis_row_is_ineligible_for_stall_and_rate_but_still_gets_silent() {
        let mut d = Driver::new();
        assert!(d
            .run(
                SETTLE_SAMPLES + 20,
                TOPIC,
                streaming(),
                floor_rate(RATE_FLOOR_BASIS_CEILING_MHZ)
            )
            .is_empty());
        assert_eq!(d.row(TOPIC).observed_is_floor, Some(true));

        // It stops. Nothing may be raised.
        let alerts = d.run(50, TOPIC, idle_dated(), None);
        assert!(
            alerts.is_empty(),
            "an untrusted basis must not page: {alerts:?}"
        );

        let row = d.row(TOPIC);
        assert_eq!(
            row.ineligible,
            vec![
                Ineligible {
                    condition: MonitorCondition::Stalled,
                    reason: IneligibleReason::UntrustedRateBasis,
                },
                Ineligible {
                    condition: MonitorCondition::RateDeviation,
                    reason: IneligibleReason::UntrustedRateBasis,
                },
            ],
            "the agent is told WHICH conditions it is not being given"
        );
        assert!(
            !row.ineligible
                .iter()
                .any(|i| i.condition == MonitorCondition::Silent),
            "an ineligible row still gets `silent`"
        );
        assert_eq!(
            row.state,
            MonitorState::Healthy,
            "a row that can NEVER learn a baseline is not `learning` forever"
        );
        assert_eq!(row.baseline_mhz, None);
    }

    /// The floor mark is PERMANENT: one labelled floor poisons the basis even if
    /// every later sample is a clean measurement. A basis that could be laundered
    /// by a good sample would page on exactly the `/tf` class D7 excludes.
    #[test]
    fn one_floor_sample_poisons_the_rate_basis_permanently() {
        let mut d = Driver::new();
        d.warm_up(TOPIC);
        assert!(d
            .run(1, TOPIC, streaming(), floor_rate(NOMINAL_MHZ))
            .is_empty());
        // 100 clean measurements afterwards.
        assert!(d.run(100, TOPIC, streaming(), rate(NOMINAL_MHZ)).is_empty());
        let row = d.row(TOPIC);
        assert_eq!(
            row.observed_is_floor,
            Some(false),
            "the LAST rate was clean"
        );
        assert!(
            row.ineligible
                .iter()
                .any(|i| i.reason == IneligibleReason::UntrustedRateBasis),
            "…and the basis is still untrusted: {row:?}"
        );
        assert!(d.run(50, TOPIC, idle_dated(), None).is_empty());
    }

    /// `rate_deviation` raises when a learned row halves, and CLEARS when it comes
    /// back — with the clear as a NEW event carrying a NEW `seq`, never a mutation
    /// of the raise (the retention contract: an agent dedupes on the highest `seq`
    /// it has seen, so an in-place edit is a change it can never learn about).
    ///
    /// The D4 re-learn is asserted in the same body: after the clear the baseline
    /// is discarded, because a restart may legitimately change the rate.
    ///
    /// What the wipe does NOT do is park the stall gate: the row still
    /// withholds nothing, because stall eligibility was earned at the freeze and
    /// the retained basis outlives the wipe. That is asserted here only as the
    /// absence of an `ineligible` entry — the behavioural pin is
    /// `a_rate_flap_that_clears_after_the_publisher_dies_still_raises_stalled`.
    #[test]
    fn a_halved_rate_raises_and_clears_as_two_events_then_the_baseline_re_learns() {
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        assert_eq!(d.row(TOPIC).baseline_mhz, Some(NOMINAL_MHZ));

        // Just under half the baseline.
        let halved = NOMINAL_MHZ / 2 - 1;
        let raised = d.run(
            MONITOR_CONFIRM_SAMPLES as u64,
            TOPIC,
            streaming(),
            rate(halved),
        );
        assert_eq!(raised.len(), 1, "{raised:?}");
        assert_eq!(raised[0].condition, MonitorCondition::RateDeviation);
        assert_eq!(raised[0].observed_mhz, Some(halved));
        assert_eq!(raised[0].baseline_mhz, Some(NOMINAL_MHZ));
        assert_eq!(raised[0].cleared_at_ms, None);
        let raised_at = raised[0].raised_at_ms;

        // Back to nominal: it clears after exactly MONITOR_CLEAR_SAMPLES.
        let cleared = d.run(
            MONITOR_CLEAR_SAMPLES as u64,
            TOPIC,
            streaming(),
            rate(NOMINAL_MHZ),
        );
        assert_eq!(cleared.len(), 1, "{cleared:?}");
        let clear = &cleared[0];
        assert_eq!(clear.condition, MonitorCondition::RateDeviation);
        assert_eq!(clear.seq, raised[0].seq + 1, "a clear is a NEW ring entry");
        assert_eq!(clear.raised_at_ms, raised_at, "it names the raise it ends");
        assert!(clear.cleared_at_ms.is_some(), "…and carries the clear time");
        assert_eq!(
            clear.baseline_mhz,
            Some(NOMINAL_MHZ),
            "the baseline it was JUDGED against, read before the re-learn"
        );

        // D4: the baseline is discarded and re-learned.
        let row = d.row(TOPIC);
        assert_eq!(row.baseline_mhz, None, "cleared alerts re-learn (D4)");
        assert_eq!(row.state, MonitorState::Learning);
        assert!(row.conditions.is_empty());
        assert!(
            row.ineligible.is_empty(),
            "the RATE band is re-learning, but nothing is being permanently \
             withheld — the stall gate rides the retained basis: {row:?}"
        );
    }

    /// The engine holds no clock, so it mints no AGE — every alert it produces
    /// carries `age_ms: None` (UNKNOWN), and the serving daemon stamps it at
    /// serve time. Driven over a real raise AND a real clear, because they are
    /// two mint sites; the stamp is then applied to prove the field is reachable
    /// from what the engine actually hands out (an `age_ms` the engine filled in
    /// would be an age relative to SAMPLE time, which is a different instant).
    #[test]
    fn the_engine_mints_no_age_because_serve_time_is_not_its_to_know() {
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        let raised = d.run(
            MONITOR_CONFIRM_SAMPLES as u64,
            TOPIC,
            streaming(),
            rate(NOMINAL_MHZ / 2 - 1),
        );
        assert_eq!(raised.len(), 1, "{raised:?}");
        assert_eq!(raised[0].age_ms, None, "a MINTED raise carries no age");

        let cleared = d.run(
            MONITOR_CLEAR_SAMPLES as u64,
            TOPIC,
            streaming(),
            rate(NOMINAL_MHZ),
        );
        assert_eq!(cleared.len(), 1, "{cleared:?}");
        assert_eq!(cleared[0].age_ms, None, "a MINTED clear carries no age");

        // …and the serve-time stamp lands on exactly that clear, measured from
        // the clear stamp the engine really wrote.
        let clear = cleared[0].clone();
        let cleared_at_ms = clear.cleared_at_ms.expect("a clear carries its time");
        assert_eq!(
            clear.with_age(cleared_at_ms + 40_000).age_ms,
            Some(40_000),
            "stalled 40 s ago, with no cross-clock arithmetic at the reader"
        );
    }

    /// The band is an OCTAVE end to end: a topic running at exactly half and at
    /// exactly double its baseline raises NOTHING over a long run. Without this
    /// the band could be tightened to anything and every other rate arm would
    /// still pass.
    #[test]
    fn a_topic_at_exactly_half_and_exactly_double_its_baseline_never_raises() {
        for observed in [NOMINAL_MHZ / 2, NOMINAL_MHZ * 2] {
            let mut d = Driver::new();
            d.warm_up_and_learn(TOPIC);
            let alerts = d.run(200, TOPIC, streaming(), rate(observed));
            assert!(
                alerts.is_empty(),
                "{observed} mHz against a {NOMINAL_MHZ} mHz baseline is INSIDE the octave: {alerts:?}"
            );
        }
    }

    /// A stopped stream serves NO rate. An absent rate must never SCORE
    /// (reading it as 0 Hz would be a confident false alert), and it must clear a
    /// raised deviation rather than hold it forever.
    #[test]
    fn an_absent_rate_clears_a_deviation_and_never_scores_one() {
        // Half 1: no rate, ever, against a real baseline => nothing raised.
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        let alerts = d.run(200, TOPIC, streaming(), None);
        assert!(
            alerts.is_empty(),
            "an absent rate is not a zero: {alerts:?}"
        );

        // Half 2: it CLEARS a raised deviation.
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        let raised = d.run(
            MONITOR_CONFIRM_SAMPLES as u64,
            TOPIC,
            streaming(),
            rate(NOMINAL_MHZ / 2 - 1),
        );
        assert_eq!(raised.len(), 1);
        let cleared = d.run(MONITOR_CLEAR_SAMPLES as u64, TOPIC, streaming(), None);
        assert_eq!(cleared.len(), 1, "{cleared:?}");
        assert!(cleared[0].cleared_at_ms.is_some());
    }

    /// Rules 1 and 2: an absent payload, an `Unknown` classification and an
    /// UNCONVERGED discovery plane each raise nothing on a row that is plainly
    /// dead — and none of them CLEARS an alert already raised.
    ///
    /// Every arm drives the same stimulus, and the POSITIVE CONTROL comes first:
    /// without it "raised nothing" is satisfied by an engine that raises nothing
    /// at all.
    #[test]
    fn no_evidence_never_raises_and_never_clears() {
        let mut control = Driver::new();
        control.warm_up_and_learn(TOPIC);
        assert_eq!(
            control
                .run(MONITOR_CONFIRM_SAMPLES as u64, TOPIC, idle_dated(), None)
                .len(),
            1,
            "control: the stimulus raises when the evidence is present"
        );

        // Arm A: the payload is absent.
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        assert!(d.run(200, TOPIC, None, None).is_empty());
        assert_eq!(d.row(TOPIC).state, MonitorState::Unknown);
        assert_eq!(d.row(TOPIC).liveness_state, None);

        // Arm B: the payload classifies Unknown.
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        assert!(d.run(200, TOPIC, unknown(), None).is_empty());
        assert_eq!(d.row(TOPIC).state, MonitorState::Unknown);

        // Arm C: discovery has not converged, on a payload that is otherwise
        // perfectly good and WOULD otherwise raise (the control proves it does).
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        d.converged = false;
        assert!(d.run(200, TOPIC, idle_dated(), None).is_empty());
        assert_eq!(d.row(TOPIC).state, MonitorState::Unknown);

        // …and none of them CLEARS. Raise first, then go blind three ways.
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        let raised = d.run(MONITOR_CONFIRM_SAMPLES as u64, TOPIC, idle_dated(), None);
        assert_eq!(raised.len(), 1);
        assert!(d.run(200, TOPIC, None, None).is_empty(), "no clear");
        assert!(d.run(200, TOPIC, unknown(), None).is_empty(), "no clear");
        d.converged = false;
        assert!(d.run(200, TOPIC, idle_dated(), None).is_empty(), "no clear");
        d.converged = true;
        let row = d.row(TOPIC);
        assert_eq!(
            row.conditions,
            vec![MonitorCondition::Stalled],
            "losing sight of a topic is not an all-clear"
        );
        assert_eq!(
            row.state,
            MonitorState::Alerting,
            "alerting outranks unknown"
        );
    }

    /// FLAP ORACLE, first half: a HEALTHY 0.5 Hz topic driven 200 samples raises
    /// EXACTLY ZERO alerts.
    ///
    /// Its 2 s period sits inside the recency window, so the substrate classifies
    /// it `Streaming` continuously and there is no transition to confirm — the
    /// design's own argument, asserted rather than asserted-in-prose.
    #[test]
    fn a_healthy_half_hertz_topic_never_flaps() {
        let mut d = Driver::new();
        d.warm_up(TOPIC);
        let alerts = d.run(200, TOPIC, streaming(), rate(500));
        assert!(alerts.is_empty(), "{alerts:?}");
        let row = d.row(TOPIC);
        assert_eq!(row.state, MonitorState::Healthy);
        assert_eq!(row.baseline_mhz, Some(500), "500 mHz is above the gate");
        assert!(row.ineligible.is_empty(), "nothing is withheld from it");
    }

    /// FLAP ORACLE, second half: a 0.1 Hz topic raises ZERO alerts AND reports
    /// `ineligible: slow_topic`.
    ///
    /// Its 10 s period is past BOTH the recency window (so the substrate genuinely
    /// oscillates `Streaming <-> Idle` — that is the truth, not a fault) and
    /// the rate estimator's horizon (so it is never given a rate at all). D7 excludes it
    /// structurally rather than papering over the flap with hysteresis, and the
    /// row SAYS it is excluded so the agent cannot read the silence as health.
    #[test]
    fn a_tenth_hertz_topic_raises_nothing_and_says_it_is_a_slow_topic() {
        let mut d = Driver::new();
        d.run(SETTLE_SAMPLES, TOPIC, idle_dated(), None);

        // 200 samples alternating exactly as a 10 s period does against a 5 s
        // window, with no rate ever offered.
        let mut alerts = Vec::new();
        for i in 0..200 {
            let liveness = if i % 2 == 0 {
                streaming()
            } else {
                idle_dated()
            };
            alerts.extend(d.observe(TOPIC, liveness, None));
            d.advance();
        }
        assert!(
            alerts.is_empty(),
            "a slow topic must never flap: {alerts:?}"
        );

        let row = d.row(TOPIC);
        assert!(
            row.ineligible.contains(&Ineligible {
                condition: MonitorCondition::Stalled,
                reason: IneligibleReason::SlowTopic,
            }),
            "the agent must be told it is not being watched for stalls: {row:?}"
        );
        assert!(row.conditions.is_empty());
    }

    /// A row whose learned baseline is BELOW the stall gate is excluded too — the
    /// other half of D7, reachable only when the substrate DOES rate the topic.
    /// Pinned on both sides of the threshold in one body.
    #[test]
    fn the_stall_gate_excludes_a_slow_baseline_and_admits_the_one_above_it() {
        // BELOW: one millihertz under the gate.
        let mut d = Driver::new();
        d.warm_up(TOPIC);
        d.run(
            MONITOR_LEARN_SAMPLES as u64,
            TOPIC,
            streaming(),
            rate(MONITOR_STALL_MIN_MHZ - 1),
        );
        assert_eq!(d.row(TOPIC).baseline_mhz, Some(MONITOR_STALL_MIN_MHZ - 1));
        assert!(d.run(50, TOPIC, idle_dated(), None).is_empty());
        assert!(d.row(TOPIC).ineligible.contains(&Ineligible {
            condition: MonitorCondition::Stalled,
            reason: IneligibleReason::SlowTopic,
        }));

        // AT the gate: admitted, and it really does raise.
        let mut d = Driver::new();
        d.warm_up(TOPIC);
        d.run(
            MONITOR_LEARN_SAMPLES as u64,
            TOPIC,
            streaming(),
            rate(MONITOR_STALL_MIN_MHZ),
        );
        assert_eq!(d.row(TOPIC).baseline_mhz, Some(MONITOR_STALL_MIN_MHZ));
        let alerts = d.run(MONITOR_CONFIRM_SAMPLES as u64, TOPIC, idle_dated(), None);
        assert_eq!(
            alerts.len(),
            1,
            "exactly at the gate is ELIGIBLE: {alerts:?}"
        );
        assert_eq!(alerts[0].condition, MonitorCondition::Stalled);
    }

    /// The settle window is a threshold pinned on BOTH sides, driven at the
    /// nanosecond: one nanosecond short and the sample contributes nothing;
    /// exactly at the window and it is admitted.
    #[test]
    fn the_settle_window_is_a_threshold_pinned_on_both_sides() {
        let mut d = Driver::new();
        // The row is created at t = 0.
        assert!(d.observe(TOPIC, streaming(), rate(NOMINAL_MHZ)).is_empty());
        assert_eq!(d.row(TOPIC).samples, 0);

        d.now_ns = MONITOR_SETTLE_NS - 1;
        d.observe(TOPIC, streaming(), rate(NOMINAL_MHZ));
        assert_eq!(d.row(TOPIC).samples, 0, "one nanosecond short is withheld");

        d.now_ns = MONITOR_SETTLE_NS;
        d.observe(TOPIC, streaming(), rate(NOMINAL_MHZ));
        assert_eq!(d.row(TOPIC).samples, 1, "exactly at the window is admitted");
    }

    /// The freshness pairing: a row inside its settle window reports `samples: 0`
    /// beside a SMALL `last_sample_age_ms`, which reads exactly "we are polling
    /// this row and have learned nothing from it" — and every condition is named
    /// as withheld, so an empty `conditions` cannot be read as health.
    #[test]
    fn a_row_with_no_admitted_evidence_says_so_on_every_condition() {
        let mut d = Driver::new();
        d.observe(TOPIC, streaming(), rate(NOMINAL_MHZ));
        d.advance();
        d.observe(TOPIC, streaming(), rate(NOMINAL_MHZ));

        let row = d.row(TOPIC);
        assert_eq!(row.samples, 0, "nothing has been admitted");
        assert_eq!(
            row.last_sample_age_ms, 0,
            "…but the FEED is fresh: we looked just now"
        );
        assert_eq!(row.state, MonitorState::Unknown);
        assert!(row.conditions.is_empty());
        assert_eq!(
            row.ineligible,
            MonitorCondition::ALL
                .into_iter()
                .map(|condition| Ineligible {
                    condition,
                    reason: IneligibleReason::NoLivenessEvidence,
                })
                .collect::<Vec<_>>()
        );
    }

    /// `last_sample_age_ms` ages with the RENDERING instant, not with the last
    /// observation — the D2 field is about how stale the evidence is NOW, which is
    /// the whole reason an agent cannot read silence as health. Rendering at a
    /// different instant must move nothing else.
    #[test]
    fn the_staleness_field_ages_while_nothing_is_sampled() {
        let mut d = Driver::new();
        d.warm_up(TOPIC);
        let last = d.now_ns - MONITOR_SAMPLE_INTERVAL_NS;
        for gap_ms in [0u64, 1_000, 60_000] {
            let rows = d.engine.rows(last + gap_ms * 1_000_000);
            assert_eq!(rows[0].last_sample_age_ms, gap_ms);
        }
        assert_eq!(
            d.engine.rows(last)[0].state,
            d.engine.rows(u64::MAX)[0].state,
            "rendering later changes no verdict"
        );
    }

    /// DETERMINISM (Principle #7 in miniature): the same sample vector through two
    /// independent engines yields byte-identical alerts — `seq` numbering included
    /// — and byte-identical rows.
    ///
    /// Anchored to a HAND oracle as well, so it is not purely a self-compare: the
    /// script must produce exactly the raise/clear pair it describes, on the topic
    /// it describes, with rows sorted regardless of the order they first appeared.
    ///
    /// The script drives a `rate_deviation` flap into `/a`'s death, so
    /// the retained stall basis is on the determinism path rather than beside it:
    /// the wipe lands mid-vector and the stall that follows is judged on the
    /// basis, so a basis that were not a pure function of the samples would show
    /// up here as a `seq` divergence.
    #[test]
    fn the_same_sample_vector_yields_byte_identical_alerts_and_rows() {
        let script = |d: &mut Driver| -> Vec<Alert> {
            let mut out = Vec::new();
            out.extend(d.warm_up_and_learn("/b"));
            out.extend(d.warm_up_and_learn("/a"));
            out.extend(d.run(
                MONITOR_CONFIRM_SAMPLES as u64,
                "/a",
                streaming(),
                rate(DEVIATING_MHZ),
            ));
            out.extend(d.run(MONITOR_CONFIRM_SAMPLES as u64, "/a", idle_dated(), None));
            out.extend(d.run(50, "/a", streaming(), rate(NOMINAL_MHZ)));
            out.extend(d.run(20, "/b", streaming(), rate(NOMINAL_MHZ)));
            out
        };

        let mut first = Driver::new();
        let a = script(&mut first);
        let mut second = Driver::new();
        let b = script(&mut second);
        assert_eq!(a, b, "two runs of one script must agree");
        assert_eq!(
            first.engine.rows(first.now_ns),
            second.engine.rows(second.now_ns)
        );
        assert_eq!(first.engine.next_seq(), second.engine.next_seq());

        // The hand oracle: the deviation raises and then clears on `/a`'s death,
        // and the stall the wipe used to swallow raises and clears after it.
        let shape: Vec<(u64, &str, MonitorCondition, bool)> = a
            .iter()
            .map(|alert| {
                (
                    alert.seq,
                    alert.topic.as_str(),
                    alert.condition,
                    alert.cleared_at_ms.is_some(),
                )
            })
            .collect();
        assert_eq!(
            shape,
            vec![
                (0, "/a", MonitorCondition::RateDeviation, false),
                (1, "/a", MonitorCondition::RateDeviation, true),
                (2, "/a", MonitorCondition::Stalled, false),
                (3, "/a", MonitorCondition::Stalled, true),
            ],
            "{a:?}"
        );
        assert_eq!(
            a[2].baseline_mhz,
            Some(NOMINAL_MHZ),
            "the stall names the basis it was judged against, wipe or no wipe"
        );
        // Rows come back SORTED, whatever order the topics were first shown in.
        let topics: Vec<String> = first
            .engine
            .rows(first.now_ns)
            .into_iter()
            .map(|r| r.topic)
            .collect();
        assert_eq!(topics, vec!["/a".to_string(), "/b".to_string()]);
    }

    /// A forgotten row is gone, and re-showing the topic starts a FRESH row —
    /// settle window included. Nothing learned about the old producer describes
    /// the new one.
    ///
    /// **The retained stall basis is part of "nothing"**, and half 2
    /// is what makes that an oracle rather than a formality.
    ///
    /// Half 1 — the topic comes back already DEAD — cannot see the basis at all:
    /// the stall predicate needs `ever_streaming`, which a fresh row has not
    /// earned either, so it answers "no stall" for a reason that has nothing to
    /// do with the gate. MEASURED: a basis hoisted out of the row and into a map
    /// that `forget` does not clear survives that half untouched. Half 2
    /// therefore lets the topic come back ALIVE for too few samples to re-freeze
    /// anything, so `ever_streaming` is set and the gate is the ONLY thing left
    /// standing between the old producer's evidence and a page for a row that has
    /// learned nothing.
    #[test]
    fn a_forgotten_row_starts_over_when_the_topic_comes_back() {
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        assert_eq!(d.engine.watched(), 1);
        assert_eq!(d.row(TOPIC).baseline_mhz, Some(NOMINAL_MHZ));

        assert!(d.engine.forget(None, TOPIC), "the row existed");
        assert!(!d.engine.forget(None, TOPIC), "…and is gone");
        assert_eq!(d.engine.watched(), 0);

        // Half 1: back again and already dead — inside a fresh settle window, so
        // it is judged on nothing.
        d.observe(TOPIC, idle_dated(), None);
        let row = d.row(TOPIC);
        assert_eq!(row.samples, 0);
        assert_eq!(row.baseline_mhz, None);
        assert!(d.run(50, TOPIC, idle_dated(), None).is_empty());

        // Half 2: back again ALIVE, but for fewer samples than a learning window
        // — so `ever_streaming` is earned and a baseline is not — and then it
        // dies. The old row's basis must not answer for it.
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        assert!(d.engine.forget(None, TOPIC));

        assert!(d
            .run(SETTLE_SAMPLES, TOPIC, streaming(), rate(NOMINAL_MHZ))
            .is_empty());
        assert_eq!(d.row(TOPIC).samples, 0, "the fresh settle window held");

        const PARTIAL: u64 = 3;
        const _: () = assert!(PARTIAL < MONITOR_LEARN_SAMPLES as u64);
        assert!(d
            .run(PARTIAL, TOPIC, streaming(), rate(NOMINAL_MHZ))
            .is_empty());
        let row = d.row(TOPIC);
        assert!(row.samples > 0, "the revival really was observed: {row:?}");
        assert_eq!(row.baseline_mhz, None, "…and froze nothing: {row:?}");

        let alerts = d.run(50, TOPIC, idle_dated(), None);
        assert!(
            alerts.is_empty(),
            "the forgotten row's basis must not page for a row that has learned \
             nothing of its own: {alerts:?}"
        );
        assert_eq!(
            d.row(TOPIC).state,
            MonitorState::Learning,
            "a row judged on nothing is `learning`: {:?}",
            d.row(TOPIC)
        );
    }

    /// THE cross-robot headline: one desk watching `/lowstate` on TWO robots must
    /// judge them independently.
    ///
    /// Robot `a` streams throughout; robot `b` streams, learns the same baseline,
    /// and then stops. Keyed by topic ALONE the two robots' samples interleave into
    /// one tracker, and `a`'s healthy `Streaming` samples break `b`'s qualifying
    /// run on every other sample — so the dead route raises NOTHING and the live
    /// one carries whatever attribution was sampled last. Keyed by (topic, robot)
    /// there are two rows: `b` alerts, `a` does not.
    ///
    /// The healthy sibling is in the same body deliberately (the anti-tautology
    /// half): "exactly one alert, attributed to `b`" is otherwise satisfied by an
    /// engine that alerts once on everything it is shown.
    #[test]
    fn two_robots_serving_one_topic_name_are_two_rows_and_only_the_dead_one_alerts() {
        const A: &str = "robot-a";
        const B: &str = "robot-b";
        let mut d = Driver::new();

        // Both robots warm up past settle and learn the SAME baseline, sampled at
        // identical instants.
        for _ in 0..SETTLE_SAMPLES + MONITOR_LEARN_SAMPLES as u64 {
            d.observe_from(Some(A), TOPIC, streaming(), rate(NOMINAL_MHZ));
            d.observe_from(Some(B), TOPIC, streaming(), rate(NOMINAL_MHZ));
            d.advance();
        }

        // `b` stops; `a` keeps streaming, interleaved at the same instants.
        let mut alerts = Vec::new();
        for _ in 0..MONITOR_CONFIRM_SAMPLES as u64 {
            alerts.extend(d.observe_from(Some(A), TOPIC, streaming(), rate(NOMINAL_MHZ)));
            alerts.extend(d.observe_from(Some(B), TOPIC, idle_dated(), None));
            d.advance();
        }

        assert_eq!(alerts.len(), 1, "exactly one row is broken: {alerts:?}");
        assert_eq!(alerts[0].condition, MonitorCondition::Stalled);
        assert_eq!(
            alerts[0].robot.as_deref(),
            Some(B),
            "the alert names the robot whose topic died"
        );

        let row_a = d.row_of(Some(A), TOPIC);
        let row_b = d.row_of(Some(B), TOPIC);
        assert_eq!(row_a.state, MonitorState::Healthy, "{row_a:?}");
        assert!(row_a.conditions.is_empty(), "{row_a:?}");
        assert_eq!(row_b.state, MonitorState::Alerting, "{row_b:?}");
        assert_eq!(row_b.conditions, vec![MonitorCondition::Stalled]);
        assert_eq!(d.engine.watched(), 2, "one topic NAME, two rows");
    }

    /// Two robots serving one topic name at DIFFERENT rates each learn their OWN
    /// baseline, and neither is scored against the other's.
    ///
    /// This is the quieter half of the same defect and the one that would have been
    /// hardest to explain on a robot: merged, the learn window interleaves 20 Hz and
    /// 200 Hz samples, the median lands on neither, and the faster robot then reads
    /// as a `rate_deviation` while doing exactly what it always did.
    #[test]
    fn each_robots_row_learns_its_own_baseline_and_is_never_scored_against_the_other() {
        const SLOW: &str = "robot-slow";
        const FAST: &str = "robot-fast";
        const FAST_MHZ: u64 = NOMINAL_MHZ * 10;
        let mut d = Driver::new();

        let mut alerts = Vec::new();
        for _ in 0..SETTLE_SAMPLES + MONITOR_LEARN_SAMPLES as u64 + 50 {
            alerts.extend(d.observe_from(Some(SLOW), TOPIC, streaming(), rate(NOMINAL_MHZ)));
            alerts.extend(d.observe_from(Some(FAST), TOPIC, streaming(), rate(FAST_MHZ)));
            d.advance();
        }

        assert!(
            alerts.is_empty(),
            "two healthy robots, two baselines, no alert: {alerts:?}"
        );
        // Asserted BEFORE the baselines: merged, this arm's real failure is a
        // wrong baseline on a row that does not exist, and a bare lookup panic
        // says less than the count does.
        assert_eq!(d.engine.watched(), 2, "one topic NAME, two rows");
        assert_eq!(
            d.row_of(Some(SLOW), TOPIC).baseline_mhz,
            Some(NOMINAL_MHZ),
            "the slow robot's own rate"
        );
        assert_eq!(
            d.row_of(Some(FAST), TOPIC).baseline_mhz,
            Some(FAST_MHZ),
            "the fast robot's own rate"
        );
    }

    /// A genuine LOCAL producer (`robot: None`) reusing a remote topic's NAME is its
    /// own row — the attribution rule, now structural rather than a
    /// last-writer-wins field.
    ///
    /// Absence and presence are DISTINCT keys, so the local row starts fresh
    /// (settle window included) and inherits nothing the remote row learned, while
    /// the remote row keeps its own history for as long as it is watched.
    #[test]
    fn a_local_producer_reusing_a_remote_topics_name_is_its_own_row() {
        let mut d = Driver::new();
        // The remote row warms up and learns.
        for _ in 0..SETTLE_SAMPLES + MONITOR_LEARN_SAMPLES as u64 {
            d.observe_from(Some("go2"), TOPIC, streaming(), rate(NOMINAL_MHZ));
            d.advance();
        }
        assert_eq!(d.engine.watched(), 1, "one robot, one row");
        assert_eq!(d.row_of(Some("go2"), TOPIC).baseline_mhz, Some(NOMINAL_MHZ));

        // A LOCAL producer of the same topic name appears.
        d.observe_from(None, TOPIC, streaming(), rate(NOMINAL_MHZ));
        assert_eq!(d.engine.watched(), 2, "two rows, not a re-attribution");
        let local = d.row_of(None, TOPIC);
        assert_eq!(local.robot, None);
        assert_eq!(local.samples, 0, "inside its OWN fresh settle window");
        assert_eq!(
            local.baseline_mhz, None,
            "it inherits nothing the remote row learned"
        );
        assert_eq!(
            d.row_of(Some("go2"), TOPIC).baseline_mhz,
            Some(NOMINAL_MHZ),
            "…and the remote row is untouched"
        );
    }

    /// `forget` releases ONE row, named by its robot — a robot going away must not
    /// take its siblings' history with it.
    #[test]
    fn forgetting_one_robots_row_leaves_every_other_row_on_that_topic_intact() {
        let mut d = Driver::new();
        for _ in 0..SETTLE_SAMPLES + MONITOR_LEARN_SAMPLES as u64 {
            d.observe_from(Some("a"), TOPIC, streaming(), rate(NOMINAL_MHZ));
            d.observe_from(Some("b"), TOPIC, streaming(), rate(NOMINAL_MHZ));
            d.observe_from(None, TOPIC, streaming(), rate(NOMINAL_MHZ));
            d.advance();
        }
        assert_eq!(d.engine.watched(), 3);

        assert!(d.engine.forget(Some("a"), TOPIC), "the row existed");
        assert!(!d.engine.forget(Some("a"), TOPIC), "…and is gone");
        assert_eq!(d.engine.watched(), 2);
        assert_eq!(d.row_of(Some("b"), TOPIC).baseline_mhz, Some(NOMINAL_MHZ));
        assert_eq!(d.row_of(None, TOPIC).baseline_mhz, Some(NOMINAL_MHZ));

        // `None` means the LOCAL row specifically, never "any robot's".
        assert!(d.engine.forget(None, TOPIC));
        assert_eq!(d.engine.watched(), 1);
        assert_eq!(d.row_of(Some("b"), TOPIC).baseline_mhz, Some(NOMINAL_MHZ));
    }

    /// Rows sort by TOPIC first and then by ROBOT, with the local row first — so a
    /// rendered list still reads as a topic list and same-named rows sit adjacent.
    #[test]
    fn rows_sort_by_topic_then_robot_with_the_local_row_first() {
        let mut d = Driver::new();
        // Shown in an order that no sorted result could be an accident of.
        for (robot, topic) in [
            (Some("zulu"), "/b"),
            (None, "/b"),
            (Some("alpha"), "/b"),
            (Some("zulu"), "/a"),
        ] {
            d.observe_from(robot, topic, streaming(), rate(NOMINAL_MHZ));
        }
        let order: Vec<(String, Option<String>)> = d
            .engine
            .rows(d.now_ns)
            .into_iter()
            .map(|r| (r.topic, r.robot))
            .collect();
        assert_eq!(
            order,
            vec![
                ("/a".to_string(), Some("zulu".to_string())),
                ("/b".to_string(), None),
                ("/b".to_string(), Some("alpha".to_string())),
                ("/b".to_string(), Some("zulu".to_string())),
            ]
        );
    }

    /// THE blind-gap headline: a baseline may not be medianed ACROSS a hole in the
    /// observation stream.
    ///
    /// D4 learns from `MONITOR_LEARN_SAMPLES` *consecutive* trustworthy `Streaming`
    /// rates, and "consecutive" is the load-bearing word. Here the topic runs at
    /// `STALE_MHZ`, the observer goes blind (a discovery blip, a robot whose catalog
    /// stopped answering), and behind the hole the publisher has restarted at
    /// `LIVE_MHZ` — ten times faster, which is well outside the octave.
    ///
    /// Without the reset the half-window of STALE samples is stitched onto the
    /// LIVE ones, freezes a baseline the topic never ran at, and then pages
    /// `rate_deviation` on a topic that is doing exactly what it is supposed to.
    /// The discriminator is asserted directly as well as through the verdict: at
    /// the instant a merged window would have closed, there must be NO baseline.
    #[test]
    fn a_learn_window_may_not_span_a_blind_gap() {
        const STALE_MHZ: u64 = NOMINAL_MHZ;
        const LIVE_MHZ: u64 = NOMINAL_MHZ * 10;
        // Half a window each side of the gap, so a merged window closes on exactly
        // `MONITOR_LEARN_SAMPLES` samples and its median is the STALE rate.
        const HALF: u64 = MONITOR_LEARN_SAMPLES as u64 / 2;

        let mut d = Driver::new();
        d.run(SETTLE_SAMPLES, TOPIC, streaming(), rate(STALE_MHZ));
        d.run(HALF, TOPIC, streaming(), rate(STALE_MHZ));

        // The hole: one sample carrying no usable evidence at all.
        assert!(d.run(1, TOPIC, None, None).is_empty());

        // Behind it, the topic is running ten times faster.
        let alerts = d.run(HALF, TOPIC, streaming(), rate(LIVE_MHZ));
        assert!(alerts.is_empty(), "still learning: {alerts:?}");
        assert_eq!(
            d.row(TOPIC).baseline_mhz,
            None,
            "a merged window would have closed here, on the STALE rate"
        );

        // Enough LIVE samples to close a window that starts AFTER the gap.
        let alerts = d.run(HALF, TOPIC, streaming(), rate(LIVE_MHZ));
        assert!(alerts.is_empty(), "{alerts:?}");
        assert_eq!(
            d.row(TOPIC).baseline_mhz,
            Some(LIVE_MHZ),
            "the baseline describes a run the topic actually had"
        );

        // …and the topic keeps running at exactly that rate without ever paging.
        let alerts = d.run(200, TOPIC, streaming(), rate(LIVE_MHZ));
        assert!(
            alerts.is_empty(),
            "a healthy topic must not page against a stale baseline: {alerts:?}"
        );
        assert_eq!(d.row(TOPIC).state, MonitorState::Healthy);
    }

    /// Every one of the three blind shapes breaks the learning run — the gap is
    /// about the ABSENCE of evidence, not about which of the four ways produced it.
    #[test]
    fn each_blind_shape_breaks_the_learning_run() {
        const HALF: u64 = MONITOR_LEARN_SAMPLES as u64 / 2;

        // Absent payload, `Unknown` classification, unconverged discovery.
        for shape in 0..3u8 {
            let mut d = Driver::new();
            d.run(SETTLE_SAMPLES, TOPIC, streaming(), rate(NOMINAL_MHZ));
            d.run(HALF, TOPIC, streaming(), rate(NOMINAL_MHZ));
            match shape {
                0 => {
                    d.run(1, TOPIC, None, None);
                }
                1 => {
                    d.run(1, TOPIC, unknown(), None);
                }
                _ => {
                    d.converged = false;
                    d.run(1, TOPIC, streaming(), rate(NOMINAL_MHZ));
                    d.converged = true;
                }
            }
            d.run(HALF, TOPIC, streaming(), rate(NOMINAL_MHZ));
            assert_eq!(
                d.row(TOPIC).baseline_mhz,
                None,
                "shape {shape}: the run must not have spanned the gap"
            );
        }

        // ANTI-TAUTOLOGY: the identical script with NO gap DOES close a window.
        let mut d = Driver::new();
        d.run(SETTLE_SAMPLES, TOPIC, streaming(), rate(NOMINAL_MHZ));
        d.run(HALF * 2, TOPIC, streaming(), rate(NOMINAL_MHZ));
        assert_eq!(d.row(TOPIC).baseline_mhz, Some(NOMINAL_MHZ));
    }

    /// The blind reset is SCOPED to the learning RUN. It must not discard state the
    /// engine already EARNED, and both halves have bitten elsewhere in this module:
    ///
    /// * a FROZEN baseline survives — D4 re-learns after a raised alert clears and
    ///   at no other time, so a topic that goes out of view and comes back is still
    ///   judged against what it was measured at; and
    /// * `streaming_without_rate` survives, because it is deliberately CUMULATIVE
    ///   (a topic past the rate horizon oscillates `Streaming <-> Idle` by
    ///   design, and its evidence arrives in exactly this interrupted shape) — a
    ///   reset there would make the slow-topic gate unreachable and the row would
    ///   report `learning` forever with nothing withheld.
    ///
    /// Half 3 is the same rule applied to the retained stall basis,
    /// and it is driven on a row whose LIVE baseline has already been wiped — so
    /// the basis is the only thing left that can open the stall gate, and a blind
    /// branch that cleared it would silence the death outright.
    #[test]
    fn a_blind_sample_breaks_the_learning_run_and_nothing_else() {
        // Half 1: a frozen baseline outlives a long blind stretch.
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        assert_eq!(d.row(TOPIC).baseline_mhz, Some(NOMINAL_MHZ));
        assert!(d.run(200, TOPIC, None, None).is_empty());
        assert_eq!(
            d.row(TOPIC).baseline_mhz,
            Some(NOMINAL_MHZ),
            "losing sight of a topic is not one of D4's re-learn triggers"
        );

        // Half 2: the slow-topic counter reaches its threshold THROUGH the gaps.
        let mut d = Driver::new();
        d.run(SETTLE_SAMPLES, TOPIC, idle_dated(), None);
        for _ in 0..MONITOR_LEARN_SAMPLES * 4 {
            d.run(1, TOPIC, streaming(), None);
            d.run(1, TOPIC, None, None);
        }
        let row = d.row(TOPIC);
        assert!(
            row.ineligible.contains(&Ineligible {
                condition: MonitorCondition::Stalled,
                reason: IneligibleReason::SlowTopic,
            }),
            "a cumulative counter must survive the gaps it exists for: {row:?}"
        );

        // Half 3: the retained stall basis survives a long blind
        // stretch, on a row whose live baseline is already gone.
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        d.flap(TOPIC, DEVIATING_MHZ, NOMINAL_MHZ);
        assert_eq!(d.row(TOPIC).baseline_mhz, None, "the wipe landed");
        assert!(d.run(200, TOPIC, None, None).is_empty(), "nothing observed");
        let raised = d.run(MONITOR_CONFIRM_SAMPLES as u64, TOPIC, idle_dated(), None);
        assert_eq!(
            raised
                .iter()
                .map(|a| (a.condition, a.baseline_mhz))
                .collect::<Vec<_>>(),
            vec![(MonitorCondition::Stalled, Some(NOMINAL_MHZ))],
            "the basis is EARNED state and a blind gap is not one of D4's \
             re-learn triggers: {raised:?}"
        );
    }

    // -----------------------------------------------------------------------
    // The retained stall basis.
    //
    // The class these arms cover: the re-learn wipe used to un-earn stall eligibility, and
    // a publisher that DIED could never earn it back, so a row parked in
    // `learning` forever on exactly the topic an operator needed paged.
    // -----------------------------------------------------------------------

    /// THE headline: a `rate_deviation` that clears BECAUSE the
    /// publisher died must not swallow the stall.
    ///
    /// The death itself supplies the clear — a stopped stream serves no rate
    /// at all, a `rate_deviation` needs a present rate to keep qualifying, and
    /// clearing takes `MONITOR_CLEAR_SAMPLES` while a stall takes
    /// `MONITOR_CONFIRM_SAMPLES`, so the clear ALWAYS lands first. D4 then wipes
    /// the frozen baseline, and re-learning needs `MONITOR_LEARN_SAMPLES`
    /// consecutive trustworthy `Streaming` samples a dead publisher cannot
    /// supply. MEASURED without the wipe: 220 `Idle` samples, ZERO stall alerts.
    ///
    /// **The counterfactual is in the same body and it is the whole oracle.** An
    /// arm that only asserted "a stall eventually raises" would pass an
    /// implementation that re-learned late or raised on some other evidence; the
    /// claim being pinned is stronger — the wipe costs the row NOTHING, so the
    /// flapped vector raises at the very same sample index as the identical
    /// vector without the flap.
    #[test]
    fn a_rate_flap_that_clears_after_the_publisher_dies_still_raises_stalled() {
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        assert_eq!(d.row(TOPIC).baseline_mhz, Some(NOMINAL_MHZ));

        // The flap: the topic degrades, `rate_deviation` confirms…
        let deviated = d.run(
            MONITOR_CONFIRM_SAMPLES as u64,
            TOPIC,
            streaming(),
            rate(DEVIATING_MHZ),
        );
        assert_eq!(deviated.len(), 1, "{deviated:?}");
        assert_eq!(deviated[0].condition, MonitorCondition::RateDeviation);
        assert_eq!(deviated[0].seq, 0);

        // …and then the publisher DIES. The clear lands on the second `Idle`
        // sample, carrying the baseline it was judged against, and wipes it.
        let cleared = d.run(MONITOR_CLEAR_SAMPLES as u64, TOPIC, idle_dated(), None);
        assert_eq!(cleared.len(), 1, "{cleared:?}");
        assert_eq!(cleared[0].condition, MonitorCondition::RateDeviation);
        assert_eq!(cleared[0].seq, 1);
        assert!(cleared[0].cleared_at_ms.is_some());
        assert_eq!(cleared[0].baseline_mhz, Some(NOMINAL_MHZ));
        assert_eq!(d.row(TOPIC).baseline_mhz, None, "D4 wiped it");

        // THE FIX: the stall still confirms, on the basis the row earned.
        let (samples_after_clear, stall) = d
            .idle_until_stall(TOPIC, 200)
            .unwrap_or_else(|| panic!("no stall in 200 Idle samples: {:?}", d.row(TOPIC)));
        assert_eq!(
            samples_after_clear,
            (MONITOR_CONFIRM_SAMPLES - MONITOR_CLEAR_SAMPLES) as u64,
            "the qualifying run started at the FIRST Idle sample, so the wipe \
             cost it nothing"
        );
        assert_eq!(stall.seq, 2);
        assert_eq!(stall.condition, MonitorCondition::Stalled);
        assert_eq!(stall.cleared_at_ms, None, "a RAISE carries no clear stamp");
        assert_eq!(
            stall.baseline_mhz,
            Some(NOMINAL_MHZ),
            "the verdict names the basis it rested on"
        );
        assert_eq!(stall.observed_mhz, None, "a dead publisher serves no rate");
        assert_eq!(stall.liveness_state, Some(LivenessState::Idle));

        // STICKY and silent afterwards, and the row says nothing is withheld.
        assert!(d.run(200, TOPIC, idle_dated(), None).is_empty());
        let row = d.row(TOPIC);
        assert_eq!(row.state, MonitorState::Alerting);
        assert_eq!(row.conditions, vec![MonitorCondition::Stalled]);
        assert_eq!(
            row.baseline_mhz, None,
            "the ROW's field is the live RATE baseline, and it is still wiped — \
             the number the verdict rested on rides the ALERT"
        );
        assert!(row.ineligible.is_empty(), "{row:?}");

        // THE COUNTERFACTUAL: the identical vector with no flap at all raises at
        // the SAME Idle sample index, which is what "the wipe costs nothing"
        // means stated as a number.
        let mut clean = Driver::new();
        clean.warm_up_and_learn(TOPIC);
        let (clean_index, clean_stall) = clean
            .idle_until_stall(TOPIC, 200)
            .expect("the un-flapped death raises");
        assert_eq!(
            clean_index, MONITOR_CONFIRM_SAMPLES as u64,
            "precondition: an un-flapped death confirms on the fourth Idle sample"
        );
        assert_eq!(
            clean_index,
            samples_after_clear + MONITOR_CLEAR_SAMPLES as u64,
            "both vectors raise on the same Idle sample counted from the death"
        );
        assert_eq!(clean_stall.baseline_mhz, stall.baseline_mhz);
    }

    /// The generalisation: a death landing anywhere inside the re-learn window
    /// still raises, whichever condition's clear opened that window.
    ///
    /// (a) is a STALL clear — a topic that heals and immediately re-stalls, which
    /// is the crash-loop shape and the one a basis dropped on a stall clear would
    /// re-park. (b) is a STANDING deviation rather than a transient flap: a
    /// degraded topic that has been alerting for a long time and then dies, which
    /// needs no flap at all because the death itself supplies the clear.
    #[test]
    fn a_death_inside_the_relearn_window_after_any_clear_still_raises_stalled() {
        // (a) heal-then-re-stall, with the re-learn deliberately incomplete.
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        let first = d.run(MONITOR_CONFIRM_SAMPLES as u64, TOPIC, idle_dated(), None);
        assert_eq!(first.len(), 1, "{first:?}");
        assert_eq!(first[0].condition, MonitorCondition::Stalled);

        let healed = d.run(
            MONITOR_CLEAR_SAMPLES as u64,
            TOPIC,
            streaming(),
            rate(NOMINAL_MHZ),
        );
        assert_eq!(healed.len(), 1, "{healed:?}");
        assert!(healed[0].cleared_at_ms.is_some(), "the stall cleared");
        assert_eq!(d.row(TOPIC).baseline_mhz, None, "…and wiped the baseline");

        // Strictly fewer than a learning window, so nothing re-freezes.
        const PARTIAL: u64 = 3;
        const _: () = assert!(PARTIAL < MONITOR_LEARN_SAMPLES as u64);
        assert!(d
            .run(PARTIAL, TOPIC, streaming(), rate(NOMINAL_MHZ))
            .is_empty());
        assert_eq!(d.row(TOPIC).baseline_mhz, None, "still no live baseline");

        let (index, again) = d.idle_until_stall(TOPIC, 200).unwrap_or_else(|| {
            panic!(
                "the re-stall must raise inside the re-learn window: {d:?}",
                d = d.row(TOPIC)
            )
        });
        assert_eq!(index, MONITOR_CONFIRM_SAMPLES as u64);
        assert_eq!(again.baseline_mhz, Some(NOMINAL_MHZ));

        // (b) a STANDING deviation, held for far longer than any window, then a
        // death. No flap: the deviation never clears until the rate goes absent.
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        let raised = d.run(20, TOPIC, streaming(), rate(NOMINAL_MHZ / 4));
        assert_eq!(raised.len(), 1, "one raise, then held: {raised:?}");
        assert_eq!(raised[0].condition, MonitorCondition::RateDeviation);

        let cleared = d.run(MONITOR_CLEAR_SAMPLES as u64, TOPIC, idle_dated(), None);
        assert_eq!(cleared.len(), 1, "{cleared:?}");
        assert_eq!(cleared[0].condition, MonitorCondition::RateDeviation);
        let (index, stall) = d
            .idle_until_stall(TOPIC, 200)
            .expect("the standing-deviation death raises too");
        assert_eq!(
            index,
            (MONITOR_CONFIRM_SAMPLES - MONITOR_CLEAR_SAMPLES) as u64
        );
        assert_eq!(stall.baseline_mhz, Some(NOMINAL_MHZ));
    }

    /// The basis is the last FROZEN baseline, never the last READING.
    ///
    /// The distinction decides the case, because the reading immediately before a
    /// wipe is by construction the DISTURBED one: it is what raised the deviation
    /// that then cleared. Here the topic flaps to a rate BELOW
    /// `MONITOR_STALL_MIN_MHZ`, so a last-known-good keyed on the last sample
    /// would read `slow_topic` and stay silent through the death — a dead route
    /// mislabelled as a slow one, which is the swallowed-stall failure in a new costume.
    #[test]
    fn the_stall_basis_is_the_last_frozen_baseline_not_the_last_reading() {
        const DISTURBED_MHZ: u64 = MONITOR_STALL_MIN_MHZ - 100;
        const _: () = assert!(DISTURBED_MHZ * MONITOR_RATE_BAND < NOMINAL_MHZ);

        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        let raised = d.run(
            MONITOR_CONFIRM_SAMPLES as u64,
            TOPIC,
            streaming(),
            rate(DISTURBED_MHZ),
        );
        assert_eq!(raised.len(), 1, "{raised:?}");
        assert_eq!(raised[0].condition, MonitorCondition::RateDeviation);
        assert_eq!(
            d.row(TOPIC).observed_mhz,
            Some(DISTURBED_MHZ),
            "the last READING is below the stall gate"
        );

        let cleared = d.run(MONITOR_CLEAR_SAMPLES as u64, TOPIC, idle_dated(), None);
        assert_eq!(cleared.len(), 1, "{cleared:?}");
        let (_, stall) = d
            .idle_until_stall(TOPIC, 200)
            .unwrap_or_else(|| panic!("the death must page: {:?}", d.row(TOPIC)));
        assert_eq!(
            stall.baseline_mhz,
            Some(NOMINAL_MHZ),
            "the basis is what the row FROZE, not what it last measured"
        );
    }

    /// A re-learned baseline REPLACES the basis, including downwards.
    ///
    /// The basis describes the row as it stands, so a topic that legitimately
    /// slowed to below the stall gate and re-froze there must stop being watched
    /// for stalls — a one-way (or `max`) basis would keep paging it against the
    /// rate it ran at yesterday.
    ///
    /// **The SECOND wipe is what makes this an oracle.** After the slow re-freeze
    /// the live baseline is `Some(SLOW)`, and the live `Some(_) => SlowTopic` arm
    /// closes the gate on its own — so the basis arm is never consulted and a
    /// variant using a one-way basis survives. Wiping again removes the live
    /// baseline and leaves the basis as the only thing answering.
    #[test]
    fn a_re_learned_slow_baseline_replaces_the_stall_basis() {
        const SLOW_MHZ: u64 = MONITOR_STALL_MIN_MHZ - 100;
        const SLOWER_MHZ: u64 = SLOW_MHZ / 3;
        const _: () = assert!(SLOW_MHZ * MONITOR_RATE_BAND < NOMINAL_MHZ);
        const _: () = assert!(SLOWER_MHZ * MONITOR_RATE_BAND < SLOW_MHZ);

        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);

        // Wipe #1, then re-learn at the SLOW rate the topic now really runs at.
        d.flap(TOPIC, SLOW_MHZ, NOMINAL_MHZ);
        assert!(d
            .run(
                MONITOR_LEARN_SAMPLES as u64,
                TOPIC,
                streaming(),
                rate(SLOW_MHZ)
            )
            .is_empty());
        assert_eq!(
            d.row(TOPIC).baseline_mhz,
            Some(SLOW_MHZ),
            "the row re-froze at its new rate"
        );

        // Wipe #2, so the LIVE baseline is gone and only the basis can answer.
        d.flap(TOPIC, SLOWER_MHZ, SLOW_MHZ);
        assert_eq!(d.row(TOPIC).baseline_mhz, None, "the second wipe landed");

        let alerts = d.run(50, TOPIC, idle_dated(), None);
        assert!(
            alerts.is_empty(),
            "the basis is the SLOW re-freeze, so D7 excludes this row: {alerts:?}"
        );
        let row = d.row(TOPIC);
        assert!(
            row.ineligible.contains(&Ineligible {
                condition: MonitorCondition::Stalled,
                reason: IneligibleReason::SlowTopic,
            }),
            "…and it SAYS so, rather than going quiet: {row:?}"
        );
    }

    /// A wiped row whose basis is below the gate reads `slow_topic`, not
    /// `learning`.
    ///
    /// The two answers are different claims: `learning` promises that judging is
    /// still in progress, and `slow_topic` states that this row will not be
    /// judged for stalls and why. On a row that froze below the gate the second is
    /// the true one whether or not a wipe has happened, so the basis arm carries the
    /// SAME threshold test the live arm does rather than collapsing to "open or
    /// still learning".
    #[test]
    fn a_wiped_slow_baseline_reads_slow_topic_not_learning() {
        const SLOW_MHZ: u64 = MONITOR_STALL_MIN_MHZ - 1;
        const SLOWER_MHZ: u64 = SLOW_MHZ / 2 - 1;

        let mut d = Driver::new();
        d.warm_up(TOPIC);
        assert!(d
            .run(
                MONITOR_LEARN_SAMPLES as u64,
                TOPIC,
                streaming(),
                rate(SLOW_MHZ)
            )
            .is_empty());
        assert_eq!(d.row(TOPIC).baseline_mhz, Some(SLOW_MHZ));

        d.flap(TOPIC, SLOWER_MHZ, SLOW_MHZ);
        assert_eq!(d.row(TOPIC).baseline_mhz, None, "the wipe landed");

        let row = d.row(TOPIC);
        assert!(
            row.ineligible.contains(&Ineligible {
                condition: MonitorCondition::Stalled,
                reason: IneligibleReason::SlowTopic,
            }),
            "a wiped row that froze below the gate is EXCLUDED, not learning: {row:?}"
        );
        assert!(d.run(50, TOPIC, idle_dated(), None).is_empty());
    }

    /// A restart the substrate declines to RATE closes the gate before the basis
    /// can page against the old publisher's evidence.
    ///
    /// The slow-topic gate's cumulative unrated counter is ordered AHEAD of the basis for exactly
    /// this shape: a topic that comes back slower than the rate horizon is
    /// slow by observation, and the basis describes a publisher that no longer
    /// exists.
    ///
    /// Half B is the named residual (the ordering only helps if the unrated
    /// `Streaming` samples arrive BEFORE the first `Idle` stretch), pinned as a
    /// BOUND rather than left to be discovered: exactly one stall pair, then the
    /// counter closes the gate and it never flaps again.
    #[test]
    fn an_unrated_slow_restart_after_a_wipe_closes_the_stall_gate_before_it_can_flap() {
        // Half A: the unrated stretch lands first.
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        d.flap(TOPIC, DEVIATING_MHZ, NOMINAL_MHZ);
        assert!(d
            .run(MONITOR_LEARN_SAMPLES as u64, TOPIC, streaming(), None)
            .is_empty());
        let alerts = d.run(12, TOPIC, idle_dated(), None);
        assert!(
            alerts.is_empty(),
            "the counter must close the gate before the basis is consulted: {alerts:?}"
        );
        let row = d.row(TOPIC);
        assert!(
            row.ineligible.contains(&Ineligible {
                condition: MonitorCondition::Stalled,
                reason: IneligibleReason::SlowTopic,
            }),
            "{row:?}"
        );
        // …and it stays closed while the slow topic oscillates by design.
        let mut alerts = Vec::new();
        for i in 0..100 {
            let liveness = if i % 2 == 0 {
                streaming()
            } else {
                idle_dated()
            };
            // Never rated, on either half of the oscillation: that is what makes
            // it the slow-topic class rather than a stall.
            alerts.extend(d.observe(TOPIC, liveness, None));
            d.advance();
        }
        assert!(
            alerts.is_empty(),
            "a slow topic must never flap: {alerts:?}"
        );

        // Half B: the `Idle` stretch lands FIRST, inside the wiped window — the
        // documented residual. One pair, and one only.
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        d.flap(TOPIC, DEVIATING_MHZ, NOMINAL_MHZ);
        let mut pair = d.run(MONITOR_CONFIRM_SAMPLES as u64, TOPIC, idle_dated(), None);
        pair.extend(d.run(MONITOR_LEARN_SAMPLES as u64 + 4, TOPIC, streaming(), None));
        assert_eq!(
            pair.iter()
                .map(|a| (a.condition, a.cleared_at_ms.is_some()))
                .collect::<Vec<_>>(),
            vec![
                (MonitorCondition::Stalled, false),
                (MonitorCondition::Stalled, true),
            ],
            "EXACTLY one stall pair — the bound this residual is accepted at: {pair:?}"
        );
        let mut alerts = Vec::new();
        for i in 0..100 {
            let liveness = if i % 2 == 0 {
                streaming()
            } else {
                idle_dated()
            };
            alerts.extend(d.observe(TOPIC, liveness, None));
            d.advance();
        }
        assert!(
            alerts.is_empty(),
            "…and then the counter has it, for good: {alerts:?}"
        );
        assert!(d.row(TOPIC).ineligible.contains(&Ineligible {
            condition: MonitorCondition::Stalled,
            reason: IneligibleReason::SlowTopic,
        }));
    }

    /// A FLOOR sample after a wipe still poisons the basis.
    ///
    /// `ever_floor` is permanent and stays FIRST in the gate: a basis frozen
    /// before the row was ever served a labelled floor describes a measurement
    /// the substrate has since retracted, and letting it answer would launder
    /// exactly the `/tf` class D7 excludes.
    #[test]
    fn a_floor_after_a_wipe_still_poisons_the_stall_basis() {
        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);
        d.flap(TOPIC, DEVIATING_MHZ, NOMINAL_MHZ);
        assert_eq!(d.row(TOPIC).baseline_mhz, None, "the wipe landed");

        assert!(d
            .run(1, TOPIC, streaming(), floor_rate(NOMINAL_MHZ))
            .is_empty());
        let row = d.row(TOPIC);
        assert_eq!(
            row.ineligible,
            vec![
                Ineligible {
                    condition: MonitorCondition::Stalled,
                    reason: IneligibleReason::UntrustedRateBasis,
                },
                Ineligible {
                    condition: MonitorCondition::RateDeviation,
                    reason: IneligibleReason::UntrustedRateBasis,
                },
            ],
            "{row:?}"
        );
        let alerts = d.run(50, TOPIC, idle_dated(), None);
        assert!(
            alerts.is_empty(),
            "an untrusted basis must not page, wiped or not: {alerts:?}"
        );
    }

    /// The NAMED residual of the last-frozen basis, pinned as a bound.
    ///
    /// A topic that legitimately slows to a sustained rate below
    /// `MONITOR_STALL_MIN_MHZ` holds a `rate_deviation` (the deviating rate is
    /// present, so nothing clears and nothing re-learns), and its first gap long
    /// enough to fall out of the recency window clears that deviation by absence
    /// — after which the basis, still describing the FAST rate, admits one
    /// `stalled` pair.
    ///
    /// It is accepted rather than removed because it is loud-REDUNDANT and not
    /// silent-wrong: the row is already alerting, and it happens ONCE — the wipe
    /// lets the slow rate re-freeze, after which the row reads `slow_topic` and
    /// goes quiet. The alternative (an adaptive stall basis) removes it by
    /// re-creating a SILENT dead route in a different shape. This arm exists so
    /// that a future change to that trade flips a test rather than moving the
    /// boundary in silence.
    #[test]
    fn a_sustained_sub_min_rate_under_a_held_deviation_gets_one_stall_pair_then_slow_topic() {
        const SLOW_MHZ: u64 = MONITOR_STALL_MIN_MHZ - 100;
        const _: () = assert!(SLOW_MHZ * MONITOR_RATE_BAND < NOMINAL_MHZ);

        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);

        // It slows down and STAYS slow: the deviation raises once and is held.
        let raised = d.run(20, TOPIC, streaming(), rate(SLOW_MHZ));
        assert_eq!(raised.len(), 1, "raised once, then held: {raised:?}");
        assert_eq!(raised[0].condition, MonitorCondition::RateDeviation);
        assert_eq!(
            d.row(TOPIC).baseline_mhz,
            Some(NOMINAL_MHZ),
            "a held alert never re-learns, so the row is still judged on the fast rate"
        );

        // Its first long gap clears the deviation by absence — and the basis
        // admits the redundant pair.
        let gap = d.run(6, TOPIC, idle_dated(), None);
        assert_eq!(
            gap.iter()
                .map(|a| (a.condition, a.cleared_at_ms.is_some()))
                .collect::<Vec<_>>(),
            vec![
                (MonitorCondition::RateDeviation, true),
                (MonitorCondition::Stalled, false),
            ],
            "{gap:?}"
        );

        // It comes back and re-freezes at the rate it really runs at, and from
        // then on D7 excludes it — the residual is ONE pair, not a flapper.
        let resumed = d.run(
            MONITOR_LEARN_SAMPLES as u64 + MONITOR_CLEAR_SAMPLES as u64 + 2,
            TOPIC,
            streaming(),
            rate(SLOW_MHZ),
        );
        assert_eq!(
            resumed
                .iter()
                .map(|a| (a.condition, a.cleared_at_ms.is_some()))
                .collect::<Vec<_>>(),
            vec![(MonitorCondition::Stalled, true)],
            "{resumed:?}"
        );
        assert_eq!(d.row(TOPIC).baseline_mhz, Some(SLOW_MHZ), "re-frozen slow");

        let alerts = d.run(50, TOPIC, idle_dated(), None);
        assert!(alerts.is_empty(), "no second pair: {alerts:?}");
        assert!(d.row(TOPIC).ineligible.contains(&Ineligible {
            condition: MonitorCondition::Stalled,
            reason: IneligibleReason::SlowTopic,
        }));
    }

    /// The cadence at which the unrated counter stops covering a fast→slow
    /// restart, pinned on BOTH sides.
    ///
    /// The counter closes the gate after `MONITOR_LEARN_SAMPLES` unrated
    /// `Streaming` samples, and a restarted slow topic only stays `Streaming` for
    /// the substrate's recency window at a time — so the protection holds exactly
    /// while a sampler fits that many samples into one window, i.e. at cadences at
    /// or below `LIVENESS_STREAMING_RECENCY_MS / MONITOR_LEARN_SAMPLES`. bagd and
    /// the attached plane sample at 400 ms and are covered; the DISCOVERY plane
    /// rides `discover` calls and is not, which is the residual this arm bounds.
    ///
    /// Driven for THREE oscillation cycles with a rated heartbeat at the head of
    /// cycles 2 and 3, because the counter resets on any trustworthy rate: the
    /// residual recurs per rated→unrated TRANSITION, not once per row.
    #[test]
    fn a_slow_cadence_fast_to_slow_transition_is_bounded_by_the_counter_at_the_boundary() {
        /// The boundary, DERIVED from the two constants it is a ratio of.
        const BOUNDARY_MS: u64 = LIVENESS_STREAMING_RECENCY_MS / MONITOR_LEARN_SAMPLES as u64;
        assert_eq!(BOUNDARY_MS, 625, "5 s of recency over eight samples");

        // AT the boundary the counter fits inside one Streaming stretch, so the
        // only stall is the one the LIVE baseline admits before any wipe — which
        // is what the engine did before the retained basis as well.
        assert_eq!(
            stall_raises_at_cadence(BOUNDARY_MS),
            1,
            "at the boundary the counter closes the gate on every later cycle"
        );
        // One millisecond slower and it does not: the basis answers instead, once
        // per rated→unrated transition.
        assert_eq!(
            stall_raises_at_cadence(BOUNDARY_MS + 1),
            3,
            "past the boundary the residual recurs per transition"
        );
    }

    /// The observer-plane crash-loop bound (pre-existing counter semantics, named
    /// rather than fixed).
    ///
    /// After a death the substrate keeps classifying the row `Streaming` for a
    /// short unrated tail (a rate is served for up to the recency window past its
    /// last window close, and `Streaming` for up to the recency window past the
    /// last frame), and a revival too brief to be rated adds more. Together they
    /// can reach the unrated counter's threshold, so a SECOND death reads
    /// `slow_topic` — silent, and mislabelled.
    ///
    /// Changing the counter to a per-stretch run would fix it and re-open the
    /// flapper the cumulative counter exists to catch, so the bound is pinned
    /// here instead of assumed.
    #[test]
    fn a_crash_loop_second_death_is_bounded_by_the_unrated_counter() {
        const TAIL: u64 = 3;
        const REVIVAL: u64 = 5;
        const _: () = assert!(TAIL + REVIVAL >= MONITOR_LEARN_SAMPLES as u64);

        let mut d = Driver::new();
        d.warm_up_and_learn(TOPIC);

        // Death 1, with its unrated `Streaming` tail…
        assert!(d.run(TAIL, TOPIC, streaming(), None).is_empty());
        let first = d.run(MONITOR_CONFIRM_SAMPLES as u64, TOPIC, idle_dated(), None);
        assert_eq!(first.len(), 1, "the first death pages: {first:?}");
        assert_eq!(first[0].condition, MonitorCondition::Stalled);

        // …a revival too brief for the substrate to rate…
        let healed = d.run(REVIVAL, TOPIC, streaming(), None);
        assert_eq!(healed.len(), 1, "{healed:?}");
        assert!(healed[0].cleared_at_ms.is_some(), "the stall cleared");

        // …and death 2, which the counter has already excluded.
        let alerts = d.run(12, TOPIC, idle_dated(), None);
        assert!(
            alerts.is_empty(),
            "the DOCUMENTED bound: an unrated crash loop is excluded, not paged: \
             {alerts:?}"
        );
        let row = d.row(TOPIC);
        assert!(
            row.ineligible.contains(&Ineligible {
                condition: MonitorCondition::Stalled,
                reason: IneligibleReason::SlowTopic,
            }),
            "…and it says so rather than going quiet: {row:?}"
        );
    }

    /// The flap-then-death vector is a pure function of its samples.
    ///
    /// Determinism is asserted elsewhere over a healthy script; this drives the
    /// retained-basis path specifically, so the retained basis is covered by Principle
    /// #7's own oracle rather than merely being believed to be pure.
    #[test]
    fn the_flap_then_death_vector_is_byte_identical_across_engines() {
        let script = |d: &mut Driver| -> Vec<Alert> {
            let mut out = d.warm_up_and_learn(TOPIC);
            out.extend(d.run(
                MONITOR_CONFIRM_SAMPLES as u64,
                TOPIC,
                streaming(),
                rate(DEVIATING_MHZ),
            ));
            out.extend(d.run(30, TOPIC, idle_dated(), None));
            out
        };

        let mut first = Driver::new();
        let a = script(&mut first);
        let mut second = Driver::new();
        let b = script(&mut second);
        assert_eq!(a, b, "two runs of one script must agree");
        assert_eq!(
            first.engine.rows(first.now_ns),
            second.engine.rows(second.now_ns)
        );
        assert_eq!(first.engine.next_seq(), second.engine.next_seq());

        // The hand oracle, so this is not purely a self-compare.
        assert_eq!(
            a.iter()
                .map(|alert| (alert.seq, alert.condition, alert.cleared_at_ms.is_some()))
                .collect::<Vec<_>>(),
            vec![
                (0, MonitorCondition::RateDeviation, false),
                (1, MonitorCondition::RateDeviation, true),
                (2, MonitorCondition::Stalled, false),
            ],
            "{a:?}"
        );
        assert_eq!(a[2].baseline_mhz, Some(NOMINAL_MHZ));
    }

    /// One fast→slow-unrated restart driven at `cadence_ms`, answering how many
    /// `stalled` RAISES it produced. Arm 12's oracle target.
    ///
    /// Deliberately off the production grid, which is why it advances `now_ns` by
    /// hand rather than using [`Driver::run`].
    fn stall_raises_at_cadence(cadence_ms: u64) -> usize {
        let cadence_ns = cadence_ms * 1_000_000;
        // How many unrated `Streaming` samples ONE recency-window-long streaming
        // stretch yields at this cadence — the quantity the boundary is about.
        let stretch = LIVENESS_STREAMING_RECENCY_MS / cadence_ms;

        let mut d = Driver::new();
        let run = |d: &mut Driver, n: u64, liveness, rate| -> Vec<Alert> {
            let mut out = Vec::new();
            for _ in 0..n {
                out.extend(d.observe(TOPIC, liveness, rate));
                d.now_ns += cadence_ns;
            }
            out
        };

        // Settle, then FREEZE a fast baseline: the row earns its basis.
        run(
            &mut d,
            MONITOR_SETTLE_NS.div_ceil(cadence_ns),
            streaming(),
            rate(NOMINAL_MHZ),
        );
        let mut alerts = run(
            &mut d,
            MONITOR_LEARN_SAMPLES as u64,
            streaming(),
            rate(NOMINAL_MHZ),
        );

        // Three cycles of the restarted publisher: `Streaming` but unrated, then
        // out of the recency window. The rated heartbeat opening cycles 2 and 3
        // RESETS the cumulative counter — the per-transition recurrence.
        for cycle in 0..3 {
            if cycle > 0 {
                alerts.extend(run(&mut d, 1, streaming(), rate(NOMINAL_MHZ)));
            }
            alerts.extend(run(&mut d, stretch, streaming(), None));
            alerts.extend(run(
                &mut d,
                MONITOR_CONFIRM_SAMPLES as u64,
                idle_dated(),
                None,
            ));
        }

        alerts
            .iter()
            .filter(|a| a.condition == MonitorCondition::Stalled && a.cleared_at_ms.is_none())
            .count()
    }
}
