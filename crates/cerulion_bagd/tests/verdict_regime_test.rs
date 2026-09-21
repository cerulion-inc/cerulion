// SPDX-License-Identifier: AGPL-3.0-only
//! Trigger 3: the monitors-verdict observer driven END TO END against
//! the REAL [`FlashbackTriggerGate`].
//!
//! # What this proves that the module's own unit arms cannot
//!
//! `verdict_observer`'s in-module tests cover the pure mapping — subject, detail,
//! raise-vs-clear. They are blind to the only question that matters for the
//! not-spammy requirement, which is what the gate does with that stream:
//! a mapping that produced a perfectly-formed `CaptureRequest` for every sample
//! would pass all of them while writing a ~155 MB bag every 400 ms.
//!
//! So every arm here composes the three real pieces — the shared
//! [`MonitorEngine`](cerulion_core::monitor::MonitorEngine) inside
//! [`VerdictObserver`], the mapping, and a real gate — and asserts the DECISION
//! SEQUENCE against a hand-written oracle.
//!
//! # Why the samples are hand-built, and why that is not "fake data"
//!
//! [`VerdictObserver::observe_samples`] is the module's declared drivable seam
//! (see its docs). Feeding it a `MonitorSample` vector is the seam-as-parameter
//! pattern, not a fabricated measurement: the SAMPLE is an input the production
//! path also constructs, and everything downstream of it — the confirmation
//! windows, the settle window, the eligibility gates, the ring, the latch, the
//! refractory floor, the rate cap — is the shipping code.
//!
//! Driving these arms through real transport instead would mean publishing real
//! frames and then WAITING OUT `MONITOR_CONFIRM_MIN_SPAN_NS` (1.2 s) plus the
//! 5 s settle window for every one of them, turning eight arms of pure
//! arithmetic into a minutes-long load-sensitive suite, the load-inversion class. The
//! transport half (a real drain feeding `note_drain`, a real record book) is
//! covered where it belongs, by the liveness substrate's own real-iceoryx2 arms.
//!
//! # No walls
//!
//! Every clock in this file is a `u64` the test advances by hand, so nothing
//! here can flake under load. The oracles are COUNTS and decision sequences.
//!
//! Parallel-safe: isolated per-test SHM roots via the `common` harness, no
//! `#[serial]`. The observer opens no port (it never calls `sweep()`), so the
//! manager exists only to give the record book a clock.

mod common;

use std::collections::BTreeMap;

use cerulion_bagd::verdict_observer::{VerdictAction, VerdictObserver, VERDICT_KIND};
use cerulion_core::flashback::switch::{TriggerPosture, TriggerSwitch};
use cerulion_core::flashback::trigger::{
    CaptureRequest, FlashbackTriggerGate, SuppressReason, TriggerDecision, TriggerPolicy,
};
use cerulion_core::monitor::{
    MonitorSample, MONITOR_CONFIRM_MIN_SPAN_NS, MONITOR_SAMPLE_INTERVAL_NS,
};
use cerulion_core::transport::liveness::{TopicLiveness, TopicRateEstimate};
use cerulion_core::LivenessState;

const MS: u64 = 1_000_000;

// ---------------------------------------------------------------------------
// Hand-built samples.
// ---------------------------------------------------------------------------

/// A `Streaming` observation carrying a trustworthy rate.
///
/// `is_floor: false` is load-bearing: the engine's D7 gate makes a row that has
/// EVER been served a floor-basis rate permanently ineligible for `stalled`, so
/// a floor here would make every arm in this file vacuous.
fn streaming(at_ns: u64, mhz: u64) -> TopicLiveness {
    let l = TopicLiveness {
        last_frame_age_ms: Some(0),
        observed_for_ms: at_ns / MS,
        frames_observed: 1_000,
        rate_estimate: Some(TopicRateEstimate {
            millihertz: mhz,
            is_floor: false,
        }),
    };
    assert_eq!(
        l.state(),
        LivenessState::Streaming,
        "the fixture must really classify Streaming, or every arm below is vacuous"
    );
    l
}

/// An `Idle` observation — frames were seen, none recently. This is what a
/// stalled topic looks like to the substrate.
fn idle(at_ns: u64, age_ms: u64) -> TopicLiveness {
    let l = TopicLiveness {
        last_frame_age_ms: Some(age_ms),
        observed_for_ms: at_ns / MS,
        frames_observed: 1_000,
        rate_estimate: None,
    };
    assert_eq!(l.state(), LivenessState::Idle, "the fixture must be Idle");
    l
}

/// An observation carrying NO evidence — the shape a topic outside the
/// observer's budget, or one on a robot with liveness disabled, produces.
fn blind() -> Option<TopicLiveness> {
    None
}

/// A `NoData` observation on a route that NEVER produced — the shape `silent`
/// exists for (`/uslam/cloud_map`: registered at graph build, never streamed).
fn never_produced(at_ns: u64) -> TopicLiveness {
    // The observation interval opened `LIVENESS_NO_DATA_MIN_MS` BEFORE this
    // sample, which is the shipping shape and not a convenience: the recorder's
    // tap is armed at bring-up, long before the monitor's first pass, so by the
    // time a verdict can confirm the substrate has been watching for a while.
    // Without it the fixture reads `Unknown` — a route the observer has not
    // watched long enough to judge — and every arm below would be vacuous.
    let l = TopicLiveness {
        last_frame_age_ms: None,
        observed_for_ms: at_ns / MS + cerulion_core::transport::liveness::LIVENESS_NO_DATA_MIN_MS,
        frames_observed: 0,
        rate_estimate: None,
    };
    assert_eq!(
        l.state(),
        LivenessState::NoData,
        "the fixture must really classify NoData, or the silent arms are vacuous"
    );
    l
}

fn sample(topic: &str, at_ns: u64, liveness: Option<TopicLiveness>) -> MonitorSample {
    MonitorSample::new(topic, None, at_ns, liveness, true)
}

// ---------------------------------------------------------------------------
// The harness: observer + real gate, driven on a hand-advanced clock.
// ---------------------------------------------------------------------------

/// How many samples a fresh row spends inside the engine's settle window.
///
/// DERIVED from the shipped constants rather than written down, so a substrate
/// change re-derives the warm-up instead of silently shortening it.
fn settle_passes() -> u64 {
    cerulion_core::monitor::MONITOR_SETTLE_NS.div_ceil(MONITOR_SAMPLE_INTERVAL_NS) + 1
}

/// Passes needed to CONFIRM a condition once it starts qualifying — the count
/// AND the span, whichever binds.
fn confirm_passes() -> u64 {
    let by_span = MONITOR_CONFIRM_MIN_SPAN_NS.div_ceil(MONITOR_SAMPLE_INTERVAL_NS) + 1;
    by_span.max(u64::from(cerulion_core::monitor::MONITOR_CONFIRM_SAMPLES))
}

struct Harness {
    observer: VerdictObserver,
    gate: FlashbackTriggerGate,
    now_ns: u64,
    /// Every decision the gate made, in order — the oracle target.
    decisions: Vec<TriggerDecision>,
    /// Every capture REQUEST that reached the gate, in order.
    ///
    /// A [`TriggerDecision`] carries no subject and no detail, so the arms that
    /// assert WHICH condition reached the gate — and with what evidence — read
    /// the request instead of the verdict.
    requested: Vec<CaptureRequest>,
    /// Per-subject recovery reports, for the arms that assert what a regime
    /// swallowed.
    recovered: Vec<(String, Option<u64>)>,
}

impl Harness {
    fn new(policy: TriggerPolicy) -> Self {
        Self::with_posture(policy, TriggerPosture::default())
    }

    /// A harness under an explicit per-trigger posture.
    ///
    /// The POSTURE is handed to BOTH halves from one value, exactly as the
    /// recorder does — two resolutions of one environment is the
    /// two-copies class, and here the halves would disagree about what is on.
    fn with_posture(policy: TriggerPolicy, posture: TriggerPosture) -> Self {
        // The manager gives the record book a clock and nothing else: this
        // harness never calls `note_drain`, so no liveness record is ever
        // written and no port is ever opened.
        let mgr = common::make_manager(16);
        Self {
            observer: VerdictObserver::with_posture(mgr, posture),
            gate: FlashbackTriggerGate::with_posture(policy, posture),
            now_ns: 0,
            decisions: Vec::new(),
            requested: Vec::new(),
            recovered: Vec::new(),
        }
    }

    /// Advance one sampling interval and drive ONE pass over `samples`, applying
    /// whatever it asks for to the real gate.
    fn pass(&mut self, samples: &[MonitorSample]) {
        self.now_ns += MONITOR_SAMPLE_INTERVAL_NS;
        // The engine's own clock and the gate's are the SAME number line here on
        // purpose: production keeps them apart (the record book rides the
        // manager's clock, the gate the recorder's drive-loop origin), and this
        // file asserts the gate's REACTION to a stream of actions, which is
        // independent of the two origins.
        let now = self.now_ns;
        let actions = self.observer.observe_samples(samples, now);
        for action in actions {
            match action {
                VerdictAction::Capture(request) => {
                    let decision = self.gate.decide(&request, now);
                    self.requested.push(request);
                    self.decisions.push(decision);
                }
                VerdictAction::Recover { subject } => {
                    let suppressed = self.gate.recover(VERDICT_KIND, &subject);
                    self.recovered.push((subject, suppressed));
                }
            }
        }
    }

    /// Jump the clock a full refractory floor forward.
    ///
    /// The floor is stamped at CAPTURE time and deliberately SURVIVES a recovery
    /// (the gate's flapper rule), so a genuinely-new occurrence of a healed
    /// condition is only news once the floor has elapsed. A jump from `now` is
    /// always sufficient because the stamp is necessarily in the past.
    fn advance_past_refractory(&mut self) {
        self.now_ns += self.gate.policy().refractory_ns + MS;
    }

    /// Close whatever capture is open, so the next request is not merely
    /// coalesced into it.
    fn close_capture(&mut self) {
        if let Some((_, ends_at)) = self.gate.active_capture() {
            self.now_ns = self.now_ns.max(ends_at) + MS;
            self.gate.finish_capture();
        }
    }

    fn captures(&self) -> usize {
        self.decisions
            .iter()
            .filter(|d| matches!(d, TriggerDecision::Capture { .. }))
            .count()
    }

    fn regime_open_suppressions(&self) -> usize {
        self.decisions
            .iter()
            .filter(|d| {
                matches!(
                    d,
                    TriggerDecision::Suppressed(SuppressReason::RegimeOpen { .. })
                )
            })
            .count()
    }
}

/// Drive one topic from healthy to stalled, returning the harness at the instant
/// the stall is CONFIRMED (i.e. the gate has seen exactly one request).
fn stall_one_topic(policy: TriggerPolicy, topic: &str) -> Harness {
    let mut h = Harness::new(policy);
    let healthy: Vec<MonitorSample> = Vec::new();
    let _ = healthy;
    // Warm up: settle the row, then learn a baseline so `stalled` is eligible.
    for _ in 0..(settle_passes() + u64::from(cerulion_core::monitor::MONITOR_LEARN_SAMPLES) + 2) {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = vec![sample(topic, at, Some(streaming(at, 100_000)))];
        h.pass(&s);
    }
    assert_eq!(
        h.captures(),
        0,
        "a healthy topic must not capture anything while it is streaming"
    );
    // Now stall it.
    for _ in 0..confirm_passes() {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = vec![sample(topic, at, Some(idle(at, 30_000)))];
        h.pass(&s);
    }
    h
}

// ---------------------------------------------------------------------------
// Arms.
// ---------------------------------------------------------------------------

/// THE headline: the full regime lifecycle against the real gate.
///
/// A condition going bad captures ONCE; while it stays bad NOTHING FURTHER IS
/// ASKED FOR AT ALL; the recovery edge closes the regime; and the SAME condition
/// going bad again is news, so it captures again.
///
/// The last step is the one the recovery edge exists for: without
/// `VerdictAction::Recover` driving `gate.recover`, the regime stays open for the
/// rest of the run and a second, genuinely new occurrence is silently swallowed.
///
/// **The middle step is where the two suppression layers show through**, and this
/// arm asserts which one does the work: the engine's raise is a ONE-SHOT, so a
/// steadily-stalled topic constructs no second request and the gate's own
/// `requests` counter stays at 1. The gate latch behind it is the second line,
/// covering the shape the engine cannot see (a duplicate observer, a clear that
/// never reached the gate).
#[test]
fn a_verdict_regime_captures_once_asks_nothing_further_and_re_arms_on_recovery() {
    let topic = "/lowstate";
    let mut h = stall_one_topic(TriggerPolicy::default(), topic);

    assert_eq!(
        h.captures(),
        1,
        "a confirmed stall must capture EXACTLY once, got {:?}",
        h.decisions
    );

    // Close the capture so the repeats below are not merely absorbed by
    // coalescing (a different mechanism, with its own arm).
    h.close_capture();

    // The topic is still stalled. Nothing further may be asked for.
    let before = h.decisions.len();
    for _ in 0..6 {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = vec![sample(topic, at, Some(idle(at, 60_000)))];
        h.pass(&s);
    }
    assert_eq!(
        h.decisions.len(),
        before,
        "a condition that is still bad must not reach the gate AT ALL — the engine's \
         raise is a one-shot. new decisions: {:?}",
        &h.decisions[before..]
    );
    assert_eq!(
        h.gate.stats().requests,
        1,
        "and the gate's own unconditional counter must agree: one request, ever"
    );

    // Heal it. The clearing edge must reach the gate. The heal is deliberately
    // long — it also re-learns the RATE baseline — but `stalled` eligibility no
    // longer waits on that: the engine keeps the last frozen baseline as the stall
    // basis, so what a shorter heal would change here is the `rate_deviation`
    // band, not whether the re-stall below can confirm.
    let heal_passes =
        u64::from(cerulion_core::monitor::MONITOR_LEARN_SAMPLES) + settle_passes() + 4;
    for _ in 0..heal_passes {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = vec![sample(topic, at, Some(streaming(at, 100_000)))];
        h.pass(&s);
    }
    assert_eq!(
        h.recovered.len(),
        1,
        "the clearing edge must drive exactly one recover, got {:?}",
        h.recovered
    );
    assert_eq!(
        h.recovered[0].0,
        format!("stalled:local:{topic}"),
        "recover must name the SAME subject the capture was keyed on, or it finds no \
         regime and leaves the real one open forever"
    );
    assert_eq!(
        h.recovered[0].1,
        Some(0),
        "the regime WAS open, and it swallowed nothing — because the engine's \
         one-shot meant nothing was re-asked. Some(0) is the correct report; None \
         would mean the recover found no regime at all"
    );

    // And now the headline: a SECOND, genuinely new stall must capture again —
    // once the refractory floor has elapsed (see the flapper arm below, which is
    // the other side of that same boundary).
    h.advance_past_refractory();
    for _ in 0..confirm_passes() {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = vec![sample(topic, at, Some(idle(at, 30_000)))];
        h.pass(&s);
    }
    assert_eq!(
        h.captures(),
        2,
        "a condition that recovered and went bad AGAIN is news and must capture; \
         without the recovery edge the regime stays open forever. decisions: {:?}",
        h.decisions
    );
}

/// A FLAPPER re-firing inside the refractory floor is refused, and told when it
/// may try again.
///
/// This is the other side of the headline's boundary, and it is the shape the
/// floor exists for: `recover` deliberately does NOT clear the refractory stamp,
/// so a condition oscillating faster than the floor cannot capture on every
/// rising edge. Without it a topic flapping every few seconds would write a
/// ~155 MB bag per oscillation — the disk-filling failure mode the whole gate is
/// built around.
///
/// It also fixes which of the two suppression arms answers: `Refractory`, not
/// `RegimeOpen`, because the regime really was closed by the recovery. Reporting
/// the arm the operator can act on is the gate's stated ordering rule.
#[test]
fn a_flapper_re_firing_inside_the_refractory_floor_is_refused_and_told_when_to_retry() {
    let topic = "/lowstate";
    let mut h = stall_one_topic(TriggerPolicy::default(), topic);
    assert_eq!(h.captures(), 1, "precondition: the first stall captured");
    h.close_capture();

    // Heal it. Long enough to re-learn the RATE baseline as well, though the
    // re-stall's eligibility rides the retained stall basis and would
    // hold for a shorter heal too — what this arm is about is the FLOOR, which is
    // stamped at capture time and survives the recovery either way.
    let heal_passes =
        u64::from(cerulion_core::monitor::MONITOR_LEARN_SAMPLES) + settle_passes() + 4;
    for _ in 0..heal_passes {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = vec![sample(topic, at, Some(streaming(at, 100_000)))];
        h.pass(&s);
    }
    assert_eq!(h.recovered.len(), 1, "precondition: the regime recovered");

    // Re-stall WITHOUT waiting out the floor. Deliberately no `advance_past_refractory`.
    for _ in 0..confirm_passes() {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = vec![sample(topic, at, Some(idle(at, 30_000)))];
        h.pass(&s);
    }

    assert_eq!(
        h.captures(),
        1,
        "a flapper must not capture on every rising edge. decisions: {:?}",
        h.decisions
    );
    let refused: Vec<&TriggerDecision> = h
        .decisions
        .iter()
        .filter(|d| {
            matches!(
                d,
                TriggerDecision::Suppressed(SuppressReason::Refractory { .. })
            )
        })
        .collect();
    assert_eq!(
        refused.len(),
        1,
        "and it must be refused as REFRACTORY — the regime was closed by the \
         recovery, so `RegimeOpen` would be the wrong reason and would point an \
         operator at the wrong mechanism. decisions: {:?}",
        h.decisions
    );
    match refused[0] {
        TriggerDecision::Suppressed(SuppressReason::Refractory { retry_in_ns }) => {
            assert!(
                *retry_in_ns > 0,
                "the refusal must say WHEN the condition may capture again, not merely \
                 that it may not now"
            );
            assert!(
                *retry_in_ns <= h.gate.policy().refractory_ns,
                "and that wait can never exceed the floor itself, got {retry_in_ns}"
            );
        }
        other => panic!("expected a refractory refusal, got {other:?}"),
    }
}

/// A blind sample never asks for anything.
///
/// `Unknown`, an absent liveness payload and an unconverged discovery plane are
/// the three shapes the engine collapses into "nothing was observed that anyone
/// may act on". A watchdog that read absence as trouble would capture a bag
/// every time a robot's observer tap budget filled.
#[test]
fn a_blind_sample_never_asks_the_gate_for_anything() {
    let mut h = Harness::new(TriggerPolicy::default());
    let topic = "/lowstate";

    // Many passes, all blind, well past every confirmation window.
    for _ in 0..(settle_passes() + confirm_passes() + 20) {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = vec![sample(topic, at, blind())];
        h.pass(&s);
    }

    assert!(
        h.decisions.is_empty(),
        "a blind observation is not a condition — the gate must never be asked, got {:?}",
        h.decisions
    );
    assert!(
        h.recovered.is_empty(),
        "and it is not an all-clear either, got {:?}",
        h.recovered
    );
    assert_eq!(
        h.gate.stats().requests,
        0,
        "the gate's own unconditional counter must agree"
    );
}

/// Three topics stalling together are ONE incident, so they COALESCE into one
/// bag with three causes — not three bags.
///
/// This is the mechanism the trigger seam asks FIRST, and deliberately: a member
/// of a burst suppressed by the capture would be dropped from the bag's own
/// cause list AND left un-latched, so the same fault would capture again the
/// moment the bag finished.
#[test]
fn topics_stalling_together_coalesce_into_one_capture_with_every_cause() {
    let topics = ["/a", "/b", "/c"];
    let mut h = Harness::new(TriggerPolicy::default());

    let healthy = |at: u64| -> Vec<MonitorSample> {
        topics
            .iter()
            .map(|t| sample(t, at, Some(streaming(at, 100_000))))
            .collect()
    };
    let stalled = |at: u64| -> Vec<MonitorSample> {
        topics
            .iter()
            .map(|t| sample(t, at, Some(idle(at, 30_000))))
            .collect()
    };

    for _ in 0..(settle_passes() + u64::from(cerulion_core::monitor::MONITOR_LEARN_SAMPLES) + 2) {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = healthy(at);
        h.pass(&s);
    }
    for _ in 0..confirm_passes() {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = stalled(at);
        h.pass(&s);
    }

    assert_eq!(
        h.captures(),
        1,
        "three topics failing together are ONE moment and must open ONE capture, got {:?}",
        h.decisions
    );
    let extends = h
        .decisions
        .iter()
        .filter(|d| matches!(d, TriggerDecision::Extend { .. }))
        .count();
    assert_eq!(
        extends, 2,
        "the other two must JOIN that capture rather than be suppressed by it, got {:?}",
        h.decisions
    );

    // Every topic must be recorded as a cause, under its own subject.
    let causes: Vec<String> = h
        .gate
        .active_causes()
        .into_iter()
        .map(|c| c.subject)
        .collect();
    let mut expected: Vec<String> = topics
        .iter()
        .map(|t| format!("stalled:local:{t}"))
        .collect();
    expected.sort();
    let mut got = causes.clone();
    got.sort();
    assert_eq!(
        got, expected,
        "the bag must record which topics failed, not merely that something did"
    );
}

/// Two DIFFERENT conditions are two regimes, and one open regime must never
/// swallow the other's head.
///
/// The subject carries the condition as well as the topic, so a stall on `/a` and
/// a stall on `/b` are different regimes — and a regime that is open for one must
/// not suppress the first occurrence of the other. This is the `FailureRegimeLatch`
/// separateness discipline, applied to the gate.
#[test]
fn an_open_regime_does_not_swallow_a_different_subjects_first_capture() {
    let mut h = Harness::new(TriggerPolicy::default());

    // Warm both topics up.
    for _ in 0..(settle_passes() + u64::from(cerulion_core::monitor::MONITOR_LEARN_SAMPLES) + 2) {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = vec![
            sample("/a", at, Some(streaming(at, 100_000))),
            sample("/b", at, Some(streaming(at, 100_000))),
        ];
        h.pass(&s);
    }

    // Stall /a alone, and let its capture finish.
    for _ in 0..confirm_passes() {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = vec![
            sample("/a", at, Some(idle(at, 30_000))),
            sample("/b", at, Some(streaming(at, 100_000))),
        ];
        h.pass(&s);
    }
    assert_eq!(h.captures(), 1, "/a's stall must capture");
    h.close_capture();

    // Now stall /b while /a's regime is still open.
    for _ in 0..confirm_passes() {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = vec![
            sample("/a", at, Some(idle(at, 60_000))),
            sample("/b", at, Some(idle(at, 30_000))),
        ];
        h.pass(&s);
    }

    assert_eq!(
        h.captures(),
        2,
        "/b's FIRST failure is news even while /a's regime is open — a subject that \
         merges the two topics would suppress it. decisions: {:?}",
        h.decisions
    );
    assert_eq!(
        h.gate.stats().requests,
        2,
        "and EXACTLY two requests were ever made: /a's raise and /b's. /a's continued \
         stall contributes nothing (the engine's one-shot), so a third request would \
         mean the raise is being re-minted"
    );
    assert_eq!(
        h.regime_open_suppressions(),
        0,
        "nothing is suppressed BY THE GATE on this path — the engine suppressed it a \
         layer earlier. Asserting this rather than leaving it unstated is what keeps \
         the two layers' roles distinct"
    );
}

/// The global rate cap is the disk backstop, and it is reported with the numbers
/// an operator can act on.
///
/// Driven with a cap of ONE so the second distinct condition is refused, which is
/// the arm a default cap of 20 would need twenty warm-up topics to reach.
#[test]
fn past_the_rate_cap_a_verdict_is_suppressed_loudly_and_counted() {
    let policy = TriggerPolicy {
        max_per_hour: 1,
        ..TriggerPolicy::default()
    };
    let mut h = Harness::new(policy);

    for _ in 0..(settle_passes() + u64::from(cerulion_core::monitor::MONITOR_LEARN_SAMPLES) + 2) {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = vec![
            sample("/a", at, Some(streaming(at, 100_000))),
            sample("/b", at, Some(streaming(at, 100_000))),
        ];
        h.pass(&s);
    }

    // /a stalls, captures, and its capture is closed.
    for _ in 0..confirm_passes() {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = vec![
            sample("/a", at, Some(idle(at, 30_000))),
            sample("/b", at, Some(streaming(at, 100_000))),
        ];
        h.pass(&s);
    }
    assert_eq!(h.captures(), 1);
    h.close_capture();

    // /b stalls. The cap is spent.
    for _ in 0..confirm_passes() {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = vec![
            sample("/a", at, Some(idle(at, 60_000))),
            sample("/b", at, Some(idle(at, 30_000))),
        ];
        h.pass(&s);
    }

    assert_eq!(
        h.captures(),
        1,
        "the hourly cap is spent — a second capture must not be written. decisions: {:?}",
        h.decisions
    );
    let capped: Vec<&TriggerDecision> = h
        .decisions
        .iter()
        .filter(|d| {
            matches!(
                d,
                TriggerDecision::Suppressed(SuppressReason::RateCapped { .. })
            )
        })
        .collect();
    assert_eq!(
        capped.len(),
        1,
        "and the refusal must name the CAP rather than the condition, so the operator \
         is told which knob to turn. decisions: {:?}",
        h.decisions
    );
    match capped[0] {
        TriggerDecision::Suppressed(SuppressReason::RateCapped {
            captures_in_window,
            cap,
            reserved_for_manual,
        }) => {
            assert_eq!(*cap, 1, "the refusal must quote the cap in force");
            assert_eq!(
                *captures_in_window, 1,
                "and how many captures are inside the window"
            );
            // By design, at a cap of one the manual reserve clamps to
            // zero, so an automatic kind still sees the whole budget. Asserted
            // here because a flat subtraction would make this arm never capture
            // at all — the reserve must never disable the triggers it protects.
            assert_eq!(
                *reserved_for_manual, 0,
                "a cap of 1 leaves nothing to reserve"
            );
        }
        other => panic!("expected a rate-cap refusal, got {other:?}"),
    }
    assert_eq!(
        h.gate.stats().rate_capped,
        1,
        "the unconditional counter must record it independently of the log"
    );
}

/// The observer outliving its sample source mints nothing.
///
/// A recorder whose tap set goes empty — every producer gone — must not conclude
/// anything. This is the same rule as the blind arm, from the other direction:
/// there is no sample at all rather than a sample with no evidence.
#[test]
fn an_observer_whose_topics_disappear_asks_for_nothing() {
    let topic = "/lowstate";
    let mut h = stall_one_topic(TriggerPolicy::default(), topic);
    assert_eq!(h.captures(), 1, "precondition: the stall captured");
    h.close_capture();

    let before = h.decisions.len();
    let watched_before = h.observer.watched();

    // The producer is gone and so is the tap: nothing to sample.
    for _ in 0..40 {
        h.pass(&[]);
    }

    assert_eq!(
        h.decisions.len(),
        before,
        "an empty sample set must ask the gate for nothing, got {:?}",
        &h.decisions[before..]
    );
    assert!(
        h.recovered.is_empty(),
        "and it must not be read as an all-clear either — a topic that vanished did \
         not recover. recovered: {:?}",
        h.recovered
    );
    assert_eq!(
        h.observer.watched(),
        watched_before,
        "the row survives (nothing resurrects or forgets it), so a later sample \
         resumes rather than starting over"
    );
}

/// The sampler is THROTTLED, and the throttle lives in the observer.
///
/// The engine confirms on a COUNT of samples as well as a span, so a caller
/// driving it at the recorder's ~1 kHz drive-loop rate would confirm a condition
/// inside the window a lull-free restart is still healing in, which is a false
/// `stalled` on a topic that is fine. Pinned on BOTH sides of the boundary.
#[test]
fn the_sampler_is_throttled_to_the_derived_interval_on_both_sides() {
    let mut h = Harness::new(TriggerPolicy::default());
    let topic = "/lowstate";

    // First pass is always due.
    let at = MONITOR_SAMPLE_INTERVAL_NS;
    let s = vec![sample(topic, at, Some(streaming(at, 100_000)))];
    let _ = h.observer.observe_samples(&s, at);
    let after_first = h.observer.passes();
    assert_eq!(after_first, 1, "the first pass is never throttled out");

    // One nanosecond SHORT of the interval: refused.
    let too_soon = at + MONITOR_SAMPLE_INTERVAL_NS - 1;
    let s = vec![sample(topic, too_soon, Some(streaming(too_soon, 100_000)))];
    let actions = h.observer.observe_samples(&s, too_soon);
    assert!(actions.is_empty(), "an un-due pass yields nothing");
    assert_eq!(
        h.observer.passes(),
        after_first,
        "and it must not COUNT either — a throttled-out pass observed nothing, so a \
         counter that moved would make the sampler's own liveness unreadable"
    );

    // Exactly AT the interval: admitted.
    let due = at + MONITOR_SAMPLE_INTERVAL_NS;
    let s = vec![sample(topic, due, Some(streaming(due, 100_000)))];
    let _ = h.observer.observe_samples(&s, due);
    assert_eq!(
        h.observer.passes(),
        after_first + 1,
        "at the boundary the pass is due"
    );
}

/// Determinism (Principle #7): the same sample vector twice yields the same
/// decision sequence.
///
/// Compared against a HAND oracle as well as against the sibling run, so a pair
/// of runs that were both wrong in the same way cannot pass.
#[test]
fn the_same_sample_stream_yields_the_same_decisions_twice() {
    let run = || -> Vec<String> {
        let mut h = stall_one_topic(TriggerPolicy::default(), "/lowstate");
        h.close_capture();
        for _ in 0..4 {
            let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
            let s = vec![sample("/lowstate", at, Some(idle(at, 60_000)))];
            h.pass(&s);
        }
        h.decisions
            .iter()
            .map(|d| match d {
                TriggerDecision::Capture { .. } => "capture".to_string(),
                TriggerDecision::Extend { .. } => "extend".to_string(),
                TriggerDecision::Suppressed(SuppressReason::RegimeOpen { suppressed }) => {
                    format!("regime_open:{suppressed}")
                }
                TriggerDecision::Suppressed(r) => format!("suppressed:{r:?}"),
            })
            .collect()
    };

    let a = run();
    let b = run();
    assert_eq!(a, b, "two identical runs must decide identically");
    assert_eq!(
        a,
        vec!["capture".to_string()],
        "and the sequence must match the HAND oracle — ONE capture and nothing else, \
         because the engine's raise is a one-shot and the four further stalled passes \
         construct no request at all"
    );
}

/// The subject really is what keys the regime, checked by taking the gate's own
/// word for it rather than by inspecting the string.
///
/// A topic that stalls opens a regime under ITS OWN subject, and a healthy
/// sibling opens none — so exactly one recovery is reported, naming exactly that
/// topic. The count it carries is `Some(0)`: the regime was open (which is why it
/// is `Some` rather than `None`) and swallowed nothing, because the engine's
/// one-shot means a steadily-stalled topic is never re-asked.
///
/// The `Some(0)` vs `None` distinction is the whole assertion: `None` is what
/// `recover` returns when it finds NO regime for that `(kind, subject)` pair,
/// i.e. what a subject-mismatch bug produces.
#[test]
fn a_stall_opens_a_regime_under_its_own_subject_and_a_healthy_sibling_opens_none() {
    let mut h = Harness::new(TriggerPolicy::default());

    for _ in 0..(settle_passes() + u64::from(cerulion_core::monitor::MONITOR_LEARN_SAMPLES) + 2) {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = vec![
            sample("/a", at, Some(streaming(at, 100_000))),
            sample("/b", at, Some(streaming(at, 100_000))),
        ];
        h.pass(&s);
    }

    // /a stalls first, capture closed.
    for _ in 0..confirm_passes() {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = vec![
            sample("/a", at, Some(idle(at, 30_000))),
            sample("/b", at, Some(streaming(at, 100_000))),
        ];
        h.pass(&s);
    }
    h.close_capture();

    // /a repeats three times while /b stays healthy.
    for _ in 0..3 {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = vec![
            sample("/a", at, Some(idle(at, 60_000))),
            sample("/b", at, Some(streaming(at, 100_000))),
        ];
        h.pass(&s);
    }

    // Heal /a and read what its regime reports.
    for _ in 0..(u64::from(cerulion_core::monitor::MONITOR_CLEAR_SAMPLES) + 2) {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        let s = vec![
            sample("/a", at, Some(streaming(at, 100_000))),
            sample("/b", at, Some(streaming(at, 100_000))),
        ];
        h.pass(&s);
    }

    let by_subject: BTreeMap<String, Option<u64>> = h.recovered.iter().cloned().collect();
    assert_eq!(
        by_subject.len(),
        1,
        "only /a's regime was ever open, so only /a recovers, got {by_subject:?}"
    );
    assert_eq!(
        by_subject.get("stalled:local:/a").copied(),
        Some(Some(0)),
        "SOME means the gate found a regime under exactly this subject — a mismatch \
         between the capture's subject and the recovery's would read None here and \
         leave the real regime open forever. ZERO because the engine's one-shot meant \
         /a's continued stall was never re-asked"
    );
    assert!(
        !by_subject.contains_key("stalled:local:/b"),
        "and a topic that never failed must open no regime at all, got {by_subject:?}"
    );
}

/// A `rate_deviation` that clears BECAUSE the publisher died must not
/// swallow the stall on its way out.
///
/// This is trigger 3's stake in the engine fix, and it is asserted HERE because
/// nothing in this crate changed to make it work: the observer consumes alert
/// TRANSITIONS and `action_for` branches on `cleared_at_ms` alone, so a verdict
/// the engine never minted is a capture the recorder can never ask for. Before
/// this fix, the row parked in `learning` forever after the wipe and the death of
/// a degraded topic reached the gate as NOTHING — no request, no refusal, no
/// counter, nothing to find afterwards.
///
/// The vector is the measured one: a topic learns a baseline, degrades far
/// enough to confirm `rate_deviation`, and then STOPS. Its rate going absent is
/// what clears the deviation (the rate estimate serves no rate for a stopped stream), the
/// clear wipes the baseline, and the stall confirms two samples later against the
/// retained basis.
///
/// Both captures are asserted by SUBJECT rather than by count alone, because a
/// count of two is also what a gate that captured the deviation twice would
/// report — and the second capture's DETAIL is asserted to carry the baseline,
/// which is the whole of the evidence claim: the number the verdict
/// rested on travels into the bag even though the row's own `baseline_mhz` was
/// wiped before the stall confirmed.
#[test]
fn a_rate_flap_that_clears_at_death_still_reaches_the_gate_as_a_stall() {
    const TOPIC: &str = "/lowstate";
    const NOMINAL_MHZ: u64 = 100_000;
    /// Just under half the baseline — outside the octave, so it confirms.
    const DEVIATING_MHZ: u64 = NOMINAL_MHZ / 2 - 1;

    let mut h = Harness::new(TriggerPolicy::default());

    // Warm up and learn, exactly as `stall_one_topic` does.
    for _ in 0..(settle_passes() + u64::from(cerulion_core::monitor::MONITOR_LEARN_SAMPLES) + 2) {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        h.pass(&[sample(TOPIC, at, Some(streaming(at, NOMINAL_MHZ)))]);
    }
    assert_eq!(h.captures(), 0, "a healthy topic captures nothing");

    // It degrades: `rate_deviation` confirms and captures.
    for _ in 0..confirm_passes() {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        h.pass(&[sample(TOPIC, at, Some(streaming(at, DEVIATING_MHZ)))]);
    }
    assert_eq!(
        h.captures(),
        1,
        "the deviation must capture: {:?}",
        h.decisions
    );
    assert_eq!(
        h.requested[0].subject,
        format!("rate_deviation:local:{TOPIC}"),
        "…under its OWN subject"
    );
    // Closed, so the stall below is judged on its own rather than coalesced.
    h.close_capture();

    // …and then the publisher DIES.
    for _ in 0..(confirm_passes() + u64::from(cerulion_core::monitor::MONITOR_CLEAR_SAMPLES)) {
        let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
        h.pass(&[sample(TOPIC, at, Some(idle(at, 30_000)))]);
    }

    // The deviation retracts — by ABSENCE of a rate, which is what wipes the
    // baseline out from under the stall gate.
    assert_eq!(
        h.recovered
            .iter()
            .map(|(subject, _)| subject.as_str())
            .collect::<Vec<_>>(),
        vec![format!("rate_deviation:local:{TOPIC}").as_str()],
        "the deviation's clearing edge must reach the gate: {:?}",
        h.recovered
    );

    // THE PIN: the stall still reaches the gate, and captures.
    assert_eq!(
        h.captures(),
        2,
        "the death of a degraded topic must capture — before this fix the row \
         parked in `learning` and this was silence. decisions: {:?}",
        h.decisions
    );
    assert_eq!(
        h.gate.stats().requests,
        2,
        "exactly two requests: the deviation's raise and the stall's. The engine's \
         raise is a one-shot, so the continuing stall re-asks for nothing"
    );
    let stall = h
        .requested
        .last()
        .expect("the stall reached the gate as a request");
    assert_eq!(
        stall.subject,
        format!("stalled:local:{TOPIC}"),
        "…and it is the STALL, not a second deviation"
    );
    assert!(
        stall
            .detail
            .contains(&format!("baseline={NOMINAL_MHZ} mHz")),
        "the evidence travels into the bag: the verdict rested on the last frozen \
         baseline, and the capture must say so rather than omitting the number \
         because the row's own field was wiped. detail: {}",
        stall.detail
    );
    assert!(
        stall.detail.contains("liveness=idle"),
        "…beside the classification it was judged on: {}",
        stall.detail
    );
}

// ---------------------------------------------------------------------------
// The `silent` demotion, driven through the same
// observer→gate seam every other arm in this file uses.
// ---------------------------------------------------------------------------

/// A never-produced route mints NOTHING by default, and the withholding is
/// COUNTED.
///
/// BOTH postures are driven over the IDENTICAL sample stream in ONE body,
/// because either half alone reads as the other's bug: "the default withholds
/// it" is satisfied by an observer whose engine never confirms anything, and
/// "opting in captures" says nothing about what a robot does by default.
///
/// The counter is the Principle #3 half. Without it a robot whose `silent` rows
/// fire steadily and are withheld reads — on `actions()`, on `passes()`, on the
/// gate's every counter — exactly like one where nothing is happening, so an
/// operator cannot tell "the feature is off" from "nothing is wrong".
#[test]
fn a_never_produced_route_is_withheld_by_default_and_captured_when_opted_in() {
    const TOPIC: &str = "/uslam/cloud_map";
    let stream = |h: &mut Harness| {
        // Long enough to clear the engine's settle window AND confirm.
        for _ in 0..(settle_passes() + confirm_passes() + 2) {
            let at = h.now_ns + MONITOR_SAMPLE_INTERVAL_NS;
            h.pass(&[sample(TOPIC, at, Some(never_produced(at)))]);
        }
    };

    // DEFAULT: nothing reaches the gate, and the observer says so.
    let mut off = Harness::new(TriggerPolicy::default());
    stream(&mut off);
    assert!(
        off.requested.is_empty(),
        "project rule: a `silent` verdict must mint no capture by \
         default. requests: {:?}",
        off.requested
    );
    assert_eq!(off.gate.stats().requests, 0, "and nothing reached the gate");
    assert!(
        off.observer.withheld() > 0,
        "the withholding must be OBSERVABLE — a count of 0 here means either the \
         verdict never confirmed (making the assertion above vacuous) or an \
         operator has no way to learn the trigger is off"
    );

    // OPTED IN: the same stream captures, and NOTHING is withheld.
    let mut on = Harness::with_posture(
        TriggerPolicy::default(),
        TriggerPosture::default().with(TriggerSwitch::Silent, true),
    );
    stream(&mut on);
    assert_eq!(
        on.requested.len(),
        1,
        "the engine's raise is a one-shot, so a steadily-dead route asks ONCE. \
         requests: {:?}",
        on.requested
    );
    assert_eq!(on.requested[0].subject, format!("silent:local:{TOPIC}"));
    assert!(matches!(
        on.decisions.as_slice(),
        [TriggerDecision::Capture { .. }]
    ));
    assert_eq!(
        on.observer.withheld(),
        0,
        "an admitted verdict is not a withheld one — the counter must not \
         double-report"
    );
}
