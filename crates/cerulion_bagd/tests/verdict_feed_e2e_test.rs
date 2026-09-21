// SPDX-License-Identifier: AGPL-3.0-only
//! Trigger 3: the TRANSPORT-FACING half — the observer's liveness feed,
//! over REAL iceoryx2.
//!
//! # What this proves that `verdict_regime_test` cannot
//!
//! That file drives the verdict arithmetic by handing the observer a
//! `MonitorSample` vector, which is deliberately blind to the half BELOW the
//! samples: whether [`VerdictObserver::note_drain`] really feeds the substrate's
//! record book, and whether [`VerdictObserver::sample`] really reads it back.
//! Both could be inert — a `note_drain` that never marks the topic externally
//! observed writes nothing, and every arm over there would stay green while the
//! shipped watchdog observed nothing at all, forever, on every robot.
//!
//! So these arms publish through a REAL publisher, drain through a REAL
//! `DataOnlySubscriber` in exactly the shape the recorder's `drain_taps` uses,
//! and read the verdict back out of the record book.
//!
//! # And the COST claim, which is the other thing only real transport can settle
//!
//! The module claims zero new iceoryx2 ports. That is not a promise about
//! intent — the observer holds a real `TopicLivenessObserver`, which WOULD attach
//! a tap per tracked topic if `sweep()` were ever called. The producer's own live
//! subscriber count is the log-independent oracle for whether one appeared, and
//! the data-only-tap arms use exactly that oracle for the mirror-image
//! claim.
//!
//! Isolated per-test SHM roots via the `common` harness ⇒ parallel-safe, no
//! `#[serial]`. No walls: the clock is a `VirtualClock` the test advances by
//! hand, so nothing here can flake under load.

mod common;

use std::sync::Arc;

use cerulion_bagd::verdict_observer::VerdictObserver;
use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::monitor::{MONITOR_SAMPLE_INTERVAL_NS, MONITOR_SETTLE_NS};
use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::transport::liveness::DrainObservation;
use cerulion_core::transport::subscriber::DataOnlySubscriber;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::LivenessState;

const SCHEMA_HASH: u64 = 0xFEED_BEEF_1260_0003;
const MS: u64 = 1_000_000;

/// Drain one topic exactly as `Recorder::drain_taps` does, and report what the
/// pass observed.
///
/// This mirrors the production drain deliberately, including the two details the
/// substrate's dating rule turns on: the batch MAX stamp and sequence (not the
/// last frame seen), and `queue_emptied` OBSERVED from the loop exiting on a
/// short read rather than inferred from arithmetic.
fn drain_like_the_recorder(sub: &mut DataOnlySubscriber) -> DrainObservation {
    let mut frames = 0u64;
    let mut newest_stamp_ns: Option<u64> = None;
    let mut newest_sequence: Option<u32> = None;
    let queue_emptied;
    let mut scratch = Vec::new();
    loop {
        let n = sub.drain_owned(8, &mut scratch).expect("drain");
        for s in scratch.drain(..) {
            if let Some(h) = s.wire_header() {
                newest_stamp_ns =
                    Some(newest_stamp_ns.map_or(h.timestamp_ns, |v: u64| v.max(h.timestamp_ns)));
                newest_sequence =
                    Some(newest_sequence.map_or(h.sequence, |v: u32| v.max(h.sequence)));
            }
            frames += 1;
        }
        if n == 0 {
            // OBSERVED, not inferred: the loop exits on a SHORT read, so an
            // empty queue is direct evidence rather than arithmetic.
            queue_emptied = true;
            break;
        }
    }
    DrainObservation {
        frames,
        newest_stamp_ns,
        newest_sequence,
        writers_seen: (frames > 0).then(|| sub.publisher_count()),
        queue_emptied,
    }
}

/// A manager whose clock the test drives by hand.
///
/// The clock is built HERE and handed to the manager, so the record book (which
/// stamps through the manager's clock) and this test read ONE number line — the
/// module's own clock-domain rule, applied to its own test. Recovering a handle
/// from the manager instead would need a downcast the `Clock` trait does not
/// offer, and building a SECOND `VirtualClock` would silently give the two halves
/// unrelated origins and make every age assertion meaningless.
fn clocked_manager() -> (Arc<TransportManager>, Arc<VirtualClock>) {
    let clock = Arc::new(VirtualClock::new());
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "verdict_feed".into(),
            clock: clock.clone(),
            subscriber_buffer_size: 64,
            network: None,
        },
        iceoryx_test_config(),
    )
    .expect("isolated transport manager");
    (mgr, clock)
}

/// THE feed arm: a real drain makes a real topic datable.
///
/// Before any drain the observer knows nothing (`None` — UNKNOWN, never dead).
/// After the recorder's own drain shape has fed it, the substrate classifies the
/// topic `Streaming` at an exact hand-oracle age.
///
/// The intermediate step is the one the dating rule forces and the one a naive
/// implementation gets wrong: the FIRST batch after any attach is a BASELINE
/// (it could be a retained-history flush), so it is banked and never dated. A
/// topic is only `Streaming` once a LATER batch's stamps out-rank it.
#[test]
fn a_real_drain_feeds_the_record_book_and_dates_the_topic() {
    let (mgr, clock) = clocked_manager();
    let topic = common::unique_topic("verdict_feed");
    let mut pub_ = common::publisher(&mgr, &topic, 256);
    let mut sub = mgr
        .create_data_only_subscriber(&topic)
        .expect("the recorder's own tap shape");
    let mut observer = VerdictObserver::new(Arc::clone(&mgr));

    assert!(
        observer.liveness(&topic).is_none(),
        "before any drain the observer knows NOTHING about the topic — and that must \
         read as UNKNOWN, never as a verdict"
    );

    // Batch 1 — the BASELINE. Banked, never dated.
    clock.set(1_000 * MS);
    let frame = common::build_frame(SCHEMA_HASH, 0, clock.now_ns(), b"one");
    pub_.publish_raw(&frame).expect("publish");
    let obs = drain_like_the_recorder(&mut sub);
    assert_eq!(obs.frames, 1, "the drain must really have taken the frame");
    assert_eq!(
        obs.writers_seen,
        Some(1),
        "and it must report the single-writer EVIDENCE the exact rate basis needs"
    );
    assert!(
        obs.queue_emptied,
        "the recorder's drain loops to a short read, so it can OBSERVE emptiness — a \
         drainer that can never report this leaves its topic permanently undatable"
    );
    observer.note_drain(&topic, obs);

    let after_baseline = observer
        .liveness(&topic)
        .expect("the feed must have created a record");
    assert_eq!(
        after_baseline.frames_observed, 1,
        "the baseline batch is BANKED"
    );
    assert_eq!(
        after_baseline.last_frame_age_ms, None,
        "but never DATED — it could be a retained-history flush, and dating it would \
         let a dead route read as live"
    );

    // Batch 2 — advances the publisher's stamp, so it dates the topic.
    clock.set(1_500 * MS);
    let frame = common::build_frame(SCHEMA_HASH, 1, clock.now_ns(), b"two");
    pub_.publish_raw(&frame).expect("publish");
    let obs = drain_like_the_recorder(&mut sub);
    assert_eq!(obs.frames, 1);
    observer.note_drain(&topic, obs);

    let dated = observer.liveness(&topic).expect("still observed");
    assert_eq!(
        dated.last_frame_age_ms,
        Some(0),
        "a batch whose stamps EXCEED the maximum proves the publisher committed a \
         frame between the two drains, so the topic dates at the observer's now"
    );
    assert_eq!(
        dated.state(),
        LivenessState::Streaming,
        "and a dated frame inside the recency window classifies Streaming"
    );

    // Age it past the recency window WITHOUT publishing: the substrate must let
    // it fall to Idle on its own, which is what a `stalled` verdict rests on.
    clock.set(1_500 * MS + 30_000 * MS);
    let stale = observer.liveness(&topic).expect("still observed");
    assert_eq!(
        stale.state(),
        LivenessState::Idle,
        "a topic that stops publishing ages into Idle with no further drains — the \
         age is computed at READ time, which is why an idle topic costs nothing"
    );
    assert_eq!(
        stale.last_frame_age_ms,
        Some(30_000),
        "and the age is exact, on the observer's own clock"
    );
}

/// The samples the engine sees really carry that observation.
///
/// The arm above proves the record book is fed; this proves `sample` READS it —
/// the other half of the wiring, and the one a `sample` that built its samples
/// from `None` would fail while every regime arm stayed green.
#[test]
fn the_samples_the_engine_sees_carry_the_drained_observation() {
    let (mgr, clock) = clocked_manager();
    let topic = common::unique_topic("verdict_feed_read");
    let mut pub_ = common::publisher(&mgr, &topic, 256);
    let mut sub = mgr.create_data_only_subscriber(&topic).expect("tap");
    let mut observer = VerdictObserver::new(Arc::clone(&mgr));

    // Two batches, so the topic is genuinely dated (see the arm above).
    for (seq, at) in [(0u32, 1_000 * MS), (1, 1_500 * MS)] {
        clock.set(at);
        let frame = common::build_frame(SCHEMA_HASH, seq, clock.now_ns(), b"x");
        pub_.publish_raw(&frame).expect("publish");
        let obs = drain_like_the_recorder(&mut sub);
        observer.note_drain(&topic, obs);
    }

    // A sampling pass over the recorder's tap set. It mints no action (one
    // sample can confirm nothing), but it must OPEN a row — which it can only do
    // from an observation it actually read.
    let now = clock.now_ns();
    let actions = observer.sample([topic.as_str()]);
    assert!(
        actions.is_empty(),
        "one sample can confirm nothing — a verdict from a single observation would \
         mean the confirmation window is not being applied"
    );
    assert_eq!(
        observer.watched(),
        1,
        "but the pass must have opened a row for the topic"
    );
    assert_eq!(
        observer.passes(),
        1,
        "and counted itself, so the sampler's own liveness is observable"
    );

    // THE EXACT-VALUE ORACLE. Everything above is satisfied by a `sample()` that
    // built its whole vector from `None` — a row would still open and a pass would
    // still count, while the watchdog observed nothing forever. What separates the
    // two is the row's CONTENT, so from here on the assertions are on values the
    // record book produced.
    //
    // Drive the row past the engine's settle window, keeping the topic streaming,
    // so the samples are ADMITTED as evidence rather than discarded.
    let settle_passes = MONITOR_SETTLE_NS / MONITOR_SAMPLE_INTERVAL_NS + 2;
    let mut at = now;
    for seq in (2u32..).take(settle_passes as usize) {
        at += MONITOR_SAMPLE_INTERVAL_NS;
        clock.set(at);
        let frame = common::build_frame(SCHEMA_HASH, seq, clock.now_ns(), b"x");
        pub_.publish_raw(&frame).expect("publish");
        let obs = drain_like_the_recorder(&mut sub);
        observer.note_drain(&topic, obs);
        let _ = observer.sample([topic.as_str()]);
    }

    let row = observer
        .rows()
        .into_iter()
        .find(|r| r.topic == topic)
        .expect("the sampler must have opened a row for the tapped topic");
    assert_eq!(
        row.liveness_state,
        Some(LivenessState::Streaming),
        "the row must carry the verdict the RECORD BOOK produced — a sampler \
         handing the engine blind samples reports None here while every counter \
         above still climbs"
    );
    assert!(
        row.samples > 0,
        "and the engine must have ADMITTED those samples as evidence, not merely \
         been handed them: {row:?}"
    );
    assert_eq!(
        row.robot, None,
        "a recorder taps its own machine, so every row it opens is a local producer"
    );

    // The value must TRACK the record book, not be a constant that happens to
    // match. Stop publishing and age past the recency window: the SAME row must
    // now report Idle.
    at += 30_000 * MS;
    clock.set(at);
    let _ = observer.sample([topic.as_str()]);
    let row = observer
        .rows()
        .into_iter()
        .find(|r| r.topic == topic)
        .expect("the row survives");
    assert_eq!(
        row.liveness_state,
        Some(LivenessState::Idle),
        "a topic that stopped publishing must be reported as such — this is what \
         makes the Streaming assertion above a measurement rather than a constant"
    );

    // The discriminator: a topic the recorder does NOT tap is never sampled, so
    // the watched universe is exactly the recorder's own.
    at += 10_000 * MS;
    clock.set(at);
    let _ = observer.sample([topic.as_str()]);
    assert_eq!(
        observer.watched(),
        1,
        "sampling the same tap set again opens no new row"
    );
}

/// THE COST CLAIM: the observer opens ZERO iceoryx2 ports.
///
/// The oracle is the PRODUCER's own live subscriber count, which is
/// log-independent and cannot be satisfied by intent. A `sweep()` creeping into
/// this path — or a `note_drain` that forgot to mark the topic externally
/// observed, leaving the observer free to attach its own tap on a later sweep —
/// would show up here as a count that moved.
///
/// The anti-tautology half is in the same body: attaching a REAL tap raises the
/// count by exactly one, so the probe is proven to be able to see an attachment
/// at all.
#[test]
fn the_observer_opens_no_port_of_its_own() {
    let (mgr, clock) = clocked_manager();
    let topic = common::unique_topic("verdict_feed_ports");
    let mut pub_ = common::publisher(&mgr, &topic, 256);

    let baseline = mgr.topic_subscriber_count(&topic);

    let mut observer = VerdictObserver::new(Arc::clone(&mgr));
    // Feed it the way production does, over several batches, so any lazy attach
    // has every opportunity to happen.
    let mut sub = mgr
        .create_data_only_subscriber(&topic)
        .expect("the recorder's own tap");
    let with_the_recorders_tap = mgr.topic_subscriber_count(&topic);
    assert_eq!(
        with_the_recorders_tap,
        baseline + 1,
        "ANTI-TAUTOLOGY: a real tap raises the count by one, so this probe can see an \
         attachment at all"
    );

    for (seq, at) in [(0u32, 1_000 * MS), (1, 1_500 * MS), (2, 2_000 * MS)] {
        clock.set(at);
        let frame = common::build_frame(SCHEMA_HASH, seq, clock.now_ns(), b"x");
        pub_.publish_raw(&frame).expect("publish");
        let obs = drain_like_the_recorder(&mut sub);
        observer.note_drain(&topic, obs);
        let _ = observer.sample([topic.as_str()]);
    }

    assert_eq!(
        mgr.topic_subscriber_count(&topic),
        with_the_recorders_tap,
        "the observer must add NO subscriber of its own — the whole cost claim. It \
         holds a real TopicLivenessObserver, which would attach a tap per tracked \
         topic if sweep() were ever called; that it never is, is what makes the claim \
         structural rather than a promise"
    );
    assert!(
        observer
            .liveness(&topic)
            .is_some_and(|l| l.frames_observed > 0),
        "and it observed the topic anyway, which is what stops this arm passing for a \
         watchdog that simply does nothing"
    );
}

/// THE CLOCK-DOMAIN ARM: a robot that has been UP FOR HOURS still reads a real
/// age, and its stalled topic still classifies `Idle`.
///
/// # The defect this exists to make loud
///
/// `read_liveness` computes every age as `now_ns - last_frame_at_ns`, and
/// `last_frame_at_ns` is stamped by the record book from the MANAGER's clock —
/// `real_ns()`, i.e. **system uptime**. The recorder's own drive-loop origin is
/// `start.elapsed()`, **nanoseconds since the recorder process started**. Two
/// unrelated number lines.
///
/// The first version of this module let the caller supply the instant, and
/// `service_monitor_verdicts` handed over the recorder's. On a robot up six hours
/// whose recorder started a minute ago that is ~60e9 against a stamp of
/// ~21_600e9, so `saturating_sub` yields **0**: every topic reads
/// `last_frame_age_ms: Some(0)`, classifies `Streaming` forever, and `stalled`
/// can NEVER fire. The watchdog is silently inert on exactly the condition it
/// exists to catch — no error, no warning, no counter, on every long-uptime
/// robot.
///
/// # Why the other arms in this file cannot see it
///
/// They set the clock near ZERO (`1_000 * MS`), where a process-relative instant
/// and a uptime instant are indistinguishable. This one puts the manager's clock
/// SIX HOURS out — far past any plausible process-relative origin — so the two
/// domains give different answers and the oracle is an exact age rather than a
/// classification that both could produce.
///
/// The shipped API takes no instant at all, which is what makes the mistake
/// unrepresentable; restoring one and passing the recorder's would repeat it.
#[test]
fn a_robot_up_for_hours_still_ages_its_topics_against_the_right_clock() {
    const SIX_HOURS: u64 = 6 * 60 * 60 * 1_000 * MS;

    let (mgr, clock) = clocked_manager();
    let topic = common::unique_topic("verdict_clock_domain");
    let mut pub_ = common::publisher(&mgr, &topic, 256);
    let mut sub = mgr.create_data_only_subscriber(&topic).expect("tap");

    // THE ROBOT IS ALREADY UP when the recorder starts, and this ordering is the
    // whole point of the arm rather than set dressing: `start.elapsed()` is the
    // manager clock MINUS the recorder's start instant, so the two origins only
    // diverge once the recorder is born into a clock that is already far from
    // zero. Constructing the observer first (at clock 0) makes the offset nil and
    // the two domains agree by accident — measured, that ordering lets the
    // wrong clock domain go undetected.
    clock.set(SIX_HOURS);
    let mut observer = VerdictObserver::new(Arc::clone(&mgr));

    // Two batches so the topic is genuinely DATED (the first is the baseline).
    for (seq, at) in [(0u32, SIX_HOURS), (1, SIX_HOURS + 500 * MS)] {
        clock.set(at);
        let frame = common::build_frame(SCHEMA_HASH, seq, clock.now_ns(), b"x");
        pub_.publish_raw(&frame).expect("publish");
        let obs = drain_like_the_recorder(&mut sub);
        observer.note_drain(&topic, obs);
    }

    let dated = observer.liveness(&topic).expect("observed");
    assert_eq!(
        dated.last_frame_age_ms,
        Some(0),
        "a frame committed at the observer's own now is zero old, whatever the \
         absolute value of that clock"
    );
    assert_eq!(dated.state(), LivenessState::Streaming);

    // Now STOP publishing and let six hours plus thirty seconds elapse. The
    // topic must age by exactly the wall that passed.
    clock.set(SIX_HOURS + 500 * MS + 30_000 * MS);

    let stale = observer.liveness(&topic).expect("still observed");
    assert_eq!(
        stale.last_frame_age_ms,
        Some(30_000),
        "THE CLOCK-DOMAIN ORACLE. The age must be measured on the clock that \
         STAMPED the frame. Read against a process-relative origin instead, this \
         subtraction saturates to 0 and the topic reads perfectly fresh forever — \
         which is the silent-inert failure this arm exists to catch"
    );
    assert_eq!(
        stale.state(),
        LivenessState::Idle,
        "and a topic that stopped publishing six hours into a run must fall out of \
         Streaming — the transition every `stalled` verdict is built on. Under the \
         wrong clock it never does"
    );

    // The sampler reads the SAME clock, so its samples carry that verdict too.
    let actions = observer.sample([topic.as_str()]);
    assert!(
        actions.is_empty(),
        "one sample confirms nothing — the confirmation window still applies"
    );
    assert_eq!(
        observer.watched(),
        1,
        "but the pass opened a row, so the sampler really did read the record book"
    );
}

/// THE WHOLE CHAIN, over real transport: a real tap on a real registered-but-dead
/// topic produces a real `CaptureRequest` the real gate turns into a capture.
///
/// # Why this arm exists
///
/// `verdict_regime_test` drives the verdict ARITHMETIC through the module's
/// declared drivable seam, which is the repo's standard for a pure state machine
/// (`failure_regime_latch_test`, `drain_latch_test`, `output_discard_latch_test`
/// are all hand-vector files, and `vizd_e2e_test` labels the same shape "a DI
/// test double — Principle #13, not fake data"). But it hands the engine its
/// samples, so it is structurally blind to whether a verdict is reachable AT ALL
/// from an observation the transport really produced — and the other arms in THIS
/// file stop at the observation, because one sample confirms nothing.
///
/// So nothing anywhere connected the two halves. This arm does, with no hand-built
/// sample and no hand-built liveness: a real publisher registers the topic and
/// then says NOTHING (the `/uslam/cloud_map` shape — the registered-but-dead route
/// `silent` exists for), the recorder's own drain marks it observed, and virtual
/// time carries it through the substrate's `NoData` threshold and the engine's
/// settle and confirmation windows into a real request.
///
/// # Why it costs no wall time
///
/// Every window the chain crosses is measured on SAMPLE TIMESTAMPS, which come
/// from the manager's clock — a `VirtualClock` this test advances by hand. So the
/// ~11.6 s of logical time this arm spans is instant, and nothing here can flake
/// under load. That is only true because the observer reads ITS OWN clock (see
/// `a_robot_up_for_hours_...`); a caller-supplied instant would put the engine's
/// windows on a different number line from the substrate's ages.
#[test]
fn a_registered_but_dead_topic_reaches_a_real_capture_over_real_transport() {
    use cerulion_bagd::verdict_observer::VerdictAction;
    use cerulion_core::flashback::switch::TriggerPosture;
    use cerulion_core::flashback::trigger::{FlashbackTriggerGate, TriggerDecision, TriggerPolicy};
    use cerulion_core::transport::liveness::LIVENESS_NO_DATA_MIN_MS;

    let (mgr, clock) = clocked_manager();
    let topic = common::unique_topic("verdict_silent");

    // A REAL publisher registers the topic — and never publishes. Held for the
    // whole arm, because dropping it would release the service and change what
    // the tap is observing.
    let _pub = common::publisher(&mgr, &topic, 256);
    let mut sub = mgr.create_data_only_subscriber(&topic).expect("tap");
    // Decision 110-B demoted `silent` to available-off, so this
    // arm opts it back IN — its subject is the CHAIN (does a registered-but-dead
    // topic reach a real capture at all?), which is a different question from
    // whether a robot captures those by default. The default-posture half is
    // asserted at the END of this body, over the SAME stimulus, so neither
    // question can be answered by accident of the other.
    let opted_in = TriggerPosture::default().with(
        cerulion_core::flashback::switch::TriggerSwitch::Silent,
        true,
    );
    let mut observer = VerdictObserver::with_posture(Arc::clone(&mgr), opted_in);
    let mut gate = FlashbackTriggerGate::new(TriggerPolicy::default());

    // The recorder's own drain, on a topic with nothing to drain: zero frames,
    // which is what MARKS the topic observed and starts the observation interval.
    let obs = drain_like_the_recorder(&mut sub);
    assert_eq!(
        obs.frames, 0,
        "a dead route yields nothing, by construction"
    );
    observer.note_drain(&topic, obs);
    assert!(
        observer.liveness(&topic).is_some(),
        "but the drain must still have OPENED an observation — that is what makes \
         a registered-but-dead topic distinguishable from one nobody is watching"
    );

    // Carry it through the substrate's NoData threshold and the engine's windows.
    // Ceiling is DERIVED and generous; the assertion is on WHAT fired, and on the
    // logical time it took, not on a wall.
    let no_data_ns = LIVENESS_NO_DATA_MIN_MS * MS;
    let ceiling = (no_data_ns + MONITOR_SETTLE_NS) / MONITOR_SAMPLE_INTERVAL_NS + 64;

    let mut at = 0u64;
    let mut captured: Option<(u64, String)> = None;
    for _ in 0..ceiling {
        at += MONITOR_SAMPLE_INTERVAL_NS;
        clock.set(at);
        // Keep draining, exactly as the recorder does every pass. Still nothing.
        let obs = drain_like_the_recorder(&mut sub);
        observer.note_drain(&topic, obs);
        for action in observer.sample([topic.as_str()]) {
            match action {
                VerdictAction::Capture(request) => {
                    let decision = gate.decide(&request, at);
                    assert!(
                        matches!(decision, TriggerDecision::Capture { .. }),
                        "the first confirmed verdict of a run must OPEN a capture, \
                         got {decision:?}"
                    );
                    assert!(captured.is_none(), "and it must fire exactly once");
                    captured = Some((at, request.subject.clone()));
                }
                VerdictAction::Recover { subject } => {
                    panic!("a topic that never streamed cannot RECOVER: {subject}")
                }
            }
        }
    }

    let (fired_at, subject) = captured.expect(
        "a registered-but-dead topic must reach a real capture — this is the whole \
         chain, and if it does not close here the watchdog is inert no matter how \
         green the pure arms are",
    );
    assert_eq!(
        subject,
        format!("silent:local:{topic}"),
        "and the condition must be SILENT (never streamed), keyed on this topic"
    );
    assert!(
        fired_at >= no_data_ns,
        "it must not fire before the substrate itself will say NoData ({no_data_ns} \
         ns) — firing earlier would mean the engine is reading a threshold it does \
         not own. fired at {fired_at}"
    );
    assert_eq!(
        gate.stats().requests,
        1,
        "exactly one request reached the gate: the engine's raise is a one-shot, so \
         a steadily-dead topic asks once and then says nothing"
    );

    // ---------------------------------------------------------------------
    // Decision 110-B, over the same stimulus: by default this chain mints
    // nothing at all.
    //
    // Driven here rather than in its own arm because the two halves are only
    // meaningful together — "the default withholds it" alone is satisfied by an
    // observer that is broken, and the chain above is what proves the stimulus
    // really does confirm a `silent` verdict. A SECOND observer over the SAME
    // live topic and the SAME advanced clock: the substrate's own record book is
    // per-observer, so this one re-opens its interval at the current instant and
    // is carried through the thresholds a second time.
    // ---------------------------------------------------------------------
    let mut default_observer = VerdictObserver::new(Arc::clone(&mgr));
    let default_gate = FlashbackTriggerGate::new(TriggerPolicy::default());
    let mut default_at = at;
    for _ in 0..ceiling {
        default_at += MONITOR_SAMPLE_INTERVAL_NS;
        clock.set(default_at);
        let obs = drain_like_the_recorder(&mut sub);
        default_observer.note_drain(&topic, obs);
        let actions = default_observer.sample([topic.as_str()]);
        assert!(
            actions.is_empty(),
            "project rule: a `silent` verdict must mint NO action under \
             the default posture, got {actions:?}"
        );
    }
    assert_eq!(
        default_gate.stats().requests,
        0,
        "nothing may reach the gate under the default posture"
    );
    // …and the withholding is COUNTED, never silent: a robot whose `silent` rows
    // would have fired reads, on every other observable, exactly like one where
    // nothing happened (Principle #3).
    assert!(
        default_observer.withheld() > 0,
        "the demotion must be OBSERVABLE — a withheld count of 0 here would mean \
         either the verdict never confirmed (making the assertion above vacuous) \
         or the withholding is invisible to an operator"
    );
}
