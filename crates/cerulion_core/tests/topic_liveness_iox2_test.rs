// SPDX-License-Identifier: AGPL-3.0-only
//! The DATA-FLOW liveness observer over REAL iceoryx2.
//!
//! # The oracle (a publisher pair observed live, reproduced in one process)
//!
//! A topic whose publisher EXISTS but has never published must read NOT LIVE,
//! while a topic that has published must read LIVE — and every test here asserts
//! that against the OLD signal in the same body:
//! [`TransportManager::topic_publisher_count_checked`] reports `Some(1)` for BOTH,
//! which is exactly the blindness the liveness observer was built to fix (measured on a Go2:
//! all 75 `ros2 attach` topics reported `producer_count: 1`, so the dead route
//! `/uslam/cloud_map` was indistinguishable from the streaming
//! `/utlidar/cloud_deskewed`). Asserting both signals side by side makes each test
//! a DISCRIMINATION proof rather than a claim about one number in isolation.
//!
//! Ages are exact hand oracles: the manager runs a
//! [`VirtualClock`](cerulion_core::clock::VirtualClock) that the test sets, and the
//! observer reads that same clock, so `last_frame_age_ms` is arithmetic on numbers
//! this file chose — never a value read back from the code under test.
//!
//! # The dating rule these tests exercise
//!
//! A topic is DATED by stamp ADVANCEMENT: a drained batch whose newest publisher
//! stamp exceeds every stamp seen before it on that topic proves a publish
//! happened between the two drains. Two consequences show up in almost every test
//! below, and they are the design, not an accident:
//!
//! * the FIRST batch after any attach establishes the baseline and does NOT date
//!   (it could equally be the publisher's flushed retained history), so a
//!   streaming topic is dated from its SECOND observed batch;
//! * a topic that produced frames but has never been datable classifies
//!   [`LivenessState::Idle`] — never `no_data`, which is reserved for
//!   `frames_observed == 0`.
//!
//! The cross-clock section at the end is why the rule compares publisher stamps
//! to publisher stamps and never to the observer's clock.
//!
//! Parallel-safe: every test mints its own isolated iceoryx2 config (per-test SHM
//! root) and unique topic names. No `#[serial]`, no `--test-threads=1`.
//!
//! ```bash
//! cargo test -p cerulion_core --test topic_liveness_iox2_test
//! ```

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::transport::liveness::{
    DrainObservation, LivenessState, TopicLiveness, TopicLivenessObserver, TopicRateEstimate,
    DEFAULT_LIVENESS_TAP_BUDGET, LIVENESS_NO_DATA_MIN_MS, LIVENESS_STREAMING_RECENCY_MS,
    LIVENESS_SWEEP_INTERVAL_NS, LIVENESS_TAP_BUFFER_SIZE, RATE_ESTIMATE_MIN_WINDOW_NS,
    RATE_FLOOR_BASIS_CEILING_MHZ, REGRESSION_RESET_MIN_GAP_NS, SUSTAINED_REGRESSION_MIN_SPAN_NS,
};
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};

const MS: u64 = 1_000_000;
/// A nonzero clock origin, so "time 0" is never confused with "unset".
const T0: u64 = 1_000 * MS;

static SEQ: AtomicU64 = AtomicU64::new(0);

fn unique(tag: &str) -> String {
    format!(
        "{tag}_{}_{}",
        std::process::id(),
        SEQ.fetch_add(1, Ordering::Relaxed)
    )
}

/// One manager on its OWN SHM root, driven by a clock the test sets. Publisher and
/// observer share it, so the observation clock is the test's clock.
fn manager(tag: &str) -> (Arc<TransportManager>, Arc<VirtualClock>) {
    let clock = Arc::new(VirtualClock::new());
    clock.set(T0);
    let clock_dyn: Arc<dyn Clock> = clock.clone();
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: unique(tag),
            clock: clock_dyn,
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated transport manager");
    (mgr, clock)
}

/// A minimal valid wire frame, so the tap slices exactly what was published.
fn frame(seq: u32, ts: u64) -> Vec<u8> {
    let total = WireHeader::SIZE + 8;
    let mut h = WireHeader::new(0x889, seq, ts);
    h.total_size = total as u32;
    let mut f = vec![0u8; total];
    h.write_to_buf(&mut f[..WireHeader::SIZE]);
    f
}

/// Run one sweep at virtual time `now`, defeating the self-throttle so the test
/// controls exactly when observation happens.
fn sweep_at(obs: &mut TopicLivenessObserver, clock: &VirtualClock, now: u64) -> u64 {
    clock.set(now);
    obs.force_next_sweep_for_test();
    obs.sweep()
}

/// An external drainer's report of a drain that emptied the queue — what the
/// gateway's egress tap hands the observer on a short read.
fn drained(frames: u64, stamp: Option<u64>) -> DrainObservation {
    DrainObservation {
        frames,
        newest_stamp_ns: stamp,
        // These arms pin the DATING rule, which the rate estimate does
        // not participate in; carrying no sequence keeps them independent of it.
        newest_sequence: None,
        // These arms pin DATING; the rate's writer evidence is
        // irrelevant to them.
        writers_seen: None,
        queue_emptied: true,
    }
}

// ===========================================================================
// THE oracle: publisher-existence vs data-flow.
// ===========================================================================

/// THE discrimination, in one body: two topics with IDENTICAL publisher
/// counts, one of which has published and one of which never has.
///
/// The OLD signal (`topic_publisher_count_checked`) reports `Some(1)` for BOTH
/// — the exact blindness this arm exists to catch — while the data-flow observation
/// separates them. Both assertions are hand oracles.
#[test]
fn a_publisher_that_never_published_reads_not_live_while_one_that_did_reads_live() {
    let (mgr, clock) = manager("oracle");
    let live_topic = format!("/{}/live", unique("t"));
    let dead_topic = format!("/{}/dead", unique("t"));

    // Both publishers EXIST (the `ros2 attach` bridge shape: a port per route).
    let mut live_pub = mgr
        .create_publisher_simple(&live_topic, MaxSliceLen::const_new(256))
        .expect("live publisher");
    let _dead_pub = mgr
        .create_publisher_simple(&dead_topic, MaxSliceLen::const_new(256))
        .expect("dead publisher — created, never published");

    // ANTI-TAUTOLOGY / the defect itself: the OLD signal cannot tell them apart.
    assert_eq!(
        mgr.topic_publisher_count_checked(&live_topic),
        Some(1),
        "publisher presence on the live topic"
    );
    assert_eq!(
        mgr.topic_publisher_count_checked(&dead_topic),
        Some(1),
        "THE DEFECT: a topic that has NEVER published still reports a live \
         publisher — so `producer_count` cannot be the liveness affordance"
    );

    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    obs.track(&live_topic);
    obs.track(&dead_topic);

    // Sweep 1 attaches both taps. Nothing has been published SINCE the attach, so
    // both are still "watched, nothing seen" — and only 0 ms of observation, which
    // is correctly UNKNOWN, not `NoData`.
    assert_eq!(sweep_at(&mut obs, &clock, T0), 0);
    assert_eq!(obs.active_tap_count(), 2);
    for t in [&live_topic, &dead_topic] {
        let l = obs.liveness(t).expect("observation started");
        assert_eq!(l.last_frame_age_ms, None);
        assert_eq!(l.observed_for_ms, 0);
        assert_eq!(
            l.state(),
            LivenessState::Unknown,
            "{t}: never libel a topic as dead in its first instant"
        );
    }

    // The live topic publishes at T0 + 1s.
    clock.set(T0 + 1_000 * MS);
    live_pub
        .publish_raw(&frame(0, T0 + 1_000 * MS))
        .expect("publish");

    // Sweep 2 at T0 + 1.2s: the live topic's frame is drained, the dead one's is
    // not. This is the FIRST batch since the tap attached, so it establishes the
    // advancement baseline — it is banked, not dated (the robot cannot tell it
    // from a flushed retained history). The two rows ALREADY differ: one has
    // frames, the other has none.
    let t2 = T0 + 1_200 * MS;
    assert_eq!(
        sweep_at(&mut obs, &clock, t2),
        1,
        "exactly one frame drained"
    );
    assert_eq!(
        obs.liveness(&live_topic),
        Some(TopicLiveness {
            last_frame_age_ms: None,
            observed_for_ms: 1_200,
            frames_observed: 1,
            rate_estimate: None,
        }),
        "the first observed batch is the baseline: banked, undated"
    );
    assert_eq!(
        obs.liveness(&live_topic).expect("observed").state(),
        LivenessState::Idle,
        "and a topic that PRODUCED a frame is Idle (freshness unknown) — never \
         the dimmed dead row, at any observation length"
    );
    assert_eq!(
        obs.liveness(&dead_topic),
        Some(TopicLiveness {
            last_frame_age_ms: None,
            observed_for_ms: 1_200,
            frames_observed: 0,
            rate_estimate: None,
        })
    );

    // Sweep 3 at T0 + 1.4s: a second frame ADVANCES past the baseline stamp, so
    // this is the batch that dates the topic.
    clock.set(T0 + 1_300 * MS);
    live_pub
        .publish_raw(&frame(1, T0 + 1_300 * MS))
        .expect("publish");
    let t3 = T0 + 1_400 * MS;
    assert_eq!(sweep_at(&mut obs, &clock, t3), 1);
    assert_eq!(
        obs.liveness(&live_topic),
        Some(TopicLiveness {
            last_frame_age_ms: Some(0),
            observed_for_ms: 1_400,
            frames_observed: 2,
            rate_estimate: None,
        }),
        "the live topic: the advancing batch drained at t3, so age 0 (hand oracle)"
    );

    // Advance to exactly the recency edge WITHOUT publishing. The live topic is
    // still Streaming (inclusive), and — the C3 ordering, behaviourally — the
    // silent one is NOT yet dead: it has been watched 6.4 s, past the recency
    // window but short of the no-data threshold, so it stays UNKNOWN.
    let t4 = t3 + LIVENESS_STREAMING_RECENCY_MS * MS;
    assert_eq!(sweep_at(&mut obs, &clock, t4), 0);
    let live = obs.liveness(&live_topic).expect("observed");
    let dead = obs.liveness(&dead_topic).expect("observed");
    assert_eq!(
        live.last_frame_age_ms,
        Some(LIVENESS_STREAMING_RECENCY_MS),
        "the live topic's age grew by exactly the elapsed virtual time"
    );
    assert_eq!(
        live.state(),
        LivenessState::Streaming,
        "exactly AT the recency window, which is inclusive"
    );
    assert!(
        dead.observed_for_ms > LIVENESS_STREAMING_RECENCY_MS
            && dead.observed_for_ms < LIVENESS_NO_DATA_MIN_MS,
        "precondition: watched longer than the recency window but short of the \
         no-data threshold ({} ms)",
        dead.observed_for_ms
    );
    assert_eq!(
        dead.state(),
        LivenessState::Unknown,
        "C3: a topic slower than `Streaming` must NOT be called dead merely for \
         missing the recency window — an identical gap AFTER its first frame would \
         only be `Idle`"
    );

    // Now past the no-data threshold: the silent topic makes the confident claim,
    // and the live one — quiet for just as long — is Idle, never NoData.
    let t5 = t3 + (LIVENESS_NO_DATA_MIN_MS + 1_000) * MS;
    assert_eq!(sweep_at(&mut obs, &clock, t5), 0);
    let live = obs.liveness(&live_topic).expect("observed");
    let dead = obs.liveness(&dead_topic).expect("observed");
    assert_eq!(
        live.last_frame_age_ms,
        Some(LIVENESS_NO_DATA_MIN_MS + 1_000)
    );
    assert_eq!(
        live.state(),
        LivenessState::Idle,
        "a topic that HAS published is Idle when it goes quiet, never NoData"
    );
    assert!(dead.observed_for_ms >= LIVENESS_NO_DATA_MIN_MS);
    assert_eq!(dead.last_frame_age_ms, None);
    assert_eq!(
        dead.state(),
        LivenessState::NoData,
        "THE HEADLINE: watched past the no-data threshold with no frame ⇒ the \
         registered-but-DEAD route the sidebar dims — while `producer_count` said Some(1)"
    );
    assert_ne!(
        live.state(),
        dead.state(),
        "and the two rows classify DIFFERENTLY even though both are silent NOW — \
         the difference is that one of them ever published"
    );

    // And the OLD signal STILL cannot tell them apart, at the end as at the start.
    assert_eq!(mgr.topic_publisher_count_checked(&live_topic), Some(1));
    assert_eq!(mgr.topic_publisher_count_checked(&dead_topic), Some(1));
}

/// A topic that streamed and then went quiet ages into `Idle` — NEVER `NoData`
/// (which would claim it never had data at all) and never stuck at `Streaming`.
#[test]
fn a_topic_that_stops_publishing_ages_into_idle_not_no_data() {
    let (mgr, clock) = manager("idle");
    let topic = format!("/{}/idle", unique("t"));
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    obs.track(&topic);
    sweep_at(&mut obs, &clock, T0);

    // Two publishes, each drained on its own sweep: the first is the baseline,
    // the SECOND advances past it and dates the topic. The timeline below is
    // anchored to that second one.
    clock.set(T0 + 10 * MS);
    publisher.publish_raw(&frame(0, T0 + 10 * MS)).expect("pub");
    assert_eq!(sweep_at(&mut obs, &clock, T0 + 11 * MS), 1);
    clock.set(T0 + 15 * MS);
    publisher.publish_raw(&frame(1, T0 + 15 * MS)).expect("pub");
    let t1 = T0 + 20 * MS;
    assert_eq!(sweep_at(&mut obs, &clock, t1), 1);
    assert_eq!(
        obs.liveness(&topic).expect("observed").state(),
        LivenessState::Streaming
    );

    // Exactly AT the recency boundary it is still streaming (inclusive) …
    let at_edge = t1 + LIVENESS_STREAMING_RECENCY_MS * MS;
    assert_eq!(sweep_at(&mut obs, &clock, at_edge), 0);
    let l = obs.liveness(&topic).expect("observed");
    assert_eq!(l.last_frame_age_ms, Some(LIVENESS_STREAMING_RECENCY_MS));
    assert_eq!(l.state(), LivenessState::Streaming);

    // … and one millisecond past it, Idle.
    let past_edge = at_edge + MS;
    assert_eq!(sweep_at(&mut obs, &clock, past_edge), 0);
    let l = obs.liveness(&topic).expect("observed");
    assert_eq!(l.last_frame_age_ms, Some(LIVENESS_STREAMING_RECENCY_MS + 1));
    assert_eq!(
        l.state(),
        LivenessState::Idle,
        "a topic that HAS streamed must never regress to `NoData`"
    );
    assert_eq!(l.frames_observed, 2);

    // … and it stays Idle indefinitely, well past the no-data threshold — the
    // deliberate asymmetry: `NoData` asserts "nothing EVER published", which is
    // now false for this topic however long it has been quiet.
    let long_after = past_edge + 100 * LIVENESS_NO_DATA_MIN_MS * MS;
    assert_eq!(sweep_at(&mut obs, &clock, long_after), 0);
    assert_eq!(
        obs.liveness(&topic).expect("observed").state(),
        LivenessState::Idle,
        "Idle is unbounded — it never decays into the dead-route claim"
    );
}

/// Frames published while nothing is watching are invisible — the direct
/// consequence of the measured iceoryx2 fact that a tap sees only what is
/// published AFTER it attaches. This test exists so that behaviour is a PINNED
/// contract rather than a surprise: it is also why the observer must be
/// long-lived, and why the naive "peek at catalog-serve time" design cannot work.
#[test]
fn frames_published_before_the_tap_attached_are_not_observed() {
    let (mgr, clock) = manager("pre");
    let topic = format!("/{}/pre", unique("t"));
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");

    // Publish BEFORE any observation exists.
    for i in 0..5u32 {
        publisher.publish_raw(&frame(i, T0)).expect("pub");
    }

    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    obs.track(&topic);
    assert_eq!(
        sweep_at(&mut obs, &clock, T0 + MS),
        0,
        "a freshly attached tap drains NOTHING already published — the measured \
         iceoryx2 behaviour the whole design follows from"
    );
    assert_eq!(
        obs.liveness(&topic).expect("observed").frames_observed,
        0,
        "and the observer reports zero rather than inventing history"
    );

    // A frame published AFTER the attach IS observed (the anti-tautology control:
    // the apparatus really does move).
    clock.set(T0 + 2 * MS);
    publisher.publish_raw(&frame(5, T0 + 2 * MS)).expect("pub");
    assert_eq!(sweep_at(&mut obs, &clock, T0 + 3 * MS), 1);
    assert_eq!(obs.liveness(&topic).expect("observed").frames_observed, 1);
}

/// O1, MEASURED: a publisher's RETAINED HISTORY really does reach a
/// freshly-attached data-only tap — and the observer refuses to call it live.
///
/// # What this measures (not assumes)
///
/// The tap itself never establishes its connection: it opens no event service,
/// so it sends no `SubscriberConnected` and the publisher's `pump_history` has
/// nothing to act on (ARM A below, drains 0). But the publisher's own next
/// `send()` calls `update_connections()` internally, which establishes every
/// pending connection AND pushes the retained history into it (ARM B, drains
/// history + 1). Measured directly here, against a hand oracle, so the module
/// docs' claim is a PIN rather than a belief.
///
/// # Why it matters
///
/// Left undefended this is a FALSE-LIVE on precisely the row the feature exists
/// to dim: a dead-but-history-carrying route flushes its stale backlog into our
/// tap the moment anything nudges the publisher, and a naive observer would stamp
/// "a frame just arrived" and render it Streaming, then Idle forever. So the
/// FIRST batch after an attach establishes the advancement baseline and banks its
/// frames without dating them (arm C).
///
/// # Mutation kill
///
/// Delete the baseline (date every batch whose stamp is readable) and arm C
/// fails: the retained frames date the topic and it classifies `Streaming`
/// despite nothing having been published since observation began.
///
/// # Scope
///
/// The DEAD route is the hazard, so the flush here is triggered WITHOUT the
/// publisher sending anything new — a listener-full subscriber connects and the
/// publisher's `pump_history()` pushes the backlog out (the module docs' third
/// measured row). The complementary shape — a flush arriving together with a NEW
/// frame — is
/// `a_flush_carrying_a_new_frame_is_still_the_baseline_and_the_next_publish_dates_it`.
#[test]
fn retained_history_reaches_a_fresh_tap_and_is_not_reported_as_a_live_frame() {
    let (mgr, clock) = manager("hist");
    let topic = format!("/{}/hist", unique("t"));
    const HISTORY: usize = 4;
    let mut publisher = mgr
        .create_publisher(&topic, MaxSliceLen::const_new(256), HISTORY)
        .expect("publisher with retained history");

    // A backlog laid down LONG before anything watched (the dead-route shape:
    // it published in the past and has been silent since).
    for i in 0..HISTORY as u32 {
        publisher.publish_raw(&frame(i, T0)).expect("pub");
    }

    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    obs.track(&topic);
    let attach_at = T0 + 1_000 * MS;
    assert_eq!(
        sweep_at(&mut obs, &clock, attach_at),
        0,
        "ARM A: a freshly attached tap drains NOTHING already published — the \
         publisher has not run update_connections, and pump_history cannot help \
         because the tap sends no SubscriberConnected"
    );

    // ARM B: a listener-full subscriber connects, so the publisher's own
    // `pump_history()` runs `update_connections()` — which establishes OUR
    // pending connection too and flushes the retained history into it. The
    // publisher never sends again, so every frame that reaches our tap predates
    // the attach: the dead-but-history-carrying route exactly.
    let _late_joiner = mgr
        .create_subscriber(&topic)
        .expect("a listener-full subscriber connects");
    let nudge_at = T0 + 2_000 * MS;
    clock.set(nudge_at);
    publisher.pump_history();
    let drained = sweep_at(&mut obs, &clock, nudge_at + 10 * MS);
    assert!(
        drained > 0,
        "ARM B (MEASURED): the retained history really does arrive at a tap that \
         published NOTHING since it attached — {drained} frames. If this ever \
         becomes 0, iceoryx2's behaviour changed and the defence below is merely \
         redundant, not wrong"
    );

    // ARM C: THE DEFENCE. This is the first batch since the attach, so it is the
    // advancement baseline: banked, never dated.
    let l = obs.liveness(&topic).expect("observing");
    assert_eq!(
        l.last_frame_age_ms, None,
        "a flushed backlog must NOT present as a fresh arrival"
    );
    assert_eq!(
        l.frames_observed, drained,
        "the frames are banked — they really were seen"
    );
    assert_eq!(
        l.state(),
        LivenessState::Idle,
        "produced (the backlog really arrived) but undatable ⇒ Idle — never a \
         fabricated live"
    );

    // And however long it stays silent from here it is STILL Idle: the backlog
    // bought it no freshness, and it must not be dimmed as a route that never
    // produced anything, because it demonstrably did.
    let settled = nudge_at + (LIVENESS_NO_DATA_MIN_MS + 1_000) * MS;
    sweep_at(&mut obs, &clock, settled);
    let settled_l = obs.liveness(&topic).expect("observing");
    assert_eq!(
        settled_l.state(),
        LivenessState::Idle,
        "THE POINT: the stale backlog bought the dead route no FRESHNESS — and it \
         is not dimmed either, because `no_data` would claim it never published"
    );
    assert_ne!(
        settled_l.state(),
        LivenessState::Streaming,
        "the false-live this whole defence exists to prevent"
    );
    assert!(
        settled_l.observed_for_ms >= LIVENESS_NO_DATA_MIN_MS,
        "precondition: it really is past the settle threshold ({settled_l:?})"
    );

    // ANTI-TAUTOLOGY: genuine traffic after that DOES date the topic — the
    // baseline is consumed once per attach, not a permanent deafness.
    let live_at = settled + 10 * MS;
    clock.set(live_at);
    publisher.publish_raw(&frame(100, live_at)).expect("pub");
    assert!(sweep_at(&mut obs, &clock, live_at + MS) >= 1);
    assert_eq!(
        obs.liveness(&topic).expect("observing").state(),
        LivenessState::Streaming,
        "a frame stamped past everything the backlog carried is a real publish"
    );
}

/// The COMPLEMENT of the arm above: the measured "5 (4 history + 1 new)" row of
/// the module docs' table — a flush arriving together with a frame the publisher
/// produced after our tap attached.
///
/// The robot cannot tell the two apart (it holds five frames and one clock it
/// does not own), so this batch is the BASELINE too: banked, undated, `Idle`. The
/// topic's NEXT publish advances past that baseline and dates it, which is the
/// bounded cost of the design — one burst per attach, and never a dimmed row in
/// the meantime.
///
/// Mutation kill: date a batch whose stamp merely postdates the attach INSTANT
/// (the earlier rule this replaced) and the `Idle`/`None` assertions below fail —
/// which is exactly what a worker `VirtualClock` publisher and a `RealClock`
/// observer make meaningless in production.
#[test]
fn a_flush_carrying_a_new_frame_is_still_the_baseline_and_the_next_publish_dates_it() {
    let (mgr, clock) = manager("mixed");
    let topic = format!("/{}/mixed", unique("t"));
    const HISTORY: usize = 4;
    let mut publisher = mgr
        .create_publisher(&topic, MaxSliceLen::const_new(256), HISTORY)
        .expect("publisher with retained history");
    for i in 0..HISTORY as u32 {
        publisher.publish_raw(&frame(i, T0)).expect("pub");
    }

    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    obs.track(&topic);
    let attach_at = T0 + 1_000 * MS;
    assert_eq!(sweep_at(&mut obs, &clock, attach_at), 0, "nothing yet");

    // The publisher's own next send establishes the connection, flushing the
    // backlog AND delivering the new frame in one batch.
    let live_at = T0 + 2_000 * MS;
    clock.set(live_at);
    publisher.publish_raw(&frame(99, live_at)).expect("pub");
    let drained = sweep_at(&mut obs, &clock, live_at + 10 * MS);
    assert!(drained >= 1, "the batch arrived: {drained} frames");

    let l = obs.liveness(&topic).expect("observing");
    assert_eq!(
        l.last_frame_age_ms, None,
        "the mixed batch is indistinguishable from a pure flush, so it is the \
         baseline: banked, undated"
    );
    assert_eq!(
        l.state(),
        LivenessState::Idle,
        "and a PRODUCED topic is Idle — it escapes `no_data` on this very batch, \
         which is what a topic publishing once per attach-window needs"
    );
    assert_ne!(l.state(), LivenessState::NoData);

    // The NEXT publish advances past the baseline and dates it.
    let next_at = live_at + 100 * MS;
    clock.set(next_at);
    publisher.publish_raw(&frame(100, next_at)).expect("pub");
    assert_eq!(sweep_at(&mut obs, &clock, next_at + 10 * MS), 1);
    let l = obs.liveness(&topic).expect("observing");
    assert_eq!(l.last_frame_age_ms, Some(0), "dated by the advancing batch");
    assert_eq!(l.state(), LivenessState::Streaming);
}

/// The `/tf_static` shape, end to end: a latched topic that publishes EXACTLY
/// ONCE, ever, after the tap attached.
///
/// One frame can never ADVANCE (there is nothing before it to advance past), so
/// this topic is never dated — and that is correct: the robot holds one frame and
/// one publisher clock it does not own, so it cannot say the frame is fresh. What
/// it CAN say is that the topic produced data, which is `frames_observed: 1` and
/// therefore `Idle` — undimmed, un-dotted, forever.
///
/// The failure this forbids is the earlier `no_data` reading: the
/// dead-route rendering on a row that demonstrably published, which is what a
/// `frames_observed > 0` topic used to get whenever nothing datable arrived.
#[test]
fn a_latched_topic_that_publishes_once_is_idle_forever_never_no_data() {
    let (mgr, clock) = manager("latched");
    let topic = format!("/{}/tf_static", unique("t"));
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    obs.track(&topic);
    sweep_at(&mut obs, &clock, T0);

    // The ONE publish this topic will ever make.
    let once_at = T0 + 2_000 * MS;
    clock.set(once_at);
    publisher.publish_raw(&frame(0, once_at)).expect("pub");
    assert_eq!(sweep_at(&mut obs, &clock, once_at + 100 * MS), 1);
    let l = obs.liveness(&topic).expect("observed");
    assert_eq!(
        l.last_frame_age_ms, None,
        "one frame cannot advance past anything, so it is the baseline — the \
         robot does not claim a freshness it cannot prove"
    );
    assert_eq!(l.frames_observed, 1);
    assert_eq!(
        l.state(),
        LivenessState::Idle,
        "but it PRODUCED, so it is Idle immediately — never the dimmed row, and \
         never waiting out the settle threshold to say so"
    );

    // Nothing ever again — far past the no-data threshold, and past it again.
    for mult in [1u64, 10, 100] {
        let later = once_at + 100 * MS + mult * (LIVENESS_NO_DATA_MIN_MS + 1_000) * MS;
        assert_eq!(sweep_at(&mut obs, &clock, later), 0);
        let l = obs.liveness(&topic).expect("observed");
        assert_eq!(
            l.state(),
            LivenessState::Idle,
            "THE PIN ({mult}x past the threshold): a latched one-shot topic is \
             Idle — it published, so `no_data` would be a false claim: {l:?}"
        );
        assert_ne!(l.state(), LivenessState::NoData);
    }
}

/// A topic slower than the recency window must NEVER read `no_data`, at the
/// first attach or at any re-attach.
///
/// Its inter-frame gap exceeds `LIVENESS_STREAMING_RECENCY_MS` and even
/// `LIVENESS_NO_DATA_MIN_MS`, so any rule that needed a SECOND observed frame
/// before the row could escape the dimmed state would libel it for a full period
/// after every attach — and every hand-off re-attaches. The escape here does not
/// depend on dating at all: the FIRST observed frame banks into
/// `frames_observed`, and a produced topic is `Idle` by construction. Dating then
/// arrives on the second frame and upgrades it to `Streaming`.
#[test]
fn a_topic_slower_than_the_recency_window_never_reads_no_data_after_an_attach() {
    let (mgr, clock) = manager("slow");
    let topic = format!("/{}/slow", unique("t"));
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    obs.track(&topic);
    sweep_at(&mut obs, &clock, T0);

    // A period comfortably past the recency window (and past the no-data
    // threshold, which is what would make a double-frame wait fatal).
    const PERIOD_MS: u64 = LIVENESS_NO_DATA_MIN_MS + 2_000;
    const _: () = assert!(
        PERIOD_MS > LIVENESS_STREAMING_RECENCY_MS,
        "precondition: the period must exceed the recency window"
    );

    let mut at = T0;
    for i in 0..3u32 {
        at += PERIOD_MS * MS;
        clock.set(at);
        publisher.publish_raw(&frame(i, at)).expect("pub");
        assert_eq!(sweep_at(&mut obs, &clock, at + 10 * MS), 1);
        let l = obs.liveness(&topic).expect("observed");
        let expected = if i == 0 {
            // The baseline batch: banked, undated — but PRODUCED, so already out
            // of the dimmed state before the settle threshold is even reached.
            LivenessState::Idle
        } else {
            LivenessState::Streaming
        };
        assert_eq!(
            l.state(),
            expected,
            "publish {i}: the first observed frame is the baseline, each one after \
             it advances and dates the topic: {l:?}"
        );
        assert_ne!(
            l.state(),
            LivenessState::NoData,
            "publish {i}: THE PIN — a slow topic is never the dimmed row"
        );
        // Just before the next publish it is stale — but Idle, never the dead
        // route, even though it has been frameless for longer than the threshold.
        let stale_at = at + (PERIOD_MS - 100) * MS;
        assert_eq!(sweep_at(&mut obs, &clock, stale_at), 0);
        assert_eq!(
            obs.liveness(&topic).expect("observed").state(),
            LivenessState::Idle,
            "publish {i}: a slow topic between frames is Idle, never NoData"
        );
    }

    // And a hand-off / re-attach — which re-opens the baseline — does not
    // restart the libel either: the topic keeps its banked frames and its last
    // age, so it stays Idle throughout rather than dropping back to `no_data`.
    obs.set_externally_observed(&topic, true);
    obs.set_externally_observed(&topic, false);
    let reattach_at = at + PERIOD_MS * MS;
    sweep_at(&mut obs, &clock, reattach_at);
    assert_eq!(obs.active_tap_count(), 1, "the observer re-attached");
    let pub_at = reattach_at + 10 * MS;
    clock.set(pub_at);
    publisher.publish_raw(&frame(99, pub_at)).expect("pub");
    assert_eq!(sweep_at(&mut obs, &clock, pub_at + 10 * MS), 1);
    let l = obs.liveness(&topic).expect("observed");
    assert_eq!(
        l.state(),
        LivenessState::Idle,
        "the post-re-attach batch is that connection's baseline, so it does not \
         re-date the topic — but the row is Idle, NEVER `no_data`: {l:?}"
    );
    // The next one advances past it and the topic is streaming again.
    let pub_at = pub_at + PERIOD_MS * MS;
    clock.set(pub_at);
    publisher.publish_raw(&frame(100, pub_at)).expect("pub");
    assert_eq!(sweep_at(&mut obs, &clock, pub_at + 10 * MS), 1);
    assert_eq!(
        obs.liveness(&topic).expect("observed").state(),
        LivenessState::Streaming,
        "one advancing frame after the re-attach restores the dated reading"
    );
}

// ===========================================================================
// Degradation: UNKNOWN is never "dead".
// ===========================================================================

/// A topic nobody is watching reports UNKNOWN — never `NoData`.
#[test]
fn an_untracked_topic_reports_unknown_not_dead() {
    let (mgr, _clock) = manager("untracked");
    let topic = format!("/{}/untracked", unique("t"));
    let _publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");
    let obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    assert_eq!(obs.liveness(&topic), None);
}

/// A tracked topic whose tap can NEVER attach (here: the topic's service does not
/// exist) reports UNKNOWN, counts the failure loudly, and keeps retrying — it must
/// never drift into a confident "no data" it did not earn.
#[test]
fn a_topic_whose_tap_cannot_attach_stays_unknown_and_is_counted() {
    let (mgr, clock) = manager("noattach");
    let ghost = format!("/{}/ghost", unique("t"));
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    obs.track(&ghost);

    // Sweep repeatedly across a span far beyond the settle threshold.
    for i in 0..4u64 {
        sweep_at(
            &mut obs,
            &clock,
            T0 + (i + 1) * LIVENESS_NO_DATA_MIN_MS * MS,
        );
    }
    assert_eq!(obs.active_tap_count(), 0, "nothing ever attached");
    assert_eq!(
        obs.liveness(&ghost),
        None,
        "UNKNOWN — a topic we never actually watched must NOT be reported as dead, \
         however long it has been tracked"
    );
    assert_eq!(
        obs.attach_failure_count(&ghost),
        4,
        "every failed attach is counted (Principle #3) and retried"
    );

    // ANTI-TAUTOLOGY: create the service and the very next sweep heals.
    let mut publisher = mgr
        .create_publisher_simple(&ghost, MaxSliceLen::const_new(256))
        .expect("publisher");
    let heal = T0 + 10 * LIVENESS_NO_DATA_MIN_MS * MS;
    sweep_at(&mut obs, &clock, heal);
    assert_eq!(obs.active_tap_count(), 1, "the retry attached");
    for (i, at) in [heal + MS, heal + 3 * MS].into_iter().enumerate() {
        clock.set(at);
        publisher.publish_raw(&frame(i as u32, at)).expect("pub");
        assert_eq!(sweep_at(&mut obs, &clock, at + MS), 1);
    }
    assert_eq!(
        obs.liveness(&ghost).expect("observed").state(),
        LivenessState::Streaming
    );
}

/// A tap that attached and was then LOST reverts to UNKNOWN — it must not keep
/// serving the verdict it last computed, because nothing is watching to update
/// it. And a re-attach RESUMES the observation with its banked history.
///
/// The lost-tap stimulus is the production one: the observer's drain fails (the
/// canonical cause is a producer service torn down and re-created, which a stale
/// tap can never see), so the observer drops the tap and ends the interval.
/// Injected here with the `fault_inject_receive_after` seam the recorder tests
/// use, since a real teardown/recreate race is not deterministically stageable.
#[test]
fn an_observation_that_is_lost_reverts_to_unknown_and_resumes_on_re_attach() {
    let (mgr, clock) = manager("lost");
    let topic = format!("/{}/lost", unique("t"));
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    obs.track(&topic);
    sweep_at(&mut obs, &clock, T0);

    // Observe long enough (with no frames) that the topic has EARNED a confident
    // `NoData` — this is the verdict that must NOT freeze.
    let settled = T0 + (LIVENESS_NO_DATA_MIN_MS + 1_000) * MS;
    sweep_at(&mut obs, &clock, settled);
    assert_eq!(
        obs.liveness(&topic).expect("observed").state(),
        LivenessState::NoData,
        "precondition: a genuinely earned dead-route verdict"
    );

    // The tap dies mid-drain. The observer counts it, drops the tap, and — the
    // pin — stops claiming to observe.
    obs.fail_next_drain_for_test(&topic);
    sweep_at(&mut obs, &clock, settled + MS);
    assert_eq!(obs.drain_failure_count(&topic), 1, "the failure is counted");
    assert_eq!(obs.active_tap_count(), 0, "and the tap was dropped");
    assert_eq!(
        obs.liveness(&topic),
        None,
        "THE PIN: nothing is watching, so the robot reports UNKNOWN — it does not \
         keep serving a `no_data` verdict it can no longer maintain"
    );

    // A re-attach resumes: the banked observation time survives, so the topic
    // does not have to re-earn its verdict from zero.
    let reattach = settled + 100 * MS;
    sweep_at(&mut obs, &clock, reattach);
    assert_eq!(obs.active_tap_count(), 1, "the next sweep re-attached");
    let resumed = obs.liveness(&topic).expect("observing again");
    assert_eq!(
        resumed.observed_for_ms,
        (settled + MS - T0) / MS,
        "the banked pre-failure observation is intact; the 99 ms unobserved gap is \
         NOT counted (hand oracle)"
    );
    assert_eq!(
        resumed.state(),
        LivenessState::NoData,
        "and with its history back it is immediately the dead route again"
    );

    // ANTI-TAUTOLOGY: the topic really is still usable — a publish after the
    // re-attach is observed.
    for (i, at) in [reattach + MS, reattach + 3 * MS].into_iter().enumerate() {
        clock.set(at);
        publisher.publish_raw(&frame(i as u32, at)).expect("pub");
        assert_eq!(sweep_at(&mut obs, &clock, at + MS), 1);
    }
    assert_eq!(
        obs.liveness(&topic).expect("observed").state(),
        LivenessState::Streaming
    );
}

/// A zero tap budget observes nothing, and says so — UNKNOWN, never `NoData`.
#[test]
fn a_topic_past_the_tap_budget_reports_unknown() {
    let (mgr, clock) = manager("budget");
    let topic = format!("/{}/budget", unique("t"));
    let _publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 0, true);
    obs.track(&topic);
    sweep_at(&mut obs, &clock, T0);
    sweep_at(&mut obs, &clock, T0 + 10 * LIVENESS_NO_DATA_MIN_MS * MS);
    assert_eq!(obs.active_tap_count(), 0);
    assert_eq!(obs.liveness(&topic), None, "budget-excluded ⇒ UNKNOWN");
    assert_eq!(
        obs.tracked_count(),
        1,
        "it is still tracked (and will attach if the budget frees up)"
    );
}

/// `CERULION_TOPIC_LIVENESS=off` (modelled by the explicit disable flag, so the
/// test never mutates process env) makes the observer completely inert: no taps,
/// no records, no liveness — the earlier behaviour exactly.
#[test]
fn a_disabled_observer_is_completely_inert() {
    let (mgr, clock) = manager("off");
    let topic = format!("/{}/off", unique("t"));
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");
    let mut obs =
        TopicLivenessObserver::with_budget(Arc::clone(&mgr), DEFAULT_LIVENESS_TAP_BUDGET, false);
    assert!(!obs.is_enabled());
    obs.track(&topic);
    obs.set_externally_observed(&topic, true);
    obs.note_frames(&topic, drained(99, Some(T0 + MS)));
    clock.set(T0 + MS);
    publisher.publish_raw(&frame(0, T0 + MS)).expect("pub");
    assert_eq!(sweep_at(&mut obs, &clock, T0 + 2 * MS), 0);
    assert_eq!(obs.tracked_count(), 0, "a disabled observer tracks nothing");
    assert_eq!(obs.active_tap_count(), 0);
    assert_eq!(obs.liveness(&topic), None);
    assert_eq!(obs.sweep_count(), 0);
}

// ===========================================================================
// One port per topic: the egress hand-off.
// ===========================================================================

/// The cost contract: while someone else drains a topic, the observer holds NO tap
/// of its own, so a demanded topic never costs two gateway subscriber ports — and
/// the hand-off does not lose the observation, in either direction.
#[test]
fn external_observation_yields_the_tap_and_hands_it_back() {
    let (mgr, clock) = manager("handoff");
    let topic = format!("/{}/handoff", unique("t"));
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    obs.track(&topic);

    // Step 1: the observer owns the tap.
    sweep_at(&mut obs, &clock, T0);
    assert_eq!(obs.active_tap_count(), 1);

    // Step 2: an external drainer (the egress tap) takes over.
    obs.set_externally_observed(&topic, true);
    assert_eq!(
        obs.active_tap_count(),
        0,
        "the observer RELEASES its port — never two ports for one topic"
    );
    // Sweeping must not re-attach behind the external drainer's back.
    assert_eq!(sweep_at(&mut obs, &clock, T0 + 100 * MS), 0);
    assert_eq!(obs.active_tap_count(), 0);

    // The external drainer reports what IT drained; observation continues. Its
    // port is brand new, so its FIRST batch could be the publisher's flushed
    // backlog — the drainer cannot tell, so the batch is the advancement
    // baseline: banked, undated.
    clock.set(T0 + 100 * MS);
    obs.note_frames(&topic, drained(3, Some(T0 - 500 * MS)));
    let l = obs.liveness(&topic).expect("observed");
    assert_eq!(l.frames_observed, 3);
    assert_eq!(
        l.last_frame_age_ms, None,
        "the drainer's first batch is its baseline, not a proven arrival"
    );

    // Its SECOND batch ADVANCES past that baseline: a real publish.
    clock.set(T0 + 200 * MS);
    obs.note_frames(&topic, drained(2, Some(T0 + 200 * MS)));
    let l = obs.liveness(&topic).expect("observed");
    assert_eq!(l.frames_observed, 5);
    assert_eq!(l.last_frame_age_ms, Some(0));
    assert_eq!(
        l.observed_for_ms, 200,
        "observation time is CONTINUOUS across the hand-off (hand oracle)"
    );

    // A zero report is an observation of ABSENCE: the interval keeps running, the
    // age keeps growing.
    clock.set(T0 + 700 * MS);
    obs.note_frames(&topic, drained(0, None));
    let l = obs.liveness(&topic).expect("observed");
    assert_eq!(l.frames_observed, 5, "no frames added");
    assert_eq!(l.last_frame_age_ms, Some(500), "the age grew");

    // Step 3: demand ends. Until something re-attaches NOTHING is watching, so
    // the topic reports UNKNOWN rather than a verdict nobody can update.
    obs.set_externally_observed(&topic, false);
    clock.set(T0 + 750 * MS);
    assert_eq!(
        obs.liveness(&topic),
        None,
        "hand-off with no drainer ⇒ UNKNOWN until the observer's next sweep"
    );

    // The observer takes the topic back, with its history, and still sees data.
    sweep_at(&mut obs, &clock, T0 + 800 * MS);
    assert_eq!(obs.active_tap_count(), 1, "the observer re-attached");
    assert_eq!(
        obs.liveness(&topic)
            .expect("observing again")
            .frames_observed,
        5,
        "the banked history came back with the tap"
    );
    for (i, at) in [T0 + 900 * MS, T0 + 1_000 * MS].into_iter().enumerate() {
        clock.set(at);
        publisher
            .publish_raw(&frame(9 + i as u32, at))
            .expect("pub");
        assert_eq!(sweep_at(&mut obs, &clock, at + 10 * MS), 1);
    }
    let l = obs.liveness(&topic).expect("observed");
    assert_eq!(l.frames_observed, 7, "5 external + 2 own");
    assert_eq!(l.last_frame_age_ms, Some(0));
}

/// `note_frames` from a drainer that is NOT the declared external observer is
/// ignored — the observer only trusts observation it knows is happening, so a
/// stale/incorrect caller cannot fabricate liveness for a topic nothing watches.
#[test]
fn note_frames_without_a_declared_external_observer_is_ignored() {
    let (mgr, _clock) = manager("nofab");
    let topic = format!("/{}/nofab", unique("t"));
    let _publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    obs.track(&topic);
    obs.note_frames(&topic, drained(500, Some(T0)));
    assert_eq!(
        obs.liveness(&topic),
        None,
        "no declared observation ⇒ nothing recorded, so liveness stays UNKNOWN"
    );
}

// ===========================================================================
// Throttle, isolation, determinism.
// ===========================================================================

/// The sweep throttles itself, so wiring it into a hot drive loop costs a clock
/// read on nearly every pass rather than N drains.
#[test]
fn the_sweep_self_throttles_to_its_interval() {
    let (mgr, clock) = manager("throttle");
    let topic = format!("/{}/throttle", unique("t"));
    let _publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    obs.track(&topic);

    // First sweep always runs (never swept before).
    clock.set(T0);
    obs.sweep();
    assert_eq!(obs.sweep_count(), 1);

    // Well inside the interval: no further sweep, however many times it is driven.
    for i in 1..50u64 {
        clock.set(T0 + i * MS);
        obs.sweep();
    }
    assert_eq!(
        obs.sweep_count(),
        1,
        "49 drive passes inside one interval must execute ZERO extra sweeps"
    );

    // Exactly at the interval it runs again (the boundary is inclusive).
    clock.set(T0 + LIVENESS_SWEEP_INTERVAL_NS);
    obs.sweep();
    assert_eq!(obs.sweep_count(), 2);
}

/// The "never swept" marker is an `Option`, NOT a `0` sentinel — so an observer
/// running on a clock that genuinely reads 0 still sweeps.
///
/// A `VirtualClock` starts at 0, and every replay/deterministic-live path uses
/// one. With a `last_sweep_ns: u64` sentinel of 0 the throttle computes
/// `0.saturating_sub(0) < INTERVAL` and SKIPS the very first sweep — so an
/// observer at the clock's origin never attaches a tap and every topic reports
/// UNKNOWN forever. Mutation kill: revert the field to `u64` + `0` and this fails
/// at `sweep_count() == 1`.
#[test]
fn the_first_sweep_runs_even_at_virtual_clock_zero() {
    let (mgr, clock) = manager("zero");
    let topic = format!("/{}/zero", unique("t"));
    let _publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    obs.track(&topic);

    clock.set(0);
    obs.sweep();
    assert_eq!(
        obs.sweep_count(),
        1,
        "time 0 is a real instant, not `never swept`"
    );
    assert_eq!(
        obs.active_tap_count(),
        1,
        "and the sweep did its job — the tap attached"
    );
    assert!(
        obs.liveness(&topic).is_some(),
        "so the topic is genuinely under observation at the clock's origin"
    );

    // The throttle then applies from 0 exactly as from anywhere else.
    clock.set(LIVENESS_SWEEP_INTERVAL_NS - 1);
    obs.sweep();
    assert_eq!(obs.sweep_count(), 1, "still inside the first interval");
    clock.set(LIVENESS_SWEEP_INTERVAL_NS);
    obs.sweep();
    assert_eq!(obs.sweep_count(), 2);
}

/// C5: when there are more tracked topics than tap budget, WHICH topics get
/// observed must be DETERMINISTIC — the first `budget` in canonical (sorted)
/// topic order — not a hash-iteration accident that varies per process.
///
/// The budget makes the sweep's walk order observable, and `HashSet` iteration is
/// randomized per process by `RandomState`, so an unsorted walk lets the same robot
/// report a different subset of its topics on every restart with no explanation
/// available to the operator. Hand oracle: the two alphabetically-first topics of
/// five, asserted across a second independently-built observer (a different
/// `RandomState`) so the agreement is not one process's luck.
#[test]
fn an_over_budget_observer_picks_its_topics_deterministically() {
    let (mgr, clock) = manager("overbudget");
    let id = unique("t");
    // Deliberately TRACKED out of alphabetical order.
    let names = ["e", "b", "d", "a", "c"];
    let topics: Vec<String> = names.iter().map(|n| format!("/{id}/{n}")).collect();
    let _pubs: Vec<_> = topics
        .iter()
        .map(|t| {
            mgr.create_publisher_simple(t, MaxSliceLen::const_new(256))
                .expect("publisher")
        })
        .collect();

    const BUDGET: usize = 2;
    let observed_set = |clock_now: u64| {
        let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), BUDGET, true);
        for t in &topics {
            obs.track(t);
        }
        sweep_at(&mut obs, &clock, clock_now);
        assert_eq!(obs.active_tap_count(), BUDGET, "the budget really bound");
        // Sorted for comparison against the oracle: the claim is about WHICH
        // topics were chosen, not the order this probe happens to read them in.
        let mut observed: Vec<String> = topics
            .iter()
            .filter(|t| obs.liveness(t).is_some())
            .cloned()
            .collect();
        observed.sort();
        observed
    };

    // HAND oracle: canonical (sorted) order ⇒ the `/a` and `/b` rows.
    let mut expected: Vec<String> = topics.clone();
    expected.sort();
    expected.truncate(BUDGET);
    assert!(
        expected[0].ends_with("/a") && expected[1].ends_with("/b"),
        "oracle sanity: {expected:?}"
    );

    let first = observed_set(T0);
    assert_eq!(
        first, expected,
        "the observed subset is the canonically-first `budget` topics"
    );
    // A SECOND observer, built independently: same answer. (Two observers cannot
    // hold taps on the same topics at once, so the first is dropped by the time
    // this runs — `observed_set` builds and drops one per call.)
    let second = observed_set(T0 + 10 * LIVENESS_SWEEP_INTERVAL_NS);
    assert_eq!(
        second, expected,
        "and it does not vary between observers — the walk order is sorted, not \
         a HashSet iteration order"
    );
}

/// The OTHER half of the budget contract: an incumbent tap is never PREEMPTED.
/// A topic tracked AFTER the budget filled is refused, even when it sorts BEFORE
/// every incumbent — nothing takes a tap away to make room for it.
///
/// This is the `cerulion ros2 attach` shape (topics are registered as DDS
/// discovery finds them, so the tracked set grows at runtime), and it is why the
/// operator warn says the FIRST observed subset follows REGISTRATION order rather
/// than alphabetical order. Claiming "the canonically-first `budget` topics"
/// would send an operator looking for the alphabetically-first rows and hand them
/// a different set.
///
/// The complementary half — a FREED slot really is claimable, so the subset is
/// stable but not fixed — is
/// `a_slot_freed_by_a_hand_off_is_claimable_by_a_topic_refused_earlier`.
#[test]
fn a_topic_tracked_after_the_budget_filled_is_refused_even_if_it_sorts_first() {
    let (mgr, clock) = manager("sticky");
    let id = unique("t");
    // `/m` and `/n` are tracked first; `/a` sorts before BOTH and arrives late.
    let late = format!("/{id}/a");
    let incumbents = [format!("/{id}/m"), format!("/{id}/n")];
    let _pubs: Vec<_> = incumbents
        .iter()
        .chain(std::iter::once(&late))
        .map(|t| {
            mgr.create_publisher_simple(t, MaxSliceLen::const_new(256))
                .expect("publisher")
        })
        .collect();

    const BUDGET: usize = 2;
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), BUDGET, true);
    for t in &incumbents {
        obs.track(t);
    }
    sweep_at(&mut obs, &clock, T0);
    assert_eq!(obs.active_tap_count(), BUDGET, "the budget is now full");

    // The canonically-FIRST topic of the three shows up late.
    obs.track(&late);
    assert!(
        late < incumbents[0] && late < incumbents[1],
        "precondition: the late topic really does sort first ({late} vs {incumbents:?})"
    );
    sweep_at(&mut obs, &clock, T0 + LIVENESS_SWEEP_INTERVAL_NS);

    assert_eq!(
        obs.active_tap_count(),
        BUDGET,
        "no eviction: the budget is a cap on concurrent taps, not a ranking"
    );
    assert_eq!(
        obs.liveness(&late),
        None,
        "THE PIN: the canonically-first topic gets NO tap, because it arrived \
         after the budget filled — so the observed subset is arrival-ordered, \
         exactly as the operator warn now says (UNKNOWN, never a fabricated \
         `no data`)"
    );
    for t in &incumbents {
        assert!(
            obs.liveness(t).is_some(),
            "{t}: the incumbents keep their taps — nothing is preempted"
        );
    }
    assert_eq!(obs.tracked_count(), 3, "all three are still tracked");
}

/// The half of the warn that is easy to get wrong: taps are not held for
/// life. A slot freed by a demand HAND-OFF (`set_externally_observed`, which the
/// gateway drives on every egress attach) or by `release_own_tap` is claimable on
/// the next sweep — and because the sweep walks canonical order, it goes to the
/// canonically-first tracked topic that has none, which may well be one refused
/// earlier.
///
/// A warn saying taps "are never taken back, so the observed topics are
/// the first `budget` REACHED" would describe a policy this code does not have:
/// an operator reading it would conclude the observed subset is frozen at boot.
#[test]
fn a_slot_freed_by_a_hand_off_is_claimable_by_a_topic_refused_earlier() {
    let (mgr, clock) = manager("recycle");
    let id = unique("t");
    // `/m` and `/n` are tracked first and fill the budget; `/a` sorts before both
    // and arrives late, so it is refused (the sticky half above).
    let late = format!("/{id}/a");
    let incumbents = [format!("/{id}/m"), format!("/{id}/n")];
    let _pubs: Vec<_> = incumbents
        .iter()
        .chain(std::iter::once(&late))
        .map(|t| {
            mgr.create_publisher_simple(t, MaxSliceLen::const_new(256))
                .expect("publisher")
        })
        .collect();

    const BUDGET: usize = 2;
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), BUDGET, true);
    for t in &incumbents {
        obs.track(t);
    }
    sweep_at(&mut obs, &clock, T0);
    obs.track(&late);
    sweep_at(&mut obs, &clock, T0 + LIVENESS_SWEEP_INTERVAL_NS);
    assert_eq!(obs.liveness(&late), None, "precondition: refused, UNKNOWN");
    assert_eq!(obs.active_tap_count(), BUDGET);

    // A remote demands `/m`: the gateway hands its observation to the egress tap,
    // which FREES the observer's slot.
    obs.set_externally_observed(&incumbents[0], true);
    assert_eq!(
        obs.active_tap_count(),
        BUDGET - 1,
        "the hand-off really freed a slot"
    );

    // THE PIN: the next sweep gives that slot to `/a` — the topic refused before.
    sweep_at(&mut obs, &clock, T0 + 2 * LIVENESS_SWEEP_INTERVAL_NS);
    assert!(
        obs.liveness(&late).is_some(),
        "a freed slot IS claimable — the observed subset is stable, not frozen"
    );
    assert_eq!(
        obs.active_tap_count(),
        BUDGET,
        "and the budget is full again"
    );
    assert!(
        obs.liveness(&incumbents[0]).is_some(),
        "the handed-off topic is still OBSERVED (by the egress drainer), just not \
         by a tap of the observer's own"
    );

    // And when demand ends, the handed-off topic competes like anyone else: the
    // budget is full, so it is now the one reporting UNKNOWN until a slot frees.
    obs.set_externally_observed(&incumbents[0], false);
    sweep_at(&mut obs, &clock, T0 + 3 * LIVENESS_SWEEP_INTERVAL_NS);
    assert_eq!(
        obs.liveness(&incumbents[0]),
        None,
        "UNKNOWN — never a fabricated `no data` — which is exactly what the warn \
         now tells the operator to expect"
    );
}

/// A SUSTAINED drain-failure regime must not DIM a streaming topic, and must not
/// permanently deafen it once the regime ends.
///
/// Every failure drops the tap and ends the interval, so the next sweep
/// re-attaches and re-opens the advancement baseline. A topic whose only frame
/// per cycle IS that cycle's baseline therefore goes undated for as long as the
/// regime lasts — correctly, since a fresh connection's first batch could always
/// be a flushed backlog. What must NOT happen is the row being dimmed: the frames
/// are banked, so it reads `Idle` (produced, freshness unknown) throughout. And
/// once the regime ends, one attach carrying two batches dates it again.
///
/// SCOPE: this is a pathological regime (a tap that fails every sweep).
/// The cost is the age, never the row's visibility.
#[test]
fn a_repeating_drain_failure_regime_never_dims_a_streaming_topic() {
    let (mgr, clock) = manager("failregime");
    let topic = format!("/{}/failregime", unique("t"));
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    obs.track(&topic);

    const CYCLES: u32 = 3;
    let mut at = T0;
    for cycle in 0..CYCLES {
        // (1) the cycle's re-attach — which RE-OPENS the advancement baseline.
        at += 100 * MS;
        sweep_at(&mut obs, &clock, at);
        assert_eq!(obs.active_tap_count(), 1, "cycle {cycle}: re-attached");

        // (2) the topic publishes, and the ONE nonempty drain of this cycle
        //     observes it — this connection's baseline, so it is banked without
        //     dating. THE PIN: the row is Idle (produced), never the dimmed one.
        at += 10 * MS;
        clock.set(at);
        publisher.publish_raw(&frame(cycle, at)).expect("pub");
        at += 10 * MS;
        assert_eq!(sweep_at(&mut obs, &clock, at), 1, "cycle {cycle}: drained");
        let l = obs.liveness(&topic).expect("observing");
        assert_eq!(
            l.last_frame_age_ms, None,
            "cycle {cycle}: this connection's first batch is its baseline: {l:?}"
        );
        assert_eq!(
            l.frames_observed,
            u64::from(cycle) + 1,
            "cycle {cycle}: every observed frame is banked"
        );
        assert_eq!(
            l.state(),
            LivenessState::Idle,
            "cycle {cycle}: produced ⇒ Idle, NEVER the dimmed `no_data` row"
        );

        // (3) the drain fails again: the tap is dropped and the interval ends, so
        //     the topic reports UNKNOWN until the next re-attach.
        obs.fail_next_drain_for_test(&topic);
        at += 10 * MS;
        sweep_at(&mut obs, &clock, at);
        assert_eq!(
            obs.drain_failure_count(&topic),
            u64::from(cycle) + 1,
            "cycle {cycle}: the failure is counted"
        );
        assert_eq!(obs.active_tap_count(), 0, "cycle {cycle}: tap dropped");
        assert_eq!(obs.liveness(&topic), None, "cycle {cycle}: unattached");
    }

    // The regime ends. The banked observation is intact (hand oracle: 30 ms of
    // genuinely-attached time per cycle, the 100 ms unattached gaps excluded).
    at += 100 * MS;
    sweep_at(&mut obs, &clock, at);
    let l = obs.liveness(&topic).expect("observing");
    assert_eq!(
        l.observed_for_ms,
        u64::from(CYCLES) * 30,
        "banked observation excludes the unattached gaps (hand oracle): {l:?}"
    );
    assert_eq!(
        l.frames_observed,
        u64::from(CYCLES),
        "one observed frame per cycle"
    );

    // ANTI-TAUTOLOGY / recovery: with the tap now stable, two publishes inside
    // ONE attach date the topic again — the regime cost the age, not the ability
    // to ever measure one.
    for i in 0..2u32 {
        at += 10 * MS;
        clock.set(at);
        publisher.publish_raw(&frame(100 + i, at)).expect("pub");
        at += 10 * MS;
        assert_eq!(sweep_at(&mut obs, &clock, at), 1);
    }
    assert_eq!(
        obs.liveness(&topic).expect("observing").state(),
        LivenessState::Streaming,
        "a streaming topic recovers a dated reading once its tap stops failing"
    );
}

/// Two topics observed by one observer never cross-contaminate.
#[test]
fn two_topics_are_isolated() {
    let (mgr, clock) = manager("iso");
    let a = format!("/{}/a", unique("t"));
    let b = format!("/{}/b", unique("t"));
    let mut pa = mgr
        .create_publisher_simple(&a, MaxSliceLen::const_new(256))
        .expect("a");
    let _pb = mgr
        .create_publisher_simple(&b, MaxSliceLen::const_new(256))
        .expect("b");
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    obs.track(&a);
    obs.track(&b);
    sweep_at(&mut obs, &clock, T0);

    // Two rounds on A, one frame each so nothing is clipped by the shallow tap
    // buffer. Both are stamped after the attach, so each dates the topic.
    for (i, at) in [T0 + MS, T0 + 2 * MS].into_iter().enumerate() {
        clock.set(at);
        pa.publish_raw(&frame(i as u32, at)).expect("pub");
        assert_eq!(sweep_at(&mut obs, &clock, at + MS), 1);
    }
    let t = T0 + 3 * MS;
    assert_eq!(
        obs.liveness(&a),
        Some(TopicLiveness {
            last_frame_age_ms: Some(0),
            observed_for_ms: 3,
            frames_observed: 2,
            rate_estimate: None,
        })
    );
    assert_eq!(
        obs.liveness(&b),
        Some(TopicLiveness {
            last_frame_age_ms: None,
            observed_for_ms: 3,
            frames_observed: 0,
            rate_estimate: None,
        }),
        "B saw nothing — A's frames must not leak into it"
    );
    assert_eq!(t, T0 + 3 * MS);
}

/// The observer's tap is deliberately SHALLOW
/// ([`LIVENESS_TAP_BUFFER_SIZE`]), not ceiling-deep, so it can never pin a large
/// share of a fast producer's SHM pool between 200 ms sweeps. This pins both
/// halves of that trade:
///
/// * `frames_observed` UNDER-counts a burst deeper than the tap buffer (the cost);
/// * the AGE still tracks the topic's NEWEST frame (the thing that must not be
///   lost — every threshold keys on it, so a clipped queue must not make a live
///   topic look increasingly stale).
///
/// It also pins that the sweep drains its queue to EMPTY rather than one
/// borrow-chunk of it: the tap's borrow budget is the same 2, so a queue of 2 is
/// only emptied by the drain LOOP continuing past its first chunk.
#[test]
fn a_burst_deeper_than_the_tap_buffer_is_clipped_but_the_age_stays_exact() {
    let (mgr, clock) = manager("burst");
    let topic = format!("/{}/burst", unique("t"));
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    obs.track(&topic);
    sweep_at(&mut obs, &clock, T0);

    // First, a single post-attach frame, drained on its own sweep.
    clock.set(T0 + MS);
    publisher.publish_raw(&frame(0, T0 + MS)).expect("pub");
    assert_eq!(sweep_at(&mut obs, &clock, T0 + 2 * MS), 1);

    // A burst far deeper than the tap buffer, all between two sweeps.
    const BURST: u32 = 8;
    assert!(
        (BURST as usize) > LIVENESS_TAP_BUFFER_SIZE,
        "precondition: the burst must exceed the tap's queue depth"
    );
    let burst_at = T0 + 3 * MS;
    clock.set(burst_at);
    for i in 0..BURST {
        publisher.publish_raw(&frame(i, burst_at)).expect("pub");
    }
    let drained = sweep_at(&mut obs, &clock, burst_at + MS);
    assert_eq!(
        drained, LIVENESS_TAP_BUFFER_SIZE as u64,
        "the shallow tap retains only its buffer depth — the older frames were \
         reclaimed by the producer rather than pinned across the sweep"
    );
    let l = obs.liveness(&topic).expect("observed");
    assert_eq!(
        l.frames_observed,
        1 + LIVENESS_TAP_BUFFER_SIZE as u64,
        "so the count is a true lower bound, not the publish total"
    );
    assert_eq!(
        l.last_frame_age_ms,
        Some(0),
        "but the AGE is exact — the NEWEST frame is always retained, and every \
         threshold keys on the age"
    );
    assert_eq!(l.state(), LivenessState::Streaming);

    // The queue really is empty now — the next sweep observes nothing.
    assert_eq!(sweep_at(&mut obs, &clock, burst_at + 2 * MS), 0);
}

/// The whole observation sequence is deterministic: two identical runs produce
/// identical snapshots, and both equal a hand oracle.
#[test]
fn observation_is_deterministic_and_matches_the_hand_oracle() {
    fn run(tag: &str) -> Vec<Option<TopicLiveness>> {
        let (mgr, clock) = manager(tag);
        let topic = format!("/{}/det", unique("t"));
        let mut publisher = mgr
            .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
            .expect("publisher");
        let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
        obs.track(&topic);
        let mut out = Vec::new();
        sweep_at(&mut obs, &clock, T0);
        out.push(obs.liveness(&topic));
        // Publish on steps 1 and 2, then go silent for step 3.
        for step in 1..=3u64 {
            let now = T0 + step * 1_000 * MS;
            if step <= 2 {
                clock.set(now - MS);
                publisher
                    .publish_raw(&frame(step as u32, now))
                    .expect("pub");
            }
            sweep_at(&mut obs, &clock, now);
            out.push(obs.liveness(&topic));
        }
        out
    }
    let oracle = vec![
        Some(TopicLiveness {
            last_frame_age_ms: None,
            observed_for_ms: 0,
            frames_observed: 0,
            rate_estimate: None,
        }),
        // Step 1: the first drain after the attach — the advancement baseline, so
        // the frame is banked and the topic is NOT dated.
        Some(TopicLiveness {
            last_frame_age_ms: None,
            observed_for_ms: 1_000,
            frames_observed: 1,
            rate_estimate: None,
        }),
        Some(TopicLiveness {
            last_frame_age_ms: Some(0),
            observed_for_ms: 2_000,
            frames_observed: 2,
            rate_estimate: None,
        }),
        Some(TopicLiveness {
            last_frame_age_ms: Some(1_000),
            observed_for_ms: 3_000,
            frames_observed: 2,
            rate_estimate: None,
        }),
    ];
    let a = run("det_a");
    let b = run("det_b");
    assert_eq!(a, oracle, "run A must equal the HAND oracle");
    assert_eq!(b, oracle, "run B must equal the HAND oracle");
    assert_eq!(a, b, "and the two runs must agree");
    // The classification timeline the oracle implies, spelled out: baseline
    // (produced, undatable) ⇒ Idle, then dated ⇒ Streaming, and 1 s of silence is
    // still inside the recency window.
    assert_eq!(oracle[0].expect("observed").state(), LivenessState::Unknown);
    assert_eq!(
        oracle[1].expect("observed").state(),
        LivenessState::Idle,
        "the baseline batch leaves the topic produced-but-undated"
    );
    assert_eq!(
        oracle[2].expect("observed").state(),
        LivenessState::Streaming
    );
    assert_eq!(
        oracle[3].expect("observed").state(),
        LivenessState::Streaming,
        "1 s of silence is still within the recency window"
    );
}

// ===========================================================================
// The observer and the publisher are on different clocks.
// ===========================================================================

/// A PUBLISHER manager and an OBSERVER manager on the SAME SHM root but on
/// UNRELATED clocks — the shipping multi-process shape.
///
/// On the default `graph run` (multi-process by default, with a separate gateway
/// process) the publisher is a `graph run-worker` whose `VirtualClock` starts at 0
/// and advances by the handed logical quantum, while the observer lives in the
/// gateway on a `RealClock` counting nanoseconds since BOOT. Every existing test
/// in this file wires ONE clock into both, so none of them can see a rule that
/// silently depends on the two agreeing.
///
/// The observer clock here is a `VirtualClock` rather than a real one so the ages
/// stay exact hand oracles; what matters is that the two number lines are
/// unrelated, and the `offset` argument puts the publisher's arbitrarily far
/// BELOW or ABOVE the observer's — the two regimes that break an attach-instant
/// comparison in opposite directions.
fn cross_clock_pair(
    tag: &str,
    publisher_base_ns: u64,
) -> (
    Arc<TransportManager>,
    Arc<VirtualClock>,
    Arc<TransportManager>,
    Arc<VirtualClock>,
) {
    let root = cerulion_core::testing::iceoryx_test_config();
    let pub_clock = Arc::new(VirtualClock::new());
    pub_clock.set(publisher_base_ns);
    let obs_clock = Arc::new(VirtualClock::new());
    obs_clock.set(T0);
    let pub_mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: unique(&format!("{tag}_p")),
            clock: pub_clock.clone() as Arc<dyn Clock>,
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init publisher manager");
    let obs_mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: unique(&format!("{tag}_o")),
            clock: obs_clock.clone() as Arc<dyn Clock>,
            ..Default::default()
        },
        root,
    )
    .expect("init observer manager");
    (pub_mgr, pub_clock, obs_mgr, obs_clock)
}

/// THE CROSS-CLOCK HEADLINE: a dead-but-history-carrying route stays undated when
/// the publisher's stamps sit FAR ABOVE the observer's clock.
///
/// This is the regime an attach-instant comparison gets exactly backwards. The
/// observer attaches at its own `T0` (1 s on its number line); the publisher's
/// backlog is stamped ~11.5 days up ITS number line (a worker `VirtualClock`
/// stepping faster than wall time, or simply a graph that has been running a
/// while). "Is the frame stamped after the attach instant" answers YES for every
/// one of those stale frames, so an attach-instant gate is INERT and the dead route
/// renders `Streaming` then `Idle` forever — the hazard on
/// the default deployment.
///
/// The advancement rule never looks at the observer's clock, so the flush is just
/// a baseline: banked, undated, `Idle`, never dimmed and never fresh.
///
/// Reinstating the
/// earlier one-shot attach-instant gate fails this test with
/// `last_frame_age_ms: Some(0)` and `Streaming`; so does the non-one-shot variant
/// of the same filter; so does computing the age as `now − wire stamp`.
#[test]
fn a_dead_route_with_history_never_dates_when_the_publisher_clock_is_far_ahead() {
    // ~11.5 days of publisher-clock, against an observer at 1 s.
    const PUB_BASE: u64 = 1_000_000_000 * MS;
    let (pub_mgr, pub_clock, obs_mgr, obs_clock) = cross_clock_pair("xclock_ahead", PUB_BASE);
    let topic = format!("/{}/xahead", unique("t"));
    const HISTORY: usize = 4;
    let mut publisher = pub_mgr
        .create_publisher(&topic, MaxSliceLen::const_new(256), HISTORY)
        .expect("publisher with retained history");

    // The dead route's backlog, laid down before anything watched.
    for i in 0..HISTORY as u32 {
        publisher.publish_raw(&frame(i, PUB_BASE)).expect("pub");
    }
    // The regime that makes an attach-instant gate INERT: every backlog
    // stamp sits ABOVE the observer's attach instant. Const-asserted so a future
    // edit to either constant cannot silently make this test non-probative.
    const _: () = assert!(
        PUB_BASE > T0,
        "precondition: every backlog stamp must be ABOVE the observer's clock"
    );

    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&obs_mgr), 8, true);
    obs.track(&topic);
    assert_eq!(sweep_at(&mut obs, &obs_clock, T0), 0, "attach, nothing yet");

    // Nudge the publisher into flushing its retained history (a listener-full
    // subscriber connects, exactly as `bagd`/`topic echo`/vizd would).
    let _late_joiner = obs_mgr
        .create_subscriber(&topic)
        .expect("a listener-full subscriber connects");
    pub_clock.set(PUB_BASE + 1_000 * MS);
    publisher.pump_history();
    let drained_count = sweep_at(&mut obs, &obs_clock, T0 + 100 * MS);
    assert!(
        drained_count > 0,
        "precondition (measured): the retained history really does reach the tap"
    );

    let l = obs.liveness(&topic).expect("observing");
    assert_eq!(
        l.last_frame_age_ms, None,
        "THE PIN: stamps far ABOVE the observer's clock are still just a baseline \
         — the rule never compares the two number lines: {l:?}"
    );
    assert_eq!(l.frames_observed, drained_count, "every frame banked");
    assert_ne!(
        l.state(),
        LivenessState::Streaming,
        "the false-live an attach-instant comparison produces in this regime"
    );

    // Far past the settle threshold it is STILL Idle: produced, never dimmed.
    let settled = T0 + (LIVENESS_NO_DATA_MIN_MS + 5_000) * MS;
    sweep_at(&mut obs, &obs_clock, settled);
    let l = obs.liveness(&topic).expect("observing");
    assert_eq!(l.last_frame_age_ms, None);
    assert_eq!(
        l.state(),
        LivenessState::Idle,
        "produced but undatable ⇒ Idle, at any observation length: {l:?}"
    );
}

/// The OTHER cross-clock regime: a STREAMING topic whose publisher stamps sit FAR
/// BELOW the observer's clock must still be dated, and its age must be the
/// OBSERVER's arithmetic.
///
/// A `graph run-worker`'s `VirtualClock` starts at 0, so on a robot that
/// has been up for minutes every frame it ever publishes is stamped below the
/// gateway's `RealClock` reading. Advancement is a relation between SUCCESSIVE
/// publisher stamps, so it dates this topic on the second observed batch exactly
/// as it would if the clocks coincided.
///
/// Scope, established against three alternative implementations:
///
/// * `now − wire stamp` as the age (the tempting wrong design the module's Clocks
///   section forbids) — FAILS here: the age reads as the whole clock offset, so a
///   streaming topic renders permanently `Idle`.
/// * a PERMANENT "only date frames stamped after the tap attached" filter — FAILS
///   here: no frame ever qualifies, so the topic is never dated at all.
/// * the earlier one-shot attach gate — passes here, because discarding exactly one
///   batch per attach is what this design does deliberately anyway. That gate is
///   killed by the far-AHEAD test above, where it dates a dead route's flushed
///   backlog. The two tests are complementary halves; neither alone covers the
///   break.
#[test]
fn a_streaming_topic_dates_when_the_publisher_clock_is_far_below_the_observers() {
    // A worker VirtualClock that started at 0, versus an observer at T0 = 1 s.
    let (pub_mgr, pub_clock, obs_mgr, obs_clock) = cross_clock_pair("xclock_below", 0);
    let topic = format!("/{}/xbelow", unique("t"));
    let mut publisher = pub_mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");

    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&obs_mgr), 8, true);
    obs.track(&topic);
    sweep_at(&mut obs, &obs_clock, T0);

    // Three publishes on the PUBLISHER's number line, each drained on its own
    // sweep on the OBSERVER's. Every stamp is far below every observer instant.
    let mut obs_now = T0;
    for step in 1..=3u64 {
        let stamp = step * 10 * MS;
        assert!(
            stamp < obs_now,
            "precondition: publisher stamp {stamp} is BELOW the observer clock \
             {obs_now} — the regime that makes an attach-instant gate a blanket discard"
        );
        pub_clock.set(stamp);
        publisher
            .publish_raw(&frame(step as u32, stamp))
            .expect("pub");
        obs_now += 100 * MS;
        assert_eq!(sweep_at(&mut obs, &obs_clock, obs_now), 1, "step {step}");

        let l = obs.liveness(&topic).expect("observing");
        if step == 1 {
            assert_eq!(
                l.last_frame_age_ms, None,
                "step 1 is the baseline (banked, undated) — but PRODUCED, so the \
                 row is already Idle, not the dimmed one: {l:?}"
            );
            assert_eq!(l.state(), LivenessState::Idle);
        } else {
            assert_eq!(
                l.last_frame_age_ms,
                Some(0),
                "THE PIN (step {step}): a publisher stamp far BELOW the observer's \
                 clock still dates the topic, because the rule compares it to the \
                 PREVIOUS STAMP and never to the observer's clock: {l:?}"
            );
            assert_eq!(l.state(), LivenessState::Streaming);
        }
    }

    // And it ages on the OBSERVER's clock, not the publisher's (hand oracle).
    let quiet = obs_now + 2_000 * MS;
    sweep_at(&mut obs, &obs_clock, quiet);
    let l = obs.liveness(&topic).expect("observing");
    assert_eq!(
        l.last_frame_age_ms,
        Some(2_000),
        "the AGE is observer-clock arithmetic — the publisher's number line never \
         enters it: {l:?}"
    );
    assert_eq!(l.frames_observed, 3);
}

/// The `/tf_static` shape ACROSS clocks: one latched publish, ever, from a
/// publisher whose clock is unrelated to the observer's. It reads `Idle` —
/// produced, freshness unknown — in both directions of clock skew, and never
/// `no_data` (the dimmed row) or `streaming` (a freshness nothing proved).
#[test]
fn a_latched_one_shot_reads_idle_whichever_way_the_clocks_are_skewed() {
    for (tag, publisher_base) in [
        ("xclock_tfs_below", 0u64),
        ("xclock_tfs_ahead", 1_000_000_000 * MS),
    ] {
        let (pub_mgr, pub_clock, obs_mgr, obs_clock) = cross_clock_pair(tag, publisher_base);
        let topic = format!("/{}/tf_static", unique("t"));
        let mut publisher = pub_mgr
            .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
            .expect("publisher");
        let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&obs_mgr), 8, true);
        obs.track(&topic);
        sweep_at(&mut obs, &obs_clock, T0);

        // The ONE publish this topic will ever make.
        let stamp = publisher_base + 5 * MS;
        pub_clock.set(stamp);
        publisher.publish_raw(&frame(0, stamp)).expect("pub");
        assert_eq!(sweep_at(&mut obs, &obs_clock, T0 + 100 * MS), 1, "{tag}");

        // And nothing ever again, far past the settle threshold.
        let settled = T0 + (LIVENESS_NO_DATA_MIN_MS + 5_000) * MS;
        assert_eq!(sweep_at(&mut obs, &obs_clock, settled), 0, "{tag}");
        let l = obs.liveness(&topic).expect("observing");
        assert_eq!(l.frames_observed, 1, "{tag}: the one frame is banked");
        assert_eq!(l.last_frame_age_ms, None, "{tag}: nothing datable");
        assert_eq!(
            l.state(),
            LivenessState::Idle,
            "{tag}: a latched one-shot is Idle whichever way the clocks are \
             skewed — never dimmed, never fresh: {l:?}"
        );
    }
}

// ===========================================================================
// The publisher restarts under a longer-lived observer.
// ===========================================================================

/// THE RESTART PIN, end to end over real iceoryx2: a publisher streams, is
/// RESTARTED onto a fresh clock that begins FAR BELOW its predecessor's, and the
/// topic must be dated again within ONE advancement — not "never again for the
/// length of the previous run".
///
/// This is a shipping-deployment shape, not a contrivance. The observer lives in
/// the embedded gateway of the per-computer `cerulion-netd` daemon, which
/// outlives any number of `cerulion graph run` invocations, while each run's
/// `graph run-worker` owns a `VirtualClock` that starts again at 0. The
/// record's stamp maximum was MONOTONE, so after a two-hour run every frame of
/// the restarted run compared BELOW it and `advanced` was false for another two
/// hours: a 20 Hz topic that is visibly streaming rendered `Idle` with no age.
/// That is the exact inversion this feature exists to prevent, and precisely the
/// state an operator restarts to escape.
///
/// The fix reads a stamp REGRESSION as structural evidence of a new clock epoch
/// (one writer's one clock cannot go backwards within a run), adopts the new
/// stamp as the maximum, and re-opens the baseline so the restarted publisher's
/// own retained history still cannot date it. A regression alone is not enough,
/// though — a later chunk of one RE-FLUSHED backlog regresses identically — so the
/// reset also requires a CLOSED baseline and `REGRESSION_RESET_MIN_GAP_NS` of
/// observer-clock silence. This test therefore models the restart's own duration
/// as frameless sweeps rather than pretending a worker reboots between two drains.
///
/// Deleting the `regressed`
/// arm from `LivenessRecord::record_frames` fails the final assertion here with
/// `last_frame_age_ms: Some(300)` — the age frozen at the pre-restart arrival and
/// growing forever — instead of `Some(0)`.
#[test]
fn a_restarted_publisher_dates_again_within_one_advancement() {
    // Run 1 has been up for two hours on its own clock; its replacement's clock
    // begins again near zero. Const-asserted so a future edit to either constant
    // cannot silently make this scenario non-probative.
    const RUN1_BASE: u64 = 2 * 60 * 60 * 1_000 * MS;
    const RUN2_BASE: u64 = 5 * MS;
    const _: () = assert!(
        RUN2_BASE < RUN1_BASE,
        "precondition: the restarted worker's stamps must begin BELOW the dead \
         run's maximum — which is what a VirtualClock restarting at 0 does"
    );

    let root = cerulion_core::testing::iceoryx_test_config();
    let make = |tag: &str, base: u64| {
        let clock = Arc::new(VirtualClock::new());
        clock.set(base);
        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: unique(tag),
                clock: clock.clone() as Arc<dyn Clock>,
                ..Default::default()
            },
            root.clone(),
        )
        .expect("init manager");
        (mgr, clock)
    };

    let (obs_mgr, obs_clock) = make("restart_o", T0);
    let topic = format!("/{}/restart", unique("t"));

    // ---- Run 1: a worker that has been publishing for two hours. ----
    let (run1_mgr, run1_clock) = make("restart_w1", RUN1_BASE);
    let mut run1_pub = run1_mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("run-1 publisher");

    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&obs_mgr), 8, true);
    obs.track(&topic);
    sweep_at(&mut obs, &obs_clock, T0);

    run1_pub
        .publish_raw(&frame(0, run1_clock.now_ns()))
        .expect("run-1 publish");
    assert_eq!(sweep_at(&mut obs, &obs_clock, T0 + 100 * MS), 1);
    assert_eq!(
        obs.liveness(&topic).expect("observing").last_frame_age_ms,
        None,
        "the first observed batch is the baseline"
    );
    run1_clock.set(RUN1_BASE + 10 * MS);
    run1_pub
        .publish_raw(&frame(1, run1_clock.now_ns()))
        .expect("run-1 publish");
    assert_eq!(sweep_at(&mut obs, &obs_clock, T0 + 200 * MS), 1);
    let l = obs.liveness(&topic).expect("observing");
    assert_eq!(l.last_frame_age_ms, Some(0), "run 1 dates normally: {l:?}");
    assert_eq!(l.state(), LivenessState::Streaming);

    // ---- THE RESTART. ----
    // The worker process goes away (its publisher AND its manager), freeing the
    // single-writer slot, and the replacement attaches to the SAME topic on a
    // FRESH clock two hours below the dead run's last stamp.
    drop(run1_pub);
    drop(run1_mgr);
    let (run2_mgr, run2_clock) = make("restart_w2", RUN2_BASE);
    let mut run2_pub = run2_mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("run-2 publisher — the restarted worker");

    // Sweeps across the restart's own duration, ALL of them frameless: a process
    // restart is teardown + boot + graph build + service open, and the observer
    // sees it as SILENCE. That silence is the second guard on the epoch reset
    // (`REGRESSION_RESET_MIN_GAP_NS` — a re-flushed backlog's chunks arrive a drain
    // cadence apart, never after a lull), so it is modelled here rather than
    // skipped. The first of these sweeps may also drop a tap the teardown broke
    // (the canonical drain failure) and the rest guarantee a live tap before the
    // new run publishes.
    const RESTART_DONE: u64 = T0 + 2_000 * MS;
    const _: () = assert!(
        RESTART_DONE - (T0 + 200 * MS) >= REGRESSION_RESET_MIN_GAP_NS,
        "precondition: the restart must present to the observer as silence longer \
         than the re-flush gate"
    );
    for at in [300, 400, 900, 1_400, 1_900] {
        sweep_at(&mut obs, &obs_clock, T0 + at * MS);
    }
    let banked = obs.liveness(&topic).expect("observing").frames_observed;

    run2_pub
        .publish_raw(&frame(0, run2_clock.now_ns()))
        .expect("run-2 publish");
    sweep_at(&mut obs, &obs_clock, RESTART_DONE);
    let l = obs.liveness(&topic).expect("observing");
    assert!(
        l.frames_observed > banked,
        "precondition (measured): the restarted publisher's frames really do \
         reach the observer's tap: {l:?}"
    );
    assert_eq!(
        l.last_frame_age_ms,
        Some(1_800),
        "the FIRST post-restart batch does not date the topic — it may be the new \
         run's own retained-history flush, so it only resets the epoch and \
         re-opens the baseline; the age is still the pre-restart arrival: {l:?}"
    );

    // ---- ONE advancement later, the topic is live again. ----
    run2_clock.set(RUN2_BASE + MS);
    run2_pub
        .publish_raw(&frame(1, run2_clock.now_ns()))
        .expect("run-2 publish");
    sweep_at(&mut obs, &obs_clock, RESTART_DONE + 100 * MS);
    let l = obs.liveness(&topic).expect("observing");
    assert_eq!(
        l.last_frame_age_ms,
        Some(0),
        "THE PIN — PRE-FIX THIS IS `Some(1_900)` AND GROWING FOREVER: the restarted \
         publisher would have had to climb past two hours of the dead run's clock \
         before a single frame could date this topic again: {l:?}"
    );
    assert_eq!(
        l.state(),
        LivenessState::Streaming,
        "and the row is live again, not the `Idle` a monotone maximum froze it \
         into: {l:?}"
    );

    // It keeps dating as the new run climbs — the reset is not one-shot luck.
    run2_clock.set(RUN2_BASE + 2 * MS);
    run2_pub
        .publish_raw(&frame(2, run2_clock.now_ns()))
        .expect("run-2 publish");
    sweep_at(&mut obs, &obs_clock, RESTART_DONE + 200 * MS);
    assert_eq!(
        obs.liveness(&topic).expect("observing").last_frame_age_ms,
        Some(0),
        "the record now genuinely lives on the new epoch's number line"
    );
}

// ===========================================================================
// The RATE ESTIMATE, over real transport.
// ===========================================================================

/// Sweeps in one rate window, derived from the two shipped constants.
const WINDOW_SWEEPS: u64 = RATE_ESTIMATE_MIN_WINDOW_NS / LIVENESS_SWEEP_INTERVAL_NS;

/// Drive `publisher` at `per_sweep` frames per sweep for one whole rate window,
/// sweeping the observer on the sweep grid, and return the frames the observer
/// DRAINED plus the observer-clock instant of the closing sweep.
///
/// `seq_of(k, i)` chooses the wire sequence of the `i`-th frame published before
/// sweep `k`, which is the only thing the two arms below differ in: a real
/// publisher's monotone counter, or a raw publisher's stalled one.
fn drive_one_window(
    obs: &mut TopicLivenessObserver,
    clock: &VirtualClock,
    publisher: &mut cerulion_core::CerulionPublisher,
    per_sweep: u32,
    seq_of: impl Fn(u64, u32) -> u32,
) -> (u64, u64) {
    // The attach sweep: the tap exists from here, so every frame below is
    // published while the robot is genuinely watching.
    assert_eq!(
        sweep_at(obs, clock, T0),
        0,
        "the attach sweep drains nothing"
    );
    let step = LIVENESS_SWEEP_INTERVAL_NS / u64::from(per_sweep);
    let mut drained = 0u64;
    let mut sweep_t = T0;
    // One extra sweep beyond the window: the FIRST is the advancement baseline
    // (and the rate window's anchor), and the window is measured from it.
    for k in 1..=(WINDOW_SWEEPS + 1) {
        let prev = sweep_t;
        sweep_t = T0 + k * LIVENESS_SWEEP_INTERVAL_NS;
        for i in 0..per_sweep {
            let ts = prev + (u64::from(i) + 1) * step;
            clock.set(ts);
            publisher
                .publish_raw(&frame(seq_of(k, i), ts))
                .expect("publish");
        }
        drained += sweep_at(obs, clock, sweep_t);
    }
    (drained, sweep_t)
}

/// THE headline, over REAL iceoryx2: a topic publishing far faster than
/// the observer's tap can hold is reported at its REAL rate.
///
/// The ask is a frequency for a topic nobody has checked, and the
/// topics that matter (`/lowstate` and friends) run orders of magnitude above the
/// observer's own throughput. The tap keeps [`LIVENESS_TAP_BUFFER_SIZE`] frames
/// and drains once per [`LIVENESS_SWEEP_INTERVAL_NS`], so counting the frames the
/// robot CAUGHT would pin every such topic at the same 10 Hz. Counting the
/// publisher's committed SEQUENCES does not.
///
/// Both numbers are asserted in the same body, which is what makes this a
/// discrimination rather than a claim about one value: the observer drained 22
/// frames across 2.2 s of observation — exactly 10 Hz if you count frames — and
/// serves 100 Hz, which is what the publisher actually did.
#[test]
fn a_topic_far_faster_than_the_tap_reports_its_real_rate_not_the_frames_we_caught() {
    let (mgr, clock) = manager("fast");
    let topic = format!("/{}/fast", unique("t"));
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    obs.track(&topic);

    // 20 commits per 200 ms sweep = 100 Hz, ten times the tap's own throughput.
    const PER_SWEEP: u32 = 20;
    let (drained, at) = drive_one_window(&mut obs, &clock, &mut publisher, PER_SWEEP, |k, i| {
        (k as u32 - 1) * PER_SWEEP + i
    });

    // What the tap could actually hold — the FLOOR, and the reason a frames-based
    // estimate is useless here.
    assert_eq!(
        drained,
        (WINDOW_SWEEPS + 1) * LIVENESS_TAP_BUFFER_SIZE as u64,
        "precondition: every sweep clipped to the tap depth, so the frame count \
         is an order of magnitude below the truth"
    );
    let l = obs.liveness(&topic).expect("observing");
    assert_eq!(l.frames_observed, drained);
    assert_eq!(
        l.observed_for_ms, 2_200,
        "precondition: 22 frames across 2.2 s — a frames-counting estimate reads \
         exactly 10 Hz here"
    );
    assert_eq!(
        l.state(),
        LivenessState::Streaming,
        "precondition: the stamps dated it, so a rate may be served at all"
    );

    assert_eq!(
        l.rate_estimate,
        Some(TopicRateEstimate {
            millihertz: 100_000,
            is_floor: false,
        }),
        "the publisher committed 200 frames across the 2 s window, and that is \
         what the robot must report — not the 10 Hz its own tap saw"
    );
    assert!(
        l.rate_estimate.expect("carried").hz() > 50.0,
        "and the two answers must be far apart, or this arm proves nothing"
    );

    // The clock is the manager's, so the observation ages with it: past the
    // recency window the topic stops being Streaming and stops claiming a rate.
    clock.set(at + (LIVENESS_STREAMING_RECENCY_MS + 1) * MS);
    let l = obs.liveness(&topic).expect("still observing");
    assert_eq!(l.state(), LivenessState::Idle);
    assert_eq!(
        l.rate_estimate, None,
        "a stream that stopped must DROP its rate — never decay one, never keep \
         serving the last measurement while the row says the topic is idle"
    );
}

/// The FLOOR basis, over REAL iceoryx2, and the reason it exists.
///
/// `publish_raw` writes the caller's header VERBATIM, so a hand-rolled raw
/// publisher can emit a sequence that never advances — and the sequence delta on
/// such a topic is 0, which as a rate is a confident and completely wrong "0 Hz"
/// on a topic that is plainly streaming. The correct answer is the frames the
/// observer itself drained, LABELLED as a floor.
///
/// The value it lands on is [`RATE_FLOOR_BASIS_CEILING_MHZ`] — the ceiling the
/// tap depth and the sweep interval derive — because the tap is saturated on
/// every sweep. That is the saturation case in one arm: the row says "at least
/// 10 Hz" about a topic really running at 100.
#[test]
fn a_publisher_whose_sequence_never_advances_serves_a_labelled_floor_not_zero() {
    let (mgr, clock) = manager("floor");
    let topic = format!("/{}/floor", unique("t"));
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
    obs.track(&topic);

    // The SAME 100 Hz stimulus as the arm above — only the sequence differs.
    const PER_SWEEP: u32 = 20;
    let (drained, _at) = drive_one_window(&mut obs, &clock, &mut publisher, PER_SWEEP, |_, _| 7);

    assert_eq!(
        drained,
        (WINDOW_SWEEPS + 1) * LIVENESS_TAP_BUFFER_SIZE as u64,
        "precondition: the same clipped drain as the exact arm"
    );
    let l = obs.liveness(&topic).expect("observing");
    assert_eq!(l.state(), LivenessState::Streaming);
    assert_eq!(
        l.rate_estimate,
        Some(TopicRateEstimate {
            millihertz: RATE_FLOOR_BASIS_CEILING_MHZ,
            is_floor: true,
        }),
        "an uncountable sequence must fall back to the frames actually drained \
         and SAY it is a floor — the topic is at 100 Hz, the robot can only \
         vouch for the ceiling its own tap allows"
    );
    assert_ne!(
        l.rate_estimate.expect("carried").millihertz,
        0,
        "and it must never be the 0 Hz the sequence delta literally implies"
    );
}

/// A topic past the tap budget carries no observation at all, so there is
/// nothing for a rate to hang on — and the topic that DID get a tap serves one
/// in the same body, which is what makes this a discrimination rather than a
/// claim that nothing works.
#[test]
fn a_topic_past_the_tap_budget_serves_no_rate_while_an_observed_one_does() {
    let (mgr, clock) = manager("rate_budget");
    // Two topics, ONE tap slot. `sweep` walks in canonical order, so the
    // alphabetically-first topic wins it.
    let stem = unique("t");
    let observed = format!("/{stem}/aaa");
    let refused = format!("/{stem}/zzz");
    let mut publisher = mgr
        .create_publisher_simple(&observed, MaxSliceLen::const_new(256))
        .expect("publisher");
    let _refused_pub = mgr
        .create_publisher_simple(&refused, MaxSliceLen::const_new(256))
        .expect("publisher — exists, but never gets a tap");
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 1, true);
    obs.track(&observed);
    obs.track(&refused);

    const PER_SWEEP: u32 = 4;
    drive_one_window(&mut obs, &clock, &mut publisher, PER_SWEEP, |k, i| {
        (k as u32 - 1) * PER_SWEEP + i
    });

    assert_eq!(obs.active_tap_count(), 1, "precondition: one slot, one tap");
    assert_eq!(
        obs.liveness(&observed).expect("observing").rate_estimate,
        Some(TopicRateEstimate {
            // 4 commits per 200 ms = 20 Hz.
            millihertz: 20_000,
            is_floor: false,
        }),
        "the topic that got the slot is measured exactly"
    );
    assert_eq!(
        mgr.topic_publisher_count_checked(&refused),
        Some(1),
        "the refused topic has a live publisher — the old signal still sees it"
    );
    assert_eq!(
        obs.liveness(&refused),
        None,
        "but nothing is observing it, so it reports UNKNOWN — never a fabricated \
         0 Hz, and never a rate borrowed from the topic next to it"
    );
}

/// The whole rate computation is deterministic (Principle #7): two identical
/// runs over real transport produce identical estimates, and both equal a hand
/// oracle — so neither is a self-compare.
#[test]
fn the_rate_estimate_is_deterministic_over_real_transport() {
    fn run(tag: &str) -> Option<TopicRateEstimate> {
        let (mgr, clock) = manager(tag);
        let topic = format!("/{}/det", unique("t"));
        let mut publisher = mgr
            .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
            .expect("publisher");
        let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&mgr), 8, true);
        obs.track(&topic);
        const PER_SWEEP: u32 = 6;
        drive_one_window(&mut obs, &clock, &mut publisher, PER_SWEEP, |k, i| {
            (k as u32 - 1) * PER_SWEEP + i
        });
        obs.liveness(&topic).expect("observing").rate_estimate
    }
    let a = run("rate_det_a");
    let b = run("rate_det_b");
    assert_eq!(a, b, "same script, same answer");
    assert_eq!(
        a,
        Some(TopicRateEstimate {
            // 6 commits per 200 ms sweep = 30 Hz.
            millihertz: 30_000,
            is_floor: false,
        }),
        "and both equal the hand-computed rate"
    );
}

// ---------------------------------------------------------------------------
// The SUSTAINED-regression reset, over real transport.
// ---------------------------------------------------------------------------

/// THE HEADLINE, over REAL iceoryx2 on the OBSERVER plane: a publisher
/// restarts WITHOUT a drain-visible lull and keeps streaming, and the topic
/// recovers a dated `Streaming` reading.
///
/// The difference from `a_restarted_publisher_dates_again_within_one_advancement`
/// is the whole point: that test models the restart's own SILENCE (five frameless
/// sweeps across teardown + boot + graph build + service open), which is what
/// clears [`REGRESSION_RESET_MIN_GAP_NS`]. This one gives the observer NO silence
/// at all — every single sweep drains frames, one sweep interval apart, which is
/// const-asserted to be inside the gate — so `quiet_long_enough` is FALSE at every
/// drain and the silence path cannot fire. It is not an exotic shape: a supervisor
/// restarting a crashed node in ~100 ms produces it, and so does any restart the
/// observer happens to see through a continuously-fed tap.
///
/// BEFORE THE SUSTAINED-REGRESSION RESET THIS NEVER HEALS. The dead run's two-hour high-water mark stands
/// for as long as the replacement keeps streaming, and the row serves the DEAD
/// run's age growing a second every second.
///
/// The no-lull property is MEASURED rather than assumed: every sweep of the regime
/// must actually drain frames (`frames_observed` strictly grows), or the test fails
/// its own precondition instead of quietly measuring the silence path.
#[test]
fn a_lull_free_restart_dates_again_within_the_confirmation_window() {
    const RUN1_BASE: u64 = 2 * 60 * 60 * 1_000 * MS;
    const RUN2_BASE: u64 = 5 * MS;
    const _: () = assert!(
        RUN2_BASE < RUN1_BASE,
        "precondition: the replacement's clock begins BELOW the dead run's \
         maximum — what a VirtualClock restarting at 0 does"
    );
    /// The observer's own grid. Every drain in this test is one of these apart.
    const S: u64 = LIVENESS_SWEEP_INTERVAL_NS;
    const _: () = assert!(
        S < REGRESSION_RESET_MIN_GAP_NS,
        "THE PRECONDITION THAT MAKES THIS TEST ABOUT THE SUSTAINED PATH: consecutive drains \
         are one sweep apart, which must be INSIDE the silence gate, or the old \
         path could clear on its own and this would prove nothing"
    );
    const SPAN_SWEEPS: u64 = SUSTAINED_REGRESSION_MIN_SPAN_NS / S;

    let root = cerulion_core::testing::iceoryx_test_config();
    let make = |tag: &str, base: u64| {
        let clock = Arc::new(VirtualClock::new());
        clock.set(base);
        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: unique(tag),
                clock: clock.clone() as Arc<dyn Clock>,
                ..Default::default()
            },
            root.clone(),
        )
        .expect("init manager");
        (mgr, clock)
    };

    let (obs_mgr, obs_clock) = make("o", T0);
    let topic = format!("/{}/lullfree", unique("t"));

    // ---- Run 1: two hours up, dating normally. ----
    let (run1_mgr, run1_clock) = make("w1", RUN1_BASE);
    let mut run1_pub = run1_mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("run-1 publisher");
    let mut obs = TopicLivenessObserver::with_budget(Arc::clone(&obs_mgr), 8, true);
    obs.track(&topic);
    sweep_at(&mut obs, &obs_clock, T0);

    run1_pub
        .publish_raw(&frame(0, run1_clock.now_ns()))
        .expect("run-1 publish");
    assert_eq!(sweep_at(&mut obs, &obs_clock, T0 + S), 1, "baseline batch");
    run1_clock.set(RUN1_BASE + 10 * MS);
    run1_pub
        .publish_raw(&frame(1, run1_clock.now_ns()))
        .expect("run-1 publish");
    const DATED_AT: u64 = T0 + 2 * S;
    assert_eq!(sweep_at(&mut obs, &obs_clock, DATED_AT), 1);
    let l = obs.liveness(&topic).expect("observing");
    assert_eq!(l.last_frame_age_ms, Some(0), "run 1 dates normally: {l:?}");
    assert_eq!(l.state(), LivenessState::Streaming);

    // ---- THE RESTART, with no observer-visible pause at all. ----
    // The worker process goes away and its replacement attaches to the SAME topic
    // on a FRESH clock two hours below the dead run's last stamp — and the very
    // next sweep already has its frames. No sweep in this test is ever frameless.
    drop(run1_pub);
    drop(run1_mgr);
    let (run2_mgr, run2_clock) = make("w2", RUN2_BASE);
    let mut run2_pub = run2_mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("run-2 publisher — the restarted worker");

    // The regime: one publish per sweep, one sweep apart, from here to the
    // confirmation boundary.
    const REGIME_START: u64 = DATED_AT + S;
    const CONFIRM_AT: u64 = REGIME_START + SUSTAINED_REGRESSION_MIN_SPAN_NS;
    let mut seq = 0u32;
    let mut at = REGIME_START;
    while at <= CONFIRM_AT {
        run2_clock.set(RUN2_BASE + u64::from(seq) * MS);
        run2_pub
            .publish_raw(&frame(seq, run2_clock.now_ns()))
            .expect("run-2 publish");
        let before = obs.liveness(&topic).expect("observing").frames_observed;
        sweep_at(&mut obs, &obs_clock, at);
        let l = obs.liveness(&topic).expect("observing");
        assert!(
            l.frames_observed > before,
            "PRECONDITION (measured): every sweep of this regime must drain \
             frames, or the silence gate could clear and the test would be \
             measuring the wrong path (at {at} ns): {l:?}"
        );
        if at < CONFIRM_AT {
            assert_eq!(
                l.last_frame_age_ms,
                Some((at - DATED_AT) / MS),
                "while the regime builds, the row serves the DEAD run's age, \
                 growing — the affirmatively wrong claim the sustained reset bounds: {l:?}"
            );
        }
        at += S;
        seq += 1;
    }
    assert_eq!(
        obs.drain_failure_count(&topic),
        0,
        "precondition: the observer's tap survived the worker swap, so the \
         baseline was never re-opened and this really is the lull-free shape"
    );
    assert_eq!(
        obs.liveness(&topic).expect("observing").last_frame_age_ms,
        Some((CONFIRM_AT - DATED_AT) / MS),
        "the resetting batch itself never dates — it may be the new run's own \
         retained-history flush"
    );

    // ---- ONE advancement past the reset, the row is live again. ----
    run2_clock.set(RUN2_BASE + u64::from(seq) * MS);
    run2_pub
        .publish_raw(&frame(seq, run2_clock.now_ns()))
        .expect("run-2 publish");
    sweep_at(&mut obs, &obs_clock, at);
    let l = obs.liveness(&topic).expect("observing");
    assert_eq!(
        l.last_frame_age_ms,
        Some(0),
        "THE PIN — BEFORE THE SUSTAINED RESET THIS IS `Some({})` AND GROWING FOREVER, because a \
         lull-free restart can never satisfy the silence gate: {l:?}",
        (at - DATED_AT) / MS
    );
    assert_eq!(
        l.state(),
        LivenessState::Streaming,
        "and the row is live again, not the `Idle` a frozen maximum pinned it \
         into: {l:?}"
    );
    assert_eq!(
        obs.liveness(&topic).expect("observing").frames_observed,
        SPAN_SWEEPS + 4,
        "two run-1 batches, the whole regime (span + 1 sweeps), and the batch \
         that dated it"
    );

    // It keeps dating as the new run climbs — the reset moved the record onto the
    // new epoch's number line rather than getting lucky once.
    seq += 1;
    at += S;
    run2_clock.set(RUN2_BASE + u64::from(seq) * MS);
    run2_pub
        .publish_raw(&frame(seq, run2_clock.now_ns()))
        .expect("run-2 publish");
    sweep_at(&mut obs, &obs_clock, at);
    assert_eq!(
        obs.liveness(&topic).expect("observing").last_frame_age_ms,
        Some(0),
        "the record genuinely lives on the new epoch now"
    );
}
