// SPDX-License-Identifier: AGPL-3.0-only
//! Ingress-build hold end-to-end: a recorder must not freeze its channel set while a
//! process on the machine is still BUILDING its ingress plane.
//!
//! # The shape these arms reproduce
//!
//! A freshly-booted robot's FIRST `graph run attach --single-process --record`
//! captured 4 topics — the declared set — while all ~98 of the bridge's raw
//! routes were reported `appeared_after_bag_creation`. The identical command two
//! minutes later, against a warm DDS bus, captured 102 COMPLETE. The bridge
//! opens its raw routes SEQUENTIALLY and a cold DDS bus makes each open wait, so
//! the routes arrive in BURSTS separated by multi-second gaps — and any gap over
//! half a second satisfies the quiet rule and closes a channel set that
//! can never re-open.
//!
//! Every arm below builds that shape with the PRODUCTION seam: routes are minted
//! by real [`TransportManager::create_ingress_publisher`] calls, the same call
//! the `dds_bridge`'s `RawIngressRoute::open` makes. Nothing here reaches into
//! the progress channel by hand, so an inert producing side fails these tests
//! rather than passing them.
//!
//! # Why the gaps are what they are
//!
//! [`BURST_GAP`] is deliberately LONGER than the quiet-scan release
//! (`DISCOVERY_SETTLE_QUIET_SCANS` × `DISCOVERY_RESCAN_INTERVAL` = 500 ms) — a
//! shorter gap would be carried by the settle alone and would pin nothing — and
//! comfortably SHORTER than the shipped 5 s stall window, so a loaded runner
//! would have to stretch a gap SIX-fold before the headline arm's hold released
//! early. The anti-tautology control does not depend on a wall at all: it waits
//! for the bag FILE to exist before creating its late routes, so those routes
//! are provably post-creation whatever the machine is doing (the load-inversion
//! rule: never assert a wall band a loaded runner can invert).
//!
//! Isolated per-instance transports ([`common::make_manager`]), so the live
//! enumeration under test sees exactly this test's topics.

mod common;

use std::collections::BTreeSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use cerulion_bag::BagReader;
use cerulion_bagd::{
    run_bagd, BagdConfig, BagdError, BagdSummary, IngressBuildRelease, RecordCoverage, RunBinding,
    TapSpec, UntappedReason, INGRESS_BUILD_STALL_WINDOW, RECORD_COVERAGE_ATTACHMENT,
};
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::run_registry::{RunHandle, RunRecord, RunState};
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;

use common::*;

/// An arbitrary, stable schema hash for the hand-built frames.
const HASH: u64 = 0x1243_1243_1243_1243;

/// The gap between route bursts. LONGER than the quiet-scan release
/// (500 ms), so without the hold the channel set closes inside it; far SHORTER
/// than the shipped 5 s stall window, so the hold carries it with 6× headroom.
const BURST_GAP: Duration = Duration::from_millis(800);

/// How many routes each burst mints. Two, so a burst is a burst and the
/// per-writer running count advances more than once inside one.
const ROUTES_PER_BURST: usize = 2;

/// The wire sequence of the frame the headline arm publishes AFTER its route is
/// tapped — distinct from every mint-time sequence, so the payload oracle cannot
/// be satisfied by a frame that predates the tap.
const LATE_SEQ: u32 = 77;

/// The stall window the SETTLED-PLANE arm runs with. Small, because that arm
/// sleeps PAST it to make its plane genuinely settled; see the arm's own docs
/// for why the window is scaled rather than the wait.
const SETTLED_STALL_OVERRIDE: Duration = Duration::from_millis(300);

/// A gap longer than the WHOLE `DEFAULT_DISCOVERY_SETTLE_MS` cap (2 s), so an
/// arm that spans it needs no timing argument for "the settle would have closed
/// the channel set here" — the settle cannot hold past its own ceiling. Still
/// well inside the shipped 5 s stall window, which is what the hold must use.
const OVER_SETTLE_GAP: Duration = Duration::from_millis(2500);

/// The stall window the SHUTDOWN arm runs with. Deliberately far larger than
/// that arm's own runtime, so "the recorder exited before this elapsed" is an
/// unambiguous statement that the signal was ACTED ON rather than waited out.
const SHUTDOWN_STALL_OVERRIDE: Duration = Duration::from_secs(60);

/// Bounded wait for the bag file. Stated in seconds, not in units of anything
/// under test.
const BAG_FILE_DEADLINE: Duration = Duration::from_secs(20);

/// Long enough that MANY discovery rescans provably ran — eight intervals, so a
/// runner would have to starve the scanner (a WORKER thread now, not
/// the drive loop) eight-fold before this were short. Used only where the
/// property under test is "the recorder ENUMERATED this and classified it"; the
/// load-bearing assertions around it never depend on a wall.
const MANY_RESCANS: Duration =
    Duration::from_millis((cerulion_bagd::DISCOVERY_RESCAN_INTERVAL.as_millis() as u64) * 8);

/// Block until bagd's discovery rescan has ATTACHED a tap to `topic`.
///
/// A data-only tap requests no late-joiner history, so a frame committed BEFORE
/// the tap attaches lands in no queue at all and can never be recovered. Any arm
/// that asserts on a route's FRAMES must therefore rendezvous with the tap's
/// existence, not sleep and hope — the lesson `discovery_e2e_test`'s own
/// `await_rescan_tap` was written for. `topic_subscriber_count` is a production
/// accessor reading iceoryx2's dynamic config; it creates no port, so asking
/// costs the recorder no budget.
///
/// Returns whether the tap appeared, deliberately rather than asserting: a tap
/// that never attaches is the DEFECT (the channel set closed before the
/// route was discovered), and the caller can say so far more usefully than a
/// generic harness panic can.
#[must_use]
fn await_rescan_tap(mgr: &TransportManager, topic: &str) -> bool {
    let start = std::time::Instant::now();
    while mgr.topic_subscriber_count(topic) == 0 {
        if start.elapsed() >= Duration::from_secs(20) {
            return false;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    true
}

fn spawn_bagd(
    mgr: Arc<TransportManager>,
    cfg: BagdConfig,
    shutdown: Arc<AtomicBool>,
) -> JoinHandle<Result<BagdSummary, BagdError>> {
    std::thread::spawn(move || run_bagd(mgr, cfg, shutdown))
}

/// An EXACT declared tap, so `learned_all()` is true from the first pass and the
/// ONLY thing that can hold bag creation open is a settle term. That isolation
/// is the point: an attach-mode declared tap would ride `schema_wait` and could
/// make a broken hold look like a working one.
fn exact_tap(topic: &str) -> TapSpec {
    TapSpec::exact(
        topic,
        cerulion_bag::TopicSchema {
            topic: topic.to_string(),
            schema_name: "geometry_msgs/Vector3".to_string(),
            schema_hash: HASH,
            wire_fixed_size: 24,
        },
    )
}

/// Fast cadences, status OFF, discovery ON (the default), schema-wait
/// SHORT so it can never be the gate that keeps the channel set open.
fn cfg_for(out: std::path::PathBuf, declared: &str, ready: std::path::PathBuf) -> BagdConfig {
    let mut cfg = BagdConfig::new(out, vec![exact_tap(declared)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.ready_file = Some(ready);
    cfg.status_period = None;
    cfg.schema_wait = Duration::from_millis(200);
    cfg.discover_live = true;
    cfg
}

/// Mint ONE runtime ingress route through the PRODUCTION seam — the same call
/// `RawIngressRoute::open` makes — and publish one hand-oracle frame on it.
fn mint_route(mgr: &TransportManager, topic: &str, seq: u32) -> CerulionPublisher {
    let mut pubr = mgr
        .create_ingress_publisher(topic, MaxSliceLen::const_new(4096))
        .expect("create_ingress_publisher");
    let frame = build_frame(HASH, seq, 1_000 + u64::from(seq), &oracle_body(topic, seq));
    pubr.publish_raw(&frame).expect("publish_raw");
    pubr
}

/// Mint ONE runtime ingress route through the PRODUCTION seam and publish
/// NOTHING on it — the shape an arm needs when it asserts EXACTLY which frames a
/// route carried.
///
/// [`mint_route`] publishes a frame at mint time, and whether that frame reaches
/// the bag is a RACE the test cannot decide: `create_ingress_publisher` creates
/// the `{topic}/data` service and only then returns, so the recorder's 250 ms
/// discovery rescan can enumerate that service and attach its tap in the window
/// BEFORE the mint frame is published — normally microseconds wide, but a
/// preempted test thread widens it past a whole rescan interval. When the tap
/// wins, the mint frame lands in the bag beside the late one and an exact frame
/// oracle fails on a recording that is perfectly correct (MEASURED on a real run:
/// the cold-boot arm's oracle came back holding `[mint, late]`).
///
/// A route minted SILENT has nothing to race: the only frame that can ever exist
/// on it is the one published after the tap is known to be attached, so the
/// oracle becomes exact by construction. That is strictly STRONGER than the
/// wall-free version of the same assertion — an accidental extra frame still
/// fails it, and it is the load-inversion rule applied to a frame set rather than to a
/// duration.
fn mint_route_silent(mgr: &TransportManager, topic: &str) -> CerulionPublisher {
    mgr.create_ingress_publisher(topic, MaxSliceLen::const_new(4096))
        .expect("create_ingress_publisher")
}

/// The hand oracle for a route's frame.
fn oracle_body(topic: &str, i: u32) -> Vec<u8> {
    format!("{topic}-{i}").into_bytes()
}

/// Names for burst `b`'s routes.
fn burst_topics(base: &str, b: usize) -> Vec<String> {
    (0..ROUTES_PER_BURST)
        .map(|r| format!("{base}/b{b}r{r}"))
        .collect()
}

fn read_coverage(out: &std::path::Path) -> RecordCoverage {
    let reader = BagReader::open(out).expect("open bag");
    let att = reader
        .attachment(RECORD_COVERAGE_ATTACHMENT)
        .expect("read attachments")
        .expect("record_coverage.json is present in EVERY finalized bag");
    serde_json::from_slice(&att.data).expect("record_coverage.json parses")
}

fn user_channels(out: &std::path::Path) -> BTreeSet<String> {
    let reader = BagReader::open(out).expect("open bag");
    reader
        .channels()
        .expect("channels")
        .into_iter()
        .map(|c| c.topic)
        .filter(|t| !t.starts_with("__cerulion/"))
        .collect()
}

fn frames_for(out: &std::path::Path, topic: &str) -> Vec<Vec<u8>> {
    let reader = BagReader::open(out).expect("open bag");
    let (msgs, completeness) = reader.recover_messages().expect("recover_messages");
    assert!(
        completeness.is_finalized(),
        "bag must be Finalized, got {completeness:?}"
    );
    msgs.into_iter()
        .filter(|m| m.topic == topic)
        .map(|m| m.data)
        .collect()
}

fn finish(
    handle: JoinHandle<Result<BagdSummary, BagdError>>,
    shutdown: &Arc<AtomicBool>,
) -> BagdSummary {
    shutdown.store(true, Ordering::Relaxed);
    handle.join().expect("bagd thread").expect("clean finalize")
}

// ===========================================================================
// THE HEADLINE — a bursty ingress build is carried across its gaps
// ===========================================================================

/// A route build whose gaps each EXCEED the quiet rule still lands
/// whole in the bag.
///
/// Runtime ingress routes commonly arrive in bursts: three bursts of routes,
/// [`BURST_GAP`] apart, where each gap is long enough to satisfy the
/// quiet rule and close the channel set. Before the hold only burst 0 could be
/// recorded — which is exactly the "4 topics of 102" bag the issue was filed on.
///
/// The oracle is the CHANNEL SET plus the coverage manifest, not a wall: every
/// route topic must be a channel, none may be reported
/// `AppearedAfterBagCreation`, and the manifest must attribute the hold to
/// ingress-build progress with the ordinary (stalled) release.
#[test]
#[serial_test::serial]
fn a_bursty_ingress_build_is_carried_across_its_gaps_and_lands_whole_in_the_bag() {
    let mgr = make_manager(16);
    let declared = unique_topic("hl_declared");
    let base = unique_topic("hl_route");
    let out = unique_out("headline");
    let ready = unique_out("headline_ready");

    let mut declared_pub = publisher_with_provisioning(&mgr, &declared, 8, 16, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(
        Arc::clone(&mgr),
        cfg_for(out.clone(), &declared, ready.clone()),
        Arc::clone(&shutdown),
    );
    assert!(wait_for_file(&ready, Duration::from_secs(20)), "bagd ready");

    // The declared tap speaks immediately, so nothing but a settle term can hold
    // creation open (see `exact_tap`).
    declared_pub
        .publish_raw(&build_frame(HASH, 0, 1, &oracle_body(&declared, 0)))
        .expect("declared publish");

    // Three bursts, each gap longer than the quiet rule. The LAST route of the
    // last burst is the one whose frames the payload oracle reads back exactly,
    // so it is minted SILENT — see `mint_route_silent` for the race a mint-time
    // frame leaves open; that race has fired on a real run.
    let mut routes: Vec<String> = Vec::new();
    let mut keep_alive: Vec<CerulionPublisher> = Vec::new();
    let burst_count = 3usize;
    for b in 0..burst_count {
        if b > 0 {
            std::thread::sleep(BURST_GAP);
        }
        let topics = burst_topics(&base, b);
        let last_of_all = topics.len() - 1;
        for (r, topic) in topics.into_iter().enumerate() {
            let silent = b == burst_count - 1 && r == last_of_all;
            keep_alive.push(if silent {
                mint_route_silent(&mgr, &topic)
            } else {
                mint_route(&mgr, &topic, b as u32)
            });
            routes.push(topic);
        }
    }
    // The last burst's route, once TAPPED, publishes the frame this arm's payload
    // oracle reads back. Doing it here — while the hold is still open, before
    // the channel set closes — is the whole point: a route discovered mid-build
    // must be recordable, not merely nameable.
    let oracle_route = routes.last().expect("routes").clone();
    assert!(
        await_rescan_tap(&mgr, &oracle_route),
        "the recorder never tapped '{oracle_route}' — a route minted MID-BUILD went undiscovered, \
         which means the channel set closed while the plane was still being built. That IS the \
         defect under test: the ingress-build hold never engaged, or released early"
    );
    keep_alive
        .last_mut()
        .expect("routes")
        .publish_raw(&build_frame(
            HASH,
            LATE_SEQ,
            9_000,
            &oracle_body(&oracle_route, LATE_SEQ),
        ))
        .expect("late publish");

    // Let the hold release on its own (stall window) and the bag get created,
    // rather than forcing creation with the shutdown flag.
    assert!(
        wait_for_file(&out, INGRESS_BUILD_STALL_WINDOW + BAG_FILE_DEADLINE),
        "the ingress-build hold must RELEASE on its own once the build stalls — the bag file \
         never appeared, which means the hold never let go"
    );
    // Three MORE routes, now that the channel set is closed. They exist only to
    // pin that the manifest reports what was known AT CLOSE, not whatever the
    // machine did afterwards.
    for r in 0..3u32 {
        keep_alive.push(mint_route(&mgr, &format!("{base}/post{r}"), 100 + r));
    }
    std::thread::sleep(MANY_RESCANS);
    let summary = finish(handle, &shutdown);

    let channels = user_channels(&out);
    for topic in &routes {
        assert!(
            channels.contains(topic),
            "route '{topic}' is missing from the bag's channel set. The channel set closed \
             mid-build — the exact defect the hold closes. Channels: {channels:?}"
        );
    }

    let coverage = &summary.record_coverage;
    for topic in &routes {
        assert!(
            !matches!(
                coverage.untapped.get(topic),
                Some(UntappedReason::AppearedAfterBagCreation)
            ),
            "route '{topic}' was REPORTED rather than recorded: {:?}",
            coverage.untapped.get(topic)
        );
    }

    let ingress = coverage
        .ingress_build
        .expect("the manifest must say WHY the channel set stayed open this long");
    assert_eq!(
        ingress.released,
        IngressBuildRelease::Stalled,
        "the build finished, so the hold must end on the ORDINARY arm"
    );
    assert_eq!(
        ingress.writers, 1,
        "one producing process minted every route"
    );
    assert_eq!(
        ingress.routes_observed,
        (3 * ROUTES_PER_BURST) as u64,
        "EXACTLY the routes that existed when the channel set closed. The three routes minted \
         AFTER the bag was created must not be counted — the manifest's number explains the \
         bag, so folding past creation would make it describe a set the bag does not have"
    );
    // A floor, not an exact count: each mint sends immediately AND the republish
    // belt re-sends the running total, so a coalesced delivery can legitimately
    // produce fewer advances than routes. Two is what matters — the hold re-armed
    // at least once AFTER the first burst, which is the property the discovery
    // quiet rule could not provide.
    assert!(
        ingress.advances >= 2,
        "expected the hold to re-arm across bursts, got {} advances",
        ingress.advances
    );

    // The DURABLE surface: the terminal summary is a log line, the bag outlives
    // it, so the on-disk manifest must carry the identical block.
    let on_disk = read_coverage(&out);
    assert_eq!(
        on_disk.ingress_build, coverage.ingress_build,
        "record_coverage.json must carry the same ingress-build block the summary reports"
    );
    assert_eq!(
        on_disk.version,
        cerulion_bagd::RECORD_COVERAGE_VERSION,
        "the block is ADDITIVE — it must not have forced a schema version bump"
    );

    // One route end to end, so this is not merely a channel-registration pin.
    // The oracle frame is the one published AFTER the tap attached (frame 2, the
    // one minted with the route, predates the tap and a data-only tap has no
    // back-fill — that is `prefix_lost` territory, not this arm's subject).
    assert_eq!(
        frames_for(&out, &oracle_route),
        vec![build_frame(
            HASH,
            LATE_SEQ,
            9_000,
            &oracle_body(&oracle_route, LATE_SEQ)
        )],
        "the last burst's route must carry the frame it published once tapped — byte-identical, \
         header included — not just a placeholder channel"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ===========================================================================
// THE ANTI-TAUTOLOGY CONTROL — the same script, the hold disabled
// ===========================================================================

/// The identical producing script with the ingress-build hold turned off loses
/// every route minted after bag creation.
///
/// Without this arm the headline proves nothing: a recorder that captured those
/// routes for some unrelated reason (a slow runner, a lucky scan) would pass it.
/// One knob differs — `ingress_stall_window_override = ZERO`, which makes
/// [`cerulion_bagd::decide_ingress_build_hold`] release immediately — so the
/// discovery settle, the schema wait, the tap plan and the producing calls are all
/// byte-identical to the headline's.
///
/// It is WALL-FREE by construction: instead of sleeping a gap and hoping the bag
/// was created inside it, the late bursts wait for the bag FILE to exist. That
/// makes "these routes appeared after the channel set closed" a fact rather than
/// a timing bet, which is what keeps this arm valid on a loaded runner.
#[test]
#[serial_test::serial]
fn with_the_hold_disabled_the_same_build_loses_every_route_minted_after_creation() {
    let mgr = make_manager(16);
    let declared = unique_topic("ctl_declared");
    let base = unique_topic("ctl_route");
    let out = unique_out("control");
    let ready = unique_out("control_ready");

    let mut cfg = cfg_for(out.clone(), &declared, ready.clone());
    cfg.ingress_stall_window_override = Some(Duration::ZERO);

    let mut declared_pub = publisher_with_provisioning(&mgr, &declared, 8, 16, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(Arc::clone(&mgr), cfg, Arc::clone(&shutdown));
    assert!(wait_for_file(&ready, Duration::from_secs(20)), "bagd ready");
    declared_pub
        .publish_raw(&build_frame(HASH, 0, 1, &oracle_body(&declared, 0)))
        .expect("declared publish");

    let mut keep_alive: Vec<CerulionPublisher> = Vec::new();
    for topic in burst_topics(&base, 0) {
        keep_alive.push(mint_route(&mgr, &topic, 0));
    }

    // THE discriminator: the bag exists, so the channel set is provably closed.
    assert!(
        wait_for_file(&out, BAG_FILE_DEADLINE),
        "with the hold disabled the bag must be created promptly — if it is not, this control \
         is not controlling anything"
    );

    let mut late: Vec<String> = Vec::new();
    for b in 1..3 {
        std::thread::sleep(BURST_GAP);
        for topic in burst_topics(&base, b) {
            keep_alive.push(mint_route(&mgr, &topic, b as u32));
            late.push(topic);
        }
    }
    // Give the enumerator room to SEE the late routes. This wall governs only the
    // reporting claim below; the load-bearing claim — that they are not in the
    // bag — cannot be affected by waiting longer, since the channel set closed
    // before they existed.
    std::thread::sleep(MANY_RESCANS);
    let summary = finish(handle, &shutdown);

    let channels = user_channels(&out);
    let mut reported = 0usize;
    for topic in &late {
        assert!(
            !channels.contains(topic),
            "'{topic}' was minted AFTER the bag existed, so it cannot have a channel — if it \
             does, the control is not disabling the hold and the headline arm proves nothing"
        );
        match summary.record_coverage.untapped.get(topic) {
            Some(UntappedReason::AppearedAfterBagCreation) => reported += 1,
            Some(other) => {
                panic!("'{topic}' appeared after bag creation but is reported {other:?}")
            }
            None => {}
        }
    }
    assert!(
        reported > 0,
        "no late route was enumerated at all after {MANY_RESCANS:?} — the enumerator never ran, \
         so this control's reporting claim would be vacuous"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ===========================================================================
// NO SIGNAL: byte-identical to the behaviour before the hold
// ===========================================================================

/// A recording whose only producers are ORDINARY publishers makes NO
/// ingress-build claim at all.
///
/// This is the arm that keeps the hold additive. Every graph that is not a
/// progressively-built ingress plane — every plain graph, every
/// `cerulion bag record` — takes it, and a manifest that carried an
/// `ingress_build` block here would be asserting a hold that never happened.
#[test]
#[serial_test::serial]
fn a_recording_with_no_ingress_routes_makes_no_ingress_build_claim() {
    let mgr = make_manager(16);
    let declared = unique_topic("none_declared");
    let other = unique_topic("none_other");
    let out = unique_out("nosignal");
    let ready = unique_out("nosignal_ready");

    let mut declared_pub = publisher_with_provisioning(&mgr, &declared, 8, 16, 4096);
    let mut other_pub = publisher_with_provisioning(&mgr, &other, 8, 16, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(
        Arc::clone(&mgr),
        cfg_for(out.clone(), &declared, ready.clone()),
        Arc::clone(&shutdown),
    );
    assert!(wait_for_file(&ready, Duration::from_secs(20)), "bagd ready");
    declared_pub
        .publish_raw(&build_frame(HASH, 0, 1, &oracle_body(&declared, 0)))
        .expect("declared publish");
    other_pub
        .publish_raw(&build_frame(HASH, 0, 1, &oracle_body(&other, 0)))
        .expect("other publish");

    // The bag must be created without any ingress term holding it.
    assert!(
        wait_for_file(&out, BAG_FILE_DEADLINE),
        "a recording that hears no ingress-build progress must not be held at all"
    );
    settle();
    let summary = finish(handle, &shutdown);

    assert_eq!(
        summary.record_coverage.ingress_build, None,
        "no runtime ingress route existed, so the manifest must make NO claim — not a \
         zero-valued one"
    );
    // Anti-vacuity: discovery really was ON and really did work, so `None` above
    // is "nothing to report", not "nothing was watching".
    assert!(summary.record_coverage.enumerated);
    assert!(
        user_channels(&out).contains(&other),
        "the ordinary undeclared producer must still be discovered and recorded"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ===========================================================================
// COLD BOOT — the recorder is armed BEFORE the first route exists
// ===========================================================================

/// The recorder arms FIRST, the producer's very FIRST route arrives, and a gap
/// longer than the whole settle CAP follows before route 2 — which still lands
/// in the bag.
///
/// This is the literal `graph run --record` cold-boot shape, and it is the
/// scenario the defect was filed on: the recorder is armed before the graph is
/// released to step 0, so the FIRST progress record it ever hears from that
/// writer is route 1's. A receiver that can only infer motion by watching a
/// count CHANGE has nothing to compare route 1 against — it banks it — and the
/// discovery quiet rule then closes the channel set inside the gap before route 2.
/// On a cold Go2 bus that gap is multi-seconds and there are 97 more routes
/// behind it.
///
/// The gap here is [`OVER_SETTLE_GAP`], longer than the whole
/// `DEFAULT_DISCOVERY_SETTLE_MS` cap and not merely longer than the quiet rule,
/// so "the channel set would have closed" needs no timing argument: the settle
/// cannot hold past its own ceiling.
///
/// # Why this arm re-runs its own SETUP
///
/// Every other arm in this file survives on the settle ALONE for its
/// first two seconds — the headline's gaps are 800 ms, the ceiling arm mints
/// every 200 ms — so a slow start costs them nothing. This is the ONE arm whose
/// success depends on the ingress hold having ARMED from route 1 with nothing
/// else covering it, and that gives it a setup race no assertion can win:
///
/// route 1 must be CREATED before the recorder's `DISCOVERY_SETTLE_MIN` floor
/// (500 ms, a production constant this arm must not move) expires, or the
/// channel set closes with no progress record ever heard, the hold never
/// engages, and everything after is `AppearedAfterBagCreation`.
///
/// The budget is 500 ms measured from bagd's drive-loop start, and route 1's own
/// mint is most of it: it is this process's FIRST `create_ingress_publisher`, so
/// it pays for the topic's iceoryx2 service AND the lazy open of the whole
/// ingress-build progress channel (service + publisher + republish thread).
/// MEASURED on a desk under ordinary load: **10-155 ms**, and a preempted run
/// stretches it without bound. Injecting a 300 ms delay before route 1
/// reproduces the failure 100 % of the time, with `channel_set_closed_after`
/// stamped at ~518 ms and `ingress_build: None` — the hold never armed.
///
/// So the arm ESTABLISHES its premise rather than assuming it
/// ([`await_tap_before_bag`]) and, when it loses that race, re-runs its own
/// setup instead of reporting a defect in the hold. The retry is scoped to the
/// PREMISE and nothing else: every assertion below panics on the spot, so an
/// oracle failure is never retried and never masked.
///
/// The mutation kill is unaffected — strengthened, if anything. After the
/// rendezvous the settle cap has at most `DEFAULT_DISCOVERY_SETTLE_MS` left and
/// [`OVER_SETTLE_GAP`] exceeds the whole of it, so with the hold removed route 2
/// still arrives after a closed channel set.
///
/// **Product residual this arm cannot cover, recorded because the measurement
/// found it:** the same 500 ms window exists on the real `graph run --record`
/// path. bagd is armed before the graph reaches step 0, and its hold can
/// only arm once route 1 EXISTS — so a bridge whose FIRST route takes longer
/// than the settle floor to open (a cold DDS bus is measured in seconds) closes
/// its channel set before there is anything to hold for. The hold covers the
/// gaps BETWEEN routes; the gap BEFORE route 1 is still the settle's alone.
#[test]
#[serial_test::serial]
fn the_very_first_route_arms_the_hold_so_a_cold_boot_does_not_lose_route_two() {
    for attempt in 1..=COLD_BOOT_SETUP_ATTEMPTS {
        if cold_boot_attempt(attempt) {
            return;
        }
    }
    panic!(
        "the cold-boot arm lost its SETUP race {COLD_BOOT_SETUP_ATTEMPTS} times: on every attempt \
         the recorder's channel set closed before route 1 was even discovered, so there was never \
         an ingress-build hold to test. That is a HARNESS liveness failure, not a hold \
         regression — this arm has only the `DISCOVERY_SETTLE_MIN` floor (500 ms) in which to \
         create route 1. Losing it {COLD_BOOT_SETUP_ATTEMPTS} times running means the desk could \
         not create one iceoryx2 service plus the ingress-build progress channel inside half a \
         second, on any attempt"
    );
}

/// How many times [`the_very_first_route_arms_the_hold_so_a_cold_boot_does_not_lose_route_two`]
/// re-runs its own setup after losing the race described there.
///
/// MEASURED failure rate of a single attempt on a loaded desk: ~1 in 8 (7 pass /
/// 1 fail over 8 in-order runs of this file, and the same 7/1 over 8 runs of the
/// arm alone — the two rates being indistinguishable is what proved this is the
/// arm's own setup race and not a predecessor's leftover state). Six attempts
/// takes that to ~1 in 2·10^5 while costing a healthy run nothing: a lost
/// attempt is detected at the rendezvous, in the time it takes the bag file to
/// appear, not at the 20 s oracle timeout.
const COLD_BOOT_SETUP_ATTEMPTS: usize = 6;

/// Wait until the recorder has TAPPED `topic`, or until the bag exists — the
/// PREMISE check the cold-boot arm needs before it starts its gap.
///
/// Returns `true` iff the tap appeared while the channel set was still open,
/// which is the fact the arm assumes and cannot otherwise establish. It is a
/// stronger statement than "the bag does not exist yet", and it is exactly the
/// one the arm needs, because a tap is attached by `plan_discovery` ONLY while
/// the bag has not been created — and the drive loop drains the ingress-build
/// progress channel BEFORE it applies a scan on the same pass, so a route that
/// got a tap is a route whose creation record the recorder had already heard.
///
/// The bag FILE is the losing condition rather than a wall, for the load-inversion
/// reason the module header states: a wall band tight enough to separate the two
/// is also tight enough for a loaded runner to invert. This asks the question
/// directly and the answer cannot be inverted by load — once the bag exists the
/// channel set is frozen and no amount of waiting re-opens it.
#[must_use]
fn await_tap_before_bag(mgr: &TransportManager, topic: &str, out: &std::path::Path) -> bool {
    let start = std::time::Instant::now();
    loop {
        // The TAP is asked first, and the order matters: if both became true
        // between two polls the premise still HELD — a tap that exists was
        // attached while the set was open, whatever happened afterwards.
        if mgr.topic_subscriber_count(topic) > 0 {
            return true;
        }
        if out.exists() {
            return false;
        }
        if start.elapsed() >= BAG_FILE_DEADLINE {
            return false;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// One attempt at the cold-boot arm. Returns `false` iff the SETUP race above
/// was lost — every other outcome asserts in place.
fn cold_boot_attempt(attempt: usize) -> bool {
    let mgr = make_manager(16);
    let declared = unique_topic("cold_declared");
    let base = unique_topic("cold_route");
    let out = unique_out("coldboot");
    let ready = unique_out("coldboot_ready");

    let mut declared_pub = publisher_with_provisioning(&mgr, &declared, 8, 16, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(
        Arc::clone(&mgr),
        cfg_for(out.clone(), &declared, ready.clone()),
        Arc::clone(&shutdown),
    );
    assert!(wait_for_file(&ready, Duration::from_secs(20)), "bagd ready");
    declared_pub
        .publish_raw(&build_frame(HASH, 0, 1, &oracle_body(&declared, 0)))
        .expect("declared publish");

    // Route ONE — the first this writer has ever created, and the first record
    // the recorder has ever heard from it.
    let first = format!("{base}/r1");
    let mut keep_alive = vec![mint_route(&mgr, &first, 0)];

    // THE PREMISE, established rather than assumed. Everything below
    // tests what the hold does across a gap; none of it means anything unless the
    // channel set was still open when route 1 arrived. If it was not, this run
    // never had a hold to exercise — tear it down and try again.
    if !await_tap_before_bag(&mgr, &first, &out) {
        let _ = finish(handle, &shutdown);
        cleanup(&out);
        let _ = std::fs::remove_file(&ready);
        eprintln!(
            "cold-boot attempt {attempt} lost its setup race — the recorder's channel \
             set closed before route 1 was discovered, so no ingress-build hold ever engaged. \
             Retrying (this is a harness race against the 500 ms settle floor, not a verdict)"
        );
        return false;
    }

    // A gap the settle provably cannot survive.
    std::thread::sleep(OVER_SETTLE_GAP);

    // Route TWO. On a cold bus this is where the other 97 live. Minted SILENT so
    // the frame oracle below is exact by construction: the only frame that can
    // ever exist on this topic is the one published once the tap is attached
    // (see `mint_route_silent` for the race a mint-time frame would leave open).
    let second = format!("{base}/r2");
    keep_alive.push(mint_route_silent(&mgr, &second));

    assert!(
        await_rescan_tap(&mgr, &second),
        "the recorder never tapped '{second}' — route 2 arrived {OVER_SETTLE_GAP:?} after route \
         1, and the channel set had already closed. Route 1 WAS tapped (the premise above holds), \
         so the hold armed and then let go inside the gap — the cold-boot shape the hold exists \
         to cover"
    );
    keep_alive
        .last_mut()
        .expect("routes")
        .publish_raw(&build_frame(
            HASH,
            LATE_SEQ,
            9_000,
            &oracle_body(&second, LATE_SEQ),
        ))
        .expect("late publish");

    assert!(
        wait_for_file(&out, INGRESS_BUILD_STALL_WINDOW + BAG_FILE_DEADLINE),
        "the hold must release on its own once the build stalls"
    );
    std::thread::sleep(MANY_RESCANS);
    let summary = finish(handle, &shutdown);

    let channels = user_channels(&out);
    for topic in [&first, &second] {
        assert!(
            channels.contains(topic),
            "'{topic}' is missing from the bag. A cold-booted robot's first recording must not \
             freeze its channel set on route 1. Channels: {channels:?}"
        );
    }
    assert!(
        !matches!(
            summary.record_coverage.untapped.get(&second),
            Some(UntappedReason::AppearedAfterBagCreation)
        ),
        "route 2 was REPORTED rather than recorded — the ingress-build defect verbatim"
    );
    assert_eq!(
        frames_for(&out, &second),
        vec![build_frame(
            HASH,
            LATE_SEQ,
            9_000,
            &oracle_body(&second, LATE_SEQ)
        )],
        "and it must carry data, not just a channel"
    );

    let ingress = summary
        .record_coverage
        .ingress_build
        .expect("the hold engaged on route 1, so the manifest must say so");
    assert_eq!(ingress.released, IngressBuildRelease::Stalled);

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
    true
}

// ===========================================================================
// A SETTLED PLANE — a first sighting is not motion
// ===========================================================================

/// A recorder started against an ALREADY-BUILT ingress plane is not held at all.
///
/// The case this protects is the common one: a robot whose bridge came up
/// minutes ago republishes its running total forever. Holding for it would make
/// EVERY recording on that robot — every `cerulion bag record`, every re-record
/// — wait out a whole stall window for a plane that finished long ago, and
/// stage (or lose) frames while it waited. Mutation-measured at `held_ms: 5298`
/// when that guard is removed.
///
/// # Why the WINDOW is scaled, not the wait
///
/// "Already built" is not a wall-clock fact, it is an EVIDENCE fact: the plane
/// is settled exactly when its last creation is older than the stall window. A
/// test cannot reasonably sleep out the shipped five seconds (and a plane built
/// 50 ms before the recorder armed genuinely might still be building — holding
/// for THAT is correct), so this arm shrinks the window and sleeps past it,
/// which reproduces the real relationship at 1/16th the cost.
///
/// The oracle is the manifest, not a wall: the hold must never ENGAGE, so the
/// block must be absent. That is deliberately stronger than "the bag appeared
/// quickly", which a fast machine would satisfy either way.
#[test]
#[serial_test::serial]
fn a_recorder_started_against_an_already_built_plane_is_not_held() {
    let mgr = make_manager(16);
    let declared = unique_topic("settled_declared");
    let base = unique_topic("settled_route");
    let out = unique_out("settled");
    let ready = unique_out("settled_ready");

    // The plane is built BEFORE the recorder exists — the settled-robot shape.
    let mut keep_alive: Vec<CerulionPublisher> = Vec::new();
    for topic in burst_topics(&base, 0) {
        keep_alive.push(mint_route(&mgr, &topic, 0));
    }
    // …and it finished long enough ago to be SETTLED by the window in force.
    std::thread::sleep(SETTLED_STALL_OVERRIDE * 4);

    let mut cfg = cfg_for(out.clone(), &declared, ready.clone());
    cfg.ingress_stall_window_override = Some(SETTLED_STALL_OVERRIDE);

    let mut declared_pub = publisher_with_provisioning(&mgr, &declared, 8, 16, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(Arc::clone(&mgr), cfg, Arc::clone(&shutdown));
    assert!(wait_for_file(&ready, Duration::from_secs(20)), "bagd ready");
    declared_pub
        .publish_raw(&build_frame(HASH, 0, 1, &oracle_body(&declared, 0)))
        .expect("declared publish");

    // Nothing new is minted. The ONLY records the recorder can hear are the
    // republish belt's re-sends of a total that is not moving.
    assert!(
        wait_for_file(&out, BAG_FILE_DEADLINE),
        "a settled plane must not hold the channel set at all"
    );
    std::thread::sleep(MANY_RESCANS);
    let summary = finish(handle, &shutdown);

    assert_eq!(
        summary.record_coverage.ingress_build, None,
        "the plane's last creation is older than the stall window, so every record the recorder \
         heard was STALE ON ARRIVAL. Stale evidence is NO CLAIM — not an instantly-stalled hold \
         — or every recording on a built robot would carry a hold it never paid"
    );
    // Anti-vacuity: the routes really are there and really were recorded, so the
    // `None` above is "nothing moved", not "nothing existed".
    let channels = user_channels(&out);
    for topic in burst_topics(&base, 0) {
        assert!(
            channels.contains(&topic),
            "the already-built routes must still be discovered and recorded: {channels:?}"
        );
    }

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ===========================================================================
// BOUNDED — the ceiling fires, loudly, and the recording proceeds
// ===========================================================================

/// A plane that never stops building is released at the ceiling, and the bag
/// says so.
///
/// A bag that is never created is worse than one missing late topics, so the
/// hold has an absolute ceiling. Reaching it means routes were STILL arriving
/// (the stalled arm is tested first), which is why this arm also pins that the
/// release is recorded as `Ceiling` rather than `Stalled` — the manifest's only
/// way of telling a reader the bag is expected to be incomplete.
#[test]
#[serial_test::serial]
fn a_never_ending_build_is_released_at_the_ceiling_and_the_manifest_says_so() {
    let mgr = make_manager(16);
    let declared = unique_topic("ceil_declared");
    let base = unique_topic("ceil_route");
    let out = unique_out("ceiling");
    let ready = unique_out("ceiling_ready");

    let mut cfg = cfg_for(out.clone(), &declared, ready.clone());
    // A ceiling a test can actually reach, and a stall window long enough that
    // the ORDINARY arm cannot fire first — otherwise this would pin nothing.
    cfg.ingress_ceiling_override = Some(Duration::from_millis(1500));
    // SHORT enough that the build below provably STALLS after the ceiling fired,
    // and long enough that it cannot fire DURING the build (mint cadence 200 ms).
    cfg.ingress_stall_window_override = Some(Duration::from_secs(3));

    let mut declared_pub = publisher_with_provisioning(&mgr, &declared, 8, 16, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(Arc::clone(&mgr), cfg, Arc::clone(&shutdown));
    assert!(wait_for_file(&ready, Duration::from_secs(20)), "bagd ready");
    declared_pub
        .publish_raw(&build_frame(HASH, 0, 1, &oracle_body(&declared, 0)))
        .expect("declared publish");

    // Keep minting routes past the ceiling, one every 200 ms.
    let mut keep_alive: Vec<CerulionPublisher> = Vec::new();
    for r in 0..12u32 {
        keep_alive.push(mint_route(&mgr, &format!("{base}/r{r}"), r));
        std::thread::sleep(Duration::from_millis(200));
    }
    assert!(
        wait_for_file(&out, BAG_FILE_DEADLINE),
        "the ceiling exists precisely so a never-ending build cannot hold a bag hostage — the \
         bag file never appeared"
    );
    // Now STOP minting and let a whole stall window pass. The verdict must not
    // change: a release is reported ONCE per recording, and the CEILING verdict
    // is the one that tells a reader this bag is expected to be incomplete. A
    // recorder that re-decided every pass would quietly relabel it `Stalled` the
    // moment the build went quiet — erasing the warning at exactly the point the
    // operator would look for it.
    std::thread::sleep(Duration::from_secs(4));
    let summary = finish(handle, &shutdown);

    let ingress = summary
        .record_coverage
        .ingress_build
        .expect("progress was observed, so the manifest must carry the block");
    assert_eq!(
        ingress.released,
        IngressBuildRelease::Ceiling,
        "routes were still arriving when the hold ended, so this bag is expected to be \
         incomplete and the manifest must say CEILING, not STALLED"
    );
    assert!(
        ingress.held_ms >= 1500,
        "the hold must have run to the ceiling, got {} ms",
        ingress.held_ms
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ===========================================================================
// SHUTDOWN — a signal never waits out the hold
// ===========================================================================

/// Ctrl-C during an ingress build closes the channel set at once, and the
/// manifest records the interruption.
///
/// The same rule the settle follows: a SIGINT must never wait out a
/// discovery window. Without the `Shutdown` arm a short bag would look
/// unexplained; with it, the manifest says the build was still running when the
/// operator stopped.
#[test]
#[serial_test::serial]
fn a_shutdown_during_the_build_closes_the_channel_set_at_once_and_is_recorded() {
    let mgr = make_manager(16);
    let declared = unique_topic("sig_declared");
    let base = unique_topic("sig_route");
    let out = unique_out("shutdown");
    let ready = unique_out("shutdown_ready");

    let mut cfg = cfg_for(out.clone(), &declared, ready.clone());
    // Long enough that the hold is unambiguously still open when we signal, and
    // long enough to be the DISCRIMINATOR below.
    cfg.ingress_stall_window_override = Some(SHUTDOWN_STALL_OVERRIDE);

    let mut declared_pub = publisher_with_provisioning(&mgr, &declared, 8, 16, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(Arc::clone(&mgr), cfg, Arc::clone(&shutdown));
    assert!(wait_for_file(&ready, Duration::from_secs(20)), "bagd ready");
    declared_pub
        .publish_raw(&build_frame(HASH, 0, 1, &oracle_body(&declared, 0)))
        .expect("declared publish");

    let mut keep_alive: Vec<CerulionPublisher> = Vec::new();
    for topic in burst_topics(&base, 0) {
        keep_alive.push(mint_route(&mgr, &topic, 0));
    }
    // The bag must NOT exist yet — the hold is open with a 60 s stall window.
    settle();
    assert!(
        !out.exists(),
        "the hold should still be open here; if the bag already exists this arm is not \
         signalling DURING a build"
    );

    let summary = finish(handle, &shutdown);

    let ingress = summary
        .record_coverage
        .ingress_build
        .expect("the hold was open when the signal arrived, so it must be reported");
    assert_eq!(
        ingress.released,
        IngressBuildRelease::Shutdown,
        "a signal closes the channel set immediately; it must not be reported as a stall"
    );

    // THE "immediately" half, and it needs its own assertion: reporting the
    // interruption is not the same as ACTING on it. A recorder that reported
    // `Shutdown` but kept holding would still be signalled, still finalize, and
    // still carry this verdict — it would just take a whole stall window to do
    // it, which in production is up to `INGRESS_BUILD_HOLD_CEILING`. An operator
    // pressing Ctrl-C would sit there watching nothing happen.
    //
    // The oracle is the recorder's OWN measured drive span, not a harness wall,
    // and the separation is ~30x: this arm signals a couple of seconds in, while
    // a recorder that waited out the hold could not exit before
    // SHUTDOWN_STALL_OVERRIDE. Load can only lengthen the pre-signal phase, and
    // it would have to lengthen it thirty-fold to reach the threshold.
    assert!(
        summary.drive_span < SHUTDOWN_STALL_OVERRIDE,
        "the drive loop ran {:?}, at least a whole stall window ({SHUTDOWN_STALL_OVERRIDE:?}) — \
         the signal did not close the channel set at once, it waited the hold out",
        summary.drive_span
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ===========================================================================
// A BOUND RUN ENDING is not an operator interrupting
// ===========================================================================

/// A bound recorder whose RUN ends mid-build records `RunEnded`, never
/// `Shutdown`.
///
/// bagd's `shutting` flag is the UNION of two very different events —
/// `shutdown.load() || run_ended.is_some()` — and only the first is an operator
/// interrupting anything. The second is the NORMAL end of a
/// `graph run --record`: the graph exits, the bound recorder finalizes with it.
///
/// Collapsing them would stamp "this recording was asked to stop while a
/// process was still opening runtime ingress routes" into the manifest of every
/// bound recording whose graph simply exited before its ingress plane finished
/// coming up — a false claim about WHO DID WHAT, in the one artifact that
/// outlives the run. The load-bearing detail is that the shutdown flag is NEVER
/// SET here: if the recorder stopped, its run is why.
#[test]
#[serial_test::serial]
fn a_bound_run_ending_mid_build_is_recorded_as_such_not_as_an_interruption() {
    let mgr = make_manager(16);
    let declared = unique_topic("runend_declared");
    let base = unique_topic("runend_route");
    let out = unique_out("runend");
    let ready = unique_out("runend_ready");
    let run_id = 0x1243_0001_0000_0000_0000_0000_0000_0001;

    let run = RunHandle::publish_on_config(
        &mgr.iox_config(),
        RunRecord {
            run_id,
            graph_name: "go2_attach".to_string(),
            run_dir: std::env::temp_dir().to_string_lossy().into_owned(),
            supervisor_pid: std::process::id(),
            run_started_at_ns: 1,
            state: RunState::Live,
        },
    )
    .expect("publish run");

    let mut cfg = cfg_for(out.clone(), &declared, ready.clone());
    // Long enough that the hold is unambiguously still OPEN when the run ends.
    cfg.ingress_stall_window_override = Some(SHUTDOWN_STALL_OVERRIDE);
    cfg.run_binding = Some(RunBinding {
        run_id,
        vanish_grace: Some(Duration::from_millis(1_500)),
    });

    let mut declared_pub = publisher_with_provisioning(&mgr, &declared, 8, 16, 4096);
    // NEVER set. If the recorder returns, it is because its RUN ended.
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(Arc::clone(&mgr), cfg, Arc::clone(&shutdown));
    assert!(wait_for_file(&ready, Duration::from_secs(20)), "bagd ready");
    declared_pub
        .publish_raw(&build_frame(HASH, 0, 1, &oracle_body(&declared, 0)))
        .expect("declared publish");

    let mut keep_alive: Vec<CerulionPublisher> = Vec::new();
    for topic in burst_topics(&base, 0) {
        keep_alive.push(mint_route(&mgr, &topic, 0));
    }
    settle();
    assert!(
        !out.exists(),
        "PRECONDITION: the hold must still be open when the run ends, or this arm is not \
         testing a run that ended MID-BUILD"
    );

    // The run ends. Nothing signals the recorder.
    drop(run);
    let summary = handle.join().expect("bagd thread").expect("clean finalize");

    assert!(
        !shutdown.load(Ordering::Relaxed),
        "nothing ever signalled this recorder — it must have stopped because its RUN did"
    );
    assert!(
        summary.run_ended.is_some(),
        "PRECONDITION: the recorder must have observed its run ending"
    );
    let ingress = summary
        .record_coverage
        .ingress_build
        .expect("the hold was open when the run ended, so it must be reported");
    assert_eq!(
        ingress.released,
        IngressBuildRelease::RunEnded,
        "the RUN ended; no operator interrupted anything. Reporting Shutdown here would put a \
         false claim about who stopped this recording into a bag that outlives the run"
    );
    // It still closed while routes were arriving, so the bag is not complete.
    assert!(
        summary.record_coverage.is_incomplete(),
        "the channel set closed mid-build — the bag cannot claim coverage"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ===========================================================================
// INVISIBILITY — the progress channel is not data
// ===========================================================================

/// The progress channel itself is never tapped, recorded, or reported.
///
/// `/__cerulion/ingress_build` carries no `/data` suffix precisely so the
/// recorder's live-service enumeration cannot see it. A regression here would
/// have the recorder tapping its own control plane and writing it into the bag
/// as robot data.
#[test]
#[serial_test::serial]
fn the_progress_channel_is_never_tapped_recorded_or_reported() {
    let mgr = make_manager(16);
    let declared = unique_topic("inv_declared");
    let base = unique_topic("inv_route");
    let out = unique_out("invisible");
    let ready = unique_out("invisible_ready");

    let mut declared_pub = publisher_with_provisioning(&mgr, &declared, 8, 16, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(
        Arc::clone(&mgr),
        cfg_for(out.clone(), &declared, ready.clone()),
        Arc::clone(&shutdown),
    );
    assert!(wait_for_file(&ready, Duration::from_secs(20)), "bagd ready");
    declared_pub
        .publish_raw(&build_frame(HASH, 0, 1, &oracle_body(&declared, 0)))
        .expect("declared publish");

    let mut keep_alive: Vec<CerulionPublisher> = Vec::new();
    for topic in burst_topics(&base, 0) {
        keep_alive.push(mint_route(&mgr, &topic, 0));
    }
    assert!(wait_for_file(
        &out,
        INGRESS_BUILD_STALL_WINDOW + BAG_FILE_DEADLINE
    ));
    settle();
    let summary = finish(handle, &shutdown);

    let service = cerulion_core::transport::ingress_build::INGRESS_BUILD_SERVICE_NAME;
    let channels = user_channels(&out);
    assert!(
        !channels.iter().any(|c| c.contains("ingress_build")),
        "the progress control channel must never become a bag channel: {channels:?}"
    );
    assert!(
        !summary
            .record_coverage
            .untapped
            .keys()
            .any(|k| k.contains("ingress_build")),
        "it must not be REPORTED either — it is not a robot topic: {:?}",
        summary.record_coverage.untapped
    );
    assert!(
        !service.ends_with("/data"),
        "the invisibility above is structural, not incidental"
    );
    // Anti-vacuity: the enumeration really did run and really did find things.
    assert!(summary.record_coverage.enumerated);
    assert!(
        !summary.record_coverage.tapped.is_empty(),
        "if nothing was tapped, the absence assertions above prove nothing"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}
