// SPDX-License-Identifier: AGPL-3.0-only
//! Live-discovery end-to-end: the recorder records what EXISTS, and SAYS what it does
//! not.
//!
//! Every test builds an ISOLATED per-instance iceoryx2 transport
//! ([`common::make_manager`]) so the live-topic enumeration under test sees
//! exactly the topics this test created and nothing else — which is what makes
//! an exact-set assertion on discovery meaningful at all. Oracles are
//! HAND-BUILT wire frames and hand-written expected sets, never a self-compare.
//!
//! The two halves of the live-discovery contract get separate pins:
//!
//! * COVERAGE — a producer the declared tap set never named is nevertheless
//!   recorded, whether it was live at arm time or appeared later.
//! * DISCLOSURE — a live producer that is NOT recorded is named, with its reason,
//!   in the bag's own `record_coverage.json`. `frames_lost = 0` must never
//!   again be the whole story.

mod common;

use std::collections::BTreeSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cerulion_bag::BagReader;
use cerulion_bagd::{
    run_bagd, BagdConfig, BagdError, BagdSummary, LossCountingBasis, RecordCoverage, RecordHealth,
    TapSource, TapSpec, UntappedReason, DISCOVERY_RESCAN_INTERVAL, RECORD_COVERAGE_ATTACHMENT,
    RECORD_HEALTH_ATTACHMENT,
};
use cerulion_core::transport::TransportManager;

use common::*;

/// An arbitrary, stable schema hash for the hand-built frames.
const HASH: u64 = 0x0942_0942_0942_0942;
/// Comfortably longer than one enumeration cadence, so a scan provably ran in
/// the window.
const RESCAN_SETTLE: Duration =
    Duration::from_millis((DISCOVERY_RESCAN_INTERVAL.as_millis() as u64) * 3 + 200);

/// How often [`await_discovered_tap`] asks. Mirrors [`FILE_POLL_INTERVAL`].
const TAP_ATTACH_POLL: Duration = Duration::from_millis(5);
/// A LIVENESS ceiling for [`await_discovered_tap`], deliberately stated in seconds
/// rather than in units of [`DISCOVERY_RESCAN_INTERVAL`]: load can delay the
/// recorder's drive loop, and a bound that scales with the thing under test is
/// the wall-in-its-own-units mistake this helper exists to remove.
const TAP_ATTACH_DEADLINE: Duration = Duration::from_secs(20);
/// Liveness bound for the bag file appearing once the channel set closes.
const BAG_CREATED_DEADLINE: Duration = Duration::from_secs(20);

/// Block until bagd's DISCOVERY has ATTACHED a tap to `topic`.
///
/// # Why this is a condition and not a sleep
///
/// A producer created after the ready-file is discovered by a worker thread
/// woken by the topic's own service file appearing, so WHEN the tap attaches is
/// the machine's business and not a cadence anyone can sleep out. Sleeping a
/// fixed interval and then publishing is a RACE, and it is the
/// one that loses in the direction that destroys data: a data-only tap requests
/// no late-joiner history, so a frame committed BEFORE the tap attaches lands in
/// no queue at all and can never be recovered. It fired on main CI (run
/// 30688694294) with the drive loop starved ~1.24 s — the late tap armed ~15 ms
/// AFTER the test had published its four frames, and the bag finalized with the
/// late channel on a placeholder schema and zero frames.
///
/// So the rendezvous is the STATE the next publish depends on. The tap is a
/// subscriber port on the topic's own data service, which
/// [`TransportManager::topic_subscriber_count`] — a production accessor, not a
/// test hook — reads out of iceoryx2's dynamic config. It opens the service
/// without creating a port, so asking costs the recorder no budget.
///
/// `>= 1` is exact rather than approximate on the arms that call this: each
/// creates the topic's ONLY publisher and attaches no subscriber of its own, so
/// the only port that can appear is the recorder's tap.
///
/// The PORT is the right boundary, not bagd's own tap vector: `open_tap` creates
/// the subscriber and `apply_discovery` pushes the `TapState` microseconds
/// later on the same thread, and a frame published in between is QUEUED by
/// iceoryx2 into a port that already exists — it is drained on the next pass,
/// not lost.
fn await_discovered_tap(mgr: &TransportManager, topic: &str) {
    let start = Instant::now();
    loop {
        let subscribers = mgr.topic_subscriber_count(topic);
        if subscribers >= 1 {
            return;
        }
        assert!(
            start.elapsed() < TAP_ATTACH_DEADLINE,
            "bagd's discovery never attached a tap to '{topic}': its data service still \
             reports 0 subscriber ports after {:?} (discovery is woken by the service directory \
             changing; its fallback cadence is {DISCOVERY_RESCAN_INTERVAL:?}). Every frame \
             published from here would land in no \
             queue at all, so this is a FAILURE of the recorder or of this harness — not a \
             frame-loss bug in the assertions below",
            start.elapsed()
        );
        std::thread::sleep(TAP_ATTACH_POLL);
    }
}

fn spawn_bagd(
    mgr: Arc<TransportManager>,
    cfg: BagdConfig,
    shutdown: Arc<AtomicBool>,
) -> JoinHandle<Result<BagdSummary, BagdError>> {
    std::thread::spawn(move || run_bagd(mgr, cfg, shutdown))
}

/// [`spawn_bagd`], with the test's `tracing` span CARRIED onto the recorder
/// thread.
///
/// `tracing-test` scopes its capture to a span entered on the TEST thread, and a
/// thread spawned inside the test does not inherit it — so by default a recorder
/// on its own thread emits lines `logs_contain` can never see. Carrying
/// `Span::current()` across and entering it in the worker is what lets an arm
/// keep BOTH halves it needs: the recorder on its own thread (so the test body
/// stays free to drive the stimulus and then OBSERVE), and its lines inside the
/// capture. Only the arms that RENDEZVOUS on a recorder line use it — everything
/// else keeps the plain spawn, so no arm acquires a log dependency it does not
/// need.
fn spawn_bagd_traced(
    mgr: Arc<TransportManager>,
    cfg: BagdConfig,
    shutdown: Arc<AtomicBool>,
) -> JoinHandle<Result<BagdSummary, BagdError>> {
    let span = tracing::Span::current();
    std::thread::spawn(move || {
        let _entered = span.enter();
        run_bagd(mgr, cfg, shutdown)
    })
}

/// The fragment that identifies bagd's CHANNEL-SET-CLOSED line.
///
/// `ensure_writer` emits it (`CHANNEL_SET_CLOSED_DISCOVERY_ON_MSG`, at `info!`)
/// in the same breath as it stamps `BagdSummary::channel_set_closed_after` —
/// one statement before `spawn_writer_thread`, which is what makes
/// `Recorder::bag_exists` true. `apply_discovery` runs at the TOP of a drive
/// pass and `ensure_writer` further down the SAME pass, so the next discovery
/// decision after this line is taken a full pass later, with the writer handle
/// already set. Once it is out, EVERY subsequent decision sees
/// `bag_created == true` — which is exactly the precondition the arms below
/// need.
const CHANNEL_SET_CLOSED_MARKER: &str = "bagd CHANNEL SET CLOSED";

/// The fragment that identifies bagd's POST-CREATION verdict line.
///
/// `apply_discovery` emits it at WARN, exactly once per topic, at the instant it
/// mints [`UntappedReason::AppearedAfterBagCreation`] — and the ledger insert the
/// coverage manifest is built from is the very next statement on the same
/// thread, inside the same uninterruptible call. So the line appearing IS the
/// state the manifest assertions depend on.
const POST_CREATION_VERDICT_MARKER: &str = "appeared after the bag was created";

/// How often [`await_post_creation_verdict`] asks.
///
/// Coarser than [`TAP_ATTACH_POLL`] on purpose: the capture probe COPIES the
/// whole process-global buffer on every call, and that buffer accumulates for the
/// lifetime of the test binary, so a 5 ms poll would spend the wait cloning it
/// and holding the writer's mutex against the very recorder it is waiting for.
const VERDICT_POLL: Duration = Duration::from_millis(25);

/// PURE: does the capture hold a SINGLE line that is bagd's post-creation
/// verdict FOR `topic`?
///
/// # Why one line, and not two `logs_contain` calls
///
/// The first cut of this rendezvous asked `logs_contain(MARKER) &&
/// logs_contain(topic)`, which searches the cumulative capture INDEPENDENTLY:
/// it is satisfied when the marker was logged for SOME topic and the name under
/// test appears in ANY unrelated line — and unrelated lines naming it are
/// guaranteed, since `cerulion_core`'s publisher logs `topic=` on create and on
/// drop. So the wait could return before the recorder had minted a verdict for
/// THIS producer, which is the exact state the arms then assert: the rendezvous
/// would have been sound only by the accident of each arm having one
/// post-creation producer, and would have silently stopped discriminating the
/// moment an arm grew a second. A rendezvous that can be satisfied by the wrong
/// observation is not a rendezvous.
///
/// The verdict is ONE event, so the predicate is one line. The level token is
/// matched as well as the text, per the repo's per-line predicate discipline: a
/// reporter demoted to `info!` would keep every word and lose the point.
fn verdict_line_present(lines: &[&str], topic: &str) -> bool {
    lines.iter().any(|line| {
        line.split_whitespace().any(|t| t == "WARN")
            && line.contains(POST_CREATION_VERDICT_MARKER)
            && line.contains(topic)
    })
}

/// Block until bagd has CLOSED its channel set — the precondition every
/// "appeared after bag creation" arm depends on.
///
/// # Why this is a condition and not a sleep
///
/// These arms create a producer that must land AFTER the bag exists, and they
/// used to establish that with a fixed sleep. That is elapsed time standing in
/// for a state, and the state is load-dependent: creation waits for the LATER of
/// the settle hold releasing (`DISCOVERY_SETTLE_MIN` = 500 ms with nothing new
/// discovered, capped at 2 s) and every declared tap learning its schema.
/// MEASURED on an idle desk, the disclosure arm closed its channel set at
/// 506-522 ms and created its late producer at 607-610 ms - a margin under
/// 100 ms.
///
/// When the margin is lost the producer appears while the set is still OPEN, so
/// discovery TAPS it - correct bagd behaviour, and the exact inverse of the
/// arm's oracle. It is also self-reinforcing: a tap that is ADDED restarts the
/// quiet window, which re-extends the hold, so the arm cannot recover by
/// waiting. That is what fired on main CI (run 31659934041) after the first
/// fix landed — reported by the verdict rendezvous as a 20 s timeout
/// with the topic TAPPED.
///
/// A single `logs_contain` is sound here, unlike the per-topic verdict (see
/// [`verdict_line_present`]): this is ONE global event per recording, with
/// nothing to correlate against a second fact. The capture is already scoped to
/// this test's span, and each arm runs one recorder.
fn await_channel_set_closed(logs_contain: fn(&str) -> bool) {
    let start = Instant::now();
    while !logs_contain(CHANNEL_SET_CLOSED_MARKER) {
        assert!(
            start.elapsed() < TAP_ATTACH_DEADLINE,
            "bagd never closed its channel set after {:?}: no `{CHANNEL_SET_CLOSED_MARKER}` line. \
             Creation waits for the LATER of the settle hold releasing and every declared tap \
             learning its schema, so either a declared tap never received a frame or the hold \
             never released — and every assertion below about a producer 'appearing after bag \
             creation' would otherwise be tested against a channel set that was still OPEN",
            start.elapsed()
        );
        std::thread::sleep(VERDICT_POLL);
    }
}

/// Build the single-event rendezvous predicate for `$topic`.
///
/// A macro rather than a function because it has to reach the `logs_assert` that
/// `#[traced_test]` INJECTS into each test body — it is the only public route to
/// the capture as LINES (`logs_contain` collapses it to one substring test), and
/// a free function has no path to it. The closure hands those lines to
/// [`verdict_line_present`] and REPORTS the answer instead of asserting it,
/// which is what makes one per-line predicate usable as a wait condition.
macro_rules! verdict_probe {
    ($topic:expr) => {{
        let probe = $topic.to_string();
        move || {
            let seen = std::cell::Cell::new(false);
            logs_assert(|lines: &[&str]| {
                seen.set(verdict_line_present(lines, &probe));
                Ok(())
            });
            seen.get()
        }
    }};
}

/// Block until bagd has MINTED the `appeared_after_bag_creation` verdict for
/// `topic`: the `await_discovered_tap` rendezvous, applied to an OBSERVATION rather
/// than to a tap attach.
///
/// # Why this is a condition and not a sleep
///
/// A producer created after the bag exists can never be TAPPED (MCAP channels
/// are frozen at creation), so `await_discovered_tap`'s port-count rendezvous has
/// nothing to watch: the recorder's whole reaction is a ledger entry. Sleeping a
/// fixed multiple of [`DISCOVERY_RESCAN_INTERVAL`] and then finalizing is
/// therefore a bet that a loaded runner completes one scanner cadence AND one
/// drive-loop apply inside the window — and it is a bet that loses in the
/// direction that reads as a product bug: the manifest comes back with the topic
/// simply absent, which is indistinguishable from the recorder having decided
/// not to report it. It fired three times in one day on main CI,
/// `left: None / right: Some(AppearedAfterBagCreation)`.
///
/// So the rendezvous is the STATE the assertion depends on. The scan runs on the
/// discovery worker thread and is applied by the drive loop, neither of which
/// exposes a counter to a test; the WARN is emitted at the one call site that
/// mints the verdict, so it is the observable. The ceiling is a LIVENESS
/// backstop stated in seconds, never the property under test.
///
/// `minted` is the SINGLE-EVENT predicate — build it with [`verdict_probe`],
/// which requires the marker and the topic on ONE line. See
/// [`verdict_line_present`] for why a two-substring search over the whole
/// capture is not good enough to wait on.
fn await_post_creation_verdict(
    mgr: &TransportManager,
    topic: &str,
    mut minted: impl FnMut() -> bool,
) {
    let start = Instant::now();
    loop {
        if minted() {
            return;
        }
        if start.elapsed() >= TAP_ATTACH_DEADLINE {
            // Two very different failures reach this line, and a timeout that
            // did not say which would turn an inverted oracle into "flaky".
            let diagnosis = if mgr.topic_subscriber_count(topic) == 0 {
                "its data service has NO subscriber port, so the recorder did not tap it either — \
                 discovery never observed it at all, which is a starved recorder or a broken \
                 harness"
            } else {
                // Sound as far as it goes: `plan_discovery` opens a tap ONLY on
                // the `!bag_created` branch, so a subscriber port really does
                // prove the set was open when discovery saw this topic. The
                // mistaken reading here was "real
                // regression rather than a timing miss", which sent a reader
                // hunting a bagd bug for what was, on main CI run 31659934041,
                // this arm's own precondition losing a ~90 ms race against a
                // load-stretched settle window. With `await_channel_set_closed`
                // now gating the producer's creation that benign reading is
                // excluded, so this state is genuinely suspicious — but the
                // wording states the FACT and leaves the verdict open.
                "its data service HAS a subscriber port, so the recorder TAPPED it, which it does \
                 only while the channel set is still OPEN. Since this arm waits for the \
                 channel-set-closed breadcrumb before creating the producer, the set was reopened \
                 or the producer was created too early — check the ordering before blaming bagd"
            };
            panic!(
                "bagd never minted the post-creation verdict for '{topic}' after {:?} (the \
                 enumeration fallback cadence is {DISCOVERY_RESCAN_INTERVAL:?}): {diagnosis}",
                start.elapsed()
            );
        }
        std::thread::sleep(VERDICT_POLL);
    }
}

/// Fast cadences, status OFF, discovery ON (the `graph run --record` default).
fn discovery_cfg(
    out: std::path::PathBuf,
    taps: Vec<TapSpec>,
    ready: std::path::PathBuf,
    schema_wait: Duration,
) -> BagdConfig {
    let mut cfg = BagdConfig::new(out, taps);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.ready_file = Some(ready);
    cfg.status_period = None;
    cfg.schema_wait = schema_wait;
    cfg.discover_live = true;
    cfg
}

/// The hand oracle for topic `t`, frame `i`: a distinct, reconstructible body.
fn oracle_body(tag: &str, i: u32) -> Vec<u8> {
    format!("{tag}-{i}").into_bytes()
}

/// A producer whose service is provisioned with room to spare.
///
/// The DEFAULT provisioning gives a tap `max_borrowed_samples = 2`, hence a
/// pre-writer held budget of ONE. A tap at that budget WAITS rather than
/// evicting drop-oldest, so the loss boundary is the topic's own queue depth,
/// but a DEFAULT queue is shallow, and an exact frame-count oracle here must
/// measure DISCOVERY, not how deep the producer happened to be provisioned.
fn producer(
    mgr: &TransportManager,
    topic: &str,
) -> cerulion_core::transport::publisher::CerulionPublisher {
    publisher_with_provisioning(mgr, topic, 8, 16, 4096)
}

fn publish_oracle(
    pubr: &mut cerulion_core::transport::publisher::CerulionPublisher,
    tag: &str,
    n: u32,
) {
    for i in 0..n {
        let frame = build_frame(HASH, i, 1_000 + i as u64, &oracle_body(tag, i));
        pubr.publish_raw(&frame).expect("publish_raw");
        settle();
    }
}

/// Read + parse the `record_coverage.json` attachment from a FINALIZED bag —
/// the machine-readable half of the disclosure contract.
fn read_coverage(out: &std::path::Path) -> RecordCoverage {
    let reader = BagReader::open(out).expect("open bag");
    let att = reader
        .attachment(RECORD_COVERAGE_ATTACHMENT)
        .expect("read attachments")
        .expect("record_coverage.json is present in EVERY finalized bag");
    assert_eq!(att.media_type, "application/json");
    let cov: RecordCoverage =
        serde_json::from_slice(&att.data).expect("record_coverage.json parses");
    // The schema marker exists so a reader can trust the shape; `RecordCoverage`
    // also derives Default (version 0), which is a real fixture in this repo, so
    // an unasserted version could let the default ship silently.
    assert_eq!(
        cov.version,
        cerulion_bagd::RECORD_COVERAGE_VERSION,
        "the coverage attachment must carry the current schema version"
    );
    cov
}

/// Read + parse the `record_health.json` attachment from a FINALIZED bag — the
/// document that carries the per-topic loss numbers and what
/// each of them is able to SEE.
fn read_health(out: &std::path::Path) -> RecordHealth {
    let reader = BagReader::open(out).expect("open bag");
    let att = reader
        .attachment(RECORD_HEALTH_ATTACHMENT)
        .expect("read attachments")
        .expect("record_health.json is present in EVERY finalized bag");
    serde_json::from_slice(&att.data).expect("record_health.json parses")
}

/// Channel topics of a finalized bag, excluding the reserved `__cerulion/*`.
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

/// Recovered payloads for `topic`, in file order.
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

/// Drive a recorder to a clean finalize and hand back its summary.
/// Wait until the recorder has CREATED the bag, which is the event every
/// "publish, then settle, then finish" site below was betting on with a fixed
/// `sleep(300)`.
///
/// The bag file appears when the channel set closes — i.e. once every tap has
/// learned its schema from a frame the drive loop actually observed. That is the
/// premise of the channel/frame oracles in these arms: a `finish()` that lands
/// before the recorder ever saw a frame would create the bag at finalization with
/// nothing learned. Waiting for the file asserts the premise instead of assuming
/// a 300 ms pass was enough, and load can only DELAY it (`await_condition`'s own
/// note in `common/mod.rs` makes the same argument).
fn await_bag_created(out: &std::path::Path) {
    assert!(
        wait_for_file(out, BAG_CREATED_DEADLINE),
        "the recorder never created {} — every channel and frame oracle below rests \
         on the channel set having closed, so this is a recorder/harness failure, not \
         a frame-loss bug in the assertions",
        out.display()
    );
}

fn finish(
    handle: JoinHandle<Result<BagdSummary, BagdError>>,
    shutdown: &Arc<AtomicBool>,
) -> BagdSummary {
    shutdown.store(true, Ordering::Relaxed);
    handle.join().expect("bagd thread").expect("clean finalize")
}

// ===========================================================================
// COVERAGE — record what exists
// ===========================================================================

/// THE headline: a live producer the declared tap set never named is recorded.
///
/// This is the `ros2 attach` robot's shape in miniature. `declared` stands for the attach
/// graph's four typed ports; `undeclared` stands for one of the bridge's ~71
/// dynamically-created ingress publishers — a real, live, single-writer SHM
/// producer that no graph YAML mentions. Without discovery the bag carries only
/// `declared`.
#[test]
#[serial_test::serial]
fn an_undeclared_live_producer_is_discovered_and_its_frames_land_in_the_bag() {
    let mgr = make_manager(16);
    let declared = unique_topic("declared");
    let undeclared = unique_topic("undeclared");
    let out = unique_out("arm");
    let ready = unique_out("arm_ready");

    // BOTH producers exist before the recorder arms (open-only taps require the
    // service; the undeclared one is simply never named to bagd).
    let mut declared_pub = producer(&mgr, &declared);
    let mut undeclared_pub = producer(&mgr, &undeclared);

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::attach(&declared)],
        ready.clone(),
        Duration::from_millis(3000),
    );
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_oracle(&mut declared_pub, "d", 3);
    publish_oracle(&mut undeclared_pub, "u", 3);
    await_bag_created(&out);
    let summary = finish(handle, &shutdown);

    // (1) The undeclared topic has a CHANNEL — it was tapped, not merely noted.
    assert_eq!(
        user_channels(&out),
        BTreeSet::from([declared.clone(), undeclared.clone()]),
        "the discovered producer must be a real channel in the bag"
    );

    // (2) Its FRAMES are in the bag, byte-identical to the hand oracle.
    let expected: Vec<Vec<u8>> = (0..3)
        .map(|i| build_frame(HASH, i, 1_000 + i as u64, &oracle_body("u", i)))
        .collect();
    assert_eq!(
        frames_for(&out, &undeclared),
        expected,
        "the discovered topic's frames must be recorded verbatim"
    );

    // (3) The manifest names it, attributes it to DISCOVERY, and does not claim
    //     it arrived late (it was live before the recorder armed).
    let coverage = read_coverage(&out);
    assert!(coverage.enumerated);
    let entry = coverage
        .tapped
        .get(&undeclared)
        .expect("the discovered topic is named in the coverage manifest");
    assert_eq!(entry.source, TapSource::Discovered);
    assert_eq!(entry.frames_recorded, 3);
    assert!(
        !entry.attached_late,
        "a producer live at ARM time is fully covered — claiming otherwise would \
         understate the recording"
    );
    assert_eq!(
        coverage
            .tapped
            .get(&declared)
            .expect("declared tapped")
            .source,
        TapSource::Declared,
        "an explicitly-named tap must not be re-attributed to discovery"
    );
    // (4) Nothing live went unrecorded.
    assert_eq!(coverage.gap_count(), 0);
    assert_eq!(
        summary.record_coverage, coverage,
        "summary == bag attachment"
    );
}

/// An ARMED run's head-coverage claim is scoped to the taps the
/// arm-ordering guarantee actually covered.
///
/// `--armed-before-producers` is a claim about the DECLARED set: `graph run
/// --record` holds the graph at step 0 until the recorder is armed. Live
/// discovery — ON for every such run — then attaches taps to producers that were
/// ALREADY RUNNING, and the document-wide token gave those rows a
/// `prefix_proven` basis their heads were never inside, so a discovered topic's
/// `frames_lost: 0` read as PROVEN.
///
/// Both halves are asserted in ONE body because the contrast IS the claim: a row
/// basis that answered `prefix_invisible` for everything would satisfy the
/// discovered arm alone, and a `prefix_proven`-for-everything would
/// satisfy the declared arm alone.
#[test]
#[serial_test::serial]
fn an_armed_runs_head_coverage_is_claimed_per_tap_not_per_document() {
    let mgr = make_manager(16);
    let declared = unique_topic("armed_declared");
    let discovered = unique_topic("armed_discovered");
    let out = unique_out("basis");
    let ready = unique_out("basis_ready");

    let mut declared_pub = producer(&mgr, &declared);
    let mut discovered_pub = producer(&mgr, &discovered);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::attach(&declared)],
        ready.clone(),
        Duration::from_millis(3000),
    );
    // The `graph run --record` guarantee, which must not be
    // read document-wide.
    cfg.armed_before_producers = true;
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_oracle(&mut declared_pub, "d", 3);
    publish_oracle(&mut discovered_pub, "u", 3);
    await_bag_created(&out);
    let summary = finish(handle, &shutdown);

    let health = read_health(&out);
    // Precondition: both rows exist, and the manifest agrees on which is which.
    let coverage = read_coverage(&out);
    assert_eq!(
        coverage
            .tapped
            .get(&discovered)
            .expect("the discovered topic is tapped")
            .source,
        TapSource::Discovered,
        "fixture precondition: this row really did arrive by discovery"
    );

    assert_eq!(
        health
            .topics
            .get(&declared)
            .expect("declared row")
            .loss_counting_basis,
        Some(LossCountingBasis::PrefixProven),
        "the guarantee covers the DECLARED tap, so its head loss IS accounted"
    );
    assert_eq!(
        health
            .topics
            .get(&discovered)
            .expect("discovered row")
            .loss_counting_basis,
        Some(LossCountingBasis::PrefixInvisible),
        "…and it never covered a tap discovery attached to an already-running producer"
    );
    // …and the DOCUMENT-wide token is the FLOOR over those rows, because the
    // surfaces that read it hold no row: `/bagd/status`, the terminal roll-up
    // and `bag info`'s counting caveat all state it on its own.
    assert_eq!(
        health.loss_counting_basis,
        Some(LossCountingBasis::PrefixInvisible),
        "one uncovered row means the document may not claim head coverage: {health:?}"
    );
    assert_eq!(
        summary.record_health.loss_counting_basis, health.loss_counting_basis,
        "the in-process summary and the bag's own document must agree"
    );

    // The CONTROL, same recorder shape with the guarantee withheld: every row is
    // invisible, so the declared row's `PrefixProven` above is the flag doing
    // work rather than a constant.
    let out2 = unique_out("basis_unarmed");
    let ready2 = unique_out("basis_unarmed_ready");
    let shutdown2 = Arc::new(AtomicBool::new(false));
    let cfg2 = discovery_cfg(
        out2.clone(),
        vec![TapSpec::attach(&declared)],
        ready2.clone(),
        Duration::from_millis(3000),
    );
    assert!(
        !cfg2.armed_before_producers,
        "precondition: the constructor claims no guarantee"
    );
    let handle2 = spawn_bagd(mgr.clone(), cfg2, shutdown2.clone());
    assert!(
        wait_for_file(&ready2, Duration::from_secs(10)),
        "bagd ready"
    );
    publish_oracle(&mut declared_pub, "d2", 2);
    await_bag_created(&out2);
    finish(handle2, &shutdown2);
    let unarmed = read_health(&out2);
    assert!(
        unarmed
            .topics
            .values()
            .all(|t| t.loss_counting_basis == Some(LossCountingBasis::PrefixInvisible)),
        "with no arm-ordering guarantee no row is covered: {unarmed:?}"
    );
    assert_eq!(
        unarmed.loss_counting_basis,
        Some(LossCountingBasis::PrefixInvisible)
    );
}

/// The ANTI-TAUTOLOGY control for the test above: with discovery OFF the very
/// same live producer is absent from the bag.
///
/// Without this arm, the headline test would pass on a build where the tap set
/// happened to be widened by something else entirely — and it is also the
/// back-compat pin, since OFF is exactly the pre-discovery behaviour.
#[test]
#[serial_test::serial]
fn with_discovery_off_the_same_undeclared_producer_is_absent_and_no_claim_is_made() {
    let mgr = make_manager(16);
    let declared = unique_topic("optout_declared");
    let undeclared = unique_topic("optout_undeclared");
    let out = unique_out("off");
    let ready = unique_out("off_ready");

    let mut declared_pub = producer(&mgr, &declared);
    let mut undeclared_pub = producer(&mgr, &undeclared);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::attach(&declared)],
        ready.clone(),
        Duration::from_millis(3000),
    );
    cfg.discover_live = false;
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_oracle(&mut declared_pub, "d", 3);
    publish_oracle(&mut undeclared_pub, "u", 3);
    await_bag_created(&out);
    finish(handle, &shutdown);

    assert_eq!(
        user_channels(&out),
        BTreeSet::from([declared.clone()]),
        "discovery OFF records ONLY the declared set (the pre-discovery behaviour)"
    );

    let coverage = read_coverage(&out);
    // The load-bearing half: an EMPTY untapped list here asserts nothing about
    // the machine, and `enumerated: false` is what tells a reader that.
    assert!(
        !coverage.enumerated,
        "a recording that did not look must not present itself as having looked"
    );
    assert!(coverage.untapped.is_empty());
    assert_eq!(coverage.gap_count(), 0);
    assert_eq!(coverage.tapped.len(), 1);
}

/// A producer that appears AFTER the recorder armed is picked up by the RESCAN.
///
/// This is the shape that actually matters on the `graph run --record` path:
/// bagd is armed BEFORE the graph is released to step 0, so a
/// runtime-registered producer does not exist at arm time and a single arm-time
/// scan would find nothing. The manifest must also mark the topic
/// `attached_late` — a data-only tap requests no late-joiner history, so
/// coverage genuinely begins at the attach instant and claiming otherwise would
/// be a back-fill the recorder cannot perform.
#[test]
#[serial_test::serial]
fn a_producer_appearing_after_arm_is_picked_up_by_discovery_and_marked_late() {
    let mgr = make_manager(16);
    let declared = unique_topic("late_declared");
    let late = unique_topic("late_arrival");
    let out = unique_out("late");
    let ready = unique_out("late_ready");

    let mut declared_pub = producer(&mgr, &declared);

    let shutdown = Arc::new(AtomicBool::new(false));
    // A long schema wait plus a DELIBERATELY SILENT declared tap keeps the bag
    // un-created (`learned_all()` is false) while discovery runs - so this test
    // exercises the "discovered before creation" window deterministically rather
    // than racing it.
    let cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::attach(&declared)],
        ready.clone(),
        Duration::from_secs(30),
    );
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    // The producer is born AFTER the recorder armed: the shape discovery exists for.
    let mut late_pub = producer(&mgr, &late);
    // Wait for DISCOVERY to have attached its tap, rather than
    // sleeping an interval and hoping. A frame published
    // before the attach reaches no queue (a data-only tap has no back-fill), so
    // this rendezvous is what makes the four-frame oracle below a statement
    // about DISCOVERY rather than about the drive loop's scheduling luck.
    await_discovered_tap(&mgr, &late);

    // Now let both speak: the bag is created once every tap (including the
    // just-discovered one) has learned its schema.
    publish_oracle(&mut declared_pub, "d", 2);
    publish_oracle(&mut late_pub, "l", 4);
    await_bag_created(&out);
    finish(handle, &shutdown);

    assert!(
        user_channels(&out).contains(&late),
        "a producer discovered by the RESCAN must get a channel"
    );
    let expected: Vec<Vec<u8>> = (0..4)
        .map(|i| build_frame(HASH, i, 1_000 + i as u64, &oracle_body("l", i)))
        .collect();
    assert_eq!(frames_for(&out, &late), expected);

    let coverage = read_coverage(&out);
    let entry = coverage.tapped.get(&late).expect("late topic in manifest");
    assert_eq!(entry.source, TapSource::Discovered);
    assert_eq!(entry.frames_recorded, 4);
    assert!(
        entry.attached_late,
        "a discovered find is covered only from its attach instant - the manifest must \
         say so rather than imply the whole run"
    );
    assert_eq!(coverage.gap_count(), 0);
}

// ===========================================================================
// DISCLOSURE — say what you are not tapping
// ===========================================================================

/// THE silent-lie regression pin.
///
/// A live producer that the recording does NOT contain must be named, with its
/// reason, in the bag's own manifest — and the run must not be able to present
/// itself as clean. The assertion is on the MANIFEST (a durable artifact a
/// reader can check months later), not on a log line that scrolls away.
///
/// `frames_lost` is asserted to be 0 IN THE SAME BODY on purpose: that is
/// precisely the reading the original field report got, and it was truthful. It counts
/// loss within the tapped set and says nothing about a producer outside it. The
/// two assertions together are the whole bug.
///
/// HARNESS NOTE: the recorder runs on a SPAN-CARRYING thread
/// ([`spawn_bagd_traced`]) so this body can rendezvous on the verdict it is
/// about to assert. See [`await_post_creation_verdict`] for why the sleep this
/// replaced could not be made reliable by lengthening it.
#[test]
#[serial_test::serial]
#[tracing_test::traced_test]
fn a_live_producer_the_bag_does_not_contain_is_named_with_its_reason() {
    let mgr = make_manager(16);
    let declared = unique_topic("silent_declared");
    let unrecorded = unique_topic("silent_unrecorded");
    let out = unique_out("silent");
    let ready = unique_out("silent_ready");

    let mut declared_pub = producer(&mgr, &declared);

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::attach(&declared)],
        ready.clone(),
        Duration::from_millis(200),
    );
    let handle = spawn_bagd_traced(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    // Publish so the single declared tap learns its schema, then WAIT for the
    // channel set to actually close. The 500 ms sleep this replaces was not
    // "the bag is created immediately": creation waits for the LATER of the
    // settle hold releasing and schema learning, which measured 506-522 ms
    // against a producer created at 607-610 ms — a ~90 ms margin that a loaded
    // runner erases (main CI run 31659934041).
    publish_oracle(&mut declared_pub, "d", 2);
    await_channel_set_closed(logs_contain);

    // Only NOW does the second producer appear. Its channel can never exist in
    // this bag (MCAP channels are registered at creation and are immutable), so
    // the recorder's only correct move is to name it.
    let mut late_pub = producer(&mgr, &unrecorded);
    publish_oracle(&mut late_pub, "z", 3);
    // Rendezvous on the recorder having OBSERVED it, rather than
    // sleeping a few enumeration cadences and finalizing into whatever happened to
    // have run. Under CI load the scan lost that race and the manifest came back
    // with no entry at all — which reads as the recorder deciding not to report,
    // the exact defect this arm exists to catch.
    await_post_creation_verdict(&mgr, &unrecorded, verdict_probe!(&unrecorded));
    let summary = finish(handle, &shutdown);

    assert!(
        !user_channels(&out).contains(&unrecorded),
        "precondition: this producer is genuinely absent from the bag"
    );
    assert_eq!(
        summary.frames_lost, 0,
        "precondition: the TAPPED set lost nothing — exactly the reading that made \
         the original bug invisible"
    );

    let coverage = read_coverage(&out);
    assert_eq!(
        coverage.untapped.get(&unrecorded),
        Some(&UntappedReason::AppearedAfterBagCreation),
        "a live producer absent from the bag MUST be named with its reason"
    );
    assert_eq!(
        coverage.gap_count(),
        1,
        "the run must count as coverage-incomplete, so its terminal line cannot \
         read clean"
    );
    assert_eq!(summary.record_coverage, coverage);
}

/// The recorder's OWN status channel is live, and is excluded BY RULE — so it is
/// reported as excluded rather than as a coverage gap.
///
/// A recorder recording its own status grows without bound and tells the
/// operator nothing, so the exclusion is correct; but silently omitting a live
/// topic from the manifest would make the manifest incomplete, and reporting it
/// as a GAP would train the operator to skim the number that matters.
#[test]
#[serial_test::serial]
fn the_recorders_own_status_topic_is_excluded_by_rule_and_reported_as_such() {
    let mgr = make_manager(16);
    let declared = unique_topic("status_declared");
    let out = unique_out("status");
    let ready = unique_out("status_ready");

    let mut declared_pub = producer(&mgr, &declared);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::attach(&declared)],
        ready.clone(),
        Duration::from_millis(3000),
    );
    // Status ON: bagd creates `/bagd/status`, which is a genuinely live topic on
    // this SHM root and WILL be enumerated.
    cfg.status_period = Some(Duration::from_millis(50));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_oracle(&mut declared_pub, "d", 2);
    std::thread::sleep(RESCAN_SETTLE);
    finish(handle, &shutdown);

    assert_eq!(
        user_channels(&out),
        BTreeSet::from([declared.clone()]),
        "the recorder must never record itself"
    );
    let coverage = read_coverage(&out);
    assert_eq!(
        coverage.untapped.get(cerulion_bagd::STATUS_TOPIC),
        Some(&UntappedReason::ExcludedInternal),
        "a live-but-excluded topic is still ACCOUNTED FOR — the manifest is a \
         complete picture of the machine or it is not a manifest"
    );
    assert_eq!(
        coverage.gap_count(),
        0,
        "an exclusion by rule is NOT a coverage gap and must not inflate the count \
         operators read"
    );
}

/// Back-compat: when the declared set already IS the live set, discovery adds
/// nothing and the recording is unchanged.
///
/// This is the plain-graph case — the overwhelming majority of `--record` runs —
/// and it must not acquire new channels, new gaps, or a different verdict just
/// because enumeration is on.
#[test]
#[serial_test::serial]
fn when_declared_equals_live_discovery_changes_nothing() {
    let mgr = make_manager(16);
    let a = unique_topic("bc_a");
    let b = unique_topic("bc_b");
    let out = unique_out("bc");
    let ready = unique_out("bc_ready");

    let mut pa = producer(&mgr, &a);
    let mut pb = producer(&mgr, &b);

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::attach(&a), TapSpec::attach(&b)],
        ready.clone(),
        Duration::from_millis(3000),
    );
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_oracle(&mut pa, "a", 2);
    publish_oracle(&mut pb, "b", 2);
    std::thread::sleep(RESCAN_SETTLE);
    let summary = finish(handle, &shutdown);

    assert_eq!(user_channels(&out), BTreeSet::from([a.clone(), b.clone()]));
    let coverage = read_coverage(&out);
    assert!(coverage.enumerated);
    assert_eq!(coverage.gap_count(), 0);
    assert!(
        coverage.untapped.is_empty(),
        "nothing else was live, so nothing is reported"
    );
    for topic in [&a, &b] {
        let e = coverage.tapped.get(topic).expect("declared tapped");
        assert_eq!(e.source, TapSource::Declared);
        assert!(!e.attached_late);
        assert_eq!(e.frames_recorded, 2);
    }
    assert_eq!(summary.messages, 4);
    assert_eq!(summary.frames_lost, 0);
}

/// A DISCOVERED topic at the DEFAULT borrow floor records in full, beside a
/// declared topic provisioned for recording — on ONE write path.
///
/// This is the shipping shape on a `graph run --record` robot, and it used to be
/// the one place discovery reached into the write-path DECISION: a recorded
/// topic is provisioned by the recorder at a raised borrow ceiling while a
/// dynamically-created producer is provisioned by its own producer at the stock
/// floor, and the old writer thread required EVERY tap to afford several
/// concurrent batches of borrowed samples. Adding such a tap therefore dropped
/// the whole run to the inline writer — which is how the flagship path ended up
/// inline, and how the Go2 lost 17,720 frames.
///
/// The fix removed the decision rather than re-taking it: a staged frame is a
/// COPY, so a discovered tap costs its producer's budget nothing and the writer
/// thread is affordable at any budget. What this test pins is the OUTCOME that
/// change is for — a mixed declared/discovered recording, both at their own
/// provisioning, losing nothing.
///
/// The oracle is DELIVERY on both topics, byte-for-byte.
#[test]
#[serial_test::serial]
fn a_discovered_default_borrow_topic_is_recorded_in_full_beside_a_declared_one() {
    // Deep enough to exceed anything one drain request can take in a pass, and
    // within the transport's 16-deep subscriber queue so the burst itself is
    // never the thing that loses frames.
    const BURST: u32 = 10;
    let mgr = make_manager(16);
    let declared = unique_topic("wp_declared");
    let discovered = unique_topic("wp_discovered");
    let out = unique_out("wp");
    let ready = unique_out("wp_ready");

    // Declared: provisioned exactly as `graph run --record` provisions a
    // RECORDED topic — enough borrow for the writer thread to engage.
    let mut declared_pub = publisher_with_provisioning(
        &mgr,
        &declared,
        cerulion_core::transport::RECORDING_SUBSCRIBER_MAX_BORROWED,
        16,
        4096,
    );
    // Discovered: the DEFAULT provisioning a runtime-created ingress publisher
    // gets — nowhere near the writer thread's requirement.
    let mut discovered_pub = publisher(&mgr, &discovered, 4096);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::attach(&declared)],
        ready.clone(),
        Duration::from_millis(3000),
    );
    // This arm is about the WRITE PATH, not the discovery window: both producers
    // are live before the recorder arms, so the arm-time scan already has the
    // full tap set and holding creation open buys nothing. Pinning the window to
    // ZERO keeps bag creation governed by schema learning (phase 1 below), so
    // the burst deterministically lands AFTER creation and cannot be confounded
    // by the settle hold's own queueing behaviour.
    cfg.discovery_settle = Duration::ZERO;
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    // PHASE 1 — one frame each, so both taps learn and the bag is CREATED, so
    // the burst below is drained by a tap whose flush can free its budget
    // (pre-creation there is no flush, so a burst deeper than the queue would
    // overflow the QUEUE and confound this arm's exact frame-count oracle).
    declared_pub
        .publish_raw(&build_frame(HASH, 0, 1_000, &oracle_body("wd", 0)))
        .expect("publish declared");
    discovered_pub
        .publish_raw(&build_frame(HASH, 0, 1_000, &oracle_body("wx", 0)))
        .expect("publish discovered");
    std::thread::sleep(Duration::from_millis(400));

    // PHASE 2 — a BURST on the low-borrow topic, faster than the flush cadence.
    //
    // The burst is what makes this test able to see anything at all. A lockstep
    // drip keeps at most one frame in flight per tap, which ANY recorder can
    // serve — the first version of this test did exactly that, and reverting to
    // the old write-path decision (never re-taken) would still pass it.
    // The burst still earns its place: it is what would have over-borrowed this
    // tap under the old writer thread, and it is what proves a default-floor
    // producer is drained at rate rather than one frame per pass.
    for i in 1..=BURST {
        let x = build_frame(HASH, i, 1_000 + i as u64, &oracle_body("wx", i));
        discovered_pub.publish_raw(&x).expect("publish discovered");
    }
    std::thread::sleep(Duration::from_millis(600));
    let summary = finish(handle, &shutdown);

    let declared_expected = vec![build_frame(HASH, 0, 1_000, &oracle_body("wd", 0))];
    assert_eq!(
        frames_for(&out, &declared),
        declared_expected,
        "the declared topic must be recorded after the write-path re-decision"
    );
    let discovered_expected: Vec<Vec<u8>> = (0..=BURST)
        .map(|i| build_frame(HASH, i, 1_000 + i as u64, &oracle_body("wx", i)))
        .collect();
    assert_eq!(
        frames_for(&out, &discovered),
        discovered_expected,
        "every frame on the default-floor discovered topic must be recorded — its \
         producer's borrow budget is not the recorder's throughput any more"
    );
    assert_eq!(summary.frames_lost, 0);
    assert_eq!(summary.dropped_unwritten, 0);
    let coverage = read_coverage(&out);
    assert_eq!(coverage.gap_count(), 0);
    assert_eq!(
        coverage
            .tapped
            .get(&discovered)
            .expect("discovered tapped")
            .source,
        TapSource::Discovered
    );
}

// ===========================================================================
// THE SHIPPING SHAPE — exact-mode declared taps (`graph run --record`)
// ===========================================================================

/// THE regression pin for the headline defect: on the
/// `graph run --record` path, discovery must actually RECORD a producer that
/// appears after the recorder arms.
///
/// Every other arm in this file builds its declared taps with
/// `TapSpec::attach(..)`, and that is the one tap mode that HIDES this bug. An
/// attach tap has no schema until its first frame, so `learned_all()` is false
/// and bag creation is deferred for free — which is how the late-producer arm
/// gets its window. `graph run --record` hands bagd `--topics-json`, which
/// produces ONLY exact-mode taps: every schema is known at construction, so
/// `learned_all()` is true on drive-loop pass 1 and the bag was created before
/// any later enumeration could run. Discovery was structurally INERT on the one
/// path it was built for, and the whole suite was green.
///
/// The producer here is created AFTER the ready-file, exactly as the attach
/// bridge's routes are created after GO releases the graph.
#[test]
#[serial_test::serial]
fn on_the_exact_mode_record_path_a_producer_appearing_after_arm_is_still_recorded() {
    let mgr = make_manager(16);
    let declared = unique_topic("exact_declared");
    let late = unique_topic("exact_late");
    let out = unique_out("exact");
    let ready = unique_out("exact_ready");

    let mut declared_pub = producer(&mgr, &declared);

    let shutdown = Arc::new(AtomicBool::new(false));
    // EXACT-mode declared tap — the `--topics-json` shape. `learned_all()` is
    // true immediately, so ONLY the discovery-settle window can hold the
    // channel set open.
    let cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::exact(
            &declared,
            cerulion_bag::TopicSchema {
                topic: declared.clone(),
                schema_name: "geometry_msgs/Vector3".to_string(),
                schema_hash: HASH,
                wire_fixed_size: 24,
            },
        )],
        ready.clone(),
        Duration::from_millis(500),
    );
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    // The producer is born AFTER the recorder armed.
    let mut late_pub = producer(&mgr, &late);
    // The same attach rendezvous as the late-arm sibling. It matters
    // MORE here, not less: this arm's channel set is held open only by the
    // discovery settle window, so publishing earlier (at the attach instant
    // rather than after a fixed sleep) also leaves more of that window intact.
    await_discovered_tap(&mgr, &late);
    publish_oracle(&mut declared_pub, "d", 2);
    publish_oracle(&mut late_pub, "l", 3);
    await_bag_created(&out);
    let summary = finish(handle, &shutdown);

    assert!(
        user_channels(&out).contains(&late),
        "a producer that appeared after arm must be RECORDED on the exact-mode path, not merely \
         reported — this is the shape `graph run --record` actually uses"
    );
    let expected: Vec<Vec<u8>> = (0..3)
        .map(|i| build_frame(HASH, i, 1_000 + i as u64, &oracle_body("l", i)))
        .collect();
    assert_eq!(frames_for(&out, &late), expected);

    let coverage = read_coverage(&out);
    let entry = coverage.tapped.get(&late).expect("late topic in manifest");
    assert_eq!(entry.source, TapSource::Discovered);
    assert!(entry.attached_late);
    assert_eq!(coverage.gap_count(), 0);
    assert!(!coverage.is_incomplete());
    // The declared tap keeps its EXACT schema — discovery must not downgrade it.
    assert_eq!(
        coverage
            .tapped
            .get(&declared)
            .expect("declared tapped")
            .source,
        TapSource::Declared
    );
    // A discovered channel is attach-mode, so the BAG is observability-grade
    // even though every declared tap was exact. The run must say so.
    assert!(
        !coverage.all_channels_exact,
        "a discovered channel carries the wire hash with schema name \"unknown\", so the bag is \
         not replay-grade and the manifest must not claim it is"
    );
    assert_eq!(summary.frames_lost, 0);
}

/// A `cerulion-netd` MIRROR of another robot's stream is folded OUT of
/// discovery, reported with its origin robot, and is not a coverage gap.
///
/// `cerulion bag record --all` has always done this (a mirror is a real local
/// `{topic}/data` service, so recording it would claim a local capture of remote
/// data). The first implementation copied that verb's PREFIX exclusion
/// list and claimed parity with it in a comment, while copying only half the
/// policy — so bagd's discovery tapped mirrors silently.
///
/// The mirror registry is real: this registers provenance through the same
/// `TransportManager` API `cerulion-netd` uses.
#[test]
#[serial_test::serial]
#[tracing_test::traced_test]
fn a_netd_mirror_of_a_remote_robot_is_not_recorded_as_a_local_producer() {
    let mgr = make_manager(16);
    let declared = unique_topic("mirror_declared");
    let mirrored = unique_topic("mirror_remote");
    let out = unique_out("mirror");
    let ready = unique_out("mirror_ready");

    let mut declared_pub = producer(&mgr, &declared);
    // A live local service that is REALLY a remote robot's re-injected stream.
    let mut mirror_pub = producer(&mgr, &mirrored);
    mgr.register_mirror_provenance(&mirrored, "go2")
        .expect("register mirror provenance");

    // CHOREOGRAPHY, not decoration. `register_mirror_provenance` PUBLISHES a
    // record onto the `/__cerulion/mirrors` service; `gather_mirror_provenance`
    // is a windowed LISTEN that a late-joining reader satisfies from retained
    // history. Whether bagd's one arm-time gather observes a record published
    // moments earlier is therefore a delivery race, and on a loaded runner it
    // loses: this arm failed on macOS CI with the mirror TAPPED, and the job log
    // shows the gather ran its full 600 ms window (armed at 41.046, next tap at
    // 41.714) and came back EMPTY — so the fold had nothing to fold and the
    // topic was correctly-by-its-own-lights discovered.
    //
    // Wait for the state the recorder is about to depend on to be OBSERVABLE,
    // rather than assuming a publish is synchronous. The oracle below is
    // unchanged; this only removes the race in front of it.
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        let seen = mgr
            .gather_mirror_provenance(Duration::from_millis(200))
            .expect("gather mirror provenance")
            .into_iter()
            .any(|r| r.topic == mirrored && r.origin_robot == "go2");
        if seen {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "the mirror provenance never became observable, so this arm could not have tested \
             the fold at all"
        );
        std::thread::sleep(Duration::from_millis(50));
    }

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::attach(&declared)],
        ready.clone(),
        Duration::from_millis(3000),
    );
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_oracle(&mut declared_pub, "d", 2);
    publish_oracle(&mut mirror_pub, "m", 2);
    std::thread::sleep(RESCAN_SETTLE);
    finish(handle, &shutdown);

    let coverage = read_coverage(&out);
    // The mirror picture, asserted FIRST because it is this arm's PREMISE. The fold can
    // only apply to a picture the recorder actually established, and reading
    // the registry is a windowed LISTEN that can expire having heard nothing —
    // which is exactly how this test failed twice on macOS CI, with the
    // recorder paying its full 600 ms window and then recording the mirror.
    //
    // Checking the verdict separates the two failure modes that would otherwise
    // both surface as "the mirror is in the bag": the FOLD is broken (the
    // regression this arm exists to catch), or this run lost the race three
    // times over and the arm never got to test the fold at all. The second is a
    // real failure too — the budget would be too small — but it is a different
    // one, and a test that cannot say which it hit sends the next reader to the
    // wrong place.
    assert_eq!(
        coverage.mirrors_established,
        Some(true),
        "the recorder must ESTABLISH the mirror picture before the fold means anything; \
         Some(false) here says every gather attempt expired with a live writer unheard (the \
         gather race, not the fold), and None says the verdict never reached the manifest"
    );

    assert_eq!(
        user_channels(&out),
        BTreeSet::from([declared.clone()]),
        "another robot's mirrored stream must not be recorded as this machine's data"
    );
    assert_eq!(
        coverage.untapped.get(&mirrored),
        Some(&UntappedReason::RemoteMirror {
            robot: "go2".to_string()
        }),
        "the fold must be REPORTED with its origin robot, not silently applied"
    );
    assert_eq!(
        coverage.gap_count(),
        0,
        "a remote robot's mirror is not a coverage gap of THIS recording"
    );
    assert!(!coverage.is_incomplete());

    // THE TWO-SURFACE PIN. `log_coverage_terminal` used a hand-rolled
    // `!= ExcludedInternal` while `gap_count()`/`is_incomplete()`/`bag info` use
    // `is_coverage_gap()`, so ONE recording with ONE mirror emitted a WARN
    // saying "LIVE producers on this machine are NOT in the bag, untapped = 1"
    // and, immediately after, the terminal line carrying `untapped = 0` — two
    // adjacent lines contradicting each other on one named field, with `bag
    // info` calling the same bag COMPLETE. The manifest assertions above cannot
    // see that; only the log can.
    logs_assert(|lines: &[&str]| {
        let contradictions: Vec<&&str> = lines
            .iter()
            .filter(|l| l.contains("coverage INCOMPLETE"))
            .collect();
        if contradictions.is_empty() {
            Ok(())
        } else {
            Err(format!(
                "a recording whose only untapped entry is a remote mirror must not report \
                 INCOMPLETE — the manifest says gap_count() == 0. Got: {contradictions:?}"
            ))
        }
    });
}

/// A desk whose netd holds a LIVE registry writer with ZERO
/// records records CLEANLY — `mirrors_established: Some(true)`, no gap, no
/// escalation.
///
/// This is the state every healthy desk spends most of its time in: netd keeps
/// its registry (and its iceoryx2 publisher) alive for the process lifetime, so
/// the moment its last mirror retires it is a live writer with an empty map.
/// Before the presence frame that writer sent NOTHING, which is byte-identical
/// on the wire to a writer the gather failed to hear — so every gather here
/// timed out, every attempt was spent, and the recorder stamped a FALSE
/// `mirrors_established: false` into the bag, warned `coverage UNVERIFIED`, and
/// paid the full ~1.8 s retry budget. On a machine with nothing wrong with it.
///
/// The sibling arm above proves a REAL mirror is folded out; this proves the
/// fold's machinery does not cry wolf when there is nothing to fold. The
/// producer is a genuine local one and MUST be recorded — a fix that made the
/// verdict clean by treating the topic as a mirror would fail here.
#[test]
#[serial_test::serial]
fn a_retired_netd_mirror_leaves_a_live_writer_that_still_settles_the_gather() {
    let mgr = make_manager(16);
    let declared = unique_topic("retired_declared");
    let out = unique_out("retired");
    let ready = unique_out("retired_ready");

    let mut declared_pub = producer(&mgr, &declared);

    // netd's shape after refcount-0: registered, then RETIRED. The registry —
    // and so the registry publisher — stays live for the process lifetime.
    let retired = unique_topic("retired_mirror");
    mgr.register_mirror_provenance(&retired, "go2")
        .expect("register mirror provenance");
    assert!(
        mgr.unregister_mirror_provenance(&retired)
            .expect("unregister mirror provenance"),
        "precondition: a mirror really was registered and is now retired"
    );

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::attach(&declared)],
        ready.clone(),
        Duration::from_millis(3000),
    );
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_oracle(&mut declared_pub, "d", 2);
    std::thread::sleep(RESCAN_SETTLE);
    finish(handle, &shutdown);

    let coverage = read_coverage(&out);
    assert_eq!(
        coverage.mirrors_established,
        Some(true),
        "a live writer holding no records is HEARD, so the mirror picture IS established. \
         Some(false) here is the defect this test pins: a false doubt stamped into the durable \
         manifest of a healthy desk's every recording"
    );
    assert!(
        !coverage.is_incomplete(),
        "nothing is missing and nothing is unverified — this run must not escalate"
    );
    assert_eq!(coverage.gap_count(), 0);
    // And the genuine local producer is still recorded: the verdict was made
    // clean by HEARING the writer, not by mistaking this topic for a mirror.
    assert_eq!(user_channels(&out), BTreeSet::from([declared.clone()]));
    assert!(
        coverage.untapped.is_empty(),
        "a retired mirror is not a live producer and must not be reported as one: {:?}",
        coverage.untapped
    );
}

/// HEADLINE: a producer at the STOCK borrow budget — the exact shape
/// the recorder used to refuse — is discovered AND recorded at full rate.
///
/// This test asserted the inverse until the held-frame change, and the inversion is the fix.
/// A held frame used to pin its shared-memory slot until the disk write
/// finished, so a tap's capacity was `max_borrowed - 1`: ZERO at a budget of 1,
/// meaning the tap would drain nothing for the whole run, which is why arming
/// refused it and the manifest reported `attach_failed`. That refusal was the
/// small end of the same defect that lost 17,720 frames on the Go2 — the
/// recorder's throughput was hostage to a number the PRODUCER chose.
///
/// Copy-at-drain removed the premise: `drain_taps` asks for one sample, copies
/// its payload, and drops the sample inside the loop, so one borrow is live at a
/// time and a budget of 1 records normally. The oracle is therefore the FRAMES,
/// byte-for-byte, from a topic no operator provisioned for recording.
///
/// `attach_failed` is NOT dead — what is gone is the BORROW BUDGET as a reason
/// for it (refusal would now need a budget of 0, which no iceoryx2 service can
/// be created with). The reason itself still fires for a topic the enumerator
/// names but the manager cannot open, and it is pinned on a different shape by
/// `a_discovered_producer_whose_subscriber_slots_are_full_is_named_with_its_error`
/// below — which exists precisely so that claim is not left resting on this
/// comment.
#[test]
#[serial_test::serial]
fn a_stock_borrow_budget_producer_is_discovered_and_recorded_at_full_rate() {
    let mgr = make_manager(16);
    let declared = unique_topic("attachfail_declared");
    let untappable = unique_topic("attachfail_untappable");
    let out = unique_out("attachfail");
    let ready = unique_out("attachfail_ready");

    let mut declared_pub = producer(&mgr, &declared);
    // borrow == 1: the LOWEST budget any service can carry, and the one arming
    // used to refuse outright.
    let mut hostile_pub = publisher_with_provisioning(&mgr, &untappable, 1, 16, 4096);

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::attach(&declared)],
        ready.clone(),
        Duration::from_millis(3000),
    );
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_oracle(&mut declared_pub, "d", 2);
    publish_oracle(&mut hostile_pub, "h", 2);
    std::thread::sleep(RESCAN_SETTLE);
    let summary = finish(handle, &shutdown);

    // The declared recording is untouched.
    let expected: Vec<Vec<u8>> = (0..2)
        .map(|i| build_frame(HASH, i, 1_000 + i as u64, &oracle_body("d", i)))
        .collect();
    assert_eq!(frames_for(&out, &declared), expected);

    // THE PIN: the borrow-1 topic is a real channel carrying its real frames,
    // against a hand oracle rather than a count.
    assert!(
        user_channels(&out).contains(&untappable),
        "a stock-borrow-budget producer must be RECORDED, not refused"
    );
    let hostile_expected: Vec<Vec<u8>> = (0..2)
        .map(|i| build_frame(HASH, i, 1_000 + i as u64, &oracle_body("h", i)))
        .collect();
    assert_eq!(
        frames_for(&out, &untappable),
        hostile_expected,
        "every frame from the borrow-1 topic, byte-for-byte"
    );

    let coverage = read_coverage(&out);
    assert!(
        !coverage.untapped.contains_key(&untappable),
        "a recorded topic is not an untapped one: {:?}",
        coverage.untapped
    );
    assert_eq!(
        coverage.gap_count(),
        0,
        "the borrow budget is no longer a coverage gap"
    );
    assert!(!coverage.is_incomplete());
    assert_eq!(summary.frames_lost, 0);
}

/// `attach_failed` still has a producer, on a shape the held-frame change did not remove.
///
/// The borrow-1 arm above USED to be this reason's only e2e producer; inverting
/// it (the whole point of that change) left `UntappedReason::AttachFailed` reachable
/// by no test in the repo, with its survival asserted only in prose. A reason
/// nothing can produce is a reason nobody can trust — and this one carries the
/// remediation an operator needs, which is DIFFERENT from every other untapped
/// reason: a widened discovery window cannot help a topic that could not be
/// OPENED.
///
/// The shape is subscriber-slot exhaustion, which is real rather than
/// contrived — `INTROSPECTION_SUBSCRIBER_HEADROOM` is finite and a machine
/// running a graph, a vizd attach, the liveness observer and a recorder
/// competes for exactly these slots. Here the producer provisions ONE slot and
/// the test takes it before the recorder can, so the discovery attach fails.
#[test]
#[serial_test::serial]
fn a_discovered_producer_whose_subscriber_slots_are_full_is_named_with_its_error() {
    let mgr = make_manager(16);
    let declared = unique_topic("slotfull_declared");
    let untappable = unique_topic("slotfull_untappable");
    let out = unique_out("slotfull");
    let ready = unique_out("slotfull_ready");

    let mut declared_pub = producer(&mgr, &declared);
    // ONE subscriber slot, and the test holds it for the whole run.
    let mut hostile_pub = publisher_with_subscriber_slots(&mgr, &untappable, 1, 4096);
    let _slot_hog = mgr
        .create_data_only_subscriber(&untappable)
        .expect("the test takes the topic's only subscriber slot");

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::attach(&declared)],
        ready.clone(),
        Duration::from_millis(3000),
    );
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_oracle(&mut declared_pub, "s", 2);
    publish_oracle(&mut hostile_pub, "x", 2);
    std::thread::sleep(RESCAN_SETTLE);
    let summary = finish(handle, &shutdown);

    // The declared recording is untouched by the neighbour it could not tap.
    let expected: Vec<Vec<u8>> = (0..2)
        .map(|i| build_frame(HASH, i, 1_000 + i as u64, &oracle_body("s", i)))
        .collect();
    assert_eq!(frames_for(&out, &declared), expected);
    assert_eq!(summary.frames_lost, 0);

    // THE PIN: named, with the reason AND the transport's own message.
    let coverage = read_coverage(&out);
    let reason = coverage.untapped.get(&untappable).unwrap_or_else(|| {
        panic!("a live producer the bag does not contain must be NAMED: {coverage:?}")
    });
    match reason {
        UntappedReason::AttachFailed { error } => assert!(
            !error.is_empty(),
            "attach_failed must carry the transport's own message — the operator cannot \
             act on a bare reason tag"
        ),
        other => panic!("expected AttachFailed, got {other:?}"),
    }
    assert!(
        !user_channels(&out).contains(&untappable),
        "a topic that could not be tapped must not have a channel"
    );
    assert!(
        coverage.is_incomplete(),
        "an un-attachable live producer is a coverage GAP"
    );
}

/// The operator-facing half: an incomplete recording's LAST word is a WARN that
/// names the untapped topic and its reason.
///
/// The manifest is the durable artifact, but an operator watching a terminal
/// sees the log, and the original field report's second defect was precisely a
/// clean-looking terminal summary printed beside unrecorded live producers.
///
/// HARNESS NOTE: `run_bagd` runs on the TEST thread here, with the stimulus on a
/// helper thread — the inverse of every other arm in this file. `tracing-test`
/// scopes its capture to the test's own SPAN, and a thread spawned inside the
/// test does not inherit it, so a recorder on a plainly spawned thread emits
/// lines this assertion can never see. (Measured: with the usual harness the
/// capture held zero lines containing "coverage" while the manifest showed the
/// gap.) A later flake fix added [`spawn_bagd_traced`], which CARRIES the span
/// across and so lifts that restriction; this arm keeps the inverted harness
/// because it works and re-shaping a passing test is churn the flake fix does
/// not need. Both shapes put the recorder's lines in the capture.
#[test]
#[serial_test::serial]
#[tracing_test::traced_test]
fn an_incomplete_recording_says_so_on_its_terminal_line() {
    let mgr = make_manager(16);
    let declared = unique_topic("loud_declared");
    let unrecorded = unique_topic("loud_unrecorded");
    let out = unique_out("loud");
    let ready = unique_out("loud_ready");

    let mut declared_pub = producer(&mgr, &declared);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::attach(&declared)],
        ready.clone(),
        Duration::from_millis(200),
    );
    // No hold: force the channel set closed early so the late producer lands in
    // the untapped ledger.
    cfg.discovery_settle = Duration::ZERO;

    let stim_mgr = mgr.clone();
    let stim_ready = ready.clone();
    let stim_topic = unrecorded.clone();
    let stim_shutdown = shutdown.clone();
    let stimulus = std::thread::spawn(move || {
        assert!(
            wait_for_file(&stim_ready, Duration::from_secs(10)),
            "bagd ready"
        );
        publish_oracle(&mut declared_pub, "d", 2);
        // Same precondition as the disclosure arm. `discovery_settle` is ZERO here,
        // so only schema learning gates creation — but that is still a state, not
        // a duration, and a loaded runner can take longer over it than any sleep
        // this arm would pick.
        await_channel_set_closed(logs_contain);
        let mut late_pub = producer(&stim_mgr, &stim_topic);
        publish_oracle(&mut late_pub, "z", 2);
        // The same rendezvous the disclosure arm uses. The `gap_count`
        // precondition below needs the recorder to have OBSERVED this producer,
        // and a fixed sleep bets on a loaded runner completing one scanner
        // cadence plus one drive-loop apply inside it.
        await_post_creation_verdict(&stim_mgr, &stim_topic, verdict_probe!(&stim_topic));
        stim_shutdown.store(true, Ordering::Relaxed);
    });

    let summary = run_bagd(mgr.clone(), cfg, shutdown.clone()).expect("clean finalize");
    stimulus.join().expect("stimulus thread");

    assert_eq!(summary.record_coverage.gap_count(), 1, "precondition");

    // The coverage line names the topic AND its reason, at WARN. Matching the
    // LEVEL token as well as the message: a reporter demoted to `info!` would
    // keep every word and lose the whole point.
    let topic = unrecorded.clone();
    logs_assert(move |lines: &[&str]| {
        let named: Vec<&&str> = lines
            .iter()
            .filter(|l| {
                l.split_whitespace().any(|t| t == "WARN")
                    && l.contains("coverage INCOMPLETE")
                    && l.contains(&topic)
                    && l.contains("appeared_after_bag_creation")
            })
            .collect();
        if named.len() == 1 {
            Ok(())
        } else {
            let coverage_lines: Vec<&&str> =
                lines.iter().filter(|l| l.contains("coverage")).collect();
            Err(format!(
                "expected exactly ONE WARN naming the untapped topic and its reason, got \
                 {named:?}; coverage lines seen: {coverage_lines:?}"
            ))
        }
    });

    // And the terminal summary itself must not read clean.
    logs_assert(|lines: &[&str]| {
        let clean = lines
            .iter()
            .filter(|l| l.contains("bagd finalized") && !l.contains("WITH ANOMALIES"))
            .count();
        if clean == 0 {
            Ok(())
        } else {
            Err(format!(
                "a coverage-incomplete run printed a clean terminal summary {clean} time(s)"
            ))
        }
    });
}

// ===========================================================================
// The settle window's two guarantees, each pinned by its own oracle
// ===========================================================================

/// THE FLOOR. Bag creation is held for at least [`DISCOVERY_SETTLE_MIN`].
///
/// The re-check found the real floor was ONE enumeration interval (~250 ms)
/// while three documents said 500 ms: `setup` discounts the ARM-TIME scan, but
/// the loop enumerated again microseconds later - still strictly before the
/// parent has seen the ready file and written GO, so equally uninformative — and
/// that scan DID count toward the quiet threshold the window then used. On the
/// flagship `ros2 attach`
/// path that closes the channel set ~250 ms after GO, which DDS discovery does
/// not beat, so every bridge route would land `appeared_after_bag_creation` and
/// the blocker's observable outcome would be unchanged.
///
/// The oracle is the RECORDER'S OWN measurement of how long it held —
/// `BagdSummary::channel_set_closed_after`, stamped inside `ensure_writer` at
/// the instant the tap set becomes final, against the SAME drive-loop origin
/// `discovery_hold_active` compares its `elapsed` to.
///
/// It used to be the bag FILE's appearance, timed from this test's own
/// observation of the ready-file — and that oracle FLAKED on main (measured
/// 487.65 ms against the 500 ms floor, run 30478166970). The undershoot was not
/// the product: load can only LENGTHEN a wall, never shorten it, so a
/// duration-floor measurement that reads SHORT under load is measuring from the
/// wrong origin. `run_bagd` writes the ready file and THEN enters `drive_loop`,
/// whose `start` is the clock the floor is enforced against, while this test
/// learns the file exists only at its next 5 ms poll — so the measured value was
/// `true hold − (anchor gap)`, and the anchor gap is exactly the poll latency,
/// which load inflates without bound. MEASURED on an idle desk: the gap runs
/// 4.1–4.8 ms against the 5 ms allowance the old assertion carried, i.e. the
/// allowance was already fully consumed before any contention.
///
/// The recorder's own number has no such term: it is the difference of two
/// reads of one monotonic clock inside one thread, and a preempted drive loop
/// can only notice the floor has expired LATER. So the assertion needs no
/// granularity allowance at all, and gets none.
///
/// It stays a LOWER bound. A ceiling assertion would invert on a loaded runner
/// and is not the property: the property is that the window does not close
/// EARLY.
///
/// The file wait below is kept, demoted from oracle to SYNCHRONISATION: it makes
/// the run reach its NATURAL release before shutdown is set, so the stamped
/// duration can never be a shutdown-driven force-create (which would be shorter
/// than the floor and would otherwise pass through as a real reading).
#[test]
#[serial_test::serial]
fn the_settle_hold_keeps_the_channel_set_open_for_at_least_the_documented_floor() {
    let mgr = make_manager(16);
    let declared = unique_topic("floor_declared");
    let out = unique_out("floor");
    let ready = unique_out("floor_ready");

    let _declared_pub = producer(&mgr, &declared);

    let shutdown = Arc::new(AtomicBool::new(false));
    // EXACT declared tap: `learned_all()` is true at construction, so the settle
    // hold is the ONLY thing that can defer bag creation. A generous cap so the
    // cap is never what is being measured.
    let cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::exact(
            &declared,
            cerulion_bag::TopicSchema {
                topic: declared.clone(),
                schema_name: "geometry_msgs/Vector3".to_string(),
                schema_hash: HASH,
                wire_fixed_size: 24,
            },
        )],
        ready.clone(),
        Duration::from_secs(30),
    );
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    // SYNCHRONISATION, not the oracle (see the doc comment): reach the natural
    // release before shutdown, so the stamped duration is never a force-create.
    assert!(
        wait_for_file(&out, Duration::from_secs(20)),
        "the bag file must eventually be created"
    );
    let summary = finish(handle, &shutdown);

    let held_for = summary.channel_set_closed_after.expect(
        "the bag was created inside the drive loop, so the recorder dated its own hold — a \
         `None` here means creation never ran there",
    );

    assert!(
        held_for >= cerulion_bagd::DISCOVERY_SETTLE_MIN,
        "the channel set closed after {held_for:?}, below the documented floor of {:?} — a \
         producer appearing inside that window cannot be recorded. This is the RECORDER's own \
         measurement against the drive loop's own clock, so load cannot explain it short",
        cerulion_bagd::DISCOVERY_SETTLE_MIN
    );
}

/// The dated hold is the WALL to bag creation, so SCHEMA LEARNING can
/// carry it PAST the settle cap.
///
/// `channel_set_closed_after` has a floor of `DISCOVERY_SETTLE_MIN` but NOT a
/// cap of `discovery_settle`. A cap claim
/// is false on a shipping shape: creation waits for the LATER of the hold
/// releasing and every DECLARED tap's schema being known, so a declared
/// attach-mode tap that never speaks holds it to the `schema_wait` force-create
/// deadline — at the defaults, 5000 ms against a 2000 ms cap.
///
/// Here the two are set 3x apart so the assertion is unambiguous about WHICH gate
/// bound: no test-side clock enters the oracle, and both bounds are LOWER bounds
/// (the wall can only grow under load).
#[test]
#[serial_test::serial]
fn the_dated_hold_is_the_wall_to_creation_so_schema_learning_can_exceed_the_settle_cap() {
    const SETTLE_CAP: Duration = Duration::from_millis(400);
    const SCHEMA_WAIT: Duration = Duration::from_millis(1200);

    let mgr = make_manager(16);
    let silent = unique_topic("cap_silent");
    let out = unique_out("cap");
    let ready = unique_out("cap_ready");

    // A real producer so the ATTACH tap can open, which never publishes — so
    // `learned_all()` stays false and only the schema-wait deadline can create
    // the bag.
    let _silent_pub = producer(&mgr, &silent);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::attach(&silent)],
        ready.clone(),
        SCHEMA_WAIT,
    );
    cfg.discovery_settle = SETTLE_CAP;
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    // Synchronisation only: creation happens naturally at the schema-wait
    // deadline, so no stimulus and no shutdown race is involved.
    assert!(
        wait_for_file(&out, Duration::from_secs(20)),
        "the schema-wait deadline must eventually create the bag"
    );
    let summary = finish(handle, &shutdown);

    let held_for = summary
        .channel_set_closed_after
        .expect("the bag was created inside the drive loop");
    assert!(
        held_for >= SCHEMA_WAIT,
        "schema learning gated creation, so the dated hold must reach the schema-wait deadline \
         {SCHEMA_WAIT:?}; got {held_for:?}"
    );
    assert!(
        held_for > SETTLE_CAP,
        "and it is therefore NOT capped by the settle window {SETTLE_CAP:?} — the value is \
         max(settle release, schema learning), which is what its doc must say; got {held_for:?}"
    );
}

/// And the FLOOR is conditional — two states reach bag creation without
/// ever paying `DISCOVERY_SETTLE_MIN`.
///
/// The other half of the same over-claim. Both scenarios assert a CEILING, which
/// this repo distrusts (a ceiling inverts on a loaded runner), so both are built
/// with no test-side timing at all and ~500x slack, rather than by timing a
/// SIGINT into the window and asserting a tight bound:
///
/// * DISCOVERY OFF (`cerulion bag record`, `CERULION_RECORD_DISCOVERY=off`) runs
///   no hold whatsoever, and an exact-mode tap makes `learned_all()` true at
///   construction, so creation happens on drive-loop pass 1.
/// * SHUTDOWN FORCES: the flag is set BEFORE the loop is entered, so the run
///   creates its bag without waiting the window out. SCOPE, because the
///   obvious reading is wrong: deleting `!shutting &&` from
///   `drive_loop`'s `discovery_hold` does NOT fail this — the loop then skips
///   `ensure_writer` and exits on its shutdown check, and `finalize` force-creates
///   with the same tiny elapsed. TWO mechanisms deliver the outcome, so this arm
///   pins the OUTCOME (a shutdown-driven creation is not floored) rather than
///   either mechanism, and a mid-window SIGINT would not discriminate them either.
///   What it DOES kill is a stamp doctored to satisfy the old doc — `held.max(
///   DISCOVERY_SETTLE_MIN)` fails both scenarios at `got 500ms`.
#[test]
#[serial_test::serial]
fn the_settle_floor_is_not_paid_when_discovery_is_off_or_shutdown_forces_creation() {
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

    // (a) DISCOVERY OFF — no hold exists to pay a floor to.
    {
        let mgr = make_manager(16);
        let declared = unique_topic("floor_off");
        let out = unique_out("floor_off");
        let ready = unique_out("floor_off_ready");
        let _p = producer(&mgr, &declared);
        let shutdown = Arc::new(AtomicBool::new(false));
        let mut cfg = discovery_cfg(
            out.clone(),
            vec![exact_tap(&declared)],
            ready.clone(),
            Duration::from_secs(30),
        );
        cfg.discover_live = false;
        let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
        assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
        assert!(
            wait_for_file(&out, Duration::from_secs(20)),
            "with no hold the bag is created on pass 1"
        );
        let summary = finish(handle, &shutdown);
        let held_for = summary
            .channel_set_closed_after
            .expect("the bag was created inside the drive loop");
        assert!(
            held_for < cerulion_bagd::DISCOVERY_SETTLE_MIN,
            "a discovery-off run pays no settle floor, so the dated hold must be far below {:?}; \
             got {held_for:?}",
            cerulion_bagd::DISCOVERY_SETTLE_MIN
        );
    }

    // (b) SHUTDOWN FORCES — a shutdown must never wait out a discovery window.
    {
        let mgr = make_manager(16);
        let declared = unique_topic("floor_sigint");
        let out = unique_out("floor_sigint");
        let ready = unique_out("floor_sigint_ready");
        let _p = producer(&mgr, &declared);
        // Set BEFORE the recorder runs: pass 1 sees `shutting` and force-creates.
        let shutdown = Arc::new(AtomicBool::new(true));
        let cfg = discovery_cfg(
            out.clone(),
            vec![exact_tap(&declared)],
            ready.clone(),
            Duration::from_secs(30),
        );
        // Discovery ON and a cap well above the floor: the hold WOULD have run
        // here, which is what makes this the bypass and not a missing feature.
        assert!(cfg.discover_live, "precondition");
        assert!(cfg.discovery_settle > cerulion_bagd::DISCOVERY_SETTLE_MIN);
        let summary = run_bagd(mgr.clone(), cfg, shutdown).expect("clean finalize");
        let held_for = summary
            .channel_set_closed_after
            .expect("a force-created bag is still dated");
        assert!(
            held_for < cerulion_bagd::DISCOVERY_SETTLE_MIN,
            "a shutdown-forced creation must not have waited out the {:?} window; got {held_for:?}",
            cerulion_bagd::DISCOVERY_SETTLE_MIN
        );
        assert!(
            !summary.bag_paths.is_empty(),
            "and it must still have produced a bag — otherwise the reading is vacuous"
        );
    }
}

/// The creation line and the terminal line of ONE discovery-off run must
/// tell the SAME story.
///
/// The review's second finding. The first version of the creation line was
/// unconditional, so a `bag record` run printed "a producer appearing after this
/// point can only be REPORTED (see record_coverage.json)" and then, from the same
/// run, "this recording makes no claim about what else was producing" — while
/// `rescan_discovery` returns before enumerating anything, so the second is the
/// true one. Which arm fires is unit-pinned in `lib.rs`; what only an end-to-end
/// run can show is that both lines really appear TOGETHER and agree.
///
/// HARNESS NOTE: `run_bagd` runs on the TEST thread with the stimulus on a helper,
/// for the reason `an_incomplete_recording_says_so_on_its_terminal_line` documents
/// — `tracing-test` scopes its capture to the test's own span.
#[test]
#[serial_test::serial]
#[tracing_test::traced_test]
fn a_discovery_off_run_does_not_promise_a_coverage_report_it_will_never_write() {
    let mgr = make_manager(16);
    let declared = unique_topic("offline_declared");
    let out = unique_out("offline");
    let ready = unique_out("offline_ready");

    let mut declared_pub = producer(&mgr, &declared);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::attach(&declared)],
        ready.clone(),
        Duration::from_millis(500),
    );
    cfg.discover_live = false;

    let stim_ready = ready.clone();
    let stim_shutdown = shutdown.clone();
    let stimulus = std::thread::spawn(move || {
        assert!(
            wait_for_file(&stim_ready, Duration::from_secs(10)),
            "bagd ready"
        );
        publish_oracle(&mut declared_pub, "d", 2);
        stim_shutdown.store(true, Ordering::Relaxed);
    });

    let summary = run_bagd(mgr.clone(), cfg, shutdown.clone()).expect("clean finalize");
    stimulus.join().expect("stimulus thread");
    assert!(
        summary.channel_set_closed_after.is_some(),
        "precondition: the line under test only fires when the hold was dated"
    );

    logs_assert(|lines: &[&str]| {
        let closed: Vec<&&str> = lines
            .iter()
            .filter(|l| l.contains("bagd CHANNEL SET CLOSED"))
            .collect();
        if closed.len() != 1 {
            return Err(format!(
                "expected exactly one channel-set-closed line; got {closed:?}"
            ));
        }
        let line = closed[0];
        // The promise nothing on this path keeps.
        if line.contains("record_coverage.json") || line.contains("REPORTED") {
            return Err(format!(
                "a discovery-off run promised a late-producer report that `rescan_discovery` \
                 returns before ever writing: {line}"
            ));
        }
        // And it must not attribute the wall to a settle window that never ran.
        if line.contains("settle window") {
            return Err(format!("nothing was settling on this run: {line}"));
        }
        // Both lines of the run, agreeing.
        let opted = lines
            .iter()
            .filter(|l| l.contains("makes no claim about what else was producing"))
            .count();
        if opted == 0 {
            return Err(
                "the terminal opted-out line must still be printed, or this arm proves nothing \
                 about the two lines agreeing"
                    .to_string(),
            );
        }
        if !line.contains("makes no claim about what else was producing") {
            return Err(format!(
                "and the creation line must carry the same claim rather than the opposite one: \
                 {line}"
            ));
        }
        Ok(())
    });
}

/// EARLY RELEASE. A settled live set releases bag creation on the QUIET rule,
/// far below the cap.
///
/// The oracle is the CHANNEL SET being closed, observed through a second
/// producer created well after the floor and reported
/// `AppearedAfterBagCreation`. The previous version of this arm asserted only
/// that the run finished in under 15 s against a 30 s cap — which the SHUTDOWN
/// path satisfies on its own (`discovery_hold = !shutting && ..`), so it passed
/// with the quiet rule deleted and proved a different guarantee than its name.
///
/// HARNESS NOTE: the oracle is a post-creation verdict, so this arm
/// carries the same rendezvous — and the same span-carrying recorder thread — as
/// the disclosure arm. A deleted quiet rule still fails it loudly: the late
/// producer would be TAPPED, which [`await_post_creation_verdict`] reports by
/// name rather than as a timeout.
#[test]
#[serial_test::serial]
#[tracing_test::traced_test]
fn a_settled_live_set_releases_the_bag_on_the_quiet_rule_not_the_cap() {
    let mgr = make_manager(16);
    let declared = unique_topic("settle_declared");
    let late = unique_topic("settle_late");
    let out = unique_out("settle");
    let ready = unique_out("settle_ready");

    let mut declared_pub = producer(&mgr, &declared);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::exact(
            &declared,
            cerulion_bag::TopicSchema {
                topic: declared.clone(),
                schema_name: "geometry_msgs/Vector3".to_string(),
                schema_hash: HASH,
                wire_fixed_size: 24,
            },
        )],
        ready.clone(),
        Duration::from_millis(500),
    );
    // A cap so large that ONLY the quiet rule can close the channel set inside
    // this test's lifetime. If the quiet rule is removed, the hold runs to this
    // cap, the late producer below is still discoverable, and it is TAPPED
    // rather than reported — which inverts the oracle.
    cfg.discovery_settle = Duration::from_secs(30);
    let handle = spawn_bagd_traced(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_oracle(&mut declared_pub, "d", 2);
    // Wait for the channel set to CLOSE, rather than sleeping past the floor and
    // hoping the quiet rule fired inside the margin. This also sharpens the
    // oracle: with the quiet rule deleted the set closes only at the 30 s cap
    // above, which is beyond this rendezvous' ceiling, so that variant fails here
    // — naming the unreleased hold — instead of further down.
    await_channel_set_closed(logs_contain);

    let mut late_pub = producer(&mgr, &late);
    publish_oracle(&mut late_pub, "l", 2);
    await_post_creation_verdict(&mgr, &late, verdict_probe!(&late));
    let summary = finish(handle, &shutdown);

    let coverage = read_coverage(&out);
    assert_eq!(
        coverage.untapped.get(&late),
        Some(&UntappedReason::AppearedAfterBagCreation),
        "the quiet rule must have closed the channel set long before the 30 s cap — a producer \
         arriving after the floor is REPORTED, not tapped"
    );
    assert!(!user_channels(&out).contains(&late));
    // The declared recording is intact.
    let expected: Vec<Vec<u8>> = (0..2)
        .map(|i| build_frame(HASH, i, 1_000 + i as u64, &oracle_body("d", i)))
        .collect();
    assert_eq!(frames_for(&out, &declared), expected);
    assert_eq!(summary.frames_lost, 0);
}

/// THE HOLD MUST NOT EVICT. Frames published INSIDE the settle window survive.
///
/// This is the regression pin for the worse of two defects: a hold that
/// drop-oldest-EVICTS, reducing a 300-frame
/// recording to 2. The ten other e2e tests that would catch it are de-fanged
/// by `BagdConfig::new`'s default being OFF, so nothing else pins it.
///
/// Shape: an EXACT declared tap (so the hold is the ONLY creation gate) and a
/// burst of `held_budget + N` frames published while the hold is active — a tap
/// sitting at `room == 0` with frames still arriving.
///
/// SCOPE NOTE: this arm's outcome is now guaranteed GLOBALLY — the
/// pre-writer drop-oldest probe it was written against is retired, so
/// `drain_taps` waits at `room == 0` on every branch. Deleting the settle
/// hold's own no-evict handling can therefore no longer fail it; what it still
/// pins is that the HOLD does not lose frames for any other reason (an
/// over-eager force-create, a queue mis-provisioned against the hold's length).
#[test]
#[serial_test::serial]
fn frames_published_inside_the_settle_hold_are_not_evicted() {
    // Comfortably above the pre-writer held budget of a default-provisioned tap
    // (max_borrowed 2 => budget 1), and within the 16-deep subscriber queue so
    // the QUEUE is never what loses a frame.
    const BURST: u32 = 12;
    let mgr = make_manager(16);
    let declared = unique_topic("noevict_declared");
    let out = unique_out("noevict");
    let ready = unique_out("noevict_ready");

    // DEFAULT provisioning: held budget 1, so every frame after the first finds
    // the tap at `room == 0` while the writer does not exist — the state that
    // used to evict and now waits.
    let mut declared_pub = publisher(&mgr, &declared, 4096);

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::exact(
            &declared,
            cerulion_bag::TopicSchema {
                topic: declared.clone(),
                schema_name: "geometry_msgs/Vector3".to_string(),
                schema_hash: HASH,
                wire_fixed_size: 24,
            },
        )],
        ready.clone(),
        Duration::from_secs(30),
    );
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    // Let the tap's connection establish before the burst. A publisher only
    // learns about a newly-attached subscriber on its next `update_connections`,
    // so a frame sent in that window reaches nobody — a transport fact, not the
    // property under test, and every other arm gets this for free via
    // `publish_oracle`'s per-frame settle. (Measured: without it exactly frame 0
    // is absent, which would have been misread as an eviction.)
    settle();
    // Burst, while the floor guarantees the hold is active and the bag provably
    // does not exist yet.
    assert!(
        !out.exists(),
        "precondition: the settle hold means no bag yet at t=0"
    );
    for i in 0..BURST {
        declared_pub
            .publish_raw(&build_frame(
                HASH,
                i,
                1_000 + i as u64,
                &oracle_body("nv", i),
            ))
            .expect("publish");
    }
    std::thread::sleep(Duration::from_millis(2000));
    let summary = finish(handle, &shutdown);

    // The MECHANISM first: the hold must not drop-oldest-evict. Asserting this
    // before the frame oracle makes a failure attributable — an eviction and a
    // queue overflow both remove frames, and only one of them is this fix.
    assert_eq!(
        summary.dropped_unwritten, 0,
        "the hold must not drop-oldest-evict; got {} drops",
        summary.dropped_unwritten
    );
    assert_eq!(
        summary.frames_lost, 0,
        "no tap-queue overflow either; got {} lost",
        summary.frames_lost
    );
    let expected: Vec<Vec<u8>> = (0..BURST)
        .map(|i| build_frame(HASH, i, 1_000 + i as u64, &oracle_body("nv", i)))
        .collect();
    assert_eq!(
        frames_for(&out, &declared),
        expected,
        "every frame published during the settle hold must survive — the hold is a wait the \
         RECORDER chose and must not pay for itself in frames"
    );
}

/// The coverage manifest round-trips through the writer thread's finalize.
///
/// SCOPE, corrected by the held-frame change: this arm was written when a discovered producer
/// at the default borrow floor dropped the whole run INLINE, so it was the only
/// arm in the file that reached the writer thread's own
/// `WriterMsg::Finalize { health, coverage }` branch — the one that serializes
/// and writes the attachment off the writer thread. With one write path left,
/// EVERY arm reaches it and that uniqueness claim is simply false.
///
/// It is kept, at its real value: a direct two-topic (declared + discovered)
/// round-trip oracle for the manifest, over producers provisioned as
/// `graph run --record` provisions them. It is no longer a path discriminator,
/// and its name no longer claims to be one.
#[test]
#[serial_test::serial]
fn the_coverage_manifest_round_trips_through_the_finalize_path() {
    let mgr = make_manager(16);
    let declared = unique_topic("thr_declared");
    let discovered = unique_topic("thr_discovered");
    let out = unique_out("threaded");
    let ready = unique_out("threaded_ready");

    let borrow = cerulion_core::transport::RECORDING_SUBSCRIBER_MAX_BORROWED;
    let mut declared_pub = publisher_with_provisioning(&mgr, &declared, borrow, 16, 4096);
    let mut discovered_pub = publisher_with_provisioning(&mgr, &discovered, borrow, 16, 4096);

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::attach(&declared)],
        ready.clone(),
        Duration::from_millis(3000),
    );
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_oracle(&mut declared_pub, "td", 3);
    publish_oracle(&mut discovered_pub, "tx", 3);
    std::thread::sleep(RESCAN_SETTLE);
    let summary = finish(handle, &shutdown);

    // Both topics recorded, byte-identical to the hand oracle.
    for (topic, tag) in [(&declared, "td"), (&discovered, "tx")] {
        let expected: Vec<Vec<u8>> = (0..3)
            .map(|i| build_frame(HASH, i, 1_000 + i as u64, &oracle_body(tag, i)))
            .collect();
        assert_eq!(frames_for(&out, topic), expected);
    }

    // THE PIN: the attachment written by the WRITER THREAD carries the
    // discovered entry.
    let coverage = read_coverage(&out);
    assert_eq!(
        coverage
            .tapped
            .get(&discovered)
            .expect("the discovered topic is in the threaded-path manifest")
            .source,
        TapSource::Discovered
    );
    assert_eq!(coverage.gap_count(), 0);
    assert!(!coverage.is_incomplete());
    assert_eq!(
        summary.record_coverage, coverage,
        "summary == bag attachment"
    );
    // A discovered channel is attach-mode, so the bag is observability-grade
    // whichever write path produced it.
    assert!(!coverage.all_channels_exact);
}

/// A LIVE run's `/__cerulion/runs` registry is INVISIBLE to the
/// recorder — it is neither recorded nor reported.
///
/// The run registry is a framework CONTROL service, so it must not appear as a
/// channel (a recorder recording the run descriptor tells the operator nothing
/// and grows with every republish) and must not appear in the coverage
/// manifest either: `untapped` is the operator's picture of what was LIVE and
/// NOT recorded, and a control service belongs in neither half of it.
///
/// Invisibility rests on TWO independent guards, and this arm is what proves
/// the FIRST one is load-bearing rather than merely believed:
///
/// 1. the service name carries no `/data` suffix, and
///    `TransportManager::list_topics` derives topics only from `*/data`
///    services — so the enumerator never yields it at all;
/// 2. `EXCLUDED_TOPIC_PREFIXES` would exclude it anyway (it lives under
///    `/__cerulion/`).
///
/// Guard 2 alone would keep the topic out of the BAG, which is why the
/// assertion is on the MANIFEST as well: naming the registry `.../data` makes
/// the enumerator yield it, guard 2 catches it, and it lands in `untapped` as
/// `ExcludedInternal` — an entry that has no business being there. That is the
/// difference the manifest assertion can see and a channel assertion cannot.
///
/// The run is published on the recorder's OWN isolated SHM root through the
/// `cerulion_core::testing` JSON adapter — `cerulion_bagd` deliberately names
/// no `iceoryx2` type (see its Cargo.toml).
#[test]
#[serial_test::serial]
fn a_live_runs_registry_is_neither_recorded_nor_reported_as_a_gap() {
    let (mgr, ix_json) = make_manager_with_json(16);
    let declared = unique_topic("runreg_declared");
    let out = unique_out("runreg");
    let ready = unique_out("runreg_ready");

    let mut declared_pub = producer(&mgr, &declared);

    // A live run on the SAME SHM root the recorder enumerates.
    let _run = cerulion_core::testing::publish_run_on_ix_config_json(
        &ix_json,
        cerulion_core::transport::run_registry::RunRecord {
            run_id: 0x0981_0000_0000_0000_0000_0000_0000_0001,
            supervisor_pid: std::process::id(),
            run_started_at_ns: 1_753_000_000_000_000_000,
            state: cerulion_core::transport::run_registry::RunState::Live,
            graph_name: "runreg_graph".to_string(),
            run_dir: "/tmp/runreg_graph".to_string(),
        },
    )
    .expect("publish the run record on the recorder's own SHM root");

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = discovery_cfg(
        out.clone(),
        vec![TapSpec::attach(&declared)],
        ready.clone(),
        Duration::from_millis(3000),
    );
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_oracle(&mut declared_pub, "d", 2);
    std::thread::sleep(RESCAN_SETTLE);
    finish(handle, &shutdown);

    assert_eq!(
        user_channels(&out),
        BTreeSet::from([declared.clone()]),
        "the recorder must record the declared topic and NOTHING else — a run \
         registry is a control service, not data"
    );
    let coverage = read_coverage(&out);
    assert!(
        coverage.enumerated,
        "precondition: this run really did enumerate the live service directory, \
         so 'the registry is absent' is a fact about the enumeration rather than \
         about enumeration never having happened"
    );
    let named: Vec<&String> = coverage
        .untapped
        .keys()
        .filter(|k| k.contains("__cerulion") || k.contains("runs"))
        .collect();
    assert!(
        named.is_empty(),
        "the run registry must not appear in the coverage manifest at all — it is \
         not a live PRODUCER the operator failed to record, it is framework \
         control traffic the enumerator should never have yielded; got {named:?}"
    );
    assert_eq!(
        coverage.gap_count(),
        0,
        "and it must certainly not count as a coverage gap"
    );
}
