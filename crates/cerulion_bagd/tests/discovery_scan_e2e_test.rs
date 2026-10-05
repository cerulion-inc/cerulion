// SPDX-License-Identifier: AGPL-3.0-only
//! The live-topic enumeration runs OFF the recorder's drive loop.
//!
//! The companion file is `discovery_event_e2e_test.rs`, which pins the other
//! half: that the worker is woken by FILESYSTEM EVENTS rather than a cadence, so
//! a settled machine runs no walks at all. This file pins that the walk is not
//! on the drive loop and that whatever replaced it actually works.
//!
//! The defect was not a decision the recorder made — every discovery verdict was
//! correct. It was WHERE the work happened: the discovery re-scan called
//! `TransportManager::list_topics()` inline on `drive_loop` every 250 ms, and
//! that call costs 117.5 ms at the Go2's 86 live topics (MEASURED first-party on
//! the Jetson). The loop therefore stopped draining for roughly a third of every
//! period, and the >= 200 Hz topics' 16-frame SHM queues overflowed inside
//! that window: 23 % frame loss with `staging_full_passes = 0` and
//! `dropped_unwritten = 0` throughout.
//!
//! That shape is why this file's headline arm is STRUCTURAL rather than a
//! stopwatch. What must be true is *"the drive loop performs no service-directory
//! walk"*, and the two obvious behavioural proxies both fail as pins:
//!
//! * A WALL on `next_scan` would be the class — a wall tight enough to
//!   separate a mutex take from a directory walk on THIS desk (2 topics,
//!   ~0.4 ms) is also tight enough for a loaded runner to invert, and the
//!   separation the fix buys is a function of the machine's live service count,
//!   which a hermetic test controls only weakly.
//! * A LOSS measurement would need the robot's topic count and rates to
//!   reproduce, which is what the kHz tap-overflow bench already does; adding a slow twin
//!   here would gate CI on a load-sensitive threshold for a property a source
//!   walk states exactly.
//!
//! So the walk pins the fix, and the real-transport arms pin that the thing
//! replacing the inline call actually works — a scanner that never produced a
//! scan would satisfy the walk perfectly while leaving discovery inert, which is
//! the defect all over again.

mod common;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_bag::BagReader;
use cerulion_bagd::discovery_scan::DiscoveryScanner;
use cerulion_bagd::{
    run_bagd, BagdConfig, RecordHealth, TapSource, TapSpec, RECORD_HEALTH_ATTACHMENT,
};
use cerulion_core::transport::TransportManager;

use common::*;

/// An arbitrary, stable schema hash for the hand-built frames.
const HASH: u64 = 0x0996_0996_0996_0996;

/// A LIVENESS ceiling for every condition wait here — stated in seconds, never
/// in units of the cadence under test (a bound that scales with the thing being
/// measured is the wall-in-its-own-units mistake).
const DEADLINE: Duration = Duration::from_secs(20);
/// How often a condition wait asks.
const POLL: Duration = Duration::from_millis(5);
/// The FALLBACK cadence the scanner arms drive at. Orders of magnitude longer
/// than the microseconds a burst of `next_scan` calls takes, which is what makes
/// the once-only assertion a property of the design rather than a race.
const SCAN_CADENCE: Duration = Duration::from_millis(100);

/// Block until `cond` holds, or fail LOUDLY with `what`.
fn await_condition(what: &str, mut cond: impl FnMut() -> bool) {
    let start = Instant::now();
    while start.elapsed() < DEADLINE {
        if cond() {
            return;
        }
        std::thread::sleep(POLL);
    }
    panic!("timed out after {DEADLINE:?} waiting for {what}");
}

// ===========================================================================
// The real-transport half: the worker really enumerates, and a scan is applied
// exactly once.
// ===========================================================================

/// The scanner runs a WORKER, that worker really RE-enumerates, and each
/// completed enumeration is handed to the caller EXACTLY ONCE.
///
/// The once-only property is what keeps the recorder's enumeration count a
/// count of enumerations: a `next_scan` that re-served the same snapshot on
/// every drive pass (~100/s) would have the recorder re-deciding a stale
/// snapshot a hundred times a second and reporting each of those as a scan.
///
/// The second half is the RE-enumeration pin, and it is the arm that separates a
/// working scanner from an inert one: a worker that enumerates ONCE and then
/// republishes that same snapshot forever satisfies every count-based assertion
/// while leaving discovery permanently blind to the producers live discovery exists to
/// record. So the later scan is checked for a topic that DID NOT EXIST when the
/// first one ran — content, not cardinality.
///
/// The waits are CONDITIONS with seconds-scale ceilings, never sleeps: the
/// property is "a later scan sees it", and how many cadences that takes is the
/// machine's business.
#[test]
fn the_scanner_re_enumerates_and_serves_each_scan_once() {
    let mgr: Arc<TransportManager> = make_manager(8);
    let first_topic = unique_topic("scan-a");
    // A live publisher is what puts a topic in the service directory.
    let _pub_a = publisher(&mgr, &first_topic, 64);

    let mut scanner = DiscoveryScanner::start_with_interval(&mgr, true, SCAN_CADENCE);
    assert!(
        scanner.is_threaded(),
        "discovery ON must run a worker thread — an inline fallback here would mean the spawn \
         failed, and the whole point of the worker is that the drive loop does not enumerate"
    );

    // Take the first COMPLETED scan (poll rather than sleep — the worker's first
    // enumeration lands whenever the machine gets to it).
    let mut first_scan = None;
    await_condition("the worker's first enumeration to be servable", || {
        first_scan = scanner.next_scan(&mgr);
        first_scan.is_some()
    });
    let topics = first_scan
        .expect("await_condition only returns once a scan was taken")
        .result
        .expect("enumeration over a healthy isolated namespace must succeed");
    assert!(
        topics.contains(&first_topic),
        "the worker's enumeration must see the live topic; got {topics:?}"
    );

    // THE ONCE-ONLY PIN: no NEW enumeration can have completed in the microseconds
    // these calls take (the cadence is orders of magnitude longer), so a `Some`
    // here is a re-serve. Asked repeatedly, because a slot cleared only on some
    // second call would pass a single check.
    for i in 0..5 {
        assert!(
            scanner.next_scan(&mgr).is_none(),
            "call {i}: a scan already served must never be served again — quiet-scan accounting \
             counts applied scans, and re-serving one collapses the discovery settle window"
        );
    }

    // THE RE-ENUMERATION PIN: a topic that did not exist during the first scan.
    let late_topic = unique_topic("scan-b");
    assert!(
        !topics.contains(&late_topic),
        "the late topic must be absent from the first scan, or the assertion below proves \
         nothing; got {topics:?}"
    );
    // The counter is sampled BEFORE the topic exists, and the order is the
    // assertion rather than a style. `ThreadedScanner` enumerates on its OWN
    // thread and `next_scan` merely TAKES the newest, so a scan can complete
    // between the publisher's creation and this read — and it would then satisfy
    // `saw_late` while having been counted ALREADY, leaving
    // `scans_run() > scans_before` false on a scanner that did exactly what the
    // arm requires. Sampled first, every scan that can contain the topic is
    // necessarily one this read did not count. MEASURED before the reorder: 1
    // failure in 5 runs, at `--test-threads=1`.
    let scans_before = scanner.scans_run();
    let _pub_b = publisher(&mgr, &late_topic, 64);

    let mut saw_late = false;
    await_condition(
        "a LATER enumeration to contain the newly-created topic",
        || {
            match scanner.next_scan(&mgr) {
                Some(scan) => {
                    // An enumeration that FAILS is not evidence either way — keep
                    // asking rather than reading it as an absence.
                    if let Ok(live) = scan.result {
                        saw_late = live.contains(&late_topic);
                    }
                    saw_late
                }
                None => false,
            }
        },
    );
    assert!(
        saw_late,
        "a scanner that enumerates once and republishes the same snapshot forever would leave \
         discovery inert for the rest of the recording"
    );
    assert!(
        scanner.scans_run() > scans_before,
        "the topic can only have been seen by an enumeration that ran AFTER it was created"
    );
}

/// Discovery OFF starts no thread and serves nothing — the anti-tautology
/// control for every assertion above, and the pin that a `bag record` run pays
/// nothing at all for a feature it did not ask for.
#[test]
fn a_disabled_scanner_starts_no_thread_and_serves_no_scans() {
    let mgr: Arc<TransportManager> = make_manager(8);
    let topic = unique_topic("off");
    let _pub = publisher(&mgr, &topic, 64);

    let mut scanner = DiscoveryScanner::start_with_interval(&mgr, false, Duration::from_millis(10));
    assert!(!scanner.is_threaded(), "discovery OFF must start no worker");

    // A cadence of 10 ms means an enabled scanner would have produced many scans
    // by now, so this is a real observation rather than a window too short to
    // see anything.
    std::thread::sleep(Duration::from_millis(200));
    assert_eq!(
        scanner.scans_run(),
        0,
        "a disabled scanner must run no enumerations"
    );
    for i in 0..5 {
        assert!(
            scanner.next_scan(&mgr).is_none(),
            "call {i}: a disabled scanner must serve nothing"
        );
    }
}

// ===========================================================================
// The observables: a real recording reports its own loop, in the bag.
// ===========================================================================

/// The pass observables are NOT INERT: a real `run_bagd` recording
/// reports its loop's cadence and its worst pass, in the summary AND in the
/// `record_health.json` a bag carries.
///
/// This is the arm the issue asked for by name — *"a `passes` counter plus a
/// max-pass-duration high-water … would have made this measurement
/// unnecessary"* — so the thing that must be pinned is that the numbers really
/// come from the run. A counter wired to nothing reads `0`, which is exactly
/// what a bag without this pass accounting reports, so nothing downstream
/// would notice.
///
/// The assertions are all `> 0` and cross-surface EQUALITY rather than any
/// value: a wall-derived expectation would be the class, while "the
/// loop ran at all" and "both surfaces agree" are properties load can only
/// delay.
#[test]
#[serial_test::serial]
fn a_recording_reports_its_own_drive_loop_in_the_summary_and_in_the_bag() {
    let mgr = make_manager(16);
    let topic = unique_topic("passes");
    let out = unique_out("passes");
    let ready = unique_out("passes_ready");
    let mut producer = publisher_with_provisioning(&mgr, &topic, 8, 16, 4096);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = BagdConfig::new(out.clone(), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.ready_file = Some(ready.clone());
    cfg.status_period = None;
    cfg.schema_wait = Duration::from_millis(3000);
    let mgr_for_bagd = mgr.clone();
    let flag = shutdown.clone();
    let handle = std::thread::spawn(move || run_bagd(mgr_for_bagd, cfg, flag));
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    for i in 0..3u32 {
        let frame = build_frame(HASH, i, 1_000 + i as u64, b"probe");
        producer.publish_raw(&frame).expect("publish_raw");
        settle();
    }
    std::thread::sleep(Duration::from_millis(200));
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("bagd thread").expect("clean finalize");

    assert!(
        summary.drive_passes > 0,
        "a recording that drove its loop must count its passes — 0 is what a pre-996 bag \
         reports, so a counter wired to nothing is indistinguishable from no fix at all"
    );
    assert!(
        summary.max_pass_duration > Duration::ZERO,
        "the worst pass must have a measured duration — the high-water is the number that \
         says whether the loop ever stopped draining, and it once had to be INFERRED from a \
         per-topic loss curve because nothing reported it"
    );

    // The bag carries the same numbers: the summary is in-process and dies with
    // the recorder, while `record_health.json` is what an operator reads later.
    let reader = BagReader::open(&out).expect("open bag");
    let att = reader
        .attachment(RECORD_HEALTH_ATTACHMENT)
        .expect("read attachments")
        .expect("record_health.json is present in EVERY finalized bag");
    let health: RecordHealth = serde_json::from_slice(&att.data).expect("record_health parses");
    assert_eq!(
        health.drive_passes, summary.drive_passes,
        "the bag and the summary must report ONE loop, not two"
    );
    assert_eq!(
        health.max_pass_duration_us,
        summary.max_pass_duration.as_micros() as u64,
        "the bag's microseconds must be the summary's duration — a unit slip here reads as a \
         1000x healthier loop"
    );
    assert!(
        health.max_pass_duration_us > 0,
        "a sub-microsecond truncation would report a loop that never did any work"
    );

    // ---- The CADENCE half, same no-inert-shipping standard --------
    //
    // `drive_passes` on its own is a count with no time base: 7,000 passes is a
    // healthy 100 Hz loop over 70 s and a broken 10 Hz one over 700 s, and
    // a document with no span field cannot tell them apart. These two fields are what make
    // the count mean something, so they get the same "did it come from the run"
    // treatment: `> 0` / non-vacant, plus cross-surface equality. No wall
    // expectation anywhere — a value assertion here would be the class.
    assert!(
        summary.drive_span > Duration::ZERO,
        "a recording that drove its loop must report the loop's SPAN — without it \
         `drive_passes` is uninterpretable"
    );
    assert!(
        !summary.drain_gaps.is_vacant(),
        "the drain-gap histogram must carry the run's own gaps — a vacant one is \
         exactly what a pre-1230 bag reports, so a histogram wired to nothing is \
         indistinguishable from no fix at all"
    );
    // A gap needs two passes to exist, so there is exactly one fewer of them.
    assert_eq!(
        summary.drain_gaps.total(),
        summary.drive_passes - 1,
        "every pass but the first must contribute exactly one gap — a mismatch \
         means the fold is skipping passes or double-counting them"
    );
    assert_eq!(
        health.drive_span_us,
        summary.drive_span.as_micros() as u64,
        "the bag's span must be the summary's — a unit slip reads as a 1000x \
         faster loop"
    );
    assert_eq!(
        health.drain_gaps, summary.drain_gaps,
        "the bag and the summary must report ONE distribution, not two"
    );
    assert_eq!(
        health.drain_gaps.edges_us,
        cerulion_bagd::DRAIN_GAP_BUCKET_EDGES_US.to_vec(),
        "the histogram must travel SELF-DESCRIBING — a reader that never linked \
         this crate has only the document to interpret the counts with"
    );
    // The derived quantity the issue actually needs, computed off the bag: at
    // some rate, how deep must a tap queue be to absorb this loop's gaps? Only
    // its EXISTENCE is asserted — the value is this machine's cadence.
    let depth_at_1k = health.drain_gaps.absorbing_depth(1_000.0, 0.5);
    assert!(
        depth_at_1k.is_some(),
        "the median gap must be inside the ladder, so a provisioning depth is \
         derivable from the bag alone; got the unbounded bucket instead"
    );

    cleanup(&out);
}

/// A recording that ends before ANY background scan completes still covers
/// everything that was live when it armed — because the ARM-TIME scan, which is
/// inline and complete, is what covers it.
///
/// # What this adjudicates
///
/// The concern is that `next_scan` can return `None` on the drive loop's first
/// pass (the worker starts asynchronously), so a recording shut down early might
/// lose coverage that the earlier loop — which enumerated INLINE on pass 1 —
/// would have had. The tap set does not depend on that scan: `Recorder::setup`
/// runs a COMPLETE inline enumeration (`rescan_discovery`) and applies it before
/// `discovery_armed` flips, and the ready-file is written only after `setup`
/// returns.
///
/// # Why `attached_late` is the assertion, and not a stopwatch
///
/// A test that tried to shut down "before the first background scan" would be
/// racing a worker whose first enumeration takes microseconds on an idle desk —
/// unwinnable, and it would prove the wrong thing. `attached_late` IS
/// `discovery_armed`, which is `false` for the arm-time scan and `true` for
/// every later one, so it names WHICH scan covered the topic. `false` here is
/// therefore a deterministic statement that the arm-time enumeration did the
/// work, whether or not the worker had also run by then.
///
/// The topic is UNDECLARED and live BEFORE the recorder arms, so discovery is
/// the only thing that could have tapped it at all.
#[test]
#[serial_test::serial]
fn a_recording_that_ends_early_still_covers_what_the_arm_time_scan_saw() {
    let mgr = make_manager(16);
    let declared = unique_topic("early-declared");
    let undeclared = unique_topic("early-undeclared");
    let out = unique_out("early");
    let ready = unique_out("early_ready");

    // BOTH live before the recorder arms; only one is named to bagd.
    let mut declared_pub = publisher_with_provisioning(&mgr, &declared, 8, 16, 4096);
    let mut undeclared_pub = publisher_with_provisioning(&mgr, &undeclared, 8, 16, 4096);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = BagdConfig::new(out.clone(), vec![TapSpec::attach(&declared)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.ready_file = Some(ready.clone());
    cfg.status_period = None;
    cfg.schema_wait = Duration::from_millis(3000);
    cfg.discover_live = true;
    let mgr_for_bagd = mgr.clone();
    let flag = shutdown.clone();
    let handle = std::thread::spawn(move || run_bagd(mgr_for_bagd, cfg, flag));
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    // Publish on both, then stop as promptly as the harness allows.
    for i in 0..3u32 {
        let frame = build_frame(HASH, i, 1_000 + i as u64, b"early");
        declared_pub.publish_raw(&frame).expect("publish declared");
        undeclared_pub
            .publish_raw(&frame)
            .expect("publish undeclared");
        settle();
    }
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("bagd thread").expect("clean finalize");

    let coverage = &summary.record_coverage;
    assert!(
        coverage.enumerated,
        "the ARM-TIME scan is a real enumeration — a sub-interval recording still made a \
         true claim about what was live"
    );
    let entry = coverage
        .tapped
        .get(&undeclared)
        .unwrap_or_else(|| panic!("the undeclared live producer must be in the bag: {coverage:?}"));
    assert_eq!(
        entry.source,
        TapSource::Discovered,
        "only discovery could have tapped a topic the caller never named"
    );
    assert!(
        !entry.attached_late,
        "THE ADJUDICATION: `attached_late` is `discovery_armed`, false ONLY for the arm-time \
         inline scan — so this topic was covered before the drive loop ran a single pass, and \
         the worker's first scan is not what the tap set depends on"
    );
    assert!(
        entry.frames_recorded > 0,
        "coverage means FRAMES, not just a channel"
    );
    assert_eq!(coverage.gap_count(), 0, "nothing live went unrecorded");

    cleanup(&out);
}

// ===========================================================================
// The structural half: the drive loop performs no directory walk.
// ===========================================================================

/// THE PIN: `drive_loop` enumerates nothing.
///
/// Restoring the inline
/// `rec.rescan_discovery(mgr)` call in the loop fails this (the body then
/// reaches `list_topics` through a call the walk names), and a COMMENT in that
/// body mentioning `list_topics` does not.
#[test]
fn the_drive_loop_never_enumerates_the_live_service_directory_itself() {
    let src = code_only(
        &std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/src/lib.rs"))
            .expect("read cerulion_bagd/src/lib.rs"),
    );
    let body = fn_body(&src, "drive_loop");

    // ANTI-TAUTOLOGY: the stripped body must still be the real loop. Without
    // this, a broken stripper (or a renamed function returning an empty body)
    // makes every absence assertion below vacuous.
    for present in ["drain_taps", "ensure_writer", "next_scan"] {
        assert!(
            body.contains(present),
            "the extracted `drive_loop` body must still contain `{present}` — the walk is reading \
             the wrong text, so its ABSENCE assertions prove nothing. Body:\n{body}"
        );
    }

    for forbidden in ["list_topics", "rescan_discovery"] {
        assert!(
            !body.contains(forbidden),
            "`drive_loop` must not call `{forbidden}`: enumerating the iceoryx2 service directory \
             costs 117.5 ms at 86 live topics (MEASURED on the Go2's Jetson) and every millisecond \
             of it is a millisecond no tap is drained — which overflowed the >= 200 Hz topics' \
             queues and lost 23 % of the firehose. The loop consumes `DiscoveryScanner::next_scan` \
             instead. Body:\n{body}"
        );
    }

    // The walk can SEE a real call when one exists — otherwise "absent from
    // `drive_loop`" would be satisfied by a probe that can never match anything.
    let arm_time = fn_body(&src, "rescan_discovery");
    assert!(
        arm_time.contains("list_topics"),
        "`rescan_discovery` is the ARM-TIME inline scan and must still enumerate — if this is \
         absent the probe string is wrong and the assertions above are vacuous. Body:\n{arm_time}"
    );
}
