// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end: a recording missing the START of a topic SAYS so.
//!
//! Queue overflow BEFORE a tap's first drain discards a contiguous PREFIX, and
//! nothing a recording shipped could see it. The gap observer needs a
//! baseline frame to compare against, so its first observation carrying a
//! nonzero wire sequence is indistinguishable — to it — from a mid-stream
//! attach: `frames_lost = 0`, no gap event, an unchanged coverage manifest, and
//! a clean "recording complete".
//!
//! The headline arm builds exactly the shape the issue measured, deterministically:
//! the `fault_inject_tap_drain_gate` holds the recorder's drain from the FIRST
//! pass while a producer bursts [`BURST`] frames into a [`QUEUE`]-slot queue, so
//! iceoryx2 reclaims the head and the tap's first-ever drained frame carries
//! sequence `BURST - QUEUE`. Every control arm then takes something away — the
//! arm-ordering guarantee, the declared source, the on-time attach — and
//! requires the marker to VANISH, because it claims only what the wire proves.
//!
//! Isolated per-test SHM roots ([`common::make_manager`]); oracles are
//! hand-built wire frames and hand-written counts, never a self-compare.

mod common;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use cerulion_bag::writer::TopicSchema;
use cerulion_bagd::{
    run_bagd, BagdConfig, BagdError, BagdSummary, RecordCoverage, TapSource, TapSpec,
    DISCOVERY_RESCAN_INTERVAL,
};
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::WireHeader;

use common::*;

/// An arbitrary, stable schema hash for the hand-built frames.
const HASH: u64 = 0x0987_0987_0987_0987;
/// Frames the producer commits before the recorder drains anything.
const BURST: u32 = 400;
/// The topic's subscriber queue depth — the loss boundary since eviction was removed, and
/// deliberately far below [`BURST`] so the overflow is not a race.
const QUEUE: usize = 8;
/// What the wire therefore proves absent from the head of that stream.
const EXPECTED_PREFIX_LOST: u64 = (BURST as u64) - (QUEUE as u64);

fn spawn_bagd(
    mgr: Arc<TransportManager>,
    cfg: BagdConfig,
    shutdown: Arc<AtomicBool>,
) -> JoinHandle<Result<BagdSummary, BagdError>> {
    std::thread::spawn(move || run_bagd(mgr, cfg, shutdown))
}

fn schema_of(topic: &str) -> TopicSchema {
    TopicSchema {
        topic: topic.to_string(),
        schema_name: "probe_msgs/Probe".to_string(),
        schema_hash: HASH,
        wire_fixed_size: 8,
    }
}

/// The `graph run --record` config shape: EXACT-mode declared taps (so the bag
/// is created on drive-loop pass 1, as it is on that path) and the caller's
/// arm-ordering guarantee.
fn record_cfg(
    out: std::path::PathBuf,
    taps: Vec<TapSpec>,
    ready: std::path::PathBuf,
    armed_before_producers: bool,
) -> BagdConfig {
    let mut cfg = BagdConfig::new(out, taps);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.ready_file = Some(ready);
    cfg.status_period = None;
    cfg.schema_wait = Duration::from_millis(500);
    cfg.discover_live = false;
    cfg.armed_before_producers = armed_before_producers;
    cfg
}

fn oracle_body(tag: &str, i: u32) -> Vec<u8> {
    format!("{tag}-{i}").into_bytes()
}

fn oracle_frame(tag: &str, i: u32) -> Vec<u8> {
    build_frame(HASH, i, 1_000 + i as u64, &oracle_body(tag, i))
}

/// Read + parse `record_coverage.json` from a FINALIZED bag.
fn read_coverage(out: &std::path::Path) -> RecordCoverage {
    let reader = cerulion_bag::BagReader::open(out).expect("open bag");
    let att = reader
        .attachment(cerulion_bagd::RECORD_COVERAGE_ATTACHMENT)
        .expect("read attachments")
        .expect("record_coverage.json is present in EVERY finalized bag");
    let cov: RecordCoverage =
        serde_json::from_slice(&att.data).expect("record_coverage.json parses");
    assert_eq!(cov.version, cerulion_bagd::RECORD_COVERAGE_VERSION);
    cov
}

/// Recovered payloads for `topic`, in file order.
fn frames_for(out: &std::path::Path, topic: &str) -> Vec<Vec<u8>> {
    let reader = cerulion_bag::BagReader::open(out).expect("open bag");
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

/// The wire `sequence` of a recorded frame — the bag's own account of where
/// this topic's recording begins.
fn sequence_of(frame: &[u8]) -> u32 {
    WireHeader::read_from_buf(frame)
        .expect("a recorded frame carries a parseable wire header")
        .sequence
}

fn finish(
    handle: JoinHandle<Result<BagdSummary, BagdError>>,
    shutdown: &Arc<AtomicBool>,
) -> BagdSummary {
    shutdown.store(true, Ordering::Relaxed);
    handle.join().expect("bagd thread").expect("clean finalize")
}

// ===========================================================================
// THE HEADLINE — the measured shape, reported
// ===========================================================================

/// A producer that bursts past its queue before the recorder's first drain
/// leaves the bag beginning mid-stream, and the manifest SAYS so.
///
/// `frames_lost == 0` is asserted IN THE SAME BODY as the marker, because that
/// is precisely the reading this defect produced and it was TRUTHFUL: the gap
/// observer had no baseline to compare the first frame against, so there was no
/// gap for it to count. A recording can be complete by every number it publishes
/// and still be missing 392 of the 400 frames its producer committed.
#[test]
#[serial_test::serial]
fn a_burst_before_the_first_drain_is_reported_as_a_lost_prefix() {
    let mgr = make_manager(16);
    let topic = unique_topic("burst");
    let out = unique_out("burst");
    let ready = unique_out("burst_ready");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, QUEUE, QUEUE, 4096);

    let gate = Arc::new(AtomicBool::new(false));
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = record_cfg(
        out.clone(),
        vec![TapSpec::exact(&topic, schema_of(&topic))],
        ready.clone(),
        true,
    );
    cfg.fault_inject_tap_drain_gate = Some(gate.clone());
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    // The tap is ARMED and drains nothing: the producer's opening burst
    // overflows the queue, so its head is reclaimed before it is ever seen.
    for i in 0..BURST {
        pubr.publish_raw(&oracle_frame("b", i))
            .expect("publish_raw");
    }
    gate.store(true, Ordering::Relaxed);
    std::thread::sleep(Duration::from_millis(500));
    let summary = finish(handle, &shutdown);

    // (1) The recording's own numbers, exactly as the defect reported them.
    assert_eq!(
        summary.messages, QUEUE as u64,
        "only the newest {QUEUE} frames survived the producer's burst"
    );
    assert_eq!(
        summary.frames_lost, 0,
        "and frames_lost is TRUTHFUL at 0 — the gap observer needs a first frame \
         to compare against, so a loss BEFORE that frame is not a gap it can see. \
         This is the reading that made the defect silent."
    );
    assert_eq!(summary.dropped_unwritten, 0);
    assert_eq!(summary.headerless, 0);

    // (2) The marker names the loss, and its count agrees with the BAG's own
    //     first recorded frame (a cross-check the marker cannot fake).
    let coverage = read_coverage(&out);
    let entry = coverage.tapped.get(&topic).expect("the declared tap");
    assert_eq!(
        entry.prefix_lost,
        Some(EXPECTED_PREFIX_LOST),
        "the wire proves {EXPECTED_PREFIX_LOST} frames were committed before the first one \
         in the bag"
    );
    let frames = frames_for(&out, &topic);
    assert_eq!(frames.len(), QUEUE);
    assert_eq!(
        u64::from(sequence_of(&frames[0])),
        EXPECTED_PREFIX_LOST,
        "the marker must equal the sequence of the first frame actually in the bag"
    );
    // The surviving frames are the burst's TAIL, byte-identical to the oracle.
    let expected: Vec<Vec<u8>> = (BURST - QUEUE as u32..BURST)
        .map(|i| oracle_frame("b", i))
        .collect();
    assert_eq!(frames, expected);

    // (3) The escalation: this recording cannot present itself as complete.
    assert_eq!(
        coverage.gap_count(),
        0,
        "no live PRODUCER is missing — this producer IS in the bag, its head is not"
    );
    assert_eq!(coverage.prefix_lost_topics(), 1);
    assert_eq!(coverage.prefix_lost_total(), EXPECTED_PREFIX_LOST);
    assert!(
        coverage.is_incomplete(),
        "which is what escalates the recorder's terminal line to the WITH-ANOMALIES arm \
         (bagd_cli_run branches on exactly this predicate) and withholds `bag info`'s COMPLETE"
    );
    assert!(
        coverage.armed_before_producers,
        "and the manifest records the guarantee the claim rests on"
    );
    // The SAME guarantee is what the absorbance surface reads
    // to decide whether it must print the "a zero means nothing was COUNTED"
    // caveat. Asserted HERE because this is the only test in the repo that runs
    // a recorder with `armed_before_producers`, and without it the
    // `PrefixProven` arm of `Recorder::loss_counting_basis` is reached by
    // nothing — a variant hardcoding `PrefixInvisible` would survive the whole
    // suite and make an armed run print a caveat it has earned the right not to.
    assert_eq!(
        summary.record_health.loss_counting_basis,
        Some(cerulion_bagd::LossCountingBasis::PrefixProven),
        "an armed-before-producers recorder CAN account for a head loss, and its \
         health document must say so"
    );
    assert_eq!(
        summary.record_coverage, coverage,
        "summary == bag attachment"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

/// The operator-facing half: BOTH terminal lines say it. The head-loss warn
/// names the topic and the count, and the run's LAST line takes the
/// WITH-ANOMALIES arm carrying `prefix_lost` — the two lines an operator who
/// scrolls past everything else still sees.
///
/// `run_bagd` runs on the TEST thread with the stimulus on a helper — the
/// INVERSE of every other arm here — because `tracing-test` scopes its capture
/// to the test's span and a spawned thread does not inherit it (with the usual
/// harness the assertion would be vacuous).
#[test]
#[serial_test::serial]
#[tracing_test::traced_test]
fn a_truncated_recording_says_so_on_its_terminal_line() {
    let mgr = make_manager(16);
    let topic = unique_topic("loud");
    let out = unique_out("loud");
    let ready = unique_out("loud_ready");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, QUEUE, QUEUE, 4096);

    let gate = Arc::new(AtomicBool::new(false));
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = record_cfg(
        out.clone(),
        vec![TapSpec::exact(&topic, schema_of(&topic))],
        ready.clone(),
        true,
    );
    cfg.fault_inject_tap_drain_gate = Some(gate.clone());

    let ready_c = ready.clone();
    let gate_c = gate.clone();
    let shutdown_c = shutdown.clone();
    let stimulus = std::thread::spawn(move || {
        assert!(
            wait_for_file(&ready_c, Duration::from_secs(10)),
            "bagd ready"
        );
        for i in 0..BURST {
            pubr.publish_raw(&oracle_frame("l", i))
                .expect("publish_raw");
        }
        gate_c.store(true, Ordering::Relaxed);
        std::thread::sleep(Duration::from_millis(500));
        shutdown_c.store(true, Ordering::Relaxed);
    });
    let summary = run_bagd(mgr.clone(), cfg, shutdown).expect("clean finalize");
    stimulus.join().expect("stimulus thread");

    assert_eq!(
        summary.record_coverage.prefix_lost_total(),
        EXPECTED_PREFIX_LOST,
        "precondition: this run really did lose a prefix"
    );
    let expected_row = format!("{topic} ({EXPECTED_PREFIX_LOST})");
    logs_assert(|lines: &[&str]| {
        let heads: Vec<&&str> = lines
            .iter()
            .filter(|l| l.contains("TRUNCATED AT THE HEAD"))
            .collect();
        if heads.len() != 1 {
            return Err(format!(
                "expected exactly one head-loss warn, got {heads:?}"
            ));
        }
        if !heads[0].contains(&expected_row) {
            return Err(format!(
                "the warn must name the topic AND the frames the wire proves absent \
                 (expected `{expected_row}`), got {:?}",
                heads[0]
            ));
        }
        if !heads[0].contains("WARN") {
            return Err(format!("a proven loss must be LOUD, got {:?}", heads[0]));
        }

        // The run's LAST line. `frames_lost=0` on this run is TRUTHFUL and
        // says nothing about the 392 frames that never reached a baseline, so
        // the terminal line must carry the head-loss count BESIDE it — a
        // reader who sees only this line must not read the run as clean.
        let terminal: Vec<&&str> = lines
            .iter()
            .filter(|l| l.contains("bagd finalized WITH ANOMALIES"))
            .collect();
        if terminal.len() != 1 {
            return Err(format!(
                "the terminal line must take the WITH-ANOMALIES arm exactly once, got \
                 {terminal:?} (all lines: {lines:?})"
            ));
        }
        let expected_total = format!("prefix_lost={EXPECTED_PREFIX_LOST}");
        if !terminal[0].contains(&expected_total) {
            return Err(format!(
                "the terminal line must carry `{expected_total}`, got {:?}",
                terminal[0]
            ));
        }
        if !terminal[0].contains("frames_lost=0") {
            return Err(format!(
                "and it must still report the TRUTHFUL frames_lost=0 beside it — that pairing \
                 is the whole point of the field: {:?}",
                terminal[0]
            ));
        }
        Ok(())
    });

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ===========================================================================
// CONTROLS — the marker claims only what the wire proves
// ===========================================================================

/// CONTROL (a): a producer the RESCAN picked up mid-run gets no claim.
///
/// Its first recorded sequence is nonzero for a reason that is not loss the
/// recorder can attribute: the tap attached after the producer registered, so
/// coverage begins where the tap did (a data-only tap requests no back-fill).
/// Stamping `prefix_lost` here would turn a documented, expected boundary into
/// a reported defect on every dynamically-registered producer a `ros2 attach`
/// bridge creates — the very population live discovery exists to record.
///
/// The arm-ordering guarantee is ON, so this arm isolates the SOURCE/LATE
/// conjuncts rather than passing for the same reason control (c) does.
#[test]
#[serial_test::serial]
fn a_producer_the_rescan_picked_up_makes_no_prefix_claim() {
    let mgr = make_manager(16);
    let declared = unique_topic("declared");
    let late = unique_topic("late");
    let out = unique_out("late");
    let ready = unique_out("late_ready");

    // A silent DECLARED attach tap holds bag creation open (the deterministic
    // way to reach a rescan pickup — see its own late-producer arm).
    let mut declared_pub = publisher_with_provisioning(&mgr, &declared, 8, 16, 4096);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = record_cfg(
        out.clone(),
        vec![TapSpec::attach(&declared)],
        ready.clone(),
        true,
    );
    cfg.discover_live = true;
    cfg.schema_wait = Duration::from_millis(4_000);
    cfg.discovery_settle = DISCOVERY_RESCAN_INTERVAL * 4;
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    // A producer that appears AFTER the recorder armed and streams BEFORE any
    // rescan can find it, so its first recorded frame is nonzero. Published
    // back-to-back deliberately: nothing is draining them yet, and the tighter
    // this burst is, the smaller the window in which a rescan could land
    // between the publisher's creation and its frame 0 (~µs against the 250 ms
    // rescan cadence). Should one ever land there, the `first recorded sequence
    // > 0` precondition below fails LOUDLY rather than inverting a verdict.
    let mut late_pub = publisher_with_provisioning(&mgr, &late, 8, 16, 4096);
    for i in 0..6u32 {
        late_pub
            .publish_raw(&oracle_frame("t", i))
            .expect("publish");
    }
    // Now let the declared tap speak, so the bag can be created and finalized.
    declared_pub
        .publish_raw(&oracle_frame("d", 0))
        .expect("publish");
    std::thread::sleep(DISCOVERY_RESCAN_INTERVAL * 4);
    for i in 6..12u32 {
        late_pub
            .publish_raw(&oracle_frame("t", i))
            .expect("publish");
        settle();
    }
    std::thread::sleep(Duration::from_millis(300));
    let summary = finish(handle, &shutdown);

    let coverage = read_coverage(&out);
    let entry = coverage
        .tapped
        .get(&late)
        .expect("the rescan-discovered topic is in the manifest");
    assert_eq!(entry.source, TapSource::Discovered);
    assert!(
        entry.attached_late,
        "precondition: this arm is only meaningful for a tap that attached mid-run"
    );
    let frames = frames_for(&out, &late);
    assert!(
        !frames.is_empty() && sequence_of(&frames[0]) > 0,
        "precondition: the discovered topic's recording really does begin mid-stream \
         (first recorded sequence {:?})",
        frames.first().map(|f| sequence_of(f))
    );
    assert_eq!(
        entry.prefix_lost, None,
        "a tap that attached after its producer registered proves NO loss — coverage \
         simply begins where the tap did, which `attached_late` already says"
    );
    assert_eq!(coverage.prefix_lost_topics(), 0);
    assert_eq!(
        coverage
            .tapped
            .get(&declared)
            .expect("declared tap")
            .prefix_lost,
        None,
        "and the on-time declared tap recorded its producer from frame 0"
    );
    assert_eq!(
        summary.record_coverage, coverage,
        "summary == bag attachment"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

/// CONTROL (b), the ANTI-TAUTOLOGY arm: a recording that began at its
/// producer's first frame carries NO marker and stays clean.
///
/// Without it every assertion above is satisfied by a marker stamped on
/// everything, and `is_incomplete()` would escalate every recording ever made.
#[test]
#[serial_test::serial]
fn a_recording_that_began_at_the_producers_first_frame_carries_no_marker() {
    let mgr = make_manager(16);
    let topic = unique_topic("clean");
    let out = unique_out("clean");
    let ready = unique_out("clean_ready");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, QUEUE, QUEUE, 4096);

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = record_cfg(
        out.clone(),
        vec![TapSpec::exact(&topic, schema_of(&topic))],
        ready.clone(),
        true,
    );
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    // LOCKSTEP: at most one frame in flight, so nothing can overflow.
    for i in 0..6u32 {
        pubr.publish_raw(&oracle_frame("c", i))
            .expect("publish_raw");
        settle();
    }
    std::thread::sleep(Duration::from_millis(300));
    let summary = finish(handle, &shutdown);

    let coverage = read_coverage(&out);
    let entry = coverage.tapped.get(&topic).expect("the declared tap");
    assert_eq!(entry.frames_recorded, 6, "every frame was recorded");
    let frames = frames_for(&out, &topic);
    assert_eq!(
        sequence_of(&frames[0]),
        0,
        "precondition: the bag really does begin at the producer's first frame"
    );
    assert_eq!(
        entry.prefix_lost, None,
        "nothing is missing, so nothing is claimed — a marker of Some(0) would escalate \
         every healthy recording"
    );
    assert_eq!(coverage.prefix_lost_topics(), 0);
    assert_eq!(coverage.prefix_lost_total(), 0);
    assert!(
        !coverage.is_incomplete(),
        "and the run keeps its clean terminal line"
    );
    assert_eq!(summary.frames_lost, 0);

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

/// CONTROL (c), the FOREIGN-ORIGIN exclusion: a declared tap whose frames carry
/// a sequence origin the recorder never saw makes no claim.
///
/// `publish_raw` writes the caller's header VERBATIM, so a netd MIRROR of
/// another robot's topic carries the ORIGIN's counter — a first recorded
/// sequence of 5000 says nothing about this machine. The only thing that can
/// rule that out is a caller who controls when its producers start, which is
/// what `armed_before_producers` declares and what `cerulion bag record`
/// (attaching to whatever is already running) never claims.
#[test]
#[serial_test::serial]
fn a_declared_tap_with_no_arm_guarantee_makes_no_claim_about_a_foreign_sequence_origin() {
    let mgr = make_manager(16);
    let topic = unique_topic("foreign");
    let out = unique_out("foreign");
    let ready = unique_out("foreign_ready");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, QUEUE, QUEUE, 4096);

    let shutdown = Arc::new(AtomicBool::new(false));
    // The `cerulion bag record` shape: the recorder attached to a stream that
    // was already running, and claims no arm ordering.
    let cfg = record_cfg(
        out.clone(),
        vec![TapSpec::exact(&topic, schema_of(&topic))],
        ready.clone(),
        false,
    );
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    // A re-injected mirror's shape: the origin robot's counter, mid-stream.
    for i in 5_000..5_006u32 {
        pubr.publish_raw(&oracle_frame("f", i))
            .expect("publish_raw");
        settle();
    }
    std::thread::sleep(Duration::from_millis(300));
    let summary = finish(handle, &shutdown);

    let coverage = read_coverage(&out);
    let entry = coverage.tapped.get(&topic).expect("the declared tap");
    assert_eq!(
        sequence_of(&frames_for(&out, &topic)[0]),
        5_000,
        "precondition: the bag's first frame carries the FOREIGN origin's sequence"
    );
    assert_eq!(
        entry.prefix_lost, None,
        "5000 frames did not go missing on this machine — without the arm-ordering \
         guarantee a nonzero first sequence is not evidence of anything"
    );
    assert!(
        !coverage.armed_before_producers,
        "and the manifest says WHY no claim is made, so an absent marker is readable"
    );
    assert!(
        !coverage.is_incomplete(),
        "a run that could prove nothing must not be reported as truncated either"
    );
    assert_eq!(summary.frames_lost, 0);

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}
