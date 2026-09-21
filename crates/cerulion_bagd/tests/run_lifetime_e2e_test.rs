// SPDX-License-Identifier: AGPL-3.0-only
//! Run binding (C3) end-to-end: a bound recorder ENDS WITH ITS RUN.
//!
//! Every test builds an ISOLATED per-instance iceoryx2 transport
//! ([`common::make_manager`]) and publishes its run record on that SAME
//! namespace, so a watcher here sees exactly the runs this test created.
//!
//! # What these arms are for
//!
//! `cerulion_core`'s `run_lifetime_iox2_test` pins the WATCHER — that a run's
//! last word survives the run, and that a successor's records do not keep a
//! binding alive. It is blind to the only question that matters at the
//! recorder: does that verdict actually **finalize a bag**?
//!
//! The earlier recorder had no parent-death detection at all — no
//! `getppid`, no `PR_SET_PDEATHSIG`, no pipe EOF — and bagd is deliberately
//! spawned into its own process group so a terminal SIGINT cannot reach it. If
//! the parent was SIGKILLed, **bagd recorded forever**. So the load-bearing
//! assertion in every arm below is that `run_bagd` RETURNS with the shutdown
//! flag never set: nobody signalled it, and it stopped anyway.
//!
//! The UNBOUND control is what makes that mean something. Without it, "the
//! recorder finalized on its own" is satisfied by a recorder that finalizes
//! unconditionally — which would end every recording on its first pass.
//!
//! # Load discipline
//!
//! No arm states a wall. Verdicts and exact frame-set relations are the
//! assertions; every wait is a CONDITION under a generous seconds-scale
//! ceiling, which load can delay but not invert. The vanish grace is INJECTED
//! short so the arms test the rule rather than spend the shipped five seconds —
//! it is still an order of magnitude above the republish interval, so a healthy
//! run cannot fall through it.

mod common;

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_bag::BagReader;
use cerulion_bagd::{run_bagd, BagdConfig, BagdSummary, RunBinding, TapSpec};
use cerulion_core::transport::run_registry::{
    RunEnded, RunHandle, RunRecord, RunState, RunWatcher, RUN_REGISTRY_MAX_READERS,
};
use cerulion_core::transport::TransportManager;

use common::*;

/// An arbitrary, stable schema hash for the hand-built frames.
const HASH: u64 = 0x0981_0981_0981_0981;

/// Injected vanish grace: an order of magnitude above the 150 ms republish
/// interval, so a healthy run can never fall through it, while an arm that must
/// reach the verdict does not spend the shipped five seconds.
const TEST_GRACE: Duration = Duration::from_millis(1_500);

/// A hand-built run record.
fn run_record(run_id: u128, graph: &str) -> RunRecord {
    RunRecord {
        run_id,
        supervisor_pid: std::process::id(),
        run_started_at_ns: 1_753_000_000_000_000_000,
        state: RunState::Live,
        graph_name: graph.to_string(),
        run_dir: format!("/tmp/{graph}-{run_id:032x}"),
    }
}

/// Fast cadences, status OFF, discovery OFF (this file is about lifetime, not
/// coverage), optionally BOUND to a run.
fn cfg_for(
    out: std::path::PathBuf,
    taps: Vec<TapSpec>,
    ready: std::path::PathBuf,
    binding: Option<RunBinding>,
) -> BagdConfig {
    let mut cfg = BagdConfig::new(out, taps);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.ready_file = Some(ready);
    cfg.status_period = None;
    cfg.discover_live = false;
    cfg.schema_wait = Duration::from_millis(200);
    cfg.run_binding = binding;
    cfg
}

/// A binding to `run_id` under the injected grace.
fn bind(run_id: u128) -> RunBinding {
    RunBinding {
        run_id,
        vanish_grace: Some(TEST_GRACE),
    }
}

/// A publisher thread that publishes gap-free frames until told to stop,
/// counting what it committed. The COUNT is the oracle several arms compare
/// the bag against.
///
/// Every frame's body carries a RUN MARKER byte. Without it a bag recorded
/// across a restart holds two runs' frames that are byte-indistinguishable, and
/// NO assertion can attribute a recorded frame to either — which is exactly how
/// a splice arm can pass while 239 of its 250 recorded frames were
/// committed after the bound run died.
struct Producer {
    published: Arc<AtomicU32>,
    stop: Arc<AtomicBool>,
    join: Option<std::thread::JoinHandle<()>>,
}

impl Producer {
    fn start(mgr: Arc<TransportManager>, topic: String) -> Self {
        Self::start_marked(mgr, topic, 0)
    }

    /// A producer whose every frame body is `[marker; 8]` — the run this data
    /// belongs to, readable straight off the recorded payload.
    fn start_marked(mgr: Arc<TransportManager>, topic: String, marker: u8) -> Self {
        let published = Arc::new(AtomicU32::new(0));
        let stop = Arc::new(AtomicBool::new(false));
        let (p, s) = (Arc::clone(&published), Arc::clone(&stop));
        let join = std::thread::spawn(move || {
            let mut pubr = publisher(&mgr, &topic, 256);
            let mut seq = 0u32;
            while !s.load(Ordering::Relaxed) {
                let frame = build_frame(HASH, seq, u64::from(seq) + 1, &[marker; 8]);
                pubr.publish_raw(&frame).expect("publish");
                seq += 1;
                p.store(seq, Ordering::Relaxed);
                std::thread::sleep(Duration::from_millis(5));
            }
        });
        Self {
            published,
            stop,
            join: Some(join),
        }
    }

    fn count(&self) -> u32 {
        self.published.load(Ordering::Relaxed)
    }

    /// Stop the thread and RELEASE the publisher port — what a real graph
    /// teardown does, and what lets the successor's publisher attach to the
    /// single-writer topic.
    fn shut_down(mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }

    /// Block until at least `n` frames have been committed, or fail.
    fn wait_for(&self, n: u32, what: &str) {
        let deadline = Instant::now() + Duration::from_secs(20);
        while self.count() < n {
            assert!(
                Instant::now() < deadline,
                "{what}: the producer only committed {} of {n} frames within 20s",
                self.count()
            );
            std::thread::sleep(Duration::from_millis(10));
        }
    }
}

impl Drop for Producer {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

/// Messages actually in the FINALIZED bag, read back through the independent
/// reader. Asserts finalization in passing: a bound recorder that ends with its
/// run must leave a cleanly closed bag, never a torn tail (the torn-tail
/// readability is the CRASH net, not the exit path).
fn recorded(summary: &BagdSummary, topic: &str) -> u64 {
    recorded_by_marker(summary, topic).values().sum()
}

/// Recorded frames on `topic`, BROKEN DOWN by the run marker in their body —
/// which run's producer committed each one.
///
/// This is what makes attribution assertable at all. The recorded payload is
/// the wire frame verbatim, so the marker byte the producer stamped is still
/// there to be read.
fn recorded_by_marker(summary: &BagdSummary, topic: &str) -> std::collections::BTreeMap<u8, u64> {
    let path = summary.bag_paths.first().expect("one bag file");
    let reader = BagReader::open(path).expect("the bag must be readable");
    let (msgs, completeness) = reader.recover_messages().expect("recover_messages");
    assert!(
        completeness.is_finalized(),
        "a run ending must drive the GRACEFUL finalize path, got {completeness:?}"
    );
    let mut by_marker = std::collections::BTreeMap::new();
    for m in msgs.into_iter().filter(|m| m.topic == topic) {
        // The body follows the 32-byte wire header; every frame this file
        // publishes carries eight identical marker bytes.
        let marker = *m.data.get(32).unwrap_or_else(|| {
            panic!(
                "a recorded frame must carry its body: {} bytes",
                m.data.len()
            )
        });
        *by_marker.entry(marker).or_insert(0) += 1;
    }
    by_marker
}

/// The durable run-binding report the bag carries — the artifact an operator
/// reads, and the one every reporting assertion below is made against.
fn binding_of(summary: &BagdSummary) -> cerulion_bagd::RunBindingCoverage {
    summary
        .record_coverage
        .run_binding
        .clone()
        .expect("a BOUND recording must stamp its run binding into the bag")
}

/// Let the recorder's watcher actually HEAR the run before an arm kills it.
///
/// The recorder looks every `RUN_OBSERVE_INTERVAL` (250 ms) and its FIRST look
/// happens microseconds after the watcher's subscriber opened, so it routinely
/// finds an empty queue and first contact lands on the SECOND look. An arm that
/// kills its run inside that gap is not testing a binding at all — it is testing
/// the never-heard degrade, and every attribution assertion in it is vacuous
/// (killing the run inside that gap reports
/// `successor_seen: false` while 239 successor frames sat in the bag).
///
/// A LOWER-bound wait: load can only make it longer, and the run stays alive
/// throughout, so it cannot invert anything. The arms that use it also ASSERT
/// first contact happened, off the bag's own report — see
/// `assert_binding_was_established`.
const OBSERVE_SETTLE: Duration = Duration::from_millis(900);

/// The binding really was ESTABLISHED: the recorder heard its run at a moment
/// when frames were already in the bag.
///
/// Read off the durable report rather than timed, and it is what makes the
/// attribution assertions non-vacuous — a never-heard binding reports the whole
/// recording as unattributed, which trivially satisfies "every successor frame
/// is inside the unattributed window".
fn assert_binding_was_established(
    binding: &cerulion_bagd::RunBindingCoverage,
    total_recorded: u64,
    what: &str,
) {
    assert!(
        !binding.never_heard,
        "{what}: PRECONDITION — the recorder must have HEARD its run, or this arm is testing the \
         never-heard degrade and its attribution assertions are vacuous: {binding:?}"
    );
    assert!(
        binding.unattributed_frames < total_recorded,
        "{what}: PRECONDITION — first contact must have landed AFTER frames were already \
         recorded, or 'inside the unattributed window' means 'anywhere in the bag': \
         unattributed={} of {total_recorded} recorded, {binding:?}",
        binding.unattributed_frames
    );
}

/// Join a recorder that is expected to end BY ITSELF, under a generous
/// seconds ceiling.
///
/// A bare `join()` is an UNBOUNDED cross-thread wait, and this file's whole
/// premise is that the recorder terminates on its own — so a regression that
/// breaks the binding would HANG here instead of failing, burning a CI job to
/// its timeout with nothing attributable in the log. That is the exact class
/// already closed in `shm_ring_test`, and the same fix applies:
/// the ceiling is liveness-only (seconds against milliseconds of work), so load
/// can delay it but not invert it.
fn join_self_terminating(
    handle: std::thread::JoinHandle<Result<BagdSummary, cerulion_bagd::BagdError>>,
    shutdown: &Arc<AtomicBool>,
    what: &str,
) -> BagdSummary {
    let deadline = Instant::now() + Duration::from_secs(30);
    while !handle.is_finished() {
        assert!(
            Instant::now() < deadline,
            "{what}: the recorder was BOUND to a run that ended, and must have finalized itself \
             within 30s. It is still recording — nothing signalled it (shutdown={}), so it would \
             record forever, which is the defect this arm exists to catch",
            shutdown.load(Ordering::Relaxed)
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    handle
        .join()
        .expect("the recorder thread must not panic")
        .expect("a run ending is not a recorder failure — this must be a CLEAN finalize")
}

/// **THE headline (C3-T1).** The run VANISHES — no announcement, the shape a
/// SIGKILLed graph leaves — and the recorder finalizes ITSELF.
///
/// The load-bearing assertion is that `run_bagd` returned with the shutdown flag
/// NEVER set. Before run binding this recorder had no parent-death detection of any
/// kind and would have recorded until the test timed out.
#[test]
fn a_recorder_bound_to_a_vanished_run_finalizes_itself_and_exits_clean() {
    let mgr = make_manager(64);
    let topic = unique_topic("bind_vanish");
    let out = unique_out("bind_vanish");
    let ready = out.with_extension("ready");
    let run_id = 0x0981_0001_0000_0000_0000_0000_0000_0001;

    let run = RunHandle::publish_on_config(&mgr.iox_config(), run_record(run_id, "go2_attach"))
        .expect("run");
    let producer = Producer::start(Arc::clone(&mgr), topic.clone());
    producer.wait_for(3, "the vanish arm");

    // NEVER set. If the recorder returns, it is because the RUN ended.
    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = cfg_for(
        out.clone(),
        vec![TapSpec::attach(&topic)],
        ready.clone(),
        Some(bind(run_id)),
    );
    let recorder = {
        let (mgr, shutdown) = (Arc::clone(&mgr), Arc::clone(&shutdown));
        std::thread::spawn(move || run_bagd(mgr, cfg, shutdown))
    };
    assert!(
        wait_for_file(&ready, Duration::from_secs(20)),
        "the recorder must arm"
    );
    producer.wait_for(10, "the vanish arm after arming");

    // The run dies without a word.
    drop(run);

    let summary = join_self_terminating(recorder, &shutdown, "the vanish arm");

    assert!(
        !shutdown.load(Ordering::Relaxed),
        "nothing ever signalled this recorder — it must have stopped because its RUN did"
    );
    assert_eq!(
        summary.run_ended,
        Some(RunEnded::Vanished),
        "a run that stopped publishing without announcing is vanished, never graceful"
    );
    // The producer is STILL running, so the recorder did not stop for want of
    // data — without this the arm passes on a recorder that exits when a topic
    // goes quiet.
    assert!(
        producer.count() > 10,
        "PRECONDITION: the producer must still be publishing at the moment the recorder returned"
    );
    assert!(
        recorded(&summary, &topic) > 0,
        "everything recorded up to the boundary is KEPT — a bound recorder finalizes, it does not \
         discard"
    );
    drop(producer);
    cleanup(&out);
}

/// The graceful twin: the run ANNOUNCES its end (what `RunDescriptor::drop`
/// does), and the recorder reports `Graceful` rather than `Vanished`.
///
/// This is the arm the long-lived subscriber exists for: the announcement is one
/// frame, sent microseconds before the writer is released, and a recorder that
/// polled a fresh gather would report `Vanished` here — a plausible-looking answer
/// that is simply wrong about what happened.
#[test]
fn a_recorder_bound_to_a_gracefully_ended_run_reports_graceful() {
    let mgr = make_manager(64);
    let topic = unique_topic("bind_graceful");
    let out = unique_out("bind_graceful");
    let ready = out.with_extension("ready");
    let run_id = 0x0981_0002_0000_0000_0000_0000_0000_0002;

    let run = RunHandle::publish_on_config(&mgr.iox_config(), run_record(run_id, "go2_attach"))
        .expect("run");
    let producer = Producer::start(Arc::clone(&mgr), topic.clone());
    producer.wait_for(3, "the graceful arm");

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = cfg_for(
        out.clone(),
        vec![TapSpec::attach(&topic)],
        ready.clone(),
        Some(bind(run_id)),
    );
    let recorder = {
        let (mgr, shutdown) = (Arc::clone(&mgr), Arc::clone(&shutdown));
        std::thread::spawn(move || run_bagd(mgr, cfg, shutdown))
    };
    assert!(
        wait_for_file(&ready, Duration::from_secs(20)),
        "the recorder must arm"
    );
    producer.wait_for(10, "the graceful arm after arming");

    // A CONTROLLED stop: announce, then go.
    run.set_ending();
    drop(run);

    let summary = join_self_terminating(recorder, &shutdown, "the graceful arm");

    assert!(!shutdown.load(Ordering::Relaxed), "nothing signalled it");
    assert_eq!(
        summary.run_ended,
        Some(RunEnded::Graceful),
        "the run's LAST WORD must reach the recorder — a fresh-subscriber poll would have called \
         this a crash"
    );
    assert!(recorded(&summary, &topic) > 0);
    drop(producer);
    cleanup(&out);
}

/// **THE ANTI-TAUTOLOGY, and the arm every other one in this file leans on.**
///
/// The SAME stimulus with NO binding: the run vanishes and the recorder keeps
/// recording, because nobody asked it to end with anything. It stops only when
/// it is signalled, and reports no outcome at all.
///
/// Without this, "the recorder finalized itself" is satisfied by a recorder that
/// finalizes unconditionally.
#[test]
fn an_unbound_recorder_ignores_the_same_run_ending_entirely() {
    let mgr = make_manager(64);
    let topic = unique_topic("bind_unbound");
    let out = unique_out("bind_unbound");
    let ready = out.with_extension("ready");
    let run_id = 0x0981_0003_0000_0000_0000_0000_0000_0003;

    let run = RunHandle::publish_on_config(&mgr.iox_config(), run_record(run_id, "go2_attach"))
        .expect("run");
    let producer = Producer::start(Arc::clone(&mgr), topic.clone());
    producer.wait_for(3, "the unbound arm");

    let shutdown = Arc::new(AtomicBool::new(false));
    // The ONLY difference from the headline arm.
    let cfg = cfg_for(
        out.clone(),
        vec![TapSpec::attach(&topic)],
        ready.clone(),
        None,
    );
    let recorder = {
        let (mgr, shutdown) = (Arc::clone(&mgr), Arc::clone(&shutdown));
        std::thread::spawn(move || run_bagd(mgr, cfg, shutdown))
    };
    assert!(
        wait_for_file(&ready, Duration::from_secs(20)),
        "the recorder must arm"
    );
    producer.wait_for(10, "the unbound arm after arming");

    let before = producer.count();
    run.set_ending();
    drop(run);

    // Give it far longer than the bound arms need to reach a verdict, then
    // require it to be STILL RECORDING.
    std::thread::sleep(TEST_GRACE * 2);
    assert!(
        !recorder.is_finished(),
        "an UNBOUND recorder must not end with a run it was never bound to"
    );
    assert!(
        producer.count() > before,
        "PRECONDITION: the producer kept publishing across the window"
    );

    // It stops when — and only when — it is signalled.
    shutdown.store(true, Ordering::Relaxed);
    let summary = join_self_terminating(recorder, &shutdown, "the unbound arm after its signal");
    assert_eq!(
        summary.run_ended, None,
        "an unbound recording makes NO claim about any run — not even the one that ended under it"
    );
    assert!(recorded(&summary, &topic) > 0);
    drop(producer);
    cleanup(&out);
}

/// **The C3-T2 scenario, rebuilt so it can fail on what it is named
/// for.**
///
/// The operator forgets to stop the recorder, the first run dies, and the graph
/// is RESTARTED onto the same (prefix-frozen) topics. Run A's producer is
/// SHUT DOWN and run B's takes over the single-writer port, so A-frames and
/// B-frames are distinguishable in the bag by the marker in their bodies.
///
/// **Why two producers, and why the markers.** ONE continuous producer
/// across both runs makes every recorded frame byte-identical in provenance,
/// so no assertion can attribute one to either run; and closing
/// assertions read AFTER the self-terminating join prove nothing, because "the bag did
/// not grow" is true of any finalized file. That shape was measured with 239 of 250
/// recorded frames committed after the bound run died — inside a PASSING arm.
///
/// What is asserted is ATTRIBUTION, and the claim is the accurate one:
/// a crash is detected by SILENCE, so the
/// recorder cannot avoid draining a window it does not yet know is over. Every
/// B-frame in this bag must therefore be ACCOUNTED FOR by the bag's own
/// `run_binding` report, and the bag must not read clean.
#[test]
fn a_restarted_graphs_frames_are_accounted_for_rather_than_silently_spliced() {
    const RUN_A: u8 = 0xAA;
    const RUN_B: u8 = 0xBB;

    let mgr = make_manager(64);
    let topic = unique_topic("bind_restart");
    let out = unique_out("bind_restart");
    let ready = out.with_extension("ready");
    let first_id = 0x0981_0004_0000_0000_0000_0000_0000_0004;
    let second_id = 0x0981_0005_0000_0000_0000_0000_0000_0005;

    let first = RunHandle::publish_on_config(&mgr.iox_config(), run_record(first_id, "go2_attach"))
        .expect("first run");
    let producer_a = Producer::start_marked(Arc::clone(&mgr), topic.clone(), RUN_A);
    producer_a.wait_for(3, "the restart arm");

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = cfg_for(
        out.clone(),
        vec![TapSpec::attach(&topic)],
        ready.clone(),
        Some(bind(first_id)),
    );
    let recorder = {
        let (mgr, shutdown) = (Arc::clone(&mgr), Arc::clone(&shutdown));
        std::thread::spawn(move || run_bagd(mgr, cfg, shutdown))
    };
    assert!(
        wait_for_file(&ready, Duration::from_secs(20)),
        "the recorder must arm"
    );
    producer_a.wait_for(10, "the restart arm after arming");
    // Let the binding be ESTABLISHED before killing the run — otherwise this
    // arm silently degrades into the never-heard case (asserted below).
    std::thread::sleep(OBSERVE_SETTLE);

    // The first run dies — record AND producer, which is what a real teardown
    // does and what frees the single-writer port — and the graph is restarted
    // AT ONCE. The successor is live and republishing its own record for the
    // whole decision window, and publishing its OWN data onto the same topic.
    drop(first);
    producer_a.shut_down();
    let _second =
        RunHandle::publish_on_config(&mgr.iox_config(), run_record(second_id, "go2_attach"))
            .expect("restarted run");
    let producer_b = Producer::start_marked(Arc::clone(&mgr), topic.clone(), RUN_B);
    producer_b.wait_for(5, "the successor's own frames");

    let summary = join_self_terminating(recorder, &shutdown, "the restart arm");
    assert!(!shutdown.load(Ordering::Relaxed), "nothing signalled it");
    assert_eq!(
        summary.run_ended,
        Some(RunEnded::Vanished),
        "the binding is to the FIRST run — a successor's records are another run's traffic, not \
         evidence this one is alive"
    );

    let by_marker = recorded_by_marker(&summary, &topic);
    let a_frames = by_marker.get(&RUN_A).copied().unwrap_or(0);
    let b_frames = by_marker.get(&RUN_B).copied().unwrap_or(0);
    let binding = binding_of(&summary);

    // PRECONDITIONS, so the assertions below cannot pass vacuously.
    assert!(
        a_frames > 0,
        "the BOUND run's own data must be kept — a bound recorder finalizes, it does not discard \
         (markers: {by_marker:?})"
    );
    assert!(
        producer_b.count() >= 5,
        "PRECONDITION: the successor must really have published its own frames while the \
         recorder was still running"
    );
    assert_binding_was_established(&binding, a_frames + b_frames, "the restart arm");

    // THE ATTRIBUTION CLAIM. Whatever of the successor's data reached this bag,
    // the bag ITSELF must account for it: those frames landed after the bound
    // run was last heard, and the report says how many that is.
    assert!(
        b_frames <= binding.unattributed_frames,
        "every SUCCESSOR frame in this bag must fall inside the window the bag reports as \
         unattributable — {b_frames} B-frames against an unattributed_frames of {}. A frame \
         outside that window is a SILENT splice, which is the whole defect. binding={binding:?} \
         markers={by_marker:?}",
        binding.unattributed_frames
    );

    // …and if any DID land, the bag must not read clean about it. This is the
    // half a recorder that stamps no report fails outright: there is
    // nothing to account with and nothing to escalate.
    if b_frames > 0 {
        assert!(
            binding.successor_seen,
            "the recorder recorded {b_frames} frames of a SUCCESSOR and must have SEEN one: \
             {binding:?}"
        );
        assert!(
            binding.is_ambiguous(),
            "frames of another run landed in this bag, so its tail is ambiguous: {binding:?}"
        );
        assert!(
            summary.record_coverage.is_incomplete(),
            "a bag whose tail may be another run's must never print a clean summary: {:?}",
            summary.record_coverage
        );
    }

    // The frames committed AFTER the recorder returned still cannot be in an
    // already-finalized bag — kept from the original arm, now as a bound on the
    // OTHER side of the boundary.
    let at_join = producer_b.count();
    producer_b.wait_for(at_join + 20, "the restarted run's post-join frames");
    let after = recorded_by_marker(&summary, &topic);
    assert_eq!(
        after, by_marker,
        "the finalized bag must not have grown by one frame after the recorder left"
    );

    producer_b.shut_down();
    cleanup(&out);
}

/// The CONTROL for the arm above, and the reason its escalation is not
/// cry-wolf: the same crash with NOTHING taking over.
///
/// A run that vanished with no successor leaves a tail that can only be its own,
/// so the bag reports the window FAITHFULLY (the frames are still counted, because
/// silence is still how the boundary was found) and does NOT escalate. Without
/// this, "an ambiguous tail escalates" is satisfied by escalating every
/// crash-terminated recording — which would train an operator to skim the line
/// that matters.
#[test]
fn a_crash_with_nothing_replacing_it_reports_its_window_without_crying_wolf() {
    const RUN_A: u8 = 0xAA;

    let mgr = make_manager(64);
    let topic = unique_topic("bind_alone");
    let out = unique_out("bind_alone");
    let ready = out.with_extension("ready");
    let run_id = 0x0981_0006_0000_0000_0000_0000_0000_0006;

    let run = RunHandle::publish_on_config(&mgr.iox_config(), run_record(run_id, "go2_attach"))
        .expect("run");
    let producer = Producer::start_marked(Arc::clone(&mgr), topic.clone(), RUN_A);
    producer.wait_for(3, "the alone arm");

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = cfg_for(
        out.clone(),
        vec![TapSpec::attach(&topic)],
        ready.clone(),
        Some(bind(run_id)),
    );
    let recorder = {
        let (mgr, shutdown) = (Arc::clone(&mgr), Arc::clone(&shutdown));
        std::thread::spawn(move || run_bagd(mgr, cfg, shutdown))
    };
    assert!(
        wait_for_file(&ready, Duration::from_secs(20)),
        "the recorder must arm"
    );
    producer.wait_for(10, "the alone arm after arming");
    std::thread::sleep(OBSERVE_SETTLE);

    // The run dies. NOTHING replaces it — the producer keeps going, which is
    // what a `graph run` whose supervisor was SIGKILLed leaves behind.
    drop(run);

    let summary = join_self_terminating(recorder, &shutdown, "the alone arm");
    assert_eq!(summary.run_ended, Some(RunEnded::Vanished));

    let binding = binding_of(&summary);
    assert!(
        !binding.successor_seen,
        "nothing announced itself as a successor here: {binding:?}"
    );
    assert!(
        !binding.is_ambiguous(),
        "a crash with nothing replacing it leaves a tail that can only be the bound run's — \
         escalating it would cry wolf on every crash-terminated recording: {binding:?}"
    );
    assert!(
        !summary.record_coverage.is_incomplete(),
        "…and so the recording's verdict stays clean: {:?}",
        summary.record_coverage
    );
    // The window is still REPORTED, because it is still how the boundary was
    // found — reporting is not conditional on escalation.
    assert!(
        binding.unattributed_frames > 0,
        "a silence-detected boundary always has frames behind it (the producer never stopped): \
         {binding:?}"
    );
    assert_eq!(
        binding.run_id, run_id,
        "the bag names the run it is bound to"
    );
    assert!(recorded(&summary, &topic) > 0);

    producer.shut_down();
    cleanup(&out);
}

/// **The FOURTH `run_ended` reading, which had no arm at all: BOUND, but the
/// RECORDER was signalled first.**
///
/// The run is alive and stays alive; the recorder is stopped. `run_ended` is
/// `None` — and that `None` means something completely different from the
/// unbound one: this recording DID look, and makes no claim about a run that is
/// very likely still executing. The bag says which of the two it is, because a
/// summary field cannot.
#[test]
fn a_bound_recorder_signalled_before_its_run_claims_nothing_about_that_run() {
    let mgr = make_manager(64);
    let topic = unique_topic("bind_signalled");
    let out = unique_out("bind_signalled");
    let ready = out.with_extension("ready");
    let run_id = 0x0981_0007_0000_0000_0000_0000_0000_0007;

    // The run OUTLIVES the recorder — held to the end of the test.
    let run = RunHandle::publish_on_config(&mgr.iox_config(), run_record(run_id, "go2_attach"))
        .expect("run");
    let producer = Producer::start(Arc::clone(&mgr), topic.clone());
    producer.wait_for(3, "the signalled arm");

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = cfg_for(
        out.clone(),
        vec![TapSpec::attach(&topic)],
        ready.clone(),
        Some(bind(run_id)),
    );
    let recorder = {
        let (mgr, shutdown) = (Arc::clone(&mgr), Arc::clone(&shutdown));
        std::thread::spawn(move || run_bagd(mgr, cfg, shutdown))
    };
    assert!(
        wait_for_file(&ready, Duration::from_secs(20)),
        "the recorder must arm"
    );
    producer.wait_for(10, "the signalled arm after arming");
    // Let the binding be ESTABLISHED — a recorder signalled inside the first
    // observation or two would report `never_heard`, which is a DIFFERENT state
    // (and has its own arm).
    std::thread::sleep(OBSERVE_SETTLE);

    shutdown.store(true, Ordering::Relaxed);
    let summary = join_self_terminating(recorder, &shutdown, "the signalled arm");

    assert_eq!(
        summary.run_ended, None,
        "the recorder outlived nothing, so it must claim nothing about how the run ended"
    );
    let binding = binding_of(&summary);
    assert_eq!(
        binding.ended, None,
        "…and the bag says the same thing durably: {binding:?}"
    );
    assert!(
        !binding.watch_failed,
        "the watcher was fine — this is a recorder that was stopped, not one that could not \
         look: {binding:?}"
    );
    assert_eq!(binding.run_id, run_id);
    assert!(
        !binding.never_heard,
        "PRECONDITION: this arm is about a recorder that WAS bound and looked — the never-heard \
         degrade has its own arm: {binding:?}"
    );
    assert!(
        !binding.is_ambiguous(),
        "a recorder stopped under a LIVE run has no ambiguity to report: {binding:?}"
    );
    assert!(
        !summary.record_coverage.is_incomplete(),
        "…so its verdict stays clean: {:?}",
        summary.record_coverage
    );
    // The DISCRIMINATION this arm exists for: an UNBOUND recording's `None`
    // means "never looked", and the two must not read alike. An unbound bag
    // carries no binding at all.
    assert!(
        summary.record_coverage.run_binding.is_some(),
        "a bound recording that was stopped first must still be distinguishable from one that \
         was never bound"
    );

    assert!(recorded(&summary, &topic) > 0);
    drop(producer);
    drop(run);
    cleanup(&out);
}

/// A binding whose run the watcher NEVER HEARD dates NOTHING, and says so.
///
/// Driven deterministically by binding to a run id that was never published:
/// the watcher opens fine and simply never receives a record for it. The
/// recording is then entirely unattributed — and, critically, its
/// `successor_seen: false` proves nothing, because without first contact the
/// watcher does not know the run's graph name and could not recognise a
/// successor at all. That is why this state escalates on its own rather than
/// riding the successor term.
#[test]
fn a_binding_to_a_run_that_never_speaks_dates_nothing_and_says_so() {
    let mgr = make_manager(64);
    let topic = unique_topic("bind_unheard");
    let out = unique_out("bind_unheard");
    let ready = out.with_extension("ready");
    // Never published. Nothing on this isolated namespace will ever announce it.
    let ghost_id = 0x0981_0009_0000_0000_0000_0000_0000_0009;

    let producer = Producer::start(Arc::clone(&mgr), topic.clone());
    producer.wait_for(3, "the unheard arm");

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = cfg_for(
        out.clone(),
        vec![TapSpec::attach(&topic)],
        ready.clone(),
        Some(bind(ghost_id)),
    );
    let recorder = {
        let (mgr, shutdown) = (Arc::clone(&mgr), Arc::clone(&shutdown));
        std::thread::spawn(move || run_bagd(mgr, cfg, shutdown))
    };
    assert!(
        wait_for_file(&ready, Duration::from_secs(20)),
        "the recorder must arm even when its run is not there"
    );
    producer.wait_for(10, "the unheard arm after arming");

    // It ends itself: an unheard run reaches the vanished verdict through the
    // full silence grace ("the run died before we ever heard it").
    let summary = join_self_terminating(recorder, &shutdown, "the unheard arm");
    assert!(!shutdown.load(Ordering::Relaxed), "nothing signalled it");
    assert_eq!(summary.run_ended, Some(RunEnded::Vanished));

    let binding = binding_of(&summary);
    assert!(
        binding.never_heard,
        "the watcher opened and no record for this run ever arrived: {binding:?}"
    );
    assert!(
        !binding.watch_failed,
        "'opened and heard nothing' is a DIFFERENT state from 'could not open': {binding:?}"
    );
    let total = recorded(&summary, &topic);
    assert!(total > 0, "it recorded throughout");
    assert_eq!(
        binding.unattributed_frames, total,
        "with no first contact the WHOLE recording is the unattributed window: {binding:?}"
    );
    assert!(
        binding.is_ambiguous() && summary.record_coverage.is_incomplete(),
        "a binding that established nothing must not print a clean summary: {binding:?}"
    );

    producer.shut_down();
    cleanup(&out);
}

/// The `Vanished` DETECTION warn reports the SAME window the bag records — on
/// the `never_heard` path, which is where the two can contradict each other.
///
/// The terminal arms read their numbers off the manifest so a reader who greps
/// the log and a reader who opens the bag agree. This warn CANNOT: it fires the
/// instant the verdict latches, mid-drive-loop, before any `RecordCoverage`
/// exists. So it re-derived the value, and the copy had drifted — it omitted the
/// `.or(drive_start)` fallback and printed `unheard_for_ms=0` for a run that was
/// never heard, while the same run's manifest reported the whole recording. That
/// also made the warn's own sentence about "the frames of the last
/// `unheard_for_ms`" nonsense at 0.
///
/// The path is REACHABLE, not theoretical: `classify_run_signal`'s
/// silence-past-the-grace route is deliberately ungated on `heard_ever`
/// precisely so it covers a run that died before the watcher heard it, and the
/// sibling arm above already drives exactly this state on every run.
///
/// `run_bagd` runs on the TEST thread — the INVERSE of every other arm here —
/// because `tracing-test` scopes its capture to the test's span and a spawned
/// thread does not inherit it (with the usual harness the assertion would be
/// vacuous). Nothing needs to signal it: an unheard run self-terminates through
/// the grace, which is the whole point.
///
/// Load discipline: the discriminating assertion is `> 0`, which load cannot
/// touch. The ordering claim is a consequence of both readers measuring from
/// `drive_start` — the warn fires strictly BEFORE finalize, so the logged value
/// can only be the smaller of the two. Neither states a wall.
#[test]
#[serial_test::serial]
#[tracing_test::traced_test]
fn the_vanish_warn_reports_the_same_unheard_window_the_bag_records() {
    let mgr = make_manager(64);
    let topic = unique_topic("bind_warnwin");
    let out = unique_out("bind_warnwin");
    let ready = out.with_extension("ready");
    // Never published, exactly as in the sibling never-heard arm.
    let ghost_id = 0x0981_0009_0000_0000_0000_0000_0000_000a;

    let producer = Producer::start(Arc::clone(&mgr), topic.clone());
    producer.wait_for(3, "the vanish-warn window arm");

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = cfg_for(
        out.clone(),
        vec![TapSpec::attach(&topic)],
        ready.clone(),
        Some(bind(ghost_id)),
    );
    let summary = run_bagd(mgr, cfg, Arc::clone(&shutdown)).expect("clean finalize");
    assert!(!shutdown.load(Ordering::Relaxed), "nothing signalled it");

    // Preconditions: this run really did take the path the two readers disagreed
    // on. Without them the log assertion below could be satisfied by some other
    // arm entirely.
    assert_eq!(summary.run_ended, Some(RunEnded::Vanished));
    let binding = binding_of(&summary);
    assert!(binding.never_heard, "the path under test: {binding:?}");
    let manifest = binding.unheard_for_ms;
    assert!(
        manifest > 0,
        "the manifest half was always right, and is what the log must agree with: {binding:?}"
    );

    logs_assert(move |lines: &[&str]| {
        let warn = lines
            .iter()
            .find(|l| l.contains("stopped publishing without announcing an end"))
            .ok_or_else(|| format!("the vanish DETECTION warn must be emitted: {lines:?}"))?;
        // Whole-token match: `unheard_for_ms=0` is a prefix of `unheard_for_ms=0…`
        // for every value beginning with a zero digit, and the message names the
        // field in prose as well.
        let logged: u64 = warn
            .split_whitespace()
            .find_map(|t| t.strip_prefix("unheard_for_ms="))
            .ok_or_else(|| format!("the warn must carry its window as a field: {warn}"))?
            .parse()
            .map_err(|e| format!("`unheard_for_ms` must be a number ({e}): {warn}"))?;
        if logged == 0 {
            return Err(format!(
                "a run NEVER HEARD was unheard for the whole recording, not 0 ms — the bag \
                 records {manifest} ms for this very run, and the line's own sentence about \
                 'the frames of the last unheard_for_ms' is nonsense at 0: {warn}"
            ));
        }
        if logged > manifest {
            return Err(format!(
                "both readers measure from the start of the recording and the warn fires BEFORE \
                 finalize, so the logged window cannot exceed the recorded one: log={logged} \
                 manifest={manifest}"
            ));
        }
        Ok(())
    });

    producer.shut_down();
    cleanup(&out);
}

/// **A binding the recorder could not watch says so in the bag.**
///
/// `run_watch_failed` has exactly one production writer, and this arm pins it:
/// the unit arm HAND-SETS the bit and the table arm drives the pure classifier,
/// so without this arm flipping the assignment to `false` survives the whole suite
/// and a recording that could not watch its run reports as one that never
/// asked.
///
/// The failure is driven for real, through the shipped ceiling: a `RunWatcher`
/// holds one of `RUN_REGISTRY_MAX_READERS` subscriber slots for its whole life,
/// so occupying all of them makes the recorder's own open fail with the
/// transport's own error.
#[test]
fn a_binding_whose_watcher_cannot_be_opened_is_recorded_as_such() {
    let mgr = make_manager(64);
    let topic = unique_topic("bind_watchfail");
    let out = unique_out("bind_watchfail");
    let ready = out.with_extension("ready");
    let run_id = 0x0981_0008_0000_0000_0000_0000_0000_0008;

    let run = RunHandle::publish_on_config(&mgr.iox_config(), run_record(run_id, "go2_attach"))
        .expect("run");

    // Occupy every reader slot on this test's OWN registry namespace.
    let hogs: Vec<RunWatcher> = (0..RUN_REGISTRY_MAX_READERS)
        .map(|i| {
            RunWatcher::open_on_config(&mgr.iox_config(), run_id).unwrap_or_else(|e| {
                panic!("hog {i} must open (slots are {RUN_REGISTRY_MAX_READERS}): {e}")
            })
        })
        .collect();
    // PRECONDITION: the ceiling is genuinely reached, so the recorder below
    // really is being refused rather than merely happening to fail.
    assert!(
        RunWatcher::open_on_config(&mgr.iox_config(), run_id).is_err(),
        "the {RUN_REGISTRY_MAX_READERS} reader slots must be exhausted, or this arm proves nothing"
    );

    let producer = Producer::start(Arc::clone(&mgr), topic.clone());
    producer.wait_for(3, "the watch-fail arm");

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = cfg_for(
        out.clone(),
        vec![TapSpec::attach(&topic)],
        ready.clone(),
        Some(bind(run_id)),
    );
    let recorder = {
        let (mgr, shutdown) = (Arc::clone(&mgr), Arc::clone(&shutdown));
        std::thread::spawn(move || run_bagd(mgr, cfg, shutdown))
    };
    assert!(
        wait_for_file(&ready, Duration::from_secs(20)),
        "a recorder that cannot watch its run must still RECORD — a recording without a lifetime \
         binding is strictly better than no recording"
    );
    producer.wait_for(10, "the watch-fail arm after arming");

    // It cannot end with its run, so it ends when it is SIGNALLED. Killing the
    // run first proves the binding really is inert rather than merely slow.
    drop(run);
    std::thread::sleep(TEST_GRACE * 2);
    assert!(
        !recorder.is_finished(),
        "with no watcher there is nothing to end this recording but a signal"
    );
    shutdown.store(true, Ordering::Relaxed);
    let summary = join_self_terminating(recorder, &shutdown, "the watch-fail arm");

    let binding = binding_of(&summary);
    assert!(
        binding.watch_failed,
        "the bag must record that its binding could not be WATCHED — 'asked and could not' is a \
         different state from 'never asked', and only the bag outlives the log: {binding:?}"
    );
    assert_eq!(
        binding.ended, None,
        "with no watcher there is no outcome to report: {binding:?}"
    );
    assert!(
        binding.is_ambiguous(),
        "this bag's boundary says nothing about its run, so the whole recording is the ambiguous \
         window: {binding:?}"
    );
    assert!(
        summary.record_coverage.is_incomplete(),
        "…and it must not print a clean summary: {:?}",
        summary.record_coverage
    );
    assert!(recorded(&summary, &topic) > 0, "it recorded throughout");

    drop(hogs);
    producer.shut_down();
    cleanup(&out);
}

// ───────────────────────────────────────────────────────────────────────────
// Producer 3: a VANISHED run asks for a capture
// ───────────────────────────────────────────────────────────────────────────
//
// The arms above pin what a run ending does to the RECORDING (it finalizes).
// These pin what it does to the BLACK BOX, which is a different claim about the
// same observation: a monolith hard crash (SIGSEGV / OOM / abort) announces
// NOTHING, so the only thing that knows it happened is the run watcher this
// recorder was already driving, and by design that moment is one
// a robot should keep.
//
// Pre-window-only, adopted by design and asserted rather than
// implied: the vanish verdict makes that very drive pass a SHUTDOWN pass, so the
// loop exits within a pass or two and `resolve_flashback_at_shutdown` closes
// whatever is open. What the capture preserves is the seconds BEFORE the run
// stopped speaking. That is the right trade — a dead run's post-window is either
// silence or a SUCCESSOR's frames, and recording the latter would produce a bag
// that claims to be about run A while containing run B.

/// A capture directory, unique per arm.
fn capture_dir(tag: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "fb-{tag}-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0)
    ));
    std::fs::create_dir_all(&dir).expect("capture dir");
    dir
}

/// Every `.mcap` capture in `dir`, sorted.
fn captures(dir: &std::path::Path) -> Vec<std::path::PathBuf> {
    let mut out: Vec<std::path::PathBuf> = std::fs::read_dir(dir)
        .map(|r| {
            r.flatten()
                .map(|e| e.path())
                .filter(|p| p.extension().and_then(|e| e.to_str()) == Some("mcap"))
                .collect()
        })
        .unwrap_or_default();
    out.sort();
    out
}

/// A window-only flashback plane over `dir`, with `posture` deciding whether the
/// `RunVanished` row is on.
fn flashback_settings(
    dir: &std::path::Path,
    posture: cerulion_core::flashback::switch::TriggerPosture,
) -> cerulion_bagd::FlashbackSettings {
    cerulion_bagd::FlashbackSettings {
        window_span: Duration::from_secs(30),
        window_max_bytes: 64 * 1024 * 1024,
        anchor_max_bytes: 64 * 1024 * 1024,
        anchor_cap_basis: cerulion_core::flashback::CapBasis::Env,
        trace_max_bytes: cerulion_core::flashback::DEFAULT_FLASHBACK_TRACE_MAX_MB * 1024 * 1024,
        dir: dir.to_path_buf(),
        label: "run_lifetime".into(),
        caps: cerulion_core::flashback::retention::RetentionCaps::default(),
        policy: cerulion_core::flashback::trigger::TriggerPolicy {
            // Short, because this capture's post window is truncated by the
            // shutdown anyway (see the section note) — a long one would only
            // make the arm slower without changing what it proves.
            post_window_ns: 200 * 1_000_000,
            ..Default::default()
        },
        posture,
        exclude_topics: cerulion_core::flashback::ExcludeTopics::default(),
        window_only: true,
        tap_budget_bytes: cerulion_core::flashback::DEFAULT_FLASHBACK_TAP_BUDGET_MB * 1024 * 1024,
    }
}

/// The capture manifest of the single `.mcap` in `dir`.
fn capture_manifest(dir: &std::path::Path) -> serde_json::Value {
    let bags = captures(dir);
    assert_eq!(bags.len(), 1, "exactly one capture: {bags:?}");
    let reader = BagReader::open(&bags[0]).expect("the capture must be readable");
    let attachment = reader
        .attachment("__cerulion/flashback.json")
        .expect("attachment lookup")
        .expect("the capture manifest");
    let text = String::from_utf8(attachment.data).expect("utf-8");
    serde_json::from_str(&text)
        .unwrap_or_else(|e| panic!("the capture manifest must be valid JSON: {e}\n{text}"))
}

/// A bound recorder with a flashback plane, run until its run ends.
///
/// Returns the summary and the capture directory. Every arm below differs from
/// the others in exactly one input, so the comparison between them is the oracle.
fn run_until_run_ends(
    tag: &str,
    posture: cerulion_core::flashback::switch::TriggerPosture,
    end_gracefully: bool,
) -> (BagdSummary, std::path::PathBuf, u128) {
    let mgr = make_manager(64);
    let topic = unique_topic(tag);
    let out = unique_out(tag);
    let ready = out.with_extension("ready");
    let dir = capture_dir(tag);
    let run_id = 0x0981_0260_0000_0000_0000_0000_0000_0003;

    let run = RunHandle::publish_on_config(&mgr.iox_config(), run_record(run_id, "go2_attach"))
        .expect("run");
    let producer = Producer::start(Arc::clone(&mgr), topic.clone());
    producer.wait_for(3, tag);

    // NEVER set: if the recorder returns, it is because the RUN ended.
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = cfg_for(
        out.clone(),
        vec![TapSpec::attach(&topic)],
        ready.clone(),
        Some(bind(run_id)),
    );
    cfg.flashback = Some(flashback_settings(&dir, posture));
    let recorder = {
        let (mgr, shutdown) = (Arc::clone(&mgr), Arc::clone(&shutdown));
        std::thread::spawn(move || run_bagd(mgr, cfg, shutdown))
    };
    assert!(
        wait_for_file(&ready, Duration::from_secs(20)),
        "the recorder must arm"
    );
    // Enough frames that the window it holds is genuinely non-empty — a capture
    // of nothing would satisfy "a capture happened" without proving anything.
    producer.wait_for(10, tag);
    // OBSERVE_SETTLE: let the watcher HEAR the run before it ends, or this is
    // the never-heard degrade wearing a vanish arm's name.
    std::thread::sleep(OBSERVE_SETTLE);

    if end_gracefully {
        run.set_ending();
    }
    drop(run);

    let summary = join_self_terminating(recorder, &shutdown, tag);
    assert!(
        !shutdown.load(Ordering::Relaxed),
        "nothing signalled this recorder — it stopped because its RUN did"
    );
    drop(producer);
    cleanup(&out);
    (summary, dir, run_id)
}

/// **THE producer-3 headline.** A run that VANISHES leaves a capture, and the
/// capture's own manifest names `run_vanished` and what was observed.
///
/// The manifest is the oracle rather than a log line, for the reason every other
/// capture arm reads it: a capture is a durable artifact an operator opens
/// later, and a `tracing` line scrolls away.
#[test]
fn a_vanished_run_leaves_a_capture_naming_run_vanished() {
    let (summary, dir, run_id) = run_until_run_ends(
        "fb_vanish",
        cerulion_core::flashback::switch::TriggerPosture::default(),
        false,
    );

    assert_eq!(
        summary.run_ended,
        Some(RunEnded::Vanished),
        "PRECONDITION: the run must have VANISHED — a graceful end mints nothing, so this arm \
         would be vacuous"
    );
    // …and the recorder actually HEARD it first. `Vanished` alone does not say
    // that: a run the watcher never heard at all also ends in an Absent streak
    // and reports `Vanished`, so on a loaded box this arm would pass while
    // proving the never-heard degrade rather than a vanish. `OBSERVE_SETTLE`'s
    // own doc promises its callers assert this; these two did not.
    assert!(
        !binding_of(&summary).never_heard,
        "PRECONDITION: the recorder must have HEARD its run before it vanished, or this is the \
         never-heard degrade wearing a vanish arm's name: {:?}",
        binding_of(&summary)
    );

    let doc = capture_manifest(&dir);
    let text = doc.to_string();
    assert!(
        text.contains("run_vanished"),
        "the capture names the KIND that asked for it: {text}"
    );
    // The SUBJECT is the run id in the same `{:#034x}` spelling `run.json` and
    // every log line use — so an operator holding a capture and a run directory
    // can match them by eye.
    assert!(
        text.contains(&format!("{run_id:#034x}")),
        "the capture names the RUN it is about: {text}"
    );
    // …and the DETAIL carries what was observed, not merely that something was.
    assert!(
        text.contains("without announcing an end"),
        "the detail says HOW the run ended: {text}"
    );
    assert!(
        text.contains("unheard for"),
        "…and for how long it had been silent: {text}"
    );

    // The capture is a real bag with the window in it — a manifest over an empty
    // capture would satisfy every assertion above.
    let bags = captures(&dir);
    let reader = BagReader::open(&bags[0]).expect("readable");
    let (msgs, _) = reader.recover_messages().expect("recover_messages");
    assert!(
        !msgs.is_empty(),
        "the capture holds the window that was already retained — that is the whole feature"
    );

    let _ = std::fs::remove_dir_all(&dir);
}

/// **THE control.** A run that ends GRACEFULLY mints NOTHING.
///
/// One input apart from the headline, and it is the input that matters: a
/// controlled `graph run` shutdown is not an incident, and capturing every one
/// would be exactly the spam the design forbids. Without this
/// arm, "a capture appeared" is satisfied by a recorder that captures on every
/// exit.
#[test]
fn a_gracefully_ended_run_asks_for_no_capture() {
    let (summary, dir, _) = run_until_run_ends(
        "fb_graceful",
        cerulion_core::flashback::switch::TriggerPosture::default(),
        true,
    );

    assert_eq!(
        summary.run_ended,
        Some(RunEnded::Graceful),
        "PRECONDITION: the run must have announced its end, or this is the vanish arm again"
    );
    assert!(
        captures(&dir).is_empty(),
        "a controlled exit is not an incident: {:?}",
        captures(&dir)
    );

    let _ = std::fs::remove_dir_all(&dir);
}

/// **THE POSTURE GATE.** With `TriggerSwitch::RunVanished` switched OFF, the
/// same vanished run mints nothing.
///
/// The producer does NOT check the switch itself — `FlashbackTriggerGate::decide`
/// applies it, and a second copy in the recorder would be one policy in two
/// places. This arm is what makes that delegation checkable: if the mint
/// bypassed the gate, a switched-off row would still write a bag.
#[test]
fn the_run_vanished_posture_switch_refuses_the_capture() {
    let posture = cerulion_core::flashback::switch::TriggerPosture::default().with(
        cerulion_core::flashback::switch::TriggerSwitch::RunVanished,
        false,
    );
    let (summary, dir, _) = run_until_run_ends("fb_off", posture, false);

    assert_eq!(
        summary.run_ended,
        Some(RunEnded::Vanished),
        "PRECONDITION: the run really did vanish — the switch is the only difference from the \
         headline"
    );
    // This arm asserts an ABSENCE, so it needs the same non-vacuity guard the
    // headline does: a run the watcher never heard also reports `Vanished` and
    // also produces no capture, which would pass this while proving nothing
    // about the switch.
    assert!(
        !binding_of(&summary).never_heard,
        "PRECONDITION: the recorder must have HEARD its run, or the empty capture directory below \
         proves the never-heard degrade rather than the posture switch: {:?}",
        binding_of(&summary)
    );
    assert!(
        captures(&dir).is_empty(),
        "the operator turned this row off: {:?}",
        captures(&dir)
    );

    let _ = std::fs::remove_dir_all(&dir);
}

// ===========================================================================
// Producer 3 — the argv -> `BagdConfig::run_binding` WIRING
// ===========================================================================

/// The recorder really BINDS the run its argv named — a STRUCTURAL pin, because
/// no behavioural arm in this suite can reach the assignment.
///
/// # The gap this closes, stated at its real strength
///
/// `cerulion bagd --run-id <ID>` is `BagdConfig::run_binding`'s only production
/// writer. Every arm above sets `cfg.run_binding` DIRECTLY (that is how this
/// file's original nine arms have always worked), so a recorder that parsed the
/// flag and then never assigned it would leave all of them green while every
/// real `graph run` shipped an UNBOUND black box — the whole producer-3 feature
/// silently inert. The mutation dance found exactly that: "bagd parses
/// `--run-id` and discards it" survived the entire suite.
///
/// The VALUE half is already covered behaviourally: `parse_run_id`'s six oracle
/// vectors cover hex, decimal, blank and malformed, and
/// `graph_cmd::the_flashback_argv_is_accepted_by_the_recorder_it_is_built_for`
/// proves the flag crosses the spawn and the real derive accepts it. What was
/// unpinned is only the ASSIGNMENT, and that is what this walks for.
///
/// # Why STRUCTURAL rather than behavioural
///
/// A behavioural pin needs a `BagdArgs -> BagdConfig` seam, and there is none:
/// the assembly lives inside `bagd_cli_run`, which first initializes the global
/// transport and reads `--topics-json` / `--attach` / `--recorder-json` off
/// disk. Extracting it is a ~270-line move of live code that has not been
/// made, so no such seam exists.
/// A source walk is this repo's established answer for wiring no behavioural arm
/// can reach (`cdylib_iox2_log_level_test`'s init walk, `completions_test`'s
/// forbidden-call walk), and it is explicit about what it proves: that the
/// assignment EXISTS and reads `args.run_id`. It would not catch an assignment
/// that computed the wrong VALUE — which is what `parse_run_id`'s oracles are
/// for.
#[test]
fn the_recorder_binds_the_run_its_argv_named() {
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/src/lib.rs"))
        .expect("read the recorder's own source");
    let body = common::fn_body(&common::code_only(&src), "bagd_cli_run");

    // The ASSIGNMENT, as one whitespace-insensitive claim: `config.run_binding`
    // is set, and `args.run_id` is what it is set from. Both halves are
    // required — a `config.run_binding = None` keeps the first and drops the
    // second, and reading `args.run_id` into a local that is then dropped keeps
    // the second and loses the first.
    let flat: String = body.split_whitespace().collect::<Vec<_>>().join(" ");
    assert!(
        flat.contains("config.run_binding = args .run_id")
            || flat.contains("config.run_binding = args.run_id"),
        "`bagd_cli_run` must ASSIGN `config.run_binding` FROM `args.run_id` — without it \
         `cerulion bagd --run-id` parses the flag and discards it, every `graph run` ships an \
         UNBOUND black box, and every arm in this file stays green because they all set \
         `cfg.run_binding` by hand. Body walked:\n{body}"
    );

    // ANTI-TAUTOLOGY: the stripped, flattened view must still contain real code
    // this function certainly has. Without it, a `code_only` that ate everything
    // (or an `fn_body` that returned an empty string) would make the assertion
    // above unreachable rather than satisfied — and it would fail OPEN.
    assert!(
        flat.contains("BagdConfig::new"),
        "the stripped view of `bagd_cli_run` lost its own code, so the assertion above proves \
         nothing: {flat}"
    );
    // …and the walk must not be satisfiable by PROSE. Guard the MECHANISM, not a
    // phrase: asserting that a particular sentence from the
    // comment block above the assignment is absent can never
    // fire — the phrase spans a comment-LINE boundary, so flattening inserts
    // `//` between its words and it matches nothing whether or not comments were
    // stripped. All three assertions passed with `code_only` neutered to the
    // identity function, which is precisely the false safety signal a guard like
    // this exists to prevent.
    //
    // The mechanism is checkable directly: this function's body DOES carry
    // comments, and the stripped view must have none.
    let raw = common::fn_body(&src, "bagd_cli_run");
    let raw_flat: String = raw.split_whitespace().collect::<Vec<_>>().join(" ");
    assert!(
        raw_flat.contains("//"),
        "fixture check: `bagd_cli_run`'s body is expected to carry comments, or the stripper \
         assertion below proves nothing"
    );
    assert!(
        !flat.contains("//"),
        "`code_only` left comments in, so the wiring assertion above could be satisfied by the \
         doc comment that describes the assignment rather than by the code"
    );
}

/// A run-vanished request SURVIVES a refusal it can be re-offered from, and the
/// re-latch sits in the `RetryLater` BRANCH — not merely somewhere in the
/// function.
///
/// # The defect this excludes
///
/// A `service_flashback` that `take()`s the latched request and hands it to
/// `decide_flashback_request` unconditionally. That gate refuses — before the
/// trigger gate, deliberately — whenever a capture is still being WRITTEN, and
/// the refusal is transient. This latch is the ONLY copy of a run-vanished
/// request: nothing re-publishes it, because the thing that would have is the run
/// that just vanished. So a run that vanishes DURING another capture would lose its
/// RunVanished bag outright.
///
/// # Scoped to the branch
///
/// Asserting only that the token and the assignment each
/// appear SOMEWHERE in the function is too weak — it is satisfied by an implementation that
/// re-latches unconditionally, or on the wrong arm, which is the very mistake
/// that would flood the log with one warn per drive pass forever. The rule is
/// a pure fn (`should_relatch`) with its own oracle in `cerulion_bagd`'s unit
/// tests, and this walk extracts the BRACED BLOCK that follows
/// `if should_relatch(` and asserts the assignment is inside it.
///
/// # Why this is STRUCTURAL at all, said plainly
///
/// A behavioural arm has to make the writer BUSY at the exact moment the vanish
/// is observed. Half of what that needs exists: `CaptureJob::stall` (the seam
/// the shutdown-bound arm below uses) stalls a REAL capture
/// writer mid-write — `fault_inject_writer_stall_gate` only ever gated the
/// CONTINUOUS writer. What still blocks a behavioural arm is the OBSERVABLE:
/// the only evidence the refusal happened — `captures_deferred` — is written to
/// a log line rather than to `BagdSummary`, from a thread `#[traced_test]` does
/// not scope to, and racing a real capture against a real vanish grace without
/// that counter would be a wall that load can invert, which this repo forbids.
/// Until `captures_deferred` is surfaced on `BagdSummary`, this walk cannot become
/// a behavioural arm on top of the stall seam.
#[test]
fn a_vanished_runs_request_is_re_latched_when_the_writer_is_busy() {
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/src/lib.rs"))
        .expect("read the recorder's own source");
    let code = common::code_only(&src);
    let body = common::fn_body(&code, "service_flashback");

    // The BRACED BLOCK guarded by `should_relatch`, and nothing else.
    let branch = braced_block_after(&body, "if should_relatch(").unwrap_or_else(|| {
        panic!(
            "`service_flashback` must branch on `should_relatch(..)` — the rule is a pure fn so \
             both call sites share it and it can be oracle-tested. Body:\n{body}"
        )
    });
    let flat: String = branch.split_whitespace().collect::<Vec<_>>().join(" ");
    assert!(
        flat.contains("self.pending_run_vanished = Some(frame.request)"),
        "the request must be PUT BACK inside the `should_relatch` branch — nothing else holds a \
         copy of it, and re-latching outside the branch would re-offer a SUPPRESSED request on \
         every drive pass forever. Branch:\n{branch}"
    );

    // …and the other half of the rule: only
    // the BUSY refusal may map to `RetryLater`.
    //
    // This assertion is not redundant with the pure `should_relatch`
    // oracle: that pins what the RULE does with a disposition, while this pins
    // which disposition `decide_flashback_request` HANDS it. A suppressed or
    // rate-capped request returning `RetryLater` would be re-offered on every
    // drive pass, warning each time, forever — and the oracle cannot see that.
    let decide = common::fn_body(&code, "decide_flashback_request");
    let decide_flat: String = decide.split_whitespace().collect::<Vec<_>>().join(" ");
    assert_eq!(
        decide_flat
            .matches("RequestDisposition::RetryLater")
            .count(),
        1,
        "exactly ONE arm of `decide_flashback_request` may return `RetryLater` — the transient \
         writer-busy refusal. Every other exit (captured, extended, suppressed, a spent path \
         reservation, no plane) is a DECISION and must be `Settled`. Body:\n{decide}"
    );
    // …and it is the BUSY arm specifically, not merely some single arm: the
    // return must sit inside the block guarded by the `busy` check.
    let busy_branch = braced_block_after(&decide, "if busy").unwrap_or_else(|| {
        panic!("`decide_flashback_request` must still gate on `busy`. Body:\n{decide}")
    });
    assert!(
        busy_branch.contains("RequestDisposition::RetryLater"),
        "the single `RetryLater` must be the WRITER-BUSY arm — a different arm returning it \
         would re-offer a settled decision every drive pass. Branch:\n{busy_branch}"
    );

    // ANTI-TAUTOLOGY, both directions: the extractor really scopes (the whole
    // body contains code the branch does not), and the stripped view holds no
    // comments (so prose cannot satisfy the assertion above).
    let body_flat: String = body.split_whitespace().collect::<Vec<_>>().join(" ");
    assert!(
        body_flat.contains("drain_requests") && !flat.contains("drain_requests"),
        "fixture check: the extracted branch must be a STRICT SUBSET of the function — if it \
         returned the whole body, the scoping this arm exists for would be undone"
    );
    let raw_flat: String = common::fn_body(&src, "service_flashback")
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ");
    assert!(
        raw_flat.contains("//"),
        "fixture check: `service_flashback` is expected to carry comments"
    );
    assert!(
        !body_flat.contains("//"),
        "`code_only` left comments in, so the assertion above could be satisfied by prose"
    );
}

/// The brace-matched block that follows the first occurrence of `needle`.
///
/// Local to this arm because it is the only caller; `common::fn_body` matches on
/// a `fn` header and cannot scope to an `if`.
fn braced_block_after(src: &str, needle: &str) -> Option<String> {
    let at = src.find(needle)?;
    let open = src[at..].find('{').map(|i| i + at)?;
    let bytes = src.as_bytes();
    let mut depth = 0usize;
    for (idx, b) in bytes.iter().enumerate().skip(open) {
        match b {
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return Some(src[open..=idx].to_string());
                }
            }
            _ => {}
        }
    }
    None
}

/// A STALLED capture writer must not hold the drive loop open forever when the
/// bound run vanishes.
///
/// # The hang
///
/// The re-latch counts as pending work so the loop does not exit on the very pass
/// that latched it. But the latch is re-offered every pass and refused while a
/// capture is being WRITTEN — so if that writer stalls, the refusal is permanent
/// and the hold is unbounded. Everything that could rescue it runs AFTER the
/// loop: the bounded join that abandons the stalled writer, and the retry that
/// serves or counts the request. bagd would wait forever instead of finalizing.
///
/// This is the same shape as a window-less hang:
/// an exit predicate given a condition nothing in the loop can clear.
///
/// # The oracle
///
/// A capture is triggered and STALLED before it writes a byte; the bound run then
/// vanishes. The recorder must still return, and the loss must be COUNTED on the
/// summary rather than only logged — a log line is not something a caller can
/// act on. The join budget is spent once, so this arm costs about
/// `CAPTURE_SHUTDOWN_DEADLINE`; the assertion is a generous ceiling above it,
/// which load can delay but not invert.
#[test]
fn a_stalled_capture_writer_cannot_hold_shutdown_open_when_the_run_vanishes() {
    let tag = "fb_stall";
    let mgr = make_manager(64);
    let topic = unique_topic(tag);
    let out = unique_out(tag);
    let ready = out.with_extension("ready");
    let dir = capture_dir(tag);
    let run_id = 0x0981_0260_0000_0000_0000_0000_0000_0009;

    let run = RunHandle::publish_on_config(&mgr.iox_config(), run_record(run_id, "go2_attach"))
        .expect("run");
    let producer = Producer::start(Arc::clone(&mgr), topic.clone());
    producer.wait_for(3, tag);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = cfg_for(
        out.clone(),
        vec![TapSpec::attach(&topic)],
        ready.clone(),
        Some(bind(run_id)),
    );
    cfg.flashback = Some(flashback_settings(
        &dir,
        cerulion_core::flashback::switch::TriggerPosture::default(),
    ));
    // The capture writer will block before writing anything, and stay blocked.
    let gate = Arc::new(cerulion_bagd::WriterStallGate::default());
    gate.engaged.store(true, Ordering::Release);
    cfg.fault_inject_capture_stall_gate = Some(Arc::clone(&gate));

    let recorder = {
        let (mgr, shutdown) = (Arc::clone(&mgr), Arc::clone(&shutdown));
        std::thread::spawn(move || run_bagd(mgr, cfg, shutdown))
    };
    assert!(
        wait_for_file(&ready, Duration::from_secs(20)),
        "the recorder must arm"
    );
    producer.wait_for(10, tag);

    // Ask for a capture, and wait until its writer is provably STALLED — a
    // condition, not a sleep, so a slow runner delays this arm rather than
    // making it prove nothing.
    let requester = cerulion_core::flashback::channel::FlashbackRequester::open_on_manager(&mgr)
        .expect("requester");
    requester
        .request(&cerulion_core::flashback::trigger::CaptureRequest::manual(
            "stall the capture writer",
        ))
        .expect("request");
    let stalled_by = Instant::now() + Duration::from_secs(30);
    while Instant::now() < stalled_by && !gate.entered.load(Ordering::Acquire) {
        std::thread::sleep(Duration::from_millis(20));
    }
    assert!(
        gate.entered.load(Ordering::Acquire),
        "PRECONDITION: the capture writer must be STALLED, or this arm is not about the hang"
    );

    // …and NOW the bound run vanishes, so a run-vanished request is latched
    // against a writer that will never finish.
    std::thread::sleep(OBSERVE_SETTLE);
    drop(run);

    // THE PIN: it still returns. With an unbounded hold the loop spins forever and
    // this times out attributably rather than failing on a value.
    let started = Instant::now();
    let summary = join_self_terminating(recorder, &shutdown, tag);
    let elapsed = started.elapsed();
    assert!(
        !shutdown.load(Ordering::Relaxed),
        "nothing signalled this recorder — it stopped because its RUN did"
    );
    assert!(
        elapsed < cerulion_bagd::CAPTURE_SHUTDOWN_DEADLINE * 4,
        "the run-vanished latch must not hold the drive loop open past the writer's own bounded \
         deadline — took {elapsed:?}"
    );

    // …and the loss is COUNTED and loud, not silent. A stalled writer means no
    // run-vanished bag was made, and that is the one fact an operator cannot
    // infer from the bag's own absence.
    assert_eq!(
        summary.run_vanished_captures_lost, 1,
        "a run-vanished capture that could not be started must be COUNTED on the summary — \
         reporting it only in a log line is not something a caller can act on"
    );

    gate.engaged.store(false, Ordering::Release);
    drop(producer);
    cleanup(&out);
    let _ = std::fs::remove_dir_all(&dir);
}

/// A run-vanished capture ACCEPTED after the drive loop has exited must be
/// WRITTEN, not merely promised.
///
/// # The hole
///
/// The latch's hold on the loop is bounded. When that bound expires the loop
/// exits — and if the capture writer happens to finish around the same moment,
/// `resolve_flashback_at_shutdown` joins it cleanly and then offers the pending
/// run-vanished request to `decide_flashback_request` with the writer FREE. The
/// busy gate does not fire, so the request is ACCEPTED: a sequence is spent, a
/// path is reserved, and `FlashbackOutcome::Accepted` — naming that path — is
/// published to the requester.
///
/// Nothing then writes it. Steps (1) and (2) of the shutdown have already run,
/// the drive loop is gone, and no later pass calls `close_due_capture`. The
/// requester holds a promise, naming a file, for a bag that can never exist.
/// That is worse than the counted loss the busy path reports, because a counted
/// loss is TRUE.
///
/// # The oracle
///
/// TWO bags on disk, not one: the manual capture that made the writer busy, and
/// the run-vanished capture accepted at shutdown. If that capture is promised and
/// never written, only the first exists.
/// The second's manifest must name `run_vanished`, so the arm cannot pass on a
/// stray file, and nothing may be counted as lost — the request was served.
///
/// # Timing, and why load delays rather than inverts it
///
/// The release is scheduled for `RUN_VANISH_GRACE + CAPTURE_SHUTDOWN_DEADLINE`
/// plus slack, measured from the vanish — i.e. just AFTER the loop's bounded
/// hold expires, and well inside the up-to-`CAPTURE_SHUTDOWN_DEADLINE` window
/// step (2)'s join then spends waiting. The window is ~10s wide and the release
/// aims a few seconds into it. A late release simply lands later in that same
/// window. The one shape that would weaken the arm is loop exit being delayed
/// past the release, which turns it into the ordinary in-loop serve — the
/// assertions still hold, so it degrades to a weaker pass rather than a flake.
#[test]
fn a_run_vanished_capture_accepted_after_the_loop_exits_is_written_not_merely_promised() {
    let tag = "fb_accept";
    let mgr = make_manager(64);
    let topic = unique_topic(tag);
    let out = unique_out(tag);
    let ready = out.with_extension("ready");
    let dir = capture_dir(tag);
    let run_id = 0x0981_0260_0000_0000_0000_0000_0000_000a;

    let run = RunHandle::publish_on_config(&mgr.iox_config(), run_record(run_id, "go2_attach"))
        .expect("run");
    let producer = Producer::start(Arc::clone(&mgr), topic.clone());
    producer.wait_for(3, tag);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = cfg_for(
        out.clone(),
        vec![TapSpec::attach(&topic)],
        ready.clone(),
        Some(bind(run_id)),
    );
    cfg.flashback = Some(flashback_settings(
        &dir,
        cerulion_core::flashback::switch::TriggerPosture::default(),
    ));
    let gate = Arc::new(cerulion_bagd::WriterStallGate::default());
    gate.engaged.store(true, Ordering::Release);
    cfg.fault_inject_capture_stall_gate = Some(Arc::clone(&gate));

    let recorder = {
        let (mgr, shutdown) = (Arc::clone(&mgr), Arc::clone(&shutdown));
        std::thread::spawn(move || run_bagd(mgr, cfg, shutdown))
    };
    assert!(
        wait_for_file(&ready, Duration::from_secs(20)),
        "the recorder must arm"
    );
    producer.wait_for(10, tag);

    let requester = cerulion_core::flashback::channel::FlashbackRequester::open_on_manager(&mgr)
        .expect("requester");
    requester
        .request(&cerulion_core::flashback::trigger::CaptureRequest::manual(
            "occupy the capture writer",
        ))
        .expect("request");
    let stalled_by = Instant::now() + Duration::from_secs(30);
    while Instant::now() < stalled_by && !gate.entered.load(Ordering::Acquire) {
        std::thread::sleep(Duration::from_millis(20));
    }
    assert!(
        gate.entered.load(Ordering::Acquire),
        "PRECONDITION: the capture writer must be STALLED, or the run-vanished request is never \
         refused and this arm is not about the accept-after-exit hole"
    );

    std::thread::sleep(OBSERVE_SETTLE);
    drop(run);

    // Free the writer just AFTER the loop's bounded hold expires, so the request
    // is offered again with the writer provably idle.
    let releaser = {
        let gate = Arc::clone(&gate);
        std::thread::spawn(move || {
            std::thread::sleep(
                cerulion_core::transport::run_registry::RUN_VANISH_GRACE
                    + cerulion_bagd::CAPTURE_SHUTDOWN_DEADLINE
                    + Duration::from_secs(3),
            );
            gate.engaged.store(false, Ordering::Release);
        })
    };

    let summary = join_self_terminating(recorder, &shutdown, tag);
    let _ = releaser.join();
    assert!(
        !shutdown.load(Ordering::Relaxed),
        "nothing signalled this recorder — it stopped because its RUN did"
    );

    // THE PIN: the accepted capture reached DISK.
    let bags = captures(&dir);
    assert_eq!(
        bags.len(),
        2,
        "the run-vanished request was accepted after the drive loop exited, so its bag must have \
         been WRITTEN — publishing `Accepted` for a path nothing ever creates is a false promise, \
         which is worse than the counted loss the busy path reports. Bags: {bags:?}"
    );
    let named_run_vanished = bags.iter().any(|b| {
        BagReader::open(b)
            .ok()
            .and_then(|r| r.attachment("__cerulion/flashback.json").ok().flatten())
            .and_then(|a| String::from_utf8(a.data).ok())
            .is_some_and(|t| t.contains("run_vanished"))
    });
    assert!(
        named_run_vanished,
        "one of the two bags must be the RUN-VANISHED capture, so this cannot pass on a stray \
         file: {bags:?}"
    );
    assert_eq!(
        summary.run_vanished_captures_lost, 0,
        "the request was SERVED, so nothing may be counted lost"
    );

    drop(producer);
    cleanup(&out);
    let _ = std::fs::remove_dir_all(&dir);
}
