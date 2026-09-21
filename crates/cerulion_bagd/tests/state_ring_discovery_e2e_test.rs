// SPDX-License-Identifier: AGPL-3.0-only
//! Rank discovery (C5) end-to-end: a recorder DISCOVERS a multi-process run's per-rank
//! node-state rings by name, and says which ranks it found.
//!
//! Under a `process_groups:` (or auto-derived) deployment the rank COUNT is
//! decided at run time, after the recorder's argv is fixed — so the per-rank ring
//! names cannot be listed on the command line. The two halves meet on a name both
//! DERIVE from `(arm tag, rank)`, and the recorder probes for it.
//!
//! Every arm here builds real `StateRingOwner`s under the derived names — exactly
//! what a graph process's `attach_state_capture_from` does — and drives the REAL
//! recorder. The records are hand-built by `cerulion_core`'s own `encode_record`
//! and compared byte-for-byte, so no arm can be satisfied by a recorder agreeing
//! with itself.
//!
//! What each arm is FOR:
//!
//! - the HEADLINE: three ranks, one armed BEFORE the recorder (the mid-run-attach
//!   shape) and two AFTER bag creation (the `graph run --record` shape, where a
//!   rank arms only once the recorder has released it) — all three land, which is
//!   the whole claim. Its anti-tautology half is in the same body: a recorder
//!   with NO arm tag discovers nothing even though the rings are right there;
//! - the HOLE: ranks are dense, so a missing rank below the highest is EVIDENCE a
//!   rank published nothing — and since a graph-wide anchor is all-or-nothing
//!   across ranks, that makes the recording incomplete;
//! - the SENTINEL: a ring sitting at the departure rank's derived name is
//!   never adopted, whatever else is armed.
//!
//! Isolated per-test SHM roots + unique tags ⇒ parallel-safe; `#[serial]` matches
//! its sibling e2e files.

mod common;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cerulion_bag::writer::TopicSchema;
use cerulion_bagd::{run_bagd, BagdConfig, BagdError, BagdSummary, StateCoverage, TapSpec};
use cerulion_core::state::StateSink;
use cerulion_core::state_arm::MappedStateArm;
use cerulion_core::state_ring::{
    encode_record, state_ring_tag, StateRecordHeader, StateRingOwner, RECORD_KIND_FINAL,
    STATE_RING_RESERVED_RANK,
};
use cerulion_core::transport::TransportManager;

use common::*;

const HASH: u64 = 0x0C1E_0C1E_0C1E_0C1E;
const RING_RECORDS: u32 = 256;

/// How long an arm waits for the recorder to notice a ring armed mid-run.
///
/// A LIVENESS bound, never a measurement: the sweep runs on the drive loop's
/// 250 ms discovery cadence, so this only decides how long a WEDGE takes to
/// report itself. Nothing here asserts on elapsed time.
const DISCOVERY_DEADLINE: Duration = Duration::from_secs(10);

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

fn base_cfg(out: std::path::PathBuf, topic: &str, ready: std::path::PathBuf) -> BagdConfig {
    let mut cfg = BagdConfig::new(out, vec![TapSpec::exact(topic, schema_of(topic))]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.ready_file = Some(ready);
    cfg.status_period = None;
    cfg.schema_wait = Duration::from_millis(500);
    cfg.discover_live = false;
    cfg
}

fn finish(
    handle: JoinHandle<Result<BagdSummary, BagdError>>,
    shutdown: &Arc<AtomicBool>,
) -> BagdSummary {
    shutdown.store(true, Ordering::Relaxed);
    handle.join().expect("bagd thread").expect("clean finalize")
}

fn state_records(out: &std::path::Path) -> Vec<Vec<u8>> {
    let reader = cerulion_bag::BagReader::open(out).expect("open bag");
    let (msgs, completeness) = reader.recover_messages().expect("recover_messages");
    assert!(
        completeness.is_finalized(),
        "bag must be Finalized, got {completeness:?}"
    );
    msgs.into_iter()
        .filter(|m| m.topic == cerulion_bag::STATE_TOPIC)
        .map(|m| m.data)
        .collect()
}

fn state_coverage(out: &std::path::Path) -> Option<StateCoverage> {
    let reader = cerulion_bag::BagReader::open(out).expect("open bag");
    let att = reader
        .attachment(cerulion_bagd::STATE_COVERAGE_ATTACHMENT)
        .expect("read attachments")?;
    Some(serde_json::from_slice(&att.data).expect("state_coverage.json parses"))
}

/// The graph's capture plane, as `arm_capture_plane` creates
/// it — created and armed by the party that owns the run, never by the recorder.
///
/// Returned so the caller BINDS it: the owner's drop unlinks the SHM name, and a
/// recorder that had not yet read the word would then read nothing and the bag
/// would make no cadence claim.
fn graph_plane(tag: &str, cadence_steps: u64) -> MappedStateArm {
    let arm = MappedStateArm::create_owned(tag).expect("create this run's capture plane");
    arm.arm(cadence_steps, 0);
    arm
}

/// One rank of a running graph: a state ring under the DERIVED `(tag, rank)`
/// name, holding its own producer — exactly the shape
/// `state_arm_attach::attach_state_capture_from` creates.
struct Rank {
    /// The SINGLE SPSC producer, minted once and held: `StateRingOwner::producer`
    /// yields `None` on every later call, so a rank that re-minted per push would
    /// silently stop writing after its first anchor.
    producer: cerulion_core::state_ring::StateRingProducer,
    /// Held so the ring's SHM name survives for the rank's lifetime — its drop
    /// unlinks it.
    _owner: StateRingOwner,
    rank: u32,
    run_id: u64,
}

impl Rank {
    fn arm(tag: &str, rank: u32, node_id: &str) -> Self {
        let run_id = cerulion_core::state_ring::state_ring_run_id(tag);
        let ring_tag = state_ring_tag(tag, rank).expect("an ordinary rank names a ring");
        let mut owner = StateRingOwner::create(&ring_tag, RING_RECORDS, rank, run_id, &[node_id])
            .expect("create this rank's state ring");
        let producer = owner.producer().expect("the single producer");
        Self {
            producer,
            _owner: owner,
            rank,
            run_id,
        }
    }

    /// Push warm-up anchors until the recorder has provably ADOPTED this ring,
    /// then push the ORACLE anchor and wait for that one to be consumed too.
    ///
    /// A discovered ring is opened at the LIVE cursor (`open_at_live`) — the only
    /// sound attach to a segment a live rank is writing into, since its head
    /// anchor is partial by construction. The consequence is that anchors pushed
    /// BEFORE the open are outside the recording's window, so a test that pushed
    /// once up front would be asserting on records the recorder was right to skip.
    ///
    /// The rendezvous is the ring's own read cursor: `free_records` returns to
    /// full capacity exactly when the consumer's cursor has caught up with the
    /// producer's, which happens on the adopting open (it skips) or on the first
    /// drain (it consumes). Either way, every push AFTER that is inside the
    /// window — so the oracle anchor is pushed second and waited for on its own.
    ///
    /// Bounded, and a LIVENESS bound only: nothing here asserts on elapsed time,
    /// and an unadopted ring fails loudly instead of hanging.
    fn pump_until_adopted_then_anchor(&mut self, deadline: Duration) -> Option<Vec<u8>> {
        let start = Instant::now();
        let mut adopted = false;
        while start.elapsed() < deadline {
            self.anchor(1);
            if self.await_caught_up(Duration::from_millis(250)) {
                adopted = true;
                break;
            }
        }
        if !adopted {
            return None;
        }
        let oracle = self.anchor(10);
        self.await_caught_up(deadline).then_some(oracle)
    }

    /// Has the consumer's read cursor caught up with everything pushed?
    fn await_caught_up(&mut self, deadline: Duration) -> bool {
        let start = Instant::now();
        while start.elapsed() < deadline {
            if self.producer.free_records().unwrap_or(0) >= u64::from(RING_RECORDS) {
                return true;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        false
    }

    /// One single-record anchor for this rank's only node at `step`, written onto the
    /// ring through the PRODUCTION sink. Returns the hand-built ORACLE bytes.
    ///
    /// Two halves, deliberately built by different code:
    ///
    /// * the ring is written by `StateRingProducer::sink` + `StateSink::write` +
    ///   `finish` — the exact chain a capture child's encoder drives (`BumpingSink`
    ///   wraps this same sink), so the framing under test is the one production
    ///   emits, chunking rule and all;
    /// * the value compared against it is assembled by hand from `encode_record` and
    ///   a literal `StateRecordHeader`.
    ///
    /// That split is what makes the read-back a CROSS-CHECK. A test that pushed
    /// hand-built bytes would prove the recorder can carry bytes; a test that
    /// compared the sink's output against the sink's output would prove nothing at
    /// all. Here the recorder must reproduce, byte for byte, a record the test
    /// predicted independently of the code that wrote it — so a change to the
    /// framing, the chunking boundary or the header layout fails HERE rather than
    /// silently agreeing with itself.
    fn anchor(&mut self, step: u64) -> Vec<u8> {
        let payload = vec![0xB0 | (self.rank as u8); 16];
        let mut sink = self.producer.sink(step, 0);
        sink.write(&payload)
            .expect("a ring with room accepts a 16-byte anchor");
        let parts = sink.finish();
        assert_eq!(
            parts, 1,
            "a 16-byte anchor is ONE record — if this ever changes the oracle below \
             is describing a different frame than the one on the ring"
        );
        encode_record(
            &StateRecordHeader {
                run_id: self.run_id,
                step,
                node_idx: 0,
                part: 0,
                kind: RECORD_KIND_FINAL,
                len: payload.len() as u32,
            },
            &payload,
        )
        .to_vec()
    }
}

// ===========================================================================
// THE HEADLINE
// ===========================================================================

/// Three ranks' rings are found by NAME — one already armed when the recorder
/// starts, two armed after the bag exists — and every rank's anchor is in the
/// bag with the ranks reported.
///
/// The two arming instants are the two production shapes, and they take DIFFERENT
/// code paths: a ring present at arm time is adopted by the drain side before the
/// writer thread takes the ring set, while one that appears later must be handed
/// ACROSS to the writer thread. A test that armed everything up front would
/// exercise only the first and leave the shipping `graph run --record` ordering —
/// where a rank arms only once the recorder has released it — unproven.
#[test]
#[serial_test::serial]
fn ranks_armed_before_and_after_bag_creation_are_all_discovered_by_name() {
    let mgr = make_manager(16);
    let topic = unique_topic("disc_disc");
    let out = unique_out("disc_disc");
    let ready = unique_out("disc_disc_ready");
    let tag = unique_ring_tag("c5disc");

    // The recorder ARMS the run: it creates the arm word, and the tag it names is
    // the only thing it will ever know about the deployment's shape.
    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_ring_discovery_tag = Some(tag.clone());
    // The graph owns the plane by design; the recorder only drains it.
    let _plane = graph_plane(&tag, 100);

    // Rank 0 is already up — the mid-run-attach shape.
    let mut r0 = Rank::arm(&tag, 0, "n0");

    let mut pubr = publisher(&mgr, &topic, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    // A data frame releases bag creation, so the writer thread now owns the ring
    // set and everything below takes the late-adoption path.
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");
    let o0 = r0
        .pump_until_adopted_then_anchor(DISCOVERY_DEADLINE)
        .expect("rank 0's ring was armed before the recorder started and must be adopted");

    // Ranks 1 and 2 arm LATE, as workers of a `graph run --record` deployment do.
    let mut r1 = Rank::arm(&tag, 1, "n1");
    let mut r2 = Rank::arm(&tag, 2, "n2");
    let o1 = r1
        .pump_until_adopted_then_anchor(DISCOVERY_DEADLINE)
        .expect("a rank that arms after bag creation must still be discovered");
    let o2 = r2
        .pump_until_adopted_then_anchor(DISCOVERY_DEADLINE)
        .expect("a rank that arms after bag creation must still be discovered");

    finish(handle, &shutdown);

    // (a) every rank's ORACLE anchor reached the bag VERBATIM. `contains`, not
    //     equality: each rank also pushed warm-up anchors to establish the
    //     rendezvous, and the recorder is right to have recorded whichever of
    //     those landed inside its window.
    let got = state_records(&out);
    for (rank, oracle) in [(0, &o0), (1, &o1), (2, &o2)] {
        assert!(
            got.contains(oracle),
            "rank {rank}'s anchor must reach the bag verbatim"
        );
    }

    // (b) the manifest names the ranks, and claims no hole.
    let cov = state_coverage(&out).expect("an armed recording carries state_coverage.json");
    assert_eq!(
        cov.ranks_discovered,
        vec![0, 1, 2],
        "the manifest must say WHICH ranks this recording's anchors came from"
    );
    assert!(cov.ranks_missing.is_empty());
    assert_eq!(
        cov.rings_declared, 0,
        "nothing was declared — every ring here was found by name"
    );
    assert_eq!(
        cov.armed.as_ref().map(|a| a.tag.as_str()),
        Some(tag.as_str())
    );
    for id in ["n0", "n1", "n2"] {
        assert!(
            cov.nodes[id].anchors_complete >= 1,
            "each rank's node must have anchored: {cov:?}"
        );
    }
    assert!(
        !cov.is_incomplete(),
        "a run whose every rank published is complete: {cov:?}"
    );
}

/// ANTI-TAUTOLOGY: a recorder told NO tag adopts nothing, with the same rings
/// sitting in `/dev/shm` under the same names — and a live, armed plane sitting
/// beside them.
///
/// Without this the headline could be satisfied by a recorder that adopts any
/// state ring it can find, which would silently pull a CO-TENANT run's anchors
/// into this bag. Graph-owned planes make that risk larger, not smaller: every
/// serving graph on the machine now publishes state rings, so "adopt what you
/// find" would mean "record whichever robot process happened to be running".
#[test]
#[serial_test::serial]
fn an_unarmed_recorder_adopts_nothing_even_with_the_rings_present() {
    let mgr = make_manager(16);
    let topic = unique_topic("disc_unarmed");
    let out = unique_out("disc_unarmed");
    let ready = unique_out("disc_unarmed_ready");
    let tag = unique_ring_tag("c5unarmed");

    // A fully live plane: armed word, ring, anchors. Everything a recorder that
    // adopted-what-it-found would happily take.
    let _plane = graph_plane(&tag, 100);
    let mut r0 = Rank::arm(&tag, 0, "n0");
    let _ = r0.anchor(10);

    // Same rings, same names, same armed word — and NO discovery tag.
    let cfg = base_cfg(out.clone(), &topic, ready.clone());
    let mut pubr = publisher(&mgr, &topic, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");
    // Long enough for several sweeps to have run had there been one to run.
    std::thread::sleep(Duration::from_millis(600));

    let summary = finish(handle, &shutdown);
    assert_eq!(
        summary.state_records, 0,
        "an unarmed recorder must not adopt a run it was not pointed at"
    );
    assert!(
        state_records(&out).is_empty(),
        "…and its bag must carry no state records"
    );
    assert!(
        state_coverage(&out).is_none(),
        "…and no state_coverage.json at all, so an ordinary bag stays byte-identical"
    );
}

// ===========================================================================
// THE HOLE
// ===========================================================================

/// A rank that exists and published NO ring is reported, and makes the recording
/// incomplete.
///
/// Ranks are dense, so rank 1 being absent while rank 2 is present is not missing
/// information — it is information. And because a graph-wide anchor is
/// all-or-nothing across ranks, that one rank voids every anchor of the
/// run, which is why it escalates rather than sitting in a field nobody reads.
#[test]
#[serial_test::serial]
fn a_rank_that_published_no_ring_is_reported_and_makes_the_recording_incomplete() {
    let mgr = make_manager(16);
    let topic = unique_topic("disc_hole");
    let out = unique_out("disc_hole");
    let ready = unique_out("disc_hole_ready");
    let tag = unique_ring_tag("c5hole");

    // Ranks 0 and 2 arm; rank 1 never does (its `attach_state_capture_from` failed —
    // loudly, on its own side — or its worker died before arming).
    let mut r0 = Rank::arm(&tag, 0, "n0");
    let mut r2 = Rank::arm(&tag, 2, "n2");

    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_ring_discovery_tag = Some(tag.clone());
    // The graph owns the plane by design; the recorder only drains it.
    let _plane = graph_plane(&tag, 100);
    let mut pubr = publisher(&mgr, &topic, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");
    let o0 = r0
        .pump_until_adopted_then_anchor(DISCOVERY_DEADLINE)
        .expect("rank 0 must be adopted");
    let o2 = r2
        .pump_until_adopted_then_anchor(DISCOVERY_DEADLINE)
        .expect("the sweep must walk PAST the hole — stopping at rank 1 would hide rank 2");

    finish(handle, &shutdown);
    let got = state_records(&out);
    assert!(got.contains(&o0) && got.contains(&o2));

    let cov = state_coverage(&out).expect("state_coverage.json");
    assert_eq!(
        cov.ranks_discovered,
        vec![0, 2],
        "the sweep must not stop at the first absent rank"
    );
    assert_eq!(
        cov.ranks_missing,
        vec![1],
        "a hole below the highest discovered rank is a rank that published nothing"
    );
    assert!(
        cov.is_incomplete(),
        "a missing rank voids every graph-wide anchor of the run, so the bag must say incomplete"
    );
}

// ===========================================================================
// THE SENTINEL
// ===========================================================================

/// A ring parked at the departure rank's derived name is NEVER adopted.
///
/// `u32::MAX` is the supervisor departure ring's header rank: it carries one
/// record per LOST WORKER and no node manifest, so there is no node in it whose
/// state could be checkpointed, and a state record stamped with it would make an
/// anchor's provenance indistinguishable from a worker departure. The pure half
/// (`state_ring_tag` refuses the rank outright) is in `cerulion_core`; this is
/// the behavioural half — a segment really sitting at that name, ignored.
#[test]
#[serial_test::serial]
fn a_ring_at_the_departure_sentinels_name_is_never_adopted() {
    let mgr = make_manager(16);
    let topic = unique_topic("disc_sent");
    let out = unique_out("disc_sent");
    let ready = unique_out("disc_sent_ready");
    let tag = unique_ring_tag("c5sent");

    // The name the sentinel rank WOULD derive, spelled by hand because
    // `state_ring_tag` refuses to spell it — which is the point.
    let sentinel_tag = format!("cer_st_{tag}_r{STATE_RING_RESERVED_RANK}");
    let run_id = cerulion_core::state_ring::state_ring_run_id(&tag);
    let mut sentinel =
        StateRingOwner::create(&sentinel_tag, RING_RECORDS, 0, run_id, &["ghost"]).expect("ring");
    let ghost = {
        let payload = vec![0xDE; 16];
        let record = encode_record(
            &StateRecordHeader {
                run_id,
                step: 10,
                node_idx: 0,
                part: 0,
                kind: RECORD_KIND_FINAL,
                len: payload.len() as u32,
            },
            &payload,
        );
        let mut p = sentinel.producer().expect("producer");
        // Pushed repeatedly, so "never adopted" cannot pass merely because the
        // one push landed before an adopting open would have skipped it.
        for _ in 0..8 {
            p.push_record(&record);
        }
        record.to_vec()
    };

    // A real rank 0 alongside it, so the arm is not vacuously passing on a
    // recorder that adopted nothing at all.
    let mut r0 = Rank::arm(&tag, 0, "n0");

    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_ring_discovery_tag = Some(tag.clone());
    // The graph owns the plane by design; the recorder only drains it.
    let _plane = graph_plane(&tag, 100);
    let mut pubr = publisher(&mgr, &topic, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");
    let oracle = r0
        .pump_until_adopted_then_anchor(DISCOVERY_DEADLINE)
        .expect("rank 0 adopted");

    finish(handle, &shutdown);
    let got = state_records(&out);
    assert!(got.contains(&oracle), "the real rank's anchor is recorded");
    assert!(
        !got.contains(&ghost),
        "a record from the sentinel-named ring must never reach the bag"
    );
    let cov = state_coverage(&out).expect("state_coverage.json");
    assert_eq!(cov.ranks_discovered, vec![0]);
    assert!(
        !cov.nodes.contains_key("ghost"),
        "the sentinel ring's manifest must never reach the bag: {cov:?}"
    );
    drop(sentinel);
}

// ===========================================================================
// The arm word is the SAME word for every rank
// ===========================================================================

/// Every rank opens the SAME arm word, so their cadence decisions agree by
/// ARITHMETIC — no message, no clock, no protocol.
///
/// This is the whole multi-process claim and it is the reason one tag is
/// stamped into every worker plan. Two independent mappings stand in for two
/// ranks (the mapping is `MAP_SHARED`, so two processes see exactly this), and
/// the agreement is asserted on EVERY step across the cadence, both before and
/// after the recorder arms — a rank that read a stale `first_anchor_step` would
/// anchor on a different step from its peers, which is precisely the divergence
/// the shared word exists to prevent.
#[test]
#[serial_test::serial]
fn two_ranks_reading_one_arm_word_agree_on_every_step_with_no_protocol() {
    let tag = unique_ring_tag("c5onset");
    let owner = MappedStateArm::create_owned(&tag).expect("the recorder creates the word");
    let rank_a = MappedStateArm::open_unowned(&tag).expect("rank a opens it");
    let rank_b = MappedStateArm::open_unowned(&tag).expect("rank b opens it");

    // Before the recorder arms, nobody anchors — and they agree about that too.
    for step in 0..64u64 {
        assert!(!rank_a.due(step) && !rank_b.due(step), "unarmed at {step}");
    }

    owner.arm(10, 3);
    // The hand oracle: armed with cadence 10 from onset 3, an anchor falls on
    // 3, 13, 23, … and nowhere else. Asserted against the ORACLE first, so the
    // agreement below cannot be two ranks being identically wrong.
    for step in 0..64u64 {
        let want = step >= 3 && (step - 3) % 10 == 0;
        assert_eq!(rank_a.due(step), want, "rank a at step {step}");
        assert_eq!(rank_b.due(step), want, "rank b at step {step}");
        assert_eq!(
            rank_a.due(step),
            rank_b.due(step),
            "two ranks must agree at step {step} with nothing passing between them"
        );
    }

    // A RE-ARM at a different onset moves both ranks together — the case a
    // per-rank cached onset would get wrong, and the reason the word publishes
    // the onset and the armed flag in one release-ordered pair.
    owner.arm(10, 7);
    for step in 0..64u64 {
        let want = step >= 7 && (step - 7) % 10 == 0;
        assert_eq!(rank_a.due(step), want, "rank a after re-arm at {step}");
        assert_eq!(rank_b.due(step), want, "rank b after re-arm at {step}");
    }

    // And a disarm stops both, which is what makes the recorder's finalize a
    // real stop rather than a request.
    owner.disarm();
    for step in 0..64u64 {
        assert!(
            !rank_a.due(step) && !rank_b.due(step),
            "disarmed at step {step}"
        );
    }
}

// ===========================================================================
// C6, the LATE ATTACH: a recorder that arms NOTHING
// ===========================================================================

/// A recorder that arms nothing, pointed only at a run's DERIVED tag, drains
/// that run's per-rank anchors.
///
/// # This is the whole point of the derivation
///
/// `cerulion bag record --run NAME` attaches to a graph that is ALREADY running
/// and already being checkpointed by somebody else. It cannot arm — the graph
/// opened its arm word long before this process existed — and without a DERIVED
/// tag it has no way even to NAME the state rings: a free-form tag is a string two
/// operators have to agree on before launch. Both halves compute it from the run's
/// `run_id`, which `run.json` carries, so a recorder that resolved a run has
/// everything it needs and coordinates with nobody.
///
/// The oracle is the anchor bytes, so a recorder that found the rings and
/// dropped their contents cannot pass.
///
/// Arming is graph-owned, and that decides the `armed` half of the oracle. It cannot
/// discriminate this test from the headline by being `None` — "this recorder set
/// no cadence" — because no recorder sets one. It states the cadence the
/// GRAPH armed, which this test pins by arming a plane at a distinctive cadence
/// and requiring the bag to report THAT number: a recorder that fabricated a
/// cadence, or reported its own default, fails.
#[test]
#[serial_test::serial]
fn a_recorder_that_arms_nothing_still_drains_a_runs_anchors_by_the_derived_tag() {
    let mgr = make_manager(16);
    let topic = unique_topic("derived_tag");
    let out = unique_out("derived_tag");
    let ready = unique_out("derived_tag_ready");

    // The run this recorder attaches to. Only its ID crosses — exactly what
    // `bag record --run` reads out of `run.json`.
    let run_id: u128 = 0x0c61_0530_0000_0000_0000_0000_dead_beef;
    let tag = cerulion_core::state_arm::state_arm_tag_for_run(run_id);

    // Two ranks of that run, already up and already publishing anchors.
    let mut r0 = Rank::arm(&tag, 0, "n0");
    let mut r1 = Rank::arm(&tag, 1, "n1");

    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    // NOTHING is armed. The ONLY thing this recorder is told is where to look —
    // and it derived that itself, from the run id.
    cfg.state_ring_discovery_tag = Some(tag.clone());
    // The GRAPH'S plane, at a cadence no default would produce.
    const RUN_CADENCE: u64 = 4_242;
    let _plane = graph_plane(&tag, RUN_CADENCE);

    let mut pubr = publisher(&mgr, &topic, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");

    let o0 = r0
        .pump_until_adopted_then_anchor(DISCOVERY_DEADLINE)
        .expect("rank 0's ring is found by the DERIVED tag with no coordination");
    let o1 = r1
        .pump_until_adopted_then_anchor(DISCOVERY_DEADLINE)
        .expect("rank 1's too — the derivation names every rank, not just the first");

    finish(handle, &shutdown);
    let got = state_records(&out);
    assert!(
        got.contains(&o0) && got.contains(&o1),
        "both ranks' anchors must reach the bag verbatim"
    );

    let cov = state_coverage(&out).expect("a recording that drained anchors says so");
    assert_eq!(cov.ranks_discovered, vec![0, 1]);
    assert!(cov.ranks_missing.is_empty());
    let armed = cov
        .armed
        .as_ref()
        .expect("the bag must state the plane whose anchors it holds");
    assert_eq!(armed.tag, tag, "…named by the tag it drained: {cov:?}");
    assert_eq!(
        armed.cadence_steps, RUN_CADENCE,
        "…and at the cadence the GRAPH armed, read off the word rather than \
         invented: {cov:?}"
    );
    assert_eq!(
        cov.rings_declared, 0,
        "nothing was declared — the tag was DERIVED and the rings found by name"
    );
}

// ===========================================================================
// THE ARMING WINDOW, THE LATE FAILURE, AND THE DECLARED SUBSET
// ===========================================================================

/// Create an SHM object under `name` and leave it UNSIZED — the exact state a ring
/// is in between its creator's `shm_open(O_CREAT|O_EXCL)` and its `ftruncate`.
///
/// That is not a synthetic shape: `ShmRingOwner::create_with_policy` passes through
/// it on every ring it makes, and the header magic is `Release`-stored later still.
/// A name in this state answers the sweep's existence probe and refuses every open.
fn plant_unpublished_shm_object(name: &str) {
    let c = std::ffi::CString::new(name).expect("nul-free");
    // SAFETY: FFI create of a named SHM object with a valid NUL-terminated name.
    let fd = unsafe {
        libc::shm_open(
            c.as_ptr(),
            libc::O_CREAT | libc::O_RDWR | libc::O_EXCL,
            0o600 as libc::c_uint,
        )
    };
    assert!(fd >= 0, "plant {name}: {}", std::io::Error::last_os_error());
    // SAFETY: closing the descriptor just opened. Deliberately NO ftruncate.
    unsafe { libc::close(fd) };
}

/// Best-effort unlink, so a planted object never outlives its test.
fn unlink_shm_object(name: &str) {
    if let Ok(c) = std::ffi::CString::new(name) {
        // SAFETY: FFI unlink of a name this test planted.
        unsafe { libc::shm_unlink(c.as_ptr()) };
    }
}

/// A rank that ARMS while a scan is in flight is still adopted.
///
/// A ring's SHM name exists from its creator's `shm_open(O_CREAT|O_EXCL)`; its header
/// magic is stored LAST. Between those instants the name answers the existence probe
/// and every open fails validation — and a rank arming after the recorder released it
/// is not a corner, it is the `graph run --record` shape this whole plane exists for.
///
/// Treating that first failure as TERMINAL put the name in the held set, so no later
/// sweep ever looked again: the rank's anchors were in NO bag for the life of the run,
/// and the bag reported the ring "unavailable" for a ring that was fine a millisecond
/// later.
///
/// Load pushes this test the SAFE way: a slower box completes FEWER scans inside the
/// window below, so it retries fewer times, never more.
#[test]
#[serial_test::serial]
fn a_ring_named_before_its_header_is_published_is_retried_not_written_off() {
    let mgr = make_manager(16);
    let topic = unique_topic("disc_arming");
    let out = unique_out("disc_arming");
    let ready = unique_out("disc_arming_ready");
    let tag = unique_ring_tag("c5arm");

    // Rank 0 is up from the start, so the run has a bag and a settled sweep.
    let mut r0 = Rank::arm(&tag, 0, "n0");

    // Rank 1's NAME appears first, with nothing published behind it.
    let r1_name = cerulion_core::state_ring::state_ring_shm_name(&tag, 1).expect("derived name");
    plant_unpublished_shm_object(&r1_name);

    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_ring_discovery_tag = Some(tag.clone());
    // The graph owns the plane by design; the recorder only drains it.
    let _plane = graph_plane(&tag, 100);
    let mut pubr = publisher(&mgr, &topic, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");
    let o0 = r0
        .pump_until_adopted_then_anchor(DISCOVERY_DEADLINE)
        .expect("rank 0 must be adopted");

    // The recorder has now swept rank 1's name at least once and failed to open it.
    // The creator finishes the job: `StateRingOwner::create` unlinks the orphan and
    // O_EXCL-creates a real ring under the SAME name, exactly as a worker does.
    let mut r1 = Rank::arm(&tag, 1, "n1");
    let o1 = r1
        .pump_until_adopted_then_anchor(DISCOVERY_DEADLINE)
        .expect(
            "a rank whose NAME was seen before its header was published must still be \
             adopted once it finishes arming — writing the name off on the first failed \
             open loses that rank's anchors for the whole run",
        );

    finish(handle, &shutdown);
    unlink_shm_object(&r1_name);

    let got = state_records(&out);
    assert!(got.contains(&o0), "rank 0's anchor must be in the bag");
    assert!(
        got.contains(&o1),
        "rank 1's anchor must be in the bag VERBATIM"
    );

    let cov = state_coverage(&out).expect("state_coverage.json");
    assert_eq!(
        cov.ranks_discovered,
        vec![0, 1],
        "both ranks were discovered by name"
    );
    assert!(
        cov.ranks_missing.is_empty(),
        "no hole: {:?}",
        cov.ranks_missing
    );
    assert!(
        cov.rings_unavailable.is_empty(),
        "a ring that was merely mid-create must not be reported unavailable: {:?}",
        cov.rings_unavailable
    );
    assert!(
        !cov.is_incomplete(),
        "every rank's anchors reached this bag, so it is complete"
    );
}

/// A ring discovered AFTER bag creation that never opens reaches the MANIFEST.
///
/// The recorder's coverage seed is taken when the writer thread is spawned. A failure
/// found after that was appended only to a recorder-side vector nothing read again, so
/// the rank's anchors were absent from the bag while `rings_unavailable` said nothing
/// — and with the unopenable rank ABOVE every rank that did open, no missing-rank
/// marker was raised either. An incomplete recording reported itself complete.
///
/// The wait below is a LIVENESS bound (the failure must persist past the retry budget,
/// four scans at the 250 ms discovery cadence ≈ 1 s), never a measurement.
#[test]
#[serial_test::serial]
fn a_late_ring_that_never_opens_is_reported_in_the_bags_coverage() {
    /// Comfortably past `STATE_RING_OPEN_RETRY_SCANS` scans at the discovery cadence.
    const PERSIST_FOR: Duration = Duration::from_secs(3);

    let mgr = make_manager(16);
    let topic = unique_topic("disc_late");
    let out = unique_out("disc_late");
    let ready = unique_out("disc_late_ready");
    let tag = unique_ring_tag("c5late");

    let mut r0 = Rank::arm(&tag, 0, "n0");

    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_ring_discovery_tag = Some(tag.clone());
    // The graph owns the plane by design; the recorder only drains it.
    let _plane = graph_plane(&tag, 100);
    let mut pubr = publisher(&mgr, &topic, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");
    // Adoption of rank 0 proves the bag exists and the writer owns the ring set —
    // so everything after this point takes the LATE path.
    let o0 = r0
        .pump_until_adopted_then_anchor(DISCOVERY_DEADLINE)
        .expect("rank 0 must be adopted");

    // Rank 1's name appears and NOTHING ever publishes behind it.
    let r1_name = cerulion_core::state_ring::state_ring_shm_name(&tag, 1).expect("derived name");
    plant_unpublished_shm_object(&r1_name);
    std::thread::sleep(PERSIST_FOR);

    finish(handle, &shutdown);
    unlink_shm_object(&r1_name);

    assert!(state_records(&out).contains(&o0));
    let cov = state_coverage(&out).expect("state_coverage.json");
    assert!(
        cov.rings_unavailable.contains_key(&r1_name),
        "a ring the recorder could not open must be named in the bag's coverage, \
         whether it was found before or after the bag existed: {:?}",
        cov.rings_unavailable
    );
    assert!(
        cov.is_incomplete(),
        "the recorder was asked for a rank's anchors and could not even look at them, \
         so this recording cannot report itself complete"
    );
}

/// An explicitly DECLARED `--state-ring` subset claims no rank holes.
///
/// `--state-ring` takes a hand-picked list of SHM object NAMES, and each ring's rank
/// comes from its own header — the operator never even sees it. Reading rank 1 out of
/// the one ring they named and concluding "rank 0 exists and published nothing" marks
/// the bag INCOMPLETE for a shape the operator chose.
///
/// The density argument needs a WALK: "rank k exists" is inferred from having swept
/// past it. A declared list swept nothing, so it can witness no hole.
#[test]
#[serial_test::serial]
fn an_explicitly_declared_ring_subset_is_never_read_as_a_rank_hole() {
    let mgr = make_manager(16);
    let topic = unique_topic("disc_decl");
    let out = unique_out("disc_decl");
    let ready = unique_out("disc_decl_ready");
    let tag = unique_ring_tag("c5decl");

    // ONLY rank 1, declared by name. No arm, no discovery tag — nothing sweeps.
    let mut r1 = Rank::arm(&tag, 1, "n1");
    let r1_name = cerulion_core::state_ring::state_ring_shm_name(&tag, 1).expect("derived name");

    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_rings = vec![r1_name.clone()];
    let mut pubr = publisher(&mgr, &topic, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");
    let oracle = r1.anchor(7);
    assert!(
        r1.await_caught_up(DISCOVERY_DEADLINE),
        "a declared ring is drained"
    );

    finish(handle, &shutdown);

    assert!(
        state_records(&out).contains(&oracle),
        "the declared ring's anchor must be in the bag"
    );
    let cov = state_coverage(&out).expect("state_coverage.json");
    assert_eq!(cov.rings_declared, 1, "one ring was declared");
    assert_eq!(
        cov.ranks_discovered,
        vec![1],
        "the bag still reports WHICH rank it drained — that claim is true"
    );
    assert!(
        cov.ranks_missing.is_empty(),
        "nothing walked a rank space here, so no hole can be proven: {:?}",
        cov.ranks_missing
    );
    assert!(
        !cov.is_incomplete(),
        "the operator asked for exactly this ring and got exactly it"
    );
}

/// The state-ring hand-off may never use a BLOCKING send.
///
/// `hand_to_writer_or_retain` is the single seam every discovered ring and every late
/// open-failure crosses to the writer thread, and it runs on the drive loop AHEAD of
/// `drain_taps`. A `send` there puts every tap's drain behind a momentarily-behind
/// writer and their SHM queues overflow — frames lost to a checkpoint-plane hand-off.
///
/// A structural walk rather than a behavioural one because the failure is the ABSENCE
/// of a bound: reproducing it needs the bounded channel genuinely full at the instant
/// a ring is discovered, and a test that engineers that is pinning its own timing
/// rather than the rule. What the rule IS, is visible in the source: this function
/// sends exactly once, and that send is `try_send`.
#[test]
fn the_state_ring_hand_off_never_blocks_on_the_writer_channel() {
    let src = std::fs::read_to_string(
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src/lib.rs"),
    )
    .expect("read cerulion_bagd/src/lib.rs");

    const FN: &str = "fn hand_to_writer_or_retain";
    let start = src.find(FN).expect(
        "the state-ring hand-off seam must still exist — if it was renamed, re-point \
         this walk rather than deleting it",
    );
    // The function body ends at the first line that closes it at method indentation.
    let rest = &src[start..];
    let end = rest
        .find("\n    }\n")
        .expect("the seam's body must be delimited at method indentation");
    let body = &rest[..end];

    assert!(
        body.contains("try_send("),
        "the hand-off must use the NON-BLOCKING send:\n{body}"
    );
    // `try_send(` contains no `.send(`, so this is not satisfied by the line above.
    assert!(
        !body.contains(".send("),
        "the hand-off must not contain a BLOCKING send — the drive loop runs it before \
         drain_taps, so waiting for channel room stops every tap draining:\n{body}"
    );
    // Anti-tautology: the walk really did read a body with sends in it, so the
    // absence assertion above is a fact about this function rather than about an
    // empty string a broken slice produced.
    assert!(
        body.contains("TrySendError::Full"),
        "the walk must have found the real body (it defers on a full channel):\n{body}"
    );
}

// ===========================================================================
// RETAINED HAND-OFFS MUST NOT BE STRANDED BY FINALIZATION
// ===========================================================================

/// A retained ADOPTION is delivered at finalization, and its anchors reach the bag.
///
/// The retry loop for a refused hand-off lives at the head of the discovery sweep, and
/// the sweep does not run again once shutdown is signalled. So a ring discovered while
/// the writer channel happened to be full was retained and then never retried: the
/// consumer was dropped with its anchors unrecorded, and coverage was built only from
/// the rings the writer already owned.
///
/// The retention branch is forced by a seam rather than by racing a genuinely full
/// one-slot channel — see the seam's own docs for why (the race's failure mode is a
/// GREEN test). What is not simulated is the part under test: finalization delivers
/// through the real `try_send`, and every assertion reads the real bag.
#[test]
#[serial_test::serial]
fn a_retained_ring_adoption_is_delivered_at_finalization_not_stranded() {
    let mgr = make_manager(16);
    let topic = unique_topic("disc_strand_a");
    let out = unique_out("disc_strand_a");
    let ready = unique_out("disc_strand_a_ready");
    let tag = unique_ring_tag("c5stra");

    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_ring_discovery_tag = Some(tag.clone());
    // The graph owns the plane by design; the recorder only drains it.
    let _plane = graph_plane(&tag, 100);
    // Every state-ring hand-off is refused for the whole run.
    cfg.fault_inject_state_ring_handoff_always_full = true;

    let mut pubr = publisher(&mgr, &topic, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    // Releases bag creation, so the writer thread owns the ring set and every later
    // adoption must cross the channel.
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");
    std::thread::sleep(Duration::from_millis(400));

    // The rank arms LATE, is discovered, and its adoption is retained every sweep.
    let mut r1 = Rank::arm(&tag, 1, "n1");
    // The ORACLE is pushed only after discovery has had time to open the ring. A
    // discovered ring is opened AT THE LIVE CURSOR (the only sound attach to a segment
    // a live rank is writing), so an anchor pushed before that open is deliberately
    // outside the recording's window — the recorder would be RIGHT to skip it, and
    // asserting on it would be asserting the wrong contract. Generous, and a LIVENESS
    // bound only: more load means fewer sweeps in the window, never more.
    std::thread::sleep(Duration::from_millis(1_500));
    let oracle = r1.anchor(11);
    std::thread::sleep(Duration::from_millis(500));

    finish(handle, &shutdown);

    assert!(
        state_records(&out).contains(&oracle),
        "the retained ring's anchor must reach the bag — finalization is the last \
         chance to hand it over, and nothing retries after shutdown"
    );
    let cov = state_coverage(&out).expect("state_coverage.json");
    assert_eq!(
        cov.ranks_discovered,
        vec![1],
        "and the bag must report the rank it drained: {cov:?}"
    );
}

/// A retained UNAVAILABLE-RING report is delivered at finalization, so the bag still
/// reads INCOMPLETE.
///
/// This is the sharper half. `rings_unavailable` is the field that makes a recording
/// with unreadable anchors report itself incomplete; strand the report and a bag with
/// a missing rank reads COMPLETE — the exact inversion the manifest exists to prevent.
#[test]
#[serial_test::serial]
fn a_retained_unavailable_ring_report_is_delivered_at_finalization() {
    /// Comfortably past `STATE_RING_OPEN_RETRY_SCANS` scans at the discovery cadence,
    /// so the failure becomes TERMINAL (and therefore reportable) during the run.
    const PERSIST_FOR: Duration = Duration::from_secs(3);

    let mgr = make_manager(16);
    let topic = unique_topic("disc_strand_u");
    let out = unique_out("disc_strand_u");
    let ready = unique_out("disc_strand_u_ready");
    let tag = unique_ring_tag("c5stru");

    // Rank 0 is real and present from the start, so the sweep walks past it to rank 1.
    let mut r0 = Rank::arm(&tag, 0, "n0");

    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_ring_discovery_tag = Some(tag.clone());
    // The graph owns the plane by design; the recorder only drains it.
    let _plane = graph_plane(&tag, 100);
    cfg.fault_inject_state_ring_handoff_always_full = true;

    let mut pubr = publisher(&mgr, &topic, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");
    let oracle0 = r0.anchor(5);
    std::thread::sleep(Duration::from_millis(400));

    // Rank 1's NAME appears and nothing ever publishes behind it.
    let r1_name = cerulion_core::state_ring::state_ring_shm_name(&tag, 1).expect("derived name");
    plant_unpublished_shm_object(&r1_name);
    std::thread::sleep(PERSIST_FOR);

    finish(handle, &shutdown);
    unlink_shm_object(&r1_name);

    let cov = state_coverage(&out).expect("state_coverage.json");
    assert!(
        cov.rings_unavailable.contains_key(&r1_name),
        "a terminal open failure retained by a full channel must still reach the \
         manifest — it is the field that makes this recording read INCOMPLETE: {:?}",
        cov.rings_unavailable
    );
    assert!(
        cov.is_incomplete(),
        "a bag whose recorder could not read a rank's anchors must never report itself \
         complete"
    );
    // Rank 0's adoption was retained too, and must have been delivered as well —
    // otherwise this arm could pass with the whole checkpoint plane empty.
    assert!(
        state_records(&out).contains(&oracle0),
        "rank 0's anchor must be in the bag"
    );
}

/// A hand-off the CHANNEL could never carry still reaches the bag — the ceiling is
/// transport, never a decision about what the artifact claims.
///
/// Consider delivering retained hand-offs at finalization through a bounded try-send
/// loop. If that ceiling expires and the loop `return`s — dropping the message in hand
/// AND every one behind it — finalization carries on to `Finalize` regardless. Then
/// an unavailable-ring report can vanish, `rings_unavailable` comes back empty, and a
/// bag whose recorder could not read a rank's anchors prints COMPLETE. A loud log
/// line is not observable state (Principle #3): the ARTIFACT has to say it.
///
/// The channel is only ever TRANSPORT — the data is the parent's, in its own hand —
/// so at expiry the messages are handed to the terminal `Finalize` message, which uses
/// the BLOCKING send that must succeed for the bag to finalize at all.
///
/// Both shapes ride the same run, because the hand-over is one path and losing either is the
/// same defect: the unavailable REPORT (which decides whether the bag may call itself
/// complete) and the ADOPTION (whose anchors would otherwise be absent).
#[test]
#[serial_test::serial]
fn a_hand_off_the_channel_never_carried_still_reaches_the_bags_coverage() {
    /// Past `STATE_RING_OPEN_RETRY_SCANS` scans, so the open failure turns TERMINAL
    /// and produces the report this arm is about.
    const PERSIST_FOR: Duration = Duration::from_secs(3);

    let mgr = make_manager(16);
    let topic = unique_topic("disc_ceil");
    let out = unique_out("disc_ceil");
    let ready = unique_out("disc_ceil_ready");
    let tag = unique_ring_tag("c5ceil");

    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_ring_discovery_tag = Some(tag.clone());
    // The graph owns the plane by design; the recorder only drains it.
    let _plane = graph_plane(&tag, 100);
    // Every sweep-side hand-off is refused...
    cfg.fault_inject_state_ring_handoff_always_full = true;
    // ...and finalization's channel ceiling is already spent when it tries.
    cfg.fault_inject_state_ring_finalize_channel_full = true;

    let mut pubr = publisher(&mgr, &topic, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");
    std::thread::sleep(Duration::from_millis(400));

    // Rank 0 arms late and is discovered — its ADOPTION is retained.
    let mut r0 = Rank::arm(&tag, 0, "n0");
    // The oracle is pushed only after discovery has opened the ring at its live
    // cursor (same reasoning as the sibling arm above).
    std::thread::sleep(Duration::from_millis(1_500));
    let oracle = r0.anchor(13);

    // Rank 1's NAME appears and nothing publishes behind it — its terminal open
    // failure's REPORT is retained too.
    let r1_name = cerulion_core::state_ring::state_ring_shm_name(&tag, 1).expect("derived name");
    plant_unpublished_shm_object(&r1_name);
    std::thread::sleep(PERSIST_FOR);

    finish(handle, &shutdown);
    unlink_shm_object(&r1_name);

    let cov = state_coverage(&out).expect("state_coverage.json");
    // THE ARM: the artifact names the rank it could not read.
    assert!(
        cov.rings_unavailable.contains_key(&r1_name),
        "a report the channel could not carry must still reach the manifest — dropping \
         it lets a bag with an unreadable rank report itself COMPLETE, which is the one \
         thing this manifest exists to prevent: {:?}",
        cov.rings_unavailable
    );
    assert!(cov.is_incomplete(), "and the bag must NOT read complete");
    // ...and the adoption arrived too, so the ceiling costs no anchors either.
    assert!(
        state_records(&out).contains(&oracle),
        "the retained adoption must also survive the expiry"
    );
    assert_eq!(
        cov.ranks_discovered,
        vec![0],
        "the adopted rank is reported: {cov:?}"
    );
}
