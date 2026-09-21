// SPDX-License-Identifier: AGPL-3.0-only
//! Node-state anchors end-to-end: the anchors reach the bag, and the bag
//! SAYS what it carries.
//!
//! The design's own gate for this chunk is "a recorded bag whose STATE section
//! matches a hand oracle", so every arm here drives a REAL `StateRingOwner`
//! producer through the REAL recorder and reads the finished MCAP back. The
//! records are built by `cerulion_core`'s own `encode_record` /
//! `encode_skip_record` and compared byte-for-byte against a hand-built
//! expectation — never a self-compare against a second run of the recorder.
//!
//! What each arm is FOR:
//!
//! - the HEADLINE: two nodes' anchors cross the ring verbatim, land on the
//!   reserved `__cerulion/state` channel, and `state_coverage.json` reports the
//!   hand-computed tally. Its anti-tautology half is in the same body — a
//!   recording configured for NO checkpoints carries no such attachment and no
//!   state records at all, which is what keeps an ordinary bag byte-identical
//!   to one recorded before node-state anchors existed;
//! - the ESCALATION: a node the ring's MANIFEST declares and that never
//!   anchored makes the manifest incomplete — the condition an operator has to
//!   be able to read on the day it starts being true. A map built only from the
//!   records that arrived could never contain that node, which is why the
//!   manifest is the source;
//! - SKIP records: a voided anchor names its cause and is NOT an
//!   escalation, because a skip is the mechanism working;
//! - a TORN anchor is reported, never served short;
//! - the ARM WORD: a peer opens it by tag and reads exactly what the recorder
//!   armed, and it is DISARMED when the recording ends — the seam a running
//!   graph reads at its step boundaries.
//!
//! Isolated per-test SHM roots ([`common::make_manager`]) + unique ring tags, so
//! the file is parallel-safe; `#[serial_test::serial]` is belt-and-braces and
//! matches its sibling e2e files.

mod common;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use cerulion_bag::writer::TopicSchema;
use cerulion_bagd::{run_bagd, BagdConfig, BagdError, BagdSummary, StateCoverage, TapSpec};
use cerulion_core::state_arm::MappedStateArm;
use cerulion_core::state_ring::{
    encode_record, encode_skip_record, SkipCause, StateRecordHeader, StateRingOwner,
    RECORD_KIND_CHUNK, RECORD_KIND_FINAL, STATE_RECORD_PAYLOAD, STATE_RECORD_SIZE,
};
use cerulion_core::transport::TransportManager;

use common::*;

/// An arbitrary, stable schema hash for the hand-built data frames.
const HASH: u64 = 0x0C1E_0C1E_0C1E_0C1E;
/// The run every state record in this file carries.
const RUN: u64 = 0x0000_C1E5_0000_0001;
/// Ring capacity in records — small enough to be cheap, far above what any arm
/// pushes so no test is measuring a lap it did not intend.
const RING_RECORDS: u32 = 256;

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

/// The state records in the finished bag, in file order.
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

/// The GRAPH'S capture plane, as `arm_capture_plane` creates
/// it — created and armed by the party that owns the run, never by the recorder.
///
/// Returned so the caller BINDS it: the owner's drop unlinks the SHM name, and a
/// recorder that had not yet read the word would then read nothing.
fn graph_plane(tag: &str, cadence_steps: u64, first_anchor_step: u64) -> MappedStateArm {
    let arm = MappedStateArm::create_owned(tag).expect("create this run's capture plane");
    arm.arm(cadence_steps, first_anchor_step);
    arm
}

/// `state_coverage.json`, when the bag carries one.
fn state_coverage(out: &std::path::Path) -> Option<StateCoverage> {
    let reader = cerulion_bag::BagReader::open(out).expect("open bag");
    let att = reader
        .attachment(cerulion_bagd::STATE_COVERAGE_ATTACHMENT)
        .expect("read attachments")?;
    let cov: StateCoverage = serde_json::from_slice(&att.data).expect("state_coverage.json parses");
    assert_eq!(cov.version, cerulion_bagd::STATE_COVERAGE_VERSION);
    Some(cov)
}

/// A whole anchor for `node_idx` at `step`: `parts - 1` full CHUNKs then a FINAL
/// carrying `tail_len` bytes. Hand-built, so the read-back comparison is against
/// a value this test decided.
fn anchor(node_idx: u32, step: u64, parts: u32, tail_len: usize) -> Vec<Vec<u8>> {
    let mut out = Vec::with_capacity(parts as usize);
    for part in 0..parts - 1 {
        out.push(
            encode_record(
                &StateRecordHeader {
                    run_id: RUN,
                    step,
                    node_idx,
                    part,
                    kind: RECORD_KIND_CHUNK,
                    len: STATE_RECORD_PAYLOAD as u32,
                },
                &vec![(0xA0 + node_idx as u8).wrapping_add(part as u8); STATE_RECORD_PAYLOAD],
            )
            .to_vec(),
        );
    }
    out.push(
        encode_record(
            &StateRecordHeader {
                run_id: RUN,
                step,
                node_idx,
                part: parts - 1,
                kind: RECORD_KIND_FINAL,
                len: tail_len as u32,
            },
            &vec![0x5A; tail_len],
        )
        .to_vec(),
    );
    out
}

/// Push `records` onto a ring the recorder is draining.
///
/// There is deliberately NO rendezvous here, and none is needed: the writer
/// thread's Finalize arm tail-drains every ring that still has data before the
/// bag seals, so what the recorder read is settled by the time `finish` returns.
/// The DRAIN PROOF is therefore the bag itself — each arm compares the recorded
/// records against the ones it pushed, which a recorder that dropped any of them
/// cannot satisfy. Polling the ring from the test would need a SECOND consumer
/// on an SPSC endpoint, whose cursor starts at zero and answers a different
/// question entirely.
fn push_records(owner: &mut StateRingOwner, records: &[Vec<u8>]) {
    let mut producer = owner.producer().expect("the single producer");
    for r in records {
        let mut buf = [0u8; STATE_RECORD_SIZE as usize];
        buf.copy_from_slice(r);
        producer.push_record(&buf);
    }
    assert_eq!(
        producer.pushed(),
        records.len() as u64,
        "every record must reach the ring"
    );
}

// ===========================================================================
// THE HEADLINE
// ===========================================================================

/// Two nodes' anchors cross a real ring, land verbatim on the reserved channel,
/// and `state_coverage.json` reports the hand-computed tally.
///
/// The BYTES are compared against records this test built, and the TALLY against
/// counts this test wrote down — so neither half can be satisfied by a recorder
/// that agrees with itself.
#[test]
#[serial_test::serial]
fn node_state_anchors_reach_the_bag_and_the_manifest_matches_a_hand_oracle() {
    let mgr = make_manager(16);
    let topic = unique_topic("state_head");
    let out = unique_out("head");
    let ready = unique_out("head_ready");
    let ring_tag = unique_ring_tag("head");

    let mut pubr = publisher(&mgr, &topic, 4096);
    let mut owner = StateRingOwner::create(&ring_tag, RING_RECORDS, 0, RUN, &["alpha", "beta"])
        .expect("create the state ring");

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_rings = vec![owner.name().to_string()];
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    // One data frame, so the bag is not state-only.
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");

    // node 0 anchors twice (a 3-record anchor then a 1-record one); node 1 once.
    let mut expected: Vec<Vec<u8>> = Vec::new();
    expected.extend(anchor(0, 10, 3, 7));
    expected.extend(anchor(1, 10, 1, 0));
    expected.extend(anchor(0, 20, 1, 5));
    push_records(&mut owner, &expected);

    let summary = finish(handle, &shutdown);

    // (a) the RECORDS: byte-identical, in push order.
    let got = state_records(&out);
    assert_eq!(
        got, expected,
        "every state record must reach the bag VERBATIM and in order"
    );
    assert_eq!(summary.state_records, expected.len() as u64);

    // (b) the MANIFEST: the hand-computed tally.
    let cov = state_coverage(&out).expect("a checkpointed bag carries state_coverage.json");
    assert_eq!(cov.rings_declared, 1);
    assert!(cov.rings_unavailable.is_empty());
    assert_eq!(cov.records, expected.len() as u64);
    assert_eq!(cov.head_records_discarded, 0);
    assert_eq!(cov.malformed_records, 0);
    assert!(!cov.attached_mid_run);
    assert!(cov.armed.is_none(), "this recorder armed nothing");
    assert_eq!(cov.nodes.len(), 2, "both manifest nodes are reported");

    let alpha = &cov.nodes["alpha"];
    assert_eq!(alpha.ring, owner.name());
    // A restore dependency: the manifest INDEX every state
    // record carries. Nothing else carries it into the bag — a record has a
    // `node_idx` and no node id, this map is keyed by node ID, and the ring
    // manifest never leaves the live ring — so without it a restore engine
    // cannot say which node a record belongs to.
    assert_eq!(alpha.node_idx, Some(0));
    assert_eq!(alpha.anchors_complete, 2);
    assert_eq!(alpha.anchors_torn, 0);
    assert_eq!(alpha.anchors_skipped, 0);
    assert_eq!(alpha.last_complete_step, Some(20));
    // 2 full chunks + a 7-byte tail, then a 5-byte one-record anchor.
    assert_eq!(alpha.bytes, 2 * STATE_RECORD_PAYLOAD as u64 + 7 + 5);

    let beta = &cov.nodes["beta"];
    assert_eq!(
        beta.node_idx,
        Some(1),
        "the SECOND declared node — a hardcoded 0 would pass on alpha alone"
    );
    assert_eq!(beta.anchors_complete, 1);
    assert_eq!(beta.last_complete_step, Some(10));
    assert_eq!(beta.bytes, 0, "a zero-length anchor is real and empty");

    assert!(
        !cov.is_incomplete(),
        "every declared node anchored: {cov:#?}"
    );
    cleanup(&out);
}

/// The ANTI-TAUTOLOGY control, and the back-compat pin in one: a recording
/// configured for NO checkpoints carries no state records and NO manifest.
///
/// Without it, "the bag carries a state manifest" is satisfied by a recorder
/// that writes one into every bag — which would change the bytes of every
/// ordinary recording ever made.
#[test]
#[serial_test::serial]
fn a_recording_with_no_checkpoint_configuration_carries_no_state_manifest() {
    let mgr = make_manager(16);
    let topic = unique_topic("state_none");
    let out = unique_out("none");
    let ready = unique_out("none_ready");

    let mut pubr = publisher(&mgr, &topic, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(
        mgr.clone(),
        base_cfg(out.clone(), &topic, ready.clone()),
        shutdown.clone(),
    );
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");
    settle();
    let summary = finish(handle, &shutdown);

    assert_eq!(summary.state_records, 0);
    assert!(summary.state_coverage.is_none());
    assert!(state_records(&out).is_empty());
    assert!(
        state_coverage(&out).is_none(),
        "an ordinary bag must carry NO state_coverage.json — its absence is no claim, \
         and a manifest in every bag would change every recording's bytes"
    );
    cleanup(&out);
}

// ===========================================================================
// THE ESCALATION
// ===========================================================================

/// A node the ring's MANIFEST declares and that never anchored makes the
/// recording incomplete — but only because the PLANE was armed.
///
/// Both halves in one body: the same records, judged against a coverage that saw
/// an armed plane and one that saw none, must reach opposite verdicts. Anchors
/// nobody was ever due to take are not anchors anybody missed, which is the
/// `schema_demand_requested` precedent one layer up.
///
/// The arming belongs to the GRAPH, not to the recorder's config: it is created
/// and armed here by `graph_plane` exactly as `arm_capture_plane` does it, and
/// READ by the recorder.
#[test]
#[serial_test::serial]
fn a_node_that_never_anchored_escalates_only_when_the_plane_was_armed() {
    let mgr = make_manager(16);
    let topic = unique_topic("state_starved");
    let out = unique_out("starved");
    let ready = unique_out("starved_ready");
    let ring_tag = unique_ring_tag("starved");
    let arm_tag = unique_ring_tag("starved_arm");

    let mut pubr = publisher(&mgr, &topic, 4096);
    // THREE declared nodes; only one of them ever anchors.
    let mut owner =
        StateRingOwner::create(&ring_tag, RING_RECORDS, 0, RUN, &["a", "b", "c"]).expect("ring");

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_rings = vec![owner.name().to_string()];
    // The GRAPH'S plane. The recorder is told where to look and reads the word.
    cfg.state_ring_discovery_tag = Some(arm_tag.clone());
    let _plane = graph_plane(&arm_tag, 500, 7);
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");

    push_records(&mut owner, &anchor(0, 5, 1, 4));
    finish(handle, &shutdown);

    let cov = state_coverage(&out).expect("state_coverage.json");
    assert_eq!(cov.nodes.len(), 3, "every DECLARED node is reported");
    assert_eq!(cov.nodes["a"].anchors_complete, 1);
    assert_eq!(cov.nodes["b"].anchors_complete, 0);
    assert_eq!(cov.nodes["c"].anchors_complete, 0);
    // The index comes from the ring MANIFEST, not from records that arrived —
    // so a node that never anchored still carries the one a restore needs to
    // recognise its records if any ever appear.
    assert_eq!(
        [
            cov.nodes["a"].node_idx,
            cov.nodes["b"].node_idx,
            cov.nodes["c"].node_idx
        ],
        [Some(0), Some(1), Some(2)]
    );
    assert_eq!(cov.nodes_without_anchor(), 2);
    assert!(
        cov.is_incomplete(),
        "a bag whose plane was armed and whose node never anchored is not clean"
    );
    // The plane the GRAPH armed is recorded verbatim — read off the word, not
    // invented: a recorder that reported its own idea of a cadence fails here.
    let armed = cov.armed.as_ref().expect("armed");
    assert_eq!(armed.tag, arm_tag);
    assert_eq!(armed.cadence_steps, 500);
    assert_eq!(armed.first_anchor_step, 7);

    // The un-armed half, on the SAME tally.
    let mut unarmed = cov.clone();
    unarmed.armed = None;
    assert!(
        !unarmed.is_incomplete(),
        "a bag that saw no armed plane knows of no anchor anybody was due to take, \
         so it cannot say one was missed"
    );
    cleanup(&out);
}

// ===========================================================================
// SKIP AND TORN
// ===========================================================================

/// A VOIDED anchor names its cause, reaches the bag, and does NOT make
/// the recording incomplete — a skip is the mechanism working.
#[test]
#[serial_test::serial]
fn a_skipped_anchor_names_its_cause_and_is_not_an_escalation() {
    let mgr = make_manager(16);
    let topic = unique_topic("state_skip");
    let out = unique_out("skip");
    let ready = unique_out("skip_ready");
    let ring_tag = unique_ring_tag("skip");

    let mut pubr = publisher(&mgr, &topic, 4096);
    let mut owner = StateRingOwner::create(&ring_tag, RING_RECORDS, 0, RUN, &["solo"]).expect("r");

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_rings = vec![owner.name().to_string()];
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");

    let mut records = anchor(0, 10, 1, 3);
    records.push(encode_skip_record(RUN, 20, 0, SkipCause::Contended, "node mutex held").to_vec());
    records
        .push(encode_skip_record(RUN, 30, 0, SkipCause::StillEncoding, "a peer is busy").to_vec());
    push_records(&mut owner, &records);
    finish(handle, &shutdown);

    assert_eq!(
        state_records(&out),
        records,
        "skip records reach the bag too"
    );
    let cov = state_coverage(&out).expect("state_coverage.json");
    let solo = &cov.nodes["solo"];
    assert_eq!(solo.anchors_complete, 1);
    assert_eq!(solo.anchors_skipped, 2);
    assert_eq!(solo.anchors_torn, 0);
    assert_eq!(solo.skip_causes.get("contended"), Some(&1));
    assert_eq!(solo.skip_causes.get("still_encoding"), Some(&1));
    assert_eq!(cov.skipped_anchors(), 2);
    assert!(
        !cov.is_incomplete(),
        "a named skip is the mechanism working, not a coverage failure"
    );
    cleanup(&out);
}

/// A TORN anchor is REPORTED — never restored short.
///
/// The tear is manufactured the way a real one arrives: a part the writer never
/// pushed, so the stream's next record does not follow the one before it.
#[test]
#[serial_test::serial]
fn a_torn_anchor_is_reported_and_makes_the_recording_incomplete() {
    let mgr = make_manager(16);
    let topic = unique_topic("state_torn");
    let out = unique_out("torn");
    let ready = unique_out("torn_ready");
    let ring_tag = unique_ring_tag("torn");

    let mut pubr = publisher(&mgr, &topic, 4096);
    let mut owner = StateRingOwner::create(&ring_tag, RING_RECORDS, 0, RUN, &["solo"]).expect("r");

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_rings = vec![owner.name().to_string()];
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");

    // A complete anchor first, so the node HAS one — the escalation below is
    // therefore about the tear and not about a starved node.
    let mut records = anchor(0, 5, 1, 2);
    // Then parts 0 and 2 of a three-part anchor: part 1 never arrives.
    let broken = anchor(0, 9, 3, 4);
    records.push(broken[0].clone());
    records.push(broken[2].clone());
    push_records(&mut owner, &records);
    finish(handle, &shutdown);

    assert_eq!(
        state_records(&out),
        records,
        "the bag carries the records VERBATIM — the tear is a READER's verdict, \
         not a reason to drop bytes"
    );
    let cov = state_coverage(&out).expect("state_coverage.json");
    let solo = &cov.nodes["solo"];
    assert_eq!(solo.anchors_complete, 1);
    assert_eq!(solo.anchors_torn, 1, "one broken anchor is ONE verdict");
    assert_eq!(solo.last_complete_step, Some(5));
    assert_eq!(cov.torn_anchors(), 1);
    assert!(cov.is_incomplete());
    cleanup(&out);
}

// ===========================================================================
// THE ARM WORD
// ===========================================================================

/// The recorder READS the graph's arm word, and NEVER
/// disarms it.
///
/// This test is the inversion of the one it replaces. Previously bagd
/// CREATED the word and DISARMED it at finalize, which was right when capture was
/// opt-in and the recorder was the only consumer: a graph left anchoring into a
/// ring nobody drained was pure waste.
///
/// Under an always-on plane that same disarm would be a BUG with a much worse
/// shape — the robot's black box would switch off the moment a recording ended,
/// which is exactly when an operator has most reason to believe it is on. So the
/// pins are: what the recorder puts in the bag is what the GRAPH armed, and the
/// word is STILL ARMED after the recording finalizes.
///
/// The peer is a real `open_unowned`, which is what a running graph does: the
/// assertion is on the seam, not on anybody's own copy.
#[test]
#[serial_test::serial]
fn the_recorder_reads_the_graphs_arm_word_and_never_disarms_it() {
    let mgr = make_manager(16);
    let topic = unique_topic("state_arm");
    let out = unique_out("arm");
    let ready = unique_out("arm_ready");
    let arm_tag = unique_ring_tag("arm_word");

    // THE GRAPH arms. Held for the whole test, as a running graph holds it.
    let plane = graph_plane(&arm_tag, 128, 64);

    let mut pubr = publisher(&mgr, &topic, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_ring_discovery_tag = Some(arm_tag.clone());
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");

    {
        let peer = MappedStateArm::open_unowned(&arm_tag).expect("a peer opens the armed word");
        assert!(peer.is_armed(), "the graph armed it");
        assert_eq!(peer.cadence_steps(), 128);
        assert_eq!(peer.first_anchor_step(), 64);
        // The seam a graph actually uses: is an anchor due at this step?
        assert!(!peer.due(63), "before the first anchor step");
        assert!(peer.due(64), "the first anchor step is due");
        assert!(peer.due(64 + 128), "and every cadence after it");
        assert!(!peer.due(64 + 1), "a step between boundaries is not");

        finish(handle, &shutdown);

        // THE HEADLINE. A recorder finishing must not take the robot's black box
        // down with it: the plane outlives every recording of it.
        assert!(
            peer.is_armed(),
            "a finished recording must NOT disarm the graph's capture plane — the \
             plane is always-on and belongs to the run, not to the recorder"
        );
        assert!(peer.due(64 + 256), "…so anchors are still due afterwards");
    }

    // SAME-OBJECT SENTINEL. `peer` above is a mapping taken BEFORE the recording
    // finished, and a mapping outlives the NAME it was opened through:
    // `create_owned` UNLINKS the tag and creates a fresh object under it. So a
    // recorder that recreated this plane and disarmed the REPLACEMENT would leave
    // `peer` — pointing at the ORIGINAL — armed and reading 128/64, satisfying
    // every assertion above, while `observe_arm_word` and every other opener
    // resolved the tag to a disarmed replacement. The test would pass and the
    // robot's black box would be off.
    //
    // So: write a value nothing else uses through the ORIGINAL owner, then read
    // it back through a FRESH open BY TAG. A replacement object cannot carry a
    // value written into the one it replaced, so this asserts identity and
    // armed-ness together rather than trusting a stale handle for either.
    const SENTINEL_CADENCE: u64 = 4_096;
    const SENTINEL_FIRST: u64 = 2_048;
    plane.arm(SENTINEL_CADENCE, SENTINEL_FIRST);
    let fresh = MappedStateArm::open_unowned(&arm_tag)
        .expect("the tag must still resolve after the recording finalized");
    assert!(
        fresh.is_armed(),
        "a FRESH open by tag must find an armed plane — a stale mapping saying so is \
         not the same claim once `create_owned` can have replaced the object"
    );
    assert_eq!(
        (fresh.cadence_steps(), fresh.first_anchor_step()),
        (SENTINEL_CADENCE, SENTINEL_FIRST),
        "the tag must still resolve to the object the GRAPH created: these values were \
         written through the original owner, so a recreated plane cannot carry them"
    );

    // …and the bag says which plane it drained, with the cadence read off that
    // word rather than any number this recorder chose.
    let cov = state_coverage(&out).expect("state_coverage.json");
    let armed = cov
        .armed
        .as_ref()
        .expect("the bag names the plane it drained");
    assert_eq!(armed.tag, arm_tag);
    assert_eq!(armed.cadence_steps, 128);
    assert_eq!(armed.first_anchor_step, 64);

    drop(plane);
    cleanup(&out);
}

// ===========================================================================
// A DECLARED RING THAT IS GONE
// ===========================================================================

/// An attach-mode recording whose DECLARED state ring vanished keeps recording,
/// names the ring, and refuses to call itself complete.
///
/// The trace rings' policy, applied to the state ones for the same reason: one
/// vanished ring must cost its anchors, never the frames of every topic in the
/// bag — and a bag that was asked for anchors and could not look must say so.
#[test]
#[serial_test::serial]
fn a_declared_state_ring_that_vanished_costs_its_anchors_not_the_recording() {
    let mgr = make_manager(16);
    let topic = unique_topic("state_gone");
    let out = unique_out("gone");
    let ready = unique_out("gone_ready");

    let mut pubr = publisher(&mgr, &topic, 4096);
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    // A name nothing ever created — exactly what a ring whose owner already
    // dropped looks like to the recorder.
    cfg.state_rings = vec!["/cer_st_never_created".to_string()];
    cfg.attached_mid_run = true;
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(
        wait_for_file(&ready, Duration::from_secs(10)),
        "bagd must still come up — a missing ring is not a reason to abandon the recording"
    );
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");
    settle();
    let summary = finish(handle, &shutdown);

    assert_eq!(summary.messages, 1, "the frames still recorded");
    let cov = state_coverage(&out).expect("state_coverage.json");
    assert_eq!(cov.rings_declared, 1);
    assert_eq!(cov.rings_unavailable.len(), 1);
    assert!(cov.rings_unavailable.contains_key("/cer_st_never_created"));
    assert!(cov.attached_mid_run);
    assert!(
        cov.is_incomplete(),
        "asked for anchors and could not even look"
    );
    cleanup(&out);
}

/// An anchor the recording STOPPED IN THE MIDDLE OF is torn — reported, not
/// silently absent.
///
/// This is the arm for the ledger's `finish` CALL SITE, and it exists because
/// the pure `an_anchor_left_open_at_finish_is_torn_and_counted_
/// truncated` cannot reach it: that test drives `finish()` itself, so a recorder
/// that builds its manifest WITHOUT finishing its ledgers passes it and every
/// other arm in this file. The consequence of the gap is a bag whose manifest
/// reports a clean node while an anchor it half-carries can never be served.
///
/// The node anchors once FIRST, so the escalation below is attributable to the
/// tear and not to a starved node — and this recorder arms nothing, so the
/// starved-node clause is not even in play.
#[test]
#[serial_test::serial]
fn an_anchor_the_recording_stopped_inside_is_reported_torn() {
    let mgr = make_manager(16);
    let topic = unique_topic("state_trunc");
    let out = unique_out("trunc");
    let ready = unique_out("trunc_ready");
    let ring_tag = unique_ring_tag("trunc");

    let mut pubr = publisher(&mgr, &topic, 4096);
    let mut owner = StateRingOwner::create(&ring_tag, RING_RECORDS, 0, RUN, &["solo"]).expect("r");

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_rings = vec![owner.name().to_string()];
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");

    let mut records = anchor(0, 4, 1, 3);
    // Then the HEAD of a three-part anchor whose remaining parts never arrive —
    // the capture child died, or the recorder stopped first.
    records.push(anchor(0, 8, 3, 6)[0].clone());
    push_records(&mut owner, &records);
    finish(handle, &shutdown);

    assert_eq!(
        state_records(&out),
        records,
        "the head record IS in the bag — being unusable is a READER's verdict"
    );
    let cov = state_coverage(&out).expect("state_coverage.json");
    let solo = &cov.nodes["solo"];
    assert_eq!(solo.anchors_complete, 1);
    assert_eq!(
        solo.anchors_torn, 1,
        "an anchor still open when the stream ended can never be served, so the \
         manifest must say so rather than report a clean node"
    );
    assert_eq!(solo.last_complete_step, Some(4));
    assert!(cov.armed.is_none(), "the tear is the ONLY escalation here");
    assert!(cov.is_incomplete());
    cleanup(&out);
}

/// A DATA-SILENT run still drains its anchors — the writer's timed wake must
/// see a state ring with unread records, not only a trace one.
///
/// This is the arm for `WriterCore::rings_have_data`'s state half, and it exists
/// because every other arm in this file is blind to it: they push
/// their anchors microseconds after their one data frame, so the batch that
/// frame produces drains the ring by luck. A recording whose topics have gone
/// quiet — a robot idling between missions, a black box before an incident — has
/// no such batch, and the anchors would sit in SHM until finalize (and be LOST
/// with the process if it is killed).
///
/// The oracle is BACKPRESSURE, which is what makes it observable at all: the
/// state ring is backpressure-mode, so a producer that fills it BLOCKS until the
/// consumer advances the cursor. The ring is deliberately tiny, the run is
/// deliberately quiet, and `backpressure_wait_timeouts()` counting ZERO is
/// therefore proof the recorder drained MID-RUN. The 5 s wait is a liveness
/// ceiling against a 20 ms flush cadence (250x), never a measurement.
#[test]
#[serial_test::serial]
fn a_data_silent_run_still_drains_its_anchors_mid_run() {
    // A ring smaller than what this arm pushes, so the second half CANNOT be
    // written without the recorder having drained the first.
    const TINY_RING: u32 = 8;

    let mgr = make_manager(16);
    let topic = unique_topic("state_silent");
    let out = unique_out("silent");
    let ready = unique_out("silent_ready");
    let ring_tag = unique_ring_tag("silent");

    let mut pubr = publisher(&mgr, &topic, 4096);
    let mut owner = StateRingOwner::create(&ring_tag, TINY_RING, 0, RUN, &["solo"]).expect("r");

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_rings = vec![owner.name().to_string()];
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    // ONE frame, so the bag is created — and then the topic goes QUIET. Settled
    // for well over the 20 ms flush cadence so that frame's batch is long gone
    // before any anchor exists: from here the only thing that can drain the
    // state ring is the writer's own timed wake.
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");
    settle();
    settle();

    let mut producer = owner.producer().expect("the single producer");
    producer.set_backpressure_wait_timeout(Duration::from_secs(5));
    let mut expected: Vec<Vec<u8>> = Vec::new();
    for step in 0..(TINY_RING as u64 * 2) {
        let rec = anchor(0, step, 1, 4).remove(0);
        let mut buf = [0u8; STATE_RECORD_SIZE as usize];
        buf.copy_from_slice(&rec);
        producer.push_record(&buf);
        expected.push(rec);
    }
    assert_eq!(
        producer.backpressure_wait_timeouts(),
        0,
        "the recorder must drain the state ring on its own timed wake — with no tap \
         batch to ride, a full ring blocks its producer instead"
    );
    assert_eq!(producer.pushed(), expected.len() as u64);
    drop(producer);
    finish(handle, &shutdown);

    assert_eq!(
        state_records(&out),
        expected,
        "every anchor pushed into a ring smaller than the run must still reach the bag"
    );
    let cov = state_coverage(&out).expect("state_coverage.json");
    assert_eq!(cov.records, expected.len() as u64);
    assert_eq!(cov.nodes["solo"].anchors_complete, expected.len() as u64);
    cleanup(&out);
}

/// A record COMMITTED while the writer is writing is not counted until it is
/// WRITTEN — and it is written.
///
/// The defect this pins: Phase C
/// re-derives the state span after the write (the earlier borrow cannot cross
/// it). If it uses the WHOLE second read to feed the ledger and advance the cursor,
/// a producer that committed records in between widens that read, so the extra
/// records are judged, counted in `state_coverage.json` and skipped over —
/// while never reaching the bag. On a continuously-anchoring graph that is
/// silent loss under a manifest reporting the records as present, which is the
/// exact class closed one layer up.
///
/// The interleave is POSITIONED, never raced: `WriterStallGate::ring_hold` stops
/// the writer between `drain_slices` and the write, which is precisely the
/// window. The oracle is a hand-built record list plus the identity the manifest
/// owes — every record it counts is in the bag.
///
/// **The hold is gated on the span's SIZE, not on the bare "a hold happened"
/// flag.** `ring_hold_entered` is set on EVERY batch taken while the gate is
/// engaged, including one whose state span is EMPTY — and a tap batch in flight
/// produces exactly that. Waiting on the flag alone would let this test push its
/// three "race" records against an empty snapshot and still pass, asserting an
/// interleave it had not set up (MEASURED: forcing that shape reports
/// `ring_hold_state_records == 0` while the flag reads `true`). So the writer is
/// told to hold only a batch carrying `SNAPSHOT` records, and the count it
/// really held is read back as a counted observable — no wall, no sleep.
#[test]
#[serial_test::serial]
fn a_record_committed_while_the_writer_holds_a_span_is_written_not_just_counted() {
    let mgr = make_manager(16);
    let topic = unique_topic("state_race");
    let out = unique_out("race");
    let ready = unique_out("race_ready");
    let ring_tag = unique_ring_tag("race");

    let mut pubr = publisher(&mgr, &topic, 4096);
    let mut owner = StateRingOwner::create(&ring_tag, RING_RECORDS, 0, RUN, &["solo"]).expect("r");

    let gate = std::sync::Arc::new(cerulion_bagd::WriterStallGate::default());
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_rings = vec![owner.name().to_string()];
    cfg.fault_inject_writer_stall_gate = Some(gate.clone());
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    // One frame, so the writer thread exists and the bag is created — then let
    // its batch finish, so the hold below can only ever be entered for a batch
    // this test set up.
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");
    assert!(
        await_condition(Duration::from_secs(10), || out.exists()),
        "the bag file must exist (the writer thread is up)"
    );
    settle();
    settle();

    let mut producer = owner.producer().expect("the single producer");
    let mut push = |records: &[Vec<u8>]| {
        for r in records {
            let mut buf = [0u8; STATE_RECORD_SIZE as usize];
            buf.copy_from_slice(r);
            producer.push_record(&buf);
        }
    };

    // THE SNAPSHOT: engage the hold first, then commit two records. The writer
    // holds only a batch that carries BOTH of them, so the span it stops on is
    // the one this test built rather than whatever a tap batch happened to find.
    const SNAPSHOT: u64 = 2;
    // The threshold is published BEFORE the flag that RELEASES it: the writer
    // acquires `ring_hold_engaged`, so everything stored before this becomes
    // visible to it. Two Relaxed stores carry no happens-before, and a writer
    // that saw `engaged` beside a stale threshold of 0 would hold an empty span
    // — the shape the threshold exists to exclude.
    gate.ring_hold_min_state_records
        .store(SNAPSHOT, Ordering::Relaxed);
    gate.ring_hold_engaged.store(true, Ordering::Release);
    let mut expected: Vec<Vec<u8>> = anchor(0, 10, SNAPSHOT as u32, 6);
    assert_eq!(
        expected.len(),
        SNAPSHOT as usize,
        "the snapshot is two records"
    );
    push(&expected);
    // ACQUIRE, because this flag is the edge: the writer stores the held count
    // and THEN releases this, so observing it here makes that count visible.
    assert!(
        await_condition(Duration::from_secs(10), || gate
            .ring_hold_entered
            .load(Ordering::Acquire)),
        "the writer must be holding a drained-but-uncommitted state span"
    );
    // The counted observable: the held span is EXACTLY the snapshot. An empty
    // (or already-widened) hold makes the interleave below assert nothing.
    //
    // Relaxed is correct HERE and only here — the acquire above already orders
    // this read against the writer's store. Reading it Relaxed WITHOUT that
    // acquire is the bug: the two atomics have no happens-before of their own,
    // so on a weakly-ordered target (aarch64) this
    // could read a stale `0` while a qualifying span really was held, failing
    // intermittently there and never on an x86-TSO machine.
    assert_eq!(
        gate.ring_hold_state_records.load(Ordering::Relaxed),
        SNAPSHOT,
        "the held span must be the two-record snapshot this test pushed — a hold \
         over an empty or wider span cannot exercise the interleave"
    );

    // THE RACE: three more anchors are committed WHILE the writer holds the
    // two-record snapshot. They are not in this batch's write, so nothing may
    // judge, count or step over them until a later batch writes them.
    let later: Vec<Vec<u8>> = anchor(0, 20, 3, 9);
    assert_eq!(later.len(), 3);
    push(&later);
    expected.extend(later);
    gate.ring_hold_engaged.store(false, Ordering::Relaxed);

    drop(producer);
    let summary = finish(handle, &shutdown);

    // (a) EVERY record reaches the bag, in order and verbatim.
    assert_eq!(
        state_records(&out),
        expected,
        "a record committed during the write must still be WRITTEN — the cursor \
         may not step over it"
    );
    // (b) and the manifest counts exactly what the bag holds. A ledger fed the
    // WHOLE second read says 5 over a bag holding 2: right about what the ring had and
    // wrong about what the recording contains, which is the worse of the two
    // failures.
    let cov = state_coverage(&out).expect("state_coverage.json");
    assert_eq!(
        cov.records,
        expected.len() as u64,
        "the manifest may not claim a record the bag does not hold"
    );
    assert_eq!(
        summary.state_records, cov.records,
        "the summary and the manifest report ONE quantity"
    );
    // (c) the anchors themselves reassemble: two whole anchors, nothing torn.
    let solo = &cov.nodes["solo"];
    assert_eq!(solo.anchors_complete, 2);
    assert_eq!(solo.anchors_torn, 0);
    assert_eq!(solo.last_complete_step, Some(20));
    assert!(!cov.is_incomplete(), "{cov:#?}");
    cleanup(&out);
}

/// The manifest's `records` counts what the BAG HOLDS, not what the ledger
/// judged — and the two differ on exactly the records that reach a bag without
/// belonging to any node's tally.
///
/// `StateAnchorLedger::records` excludes the MALFORMED and the mid-run attach's
/// discarded partial head. Both are written VERBATIM (the recorder does not
/// decode, and a headless anchor is detectable at read time), so a manifest
/// built from the ledger's count under-reports its own bag. This arm is the only
/// one in the file where the two numbers can disagree, which is why it exists:
/// every other arm pushes whole anchors from record zero, where judged and
/// written are equal and the distinction is invisible.
///
/// It is also the e2e for `open_at_live` on a STATE ring: records committed
/// before the recorder attached are outside its window and are NOT in the bag.
#[test]
#[serial_test::serial]
fn a_mid_run_attach_counts_the_records_the_bag_holds_not_the_ones_it_judged() {
    let mgr = make_manager(16);
    let topic = unique_topic("state_head_disc");
    let out = unique_out("head_disc");
    let ready = unique_out("head_disc_ready");
    let ring_tag = unique_ring_tag("head_disc");

    let mut pubr = publisher(&mgr, &topic, 4096);
    let mut owner = StateRingOwner::create(&ring_tag, RING_RECORDS, 0, RUN, &["solo"]).expect("r");

    // ONE producer for the whole arm: the ring is SPSC, so it is minted once and
    // pushed to in two phases either side of the recorder's attach.
    let mut producer = owner.producer().expect("the single producer");
    let mut push = |records: &[Vec<u8>]| {
        for r in records {
            let mut buf = [0u8; STATE_RECORD_SIZE as usize];
            buf.copy_from_slice(r);
            producer.push_record(&buf);
        }
    };

    // BEFORE the recorder exists: a whole anchor it will never see. `open_at_live`
    // lands its cursor past these, so they are outside the recording's window —
    // not loss, and not in the bag.
    let unseen = anchor(0, 1, 2, 3);
    push(&unseen);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_rings = vec![owner.name().to_string()];
    cfg.attached_mid_run = true;
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");

    // The tail of an anchor whose head was committed before the attach (parts 1
    // and 2 of a three-part one), then a record this build cannot key at all,
    // then a whole anchor the ledger DOES judge.
    let orphan = anchor(0, 5, 3, 4);
    let mut expected: Vec<Vec<u8>> = vec![orphan[1].clone(), orphan[2].clone()];
    let mut malformed = vec![0u8; STATE_RECORD_SIZE as usize];
    malformed[0] = 1; // a non-zero byte, but kind 0 — no valid record has it
    expected.push(malformed);
    expected.extend(anchor(0, 9, 1, 6));
    push(&expected);
    // The closure's borrow of the producer ends at its last call, so the ring's
    // single producer can be released before the recorder finalizes.
    drop(producer);
    let summary = finish(handle, &shutdown);

    // (a) the WINDOW: the pre-attach anchor is absent, everything after is there.
    let got = state_records(&out);
    assert_eq!(
        got, expected,
        "a mid-run attach records from its live cursor forward — and everything \
         from there, discarded head and unkeyable record included"
    );
    for u in &unseen {
        assert!(
            !got.contains(u),
            "a record committed before the attach is outside the window, not in the bag"
        );
    }

    // (b) the IDENTITY: the manifest counts the bag, and the two halves it does
    // NOT judge are each named.
    let cov = state_coverage(&out).expect("state_coverage.json");
    assert!(cov.attached_mid_run);
    assert_eq!(
        cov.records,
        expected.len() as u64,
        "`records` is what reached the channel — the ledger judged fewer"
    );
    assert_eq!(summary.state_records, cov.records);
    assert_eq!(
        cov.head_records_discarded, 2,
        "the partial head anchor is discarded and EXPLAINED, never silent"
    );
    assert_eq!(cov.malformed_records, 1);
    // records == judged + discarded + malformed, so the judged half is what is
    // left — and it is the one whole anchor.
    assert_eq!(cov.nodes["solo"].anchors_complete, 1);
    assert_eq!(cov.nodes["solo"].anchors_torn, 0);
    assert_eq!(cov.nodes["solo"].last_complete_step, Some(9));
    assert!(
        cov.is_incomplete(),
        "a record the recorder could not key escalates"
    );
    cleanup(&out);
}

/// The writer reports a STATE ring's failure through the state mapper — pinned
/// STRUCTURALLY, because nothing else can see it.
///
/// A state ring is BACKPRESSURE-mode (`StateRingConsumer::open` refuses any
/// other policy), so its producer waits at capacity and the lap this reporting
/// describes cannot be produced by a production producer at all. That is why the
/// wrong sentence is easy to ship: `map_ring_err` says `scheduler-trace ring '{ring}'
/// overran … record(s) irrecoverably lost`, which on a state ring is wrong about
/// the ring, wrong about what was lost, and points an operator at a different
/// artifact. Routing either site back
/// through `map_ring_err` fails this arm, and passes every behavioural arm in
/// the crate — which is the whole reason it is written this way.
///
/// The vocabulary itself is oracle-tested in
/// `lib.rs::a_state_ring_failure_is_not_reported_as_a_scheduler_trace_one`; this
/// arm pins only that the call sites CHOOSE it.
#[test]
fn the_writer_reports_a_state_rings_failure_through_the_state_mapper() {
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/src/lib.rs"))
        .expect("read the crate's own source");
    let code = code_only(&src);
    let body = fn_body(&code, "write_batch");

    // ANTI-TAUTOLOGY: the walk really reached the function under test, and the
    // stripper did not eat it.
    //
    // The markers are `state_spans` (Phase B's own local) and the Phase C CALL,
    // both of which are `write_batch`'s. `take_leading_records(` cannot stand
    // here: the per-ring harvest lives in
    // `harvest_written_state_ring` so its commit can be injected, and the
    // record walk lives there with it. A guard like this is the point — it is the
    // one thing in the tree that catches a refactor silently emptying
    // the body it inspects.
    assert!(
        body.contains("state_spans") && body.contains("harvest_written_state_ring("),
        "the extracted body is not `write_batch` — every assertion below would \
         be vacuous"
    );

    assert_eq!(
        body.matches("map_state_ring_err(").count(),
        2,
        "both state-ring failure sites — the Phase C commit and the deferred \
         up-front lap — must report through the state mapper:\n{body}"
    );
    // The trace half keeps its own, which is what makes the assertion above a
    // statement about CHOICE rather than about one mapper having won.
    assert!(
        body.contains("map_ring_err("),
        "the scheduler-trace half still reports through its own mapper"
    );

    // …and the LIFTED body cannot smuggle the wrong mapper back in. The harvest
    // now runs outside `write_batch`, so the count above no longer sees it: a
    // `map_ring_err` added there would report a STATE ring's failure as a
    // scheduler-trace overrun — wrong about the ring, wrong about what was lost,
    // and pointing an operator at a different artifact — while every assertion
    // above stayed green.
    let harvest = fn_body(&code, "harvest_written_state_ring");
    assert!(
        harvest.contains("take_leading_records("),
        "the extracted body is not `harvest_written_state_ring` — the assertion \
         below would be vacuous"
    );
    assert!(
        !harvest.contains("map_ring_err("),
        "the state-ring harvest must never report through the scheduler-trace \
         mapper:\n{harvest}"
    );
}

/// A ring declared TWICE is refused before the recorder opens anything — on
/// both flags, and the bag is never created.
///
/// The names are ones nothing ever created, which is the discriminator: opening
/// either would fail with a RING error naming the missing object, so a CONFIG
/// error naming the duplicate proves the check runs FIRST. Without it a second
/// consumer would be opened on the same ring, and since each holds its own read
/// cursor every record would reach the bag twice, be counted twice, and — for a
/// state ring — be judged by two ledgers, doubling the manifest's anchor tally.
#[test]
#[serial_test::serial]
fn a_ring_declared_twice_is_refused_before_anything_is_opened() {
    for (flag, dup) in [
        ("--state-ring", "/cer_st_dup"),
        ("--trace-ring", "/cer_rg_dup"),
    ] {
        let mgr = make_manager(16);
        let topic = unique_topic("state_dup");
        let out = unique_out("dup");
        let ready = unique_out("dup_ready");
        let _pubr = publisher(&mgr, &topic, 4096);

        let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
        if flag == "--state-ring" {
            cfg.state_rings = vec![dup.to_string(), dup.to_string()];
        } else {
            cfg.rings = vec![dup.to_string(), dup.to_string()];
        }
        let shutdown = Arc::new(AtomicBool::new(false));
        let err = run_bagd(mgr.clone(), cfg, shutdown).expect_err("a duplicate ring is refused");
        match &err {
            BagdError::Config(text) => {
                assert!(text.contains(flag), "{text}");
                assert!(text.contains(dup), "the duplicate is named: {text}");
                assert!(text.contains("TWICE"), "{text}");
            }
            other => panic!("expected a Config refusal naming the duplicate, got {other:?}"),
        }
        assert!(
            !out.exists(),
            "a refused run creates no bag — the refusal precedes every open"
        );
        assert!(!ready.exists(), "and never signals ready");
        cleanup(&out);
    }
}

// Two tests lived HERE and are DELETED with the code they
// pinned, not with their coverage silently dropped.
//
// `a_setup_that_aborts_never_announced_that_it_armed` and
// `nothing_after_the_arm_word_can_exit_setup_early` both guarded ONE invariant of
// `Recorder::setup`: the arm word had to be created LAST, because
// `MappedStateArm::drop` unlinks the NAME without DISARMING, so any fallible step
// after the create could strand a word reading `armed = true` with no recorder
// behind it — a graph anchoring into a ring nobody would ever drain.
//
// The recorder no longer creates a word (the graph owns the plane), so the
// invariant has no subject: an abort inside `setup` now leaves the SHM namespace
// exactly as it found it. What the recorder does with the word instead —
// `observe_arm_word` — is a read through `open_unowned` that drops its mapping
// immediately, so it can neither create nor strand anything, and the property
// those tests protected is now structural rather than ordered.
//
// The half worth keeping moved rather than vanished: that a recorder must not
// take the plane down is pinned by
// `the_recorder_reads_the_graphs_arm_word_and_never_disarms_it` above.

/// Across a ROTATED recording, the state manifest behaves EXACTLY like its two
/// siblings: written to the FINAL file, absent from every earlier one.
///
/// `bag info` reads `state_coverage.json` from the supplied
/// `BagReader`, which can look like a state-coverage defect. It is not — it is the
/// house shape, and this arm is what makes that a checkable fact rather than a
/// claim. `WriterCore::finalize_bag` writes `record_health`,
/// `record_coverage` AND `state_coverage` into `self.writer`, which by then is
/// the LAST file; `maybe_rotate` calls the raw `BagWriter::finalize` on the
/// outgoing one, carrying forward only the ring manifests, the caller's
/// `--attach` files and the schema catalog.
///
/// So the load-bearing assertion is the CONSISTENCY one: all THREE are absent
/// together from the earlier file and present together in the final one. Testing
/// state coverage alone would pass against a bag that had somehow kept its
/// siblings, which is the divergence a future reader would actually be misled
/// by. Whether the house shape should aggregate across rotated files at all is a
/// question about all three attachments, not about this one.
///
/// `bag info`'s side of it is already pinned: `StateCoverageReading::Absent`
/// renders NOTHING (`the_non_present_arms_say_only_what_they_know`), so an
/// earlier rotation file gets NO state paragraph and therefore makes no false
/// claim about what the recording checkpointed.
#[test]
#[serial_test::serial]
fn a_rotated_recording_carries_its_state_manifest_exactly_where_its_siblings_go() {
    const BODY: usize = 1024;
    const FRAMES: usize = 12;

    let mgr = make_manager(16);
    let topic = unique_topic("state_rot");
    let out = unique_out("rot");
    let ready = unique_out("rot_ready");
    let ring_tag = unique_ring_tag("rot");

    let mut pubr = publisher(&mgr, &topic, (BODY + 128) as u32);
    let mut owner = StateRingOwner::create(&ring_tag, RING_RECORDS, 0, RUN, &["solo"]).expect("r");

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_rings = vec![owner.name().to_string()];
    // Tiny cap — forces several rolls (crib `e2e_test::rotation_produces_...`).
    cfg.size_cap_bytes = Some(3000);
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    push_records(&mut owner, &anchor(0, 3, 1, 5));
    for i in 0..FRAMES {
        pubr.publish_raw(&build_frame(
            HASH,
            i as u32,
            1_000 + i as u64,
            &[(i as u8).wrapping_add(1); BODY],
        ))
        .expect("publish");
        // Pace so a flush + rotation runs between frames.
        std::thread::sleep(Duration::from_millis(10));
    }
    settle();
    let summary = finish(handle, &shutdown);

    assert!(
        summary.bag_paths.len() >= 2,
        "the size cap must produce >= 2 files, got {:?}",
        summary.bag_paths
    );
    // The manifests live in the FINAL file, and `out` (index 0) is not it.
    let last = summary.bag_paths.last().expect("a final file");
    assert_ne!(
        last, &out,
        "this arm is only meaningful when the base file is NOT the final one"
    );

    let present = |path: &std::path::Path, name: &str| -> bool {
        let reader = cerulion_bag::BagReader::open(path).expect("open rotation file");
        reader.attachment(name).expect("read attachments").is_some()
    };
    let trio = [
        cerulion_bagd::STATE_COVERAGE_ATTACHMENT,
        cerulion_bagd::RECORD_COVERAGE_ATTACHMENT,
        cerulion_bagd::RECORD_HEALTH_ATTACHMENT,
    ];

    for name in trio {
        assert!(
            present(last, name),
            "the FINAL file must carry `{name}` — that is where finalize_bag \
             writes all three"
        );
    }
    for path in &summary.bag_paths[..summary.bag_paths.len() - 1] {
        for name in trio {
            assert!(
                !present(path, name),
                "an EARLIER rotation file must carry no `{name}`: rotation calls \
                 the raw BagWriter::finalize, so state coverage is absent on \
                 exactly the same terms as record_coverage and record_health — \
                 a divergence here is what would mislead a reader"
            );
        }
    }

    // And the final file's state manifest is a real one, so "present" is not
    // satisfied by an empty or placeholder attachment.
    let cov = state_coverage(last).expect("the final file's state_coverage.json");
    assert_eq!(cov.nodes["solo"].anchors_complete, 1);

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

/// A real node NAMED like a synthetic unknown-index key keeps its own index and its
/// own tallies.
///
/// Suppose the unknown-index walk filed its entry in the SAME string-keyed map as
/// real node ids, under `node_idx {i} (not in the ring manifest)`. Node ids come
/// from graph YAML and are completely unvalidated — MEASURED against
/// `cerulion_core::graph::validate_graph`, which accepts a node whose id is that
/// exact string and checks node ids for DUPLICATES and nothing else (no charset
/// rule at all, not even the zenoh-reserved set the graph PREFIX is checked
/// against). A node so named would collide with the synthetic entry, and the walk
/// would overwrite its `node_idx` and merge a stranger's anchor tallies into its row
/// — corrupting the exact field a restore engine uses to decide which node a
/// record belongs to.
///
/// Pathological, and reachable, and cheap to close: the unknown indices live
/// in a `u32`-keyed map, so a `String` node id cannot key into it by TYPE.
///
/// The fixture is the collision itself: ONE declared node whose id is the label
/// index 1 would have produced, and records for index 1 — which that one-node
/// manifest does not declare.
#[test]
#[serial_test::serial]
fn a_node_named_like_the_unknown_index_label_keeps_its_own_index_and_tallies() {
    // Byte-for-byte the key a string-keyed unknown-index walk would use for index 1.
    const COLLIDING_ID: &str = "node_idx 1 (not in the ring manifest)";

    let mgr = make_manager(16);
    let topic = unique_topic("state_collide");
    let out = unique_out("collide");
    let ready = unique_out("collide_ready");
    let ring_tag = unique_ring_tag("collide");

    let mut pubr = publisher(&mgr, &topic, 4096);
    // ONE declared node, at index 0, named exactly what index 1's synthetic key
    // would have been.
    let mut owner =
        StateRingOwner::create(&ring_tag, RING_RECORDS, 0, RUN, &[COLLIDING_ID]).expect("ring");

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_rings = vec![owner.name().to_string()];
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");

    let mut records: Vec<Vec<u8>> = Vec::new();
    // The REAL node (index 0): one whole anchor.
    records.extend(anchor(0, 10, 1, 4));
    // Index 1, which this one-node manifest does NOT declare: a TORN anchor, so
    // the tallies that would have been merged are distinguishable from the real
    // node's (which has none).
    let broken = anchor(1, 10, 3, 4);
    records.push(broken[0].clone());
    records.push(broken[2].clone());
    push_records(&mut owner, &records);
    finish(handle, &shutdown);

    let cov = state_coverage(&out).expect("state_coverage.json");

    // THE PIN: the real node keeps ITS index and ITS tallies.
    let real = &cov.nodes[COLLIDING_ID];
    assert_eq!(
        real.node_idx,
        Some(0),
        "the declared node's manifest index must survive — the unknown-index walk \
         overwrote it with 1 when both shared one keyspace"
    );
    assert_eq!(real.anchors_complete, 1);
    assert_eq!(
        real.anchors_torn, 0,
        "index 1's broken anchor must NOT be merged into this node's row"
    );
    assert_eq!(real.last_complete_step, Some(10));
    assert_eq!(cov.nodes.len(), 1, "one DECLARED node: {:#?}", cov.nodes);

    // And the unknown index is still REPORTED — separately, by RING and index.
    let stray = &cov.unattributed_indices[owner.name()][&1];
    assert_eq!(stray.node_idx, Some(1));
    assert_eq!(stray.anchors_torn, 1);
    assert_eq!(stray.anchors_complete, 0);

    // The escalation the split must not lose: a tear counts wherever it lives.
    assert_eq!(cov.torn_anchors(), 1);
    assert!(cov.is_incomplete());
    // ...but an unattributed index is NOT a starved declared node.
    assert_eq!(
        cov.nodes_without_anchor(),
        0,
        "the only declared node anchored; an index no manifest declares was \
         never DUE to anchor"
    );
    cleanup(&out);
}

/// Two rings each emitting the SAME unknown index are two unrelated facts.
///
/// `node_idx` is scoped to ONE ring's manifest — every ring numbers its own
/// nodes from 0 — so a bare index key merged two rings' tallies into one row and
/// let whichever ring was walked last overwrite `ring`, reporting one ring's
/// tears under the other's name. The keyspace is nested by ring for exactly
/// that reason.
///
/// The fixture makes the two contributions DISTINGUISHABLE on purpose: ring A's
/// stray index is TORN, ring B's is SKIPPED. Merged, one row would carry both;
/// separated, each carries its own and neither carries the other's.
#[test]
#[serial_test::serial]
fn two_rings_emitting_the_same_unknown_index_are_kept_apart() {
    let mgr = make_manager(16);
    let topic = unique_topic("state_two_rings");
    let out = unique_out("two_rings");
    let ready = unique_out("two_rings_ready");

    let mut pubr = publisher(&mgr, &topic, 4096);
    // Two rings, each declaring ONE node (index 0). Index 1 is unknown to BOTH.
    let mut ring_a =
        StateRingOwner::create(&unique_ring_tag("two_a"), RING_RECORDS, 0, RUN, &["alpha"])
            .expect("ring a");
    let mut ring_b =
        StateRingOwner::create(&unique_ring_tag("two_b"), RING_RECORDS, 1, RUN, &["beta"])
            .expect("ring b");
    let (name_a, name_b) = (ring_a.name().to_string(), ring_b.name().to_string());
    assert_ne!(name_a, name_b);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = base_cfg(out.clone(), &topic, ready.clone());
    cfg.state_rings = vec![name_a.clone(), name_b.clone()];
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");
    pubr.publish_raw(&build_frame(HASH, 0, 1_000, b"payload"))
        .expect("publish");

    // Ring A: its declared node anchors, and its stray index 1 is TORN.
    let mut a_records = anchor(0, 10, 1, 4);
    let broken = anchor(1, 10, 3, 4);
    a_records.push(broken[0].clone());
    a_records.push(broken[2].clone());
    push_records(&mut ring_a, &a_records);

    // Ring B: its declared node anchors, and its stray index 1 is SKIPPED.
    let mut b_records = anchor(0, 10, 1, 6);
    b_records
        .push(encode_skip_record(RUN, 20, 1, SkipCause::LowMemory, "b's stray index").to_vec());
    push_records(&mut ring_b, &b_records);

    finish(handle, &shutdown);
    let cov = state_coverage(&out).expect("state_coverage.json");

    // The DECLARED nodes are untouched by either stray.
    assert_eq!(cov.nodes["alpha"].anchors_complete, 1);
    assert_eq!(cov.nodes["beta"].anchors_complete, 1);
    assert_eq!(cov.nodes["alpha"].anchors_torn, 0);
    assert_eq!(cov.nodes["beta"].anchors_skipped, 0);

    // THE PIN: BOTH rings are served, each under its OWN name, each with its
    // OWN tallies. Merged under a bare index, one row would carry torn=1 AND
    // skipped=1 and name only one ring.
    assert_eq!(
        cov.unattributed_indices.len(),
        2,
        "one entry per RING, not one per index: {:#?}",
        cov.unattributed_indices
    );
    let stray_a = &cov.unattributed_indices[&name_a][&1];
    let stray_b = &cov.unattributed_indices[&name_b][&1];
    assert_eq!(stray_a.ring, name_a);
    assert_eq!(stray_b.ring, name_b);
    assert_eq!((stray_a.anchors_torn, stray_a.anchors_skipped), (1, 0));
    assert_eq!((stray_b.anchors_torn, stray_b.anchors_skipped), (0, 1));
    assert_eq!(stray_b.skip_causes.get("low_memory"), Some(&1));
    assert!(
        stray_a.skip_causes.is_empty(),
        "ring A's row must not carry ring B's cause"
    );

    // The totals still see both, wherever they live.
    assert_eq!(cov.torn_anchors(), 1);
    assert_eq!(cov.skipped_anchors(), 1);
    assert!(cov.is_incomplete());
    cleanup(&out);
}
