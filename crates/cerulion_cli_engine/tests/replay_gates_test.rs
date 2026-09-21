// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion replay` entry-gate coverage.
//!
//! Builds bags in a tempdir via `cerulion_bag::BagWriter`, then drives
//! `cerulion_cli_engine::replay_cmd::run_replay` and asserts the typed
//! `ReplayError` variant + its exit code for every gate class. No iceoryx2, no
//! transport, unique tempdir per test — parallel-safe.
//!
//! Unix-gated: `cerulion_bag` and the `replay_cmd` module it feeds are both
//! `#![cfg(unix)]`.

#![cfg(unix)]

use std::path::Path;

use cerulion_bag::{BagWriter, BagWriterConfig, TopicSchema};
use cerulion_cli_engine::replay_cmd::{run_replay, ReplayError, ReplayOptions};
use cerulion_core::read_outcome::{ReadOutcomeKind, ReadSiteRole};
use cerulion_core::trace_ring::{TraceRingRecord, RECORD_TYPE_DEPARTURE, RECORD_TYPE_FIRE};
use cerulion_core::wire::WireHeader;

/// The gate tests all exercise a FAILING gate, so `run_replay` returns before
/// the engine runs — the options never matter. A shared default keeps the call
/// sites terse.
fn gate(path: &Path) -> ReplayError {
    run_replay(path, ReplayOptions::default()).unwrap_err()
}

const USER_TOPIC: &str = "/data";

/// One attachment: (name, media_type, bytes).
type Att = (String, String, Vec<u8>);

fn att(name: &str, media_type: &str, bytes: Vec<u8>) -> Att {
    (name.to_string(), media_type.to_string(), bytes)
}

fn topics() -> Vec<TopicSchema> {
    vec![TopicSchema {
        topic: USER_TOPIC.into(),
        schema_name: "std_msgs/UInt8".into(),
        schema_hash: 0x0102_0304_0506_0708,
        wire_fixed_size: 8,
    }]
}

fn trace_rec(record_type: u32, node_idx: u32, step: u64) -> TraceRingRecord {
    TraceRingRecord {
        step,
        fire_time_ns: 1000 + step,
        duration_ns: 10,
        node_idx,
        global_level: 0,
        record_type,
        reserved: 0,
    }
}

fn fire(node_idx: u32, step: u64) -> TraceRingRecord {
    trace_rec(RECORD_TYPE_FIRE, node_idx, step)
}

/// A minimal graph the parser AND validator accept: one source node with an
/// explicit prefix (deterministic — no hostname dependency).
fn valid_graph_yaml() -> Vec<u8> {
    b"name: replaytest\nprefix: replaytest\nnodes:\n  - id: pub1\n    type: test_pub\n    \
      outputs:\n      - name: data\n        schema: test/Data\n"
        .to_vec()
}

fn manifest_bytes(rank: u32, node_ids: &[&str]) -> Vec<u8> {
    let ids: Vec<String> = node_ids.iter().map(|s| s.to_string()).collect();
    // Mirror the recorder's shape (rank + generation + node_ids); the reader
    // consumes node_ids and reads the rank from the attachment name.
    serde_json::to_vec(&serde_json::json!({
        "rank": rank,
        "generation": 0,
        "node_ids": ids,
    }))
    .unwrap()
}

fn manifest_name(rank: u32) -> String {
    format!("__cerulion/trace_manifest_rank{rank}.json")
}

/// A full replay-grade attachment set: valid graph, env.json, rank-0 manifest.
fn replay_grade_attachments(node_ids: &[&str]) -> Vec<Att> {
    vec![
        att("graph.yaml", "application/yaml", valid_graph_yaml()),
        att(
            "env.json",
            "application/json",
            br#"{"RUST_LOG":"info"}"#.to_vec(),
        ),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, node_ids),
        ),
    ]
}

/// Write a bag at `path` with `trace` scheduler-trace records (interleaved in a
/// single chunk with one user message) and `attachments`. `finalize=false`
/// DROPS the writer instead — the chunk is flushed by `write_chunk`'s scope-end
/// but no epilogue is written, so the reader classifies it non-finalized.
fn write_bag(path: &Path, trace: &[TraceRingRecord], attachments: &[Att], finalize: bool) {
    // Payload MUST outlive the whole write_chunk call (zero-copy pointer
    // contract) — declared before the closure.
    let payload = [0xABu8; 8];
    let mut w = BagWriter::create(path, BagWriterConfig::default(), &topics()).unwrap();
    w.write_chunk(|c| {
        c.write_message(USER_TOPIC, 0, 1000, 1000, &[&payload[..]])?;
        for (i, rec) in trace.iter().enumerate() {
            c.write_scheduler_trace(i as u32, rec.fire_time_ns, rec.fire_time_ns, rec)?;
        }
        Ok(())
    })
    .unwrap();
    for (name, media_type, data) in attachments {
        w.write_attachment(name, media_type, 0, 0, data).unwrap();
    }
    if finalize {
        w.finalize().unwrap();
    } else {
        drop(w);
    }
}

fn tmp_bag(name: &str) -> (tempfile::TempDir, std::path::PathBuf) {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join(name);
    (dir, path)
}

// ── exit-2 family: bag I/O / not-replay-grade ──────────────────────────────

#[test]
fn missing_file_is_bag_open_exit_2() {
    let (_dir, path) = tmp_bag("does_not_exist.mcap");
    let err = gate(&path);
    assert!(matches!(err, ReplayError::BagOpen { .. }), "got {err:?}");
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn garbage_non_mcap_is_not_mcap_bag_exit_2() {
    let (_dir, path) = tmp_bag("garbage.mcap");
    std::fs::write(&path, b"this is not an MCAP file, just random bytes").unwrap();
    let err = gate(&path);
    // A file without the MCAP magic is caught by the fail-fast probe with a
    // clean "not an MCAP bag" message (NOT the confusing "not finalized").
    assert!(matches!(err, ReplayError::NotMcapBag { .. }), "got {err:?}");
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn unfinalized_bag_is_bag_not_finalized_exit_2() {
    let (_dir, path) = tmp_bag("unfinalized.mcap");
    // Drop without finalize — the flushed chunk is on disk, but no epilogue.
    write_bag(&path, &[fire(0, 1)], &[], false);
    let err = gate(&path);
    match &err {
        ReplayError::BagNotFinalized { completeness } => {
            // OBSERVED behavior: a dropped-without-finalize writer reads back as
            // TruncatedAtChunkBoundary (the chunk flushed cleanly; only the
            // finalization epilogue is absent).
            assert!(
                completeness.contains("TruncatedAtChunkBoundary"),
                "got completeness: {completeness}"
            );
        }
        other => panic!("expected BagNotFinalized, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn finalized_empty_trace_is_no_scheduler_trace_exit_2() {
    let (_dir, path) = tmp_bag("empty_trace.mcap");
    // A finalized bag with replay-grade attachments but no trace
    // records.
    write_bag(&path, &[], &replay_grade_attachments(&["node0"]), true);
    let err = gate(&path);
    assert!(
        matches!(err, ReplayError::BagNoSchedulerTrace),
        "got {err:?}"
    );
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn finalized_trace_missing_graph_yaml_is_missing_attachment_exit_2() {
    let (_dir, path) = tmp_bag("no_graph.mcap");
    let atts = vec![att(
        &manifest_name(0),
        "application/json",
        manifest_bytes(0, &["node0"]),
    )];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::BagMissingAttachment { name } => assert_eq!(name, "graph.yaml"),
        other => panic!("expected BagMissingAttachment graph.yaml, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn graph_yaml_unparseable_is_invalid_attachment_exit_2() {
    let (_dir, path) = tmp_bag("bad_graph_syntax.mcap");
    let atts = vec![
        // Unterminated flow sequence — a hard YAML syntax error.
        att(
            "graph.yaml",
            "application/yaml",
            b"name: test\nnodes: [".to_vec(),
        ),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["node0"]),
        ),
    ];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { name, reason } => {
            assert_eq!(name, "graph.yaml");
            assert!(reason.contains("parse"), "got reason: {reason}");
        }
        other => panic!("expected BagInvalidAttachment graph.yaml, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

/// A bag whose embedded graph carries a key the format does not
/// define must be refused ACTIONABLY, not with a raw serde string.
///
/// The crafted bag is the legacy embedded-graph shape: the
/// recorder embedded the ON-DISK `graphs/<name>.yaml`, which could still carry
/// a legacy `policy:` block, dead configuration a lenient parser swallows
/// and the strict one denies. Such a bag does not replay, and the refusal has
/// to say WHY rather than read like the user mistyped something.
#[test]
fn a_legacy_graph_attachment_is_refused_with_the_migration_remedy() {
    let (_dir, path) = tmp_bag("legacy_policy_graph.mcap");
    let atts = vec![
        att(
            "graph.yaml",
            "application/yaml",
            // Valid YAML, valid legacy graph, an unknown key to this build.
            b"nodes:\n  - id: node0\n    type: node0\n    policy:\n      period_ms: 10\n".to_vec(),
        ),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["node0"]),
        ),
    ];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { name, reason } => {
            assert_eq!(name, "graph.yaml");
            // The KEY the user has to find.
            assert!(reason.contains("`policy`"), "must name the key: {reason}");
            // WHERE it is — which artifact inside the bag.
            assert!(
                reason.contains("graph.yaml") && reason.contains("attachment"),
                "must locate it in the bag: {reason}"
            );
            // Which embedded-graph shape explains the compatibility boundary.
            assert!(
                reason.contains("embedded the on-disk") && reason.contains("EFFECTIVE config"),
                "must explain the embedded-graph shape: {reason}"
            );
            // The REMEDY names its verb and issue...
            assert!(
                reason.contains("cerulion bag migrate") && reason.contains("writes a NEW bag"),
                "must name the remedy and what it does: {reason}"
            );
            // ...as a command to RUN. The verb exists, so the two
            // tokens that would mark it missing are forbidden — a user who
            // reads "PLANNED" will not try the one thing that fixes this bag.
            assert!(
                !reason.contains("PLANNED") && !reason.contains("NOT a subcommand yet"),
                "the verb exists — the refusal must not call it planned: {reason}"
            );
            // The second path stays named: for a graph that has moved on,
            // re-recording is the better answer.
            assert!(
                reason.contains("re-record"),
                "must keep the re-record path: {reason}"
            );
            // And it must not claim the rewrite happens IN PLACE — `bag
            // migrate` writes a new bag and never touches this one.
            assert!(
                !reason.contains("in place"),
                "migration never rewrites a bag in place: {reason}"
            );
        }
        other => panic!("expected BagInvalidAttachment graph.yaml, got {other:?}"),
    }
    // The refusal STANDS and its exit code is unchanged — a bag whose embedded
    // graph does not parse is still not replay-grade.
    assert_eq!(err.exit_code(), 2);
}

/// ANTI-TAUTOLOGY for the arm above: a CURRENT-format graph attachment must
/// not be met with a migration lecture.
///
/// Without this, a refusal that emitted the migration paragraph for every parse
/// failure — or for every bag — would pass the arm above and mislead every
/// user whose bag is fine.
#[test]
fn a_current_format_graph_attachment_is_not_offered_the_migration_remedy() {
    let (_dir, path) = tmp_bag("clean_graph_control.mcap");
    let atts = vec![
        att(
            "graph.yaml",
            "application/yaml",
            // Malformed YAML — a real parse failure, but NOT the legacy shape.
            b"nodes: [".to_vec(),
        ),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["node0"]),
        ),
    ];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { reason, .. } => {
            assert!(
                !reason.contains("bag migrate") && !reason.contains("writes a NEW bag"),
                "real YAML damage must not be told to migrate: {reason}"
            );
            assert!(reason.contains("parse"), "got reason: {reason}");
        }
        other => panic!("expected BagInvalidAttachment, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn graph_yaml_fails_validation_is_invalid_attachment_exit_2() {
    let (_dir, path) = tmp_bag("bad_graph_validate.mcap");
    let atts = vec![
        // Parses fine, but an empty node list fails validate_graph.
        att(
            "graph.yaml",
            "application/yaml",
            b"name: test\nprefix: valid\nnodes: []\n".to_vec(),
        ),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["node0"]),
        ),
    ];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { name, reason } => {
            assert_eq!(name, "graph.yaml");
            assert!(reason.contains("validation"), "got reason: {reason}");
        }
        other => panic!("expected BagInvalidAttachment graph.yaml (validation), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn manifest_missing_is_missing_attachment_exit_2() {
    let (_dir, path) = tmp_bag("no_manifest.mcap");
    let atts = vec![att("graph.yaml", "application/yaml", valid_graph_yaml())];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::BagMissingAttachment { name } => {
            assert!(name.contains("trace_manifest_rank0"), "got {name}")
        }
        other => panic!("expected BagMissingAttachment manifest, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn manifest_out_of_bounds_node_idx_is_invalid_attachment_exit_2() {
    let (_dir, path) = tmp_bag("oob_node_idx.mcap");
    let atts = vec![
        att("graph.yaml", "application/yaml", valid_graph_yaml()),
        // Manifest lists ONE node id, but a trace record references node_idx 5.
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["only_one"]),
        ),
    ];
    write_bag(&path, &[fire(5, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { name, reason } => {
            assert!(name.contains("trace_manifest_rank0"), "got {name}");
            assert!(
                reason.contains("node_idx 5") && reason.contains("1 node id"),
                "got reason: {reason}"
            );
        }
        other => panic!("expected BagInvalidAttachment (oob node_idx), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn manifest_non_numeric_rank_is_invalid_attachment_exit_2() {
    let (_dir, path) = tmp_bag("bad_rank_name.mcap");
    let atts = vec![
        att("graph.yaml", "application/yaml", valid_graph_yaml()),
        // Matches the prefix/suffix but the rank segment is not an integer.
        att(
            "__cerulion/trace_manifest_rankX.json",
            "application/json",
            manifest_bytes(0, &["node0"]),
        ),
    ];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { name, reason } => {
            assert!(name.contains("rankX"), "got name: {name}");
            assert!(reason.contains("integer rank"), "got reason: {reason}");
        }
        other => panic!("expected BagInvalidAttachment (bad rank segment), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn two_rank_manifests_pass_the_gates_and_reach_the_engine_exit_3() {
    let (_dir, path) = tmp_bag("two_ranks.mcap");
    // Contiguous worker ranks (0, 1) are ACCEPTED through the
    // WHOLE gate layer — the multi-rank bag proceeds into the replay engine
    // (per-rank demux + k-way merge) and fails only at cdylib resolution
    // (NodeLoad, exit 3 — this test env has no `test_pub` cdylib), exactly
    // like the single-rank `full_replay_grade_bag_reaches_engine_...` pin.
    let atts = vec![
        att("graph.yaml", "application/yaml", valid_graph_yaml()),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["n0"]),
        ),
        att(
            &manifest_name(1),
            "application/json",
            manifest_bytes(1, &["n1"]),
        ),
    ];
    // One rank-0 FIRE + one rank-1 FIRE (the rank-1 record is bounds-checked
    // against rank 1's OWN one-entry table — the per-rank gate check).
    let mut rank1_fire = fire(0, 1);
    rank1_fire.reserved = 1;
    write_bag(&path, &[fire(0, 1), rank1_fire], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::NodeLoad { node_type, .. } => assert_eq!(node_type, "test_pub"),
        other => panic!("expected NodeLoad (multi-rank bag reaches the engine), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 3);
}

#[test]
fn sentinel_departure_ring_manifest_alongside_workers_is_tolerated_reaches_engine_exit_3() {
    let (_dir, path) = tmp_bag("sentinel_plus_workers.mcap");
    // The supervisor departure-ring manifest (rank u32::MAX =
    // trace_manifest_rank4294967295.json) is a supervisor artifact — TOLERATED
    // (excluded from worker-rank contiguity, never indexed). The (0, 1) worker
    // bag still passes the whole gate layer and reaches the engine (exit 3
    // here — no cdylib in the test env).
    let atts = vec![
        att("graph.yaml", "application/yaml", valid_graph_yaml()),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["n0"]),
        ),
        att(
            &manifest_name(1),
            "application/json",
            manifest_bytes(1, &["n1"]),
        ),
        att(
            &manifest_name(u32::MAX),
            "application/json",
            // The departure ring's manifest carries an EMPTY node table.
            manifest_bytes(u32::MAX, &[]),
        ),
    ];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::NodeLoad { node_type, .. } => assert_eq!(node_type, "test_pub"),
        other => {
            panic!("expected NodeLoad (sentinel tolerated, bag reaches engine), got {other:?}")
        }
    }
    assert_eq!(err.exit_code(), 3);
}

#[test]
fn rank1_fire_out_of_rank1_table_bounds_is_invalid_attachment_exit_2() {
    let (_dir, path) = tmp_bag("rank1_oob.mcap");
    // Per-rank bounds: a rank-1 FIRE's node_idx indexes
    // rank 1's OWN manifest table — node_idx 5 against a one-entry rank-1
    // table must be refused naming the RANK-1 manifest (not rank 0's).
    let atts = vec![
        att("graph.yaml", "application/yaml", valid_graph_yaml()),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["n0"]),
        ),
        att(
            &manifest_name(1),
            "application/json",
            manifest_bytes(1, &["n1"]),
        ),
    ];
    let mut oob = fire(5, 1);
    oob.reserved = 1;
    write_bag(&path, &[fire(0, 1), oob], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { name, reason } => {
            assert!(name.contains("trace_manifest_rank1"), "got {name}");
            assert!(reason.contains("node_idx 5"), "got reason: {reason}");
        }
        other => panic!("expected BagInvalidAttachment (rank-1 bounds), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn fire_stamped_with_unmanifested_rank_is_recording_inconsistent_exit_2() {
    let (_dir, path) = tmp_bag("unmanifested_rank.mcap");
    // A record stamped with a rank that has NO worker manifest (reserved = 3
    // on a 2-rank bag) is corruption: the per-rank demux would have no table
    // or cursor for it. Refused at the gate naming the stamp + manifest range.
    let atts = vec![
        att("graph.yaml", "application/yaml", valid_graph_yaml()),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["n0"]),
        ),
        att(
            &manifest_name(1),
            "application/json",
            manifest_bytes(1, &["n1"]),
        ),
    ];
    let mut foreign = fire(0, 1);
    foreign.reserved = 3;
    write_bag(&path, &[fire(0, 1), foreign], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::RecordingInconsistent { detail } => {
            assert!(
                detail.contains("rank 3") && detail.contains("0..=1"),
                "got detail: {detail}"
            );
        }
        other => panic!("expected RecordingInconsistent (unmanifested rank), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn worker_rank_gap_is_manifest_gap_exit_2() {
    let (_dir, path) = tmp_bag("rank_gap.mcap");
    // Worker ranks (0, 2) with rank 1 absent is a HOLE — a corrupt bag. The
    // message must name the missing rank (1).
    let atts = vec![
        att("graph.yaml", "application/yaml", valid_graph_yaml()),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["n0"]),
        ),
        att(
            &manifest_name(2),
            "application/json",
            manifest_bytes(2, &["n2"]),
        ),
    ];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::MultiRankManifestGap {
            present,
            missing,
            max,
        } => {
            assert_eq!(present, &vec![0, 2]);
            assert_eq!(*missing, 1);
            assert_eq!(*max, 2);
            assert!(err.to_string().contains("rank 1"), "got: {err}");
        }
        other => panic!("expected MultiRankManifestGap (missing rank 1), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn lone_nonzero_rank_manifest_is_manifest_gap_missing_rank_0_exit_2() {
    let (_dir, path) = tmp_bag("lone_rank3.mcap");
    // A single rank-3 manifest (no ranks 0..=2) is a gap missing rank 0 — a
    // corrupt bag, not a multi-process "unsupported" refusal.
    let atts = vec![
        att("graph.yaml", "application/yaml", valid_graph_yaml()),
        att(
            &manifest_name(3),
            "application/json",
            manifest_bytes(3, &["n3"]),
        ),
    ];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::MultiRankManifestGap {
            present,
            missing,
            max,
        } => {
            assert_eq!(present, &vec![3]);
            assert_eq!(*missing, 0);
            assert_eq!(*max, 3);
        }
        other => panic!("expected MultiRankManifestGap (missing rank 0), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn env_json_unparseable_is_invalid_attachment_exit_2() {
    let (_dir, path) = tmp_bag("bad_env_syntax.mcap");
    let atts = vec![
        att("graph.yaml", "application/yaml", valid_graph_yaml()),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["n0"]),
        ),
        att(
            "env.json",
            "application/json",
            b"not json at all {[".to_vec(),
        ),
    ];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { name, .. } => assert_eq!(name, "env.json"),
        other => panic!("expected BagInvalidAttachment env.json, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn env_json_non_object_is_invalid_attachment_exit_2() {
    let (_dir, path) = tmp_bag("env_not_object.mcap");
    let atts = vec![
        att("graph.yaml", "application/yaml", valid_graph_yaml()),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["n0"]),
        ),
        // Valid JSON, but a top-level array, not an object.
        att("env.json", "application/json", b"[1, 2, 3]".to_vec()),
    ];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { name, reason } => {
            assert_eq!(name, "env.json");
            assert!(reason.contains("object"), "got reason: {reason}");
        }
        other => panic!("expected BagInvalidAttachment env.json (non-object), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

/// A corrupt footer `summary_start` through the FULL
/// `run_replay` path is the exit-2 corrupt-bag class — NEVER exit 5 (which
/// would misroute the operator to "file a tool bug" for a bad bag), and NEVER
/// a panic (a `user_message_index` slice panic). The finalization
/// gate validates only the footer fingerprint, so a value corrupted into
/// `[1, 8)` survives the completeness check and dies at whichever later
/// reader-borrowing step first trusts it — the exit class is the contract, not
/// the specific gate.
#[test]
fn corrupt_footer_summary_start_is_exit_2_never_exit_5_or_panic() {
    let (_dir, path) = tmp_bag("corrupt_summary_start.mcap");
    write_bag(
        &path,
        &[fire(0, 1)],
        &replay_grade_attachments(&["pub1"]),
        true,
    );
    // Overwrite the footer's summary_start field with 4. Footer tail layout:
    // [0x02][len=20 u64][summary_start u64][summary_offset_start u64]
    // [summary_crc u32][MAGIC 8] → the field sits at len - 28.
    let mut bytes = std::fs::read(&path).unwrap();
    let off = bytes.len() - 28;
    let original = u64::from_le_bytes(bytes[off..off + 8].try_into().unwrap()) as usize;
    assert!(
        original >= 8 && original < bytes.len(),
        "summary_start offset arithmetic drifted (decoded {original})"
    );
    bytes[off..off + 8].copy_from_slice(&4u64.to_le_bytes());
    std::fs::write(&path, &bytes).unwrap();

    let err = gate(&path);
    assert_eq!(
        err.exit_code(),
        2,
        "a corrupt bag is exit 2 (not 5 internal): {err:?}"
    );
    assert!(
        !matches!(err, ReplayError::Internal { .. }),
        "must not be the harness-bug class: {err:?}"
    );
}

// ── strict record_type gate: v1 accepts FIRE + STEP_BOUNDARY records ───────

#[test]
fn departure_record_is_degraded_recording_exit_2() {
    let (_dir, path) = tmp_bag("departure_rec.mcap");
    // Otherwise replay-grade bag: a DEPARTURE (fault) record at index 1 marks a
    // DEGRADED recording (a peer worker was lost mid-run) — refused
    // PRE-canonicalization with the fault-replay wording.
    write_bag(
        &path,
        &[fire(0, 1), trace_rec(RECORD_TYPE_DEPARTURE, 1, 2)],
        &replay_grade_attachments(&["node0"]),
        true,
    );
    let err = gate(&path);
    match &err {
        ReplayError::DegradedRecordingDeparture { index, detail } => {
            assert_eq!(*index, 1);
            assert!(detail.contains("DEPARTURE"), "got detail: {detail}");
            // The fault-replay wording is in the Display, not the detail.
            let msg = err.to_string();
            assert!(
                msg.contains("degraded recording")
                    && msg.contains("fault replay")
                    && msg.contains("post-launch"),
                "got: {msg}"
            );
        }
        other => panic!("expected DegradedRecordingDeparture, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn fire_record_stamped_with_sentinel_rank_is_degraded_recording_exit_2() {
    let (_dir, path) = tmp_bag("sentinel_stamped_fire.mcap");
    // bagd stamps the ring's rank into `reserved`; a FIRE/BOUNDARY record
    // carrying the u32::MAX supervisor departure-ring sentinel rank is departure
    // provenance leaking onto the data path — the SAME degraded-recording
    // refusal as an explicit departure record (the OR arm of that gate).
    let sentinel_fire = TraceRingRecord {
        reserved: u32::MAX,
        ..fire(0, 1)
    };
    write_bag(
        &path,
        &[fire(0, 0), sentinel_fire],
        &replay_grade_attachments(&["node0"]),
        true,
    );
    let err = gate(&path);
    match &err {
        ReplayError::DegradedRecordingDeparture { index, detail } => {
            assert_eq!(*index, 1);
            assert!(
                detail.contains("sentinel") || detail.contains("reserved"),
                "got detail: {detail}"
            );
        }
        other => panic!("expected DegradedRecordingDeparture (sentinel rank), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn zeroed_record_type_is_trace_record_unsupported_exit_2() {
    let (_dir, path) = tmp_bag("zeroed_rec.mcap");
    // record_type 0 is documented INVALID (a zeroed slot) in trace_ring.
    write_bag(
        &path,
        &[trace_rec(0, 0, 1)],
        &replay_grade_attachments(&["node0"]),
        true,
    );
    let err = gate(&path);
    match &err {
        ReplayError::TraceRecordUnsupported {
            index,
            record_type,
            reason,
        } => {
            assert_eq!(*index, 0);
            assert_eq!(*record_type, 0);
            assert!(
                reason.contains("invalid") && reason.contains("corrupt"),
                "got reason: {reason}"
            );
        }
        other => panic!("expected TraceRecordUnsupported (zeroed), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn reserved_record_type_is_trace_record_unsupported_exit_2() {
    let (_dir, path) = tmp_bag("reserved_rec.mcap");
    // record_type 4/5 are RESERVED (keyframe = kind 4 / nondeterminism =
    // kind 5), as are 7+ — a bag carrying one was written by a newer/foreign
    // writer. (Kind 3 is STEP_BOUNDARY and kind 6 is READ_OUTCOME, and both
    // are accepted: see
    // `step_boundary_records_pass_the_record_type_gate` /
    // `read_outcome_records_pass_the_record_type_gate`.)
    write_bag(
        &path,
        &[fire(0, 1), fire(0, 2), trace_rec(4, 0, 3)],
        &replay_grade_attachments(&["node0"]),
        true,
    );
    let err = gate(&path);
    match &err {
        ReplayError::TraceRecordUnsupported {
            index,
            record_type,
            reason,
        } => {
            assert_eq!(*index, 2);
            assert_eq!(*record_type, 4);
            assert!(
                reason.contains("reserved") && reason.contains("newer or foreign writer"),
                "got reason: {reason}"
            );
        }
        other => panic!("expected TraceRecordUnsupported (reserved), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn step_boundary_records_pass_the_record_type_gate() {
    // STEP_BOUNDARY (kind 3) records are ACCEPTED by the
    // strict record_type gate — the bag reaches the engine and fails at cdylib
    // load (exit 3, no `test_pub` in this crate's tree), NOT with the exit-2
    // TraceRecordUnsupported a stricter gate would raise for kind 3. Their
    // node_idx (0, meaningless) is NOT bounds-checked (semantic validation is
    // the engine's cross-check).
    let (_dir, path) = tmp_bag("boundary_ok.mcap");
    let boundary = |step: u64, target: u64| TraceRingRecord {
        step,
        fire_time_ns: target,
        duration_ns: 0,
        node_idx: 0,
        global_level: 0,
        record_type: cerulion_core::trace_ring::RECORD_TYPE_STEP_BOUNDARY,
        reserved: 0,
    };
    write_bag(
        &path,
        &[boundary(1, 1001), fire(0, 1), boundary(2, 1002), fire(0, 2)],
        &replay_grade_attachments(&["node0"]),
        true,
    );
    let err = gate(&path);
    assert!(
        matches!(err, ReplayError::NodeLoad { .. }),
        "a boundary-bearing bag must pass the record_type gate and reach the engine \
         (NodeLoad for the missing cdylib); got {err:?}"
    );
    assert_eq!(err.exit_code(), 3);
}

#[test]
fn read_outcome_records_pass_the_record_type_gate() {
    // Kind-6 READ-OUTCOME records are ACCEPTED by the strict
    // record_type gate — the bag reaches the engine and fails at cdylib load
    // (exit 3, no `test_pub` in this crate's tree), NOT with the exit-2
    // TraceRecordUnsupported a gate unaware of kind 6 would raise. Both served-seq
    // shapes are exercised (a real sequence and the NO_FRAME sentinel in the
    // fire_time_ns slot must not confuse the gate).
    let (_dir, path) = tmp_bag("read_outcome_ok.mcap");
    // Stamp the bag at the current format explicitly —
    // without recorder.json the reader defaults to v1, so a regression that
    // rejects kind 6 ONLY on the real stamped path would slip this arm. The
    // stamp is `SUPPORTED_TRACE_FORMAT` and NOT a literal,
    // so it tracks the format rather than naming a version that can
    // move.
    let mut atts = replay_grade_attachments(&["node0"]);
    atts.push(recorder_att(Some(
        cerulion_cli_engine::replay_engine::SUPPORTED_TRACE_FORMAT,
    )));
    write_bag(
        &path,
        &[
            fire(0, 1),
            TraceRingRecord::read_outcome(
                1,
                0,
                0,
                ReadOutcomeKind::Served,
                Some(7),
                cerulion_core::trace_ring::ReadRun::once(1),
                ReadSiteRole::Body,
            ),
            TraceRingRecord::read_outcome(
                1,
                0,
                1,
                ReadOutcomeKind::NoFrame,
                None,
                cerulion_core::trace_ring::ReadRun::once(0),
                ReadSiteRole::Body,
            ),
            fire(0, 2),
        ],
        &atts,
        true,
    );
    let err = gate(&path);
    assert!(
        matches!(err, ReplayError::NodeLoad { .. }),
        "a kind-6-bearing bag must pass the record_type gate and reach the engine \
         (NodeLoad for the missing cdylib); got {err:?}"
    );
    assert_eq!(err.exit_code(), 3);
}

#[test]
fn read_outcome_node_idx_out_of_bounds_is_exit_2() {
    // A kind-6 record's node_idx is the CONSUMER node, manifest-indexed like a
    // FIRE's — index 1 into a 1-element table is the first invalid value.
    let (_dir, path) = tmp_bag("read_outcome_idx.mcap");
    write_bag(
        &path,
        &[TraceRingRecord::read_outcome(
            1,
            1,
            0,
            ReadOutcomeKind::Served,
            Some(7),
            cerulion_core::trace_ring::ReadRun::once(1),
            ReadSiteRole::Body,
        )],
        &replay_grade_attachments(&["a"]),
        true,
    );
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { name, reason } => {
            assert!(name.contains("trace_manifest_rank0"), "got {name}");
            assert!(
                reason.contains("READ-OUTCOME")
                    && reason.contains("node_idx 1")
                    && reason.contains("1 node id"),
                "got reason: {reason}"
            );
        }
        other => panic!("expected BagInvalidAttachment (read-outcome node_idx), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

// ── node_idx bounds: exact boundary + empty table ───────────────────────────

#[test]
fn node_idx_equal_to_manifest_len_is_out_of_bounds_exit_2() {
    let (_dir, path) = tmp_bag("boundary_idx.mcap");
    // node_ids = ["a"] (len 1) + FIRE(node_idx = 1): pins the `>=` boundary
    // exactly — index 1 into a 1-element table is the first invalid value.
    write_bag(
        &path,
        &[fire(1, 1)],
        &replay_grade_attachments(&["a"]),
        true,
    );
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { name, reason } => {
            assert!(name.contains("trace_manifest_rank0"), "got {name}");
            assert!(
                reason.contains("node_idx 1") && reason.contains("1 node id"),
                "got reason: {reason}"
            );
        }
        other => panic!("expected BagInvalidAttachment (boundary node_idx), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn empty_node_ids_with_any_fire_is_out_of_bounds_exit_2() {
    let (_dir, path) = tmp_bag("empty_table.mcap");
    // node_ids = [] + FIRE(node_idx = 0): the empty-table case — NO index is
    // valid.
    write_bag(&path, &[fire(0, 1)], &replay_grade_attachments(&[]), true);
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { name, reason } => {
            assert!(name.contains("trace_manifest_rank0"), "got {name}");
            assert!(
                reason.contains("node_idx 0") && reason.contains("0 node id"),
                "got reason: {reason}"
            );
        }
        other => panic!("expected BagInvalidAttachment (empty table), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

// ── manifest body decode failures ───────────────────────────────────────────

#[test]
fn manifest_not_json_is_invalid_attachment_exit_2() {
    let (_dir, path) = tmp_bag("manifest_not_json.mcap");
    let atts = vec![
        att("graph.yaml", "application/yaml", valid_graph_yaml()),
        att(&manifest_name(0), "application/json", b"{not json".to_vec()),
    ];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { name, reason } => {
            assert!(name.contains("trace_manifest_rank0"), "got {name}");
            assert!(reason.contains("did not parse"), "got reason: {reason}");
        }
        other => panic!("expected BagInvalidAttachment (manifest not JSON), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn manifest_missing_node_ids_key_is_invalid_attachment_exit_2() {
    let (_dir, path) = tmp_bag("manifest_no_node_ids.mcap");
    let atts = vec![
        att("graph.yaml", "application/yaml", valid_graph_yaml()),
        // Valid JSON object, but no `node_ids` key — serde refuses it.
        att(&manifest_name(0), "application/json", b"{}".to_vec()),
    ];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { name, reason } => {
            assert!(name.contains("trace_manifest_rank0"), "got {name}");
            assert!(reason.contains("node_ids"), "got reason: {reason}");
        }
        other => panic!("expected BagInvalidAttachment (missing node_ids), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

// ── duplicate-rank manifests: corruption, not multi-process ────────────────

#[test]
fn duplicate_rank_0_manifests_is_invalid_attachment_exit_2() {
    let (_dir, path) = tmp_bag("dup_rank0.mcap");
    // BagWriter allows same-name attachments — write the rank-0 manifest
    // TWICE. That is corruption (a rank recorded twice), NOT a multi-process
    // recording, and must not mislabel as a multi-rank gap/seam.
    let atts = vec![
        att("graph.yaml", "application/yaml", valid_graph_yaml()),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["n0"]),
        ),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["n0"]),
        ),
    ];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { name, reason } => {
            assert!(name.contains("trace_manifest_rank0"), "got {name}");
            assert!(
                reason.contains("duplicate trace manifest for rank 0"),
                "got reason: {reason}"
            );
        }
        other => panic!("expected BagInvalidAttachment (duplicate rank), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

#[test]
fn duplicate_rank_via_leading_zero_names_is_invalid_attachment_exit_2() {
    let (_dir, path) = tmp_bag("dup_rank_leading_zero.mcap");
    // Two DIFFERENT attachment names that parse to the SAME rank via a leading
    // zero (`rank0` and `rank00` both parse to 0). This is parsed-rank
    // duplication (corruption), and must be caught as BagInvalidAttachment — a
    // NAME-keyed dedup would see two distinct names, miss it, and mislabel as
    // a multi-rank gap/seam. The error must name BOTH real offending
    // attachments (not a synthesized canonical name that could match neither).
    let rank0 = "__cerulion/trace_manifest_rank0.json";
    let rank00 = "__cerulion/trace_manifest_rank00.json";
    let atts = vec![
        att("graph.yaml", "application/yaml", valid_graph_yaml()),
        att(rank0, "application/json", manifest_bytes(0, &["n0"])),
        att(rank00, "application/json", manifest_bytes(0, &["n0"])),
    ];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { name, reason } => {
            // `name` is the FIRST real offending attachment (rank0 sorts before
            // rank00 by the name tie-break), not a synthesized canonical name.
            assert_eq!(name, rank0, "got {name}");
            assert!(
                reason.contains("duplicate trace manifest for rank 0"),
                "got reason: {reason}"
            );
            // BOTH real names appear — the discriminating pin vs the old
            // synthesized-name behavior.
            assert!(reason.contains(rank0), "reason missing {rank0}: {reason}");
            assert!(reason.contains(rank00), "reason missing {rank00}: {reason}");
        }
        other => {
            panic!("expected BagInvalidAttachment (leading-zero duplicate rank), got {other:?}")
        }
    }
    assert_eq!(err.exit_code(), 2);
}

// ── non-UTF-8 graph.yaml ────────────────────────────────────────────────────

#[test]
fn graph_yaml_non_utf8_is_invalid_attachment_exit_2() {
    let (_dir, path) = tmp_bag("graph_non_utf8.mcap");
    let atts = vec![
        // Invalid UTF-8 bytes (0xFF/0xFE are never valid UTF-8 sequences).
        att(
            "graph.yaml",
            "application/yaml",
            vec![0xFF, 0xFE, 0x00, 0x9F],
        ),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["node0"]),
        ),
    ];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { name, reason } => {
            assert_eq!(name, "graph.yaml");
            assert!(reason.contains("UTF-8"), "got reason: {reason}");
        }
        other => panic!("expected BagInvalidAttachment (non-UTF-8), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

// ── gates pass → the engine loads the CURRENT workspace's candidate cdylibs ──
//
// Once every replay-grade gate passes, `run_replay` hands off
// to the engine, which loads the graph's node cdylibs from the workspace build.
// These bags declare node type `test_pub`, which does NOT exist in this crate's
// build tree (the test CWD is `cerulion_cli_engine/`, not a node workspace), so
// the engine fails at load time with a precise `NodeLoad` (exit 3) BEFORE any
// iceoryx2 / transport state is created — keeping these tests iceoryx2-free and
// parallel-safe. (The happy-path replay is exercised end-to-end with in-process
// factories in `replay_engine_test.rs`.)

#[test]
fn full_replay_grade_bag_reaches_engine_and_fails_node_load_exit_3() {
    let (_dir, path) = tmp_bag("replay_grade.mcap");
    write_bag(
        &path,
        &[fire(0, 1), fire(0, 2)],
        &replay_grade_attachments(&["node0"]),
        true,
    );
    let err = gate(&path);
    match &err {
        ReplayError::NodeLoad { node_type, .. } => assert_eq!(node_type, "test_pub"),
        other => panic!("expected NodeLoad for the missing `test_pub` cdylib, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 3);
}

// ── What NAMES a resim ───────────────────────────────────────────
//
// The file stem is the graph's identity, and `graph create` writes no
// `name:`. A bag's embedded `graph.yaml` has no file to stem from, so
// a bag recorded from such a graph is nameless, and a resim that
// called itself `unnamed` would be cosmetically loud and indistinguishable: two
// concurrent resims of two different bags would both mint `cerulion_replay_unnamed`.
//
// The identity is not lost: `graph_cmd::resolve_recording_paths`
// stems the bag BY it. These two arms drive the REAL `run_replay` gate, because
// the pure oracles in `replay_cmd` cannot see a fallback that exists and is
// never called (both bags still fail `NodeLoad` afterwards, which is unrelated
// and asserted only so a gate regression cannot masquerade as an adoption one).

/// An embed from a current graph: no `name:` line at all, which is what `graph create`
/// writes and what `render_effective_graph_yaml`
/// therefore serializes.
fn nameless_graph_yaml() -> Vec<u8> {
    b"prefix: replaytest\nnodes:\n  - id: pub1\n    type: test_pub\n    \
      outputs:\n      - name: data\n        schema: test/Data\n"
        .to_vec()
}

/// `logs_assert` predicate: some captured line carries `key=value` as a WHOLE
/// whitespace token.
///
/// A bare `contains` would match a PREFIX (`graph=x` inside
/// `graph=xy`) and would also be satisfied by the same text appearing
/// in prose.
fn has_field(lines: &[&str], field: &str) -> bool {
    lines
        .iter()
        .any(|l| l.split_whitespace().any(|tok| tok == field))
}

/// THE RULE: a nameless embed makes the resim take the BAG's file stem.
#[test]
#[tracing_test::traced_test]
fn a_nameless_embed_is_named_by_the_bag_file() {
    let (_dir, path) = tmp_bag("named_by_the_bag_20260101T000000Z.mcap");
    let atts = vec![
        att("graph.yaml", "application/yaml", nameless_graph_yaml()),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["node0"]),
        ),
    ];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    assert!(
        matches!(err, ReplayError::NodeLoad { .. }),
        "the bag must still reach the engine; got {err:?}"
    );
    logs_assert(|lines: &[&str]| {
        // The gate logs the identity IN FORCE (`config.identity()`), so this is
        // an observable of the ASSIGNMENT, not of the resolver having produced
        // a candidate — logging the stem without assigning it prints
        // `graph=unnamed` here.
        if !has_field(lines, "graph=named_by_the_bag_20260101T000000Z") {
            return Err(format!(
                "a nameless embed must be named by the BAG FILE — no line carried \
                 `graph=named_by_the_bag_20260101T000000Z`. Captured:\n{}",
                lines.join("\n")
            ));
        }
        if has_field(lines, "graph=unnamed") {
            return Err(format!(
                "…and nothing may still be calling this resim `unnamed`. Captured:\n{}",
                lines.join("\n")
            ));
        }
        Ok(())
    });
}

/// ANTI-TAUTOLOGY / back-compat: a bag that named itself is left alone.
///
/// Without this arm the rule is satisfied by "always rename the resim after the
/// file", which would change the identity of every older bag — bags that
/// work. `valid_graph_yaml()` declares `name: replaytest` while the bag
/// file is named something else, so the two answers are distinguishable.
#[test]
#[tracing_test::traced_test]
fn an_embed_that_named_itself_keeps_its_name() {
    let (_dir, path) = tmp_bag("a_totally_different_bag_name.mcap");
    write_bag(
        &path,
        &[fire(0, 1)],
        &replay_grade_attachments(&["node0"]),
        true,
    );
    let err = gate(&path);
    assert!(
        matches!(err, ReplayError::NodeLoad { .. }),
        "the bag must still reach the engine; got {err:?}"
    );
    logs_assert(|lines: &[&str]| {
        if has_field(lines, "graph=a_totally_different_bag_name") {
            return Err(format!(
                "an older bag's declared `name:` must WIN — the bag file must not \
                 rename it. Captured:\n{}",
                lines.join("\n")
            ));
        }
        // The POSITIVE half, in the same body: the declared name really is what
        // is in force. Without it the absence above is satisfied by a gate that
        // resolved no identity at all (or logged nothing), which is the vacuous
        // reading of a negative assertion.
        if !has_field(lines, "graph=replaytest") {
            return Err(format!(
                "the declared `name: replaytest` must be the identity in force — no line \
                 carried `graph=replaytest`. Captured:\n{}",
                lines.join("\n")
            ));
        }
        Ok(())
    });
}

/// A UNICODE bag name must not PANIC the resim.
///
/// UTF-8 is not sufficient for an iceoryx2 node name (`StaticString` refuses any
/// byte at or above U+0080), and `TransportManager::init` `.expect()`s that
/// conversion, so a fallback that adopted `naïve` turned a working resim into a
/// crash with no exit code and no diagnostic. The pure oracles cannot see this:
/// they never reach the gate, and a panic is not a return value.
///
/// SCOPE, measured rather than assumed: this arm does NOT reproduce the
/// crash. `run_replay` fails at `NodeLoad` (the test CWD has no `test_pub`
/// cdylib) BEFORE the engine reaches `TransportManager::init`, so what is
/// pinned here is the PRECONDITION — the unrepresentable stem must never be
/// ADOPTED. With an inert check this arm fails on exactly that, with
/// `graph=naïve_café_20260101T000000Z` in its own diagnostic; in a real
/// workspace that same adoption is what panics. Reproducing the abort itself
/// would need a bag whose cdylibs resolve, which is `replay_cli_test`'s
/// prebuilt-fixture territory and would trade an attributable assertion for a
/// SIGABRT.
///
/// It then pins the correct outcome: the run keeps going as `unnamed`, naming
/// the cause, which is what the decline is for.
#[test]
#[tracing_test::traced_test]
fn a_unicode_bag_name_declines_rather_than_panicking() {
    let (_dir, path) = tmp_bag("naïve_café_20260101T000000Z.mcap");
    let atts = vec![
        att("graph.yaml", "application/yaml", nameless_graph_yaml()),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["node0"]),
        ),
    ];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    assert!(
        matches!(err, ReplayError::NodeLoad { .. }),
        "the bag must still reach the engine; got {err:?}"
    );
    logs_assert(|lines: &[&str]| {
        // The stem must NOT have been adopted — that is the panic path.
        if has_field(lines, "graph=naïve_café_20260101T000000Z") {
            return Err(format!(
                "a stem iceoryx2 cannot name must not be adopted — that is what \
                 panics `TransportManager::init`. Captured:\n{}",
                lines.join("\n")
            ));
        }
        // …and the run must say so, naming the cause,
        // rather than declining silently.
        if !has_field(lines, "graph=unnamed") {
            return Err(format!(
                "a declined resim must still report the identity in force. \
                 Captured:\n{}",
                lines.join("\n")
            ));
        }
        if !lines.iter().any(|l| l.contains("U+0080")) {
            return Err(format!(
                "…and must name the CAUSE, not merely decline. Captured:\n{}",
                lines.join("\n")
            ));
        }
        Ok(())
    });
}

#[test]
fn env_json_is_optional_replay_grade_without_it_reaches_engine() {
    let (_dir, path) = tmp_bag("no_env.mcap");
    // No env.json — the gate is SKIPPED (optional), so the bag still reaches the
    // engine and fails the same way (the missing `test_pub` cdylib) — proving
    // env.json absence is not a gate failure.
    let atts = vec![
        att("graph.yaml", "application/yaml", valid_graph_yaml()),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["node0"]),
        ),
    ];
    write_bag(&path, &[fire(0, 1)], &atts, true);
    let err = gate(&path);
    assert!(
        matches!(err, ReplayError::NodeLoad { .. }),
        "env.json is optional; the bag must still reach the engine; got {err:?}"
    );
    assert_eq!(err.exit_code(), 3);
}

// ── Discard-marker gate behavior (rank() masking + version gate) ────

use cerulion_core::trace_ring::TRACE_DISCARD_BIT;

/// A recorder.json attachment. `trace_format = None` OMITS the field entirely
/// (models a bag from before the field existed ⇒ serde default v1); `Some(n)` writes it.
///
/// No `coordination` key either (the pre-stamp shape), which
/// the read side resolves to INFERRED lockstep. A bag that stamps one is built
/// by [`production_recorder_att`], through the production renderer.
fn recorder_att(trace_format: Option<u32>) -> Att {
    let mut obj = serde_json::json!({
        "arch": std::env::consts::ARCH,
        "os": std::env::consts::OS,
        "cerulion_version": "0.1.0",
        "recorded_at_ns": 42u64,
    });
    if let Some(tf) = trace_format {
        obj["trace_format"] = serde_json::json!(tf);
    }
    att(
        "__cerulion/recorder.json",
        "application/json",
        serde_json::to_vec(&obj).unwrap(),
    )
}

/// A recorder.json rendered by the PRODUCTION writer
/// (`graph_cmd::render_recorder_json`) for `coordination`.
///
/// Deliberately not hand-built: no production route resolves `FreeRun` until
/// 2d, so a hand-written free-run attachment is free to drift from what 2d's
/// recorder will actually write — including its `trace_format` stamp, which is
/// derived from the mode rather than chosen. Going through the real renderer
/// makes the crafted bag's identity byte-identical to a real one's.
fn production_recorder_att(coordination: CoordinationMode) -> Att {
    att(
        "__cerulion/recorder.json",
        "application/json",
        cerulion_cli_engine::graph_cmd::render_recorder_json(
            std::time::Duration::from_nanos(1_783_092_278_948_123_456),
            coordination,
        ),
    )
}

/// A `__cerulion/recorder.json` attachment written from
/// RAW BYTES — the injector for the CONTRACT-CARRIER arms below.
///
/// [`recorder_att`] renders a well-formed document and cannot express the
/// shapes those arms need: bytes that are not JSON, JSON that is not an
/// object, a `trace_format` that is not a version, and a `coordination` value
/// this binary has no variant for. Each of those is a document a NEWER
/// Cerulion (or a corrupted bag) can really present, and the whole point of
/// the contract-carrier gate is what replay does when it meets one.
fn recorder_att_raw(bytes: &[u8]) -> Att {
    att(
        "__cerulion/recorder.json",
        "application/json",
        bytes.to_vec(),
    )
}

/// A recorder.json object with an ARBITRARY `trace_format` / `coordination`
/// value — including values no enum and no `u32` can hold. The advisory half
/// (arch/os/version) is always well-formed, so an arm that refuses is refusing
/// on the CONTRACT key it names and nothing else.
fn recorder_att_with(
    trace_format: Option<serde_json::Value>,
    coordination: Option<serde_json::Value>,
) -> Att {
    let mut obj = serde_json::json!({
        "arch": std::env::consts::ARCH,
        "os": std::env::consts::OS,
        "cerulion_version": "0.1.0",
        "recorded_at_ns": 42u64,
    });
    if let Some(tf) = trace_format {
        obj["trace_format"] = tf;
    }
    if let Some(c) = coordination {
        obj["coordination"] = c;
    }
    recorder_att_raw(&serde_json::to_vec(&obj).unwrap())
}

/// Adversarial (b): a discard-MARKED FIRE record whose masked rank is OUT OF
/// RANGE (rank 5 on a single-manifest bag) is still rejected — `rank()` strips
/// the discard bit, the bounds check bites the real rank. Kills a "mask hides an
/// out-of-range rank" regression.
#[test]
fn marked_fire_with_out_of_range_rank_is_recording_inconsistent_exit_2() {
    let (_dir, path) = tmp_bag("marked_oob_rank.mcap");
    let mut marked_oob = fire(0, 1);
    marked_oob.reserved = 5 | TRACE_DISCARD_BIT; // rank 5 (no manifest) + discard bit
    write_bag(
        &path,
        &[fire(0, 0), marked_oob],
        &replay_grade_attachments(&["node0"]),
        true,
    );
    let err = gate(&path);
    match &err {
        ReplayError::RecordingInconsistent { detail } => {
            // The message reports the MASKED rank (5), not the raw 0x8000_0005.
            assert!(
                detail.contains("rank 5"),
                "the masked rank is reported, not the raw reserved: {detail}"
            );
        }
        other => panic!("expected RecordingInconsistent (masked-rank OOB), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

/// Positive control: a LEGITIMATELY discard-marked FIRE record (`reserved = 0 |
/// DISCARD_BIT`, rank 0) PASSES the rank/record-type gate — WITHOUT the `rank()`
/// mask the raw `0x8000_0000 >= 1` bounds check would have wrongly refused it as
/// RecordingInconsistent. It gets PAST the gate (the failure that surfaces is the
/// downstream cdylib NodeLoad, exit 3 — NOT a gate refusal). Proves the gate
/// ACCEPTS marked records, not just rejects corrupt ones.
#[test]
fn legitimately_marked_fire_passes_the_rank_gate() {
    let (_dir, path) = tmp_bag("marked_ok.mcap");
    let marked = TraceRingRecord {
        reserved: TRACE_DISCARD_BIT, // rank 0, discard bit set
        ..fire(0, 1)
    };
    write_bag(
        &path,
        &[fire(0, 0), marked],
        &replay_grade_attachments(&["node0"]),
        true,
    );
    let err = gate(&path);
    // It must NOT be the rank/record-type refusal — the marked record cleared the
    // gate. (Some later stage fails: cdylib NodeLoad in a non-workspace CWD.)
    assert!(
        !matches!(
            err,
            ReplayError::RecordingInconsistent { .. }
                | ReplayError::DegradedRecordingDeparture { .. }
                | ReplayError::TraceRecordUnsupported { .. }
        ),
        "a legitimately-marked rank-0 FIRE record must clear the gate; got {err:?}"
    );
}

/// Version gate — NEWER bag: a `trace_format` GREATER than this binary supports
/// is refused up front with the explicit "newer Cerulion; upgrade to replay"
/// message (exit 2), NOT a generic mis-decode.
#[test]
fn newer_trace_format_is_refused_with_upgrade_message_exit_2() {
    let (_dir, path) = tmp_bag("newer_format.mcap");
    let mut atts = replay_grade_attachments(&["node0"]);
    // Far newer than SUPPORTED_TRACE_FORMAT (whatever it currently is — the
    // boundary itself is pinned by `supported_trace_format_at_boundary_is_accepted`).
    atts.push(recorder_att(Some(999)));
    write_bag(&path, &[fire(0, 0), fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::RecordingInconsistent { detail } => {
            assert!(
                detail.contains("version 999")
                    && detail.to_lowercase().contains("newer")
                    && detail.to_lowercase().contains("upgrade"),
                "expected an explicit upgrade message, got: {detail}"
            );
        }
        other => panic!("expected RecordingInconsistent (newer trace_format), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

/// Version gate — ABSENT field is v1 (back-compat): a recorder.json WITHOUT a
/// `trace_format` field must NOT be version-refused; it clears the version gate
/// (the later failure is the downstream cdylib NodeLoad, not a version refusal).
#[test]
fn absent_trace_format_is_version_1_backcompat() {
    let (_dir, path) = tmp_bag("absent_format.mcap");
    let mut atts = replay_grade_attachments(&["node0"]);
    atts.push(recorder_att(None)); // legacy recorder.json (no trace_format)
    write_bag(&path, &[fire(0, 0), fire(0, 1)], &atts, true);
    let err = gate(&path);
    // The version gate did not fire — the "upgrade to replay" message is absent.
    if let ReplayError::RecordingInconsistent { detail } = &err {
        assert!(
            !detail.to_lowercase().contains("newer cerulion"),
            "an absent trace_format must be treated as v1, not refused: {detail}"
        );
    }
}

/// Version gate — trace_format == SUPPORTED is accepted (the boundary): a bag at
/// exactly the supported version clears the version gate.
#[test]
fn supported_trace_format_at_boundary_is_accepted() {
    let (_dir, path) = tmp_bag("supported_format.mcap");
    let mut atts = replay_grade_attachments(&["node0"]);
    // The CONSTANT, not a literal — this test is the boundary pin, so it must
    // track a bump (it has moved before: 2 → 3) instead of silently becoming a
    // below-boundary case.
    atts.push(recorder_att(Some(
        cerulion_cli_engine::replay_engine::SUPPORTED_TRACE_FORMAT,
    )));
    write_bag(&path, &[fire(0, 0), fire(0, 1)], &atts, true);
    let err = gate(&path);
    if let ReplayError::RecordingInconsistent { detail } = &err {
        assert!(
            !detail.to_lowercase().contains("newer cerulion"),
            "trace_format == SUPPORTED must be accepted: {detail}"
        );
    }
}

// ── The coordination stamp at the replay ENTRY ───────────────────────────

use cerulion_cli_engine::replay_engine::CoordinationMode;

/// BOTH coordination stamps clear the replay ENTRY.
///
/// A `coordination: free_run` bag would have to be refused here if no executor
/// could apply its contract, since replaying it under the
/// LOCKSTEP contract would be a confident wrong answer. The engine has that
/// executor (sequential per-rank re-execution), so there is no
/// refusal, and this arm is its inverse: the entry gate does not
/// discriminate on coordination at all, and both stamps fall through to the
/// downstream cdylib `NodeLoad` — the failure every bag in this file reaches
/// once it clears the gates.
///
/// The DISCRIMINATOR lives downstream rather than at the entry, and it is worth naming
/// where: the two stamps are still replayed under different contracts, and
/// which one was applied is read off the verdict's `coordination` provenance
/// line and the `--report` JSON (`replay_engine_test`'s
/// `free_run_crafted_bag_*` arms assert that end). A gate test cannot
/// see a contract; it can only see whether the bag was let through, and that
/// is exactly what this asserts.
///
/// Note what the entry gate does NOT key on: this bag's `trace_format` is whatever
/// the PRODUCTION writer stamps (`SUPPORTED_TRACE_FORMAT`),
/// so the version gate deliberately
/// lets it through.
/// Every record in it decodes. That is why coordination needs its own gate rather than
/// leaning on the version number — and why not refusing here is a statement
/// about the EXECUTOR, not about the format.
#[test]
fn both_coordination_stamps_clear_the_replay_entry() {
    for (bag_name, mode) in [
        ("free_run_entry.mcap", CoordinationMode::FreeRun),
        ("lockstep_entry.mcap", CoordinationMode::Lockstep),
    ] {
        let (_dir, path) = tmp_bag(bag_name);
        let mut atts = replay_grade_attachments(&["node0"]);
        atts.push(production_recorder_att(mode));
        write_bag(&path, &[fire(0, 0), fire(0, 1)], &atts, true);
        let err = gate(&path);
        assert!(
            matches!(err, ReplayError::NodeLoad { .. }),
            "{bag_name}: a {mode:?} bag must clear the coordination gate and reach the \
             downstream node load like every other gated bag; got {err:?}"
        );
    }
}

/// A bag recorded before the stamp existed is not refused: the absent stamp
/// resolves to lockstep, which is the contract it was in fact recorded under.
/// The back-compat half of the same gate.
///
/// TWO shapes, structurally
/// different: a recorder.json PRESENT but carrying no `coordination` key, and
/// NO recorder.json at all. An unreadable attachment does not land
/// here (folding it to "absent" would inherit this inference); an
/// unreadable one REFUSES, so **the absent attachment is the only
/// inferred-lockstep path** — and it is the one where the inference is true by
/// construction, since a bag that says nothing was written before there was
/// anything to say. Both halves belong in one body precisely because the
/// arms below turn the two apart.
#[test]
fn a_pre_1289_bag_with_no_coordination_key_is_not_refused() {
    for (bag_name, recorder) in [
        // Attachment present, contract key absent (a bag from before the stamp).
        ("no_coord_entry.mcap", Some(recorder_att(Some(3)))),
        // No attachment at all (a pre-recorder.json bag).
        ("no_recorder_entry.mcap", None),
    ] {
        let (_dir, path) = tmp_bag(bag_name);
        let mut atts = replay_grade_attachments(&["node0"]);
        atts.extend(recorder);
        write_bag(&path, &[fire(0, 0), fire(0, 1)], &atts, true);
        let err = gate(&path);
        // The entry does not refuse on coordination at all, so
        // what this arm pins is the INFERENCE — an absent stamp must not be
        // mistaken for an UNKNOWN one, which IS still refused (the arm below).
        assert!(
            !matches!(&err, ReplayError::BagInvalidAttachment { name, .. }
                if name == "__cerulion/recorder.json"),
            "{bag_name}: saying nothing is not the same as saying something unreadable; \
             got {err:?}"
        );
        assert!(
            matches!(err, ReplayError::NodeLoad { .. }),
            "{bag_name}: …and it reaches the downstream node load like every other gated \
             bag; got {err:?}"
        );
    }
}

// ── recorder.json is a CONTRACT CARRIER ─────────────────────────────────
//
// The attachment carries host identity — advisory — but it ALSO carries
// `trace_format` and `coordination`, and BOTH gates
// live behind "did this parse?". If a parse failure folded to "absent", a malformed document would skip the
// version refusal AND the coordination gate, and the verdict would announce
// `(inferred: no coordination stamp)`, a positive claim about a bag whose
// stamp is right there and unreadable.
//
// These arms belong HERE and not in `replay_engine_test.rs` because both gates
// live at the CLI entry (`run_replay`), above the engine seam: the engine is
// deliberately left drivable for crafted free-run bags, which is exactly why
// the refusals are not in `run_engine`.

/// Bytes that are not JSON at all REFUSE (exit 2), naming what was lost.
///
/// The message has to name the CONTRACT keys rather than say "the host
/// identity is unavailable": the reason this is fatal is that
/// `trace_format`/`coordination` decide WHICH CONTRACT the recording is
/// replayed under, and a reader who is told only that an advisory is missing
/// would reasonably retry with the same bag.
#[test]
fn a_recorder_json_that_is_not_json_is_refused_naming_what_was_lost() {
    let (_dir, path) = tmp_bag("recorder_not_json.mcap");
    let mut atts = replay_grade_attachments(&["node0"]);
    atts.push(recorder_att_raw(b"{not json at all"));
    write_bag(&path, &[fire(0, 0), fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { name, reason } => {
            assert_eq!(name, "__cerulion/recorder.json");
            assert!(
                reason.contains("trace_format") && reason.contains("coordination"),
                "the refusal names the CONTRACT keys that were lost: {reason}"
            );
            assert!(
                reason.contains("did not parse as JSON"),
                "…and what went wrong with the document: {reason}"
            );
            assert!(
                reason.contains("Re-record") || reason.contains("re-record"),
                "…and the remedy: {reason}"
            );
        }
        other => panic!("expected BagInvalidAttachment recorder.json, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2, "a bag replay cannot read is exit 2");
}

/// VALID JSON of the wrong SHAPE refuses too — a distinct arm because it is a
/// distinct production branch (`serde_json::from_slice` SUCCEEDS on
/// `[1,2,3]`, and it is the `as_object` check that catches it).
#[test]
fn a_recorder_json_that_is_not_an_object_is_refused_naming_the_contract_keys() {
    let (_dir, path) = tmp_bag("recorder_not_object.mcap");
    let mut atts = replay_grade_attachments(&["node0"]);
    atts.push(recorder_att_raw(b"[1, 2, 3]"));
    write_bag(&path, &[fire(0, 0), fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::BagInvalidAttachment { name, reason } => {
            assert_eq!(name, "__cerulion/recorder.json");
            assert!(
                reason.contains("expected a JSON object"),
                "the refusal names the shape it needed: {reason}"
            );
            assert!(
                reason.contains("trace_format") && reason.contains("coordination"),
                "…and why the shape matters: {reason}"
            );
        }
        other => panic!("expected BagInvalidAttachment (non-object), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

/// A `trace_format` that is PRESENT but is not a version REFUSES rather than
/// being assumed — it is the whole input to the version gate, and
/// guessing a version is precisely the hazard that gate exists for.
///
/// Two shapes in one body because they take different `as_u64` paths: a
/// STRING (never a number) and a NEGATIVE number (a number, not a `u32`).
/// Neither may fall back to v1: a bag whose format key is unreadable is not a
/// legacy bag, it is a bag this binary cannot classify.
#[test]
fn a_trace_format_that_is_not_a_version_is_refused_rather_than_assumed() {
    for (name, value) in [
        ("tf_string.mcap", serde_json::json!("three")),
        ("tf_negative.mcap", serde_json::json!(-1)),
    ] {
        let (_dir, path) = tmp_bag(name);
        let mut atts = replay_grade_attachments(&["node0"]);
        atts.push(recorder_att_with(Some(value), None));
        write_bag(&path, &[fire(0, 0), fire(0, 1)], &atts, true);
        let err = gate(&path);
        match &err {
            ReplayError::BagInvalidAttachment {
                name: att_name,
                reason,
            } => {
                assert_eq!(att_name, "__cerulion/recorder.json", "{name}");
                assert!(
                    reason.contains("trace_format"),
                    "{name}: the refusal names the key: {reason}"
                );
                assert!(
                    reason.contains("refused rather than assumed"),
                    "{name}: …and says it refuses rather than guessing: {reason}"
                );
            }
            other => panic!("{name}: expected BagInvalidAttachment (trace_format), got {other:?}"),
        }
        assert_eq!(err.exit_code(), 2, "{name}");
    }
}

/// An UNKNOWN `coordination` value on a format this binary DOES support is
/// refused BY NAME (exit 2) — never inferred to lockstep.
///
/// "Assume lockstep" is exactly the wrong guess here: a value with no variant
/// was written by a NEWER Cerulion, so the recording was taken under a
/// contract this binary cannot apply, and applying the lockstep one would
/// either refuse a correct bag as corrupt or mis-anchor it silently.
///
/// The `trace_format` is pinned at `SUPPORTED_TRACE_FORMAT` — the CONSTANT,
/// not a literal — so this arm keeps testing the coordination gate across a
/// future version bump instead of silently becoming the version arm below.
#[test]
fn an_unknown_coordination_value_on_a_supported_format_is_refused_by_name() {
    let (_dir, path) = tmp_bag("coord_unknown.mcap");
    let mut atts = replay_grade_attachments(&["node0"]);
    atts.push(recorder_att_with(
        Some(serde_json::json!(
            cerulion_cli_engine::replay_engine::SUPPORTED_TRACE_FORMAT
        )),
        Some(serde_json::json!("quantum_entangled")),
    ));
    write_bag(&path, &[fire(0, 0), fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::RecordingInconsistent { detail } => {
            assert!(
                detail.contains("quantum_entangled"),
                "the refusal quotes the VALUE it could not read: {detail}"
            );
            assert!(detail.contains("coordination"), "…names the key: {detail}");
            assert!(
                detail.to_uppercase().contains("NEWER") && detail.contains("upgrade"),
                "…and says where the support is: {detail}"
            );
            assert!(
                !detail.contains("version"),
                "an unknown VALUE is not a version refusal — that arm is separate: {detail}"
            );
        }
        other => panic!("expected RecordingInconsistent (unknown coordination), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

/// ANTI-TAUTOLOGY for the arm above: the SAME document with a KNOWN
/// coordination value at the SAME `trace_format` is NOT refused.
///
/// Without it, "an unknown value is refused" is satisfied by a gate that
/// refuses every bag carrying a `coordination` key at all.
#[test]
fn a_known_coordination_value_on_a_supported_format_is_not_refused() {
    let (_dir, path) = tmp_bag("coord_known.mcap");
    let mut atts = replay_grade_attachments(&["node0"]);
    atts.push(recorder_att_with(
        Some(serde_json::json!(
            cerulion_cli_engine::replay_engine::SUPPORTED_TRACE_FORMAT
        )),
        Some(serde_json::json!("lockstep")),
    ));
    write_bag(&path, &[fire(0, 0), fire(0, 1)], &atts, true);
    let err = gate(&path);
    assert!(
        matches!(err, ReplayError::NodeLoad { .. }),
        "a hand-written but KNOWN stamp clears both contract gates and reaches the \
         downstream node load; got {err:?}"
    );
}

/// ORDERING: when a bag is BOTH from a newer format AND carries an unknown
/// coordination value, the VERSION refusal wins.
///
/// Both facts are true at once and they are ONE fact — a bag from a newer
/// Cerulion. The version message states the CAUSE ("recorded by a NEWER
/// Cerulion; upgrade to replay it"); the unknown-value message states only a
/// SYMPTOM. So the more informative refusal is the one an operator gets, and
/// the unknown-value arm above is what catches the same skew on a bag whose
/// format version this binary DOES support.
///
/// This arm exists because the ordering is otherwise invisible: both refusals
/// are `RecordingInconsistent` at exit 2, so swapping the two blocks changes
/// only the TEXT — and the text is the whole remedy.
#[test]
fn a_newer_format_wins_over_the_unknown_coordination_value() {
    let (_dir, path) = tmp_bag("coord_unknown_newer.mcap");
    let mut atts = replay_grade_attachments(&["node0"]);
    let newer = cerulion_cli_engine::replay_engine::SUPPORTED_TRACE_FORMAT + 1;
    atts.push(recorder_att_with(
        Some(serde_json::json!(newer)),
        Some(serde_json::json!("quantum_entangled")),
    ));
    write_bag(&path, &[fire(0, 0), fire(0, 1)], &atts, true);
    let err = gate(&path);
    match &err {
        ReplayError::RecordingInconsistent { detail } => {
            assert!(
                detail.contains(&format!("version {newer}")),
                "the VERSION refusal is the one served: {detail}"
            );
            assert!(
                detail.to_lowercase().contains("newer")
                    && detail.to_lowercase().contains("upgrade"),
                "…with its upgrade remedy: {detail}"
            );
            assert!(
                !detail.contains("quantum_entangled"),
                "…and NOT the unknown-value message, which states a symptom rather than \
                 the cause: {detail}"
            );
        }
        other => panic!("expected RecordingInconsistent (version wins), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
}

// ── The `--tolerance` PRE-FLIGHT gate (exit 4) ──────────────
//
// The tolerance document is parsed + validated AFTER the bag's graph attachment
// loads but BEFORE the engine constructs the runtime, so an invalid tolerance
// is exit 4 and PREEMPTS the node-load exit 3 these bags otherwise hit (the
// candidate `camera` cdylib does not exist in this crate's build tree). A valid
// tolerance is transparent — the bag reaches the same node-load exit 3 as no
// tolerance at all (the diff stays byte-exact, so a valid document
// changes nothing before the engine).

/// A graph whose one produced output carries a BUILT-IN schema
/// (`sensor_msgs/Image`), so the field registry can resolve field paths
/// (`height`, `width`, …) for the typo tests. Produced topic:
/// `/replaytest/cam/image`.
fn image_graph_yaml() -> Vec<u8> {
    b"name: replaytest\nprefix: replaytest\nnodes:\n  - id: cam\n    type: camera\n    \
      outputs:\n      - name: image\n        schema: sensor_msgs/Image\n"
        .to_vec()
}

/// Replay-grade attachments for the image graph (rank-0 manifest names `cam`,
/// matching the FIRE records' node_idx 0).
fn image_replay_attachments() -> Vec<Att> {
    vec![
        att("graph.yaml", "application/yaml", image_graph_yaml()),
        att(
            "env.json",
            "application/json",
            br#"{"RUST_LOG":"info"}"#.to_vec(),
        ),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["cam"]),
        ),
    ]
}

/// Write a full replay-grade image bag at `path`.
fn write_image_bag(path: &Path) {
    write_bag(
        path,
        &[fire(0, 1), fire(0, 2)],
        &image_replay_attachments(),
        true,
    );
}

/// Run the gate with a tolerance YAML written to a sibling temp file.
fn gate_with_tolerance(dir: &tempfile::TempDir, bag: &Path, tol_yaml: &str) -> ReplayError {
    let tol = dir.path().join("tolerance.yaml");
    std::fs::write(&tol, tol_yaml).unwrap();
    run_replay(
        bag,
        ReplayOptions {
            tolerance_path: Some(tol),
            ..Default::default()
        },
    )
    .unwrap_err()
}

#[test]
fn tolerance_unknown_key_is_exit_4() {
    let (dir, path) = tmp_bag("tol_unknown.mcap");
    write_image_bag(&path);
    let err = gate_with_tolerance(&dir, &path, "defualt_metric:\n  kind: bit_exact\n");
    match &err {
        ReplayError::ToleranceInvalid { reason } => assert!(
            reason.contains("defualt_metric"),
            "reason names the bad key: {reason}"
        ),
        other => panic!("expected ToleranceInvalid, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 4);
}

#[test]
fn tolerance_out_of_range_min_iou_is_exit_4() {
    let (dir, path) = tmp_bag("tol_range.mcap");
    write_image_bag(&path);
    let err = gate_with_tolerance(
        &dir,
        &path,
        "default_metric:\n  kind: bbox_iou\n  min_iou: 1.5\n",
    );
    match &err {
        ReplayError::ToleranceInvalid { reason } => {
            assert!(
                reason.contains("min_iou") && reason.contains("1.5"),
                "{reason}"
            )
        }
        other => panic!("expected ToleranceInvalid, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 4);
}

#[test]
fn tolerance_typoed_topic_is_exit_4_with_suggestion() {
    let (dir, path) = tmp_bag("tol_topic.mcap");
    write_image_bag(&path);
    let err = gate_with_tolerance(&dir, &path, "topics:\n  /replaytest/cam/imag: {}\n");
    match &err {
        ReplayError::ToleranceInvalid { reason } => assert!(
            reason.contains("/replaytest/cam/imag")
                && reason.contains("did you mean")
                && reason.contains("/replaytest/cam/image"),
            "topic suggestion: {reason}"
        ),
        other => panic!("expected ToleranceInvalid, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 4);
}

#[test]
fn tolerance_typoed_field_is_exit_4_with_suggestion() {
    let (dir, path) = tmp_bag("tol_field.mcap");
    write_image_bag(&path);
    let err = gate_with_tolerance(
        &dir,
        &path,
        "topics:\n  /replaytest/cam/image:\n    fields:\n      heigth:\n        kind: max_abs\n        threshold: 0.1\n",
    );
    match &err {
        ReplayError::ToleranceInvalid { reason } => assert!(
            reason.contains("heigth")
                && reason.contains("did you mean")
                && reason.contains("height"),
            "field suggestion: {reason}"
        ),
        other => panic!("expected ToleranceInvalid, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 4);
}

#[test]
fn tolerance_composed_topic_and_field_typo_names_both() {
    // The category-A load-bearing pin over a REAL bag: BOTH the topic
    // (`/replaytest/cam/imag`) and the field (`heigth`) are typoed; the single
    // exit-4 error names BOTH corrections.
    let (dir, path) = tmp_bag("tol_composed.mcap");
    write_image_bag(&path);
    let err = gate_with_tolerance(
        &dir,
        &path,
        "topics:\n  /replaytest/cam/imag:\n    fields:\n      heigth:\n        kind: set_equal\n",
    );
    match &err {
        ReplayError::ToleranceInvalid { reason } => {
            assert!(
                reason.contains("/replaytest/cam/image"),
                "topic suggestion: {reason}"
            );
            assert!(reason.contains("height"), "field suggestion: {reason}");
            assert!(reason.contains("heigth"), "offending field named: {reason}");
        }
        other => panic!("expected ToleranceInvalid, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 4);
}

#[test]
fn no_tolerance_and_valid_bit_exact_tolerance_reach_same_node_load_exit_3() {
    // Transparency pin: a VALID tolerance (default bit_exact + a real
    // topic/field) is a no-op before the engine — the bag reaches the SAME
    // node-load exit 3 as no tolerance at all (the diff is unchanged).
    let (dir, path) = tmp_bag("tol_transparent.mcap");
    write_image_bag(&path);

    // No tolerance → NodeLoad exit 3.
    let baseline = run_replay(&path, ReplayOptions::default()).unwrap_err();
    assert!(
        matches!(baseline, ReplayError::NodeLoad { .. }),
        "baseline: {baseline:?}"
    );
    assert_eq!(baseline.exit_code(), 3);

    // Valid tolerance naming a REAL topic + field → same NodeLoad exit 3.
    let with_tol = gate_with_tolerance(
        &dir,
        &path,
        "default_metric:\n  kind: bit_exact\ntopics:\n  /replaytest/cam/image:\n    fields:\n      height:\n        kind: max_abs\n        threshold: 0.5\n",
    );
    assert!(
        matches!(with_tol, ReplayError::NodeLoad { .. }),
        "with valid tolerance: {with_tol:?}"
    );
    assert_eq!(with_tol.exit_code(), 3);
    assert_eq!(baseline.exit_code(), with_tol.exit_code());
}

// ── The OPAQUE-field metric loudness (exit 4) ───────────────

#[test]
fn tolerance_numeric_metric_on_opaque_field_is_exit_4() {
    // `sensor_msgs/Image.data` is a `uint8[]` (publisher-opaque bytes) — a
    // non-bit_exact metric on it is refused LOUDLY at validation (exit 4),
    // naming the limitation and the bit_exact alternative.
    let (dir, path) = tmp_bag("tol_opaque.mcap");
    write_image_bag(&path);
    let err = gate_with_tolerance(
        &dir,
        &path,
        "topics:\n  /replaytest/cam/image:\n    fields:\n      data:\n        kind: max_abs\n        threshold: 0.1\n",
    );
    match &err {
        ReplayError::ToleranceInvalid { reason } => assert!(
            reason.contains("data")
                && reason.contains("bit_exact")
                && (reason.contains("opaque") || reason.contains("decodable")),
            "opaque-field message: {reason}"
        ),
        other => panic!("expected ToleranceInvalid, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 4);
}

#[test]
fn tolerance_numeric_metric_on_nested_dotted_path_is_exit_4() {
    // `header.frame_id` (a nested per-element path into std_msgs/Header) has no
    // addressable wire span — a non-bit_exact metric is exit 4.
    let (dir, path) = tmp_bag("tol_nested.mcap");
    write_image_bag(&path);
    let err = gate_with_tolerance(
        &dir,
        &path,
        "topics:\n  /replaytest/cam/image:\n    fields:\n      header.frame_id:\n        kind: rmse\n        threshold: 0.1\n",
    );
    match &err {
        ReplayError::ToleranceInvalid { reason } => assert!(
            reason.contains("header.frame_id")
                && reason.contains("bit_exact")
                && reason.contains("NESTED"),
            "nested-path message: {reason}"
        ),
        other => panic!("expected ToleranceInvalid, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 4);
}

#[test]
fn tolerance_bit_exact_on_opaque_field_is_allowed() {
    // bit_exact stays LEGAL on an opaque field (byte compare needs no decode) —
    // it validates and reaches the same node-load exit 3 as the baseline.
    let (dir, path) = tmp_bag("tol_opaque_bitexact.mcap");
    write_image_bag(&path);
    let with_tol = gate_with_tolerance(
        &dir,
        &path,
        "topics:\n  /replaytest/cam/image:\n    fields:\n      data:\n        kind: bit_exact\n",
    );
    assert!(
        matches!(with_tol, ReplayError::NodeLoad { .. }),
        "bit_exact on an opaque field must validate (reach node-load): {with_tol:?}"
    );
    assert_eq!(with_tol.exit_code(), 3);
}

// ── Topic-wide `metric:` / `default_metric` COVERAGE exit-4 (real registry)

#[test]
fn tolerance_topic_wide_metric_on_a_topic_with_an_opaque_field_is_exit_4() {
    // A bare topic-wide non-bit_exact metric must be decodable on EVERY
    // field of the topic's schema. `sensor_msgs/Image` carries opaque fields
    // (`data: uint8[]`, the nested `header`, `encoding: string`), so a topic-wide
    // max_abs is refused LOUDLY at validation — scoped as a topic-wide metric,
    // naming the offending field and the bit_exact remedy.
    let (dir, path) = tmp_bag("tol_topicwide_opaque.mcap");
    write_image_bag(&path);
    let err = gate_with_tolerance(
        &dir,
        &path,
        "topics:\n  /replaytest/cam/image:\n    metric:\n      kind: max_abs\n      threshold: 0.1\n",
    );
    match &err {
        ReplayError::ToleranceInvalid { reason } => assert!(
            reason.contains("topic-wide metric")
                && reason.contains("bit_exact")
                && (reason.contains("opaque") || reason.contains("decodable")),
            "topic-wide opaque message: {reason}"
        ),
        other => panic!("expected ToleranceInvalid, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 4);
}

#[test]
fn tolerance_default_metric_on_a_topic_with_an_opaque_field_is_exit_4() {
    // A non-bit_exact `default_metric` covers every field of every graph
    // topic — the Image topic's opaque field trips it, scoped as default_metric.
    let (dir, path) = tmp_bag("tol_default_opaque.mcap");
    write_image_bag(&path);
    let err = gate_with_tolerance(
        &dir,
        &path,
        "default_metric:\n  kind: max_abs\n  threshold: 0.1\n",
    );
    match &err {
        ReplayError::ToleranceInvalid { reason } => assert!(
            reason.contains("default_metric")
                && reason.contains("bit_exact")
                && (reason.contains("opaque") || reason.contains("decodable")),
            "default_metric opaque message: {reason}"
        ),
        other => panic!("expected ToleranceInvalid, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 4);
}

/// A graph whose one produced output is `std_msgs/UInt64` (a lone top-level
/// `uint64 data` field) — for the 64-bit-int lossy-metric composition. Produced
/// topic: `/replaytest/ctr/count`.
fn u64_graph_yaml() -> Vec<u8> {
    b"name: replaytest\nprefix: replaytest\nnodes:\n  - id: ctr\n    type: counter\n    \
      outputs:\n      - name: count\n        schema: std_msgs/UInt64\n"
        .to_vec()
}

fn write_u64_bag(path: &Path) {
    let atts = vec![
        att("graph.yaml", "application/yaml", u64_graph_yaml()),
        att(
            "env.json",
            "application/json",
            br#"{"RUST_LOG":"info"}"#.to_vec(),
        ),
        att(
            &manifest_name(0),
            "application/json",
            manifest_bytes(0, &["ctr"]),
        ),
    ];
    write_bag(path, &[fire(0, 1), fire(0, 2)], &atts, true);
}

#[test]
fn tolerance_topic_wide_lossy_metric_on_a_u64_field_is_exit_4() {
    // Coverage + lossy-metric composition: a topic-wide max_rel over a topic whose only
    // field is a `uint64` is refused (precision-lossy on 64-bit integers), naming
    // the field and pointing at max_abs / bit_exact.
    let (dir, path) = tmp_bag("tol_topicwide_u64.mcap");
    write_u64_bag(&path);
    let err = gate_with_tolerance(
        &dir,
        &path,
        "topics:\n  /replaytest/ctr/count:\n    metric:\n      kind: max_rel\n      threshold: 0.1\n",
    );
    match &err {
        ReplayError::ToleranceInvalid { reason } => assert!(
            reason.contains("topic-wide metric")
                && reason.contains("'data'")
                && reason.contains("max_abs"),
            "topic-wide u64 lossy message: {reason}"
        ),
        other => panic!("expected ToleranceInvalid, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 4);
}

// ── Tolerance-gate hardening (directory path + unknown-kind) ────

#[test]
fn tolerance_path_is_a_directory_is_exit_4() {
    // `--tolerance` pointing at a DIRECTORY is the exit-4 class with a
    // message naming the PATH and the structural cause (a directory), not a bare
    // "Is a directory (os error 21)" io error.
    let (dir, path) = tmp_bag("tol_isdir.mcap");
    write_image_bag(&path);
    let tol_dir = dir.path().join("a_directory_not_a_file");
    std::fs::create_dir(&tol_dir).unwrap();
    let err = run_replay(
        &path,
        ReplayOptions {
            tolerance_path: Some(tol_dir.clone()),
            ..Default::default()
        },
    )
    .unwrap_err();
    match &err {
        ReplayError::ToleranceInvalid { reason } => assert!(
            reason.contains(&tol_dir.display().to_string()) && reason.contains("directory"),
            "reason must name the path AND that it is a directory: {reason}"
        ),
        other => panic!("expected ToleranceInvalid, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 4);
}

#[test]
fn tolerance_unknown_metric_kind_lists_all_eight_kinds_is_exit_4() {
    // An unknown metric `kind:` surfaces through the exit-4 gate naming
    // EVERY one of the eight valid kinds so the user can self-correct.
    let (dir, path) = tmp_bag("tol_badkind.mcap");
    write_image_bag(&path);
    let err = gate_with_tolerance(&dir, &path, "default_metric:\n  kind: nonsense\n");
    match &err {
        ReplayError::ToleranceInvalid { reason } => {
            for kind in [
                "bit_exact",
                "max_abs",
                "max_rel",
                "rmse",
                "bbox_iou",
                "set_equal",
                "set_subset",
                "ordered_list_equal",
            ] {
                assert!(reason.contains(kind), "kind '{kind}' must appear: {reason}");
            }
        }
        other => panic!("expected ToleranceInvalid, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 4);
}

// ===========================================================================
// The schema-drift PRE-FLIGHT (exit 2)
//
// A graph whose PRODUCED topic was recorded under a schema whose layout hash
// differs from the current workspace's is refused BEFORE the engine loads a
// cdylib — an actionable "not replay-grade for this workspace" refusal instead
// of an unexplained per-frame byte mismatch (exit 1). The recorded-side hash is
// read from the FIRST FRAME's WireHeader (the byte the candidate must
// reproduce), never the bag channel descriptor (which these synthetic bags
// stamp 0). Fires only for topics with recorded frames + a resolvable current
// hash + a channel recorded under the current hash recipe.
// ===========================================================================

/// A graph with two produced topics, each on a distinct built-in schema so the
/// registry resolves a current hash for BOTH — used by the multi-topic drift
/// refusal pin. Produced topics: `/replaytest/cam/image` (sensor_msgs/Image),
/// `/replaytest/imu/data` (sensor_msgs/Imu).
fn two_produced_graph_yaml() -> Vec<u8> {
    b"name: replaytest\nprefix: replaytest\nnodes:\n\
      \x20 - id: cam\n    type: camera\n    outputs:\n      - name: image\n        schema: sensor_msgs/Image\n\
      \x20 - id: imu\n    type: imu\n    outputs:\n      - name: data\n        schema: sensor_msgs/Imu\n"
        .to_vec()
}

/// Build a bag that records `frames` (one per `(topic, schema_hash)` pair,
/// each a hand-built WireHeader carrying the given schema hash) on graph-PRODUCED
/// topics, with the given graph.yaml + a rank-0 manifest naming `node_ids`. The
/// channel descriptors are stamped with the CURRENT hash recipe (BagWriter's
/// default) so the drift preflight's recipe guard passes; the recorded-side hash
/// under test is the FRAME header's, not the channel's.
fn write_produced_topic_bag(
    path: &Path,
    graph_yaml: &[u8],
    node_ids: &[&str],
    frames: &[(&str, &str, u32, u64)], // (topic, schema_name, wire_fixed_size, frame_schema_hash)
) {
    let topics: Vec<TopicSchema> = frames
        .iter()
        .map(|(topic, schema_name, wfs, _)| TopicSchema {
            topic: (*topic).into(),
            schema_name: (*schema_name).into(),
            schema_hash: 0, // descriptor hash is unused by the preflight (frame hash wins)
            wire_fixed_size: *wfs,
        })
        .collect();
    let mut w = BagWriter::create(path, BagWriterConfig::default(), &topics).unwrap();
    // Frame bodies must outlive the write_chunk closure (zero-copy contract).
    let bodies: Vec<Vec<u8>> = frames
        .iter()
        .map(|(_, _, _, hash)| {
            let mut frame = vec![0u8; WireHeader::SIZE + 8];
            WireHeader::new(*hash, 0, 1000).write_to_buf(&mut frame);
            frame
        })
        .collect();
    w.write_chunk(|c| {
        for ((topic, _, _, _), body) in frames.iter().zip(&bodies) {
            c.write_message(*topic, 0, 1000, 1000, &[&body[..]])?;
        }
        // A minimal valid trace: one FIRE for node 0 (bounds-checked against the
        // manifest by the gate loop; the drift preflight runs after).
        c.write_scheduler_trace(0, 1000, 1000, &fire(0, 1))?;
        Ok(())
    })
    .unwrap();
    w.write_attachment("graph.yaml", "application/yaml", 0, 0, graph_yaml)
        .unwrap();
    w.write_attachment(
        "env.json",
        "application/json",
        0,
        0,
        br#"{"RUST_LOG":"info"}"#,
    )
    .unwrap();
    w.write_attachment(
        &manifest_name(0),
        "application/json",
        0,
        0,
        &manifest_bytes(0, node_ids),
    )
    .unwrap();
    w.finalize().unwrap();
}

/// The current workspace's recipe-3 schema hash for `topic`, resolved through
/// the SAME field-registry machinery `run_replay`'s preflight uses. Built-in ROS
/// 2 types (Image/Imu/Vector3) are never shadowed by a workspace schema, so this
/// value is stable across the discovered workspace root run_replay picks.
fn current_hash(graph_yaml: &[u8], topic: &str) -> u64 {
    let config =
        cerulion_core::graph::parse_graph(std::str::from_utf8(graph_yaml).unwrap()).unwrap();
    cerulion_cli_engine::replay_field_registry::FieldRegistry::from_graph(
        &config,
        std::path::Path::new("."),
    )
    .expected_schema_hash(topic)
    .expect("built-in schema resolves a current hash")
}

#[test]
fn produced_topic_schema_drift_is_exit_2_with_both_hashes_and_remediation() {
    let (_dir, path) = tmp_bag("drift_single.mcap");
    let graph = image_graph_yaml();
    let topic = "/replaytest/cam/image";
    // Frame carries a STALE hash that differs from the current sensor_msgs/Image
    // layout hash → drift.
    let stale = 0xDEAD_BEEF_DEAD_BEEFu64;
    let current = current_hash(&graph, topic);
    assert_ne!(
        stale, current,
        "the crafted stale hash must actually differ"
    );
    write_produced_topic_bag(
        &path,
        &graph,
        &["cam"],
        &[(topic, "sensor_msgs/Image", 8, stale)],
    );

    let err = gate(&path);
    match &err {
        ReplayError::SchemaDrift { drifts } => {
            assert_eq!(drifts.len(), 1, "one drifted topic: {drifts:?}");
            assert_eq!(drifts[0].topic, topic);
            assert_eq!(drifts[0].recorded_hash, stale);
            assert_eq!(drifts[0].current_hash, current);
        }
        other => panic!("expected SchemaDrift, got {other:?}"),
    }
    assert_eq!(
        err.exit_code(),
        2,
        "schema drift is not-replay-grade (exit 2)"
    );
    // The rendered message names the topic, BOTH hashes, and the remediation pair.
    let msg = err.to_string();
    assert!(msg.contains(topic), "names the topic: {msg}");
    assert!(
        msg.contains(&format!("0x{stale:016x}")) && msg.contains(&format!("0x{current:016x}")),
        "names both hashes: {msg}"
    );
    assert!(
        msg.contains("Re-record") && msg.contains("check out the recording-era schemas"),
        "carries the remediation pair: {msg}"
    );
}

#[test]
fn multi_topic_schema_drift_lists_every_drifted_topic_in_one_refusal() {
    let (_dir, path) = tmp_bag("drift_multi.mcap");
    let graph = two_produced_graph_yaml();
    let img = "/replaytest/cam/image";
    let imu = "/replaytest/imu/data";
    write_produced_topic_bag(
        &path,
        &graph,
        &["cam", "imu"],
        &[
            (img, "sensor_msgs/Image", 8, 0x1111_1111_1111_1111),
            (imu, "sensor_msgs/Imu", 8, 0x2222_2222_2222_2222),
        ],
    );

    let err = gate(&path);
    match &err {
        ReplayError::SchemaDrift { drifts } => {
            assert_eq!(drifts.len(), 2, "both topics drifted: {drifts:?}");
            let topics: Vec<&str> = drifts.iter().map(|d| d.topic.as_str()).collect();
            assert!(topics.contains(&img) && topics.contains(&imu), "{topics:?}");
        }
        other => panic!("expected SchemaDrift naming both, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
    let msg = err.to_string();
    assert!(
        msg.contains(img) && msg.contains(imu),
        "one refusal names both: {msg}"
    );
}

#[test]
fn matching_produced_topic_hash_is_no_drift_and_reaches_engine() {
    // The no-drift CONTROL: a produced topic whose recorded frame hash EQUALS
    // the current workspace hash must NOT trip the preflight — the bag proceeds
    // to the engine and fails at node load (the `camera` cdylib is absent in
    // this crate's build tree). Proves the preflight is inert
    // on a matching bag (byte-identical behavior to a run without it).
    let (_dir, path) = tmp_bag("drift_none.mcap");
    let graph = image_graph_yaml();
    let topic = "/replaytest/cam/image";
    let current = current_hash(&graph, topic);
    write_produced_topic_bag(
        &path,
        &graph,
        &["cam"],
        &[(topic, "sensor_msgs/Image", 8, current)],
    );

    let err = gate(&path);
    match &err {
        ReplayError::NodeLoad { node_type, .. } => assert_eq!(node_type, "camera"),
        ReplayError::SchemaDrift { drifts } => {
            panic!("matching hash must NOT be flagged as drift: {drifts:?}")
        }
        other => panic!("expected NodeLoad (reached the engine), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 3);
}

// ── The legacy hash-recipe SKIP branch ─────────────────────────────
//
// `detect_schema_drift`'s recipe guard — `recipe_current.get(topic) != Some(true)`
// — SKIPS any produced topic whose channel descriptor was NOT stamped with the
// current `HASH_RECIPE`: a legacy (recipe-1) bag carries a frame hash that
// is not comparable to the recipe-3 registry hash, so it proceeds to the engine
// (an unexplained byte mismatch downstream) rather than a spurious schema-drift refusal.
// The pin below covers that `continue`: it crafts a bag with a GENUINE
// frame-hash drift but a LEGACY recipe descriptor and proves it is NOT refused
// as `SchemaDrift` — it clears the preflight and reaches the engine.

use cerulion_bag::{SchemaDescriptor, DESCRIPTOR_LEN};

/// Rewrite the produced topic's channel descriptor to a LEGACY (non-current)
/// hash recipe by patching the on-disk MCAP bytes. `write_produced_topic_bag`
/// stamps every user channel with `SchemaDescriptor::new(0, wire_fixed_size)`
/// (descriptor `schema_hash` is 0 — the frame header carries the hash under
/// test) at the current `HASH_RECIPE`; that fixed 18-byte blob appears
/// uncompressed (Cerulion chunks are never compressed) in BOTH the leading
/// Schema record and the summary Schema record. Patch EVERY occurrence — only
/// the `hash_recipe` u32 at descriptor offset `2..6`, leaving the record length
/// and `descriptor_version` intact — so the two copies stay consistent and the
/// blob still decodes (as recipe N, not recipe 3). Returns the count rewritten.
fn downgrade_channel_recipe(path: &Path, wire_fixed_size: u32, legacy_recipe: u32) -> usize {
    assert_ne!(
        legacy_recipe,
        cerulion_core::trace::bag::HASH_RECIPE,
        "the legacy recipe must actually differ from the current one"
    );
    let needle = SchemaDescriptor::new(0, wire_fixed_size).encode();
    let mut bytes = std::fs::read(path).unwrap();
    let mut patched = 0usize;
    let mut i = 0usize;
    while i + DESCRIPTOR_LEN <= bytes.len() {
        if bytes[i..i + DESCRIPTOR_LEN] == needle[..] {
            bytes[i + 2..i + 6].copy_from_slice(&legacy_recipe.to_le_bytes());
            patched += 1;
            i += DESCRIPTOR_LEN;
        } else {
            i += 1;
        }
    }
    std::fs::write(path, &bytes).unwrap();
    patched
}

#[test]
fn legacy_recipe_channel_skips_drift_preflight_and_reaches_engine() {
    let (_dir, path) = tmp_bag("drift_legacy_recipe.mcap");
    let graph = image_graph_yaml();
    let topic = "/replaytest/cam/image";
    // A GENUINE drift: the recorded frame hash differs from the current layout —
    // WITHOUT the recipe downgrade below this bag is `SchemaDrift` (exit 2), so
    // the legacy-recipe skip is the ONLY thing that lets it through.
    let stale = 0xDEAD_BEEF_DEAD_BEEFu64;
    let current = current_hash(&graph, topic);
    assert_ne!(
        stale, current,
        "the crafted stale hash must actually differ"
    );
    write_produced_topic_bag(
        &path,
        &graph,
        &["cam"],
        &[(topic, "sensor_msgs/Image", 8, stale)],
    );
    // Downgrade the channel to a legacy recipe (1): the recorded frame hash
    // is now recipe-incomparable to the recipe-3 registry, so the preflight must
    // SKIP it even though the frame hash genuinely drifted.
    let patched = downgrade_channel_recipe(&path, 8, 1);
    assert!(
        patched >= 1,
        "expected to rewrite the channel descriptor recipe at least once (chunks are \
         never compressed, so the 18-byte blob is byte-addressable)"
    );

    let err = gate(&path);
    // NOT refused as drift — the legacy-recipe skip fired. The bag reaches the
    // engine and dies at cdylib load (the `camera` cdylib is absent here),
    // exactly like the matching-hash no-drift control.
    match &err {
        ReplayError::NodeLoad { node_type, .. } => assert_eq!(node_type, "camera"),
        ReplayError::SchemaDrift { drifts } => {
            panic!("a legacy-recipe channel must NOT trip the drift preflight: {drifts:?}")
        }
        other => panic!("expected NodeLoad (reached the engine), got {other:?}"),
    }
    assert_eq!(err.exit_code(), 3);
}

// ── Tolerance (exit 4) PREEMPTS schema drift (exit 2) ──────────────
//
// The ordering documented at the `detect_schema_drift` call site: the
// `--tolerance` preflight (exit 4) runs BEFORE the schema-drift preflight (exit
// 2), so a run with BOTH problems reports the tolerance error first — "the
// user's own input is validated before the bag's replay-grade is judged." A
// GENUINE drift bag + a tolerance YAML that fails validation must surface
// `ToleranceInvalid` (exit 4), NEVER `SchemaDrift`.

#[test]
fn tolerance_error_wins_over_schema_drift_ordering_pin() {
    let (dir, path) = tmp_bag("tol_before_drift.mcap");
    let graph = image_graph_yaml();
    let topic = "/replaytest/cam/image";
    // A REAL drift (would be exit-2 `SchemaDrift` on its own — proven by
    // `produced_topic_schema_drift_is_exit_2_with_both_hashes_and_remediation`).
    let stale = 0xDEAD_BEEF_DEAD_BEEFu64;
    let current = current_hash(&graph, topic);
    assert_ne!(
        stale, current,
        "the crafted stale hash must actually differ (a REAL drift)"
    );
    write_produced_topic_bag(
        &path,
        &graph,
        &["cam"],
        &[(topic, "sensor_msgs/Image", 8, stale)],
    );

    // A typoed topic in the tolerance doc — invalid (mirrors
    // `tolerance_typoed_topic_is_exit_4_with_suggestion`). Because the tolerance
    // preflight runs first, this exit-4 refusal must PREEMPT the exit-2 drift.
    let err = gate_with_tolerance(&dir, &path, "topics:\n  /replaytest/cam/imag: {}\n");
    match &err {
        ReplayError::ToleranceInvalid { reason } => assert!(
            reason.contains("/replaytest/cam/imag") && reason.contains("did you mean"),
            "tolerance suggestion: {reason}"
        ),
        ReplayError::SchemaDrift { drifts } => panic!(
            "the tolerance error must PREEMPT the schema drift (exit 4 before exit 2); \
             got SchemaDrift {drifts:?}"
        ),
        other => panic!("expected ToleranceInvalid, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 4);
}
