// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion bag migrate` end to end.
//!
//! Every bag here is CRAFTED in a tempdir through `cerulion_bag::BagWriter`
//! (the `replay_gates_test.rs` pattern) so the legacy shape under test
//! (an embedded `graph.yaml` carrying a key the format no longer defines) is
//! reproduced exactly rather than approximated. No iceoryx2, no transport,
//! unique tempdir per test: parallel-safe.
//!
//! Unix-gated: `cerulion_bag` and `bag_migrate` are both `#![cfg(unix)]`.

#![cfg(unix)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use cerulion_bag::{BagReader, BagWriter, BagWriterConfig, TopicSchema};
use cerulion_cli_engine::bag_migrate::{
    bag_migrate, MigrateOptions, MigrateOutcome, MigrationRecord, MigrationStamp,
    MIGRATION_ATTACHMENT,
};
use cerulion_cli_engine::error::CliResult;
use cerulion_core::trace_ring::{TraceRingRecord, RECORD_TYPE_FIRE};

/// A fixed migration date, so two runs of the same migration are comparable.
/// This is the whole reason `MigrationStamp` is a parameter — see its docs.
const FIXED_STAMP_NS: u64 = 1_753_000_000_000_000_000;

const USER_TOPIC_A: &str = "/data";
const USER_TOPIC_B: &str = "/telemetry";

/// One attachment: (name, media_type, log_time, create_time, bytes).
type Att = (String, String, u64, u64, Vec<u8>);

fn att(name: &str, media_type: &str, log_time: u64, create_time: u64, bytes: &[u8]) -> Att {
    (
        name.to_string(),
        media_type.to_string(),
        log_time,
        create_time,
        bytes.to_vec(),
    )
}

fn topics() -> Vec<TopicSchema> {
    vec![
        TopicSchema {
            topic: USER_TOPIC_A.into(),
            schema_name: "std_msgs/UInt8".into(),
            schema_hash: 0x0102_0304_0506_0708,
            wire_fixed_size: 8,
        },
        TopicSchema {
            topic: USER_TOPIC_B.into(),
            schema_name: "geometry_msgs/Vector3".into(),
            schema_hash: 0x0BAD_F00D_DEAD_BEEF,
            wire_fixed_size: 24,
        },
    ]
}

/// The legacy shape: an on-disk graph file that still carries a
/// since-removed `policy:` block, PLUS a second undefined key at a different
/// nesting depth so a migration that only ever finds one would fail.
///
/// Line numbers are hand-counted and asserted by the listing arms:
///   1  name: legacy
///   2  prefix: replaytest
///   3  nodes:
///   4    - id: pub1
///   5      type: test_pub
///   6      policy:                    <- nodes[0].policy
///   7        period_ms: 10
///   8      outputs:
///   9        - name: data
///   10         schema: test/Data
///   11         dpeth: 4               <- nodes[0].outputs[0].dpeth
fn legacy_graph_yaml() -> Vec<u8> {
    concat!(
        "name: legacy\n",
        "prefix: replaytest\n",
        "nodes:\n",
        "  - id: pub1\n",
        "    type: test_pub\n",
        "    policy:\n",
        "      period_ms: 10\n",
        "    outputs:\n",
        "      - name: data\n",
        "        schema: test/Data\n",
        "        dpeth: 4\n",
    )
    .as_bytes()
    .to_vec()
}

/// The same graph with nothing the format rejects — the CONTROL twin.
fn clean_graph_yaml() -> Vec<u8> {
    concat!(
        "name: legacy\n",
        "prefix: replaytest\n",
        "nodes:\n",
        "  - id: pub1\n",
        "    type: test_pub\n",
        "    outputs:\n",
        "      - name: data\n",
        "        schema: test/Data\n",
    )
    .as_bytes()
    .to_vec()
}

fn manifest_bytes(rank: u32, node_ids: &[&str]) -> Vec<u8> {
    let ids: Vec<String> = node_ids.iter().map(|s| s.to_string()).collect();
    serde_json::to_vec(&serde_json::json!({
        "rank": rank,
        "generation": 0,
        "node_ids": ids,
    }))
    .unwrap()
}

fn fire(node_idx: u32, step: u64) -> TraceRingRecord {
    TraceRingRecord {
        step,
        fire_time_ns: 1000 + step,
        duration_ns: 10,
        node_idx,
        global_level: 0,
        record_type: RECORD_TYPE_FIRE,
        reserved: 0,
    }
}

/// The standard attachment set — deliberately MORE than `graph.yaml`, with
/// distinct media types and non-zero, DIFFERING log/create times, so a rewrite
/// that dropped, reordered or re-stamped any of them is visible.
fn attachments(graph: Vec<u8>) -> Vec<Att> {
    vec![
        att("graph.yaml", "application/yaml", 0, 0, &graph),
        att(
            "env.json",
            "application/json",
            11,
            22,
            br#"{"RUST_LOG":"info"}"#,
        ),
        att(
            "__cerulion/recorder.json",
            "application/json",
            33,
            44,
            br#"{"arch":"aarch64","os":"macos"}"#,
        ),
        att(
            "__cerulion/trace_manifest_rank0.json",
            "application/json",
            55,
            66,
            &manifest_bytes(0, &["pub1"]),
        ),
    ]
}

/// Write a bag carrying two user topics, several frames on each, a handful of
/// scheduler-trace records, and `atts`.
fn write_bag(path: &Path, atts: &[Att], finalize: bool) {
    let payload_a = [0xABu8; 8];
    let payload_b = [0xCDu8; 24];
    let mut w = BagWriter::create(path, BagWriterConfig::default(), &topics()).unwrap();
    w.write_chunk(|c| {
        for i in 0..4u32 {
            c.write_message(
                USER_TOPIC_A,
                i,
                1000 + i as u64,
                2000 + i as u64,
                &[&payload_a[..]],
            )?;
            c.write_message(
                USER_TOPIC_B,
                i,
                1500 + i as u64,
                2500 + i as u64,
                &[&payload_b[..]],
            )?;
        }
        for step in 0..3u64 {
            let rec = fire(0, step);
            c.write_scheduler_trace(step as u32, rec.fire_time_ns, rec.fire_time_ns, &rec)?;
        }
        Ok(())
    })
    .unwrap();
    for (name, media_type, log_time, create_time, data) in atts {
        w.write_attachment(name, media_type, *log_time, *create_time, data)
            .unwrap();
    }
    if finalize {
        w.finalize().unwrap();
    } else {
        // No epilogue: the reader classifies this as not finalized, exactly
        // like a recorder that was killed.
        drop(w);
    }
}

fn tmp() -> tempfile::TempDir {
    tempfile::tempdir().unwrap()
}

/// A confirm closure that always answers `yes` and records that it was asked.
fn always_yes(seen: &mut Vec<String>) -> impl FnMut(&str) -> CliResult<bool> + '_ {
    move |preview: &str| {
        seen.push(preview.to_string());
        Ok(true)
    }
}

/// A confirm closure that must never be called.
fn never_asked(preview: &str) -> CliResult<bool> {
    panic!("the consent prompt must not run here; it was shown:\n{preview}")
}

fn migrate_with_yes(input: &Path, out: Option<PathBuf>) -> CliResult<MigrateOutcome> {
    let mut confirm = never_asked;
    bag_migrate(
        input,
        &MigrateOptions {
            out,
            dry_run: false,
            assume_yes: true,
        },
        MigrationStamp::at_epoch_ns(FIXED_STAMP_NS),
        false,
        &mut confirm,
    )
    .map(|r| r.outcome)
}

// ── a readback view of a whole bag, for the byte-identity walk ─────────────

#[derive(Debug, PartialEq, Eq)]
struct Frame {
    topic: String,
    sequence: u32,
    log_time: u64,
    publish_time: u64,
    data: Vec<u8>,
}

#[derive(Debug, PartialEq, Eq)]
struct Chan {
    schema_name: String,
    schema_encoding: String,
    message_encoding: String,
    descriptor_version: Option<u16>,
    hash_recipe: Option<u32>,
    schema_hash: Option<u64>,
    wire_fixed_size: Option<u32>,
}

fn frames_of(reader: &BagReader) -> Vec<Frame> {
    reader
        .messages()
        .unwrap()
        .map(|m| {
            let m = m.unwrap();
            Frame {
                topic: m.topic,
                sequence: m.sequence,
                log_time: m.log_time,
                publish_time: m.publish_time,
                data: m.data,
            }
        })
        .collect()
}

fn channels_of(reader: &BagReader) -> BTreeMap<String, Chan> {
    reader
        .channels()
        .unwrap()
        .into_iter()
        .map(|c| {
            (
                c.topic.clone(),
                Chan {
                    schema_name: c.schema_name,
                    schema_encoding: c.schema_encoding,
                    message_encoding: c.message_encoding,
                    descriptor_version: c.descriptor.map(|d| d.descriptor_version),
                    hash_recipe: c.descriptor.map(|d| d.hash_recipe),
                    schema_hash: c.descriptor.map(|d| d.schema_hash),
                    wire_fixed_size: c.descriptor.map(|d| d.wire_fixed_size),
                },
            )
        })
        .collect()
}

/// Every attachment IN FILE ORDER, duplicates and all — the same [`Att`] tuple
/// the fixtures are written in, so the comparison is against a shape a reader
/// of this file already knows.
///
/// A `BTreeMap` keyed by name discards the ORDER and
/// COLLAPSES duplicates, so a migration that reordered the attachments or
/// dropped one of two same-named ones would still satisfy a byte-identity oracle
/// (a lossy test oracle). MCAP attachment names are not unique
/// and `BagReader::attachment` answers with the FIRST match, so a duplicate is
/// exactly the thing whose position decides what every reader sees — the same
/// property the already-migrated refusal exists to protect.
fn attachments_of(reader: &BagReader) -> Vec<Att> {
    reader
        .attachments()
        .unwrap()
        .into_iter()
        .map(|a| (a.name, a.media_type, a.log_time, a.create_time, a.data))
        .collect()
}

// ===========================================================================
// The headline: a legacy bag migrates, and the copy resims
// ===========================================================================

#[test]
fn a_legacy_bag_is_refused_by_the_resim_gate_and_its_migrated_copy_is_not() {
    use cerulion_cli_engine::replay_cmd::{run_replay, ReplayError, ReplayOptions};

    let dir = tmp();
    let legacy = dir.path().join("legacy.mcap");
    let clean = dir.path().join("clean.mcap");
    write_bag(&legacy, &attachments(legacy_graph_yaml()), true);
    write_bag(&clean, &attachments(clean_graph_yaml()), true);

    // PREMISE: the legacy bag really is refused, and refused for THIS reason.
    // Without this the arm below could pass on a bag that never had the
    // problem.
    let before = run_replay(&legacy, ReplayOptions::default()).unwrap_err();
    assert!(
        matches!(&before, ReplayError::BagInvalidAttachment { name, .. } if name == "graph.yaml"),
        "the crafted bag must reproduce the legacy-attachment refusal, got {before:?}"
    );
    assert_eq!(before.exit_code(), 2);

    assert_eq!(
        migrate_with_yes(&legacy, None).unwrap(),
        MigrateOutcome::Written
    );
    let migrated = dir.path().join("legacy.migrated.mcap");
    assert!(migrated.is_file(), "the migrated bag must exist");

    // THE oracle: the migrated bag reaches the same verdict as a bag that
    // never had the bad key — compared against the CONTROL twin rather than
    // against a hard-coded code, so this cannot rot into "some error".
    let after = run_replay(&migrated, ReplayOptions::default()).unwrap_err();
    let control = run_replay(&clean, ReplayOptions::default()).unwrap_err();
    assert!(
        !matches!(&after, ReplayError::BagInvalidAttachment { .. }),
        "the migrated bag must clear the attachment gate, got {after:?}"
    );
    assert_eq!(
        after.exit_code(),
        control.exit_code(),
        "migrated verdict {after:?} must match the clean twin's {control:?}"
    );
    assert_eq!(
        std::mem::discriminant(&after),
        std::mem::discriminant(&control),
        "migrated verdict {after:?} must be the same KIND as the clean twin's {control:?}"
    );
}

/// A FLOW-STYLE legacy graph migrates end to end.
///
/// `outputs: [{...}]` is ordinary hand-written YAML, and the offending key sits
/// BEHIND its siblings on the line — so the block path's whole-line blank would
/// take the node with it. A verb without the flow arm refuses such a bag outright.
#[test]
fn a_flow_style_legacy_bag_migrates_and_keeps_its_siblings() {
    let dir = tmp();
    let input = dir.path().join("flow.mcap");
    // Line 5 is the flow mapping; hand-counted, asserted below.
    let graph = concat!(
        "name: legacy\n",                                             // 1
        "prefix: replaytest\n",                                       // 2
        "nodes:\n",                                                   // 3
        "  - id: pub1\n",                                             // 4
        "    outputs: [{name: data, dpeth: 4, schema: test/Data}]\n", // 5
        "    type: test_pub\n",                                       // 6
    );
    write_bag(&input, &attachments(graph.as_bytes().to_vec()), true);

    let mut confirm = never_asked;
    let report = bag_migrate(
        &input,
        &MigrateOptions {
            out: None,
            dry_run: false,
            assume_yes: true,
        },
        MigrationStamp::at_epoch_ns(FIXED_STAMP_NS),
        false,
        &mut confirm,
    )
    .expect("a flow-style bag migrates");
    assert_eq!(report.outcome, MigrateOutcome::Written);
    assert_eq!(
        report
            .stripped
            .iter()
            .map(|s| (s.key.as_str(), s.path.as_str(), s.line))
            .collect::<Vec<_>>(),
        vec![("dpeth", "nodes[0].outputs[0].dpeth", 5)]
    );

    // The siblings on that line survive into the migrated bag.
    let dst = BagReader::open(dir.path().join("flow.migrated.mcap")).unwrap();
    let bytes = dst.attachment("graph.yaml").unwrap().unwrap().data;
    let cfg = cerulion_core::graph::parse_graph_raw(std::str::from_utf8(&bytes).unwrap())
        .expect("the migrated graph must parse");
    assert_eq!(cfg.nodes.len(), 1);
    assert_eq!(cfg.nodes[0].id, "pub1");
    assert_eq!(cfg.nodes[0].outputs.len(), 1);
    assert_eq!(cfg.nodes[0].outputs[0].name, "data");
    assert_eq!(cfg.nodes[0].outputs[0].schema, "test/Data");
}

#[test]
fn the_listing_names_every_stripped_key_by_path_and_line() {
    let dir = tmp();
    let input = dir.path().join("legacy.mcap");
    write_bag(&input, &attachments(legacy_graph_yaml()), true);

    let mut confirm = never_asked;
    let report = bag_migrate(
        &input,
        &MigrateOptions {
            out: None,
            dry_run: true,
            assume_yes: false,
        },
        MigrationStamp::at_epoch_ns(FIXED_STAMP_NS),
        false,
        &mut confirm,
    )
    .expect("dry run");

    // Hand oracle against `legacy_graph_yaml`'s counted lines — both keys, at
    // two DIFFERENT nesting depths, each with its own path and line.
    assert_eq!(
        report
            .stripped
            .iter()
            .map(|s| (s.key.as_str(), s.path.as_str(), s.line))
            .collect::<Vec<_>>(),
        vec![
            ("policy", "nodes[0].policy", 6),
            ("dpeth", "nodes[0].outputs[0].dpeth", 11),
        ]
    );
    assert!(report.preview.contains("nodes[0].outputs[0].dpeth"));
    assert!(report.preview.contains("line 11"));
}

// ===========================================================================
// Byte identity of everything that is not the graph
// ===========================================================================

#[test]
fn every_frame_channel_and_other_attachment_survives_byte_for_byte() {
    let dir = tmp();
    let input = dir.path().join("legacy.mcap");
    write_bag(&input, &attachments(legacy_graph_yaml()), true);
    assert_eq!(
        migrate_with_yes(&input, None).unwrap(),
        MigrateOutcome::Written
    );
    let migrated = dir.path().join("legacy.migrated.mcap");

    let src = BagReader::open(&input).unwrap();
    let dst = BagReader::open(&migrated).unwrap();

    // FRAMES: the whole stream, in order, with its own timestamps and bytes.
    // Includes the reserved `__cerulion/scheduler_trace` records.
    let src_frames = frames_of(&src);
    let dst_frames = frames_of(&dst);
    assert!(
        src_frames.len() >= 11,
        "the fixture must actually carry frames, got {}",
        src_frames.len()
    );
    assert_eq!(src_frames, dst_frames, "every frame must survive unchanged");
    assert!(
        src_frames
            .iter()
            .any(|f| f.topic == cerulion_bag::SCHEDULER_TRACE_TOPIC),
        "the fixture must exercise a reserved channel too"
    );

    // CHANNELS: schema name, encodings and the full descriptor.
    assert_eq!(channels_of(&src), channels_of(&dst));

    // ATTACHMENTS: everything but the graph, byte for byte including both
    // timestamps and the media type.
    let src_atts = attachments_of(&src);
    let dst_atts = attachments_of(&dst);

    // ORDERED, duplicates preserved. The expected output is the SOURCE list
    // with `graph.yaml` replaced in place and the one record this verb adds
    // appended — which is exactly what `write_bag_contents` claims to do
    // ("Attachments, in the input's own order, with `graph.yaml` replaced").
    // Asserting the whole SEQUENCE is what makes a reorder, a drop, or a
    // collapsed duplicate fail; a per-name lookup cannot see any of the three.
    let graph_at = |v: &[Att]| -> usize {
        v.iter()
            .position(|a| a.0 == "graph.yaml")
            .expect("the fixture carries a graph attachment")
    };
    let src_graph_idx = graph_at(&src_atts);
    let dst_graph_idx = graph_at(&dst_atts);
    assert_eq!(
        src_graph_idx, dst_graph_idx,
        "the graph attachment must be replaced IN PLACE, not moved"
    );

    let new_graph = dst_atts[dst_graph_idx].4.clone();
    let mut expected = src_atts.clone();
    expected[src_graph_idx].4 = new_graph.clone();
    expected.push((
        MIGRATION_ATTACHMENT.to_string(),
        dst_atts
            .last()
            .expect("the migrated bag has attachments")
            .1
            .clone(),
        dst_atts.last().unwrap().2,
        dst_atts.last().unwrap().3,
        dst_atts.last().unwrap().4.clone(),
    ));
    assert_eq!(
        dst_atts, expected,
        "every attachment must survive in order, byte for byte, with the graph          replaced in place and exactly one record appended"
    );
    // The appended one really is the provenance record, and there is EXACTLY
    // one of it — without this, `expected` is built from `dst`'s own tail and
    // would accept whatever happened to be there.
    assert_eq!(dst_atts.last().unwrap().0, MIGRATION_ATTACHMENT);
    assert_eq!(
        dst_atts
            .iter()
            .filter(|a| a.0 == MIGRATION_ATTACHMENT)
            .count(),
        1
    );

    // The graph itself DID change, and into something that parses.
    let new_graph = &new_graph;
    assert_ne!(new_graph, &src_atts[src_graph_idx].4);
    let new_text = std::str::from_utf8(new_graph).unwrap();
    let cfg =
        cerulion_core::graph::parse_graph_raw(new_text).expect("the migrated graph must parse");
    // It is the RE-RENDERED effective config, not the original document with
    // the offending block blanked out: the migrated bag must carry the shape
    // `graph run --record` writes, and a blanked original still parses. The
    // blanking leaves the removed block's lines behind as blanks, so their
    // absence is what separates the two.
    assert!(
        !new_text.lines().any(|l| l.trim().is_empty()),
        "the migrated graph must be the serde-rendered document, not the \
         blanked original: {new_text}"
    );
    assert!(
        !new_text.contains("policy") && !new_text.contains("dpeth"),
        "no stripped key may survive: {new_text}"
    );
    assert_eq!(cfg.prefix, "replaytest");
    assert_eq!(cfg.nodes.len(), 1);
    assert_eq!(cfg.nodes[0].outputs[0].schema, "test/Data");
}

#[test]
fn the_migration_record_states_what_was_removed_and_which_bag_it_came_from() {
    let dir = tmp();
    let input = dir.path().join("legacy.mcap");
    write_bag(&input, &attachments(legacy_graph_yaml()), true);
    assert_eq!(
        migrate_with_yes(&input, None).unwrap(),
        MigrateOutcome::Written
    );
    let migrated = dir.path().join("legacy.migrated.mcap");

    let dst = BagReader::open(&migrated).unwrap();
    let bytes = dst
        .attachment(MIGRATION_ATTACHMENT)
        .unwrap()
        .expect("the migration record must be present")
        .data;
    let rec: MigrationRecord = serde_json::from_slice(&bytes).unwrap();

    assert_eq!(rec.migrated_at_ns, FIXED_STAMP_NS, "the injected stamp");
    assert_eq!(rec.source_bag_file_name, "legacy.mcap");
    assert_eq!(
        rec.source_bag_size_bytes,
        std::fs::metadata(&input).unwrap().len()
    );
    assert_eq!(
        rec.stripped
            .iter()
            .map(|s| (s.path.as_str(), s.line))
            .collect::<Vec<_>>(),
        vec![("nodes[0].policy", 6), ("nodes[0].outputs[0].dpeth", 11)]
    );
    // The provenance is CHECKABLE: the recorded digest is the digest of the
    // file it names, recomputed here independently.
    assert_eq!(rec.source_bag_sha256, sha256_of_file(&input));
    assert_eq!(rec.source_bag_sha256.len(), 64);
    // ...and the graph digests name the documents on either side of the edit.
    let src = BagReader::open(&input).unwrap();
    assert_eq!(
        rec.source_graph_yaml_sha256,
        sha256_of(&src.attachment("graph.yaml").unwrap().unwrap().data)
    );
    assert_eq!(
        rec.migrated_graph_yaml_sha256,
        sha256_of(&dst.attachment("graph.yaml").unwrap().unwrap().data)
    );
    assert_ne!(rec.source_graph_yaml_sha256, rec.migrated_graph_yaml_sha256);
}

fn sha256_of(bytes: &[u8]) -> String {
    use sha2::Digest as _;
    sha2::Sha256::digest(bytes)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

fn sha256_of_file(path: &Path) -> String {
    sha256_of(&std::fs::read(path).unwrap())
}

// ===========================================================================
// Determinism
// ===========================================================================

#[test]
fn two_migrations_of_one_bag_are_byte_identical() {
    let dir = tmp();
    let input = dir.path().join("legacy.mcap");
    write_bag(&input, &attachments(legacy_graph_yaml()), true);

    let a = dir.path().join("a.mcap");
    let b = dir.path().join("b.mcap");
    migrate_with_yes(&input, Some(a.clone())).unwrap();
    migrate_with_yes(&input, Some(b.clone())).unwrap();

    let (ba, bb) = (std::fs::read(&a).unwrap(), std::fs::read(&b).unwrap());
    assert!(!ba.is_empty());
    assert_eq!(
        ba, bb,
        "two migrations of one bag, at one stamp, must be byte-identical"
    );
}

#[test]
fn the_date_is_the_only_thing_the_caller_supplies() {
    // ANTI-TAUTOLOGY for the arm above: it would also pass if the writer
    // ignored the stamp entirely. Changing ONLY the stamp must change the
    // bytes — that is what proves the seam is live rather than inert.
    let dir = tmp();
    let input = dir.path().join("legacy.mcap");
    write_bag(&input, &attachments(legacy_graph_yaml()), true);

    let mut confirm = never_asked;
    let mut run = |out: PathBuf, ns: u64| {
        bag_migrate(
            &input,
            &MigrateOptions {
                out: Some(out.clone()),
                dry_run: false,
                assume_yes: true,
            },
            MigrationStamp::at_epoch_ns(ns),
            false,
            &mut confirm,
        )
        .unwrap();
        std::fs::read(&out).unwrap()
    };
    let early = run(dir.path().join("early.mcap"), FIXED_STAMP_NS);
    let later = run(dir.path().join("later.mcap"), FIXED_STAMP_NS + 1);
    assert_ne!(early, later, "the recorded date must reach the file");
}

// ===========================================================================
// The consent ladder
// ===========================================================================

#[test]
fn dry_run_writes_nothing_and_wins_over_yes() {
    let dir = tmp();
    let input = dir.path().join("legacy.mcap");
    write_bag(&input, &attachments(legacy_graph_yaml()), true);
    let before = std::fs::read(&input).unwrap();

    let mut confirm = never_asked;
    let report = bag_migrate(
        &input,
        &MigrateOptions {
            out: None,
            // BOTH set: --dry-run must win.
            dry_run: true,
            assume_yes: true,
        },
        MigrationStamp::at_epoch_ns(FIXED_STAMP_NS),
        true,
        &mut confirm,
    )
    .expect("dry run");

    assert_eq!(report.outcome, MigrateOutcome::DryRun);
    assert!(!report.preview_shown, "no prompt ran, so the caller prints");
    assert!(
        !dir.path().join("legacy.migrated.mcap").exists(),
        "a dry run must write nothing"
    );
    assert_eq!(
        std::fs::read(&input).unwrap(),
        before,
        "the input bag must be untouched"
    );
}

#[test]
fn an_interactive_yes_writes_and_an_interactive_no_does_not() {
    let dir = tmp();
    let input = dir.path().join("legacy.mcap");
    write_bag(&input, &attachments(legacy_graph_yaml()), true);

    // NO: declined, nothing written, and the prompt really was shown.
    let mut shown = Vec::new();
    let mut decline = |preview: &str| -> CliResult<bool> {
        shown.push(preview.to_string());
        Ok(false)
    };
    let declined = bag_migrate(
        &input,
        &MigrateOptions {
            out: Some(dir.path().join("no.mcap")),
            dry_run: false,
            assume_yes: false,
        },
        MigrationStamp::at_epoch_ns(FIXED_STAMP_NS),
        true,
        &mut decline,
    )
    .expect("declined is not an error");
    assert_eq!(declined.outcome, MigrateOutcome::Declined);
    assert!(declined.preview_shown);
    assert_eq!(shown.len(), 1, "the prompt must be asked exactly once");
    assert!(shown[0].contains("nodes[0].policy"));
    assert!(!dir.path().join("no.mcap").exists(), "N must write nothing");

    // YES: written.
    let mut seen = Vec::new();
    let mut accept = always_yes(&mut seen);
    let written = bag_migrate(
        &input,
        &MigrateOptions {
            out: Some(dir.path().join("yes.mcap")),
            dry_run: false,
            assume_yes: false,
        },
        MigrationStamp::at_epoch_ns(FIXED_STAMP_NS),
        true,
        &mut accept,
    )
    .expect("accepted");
    assert_eq!(written.outcome, MigrateOutcome::Written);
    assert!(dir.path().join("yes.mcap").is_file());
}

#[test]
fn a_non_tty_run_without_yes_refuses_and_names_both_escape_hatches() {
    let dir = tmp();
    let input = dir.path().join("legacy.mcap");
    write_bag(&input, &attachments(legacy_graph_yaml()), true);

    let mut confirm = never_asked;
    let err = bag_migrate(
        &input,
        &MigrateOptions {
            out: None,
            dry_run: false,
            assume_yes: false,
        },
        MigrationStamp::at_epoch_ns(FIXED_STAMP_NS),
        false,
        &mut confirm,
    )
    .unwrap_err()
    .to_string();

    assert!(err.contains("--yes"), "must name the script escape: {err}");
    assert!(
        err.contains("--dry-run"),
        "must name the inspection escape: {err}"
    );
    assert!(
        err.contains("nothing was written"),
        "must say the file is untouched: {err}"
    );
    assert!(!dir.path().join("legacy.migrated.mcap").exists());
}

// ===========================================================================
// Refusals
// ===========================================================================

#[test]
fn a_bag_whose_graph_already_parses_is_refused_with_nothing_to_do() {
    let dir = tmp();
    let input = dir.path().join("clean.mcap");
    write_bag(&input, &attachments(clean_graph_yaml()), true);

    let mut confirm = never_asked;
    let err = bag_migrate(
        &input,
        &MigrateOptions {
            out: None,
            dry_run: false,
            assume_yes: true,
        },
        MigrationStamp::at_epoch_ns(FIXED_STAMP_NS),
        false,
        &mut confirm,
    )
    .unwrap_err()
    .to_string();

    assert!(
        err.contains("already parses") && err.contains("nothing to migrate"),
        "must say why there is nothing to do: {err}"
    );
    assert!(
        !dir.path().join("clean.migrated.mcap").exists(),
        "a refusal must mint no copy"
    );
}

#[test]
fn a_bag_with_no_embedded_graph_is_refused_by_name() {
    let dir = tmp();
    let input = dir.path().join("nograph.mcap");
    let atts: Vec<Att> = attachments(legacy_graph_yaml())
        .into_iter()
        .filter(|(name, ..)| name != "graph.yaml")
        .collect();
    write_bag(&input, &atts, true);

    let mut confirm = never_asked;
    let err = bag_migrate(
        &input,
        &MigrateOptions {
            out: None,
            dry_run: false,
            assume_yes: true,
        },
        MigrationStamp::at_epoch_ns(FIXED_STAMP_NS),
        false,
        &mut confirm,
    )
    .unwrap_err()
    .to_string();

    assert!(
        err.contains("carries no `graph.yaml` attachment"),
        "must name what is missing: {err}"
    );
    assert!(
        err.contains("bag record"),
        "must say which bags legitimately have none: {err}"
    );
    assert!(!dir.path().join("nograph.migrated.mcap").exists());
}

#[test]
fn a_torn_bag_is_refused_and_pointed_at_the_path_that_does_exist() {
    let dir = tmp();
    let input = dir.path().join("torn.mcap");
    write_bag(&input, &attachments(legacy_graph_yaml()), false);

    let mut confirm = never_asked;
    let err = bag_migrate(
        &input,
        &MigrateOptions {
            out: None,
            dry_run: false,
            assume_yes: true,
        },
        MigrationStamp::at_epoch_ns(FIXED_STAMP_NS),
        false,
        &mut confirm,
    )
    .unwrap_err()
    .to_string();

    assert!(
        err.contains("not finalized"),
        "must name the condition: {err}"
    );
    assert!(
        err.contains("cerulion bag info"),
        "must point at the diagnostic that DOES read a torn bag: {err}"
    );
    assert!(
        err.contains("re-record"),
        "must say what actually recovers: {err}"
    );
    assert!(!dir.path().join("torn.migrated.mcap").exists());
}

#[test]
fn real_yaml_damage_is_refused_rather_than_rewritten() {
    let dir = tmp();
    let input = dir.path().join("damaged.mcap");
    write_bag(&input, &attachments(b"nodes: [\n".to_vec()), true);

    let mut confirm = never_asked;
    let err = bag_migrate(
        &input,
        &MigrateOptions {
            out: None,
            dry_run: false,
            assume_yes: true,
        },
        MigrationStamp::at_epoch_ns(FIXED_STAMP_NS),
        false,
        &mut confirm,
    )
    .unwrap_err()
    .to_string();

    assert!(
        err.contains("NOT a key the format no longer defines"),
        "must distinguish damage from a stale key: {err}"
    );
    assert!(!dir.path().join("damaged.migrated.mcap").exists());
}

#[test]
fn a_missing_bag_is_refused_by_name() {
    let dir = tmp();
    let mut confirm = never_asked;
    let err = bag_migrate(
        &dir.path().join("nope.mcap"),
        &MigrateOptions::default(),
        MigrationStamp::at_epoch_ns(FIXED_STAMP_NS),
        false,
        &mut confirm,
    )
    .unwrap_err()
    .to_string();
    assert!(err.contains("does not exist"), "got: {err}");
}

#[test]
fn an_output_that_already_exists_is_refused_and_left_alone() {
    let dir = tmp();
    let input = dir.path().join("legacy.mcap");
    write_bag(&input, &attachments(legacy_graph_yaml()), true);
    let out = dir.path().join("taken.mcap");
    std::fs::write(&out, b"precious").unwrap();

    let mut confirm = never_asked;
    let err = bag_migrate(
        &input,
        &MigrateOptions {
            out: Some(out.clone()),
            dry_run: false,
            assume_yes: true,
        },
        MigrationStamp::at_epoch_ns(FIXED_STAMP_NS),
        false,
        &mut confirm,
    )
    .unwrap_err()
    .to_string();

    assert!(err.contains("already exists"), "got: {err}");
    assert_eq!(std::fs::read(&out).unwrap(), b"precious");
}

#[test]
fn the_input_bag_can_never_be_the_output() {
    let dir = tmp();
    let input = dir.path().join("legacy.mcap");
    write_bag(&input, &attachments(legacy_graph_yaml()), true);
    let before = std::fs::read(&input).unwrap();

    let mut confirm = never_asked;
    let err = bag_migrate(
        &input,
        &MigrateOptions {
            // Same file, spelled differently.
            out: Some(dir.path().join(".").join("legacy.mcap")),
            dry_run: false,
            assume_yes: true,
        },
        MigrationStamp::at_epoch_ns(FIXED_STAMP_NS),
        false,
        &mut confirm,
    )
    .unwrap_err()
    .to_string();

    assert!(
        err.contains("NEVER rewrites a bag in place"),
        "must state the rule: {err}"
    );
    assert_eq!(
        std::fs::read(&input).unwrap(),
        before,
        "the recording must be byte-untouched"
    );
}
