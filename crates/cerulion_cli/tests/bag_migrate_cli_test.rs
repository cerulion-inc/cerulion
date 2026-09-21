// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion bag migrate` over the REAL binary.
//!
//! The engine's own contracts (which keys are stripped, what survives byte for
//! byte, the whole consent ladder) are pinned hermetically in
//! `cerulion_cli_engine/tests/bag_migrate_test.rs`. What can only be proven
//! from out here is that the verb is REACHABLE and WIRED: that clap parses it,
//! that the binary's TTY probe and consent plumbing reach the engine's ladder,
//! and that each outcome maps to the exit code and the operator output a script
//! would key on. Without this, the whole feature could ship inert behind a
//! dispatch arm nothing calls.
//!
//! `Command::output()` gives the child a NULL stdin, so it is genuinely not a
//! terminal — which is exactly the non-TTY rung, reached for real rather than
//! simulated.
//!
//! Unix-gated (`cerulion_bag` is `#![cfg(unix)]`); a unique tempdir per test,
//! self-terminating subprocesses, no transport ⇒ parallel-safe.

#![cfg(unix)]

use std::path::Path;
use std::process::Command;

use cerulion_bag::{BagWriter, BagWriterConfig, TopicSchema};

/// Line 6 of this document is `    policy:` and line 11 is `        dpeth: 4`
/// — both asserted below, so the listing is pinned against a hand count rather
/// than against itself.
const LEGACY_GRAPH: &str = concat!(
    "name: legacy\n",              // 1
    "prefix: replaytest\n",        // 2
    "nodes:\n",                    // 3
    "  - id: pub1\n",              // 4
    "    type: test_pub\n",        // 5
    "    policy:\n",               // 6
    "      period_ms: 10\n",       // 7
    "    outputs:\n",              // 8
    "      - name: data\n",        // 9
    "        schema: test/Data\n", // 10
    "        dpeth: 4\n",          // 11
);

const CLEAN_GRAPH: &str = concat!(
    "name: legacy\n",
    "prefix: replaytest\n",
    "nodes:\n",
    "  - id: pub1\n",
    "    type: test_pub\n",
    "    outputs:\n",
    "      - name: data\n",
    "        schema: test/Data\n",
);

fn write_bag(path: &Path, graph: &str) {
    let payload = [0xABu8; 8];
    let topics = [TopicSchema {
        topic: "/data".into(),
        schema_name: "std_msgs/UInt8".into(),
        schema_hash: 0x0102_0304_0506_0708,
        wire_fixed_size: 8,
    }];
    let mut w = BagWriter::create(path, BagWriterConfig::default(), &topics).unwrap();
    w.write_chunk(|c| c.write_message("/data", 0, 1000, 1000, &[&payload[..]]))
        .unwrap();
    w.write_attachment("graph.yaml", "application/yaml", 0, 0, graph.as_bytes())
        .unwrap();
    w.write_attachment(
        "__cerulion/trace_manifest_rank0.json",
        "application/json",
        0,
        0,
        br#"{"rank":0,"generation":0,"node_ids":["pub1"]}"#,
    )
    .unwrap();
    w.finalize().unwrap();
}

struct Run {
    code: Option<i32>,
    stdout: String,
    stderr: String,
}

fn migrate(args: &[&str]) -> Run {
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .arg("bag")
        .arg("migrate")
        .args(args)
        // This verb touches no transport at all, but a stray gateway spawn in
        // a test process is never wanted.
        .env("CERULION_NETWORK", "off")
        .output()
        .expect("failed to spawn the cerulion binary");
    Run {
        code: out.status.code(),
        stdout: String::from_utf8_lossy(&out.stdout).into_owned(),
        stderr: String::from_utf8_lossy(&out.stderr).into_owned(),
    }
}

#[test]
fn the_verb_exists_and_a_dry_run_lists_every_key_by_path_and_line_without_writing() {
    let dir = tempfile::tempdir().unwrap();
    let bag = dir.path().join("legacy.mcap");
    write_bag(&bag, LEGACY_GRAPH);
    let before = std::fs::read(&bag).unwrap();

    let r = migrate(&[bag.to_str().unwrap(), "--dry-run"]);
    assert_eq!(
        r.code,
        Some(0),
        "a dry run is a successful inspection; stderr: {}",
        r.stderr
    );
    // The listing, against the hand-counted lines in `LEGACY_GRAPH`.
    assert!(
        r.stdout.contains("nodes[0].policy") && r.stdout.contains("line 6"),
        "stdout must locate the policy block: {}",
        r.stdout
    );
    assert!(
        r.stdout.contains("nodes[0].outputs[0].dpeth") && r.stdout.contains("line 11"),
        "stdout must locate the deeper key too: {}",
        r.stdout
    );
    assert!(
        r.stdout.contains("dry run — nothing written"),
        "stdout must say nothing was written: {}",
        r.stdout
    );
    assert!(
        !dir.path().join("legacy.migrated.mcap").exists(),
        "a dry run must write no bag"
    );
    assert_eq!(
        std::fs::read(&bag).unwrap(),
        before,
        "the input bag must be byte-untouched"
    );
}

#[test]
fn without_a_tty_and_without_yes_the_binary_refuses_and_names_the_flag() {
    let dir = tempfile::tempdir().unwrap();
    let bag = dir.path().join("legacy.mcap");
    write_bag(&bag, LEGACY_GRAPH);

    let r = migrate(&[bag.to_str().unwrap()]);
    assert!(
        matches!(r.code, Some(c) if c != 0),
        "a refusal must exit non-zero, got {:?}; stderr: {}",
        r.code,
        r.stderr
    );
    assert!(
        r.stderr.contains("--yes") && r.stderr.contains("--dry-run"),
        "stderr must name both escape hatches: {}",
        r.stderr
    );
    assert!(
        !dir.path().join("legacy.migrated.mcap").exists(),
        "a refusal must write no bag"
    );
}

#[test]
fn yes_writes_the_migrated_bag_beside_the_original_and_leaves_it_alone() {
    let dir = tempfile::tempdir().unwrap();
    let bag = dir.path().join("legacy.mcap");
    write_bag(&bag, LEGACY_GRAPH);
    let before = std::fs::read(&bag).unwrap();

    let r = migrate(&[bag.to_str().unwrap(), "--yes"]);
    assert_eq!(r.code, Some(0), "stderr: {}", r.stderr);

    let migrated = dir.path().join("legacy.migrated.mcap");
    assert!(
        migrated.is_file(),
        "stdout: {} stderr: {}",
        r.stdout,
        r.stderr
    );
    assert!(
        r.stdout.contains("Migrated bag written to"),
        "stdout must name the file it wrote: {}",
        r.stdout
    );
    assert!(
        r.stdout.contains("is unchanged"),
        "stdout must reassure about the original: {}",
        r.stdout
    );
    assert_eq!(
        std::fs::read(&bag).unwrap(),
        before,
        "the input bag must be byte-untouched"
    );

    // The product is real, and it is what the whole verb is for.
    let reader = cerulion_bag::BagReader::open(&migrated).unwrap();
    let graph = reader.attachment("graph.yaml").unwrap().unwrap().data;
    cerulion_core::graph::parse_graph_raw(std::str::from_utf8(&graph).unwrap())
        .expect("the migrated bag's graph must parse");
    assert!(
        reader
            .attachment("__cerulion/migration.json")
            .unwrap()
            .is_some(),
        "the provenance attachment must be there"
    );
    // No scratch file survives a successful run.
    //
    // Swept by PREFIX, not by name: since the scratch binding was added the tail
    // is unpredictable (`.migrating.<out>.<pid>.<nonce>.<attempt>`), so a
    // fixed-name check would be VACUOUSLY true — it would assert the absence of
    // a name nothing ever creates. The sweep is also strictly stronger: it
    // catches a leftover under any scratch name, including one from a retry.
    let leftovers: Vec<String> = std::fs::read_dir(dir.path())
        .expect("reads the workspace dir")
        .filter_map(|e| e.ok())
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .filter(|n| n.starts_with(".migrating."))
        .collect();
    assert_eq!(
        leftovers,
        Vec::<String>::new(),
        "the scratch file must have been cleaned up"
    );

    // A SECOND run refuses rather than clobbering what the first wrote.
    let again = migrate(&[bag.to_str().unwrap(), "--yes"]);
    assert!(
        matches!(again.code, Some(c) if c != 0) && again.stderr.contains("already exists"),
        "code {:?} stderr {}",
        again.code,
        again.stderr
    );
}

#[test]
fn an_out_flag_places_the_bag_where_it_says() {
    let dir = tempfile::tempdir().unwrap();
    let bag = dir.path().join("legacy.mcap");
    write_bag(&bag, LEGACY_GRAPH);
    let out = dir.path().join("elsewhere.mcap");

    let r = migrate(&[bag.to_str().unwrap(), "-o", out.to_str().unwrap(), "--yes"]);
    assert_eq!(r.code, Some(0), "stderr: {}", r.stderr);
    assert!(out.is_file());
    assert!(
        !dir.path().join("legacy.migrated.mcap").exists(),
        "the default name must not also be written"
    );
}

#[test]
fn a_bag_that_needs_no_migration_is_refused_rather_than_copied() {
    // ANTI-TAUTOLOGY for every arm above: they would all pass a verb that
    // copied any bag handed to it.
    let dir = tempfile::tempdir().unwrap();
    let bag = dir.path().join("clean.mcap");
    write_bag(&bag, CLEAN_GRAPH);

    let r = migrate(&[bag.to_str().unwrap(), "--yes"]);
    assert!(
        matches!(r.code, Some(c) if c != 0),
        "code {:?} stdout {} stderr {}",
        r.code,
        r.stdout,
        r.stderr
    );
    assert!(
        r.stderr.contains("nothing to migrate"),
        "stderr must say why: {}",
        r.stderr
    );
    assert!(!dir.path().join("clean.migrated.mcap").exists());
}
