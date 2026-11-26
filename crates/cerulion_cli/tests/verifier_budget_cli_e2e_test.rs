// SPDX-License-Identifier: AGPL-3.0-only
//! Real-binary end-to-end tests for `--verifier-record-budget` and `--duration`
//! read-log census scoping on `cerulion bag play --resim all`.
//!
//! Spawns `CARGO_BIN_EXE_cerulion` against a free-run bag whose rank-0 read log
//! holds 200016 reads, more than 200000, and verifies:
//!
//! 1. Under the default whole-recording budget (3000000), `bag play --resim
//!    all --verify` verifies the recording byte-for-byte with exit 0.
//! 2. With `--verifier-record-budget 200000`, `bag play --resim all --verify`
//!    refuses before step 1 with exit 2, naming `read_identity_unplanned`,
//!    `verifier_declined_read_log_budget`, both counts (200016 vs 200000), and
//!    the `--verifier-record-budget <COUNT>` and `--duration` remedies, without
//!    creating a `--record-out` file.
//! 3. With `--verifier-record-budget 200000 --duration 1.0`, the read-log
//!    census covers only steps inside the 1.0 s window (16000 reads), so the
//!    verify passes over those 1000 ticks instead of being refused.
//! 4. Usage errors (`--verifier-record-budget 0` and `--verifier-record-budget`
//!    without `--resim`) exit 2 naming the flag, and `bag play --help` lists
//!    `--verifier-record-budget <COUNT>`.
//!
//! Prerequisites:
//! `cargo build -p test_node_macro_period_cdylib -p test_node_macro_data_trigger_cdylib`.
//!
//! Runs on both macOS and Linux: this test process never calls
//! `DylibNodeEntry::load` and never spawns `cerulion graph run`; each
//! `cerulion bag play --resim` subprocess loads its cdylib once and exits.

#![cfg(unix)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};
use std::time::Duration;

use cerulion_bag::{BagWriter, BagWriterConfig, TopicSchema};
use cerulion_cli_engine::graph_cmd::render_recorder_json;
use cerulion_cli_engine::replay_engine::CoordinationMode;
use cerulion_core::read_outcome::{ReadSiteRole, ReadStageRole};
use cerulion_core::trace_ring::{
    pack_read_outcome_aux_run, pack_read_outcome_meta, TraceRingRecord, READ_OUTCOME_NONE,
    RECORD_TYPE_FIRE, RECORD_TYPE_READ_OUTCOME, RECORD_TYPE_STEP_BOUNDARY,
};
use cerulion_core::wire::WireHeader;
use cerulion_core::ShmMessage;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

const RELAY_COUNT: usize = 16;
const STEPS: u64 = 12_501;
const EXPECTED_READS: u64 = (RELAY_COUNT as u64) * STEPS; // 200_016 > 200_000
const STEP_NS: u64 = 1_000_000; // 1 ms per step
const FIRST_STEP_NS: u64 = 50_000_000; // 50 ms = first ticker period boundary
const TICKER_PERIOD_STEPS: u64 = 50; // period_ms = 50

fn dylib_file(name: &str) -> String {
    if cfg!(target_os = "macos") {
        format!("lib{name}.dylib")
    } else {
        format!("lib{name}.so")
    }
}

fn build_workspace(root: &Path, prefix: &str) -> String {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("nodes/ticker/src")).unwrap();
    std::fs::create_dir_all(root.join("nodes/relay/src")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();

    let mut yaml = format!(
        "name: budget_e2e\n\
         prefix: {prefix}\n\
         nodes:\n\
         - id: ticker\n\
         \x20 type: ticker\n\
         \x20 inputs: []\n\
         \x20 outputs:\n\
         \x20 - name: cmd\n\
         \x20   schema: geometry_msgs/Vector3\n"
    );
    for i in 0..RELAY_COUNT {
        yaml.push_str(&format!(
            "- id: relay_{i}\n\
             \x20 type: relay\n\
             \x20 inputs:\n\
             \x20 - name: trigger_in\n\
             \x20   source: /ext/{prefix}/in_{i}\n\
             \x20 outputs:\n\
             \x20 - name: cmd\n\
             \x20   schema: geometry_msgs/Vector3\n"
        ));
    }
    std::fs::write(root.join("graphs/budget_e2e.yaml"), &yaml).unwrap();

    let crates_dir = Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap();
    std::fs::copy(
        crates_dir.join("test_fixtures/test_node_macro_period_cdylib/src/lib.rs"),
        root.join("nodes/ticker/src/lib.rs"),
    )
    .expect("copy ticker source");
    std::fs::copy(
        crates_dir.join("test_fixtures/test_node_macro_data_trigger_cdylib/src/lib.rs"),
        root.join("nodes/relay/src/lib.rs"),
    )
    .expect("copy relay source");

    let ticker_cdylib =
        cerulion_core::testing::find_fixture_cdylib("test_node_macro_period_cdylib");
    let relay_cdylib =
        cerulion_core::testing::find_fixture_cdylib("test_node_macro_data_trigger_cdylib");
    std::fs::copy(
        ticker_cdylib,
        root.join("target/debug").join(dylib_file("ticker")),
    )
    .expect("copy ticker cdylib");
    std::fs::copy(
        relay_cdylib,
        root.join("target/debug").join(dylib_file("relay")),
    )
    .expect("copy relay cdylib");

    yaml
}

fn write_long_free_run_bag(path: &Path, prefix: &str, graph_yaml: &str) {
    let ticker_topic = format!("/{prefix}/ticker/cmd");
    let mut topics = vec![TopicSchema {
        topic: ticker_topic.clone(),
        schema_name: "geometry_msgs/Vector3".into(),
        schema_hash: Vector3::SCHEMA_HASH,
        wire_fixed_size: 24,
    }];
    for i in 0..RELAY_COUNT {
        topics.push(TopicSchema {
            topic: format!("/ext/{prefix}/in_{i}"),
            schema_name: "geometry_msgs/Vector3".into(),
            schema_hash: Vector3::SCHEMA_HASH,
            wire_fixed_size: 24,
        });
        topics.push(TopicSchema {
            topic: format!("/{prefix}/relay_{i}/cmd"),
            schema_name: "geometry_msgs/Vector3".into(),
            schema_hash: Vector3::SCHEMA_HASH,
            wire_fixed_size: 24,
        });
    }

    let mut node_ids: Vec<String> = Vec::with_capacity(1 + RELAY_COUNT);
    node_ids.push("ticker".to_string());
    let mut inputs_map = serde_json::Map::new();
    let mut caps_map = serde_json::Map::new();
    let mut node_external: BTreeMap<String, bool> = BTreeMap::new();
    node_external.insert("ticker".to_string(), false);
    for i in 0..RELAY_COUNT {
        let id = format!("relay_{i}");
        node_ids.push(id.clone());
        inputs_map.insert(id.clone(), serde_json::json!(["trigger_in"]));
        caps_map.insert(
            id.clone(),
            serde_json::json!([[0u16, ReadStageRole::Body.wire(), 64u32]]),
        );
        node_external.insert(id, false);
    }

    let read_meta = pack_read_outcome_meta(0, READ_OUTCOME_NONE, ReadSiteRole::Drain);
    let read_aux = pack_read_outcome_aux_run(0, 1);

    let mut w = BagWriter::create(path, BagWriterConfig::default(), &topics).unwrap();
    let mut frame_seq: u32 = 0;
    let mut trace_seq: u32 = 0;
    // Write in chunks of 500 steps so the bag stays compact and streams cleanly.
    let chunk_steps: u64 = 500;
    let mut step_start: u64 = 0;
    while step_start < STEPS {
        let step_end = (step_start + chunk_steps).min(STEPS);
        w.write_chunk(|c| {
            for step in step_start..step_end {
                let t = FIRST_STEP_NS + step * STEP_NS;
                let boundary = TraceRingRecord {
                    step,
                    fire_time_ns: t,
                    duration_ns: 0,
                    node_idx: 0,
                    global_level: 0,
                    record_type: RECORD_TYPE_STEP_BOUNDARY,
                    reserved: 0,
                };
                c.write_scheduler_trace(trace_seq, t, t, &boundary)?;
                trace_seq += 1;

                if step % TICKER_PERIOD_STEPS == 0 {
                    let fire = TraceRingRecord {
                        step,
                        fire_time_ns: t,
                        duration_ns: 0,
                        node_idx: 0,
                        global_level: 0,
                        record_type: RECORD_TYPE_FIRE,
                        reserved: 0,
                    };
                    c.write_scheduler_trace(trace_seq, t, t, &fire)?;
                    trace_seq += 1;

                    let mut frame = [0u8; WireHeader::SIZE + 24];
                    let mut hdr = WireHeader::new(Vector3::SCHEMA_HASH, frame_seq, t);
                    hdr.total_size = (WireHeader::SIZE + 24) as u32;
                    hdr.offset_table_offset = (WireHeader::SIZE + 24) as u32;
                    hdr.write_to_buf(&mut frame[..WireHeader::SIZE]);
                    c.write_message(&ticker_topic, frame_seq, t, t, &[&frame[..]])?;
                    frame_seq += 1;
                }

                for i in 0..RELAY_COUNT {
                    let rec = TraceRingRecord {
                        step,
                        fire_time_ns: u64::MAX,
                        duration_ns: read_aux,
                        node_idx: (i + 1) as u32,
                        global_level: read_meta,
                        record_type: RECORD_TYPE_READ_OUTCOME,
                        reserved: 0,
                    };
                    c.write_scheduler_trace(trace_seq, t, t, &rec)?;
                    trace_seq += 1;
                }
            }
            Ok(())
        })
        .unwrap();
        step_start = step_end;
    }

    w.write_attachment(
        "graph.yaml",
        "application/yaml",
        0,
        0,
        graph_yaml.as_bytes(),
    )
    .unwrap();
    let manifest = serde_json::to_vec(&serde_json::json!({
        "rank": 0,
        "generation": 0,
        "node_ids": node_ids,
        "inputs": inputs_map,
        "read_log_capacity": 0,
        "read_log_capacities": caps_map,
    }))
    .unwrap();
    w.write_attachment(
        "__cerulion/trace_manifest_rank0.json",
        "application/json",
        0,
        0,
        &manifest,
    )
    .unwrap();
    let recorder_json = render_recorder_json(
        Duration::from_nanos(FIRST_STEP_NS),
        CoordinationMode::FreeRun,
        &node_external,
        &BTreeMap::new(),
        0,
    );
    w.write_attachment(
        "__cerulion/recorder.json",
        "application/json",
        0,
        0,
        &recorder_json,
    )
    .unwrap();
    w.finalize().unwrap();
}

fn run_cerulion(cwd: &Path, args: &[&str]) -> Output {
    let home = cwd.join(".test_home");
    std::fs::create_dir_all(&home).unwrap();
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(args)
        .current_dir(cwd)
        .env("HOME", &home)
        .env("CERULION_NETWORK", "off")
        .env("CERULION_TELEMETRY", "0")
        .env_remove("CARGO_TARGET_DIR")
        .output()
        .expect("spawn cerulion binary")
}

#[test]
#[serial]
fn long_recording_past_old_cap_verifies_and_over_configured_budget_refuses_naming_flag() {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path();
    let prefix = format!("vb{}e2e", std::process::id());
    let graph_yaml = build_workspace(root, &prefix);
    let bag = root.join("long_free_run.mcap");
    write_long_free_run_bag(&bag, &prefix, &graph_yaml);
    let bag_str = bag.to_str().unwrap();

    // 1. The default budget (3000000) admits and verifies a recording of
    //    200016 reads.
    let pass = run_cerulion(
        root,
        &["bag", "play", bag_str, "--resim", "all", "--verify"],
    );
    let pass_out = String::from_utf8_lossy(&pass.stdout);
    let pass_err = String::from_utf8_lossy(&pass.stderr);
    assert_eq!(
        pass.status.code(),
        Some(0),
        "recording with {EXPECTED_READS} reads must verify under default budget;\nstdout:\n{pass_out}\nstderr:\n{pass_err}"
    );
    assert!(
        pass_out.contains("replay PASS:")
            && pass_out.contains(&format!("{STEPS} tick(s) replayed")),
        "stdout must report replay PASS over all {STEPS} ticks:\n{pass_out}"
    );

    // 2. Lowering `--verifier-record-budget` to 200000 refuses
    //    before step 1 with exit 2, naming both counts and the remedies, and
    //    leaves no `--record-out` file behind.
    let refused_out: PathBuf = root.join("refused_out.mcap");
    let refused = run_cerulion(
        root,
        &[
            "bag",
            "play",
            bag_str,
            "--resim",
            "all",
            "--verify",
            "--verifier-record-budget",
            "200000",
            "--record-out",
            refused_out.to_str().unwrap(),
        ],
    );
    let refused_err = String::from_utf8_lossy(&refused.stderr);
    assert_eq!(
        refused.status.code(),
        Some(2),
        "recording over configured budget must exit 2;\nstderr:\n{refused_err}"
    );
    for needle in [
        "read_identity_unplanned",
        "verifier_declined_read_log_budget",
        "it holds 200016 reads, more than the verifier's budget of 200000 reads per rank",
        "--verifier-record-budget <COUNT>",
        "--duration",
    ] {
        assert!(
            refused_err.contains(needle),
            "refusal stderr must contain {needle:?}:\n{refused_err}"
        );
    }
    assert!(
        !refused_out.exists(),
        "a pre-step-1 budget refusal must not create the --record-out bag"
    );

    // 3. Bounding with `--duration 1.0` scopes the read-log census to the
    //    first 1000 steps (16000 reads, under 200000), so the same verify under
    //    the same 200000 budget passes over exactly those 1000 ticks.
    let bounded = run_cerulion(
        root,
        &[
            "bag",
            "play",
            bag_str,
            "--resim",
            "all",
            "--verify",
            "--verifier-record-budget",
            "200000",
            "--duration",
            "1.0",
        ],
    );
    let bounded_out = String::from_utf8_lossy(&bounded.stdout);
    let bounded_err = String::from_utf8_lossy(&bounded.stderr);
    assert_eq!(
        bounded.status.code(),
        Some(0),
        "--duration 1.0 must fit 16000 reads inside budget 200000;\nstdout:\n{bounded_out}\nstderr:\n{bounded_err}"
    );
    assert!(
        bounded_out.contains("replay PASS:") && bounded_out.contains("1000 tick(s) replayed"),
        "stdout must report replay PASS over the 1000 ticks inside --duration:\n{bounded_out}"
    );

    // 4. Usage errors on the real binary: zero count and missing `--resim`
    //    both exit 2 naming `--verifier-record-budget`.
    for (argv, extra_needle) in [
        (
            vec![
                "bag",
                "play",
                bag_str,
                "--resim",
                "all",
                "--verify",
                "--verifier-record-budget",
                "0",
            ],
            "positive record count",
        ),
        (
            vec!["bag", "play", bag_str, "--verifier-record-budget", "200000"],
            "--resim all",
        ),
    ] {
        let out = run_cerulion(root, &argv);
        let err = String::from_utf8_lossy(&out.stderr);
        assert_eq!(out.status.code(), Some(2), "{argv:?}: stderr:\n{err}");
        assert!(
            err.contains("--verifier-record-budget") && err.contains(extra_needle),
            "{argv:?}: stderr must name flag and {extra_needle:?}:\n{err}"
        );
    }

    // 5. Real-binary `--help` documents `--verifier-record-budget <COUNT>`.
    let help = run_cerulion(root, &["bag", "play", "--help"]);
    assert_eq!(help.status.code(), Some(0));
    let help_out = String::from_utf8_lossy(&help.stdout);
    assert!(
        help_out.contains("--verifier-record-budget <COUNT>") && help_out.contains("3000000"),
        "bag play --help must document --verifier-record-budget <COUNT>:\n{help_out}"
    );
}
