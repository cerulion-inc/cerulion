// SPDX-License-Identifier: AGPL-3.0-only
//! A recording from a newer Cerulion, over the REAL binary
//! (`CARGO_BIN_EXE_cerulion`).
//!
//! Pinned: `cerulion bag play <bag> --resim all --verify` on a bag whose
//! `__cerulion/recorder.json` stamps a `trace_format` one past the highest
//! this build reads exits 2, and stderr carries the refusal that names the
//! newer recording and says to upgrade, as one `Error:` line with no
//! corrupt-bag remedy. Every arm of that refusal is pinned in
//! `cerulion_cli_engine/tests/replay_gates_test.rs`; this file proves the
//! binary prints it and exits with its code.
//!
//! The bag is refused at its `recorder.json` gate, before any node library
//! loads, so no `#[serial]` is needed. The subprocess runs under an isolated
//! `HOME` and a temp cwd.

#![cfg(unix)]

use std::path::Path;
use std::process::{Command, Output};

use cerulion_bag::{BagWriter, BagWriterConfig, TopicSchema};
use cerulion_core::trace_ring::{TraceRingRecord, RECORD_TYPE_FIRE, SUPPORTED_TRACE_FORMAT};

const USER_TOPIC: &str = "/data";

fn fire(step: u64) -> TraceRingRecord {
    TraceRingRecord {
        step,
        fire_time_ns: 1000 + step,
        duration_ns: 10,
        node_idx: 0,
        global_level: 0,
        record_type: RECORD_TYPE_FIRE,
        reserved: 0,
    }
}

/// A finalized bag that is otherwise replay-grade (one user frame, two FIRE
/// records of the one manifest node, a graph that parses and validates), so
/// only `recorder_json`, its recorder stamp, can refuse it.
fn write_bag(path: &Path, recorder_json: &[u8]) {
    let graph =
        b"name: replaytest\nprefix: replaytest\nnodes:\n  - id: pub1\n    type: test_pub\n    \
      outputs:\n      - name: data\n        schema: test/Data\n";
    let manifest = serde_json::to_vec(&serde_json::json!({
        "rank": 0,
        "generation": 0,
        "node_ids": ["node0"],
    }))
    .unwrap();
    let topics = [TopicSchema {
        topic: USER_TOPIC.into(),
        schema_name: "std_msgs/UInt8".into(),
        schema_hash: 0x0102_0304_0506_0708,
        wire_fixed_size: 8,
    }];
    let payload = [0xABu8; 8];
    let mut w = BagWriter::create(path, BagWriterConfig::default(), &topics).unwrap();
    w.write_chunk(|c| {
        c.write_message(USER_TOPIC, 0, 1000, 1000, &[&payload[..]])?;
        for (i, rec) in [fire(0), fire(1)].iter().enumerate() {
            c.write_scheduler_trace(i as u32, rec.fire_time_ns, rec.fire_time_ns, rec)?;
        }
        Ok(())
    })
    .unwrap();
    let attachments: [(&str, &[u8]); 4] = [
        ("graph.yaml", graph),
        ("env.json", br#"{"RUST_LOG":"info"}"#),
        ("__cerulion/trace_manifest_rank0.json", &manifest),
        ("__cerulion/recorder.json", recorder_json),
    ];
    for (name, data) in attachments {
        let media_type = if name.ends_with(".yaml") {
            "application/yaml"
        } else {
            "application/json"
        };
        w.write_attachment(name, media_type, 0, 0, data).unwrap();
    }
    w.finalize().unwrap();
}

/// `cerulion bag play <bag> --resim all --verify` in `root`, under its own
/// `HOME`.
fn resim_verify(bag: &Path, root: &Path) -> Output {
    let home = root.join("home");
    std::fs::create_dir_all(&home).expect("home");
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["bag", "play", "--resim", "all", "--verify"])
        .arg(bag)
        .env("HOME", &home)
        .env("IOX2_LOG_LEVEL", "error")
        .env("CERULION_LOGIN_GATE", "off")
        .env("CERULION_NETWORK", "off")
        .env_remove("COMPLETE")
        .env_remove("RUST_LOG")
        .current_dir(root)
        .output()
        .expect("run cerulion")
}

/// A bag stamped one format past this build, by a recorder that stamps
/// `99.0.0`, exits 2 with one `Error:` line, the sentence an operator reads,
/// written out by hand, and nothing that calls the recording corrupt or says
/// to re-record it.
#[test]
fn a_bag_from_a_newer_cerulion_exits_2_and_says_to_upgrade() {
    let root = tempfile::tempdir().expect("tempdir");
    let newer = SUPPORTED_TRACE_FORMAT + 1;
    let bag = root.path().join("newer.mcap");
    write_bag(
        &bag,
        &serde_json::to_vec(&serde_json::json!({
            "arch": std::env::consts::ARCH,
            "os": std::env::consts::OS,
            "cerulion_version": "99.0.0",
            "recorded_at_ns": 42u64,
            "trace_format": newer,
            "coordination": "lockstep",
        }))
        .unwrap(),
    );
    let out = resim_verify(&bag, root.path());
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert_eq!(out.status.code(), Some(2), "stderr: {stderr}");
    let line = format!(
        "Error: this bag was recorded by a newer Cerulion (99.0.0): its scheduler-trace format \
         is version {newer}, and this Cerulion ({}) reads up to version \
         {SUPPORTED_TRACE_FORMAT}. Upgrade Cerulion to replay it.",
        env!("CARGO_PKG_VERSION")
    );
    // Every line that carries `Error:`, wherever on the line: exactly the one
    // sentence, whole, on a line of its own.
    let errors: Vec<&str> = stderr.lines().filter(|l| l.contains("Error:")).collect();
    assert_eq!(errors, [line.as_str()], "stderr: {stderr}");
    for word in [
        "corrupt",
        "re-record",
        "internal consistency",
        "panicked at",
    ] {
        assert!(!stderr.contains(word), "{word:?} in stderr: {stderr}");
    }
}
