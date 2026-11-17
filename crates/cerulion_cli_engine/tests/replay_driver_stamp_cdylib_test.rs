// SPDX-License-Identifier: AGPL-3.0-only
//! The recording's driver stamp reaches the judge through the production
//! replay path.
//!
//! A crafted bag records `ticker` as an `external` (driver) node in its
//! `__cerulion/recorder.json` (`node_external`), and the workspace the replay
//! runs from builds `ticker` from the prebuilt `test_node_macro_period_cdylib`
//! fixture, an ordinary period node. `run_replay` reads and admits the stamp
//! at its gate, hands it to the engine, loads the workspace's cdylib, and the
//! engine refuses the build that runs `ticker` the other way before any pass
//! runs: `ReplayError::DriverSetDrift`, exit 2, naming the node. A replay that
//! lost the stamp between the gate and the engine would take the driver set
//! from the build, which declares no driver, and go on to replay the bag.
//!
//! Prerequisite (the test PANICS with the build instruction when it is
//! missing):
//! ```bash
//! cargo build -p test_node_macro_period_cdylib
//! ```
//!
//! The replay mints this process's resim transport and the test moves the
//! process's working directory into the workspace, so this binary holds one
//! test.

#![cfg(unix)]

use std::path::{Path, PathBuf};

use cerulion_bag::{BagWriter, BagWriterConfig, TopicSchema};
use cerulion_cli_engine::replay_cmd::{run_replay, DriverDrift, ReplayError, ReplayOptions};
use cerulion_cli_engine::replay_engine::{ReplayPurpose, SUPPORTED_TRACE_FORMAT};
use cerulion_core::trace_ring::{TraceRingRecord, RECORD_TYPE_FIRE};

/// The platform cdylib filename for a node type.
fn dylib_file(name: &str) -> String {
    if cfg!(target_os = "macos") {
        format!("lib{name}.dylib")
    } else {
        format!("lib{name}.so")
    }
}

/// Restores the working directory and `CARGO_TARGET_DIR` on drop, so a
/// failing assertion leaves the process as it found it. `CARGO_TARGET_DIR` is
/// cleared because the workspace's own `target/` is where its cdylib is.
struct ProcessStateGuard {
    cwd: PathBuf,
    target_dir: Option<std::ffi::OsString>,
}

impl ProcessStateGuard {
    fn enter(workspace: &Path) -> Self {
        let guard = Self {
            cwd: std::env::current_dir().expect("the current directory"),
            target_dir: std::env::var_os("CARGO_TARGET_DIR"),
        };
        std::env::remove_var("CARGO_TARGET_DIR");
        std::env::set_current_dir(workspace).expect("enter the workspace");
        guard
    }
}

impl Drop for ProcessStateGuard {
    fn drop(&mut self) {
        let _ = std::env::set_current_dir(&self.cwd);
        if let Some(v) = self.target_dir.take() {
            std::env::set_var("CARGO_TARGET_DIR", v);
        }
    }
}

/// A workspace whose `ticker` node type is the prebuilt period fixture: its
/// source (for the staleness walk), then its cdylib, so the library is the
/// newer of the two.
fn build_workspace(root: &Path) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("nodes/ticker/src")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    std::fs::copy(
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("the crates directory")
            .join("test_fixtures/test_node_macro_period_cdylib/src/lib.rs"),
        root.join("nodes/ticker/src/lib.rs"),
    )
    .expect("copy the ticker source");
    std::fs::copy(
        cerulion_core::testing::find_fixture_cdylib("test_node_macro_period_cdylib"),
        root.join("target/debug").join(dylib_file("ticker")),
    )
    .expect("copy the ticker cdylib");
}

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

/// A finalized bag of `ticker`'s graph: two FIRE records of it, its rank-0
/// trace manifest, and a recorder identity at this binary's trace format that
/// records `ticker` as a driver.
fn write_stamped_bag(path: &Path) {
    let graph = format!(
        "name: stampdrift\nprefix: stampdrift{pid}\nnodes:\n  - id: ticker\n    type: ticker\n    \
         outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n",
        pid = std::process::id()
    );
    let recorder = serde_json::json!({
        "arch": std::env::consts::ARCH,
        "os": std::env::consts::OS,
        "cerulion_version": "0.1.0",
        "recorded_at_ns": 42u64,
        "trace_format": SUPPORTED_TRACE_FORMAT,
        "coordination": "lockstep",
        "node_external": {"ticker": true},
    });
    let manifest = serde_json::json!({"rank": 0, "generation": 0, "node_ids": ["ticker"]});
    let attachments: Vec<(&str, &str, Vec<u8>)> = vec![
        ("graph.yaml", "application/yaml", graph.into_bytes()),
        ("env.json", "application/json", b"{}".to_vec()),
        (
            "__cerulion/trace_manifest_rank0.json",
            "application/json",
            serde_json::to_vec(&manifest).unwrap(),
        ),
        (
            "__cerulion/recorder.json",
            "application/json",
            serde_json::to_vec(&recorder).unwrap(),
        ),
    ];
    let topics = [TopicSchema {
        topic: "/stampdrift/unused".into(),
        schema_name: "std_msgs/UInt8".into(),
        schema_hash: 0x0102_0304_0506_0708,
        wire_fixed_size: 8,
    }];
    let trace = [fire(0), fire(1)];
    let mut w = BagWriter::create(path, BagWriterConfig::default(), &topics).unwrap();
    w.write_chunk(|c| {
        for (i, rec) in trace.iter().enumerate() {
            c.write_scheduler_trace(i as u32, rec.fire_time_ns, rec.fire_time_ns, rec)?;
        }
        Ok(())
    })
    .unwrap();
    for (name, media_type, bytes) in &attachments {
        w.write_attachment(name, media_type, 0, 0, bytes).unwrap();
    }
    w.finalize().unwrap();
}

/// The recording runs `ticker` as a driver; the workspace builds it as an
/// ordinary period node. `--verify` through `run_replay` is refused with
/// `DriverSetDrift` (exit 2) naming `ticker`, the recorded side `external`.
#[test]
fn a_build_that_runs_a_recorded_driver_as_an_ordinary_node_is_refused_through_run_replay() {
    let tmp = tempfile::tempdir().unwrap();
    let workspace = tmp.path().join("ws");
    build_workspace(&workspace);
    let bag = tmp.path().join("stamped.mcap");
    write_stamped_bag(&bag);

    let err = {
        let _state = ProcessStateGuard::enter(&workspace);
        run_replay(
            &bag,
            ReplayOptions {
                purpose: ReplayPurpose::Verify,
                ..ReplayOptions::default()
            },
        )
        .expect_err("a build that changes a recorded driver is refused")
    };
    match &err {
        ReplayError::DriverSetDrift { drifts } => assert_eq!(
            drifts,
            &vec![DriverDrift {
                node: "ticker".to_string(),
                recorded_external: true,
            }]
        ),
        other => panic!("expected DriverSetDrift, got {other:?}"),
    }
    assert_eq!(err.exit_code(), 2);
    assert!(
        err.to_string().contains(
            "\n  - ticker: recorded as an external (driver) node; the replayed build does not \
             declare it external\n"
        ),
        "the refusal names the node and the direction: {err}"
    );
}
