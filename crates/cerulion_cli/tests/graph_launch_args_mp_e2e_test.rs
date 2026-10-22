// SPDX-License-Identifier: AGPL-3.0-only
//! Launch arguments over a REAL multi-process `cerulion graph run`: the arm
//! here needs the supervisor and its workers, so it lives apart from the
//! single-process `graph_launch_args_e2e_test`, which still runs on macOS.
//!
//! Prerequisite: `cargo build -p test_node_macro_param_cdylib -p
//! test_node_macro_data_trigger_cdylib`. `#[serial]` arms run on the default
//! iceoryx2 namespace; run this binary alone with `-- --test-threads=1`.

#![cfg(not(target_os = "macos"))]
// WAIVED WHOLE on macOS: upstream iceoryx2 0.10.0 defect 2034. The arm here
// spawns a `cerulion` supervisor child that loads plugin nodes, and on macOS such a
// process cannot create any further event resource. The mechanism, the derivation
// that selects this file, and the coverage this costs are stated once in
// `cerulion_core/tests/upstream_waivers_test.rs`. Runs normally on Linux.
#![cfg(unix)]

use std::process::{Command, Stdio};
use std::time::Duration;

use cerulion_bag::BagReader;
use serial_test::serial;

mod launch_args_support;
mod mp_support;
use launch_args_support::{args_graph, build_workspace, cerulion};
use mp_support::{
    read_file, signal_live_process, wait_for_bag, wait_for_bag_state, ChildGuard,
    RECORDED_WINDOW_TIMEOUT,
};

/// The other side of the `--yes` guard: when the graph file already carries
/// exactly the partition this run derives, there is nothing to save, so a
/// wiring override with `--auto-partition --yes` runs. The partition is saved
/// for the declared defaults by `graph partition --yes`; `side:=right` then
/// moves the camera topic but not the node set, so the re-derived
/// process-per-node partition is the saved one. The run reaches the
/// supervisor GO, delivers frames on the overridden `/<ns>/right/cmd` (never
/// on the default `/<ns>/left/cmd`), records the resolved wiring as its
/// `graph.yaml`, exits 0 on Ctrl-C, says the file already carries the
/// partition, and leaves the file byte-identical with no new backup.
#[test]
#[serial]
fn a_yes_run_under_a_wiring_override_runs_when_the_saved_partition_is_already_current() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    let ns = format!("largscur{}", std::process::id());
    build_workspace(root, &args_graph(&ns));
    let graph_path = root.join("graphs/demo.yaml");
    let backup_path = root.join("graphs/demo.yaml.bak");

    let (status, stdout, stderr) = cerulion(root, &["graph", "partition", "demo", "--yes"]);
    assert!(
        status.success() && stdout.contains("Partition written to"),
        "graph partition --yes must save the declared partition:\n{stdout}\n{stderr}"
    );
    let saved = std::fs::read_to_string(&graph_path).unwrap();
    assert!(saved.contains("process_groups:"), "{saved}");
    std::fs::remove_file(&backup_path).expect("the partition save wrote a backup");

    let stderr_path = root.join("yes-current.stderr");
    let recordings = root.join("yes-current-recordings");
    let record_flag = format!("--record={}", recordings.display());
    let mut guard = ChildGuard::spawn_group_leader(
        Command::new(env!("CARGO_BIN_EXE_cerulion"))
            .args([
                "graph",
                "run",
                "demo",
                "side:=right",
                "--auto-partition",
                "--yes",
                "--no-validate",
                &record_flag,
            ])
            .current_dir(root)
            .env_remove("CARGO_TARGET_DIR")
            .env_remove("CERULION_EXECUTION_MODE")
            .env("CERULION_NETWORK", "off")
            .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
            .stdin(Stdio::null())
            .stdout(Stdio::from(
                std::fs::File::create(root.join("yes-current.stdout")).unwrap(),
            ))
            .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap())),
    )
    .expect("spawn cerulion graph run");
    let deadline = std::time::Instant::now() + Duration::from_secs(90);
    while !read_file(&stderr_path).contains("GO signaled; deployment live") {
        if let Ok(Some(early)) = guard.try_wait_noting() {
            panic!(
                "the run exited {early:?} before GO:\n{}",
                read_file(&stderr_path)
            );
        }
        assert!(
            std::time::Instant::now() < deadline,
            "no supervisor GO within 90 s:\n{}",
            read_file(&stderr_path)
        );
        std::thread::sleep(Duration::from_millis(100));
    }
    let workers = guard.note_workers().len();
    assert!(workers > 0, "a multi-process run has workers to reap");
    // The override is observable on the wire: the camera publishes on the
    // `right` topic and the relay, wired to that source, forwards it.
    let right_topic = format!("/{ns}/right/cmd");
    let left_topic = format!("/{ns}/left/cmd");
    let relay_topic = format!("/{ns}/relay/cmd");
    let bag = wait_for_bag(&recordings, Duration::from_secs(60))
        .unwrap_or_else(|| panic!("no bag appeared; stderr:\n{}", read_file(&stderr_path)));
    wait_for_bag_state(
        &bag,
        "frames on the overridden camera topic and the relay",
        RECORDED_WINDOW_TIMEOUT,
        |snap| snap.frames_on(&right_topic) >= 3 && snap.frames_on(&relay_topic) >= 3,
    );
    signal_live_process(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(90))
        .expect("graph run did not exit after SIGINT");
    let err = read_file(&stderr_path);
    assert!(
        status.success(),
        "Ctrl-C must exit 0, got {status:?}:\n{err}"
    );
    guard.finish().assert_clean();
    assert!(
        err.contains("already carries exactly the derived partition")
            && !err.contains("change its wiring"),
        "{err}"
    );
    assert_eq!(
        std::fs::read_to_string(&graph_path).unwrap(),
        saved,
        "the graph file is byte-identical"
    );
    assert!(!backup_path.exists(), "no backup was written");

    let reader = BagReader::open(&bag).expect("open bag");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(completeness.is_finalized(), "{completeness:?}");
    assert_eq!(
        msgs.iter().filter(|m| m.topic == left_topic).count(),
        0,
        "nothing is published on the default topic under `side:=right`"
    );
    let graph_yaml = reader
        .attachment("graph.yaml")
        .expect("read attachment")
        .expect("graph.yaml attachment present");
    let graph_yaml = String::from_utf8_lossy(&graph_yaml.data).into_owned();
    for line in [
        format!("topic: /{ns}/right/cmd"),
        format!("source: /{ns}/right/cmd"),
        "frame_id: right_cam".to_string(),
    ] {
        assert!(
            graph_yaml.contains(&line),
            "the recorded graph is the resolved one, missing `{line}`:\n{graph_yaml}"
        );
    }
    assert!(
        !graph_yaml.contains("left") && !graph_yaml.contains("args:"),
        "{graph_yaml}"
    );
}
