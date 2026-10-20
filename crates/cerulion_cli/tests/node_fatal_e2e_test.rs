// SPDX-License-Identifier: AGPL-3.0-only
//! `NodeError::Fatal` through the real `cerulion` binary, on a user workspace.
//!
//! The workspace holds two node types: `ticker` (the prebuilt
//! `test_node_macro_period_cdylib`, a healthy 50 ms source) and `prober` (the
//! prebuilt `test_node_error_kind_cdylib`, a 10 ms source that publishes its
//! tick number in `out.x` and returns the chosen error from tick 3 on, both
//! chosen through `CERULION_TEST_ERROR_KIND` / `CERULION_TEST_ERROR_AT`).
//!
//! Arms:
//! 1. `graph run --record`: a fatal `prober` publishes exactly ticks 1 and 2
//!    (hand oracle `[1.0, 2.0]`), the `ticker` keeps publishing, the log has
//!    ONE fatal line naming the node, the kind and the message, the run exits
//!    3 after Ctrl-C, and `bag play --resim all --verify` on the bag exits 0:
//!    the re-executed node stops at the same step with byte-identical frames
//!    and the same schedule.
//! 2. `graph run` (no recording): the same fatal stop exits 3.
//! 3. The control, recorded: a `logic` error keeps today's behaviour, exit 0
//!    after Ctrl-C, with the same two frames (an erroring tick publishes
//!    nothing).
//!
//! Determinism is checked by resim, not by a second recording: a live
//! `graph run` advances its gating clock by measured wall time, so two live
//! recordings do not share a step timeline even without a fatal node. What
//! must hold is Replay = Live: the resim of THIS recording re-fires the same
//! schedule, stop included (`--verify` exits 6 on a diverged schedule). Two
//! runs on the virtual clock are compared bit for bit in
//! `cerulion_core/tests/node_fatal_stop_test.rs`.
//!
//! These arms force `--single-process`; the multi-process default has its
//! own binary, `node_fatal_mp_e2e_test` (Linux only).
//!
//! Prereq: `cargo build -p test_node_macro_period_cdylib -p test_node_error_kind_cdylib`.
//! Run alone: `cargo test -p cerulion_cli --test node_fatal_e2e_test -- --test-threads=1`.

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use cerulion_bag::BagReader;
use serial_test::serial;

mod mp_support;
use mp_support::{
    dylib_file, read_file, signal_live_process, wait_for_bag, wait_for_bag_state, ChildGuard,
    RECORDED_WINDOW_TIMEOUT,
};

/// Kills any `bagd` the recording run of graph `.0` spawned (its own process
/// group), so a panicking arm never orphans it. Matched by the bag's
/// `<graph>_` file prefix, so it never reaches another suite's recorder.
struct BagdGuard(&'static str);
impl Drop for BagdGuard {
    fn drop(&mut self) {
        if let Ok(out) = Command::new("pgrep")
            .args(["-f", &format!("bagd --out recordings/{}_", self.0)])
            .output()
        {
            for pid in String::from_utf8_lossy(&out.stdout)
                .lines()
                .filter_map(|l| l.trim().parse::<i32>().ok())
            {
                // SAFETY: killpg(2) on bagd's own process group; no memory is
                // touched. ESRCH after a clean exit is the expected no-op.
                unsafe {
                    libc::killpg(pid, libc::SIGKILL);
                }
            }
        }
    }
}

/// `nodes/<type>/` for each node (the fixture's source, for the metadata
/// walkers) + `target/debug/lib<type>.*` (the PREBUILT fixture cdylib) +
/// `graphs/<graph>.yaml`.
fn build_workspace(root: &Path, graph: &str, prefix: &str) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    std::fs::write(
        root.join(format!("graphs/{graph}.yaml")),
        format!(
            "name: {graph}\nprefix: {prefix}\nnodes:\n\
             - id: ticker\n  type: ticker\n  inputs: []\n  outputs:\n  - name: cmd\n    schema: geometry_msgs/Vector3\n\
             - id: prober\n  type: prober\n  inputs: []\n  outputs:\n  - name: out\n    schema: geometry_msgs/Vector3\n"
        ),
    )
    .unwrap();
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test_fixtures");
    for (node_type, fixture) in [
        ("ticker", "test_node_macro_period_cdylib"),
        ("prober", "test_node_error_kind_cdylib"),
    ] {
        std::fs::create_dir_all(root.join(format!("nodes/{node_type}/src"))).unwrap();
        std::fs::copy(
            fixtures.join(fixture).join("src/lib.rs"),
            root.join(format!("nodes/{node_type}/src/lib.rs")),
        )
        .expect("copy fixture src");
        std::fs::copy(
            cerulion_core::testing::find_fixture_cdylib(fixture),
            root.join("target/debug").join(dylib_file(node_type)),
        )
        .expect("copy fixture cdylib");
    }
}

/// `cerulion graph run <graph> --single-process [extra...]` in `root` with the
/// prober's error chosen by `kind`, stdout/stderr to files. The node reads
/// both keys from its env snapshot, which is taken from this process env.
fn spawn_run(root: &Path, graph: &str, kind: &str, extra: &[&str]) -> (ChildGuard, PathBuf) {
    let stderr_path = root.join(format!("{graph}.stderr"));
    let child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["graph", "run", graph, "--single-process"])
        .args(extra)
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        .env("CERULION_NETWORK", "off")
        .env("CERULION_TEST_ERROR_KIND", kind)
        .env("CERULION_TEST_ERROR_AT", "3")
        .env("NO_COLOR", "1")
        .env("RUST_LOG", "info")
        .stdout(Stdio::from(
            std::fs::File::create(root.join(format!("{graph}.stdout"))).unwrap(),
        ))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()))
        .spawn()
        .expect("spawn cerulion graph run");
    // `--single-process`: no worker subtree, so the teardown targets the pid.
    (ChildGuard::single_process(child), stderr_path)
}

const FATAL_LINE: &str = "node returned a fatal error and is stopped";

/// The scheduler's line for the SECOND failure of a node with the same
/// reason. Later repeats log at debug, so this line is the proof that a node
/// kept firing after its first error.
const LATER_FAILURE_LINE: &str = "node tick failed again with the same reason";

/// Poll `path` until it contains `needle` or `timeout` elapses.
fn wait_for_log(path: &Path, needle: &str, timeout: Duration) -> bool {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if read_file(path).contains(needle) {
            return true;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    false
}

/// `out.x` of every recorded frame on `topic`: the f64 right after the
/// 32-byte wire header of a `geometry_msgs/Vector3` frame.
fn recorded_x(bag: &Path, topic: &str) -> Vec<f64> {
    let reader = BagReader::open(bag).expect("open bag");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        completeness.is_finalized(),
        "bag must finalize: {completeness:?}"
    );
    msgs.iter()
        .filter(|m| m.topic == topic)
        .map(|m| f64::from_le_bytes(m.data[32..40].try_into().unwrap()))
        .collect()
}

#[test]
#[serial]
fn a_fatal_node_stops_exits_3_and_its_recording_resims_to_the_same_stop() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    build_workspace(root, "fatal", "nfatal1");
    let (mut guard, stderr) = spawn_run(root, "fatal", "fatal", &["--record=recordings"]);
    let _bagd = BagdGuard("fatal");

    let bag = wait_for_bag(&root.join("recordings"), Duration::from_secs(60))
        .expect("bagd never created the bag");
    assert!(
        wait_for_log(&stderr, FATAL_LINE, Duration::from_secs(60)),
        "the fatal line never appeared; log:\n{}",
        read_file(&stderr)
    );
    // Let the healthy ticker publish past the stop: the prober stops at its
    // 3rd 10 ms tick, and the 50 ms ticker's 2nd frame comes after that.
    wait_for_bag_state(&bag, "two ticker frames", RECORDED_WINDOW_TIMEOUT, |snap| {
        snap.frames_on("/nfatal1/ticker/cmd") >= 2
    });
    signal_live_process(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .expect("run did not exit");
    let log = read_file(&stderr);
    assert_eq!(status.code(), Some(3), "a fatal node exits 3; log:\n{log}");

    // ONE fatal line, naming the node, the kind and the message.
    let fatal_lines: Vec<&str> = log.lines().filter(|l| l.contains(FATAL_LINE)).collect();
    assert_eq!(fatal_lines.len(), 1, "exactly one fatal line; log:\n{log}");
    let line = fatal_lines[0];
    assert!(line.contains("node_id=prober"), "names the node: {line}");
    assert!(line.contains("kind=fatal"), "names the kind: {line}");
    assert!(
        line.contains("Fatal error: probe fatal"),
        "names the message: {line}"
    );
    // The verdict: the node's own message, the same text the in-process
    // macro form reports (the cdylib's `tick failed: ` label is not in it).
    assert_eq!(
        log.lines()
            .filter(|l| l.starts_with("Error: "))
            .collect::<Vec<_>>(),
        vec!["Error: node 'prober' stopped on a fatal error: Fatal error: probe fatal"],
        "ONE verdict line naming the node and its message; log:\n{log}"
    );

    // Delivered outputs: ticks 1 and 2 only; the ticker kept running.
    assert_eq!(recorded_x(&bag, "/nfatal1/prober/out"), vec![1.0, 2.0]);
    assert!(
        recorded_x(&bag, "/nfatal1/ticker/cmd").len() >= 2,
        "the rest of the graph keeps publishing"
    );

    // The recording re-executes to the same stop: byte-identical frames and
    // the same schedule (exit 6 would be a diverged schedule, 1 a data diff).
    // A resim runs under the CURRENT process env (it warns on a divergence
    // from the bag's env.json), so it gets the node's same two keys. This
    // `--verify` is the determinism check (see the module doc).
    let resim = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["bag", "play"])
        .arg(&bag)
        .args(["--resim", "all", "--verify"])
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        .env("CERULION_NETWORK", "off")
        .env("CERULION_TEST_ERROR_KIND", "fatal")
        .env("CERULION_TEST_ERROR_AT", "3")
        .env("NO_COLOR", "1")
        .env("RUST_LOG", "info")
        .output()
        .expect("run bag play --resim");
    let resim_err = String::from_utf8_lossy(&resim.stderr);
    assert_eq!(
        resim.status.code(),
        Some(0),
        "resim --verify of a fatal run must pass; stdout:\n{}\nstderr:\n{resim_err}",
        String::from_utf8_lossy(&resim.stdout),
    );
    // The re-executed node hit the fatal STOP, not a run of errors: one fatal
    // line, for the prober. (The fixture fails on every tick from 3, so the
    // frames alone cannot tell "stopped" from "kept erroring".)
    let resim_fatal: Vec<&str> = resim_err
        .lines()
        .filter(|l| l.contains(FATAL_LINE))
        .collect();
    assert_eq!(
        resim_fatal.len(),
        1,
        "the resim stops the node once; stderr:\n{resim_err}"
    );
    assert!(
        resim_fatal[0].contains("node_id=prober"),
        "{}",
        resim_fatal[0]
    );
}

#[test]
#[serial]
fn a_fatal_node_in_a_plain_run_exits_3() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    build_workspace(root, "plain", "nfatal2");
    let (mut guard, stderr) = spawn_run(root, "plain", "fatal", &[]);
    assert!(
        wait_for_log(&stderr, FATAL_LINE, Duration::from_secs(60)),
        "the fatal line never appeared; log:\n{}",
        read_file(&stderr)
    );
    signal_live_process(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .expect("run did not exit");
    assert_eq!(status.code(), Some(3), "log:\n{}", read_file(&stderr));
}

#[test]
#[serial]
fn a_logic_error_keeps_the_node_firing_and_the_run_exits_0() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    build_workspace(root, "logic", "nfatal3");
    let (mut guard, stderr) = spawn_run(root, "logic", "logic", &["--record=recordings"]);
    let _bagd = BagdGuard("logic");
    let bag = wait_for_bag(&root.join("recordings"), Duration::from_secs(60))
        .expect("bagd never created the bag");
    assert!(
        wait_for_log(&stderr, "node tick failed", Duration::from_secs(60)),
        "the tick-failure line never appeared; log:\n{}",
        read_file(&stderr)
    );
    // A LATER failure while the run is live: the node fired again after its
    // first error (tick 3) and failed on tick 4. One failure line alone would
    // also pass a node wrongly disabled after its first error.
    assert!(
        wait_for_log(&stderr, LATER_FAILURE_LINE, Duration::from_secs(60)),
        "the node never failed a second time, so it stopped firing; log:\n{}",
        read_file(&stderr)
    );
    signal_live_process(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .expect("run did not exit");
    let log = read_file(&stderr);
    assert_eq!(
        status.code(),
        Some(0),
        "a non-fatal error keeps exit 0; log:\n{log}"
    );
    assert!(!log.contains(FATAL_LINE), "no fatal line for a logic error");
    // The node kept firing but every tick from the 3rd returned the error,
    // which publishes nothing: ticks 1 and 2 only, as in the fatal arm.
    assert_eq!(recorded_x(&bag, "/nfatal3/prober/out"), vec![1.0, 2.0]);
}
