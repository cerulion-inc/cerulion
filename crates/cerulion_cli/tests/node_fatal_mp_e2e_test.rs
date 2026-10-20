// SPDX-License-Identifier: AGPL-3.0-only
//! `NodeError::Fatal` through the DEFAULT `graph run`: multi-process.
//!
//! On Unix with the real clock, an unpartitioned graph runs multi-process by
//! default (one worker process per node). The workspace is the one
//! `node_fatal_e2e_test` uses: `ticker` (a healthy 50 ms source) and `prober`
//! (a 10 ms source that returns the chosen error from tick 3 on).
//!
//! Arms:
//! 1. A fatal `prober`: its worker logs ONE fatal line, stops the node, keeps
//!    running until Ctrl-C, then exits 3 with an `Error:` line naming the node;
//!    the supervisor folds that exit into its own verdict and also exits 3.
//! 2. The control: a `logic` error keeps the node firing, and the run exits 0
//!    after Ctrl-C with no fatal line.
//!
//! The worker-exit classification itself (0 and 3 are clean exits, every
//! other status is a death) is pinned portably by the engine's unit tests.
//!
//! Prereq: `cargo build -p test_node_macro_period_cdylib -p test_node_error_kind_cdylib`.
//! Run alone: `cargo test -p cerulion_cli --test node_fatal_mp_e2e_test -- --test-threads=1`.

#![cfg(not(target_os = "macos"))]
// WAIVED WHOLE on macOS: upstream iceoryx2 0.10.0 defect 2034. Every arm here
// spawns a `cerulion` supervisor child that loads plugin nodes, and on macOS such a
// process cannot create any further event resource. The mechanism, the derivation
// that selects this file, and the coverage this costs are stated once in
// `cerulion_core/tests/upstream_waivers_test.rs`. Runs normally on Linux.
#![cfg(unix)]

use std::io::Read as _;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use serial_test::serial;

mod mp_support;
use mp_support::{dylib_file, signal_live_process, ChildGuard};

fn read_file(p: &Path) -> String {
    let mut s = String::new();
    if let Ok(mut f) = std::fs::File::open(p) {
        let _ = f.read_to_string(&mut s);
    }
    s
}

/// The same workspace as `node_fatal_e2e_test`: an unpartitioned two-node
/// graph over the prebuilt fixture cdylibs.
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

/// `cerulion graph run <graph>` with NO `--single-process`: the multi-process
/// default. stdin is `/dev/null`, stdout and stderr go to files; the workers
/// inherit the supervisor's stderr, so their lines land in the same file.
/// The supervisor leads its own process group, so the guard's teardown
/// reaches every worker and its orphan verdict can check them.
fn spawn_run(root: &Path, graph: &str, kind: &str) -> (ChildGuard, PathBuf) {
    let stderr_path = root.join(format!("{graph}.stderr"));
    let home = root.join("cerhome");
    std::fs::create_dir_all(&home).unwrap();
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(["graph", "run", graph])
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        // The shipped default (free run), whatever the developer exported.
        .env_remove("CERULION_EXECUTION_MODE")
        .env("CERULION_HOME", &home)
        .env("CERULION_NETWORK", "off")
        .env("CERULION_TEST_ERROR_KIND", kind)
        .env("CERULION_TEST_ERROR_AT", "3")
        .env("NO_COLOR", "1")
        .env("RUST_LOG", "info")
        .stdin(Stdio::null())
        .stdout(Stdio::from(
            std::fs::File::create(root.join(format!("{graph}.stdout"))).unwrap(),
        ))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()));
    let guard = ChildGuard::spawn_group_leader(&mut cmd).expect("spawn cerulion graph run");
    (guard, stderr_path)
}

/// The supervisor's GO line: every worker is up and stepping.
const GO_MARKER: &str = "GO signaled; deployment live";

const FATAL_LINE: &str = "node returned a fatal error and is stopped";

/// The scheduler's line for the SECOND failure of a node with the same
/// reason. Later repeats log at debug, so this line is the proof that a node
/// kept firing after its first error.
const LATER_FAILURE_LINE: &str = "node tick failed again with the same reason";

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

/// Wait for GO, then check the supervisor runs both workers: the run is
/// really multi-process, so the arm tests the default path and not the
/// monolith. Noting them is also what lets the orphan verdict check them.
fn assert_two_workers(guard: &mut ChildGuard, stderr: &Path) {
    assert!(
        wait_for_log(stderr, GO_MARKER, Duration::from_secs(60)),
        "the deployment never went live; log:\n{}",
        read_file(stderr)
    );
    let pids = guard.note_workers().to_vec();
    assert_eq!(
        pids.len(),
        2,
        "expected two worker processes (one per node), observed {pids:?}; log:\n{}",
        read_file(stderr)
    );
}

#[test]
#[serial]
fn a_fatal_node_in_the_default_multi_process_run_exits_3() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    build_workspace(root, "fatalmp", "nfatalmp1");
    let (mut guard, stderr) = spawn_run(root, "fatalmp", "fatal");
    assert_two_workers(&mut guard, &stderr);
    assert!(
        wait_for_log(&stderr, FATAL_LINE, Duration::from_secs(60)),
        "the fatal line never appeared; log:\n{}",
        read_file(&stderr)
    );
    signal_live_process(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .expect("run did not exit");
    let log = read_file(&stderr);
    assert_eq!(
        status.code(),
        Some(3),
        "a fatal node exits 3 on the default path; log:\n{log}"
    );

    let fatal_lines: Vec<&str> = log.lines().filter(|l| l.contains(FATAL_LINE)).collect();
    assert_eq!(fatal_lines.len(), 1, "exactly one fatal line; log:\n{log}");
    assert!(
        fatal_lines[0].contains("node_id=prober") && fatal_lines[0].contains("kind=fatal"),
        "names the node and the kind: {}",
        fatal_lines[0]
    );
    // The worker's verdict names the node and the message ...
    assert!(
        log.contains("Error: node 'prober' stopped on a fatal error: Fatal error: probe fatal"),
        "the worker's verdict names the node; log:\n{log}"
    );
    // ... and the supervisor's names the worker group, once.
    assert_eq!(
        log.matches("stopped on a fatal error (that worker's")
            .count(),
        1,
        "the supervisor's verdict names one worker group; log:\n{log}"
    );
    // A fatal node is not a worker death: the deployment is not degraded.
    assert!(
        !log.contains("ran DEGRADED") && !log.contains("worker process died"),
        "a fatal node must not read as a crashed worker; log:\n{log}"
    );
    guard.finish().assert_clean();
}

#[test]
#[serial]
fn a_logic_error_in_the_default_multi_process_run_exits_0() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    build_workspace(root, "logicmp", "nfatalmp2");
    let (mut guard, stderr) = spawn_run(root, "logicmp", "logic");
    assert_two_workers(&mut guard, &stderr);
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
    guard.finish().assert_clean();
}
