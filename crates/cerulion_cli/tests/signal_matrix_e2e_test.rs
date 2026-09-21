// SPDX-License-Identifier: AGPL-3.0-only
//! Signal-matrix acceptance: PIN that the `cerulion` binary's ONE
//! `setup_ctrlc_handler` shuts a long-running verb down CLEANLY for EACH of
//! SIGINT, SIGTERM, and SIGHUP.
//!
//! SIGINT already exits cleanly (the interactive Ctrl-C path). SIGTERM + SIGHUP
//! ride the ctrlc crate's `termination` feature. That feature is an
//! EXPLICIT `cerulion_cli` dependency (the DECLARED contract) so the coverage no
//! longer merely leans on the `cfg(unix)` `cerulion_bagd` dep transitively
//! unifying it in.
//!
//! Scope: this file pins the END-TO-END BEHAVIOR (each signal → graceful
//! exit 0), NOT the Cargo line itself. Reverting ONLY the explicit dependency
//! stays GREEN, because bagd's incidental feature unification still supplies
//! `termination`; the matrix regresses (SIGTERM/SIGHUP fall back to their
//! default disposition — terminate the process with a signal, NOT a clean
//! exit-0 — and the matching arms below FAIL) only if BOTH sources of the
//! feature vanish. The explicit Cargo line is the declared contract; this file
//! is the behavioral floor beneath it.
//!
//! Target verb: `cerulion graph run <demo> --single-process` (a period node —
//! the ONLY zero-argument verb that genuinely BLOCKS on the live loop; `topic
//! echo`/`hz` REQUIRE a live topic via `require_topic_exists`, so on a
//! nonexistent topic they exit non-zero IMMEDIATELY rather than blocking on the
//! signal). Hermetic: `CERULION_NETWORK=off` keeps the run LOCAL-ONLY (no
//! gateway/scouting), so this file pins ONLY the signal → clean-exit contract.
//!
//! Grounded exit-code semantics (asserted below): a clean signal shutdown
//! returns exit code 0 (`graph_run` returns `Ok(())` → `main` maps it to 0).
//! The topic-introspect e2e already pins SIGINT→0 for `graph run`; this file
//! extends it to the full three-signal matrix + repeat-signal idempotency.
//!
//! Needs `cargo build -p test_node_macro_period_cdylib` first (panics with that
//! instruction if the fixture artifact is missing). `#[cfg(unix)]` (signals);
//! `#[serial]` + unique per-test prefixes (the run looks on the DEFAULT
//! iceoryx2 namespace, so topic names must not collide across serial tests).

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::process::{Command, ExitStatus, Stdio};
use std::time::{Duration, Instant};

use serial_test::serial;

// Reusable subprocess helpers (ChildGuard pid teardown + reap, bounded waits,
// send_signal, read_file, the prebuilt-fixture resolvers). `#![allow(dead_code)]`
// in the module covers the helpers this binary does not use.
mod mp_support;
use mp_support::{dylib_file, fixture_cdylib, read_file, send_signal, ChildGuard};

const PERIOD_FIXTURE: &str = "test_node_macro_period_cdylib";

/// Hand-build a minimal single-node graph workspace in `root`: a `Cargo.toml`,
/// a `graphs/demo.yaml` (one `ticker` period node producing
/// `/{prefix}/ticker/cmd`), a `nodes/ticker/src/lib.rs` copy of the fixture
/// source (for the metadata walkers), and a `target/debug/libticker.*` copy of
/// the prebuilt fixture cdylib. Mirrors
/// `topic_introspect_cli_e2e_test.rs`'s `build_workspace`.
fn build_workspace(root: &Path, prefix: &str) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("nodes/ticker/src")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    std::fs::write(
        root.join("graphs/demo.yaml"),
        format!(
            "name: demo\nprefix: {prefix}\nnodes:\n- id: ticker\n  type: ticker\n  inputs: []\n  \
             outputs:\n  - name: cmd\n    schema: geometry_msgs/Vector3\n"
        ),
    )
    .unwrap();
    let fixture_src = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test_fixtures")
        .join(PERIOD_FIXTURE)
        .join("src/lib.rs");
    std::fs::copy(&fixture_src, root.join("nodes/ticker/src/lib.rs")).expect("copy fixture src");
    std::fs::copy(
        fixture_cdylib(PERIOD_FIXTURE),
        root.join("target/debug").join(dylib_file("ticker")),
    )
    .expect("copy fixture cdylib");
}

/// Spawn `cerulion graph run demo --no-validate --single-process` (LOCAL-ONLY —
/// `CERULION_NETWORK=off`) in `root`, redirecting stdout/stderr to files.
fn spawn_graph_run(root: &Path) -> (ChildGuard, PathBuf) {
    let stderr_path = root.join("run.stderr");
    let child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["graph", "run", "demo", "--no-validate", "--single-process"])
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        // Hermetic: no gateway/scouting — this file pins ONLY the signal path.
        .env("CERULION_NETWORK", "off")
        .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
        .stdin(Stdio::null())
        .stdout(Stdio::from(
            std::fs::File::create(root.join("run.stdout")).unwrap(),
        ))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()))
        .spawn()
        .expect("spawn cerulion graph run");
    // `--single-process` + `CERULION_NETWORK=off`: a monolith with no worker and
    // no gateway, so there is no subtree to signal. Deliberately NOT a group
    // leader — this file's arms signal the pid and assert exit code EXACTLY 0,
    // and a group signal would be a different stimulus than the one under test.
    (ChildGuard::single_process(child), stderr_path)
}

/// Bounded poll for `needle` in the (growing) log file; panics at the deadline
/// (the caller's `ChildGuard` reaps on the panic unwind).
fn wait_for_log_line(path: &Path, needle: &str, timeout: Duration) {
    let start = Instant::now();
    loop {
        if read_file(path).contains(needle) {
            return;
        }
        assert!(
            start.elapsed() < timeout,
            "log line {needle:?} not seen within {timeout:?}; log so far:\n{}",
            read_file(path)
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Spawn the blocking graph run, wait for live readiness, deliver each signal in
/// `signals` (in order, with a small gap so repeat signals land while the
/// process is still tearing down), and return the child's exit status. Panics
/// (reaping via `ChildGuard`) if it does not exit within the bound.
fn run_and_signal(prefix: &str, signals: &[libc::c_int]) -> ExitStatus {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path(), prefix);
    let (mut guard, stderr_path) = spawn_graph_run(tmp.path());

    // "starting graph (live)" is a `tracing::info!` → STDERR (init_logging writes
    // there so a command's stdout stays clean data). The live loop is blocking by
    // the time this prints.
    wait_for_log_line(
        &stderr_path,
        "starting graph (live)",
        Duration::from_secs(30),
    );

    let pid = guard.id();
    for (i, &sig) in signals.iter().enumerate() {
        if i > 0 {
            // Deliver the repeat while the first is still draining.
            std::thread::sleep(Duration::from_millis(50));
        }
        send_signal(pid, sig);
    }

    let status = guard
        .wait_bounded(Duration::from_secs(40))
        .unwrap_or_else(|| {
            panic!(
                "graph run did not exit after {signals:?}; stderr so far:\n{}",
                read_file(&stderr_path)
            )
        });
    // Belt-and-suspenders: `wait_bounded` already reaped via `try_wait`.
    // The guard's own teardown, not a bare reap: it notes the live worker set
    // first and renders the orphan verdict over it. A `single_process` guard
    // has no subtree, so this is `NotChecked` by design — the point is that the
    // reap goes through the type rather than around it.
    guard.finish().assert_clean();
    status
}

/// Assert the child exited CLEANLY on `sig` — exit code EXACTLY 0 (not
/// signal-terminated). A signal-terminated process has `code() == None` and
/// `success() == false`, so this is the discriminator that proves the handler
/// caught the signal and drove the graceful `running`-flip shutdown.
fn assert_clean_exit(status: ExitStatus, sig_name: &str) {
    assert!(
        status.success(),
        "graph run must exit CLEANLY on {sig_name} (the signal contract), got {status:?} \
         — a signal-terminated process here means the handler did NOT catch {sig_name}"
    );
    assert_eq!(
        status.code(),
        Some(0),
        "clean {sig_name} shutdown must return exit code 0, got {:?}",
        status.code()
    );
}

/// SIGINT → clean exit 0 (the interactive Ctrl-C path; the baseline).
#[test]
#[serial]
fn graph_run_exits_cleanly_on_sigint() {
    let status = run_and_signal("sigmata", &[libc::SIGINT]);
    assert_clean_exit(status, "SIGINT");
}

/// SIGTERM → clean exit 0 (the DIRECTED `kill <pid>` / systemd stop path — the
/// arm that FAILS if the ctrlc `termination` feature is ever lost).
#[test]
#[serial]
fn graph_run_exits_cleanly_on_sigterm() {
    let status = run_and_signal("sigmatb", &[libc::SIGTERM]);
    assert_clean_exit(status, "SIGTERM");
}

/// SIGHUP → clean exit 0 (terminal-hangup path; also `termination`-gated).
#[test]
#[serial]
fn graph_run_exits_cleanly_on_sighup() {
    let status = run_and_signal("sigmatc", &[libc::SIGHUP]);
    assert_clean_exit(status, "SIGHUP");
}

/// Repeat-signal idempotency: a SECOND SIGINT delivered while the process is
/// already tearing down does not panic/abort — the handler is a persistent,
/// idempotent `store(false)`, so the run still exits cleanly with code 0.
#[test]
#[serial]
fn graph_run_repeat_signal_is_idempotent() {
    let status = run_and_signal("sigmatd", &[libc::SIGINT, libc::SIGINT]);
    assert_clean_exit(status, "repeated SIGINT");
}
