// SPDX-License-Identifier: AGPL-3.0-only
//! **A `process_groups:` run FREE-RUNS by default, over the REAL binary**, and
//! `CERULION_EXECUTION_MODE=lockstep` opts it back out.
//!
//! The resolver pins in `cerulion_cli_engine::graph_cmd`
//! (`the_supervisor_route_free_runs_by_default_and_lockstep_opts_out` and its
//! siblings) prove the PURE decision; this file proves the decision REACHES a
//! deployment: the supervisor's barrier creation, each worker's build path and
//! the run directory the supervisor writes all follow the one resolved mode,
//! on a run nobody exported anything for.
//!
//! # Witnesses: positive first, absences second
//!
//! Every arm's verdict rests on POSITIVE monotone witnesses read off the run:
//! `run.json`'s `gating` label (written from the ONE resolved mode through
//! `GatingClock::classify`: `recorded_wall` for a free-run rank that mints its
//! trace ring, which every plain multi-process run does, `quantum` for a
//! lockstep one) and each worker's own `build_path=` line (`FreeRunTraced` /
//! `Lockstep`, one per rank). The breadcrumb ABSENCES ("no shared barrier",
//! "left the barrier cohort") are a second, weaker layer, and each arm counts
//! BOTH breadcrumb families so the two supervisor arms are each other's
//! positive control. The apparatus itself is proven before any absence is
//! believed: the descriptor was found and parsed, the GO breadcrumb landed,
//! and exactly two worker build lines were captured.
//!
//! # Arms
//!
//! 1. DEFAULT: the variable REMOVED from the child's environment ⇒
//!    `gating: recorded_wall` (and `partition.process_groups: true`), both
//!    workers `build_path=FreeRunTraced`, the supervisor creates no barrier, no
//!    worker leaves a cohort.
//! 2. OPT-OUT: `CERULION_EXECUTION_MODE=lockstep` ⇒ `gating: quantum`, both
//!    workers `build_path=Lockstep`, both leave the barrier cohort on
//!    shutdown, no free-run breadcrumb.
//! 3. NEGATIVE CONTROL: `--single-process`, variable removed ⇒ the monolith
//!    route did not move: `partition.process_groups: false`, the read-only
//!    `wall` label it always carried, no worker line, no barrier breadcrumb of
//!    either kind.
//!
//! The `--record` twin of arm 1 is `mp_record_e2e_test`'s `RecordMode::Default`
//! arm (the bag's `coordination: free_run` stamp + the epoch-placed streams);
//! the record -> resim -> verify twin is `plain_run_resim_e2e_test`'s default
//! one-rank arm.
//!
//! Hermetic: `CERULION_NETWORK=off`, an owned `CERULION_HOME` (the run
//! directory lands in the tempdir), and the execution mode decided in all
//! three directions by `SpawnExecutionMode`, never inherited from the
//! developer's shell. `#![cfg(unix)]` (the supervisor is real on Linux AND
//! macOS) and `#[serial]` (the planning build and the data plane share
//! process-global iceoryx2 namespaces).
//!
//! Prerequisites, the repo's fixture pattern (the helpers PANIC with the exact
//! instruction if missing):
//! `cargo build -p test_node_macro_period_cdylib -p test_node_macro_data_trigger_cdylib`

#![cfg(not(target_os = "macos"))]
// WAIVED WHOLE on macOS: upstream iceoryx2 0.10.0 defect 2034. Every arm here
// spawns a `cerulion` supervisor child that loads plugin nodes, and on macOS such a
// process cannot create any further event resource. The mechanism, the derivation
// that selects this file, and the coverage this costs are stated once in
// `cerulion_core/tests/upstream_waivers_test.rs`. Runs normally on Linux.
#![cfg(unix)]

use std::path::Path;
use std::time::{Duration, Instant};

use serial_test::serial;

mod mp_support;
use mp_support::*;

/// `RUST_LOG` raising the engine to `debug`: the worker's clean-shutdown cohort
/// lines ("worker left the barrier cohort", "no barrier cohort to leave") are
/// lifecycle bookkeeping and ride `debug!`, filtered at the spawn helper's
/// `cerulion_cli_engine=info` default. The arms count them as evidence of
/// which exit path every worker took, so they ask for them explicitly (a
/// dev-profile binary, where `debug!` is compiled in).
const ENGINE_DEBUG: (&str, &str) = (
    "RUST_LOG",
    "cerulion=info,cerulion_cli_engine=debug,cerulion_bagd=info",
);

/// Strip ANSI SGR escapes from captured output.
///
/// LOAD-BEARING, not cosmetic: `tracing`'s default formatter wraps a
/// structured field's name and its `=` in separate escape sequences, so the
/// rendered line contains
/// `<esc>[3mbuild_path<esc>[0m<esc>[2m=<esc>[0mFreeRunTraced` and the literal
/// `build_path=FreeRunTraced` is NOT a substring of it. Every `key=value`
/// assertion below matches the stripped capture.
fn strip_ansi(s: &str) -> String {
    let bytes = s.as_bytes();
    let mut out = String::with_capacity(s.len());
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == 0x1b && i + 1 < bytes.len() && bytes[i + 1] == b'[' {
            // Skip to the final byte of the CSI sequence (`@`..`~`).
            i += 2;
            while i < bytes.len() && !(0x40..=0x7e).contains(&bytes[i]) {
                i += 1;
            }
            i += 1; // consume the final byte
            continue;
        }
        out.push(bytes[i] as char);
        i += 1;
    }
    out
}

/// What one run left behind: its `run.json` (read while the run was LIVE; the
/// directory is removed at exit) and its ANSI-stripped stdout + stderr.
struct RunOutcome {
    run_json: serde_json::Value,
    log: String,
}

/// Block until a run directory carrying a parseable `run.json` appears under
/// `runs`, a bound in seconds: load can delay a spawn, it cannot make an
/// unwritten descriptor appear.
fn wait_for_run_json(runs: &Path, stdout_path: &Path, stderr_path: &Path) -> serde_json::Value {
    let deadline = Instant::now() + Duration::from_secs(90);
    loop {
        let found = std::fs::read_dir(runs).ok().and_then(|d| {
            d.filter_map(Result::ok)
                .map(|e| e.path())
                .find(|p| p.join("run.json").exists())
        });
        if let Some(dir) = found {
            // A descriptor caught mid-write does not parse; keep waiting.
            if let Ok(v) =
                serde_json::from_str::<serde_json::Value>(&read_file(&dir.join("run.json")))
            {
                return v;
            }
        }
        assert!(
            Instant::now() < deadline,
            "no parseable run.json under {} within 90 s\nstdout:\n{}\nstderr:\n{}",
            runs.display(),
            read_file(stdout_path),
            read_file(stderr_path)
        );
        std::thread::sleep(Duration::from_millis(100));
    }
}

/// Block until `needle` shows up in either capture, bounded.
fn wait_for_line(stdout_path: &Path, stderr_path: &Path, needle: &str, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    loop {
        if read_file(stdout_path).contains(needle) || read_file(stderr_path).contains(needle) {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
}

/// Run `cerulion graph run mpdemo [extra]` under `mode` (NO `--record`), wait
/// for the run descriptor and, on a supervisor arm, for GO; give the
/// deployment a healthy window, SIGINT it, require exit 0, and hand back what
/// it left behind.
fn run_and_stop(
    prefix: &str,
    extra: &[&str],
    mode: SpawnExecutionMode,
    expect_supervisor: bool,
) -> RunOutcome {
    let tmp = tempfile::tempdir().unwrap();
    build_mp_workspace(tmp.path(), prefix);
    let home = tmp.path().join("cerhome");
    std::fs::create_dir_all(&home).unwrap();
    let home_str = home.to_str().expect("tempdir paths are UTF-8").to_owned();
    let (mut guard, stdout_path, stderr_path) = spawn_graph_run_graph(
        tmp.path(),
        "mpdemo",
        false,
        extra,
        &[("CERULION_HOME", &home_str), ENGINE_DEBUG],
        mode,
    );
    let _bagd_guard = BagdGuard::arm();

    // The descriptor is written BEFORE dispatch and removed at exit: read it
    // while the run is live.
    let run_json = wait_for_run_json(&home.join("runs"), &stdout_path, &stderr_path);
    if expect_supervisor {
        assert!(
            wait_for_line(
                &stdout_path,
                &stderr_path,
                "GO signaled; deployment live",
                Duration::from_secs(90)
            ),
            "the deployment never went live (mode={mode:?})\nstdout:\n{}\nstderr:\n{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        );
    }
    std::thread::sleep(Duration::from_secs(3));

    // REAP FIRST, then signal. `signal_live_process` reads liveness with
    // `kill`, and a run that exited during the sleep above is a ZOMBIE until
    // its parent collects it: `kill` still lands on a zombie, so the liveness
    // assert would pass on a run that was already over. `try_wait_noting`
    // collects that child (and notes the live worker set while there is still
    // one to note), which turns an early exit into the failure below instead
    // of a silently vacuous signal.
    assert!(
        guard
            .try_wait_noting()
            .expect("polling the graph run for an early exit must not fail")
            .is_none(),
        "the graph run (mode={mode:?}, extra={extra:?}) exited on its own before the SIGINT, \
         so nothing below exercises the signal path\nstdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );
    // ALIVE, not merely signalled. A run that exited on its own before this
    // point satisfies `wait_bounded`, `status.success()` and every log assert
    // below without ever receiving the SIGINT, so the arm would be reporting a
    // graceful shutdown it never exercised.
    signal_live_process(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(90))
        .expect("graph run did not exit after SIGINT");
    assert!(
        status.success(),
        "graph run (mode={mode:?}, extra={extra:?}) must exit 0 on SIGINT, got {status:?}\n\
         stdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );
    let log = strip_ansi(&format!(
        "{}\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    ));
    RunOutcome { run_json, log }
}

/// The apparatus saw both workers: exactly two build lines, no more (a third
/// would be a respawn) and no fewer (a missing one would make every per-rank
/// count below vacuous).
fn assert_two_worker_build_lines(log: &str) {
    assert_eq!(
        log.matches("worker build path resolved from the stamped execution mode")
            .count(),
        2,
        "both workers log their resolved build path to the shared stderr; log was:\n{log}"
    );
}

/// Arm 1, DEFAULT: nothing exported ⇒ the deployment free-runs.
#[test]
#[serial]
fn a_process_groups_run_free_runs_by_default() {
    let out = run_and_stop("emdef", &[], SpawnExecutionMode::Default, true);
    // POSITIVE witness 1: the descriptor names the EXECUTED deployment and the
    // gating clock of the mode the resolver chose with nothing exported. A
    // plain multi-process run mints its scheduler-trace rings, so every rank
    // runs the controlled wall-following clock (`recorded_wall`); `wall` is
    // the `--no-rings` shape and `quantum` the opt-out.
    assert_eq!(
        out.run_json["partition"]["process_groups"],
        serde_json::json!(true),
        "the supervisor route executed: {}",
        out.run_json
    );
    assert_eq!(
        out.run_json["gating"],
        serde_json::json!("recorded_wall"),
        "a default supervisor run puts every rank on its own wall-following clock \
         (`FreeRunTraced`), never a handed quantum: {}",
        out.run_json
    );
    // POSITIVE witness 2: each worker's OWN build line names the traced
    // free-run path.
    assert_two_worker_build_lines(&out.log);
    assert_eq!(
        out.log.matches("build_path=FreeRunTraced").count(),
        2,
        "both workers must resolve the traced free-run build path; log was:\n{}",
        out.log
    );
    assert_eq!(
        out.log.matches("build_path=Lockstep").count(),
        0,
        "no worker may resolve the lockstep path on a default run; log was:\n{}",
        out.log
    );
    assert_eq!(
        out.log.matches("build_path=FreeRunLive").count(),
        0,
        "no worker may resolve the ring-less live path on a run that mints rings; log was:\n{}",
        out.log
    );
    // Second layer: the supervisor and workers say what they did NOT build.
    assert_eq!(
        out.log
            .matches("free-run deployment: no shared barrier is created")
            .count(),
        1,
        "the supervisor creates no barrier for a free-run deployment; log was:\n{}",
        out.log
    );
    assert_eq!(
        out.log.matches("no barrier cohort to leave").count(),
        2,
        "both free-run workers exit with no cohort to leave; log was:\n{}",
        out.log
    );
    assert_eq!(
        out.log.matches("worker left the barrier cohort").count(),
        0,
        "no free-run worker ever leaves a cohort; log was:\n{}",
        out.log
    );
}

/// Arm 2, OPT-OUT: `CERULION_EXECUTION_MODE=lockstep` ⇒ the barrier
/// deployment.
#[test]
#[serial]
fn the_lockstep_opt_out_builds_the_barrier_deployment() {
    let out = run_and_stop("emlock", &[], SpawnExecutionMode::Lockstep, true);
    assert_eq!(
        out.run_json["partition"]["process_groups"],
        serde_json::json!(true),
        "the supervisor route executed: {}",
        out.run_json
    );
    assert_eq!(
        out.run_json["gating"],
        serde_json::json!("quantum"),
        "the opt-out hands every rank the same quantum behind the barrier: {}",
        out.run_json
    );
    assert_two_worker_build_lines(&out.log);
    assert_eq!(
        out.log.matches("build_path=Lockstep").count(),
        2,
        "both workers must resolve the lockstep build path under the opt-out; log was:\n{}",
        out.log
    );
    assert_eq!(
        out.log.matches("build_path=FreeRun").count(),
        0,
        "no worker may resolve either free-run path under the opt-out; log was:\n{}",
        out.log
    );
    // The cohort machinery RAN on both ranks: the lockstep-only breadcrumb,
    // which a free-run worker structurally never emits.
    assert_eq!(
        out.log.matches("worker left the barrier cohort").count(),
        2,
        "BOTH lockstep workers leave the cohort on clean exit; log was:\n{}",
        out.log
    );
    assert_eq!(
        out.log
            .matches("free-run deployment: no shared barrier is created")
            .count(),
        0,
        "a lockstep deployment creates the barrier; log was:\n{}",
        out.log
    );
    assert_eq!(
        out.log.matches("no barrier cohort to leave").count(),
        0,
        "no lockstep worker exits without a cohort; log was:\n{}",
        out.log
    );
}

/// Arm 3, NEGATIVE CONTROL: `--single-process` with nothing exported is the
/// monolith it always was. The default is keyed on the deployment fact, so a
/// route with no ranks shows none of it.
#[test]
#[serial]
fn a_single_process_run_is_untouched_by_the_flip() {
    let out = run_and_stop(
        "emmono",
        &["--single-process"],
        SpawnExecutionMode::Default,
        false,
    );
    // POSITIVE witnesses: the descriptor reports the monolith and the
    // read-only `wall` label a live monolith carries (a monolith mints no
    // trace ring).
    assert_eq!(
        out.run_json["partition"]["process_groups"],
        serde_json::json!(false),
        "--single-process executes no process groups: {}",
        out.run_json
    );
    assert_eq!(
        out.run_json["gating"],
        serde_json::json!("wall"),
        "a live monolith is the read-only RealClock arm: {}",
        out.run_json
    );
    // No workers, so no build line and no barrier breadcrumb of EITHER kind.
    for absent in [
        "worker build path resolved from the stamped execution mode",
        "free-run deployment: no shared barrier is created",
        "no barrier cohort to leave",
        "worker left the barrier cohort",
        "GO signaled; deployment live",
    ] {
        assert_eq!(
            out.log.matches(absent).count(),
            0,
            "a monolith must not log `{absent}`; log was:\n{}",
            out.log
        );
    }
}
