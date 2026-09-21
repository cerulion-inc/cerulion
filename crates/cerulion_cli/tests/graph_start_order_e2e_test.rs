// SPDX-License-Identifier: AGPL-3.0-only
//! Permanent regression pin — producer-graph-FIRST start order at the
//! TRUE production surface (two `cerulion graph run` PROCESSES over real
//! CLI-scaffolded cdylib nodes).
//!
//! # The contract
//!
//! The cross-step HOLD needs a snapshot CONSUMER to attach with iceoryx2
//! `subscriber_max_borrowed_samples >= 3`, but iceoryx2 bakes that cap in at
//! service CREATION (default 2) by whichever process creates the service
//! first. If a PRODUCER graph starting before its snapshot-consumer graph
//! created the shared topic at the default borrow 2, the consumer's open
//! would HARD-ABORT with
//! `PublishSubscribeOpenError::DoesNotSupportRequestedMinSubscriberBorrowedSamples`
//! — a real trap on robots, where processes restart in arbitrary order, and
//! one that only "always start the consumer first" would avoid. So
//! OWNED topics CREATE at the hold borrow
//! floor `max(genuine, SUBSCRIBER_MAX_BORROWED_HELD)`, and start order does not
//! matter. This file pins that contract end-to-end.
//!
//! # What this test does
//!
//! Scaffolds a workspace ENTIRELY through the real `cerulion` binary
//! (`env!("CARGO_BIN_EXE_cerulion")`):
//!   - `prod` — `period_ms=10`, one `geometry_msgs::Vector3` output — staged
//!     into graph `pgraph` (prefix `sop`), owning topic `/sop/prod/out`;
//!   - `cons` — `period_ms=10`, one PLAIN non-trigger `#[input]` (the cross-step
//!     hold path) — staged into graph `cgraph` (prefix `soc`), its input
//!     bound to the ABSOLUTE producer topic `/sop/prod/out`.
//!
//! Both node crates are compiled in-test via `cerulion node build`. Then it
//! starts `graph run pgraph` FIRST (the load-bearing ordering — without the
//! floor this creates `/sop/prod/out` at borrow 2), waits for the producer to go live
//! (its owned service now exists — created at the borrow FLOOR), starts
//! `graph run cgraph`, and asserts the consumer builds PAST the borrow-open and
//! RUNS: it reaches the "starting graph (live)" readiness line, stays alive
//! over a bounded window, and its logs never carry the
//! `DoesNotSupportRequestedMinSubscriberBorrowedSamples` regression signature.
//! Both processes then exit 0 on SIGINT (the clean-shutdown contract).
//!
//! Both runs pass `--single-process` so each is ONE monolith process — the
//! Unix default would otherwise auto-partition each graph into a
//! supervisor + worker, obscuring the two-process cross-graph scenario this pin
//! is actually about (and changing the readiness/teardown shape).
//!
//! # Anti-tautology
//!
//! The negative control was run by hand on Linux x86_64: a binary built WITH
//! the borrow floor PASSES this exact scenario, and one built WITHOUT it
//! ABORTS the consumer with `...DoesNotSupportRequestedMinSubscriberBorrowedSamples`.
//! That negative control is NOT reproducible in-tree — it needs a build
//! of the binary without the floor, which CI cannot produce from this
//! checkout — so it is recorded HERE rather than encoded as a second test arm.
//! The in-tree teeth are: (1) the signature assertion (the exact borrow-open
//! failure string, asserted absent), and (2) the scaffold-shape guard, which
//! fails LOUDLY if the CLI ever stops emitting a plain non-trigger `#[input]`
//! for `cons` — a trigger input would drain-not-hold, provision NO borrow-3
//! requirement, and make this whole test pass VACUOUSLY (green with or without
//! the borrow floor).
//!
//! # Qualified schema names
//!
//! Ports are added with the QUALIFIED schema name `geometry_msgs::Vector3`.
//! Bare (`Vector3`) names generate broken node-crate imports, so the
//! scaffold would fail to compile at the `node build` step.
//!
//! # Runtime cost (why this test is heavyweight)
//!
//! Unlike the sibling e2e tests (which COPY prebuilt fixture cdylibs), this
//! test compiles two node crates in-process via `cerulion node build`, because
//! the point is the TRUE production surface — real CLI-scaffolded nodes, not a
//! hand-copied fixture. The generated node crates path-dep the repo's
//! `cerulion_core`, so the FIRST build cold-compiles the cerulion_core family
//! (its dependencies come warm from `~/.cargo`); `mp_supervisor_box_test`
//! measures ~8 min for three such crates cold. The second build reuses
//! `<ws>/target` and is fast. Timeouts below are budgeted GENEROUSLY for a slow
//! CI runner; every wait is bounded (no unbounded waits, no exact-timing asserts —
//! liveness and the regression signature only).
//!
//! Runs on the GLOBAL iceoryx2 namespace (no isolation seam at the binary
//! layer, same as `node_run_e2e_test.rs` / `mp_record_e2e_test.rs`);
//! `#[serial]` + a unique prefix pair (`sop`/`soc`) + a unique tempdir keep
//! it safe.

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::{Duration, Instant};

use serial_test::serial;

/// The EXACT iceoryx2 open-error variant a hold consumer aborts with when its
/// topic was created below the borrow floor
/// (see `cerulion_core/src/transport/mod.rs`). This must NEVER appear in
/// the consumer's captured output — the owned topic is created at the
/// borrow floor, so the consumer's borrow-3 open succeeds.
const REGRESSION_SIGNATURE: &str = "DoesNotSupportRequestedMinSubscriberBorrowedSamples";

/// The live-loop readiness line `graph_cmd::graph_run` logs AFTER the runtime
/// (and hence every owned topic service) is built, right before entering the
/// live loop.
const READY_LINE: &str = "starting graph (live)";

// ── Timeout budget ──────────────────────────────────────────────────────────
// Every wait is bounded and generous; none asserts an exact timing.
/// Fast scaffold steps (workspace/node/graph file ops via the binary).
const SCAFFOLD_TIMEOUT: Duration = Duration::from_secs(60);
/// FIRST `node build` — cold-compiles the cerulion_core family into
/// `<ws>/target` (deps warm from `~/.cargo`). `mp_supervisor_box_test` measures
/// ~8 min for three such crates cold; ~2x headroom for a slow machine.
const FIRST_BUILD_TIMEOUT: Duration = Duration::from_secs(900);
/// SECOND `node build` — reuses `<ws>/target`, compiles only the node crate.
const SECOND_BUILD_TIMEOUT: Duration = Duration::from_secs(300);
/// Producer readiness (mirrors `node_run_e2e_test.rs`'s 30 s readiness wait).
const PRODUCER_READY_TIMEOUT: Duration = Duration::from_secs(30);
/// Consumer readiness — a touch more headroom than the producer (it opens the
/// external service the producer created).
const CONSUMER_READY_TIMEOUT: Duration = Duration::from_secs(40);
/// The consumer must keep running for this window after going live (proves it
/// does not crash shortly after building).
const LIVENESS_WINDOW: Duration = Duration::from_secs(6);
/// Clean-shutdown exit after SIGINT (mirrors `node_run_e2e_test.rs`'s 40 s).
const SHUTDOWN_TIMEOUT: Duration = Duration::from_secs(40);

/// SIGKILL + reap on drop so a panicking assert never leaks a child.
struct ChildGuard(Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// Poll `try_wait` until the child exits or `timeout` elapses. `Some(status)` =
/// exited; `None` = still running at the deadline.
fn wait_bounded(child: &mut Child, timeout: Duration) -> Option<ExitStatus> {
    let start = Instant::now();
    loop {
        match child.try_wait().expect("try_wait") {
            Some(status) => return Some(status),
            None if start.elapsed() > timeout => return None,
            None => std::thread::sleep(Duration::from_millis(50)),
        }
    }
}

/// `kill(2)` a real spawned child. Guarded against ever signalling pid <= 1
/// (never signal init / the whole process group).
fn send_signal(pid: u32, sig: libc::c_int) {
    assert!(pid > 1, "refusing to signal pid {pid} (<= 1)");
    // SAFETY: kill(2) with a valid pid + signal; no memory is touched.
    unsafe {
        libc::kill(pid as libc::pid_t, sig);
    }
}

fn read_file(path: &Path) -> String {
    std::fs::read_to_string(path).unwrap_or_default()
}

/// Bounded poll for `needle` in the (growing) log file at `path`; returns the
/// full content once seen, panics at the deadline with the log so far (the
/// caller's `ChildGuard` reaps on the panic unwind).
fn wait_for_log_line(path: &Path, needle: &str, timeout: Duration) -> String {
    let start = Instant::now();
    loop {
        let content = read_file(path);
        if content.contains(needle) {
            return content;
        }
        assert!(
            start.elapsed() < timeout,
            "log line {needle:?} not seen within {timeout:?}; log so far:\n{content}"
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Run one scaffold/build step through the REAL binary to completion in `root`,
/// asserting exit 0. Output is redirected to files so a build's flood of stdout
/// never deadlocks a pipe; on timeout or failure both are dumped.
///
/// `CARGO_TARGET_DIR` is stripped so every build lands in `<ws>/target` and the
/// later `graph run` resolves the cdylib from the same place (the
/// `node_run_e2e_test.rs` convention). `NO_COLOR` keeps captured output plain.
fn run_step(root: &Path, args: &[&str], timeout: Duration) {
    let out_path = root.join("step.stdout");
    let err_path = root.join("step.stderr");
    let mut guard = ChildGuard(
        Command::new(env!("CARGO_BIN_EXE_cerulion"))
            .args(args)
            .current_dir(root)
            .env_remove("CARGO_TARGET_DIR")
            .env("NO_COLOR", "1")
            .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
            .stdout(Stdio::from(std::fs::File::create(&out_path).unwrap()))
            .stderr(Stdio::from(std::fs::File::create(&err_path).unwrap()))
            .spawn()
            .unwrap_or_else(|e| panic!("failed to spawn `cerulion {}`: {e}", args.join(" "))),
    );
    match wait_bounded(&mut guard.0, timeout) {
        None => panic!(
            "`cerulion {}` did not finish within {timeout:?}\nstdout:\n{}\nstderr:\n{}",
            args.join(" "),
            read_file(&out_path),
            read_file(&err_path)
        ),
        Some(status) => assert!(
            status.success(),
            "`cerulion {}` must exit 0, got {status:?}\nstdout:\n{}\nstderr:\n{}",
            args.join(" "),
            read_file(&out_path),
            read_file(&err_path)
        ),
    }
}

/// Spawn `cerulion graph run <graph> --single-process` as a background child in
/// `root`, stdout+stderr to distinct files (readable while the child runs).
fn spawn_graph_run(root: &Path, graph: &str) -> (ChildGuard, PathBuf, PathBuf) {
    let stdout_path = root.join(format!("{graph}.stdout"));
    let stderr_path = root.join(format!("{graph}.stderr"));
    let child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["graph", "run", graph, "--single-process"])
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        // Hermetic — no scouting session/gateway in CI (a real-clock
        // run is permissive-by-default; the kill-switch env keeps it LOCAL-ONLY).
        .env("CERULION_NETWORK", "off")
        .env("NO_COLOR", "1")
        .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
        .stdout(Stdio::from(std::fs::File::create(&stdout_path).unwrap()))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()))
        .spawn()
        .unwrap_or_else(|e| panic!("failed to spawn `cerulion graph run {graph}`: {e}"));
    (ChildGuard(child), stdout_path, stderr_path)
}

/// THE pin: producer graph started FIRST, snapshot-hold consumer graph
/// starts and runs (without the borrow floor it aborts at the borrow-3 open).
#[test]
#[serial]
fn producer_first_then_snapshot_consumer_starts_and_runs() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();

    // ── Scaffold the workspace ENTIRELY through the real binary ──────────────
    // `workspace init .` in the (empty) tempdir; every later Node/Graph command
    // runs with cwd = root and is resolved by `discover_workspace()`.
    run_step(root, &["workspace", "init", "."], SCAFFOLD_TIMEOUT);

    // Producer: a source-only period node, then add its Vector3 output.
    run_step(
        root,
        &["node", "create", "prod", "--policy", "period_ms=10"],
        SCAFFOLD_TIMEOUT,
    );
    run_step(
        root,
        &[
            "node",
            "modify",
            "prod",
            "-o",
            "geometry_msgs::Vector3",
            "out",
        ],
        SCAFFOLD_TIMEOUT,
    );

    // Consumer: a period node, then add a PLAIN non-trigger input (the hold
    // path). `-i` (not `-T`) yields `#[input]`, NOT `#[input(trigger)]`.
    run_step(
        root,
        &["node", "create", "cons", "--policy", "period_ms=10"],
        SCAFFOLD_TIMEOUT,
    );
    run_step(
        root,
        &[
            "node",
            "modify",
            "cons",
            "-i",
            "geometry_msgs::Vector3",
            "inp",
        ],
        SCAFFOLD_TIMEOUT,
    );

    // ── Scaffold-shape guard (anti-tautology) ────────────────────────────────
    // The whole pin rests on `cons` being a HOLD consumer: a plain non-trigger
    // `#[input]` makes `/sop/prod/out` a snapshot-source topic → the consumer
    // opens it requiring borrow 3. If the CLI scaffold ever emits a trigger
    // input instead, that requirement vanishes and the test would pass
    // vacuously — so fail LOUDLY here. (`#[input]` is NOT a substring of
    // `#[input(trigger)]`, so these two checks are mutually distinguishing.)
    let cons_src = read_file(&root.join("nodes/cons/src/lib.rs"));
    assert!(
        cons_src.contains("#[input]"),
        "scaffold-shape guard: `cons` must carry a PLAIN non-trigger `#[input]` (the cross-step \
         hold path that provisions borrow-3); generated lib.rs was:\n{cons_src}"
    );
    assert!(
        !cons_src.contains("#[input(trigger)]"),
        "scaffold-shape guard: `cons` must NOT carry `#[input(trigger)]` — a trigger input \
         drains (not holds), provisions NO borrow-3 requirement, and makes this start-order pin \
         pass VACUOUSLY; generated lib.rs was:\n{cons_src}"
    );

    // ── Graphs + staging ─────────────────────────────────────────────────────
    run_step(
        root,
        &["graph", "create", "pgraph", "-n", "sop"],
        SCAFFOLD_TIMEOUT,
    );
    run_step(
        root,
        &["graph", "create", "cgraph", "-n", "soc"],
        SCAFFOLD_TIMEOUT,
    );
    run_step(
        root,
        &["node", "stage", "prod", "-g", "pgraph"],
        SCAFFOLD_TIMEOUT,
    );
    // `cons.inp` binds to the ABSOLUTE producer topic — the cross-graph
    // (cross-process) source that makes the two graphs share one iceoryx2
    // service, which is where the borrow-floor asymmetry lives.
    run_step(
        root,
        &[
            "node",
            "stage",
            "cons",
            "-g",
            "cgraph",
            "-I",
            "inp",
            "/sop/prod/out",
        ],
        SCAFFOLD_TIMEOUT,
    );

    // ── In-test node builds (the heavyweight step; see module doc) ───────────
    // prod first cold-compiles the cerulion_core family into <ws>/target; cons
    // then reuses it.
    run_step(root, &["node", "build", "prod"], FIRST_BUILD_TIMEOUT);
    run_step(root, &["node", "build", "cons"], SECOND_BUILD_TIMEOUT);

    // ── Start the PRODUCER graph FIRST (the load-bearing ordering) ───────────
    let (mut producer, _prod_stdout, prod_stderr) = spawn_graph_run(root, "pgraph");
    // Wait for the producer to go live — its owned `/sop/prod/out` service is
    // now created (at the borrow FLOOR), so the consumer that follows
    // opens an already-existing service (deterministic ordering, no race). The
    // READY_LINE is a `tracing::info!`, which goes to STDERR (init_logging
    // writes there so a command's stdout stays clean data).
    wait_for_log_line(&prod_stderr, READY_LINE, PRODUCER_READY_TIMEOUT);

    // ── Now start the CONSUMER graph ─────────────────────────────────────────
    let (mut consumer, cons_stdout, cons_stderr) = spawn_graph_run(root, "cgraph");

    // Positive proof it built PAST the borrow-open: it reaches the live-loop
    // readiness line. Without the floor the consumer aborts at build with the borrow
    // error and never logs this — on timeout we dump its stderr (which carries
    // the abort) so the failure is legible.
    {
        let start = Instant::now();
        loop {
            // READY_LINE is a `tracing::info!` → STDERR (see the producer wait
            // above); poll the consumer's stderr for it.
            if read_file(&cons_stderr).contains(READY_LINE) {
                break;
            }
            // A borrow-open abort exits fast; surface it immediately rather than
            // waiting out the whole readiness budget.
            if let Some(status) = consumer.0.try_wait().expect("try_wait consumer") {
                panic!(
                    "consumer graph exited before going live ({status:?}) — start-order regression? \
                     it must build past the borrow-3 open and run.\nstdout:\n{}\nstderr:\n{}",
                    read_file(&cons_stdout),
                    read_file(&cons_stderr)
                );
            }
            assert!(
                start.elapsed() < CONSUMER_READY_TIMEOUT,
                "consumer graph never reached {READY_LINE:?} within {CONSUMER_READY_TIMEOUT:?}\n\
                 stdout:\n{}\nstderr:\n{}",
                read_file(&cons_stdout),
                read_file(&cons_stderr)
            );
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    // Liveness: it must keep running (not crash shortly after going live).
    if let Some(status) = wait_bounded(&mut consumer.0, LIVENESS_WINDOW) {
        panic!(
            "consumer graph exited early ({status:?}) after going live — it must start AND run.\n\
             stdout:\n{}\nstderr:\n{}",
            read_file(&cons_stdout),
            read_file(&cons_stderr)
        );
    }

    // Signature: assert on the EXACT borrow-open failure string, not just liveness.
    let cons_out = read_file(&cons_stdout);
    let cons_err = read_file(&cons_stderr);
    assert!(
        !cons_out.contains(REGRESSION_SIGNATURE) && !cons_err.contains(REGRESSION_SIGNATURE),
        "start-order regression: the consumer's output carries {REGRESSION_SIGNATURE:?} — the \
         producer-first start order aborted the snapshot consumer's borrow-3 open.\n\
         stdout:\n{cons_out}\nstderr:\n{cons_err}"
    );

    // ── Clean shutdown: SIGINT both (reverse of start order); both exit 0 ────
    send_signal(consumer.0.id(), libc::SIGINT);
    let cons_status = wait_bounded(&mut consumer.0, SHUTDOWN_TIMEOUT)
        .expect("consumer graph did not exit after SIGINT");
    assert!(
        cons_status.success(),
        "consumer graph must exit 0 on Ctrl-C, got {cons_status:?}\nstderr:\n{}",
        read_file(&cons_stderr)
    );

    send_signal(producer.0.id(), libc::SIGINT);
    let prod_status = wait_bounded(&mut producer.0, SHUTDOWN_TIMEOUT)
        .expect("producer graph did not exit after SIGINT");
    assert!(
        prod_status.success(),
        "producer graph must exit 0 on Ctrl-C, got {prod_status:?}\nstderr:\n{}",
        read_file(&prod_stderr)
    );
}
