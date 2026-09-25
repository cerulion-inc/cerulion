// SPDX-License-Identifier: AGPL-3.0-only
//! Opt-in (`#[ignore]`d) end-to-end test of the multi-process WORKER entry
//! [`graph_cmd::graph_run_worker`](cerulion_cli_engine::graph_cmd::graph_run_worker).
//!
//! A minimal test-SUPERVISOR reproduces what the real supervisor does
//! for ONE worker: it scaffolds a workspace with a single pure-period source node
//! ("ticker", `period_ms = 1`) + builds its cdylib, mints ONE shared iceoryx2
//! `Config`, creates the shared cross-process barrier, writes a [`WorkerPlan`]
//! (1-node subgraph, the minted `Config` in `ix_config_json`), then self-re-execs
//! THIS test binary into the `worker_entrypoint` `#[test]` (env-switched) which
//! calls `graph_run_worker`. The worker inits the shared transport, OPENS the
//! barrier, builds the deterministic-live runtime as ONE barrier-gated context,
//! signals READY, and runs the live loop. A watchdog (the test's stand-in for
//! Ctrl+C) stops the loop after `WORKER_STOP_MS`.
//!
//! Two cases:
//!  * **happy** — barrier `expected = 1`; the lone worker rendezvouses solo at
//!    every level boundary, so its period node fires and the shared barrier
//!    generation ADVANCES. The worker exits 0; the supervisor asserts the owner's
//!    generation advanced.
//!  * **poison** — barrier `expected = 2` but only ONE worker spawns; its first
//!    barrier boundary never completes → times out (~5s) → the runtime TERMINALLY
//!    poisons → `graph_run_worker` exits(2). Proves the barrier GATES (blocks)
//!    level advance, not merely counts (Principle #3 — a stalled peer is a LOUD,
//!    distinguishable failure).
//!
//! ## Scope + how to run
//!
//! This file COMPILES on all platforms, but the two supervisor tests are
//! `#[ignore]` + `#[serial]` and internally guard
//! `if !cfg!(unix) { return; }` (the guard is Unix, not Linux:
//! `MappedBarrier` is real POSIX `shm_open` + `MAP_SHARED` on every Unix,
//! so a re-exec'd worker rendezvouses on macOS too). Run on Linux
//! (x86 with WAITPKG / aarch64 with WFE) or on macOS:
//!
//! ```bash
//! cargo test -p cerulion_cli_engine --test mp_worker_box_test -- --ignored --nocapture
//! ```
//!
//! Hang/orphan safety is mandatory: `ChildGuard::Drop` SIGKILLs + reaps, every
//! wait is HARD-bounded, and a poisoned worker exits non-zero (caught by the
//! supervisor's `.success()` assert). `#[serial]` (iceoryx2 SHM singleton + the
//! barrier registry; the barrier namespace is pid+tag-scoped so concurrent
//! binaries never collide).
//!
//! No fake data (Principle #13): the ticker is a REAL `#[cerulion_node]` cdylib
//! over a REAL iceoryx2 SHM region; the barrier is a REAL cross-process segment.

use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_cli_engine::multiprocess::WorkerPlan;
use cerulion_cli_engine::node_cmd::NodeCreateOptions;
use cerulion_cli_engine::{graph_cmd, node_cmd, workspace};
use cerulion_core::barrier::MappedBarrier;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use serial_test::serial;

// Env vars the supervisor sets to drive the re-exec'd worker child. ALL UNSET in
// a normal `cargo test` run → `worker_entrypoint` is a harmless no-op pass.

/// Path to the serialized `WorkerPlan` JSON the worker reads.
const ENV_PLAN: &str = "WORKER_PLAN";
/// The workspace root the worker resolves the subgraph's node cdylibs against.
const ENV_WS: &str = "WORKER_WS";
/// Milliseconds after which the worker's watchdog flips `running` false (the
/// test's stand-in for Ctrl+C, since a re-exec'd child has no interactive SIGINT).
const ENV_STOP_MS: &str = "WORKER_STOP_MS";

/// When set on the re-exec'd worker, install the SIGINT→`running`
/// bridge (see [`install_sigint_bridge`]) so the SIGINT-drain arm exercises
/// the production ctrlc drain path instead of a default signal death.
const ENV_SIGINT: &str = "WORKER_SIGINT";

/// The shared barrier id (within the pid+tag-scoped namespace). The supervisor
/// `create_owned`s it; the worker's `graph_run_worker` `open_unowned`s it.
const BARRIER_ID: &str = "worker_gate";

// ===========================================================================
// The WORKER child entrypoint — re-invoked as a subprocess via `current_exe`.
// ===========================================================================

/// When `WORKER_PLAN` is UNSET this is a harmless no-op pass (so a normal
/// `cargo test` run is not disturbed). When SET (the supervisor re-invokes THIS
/// binary with `--exact worker_entrypoint` + the env), it drives the worker to
/// completion via `graph_run_worker`:
///  * on a clean run it returns Ok → this test returns → libtest exits 0;
///  * on a barrier poison `graph_run_worker` calls `std::process::exit(2)` itself;
///  * on any OTHER error it panics → libtest exits non-zero.
#[test]
fn worker_entrypoint() {
    let plan = match std::env::var(ENV_PLAN) {
        Ok(p) => p,
        // Not the child invocation — normal `cargo test` run. No-op pass.
        Err(_) => return,
    };
    let ws = std::env::var(ENV_WS).expect("WORKER_WS must be set alongside the plan");
    let stop_ms: u64 = std::env::var(ENV_STOP_MS)
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(8000);

    // Watchdog: flip `running` false after `stop_ms` so the worker's `run_live`
    // returns (a re-exec'd child has no Ctrl+C). On the happy path this is well
    // after the barrier has advanced; on the poison path it is past the ~5s
    // barrier boundary timeout, so the runtime has already poisoned and
    // `graph_run_worker` exits(2) after the loop returns.
    let running = Arc::new(AtomicBool::new(true));
    let running_wd = Arc::clone(&running);
    std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(stop_ms));
        running_wd.store(false, Ordering::Relaxed);
    });

    // SIGINT-drain arm: opt-in mirror of production's
    // `setup_ctrlc_handler` (libtest installs NO SIGINT handler, so a directed
    // SIGINT would DEFAULT-TERMINATE this child and the drain pin would pass
    // vacuously as a signal death instead of exercising the worker's
    // `running`-flag drain path).
    #[cfg(unix)]
    if std::env::var(ENV_SIGINT).is_ok() {
        install_sigint_bridge(Arc::clone(&running));
    }

    graph_cmd::graph_run_worker(Path::new(&ws), Path::new(&plan), running)
        .expect("graph_run_worker returned an unexpected error");
}

/// Bridge a directed SIGINT to the worker's `running` flag, mirroring
/// production `setup_ctrlc_handler` semantics (a pure `store(false)` — the
/// ctrlc crate does exactly this on its handler thread). The `extern "C"`
/// handler performs ONLY an atomic store (async-signal-safe); a watcher thread
/// polls the static at 5ms and flips the worker's `running` — the same
/// observable contract as production, with ≤5ms bridge latency (noise next to
/// the drain window).
#[cfg(unix)]
fn install_sigint_bridge(running: Arc<AtomicBool>) {
    static SIGINT_HIT: AtomicBool = AtomicBool::new(false);
    extern "C" fn on_sigint(_sig: libc::c_int) {
        SIGINT_HIT.store(true, Ordering::Relaxed);
    }
    // Bind the fn ITEM to an explicit fn POINTER first (clippy
    // `fn_to_numeric_cast_any` rejects a direct item→integer cast); the
    // pointer→`sighandler_t` (usize) cast is the documented libc::signal idiom.
    let handler: extern "C" fn(libc::c_int) = on_sigint;
    // SAFETY: installing a SIGINT handler whose body is a single atomic store
    // (async-signal-safe).
    unsafe {
        libc::signal(libc::SIGINT, handler as libc::sighandler_t);
    }
    std::thread::spawn(move || loop {
        if SIGINT_HIT.load(Ordering::Relaxed) {
            running.store(false, Ordering::Relaxed);
            return;
        }
        std::thread::sleep(Duration::from_millis(5));
    });
}

// ===========================================================================
// Supervisor helpers.
// ===========================================================================

/// Scaffold a minimal workspace with ONE pure-period source node ("ticker",
/// `period_ms = 1`, one `geometry_msgs/Vector3` output, no inputs) and build its
/// cdylib. The node fires purely on the gating clock, so the worker advances the
/// barrier without needing any external publisher. Returns the (kept-alive)
/// `TempDir` + the workspace root.
fn build_ticker_workspace() -> (tempfile::TempDir, PathBuf) {
    let tmp = tempfile::tempdir().expect("temp dir for worker workspace");
    let ws = workspace::workspace_create(tmp.path(), "ws").expect("workspace_create");

    let opts = NodeCreateOptions {
        outputs: vec![("geometry_msgs/Vector3".to_string(), "out".to_string())],
        inputs: Vec::new(),
        trigger: None,
        raw_ffi: false,
    };
    node_cmd::node_create_with_options(
        &ws.nodes_dir,
        &ws.root.join("Cargo.toml"),
        "ticker",
        Some(cerulion_core::MacroPolicy::Period { period_ms: 1 }),
        &opts,
    )
    .expect("node_create ticker");

    node_cmd::node_build(&ws.root, "ticker", false).expect("node_build ticker");
    (tmp, ws.root)
}

/// Write a 1-node WorkerPlan (the "ticker" subgraph) with the given barrier ns +
/// READY path + minted iceoryx2 Config JSON, returning the plan file path.
fn write_worker_plan(dir: &Path, ns: &str, ready: &Path, ix_config_json: String) -> PathBuf {
    let subgraph = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        name: None,
        identity: "ws_solo".to_string(),
        prefix: "worker".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "ticker".to_string(),
            node_type: "ticker".to_string(),
            inputs: Vec::new(),
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "geometry_msgs/Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
        multi_publisher_topics: Vec::new(),
        // `IndexMap: Default` — avoids naming `indexmap` (a normal dep, not a
        // dev-dep, so not directly usable from this integration-test crate).
        process_groups: Default::default(),
        process_group_order: Vec::new(),
    };
    // GO gate: this hand-written test-SUPERVISOR pre-signals GO before the worker
    // even spawns — a SINGLE-worker deployment has no siblings to wait for, and
    // both tests here pin worker behavior AFTER the start gate (happy live-loop
    // advance / barrier poison). The real supervisor writes GO only once ALL
    // workers are READY; that ordering is `mp_supervisor_box_test`'s concern.
    let go = dir.join("worker.go");
    std::fs::write(&go, b"go").expect("pre-signal GO sentinel");
    let plan = WorkerPlan {
        group: "solo".to_string(),
        graph_identity: "boxwork_solo".to_string(),
        rank: 0,
        global_level_map: vec![Some(0)],
        // One global level, no split same-level non-trigger pair —
        // so no mid-level rendezvous and exactly one barrier generation per
        // step, the one-generation-per-step law this fixture's generation asserts read.
        mid_level_barrier: vec![false],
        handed_quantum_ns: 1_000_000,
        node_name: "cerulion_worker".to_string(),
        barrier_ns: ns.to_string(),
        barrier_id: BARRIER_ID.to_string(),
        ready_path: ready.display().to_string(),
        go_path: go.display().to_string(),
        // Pre-signaled GO (above) makes the deadline moot; use the planner's
        // base for realism.
        go_deadline_ms: cerulion_cli_engine::multiprocess::GO_BASE_MS,
        subgraph,
        ix_config_json,
        // Debug-build test fixture: the ticker cdylib is built with a plain
        // `cargo build`, so the debug/release freshest-wins default is correct.
        prefer_release: false,
        // The production default cap (the supervisor stamps
        // `--trace-limit` here; this harness plays the supervisor).
        trace_limit: cerulion_cli_engine::graph_cmd::PRODUCTION_TRACE_LIMIT,
        // Park off in this harness (the latency bench validates the park).
        monitor_wait: false,
        doorbell: false,
        // Cap Auto in this harness.
        cap_disabled: false,
        // The shipped barrier-lockstep worker: this harness pins
        // the barrier path (happy advance / poison), so it must stay on it.
        execution_mode: cerulion_cli_engine::multiprocess::ExecutionMode::Lockstep,
        // No cross-group provisioning union in this single-worker
        // harness (nothing to union — the supervisor stamps this in production).
        topic_requirements: std::collections::BTreeMap::new(),
        sibling_topics: std::collections::BTreeSet::new(),
        // No credit-backed cross-process `block` edge.
        credit_edges: Vec::new(),
        // Not recording in this harness (the supervisor stamps the
        // ring tag in production; the worker recording seam has its own pins).
        recording_ring: None,
        trace_ring: None,
        state_arm_tag: None,
        run_dir: None,
        wedge_page: None,
    };
    let path = dir.join("worker_plan.json");
    std::fs::write(
        &path,
        serde_json::to_string(&plan).expect("serialize WorkerPlan"),
    )
    .expect("write worker plan");
    path
}

/// Spawn the worker by re-invoking THIS test binary at `--exact worker_entrypoint`
/// with cwd = the workspace root (so cdylib resolution + workspace discovery
/// work) and the plan / ws / stop-ms env set. `sigint_bridge` additionally sets
/// [`ENV_SIGINT`] so the child mirrors production's ctrlc handler (the
/// SIGINT-drain arms).
fn spawn_worker_opts(
    ws_root: &Path,
    plan_path: &Path,
    stop_ms: u64,
    sigint_bridge: bool,
) -> std::process::Child {
    let exe = std::env::current_exe().expect("current_exe");
    let mut cmd = Command::new(exe);
    cmd.args([
        "--exact",
        "worker_entrypoint",
        "--test-threads=1",
        "--nocapture",
    ])
    .env(ENV_PLAN, plan_path)
    .env(ENV_WS, ws_root)
    .env(ENV_STOP_MS, stop_ms.to_string())
    .current_dir(ws_root)
    // Inherit stdio so a worker panic / failure is visible under --nocapture; the
    // supervisor reads the barrier generation + exit status, not the output.
    .stdout(std::process::Stdio::inherit())
    .stderr(std::process::Stdio::inherit());
    if sigint_bridge {
        cmd.env(ENV_SIGINT, "1");
    }
    cmd.spawn().expect("spawn worker child")
}

/// The original two-arg spawn (no SIGINT bridge) — see [`spawn_worker_opts`].
fn spawn_worker(ws_root: &Path, plan_path: &Path, stop_ms: u64) -> std::process::Child {
    spawn_worker_opts(ws_root, plan_path, stop_ms, false)
}

/// pid+tag-scoped barrier namespace so the two `#[serial]` supervisor tests (and
/// concurrent binaries / re-runs) never collide on a `/dev/shm` object.
fn barrier_ns(tag: &str) -> String {
    format!("worker_{}_{tag}", std::process::id())
}

/// BOUNDED wait for `path` to appear (the worker's READY sentinel). Fail-fasts
/// (returns false) at the deadline instead of hanging.
fn wait_for_file(path: &Path, deadline: Duration) -> bool {
    let start = Instant::now();
    while start.elapsed() < deadline {
        if path.exists() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    path.exists()
}

// ===========================================================================
// RAII child guard — Drop kills + reaps so a supervisor panic never orphans.
// (Mirrors `barrier_level_gate_subprocess_iox2_test.rs`.)
// ===========================================================================

struct ChildGuard {
    child: std::process::Child,
    reaped: bool,
}

impl ChildGuard {
    fn new(child: std::process::Child) -> Self {
        Self {
            child,
            reaped: false,
        }
    }

    /// BOUNDED wait for the child to exit: returns its `ExitStatus`, or `None` if
    /// the deadline elapsed (in which case the child is SIGKILLed + reaped — a
    /// hung child must NEVER hang a test run). Idempotent.
    fn wait_bounded(&mut self, deadline: Duration) -> Option<std::process::ExitStatus> {
        if self.reaped {
            return None;
        }
        let start = Instant::now();
        loop {
            match self.child.try_wait() {
                Ok(Some(status)) => {
                    self.reaped = true;
                    return Some(status);
                }
                Ok(None) => {
                    if start.elapsed() >= deadline {
                        let _ = self.child.kill();
                        let _ = self.child.wait();
                        self.reaped = true;
                        return None;
                    }
                    std::thread::sleep(Duration::from_millis(20));
                }
                Err(e) => {
                    eprintln!("[worker supervisor] try_wait on child errored: {e}");
                    let _ = self.child.kill();
                    let _ = self.child.wait();
                    self.reaped = true;
                    return None;
                }
            }
        }
    }
}

impl Drop for ChildGuard {
    fn drop(&mut self) {
        if self.reaped {
            return;
        }
        let _ = self.child.kill();
        let _ = self.child.wait();
        self.reaped = true;
    }
}

// ===========================================================================
// THE SUPERVISOR TESTS. `#[ignore]` (opt-in — needs real MAP_SHARED + a built
// cdylib) + `#[serial]`. Run with:
//   cargo test -p cerulion_cli_engine --test mp_worker_box_test -- --ignored --nocapture
// ===========================================================================

/// HAPPY: a single barrier-gated worker (`expected = 1`) advances the shared
/// barrier generation and exits 0.
#[test]
#[ignore]
#[serial]
fn box_single_worker_advances_barrier_and_exits_clean() {
    if !cfg!(unix) {
        // Cross-process MAP_SHARED barrier is Unix-only (POSIX shm_open;
        // not Linux-only: macOS real-maps too).
        return;
    }

    let (ws_tmp, ws_root) = build_ticker_workspace();

    // Mint ONE shared iceoryx2 config; the worker deserializes the SAME namespace.
    let ix = cerulion_core::testing::iceoryx_test_config();
    let ix_json = serde_json::to_string(&ix).expect("serialize iceoryx2 config");

    // The supervisor OWNS the barrier (expected = 1) — it does NOT participate; it
    // reads the shared generation after the worker exits. The single worker is the
    // one participant, so each boundary opens on its solo rendezvous.
    let ns = barrier_ns("happy");
    let owner = MappedBarrier::create_owned(&ns, BARRIER_ID, 1).expect("barrier owner create");

    let run_tmp = tempfile::tempdir().expect("temp dir for plan + ready");
    let ready = run_tmp.path().join("worker.ready");
    let plan_path = write_worker_plan(run_tmp.path(), &ns, &ready, ix_json);

    // Stop after 3s — the happy path advances the barrier within milliseconds, so
    // 3s is ample (and the generation is read AFTER the worker exits).
    let child = spawn_worker(&ws_root, &plan_path, 3000);
    let mut guard = ChildGuard::new(child);

    assert!(
        wait_for_file(&ready, Duration::from_secs(30)),
        "worker never signalled READY within 30s — its transport init / cdylib load likely \
         failed (see its stderr above). This is a fail-fast, not a hang."
    );

    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .expect("worker did not exit within 60s (killed) — likely stuck in the live loop");
    assert!(
        status.success(),
        "worker exited non-zero ({status:?}) — it should run cleanly then exit 0 (see stderr above)"
    );
    // This proves the worker's barrier-gated live loop RAN and advanced through
    // MANY global level boundaries (the runtime built, the level executor + gating
    // clock ran under the SHARED barrier, and the worker exited 0). It does NOT by
    // itself prove the ticker's `tick` fired — an EMPTY level still rendezvouses the
    // barrier — so this pins the worker MECHANISM (init → barrier → build → run_live
    // → clean exit), not node-fire/data-flow. Cross-process node-fire + data-flow is
    // pinned by the supervisor's trace-merge test (the merged fire SEQUENCE == the
    // monolith oracle). `> 5` (not `> 0`) proves sustained advancement and excludes an
    // advance-once-then-stall regression.
    let gen = owner.current_generation();
    assert!(
        gen > 5,
        "the shared barrier generation must have advanced through many boundaries — the worker's \
         barrier-gated live loop ran under the shared MAP_SHARED barrier (got generation {gen})"
    );

    drop(guard);
    drop(run_tmp);
    drop(ws_tmp);
    drop(owner);
}

/// POISON (anti-tautology): a lone worker against an `expected = 2` barrier BLOCKS
/// at its first boundary → times out → poisons → exits non-zero. Proves the
/// barrier GATES (blocks) level advance, not merely counts — the same guarantee
/// that makes the happy path non-tautological.
#[test]
#[ignore]
#[serial]
fn box_lone_worker_times_out_poisons_and_exits_nonzero() {
    if !cfg!(unix) {
        return;
    }

    let (ws_tmp, ws_root) = build_ticker_workspace();
    let ix = cerulion_core::testing::iceoryx_test_config();
    let ix_json = serde_json::to_string(&ix).expect("serialize iceoryx2 config");

    // expected = 2, but we spawn ONLY ONE worker → its first barrier boundary
    // never completes → times out (~5s) → the runtime TERMINALLY poisons →
    // graph_run_worker exits(2) once the watchdog stops the loop.
    let ns = barrier_ns("poison");
    let owner = MappedBarrier::create_owned(&ns, BARRIER_ID, 2).expect("barrier owner create");

    let run_tmp = tempfile::tempdir().expect("temp dir for plan + ready");
    let ready = run_tmp.path().join("worker.ready");
    let plan_path = write_worker_plan(run_tmp.path(), &ns, &ready, ix_json);

    // Watchdog set FAR out (30s) so it is NOT the exit trigger:
    // the worker's OWN barrier-boundary timeout (~5s) poisons the
    // runtime, `run_live` BREAKS on `barrier_failed`, and `graph_run_worker`
    // exits(2) — all without the watchdog. This is the regression guard for a
    // dead `exit(2)` (a poisoned `run_live` that idle-spins forever makes
    // `exit(2)` unreachable): if that regresses the worker never self-exits
    // and `wait_bounded(15s)` below SIGKILLs it at the deadline (returns `None`),
    // so the `.expect` fails the test.
    let child = spawn_worker(&ws_root, &plan_path, 30_000);
    let mut guard = ChildGuard::new(child);

    assert!(
        wait_for_file(&ready, Duration::from_secs(30)),
        "lone worker never signalled READY within 30s — build / transport init likely failed."
    );

    // 15s deadline: > the ~5s poison, << the 30s watchdog. A `Some` means the
    // worker exited ON ITS OWN (the barrier poison broke `run_live`), NOT that we
    // killed it at the deadline.
    let status = guard.wait_bounded(Duration::from_secs(15)).expect(
        "lone worker did not SELF-exit within 15s — with a 30s watchdog this means `run_live` did \
         NOT break on the barrier poison (the `exit(2)` path is dead → a silent idle-spin zombie). \
         This is the dead-`exit(2)` regression.",
    );
    assert!(
        !status.success(),
        "lone worker MUST exit non-zero when its barrier peer never arrives ({status:?}) — the \
         poisoned wait proves the shared MAP_SHARED barrier GATES (blocks) level advance, AND that \
         `run_live` self-exits on the poison (no watchdog needed). A success here means the barrier \
         degraded to count-without-block, or the poison was masked."
    );

    drop(guard);
    drop(run_tmp);
    drop(ws_tmp);
    drop(owner);
}

/// SIGINT-drain regression pin (arm 1 — FREE-RUNNING worker): a
/// worker whose live loop is advancing (solo `expected = 1` barrier) receives
/// a DIRECTED SIGINT — `libc::kill(pid, SIGINT)` to the worker pid, exactly
/// what the supervisor's Ctrl-C fan-out sends and exactly the delivery
/// mode a macOS `kill -INT <supervisor-pid>` bench script exercises (no
/// process-group signal anywhere) — and must exit CLEANLY (code 0, the
/// `leave_barrier_cohort` drain path) well inside the 20s drain window.
///
/// On macOS, a supervisor whose Ctrl-C arm sends nothing (betting on
/// process-group delivery) leaves the workers with NO signal on Ctrl-C,
/// so they run to the drain-deadline SIGKILL. This arm pins the worker HALF:
/// a directed SIGINT drains a live worker cleanly. The child mirrors
/// production's ctrlc handler via [`install_sigint_bridge`] (asserting
/// `success()` also guards the bridge: an uninstalled handler would make the
/// SIGINT a default signal DEATH — `code() == None`, not 0 — failing loudly).
#[cfg(unix)]
#[test]
#[ignore]
#[serial]
fn box_running_worker_directed_sigint_drains_clean_within_window() {
    let (ws_tmp, ws_root) = build_ticker_workspace();
    let ix = cerulion_core::testing::iceoryx_test_config();
    let ix_json = serde_json::to_string(&ix).expect("serialize iceoryx2 config");

    // Solo cohort: the worker free-runs, advancing the barrier each boundary.
    let ns = barrier_ns("sigint_run");
    let owner = MappedBarrier::create_owned(&ns, BARRIER_ID, 1).expect("barrier owner create");

    let run_tmp = tempfile::tempdir().expect("temp dir for plan + ready");
    let ready = run_tmp.path().join("worker.ready");
    let plan_path = write_worker_plan(run_tmp.path(), &ns, &ready, ix_json);

    // Watchdog FAR out (60s): the directed SIGINT must be the ONLY stop
    // trigger inside this test's bounded windows — if the SIGINT drain path
    // regresses, the wait below SIGKILLs at its deadline (None) and the
    // `.expect` fails, exactly the observed macOS field failure shape.
    let child = spawn_worker_opts(&ws_root, &plan_path, 60_000, true);
    let pid = child.id();
    let mut guard = ChildGuard::new(child);

    assert!(
        wait_for_file(&ready, Duration::from_secs(30)),
        "worker never signalled READY within 30s — build / transport init likely failed."
    );
    // Let the live loop genuinely run (the barrier generation advances).
    std::thread::sleep(Duration::from_millis(800));

    // THE fixed delivery: a directed SIGINT to the worker pid.
    // SAFETY: FFI kill of our own spawned child with the graceful-shutdown
    // signal its ctrlc-mirroring bridge handles.
    unsafe {
        libc::kill(pid as libc::pid_t, libc::SIGINT);
    }

    // 10s deadline: generous for the drain (flag flip → current step/park
    // completes ≤ ~5s worst case → leave cohort → trace/exit), yet HALF the
    // production 20s drain window — a pass here proves the supervisor's
    // straggler SIGKILL is never needed for a signaled worker.
    let status = guard.wait_bounded(Duration::from_secs(10)).expect(
        "worker did not exit within 10s of a directed SIGINT — the SIGINT drain regression \
         (worker never drains without process-group delivery; it would have hit the supervisor's \
         20s straggler SIGKILL)",
    );
    assert!(
        status.success(),
        "a SIGINTed free-running worker must drain CLEANLY (exit 0 via running=false → run_live \
         return → leave_barrier_cohort); got {status:?} — a signal death (code None) means the \
         ctrlc-mirroring bridge was not exercised, a code 2 means it poisoned instead of draining"
    );
    assert!(
        owner.current_generation() > 0,
        "the live loop must have genuinely advanced the barrier before the drain \
         (generation = {}) — otherwise this pin never exercised a RUNNING worker",
        owner.current_generation()
    );

    drop(guard);
    drop(run_tmp);
    drop(ws_tmp);
    drop(owner);
}

/// SIGINT-drain regression pin (arm 2 — PARKED worker): a worker
/// PARKED at a barrier boundary (`expected = 2`, no peer ever arrives — on
/// macOS the park is the boundary spin + chunked ~100µs sleep-recheck ladder)
/// receives the same directed SIGINT and must still EXIT BOUNDED, well inside
/// the 20s drain window: the park is hard-bounded by the ~5s
/// `BARRIER_BOUNDARY_TIMEOUT`, which poisons the runtime → `run_live` breaks →
/// `graph_run_worker` exits(2). The drain window tolerates any exit status —
/// the contract pinned here is BOUNDED EXIT, never a sleep-through-the-window
/// wedge (the coordinator's attribution question 2: can a parked worker sleep
/// through the drain? No — and this arm keeps it that way).
///
/// `code() == Some(2)` is asserted exactly: a signal DEATH (code None) would
/// mean the ctrlc bridge was not installed (vacuous pass); a 0 would mean the
/// poison was masked as a clean drain.
#[cfg(unix)]
#[test]
#[ignore]
#[serial]
fn box_parked_worker_directed_sigint_exits_bounded() {
    let (ws_tmp, ws_root) = build_ticker_workspace();
    let ix = cerulion_core::testing::iceoryx_test_config();
    let ix_json = serde_json::to_string(&ix).expect("serialize iceoryx2 config");

    // expected = 2 with only ONE worker → it parks at its FIRST boundary.
    let ns = barrier_ns("sigint_park");
    let owner = MappedBarrier::create_owned(&ns, BARRIER_ID, 2).expect("barrier owner create");

    let run_tmp = tempfile::tempdir().expect("temp dir for plan + ready");
    let ready = run_tmp.path().join("worker.ready");
    let plan_path = write_worker_plan(run_tmp.path(), &ns, &ready, ix_json);

    // Watchdog FAR out (60s) — the SIGINT + the bounded boundary timeout must
    // be what ends the worker, never the watchdog.
    let child = spawn_worker_opts(&ws_root, &plan_path, 60_000, true);
    let pid = child.id();
    let mut guard = ChildGuard::new(child);

    assert!(
        wait_for_file(&ready, Duration::from_secs(30)),
        "worker never signalled READY within 30s — build / transport init likely failed."
    );
    // Settle INTO the park: past READY + GO, the first step's barrier wait has
    // no peer, so ~1s in the worker is parked mid-ladder.
    std::thread::sleep(Duration::from_millis(1000));

    // SAFETY: FFI kill of our own spawned child (see arm 1).
    unsafe {
        libc::kill(pid as libc::pid_t, libc::SIGINT);
    }

    // 12s deadline: > the ~5s boundary-timeout poison with headroom, << the
    // 20s drain window and the 60s watchdog. `Some` = the worker exited on its
    // own; `None` = it slept through (SIGKILLed by wait_bounded) — the wedge
    // this pin exists to prevent.
    let status = guard.wait_bounded(Duration::from_secs(12)).expect(
        "PARKED worker did not exit within 12s of a directed SIGINT — a parked barrier wait \
         must be bounded by the ~5s boundary timeout (poison → exit 2), never sleep through \
         the drain window",
    );
    assert_eq!(
        status.code(),
        Some(2),
        "a SIGINTed parked worker (peer never arrives) must exit via the bounded barrier-poison \
         path (code 2); None = signal death (ctrlc bridge missing — vacuous), 0 = poison masked; \
         got {status:?}"
    );

    drop(guard);
    drop(run_tmp);
    drop(ws_tmp);
    drop(owner);
}

/// SIGINT-drain regression pin (arm 3 — the SUPERVISOR fan-out
/// itself): spawns the REAL `cerulion graph run <split graph>` (2 groups →
/// supervisor + 2 real workers; the real CLI binary, because
/// `graph_run_supervisor` re-execs `current_exe()` for its workers — a
/// test-binary supervisor would spawn libtest, not workers), waits for the
/// "deployment live" line, then sends SIGINT to the SUPERVISOR PID ONLY.
///
/// The supervisor child is put in its OWN process group
/// (`process_group(0)`), so the workers — which inherit the supervisor's
/// group, not the test's — can receive the drain ONLY via the
/// Ctrl-C→Draining `for g in guards { g.signal_int() }` fan-out. Asserts:
/// exit code 0 within 15s (comfortably under the 20s drain deadline) and
/// every worker pid gone (no stragglers).
///
/// The regression guard: reverting the fan-out loop in
/// `graph_cmd.rs`'s Ctrl-C→Draining arm makes this test FAIL at the 15s
/// deadline — the directed SIGINT reaches only the supervisor, the workers
/// (own process group, no terminal) never drain, and the supervisor only
/// exits after the 20s drain-deadline straggler SIGKILL. That is exactly
/// the failure this pin exists to prevent.
#[cfg(unix)]
#[test]
#[ignore]
#[serial]
fn box_supervisor_directed_sigint_fans_out_and_drains_all_workers() {
    use std::os::unix::process::CommandExt as _;

    // ── (1) Scaffold: ticker workspace + a 2-group split graph. ─────────
    let (ws_tmp, ws_root) = build_ticker_workspace();
    let split = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        name: None,
        identity: "split_sigint".to_string(),
        prefix: "sup".to_string(),
        nodes: ["t1", "t2"]
            .iter()
            .map(|id| NodeDef {
                fuse: None,
                ros2: None,
                id: (*id).to_string(),
                node_type: "ticker".to_string(),
                inputs: Vec::new(),
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "geometry_msgs/Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            })
            .collect(),
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: vec!["g1".to_string(), "g2".to_string()],
    };
    let mut split = split;
    split
        .process_groups
        .insert("g1".to_string(), vec!["t1".to_string()]);
    split
        .process_groups
        .insert("g2".to_string(), vec!["t2".to_string()]);
    // serde_yaml is the engine's OWN graph-write path (`graph_create` uses
    // `serde_yaml::to_string(&GraphConfig)`), so this roundtrips by
    // construction.
    std::fs::write(
        ws_root.join("graphs").join("split_sigint.yaml"),
        serde_yaml::to_string(&split).expect("serialize split graph"),
    )
    .expect("write split graph yaml");

    // ── (2) The REAL CLI binary. ─────────────────────────────────────────
    let repo_root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("cerulion_cli_engine has a workspace parent")
        .parent()
        .expect("crates/ has a parent (the repo root)")
        .to_path_buf();
    let build = Command::new(env!("CARGO"))
        .args(["build", "-p", "cerulion_cli"])
        .current_dir(&repo_root)
        .status()
        .expect("spawn cargo build -p cerulion_cli");
    assert!(build.success(), "building the cerulion CLI failed");
    let target_dir = std::env::var("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| repo_root.join("target"));
    let cerulion_bin = target_dir.join("debug").join("cerulion");
    assert!(
        cerulion_bin.exists(),
        "cerulion binary not found at {}",
        cerulion_bin.display()
    );

    // ── (3) Spawn the supervisor in its OWN process group. ──────────────
    let log_path = ws_root.join("supervisor_sigint.log");
    let log_file = std::fs::File::create(&log_path).expect("create supervisor log");
    let mut cmd = Command::new(&cerulion_bin);
    cmd.args(["graph", "run", "split_sigint"])
        .current_dir(&ws_root)
        // Hermetic — no scouting session/gateway in CI (a real-clock
        // run is permissive-by-default; the kill-switch env keeps it LOCAL-ONLY).
        .env("CERULION_NETWORK", "off")
        .env("RUST_LOG", "info")
        // Belt-and-suspenders with `strip_ansi` below: ask the child's fmt
        // layer to skip color codes entirely (honored by newer
        // tracing-subscriber; the parser strips ANSI regardless).
        .env("NO_COLOR", "1")
        // Own process group: the test's directed kill(supervisor_pid) can
        // reach NOTHING else, and the workers (inheriting the supervisor's
        // fresh group) can only be drained by the supervisor's fan-out.
        .process_group(0)
        .stdout(std::process::Stdio::from(
            log_file.try_clone().expect("clone log handle"),
        ))
        .stderr(std::process::Stdio::from(log_file));
    let child = cmd.spawn().expect("spawn cerulion graph run");
    let sup_pid = child.id();
    let mut guard = ChildGuard::new(child);

    // Prove the process-group isolation before any signalling: the
    // supervisor must be its own group leader (pgid == its pid).
    // If `process_group(0)` silently failed to take, the supervisor (and its
    // workers) share the TEST's group, and the pin's premise — "the workers
    // can only be drained by the fan-out" — is void. Fail loud instead.
    // SAFETY: getpgid(2) on our own spawned child pid; read-only query.
    let pgid = unsafe { libc::getpgid(sup_pid as libc::pid_t) };
    assert_eq!(
        pgid, sup_pid as libc::pid_t,
        "the spawned supervisor must lead its OWN process group \
         (process_group(0) did not take: getpgid = {pgid}, pid = {sup_pid}) — \
         aborting before any signal can touch the test's group"
    );

    // Orphan window: workers spawned BEFORE "deployment live"
    // appears are unknown to `PidSweeper` (it arms after the pid parse) —
    // if the go-live wait panics, they would linger for the supervisor's
    // go_deadline. The supervisor provably LEADS its own group (asserted
    // above), so a Drop-time SIGKILL of the NEGATIVE pgid sweeps it and
    // every worker in one call. Idempotent with the sibling guards (a
    // second SIGKILL is ESRCH, ignored); the >1 floor keeps a bogus pid
    // from ever addressing the caller's own group.
    struct GroupSweeper(libc::pid_t);
    impl Drop for GroupSweeper {
        fn drop(&mut self) {
            if self.0 > 1 {
                // SAFETY: SIGKILL to the negative pgid of a group we created
                // and asserted leadership of; never pid <= 1.
                unsafe {
                    libc::kill(-self.0, libc::SIGKILL);
                }
            }
        }
    }
    let _group_sweeper = GroupSweeper(sup_pid as libc::pid_t);

    // ── (4) Wait (bounded) for the deployment to go live; harvest pids. ──
    let log = wait_for_log_line(&log_path, "deployment live", Duration::from_secs(90));
    let worker_pids = parse_worker_pids(&log);
    assert_eq!(
        worker_pids.len(),
        2,
        "the split graph must spawn exactly 2 workers (parsed from \
         'spawned worker process' lines); log:\n{log}"
    );
    // Belt-and-suspenders (the original root cause): a mis-parsed pid of 0
    // would make ANY later `kill(0, ..)` a CALLER-PROCESS-GROUP signal —
    // killing libtest/cargo themselves. Validate every parsed pid is a
    // plausible foreign pid BEFORE the sweeper (which signals on Drop) exists.
    for pid in &worker_pids {
        assert!(
            *pid > 1 && *pid != std::process::id() && *pid != sup_pid,
            "parsed a bogus worker pid {pid} (log parse must never yield 0/1/self/supervisor \
             — kill(0, sig) would signal the test's own process group); log:\n{log}"
        );
    }
    // Orphan safety: SIGKILL any surviving worker on ANY exit path (panic
    // included) — the supervisor guard only covers the supervisor pid.
    let _sweeper = PidSweeper(worker_pids.clone());

    // Let the deployment genuinely run before draining it.
    std::thread::sleep(Duration::from_millis(1500));

    // ── (5) THE pin: a DIRECTED SIGINT to the supervisor pid ONLY. ──────
    // SAFETY: FFI kill of our own spawned child with the graceful signal.
    unsafe {
        libc::kill(sup_pid as libc::pid_t, libc::SIGINT);
    }

    // ── (6) Clean exit comfortably under the 20s drain deadline. ────────
    let status = guard.wait_bounded(Duration::from_secs(15)).expect(
        "supervisor did not exit within 15s of a directed SIGINT — the Ctrl-C fan-out \
         regression: with it reverted the workers (own process group) never drain and \
         the supervisor only exits after the 20s straggler SIGKILL",
    );
    assert!(
        status.success(),
        "a directed-SIGINT drain must exit 0 (workers SIGINTed by the fan-out, drained \
         clean, reaped); got {status:?} — see {}",
        log_path.display()
    );

    // ── (7) No stragglers: every worker pid is gone (bounded grace). ────
    let deadline = Instant::now() + Duration::from_secs(3);
    for pid in &worker_pids {
        loop {
            // SAFETY: signal 0 = existence probe only, no signal delivered.
            let alive = unsafe { libc::kill(*pid as libc::pid_t, 0) } == 0;
            if !alive {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "worker pid {pid} still alive after the supervisor exited — a straggler \
                 the fan-out should have drained"
            );
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    drop(ws_tmp);
}

/// Bounded poll for `needle` in the (growing) log file at `path`; returns the
/// full log content once seen, panics at the deadline (→ guards reap).
#[cfg(unix)]
fn wait_for_log_line(path: &Path, needle: &str, deadline: Duration) -> String {
    let end = Instant::now() + deadline;
    loop {
        let content = std::fs::read_to_string(path).unwrap_or_default();
        if content.contains(needle) {
            return content;
        }
        assert!(
            Instant::now() < end,
            "log line {needle:?} not seen within {deadline:?}; log so far:\n{content}"
        );
        std::thread::sleep(Duration::from_millis(100));
    }
}

/// Strip ANSI SGR/CSI escape sequences (`ESC [ ... <alpha>`). tracing's fmt
/// layer colors output by default EVEN INTO A FILE (no tty detection), so a
/// naive "first digits after `pid`" parse can land on a color-code digit —
/// e.g. the `0` of `\x1b[0m` — yielding pid 0, and `kill(0, sig)` signals the
/// caller's whole process group (the original root cause: the sweeper's Drop
/// SIGKILLed libtest + cargo themselves).
#[cfg(unix)]
fn strip_ansi(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut chars = s.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '\u{1b}' {
            // CSI: ESC '[' params… final-byte in '@'..='~'. Swallow it.
            if chars.peek() == Some(&'[') {
                chars.next();
                for t in chars.by_ref() {
                    if ('@'..='~').contains(&t) {
                        break;
                    }
                }
            }
            continue;
        }
        out.push(c);
    }
    out
}

/// Pull worker pids out of the supervisor's `spawned worker process` info
/// lines. STRICT shape after ANSI stripping: the literal `pid=` key with the
/// digits IMMEDIATELY after the `=` (tracing fmt renders structured fields as
/// `key=value`) — never "first digits somewhere after the word pid", which
/// can capture escape-code digits.
#[cfg(unix)]
fn parse_worker_pids(log: &str) -> Vec<u32> {
    log.lines()
        .filter(|l| l.contains("spawned worker process"))
        .filter_map(|l| {
            let clean = strip_ansi(l);
            clean.split_once("pid=").and_then(|(_, rest)| {
                let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
                digits.parse().ok()
            })
        })
        .collect()
}

/// Orphan-safety RAII: best-effort SIGKILL of the listed pids on drop (ESRCH
/// — already gone — is the happy case). Covers the worker processes, which
/// the supervisor `ChildGuard` cannot (it owns only the supervisor pid).
///
/// HARD safety floor: pids `<= 1` are NEVER signalled — `kill(0, sig)` hits
/// the caller's whole process group and `kill(1, ..)`/negative pids are
/// system-wide hazards. The caller also validates parsed pids up front; this
/// guard keeps the Drop safe even if that validation drifts.
#[cfg(unix)]
struct PidSweeper(Vec<u32>);

#[cfg(unix)]
impl Drop for PidSweeper {
    fn drop(&mut self) {
        for pid in &self.0 {
            if *pid <= 1 {
                continue;
            }
            // SAFETY: best-effort SIGKILL of validated (> 1) pids this test's
            // supervisor spawned; ESRCH (already reaped) is ignored.
            unsafe {
                libc::kill(*pid as libc::pid_t, libc::SIGKILL);
            }
        }
    }
}
