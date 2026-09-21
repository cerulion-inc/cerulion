// SPDX-License-Identifier: AGPL-3.0-only
//! Linux-only end-to-end tests of the multi-process SUPERVISOR
//! ([`graph_cmd::graph_run_supervisor`], reached when a graph YAML declares
//! `process_groups:`). Unlike the single-worker test (which drives ONE worker
//! from a hand-written test-supervisor), these tests spawn the REAL `cerulion`
//! binary at `graph run <name>` and let the PRODUCTION supervisor plan, mint the
//! shared iceoryx2 `Config`, create the cross-process `MappedBarrier`, spawn one
//! `graph run-worker` process per group (producer-first, READY-gated, released
//! together by the deployment-wide GO sentinel), and run its JOIN lifecycle.
//!
//! ## What they prove
//!
//! * **`box_two_group_split_matches_monolith_oracle`** — the FIREWALL
//!   (Principle #7, replay = live). The 3-node chain `ticker`(Period 1ms, L0) →
//!   `relay`(data-trigger, L1) → `sink`(data-trigger, L2) is split across two
//!   process groups `g1:[ticker]` / `g2:[relay,sink]`, run as two REAL OS
//!   processes gated by a cross-address-space `/dev/shm` `MAP_SHARED` barrier.
//!   Their per-process fire traces (dumped via the `CERULION_MP_TRACE_DIR` seam)
//!   are merged with the PRODUCTION [`merge_partition_traces`] and asserted
//!   byte-identical (on `(node_id, step, global_level)`) to a single-process
//!   MONOLITH built + stepped in THIS test process — plus a HAND oracle
//!   (`[ticker@L0, relay@L1, sink@L2]` per steady-state step) so it is NOT a
//!   self-compare. A terminal Ctrl-C (a process-group `SIGINT`) drains cleanly:
//!   every worker self-drops from the barrier cohort
//!   (`leave_barrier_cohort`) and exits 0 — NO ~5s poison stagger — and the
//!   supervisor returns Ok / exits 0.
//! * **`box_worker_crash_fails_loud_no_orphans`** — under
//!   `CERULION_MP_PEER_LOSS=fail` (the fail-loud policy, opt-in),
//!   SIGKILL ONE worker while the deployment is live; the supervisor must exit
//!   NONZERO with the fail-loud marker in its stderr AND leave NO orphaned worker
//!   processes (it SIGKILLs + reaps every sibling). This env-seam path stays
//!   live beside the CLI flag: the flag is absent, so `resolve_peer_loss(None)`
//!   still consults the env.
//! * **`box_peer_loss_fail_flag_reproduces_fail_loud`** — the SAME crash
//!   scenario driven by the REAL user-facing `--peer-loss fail` CLI flag
//!   with NO env seam set, which pins the flag's e2e path
//!   through clap → `graph_run` → `resolve_peer_loss` → the supervisor's
//!   fail-loud arm.
//! * **`box_worker_crash_continue_survivors_keep_firing`** — the DEFAULT
//!   peer-loss=continue: SIGKILL the CONSUMER group's worker
//!   mid-run; the supervisor drops it from the shared barrier, STAYS ALIVE, and
//!   logs the degraded-continue error, while the surviving Period producer keeps
//!   firing (its dumped trace advances well past the crash — a stall would have
//!   poisoned it at ~5s and it would have exited 2 → all-crashed → supervisor
//!   Err). A terminal SIGINT then drains the survivor to exit 0 and the supervisor
//!   exits Ok, with no orphans.
//! * **`box_all_workers_crash_exits_nonzero_all_crashed`** — the ALL-crashed edge
//!   of peer-loss=continue: SIGKILL BOTH workers near-simultaneously → the JOIN
//!   loop records both crashes → the supervisor exits NONZERO with the
//!   all-crashed marker ("every worker process crashed" / "deployment failed")
//!   and leaves no orphans. Two back-to-back SIGKILLs are TYPICALLY collected in
//!   one pass window (touching the batched multi-death drop), but that is
//!   timing-dependent — the batch's one-grace property is deterministically
//!   pinned by the `drop_dead_peers_batch` unit test in `graph_cmd.rs`, not here.
//!   Likewise, the sweep-consumed-CLEAN-exit e2e path (a clean exit observed
//!   mid-death-sweep) is timing-dependent and pinned at the unit level
//!   (`sweep_classifies_dead_vs_clean_and_reaps_both`) instead of here.
//! * **`box_clean_worker_exit_drains_deployment`** — SIGINT ONE worker only (not
//!   the group, not SIGKILL): it exits 0 while the deployment is otherwise live →
//!   the supervisor's JOIN clean-exit→Draining transition, which SIGINTs the
//!   remaining worker so it ALSO self-drops + exits 0 (NO barrier poison, NO ~5s
//!   stagger — the self-drop regression guard); the supervisor exits 0 with no
//!   orphans and NO `exit code 2` marker anywhere in its output.
//! * **`box_drain_deadline_kills_stopped_straggler`** — SIGSTOP one worker (a
//!   frozen straggler that can never exit), Ctrl-C the group: the SURVIVOR is
//!   blocked INSIDE its step at the barrier boundary (SIGINT is only observable
//!   at step boundaries), so it legitimately takes the bounded ~5s POISON escape
//!   (exit 2, TOLERATED in Draining — the all-exit-0 guarantee is a
//!   responsive-cohort property, pinned by the clean-SIGINT test); the
//!   supervisor must SIGKILL the frozen straggler at the drain deadline (shrunk
//!   via the hidden `CERULION_MP_DRAIN_MS` seam) and still exit 0 — the JOIN
//!   drain-deadline branch.
//!
//! ## Scope + how to run
//!
//! This file COMPILES on all platforms (macOS fully type-checks it), but all
//! tests are `#[ignore]` + `#[serial]` and internally guard
//! `if !cfg!(target_os = "linux") { return; }`: the cross-process barrier needs a
//! real `/dev/shm` `MAP_SHARED` page (on macOS `MappedBarrier` uses an in-process
//! registry, so separate processes never rendezvous) and terminal-Ctrl-C
//! emulation needs POSIX process-group signals. Run on Linux (x86 with
//! WAITPKG, or aarch64 with WFE):
//!
//! ```bash
//! cargo test -p cerulion_cli --test mp_supervisor_box_test -- --ignored --nocapture --test-threads=1
//! ```
//!
//! ### ~8 min scaffold cost
//!
//! The shared [`scaffold`] compiles a 3-node cdylib workspace ONCE per test
//! process (cold `cargo build` of three node crates — the dominant cost, ~8 min).
//! It is memoized in a `OnceLock`, and its `TempDir` is intentionally kept alive
//! for the whole process so both `#[serial]` tests reuse the one build.
//!
//! Hang/orphan safety is mandatory: `ChildGuard::Drop` SIGKILLs the whole process
//! GROUP + reaps, every wait is HARD-bounded, and the supervisor's own fail-loud
//! contract SIGKILLs siblings on any worker death. No fake data (Principle #13):
//! REAL `#[cerulion_node]` cdylibs over REAL iceoryx2 SHM + a REAL barrier.

use std::io::Read;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant};

use cerulion_cli_engine::node_cmd::NodeCreateOptions;
use cerulion_cli_engine::{node_cmd, workspace};
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::{
    merge_partition_traces, DylibNodeEntry, GraphRuntime, MacroPolicy, NodeEntry, ProcessTrace,
    TraceEntry, TransportConfig, TransportManager, VirtualClock,
};
use serde::Deserialize;
use serial_test::serial;

/// The graph name (also the YAML file stem) the supervisor runs.
const GRAPH_NAME: &str = "mpsup_graph";
/// The graph topic prefix — topics resolve to `/mpsup/<node>/<out>`.
const GRAPH_PREFIX: &str = "mpsup";
/// The message schema every port carries.
const SCHEMA: &str = "geometry_msgs/Vector3";

// NO comparison warmup: the supervisor's GO start-gate means
// NO worker enters `run_live` (so no worker publishes) until EVERY worker is
// built + READY — all subscribers are connected before the first publish. First
// -sample delivery is therefore deterministic and the firewall compare is
// STRICT FROM STEP 0 (no prefix is skipped: the GO gate closes
// the pre-GO startup first-sample race).

// ===========================================================================
// Shared scaffold — built ONCE per test process (the ~8 min cost), memoized.
// ===========================================================================

/// The one-per-process scaffolded workspace + the graph the supervisor runs. The
/// `TempDir` is stored (not dropped) so the built cdylibs + graph YAML survive for
/// the whole process; both `#[serial]` tests reuse the single build.
struct Scaffold {
    /// Kept alive for the process lifetime so `ws_root` stays valid.
    _tmp: tempfile::TempDir,
    ws_root: PathBuf,
    /// The constructed graph config — reused as the MONOLITH oracle leg's config
    /// (the supervisor's own planning build feeds the SAME shape to
    /// `build_with_schema_hashes`, so a process-groups-carrying config is valid).
    config: GraphConfig,
}

fn scaffold() -> &'static Scaffold {
    static SCAFFOLD: OnceLock<Scaffold> = OnceLock::new();
    SCAFFOLD.get_or_init(build_scaffold)
}

/// Construct the graph config: `ticker`(L0) → `relay`(L1) → `sink`(L2) split into
/// `g1:[ticker]` / `g2:[relay,sink]` (declaration order ⇒ ranks 0/1). Serializing
/// this exact struct into the YAML the supervisor parses makes it parse-compatible
/// by construction.
fn build_graph_config() -> GraphConfig {
    let out = |name: &str| OutputDef {
        name: name.to_string(),
        schema: SCHEMA.to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    };
    let input = |source: &str| InputDef {
        name: "inp".to_string(),
        source: source.to_string(),
    };
    GraphConfig {
        level_assignments: None,
        network: None,
        name: None,
        identity: GRAPH_NAME.to_string(),
        prefix: GRAPH_PREFIX.to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "ticker".to_string(),
                node_type: "ticker".to_string(),
                inputs: vec![],
                outputs: vec![out("out")],
            },
            NodeDef {
                ros2: None,
                id: "relay".to_string(),
                node_type: "relay".to_string(),
                inputs: vec![input("ticker/out")],
                outputs: vec![out("out")],
            },
            NodeDef {
                ros2: None,
                id: "sink".to_string(),
                node_type: "sink".to_string(),
                inputs: vec![input("relay/out")],
                outputs: vec![],
            },
        ],
        multi_publisher_topics: vec![],
        // Collect target type is inferred as the field's `IndexMap` — insertion
        // order (g1, g2) defines ranks 0 and 1, so we needn't name `indexmap` here.
        process_groups: [
            ("g1".to_string(), vec!["ticker".to_string()]),
            (
                "g2".to_string(),
                vec!["relay".to_string(), "sink".to_string()],
            ),
        ]
        .into_iter()
        .collect(),
        process_group_order: vec![],
    }
}

fn build_scaffold() -> Scaffold {
    let tmp = tempfile::tempdir().expect("temp dir for scaffold workspace");
    let ws = workspace::workspace_create(tmp.path(), "sup_ws").expect("workspace_create");
    let cargo_toml = ws.root.join("Cargo.toml");

    // ticker: pure Period 1ms source (no inputs, one output).
    node_cmd::node_create_with_options(
        &ws.nodes_dir,
        &cargo_toml,
        "ticker",
        Some(MacroPolicy::Period { period_ms: 1 }),
        &NodeCreateOptions {
            outputs: vec![(SCHEMA.to_string(), "out".to_string())],
            inputs: vec![],
            trigger: None,
            raw_ffi: false,
        },
    )
    .expect("node_create ticker");

    // relay: EXPLICIT data-trigger on `inp` (policy DataTrigger + the matching
    // `#[input(trigger)]` field via `NodeCreateOptions::trigger`, mirroring what
    // the CLI's `-T` shorthand resolves to) — NOT the 1-input defaulting warn.
    node_cmd::node_create_with_options(
        &ws.nodes_dir,
        &cargo_toml,
        "relay",
        Some(MacroPolicy::DataTrigger {
            input_name: "inp".to_string(),
        }),
        &NodeCreateOptions {
            outputs: vec![(SCHEMA.to_string(), "out".to_string())],
            inputs: vec![(SCHEMA.to_string(), "inp".to_string())],
            trigger: Some("inp".to_string()),
            raw_ffi: false,
        },
    )
    .expect("node_create relay");

    // sink: EXPLICIT data-trigger on `inp`, no outputs.
    node_cmd::node_create_with_options(
        &ws.nodes_dir,
        &cargo_toml,
        "sink",
        Some(MacroPolicy::DataTrigger {
            input_name: "inp".to_string(),
        }),
        &NodeCreateOptions {
            outputs: vec![],
            inputs: vec![(SCHEMA.to_string(), "inp".to_string())],
            trigger: Some("inp".to_string()),
            raw_ffi: false,
        },
    )
    .expect("node_create sink");

    // Debug builds (matches the workers' default freshest-debug cdylib resolution).
    node_cmd::node_build(&ws.root, "ticker", false).expect("node_build ticker");
    node_cmd::node_build(&ws.root, "relay", false).expect("node_build relay");
    node_cmd::node_build(&ws.root, "sink", false).expect("node_build sink");

    let config = build_graph_config();
    let yaml = serde_yaml::to_string(&config).expect("serialize graph config to YAML");
    std::fs::write(ws.graphs_dir.join(format!("{GRAPH_NAME}.yaml")), yaml)
        .expect("write graph YAML");

    Scaffold {
        _tmp: tmp,
        ws_root: ws.root,
        config,
    }
}

// ===========================================================================
// Trace wire format — the worker dumps `{node_id, step, fire_time_ns,
// global_level}`; we deserialize a local twin and reconstruct `TraceEntry`s.
// (Lifted from `barrier_level_gate_subprocess_iox2_test.rs`.)
// ===========================================================================

#[derive(Deserialize)]
struct FlatEntry {
    node_id: String,
    step: u64,
    fire_time_ns: u64,
    global_level: usize,
}

/// Read + parse a worker's `trace_{group}.json`. Its ABSENCE means the trace seam
/// or the worker broke — fail loud (not a hang). NOTE: the dump filenames are the
/// SANITIZED group names (`sanitize_ns`, `[A-Za-z0-9_]`); this scaffold's groups
/// `g1`/`g2` sanitize to themselves, so the raw name is the filename here.
fn read_worker_trace(dir: &Path, group: &str) -> Vec<FlatEntry> {
    let path = dir.join(format!("trace_{group}.json"));
    let json = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "worker trace `{}` missing/unreadable ({e}) — the CERULION_MP_TRACE_DIR \
             seam or the worker itself broke",
            path.display()
        )
    });
    serde_json::from_str(&json)
        .unwrap_or_else(|e| panic!("worker trace `{}` is not valid JSON: {e}", path.display()))
}

/// Reconstruct `TraceEntry`s from the wire `FlatEntry`s. `duration_ns` is set to 0
/// (wall time — EXCLUDED from `TraceEntry`'s `Eq`, so it cannot affect any assert).
/// `discarded` likewise defaults false (also excluded from `PartialEq`).
fn reconstruct(flats: &[FlatEntry]) -> Vec<TraceEntry> {
    flats
        .iter()
        .map(|f| TraceEntry {
            node_id: Arc::from(f.node_id.as_str()),
            step: f.step,
            fire_time_ns: f.fire_time_ns,
            global_level: f.global_level,
            duration_ns: 0,
            discarded: false,
        })
        .collect()
}

// ===========================================================================
// Trace projections + oracle helpers.
// ===========================================================================

fn max_step(entries: &[TraceEntry]) -> u64 {
    entries.iter().map(|e| e.step).max().unwrap_or(0)
}

/// The comparison sequence: `(node_id, step, global_level)` per fire, for steps in
/// `[lo, hi]`. `fire_time_ns` is EXCLUDED — the polled monolith advances its clock
/// by the fixed 1ms step delta while the deterministic-live workers advance by the
/// handed quantum, so their `fire_time_ns` stamps legitimately differ (same
/// exclusion as `cerulion_core`'s `polled_vs_live_iox2_test`). `step` +
/// `global_level` are replay-deterministic, so they are the firewall keys.
fn seq_tuples(entries: &[TraceEntry], lo: u64, hi: u64) -> Vec<(String, u64, usize)> {
    entries
        .iter()
        .filter(|e| e.step >= lo && e.step <= hi)
        .map(|e| (e.node_id.to_string(), e.step, e.global_level))
        .collect()
}

/// Group a `(step, global_level)`-ordered trace slice into per-step
/// `(node_id, global_level)` lists (for the hand-oracle structural pin). Both the
/// merged trace (sorted by `merge_partition_traces`) and a monolith trace (fired
/// in level order per step) are already `(step, level)` ascending, so a linear
/// group-by-consecutive-step is correct.
fn group_by_step(entries: &[TraceEntry], lo: u64, hi: u64) -> Vec<(u64, Vec<(String, usize)>)> {
    let mut out: Vec<(u64, Vec<(String, usize)>)> = Vec::new();
    for e in entries.iter().filter(|e| e.step >= lo && e.step <= hi) {
        let tuple = (e.node_id.to_string(), e.global_level);
        match out.last_mut() {
            Some((s, v)) if *s == e.step => v.push(tuple),
            _ => out.push((e.step, vec![tuple])),
        }
    }
    out
}

// ===========================================================================
// The MONOLITH oracle leg — a single-process build of the SAME graph, run in
// THIS test process AFTER the multi-process leg. Its per-step fire sequence is
// the non-self-compare anchor for the cross-process merged trace.
// ===========================================================================

/// Mirror of `graph_cmd::cdylib_target_base`: honor `CARGO_TARGET_DIR` (abs or
/// ws-relative) exactly as cargo + `node_build` do, else `<ws>/target`.
fn cdylib_target_base(ws_root: &Path) -> PathBuf {
    match std::env::var_os("CARGO_TARGET_DIR") {
        Some(v) if !v.is_empty() => {
            let dir = PathBuf::from(v);
            if dir.is_absolute() {
                dir
            } else {
                ws_root.join(dir)
            }
        }
        _ => ws_root.join("target"),
    }
}

/// Build the monolith on an ISOLATED per-test transport (`init_for_test` +
/// `iceoryx_test_config` — a distinct SHM root from the DEFAULT data-plane
/// namespace the worker child processes used), load the three debug
/// cdylibs, step `steps` times at 1ms (so the Period ticker fires once per
/// step), and return its full trace.
fn run_monolith(sc: &Scaffold, steps: u64) -> Vec<TraceEntry> {
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "sup_monolith".to_string(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 16,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated monolith test transport");

    // Debug artifacts (`node_build ... false`): `<base>/debug/lib<type>.so`. This
    // leg only RUNS on Linux (body-guarded), so the `.so` suffix is correct.
    let debug_dir = cdylib_target_base(&sc.ws_root).join("debug");
    let mut factories: indexmap::IndexMap<String, Box<dyn NodeEntry>> = indexmap::IndexMap::new();
    for id in ["ticker", "relay", "sink"] {
        let path = debug_dir.join(format!("lib{id}.so"));
        let entry = DylibNodeEntry::load(&path)
            .unwrap_or_else(|e| panic!("load cdylib `{}`: {e}", path.display()));
        factories.insert(id.to_string(), Box::new(entry) as Box<dyn NodeEntry>);
    }

    // Same GraphConfig as the multi-process leg (process_groups present is fine —
    // the supervisor's own planning build feeds the identical shape here).
    let mut runtime = GraphRuntime::build_with_schema_hashes(
        sc.config.clone(),
        factories,
        &mgr,
        Arc::new(VirtualClock::new()),
        None,
    )
    .expect("build monolith runtime");

    for _ in 0..steps {
        runtime.step(Duration::from_millis(1));
    }
    let trace = runtime.trace().to_vec();
    runtime.shutdown();
    trace
}

// ===========================================================================
// Process-group signal helpers — all libc references confined to `#[cfg(unix)]`
// (macOS + Linux compile the real ones; a no-op stub keeps the file portable).
// The tests only RUN on Linux (body-guarded).
// ===========================================================================

/// SIGINT the whole process GROUP led by `pid` (negative pid) — exactly what a
/// terminal Ctrl-C does: signals the supervisor AND every worker it spawned.
#[cfg(unix)]
fn sigint_group(pid: u32) {
    unsafe {
        libc::kill(-(pid as i32), libc::SIGINT);
    }
}
#[cfg(not(unix))]
fn sigint_group(_pid: u32) {}

/// SIGKILL a single process.
#[cfg(unix)]
fn sigkill_pid(pid: u32) {
    unsafe {
        libc::kill(pid as i32, libc::SIGKILL);
    }
}
#[cfg(not(unix))]
fn sigkill_pid(_pid: u32) {}

/// SIGINT a SINGLE process (one worker — NOT the group): models a worker that
/// stops itself while the deployment is otherwise live (the JOIN clean-exit
/// transition), distinct from a terminal Ctrl-C (which signals the whole group).
#[cfg(unix)]
fn sigint_pid(pid: u32) {
    unsafe {
        libc::kill(pid as i32, libc::SIGINT);
    }
}
#[cfg(not(unix))]
fn sigint_pid(_pid: u32) {}

/// SIGSTOP a SINGLE process: freezes it — a stopped process cannot run its
/// SIGINT handler and cannot exit, making it the deterministic DRAIN STRAGGLER
/// (SIGKILL still works on stopped processes, which is exactly what the
/// supervisor's drain-deadline branch relies on).
#[cfg(unix)]
fn sigstop_pid(pid: u32) {
    unsafe {
        libc::kill(pid as i32, libc::SIGSTOP);
    }
}
#[cfg(not(unix))]
fn sigstop_pid(_pid: u32) {}

/// SIGKILL the whole process GROUP led by `pid` (cleanup — no orphans on panic).
#[cfg(unix)]
fn sigkill_group(pid: u32) {
    unsafe {
        libc::kill(-(pid as i32), libc::SIGKILL);
    }
}
#[cfg(not(unix))]
fn sigkill_group(_pid: u32) {}

/// True while `pid` still exists (as a live process or an unreaped zombie).
/// `kill(pid, 0)` returns 0 if present; `-1` (ESRCH) once it is gone + reaped.
#[cfg(unix)]
fn process_alive(pid: u32) -> bool {
    unsafe { libc::kill(pid as i32, 0) == 0 }
}
#[cfg(not(unix))]
fn process_alive(_pid: u32) -> bool {
    false
}

// ===========================================================================
// RAII child guard — Drop SIGKILLs the whole GROUP then reaps, so a test panic
// never orphans the supervisor OR its workers.
// ===========================================================================

struct ChildGuard {
    child: std::process::Child,
    /// The spawned supervisor's pid == its process-GROUP id (it is a group leader).
    pid: u32,
    reaped: bool,
}

impl ChildGuard {
    fn new(child: std::process::Child) -> Self {
        let pid = child.id();
        Self {
            child,
            pid,
            reaped: false,
        }
    }

    /// BOUNDED wait for the supervisor to exit: `Some(status)` on exit, or `None`
    /// if `deadline` elapsed (in which case the whole group is SIGKILLed + reaped
    /// — a hung deployment must NEVER hang a test run). Idempotent.
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
                        self.kill_group_and_reap();
                        return None;
                    }
                    std::thread::sleep(Duration::from_millis(20));
                }
                Err(e) => {
                    eprintln!("[supervisor test] try_wait errored: {e}");
                    self.kill_group_and_reap();
                    return None;
                }
            }
        }
    }

    fn kill_group_and_reap(&mut self) {
        if self.reaped {
            return;
        }
        sigkill_group(self.pid);
        let _ = self.child.kill();
        let _ = self.child.wait();
        self.reaped = true;
    }
}

impl Drop for ChildGuard {
    fn drop(&mut self) {
        if self.reaped {
            return;
        }
        sigkill_group(self.pid);
        let _ = self.child.kill();
        let _ = self.child.wait();
        self.reaped = true;
    }
}

// ===========================================================================
// Spawn + worker-discovery helpers.
// ===========================================================================

/// Spawn the REAL `cerulion graph run <name>` binary as a process-GROUP LEADER
/// (so one `kill(-pid, sig)` reaches the supervisor + every worker it spawns —
/// exactly a terminal Ctrl-C). `trace_dir` sets `CERULION_MP_TRACE_DIR` for the
/// worker fire-trace dump; `pipe_stderr` captures stderr for the fail-loud
/// asserts (cerulion's tracing fmt layer writes to STDERR — init_logging uses
/// `.with_writer(std::io::stderr)` so a command's stdout stays clean data);
/// `pipe_stdout` additionally captures stdout — see the log-stream-split
/// comment in test 4;
/// `extra_env` sets additional env vars (e.g. the hidden `CERULION_MP_DRAIN_MS`
/// drain-window seam for the straggler test); `extra_args` appends REAL CLI
/// args after `graph run <name>` (e.g. `--peer-loss fail`, the
/// user-facing flag surface, exercised e2e by test 7).
fn spawn_supervisor(
    sc: &Scaffold,
    trace_dir: Option<&Path>,
    pipe_stderr: bool,
    pipe_stdout: bool,
    extra_env: &[(&str, &str)],
    extra_args: &[&str],
) -> std::process::Child {
    let exe = env!("CARGO_BIN_EXE_cerulion");
    let mut cmd = Command::new(exe);
    cmd.args(["graph", "run", GRAPH_NAME])
        .args(extra_args)
        .current_dir(&sc.ws_root)
        // Hermetic — no scouting session/gateway in CI (a real-clock
        // run is permissive-by-default; the kill-switch env keeps it LOCAL-ONLY).
        .env("CERULION_NETWORK", "off");
    if let Some(dir) = trace_dir {
        cmd.env("CERULION_MP_TRACE_DIR", dir);
    }
    for (k, v) in extra_env {
        cmd.env(k, v);
    }
    if pipe_stderr {
        cmd.stderr(Stdio::piped());
    }
    if pipe_stdout {
        cmd.stdout(Stdio::piped());
    }
    // Process-group leader. `CommandExt` is unix-only (compiles on macOS +
    // Linux); the tests only run on Linux.
    #[cfg(unix)]
    {
        use std::os::unix::process::CommandExt;
        cmd.process_group(0);
    }
    cmd.spawn().expect("spawn `cerulion graph run` supervisor")
}

/// The worker pids the supervisor spawned (its children whose argv holds
/// `run-worker`). Empty if `pgrep` is unavailable or none are up yet.
fn worker_pids(supervisor_pid: u32) -> Vec<u32> {
    let out = Command::new("pgrep")
        .args(["-P", &supervisor_pid.to_string(), "-f", "run-worker"])
        .output();
    match out {
        Ok(o) => String::from_utf8_lossy(&o.stdout)
            .lines()
            .filter_map(|l| l.trim().parse::<u32>().ok())
            .collect(),
        Err(_) => Vec::new(),
    }
}

/// BOUNDED poll until the supervisor has spawned BOTH worker processes (the
/// READY-gated producer-first spawn brings them up sequentially). Returns whatever
/// was observed at the deadline.
fn wait_for_two_workers(supervisor_pid: u32, deadline: Duration) -> Vec<u32> {
    let start = Instant::now();
    loop {
        let pids = worker_pids(supervisor_pid);
        if pids.len() >= 2 || start.elapsed() >= deadline {
            return pids;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// The worker pid running a SPECIFIC group's plan (its argv holds `plan_<group>.json`
/// — the supervisor writes one plan file per group under the deployment plan dir,
/// named `plan_{sanitize_ns(group)}.json`, and this scaffold's groups `g1`/`g2`
/// sanitize to themselves). `None` until that worker is up. Used to target a
/// specific group (e.g. SIGKILL the CONSUMER group `g2` while keeping the Period
/// producer `g1` alive).
fn worker_pid_for_group(supervisor_pid: u32, group: &str) -> Option<u32> {
    let pat = format!("plan_{group}.json");
    let out = Command::new("pgrep")
        .args(["-P", &supervisor_pid.to_string(), "-f", &pat])
        .output();
    match out {
        Ok(o) => String::from_utf8_lossy(&o.stdout)
            .lines()
            .find_map(|l| l.trim().parse::<u32>().ok()),
        Err(_) => None,
    }
}

// ===========================================================================
// TEST 1 — the FIREWALL: 2-group split == monolith oracle == hand oracle.
// ===========================================================================

#[test]
#[ignore]
#[serial]
fn box_two_group_split_matches_monolith_oracle() {
    if !cfg!(target_os = "linux") {
        // Cross-process MAP_SHARED barrier + process-group signals are Linux-only.
        return;
    }

    let sc = scaffold();
    let trace_tmp = tempfile::tempdir().expect("temp dir for worker trace dumps");
    let trace_dir = trace_tmp.path();

    // (1) spawn the production supervisor (process-group leader; trace seam on).
    let child = spawn_supervisor(sc, Some(trace_dir), false, false, &[], &[]);
    let mut guard = ChildGuard::new(child);
    let supervisor_pid = guard.pid;

    // Wait until BOTH workers are up + stepping, THEN let them run ~4s of
    // barrier-lockstep so the trace is long (K well past the liveness floor).
    let pids = wait_for_two_workers(supervisor_pid, Duration::from_secs(30));
    assert!(
        pids.len() >= 2,
        "supervisor never brought up both worker processes within 30s (saw {pids:?}) — \
         a build/transport-init/barrier-open failure (see stderr above); this is a \
         fail-fast, not a hang"
    );
    std::thread::sleep(Duration::from_secs(4));

    // (2) terminal Ctrl-C: SIGINT the whole process group (supervisor + workers).
    sigint_group(supervisor_pid);

    // (3) the supervisor must DRAIN and exit 0. Every worker
    // receives the group SIGINT directly, self-drops from the barrier cohort
    // (`leave_barrier_cohort`), and exits 0 — so the drain is FAST (no ~5s poison
    // stagger). 40s is a generous hang backstop, not the expected duration.
    let status = guard.wait_bounded(Duration::from_secs(40)).expect(
        "supervisor did not exit within 40s of Ctrl-C (killed) — the JOIN drain loop \
         hung or a worker never wound down",
    );
    assert!(
        status.success(),
        "supervisor must exit 0 after Ctrl-C (every worker self-drops + exits 0); got {status:?}"
    );

    // (4) read both worker traces (must exist + be non-empty — absence means the
    // seam or a worker broke).
    let g1_flats = read_worker_trace(trace_dir, "g1");
    let g2_flats = read_worker_trace(trace_dir, "g2");
    assert!(!g1_flats.is_empty(), "g1 (ticker) trace is empty");
    assert!(!g2_flats.is_empty(), "g2 (relay+sink) trace is empty");
    let g1 = reconstruct(&g1_flats);
    let g2 = reconstruct(&g2_flats);

    // (5) lockstep pin: the barrier gates level advance, so the two groups' step
    // counts differ by at most 1 (SIGINT stops them within a step of each other).
    let g1_max = max_step(&g1);
    let g2_max = max_step(&g2);
    let drift = g1_max.abs_diff(g2_max);
    assert!(
        drift <= 2,
        "g1/g2 step drift {drift} (g1_max={g1_max}, g2_max={g2_max}) exceeds 2 — the \
         cross-process barrier is NOT gating level advance in lockstep"
    );

    // (6) truncate to the last step BOTH completed; require real liveness.
    let k = g1_max.min(g2_max);
    assert!(
        k >= 50,
        "only {k} lockstep steps completed — the deployment barely advanced (expected \
         hundreds after ~4s at a 1ms quantum); liveness floor not met"
    );

    // (7) merge with the PRODUCTION merge (g1 = rank 0 producer, g2 = rank 1).
    let merged = merge_partition_traces(&[
        ProcessTrace {
            rank: 0,
            trace: &g1,
        },
        ProcessTrace {
            rank: 1,
            trace: &g2,
        },
    ])
    .expect("merge distinct ranks 0,1");

    // (8) MONOLITH oracle leg (single-process, THIS process, AFTER the MP leg).
    // Step past K so its trace covers the compared window; truncation aligns them.
    let monolith = run_monolith(sc, k + 2);

    // (9) THE FIREWALL: the merged cross-process sequence == the monolith sequence
    // over the FULL window `[0, K]` on (node_id, step, global_level) — STRICT FROM
    // STEP 0, no warmup skip: the GO gate guarantees every worker's subscribers
    // were connected before ANY worker published, so even the very first sample
    // deliveries are deterministic. Byte-identical → the barrier is a WHEN-gate on
    // level advance, never a change to WHAT fires or in what order (Principle #7).
    let merged_seq = seq_tuples(&merged, 0, k);
    let mono_seq = seq_tuples(&monolith, 0, k);
    assert_eq!(
        merged_seq, mono_seq,
        "cross-process merged trace diverges from the single-process monolith over \
         steps [0, {k}] — the multi-process split is NOT replay-identical \
         to the monolith (firewall breach, Principle #7)"
    );

    // (10) HAND-oracle structural pin (anti-tautology — step 9 would pass if BOTH
    // legs broke identically). Assert each step fires EXACTLY
    // [ticker@L0, relay@L1, sink@L2] in that order, over a window starting at the
    // FIRST step.
    //
    // Strict-from-the-first-step grounding: the workers spawn producer-first +
    // READY-gated, so g2's relay/sink subscribers OPEN g1's already-created topics
    // at BUILD time — and the GO gate holds EVERY worker between READY and
    // `run_live` until ALL are built, so all subscribers are connected before the
    // FIRST publish. Under the same-step within-level collapse (cf.
    // `polled_vs_live_iox2_test`) a data-trigger consumer fires the SAME step
    // its upstream published, so EVERY step — including the very first — is
    // [ticker, relay, sink]. (No mid-run window is needed to dodge
    // a pre-GO first-sample race: the GO gate closes it.) The window is capped 40
    // wide, well below k (>= 50 asserted above), keeping it clear of the
    // SIGINT-truncated tail; the K truncation already bounds the upper edge.
    let expected_step: Vec<(String, usize)> = vec![
        ("ticker".to_string(), 0),
        ("relay".to_string(), 1),
        ("sink".to_string(), 2),
    ];
    let sample_lo = 0;
    let sample_hi = 40u64.min(k);
    let sampled = group_by_step(&merged, sample_lo, sample_hi);
    assert!(
        !sampled.is_empty(),
        "sampled window [{sample_lo}, {sample_hi}] is empty (k={k})"
    );
    for (step, group) in &sampled {
        assert_eq!(
            *group, expected_step,
            "step {step} did not fire exactly [ticker@L0, relay@L1, sink@L2] in order \
             (got {group:?}) — the DAG fire structure is wrong in the merged trace"
        );
    }

    drop(guard);
    drop(trace_tmp);
}

// ===========================================================================
// TEST 2 — worker CRASH → fail-loud + no orphans.
// ===========================================================================

#[test]
#[ignore]
#[serial]
fn box_worker_crash_fails_loud_no_orphans() {
    if !cfg!(target_os = "linux") {
        return;
    }

    let sc = scaffold();

    // (1) spawn the supervisor under the OPT-IN fail-loud policy
    // (`CERULION_MP_PEER_LOSS=fail` — the fail-loud behavior, non-default);
    // process-group leader; pipe stderr for the marker. stderr only: the fail-loud
    // marker asserted below comes from main()'s `eprintln!("Error: {e}")` (a
    // CliError render) — genuinely stderr.
    let mut child = spawn_supervisor(
        sc,
        None,
        true,
        false,
        &[("CERULION_MP_PEER_LOSS", "fail")],
        &[],
    );
    // Take stderr BEFORE the guard owns the child; drain it on a thread to avoid a
    // pipe-buffer deadlock (workers inherit the supervisor's stderr, so BOTH flow
    // into this pipe). The reader returns once the pipe hits EOF (all writers exit).
    let stderr = child.stderr.take();
    let reader = std::thread::spawn(move || {
        let mut s = String::new();
        if let Some(mut e) = stderr {
            let _ = e.read_to_string(&mut s);
        }
        s
    });
    let mut guard = ChildGuard::new(child);
    let supervisor_pid = guard.pid;

    // (2) wait until both workers are up + stepping.
    let pids = wait_for_two_workers(supervisor_pid, Duration::from_secs(30));
    assert!(
        pids.len() >= 2,
        "supervisor never brought up both workers within 30s (saw {pids:?})"
    );
    // Small settle so the deployment is genuinely LIVE (JOIN state == Normal).
    std::thread::sleep(Duration::from_secs(2));

    // (3) CRASH one worker (SIGKILL a single worker pid — NOT the group).
    let killed = pids[0];
    let survivor = pids[1];
    sigkill_pid(killed);

    // (4) fail-loud: the supervisor must exit NONZERO within a bounded window.
    let status = guard.wait_bounded(Duration::from_secs(30)).expect(
        "supervisor did not exit within 30s of a worker crash (killed) — it should \
         fail-loud the moment a worker dies while running",
    );
    assert!(
        !status.success(),
        "supervisor MUST exit non-zero when a worker crashes ({status:?}) — fail-loud \
         semantics (any worker death stops the graph)"
    );

    // (5) its stderr must carry the fail-loud marker (the worker's own stderr also
    // flows into this pipe, so we assert on the supervisor's distinctive fragment).
    let stderr_text = reader.join().unwrap_or_default();
    assert!(
        stderr_text.contains("died") && stderr_text.contains("fail-loud"),
        "supervisor stderr must name the fail-loud worker-death contract (\"died\" + \
         \"fail-loud\"); got:\n{stderr_text}"
    );

    // (6) NO ORPHANS: after the supervisor exits it has SIGKILLed + reaped every
    // sibling, so BOTH recorded worker pids must be gone. (pid reuse within a few
    // seconds is not a realistic hazard for our own reaped children.)
    let orphan_deadline = Instant::now() + Duration::from_secs(10);
    for pid in [killed, survivor] {
        while process_alive(pid) && Instant::now() < orphan_deadline {
            std::thread::sleep(Duration::from_millis(50));
        }
        assert!(
            !process_alive(pid),
            "worker pid {pid} is still alive after the supervisor exited — the fail-loud \
             path leaked an orphan (it must SIGKILL + reap every sibling)"
        );
    }

    drop(guard);
}

// ===========================================================================
// TEST 3 — clean worker exit → Draining tolerance → supervisor exits 0.
// ===========================================================================

/// SIGINT ONE worker (not the group, not SIGKILL): its own Ctrl-C handler flips
/// `running` → `run_live` returns → it exits 0 while the deployment is otherwise
/// live. The supervisor's JOIN loop must take the clean-exit → Draining
/// transition (NOT fail-loud) AND SIGINT the REMAINING worker
/// so it too self-drops from the barrier and exits 0. The peer must NOT poison
/// (no ~5s barrier stagger, no `exit code 2` anywhere): the departed worker's own
/// self-drop already removed the barrier stall, and the supervisor's SIGINT tells
/// the survivor to leave gracefully. The supervisor exits SUCCESS with no orphans.
#[test]
#[ignore]
#[serial]
fn box_clean_worker_exit_drains_deployment() {
    if !cfg!(target_os = "linux") {
        // Cross-process MAP_SHARED barrier + POSIX signals are Linux-only here.
        return;
    }

    let sc = scaffold();

    // (1) spawn the production supervisor (process-group leader). Pipe BOTH streams
    // (the tracing fmt layer writes to STDERR, as does CliError) so we can
    // assert NO poison/exit-2 marker appears — the self-drop regression guard.
    // RUST_LOG=warn so the drain-path warns/errors are visible.
    let mut child = spawn_supervisor(sc, None, true, true, &[("RUST_LOG", "warn")], &[]);
    let stderr = child.stderr.take();
    let stderr_reader = std::thread::spawn(move || {
        let mut s = String::new();
        if let Some(mut e) = stderr {
            let _ = e.read_to_string(&mut s);
        }
        s
    });
    let stdout = child.stdout.take();
    let stdout_reader = std::thread::spawn(move || {
        let mut s = String::new();
        if let Some(mut o) = stdout {
            let _ = o.read_to_string(&mut s);
        }
        s
    });
    let mut guard = ChildGuard::new(child);
    let supervisor_pid = guard.pid;

    // (2) wait until both workers are up, then settle so the deployment is
    // genuinely LIVE (GO signaled, JOIN in Normal).
    let pids = wait_for_two_workers(supervisor_pid, Duration::from_secs(30));
    assert!(
        pids.len() >= 2,
        "supervisor never brought up both workers within 30s (saw {pids:?})"
    );
    std::thread::sleep(Duration::from_secs(2));

    // (3) SIGINT ONE worker only — a clean self-stop, not a crash.
    let sigint_at = Instant::now();
    sigint_pid(pids[0]);

    // (4) the supervisor must drain and exit 0 within a bounded window:
    // clean exit observed → Draining + SIGINT the survivor → the survivor self-drops
    // + exits 0 (NO 5s poison) → all reaped → Ok. Bounded 40s hang backstop.
    let status = guard.wait_bounded(Duration::from_secs(40)).expect(
        "supervisor did not exit within 40s of a clean worker exit — the JOIN \
         clean-exit→Draining transition or the drain loop hung",
    );
    let drain_elapsed = sigint_at.elapsed();
    assert!(
        status.success(),
        "supervisor MUST exit 0 after a clean worker exit (Draining + SIGINT the \
         survivor → it self-drops + exits 0); got {status:?} — a fail-loud here means \
         the JOIN loop treated a clean self-stop as a worker death"
    );

    // (5) no orphans: both workers must be gone once the supervisor exits.
    let orphan_deadline = Instant::now() + Duration::from_secs(10);
    for pid in pids {
        while process_alive(pid) && Instant::now() < orphan_deadline {
            std::thread::sleep(Duration::from_millis(50));
        }
        assert!(
            !process_alive(pid),
            "worker pid {pid} is still alive after the supervisor exited — the drain \
             path leaked an orphan"
        );
    }

    // (6) self-drop regression guard: the survivor self-dropped + exited 0, so NO
    // barrier poison happened. Assert the combined output carries NO `exit code 2`
    // /poison marker (without the self-drop the survivor would be poisoned at
    // ~5s and log the "exiting with code 2" error). The survivor should exit
    // fast; a >15s drain would itself indicate a poison-stagger regression.
    let stderr_text = stderr_reader.join().unwrap_or_default();
    let stdout_text = stdout_reader.join().unwrap_or_default();
    let combined = format!("{stdout_text}{stderr_text}");
    assert!(
        !combined.contains("exiting with code 2") && !combined.contains("poisoning runtime"),
        "a clean shutdown must NOT poison any worker (the self-drop) — found an \
         exit-2/poison marker in the supervisor output:\n{combined}"
    );
    assert!(
        drain_elapsed < Duration::from_secs(15),
        "clean shutdown took {drain_elapsed:?} (> 15s) — a self-drop drain should be \
         fast; this suggests a ~5s barrier-poison stagger regression"
    );

    drop(guard);
}

// ===========================================================================
// TEST 4 — drain-deadline straggler: a FROZEN worker is SIGKILLed at the
// (env-shrunk) drain deadline; the supervisor still exits 0. The SURVIVOR
// legitimately poisons (exit 2, tolerated) — see the doc comment.
// ===========================================================================

/// SIGSTOP one worker (frozen — it cannot run its SIGINT handler, cannot
/// exit), then terminal-Ctrl-C the whole group: the supervisor enters Draining,
/// and the frozen straggler must be SIGKILLed at the drain deadline (shrunk via
/// the hidden `CERULION_MP_DRAIN_MS` seam) — the JOIN branch no other test
/// reached.
///
/// WHY THE SURVIVOR LEGITIMATELY POISONS HERE (do NOT
/// re-tighten this to "survivor exits 0"): with its peer kernel-STOPPED, the
/// survivor is blocked INSIDE its step at the level-boundary barrier wait,
/// waiting on the stopped peer's slot. The SIGINT `running` flag is only
/// observable at STEP boundaries, so the survivor can never reach its
/// `leave_barrier_cohort` self-drop point — after the ~5s
/// `BARRIER_BOUNDARY_TIMEOUT` it takes the bounded POISON escape and exits 2.
/// That is the correct, bounded behavior for a kernel-stopped peer: an
/// early-cancelled barrier wait would abandon a step mid-level (a
/// determinism/data-loss breach). The all-workers-exit-0 guarantee is a
/// RESPONSIVE-cohort property, pinned by the clean-SIGINT test
/// (`box_two_group_split_matches_monolith_oracle`).
///
/// Asserts: (a) the straggler is killed at the drain deadline (the production
/// straggler-kill warn), (b) the supervisor returns Ok, (c) the Draining
/// tolerating-warn fires for the poisoned survivor ("worker exited NONZERO
/// during drain"), (d) BOTH worker pids gone (SIGKILL works on stopped
/// processes) — no orphans.
#[test]
#[ignore]
#[serial]
fn box_drain_deadline_kills_stopped_straggler() {
    if !cfg!(target_os = "linux") {
        return;
    }

    let sc = scaffold();

    // (1) spawn the supervisor with an 8s drain window + BOTH output streams
    // piped. 8s (not 5s): the SURVIVOR's ~5s barrier-poison exit-2 must be
    // OBSERVED + tolerated by the JOIN loop (assert (c)) BEFORE the drain
    // deadline SIGKILLs everything — a 5s window would race the 5s poison by
    // mere tens of ms. The straggler kill still fires (the stopped worker can
    // never exit), well inside the 30s bound.
    // The log streams: cerulion's tracing fmt layer writes to STDERR
    // (`.with_writer(std::io::stderr)` in `cerulion_core::init_logging`), so the
    // straggler-kill `tracing::warn!` lands on stderr, as does a CliError
    // rendered by main()'s `eprintln!("Error: {e}")`. We capture BOTH streams
    // and assert on the concatenation, keeping the test robust to a future
    // `with_writer(std::io::stderr)` change. RUST_LOG is pinned to `warn`
    // explicitly so the warn is visible regardless of inherited env.
    let mut child = spawn_supervisor(
        sc,
        None,
        true,
        true,
        &[("CERULION_MP_DRAIN_MS", "8000"), ("RUST_LOG", "warn")],
        &[],
    );
    // Drain BOTH pipes on threads (workers inherit them; an undrained pipe
    // buffer would deadlock the deployment). EOF once all writers exit.
    let stderr = child.stderr.take();
    let stderr_reader = std::thread::spawn(move || {
        let mut s = String::new();
        if let Some(mut e) = stderr {
            let _ = e.read_to_string(&mut s);
        }
        s
    });
    let stdout = child.stdout.take();
    let stdout_reader = std::thread::spawn(move || {
        let mut s = String::new();
        if let Some(mut o) = stdout {
            let _ = o.read_to_string(&mut s);
        }
        s
    });
    let mut guard = ChildGuard::new(child);
    let supervisor_pid = guard.pid;

    // (2) both workers up + settled (deployment live, JOIN in Normal).
    let pids = wait_for_two_workers(supervisor_pid, Duration::from_secs(30));
    assert!(
        pids.len() >= 2,
        "supervisor never brought up both workers within 30s (saw {pids:?})"
    );
    std::thread::sleep(Duration::from_secs(2));

    // (3) FREEZE one worker, then Ctrl-C the whole group. The frozen worker's
    // pending SIGINT is undeliverable (stopped), so it can never exit on its
    // own — the deterministic straggler.
    sigstop_pid(pids[0]);
    sigint_group(supervisor_pid);

    // (4) the survivor poisons at ~5s (exit 2, tolerated in Draining); the
    // supervisor must SIGKILL the straggler at the 8s drain deadline and exit
    // SUCCESS well within the bounded 30s (8s drain + overhead).
    let status = guard.wait_bounded(Duration::from_secs(30)).expect(
        "supervisor did not exit within 30s of Ctrl-C with a frozen straggler — the \
         drain-deadline SIGKILL branch did not fire (the JOIN drain loop hung)",
    );
    assert!(
        status.success(),
        "supervisor must exit 0 after killing the drain straggler; got {status:?}"
    );

    // (5) BOTH worker pids gone — SIGKILL works on stopped processes, which is
    // exactly what the drain-deadline branch relies on.
    let orphan_deadline = Instant::now() + Duration::from_secs(10);
    for pid in pids {
        while process_alive(pid) && Instant::now() < orphan_deadline {
            std::thread::sleep(Duration::from_millis(50));
        }
        assert!(
            !process_alive(pid),
            "worker pid {pid} is still alive after the supervisor exited — the \
             drain-deadline branch failed to SIGKILL the frozen straggler"
        );
    }

    // (6) the production straggler-kill warn must appear in the supervisor's
    // COMBINED output (exact fragment of the `tracing::warn!` at the
    // drain-deadline branch). Combined for robustness: the tracing fmt layer
    // writes to stderr (as do CliError renders) — asserting on the
    // concatenation stays correct across the writer choice.
    let stderr_text = stderr_reader.join().unwrap_or_default();
    let stdout_text = stdout_reader.join().unwrap_or_default();
    let combined = format!("{stdout_text}{stderr_text}");
    assert!(
        combined.contains("killing straggler"),
        "supervisor output (stdout+stderr) must carry the drain-deadline straggler-kill \
         warn (\"killing straggler\"); got:\n{combined}"
    );

    // (7) the poisoned SURVIVOR's exit 2 must be TOLERATED in Draining — the
    // drain-phase warn ("worker exited NONZERO during drain") fires, and the
    // run still ends Ok (asserted in (4)). See the doc comment: with its peer
    // kernel-stopped, the survivor is blocked INSIDE a step at the barrier
    // boundary; SIGINT is observable only at step boundaries, so the ~5s poison
    // is its bounded escape — legitimately exit 2, never a hang, never Err.
    assert!(
        combined.contains("worker exited NONZERO during drain"),
        "the poisoned survivor's nonzero drain exit must be observed + tolerated \
         (marker \"worker exited NONZERO during drain\"); got:\n{combined}"
    );

    drop(guard);
}

// ===========================================================================
// TEST 5 — DEFAULT peer-loss=continue: a CRASHED consumer group is dropped from
// the barrier; the surviving Period producer keeps firing; supervisor stays
// alive + logs the degraded-continue error; a terminal SIGINT then drains it 0.
// ===========================================================================

/// SIGKILL the CONSUMER group's worker (`g2` = relay+sink) mid-run under the
/// DEFAULT peer-loss=continue policy. The supervisor must (a) NOT fail-loud — stay
/// alive and keep joining; (b) drop the dead peer from the shared barrier so the
/// surviving Period producer (`g1` = ticker) proceeds past the boundary and KEEPS
/// FIRING (its dumped trace advances well past the crash — had the drop failed, the
/// producer would stall, poison at ~5s, exit 2, and, with the consumer already
/// crashed, ALL-crashed → supervisor `Err`); (c) log the degraded-continue error. A
/// terminal SIGINT then drains the survivor (self-drop + exit 0) and the supervisor
/// exits SUCCESS with the degraded-run warning, no orphans.
///
/// The producer-keeps-firing evidence is layered: the supervisor STAYING ALIVE
/// across a >6s post-crash window is itself the proof the survivor did not poison
/// (a poison would have surfaced as all-crashed → `Err` within ~5s), and the
/// survivor's dumped trace (`max_step` well past the ~1s pre-crash mark) confirms
/// post-crash progress.
#[test]
#[ignore]
#[serial]
fn box_worker_crash_continue_survivors_keep_firing() {
    if !cfg!(target_os = "linux") {
        return;
    }

    let sc = scaffold();
    let trace_tmp = tempfile::tempdir().expect("temp dir for worker trace dumps");
    let trace_dir = trace_tmp.path();

    // (1) spawn the supervisor under the DEFAULT policy (NO CERULION_MP_PEER_LOSS =
    // peer-loss=continue). Trace seam ON so the SURVIVOR (g1) dumps its trace; pipe
    // BOTH streams for the degraded-continue marker; RUST_LOG=warn shows the
    // error-level continue log.
    let mut child = spawn_supervisor(
        sc,
        Some(trace_dir),
        true,
        true,
        &[("RUST_LOG", "warn")],
        &[],
    );
    let stderr = child.stderr.take();
    let stderr_reader = std::thread::spawn(move || {
        let mut s = String::new();
        if let Some(mut e) = stderr {
            let _ = e.read_to_string(&mut s);
        }
        s
    });
    let stdout = child.stdout.take();
    let stdout_reader = std::thread::spawn(move || {
        let mut s = String::new();
        if let Some(mut o) = stdout {
            let _ = o.read_to_string(&mut s);
        }
        s
    });
    let mut guard = ChildGuard::new(child);
    let supervisor_pid = guard.pid;

    // (2) both workers up; settle only briefly so the CRASH is early and the
    // post-crash progress window is long.
    let pids = wait_for_two_workers(supervisor_pid, Duration::from_secs(30));
    assert!(
        pids.len() >= 2,
        "supervisor never brought up both workers within 30s (saw {pids:?})"
    );
    std::thread::sleep(Duration::from_secs(1));

    // (3) SIGKILL the CONSUMER group `g2` (relay+sink) — the Period producer `g1`
    // (ticker) keeps firing standalone, giving strong "survivor kept firing"
    // evidence in its trace.
    let g2_pid = worker_pid_for_group(supervisor_pid, "g2")
        .expect("could not find the g2 (consumer) worker pid via pgrep");
    let g1_pid = worker_pid_for_group(supervisor_pid, "g1")
        .expect("could not find the g1 (producer) worker pid via pgrep");
    sigkill_pid(g2_pid);

    // (4) the supervisor must STAY ALIVE across a >6s post-crash window. If the
    // drop failed, the survivor would poison at ~5s → all-crashed → the supervisor
    // returns Err and exits within ~5s. Polling it alive for 7s proves the survivor
    // did NOT poison (the drop worked and the survivor kept running).
    let watch_until = Instant::now() + Duration::from_secs(7);
    while Instant::now() < watch_until {
        assert!(
            process_alive(supervisor_pid),
            "supervisor exited on a worker crash under peer-loss=continue — it must \
             stay alive and continue degraded (a poisoned survivor within ~5s would \
             indicate the barrier drop failed)"
        );
        std::thread::sleep(Duration::from_millis(200));
    }

    // (5) terminal SIGINT the whole group → the survivor self-drops + exits 0; the
    // supervisor drains and exits SUCCESS (degraded run, but not a failure).
    sigint_group(supervisor_pid);
    let status = guard.wait_bounded(Duration::from_secs(30)).expect(
        "supervisor did not exit within 30s of the terminal SIGINT after a \
         crash-continue — the drain loop hung",
    );
    assert!(
        status.success(),
        "supervisor MUST exit 0 after a crash-continue + terminal drain (one group \
         crashed, one survived); got {status:?}"
    );

    // (6) no orphans: both worker pids gone.
    let orphan_deadline = Instant::now() + Duration::from_secs(10);
    for pid in [g1_pid, g2_pid] {
        while process_alive(pid) && Instant::now() < orphan_deadline {
            std::thread::sleep(Duration::from_millis(50));
        }
        assert!(
            !process_alive(pid),
            "worker pid {pid} is still alive after the supervisor exited — an orphan leaked"
        );
    }

    // (7) the supervisor must have logged the degraded-continue error (the drop
    // reaction), NOT the fail-loud death marker.
    let stderr_text = stderr_reader.join().unwrap_or_default();
    let stdout_text = stdout_reader.join().unwrap_or_default();
    let combined = format!("{stdout_text}{stderr_text}");
    assert!(
        combined.contains("continuing degraded (peer-loss=continue)"),
        "supervisor output must carry the degraded-continue marker \
         (\"continuing degraded (peer-loss=continue)\"); got:\n{combined}"
    );

    // (8) survivor kept firing: g1 (ticker) dumped a trace advancing well past the
    // ~1s pre-crash mark. g2 was SIGKILLed (no dump), so only g1's trace exists.
    let g1_flats = read_worker_trace(trace_dir, "g1");
    let g1 = reconstruct(&g1_flats);
    assert!(
        !g1.is_empty(),
        "the surviving producer g1 dumped an empty trace"
    );
    let g1_max = max_step(&g1);
    assert!(
        g1_max >= 50,
        "the surviving producer only reached step {g1_max} (< 50) — it did not keep \
         firing across the >6s post-crash window (a stalled/poisoned survivor)"
    );

    drop(guard);
    drop(trace_tmp);
}

// ===========================================================================
// TEST 6 — ALL workers crash (default peer-loss=continue): a run with NO
// survivors is a FAILURE — supervisor exits NONZERO with the all-crashed
// marker. Two back-to-back SIGKILLs TYPICALLY also touch the batched
// multi-death drop path (timing-dependent; deterministically pinned by the
// `drop_dead_peers_batch` unit test, not here).
// ===========================================================================

/// SIGKILL BOTH workers near-simultaneously under the DEFAULT peer-loss=continue
/// policy. Continue tolerates PARTIAL loss, but a run where EVERY worker crashed
/// (no clean exits) must still fail: the JOIN loop records both groups in the
/// crash ledger, the all-crashed edge returns `Err`, and the supervisor exits
/// NONZERO with the all-crashed marker in its output. The two near-simultaneous
/// deaths TYPICALLY land in one JOIN pass window and route through the batch path
/// (one death event + the sweep collects the second), but that is
/// timing-dependent — the one-grace property is deterministically pinned by the
/// `drop_dead_peers_batch` unit test in `graph_cmd.rs`, not by this test.
/// No orphans afterwards.
#[test]
#[ignore]
#[serial]
fn box_all_workers_crash_exits_nonzero_all_crashed() {
    if !cfg!(target_os = "linux") {
        return;
    }

    let sc = scaffold();

    // (1) spawn the supervisor under the DEFAULT policy (no CERULION_MP_PEER_LOSS).
    // Pipe BOTH streams: the all-crashed `tracing::error!` lands on stderr
    // (fmt layer), as does the CliError render from main()'s
    // `eprintln!("Error: {e}")` — assert on the concatenation. Drain both on threads
    // (undrained pipes deadlock; workers inherit them). RUST_LOG=warn for the
    // error-level markers.
    let mut child = spawn_supervisor(sc, None, true, true, &[("RUST_LOG", "warn")], &[]);
    let stderr = child.stderr.take();
    let stderr_reader = std::thread::spawn(move || {
        let mut s = String::new();
        if let Some(mut e) = stderr {
            let _ = e.read_to_string(&mut s);
        }
        s
    });
    let stdout = child.stdout.take();
    let stdout_reader = std::thread::spawn(move || {
        let mut s = String::new();
        if let Some(mut o) = stdout {
            let _ = o.read_to_string(&mut s);
        }
        s
    });
    let mut guard = ChildGuard::new(child);
    let supervisor_pid = guard.pid;

    // (2) both workers up + a short settle (deployment live, JOIN in Normal).
    let pids = wait_for_two_workers(supervisor_pid, Duration::from_secs(30));
    assert!(
        pids.len() >= 2,
        "supervisor never brought up both workers within 30s (saw {pids:?})"
    );
    std::thread::sleep(Duration::from_secs(1));

    // (3) SIGKILL BOTH workers back-to-back. Near-simultaneous deaths TYPICALLY
    // land in one ~20ms JOIN pass window and route through the batched drop (one
    // full stall grace + one ZERO-grace batch-mate), but that is timing-dependent;
    // the one-grace property is pinned deterministically by the
    // `drop_dead_peers_batch` unit test. THIS test's contract is the all-crashed
    // edge regardless of how the two deaths were batched.
    sigkill_pid(pids[0]);
    sigkill_pid(pids[1]);

    // (4) the supervisor must exit NONZERO within a bounded window (both deaths
    // recorded → all-crashed edge → Err). The exit status is read bare via
    // try_wait (never through a pipe).
    let status = guard.wait_bounded(Duration::from_secs(30)).expect(
        "supervisor did not exit within 30s of ALL workers crashing — the \
         all-crashed edge did not fire (the JOIN loop hung)",
    );
    assert!(
        !status.success(),
        "supervisor MUST exit non-zero when EVERY worker crashed (no survivors is a \
         failure even under peer-loss=continue); got {status:?}"
    );

    // (5) the output must carry the all-crashed marker — the tracing::error! says
    // "every worker process crashed"; the CliError render says "deployment failed".
    let stderr_text = stderr_reader.join().unwrap_or_default();
    let stdout_text = stdout_reader.join().unwrap_or_default();
    let combined = format!("{stdout_text}{stderr_text}");
    assert!(
        combined.contains("every worker process crashed") || combined.contains("crashed ("),
        "supervisor output must name the all-crashed condition; got:\n{combined}"
    );
    assert!(
        combined.contains("deployment failed"),
        "supervisor output must carry the all-crashed failure marker \
         (\"deployment failed\"); got:\n{combined}"
    );

    // (6) no orphans: both (SIGKILLed) worker pids must be reaped and gone.
    let orphan_deadline = Instant::now() + Duration::from_secs(10);
    for pid in pids {
        while process_alive(pid) && Instant::now() < orphan_deadline {
            std::thread::sleep(Duration::from_millis(50));
        }
        assert!(
            !process_alive(pid),
            "worker pid {pid} is still alive after the supervisor exited — an orphan leaked"
        );
    }

    drop(guard);
}

// ===========================================================================
// TEST 7: the REAL `--peer-loss fail` CLI flag
// reproduces the fail-loud crash behavior WITHOUT the env seam.
// ===========================================================================

/// The flag twin of `box_worker_crash_fails_loud_no_orphans`: the fail-loud
/// policy selected via the user-facing `--peer-loss fail` argument (NO
/// `CERULION_MP_PEER_LOSS` in the environment), proving the full flag path —
/// clap parse → `graph_run(peer_loss = Some(Fail))` → `resolve_peer_loss`
/// (flag wins) → the supervisor's fail-loud arm. SIGKILL one worker mid-run →
/// supervisor exits NONZERO with the fail-loud marker; no orphans.
#[test]
#[ignore]
#[serial]
fn box_peer_loss_fail_flag_reproduces_fail_loud() {
    if !cfg!(target_os = "linux") {
        return;
    }

    let sc = scaffold();

    // (1) spawn with the REAL flag; stderr piped for the fail-loud marker
    // (main()'s `eprintln!("Error: {e}")` CliError render). No peer-loss env.
    let mut child = spawn_supervisor(sc, None, true, false, &[], &["--peer-loss", "fail"]);
    let stderr = child.stderr.take();
    let reader = std::thread::spawn(move || {
        let mut s = String::new();
        if let Some(mut e) = stderr {
            let _ = e.read_to_string(&mut s);
        }
        s
    });
    let mut guard = ChildGuard::new(child);
    let supervisor_pid = guard.pid;

    // (2) both workers up + settled (deployment live, JOIN in Normal).
    let pids = wait_for_two_workers(supervisor_pid, Duration::from_secs(30));
    assert!(
        pids.len() >= 2,
        "supervisor never brought up both workers within 30s (saw {pids:?})"
    );
    std::thread::sleep(Duration::from_secs(2));

    // (3) CRASH one worker (SIGKILL a single worker pid — NOT the group).
    let killed = pids[0];
    let survivor = pids[1];
    sigkill_pid(killed);

    // (4) fail-loud: the supervisor must exit NONZERO within a bounded window.
    let status = guard.wait_bounded(Duration::from_secs(30)).expect(
        "supervisor did not exit within 30s of a worker crash under --peer-loss fail — \
         the flag did not reach the supervisor's fail-loud arm",
    );
    assert!(
        !status.success(),
        "supervisor MUST exit non-zero when a worker crashes under --peer-loss fail \
         ({status:?}) — the CLI flag must select the fail-loud policy"
    );

    // (5) the fail-loud marker (same contract as the env-seam twin).
    let stderr_text = reader.join().unwrap_or_default();
    assert!(
        stderr_text.contains("died") && stderr_text.contains("fail-loud"),
        "supervisor stderr must name the fail-loud worker-death contract (\"died\" + \
         \"fail-loud\"); got:\n{stderr_text}"
    );

    // (6) NO ORPHANS.
    let orphan_deadline = Instant::now() + Duration::from_secs(10);
    for pid in [killed, survivor] {
        while process_alive(pid) && Instant::now() < orphan_deadline {
            std::thread::sleep(Duration::from_millis(50));
        }
        assert!(
            !process_alive(pid),
            "worker pid {pid} is still alive after the supervisor exited — the fail-loud \
             path leaked an orphan"
        );
    }

    drop(guard);
}
