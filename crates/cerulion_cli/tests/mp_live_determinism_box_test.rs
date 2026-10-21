// SPDX-License-Identifier: AGPL-3.0-only
//! The live DETERMINISM ORACLE for multi-process `graph run`: does a real
//! deployment, run several times under CPU stress, fire the same sequence as
//! a single-process monolith of the same graph, every time?
//!
//! ## What the arms prove
//!
//! One 3-node chain, `ticker` (Period 1ms, L0) -> `relay` (data-trigger, L1)
//! -> `sink` (data-trigger, L2), split two ways from ONE scaffold build:
//!
//! * two ranks: `g1:[ticker]` / `g2:[relay,sink]`;
//! * three ranks: `g1:[ticker]` / `g2:[relay]` / `g3:[sink]`.
//!
//! Each shape runs under the two execution modes a multi-process run has:
//!
//! * `lockstep`: the child carries `CERULION_EXECUTION_MODE=lockstep`, the
//!   shared-barrier opt-out;
//! * `free_run`: the child carries NO `CERULION_EXECUTION_MODE` at all (it is
//!   removed, so a value exported in the shell cannot leak in), which is the
//!   shipped default.
//!
//! Every arm performs `CER_DET_RUNS` live runs (default 5). Each run spawns the
//! REAL `cerulion graph run <graph>` supervisor while busy-loop threads, one per
//! available CPU, load the host, so the scheduling noise the oracle has to
//! survive is real. The workers' fire traces (the `CERULION_MP_TRACE_DIR` seam)
//! are merged with the PRODUCTION [`merge_partition_traces`] and compared, on
//! `(node_id, step, global_level)` over steps `[0, k]` (`k` = the last step
//! every rank reached), against two oracles:
//!
//! * a single-process MONOLITH of the same graph stepped in THIS process (the
//!   replay = live anchor, Principle #7);
//! * a HAND oracle, `[ticker@L0, relay@L1, sink@L2]` at every step of
//!   `[0, min(40, k)]`, so the compare is never only a run against a run.
//!
//! Each run prints ONE machine-readable verdict line:
//!
//! ```text
//! DET_ORACLE mode=<lockstep|free_run> ranks=<2|3> run=<i> verdict=<equal|diverged> first_diff=<idx|-> k=<k>
//! ```
//!
//! `run` counts from 1. `first_diff` is the index into the merged
//! `(node_id, step, global_level)` sequence of the first entry that disagrees
//! with either oracle; a diverged run also prints a `DET_ORACLE_DIFF` line
//! naming which oracle disagreed and the two tuples at that index (`-` where a
//! sequence ended early). After all runs the arm prints
//! `DET_ORACLE_SUMMARY mode= ranks= judged=<n> equal=<n> diverged=<n>` and then
//! asserts every run was `equal`.
//!
//! Before a run is judged it must prove it executed the mode its line names:
//! each worker logs one `build_path=` line (`Lockstep`, or `FreeRunTraced` /
//! `FreeRunLive`), and the run needs exactly one per rank of the expected
//! family and none of the other. Lockstep runs must also stop within 2 steps
//! of each other across ranks (the barrier bound).
//!
//! ## Reading a red arm: harness failure vs divergence
//!
//! Every failure that is NOT a determinism verdict (bring-up, an early
//! supervisor exit, the drain, the mode witness, a missing trace, the
//! liveness floor, the lockstep drift bound, an orphan) panics with a message
//! starting `DET_HARNESS:`, and stops the arm before its summary line. A
//! divergence never panics mid-arm: it is a `verdict=diverged` line, and the
//! arm fails only at its closing assertion, after the summary line. So an arm
//! that failed by DIVERGING (the expected red of the free-run arms) shows
//! exactly `CER_DET_RUNS` `DET_ORACLE` lines, a `DET_ORACLE_SUMMARY` line and
//! no `DET_HARNESS`; anything else is a broken harness, not a measurement.
//!
//! ## Free run is expected to fail these arms
//!
//! A free-run rank steps on its own wall-following clock, so the step number
//! a cross-process input lands on depends on scheduling, and the merged trace
//! is not the monolith's. The two `free_run` arms are therefore expected to
//! fail on a current build; that failure is the measurement. They turn green only when free-run
//! ranks agree on the step every cross-process input belongs to (tagged free
//! run). Do not weaken their assertion to make them pass.
//!
//! ## Scope and how to run
//!
//! Every live arm is `#[ignore]` + `#[serial]` and returns early off Linux: the
//! lockstep barrier needs a real `/dev/shm` `MAP_SHARED` page shared between
//! processes, and the terminal Ctrl-C emulation needs POSIX process-group
//! signals. The one arm that is not ignored is a pure check of the verdict
//! function against hand-written vectors. Run the oracle on Linux:
//!
//! ```bash
//! CER_DET_RUNS=5 cargo test -p cerulion_cli --test mp_live_determinism_box_test \
//!     -- --ignored --nocapture --test-threads=1
//! ```
//!
//! The scaffold compiles a 3-node cdylib workspace ONCE per test process (the
//! dominant cost, minutes on a cold cargo cache) and every arm reuses it.
//!
//! Each run owns its environment: its own `CERULION_HOME`, a pinned
//! `RUST_LOG`, the run-shaping `CERULION_*` knobs (peer-loss policy, drain
//! window, flashback and barrier tuning) removed, and SIGINT reset to its
//! default disposition in the child, so a test started as a background job of
//! a non-interactive shell (where SIGINT arrives ignored) still drains on
//! Ctrl-C.
//!
//! Hang and orphan safety: the supervisor is spawned through
//! `mp_support::ChildGuard::spawn_group_leader`, whose teardown signals the
//! whole process group; every wait is bounded; each run asserts the guard's
//! orphan verdict; the stress threads stop in a `Drop` so a panicking run
//! cannot leave them spinning. No fake data (Principle #13): REAL
//! `#[cerulion_node]` cdylibs over REAL iceoryx2 shared memory.

#![cfg(not(target_os = "macos"))]
// WAIVED WHOLE on macOS: upstream iceoryx2 0.10.0 defect 2034. Arms here spawn a
// `cerulion` supervisor child that loads plugin nodes, and load node cdylibs into
// the test process for the monolith oracle; on macOS such a process cannot create
// any further event resource. The mechanism, the derivation that selects this
// file, and the coverage this costs are stated once in
// `cerulion_core/tests/upstream_waivers_test.rs`. Runs normally on Linux.

mod mp_support;

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use cerulion_cli_engine::node_cmd::NodeCreateOptions;
use cerulion_cli_engine::node_cmd::NodeLanguage;
use cerulion_cli_engine::{node_cmd, workspace};
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::{
    merge_partition_traces, DylibNodeEntry, GraphRuntime, MacroPolicy, NodeEntry, ProcessTrace,
    TraceEntry, TransportConfig, TransportManager, VirtualClock,
};
use mp_support::{dylib_file, ChildGuard};
use serde::Deserialize;
use serial_test::serial;

/// The graph topic prefix: topics resolve to `/detfree/<node>/<out>`.
const GRAPH_PREFIX: &str = "detfree";
/// The message schema every port carries.
const SCHEMA: &str = "geometry_msgs/Vector3";
/// The env var naming how many live runs each arm performs.
const RUNS_ENV: &str = "CER_DET_RUNS";
/// Runs per arm when [`RUNS_ENV`] is unset.
const DEFAULT_RUNS: usize = 5;
/// How long a deployment runs once every worker is up. At the 1ms quantum
/// that is thousands of steps, far past the [`LIVENESS_FLOOR`].
const RUN_WINDOW: Duration = Duration::from_secs(4);
/// Fewer compared steps than this means the deployment barely advanced, and
/// a verdict over so short a window would say nothing.
const LIVENESS_FLOOR: u64 = 50;
/// The hand oracle's window is `[0, min(HAND_WINDOW, k)]`, the window
/// `mp_supervisor_box_test`'s firewall arm samples.
const HAND_WINDOW: u64 = 40;
/// The monolith is stepped at least this far, then reused by every run of
/// the arm whose `k` it covers (stepping a polled monolith further never
/// changes an earlier step, so a longer trace is a superset of a shorter one).
const MONOLITH_MIN_STEPS: u64 = 20_000;

// ===========================================================================
// Shapes and modes.
// ===========================================================================

/// One process-group split of the chain. Declaration order of `groups` is the
/// rank order (`g1` = rank 0).
struct Shape {
    /// The graph name, also the YAML file stem and the graph identity.
    graph: &'static str,
    groups: &'static [(&'static str, &'static [&'static str])],
}

impl Shape {
    fn ranks(&self) -> usize {
        self.groups.len()
    }
}

const TWO_RANKS: Shape = Shape {
    graph: "detfree_ranks2",
    groups: &[("g1", &["ticker"]), ("g2", &["relay", "sink"])],
};

const THREE_RANKS: Shape = Shape {
    graph: "detfree_ranks3",
    groups: &[("g1", &["ticker"]), ("g2", &["relay"]), ("g3", &["sink"])],
};

/// The execution mode a run's supervisor child is spawned under.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mode {
    /// `CERULION_EXECUTION_MODE=lockstep` in the child's environment.
    Lockstep,
    /// `CERULION_EXECUTION_MODE` removed from the child's environment.
    FreeRun,
}

impl Mode {
    /// The spelling the verdict line carries (the plan enum's serde names).
    fn label(self) -> &'static str {
        match self {
            Mode::Lockstep => "lockstep",
            Mode::FreeRun => "free_run",
        }
    }
}

// ===========================================================================
// Shared scaffold: built ONCE per test process, memoized.
// ===========================================================================

/// The one-per-process scaffolded workspace holding both shapes' graphs. The
/// `TempDir` is stored so the built cdylibs and YAMLs survive the process.
struct Scaffold {
    /// Kept alive for the process lifetime so `ws_root` stays valid.
    _tmp: tempfile::TempDir,
    ws_root: PathBuf,
    /// Graph name -> the config written to `graphs/<name>.yaml`, reused as the
    /// monolith oracle leg's config.
    configs: HashMap<&'static str, GraphConfig>,
}

fn scaffold() -> &'static Scaffold {
    static SCAFFOLD: OnceLock<Scaffold> = OnceLock::new();
    SCAFFOLD.get_or_init(build_scaffold)
}

/// The chain's graph config split per `shape`. Serializing this exact struct
/// into the YAML the supervisor parses makes the two parse-compatible by
/// construction.
fn build_graph_config(shape: &Shape) -> GraphConfig {
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
        identity: shape.graph.to_string(),
        prefix: GRAPH_PREFIX.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "ticker".to_string(),
                node_type: "ticker".to_string(),
                inputs: vec![],
                outputs: vec![out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "relay".to_string(),
                node_type: "relay".to_string(),
                inputs: vec![input("ticker/out")],
                outputs: vec![out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "sink".to_string(),
                node_type: "sink".to_string(),
                inputs: vec![input("relay/out")],
                outputs: vec![],
            },
        ],
        multi_publisher_topics: vec![],
        // Insertion order defines the ranks (g1 = 0, g2 = 1, g3 = 2).
        process_groups: shape
            .groups
            .iter()
            .map(|(g, members)| {
                (
                    (*g).to_string(),
                    members.iter().map(|m| (*m).to_string()).collect(),
                )
            })
            .collect(),
        process_group_order: vec![],
    }
}

/// Replace the scaffold's `// TODO: populate out (...)` line in node
/// `node`'s source with `stmt`, so the node writes (and therefore publishes) its
/// output every tick. Panics when the marker is missing: a changed scaffold
/// would otherwise build a chain that carries nothing.
fn write_output(nodes_dir: &Path, node: &str, stmt: &str) {
    let path = nodes_dir.join(node).join("src").join("lib.rs");
    let src = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("DET_HARNESS: read {}: {e}", path.display()));
    let marker = "// TODO: populate out (Vector3)";
    assert_eq!(
        src.matches(marker).count(),
        1,
        "DET_HARNESS: {} carries the scaffold's output marker exactly once",
        path.display()
    );
    std::fs::write(&path, src.replace(marker, stmt))
        .unwrap_or_else(|e| panic!("DET_HARNESS: write {}: {e}", path.display()));
}

fn build_scaffold() -> Scaffold {
    let tmp = tempfile::tempdir().expect("DET_HARNESS: temp dir for scaffold workspace");
    let ws =
        workspace::workspace_create(tmp.path(), "det_ws").expect("DET_HARNESS: workspace_create");
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
            language: NodeLanguage::Rust,
        },
    )
    .expect("DET_HARNESS: node_create ticker");

    // relay: EXPLICIT data-trigger on `inp` (policy plus the matching
    // `#[input(trigger)]` field), not the one-input defaulting warn.
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
            language: NodeLanguage::Rust,
        },
    )
    .expect("DET_HARNESS: node_create relay");

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
            language: NodeLanguage::Rust,
        },
    )
    .expect("DET_HARNESS: node_create sink");

    // Debug builds (the workers' default freshest-debug cdylib resolution).
    // The scaffold leaves every output unwritten (a TODO comment), and an
    // output nobody writes is never published. Make the chain carry data: the
    // ticker publishes its tick count, the relay forwards what it read.
    write_output(
        &ws.nodes_dir,
        "ticker",
        "self.out.x = f64::from(self.tick_count);",
    );
    write_output(&ws.nodes_dir, "relay", "self.out.x = self.inp.x;");
    node_cmd::node_build(&ws.root, "ticker", false).expect("DET_HARNESS: node_build ticker");
    node_cmd::node_build(&ws.root, "relay", false).expect("DET_HARNESS: node_build relay");
    node_cmd::node_build(&ws.root, "sink", false).expect("DET_HARNESS: node_build sink");

    let mut configs = HashMap::new();
    for shape in [&TWO_RANKS, &THREE_RANKS] {
        let config = build_graph_config(shape);
        let yaml =
            serde_yaml::to_string(&config).expect("DET_HARNESS: serialize graph config to YAML");
        std::fs::write(ws.graphs_dir.join(format!("{}.yaml", shape.graph)), yaml)
            .expect("DET_HARNESS: write graph YAML");
        configs.insert(shape.graph, config);
    }

    Scaffold {
        _tmp: tmp,
        ws_root: ws.root,
        configs,
    }
}

// ===========================================================================
// Trace wire format: the worker dumps `{node_id, step, fire_time_ns,
// global_level}`; a local twin is deserialized and turned back into
// `TraceEntry`s.
// ===========================================================================

#[derive(Deserialize)]
struct FlatEntry {
    node_id: String,
    step: u64,
    fire_time_ns: u64,
    global_level: usize,
}

/// Read and parse a worker's `trace_{group}.json`. Its ABSENCE means the trace
/// seam or the worker broke: fail loud. The dump file names are the SANITIZED
/// group names (`[A-Za-z0-9_]`); `g1`..`g3` sanitize to themselves.
fn read_worker_trace(dir: &Path, group: &str) -> Vec<FlatEntry> {
    let path = dir.join(format!("trace_{group}.json"));
    let json = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "DET_HARNESS: worker trace `{}` missing or unreadable ({e}): the CERULION_MP_TRACE_DIR \
             seam or the worker itself broke",
            path.display()
        )
    });
    serde_json::from_str(&json).unwrap_or_else(|e| {
        panic!(
            "DET_HARNESS: worker trace `{}` is not valid JSON: {e}",
            path.display()
        )
    })
}

/// Rebuild `TraceEntry`s from the wire entries. `duration_ns` and `discarded`
/// are excluded from `TraceEntry`'s equality, so their placeholder values
/// cannot affect any comparison.
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
// Projections, oracles and the verdict.
// ===========================================================================

/// One compared fire: `(node_id, step, global_level)`.
type Fire = (String, u64, usize);

fn max_step(entries: &[TraceEntry]) -> u64 {
    entries.iter().map(|e| e.step).max().unwrap_or(0)
}

/// The comparison sequence for steps in `[lo, hi]`. `fire_time_ns` is
/// EXCLUDED: the polled monolith advances its clock by the fixed 1ms step while
/// live workers advance by their own clocks, so the stamps legitimately differ;
/// `step` and `global_level` are the replay-deterministic keys.
fn seq_tuples(entries: &[TraceEntry], lo: u64, hi: u64) -> Vec<Fire> {
    entries
        .iter()
        .filter(|e| e.step >= lo && e.step <= hi)
        .map(|e| (e.node_id.to_string(), e.step, e.global_level))
        .collect()
}

/// The hand oracle: `[ticker@L0, relay@L1, sink@L2]` at every step of
/// `[0, hi]`. Every step fires all three: the GO gate connects every
/// subscriber before the first publish, and a data-trigger consumer fires in
/// the same step its upstream published (the within-level collapse).
fn hand_oracle(hi: u64) -> Vec<Fire> {
    (0..=hi)
        .flat_map(|s| {
            [
                ("ticker".to_string(), s, 0),
                ("relay".to_string(), s, 1),
                ("sink".to_string(), s, 2),
            ]
        })
        .collect()
}

/// The first index at which `got` and `want` disagree, with the entry each
/// carries there (`None` where that sequence already ended). `None` overall
/// means the two are equal, length included.
fn first_divergence(got: &[Fire], want: &[Fire]) -> Option<(usize, Option<Fire>, Option<Fire>)> {
    let n = got.len().max(want.len());
    (0..n).find_map(|i| {
        let g = got.get(i);
        let w = want.get(i);
        (g != w).then(|| (i, g.cloned(), w.cloned()))
    })
}

/// A run's verdict against both oracles.
#[derive(Debug, PartialEq, Eq)]
enum Verdict {
    Equal,
    Diverged {
        /// Index into the merged sequence over `[0, k]`.
        index: usize,
        /// Which oracle disagreed first: `monolith` or `hand`.
        against: &'static str,
        got: Option<Fire>,
        want: Option<Fire>,
    },
}

/// Judge one run. `merged` is the merged sequence over `[0, k]`, `monolith`
/// the monolith's over the same window, `hand` the hand oracle over
/// `[0, min(HAND_WINDOW, k)]`. Because both sequences are ordered by step, the
/// merged sequence over the hand window is a PREFIX of `merged`, so one index
/// space serves both oracles; the smaller index wins, and on a tie the
/// monolith is named (it is the stronger claim).
fn judge(merged: &[Fire], monolith: &[Fire], hand: &[Fire]) -> Verdict {
    let hand_hi = hand.last().map(|f| f.1);
    let merged_hand: Vec<Fire> = match hand_hi {
        Some(hi) => merged.iter().filter(|f| f.1 <= hi).cloned().collect(),
        None => Vec::new(),
    };
    let vs_mono = first_divergence(merged, monolith);
    let vs_hand = first_divergence(&merged_hand, hand);
    let pick = match (vs_mono, vs_hand) {
        (None, None) => return Verdict::Equal,
        (Some(m), None) => ("monolith", m),
        (None, Some(h)) => ("hand", h),
        (Some(m), Some(h)) => {
            if h.0 < m.0 {
                ("hand", h)
            } else {
                ("monolith", m)
            }
        }
    };
    let (against, (index, got, want)) = pick;
    Verdict::Diverged {
        index,
        against,
        got,
        want,
    }
}

fn fmt_fire(f: &Option<Fire>) -> String {
    match f {
        Some((node, step, level)) => format!("{node}@{step}/L{level}"),
        None => "-".to_string(),
    }
}

/// The machine-readable verdict line (and, for a divergence, its detail line).
fn verdict_lines(mode: Mode, ranks: usize, run: usize, k: u64, v: &Verdict) -> Vec<String> {
    let head = |verdict: &str, first: &str| {
        format!(
            "DET_ORACLE mode={} ranks={ranks} run={run} verdict={verdict} first_diff={first} k={k}",
            mode.label()
        )
    };
    match v {
        Verdict::Equal => vec![head("equal", "-")],
        Verdict::Diverged {
            index,
            against,
            got,
            want,
        } => vec![
            head("diverged", &index.to_string()),
            format!(
                "DET_ORACLE_DIFF mode={} ranks={ranks} run={run} against={against} index={index} \
                 got={} want={}",
                mode.label(),
                fmt_fire(got),
                fmt_fire(want)
            ),
        ],
    }
}

/// The number of runs per arm, from [`RUNS_ENV`]. A value that does not parse,
/// or zero, is refused: an arm that ran nothing would pass having judged
/// nothing.
fn runs_per_arm() -> usize {
    match std::env::var(RUNS_ENV) {
        Err(std::env::VarError::NotPresent) => DEFAULT_RUNS,
        Err(e) => panic!("DET_HARNESS: {RUNS_ENV} is set but unreadable: {e}"),
        Ok(raw) => match raw.trim().parse::<usize>() {
            Ok(0) => panic!("DET_HARNESS: {RUNS_ENV}=0 would judge zero runs; set at least 1"),
            Ok(n) => n,
            Err(e) => panic!("DET_HARNESS: {RUNS_ENV}={raw:?} is not a run count: {e}"),
        },
    }
}

// ===========================================================================
// The MONOLITH oracle leg: a single-process build of the same graph, stepped
// in THIS process.
// ===========================================================================

/// Mirror of `graph_cmd::cdylib_target_base`: honor `CARGO_TARGET_DIR` (absolute
/// or workspace-relative) exactly as cargo and `node_build` do, else
/// `<ws>/target`.
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

/// Build the monolith of `config` on an ISOLATED per-test transport (a
/// distinct shared-memory root from the default namespace the workers used),
/// load the three debug cdylibs, step `steps` times at 1ms (so the Period
/// ticker fires once per step), and return the full trace.
fn run_monolith(sc: &Scaffold, config: &GraphConfig, steps: u64) -> Vec<TraceEntry> {
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "det_monolith".to_string(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 16,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("DET_HARNESS: init isolated monolith test transport");

    let debug_dir = cdylib_target_base(&sc.ws_root).join("debug");
    let mut factories: indexmap::IndexMap<String, Box<dyn NodeEntry>> = indexmap::IndexMap::new();
    for id in ["ticker", "relay", "sink"] {
        let path = debug_dir.join(dylib_file(id));
        let entry = DylibNodeEntry::load(&path)
            .unwrap_or_else(|e| panic!("DET_HARNESS: load cdylib `{}`: {e}", path.display()));
        factories.insert(id.to_string(), Box::new(entry) as Box<dyn NodeEntry>);
    }

    let mut runtime = GraphRuntime::build_with_schema_hashes(
        config.clone(),
        factories,
        &mgr,
        Arc::new(VirtualClock::new()),
        None,
    )
    .expect("DET_HARNESS: build monolith runtime");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(1));
    }
    let trace = runtime.trace().to_vec();
    runtime.shutdown();
    trace
}

/// The monolith trace of `shape` covering at least `[0, k]`, built once per
/// shape and rebuilt longer only when a run reaches past it.
fn monolith_covering(sc: &Scaffold, shape: &Shape, k: u64) -> Vec<TraceEntry> {
    static CACHE: OnceLock<Mutex<HashMap<&'static str, Vec<TraceEntry>>>> = OnceLock::new();
    let cache = CACHE.get_or_init(|| Mutex::new(HashMap::new()));
    let mut cache = cache.lock().unwrap_or_else(|p| p.into_inner());
    let covers = cache
        .get(shape.graph)
        .is_some_and(|t| max_step(t) >= k.saturating_add(1));
    if !covers {
        let config = sc
            .configs
            .get(shape.graph)
            .unwrap_or_else(|| panic!("DET_HARNESS: scaffold has no config for `{}`", shape.graph));
        let steps = MONOLITH_MIN_STEPS.max(k.saturating_add(2));
        cache.insert(shape.graph, run_monolith(sc, config, steps));
    }
    cache[shape.graph].clone()
}

// ===========================================================================
// CPU stress: busy-loop threads in THIS process, one per available CPU,
// stopped (and joined) when the guard drops.
// ===========================================================================

struct Stress {
    stop: Arc<AtomicBool>,
    threads: Vec<std::thread::JoinHandle<()>>,
}

impl Stress {
    fn start() -> Self {
        let n = std::thread::available_parallelism().map_or(1, |n| n.get());
        let stop = Arc::new(AtomicBool::new(false));
        let threads = (0..n)
            .map(|_| {
                let stop = Arc::clone(&stop);
                std::thread::spawn(move || {
                    let mut x: u64 = 0x9E37_79B9_7F4A_7C15;
                    while !stop.load(Ordering::Relaxed) {
                        // xorshift: real ALU work the optimizer cannot drop.
                        x ^= x << 13;
                        x ^= x >> 7;
                        x ^= x << 17;
                        std::hint::black_box(x);
                    }
                })
            })
            .collect();
        Self { stop, threads }
    }
}

impl Drop for Stress {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
    }
}

// ===========================================================================
// Spawn and signal helpers.
// ===========================================================================

/// SIGINT the whole process GROUP led by `pid` (negative pid): exactly what a
/// terminal Ctrl-C does, reaching the supervisor and every worker.
fn sigint_group(pid: u32) {
    assert!(
        pid > 1,
        "DET_HARNESS: refusing to signal process group {pid}: 0 or 1 is a failed lookup, \
         not a target"
    );
    // SAFETY: kill(2) with a negated valid pid and a valid signal; no memory is touched.
    let rc = unsafe { libc::kill(-(pid as libc::pid_t), libc::SIGINT) };
    assert_eq!(
        rc,
        0,
        "DET_HARNESS: kill(-{pid}, SIGINT) did not land: {}",
        std::io::Error::last_os_error()
    );
}

/// The run-shaping `CERULION_*` knobs a developer shell may export, removed
/// from the child so a run's shape is this file's alone: the peer-loss policy,
/// the drain window, the flashback and barrier tuning families. Exact names
/// first, then every variable carrying one of the family prefixes.
const SCRUBBED_ENV: &[&str] = &["CERULION_MP_PEER_LOSS", "CERULION_MP_DRAIN_MS"];
const SCRUBBED_ENV_PREFIXES: &[&str] = &["CERULION_FLASHBACK", "CERULION_BARRIER_"];

/// Where one run's supervisor output and owned state live.
struct RunFiles {
    /// The run's own `CERULION_HOME` (its run directory lands under it).
    home: PathBuf,
    stdout: PathBuf,
    stderr: PathBuf,
}

/// Spawn the REAL `cerulion graph run <graph>` as a process-group leader with
/// the worker trace seam pointed at `trace_dir`, under `mode`, its output
/// captured to files in `files`. The mode is HERMETIC in both directions:
/// pinned for lockstep, REMOVED for free run, so the shell's own export can
/// never pick the contract.
fn spawn_supervisor(
    sc: &Scaffold,
    shape: &Shape,
    mode: Mode,
    trace_dir: &Path,
    files: &RunFiles,
) -> ChildGuard {
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(["graph", "run", shape.graph])
        .current_dir(&sc.ws_root)
        // Hermetic: no scouting session or gateway (the kill switch keeps a
        // real-clock run local-only).
        .env("CERULION_NETWORK", "off")
        .env("CERULION_MP_TRACE_DIR", trace_dir)
        .env("CERULION_HOME", &files.home)
        // Pinned so an inherited RUST_LOG cannot hide the per-worker
        // `build_path=` line the mode witness counts (an INFO line).
        .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info");
    for name in SCRUBBED_ENV {
        cmd.env_remove(name);
    }
    for (name, _) in std::env::vars_os() {
        if let Some(n) = name.to_str() {
            if SCRUBBED_ENV_PREFIXES.iter().any(|p| n.starts_with(p)) {
                cmd.env_remove(n);
            }
        }
    }
    match mode {
        Mode::Lockstep => {
            cmd.env("CERULION_EXECUTION_MODE", "lockstep");
        }
        Mode::FreeRun => {
            cmd.env_remove("CERULION_EXECUTION_MODE");
        }
    }
    let out = std::fs::File::create(&files.stdout)
        .unwrap_or_else(|e| panic!("DET_HARNESS: create {}: {e}", files.stdout.display()));
    let err = std::fs::File::create(&files.stderr)
        .unwrap_or_else(|e| panic!("DET_HARNESS: create {}: {e}", files.stderr.display()));
    cmd.stdout(Stdio::from(out)).stderr(Stdio::from(err));
    // A test launched as a background job of a non-interactive shell (a
    // `nohup ... &` launcher) inherits SIGINT IGNORED, and an ignored
    // disposition survives exec, so the terminal Ctrl-C below would reach a
    // child that cannot see it. Reset it to the default in the child before
    // exec.
    {
        use std::os::unix::process::CommandExt;
        // SAFETY: `pre_exec` runs between fork and exec, where only
        // async-signal-safe calls are allowed; `signal(2)` is one, and the
        // closure touches no memory and allocates nothing.
        unsafe {
            cmd.pre_exec(|| {
                if libc::signal(libc::SIGINT, libc::SIG_DFL) == libc::SIG_ERR {
                    return Err(std::io::Error::last_os_error());
                }
                Ok(())
            });
        }
    }
    ChildGuard::spawn_group_leader(&mut cmd).unwrap_or_else(|e| {
        panic!(
            "DET_HARNESS: spawn `cerulion graph run {}`: {e}",
            shape.graph
        )
    })
}

/// BOUNDED poll until the guard has noted `ranks` workers. Returns how many
/// were noted at the deadline.
fn wait_for_workers(guard: &mut ChildGuard, ranks: usize, deadline: Duration) -> usize {
    let start = Instant::now();
    loop {
        let seen = guard.note_workers().len();
        if seen >= ranks || start.elapsed() >= deadline {
            return seen;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Strip ANSI SGR escapes. LOAD-BEARING: `tracing`'s default formatter wraps a
/// field's name and its `=` in separate escape sequences, so the rendered
/// `build_path=Lockstep` is not a substring of the raw capture.
fn strip_ansi(s: &str) -> String {
    let bytes = s.as_bytes();
    let mut out = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == 0x1b && bytes.get(i + 1) == Some(&b'[') {
            i += 2;
            while i < bytes.len() && !(0x40..=0x7e).contains(&bytes[i]) {
                i += 1;
            }
            i += 1;
            continue;
        }
        out.push(bytes[i]);
        i += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

/// The POSITIVE witness that a run executed the mode its verdict line names:
/// every worker logs one `build_path=<path>` INFO line (`Lockstep`,
/// `FreeRunTraced` or `FreeRunLive`). `None` when there are exactly `ranks`
/// lines of the expected family and none of the other; otherwise the reason.
fn mode_witness_mismatch(log: &str, mode: Mode, ranks: usize) -> Option<String> {
    let lockstep = log.matches("build_path=Lockstep").count();
    let free_run = log.matches("build_path=FreeRun").count();
    let (want, other) = match mode {
        Mode::Lockstep => (lockstep, free_run),
        Mode::FreeRun => (free_run, lockstep),
    };
    (want != ranks || other != 0).then(|| {
        format!(
            "expected {ranks} `build_path=` worker line(s) of the {} family and none of \
             the other; found Lockstep={lockstep} FreeRun*={free_run}",
            mode.label()
        )
    })
}

// ===========================================================================
// One live run and one arm.
// ===========================================================================

/// One live run of `shape` under `mode` with the host under stress. Returns
/// `k` and the verdict. Every harness failure (bring-up, an early supervisor
/// exit, the drain, the mode witness, a missing trace, the liveness floor, the
/// lockstep drift bound, an orphan) panics with a `DET_HARNESS:` prefix,
/// because it is not a verdict about determinism.
fn live_run(sc: &Scaffold, shape: &Shape, mode: Mode) -> (u64, Verdict) {
    let run_tmp = tempfile::tempdir().expect("DET_HARNESS: temp dir for one run");
    let trace_dir = run_tmp.path().join("traces");
    let files = RunFiles {
        home: run_tmp.path().join("cerhome"),
        stdout: run_tmp.path().join("supervisor.stdout"),
        stderr: run_tmp.path().join("supervisor.stderr"),
    };
    std::fs::create_dir_all(&trace_dir).expect("DET_HARNESS: create the trace dir");
    std::fs::create_dir_all(&files.home).expect("DET_HARNESS: create the run's CERULION_HOME");
    let ranks = shape.ranks();
    let what = format!("{} under {}", shape.graph, mode.label());
    let logs = || {
        format!(
            "stdout:\n{}\nstderr:\n{}",
            mp_support::read_file(&files.stdout),
            mp_support::read_file(&files.stderr)
        )
    };

    let stress = Stress::start();
    let mut guard = spawn_supervisor(sc, shape, mode, &trace_dir, &files);
    let supervisor_pid = guard.id();

    // Bring-up runs under the stress too, so the bound is wider than the
    // unloaded suites' 30s.
    let seen = wait_for_workers(&mut guard, ranks, Duration::from_secs(60));
    assert!(
        seen >= ranks,
        "DET_HARNESS: {what}: the supervisor brought up {seen} of {ranks} workers within 60s \
         (a build, transport-init or barrier-open failure)\n{}",
        logs()
    );
    std::thread::sleep(RUN_WINDOW);

    // REAP FIRST, then signal: a supervisor that died during the window is a
    // zombie `kill` still lands on, so the Ctrl-C below would "succeed" on a
    // run that was already over.
    let early = guard
        .try_wait_noting()
        .unwrap_or_else(|e| panic!("DET_HARNESS: {what}: polling the supervisor failed: {e}"));
    if let Some(status) = early {
        panic!(
            "DET_HARNESS: {what}: the supervisor exited on its own ({status:?}) before the \
             Ctrl-C\n{}",
            logs()
        );
    }
    sigint_group(supervisor_pid);
    let status = guard
        .wait_bounded(Duration::from_secs(40))
        .unwrap_or_else(|| {
            panic!(
                "DET_HARNESS: {what}: the supervisor did not exit within 40s of Ctrl-C\n{}",
                logs()
            )
        });
    drop(stress);
    assert!(
        status.success(),
        "DET_HARNESS: {what}: the supervisor must exit 0 after Ctrl-C; got {status:?}\n{}",
        logs()
    );
    if let mp_support::OrphanVerdict::Leaked(pids) = guard.orphan_verdict() {
        panic!("DET_HARNESS: {what}: worker pids {pids:?} survived the supervisor's exit");
    }

    let log = strip_ansi(&logs());
    if let Some(why) = mode_witness_mismatch(&log, mode, ranks) {
        panic!(
            "DET_HARNESS: {what}: the run did not execute the mode it is labeled with: \
             {why}\n{log}"
        );
    }

    let traces: Vec<Vec<TraceEntry>> = shape
        .groups
        .iter()
        .map(|(g, _)| {
            let flats = read_worker_trace(&trace_dir, g);
            assert!(
                !flats.is_empty(),
                "DET_HARNESS: {what}: group {g} dumped an empty trace\n{log}"
            );
            reconstruct(&flats)
        })
        .collect();

    let last_steps: Vec<u64> = traces.iter().map(|t| max_step(t)).collect();
    // The last step EVERY rank reached.
    let k = last_steps.iter().copied().min().unwrap_or(0);
    let drift = last_steps.iter().copied().max().unwrap_or(0) - k;
    // Lockstep only: the barrier gates level advance, so the ranks stop within
    // a step or two of each other (the Ctrl-C lands at step boundaries). Free
    // run has no such bound; its alignment is exactly what the verdict judges.
    if mode == Mode::Lockstep {
        assert!(
            drift <= 2,
            "DET_HARNESS: {what}: per-rank last steps {last_steps:?} drift by {drift} (> 2): \
             the cross-process barrier is not gating level advance in lockstep"
        );
    }
    assert!(
        k >= LIVENESS_FLOOR,
        "DET_HARNESS: {what}: only {k} steps completed on every rank (floor \
         {LIVENESS_FLOOR}); the deployment barely advanced, so no verdict is possible"
    );

    let processes: Vec<ProcessTrace<'_>> = traces
        .iter()
        .enumerate()
        .map(|(rank, trace)| ProcessTrace { rank, trace })
        .collect();
    let merged = merge_partition_traces(&processes).expect("DET_HARNESS: merge distinct ranks");
    let monolith = monolith_covering(sc, shape, k);

    let merged_seq = seq_tuples(&merged, 0, k);
    let mono_seq = seq_tuples(&monolith, 0, k);
    let hand = hand_oracle(HAND_WINDOW.min(k));
    let verdict = judge(&merged_seq, &mono_seq, &hand);

    drop(guard);
    drop(run_tmp);
    (k, verdict)
}

/// The arm's closing summary line: how many runs were judged, and how they
/// split.
fn summary_line(mode: Mode, ranks: usize, judged: usize, diverged: usize) -> String {
    format!(
        "DET_ORACLE_SUMMARY mode={} ranks={ranks} judged={judged} equal={} diverged={diverged}",
        mode.label(),
        judged - diverged
    )
}

/// Run the arm: `CER_DET_RUNS` live runs, one verdict line each, a summary
/// line, then assert every run was equal.
fn determinism_arm(shape: &Shape, mode: Mode) {
    let sc = scaffold();
    let runs = runs_per_arm();
    let mut diverged = Vec::new();
    for run in 1..=runs {
        let (k, verdict) = live_run(sc, shape, mode);
        for line in verdict_lines(mode, shape.ranks(), run, k, &verdict) {
            println!("{line}");
        }
        if verdict != Verdict::Equal {
            diverged.push((run, verdict));
        }
    }
    println!(
        "{}",
        summary_line(mode, shape.ranks(), runs, diverged.len())
    );
    assert!(
        diverged.is_empty(),
        "{} of {runs} live run(s) of {} under {} diverged from the monolith or the hand \
         oracle (see the DET_ORACLE lines): {diverged:?}",
        diverged.len(),
        shape.graph,
        mode.label()
    );
}

// ===========================================================================
// The arms.
// ===========================================================================

#[test]
#[ignore]
#[serial]
fn box_lockstep_two_ranks_matches_the_monolith_on_every_stressed_run() {
    if !cfg!(target_os = "linux") {
        // Cross-process MAP_SHARED barrier + process-group signals are Linux-only.
        return;
    }
    determinism_arm(&TWO_RANKS, Mode::Lockstep);
}

#[test]
#[ignore]
#[serial]
fn box_lockstep_three_ranks_matches_the_monolith_on_every_stressed_run() {
    if !cfg!(target_os = "linux") {
        return;
    }
    determinism_arm(&THREE_RANKS, Mode::Lockstep);
}

/// Expected to FAIL on a current build: free-run ranks do not agree on the
/// step a cross-process input lands on. Turns green with tagged free run.
#[test]
#[ignore]
#[serial]
fn box_free_run_two_ranks_matches_the_monolith_on_every_stressed_run() {
    if !cfg!(target_os = "linux") {
        return;
    }
    determinism_arm(&TWO_RANKS, Mode::FreeRun);
}

/// Expected to FAIL on a current build, for the same reason as the two-rank
/// free-run arm.
#[test]
#[ignore]
#[serial]
fn box_free_run_three_ranks_matches_the_monolith_on_every_stressed_run() {
    if !cfg!(target_os = "linux") {
        return;
    }
    determinism_arm(&THREE_RANKS, Mode::FreeRun);
}

/// The verdict function against hand-written vectors: equal sequences, a
/// mid-sequence disagreement, a merged trace that ends early, one that runs
/// long, a hand-oracle disagreement the monolith shares (both legs broken the
/// same way), and the tie rule.
#[test]
fn the_verdict_names_the_first_entry_either_oracle_disagrees_with() {
    let f = |n: &str, s: u64, l: usize| (n.to_string(), s, l);
    let good = vec![
        f("ticker", 0, 0),
        f("relay", 0, 1),
        f("sink", 0, 2),
        f("ticker", 1, 0),
        f("relay", 1, 1),
        f("sink", 1, 2),
    ];
    let hand = hand_oracle(1);
    assert_eq!(
        hand, good,
        "the hand oracle over [0, 1] is the six fires above"
    );

    assert_eq!(judge(&good, &good, &hand), Verdict::Equal);

    // relay and sink landed one step late on the merged side. Against the
    // monolith index 4 disagrees (relay@2 where relay@1 belongs); against the
    // hand window [0, 1] the merged prefix ENDS at index 4. Both first
    // disagree at 4, and the tie names the monolith.
    let late = vec![
        f("ticker", 0, 0),
        f("relay", 0, 1),
        f("sink", 0, 2),
        f("ticker", 1, 0),
        f("relay", 2, 1),
        f("sink", 2, 2),
    ];
    assert_eq!(
        judge(&late, &good, &hand),
        Verdict::Diverged {
            index: 4,
            against: "monolith",
            got: Some(f("relay", 2, 1)),
            want: Some(f("relay", 1, 1)),
        }
    );

    // A merged trace that ends early: the missing entry is `None`.
    let short = good[..5].to_vec();
    assert_eq!(
        judge(&short, &good, &hand),
        Verdict::Diverged {
            index: 5,
            against: "monolith",
            got: None,
            want: Some(f("sink", 1, 2)),
        }
    );

    // A merged trace with an extra fire past the monolith's end.
    let mut long = good.clone();
    long.push(f("ticker", 1, 0));
    assert_eq!(
        judge(&long, &good, &hand),
        Verdict::Diverged {
            index: 6,
            against: "monolith",
            got: Some(f("ticker", 1, 0)),
            want: None,
        }
    );

    // Both legs broken identically (sink never fires): the monolith agrees,
    // the hand oracle does not.
    let no_sink = vec![
        f("ticker", 0, 0),
        f("relay", 0, 1),
        f("ticker", 1, 0),
        f("relay", 1, 1),
    ];
    assert_eq!(
        judge(&no_sink, &no_sink, &hand),
        Verdict::Diverged {
            index: 2,
            against: "hand",
            got: Some(f("ticker", 1, 0)),
            want: Some(f("sink", 0, 2)),
        }
    );

    // Tie: both oracles first disagree at the same index; the monolith is named.
    let wrong_first = vec![f("relay", 0, 1)];
    let mono_one = vec![f("ticker", 0, 0)];
    let hand_zero = hand_oracle(0);
    assert_eq!(
        judge(&wrong_first, &mono_one, &hand_zero),
        Verdict::Diverged {
            index: 0,
            against: "monolith",
            got: Some(f("relay", 0, 1)),
            want: Some(f("ticker", 0, 0)),
        }
    );

    // The closing summary line, byte for byte.
    assert_eq!(
        summary_line(Mode::FreeRun, 2, 5, 3),
        "DET_ORACLE_SUMMARY mode=free_run ranks=2 judged=5 equal=2 diverged=3"
    );

    // The mode witness: exactly one line per rank of the expected family,
    // none of the other, read after the ANSI strip the real log needs.
    let line = |path: &str| {
        format!(
            "\x1b[32m INFO\x1b[0m worker build path resolved \x1b[3mbuild_path\x1b[0m\x1b[2m=\x1b[0m{path}\n"
        )
    };
    let two_free = strip_ansi(&(line("FreeRunTraced") + &line("FreeRunLive")));
    assert!(two_free.contains("build_path=FreeRunTraced"));
    assert_eq!(mode_witness_mismatch(&two_free, Mode::FreeRun, 2), None);
    assert!(mode_witness_mismatch(&two_free, Mode::Lockstep, 2).is_some());
    assert!(mode_witness_mismatch(&two_free, Mode::FreeRun, 3).is_some());
    let mixed = strip_ansi(&(line("Lockstep") + &line("FreeRunTraced")));
    assert!(mode_witness_mismatch(&mixed, Mode::Lockstep, 1).is_some());
    let raw = line("Lockstep");
    assert!(
        mode_witness_mismatch(&raw, Mode::Lockstep, 1).is_some(),
        "an UNSTRIPPED capture carries no `build_path=` substring"
    );
    assert_eq!(
        mode_witness_mismatch(&strip_ansi(&raw), Mode::Lockstep, 1),
        None
    );

    // The machine-readable lines, byte for byte.
    assert_eq!(
        verdict_lines(Mode::FreeRun, 3, 2, 400, &Verdict::Equal),
        vec!["DET_ORACLE mode=free_run ranks=3 run=2 verdict=equal first_diff=- k=400".to_string()]
    );
    assert_eq!(
        verdict_lines(Mode::Lockstep, 2, 1, 99, &judge(&short, &good, &hand)),
        vec![
            "DET_ORACLE mode=lockstep ranks=2 run=1 verdict=diverged first_diff=5 k=99".to_string(),
            "DET_ORACLE_DIFF mode=lockstep ranks=2 run=1 against=monolith index=5 got=- \
             want=sink@1/L2"
                .to_string(),
        ]
    );
}
