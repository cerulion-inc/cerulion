// SPDX-License-Identifier: AGPL-3.0-only
//! The REAL-SUBPROCESS (cross-address-space
//! `MAP_SHARED` + real CPU monitor-wait park) confirmation of the in-process
//! barrier-gating FIREWALL already pinned by `barrier_level_gate_iox2_test.rs`.
//!
//! This is the cross-process replica of that file's firewall test
//! (`two_context_split_via_build_fn_merges_to_oracle`). The in-process test runs
//! the two contexts on two OS THREADS of ONE process — a REAL shared-state
//! rendezvous (two mappings of one `MAP_SHARED` page: the
//! [`MappedBarrier`](cerulion_core::barrier::MappedBarrier) is real POSIX SHM on
//! every Unix), but NOT cross-ADDRESS-SPACE, and not the real per-OS wait
//! shape under separate page tables. THIS file closes that scope gap:
//! it runs the SAME 5-node chain split across context
//! A = `{n0,n1}` / B = `{n2,n3,n4}` as TWO REAL OS PROCESSES (self-re-exec, NOT
//! threads) that share ONE iceoryx2 SHM root + ONE `MappedBarrier` over a real
//! POSIX SHM segment (`/dev/shm` on Linux), each built through the PRODUCTION
//! [`GraphRuntime::build_live_deterministic_with_manager_and_barrier`] and driven
//! on the deterministic-LIVE seam (`run_live_step_once_for_test`).
//!
//! ## The chain (5 nodes, pure external-data-driven — NO Period anywhere)
//!
//! ```text
//! /blg/ext (absolute external) ─▶ n0 ─▶ n1 ─▶ n2 ─▶ n3 ─▶ n4(sink)
//!                                 L0   L1   L2   L3   L4
//! ```
//!
//! `n0..n3` are `#[cerulion_node]` relays forwarding `out.x = inp.x`; `n4` records
//! each observed `inp.x` (the data-flow oracle). Context A owns globals 0,1;
//! context B owns globals 2,3,4. The `n1→n2` edge is a REAL iceoryx2 topic
//! (`/blga/n1/out`, graph-owned single-writer in A, consumed by B as an ABSOLUTE
//! external source). Global-level maps (the SUPERVISOR owns them):
//!
//! ```text
//!   A: [Some(0), Some(1), None,    None,    None   ]   owns globals 0,1
//!   B: [None,    None,    Some(0), Some(1), Some(2)]   owns globals 2,3,4
//! ```
//!
//! ## Process topology (the make-or-break facts)
//!
//! - **Config sharing**: the supervisor calls
//!   `cerulion_core::testing::iceoryx_test_config()` ONCE, serializes the Config
//!   (shared root_path + unique prefix) to a temp file, and hands BOTH children
//!   the SAME file via env. A child that minted its own config would land on a
//!   DIFFERENT SHM prefix and never see the cross-context handoff.
//! - **Shared barrier**: the supervisor `create_owned`s the segment (expected = 2)
//!   BEFORE spawning, so it OWNS (keeps mapped + reads the shared generation) but
//!   does NOT participate. The two children each `open_unowned` their own handle
//!   (bounded retry — exec startup races the create) → the two participants.
//! - **Build-order race**: cross-process has no main-thread sequential build, so
//!   child A writes a READY sentinel once it has created `/blg/ext` + the
//!   graph-owned `/blga/n1/out`; the supervisor waits for it before spawning B
//!   (which OPENS `/blga/n1/out` as a subscriber). A's first barrier wait tolerates
//!   B's startup latency (well under the ~5s `BARRIER_BOUNDARY_TIMEOUT`).
//!
//! ## The firewall (reproduces the in-process test's FIVE assertion classes cross-process)
//!
//! Each child writes its per-process trace (as a serializable `FlatEntry` vec) +
//! B writes its sink-observed `Vec<f64>` to a result file. The supervisor merges
//! the two via the PRODUCTION [`merge_partition_traces`] (A = rank 0, B = rank 1)
//! and asserts the merged result against a HAND oracle (NOT a self-compare): the
//! node-id sequence is `[n0,n1,n2,n3,n4] × N`, global levels `0..4` per step, all
//! 5 fires of a logical step share one `step`/`fire_time_ns` advancing by the
//! handed 4ms quantum, B observes exactly `1.0..=N`, and the shared barrier
//! generation advanced `(WARMUP + N) * GLOBAL_LEVELS`. The single-process monolith
//! (run in-supervisor) is itself asserted == the hand oracle, anchoring the
//! oracle. The barrier is a WHEN-gate on level advance, NEVER a change to the fire
//! set/order/data (determinism firewall, Principle #7).
//!
//! ## Scope + how to run
//!
//! HARDWARE-ONLY: the whole module is `#![cfg(unix)]` (not only
//! `target_os = "linux"`: the macOS `MappedBarrier` arm is the SAME real
//! POSIX `shm_open` + `MAP_SHARED` path, so separate processes on macOS share
//! the real cross-address-space page — no per-process stub registry
//! exists on Unix. The wait shape differs per OS: Linux futex + CPU park,
//! macOS boundary spin + chunked sleep-recheck). BOTH supervisor tests —
//! `box_two_process_split_merges_to_oracle` and
//! `box_stalled_peer_poisons_and_exits_nonzero` — stay `#[ignore]`'d so they run
//! only on demand on real hardware (x86 WAITPKG / aarch64 WFE / macOS). The
//! CI gate stays the in-process `barrier_level_gate_iox2_test` (OS-portable).
//!
//! ### What CI runs in this binary: ONE of three arms, and it asserts NOTHING
//!
//! Stated as a count, because "the supervisor test is `#[ignore]`'d" understates
//! it. This file holds exactly three `#[test]`s: the two `#[ignore]`'d
//! supervisors above, and `subprocess_child_entrypoint` — the body of the
//! SELF-RE-EXEC CHILD, which returns immediately when `CER_BARRIER_SUBPROC_ROLE`
//! is unset so a plain `cargo test` is not disturbed. CI therefore compiles,
//! links and launches this binary on every run and reports `1 passed; 2
//! ignored` having verified nothing at all. That `1 passed` reads like a gate
//! and is not one: a regression in the cross-process barrier path surfaces only
//! when somebody runs `-- --ignored` by hand. The CI-side safety net is the
//! in-process sibling named above, which is why it must keep its arms.
//!
//! Run with:
//!
//! ```bash
//! cargo test -p cerulion_core --test barrier_level_gate_subprocess_iox2_test \
//!     -- --ignored --nocapture
//! ```
//!
//! All hang/orphan safety nets are mandatory (a hung cross-process test gating a
//! manual run is worse than nothing): `ChildGuard`'s `Drop` SIGKILLs + reaps, every
//! wait loop is HARD-bounded (never an unbounded `while`), and a child that hits a
//! barrier timeout poisons + exits non-zero (caught by the supervisor's
//! `.success()` assert). `#[serial]` (real iceoryx2 SHM singleton + the barrier
//! registry; the namespace is pid-scoped so concurrent binaries never collide).
//!
//! No fake data (Principle #13): every node is a REAL `#[cerulion_node]` over a
//! REAL iceoryx2 SHM region; the cross-context handoff is a REAL iceoryx2 topic.
#![cfg(unix)]

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::barrier::MappedBarrier;
use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::CrossProcessWiring;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{merge_partition_traces, ProcessTrace, TraceEntry};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serde::{Deserialize, Serialize};
use serial_test::serial;

// ===========================================================================
// Tunables (mirror `barrier_level_gate_iox2_test.rs`).
// ===========================================================================

/// Number of measured publishes (and the final sink value).
const N: u64 = 5;

/// No-publish priming steps run (in lockstep, through the barrier) before the
/// measured loop — drain the build/attach connection-lifecycle noise so the
/// first MEASURED publish flows cleanly. No data flows during priming, so nothing
/// fires and no warmup value leaks into the measured window.
const WARMUP: u64 = 4;

/// Number of global DAG levels in the chain (= the global-level-map length). The
/// shared barrier crosses exactly this many generations per step.
const GLOBAL_LEVELS: u64 = 5;

/// The per-`step()` virtual-clock delta (the in-supervisor monolith oracle leg).
const STEP: Duration = Duration::from_millis(1);

/// The per-iteration WaitSet wake timeout for the `run_live_step_once_for_test`-
/// driven children. Bounds ONLY a no-data wait; a barrier rendezvous is gated
/// INSIDE `step_live`, so this never causes a deadlock and is well under the
/// production `BARRIER_BOUNDARY_TIMEOUT` (~5s).
const LIVE_TIMEOUT: Duration = Duration::from_millis(100);

/// The absolute external trigger topic feeding `n0` (no in-graph producer → the
/// graph provisions it `External` and an out-of-graph publisher attaches freely).
const EXT_TOPIC: &str = "/blg/ext";

/// The cross-context handoff topic: graph-owned by context A (`n1`'s output,
/// single-writer), consumed by context B's `n2` as an absolute external source.
const HANDOFF_TOPIC: &str = "/blga/n1/out";

/// Context A's global-level map: owns globals 0,1 (A-local 0,1), empty at 2,3,4.
const MAP_A: [Option<usize>; 5] = [Some(0), Some(1), None, None, None];

/// Context B's global-level map: empty at 0,1, owns globals 2,3,4 (B-local 0,1,2).
const MAP_B: [Option<usize>; 5] = [None, None, Some(0), Some(1), Some(2)];

/// The shared barrier id (within the pid-scoped namespace). Both children open it.
const BARRIER_ID: &str = "g";

// Env vars the supervisor sets to drive each child subprocess. ALL UNSET in a
// normal `cargo test` run → the child entrypoint is a harmless no-op pass.

/// `"A"` or `"B"` — selects which context the re-exec'd child binary runs.
const ENV_ROLE: &str = "CER_BARRIER_SUBPROC_ROLE";
/// Path to the serialized shared iceoryx2 `Config` (same root_path + prefix).
const ENV_CONFIG: &str = "CER_BARRIER_SUBPROC_CONFIG";
/// The pid-scoped barrier namespace both children `open_unowned` under.
const ENV_NS: &str = "CER_BARRIER_SUBPROC_NS";
/// The shared handed quantum (in ns) — both children advance their gating clocks
/// by exactly this per step.
const ENV_HANDED_NS: &str = "CER_BARRIER_SUBPROC_HANDED_NS";
/// Path this child writes its `ChildResult` JSON to.
const ENV_RESULT: &str = "CER_BARRIER_SUBPROC_RESULT";
/// Path child A writes its READY sentinel to (set only for role A).
const ENV_READY: &str = "CER_BARRIER_SUBPROC_READY";

// ===========================================================================
// Nodes — a relay (n0..n3) and a recording sink (n4).
// ===========================================================================

/// Forwarding relay: triggers on `inp`, copies `inp.x` to `out.x`. Used for
/// `n0..n3` (only their wiring differs, which lives in the config, not the type).
#[cerulion_node]
#[derive(Default)]
struct ChainRelay {
    #[input(trigger)]
    inp: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl ChainRelay {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.inp.x;
        Ok(())
    }
}

/// Recording sink (`n4`): triggers on `inp`, pushes each observed `inp.x` into a
/// shared `Vec<f64>` — the data-flow oracle.
#[cerulion_node]
#[derive(Default)]
struct ChainSink {
    #[input(trigger)]
    inp: Vector3,
    observed: Arc<Mutex<Vec<f64>>>,
}

#[cerulion_node_impl]
impl ChainSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.observed.lock().unwrap().push(self.inp.x);
        Ok(())
    }
}

// ===========================================================================
// Config builders (verbatim from `barrier_level_gate_iox2_test.rs`).
// ===========================================================================

/// A defaulted `geometry_msgs/Vector3` output (derived topic, runtime-resolved
/// slice len, volatile history) — the shape the build path expects.
fn out_def(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "geometry_msgs/Vector3".to_string(),
        max_slice_len: None,
        topic: None,
        history_size: 0,
    }
}

fn in_def(name: &str, source: &str) -> InputDef {
    InputDef {
        name: name.to_string(),
        source: source.to_string(),
    }
}

/// A relay `NodeDef`: one trigger input `inp` from `source`, one output `out`.
fn relay_node(id: &str, source: &str) -> NodeDef {
    NodeDef {
        fuse: None,
        ros2: None,
        id: id.to_string(),
        node_type: "chain_relay".to_string(),
        inputs: vec![in_def("inp", source)],
        outputs: vec![out_def("out")],
    }
}

/// The sink `NodeDef`: one trigger input `inp` from `source`, no output.
fn sink_node(id: &str, source: &str) -> NodeDef {
    NodeDef {
        fuse: None,
        ros2: None,
        id: id.to_string(),
        node_type: "chain_sink".to_string(),
        inputs: vec![in_def("inp", source)],
        outputs: vec![],
    }
}

/// MONOLITH: all 5 nodes in one graph (prefix `blg`). `n0` reads the absolute
/// external `/blg/ext`; the rest chain `n{k}` ← `n{k-1}/out`.
fn monolith_graph(
    observed: Arc<Mutex<Vec<f64>>>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "barrier_gate_monolith".to_string(),
        prefix: "blg".to_string(),
        nodes: vec![
            relay_node("n0", EXT_TOPIC),
            relay_node("n1", "n0/out"),
            relay_node("n2", "n1/out"),
            relay_node("n3", "n2/out"),
            sink_node("n4", "n3/out"),
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for id in ["n0", "n1", "n2", "n3"] {
        factories.insert(id.to_string(), Box::new(ChainRelayEntry::new()));
    }
    factories.insert(
        "n4".to_string(),
        Box::new(ChainSinkEntry::with_state(ChainSink {
            observed,
            ..Default::default()
        })),
    );
    (config, factories)
}

/// CONTEXT A: `{n0, n1}` (prefix `blga`). `n0` reads `/blg/ext`; `n1` reads
/// `n0/out` and publishes to the derived `/blga/n1/out` (the cross-context
/// handoff, graph-owned single-writer here).
fn context_a_graph() -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "barrier_gate_ctx_a".to_string(),
        prefix: "blga".to_string(),
        nodes: vec![relay_node("n0", EXT_TOPIC), relay_node("n1", "n0/out")],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for id in ["n0", "n1"] {
        factories.insert(id.to_string(), Box::new(ChainRelayEntry::new()));
    }
    (config, factories)
}

/// CONTEXT B: `{n2, n3, n4}` (prefix `blgb`). `n2` reads the ABSOLUTE external
/// `/blga/n1/out` (no in-graph producer in B → levelizes to B-local L0); `n3`
/// reads `n2/out`; `n4` (sink) reads `n3/out`.
fn context_b_graph(
    observed: Arc<Mutex<Vec<f64>>>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "barrier_gate_ctx_b".to_string(),
        prefix: "blgb".to_string(),
        nodes: vec![
            relay_node("n2", HANDOFF_TOPIC),
            relay_node("n3", "n2/out"),
            sink_node("n4", "n3/out"),
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for id in ["n2", "n3"] {
        factories.insert(id.to_string(), Box::new(ChainRelayEntry::new()));
    }
    factories.insert(
        "n4".to_string(),
        Box::new(ChainSinkEntry::with_state(ChainSink {
            observed,
            ..Default::default()
        })),
    );
    (config, factories)
}

// ===========================================================================
// Transport / barrier / publish helpers.
// ===========================================================================

/// Build one "graph process"'s `TransportManager` (iceoryx2 node) over the shared
/// isolated SHM root `ix`, taking the [`VirtualClock`] the build fn will also be
/// handed: the realized cross-process entrypoint enforces the clock contract —
/// `transport.clock_arc()` must `Arc::ptr_eq` the `clock` passed to it — so the
/// child must build the manager with the SAME clock Arc it hands to the build fn.
///
/// `subscriber_buffer_size = 16` (not 8): context A OWNS the `/blga/n1/out`
/// handoff service but has NO in-graph consumer, so it provisions the service's
/// buffer ceiling from this transport default alone (`max(0, 16) = 16`). Context
/// B's `n2` then opens it as a subscriber requiring `DEFAULT_CONSUMER_DEPTH` (10)
/// — which 16 covers but the stock 8 would NOT (`10 > 8` ⇒
/// `DoesNotSupportRequestedMinBufferSize`).
fn manager_with_clock(
    name: &str,
    ix: iceoryx2::config::Config,
    clock: Arc<VirtualClock>,
) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: name.into(),
            clock,
            subscriber_buffer_size: 16,
            network: None,
        },
        ix,
    )
    .expect("init isolated transport manager (shared clock)")
}

/// Publish exactly ONE `Vector3` frame carrying `x` (the loan proxy publishes on
/// drop).
fn publish_one(pubr: &mut cerulion_core::CerulionPublisher, x: f64) {
    let mut proxy = pubr.loan_proxy::<Vector3>().expect("loan");
    proxy.x = x;
    drop(proxy); // publish
}

/// `open_unowned` the shared barrier with a BOUNDED retry: exec startup races the
/// supervisor's `create_owned`, so a child may briefly see the segment missing.
/// Caps at a ~10s deadline (sleeping 5ms) so a genuinely-broken config fail-fasts
/// instead of spinning forever.
fn open_unowned_with_retry(ns: &str, id: &str) -> std::io::Result<MappedBarrier> {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        match MappedBarrier::open_unowned(ns, id) {
            Ok(b) => return Ok(b),
            Err(e) => {
                if Instant::now() >= deadline {
                    return Err(e);
                }
                std::thread::sleep(Duration::from_millis(5));
            }
        }
    }
}

/// pid+tag-scoped barrier namespace so the two `#[serial]` supervisor tests (and
/// concurrent binaries / re-runs) never collide on a `/dev/shm` object.
fn barrier_ns(tag: &str) -> String {
    format!("barrier_subproc_{}_{tag}", std::process::id())
}

// ===========================================================================
// Oracle helpers (verbatim from `barrier_level_gate_iox2_test.rs`).
// ===========================================================================

/// The HAND oracle fire sequence for `n` publishes: each publish collapses to one
/// same-step `n0→n1→n2→n3→n4` fire, so the trace is `[n0..n4]` repeated `n` times.
fn expected_fire_sequence(n: u64) -> Vec<String> {
    let mut seq = Vec::with_capacity(n as usize * 5);
    for _ in 0..n {
        for id in ["n0", "n1", "n2", "n3", "n4"] {
            seq.push(id.to_string());
        }
    }
    seq
}

/// The HAND oracle observed-value sequence: exactly `1.0..=n` in order.
fn expected_values(n: u64) -> Vec<f64> {
    (1..=n).map(|i| i as f64).collect()
}

/// Derive the node-id fire SEQUENCE from a `TraceEntry` slice.
fn node_ids(entries: &[TraceEntry]) -> Vec<String> {
    entries.iter().map(|e| e.node_id.to_string()).collect()
}

/// Project a `TraceEntry` slice onto its GLOBAL DAG levels.
fn global_levels(entries: &[TraceEntry]) -> Vec<usize> {
    entries.iter().map(|e| e.global_level).collect()
}

/// The HAND oracle global-level sequence for `n` publishes: each same-step
/// collapse fires global levels `0,1,2,3,4` in order, repeated `n` times.
fn expected_global_levels(n: u64) -> Vec<usize> {
    let mut v = Vec::with_capacity(n as usize * 5);
    for _ in 0..n {
        for g in 0..5usize {
            v.push(g);
        }
    }
    v
}

/// Merge two contexts' per-process traces via the PRODUCTION
/// [`merge_partition_traces`]: context A is `rank 0`, context B is `rank 1`. The
/// merge reads each entry's own `step` + GLOBAL `global_level` and sorts by
/// `(step, global_level, rank, seq)`.
fn merge_two_contexts(a_entries: &[TraceEntry], b_entries: &[TraceEntry]) -> Vec<TraceEntry> {
    merge_partition_traces(&[
        ProcessTrace {
            rank: 0,
            trace: a_entries,
        },
        ProcessTrace {
            rank: 1,
            trace: b_entries,
        },
    ])
    .expect("partition ranks are distinct (0 and 1)")
}

// ===========================================================================
// The MONOLITH leg (in-supervisor oracle anchor).
// ===========================================================================

/// Build + run the 5-node monolith for `n` measured publishes (polled `step()` at
/// 1ms). Returns `(trace_entries, observed_values)`. The supervisor asserts BOTH
/// against the hand oracle to anchor it — proving the hand vectors are the true
/// single-process behavior (so the cross-process comparison is not a self-compare).
/// NOTE: the monolith uses the 1ms polled clock model, so its `fire_time_ns`
/// stamps differ from the live children's 4ms handed quantum — the supervisor does
/// NOT compare merged == monolith (exactly like the in-process test); only the node-id + value
/// hand oracles are shared.
fn run_monolith(n: u64) -> (Vec<TraceEntry>, Vec<f64>) {
    let observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let (config, factories) = monolith_graph(Arc::clone(&observed));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build monolith chain");

    let mut pubr = {
        let mgr: &Arc<TransportManager> = runtime.test_transport().expect("test transport parked");
        mgr.create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
            .expect("external publisher must attach to /blg/ext")
    };

    for _ in 0..WARMUP {
        runtime.step(STEP);
    }
    runtime.clear_trace();

    for i in 1..=n {
        publish_one(&mut pubr, i as f64);
        runtime.step(STEP);
    }

    let trace_entries: Vec<TraceEntry> = runtime.trace().to_vec();
    let observed_values = observed.lock().unwrap().clone();
    runtime.shutdown();
    (trace_entries, observed_values)
}

// ===========================================================================
// Cross-process result wire format. `TraceEntry` is not Serialize, but all its
// fields are pub + replay-deterministic (except `duration_ns`, which is EXCLUDED
// from `Eq`), so a child flattens its trace into `FlatEntry`s the supervisor
// reconstructs.
// ===========================================================================

/// The serializable projection of a `TraceEntry` (omits `duration_ns` — wall time,
/// excluded from `TraceEntry`'s `PartialEq`/`Eq`).
#[derive(Serialize, Deserialize)]
struct FlatEntry {
    node_id: String,
    step: u64,
    fire_time_ns: u64,
    global_level: usize,
}

impl From<&TraceEntry> for FlatEntry {
    fn from(e: &TraceEntry) -> Self {
        FlatEntry {
            node_id: e.node_id.to_string(),
            step: e.step,
            fire_time_ns: e.fire_time_ns,
            global_level: e.global_level,
        }
    }
}

/// One child's complete result: its per-process trace + (for B) its sink's
/// observed data flow (empty for A, which has no sink).
#[derive(Serialize, Deserialize)]
struct ChildResult {
    entries: Vec<FlatEntry>,
    observed: Vec<f64>,
}

/// Reconstruct a `Vec<TraceEntry>` from the wire `FlatEntry`s. `duration_ns` is set
/// to 0 (excluded from `Eq`, so it cannot affect any merge/equality assertion).
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
// Child entrypoint — re-invoked as a subprocess via `current_exe`.
// ===========================================================================

/// When `CER_BARRIER_SUBPROC_ROLE` is UNSET this is a harmless no-op pass (so a
/// normal `cargo test` run isn't disturbed). When SET (the supervisor re-invokes
/// THIS binary with `--exact subprocess_child_entrypoint` + the env), it runs the
/// named context to completion and `std::process::exit(0)` on success, or `exit(2)`
/// on a barrier timeout / any failure (the supervisor catches a non-zero exit via
/// `.success()`).
#[test]
// P12 exemption, scoped to this fn rather than the file: this is the body of a
// SELF-RE-EXEC CHILD process — a process entrypoint by construction, whose exit
// code IS the channel the parent reads its verdict from. The ban stays armed for
// every other line in this binary, which is the half a file-wide allow gave up.
#[allow(clippy::disallowed_methods)]
fn subprocess_child_entrypoint() {
    let role = match std::env::var(ENV_ROLE) {
        Ok(r) => r,
        // Not the child invocation — normal `cargo test` run. No-op pass.
        Err(_) => return,
    };
    match run_child(&role) {
        Ok(()) => std::process::exit(0),
        Err(e) => {
            eprintln!("[barrier subproc role={role}] FAILED: {e}");
            std::process::exit(2);
        }
    }
}

fn run_child(role: &str) -> Result<(), Box<dyn std::error::Error>> {
    let ix = deserialize_ix_config(&env_required(ENV_CONFIG)?)?;
    let ns = env_required(ENV_NS)?;
    let handed = Duration::from_nanos(env_required(ENV_HANDED_NS)?.parse::<u64>()?);
    let result_path = env_required(ENV_RESULT)?;
    match role {
        "A" => {
            let ready_path = env_required(ENV_READY)?;
            run_child_a(ix, ns, handed, result_path, ready_path)
        }
        "B" => run_child_b(ix, ns, handed, result_path),
        other => Err(format!("unknown subprocess role {other:?} (expected \"A\" or \"B\")").into()),
    }
}

/// Role A `{n0, n1}`: build via the (e) entrypoint, attach the `/blg/ext`
/// publisher, signal READY (build + the graph-owned `/blga/n1/out` exist now),
/// then prime + publish `1..=N`, one live iteration each.
fn run_child_a(
    ix: iceoryx2::config::Config,
    ns: String,
    handed: Duration,
    result_path: String,
    ready_path: String,
) -> Result<(), Box<dyn std::error::Error>> {
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager_with_clock("barrier_subproc_a", ix, Arc::clone(&clock));
    let barrier = Arc::new(open_unowned_with_retry(&ns, BARRIER_ID)?);

    let (cfg_a, fac_a) = context_a_graph();
    let mut rt_a = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        cfg_a,
        fac_a,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        barrier,
        MAP_A.to_vec(),
        // No level takes the extra mid-level rendezvous in this
        // fixture, so the run keeps exactly one barrier generation per global
        // level — the law every generation pin here asserts.
        vec![false; MAP_A.len()],
        0,
        handed,
        // In-process test harness — no cross-group provisioning union.
        CrossProcessWiring::requirements_only(None),
    )?;

    let mut pubr = mgr.create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)?;

    // READY: the build + the graph-owned single-writer `/blga/n1/out` + the
    // `/blg/ext` publisher all exist, so the supervisor may spawn B (which OPENS
    // `/blga/n1/out` as a subscriber).
    std::fs::write(&ready_path, b"ready")?;

    for _ in 0..WARMUP {
        rt_a.run_live_step_once_for_test(LIVE_TIMEOUT);
    }
    rt_a.clear_trace();
    for i in 1..=N {
        publish_one(&mut pubr, i as f64);
        rt_a.run_live_step_once_for_test(LIVE_TIMEOUT);
    }

    if rt_a.is_barrier_failed() {
        return Err(
            "context A poisoned: a barrier boundary timed out (peer B never rendezvoused)".into(),
        );
    }
    let entries: Vec<FlatEntry> = rt_a.trace().iter().map(FlatEntry::from).collect();
    rt_a.shutdown();
    drop(pubr);
    drop(mgr);

    let result = ChildResult {
        entries,
        observed: Vec::new(),
    };
    std::fs::write(&result_path, serde_json::to_string(&result)?)?;
    Ok(())
}

/// Role B `{n2, n3, n4}`: build via the (e) entrypoint (consuming the absolute
/// external `/blga/n1/out`), prime + step `N`, record the sink's observed values.
fn run_child_b(
    ix: iceoryx2::config::Config,
    ns: String,
    handed: Duration,
    result_path: String,
) -> Result<(), Box<dyn std::error::Error>> {
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager_with_clock("barrier_subproc_b", ix, Arc::clone(&clock));
    let barrier = Arc::new(open_unowned_with_retry(&ns, BARRIER_ID)?);

    let b_observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let (cfg_b, fac_b) = context_b_graph(Arc::clone(&b_observed));
    let mut rt_b = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        cfg_b,
        fac_b,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        barrier,
        MAP_B.to_vec(),
        // No level takes the extra mid-level rendezvous in this
        // fixture, so the run keeps exactly one barrier generation per global
        // level — the law every generation pin here asserts.
        vec![false; MAP_B.len()],
        1,
        handed,
        // In-process test harness — no cross-group provisioning union.
        CrossProcessWiring::requirements_only(None),
    )?;

    for _ in 0..WARMUP {
        rt_b.run_live_step_once_for_test(LIVE_TIMEOUT);
    }
    rt_b.clear_trace();
    for _ in 0..N {
        rt_b.run_live_step_once_for_test(LIVE_TIMEOUT);
    }

    if rt_b.is_barrier_failed() {
        return Err(
            "context B poisoned: a barrier boundary timed out (peer A never rendezvoused)".into(),
        );
    }
    let entries: Vec<FlatEntry> = rt_b.trace().iter().map(FlatEntry::from).collect();
    let observed = b_observed.lock().unwrap().clone();
    rt_b.shutdown();
    drop(mgr);

    let result = ChildResult { entries, observed };
    std::fs::write(&result_path, serde_json::to_string(&result)?)?;
    Ok(())
}

fn env_required(key: &str) -> Result<String, Box<dyn std::error::Error>> {
    match std::env::var(key) {
        Ok(v) => Ok(v),
        Err(_) => Err(format!("required env var {key} is unset").into()),
    }
}

fn deserialize_ix_config(
    path: &str,
) -> Result<iceoryx2::config::Config, Box<dyn std::error::Error>> {
    let contents = std::fs::read_to_string(path)?;
    Ok(serde_json::from_str(&contents)?)
}

// ===========================================================================
// RAII child guard — Drop kills + reaps so a supervisor panic never orphans.
// ===========================================================================

/// Owns the spawned child `std::process::Child`. `Drop` SIGKILLs then reaps it, so
/// a supervisor panic / early-return / assertion failure cannot leave an orphan.
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
    /// the deadline elapsed (in which case the child is SIGKILLed + reaped — a hung
    /// child must NEVER hang a manual run). Idempotent: after a successful wait the
    /// guard's `Drop` is a no-op.
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
                    // A spurious `try_wait` error (e.g. ECHILD) is NOT a deadline
                    // timeout: surface it (visible under `--nocapture`) so the
                    // supervisor's later timeout `.expect` isn't a misleading
                    // "barrier deadlock" diagnostic, AND force-reap here — otherwise
                    // setting `reaped = true` with no kill/wait makes `Drop` skip the
                    // reap too, orphaning the child.
                    eprintln!("[barrier subproc supervisor] try_wait on child errored: {e}");
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
// Supervisor spawn + sentinel helpers.
// ===========================================================================

/// Spawn a child by re-invoking THIS test binary with a filter that runs ONLY
/// `subprocess_child_entrypoint`, handing it its role + shared-state paths via env.
fn spawn_child(
    role: &str,
    config_path: &std::path::Path,
    ns: &str,
    handed: Duration,
    result_path: &std::path::Path,
    ready_path: Option<&std::path::Path>,
) -> std::io::Result<std::process::Child> {
    let exe = std::env::current_exe()?;
    let mut cmd = std::process::Command::new(exe);
    cmd.args([
        "--exact",
        "subprocess_child_entrypoint",
        "--test-threads=1",
        "--nocapture",
    ])
    .env(ENV_ROLE, role)
    .env(ENV_CONFIG, config_path)
    .env(ENV_NS, ns)
    .env(ENV_HANDED_NS, handed.as_nanos().to_string())
    .env(ENV_RESULT, result_path)
    // Inherit stdout/stderr so a child panic / failure message is visible under
    // --nocapture; the supervisor does not parse it (it reads result files).
    .stdout(std::process::Stdio::inherit())
    .stderr(std::process::Stdio::inherit());
    if let Some(rp) = ready_path {
        cmd.env(ENV_READY, rp);
    }
    cmd.spawn()
}

/// BOUNDED wait for `path` to appear (child A's READY sentinel). Fail-fasts (returns
/// false) at the deadline instead of hanging.
fn wait_for_file(path: &std::path::Path, deadline: Duration) -> bool {
    let start = Instant::now();
    while start.elapsed() < deadline {
        if path.exists() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    path.exists()
}

fn read_child_result(path: &std::path::Path) -> ChildResult {
    let contents = std::fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("read child result {}: {e}", path.display()));
    serde_json::from_str(&contents)
        .unwrap_or_else(|e| panic!("deserialize child result {}: {e}", path.display()))
}

// ===========================================================================
// THE SUPERVISOR — the cross-process replica of the in-process firewall test
// (`two_context_split_via_build_fn_merges_to_oracle`).
//
// `#[ignore]` so it runs ONLY on demand on real hardware (real MAP_SHARED + CPU park):
//   cargo test -p cerulion_core --test barrier_level_gate_subprocess_iox2_test \
//       -- --ignored --nocapture
// ===========================================================================

#[test]
#[ignore]
#[serial]
fn box_two_process_split_merges_to_oracle() {
    // The shared handed quantum: the production supervisor would derive
    // `tightest_timing_ns().unwrap_or(1ms).max(1ms)` = 1ms for the pure-data /blg
    // chain, but (exactly like the in-process test) we pin 4ms so the per-step `fire_time_ns += quantum`
    // assertion is meaningful (a HAND oracle, not a self-compare).
    const HANDED: Duration = Duration::from_millis(4);
    const HANDED_NS: u64 = 4_000_000;

    // ----- Oracle anchor: the single-process MONOLITH itself == the hand oracle.
    // The cross-process split below is compared against the SAME hand vectors, so
    // proving the monolith reproduces them keeps the firewall non-tautological.
    let (mono_entries, mono_vals) = run_monolith(N);
    assert_eq!(
        node_ids(&mono_entries),
        expected_fire_sequence(N),
        "the single-process monolith must itself fire [n0..n4] × N — anchors the \
         hand oracle (so the cross-process comparison below is not a self-compare)"
    );
    assert_eq!(
        mono_vals,
        expected_values(N),
        "the monolith sink must observe exactly 1.0..=N in order"
    );

    // ----- Shared SHM config: serialize ONE isolated iceoryx2 Config (shared
    // root_path + unique prefix); both children deserialize the SAME namespace.
    let ix = cerulion_core::testing::iceoryx_test_config();
    let tmp = tempfile::TempDir::new().expect("temp dir for child config + results");
    let config_path = tmp.path().join("ix_config.json");
    std::fs::write(
        &config_path,
        serde_json::to_string(&ix).expect("serialize iceoryx2 config"),
    )
    .expect("write shared iceoryx2 config");

    // ----- Shared barrier: the supervisor OWNS the segment (keeps it mapped + reads
    // the shared generation) but does NOT participate; the two children are the two
    // participants (expected = 2). pid+tag-scoped ns so concurrent binaries never collide.
    let ns = barrier_ns("firewall");
    let owner = MappedBarrier::create_owned(&ns, BARRIER_ID, 2).expect("barrier owner create");

    let a_result = tmp.path().join("a_result.json");
    let b_result = tmp.path().join("b_result.json");
    let a_ready = tmp.path().join("a_ready");

    // ----- Serialized spawn (avoids the cross-process build-order race): spawn A,
    // wait for its READY sentinel (A has created /blg/ext + the graph-owned
    // /blga/n1/out), THEN spawn B (which OPENS /blga/n1/out as a subscriber). Each
    // child is wrapped in a ChildGuard immediately so an assert still reaps it.
    let child_a = spawn_child(
        "A",
        &config_path,
        &ns,
        HANDED,
        &a_result,
        Some(a_ready.as_path()),
    )
    .expect("spawn child A");
    let mut guard_a = ChildGuard::new(child_a);

    let ready = wait_for_file(&a_ready, Duration::from_secs(10));
    assert!(
        ready,
        "child A never signalled READY within 10s — its build / publisher-attach \
         likely failed (see its stderr above). This is a fail-fast, not a hang."
    );

    let child_b =
        spawn_child("B", &config_path, &ns, HANDED, &b_result, None).expect("spawn child B");
    let mut guard_b = ChildGuard::new(child_b);

    // ----- Wait (bounded) for both to exit; a non-zero exit = a child-side failure
    // / barrier timeout. The generous 120s cap is belt-and-suspenders: each child
    // self-terminates within ~(WARMUP+N) × BARRIER_BOUNDARY_TIMEOUT even on a stall.
    let status_a = guard_a
        .wait_bounded(Duration::from_secs(120))
        .expect("child A did not exit within 120s (killed) — likely a barrier deadlock");
    let status_b = guard_b
        .wait_bounded(Duration::from_secs(120))
        .expect("child B did not exit within 120s (killed) — likely a barrier deadlock");
    assert!(
        status_a.success(),
        "child A (context n0,n1) exited non-zero ({status_a:?}) — see its stderr above"
    );
    assert!(
        status_b.success(),
        "child B (context n2,n3,n4) exited non-zero ({status_b:?}) — see its stderr above"
    );

    // ----- Read each child's per-process trace + B's observed data flow.
    let a_res = read_child_result(&a_result);
    let b_res = read_child_result(&b_result);
    let a_entries = reconstruct(&a_res.entries);
    let b_entries = reconstruct(&b_res.entries);
    let b_observed = b_res.observed;

    // (1) COUNTS + the PRODUCTION merge length.
    assert_eq!(
        a_entries.len() as u64,
        2 * N,
        "context A must fire n0,n1 once each per step"
    );
    assert_eq!(
        b_entries.len() as u64,
        3 * N,
        "context B must fire n2,n3,n4 once each per step"
    );
    let merged = merge_two_contexts(&a_entries, &b_entries);
    assert_eq!(
        merged.len() as u64,
        5 * N,
        "the merged trace must hold all 5 fires per step"
    );

    // (2) NODE-ID firewall — the merged sequence equals the HAND oracle (NOT a
    // self-compare against the monolith, which uses a different 1ms clock model).
    assert_eq!(
        node_ids(&merged),
        expected_fire_sequence(N),
        "the production-(e)-built CROSS-PROCESS 2-context fire sequence must equal \
         the hand oracle [n0,n1,n2,n3,n4] × N — the cross-process barrier is a \
         WHEN-gate on level advance, never a change to the fire set/order (Principle #7)"
    );

    // (3) GLOBAL-LEVEL firewall — fires global levels 0..4 in order each step.
    assert_eq!(
        global_levels(&merged),
        expected_global_levels(N),
        "the merged trace must fire global levels 0,1,2,3,4 in order each step"
    );

    // (4) STEP + FIRE_TIME firewall — within each logical step all 5 fires share one
    // `step` AND one deterministic `fire_time_ns`; across steps both advance by
    // exactly one step / one handed 4ms quantum (delta-based ⇒ robust to the WARMUP
    // offset, and proves the HANDED quantum drives the merged cross-process timeline).
    for c in 0..N as usize {
        let chunk = &merged[5 * c..5 * c + 5];
        let step0 = chunk[0].step;
        let ft0 = chunk[0].fire_time_ns;
        for e in chunk {
            assert_eq!(
                e.step, step0,
                "all 5 fires of one logical step must share one `step` value"
            );
            assert_eq!(
                e.fire_time_ns, ft0,
                "all 5 fires of one logical step must share one deterministic \
                 `fire_time_ns` (lockstep gating clocks under the shared handed quantum)"
            );
        }
        if c > 0 {
            let prev0 = &merged[5 * (c - 1)];
            assert_eq!(
                step0,
                prev0.step + 1,
                "`step` must increment by exactly 1 per logical step"
            );
            assert_eq!(
                ft0,
                prev0.fire_time_ns + HANDED_NS,
                "`fire_time_ns` must advance by exactly the handed 4ms quantum per step \
                 across the merged CROSS-PROCESS trace"
            );
        }
    }

    // (5) DATA-FLOW firewall — each value 1..=N crossed the REAL cross-PROCESS
    // iceoryx2 handoff in order (the hand oracle, NOT a self-compare).
    assert_eq!(
        b_observed,
        expected_values(N),
        "the split sink must observe exactly 1.0..=N in order — the cross-process \
         barrier's happens-before makes B's n2 read A's n1 same-step publish"
    );

    // (6) LOCKSTEP — the supervisor (segment owner) reads the shared generation
    // after both children exit: it advanced GLOBAL_LEVELS per step across BOTH
    // warmup and measured steps (every boundary, incl. None entries, rendezvoused
    // over the real MAP_SHARED page).
    assert_eq!(
        owner.current_generation(),
        (WARMUP + N) * GLOBAL_LEVELS,
        "the shared barrier generation must advance by GLOBAL_LEVELS per step in \
         lockstep across BOTH real processes"
    );

    // Guards are already reaped; explicit drops keep `tmp` (the result files) +
    // `owner` (the mapped segment) alive until every assertion has run.
    drop(guard_a);
    drop(guard_b);
    drop(tmp);
    drop(owner);
}

// ===========================================================================
// CROSS-PROCESS BARRIER-BLOCKS proof (the in-file anti-tautology). The
// deterministic, non-flaky cross-process analogue of the in-process sibling's
// `anti_tautology_barrier_is_load_bearing`: a participant whose peer never
// arrives MUST BLOCK at the first boundary → time out (one
// `BARRIER_BOUNDARY_TIMEOUT` ~5s) → poison → exit NON-ZERO.
//
// This is what makes the happy-path firewall above non-tautological: it proves
// the shared `MAP_SHARED` barrier GATES level advance (BLOCKS), not merely COUNTS
// rendezvous. If the barrier silently degraded to count-without-block, child A
// would proceed without its peer and exit 0 — FAILING this test. (A forced
// hostile interleave like the in-process control would need cross-process IPC
// sync — itself a barrier — so it would be inherently flaky; the stalled-peer
// poison is the deterministic cross-process equivalent. The park `TimedOut`
// primitive is also exercised cross-process in `barrier_test.rs`'s `box_harness`; THIS pins
// it end-to-end through the production `build_live_deterministic_with_manager_and_barrier`
// + runtime poison path.)
// ===========================================================================

#[test]
#[ignore]
#[serial]
fn box_stalled_peer_poisons_and_exits_nonzero() {
    // Shared SHM config (isolated root) for the lone child.
    let ix = cerulion_core::testing::iceoryx_test_config();
    let tmp = tempfile::TempDir::new().expect("temp dir for child config + result");
    let config_path = tmp.path().join("ix_config.json");
    std::fs::write(
        &config_path,
        serde_json::to_string(&ix).expect("serialize iceoryx2 config"),
    )
    .expect("write shared iceoryx2 config");

    // A barrier with expected = 2 — but we spawn ONLY child A. Its peer never
    // arrives, so A's FIRST barrier boundary blocks until the timeout. The "poison"
    // tag keeps this `#[serial]` test's ns distinct from the firewall test's.
    let ns = barrier_ns("poison");
    let owner = MappedBarrier::create_owned(&ns, BARRIER_ID, 2).expect("barrier owner create");

    let a_result = tmp.path().join("a_result.json");
    let a_ready = tmp.path().join("a_ready");
    let child_a = spawn_child(
        "A",
        &config_path,
        &ns,
        Duration::from_millis(4),
        &a_result,
        Some(a_ready.as_path()),
    )
    .expect("spawn lone child A");
    let mut guard_a = ChildGuard::new(child_a);

    // A reaches its step loop (READY) within seconds, then parks at the first
    // barrier boundary. After ONE BARRIER_BOUNDARY_TIMEOUT (~5s) its step poisons;
    // the remaining steps no-op, `is_barrier_failed()` trips, `run_child_a` returns
    // Err → exit(2). The 90s wait bound generously covers the worst case.
    assert!(
        wait_for_file(&a_ready, Duration::from_secs(10)),
        "lone child A never signalled READY within 10s — its build / publisher-attach \
         likely failed (see its stderr above). This is a fail-fast, not a hang."
    );
    let status = guard_a.wait_bounded(Duration::from_secs(90)).expect(
        "lone child A did not exit within 90s (killed) — it should poison + exit ~5s after READY",
    );

    assert!(
        !status.success(),
        "child A must exit NON-ZERO when its barrier peer never arrives ({status:?}) — \
         a poisoned barrier wait proves the shared MAP_SHARED barrier GATES (blocks) \
         level advance. A success here means the barrier degraded to count-without-block \
         (it did not gate), which would make the happy-path firewall pass-by-luck."
    );

    drop(guard_a);
    drop(tmp);
    drop(owner);
}
