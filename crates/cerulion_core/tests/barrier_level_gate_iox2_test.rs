// SPDX-License-Identifier: AGPL-3.0-only
//! The cross-process barrier-gating
//! FIREWALL pin, end-to-end over real iceoryx2.
//!
//! Commits 1-2 extracted `GraphRuntime::run_level` and wired a test-only
//! [`BarrierParticipant`] into `GraphRuntime::step`: when a participant is
//! present, `step` drives its level loop by GLOBAL level and rendezvouses on a
//! shared [`MappedBarrier`](cerulion_core::barrier::MappedBarrier) at EVERY
//! global-level boundary (incl. the `None` "empty participation" entries), so no
//! context begins global level `g+1` until ALL contexts finished global level
//! `g`. This file is the END-TO-END proof of the headline invariant:
//!
//! **A pure-data chain executed as ONE monolith fires the SAME nodes in the SAME
//! order and flows the SAME data as the SAME chain SPLIT across two contexts that
//! rendezvous on the shared barrier — and that split result equals a HAND-WRITTEN
//! oracle (so it is not a self-compare).**
//!
//! ## The chain (5 nodes, pure external-data-driven — NO Period anywhere)
//!
//! ```text
//! /blg/ext (absolute external) ─▶ n0 ─▶ n1 ─▶ n2 ─▶ n3 ─▶ n4(sink)
//!                                 L0   L1   L2   L3   L4
//! ```
//!
//! Each `n0..n3` is a `#[cerulion_node]` relay forwarding `out.x = inp.x`; `n4`
//! records each observed `inp.x` into a shared `Vec<f64>` (the data-flow oracle).
//! Linear single-node-per-level, so the monolith's global levelization is
//! `n0=L0 … n4=L4`. The harness publishes `x = 1.0..=N` on `/blg/ext`, one frame
//! per step; with the within-step level collapse each publish
//! flows `ext→n0→…→n4` in ONE `step()`.
//!
//! ## The two-context split
//!
//! Context A owns `{n0, n1}` (A-local levels 0,1); context B owns `{n2, n3, n4}`
//! (B-local levels 0,1,2). They run over the SAME isolated iceoryx2 SHM root as
//! two separate `TransportManager`s ("graph processes"). The `n1→n2` edge is a
//! REAL iceoryx2 topic A publishes (`/blga/n1/out`, graph-owned single-writer in
//! A) and B consumes as an ABSOLUTE external source (no in-graph producer in B,
//! so it levelizes to B-local L0). Global-level maps (the HARNESS owns them):
//!
//! ```text
//!   A: [Some(0), Some(1), None,    None,    None   ]   owns globals 0,1
//!   B: [None,    None,    Some(0), Some(1), Some(2)]   owns globals 2,3,4
//! ```
//!
//! Both contexts `arrive`+`wait` at every global boundary, so the shared barrier
//! generation advances by `GLOBAL_LEVELS` (5) per step regardless of which
//! context owns nodes there — the `None` entries are pure rendezvous.
//!
//! ## DETERMINISM FIREWALL (the thing under test)
//!
//! The barrier is a WHEN-gate on level advance ONLY — its generation /
//! `ArriveOutcome` / `WaitOutcome` NEVER enter a fire decision or the trace. So
//! the SET + ORDER of fired nodes and their data are decided entirely inside
//! `run_level`, identically to the single-process path. The merged 2-context fire
//! sequence — reconstructed by the PRODUCTION merge
//! [`merge_partition_traces`](cerulion_core::merge_partition_traces) over each
//! context's per-process [`TraceEntry`](cerulion_core::TraceEntry)s (every entry
//! already carries the deterministic `step` + GLOBAL `global_level` the levelized
//! executor stamped, both in `TraceEntry`'s `PartialEq`/`Eq`), sorted by
//! `(step, global_level, rank, seq)` — must be `==` the monolith
//! `Vec<TraceEntry>` AND its node-id sequence must equal the hand oracle
//! `[n0,n1,n2,n3,n4] × N`. (This file used to reconstruct the merge with a
//! file-local POSITIONAL hand-merge; that was deleted in favor of the production
//! `merge_partition_traces`, which is the stronger pin — it compares full
//! `TraceEntry`s, not just node-id strings.)
//!
//! ## In-process vs cross-process (scope)
//!
//! Since the macOS stub removal the `MappedBarrier` is REAL POSIX SHM on every Unix (macOS
//! included): two handles for the same `(ns,id)` are two `MAP_SHARED` mappings
//! of ONE physical page, so this is a REAL shared-state rendezvous test on
//! every OS — the two `GraphRuntime`s on two OS threads genuinely block on ONE
//! barrier. What is NOT exercised here is the cross-ADDRESS-SPACE split
//! (separate page tables) + the real per-OS wait shape under separate
//! processes; those are verified by `barrier_test.rs`'s `box_harness`
//! (multi-process self-re-exec — the two-subprocess pin runs in the
//! normal suite; the wide/park variants stay `#[ignore]`'d hardware-only) and by
//! `barrier_level_gate_subprocess_iox2_test.rs`. This file pins the SCHEDULER
//! INTEGRATION (the barrier-gated `step()` level loop), which is OS-portable.
//!
//! All tests `#[serial]` (real iceoryx2 transport over the shared-memory
//! singleton; the barrier registry is pid+tag scoped so concurrent binaries never
//! collide).

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

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
use serial_test::serial;

// ===========================================================================
// Tunables
// ===========================================================================

/// Number of measured publishes (and the final sink value).
const N: u64 = 5;

/// No-publish priming steps run (in lockstep, through the barrier) before the
/// measured loop. They drain the build/attach connection-lifecycle noise and
/// establish every iceoryx2 connection — including the cross-context `n1→n2`
/// handoff — so the first MEASURED publish flows cleanly (mirrors
/// `polled_vs_live_iox2_test`'s pre-loop drive, generalized to W steps for the
/// 5-level + cross-context chain). No data flows during priming, so nothing fires
/// and no warmup value can leak into the measured window.
const WARMUP: u64 = 4;

/// Number of global DAG levels in the chain (= the global-level-map length). The
/// shared barrier crosses exactly this many generations per step.
const GLOBAL_LEVELS: u64 = 5;

/// The per-`step()` virtual-clock delta (deterministic `VirtualClock`).
const STEP: Duration = Duration::from_millis(1);

/// The per-iteration WaitSet wake timeout for the
/// `run_live_step_once_for_test`-driven tests (B1/B4). It bounds ONLY a no-data
/// wait — a publish-then-drive wakes promptly on the iceoryx2 notification, and a
/// barrier rendezvous is gated INSIDE `step_live` (not here), so this never
/// causes a deadlock. It is well under the production `BARRIER_BOUNDARY_TIMEOUT`
/// (~5s), so a context that times out its wait still proceeds to `step_live` and
/// rendezvouses long before any peer poisons.
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

// ===========================================================================
// Nodes — a relay (n0..n3) and a recording sink (n4).
// ===========================================================================

/// Forwarding relay: triggers on `inp`, copies `inp.x` to `out.x`. Used for
/// `n0..n3` (the source `n0` and the inner relays share this shape — only their
/// wiring differs, which lives in the YAML/config, not the type).
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
// Config builders.
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

/// One "graph process": its own `TransportManager` (iceoryx2 node) over the
/// shared isolated SHM root `ix`.
///
/// `subscriber_buffer_size = 16` (not the 8 a single-context test would use):
/// context A OWNS the `/blga/n1/out` handoff service but has NO in-graph consumer
/// of it, so it provisions the service's buffer ceiling from this transport
/// default alone (`max(0, 16) = 16`). Context B's `n2` then opens that service as
/// a subscriber requiring `DEFAULT_CONSUMER_DEPTH` (10) — which 16 covers but the
/// stock 8 would NOT (`10 > 8` ⇒ `DoesNotSupportRequestedMinBufferSize`). So the
/// cross-context buffer ceiling must be raised at the OWNER (A) to satisfy the
/// downstream subscriber (B).
fn manager(name: &str, ix: iceoryx2::config::Config) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: name.into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 16,
            network: None,
        },
        ix,
    )
    .expect("init isolated transport manager")
}

/// Like [`manager`] but takes the
/// [`VirtualClock`] instead of minting a fresh one. The realized cross-process
/// entrypoint (`build_live_deterministic_with_manager_and_barrier`) enforces the
/// clock contract — `transport.clock_arc()` must `Arc::ptr_eq` the `clock` passed
/// to it — so the test must build the manager with the SAME clock Arc it hands to
/// the build fn. Mirrors `build_for_test_barrier`'s wiring: the config clones the
/// clock, the build fn gets the original Arc.
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

/// pid+tag-scoped barrier namespace so concurrent test binaries / re-runs never
/// collide on a POSIX SHM object (any Unix) or a registry key (the
/// non-Unix stub).
fn barrier_ns(tag: &str) -> String {
    format!("barrier_gate_{}_{tag}", std::process::id())
}

/// Publish exactly ONE `Vector3` frame carrying `x` (the loan proxy publishes on
/// drop).
fn publish_one(pubr: &mut cerulion_core::CerulionPublisher, x: f64) {
    let mut proxy = pubr.loan_proxy::<Vector3>().expect("loan");
    proxy.x = x;
    drop(proxy); // publish
}

/// The HAND oracle fire sequence for `n` publishes: each publish collapses to one
/// same-step `n0→n1→n2→n3→n4` fire, so the trace is `[n0..n4]` repeated `n` times.
/// NON-tautological — the absolute expected content, not a self-compare.
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

/// Derive the node-id fire SEQUENCE (`Vec<String>`) from a `TraceEntry` slice —
/// used to assert the merged/monolith trace against the node-id hand oracle
/// `expected_fire_sequence`. The full `TraceEntry` equality (`==`) is the
/// STRONGER pin (it also compares `step`/`fire_time_ns`/`global_level`); this
/// projection just maps the merged entries back onto the absolute node-id oracle.
fn node_ids(entries: &[TraceEntry]) -> Vec<String> {
    entries.iter().map(|e| e.node_id.to_string()).collect()
}

/// Project a `TraceEntry` slice onto its GLOBAL
/// DAG levels — used by B4 to assert the merged cross-process trace fires in
/// global-level order against [`expected_global_levels`].
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
/// merge reads each entry's own `step` + GLOBAL `global_level` (stamped by the
/// levelized executor — the harness no longer reconstructs the level positionally
/// from the global-level map) and sorts by `(step, global_level, rank, seq)`.
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
// The MONOLITH leg (oracle source of truth).
// ===========================================================================

/// Build + run the 5-node monolith for `n` measured publishes. Returns
/// `(trace_entries, observed_values)` — the OWNED monolith `Vec<TraceEntry>`
/// (cloned out AFTER the measured steps, BEFORE shutdown) is the byte-identity
/// oracle the production cross-process merge must reproduce; the node-id sequence
/// is derived from it via [`node_ids`] where the string oracle is asserted.
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

    // Priming: WARMUP no-publish steps establish every connection; nothing fires.
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
// The TWO-CONTEXT BARRIER-GATED leg.
// ===========================================================================

/// Outcome of one barrier-gated 2-context run. `a_entries`/`b_entries` are each
/// context's OWNED per-process `Vec<TraceEntry>` (cloned out after the measured
/// steps, before shutdown) — the inputs the production
/// [`merge_partition_traces`] stitches into the single global fire sequence.
struct TwoCtx {
    a_entries: Vec<TraceEntry>,
    b_entries: Vec<TraceEntry>,
    b_observed: Vec<f64>,
    final_generation: u64,
}

/// Build context A + context B over ONE shared SHM root, inject the shared
/// barrier participant into each, then run BOTH concurrently (one OS thread each)
/// for `WARMUP` priming + `n` measured steps in lockstep. `GraphRuntime` is `Send`
/// (verified), so each runtime is built on the main thread then MOVED into its
/// worker thread.
fn run_two_context_barrier(tag: &str, n: u64) -> TwoCtx {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr_a = manager("barrier_gate_a", ix.clone());
    let mgr_b = manager("barrier_gate_b", ix);

    // Build A FIRST so it creates `/blg/ext` + the graph-owned single-writer
    // `/blga/n1/out` service; then attach the external publisher; then build B,
    // which OPENS the existing handoff service as a subscriber.
    let (cfg_a, fac_a) = context_a_graph();
    let mut rt_a = GraphRuntime::build(cfg_a, fac_a, &mgr_a, Arc::new(VirtualClock::new()))
        .expect("build context A");

    let pubr = mgr_a
        .create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
        .expect("external publisher must attach to /blg/ext");

    let b_observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let (cfg_b, fac_b) = context_b_graph(Arc::clone(&b_observed));
    let mut rt_b = GraphRuntime::build(cfg_b, fac_b, &mgr_b, Arc::new(VirtualClock::new()))
        .expect("build context B");

    // ONE shared barrier (expected = 2). The owner binds the segment; the peer
    // opens it. On macOS both Deref to the SAME `BarrierShared` (by-name
    // registry); on Linux they map the SAME `MAP_SHARED` page. `probe` is a third
    // handle the main thread keeps to read the shared generation after the join.
    let ns = barrier_ns(tag);
    let owner = Arc::new(MappedBarrier::create_owned(&ns, "g", 2).expect("barrier owner create"));
    let peer = Arc::new(MappedBarrier::open_unowned(&ns, "g").expect("barrier peer open"));
    let probe = Arc::clone(&owner);

    rt_a.set_barrier_participant_for_test(owner, MAP_A.to_vec(), 0);
    rt_b.set_barrier_participant_for_test(peer, MAP_B.to_vec(), 1);

    // Worker A: owns the external publisher; primes, then publishes 1..=n.
    let a_handle = thread::spawn(move || {
        let mut pubr = pubr;
        for _ in 0..WARMUP {
            rt_a.step(STEP);
        }
        rt_a.clear_trace();
        for i in 1..=n {
            publish_one(&mut pubr, i as f64);
            rt_a.step(STEP);
        }
        let entries: Vec<TraceEntry> = rt_a.trace().to_vec();
        rt_a.shutdown();
        // Keep the manager + publisher alive until the runtime has shut down.
        drop(pubr);
        drop(mgr_a);
        entries
    });

    // Worker B: primes, then steps n times (its sink records the data flow).
    let b_handle = thread::spawn(move || {
        for _ in 0..WARMUP {
            rt_b.step(STEP);
        }
        rt_b.clear_trace();
        for _ in 0..n {
            rt_b.step(STEP);
        }
        let entries: Vec<TraceEntry> = rt_b.trace().to_vec();
        rt_b.shutdown();
        drop(mgr_b);
        entries
    });

    let a_entries = a_handle.join().expect("context A thread panicked");
    let b_entries = b_handle.join().expect("context B thread panicked");
    let b_observed = b_observed.lock().unwrap().clone();
    let final_generation = probe.current_generation();

    TwoCtx {
        a_entries,
        b_entries,
        b_observed,
        final_generation,
    }
}

/// The run_two_context_barrier sibling that builds
/// BOTH contexts through the PRODUCTION
/// [`GraphRuntime::build_live_deterministic_with_manager_and_barrier`] (the
/// realized (e) entrypoint) and drives them on the DETERMINISTIC-LIVE seam
/// (`run_live_step_once_for_test`), NOT the polled `step()` + test-only
/// `set_barrier_participant_for_test` path the original helper uses.
///
/// Both contexts are handed the SAME `handed_quantum` (so their gating clocks
/// advance in lockstep) and share ONE [`MappedBarrier`] (expected = 2). Each
/// builds its `TransportManager` with its OWN [`VirtualClock`] and passes that
/// same clock to the build fn (the clock contract). They run concurrently on two
/// OS threads, each calling `run_live_step_once_for_test` for `WARMUP` priming +
/// `n` measured iterations in barrier lockstep.
fn run_two_context_barrier_live(tag: &str, handed: Duration, n: u64) -> TwoCtx {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let clock_a = Arc::new(VirtualClock::new());
    let clock_b = Arc::new(VirtualClock::new());
    let mgr_a = manager_with_clock("barrier_gate_live_a", ix.clone(), Arc::clone(&clock_a));
    let mgr_b = manager_with_clock("barrier_gate_live_b", ix, Arc::clone(&clock_b));

    // ONE shared barrier (expected = 2): owner binds the segment, peer opens it;
    // `probe` keeps a third handle so the main thread reads the generation count
    // after the join.
    let ns = barrier_ns(tag);
    let owner = Arc::new(MappedBarrier::create_owned(&ns, "g", 2).expect("barrier owner create"));
    let peer = Arc::new(MappedBarrier::open_unowned(&ns, "g").expect("barrier peer open"));
    let probe = Arc::clone(&owner);

    // Build A FIRST (creates `/blg/ext` + the graph-owned single-writer
    // `/blga/n1/out`); attach the external publisher; then build B (opens the
    // handoff service as a subscriber). Each via the production (e) entrypoint.
    let (cfg_a, fac_a) = context_a_graph();
    let mut rt_a = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        cfg_a,
        fac_a,
        &mgr_a,
        Arc::clone(&clock_a),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        owner,
        MAP_A.to_vec(),
        // No level takes the extra mid-level rendezvous in this
        // fixture, so the run keeps exactly one barrier generation per global
        // level — the earlier law every generation pin here asserts.
        vec![false; MAP_A.len()],
        0,
        handed,
        // In-process test harness — no cross-group provisioning union.
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build context A via the (e) entrypoint");

    let pubr = mgr_a
        .create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
        .expect("external publisher must attach to /blg/ext");

    let b_observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let (cfg_b, fac_b) = context_b_graph(Arc::clone(&b_observed));
    let mut rt_b = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        cfg_b,
        fac_b,
        &mgr_b,
        Arc::clone(&clock_b),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        peer,
        MAP_B.to_vec(),
        // No level takes the extra mid-level rendezvous in this
        // fixture, so the run keeps exactly one barrier generation per global
        // level — the earlier law every generation pin here asserts.
        vec![false; MAP_B.len()],
        1,
        handed,
        // In-process test harness — no cross-group provisioning union.
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build context B via the (e) entrypoint");

    // Worker A: owns the external publisher; primes (no publish), then publishes
    // 1..=n, ONE live iteration each.
    let a_handle = thread::spawn(move || {
        let mut pubr = pubr;
        for _ in 0..WARMUP {
            rt_a.run_live_step_once_for_test(LIVE_TIMEOUT);
        }
        rt_a.clear_trace();
        for i in 1..=n {
            publish_one(&mut pubr, i as f64);
            rt_a.run_live_step_once_for_test(LIVE_TIMEOUT);
        }
        let entries: Vec<TraceEntry> = rt_a.trace().to_vec();
        rt_a.shutdown();
        drop(pubr);
        drop(mgr_a);
        entries
    });

    // Worker B: primes, then steps n times (its sink records the data flow).
    let b_handle = thread::spawn(move || {
        for _ in 0..WARMUP {
            rt_b.run_live_step_once_for_test(LIVE_TIMEOUT);
        }
        rt_b.clear_trace();
        for _ in 0..n {
            rt_b.run_live_step_once_for_test(LIVE_TIMEOUT);
        }
        let entries: Vec<TraceEntry> = rt_b.trace().to_vec();
        rt_b.shutdown();
        drop(mgr_b);
        entries
    });

    let a_entries = a_handle.join().expect("context A thread panicked");
    let b_entries = b_handle.join().expect("context B thread panicked");
    let b_observed = b_observed.lock().unwrap().clone();
    let final_generation = probe.current_generation();

    TwoCtx {
        a_entries,
        b_entries,
        b_observed,
        final_generation,
    }
}

// ===========================================================================
// The anti-tautology control (NO barrier participant).
// ===========================================================================

/// Build the SAME context A / context B split but inject NO barrier participant,
/// then drive them with a fixed HOSTILE interleave — within each measured step,
/// `B.step()` runs BEFORE A produces that step's handoff. With no cross-context
/// gating, B's global-L2 drain reads the PRIOR step's handoff (or nothing on the
/// first step), so its sink observes a shifted, INCOMPLETE sequence. Returns B's
/// observed values.
///
/// This is a DETERMINISTIC witness of the reorder hazard the barrier eliminates:
/// concurrent unsynchronized execution could produce this (or worse) NON-
/// deterministically; pinning the worst-case fixed interleave keeps the control
/// non-flaky. The control differs from the barrier'd run ONLY in (a) the absence
/// of the participant and (b) the resulting freedom to interleave — exactly the
/// load-bearing property under test. The step()-call ordering is legitimate test
/// driving (the runtimes carry NO barrier here); it is NOT a hand-simulation of
/// the barrier's arrive/wait.
fn run_two_context_no_barrier_shifted(n: u64) -> Vec<f64> {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr_a = manager("nobar_a", ix.clone());
    let mgr_b = manager("nobar_b", ix);

    let (cfg_a, fac_a) = context_a_graph();
    let mut rt_a = GraphRuntime::build(cfg_a, fac_a, &mgr_a, Arc::new(VirtualClock::new()))
        .expect("build context A (no-barrier control)");
    let mut pubr = mgr_a
        .create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
        .expect("external publisher must attach to /blg/ext");

    let b_observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let (cfg_b, fac_b) = context_b_graph(Arc::clone(&b_observed));
    let mut rt_b = GraphRuntime::build(cfg_b, fac_b, &mgr_b, Arc::new(VirtualClock::new()))
        .expect("build context B (no-barrier control)");

    // Prime both (no publish) to establish every connection.
    for _ in 0..WARMUP {
        rt_a.step(STEP);
        rt_b.step(STEP);
    }

    // Hostile interleave: B steps BEFORE A produces this step's handoff.
    for i in 1..=n {
        rt_b.step(STEP);
        publish_one(&mut pubr, i as f64);
        rt_a.step(STEP);
    }

    let observed = b_observed.lock().unwrap().clone();
    rt_a.shutdown();
    rt_b.shutdown();
    observed
}

// ===========================================================================
// Test 1 — the HEADLINE firewall: monolith == 2-context split == hand oracle.
// ===========================================================================

#[test]
#[serial]
fn monolith_eq_two_context_split_eq_oracle() {
    // MONOLITH (oracle source) — and prove the monolith ITSELF matches the hand
    // oracle, so the cross-leg comparison is not a self-compare.
    let (mono_entries, mono_vals) = run_monolith(N);
    let mono_seq = node_ids(&mono_entries);
    assert_eq!(
        mono_seq,
        expected_fire_sequence(N),
        "monolith fire sequence must be exactly [n0,n1,n2,n3,n4] × N (within-step collapse)"
    );
    assert_eq!(
        mono_vals,
        expected_values(N),
        "monolith sink must observe exactly 1.0..=N in order"
    );

    // TWO-CONTEXT split, barrier-gated.
    let split = run_two_context_barrier("headline", N);

    // Each context fired the right COUNT (A: 2/step, B: 3/step) — a dropped or
    // duplicated fire would desync the production merge below.
    assert_eq!(
        split.a_entries.len() as u64,
        2 * N,
        "context A must fire n0,n1 once each per step"
    );
    assert_eq!(
        split.b_entries.len() as u64,
        3 * N,
        "context B must fire n2,n3,n4 once each per step"
    );

    // THE FIREWALL — reconstructed by the PRODUCTION merge `merge_partition_traces`
    // (NOT a file-local positional hand-merge): each context's per-process
    // `TraceEntry`s carry their own deterministic `step` + GLOBAL `global_level`,
    // and the merge sorts by `(step, global_level, rank, seq)`. The barrier gated
    // WHEN each context advanced; it never changed WHAT fired.
    let merged = merge_two_contexts(&split.a_entries, &split.b_entries);

    // (a) The NEW, STRONGER pin: the merged `Vec<TraceEntry>` is byte-identical to
    // the monolith `Vec<TraceEntry>` under `TraceEntry`'s `PartialEq` — which
    // compares `node_id` + `step` + `fire_time_ns` + `global_level` (all
    // replay-deterministic), not just node-id strings. All three legs run the same
    // WARMUP+measured `VirtualClock` schedule, so for each measured step every leg
    // stamps the SAME `step`/`fire_time_ns` and the monolith's per-step level order
    // n0..n4 is reproduced exactly by the (step, global_level, rank) sort.
    assert_eq!(
        merged, mono_entries,
        "the production-merged 2-context trace must equal the monolith trace \
         ENTRY-FOR-ENTRY (step + fire_time_ns + global_level + node_id) — the \
         cross-process barrier is a WHEN-gate on level advance, never a change to \
         the fire set/order (determinism firewall, Principle #7)"
    );
    // (b) KEEP the absolute node-id hand oracle (non-tautological — not a
    // self-compare against the monolith).
    assert_eq!(
        node_ids(&merged),
        expected_fire_sequence(N),
        "the merged 2-context fire sequence must equal the hand oracle \
         [n0,n1,n2,n3,n4] × N (non-tautological)"
    );

    // Byte-identical DATA FLOW: each value 1..=N crossed the REAL cross-context
    // iceoryx2 handoff in order — the barrier's happens-before is what makes B's
    // n2 read A's n1 same-step publish.
    assert_eq!(
        split.b_observed, mono_vals,
        "the split sink must observe the SAME values as the monolith sink"
    );
    assert_eq!(
        split.b_observed,
        expected_values(N),
        "the split sink must observe exactly 1.0..=N in order (hand oracle)"
    );
}

// ===========================================================================
// Test 2 — determinism: two barrier-gated runs are byte-identical.
// ===========================================================================

#[test]
#[serial]
fn two_context_split_is_deterministic() {
    let a = run_two_context_barrier("determ_a", N);
    let b = run_two_context_barrier("determ_b", N);

    let merged_a = merge_two_contexts(&a.a_entries, &a.b_entries);
    let merged_b = merge_two_contexts(&b.a_entries, &b.b_entries);

    assert_eq!(
        merged_a, merged_b,
        "two barrier-gated 2-context runs must produce byte-identical production-merged \
         traces (ENTRY-for-entry: step + fire_time_ns + global_level + node_id) \
         (Principle #7)"
    );
    assert_eq!(
        a.b_observed, b.b_observed,
        "two barrier-gated 2-context runs must flow byte-identical data"
    );
    // …and both still equal the hand oracle (anchors determinism to the absolute
    // expected content, not just to each other).
    assert_eq!(node_ids(&merged_a), expected_fire_sequence(N));
    assert_eq!(a.b_observed, expected_values(N));
}

// ===========================================================================
// Test 3 — empty-level participation crosses EVERY global generation.
// ===========================================================================

#[test]
#[serial]
fn empty_level_participation_crosses_all_generations() {
    let split = run_two_context_barrier("empty_levels", N);

    // Every context arrives+waits at EVERY global boundary — including its `None`
    // entries (A's globals 2,3,4 and B's globals 0,1) — so the shared generation
    // advanced exactly `(WARMUP + N) * GLOBAL_LEVELS`. If a `None` entry SKIPPED
    // its rendezvous, (a) this count would be short AND (b) the peer would have
    // deadlocked waiting for it (the run would have timed out, not completed). The
    // run completed AND the count is exact → None entries genuinely rendezvous.
    let expected_gen = (WARMUP + N) * GLOBAL_LEVELS;
    assert_eq!(
        split.final_generation, expected_gen,
        "shared barrier generation must advance by GLOBAL_LEVELS per step across \
         BOTH warmup and measured steps (every global boundary rendezvoused, incl. \
         empty-participation None entries): expected {expected_gen}"
    );

    // Sanity: the run still produced the correct data (the None rendezvous did not
    // corrupt the fire path).
    assert_eq!(split.b_observed, expected_values(N));
}

// ===========================================================================
// Test 4 — multi-step lockstep: the data value at each step is exactly k.
// ===========================================================================

#[test]
#[serial]
fn multi_step_stays_in_lockstep() {
    // N > 1: prove lockstep held across EVERY step, not just the first. The sink's
    // k-th observed value must be exactly `k` — a single lost lockstep boundary
    // would shift, drop, or duplicate a value, breaking the strict 1..=N sequence.
    const STEPS: u64 = 8;
    let split = run_two_context_barrier("multi_step", STEPS);

    assert_eq!(
        split.b_observed,
        expected_values(STEPS),
        "across {STEPS} steps the sink must observe exactly 1.0..={STEPS} in order \
         — strict cross-context lockstep every step"
    );
    let merged = merge_two_contexts(&split.a_entries, &split.b_entries);
    assert_eq!(node_ids(&merged), expected_fire_sequence(STEPS));
    assert_eq!(
        split.final_generation,
        (WARMUP + STEPS) * GLOBAL_LEVELS,
        "generation count tracks the multi-step run exactly"
    );
}

// ===========================================================================
// Test 5 — anti-tautology: the barrier is LOAD-BEARING.
// ===========================================================================

#[test]
#[serial]
fn anti_tautology_barrier_is_load_bearing() {
    // WITHOUT the barrier (hostile fixed interleave), context B's sink observes a
    // SHIFTED, INCOMPLETE sequence: its global-L2 drain runs before A produces the
    // same-step handoff, so it reads the PRIOR step's value and loses the last
    // one. The deterministic oracle is `1.0..=(N-1)` — N-1 values, missing the
    // final.
    let no_barrier = run_two_context_no_barrier_shifted(N);
    assert_eq!(
        no_barrier,
        expected_values(N - 1),
        "WITHOUT the barrier the hostile interleave loses the last value — B \
         observes only 1.0..=(N-1)"
    );
    assert_ne!(
        no_barrier,
        expected_values(N),
        "the no-barrier control must NOT reproduce the complete 1.0..=N sequence"
    );

    // WITH the barrier (same nodes, same data source), B observes the COMPLETE,
    // in-order 1.0..=N — the barrier's level gating is what recovers the value the
    // unsynchronized interleave lost. Same split, toggled participant → different
    // result ⇒ the barrier is load-bearing.
    let with_barrier = run_two_context_barrier("load_bearing", N);
    assert_eq!(
        with_barrier.b_observed,
        expected_values(N),
        "WITH the barrier B observes the complete 1.0..=N — the cross-process \
         level gate makes B's n2 read A's n1 same-step publish"
    );
    assert_ne!(
        with_barrier.b_observed, no_barrier,
        "barrier-gated vs un-gated results MUST differ — the barrier is the only \
         thing that changed"
    );
}

// ===========================================================================
// Test 6 — a stalled peer POISONS the runtime (the bounded, terminal escape),
// not a silent proceed.
//
// COST: this test deliberately waits out the production `BARRIER_BOUNDARY_TIMEOUT`
// (~5s) — it cannot be shortened without touching production code (out of scope
// for a test-only commit). It is `#[serial]`, so the ~5s is paid once.
// ===========================================================================

#[test]
#[serial]
fn stalled_peer_poisons_runtime_no_silent_proceed() {
    // Build ONLY context A, give it a barrier expecting 2 participants, but never
    // start the peer. A's first global boundary `arrive` leaves `remaining = 1`
    // (Pending); `wait` then blocks until BARRIER_BOUNDARY_TIMEOUT elapses and
    // returns TimedOut → step() POISONS the runtime and returns (no silent
    // proceed past an un-rendezvoused peer; terminal in every build mode, unlike
    // a debug-only panic). An unopenable generation is a BOUNDED, LOUD,
    // TERMINAL escape.
    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr_a = manager("stalled_a", ix);
    let (cfg_a, fac_a) = context_a_graph();
    // Keep a handle to the runtime's VirtualClock: the poison check returns BEFORE
    // `begin_step`, so a poisoned step never advances the logical clock — a
    // DISTINGUISHING observable for the no-op proof below (a data-starved trace
    // would stay empty WITHOUT the poison check too, so trace length alone can't
    // tell poison-present from poison-absent).
    let clock = Arc::new(VirtualClock::new());
    let mut rt_a = GraphRuntime::build(cfg_a, fac_a, &mgr_a, Arc::clone(&clock))
        .expect("build context A (stalled-peer)");
    let ns = barrier_ns("stalled");
    let owner = Arc::new(MappedBarrier::create_owned(&ns, "g", 2).expect("barrier owner create"));
    rt_a.set_barrier_participant_for_test(owner, MAP_A.to_vec(), 0);

    assert!(!rt_a.is_barrier_failed(), "not poisoned before the timeout");
    // First step: the peer never arrives → wait TimedOut → poison + return.
    rt_a.step(STEP);
    assert!(
        rt_a.is_barrier_failed(),
        "a barrier boundary timeout must POISON the runtime (terminal, not a silent proceed)"
    );
    // A poisoned runtime refuses to step further: a second step is a TRUE no-op.
    // The DISTINGUISHING observable is the logical clock — `step()`'s poison check
    // returns before `begin_step`, so the clock is frozen at the value step 1 left
    // it (one advance, before the timed-out barrier wait). WITHOUT the poison check
    // the second step would re-enter `begin_step` and advance the clock again
    // (and re-run levels 0..g + re-arrive the corrupted generation) — so a frozen
    // clock proves the second step did nothing, where trace length alone (the
    // graph is data-starved here) cannot.
    let clock_after_poison = clock.now_ns();
    let trace_len_after_poison = rt_a.trace().len();
    rt_a.step(STEP);
    assert_eq!(
        clock.now_ns(),
        clock_after_poison,
        "a poisoned runtime must NOT advance the clock (poison returns before begin_step)"
    );
    assert_eq!(
        rt_a.trace().len(),
        trace_len_after_poison,
        "a poisoned runtime must refuse to step (no replay of levels 0..g, no barrier re-arrive)"
    );
}

// ===========================================================================
// Worker self-drop (`leave_barrier_cohort`): a survivor context
// continues at reduced `expected` after a peer LEAVES the cohort; and the
// idempotency / poisoned-no-drop contract.
// ===========================================================================

/// A SURVIVOR-continue in-process pin. Context B runs `N_LEAVE`
/// steps in lockstep, calls `leave_barrier_cohort()` (its worker-side self-drop),
/// then STOPS; context A must continue to `N_TOTAL` steps ALONE (the barrier
/// `expected` shrinks 2→1, so A no longer stalls at each boundary). The HAND oracle:
/// A fires `[n0@L0, n1@L1]` every one of its `N_TOTAL` steps — INCLUDING the ones
/// AFTER B left — and A is NOT poisoned. (Had B's leave failed to drop the barrier
/// slot, A would stall at its next boundary, time out at `BARRIER_BOUNDARY_TIMEOUT`
/// (~5s), poison, and truncate its trace — so both the oracle and `!a_failed` catch
/// a broken drop.) Also pins `leave_barrier_cohort` idempotency (second call false).
#[test]
#[serial]
fn survivor_continues_after_peer_leaves_barrier_cohort() {
    const N_LEAVE: u64 = 2;
    const N_TOTAL: u64 = 6;

    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr_a = manager("survivor_a", ix.clone());
    let mgr_b = manager("survivor_b", ix);

    // Build A FIRST (creates `/blg/ext` + the graph-owned single-writer handoff);
    // then attach the external publisher; then build B (opens the handoff).
    let (cfg_a, fac_a) = context_a_graph();
    let mut rt_a = GraphRuntime::build(cfg_a, fac_a, &mgr_a, Arc::new(VirtualClock::new()))
        .expect("build context A");
    let pubr = mgr_a
        .create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
        .expect("external publisher must attach to /blg/ext");

    let b_observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let (cfg_b, fac_b) = context_b_graph(Arc::clone(&b_observed));
    let mut rt_b = GraphRuntime::build(cfg_b, fac_b, &mgr_b, Arc::new(VirtualClock::new()))
        .expect("build context B");

    let ns = barrier_ns("survivor_continue");
    let owner = Arc::new(MappedBarrier::create_owned(&ns, "g", 2).expect("barrier owner create"));
    let peer = Arc::new(MappedBarrier::open_unowned(&ns, "g").expect("barrier peer open"));
    let probe = Arc::clone(&owner);
    rt_a.set_barrier_participant_for_test(owner, MAP_A.to_vec(), 0);
    rt_b.set_barrier_participant_for_test(peer, MAP_B.to_vec(), 1);

    // A: prime, then publish + step N_TOTAL (alone after B leaves at N_LEAVE).
    let a_handle = thread::spawn(move || {
        let mut pubr = pubr;
        for _ in 0..WARMUP {
            rt_a.step(STEP);
        }
        rt_a.clear_trace();
        for i in 1..=N_TOTAL {
            publish_one(&mut pubr, i as f64);
            rt_a.step(STEP);
        }
        let entries: Vec<TraceEntry> = rt_a.trace().to_vec();
        let failed = rt_a.is_barrier_failed();
        rt_a.shutdown();
        drop(pubr);
        drop(mgr_a);
        (entries, failed)
    });

    // B: prime, step N_LEAVE, then LEAVE the cohort (self-drop) and STOP stepping.
    let b_handle = thread::spawn(move || {
        for _ in 0..WARMUP {
            rt_b.step(STEP);
        }
        rt_b.clear_trace();
        for _ in 0..N_LEAVE {
            rt_b.step(STEP);
        }
        let left = rt_b.leave_barrier_cohort();
        let left_again = rt_b.leave_barrier_cohort(); // idempotent second call
        rt_b.shutdown();
        drop(mgr_b);
        (left, left_again)
    });

    let (a_entries, a_failed) = a_handle.join().expect("context A thread panicked");
    let (b_left, b_left_again) = b_handle.join().expect("context B thread panicked");

    assert!(b_left, "B's first leave_barrier_cohort must perform a drop");
    assert!(
        !b_left_again,
        "a second leave_barrier_cohort is an idempotent no-op (participant taken)"
    );
    assert!(
        !a_failed,
        "A must NOT poison — B's graceful leave dropped the barrier slot so A never stalled"
    );

    // HAND oracle: A fired [n0, n1] every one of its N_TOTAL steps (incl. after B left).
    let a_ids = node_ids(&a_entries);
    let expected_a: Vec<String> = (0..N_TOTAL)
        .flat_map(|_| ["n0".to_string(), "n1".to_string()])
        .collect();
    assert_eq!(
        a_ids, expected_a,
        "A's continued fire sequence must be [n0, n1] × N_TOTAL (the survivor keeps firing after the peer left)"
    );
    assert_eq!(
        probe.expected(),
        1,
        "B's self-drop resized the barrier cohort to the 1 survivor"
    );
}

/// `leave_barrier_cohort` idempotency + the poisoned-no-drop
/// contract. Part 1: a healthy participant leaves (`true`), a second leave is a
/// no-op (`false`), and `expected` shrank 2→1. Part 2: a runtime POISONED by a
/// stalled-peer boundary timeout (reusing test 6's shape) — `leave_barrier_cohort`
/// returns `false` WITHOUT touching the barrier (`expected` stays 2), because a
/// poisoned cohort must never be re-dropped.
#[test]
#[serial]
fn leave_barrier_cohort_idempotent_and_noop_on_poisoned() {
    // Part 1 — idempotency + drop-shrinks-expected (fast, no timeout).
    {
        let ix = cerulion_core::testing::iceoryx_test_config();
        let mgr_a = manager("leave_idem_a", ix);
        let (cfg_a, fac_a) = context_a_graph();
        let mut rt_a = GraphRuntime::build(cfg_a, fac_a, &mgr_a, Arc::new(VirtualClock::new()))
            .expect("build A (idempotent)");
        let ns = barrier_ns("leave_idem");
        let owner = Arc::new(MappedBarrier::create_owned(&ns, "g", 2).expect("barrier owner"));
        let probe = Arc::clone(&owner);
        rt_a.set_barrier_participant_for_test(owner, MAP_A.to_vec(), 0);

        assert!(
            rt_a.leave_barrier_cohort(),
            "first leave performs a drop (returns true)"
        );
        assert!(
            !rt_a.leave_barrier_cohort(),
            "second leave is an idempotent no-op (returns false)"
        );
        assert_eq!(probe.expected(), 1, "the self-drop shrank expected 2→1");
        rt_a.shutdown();
        drop(mgr_a);
    }

    // Part 2 — poisoned runtime: leave is a no-op false, barrier untouched.
    // COST: waits out BARRIER_BOUNDARY_TIMEOUT (~5s), like test 6 (serial → paid once).
    {
        let ix = cerulion_core::testing::iceoryx_test_config();
        let mgr_a = manager("leave_poison_a", ix);
        let (cfg_a, fac_a) = context_a_graph();
        let clock = Arc::new(VirtualClock::new());
        let mut rt_a = GraphRuntime::build(cfg_a, fac_a, &mgr_a, Arc::clone(&clock))
            .expect("build A (poison)");
        let ns = barrier_ns("leave_poison");
        let owner = Arc::new(MappedBarrier::create_owned(&ns, "g", 2).expect("barrier owner"));
        let probe = Arc::clone(&owner);
        rt_a.set_barrier_participant_for_test(owner, MAP_A.to_vec(), 0);

        // Peer never arrives → first boundary wait TimedOut → poison.
        rt_a.step(STEP);
        assert!(
            rt_a.is_barrier_failed(),
            "the boundary timeout must poison the runtime"
        );
        assert!(
            !rt_a.leave_barrier_cohort(),
            "leave on a poisoned runtime is a no-op (returns false)"
        );
        assert_eq!(
            probe.expected(),
            2,
            "a poisoned cohort must NOT be re-dropped (expected untouched)"
        );
        rt_a.shutdown();
        drop(mgr_a);
    }
}

/// The loud-Stale arm of `leave_barrier_cohort`. A
/// participant whose OWN next-arrival generation has been overtaken by the shared
/// generation is an invariant violation at clean shutdown — `leave_barrier_cohort`
/// must detect it (`DropAttempt::Stale`), return FALSE (the loud `tracing::error!`
/// path, not a phantom "left" success), and touch NOTHING (`expected` unchanged —
/// a Stale drop is a FULL no-op on the barrier counters).
///
/// Setup: `expected = 1`, so the runtime's participant alone completes every
/// boundary — one `step()` crosses all `GLOBAL_LEVELS` (5) generations instantly
/// (no timeout, no poison), leaving the participant's gen at 5. The TEST then opens
/// the SAME barrier (`open_unowned`) and, as the sole cohort member, arrives three
/// times to drive the shared generation 5→8 — strictly AHEAD of the participant.
#[test]
#[serial]
fn leave_barrier_cohort_stale_gen_returns_false_and_touches_nothing() {
    use cerulion_core::barrier::ArriveOutcome;

    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr_a = manager("leave_stale_a", ix);
    let (cfg_a, fac_a) = context_a_graph();
    let mut rt_a = GraphRuntime::build(cfg_a, fac_a, &mgr_a, Arc::new(VirtualClock::new()))
        .expect("build A (stale)");
    let ns = barrier_ns("leave_stale");
    let owner = Arc::new(MappedBarrier::create_owned(&ns, "g", 1).expect("barrier owner"));
    let probe = Arc::clone(&owner);
    rt_a.set_barrier_participant_for_test(owner, MAP_A.to_vec(), 0);

    // One step: the sole participant opens each of the 5 global boundaries itself
    // → participant gen = 5, shared generation = 5, no poison.
    rt_a.step(STEP);
    assert!(
        !rt_a.is_barrier_failed(),
        "no poison on the sole-member step"
    );
    assert_eq!(
        probe.current_generation(),
        5,
        "one step crossed 5 boundaries"
    );

    // Drive the SHARED generation ahead of the participant's gen (5): each arrive
    // as the sole cohort member opens its generation (5→6→7→8).
    let ext = MappedBarrier::open_unowned(&ns, "g").expect("peer open of the same barrier");
    for g in 5..8u64 {
        assert_eq!(ext.arrive(g), ArriveOutcome::Opened(g + 1));
    }
    assert_eq!(
        probe.current_generation(),
        8,
        "shared generation driven to 8"
    );

    // The participant's own next-arrival gen (5) is now STALE → leave must return
    // FALSE (the loud-error path) and leave the counters untouched.
    assert!(
        !rt_a.leave_barrier_cohort(),
        "a stale own-generation must be detected and reported false — not a phantom drop"
    );
    assert_eq!(
        probe.expected(),
        1,
        "the stale attempt is a FULL no-op on expected (nothing decremented)"
    );
    assert_eq!(
        probe.current_generation(),
        8,
        "generation untouched by the stale attempt"
    );

    rt_a.shutdown();
    drop(mgr_a);
}

// ===========================================================================
// The PRODUCTION build path that wires the
// handed-quantum + barrier-participant combo
// (`build_live_deterministic_with_manager_and_barrier`). Tests 1-6 above prove
// the barrier ENGINE via the test-only `set_barrier_participant_for_test`;
// these prove the production entrypoint that SETS both fields so the pre-built
// engine fires the cross-process handed-quantum path.
// ===========================================================================

// ---------------------------------------------------------------------------
// B1 — the new fn wires the handed quantum AND a single-participant barrier.
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn new_fn_wires_handed_quantum_and_barrier_single_participant() {
    // ONE context (the full 5-node monolith chain) built through the production
    // (e) entrypoint, with a SINGLE-participant barrier (expected = 1) so every
    // global-level boundary opens immediately (no peer ⇒ no deadlock) and a
    // CONTIGUOUS global-level map matching the chain's 5 levels. The handed
    // quantum is 4ms — DELIBERATELY NOT the 1ms the single-process path would
    // derive — so the assertion (consecutive fire_time_ns advance by exactly 4ms)
    // proves `live_gating_quantum = Some(handed)` took effect AND the barrier(1)
    // opened each boundary (a deadlock would hang past LIVE_TIMEOUT / the 5s
    // barrier timeout). The 4ms is a HAND oracle, not a self-compare.
    let ix = cerulion_core::testing::iceoryx_test_config();
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager_with_clock("b1_handed", ix, Arc::clone(&clock));

    let observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let (config, factories) = monolith_graph(Arc::clone(&observed));

    let ns = barrier_ns("b1_handed");
    let barrier =
        Arc::new(MappedBarrier::create_owned(&ns, "g", 1).expect("single-participant barrier"));
    let handed = Duration::from_millis(4);
    // Contiguous bijection onto the monolith's 5 local levels (n0=L0 … n4=L4).
    let map = vec![Some(0), Some(1), Some(2), Some(3), Some(4)];

    let mut rt = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        barrier,
        map.clone(),
        // No level takes the extra mid-level rendezvous in this
        // fixture, so the run keeps exactly one barrier generation per global
        // level — the earlier law every generation pin here asserts.
        vec![false; map.len()],
        0,
        handed,
        // In-process test harness — no cross-group provisioning union.
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build single-participant handed-quantum runtime via the (e) entrypoint");

    let mut pubr = mgr
        .create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
        .expect("external publisher must attach to /blg/ext");

    // Priming (no publish) establishes the external /blg/ext connection; nothing
    // fires. Each no-data live iteration may sleep up to LIVE_TIMEOUT.
    for _ in 0..WARMUP {
        rt.run_live_step_once_for_test(LIVE_TIMEOUT);
    }
    rt.clear_trace();

    const STEPS: u64 = 4;
    for i in 1..=STEPS {
        publish_one(&mut pubr, i as f64);
        rt.run_live_step_once_for_test(LIVE_TIMEOUT);
    }

    // n0 fires once per measured step (the chain collapses within one step). Its
    // consecutive fire_time_ns must advance by EXACTLY the handed 4ms quantum.
    let n0_times: Vec<u64> = rt
        .trace()
        .iter()
        .filter(|e| &*e.node_id == "n0")
        .map(|e| e.fire_time_ns)
        .collect();
    let observed_vals = observed.lock().unwrap().clone();
    // T10: a healthy multi-step run must never poison the runtime. An earlier
    // version only asserted `!is_barrier_failed()` on a fresh, unstepped runtime; this pins
    // "healthy through normal operation" AFTER the full WARMUP + measured run.
    // `shutdown` CONSUMES `rt`, so this read must PRECEDE it (the n0_times /
    // observed_vals assertions below are read-only on already-collected vecs, so
    // barrier-failed state is identical here vs after them).
    assert!(
        !rt.is_barrier_failed(),
        "a healthy multi-step run must never poison the runtime"
    );
    rt.shutdown();
    drop(pubr);
    drop(mgr);

    assert_eq!(
        n0_times.len() as u64,
        STEPS,
        "n0 must fire exactly once per measured step (barrier(1) opened every \
         boundary; a deadlock would have hung)"
    );
    for w in n0_times.windows(2) {
        assert_eq!(
            w[1] - w[0],
            4_000_000,
            "the HANDED 4ms quantum must advance fire_time_ns by exactly 4_000_000 ns \
             per step — proving live_gating_quantum = Some(handed) took effect (NOT the \
             1ms a single-process path would derive)"
        );
    }
    // Sanity: the data actually flowed through the chain (n0 fired on real input).
    assert_eq!(
        observed_vals,
        expected_values(STEPS),
        "the sink must observe exactly 1.0..=STEPS — the chain fired on real data"
    );
}

// ---------------------------------------------------------------------------
// B2 — the new fn rejects a non-contiguous / non-bijective global-level map.
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn new_fn_rejects_non_contiguous_global_level_map() {
    // Context A has 2 local levels (n0=L0, n1=L1). A map whose non-None entries
    // are `[0, 2]` (a GAP — skips local level 1) is NOT a bijection onto 0..2, so
    // `install_barrier_participant` must reject it with a GraphError naming the
    // bijection contract (the production early-return half of the test-only
    // assert). The clock matches (same Arc) so the build reaches the install step.
    let ix = cerulion_core::testing::iceoryx_test_config();
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager_with_clock("b2_noncontig", ix, Arc::clone(&clock));
    let (config, factories) = context_a_graph();

    let ns = barrier_ns("b2_noncontig");
    let barrier = Arc::new(MappedBarrier::create_owned(&ns, "g", 1).expect("barrier"));
    // 3 global levels, non-None entries [Some(0), Some(2)] ⇒ locals [0,2] ≠ [0,1].
    let bad_map = vec![Some(0), None, Some(2)];

    // NOTE: `GraphRuntime` is not `Debug`, so the Ok arm cannot use `.expect_err`
    // — match explicitly.
    let result = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        barrier,
        bad_map.clone(),
        // No level takes the extra mid-level rendezvous in this
        // fixture, so the run keeps exactly one barrier generation per global
        // level — the earlier law every generation pin here asserts.
        vec![false; bad_map.len()],
        0,
        Duration::from_millis(4),
        // In-process test harness — no cross-group provisioning union.
        CrossProcessWiring::requirements_only(None),
    );
    match result {
        Ok(_) => panic!("a non-contiguous global-level map must be rejected"),
        Err(TransportError::GraphError { reason }) => assert!(
            reason.contains("bijection"),
            "the rejection must name the bijection contract, got: {reason}"
        ),
        Err(other) => panic!("expected TransportError::GraphError, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// B3 — the new fn rejects a transport/clock contract mismatch.
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn new_fn_rejects_clock_contract_mismatch() {
    // The transport is built with clock C1; the build fn is handed a DIFFERENT
    // clock C2. The shared `build_live_deterministic_core`'s Arc::ptr_eq check must
    // reject it with InvalidTransportConfig (a mismatched transport would silently
    // drift QoS anchors + sample(N) decimation off the gating timeline — Principle
    // #7).
    let ix = cerulion_core::testing::iceoryx_test_config();
    let clock_transport = Arc::new(VirtualClock::new()); // C1
    let mgr = manager_with_clock("b3_clockmismatch", ix, Arc::clone(&clock_transport));
    let (config, factories) = context_a_graph();

    let ns = barrier_ns("b3_clockmismatch");
    let barrier = Arc::new(MappedBarrier::create_owned(&ns, "g", 1).expect("barrier"));
    let map = vec![Some(0), Some(1)]; // valid bijection for context A's 2 levels
    let clock_build = Arc::new(VirtualClock::new()); // C2 — a DIFFERENT allocation

    // NOTE: `GraphRuntime` is not `Debug`, so the Ok arm cannot use `.expect_err`
    // — match explicitly.
    let result = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        config,
        factories,
        &mgr,
        clock_build,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        barrier,
        map.clone(),
        // No level takes the extra mid-level rendezvous in this
        // fixture, so the run keeps exactly one barrier generation per global
        // level — the earlier law every generation pin here asserts.
        vec![false; map.len()],
        0,
        Duration::from_millis(4),
        // In-process test harness — no cross-group provisioning union.
        CrossProcessWiring::requirements_only(None),
    );
    match result {
        Ok(_) => panic!("a transport built with a different clock must be rejected"),
        Err(TransportError::InvalidTransportConfig { reason }) => assert!(
            reason.contains("SAME VirtualClock"),
            "a clock-contract mismatch must name the SAME-VirtualClock contract \
             (mirrors B2's bijection substring check — a future unrelated \
             InvalidTransportConfig must NOT pass this spuriously), got: {reason}"
        ),
        Err(other) => panic!("expected TransportError::InvalidTransportConfig, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// B4 — THE FIREWALL PIN via the production (e) entrypoint: two contexts built
// through `build_live_deterministic_with_manager_and_barrier`, sharing one
// barrier + the same handed quantum, merge to the HAND oracle.
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn two_context_split_via_build_fn_merges_to_oracle() {
    const HANDED_NS: u64 = 4_000_000; // 4ms — the shared handed quantum below
    let split = run_two_context_barrier_live("b4_firewall", Duration::from_millis(4), N);

    // Each context fired the right COUNT (A: n0,n1 per step; B: n2,n3,n4 per step).
    assert_eq!(
        split.a_entries.len() as u64,
        2 * N,
        "context A must fire n0,n1 once each per step"
    );
    assert_eq!(
        split.b_entries.len() as u64,
        3 * N,
        "context B must fire n2,n3,n4 once each per step"
    );

    // Reconstruct the global fire sequence via the PRODUCTION merge.
    let merged = merge_two_contexts(&split.a_entries, &split.b_entries);
    assert_eq!(
        merged.len() as u64,
        5 * N,
        "the merged trace must hold all 5 fires per step"
    );

    // (1) NODE-ID firewall — the merged sequence equals the HAND oracle
    // [n0,n1,n2,n3,n4] × N (NOT a self-compare against the monolith, which uses a
    // different 1ms clock model — this asserts the absolute expected content).
    assert_eq!(
        node_ids(&merged),
        expected_fire_sequence(N),
        "the production-(e)-built 2-context fire sequence must equal the hand oracle \
         [n0,n1,n2,n3,n4] × N — the barrier is a WHEN-gate on level advance, never a \
         change to the fire set/order (Principle #7)"
    );

    // (2) GLOBAL-LEVEL firewall — the merged sequence fires global levels 0..4 in
    // order each step (hand oracle). The merge sorts by (step, global_level, …),
    // so a scrambled node-id sequence would mean a wrong global_level; pinning
    // both is belt-and-suspenders against a level-stamping regression.
    assert_eq!(
        global_levels(&merged),
        expected_global_levels(N),
        "the merged trace must fire global levels 0,1,2,3,4 in order each step"
    );

    // (3) STEP + FIRE_TIME firewall — within each logical step all 5 fires share
    // one `step` AND one deterministic `fire_time_ns` (the two contexts' gating
    // clocks advance in lockstep by the SAME handed quantum); across steps both
    // advance by exactly one step / one handed quantum. Delta-based ⇒ robust to
    // the WARMUP clock offset, and proves the HANDED 4ms quantum drives the merged
    // cross-process timeline (NOT a tautology — 4ms is the hand-known schedule).
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
                 across the merged cross-process trace"
            );
        }
    }

    // (4) DATA-FLOW firewall — each value 1..=N crossed the REAL cross-context
    // iceoryx2 handoff in order (the hand oracle, NOT a self-compare).
    assert_eq!(
        split.b_observed,
        expected_values(N),
        "the split sink must observe exactly 1.0..=N in order — the cross-process \
         barrier's happens-before makes B's n2 read A's n1 same-step publish"
    );

    // (5) LOCKSTEP — the shared barrier crossed GLOBAL_LEVELS generations per step
    // across BOTH warmup and measured steps (every boundary, incl. None entries,
    // rendezvoused).
    assert_eq!(
        split.final_generation,
        (WARMUP + N) * GLOBAL_LEVELS,
        "the shared generation must advance by GLOBAL_LEVELS per step in lockstep"
    );
}

// ===========================================================================
// Review coverage gaps: order- and shape-sensitivity of
// the global-level-map bijection check (siblings to B2), the handed-quantum
// floor guard, and the LIVE-seam poison honoring (the live analogue of test 6).
// ===========================================================================

// ---------------------------------------------------------------------------
// T3 — the new fn rejects a REVERSED (out-of-order) global-level map.
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn new_fn_rejects_reversed_global_level_map() {
    // Sibling to `new_fn_rejects_non_contiguous_global_level_map` (B2), but pins
    // ORDER-sensitivity rather than the GAP case. Context A has 2 local levels
    // (n0=L0, n1=L1). A map whose non-None entries are `[1, 0]` (reversed) COVERS
    // every local level but is NOT strictly increasing, so it is not a bijection
    // onto 0..2 and `install_barrier_participant` must reject it. The clock matches
    // (same Arc) and the quantum is valid, so the bijection contract is the only
    // one that can fire.
    let ix = cerulion_core::testing::iceoryx_test_config();
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager_with_clock("t3_reversed", ix, Arc::clone(&clock));
    let (config, factories) = context_a_graph();

    let ns = barrier_ns("t3_reversed");
    let barrier = Arc::new(MappedBarrier::create_owned(&ns, "g", 1).expect("barrier"));
    // 2 global levels, non-None entries [Some(1), Some(0)] ⇒ locals [1,0] ≠ [0,1].
    let reversed_map = vec![Some(1), Some(0)];

    // NOTE: `GraphRuntime` is not `Debug`, so the Ok arm cannot use `.expect_err`
    // — match explicitly.
    let result = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        barrier,
        reversed_map.clone(),
        // No level takes the extra mid-level rendezvous in this
        // fixture, so the run keeps exactly one barrier generation per global
        // level — the earlier law every generation pin here asserts.
        vec![false; reversed_map.len()],
        0,
        Duration::from_millis(4),
        // In-process test harness — no cross-group provisioning union.
        CrossProcessWiring::requirements_only(None),
    );
    match result {
        Ok(_) => panic!("a reversed global-level map must be rejected"),
        Err(TransportError::GraphError { reason }) => assert!(
            reason.contains("bijection"),
            "the rejection must name the bijection contract, got: {reason}"
        ),
        Err(other) => panic!("expected TransportError::GraphError, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// T4 — the new fn rejects an ALL-None global-level map.
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn new_fn_rejects_all_none_global_level_map() {
    // Same context-A setup as T3/B2, but an ALL-None map: its non-None entries are
    // EMPTY (`[] ≠ [0,1]`), so it cannot be a bijection onto the 2 local levels.
    // A cheap SHAPE-variant of the same `install_barrier_participant` reject branch
    // (distinct from B2's gap and T3's reversal).
    let ix = cerulion_core::testing::iceoryx_test_config();
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager_with_clock("t4_allnone", ix, Arc::clone(&clock));
    let (config, factories) = context_a_graph();

    let ns = barrier_ns("t4_allnone");
    let barrier = Arc::new(MappedBarrier::create_owned(&ns, "g", 1).expect("barrier"));
    let all_none_map = vec![None, None];

    let result = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        barrier,
        all_none_map.clone(),
        // No level takes the extra mid-level rendezvous in this
        // fixture, so the run keeps exactly one barrier generation per global
        // level — the earlier law every generation pin here asserts.
        vec![false; all_none_map.len()],
        0,
        Duration::from_millis(4),
        // In-process test harness — no cross-group provisioning union.
        CrossProcessWiring::requirements_only(None),
    );
    match result {
        Ok(_) => panic!("an all-None global-level map must be rejected"),
        Err(TransportError::GraphError { reason }) => assert!(
            reason.contains("bijection"),
            "the rejection must name the bijection contract, got: {reason}"
        ),
        Err(other) => panic!("expected TransportError::GraphError, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// T6 — the new fn rejects a zero / sub-1ms handed quantum.
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn new_fn_rejects_zero_and_sub_1ms_handed_quantum() {
    // The handed-quantum floor (>= 1ms) is guarded at the TOP of the build fn, so
    // it returns Err before the graph/services are built. Two sub-floor values —
    // exactly ZERO and a sub-millisecond 500µs — must BOTH be rejected with the
    // ">= 1ms" contract. Everything else (clock match + a valid contiguous map) is
    // VALID, so the quantum guard is the only contract that can fire. Each
    // iteration uses a fresh isolated SHM root + barrier namespace so they never
    // collide.
    for (i, bad) in [Duration::ZERO, Duration::from_micros(500)]
        .into_iter()
        .enumerate()
    {
        let ix = cerulion_core::testing::iceoryx_test_config();
        let clock = Arc::new(VirtualClock::new());
        let mgr = manager_with_clock("t6_subquantum", ix, Arc::clone(&clock));
        let (config, factories) = context_a_graph();

        let ns = barrier_ns(&format!("t6_subquantum_{i}"));
        let barrier = Arc::new(MappedBarrier::create_owned(&ns, "g", 1).expect("barrier"));

        let result = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
            config,
            factories,
            &mgr,
            Arc::clone(&clock),
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            barrier,
            vec![Some(0), Some(1)],
            // No level takes the extra mid-level rendezvous in this
            // fixture, so the run keeps exactly one barrier generation per global
            // level — the earlier law every generation pin here asserts.
            vec![false; 2],
            0,
            bad,
            // In-process test harness — no cross-group provisioning union.
            CrossProcessWiring::requirements_only(None),
        );
        match result {
            Ok(_) => panic!("a sub-1ms handed quantum ({bad:?}) must be rejected"),
            Err(TransportError::InvalidTransportConfig { reason }) => assert!(
                reason.contains(">= 1ms"),
                "the rejection must name the >= 1ms quantum floor (got {bad:?}): {reason}"
            ),
            Err(other) => panic!("expected InvalidTransportConfig for {bad:?}, got {other:?}"),
        }
    }
}

// ---------------------------------------------------------------------------
// T7 — the LIVE seam honors the barrier poison (the live analogue of test 6).
//
// COST: this deliberately waits out the production `BARRIER_BOUNDARY_TIMEOUT`
// (~5s). It is `#[serial]`, so the cost is paid once.
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn new_fn_live_seam_honors_barrier_poison() {
    // The LIVE-seam analogue of test 6 (`stalled_peer_poisons_runtime_no_silent_
    // proceed`), but ENTERED through the PRODUCTION (e) build fn and driven on the
    // deterministic-LIVE seam (`run_live_step_once_for_test`) instead of the polled
    // `step()`. ONE context built with a barrier expecting 2 participants and NO
    // peer ever arriving: the first live iteration arrives at the first global
    // boundary, waits, times out at BARRIER_BOUNDARY_TIMEOUT (~5s) → poisons.
    //
    // A poisoned runtime then refuses to advance: the second live iteration's
    // poison check returns BEFORE `begin_step`, so the logical clock stays frozen —
    // the SAME distinguishing observable test 6 uses (a data-starved trace would
    // stay empty WITHOUT the poison check too, so trace length alone can't tell
    // poison-present from poison-absent; the frozen clock can).
    let ix = cerulion_core::testing::iceoryx_test_config();
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager_with_clock("t7_livepoison", ix, Arc::clone(&clock));
    let (config, factories) = context_a_graph();

    let ns = barrier_ns("t7_livepoison");
    // expected = 2, but no peer is ever built — the first boundary `arrive` leaves
    // `remaining = 1`, the `wait` times out at the ~5s production boundary.
    let barrier = Arc::new(MappedBarrier::create_owned(&ns, "g", 2).expect("barrier owner create"));
    // Valid contiguous bijection for context A's 2 local levels + a >= 1ms handed
    // quantum, so the BUILD succeeds and the ONLY failure is the stalled peer.
    let map = vec![Some(0), Some(1)];

    let mut rt = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        barrier,
        map.clone(),
        // No level takes the extra mid-level rendezvous in this
        // fixture, so the run keeps exactly one barrier generation per global
        // level — the earlier law every generation pin here asserts.
        vec![false; map.len()],
        0,
        Duration::from_millis(4),
        // In-process test harness — no cross-group provisioning union.
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build single context via the (e) entrypoint (valid clock/map/quantum)");

    assert!(
        !rt.is_barrier_failed(),
        "not poisoned before the first live step"
    );

    // First live iteration: the peer never arrives → arrive+wait TimedOut at the
    // ~5s boundary → poison.
    rt.run_live_step_once_for_test(LIVE_TIMEOUT);
    assert!(
        rt.is_barrier_failed(),
        "a barrier boundary timeout on the LIVE seam must POISON the runtime \
         (terminal, not a silent proceed) — proving the (e) entrypoint's live path \
         honors the poison, distinct from test 6's polled step()"
    );

    // A poisoned runtime refuses to advance: the second live iteration is a TRUE
    // no-op. The DISTINGUISHING observable is the logical clock — the poison check
    // returns before `begin_step`, so the clock is frozen at the value the first
    // iteration left it (one advance, before the timed-out barrier wait).
    let clock_after_poison = clock.now_ns();
    let trace_len_after_poison = rt.trace().len();
    rt.run_live_step_once_for_test(LIVE_TIMEOUT);
    assert_eq!(
        clock.now_ns(),
        clock_after_poison,
        "a poisoned runtime must NOT advance the clock on the LIVE seam \
         (poison returns before begin_step)"
    );
    assert_eq!(
        rt.trace().len(),
        trace_len_after_poison,
        "a poisoned runtime must refuse to step (no replay of levels 0..g, no \
         barrier re-arrive)"
    );
}

// ===========================================================================
// Further review coverage gaps: the boundary controls
// for BOTH handed-quantum guards (the floor + the per-context "coarser" guard)
// and a post-success liveness pin (T10, folded into B1 above).
// ===========================================================================

// ---------------------------------------------------------------------------
// T8 — the new fn ACCEPTS a handed quantum at EXACTLY the 1ms floor (the
// boundary control for guard (a), which T6 exercises from below).
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn new_fn_accepts_handed_quantum_at_exactly_1ms() {
    // The BOUNDARY CONTROL for guard (a)'s `>= 1ms` floor (T6 rejects ZERO + 500µs
    // from below). At EXACTLY 1ms the floor guard must NOT fire — the guard is
    // `handed_quantum < 1ms` (`<`, not `<=`) — so the build SUCCEEDS and the handed
    // 1ms quantum drives the gating clock. The /blg monolith relays + sink are pure
    // DATA-driven, so this context's `tightest_timing_ns()` is `None` ⇒ guard (b)
    // (handed > local tightest) SKIPS and 1ms is accepted cleanly. Mirrors B1's
    // fire_time read with the quantum pinned at the floor instead of 4ms; the
    // 1_000_000 ns delta is a HAND oracle, not a self-compare. Changing `<` to
    // `<=` in guard (a) fails this test.
    let ix = cerulion_core::testing::iceoryx_test_config();
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager_with_clock("t8_1ms", ix, Arc::clone(&clock));

    let observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let (config, factories) = monolith_graph(Arc::clone(&observed));

    let ns = barrier_ns("t8_1ms");
    let barrier =
        Arc::new(MappedBarrier::create_owned(&ns, "g", 1).expect("single-participant barrier"));
    let handed = Duration::from_millis(1); // EXACTLY the floor — must be accepted.
                                           // Contiguous bijection onto the monolith's 5 local levels (n0=L0 … n4=L4).
    let map = vec![Some(0), Some(1), Some(2), Some(3), Some(4)];

    let mut rt = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        barrier,
        map.clone(),
        // No level takes the extra mid-level rendezvous in this
        // fixture, so the run keeps exactly one barrier generation per global
        // level — the earlier law every generation pin here asserts.
        vec![false; map.len()],
        0,
        handed,
        // In-process test harness — no cross-group provisioning union.
        CrossProcessWiring::requirements_only(None),
    )
    .expect("a handed quantum of EXACTLY 1ms must be accepted (guard (a) is `<`, not `<=`)");

    let mut pubr = mgr
        .create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
        .expect("external publisher must attach to /blg/ext");

    // Priming (no publish) establishes the external /blg/ext connection; nothing
    // fires. Each no-data live iteration may sleep up to LIVE_TIMEOUT.
    for _ in 0..WARMUP {
        rt.run_live_step_once_for_test(LIVE_TIMEOUT);
    }
    rt.clear_trace();

    const STEPS: u64 = 4;
    for i in 1..=STEPS {
        publish_one(&mut pubr, i as f64);
        rt.run_live_step_once_for_test(LIVE_TIMEOUT);
    }

    // n0 fires once per measured step (the chain collapses within one step). Its
    // consecutive fire_time_ns must advance by EXACTLY the handed 1ms quantum.
    let n0_times: Vec<u64> = rt
        .trace()
        .iter()
        .filter(|e| &*e.node_id == "n0")
        .map(|e| e.fire_time_ns)
        .collect();
    let observed_vals = observed.lock().unwrap().clone();
    rt.shutdown();
    drop(pubr);
    drop(mgr);

    assert_eq!(
        n0_times.len() as u64,
        STEPS,
        "n0 must fire exactly once per measured step (the 1ms-quantum build \
         succeeded and barrier(1) opened every boundary; a deadlock would have hung)"
    );
    for w in n0_times.windows(2) {
        assert_eq!(
            w[1] - w[0],
            1_000_000,
            "the HANDED 1ms quantum (the floor, ACCEPTED) must advance fire_time_ns \
             by exactly 1_000_000 ns per step — proving guard (a) does NOT reject a \
             quantum AT the 1ms floor (`<`, not `<=`)"
        );
    }
    // Sanity: the data actually flowed through the chain (n0 fired on real input).
    assert_eq!(
        observed_vals,
        expected_values(STEPS),
        "the sink must observe exactly 1.0..=STEPS — the chain fired on real data"
    );
}

// ---------------------------------------------------------------------------
// T9 — the new fn rejects a handed quantum COARSER than this context's own
// declared tightest timing (guard (b)); a quantum == that tightest is ACCEPTED.
//
// The pure-data /blg relays have `tightest_timing_ns() == None`, so they cannot
// arm guard (b). This needs a context with a DECLARED local timing — a single
// `period_ms = 2` source node, whose Period policy makes the owning context's
// `tightest_timing_ns()` == Some(2_000_000).
// ---------------------------------------------------------------------------

/// A single-output `period_ms = 2` source (no inputs). Its Period policy gives
/// the owning context a declared local tightest timing of 2ms (Some(2_000_000)) —
/// the ONLY way to arm guard (b) (the pure-data ChainRelay/ChainSink report
/// `None`). T9-only fixture.
#[cerulion_node(period_ms = 2)]
#[derive(Default)]
struct PeriodSource {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl PeriodSource {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        Ok(())
    }
}

/// A single-node `period_ms = 2` source graph (prefix `blgp`). The source has no
/// inputs → it levelizes to one DAG level (L0), so the global-level map is
/// `vec![Some(0)]`. `tightest_timing_ns()` == Some(2_000_000), arming guard (b).
/// The output `/blgp/p0/out` has no consumer — fine for a graph-owned single
/// writer (T7 builds `context_a_graph` whose `/blga/n1/out` is likewise
/// consumer-less).
fn period_source_graph() -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "barrier_gate_period_src".to_string(),
        prefix: "blgp".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "p0".to_string(),
            node_type: "period_source".to_string(),
            inputs: vec![],
            outputs: vec![out_def("out")],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("p0".to_string(), Box::new(PeriodSourceEntry::new()));
    (config, factories)
}

#[test]
#[serial]
fn new_fn_rejects_handed_quantum_coarser_than_local_tightest() {
    // (1) REJECT — a handed quantum of 8ms is COARSER than this context's own
    // declared tightest timing (the 2ms Period). That is impossible for a correct
    // supervisor (the GLOBAL graph tightest is the MIN over all groups, so it
    // cannot exceed any group's local tightest), so the build must reject it with
    // InvalidTransportConfig naming the COARSER contract. Everything else (clock
    // match, valid 1-level bijection map, >= 1ms floor) is VALID, so guard (b) is
    // the only contract that can fire.
    {
        let ix = cerulion_core::testing::iceoryx_test_config();
        let clock = Arc::new(VirtualClock::new());
        let mgr = manager_with_clock("t9_coarser", ix, Arc::clone(&clock));
        let (config, factories) = period_source_graph();

        let ns = barrier_ns("t9_coarser");
        let barrier = Arc::new(MappedBarrier::create_owned(&ns, "g", 1).expect("barrier"));

        // NOTE: `GraphRuntime` is not `Debug`, so the Ok arm cannot use `.expect_err`
        // — match explicitly.
        let result = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
            config,
            factories,
            &mgr,
            Arc::clone(&clock),
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            barrier,
            vec![Some(0)],
            // No level takes the extra mid-level rendezvous in this
            // fixture, so the run keeps exactly one barrier generation per global
            // level — the earlier law every generation pin here asserts.
            vec![false; 1],
            0,
            Duration::from_millis(8), // > the 2ms local tightest ⇒ COARSER ⇒ reject
            // In-process test harness — no cross-group provisioning union.
            CrossProcessWiring::requirements_only(None),
        );
        match result {
            Ok(_) => {
                panic!(
                    "a handed quantum (8ms) coarser than the local 2ms tightest must be rejected"
                )
            }
            Err(TransportError::InvalidTransportConfig { reason }) => assert!(
                reason.contains("COARSER"),
                "the rejection must name the COARSER contract (case-sensitive), got: {reason}"
            ),
            Err(other) => panic!("expected InvalidTransportConfig (COARSER), got {other:?}"),
        }
    }

    // (2) BOUNDARY CONTROL — a handed quantum of EXACTLY 2ms (== the local tightest)
    // must be ACCEPTED: guard (b) is `>`, not `>=`. A fresh isolated SHM root +
    // barrier namespace so it never collides with the reject arm above. The build
    // succeeding at the boundary proves the guard fires ONLY on STRICTLY coarser
    // quanta (and that a correctly-handed quantum at the boundary is honored). This
    // ALSO confirms `tightest_timing_ns()` is Some(2_000_000) here — an 8ms reject
    // with a 2ms accept is only possible if the local tightest is exactly 2ms.
    {
        let ix = cerulion_core::testing::iceoryx_test_config();
        let clock = Arc::new(VirtualClock::new());
        let mgr = manager_with_clock("t9_boundary", ix, Arc::clone(&clock));
        let (config, factories) = period_source_graph();

        let ns = barrier_ns("t9_boundary");
        let barrier = Arc::new(MappedBarrier::create_owned(&ns, "g", 1).expect("barrier"));

        let rt = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
            config,
            factories,
            &mgr,
            Arc::clone(&clock),
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            barrier,
            vec![Some(0)],
            // No level takes the extra mid-level rendezvous in this
            // fixture, so the run keeps exactly one barrier generation per global
            // level — the earlier law every generation pin here asserts.
            vec![false; 1],
            0,
            Duration::from_millis(2), // == the 2ms local tightest ⇒ NOT coarser ⇒ accept
            // In-process test harness — no cross-group provisioning union.
            CrossProcessWiring::requirements_only(None),
        )
        .expect(
            "a handed quantum EXACTLY == the local 2ms tightest must be accepted \
             (guard (b) is `>`, not `>=`)",
        );
        rt.shutdown();
        drop(mgr);
    }
}
