// SPDX-License-Identifier: AGPL-3.0-only
//! WITHIN-LEVEL RAYON PARALLEL FIRE determinism gate, over
//! real iceoryx2.
//!
//! `GraphRuntime::step` fires a DAG level's decided nodes IN
//! PARALLEL on a build-time `rayon::ThreadPool` (`tick_decided_parallel`):
//! `decide_fires` (serial) → fire-gated step-boundary snapshot (serial) →
//! `tick_decided_parallel(decisions, &fire_pool, &serial_fire_node_ids)`.
//! The multi-fire path fills a reusable index-keyed scratch (no
//! per-step HashMap), fires "serial-gated" nodes FIRST on the calling thread
//! (PASS 1), then fires the REST — SERIALLY when the level is narrow (< 8
//! non-serial-gated REST fires — the gate keys on `rest_fire_count`, the nodes
//! that actually fan out, NOT the total decided count) or via
//! `pool.install(par_values_mut().enumerate().for_each(..))`
//! when it is wide (PASS 2) — each into its OWN reusable per-node
//! `trace_fragment`, then MERGES the fragments by DRAINING `decisions` in
//! decision (pos) order (PASS 3) → BYTE-IDENTICAL to the serial fire order. The
//! flat `Scheduler::step` path stays serial + byte-identical.
//!
//! The thread count is read ONCE at build from env `CERULION_FIRE_THREADS` (if
//! set and > 0) ELSE `min(available_parallelism, max_level_width).max(1)`. So
//! these tests set `CERULION_FIRE_THREADS` BEFORE `build_for_test` to FORCE a
//! real (4-thread) parallel fire vs a forced-serial (1-thread) merge through the
//! SAME `tick_decided_parallel` code path. Env is process-global → env-forcing
//! tests carry an RAII guard + `#[serial]` (matches `env_snapshot_isolation_test`
//! / `chunk_c_ffi_codes_3_4_test`'s `EnvVarGuard` pattern).
//!
//! What this file pins:
//! - **Test 1 (`parallel_serial_flat_traces_byte_identical`):** THE rayon
//!   byte-identity gate. A WIDE level (16 `Period(10)` sources, no DAG edges →
//!   one wide level 0) produces a byte-equal `TraceEntry` stream under THREE
//!   drivers: real parallel (`THREADS=4`), forced serial-merge (`THREADS=1`)
//!   through the same code path, AND a bare `Scheduler::step` flat path with the
//!   same ids/order/Period policy. 16 ≥ 2×4 so parallelism is genuine.
//! - **Test 2 (`parallel_fire_deterministic_across_many_runs`):** the wide level
//!   under `THREADS=4` run once for a reference trace, then 40 FRESH runtimes
//!   (each `build_for_test`, per-test SHM root) must each reproduce it
//!   bit-for-bit. Fire ORDER varies run-to-run under real threads; the merged
//!   trace must not (catches merge-order / shared-state nondeterminism).
//! - **Test 3 (`snapshot_freeze_holds_under_real_parallel_threads`):** the
//!   headline "snapshot earns its keep under concurrency" race test. 4 same-level
//!   read `Period(10)` producers + 4 unconsumed filler `Period(10)` producers + a
//!   `Period(10)` macro consumer with 4 plain (snapshotted) `#[input]`s (one per
//!   read producer) = a 9-wide level (9 ≥ the parallel threshold of 8 → the REST
//!   genuinely fires via `par_values_mut`; without the filler the 5-wide level
//!   would route NARROW and the "under REAL parallel threads" claim would be
//!   vacuous). All fire on real threads (`THREADS=4`). The consumer ALWAYS reads
//!   each read producer's PRIOR-step value (`v - 1`), NEVER the same-step value
//!   (`v`) — across many steps + many runs. WITHOUT 4a's level-boundary snapshot
//!   this is a genuine data race (producers publish on other threads while the
//!   consumer reads); WITH it, rock-solid. The 4 filler producers only widen the
//!   level to engage rayon — they are not read.
//! - **Test 4 (`period_catchup_byte_identical_under_parallel`):** a wide level
//!   stepped `30ms` so each `Period(10)` node catches up 3 intervals (3 fires)
//!   in one step. `THREADS=4` trace == `THREADS=1` trace — pins
//!   `tick_node_into`'s Period multi-fire + per-catch-up re-check under
//!   parallel.
//! - **Test 5 (`panic_in_one_parallel_node_isolated`):** a wide level where ONE
//!   node panics once a shared counter crosses a threshold. The headline is
//!   ISOLATION: the panic on one rayon worker does NOT abort the pool — every
//!   survivor fires to the end of the run. It also pins the TRUE
//!   `GraphRuntime` panic-recovery semantics: the runtime's tick callback holds
//!   the node's entry `Mutex` across `tick()`, so a panic POISONS it → the node
//!   panics EXACTLY ONCE (`panic_count == 1`) then goes inert (every later fire
//!   no-ops on the poisoned lock; `fire_count` keeps climbing) — the
//!   bare-`Scheduler` `MAX_CONSECUTIVE_PANICS` disable path is NOT reached
//!   through the runtime. All of it is identical `THREADS=4` vs `=1`.
//! - **Test 6 (`mixed_macro_and_closure_gated_serial`):** a wide level mixing
//!   macro `Period` nodes (parallel) with a `ClosureNodeEntry` carrying a
//!   non-trigger input (→ `performs_input_snapshot()==false` + present in
//!   `snapshot_input_names` → routed to `serial_fire_node_ids`, fires serially in
//!   decision position). The gated node is LAST in the level. Trace is
//!   byte-identical `THREADS=4` vs `=1` — the serial-gated node never perturbs
//!   the merged order.
//! - **Test 7 (`block_topic_deterministic_under_parallel_fire`):** a POSITIVE
//!   determinism BACKSTOP — a `block` topic (fast producer + slow depth-2
//!   consumer) sharing a wide level 0 with 8 macro siblings stays deterministic
//!   under forced `THREADS=4` (defer count bit-identical across 15 runs AND ==
//!   the `THREADS=1` value). The block pair is block-involved → it runs through
//!   the fused seam, NOT `tick_decided_parallel`'s decisions; the 8 non-block
//!   macros are the non-serial REST → 8 ≥ the parallel threshold → fired via
//!   `par_values_mut`. The consumer is a `Period` node, so its `block`
//!   input is NON-trigger → no DAG edge → it is a LEVEL-0 ROOT in the SAME wide
//!   level as its producer (NOT level 1). NOT a mutation-decisive pin for set B
//!   (it passes with OR without block-serialization) — but the reason is the
//!   PACING, not the level: the producer's defers land on the SATURATED
//!   mid-window steps where `outstanding == depth` regardless of the ±1 jitter
//!   the boundary drain/publish race causes, so the TOTAL defer count is
//!   insensitive to the interleave. The DECISIVE set-B coverage lives in
//!   `backpressure_event_iox2_test::block_event_rearms_after_below_threshold_drain`
//!   + `snapshot_wiring_iox2_test::block_input_excluded_from_snapshot_plain_sibling_frozen`.
//! - **Test 8 (`all_serial_level_no_rayon_dispatch`):** two independent block
//!   pairs (`pa→ca`, `pb→cb`). Every consumer's `block` input is NON-trigger →
//!   no DAG edge → ALL FOUR nodes are level-0 roots in ONE wide level. All four
//!   are block-involved, so the level executor's `other_ids` (level MINUS the
//!   block-involved set) is EMPTY → `tick_decided_parallel` receives ZERO
//!   decisions → the `<= 1` FAST-PATH (NO rayon dispatch — hence the name). The
//!   four nodes fire through `evaluate_nodes_fused` (the block seam), NOT
//!   `tick_decided_parallel`. The run completes cleanly and the trace is
//!   byte-identical `THREADS=4` vs `=1` (the all-block fused fire + the
//!   empty-decisions fast-path are thread-count-invariant).
//! - **Test 9 (`interleaved_gated_node_merges_in_decision_order`):** the
//!   MERGE-ORDER-DECISIVE pin. A single WIDE level of 9 (1 serial-gated node + 8
//!   non-serial-gated REST fires; the REST is 8 ≥ the parallel threshold of 8 →
//!   the REST fires via `par_values_mut`) with a serial-gated
//!   `ClosureNodeEntry` (non-trigger input → set A) at decision position 2 — the
//!   MIDDLE of the level — compared ENTRY-FOR-ENTRY against a bare
//!   `Scheduler::step` FLAT oracle. The gated node fires FIRST (PASS 1) but its
//!   fragment must land at trace position 2: PASS 3 merges by DRAINING
//!   `decisions` in decision (pos) order, NOT in fire-production
//!   (serial-gated-first) order. A regression that merged fragments in production
//!   order (or any non-pos order) would put the gated fragment first → diverges
//!   from the oracle. This makes the pos-order drain load-bearing (Tests 1/2/6
//!   leave it untested — no mid-level interleave).
//!
//! Real `#[cerulion_node]` macro nodes back every snapshot/freeze test (closures
//! inherit the no-op `snapshot_inputs` so they never freeze); `ClosureNodeEntry`
//! appears ONLY in Test 6 as the serial-gated node. The only legitimate cross-run
//! equality is the determinism test (2) + the `THREADS=4`-vs-`=1` comparisons
//! (which ARE the point). No fake data (Principle #13): every recorded value is a
//! real iceoryx2 publish read inside a real tick.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{
    BackpressurePolicy, ClosureNodeEntry, InputMeta, MacroPolicy, NodeEntry, NodeInfo,
};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::message::ShmMessage;
use cerulion_core::prelude::*;
use cerulion_core::scheduler::{NodeConfig, Scheduler, TraceEntry, TriggerPolicy};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

// A sentinel a producer never publishes (producers publish 1, 2, 3, ...). A
// recorded MISSING means the consumer tick did NOT run on a measured step (a
// `None`/Empty frozen slot collapses the macro's `try_view` chain to a no-op),
// which the measured-step asserts REJECT — guards against a silent tautology.
const MISSING: u64 = u64::MAX;

// The forced thread counts. 4 is real parallel on a 16-core machine (and any
// CI runner with ≥ 2 cores; if `available_parallelism` were 1 the pool would
// still be 4 here because the env override bypasses the auto-size — see
// `CERULION_FIRE_THREADS` read at build, which does NOT clamp to
// available_parallelism). 1 forces serial-merge through the SAME code path.
const THREADS_PARALLEL: &str = "4";
const THREADS_SERIAL: &str = "1";

/// RAII guard that restores (removes) `CERULION_FIRE_THREADS` on drop, even if an
/// assertion panics mid-test. `build_for_test` reads the env at BUILD time, so a
/// test sets the var, builds the runtime (capturing the count), and may then let
/// the guard drop — but we keep it alive for the whole test body for safety.
/// Pairs with `#[serial]` since env is process-global (mirrors
/// `chunk_c_ffi_codes_3_4_test`'s `EnvVarGuard`).
struct FireThreadsGuard;

impl FireThreadsGuard {
    fn set(value: &str) -> Self {
        std::env::set_var("CERULION_FIRE_THREADS", value);
        Self
    }
}

impl Drop for FireThreadsGuard {
    fn drop(&mut self) {
        std::env::remove_var("CERULION_FIRE_THREADS");
    }
}

// ===========================================================================
// Shared producer: a Period(10) node publishing an incrementing counter
// (1, 2, 3, ...) into Vector3.x. `self.out.x = n` writes straight into the
// loaned SHM slot (Vector3 is a fixed schema; fields via Deref).
// ===========================================================================

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct WideProducer {
    #[output]
    out: Vector3,
    n: u64,
}

#[cerulion_node_impl]
impl WideProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Increment FIRST so the first publish carries 1, not 0 — keeps the
        // recorded values strictly positive and the MISSING sentinel distinct.
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// A standalone counter producer that records nothing — used for the
/// wide-level byte-identity / determinism / catch-up tests where only the TRACE
/// (fire ordering + times) matters, not any read value.
fn wide_producer_def(id: &str) -> NodeDef {
    NodeDef {
        fuse: None,
        ros2: None,
        id: id.to_string(),
        node_type: "wide_producer".to_string(),
        inputs: vec![],
        outputs: vec![OutputDef {
            name: "out".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }],
    }
}

fn wide_ids(n: usize) -> Vec<String> {
    (0..n).map(|i| format!("p{i:02}")).collect()
}

/// Build an all-`Period(10)` wide-level graph: `n` `WideProducer` sources, no
/// inputs → no DAG edges → ONE wide level 0. Returns the config + factories.
fn wide_graph(prefix: &str, ids: &[String]) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "wide_level".to_string(),
        prefix: prefix.to_string(),
        nodes: ids.iter().map(|id| wide_producer_def(id)).collect(),
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for id in ids {
        factories.insert(id.clone(), Box::new(WideProducerEntry::new()));
    }
    (config, factories)
}

/// Build + run a wide-level graph under a forced thread count, `steps` steps of
/// `step_ms` ms each, and return its `TraceEntry` stream. The guard is built
/// here (env set BEFORE `build_for_test` reads it) and held until after the run.
fn run_wide_trace(
    prefix: &str,
    ids: &[String],
    threads: &str,
    steps: u32,
    step_ms: u64,
) -> Vec<TraceEntry> {
    let _guard = FireThreadsGuard::set(threads);
    let (config, factories) = wide_graph(prefix, ids);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build wide graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(step_ms));
    }
    runtime.trace().to_vec()
}

// ===========================================================================
// Test 1: THE rayon byte-identity gate — parallel == serial-merge == flat.
//
// A WIDE level (16 Period(10) sources) yields a byte-equal TraceEntry stream
// under THREE drivers: real parallel (THREADS=4), forced serial-merge
// (THREADS=1) through the same tick_decided_parallel code path, and a bare
// Scheduler::step flat path with the SAME ids/order/Period policy. The trace
// records only (node_id, fire_time_ns), so the macro callback bodies are
// irrelevant — the FIRE ORDER (graph/insertion order within the level) and the
// FIRE TIMES must match across all three. 16 ≥ 2×4 → parallelism is genuine.
// ===========================================================================

#[test]
#[serial]
fn parallel_serial_flat_traces_byte_identical() {
    const N: usize = 16;
    const STEPS: u32 = 8;
    let ids = wide_ids(N);

    // (a) real parallel (4 threads).
    let parallel = run_wide_trace("wid_par", &ids, THREADS_PARALLEL, STEPS, 10);
    // (b) forced serial-merge (1 thread) through the SAME tick_decided_parallel.
    let serial = run_wide_trace("wid_ser", &ids, THREADS_SERIAL, STEPS, 10);

    // (c) bare Scheduler flat path: same ids/order/Period policy. Callback
    // bodies are irrelevant to the trace (records only node_id + fire_time_ns).
    let flat: Vec<TraceEntry> = {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        for id in &ids {
            scheduler
                .add_node(NodeConfig {
                    id: id.clone(),
                    policy: TriggerPolicy::Period {
                        interval: Duration::from_millis(10),
                        max_catchup: None,
                    },
                    callback: Box::new(|| {}),
                })
                .expect("add flat node");
        }
        for _ in 0..STEPS {
            scheduler.step_ms(10);
        }
        scheduler.trace().to_vec()
    };

    // Non-vacuous: every driver fired N nodes × STEPS steps.
    let expected = N * STEPS as usize;
    assert_eq!(
        parallel.len(),
        expected,
        "parallel trace recorded every fire"
    );
    assert_eq!(serial.len(), expected, "serial trace recorded every fire");
    assert_eq!(flat.len(), expected, "flat trace recorded every fire");

    // THE byte-identity pins.
    assert_eq!(
        parallel, serial,
        "the real-parallel (THREADS=4) merged trace must be BYTE-IDENTICAL to the \
         forced-serial-merge (THREADS=1) trace through the SAME \
         tick_decided_parallel path. parallel={parallel:?} serial={serial:?}"
    );
    assert_eq!(
        parallel, flat,
        "the level executor's parallel-merged trace must be BYTE-IDENTICAL to the \
         FLAT Scheduler::step path for the same ids/order/Period policy \
         (the byte-identity claim). parallel={parallel:?} flat={flat:?}"
    );
}

// ===========================================================================
// Test 2: determinism across many fresh runs under real parallel threads.
//
// Fire ORDER on the rayon workers varies run-to-run; the MERGED trace (re-sorted
// by decision position) must not. Capture a reference under THREADS=4, then 40
// FRESH runtimes (each build_for_test → per-test SHM root) must each reproduce it
// bit-for-bit. Catches any merge-order / shared-state nondeterminism the single
// THREADS=4-vs-THREADS=1 compare in Test 1 could miss (e.g. a rare interleave).
// ===========================================================================

#[test]
#[serial]
fn parallel_fire_deterministic_across_many_runs() {
    const N: usize = 16;
    const STEPS: u32 = 6;
    const RUNS: usize = 40;
    let ids = wide_ids(N);

    let reference = run_wide_trace("wid_det00", &ids, THREADS_PARALLEL, STEPS, 10);
    assert_eq!(
        reference.len(),
        N * STEPS as usize,
        "reference run actually fired (non-vacuous)"
    );

    for run in 1..=RUNS {
        let prefix = format!("wid_det{run:02}");
        let t = run_wide_trace(&prefix, &ids, THREADS_PARALLEL, STEPS, 10);
        assert_eq!(
            t, reference,
            "run {run} under THREADS=4 must reproduce the reference trace \
             bit-for-bit (fire order varies on the workers; the merged trace \
             must not — Principle #7). run={t:?} reference={reference:?}"
        );
    }
}

// ===========================================================================
// Test 3 (HEADLINE race): the level-boundary snapshot holds under real threads.
//
// N same-level Period(10) producers + a Period(10) macro consumer with N plain
// (snapshotted) #[input]s, one per producer. The consumer fires in PARALLEL with
// the producers (THREADS=4). The level-boundary snapshot FREEZES the consumer's
// inputs AFTER deciding the level's fires + draining triggers, BEFORE any
// level-0 tick runs — so the consumer reads each producer's PRIOR-step value
// (v-1), NEVER the same-step value (v) the producer publishes on another thread
// THIS step.
//
// WITHOUT 4a's snapshot the consumer's tick-time read would RACE the producers'
// concurrent publishes (a genuine data race → flaky reads of v or v-1). WITH the
// snapshot the read is a frozen owned copy → rock-solid v-1, every step, every
// run. Asserting `read == v-1` AND `read != v` makes the freeze the only thing
// that can produce a passing run.
// ===========================================================================

// 4 plain inputs from 4 same-level producers (the task allows "a few — 3-4");
// 4 keeps the macro field list readable while still exercising 4 concurrent
// producer publishes against the consumer's frozen reads on the worker pool.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct FreezeConsumer {
    #[input]
    a: Vector3,
    #[input]
    b: Vector3,
    #[input]
    c: Vector3,
    #[input]
    d: Vector3,
    /// Shared with the harness: each input's read value, set EVERY tick. The
    /// MIN across the four is recorded so a single laggard input (cold-start)
    /// is detectable; at steady state all four carry the SAME counter (all
    /// producers are Period(10) publishing in lockstep), so min == each.
    read_a: Arc<AtomicU64>,
    read_b: Arc<AtomicU64>,
    read_c: Arc<AtomicU64>,
    read_d: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl FreezeConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.read_a.store(self.a.x as u64, Ordering::Relaxed);
        self.read_b.store(self.b.x as u64, Ordering::Relaxed);
        self.read_c.store(self.c.x as u64, Ordering::Relaxed);
        self.read_d.store(self.d.x as u64, Ordering::Relaxed);
        Ok(())
    }
}

/// One frozen-read run: 4 read producers + 4 filler producers + 1 consumer = a
/// 9-wide level, all Period(10), THREADS=4 (9 ≥ the parallel threshold of 8 so
/// the non-serial REST genuinely fires via `par_values_mut` — the filler only
/// widens the level to engage rayon; it is NOT read by the consumer).
/// Returns per measured step `(producer_value_v, [read_a, read_b, read_c,
/// read_d])`. On step k (1-indexed over the whole run) every producer publishes
/// k, so on the i-th measured step the producer value is `warmup + i + 1`.
#[allow(clippy::type_complexity)]
fn run_freeze(prefix: &str, warmup: u32, measured: u32) -> Vec<(u64, [u64; 4])> {
    let _guard = FireThreadsGuard::set(THREADS_PARALLEL);
    let reads: [Arc<AtomicU64>; 4] = std::array::from_fn(|_| Arc::new(AtomicU64::new(MISSING)));

    let producer_ids = ["pa", "pb", "pc", "pd"];
    let mut nodes: Vec<NodeDef> = producer_ids
        .iter()
        .map(|id| wide_producer_def(id))
        .collect();
    // FILLER: unconsumed Period(10) producers (distinct ids → distinct output
    // topics, no single-writer collision) that share level 0 ONLY to widen the
    // non-serial REST to ≥ the parallel threshold so the rayon `par_values_mut`
    // path actually engages. Without them the level is 4 read producers + 1
    // consumer = 5 non-serial fires < 8 → NARROW (serial), and "under REAL
    // parallel threads" would be vacuous. The consumer reads NONE of them, so the
    // freeze oracle below (read == v-1 for pa..pd) is unchanged.
    const FILLER: usize = 4;
    for i in 0..FILLER {
        nodes.push(wide_producer_def(&format!("ff{i:02}")));
    }
    // Consumer declared LAST → ticks after the producers within level 0 (graph
    // order). With a LIVE read (no snapshot) it would see the same-step value;
    // the snapshot freezes it to the prior step.
    nodes.push(NodeDef {
        fuse: None,
        ros2: None,
        id: "consumer".to_string(),
        node_type: "freeze_consumer".to_string(),
        inputs: vec![
            InputDef {
                name: "a".to_string(),
                source: "pa/out".to_string(),
            },
            InputDef {
                name: "b".to_string(),
                source: "pb/out".to_string(),
            },
            InputDef {
                name: "c".to_string(),
                source: "pc/out".to_string(),
            },
            InputDef {
                name: "d".to_string(),
                source: "pd/out".to_string(),
            },
        ],
        outputs: vec![],
    });
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "freeze_race".to_string(),
        prefix: prefix.to_string(),
        nodes,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for id in producer_ids {
        factories.insert(id.to_string(), Box::new(WideProducerEntry::new()));
    }
    for i in 0..FILLER {
        factories.insert(format!("ff{i:02}"), Box::new(WideProducerEntry::new()));
    }
    let consumer = FreezeConsumer {
        read_a: Arc::clone(&reads[0]),
        read_b: Arc::clone(&reads[1]),
        read_c: Arc::clone(&reads[2]),
        read_d: Arc::clone(&reads[3]),
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(FreezeConsumerEntry::with_state(consumer)),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build freeze graph");

    // WARMUP: establish a non-Empty frozen slot on every input so the measured
    // window is free of the cold-start Empty read (macro tick no-ops).
    for _ in 0..warmup {
        runtime.step(Duration::from_millis(10));
    }
    let mut out = Vec::with_capacity(measured as usize);
    for i in 0..measured {
        let v = (warmup + i + 1) as u64;
        for r in &reads {
            r.store(MISSING, Ordering::Relaxed);
        }
        runtime.step(Duration::from_millis(10));
        out.push((
            v,
            [
                reads[0].load(Ordering::Relaxed),
                reads[1].load(Ordering::Relaxed),
                reads[2].load(Ordering::Relaxed),
                reads[3].load(Ordering::Relaxed),
            ],
        ));
    }
    out
}

#[test]
#[serial]
fn snapshot_freeze_holds_under_real_parallel_threads() {
    // Many runs × many steps: the race (if the snapshot did not hold) would flip
    // a read to `v` on SOME interleave. With the freeze, every read is `v-1`.
    const RUNS: usize = 12;
    for run in 0..RUNS {
        let prefix = format!("frz{run:02}");
        let reads = run_freeze(&prefix, 4, 6);
        assert_eq!(
            reads.len(),
            6,
            "the measured window actually ran (run {run})"
        );
        for (v, rs) in &reads {
            for (k, read) in rs.iter().enumerate() {
                assert_ne!(
                    *read, MISSING,
                    "run {run}: consumer tick did not run on a measured step \
                     (input {k} frozen slot Empty — warmup should make this \
                     unreachable). producer value was {v}"
                );
                // THE freeze oracle: prior-step value, under real concurrency.
                assert_eq!(
                    *read,
                    v - 1,
                    "run {run}: the level-boundary snapshot must freeze input {k} \
                     to the PRIOR step's value {} while its producer publishes {v} \
                     on another thread THIS step, got {read}",
                    v - 1
                );
                // Decisive race pin: a same-step read (snapshot lost the race /
                // did not freeze) would record `v`.
                assert_ne!(
                    *read, *v,
                    "run {run}: input {k} read the producer's SAME-STEP value {v} \
                     — the snapshot FAILED to freeze before the parallel producer \
                     publish (the data race this test guards against)"
                );
            }
        }
    }
}

// ===========================================================================
// Test 4: Period catch-up byte-identity under parallel.
//
// A wide level of Period(10) nodes stepped with step(30ms) → each catches up 3
// intervals in ONE step (3 fires/node). Compare THREADS=4 vs THREADS=1: the
// merged trace must be byte-identical. Pins tick_node_into's Period multi-fire
// loop + the per-catch-up run_pre_fire_check re-check under parallel (each node's
// 3 catch-up TraceEntries land contiguously, in the right order, after the merge).
// ===========================================================================

#[test]
#[serial]
fn period_catchup_byte_identical_under_parallel() {
    const N: usize = 16;
    const STEPS: u32 = 5;
    let ids = wide_ids(N);

    // step(30ms) with Period(10) → 3 catch-up fires per node per step.
    let parallel = run_wide_trace("wid_cu_par", &ids, THREADS_PARALLEL, STEPS, 30);
    let serial = run_wide_trace("wid_cu_ser", &ids, THREADS_SERIAL, STEPS, 30);

    // Non-vacuous: 3 fires/node/step → 3 × N × STEPS entries.
    let expected = 3 * N * STEPS as usize;
    assert_eq!(
        parallel.len(),
        expected,
        "catch-up parallel trace: 3 fires/node × {N} nodes × {STEPS} steps"
    );
    assert_eq!(serial.len(), expected, "catch-up serial trace: same count");
    assert_eq!(
        parallel, serial,
        "Period catch-up (3 fires/node/step) merged trace must be BYTE-IDENTICAL \
         THREADS=4 vs THREADS=1 — the multi-fire loop + per-catch-up re-check are \
         parallel-safe. parallel={parallel:?} serial={serial:?}"
    );
}

// ===========================================================================
// Test 5: one panicking parallel node is ISOLATED (pool not aborted).
//
// A wide level where ONE node panics on EVERY tick once a shared counter crosses
// a threshold. The headline guarantee under THREADS=4 is ISOLATION: a panic on
// one rayon worker does NOT abort the pool — every OTHER node fires through the
// panic and to the end of the run, with byte-for-byte the same behavior as
// THREADS=1 (per-fire `catch_unwind`, not a pool-level abort).
//
// TRUE panic-recovery semantics through `GraphRuntime` (this test pins them, and
// they are NOT the bare-`Scheduler` `MAX_CONSECUTIVE_PANICS` story):
//   The runtime's per-node tick callback locks the node's
//   `Arc<Mutex<Box<dyn NodeEntry>>>` and calls `NodeEntry::tick()` while holding
//   the guard (runtime.rs ~1733). A panic in `tick()` unwinds THROUGH the held
//   `MutexGuard` → the mutex is POISONED. The scheduler's `catch_unwind` still
//   sees the panic and bumps `panic_count` to 1, but on EVERY subsequent step the
//   callback's `entry.lock()` returns `Err(poisoned)` → it logs + skips, so
//   `tick()` (and the `panic!`) is NEVER reached again. The node therefore panics
//   EXACTLY ONCE then goes inert: `panic_count == 1` forever, `fire_count` keeps
//   climbing (the scheduler still "fires" it — the callback just no-ops on the
//   poisoned lock), and it never reaches the 3-consecutive-panic disable path
//   (that path is reachable only via a bare `Scheduler` whose callback does not
//   poison a per-entry mutex). This poison-once-then-inert behavior is itself a
//   real contract the parallel executor must preserve identically across thread
//   counts.
//
// The panic node is a CLOSURE so the panic is reliably reached on the FIRST
// post-threshold tick (a `#[cerulion_node]` macro node wraps the body in
// loan/try_view machinery that can skip the body, muddying "did it panic"). It is
// a no-port `Period` closure → NOT in `snapshot_input_names`, NOT block-involved
// → fires on the PARALLEL pool (the point of the test). The SURVIVORS are real
// macro `Period` nodes.
// ===========================================================================

/// A plain Period(10) survivor that records its own monotonically increasing
/// tick count — proof the pool kept running other nodes through the panic.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct SurvivorNode {
    // The macro requires ≥1 port; a (never-read) output satisfies it.
    #[output]
    out: Vector3,
    n: u64,
    last_n: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl SurvivorNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        self.last_n.store(self.n, Ordering::Relaxed);
        Ok(())
    }
}

/// Build + run the panic-isolation graph under `threads`: 1 feeder + 14
/// survivors + 1 panicking CLOSURE node (panics from tick `panic_from`), all
/// Period(10), level 0 (no DAG edges — every node is a Period source/sink with no
/// trigger inputs, so all 16 share the wide level). Run `steps` of 10ms. Returns
/// `(panic_count, panic_fire_count, survivor_last_ns, panicker_last_clean)`.
fn run_panic_iso(
    prefix: &str,
    threads: &str,
    steps: u32,
    panic_from: u64,
) -> (u64, u64, Vec<u64>, u64) {
    let _guard = FireThreadsGuard::set(threads);
    const SURVIVORS: usize = 15;
    let panic_last_clean = Arc::new(AtomicU64::new(0));
    let panic_tick_seen = Arc::new(AtomicU64::new(0));
    let survivor_lasts: Vec<Arc<AtomicU64>> = (0..SURVIVORS)
        .map(|_| Arc::new(AtomicU64::new(0)))
        .collect();

    let mut nodes: Vec<NodeDef> = Vec::new();
    // The panic node is placed FIRST in declaration order but is NOT first in the
    // fire order — within level 0 the survivors and panic node interleave on the
    // pool; the merge re-sorts by decision position regardless. It declares one
    // output (never loaned by the closure body) to carry a port; it fires in
    // PARALLEL (no-port-from-the-snapshot-set Period closure → not serial-gated).
    nodes.push(NodeDef {
        fuse: None,
        ros2: None,
        id: "panic".to_string(),
        node_type: "panic_node".to_string(),
        inputs: vec![],
        outputs: vec![OutputDef {
            name: "out".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }],
    });
    for i in 0..SURVIVORS {
        nodes.push(NodeDef {
            fuse: None,
            ros2: None,
            id: format!("s{i:02}"),
            node_type: "survivor_node".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        });
    }
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "panic_iso".to_string(),
        prefix: prefix.to_string(),
        nodes,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    // The panicking node: a Period(10) CLOSURE that panics on every tick once a
    // SHARED atomic counter crosses `panic_from`. The closure body runs
    // unconditionally each fire (no macro loan/try_view wrapper), so the panic is
    // reliably repeated → consecutive panics → disable. It never loans `out`.
    let info = NodeInfo::from_names(vec![], vec!["out".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 10 });
    let tick_seen_cb = Arc::clone(&panic_tick_seen);
    let clean_cb = Arc::clone(&panic_last_clean);
    let panic_closure = ClosureNodeEntry::new(info, move |_ctx| {
        let this_tick = tick_seen_cb.fetch_add(1, Ordering::Relaxed) + 1;
        if panic_from != 0 && this_tick >= panic_from {
            panic!("panic_node deliberate panic at tick {this_tick}");
        }
        clean_cb.store(this_tick, Ordering::Relaxed);
        Ok(())
    })
    .with_label("panic_node");
    factories.insert("panic".to_string(), Box::new(panic_closure));
    for (i, last) in survivor_lasts.iter().enumerate() {
        let s = SurvivorNode {
            last_n: Arc::clone(last),
            ..Default::default()
        };
        factories.insert(
            format!("s{i:02}"),
            Box::new(SurvivorNodeEntry::with_state(s)),
        );
    }
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build panic graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(10));
    }

    let h = runtime.node_handle("panic").unwrap();
    let panic_count = h.panic_count();
    let panic_fires = h.fire_count();
    let survivor_last: Vec<u64> = survivor_lasts
        .iter()
        .map(|a| a.load(Ordering::Relaxed))
        .collect();
    let panicker_clean = panic_last_clean.load(Ordering::Relaxed);
    (panic_count, panic_fires, survivor_last, panicker_clean)
}

#[test]
#[serial]
fn panic_in_one_parallel_node_isolated() {
    // Panic FROM tick 3. Run 12 steps: ticks 1-2 run clean (last_clean=2); tick 3
    // panics → poisons the node's entry mutex → panic_count=1; ticks 4..=12 fire
    // but the poisoned-lock callback no-ops (no further panic, no further clean
    // tick). The node never reaches the bare-Scheduler 3-consecutive-panic disable
    // path (the poison short-circuits before a 2nd panic). See the section doc.
    const PANIC_FROM: u64 = 3;
    const STEPS: u32 = 12;

    let (pc4, pf4, surv4, clean4) =
        run_panic_iso("paniso_par", THREADS_PARALLEL, STEPS, PANIC_FROM);
    let (pc1, pf1, surv1, clean1) = run_panic_iso("paniso_ser", THREADS_SERIAL, STEPS, PANIC_FROM);

    // (a) ISOLATION — the pool was NOT aborted: EVERY survivor advanced to STEPS
    // ticks (all 12 steps fired them) under BOTH thread counts. This is the
    // headline guarantee (a panic on one worker must not poison the join / kill
    // siblings). Non-vacuous: 15 survivors.
    assert_eq!(surv4.len(), 15, "non-vacuous: 15 survivors recorded");
    for (i, n) in surv4.iter().enumerate() {
        assert_eq!(
            *n, STEPS as u64,
            "THREADS=4: survivor s{i:02} must have fired every step ({STEPS}) — \
             the panicking sibling did not abort the rayon pool, got {n}"
        );
    }
    // (b) the panicker panicked EXACTLY ONCE (then the poisoned entry mutex makes
    // it inert), yet the scheduler keeps "firing" it every step.
    assert_eq!(
        pc4, 1,
        "THREADS=4: panicker panics exactly ONCE (tick {PANIC_FROM}) then its entry \
         mutex is poisoned, so subsequent ticks no-op on the poisoned lock and \
         never re-enter the panic — panic_count must be 1, got {pc4}"
    );
    assert_eq!(
        pf4, STEPS as u64,
        "THREADS=4: the scheduler keeps firing the (now-inert) panicker every step \
         — fire_count must be {STEPS} (the poisoned-lock callback no-ops but still \
         counts as a fire), got {pf4}"
    );
    assert_eq!(
        clean4,
        PANIC_FROM - 1,
        "THREADS=4: panicker's last CLEAN tick is {} (tick {PANIC_FROM} panicked \
         and every tick after no-ops on the poisoned lock — last_clean never \
         advances), got {clean4}",
        PANIC_FROM - 1
    );

    // (c) ALL of it is IDENTICAL at THREADS=1 — panic isolation + poison-recovery
    // are thread-count-invariant (per-fire catch_unwind + per-entry mutex, not a
    // pool-level abort).
    assert_eq!(
        surv4, surv1,
        "survivor tick counts must match THREADS=4 vs THREADS=1 (panic isolation \
         is thread-count-invariant). par={surv4:?} ser={surv1:?}"
    );
    assert_eq!(
        (pc4, pf4, clean4),
        (pc1, pf1, clean1),
        "panicker (panic_count, fire_count, last_clean) must match THREADS=4 vs \
         THREADS=1. par=({pc4},{pf4},{clean4}) ser=({pc1},{pf1},{clean1})"
    );
}

// ===========================================================================
// Test 6: macro (parallel) + closure (serial-gated) mixed level, byte-identical.
//
// A wide level mixing macro Period(10) nodes (parallel-fired) with ONE
// ClosureNodeEntry carrying a NON-TRIGGER plain input. The closure declares a
// MacroPolicy::Period → it lands in `snapshot_input_names` (Period ⇒ all inputs
// latest-value), and because ClosureNodeEntry::performs_input_snapshot() is the default
// `false`, the build routes it into `serial_fire_node_ids` (set A: a node with
// latest-value inputs whose snapshot_inputs is a no-op). So the closure fires
// SERIALLY in decision position on the calling thread; the macro siblings fire on
// the pool. The merged trace must be byte-identical THREADS=4 vs THREADS=1 —
// confirming the serial-gated node lands in its true decision slot and never
// perturbs the merge order.
//
// (The closure's READ is LIVE — its no-op snapshot does not freeze — but this
// test asserts the TRACE, not a read value, so that is irrelevant here.)
// ===========================================================================

/// A plain (DropOldest) non-trigger input meta for the gated closure node.
fn plain_input_meta(name: &str) -> InputMeta {
    InputMeta {
        name: name.to_string(),
        schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
        trigger: false,
        depth: 8,
        backpressure: BackpressurePolicy::DropOldest,
        expect_within_ms: None,
    }
}

/// Build + run the mixed graph under `threads`: 1 producer (feeds the closure's
/// input) + 14 plain macro Period producers + 1 gated closure consumer, all
/// Period(10), level 0. Returns the trace. The closure reads its `inp` each tick
/// (a real iceoryx2 read — no fake data) but records nothing the trace needs.
fn run_mixed_trace(prefix: &str, threads: &str, steps: u32) -> Vec<TraceEntry> {
    let _guard = FireThreadsGuard::set(threads);
    const EXTRA_MACROS: usize = 14;
    let closure_reads = Arc::new(AtomicU64::new(0));

    let mut nodes: Vec<NodeDef> = Vec::new();
    // The producer feeding the closure's input.
    nodes.push(wide_producer_def("feeder"));
    // Extra macro siblings (parallel-fired) sharing level 0.
    for i in 0..EXTRA_MACROS {
        nodes.push(wide_producer_def(&format!("m{i:02}")));
    }
    // The gated closure consumer, declared LAST → ticks after the macros in
    // decision order; routed to the serial path by serial_fire_node_ids.
    nodes.push(NodeDef {
        fuse: None,
        ros2: None,
        id: "gated".to_string(),
        node_type: "gated_closure".to_string(),
        inputs: vec![InputDef {
            name: "inp".to_string(),
            source: "feeder/out".to_string(),
        }],
        outputs: vec![],
    });
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "mixed_gated".to_string(),
        prefix: prefix.to_string(),
        nodes,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("feeder".to_string(), Box::new(WideProducerEntry::new()));
    for i in 0..EXTRA_MACROS {
        factories.insert(format!("m{i:02}"), Box::new(WideProducerEntry::new()));
    }
    // The gated closure: Period(10) + a plain non-trigger input → serial-gated.
    let info = NodeInfo::with_meta(vec![plain_input_meta("inp")], vec![])
        .with_policy(MacroPolicy::Period { period_ms: 10 });
    let reads_cb = Arc::clone(&closure_reads);
    let closure = ClosureNodeEntry::new(info, move |ctx| {
        // A real iceoryx2 read of the frozen/live slot every tick (no fake
        // data). The value is recorded but the trace assertion does not depend
        // on it — this just proves the gated node's tick actually runs.
        if let Some(sub) = ctx.subscriber_mut("inp") {
            if let Ok(Some(x)) = sub.try_view::<Vector3, _>(|view| view.x) {
                reads_cb.store(x as u64, Ordering::Relaxed);
            }
        }
        Ok(())
    })
    .with_label("gated_closure");
    factories.insert("gated".to_string(), Box::new(closure));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build mixed graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(10));
    }
    runtime.trace().to_vec()
}

#[test]
#[serial]
fn mixed_macro_and_closure_gated_serial() {
    const STEPS: u32 = 8;
    // 1 feeder + 14 macros + 1 gated closure = 16 nodes, all level 0.
    const N: usize = 16;

    let parallel = run_mixed_trace("mix_par", THREADS_PARALLEL, STEPS);
    let serial = run_mixed_trace("mix_ser", THREADS_SERIAL, STEPS);

    // Non-vacuous: all 16 nodes (incl. the gated closure) fire every step.
    let expected = N * STEPS as usize;
    assert_eq!(
        parallel.len(),
        expected,
        "mixed parallel trace: {N} nodes (15 macro + 1 gated closure) × {STEPS} steps"
    );
    assert_eq!(serial.len(), expected, "mixed serial trace: same count");
    assert_eq!(
        parallel, serial,
        "the serial-gated closure node (non-trigger input + no-op snapshot → \
         serial_fire_node_ids) must fire in its TRUE decision position so the \
         merged trace is BYTE-IDENTICAL THREADS=4 vs THREADS=1 — the gated node \
         never perturbs the merge order. parallel={parallel:?} serial={serial:?}"
    );
}

// ===========================================================================
// Test 7 (POSITIVE determinism BACKSTOP for `block` under parallel fire — NOT
// a decisiveness pin for set B): a `block` topic stays deterministic under a
// forced high thread count.
//
// Scope note: this test is a positive backstop,
// not a mutation-decisive pin for the block-serialization fix (set B of
// `serial_fire_node_ids`). It passes with or without set
// B. Note the consumer is a `Period` node, so its `block` input is NON-trigger
// → no DAG edge (topology.rs `derive_levels` skips non-trigger edges) → it is a
// LEVEL-0 ROOT in the SAME wide level as its producer (NOT level 1). So WITHOUT
// set B the pair DOES fire in the same level-0 parallel batch and the consumer's
// tick-time drain genuinely races the producer's publish. The test stays
// non-decisive because of the PACING, not the level: the producer's defers land
// on the SATURATED mid-window steps (between the 20ms-spaced consumer drains,
// `outstanding` sits AT depth for several 5ms producer steps), where the
// deferred decision reads `outstanding == depth` regardless of the ±1 jitter the
// boundary drain/publish race causes — so the TOTAL defer count is insensitive
// to the interleave. So it proves block stays deterministic under parallel fire
// (a real, worth-keeping property) but it does NOT, on its own, prove set B is
// load-bearing.
//
// The DECISIVE set-B coverage lives in two OTHER tests that fire ~2-threaded by
// default (their level width ≥ 2 → `available_parallelism`-sized pool) and FAIL
// 3/3 with set B disabled:
//   - `backpressure_event_iox2_test::block_event_rearms_after_below_threshold_drain`
//   - `snapshot_wiring_iox2_test::block_input_excluded_from_snapshot_plain_sibling_frozen`
// Those are where the producer + consumer share a level and the un-serialized
// drain/publish race actually perturbs the `outstanding` mirror.
//
// Set-B rationale (the serialization this BACKSTOPS, in `runtime.rs`): `block` is the ONE
// backpressure policy excluded from the step-boundary snapshot — its drain stays
// on the consumer's TICK to keep the producer-pacing `outstanding` mirror
// (published − drained) in lock-step. If a block consumer's tick-time drain
// RACED its same-level producer's PARALLEL publish, the shared `outstanding`
// mirror would get a nondeterministic per-step trajectory → nondeterministic
// false defers on the producer's next-step pre_fire_check (replay ≠ live).
// Routing every producer + consumer of a block topic into
// `serial_fire_node_ids` (set B) restores the deterministic publish-then-drain
// order.
//
// This graph FORCES the wide parallel REST WHILE the block pair is present: a
// fast block producer (5ms) → depth-2 block consumer (20ms) PLUS 8 unrelated
// `WideProducer` macro siblings sharing the producer's level 0 — 10 nodes total
// at level 0 (bprod + 8 macros + bcons, the consumer a level-0 root since its
// block input is non-trigger). The block producer + consumer are block-involved
// (set B) → they run through the FUSED `evaluate_nodes_fused` seam (decide+tick,
// graph order) and are NOT in `tick_decided_parallel`'s
// decisions. The 8 non-block macros ARE the decisions, all non-serial → the REST
// is 8 ≥ the parallel threshold (8) → it fires via `par_values_mut` (PASS 2 wide)
// on every step. Either way the merge is decision-ordered and deterministic.
// The consumer's `block_fires_deferred_count` must be (a) non-zero (the producer
// really got deferred), (b) BIT-IDENTICAL across 15 fresh THREADS=4 runs (block
// is deterministic under parallel fire), and (c) EQUAL to the THREADS=1 value
// (parallel == serial). These pin DETERMINISM; for the set-B decisiveness pin
// see the two tests named above.
// ===========================================================================

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct BlockFastProducer {
    #[output]
    out: Vector3,
    n: u64,
}

#[cerulion_node_impl]
impl BlockFastProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// depth-2 `block` consumer draining every 20 ms — far slower than the 5 ms
/// producer, so the producer is repeatedly deferred at the block threshold.
#[cerulion_node(period_ms = 20)]
#[derive(Default)]
struct BlockSlowConsumer {
    #[input(backpressure = block, depth = 2)]
    inp: Vector3,
    last_seen: u64,
}

#[cerulion_node_impl]
impl BlockSlowConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last_seen = self.inp.x as u64;
        Ok(())
    }
}

/// Build + run the block-under-parallel graph under `threads`: a fast block
/// producer + a slow depth-2 block consumer + `extra` unrelated WideProducer
/// macros sharing the producer's level 0. The block pair (bprod/bcons) is
/// block-involved → it runs through the FUSED `evaluate_nodes_fused` seam and is
/// NOT in `tick_decided_parallel`'s decisions; the `extra` macros ARE the
/// decisions (the non-serial REST). With `extra` ≥ PARALLEL_FIRE_THRESHOLD the
/// REST fires via `par_values_mut` (PASS 2 wide) on every step. Returns the
/// consumer's `block_fires_deferred_count("inp")` (the producer-side defer
/// counter keyed on the block edge).
fn run_block_under_parallel(prefix: &str, threads: &str, steps: u32, extra: usize) -> u64 {
    let _guard = FireThreadsGuard::set(threads);

    let mut nodes: Vec<NodeDef> = Vec::new();
    // The fast block producer (level 0).
    nodes.push(NodeDef {
        fuse: None,
        ros2: None,
        id: "bprod".to_string(),
        node_type: "block_fast_producer".to_string(),
        inputs: vec![],
        outputs: vec![OutputDef {
            name: "out".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }],
    });
    // Unrelated REST siblings sharing level 0 — these ARE the
    // `tick_decided_parallel` decisions (the block pair runs through the fused
    // seam, not the decided set). With `extra` ≥ PARALLEL_FIRE_THRESHOLD the
    // non-serial REST fires in PASS 2 via `par_values_mut` on every step.
    for i in 0..extra {
        nodes.push(wide_producer_def(&format!("x{i:02}")));
    }
    // The slow depth-2 block consumer. It is a Period node, so its `block` input
    // is NON-trigger → no DAG edge → it is a LEVEL-0 ROOT in the SAME wide level
    // as bprod + the macros (not level 1).
    nodes.push(NodeDef {
        fuse: None,
        ros2: None,
        id: "bcons".to_string(),
        node_type: "block_slow_consumer".to_string(),
        inputs: vec![InputDef {
            name: "inp".to_string(),
            source: "bprod/out".to_string(),
        }],
        outputs: vec![],
    });

    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "block_parallel".to_string(),
        prefix: prefix.to_string(),
        nodes,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("bprod".to_string(), Box::new(BlockFastProducerEntry::new()));
    for i in 0..extra {
        factories.insert(format!("x{i:02}"), Box::new(WideProducerEntry::new()));
    }
    factories.insert("bcons".to_string(), Box::new(BlockSlowConsumerEntry::new()));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build block-parallel graph");
    // Step 5ms each so the 5ms producer fires every step and the 20ms consumer
    // drains every 4th — a robust, deterministic defer regime.
    for _ in 0..steps {
        runtime.step(Duration::from_millis(5));
    }
    runtime
        .node_handle("bcons")
        .expect("consumer handle")
        .backpressure_block_fires_deferred_count("inp")
}

#[test]
#[serial]
fn block_topic_deterministic_under_parallel_fire() {
    const STEPS: u32 = 60;
    // level 0 = bprod + 8 macros + bcons; the 8 NON-block macros are the
    // `tick_decided_parallel` decisions → 8 >= PARALLEL_FIRE_THRESHOLD → the REST
    // fires via `par_values_mut` (bprod/bcons are block-involved → run through
    // `evaluate_nodes_fused`, NOT counted in the decisions).
    const EXTRA: usize = 8;
    const RUNS: usize = 15;

    // Reference under THREADS=4 (real parallel fire on the wide level 0).
    let reference = run_block_under_parallel("bpar_ref", THREADS_PARALLEL, STEPS, EXTRA);
    // (a) the producer was ACTUALLY deferred — otherwise the determinism claim
    // is vacuous (a count stuck at 0 is trivially "deterministic").
    assert!(
        reference > 0,
        "the fast (5ms) producer into a depth-2 block consumer (20ms drain) must \
         have been deferred at least once under parallel fire (block_fires_deferred \
         > 0), else the determinism pin is vacuous — got {reference}"
    );

    // (b) BIT-IDENTICAL across many fresh THREADS=4 runs — the POSITIVE
    // determinism property: block stays deterministic under PARALLEL fire. (This
    // assertion is NOT mutation-decisive for set B — see the banner's SCOPE
    // NOTE: the pair IS in the same wide level 0, but the saturated-mid-window
    // pacing makes the total defer count insensitive to the drain/publish
    // interleave, so it stays stable even without the pair being serialized. The
    // decisive set-B pins live in backpressure_event_iox2_test +
    // snapshot_wiring_iox2_test.)
    for run in 0..RUNS {
        let prefix = format!("bpar{run:02}");
        let n = run_block_under_parallel(&prefix, THREADS_PARALLEL, STEPS, EXTRA);
        assert_eq!(
            n, reference,
            "run {run}: block_fires_deferred_count under THREADS=4 must be \
             bit-identical to the reference ({reference}) — the block pair is \
             serialized off the rayon path, so the publish-then-drain order (and \
             thus the defer count) is deterministic under parallel fire. got {n}"
        );
    }

    // (c) EQUAL to the forced-serial (THREADS=1) value — parallel == serial.
    let serial = run_block_under_parallel("bpar_ser", THREADS_SERIAL, STEPS, EXTRA);
    assert_eq!(
        serial, reference,
        "the THREADS=1 block_fires_deferred_count ({serial}) must equal the \
         THREADS=4 value ({reference}) — serializing the block pair makes \
         parallel fire produce EXACTLY the serial defer trajectory (replay = live)"
    );
}

// ===========================================================================
// Test 8: an ALL-BLOCK level → `tick_decided_parallel` receives EMPTY decisions
// (the ≤1 fast-path, NO rayon dispatch); the level fires entirely via the fused
// block seam, and the trace stays thread-count-invariant.
//
// TWO independent block pairs: producer A → depth-2 block consumer A, producer
// B → depth-2 block consumer B. The two producers have no inputs (level-0
// roots); each consumer is a Period node whose `block` input is NON-trigger →
// no DAG edge (topology.rs `derive_levels` skips non-trigger edges) → in-degree
// 0 → also a level-0 root. So ALL FOUR nodes sit in ONE wide level 0 (NOT two
// levels of 2). EVERY one of the four is BLOCK-INVOLVED (each producer feeds a
// block topic; each consumer reads one). Block-involved nodes are routed to the
// fused seam: the level executor builds `other_ids` = level nodes MINUS the
// block-involved set, so here `other_ids` is EMPTY and `decide_fires(other_ids)`
// produces ZERO decisions. `tick_decided_parallel` is still called, but with an
// empty `decisions` Vec → it takes the `decisions.len() <= 1` FAST-PATH (NO
// scratch, NO PASS 1/2/3, NO rayon dispatch — hence the test name). All four
// nodes fire through `evaluate_nodes_fused` (the decide+tick-per-node
// block seam), NOT through `tick_decided_parallel`. The run must complete
// cleanly and the trace must be deterministic (THREADS=4 == THREADS=1) — the
// all-block level's fused fire + the empty-decisions fast-path are both
// thread-count-invariant. (The complementary path — a level whose decisions are
// all SET-A serial-gated non-block nodes, so PASS 1 fires all and the PASS 2
// REST loop iterates over nothing — is a trivial skip-everything loop; it is
// exercised in spirit by the serial-gated node in `mixed_macro_and_closure_
// gated_serial` / the `step_zero_alloc_test` serial-gated gate.)
// ===========================================================================

/// Build + run the two-block-pair graph under `threads`; return the trace.
fn run_two_block_pairs(prefix: &str, threads: &str, steps: u32) -> Vec<TraceEntry> {
    let _guard = FireThreadsGuard::set(threads);

    let mk_producer = |id: &str| NodeDef {
        fuse: None,
        ros2: None,
        id: id.to_string(),
        node_type: "block_fast_producer".to_string(),
        inputs: vec![],
        outputs: vec![OutputDef {
            name: "out".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }],
    };
    let mk_consumer = |id: &str, src: &str| NodeDef {
        fuse: None,
        ros2: None,
        id: id.to_string(),
        node_type: "block_slow_consumer".to_string(),
        inputs: vec![InputDef {
            name: "inp".to_string(),
            source: src.to_string(),
        }],
        outputs: vec![],
    };
    let nodes = vec![
        // Two independent producers → level-0 roots (no inputs).
        mk_producer("pa"),
        mk_producer("pb"),
        // Two independent block consumers. Each is a Period node with a
        // NON-trigger `block` input → no DAG edge → also a level-0 root. So all
        // four nodes share ONE wide level 0 (not level 1).
        mk_consumer("ca", "pa/out"),
        mk_consumer("cb", "pb/out"),
    ];
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "two_block_pairs".to_string(),
        prefix: prefix.to_string(),
        nodes,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("pa".to_string(), Box::new(BlockFastProducerEntry::new()));
    factories.insert("pb".to_string(), Box::new(BlockFastProducerEntry::new()));
    factories.insert("ca".to_string(), Box::new(BlockSlowConsumerEntry::new()));
    factories.insert("cb".to_string(), Box::new(BlockSlowConsumerEntry::new()));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build two-block-pair graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(5));
    }
    runtime.trace().to_vec()
}

#[test]
#[serial]
fn all_serial_level_no_rayon_dispatch() {
    const STEPS: u32 = 24;

    // THREADS=4: the single level 0 is ALL block-involved, so `other_ids` (level
    // minus block-involved) is EMPTY → `tick_decided_parallel` gets ZERO
    // decisions → the `<= 1` fast-path (NO rayon dispatch). All four fire via the
    // fused block seam. The run must complete WITHOUT panicking and be
    // thread-count-invariant.
    let parallel = run_two_block_pairs("twobp_par", THREADS_PARALLEL, STEPS);
    let serial = run_two_block_pairs("twobp_ser", THREADS_SERIAL, STEPS);

    // Non-vacuous: every one of the four nodes fired at least once (the
    // run actually exercised the all-serial level, not just built). We do NOT
    // pin the producers' fire count: each producer feeds a depth-2 BLOCK
    // consumer, so block backpressure DEFERS some producer fires — the exact
    // producer count is pacing-dependent (but deterministic, pinned by the
    // byte-identity below). The two consumers, however, are never deferred —
    // they drain on every 20ms period = STEPS/4 drains each.
    for id in ["pa", "pb", "ca", "cb"] {
        assert!(
            parallel.iter().any(|e| &*e.node_id == id),
            "all-serial-level run must have fired node '{id}' at least once \
             (non-vacuous). trace={parallel:?}"
        );
    }
    let drains_per_consumer = STEPS as usize / 4; // 20ms drain / 5ms step.
    for id in ["ca", "cb"] {
        let n = parallel.iter().filter(|e| &*e.node_id == id).count();
        assert_eq!(
            n, drains_per_consumer,
            "block consumer '{id}' is never deferred — it drains on every 20ms \
             period ({drains_per_consumer} times over {STEPS} 5ms steps), got {n}"
        );
    }

    // The all-block level (empty `tick_decided_parallel` decisions → fast-path)
    // must still produce the byte-identical serial trace: the four block nodes
    // fire through the fused seam in deterministic (block-paced) order regardless
    // of the fire-pool thread count. This is the load-bearing pin.
    assert_eq!(
        parallel, serial,
        "an all-block level (tick_decided_parallel gets empty decisions → fast-path; \
         the four nodes fire via the fused block seam) must not panic AND must produce \
         a trace byte-identical to THREADS=1. parallel={parallel:?} serial={serial:?}"
    );
}

// ===========================================================================
// Test 9 (merge-order-decisive): a
// serial-gated node in the MIDDLE of decision order merges in DECISION position,
// not in fire-production (serial-gated-first) order.
//
// `tick_decided_parallel` fires serial-gated nodes on the calling thread
// FIRST (PASS 1) into their OWN per-node `trace_fragment`, then fires the REST
// (PASS 2, here via `par_values_mut` since the level is wide), then MERGES (PASS
// 3) by DRAINING `decisions` — which is already in decision (pos) order — and,
// per decision, draining that node's fragment into the trace. The pos-order
// drain is the ONLY thing that restores decision order: a regression that merged
// the fragments in FIRE-PRODUCTION order (serial-gated PASS 1 first, then the
// PASS 2 REST) would put the gated fragment FIRST, which only HAPPENS to match
// decision order when the gated node is FIRST or LAST in the level. Tests 1/2/6
// don't catch a non-pos merge (1/2 have no serial+parallel interleave; 6's gated
// node is LAST; 6 compares THREADS=4 vs =1 — both run the same buggy path →
// symmetric pass). This test puts the gated node MID-LEVEL and compares
// ENTRY-FOR-ENTRY against a bare `Scheduler::step` FLAT oracle (which fires
// evaluate_one in strict insertion/decision order = the absolute truth).
//
// Topology: a SINGLE WIDE level 0 of 9 Period(10) nodes (1 serial-gated + 8
// non-serial-gated REST fires; the REST is 8 ≥ the parallel threshold of 8 → it
// fires via `par_values_mut`) at decision positions 0,1,[gated],3,4,5,6,7,8.
// Decision position == graph/insertion order. Position 2 is a `ClosureNodeEntry`
// carrying a NON-TRIGGER plain input (→ no-op snapshot + in snapshot_input_names
// → set A of serial_fire_node_ids) so it fires in PASS 1 on the calling thread;
// the other 8 are macro `WideProducer` nodes that fire on the rayon pool
// (THREADS=4 forced). The non-trigger input keeps the gated
// node at level 0 (trigger edges define levels; a non-trigger edge does not —
// same as Test 6). If PASS 3 merged in production order the gated node's
// TraceEntry would land at trace index 0 (PASS 1 fired it first) instead of
// index 2 → diverges from the flat oracle → this test FAILS. Merging by draining
// `decisions` in pos order lands it at index 2, byte-identical to the flat
// oracle. (Mutation-confirmed: replacing the pos-order drain with a
// production-order merge makes this test fail.)
// ===========================================================================

#[test]
#[serial]
fn interleaved_gated_node_merges_in_decision_order() {
    const STEPS: u32 = 8;
    // 9 nodes, all level 0. Insertion (= decision) order: p0, p1, gated, p3, p4,
    // p5, p6, p7, p8. The gated node sits at decision position 2 — strictly in
    // the MIDDLE. The non-gated REST is 8 ≥ the parallel threshold (8) so the REST
    // fires via `par_values_mut` (the wide path), exercising the new wide-fire
    // mechanism.
    const IDS: [&str; 9] = ["p0", "p1", "gated", "p3", "p4", "p5", "p6", "p7", "p8"];

    // LEVEL path (real parallel, THREADS=4): 8 macro Period(10) producers + 1
    // mid-level serial-gated closure (Period(10) + a non-trigger plain input).
    let level_trace: Vec<TraceEntry> = {
        let _guard = FireThreadsGuard::set(THREADS_PARALLEL);
        let gated_reads = Arc::new(AtomicU64::new(0));

        let nodes: Vec<NodeDef> = vec![
            wide_producer_def("p0"),
            wide_producer_def("p1"),
            // The gated closure at decision position 2 — sourced from p0 (a
            // level-0 sibling); the input is NON-TRIGGER so it does NOT push the
            // closure to a later level (it stays at level 0, decision position 2).
            NodeDef {
                fuse: None,
                ros2: None,
                id: "gated".to_string(),
                node_type: "gated_closure".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "p0/out".to_string(),
                }],
                outputs: vec![],
            },
            wide_producer_def("p3"),
            wide_producer_def("p4"),
            wide_producer_def("p5"),
            wide_producer_def("p6"),
            wide_producer_def("p7"),
            wide_producer_def("p8"),
        ];

        let config = GraphConfig {
            execution: None,
            level_assignments: None,
            network: None,
            process_groups: Default::default(),
            process_group_order: Default::default(),
            multi_publisher_topics: Vec::new(),
            name: None,
            identity: "mid_gated".to_string(),
            prefix: "midg".to_string(),
            nodes,
        };
        let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        factories.insert("p0".to_string(), Box::new(WideProducerEntry::new()));
        factories.insert("p1".to_string(), Box::new(WideProducerEntry::new()));
        // Period(10) + a plain non-trigger input → no-op snapshot + present in
        // snapshot_input_names → routed to serial_fire_node_ids (set A). Fires
        // serially on the calling thread, mid-level.
        let info = NodeInfo::with_meta(vec![plain_input_meta("inp")], vec![])
            .with_policy(MacroPolicy::Period { period_ms: 10 });
        let reads_cb = Arc::clone(&gated_reads);
        let closure = ClosureNodeEntry::new(info, move |ctx| {
            // A real iceoryx2 read every tick (no fake data); value is recorded
            // but the trace assertion ignores it — this just proves the gated
            // tick actually runs.
            if let Some(sub) = ctx.subscriber_mut("inp") {
                if let Ok(Some(x)) = sub.try_view::<Vector3, _>(|view| view.x) {
                    reads_cb.store(x as u64, Ordering::Relaxed);
                }
            }
            Ok(())
        })
        .with_label("gated_closure");
        factories.insert("gated".to_string(), Box::new(closure));
        factories.insert("p3".to_string(), Box::new(WideProducerEntry::new()));
        factories.insert("p4".to_string(), Box::new(WideProducerEntry::new()));
        factories.insert("p5".to_string(), Box::new(WideProducerEntry::new()));
        factories.insert("p6".to_string(), Box::new(WideProducerEntry::new()));
        factories.insert("p7".to_string(), Box::new(WideProducerEntry::new()));
        factories.insert("p8".to_string(), Box::new(WideProducerEntry::new()));

        let clock = Arc::new(VirtualClock::new());
        let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
            .expect("build mid-gated graph");
        for _ in 0..STEPS {
            runtime.step(Duration::from_millis(10));
        }
        runtime.trace().to_vec()
    };

    // FLAT oracle: a bare Scheduler with the SAME ids in the SAME insertion order
    // + the SAME Period(10) policy. `Scheduler::step` fires evaluate_one in strict
    // insertion order = decision order = the absolute fire-order truth. Callback
    // bodies are irrelevant (trace records only node_id + fire_time_ns).
    let flat_trace: Vec<TraceEntry> = {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        for id in IDS {
            scheduler
                .add_node(NodeConfig {
                    id: id.to_string(),
                    policy: TriggerPolicy::Period {
                        interval: Duration::from_millis(10),
                        max_catchup: None,
                    },
                    callback: Box::new(|| {}),
                })
                .expect("add flat node");
        }
        for _ in 0..STEPS {
            scheduler.step_ms(10);
        }
        scheduler.trace().to_vec()
    };

    // Non-vacuous: 9 nodes × 8 steps = 72 fires; and the gated node fired (its
    // entries are present, so the mid-level merge actually placed something).
    let expected = IDS.len() * STEPS as usize;
    assert_eq!(
        level_trace.len(),
        expected,
        "level trace must record every fire (9 nodes × {STEPS} steps)"
    );
    assert!(
        level_trace.iter().any(|e| &*e.node_id == "gated"),
        "the mid-level serial-gated node must actually have fired (non-vacuous). \
         trace={level_trace:?}"
    );

    // THE merge-order-decisive pin: the gated node sits at decision position 2,
    // so within every step its TraceEntry must land at trace index 2 (between p1
    // and p3), byte-identical to the flat oracle. PASS 3 merges by DRAINING
    // `decisions` in decision (pos) order; a regression that merged the per-node
    // fragments in fire-production order (the serial-gated PASS 1 fragment first,
    // then the PASS 2 REST) would put "gated" at index 0 of each step's block →
    // diverges from the flat oracle here.
    assert_eq!(
        level_trace, flat_trace,
        "a serial-gated node MID-LEVEL (decision position 2) must merge into its \
         TRUE decision slot — the parallel executor's trace must be BYTE-IDENTICAL \
         to the FLAT Scheduler::step oracle (strict insertion/decision order). If \
         PASS 3 merged in fire-production order the gated fragment would land first \
         and this diverges. level={level_trace:?} flat={flat_trace:?}"
    );
}

// ===========================================================================
// Test 10: the wide rayon
// dispatch stamps each fire with its TRUE NON-ZERO global level.
//
// `TraceEntry.global_level` is threaded through every fire path. The NARROW
// single-node dispatch (`tick_decided` → `fire_node_into`) is pinned at a
// NON-ZERO level by `barrier_level_gate_iox2_test` (its context B owns global
// levels {2,3,4} = B-local {0,1,2}, so a "stamped local instead of global" bug
// is caught there). But the WIDE rayon dispatch — `tick_decided_parallel` → PASS
// 2's `par_values_mut().for_each(|..| fire_into_fragment(node, slot,
// global_level, step))` (a DISTINCT call site, taken ONLY when a level's
// non-serial REST is ≥ `PARALLEL_FIRE_THRESHOLD` = 8) — is exercised only by
// Test 1 (`parallel_serial_flat_traces_byte_identical`), whose wide level is
// GLOBAL LEVEL 0 (16 sources, no edges) and whose flat baseline ALSO stamps
// `global_level = 0` (the sentinel the flat `Scheduler::step` path uses). So at
// level 0, `0 == 0` MASKS a wide-path regression that drops/hardcodes
// `global_level` to 0 in `fire_into_fragment`'s wide call site.
//
// This test pins the WIDE path at a NON-ZERO global level: ONE Period(10) source
// at global level 0 feeds N (≥ 8) macro consumers that all data-trigger off the
// source's output → the N consumers form ONE wide level at GLOBAL LEVEL 1 (a
// trigger edge IS a level boundary), routed through the rayon path because
// N ≥ `PARALLEL_FIRE_THRESHOLD`. With the within-step level
// collapse, the source publishes at L0 and the consumers drain+fire at L1 in the
// SAME `step()` call. A drop-to-0 / mis-thread of `global_level` in the wide
// `fire_into_fragment` call site stamps the consumers `global_level = 0` →
// assertion (1) FAILS here — invisible at level 0 in Test 1. This is the
// wide-path analog of the merge-side global_level pin:
// the narrow path is level-pinned by the barrier test, the merge side
// by the trace-merge sort key, and the wide rayon fire by THIS test.
// ===========================================================================

/// Data-trigger sink: fires on `inp` (the source's output), records `inp.x`. A
/// `#[cerulion_node]` macro node ⇒ `performs_input_snapshot() == true` ⇒ NOT
/// serial-gated ⇒ fires on the rayon REST (PASS 2), NOT the serial PASS 1. No
/// output port — a pure leaf at global level 1.
#[cerulion_node]
#[derive(Default)]
struct WideConsumer {
    #[input(trigger)]
    inp: Vector3,
    seen: u64,
}

#[cerulion_node_impl]
impl WideConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen = self.inp.x as u64;
        Ok(())
    }
}

#[test]
#[serial]
fn wide_rayon_level_stamps_nonzero_global_level() {
    // N ≥ PARALLEL_FIRE_THRESHOLD (8) so the level-1 consumer batch routes WIDE
    // (`par_values_mut`). 10 gives margin above the threshold; the assert below
    // fails LOUDLY if a future threshold bump would route this test narrow.
    const N: usize = 10;
    const STEPS: u32 = 4;
    let threshold = GraphRuntime::parallel_fire_threshold();
    assert!(
        N >= threshold,
        "test precondition: N ({N}) must be >= PARALLEL_FIRE_THRESHOLD ({threshold}) so the \
         level-1 consumers route through the WIDE rayon path (`par_values_mut`)"
    );

    let _guard = FireThreadsGuard::set(THREADS_PARALLEL);

    let consumer_ids: Vec<String> = (0..N).map(|i| format!("c{i:02}")).collect();
    let mut nodes: Vec<NodeDef> = Vec::new();
    // The single Period(10) source — no inputs → a level-0 root.
    nodes.push(wide_producer_def("src"));
    // N macro consumers, each data-triggering off the source's single output →
    // all at global level 1, all in ONE wide level (trigger edge = level edge).
    for id in &consumer_ids {
        nodes.push(NodeDef {
            fuse: None,
            ros2: None,
            id: id.clone(),
            node_type: "wide_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: "src/out".to_string(),
            }],
            outputs: vec![],
        });
    }
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "wide_nonzero_level".to_string(),
        prefix: "wnz".to_string(),
        nodes,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("src".to_string(), Box::new(WideProducerEntry::new()));
    for id in &consumer_ids {
        factories.insert(id.clone(), Box::new(WideConsumerEntry::new()));
    }

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build wide-consumer graph");

    // ROUTING PIN (end-to-end classification): none of the consumers may be
    // serial-gated — they must be the non-serial REST the wide path fires via
    // `par_values_mut`. If a future change demoted them to PASS 1 (serial) the
    // wide call site under test would never run and this test would SILENTLY stop
    // covering it (mirrors `step_zero_alloc_test`'s serial_fire_node_ids pin).
    for id in &consumer_ids {
        assert!(
            !runtime.serial_fire_node_ids().contains(id),
            "consumer '{id}' must NOT be serial-gated (it must fire on the rayon REST); \
             serial_fire_node_ids = {:?}",
            runtime.serial_fire_node_ids()
        );
    }

    for _ in 0..STEPS {
        runtime.step(Duration::from_millis(10));
    }
    let trace = runtime.trace();

    // (2) The source's fires — must be the level-0 root, every step.
    let src_entries: Vec<&TraceEntry> = trace.iter().filter(|e| &*e.node_id == "src").collect();
    assert_eq!(
        src_entries.len(),
        STEPS as usize,
        "the Period(10) source fired once per step (non-vacuous); trace={trace:?}"
    );
    for e in &src_entries {
        assert_eq!(
            e.global_level, 0,
            "the Period source is a level-0 root — global_level must be 0, got {} (node {})",
            e.global_level, e.node_id
        );
    }

    // The consumers' fires (everything that is not the source). Each is a
    // distinct wide-level-1 fire stamped by `fire_into_fragment`'s wide call site.
    let consumer_entries: Vec<&TraceEntry> =
        trace.iter().filter(|e| &*e.node_id != "src").collect();
    // Non-vacuous: every consumer fired every step. The source publishes each
    // step → all N data-trigger and fire at L1 in the SAME step (the within-step
    // collapse) → N × STEPS consumer fires.
    assert_eq!(
        consumer_entries.len(),
        N * STEPS as usize,
        "every consumer fired every step in the wide level-1 batch \
         (N={N} × STEPS={STEPS}); trace={trace:?}"
    );

    // ROUTING PRECONDITION (live, not just static): in EVERY step the source
    // published once and ALL N consumers fired in ONE level-1 batch — so that
    // step's level-1 fire has rest_fire_count == N ≥ PARALLEL_FIRE_THRESHOLD ⇒
    // it genuinely took the `par_values_mut` wide path (not the narrow serial
    // REST). Without this the "wide" claim would be unverified.
    for step in 0..STEPS as u64 {
        let in_step = consumer_entries.iter().filter(|e| e.step == step).count();
        assert_eq!(
            in_step, N,
            "step {step}: all {N} consumers must fire in ONE wide level-1 batch \
             (≥ threshold {threshold} ⇒ rayon `par_values_mut`), got {in_step}"
        );
    }

    // (1) THE pin: every consumer fire is stamped its TRUE NON-ZERO global level
    // 1. A drop-to-0 / mis-thread of `global_level` in the wide
    // `fire_into_fragment` call site (PASS 2 `par_values_mut`) stamps 0 → fails
    // here. Invisible at level 0 (Test 1), the only OTHER wide-path test.
    for e in &consumer_entries {
        assert_eq!(
            e.global_level, 1,
            "the WIDE rayon dispatch must stamp consumer '{}' with its TRUE global \
             level 1 (it data-triggers off the level-0 source) — a wide-path \
             global_level drop/hardcode to 0 surfaces HERE, got {}",
            e.node_id, e.global_level
        );
    }

    // (3) bonus: `step` also threads through the wide path — each consumer fires
    // once per step with a strictly-increasing, contiguous `step` over 0..STEPS.
    for id in &consumer_ids {
        let steps_for: Vec<u64> = consumer_entries
            .iter()
            .filter(|e| &*e.node_id == id.as_str())
            .map(|e| e.step)
            .collect();
        assert_eq!(
            steps_for,
            (0..STEPS as u64).collect::<Vec<_>>(),
            "consumer '{id}' must fire once per step with a monotonically increasing \
             `step` (wide-path step threading), got {steps_for:?}"
        );
    }
}
