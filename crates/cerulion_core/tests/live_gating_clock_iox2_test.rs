// SPDX-License-Identifier: AGPL-3.0-only
//! The deterministic-live Barrier gating clock, e2e over real iceoryx2.
//!
//! The deterministic-live build path ([`GraphRuntime::build_live_deterministic`],
//! reached in tests via [`GraphRuntime::build_for_test_barrier`]) wires the
//! scheduler onto the `Barrier` gating clock and makes the live loop advance that
//! clock by a fixed, run-INDEPENDENT logical QUANTUM (the graph's
//! `tightest_timing_ns`) on every `live_step` — instead of by the WALL elapsed of
//! the polled/default-live path. The payoff is that `fire_time_ns` is
//! replay-deterministic across runs (Principle #7): wall jitter does not leak
//! into the recorded gating time.
//!
//! This file starts from the SMOKE pin: a single `period_ms`
//! node driven through `run_live_step_once_for_test` must advance its consecutive
//! `fire_time_ns` by EXACTLY the quantum (`= period_ms * 1_000_000` for a lone
//! period node), proving the gate moves on the logical quantum and NOT on the
//! wall elapsed of the sleeps between live steps.
//!
//! The oracle is hand-computed from the quantum — NOT a two-run self-compare.
//!
//! `#[serial]` — the live loop builds an iceoryx2 WaitSet over the process-global
//! shared-memory singleton (mirrors `monitor_wait_park_iox2_test` /
//! `polled_vs_live_iox2_test`). Run with `--test-threads=1`.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// The lone node's period (ms). The deterministic-live quantum is derived from
/// the graph's `tightest_timing_ns`, which for a single `period_ms` node is
/// exactly `PERIOD_MS * 1_000_000` ns — that derived value IS the per-step gating
/// advance the oracle below checks for.
const PERIOD_MS: u64 = 4;
/// The hand-computed oracle quantum (ns): `tightest_timing_ns()` for one
/// `period_ms = PERIOD_MS` node.
const QUANTUM_NS: u64 = PERIOD_MS * 1_000_000;

/// A small per-iteration WaitSet timeout. The pure-Period graph has EMPTY
/// `sources`, so each `live_step` does not block on an event — it sleeps this
/// budget then steps. Kept small so the test runs fast; the WALL elapsed of these
/// sleeps is DELIBERATELY NOT the gating advance (that is the whole point — gating
/// moves by `QUANTUM_NS`, not by wall).
const STEP_TIMEOUT: Duration = Duration::from_millis(2);

/// The number of live steps to drive. With quantum == period the node fires once
/// per step, so this yields ~`LIVE_STEPS` fires (≥ the few we assert over).
const LIVE_STEPS: usize = 8;

// ===========================================================================
// Node — replicated inline (test binaries are separate crates).
// ===========================================================================

/// A pure-Period node with NO data-trigger INPUTS (so EMPTY `sources`). The
/// `#[cerulion_node]` macro requires ≥1 port field, so it carries a single
/// `#[output]` (irrelevant to the gating-clock property under test — only the
/// `period_ms` deadline drives firing). Bumps a shared counter each tick purely as
/// a liveness sanity backstop; the real oracle reads `fire_time_ns` from the trace.
#[cerulion_node(period_ms = 4)]
#[derive(Default)]
struct Ticker {
    #[output]
    out: Vector3,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl Ticker {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

/// The humanoid limb-rate node — `period_ms = 1` (1 kHz). Pure-Period, NO data
/// inputs (so EMPTY `sources`), single `#[output]` to satisfy the macro's ≥1-port
/// requirement. Bumps a shared counter as a liveness backstop; the real oracle
/// reads `fire_time_ns` from the trace.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct FastTicker {
    #[output]
    out: Vector3,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl FastTicker {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

/// The humanoid control-loop node — `period_ms = 33` (~30 Hz). Pure-Period, NO
/// data inputs. With the 1 kHz `FastTicker` present, the graph's
/// `tightest_timing_ns` (the deterministic-live QUANTUM) is `min(1ms, 33ms) =
/// 1ms`, so this node fires every 33 steps while `FastTicker` fires every step.
#[cerulion_node(period_ms = 33)]
#[derive(Default)]
struct SlowTicker {
    #[output]
    out: Vector3,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl SlowTicker {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

/// A node stacking a LARGE `period_ms = 50` with a
/// SMALLER `tick_within_ms = 5` (a per-node QoS budget). `tick_within_ms` and
/// `period_ms` are orthogonal gates that stack cleanly (see
/// `tick_within_iox2_test`'s `#[cerulion_node(period_ms = 10, tick_within_ms = 1)]`),
/// so the graph's `Scheduler::tightest_timing_ns()` is `min(50ms, 5ms) = 5ms` —
/// derived from the NON-`Period` `tick_within_ms` source, NOT the 50ms period and
/// NOT the 1ms data-driven fallback. Pure-Period (NO data inputs → empty
/// `sources`), single `#[output]` to satisfy the macro's ≥1-port requirement; the
/// tick body is irrelevant (the quantum is fixed at build, read directly below).
#[cerulion_node(period_ms = 50, tick_within_ms = 5)]
#[derive(Default)]
struct QuantumNode {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl QuantumNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        Ok(())
    }
}

/// A one-node `Ticker` (`period_ms = PERIOD_MS`) graph + factories, sharing
/// `fires`. NO inputs → empty `sources`. `id` == factory-map key
/// (`build_for_test_barrier` keys by node ID).
fn ticker_graph(fires: Arc<AtomicU64>) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "lgc_ticker".to_string(),
        prefix: "lgc".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "ticker".to_string(),
            node_type: "ticker".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "geometry_msgs/Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "ticker".to_string(),
        Box::new(TickerEntry::with_state(Ticker {
            fires: Arc::clone(&fires),
            ..Default::default()
        })),
    );
    (config, factories)
}

/// A two-node humanoid-interleave graph: a 1 kHz `FastTicker` (`fast`) + a ~30 Hz
/// `SlowTicker` (`slow`), each a pure-Period SOURCE (NO inputs → empty `sources`,
/// distinct derived output topics since the node ids differ). Both are level-0
/// sources, so the steps where BOTH fire (every 33rd) also exercise the
/// multi-fire level path. Factories are keyed by node ID (`build_for_test_barrier`
/// → `swap_remove(&node_def.id)`).
fn humanoid_graph(
    fast_fires: Arc<AtomicU64>,
    slow_fires: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let out_def = |name: &str| OutputDef {
        name: name.to_string(),
        schema: "geometry_msgs/Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    };
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "lgc_humanoid".to_string(),
        prefix: "hum".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "fast".to_string(),
                node_type: "fast".to_string(),
                inputs: vec![],
                outputs: vec![out_def("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "slow".to_string(),
                node_type: "slow".to_string(),
                inputs: vec![],
                outputs: vec![out_def("out")],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "fast".to_string(),
        Box::new(FastTickerEntry::with_state(FastTicker {
            fires: Arc::clone(&fast_fires),
            ..Default::default()
        })),
    );
    factories.insert(
        "slow".to_string(),
        Box::new(SlowTickerEntry::with_state(SlowTicker {
            fires: Arc::clone(&slow_fires),
            ..Default::default()
        })),
    );
    (config, factories)
}

/// A one-node `QuantumNode` graph (`period_ms = 50`, `tick_within_ms = 5`) +
/// factory. Used by the quantum-derivation pin: the deterministic-live
/// build must resolve its gating quantum to 5ms (the `tick_within_ms`), proving
/// the quantum is derived from the FULL `tightest_timing_ns` — not narrowed to the
/// `Period` cadence (50ms) or the 1ms fallback. `id` == factory-map key.
fn quantum_graph() -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "lgc_quantum".to_string(),
        prefix: "lgcq".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "qn".to_string(),
            node_type: "qn".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "geometry_msgs/Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("qn".to_string(), Box::new(QuantumNodeEntry::new()));
    (config, factories)
}

// ===========================================================================
// Test — fire_time_ns advances by exactly the quantum, wall-INDEPENDENT.
// ===========================================================================

#[test]
#[serial]
fn barrier_live_build_advances_fire_time_by_quantum() {
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = ticker_graph(Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    // Deterministic-live build: the scheduler rides the Barrier gating clock, and
    // `live_step` advances it by the derived quantum each step. NO manual clock
    // advance is needed (contrast the read-only-Real `build_for_test_with_policy`
    // path, where the test must advance the VirtualClock itself).
    let mut runtime = GraphRuntime::build_for_test_barrier(config, factories, clock, 8)
        .expect("build deterministic-live ticker graph");

    // Drive the live loop. The WALL elapsed of these `STEP_TIMEOUT` sleeps is
    // deliberately NOT the gating advance — the Barrier clock moves by QUANTUM_NS.
    for _ in 0..LIVE_STEPS {
        runtime.run_live_step_once_for_test(STEP_TIMEOUT);
    }

    // Collect the lone node's recorded fire times in trace order.
    let fire_times: Vec<u64> = runtime
        .trace()
        .iter()
        .filter(|e| &*e.node_id == "ticker")
        .map(|e| e.fire_time_ns)
        .collect();

    // Liveness: with quantum == period the node fires once per step, so we expect
    // multiple fires (need ≥2 to compare a consecutive delta at all).
    assert!(
        fire_times.len() >= 2,
        "expected the period node to fire repeatedly under the deterministic-live \
         park (got {} fires); the live loop is not advancing the gating clock",
        fire_times.len()
    );

    // ORACLE: consecutive `fire_time_ns` must differ by EXACTLY the logical quantum
    // (= PERIOD_MS * 1_000_000), proving the gate advances on the run-independent
    // quantum and NOT on the wall elapsed of the inter-step sleeps.
    for pair in fire_times.windows(2) {
        let delta = pair[1] - pair[0];
        assert_eq!(
            delta, QUANTUM_NS,
            "consecutive fire_time_ns must advance by exactly the quantum {} ns \
             (period {} ms); saw delta {} ns across {:?}",
            QUANTUM_NS, PERIOD_MS, delta, pair
        );
    }

    // Backstop: the trace fire count matches the tick-counter (no double-count).
    assert_eq!(
        fires.load(Ordering::Relaxed) as usize,
        fire_times.len(),
        "tick-counter and trace fire count must agree"
    );

    runtime.shutdown();
}

// ===========================================================================
// The deterministic-live quantum is
// derived from the FULL `tightest_timing_ns`, INCLUDING a NON-`Period` source.
// ===========================================================================

/// The deterministic-live build resolves its gating QUANTUM from the graph's FULL
/// `Scheduler::tightest_timing_ns()`, not from `Period` cadences alone.
/// `QuantumNode` stacks `period_ms = 50` with `tick_within_ms = 5`, so
/// `tightest_timing_ns()` = `min(50ms, 5ms) = 5ms` — the `tick_within_ms` budget.
///
/// A regression that narrowed the quantum derivation to `Period`-only would
/// resolve the 50ms period (or, if it also dropped the period, fall to the 1ms
/// data-driven fallback); either way `fire_time_ns` would silently ride the wrong
/// cadence. The EXACT 5ms hand-assert below catches all three: it is NOT 50ms (the
/// period), NOT 1ms (the fallback), and NOT a self-compare.
///
/// Direct pin via [`GraphRuntime::live_gating_quantum_for_test`] — the quantum is
/// fixed at build (`build_live_deterministic`), so no fire/timing is needed.
///
/// `#[serial]` — `build_for_test_barrier` builds an iceoryx2 node over the SHM
/// singleton (its isolated config root keeps it parallel-safe within the binary,
/// but the suite convention is `--test-threads=1`).
#[test]
#[serial]
fn barrier_build_quantum_is_derived_from_non_period_tick_within() {
    let clock = Arc::new(VirtualClock::new());
    let (config, factories) = quantum_graph();
    let runtime = GraphRuntime::build_for_test_barrier(config, factories, clock, 8)
        .expect("build deterministic-live quantum graph");

    // EXACT hand oracle: 5ms (the `tick_within_ms`), proving `tightest_timing_ns`
    // (the min across Period + every QoS source) drives the quantum — NOT a
    // Period-only narrowing (which would give 50ms) and NOT the 1ms fallback.
    assert_eq!(
        runtime.live_gating_quantum_for_test(),
        Some(Duration::from_millis(5)),
        "the deterministic-live gating quantum must be the 5ms tick_within_ms \
         (min of the 50ms period and the 5ms tick_within_ms), proving the quantum \
         is derived from the FULL tightest_timing_ns and NOT narrowed to the \
         50ms period or the 1ms data-driven fallback"
    );

    runtime.shutdown();
}

// ===========================================================================
// The HEADLINE pin.
// Wall-independence + cross-run determinism on a humanoid limb-vs-loop
// interleave, with the advance-by-wall mutation guard baked in.
// ===========================================================================

/// The deterministic-live gating advance is WALL-INDEPENDENT and
/// replay-deterministic across runs, on a realistic humanoid interleave (1 kHz
/// `FastTicker` + ~30 Hz `SlowTicker`).
///
/// The graph's `tightest_timing_ns` is `min(1ms, 33ms) = 1ms`, so the
/// deterministic-live loop advances the Barrier gating clock by EXACTLY 1ms per
/// `live_step` regardless of each step's WALL sleep. `FastTicker` fires once per
/// step (period == quantum == 1ms); `SlowTicker` fires every 33rd step.
///
/// ## ASSERT 1 — the advance-by-wall mutation guard (the headline)
/// Run A drives `K` steps with a 1ms WALL timeout; run B drives a FRESH build of
/// the SAME graph with a 4ms WALL timeout — a ~4× difference in real elapsed per
/// step. Because `step_live` on this path advances gating by the run-INDEPENDENT
/// QUANTUM (NOT by the wall delta of the polled/default-live path), both runs must
/// produce a BYTE-IDENTICAL trace, including every `fire_time_ns` (`TraceEntry`'s
/// `Eq` covers `node_id`/`step`/`global_level`/`fire_time_ns`; only the
/// wall-time `duration_ns` is excluded). If `step_live` ever regressed to
/// advancing gating by the WALL delta, run A (~1×) and run B (~4×) would advance
/// gating at different rates ⇒ divergent `fire_time_ns` ⇒ `trace_a == trace_b`
/// would FAIL. That equality is therefore the committed advance-by-wall guard.
///
/// ## ASSERT 2 — EXACT hand-oracle `fire_time_ns` SEQUENCES (NOT a self-compare,
/// NOT mere divisibility)
/// Hand-derived from the Period firing semantics + the 1ms quantum, checked over
/// run A. The Barrier gating clock starts at 0; `run_live_step_once_for_test`
/// advances it by EXACTLY one quantum (1ms) per call (the first call takes the
/// clock 0→1ms — see `polled_vs_live_iox2_test`'s `expected_fire_times_ns`), and
/// `advance_by_recorded` returns the POST-advance time, so step `s` (1-indexed)
/// stamps gating time `s·1ms`:
///   * `fast` (`period_ms = 1`): `next_fire` inits to 1ms (the clock reads 0 at
///     build); every step `s` has clock `s·1ms ≥ next_fire`, so it fires EVERY step
///     at `s·1ms` → the EXACT sequence `[1ms, 2ms, …, K·1ms]`;
///   * `slow` (`period_ms = 33`): `next_fire` inits to 33ms; it fires only when the
///     gating clock reaches a 33ms boundary — steps 33/66/99 (clock 33/66/99ms),
///     then `next_fire` 132ms > the K·1ms = 100ms horizon ⇒ no 4th fire → the EXACT
///     sequence `[33ms, 66ms, 99ms]`.
/// This is STRICTLY stronger than a divisibility check: a wrong-but-divisible
/// schedule (e.g. fast firing at 2/5/17ms) would satisfy `% 1ms == 0` yet FAIL this
/// exact-sequence oracle. The expected vectors are hand-derived from the semantics
/// above (independent of `trace_a`), so the comparison is a HAND ORACLE — not a
/// self-compare. It also subsumes an interleave-ratio assert (it pins the
/// exact counts: fast = K = 100, slow = 3).
///
/// `#[serial]` — real iceoryx2 WaitSet over the SHM singleton (the two builds use
/// distinct `generate_isolated_config` roots, so the sequential builds within the
/// test do not collide).
#[test]
#[serial]
fn humanoid_interleave_is_wall_independent_and_deterministic() {
    // K live steps. At quantum 1ms the slow (33ms) node fires at gating
    // 33/66/99ms ⇒ 3 fires within K; the fast (1ms) node fires every step.
    const K: usize = 100;

    // Drive a FRESH deterministic-live humanoid build for `K` steps at the given
    // per-step WALL `timeout`; return (full trace, fast tick-count, slow
    // tick-count). The gating advance is the run-INDEPENDENT 1ms quantum, so the
    // returned trace must NOT depend on `timeout` — that independence is ASSERT 1.
    let drive = |timeout: Duration| {
        let fast_fires = Arc::new(AtomicU64::new(0));
        let slow_fires = Arc::new(AtomicU64::new(0));
        let (config, factories) = humanoid_graph(Arc::clone(&fast_fires), Arc::clone(&slow_fires));
        let clock = Arc::new(VirtualClock::new());
        let mut runtime = GraphRuntime::build_for_test_barrier(config, factories, clock, 8)
            .expect("build deterministic-live humanoid graph");
        for _ in 0..K {
            runtime.run_live_step_once_for_test(timeout);
        }
        let trace = runtime.trace().to_vec();
        let fast = fast_fires.load(Ordering::Relaxed);
        let slow = slow_fires.load(Ordering::Relaxed);
        runtime.shutdown();
        (trace, fast, slow)
    };

    // Run A: 1ms WALL per step.   Run B: 4ms WALL per step (~4× the real elapsed).
    let (trace_a, fast_a, slow_a) = drive(Duration::from_millis(1));
    let (trace_b, _fast_b, _slow_b) = drive(Duration::from_millis(4));

    // ASSERT 1 — the headline. The two runs differ ONLY in WALL timeout, yet the
    // gating-driven trace must be byte-identical (advance-by-wall mutation guard).
    assert_eq!(
        trace_a, trace_b,
        "deterministic-live gating advance must be WALL-INDEPENDENT: a 1ms-timeout \
         run and a 4ms-timeout run must yield byte-identical traces (incl fire_time_ns)"
    );

    // Liveness backstop: trace fire counts agree with the per-node tick counters
    // (no double-count, no dropped fire). Computed off run A.
    let trace_fast = trace_a.iter().filter(|e| &*e.node_id == "fast").count() as u64;
    let trace_slow = trace_a.iter().filter(|e| &*e.node_id == "slow").count() as u64;
    assert_eq!(
        trace_fast, fast_a,
        "fast trace count must match its tick counter"
    );
    assert_eq!(
        trace_slow, slow_a,
        "slow trace count must match its tick counter"
    );

    // ASSERT 2 — EXACT hand-oracle fire_time_ns SEQUENCES over run A (NOT a
    // self-compare, NOT mere divisibility). See the test doc for the derivation.
    const FAST_PERIOD_NS: u64 = 1_000_000; // 1ms — the fast period == the quantum
    const SLOW_PERIOD_NS: u64 = 33_000_000; // 33ms — the slow period

    // The per-node fire_time_ns vectors, in trace order (the within-step interleave
    // between fast and slow is irrelevant — each node's own sequence is extracted).
    let fast_times: Vec<u64> = trace_a
        .iter()
        .filter(|e| &*e.node_id == "fast")
        .map(|e| e.fire_time_ns)
        .collect();
    let slow_times: Vec<u64> = trace_a
        .iter()
        .filter(|e| &*e.node_id == "slow")
        .map(|e| e.fire_time_ns)
        .collect();

    // HAND-DERIVED oracles (independent of trace_a). The gating clock reaches
    // s·1ms after step s, for s in 1..=K (horizon = K·1ms = 100ms).
    //   fast: fires EVERY step → [1ms, 2ms, …, K·1ms].
    let expected_fast: Vec<u64> = (1..=K as u64).map(|s| s * FAST_PERIOD_NS).collect();
    //   slow: fires on each 33ms multiple the clock reaches within the horizon →
    //         [33ms, 66ms, 99ms] (132ms > 100ms ⇒ stop). `take_while` derives the
    //         set from the semantics, so it is NOT read off trace_a.
    let horizon_ns: u64 = K as u64 * FAST_PERIOD_NS;
    let expected_slow: Vec<u64> = (1u64..)
        .map(|m| m * SLOW_PERIOD_NS)
        .take_while(|&t| t <= horizon_ns)
        .collect();

    assert_eq!(
        fast_times, expected_fast,
        "fast (period 1ms == quantum) must fire EVERY step at the EXACT gating \
         boundary s·1ms for s in 1..=K — exact-sequence oracle (a wrong-but-divisible \
         schedule like 2/5/17ms would pass divisibility but FAIL this)"
    );
    assert_eq!(
        slow_times, expected_slow,
        "slow (period 33ms) must fire ONLY on the 33ms gating boundaries reached \
         within K=100 steps — EXACTLY [33ms, 66ms, 99ms] (next_fire 132ms exceeds \
         the K·1ms = 100ms horizon, so no 4th fire)"
    );
}

// ===========================================================================
// The liveliness-unperturbed pin.
// The deterministic-live path's SEPARATE `RealClock` `watch_clock` still drives
// the wall-cadenced liveliness sweep; advancing the gating clock by the logical
// QUANTUM does NOT perturb it.
// ===========================================================================

/// Shared observation surface the consumer's `LivelinessEvent` handler writes
/// and the test reads (REAL handler execution — no fake data, Principle #13).
#[derive(Default, cerulion_core::state::CerulionState)]
struct LiveObs {
    /// `Lost` (`PublisherDisconnected`) handler firings.
    lost_fires: AtomicU64,
    /// `Alive` (`PublisherConnected`) handler firings.
    alive_fires: AtomicU64,
}

/// A periodic (5 ms) consumer of an absolute external topic (`/livegc/cam`) with
/// a `LivelinessEvent` handler. Periodic (not data-trigger) so it ticks EVERY
/// step and drains a pending liveliness event on the tick Ok-path even once the
/// publisher drops and data stops. The deterministic-live QUANTUM for this graph
/// is `min(5ms) = 5ms` (gating), DISTINCT from the WALL timeout the sweep cadence
/// reads — exactly the separation under test.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct LiveConsumer {
    #[input]
    inp: Vector3,
    sum: f64,
    obs: Arc<LiveObs>,
}

#[cerulion_node_impl]
impl LiveConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.sum += self.inp.x;
        Ok(())
    }

    /// Routed to `take_liveliness_event` by the `LivelinessEvent` param type.
    #[on_event(input = "inp")]
    fn on_live(&mut self, event: LivelinessEvent) {
        assert_eq!(
            event.input_name.as_ref(),
            "inp",
            "the event must carry its input name"
        );
        match event.state {
            LivelinessState::Lost => {
                assert_eq!(
                    event.cause,
                    LivelinessCause::PublisherDisconnected,
                    "Lost must carry PublisherDisconnected"
                );
                assert_eq!(event.publisher_count, 0, "Lost means no publishers remain");
                self.obs.lost_fires.fetch_add(1, Ordering::Relaxed);
            }
            LivelinessState::Alive => {
                assert!(
                    event.publisher_count >= 1,
                    "Alive means ≥1 publisher present"
                );
                self.obs.alive_fires.fetch_add(1, Ordering::Relaxed);
            }
            _ => unreachable!("only Lost/Alive exist on the graceful-disconnect path"),
        }
    }
}

/// A single-consumer graph whose ONLY input is an absolute external topic
/// (`/livegc/cam`) with NO in-graph producer, so the test attaches/drops external
/// publishers to drive real liveliness transitions.
fn liveliness_graph(obs: Arc<LiveObs>) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "lgc_liveliness".to_string(),
        prefix: "lgcl".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "cons".to_string(),
            node_type: "live_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: "/livegc/cam".to_string(),
            }],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "cons".to_string(),
        Box::new(LiveConsumerEntry::with_state(LiveConsumer {
            obs,
            ..Default::default()
        })),
    );
    (config, factories)
}

/// Attach an external publisher to `/livegc/cam` via the runtime's parked test
/// transport (the producer-less external topic takes the External provisioning
/// arm — iceoryx2's create-default publisher ceiling admits it with no opt-in).
fn attach_publisher(
    runtime: &GraphRuntime,
) -> cerulion_core::transport::publisher::CerulionPublisher {
    runtime
        .test_transport()
        .expect("test transport parked")
        .create_publisher("/livegc/cam", MaxSliceLen::const_new(256), 0)
        .expect("external publisher attaches to /livegc/cam")
}

/// On the deterministic-live path, the liveliness SWEEP is driven by the SEPARATE
/// `RealClock` `watch_clock` (WALL elapsed), NOT the quantum-advanced gating
/// clock — so a real publisher connect/disconnect is still observed even though
/// `fire_time_ns` rides the logical quantum.
///
/// The sweep order within `step_live` is unchanged from the polled path: each
/// level runs FIRST (the period consumer ticks, draining a PENDING liveliness
/// event), THEN `liveliness_sweep` observes the publisher count and PUSHES any
/// transition. So a transition the sweep pushes on step N is dispatched by the
/// tick on step N+1 — hence the two-step "observe then drain" cadence below
/// (identical to `liveliness_sweep_iox2_test`'s `observe_transition`).
///
/// The 1 ms sweep period vs the (≥3 ms) WALL `run_live_step_once_for_test`
/// timeout guarantees ≥1 sweep per step (and exactly one — the sweep is not a
/// catch-up loop), so the controlled attach→Alive→drop→Lost sequence is reliable.
/// `changed_at_ns` is REAL wall time on this path (NOT sim-clock), so — unlike
/// the polled `liveliness_sweep_iox2_test` — it is deliberately NOT asserted; the
/// COUNT-based observation (handler fires + per-node disconnect counter) is what
/// proves the sweep functions.
#[test]
#[serial]
fn liveliness_sweep_runs_on_deterministic_live_path() {
    // Per-step WALL timeout (the sweep cadence source) — DISTINCT from the 5 ms
    // gating quantum. ≥ the 1 ms sweep period below, guaranteeing ≥1 sweep/step.
    const STEP_WALL: Duration = Duration::from_millis(3);

    // Two live steps per observed transition: step 1's sweep observes + pushes the
    // transition; step 2's tick drains + dispatches the pushed event.
    let observe = |runtime: &mut GraphRuntime| {
        runtime.run_live_step_once_for_test(STEP_WALL);
        runtime.run_live_step_once_for_test(STEP_WALL);
    };

    let obs = Arc::new(LiveObs::default());
    let (config, factories) = liveliness_graph(Arc::clone(&obs));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test_barrier(config, factories, clock, 16)
        .expect("build deterministic-live liveliness graph");
    // 1 ms sweep cadence: the (≥3 ms) WALL timeout crosses it every step.
    runtime.set_liveliness_sweep_period_for_test(1);

    // Baseline 0 publishers at build → attach one → the next sweep observes the
    // Alive edge (0 → 1) through the RealClock-driven cadence.
    let pubr = attach_publisher(&runtime);
    observe(&mut runtime);
    assert_eq!(
        obs.alive_fires.load(Ordering::Relaxed),
        1,
        "attaching a publisher after a 0-baseline deterministic-live build is an \
         Alive transition observed by the RealClock-driven sweep"
    );
    assert_eq!(
        obs.lost_fires.load(Ordering::Relaxed),
        0,
        "no Lost yet — the publisher is still attached"
    );

    // Drop the publisher → the next wall-cadenced sweep observes the Lost edge
    // (1 → 0). The gating quantum advancing in lockstep does NOT mask it.
    drop(pubr);
    observe(&mut runtime);
    assert_eq!(
        obs.lost_fires.load(Ordering::Relaxed),
        1,
        "dropping the last publisher fires exactly one Lost handler — the SEPARATE \
         RealClock watch_clock still drives the sweep on the deterministic-live path"
    );
    let handle = runtime.node_handle("cons").expect("consumer node handle");
    assert_eq!(
        handle.publisher_disconnects_observed_count(),
        1,
        "the per-node disconnect counter bumps once on the Lost transition"
    );

    // Interleave unbroken: the period consumer ticked every step under the
    // deterministic-live park (its fires appear in the trace) — the liveliness
    // wiring did not stall firing.
    let cons_fires = runtime
        .trace()
        .iter()
        .filter(|e| &*e.node_id == "cons")
        .count();
    assert!(
        cons_fires >= 4,
        "the period consumer must keep firing every step under the deterministic-live \
         park (saw {cons_fires} fires across 4 steps)"
    );

    runtime.shutdown();
}

// ===========================================================================
// A direct caller of the THIN 4-arg
// `build_live_deterministic` (vs the `_with_schema_hashes_and_policy` variant the
// `build_for_test_barrier` helper funnels through). The thin ctor otherwise has
// ZERO callers; this gives it minimal coverage: build a deterministic-live runtime
// over a hand-built `init_for_test` transport, drive one live step, prove the node
// fires under the quantum-advanced Barrier gating clock.
// ===========================================================================

/// The thin [`GraphRuntime::build_live_deterministic`] (4 args) builds a working
/// deterministic-live runtime: it resolves the gating quantum from the graph and
/// the node fires under the quantum-advanced Barrier gating clock.
///
/// CLOCK CONTRACT: the transport MUST be built with the SAME `VirtualClock`
/// Arc passed to the build (a `debug_assert!` in
/// `build_live_deterministic_with_schema_hashes_and_policy` enforces it — QoS
/// anchors + `sample(N)` decimation are stamped from the transport clock and must
/// not drift from the gating timeline). So `clock.clone()` is handed to BOTH
/// `TransportConfig.clock` and the build — exactly as `build_for_test_barrier`
/// does internally, but here through the THIN public ctor.
///
/// `TransportConfig` / `TransportManager` come from the `cerulion_core::prelude::*`
/// glob; `iceoryx_test_config` is referenced fully-qualified.
///
/// `#[serial]` — the live loop builds an iceoryx2 WaitSet over the SHM singleton.
#[test]
#[serial]
fn build_live_deterministic_thin_ctor_fires_node() {
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = ticker_graph(Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());

    // Build the isolated test transport with the SAME clock Arc the build rides
    // (the clock contract). `clock.clone()` coerces Arc<VirtualClock> →
    // Arc<dyn Clock> at the struct field, so the transport stores the SAME
    // allocation the build upcasts → the build's `Arc::ptr_eq` invariant holds.
    let transport_config = TransportConfig {
        node_name: "cerulion_graph_test".into(),
        clock: clock.clone(),
        subscriber_buffer_size: 8,
        network: None,
    };
    let mgr = TransportManager::init_for_test(
        transport_config,
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated test transport");

    // DIRECT call of the thin 4-arg constructor (the one with no other callers).
    // `mgr` is borrowed only during the build; the returned runtime is owned. `mgr`
    // (declared before `runtime`) drops AFTER `runtime` is consumed by `shutdown()`,
    // so the iceoryx2 node outlives the runtime's ports.
    let mut runtime = GraphRuntime::build_live_deterministic(config, factories, &mgr, clock)
        .expect("build_live_deterministic ticker graph");

    // The thin ctor resolved the quantum through the same path: a lone period node
    // (no QoS / no smaller source) → the quantum is its PERIOD_MS (4ms) cadence.
    assert_eq!(
        runtime.live_gating_quantum_for_test(),
        Some(Duration::from_millis(PERIOD_MS)),
        "the thin build_live_deterministic must resolve the 4ms period quantum"
    );

    // Drive the live loop a few steps. With quantum == period the node fires once
    // per step, so it must have fired at least once under the gating-clock advance.
    for _ in 0..3 {
        runtime.run_live_step_once_for_test(STEP_TIMEOUT);
    }
    assert!(
        fires.load(Ordering::Relaxed) >= 1,
        "the node built via the THIN build_live_deterministic must fire under the \
         quantum-advanced deterministic-live gating clock (got 0 fires)"
    );

    runtime.shutdown();
    // Keep `mgr` alive until after the runtime is torn down (the node must outlive
    // the ports); the explicit drop documents the ordering.
    drop(mgr);
}

// ===========================================================================
// The deterministic-live external-topic
// silence deadline rides the SEPARATE `watch_clock` (a dedicated `RealClock`, so
// the grace is measured in WALL time), NOT the quantum-advanced gating clock.
//
// The build-time `external_silence_deadline_ns` seed is taken from
// `watch_clock`, and `check_external_silence` reads
// `watch_clock.now_ns()` (both in `runtime.rs`). On the det-live path `watch_clock`
// is a dedicated `RealClock` distinct from the gating `VirtualClock`, and no other test
// exercises that on the det-live path (the polled silence pins in
// `absolute_source_external_iox2_test` run where `watch == gating`), so a mutation
// reverting either site to the gating clock would escape them. This pin
// closes that escape on BOTH sites.
// ===========================================================================

/// The deterministic-live external-silence deadline is WALL-driven (`watch_clock`
/// = `RealClock`), independent of the gating quantum. Two mutually-reinforcing
/// parts, each with a hand oracle (NOT a self-compare):
///
/// PART 1 — BUILD seed rides `watch_clock` (kills a revert of the build seed to
/// the gating clock). No publisher attaches. The production 5s grace seeded from
/// the `RealClock` `watch_clock` (`real_ns` ≈ ns-since-boot, a large absolute
/// value) leaves a couple of short live steps WELL inside the grace → ZERO warn.
/// Mutation oracle: were the build seed taken from the gating clock, the deadline
/// would be ≈ `5e9` ns while `check_external_silence` reads `watch_clock.now_ns()`
/// = `real_ns` (≫ `5e9` on any machine up > 5s) → the warn would fire IMMEDIATELY on
/// step 1 → this `== 0` assertion fails.
///
/// PART 2 — the deadline crossing is a function of WALL time, not the gating
/// quantum (kills a revert of the `check_external_silence` COMPARISON to the gating
/// clock). The `set_external_silence_grace_for_test` seam reseeds the deadline
/// = `watch_clock.now_ns() + GRACE_MS*1e6` (the SAME recipe `build` uses). Driving
/// producer-less live steps whose summed WALL exceeds `GRACE_MS` (each blocks ~its
/// timeout — no event ever arrives on the no-publisher topic) fires the warn
/// EXACTLY once. Mutation oracle: were `check_external_silence` to compare the
/// gating clock, `gating.now()` ≈ `quantum * steps` (tens of ms) is astronomically
/// below the `watch`-seeded deadline (`real_ns` + grace, ≫ seconds-since-boot) →
/// the warn would NEVER fire → this `== 1` assertion fails (gets 0).
///
/// `tracing_test::traced_test` captures the warn; `logs_assert` accumulates over
/// the whole test, so PART 2's total count (0 from PART 1 + 1 from PART 2) is
/// exactly 1 — identical observation mechanism to
/// `absolute_source_external_iox2_test::external_topic_silence_warns_once_after_grace`,
/// here over the DETERMINISTIC-LIVE build + the WALL-keyed `run_live_step_once_for_test`.
///
/// `#[serial]` — real iceoryx2 WaitSet over the SHM singleton.
#[test]
#[serial]
#[tracing_test::traced_test]
fn external_silence_deadline_rides_watch_clock_on_deterministic_live_path() {
    let obs = Arc::new(LiveObs::default());
    let (config, factories) = liveliness_graph(Arc::clone(&obs));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test_barrier(config, factories, clock, 16)
        .expect("build deterministic-live external-silence graph");

    // PART 1: production 5s grace, RealClock-seeded → a couple of short steps stay
    // inside the grace → no warn. (Catches a gating-clock build-seed revert: that
    // would put the deadline at ~5e9 ns while watch_clock.now_ns() = real_ns ≫ 5e9
    // → an immediate step-1 warn.)
    const SHORT_WALL: Duration = Duration::from_millis(2);
    runtime.run_live_step_once_for_test(SHORT_WALL);
    runtime.run_live_step_once_for_test(SHORT_WALL);
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("NO publisher attach"))
            .count();
        if n == 0 {
            Ok(())
        } else {
            Err(format!(
                "deterministic-live silence warn must NOT fire inside the RealClock-seeded \
                 production grace (saw {n}); a gating-clock build seed would fire immediately"
            ))
        }
    });

    // PART 2: shorten the grace via the WALL-keyed seam, then cross it in WALL time.
    // GRACE_MS measured off watch_clock (RealClock); the 5ms gating quantum is
    // irrelevant to the crossing. ~6 steps × ~4ms WALL ≫ 6ms grace.
    const GRACE_MS: u64 = 6;
    const STEP_WALL: Duration = Duration::from_millis(4);
    runtime.set_external_silence_grace_for_test(GRACE_MS);
    for _ in 0..6 {
        runtime.run_live_step_once_for_test(STEP_WALL);
    }
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("NO publisher attach"))
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected EXACTLY one deterministic-live silence warn after the WALL grace was \
                 crossed (got {n}); the deadline rides watch_clock (RealClock), not the gating quantum"
            ))
        }
    });

    runtime.shutdown();
}
