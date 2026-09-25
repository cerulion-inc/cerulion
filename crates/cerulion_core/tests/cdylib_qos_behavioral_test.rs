// SPDX-License-Identifier: AGPL-3.0-only
//! Cdylib-parity cluster 5: the QoS / backpressure knobs behaving END-TO-END on macro
//! cdylibs.
//!
//! `cdylib_depth_ffi_test` + `cdylib_qos_ffi_test` prove the DECLARATIONS
//! (`expect_within_ms`, `promise_within_ms`, `tick_within_ms`, `throttle_ms`,
//! `depth`, `sample(N)`, `block`) cross the info-JSON FFI. The in-process
//! suites (`expect_within_iox2_test`, `promise_within_iox2_test`,
//! `tick_within_iox2_test`, `backpressure_throttle_iox2_test`,
//! `backpressure_sample_iox2_test`) prove the BEHAVIOR — but never drove a
//! LOADED cdylib. This file closes that gap over real iceoryx2
//! (`GraphRuntime::build_for_test`), cribbing each in-process suite's stimulus:
//!
//! - **(a) expect_within** (`test_node_macro_qos_cdylib`, `velocity_in`, 20 ms):
//!   a slow (100 ms) producer trips `expect_within_missed_count`; a fast (5 ms)
//!   one keeps it quiet.
//! - **(b) promise_within** (`cmd_out`, 30 ms): the dylib publishing slower than
//!   its promise trips `promise_within_missed_count`; a fast producer keeps it
//!   quiet.
//! - **(c) tick_within** (`tick_within_ms = 500`; raised from 10 after a
//!   scheduler-preemption-class Linux CI flake — the budget is a WALL-CLOCK gate and
//!   a routine ≥10 ms scheduler preemption of the dylib's tick on a loaded
//!   runner blew it): the fast dylib tick stays inside budget ⇒
//!   `tick_within_missed_count == 0` on a quiet runner (liveness — see the
//!   SKIP note; the `> 0` budget-blown arm needs a deliberately-slow-tick
//!   cdylib). Made airtight under load: the harness wall-times every `step()`
//!   against the SAME budget and counts `slow_steps`; a step's wall time
//!   strictly CONTAINS the tick's wall time, so a correctly-wired counter can
//!   never exceed the harness's own slow-step count — `tick_miss <=
//!   slow_steps` always, `== 0` asserted only when `slow_steps == 0`. A
//!   false-firing counter (FFI wiring/units bugs: the budget crossing as 0,
//!   ns-vs-ms confusion — every µs-class tick blows ANY budget) still fails
//!   on any quiet run; a genuinely stalled run degrades the assertion instead
//!   of red-ing CI.
//! - **(d) throttle** (`throttle_ms = 5`): a 1 ms flood upstream is capped —
//!   the dylib's fire count is far below the upstream's, deterministically.
//! - **(e) sample** (`test_node_macro_depth_cdylib`, `aux` = `sample(7)`): a
//!   1 ms flood into the fired dylib's aux input decimates ⇒
//!   `backpressure_sampled_count("aux") > 0`.
//! - **(g) determinism**: the deterministic counters (expect/promise/fires) are
//!   bit-identical across two runs (Principle #7). (`tick_within` is
//!   wall-clocked, so it is EXCLUDED from the two-run compare.)
//!
//! SKIPPED:
//! - `bp-drop-oldest`: no existing cdylib fixture reliably overflows a
//!   `drop_oldest` input — `test_node_macro_period_input_cdylib` (period 10 ms,
//!   default depth 10) drains EXACTLY depth-per-window under a 1 ms flood
//!   (0 evictions), and `qos_cdylib`'s `velocity_in` is a trigger drained every
//!   step. A dedicated slow-draining fixture (e.g. `period_ms>=50` + bare
//!   `#[input]`) would be needed.
//! - `hold-block-excluded`: the depth cdylib reads its `block` input but has NO
//!   output, so "reads live not held" is not observable via it; the block
//!   plateau in `cdylib_depth_ffi_test` already IMPLIES block is excluded from
//!   the snapshot (the producer would mis-pace otherwise).
//! - `graph-max-slice-len`: the qos/depth cdylib outputs are FIXED schemas
//!   (`Vector3`, `MAX_SLICE_LEN == None`), so the schema-const provisioning
//!   oracle is degenerate; the only variable-output cdylib is the cluster-2
//!   Image fixture, and a clean opener-ceiling check there is blocked by
//!   single-writer / needs a >4 MiB payload.
//!
//! # Build requirement + serial
//!
//! Requires `cargo build -p test_node_macro_qos_cdylib -p test_node_macro_depth_cdylib`.
//! All tests `#[serial]` (cdylib `NODES` + iceoryx2 SHM singletons).

use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

fn find_cdylib(crate_name: &str) -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib(crate_name)
}

fn load(crate_name: &str) -> Box<dyn NodeEntry> {
    Box::new(DylibNodeEntry::load(&find_cdylib(crate_name)).expect("load fixture"))
}

fn out_def(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    }
}

// ---------------------------------------------------------------------------
// In-process drivers (produce Vector3 at various rates)
// ---------------------------------------------------------------------------

/// 100 ms — far slower than the qos node's 20 ms input window / 30 ms output
/// promise.
#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct SlowQosProducer {
    #[output]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl SlowQosProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// 5 ms — comfortably inside the qos node's windows.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct FastQosProducer {
    #[output]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl FastQosProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// 3 ms — the ONE rate at which the shipped qos fixture can be
/// driven into a real backlog against its own declared window. The node is
/// capped to one fire per 5 ms, so a 3 ms producer outruns it and its
/// depth-10 `velocity_in` queue fills; the FIFO head is then popped ~9 x 3 ms
/// behind the newest frame and held for up to the 5 ms defer, so the 20 ms
/// window lapses with frames still queued. The existing 1 ms flood CANNOT
/// reach that state: `drop_oldest` caps the head's age at the queue depth, so
/// 9 x 1 ms + 5 ms stays inside 20 ms and no window ever lapses.
#[cerulion_node(period_ms = 3)]
#[derive(Default)]
struct BackloggedQosProducer {
    #[output]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl BackloggedQosProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// 1 ms flood — used for the throttle cap (the qos node would fire every step
/// but throttle_ms=5 caps it).
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct FloodQosProducer {
    #[output]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl FloodQosProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Minimal drain for the qos node's `cmd_out` so the output topic has an
/// in-graph consumer edge (isolates the promise watchdog).
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct PlainDrainConsumer {
    #[input]
    inp: Vector3,
    last: f64,
}
#[cerulion_node_impl]
impl PlainDrainConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last = self.inp.x;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// QoS graph: producer -> qos cdylib -> drain
// ---------------------------------------------------------------------------

/// The qos fixture's declared `tick_within_ms`, mirrored for the harness's
/// slow-step timing. MUST equal the attr on `test_node_macro_qos_cdylib`'s
/// `#[cerulion_node(...)]` — `run_qos` asserts the loaded dylib's
/// `info().tick_within_ms()` against this const on every run, so a drift
/// between the two fails loudly instead of silently unsoundening the
/// `tick_miss <= slow_steps` containment bound (which only holds when both
/// sides measure against the SAME budget).
const TICK_WITHIN_BUDGET_MS: u64 = 500;

/// Result tuple for a qos run. `tick_miss` AND `slow_steps` are wall-clocked
/// (NOT replay-deterministic) — both excluded from the two-run determinism
/// compare.
struct QosRun {
    expect_miss: u64,
    /// `expect_within_ms` windows that lapsed on the dylib's FIFO
    /// trigger input while unconsumed arrivals were still signalled on it.
    expect_backlogged: u64,
    promise_miss: u64,
    tick_miss: u64,
    qos_fires: u64,
    prod_fires: u64,
    /// Steps whose WALL elapsed exceeded [`TICK_WITHIN_BUDGET_MS`]. A step's
    /// wall time strictly contains the dylib tick's wall time, so
    /// `tick_miss <= slow_steps` for a correctly-wired counter — the
    /// runner-load-proof upper bound for arm (c).
    slow_steps: u64,
}

fn run_qos(producer_type: &str, step_ms: u64, steps: usize) -> QosRun {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cqb_qos".to_string(),
        prefix: "cqbq".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "prod".to_string(),
                node_type: producer_type.to_string(),
                inputs: vec![],
                outputs: vec![out_def("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "qos".to_string(),
                node_type: "qos_node".to_string(),
                inputs: vec![InputDef {
                    name: "velocity_in".to_string(),
                    source: "prod/out".to_string(),
                }],
                outputs: vec![out_def("cmd_out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "cons".to_string(),
                node_type: "drain".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "qos/cmd_out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    let prod: Box<dyn NodeEntry> = match producer_type {
        "slow" => Box::new(SlowQosProducerEntry::new()),
        "fast" => Box::new(FastQosProducerEntry::new()),
        "flood" => Box::new(FloodQosProducerEntry::new()),
        "backlogged" => Box::new(BackloggedQosProducerEntry::new()),
        other => panic!("unknown producer {other}"),
    };
    factories.insert("prod".to_string(), prod);
    let qos_entry = load("test_node_macro_qos_cdylib");
    let declared = qos_entry
        .info()
        .expect("qos cdylib info should parse")
        .tick_within_ms();
    assert_eq!(
        declared,
        Some(TICK_WITHIN_BUDGET_MS),
        "TICK_WITHIN_BUDGET_MS must mirror the fixture's declared \
         tick_within_ms — the slow-step containment bound is only sound when \
         both sides measure against the SAME budget; got {declared:?}"
    );
    factories.insert("qos".to_string(), qos_entry);
    factories.insert("cons".to_string(), Box::new(PlainDrainConsumerEntry::new()));

    let clock = Arc::new(VirtualClock::new());
    let mut rt =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build qos graph");
    let mut slow_steps: u64 = 0;
    for _ in 0..steps {
        let wall = Instant::now();
        rt.step(Duration::from_millis(step_ms));
        if wall.elapsed() > Duration::from_millis(TICK_WITHIN_BUDGET_MS) {
            slow_steps += 1;
        }
    }
    let qos = rt.node_handle("qos").unwrap();
    QosRun {
        expect_miss: qos.expect_within_missed_count(),
        expect_backlogged: qos.expect_within_backlogged_count(),
        promise_miss: qos.promise_within_missed_count(),
        tick_miss: qos.tick_within_missed_count(),
        qos_fires: qos.fire_count(),
        prod_fires: rt.node_handle("prod").unwrap().fire_count(),
        slow_steps,
    }
}

// ===========================================================================
// (a) expect_within + (b) promise_within + (c) tick_within (one slow, one fast)
// ===========================================================================

#[test]
#[serial]
fn slow_producer_trips_expect_and_promise_watchdogs_on_the_dylib() {
    // 40 × 5 ms = 200 ms. The 100 ms producer starves the qos node's 20 ms
    // velocity_in window AND makes its cmd_out publishes 100 ms apart (> the
    // 30 ms promise) — BOTH counters go positive across the FFI.
    let run = run_qos("slow", 5, 40);
    assert!(
        run.expect_miss > 0,
        "(a) a 100ms producer must trip the dylib's 20ms expect_within (got {})",
        run.expect_miss
    );
    assert!(
        run.promise_miss > 0,
        "(b) the dylib publishing cmd_out every 100ms must break its 30ms \
         promise_within (got {})",
        run.promise_miss
    );
}

#[test]
#[serial]
fn fast_producer_keeps_qos_watchdogs_quiet_and_tick_inside_budget() {
    // 5 ms producer keeps velocity_in fresh (< 20 ms) and cmd_out timely
    // (< 30 ms); the fast dylib tick stays inside the 500 ms budget.
    let run = run_qos("fast", 5, 40);
    assert_eq!(
        run.expect_miss, 0,
        "(a) a 5ms producer must keep the dylib's expect_within quiet"
    );
    assert_eq!(
        run.promise_miss, 0,
        "(b) a 5ms cmd_out cadence must keep the dylib's promise_within quiet"
    );
    // (c) tick_within LIVENESS: the trivial 3-assign tick is orders of
    // magnitude inside the 500 ms budget, so on a quiet runner the counter
    // must be exactly 0. The budget is a WALL-CLOCK gate, so a loaded runner
    // CAN legitimately stall a step past it (the scheduler-preemption flake class: a
    // routine preemption exceeds a 10 ms budget on a loaded Linux runner); the
    // harness times every step() against the SAME budget, and a step's wall
    // time strictly CONTAINS the tick's wall time, so a correctly-wired
    // counter can NEVER exceed the harness's slow-step count. A false-firing
    // counter (FFI wiring/units bugs: the budget crossing as 0, ns-vs-ms
    // confusion — every µs-class tick blows ANY budget) still fails on any
    // quiet run via the `== 0` arm; a genuinely stalled run degrades to the
    // containment bound instead of red-ing CI. Only the inside-budget arm is
    // expressible — the `> 0` budget-blown arm needs a deliberately-slow-tick
    // cdylib (SKIP note in the module header).
    assert!(
        run.tick_miss <= run.slow_steps,
        "(c) tick_within_missed_count ({}) exceeded the harness's slow-step \
         count ({}) — the counter fired on a tick the harness measured as \
         inside-budget: a false-firing / mis-wired FFI counter",
        run.tick_miss,
        run.slow_steps
    );
    if run.slow_steps == 0 {
        assert_eq!(
            run.tick_miss, 0,
            "(c) the fast dylib tick must stay inside the {TICK_WITHIN_BUDGET_MS} ms \
             tick_within budget on a quiet runner (counter wired across the \
             FFI, does not false-fire; got {})",
            run.tick_miss
        );
    } else {
        eprintln!(
            "(c) degraded assertion: {} slow step(s) on this runner — \
             asserted tick_miss ({}) <= slow_steps only",
            run.slow_steps, run.tick_miss
        );
    }
    assert!(run.qos_fires > 0, "the qos dylib must have fired");
}

// ===========================================================================
// (d) throttle_ms=5 caps the dylib fire rate
// ===========================================================================

#[test]
#[serial]
fn throttle_caps_the_dylib_fire_rate() {
    // 1 ms flood into the qos node (data-triggered on velocity_in): it would
    // fire every step, but throttle_ms=5 caps it to ~one fire per 5 ms.
    let run = run_qos("flood", 1, 20);
    assert_eq!(run.prod_fires, 20, "the 1ms flood fires every step");
    assert!(
        run.qos_fires < run.prod_fires,
        "(d) throttle_ms=5 must cap the dylib below the 1ms flood rate \
         (qos={}, prod={})",
        run.qos_fires,
        run.prod_fires
    );
    // ~one fire per 5 ms over a ~20 ms window ≈ 3-5.
    assert!(
        (3..=5).contains(&run.qos_fires),
        "(d) the throttled dylib should fire ~once per 5ms (got {})",
        run.qos_fires
    );
}

// ===========================================================================
// (g) determinism of the wire-keyed counters (Principle #7)
// ===========================================================================

#[test]
#[serial]
fn qos_watchdog_counters_are_deterministic() {
    let a = run_qos("slow", 5, 40);
    let b = run_qos("slow", 5, 40);
    // expect/promise key off the wire timestamp + scheduler clock; fires are
    // scheduler-deterministic. tick_within is wall-clocked ⇒ excluded.
    assert_eq!(
        (a.expect_miss, a.promise_miss, a.qos_fires),
        (b.expect_miss, b.promise_miss, b.qos_fires),
        "the wire-keyed QoS counters + fires must be bit-identical across runs"
    );
    assert!(
        a.expect_miss > 0 && a.promise_miss > 0,
        "the deterministic run must actually trip both watchdogs"
    );
}

// ===========================================================================
// (e) sample(7) on the depth cdylib's aux input decimates
// ===========================================================================

/// 1 ms flood publishing TWO Vector3 outputs (`out` → depth `inp`,
/// `aux_out` → depth `aux`) so the sample(7) aux input is fed fast enough to
/// decimate (arrivals 1 ms apart vs a 7 ms accept window).
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct FloodDepthProducer {
    #[output]
    out: Vector3,
    #[output]
    aux_out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl FloodDepthProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        self.aux_out.x = self.n as f64;
        Ok(())
    }
}

fn run_sample(steps: usize) -> (u64, u64) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cqb_sample".to_string(),
        prefix: "cqbs".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "prod".to_string(),
                node_type: "flood_depth".to_string(),
                inputs: vec![],
                outputs: vec![out_def("out"), out_def("aux_out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "depth_probe".to_string(),
                inputs: vec![
                    InputDef {
                        name: "inp".to_string(),
                        source: "prod/out".to_string(),
                    },
                    InputDef {
                        name: "aux".to_string(),
                        source: "prod/aux_out".to_string(),
                    },
                ],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("prod".to_string(), Box::new(FloodDepthProducerEntry::new()));
    factories.insert("consumer".to_string(), load("test_node_macro_depth_cdylib"));
    let clock = Arc::new(VirtualClock::new());
    let mut rt =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build sample graph");
    // The depth cdylib is external/HostDriven → fire it every step so it READS
    // aux (driving the sample gate). Draining inp every fire keeps its block
    // input from overflowing.
    for _ in 0..steps {
        rt.trigger_external("consumer").expect("trigger_external");
        rt.step(Duration::from_millis(1));
    }
    let h = rt.node_handle("consumer").unwrap();
    (h.backpressure_sampled_count("aux"), h.fire_count())
}

#[test]
#[serial]
fn sample_gate_on_dylib_aux_decimates_fast_arrivals() {
    // aux arrives every 1 ms; sample(7) accepts at most one per 7 ms → most
    // reads are decimated.
    let steps = 30;
    let (sampled, fires) = run_sample(steps);
    assert!(
        fires > 0,
        "the depth cdylib must have fired via trigger_external"
    );
    assert!(
        sampled > 0,
        "(e) sample(7) must decimate the 1ms-spaced aux reads across the FFI \
         (got {sampled})"
    );
    assert!(
        (sampled as usize) < steps,
        "(e) sample(7) must still ACCEPT some aux reads — not gate everything \
         (got {sampled} of {steps})"
    );
}

#[test]
#[serial]
fn sample_decimation_on_dylib_is_deterministic() {
    let a = run_sample(30);
    let b = run_sample(30);
    assert_eq!(
        a, b,
        "sample decimation on the dylib must be bit-identical across runs \
         (keyed off the wire timestamp — Principle #7); a={a:?} b={b:?}"
    );
    assert!(a.0 > 0, "decimation actually happened");
}

// ===========================================================================
// (a2) FIFO trigger: a BACKLOGGED trigger input on the dylib counts no miss
// ===========================================================================

/// The no-inert-shipping proof for the FIFO-trigger change on the FFI path: a cdylib
/// data-trigger node's watchdog tracker really is marked as the node's FIFO
/// trigger, so a window that lapses while frames are queued on `velocity_in`
/// is reported as BACKLOG instead of counted as producer silence.
///
/// Stimulus arithmetic (all three numbers are the shipped fixture's, only the
/// producer rate is ours): `test_node_macro_qos_cdylib` declares
/// `throttle_ms = 5` and `expect_within_ms = 20` on a default depth-10 input.
/// A 3 ms producer outruns the 5 ms fire cap, so the queue fills and stays
/// full; `drop_oldest` then pins the FIFO head ~9 x 3 = 27 ms behind the
/// newest frame, and the boundary re-offers that head for up to the 5 ms
/// defer — so the 20 ms window lapses with several frames still queued.
///
/// The queue needs 10 / (1/3 - 1/5) = 75 ms to fill, so 240 x 1 ms leaves
/// ample margin. A tracker blind to the backlog would count a miss (and emit a `warn!`) for
/// every one of those windows on a producer that never stopped.
#[test]
#[serial]
fn a_backlogged_trigger_input_on_the_dylib_counts_no_expect_within_miss() {
    let run = run_qos("backlogged", 1, 240);
    assert_eq!(
        run.expect_miss, 0,
        "(a2) a window that lapsed while frames were queued on the dylib's \
         trigger input is BACKLOG, not silence — it must count no miss (got \
         {})",
        run.expect_miss
    );
    assert!(
        run.expect_backlogged > 0,
        "(a2) anti-vacuity AND the marking proof: the window must really have \
         lapsed, and it can only land in this bucket if the CDYLIB node's \
         tracker was marked as its FIFO trigger across the info-JSON FFI (got \
         {})",
        run.expect_backlogged
    );
    assert!(
        run.qos_fires > 0 && run.qos_fires < run.prod_fires,
        "(a2) the dylib must really run and really be throttled behind its \
         producer (dylib {} vs producer {})",
        run.qos_fires,
        run.prod_fires
    );
}
