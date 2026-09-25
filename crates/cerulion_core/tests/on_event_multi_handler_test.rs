// SPDX-License-Identifier: AGPL-3.0-only
//! Multi-handler runtime coverage for the type-routed
//! `#[on_event]` macro, end-to-end over real iceoryx2 via
//! `GraphRuntime::build_for_test` (per-test SHM root — parallel-safe; serial
//! iceoryx2 tests). Two tests:
//!
//!  1. `two_different_kind_handlers_on_same_input_both_fire` — proves the
//!     `(port, kind)` ALLOW-case: a `BackpressureEvent` handler and an
//!     `ExpectWithinEvent` handler on the SAME input are BOTH dispatched, each
//!     to its OWN `ctx.take_*_event` accessor (a routing-by-event-type
//!     regression that collapsed both onto one accessor would drop one).
//!
//!  2. `dispatch_order_is_declaration_order` — proves dispatch order ==
//!     SOURCE/declaration order, killing a HashMap-random regression AND a
//!     sort-by-port-name regression. Handlers are declared in REVERSE
//!     alphabetical order (`zeta` first, `alpha` second); a variant that sorts
//!     by name would emit `[alpha, zeta]`, so asserting the recorded order is
//!     `[zeta, alpha]` distinguishes declaration order from alphabetical order.
//!
//! No fake data (Principle #13): every fire increments a shared atomic / pushes
//! to a shared Vec from REAL handler execution. Deterministic (Principle #7):
//! the gates key off the wire `timestamp_ns` / `sequence` + the scheduler
//! `VirtualClock`, never wall-clock.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

// ===========================================================================
// Test 1: two DIFFERENT-kind handlers on the SAME input both fire.
// ===========================================================================

/// Publishes one Vector3 per 5 ms tick (wire timestamp = step time via the
/// shared VirtualClock).
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct FastProducer {
    #[output]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl FastProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Periodic (10 ms) consumer whose SINGLE input carries BOTH a `sample(50)`
/// read-gate AND a 30 ms `expect_within_ms` watchdog. The input is deliberately
/// NON-trigger: for a trigger input `drain_level` resets the watchdog
/// same-step on EVERY raw arrival (bypassing the sample gate), which would keep
/// the watchdog quiet — see `expect_within_iox2_test::SampleWatchConsumer` and
/// `sample_gate_decimation_does_not_reset_watchdog`. With a NON-trigger input
/// the ONLY watchdog reset is the body `try_view` read, which IS subject to the
/// gate. With the gate (50 ms) wider than the window (30 ms), accepted frames
/// are too sparse to satisfy the watchdog, so:
///   - the sample gate decimates → `BackpressureEvent` fires (`on_bp`), and
///   - decimated frames don't reset the watchdog → `ExpectWithinEvent` fires
///     (`on_stale`).
/// Both handlers are on the SAME input but route to DISTINCT accessors.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct DualHandlerConsumer {
    #[input(backpressure = sample(50), expect_within_ms = 30)]
    inp: Vector3,
    last: f64,
    /// Bumped inside the BackpressureEvent handler.
    bp_fires: Arc<AtomicU64>,
    /// Bumped inside the ExpectWithinEvent handler.
    stale_fires: Arc<AtomicU64>,
}
#[cerulion_node_impl]
impl DualHandlerConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Reading drives `try_view`: the sample gate decimates here (queuing the
        // BackpressureEvent on the first decimate of a regime) and a surviving
        // frame would reset the watchdog (it doesn't, because the gate is wider
        // than the window).
        self.last = self.inp.x;
        Ok(())
    }

    /// Routed to `take_backpressure_event` by the `BackpressureEvent` type.
    #[on_event(input = "inp")]
    fn on_bp(&mut self, event: BackpressureEvent) {
        assert_eq!(
            &*event.input_name, "inp",
            "BackpressureEvent must carry its input name"
        );
        assert!(matches!(event.policy, BackpressurePolicy::Sample(_)));
        self.bp_fires.fetch_add(1, Ordering::Relaxed);
    }

    /// Routed to `take_expect_within_event` by the `ExpectWithinEvent` type.
    #[on_event(input = "inp")]
    fn on_stale(&mut self, event: ExpectWithinEvent) {
        assert_eq!(
            &*event.input_name, "inp",
            "ExpectWithinEvent must carry its input name"
        );
        assert_eq!(event.expect_within_ms, 30);
        assert!(event.elapsed_ms > event.expect_within_ms);
        self.stale_fires.fetch_add(1, Ordering::Relaxed);
    }
}

fn dual_handler_graph(
    bp_fires: Arc<AtomicU64>,
    stale_fires: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "on_event_dual".to_string(),
        prefix: "oed".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "prod".to_string(),
                node_type: "fast_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "cons".to_string(),
                node_type: "dual_handler_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "prod/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("prod".to_string(), Box::new(FastProducerEntry::new()));
    let consumer = DualHandlerConsumer {
        bp_fires,
        stale_fires,
        ..Default::default()
    };
    factories.insert(
        "cons".to_string(),
        Box::new(DualHandlerConsumerEntry::with_state(consumer)),
    );
    (config, factories)
}

/// Run `steps` 5 ms steps; return (bp_fires, stale_fires).
fn run_dual_handler(steps: usize) -> (u64, u64) {
    let bp_fires = Arc::new(AtomicU64::new(0));
    let stale_fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = dual_handler_graph(Arc::clone(&bp_fires), Arc::clone(&stale_fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 16)
        .expect("build dual-handler graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(5));
    }
    (
        bp_fires.load(Ordering::Relaxed),
        stale_fires.load(Ordering::Relaxed),
    )
}

#[test]
fn two_different_kind_handlers_on_same_input_both_fire() {
    // 80 × 5 ms = 400 ms of wire time. A 5 ms producer over a sample(50) gate +
    // 30 ms watchdog drives both regimes: decimation (BackpressureEvent) and
    // window-miss (ExpectWithinEvent). BOTH handlers on the SAME input must fire
    // (routed to distinct accessors) — a routing-collapse regression would zero
    // one of them.
    let (bp, stale) = run_dual_handler(80);
    assert!(
        bp >= 1,
        "the BackpressureEvent handler on `inp` must fire under sample(50) \
         decimation (got {bp})"
    );
    assert!(
        stale >= 1,
        "the ExpectWithinEvent handler on `inp` must ALSO fire — the sample gate \
         (50 ms) is wider than the watchdog (30 ms), so accepted frames are too \
         sparse to keep the window fresh (got {stale})"
    );
}

#[test]
fn two_different_kind_handlers_are_deterministic() {
    // Principle #7: both fire counts bit-identical across two runs.
    let a = run_dual_handler(80);
    let b = run_dual_handler(80);
    assert_eq!(
        a, b,
        "both same-input handler fire counts must be deterministic across runs \
         (a={a:?} b={b:?})"
    );
    assert!(a.0 >= 1 && a.1 >= 1, "both handlers actually fired");
}

// ===========================================================================
// Test 2: dispatch order == DECLARATION order (not alphabetical).
// ===========================================================================

/// Fast flood producer (1 ms) — far faster than the order-consumer drains, so
/// its `drop_oldest` input overflows and the eviction detector fires.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct FloodProducer {
    #[output]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl FloodProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Consumer with TWO `drop_oldest` inputs, `zeta` and `alpha`. The two
/// `#[on_event]` handlers are declared in REVERSE alphabetical order — the
/// handler for `zeta` FIRST, the handler for `alpha` SECOND — so a variant
/// that sorts by port name would dispatch `[alpha, zeta]` while the correct
/// (declaration-order) dispatch is `[zeta, alpha]`. The reverse-alphabetical
/// choice is what makes the order assertion DISTINGUISH the two.
///
/// Both inputs are fed by identical 1 ms flood producers and drained together
/// each 12 ms tick. After the detectors establish their per-stream baseline on
/// the first drain, the next drain evicts on BOTH inputs in lockstep, so both
/// events are pending in the SAME tick — and the Ok-path dispatch (emitted after
/// the tick body) fires both handlers in declaration order within that one tick.
///
/// Per-tick capture: `tick()` snapshots the previous tick's handler order into
/// `recorded` (only when both fired) and clears the per-tick scratch. The
/// handlers append their port name to the scratch. Because dispatch runs AFTER
/// the tick body, the scratch holds exactly the co-fire order for that tick.
#[cerulion_node(period_ms = 12)]
#[derive(Default)]
struct OrderConsumer {
    #[input(backpressure = drop_oldest)]
    zeta: Vector3,
    #[input(backpressure = drop_oldest)]
    alpha: Vector3,
    last: f64,
    /// Per-tick scratch: handler port names appended in dispatch order; read +
    /// cleared at the start of the NEXT tick.
    per_tick: Arc<Mutex<Vec<String>>>,
    /// One entry per tick where BOTH handlers fired: that tick's dispatch order.
    recorded: Arc<Mutex<Vec<Vec<String>>>>,
}
#[cerulion_node_impl]
impl OrderConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Snapshot the PREVIOUS tick's co-fire (if both handlers fired), then
        // clear the scratch for this tick's dispatch (which runs after this body
        // on the Ok path).
        {
            let mut scratch = self.per_tick.lock().unwrap();
            if scratch.len() == 2 {
                self.recorded.lock().unwrap().push(scratch.clone());
            }
            scratch.clear();
        }
        // Read BOTH inputs to drive both `drop_oldest` detectors this tick.
        self.last = self.zeta.x + self.alpha.x;
        Ok(())
    }

    // Declared FIRST: the handler for `zeta` (reverse-alphabetical on purpose —
    // see the struct doc; a variant that sorts by name would emit this SECOND).
    #[on_event(input = "zeta")]
    fn on_zeta(&mut self, event: BackpressureEvent) {
        assert_eq!(&*event.input_name, "zeta");
        self.per_tick.lock().unwrap().push("zeta".to_string());
    }

    // Declared SECOND: the handler for `alpha`.
    #[on_event(input = "alpha")]
    fn on_alpha(&mut self, event: BackpressureEvent) {
        assert_eq!(&*event.input_name, "alpha");
        self.per_tick.lock().unwrap().push("alpha".to_string());
    }
}

fn order_graph(
    recorded: Arc<Mutex<Vec<Vec<String>>>>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "on_event_order".to_string(),
        prefix: "oeo".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "prod_zeta".to_string(),
                node_type: "flood_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "prod_alpha".to_string(),
                node_type: "flood_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "cons".to_string(),
                node_type: "order_consumer".to_string(),
                inputs: vec![
                    InputDef {
                        name: "zeta".to_string(),
                        source: "prod_zeta/out".to_string(),
                    },
                    InputDef {
                        name: "alpha".to_string(),
                        source: "prod_alpha/out".to_string(),
                    },
                ],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("prod_zeta".to_string(), Box::new(FloodProducerEntry::new()));
    factories.insert(
        "prod_alpha".to_string(),
        Box::new(FloodProducerEntry::new()),
    );
    let consumer = OrderConsumer {
        per_tick: Arc::new(Mutex::new(Vec::new())),
        recorded,
        ..Default::default()
    };
    factories.insert(
        "cons".to_string(),
        Box::new(OrderConsumerEntry::with_state(consumer)),
    );
    (config, factories)
}

/// Run the order graph for `steps` 1 ms steps; return the recorded per-tick
/// co-fire orders.
fn run_order(steps: usize) -> Vec<Vec<String>> {
    let recorded = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = order_graph(Arc::clone(&recorded));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build order graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(1));
    }
    // One extra tick so the FINAL co-fire's scratch is snapshotted into
    // `recorded` (the snapshot happens at the START of the following tick).
    runtime.step(Duration::from_millis(12));
    let out = recorded.lock().unwrap().clone();
    out
}

#[test]
fn dispatch_order_is_declaration_order() {
    // 60 × 1 ms producers vs a 12 ms consumer: ~12 frames/drain into the 10-deep
    // `drop_oldest` queues → both inputs evict each drain after baseline. Both
    // events are pending in the same tick, so both handlers co-fire.
    let recorded = run_order(60);
    assert!(
        !recorded.is_empty(),
        "at least one tick must have co-fired both handlers (got none) — the \
         flood must overflow both drop_oldest queues so both events pend together"
    );
    // EVERY co-fire tick must be in DECLARATION order. `zeta` is declared FIRST,
    // `alpha` SECOND, which is REVERSE alphabetical — so a variant that sorts
    // by port name would record `["alpha","zeta"]`, failing this assertion,
    // while a variant with HashMap-random ordering would record a mix. Only
    // declaration-order dispatch records `["zeta","alpha"]` for every co-fire.
    for (i, order) in recorded.iter().enumerate() {
        assert_eq!(
            order,
            &vec!["zeta".to_string(), "alpha".to_string()],
            "co-fire tick {i} must dispatch in DECLARATION order [zeta, alpha], \
             not alphabetical [alpha, zeta] (a sort-by-name regression) — got {order:?}"
        );
    }
}

#[test]
fn dispatch_order_is_deterministic() {
    // Principle #7: the recorded per-tick order timeline is bit-identical across
    // two runs (the eviction detector keys off the wire `sequence`, not wall).
    let a = run_order(60);
    let b = run_order(60);
    assert_eq!(
        a, b,
        "recorded co-fire order timeline must be deterministic across runs"
    );
    assert!(!a.is_empty(), "both handlers actually co-fired");
}
