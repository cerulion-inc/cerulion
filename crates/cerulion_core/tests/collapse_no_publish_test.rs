// SPDX-License-Identifier: AGPL-3.0-only
//! A COLLAPSED tick must not arm publish on a fixed-schema
//! `#[output]`, over real iceoryx2 (`GraphRuntime::build_for_test`).
//!
//! # The bug this pins
//!
//! `cerulion_macros/src/impl_macro.rs`'s `build_nested_try_view` collapses
//! the WHOLE tick to a no-op when a non-trigger `#[input]` has nothing to
//! view (`Ok(None)` — genuinely no sample, no held/replayed value either).
//! Were this collapsed no-op and a genuinely successful tick
//! BOTH represented as the identical `Ok(Ok(()))`, `rewrite_tick_method`'s
//! publish-arm guard (`arm_on_ok_two_layer`, the "publish-on-success
//! inversion" mechanism) could not tell them apart — a collapsed tick's
//! already-loaned, zero-initialized fixed-schema output would get armed and
//! published as a real, new-sequence-numbered sample. Fabricated fake data,
//! violating Principle #13.
//!
//! The chain threads a `bool` discriminant (`Result<bool, NodeError>`,
//! `true` = ran, `false` = collapsed) through the `try_view` chain so the
//! arm guard keys on `Ok(Ok(true))` specifically.
//!
//! # Observability
//!
//! A downstream `#[input(trigger)]` sink data-triggers on the DUT's
//! output. Because delivery is trigger-gated, a recorded increment is
//! unambiguous proof the DUT published THIS step — there is no ambiguity
//! between "published a zero-init frame" and "did not publish at all"
//! (exactly the distinction this file pins). Every recorded value
//! comes from a REAL iceoryx2 publish read inside a real tick (no fake
//! data, Principle #13); every assertion is against a HAND-WRITTEN oracle
//! (e.g. "delivery count == 0"), never a self-compare.
//!
//! # Running (iceoryx2 SHM singleton → serial)
//!
//! ```bash
//! cargo test -p cerulion_core --test collapse_no_publish_test -- --test-threads=1
//! ```

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// The DUT's period; also used as the harness step delta so a Period DUT
/// fires on every step.
const STEP: Duration = Duration::from_millis(10);

/// Monotonic prefix counter so re-builds within one process never collide
/// on an iceoryx2 service name (mirrors non_trigger_hold_iox2_test /
/// cdylib_non_trigger_hold_test).
static PREFIX_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_prefix(stem: &str) -> String {
    format!("{stem}{}", PREFIX_COUNTER.fetch_add(1, Ordering::Relaxed))
}

fn vec3_out(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    }
}

// ===========================================================================
// Node types
// ===========================================================================

/// External producer publishing a FIXED scalar into `out.x` on each fire.
/// `HostDriven` so the harness controls EXACTLY which steps it fires.
#[cerulion_node(external)]
#[derive(Default)]
struct FixedProducer {
    #[output]
    out: Vector3,
    val: f64,
}

#[cerulion_node_impl]
impl FixedProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.val;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// Period DUT: fires every step regardless of `inp`'s state. Its plain
/// (default `DropOldest`) non-trigger `#[input]` is snapshotted at the
/// level boundary and HELD across silent steps once
/// delivered. Before ANY delivery the snapshot is `Empty` and the WHOLE
/// tick collapses — the scenario under test.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct PeriodDut {
    #[input]
    inp: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl PeriodDut {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.inp.x;
        Ok(())
    }
}

/// `block`-backpressure DUT: `external`/`HostDriven` so the harness
/// controls exactly which ticks fire. `block` inputs are EXCLUDED from
/// the snapshot/hold (they read LIVE, per
/// `non_trigger_hold_iox2_test.rs`'s `block_non_trigger_reads_live_not_held`)
/// — so once its one queued frame drains, the input goes genuinely Empty
/// on every subsequent tick (never a held replay). This is the sharpest
/// test of the collapse-must-not-publish contract: a node that ONCE
/// published a real value must go completely silent once its live input
/// runs dry, never re-publishing stale or zero-init content.
#[cerulion_node(external)]
#[derive(Default)]
struct BlockOutDut {
    #[input(backpressure = block, depth = 4)]
    inp: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl BlockOutDut {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.inp.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// Downstream observer: a DataTrigger sink that fires ONLY when the DUT
/// actually publishes `out`. `delivered` counts every fire (unambiguous
/// "did a publish reach here" oracle); `last_value` records what it read
/// (used to confirm an establishing delivery carried the REAL value, not
/// a fabricated one).
#[cerulion_node]
#[derive(Default)]
struct DeliverySink {
    #[input(trigger)]
    inp: Vector3,
    delivered: Arc<AtomicU64>,
    last_value: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl DeliverySink {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.delivered.fetch_add(1, Ordering::Relaxed);
        self.last_value.store(self.inp.x as u64, Ordering::Relaxed);
        Ok(())
    }
}

// ===========================================================================
// Graph builders
// ===========================================================================

/// `producer/out` (external, HostDriven) -> `dut.inp` (Period, plain
/// non-trigger) -> `sink.inp` (DataTrigger, observes `dut/out`).
fn build_period_graph(prefix: &str, val: f64) -> (GraphRuntime, Arc<AtomicU64>, Arc<AtomicU64>) {
    let delivered = Arc::new(AtomicU64::new(0));
    let last_value = Arc::new(AtomicU64::new(u64::MAX));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "collapse_no_publish_period".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "fixed_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "dut".to_string(),
                node_type: "period_dut".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "sink".to_string(),
                node_type: "delivery_sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "dut/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(FixedProducerEntry::with_state(FixedProducer {
            val,
            ..Default::default()
        })),
    );
    factories.insert("dut".to_string(), Box::new(PeriodDutEntry::new()));
    factories.insert(
        "sink".to_string(),
        Box::new(DeliverySinkEntry::with_state(DeliverySink {
            delivered: Arc::clone(&delivered),
            last_value: Arc::clone(&last_value),
            ..Default::default()
        })),
    );
    let clock = Arc::new(VirtualClock::new());
    let runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build period collapse graph");
    (runtime, delivered, last_value)
}

/// Same topology, `dut` node type swapped for the `block`-backpressure DUT
/// (external/HostDriven, so the harness triggers `producer` AND `dut`
/// explicitly).
fn build_block_graph(prefix: &str) -> (GraphRuntime, Arc<AtomicU64>, Arc<AtomicU64>) {
    let delivered = Arc::new(AtomicU64::new(0));
    let last_value = Arc::new(AtomicU64::new(u64::MAX));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "collapse_no_publish_block".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "fixed_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "dut".to_string(),
                node_type: "block_out_dut".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "sink".to_string(),
                node_type: "delivery_sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "dut/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(FixedProducerEntry::new()));
    factories.insert("dut".to_string(), Box::new(BlockOutDutEntry::new()));
    factories.insert(
        "sink".to_string(),
        Box::new(DeliverySinkEntry::with_state(DeliverySink {
            delivered: Arc::clone(&delivered),
            last_value: Arc::clone(&last_value),
            ..Default::default()
        })),
    );
    let clock = Arc::new(VirtualClock::new());
    let runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build block collapse graph");
    (runtime, delivered, last_value)
}

// ===========================================================================
// PIN 1 (HEADLINE): a Period DUT whose non-trigger input is NEVER
// delivered must NEVER publish — every tick collapses.
// ===========================================================================

#[test]
#[serial]
fn pre_first_delivery_never_publishes() {
    const N: u32 = 30;
    let (mut rt, delivered, _last_value) =
        build_period_graph(&unique_prefix("collapse_period_never"), 123.0);

    // producer NEVER triggered — dut's `inp` is NEVER delivered, so every
    // one of its Period(10ms) ticks must collapse.
    for _ in 0..N {
        rt.step(STEP);
    }

    assert_eq!(
        delivered.load(Ordering::Relaxed),
        0,
        "a Period DUT whose non-trigger input is NEVER delivered \
         must collapse EVERY tick and therefore must NEVER arm its \
         fixed-schema output's publish. The downstream data-trigger sink \
         must record ZERO deliveries across {N} steps (hand oracle: 0 — \
         a collapsed tick that still armed publish would make \
         this {N})."
    );
}

// ===========================================================================
// PIN 1b (DETERMINISM): two independent runs of the headline scenario are
// both pinned to the hand oracle (0), not just equal to each other.
// ===========================================================================

#[test]
#[serial]
fn pre_first_delivery_never_publishes_is_deterministic() {
    const N: u32 = 20;

    let run = || {
        let (mut rt, delivered, _last_value) =
            build_period_graph(&unique_prefix("collapse_period_det"), 7.0);
        for _ in 0..N {
            rt.step(STEP);
        }
        delivered.load(Ordering::Relaxed)
    };

    let a = run();
    let b = run();

    assert_eq!(
        a, b,
        "two runs of the never-fed collapse must be byte-identical"
    );
    assert_eq!(
        a, 0,
        "AND both must equal the hand oracle (anti-tautology): a collapsed \
         tick never publishes, so delivered count is 0 on every run. got {a}"
    );
}

// ===========================================================================
// PIN 2: a `block`-excluded input, established once then silent, must NOT
// re-publish stale/zero content once its queue drains.
// ===========================================================================

#[test]
#[serial]
fn block_excluded_dut_stops_publishing_once_silent() {
    const SILENT_STEPS: usize = 5;

    let (mut rt, delivered, last_value) = build_block_graph(&unique_prefix("collapse_block"));

    // Establish: fire the producer ONCE (FixedProducer defaults to
    // `val == 0.0`), then trigger the DUT (external, HostDriven) until its
    // `block` queue's one frame drains and the sink observes it — bounded
    // loop tolerates iceoryx2 connection warmup (mirrors
    // `block_non_trigger_reads_live_not_held`'s establish loop).
    rt.trigger_external("producer").expect("trigger producer");
    let mut tries = 0;
    loop {
        rt.trigger_external("dut").expect("trigger dut");
        rt.step(STEP);
        if delivered.load(Ordering::Relaxed) >= 1 {
            break;
        }
        tries += 1;
        assert!(
            tries < 200,
            "block DUT never published the established value within 200 tries"
        );
    }
    let established_count = delivered.load(Ordering::Relaxed);
    assert_eq!(
        established_count, 1,
        "exactly one delivery from the establish phase (hand oracle)"
    );
    assert_eq!(
        last_value.load(Ordering::Relaxed),
        0,
        "the established delivery must carry the real produced value (0.0), \
         not a fabricated one"
    );

    // Silent: producer NOT fired again. `block` is EXCLUDED from the
    // cross-step hold (non_trigger_hold_iox2_test.rs's
    // block_non_trigger_reads_live_not_held pins this), so the queue is
    // genuinely Empty on every subsequent trigger — the tick collapses.
    // A collapsed tick that still armed publish would deliver stale/zero
    // content; it must NOT publish at all, so `delivered` must
    // stay pinned at `established_count`.
    for _ in 0..SILENT_STEPS {
        rt.trigger_external("dut").expect("trigger dut");
        rt.step(STEP);
    }
    assert_eq!(
        delivered.load(Ordering::Relaxed),
        established_count,
        "once a `block`-excluded input's queue drains, EVERY \
         subsequent silent tick collapses and must NOT publish — delivered \
         count must stay pinned at {established_count} across \
         {SILENT_STEPS} silent triggers (hand oracle), not \
         {established_count} + {SILENT_STEPS}"
    );
}

// ===========================================================================
// PIN 3 (REGRESSION GUARD): a genuinely successful tick — a Period DUT
// whose input has been established and is now HELD (a cross-step replay, NOT
// a collapse: try_view returns Some every step) — must keep publishing on
// EVERY tick, completely unaffected by the collapse rule.
// ===========================================================================

#[test]
#[serial]
fn genuine_success_still_publishes_every_tick() {
    const V: u64 = 55;
    const HORIZON: u32 = 20;

    let (mut rt, delivered, last_value) =
        build_period_graph(&unique_prefix("collapse_period_held"), V as f64);

    // Establish: fire the producer ONCE, then step (bounded) until the
    // sink first observes V — producer and dut share level 0 (the `inp`
    // edge is non-trigger), so the dut's snapshot reads the producer's
    // PRIOR-step publish (mirrors `establish_held` across this test
    // family). Every step before establishment collapses (0 deliveries);
    // this loop terminates the instant the collapse ends.
    rt.trigger_external("producer").expect("trigger producer");
    let mut tries = 0;
    loop {
        rt.step(STEP);
        if last_value.load(Ordering::Relaxed) == V {
            break;
        }
        tries += 1;
        assert!(
            tries < 200,
            "period DUT never established the held value {V}"
        );
    }
    let established_delivered = delivered.load(Ordering::Relaxed);
    assert!(
        established_delivered >= 1,
        "at least one delivery once established"
    );

    // HORIZON more silent steps: producer never fires again, but the DUT
    // is Period(10) and its input is now HELD (the hold replays the
    // last-delivered value every step) — genuinely runs EVERY tick
    // (try_view returns Some, never None: NOT a collapse), so it must
    // publish EVERY tick.
    for _ in 0..HORIZON {
        rt.step(STEP);
    }
    let total_delivered = delivered.load(Ordering::Relaxed);
    assert_eq!(
        total_delivered - established_delivered,
        u64::from(HORIZON),
        "the collapse rule must not affect the genuine-success path: a Period DUT \
         whose input is HELD runs its body EVERY tick and must \
         publish EVERY tick — expected exactly {HORIZON} additional \
         deliveries after establish (hand oracle), got {}",
        total_delivered - established_delivered
    );
    assert_eq!(
        last_value.load(Ordering::Relaxed),
        V,
        "every held-replay delivery must carry the established value {V}"
    );
}

// ===========================================================================
// PIN 4 (MULTI-INPUT COMPOSITION): a node with 2+ non-trigger inputs
// must collapse (and NOT arm publish) when ANY one of them is empty, even
// if an EARLIER input in the nested try_view chain has real, held data.
// This directly exercises `build_nested_try_view`'s recursive bubbling —
// `Ok(Some(layer_result)) => Ok(layer_result)` — proving a `false` from a
// DEEPER (later) input correctly propagates all the way out through an
// OUTER (earlier) input that itself succeeded, rather than the outer
// level's own success accidentally re-arming things.
// ===========================================================================

/// External producer publishing a FIXED scalar into `out.x` on each fire —
/// a second, independently-triggerable instance of the same shape as
/// `FixedProducer` (kept distinct so `producer_a`/`producer_b` are two
/// separate node types with independent factories).
#[cerulion_node(external)]
#[derive(Default)]
struct FixedProducerB {
    #[output]
    out: Vector3,
    val: f64,
}

#[cerulion_node_impl]
impl FixedProducerB {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.val;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// Period DUT with TWO non-trigger inputs (`a`, `b`) and one output. Per
/// `build_nested_try_view`, BOTH inputs must have something to view for
/// the tick to run at all — `a` is nested OUTSIDE `b` (declaration order),
/// so this DUT is the exact shape needed to prove a collapse at the INNER
/// (`b`) input correctly suppresses arming even when the OUTER (`a`) input
/// already succeeded.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct TwoInputDut {
    #[input]
    a: Vector3,
    #[input]
    b: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl TwoInputDut {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.a.x + self.b.x;
        Ok(())
    }
}

fn build_two_input_graph(prefix: &str) -> (GraphRuntime, Arc<AtomicU64>, Arc<AtomicU64>) {
    let delivered = Arc::new(AtomicU64::new(0));
    let last_value = Arc::new(AtomicU64::new(u64::MAX));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "collapse_two_input".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer_a".to_string(),
                node_type: "fixed_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer_b".to_string(),
                node_type: "fixed_producer_b".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "dut".to_string(),
                node_type: "two_input_dut".to_string(),
                inputs: vec![
                    InputDef {
                        name: "a".to_string(),
                        source: "producer_a/out".to_string(),
                    },
                    InputDef {
                        name: "b".to_string(),
                        source: "producer_b/out".to_string(),
                    },
                ],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "sink".to_string(),
                node_type: "delivery_sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "dut/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer_a".to_string(),
        Box::new(FixedProducerEntry::with_state(FixedProducer {
            val: 10.0,
            ..Default::default()
        })),
    );
    factories.insert(
        "producer_b".to_string(),
        Box::new(FixedProducerBEntry::with_state(FixedProducerB {
            val: 100.0,
            ..Default::default()
        })),
    );
    factories.insert("dut".to_string(), Box::new(TwoInputDutEntry::new()));
    factories.insert(
        "sink".to_string(),
        Box::new(DeliverySinkEntry::with_state(DeliverySink {
            delivered: Arc::clone(&delivered),
            last_value: Arc::clone(&last_value),
            ..Default::default()
        })),
    );
    let clock = Arc::new(VirtualClock::new());
    let runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build two-input collapse graph");
    (runtime, delivered, last_value)
}

#[test]
#[serial]
fn two_input_dut_never_publishes_while_either_input_undelivered() {
    const N: u32 = 20;
    let (mut rt, delivered, _last_value) =
        build_two_input_graph(&unique_prefix("collapse_2in_neither"));

    // NEITHER producer triggered — both `a` and `b` are Empty on every tick.
    for _ in 0..N {
        rt.step(STEP);
    }
    assert_eq!(
        delivered.load(Ordering::Relaxed),
        0,
        "a 2-input DUT with BOTH inputs undelivered must never publish (hand \
         oracle: 0 deliveries across {N} steps)"
    );
}

#[test]
#[serial]
fn two_input_dut_never_publishes_when_only_the_inner_input_is_empty() {
    const N: u32 = 20;
    let (mut rt, delivered, _last_value) =
        build_two_input_graph(&unique_prefix("collapse_2in_inner_empty"));

    // Establish `a` ONLY (fire producer_a repeatedly; producer_b NEVER
    // fires). `a` is the OUTER input in the nested try_view chain
    // (declared first) — this is the exact composition under
    // test: does an outer input's real, held data accidentally
    // re-arm publish even though the INNER (`b`) input is still Empty?
    for _ in 0..5 {
        rt.trigger_external("producer_a")
            .expect("trigger producer_a");
        rt.step(STEP);
    }
    assert_eq!(
        delivered.load(Ordering::Relaxed),
        0,
        "multi-input composition: `a` has real held data but `b` \
         has NEVER delivered — the WHOLE tick must still collapse (the \
         inner input's `Ok(Ok(false))` must bubble out through the outer \
         input's successful view, per build_nested_try_view's \
         `Ok(Some(layer_result)) => Ok(layer_result)` forwarding arm), so \
         the sink must record ZERO deliveries (hand oracle: 0) across the \
         next {N} ticks",
    );

    // Continue stepping WITHOUT ever firing producer_b — the collapse must
    // persist indefinitely, not just transiently during establishment.
    for _ in 0..N {
        rt.step(STEP);
    }
    assert_eq!(
        delivered.load(Ordering::Relaxed),
        0,
        "the collapse must persist across {N} more steps while `b` remains \
         undelivered — got a nonzero delivery count, meaning the outer \
         input's success incorrectly re-armed publish"
    );
}

#[test]
#[serial]
fn two_input_dut_publishes_once_both_inputs_are_established() {
    const N: u32 = 15;
    let (mut rt, delivered, last_value) =
        build_two_input_graph(&unique_prefix("collapse_2in_both"));

    // Establish BOTH a (val=10.0) and b (val=100.0).
    rt.trigger_external("producer_a")
        .expect("trigger producer_a");
    rt.trigger_external("producer_b")
        .expect("trigger producer_b");
    let mut tries = 0;
    loop {
        rt.step(STEP);
        if last_value.load(Ordering::Relaxed) == 110 {
            break;
        }
        tries += 1;
        assert!(
            tries < 200,
            "two-input DUT never established a.x+b.x == 110 within 200 tries"
        );
    }
    let established = delivered.load(Ordering::Relaxed);
    assert!(
        established >= 1,
        "at least one delivery once both established"
    );

    // HORIZON more steps: both inputs are now HELD (cross-step replay) — the
    // tick genuinely runs (never collapses) every step, so it must publish
    // every step, carrying the SAME composed value every time.
    for _ in 0..N {
        rt.step(STEP);
    }
    assert_eq!(
        delivered.load(Ordering::Relaxed) - established,
        u64::from(N),
        "once BOTH inputs are held, the 2-input DUT must publish every \
         tick — expected exactly {N} additional deliveries (hand oracle)"
    );
    assert_eq!(
        last_value.load(Ordering::Relaxed),
        110,
        "every delivery once established must carry a.x+b.x == 10+100 == 110"
    );
}
