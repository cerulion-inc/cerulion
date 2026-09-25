// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end `expect_within_ms` INPUT watchdog over real
//! iceoryx2 via `GraphRuntime::build_for_test` (per-test SHM root —
//! parallel-safe).
//!
//! The watchdog is INERT unless the runtime wires it: the scheduler has all the
//! machinery (`set_expect_within` / `step()` miss check / counter), and if
//! `GraphRuntime` did not wire it from `InputMeta.expect_within_ms`,
//! `expect_within_missed_count()` would be permanently 0 on real graphs. These
//! tests prove it fires e2e:
//!
//! - a TRIGGER input (`#[input(trigger, expect_within_ms = N)]`) counts a
//!   miss when the upstream is too slow, and stays quiet when it is fast
//!   enough — `drain_level` resets the window same-step on arrival;
//! - a NON-TRIGGER input (`#[input(expect_within_ms = N)]`, read in the tick
//!   body) resets on each delivered body read (no false-fire while data
//!   flows) and fires when arrivals are sparse — the subscriber writes the
//!   wire timestamp into the shared window anchor on `try_view` delivery.
//!
//! Counts are bit-identical across runs (Principle #7): the watchdog keys off
//! the wire `timestamp_ns` + the scheduler clock, never wall time.
//!
//! The rate-mismatch design needs no external-trigger / stop mechanism: a
//! producer slower than the consumer's window forces misses; a producer
//! faster than the window keeps it quiet. Fully deterministic under
//! `VirtualClock`.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::error::TransportResult;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{
    BackpressurePolicy, InputMeta, MacroPolicy, NodeContext, NodeEntry, NodeInfo,
};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::message::ShmMessage;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

// --- Producers (publish one Vector3 per period tick) -----------------------

/// Fast: publishes every 5 ms — well inside the consumers' 30 ms window.
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

/// Slow: publishes every 100 ms — far outside the 30 ms window, so the
/// consumer's watchdog misses several 30 ms windows between arrivals.
#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct SlowProducer {
    #[output]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl SlowProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

// --- Consumers (expect fresh data within 30 ms) ----------------------------

/// Data-triggered: fires on arrival. The trigger drain (`drain_level`)
/// resets the watchdog window same-step.
#[cerulion_node]
#[derive(Default)]
struct TriggerWatchConsumer {
    #[input(trigger, expect_within_ms = 30)]
    inp: Vector3,
    last: f64,
}
#[cerulion_node_impl]
impl TriggerWatchConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last = self.inp.x;
        Ok(())
    }
}

/// Periodic (10 ms): reads its NON-trigger input in the tick body, which
/// drives `try_view` and resets the watchdog window on each delivered sample.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct NonTriggerWatchConsumer {
    #[input(expect_within_ms = 30)]
    inp: Vector3,
    last: f64,
}
#[cerulion_node_impl]
impl NonTriggerWatchConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Reading the input drives `try_view`; on a delivered sample the
        // subscriber writes its wire timestamp into the shared watchdog
        // anchor, resetting the window for non-trigger inputs.
        self.last = self.inp.x;
        Ok(())
    }
}

/// Periodic (10 ms) consumer whose input carries BOTH a `sample(50)` read-gate
/// AND a 30 ms `expect_within_ms` watchdog. The two are
/// orthogonal: the sample gate decimates reads to ≥ 50 ms apart, and ONLY a
/// frame that survives the gate (a delivered sample) resets the watchdog. With
/// the gate (50 ms) wider than the window (30 ms), accepted frames are too
/// sparse to satisfy the watchdog — so it fires regardless of how fast the
/// producer publishes. Pins the gate-decimate-before-anchor-reset ordering in
/// `subscriber.rs::try_view`.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct SampleWatchConsumer {
    #[input(backpressure = sample(50), expect_within_ms = 30)]
    inp: Vector3,
    last: f64,
}
#[cerulion_node_impl]
impl SampleWatchConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last = self.inp.x;
        Ok(())
    }
}

/// Hand-written (NON-macro) data-triggered consumer whose `tick()`
/// DELIBERATELY does not read its input. The macro always generates a body
/// `try_view` for every declared input (which would reset the watchdog one
/// step later and mask the same-step reset), so a hand-written node is the
/// only way to make `drain_level`'s `signal_input_received` the SOLE reset
/// path for the `expect_within_ms` watchdog. Deleting that call (e.g. the
/// mutation a crashed review agent once left in the tree) then becomes a
/// mutation kill: with no reset at all, a fast producer's window still lapses
/// and the watchdog false-fires.
struct NoReadTriggerConsumer {
    /// Held so the node's subscribers/publishers survive for the run; never
    /// read (no `try_view` → no body-side watchdog reset).
    _context: Option<NodeContext>,
}

impl NoReadTriggerConsumer {
    fn new() -> Self {
        Self { _context: None }
    }
}

impl NodeEntry for NoReadTriggerConsumer {
    fn info(&self) -> TransportResult<NodeInfo> {
        Ok(NodeInfo::with_meta(
            vec![InputMeta {
                name: "inp".to_string(),
                schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
                trigger: true,
                depth: 10,
                backpressure: BackpressurePolicy::DropOldest,
                expect_within_ms: Some(30),
            }],
            Vec::new(),
        )
        .with_policy(MacroPolicy::DataTrigger {
            input_name: "inp".to_string(),
        }))
    }

    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        self._context = Some(context);
        Ok(())
    }

    fn tick(&mut self) -> TransportResult<()> {
        // Intentionally empty — the data trigger fires the node, but the body
        // never reads `inp`, so the ONLY thing keeping the 30 ms watchdog quiet
        // is `drain_level` → `signal_input_received` (the same-step reset).
        Ok(())
    }

    fn shutdown(&mut self) -> TransportResult<()> {
        Ok(())
    }
}

/// Hand-written data-triggered consumer that DRAINS the
/// edge-triggered `ExpectWithinEvent` in its tick body via
/// `ctx.take_expect_within_event` and counts how many it observes. Proves the
/// e2e wiring end-to-end: the runtime mints the per-node QoS store → injects it
/// into this context (`set_qos_event_store`) → the scheduler `push_*`es on a
/// watchdog miss in `step()` → the node drains it from `tick`. Like
/// `NoReadTriggerConsumer` it never reads its input body, so the watchdog reset
/// is solely `drain_level`'s `signal_input_received` (a real arrival rearms
/// the edge latch).
struct DrainExpectConsumer {
    context: Option<NodeContext>,
    events_seen: Arc<AtomicU64>,
}

impl DrainExpectConsumer {
    fn new(events_seen: Arc<AtomicU64>) -> Self {
        Self {
            context: None,
            events_seen,
        }
    }
}

impl NodeEntry for DrainExpectConsumer {
    fn info(&self) -> TransportResult<NodeInfo> {
        Ok(NodeInfo::with_meta(
            vec![InputMeta {
                name: "inp".to_string(),
                schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
                trigger: true,
                depth: 10,
                backpressure: BackpressurePolicy::DropOldest,
                expect_within_ms: Some(30),
            }],
            Vec::new(),
        )
        .with_policy(MacroPolicy::DataTrigger {
            input_name: "inp".to_string(),
        }))
    }

    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        self.context = Some(context);
        Ok(())
    }

    fn tick(&mut self) -> TransportResult<()> {
        // Drain the edge-triggered watchdog event (at most one per silence
        // regime). Intentionally does NOT read `inp` — the rearm path is
        // `signal_input_received`, not a body read.
        if let Some(ctx) = self.context.as_mut() {
            if ctx.take_expect_within_event("inp").is_some() {
                self.events_seen.fetch_add(1, Ordering::Relaxed);
            }
        }
        Ok(())
    }

    fn shutdown(&mut self) -> TransportResult<()> {
        Ok(())
    }
}

// --- Graph builders --------------------------------------------------------

fn producer_def(id: &str, node_type: &str) -> NodeDef {
    NodeDef {
        fuse: None,
        ros2: None,
        id: id.to_string(),
        node_type: node_type.to_string(),
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

fn consumer_def(id: &str, node_type: &str) -> NodeDef {
    NodeDef {
        fuse: None,
        ros2: None,
        id: id.to_string(),
        node_type: node_type.to_string(),
        inputs: vec![InputDef {
            name: "inp".to_string(),
            source: "prod/out".to_string(),
        }],
        outputs: vec![],
    }
}

/// Build a producer→consumer graph. `producer_type` selects fast/slow;
/// `consumer_type` selects trigger/non-trigger.
fn watch_graph(
    producer_type: &str,
    consumer_type: &str,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "expect_within_test".to_string(),
        prefix: "ew".to_string(),
        nodes: vec![
            producer_def("prod", producer_type),
            consumer_def("cons", consumer_type),
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    let producer: Box<dyn NodeEntry> = match producer_type {
        "fast" => Box::new(FastProducerEntry::new()),
        "slow" => Box::new(SlowProducerEntry::new()),
        other => panic!("unknown producer type {other}"),
    };
    let consumer: Box<dyn NodeEntry> = match consumer_type {
        "trigger" => Box::new(TriggerWatchConsumerEntry::new()),
        "nontrigger" => Box::new(NonTriggerWatchConsumerEntry::new()),
        "sample" => Box::new(SampleWatchConsumerEntry::new()),
        "noread" => Box::new(NoReadTriggerConsumer::new()),
        other => panic!("unknown consumer type {other}"),
    };
    // `build_for_test` keys the factory map by NODE ID (not node_type).
    factories.insert("prod".to_string(), producer);
    factories.insert("cons".to_string(), consumer);
    (config, factories)
}

/// Run the graph for `steps` 5 ms steps; return the consumer's
/// `(expect_within_missed_count, expect_within_backlogged_count)`.
///
/// BOTH buckets are returned because a lapsed window now lands in
/// exactly one of them, so `missed == 0` on its own no longer distinguishes
/// "the window never lapsed" from "every lapse was suppressed as backlog".
/// Every arm below therefore states what it expects of both.
fn run_expect_within(producer_type: &str, consumer_type: &str, steps: usize) -> (u64, u64) {
    let (config, factories) = watch_graph(producer_type, consumer_type);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build watch graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(5));
    }
    let handle = runtime.node_handle("cons").unwrap();
    (
        handle.expect_within_missed_count(),
        handle.expect_within_backlogged_count(),
    )
}

// --- Tests -----------------------------------------------------------------

#[test]
fn trigger_input_watchdog_fires_when_producer_too_slow() {
    // Slow producer (100 ms) vs a 30 ms expect window → the consumer misses
    // several windows between arrivals (and before the first arrival).
    // 40 steps × 5 ms = 200 ms.
    let (misses, backlogged) = run_expect_within("slow", "trigger", 40);
    assert!(
        misses > 0,
        "a 100 ms producer must trip the 30 ms expect_within watchdog (got {misses})"
    );
    // Every lapse here is genuine SILENCE — the consumer serves each
    // sparse arrival on the step it lands, so the backlog bucket must stay
    // empty. Without this, an over-suppressing guard could move every lapse
    // into the other bucket and the `> 0` oracle above would still be blind
    // to it only because the arrival step itself carries a pending count.
    assert_eq!(
        backlogged, 0,
        "a starved (not backlogged) trigger input must count NO backlogged \
         windows (got {backlogged})"
    );
}

#[test]
fn trigger_input_watchdog_quiet_when_producer_fast_enough() {
    // Fast producer (5 ms) vs a 30 ms window → data always fresh, never a miss.
    let (misses, backlogged) = run_expect_within("fast", "trigger", 40);
    assert_eq!(
        misses, 0,
        "a 5 ms producer must keep the 30 ms expect_within watchdog quiet (got {misses})"
    );
    // Quiet because the data is FRESH, not because a backlog
    // suppressed the window — the distinction this arm would otherwise lose.
    assert_eq!(
        backlogged, 0,
        "a fast producer's window must never even lapse, so nothing can be \
         suppressed as backlog (got {backlogged})"
    );
}

#[test]
fn non_trigger_input_watchdog_resets_on_body_reads() {
    // The KEY non-trigger correctness pin: the consumer reads its non-trigger
    // input every 10 ms in its body; data arrives every 5 ms, so every read
    // delivers a fresh sample and resets the 30 ms window — NO false-fire
    // while data flows. (This pins the reset path for
    // non-trigger inputs.)
    let (misses, _backlogged) = run_expect_within("fast", "nontrigger", 40);
    assert_eq!(
        misses, 0,
        "non-trigger body reads of fresh data must keep the watchdog quiet (got {misses})"
    );
}

#[test]
fn non_trigger_input_watchdog_fires_when_data_sparse() {
    // Slow producer (100 ms) vs a 30 ms window: most 10 ms body reads find no
    // new data (try_view → None → no reset), so the window lapses and the
    // watchdog fires between the sparse arrivals.
    let (misses, _backlogged) = run_expect_within("slow", "nontrigger", 40);
    assert!(
        misses > 0,
        "sparse 100 ms arrivals must trip the non-trigger 30 ms watchdog (got {misses})"
    );
}

#[test]
fn sample_gate_decimation_does_not_reset_watchdog() {
    // An input with BOTH sample(50) and
    // expect_within_ms = 30. Even a FAST (5 ms) producer trips the watchdog,
    // because the sample gate decimates accepted frames to ≥ 50 ms apart
    // (> the 30 ms window) and a decimated frame does NOT reset the watchdog
    // (it returns before the anchor store in try_view). So the window lapses
    // between accepted frames.
    let (misses, _backlogged) = run_expect_within("fast", "sample", 40);
    assert!(
        misses > 0,
        "sample(50) decimation below the 30 ms window must trip the watchdog \
         even with a 5 ms producer (got {misses})"
    );
}

#[test]
fn signal_input_received_is_the_sole_reset_for_a_non_reading_trigger_node() {
    // Distinct e2e pin for `signal_input_received`.
    // The consumer is data-triggered and fed by a FAST (5 ms) producer, but
    // its hand-written `tick()` NEVER reads the input — so there is no body
    // `try_view` reset. The watchdog (30 ms) stays quiet ONLY because
    // `drain_level` calls `signal_input_received` same-step on each
    // arrival. Deleting that call (a mutation that once landed in the tree
    // by accident) makes this assertion fail: with no reset path at all,
    // the 30 ms window lapses between the scheduler's own miss-resets and the
    // counter climbs.
    let (misses, backlogged) = run_expect_within("fast", "noread", 40);
    assert_eq!(
        misses, 0,
        "a fast producer must keep the watchdog quiet via signal_input_received \
         even though the node never reads its input in the body (got {misses})"
    );
    // This assertion is required to catch the defect below, and it must stay
    // here. This consumer is a hand-written `NodeEntry` (⇒ the SEPARATE drain
    // discipline) whose tick never reads `inp`, so `drain_level` signals one
    // arrival per drained timestamp and `decide_node` consumes it the same
    // step: `pending_data_count == 1` at EVERY `run_qos_windows` evaluation.
    // With `signal_input_received` deleted the anchor never moves and every
    // 30 ms window lapses — but each lapse arrives with a backlog, so the
    // backlog guard routes ALL of them into the backlogged bucket and
    // `misses` stays 0. Asserting `missed == 0` alone therefore no longer
    // catches the defect; asserting BOTH buckets restores full coverage.
    assert_eq!(
        backlogged, 0,
        "the window must never lapse at all — a non-zero backlogged count \
         means the anchor stopped being reset and the lapses were merely \
         re-bucketed (got {backlogged})"
    );
}

#[test]
fn expect_within_counter_is_deterministic() {
    // Same graph, two runs → bit-identical miss counts (Principle #7). The
    // watchdog keys off the wire timestamp + scheduler clock, never wall time.
    let a = run_expect_within("slow", "trigger", 40);
    let b = run_expect_within("slow", "trigger", 40);
    assert_eq!(
        a, b,
        "expect_within miss/backlog counts must be deterministic across runs"
    );
    // And the non-trigger path likewise.
    let c = run_expect_within("slow", "nontrigger", 40);
    let d = run_expect_within("slow", "nontrigger", 40);
    assert_eq!(
        c, d,
        "non-trigger expect_within counts must be deterministic"
    );
}

// --- e2e reactable ExpectWithinEvent drain ---------------------------------

/// Build a slow/fast-producer → `DrainExpectConsumer` graph, run `steps` 5 ms
/// steps, and return `(events_drained_in_body, expect_within_missed_count)`.
fn run_drain_expect(producer_type: &str, steps: usize) -> (u64, u64) {
    let events_seen = Arc::new(AtomicU64::new(0));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "expect_within_drain_test".to_string(),
        prefix: "ewd".to_string(),
        nodes: vec![
            producer_def("prod", producer_type),
            consumer_def("cons", "drainexpect"),
        ],
    };
    let producer: Box<dyn NodeEntry> = match producer_type {
        "fast" => Box::new(FastProducerEntry::new()),
        "slow" => Box::new(SlowProducerEntry::new()),
        other => panic!("unknown producer type {other}"),
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("prod".to_string(), producer);
    factories.insert(
        "cons".to_string(),
        Box::new(DrainExpectConsumer::new(Arc::clone(&events_seen))),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build drain graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(5));
    }
    let misses = runtime
        .node_handle("cons")
        .unwrap()
        .expect_within_missed_count();
    (events_seen.load(Ordering::Relaxed), misses)
}

#[test]
fn expect_within_event_drained_in_node_body_e2e() {
    // Slow producer (100 ms) vs a 30 ms window over 400 ms (80 × 5 ms): several
    // silence regimes. The node drains the edge-triggered event in its tick.
    let (events, misses) = run_drain_expect("slow", 80);
    assert!(
        misses > 0,
        "slow producer must trip the watchdog (misses={misses})"
    );
    assert!(
        events >= 1,
        "the edge-triggered event must reach the node body e2e (events={events})"
    );
    assert!(
        events < misses,
        "edge-trigger: strictly fewer events than counter bumps \
         (events={events}, misses={misses})"
    );
    assert!(
        events >= 2,
        "multiple silence regimes must rearm the latch and fire again \
         (events={events})"
    );
}

#[test]
fn expect_within_event_quiet_when_producer_fast_enough_e2e() {
    // Fast producer (5 ms) keeps the 30 ms window fresh → no miss, no event.
    let (events, misses) = run_drain_expect("fast", 80);
    assert_eq!(
        misses, 0,
        "fast producer keeps the watchdog quiet (misses={misses})"
    );
    assert_eq!(events, 0, "no miss ⇒ no event drained (events={events})");
}

#[test]
fn expect_within_event_drain_is_deterministic_e2e() {
    // Principle #7: identical (events, misses) across two runs.
    let a = run_drain_expect("slow", 80);
    let b = run_drain_expect("slow", 80);
    assert_eq!(a, b, "e2e event drain must be deterministic across runs");
}
