// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end proof that the type-routed `#[on_event]` macro
//! handler fires for the WATCHDOG events — `ExpectWithinEvent` (input-scoped)
//! and `PromiseWithinEvent` (output-scoped) — over real iceoryx2 via
//! `GraphRuntime::build_for_test` (per-test SHM root — parallel-safe; serial
//! iceoryx2 tests).
//!
//! This is NEW capability: the old `#[on_backpressure]` handler covered ONLY
//! `BackpressureEvent`; the watchdog events had no declarative handler at all
//! (they were drained manually via `ctx.take_{expect,promise}_within_event` —
//! see `expect_within_iox2_test::DrainExpectConsumer` and
//! `promise_within_iox2_test::DrainPromiseProducer`). The macro routes the
//! handler by its event PARAMETER TYPE, so the SAME `#[on_event]` keyword now
//! binds these two windows to the matching `ctx.take_*_event` accessor.
//!
//! Determinism (Principle #7): the watchdog keys off the wire `timestamp_ns`
//! and the scheduler clock, never wall time. We assert the handler fires >= 1
//! AND the fire count is bit-identical across two runs.
//!
//! No fake data (Principle #13): each handler increments a shared
//! `Arc<AtomicU64>` injected via the macro-generated `with_state(inner)`
//! constructor, so the count is observed from REAL handler execution.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::scheduler::TraceEntry;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

// ===========================================================================
// ExpectWithinEvent — INPUT-scoped `#[on_event(input = "...")]`
// ===========================================================================

/// Slow producer (100 ms) — far outside the consumer's 30 ms expect window,
/// so the consumer's watchdog misses several windows between arrivals.
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

/// Fast producer (5 ms) — well inside the 30 ms window, so the watchdog stays
/// quiet (used to prove the no-false-fire quiet path).
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

/// Data-triggered consumer with a 30 ms input watchdog. Its
/// `#[on_event(input = "inp")] fn on_stale(&mut self, ev: ExpectWithinEvent)`
/// handler — type-routed to `take_expect_within_event` — fires once per
/// silence regime (the first miss after data goes quiet) and counts firings in
/// a shared atomic.
#[cerulion_node]
#[derive(Default)]
struct ExpectWatchConsumer {
    #[input(trigger, expect_within_ms = 30)]
    inp: Vector3,
    last: f64,
    /// Shared with the test harness so it can observe handler firings.
    stale_fires: Arc<AtomicU64>,
}
#[cerulion_node_impl]
impl ExpectWatchConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last = self.inp.x;
        Ok(())
    }

    /// ExpectWithin handler — type-routed to `take_expect_within_event` via
    /// the `ExpectWithinEvent` param type.
    #[on_event(input = "inp")]
    fn on_stale(&mut self, event: ExpectWithinEvent) {
        // Foundation invariants pinned here so a regression in the event
        // payload surfaces in this declarative-handler test too.
        assert_eq!(&*event.input_name, "inp");
        assert_eq!(event.expect_within_ms, 30);
        assert!(event.elapsed_ms > event.expect_within_ms);
        self.stale_fires.fetch_add(1, Ordering::Relaxed);
    }
}

fn expect_graph(
    producer_type: &str,
    stale_fires: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "on_event_expect_test".to_string(),
        prefix: "oee".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "prod".to_string(),
                node_type: producer_type.to_string(),
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
                ros2: None,
                id: "cons".to_string(),
                node_type: "expect_watch_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "prod/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    let producer: Box<dyn NodeEntry> = match producer_type {
        "slow_producer" => Box::new(SlowProducerEntry::new()),
        "fast_producer" => Box::new(FastProducerEntry::new()),
        other => panic!("unknown producer type {other}"),
    };
    factories.insert("prod".to_string(), producer);
    let consumer = ExpectWatchConsumer {
        stale_fires,
        ..Default::default()
    };
    factories.insert(
        "cons".to_string(),
        Box::new(ExpectWatchConsumerEntry::with_state(consumer)),
    );
    (config, factories)
}

/// Run `steps` 5 ms steps; return the number of `ExpectWithinEvent` firings the
/// declarative handler observed.
fn run_expect_handler(producer_type: &str, steps: usize) -> u64 {
    run_expect_handler_full(producer_type, steps).0
}

/// Like [`run_expect_handler`] but also returns the per-window
/// `expect_within_missed_count` (the unconditional counter cadence) AND the
/// scheduler's per-fire [`TraceEntry`] timeline — for the strengthened
/// determinism test.
fn run_expect_handler_full(producer_type: &str, steps: usize) -> (u64, u64, Vec<TraceEntry>) {
    let stale_fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = expect_graph(producer_type, Arc::clone(&stale_fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build expect graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(5));
    }
    let missed = runtime
        .node_handle("cons")
        .unwrap()
        .expect_within_missed_count();
    let trace = runtime.trace().to_vec();
    (stale_fires.load(Ordering::Relaxed), missed, trace)
}

#[test]
fn on_event_expect_within_handler_fires_when_data_sparse() {
    // Slow producer (100 ms) vs a 30 ms window over 400 ms (80 × 5 ms): the
    // watchdog misses several windows between the sparse arrivals, and the
    // edge-triggered handler fires once per silence regime.
    let fires = run_expect_handler("slow_producer", 80);
    assert!(
        fires >= 1,
        "#[on_event] ExpectWithinEvent handler must fire when a 100 ms producer \
         starves a 30 ms expect window (got {fires})"
    );
}

#[test]
fn on_event_expect_within_handler_quiet_when_fast() {
    // Fast producer (5 ms) keeps the 30 ms window fresh → no miss, no event.
    let fires = run_expect_handler("fast_producer", 80);
    assert_eq!(
        fires, 0,
        "#[on_event] ExpectWithinEvent handler must stay quiet when a 5 ms \
         producer keeps the 30 ms window fresh (got {fires})"
    );
}

#[test]
fn on_event_expect_within_dispatch_is_deterministic() {
    // Principle #7: bit-identical fire count across two runs (the watchdog keys
    // off the wire timestamp + scheduler clock, never wall time).
    //
    // A bare `assert_eq!(a, b) + >= 1` cannot catch a latch/rearm
    // count-shift that changes the per-regime fire count IDENTICALLY across both
    // runs (both still match). Strengthen to the sibling `on_event_test.rs`
    // standard with TWO independent guards:
    //   (1) trace-timeline equality — the scheduler's full per-fire timeline,
    //       compared as `replay_test.rs` does (trace-level determinism); and
    //   (2) the edge-trigger ORACLE band `2 <= fires < missed` — the SAME
    //       invariant `expect_within_iox2_test::expect_within_event_drained_*`
    //       pins. `fires >= 2` kills the never-rearm collapse-to-1 regression;
    //       `fires < missed` kills the latch-ignored inflate-one-event-per-
    //       window regression (an unlatched handler would fire on EVERY one of
    //       the `missed` per-window counter bumps). This is a verified,
    //       non-magic oracle (no hand-computed fire count to drift).
    let (a, missed_a, trace_a) = run_expect_handler_full("slow_producer", 80);
    let (b, missed_b, trace_b) = run_expect_handler_full("slow_producer", 80);
    assert_eq!(
        a, b,
        "#[on_event] ExpectWithinEvent fire count must be deterministic across runs"
    );
    assert_eq!(
        trace_a, trace_b,
        "per-fire TraceEntry timeline must be bit-identical across runs \
         (Principle #7 — trace-level determinism, not just counter parity)"
    );
    assert_eq!(missed_a, missed_b, "miss counter deterministic across runs");
    assert!(!trace_a.is_empty(), "the graph actually fired");
    assert!(
        a >= 2,
        "multiple silence regimes must rearm the latch and fire again — \
         exactly 1 means the latch never rearmed (got {a})"
    );
    assert!(
        a < missed_a,
        "edge-trigger: strictly fewer events ({a}) than per-window miss counter \
         bumps ({missed_a}) — equal/greater means the latch was ignored and the \
         handler fired on every window"
    );
}

// ===========================================================================
// PromiseWithinEvent — OUTPUT-scoped `#[on_event(output = "...")]`
// ===========================================================================

/// Slow promise producer (100 ms) that declares a 30 ms publish promise on its
/// output and breaks it repeatedly. Its
/// `#[on_event(output = "out")] fn on_late(&mut self, ev: PromiseWithinEvent)`
/// handler — type-routed to `take_promise_within_event` — fires once per
/// silence regime and counts firings in a shared atomic.
#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct LatePromiseProducer {
    #[output(promise_within_ms = 30)]
    out: Vector3,
    n: u32,
    /// Shared with the test harness so it can observe handler firings.
    late_fires: Arc<AtomicU64>,
}
#[cerulion_node_impl]
impl LatePromiseProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }

    /// PromiseWithin handler — type-routed to `take_promise_within_event` via
    /// the `PromiseWithinEvent` param type.
    #[on_event(output = "out")]
    fn on_late(&mut self, event: PromiseWithinEvent) {
        assert_eq!(&*event.output_name, "out");
        assert_eq!(event.promise_within_ms, 30);
        assert!(event.elapsed_ms > event.promise_within_ms);
        self.late_fires.fetch_add(1, Ordering::Relaxed);
    }
}

/// Fast promise producer (5 ms) that keeps its 30 ms promise comfortably —
/// used to prove the quiet path.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct OnTimePromiseProducer {
    #[output(promise_within_ms = 30)]
    out: Vector3,
    n: u32,
    late_fires: Arc<AtomicU64>,
}
#[cerulion_node_impl]
impl OnTimePromiseProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }

    #[on_event(output = "out")]
    fn on_late(&mut self, _event: PromiseWithinEvent) {
        self.late_fires.fetch_add(1, Ordering::Relaxed);
    }
}

/// Minimal drain consumer so the produced topic has an in-graph consumer edge.
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

fn promise_graph(
    producer_type: &str,
    late_fires: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "on_event_promise_test".to_string(),
        prefix: "oep".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "prod".to_string(),
                node_type: producer_type.to_string(),
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
                ros2: None,
                id: "cons".to_string(),
                node_type: "plain_drain_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "prod/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    let producer: Box<dyn NodeEntry> = match producer_type {
        "late_promise_producer" => {
            let inner = LatePromiseProducer {
                late_fires,
                ..Default::default()
            };
            Box::new(LatePromiseProducerEntry::with_state(inner))
        }
        "on_time_promise_producer" => {
            let inner = OnTimePromiseProducer {
                late_fires,
                ..Default::default()
            };
            Box::new(OnTimePromiseProducerEntry::with_state(inner))
        }
        other => panic!("unknown producer type {other}"),
    };
    factories.insert("prod".to_string(), producer);
    factories.insert("cons".to_string(), Box::new(PlainDrainConsumerEntry::new()));
    (config, factories)
}

/// Run `steps` 5 ms steps; return the number of `PromiseWithinEvent` firings
/// the declarative handler observed.
fn run_promise_handler(producer_type: &str, steps: usize) -> u64 {
    run_promise_handler_full(producer_type, steps).0
}

/// Like [`run_promise_handler`] but also returns the per-window
/// `promise_within_missed_count` AND the scheduler's per-fire [`TraceEntry`]
/// timeline — for the strengthened determinism test.
fn run_promise_handler_full(producer_type: &str, steps: usize) -> (u64, u64, Vec<TraceEntry>) {
    let late_fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = promise_graph(producer_type, Arc::clone(&late_fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build promise graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(5));
    }
    let missed = runtime
        .node_handle("prod")
        .unwrap()
        .promise_within_missed_count();
    let trace = runtime.trace().to_vec();
    (late_fires.load(Ordering::Relaxed), missed, trace)
}

#[test]
fn on_event_promise_within_handler_fires_when_producer_late() {
    // 100 ms publish rate vs a 30 ms promise over 400 ms: the watchdog misses
    // between publishes and the edge-triggered handler fires per silence regime.
    let fires = run_promise_handler("late_promise_producer", 80);
    assert!(
        fires >= 1,
        "#[on_event] PromiseWithinEvent handler must fire when a 100 ms producer \
         breaks its 30 ms publish promise (got {fires})"
    );
}

#[test]
fn on_event_promise_within_handler_quiet_when_on_time() {
    // 5 ms publish rate vs a 30 ms promise → every window has a fresh publish;
    // no miss, no event.
    let fires = run_promise_handler("on_time_promise_producer", 80);
    assert_eq!(
        fires, 0,
        "#[on_event] PromiseWithinEvent handler must stay quiet when a 5 ms \
         producer keeps its 30 ms promise (got {fires})"
    );
}

#[test]
fn on_event_promise_within_dispatch_is_deterministic() {
    // Principle #7: bit-identical fire count across two runs.
    //
    // Strengthened to the sibling standard with the same two guards as
    // the ExpectWithin determinism test above — (1) trace-timeline equality and
    // (2) the edge-trigger oracle band `2 <= fires < missed` — so a latch/rearm
    // count-shift that is IDENTICAL across both runs (which the bare
    // `assert_eq!(a, b)` would miss) is caught.
    let (a, missed_a, trace_a) = run_promise_handler_full("late_promise_producer", 80);
    let (b, missed_b, trace_b) = run_promise_handler_full("late_promise_producer", 80);
    assert_eq!(
        a, b,
        "#[on_event] PromiseWithinEvent fire count must be deterministic across runs"
    );
    assert_eq!(
        trace_a, trace_b,
        "per-fire TraceEntry timeline must be bit-identical across runs \
         (Principle #7 — trace-level determinism, not just counter parity)"
    );
    assert_eq!(missed_a, missed_b, "miss counter deterministic across runs");
    assert!(!trace_a.is_empty(), "the graph actually fired");
    assert!(
        a >= 2,
        "multiple silence regimes must rearm the latch and fire again — \
         exactly 1 means the latch never rearmed (got {a})"
    );
    assert!(
        a < missed_a,
        "edge-trigger: strictly fewer events ({a}) than per-window miss counter \
         bumps ({missed_a}) — equal/greater means the latch was ignored and the \
         handler fired on every window"
    );
}
