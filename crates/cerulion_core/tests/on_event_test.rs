// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end `#[on_event(input = "...")]` macro for a
//! `BackpressureEvent` handler.
//!
//! Proves the declarative event-handler sugar over the shipped foundation
//! (`NodeContext::take_backpressure_event` + the `sample(N)` read-gate). A
//! `sample(N)` graph drives a consumer whose
//! `#[on_event(input = "inp")] fn on_inp_pressure(&mut self, ev: BackpressureEvent)`
//! handler — type-routed to the backpressure accessor — fires once per
//! decimation REGIME (edge-triggered by the foundation). We assert:
//!
//! 1. the handler fires >= 1 time (a decimation regime started), and
//! 2. the fire count is bit-identical across two runs (Principle #7 — the
//!    gate keys off the WIRE timestamp via the shared `VirtualClock`, not
//!    wall-clock, so it is replay-deterministic).
//!
//! The handler increments a shared `Arc<AtomicU64>` injected via the
//! macro-generated `with_state(inner)` constructor, so the count is observed
//! from real handler execution — no fake/mocked data (Principle #13).

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

/// Producer: publishes one Vector3 per 5 ms tick (wire timestamp = step
/// time via the shared VirtualClock), identical to the foundation
/// `sample(N)` e2e fixture.
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

/// Consumer: reads its input every 5 ms tick under a `sample(15)` gate. The
/// `#[on_event(input = "inp")]` handler counts decimation regimes by
/// incrementing a shared atomic injected through `with_state`.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct SamplingConsumer {
    #[input(backpressure = sample(15))]
    inp: Vector3,
    last_seen: f64,
    /// Shared with the test harness so it can observe handler firings.
    regimes: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl SamplingConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Reading `self.inp` drives `try_view`, which is where the sample
        // gate decimates and (on the first decimate of a regime) queues the
        // edge-triggered BackpressureEvent the handler below drains.
        self.last_seen = self.inp.x;
        Ok(())
    }

    /// Backpressure handler — fires once per decimation regime. Type-routed
    /// to `take_backpressure_event` via the `BackpressureEvent` param type.
    #[on_event(input = "inp")]
    fn on_inp_pressure(&mut self, event: BackpressureEvent) {
        // Sanity: the event must carry the right input name and a non-empty
        // regime (>= 1 drop). These are foundation invariants, pinned here
        // so a regression in the event payload surfaces in this test too.
        assert_eq!(&*event.input_name, "inp");
        assert!(event.count_in_regime >= 1);
        // For sample(N), `dropped` (messages lost)
        // equals the regime decimation count — pins the `dropped` field on the
        // sample path (drop_oldest + block are pinned in backpressure_event_*).
        assert!(matches!(event.policy, BackpressurePolicy::Sample(_)));
        assert_eq!(event.dropped, event.count_in_regime);
        // A mutation pin (kills the registration buffer_capacity → global
        // mutation on the sample arm): the event carries the input's REAL
        // queue — its default declared depth of 10.
        assert_eq!(
            event.buffer_capacity, 10,
            "sample events must carry the input's declared depth"
        );
        self.regimes.fetch_add(1, Ordering::Relaxed);
    }
}

/// Build the `sample(15)` graph; the consumer's handler increments
/// `regimes` (shared back to the caller).
fn sample_graph(regimes: Arc<AtomicU64>) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "on_bp_test".to_string(),
        prefix: "obp".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
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
                id: "consumer".to_string(),
                node_type: "sampling_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(FastProducerEntry::new()));
    // Inject the shared atomic via `with_state` so the handler's increments
    // are observable from the test after the run.
    let consumer = SamplingConsumer {
        regimes,
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(SamplingConsumerEntry::with_state(consumer)),
    );
    (config, factories)
}

/// Run for `steps` 5ms steps; return the number of decimation regimes the
/// handler observed.
fn run_handler_graph(steps: usize) -> u64 {
    run_handler_graph_with_trace(steps).0
}

/// Run for `steps` 5ms steps; return both the regime count AND the per-fire
/// [`TraceEntry`] timeline (Principle #7 is a
/// trace-level guarantee, so determinism must be verified at the trace
/// level, not just on aggregate counters).
fn run_handler_graph_with_trace(steps: usize) -> (u64, Vec<TraceEntry>) {
    let regimes = Arc::new(AtomicU64::new(0));
    let (config, factories) = sample_graph(Arc::clone(&regimes));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build handler graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(5));
    }
    let trace = runtime.trace().to_vec();
    (regimes.load(Ordering::Relaxed), trace)
}

#[test]
fn on_event_handler_fires_on_decimation_regime() {
    // Producer publishes every 5ms; sample(15) decimates ~2 of every 3
    // reads. The handler is edge-triggered: it fires once when a decimation
    // regime starts (first decimate after a successful accept). Over 12
    // steps at least one regime must start.
    let regimes = run_handler_graph(12);
    assert!(
        regimes >= 1,
        "#[on_event] handler must fire at least once when a sample(15) \
         decimation regime starts (got {regimes})"
    );
}

#[test]
fn on_event_dispatch_is_deterministic() {
    let (a, trace_a) = run_handler_graph_with_trace(20);
    let (b, trace_b) = run_handler_graph_with_trace(20);
    assert_eq!(
        a, b,
        "#[on_event] handler fire count must be bit-identical across runs \
         (edge-trigger keyed off the wire timestamp, not wall-clock — Principle #7)"
    );
    // Aggregate counters can match while the per-fire
    // timeline diverges. Principle #7 (Replay = Live) is a TRACE-level
    // guarantee, so compare the full `TraceEntry` timeline (node_id +
    // fire_time_ns per fire), as `replay_test.rs` does.
    assert_eq!(
        trace_a, trace_b,
        "per-fire TraceEntry timeline must be bit-identical across runs \
         (Principle #7 — trace-level determinism, not just counter parity)"
    );
    assert!(!trace_a.is_empty(), "the graph actually fired");
    assert!(a >= 1, "a decimation regime actually started");
}

/// Pin the exact regime count, not just `>= 1`. The
/// edge-trigger latch + rearm logic is deterministic for `period_ms = 5`
/// producer + `sample(15)` consumer over a fixed step count, so the regime
/// count is a stable oracle. A `>= 1` assertion would silently pass if a
/// regression (a) ignored the latch and fired one event per decimate (count
/// would inflate well past the regime count) or (b) never rearmed after an
/// accept (count would collapse to exactly 1). Pinning the exact value
/// catches both. The oracle below is the observed deterministic value at
/// 30 steps; if the gate timing is intentionally changed, update it.
#[test]
fn on_event_regime_count_is_exact() {
    // 30 steps * 5ms = 150ms of wire time. With a 5ms producer and a
    // sample(15) gate, the accept→decimate→decimate→accept cadence repeats
    // every 15ms, so the number of regimes is the count of distinct
    // decimation runs (each preceded by an accept). This value is
    // deterministic across runs (asserted separately above).
    let r1 = run_handler_graph(30);
    let r2 = run_handler_graph(30);
    assert_eq!(r1, r2, "regime count deterministic across runs");
    assert_eq!(
        r1, EXPECTED_REGIMES_30_STEPS,
        "exact decimation-regime count must match the pinned oracle — a change \
         means the edge-trigger latch/rearm behavior shifted (latch ignored = \
         inflated count; never-rearm = collapses to 1)"
    );
}

/// Observed deterministic regime count for `run_handler_graph(30)` (period=5,
/// sample(15)). Pinned as an oracle for the edge-trigger latch contract.
const EXPECTED_REGIMES_30_STEPS: u64 = 10;
