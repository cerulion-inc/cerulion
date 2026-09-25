// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end proof that `#[on_event]` (BackpressureEvent) fires
//! for the `drop_oldest` and `block` policies (not just `sample(N)`), over
//! real iceoryx2.
//!
//! - `drop_oldest`: a fast producer floods a slow consumer so iceoryx2
//!   evicts the oldest on overflow. The consumer's handler must fire (the
//!   subscriber counts evictions via per-publisher-stream wire-sequence
//!   gaps, keyed by `sample.origin()`, EXACT per publisher stream)
//!   AND `backpressure_drop_oldest_count` must be > 0.
//! - `block`: a block consumer whose queue reaches the defer threshold fires
//!   its handler with `dropped == 0` (lossless flow-control signal).
//! - multi-publisher topics + restarts (real `sample.origin()` ids from
//!   real second publishers): counting is EXACT PER PUBLISHER STREAM
//!   (per-id baselines) — interleaved evictions are attributed to
//!   the right stream, a restart's new port id simply
//!   baseline-establishes, and one stream's replay never blocks another
//!   stream's counting.
//!
//! All assert the handler fires from REAL execution (a shared atomic
//! incremented inside the handler — no fake data, Principle #13). The
//! graph-runtime suites also assert fire counts bit-identical across two
//! runs (Principle #7 — the drop_oldest detector keys off the wire
//! `sequence` + origin id, the block probe off the `outstanding` counter;
//! neither uses wall-clock). The anomaly suites instead assert order-robust
//! exact ORACLES (cross-publisher drain order is not contractual in
//! iceoryx2, so two-run bit-identity would promise more than the transport
//! does).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::error::TransportError;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{AnyPublisher, NodeContext, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::scheduler::BackpressureCounters;
use cerulion_core::testing::TestTransport;
use cerulion_core::testing::{
    count_at_exclusively, debug_level_compiled_in, debug_lines_expected, line_level,
};
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::subscriber::CerulionSubscriber;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use tracing_test::traced_test;

// ===========================================================================
// drop_oldest: fast producer floods a slow drop_oldest consumer → iceoryx2
// evicts the oldest → the consumer detects it via a sequence gap.
// ===========================================================================

/// Publishes one Vector3 per 1 ms tick — far faster than the consumer drains.
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

/// Drains its `drop_oldest` input every 12 ms — so ~12 samples accumulate per
/// drain cycle against the input's 10-deep real queue (its default declared
/// depth), forcing ~2 evictions each cycle. The handler counts eviction regimes; the harness also reads the
/// `drop_oldest_count` from the node handle. (The wildly-behind case — gap
/// larger than a buffer — is covered by the heavy-overload test below; since
/// the exactness fix it counts EXACTLY too.)
#[cerulion_node(period_ms = 12)]
#[derive(Default)]
struct SlowDropConsumer {
    #[input(backpressure = drop_oldest)]
    inp: Vector3,
    last_seen: f64,
    /// Shared with the harness — incremented inside the handler.
    regimes: Arc<AtomicU64>,
    /// Shared with the harness — set to the largest `event.dropped` observed.
    max_dropped: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl SlowDropConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Reading the input drives `try_view`, where the drop_oldest detector
        // runs and (on the first eviction of a regime) queues the event.
        self.last_seen = self.inp.x;
        Ok(())
    }

    #[on_event(input = "inp")]
    fn on_inp_pressure(&mut self, event: BackpressureEvent) {
        assert_eq!(&*event.input_name, "inp");
        assert!(
            matches!(event.policy, BackpressurePolicy::DropOldest),
            "drop_oldest input must surface a DropOldest event, got {:?}",
            event.policy
        );
        // drop_oldest IS data loss — `dropped` must be the evicted count (>=1).
        assert!(
            event.dropped >= 1,
            "an eviction event must report dropped >= 1"
        );
        assert_eq!(
            event.dropped, event.count_in_regime,
            "for drop_oldest, dropped == count_in_regime (both = evicted)"
        );
        self.regimes.fetch_add(1, Ordering::Relaxed);
        self.max_dropped.fetch_max(event.dropped, Ordering::Relaxed);
    }
}

fn drop_oldest_graph(
    regimes: Arc<AtomicU64>,
    max_dropped: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "bp_evt_drop".to_string(),
        prefix: "bped".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
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
                id: "consumer".to_string(),
                node_type: "slow_drop_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(FloodProducerEntry::new()));
    let consumer = SlowDropConsumer {
        regimes,
        max_dropped,
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(SlowDropConsumerEntry::with_state(consumer)),
    );
    (config, factories)
}

/// Run the drop_oldest graph for `steps` 1 ms steps; return
/// (handler regimes, max event.dropped, drop_oldest_count from the handle).
fn run_drop_oldest(steps: usize) -> (u64, u64, u64) {
    let regimes = Arc::new(AtomicU64::new(0));
    let max_dropped = Arc::new(AtomicU64::new(0));
    let (config, factories) = drop_oldest_graph(Arc::clone(&regimes), Arc::clone(&max_dropped));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build drop_oldest graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(1));
    }
    let count = runtime
        .node_handle("consumer")
        .unwrap()
        .backpressure_drop_oldest_count("inp");
    (
        regimes.load(Ordering::Relaxed),
        max_dropped.load(Ordering::Relaxed),
        count,
    )
}

#[test]
fn drop_oldest_event_fires_and_counter_is_real() {
    // 120 ms: ~120 producer publishes (1ms), ~10 consumer drains (12ms).
    // With the 10-deep input queue and ~12 samples/cycle, iceoryx2 evicts ~2
    // every cycle after the first — the handler must fire and the
    // drop_oldest_count must be non-zero.
    let (regimes, max_dropped, count) = run_drop_oldest(120);
    assert!(
        regimes >= 1,
        "the drop_oldest #[on_event] handler must fire when iceoryx2 \
         evicts (got {regimes} regimes)"
    );
    assert!(
        count > 0,
        "backpressure_drop_oldest_count must be REAL (> 0) now that evictions \
         are detected (got {count})"
    );
    assert!(
        max_dropped >= 1,
        "an eviction event must report a non-zero dropped count (got {max_dropped})"
    );
}

#[test]
fn drop_oldest_event_is_deterministic() {
    let a = run_drop_oldest(120);
    let b = run_drop_oldest(120);
    assert_eq!(
        a, b,
        "drop_oldest handler fires + dropped + counter must be bit-identical \
         across runs (detector keys off the wire `sequence`, not wall-clock — \
         Principle #7). a={a:?} b={b:?}"
    );
    assert!(a.0 >= 1, "an eviction regime actually started");
}

// ===========================================================================
// The drop_oldest DE-PHANTOM arm: a producer that DISCARDS every
// other tick (the macro Err class) feeding a drop_oldest consumer
// that KEEPS UP (queue never overflows) must produce ZERO evictions — no
// handler fires, drop_oldest_count == 0. A loan-time sequence
// stamp would make every discarded tick burn a number, so the committed frames
// would arrive with wire seqs 0,2,4,… and the gap detector would book a PHANTOM
// eviction on every single frame (the same phantom class bagd's
// frames_lost would inherit).
// ===========================================================================

/// Publishes every 4 ms but Errs (→ macro discard) on every even tick, so
/// committed frames land every ~8 ms — well within the consumer's pace.
#[cerulion_node(period_ms = 4)]
#[derive(Default)]
struct DiscardPaceProducer {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl DiscardPaceProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        if self.n.is_multiple_of(2) {
            return Err(NodeError::Logic("every other tick discards".to_string()));
        }
        Ok(())
    }
}

/// drop_oldest consumer that keeps pace (12 ms drain vs ~8 ms committed
/// cadence, depth 10 queue never overflows). Shares a seen-frames counter
/// (anti-tautology: data really flowed) and a handler-fires counter (must
/// stay 0 — any fire is a phantom eviction).
#[cerulion_node(period_ms = 12)]
#[derive(Default)]
struct GaplessDropConsumer {
    #[input(backpressure = drop_oldest)]
    inp: Vector3,
    seen: Arc<AtomicU64>,
    phantom_regimes: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl GaplessDropConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        if self.inp.x > 0.0 {
            self.seen.fetch_add(1, Ordering::Relaxed);
        }
        Ok(())
    }

    #[on_event(input = "inp")]
    fn on_inp_pressure(&mut self, event: BackpressureEvent) {
        // Reaching here at all is the bug: no eviction can have happened.
        let _ = event;
        self.phantom_regimes.fetch_add(1, Ordering::Relaxed);
    }
}

/// Run the discard-interleaved graph; return
/// (seen, phantom handler fires, drop_oldest_count).
fn run_discard_interleaved(steps: usize) -> (u64, u64, u64) {
    let seen = Arc::new(AtomicU64::new(0));
    let phantom = Arc::new(AtomicU64::new(0));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "bp_evt_dephantom".to_string(),
        prefix: "bpdp".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "discard_pace_producer".to_string(),
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
                node_type: "gapless_drop_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(DiscardPaceProducerEntry::new()),
    );
    let consumer = GaplessDropConsumer {
        seen: Arc::clone(&seen),
        phantom_regimes: Arc::clone(&phantom),
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(GaplessDropConsumerEntry::with_state(consumer)),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build discard-interleaved graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(1));
    }
    let count = runtime
        .node_handle("consumer")
        .unwrap()
        .backpressure_drop_oldest_count("inp");
    (
        seen.load(Ordering::Relaxed),
        phantom.load(Ordering::Relaxed),
        count,
    )
}

#[test]
fn discard_interleaved_producer_causes_no_phantom_evictions() {
    let (seen, phantom, count) = run_discard_interleaved(120);
    // Anti-tautology: committed frames really flowed to the consumer.
    assert!(
        seen >= 5,
        "the consumer must have observed committed frames (got {seen})"
    );
    // The de-phantom pins: nothing was evicted, so the detector must be
    // silent. With a loan-time stamp every frame would arrive with a wire-seq
    // gap → phantom count > 0 and phantom handler fires.
    assert_eq!(
        count, 0,
        "discard-interleaved traffic with a keeping-up consumer must book \
         ZERO drop_oldest evictions — a nonzero count is the commit-time-sequence \
         phantom-gap bug"
    );
    assert_eq!(
        phantom, 0,
        "no BackpressureEvent may fire without a real eviction (got {phantom})"
    );
}

#[test]
fn discard_interleaved_run_is_deterministic() {
    let a = run_discard_interleaved(120);
    let b = run_discard_interleaved(120);
    assert_eq!(
        a, b,
        "discard-interleaved runs must be bit-identical (Principle #7)"
    );
}

// ===========================================================================
// block: a slow block consumer whose queue reaches the defer threshold fires
// its handler with dropped == 0 (lossless).
// ===========================================================================

/// Publishes one Vector3 per 5 ms tick into a block consumer.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct BlockProducer {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl BlockProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Block consumer (depth 2) draining every 20 ms — so its queue reaches the
/// defer threshold between drains. The handler must fire with `dropped == 0`.
#[cerulion_node(period_ms = 20)]
#[derive(Default)]
struct BlockHandlerConsumer {
    #[input(backpressure = block, depth = 2)]
    inp: Vector3,
    last_seen: f64,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl BlockHandlerConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last_seen = self.inp.x;
        Ok(())
    }

    #[on_event(input = "inp")]
    fn on_inp_pressure(&mut self, event: BackpressureEvent) {
        assert_eq!(&*event.input_name, "inp");
        assert!(
            matches!(event.policy, BackpressurePolicy::Block),
            "block input must surface a Block event, got {:?}",
            event.policy
        );
        // block loses NOTHING — the event is a flow-control signal.
        assert_eq!(event.dropped, 0, "a block event must report dropped == 0");
        // The block event's regime stamp is the drain's high-water wire timestamp,
        // captured via the SAME per-stream scratch the drop_oldest probe
        // uses — a regression that stops filling the scratch for Block
        // (e.g. dropping Block from try_view's `probing` predicate) reads 0
        // here. Every frame in this graph carries a real wire timestamp
        // > 0 (the regime opens well past VirtualClock t=0).
        assert_ne!(
            event.regime_started_at_ns, 0,
            "a block event's regime timestamp must be the drain's high-water \
             wire timestamp, not 0"
        );
        // Block regime_count is always 1
        // (written only at regime open); count_total READS the
        // producer-maintained defer counter, which is >= 1 by the time the
        // consumer's first at-threshold drain fires (the producer was
        // deferred at t=15ms, before the consumer's first 20ms drain); the
        // buffer_capacity carries the input's REAL iceoryx2 queue — the
        // declared `depth = 2` (depth IS the buffer; the
        // global build_for_test(.., 8) default does not apply to a
        // depth-declaring input).
        assert_eq!(event.count_in_regime, 1, "block regime_count is always 1");
        assert!(
            event.count_total >= 1,
            "count_total must read the producer's defer counter (got {})",
            event.count_total
        );
        assert_eq!(
            event.buffer_capacity, 2,
            "buffer_capacity must carry the input's declared depth (its real queue)"
        );
        self.fires.fetch_add(1, Ordering::Relaxed);
    }
}

fn block_event_graph(fires: Arc<AtomicU64>) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "bp_evt_block".to_string(),
        prefix: "bpeb".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "block_producer".to_string(),
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
                node_type: "block_handler_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(BlockProducerEntry::new()));
    let consumer = BlockHandlerConsumer {
        fires,
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(BlockHandlerConsumerEntry::with_state(consumer)),
    );
    (config, factories)
}

/// One run of the block-event graph, as the three counters a test compares.
#[derive(Debug, PartialEq, Eq)]
struct BlockRun {
    /// Consumer handler fires.
    fires: u64,
    /// Producer-side `block_fires_deferred_count` on the consumer's input.
    defers: u64,
    /// Producer-side `block_defer_regimes_count` — regimes the edge opened.
    regimes: u64,
}

/// Run the block-event graph for `steps` steps.
fn run_block_event(steps: usize) -> BlockRun {
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = block_event_graph(Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build block-event graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(5));
    }
    let handle = runtime.node_handle("consumer").unwrap();
    // Reciprocal dormancy pins (mirroring the sample
    // suite): a block input's other two policy counters must stay silent —
    // type-enforced by the single `BackpressureProbe` slot, pinned
    // behaviorally here for every caller of this helper.
    assert_eq!(
        handle.backpressure_sampled_count("inp"),
        0,
        "a block input must never bump sampled_count"
    );
    assert_eq!(
        handle.backpressure_drop_oldest_count("inp"),
        0,
        "a block input must never bump drop_oldest_count (block is lossless)"
    );
    let defers = handle.backpressure_block_fires_deferred_count("inp");
    let regimes = handle.backpressure_block_defer_regimes_count("inp");
    // Every regime opening is also a deferred step (the documented relation,
    // pinned on every run of this helper, not only in the traced arm).
    assert!(
        regimes <= defers,
        "regimes ({regimes}) opened without a deferred step (defers {defers})"
    );
    BlockRun {
        fires: fires.load(Ordering::Relaxed),
        defers,
        regimes,
    }
}

// ===========================================================================
// This section pins the block probe's rearm branch
// (`armed = true` on a below-threshold drain). The threshold graph above
// never drains below threshold (every drain sees pre == 2), so deleting the
// rearm would still pass it with exactly one fire. This graph paces
// producer (10 ms) vs consumer (15 ms) so drains ALTERNATE below-threshold
// (rearm) and at-threshold (fire) — without the rearm, fires stays 1.
// ===========================================================================

/// Publishes every 10 ms — paced against the 15 ms consumer so its queue
/// oscillates around the depth-2 defer line.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct RearmProducer {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl RearmProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Block consumer (depth 2) draining every 15 ms: sees pre-drain depths
/// 1, 2, 1, 2, ... — each below-threshold drain must rearm the edge
/// trigger so the next at-threshold drain fires again.
#[cerulion_node(period_ms = 15)]
#[derive(Default)]
struct RearmBlockConsumer {
    #[input(backpressure = block, depth = 2)]
    inp: Vector3,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl RearmBlockConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }

    #[on_event(input = "inp")]
    fn on_inp_pressure(&mut self, event: BackpressureEvent) {
        assert!(
            matches!(event.policy, BackpressurePolicy::Block),
            "block input must surface a Block event, got {:?}",
            event.policy
        );
        assert_eq!(event.dropped, 0, "a block event must report dropped == 0");
        // Deliberate asymmetry with BlockHandlerConsumer: in THIS
        // graph the producer is never deferred — outstanding only reaches
        // the threshold transiently within the same step the consumer
        // drains it, so the pre-fire always reads <= 1. count_total == 0
        // pins that block events are CONSUMER-queue-keyed, not defer-keyed
        // (and a `count_total >= 1` copy-paste here would fail).
        assert_eq!(
            event.count_total, 0,
            "block events fire from the consumer-queue threshold even with \
             zero producer defers (got {})",
            event.count_total
        );
        self.fires.fetch_add(1, Ordering::Relaxed);
    }
}

#[test]
fn block_event_rearms_after_below_threshold_drain() {
    let fires = Arc::new(AtomicU64::new(0));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "bp_evt_block_rearm".to_string(),
        prefix: "bpebr".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "rearm_producer".to_string(),
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
                node_type: "rearm_block_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(RearmProducerEntry::new()));
    let consumer = RearmBlockConsumer {
        fires: Arc::clone(&fires),
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(RearmBlockConsumerEntry::with_state(consumer)),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build rearm graph");
    for _ in 0..60 {
        runtime.step(Duration::from_millis(5));
    }
    let fired = fires.load(Ordering::Relaxed);
    assert!(
        fired >= 2,
        "below-threshold drains must REARM the block edge-trigger so later \
         at-threshold drains fire again (got {fired} fires — exactly 1 means \
         the rearm branch is dead)"
    );
}

#[test]
fn block_event_fires_at_threshold() {
    // Producer (5ms) outpaces the consumer (20ms) into a depth-2 block input,
    // so the consumer's queue reaches the defer threshold between drains; its
    // handler must fire (dropped == 0, asserted in the handler).
    let BlockRun { fires, defers, .. } = run_block_event(60);
    // EXACTLY one fire: this graph never
    // drains below threshold — every drain sees pre == 2, so the
    // edge-trigger never rearms after the first fire. Exactly-1 is the
    // once-per-regime pin: a level-trigger regression (`fire = true` /
    // dropping the disarm) fires on EVERY at-threshold drain (~15 here)
    // and would slip past a `>= 1` bound. Deterministic under
    // VirtualClock (block_event_is_deterministic pins the stability).
    assert_eq!(
        fires, 1,
        "the block #[on_event] handler must fire EXACTLY once per \
         regime — this graph's regime never closes (got {fires})"
    );
    // Tie the consumer event to the real block
    // mechanism — the producer must actually have been deferred. (A regression
    // that broke the producer pre-fire defer while leaving the consumer
    // threshold-event intact would slip past a `fires >= 1`-only check.)
    assert!(
        defers >= 1,
        "the producer must actually have been deferred (block_fires_deferred_count \
         > 0), not just the consumer event fired (got {defers})"
    );
}

#[test]
fn block_event_is_deterministic() {
    let a = run_block_event(60);
    let b = run_block_event(60);
    assert_eq!(
        a, b,
        "block handler fires + producer defers must be bit-identical across runs \
         (the probe keys off the outstanding counter, not wall-clock — Principle \
         #7). a={a:?} b={b:?}"
    );
    assert!(a.fires >= 1, "the block handler actually fired");
    assert!(a.defers >= 1, "the producer was actually deferred");
}

/// The producer-side `block` defer WARN rides a once-per-regime
/// edge (`BlockDeferEdge::armed`, rearmed by a below-threshold observation —
/// the SAME rearm rule as the consumer-side block `BackpressureEvent`), so a
/// consumer sitting at its buffer threshold — a LEGITIMATE steady state for
/// designed lossless backpressure (USER_API.md, "Backpressure") — does not
/// flood the log with one warn per deferred step. The `block_fires_deferred_count`
/// still counts EVERY deferred step (Principle #3: truth is the counter). This is
/// the WARN-cadence companion to `block_event_fires_at_threshold` (which pins the
/// consumer EVENT cadence). This graph's producer re-arms after each 20ms drain
/// (drain-to-latest pops all queued → `outstanding` returns toward 0), so
/// multiple regimes open across the run — pinning both "1 warn per regime open"
/// AND "re-arm ⇒ a later regime warns loud again".
#[traced_test]
#[test]
fn block_defer_warn_is_once_per_regime_not_per_step() {
    let BlockRun {
        fires,
        defers,
        regimes,
    } = run_block_event(60);
    assert!(fires >= 1, "the consumer block event fired");
    assert!(
        defers >= 2,
        "the producer was deferred across multiple steps (got {defers})"
    );

    logs_assert(|lines: &[&str]| {
        // Level-free twin: a sustained block defer must never be LOUD — the half of the
        // contract that survives `release_max_level_info`, where the gated
        // DEBUG count reads 0.
        for level in ["WARN", "INFO", "ERROR"] {
            let loud = lines
                .iter()
                .filter(|l| {
                    line_level(l) == Some(level) && (l.contains("backpressure event (sustained"))
                })
                .count();
            if loud != 0 {
                return Err(format!(
                    "a sustained block defer was emitted at {level} ({loud} line(s))"
                ));
            }
        }
        // The loud, regime-opening warn (substring unique to the block WARN arm;
        // the debug arm opens with "backpressure event (sustained ..."), matched
        // WITH its level token AND against the level-free total of that marker:
        // the level IS the contract — a head demoted to INFO/ERROR would
        // otherwise still equal `regimes` — and a second copy of it at another
        // level is not the one head either.
        let warns = count_at_exclusively(
            lines,
            "WARN",
            &["backpressure event: producer's tick deferred"],
        )?;
        // The sustained defer demoted to debug (substring unique to the block
        // DEBUG arm — drop_oldest's sustained says "iceoryx2 evicted", sample's
        // says "dropped message").
        let sustained = count_at_exclusively(
            lines,
            "DEBUG",
            &["backpressure event (sustained", "producer's tick deferred"],
        )?;
        // (1) Every deferred step logs EXACTLY one line (warn or debug) and bumps
        // the counter once: warn + debug == block_fires_deferred_count. A
        // per-step-warn regression (dropping the `armed` gate) makes warns ==
        // defers and sustained == 0.
        // Release-observable and EXACT: the edge counts every regime it opens at
        // the state transition, so the loud regime-opening warns must match it
        // one-for-one — an edge that warns on only the first regimes fails here
        // in release, where the sustained `debug!` count below reads 0.
        if warns != regimes as usize {
            return Err(format!(
                "every regime opening must warn loud exactly once: the edge opened {regimes} \
                 regime(s), got {warns} regime-opening warn(s)"
            ));
        }
        let want_sustained = debug_lines_expected((defers as usize).saturating_sub(warns));
        if sustained != want_sustained {
            return Err(format!(
                "every deferred step must log exactly one line and bump the counter: \
                 expected {want_sustained} sustained debug lines beside {warns} warns for \
                 {defers} defers (0 where `debug!` is compiled out), got {sustained}"
            ));
        }
        // (2) At least one sustained defer was demoted to debug — the flood IS
        // suppressed (the whole point of the once-per-regime gate).
        if debug_level_compiled_in() && sustained < 1 {
            return Err(format!(
                "expected >= 1 sustained defer demoted to debug (flood suppressed), \
                 got {sustained}"
            ));
        }
        // (3) RE-ARM: the producer's queue drains below threshold between the
        // consumer's periodic drains, re-arming the edge so a LATER regime warns
        // loud again — more than one regime opened. A dead rearm branch caps
        // warns at 1.
        if warns < 2 {
            return Err(format!(
                "below-threshold re-arm must let later regimes warn loud again: \
                 expected >= 2 regime-open warns, got {warns}"
            ));
        }
        // (4) And the headline: fewer warns than deferred steps — the warn does
        // NOT fire per deferred step.
        if warns >= defers as usize {
            return Err(format!(
                "the warn must NOT fire per deferred step: warns({warns}) >= defers({defers})"
            ));
        }
        Ok(())
    });
}

// ===========================================================================
// drop_oldest HEAVY OVERLOAD (eviction-count exactness): a consumer lagging by
// MORE than a buffer per drain would SATURATE an epoch-blind seq-gap
// detector (loud lower-bound 0 — it cannot tell heavy eviction from a
// publisher restart). With `sample.origin()` pinning both sides of the gap
// to one publisher epoch, the same-id gap IS the true loss: the counter must
// be EXACT and large (well past a one-buffer cap) and the handler
// must fire.
// ===========================================================================

/// Drains every 40 ms while the producer publishes every 1 ms into its 10-deep
/// buffer → ~40 samples accumulate per cycle, ~32 evicted (≫ buffer) every
/// cycle. Since the exactness fix the detector counts this exactly; the handler
/// fires on the regime onset, and `event.dropped` carries a > one-buffer
/// count.
#[cerulion_node(period_ms = 40)]
#[derive(Default)]
struct OverloadedDropConsumer {
    #[input(backpressure = drop_oldest)]
    inp: Vector3,
    last_seen: f64,
    regimes: Arc<AtomicU64>,
    /// Largest `event.dropped` observed — must exceed one buffer's worth
    /// (the pre-epoch detector could never report more than the buffer).
    max_dropped: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl OverloadedDropConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last_seen = self.inp.x;
        Ok(())
    }

    #[on_event(input = "inp")]
    fn on_inp_pressure(&mut self, event: BackpressureEvent) {
        // Kills the registration buffer_capacity → global
        // mutation on the drop_oldest arm: the event carries the input's
        // REAL queue — its default declared depth of 10, not the
        // build_for_test global of 8.
        assert_eq!(
            event.buffer_capacity, 10,
            "drop_oldest events must carry the input's declared depth"
        );
        self.regimes.fetch_add(1, Ordering::Relaxed);
        self.max_dropped.fetch_max(event.dropped, Ordering::Relaxed);
    }
}

fn overloaded_graph(
    regimes: Arc<AtomicU64>,
    max_dropped: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "bp_evt_sat".to_string(),
        prefix: "bpes".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
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
                id: "consumer".to_string(),
                node_type: "overloaded_drop_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(FloodProducerEntry::new()));
    let consumer = OverloadedDropConsumer {
        regimes,
        max_dropped,
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(OverloadedDropConsumerEntry::with_state(consumer)),
    );
    (config, factories)
}

/// Run the heavy-overload graph for `steps` 1 ms steps; return
/// (handler regimes, max event.dropped, drop_oldest_count from the handle).
fn run_overloaded(steps: usize) -> (u64, u64, u64) {
    let regimes = Arc::new(AtomicU64::new(0));
    let max_dropped = Arc::new(AtomicU64::new(0));
    let (config, factories) = overloaded_graph(Arc::clone(&regimes), Arc::clone(&max_dropped));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build overloaded graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(1));
    }
    let count = runtime
        .node_handle("consumer")
        .unwrap()
        .backpressure_drop_oldest_count("inp");
    (
        regimes.load(Ordering::Relaxed),
        max_dropped.load(Ordering::Relaxed),
        count,
    )
}

/// Hand-pinned oracle for `run_overloaded(160)` — derived analytically and
/// confirmed against a live run (NOT computed by re-running the system
/// under test; see `feedback_pin_docs_with_oracle_tests`). Derivation: the
/// 40 ms consumer drains 4× in 160 steps; each inter-drain window
/// accumulates ~40 publishes against the input's REAL queue — its default
/// declared depth of 10 (depth IS the buffer; the
/// build_for_test global of 8 does not size graph inputs) → 30 evicted
/// per window after the first; the first window's loss lands before a
/// baseline exists and the run ends mid-window, leaving 3 fully-counted
/// windows × 30 = 90. All post-baseline drains evict, so the regime never
/// closes: exactly 1 handler fire whose `dropped` is the opening window's
/// 30.
const OVERLOADED_160_ORACLE: (u64, u64, u64) = (1, 30, 90);

#[test]
fn drop_oldest_heavy_overload_counts_exactly_no_cap() {
    // ~40 samples accumulate per 40 ms drain cycle against the input's
    // 10-deep real queue (default depth) → ~30 evictions per cycle, i.e. a
    // per-drain gap of ~3× the queue. The pre-epoch detector SATURATED here (counter pinned at 0);
    // with `sample.origin()` the same-id gap is the true loss. The exact
    // oracle (not a floor) catches systematic off-by-ones in the gap
    // arithmetic that a two-run self-comparison would shift identically.
    let triple = run_overloaded(160);
    assert_eq!(
        triple, OVERLOADED_160_ORACLE,
        "(regimes, max event.dropped, drop_oldest_count) must match the \
         hand-pinned oracle — both a fabrication (too high) and a residual \
         cap/saturation (too low) shift it"
    );
    // Redundant readability floors (subsumed by the oracle): the count and
    // a single event's `dropped` both exceed one buffer's worth (8) — the
    // pre-epoch detector could never report either.
    assert!(triple.2 > 16 && triple.1 > 8);
}

#[test]
fn drop_oldest_heavy_overload_is_deterministic() {
    let a = run_overloaded(160);
    let b = run_overloaded(160);
    assert_eq!(
        a, b,
        "heavy-overload exact counts must be bit-identical across runs \
         (detector keys off wire sequence + origin id, not wall-clock — \
         Principle #7). a={a:?} b={b:?}"
    );
    assert!(a.2 > 16, "the exact count actually accumulated");
}

// ===========================================================================
// Multi-publisher streams: restart hand-over and
// multi-publisher interleave, with REAL second publishers (real
// `sample.origin()` ids — no fake data, Principle #13). Per-id baselines
// make these ordinary: every publisher stream counts EXACTLY against its
// own baseline; a restart's new port id simply baseline-establishes
// (uncounted — prior history unknowable); there is no suspension and no
// tripwire warn. `GraphRuntime` wiring deliberately cannot express these
// topologies (topology validation rejects double-producer topics), so the
// probe is installed via the test-only registration hook on a raw
// `TestTransport` subscriber.
// ===========================================================================

/// Minimal zero-copy Vector3 source — ticked manually so each tick publishes
/// exactly one frame from whichever publisher its context wraps (each
/// publisher carries its own `UniquePublisherId` + wire-sequence counter).
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct EpochSource {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl EpochSource {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Wrap a raw publisher in an initialized [`EpochSource`] entry.
fn epoch_source(pubr: CerulionPublisher) -> EpochSourceEntry {
    let mut pubs: IndexMap<String, AnyPublisher> = IndexMap::new();
    pubs.insert("out".to_string(), AnyPublisher::Ipc(pubr));
    let mut entry = EpochSourceEntry::new();
    entry
        .init(NodeContext::for_tests(pubs, IndexMap::new()))
        .expect("init epoch source");
    entry
}

fn tick_n(entry: &mut EpochSourceEntry, n: usize) {
    for _ in 0..n {
        entry.tick().expect("tick");
    }
}

/// One probe "drain": a single `try_view` call (it drains the whole iceoryx2
/// queue, latest-wins). Returns whether any sample was present.
fn drain(sub: &mut CerulionSubscriber) -> bool {
    sub.try_view::<Vector3, _>(|_v| {})
        .expect("try_view")
        .is_some()
}

/// Build the two-publisher harness: a probe-equipped subscriber plus two
/// independent publishers on the SAME topic.
fn epoch_harness(
    topic: &str,
) -> (
    EpochSourceEntry,
    EpochSourceEntry,
    CerulionSubscriber,
    Arc<BackpressureCounters>,
) {
    let tt = TestTransport::with_buffer_size(8);
    let pub_a = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let pub_b = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );
    (epoch_source(pub_a), epoch_source(pub_b), sub, counters)
}

#[traced_test]
#[test]
fn drop_oldest_publisher_restart_baselines_new_stream_then_counts_exact() {
    // A publisher restart is not an "anomaly" that
    // suspends counting — the new port id is simply a new stream that
    // baseline-establishes (its pre-observation history is unknowable) and
    // then counts exactly. No suspension warn exists.
    let (mut a, mut b, mut sub, counters) = epoch_harness("bped_epoch/out");

    // Baseline on publisher A (first drain counts nothing).
    tick_n(&mut a, 3);
    assert!(drain(&mut sub), "A's samples arrived");
    assert_eq!(counters.drop_oldest_count.load(Ordering::Relaxed), 0);
    assert!(sub.try_take_backpressure_event().is_none());

    // Same-stream eviction is EXACT: 10 publishes into the 8-deep queue →
    // 2 evicted, and the wire-seq gap pinned to A's stream counts exactly 2.
    tick_n(&mut a, 10);
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        2,
        "same-stream gap must count exactly (10 publishes, 8-deep queue)"
    );
    let ev = sub.try_take_backpressure_event().expect("eviction event");
    assert_eq!(ev.dropped, 2, "the event carries the exact evicted count");

    // "Restart": B — a NEW publisher port, hence a NEW UniquePublisherId —
    // takes over the topic. The new stream baseline-establishes, uncounted
    // and WITHOUT any suspension warn (per-id baselines make a restart routine).
    tick_n(&mut b, 1);
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        2,
        "a new stream's first observation must not fabricate evictions"
    );
    assert!(
        sub.try_take_backpressure_event().is_none(),
        "a baseline-establishing drain queues no BackpressureEvent"
    );
    assert!(
        !logs_contain("eviction counting suspended"),
        "restarts are baseline-establishment, not a suspension \
         anomaly — no suspension warn exists"
    );

    // Stable on B: a contiguous drain counts nothing…
    tick_n(&mut b, 2);
    assert!(drain(&mut sub));
    assert_eq!(counters.drop_oldest_count.load(Ordering::Relaxed), 2);

    // …and exact counting continues on B's stream: 12 publishes → 4 evicted.
    tick_n(&mut b, 12);
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        6,
        "exact counting must continue on the new publisher stream (2 + 4)"
    );
    assert_eq!(
        sub.try_take_backpressure_event().expect("event").dropped,
        4,
        "the new regime's event carries the exact post-restart count"
    );
}

#[test]
fn drop_oldest_mixed_publishers_count_exactly_per_id() {
    // TWO live publishers interleaving on one topic
    // count EXACTLY per origin id (counting is never suspended here). Uses
    // the macro-node harness — real evictions from real queue overflow.
    let (mut a, mut b, mut sub, counters) = epoch_harness("bped_mixed/out");

    // Mixed drain: both streams baseline-establish (first sighting each).
    tick_n(&mut a, 2);
    tick_n(&mut b, 2);
    tick_n(&mut a, 1);
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        0,
        "first sighting of each stream baseline-establishes, uncounted"
    );
    assert!(sub.try_take_backpressure_event().is_none());

    // A alone publishes contiguously — clean, still nothing to count.
    tick_n(&mut a, 2);
    assert!(drain(&mut sub));
    assert_eq!(counters.drop_oldest_count.load(Ordering::Relaxed), 0);

    // A floods: 12 publishes into the 8-deep queue → exactly 4 of A's
    // frames evicted; B's baseline is untouched.
    tick_n(&mut a, 12);
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        4,
        "A's stream must count exactly while B's stream is quiet"
    );
    assert_eq!(sub.try_take_backpressure_event().expect("event").dropped, 4);

    // B floods next: per-id baselines mean B's evictions count exactly
    // too, independent of everything A did in between.
    tick_n(&mut b, 12);
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        8,
        "B's stream must count exactly against ITS baseline (4 + 4)"
    );
}

// ===========================================================================
// Regression suite:
//  - history replay (same-id BACKWARD sequences) must never be counted —
//    a naive detector would fabricate a ~2^32 count when a late joiner
//    attached to a history-enabled topic;
//  - corrupt (undersized) frames reset ALL baselines instead of
//    fabricating; errored drains likewise (when they consumed frames);
//  - a baseline-establishing (restart) drain REARMS the event trigger
//    via the clean-drain rule;
//  - sustained re-delivery warns every ANOMALY_REWARN_EVERY = 64 backward
//    drains (no onset warn — single replay drains are routine);
//  - per-id baselines make multi-publisher counting exact;
//    capacity eviction (streams beyond the topic's max_publishers — the
//    iceoryx2 DEFAULT is 2, so eviction fires from the 3rd distinct
//    stream) keeps exactness, and spares streams seen in the drain.
// ===========================================================================

/// Hand-stamped Vector3 wire frame (header + zeroed fixed payload) so tests
/// control the wire sequence precisely, re-published through the raw-FFI
/// `publish_raw` path.
fn vector3_frame(seq: u32) -> Vec<u8> {
    use cerulion_core::message::ShmMessage;
    use cerulion_core::wire::WireHeader;
    let payload_len = <Vector3 as ShmMessage>::WIRE_FIXED_SIZE;
    let header = WireHeader::new(
        <Vector3 as ShmMessage>::SCHEMA_HASH,
        seq,
        u64::from(seq) * 1_000_000,
    );
    let mut buf = vec![0u8; WireHeader::SIZE + payload_len];
    header.write_to_buf(&mut buf[..WireHeader::SIZE]);
    let total = (WireHeader::SIZE + payload_len) as u32;
    buf[8..12].copy_from_slice(&total.to_le_bytes());
    buf
}

/// Drain a queue whose latest-wins frame is EXPECTED to be corrupt: the
/// probe's state updates happen during the drain, BEFORE the latest-wins
/// frame is validated, and the corrupt frame must surface as the loud
/// `Deserialization` error — any OTHER outcome (unexpected `Receive`,
/// `SchemaMismatch`, or silent success) fails the test instead of
/// vacuously passing it.
fn drain_expect_corrupt(sub: &mut CerulionSubscriber) {
    let result = sub.try_view::<Vector3, _>(|_v| {});
    assert!(
        matches!(result, Err(TransportError::Deserialization { .. })),
        "expected the corrupt latest-wins frame to surface as a \
         Deserialization error, got {result:?}"
    );
}

#[test]
fn native_history_delivery_to_late_joiner_not_counted_as_eviction() {
    // Native iceoryx2 history delivers a LATE joiner the
    // retained frames with their ORIGINAL stale sequences (by SHM offset,
    // oldest-first, into the joiner's OWN data queue — NOT re-published to
    // already-connected subscribers). The joiner's drop_oldest probe must
    // baseline-establish on its first drain and NOT fabricate an eviction
    // count from those retained-then-live sequences. (A heap
    // `deliver_history` that re-published stale sequences through `publish_raw` to
    // EVERY connected subscriber would exercise the `classify_gap`
    // Backward path; native ordered delivery removes that re-delivery, and
    // the Backward path stays as the defensive duplicate-re-delivery guard.)
    let topic = "bped_replay/out";
    let tt = TestTransport::with_buffer_size(8);
    // History-enabled publisher: a late joiner gets the last 4 frames
    // delivered natively.
    let pub_a = tt.publisher(topic, MaxSliceLen::const_new(256), 4);
    // An already-connected subscriber so the publisher has live connections
    // before the late joiner attaches.
    let mut early = tt.subscriber(topic);
    let mut a = epoch_source(pub_a);

    tick_n(&mut a, 5); // 5 publishes; the publisher's native history retains the last 4
    assert!(drain(&mut early), "early subscriber sees A's live samples");

    // A LATE JOINER attaches with its OWN drop_oldest probe. iceoryx2 native
    // history delivers it the last 4 retained frames (oldest-first) on the
    // publisher's next tick (which pumps SubscriberConnected →
    // update_connections + the SentHistory wake).
    let mut late = tt.subscriber(topic);
    let counters = Arc::new(BackpressureCounters::new());
    late.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );

    tick_n(&mut a, 1); // pump native delivery to `late` + 1 live frame
    assert!(drain(&mut late), "late joiner drains its native history");
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        0,
        "native history (retained sequences delivered oldest-first into the \
         late joiner's queue) must NOT be counted as evictions on its first, \
         baseline-establishing drain"
    );
    assert!(
        late.try_take_backpressure_event().is_none(),
        "a baseline drain over native history queues no BackpressureEvent"
    );

    // The late joiner's baseline advanced to the live tail, so exact same-
    // stream counting continues: 12 publishes into its 8-deep queue → 4
    // evicted.
    tick_n(&mut a, 12);
    assert!(drain(&mut late));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        4,
        "exact counting must continue seamlessly after native history delivery"
    );
}

#[traced_test]
#[test]
fn drop_oldest_corrupt_frame_resets_baseline_never_fabricates() {
    let topic = "bped_corrupt/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut pubr = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );

    for seq in [1u32, 2, 3] {
        pubr.publish_raw(&vector3_frame(seq)).expect("publish");
    }
    assert!(drain(&mut sub)); // baseline (P, 3)
    assert_eq!(counters.drop_oldest_count.load(Ordering::Relaxed), 0);

    // A valid frame followed by an UNDERSIZED one (occupies an unknowable
    // sequence slot). The corrupt drain must not count anything and must
    // RESET the baseline — advancing it to seq 4 would let the next drain
    // count the corrupt frame's unknowable slot as an eviction.
    pubr.publish_raw(&vector3_frame(4)).expect("publish");
    pubr.publish_raw(&[0u8; 16]).expect("publish undersized");
    drain_expect_corrupt(&mut sub);
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        0,
        "a corrupt drain must not count"
    );
    assert!(sub.try_take_backpressure_event().is_none());

    // This drain re-establishes the baseline (it was reset to None); the
    // gap between the corrupt drain's frames and this one is deliberately
    // uncountable — asserting 0 here catches a baseline that advances
    // instead of resetting (which would report Exact(1)).
    pubr.publish_raw(&vector3_frame(6)).expect("publish");
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        0,
        "the drain after a corrupt drain re-establishes the baseline and \
         must not fabricate an eviction from the corrupt frame's slot"
    );

    // Exact counting resumes on the re-established baseline: 6 → 8 skips 7.
    pubr.publish_raw(&vector3_frame(8)).expect("publish");
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        1,
        "exact counting must resume after the corrupt-drain re-baseline"
    );

    // An all-undersized drain (no readable frame at all) must
    // leave the SAME state as the mixed corrupt drain — the reset is
    // deliberately hoisted above the first-readable-frame gate. A mutation
    // moving it back inside would keep the stale (P, 8) baseline and count
    // the corrupt frame's slot on the next drain.
    pubr.publish_raw(&[0u8; 16]).expect("publish undersized");
    drain_expect_corrupt(&mut sub);
    pubr.publish_raw(&vector3_frame(10)).expect("publish");
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        1,
        "an all-undersized drain must also reset the baseline (hoisted \
         contract) — counting here would fabricate from the corrupt slot"
    );
    pubr.publish_raw(&vector3_frame(12)).expect("publish");
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        2,
        "exact counting must resume after the all-undersized re-baseline"
    );

    // Corrupt is not an identity anomaly — the suspension
    // tripwire must never have fired in this test.
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("eviction counting suspended"))
            .count();
        if n == 0 {
            Ok(())
        } else {
            Err(format!(
                "corrupt drains must not fire the identity-anomaly warn (got {n})"
            ))
        }
    });
}

/// The drop_oldest eviction `tracing::warn!` is ONCE-PER-REGIME
/// (aligned with the `BackpressureEvent` edge-trigger), while the counter bump
/// stays UNCONDITIONAL (Principle #3). A warn on EVERY evicting
/// drain measured 91% of a healthy nav2 `record.log`.
///
/// Oracle (NOT a self-compare): N drains, each evicting exactly 1 frame within
/// ONE unbroken regime (no clean drain between → the edge-trigger stays closed
/// after the first). The eviction warn must appear EXACTLY ONCE; the counter
/// must equal N. Reverting the `emit_warn = fire` gate re-floods to N warns and
/// fails the `== 1` assert; gating the COUNTER instead of the warn would break
/// the `== N` assert.
///
/// The sustained (post-head) drains are not FULLY silent:
/// each emits a `debug!` carrying the running `total` (invisible at the default
/// `info` level, so the flood stays dead, but present under a debug filter).
/// `#[traced_test]` captures at TRACE, so the oracle also pins exactly N-1
/// sustained debug lines + the running total on the last one. The warn count is
/// taken on a WARN-UNIQUE substring ("detected via wire-sequence gap") because
/// the debug line shares the "evicted oldest queued message" phrase.
#[traced_test]
#[test]
fn drop_oldest_eviction_warn_is_once_per_regime_counter_is_unconditional() {
    const N: u32 = 5;
    let topic = "bped_warn_once/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut pubr = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );

    // Baseline (contiguous seqs 1,2,3): a clean drain, count 0, latch armed.
    for seq in [1u32, 2, 3] {
        pubr.publish_raw(&vector3_frame(seq)).expect("publish");
    }
    assert!(drain(&mut sub));
    assert_eq!(counters.drop_oldest_count.load(Ordering::Relaxed), 0);

    // N drains, each publishing ONE frame with a +2 seq gap (skips exactly one
    // seq → exactly 1 eviction). Every drain evicts, so no clean drain rearms
    // the edge-trigger: the whole run is ONE regime.
    let mut seq = 3u32;
    for i in 1..=N {
        seq += 2; // 5 (skip 4), 7 (skip 6), 9 (skip 8), ...
        pubr.publish_raw(&vector3_frame(seq)).expect("publish");
        assert!(drain(&mut sub));
        assert_eq!(
            counters.drop_oldest_count.load(Ordering::Relaxed),
            u64::from(i),
            "the counter bumps unconditionally on every evicting drain"
        );
    }

    // Counter == N (unconditional, one eviction per drain).
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        u64::from(N),
        "every eviction is counted regardless of the warn gate"
    );

    // Exactly ONE eviction warn (WARN-unique substring) across the whole
    // regime, and exactly N-1 sustained `debug!` lines — the post-head
    // drains stay diagnosable under a debug filter without flooding `info`.
    //
    // RELEASE-SAFE (re-stated): the `sustained_debugs` count
    // observes `debug!` events, which `release_max_level_info` compiles out
    // under `--release`, so its expectation goes through `debug_lines_expected`
    // — the gate the discipline walk requires; the profile is not a property a
    // test may assume. What carries the contract in release is level-free: the
    // never-loud twin below and the unconditional `drop_oldest_count`.
    logs_assert(|lines: &[&str]| {
        // Level-free twin: a sustained eviction repeat must never be LOUD — the half of the
        // contract that survives `release_max_level_info`, where the gated
        // DEBUG count reads 0.
        for level in ["WARN", "INFO", "ERROR"] {
            let loud = lines
                .iter()
                .filter(|l| {
                    line_level(l) == Some(level)
                        && (l.contains("(sustained; first of regime logged loudly at warn)"))
                })
                .count();
            if loud != 0 {
                return Err(format!(
                    "a sustained eviction repeat was emitted at {level} ({loud} line(s))"
                ));
            }
        }
        // The loud head, matched WITH its level token AND against the
        // level-free total of that marker: a head demoted to INFO/ERROR is not
        // the once-per-regime warn this pins, and neither is a second copy of
        // it at another level.
        let warns = count_at_exclusively(lines, "WARN", &["detected via wire-sequence gap"])?;
        let sustained_debugs = count_at_exclusively(
            lines,
            "DEBUG",
            &["(sustained; first of regime logged loudly at warn)"],
        )?;
        if warns != 1 {
            return Err(format!(
                "the drop_oldest eviction warn must fire ONCE per regime across \
                 {N} evicting drains, got {warns} (pre-flood-latch this flooded to {N})"
            ));
        }
        let want_sustained = debug_lines_expected((N - 1) as usize);
        if sustained_debugs != want_sustained {
            return Err(format!(
                "expected {want_sustained} sustained debug! lines from the {} post-head \
                 drains (0 where `debug!` is compiled out), got {sustained_debugs}",
                N - 1
            ));
        }
        Ok(())
    });
    // The last sustained debug carries the running total (== N) — proving the
    // debug line is quiet-but-PRESENT with the live count, not blank.
    // A DYNAMIC marker (`format!`) on a `debug!` line: invisible to the discipline
    // walk's cross-reference, so gated by hand — the line exists only where
    // `debug!` is compiled in. The claim is carried AT DEBUG and ON THE
    // SUSTAINED LINE: a bare `logs_contain` would accept `total=N` from a line
    // at any level, emitted by anything.
    logs_assert(|lines: &[&str]| {
        let carried = count_at_exclusively(
            lines,
            "DEBUG",
            &[
                "(sustained; first of regime logged loudly at warn)",
                &format!("total={N}"),
            ],
        )?;
        if carried == debug_lines_expected(1) {
            Ok(())
        } else {
            Err(format!(
                "the sustained debug! must carry the running drop_oldest total (== {N}) on \
                 EXACTLY one line, got {carried} such DEBUG line(s)"
            ))
        }
    });
}

/// The `sample(N)` decimate `tracing::warn!` is
/// ONCE-PER-REGIME (aligned with the same `BackpressureEvent` edge-trigger as
/// `drop_oldest`), while the `sampled_count` bump stays UNCONDITIONAL
/// (Principle #3). Calling the single-event
/// `record_backpressure_event` (`emit_warn = true`) on EVERY decimated frame —
/// a fast producer into a `sample(100ms)` input at 1kHz would warn ~990/s, the exact
/// healthy-steady-state flood class also suppressed for `drop_oldest`.
///
/// Oracle (NOT a self-compare): frame ts = `seq * 1ms` (from `vector3_frame`).
/// With `sample(15)`, the first frame (seq=1) is ACCEPTED (opens the regime,
/// rearms the edge-trigger); the next N frames (seqs 2..=N+1, each < 16ms after
/// the accept) all DECIMATE within ONE unbroken regime (no accept between →
/// the edge-trigger stays closed after the first decimation). Exactly ONE warn;
/// exactly N-1 sustained `debug!` lines; `sampled_count` == N. Reverting
/// the edge-trigger gate to the unconditional single-event call re-floods to N warns; gating the
/// COUNTER instead of the warn would break the `== N` assert.
///
/// RELEASE-SAFE (re-stated): the `sustained_debugs` count
/// observes `debug!` events, compiled out under `--release`'s
/// `release_max_level_info`, so it goes through `debug_lines_expected`; the
/// contract in release rides the level-free never-loud twin and the
/// unconditional `sampled_count` — the same shape as the `drop_oldest`
/// sibling above.
#[traced_test]
#[test]
fn sample_decimate_warn_is_once_per_regime_counter_is_unconditional() {
    const N: u32 = 6;
    let topic = "bped_sample_warn_once/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut pubr = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_sample_gate_for_test(
        15,
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );

    // seq=1 (ts=1ms): first frame, last_accepted=None ⇒ ACCEPTED. Opens the
    // regime; the accept rearms the edge-trigger. sampled_count stays 0.
    pubr.publish_raw(&vector3_frame(1)).expect("publish");
    assert!(drain(&mut sub), "the first (accepted) frame is delivered");
    assert_eq!(
        counters.sampled_count.load(Ordering::Relaxed),
        0,
        "an ACCEPTED frame never bumps sampled_count"
    );

    // seqs 2..=N+1 (ts 2..=N+1 ms, all < 1+15 = 16ms after the accept): each
    // DECIMATES. No accept between them ⇒ ONE unbroken regime. A decimated
    // frame is not delivered (drain yields None).
    for seq in 2..=(N + 1) {
        pubr.publish_raw(&vector3_frame(seq)).expect("publish");
        assert!(
            !drain(&mut sub),
            "a decimated frame delivers nothing (seq {seq})"
        );
        assert_eq!(
            counters.sampled_count.load(Ordering::Relaxed),
            u64::from(seq - 1),
            "sampled_count bumps unconditionally on every decimated frame"
        );
    }

    // Counter == N (unconditional, one decimation per drained frame).
    assert_eq!(
        counters.sampled_count.load(Ordering::Relaxed),
        u64::from(N),
        "every decimation is counted regardless of the warn gate"
    );

    // Exactly ONE decimate warn across the whole regime + exactly N-1 sustained
    // `debug!` lines. The warn-unique substring "backpressure event:
    // dropped message arrived within sample window" is absent from the debug
    // line (which reads "backpressure event (sustained; ...)").
    logs_assert(|lines: &[&str]| {
        // Level-free twin: a sustained decimation repeat must never be LOUD — the half of the
        // contract that survives `release_max_level_info`, where the gated
        // DEBUG count reads 0.
        for level in ["WARN", "INFO", "ERROR"] {
            let loud = lines
                .iter()
                .filter(|l| {
                    line_level(l) == Some(level)
                        && (l.contains("(sustained; first of regime logged loudly at warn)"))
                })
                .count();
            if loud != 0 {
                return Err(format!(
                    "a sustained decimation repeat was emitted at {level} ({loud} line(s))"
                ));
            }
        }
        // The loud head, matched WITH its level token AND against the
        // level-free total of that marker: a head demoted to INFO/ERROR is not
        // the once-per-regime warn this pins, and neither is a second copy of
        // it at another level.
        let warns = count_at_exclusively(
            lines,
            "WARN",
            &["backpressure event: dropped message arrived within sample window"],
        )?;
        let sustained_debugs = count_at_exclusively(
            lines,
            "DEBUG",
            &[
                "(sustained; first of regime logged loudly at warn)",
                "dropped message arrived within sample window",
            ],
        )?;
        if warns != 1 {
            return Err(format!(
                "the sample(N) decimate warn must fire ONCE per regime across \
                 {N} decimated frames, got {warns} (pre-flood-latch this flooded to {N})"
            ));
        }
        let want_sustained = debug_lines_expected((N - 1) as usize);
        if sustained_debugs != want_sustained {
            return Err(format!(
                "expected {want_sustained} sustained debug! lines from the {} post-head \
                 decimations (0 where `debug!` is compiled out), got {sustained_debugs}",
                N - 1
            ));
        }
        Ok(())
    });
    // The last sustained debug carries the running total (== N).
    // A DYNAMIC marker (`format!`) on a `debug!` line: invisible to the discipline
    // walk's cross-reference, so gated by hand — the line exists only where
    // `debug!` is compiled in. The claim is carried AT DEBUG and ON THE
    // SUSTAINED LINE: a bare `logs_contain` would accept `total=N` from a line
    // at any level, emitted by anything.
    logs_assert(|lines: &[&str]| {
        let carried = count_at_exclusively(
            lines,
            "DEBUG",
            &[
                "(sustained; first of regime logged loudly at warn)",
                &format!("total={N}"),
            ],
        )?;
        if carried == debug_lines_expected(1) {
            Ok(())
        } else {
            Err(format!(
                "the sustained debug! must carry the running sampled total (== {N}) on \
                 EXACTLY one line, got {carried} such DEBUG line(s)"
            ))
        }
    });
}

#[test]
fn drop_oldest_mid_drain_receive_error_resets_baseline() {
    // An errored drain consumed an unknowable set of frames
    // (receive() pops before the error surfaces) that the probe never
    // observed — keeping the old baseline would count those CONSUMED frames
    // as evictions on the next successful drain (fabrication).
    let topic = "bped_recv_err/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut pubr = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );

    for seq in [1u32, 2, 3] {
        pubr.publish_raw(&vector3_frame(seq)).expect("publish");
    }
    assert!(drain(&mut sub)); // baseline (P, 3)

    for seq in [4u32, 5, 6] {
        pubr.publish_raw(&vector3_frame(seq)).expect("publish");
    }
    // First receive() pops frame 4, the second errors → frames 5, 6 stay
    // queued; frame 4 was consumed by the errored drain.
    sub.fault_inject_receive_after_for_test(1);
    let result = sub.try_view::<Vector3, _>(|_v| {});
    assert!(
        matches!(result, Err(TransportError::Receive { .. })),
        "the injected receive fault must surface, got {result:?}"
    );

    // The baseline must have RESET: with the stale (P, 3) baseline the next
    // drain (first = 5) would report Exact(1) — fabricating an eviction
    // from the frame the ERRORED drain consumed.
    assert!(drain(&mut sub)); // drains 5, 6 → baseline-establishing
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        0,
        "an errored drain is a no-information drain — the next drain \
         re-establishes the baseline and must not fabricate"
    );

    // Exact counting resumes: 6 → 8 skips 7.
    pubr.publish_raw(&vector3_frame(8)).expect("publish");
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        1,
        "exact counting must resume after the errored-drain re-baseline"
    );
}

#[test]
fn drop_oldest_trailing_replay_does_not_regress_baseline() {
    // The high-water `newest` exists precisely for replayed
    // frames arriving at the TAIL of a drain (live-then-replay in one
    // window). A drain-order baseline would regress onto the stale replayed
    // sequence and the next drain would count this drain's CONSUMED live
    // frames as evicted.
    let topic = "bped_tail_replay/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut pubr = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );

    for seq in [1u32, 2, 3] {
        pubr.publish_raw(&vector3_frame(seq)).expect("publish");
    }
    assert!(drain(&mut sub)); // baseline (P, 3)

    // Live head (4, 5), replayed tail (1, 2) — one drain window.
    for seq in [4u32, 5, 1, 2] {
        pubr.publish_raw(&vector3_frame(seq)).expect("publish");
    }
    assert!(drain(&mut sub)); // contiguous (first 4 vs baseline 3) → None
    assert_eq!(counters.drop_oldest_count.load(Ordering::Relaxed), 0);

    // The baseline must be the high-water (P, 5), NOT the trailing replayed
    // (P, 2): with the regressed baseline this drain would report Exact(3)
    // — counting the consumed live frames 3, 4, 5 as evicted.
    pubr.publish_raw(&vector3_frame(6)).expect("publish");
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        0,
        "a trailing replayed frame must not regress the baseline — the \
         high-water mark is the contract"
    );

    // Exactness is still live: 6 → 8 skips 7.
    pubr.publish_raw(&vector3_frame(8)).expect("publish");
    assert!(drain(&mut sub));
    assert_eq!(counters.drop_oldest_count.load(Ordering::Relaxed), 1);
}

#[test]
fn drop_oldest_pure_replay_drain_keeps_baseline_and_regime() {
    // A pure replay drain (only backward frames, no live
    // tail) must keep the live high-water baseline (the conditional advance
    // must not fire) AND must not touch the event regime (`armed` stays as
    // it was — a replay drain is no-information).
    let topic = "bped_pure_replay/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut pubr = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );

    for seq in [1u32, 2, 3] {
        pubr.publish_raw(&vector3_frame(seq)).expect("publish");
    }
    assert!(drain(&mut sub)); // baseline (P, 3)

    // Same-epoch eviction: 3 → 5 skips 4 → Exact(1); take the event →
    // the probe is now DISARMED with an open regime.
    for seq in [5u32, 6] {
        pubr.publish_raw(&vector3_frame(seq)).expect("publish");
    }
    assert!(drain(&mut sub));
    assert_eq!(counters.drop_oldest_count.load(Ordering::Relaxed), 1);
    assert_eq!(sub.try_take_backpressure_event().expect("event").dropped, 1);

    // PURE replay drain: only backward frames. Baseline must stay (P, 6) —
    // an unconditional advance would set it to (P, 2) and the next drain
    // would fabricate Exact(5).
    for seq in [1u32, 2] {
        pubr.publish_raw(&vector3_frame(seq)).expect("publish");
    }
    assert!(drain(&mut sub)); // Backward — no count, no event, no state churn
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        1,
        "a pure-replay drain must not count"
    );
    assert!(sub.try_take_backpressure_event().is_none());

    // Next eviction continues the SAME regime (Backward did not rearm):
    // counter advances exactly, but no fresh event fires.
    pubr.publish_raw(&vector3_frame(8)).expect("publish");
    assert!(drain(&mut sub)); // first 8 vs baseline 6 → Exact(1)
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        2,
        "the baseline must still be the live high-water (P, 6) — an \
         unconditional Backward advance to (P, 2) would fabricate Exact(5)"
    );
    assert!(
        sub.try_take_backpressure_event().is_none(),
        "Backward is no-information: it must NOT rearm the event trigger — \
         the still-open regime accumulates without a fresh event"
    );
}

#[test]
fn drop_oldest_alternating_publishers_count_exactly_no_warns() {
    // What was the "sustained identity anomaly" (the detector
    // suspended counting + tripwire-warned on a cadence) is now plain
    // multi-publisher operation — alternating single-frame publishers
    // count exactly (no gaps → nothing to count) with no warns at all.
    let (mut a, mut b, mut sub, counters) = epoch_harness("bped_cadence/out");

    tick_n(&mut a, 1);
    assert!(drain(&mut sub)); // A baseline-establishes

    // 64 alternating single-publisher drains: B establishes on the first;
    // every subsequent drain is contiguous on its stream — Clean per id.
    for i in 0..64 {
        if i % 2 == 0 {
            tick_n(&mut b, 1);
        } else {
            tick_n(&mut a, 1);
        }
        assert!(drain(&mut sub));
    }
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        0,
        "contiguous per-stream sequences must never count, however many \
         publisher alternations occur"
    );
    assert!(sub.try_take_backpressure_event().is_none());

    // Per-id exactness still live after the churn: flood A → exactly 4.
    tick_n(&mut a, 12);
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        4,
        "per-stream exact counting must survive sustained publisher churn"
    );
}

#[test]
fn drop_oldest_restart_drain_rearms_event_for_post_restart_loss() {
    let (mut a, mut b, mut sub, counters) = epoch_harness("bped_rearm/out");

    // Baseline, then a same-stream overflow takes the event → probe DISARMED.
    tick_n(&mut a, 3);
    assert!(drain(&mut sub));
    tick_n(&mut a, 10); // 2 evicted (8-deep queue)
    assert!(drain(&mut sub));
    assert_eq!(counters.drop_oldest_count.load(Ordering::Relaxed), 2);
    assert_eq!(sub.try_take_backpressure_event().expect("event").dropped, 2);

    // Restart hand-over with NO intervening same-stream clean drain: B's
    // baseline-establishing drain has no eviction and no backward stream,
    // so it is CLEAN — it must rearm the trigger (the rearm comes
    // from the clean-drain rule, not a dedicated anomaly arm).
    tick_n(&mut b, 1);
    assert!(drain(&mut sub));
    assert!(sub.try_take_backpressure_event().is_none());

    // …and the very next drain is heavy post-restart loss. If the
    // baseline-establishing drain failed to rearm, sustained overload
    // after a restart would never fire the handler again (counter
    // advances, event channel dark) — this drain pins the rearm.
    tick_n(&mut b, 12); // 4 evicted on B's stream
    assert!(drain(&mut sub));
    assert_eq!(counters.drop_oldest_count.load(Ordering::Relaxed), 6);
    let ev = sub
        .try_take_backpressure_event()
        .expect("post-restart eviction must fire a FRESH event (clean drain rearms)");
    assert_eq!(ev.dropped, 4);
}

#[test]
fn drop_oldest_two_publishers_real_interleaved_eviction_counts_per_id() {
    // REAL queue overflow on a two-publisher topic.
    // iceoryx2 queues are PER (publisher, subscriber) CONNECTION (verified
    // in the SPSC ring source — `buffer_size` bounds each connection
    // independently), so each stream overflows on its own: A's 12 frames
    // evict 4 of A's, B's 10 frames evict 2 of B's, and the per-id
    // baselines attribute both exactly in ONE drain.
    let topic = "bped_two_pub/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut pub_a = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut pub_b = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );

    pub_a.publish_raw(&vector3_frame(1)).expect("publish");
    pub_b.publish_raw(&vector3_frame(1)).expect("publish");
    assert!(drain(&mut sub)); // baselines (A,1), (B,1)
    assert_eq!(counters.drop_oldest_count.load(Ordering::Relaxed), 0);

    // A: a2..a13 into its 8-deep connection queue → keeps a6..a13,
    // evicts a2..a5 (4). B: b2..b11 → keeps b4..b11, evicts b2..b3 (2).
    for seq in 2u32..=13 {
        pub_a.publish_raw(&vector3_frame(seq)).expect("publish");
    }
    for seq in 2u32..=11 {
        pub_b.publish_raw(&vector3_frame(seq)).expect("publish");
    }
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        6,
        "real per-connection overflow must be attributed exactly per \
         stream (A lost 4, B lost 2 → total 6)"
    );
    assert_eq!(
        sub.try_take_backpressure_event().expect("event").dropped,
        6,
        "the event carries the drain's exact cross-stream evicted total"
    );
}

#[test]
fn drop_oldest_per_stream_backward_does_not_block_other_stream_counting() {
    // A replay (Backward) on ONE stream must not suspend
    // counting on the OTHER stream in the same drain — and the replayed
    // stream's baseline must survive for its own later exact counting.
    let topic = "bped_per_stream_bwd/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut pub_a = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut pub_b = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );

    for seq in [1u32, 2, 3] {
        pub_a.publish_raw(&vector3_frame(seq)).expect("publish");
        pub_b.publish_raw(&vector3_frame(seq)).expect("publish");
    }
    assert!(drain(&mut sub)); // baselines (A,3), (B,3)

    // One drain: A replays stale frames (Backward — never counted) while
    // B has a genuine gap (3 → 5 skips 4 → Exact(1)).
    for seq in [1u32, 2] {
        pub_a.publish_raw(&vector3_frame(seq)).expect("publish");
    }
    for seq in [5u32, 6] {
        pub_b.publish_raw(&vector3_frame(seq)).expect("publish");
    }
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        1,
        "B's exact count must land even though A's stream was replaying"
    );
    assert_eq!(sub.try_take_backpressure_event().expect("event").dropped, 1);

    // A's baseline survived its replay: its own later gap counts exactly.
    pub_a.publish_raw(&vector3_frame(5)).expect("publish"); // 3 → 5 skips 4
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        2,
        "A's baseline must survive its replay drain (3 → 5 counts Exact(1))"
    );
}

#[test]
fn drop_oldest_baseline_capacity_eviction_keeps_counting_exact() {
    // Per-id baselines are capped at the topic's
    // max_publishers — the iceoryx2 DEFAULT is 2 (config.rs
    // `max_publishers: 2`; `with_buffer_size(8)` sets the SUBSCRIBER queue
    // depth, a different knob). Publisher RESTARTS mint new ids over time,
    // so from the 3rd distinct stream onward every insert evicts the
    // longest-unseen dead stream's baseline — and exact counting must keep
    // working throughout all 9 generations.
    let topic = "bped_cap_evict/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut sub = tt.subscriber(topic);
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );

    // 9 publisher generations (each drop frees the service slot; each new
    // port mints a fresh UniquePublisherId). Every generation
    // baseline-establishes (count 0) and then proves exactness with a
    // hand-stamped gap.
    for generation in 0u32..9 {
        let mut pubr = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
        pubr.publish_raw(&vector3_frame(1)).expect("publish");
        assert!(drain(&mut sub)); // stream baseline-establishes
        pubr.publish_raw(&vector3_frame(3)).expect("publish"); // skips 2
        assert!(drain(&mut sub)); // Exact(1) on this generation's stream
        assert_eq!(
            counters.drop_oldest_count.load(Ordering::Relaxed),
            u64::from(generation) + 1,
            "exact counting must keep working across baseline-capacity \
             eviction (generation {generation})"
        );
        // pubr drops here — the stream dies, its baseline lingers until
        // capacity pressure evicts it.
    }
}

#[test]
fn drop_oldest_event_regime_timestamp_is_high_water_not_drain_order() {
    // Pins the high-water regime timestamp. `vector3_frame` stamps
    // `timestamp_ns = seq * 1_000_000`, so a trailing replayed frame
    // carries a visibly STALE clock — the fired event's
    // `regime_started_at_ns` must come from the high-water frame, not the
    // drain-order-last (replayed) one.
    let topic = "bped_regime_ts/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut pubr = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );

    for seq in [1u32, 2, 3] {
        pubr.publish_raw(&vector3_frame(seq)).expect("publish");
    }
    assert!(drain(&mut sub)); // baseline (P, 3)

    // One drain: an eviction gap (3 → 5 skips 4) plus a TRAILING replayed
    // frame (seq 1, ts 1_000_000).
    for seq in [5u32, 1] {
        pubr.publish_raw(&vector3_frame(seq)).expect("publish");
    }
    assert!(drain(&mut sub)); // Exact(1) fires
    let ev = sub.try_take_backpressure_event().expect("eviction event");
    assert_eq!(ev.dropped, 1);
    assert_eq!(
        ev.regime_started_at_ns, 5_000_000,
        "the regime timestamp must be the high-water frame's wire clock \
         (5_000_000), not the trailing replayed frame's stale 1_000_000"
    );
}

/// The drop_oldest intermittency warn ("eviction counting is intermittent"),
/// as (lines carrying the marker at ANY level, lines carrying it at WARN). The
/// silence arms use the level-free count (an absence holds at every level);
/// the exactly-once arms require both, so a cadence warn demoted to INFO or
/// ERROR cannot pass as the one warn.
fn intermittent(lines: &[&str]) -> (usize, usize) {
    const MARKER: &str = "eviction counting is intermittent";
    let all = lines.iter().filter(|l| l.contains(MARKER)).count();
    let at_warn = lines
        .iter()
        .filter(|l| line_level(l) == Some("WARN") && l.contains(MARKER))
        .count();
    (all, at_warn)
}

#[traced_test]
#[test]
fn drop_oldest_sustained_replay_warns_at_cadence() {
    // Symmetric with the identity-anomaly
    // cadence test — sustained re-delivery (every drain Backward) must
    // surface at warn level every ANOMALY_REWARN_EVERY = 64 drains, with
    // NO onset warn (a single replay drain is routine: every late joiner
    // on a history-enabled topic produces one).
    let topic = "bped_replay_cadence/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut pubr = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );

    for seq in [100u32, 101, 102] {
        pubr.publish_raw(&vector3_frame(seq)).expect("publish");
    }
    assert!(drain(&mut sub)); // baseline (P, 102)

    // 63 consecutive pure-replay drains: all silent (no onset warn).
    for _ in 0..63 {
        pubr.publish_raw(&vector3_frame(1)).expect("publish");
        assert!(drain(&mut sub)); // Backward — baseline kept at (P, 102)
    }
    assert_eq!(counters.drop_oldest_count.load(Ordering::Relaxed), 0);
    logs_assert(|lines: &[&str]| {
        let (n, _) = intermittent(lines);
        if n == 0 {
            Ok(())
        } else {
            Err(format!(
                "replay drains below the cadence must not warn (got {n})"
            ))
        }
    });

    // Drain 64: the cadence warn fires exactly once.
    pubr.publish_raw(&vector3_frame(1)).expect("publish");
    assert!(drain(&mut sub));
    logs_assert(|lines: &[&str]| {
        let (n, at_warn) = intermittent(lines);
        if (n, at_warn) == (1, 1) {
            Ok(())
        } else {
            Err(format!(
                "expected exactly 1 intermittency warn at replay drain 64, got {n} ({at_warn} at WARN)"
            ))
        }
    });

    // Drain 65: 65 % 64 == 1 → silent again.
    pubr.publish_raw(&vector3_frame(1)).expect("publish");
    assert!(drain(&mut sub));
    logs_assert(|lines: &[&str]| {
        let (n, at_warn) = intermittent(lines);
        if (n, at_warn) == (1, 1) {
            Ok(())
        } else {
            Err(format!(
                "replay drain 65 must stay silent (got {n}, {at_warn} at WARN)"
            ))
        }
    });
}

#[test]
fn drop_oldest_zero_pop_receive_error_keeps_baseline() {
    // An error on the very first receive() call
    // consumed NOTHING — the baseline is still valid and must survive, so
    // evictions spanning the errored drain are still counted exactly. (An
    // unconditional reset would silently un-count one interval
    // on every zero-pop fault.)
    let topic = "bped_zero_pop/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut pubr = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );

    for seq in [1u32, 2, 3] {
        pubr.publish_raw(&vector3_frame(seq)).expect("publish");
    }
    assert!(drain(&mut sub)); // baseline (P, 3)

    // The very next receive() errors — zero frames popped.
    sub.fault_inject_receive_after_for_test(0);
    let result = sub.try_view::<Vector3, _>(|_v| {});
    assert!(
        matches!(result, Err(TransportError::Receive { .. })),
        "the injected receive fault must surface, got {result:?}"
    );

    // The baseline survived, so the eviction across the errored drain is
    // still EXACTLY countable: 3 → 5 skips 4. (The over-resetting variant
    // re-establishes the baseline here and reports 0.)
    pubr.publish_raw(&vector3_frame(5)).expect("publish");
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        1,
        "a zero-pop errored drain must keep the still-valid baseline — \
         the spanning eviction is countable and must be counted"
    );
}

#[test]
fn drop_oldest_capacity_eviction_spares_stream_seen_in_drain() {
    // The eviction-victim choice must never pick a
    // stream that appeared in the CURRENT drain — and among the unseen, it
    // picks the longest-unseen (dead streams never refresh their stamp).
    // An unconditional evict-first mutation would displace the ACTIVE
    // stream A here and silently un-count its in-flight gap; the two-drain
    // cumulative asserts kill that mutation under either cross-connection
    // drain order.
    let topic = "bped_evict_seen/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut sub = tt.subscriber(topic);
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );

    // Stream A (stays live for the whole test) + one dead generation G1 —
    // fills the baseline set to the iceoryx2-default max_publishers = 2.
    let mut pub_a = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    pub_a.publish_raw(&vector3_frame(1)).expect("publish");
    assert!(drain(&mut sub)); // baselines [A]
    {
        let mut g1 = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
        g1.publish_raw(&vector3_frame(1)).expect("publish");
        assert!(drain(&mut sub)); // baselines [A, G1] — at capacity
    } // G1 dies; its baseline lingers with the oldest stamp.

    // One drain with BOTH an active-stream gap and a brand-new 3rd id:
    // A skips seq 2 (Exact(1)) while NEW forces a capacity eviction. The
    // victim must be G1 (unseen + oldest stamp), never A (in this drain).
    let mut pub_new = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    pub_a.publish_raw(&vector3_frame(3)).expect("publish");
    pub_new.publish_raw(&vector3_frame(1)).expect("publish");
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        1,
        "A's gap must count in the same drain that evicts a baseline — \
         evicting the in-drain stream instead would lose it"
    );

    // And A's baseline survived the eviction: its next gap still counts.
    pub_a.publish_raw(&vector3_frame(5)).expect("publish"); // skips 4
    pub_new.publish_raw(&vector3_frame(2)).expect("publish"); // contiguous
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        2,
        "A's baseline must survive the capacity eviction (a displaced A \
         would re-establish and count 0 here)"
    );
}

#[test]
fn drop_oldest_capacity_eviction_stamp_refresh_protects_quiet_live_stream() {
    // The per-drain stamp refresh in the found-baseline
    // arm (`probe.baselines[bi].2 = now_drain`) is load-bearing exactly when
    // NEITHER eviction candidate appears in the eviction drain itself — the
    // not-in-scratch filter passes both, so the victim choice falls entirely
    // on the stamps. The sibling test above can't catch its removal: there
    // the live stream A is always IN the eviction drain, and the filter
    // alone protects it.
    //
    // Here A is live but QUIET in the eviction drain (last seen drain 3);
    // G1 died after drain 2. The drain-4 capacity eviction must pick G1
    // (last seen 2) over A (last seen 3). Without the refresh, A's stamp
    // stays at its INSERT drain (1) < G1's (2) → A is displaced and its
    // next gap re-establishes uncounted.
    let topic = "bped_evict_stamp/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut sub = tt.subscriber(topic);
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );

    // Drain 1: A inserts with stamp 1.
    let mut pub_a = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    pub_a.publish_raw(&vector3_frame(1)).expect("publish");
    assert!(drain(&mut sub)); // baselines [A@1]

    // Drain 2: G1 inserts with stamp 2 — at the iceoryx2-default
    // max_publishers = 2 capacity — then dies (baseline lingers).
    {
        let mut g1 = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
        g1.publish_raw(&vector3_frame(1)).expect("publish");
        assert!(drain(&mut sub)); // baselines [A@1, G1@2]
    }

    // Drain 3: A appears again, contiguous (counts nothing) — the Some(bi)
    // arm must refresh A's stamp to 3.
    pub_a.publish_raw(&vector3_frame(2)).expect("publish");
    assert!(drain(&mut sub));
    assert_eq!(counters.drop_oldest_count.load(Ordering::Relaxed), 0);

    // Drain 4: ONLY a brand-new 3rd id appears (A quiet, G1 dead). Both
    // A and G1 pass the not-in-scratch filter; stamps alone pick the
    // victim — G1 (last seen 2) must lose to A (last seen 3).
    let mut pub_new = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    pub_new.publish_raw(&vector3_frame(1)).expect("publish");
    assert!(drain(&mut sub));

    // Drain 5: A's baseline survived → its gap still counts exactly.
    pub_a.publish_raw(&vector3_frame(4)).expect("publish"); // skips 3
    assert!(drain(&mut sub));
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        1,
        "a live-but-quiet stream must survive a capacity eviction it does \
         not appear in — the per-appearance stamp refresh must make the \
         longest-unseen (dead) stream the victim, not insertion order"
    );
}

#[traced_test]
#[test]
fn drop_oldest_mixed_backward_drains_still_rewarn_at_cadence() {
    // The sustained-re-delivery cadence must also fire
    // when every backward drain ALSO contains a cleanly-counting stream —
    // backward_run is keyed off the whole drain's any_backward, and a
    // mutation that only ticks it on PURE-replay drains would silently
    // lose the intermittency warn for exactly the multi-stream topics
    // that keep counting (and so look healthy).
    let topic = "bped_mixed_cadence/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut pub_a = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut pub_b = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );

    pub_a.publish_raw(&vector3_frame(100)).expect("publish");
    pub_b.publish_raw(&vector3_frame(1)).expect("publish");
    assert!(drain(&mut sub)); // baselines (A,100), (B,1)

    // 63 MIXED drains: A replays a stale frame (Backward) while B stays
    // contiguous (Clean) — all silent below the cadence.
    for i in 0..63u32 {
        pub_a.publish_raw(&vector3_frame(1)).expect("publish");
        pub_b.publish_raw(&vector3_frame(2 + i)).expect("publish");
        assert!(drain(&mut sub));
    }
    assert_eq!(counters.drop_oldest_count.load(Ordering::Relaxed), 0);
    logs_assert(|lines: &[&str]| {
        let (n, _) = intermittent(lines);
        if n == 0 {
            Ok(())
        } else {
            Err(format!(
                "mixed drains below the cadence must not warn (got {n})"
            ))
        }
    });

    // Mixed drain 64: the cadence warn fires exactly once.
    pub_a.publish_raw(&vector3_frame(1)).expect("publish");
    pub_b.publish_raw(&vector3_frame(65)).expect("publish");
    assert!(drain(&mut sub));
    logs_assert(|lines: &[&str]| {
        let (n, at_warn) = intermittent(lines);
        if (n, at_warn) == (1, 1) {
            Ok(())
        } else {
            Err(format!(
                "expected exactly 1 intermittency warn at mixed drain 64, got {n} ({at_warn} at WARN)"
            ))
        }
    });
}

#[traced_test]
#[test]
fn drop_oldest_backward_plus_counting_drains_still_rewarn_at_cadence() {
    // The sibling mixed-cadence test pairs the backward
    // stream with a CLEAN one, so every drain there has evicted_total == 0
    // and a `if any_backward && evicted_total == 0` mutation survives it.
    // This is the case that test's comment actually describes — "topics
    // that keep counting (and so look healthy)": A replays every drain
    // while B is being EVICTED every drain (Exact(1), evicted_total > 0).
    // Under the mutation the else arm RESETS backward_run on every such
    // drain, so the intermittency warn is lost forever — exactly while
    // A's own losses go uncounted behind the replay.
    let topic = "bped_counting_cadence/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut pub_a = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut pub_b = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber(topic);
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );

    pub_a.publish_raw(&vector3_frame(100)).expect("publish");
    pub_b.publish_raw(&vector3_frame(1)).expect("publish");
    assert!(drain(&mut sub)); // baselines (A,100), (B,1)

    // 63 backward+COUNTING drains: A replays a stale frame (Backward)
    // while B skips one sequence per drain (Exact(1)) — counting accrues,
    // cadence stays silent below 64.
    for i in 0..63u32 {
        pub_a.publish_raw(&vector3_frame(1)).expect("publish");
        pub_b
            .publish_raw(&vector3_frame(3 + 2 * i))
            .expect("publish");
        assert!(drain(&mut sub));
    }
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Relaxed),
        63,
        "B must keep counting exactly (one eviction per drain) while A replays"
    );
    logs_assert(|lines: &[&str]| {
        let (n, _) = intermittent(lines);
        if n == 0 {
            Ok(())
        } else {
            Err(format!(
                "backward+counting drains below the cadence must not warn (got {n})"
            ))
        }
    });

    // Drain 64 (still backward + counting): the cadence warn fires exactly
    // once — an eviction in the same drain must not suppress or reset it.
    pub_a.publish_raw(&vector3_frame(1)).expect("publish");
    pub_b.publish_raw(&vector3_frame(129)).expect("publish");
    assert!(drain(&mut sub));
    assert_eq!(counters.drop_oldest_count.load(Ordering::Relaxed), 64);
    logs_assert(|lines: &[&str]| {
        let (n, at_warn) = intermittent(lines);
        if (n, at_warn) == (1, 1) {
            Ok(())
        } else {
            Err(format!(
                "expected exactly 1 intermittency warn at backward+counting \
                 drain 64, got {n} ({at_warn} at WARN)"
            ))
        }
    });
}

// ===========================================================================
// At most one probe per input, installed at most once. Two-at-once
// is unrepresentable since the `BackpressureProbe` enum collapse; the
// surviving misuse is registering TWICE, which would silently replace live
// probe state (baselines, edge latches — and for `block`, orphan the shared
// outstanding mirror, deferring the producer FOREVER). Registration is
// cold-path wiring code, so the assert is hard in all builds (never
// a debug_assert). All three `register_*` entry points share the
// assert; the three tests below pin each call site separately so deleting
// the call from any one of them fails a test.
// ===========================================================================

/// A second probe registration on the same input panics —
/// in all builds — instead of silently resetting the live probe's state.
/// (Mutation oracle: deleting `assert_no_probe_installed`, or its call in
/// `register_drop_oldest_probe`, makes this test fail.)
#[test]
#[should_panic(expected = "a backpressure probe is already installed")]
fn double_probe_registration_panics() {
    let tt = TestTransport::with_buffer_size(8);
    let _pubr = tt.publisher("bped_double_reg/out", MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber("bped_double_reg/out");
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );
    sub.register_drop_oldest_probe_for_test(counters, Arc::from("consumer"), Arc::from("inp"), 8);
}

/// Cross-policy variant: a live `drop_oldest` probe must not be silently
/// replaced by a `sample(N)` gate. (Mutation oracle: deleting the assert
/// call in `register_sample_gate` makes this test fail.)
#[test]
#[should_panic(expected = "a backpressure probe is already installed")]
fn cross_policy_registration_drop_oldest_then_sample_panics() {
    let tt = TestTransport::with_buffer_size(8);
    let _pubr = tt.publisher("bped_cross_ds/out", MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber("bped_cross_ds/out");
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );
    sub.register_sample_gate_for_test(15, counters, Arc::from("consumer"), Arc::from("inp"), 8);
}

/// Cross-policy variant: a live `sample(N)` gate must not be silently
/// replaced by a `block` probe — the input would silently lose its
/// decimation gate. (The inverse ordering — a live `block` probe being
/// REPLACED — is the one that orphans the outstanding mirror, the worst
/// silent mode; both orderings die on the same assert.)
/// (Mutation oracle: deleting the assert call in `register_block_probe`
/// makes this test fail.)
#[test]
#[should_panic(expected = "a backpressure probe is already installed")]
fn cross_policy_registration_sample_then_block_panics() {
    let tt = TestTransport::with_buffer_size(8);
    let _pubr = tt.publisher("bped_cross_sb/out", MaxSliceLen::const_new(256), 0);
    let mut sub = tt.subscriber("bped_cross_sb/out");
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_sample_gate_for_test(
        15,
        Arc::clone(&counters),
        Arc::from("consumer"),
        Arc::from("inp"),
        8,
    );
    sub.register_block_probe_for_test(
        cerulion_core::credit::CreditWord::local(2),
        2,
        counters,
        Arc::from("inp"),
        8,
    );
}
