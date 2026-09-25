// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end `block` backpressure over real iceoryx2 (zero-copy).
//!
//! Proves the scheduling-based `block` policy: a producer publishing into a
//! consumer that never drains is DEFERRED by the scheduler the moment the
//! consumer's queue fills (`outstanding == depth`) — so the queue never
//! overflows, no data is lost (Principle #6), and the producer's fire count
//! plateaus at exactly `depth`. The consumer's `NodeHandle` observes the
//! defers via `backpressure_block_fires_deferred_count`. A second run yields
//! identical counts (determinism — Principle #7).
//!
//! No Cerulion buffer, no copy: the block mirror is a plain atomic
//! incremented at publish and decremented at drain.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::InputMeta;
use cerulion_core::graph::node::{ClosureNodeEntry, MacroPolicy, NodeEntry, NodeInfo};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::message::ShmMessage;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// Period producer: publishes one Vector3 per tick.
#[cerulion_node(period_ms = 10)]
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

/// Block consumer with depth 2 that NEVER drains its input: it is
/// `external`-triggered and the test never calls `trigger_external`, so it
/// never ticks → never drains. Its iceoryx2 queue fills and stays full,
/// forcing the producer's pre-fire to defer.
#[cerulion_node(external)]
#[derive(Default)]
struct StalledBlockConsumer {
    #[input(backpressure = block, depth = 2)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl StalledBlockConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Reads the input so the macro wires the subscriber drain — but the
        // node is `external` and never triggered, so tick never runs.
        let _ = self.inp.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// Build the producer→consumer graph: `producer.out` feeds `consumer.inp`.
fn block_graph() -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "block_test".to_string(),
        prefix: "bp".to_string(),
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
                node_type: "stalled_block_consumer".to_string(),
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
    factories.insert(
        "consumer".to_string(),
        Box::new(StalledBlockConsumerEntry::new()),
    );
    (config, factories)
}

/// Run the graph for `steps` 10ms steps; return
/// (producer_fire_count, consumer_block_deferred_count).
fn run_block_graph(steps: usize) -> (u64, u64) {
    let (config, factories) = block_graph();
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build block graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(10));
    }
    let producer_fires = runtime.node_handle("producer").unwrap().fire_count();
    let consumer_deferred = runtime
        .node_handle("consumer")
        .unwrap()
        .backpressure_block_fires_deferred_count("inp");
    (producer_fires, consumer_deferred)
}

#[test]
fn block_producer_defers_at_depth_no_data_loss() {
    // depth = 2, consumer never drains. The producer fires until the
    // consumer's queue holds 2 unconsumed frames, then the pre-fire defers
    // every subsequent step.
    let (producer_fires, consumer_deferred) = run_block_graph(10);

    // The producer published exactly `depth` frames before the queue filled
    // and it was deferred. (Pre-fire sees outstanding 0, fires→1; sees 1,
    // fires→2; sees 2 >= 2, defers from then on.)
    assert_eq!(
        producer_fires, 2,
        "producer must plateau at depth=2 (each published frame mirrors into the \
         never-drained consumer queue; the 3rd pre-fire defers)"
    );
    // Every deferred step bumps the consumer's block counter; 10 steps − 2
    // fires = 8 deferred steps.
    assert_eq!(
        consumer_deferred, 8,
        "8 of 10 steps deferred the producer (no data lost — the frames simply \
         were never produced)"
    );
}

#[test]
fn block_defer_is_deterministic_across_runs() {
    let a = run_block_graph(12);
    let b = run_block_graph(12);
    assert_eq!(
        a, b,
        "block defer counts must be bit-identical across runs (Principle #7)"
    );
    assert_eq!(a.0, 2, "producer still plateaus at depth");
}

// ===========================================================================
// Block drain + resume cycle —
// the no-data-loss invariant (Principle #6). The tests above use a
// never-draining `external` consumer, so `record_block_drained` (the
// decrement) and producer RESUME are never exercised. This test uses a DRAINING
// block consumer and asserts:
//   (a) the producer's fire_count climbs PAST `depth` once the consumer drains,
//   (b) NO data loss — EVERY published value is delivered with NO gaps
//       (sequence 1,2,3,... contiguous), and
//   (c) the producer deferred at least once (so the block path is exercised).
//
// Strengthening rationale: the original consumer used a latest-wins
// `try_view` and asserted only `w[1] > w[0]` (strictly increasing). That is
// BLIND to data loss — a dropped frame leaves the SURVIVING values still
// strictly increasing, so an overflow drop would NOT fail the test. The
// strengthened consumer FULL-DRAINS via `AnySubscriber::try_receive` (which
// delivers EVERY queued sample, not just the latest) and the test asserts the
// received value sequence is CONTIGUOUS. If the `block` defer ever failed and
// the iceoryx2 queue overflowed, a value would be silently dropped → a gap →
// this test FAILS.
//
// The consumer is a hand-rolled `ClosureNodeEntry` (not a macro node): the
// `#[cerulion_node]` macro auto-drains each input latest-wins via `try_view`
// BEFORE the user tick body runs and `.take()`s the subscriber out of the ctx,
// so full-drain is not expressible through the macro input path. The closure
// keeps the subscriber in the ctx and drains it itself.
// ===========================================================================

/// Build the block `InputMeta` for the draining consumer's `inp` port.
fn block_input_meta(depth: usize) -> InputMeta {
    InputMeta {
        name: "inp".to_string(),
        schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
        trigger: false,
        depth,
        backpressure: BackpressurePolicy::Block,
        expect_within_ms: None,
    }
}

/// Decode the producer's `n` value (Vector3.x, written as `n as f64`) from an
/// inbound frame BODY. `deliver_raw_frame` strips the 32-byte `WireHeader`
/// before invoking the `try_receive` callback, so `payload` is the Vector3
/// fixed section (`x,y,z` f64); `x` is the first f64 at `[0..8]` little-endian.
fn decode_n(payload: &[u8]) -> u32 {
    let x = f64::from_le_bytes(payload[0..8].try_into().unwrap());
    x as u32
}

/// Build a producer→draining-consumer graph. The producer is Period @ 10ms; the
/// consumer is Period @ 30ms (SLOWER), so the depth-2 queue fills between
/// consumer drains, forcing the producer to defer-then-resume. The shared
/// `seen` Vec collects EVERY value the consumer's full-drain observed, plus the
/// raw wire `sequence` of each, so the test can assert sequence continuity.
fn draining_block_graph(
    seen: Arc<Mutex<Vec<u32>>>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "drain_block_test".to_string(),
        prefix: "dbp".to_string(),
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
                node_type: "draining_block_consumer".to_string(),
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

    // Hand-rolled FULL-DRAIN consumer. Period @ 30ms (slower than the 10ms
    // producer). NodeInfo carries the block `inp` input so the graph build
    // wires the outstanding mirror + register_block_outstanding on the
    // subscriber exactly as it would for a macro node.
    let info = NodeInfo::with_meta(vec![block_input_meta(2)], vec![])
        .with_policy(MacroPolicy::Period { period_ms: 30 });
    let seen_cb = Arc::clone(&seen);
    let consumer = ClosureNodeEntry::new(info, move |ctx| {
        // FULL-DRAIN: `try_receive` invokes the callback for EVERY queued
        // sample (and decrements the block mirror by every removal). Collect
        // each producer value `n`. A dropped frame would manifest as a GAP in
        // this sequence.
        let sub = ctx
            .subscriber("inp")
            .expect("consumer subscriber 'inp' must be wired");
        let mut batch: Vec<u32> = Vec::new();
        let _drained = sub.try_receive(|msg| {
            batch.push(decode_n(msg.payload()));
        })?;
        if !batch.is_empty() {
            seen_cb.lock().unwrap().extend(batch);
        }
        Ok(())
    })
    .with_label("draining_block_consumer");
    factories.insert("consumer".to_string(), Box::new(consumer));
    (config, factories)
}

#[test]
fn block_drain_resume_no_data_loss() {
    let seen = Arc::new(Mutex::new(Vec::<u32>::new()));
    let (config, factories) = draining_block_graph(Arc::clone(&seen));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build draining graph");
    // Run long enough that the producer must defer-then-resume many times as
    // the consumer (30ms) lags the producer (10ms). The depth-2 queue fills
    // between consumer drains, deferring the producer; each consumer tick
    // full-drains the queue, so the producer resumes on its next step.
    let steps = 60;
    for _ in 0..steps {
        runtime.step(Duration::from_millis(10));
    }

    let producer_fires = runtime.node_handle("producer").unwrap().fire_count();
    let deferred = runtime
        .node_handle("consumer")
        .unwrap()
        .backpressure_block_fires_deferred_count("inp");

    // (a) fire_count climbs PAST depth=2 — the producer resumed after the
    // consumer drained (the stalled-consumer test plateaus at exactly 2).
    assert!(
        producer_fires > 2,
        "producer must resume past depth once the consumer drains (got {producer_fires} fires)"
    );

    // (b) NO data loss — the FULL-DRAIN consumer saw EVERY published value with
    // NO gaps. The producer publishes 1,2,3,...; the consumer drains every
    // queued sample. The received sequence must therefore be CONTIGUOUS from 1.
    // A real block-overflow drop (the bug this test guards) would silently
    // remove a value from the iceoryx2 queue → a gap here → FAIL.
    let seen = seen.lock().unwrap();
    assert!(!seen.is_empty(), "consumer must have received data");
    assert_eq!(
        seen[0], 1,
        "first delivered value must be the producer's first publish (1) — a missing \
         head value means an early overflow drop. Got {seen:?}"
    );
    for w in seen.windows(2) {
        assert_eq!(
            w[1],
            w[0] + 1,
            "delivered values must be CONTIGUOUS (no gap) — a gap means the block defer \
             failed and the iceoryx2 queue overflowed, silently dropping a frame. Got {seen:?}"
        );
    }
    // The number of distinct values the consumer saw must equal the producer's
    // fire count: full-drain + no-loss means every publish is delivered exactly
    // once (no duplicate redelivery, no overflow drop).
    let last_seen = *seen.last().unwrap();
    assert_eq!(
        seen.len() as u64,
        producer_fires,
        "full-drain must deliver every publish exactly once (saw {} values, producer \
         fired {producer_fires}); last_seen={last_seen}",
        seen.len()
    );

    // (c) The producer did defer at least once (otherwise this isn't testing
    // the block path) AND it resumed (fire_count > depth above proves resume).
    assert!(
        deferred >= 1,
        "the block producer must have deferred at least once (else the drain/resume \
         path is untested)"
    );
}

// ===========================================================================
// Mixed-consumer degrade. A topic with one `block`
// consumer + one `drop_oldest` consumer → `is_all_block()` is false → the
// producer is NOT deferred (fires full rate) and the block consumer degrades
// to native drop_oldest. Asserts the producer fires at the UN-pressured rate
// (the non-block sibling isn't starved) and the block consumer's
// `backpressure_block_fires_deferred_count` stays 0.
// ===========================================================================

/// A stalled block consumer (never drains) sharing a topic with a
/// drop_oldest sibling. On a mixed topic this should degrade — never defer.
#[cerulion_node(external)]
#[derive(Default)]
struct MixedBlockConsumer {
    #[input(backpressure = block, depth = 2)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl MixedBlockConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// A drop_oldest consumer (the default) on the same topic — its presence
/// makes the topic MIXED.
#[cerulion_node(external)]
#[derive(Default)]
struct DropOldestConsumer {
    #[input(backpressure = drop_oldest, depth = 2)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl DropOldestConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn mixed_block_dropoldest_producer_not_deferred() {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "mixed_test".to_string(),
        prefix: "mbp".to_string(),
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
                id: "blocker".to_string(),
                node_type: "mixed_block_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "dropper".to_string(),
                node_type: "drop_oldest_consumer".to_string(),
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
    factories.insert(
        "blocker".to_string(),
        Box::new(MixedBlockConsumerEntry::new()),
    );
    factories.insert(
        "dropper".to_string(),
        Box::new(DropOldestConsumerEntry::new()),
    );

    let clock = Arc::new(VirtualClock::new());
    // Subscriber buffer is large enough that neither stalled consumer can
    // overflow over the run — the point is the producer is NEVER deferred.
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 64).expect("build mixed graph");
    let steps = 10;
    for _ in 0..steps {
        runtime.step(Duration::from_millis(10));
    }

    let producer_fires = runtime.node_handle("producer").unwrap().fire_count();
    // The producer fires once per 10ms step — UN-pressured. On a mixed topic
    // the block consumer degrades and installs no defer edge, so the producer
    // is never deferred (decision K — don't starve the non-block sibling).
    assert_eq!(
        producer_fires, steps as u64,
        "producer must fire at the un-pressured rate on a mixed topic (block \
         consumer degrades to drop_oldest, no defer) — got {producer_fires}"
    );
    // The degraded block consumer must record ZERO defers — its block policy
    // was downgraded, so no outstanding mirror / pre-fire edge was wired.
    let blocker_deferred = runtime
        .node_handle("blocker")
        .unwrap()
        .backpressure_block_fires_deferred_count("inp");
    assert_eq!(
        blocker_deferred, 0,
        "a degraded (mixed-topic) block consumer must observe 0 block defers \
         (got {blocker_deferred})"
    );
}

// ===========================================================================
// The declared depth is
// the input's real iceoryx2 queue. A depth above the global
// subscriber buffer is NOT capped at the buffer (no `min(depth, sub_buf)`):
// the topology raises the topic's service ceiling to
// max(global, max depth) and the subscriber requests its declared depth —
// so a depth-16 input on a global-4 transport plateaus the producer at 16,
// honoring the declaration. (A capped-at-4 plateau is exactly what
// this test refuses.)
// ===========================================================================

/// Stalled block consumer with a LARGE declared depth (16) — honored as its
/// real queue even though the global default buffer is 4.
#[cerulion_node(external)]
#[derive(Default)]
struct DeepBlockConsumer {
    #[input(backpressure = block, depth = 16)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl DeepBlockConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn block_declared_depth_honored_beyond_global_default() {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "deep_block_test".to_string(),
        prefix: "depbp".to_string(),
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
                node_type: "deep_block_consumer".to_string(),
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
    factories.insert(
        "consumer".to_string(),
        Box::new(DeepBlockConsumerEntry::new()),
    );

    let clock = Arc::new(VirtualClock::new());
    // Global default buffer = 4 — SMALLER than the declared depth (16). The
    // topology raises the topic's service ceiling to max(4, 16) = 16 and
    // the input's subscriber requests its full declared depth.
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 4).expect("build deep block graph");
    let steps = 20;
    for _ in 0..steps {
        runtime.step(Duration::from_millis(10));
    }

    let producer_fires = runtime.node_handle("producer").unwrap().fire_count();
    // The producer plateaus at the DECLARED depth (16): the input's real
    // queue is 16 deep, so the pre-fire defers only once 16 frames are
    // outstanding — the declaration is honored, never silently capped.
    assert_eq!(
        producer_fires, 16,
        "producer must plateau at the declared depth (16) even though the \
         global default buffer is 4 — depth is the real queue (got \
         {producer_fires})"
    );
    let deferred = runtime
        .node_handle("consumer")
        .unwrap()
        .backpressure_block_fires_deferred_count("inp");
    assert_eq!(
        deferred,
        (steps as u64) - 16,
        "every step past the 16 fires deferred the producer (got {deferred})"
    );
}

// ===========================================================================
// The degrade contract's negative
// pin is mixed_block_dropoldest_producer_not_deferred (no defers, producer
// un-pressured). This is the POSITIVE half: a `block` input on a MIXED topic
// lands in the wiring chain's `else` arm and gets a REAL drop_oldest probe —
// when the un-deferred producer overflows its buffer, evictions are counted
// and its `#[on_event]` handler sees a DropOldest event (the policy
// flip is user-visible, exactly as USER_API documents the degrade).
// ===========================================================================

/// Flooding producer for the degraded-positive test: publishes every 5 ms,
/// far faster than the degraded consumer drains.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct MixedFloodProducer {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl MixedFloodProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// A SLOW block consumer (drains every 60 ms) on a mixed topic. Degraded to
/// drop_oldest: ~12 publishes land per drain cycle against a 4-deep buffer,
/// so iceoryx2 evicts between drains and the probe must count it.
#[cerulion_node(period_ms = 60)]
#[derive(Default)]
struct DegradedBlockConsumer {
    #[input(backpressure = block, depth = 2)]
    inp: Vector3,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl DegradedBlockConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }

    #[on_event(input = "inp")]
    fn on_inp_pressure(&mut self, event: BackpressureEvent) {
        assert!(
            matches!(event.policy, BackpressurePolicy::DropOldest),
            "a DEGRADED block input must surface DropOldest events (the \
             policy flip is user-visible), got {:?}",
            event.policy
        );
        assert!(
            event.dropped > 0,
            "a drop_oldest event carries the evicted count (got {})",
            event.dropped
        );
        self.fires.fetch_add(1, Ordering::Relaxed);
    }
}

#[test]
fn mixed_topic_degraded_block_counts_drop_oldest() {
    let fires = Arc::new(AtomicU64::new(0));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "mixed_degrade_positive".to_string(),
        prefix: "mdp".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "mixed_flood_producer".to_string(),
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
                id: "blocker".to_string(),
                node_type: "degraded_block_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "dropper".to_string(),
                node_type: "drop_oldest_consumer".to_string(),
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
        Box::new(MixedFloodProducerEntry::new()),
    );
    let blocker = DegradedBlockConsumer {
        fires: Arc::clone(&fires),
        ..Default::default()
    };
    factories.insert(
        "blocker".to_string(),
        Box::new(DegradedBlockConsumerEntry::with_state(blocker)),
    );
    factories.insert(
        "dropper".to_string(),
        Box::new(DropOldestConsumerEntry::new()),
    );
    let clock = Arc::new(VirtualClock::new());
    // Buffer 4 — small enough that the 5 ms flood overflows it between the
    // blocker's 60 ms drains.
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 4).expect("build degraded graph");
    for _ in 0..60 {
        runtime.step(Duration::from_millis(5));
    }
    let handle = runtime.node_handle("blocker").unwrap();
    let evicted = handle.backpressure_drop_oldest_count("inp");
    assert!(
        evicted > 0,
        "the degraded block input's drop_oldest probe must count real \
         iceoryx2 evictions (got {evicted})"
    );
    assert_eq!(
        handle.backpressure_block_fires_deferred_count("inp"),
        0,
        "the degraded input observes zero block defers (no defer edge wired)"
    );
    let fired = fires.load(Ordering::Relaxed);
    assert!(
        fired >= 1,
        "the degraded input's #[on_event] handler must fire with a \
         DropOldest event (got {fired})"
    );
}

// ===========================================================================
// Flow mode: the REPLAY DEMOTION of the `block` pre-fire gate.
//
// Under trace-driven replay the fires come from the recording, so a gate that
// reads CROSS-RANK queue occupancy — occupancy the recording captures nothing
// about — must not decide anything: re-deriving it can legally
// disagree with the recorded schedule and manufacture spurious fire-schedule
// divergences on exactly the block-paced graphs flow mode exists for. It is demoted
// to an OBSERVATION (`GraphRuntime::block_credits`) that the verifier asserts
// conservation on.
//
// `throttle_ms` is NOT demoted: it is rank-local and re-derivable from the
// recorded clock, so it reproduces the recording exactly.
//
// The pure decision (including "the block half is not even EVALUATED", which no
// behavioural assertion can see) is oracle-tested at
// `graph::runtime::tests::the_pre_fire_composer_answers_its_hand_vectors`; this
// arm is the no-inert-shipping half — the flag really is wired into the
// composed closure a real graph build installs.
// ===========================================================================

/// A producer that is BOTH `block`-gated (its consumer's queue fills) and
/// rate-capped. `throttle_ms` is mutually exclusive with `period_ms`, so the
/// trigger is `external` + host-driven and the test fires it every step.
#[cerulion_node(external, throttle_ms = 25)]
#[derive(Default)]
struct ThrottledBlockProducer {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl ThrottledBlockProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// `producer.out` → the never-draining depth-2 `block` consumer, with the
/// producer additionally throttled.
fn throttled_block_graph() -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let (mut config, mut factories) = block_graph();
    config.prefix = "bpthr".to_string();
    config.nodes[0].node_type = "throttled_block_producer".to_string();
    factories.insert(
        "producer".to_string(),
        Box::new(ThrottledBlockProducerEntry::new()),
    );
    (config, factories)
}

/// Run the throttled block graph for `steps` 10 ms steps, host-triggering the
/// producer every step. Returns
/// `(producer fires, consumer block-defer count, edge saturated?, outstanding)`.
fn run_throttled_block_graph(steps: usize, replay_bypass: bool) -> (u64, u64, bool, u64) {
    let (config, factories) = throttled_block_graph();
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build throttled block graph");
    runtime.set_block_gate_replay_bypass(replay_bypass);
    assert_eq!(
        runtime.block_gate_replay_bypass(),
        replay_bypass,
        "the bypass must be observable (Principle #3)"
    );
    for _ in 0..steps {
        runtime.trigger_external("producer").expect("trigger");
        runtime.step(Duration::from_millis(10));
    }
    let credits: Vec<_> = runtime.block_credits().collect();
    assert_eq!(
        credits.len(),
        1,
        "one credit record per block CONSUMER edge, never one per producer"
    );
    let credit = credits[0];
    assert_eq!(
        (credit.topic, credit.consumer_node, credit.consumer_input),
        ("/bpthr/producer/out", "consumer", "inp"),
        "the credit names the edge it belongs to"
    );
    assert_eq!(credit.depth, 2, "the consumer's declared `depth = 2`");
    let fires = runtime.node_handle("producer").unwrap().fire_count();
    let deferred = runtime
        .node_handle("consumer")
        .unwrap()
        .backpressure_block_fires_deferred_count("inp");
    (fires, deferred, credit.is_saturated(), credit.outstanding)
}

#[test]
fn a_replay_bypassed_block_gate_stops_gating_while_the_throttle_still_does() {
    const STEPS: usize = 10;

    // LIVE (the shipping contract, restated here as the control): both gates
    // apply. The producer fires at t = 10 (never fired ⇒ no throttle) and t = 40
    // (30 ms ≥ 25), by which point the never-drained depth-2 queue holds 2 and
    // the block gate defers every later step.
    let (live_fires, live_deferred, live_saturated, _) = run_throttled_block_graph(STEPS, false);
    assert_eq!(
        live_fires, 2,
        "LIVE: the producer plateaus at the consumer's depth"
    );
    assert!(
        live_deferred > 0,
        "LIVE: the block gate accounts every step it defers"
    );
    assert!(live_saturated, "LIVE: the edge really did fill");

    // REPLAY: the block gate is demoted. The producer now fires on its THROTTLE
    // alone — t = 10, 40, 70, 100 — i.e. 4 of 10 steps, publishing PAST the
    // consumer's depth.
    let (replay_fires, replay_deferred, replay_saturated, outstanding) =
        run_throttled_block_graph(STEPS, true);
    assert_eq!(
        replay_fires, 4,
        "REPLAY: the block gate no longer defers (4 > depth 2), and the THROTTLE \
         still does (4 < 10 steps — a bypass that dropped the throttle disjunct \
         too would fire on every step)"
    );
    assert_eq!(
        replay_deferred, 0,
        "REPLAY: observing is not accounting — bumping the consumer's counter for \
         a defer that did not happen would falsify it AND dispatch a \
         `BackpressureEvent` into the consumer's handler, changing what the \
         candidate computes"
    );

    // …and the observation the verifier needs is available anyway, which is the
    // whole reason the accounting can be dropped.
    assert!(
        replay_saturated,
        "REPLAY: `block_credits` still reports the edge as saturated"
    );
    assert_eq!(
        outstanding, replay_fires,
        "every published frame is outstanding (nothing drains this consumer), so \
         the credit is a live count rather than a stale snapshot"
    );
}

/// A producer on its OWN topic (no consumer, no `block` edge) — the
/// plan-driven fire's oracle on the NON-block level path. Its 10 ms period never
/// elapses in the 1 ms steps below, so any fire it performs came from the plan
/// and nowhere else. (`#[cerulion_node]` requires at least one port, hence the
/// output.)
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct SoloTicker {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl SoloTicker {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Flow mode: the TRACE-DRIVEN plan reaches BOTH level seams.
///
/// `GraphRuntime::step` splits a level in two: the non-block subset goes through
/// `Scheduler::decide_fires`, and the `block`-involved subset through
/// `evaluate_nodes_fused` (the ordering fix: a block producer must publish
/// before the next one decides). The scheduler-level arms in `scheduler_test.rs`
/// drive the FLAT seam, so without this arm the two level seams' plan branches
/// are pinned by nothing — and the fused one serves exactly the `block`
/// producers the whole demotion is about, which is where an inert branch would
/// hurt most. That is why this arm lives in the `block` file: it needs a real
/// block edge to have a fused subset at all.
///
/// The oracle is a graph in which the TRIGGERS fire nothing: both `producer`
/// (period 10 ms, block-involved ⇒ fused seam) and `solo` (period 10 ms, no
/// ports ⇒ non-block seam) are stepped 1 ms at a time, so every fire below came
/// from the plan. The second step inverts it — the periods elapse and the plan
/// is EMPTY, so a seam that fell through to live deciding fires.
#[test]
fn a_trace_driven_plan_reaches_the_block_and_non_block_level_seams() {
    let (mut config, mut factories) = block_graph();
    config.prefix = "bpplan".to_string();
    config.nodes.push(NodeDef {
        fuse: None,
        ros2: None,
        id: "solo".to_string(),
        node_type: "solo_ticker".to_string(),
        inputs: vec![],
        outputs: vec![OutputDef {
            name: "out".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }],
    });
    factories.insert("solo".to_string(), Box::new(SoloTickerEntry::new()));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build plan graph");

    // Step 0 at t = 1 ms: neither period has elapsed, and the plan names both.
    runtime
        .set_replay_fire_plan(
            0,
            &[
                cerulion_core::scheduler::ReplayFire {
                    node_id: "producer",
                    first_fire_ns: 4_242,
                    fire_count: 1,
                    interval_ns: 0,
                },
                cerulion_core::scheduler::ReplayFire {
                    node_id: "solo",
                    first_fire_ns: 4_242,
                    fire_count: 1,
                    interval_ns: 0,
                },
            ],
        )
        .expect("install plan");
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        runtime.node_handle("producer").unwrap().fire_count(),
        1,
        "the FUSED (block-involved) seam must honour the plan — its 10 ms period \
         has not elapsed at t = 1 ms"
    );
    assert_eq!(
        runtime.node_handle("solo").unwrap().fire_count(),
        1,
        "the NON-block level seam must honour the plan too"
    );
    assert!(runtime.unconsumed_replay_fires().is_empty());

    // Step 1 at t = 11 ms: BOTH periods have now elapsed, and the plan is empty.
    runtime.set_replay_fire_plan(1, &[]).expect("install plan");
    runtime.step(Duration::from_millis(10));
    assert_eq!(
        (
            runtime.node_handle("producer").unwrap().fire_count(),
            runtime.node_handle("solo").unwrap().fire_count()
        ),
        (1, 1),
        "an EMPTY plan fires nothing on EITHER seam — a seam that fell through to \
         live deciding would fire both periods here"
    );
}

/// The INTRA-STEP pause seam through the `GraphRuntime` WRAPPERS,
/// on the FUSED (block-involved) tick path.
///
/// Two gaps, one arm. (a) All EIGHT `GraphRuntime` replay-pause wrappers had
/// zero callers and zero tests — four of them are pairwise swappable with a
/// compile-clean delegation slip (`has_replay_injection_hook` ⇄
/// `is_replay_pause_armed`, `clear_replay_injection_hook` ⇄
/// `clear_replay_intra_step_pauses`, and `replay_pause_mismatches` ⇄
/// `replay_plan_mismatches`), so the state ladder below asserts a point where
/// each pair DISAGREES rather than a state both satisfy. (b) The seam's own
/// justification names `evaluate_nodes_fused` as the path an inert branch would
/// hurt most — it serves exactly the `block`-involved nodes — and no test
/// reached it, which is why this arm lives in the `block` file: it needs a real
/// block edge to have a fused subset at all.
///
/// The block gate is DEMOTED (`set_block_gate_replay_bypass`), which is the
/// shipping replay posture and what keeps this arm about the pause seam rather
/// than about `outstanding` credit arithmetic.
#[test]
fn the_graph_runtime_pause_wrappers_drive_the_seam_on_the_fused_block_path() {
    let (mut config, mut factories) = block_graph();
    config.prefix = "bppause".to_string();
    // A second producer with NO block edge, so the arm covers the non-block
    // level seam in the same step and pins the report's insertion order across
    // the two seams (`producer` idx 0, `consumer` idx 1, `solo` idx 2).
    config.nodes.push(NodeDef {
        fuse: None,
        ros2: None,
        id: "solo".to_string(),
        node_type: "solo_ticker".to_string(),
        inputs: vec![],
        outputs: vec![OutputDef {
            name: "out".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }],
    });
    factories.insert("solo".to_string(), Box::new(SoloTickerEntry::new()));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build pause graph");
    runtime.set_block_gate_replay_bypass(true);

    let journal: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let hook: cerulion_core::scheduler::ReplayInjectionHook = {
        let journal = Arc::clone(&journal);
        Arc::new(move |id: &str, after_fire: u32| {
            journal
                .lock()
                .expect("journal not poisoned")
                .push(format!("pause({id},{after_fire})"));
        })
    };

    // --- the state ladder: each rung distinguishes one swappable pair --------
    assert!(!runtime.has_replay_injection_hook());
    assert!(!runtime.is_replay_pause_armed());

    runtime.set_replay_injection_hook(hook);
    assert!(
        runtime.has_replay_injection_hook(),
        "the hook wrapper must reach `set_replay_injection_hook`"
    );
    assert!(
        !runtime.is_replay_pause_armed(),
        "installing a HOOK does not arm the PAUSES — the point where a \
         has_hook ⇄ is_armed swap disagrees"
    );

    // Installed in an order that DISCRIMINATES the report's sort: `solo` is
    // insertion idx 2 and is named FIRST, so a seam that dropped the
    // touched-list sort would report (solo, producer) rather than the insertion
    // order asserted below. (Naming `producer` first is ALREADY ascending, so
    // that order pins nothing.)
    runtime
        .set_replay_intra_step_pauses(
            0,
            &[
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "solo",
                    after_fire: 9,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "producer",
                    after_fire: 5,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "producer",
                    after_fire: 1,
                },
                // REACHED on the non-block level seam, so the journal carries
                // one entry from EACH seam and its order is an oracle rather
                // than a single-element tautology.
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "solo",
                    after_fire: 1,
                },
            ],
        )
        .expect("install pauses");
    assert!(runtime.is_replay_pause_armed());

    // The FUSED-path discriminator, cribbed from
    // `a_trace_driven_plan_reaches_the_block_and_non_block_level_seams`: the
    // claim "the hook ran on the fused seam" is only worth anything if
    // `producer` really is block-involved, and nothing in a journal can show
    // that. `block_credits` can — it is the registry the block gate is built
    // from, and a node on a live block edge is exactly the subset
    // `GraphRuntime::step` routes through `evaluate_nodes_fused`. If the edge
    // ever stopped being wired, this fails HERE rather than passing on the
    // ordinary level seam while still claiming the fused one.
    let credits: Vec<_> = runtime.block_credits().collect();
    assert_eq!(
        credits.len(),
        1,
        "exactly one block edge, so the fused subset is non-empty"
    );
    assert_eq!(
        (credits[0].consumer_node, credits[0].consumer_input),
        ("consumer", "inp"),
        "the block edge consumes `producer`'s topic — which is what puts \
         `producer` in the FUSED subset rather than the plain level seam"
    );
    assert_eq!(
        credits[0].topic, "/bppause/producer/out",
        "and it is the producer's own topic, not some other edge"
    );

    // Step 0 at t = 1 ms: neither 10 ms period has elapsed, so every fire below
    // came from the plan and nowhere else. The plan is listed `solo` FIRST —
    // the REVERSE of insertion order — so the journal's ordering claim cannot be
    // satisfied by a seam that merely walks the plan as given.
    runtime
        .set_replay_fire_plan(
            0,
            &[
                cerulion_core::scheduler::ReplayFire {
                    node_id: "solo",
                    first_fire_ns: 4_242,
                    fire_count: 2,
                    interval_ns: 0,
                },
                cerulion_core::scheduler::ReplayFire {
                    node_id: "producer",
                    first_fire_ns: 4_242,
                    fire_count: 2,
                    interval_ns: 0,
                },
            ],
        )
        .expect("install plan");
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        journal.lock().expect("journal not poisoned").clone(),
        vec!["pause(producer,1)".to_string(), "pause(solo,1)".to_string()],
        "the hook ran on the FUSED (block-involved) path AND on the plain level \
         seam, in that order — the fused subset of a level runs first, and the \
         plan named `solo` first, so this cannot be the plan's order"
    );
    assert_eq!(
        runtime.node_handle("producer").unwrap().fire_count(),
        2,
        "the fused seam honoured the plan's burst length"
    );
    assert_eq!(
        runtime.unconsumed_replay_pauses(),
        vec![("producer", 5), ("solo", 9)],
        "unreached slots are reported, in node insertion order"
    );
    assert_eq!(runtime.replay_pause_mismatches(), 0);
    assert_eq!(runtime.replay_hook_panics(), 0);
    assert!(runtime.unconsumed_replay_fires().is_empty());

    // --- the two `clear_*` wrappers, at the point they DISAGREE -------------
    runtime.clear_replay_injection_hook();
    assert!(!runtime.has_replay_injection_hook());
    assert!(
        runtime.is_replay_pause_armed(),
        "clearing the HOOK deliberately leaves the pauses armed — a \
         clear_hook ⇄ clear_pauses swap disagrees exactly here"
    );
    runtime.clear_replay_intra_step_pauses();
    assert!(!runtime.is_replay_pause_armed());
    assert!(runtime.unconsumed_replay_pauses().is_empty());

    // --- a STALE pause list: pause_mismatches moves, plan_mismatches does not
    // (the point where that swappable pair disagrees).
    let hook2: cerulion_core::scheduler::ReplayInjectionHook = {
        let journal = Arc::clone(&journal);
        Arc::new(move |id: &str, after_fire: u32| {
            journal
                .lock()
                .expect("journal not poisoned")
                .push(format!("stale-pause({id},{after_fire})"));
        })
    };
    runtime.set_replay_injection_hook(hook2);
    runtime
        .set_replay_intra_step_pauses(
            99,
            &[cerulion_core::scheduler::IntraStepPause {
                node_id: "producer",
                after_fire: 1,
            }],
        )
        .expect("install stale pauses");
    runtime
        .set_replay_fire_plan(
            1,
            &[cerulion_core::scheduler::ReplayFire {
                node_id: "producer",
                first_fire_ns: 4_242,
                fire_count: 2,
                interval_ns: 0,
            }],
        )
        .expect("install plan");
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        journal.lock().expect("journal not poisoned").clone(),
        vec!["pause(producer,1)".to_string(), "pause(solo,1)".to_string()],
        "a stale pause list delivers NOTHING — the journal is UNCHANGED from \
         the step-0 pair, with no `stale-pause(..)` entry appended"
    );
    assert_eq!(
        runtime.replay_pause_mismatches(),
        2,
        "one consult per fire of the 2-fire burst"
    );
    assert_eq!(
        runtime.replay_plan_mismatches(),
        0,
        "the FIRE PLAN was current — the point where a pause ⇄ plan mismatch \
         swap disagrees"
    );
    assert_eq!(
        runtime.replay_hook_panics(),
        0,
        "no hook PANICKED — and this is the point where a hook_panics ⇄ \
         pause_mismatches delegation slip disagrees (the pause counter reads 2 \
         here). Asserted zero earlier too, but back there BOTH counters were 0 \
         and the swap was invisible."
    );
    assert_eq!(
        runtime.unconsumed_replay_pauses(),
        vec![("producer", 1)],
        "nothing was delivered, so nothing is consumed"
    );
}

/// The injection hook performs a REAL PUBLISH from inside a
/// replayed burst, and a draining consumer RECEIVES it — in the same step.
///
/// Every other arm on this seam journals a string from the hook, which proves
/// the CALL and nothing else. This is the first arm in which the hook does what
/// the engine's hook will actually do: hold a live publisher, loan a slot and
/// commit a frame, while the scheduler holds the graph `&mut` and a node's
/// burst is mid-flight. It is also the DELIVERY pin the repo's test rules ask
/// for — a fire count proves scheduling, a received payload proves the frame.
///
/// # What this arm can and cannot pin, MEASURED
///
/// The design note on `Scheduler::set_replay_intra_step_pauses` says a
/// before-step bucket "puts every foreign frame in the consumer FIFO ahead of
/// ALL of this step's local fires". That is true of ONE FIFO and NOT of
/// iceoryx2, which gives every PUBLISHER its own connection queue: a batched
/// consumer drain walks connections in attach order, so a frame from a SECOND
/// port lands after every frame of the first port's queue no matter when it was
/// published. Measured on this transport, publishing `1.0` (graph port),
/// `99.0` (injector port), `2.0` (graph port) and then draining yields
/// `[1.0, 2.0, 99.0]` — and publishing the injector's frame FIRST yields the
/// same shape (`[11.0, 12.0, 91.0]`). Publish-time interleaving across two
/// ports survives only if the consumer drains BETWEEN the publishes, which a
/// downstream node cannot do: it fires at its own level, after the producer's
/// whole burst.
///
/// So the ordering half of the seam's contract is pinned where it is real — the
/// hook's CALL position, by the journal arms in `scheduler_test.rs` — and this
/// arm pins the half that IS observable downstream: the injected frame is on
/// the wire DURING the step, so the consumer's fire in THAT step sees it. The
/// control is an identical publish made AFTER `step()` returns, which that
/// step's consumer does NOT see. The connection-order fact is asserted rather
/// than described, so a future reader cannot quietly re-assume a global FIFO.
#[test]
fn the_injection_hook_publishes_a_real_frame_that_reaches_the_consumer_in_the_same_step() {
    use cerulion_core::transport::publisher::CerulionPublisher;
    use cerulion_core::wire::MaxSliceLen;

    const TOPIC: &str = "/pausedeliv/shared";

    // The consumer's drain log: every frame it ever sees, in the order it saw
    // them. Accumulate-all (`try_receive`), never latest-wins — a latest-wins
    // read would structurally hide the very frame this arm is about.
    let seen: Arc<Mutex<Vec<f64>>> = Arc::new(Mutex::new(Vec::new()));

    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        // The graph producer holds one publisher slot; the injector needs the
        // second. Without the opt-in the topic is single-writer and the
        // injector's `create_publisher` is refused.
        multi_publisher_topics: vec![TOPIC.to_string()],
        name: None,
        identity: "pausedeliv".to_string(),
        prefix: "pausedeliv".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                id: "producer".to_string(),
                node_type: "block_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: Some(TOPIC.to_string()),
                }],
                ros2: None,
            },
            NodeDef {
                fuse: None,
                id: "listener".to_string(),
                node_type: "drain_all_listener".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: TOPIC.to_string(),
                }],
                outputs: vec![],
                ros2: None,
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(BlockProducerEntry::new()));
    let listener_info = NodeInfo::with_meta(
        vec![InputMeta {
            name: "inp".to_string(),
            schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
            trigger: true,
            depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
            backpressure: BackpressurePolicy::DropOldest,
            expect_within_ms: None,
        }],
        vec![],
    )
    .with_policy(MacroPolicy::DataTrigger {
        input_name: "inp".to_string(),
    });
    let seen_c = Arc::clone(&seen);
    let listener = ClosureNodeEntry::new(listener_info, move |ctx| {
        let sub = ctx.subscriber("inp").expect("listener subscriber wired");
        let _ = sub.try_receive(|msg| {
            let x = f64::from_le_bytes(msg.payload()[0..8].try_into().expect("8 payload bytes"));
            seen_c.lock().expect("seen not poisoned").push(x);
        })?;
        Ok(())
    })
    .with_label("drain_all_listener")
    // Accumulate-all reads via `try_receive`, which the unified drain's frozen
    // slot does not serve (it serves latest-wins `try_view`) — so keep the
    // Separate drain, exactly as the `/tf` listener in
    // `multi_publisher_iox2_test.rs` does.
    .with_unified_drain(false);
    factories.insert("listener".to_string(), Box::new(listener));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build the shared-topic delivery graph");

    // The injector: a SECOND real publisher on the producer's topic, held by the
    // hook. `Mutex` because `loan_proxy` needs `&mut` and the hook is an
    // `Arc<dyn Fn(..) + Send + Sync>` — the same shape the engine's own injector
    // will need.
    let mgr = Arc::clone(runtime.test_transport().expect("test transport parked"));
    let injector = Arc::new(Mutex::new(
        mgr.create_publisher(TOPIC, MaxSliceLen::const_new(256), 0)
            .expect("the injector attaches to the listed topic"),
    ));
    let publish = |p: &Arc<Mutex<CerulionPublisher>>, v: f64| {
        let mut guard = p.lock().expect("injector not poisoned");
        let mut proxy = guard.loan_proxy::<Vector3>().expect("loan");
        proxy.x = v;
        drop(proxy);
    };

    // ---- warm-up + ANTI-VACUITY control -----------------------------------
    // The injector's own path must deliver INDEPENDENTLY of the hook, or every
    // "the frame arrived" assertion below could be satisfied by the graph
    // producer alone. Fire the listener ONLY (a plan fires exactly the nodes it
    // names), so nothing but the injector's frame can appear.
    publish(&injector, -1.0);
    runtime
        .set_replay_fire_plan(
            0,
            &[cerulion_core::scheduler::ReplayFire {
                node_id: "listener",
                first_fire_ns: 4_242,
                fire_count: 1,
                interval_ns: 0,
            }],
        )
        .expect("install warm-up plan");
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        std::mem::take(&mut *seen.lock().expect("seen not poisoned")),
        vec![-1.0],
        "the injector's publisher reaches the listener on its own — without \
         this, every delivery assertion below could be the graph producer's"
    );
    assert_eq!(
        runtime.node_handle("producer").unwrap().fire_count(),
        0,
        "the warm-up plan named the listener only"
    );

    // ---- the measured step: the hook publishes from INSIDE the burst -------
    let hook: cerulion_core::scheduler::ReplayInjectionHook = {
        let injector = Arc::clone(&injector);
        Arc::new(move |_id: &str, _after_fire: u32| {
            let mut guard = injector.lock().expect("injector not poisoned");
            let mut proxy = guard.loan_proxy::<Vector3>().expect("loan from the hook");
            proxy.x = 99.0;
            drop(proxy);
        })
    };
    runtime.set_replay_injection_hook(hook);
    runtime
        .set_replay_intra_step_pauses(
            1,
            &[cerulion_core::scheduler::IntraStepPause {
                node_id: "producer",
                after_fire: 1,
            }],
        )
        .expect("install pauses");
    runtime
        .set_replay_fire_plan(
            1,
            &[
                cerulion_core::scheduler::ReplayFire {
                    node_id: "producer",
                    first_fire_ns: 5_000,
                    fire_count: 2,
                    interval_ns: 0,
                },
                cerulion_core::scheduler::ReplayFire {
                    node_id: "listener",
                    first_fire_ns: 5_000,
                    fire_count: 1,
                    interval_ns: 0,
                },
            ],
        )
        .expect("install plan");
    runtime.step(Duration::from_millis(1));

    let observed = std::mem::take(&mut *seen.lock().expect("seen not poisoned"));
    assert!(
        observed.contains(&99.0),
        "THE DELIVERY PIN: the hook's own publish, issued from inside the \
         producer's burst while the scheduler held the graph, reached the \
         consumer IN THE SAME STEP — got {observed:?}"
    );
    let mut sorted = observed.clone();
    sorted.sort_by(f64::total_cmp);
    assert_eq!(
        sorted,
        vec![1.0, 2.0, 99.0],
        "the hand oracle: BOTH graph frames and the injected one arrive in this \
         one step — no more, no fewer — got {observed:?}"
    );
    // And the MEASURED transport fact, asserted as the PROPERTY rather than the
    // accident. On this box the drain reads `[1.0, 2.0, 99.0]`: iceoryx2 queues
    // per PUBLISHER CONNECTION and a batched drain walks connections in attach
    // order, so the injected frame sorts after BOTH graph frames even though it
    // was published between them (measured directly: publishing `1.0`, `99.0`,
    // `2.0` drains as `[1.0, 2.0, 99.0]`, and publishing the injector's frame
    // FIRST drains as `[11.0, 12.0, 91.0]` — the injector's port loses either
    // way). Pinning the literal order would pin WHICH connection is walked
    // first, which is iceoryx2's business and may differ per platform; the
    // property that actually matters — publish-time interleaving across two
    // ports is ERASED, so the two graph frames stay ADJACENT — holds under
    // either walk. This is why the seam's ordering contract is pinned on the
    // hook's CALL position (the journal arms in `scheduler_test.rs`) and not
    // on a consumer's FIFO.
    let one = observed
        .iter()
        .position(|v| *v == 1.0)
        .expect("1.0 arrived");
    let two = observed
        .iter()
        .position(|v| *v == 2.0)
        .expect("2.0 arrived");
    assert_eq!(
        two,
        one + 1,
        "the producer's two frames are ADJACENT in the drain — the injected \
         frame did NOT land between them, because iceoryx2's per-connection \
         queues erase publish-time interleaving across ports (got {observed:?})"
    );
    assert!(runtime.unconsumed_replay_pauses().is_empty());
    assert_eq!(runtime.replay_hook_panics(), 0);
    assert_eq!(runtime.replay_pause_mismatches(), 0);

    // ---- the CONTROL: the same publish, made AFTER `step()` returns --------
    // This is the half that DOES discriminate. An injection performed outside
    // the step is invisible to that step's consumer fire; the pause is what
    // puts the frame on the wire while the step is still running.
    runtime
        .set_replay_intra_step_pauses(2, &[])
        .expect("no pauses this step");
    runtime
        .set_replay_fire_plan(
            2,
            &[
                cerulion_core::scheduler::ReplayFire {
                    node_id: "producer",
                    first_fire_ns: 6_000,
                    fire_count: 2,
                    interval_ns: 0,
                },
                cerulion_core::scheduler::ReplayFire {
                    node_id: "listener",
                    first_fire_ns: 6_000,
                    fire_count: 1,
                    interval_ns: 0,
                },
            ],
        )
        .expect("install plan");
    runtime.step(Duration::from_millis(1));
    publish(&injector, 98.0); // AFTER the step, not inside it.

    assert_eq!(
        std::mem::take(&mut *seen.lock().expect("seen not poisoned")),
        vec![3.0, 4.0],
        "an injection made after `step()` returns is NOT in that step's serve \
         set — the burst's two frames are, and nothing else"
    );

    // …and it is not lost either: it lands on the NEXT step, one place later in
    // the recording's serve order than the pause would have put it.
    runtime
        .set_replay_fire_plan(
            3,
            &[cerulion_core::scheduler::ReplayFire {
                node_id: "listener",
                first_fire_ns: 7_000,
                fire_count: 1,
                interval_ns: 0,
            }],
        )
        .expect("install plan");
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        std::mem::take(&mut *seen.lock().expect("seen not poisoned")),
        vec![98.0],
        "the after-step frame arrives one step LATE — the divergence the \
         intra-step pause exists to prevent"
    );
}
