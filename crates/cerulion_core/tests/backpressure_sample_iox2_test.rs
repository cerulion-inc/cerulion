// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end `sample(N)` read-gate over real iceoryx2 (zero-copy).
//!
//! Proves the subscriber-side decimation policy: a consumer reading at the
//! full step rate from a faster producer accepts at most one message per N
//! ms (keyed off the WIRE timestamp, so it is replay-deterministic), bumping
//! `backpressure_sampled_count` for every decimated read. Nothing is
//! buffered or copied — a too-soon sample is simply dropped (its SHM slot
//! released) and `try_view` returns `Ok(None)`.

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// Producer: publishes one Vector3 per 5 ms tick (wire timestamp = step time
/// via the shared VirtualClock).
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

/// Consumer: reads its input every 5 ms tick, but `sample(15)` decimates to
/// at most one accepted read per 15 ms.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct SamplingConsumer {
    #[input(backpressure = sample(15))]
    inp: Vector3,
    last_seen: f64,
}

#[cerulion_node_impl]
impl SamplingConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Reading `self.inp` drives `try_view`, which is where the sample
        // gate decimates. On a decimated tick the read yields nothing.
        self.last_seen = self.inp.x;
        Ok(())
    }
}

fn sample_graph() -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "sample_test".to_string(),
        prefix: "sp".to_string(),
        nodes: vec![
            NodeDef {
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
    factories.insert(
        "consumer".to_string(),
        Box::new(SamplingConsumerEntry::new()),
    );
    (config, factories)
}

/// Run for `steps` 5ms steps; return the consumer's sampled (decimated) count.
fn run_sample_graph(steps: usize) -> u64 {
    let (config, factories) = sample_graph();
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build sample graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(5));
    }
    let handle = runtime.node_handle("consumer").unwrap();
    // Belt-and-suspenders (a review addition): a sample(N) input
    // installs ONLY the read-gate — the other two policies' machinery must
    // stay dormant on it. Dormancy of `drop_oldest_count` is type-enforced
    // by the single `BackpressureProbe` slot; `block_fires_deferred_count`
    // stays dormant for a different reason — a sample edge makes the topic
    // non-all-block, so no producer defer edge is ever wired. These asserts
    // pin both behaviorally for every caller of this helper.
    assert_eq!(
        handle.backpressure_drop_oldest_count("inp"),
        0,
        "a sample(N) input must never bump drop_oldest_count"
    );
    assert_eq!(
        handle.backpressure_block_fires_deferred_count("inp"),
        0,
        "a sample(N) input must never bump block_fires_deferred_count"
    );
    handle.backpressure_sampled_count("inp")
}

#[test]
fn sample_gate_decimates_too_soon_reads() {
    // Producer publishes every 5ms; sample(15) accepts at most one read per
    // 15ms → ~2 of every 3 reads are decimated. Over 12 steps there must be
    // a non-trivial number of decimations, and far fewer than 12 (some reads
    // ARE accepted — the gate is not dropping everything).
    let sampled = run_sample_graph(12);
    assert!(
        sampled > 0,
        "sample(15) must decimate at least some of the 5ms-spaced reads (got {sampled})"
    );
    assert!(
        sampled < 12,
        "sample(15) must still ACCEPT some reads — not gate everything (got {sampled})"
    );
}

#[test]
fn sample_decimation_is_deterministic() {
    let a = run_sample_graph(20);
    let b = run_sample_graph(20);
    assert_eq!(
        a, b,
        "sample decimation count must be bit-identical across runs (keyed off wire \
         timestamp, not wall-clock — Principle #7)"
    );
    assert!(a > 0, "decimation actually happened");
}
