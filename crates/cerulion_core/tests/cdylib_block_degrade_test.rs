// SPDX-License-Identifier: AGPL-3.0-only
//! Cdylib parity for a `block` input on a MIXED topic
//! (`backpressure_block_iox2_test.rs`'s
//! `mixed_block_dropoldest_block_consumer_still_defers`), through the LOADED
//! `test_node_macro_depth_cdylib` fixture.
//!
//! The fixture's `inp` field is `#[input(backpressure = block, depth = 32)]`,
//! `external` + `HostDriven`, and never triggered here. An in-process
//! `drop_oldest` sibling reads the same topic, so the topic is mixed.
//!
//! # Topology
//!
//! `DegradeBlockProducer` (in-process, `period_ms = 10`) publishes `out`
//! (shared by BOTH consumers) and `aux_out` (the depth cdylib's `aux` input).
//!
//! - `depth_consumer` (the cdylib): `inp` <- `producer/out` (block, depth
//!   32), `aux` <- `producer/aux_out` (sample(7)). NEVER triggered.
//! - `dropper` (in-process): `inp` <- `producer/out` (drop_oldest, depth 2).
//!
//! # Oracle
//!
//! The cdylib's block input holds its own credit slot whatever the sibling
//! declares: the producer fires exactly 32 times (the depth) and every later
//! step is a block defer on the cdylib consumer's `inp`.
//!
//! # Build requirement + serial
//!
//! Requires `cargo build -p test_node_macro_depth_cdylib`. `#[serial]`
//! (cdylib `NODES` + iceoryx2 SHM singletons).

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// Producer period; also the step delta. The producer would fire once per
/// step unpressured.
const STEPS: usize = 50;
/// The fixture's declared depth on `inp`.
const DEPTH: u64 = 32;

fn find_cdylib(crate_name: &str) -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib(crate_name)
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

/// Period producer publishing on TWO outputs: `out` (shared by both
/// consumers, the mixed topic under test) and `aux_out` (the depth cdylib's
/// `aux` input, kept on its own topic — crib:
/// `cdylib_depth_ffi_test.rs::DepthFeedProducer`).
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct DegradeBlockProducer {
    #[output]
    out: Vector3,
    #[output]
    aux_out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl DegradeBlockProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        self.aux_out.x = self.n as f64;
        Ok(())
    }
}

/// A stalled drop_oldest consumer sharing the `out` topic with the depth
/// cdylib's block `inp` — its mere presence makes the topic MIXED (crib:
/// `backpressure_block_iox2_test.rs::DropOldestConsumer`).
#[cerulion_node(external)]
#[derive(Default)]
struct DropOldestSibling {
    #[input(backpressure = drop_oldest, depth = 2)]
    inp: Vector3,
}
#[cerulion_node_impl]
impl DropOldestSibling {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// Run the mixed-topic graph for `STEPS` 10ms steps (the producer's own
/// period) WITHOUT ever triggering either consumer — the degrade is a
/// build-time topology property, not a runtime draining behavior.
/// Returns (producer_fire_count, depth_consumer_block_deferred_count).
fn run_degrade() -> (u64, u64) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cbd_degrade".to_string(),
        prefix: "cbd".to_string(),
        nodes: vec![
            NodeDef {
                isolate: None,
                params: Default::default(),
                ros2: None,
                id: "producer".to_string(),
                node_type: "degrade_block_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out"), vec3_out("aux_out")],
            },
            NodeDef {
                isolate: None,
                params: Default::default(),
                ros2: None,
                id: "depth_consumer".to_string(),
                node_type: "depth_probe".to_string(),
                inputs: vec![
                    InputDef {
                        name: "inp".to_string(),
                        source: "producer/out".to_string(),
                        ..Default::default()
                    },
                    InputDef {
                        name: "aux".to_string(),
                        source: "producer/aux_out".to_string(),
                        ..Default::default()
                    },
                ],
                outputs: vec![],
            },
            NodeDef {
                isolate: None,
                params: Default::default(),
                ros2: None,
                id: "dropper".to_string(),
                node_type: "drop_oldest_sibling".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                    ..Default::default()
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(DegradeBlockProducerEntry::new()),
    );
    factories.insert(
        "depth_consumer".to_string(),
        Box::new(
            DylibNodeEntry::load(&find_cdylib("test_node_macro_depth_cdylib"))
                .expect("load depth fixture"),
        ),
    );
    factories.insert(
        "dropper".to_string(),
        Box::new(DropOldestSiblingEntry::new()),
    );

    let clock = Arc::new(VirtualClock::new());
    // Buffer large enough that neither never-drained consumer overflows over
    // the run — the point under test is the PRODUCER's rate, not eviction.
    let mut rt =
        GraphRuntime::build_for_test(config, factories, clock, 64).expect("build degrade graph");
    for _ in 0..STEPS {
        rt.step(Duration::from_millis(10));
    }
    let producer_fires = rt.node_handle("producer").unwrap().fire_count();
    let deferred = rt
        .node_handle("depth_consumer")
        .unwrap()
        .backpressure_block_fires_deferred_count("inp");
    (producer_fires, deferred)
}

#[test]
#[serial]
fn dylib_block_input_on_a_mixed_topic_defers_the_producer() {
    let (producer_fires, deferred) = run_degrade();
    assert_eq!(
        producer_fires, DEPTH,
        "the dylib's block input defers the producer at its depth on a mixed topic \
         got {producer_fires}"
    );
    assert_eq!(
        deferred,
        STEPS as u64 - DEPTH,
        "every step after the slot filled is a block defer (got {deferred})"
    );
}

/// DETERMINISM (Principle #7): two runs byte-identical.
#[cfg_attr(
    target_os = "macos",
    ignore = "upstream iceoryx2 0.10.0 defect 2034: on macOS a process that has run one graph containing a plugin node cannot create any further iceoryx2 resource, so this arm cannot run here until that is fixed. Runs normally on Linux."
)]
#[test]
#[serial]
fn dylib_block_on_a_mixed_topic_is_deterministic() {
    let a = run_degrade();
    let b = run_degrade();
    assert_eq!(
        a, b,
        "two runs of the mixed-topic graph must be byte-identical \
         (VirtualClock, no wall-clock). a={a:?} b={b:?}"
    );
    assert_eq!(a.0, DEPTH, "the deterministic run plateaus at the depth");
}
