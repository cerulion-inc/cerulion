// SPDX-License-Identifier: AGPL-3.0-only
//! Cdylib-parity third pass — bp-block-degrade: the MIXED-topic block-degrade
//! contract (`backpressure_block_iox2_test.rs`'s
//! `mixed_block_dropoldest_producer_not_deferred`), proven through the
//! LOADED `test_node_macro_depth_cdylib` fixture.
//!
//! No NEW fixture: `test_node_macro_depth_cdylib`'s `inp` field
//! (`#[input(backpressure = block, depth = 32)]`, `external` + `HostDriven`,
//! never triggered by `cdylib_depth_ffi_test.rs`'s plateau test) is the
//! natural candidate — reused as-is. The degrade contract is purely a
//! BUILD-TIME topology property (`GraphTopology::build`'s per-topic
//! `is_all_block()` check across ALL declared consumer backpressure
//! policies), so neither consumer needs to ever be triggered — mirroring the
//! in-process crib, which never calls `trigger_external` on either sibling.
//!
//! # Topology
//!
//! `DegradeBlockProducer` (in-process, `period_ms = 10`) publishes `out`
//! (shared by BOTH consumers — the mixedness under test) and `aux_out`
//! (dedicated, so the depth cdylib's `aux` input keeps its own non-mixed
//! topic, matching `cdylib_depth_ffi_test.rs`'s wiring convention).
//!
//! - `depth_consumer` (the cdylib): `inp` <- `producer/out` (block, depth
//!   32), `aux` <- `producer/aux_out` (sample(7)). NEVER triggered.
//! - `dropper` (in-process): `inp` <- `producer/out` (drop_oldest, depth 2).
//!   NEVER triggered. Its mere presence on the SAME topic as `depth_consumer`
//!   makes that topic mixed.
//!
//! # Oracle
//!
//! On a mixed topic the block consumer degrades (no defer edge installed):
//! the producer fires at its UN-pressured rate (exactly once per 10ms step,
//! matching `mixed_block_dropoldest_producer_not_deferred`'s pin) and the
//! degraded consumer's `backpressure_block_fires_deferred_count("inp")`
//! stays exactly 0.
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

/// Producer period; also the step delta, so `fire_count == STEPS` is the
/// un-pressured (never-deferred) oracle.
const STEPS: usize = 50;

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
                ros2: None,
                id: "producer".to_string(),
                node_type: "degrade_block_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out"), vec3_out("aux_out")],
            },
            NodeDef {
                ros2: None,
                id: "depth_consumer".to_string(),
                node_type: "depth_probe".to_string(),
                inputs: vec![
                    InputDef {
                        name: "inp".to_string(),
                        source: "producer/out".to_string(),
                    },
                    InputDef {
                        name: "aux".to_string(),
                        source: "producer/aux_out".to_string(),
                    },
                ],
                outputs: vec![],
            },
            NodeDef {
                ros2: None,
                id: "dropper".to_string(),
                node_type: "drop_oldest_sibling".to_string(),
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
fn dylib_block_input_degrades_on_mixed_topic_producer_not_deferred() {
    let (producer_fires, deferred) = run_degrade();
    assert_eq!(
        producer_fires, STEPS as u64,
        "producer must fire at the un-pressured rate on a mixed topic (the \
         dylib's block consumer degrades to drop_oldest, no defer edge \
         installed) — got {producer_fires}"
    );
    assert_eq!(
        deferred, 0,
        "the degraded (mixed-topic) dylib block consumer must observe ZERO \
         block defers — its declared block policy was downgraded across the \
         FFI just as it is in-process (got {deferred})"
    );
}

/// DETERMINISM (Principle #7): two runs byte-identical.
#[test]
#[serial]
fn dylib_block_degrade_is_deterministic() {
    let a = run_degrade();
    let b = run_degrade();
    assert_eq!(
        a, b,
        "two runs of the mixed-topic degrade must be byte-identical \
         (VirtualClock, no wall-clock). a={a:?} b={b:?}"
    );
    assert_eq!(
        a.0, STEPS as u64,
        "the deterministic run must un-pressured-fire"
    );
    assert_eq!(a.1, 0, "the deterministic run must show zero defers");
}
