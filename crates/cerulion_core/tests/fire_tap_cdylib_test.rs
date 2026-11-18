//! The fire tap sees what a cdylib node wrote. The host keeps a loaded node's
//! context, so `DylibNodeEntry` forwards its output commit counts and its
//! fires carry the outputs they wrote instead of `FireFlags::WROTE_UNKNOWN`.
//!
//! Requires the fixture built: `cargo build -p test_node_macro_period_cdylib`.

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::fire_tap::{Cursor, FireFlags, FireTapConfig, FIRE_SERVE_MAX};
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use indexmap::IndexMap;
use serial_test::serial;

fn period_cdylib_graph() -> GraphRuntime {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ftcd".to_string(),
        prefix: "ftcd".to_string(),
        nodes: vec![NodeDef {
            isolate: None,
            params: Default::default(),
            ros2: None,
            id: "node".to_string(),
            node_type: "period_node".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "cmd".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_macro_period_cdylib");
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "node".to_string(),
        Box::new(DylibNodeEntry::load(&path).expect("load fixture")),
    );
    GraphRuntime::build_for_test(config, factories, Arc::new(VirtualClock::new()), 8)
        .expect("build single-node graph")
}

#[test]
#[serial]
fn a_cdylib_node_reports_the_outputs_its_fires_wrote() {
    let mut runtime = period_cdylib_graph();
    let tap = runtime.install_fire_tap(FireTapConfig {
        capacity: 64,
        lease: Duration::from_secs(5),
    });
    assert!(
        runtime.run_topology().nodes[0].reports_writes,
        "a loaded cdylib node reports its writes"
    );
    assert!(tap.serve(Cursor::After(0), 0).records.is_empty());
    for _ in 0..30 {
        runtime.step(Duration::from_millis(10));
    }
    let batch = tap.serve(Cursor::After(0), FIRE_SERVE_MAX);
    assert!(!batch.records.is_empty(), "the period node fired");
    for record in &batch.records {
        assert_eq!(
            record.flags.0 & FireFlags::WROTE_UNKNOWN,
            0,
            "a cdylib fire names what it wrote: {record:?}"
        );
    }
}
