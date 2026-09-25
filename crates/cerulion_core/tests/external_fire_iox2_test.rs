// SPDX-License-Identifier: AGPL-3.0-only
//! An `External`-policy node FIRES via `GraphRuntime::trigger_external`.
//!
//! `External` nodes are host-driven — nothing in the graph fires them. Before
//! the `trigger_external` pass-through there was NO way for a host to fire an
//! External node through `GraphRuntime` (the scheduler's `trigger_external` was
//! unreachable from outside the crate), so External nodes never fired in
//! production. `test_external_fires_on_trigger` pins fire / re-arm / re-fire.
//!
//! `#[serial]` — real iceoryx2 over the process-global SHM singleton;
//! per-test SHM root via `build_for_test`.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// A host-triggered External node: increments a shared counter each fire.
#[cerulion_node(external)]
#[derive(Default)]
struct ExtNode {
    #[output]
    tick_out: Vector3,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl ExtNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

fn ext_graph(fires: Arc<AtomicU64>) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ext_fire_test".to_string(),
        prefix: "extf".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "ext".to_string(),
            node_type: "ext_node".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "tick_out".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "ext".to_string(),
        Box::new(ExtNodeEntry::with_state(ExtNode {
            fires,
            ..Default::default()
        })),
    );
    (config, factories)
}

/// fire / re-arm / re-fire: `trigger_external` fires the node exactly once on
/// the next step; an un-triggered step does NOT re-fire; a second trigger fires
/// again. The first fire was UNREACHABLE before the pass-through (the bug).
#[test]
#[serial]
fn test_external_fires_on_trigger() {
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = ext_graph(Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build ext graph");

    // No trigger → no fire.
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        fires.load(Ordering::Relaxed),
        0,
        "External node must not fire without a trigger"
    );

    // Trigger → next step fires once.
    runtime.trigger_external("ext").expect("trigger_external");
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        fires.load(Ordering::Relaxed),
        1,
        "External node fires once after trigger_external (UNREACHABLE before the pass-through)"
    );

    // Re-arm: a step WITHOUT a fresh trigger does NOT re-fire.
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        fires.load(Ordering::Relaxed),
        1,
        "External node must NOT re-fire without a fresh trigger (external_triggered reset on fire)"
    );

    // A second trigger fires again.
    runtime.trigger_external("ext").expect("trigger_external 2");
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        fires.load(Ordering::Relaxed),
        2,
        "a second trigger_external fires the node again"
    );
}

/// `trigger_external` on an unknown node id returns an Err (does not panic).
#[test]
#[serial]
fn test_trigger_external_unknown_node_errs() {
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = ext_graph(Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build ext graph");

    assert!(
        runtime.trigger_external("does_not_exist").is_err(),
        "trigger_external on an unknown node must return Err, not panic"
    );
}
