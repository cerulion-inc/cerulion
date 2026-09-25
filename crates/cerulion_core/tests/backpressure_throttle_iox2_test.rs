// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end node-level `throttle_ms` producer rate cap.
//!
//! Proves the scheduler pre-fire time-gate: a data-triggered node fed by a
//! faster upstream fires no more than once per N ms — its fire count is
//! capped well below the upstream's, deterministically. `throttle_ms` is a
//! node-level execution policy (a producer rate cap), distinct from the
//! input-level `sample(N)` decimation. Zero-copy: the gate just defers the
//! tick (reading the scheduler's canonical last_fire_ns); no data is buffered.

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// Upstream: publishes one Vector3 every 5 ms.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct Upstream {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl Upstream {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Throttled relay: data-triggered on its input (so it would fire every step
/// the upstream publishes), but `throttle_ms = 15` caps it to one fire per
/// 15 ms. Stacks with the data trigger (mutually exclusive only with
/// `period_ms`).
#[cerulion_node(throttle_ms = 15)]
#[derive(Default)]
struct ThrottledRelay {
    #[input(trigger)]
    inp: Vector3,
    fires: u32,
}

#[cerulion_node_impl]
impl ThrottledRelay {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        self.fires += 1;
        Ok(())
    }
}

fn throttle_graph() -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "throttle_test".to_string(),
        prefix: "tp".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "upstream".to_string(),
                node_type: "upstream".to_string(),
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
                id: "relay".to_string(),
                node_type: "throttled_relay".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "upstream/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("upstream".to_string(), Box::new(UpstreamEntry::new()));
    factories.insert("relay".to_string(), Box::new(ThrottledRelayEntry::new()));
    (config, factories)
}

/// Run for `steps` 5ms steps; return (upstream_fires, relay_fires).
fn run_throttle_graph(steps: usize) -> (u64, u64) {
    let (config, factories) = throttle_graph();
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build throttle graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(5));
    }
    let upstream_fires = runtime.node_handle("upstream").unwrap().fire_count();
    let relay_fires = runtime.node_handle("relay").unwrap().fire_count();
    (upstream_fires, relay_fires)
}

#[test]
fn throttle_caps_producer_fire_rate() {
    // Upstream fires every 5ms (12 steps → 12 fires). The relay is
    // data-triggered (would fire each step) but throttle_ms=15 caps it to one
    // fire per 15ms → far fewer fires than upstream.
    let (upstream_fires, relay_fires) = run_throttle_graph(12);
    assert_eq!(upstream_fires, 12, "upstream fires every 5ms step");
    assert!(
        relay_fires < upstream_fires,
        "throttle_ms=15 must cap the relay below the 5ms upstream rate (relay={relay_fires}, upstream={upstream_fires})"
    );
    // One fire per 15ms over a ~60ms window ≈ 4 fires (t=5,20,35,50).
    assert!(
        (3..=5).contains(&relay_fires),
        "relay should fire ~once per 15ms (got {relay_fires})"
    );
}

#[test]
fn throttle_is_deterministic() {
    let a = run_throttle_graph(20);
    let b = run_throttle_graph(20);
    assert_eq!(
        a, b,
        "throttle fire counts must be bit-identical across runs (Principle #7)"
    );
}
