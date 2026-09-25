// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end `tick_within_ms` node-level tick-budget
//! counter over real iceoryx2 via `GraphRuntime::build_for_test` (per-test SHM
//! root — parallel-safe).
//!
//! The scheduler measures the
//! tick in `fire_node` and bumps `tick_within_missed`, and `GraphRuntime`
//! calls `set_tick_within` from `NodeInfo.tick_within_ms`; without that call the
//! counter is permanently 0 on real graphs. This proves it fires e2e.
//!
//! COUNTER-ONLY (no reactable event): the per-fire measurement keys off a
//! wall-clock `Instant`, so the counter VALUE is not replay-deterministic.
//! We therefore assert only LIVENESS (`> 0` when the budget is blown, `== 0`
//! when it is not) — never a bit-identical value across runs. `tick_within_ms`
//! graduates to a deterministic reactable event only with the
//! record-execution-time clock model; until then it is a
//! counter+warn observability surface.

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// Tick deliberately sleeps 5 ms — far over the 1 ms budget → every fire is a
/// miss.
#[cerulion_node(period_ms = 10, tick_within_ms = 1)]
#[derive(Default)]
struct SlowTickNode {
    #[output]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl SlowTickNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        std::thread::sleep(Duration::from_millis(5));
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Tick returns immediately — comfortably inside the 500 ms budget. The
/// budget is deliberately ENORMOUS relative to the 2-assign tick: the quiet
/// arm pins that the counter does not false-fire, which is budget-independent
/// (a wiring/units bug blows ANY budget), while a routine 100 ms CI-runner
/// preemption inside one tick can no longer flake the `== 0` assert — only a
/// half-second single-tick stall can (the accepted rarity class).
#[cerulion_node(period_ms = 10, tick_within_ms = 500)]
#[derive(Default)]
struct FastTickNode {
    #[output]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl FastTickNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

fn tick_graph(node_type: &str) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "tick_within_test".to_string(),
        prefix: "tw".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "node".to_string(),
            node_type: node_type.to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    let node: Box<dyn NodeEntry> = match node_type {
        "slow" => Box::new(SlowTickNodeEntry::new()),
        "fast" => Box::new(FastTickNodeEntry::new()),
        other => panic!("unknown node type {other}"),
    };
    // `build_for_test` keys the factory map by NODE ID (not node_type).
    factories.insert("node".to_string(), node);
    (config, factories)
}

/// Run for `steps` 10 ms steps; return the node's `tick_within_missed_count`.
fn run_tick_within(node_type: &str, steps: usize) -> u64 {
    let (config, factories) = tick_graph(node_type);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build tick graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(10));
    }
    runtime
        .node_handle("node")
        .unwrap()
        .tick_within_missed_count()
}

#[test]
fn tick_within_counter_fires_when_budget_blown() {
    // 5 ms tick vs a 1 ms budget → every fire over ~5 fires misses. Assert
    // LIVENESS only (wall-clock measurement — no exact / deterministic count).
    let misses = run_tick_within("slow", 5);
    assert!(
        misses > 0,
        "a 5 ms tick must trip the 1 ms tick_within budget at least once (got {misses})"
    );
}

#[test]
fn tick_within_counter_quiet_when_inside_budget() {
    // Fast tick vs a 500 ms budget → no miss (budget sized so only a 500 ms+
    // single-tick VM stall can false-fail — see FastTickNode's doc).
    let misses = run_tick_within("fast", 5);
    assert_eq!(
        misses, 0,
        "a sub-millisecond tick must not trip a 500 ms budget (got {misses})"
    );
}
