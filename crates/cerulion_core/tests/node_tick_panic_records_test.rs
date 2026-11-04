// SPDX-License-Identifier: AGPL-3.0-only
//! `GraphRuntime::node_tick_panic_records()` reads a graph's contained tick
//! panics from outside the runtime.
//!
//! An embedder that moves the runtime into its own thread (to run the graph
//! live) can no longer call `node_tick_panics()`. It takes this handle first
//! and reads it while the graph runs and after the runtime is gone. Through
//! `GraphRuntime::build_for_test` (per-test SHM root, parallel-safe): two
//! period producers panic at a chosen tick, a third stays healthy. The
//! expected records are hand-written from the producers' panic messages; the
//! determinism test compares two runs AND pins the hand oracle.

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::{GraphRuntime, NodeTickPanicRecords};
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// Publishes its tick number in `out.x`; panics on tick `panic_at`, and
/// from tick `error_at` on returns an error instead, `Fatal` when `fatal`
/// (0 = never).
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct Producer {
    #[output]
    out: Vector3,
    n: u32,
    panic_at: u32,
    error_at: u32,
    fatal: bool,
}
#[cerulion_node_impl]
impl Producer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        if self.panic_at != 0 && self.n >= self.panic_at {
            panic!("encoder lost at tick {}", self.n);
        }
        if self.error_at != 0 && self.n >= self.error_at {
            let message = format!("encoder lost at tick {}", self.n);
            return Err(if self.fatal {
                NodeError::Fatal(message)
            } else {
                NodeError::Logic(message)
            });
        }
        self.out.x = self.n as f64;
        Ok(())
    }
}

fn producer(id: &str) -> NodeDef {
    NodeDef {
        params: Default::default(),
        fuse: None,
        ros2: None,
        id: id.to_string(),
        node_type: "Producer".to_string(),
        inputs: Vec::new(),
        outputs: vec![OutputDef {
            name: "out".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }],
    }
}

/// A graph of producers `(id, panic_at)`, in this order.
fn build(producers: &[(&str, u32)]) -> GraphRuntime {
    build_with(
        producers
            .iter()
            .map(|(id, panic_at)| {
                (
                    *id,
                    Producer {
                        panic_at: *panic_at,
                        ..Default::default()
                    },
                )
            })
            .collect(),
    )
}

/// A graph of producers `(id, state)`, in this order.
fn build_with(producers: Vec<(&str, Producer)>) -> GraphRuntime {
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    let mut nodes = Vec::new();
    for (id, state) in producers {
        nodes.push(producer(id));
        factories.insert(id.to_string(), Box::new(ProducerEntry::with_state(state)));
    }
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "node_tick_panic_records_test".to_string(),
        prefix: "ntpr".to_string(),
        nodes,
    };
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 4096).expect("build the panic graph")
}

fn step(rt: &mut GraphRuntime, steps: usize) {
    for _ in 0..steps {
        rt.step(Duration::from_millis(5));
    }
}

fn records(handle: &NodeTickPanicRecords) -> Vec<(String, String)> {
    handle.snapshot()
}

fn owned(pairs: &[(&str, &str)]) -> Vec<(String, String)> {
    pairs
        .iter()
        .map(|(id, reason)| ((*id).to_string(), (*reason).to_string()))
        .collect()
}

#[test]
fn a_handle_taken_before_the_run_reads_each_panic_as_it_is_recorded() {
    let mut rt = build(&[("left", 3), ("steady", 0), ("right", 5)]);
    let handle = rt.node_tick_panic_records();
    assert!(records(&handle).is_empty(), "no tick ran yet");

    // Step 3 runs left's panicking tick; right is still healthy.
    step(&mut rt, 3);
    assert_eq!(
        records(&handle),
        owned(&[("left", "encoder lost at tick 3")])
    );

    // Step 5 runs right's. Records come in graph order, the first message
    // only: left stays poisoned-dead and adds nothing.
    step(&mut rt, 7);
    let expected = owned(&[
        ("left", "encoder lost at tick 3"),
        ("right", "encoder lost at tick 5"),
    ]);
    assert_eq!(records(&handle), expected);
    assert_eq!(
        rt.node_tick_panics(),
        expected,
        "the runtime reads the same"
    );
}

#[test]
fn records_list_in_graph_order_not_in_panic_or_name_order() {
    // Graph order is zulu, steady, alpha: neither the panic order (alpha
    // first) nor the name order (alpha first).
    let mut rt = build(&[("zulu", 5), ("steady", 0), ("alpha", 3)]);
    let handle = rt.node_tick_panic_records();
    step(&mut rt, 3);
    assert_eq!(
        records(&handle),
        owned(&[("alpha", "encoder lost at tick 3")])
    );
    step(&mut rt, 2);
    assert_eq!(
        records(&handle),
        owned(&[
            ("zulu", "encoder lost at tick 5"),
            ("alpha", "encoder lost at tick 3"),
        ])
    );
}

#[test]
fn a_tick_that_returns_an_error_is_not_recorded_as_a_panic() {
    let failing = |fatal| Producer {
        error_at: 2,
        fatal,
        ..Default::default()
    };
    let mut rt = build_with(vec![("logic", failing(false)), ("fatal", failing(true))]);
    let handle = rt.node_tick_panic_records();
    step(&mut rt, 5);
    assert!(records(&handle).is_empty(), "{:?}", records(&handle));
    assert_eq!(
        rt.node_fatal_errors(),
        owned(&[("fatal", "Fatal error: encoder lost at tick 2")]),
        "the fatal error is reported apart from the panics"
    );
}

#[test]
fn a_handle_keeps_reading_after_the_runtime_is_dropped() {
    let mut rt = build(&[("left", 2)]);
    let handle = rt.node_tick_panic_records();
    step(&mut rt, 4);
    drop(rt);
    assert_eq!(
        records(&handle),
        owned(&[("left", "encoder lost at tick 2")])
    );
}

#[test]
fn a_graph_with_no_panics_reads_empty() {
    let mut rt = build(&[("steady", 0)]);
    let handle = rt.node_tick_panic_records();
    step(&mut rt, 10);
    assert!(records(&handle).is_empty());
    assert!(rt.node_tick_panics().is_empty());
}

#[test]
fn two_handles_and_two_runs_read_the_same_records() {
    let run = || {
        let mut rt = build(&[("left", 3), ("right", 4)]);
        let first = rt.node_tick_panic_records();
        let second = first.clone();
        step(&mut rt, 6);
        assert_eq!(records(&first), records(&second));
        records(&first)
    };
    let one = run();
    let two = run();
    assert_eq!(one, two, "two runs record the same panics");
    assert_eq!(
        one,
        owned(&[
            ("left", "encoder lost at tick 3"),
            ("right", "encoder lost at tick 4"),
        ])
    );
}
