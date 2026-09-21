// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end test that the struct macro's
//! injected `self.virt_ns` and
//! `self.request_shutdown()` shim methods actually reach the runtime
//! clock + shared shutdown signal.
//!
//! - Spins up a real `GraphRuntime` with a `VirtualClock`.
//! - Captures `self.virt_ns().expect(...)` per tick (test runs under
//!   VirtualClock so `Some(_)` is always present).
//! - Trips `self.request_shutdown()` on the third tick.
//! - Drives the graph via `run_until_shutdown` and asserts the
//!   captured ns values match the simulated clock's advances and the
//!   runtime exits exactly when the shim is called.
//!
//! Why `virt_ns` not `real_ns`: this test asserts deterministic
//! 1_000_000-ns increments, which only the simulated clock provides.
//! `real_ns()` would return CLOCK_MONOTONIC and the assertions would
//! be timing-dependent.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// Shared between the test and the node so we can assert on what the
/// node observed without poking at private macro innards.
#[derive(Debug)]
struct ObservedTimeline {
    now_ns_per_tick: Mutex<Vec<u64>>,
    requested_shutdown_at_tick: Mutex<Option<usize>>,
}

impl ObservedTimeline {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            now_ns_per_tick: Mutex::new(Vec::new()),
            requested_shutdown_at_tick: Mutex::new(None),
        })
    }
}

// `static OBSERVED` is the simplest way to share state with the
// macro-generated node without forcing the macro to surface a custom
// constructor signature. The struct macro auto-derives Default; user
// state can poke at the static directly inside `tick`.
static OBSERVED: std::sync::OnceLock<Arc<ObservedTimeline>> = std::sync::OnceLock::new();

#[cerulion_node(period_ms = 1)]
struct ShimSink {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl ShimSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let observed = OBSERVED
            .get()
            .expect("OBSERVED set by test before runtime build");

        // Macro-injected `self.virt_ns()` reaches the runtime clock
        // (which is `VirtualClock` under this test).
        let t = self
            .virt_ns()
            .expect("test wires VirtualClock; virt_ns must be Some");
        let mut times = observed.now_ns_per_tick.lock().unwrap();
        times.push(t);
        let tick_idx = times.len();
        drop(times);

        // Write a marker into the loaned proxy so the per-tick scope is
        // exercised end-to-end (not strictly needed for the assertions).
        self.out.x = t as f64;

        if tick_idx == 3 {
            *observed.requested_shutdown_at_tick.lock().unwrap() = Some(tick_idx);
            // Macro-injected `self.request_shutdown()` reaches the
            // graph-wide ShutdownSignal.
            self.request_shutdown();
        }
        Ok(())
    }
}

#[test]
fn shim_methods_reach_runtime_clock_and_shutdown_signal() {
    let observed = ObservedTimeline::new();
    OBSERVED
        .set(observed.clone())
        .expect("OBSERVED set exactly once");

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "shim_methods".to_string(),
        prefix: "shim_methods".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "sink".to_string(),
            node_type: "shim_sink".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "geometry_msgs/Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("sink".to_string(), Box::new(ShimSinkEntry::new()));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock.clone(), 4).expect("build_for_test");

    // Each step advances simulated time by 1ms = 1_000_000 ns. The Period
    // trigger fires once per ms, so per-tick `now_ns()` should observe
    // the cumulative advance after each step.
    let steps = runtime.run_until_shutdown(Duration::from_millis(1), Some(64));

    // The sink trips request_shutdown on its third tick; the runtime's
    // outer loop checks the signal *before* each step, so it returns
    // exactly `steps` once it sees the signal go true.
    assert!(
        runtime.shutdown_requested(),
        "runtime must exit because the shim flipped the signal"
    );
    assert_eq!(
        *observed.requested_shutdown_at_tick.lock().unwrap(),
        Some(3),
        "node body should record that it requested shutdown on tick 3"
    );

    let times = observed.now_ns_per_tick.lock().unwrap();
    assert_eq!(
        times.len(),
        3,
        "exactly three ticks should have run before shutdown propagated"
    );
    // Every captured time should be strictly increasing because the
    // simulated clock advanced by 1_000_000 ns each step.
    for w in times.windows(2) {
        assert!(
            w[1] > w[0],
            "now_ns must strictly increase between ticks; got {:?}",
            w
        );
    }
    // The first tick observes the clock AFTER the first 1ms advance, so
    // it should be exactly 1_000_000 ns. (`step()` advances before
    // dispatching the period.)
    assert_eq!(
        times[0], 1_000_000,
        "first tick should see the clock at exactly 1ms (1_000_000 ns)"
    );
    assert_eq!(
        steps, 3,
        "outer loop should exit immediately after the trip"
    );
}
