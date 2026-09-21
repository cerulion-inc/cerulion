// SPDX-License-Identifier: AGPL-3.0-only
//! Pins the live loop's busy-spin guard.
//!
//! The live loop sleeps a heartbeat ONLY when the WaitSet did not
//! actually block (no event sources, or every `attach_notification` failed) —
//! otherwise it would busy-spin at 100% CPU. The decision hinges on
//! `WaitSetReactor::last_wait_blocked()`, which `run_once` must set to `true`
//! ONLY AFTER it has at least one successful attachment and enters the blocking
//! wait. If `run_once` short-circuits on `guards.is_empty()` (all attaches
//! failed) it must leave `last_wait_blocked = false`.
//!
//! `run_waitset_reactor_once_blocked_for_test(timeout, force_attach_failure)`
//! exposes that bool. With `force_attach_failure = true` every attach is
//! skipped → `guards.is_empty()` → the reactor returns instantly with
//! `last_wait_blocked = false`. With `false` the real attach succeeds and the
//! reactor blocks for the timeout → `last_wait_blocked = true`.
//!
//! The LOAD-BEARING assertion is the FIRST (all-attach-fail → `false`): the
//! 25a fix is that `run_once` short-circuits on `guards.is_empty()` BEFORE
//! setting `last_wait_blocked = true`. This kills a mutation that sets the flag
//! unconditionally `true` (which would make the live loop busy-spin on a
//! permanently-failing attach). The SECOND assertion (real attach → `true`)
//! kills the opposite mutation that sets it unconditionally `false` (which
//! would make the live loop sleep the full heartbeat on EVERY iteration,
//! ignoring real wakeups).
//!
//! Both probes run on ONE graph in declaration order (forced first, normal
//! second) to avoid any state leak between them; the seam resets the
//! force-attach toggle before returning, so the second probe sees a clean
//! attach path.
//!
//! `#[serial]` — builds an iceoryx2 WaitSet over the process-global
//! shared-memory singleton; per-test isolated SHM root via `build_for_test`.

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// Period producer: publishes one Vector3 onto `out` per tick, so the consumer
/// has a real in-graph data-trigger source (non-empty `data_trigger_bindings`).
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct BsProducer {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl BsProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Data-trigger consumer: gives the graph one data-trigger binding so the
/// reactor has a real listener to attach.
#[cerulion_node]
#[derive(Default)]
struct BsConsumer {
    #[input(trigger)]
    inp: Vector3,
    sum: f64,
}

#[cerulion_node_impl]
impl BsConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.sum += self.inp.x;
        Ok(())
    }
}

/// Build `producer.out → consumer.inp` (one data-trigger binding) over an
/// isolated per-test transport.
fn build() -> GraphRuntime {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "waitset_busyspin_test".to_string(),
        prefix: "bs".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "producer".to_string(),
                node_type: "bs_producer".to_string(),
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
                node_type: "bs_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(BsProducerEntry::new()));
    factories.insert("consumer".to_string(), Box::new(BsConsumerEntry::new()));
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 8).expect("build busy-spin graph")
}

#[test]
#[serial]
fn busy_spin_guard_blocked_flag_tracks_attach_outcome() {
    let mut runtime = build();

    // A short timeout: the normal (non-forced) probe BLOCKS for the full
    // timeout, so keep it small. The forced probe short-circuits and ignores
    // the timeout entirely.
    let timeout = Duration::from_millis(5);

    // Probe 1 (LOAD-BEARING): force every attach to fail → `guards.is_empty()`
    // → `run_once` short-circuits BEFORE setting `last_wait_blocked = true`, so
    // the seam returns `false`. A mutation that sets the flag unconditionally
    // `true` (busy-spin on permanently-failing attach) makes this fail.
    let blocked_forced = runtime.run_waitset_reactor_once_blocked_for_test(timeout, true);
    assert!(
        !blocked_forced,
        "all-attach-fail must leave last_wait_blocked = false (the 25a busy-spin \
         guard short-circuits on guards.is_empty() before setting the flag)"
    );

    // Probe 2: a real attach succeeds and the reactor blocks for the timeout →
    // `last_wait_blocked = true`. The seam reset the force-attach toggle before
    // returning from probe 1, so this sees a clean attach path. A mutation that
    // sets the flag unconditionally `false` (live loop sleeps the heartbeat on
    // EVERY iteration, ignoring real wakeups) makes this fail.
    let blocked_normal = runtime.run_waitset_reactor_once_blocked_for_test(timeout, false);
    assert!(
        blocked_normal,
        "a real attach must set last_wait_blocked = true (the reactor entered \
         the blocking wait after a successful attachment)"
    );
}
