// SPDX-License-Identifier: AGPL-3.0-only
//! The RUNTIME drives `pump_history` each live-loop iteration.
//!
//! 30.1 proved the publisher-level mechanism (`CerulionPublisher::pump_history`
//! delivers retained history to a quiescent publisher's late joiner) and 30.2
//! the cdylib FFI half. This test pins the integration GLUE that ties them to
//! the runtime: `GraphRuntime::live_step` must call `pump_history_all`, which
//! must call `NodeEntry::pump_history` on every node.
//!
//! It uses a spy `NodeEntry` whose `pump_history` bumps a counter, so it
//! catches a variant that removes the `pump_history_all()` call from
//! `live_step` — one the replay firewall tests can NOT catch (they only
//! assert `step`'s firing path is byte-identical, and the pump is additive
//! OUTSIDE `step`).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::error::TransportResult;
use cerulion_core::graph::config::{GraphConfig, NodeDef};
use cerulion_core::graph::node::{NodeContext, NodeEntry, NodeInfo};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::MacroPolicy;
use indexmap::IndexMap;
use serial_test::serial;

/// A minimal node entry that counts how many times the runtime drove its
/// `pump_history`. No inputs, no outputs — a long-period source that never
/// fires during the short live steps below, so the ONLY thing the test
/// observes is the runtime's per-iteration pump.
struct PumpSpy {
    pumped: Arc<AtomicU64>,
}

impl NodeEntry for PumpSpy {
    fn info(&self) -> TransportResult<NodeInfo> {
        // A long period so `step` never fires it during the test — we are
        // measuring the pump, not the firing path.
        Ok(NodeInfo::from_names(vec![], vec![])
            .with_policy(MacroPolicy::Period { period_ms: 100_000 }))
    }
    fn init(&mut self, _context: NodeContext) -> TransportResult<()> {
        Ok(())
    }
    fn tick(&mut self) -> TransportResult<()> {
        Ok(())
    }
    fn pump_history(&mut self) {
        self.pumped.fetch_add(1, Ordering::Relaxed);
    }
}

fn build_spy_graph() -> (GraphRuntime, Arc<AtomicU64>) {
    let pumped = Arc::new(AtomicU64::new(0));
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "pump_history_runtime_test".to_string(),
        prefix: "phr".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "spy".to_string(),
            node_type: "pump_spy".to_string(),
            inputs: vec![],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "spy".to_string(),
        Box::new(PumpSpy {
            pumped: Arc::clone(&pumped),
        }),
    );
    let clock = Arc::new(VirtualClock::new());
    let runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build pump-history runtime graph");
    (runtime, pumped)
}

/// One `live_step` must drive exactly one `pump_history` per node; N steps → N.
/// Removing the `pump_history_all()` call from `live_step` leaves this at 0.
#[test]
#[serial]
fn live_step_drives_pump_history_on_every_node_each_iteration() {
    let (mut runtime, pumped) = build_spy_graph();

    assert_eq!(
        pumped.load(Ordering::Relaxed),
        0,
        "no pump before any live step"
    );

    runtime.run_live_step_once_for_test(Duration::from_millis(20));
    assert_eq!(
        pumped.load(Ordering::Relaxed),
        1,
        "one live_step must drive exactly one pump_history on the node \
         (kills a 'pump_history_all removed from live_step' mutant)"
    );

    // Subsequent iterations keep pumping — the cadence is per-iteration.
    for expected in 2..=4 {
        runtime.run_live_step_once_for_test(Duration::from_millis(20));
        assert_eq!(
            pumped.load(Ordering::Relaxed),
            expected,
            "each live_step iteration must drive one more pump_history"
        );
    }
}

/// A node whose tick PANICS (caught by the scheduler) — a SHORT period so it
/// fires on the first `live_step`. The tick callback holds the entry mutex
/// across `tick()`, so a caught panic POISONS that mutex. `pump_history`
/// bumps a counter so the test can prove the pump still ran afterwards.
struct PanicTickSpy {
    pumped: Arc<AtomicU64>,
    ticked: Arc<AtomicU64>,
}

impl NodeEntry for PanicTickSpy {
    fn info(&self) -> TransportResult<NodeInfo> {
        Ok(NodeInfo::from_names(vec![], vec![]).with_policy(MacroPolicy::Period { period_ms: 1 }))
    }
    fn init(&mut self, _context: NodeContext) -> TransportResult<()> {
        Ok(())
    }
    fn tick(&mut self) -> TransportResult<()> {
        self.ticked.fetch_add(1, Ordering::Relaxed);
        panic!("PanicTickSpy: deliberate tick panic to poison the entry mutex");
    }
    fn pump_history(&mut self) {
        self.pumped.fetch_add(1, Ordering::Relaxed);
    }
}

/// After a tick panic poisons a node's entry mutex,
/// `pump_history_all` must RECOVER the poison and still pump — not skip the
/// node (which would also flood the live loop with a per-iteration
/// "poisoned; skipping" `error!`). Reverting the poison-recovery to a skip
/// is what this test catches: the spy's pump counter would stay 0.
#[test]
#[serial]
fn pump_history_recovers_after_a_tick_panic_poisons_the_entry_mutex() {
    let pumped = Arc::new(AtomicU64::new(0));
    let ticked = Arc::new(AtomicU64::new(0));
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "pump_history_poison_recovery_test".to_string(),
        prefix: "phpr".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "panic_spy".to_string(),
            node_type: "panic_tick_spy".to_string(),
            inputs: vec![],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "panic_spy".to_string(),
        Box::new(PanicTickSpy {
            pumped: Arc::clone(&pumped),
            ticked: Arc::clone(&ticked),
        }),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build poison-recovery graph");

    // One live_step with a >1ms timeout: the period-1ms node fires (tick panics
    // → caught by the scheduler → POISONS the entry mutex), then
    // `pump_history_all` runs over the now-poisoned node. The test thread
    // SURVIVES because the scheduler catches the tick panic.
    runtime.run_live_step_once_for_test(Duration::from_millis(5));

    assert!(
        ticked.load(Ordering::Relaxed) >= 1,
        "the period-1ms spy's tick must have fired (and panicked, poisoning its \
         entry mutex)"
    );
    assert!(
        pumped.load(Ordering::Relaxed) >= 1,
        "pump_history_all must RECOVER the poisoned entry mutex and still pump \
         the node (a 'skip poisoned node' regression would leave this at 0 — and \
         flood the live loop with a per-iteration poison error!)"
    );
}
