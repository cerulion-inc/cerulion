// SPDX-License-Identifier: AGPL-3.0-only
//! Group 4: `build_live` + `live_timeout_for_test` over
//! real iceoryx2.
//!
//! `GraphRuntime::build_live(config, factories, &transport, clock)` is the
//! production/RealClock builder (additive sibling of the simulated-clock
//! `build`). These tests construct a per-test isolated transport
//! (`init_for_test` over `iceoryx_test_config`, so the file is parallel-safe)
//! and build via `build_live(.., Arc::new(RealClock))` — the real production
//! time source — then assert:
//!
//! 1. `live_timeout_for_test` tracks the next Period deadline (~100ms),
//!    proving the live-loop WaitSet timeout is NOT pinned to the 250ms
//!    liveliness cap (which would be the symptom of `ns_until_next_fire`
//!    wrongly returning `None` for a Period graph) and NOT the 1ms floor.
//! 2. With NO Period node, `live_timeout_for_test` is the 250ms liveliness
//!    sweep cap.
//! 3. A `build_live` graph yields a FUNCTIONING live runtime: driving
//!    `run_live_step_once_for_test` fires the consumer over real iceoryx2,
//!    parity with the `build_for_test` live-loop test.
//!
//! All `#[serial]` — the live loop builds an iceoryx2 WaitSet over the
//! process-global shared-memory singleton. `build_live` does NOT park the
//! transport on the runtime (unlike `build_for_test`), so each helper RETURNS
//! the owning `Arc<TransportManager>` and the test holds it alive for the
//! runtime's lifetime.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::Clock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::RealClock;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// The absolute external topic the live consumer's trigger input subscribes —
/// no in-graph producer, so the graph provisions it as `External` and an
/// out-of-graph publisher attaches freely.
const EXT_TOPIC: &str = "/c25b/ext/cam";

/// A wake timeout for the data-flow test: long enough that a published event
/// wakes the reactor well before it elapses.
const WAKE_TIMEOUT: Duration = Duration::from_millis(150);

// --------------------------------------------------------------------------
// Node types.
// --------------------------------------------------------------------------

/// A lone 100ms period producer (no data inputs) — the Period-deadline lever
/// for `live_timeout` test 1.
#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct C25bPeriod {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl C25bPeriod {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// A data-trigger consumer of the absolute external topic — the no-Period
/// graph for `live_timeout` test 2 AND the data-flow consumer for test 3.
/// Counts its fires in a shared `Arc<AtomicU64>`.
#[cerulion_node]
#[derive(Default)]
struct C25bConsumer {
    #[input(trigger)]
    inp: Vector3,
    fires: Arc<AtomicU64>,
    sum: f64,
}

#[cerulion_node_impl]
impl C25bConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.sum += self.inp.x;
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

// --------------------------------------------------------------------------
// build_live harness over a per-test isolated transport.
// --------------------------------------------------------------------------

/// Construct a per-test isolated `TransportManager` whose clock is `RealClock`
/// (matching the runtime's `build_live` clock). Returned as an owned `Arc` so
/// the caller keeps the iceoryx2 node alive for the runtime's lifetime.
fn live_transport() -> Arc<TransportManager> {
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let transport_config = TransportConfig {
        node_name: "cerulion_c25b_live".into(),
        clock: Arc::new(RealClock) as Arc<dyn Clock>,
        subscriber_buffer_size: 8,
        network: None,
    };
    TransportManager::init_for_test(transport_config, ix_config)
        .expect("init per-test live transport")
}

/// Build a graph via `build_live` (RealClock) over `transport`. The caller
/// owns `transport` and must keep it alive while using the runtime.
fn build_live(
    transport: &TransportManager,
    config: GraphConfig,
    factories: IndexMap<String, Box<dyn NodeEntry>>,
) -> GraphRuntime {
    GraphRuntime::build_live(config, factories, transport, Arc::new(RealClock))
        .expect("build_live graph")
}

/// A lone 100ms-period-producer graph (no data inputs).
fn period_only_graph() -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "c25b_period_only".to_string(),
        prefix: "c25bp".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "producer".to_string(),
            node_type: "c25b_period".to_string(),
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
    factories.insert("producer".to_string(), Box::new(C25bPeriodEntry::new()));
    (config, factories)
}

/// A lone data-trigger consumer of the absolute external topic (NO Period node).
fn data_only_graph(fires: Arc<AtomicU64>) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "c25b_data_only".to_string(),
        prefix: "c25bd".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "consumer".to_string(),
            node_type: "c25b_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: EXT_TOPIC.to_string(),
            }],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "consumer".to_string(),
        Box::new(C25bConsumerEntry::with_state(C25bConsumer {
            fires,
            ..Default::default()
        })),
    );
    (config, factories)
}

// --------------------------------------------------------------------------
// Test 1: live_timeout tracks the Period deadline (~100ms), not the cap/floor.
// --------------------------------------------------------------------------
#[test]
#[serial]
fn build_live_period_timeout_tracks_period() {
    let transport = live_transport();
    let (config, factories) = period_only_graph();
    let runtime = build_live(&transport, config, factories);

    // next_fire = build_wall + 100ms; `live_timeout` = min(ns_until_next_fire,
    // 250ms).max(1ms). At call time the remaining is ~100ms minus the build→read
    // elapsed, so the timeout sits inside (20ms, 200ms): it tracks the ~100ms
    // Period deadline, NOT the 250ms liveliness cap (the symptom of
    // ns_until_next_fire returning None for a Period graph → timeout == 250ms,
    // failing `< 200ms`) and NOT the 1ms floor. The lower bound is deliberately
    // generous (20ms, well below the ~100ms deadline) so the build→read wall
    // window can stretch under concurrent CI load without flaking — the three
    // failure modes land at exactly 250ms (None) or 1ms (floor), both far from 20ms.
    let timeout = runtime.live_timeout_for_test();
    assert!(
        timeout > Duration::from_millis(20),
        "live_timeout {timeout:?} must be > 20ms (it tracks the ~100ms Period \
         deadline, not the 1ms floor)"
    );
    assert!(
        timeout < Duration::from_millis(200),
        "live_timeout {timeout:?} must be < 200ms (it tracks the ~100ms Period \
         deadline, NOT the 250ms liveliness cap — a regression where \
         ns_until_next_fire returns None for a Period graph would pin it to 250ms)"
    );

    // Keep the transport alive until here (drops with the runtime's ports).
    drop(runtime);
    drop(transport);
}

// --------------------------------------------------------------------------
// Test 1b: a Period node FIRES end-to-end through build_live + the live loop.
//
// Test 1 pins the live_timeout SIZING for a Period graph; G2 (scheduler_test)
// pins Period firing at the raw Scheduler::step layer; this composes them:
// a period_ms=100 producer built via `build_live` (RealClock) actually FIRES
// when driven through `run_live_step_once_for_test` (the production live path),
// with NO data input. The graph has no data-trigger bindings, so each
// `run_live_step_once_for_test` sleeps its timeout then steps (advancing real
// wall time); once >100ms have elapsed since build the period deadline
// (build_wall + 100ms) is reached and the producer fires. This is the headline
// "a period_ms node fires on time under the live loop" behavior, pinned e2e.
// --------------------------------------------------------------------------
#[test]
#[serial]
fn build_live_period_node_fires_through_run_live() {
    let transport = live_transport();
    let (config, factories) = period_only_graph();
    let mut runtime = build_live(&transport, config, factories);

    // Loop until the producer fires OR a generous 500ms wall budget expires
    // (the 100ms period fires within ~100-160ms; 500ms is ~3-5x margin, so no
    // flake under concurrent CI load). Each iteration waits up to 40ms.
    let budget_end = std::time::Instant::now() + Duration::from_millis(500);
    while runtime
        .node_handle("producer")
        .expect("producer handle")
        .fire_count()
        == 0
        && std::time::Instant::now() < budget_end
    {
        runtime.run_live_step_once_for_test(Duration::from_millis(40));
    }

    let fires = runtime
        .node_handle("producer")
        .expect("producer handle")
        .fire_count();
    assert!(
        fires > 0,
        "a period_ms=100 node must FIRE end-to-end through build_live + run_live \
         within the 500ms budget (period firing through the live loop), got {fires}"
    );

    drop(runtime);
    drop(transport);
}

// --------------------------------------------------------------------------
// Test 2: no Period node → live_timeout is the 250ms liveliness sweep cap.
// --------------------------------------------------------------------------
#[test]
#[serial]
fn build_live_no_period_timeout_is_sweep_cap() {
    let transport = live_transport();
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = data_only_graph(Arc::clone(&fires));
    let runtime = build_live(&transport, config, factories);

    // No Period node → ns_until_next_fire returns None → live_timeout falls
    // back to the 250ms liveliness sweep cap (then .max(1ms), a no-op here).
    let timeout = runtime.live_timeout_for_test();
    assert!(
        timeout >= Duration::from_millis(249) && timeout <= Duration::from_millis(250),
        "with no Period node the live_timeout must be the 250ms liveliness \
         sweep cap (within ~1ms), got {timeout:?}"
    );

    drop(runtime);
    drop(transport);
}

// --------------------------------------------------------------------------
// Test 3: build_live yields a FUNCTIONING live runtime over real iceoryx2.
//
// An out-of-graph publisher writes onto the absolute external topic; driving
// `run_live_step_once_for_test` fires the data-trigger consumer — parity with
// the `build_for_test` live-loop test, but proving the additive
// `build_live` path (RealClock) wires the same functioning live runtime.
// --------------------------------------------------------------------------
#[test]
#[serial]
fn build_live_advances_under_run_live_step() {
    let transport = live_transport();
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = data_only_graph(Arc::clone(&fires));
    let mut runtime = build_live(&transport, config, factories);

    // Attach an out-of-graph publisher on the absolute external topic via the
    // OWNED transport (build_live does not park it on the runtime).
    let mut pubr = transport
        .create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
        .expect("external publisher must attach to the absolute external topic");

    // Prime: drain build/attach connection-lifecycle noise so the fire count
    // below is attributable to the data publishes (a connection-noise wake
    // calls `step`, but `drain_level` finds no data → no fire).
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    assert_eq!(
        fires.load(Ordering::Relaxed),
        0,
        "priming (no data published) must NOT fire the consumer"
    );

    // Publish a handful of frames, one live iteration each; the consumer's
    // fire_count must increase — proving build_live + run_live_step + RealClock
    // wire a working data path (and do NOT explode).
    const N: u64 = 4;
    for i in 1..=N {
        {
            let mut proxy = pubr.loan_proxy::<Vector3>().expect("loan");
            proxy.x = i as f64;
            // proxy publishes on drop
        }
        runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    }

    let counter = fires.load(Ordering::Relaxed);
    assert!(
        counter > 0,
        "build_live runtime must FIRE the consumer under run_live_step after \
         external publishes (got {counter} fires) — proves a functioning live \
         runtime over real iceoryx2"
    );
    // Cross-check the scheduler's own counter agrees with the in-node counter.
    let handle_count = runtime
        .node_handle("consumer")
        .expect("consumer handle")
        .fire_count();
    assert_eq!(
        counter, handle_count,
        "the in-node fire counter ({counter}) and the scheduler's fire_count \
         ({handle_count}) must agree (same firing path)"
    );

    drop(pubr);
    drop(runtime);
    drop(transport);
}
