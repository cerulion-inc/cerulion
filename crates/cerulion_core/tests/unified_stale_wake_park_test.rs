// SPDX-License-Identifier: AGPL-3.0-only
//! Regression pin: a live park-path run with a UNIFIED
//! data-trigger binding must actually PARK between paced publishes — the
//! standalone `ListenerOnly`'s stale `SentSample` event must not survive the
//! step and masquerade as "data pending" to the next idle poll.
//!
//! # The leak this pins (measured on a Jetson, multi-process)
//!
//! A publish onto a Unified input's topic lands one notification event in
//! the binding's standalone `TriggerSubscriber::ListenerOnly`. Unless the
//! Unified arm drains it, NOTHING on the step path drains that queue (the Separate arm's
//! `try_receive_timestamps` drains its trigger-sub listener inside
//! `drain_level`; the Unified body drain clears only the BODY subscriber's
//! own listener) — so the event SURVIVES the step that consumed the data,
//! and the NEXT `live_step`'s idle poll (`spin_sources`, or the park's first
//! recheck) reads it as a data wake: one spurious wake+step per publish. On
//! the multi-process split that runaway is self-sustaining (the
//! spurious step crosses every barrier boundary, waking peers via the
//! barrier-arrival predicate, and advances the handed-quantum gating clock so the
//! Period producer re-fires): a Jetson run measured the data worker parking ONCE in
//! 40s and the 1kHz ticker free-running at ~7.5kHz wall. So `drain_level`'s
//! Unified arm drains the ListenerOnly's notification queue
//! (events-then-samples — `try_receive`'s exact order), matching the Separate
//! arm.
//!
//! # The in-process pin (the cross-process runaway itself needs two processes)
//!
//! The full free-run needs two processes + a barrier + the handed quantum
//! (only a real multi-process run shows that shape). What IS
//! pinnable in-process is the LEAK itself, via the cfg-test park counters
//! on a forced-park single runtime:
//!
//! - an External-policy producer publishes EXACTLY when triggered (paced by
//!   the harness, not a period — so silent iterations are truly silent);
//! - the macro data-trigger consumer is UNIFIED (`unified_binding_count == 1`);
//! - `CERULION_LIVE_SPIN_US=0` kills the spin arm so ALL idle routing goes
//!   through `monitor_wait_block` and the park counters see every wake;
//! - `CERULION_NOTIFY_ELISION=off` keeps the publish NOTIFYING, which is what
//!   the modelled multi-process shape does and what a single-process harness
//!   otherwise would not (see [`ParkEnvGuard`]) — without it the publish sends
//!   no event at all, so there is no stale event to survive and the
//!   discriminating assert below passes with or without the drain;
//! - after a fire+deliver iteration, M SILENT iterations must ALL be
//!   TIMEOUT-paced: `wakes_listener` delta == 0 (without the drain: 1 — the stale
//!   event from the publish; THE discriminating assert), `wakes_timeout`
//!   delta == M, `park_entries` delta == M;
//! - delivery stays intact across a re-arm (hand oracle 1 then 2 — the
//!   unified-drain live-wake contract: the drain removes STALE wakes, never real
//!   ones).
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test unified_stale_wake_park_test -- --test-threads=1
//! ```
//!
//! `#[serial]`: the env knobs are process-global, and so is iceoryx2.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{ClosureNodeEntry, NodeEntry, NodeInfo};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::monitor_wait::MonitorWaitPolicy;
use cerulion_core::prelude::*;
use cerulion_core::MacroPolicy;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// Sentinel the producer never publishes (it publishes 1, 2, ...).
const MISSING: u64 = u64::MAX;

/// Silent (no-trigger) live iterations in the measured window.
const SILENT_ITERS: u64 = 8;

/// Per-iteration live timeout (the park's deadline).
const STEP_TIMEOUT: Duration = Duration::from_millis(3);

/// RAII guard pinning the two env knobs this pin depends on. Panic-safe
/// removal of both.
///
/// - `CERULION_LIVE_SPIN_US=0` disables the spin arm, so ALL idle routing goes
///   through the park and its wake counters see every wake.
/// - `CERULION_NOTIFY_ELISION=off` restores the notify shape of the leak this
///   file models. Elision skips a graph-owned topic's per-publish notify while
///   the topic's LIVE listener count equals the in-process count the runtime
///   proved it owns. The measured leak is a MULTI-PROCESS one: the consumer
///   lives in another process, the live count exceeds the owned count, the gate
///   stays open and every publish notifies. This harness is single-process, so
///   every listener on the producer's topic is graph-owned, the gate closes,
///   and the publish sends no notify at all. With no notify there is no stale
///   `ListenerOnly` event, the drain under test has nothing to remove, and the
///   discriminating assert below reads 0 whether the drain runs or not. Pinning
///   the knob off is what makes this pin BITE: verified by mutation, the assert
///   fails with `left: 1, right: 0` when the Unified drain is removed.
struct ParkEnvGuard;
impl ParkEnvGuard {
    fn set() -> Self {
        std::env::set_var("CERULION_LIVE_SPIN_US", "0");
        std::env::set_var("CERULION_NOTIFY_ELISION", "off");
        Self
    }
}
impl Drop for ParkEnvGuard {
    fn drop(&mut self) {
        std::env::remove_var("CERULION_LIVE_SPIN_US");
        std::env::remove_var("CERULION_NOTIFY_ELISION");
    }
}

// Macro data-trigger consumer — UNIFIED by default (DropOldest input_meta
// from the macro; `unifies_trigger_drain() == true`). Records each read.
#[cerulion_node]
#[derive(Default)]
struct StaleWakeConsumer {
    #[input(trigger)]
    inp: Vector3,
    last_read: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl StaleWakeConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last_read.store(self.inp.x as u64, Ordering::Relaxed);
        Ok(())
    }
}

#[test]
#[serial]
fn unified_binding_parks_between_paced_publishes_no_stale_listener_wakes() {
    let _park_env = ParkEnvGuard::set();

    let last_read = Arc::new(AtomicU64::new(MISSING));

    // External-policy closure producer: publishes its fire count into
    // Vector3.x ONLY when the harness calls `trigger_external` (so the
    // silent window is truly publish-free — a Period producer would keep
    // firing on the live clock and confound the stale-wake attribution).
    let fires = std::sync::atomic::AtomicU64::new(0);
    let producer = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["out".to_string()]).with_policy(MacroPolicy::External),
        move |ctx| {
            let n = fires.fetch_add(1, Ordering::Relaxed) + 1;
            if let Some(pub_port) = ctx.publisher_mut("out") {
                let mut proxy = pub_port.loan_proxy::<Vector3>()?;
                proxy.x = n as f64;
                proxy.y = 0.0;
                proxy.z = 0.0;
            }
            Ok(())
        },
    )
    .with_label("stale_wake_producer");

    let consumer = StaleWakeConsumer {
        last_read: Arc::clone(&last_read),
        ..Default::default()
    };

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "unified_stale_wake".to_string(),
        prefix: "usw".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "producer".to_string(),
                node_type: "stale_wake_producer".to_string(),
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
                node_type: "stale_wake_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(producer));
    factories.insert(
        "consumer".to_string(),
        Box::new(StaleWakeConsumerEntry::with_state(consumer)),
    );

    // Forced park policy (doorbell OFF — the doorbell counter must stay 0 so
    // listener/timeout attribution is unambiguous).
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test_with_policy(
        config,
        factories,
        clock,
        8,
        MonitorWaitPolicy::new(true, false, "usw".into()),
    )
    .expect("build stale-wake graph");

    assert_eq!(
        runtime.unified_binding_count_for_test(),
        1,
        "the macro data-trigger consumer must be wired Unified (anti-vacuity)"
    );
    // Anti-vacuity, the second half: the publish must really notify. Elision
    // arms per BUILD off the env read, so a build that armed anything on this
    // graph would mean the guard above did not take and the publish would send
    // no event — leaving nothing for the drain to remove and nothing for the
    // discriminating assert to catch.
    assert_eq!(
        runtime.notify_elision_armed_topic_count_for_test(),
        0,
        "notify elision must be DISARMED for this pin: an armed publisher elides \
         the very notify whose stale event the Unified drain exists to clear, \
         which silently turns the listener-wake assert below into a tautology"
    );

    // Iteration 1: fire the producer — the same live step delivers to the
    // unified consumer (hand oracle: the first publish carries 1).
    runtime
        .trigger_external("producer")
        .expect("trigger producer fire 1");
    runtime.run_live_step_once_for_test(STEP_TIMEOUT);
    assert_eq!(
        last_read.load(Ordering::Relaxed),
        1,
        "fire 1 must deliver same-step through the unified binding"
    );

    // THE PIN: M silent iterations. Every one must be TIMEOUT-paced — the
    // publish consumed by iteration 1 must leave NO stale listener event.
    let (e0, l0, d0, t0) = runtime.park_wake_counts();
    for _ in 0..SILENT_ITERS {
        runtime.run_live_step_once_for_test(STEP_TIMEOUT);
    }
    let (e1, l1, d1, t1) = runtime.park_wake_counts();
    assert_eq!(
        l1 - l0,
        0,
        "SILENT iterations saw a LISTENER park wake — a stale ListenerOnly \
         event survived the step that drained its data (the multi-process free-run \
         leak; without the ListenerOnly drain this delta is 1)"
    );
    assert_eq!(
        t1 - t0,
        SILENT_ITERS,
        "every silent iteration must be timer-paced (timeout wakes)"
    );
    assert_eq!(
        e1 - e0,
        SILENT_ITERS,
        "every silent iteration must actually ENTER the park (spin disabled)"
    );
    assert_eq!(d1 - d0, 0, "doorbell policy is off — no doorbell wakes");

    // Re-arm: a REAL publish still wakes + delivers (the live-wake
    // contract — the drain removes STALE wakes, never real ones), and the
    // window after it is silent again.
    runtime
        .trigger_external("producer")
        .expect("trigger producer fire 2");
    runtime.run_live_step_once_for_test(STEP_TIMEOUT);
    assert_eq!(
        last_read.load(Ordering::Relaxed),
        2,
        "fire 2 must deliver (hand oracle: second publish carries 2)"
    );
    let (_, l2, _, _) = runtime.park_wake_counts();
    for _ in 0..2 {
        runtime.run_live_step_once_for_test(STEP_TIMEOUT);
    }
    let (_, l3, _, _) = runtime.park_wake_counts();
    assert_eq!(
        l3 - l2,
        0,
        "the re-arm publish (consumed by its own step) must leave no stale \
         listener event either"
    );
}
