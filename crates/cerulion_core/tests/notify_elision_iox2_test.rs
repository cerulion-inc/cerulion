// SPDX-License-Identifier: AGPL-3.0-only
//! The NOTIFY-ELISION self-healing gate — end-to-end pins over
//! real iceoryx2 (`GraphRuntime::build_for_test`, per-test SHM root; no fake
//! data, Principle #13).
//!
//! For a graph-owned topic whose consumers are all SAME-process + level-gated,
//! the scheduler already drains every consumer at its DAG level within the SAME
//! step the producer fired — so the per-publish iceoryx2 `SentSample` notify
//! only ever produced a SPURIOUS next-step wake (the `unified_stale_wake_park`
//! class). The publisher ELIDES that notify while the topic's LIVE listener count
//! equals the count the runtime PROVED it owns (this publisher's own listener +
//! every in-graph body/trigger/sync listener). The instant a FOREIGN listener
//! attaches (a `topic echo`/`hz`, another process's subscriber, an mp sibling),
//! live rises above `expected` and notifies resume within ONE publish — the
//! re-check IS the self-heal. The DOORBELL ring is untouched (the park's data
//! wake rides it), and the ≤250ms live heartbeat is the always-present wake
//! backstop, so eliding a notify changes only WHEN a consumer next wakes, never
//! WHAT it reads/fires (firewall, Principle #7).
//!
//! What this file pins:
//!
//! - **(A) engagement + exact bookkeeping:** a `producer → data-trigger
//!   consumer` graph elides EXACTLY one notify per publish while no foreign
//!   listener is attached (`notify_elided_count == steps`), the runtime
//!   PROVED-owned `expected == 3` (publisher listener + body subscriber + the
//!   Unified `ListenerOnly`), exactly one publisher armed, and the consumer
//!   still receives the contiguous producer sequence (a HAND ORACLE, not a
//!   self-compare) — elision does not change delivery.
//! - **(B) self-heal:** with the gate eliding, attach a FOREIGN listener on the
//!   topic's event service mid-run → elision STOPS within one publish (the
//!   elided-count delta is 0) while delivery continues; DROP the foreign
//!   listener → elision RESUMES. The gate opens/closes purely on the live vs
//!   expected listener delta.
//! - **(C) mp negative:** a persistent foreign listener attached BEFORE any
//!   step (an mp-sibling / cross-process consumer stand-in) keeps elision OFF
//!   for the whole run (`notify_elided_count == 0`) while delivery still works.
//! - **(D) kill switch:** `CERULION_NOTIFY_ELISION=off` arms ZERO publishers
//!   (`notify_elision_armed_count == 0`), elides nothing, and delivers the same
//!   hand oracle (byte-identical to the earlier always-notify path).
//!
//! `#[serial]` — iceoryx2 SHM singleton + the process-global
//! `CERULION_NOTIFY_ELISION` env (test D); an RAII guard removes it on drop.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test notify_elision_iox2_test -- --test-threads=1
//! ```

use std::sync::atomic::{AtomicU64, Ordering};
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

/// Steps discarded before the measured window (let the data-trigger chain reach
/// steady state).
const WARMUP: u32 = 3;
/// Measured steps: one producer publish + one consumer fire each.
const MEASURED: u32 = 8;
/// A sentinel the producer never publishes (it publishes 1, 2, 3, ...). A
/// measured read of this means the consumer's tick did NOT run — the oracle
/// then fails loud instead of the test silently degrading.
const MISSING: u64 = u64::MAX;

/// The runtime PROVED-owned listener count for the single graph topic: the
/// publisher's own event listener + the consumer's BODY subscriber listener +
/// the consumer's Unified `ListenerOnly` wake source (the macro data-trigger
/// consumer is Unified-eligible by default). Live == this ⇒ elide.
const EXPECTED_LISTENERS: usize = 3;

/// The kill switch (test D).
const ELISION_KNOB: &str = "CERULION_NOTIFY_ELISION";

/// RAII: remove `CERULION_NOTIFY_ELISION` on drop (panic-safe).
struct ElisionEnvGuard;
impl ElisionEnvGuard {
    fn set(value: &str) -> Self {
        std::env::set_var(ELISION_KNOB, value);
        Self
    }
}
impl Drop for ElisionEnvGuard {
    fn drop(&mut self) {
        std::env::remove_var(ELISION_KNOB);
    }
}

// ===========================================================================
// Nodes: a Period(10) producer publishing an incrementing counter, and a macro
// data-trigger consumer that records the LIVE trigger read each tick (a macro
// node with a default DropOldest trigger input is Unified-eligible).
// ===========================================================================

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct ElideProducer {
    #[output]
    out: Vector3,
    n: u64,
}

#[cerulion_node_impl]
impl ElideProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

#[cerulion_node]
#[derive(Default)]
struct ElideConsumer {
    #[input(trigger)]
    inp: Vector3,
    /// Records the LIVE trigger read on each fire (shared with the harness).
    last_read: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl ElideConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last_read.store(self.inp.x as u64, Ordering::Relaxed);
        Ok(())
    }
}

/// Build the `producer -> data-trigger consumer` graph over an isolated test
/// transport. Returns `(runtime, last_read, topic)`. Reads the elision env
/// FRESH at build, so the caller sets/clears the kill switch before calling.
fn build_chain(prefix: &str) -> (GraphRuntime, Arc<AtomicU64>, String) {
    let last_read = Arc::new(AtomicU64::new(MISSING));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "notify_elision".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "elide_producer".to_string(),
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
                id: "consumer".to_string(),
                node_type: "elide_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(ElideProducerEntry::new()));
    let consumer = ElideConsumer {
        last_read: Arc::clone(&last_read),
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(ElideConsumerEntry::with_state(consumer)),
    );

    let clock = Arc::new(VirtualClock::new());
    let runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build notify-elision graph");
    // `producer/out` resolves to `/{prefix}/producer/out`.
    let topic = format!("/{prefix}/producer/out");
    (runtime, last_read, topic)
}

/// Drive `n` steps, returning the delivered sequence (the consumer's LIVE read
/// each step). Resets to `MISSING` before each step so a non-running consumer
/// is detectable.
fn drive(runtime: &mut GraphRuntime, last_read: &Arc<AtomicU64>, n: u32) -> Vec<u64> {
    let mut seq = Vec::with_capacity(n as usize);
    for _ in 0..n {
        last_read.store(MISSING, Ordering::Relaxed);
        runtime.step(Duration::from_millis(10));
        seq.push(last_read.load(Ordering::Relaxed));
    }
    seq
}

/// Attach a FOREIGN event listener on `topic`'s event service via the runtime's
/// OWN transport manager (the gate cannot tell WHO owns a listener — it only
/// compares live vs the count it PROVED at build, so any extra listener raises
/// live and opens the gate, exactly as a `topic echo` / mp sibling would).
fn attach_foreign_listener(
    runtime: &GraphRuntime,
    topic: &str,
) -> iceoryx2::port::listener::Listener<iceoryx2::service::ipc_threadsafe::Service> {
    let mgr = runtime
        .test_transport()
        .expect("build_for_test parks a test transport");
    mgr.create_trigger_listener_for_test(topic, mgr.default_topic_config())
        .expect("attach a foreign listener on the graph topic's event service")
}

// ===========================================================================
// (A) engagement + exact bookkeeping
// ===========================================================================

#[test]
#[serial]
fn elision_engages_and_bookkeeping_is_exact() {
    let (mut runtime, last_read, topic) = build_chain("ntfy_elide_a");

    // The runtime armed exactly ONE publisher and PROVED it owns 3 listeners.
    assert_eq!(
        runtime.notify_elision_armed_topic_count_for_test(),
        1,
        "one graph-owned publisher must be armed (default: elision ON)"
    );
    assert_eq!(
        runtime.notify_elision_expected_for_test(&topic),
        Some(EXPECTED_LISTENERS),
        "expected = publisher listener + body subscriber + Unified ListenerOnly"
    );

    // Drive the whole run; every publish is gated (no foreign listener).
    let total_steps = WARMUP + MEASURED;
    let warm = drive(&mut runtime, &last_read, WARMUP);
    let measured = drive(&mut runtime, &last_read, MEASURED);

    // Every publish elided exactly once.
    assert_eq!(
        runtime.notify_elided_count_for_test(&topic),
        total_steps as u64,
        "one elided notify per publish while gated (no foreign listener)"
    );

    // Delivery is unaffected — the consumer read the contiguous producer
    // sequence. Hand oracle (producer publishes 1,2,3,... one per step), NOT a
    // self-compare.
    let oracle: Vec<u64> = (1..=total_steps as u64).collect();
    let delivered: Vec<u64> = warm.into_iter().chain(measured).collect();
    assert_eq!(
        delivered, oracle,
        "elision must not change WHAT is delivered — the full contiguous sequence flows"
    );
}

// ===========================================================================
// (B) self-heal: foreign listener attaches mid-run -> notifies resume; detach
//     -> re-elide.
// ===========================================================================

#[test]
#[serial]
fn foreign_listener_opens_gate_within_one_publish_then_re_elides_on_detach() {
    let (mut runtime, last_read, topic) = build_chain("ntfy_elide_b");

    // First phase: gated — elision engages.
    let p1 = drive(&mut runtime, &last_read, WARMUP + MEASURED);
    let elided_after_p1 = runtime.notify_elided_count_for_test(&topic);
    assert_eq!(
        elided_after_p1,
        (WARMUP + MEASURED) as u64,
        "phase 1 (no foreign listener): every publish elided"
    );

    // Second phase: attach a FOREIGN listener -> live = 4 != expected 3 -> notifies
    // resume within one publish.
    let foreign = attach_foreign_listener(&runtime, &topic);
    let p2 = drive(&mut runtime, &last_read, MEASURED);
    let elided_after_p2 = runtime.notify_elided_count_for_test(&topic);
    assert_eq!(
        elided_after_p2, elided_after_p1,
        "phase 2 (foreign listener present): elision STOPS immediately — the delta is 0 \
         (notifies resume within one publish; the re-check IS the self-heal)"
    );

    // Third phase: DROP the foreign listener -> live back to 3 -> elision resumes.
    drop(foreign);
    let p3 = drive(&mut runtime, &last_read, MEASURED);
    let elided_after_p3 = runtime.notify_elided_count_for_test(&topic);
    assert_eq!(
        elided_after_p3,
        elided_after_p2 + MEASURED as u64,
        "phase 3 (foreign listener gone): elision RESUMES — every publish elided again"
    );

    // Delivery continued through EVERY phase (elision changes WHEN a consumer
    // wakes, never WHAT it delivers). The producer publishes monotonically
    // 1,2,3,...; each phase's reads must be strictly increasing and never MISSING.
    let all: Vec<u64> = p1.into_iter().chain(p2).chain(p3).collect();
    let full_oracle: Vec<u64> = (1..=all.len() as u64).collect();
    assert_eq!(
        all, full_oracle,
        "delivery is byte-identical across the gate open/close cycle (firewall)"
    );
}

// ===========================================================================
// (C) mp negative: a persistent foreign listener attached BEFORE any step keeps
//     elision OFF for the whole run (models an mp sibling / cross-process
//     consumer holding a listener on the same event service).
// ===========================================================================

#[test]
#[serial]
fn persistent_foreign_listener_keeps_elision_off_but_still_delivers() {
    let (mut runtime, last_read, topic) = build_chain("ntfy_elide_c");

    // Attach the "mp sibling" listener BEFORE driving any step.
    let _sibling = attach_foreign_listener(&runtime, &topic);

    let seq = drive(&mut runtime, &last_read, WARMUP + MEASURED);

    assert_eq!(
        runtime.notify_elided_count_for_test(&topic),
        0,
        "a persistent foreign (mp-sibling) listener keeps the gate OPEN — never elide"
    );
    // Delivery still works — the sibling listener does not disturb the level drain.
    let oracle: Vec<u64> = (1..=(WARMUP + MEASURED) as u64).collect();
    assert_eq!(
        seq, oracle,
        "delivery unaffected while notifies are (correctly) never elided"
    );
}

// ===========================================================================
// (D) kill switch: CERULION_NOTIFY_ELISION=off arms nothing and elides nothing,
//     yet delivers the same hand oracle.
// ===========================================================================

#[test]
#[serial]
fn kill_switch_off_arms_nothing_and_still_delivers() {
    let _g = ElisionEnvGuard::set("off");
    let (mut runtime, last_read, topic) = build_chain("ntfy_elide_d");

    assert_eq!(
        runtime.notify_elision_armed_topic_count_for_test(),
        0,
        "CERULION_NOTIFY_ELISION=off must arm ZERO publishers"
    );
    assert_eq!(
        runtime.notify_elision_expected_for_test(&topic),
        None,
        "no publisher armed ⇒ no expected count retained"
    );

    let seq = drive(&mut runtime, &last_read, WARMUP + MEASURED);
    assert_eq!(
        runtime.notify_elided_count_for_test(&topic),
        0,
        "kill switch off ⇒ every publish notifies (nothing elided)"
    );
    let oracle: Vec<u64> = (1..=(WARMUP + MEASURED) as u64).collect();
    assert_eq!(
        seq, oracle,
        "delivery is byte-identical to the elision-on path (the firewall holds either way)"
    );
}

// ===========================================================================
// (E) A DATA-ONLY capture tap keeps elision ARMED — the "bonus" that
//     makes a pure `graph run --record` issue zero notifies. Mutation oracle:
//     the OLD listener-full open-only tap (the earlier recorder path) DOES
//     disarm the gate, so this pin fails if the phase-2 tap constructor is
//     reverted from `create_data_only_subscriber` to `create_subscriber_open_only`.
// ===========================================================================

#[test]
#[serial]
fn data_only_tap_keeps_elision_armed_but_listener_full_tap_disarms_it() {
    let (mut runtime, last_read, topic) = build_chain("ntfy_elide_e");

    // First phase: gated baseline (no tap) — every publish elides.
    let p1 = drive(&mut runtime, &last_read, WARMUP + MEASURED);
    let elided_after_p1 = runtime.notify_elided_count_for_test(&topic);
    assert_eq!(
        elided_after_p1,
        (WARMUP + MEASURED) as u64,
        "phase 1 (no tap): every publish elided"
    );

    // Second phase (HEADLINE): attach a DATA-ONLY capture tap. It opens only
    // the data service and registers NO event listener, so the producer's LIVE
    // listener count stays == expected → the gate STAYS ARMED → elision keeps
    // firing every publish. The record run pays ZERO notifies to the real
    // consumers because the tap is invisible to the event service.
    let data_tap = {
        let mgr = runtime
            .test_transport()
            .expect("build_for_test parks a test transport");
        mgr.create_data_only_subscriber(&topic)
            .expect("data-only capture tap attaches to the graph topic")
    };
    let p2 = drive(&mut runtime, &last_read, MEASURED);
    let elided_after_p2 = runtime.notify_elided_count_for_test(&topic);
    assert_eq!(
        elided_after_p2,
        elided_after_p1 + MEASURED as u64,
        "phase 2 (DATA-ONLY tap attached): elision STAYS ARMED — every publish still \
         elided (the tap registered no listener, so live == expected). Reverting the \
         phase-2 constructor call above from `create_data_only_subscriber` to the \
         listener-full `create_subscriber_open_only` FAILS here (delta would be 0) — \
         that transport-constructor choice IS the data-only-tap mechanism (the recorder/replay \
         call sites are wired to `DataOnlySubscriber` by TYPE, so reverting THEM is a \
         compile error, not a runtime miss)."
    );

    // Third phase: attach the OLD listener-full
    // open-only tap (a `CerulionSubscriber` DOES create an event listener). Now
    // live > expected → the gate DISARMS → notifies resume (elided delta 0).
    // This proves the apparatus bites: a listener-bearing tap is exactly what
    // the data-only tap does away with, and its presence is observable here.
    let listener_full_tap = {
        let mgr = runtime
            .test_transport()
            .expect("build_for_test parks a test transport");
        mgr.create_subscriber_open_only(&topic)
            .expect("listener-full open-only tap attaches")
    };
    let p3 = drive(&mut runtime, &last_read, MEASURED);
    let elided_after_p3 = runtime.notify_elided_count_for_test(&topic);
    assert_eq!(
        elided_after_p3, elided_after_p2,
        "phase 3 (listener-full tap attached): elision DISARMS — delta 0 (a listener \
         raised live above expected). The data-only vs listener-full contrast is the \
         whole data-only-tap mechanism."
    );

    // Firewall: delivery was byte-identical through every phase — the tap (of
    // either kind) and the elision gate change only WHEN a consumer wakes, never
    // WHAT it reads. Hand oracle: producer publishes 1,2,3,... one per step.
    let all: Vec<u64> = p1.into_iter().chain(p2).chain(p3).collect();
    let full_oracle: Vec<u64> = (1..=all.len() as u64).collect();
    assert_eq!(
        all, full_oracle,
        "delivery is byte-identical across the data-only and listener-full tap phases"
    );

    drop(data_tap);
    drop(listener_full_tap);
}

// ===========================================================================
// (F) the boundary resweep does NOT fire a redundant notify
//     when the per-publish path already woke the listener this window. The
//     runtime runs the boundary (`pump_history_all`) on EVERY live_step, so with
//     an ACTIVE producer + a foreign listener, the notify count per window must
//     ≈ the PUBLISH count, not publish + step count.
// ===========================================================================

#[test]
#[serial]
fn boundary_resweep_does_not_double_notify_an_active_producer() {
    let (mut runtime, last_read, topic) = build_chain("ntfy_elide_f");

    // A persistent FOREIGN listener (a `topic hz`) — so the per-publish path
    // never elides (live > expected) and notifies on EVERY publish.
    let foreign = attach_foreign_listener(&runtime, &topic);

    // Drive N steps as the live loop does: step() (producer publishes +
    // per-publish notifies) THEN the boundary pass (pump_history_all).
    const N: u32 = 8;
    let mut delivered = Vec::with_capacity(N as usize);
    for _ in 0..N {
        last_read.store(MISSING, Ordering::Relaxed);
        runtime.step(Duration::from_millis(10));
        runtime.pump_history_all_for_test(); // the boundary re-check
        delivered.push(last_read.load(Ordering::Relaxed));
    }

    // THE PIN: the per-publish path notified every step (foreign present ⇒ not
    // elided), so the GUARD skipped every boundary notify — the boundary FIRE
    // count is ZERO, not N. WITHOUT the guard the resweep would fire on every
    // step (== N), doubling the notify rate for every echoed armed topic.
    assert_eq!(
        runtime.notify_resweep_count_for_test(&topic),
        0,
        "active producer + foreign listener: the boundary sweep fires ZERO redundant \
         notifies (the per-publish path woke the listener every step)"
    );
    // And it elided nothing (foreign present the whole time).
    assert_eq!(
        runtime.notify_elided_count_for_test(&topic),
        0,
        "foreign listener present: the per-publish path never elided"
    );

    // Firewall: delivery is complete + contiguous regardless (the guard changes
    // only WHETHER a redundant wake fires, never WHAT is delivered). Hand oracle.
    let oracle: Vec<u64> = (1..=N as u64).collect();
    assert_eq!(
        delivered, oracle,
        "delivery is byte-identical — the guard only suppresses a redundant wake"
    );

    drop(foreign);
}
