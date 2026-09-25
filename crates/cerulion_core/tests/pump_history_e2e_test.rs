// SPDX-License-Identifier: AGPL-3.0-only
//! The FULL end-to-end proof that a QUIESCENT publisher
//! delivers its retained native history to a LATE joiner through the REAL
//! runtime pump chain — for BOTH an in-process node AND a cdylib node.
//!
//! # What gap this closes
//!
//! The existing coverage is PIECEWISE:
//!
//! - `deliver_history_failure_iox2_test.rs` proves the publisher-LEVEL
//!   mechanism (`CerulionPublisher::pump_history` → `deliver_history` →
//!   `update_connections()` + `SentHistory`) but pokes the publisher directly,
//!   bypassing the node and the runtime.
//! - `pump_history_runtime_test.rs` proves the runtime GLUE
//!   (`live_step` → `pump_history_all` → `NodeEntry::pump_history`) with a SPY
//!   node whose `pump_history` just bumps a counter — no real publisher, no real
//!   late joiner, no real history delivery.
//!
//! Nothing drives a REAL node's forwarding chain end-to-end to a real late
//! joiner. This test does, exercising every link:
//!
//! ```text
//! run_live_step_once_for_test
//!   → live_step
//!     → pump_history_all          (runtime.rs)
//!       → entry.pump_history      (NodeEntry: ClosureNodeEntry / DylibNodeEntry)
//!         → NodeContext::pump_history
//!           → AnyPublisher::pump_history
//!             → CerulionPublisher::pump_history
//!               → check_subscriber_events
//!                 → deliver_history
//!                   → publisher.update_connections()   (native iceoryx2 history)
//!                   → notify SentHistory               (wakes the late joiner)
//! ```
//!
//! The cdylib variant additionally crosses the `cerulion_node_pump_history`
//! FFI seam (ABI v7).
//!
//! # Orchestration (the non-vacuity contract)
//!
//! The producer is an `External`-policy node. The test:
//!
//! 1. fires it THREE times (`trigger_external` + `step_ms(1)`) BEFORE any late
//!    joiner exists → it publishes sequences 0,1,2 into its publisher's native
//!    history. No subscriber is connected yet, so the publish-path
//!    `check_subscriber_events` delivers nothing to anyone.
//! 2. opens a LATE external subscriber on the producer's output topic (its
//!    constructor enqueues a `SubscriberConnected` on the publisher's listener,
//!    PENDING and undrained).
//! 3. runs ONE `run_live_step_once_for_test` WITHOUT another trigger. The
//!    producer is `External` and untriggered → it does NOT fire → there is NO
//!    publish during this step. The ONLY thing that can drain the pending
//!    `SubscriberConnected` (and thus deliver the retained history) is the
//!    runtime's `pump_history` cadence.
//!
//! So if the late joiner ends up with {0,1,2}, those frames could ONLY have
//! arrived via the runtime pump chain — the assertion is non-vacuous.
//!
//! ## Mutation kill
//!
//! Reverting `pump_history_all` to a no-op (or deleting its call from
//! `live_step`) leaves the late joiner EMPTY: the pending `SubscriberConnected`
//! is never drained, `update_connections()` never runs, no `SentHistory` wakes
//! the subscriber, and {0,1,2} stay stranded in the publisher's history. The
//! `assert_eq!(seqs, {0,1,2})` then fails. The cdylib variant fails the same
//! way, and also fails if the `cerulion_node_pump_history` FFI dispatch
//! breaks.
//!
//! Both tests are `#[serial]`: `build_for_test` builds over the process-global
//! iceoryx2 shared-memory singleton.

use std::collections::BTreeSet;
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::{
    ClosureNodeEntry, DylibNodeEntry, MacroPolicy, NodeEntry, NodeInfo,
};
use cerulion_core::graph::{resolve_output_topic, GraphRuntime};
use cerulion_core::transport::subscriber::CerulionSubscriber;
use cerulion_core::transport::TransportManager;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// Late-joiner replay depth retained natively by the producer's publisher.
const HISTORY_SIZE: usize = 3;

/// Build a single-producer graph: one node `node_id` with one output
/// `output_name` (schema `Vector3`, `history_size = HISTORY_SIZE`), wired via
/// the supplied factory. No consumers in the graph — the late joiner is opened
/// out-of-graph against the producer's output topic.
fn producer_graph(
    node_id: &str,
    node_type: &str,
    output_name: &str,
    prefix: &str,
    factory: Box<dyn NodeEntry>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "pump_history_e2e_test".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: node_id.to_string(),
            node_type: node_type.to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: output_name.to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                topic: None,
                history_size: HISTORY_SIZE,
            }],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(node_id.to_string(), factory);
    (config, factories)
}

/// Drain a subscriber's wire sequences (a few non-blocking-ish rounds), keyed
/// off `msg.header().sequence` — the publisher-assigned per-publish counter,
/// independent of payload, so identical-payload cdylib frames are still
/// distinguishable.
fn drain_sequences(sub: &mut CerulionSubscriber) -> BTreeSet<u32> {
    let mut seqs = BTreeSet::new();
    for _ in 0..6 {
        let _ = sub
            .wait_for_message(Duration::from_millis(100), |msg| {
                seqs.insert(msg.header().sequence);
            })
            .expect("wait_for_message");
    }
    seqs
}

/// Drive the FULL runtime pump chain end-to-end and return the set of wire
/// sequences the late joiner received.
///
/// `node_id` / `node_type` / `output_name` / `prefix` parametrize the graph;
/// `factory` is the producer node entry (in-process closure OR loaded cdylib).
/// The producer MUST be `External`-policy so it fires only when the test calls
/// `trigger_external` — that is what makes step (3) below a true no-publish
/// step.
fn run_quiescent_history_e2e(
    node_id: &str,
    node_type: &str,
    output_name: &str,
    prefix: &str,
    factory: Box<dyn NodeEntry>,
) -> BTreeSet<u32> {
    let (config, factories) = producer_graph(node_id, node_type, output_name, prefix, factory);
    let clock = Arc::new(VirtualClock::new());
    // subscriber_buffer_size = 8 ≥ HISTORY_SIZE = 3, so the late joiner's queue
    // can absorb the FULL replay (no per-consumer truncation).
    let mut runtime = match GraphRuntime::build_for_test(config, factories, clock, 8) {
        Ok(r) => r,
        Err(e) => panic!("build_for_test must succeed: {e}"),
    };

    // (1) Fire the External producer THREE times, each on its own step, BEFORE
    // any late joiner exists. Each fire publishes one frame into the producer's
    // publisher native history (wire sequences 0,1,2). No subscriber is
    // connected, so the publish-path drain delivers nothing yet.
    for _ in 0..HISTORY_SIZE {
        runtime
            .trigger_external(node_id)
            .expect("trigger_external must arm the External producer");
        runtime.step_ms(1);
    }
    assert_eq!(
        runtime
            .node_handle(node_id)
            .expect("producer node handle")
            .fire_count(),
        HISTORY_SIZE as u64,
        "the External producer must have fired exactly {HISTORY_SIZE} times \
         (one publish per trigger) — the frames the late joiner should later \
         receive via the pump"
    );

    // (2) Open a LATE external subscriber on the producer's output topic via the
    // runtime's parked test transport. Its constructor enqueues a
    // `SubscriberConnected` on the publisher's listener — PENDING, undrained.
    let output_def = OutputDef {
        name: output_name.to_string(),
        schema: "Vector3".to_string(),
        max_slice_len: None,
        topic: None,
        history_size: HISTORY_SIZE,
    };
    let topic = resolve_output_topic(prefix, node_id, &output_def);
    let mgr: &Arc<TransportManager> = runtime
        .test_transport()
        .expect("build_for_test parks a test transport");
    let mut late = mgr
        .create_subscriber(&topic)
        .expect("late external subscriber must attach to the producer's output topic");

    // (3) ONE live iteration with NO trigger. The External producer does NOT
    // fire (untriggered), so there is NO publish this step. The ONLY driver that
    // can drain the pending `SubscriberConnected` — and thus run
    // `update_connections()` to deliver the retained history + fire
    // `SentHistory` — is the runtime's `pump_history_all` cadence inside
    // `live_step`. This is the load-bearing call.
    runtime.run_live_step_once_for_test(Duration::from_millis(20));

    // The producer must still have fired exactly HISTORY_SIZE times — the
    // pumped step published nothing (catches a variant that fires the External
    // node on an untriggered step, which would muddy the non-vacuity argument).
    assert_eq!(
        runtime
            .node_handle(node_id)
            .expect("producer node handle")
            .fire_count(),
        HISTORY_SIZE as u64,
        "the untriggered live step must NOT fire the External producer (no new \
         publish) — so any frame the late joiner sees came from the pump, not a \
         fresh send"
    );

    // (4) Drain the late joiner. With the pump having run, it should hold the
    // three retained history frames.
    drain_sequences(&mut late)
}

/// In-PROCESS node (`ClosureNodeEntry`, External policy) → real iceoryx2
/// publisher → real late joiner, driven entirely by the runtime pump chain.
///
/// Mutation kill: revert `pump_history_all` to a no-op (or drop its call from
/// `live_step`) and the late joiner receives NOTHING — the assertion below
/// fails. No publish happens during the pumped step, so {0,1,2} can ONLY have
/// arrived via the pump.
#[test]
#[serial]
fn in_process_quiescent_node_delivers_history_to_late_joiner_via_runtime_pump() {
    // External producer whose tick publishes one incrementing Vector3 on "out".
    // The payload increments (frame 0 → x=1.0, etc.) so a corrupted/dropped
    // frame would also corrupt the running value, but the load-bearing assertion
    // is the wire-sequence SET.
    let info =
        NodeInfo::from_names(vec![], vec!["out".to_string()]).with_policy(MacroPolicy::External);
    let mut frame: f64 = 0.0;
    let factory = Box::new(ClosureNodeEntry::new(info, move |ctx| {
        frame += 1.0;
        let pubr = ctx
            .publisher_mut("out")
            .expect("the 'out' publisher must be wired for the producer's output");
        let mut proxy = pubr.loan_proxy::<Vector3>().expect("loan_proxy 'out'");
        proxy.x = frame;
        proxy.y = 0.0;
        proxy.z = 0.0;
        drop(proxy); // publish on drop
        Ok(())
    }));

    let seqs = run_quiescent_history_e2e("ip_src", "ip_source", "out", "phe2e_ip", factory);

    let expected: BTreeSet<u32> = (0u32..HISTORY_SIZE as u32).collect();
    assert_eq!(
        seqs, expected,
        "the in-process quiescent producer's late joiner must receive ALL three \
         retained native-history frames (wire sequences {expected:?}) via the \
         runtime pump chain (live_step → pump_history_all → NodeContext::\
         pump_history → publisher pump_history → deliver_history → \
         update_connections); got {seqs:?}. Empty here = the pump never ran."
    );
}

/// CDYLIB node (`DylibNodeEntry::load` of `test_node_macro_external_cdylib`,
/// `#[cerulion_node(external)] #[output] cmd: Vector3`) → real iceoryx2
/// publisher → real late joiner, driven by the runtime pump chain ACROSS THE
/// FFI BOUNDARY (`cerulion_node_pump_history`, ABI v7).
///
/// Reverting `pump_history_all` to a no-op leaves the late joiner empty;
/// additionally, breaking the FFI `pump_history` dispatch
/// (`DylibNodeEntry::pump_history` → `cerulion_node_pump_history`) strands the
/// history too.
#[test]
#[serial]
fn cdylib_quiescent_node_delivers_history_to_late_joiner_via_runtime_pump() {
    let dylib_path = find_external_macro_cdylib();
    let factory =
        Box::new(DylibNodeEntry::load(&dylib_path).expect("load test_node_macro_external_cdylib"));

    // The cdylib's tick writes a CONSTANT payload (`cmd.x = 0.0`), so all frames
    // share a payload — but the publisher assigns distinct wire sequences
    // (0,1,2) per publish, which `drain_sequences` keys off, so the frames are
    // still distinguishable. A unique prefix avoids any cross-test SHM service
    // collision with the in-process variant.
    let seqs = run_quiescent_history_e2e("cd_src", "cd_source", "cmd", "phe2e_cd", factory);

    let expected: BTreeSet<u32> = (0u32..HISTORY_SIZE as u32).collect();
    assert_eq!(
        seqs, expected,
        "the cdylib quiescent producer's late joiner must receive ALL three \
         retained native-history frames (wire sequences {expected:?}) via the \
         runtime pump chain crossing the cerulion_node_pump_history FFI seam; \
         got {seqs:?}. Empty here = the pump (or its FFI dispatch) never ran."
    );
}

/// Locate the built `test_node_macro_external_cdylib` shared library in this
/// run's own cargo target directory. Mirrors `node_test.rs::find_test_cdylib`.
fn find_external_macro_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_external_cdylib")
}
