// SPDX-License-Identifier: AGPL-3.0-only
//! Cdylib-parity cluster 3: the trigger-policy FIRE semantics of macro cdylibs,
//! end-to-end over `GraphRuntime`.
//!
//! The macro's per-policy JSON round-trip (`period_ms`, `data_trigger`,
//! `sync_window_ms`, `external`/`HostDriven`, the default-Data warn, and the
//! inert-at-launch refusal) is pinned by `macro_cdylib_policy_round_trip_test`
//! (the DECLARATION crosses the FFI) and the in-process fire suites
//! (`sync_fire_iox2_test`, `external_fire_iox2_test`, `graph_default_policy_warn_test`,
//! `external_gating_replay_iox2_test`) — but those in-process suites never
//! drove a LOADED cdylib. This file proves the FIRE BEHAVIOR through the
//! production `DylibNodeEntry` path for each policy:
//!
//! - **(a) Period** — `test_node_macro_period_cdylib` (`period_ms = 50`) fires
//!   `floor(elapsed / period)` times under VirtualClock stepping (a SCHEDULE
//!   oracle, not a timing one — sub-period steps prove it is not one-per-step).
//! - **(b) DataTrigger** — `test_node_macro_data_trigger_cdylib` fires exactly
//!   once per published trigger frame and zero on silent steps.
//! - **(c) Sync** — `test_node_macro_sync_cdylib` (`sync_window_ms = 25`) fires
//!   when BOTH trigger inputs arrive in-window, and NOT when only one arrives.
//! - **(d) External (HostDriven)** — `test_node_macro_external_cdylib` fires via
//!   `trigger_external` + `step` and DELIVERS its output; a silent step never
//!   re-fires.
//! - **(e) Default-policy warn** — the raw-FFI `test_node_cdylib` (policy None,
//!   no trigger inputs) emits the host's `no macro-declared policy` warn at
//!   `build_for_test`.
//! - **(f) Inert-at-launch refusal** — `run_live` REFUSES the loaded HostDriven
//!   cdylib with `ExternalNodesInertAtLaunch` naming it (through the FFI-read
//!   `external_source()`).
//!
//! SKIPPED (noted): the tier-1 `ExternalSource::Fd` device-drain arm is
//! box-hardware-flavored (a real pollable device fd) and is covered by
//! `external_live_fire_iox2_test` / `cdylib_blocking_doorbell_test`; it is not
//! re-driven here.
//!
//! # Build requirement + serial
//!
//! Requires the five fixtures built:
//! `cargo build -p test_node_macro_period_cdylib -p test_node_macro_data_trigger_cdylib \
//!  -p test_node_macro_sync_cdylib -p test_node_macro_external_cdylib -p test_node_cdylib`.
//! All tests `#[serial]` (cdylib `NODES` + iceoryx2 SHM singletons).

use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

fn find_cdylib(crate_name: &str) -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib(crate_name)
}

fn load(crate_name: &str) -> Box<dyn NodeEntry> {
    Box::new(DylibNodeEntry::load(&find_cdylib(crate_name)).expect("load fixture"))
}

/// Single Vector3 output def.
fn out_def(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    }
}

// ===========================================================================
// (a) Period — floor(elapsed / period) schedule oracle
// ===========================================================================

fn build_single(
    node_type: &str,
    fixture: &str,
    outputs: Vec<OutputDef>,
    inputs: Vec<InputDef>,
    prefix: &str,
    buffer: usize,
) -> GraphRuntime {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("cts_{prefix}"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "node".to_string(),
            node_type: node_type.to_string(),
            inputs,
            outputs,
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("node".to_string(), load(fixture));
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, buffer).expect("build single-node graph")
}

#[test]
#[serial]
fn period_cdylib_fires_on_schedule_not_per_step() {
    // FINE granularity (10 ms steps, 50 ms period): if the node fired per-step
    // it would fire 30×; the 50 ms schedule fires floor(300/50) = 6 (±1 for the
    // t=0 boundary alignment). Proves the fire cadence is the PERIOD schedule,
    // not the step rate.
    let mut fine = build_single(
        "period_node",
        "test_node_macro_period_cdylib",
        vec![out_def("cmd")],
        vec![],
        "ctspa",
        8,
    );
    for _ in 0..30 {
        fine.step(Duration::from_millis(10));
    }
    let fine_fires = fine.node_handle("node").unwrap().fire_count();
    assert!(
        (5..=8).contains(&fine_fires),
        "period_ms=50 stepped 30×10ms must fire ~floor(300/50)=6 times (±first-fire \
         boundary), NOT 30 (schedule oracle, got {fine_fires})"
    );
    assert!(
        fine_fires < 30,
        "the period node must not fire once per step"
    );

    // COARSE granularity (step == period): a fresh graph stepped N× at exactly
    // the 50 ms period fires exactly N (matches the in-process `throttle` /
    // period precedent that step==period ⇒ one fire per step).
    let mut coarse = build_single(
        "period_node",
        "test_node_macro_period_cdylib",
        vec![out_def("cmd")],
        vec![],
        "ctspb",
        8,
    );
    for _ in 0..8 {
        coarse.step(Duration::from_millis(50));
    }
    assert_eq!(
        coarse.node_handle("node").unwrap().fire_count(),
        8,
        "period_ms=50 stepped 8× at 50ms must fire exactly 8 times (floor(400/50))"
    );
}

// ===========================================================================
// (b) DataTrigger — one fire per published trigger frame, zero on silence
// ===========================================================================

#[test]
#[serial]
fn data_trigger_cdylib_fires_once_per_frame_zero_on_silence() {
    // The trigger input `trigger_in` is sourced from an ABSOLUTE external topic
    // so the test controls exactly when a frame arrives.
    const TOPIC: &str = "/ctsdt/in";
    let mut rt = build_single(
        "data_trigger_node",
        "test_node_macro_data_trigger_cdylib",
        vec![out_def("cmd")],
        vec![InputDef {
            name: "trigger_in".to_string(),
            source: TOPIC.to_string(),
        }],
        "ctsdt",
        8,
    );
    let mgr = Arc::clone(rt.test_transport().expect("test transport parked"));
    let mut pubr = mgr
        .create_publisher(TOPIC, MaxSliceLen::const_new(64), 0)
        .expect("external publisher on the trigger topic");

    let fc = |rt: &GraphRuntime| rt.node_handle("node").unwrap().fire_count();

    // Frame 1 → fires once.
    {
        let mut p = pubr.loan_proxy::<Vector3>().expect("loan");
        p.x = 1.0;
    }
    rt.step(Duration::from_millis(1));
    assert_eq!(fc(&rt), 1, "one trigger frame → exactly one fire");

    // Silent step → no fire.
    rt.step(Duration::from_millis(1));
    assert_eq!(
        fc(&rt),
        1,
        "a silent step must NOT fire a data-trigger node"
    );

    // Frame 2 → fires again.
    {
        let mut p = pubr.loan_proxy::<Vector3>().expect("loan");
        p.x = 2.0;
    }
    rt.step(Duration::from_millis(1));
    assert_eq!(fc(&rt), 2, "a second trigger frame → a second fire");

    // Two more silent steps → still 2.
    rt.step(Duration::from_millis(1));
    rt.step(Duration::from_millis(1));
    assert_eq!(fc(&rt), 2, "silent steps never accrue fires");
}

// ===========================================================================
// (c) Sync — both-in-window fires, one-only does not (crib sync_fire_iox2)
// ===========================================================================

fn build_sync(prefix: &str, topic_cam: &str, topic_imu: &str) -> GraphRuntime {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("cts_{prefix}"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "node".to_string(),
            node_type: "sync_node".to_string(),
            inputs: vec![
                InputDef {
                    name: "cam".to_string(),
                    source: topic_cam.to_string(),
                },
                InputDef {
                    name: "imu".to_string(),
                    source: topic_imu.to_string(),
                },
            ],
            outputs: vec![out_def("fused")],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("node".to_string(), load("test_node_macro_sync_cdylib"));
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 8).expect("build sync graph")
}

#[test]
#[serial]
fn sync_cdylib_fires_only_when_both_inputs_in_window() {
    const CAM: &str = "/ctsc/cam";
    const IMU: &str = "/ctsc/imu";

    // POSITIVE: both inputs arrive at the same sim-time (within the 25 ms
    // window) → fires once on the next step.
    let mut rt = build_sync("ctsc1", CAM, IMU);
    let mgr = Arc::clone(rt.test_transport().expect("test transport parked"));
    let mut pub_cam = mgr
        .create_publisher(CAM, MaxSliceLen::const_new(64), 0)
        .expect("cam publisher");
    let mut pub_imu = mgr
        .create_publisher(IMU, MaxSliceLen::const_new(64), 0)
        .expect("imu publisher");
    {
        let mut c = pub_cam.loan_proxy::<Vector3>().expect("loan cam");
        c.x = 1.0;
    }
    {
        let mut i = pub_imu.loan_proxy::<Vector3>().expect("loan imu");
        i.x = 2.0;
    }
    rt.step(Duration::from_millis(1));
    assert_eq!(
        rt.node_handle("node").unwrap().fire_count(),
        1,
        "a Sync cdylib must fire once when BOTH inputs arrive within the window"
    );

    // NEGATIVE: only `cam` arrives → never fires (check_sync needs both).
    let mut rt2 = build_sync("ctsc2", CAM, IMU);
    let mgr2 = Arc::clone(rt2.test_transport().expect("test transport parked"));
    let mut pub_cam2 = mgr2
        .create_publisher(CAM, MaxSliceLen::const_new(64), 0)
        .expect("cam publisher 2");
    for _ in 0..5 {
        {
            let mut c = pub_cam2.loan_proxy::<Vector3>().expect("loan cam");
            c.x = 1.0;
        }
        rt2.step(Duration::from_millis(1));
    }
    assert_eq!(
        rt2.node_handle("node").unwrap().fire_count(),
        0,
        "a Sync cdylib must NOT fire when only one of its two inputs has arrived"
    );
}

// ===========================================================================
// (d) External (HostDriven) — fires via trigger_external + delivers
// ===========================================================================

#[test]
#[serial]
fn external_cdylib_fires_via_trigger_external_and_delivers() {
    let mut rt = build_single(
        "external_node",
        "test_node_macro_external_cdylib",
        vec![out_def("cmd")],
        vec![],
        "ctsx",
        8,
    );
    let mgr = Arc::clone(rt.test_transport().expect("test transport parked"));
    // Connect a subscriber to the external node's output BEFORE any fire.
    let mut cmd_sub = mgr
        .create_subscriber("/ctsx/node/cmd")
        .expect("open external cmd topic");
    let present = |sub: &mut cerulion_core::transport::subscriber::CerulionSubscriber| {
        sub.try_view::<Vector3, _>(|_v| {})
            .expect("try_view")
            .is_some()
    };

    // A silent step: External never self-fires under the polled seam.
    rt.step(Duration::from_millis(1));
    assert_eq!(
        rt.node_handle("node").unwrap().fire_count(),
        0,
        "an External node must NOT fire without trigger_external"
    );
    assert!(!present(&mut cmd_sub), "no fire ⇒ no delivery");

    // trigger_external + step → fires once and DELIVERS.
    rt.trigger_external("node").expect("trigger_external");
    rt.step(Duration::from_millis(1));
    assert_eq!(
        rt.node_handle("node").unwrap().fire_count(),
        1,
        "trigger_external + step must fire the External cdylib exactly once"
    );
    assert!(
        present(&mut cmd_sub),
        "the fired External cdylib must DELIVER its output frame"
    );

    // Another silent step → no re-fire, no new frame.
    rt.step(Duration::from_millis(1));
    assert_eq!(
        rt.node_handle("node").unwrap().fire_count(),
        1,
        "a silent step must not re-fire the External node"
    );
    assert!(!present(&mut cmd_sub), "no re-fire ⇒ no new delivery");
}

// ===========================================================================
// (e) Default-policy warn — raw-FFI cdylib (policy None, no trigger inputs)
// ===========================================================================

#[test]
#[serial]
#[tracing_test::traced_test]
fn raw_ffi_cdylib_no_policy_emits_default_policy_warn() {
    // `test_node_cdylib` is a raw-FFI node whose info JSON carries no "policy"
    // key and no inputs → policy None + no trigger inputs → the host's
    // `build_for_test` emits the loud `no macro-declared policy` warn (the same
    // phrase pinned by graph_default_policy_warn_test). The warn is HOST-side
    // (cerulion_core), so `#[traced_test]` captures it (the cdylib's own
    // tracing is irrelevant here).
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cts_warn".to_string(),
        prefix: "ctsw".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "raw".to_string(),
            node_type: "raw".to_string(),
            inputs: vec![],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("raw".to_string(), load("test_node_cdylib"));
    let clock = Arc::new(VirtualClock::new());
    // The warn fires in the per-node policy-resolution loop (after loading the
    // node's info) — BEFORE any port wiring — so it is captured regardless of
    // whether a portless node completes the rest of the build. Ignore the
    // build Result and pin the warn directly.
    let _ = GraphRuntime::build_for_test(config, factories, clock, 4);

    assert!(
        logs_contain("no macro-declared policy"),
        "a loaded raw-FFI cdylib with no policy + no trigger inputs must emit \
         the host's default-policy warn"
    );
    assert!(
        logs_contain("node_id=raw"),
        "the warn must carry the structured node_id field"
    );
}

// ===========================================================================
// (f) Inert-at-launch refusal — run_live refuses the loaded HostDriven cdylib
// ===========================================================================

#[test]
#[serial]
fn run_live_refuses_loaded_host_driven_cdylib() {
    // The loaded external cdylib's FFI `external_source()` returns HostDriven
    // (kind 3); `run_live`'s entry collect reads it and REFUSES the launch —
    // a provably-inert external node can never fire on the live path.
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cts_refuse".to_string(),
        prefix: "ctsr".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "camera".to_string(),
            node_type: "external_node".to_string(),
            inputs: vec![],
            outputs: vec![out_def("cmd")],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "camera".to_string(),
        load("test_node_macro_external_cdylib"),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut rt =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build refusal graph");

    let running = AtomicBool::new(false);
    let err = rt
        .run_live(&running)
        .expect_err("run_live must REFUSE a loaded HostDriven external cdylib");
    match err {
        cerulion_core::TransportError::ExternalNodesInertAtLaunch {
            ref graph,
            ref nodes,
        } => {
            assert_eq!(graph, "cts_refuse", "the refusal names the graph");
            assert_eq!(
                nodes.as_slice(),
                [("camera".to_string(), cerulion_core::InertReason::HostDriven)],
                "the refusal must name the loaded cdylib node + HostDriven reason \
                 (its external_source() crossed the FFI); got {nodes:?}"
            );
        }
        other => panic!("expected ExternalNodesInertAtLaunch, got {other:?}"),
    }
    let msg = err.to_string();
    assert!(
        msg.contains("trigger_external") && msg.contains("step()"),
        "the refusal must explain the fix (trigger_external / polled step()); got: {msg}"
    );
    assert_eq!(
        rt.node_handle("camera").unwrap().fire_count(),
        0,
        "a refused live run never fires the node"
    );
}
