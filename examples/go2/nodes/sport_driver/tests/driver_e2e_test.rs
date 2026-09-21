// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end: the SportDriver node over a REAL graph runtime + iceoryx2,
//! with the DDS writer replaced by the captured-sink seam (no DDS, no
//! multicast: this runs on any CI runner).
//!
//! # Seam
//!
//! `TransportManager::init_for_test` (isolated per-test SHM root) +
//! `GraphRuntime::build` with a caller-owned manager (the teleop_mux e2e
//! precedent). The driver's trigger input is wired to an ABSOLUTE external
//! topic with no in-graph producer; a raw external publisher scripts the
//! arbitrated commands, one frame per beat, and the captured sink is read
//! back after each step.
//!
//! # Oracle discipline
//!
//! Every captured request is compared against a HAND-BUILT oracle of
//! (api_id, parameter) pairs, never against a re-run of the policy. The
//! request identities must climb by exactly one per request sent. The whole
//! script runs twice on isolated transports and must be byte-identical.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::CerulionPublisher;
use cerulion_go2_dds::messages::Request;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Twist;
use sport_driver::{SportDriver, SportDriverEntry};

/// One beat = one 20 ms step (the mux's cadence upstream of this node).
const STEP: Duration = Duration::from_millis(20);

const MOVE: i64 = 1008;
const STOP: i64 = 1003;

struct Rig {
    rt: GraphRuntime,
    cmd: CerulionPublisher,
    captured: Arc<Mutex<Vec<Request>>>,
}

fn build_rig(prefix: &str) -> Rig {
    let clock = Arc::new(VirtualClock::new());
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("sport_driver_e2e_{prefix}"),
            clock: clock.clone(),
            subscriber_buffer_size: 16,
            network: None,
        },
        ix_config,
    )
    .expect("init isolated e2e transport");

    let cmd_topic = format!("/e_sd/{prefix}/cmd_vel");
    let config = GraphConfig {
        network: None,
        level_assignments: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("sport_driver_e2e_{prefix}"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "sport".to_string(),
            node_type: "sport_driver".to_string(),
            inputs: vec![InputDef {
                name: "cmd".to_string(),
                source: cmd_topic.clone(),
            }],
            outputs: vec![],
        }],
    };
    let captured: Arc<Mutex<Vec<Request>>> = Arc::new(Mutex::new(Vec::new()));
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "sport".to_string(),
        Box::new(SportDriverEntry::with_state(
            SportDriver::with_captured_sink(Arc::clone(&captured)),
        )),
    );
    let rt = GraphRuntime::build(config, factories, &mgr, clock)
        .expect("the driver graph must build over an absolute external source");
    let cmd = mgr
        .create_publisher(&cmd_topic, MaxSliceLen::const_new(256), 0)
        .expect("external command publisher attaches");
    Rig { rt, cmd, captured }
}

fn publish(publisher: &mut CerulionPublisher, vx: f64, vy: f64, vyaw: f64) {
    let mut proxy = publisher.loan_proxy::<Twist>().expect("loan Twist");
    proxy.linear.x = vx;
    proxy.linear.y = vy;
    proxy.linear.z = 0.0;
    proxy.angular.x = 0.0;
    proxy.angular.y = 0.0;
    proxy.angular.z = vyaw;
}

/// Publish one command, step one beat, return the requests captured so far
/// as (api_id, parameter) pairs.
fn beat(rig: &mut Rig, vx: f64, vy: f64, vyaw: f64) -> Vec<(i64, String)> {
    publish(&mut rig.cmd, vx, vy, vyaw);
    rig.rt.step(STEP);
    rig.captured
        .lock()
        .unwrap()
        .iter()
        .map(|r| (r.header.identity.api_id, r.parameter.clone()))
        .collect()
}

fn run_script(prefix: &str) -> Vec<(i64, String)> {
    let mut rig = build_rig(prefix);

    // Establishment: re-publish a Move each beat until the first request
    // lands (the initial iceoryx2 connection is the ONLY latency here).
    // Every landed frame is a Move, so the count of captured requests
    // tells how many of the establishment frames actually crossed.
    let mut tries = 0;
    let established = loop {
        let got = beat(&mut rig, 0.5, 0.0, 0.25);
        if !got.is_empty() {
            break got.len();
        }
        tries += 1;
        assert!(tries < 200, "no command reached the driver in 200 beats");
    };
    for r in rig.captured.lock().unwrap().iter() {
        assert_eq!(r.header.identity.api_id, MOVE);
        assert_eq!(r.parameter, "{\"x\":0.5,\"y\":0,\"z\":0.25}");
    }

    // The scripted beats (each publish is one fire of the trigger input).
    // The clock advances 20 ms per beat, so a StopMove sent at beat s is due
    // again at beat s + 10.
    let script: [(f64, f64, f64); 15] = [
        (0.5, 0.0, 0.25), // Move
        (0.0, 0.0, 0.0),  // transition: StopMove (beat s)
        (0.0, 0.0, 0.0),  // s+1 .. s+9: nothing
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),      // s+10: keepalive StopMove
        (5.0, -3.0, 9.0),     // clamped Move
        (f64::NAN, 0.0, 0.0), // non-finite after motion: StopMove
        (0.0, 0.0, 0.5),      // Move (yaw only)
    ];
    for (vx, vy, vyaw) in script {
        let _ = beat(&mut rig, vx, vy, vyaw);
    }
    let mut all = beat(&mut rig, 0.0, 0.0, 0.0); // transition: StopMove

    // Identities climb by exactly one per request, from 1.
    let ids: Vec<i64> = rig
        .captured
        .lock()
        .unwrap()
        .iter()
        .map(|r| r.header.identity.id)
        .collect();
    let expected_ids: Vec<i64> = (1..=ids.len() as i64).collect();
    assert_eq!(
        ids, expected_ids,
        "request identities must be gap-free from 1"
    );

    // Return only the scripted part (after the establishment Moves).
    all.split_off(established)
}

#[test]
fn the_driver_maps_arbitrated_commands_to_sport_requests_end_to_end() {
    let got = run_script("sd1");
    let oracle: Vec<(i64, String)> = vec![
        (MOVE, "{\"x\":0.5,\"y\":0,\"z\":0.25}".to_string()),
        (STOP, String::new()),
        (STOP, String::new()),
        (MOVE, "{\"x\":0.6,\"y\":-0.4,\"z\":1}".to_string()),
        (STOP, String::new()),
        (MOVE, "{\"x\":0,\"y\":0,\"z\":0.5}".to_string()),
        (STOP, String::new()),
    ];
    assert_eq!(got, oracle);
}

#[test]
fn the_driver_is_deterministic_across_two_isolated_runs() {
    let a = run_script("sd2a");
    let b = run_script("sd2b");
    assert_eq!(a, b);
    assert_eq!(a.len(), 7, "the script sends exactly seven requests");
}
