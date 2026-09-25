// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end: the KeyboardTeleop node over a REAL graph runtime + iceoryx2,
//! driven by a SCRIPTED `KeyOp` stream (no terminal).
//!
//! # Seam
//!
//! `TransportManager::init_for_test` (isolated per-test SHM root) +
//! `GraphRuntime::build` with a caller-owned manager (the teleop_mux e2e
//! pattern). The node is injected via `KeyboardTeleopEntry::with_state(..)`
//! sharing an `inbox` the test pushes scripted `KeyOp`s into; it is fired with
//! `trigger_external` (its `external_source()` crossterm pump is NEVER queried
//! on the polled path — Principle #7). A raw subscriber on the derived output
//! topic observes the published Twist stream. One shared `VirtualClock` drives
//! the manager, the runtime, and the node's `now_ns()`, so every auto-zero age
//! in the script is exact.
//!
//! # What it pins
//!
//! - The BINDING startup-zero + no-idle-publish contract.
//! - The AUTO-ZERO pin: after a keypress, keepalive republishes the latched
//!   command until exactly `AUTO_ZERO_NS` of keyboard silence, then the node
//!   emits ONE zero and goes quiet.
//! - `space` immediate zero; latched multi-axis with the documented signs.
//! - The full driving sequence matches a HAND oracle; determinism (two
//!   isolated runs byte-identical AND both equal the oracle).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::CerulionSubscriber;
use indexmap::IndexMap;
use keyboard_teleop::keymap::KeyOp;
use keyboard_teleop::{KeyboardTeleop, KeyboardTeleopEntry, MAX_VX, MAX_VY, MAX_VYAW};

/// 100 ms step — matches the 10 Hz keepalive and makes the 500 ms auto-zero
/// window exactly 5 steps, so the clock arithmetic in the oracle is exact.
const STEP: Duration = Duration::from_millis(100);

type Cmd3 = (f64, f64, f64);
const ZERO: Cmd3 = (0.0, 0.0, 0.0);

struct Rig {
    rt: GraphRuntime,
    inbox: Arc<Mutex<Vec<KeyOp>>>,
    obs: CerulionSubscriber,
}

fn build_rig(prefix: &str) -> Rig {
    let clock = Arc::new(VirtualClock::new());
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("key_e2e_{prefix}"),
            clock: clock.clone(),
            subscriber_buffer_size: 16,
            network: None,
        },
        ix_config,
    )
    .expect("init isolated e2e transport");

    let inbox: Arc<Mutex<Vec<KeyOp>>> = Arc::new(Mutex::new(Vec::new()));

    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("key_e2e_{prefix}"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "keyboard".to_string(),
            node_type: "keyboard_teleop".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "cmd".to_string(),
                schema: "geometry_msgs/Twist".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "keyboard".to_string(),
        Box::new(KeyboardTeleopEntry::with_state(KeyboardTeleop::with_inbox(
            Arc::clone(&inbox),
        ))),
    );

    let rt = GraphRuntime::build(config, factories, &mgr, clock).expect("keyboard graph builds");

    let obs = mgr
        .create_subscriber(&format!("/{prefix}/keyboard/cmd"))
        .expect("observer subscriber on the keyboard output");

    Rig { rt, inbox, obs }
}

fn inject(rig: &Rig, ops: &[KeyOp]) {
    let mut q = rig.inbox.lock().expect("inbox not poisoned");
    q.extend_from_slice(ops);
}

/// Drain ALL queued frames, decoding `(linear.x, linear.y, angular.z)`.
fn drain(obs: &CerulionSubscriber) -> Vec<Cmd3> {
    let mut out = Vec::new();
    obs.try_receive(|msg| {
        let p = msg.payload();
        let vx = f64::from_le_bytes(p[0..8].try_into().expect("linear.x 8 bytes"));
        let vy = f64::from_le_bytes(p[8..16].try_into().expect("linear.y 8 bytes"));
        let vyaw = f64::from_le_bytes(p[40..48].try_into().expect("angular.z 8 bytes"));
        out.push((vx, vy, vyaw));
    })
    .expect("drain observer");
    out
}

/// One FIRED beat: inject ops, `trigger_external` + `step`, drain.
fn fire(rig: &mut Rig, ops: &[KeyOp]) -> Vec<Cmd3> {
    inject(rig, ops);
    rig.rt
        .trigger_external("keyboard")
        .expect("trigger_external keyboard");
    rig.rt.step(STEP);
    drain(&rig.obs)
}

/// One IDLE beat: `step` with NO trigger, drain (the node must not fire).
fn idle(rig: &mut Rig) -> Vec<Cmd3> {
    rig.rt.step(STEP);
    drain(&rig.obs)
}

/// Fire one beat and assert it published EXACTLY one frame (one fire
/// ⇒ one publish), returning that frame.
fn one_fire(rig: &mut Rig, ops: &[KeyOp]) -> Cmd3 {
    let frames = fire(rig, ops);
    assert_eq!(
        frames.len(),
        1,
        "one fire ⇒ one publish, got {} frames: {frames:?}",
        frames.len()
    );
    frames[0]
}

/// Run the full scripted session on an isolated transport. Asserts the binding
/// startup-zero + no-idle pins and the auto-zero-then-quiet pin INLINE, and
/// returns the recorded driving-beat sequence for the hand-oracle comparison.
fn run_session(prefix: &str) -> Vec<Cmd3> {
    let mut rig = build_rig(prefix);

    // ---- Startup zero (binding): ONE trigger, empty inbox → exactly one
    // published zero (accumulated across three idle steps — a one-cycle-late
    // first delivery is still counted exactly once; idle publishes nothing).
    let mut startup = fire(&mut rig, &[]);
    for _ in 0..3 {
        startup.extend(idle(&mut rig));
    }
    assert_eq!(
        startup.len(),
        1,
        "exactly ONE startup zero must be published (got {startup:?})"
    );
    assert_eq!(
        startup[0], ZERO,
        "the startup frame must be an all-zero Twist"
    );

    // ---- Forward, then 4 keepalive republishes, then the AUTO-ZERO at exactly
    // 5 * 100 ms = 500 ms of keyboard silence since the keypress.
    // (`vec![]` arguments evaluate left-to-right, so the effectful one_fire
    // beats run in exactly this order; each `&mut rig` borrow ends before the
    // next argument.)
    let mut seq: Vec<Cmd3> = vec![
        one_fire(&mut rig, &[KeyOp::Forward]), // t=500ms: latch forward
        one_fire(&mut rig, &[]),               // t=600ms: age 100ms → still forward
        one_fire(&mut rig, &[]),               // t=700ms: age 200ms → forward
        one_fire(&mut rig, &[]),               // t=800ms: age 300ms → forward
        one_fire(&mut rig, &[]),               // t=900ms: age 400ms → forward
        one_fire(&mut rig, &[]),               // t=1000ms: age 500ms → AUTO-ZERO
    ];

    // ---- Quiet after auto-zero: idle steps publish nothing.
    for _ in 0..3 {
        assert!(
            idle(&mut rig).is_empty(),
            "after the auto-zero the keyboard must go quiet (no idle republish)"
        );
    }

    // ---- Re-arm with strafe + yaw (latched multi-axis, documented signs),
    // then `space` immediate zero.
    seq.push(one_fire(&mut rig, &[KeyOp::StrafeLeft])); // strafe LEFT → +y
    seq.push(one_fire(&mut rig, &[KeyOp::YawRight])); // + yaw RIGHT → -z (strafe latched)
    seq.push(one_fire(&mut rig, &[KeyOp::Stop])); // space → immediate zero

    // ---- Quiet after the space-stop.
    for _ in 0..3 {
        assert!(
            idle(&mut rig).is_empty(),
            "after a space-stop the keyboard must go quiet"
        );
    }

    rig.rt.shutdown();
    seq
}

/// HAND-BUILT oracle (values + auto-zero boundary derived by hand from the
/// documented mapping + the 500 ms window — never via the keymap fn).
fn hand_oracle() -> Vec<Cmd3> {
    vec![
        (MAX_VX, 0.0, 0.0),       // forward
        (MAX_VX, 0.0, 0.0),       // keepalive (age 100ms)
        (MAX_VX, 0.0, 0.0),       // keepalive (age 200ms)
        (MAX_VX, 0.0, 0.0),       // keepalive (age 300ms)
        (MAX_VX, 0.0, 0.0),       // keepalive (age 400ms)
        ZERO,                     // AUTO-ZERO at age 500ms (inclusive)
        (0.0, MAX_VY, 0.0),       // strafe LEFT (+y)
        (0.0, MAX_VY, -MAX_VYAW), // + yaw RIGHT (-z), strafe still latched
        ZERO,                     // space stop
    ]
}

#[test]
fn keyboard_e2e_startup_zero_auto_zero_oracle_and_deterministic() {
    let oracle = hand_oracle();

    let run_a = run_session("k673a");
    assert_eq!(run_a, oracle, "run A must equal the hand oracle");

    let run_b = run_session("k673b");
    assert_eq!(run_b, oracle, "run B must equal the hand oracle");
    assert_eq!(run_a, run_b, "the two runs must be byte-identical");
}
