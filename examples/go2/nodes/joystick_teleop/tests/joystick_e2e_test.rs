// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end: the JoystickTeleop node over a REAL graph runtime + iceoryx2,
//! driven by a SCRIPTED `PadEvent` stream (no hardware).
//!
//! # Seam
//!
//! `TransportManager::init_for_test` (isolated per-test SHM root) +
//! `GraphRuntime::build` with a caller-owned manager (the teleop_mux e2e
//! pattern). The node is injected via `JoystickTeleopEntry::with_state(..)`
//! sharing an `inbox` the test pushes scripted `PadEvent`s into; the node is
//! fired with `trigger_external` (its `external_source()` hardware pump is
//! NEVER queried on the polled path — Principle #7). A raw subscriber on the
//! node's derived output topic observes the published Twist stream.
//!
//! # What it pins
//!
//! - The BINDING startup-zero + no-idle-publish contract: one
//!   trigger with an empty inbox publishes EXACTLY ONE zero Twist; subsequent
//!   idle steps (no trigger) publish NOTHING.
//! - The full driving script (arm → move → release-zero → re-arm →
//!   disconnect-zero → reconnect-still-zero → re-arm) matches a HAND-BUILT
//!   oracle (never a re-run of the mapping under test).
//! - Determinism (Principle #7): two isolated runs are byte-identical AND
//!   both equal the oracle.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::CerulionSubscriber;
use indexmap::IndexMap;
use joystick_teleop::mapping::{PadAxis, PadEvent};
use joystick_teleop::{JoystickTeleop, JoystickTeleopEntry, MAX_VX, MAX_VY, MAX_VYAW};

/// Step size (arbitrary — the joystick's output is timing-free; the node
/// reads no clock, so `dt` only advances the virtual clock the fire schedule
/// rides).
const STEP: Duration = Duration::from_millis(50);

/// A decoded output frame: `(linear.x, linear.y, angular.z)` = `(vx, vy, vyaw)`.
type Cmd3 = (f64, f64, f64);

struct Rig {
    rt: GraphRuntime,
    inbox: Arc<Mutex<Vec<PadEvent>>>,
    obs: CerulionSubscriber,
}

fn build_rig(prefix: &str) -> Rig {
    let clock = Arc::new(VirtualClock::new());
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("joy_e2e_{prefix}"),
            clock: clock.clone(),
            subscriber_buffer_size: 16,
            network: None,
        },
        ix_config,
    )
    .expect("init isolated e2e transport");

    let inbox: Arc<Mutex<Vec<PadEvent>>> = Arc::new(Mutex::new(Vec::new()));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("joy_e2e_{prefix}"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "joystick".to_string(),
            node_type: "joystick_teleop".to_string(),
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
        "joystick".to_string(),
        Box::new(JoystickTeleopEntry::with_state(JoystickTeleop::with_inbox(
            Arc::clone(&inbox),
        ))),
    );

    let rt = GraphRuntime::build(config, factories, &mgr, clock).expect("joystick graph builds");

    let obs = mgr
        .create_subscriber(&format!("/{prefix}/joystick/cmd"))
        .expect("observer subscriber on the joystick output");

    Rig { rt, inbox, obs }
}

/// Push scripted events into the shared inbox (the next fire drains them).
fn inject(rig: &Rig, events: &[PadEvent]) {
    let mut q = rig.inbox.lock().expect("inbox not poisoned");
    q.extend_from_slice(events);
}

/// Drain ALL queued output frames, decoding each to `(vx, vy, vyaw)`.
/// Twist wire layout: linear.x[0..8], linear.y[8..16], linear.z[16..24],
/// angular.x[24..32], angular.y[32..40], angular.z[40..48].
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

/// One FIRED beat: inject events, `trigger_external` + `step`, drain.
fn fire(rig: &mut Rig, events: &[PadEvent]) -> Vec<Cmd3> {
    inject(rig, events);
    rig.rt
        .trigger_external("joystick")
        .expect("trigger_external joystick");
    rig.rt.step(STEP);
    drain(&rig.obs)
}

/// One IDLE beat: `step` with NO trigger (the node must not fire), drain.
fn idle(rig: &mut Rig) -> Vec<Cmd3> {
    rig.rt.step(STEP);
    drain(&rig.obs)
}

/// Fire one beat and assert it published EXACTLY one frame (one fire
/// ⇒ one publish), returning that frame.
fn one_fire(rig: &mut Rig, events: &[PadEvent]) -> Cmd3 {
    let frames = fire(rig, events);
    assert_eq!(
        frames.len(),
        1,
        "one fire ⇒ one publish, got {} frames: {frames:?}",
        frames.len()
    );
    frames[0]
}

const ZERO: Cmd3 = (0.0, 0.0, 0.0);

/// Run the full scripted session on an isolated transport. Asserts the
/// binding startup-zero + no-idle pins INLINE, and returns the driving-beat
/// output sequence for the hand-oracle comparison.
fn run_session(prefix: &str) -> Vec<Cmd3> {
    let mut rig = build_rig(prefix);

    // ---- Startup zero (binding): ONE trigger, empty inbox → exactly one
    // published zero. Accumulate across the fire + three idle steps so a
    // one-cycle-late first delivery is still counted exactly once (idle steps
    // publish NOTHING, so the total can only be the single startup frame).
    let mut startup: Vec<Cmd3> = fire(&mut rig, &[]);
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

    // ---- No-idle-publish (binding), reconfirmed: more idle steps → nothing.
    for _ in 0..4 {
        assert!(
            idle(&mut rig).is_empty(),
            "the joystick must publish NOTHING while idle after the startup zero"
        );
    }

    // ---- Driving script. The single frame of each fired beat is recorded.
    // (`vec![]` arguments evaluate left-to-right, so the effectful one_fire
    // beats run in exactly this order; each `&mut rig` borrow ends before the
    // next argument.)
    let mut seq: Vec<Cmd3> = vec![
        // B1: connect + arm (RB) + full forward.
        one_fire(
            &mut rig,
            &[
                PadEvent::Connected,
                PadEvent::Deadman(true),
                PadEvent::Axis(PadAxis::LeftY, 1.0),
            ],
        ),
        // B2: keepalive (no new events) — latched forward.
        one_fire(&mut rig, &[]),
        // B3: add strafe LEFT (stick left, LeftStickX = -1) ⇒ +y.
        one_fire(&mut rig, &[PadEvent::Axis(PadAxis::LeftX, -1.0)]),
        // B4: add yaw RIGHT (stick right, RightStickX = +1) ⇒ -z.
        one_fire(&mut rig, &[PadEvent::Axis(PadAxis::RightX, 1.0)]),
        // B5: release the deadman → one zero.
        one_fire(&mut rig, &[PadEvent::Deadman(false)]),
    ];

    // ---- No-idle after release: quiet.
    for _ in 0..3 {
        assert!(
            idle(&mut rig).is_empty(),
            "after a deadman-release zero the joystick must go quiet"
        );
    }

    // B6: re-arm → resumes the LATCHED axes (forward + strafe-left + yaw-right).
    seq.push(one_fire(&mut rig, &[PadEvent::Deadman(true)]));
    // B7: Bluetooth drop → one zero (disconnect zeroes AND disarms).
    seq.push(one_fire(&mut rig, &[PadEvent::Disconnected]));
    // B8: reconnect alone does NOT re-arm → still zero.
    seq.push(one_fire(&mut rig, &[PadEvent::Connected]));
    // B9: re-press RB → resumes the latched axes.
    seq.push(one_fire(&mut rig, &[PadEvent::Deadman(true)]));

    rig.rt.shutdown();
    seq
}

/// HAND-BUILT oracle for the driving-beat sequence (values derived by hand
/// from the documented mapping + sign conventions — never via the mapping fn).
fn hand_oracle() -> Vec<Cmd3> {
    vec![
        (MAX_VX, 0.0, 0.0),          // B1 forward
        (MAX_VX, 0.0, 0.0),          // B2 latched forward
        (MAX_VX, MAX_VY, 0.0),       // B3 + strafe LEFT (+y)
        (MAX_VX, MAX_VY, -MAX_VYAW), // B4 + yaw RIGHT (-z)
        ZERO,                        // B5 release
        (MAX_VX, MAX_VY, -MAX_VYAW), // B6 re-arm, latched
        ZERO,                        // B7 disconnect
        ZERO,                        // B8 reconnect, not re-armed
        (MAX_VX, MAX_VY, -MAX_VYAW), // B9 re-arm
    ]
}

#[test]
fn joystick_e2e_startup_zero_no_idle_oracle_and_deterministic() {
    let oracle = hand_oracle();

    let run_a = run_session("j672a");
    assert_eq!(
        run_a, oracle,
        "run A driving sequence must equal the hand oracle"
    );

    // Determinism: a fresh isolated run is byte-identical AND equals the
    // oracle (two-run equality alone would pass a both-wrong bug).
    let run_b = run_session("j672b");
    assert_eq!(run_b, oracle, "run B must equal the hand oracle");
    assert_eq!(run_a, run_b, "the two runs must be byte-identical");
}
