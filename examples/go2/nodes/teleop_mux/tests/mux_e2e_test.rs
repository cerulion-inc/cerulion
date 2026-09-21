// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end: the TeleopMux node over a REAL graph runtime + iceoryx2.
//!
//! # Seam
//!
//! `TransportManager::init_for_test` (isolated per-test SHM root) +
//! `GraphRuntime::build` with a caller-owned manager — the
//! `non_trigger_hold_iox2_test` absolute-source pattern. The mux's two
//! inputs are wired to ABSOLUTE external topics with no in-graph producer;
//! the graph builds FIRST (creating those services at the held-input
//! snapshot-source `subscriber_max_borrowed_samples = 3`), then raw
//! external publishers attach and script the sources. A raw subscriber on
//! the mux's derived output topic observes the arbitrated stream. The
//! manager, the runtime, and every wire stamp share ONE `VirtualClock`, so
//! every freshness age in the script is exact.
//!
//! # The headline pin (the hold interplay)
//!
//! A silent-but-HELD source goes stale by TIMESTAMP while its held VALUE is
//! still readable: in phase 2 the joystick stops publishing but its held
//! frame keeps serving `JOY_X` — the mux must keep emitting `JOY_X` while
//! the held stamp is fresh (beats 1..=11, ages 40..=240 ms) and flip to the
//! keyboard at the first fire past the 250 ms window (beat 12, age 260 ms),
//! with the joystick's value STILL readable the whole time. This is exactly
//! what `InputView::wire_timestamp_ns()` exists for. Then phase 3 repeats the
//! shape for the keyboard's 750 ms window (death → safety zero), and phase 4
//! pins the centered-fresh-joystick-mutes-keyboard case and its release.
//!
//! # Oracle discipline
//!
//! The full per-beat output sequence is asserted against a HAND-BUILT
//! oracle vector (never a re-run of `arbitrate()` — that would be a
//! tautology against the module under test). Determinism: the whole script
//! runs twice on isolated transports and must be byte-identical AND equal
//! to the oracle.
//!
//! # Timing model (documented assumption)
//!
//! Beats are: publish (stamped at the pre-step clock) → `rt.step(20 ms)` →
//! drain. iceoryx2 delivers a sent sample into connected subscriber queues
//! synchronously (same process), so a pre-step publish is visible to that
//! step's snapshot; the ONLY establishment-latency tolerance needed is the
//! initial connection, absorbed by the bounded gate loop (which re-publishes
//! its startup zeros each iteration). No wall-clock sleeps anywhere.

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{CerulionPublisher, CerulionSubscriber};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Twist;
use teleop_mux::TeleopMuxEntry;

/// One beat = one 20 ms step (== the mux's `period_ms`, so the mux fires
/// exactly once per beat once its period schedule is caught up).
const STEP: Duration = Duration::from_millis(20);
/// Joystick linear.x while driving (distinct from the keyboard's so a
/// wrong-source bug shows as a wrong VALUE).
const JOY_X: f64 = 2.0;
/// Keyboard linear.x while driving.
const KEY_X: f64 = 0.5;

/// The e2e rig: one shared `VirtualClock` drives the manager (publisher
/// wire stamps), the runtime (scheduler + node `now_ns()`), and therefore
/// every freshness age in the script.
struct Rig {
    rt: GraphRuntime,
    joy: CerulionPublisher,
    key: CerulionPublisher,
    obs: CerulionSubscriber,
}

fn build_rig(prefix: &str) -> Rig {
    let clock = Arc::new(VirtualClock::new());
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("mux_e2e_{prefix}"),
            clock: clock.clone(),
            subscriber_buffer_size: 16,
            network: None,
        },
        ix_config,
    )
    .expect("init isolated e2e transport");

    let joy_topic = format!("/teleop/{prefix}/joy");
    let key_topic = format!("/teleop/{prefix}/key");

    let config = GraphConfig {
        network: None,
        level_assignments: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("mux_e2e_{prefix}"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "mux".to_string(),
            node_type: "teleop_mux".to_string(),
            inputs: vec![
                InputDef {
                    name: "joy_cmd".to_string(),
                    source: joy_topic.clone(),
                },
                InputDef {
                    name: "key_cmd".to_string(),
                    source: key_topic.clone(),
                },
            ],
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
    factories.insert("mux".to_string(), Box::new(TeleopMuxEntry::new()));

    // Graph builds FIRST: the mux HOLDS its non-trigger inputs, so the two
    // absolute external source services are created at the borrow-3
    // floor; the raw publishers below then attach (External provisioning
    // carries no port caps).
    let rt = GraphRuntime::build(config, factories, &mgr, clock).expect(
        "the mux graph must build — absolute external snapshot sources are \
         created by the consumer graph at subscriber_max_borrowed_samples=3",
    );

    let joy = mgr
        .create_publisher(&joy_topic, MaxSliceLen::const_new(256), 0)
        .expect("external joystick publisher attaches");
    let key = mgr
        .create_publisher(&key_topic, MaxSliceLen::const_new(256), 0)
        .expect("external keyboard publisher attaches");
    // The observer rides one of the output topic's introspection-headroom
    // subscriber slots.
    let obs = mgr
        .create_subscriber(&format!("/{prefix}/mux/cmd"))
        .expect("observer subscriber on the mux output");

    Rig { rt, joy, key, obs }
}

/// Publish a Twist whose `linear.x` is `lx` (all other components 0.0),
/// wire-stamped at the manager clock's CURRENT time (loan-time stamping).
fn publish_x(publisher: &mut CerulionPublisher, lx: f64) {
    let mut proxy = publisher.loan_proxy::<Twist>().expect("loan Twist");
    proxy.linear.x = lx;
    proxy.linear.y = 0.0;
    proxy.linear.z = 0.0;
    proxy.angular.x = 0.0;
    proxy.angular.y = 0.0;
    proxy.angular.z = 0.0;
    // Drop publishes (direct-path proxy).
}

/// Drain ALL queued output frames, returning each frame's `linear.x`
/// (payload bytes `[0..8]`, LE — the fixed section starts at payload 0).
fn drain_x(obs: &CerulionSubscriber) -> Vec<f64> {
    let mut xs = Vec::new();
    obs.try_receive(|msg| {
        xs.push(f64::from_le_bytes(
            msg.payload()[0..8].try_into().expect("linear.x is 8 bytes"),
        ));
    })
    .expect("drain observer");
    xs
}

/// One scripted beat: run the publishes closure (stamps = pre-step clock),
/// step 20 ms (the mux fires once), drain. Returns all frames' `linear.x`.
fn beat(rig: &mut Rig, publishes: impl FnOnce(&mut Rig)) -> Vec<f64> {
    publishes(rig);
    rig.rt.step(STEP);
    drain_x(&rig.obs)
}

/// Run the full arbitration script on an isolated transport, returning the
/// per-beat output sequence (LAST frame per beat) across the deterministic
/// windows. Panics on any gate/liveness violation. See the module docs for
/// the phase-by-phase age arithmetic.
fn run_script(prefix: &str) -> Vec<f64> {
    let mut rig = build_rig(prefix);
    let mut seq: Vec<f64> = Vec::new();

    // ---- AND-gate pin: neither source has EVER delivered → the mux's
    // tick collapses and publishes NOTHING, despite its Period
    // trigger firing every beat.
    for b in 0..5 {
        let frames = beat(&mut rig, |_| {});
        assert!(
            frames.is_empty(),
            "pre-delivery beat {b}: the AND-gate must hold — no output until \
             BOTH sources have delivered once (got {frames:?})"
        );
    }

    // ---- Gate opens on the startup zeros (the source-node
    // contract). Bounded: re-publish the zeros each iteration so the very
    // first CONNECTED delivery opens the gate; output is 0.0 in every
    // freshness regime here, so the value assert is establishment-robust.
    let mut tries = 0;
    loop {
        let frames = beat(&mut rig, |r| {
            publish_x(&mut r.joy, 0.0);
            publish_x(&mut r.key, 0.0);
        });
        if let Some(&last) = frames.last() {
            assert_eq!(
                last, 0.0,
                "the first post-gate output must be the startup zero (got {last})"
            );
            break;
        }
        tries += 1;
        assert!(
            tries < 200,
            "the AND-gate never opened within 200 startup-zero beats"
        );
    }

    // ---- phase 1 (5 beats): both sources fresh + driving → joystick wins
    // (the keyboard's 0.5 is on the wire and MUST be ignored).
    //
    // Two WARM beats first (drained + discarded, not in the oracle): the
    // phase ENTRY is the one place a same-beat vs one-beat-lagged iceoryx2
    // delivery would change the observed value (the held frame flips zero →
    // JOY_X here); warming saturates the pipeline without touching the
    // stamp math (joy's P2 anchor stays the LAST recorded-beat publish).
    for _ in 0..2 {
        let _ = beat(&mut rig, |r| {
            publish_x(&mut r.joy, JOY_X);
            publish_x(&mut r.key, KEY_X);
        });
    }
    for b in 0..5 {
        let frames = beat(&mut rig, |r| {
            publish_x(&mut r.joy, JOY_X);
            publish_x(&mut r.key, KEY_X);
        });
        let last = *frames
            .last()
            .unwrap_or_else(|| panic!("phase 1 beat {b}: the mux must publish every fired beat"));
        seq.push(last);
    }

    // ---- phase 2 (18 beats) — THE HEADLINE: the joystick goes SILENT but
    // its held frame keeps serving JOY_X. Ages at fire: 20+20b ms after its
    // last stamp (phase-1 beat 5). Beats 1..=11 (ages 40..=240) the held
    // stamp is FRESH → JOY_X still drives; beat 12 (age 260 ≥ 250) the held
    // frame is STALE BY TIMESTAMP — value still readable! — and the mux
    // flips to the fresh keyboard. The keyboard republishes every beat.
    for b in 0..18 {
        let frames = beat(&mut rig, |r| {
            publish_x(&mut r.key, KEY_X);
        });
        let last = *frames
            .last()
            .unwrap_or_else(|| panic!("phase 2 beat {b}: the mux must publish every fired beat"));
        seq.push(last);
    }

    // ---- phase 3 (40 beats): the keyboard dies too. Its HELD frame drives
    // (ages 20+20b after its last stamp, phase-2 beat 18) through beat 36
    // (age 740 < 750); beat 37 (age 760) → both stale → safety zero.
    for b in 0..40 {
        let frames = beat(&mut rig, |_| {});
        let last = *frames
            .last()
            .unwrap_or_else(|| panic!("phase 3 beat {b}: the mux must publish every fired beat"));
        seq.push(last);
    }

    // ---- phase 4a (5 beats): a fresh CENTERED joystick (all-zero cmd) +
    // a fresh nonzero keyboard → the joystick's ZERO drives (a centered
    // stick is a real command that MUTES the keyboard).
    //
    // Two WARM beats (drained + discarded) for the same phase-entry
    // delivery-lag tolerance as phase 1 (the held joy flips stale → fresh
    // zero here); joy's P4b stamp anchor stays the LAST recorded beat.
    for _ in 0..2 {
        let _ = beat(&mut rig, |r| {
            publish_x(&mut r.joy, 0.0);
            publish_x(&mut r.key, KEY_X);
        });
    }
    for b in 0..5 {
        let frames = beat(&mut rig, |r| {
            publish_x(&mut r.joy, 0.0);
            publish_x(&mut r.key, KEY_X);
        });
        let last = *frames
            .last()
            .unwrap_or_else(|| panic!("phase 4a beat {b}: the mux must publish every fired beat"));
        seq.push(last);
    }

    // ---- phase 4b (14 beats): the centered joystick goes silent; its held
    // ZERO keeps muting the keyboard while fresh by stamp (beats 1..=11),
    // then the mute releases → keyboard (beats 12..=14).
    for b in 0..14 {
        let frames = beat(&mut rig, |r| {
            publish_x(&mut r.key, KEY_X);
        });
        let last = *frames
            .last()
            .unwrap_or_else(|| panic!("phase 4b beat {b}: the mux must publish every fired beat"));
        seq.push(last);
    }

    rig.rt.shutdown();
    seq
}

/// HAND-BUILT oracle for the windowed sequence (phase-by-phase, ages
/// derived by hand in the phase comments above — never via `arbitrate()`).
fn hand_oracle() -> Vec<f64> {
    let mut o = Vec::new();
    o.extend(std::iter::repeat_n(JOY_X, 5)); // P1: joy drives
    o.extend(std::iter::repeat_n(JOY_X, 11)); // P2 beats 1..=11: HELD joy, fresh by stamp
    o.extend(std::iter::repeat_n(KEY_X, 7)); // P2 beats 12..=18: held joy STALE → keyboard
    o.extend(std::iter::repeat_n(KEY_X, 36)); // P3 beats 1..=36: HELD key, fresh by stamp
    o.extend(std::iter::repeat_n(0.0, 4)); // P3 beats 37..=40: both stale → zero
    o.extend(std::iter::repeat_n(0.0, 5)); // P4a: fresh centered joy mutes keyboard
    o.extend(std::iter::repeat_n(0.0, 11)); // P4b beats 1..=11: held centered joy still mutes
    o.extend(std::iter::repeat_n(KEY_X, 3)); // P4b beats 12..=14: mute released → keyboard
    o
}

/// The full script matches the hand oracle, and a second isolated run is
/// byte-identical (determinism, Principle #7). One `#[test]` body so the two
/// isolated transports never coexist with a third (fd headroom on macOS).
#[test]
fn mux_e2e_script_matches_hand_oracle_and_is_deterministic() {
    let oracle = hand_oracle();

    let run_a = run_script("runa");
    assert_eq!(
        run_a.len(),
        oracle.len(),
        "run A length must match the oracle ({} beats)",
        oracle.len()
    );
    // Phase-sliced comparison first so a failure names the phase.
    let bounds = [
        (0usize, 5usize, "P1 joy drives"),
        (
            5,
            16,
            "P2 held-joy fresh-by-stamp drives (the headline hold)",
        ),
        (
            16,
            23,
            "P2 held-joy stale-by-stamp -> keyboard (the headline flip)",
        ),
        (23, 59, "P3 held-key fresh-by-stamp drives"),
        (59, 63, "P3 both stale -> safety zero"),
        (63, 68, "P4a fresh centered joy mutes keyboard"),
        (68, 79, "P4b held centered joy still mutes"),
        (79, 82, "P4b mute released -> keyboard"),
    ];
    for &(lo, hi, label) in &bounds {
        assert_eq!(
            &run_a[lo..hi],
            &oracle[lo..hi],
            "run A diverged from the hand oracle in phase [{label}] \
             (beats {lo}..{hi}): got {:?}, want {:?}",
            &run_a[lo..hi],
            &oracle[lo..hi]
        );
    }
    assert_eq!(run_a, oracle, "run A must equal the full hand oracle");

    // Determinism: an independent isolated run is byte-identical AND equals
    // the oracle (two-run equality alone would pass a both-wrong bug).
    let run_b = run_script("runb");
    assert_eq!(run_b, oracle, "run B must equal the full hand oracle");
    assert_eq!(run_a, run_b, "the two runs must be byte-identical");
}
