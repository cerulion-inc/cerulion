// SPDX-License-Identifier: AGPL-3.0-only
//! The GATEWAY-RESTART self-heal e2e over REAL loopback zenoh + iceoryx2.
//!
//! Reproduces the robot-restart shape: a desk `cerulion-netd` mirror keeps a robot's
//! topic streaming; the robot's `graph run` (and thus its gateway) is RESTARTED;
//! and the mirror RESUMES delivering the RESTARTED
//! robot's frames WITHOUT any client-side re-demand (the held demand self-heals).
//! Without the re-affirm, netd's persistent session never re-declares demand across the fresh
//! link, so every consumer sees a silent black hole until the netd dies.
//!
//! Shape (crib of `cerulion_core/tests/network_ingress_e2e_test.rs` +
//! `mirror_plane_iox2_test.rs`):
//!
//! ```text
//!  ROBOT (SHM root A)                         DESK / netd (SHM root B)
//!  producer P  --SHM-->  gateway G (listen X) ⇦ zenoh TCP X ⇨  TransportManager B
//!  publish_raw frames    demand-driven tap                     GatewayMirrorPlane
//!                        (AllowAll, drive_once)                 ensure_mirror +
//!                                                               demand keepalive +
//!                                                               health watch
//!                                                               B's local subscriber
//! ```
//!
//! 1. Robot 1 up (port X); netd demands the topic → robot-1 frames re-inject into
//!    B's SHM (hand-oracle value range [1000, 2000)).
//! 2. Robot 1 is DROPPED (its session dies). B's health watch observes DEGRADED
//!    (the OBSERVABLE `mirror_health`, cross-thread — no log scraping).
//! 3. Robot 2 up on the SAME port X (fresh producer + gateway, value range
//!    [2000, 3000)). B's demand-GET keepalive re-crosses the reconnected link →
//!    robot-2 frames re-inject → B's subscriber sees a value >= 2000 (NEW data,
//!    NOT stale robot-1 frames) and `mirror_health` returns HEALTHY — all with NO
//!    second `ensure_mirror` call (the demand was never re-issued by a consumer).
//!
//! NOT `#[serial]`: distinct per-test SHM roots + probed ports + scouting-off
//! sessions ⇒ parallel-safe (the `network_ingress_e2e_test` convention). Bounded
//! everywhere (no hangs); the robot-2 bind uses the same listener port (LISTEN
//! sockets do not TIME_WAIT, so the rebind is prompt) with a short retry.

use std::net::TcpListener;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::transport::gateway::{GatewayEgressPolicy, GatewayPlan, GatewayRuntime};
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};

use cerulion_netd::health::{HealthConfig, HealthState};
use cerulion_netd::mirror::{GatewayMirrorPlane, MirrorPlane};
use cerulion_netd::registry::TopicKey;

/// An arbitrary wire schema hash — the mirror validates INBOUND frames against it,
/// and the hand-built producer frames carry it, so validation passes.
const PROBE_HASH: u64 = 0x0BAD_F00D_DEAD_BEEF;

/// The robot's announced identity (both restarts share it — same robot, new
/// gateway). The netd's demand-GET harvests it from the announce space to target
/// the explicit `cerulion_q/{robot}/demand{topic}` selector.
const ROBOT_ID: &str = "robot";

static COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_id() -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{nanos}_{id}")
}

fn probe_ephemeral_port() -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind ephemeral probe");
    let port = listener.local_addr().expect("probe local_addr").port();
    drop(listener);
    port
}

/// Hand-build a wire frame carrying a u64 `value` payload (LE) — the generation +
/// sequence oracle the netd decodes back (never a self-compare: robot 1 stamps
/// [1000,2000), robot 2 stamps [2000,3000)).
fn value_frame(seq: u32, value: u64) -> Vec<u8> {
    let payload = value.to_le_bytes();
    let header = WireHeader {
        schema_hash: PROBE_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: 1_000 + seq as u64,
    };
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

/// A robot side: producer `p` + a listening gateway `g`, sharing one SHM root.
struct Robot {
    _p: Arc<TransportManager>,
    publisher: cerulion_core::transport::publisher::CerulionPublisher,
    gateway: GatewayRuntime,
}

/// Drive the robot's gateway + publish fresh frames (value = `base + seq`) while
/// polling B's subscriber, until B receives a frame whose decoded value is in
/// `[want_lo, want_hi)`, or the deadline elapses. Returns the received value (if
/// any). The tap has no history, so fresh frames are published every iteration so
/// one lands after the tap attaches.
fn drive_until_value_in_range(
    robot: &mut Robot,
    b_sub: &cerulion_core::transport::subscriber::CerulionSubscriber,
    base: u64,
    want_lo: u64,
    want_hi: u64,
    budget: Duration,
) -> Option<u64> {
    let deadline = Instant::now() + budget;
    let mut seq: u32 = 0;
    let mut got: Option<u64> = None;
    while Instant::now() < deadline {
        // Publish a fresh frame (monotonic seq) so a live frame exists post-attach.
        let value = base + seq as u64;
        let frame = value_frame(seq, value);
        let _ = robot.publisher.publish_raw(&frame);
        seq = seq.wrapping_add(1);

        // Forward one gateway pass.
        robot.gateway.drive_once().expect("gateway drive_once");

        // Drain B's mirror subscriber (one persistent subscriber across the run).
        b_sub
            .try_receive(|msg| {
                let payload = msg.payload();
                if payload.len() >= 8 {
                    let v = u64::from_le_bytes(payload[..8].try_into().unwrap());
                    if (want_lo..want_hi).contains(&v) {
                        got = Some(v);
                    }
                }
            })
            .expect("B try_receive");
        if got.is_some() {
            return got;
        }
        std::thread::sleep(Duration::from_millis(30));
    }
    got
}

/// Poll `plane.mirror_health(topic)` until it equals `want`, or the deadline
/// elapses. Returns whether it reached `want`.
fn wait_for_health(
    plane: &GatewayMirrorPlane,
    topic: &str,
    want: HealthState,
    budget: Duration,
) -> bool {
    let deadline = Instant::now() + budget;
    loop {
        if plane.mirror_health(topic) == Some(want) {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// The REALISTIC LAN run: both demand paths active (the liveliness token AND the
/// demand-GET keepalive). Proves the end-to-end product requirement — a restarted
/// robot gateway is invisible to a held demand — as it would happen on a healthy
/// LAN. On loopback the liveliness token alone re-crosses (so this arm does not by
/// itself PROVE the keepalive is load-bearing — see the `_suppressed` twin, which
/// isolates the keepalive as the SOLE demand path, the strict-link repro condition).
#[test]
fn mirror_self_heals_when_the_robot_gateway_restarts() {
    run_restart_self_heal(false);
}

/// The LOAD-BEARING run: SUPPRESS B's liveliness demand token
/// (`set_suppress_ingress_token_for_test`, the isolation seam) so the
/// robot's liveliness watch never sees a demand `Put` — the ONLY way robot egress
/// can enable (initial OR after the restart) is B's demand-GET keepalive (a
/// dialer→accepter query). This reproduces the strict connect-only link a real
/// robot presents (where the dialer's demand-token declaration never forwards) WITHOUT the
/// loopback liveliness back-sync. Disabling
/// `ensure_ingress_demand_keepalive` makes even ROBOT 1's initial frames never
/// arrive here (the keepalive is the sole demand path), so this test fails.
#[test]
fn mirror_self_heals_via_demand_get_keepalive_when_liveliness_is_suppressed() {
    run_restart_self_heal(true);
}

fn run_restart_self_heal(suppress_liveliness_token: bool) {
    let id = unique_id();
    let topic = format!("/restart/{id}");
    let root_a = iceoryx_test_config();
    let port = probe_ephemeral_port();

    // Bring a robot (producer P + listening gateway G, shared SHM root A) up on the
    // shared `port`, retrying the listener bind a few times (the restart rebinds the
    // SAME port — LISTEN sockets do not TIME_WAIT, so the rebind is prompt). A
    // closure so it can capture `root_a` without naming its (private-to-netd)
    // iceoryx2 config type.
    let spin_up_robot = |tag: &str| -> Robot {
        let mut established: Option<(Arc<TransportManager>, GatewayRuntime)> = None;
        for attempt in 0..8 {
            let p = TransportManager::init_for_test(
                TransportConfig {
                    node_name: format!("p_{tag}_{id}_{attempt}"),
                    ..Default::default()
                },
                root_a.clone(),
            )
            .expect("init producer P");
            let g = TransportManager::init_for_test(
                TransportConfig {
                    node_name: format!("g_{tag}_{id}_{attempt}"),
                    network: Some(NetworkConfig {
                        listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                        robot_identity: Some(ROBOT_ID.to_string()),
                        ..Default::default()
                    }),
                    ..Default::default()
                },
                root_a.clone(),
            )
            .expect("init gateway G");
            let plan = GatewayPlan {
                egress_policy: GatewayEgressPolicy::AllowAll,
                announce: vec![topic.clone()],
                ingress: vec![],
            };
            match GatewayRuntime::new(g, plan) {
                Ok(gateway) => {
                    established = Some((p, gateway));
                    break;
                }
                Err(e) => {
                    eprintln!(
                        "robot {tag} attempt {attempt}: gateway session failed on port {port}: {e}"
                    );
                    std::thread::sleep(Duration::from_millis(400));
                }
            }
        }
        let Some((p, gateway)) = established else {
            panic!(
                "robot {tag} could not establish its gateway session on port {port} in 8 attempts"
            );
        };
        // The producer's publisher creates the topic SHM service the gateway taps.
        let publisher = p
            .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
            .expect("producer publisher");
        Robot {
            _p: p,
            publisher,
            gateway,
        }
    };

    // ---- Robot 1 up (SHM root A, listen port X).
    let mut robot1 = spin_up_robot("r1");

    // ---- netd side: TransportManager B (distinct root) + GatewayMirrorPlane with
    //      SHORT health timings so the degrade/recover cycle is fast.
    let b = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("b_{id}"),
            network: Some(NetworkConfig {
                connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                // scouting off (hermetic); peer mode default -> the connect endpoint
                // is retried forever, so a restarted gateway on the same port
                // reconnects on its own.
                ..Default::default()
            }),
            ..Default::default()
        },
        iceoryx_test_config(),
    )
    .expect("init netd manager B");
    if suppress_liveliness_token {
        // Isolation: never declare B's liveliness demand token, so the
        // ONLY demand path to the robot is the demand-GET keepalive (the strict-link
        // condition a real robot presents). Must be set BEFORE ensure_mirror (which
        // calls register_ingress_topic, where the token would otherwise be declared).
        b.network()
            .expect("B has network")
            .set_suppress_ingress_token_for_test(true);
    }
    let plane = GatewayMirrorPlane::with_health_config(
        Arc::clone(&b),
        HealthConfig {
            poll: Duration::from_millis(200),
            degrade_after: Duration::from_secs(1),
            flap_window: Duration::from_secs(2),
        },
    );
    let key = TopicKey::new(ROBOT_ID, &topic);

    // ---- The ONE and ONLY consumer demand (a `topic hz`-style consumer). After
    //      this, self-heal must be INVISIBLE — no second ensure_mirror.
    plane
        .ensure_mirror(&key, PROBE_HASH)
        .expect("ensure_mirror registers the shared mirror + arms the keepalive");

    // ONE persistent subscriber on B's mirror service (survives the restart — the
    // mirror service is B's injection publisher, not the robot's).
    let b_sub = b
        .create_subscriber_open_only(&topic)
        .expect("open B's mirror subscriber");

    // ---- Robot 1 streams → B's mirror receives robot-1 frames [1000, 2000).
    let v1 = drive_until_value_in_range(
        &mut robot1,
        &b_sub,
        1000,
        1000,
        2000,
        Duration::from_secs(30),
    );
    assert!(
        v1.is_some(),
        "robot 1's frames must re-inject into B's mirror (initial demand crossed)"
    );
    // The mirror is healthy while streaming (bounded: it takes a health poll or two).
    assert!(
        wait_for_health(
            &plane,
            &topic,
            HealthState::Healthy,
            Duration::from_secs(10)
        ),
        "the streaming mirror is HEALTHY"
    );

    // ================= THE RESTART: drop robot 1 entirely =================
    drop(robot1);

    // ---- B's health watch observes DEGRADED (frames stopped; the loud warn +
    //      demand re-affirm belt fire — asserted via the cross-thread observable).
    assert!(
        wait_for_health(
            &plane,
            &topic,
            HealthState::Degraded,
            Duration::from_secs(15)
        ),
        "after the robot's gateway dies, the mirror is observed DEGRADED (loud, not silent)"
    );

    // The PROMPT belt is LOAD-BEARING — the Degraded edge armed a burst
    // of demand-reaffirm passes (observable). On the liveliness-suppressed arm the
    // demand-GET is the ONLY re-affirm path, so this is the mechanism that heals a
    // returning robot faster than the 2s background loop.
    let belt_passes_at_degrade = plane.belt_pass_count();
    if suppress_liveliness_token {
        assert!(
            belt_passes_at_degrade > 0,
            "the Degraded edge fired at least one prompt belt pass (load-bearing self-heal)"
        );
    }

    // ---- Robot 2 up on the SAME port X (fresh producer + gateway; values >= 2000).
    let mut robot2 = spin_up_robot("r2");

    // ---- WITHOUT any client re-demand, the mirror RESUMES delivering robot-2's
    //      frames: B's session reconnects (peer retry) and the demand-GET keepalive
    //      re-affirms egress over the fresh link. B sees a NEW value >= 2000.
    let v2 = drive_until_value_in_range(
        &mut robot2,
        &b_sub,
        2000,
        2000,
        3000,
        Duration::from_secs(45),
    );
    assert!(
        v2.is_some(),
        "the mirror SELF-HEALS — robot 2's frames re-inject with NO client re-demand \
         (the self-heal contract). Got none within the budget"
    );
    // And the health watch observed RECOVERY.
    assert!(
        wait_for_health(
            &plane,
            &topic,
            HealthState::Healthy,
            Duration::from_secs(10)
        ),
        "the mirror is observed RECOVERED (self-healed)"
    );

    // Cleanup: dropping the plane stops the health thread + drops the session.
    drop(robot2);
    drop(plane);
}
