// SPDX-License-Identifier: AGPL-3.0-only
//! CROSS-SESSION machine-A → network → machine-B ingress
//! e2e, with machine A's egress driven by a GATEWAY.
//!
//! Machine A is a producer manager `P` (local, network-free) + a gateway manager
//! `G` (network) sharing ONE SHM root — the graph-process + gateway-process
//! split. Machine B is a manager on a DISTINCT root that declares a DEMAND token
//! (`register_ingress_topic`) over a real 127.0.0.1 TCP hop; the token flips G's
//! egress flag, G attaches a listener-less tap to P's produced topic and forwards
//! each frame to zenoh, and B re-injects it into local SHM. B's local subscriber
//! must observe A's frames BYTE-IDENTICAL to a hand oracle (Principle #7).
//!
//! ```text
//! B: register_ingress_topic ──demand token──▶ G's watch ──enable_bridge──▶ flag
//! P: loan_proxy::<Vector3>() ──SHM──▶ G tap ──drive_once──▶ zenoh TCP ──▶ B callback
//!                                                              B's local iceoryx2 subscriber
//! ```
//!
//! NOT `#[serial]`: distinct per-test SHM roots, per-run probed ports, bounded
//! retries ⇒ parallel-safe.

use std::net::TcpListener;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::gateway::{GatewayEgressPolicy, GatewayPlan, GatewayRuntime};
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use native_ros2_messages::geometry_msgs::Vector3;

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

type Received = (u64, u32, u64, u32, Vec<u8>, Vec<u8>);

fn vector3_payload(x: f64, y: f64, z: f64) -> Vec<u8> {
    let mut v = Vec::with_capacity(24);
    v.extend_from_slice(&x.to_le_bytes());
    v.extend_from_slice(&y.to_le_bytes());
    v.extend_from_slice(&z.to_le_bytes());
    v
}

fn oracle_frame(seq: u32, ts: u64, payload: &[u8]) -> Vec<u8> {
    let header = WireHeader {
        schema_hash: Vector3::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: ts,
    };
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(payload);
    frame
}

const PLAN: [(u64, (f64, f64, f64)); 3] = [
    (1_000_000, (1.5, -2.5, 3.5)),
    (2_000_000, (4.5, 5.5, -6.5)),
    (3_000_000, (7.5, 8.5, 9.5)),
];

fn oracle() -> Vec<Received> {
    PLAN.iter()
        .enumerate()
        .map(|(i, (ts, (x, y, z)))| {
            let payload = vector3_payload(*x, *y, *z);
            let full = oracle_frame(i as u32, *ts, &payload);
            (
                Vector3::SCHEMA_HASH,
                i as u32,
                *ts,
                (WireHeader::SIZE + 24) as u32,
                payload,
                full,
            )
        })
        .collect()
}

/// Run the full P+G+B gateway-egress flow once; returns the frames B's local
/// subscriber received. Bounded (no hangs).
fn run_cross_manager_flow(tag: &str) -> Vec<Received> {
    let id = unique_id();
    let topic = format!("/e2e/{tag}/{id}");
    let root_a = cerulion_core::testing::iceoryx_test_config();

    // ---- Machine A: producer P (local) + gateway G (listen), shared root.
    let mut a: Option<(
        Arc<TransportManager>,
        Arc<VirtualClock>,
        GatewayRuntime,
        u16,
    )> = None;
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let clock = Arc::new(VirtualClock::new());
        let clock_dyn: Arc<dyn Clock> = clock.clone();
        let p = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("e2e_p_{tag}_{id}_{attempt}"),
                clock: clock_dyn,
                ..Default::default()
            },
            root_a.clone(),
        )
        .expect("init P");
        let g = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("e2e_g_{tag}_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    // Announce keys carry the robot chunk.
                    robot_identity: Some("e2e-g".to_string()),
                    ..Default::default()
                }),
                ..Default::default()
            },
            root_a.clone(),
        )
        .expect("init G");
        let plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![topic.clone()],
            ingress: vec![],
        };
        match GatewayRuntime::new(g, plan) {
            Ok(gateway) => {
                a = Some((p, clock, gateway, port));
                break;
            }
            Err(e) => eprintln!("attempt {attempt}: G session failed (port {port}): {e}"),
        }
    }
    let Some((p_transport, a_clock, mut gateway, port)) = a else {
        panic!("could not establish G's listening session in 3 attempts");
    };
    // P's producer publisher creates the topic's SHM data service (G taps it).
    let mut a_pub = p_transport
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("P publisher");

    // ---- Machine B: distinct root, connects to G, declares the demand token.
    let b_transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("e2e_b_{tag}_{id}"),
            network: Some(NetworkConfig {
                connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                ..Default::default()
            }),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init manager B");
    let b_sub = b_transport.create_subscriber(&topic).expect("B subscriber");
    b_transport
        .register_ingress_topic(&topic, Vector3::SCHEMA_HASH, MaxSliceLen::const_new(256))
        .expect("B register_ingress_topic");

    // ---- Handshake: B's demand token flips G's egress flag (bounded).
    let flag = gateway
        .manager()
        .bridge_manager()
        .register_topic(&topic)
        .expect("G bridge flag handle");
    let deadline = Instant::now() + Duration::from_secs(10);
    while !flag.load(Ordering::Relaxed) {
        assert!(
            Instant::now() < deadline,
            "demand token did not flip G's egress flag within 10s"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    // Attach the tap BEFORE publishing (the tap has no history).
    let attach_deadline = Instant::now() + Duration::from_secs(5);
    while gateway.active_tap_topics().is_empty() {
        assert!(Instant::now() < attach_deadline, "tap did not attach");
        gateway.drive_once().expect("drive attach");
        std::thread::sleep(Duration::from_millis(10));
    }

    // ---- Publish the plan on P.
    for (ts, (x, y, z)) in PLAN {
        a_clock.set(ts);
        let mut proxy = a_pub.loan_proxy::<Vector3>().expect("P loan_proxy");
        proxy.x = x;
        proxy.y = y;
        proxy.z = z;
    }

    // ---- Drive G to forward + collect on B (bounded).
    let mut got: Vec<Received> = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(10);
    while got.len() < PLAN.len() && Instant::now() < deadline {
        gateway.drive_once().expect("drive forward");
        b_sub
            .try_receive(|msg| {
                let h = msg.header();
                let payload = msg.payload().to_vec();
                let mut full = vec![0u8; WireHeader::SIZE + payload.len()];
                h.write_to_buf(&mut full[..WireHeader::SIZE]);
                full[WireHeader::SIZE..].copy_from_slice(&payload);
                got.push((
                    h.schema_hash,
                    h.sequence,
                    h.timestamp_ns,
                    h.total_size,
                    payload,
                    full,
                ));
            })
            .expect("B try_receive");
        if got.len() < PLAN.len() {
            std::thread::sleep(Duration::from_millis(20));
        }
    }
    got
}

/// The headline cross-session pin: P's frames flow through G's demand-driven tap
/// to B BYTE-IDENTICAL — wire sequence 0,1,2 and the hand-set VirtualClock
/// timestamps intact (hand oracle, Principle #7).
#[test]
fn test_cross_manager_ingress_delivers_byte_identical_frames() {
    let got = run_cross_manager_flow("headline");
    assert_eq!(
        got,
        oracle(),
        "B must receive A's frames verbatim through the gateway tap"
    );
}

/// Determinism (Principle #7): two full runs deliver IDENTICAL frame vectors,
/// both equal to the hand oracle (not merely each other).
#[test]
fn test_cross_manager_ingress_is_deterministic() {
    let a = run_cross_manager_flow("det_a");
    let b = run_cross_manager_flow("det_b");
    let oracle = oracle();
    assert_eq!(a, oracle, "run A must equal the hand oracle");
    assert_eq!(b, oracle, "run B must equal the hand oracle");
    assert_eq!(a, b, "two identical runs must deliver byte-identically");
}

/// Replay-inert pin: a manager built with `network: None` has NO
/// `NetworkManager` — no session, structurally zero zenoh activity — and both
/// network entry points err loudly naming the `network:` fix (Principle #7:
/// replay builds transport without a network and can never touch it).
#[test]
fn test_network_none_manager_is_structurally_inert() {
    let id = unique_id();
    let topic = format!("/e2e/inert/{id}");
    let transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("e2e_inert_{id}"),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init network-less manager");

    assert!(
        transport.network().is_none(),
        "network: None must mean NO NetworkManager exists"
    );

    let err = transport
        .register_ingress_topic(&topic, 0x1, MaxSliceLen::const_new(256))
        .expect_err("ingress on a network-less manager must be refused");
    assert!(
        format!("{err}").contains("`network:` block"),
        "refusal must name the fix: {err}"
    );

    let err = transport
        .start_network_bridge_watch()
        .expect_err("bridge watch on a network-less manager must be refused");
    assert!(
        format!("{err}").contains("`network:` block"),
        "refusal must name the fix: {err}"
    );

    // A gateway on a network-less manager is refused too.
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic],
        ingress: vec![],
    };
    let err =
        GatewayRuntime::new(transport, plan).expect_err("network-less gateway must be refused");
    assert!(
        format!("{err}").contains("`network:` block"),
        "gateway refusal must name the fix: {err}"
    );
}
