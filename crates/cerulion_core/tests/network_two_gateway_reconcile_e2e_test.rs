// SPDX-License-Identifier: AGPL-3.0-only
//! The TRUE two-GATEWAY connect-only egress e2e + the reconciler-only
//! delivery proof.
//!
//! Both existing gateway e2e files ([`gateway_iox2_test`] /
//! [`network_ingress_e2e_test`]) use a raw `register_ingress_topic` manager as
//! the demander. This file closes the coverage gap: the CONSUMER is a SECOND
//! [`GatewayRuntime`] (connect-only, listen-less, scouting off) — the real
//! robot↔Mac topology, where subscriber-interest propagation can fail while
//! bounded liveliness QUERIES still work.
//!
//! ```text
//! Machine A (root_a):  P (network-free)  +  G_prod (listen, announce /t)
//! Machine B (root_b):  G_cons (connect-only, listen-less, ingress /t)  + local sub
//!
//! G_cons.new() ── demand token ──▶ G_prod flag ──▶ G_prod tap ──▶ zenoh TCP ──▶ G_cons ingress ──▶ B local sub
//! ```
//!
//! - Test (a) BOTH-PATHS-LIVE: the subscriber flips the flag; frames deliver
//!   byte-identical to a hand oracle.
//! - Test (b) RECONCILER-ONLY: G_prod's subscriber flag-flip is SUPPRESSED, so
//!   ONLY the reconciler's bounded query can flip the flag — the full chain still
//!   delivers byte-identical AND `reconciler_enabled_count > 0` (attribution).
//!
//! NOT `#[serial]`: distinct per-test SHM roots, per-run probed ports, bounded
//! retries ⇒ parallel-safe.

use std::net::TcpListener;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::gateway::{
    GatewayEgressPolicy, GatewayIngressEntry, GatewayPlan, GatewayRuntime,
};
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

/// (schema_hash, sequence, timestamp_ns, total_size, payload, full-frame).
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

/// Run the full two-gateway flow once. When `suppress` is true, G_prod's
/// liveliness SUBSCRIBER is muted (test seam) so ONLY the reconciler query can
/// flip the egress flag — and the flow is driven by `reconcile_demand_once`.
/// Returns (frames B's local subscriber received, G_prod's reconciler enable
/// count). Bounded (no hangs).
fn run_two_gateway_flow(tag: &str, suppress: bool) -> (Vec<Received>, u64) {
    let id = unique_id();
    let topic = format!("/twogw/{tag}/{id}");
    let root_a = cerulion_core::testing::iceoryx_test_config();

    // ---- Machine A: producer P (network-free) + gateway G_prod (listen),
    // sharing ONE SHM root. Bounded retry absorbs the probe→rebind port race.
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
                node_name: format!("twogw_p_{tag}_{id}_{attempt}"),
                clock: clock_dyn,
                ..Default::default()
            },
            root_a.clone(),
        )
        .expect("init P");
        let g = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("twogw_gprod_{tag}_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    // Announce keys carry the robot chunk.
                    robot_identity: Some("twogw-prod".to_string()),
                    ..Default::default()
                }),
                ..Default::default()
            },
            root_a.clone(),
        )
        .expect("init G_prod");
        // Mute the subscriber's flag-flip BEFORE the watch starts, so
        // in the reconciler-only run the ONLY flag-flipper is the query.
        if suppress {
            g.network()
                .expect("G_prod network")
                .set_suppress_live_demand_for_test(true);
        }
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
            Err(e) => eprintln!("attempt {attempt}: G_prod session failed (port {port}): {e}"),
        }
    }
    let Some((p_transport, a_clock, mut gateway_prod, port)) = a else {
        panic!("could not establish G_prod's listening session in 3 attempts");
    };
    // P's producer publisher creates the topic's SHM data service (G_prod taps it).
    let mut a_pub = p_transport
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("P publisher");

    // ---- Machine B: distinct root; G_cons is a SECOND GatewayRuntime, connect-
    // only + listen-less + scouting off, with an ingress plan for the topic (its
    // new() declares the demand token via register_ingress_topic).
    let b_mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("twogw_gcons_{tag}_{id}"),
            network: Some(NetworkConfig {
                connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                // listen_endpoints empty ⇒ listen-less; scouting off by default.
                robot_identity: Some("twogw-cons".to_string()),
                ..Default::default()
            }),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init G_cons manager");
    // B's local subscriber first (before the gateway's ingress re-injects).
    let b_sub = b_mgr.create_subscriber(&topic).expect("B subscriber");
    let plan_cons = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowList(vec![]), // ingress-only
        announce: vec![],
        ingress: vec![GatewayIngressEntry {
            topic: topic.clone(),
            schema_hash: Vector3::SCHEMA_HASH,
        }],
    };
    // Keep the consumer gateway ALIVE for the whole flow (its Drop tears down the
    // ingress bridge). Its ingress is zenoh-callback-driven — no drive needed.
    let _gateway_cons = GatewayRuntime::new(b_mgr, plan_cons).expect("G_cons boot");

    // ---- Handshake: flip G_prod's egress flag (bounded). In the reconciler-only
    // run the subscriber is muted, so drive `reconcile_demand_once` explicitly;
    // in the both-paths-live run the subscriber flips it (the reconciler thread
    // is a live backup).
    let flag = gateway_prod
        .manager()
        .bridge_manager()
        .register_topic(&topic)
        .expect("G_prod bridge flag handle");

    // F-C3 CONTROL (reconciler-only run): prove the suppression is genuinely
    // effective — that NOTHING other than a reconcile pass flips the flag. With
    // the subscriber muted, drive the tap forward WITHOUT running any reconcile
    // pass; while ZERO reconcile passes have run the egress flag MUST stay false
    // (the subscriber can't flip it, and drive_once only forwards). We gate the
    // assertion on reconcile_pass_count == 0 so the ~1 s background reconciler
    // thread (which sleeps a full interval before its first pass) can never race
    // this into a spurious failure — a false "stays-false" here would mean the
    // suppress seam silently no-ops (the subscriber flipped it). The handshake
    // below then runs reconcile passes and the flag flips — proving the
    // reconciler is the sole causal flipper.
    if suppress {
        let control_deadline = Instant::now() + Duration::from_millis(400);
        while Instant::now() < control_deadline {
            gateway_prod
                .drive_once()
                .expect("drive (F-C3 control, no reconcile)");
            if gateway_prod.reconcile_pass_count() == 0 {
                assert!(
                    !flag.load(Ordering::Relaxed),
                    "F-C3: with the subscriber suppressed and zero reconcile passes run, \
                     nothing may flip the egress flag — a flip here means the suppress seam \
                     silently no-ops (the subscriber flipped it, not the reconciler)"
                );
            }
            std::thread::sleep(Duration::from_millis(20));
        }
    }

    let deadline = Instant::now() + Duration::from_secs(20);
    while !flag.load(Ordering::Relaxed) {
        assert!(
            Instant::now() < deadline,
            "demand did not flip G_prod's egress flag within 20s (suppress={suppress})"
        );
        if suppress {
            gateway_prod
                .reconcile_demand_once()
                .expect("reconcile pass");
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    // Attach the tap BEFORE publishing (the tap has no history).
    let attach_deadline = Instant::now() + Duration::from_secs(5);
    while gateway_prod.active_tap_topics().is_empty() {
        assert!(Instant::now() < attach_deadline, "tap did not attach");
        gateway_prod.drive_once().expect("drive attach");
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

    // ---- Drive G_prod to forward + collect on B (bounded). In the reconciler-
    // only run, keep the reconciler affirming demand between drives.
    let mut got: Vec<Received> = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(10);
    while got.len() < PLAN.len() && Instant::now() < deadline {
        if suppress {
            gateway_prod
                .reconcile_demand_once()
                .expect("reconcile pass");
        }
        gateway_prod.drive_once().expect("drive forward");
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
    (got, gateway_prod.reconciler_enabled_count())
}

/// Test (a) BOTH-PATHS-LIVE: with a second GATEWAY as the demander (connect-only,
/// listen-less), P's frames flow through G_prod's demand-driven tap to B's local
/// subscriber BYTE-IDENTICAL — the coverage gap no existing test closes.
#[test]
fn two_gateway_egress_delivers_byte_identical_frames() {
    let (got, _enabled) = run_two_gateway_flow("both", false);
    assert_eq!(
        got,
        oracle(),
        "B must receive P's frames verbatim through the two-gateway link"
    );
}

/// Test (b) RECONCILER-ONLY (the decisive fix proof): G_prod's liveliness
/// subscriber is SUPPRESSED, so the ONLY path that can flip the egress flag is
/// the reconciler's bounded query. The full chain STILL delivers byte-identical,
/// and `reconciler_enabled_count > 0` attributes the flip to the reconciler.
#[test]
fn two_gateway_egress_delivers_via_reconciler_only() {
    let (got, enabled) = run_two_gateway_flow("reconciler_only", true);
    assert_eq!(
        got,
        oracle(),
        "the reconciler-only path must deliver P's frames byte-identical"
    );
    assert!(
        enabled >= 1,
        "the egress flip must be attributed to the reconciler (enabled_count >= 1), \
         not the suppressed subscriber; got {enabled}"
    );
}
