// SPDX-License-Identifier: AGPL-3.0-only
//! The network GATEWAY end-to-end + plan oracles.
//!
//! A [`GatewayRuntime`] owns one graph's network plane: it announces produced
//! topics, hears remote DEMAND tokens, and forwards each demanded topic's SHM
//! frames to zenoh via listener-less taps (egress); it also re-injects declared
//! ingress topics into local SHM. Publishers are network-free — the gateway taps
//! their produced topics.
//!
//! # Test shape
//!
//! - Pure [`GatewayPlan`] oracles (validate + serde) — no transport.
//! - Demand-driven egress e2e: a producer manager `P` and a gateway manager `G`
//!   share ONE SHM root (two processes on one machine); a remote manager `B` on
//!   a DISTINCT root declares a demand token over a real 127.0.0.1 TCP hop, `G`'s
//!   tap attaches and forwards `P`'s frames, and `B`'s local subscriber receives
//!   them BYTE-IDENTICAL to a hand oracle (crib `network_ingress_e2e_test`).
//! - Tap attach/detach + allow-list gating driven DETERMINISTICALLY by flipping
//!   the bridge flag directly (a demand token's effect) — no zenoh timing.
//!
//! Per-test SHM roots + probed ports + isolated scouting-off sessions ⇒
//! parallel-safe, no `#[serial]`.

use std::collections::{HashMap, HashSet};
use std::net::TcpListener;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::message::ShmMessage;
use cerulion_core::testing::{count_at_exclusively, debug_lines_expected, line_level, logged_at};
use cerulion_core::transport::bridge::TopicBridgeManager;
use cerulion_core::transport::demand_authorizer::{
    AllowAllAuthorizer, DemandAuthorizer, DemandDecision, DemandSubject,
};
use cerulion_core::transport::gateway::{
    GatewayEgressPolicy, GatewayIngressEntry, GatewayPlan, GatewayRuntime, GATEWAY_ZERO_DEMAND_IDLE,
};
use cerulion_core::transport::liveness::{
    LivenessState, LIVENESS_NO_DATA_MIN_MS, LIVENESS_STREAMING_RECENCY_MS,
    LIVENESS_SWEEP_INTERVAL_NS, REGRESSION_RESET_MIN_GAP_NS, SUSTAINED_REGRESSION_MIN_SPAN_NS,
};
use cerulion_core::transport::network::{
    apply_demand_reconcile_for_test, refresh_and_apply_demand_reconcile_for_test,
    refresh_reconcile_topics_for_test, NetworkConfig,
};
use cerulion_core::transport::reg_channel::REG_CHANNEL_SERVICE_NAME;
use cerulion_core::transport::{PublisherProvisioning, TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use native_ros2_messages::geometry_msgs::Vector3;
use tracing_test::traced_test;

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

fn vector3_payload(x: f64, y: f64, z: f64) -> Vec<u8> {
    let mut v = Vec::with_capacity(24);
    v.extend_from_slice(&x.to_le_bytes());
    v.extend_from_slice(&y.to_le_bytes());
    v.extend_from_slice(&z.to_le_bytes());
    v
}

/// Hand-build the FULL wire frame a `loan_proxy::<Vector3>` publish commits
/// (crib `network_ingress_e2e_test::oracle_frame`): fixed schemas stamp
/// `offset_table_offset = WireHeader::SIZE + payload.len()` with count 0.
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

/// (schema_hash, sequence, timestamp_ns, total_size, payload, full-frame).
type Received = (u64, u32, u64, u32, Vec<u8>, Vec<u8>);

/// Hand-stamped publish plan (virtual-clock ns, Vector3 values). Sequences are
/// the commit-consumed counter, 0,1,2 by construction.
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

// ---------------------------------------------------------------------------
// (f)+(g pure): GatewayPlan oracles (moved out of gateway.rs to keep it free
// of a test module for the hot-path alloc lint).
// ---------------------------------------------------------------------------

#[test]
fn plan_validate_accepts_disjoint_announce_and_ingress() {
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec!["/a".to_string(), "/b".to_string()],
        ingress: vec![GatewayIngressEntry {
            topic: "/c".to_string(),
            schema_hash: 0x1,
        }],
    };
    assert!(plan.validate().is_ok());
}

/// (f) loop safety: announce ∩ ingress must be empty — canonical comparison
/// catches a raw-vs-slashed spelling.
#[test]
fn plan_validate_refuses_announce_ingress_overlap_canonically() {
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowList(vec!["/shared".to_string()]),
        announce: vec!["/shared".to_string()],
        ingress: vec![GatewayIngressEntry {
            topic: "shared".to_string(), // raw spelling of the announced /shared
            schema_hash: 0x1,
        }],
    };
    let err = plan.validate().expect_err("overlap must be refused");
    let msg = format!("{err}");
    assert!(
        msg.contains("network loop") && msg.contains("/shared"),
        "refusal must name the loop + the topic: {msg}"
    );
}

#[test]
fn plan_round_trips_through_json() {
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowList(vec!["/x".to_string()]),
        announce: vec!["/x".to_string()],
        ingress: vec![GatewayIngressEntry {
            topic: "/y".to_string(),
            schema_hash: 0xDEAD_BEEF,
        }],
    };
    let json = serde_json::to_string(&plan).expect("serialize");
    let back: GatewayPlan = serde_json::from_str(&json).expect("deserialize");
    assert_eq!(plan, back, "plan must survive a JSON round-trip byte-equal");
}

// ---------------------------------------------------------------------------
// (h): a gateway on a network-less manager is refused loudly.
// ---------------------------------------------------------------------------

#[test]
fn gateway_on_network_less_manager_is_refused() {
    let id = unique_id();
    let transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("gw_inert_{id}"),
            ..Default::default() // network: None
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init network-less manager");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![format!("/gw/{id}/topic")],
        ingress: vec![],
    };
    let err = GatewayRuntime::new(transport, plan).expect_err("network-less gateway must refuse");
    assert!(
        format!("{err}").contains("`network:` block"),
        "refusal must name the network: fix: {err}"
    );
}

// ---------------------------------------------------------------------------
// (b),(c),(d): tap attach/detach + allow-list gating, driven deterministically
// by flipping the bridge flag directly (the effect of a demand token). A
// producer manager P shares the gateway's SHM root; the gateway's default
// (loopback, no-endpoint) session accepts `publish_to_network` with no remote
// subscriber (Ok), so forwarded_count is observable without a receiver.
// ---------------------------------------------------------------------------

/// Build a producer manager P + a gateway on manager G, sharing one SHM root.
/// P creates a publisher per topic in `produced` FIRST (so the gateway tap can
/// open the data service). Returns (P, gateway, per-topic publishers).
fn make_producer_and_gateway(
    tag: &str,
    plan: GatewayPlan,
    produced: &[String],
) -> (
    Arc<TransportManager>,
    Arc<VirtualClock>,
    GatewayRuntime,
    Vec<cerulion_core::CerulionPublisher>,
) {
    let id = unique_id();
    let root = cerulion_core::testing::iceoryx_test_config();
    let clock = Arc::new(VirtualClock::new());
    let clock_dyn: Arc<dyn Clock> = clock.clone();
    // P: producer, LOCAL-ONLY (a graph process is network-free).
    let p = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("gw_p_{tag}_{id}"),
            clock: clock_dyn,
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init producer P");
    let publishers: Vec<_> = produced
        .iter()
        .map(|t| {
            p.create_publisher_simple(t, MaxSliceLen::const_new(256))
                .expect("P publisher")
        })
        .collect();
    // G: gateway, NETWORK (default loopback, no endpoints), SAME root as P.
    // Announce keys carry the robot chunk, so the gateway manager
    // needs a robot identity.
    let g = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("gw_g_{tag}_{id}"),
            network: Some(NetworkConfig {
                robot_identity: Some("gwtest".to_string()),
                ..NetworkConfig::default()
            }),
            ..Default::default()
        },
        root,
    )
    .expect("init gateway G");
    let gateway = GatewayRuntime::new(g, plan).expect("gateway boot");
    (p, clock, gateway, publishers)
}

/// Publish one Vector3 frame on `publisher` at virtual time `ts`.
fn publish_one(
    publisher: &mut cerulion_core::CerulionPublisher,
    clock: &VirtualClock,
    ts: u64,
    v: (f64, f64, f64),
) {
    clock.set(ts);
    let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
    proxy.x = v.0;
    proxy.y = v.1;
    proxy.z = v.2;
}

/// (b) A demand token attaches the tap and forwards; dropping it detaches the
/// tap and stops forwarding.
#[test]
fn demand_attaches_tap_then_drop_detaches_and_stops_forwarding() {
    let topic = format!("/gw/b/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic.clone()],
        ingress: vec![],
    };
    let (p, clock, mut gateway, mut pubs) =
        make_producer_and_gateway("b", plan, std::slice::from_ref(&topic));
    let _ = &p;

    // No demand yet → drive is a no-op, no tap, no forward.
    assert_eq!(gateway.drive_once().expect("drive"), 0);
    assert!(
        gateway.active_tap_topics().is_empty(),
        "no tap before demand"
    );

    // Demand arrives (a remote subscriber): flip the flag directly.
    gateway
        .manager()
        .bridge_manager()
        .enable_bridge(&topic)
        .expect("enable");
    // First drive attaches the tap (no frames yet).
    assert_eq!(gateway.drive_once().expect("drive"), 0);
    assert_eq!(
        gateway.active_tap_topics(),
        vec![topic.clone()],
        "tap must attach on demand"
    );

    // Producer publishes 2 frames; the next drive forwards them.
    publish_one(&mut pubs[0], &clock, 10, (1.0, 2.0, 3.0));
    publish_one(&mut pubs[0], &clock, 20, (4.0, 5.0, 6.0));
    // Poll (the SHM frames must be visible to the tap).
    let forwarded = drive_until(&mut gateway, 2, &topic);
    assert_eq!(forwarded, 2, "both frames forwarded");
    assert_eq!(gateway.forwarded_count(&topic), 2);

    // Demand ends: flag off → drive detaches the tap.
    gateway
        .manager()
        .bridge_manager()
        .disable_bridge(&topic)
        .expect("disable");
    assert_eq!(gateway.drive_once().expect("drive"), 0);
    assert!(
        gateway.active_tap_topics().is_empty(),
        "tap must detach when demand ends"
    );

    // Further publishes are NOT forwarded (tap gone).
    publish_one(&mut pubs[0], &clock, 30, (7.0, 8.0, 9.0));
    for _ in 0..5 {
        assert_eq!(gateway.drive_once().expect("drive"), 0);
    }
    assert_eq!(
        gateway.forwarded_count(&topic),
        2,
        "no forwards after detach"
    );
}

/// A tap drain failure is
/// non-fatal — `drive_once` returns `Ok` (a `?` there would kill the whole
/// network plane over one topic's transient error), the failure is recorded,
/// the wedged tap is DROPPED, the next pass re-attaches a fresh one (the flag
/// stays ON), and forwarding recovers.
#[test]
fn drain_failure_is_non_fatal_and_forwarding_recovers() {
    let topic = format!("/gw/df/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic.clone()],
        ingress: vec![],
    };
    let (p, clock, mut gateway, mut pubs) =
        make_producer_and_gateway("df", plan, std::slice::from_ref(&topic));
    let _ = &p;

    // Healthy baseline: demand → attach → one frame forwarded.
    gateway
        .manager()
        .bridge_manager()
        .enable_bridge(&topic)
        .expect("enable");
    assert_eq!(gateway.drive_once().expect("attach pass"), 0);
    publish_one(&mut pubs[0], &clock, 10, (1.0, 2.0, 3.0));
    drive_until(&mut gateway, 1, &topic);
    assert_eq!(gateway.forwarded_count(&topic), 1);

    // Arm the fire-once drain fault (after=0: the tap's next receive fails —
    // the deterministic stand-in for a producer-restart connection error).
    assert!(
        gateway.fault_inject_tap_drain_for_test(&topic, 0),
        "the tap must be attached before arming the fault"
    );
    publish_one(&mut pubs[0], &clock, 20, (4.0, 5.0, 6.0));
    // THE survival pin: the faulted pass must return Ok, not Err (a `?`
    // there would propagate fatally and run() would exit).
    let forwarded = gateway
        .drive_once()
        .expect("drive_once must SURVIVE a tap drain failure — non-fatal per-topic error");
    assert_eq!(forwarded, 0, "the faulted pass forwards nothing");
    assert!(
        gateway.active_tap_topics().is_empty(),
        "the wedged tap is dropped so the next pass re-attaches fresh"
    );

    // Next pass re-attaches (flag still ON).
    assert_eq!(gateway.drive_once().expect("re-attach pass"), 0);
    assert_eq!(gateway.active_tap_topics(), vec![topic.clone()]);

    // Post-recovery forwarding: a fresh publish flows through the new tap.
    // (Frame 2 sat in the broken tap's queue — unrecoverable by design; the
    // alternative was killing the whole network plane.)
    publish_one(&mut pubs[0], &clock, 30, (7.0, 8.0, 9.0));
    drive_until(&mut gateway, 2, &topic);
    assert_eq!(
        gateway.forwarded_count(&topic),
        2,
        "forwarding must recover after the drain-failure regime"
    );
}

/// A tap attach failure (here: the producer's
/// service does not exist yet — the same non-fatal code path as
/// subscriber-slot exhaustion) is COUNTED via `attach_failure_count`
/// (Principle #3), retried every pass, and HEALS when the service appears.
#[test]
fn attach_failure_is_counted_non_fatal_and_heals() {
    let topic = format!("/gw/af/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic.clone()],
        ingress: vec![],
    };
    // NO producer publisher yet — the topic's data service does not exist, so
    // every attach attempt fails (open-only tap).
    let (p, clock, mut gateway, _pubs) = make_producer_and_gateway("af", plan, &[]);

    gateway
        .manager()
        .bridge_manager()
        .enable_bridge(&topic)
        .expect("enable");
    assert_eq!(gateway.attach_failure_count(&topic), 0);
    // Two failing passes: both Ok (non-fatal), no tap, count grows per pass.
    assert_eq!(
        gateway.drive_once().expect("failing pass 1 is non-fatal"),
        0
    );
    assert_eq!(
        gateway.drive_once().expect("failing pass 2 is non-fatal"),
        0
    );
    assert!(
        gateway.active_tap_topics().is_empty(),
        "no tap while attach fails"
    );
    assert_eq!(
        gateway.attach_failure_count(&topic),
        2,
        "every failed attach is counted (Principle #3)"
    );

    // The producer's service appears: the next pass attaches (heals), the
    // count stops growing, and forwarding works end to end.
    let mut producer = p
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("late producer");
    assert_eq!(gateway.drive_once().expect("healing pass"), 0);
    assert_eq!(gateway.active_tap_topics(), vec![topic.clone()]);
    assert_eq!(
        gateway.attach_failure_count(&topic),
        2,
        "a successful attach stops the count"
    );
    publish_one(&mut producer, &clock, 10, (1.0, 2.0, 3.0));
    drive_until(&mut gateway, 1, &topic);
    assert_eq!(
        gateway.forwarded_count(&topic),
        1,
        "egress flows after healing"
    );
}

/// (c) An un-demanded topic is NEVER tapped (idle cost zero).
#[test]
fn undemanded_topic_never_taps() {
    let topic = format!("/gw/c/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic.clone()],
        ingress: vec![],
    };
    let (p, clock, mut gateway, mut pubs) =
        make_producer_and_gateway("c", plan, std::slice::from_ref(&topic));
    let _ = &p;
    // Producer publishes but NO demand token ever flips the flag.
    publish_one(&mut pubs[0], &clock, 10, (1.0, 2.0, 3.0));
    for _ in 0..5 {
        assert_eq!(gateway.drive_once().expect("drive"), 0);
    }
    assert!(
        gateway.active_tap_topics().is_empty(),
        "an un-demanded topic must never be tapped"
    );
    assert_eq!(gateway.forwarded_count(&topic), 0);
}

/// (d) An AllowList gateway refuses demand for a topic outside the list (its
/// flag never flips → never tapped) while a LISTED sibling forwards. The plan
/// announces BOTH (both get flags) but the allow-list admits only `listed` —
/// exercising the structural allow-list refusal in the tap loop.
#[test]
fn allowlist_gates_which_announced_topic_taps() {
    let base = unique_id();
    let listed = format!("/gw/d/{base}/listed");
    let unlisted = format!("/gw/d/{base}/unlisted");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowList(vec![listed.clone()]),
        announce: vec![listed.clone(), unlisted.clone()],
        ingress: vec![],
    };
    let (p, clock, mut gateway, mut pubs) =
        make_producer_and_gateway("d", plan, &[listed.clone(), unlisted.clone()]);
    let _ = &p;

    // A remote demands BOTH. The allow-list refuses `unlisted` structurally.
    let bm = gateway.manager().bridge_manager();
    bm.enable_bridge(&listed).expect("enable listed");
    bm.enable_bridge(&unlisted)
        .expect("enable unlisted (refused)");

    // One drive pass attaches the listed tap BEFORE publishing: a
    // listener-less tap has no history, so frames published before the
    // attach are dropped — the drop-to-live semantics remote viewing wants.
    let _ = gateway.drive_once().expect("tap attach pass");

    // Publish on both.
    publish_one(&mut pubs[0], &clock, 10, (1.0, 2.0, 3.0)); // listed
    publish_one(&mut pubs[1], &clock, 10, (9.0, 9.0, 9.0)); // unlisted
    let _ = drive_until(&mut gateway, 1, &listed);

    assert_eq!(
        gateway.active_tap_topics(),
        vec![listed.clone()],
        "only the listed topic is tapped"
    );
    assert!(
        gateway.forwarded_count(&listed) >= 1,
        "listed topic must forward"
    );
    assert_eq!(
        gateway.forwarded_count(&unlisted),
        0,
        "an unlisted topic must never forward"
    );
}

/// Drive the gateway until `want` frames have been forwarded for `topic` or a
/// bounded deadline; returns the total forwarded across all topics.
fn drive_until(gateway: &mut GatewayRuntime, want: u64, topic: &str) -> usize {
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut total = 0usize;
    while gateway.forwarded_count(topic) < want && Instant::now() < deadline {
        total += gateway.drive_once().expect("drive");
        std::thread::sleep(Duration::from_millis(5));
    }
    total
}

// ---------------------------------------------------------------------------
// (a)+(g): demand-driven egress e2e over a real TCP hop, byte-identical +
// deterministic.
// ---------------------------------------------------------------------------

/// Run the full P+G+B egress flow once; returns the frames B's local subscriber
/// received. Bounded (no hangs).
fn run_gateway_egress_e2e(tag: &str) -> Vec<Received> {
    let id = unique_id();
    let topic = format!("/gwe2e/{tag}/{id}");
    let root_a = cerulion_core::testing::iceoryx_test_config();

    // ---- Machine A: producer P (local) + gateway G (listen), shared root.
    // Bounded retry absorbs the probe→rebind port-steal race.
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
                node_name: format!("gwe2e_p_{tag}_{id}_{attempt}"),
                clock: clock_dyn,
                ..Default::default()
            },
            root_a.clone(),
        )
        .expect("init P");
        let g = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("gwe2e_g_{tag}_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    // Announce keys carry the robot chunk.
                    robot_identity: Some("gwe2e".to_string()),
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
        // GatewayRuntime::new opens G's session (binds the listen endpoint). A
        // bind failure (port stolen) surfaces here.
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
    // P's producer publisher creates the topic's SHM data service (the gateway
    // taps it). Created after the retry loop so exactly one exists.
    let mut a_pub = p_transport
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("P publisher");

    // ---- Machine B: distinct root, connects to G, declares the demand token
    // via register_ingress_topic, and re-injects into its own local SHM.
    let b_transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("gwe2e_b_{tag}_{id}"),
            network: Some(NetworkConfig {
                connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                ..Default::default()
            }),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init B");
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
    // Attach the tap BEFORE publishing (the tap has no history — it only sees
    // frames published after it attaches).
    let attach_deadline = Instant::now() + Duration::from_secs(5);
    while gateway.active_tap_topics().is_empty() {
        assert!(Instant::now() < attach_deadline, "tap did not attach");
        gateway.drive_once().expect("drive attach");
        std::thread::sleep(Duration::from_millis(10));
    }

    // ---- Publish the plan on P.
    for (ts, (x, y, z)) in PLAN {
        publish_one(&mut a_pub, &a_clock, ts, (x, y, z));
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

/// (a) The headline egress pin: P's frames flow through G's demand-driven tap to
/// B BYTE-IDENTICAL (hand oracle, Principle #7).
#[test]
fn gateway_egress_delivers_byte_identical_frames() {
    let got = run_gateway_egress_e2e("headline");
    assert_eq!(
        got,
        oracle(),
        "B must receive P's frames verbatim via the gateway tap"
    );
}

/// (g) Determinism: two full runs deliver identical frame vectors, both equal to
/// the hand oracle (not merely each other).
#[test]
fn gateway_egress_is_deterministic() {
    let a = run_gateway_egress_e2e("det_a");
    let b = run_gateway_egress_e2e("det_b");
    let oracle = oracle();
    assert_eq!(a, oracle, "run A must equal the hand oracle");
    assert_eq!(b, oracle, "run B must equal the hand oracle");
    assert_eq!(a, b, "two runs must be byte-identical");
}

/// The GAP FRAME (the one stated layout in
/// [`cerulion_core::testing::gap_frame`]: big variable field page-aligned in
/// the payload tail, fields out of declaration order, dead gap bytes inside
/// `total_size`) crosses the full P→G→zenoh→B egress path BYTE-IDENTICAL,
/// and B's ingress validation (`total_size` + `schema_hash` before
/// re-injection) ACCEPTS it. A gateway or re-injector that re-framed,
/// compacted, or refused the non-contiguous placement fails here.
///
/// Same shape as [`run_gateway_egress_e2e`], with the frames published via
/// `publish_raw` (the hand-built gap bytes are the oracle — never a
/// self-compare) on a 64 KiB slice ceiling.
#[test]
fn gateway_forwards_the_gap_frame_byte_identical() {
    use cerulion_core::testing::gap_frame::image_gap_frame;
    use native_ros2_messages::sensor_msgs::Image;

    let id = unique_id();
    let topic = format!("/gwe2e/gap/{id}");
    let root_a = cerulion_core::testing::iceoryx_test_config();
    let msl = MaxSliceLen::const_new(64 * 1024);

    // ---- Machine A: producer P (local) + gateway G (listen), shared root.
    // Bounded retry absorbs the probe→rebind port-steal race.
    let mut a: Option<(Arc<TransportManager>, GatewayRuntime, u16)> = None;
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let p = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("gwgap_p_{id}_{attempt}"),
                ..Default::default()
            },
            root_a.clone(),
        )
        .expect("init P");
        let g = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("gwgap_g_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    robot_identity: Some("gwgap".to_string()),
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
                a = Some((p, gateway, port));
                break;
            }
            Err(e) => eprintln!("attempt {attempt}: G session failed (port {port}): {e}"),
        }
    }
    let Some((p_transport, mut gateway, port)) = a else {
        panic!("could not establish G's listening session in 3 attempts");
    };
    let mut a_pub = p_transport
        .create_publisher_simple(&topic, msl)
        .expect("P publisher");

    // ---- Machine B: distinct root, connects to G, demands + re-injects.
    let b_transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("gwgap_b_{id}"),
            network: Some(NetworkConfig {
                connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                ..Default::default()
            }),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init B");
    let b_sub = b_transport.create_subscriber(&topic).expect("B subscriber");
    b_transport
        .register_ingress_topic(&topic, Image::SCHEMA_HASH, msl)
        .expect("B register_ingress_topic — the ingress gate must accept the gap frame's shape");

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
    let attach_deadline = Instant::now() + Duration::from_secs(5);
    while gateway.active_tap_topics().is_empty() {
        assert!(Instant::now() < attach_deadline, "tap did not attach");
        gateway.drive_once().expect("drive attach");
        std::thread::sleep(Duration::from_millis(10));
    }

    // ---- Publish TWO hand-built gap frames (distinct wire stamps).
    let expected: Vec<Vec<u8>> = (0..2u32)
        .map(|i| image_gap_frame(Image::SCHEMA_HASH, i, 5_000 + i as u64))
        .collect();
    for frame in &expected {
        let _ = a_pub.publish_raw(frame).expect("publish gap frame on P");
    }

    // ---- Drive G to forward + collect FULL frames on B (bounded).
    let mut got: Vec<Vec<u8>> = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(10);
    while got.len() < expected.len() && Instant::now() < deadline {
        gateway.drive_once().expect("drive forward");
        b_sub
            .try_receive(|msg| {
                let payload = msg.payload();
                let mut full = vec![0u8; WireHeader::SIZE + payload.len()];
                msg.header().write_to_buf(&mut full[..WireHeader::SIZE]);
                full[WireHeader::SIZE..].copy_from_slice(payload);
                got.push(full);
            })
            .expect("B try_receive");
        if got.len() < expected.len() {
            std::thread::sleep(Duration::from_millis(20));
        }
    }
    assert_eq!(
        got, expected,
        "B must receive the gap frames verbatim — dead gap bytes, page-aligned \
         placement and out-of-order entries included"
    );
}

// ---------------------------------------------------------------------------
// (e): ingress via the gateway → a local subscriber receives byte-identical.
// ---------------------------------------------------------------------------

/// Hand-build a raw wire frame exactly as an egress peer puts it on the wire.
fn make_frame(seq: u32, ts: u64, payload: &[u8]) -> Vec<u8> {
    let header = WireHeader {
        schema_hash: Vector3::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: ts,
    };
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(payload);
    frame
}

/// (e) A gateway with an ingress plan re-injects a remote frame into local SHM;
/// a local subscriber on the gateway's manager reads it byte-identical.
#[test]
fn gateway_ingress_reinjects_into_local_shm() {
    let id = unique_id();
    let topic = format!("/gwing/{id}");

    // Machine A (remote egress): a network manager that puts raw frames.
    let mut a: Option<(Arc<TransportManager>, u16)> = None;
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("gwing_a_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    ..Default::default()
                }),
                ..Default::default()
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .expect("init A");
        match mgr.start_network_bridge_watch() {
            Ok(()) => {
                a = Some((mgr, port));
                break;
            }
            Err(e) => eprintln!("attempt {attempt}: A listen failed (port {port}): {e}"),
        }
    }
    let Some((a_mgr, port)) = a else {
        panic!("A listen session failed in 3 attempts");
    };

    // Machine B (gateway): ingress plan for the topic; connects to A. Even an
    // ingress-only gateway declares its bare identity token, so it
    // carries a robot identity.
    let b_mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("gwing_b_{id}"),
            network: Some(NetworkConfig {
                connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                robot_identity: Some("gwing-b".to_string()),
                ..Default::default()
            }),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init B");
    // B's local subscriber first (before the gateway's ingress re-injects).
    let b_sub = b_mgr.create_subscriber(&topic).expect("B subscriber");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowList(vec![]), // ingress-only
        announce: vec![],
        ingress: vec![GatewayIngressEntry {
            topic: topic.clone(),
            schema_hash: Vector3::SCHEMA_HASH,
        }],
    };
    let _gateway = GatewayRuntime::new(b_mgr, plan).expect("gateway boot (ingress)");

    // A RE-PUTS a hand-built frame each iteration (the cross-session ingress
    // subscriber may not have propagated to A yet — re-putting is robust against
    // that settle race; B's subscriber yields the first re-injected copy and we
    // stop). B's gateway re-injects into B's local SHM.
    let payload = vector3_payload(3.25, -4.5, 5.75);
    let a_net = a_mgr.network().expect("A network");
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut got: Option<(u64, u32, u64, Vec<u8>)> = None;
    while got.is_none() && Instant::now() < deadline {
        a_net
            .publish_to_network(&topic, make_frame(7, 9_000, &payload))
            .expect("A put");
        std::thread::sleep(Duration::from_millis(20));
        b_sub
            .try_receive(|msg| {
                let h = msg.header();
                got = Some((
                    h.schema_hash,
                    h.sequence,
                    h.timestamp_ns,
                    msg.payload().to_vec(),
                ));
            })
            .expect("B try_receive");
    }
    assert_eq!(
        got,
        Some((Vector3::SCHEMA_HASH, 7, 9_000, payload)),
        "the gateway must re-inject the remote frame byte-identical into local SHM"
    );
}

// ---------------------------------------------------------------------------
// The demand RECONCILER — the query-shaped belt to the subscriber
// watch. Test 1 is LIVE (cross-session, with the subscriber path SUPPRESSED so
// only the reconciler can flip the flag). Tests 2 + 3 are PURE oracle vectors
// over `apply_demand_reconcile_for_test` (the exact hysteresis/allow-list body
// the reconciler runs) — no zenoh, fully deterministic.
// ---------------------------------------------------------------------------

/// Test 1: with the liveliness SUBSCRIBER suppressed on the producing gateway G
/// (simulating the strict-link failure where subscriber interest never wakes),
/// a remote demand token STILL flips G's egress flag — driven ONLY by the
/// reconciler's bounded liveliness QUERY. Attribution: `reconciler_enabled_count`
/// grows and the flag flips despite the subscriber being unable to touch it.
#[test]
fn reconciler_flips_flag_from_live_demand_while_subscriber_suppressed() {
    let id = unique_id();
    let topic = format!("/rec/live/{id}");

    // ---- G: gateway (listen), AllowAll, announces the topic, subscriber path
    // SUPPRESSED (set BEFORE new() so the watch subscriber is born suppressed).
    let mut g_state: Option<(GatewayRuntime, u16)> = None;
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let g = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("rec_g_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    robot_identity: Some("rec-g".to_string()),
                    ..Default::default()
                }),
                ..Default::default()
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .expect("init G");
        // Suppress the subscriber flag-flip BEFORE the watch starts.
        g.network()
            .expect("G network")
            .set_suppress_live_demand_for_test(true);
        let plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![topic.clone()],
            ingress: vec![],
        };
        match GatewayRuntime::new(g, plan) {
            Ok(gateway) => {
                g_state = Some((gateway, port));
                break;
            }
            Err(e) => eprintln!("attempt {attempt}: G session failed (port {port}): {e}"),
        }
    }
    let Some((mut gateway, port)) = g_state else {
        panic!("could not establish G's listening session in 3 attempts");
    };

    // ---- B: distinct root, connects to G, declares the demand token.
    let b = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("rec_b_{id}"),
            network: Some(NetworkConfig {
                connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                ..Default::default()
            }),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init B");
    b.register_ingress_topic(&topic, Vector3::SCHEMA_HASH, MaxSliceLen::const_new(256))
        .expect("B register_ingress_topic (declares the demand token)");

    // The flag handle on G (register_topic is idempotent — returns the boot flag).
    let flag = gateway
        .manager()
        .bridge_manager()
        .register_topic(&topic)
        .expect("G bridge flag handle");

    // Drive the reconciler synchronously until the flag flips (the subscriber is
    // suppressed, so ONLY the reconciler query can flip it). Bounded — no hang.
    let deadline = Instant::now() + Duration::from_secs(20);
    while !flag.load(Ordering::Relaxed) {
        assert!(
            Instant::now() < deadline,
            "the reconciler did not flip G's egress flag within 20s (subscriber suppressed)"
        );
        gateway.reconcile_demand_once().expect("reconcile pass");
        std::thread::sleep(Duration::from_millis(50));
    }

    assert!(
        flag.load(Ordering::Relaxed),
        "the reconciler must flip the egress flag from a live demand token"
    );
    assert!(
        gateway.reconciler_enabled_count() >= 1,
        "the flip must be attributed to a reconciler-driven enable (>=1), not the \
         suppressed subscriber"
    );
    assert!(
        gateway.reconcile_pass_count() >= 1,
        "at least one reconcile pass must have run"
    );
}

/// Test 2 (pure oracle): 3-pass removal hysteresis. Demand present → enable; then
/// three consecutive absences before the flag disables (surviving absences #1 and
/// #2); a returning demand re-enables and HEALS the absence counter. Every value
/// is hand-oracled against the real `apply_demand_reconcile` body.
#[test]
fn reconciler_hysteresis_disables_only_after_three_absences() {
    let topic = "/rec/hyst".to_string();
    let bm = TopicBridgeManager::new();
    let flag = bm.register_topic(&topic).unwrap();
    bm.set_egress_allow_all().unwrap(); // admit the topic (deny-all is the default)
    let announced = vec![topic.clone()];
    let mut absence: HashMap<String, u32> = HashMap::new();
    let pass = AtomicU64::new(0);
    let enabled = AtomicU64::new(0);

    let present: HashSet<String> = std::iter::once(topic.clone()).collect();
    let absent: HashSet<String> = HashSet::new();

    // Pass 1: demand present → enable + track.
    apply_demand_reconcile_for_test(&present, &bm, &announced, &mut absence, &pass, &enabled);
    assert!(flag.load(Ordering::Relaxed), "demand present must enable");
    assert_eq!(pass.load(Ordering::Relaxed), 1);
    assert_eq!(enabled.load(Ordering::Relaxed), 1);

    // Absence #1 and #2: flag SURVIVES (hysteresis).
    apply_demand_reconcile_for_test(&absent, &bm, &announced, &mut absence, &pass, &enabled);
    assert!(flag.load(Ordering::Relaxed), "flag survives absence #1");
    apply_demand_reconcile_for_test(&absent, &bm, &announced, &mut absence, &pass, &enabled);
    assert!(flag.load(Ordering::Relaxed), "flag survives absence #2");

    // Absence #3: DISABLE (REMOVE_CONFIRM == 3).
    apply_demand_reconcile_for_test(&absent, &bm, &announced, &mut absence, &pass, &enabled);
    assert!(!flag.load(Ordering::Relaxed), "flag disables on absence #3");
    assert_eq!(pass.load(Ordering::Relaxed), 4, "four passes total");
    assert_eq!(
        enabled.load(Ordering::Relaxed),
        1,
        "only the one demand-present pass enabled"
    );

    // Demand returns → re-enable + the absence counter heals (re-tracks at 0).
    apply_demand_reconcile_for_test(&present, &bm, &announced, &mut absence, &pass, &enabled);
    assert!(flag.load(Ordering::Relaxed), "returning demand re-enables");
    assert_eq!(enabled.load(Ordering::Relaxed), 2);
    // Two absences now must NOT disable — the counter was healed to 0.
    apply_demand_reconcile_for_test(&absent, &bm, &announced, &mut absence, &pass, &enabled);
    apply_demand_reconcile_for_test(&absent, &bm, &announced, &mut absence, &pass, &enabled);
    assert!(
        flag.load(Ordering::Relaxed),
        "the absence counter healed — two absences are not enough after a re-enable"
    );
}

/// Test 3 (pure oracle): the reconciler NEVER widens egress past the allow-list.
/// A demanded topic OUTSIDE the egress allow-list has its flag refused (the gate
/// inside `enable_bridge`) while a listed sibling flips — the posture cannot be
/// widened by demand. A tracked, allow-listed topic still respects hysteresis in
/// test 2; here the point is the structural refusal.
#[test]
fn reconciler_never_widens_egress_past_the_allowlist() {
    let listed = "/rec/listed".to_string();
    let unlisted = "/rec/unlisted".to_string();
    let bm = TopicBridgeManager::new();
    let listed_flag = bm.register_topic(&listed).unwrap();
    let unlisted_flag = bm.register_topic(&unlisted).unwrap();
    bm.set_egress_allowlist(std::slice::from_ref(&listed))
        .unwrap();
    let announced = vec![listed.clone(), unlisted.clone()];
    let mut absence: HashMap<String, u32> = HashMap::new();
    let pass = AtomicU64::new(0);
    let enabled = AtomicU64::new(0);

    // BOTH demanded — the allow-list refuses the unlisted one structurally.
    let demand: HashSet<String> = [listed.clone(), unlisted.clone()].into_iter().collect();
    apply_demand_reconcile_for_test(&demand, &bm, &announced, &mut absence, &pass, &enabled);
    assert!(
        listed_flag.load(Ordering::Relaxed),
        "the listed topic's flag flips"
    );
    assert!(
        !unlisted_flag.load(Ordering::Relaxed),
        "the unlisted topic's flag NEVER flips — the reconciler cannot widen egress"
    );

    // Repeated demand still cannot widen it.
    apply_demand_reconcile_for_test(&demand, &bm, &announced, &mut absence, &pass, &enabled);
    assert!(
        !unlisted_flag.load(Ordering::Relaxed),
        "still refused on the next pass"
    );
    assert!(
        listed_flag.load(Ordering::Relaxed),
        "the listed topic stays enabled"
    );
}

/// Test 4 (pure oracle): the tracking guard is SELECTIVE. A flag
/// set true OUT-OF-BAND (as the live liveliness subscriber would — NOT via the
/// reconciler) for a topic the reconciler NEVER sees demand for SURVIVES every
/// reconcile pass (the guard protects never-observed flags). A sibling topic
/// whose demand the reconciler DID observe (once) is absence-disabled at exactly
/// pass 3 — the two in one announce list make the guard's selectivity a sharp,
/// checkable contrast (comment out the `absence.get_mut` tracking
/// guard and the guarded flag disables at pass 3, failing this test).
#[test]
fn reconciler_tracking_guard_protects_never_observed_flag() {
    let guarded = "/rec/guarded".to_string(); // set true out-of-band, never demanded
    let observed = "/rec/observed".to_string(); // demanded once, then vanishes
    let bm = TopicBridgeManager::new();
    let guarded_flag = bm.register_topic(&guarded).unwrap();
    let observed_flag = bm.register_topic(&observed).unwrap();
    bm.set_egress_allow_all().unwrap(); // admit both (deny-all is the default)
    let announced = vec![guarded.clone(), observed.clone()];
    let mut absence: HashMap<String, u32> = HashMap::new();
    let pass = AtomicU64::new(0);
    let enabled = AtomicU64::new(0);

    // Out-of-band enable of `guarded` — simulates the live liveliness subscriber
    // flipping it, NOT the reconciler. It is never in any demand gather.
    let transitioned = bm.enable_bridge(&guarded).unwrap();
    assert!(
        transitioned,
        "the first out-of-band enable transitions the guarded flag false→true"
    );
    assert!(guarded_flag.load(Ordering::Relaxed));

    // Pass 1: ONLY `observed` is demanded → reconciler adopts its lifecycle.
    let demand_observed: HashSet<String> = std::iter::once(observed.clone()).collect();
    apply_demand_reconcile_for_test(
        &demand_observed,
        &bm,
        &announced,
        &mut absence,
        &pass,
        &enabled,
    );
    assert!(
        observed_flag.load(Ordering::Relaxed),
        "the observed topic is enabled from its demand token"
    );
    assert_eq!(
        enabled.load(Ordering::Relaxed),
        1,
        "only the reconciler-observed enable is a reconciler-caused transition; the \
         out-of-band guarded flag is NOT attributed to the reconciler"
    );

    // Demand now vanishes entirely. `guarded` is never demanded and never
    // tracked → the guard leaves it alone across every pass; `observed` was
    // tracked → disables at exactly the 3rd consecutive absence.
    let empty: HashSet<String> = HashSet::new();
    apply_demand_reconcile_for_test(&empty, &bm, &announced, &mut absence, &pass, &enabled); // #1
    assert!(
        guarded_flag.load(Ordering::Relaxed),
        "guarded survives absence #1"
    );
    assert!(
        observed_flag.load(Ordering::Relaxed),
        "observed survives absence #1"
    );
    apply_demand_reconcile_for_test(&empty, &bm, &announced, &mut absence, &pass, &enabled); // #2
    assert!(
        guarded_flag.load(Ordering::Relaxed),
        "guarded survives absence #2"
    );
    assert!(
        observed_flag.load(Ordering::Relaxed),
        "observed survives absence #2"
    );
    apply_demand_reconcile_for_test(&empty, &bm, &announced, &mut absence, &pass, &enabled); // #3
    assert!(
        guarded_flag.load(Ordering::Relaxed),
        "the guard PROTECTS the never-observed flag — it survives the 3rd absence"
    );
    assert!(
        !observed_flag.load(Ordering::Relaxed),
        "the reconciler-observed topic disables at exactly the 3rd consecutive absence"
    );

    // Further absences (pass 4, 5) still never touch the guarded flag — the guard
    // protects a never-observed flag INDEFINITELY (3+ passes survive).
    apply_demand_reconcile_for_test(&empty, &bm, &announced, &mut absence, &pass, &enabled);
    apply_demand_reconcile_for_test(&empty, &bm, &announced, &mut absence, &pass, &enabled);
    assert!(
        guarded_flag.load(Ordering::Relaxed),
        "the never-observed flag survives 5 absence passes — the guard never disables it"
    );
}

/// Test 5: an ingress-only gateway (empty `announce`) starts NO
/// demand reconciler — nothing to reconcile. Observable via
/// `reconcile_pass_count` staying 0 across a window LONGER than the reconcile
/// interval (~1 s): were a background thread running, it would bump the count.
#[test]
fn ingress_only_gateway_starts_no_reconciler() {
    let id = unique_id();
    let topic = format!("/rec/ingress_only/{id}");

    // A listen gateway with empty announce + one ingress entry (ingress-only).
    let mut g_state: Option<GatewayRuntime> = None;
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let g = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("rec_ing_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    robot_identity: Some("rec-ingress".to_string()),
                    ..Default::default()
                }),
                ..Default::default()
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .expect("init G (ingress-only)");
        let plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowList(vec![]), // deny-all = ingress-only
            announce: vec![],
            ingress: vec![GatewayIngressEntry {
                topic: topic.clone(),
                schema_hash: Vector3::SCHEMA_HASH,
            }],
        };
        match GatewayRuntime::new(g, plan) {
            Ok(gateway) => {
                g_state = Some(gateway);
                break;
            }
            Err(e) => {
                eprintln!("attempt {attempt}: ingress-only session failed (port {port}): {e}")
            }
        }
    }
    let Some(gateway) = g_state else {
        panic!("could not establish the ingress-only gateway session in 3 attempts");
    };

    // Wait a window well past RECONCILE_INTERVAL (~1 s). A running reconciler
    // thread would have bumped the count at least once by now.
    let window = Instant::now() + Duration::from_millis(1600);
    while Instant::now() < window {
        assert_eq!(
            gateway.reconcile_pass_count(),
            0,
            "an ingress-only gateway (empty announce) must NOT run the reconciler — no pass \
             may ever be counted"
        );
        std::thread::sleep(Duration::from_millis(100));
    }
    assert_eq!(
        gateway.reconcile_pass_count(),
        0,
        "reconcile_pass_count stayed 0 for the whole window — the reconciler was never started"
    );
}

/// Test 6: the REAL background reconciler THREAD advances passes
/// on a live announcing gateway, and its Drop join is PROMPT. Poll-with-deadline
/// (generous bounds, no exact-count asserts) keeps it flake-proof.
#[test]
fn reconciler_background_thread_advances_and_joins_promptly() {
    let id = unique_id();
    let topic = format!("/rec/bg/{id}");

    // A listen gateway that ANNOUNCES a topic → the reconciler thread starts and
    // its per-second gather (empty demand is fine — the pass still counts) bumps
    // reconcile_pass_count. AllowAll so the announce flag is registered.
    let mut g_state: Option<GatewayRuntime> = None;
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let g = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("rec_bg_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    robot_identity: Some("rec-bg".to_string()),
                    ..Default::default()
                }),
                ..Default::default()
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .expect("init G (background)");
        let plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![topic.clone()],
            ingress: vec![],
        };
        match GatewayRuntime::new(g, plan) {
            Ok(gateway) => {
                g_state = Some(gateway);
                break;
            }
            Err(e) => {
                eprintln!("attempt {attempt}: background gateway session failed (port {port}): {e}")
            }
        }
    }
    let Some(gateway) = g_state else {
        panic!("could not establish the background gateway session in 3 attempts");
    };

    // Poll (generous 10 s deadline) until the ACTUAL interval thread has run at
    // least 2 passes — the passes are ~1 s apart (thread sleeps first), so this
    // is ~2-3 s typically. No exact-count assert (the thread's cadence vs the
    // sync seam is not deterministic).
    let deadline = Instant::now() + Duration::from_secs(10);
    while gateway.reconcile_pass_count() < 2 {
        assert!(
            Instant::now() < deadline,
            "the background reconciler thread did not advance to >=2 passes within 10s (got {})",
            gateway.reconcile_pass_count()
        );
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        gateway.reconcile_pass_count() >= 2,
        "the real interval thread advanced the pass count"
    );

    // Drop the gateway (its only manager Arc) → NetworkManager::Drop stops the
    // reconciler thread (broadcast + join). The join must be PROMPT — the thread
    // polls the shutdown signal every <=50 ms.
    let drop_start = Instant::now();
    drop(gateway);
    let elapsed = drop_start.elapsed();
    assert!(
        elapsed < Duration::from_secs(2),
        "dropping the gateway must join the reconciler thread promptly (<2s); took {elapsed:?}"
    );
}

// ---------------------------------------------------------------------------
// Runtime-registered egress topics. A `cerulion ros2 attach` robot's
// dds_bridge creates ~90 SHM publishers at RUNTIME (raw routes like
// `/utlidar/cloud`); those are not in the boot GatewayPlan, so a plan-only
// gateway would refuse remote demand for them and never announce them. The gateway
// core exposes `register_runtime_topic`, which makes a runtime topic BOTH
// announced (discovery truth) AND demand-grantable exactly like a plan-declared
// producer, and `drive_once` reconciles it into the tap set.
//
// Demand is driven DETERMINISTICALLY by flipping the bridge flag (the effect a
// demand token has — the queryable AND liveliness watch both call
// `enable_bridge`), so these pins are zenoh-timing-free.
// ---------------------------------------------------------------------------

/// (a) Headline: a topic registered AT RUNTIME becomes announced + demand-
/// grantable + tapped + forwarded — everything a boot-plan topic gets. Hand
/// oracles throughout (never a self-compare).
#[test]
fn runtime_registered_topic_is_announced_grantable_and_forwarded() {
    let boot_topic = format!("/boot/{}", unique_id());
    let runtime_topic = format!("/runtime/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![boot_topic.clone()],
        ingress: vec![],
    };
    let (p, clock, mut gateway, _boot_pubs) =
        make_producer_and_gateway("rra", plan, std::slice::from_ref(&boot_topic));

    // dds_bridge-style: the runtime topic's producer publisher is created AT
    // RUNTIME (it did not exist at gateway boot).
    let mut runtime_pub = p
        .create_publisher_simple(&runtime_topic, MaxSliceLen::const_new(256))
        .expect("runtime producer publisher");

    // Before registration: not runtime-registered, not announced.
    assert!(
        !gateway
            .is_runtime_registered(&runtime_topic)
            .expect("pre is_runtime_registered"),
        "the runtime topic is not registered before register_runtime_topic"
    );
    assert!(
        !gateway
            .manager()
            .network()
            .expect("network")
            .announced_topics()
            .contains(&runtime_topic),
        "the runtime topic is not announced before registration"
    );

    // Register at runtime → genuinely new.
    assert!(
        gateway
            .register_runtime_topic(&runtime_topic)
            .expect("register runtime"),
        "a fresh runtime registration returns Ok(true)"
    );

    // Principle #3 (FRESH — no drive pass): announced + registered + runtime-tagged.
    assert_eq!(
        gateway.runtime_registered_topics().expect("rt topics"),
        vec![runtime_topic.clone()],
        "the runtime topic is reported as runtime-registered immediately"
    );
    assert_eq!(gateway.runtime_registered_count().expect("rt count"), 1);
    assert!(gateway
        .is_runtime_registered(&runtime_topic)
        .expect("is runtime"));
    assert!(
        !gateway
            .is_runtime_registered(&boot_topic)
            .expect("boot not rt"),
        "a boot-plan topic is NEVER reported as runtime-registered"
    );
    assert_eq!(
        gateway.registered_topic_count().expect("total"),
        2,
        "boot + runtime = 2 registered egress topics"
    );
    // Discovery truth: BOTH topics announced on the network.
    let mut announced = gateway
        .manager()
        .network()
        .expect("network")
        .announced_topics();
    announced.sort();
    let mut want_announced = vec![boot_topic.clone(), runtime_topic.clone()];
    want_announced.sort();
    assert_eq!(
        announced, want_announced,
        "both boot + runtime topics are announced (discovery truth)"
    );

    // GRANTABLE exactly like the boot topic: a demand token transitions its flag
    // + `is_enabled` (the ack the queryable would return) — the demand grant
    // path (queryable + liveliness watch) both go through `enable_bridge`.
    let bm = gateway.manager().bridge_manager();
    assert!(
        bm.enable_bridge(&runtime_topic).expect("enable runtime"),
        "a demand token transitions the runtime topic's flag — grantable"
    );
    assert!(
        bm.is_enabled(&runtime_topic).expect("is_enabled runtime"),
        "the runtime topic egresses after demand (grantable like a plan topic)"
    );

    // Drive → the runtime topic's tap attaches, then forwards exactly 2 frames.
    let _ = gateway.drive_once().expect("tap attach pass");
    assert!(
        gateway.active_tap_topics().contains(&runtime_topic),
        "the runtime topic's tap attaches on the demand pass"
    );
    publish_one(&mut runtime_pub, &clock, 10, (1.0, 2.0, 3.0));
    publish_one(&mut runtime_pub, &clock, 20, (4.0, 5.0, 6.0));
    let forwarded = drive_until(&mut gateway, 2, &runtime_topic);
    assert_eq!(forwarded, 2, "both runtime-topic frames forwarded");
    assert_eq!(gateway.forwarded_count(&runtime_topic), 2);
}

/// (b) A registration in the reserved control-plane namespace (`__cerulion/*` —
/// where the `__cerulion/gateway_topics` control service lives) is refused
/// LOUDLY, naming the topic + the reason; an empty name is refused too; and a
/// non-reserved lookalike is accepted. Nothing refused is registered or announced.
#[test]
fn runtime_registration_refuses_reserved_namespace_and_empty_name() {
    let boot_topic = format!("/breserved/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![boot_topic.clone()],
        ingress: vec![],
    };
    let (p, _clock, gateway, _boot_pubs) =
        make_producer_and_gateway("rrb", plan, std::slice::from_ref(&boot_topic));
    let _ = &p;

    // The control service name, the bare namespace, and raw spellings.
    for reserved in [
        "__cerulion/gateway_topics",
        "/__cerulion/gateway_topics",
        "/__cerulion",
        "__cerulion",
    ] {
        let err = gateway
            .register_runtime_topic(reserved)
            .expect_err("reserved namespace must refuse");
        let msg = format!("{err}");
        assert!(
            msg.contains("reserved") && msg.contains("__cerulion"),
            "the refusal must name the reserved namespace + reason: {msg}"
        );
    }
    // Nothing reserved was registered or announced.
    assert_eq!(
        gateway.registered_topic_count().expect("count"),
        1,
        "only the boot topic is registered — no reserved topic slipped through"
    );
    assert!(
        !gateway
            .manager()
            .network()
            .expect("network")
            .announced_topics()
            .iter()
            .any(|t| t.contains("__cerulion")),
        "no reserved topic was announced"
    );

    // A distinct name that merely shares the leading letters is NOT reserved.
    assert!(
        gateway
            .register_runtime_topic("/__cerulionx/ok")
            .expect("non-reserved lookalike accepted"),
        "a name outside the namespace boundary is not reserved"
    );

    // An empty name is refused, naming the cause.
    let err = gateway
        .register_runtime_topic("")
        .expect_err("empty name must refuse");
    assert!(
        format!("{err}").contains("non-empty"),
        "the empty-name refusal names the cause: {err}"
    );
}

/// (c) Re-registration is idempotent: a second (or raw-spelled) call returns
/// `Ok(false)` and adds no duplicate registration, announce, or counter.
#[test]
fn runtime_registration_is_idempotent() {
    let boot_topic = format!("/cboot/{}", unique_id());
    let runtime_topic = format!("/idem/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![boot_topic.clone()],
        ingress: vec![],
    };
    let (p, _clock, gateway, _boot_pubs) =
        make_producer_and_gateway("rrc", plan, std::slice::from_ref(&boot_topic));
    let _ = &p;

    assert!(
        gateway
            .register_runtime_topic(&runtime_topic)
            .expect("first register"),
        "the first registration is new"
    );
    let announced_after_first = gateway
        .manager()
        .network()
        .expect("network")
        .announced_topic_count();

    // Re-register the canonical name, then a raw (no-slash) spelling → both no-ops.
    assert!(
        !gateway
            .register_runtime_topic(&runtime_topic)
            .expect("second register"),
        "re-registering the same canonical topic is not new"
    );
    let raw = runtime_topic.trim_start_matches('/').to_string();
    assert!(
        !gateway
            .register_runtime_topic(&raw)
            .expect("raw-spelling register"),
        "the raw spelling of an already-registered topic is a duplicate"
    );

    // No duplicate registration, runtime entry, or announce.
    assert_eq!(
        gateway.registered_topic_count().expect("count"),
        2,
        "boot + exactly one runtime topic — no duplicate registrations"
    );
    assert_eq!(
        gateway.runtime_registered_topics().expect("rt topics"),
        vec![runtime_topic.clone()],
        "the runtime set holds the topic exactly once"
    );
    assert_eq!(
        gateway
            .manager()
            .network()
            .expect("network")
            .announced_topic_count(),
        announced_after_first,
        "a duplicate registration announces nothing new"
    );
}

/// (d) The Principle-#3 accessors distinguish boot from runtime, sort + count
/// exactly, and `drive_once` reconciles runtime registrations into the drive
/// view. Fresh accessors reflect a registration BEFORE any drive pass.
#[test]
fn runtime_accessors_distinguish_boot_and_reconcile_into_drive_view() {
    let boot_topic = format!("/dboot/{}", unique_id());
    // Two runtime topics registered OUT of sorted order (bbb before aaa).
    let rt_b = format!("/d/bbb_{}", unique_id());
    let rt_a = format!("/d/aaa_{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![boot_topic.clone()],
        ingress: vec![],
    };
    let (p, _clock, mut gateway, _boot_pubs) =
        make_producer_and_gateway("rrd", plan, std::slice::from_ref(&boot_topic));
    let _ = &p;

    // Boot: 1 registered, 0 runtime; the drive view already tracks the boot topic.
    assert_eq!(gateway.registered_topic_count().expect("c0"), 1);
    assert_eq!(gateway.runtime_registered_count().expect("rc0"), 0);
    assert!(gateway.runtime_registered_topics().expect("rt0").is_empty());
    assert_eq!(
        gateway.egress_view_count(),
        1,
        "boot topic is in the drive view"
    );

    assert!(gateway.register_runtime_topic(&rt_b).expect("reg b"));
    assert!(gateway.register_runtime_topic(&rt_a).expect("reg a"));

    // FRESH accessors (no drive pass yet): sorted runtime set, exact counts,
    // boot + unregistered excluded.
    let mut want_rt = vec![rt_a.clone(), rt_b.clone()];
    want_rt.sort();
    assert_eq!(
        gateway.runtime_registered_topics().expect("rt"),
        want_rt,
        "the runtime set is sorted regardless of registration order"
    );
    assert_eq!(gateway.runtime_registered_count().expect("rc"), 2);
    assert_eq!(gateway.registered_topic_count().expect("c"), 3);
    assert!(gateway.is_runtime_registered(&rt_a).expect("a rt"));
    assert!(gateway.is_runtime_registered(&rt_b).expect("b rt"));
    assert!(
        !gateway.is_runtime_registered(&boot_topic).expect("boot"),
        "a boot topic is not runtime-registered"
    );
    assert!(
        !gateway
            .is_runtime_registered("/never/registered")
            .expect("unreg"),
        "an unregistered topic is not runtime-registered"
    );

    // The DRIVE VIEW is stale until a pass; one `drive_once` reconciles the two
    // runtime topics in (their flags are OFF, so no tap attaches — pure reconcile).
    assert_eq!(
        gateway.egress_view_count(),
        1,
        "the drive view is stale before a drive pass"
    );
    gateway.drive_once().expect("reconcile pass");
    assert_eq!(
        gateway.egress_view_count(),
        3,
        "drive_once reconciled the two runtime topics into the drive view"
    );
}

/// (f) Under a STRICT allow-list posture (not the permissive robot default), a
/// runtime topic is still ANNOUNCED (discovery truth), but the egress gate
/// REFUSES to grant it (it is not in the declared list) — while the listed boot
/// topic is grantable (the anti-tautology). A gateway always has a network
/// (`GatewayRuntime::new` refuses a network-less manager), so "no network" is
/// unreachable; the meaningful "restrictive posture" case is pinned here.
#[test]
fn runtime_registration_under_strict_posture_announces_but_gate_refuses_egress() {
    let boot_topic = format!("/eboot/{}", unique_id());
    let runtime_topic = format!("/e/rt/{}", unique_id());
    // Strict allow-list naming ONLY the boot topic; the runtime topic is absent.
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowList(vec![boot_topic.clone()]),
        announce: vec![boot_topic.clone()],
        ingress: vec![],
    };
    let (p, _clock, gateway, _boot_pubs) =
        make_producer_and_gateway("rre", plan, std::slice::from_ref(&boot_topic));
    let _ = &p;

    // Registration succeeds + announces (discovery truth holds under Strict).
    assert!(
        gateway
            .register_runtime_topic(&runtime_topic)
            .expect("register under strict"),
        "a runtime registration succeeds regardless of the egress posture"
    );
    assert!(gateway
        .is_runtime_registered(&runtime_topic)
        .expect("is runtime"));
    assert!(
        gateway
            .manager()
            .network()
            .expect("network")
            .announced_topics()
            .contains(&runtime_topic),
        "a runtime topic is announced even under a strict posture (discovery truth)"
    );

    // The Strict gate REFUSES egress for the non-listed runtime topic: a demand
    // token does not transition its flag and it never egresses.
    let bm = gateway.manager().bridge_manager();
    assert!(
        !bm.enable_bridge(&runtime_topic).expect("enable runtime"),
        "a strict allow-list refuses egress for a non-listed runtime topic (no transition)"
    );
    assert!(
        !bm.is_enabled(&runtime_topic).expect("is_enabled runtime"),
        "the non-listed runtime topic must not egress under Strict"
    );
    // Anti-tautology: the LISTED boot topic IS grantable under the same gate.
    assert!(
        bm.enable_bridge(&boot_topic).expect("enable boot"),
        "the listed boot topic is grantable — the gate is load-bearing, not blanket-deny"
    );
    assert!(bm.is_enabled(&boot_topic).expect("is_enabled boot"));
}

/// (g) The announce-failure path is a LOUD partial commit
/// that HEALS on retry, not a silent one. `register_runtime_topic` commits the
/// demand flag (add-only, grantable) BEFORE the reachably-fallible network
/// announce; when the announce fails the flag is RETAINED (conservative — demand
/// still works, only discovery is degraded) and the fn warns EXACTLY ONCE naming
/// the retained registered-but-unannounced state, then propagates the Err. A
/// later registration (fault cleared) re-attempts the announce idempotently and
/// succeeds. Driven by the fire-once `fault_inject_announce_egress_once` seam
/// (no broken zenoh session needed); hand oracles on every count.
#[test]
#[traced_test]
fn runtime_registration_announce_failure_warns_retains_flag_and_heals() {
    let boot_topic = format!("/gboot/{}", unique_id());
    let runtime_topic = format!("/g/rt/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![boot_topic.clone()],
        ingress: vec![],
    };
    let (p, _clock, gateway, _boot_pubs) =
        make_producer_and_gateway("rrg", plan, std::slice::from_ref(&boot_topic));
    let _ = &p;

    // Arm the fire-once announce fault: the NEXT announce_egress_topic fails
    // (boot's announce already ran, so this fires on the runtime topic).
    gateway
        .manager()
        .network()
        .expect("network")
        .fault_inject_announce_egress_once();

    // (a) register_runtime_topic returns Err (the announce failed).
    let err = gateway
        .register_runtime_topic(&runtime_topic)
        .expect_err("a failed announce must propagate as Err");
    assert!(
        format!("{err}").contains("announce"),
        "the propagated error names the announce failure: {err}"
    );

    // (c) The flag IS registered despite the failure — RETAINED, not rolled back
    // (demand-grantable) — while discovery truth is ABSENT (no token declared).
    assert_eq!(
        gateway.registered_topic_count().expect("count"),
        2,
        "boot + the RETAINED runtime flag = 2 (the flag is not rolled back)"
    );
    assert!(
        gateway
            .is_runtime_registered(&runtime_topic)
            .expect("is runtime"),
        "the runtime topic is registered + demand-grantable despite the announce failure"
    );
    assert!(
        !gateway
            .manager()
            .network()
            .expect("network")
            .announced_topics()
            .contains(&runtime_topic),
        "the announce failed — the runtime topic is NOT announced (discovery truth absent)"
    );

    // (b) EXACTLY ONE warn fired, naming the topic + the registered-but-
    // unannounced statement (the message substring is unique to this warn site
    // and the topic is unique per test → the count is robust in parallel).
    logs_assert(|lines: &[&str]| {
        // count_at_exclusively: the WARN token AND the level-free total of the
        // same conjunction, so neither a head demoted to INFO/ERROR nor a
        // SECOND copy of it at another level can read as the one warn.
        let warns = count_at_exclusively(
            lines,
            "WARN",
            &[
                "registered + demand-grantable but its network announce FAILED",
                &runtime_topic,
            ],
        )?;
        if warns != 1 {
            return Err(format!(
                "expected EXACTLY ONE announce-failure warn naming the topic + the \
                 registered-but-unannounced state, got {warns}"
            ));
        }
        Ok(())
    });

    // (d) HEAL: fault cleared, a second registration re-attempts the announce.
    // Returns Ok(false) — the flag was ALREADY committed on the failed call —
    // and the topic is now announced (exactly one new announce token).
    let announced_before_heal = gateway
        .manager()
        .network()
        .expect("network")
        .announced_topic_count();
    assert!(
        !gateway
            .register_runtime_topic(&runtime_topic)
            .expect("heal: the re-registration succeeds"),
        "the heal re-registration returns Ok(false) — the flag was already committed"
    );
    assert!(
        gateway
            .manager()
            .network()
            .expect("network")
            .announced_topics()
            .contains(&runtime_topic),
        "the heal re-attempts the announce — the runtime topic is now announced"
    );
    assert_eq!(
        gateway
            .manager()
            .network()
            .expect("network")
            .announced_topic_count(),
        announced_before_heal + 1,
        "exactly the runtime topic's announce was added by the heal"
    );
}

// ---------------------------------------------------------------------------
// The AllowAll-only permissive SHM probe + the empty-boot-plan
// demand surface. When a remote DEMAND names a topic the gateway has NOT
// registered AND the egress posture is permissive (AllowAll), the gateway probes
// local SHM (open-only, never creating) for a live service of that name; a live
// service is registered + announced ON DEMAND (announce-on-first-serve) and the
// demand GRANTS exactly like a plan-declared producer. A miss / a Strict posture
// / an ingress topic / a reserved name is refused. Both demand paths — the
// queryable AND the liveliness watch — funnel through the SAME probe body.
//
// Driven DETERMINISTICALLY by the sync seams the real demand paths share:
// `demand_query_grants_for_test` (the EXACT queryable decision — probe + enable +
// ack) and `liveliness_demand_grants_for_test` (the EXACT liveliness Put action —
// probe + enable), so these pins are zenoh-timing-free. Hand oracles throughout.
// ---------------------------------------------------------------------------

/// (a) HEADLINE: a remote demand for an UNregistered topic that is LIVE in local
/// SHM (a raw external publisher's, dds_bridge-style) is probe-served — registered
/// + announced + granted + tapped + forwarded, everything a boot-plan topic gets.
#[test]
fn permissive_probe_hit_serves_unregistered_live_topic_e2e() {
    let boot_topic = format!("/probe/a/boot/{}", unique_id());
    let probed_topic = format!("/probe/a/probed/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![boot_topic.clone()],
        ingress: vec![],
    };
    let (p, clock, mut gateway, _boot_pubs) =
        make_producer_and_gateway("ppa", plan, std::slice::from_ref(&boot_topic));

    // A raw external publisher creates + will publish an UNregistered topic in
    // local SHM (not in the boot plan) — the `ros2 attach` runtime raw route.
    let mut probed_pub = p
        .create_publisher_simple(&probed_topic, MaxSliceLen::const_new(256))
        .expect("probed producer publisher");

    // Before the demand: not registered, not announced, counters 0.
    assert!(!gateway
        .is_runtime_registered(&probed_topic)
        .expect("pre reg"));
    assert!(!gateway
        .manager()
        .network()
        .expect("network")
        .announced_topics()
        .contains(&probed_topic));
    assert_eq!(gateway.probe_hit_count(), 0);
    assert_eq!(gateway.probe_miss_count(), 0);

    // A demand arrives via the QUERYABLE decision → probe HIT → GRANTED.
    assert!(
        gateway.demand_query_grants_for_test(&probed_topic),
        "a live unregistered AllowAll topic is probe-served + granted"
    );

    // Observable: registered + announced + runtime-tagged, exactly one hit.
    assert!(gateway
        .is_runtime_registered(&probed_topic)
        .expect("post reg"));
    assert_eq!(gateway.probe_hit_count(), 1);
    assert_eq!(gateway.probe_miss_count(), 0);
    assert_eq!(gateway.probe_loop_refusal_count(), 0);
    assert!(
        gateway
            .manager()
            .network()
            .expect("network")
            .announced_topics()
            .contains(&probed_topic),
        "the probe-served topic is announced (discovery truth)"
    );

    // Drive → the tap attaches, then forwards exactly 2 frames (delivery oracle).
    let _ = gateway.drive_once().expect("tap attach pass");
    assert!(
        gateway.active_tap_topics().contains(&probed_topic),
        "the probe-served topic's tap attaches on the demand pass"
    );
    publish_one(&mut probed_pub, &clock, 10, (1.0, 2.0, 3.0));
    publish_one(&mut probed_pub, &clock, 20, (4.0, 5.0, 6.0));
    let forwarded = drive_until(&mut gateway, 2, &probed_topic);
    assert_eq!(forwarded, 2, "both probe-served frames forwarded");
    assert_eq!(gateway.forwarded_count(&probed_topic), 2);
}

/// (a2) CONVERGENCE: a probe hit via the QUERYABLE path and via the LIVELINESS
/// path both land the SAME registered + announced + enabled state — the two demand
/// paths behave identically. Distinct topics so each path serves
/// its own (a fresh hit), then both are asserted grantable + announced.
#[test]
fn both_demand_paths_serve_a_probe_hit_identically() {
    let via_query = format!("/probe/a2/query/{}", unique_id());
    let via_live = format!("/probe/a2/live/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![],
        ingress: vec![],
    };
    let (p, _clock, gateway, _pubs) = make_producer_and_gateway("ppa2", plan, &[]);
    let _q_pub = p
        .create_publisher_simple(&via_query, MaxSliceLen::const_new(256))
        .expect("query-path pub");
    let _l_pub = p
        .create_publisher_simple(&via_live, MaxSliceLen::const_new(256))
        .expect("live-path pub");

    // Queryable path serves `via_query`; liveliness path serves `via_live`.
    assert!(gateway.demand_query_grants_for_test(&via_query));
    assert!(gateway.liveliness_demand_grants_for_test(&via_live));

    // BOTH converge to registered + announced + enabled — identical end state.
    let bm = gateway.manager().bridge_manager();
    for t in [&via_query, &via_live] {
        assert!(
            gateway.is_runtime_registered(t).expect("reg"),
            "{t} registered"
        );
        assert!(bm.is_enabled(t).expect("enabled"), "{t} enabled/grantable");
        assert!(
            gateway
                .manager()
                .network()
                .expect("network")
                .announced_topics()
                .contains(t),
            "{t} announced"
        );
    }
    assert_eq!(gateway.probe_hit_count(), 2, "one hit per path");
    assert_eq!(gateway.probe_miss_count(), 0);
}

/// (b) MISS: a demand for a topic with NO live local SHM service is refused, the
/// miss counter bumps, and the open-only probe creates NOTHING — a fresh open-only
/// subscriber on the probed name still errors (no phantom service was minted).
#[test]
fn permissive_probe_miss_refuses_and_creates_no_service() {
    let boot_topic = format!("/probe/b/boot/{}", unique_id());
    let missing_topic = format!("/probe/b/missing/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![boot_topic.clone()],
        ingress: vec![],
    };
    let (p, _clock, gateway, _pubs) =
        make_producer_and_gateway("ppb", plan, std::slice::from_ref(&boot_topic));
    let _ = &p;

    assert!(
        !gateway.demand_query_grants_for_test(&missing_topic),
        "a demand for an absent topic is refused"
    );
    assert_eq!(gateway.probe_miss_count(), 1, "the miss is counted");
    assert_eq!(gateway.probe_hit_count(), 0);
    assert!(
        !gateway.is_runtime_registered(&missing_topic).expect("reg"),
        "a miss registers nothing"
    );
    assert!(!gateway
        .manager()
        .network()
        .expect("network")
        .announced_topics()
        .contains(&missing_topic));
    // The open-only pin: the probe created NO service — a fresh open-only tap on
    // the missing name still errors (zero-services delta on the probed name).
    assert!(
        gateway
            .manager()
            .create_data_only_subscriber(&missing_topic)
            .is_err(),
        "the probe must NOT have created a service for the missing topic"
    );
}

/// (c) STRICT never probes: the SAME live-SHM setup as (a), but under a Strict
/// (allow-list) posture the probe never runs — the demand is refused and the probe
/// counters are UNMOVED (the anti-tautology pair with (a): an explicit `network:`
/// block is a tightening, so unknown topics stay refused).
#[test]
fn strict_posture_never_probes_even_with_a_live_service() {
    let boot_topic = format!("/probe/c/boot/{}", unique_id());
    let live_topic = format!("/probe/c/live/{}", unique_id());
    // Strict = an allow-list carrying ONLY the boot topic.
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowList(vec![boot_topic.clone()]),
        announce: vec![boot_topic.clone()],
        ingress: vec![],
    };
    let (p, _clock, gateway, _pubs) =
        make_producer_and_gateway("ppc", plan, std::slice::from_ref(&boot_topic));

    // A LIVE SHM service for an unregistered topic exists (identical to (a)).
    let _live_pub = p
        .create_publisher_simple(&live_topic, MaxSliceLen::const_new(256))
        .expect("live pub");

    assert!(
        !gateway.demand_query_grants_for_test(&live_topic),
        "Strict never probes → the demand is refused"
    );
    assert_eq!(
        gateway.probe_hit_count(),
        0,
        "Strict must NOT probe a live service (anti-tautology vs (a), which hits it)"
    );
    assert_eq!(
        gateway.probe_miss_count(),
        0,
        "Strict short-circuits at the posture gate — not even a miss is counted"
    );
    assert!(!gateway.is_runtime_registered(&live_topic).expect("reg"));
    assert!(!gateway
        .manager()
        .network()
        .expect("network")
        .announced_topics()
        .contains(&live_topic));
}

/// (d1) THE EMPTY-BOOT-PLAN PIN (deterministic): an EMPTY-boot-plan gateway under a
/// PERMISSIVE (AllowAll) posture STARTS its query surface (the
/// verb-dispatched queryable serving demand + catalog) — the surface remote demand
/// arrives on (and the probe fires on). The anti-tautology: an empty NON-permissive
/// (deny-all) gateway starts NEITHER (it can never egress).
/// Reverting the empty-plan gating flips the AllowAll arm to `false`.
#[test]
fn empty_allow_all_plan_starts_query_surface_but_deny_all_does_not() {
    // Empty AllowAll plan → query surface STARTED.
    let allow_all = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![],
        ingress: vec![],
    };
    let (_pa, _ca, gw_all, _pua) = make_producer_and_gateway("ppd1all", allow_all, &[]);
    assert!(
        gw_all.query_surface_active(),
        "an empty AllowAll gateway starts the query surface"
    );

    // Empty deny-all plan → query surface NOT started.
    let deny_all = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowList(vec![]),
        announce: vec![],
        ingress: vec![],
    };
    let (_pd, _cd, gw_deny, _pud) = make_producer_and_gateway("ppd1deny", deny_all, &[]);
    assert!(
        !gw_deny.query_surface_active(),
        "an empty deny-all gateway starts NO query surface (it can never egress)"
    );
}

/// (d2) THE EMPTY-BOOT-PLAN PIN (e2e): on an EMPTY-boot-plan AllowAll gateway, a
/// control-channel registration AND a probe-hit topic are BOTH grantable +
/// served end-to-end — registered, granted, tapped, forwarded. The empty plan
/// fills entirely at runtime.
#[test]
fn empty_plan_gateway_serves_control_channel_and_probe_topics_e2e() {
    let ctrl_topic = format!("/probe/d2/ctrl/{}", unique_id());
    let probed_topic = format!("/probe/d2/probed/{}", unique_id());
    const CTRL_HASH: u64 = 0x0821_D2C0_0000_0001;
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![],
        ingress: vec![],
    };
    let (p, clock, mut gateway, _pubs) = make_producer_and_gateway("ppd2", plan, &[]);
    assert!(
        gateway.query_surface_active(),
        "the empty-plan gateway started its query surface"
    );

    // (1) A control-channel registration (a network-free worker's raw
    //     route). Create its SHM service + register it over the control channel.
    let mut ctrl_pub = p
        .create_publisher_simple(&ctrl_topic, MaxSliceLen::const_new(256))
        .expect("ctrl producer publisher");
    assert!(p
        .register_dynamic_egress_topic(&ctrl_topic, CTRL_HASH)
        .expect("register control topic"));
    // Settle: republish inline + drive the gateway (drains the control channel).
    for _ in 0..3 {
        p.republish_dynamic_egress_for_test();
        gateway.drive_once().expect("drive");
    }
    assert!(
        gateway
            .is_runtime_registered(&ctrl_topic)
            .expect("ctrl reg"),
        "the control-channel topic registered on the empty-plan gateway"
    );
    // Grant it (the demand effect) → drive → tap → forward.
    gateway
        .manager()
        .bridge_manager()
        .enable_bridge(&ctrl_topic)
        .expect("enable ctrl");
    let _ = gateway.drive_once().expect("ctrl attach pass");
    publish_one(&mut ctrl_pub, &clock, 10, (1.0, 0.0, 0.0));
    assert_eq!(
        drive_until(&mut gateway, 1, &ctrl_topic),
        1,
        "the control-channel topic forwards a frame"
    );

    // (2) A probe-hit topic on the SAME empty-plan gateway.
    let mut probed_pub = p
        .create_publisher_simple(&probed_topic, MaxSliceLen::const_new(256))
        .expect("probed producer publisher");
    assert!(
        gateway.demand_query_grants_for_test(&probed_topic),
        "a probe-hit topic is served on the empty-plan gateway"
    );
    assert_eq!(gateway.probe_hit_count(), 1);
    let _ = gateway.drive_once().expect("probed attach pass");
    publish_one(&mut probed_pub, &clock, 20, (2.0, 0.0, 0.0));
    assert_eq!(
        drive_until(&mut gateway, 1, &probed_topic),
        1,
        "the probe-served topic forwards a frame"
    );
}

/// (e) LOOP SAFETY: a topic this gateway INGRESSES (its local SHM service is the
/// gateway's own re-injection) is NEVER probe-served — a loud loop refusal naming
/// the loop, nothing registered / announced / granted. BOTH demand paths refuse it
/// identically.
///
/// NOTE the count is `>=`, not exact: the gateway's OWN ingress DEMAND token
/// round-trips to its own liveliness watch (history replay) and the background
/// watch ALSO loop-refuses it — a real proof the production liveliness path enforces
/// loop safety, but an async bump. My two explicit seam calls guarantee `>= 2`; the
/// deterministic invariants (never SERVED — `hit == 0`; not registered / announced /
/// granted) hold regardless of path.
#[traced_test]
#[test]
fn permissive_probe_refuses_an_ingress_topic_naming_the_loop() {
    let ingress_topic = format!("/probe/e/ingress/{}", unique_id());
    // AllowAll (so the probe is otherwise-eligible) + one ingress entry. The
    // gateway's boot registers the ingress → a live local re-injection SHM service.
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![],
        ingress: vec![GatewayIngressEntry {
            topic: ingress_topic.clone(),
            schema_hash: Vector3::SCHEMA_HASH,
        }],
    };
    let (p, _clock, gateway, _pubs) = make_producer_and_gateway("ppe", plan, &[]);
    let _ = &p;

    // The queryable path refuses the ingress topic (loop safety) — NOT granted.
    assert!(
        !gateway.demand_query_grants_for_test(&ingress_topic),
        "an ingress topic is never granted (echo-loop safety)"
    );
    assert!(
        gateway.probe_loop_refusal_count() >= 1,
        "the loop refusal is counted"
    );
    // The liveliness path refuses it identically (both paths converge).
    assert!(!gateway.liveliness_demand_grants_for_test(&ingress_topic));
    assert!(
        gateway.probe_loop_refusal_count() >= 2,
        "both demand paths refuse the loop (>= my 2 explicit calls; the background \
         self-token replay may add more)"
    );

    // DETERMINISTIC invariants — an ingress topic is NEVER served by any path.
    assert_eq!(
        gateway.probe_hit_count(),
        0,
        "an ingress topic is NEVER probe-served (its live SHM is the gateway's own re-injection)"
    );
    assert!(
        !gateway.is_runtime_registered(&ingress_topic).expect("reg"),
        "the ingress topic is not registered as egress"
    );
    assert!(
        !gateway
            .manager()
            .network()
            .expect("network")
            .announced_topics()
            .contains(&ingress_topic),
        "nothing announced for the refused ingress topic"
    );
    // The refusal is LOUD and names the loop (the echo-loop hazard the operator
    // must see). Captured on the test thread from the explicit seam calls; the
    // identifying phrase appears in BOTH the first warn and any latched debug line.
    // LOUD is a LEVEL claim: the first refusal is the latch's WARN head (the latch
    // is fresh and only a HIT re-arms it), so the WARN token is required — the
    // debug repeat carries the same phrase and must not satisfy this.
    logs_assert(|lines: &[&str]| {
        logged_at(lines, "WARN", "network INGRESS topic — echo-loop")
            .map_err(|e| format!("the loop refusal names the ingress/echo-loop hazard — {e}"))
    });
}

/// The asymmetric-guard pin: the QUERYABLE demand path (not just the
/// liveliness watch) must SKIP `enable_bridge` for a topic THIS gateway
/// INGRESSES. A STRICT (AllowList) gateway that declares BOTH egress and ingress
/// topics runs its OWN demand-GET loop over its ingress topics, self-querying
/// this queryable every pass; without the skip the queryable's `enable_bridge`
/// refuses the ingress topic (ingress∩egress=∅ by `GatewayPlan::validate`) and
/// emits the wrong-audience WARN about the machine's OWN ingress. The observable
/// is the durable per-topic egress-refusal latch (`was_egress_refused`) — a WARN
/// on the callback thread is not `tracing_test`-capturable. Hand oracle: the
/// self-ingress GET is NOT granted and NOT egress-refused (skipped before enable);
/// a genuinely-foreign GET (a topic this gateway neither produces nor
/// egress-lists) IS refused — the anti-tautology control proving the refusal path
/// stays alive on the queryable side. Removing the queryable
/// self-ingress skip flips `was_egress_refused(ingress_topic)` to `true`.
#[test]
fn queryable_skips_self_ingress_demand_but_refuses_foreign() {
    let ingress_topic = format!("/probe/ingress/{}", unique_id());
    let egress_topic = format!("/probe/egress/{}", unique_id());
    let foreign_topic = format!("/probe/foreign/{}", unique_id());
    // STRICT gateway: an AllowList admitting ONLY `egress_topic`, plus an ingress
    // entry — ingress∩egress=∅ (GatewayPlan::validate).
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowList(vec![egress_topic.clone()]),
        announce: vec![egress_topic.clone()],
        ingress: vec![GatewayIngressEntry {
            topic: ingress_topic.clone(),
            schema_hash: Vector3::SCHEMA_HASH,
        }],
    };
    let (p, _clock, gateway, _pubs) = make_producer_and_gateway("qsi", plan, &[]);
    let _ = &p;
    let bridge = gateway.manager().bridge_manager();

    // Self-ingress GET: refused (not granted) AND NOT egress-refused — the
    // queryable skips enable_bridge before the wrong-audience warn.
    assert!(
        !gateway.demand_query_grants_for_test(&ingress_topic),
        "a self-ingress demand GET must not be granted (echo-loop safety)"
    );
    assert!(
        !bridge
            .was_egress_refused(&ingress_topic)
            .expect("gate readable"),
        "the queryable must NOT egress-refuse the gateway's OWN ingress topic (no \
         wrong-audience warn about our own demand)"
    );

    // Control: a genuinely-foreign topic (not produced, not egress-listed) IS
    // egress-refused by the queryable — the genuine-refusal path stays alive.
    assert!(
        !gateway.demand_query_grants_for_test(&foreign_topic),
        "a foreign topic is not granted under a Strict allow-list"
    );
    assert!(
        bridge
            .was_egress_refused(&foreign_topic)
            .expect("gate readable"),
        "control: a foreign demand GET for a topic the gateway does not egress-list IS \
         refused (the genuine-refusal path stays alive on the queryable side)"
    );
}

/// (e2) LOOP SAFETY (reserved control plane): the gateway's boot opens the
/// control service (`/__cerulion/gateway_topics` — a LIVE SHM service), so
/// an UNguarded probe would FIND + serve it, leaking the control plane onto the
/// network. A demand for a reserved name is REFUSED even though the service is live
/// (defense in depth — the SAME reserved predicate `register_runtime_topic` uses).
/// Removing the reserved guard turns this into a `hit`.
#[test]
fn permissive_probe_refuses_a_reserved_control_plane_name() {
    let boot_topic = format!("/probe/e2/boot/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![boot_topic.clone()],
        ingress: vec![],
    };
    let (p, _clock, gateway, _pubs) =
        make_producer_and_gateway("ppe2", plan, std::slice::from_ref(&boot_topic));
    let _ = &p;
    assert!(
        gateway.runtime_registration_active(),
        "the gateway opened the reserved control service at boot (it is LIVE in SHM)"
    );

    // The control service name — a live SHM service the probe MUST refuse.
    let reserved = "/__cerulion/gateway_topics";
    assert!(
        !gateway.demand_query_grants_for_test(reserved),
        "a reserved control-plane name is never granted"
    );
    assert_eq!(
        gateway.probe_hit_count(),
        0,
        "the LIVE control service is NEVER probe-served (no control-plane leak onto the network)"
    );
    assert!(
        gateway.probe_loop_refusal_count() >= 1,
        "the reserved refusal is counted (loop-class)"
    );
    assert!(
        !gateway
            .manager()
            .network()
            .expect("network")
            .announced_topics()
            .iter()
            .any(|t| t.contains("__cerulion")),
        "no reserved control-plane topic is announced"
    );
}

/// (f) HOSTILE FLOOD: N repeated demands for distinct MISSING names are all
/// counted, but the miss WARN fires ONCE per regime (not per demand — bounded log,
/// bounded state), and the queryable stays LIVE (a subsequent legitimate probe-hit
/// still works + forwards).
#[traced_test]
#[test]
fn permissive_probe_miss_flood_counts_all_warns_once_and_stays_live() {
    let boot_topic = format!("/probe/f/boot/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![boot_topic.clone()],
        ingress: vec![],
    };
    let (p, clock, mut gateway, _pubs) =
        make_producer_and_gateway("ppf", plan, std::slice::from_ref(&boot_topic));

    const N: u64 = 12;
    for i in 0..N {
        let missing = format!("/probe/f/miss/{}/{i}", unique_id());
        assert!(
            !gateway.demand_query_grants_for_test(&missing),
            "each flood demand for an absent topic is refused"
        );
    }
    assert_eq!(gateway.probe_miss_count(), N, "every flood miss is counted");

    // The MISS warn fires exactly ONCE per regime; the other N-1 ride debug.
    logs_assert(|lines: &[&str]| {
        // Level-free twin: a suppressed MISS repeat must never be LOUD — the half of the
        // contract that survives `release_max_level_info`, where the gated
        // DEBUG count reads 0.
        for level in ["WARN", "INFO", "ERROR"] {
            let loud = lines
                .iter()
                .filter(|l| {
                    line_level(l) == Some(level) && (l.contains("(repeat — see the first warning)"))
                })
                .count();
            if loud != 0 {
                return Err(format!(
                    "a suppressed MISS repeat was emitted at {level} ({loud} line(s))"
                ));
            }
        }
        // The loud head, matched WITH its level token AND against the
        // level-free total of the same marker: a head demoted to INFO/ERROR is
        // not the one MISS warn this pins, and a second copy of it at another
        // level is not one either.
        let warns = count_at_exclusively(lines, "WARN", &["refused (nothing created)"])?;
        // Counted AT DEBUG, not by text: a repeat re-emitted at `trace!` is
        // still one line, and the loud sweep above permits TRACE.
        let suppressed =
            count_at_exclusively(lines, "DEBUG", &["(repeat — see the first warning)"])?;
        if warns != 1 {
            return Err(format!("expected exactly 1 MISS warn, got {warns}"));
        }
        let want_suppressed = debug_lines_expected((N - 1) as usize);
        if suppressed != want_suppressed {
            return Err(format!(
                "expected {want_suppressed} suppressed MISS debug lines, got {suppressed}"
            ));
        }
        Ok(())
    });

    // The queryable stays LIVE — a subsequent legitimate probe-hit still works and
    // the gateway forwards its frames.
    let live_topic = format!("/probe/f/live/{}", unique_id());
    let mut live_pub = p
        .create_publisher_simple(&live_topic, MaxSliceLen::const_new(256))
        .expect("live pub");
    assert!(
        gateway.demand_query_grants_for_test(&live_topic),
        "a legit probe-hit works after a miss flood — the surface stays live"
    );
    assert_eq!(gateway.probe_hit_count(), 1);
    let _ = gateway.drive_once().expect("attach pass");
    publish_one(&mut live_pub, &clock, 10, (1.0, 2.0, 3.0));
    assert_eq!(
        drive_until(&mut gateway, 1, &live_topic),
        1,
        "the gateway still forwards after the flood"
    );
}

/// (g) IDEMPOTENT double-demand: a second demand for the SAME probeable topic
/// still GRANTS (already registered + enabled) but performs NO second registration,
/// announce, or hit count — one registration, one announce.
#[test]
fn permissive_probe_double_demand_registers_and_announces_once() {
    let boot_topic = format!("/probe/g/boot/{}", unique_id());
    let probed_topic = format!("/probe/g/probed/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![boot_topic.clone()],
        ingress: vec![],
    };
    let (p, _clock, gateway, _pubs) =
        make_producer_and_gateway("ppg", plan, std::slice::from_ref(&boot_topic));
    let _probed_pub = p
        .create_publisher_simple(&probed_topic, MaxSliceLen::const_new(256))
        .expect("probed pub");

    // First demand → HIT (registers + announces once).
    assert!(gateway.demand_query_grants_for_test(&probed_topic));
    assert_eq!(gateway.probe_hit_count(), 1);
    let announced_after_first = gateway
        .manager()
        .network()
        .expect("network")
        .announced_topic_count();

    // Second demand for the SAME topic → still GRANTS, but no second hit / register
    // / announce (idempotent).
    assert!(
        gateway.demand_query_grants_for_test(&probed_topic),
        "the second demand still grants (already registered + enabled)"
    );
    assert_eq!(
        gateway.probe_hit_count(),
        1,
        "no second probe hit — the register is idempotent (AlreadyRegistered)"
    );
    assert_eq!(
        gateway
            .manager()
            .network()
            .expect("network")
            .announced_topic_count(),
        announced_after_first,
        "no second announce — one token for the probeable topic"
    );
    assert_eq!(
        gateway.runtime_registered_topics().expect("rt"),
        vec![probed_topic.clone()],
        "registered exactly once"
    );
}

/// (h) DETERMINISM: the same probe stimulus (hit A, miss X, hit B, hit A again)
/// yields IDENTICAL hit/miss/loop counters across two independent runs, matching a
/// hand oracle (2 hits — the second A is an idempotent no-op — 1 miss, 0 loop).
#[test]
fn permissive_probe_counters_are_deterministic_across_two_runs() {
    fn run(tag: &str) -> (u64, u64, u64) {
        let boot_topic = format!("/probe/h/{tag}/boot/{}", unique_id());
        let live_a = format!("/probe/h/{tag}/a/{}", unique_id());
        let live_b = format!("/probe/h/{tag}/b/{}", unique_id());
        let missing = format!("/probe/h/{tag}/x/{}", unique_id());
        let plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![boot_topic.clone()],
            ingress: vec![],
        };
        let (p, _clock, gateway, _pubs) = make_producer_and_gateway(
            &format!("pph{tag}"),
            plan,
            std::slice::from_ref(&boot_topic),
        );
        let _pa = p
            .create_publisher_simple(&live_a, MaxSliceLen::const_new(256))
            .expect("pa");
        let _pb = p
            .create_publisher_simple(&live_b, MaxSliceLen::const_new(256))
            .expect("pb");
        // Stimulus: hit A, miss X, hit B, hit A again (idempotent no-op).
        gateway.demand_query_grants_for_test(&live_a);
        gateway.demand_query_grants_for_test(&missing);
        gateway.demand_query_grants_for_test(&live_b);
        gateway.demand_query_grants_for_test(&live_a);
        (
            gateway.probe_hit_count(),
            gateway.probe_miss_count(),
            gateway.probe_loop_refusal_count(),
        )
    }
    let run_a = run("A");
    let run_b = run("B");
    assert_eq!(
        run_a, run_b,
        "the probe's hit/miss/loop counters are deterministic across runs"
    );
    assert_eq!(
        run_a,
        (2, 1, 0),
        "hand oracle: 2 hits (A,B; the second A is idempotent), 1 miss, 0 loop refusals"
    );
}

/// (i) VERDICT TAXONOMY: the raw probe classifies each demand shape into its exact
/// verdict — `served` (live), `missing` (absent), `loop_refused` (ingress/reserved),
/// `already_registered` (idempotent), and `not_permissive` (Strict). One oracle per
/// case (each verdict LABEL vs a hand-written string).
#[test]
fn probe_verdict_taxonomy_classifies_each_demand_shape() {
    // AllowAll gateway with one ingress topic (for the loop-refused case).
    let ingress_topic = format!("/probe/i/ingress/{}", unique_id());
    let live_topic = format!("/probe/i/live/{}", unique_id());
    let missing_topic = format!("/probe/i/missing/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![],
        ingress: vec![GatewayIngressEntry {
            topic: ingress_topic.clone(),
            schema_hash: Vector3::SCHEMA_HASH,
        }],
    };
    let (p, _clock, gateway, _pubs) = make_producer_and_gateway("ppi", plan, &[]);
    let _live_pub = p
        .create_publisher_simple(&live_topic, MaxSliceLen::const_new(256))
        .expect("live pub");

    // SERVED: a live unregistered topic.
    assert_eq!(gateway.probe_verdict_for_test(&live_topic), "served");
    // ALREADY_REGISTERED: the same topic again (registered by the serve above).
    assert_eq!(
        gateway.probe_verdict_for_test(&live_topic),
        "already_registered"
    );
    // MISSING: a topic with no live local SHM service.
    assert_eq!(gateway.probe_verdict_for_test(&missing_topic), "missing");
    // LOOP_REFUSED: an ingress topic (echo-loop safety).
    assert_eq!(
        gateway.probe_verdict_for_test(&ingress_topic),
        "loop_refused"
    );
    // LOOP_REFUSED: a reserved control-plane name (defense in depth).
    assert_eq!(
        gateway.probe_verdict_for_test("/__cerulion/gateway_topics"),
        "loop_refused"
    );

    // NOT_PERMISSIVE needs a Strict gateway — the SAME live-topic setup refuses to
    // probe (the "Strict never probes" decision).
    let strict_boot = format!("/probe/i/strictboot/{}", unique_id());
    let strict_live = format!("/probe/i/strictlive/{}", unique_id());
    let strict_plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowList(vec![strict_boot.clone()]),
        announce: vec![strict_boot.clone()],
        ingress: vec![],
    };
    let (ps, _cs, strict_gw, _spu) =
        make_producer_and_gateway("ppistrict", strict_plan, std::slice::from_ref(&strict_boot));
    let _strict_live_pub = ps
        .create_publisher_simple(&strict_live, MaxSliceLen::const_new(256))
        .expect("strict live pub");
    assert_eq!(
        strict_gw.probe_verdict_for_test(&strict_live),
        "not_permissive",
        "Strict never probes — even a live service classifies not_permissive"
    );
}

// ---------------------------------------------------------------------------
// Probe hardening: deferred-announce heal, reconciler-on-empty, boot TOCTOU close.
// ---------------------------------------------------------------------------

/// Deferred-announce heal: a probe HIT whose
/// network announce FAILS DEFERS the announce — the flag is RETAINED (registered +
/// grantable), the topic is NOT announced, and the deferred counter bumps. The NEXT
/// demand's `AlreadyRegistered` arm HEALS it: it re-attempts the announce (fault
/// cleared), which succeeds, announces the topic, and fires the recovery info EXACTLY
/// ONCE. Driven by the fire-once `fault_inject_announce_egress_once` seam (no broken
/// zenoh session). Hand oracles on every count.
#[test]
#[traced_test]
fn permissive_probe_announce_failure_defers_then_heals_on_next_demand() {
    let boot_topic = format!("/probe/def/boot/{}", unique_id());
    let probed_topic = format!("/probe/def/probed/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![boot_topic.clone()],
        ingress: vec![],
    };
    let (p, _clock, gateway, _boot_pubs) =
        make_producer_and_gateway("ppdef", plan, std::slice::from_ref(&boot_topic));
    // A live SHM service for the (unregistered) probed topic — a probe HIT candidate.
    let _probed_pub = p
        .create_publisher_simple(&probed_topic, MaxSliceLen::const_new(256))
        .expect("probed pub");

    // Arm the fire-once announce fault: boot's announce already ran, so the NEXT
    // announce_egress_topic (the probe's HIT) fails.
    gateway
        .manager()
        .network()
        .expect("network")
        .fault_inject_announce_egress_once();

    // First demand → probe HIT, announce FAILS → deferred. The verdict is still
    // SERVED (the flag is retained / grantable, NOT downgraded to a miss).
    assert_eq!(
        gateway.probe_verdict_for_test(&probed_topic),
        "served",
        "an announce failure does not downgrade a HIT — the flag is retained (served)"
    );
    assert_eq!(gateway.probe_hit_count(), 1, "exactly one hit");
    assert_eq!(
        gateway.probe_miss_count(),
        0,
        "a deferred hit is NOT a miss"
    );
    assert_eq!(
        gateway.probe_announce_deferred_count(),
        1,
        "the failed announce bumps the deferred counter (Principle #3)"
    );
    assert!(
        gateway.is_runtime_registered(&probed_topic).expect("reg"),
        "the flag is RETAINED (registered + demand-grantable) despite the announce failure"
    );
    assert!(
        !gateway
            .manager()
            .network()
            .expect("network")
            .is_announced(&probed_topic),
        "the failed announce leaves the topic UNannounced (discovery deferred)"
    );

    // Second demand (fault cleared) → AlreadyRegistered → HEAL: the announce
    // re-attempts idempotently and SUCCEEDS.
    assert_eq!(
        gateway.probe_verdict_for_test(&probed_topic),
        "already_registered",
        "the second demand hits the AlreadyRegistered heal arm"
    );
    assert!(
        gateway
            .manager()
            .network()
            .expect("network")
            .is_announced(&probed_topic),
        "the heal re-attempt announced the deferred topic (discovery truth restored)"
    );
    assert_eq!(
        gateway.probe_announce_deferred_count(),
        1,
        "the heal succeeded — the deferred counter does not bump again"
    );
    assert_eq!(
        gateway.probe_hit_count(),
        1,
        "no second hit — the heal rides the AlreadyRegistered arm"
    );

    // The recovery info fired EXACTLY ONCE, naming the topic + the heal.
    logs_assert(|lines: &[&str]| {
        // count_at_exclusively: the INFO token AND the level-free total (see
        // the announce-failure warn above).
        let heals =
            count_at_exclusively(lines, "INFO", &["deferred announce HEALED", &probed_topic])?;
        if heals != 1 {
            return Err(format!(
                "expected EXACTLY ONE deferred-announce heal info naming the topic, got {heals}"
            ));
        }
        Ok(())
    });
}

/// Reconciler-on-empty: an EMPTY-announce gateway under a
/// PERMISSIVE (AllowAll) posture STARTS the demand reconciler (so its GET-expiry
/// sweep can release runtime-granted topics). Observable via the background
/// thread advancing `reconcile_pass_count`. Flipping the
/// `start_when_empty` arg to `false` leaves an empty-announce gateway's reconciler
/// UNstarted → the count never advances. Anti-tautology twin: test 5
/// (`ingress_only_gateway_starts_no_reconciler`, empty deny-all → count stays 0).
#[test]
fn empty_allow_all_gateway_starts_reconciler_for_get_expiry() {
    let id = unique_id();
    let mut g_state: Option<GatewayRuntime> = None;
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let g = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("empty_all_rec_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    robot_identity: Some("empty-allow-all".to_string()),
                    ..Default::default()
                }),
                ..Default::default()
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .expect("init G (empty AllowAll)");
        // EMPTY announce + AllowAll = the generic `ros2 attach` shape.
        let plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![],
            ingress: vec![],
        };
        match GatewayRuntime::new(g, plan) {
            Ok(gateway) => {
                g_state = Some(gateway);
                break;
            }
            Err(e) => {
                eprintln!("attempt {attempt}: empty AllowAll session failed (port {port}): {e}")
            }
        }
    }
    let Some(gateway) = g_state else {
        panic!("could not establish the empty AllowAll gateway session in 3 attempts");
    };

    // The reconciler thread must advance the pass count (generous 10 s deadline). On
    // an empty-announce gateway this only happens because `start_when_empty` was
    // passed `true` (permissive) — flipping it to `false` leaves the count at 0.
    let deadline = Instant::now() + Duration::from_secs(10);
    while gateway.reconcile_pass_count() < 1 {
        assert!(
            Instant::now() < deadline,
            "the empty-AllowAll gateway's reconciler did not advance within 10s — it \
             must start for the GET-expiry sweep (got {})",
            gateway.reconcile_pass_count()
        );
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        gateway.reconcile_pass_count() >= 1,
        "an empty-AllowAll gateway starts the reconciler (GET-expiry sweep)"
    );
}

/// GET-expiry RELEASE of a probe-served topic: a probe-served
/// (runtime-granted) topic is RELEASED when demand ends — its last GET ages past
/// DEMAND_TTL AND liveliness is absent → the GET-expiry sweep disables the egress
/// flag, the next drive releases the tap, and the expiry is counted. Driven
/// deterministically via the `run_get_expiry_once(now)` sync seam at a future `now`
/// (crib the inversion test's TTL mechanics).
#[test]
fn probe_served_topic_released_on_get_expiry() {
    let probed_topic = format!("/probe/exp/probed/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![],
        ingress: vec![],
    };
    let (p, clock, mut gateway, _pubs) = make_producer_and_gateway("ppexp", plan, &[]);
    let mut probed_pub = p
        .create_publisher_simple(&probed_topic, MaxSliceLen::const_new(256))
        .expect("probed pub");

    // A demand via the QUERYABLE decision → probe HIT → granted (enable + keepalive
    // stamp), then drive → the tap attaches + forwards (the grant is real).
    assert!(
        gateway.demand_query_grants_for_test(&probed_topic),
        "the live unregistered AllowAll topic is probe-served + granted"
    );
    assert!(
        gateway
            .manager()
            .bridge_manager()
            .is_enabled(&probed_topic)
            .expect("enabled"),
        "the probe-served topic's egress flag is ON after the grant"
    );
    let _ = gateway.drive_once().expect("attach pass");
    assert!(
        gateway.active_tap_topics().contains(&probed_topic),
        "the probe-served topic's tap attaches on the demand pass"
    );
    publish_one(&mut probed_pub, &clock, 10, (1.0, 2.0, 3.0));
    assert_eq!(drive_until(&mut gateway, 1, &probed_topic), 1);

    // A same-time expiry sweep must NOT release (the grant is fresh).
    gateway
        .run_get_expiry_once(Instant::now())
        .expect("fresh expiry pass");
    assert!(
        gateway
            .manager()
            .bridge_manager()
            .is_enabled(&probed_topic)
            .expect("enabled"),
        "a fresh GET-granted probe topic must NOT expire"
    );
    assert_eq!(
        gateway.demand_expiry_count(),
        0,
        "no expiry while the grant is fresh"
    );

    // Advance `now` past DEMAND_TTL (the demander stopped GETting) AND liveliness is
    // absent (loopback, no demand token) → the topic is RELEASED.
    let future = Instant::now() + Duration::from_secs(8);
    gateway
        .run_get_expiry_once(future)
        .expect("aged expiry pass");
    assert!(
        !gateway
            .manager()
            .bridge_manager()
            .is_enabled(&probed_topic)
            .expect("enabled"),
        "the probe-served topic is DISABLED once its last GET aged past DEMAND_TTL and \
         liveliness is absent"
    );
    assert!(
        gateway.demand_expiry_count() >= 1,
        "the release is attributed to the GET-expiry sweep; got {}",
        gateway.demand_expiry_count()
    );

    // The next drive releases the tap (the flag is off).
    let _ = gateway.drive_once().expect("release pass");
    assert!(
        !gateway.active_tap_topics().contains(&probed_topic),
        "the tap is released once the egress flag went off"
    );
}

/// Boot TOCTOU close: a topic in the gateway's DECLARED-ingress set is
/// loop-refused by the probe EVEN before its ingress SHM service / dynamic-map entry
/// exists — the static set closes the boot window where the ingress service goes
/// live (at `create_ingress_publisher`) before the dynamic map is populated (at the
/// end of `register_ingress`). With an EMPTY declared set (the
/// anti-tautology control) the SAME live topic classifies `served`, so it is the
/// STATIC set doing the refusing, not the absence of a service. Driven via the test
/// seam that builds a fresh probe with an explicit declared set WITHOUT registering
/// any ingress bridge (the dynamic map stays empty for the topic).
#[test]
fn probe_declared_ingress_loop_refuses_before_dynamic_map() {
    // An EMPTY-ingress-plan gateway → the dynamic ingress map is EMPTY for the target
    // (register_ingress is never called for it).
    let target = format!("/probe/toctou/ingress/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![],
        ingress: vec![],
    };
    let (p, _clock, gateway, _pubs) = make_producer_and_gateway("pptoc", plan, &[]);
    // A LIVE SHM service for the target — without the static-set check this would be
    // a probe HIT (served).
    let _live_pub = p
        .create_publisher_simple(&target, MaxSliceLen::const_new(256))
        .expect("live pub");

    // WITH the static declared-ingress set (dynamic map still EMPTY for `target`) →
    // LOOP-REFUSED. This is the boot TOCTOU close: no ingress SHM service / dynamic
    // entry exists yet, and the topic is still refused.
    assert_eq!(
        gateway.probe_verdict_with_declared_ingress_for_test(
            &target,
            std::slice::from_ref(&target)
        ),
        "loop_refused",
        "a DECLARED-ingress topic is loop-refused before any ingress SHM service / dynamic-map entry"
    );

    // ANTI-TAUTOLOGY control: the SAME live topic with an EMPTY declared set (and the
    // still-empty dynamic map) is SERVED — proving the static set is what refuses.
    assert_eq!(
        gateway.probe_verdict_with_declared_ingress_for_test(&target, &[]),
        "served",
        "with no static set and an empty dynamic map, the same live topic is served (the \
         apparatus works — the static set is the load-bearing refuser)"
    );
}

// ===========================================================================
// The reconciler BELT FEED + discovery-truth e2e.
//
// The demand reconciler's ~1 s liveliness-query BELT re-affirms egress
// for the topics it iterates. A set FROZEN at the boot
// announce list would mean a topic REGISTERED AT RUNTIME (the control channel /
// the SHM probe) is never re-affirmed by the belt. The belt is therefore fed
// the LIVE registered set (`refresh_reconcile_topics`, monotonic-count
// dirty-checked). These pins: the dirty-check discipline, the belt-pass
// integration (deterministic + live), and the discovery-truth headline — a
// runtime/probe-served topic is visible to the `cerulion_ann` announce harvest a
// remote `topic list` queries.
// ===========================================================================

/// (belt-feed pure oracle) `refresh_reconcile_topics` grows the belt's iteration
/// set on a RUNTIME registration and leaves it byte-identical when the registered
/// count is unchanged — the monotonic-count dirty-check `drive_once` uses. Hand
/// oracle, no zenoh. Reverting the helper's live-set read leaves
/// `topics` frozen at `[a]` after `b` registers → the second assert fails.
#[test]
fn refresh_reconcile_topics_grows_on_runtime_registration_dirty_checked() {
    let bm = TopicBridgeManager::new();
    let a = "/c4/refresh/a".to_string();
    let b = "/c4/refresh/b".to_string();
    bm.register_topic(&a).unwrap();

    // Seed the belt set with the boot topic (canonical). len == registered count.
    let mut topics = vec![a.clone()];

    // Steady state: count(1) == len(1) → NO re-snapshot, byte-identical.
    refresh_reconcile_topics_for_test(&bm, &mut topics);
    assert_eq!(
        topics,
        vec![a.clone()],
        "an unchanged registered count leaves the belt set byte-identical (no spurious snapshot)"
    );

    // A RUNTIME registration grows the count → re-snapshot (canonical + sorted).
    bm.register_topic(&b).unwrap();
    refresh_reconcile_topics_for_test(&bm, &mut topics);
    assert_eq!(
        topics,
        vec![a.clone(), b.clone()],
        "a runtime registration is pulled into the belt's iteration set (sorted) — the belt feed"
    );

    // Idempotent re-register (no count growth) → unchanged, not re-ordered.
    bm.register_topic(&a).unwrap();
    refresh_reconcile_topics_for_test(&bm, &mut topics);
    assert_eq!(
        topics,
        vec![a, b],
        "an idempotent re-register does not grow or re-order the belt set"
    );
}

/// (belt-feed integration, deterministic) ONE belt pass on an EMPTY seed re-affirms
/// a RUNTIME-registered topic — refresh (picks it up) THEN apply (enables it) — the
/// exact two-line body the production thread runs. Hand oracle, no zenoh.
/// Reverting `refresh_reconcile_topics` leaves the empty seed frozen → the
/// apply iterates nothing → the flag never flips (enabled stays 0).
#[test]
fn belt_reaffirms_a_runtime_registered_topic_deterministic() {
    let bm = TopicBridgeManager::new();
    bm.set_egress_allow_all().unwrap(); // permissive: admits the runtime topic
    let runtime = "/c4/belt/runtime".to_string();
    // The `register_runtime_topic` effect: the flag is registered but starts OFF.
    let flag = bm.register_topic(&runtime).unwrap();
    assert!(
        !flag.load(Ordering::Relaxed),
        "registration does not flip the flag — demand does"
    );

    // Empty boot seed (a permissive empty-plan gateway).
    let mut topics: Vec<String> = vec![];
    let mut absence: HashMap<String, u32> = HashMap::new();
    let pass = AtomicU64::new(0);
    let enabled = AtomicU64::new(0);
    let demand: HashSet<String> = std::iter::once(runtime.clone()).collect();

    refresh_and_apply_demand_reconcile_for_test(
        &demand,
        &bm,
        &mut topics,
        &mut absence,
        &pass,
        &enabled,
    );

    assert!(
        flag.load(Ordering::Relaxed),
        "the belt re-affirms the RUNTIME-registered topic's egress (the belt feed) — \
         without the refresh the empty seed never enables it"
    );
    assert_eq!(
        enabled.load(Ordering::Relaxed),
        1,
        "the enable is attributed to this belt pass"
    );
    assert_eq!(
        pass.load(Ordering::Relaxed),
        1,
        "exactly one reconcile pass applied"
    );
    assert_eq!(
        topics,
        vec![runtime],
        "the belt's iteration set grew to include the runtime topic"
    );
}

/// A network-free producer P + a LISTENING gateway G on a
/// probed port (robot identity set), sharing ONE SHM root — the cross-session
/// discovery-truth harness. A remote observer/demander connects to the returned
/// port. Probe-then-bind with a 3× retry absorbs the probe→bind race. Returns
/// (P, clock, gateway, port).
fn make_listening_producer_and_gateway(
    tag: &str,
    plan: GatewayPlan,
    robot: &str,
) -> (
    Arc<TransportManager>,
    Arc<VirtualClock>,
    GatewayRuntime,
    u16,
) {
    let id = unique_id();
    let root = cerulion_core::testing::iceoryx_test_config();
    let clock = Arc::new(VirtualClock::new());
    let clock_dyn: Arc<dyn Clock> = clock.clone();
    // P: producer, LOCAL-ONLY (a graph/worker process is network-free).
    let p = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("gw_lp_{tag}_{id}"),
            clock: clock_dyn,
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init producer P");
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let g = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("gw_lg_{tag}_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    robot_identity: Some(robot.to_string()),
                    ..NetworkConfig::default()
                }),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("init gateway G");
        match GatewayRuntime::new(g, plan.clone()) {
            Ok(gateway) => return (p, clock, gateway, port),
            Err(e) => {
                eprintln!("attempt {attempt}: listening gateway boot failed (port {port}): {e}")
            }
        }
    }
    panic!("could not boot a listening gateway in 3 attempts");
}

/// (belt-feed integration, LIVE) the production sync seam `reconcile_demand_once`
/// re-affirms a RUNTIME-registered topic on an EMPTY-plan gateway with the
/// liveliness SUBSCRIBER suppressed — so ONLY the reconciler belt can flip the
/// flag. The belt's seed is empty (empty boot plan); the belt feed pulls
/// the runtime topic into the belt's iteration set each pass. Reverting
/// the refresh in `run_reconcile_pass_on_session` leaves the empty seed
/// frozen → the runtime topic is never re-affirmed → the flag never flips and the
/// 20 s deadline trips.
#[test]
fn belt_reaffirms_a_runtime_registered_topic_via_live_reconcile_seam() {
    let id = unique_id();
    let runtime_topic = format!("/c4/beltlive/{id}");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![], // EMPTY boot plan → empty belt seed
        ingress: vec![],
    };

    // G: listening gateway, subscriber SUPPRESSED (set BEFORE new() so the watch
    // subscriber is born suppressed). Build it manually to interleave the suppress.
    let root = cerulion_core::testing::iceoryx_test_config();
    let mut g_state: Option<(GatewayRuntime, u16)> = None;
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let g = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("c4bl_g_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    robot_identity: Some("c4beltlive".to_string()),
                    ..NetworkConfig::default()
                }),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("init G");
        g.network()
            .expect("G network")
            .set_suppress_live_demand_for_test(true);
        match GatewayRuntime::new(g, plan.clone()) {
            Ok(gateway) => {
                g_state = Some((gateway, port));
                break;
            }
            Err(e) => eprintln!("attempt {attempt}: G boot failed (port {port}): {e}"),
        }
    }
    let Some((mut gateway, port)) = g_state else {
        panic!("could not establish G's listening session in 3 attempts");
    };

    // Register the runtime topic (the gateway-core entry point the control channel
    // and the SHM probe both feed) — the flag is registered but starts OFF.
    assert!(
        gateway
            .register_runtime_topic(&runtime_topic)
            .expect("register runtime topic"),
        "the runtime topic is a fresh registration"
    );

    // B: distinct root, connects to G, declares the demand token (register_ingress).
    let b = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("c4bl_b_{id}"),
            network: Some(NetworkConfig {
                connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                ..Default::default()
            }),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init B");
    b.register_ingress_topic(
        &runtime_topic,
        Vector3::SCHEMA_HASH,
        MaxSliceLen::const_new(256),
    )
    .expect("B register_ingress_topic (declares the demand token)");

    // The runtime topic's flag handle (register_topic is idempotent).
    let flag = gateway
        .manager()
        .bridge_manager()
        .register_topic(&runtime_topic)
        .expect("G bridge flag handle");

    // Drive the reconciler synchronously until the flag flips. The subscriber is
    // suppressed, so ONLY the reconciler belt can flip it — and it can ONLY see the
    // runtime topic because the belt feed refreshes its iteration set.
    let deadline = Instant::now() + Duration::from_secs(20);
    while !flag.load(Ordering::Relaxed) {
        assert!(
            Instant::now() < deadline,
            "the belt did not re-affirm the RUNTIME topic within 20 s (subscriber suppressed) — \
             the belt feed did not pull it into the iteration set"
        );
        gateway.reconcile_demand_once().expect("reconcile pass");
        std::thread::sleep(Duration::from_millis(50));
    }

    assert!(
        flag.load(Ordering::Relaxed),
        "the belt must re-affirm the runtime topic's egress from a live demand token"
    );
    assert!(
        gateway.reconciler_enabled_count() >= 1,
        "the flip must be attributed to a reconciler-driven enable (the belt feed), not the \
         suppressed subscriber"
    );
}

/// Bounded remote-observer query of G's `cerulion_ann`
/// announce space (the SAME harvest a remote `topic list` runs) until `want`
/// appears attributed to `robot`. Asserts the reserved control channel NEVER
/// appears (the discovery-space inertness pin, checked every pass). Panics on the
/// deadline naming the last-observed entries.
fn await_announce_entry(session: &zenoh::Session, robot: &str, want: &str) {
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        let entries = cerulion_core::transport::discovery::query_announce_entries(
            session,
            Duration::from_millis(300),
        )
        .expect("announce-space liveliness query");
        // Inertness: the runtime-registration control service is NEVER announced.
        assert!(
            !entries
                .iter()
                .any(|(_, t)| t.as_deref() == Some(REG_CHANNEL_SERVICE_NAME)),
            "the reserved control channel must NEVER appear in the announce space: {entries:?}"
        );
        if entries
            .iter()
            .any(|(r, t)| r == robot && t.as_deref() == Some(want))
        {
            return;
        }
        assert!(
            Instant::now() < deadline,
            "'{want}' never appeared in the remote announce space (robot '{robot}'); \
             last observed {entries:?}"
        );
        std::thread::sleep(Duration::from_millis(100));
    }
}

/// (discovery-truth HEADLINE, channel-registered) a topic registered AT RUNTIME via
/// the REAL control channel — from a separate NETWORK-FREE manager — is
/// visible to the `cerulion_ann` announce HARVEST a remote `topic list` queries,
/// BEFORE any demand exists (discovery is truth independent of demand), then
/// demandable end-to-end. Announce-on-registration made discovery-true.
#[test]
fn discovery_truth_channel_registered_topic_is_announce_visible_pre_demand() {
    let id = unique_id();
    let runtime_topic = format!("/c4/disco/chan/{id}");
    const HASH: u64 = 0x0821_C400_0000_0001;
    let robot = "c4dchan";
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![], // empty boot plan — the raw route registers at runtime
        ingress: vec![],
    };
    let (p, clock, mut gateway, port) = make_listening_producer_and_gateway("c4dchan", plan, robot);

    // (1) A network-free worker P creates the raw-route SHM service + registers it
    //     over the REAL control channel (register_dynamic_egress_topic).
    let mut rt_pub = p
        .create_publisher_simple(&runtime_topic, MaxSliceLen::const_new(256))
        .expect("runtime-route publisher");
    assert!(
        p.register_dynamic_egress_topic(&runtime_topic, HASH)
            .expect("register runtime topic over the channel"),
        "the runtime topic is a fresh channel registration"
    );

    // (2) G drains the channel → register_runtime_topic → ANNOUNCE (settle).
    for _ in 0..3 {
        p.republish_dynamic_egress_for_test();
        gateway.drive_once().expect("drive");
    }
    assert!(
        gateway
            .is_runtime_registered(&runtime_topic)
            .expect("runtime reg"),
        "the channel topic registered on the empty-plan gateway"
    );
    // Source-of-truth cross-check: it carries a retained announce token.
    assert!(
        gateway
            .manager()
            .network()
            .expect("G network")
            .is_announced(&runtime_topic),
        "the runtime topic is announced (announce-on-registration)"
    );

    // (3) NO demand exists yet — a remote observer B (distinct root) connects to G
    //     and queries the ANNOUNCE space. The runtime topic is discoverable, the
    //     control channel is not.
    let b = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("c4dchan_b_{id}"),
            network: Some(NetworkConfig {
                connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                ..Default::default()
            }),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init B");
    let b_session = b
        .network()
        .expect("B network")
        .session()
        .expect("B session");
    await_announce_entry(b_session, robot, &runtime_topic);

    // (4) Then demandable end-to-end: a demand grant attaches the tap; a published
    //     frame forwards. Discoverable AND demandable.
    gateway
        .manager()
        .bridge_manager()
        .enable_bridge(&runtime_topic)
        .expect("enable (demand effect)");
    let _ = gateway.drive_once().expect("attach pass");
    publish_one(&mut rt_pub, &clock, 10, (1.0, 0.0, 0.0));
    assert_eq!(
        drive_until(&mut gateway, 1, &runtime_topic),
        1,
        "the channel-registered runtime topic forwards a frame on demand"
    );
}

/// (discovery-truth, probe-served variant) after a permissive SHM PROBE hit (a
/// demand for a topic LIVE in local SHM that no worker registered), the topic
/// appears in the `cerulion_ann` announce space — announce-on-first-serve made
/// discovery-true.
#[test]
fn discovery_truth_probe_served_topic_becomes_announce_visible() {
    let id = unique_id();
    let probe_topic = format!("/c4/disco/probe/{id}");
    let robot = "c4dprobe";
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![],
        ingress: vec![],
    };
    let (p, _clock, gateway, port) = make_listening_producer_and_gateway("c4dprobe", plan, robot);

    // A LIVE SHM service for a raw route NO worker registered over the channel.
    let _live_pub = p
        .create_publisher_simple(&probe_topic, MaxSliceLen::const_new(256))
        .expect("live raw-route publisher");
    assert!(
        !gateway
            .manager()
            .network()
            .expect("G network")
            .is_announced(&probe_topic),
        "the probe topic is NOT announced before any demand/probe"
    );

    // A demand triggers the AllowAll SHM probe (deterministic seam) → HIT →
    // register + announce on the spot (announce-on-first-serve).
    assert!(
        gateway.demand_query_grants_for_test(&probe_topic),
        "a live unregistered topic is probe-served + granted"
    );
    assert_eq!(gateway.probe_hit_count(), 1, "exactly one probe hit");
    assert!(
        gateway
            .manager()
            .network()
            .expect("G network")
            .is_announced(&probe_topic),
        "the probe-served topic is announced on the spot (announce-on-first-serve)"
    );

    // The probe-served topic is now visible to the remote announce harvest.
    let b = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("c4dprobe_b_{id}"),
            network: Some(NetworkConfig {
                connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                ..Default::default()
            }),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init B");
    let b_session = b
        .network()
        .expect("B network")
        .session()
        .expect("B session");
    await_announce_entry(b_session, robot, &probe_topic);
}

/// Boot a LISTENING producer P (network-free) + gateway G booted via
/// [`GatewayRuntime::new_with_schema_serving`] (so the catalog/schema surface
/// serves `serving`). Mirrors `make_listening_producer_and_gateway` but threads
/// the schema serving through the gateway.
fn make_serving_producer_and_gateway(
    tag: &str,
    plan: GatewayPlan,
    robot: &str,
    serving: cerulion_core::SchemaServing,
) -> (
    Arc<TransportManager>,
    Arc<VirtualClock>,
    GatewayRuntime,
    u16,
) {
    let id = unique_id();
    let root = cerulion_core::testing::iceoryx_test_config();
    let clock = Arc::new(VirtualClock::new());
    let clock_dyn: Arc<dyn Clock> = clock.clone();
    let p = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("gw_sp_{tag}_{id}"),
            clock: clock_dyn,
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init producer P");
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let g = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("gw_sg_{tag}_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    robot_identity: Some(robot.to_string()),
                    ..NetworkConfig::default()
                }),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("init gateway G");
        match GatewayRuntime::new_with_schema_serving(g, plan.clone(), serving.clone()) {
            Ok(gateway) => return (p, clock, gateway, port),
            Err(e) => {
                eprintln!("attempt {attempt}: serving gateway boot failed (port {port}): {e}")
            }
        }
    }
    panic!("could not boot a serving gateway in 3 attempts");
}

/// Schema-serving e2e: a gateway booted via
/// `new_with_schema_serving` serves the schema-serving surface, AND a topic
/// registered AT RUNTIME over the reg-channel (a `ros2 attach` raw route — hash
/// only, NO name) is NAMED in the catalog by resolving ITS reg-channel hash
/// through the served hash→name reverse map. The desk then FETCHES the
/// named type over the `schema` verb (the serving is queryable
/// post-construction). Real gateway + reg-channel + a remote desk over a 127.0.0.1
/// TCP hop.
///
/// Reverting the `.or_else(hash reverse-map)` fold leaves
/// `/rt/widget` catalogued with `schema_name: None` and fails the catalog-name
/// assertion.
#[test]
fn serving_gateway_names_runtime_route_via_hash_and_serves_schema() {
    let id = unique_id();
    let runtime_topic = format!("/rt/widget/{id}");
    let widget_text = "float64 x\nfloat64 y\n";
    let widget_q = "probe_msgs/Widget";
    // The wire hash the raw-route producer advertises == the reverse-map key.
    let widget_hash =
        cerulion_core::codegen::parse_rosmsg(widget_text, "Widget", Some("probe_msgs"))
            .expect("parse Widget")
            .schema_hash();
    let robot = "hashserv";

    // The serving: the custom Widget doc + the hash→name reverse binding, with NO
    // explicit topic→name binding (so the catalog MUST use the hash reverse map).
    let serving = cerulion_core::SchemaServing {
        topic_schemas: vec![],
        schema_docs: vec![cerulion_core::SchemaDoc {
            qualified: widget_q.to_string(),
            encoding: cerulion_core::SchemaEncoding::Msg,
            text: widget_text.to_string(),
            deps: vec![],
        }],
        schema_hashes: vec![cerulion_core::SchemaHashName {
            schema_hash: widget_hash,
            qualified: widget_q.to_string(),
        }],
    };
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![], // empty boot plan — the raw route registers at runtime
        ingress: vec![],
    };
    let (p, _clock, mut gateway, port) =
        make_serving_producer_and_gateway("hashserv", plan, robot, serving);

    // A network-free worker registers the raw route over the reg-channel, carrying
    // ONLY (topic, schema_hash) — no name (the ros2-attach shape).
    let _rt_pub = p
        .create_publisher_simple(&runtime_topic, MaxSliceLen::const_new(256))
        .expect("runtime-route publisher");
    assert!(
        p.register_dynamic_egress_topic(&runtime_topic, widget_hash)
            .expect("register runtime topic"),
        "the runtime topic is a fresh channel registration"
    );
    // Drain the reg-channel into the gateway.
    for _ in 0..5 {
        p.republish_dynamic_egress_for_test();
        gateway.drive_once().expect("drive");
        if gateway
            .is_runtime_registered(&runtime_topic)
            .expect("reg check")
        {
            break;
        }
    }
    assert!(
        gateway
            .is_runtime_registered(&runtime_topic)
            .expect("reg check"),
        "the raw route registered on the gateway"
    );

    // A remote desk B (distinct-nothing — a plain scouting-off session) connects.
    let desk = cerulion_core::transport::network::NetworkManager::new(NetworkConfig {
        connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
        ..NetworkConfig::default()
    });
    let session = desk.session().expect("desk session");

    // The catalog NAMES the runtime raw route via the hash reverse map.
    let mut named = false;
    for _ in 0..40 {
        if let Some(catalog) = cerulion_core::transport::discovery::query_robot_catalog(
            session,
            robot,
            Duration::from_millis(300),
        ) {
            if let Some(entry) = catalog.entries.iter().find(|e| e.topic == runtime_topic) {
                assert_eq!(
                    entry.schema_hash,
                    Some(widget_hash),
                    "the catalog carries the reg-channel hash"
                );
                if entry.schema_name.as_deref() == Some(widget_q) {
                    named = true;
                    break;
                }
            }
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        named,
        "the runtime raw route was never NAMED in the catalog via the hash reverse map \
         (a hash-only reg-channel topic must resolve its name)"
    );

    // The served schema is queryable post-construction — the desk
    // fetches the named custom type and gets its verbatim doc.
    let mut served = false;
    for _ in 0..40 {
        if let Some(reply) = cerulion_core::transport::discovery::query_robot_schema(
            session,
            robot,
            widget_q,
            Duration::from_millis(300),
        ) {
            if !reply.docs.is_empty() {
                assert_eq!(reply.docs[0].qualified, widget_q);
                assert_eq!(reply.docs[0].text, widget_text);
                served = true;
                break;
            }
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        served,
        "the schema-serving surface never served the custom type post-construction"
    );
}

/// The additive `GatewayRuntime::new_embedded` constructor boots a
/// desk-embeddable gateway on an already-init'd, networked, identity-carrying
/// SHARED manager — an EMPTY announce set under the permissive `AllowAll` posture,
/// so a topic pushed at RUNTIME over the reg-channel
/// (`register_dynamic_egress_topic`) is picked up and announced with no restart
/// (the exact shape `cerulion-netd`'s egress plane uses). Proves the embedded boot
/// is functionally a runtime-registration gateway: empty announce at boot, a
/// runtime registration lands + announces. (The full producer→zenoh→consumer e2e
/// over this constructor is `cerulion_netd`'s `egress_plane_iox2_test`.)
#[test]
fn new_embedded_boots_empty_permissive_and_accepts_a_runtime_egress_registration() {
    let id = unique_id();
    let root = cerulion_core::testing::iceoryx_test_config();
    let runtime_topic = format!("/c5a/embed/{id}");
    const HASH: u64 = 0x0837_C5A0_0000_0001;

    // A network-free producer P + the desk daemon's SHARED networked manager G on
    // the SAME root (P is the graph/worker; G is netd's one session).
    let p = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("c5a_embed_p_{id}"),
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init producer P");
    let g = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("c5a_embed_g_{id}"),
            network: Some(NetworkConfig {
                robot_identity: Some("c5aembed".to_string()),
                ..NetworkConfig::default()
            }),
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init embedded gateway G");

    // The additive constructor: empty announce, permissive posture.
    let mut gateway = GatewayRuntime::new_embedded(g, cerulion_core::SchemaServing::default())
        .expect("embedded boot");
    assert!(
        gateway
            .runtime_registered_topics()
            .expect("runtime set")
            .is_empty(),
        "an embedded gateway boots with ZERO announced/registered topics"
    );

    // A desk graph produces a topic + pushes it over the reg-channel (the seam
    // netd's egress plane drives). G's runtime-registration reader picks it up.
    let _rt_pub = p
        .create_publisher_simple(&runtime_topic, MaxSliceLen::const_new(256))
        .expect("runtime egress publisher");
    assert!(
        p.register_dynamic_egress_topic(&runtime_topic, HASH)
            .expect("register runtime egress topic"),
        "a fresh runtime egress registration"
    );
    for _ in 0..3 {
        p.republish_dynamic_egress_for_test();
        gateway.drive_once().expect("drive");
    }
    assert!(
        gateway
            .is_runtime_registered(&runtime_topic)
            .expect("runtime reg"),
        "the runtime egress topic registered on the EMBEDDED empty-plan gateway"
    );
    assert!(
        gateway
            .manager()
            .network()
            .expect("G network")
            .is_announced(&runtime_topic),
        "the runtime egress topic is announced (announce-on-registration)"
    );
}

/// The AllowAll choice in `new_embedded` is LOAD-BEARING
/// — a permissive (`AllowAll`) empty-boot gateway STARTS the demand surface (its
/// reconciler thread runs), while a NON-permissive (`AllowList`) empty-boot gateway
/// starts NOTHING. The anti-tautology control proving netd's embedded
/// gateway must be `AllowAll` for runtime egress to ever become demandable: an empty
/// `AllowList` boot would silently never egress a runtime-registered topic. Observed
/// via `reconcile_pass_count()` — the reconciler thread bumps it ~1/s ONLY when the
/// demand surface started.
#[test]
fn embedded_allow_all_starts_the_demand_surface_but_empty_allowlist_does_not() {
    let id = unique_id();
    let root = cerulion_core::testing::iceoryx_test_config();
    let mk = |tag: &str| {
        TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("c5a_4b_{tag}_{id}"),
                network: Some(NetworkConfig {
                    robot_identity: Some(format!("c5a4b{tag}")),
                    ..NetworkConfig::default()
                }),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("init")
    };

    // AllowAll (the netd embedding) — the demand surface (reconciler) STARTS.
    let allow = GatewayRuntime::new_embedded(mk("allow"), cerulion_core::SchemaServing::default())
        .expect("embedded AllowAll boot");
    // Empty NON-permissive AllowList — the control: NO demand surface.
    let deny = GatewayRuntime::new(
        mk("deny"),
        GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowList(vec![]),
            announce: vec![],
            ingress: vec![],
        },
    )
    .expect("empty AllowList boot");

    // The AllowAll reconciler bumps its pass count within a couple of intervals
    // (~1/s); poll up to a bound.
    let deadline = Instant::now() + Duration::from_secs(6);
    while allow.reconcile_pass_count() == 0 && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        allow.reconcile_pass_count() > 0,
        "AllowAll (permissive) starts the demand surface — its reconciler runs"
    );
    // The empty-AllowList control started NO reconciler, so its count stays 0 over the
    // SAME window (checked after the AllowAll one already passed).
    assert_eq!(
        deny.reconcile_pass_count(),
        0,
        "an empty AllowList boot starts NO demand surface (the AllowAll choice matters)"
    );
}

// ---------------------------------------------------------------------------
// The LAN (zenoh) demand-authorization gate. The account/pairing
// grant is the access boundary; the DemandAuthorizer
// seam sits on the LAN plane, with a deny-nothing AllowAll default.
// These pins use `demand_query_grants_for_test` / `liveliness_demand_grants_for_test`
// (the EXACT queryable + liveliness demand decisions the real callbacks run), so they
// are zenoh-timing-free. A STUB deny-authorizer stands in for the real
// `is_allowed(account)` predicate; hand oracles throughout (never a self-compare).
// ---------------------------------------------------------------------------

/// A hand-oracle authorizer that DENIES exactly one topic on the LAN plane (and
/// asserts the subject IS the LAN variant with no identity yet, which is all the
/// LAN plane supplies) — the stand-in for the account authorizer.
struct DenyOneLanTopic(String);
impl DemandAuthorizer for DenyOneLanTopic {
    fn authorize_demand(&self, subject: &DemandSubject, topic: &str) -> DemandDecision {
        // The LAN plane's subject carries no authenticated identity yet.
        assert!(
            matches!(subject, DemandSubject::Lan { locator: None }),
            "the LAN demand-grant must pass a Lan{{locator:None}} subject, got {subject:?}"
        );
        if topic == self.0 {
            DemandDecision::deny(format!("test: {topic} not authorized"))
        } else {
            DemandDecision::Allow
        }
    }
}

/// HEADLINE: with a deny-authorizer installed, a demand for the DENIED topic is
/// refused on BOTH the queryable AND the liveliness demand paths — it serves NOTHING
/// (not probe-registered, not announced, no egress flag), while a SIBLING topic is
/// still admitted (the gate is selective, not a blanket deny). Then the default
/// AllowAll re-install admits a previously-un-served topic (the deny was load-bearing).
#[test]
#[traced_test]
fn c5c_demand_authorizer_refuses_denied_topic_on_both_lan_paths() {
    let denied = format!("/egress/denied/{}", unique_id());
    let allowed = format!("/egress/allowed/{}", unique_id());
    let live_after = format!("/egress/afterreset/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![],
        ingress: vec![],
    };
    let (p, _clock, gateway, _pubs) = make_producer_and_gateway("c5c", plan, &[]);
    // Live SHM producers so a MISS is excluded; a refusal is the authorizer's doing.
    let _denied_pub = p
        .create_publisher_simple(&denied, MaxSliceLen::const_new(256))
        .expect("denied producer");
    let _allowed_pub = p
        .create_publisher_simple(&allowed, MaxSliceLen::const_new(256))
        .expect("allowed producer");
    let _after_pub = p
        .create_publisher_simple(&live_after, MaxSliceLen::const_new(256))
        .expect("after producer");

    // Install the deny-authorizer (the account gate stands in here).
    gateway.set_demand_authorizer(Arc::new(DenyOneLanTopic(denied.clone())));

    // The DENIED topic is refused on the QUERYABLE path — serves nothing.
    assert!(
        !gateway.demand_query_grants_for_test(&denied),
        "the demand-authorization gate must REFUSE the denied topic (queryable path)"
    );
    assert!(
        !gateway.is_runtime_registered(&denied).expect("reg check"),
        "a refused demand registers NOTHING (no probe-serve past the gate)"
    );
    assert!(
        !gateway
            .manager()
            .network()
            .expect("network")
            .announced_topics()
            .contains(&denied),
        "a refused demand announces NOTHING"
    );
    // And on the LIVELINESS path (same gate).
    assert!(
        !gateway.liveliness_demand_grants_for_test(&denied),
        "the demand-authorization gate must REFUSE the denied topic (liveliness path)"
    );

    // A SIBLING topic is still ADMITTED under the SAME authorizer — selective, not a
    // blanket deny (byte-identical to the default admit path).
    assert!(
        gateway.demand_query_grants_for_test(&allowed),
        "an untargeted topic is still granted under the deny-authorizer"
    );
    assert!(gateway
        .is_runtime_registered(&allowed)
        .expect("allowed reg"));

    // The refusal is LOUD (Principle #3 / the loud-over-silent rule) — a WARN,
    // level token included.
    logs_assert(|lines: &[&str]| {
        logged_at(
            lines,
            "WARN",
            "demand REFUSED by the demand-authorization gate",
        )
    });

    // ANTI-TAUTOLOGY: re-install the deny-nothing default → a fresh live topic admits,
    // proving the deny was the authorizer's doing (not a MISS).
    gateway.set_demand_authorizer(Arc::new(AllowAllAuthorizer));
    assert!(
        gateway.demand_query_grants_for_test(&live_after),
        "AllowAll (the default) admits — the deny was the authorizer's, not a MISS"
    );
}

/// A SPY authorizer that ADMITS everything (AllowAll semantics) but records that it
/// was CONSULTED with the expected `Lan{locator:None}` subject + the exact canonical
/// topic — the anti-inert-shipping oracle proving the LAN grant path routes through
/// the installed authorizer. Asserts the subject inline (a wrong subject panics).
struct SpyLanAuthorizer {
    /// Bumped ONLY on a consultation whose topic == `expected_topic` (a hand oracle:
    /// the LAN grant path passes the exact demanded topic, not a placeholder).
    hits_for_topic: Arc<AtomicU64>,
    expected_topic: String,
}
impl DemandAuthorizer for SpyLanAuthorizer {
    fn authorize_demand(&self, subject: &DemandSubject, topic: &str) -> DemandDecision {
        assert!(
            matches!(subject, DemandSubject::Lan { locator: None }),
            "the LAN grant path must pass a Lan{{locator:None}} subject, got {subject:?}"
        );
        if topic == self.expected_topic {
            self.hits_for_topic.fetch_add(1, Ordering::Relaxed);
        }
        DemandDecision::Allow
    }
}

/// The DEFAULT gate is deny-nothing AND the seam is genuinely wired: a fresh
/// gateway (no authorizer installed) not only GRANTS a live topic but the grant has
/// an EFFECT — it probe-REGISTERS + ANNOUNCES the topic (a served mirror, not just a
/// `true` return). Then a SPY authorizer (AllowAll semantics + a call-count) proves
/// the LAN grant path actually CONSULTS the installed authorizer with the exact
/// `Lan{locator:None}` subject + canonical topic. Hand oracles throughout (never a
/// self-compare); the anti-inert-shipping pin that the deny-nothing default is a real
/// gate, not a dropped no-op.
#[test]
fn c5c_default_authorizer_grants_with_effect_and_is_consulted() {
    let live = format!("/egress/default/{}", unique_id());
    let live2 = format!("/egress/default2/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![],
        ingress: vec![],
    };
    let (p, _clock, gateway, _pubs) = make_producer_and_gateway("c5cdef", plan, &[]);
    let _pub = p
        .create_publisher_simple(&live, MaxSliceLen::const_new(256))
        .expect("live producer");
    let _pub2 = p
        .create_publisher_simple(&live2, MaxSliceLen::const_new(256))
        .expect("live producer 2");

    // --- DEFAULT authorizer (no install): admits AND the grant has EFFECT. ---
    assert!(
        gateway.demand_query_grants_for_test(&live),
        "the default deny-nothing authorizer grants a live topic"
    );
    assert!(gateway.liveliness_demand_grants_for_test(&live));
    // The grant's EFFECT — a served mirror, not just a `true`: the probe REGISTERED
    // the live topic and ANNOUNCED it (the exact positive of the deny test's negatives).
    assert!(
        gateway.is_runtime_registered(&live).expect("reg check"),
        "the default grant probe-REGISTERED the live topic (a real served mirror)"
    );
    assert!(
        gateway
            .manager()
            .network()
            .expect("network")
            .announced_topics()
            .contains(&live),
        "the default grant ANNOUNCED the granted topic"
    );

    // --- CONSULTATION: a spy (AllowAll semantics) proves the LAN grant path routes
    //     through the installed authorizer with the exact subject + topic. ---
    let hits = Arc::new(AtomicU64::new(0));
    gateway.set_demand_authorizer(Arc::new(SpyLanAuthorizer {
        hits_for_topic: Arc::clone(&hits),
        expected_topic: live2.clone(),
    }));
    assert!(
        gateway.demand_query_grants_for_test(&live2),
        "the spy admits (AllowAll semantics) — byte-identical to the default"
    );
    assert!(
        hits.load(Ordering::Relaxed) >= 1,
        "the LAN grant path must CONSULT the installed authorizer with the exact topic"
    );
    // The spy-admitted grant ALSO has effect (registered) — the seam did not swallow it.
    assert!(
        gateway.is_runtime_registered(&live2).expect("reg check 2"),
        "the spy-admitted grant is served (registered), not swallowed by the seam"
    );
}

/// A hand-oracle authorizer that DENIES EVERYTHING on the LAN plane (and asserts
/// the subject is the LAN plane's `Lan{locator:None}`) — the stand-in for
/// the account gate refusing this desk's account. Blanket deny models "no access
/// to this machine at all", covering the demand AND the discovery (catalog/schema)
/// verbs uniformly.
struct DenyAllLan;
impl DemandAuthorizer for DenyAllLan {
    fn authorize_demand(&self, subject: &DemandSubject, topic: &str) -> DemandDecision {
        assert!(
            matches!(subject, DemandSubject::Lan { locator: None }),
            "the LAN query surface must pass a Lan{{locator:None}} subject, got {subject:?}"
        );
        DemandDecision::deny(format!("test: no access to {topic}"))
    }
}

/// The demand-authorization gate covers the
/// WHOLE query surface — a denied authorizer refuses the `catalog` AND `schema`
/// verbs (the discovery plane), not just `demand`, so an unauthorized party
/// cannot ENUMERATE the robot's topic catalog or fetch its `.msg`/YAML schemas.
/// Hand oracles throughout (never a self-compare): under AllowAll the catalog lists
/// a live topic and the schema serves its served doc; under a deny-all authorizer
/// BOTH return an EXPLICIT refusal (empty payload + `error`), never a silent empty
/// reply. The AllowAll re-install (anti-tautology) proves the deny was load-bearing.
#[test]
#[traced_test]
fn c5c_demand_authorizer_refuses_catalog_and_schema_enumeration() {
    let robot = "c5cenum";
    let id = unique_id();
    let topic = format!("/egress/enum/{id}");
    let served_q = "probe_msgs/Probe";
    // A gateway that SERVES a schema doc (so the AllowAll schema path returns a real
    // `found` closure, and the deny path visibly withholds it).
    let serving = cerulion_core::SchemaServing {
        topic_schemas: vec![],
        schema_docs: vec![cerulion_core::SchemaDoc {
            qualified: served_q.to_string(),
            encoding: cerulion_core::SchemaEncoding::Msg,
            text: "float64 v\n".to_string(),
            deps: vec![],
        }],
        schema_hashes: vec![],
    };
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![],
        ingress: vec![],
    };
    let (p, _clock, gateway, _port) =
        make_serving_producer_and_gateway("c5cenum", plan, robot, serving);
    // A live SHM producer + an AllowAll demand registers the topic, so the catalog
    // has a REAL entry to (later) withhold.
    let _live_pub = p
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("live producer");
    assert!(
        gateway.demand_query_grants_for_test(&topic),
        "AllowAll registers the live topic (so the catalog can list it)"
    );

    // --- BASELINE (AllowAll default): catalog + schema BOTH serve. ---
    let cat_allowed = gateway.catalog_serve_for_test(robot);
    assert!(
        cat_allowed.error.is_none(),
        "an AllowAll catalog is not a refusal"
    );
    assert!(
        cat_allowed.entries.iter().any(|e| e.topic == topic),
        "an AllowAll catalog lists the live topic (hand oracle): {:?}",
        cat_allowed.entries
    );
    let schema_allowed = gateway.schema_serve_for_test(robot, served_q);
    assert!(
        schema_allowed.error.is_none(),
        "an AllowAll schema is not a refusal"
    );
    assert_eq!(
        schema_allowed.docs.first().map(|d| d.qualified.as_str()),
        Some(served_q),
        "an AllowAll schema serves the requested doc closure"
    );

    // --- DENY: a deny-all authorizer refuses BOTH discovery verbs. ---
    gateway.set_demand_authorizer(Arc::new(DenyAllLan));
    let cat_denied = gateway.catalog_serve_for_test(robot);
    assert!(
        cat_denied.error.is_some(),
        "a denied catalog is an EXPLICIT refusal (not a silent empty reply)"
    );
    assert!(
        cat_denied.entries.is_empty(),
        "a denied catalog enumerates NOTHING (the live topic is withheld): {:?}",
        cat_denied.entries
    );
    let schema_denied = gateway.schema_serve_for_test(robot, served_q);
    assert!(
        schema_denied.error.is_some(),
        "a denied schema is an EXPLICIT refusal"
    );
    assert!(
        schema_denied.docs.is_empty(),
        "a denied schema serves NO `.msg`/YAML text"
    );
    // The refusals are LOUD (Principle #3 / the loud-over-silent rule) — WARNs,
    // level token included.
    logs_assert(|lines: &[&str]| {
        logged_at(
            lines,
            "WARN",
            "catalog GET REFUSED by the demand-authorization gate",
        )?;
        logged_at(
            lines,
            "WARN",
            "schema GET REFUSED by the demand-authorization gate",
        )
    });

    // --- ANTI-TAUTOLOGY: restore AllowAll → both serve again (the deny was the
    // authorizer's doing, not an empty gateway). ---
    gateway.set_demand_authorizer(Arc::new(AllowAllAuthorizer));
    let cat_restored = gateway.catalog_serve_for_test(robot);
    assert!(cat_restored.error.is_none());
    assert!(cat_restored.entries.iter().any(|e| e.topic == topic));
    let schema_restored = gateway.schema_serve_for_test(robot, served_q);
    assert!(schema_restored.error.is_none());
    assert!(!schema_restored.docs.is_empty());
}

/// The robot-side catalog serve stamps each entry's LIVE producer count —
/// the desk sidebar's per-row liveness affordance. An attached-robot scenario over real
/// iceoryx2 (hand oracle, never a self-compare): two boot-announced topics, ONE with
/// a live SHM producer (a 20 Hz topic) and ONE with NO producer (a registered-but-
/// DEAD DDS route, the `/uslam/cloud_map` case). The served catalog must carry
/// `producer_count == Some(1)` for the live topic and `Some(0)` for the dead route,
/// so the desk can dim + label the dead one "no data yet" while the live one renders
/// normally. Probes the gateway's manager over the SHARED SHM root, so the count is
/// GROUND TRUTH from iceoryx2's dynamic config, not a fabricated value.
#[test]
fn catalog_stamps_producer_count_live_vs_dead_route() {
    let id = unique_id();
    let topic_live = format!("/pc/live/{id}");
    let topic_dead = format!("/pc/dead/{id}"); // a boot announce with NO producer
    let robot = "pcgw";
    // Announce BOTH topics (boot-registers each into the catalog regardless of a
    // producer); create a live producer for ONLY the live one.
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic_live.clone(), topic_dead.clone()],
        ingress: vec![],
    };
    let (_p, _clock, gateway, _pubs) =
        make_producer_and_gateway("pc", plan, std::slice::from_ref(&topic_live));

    let catalog = gateway.catalog_serve_for_test(robot);
    assert!(
        catalog.error.is_none(),
        "an AllowAll catalog is not a refusal"
    );

    let live = catalog
        .entries
        .iter()
        .find(|e| e.topic == topic_live)
        .expect("catalog lists the live topic");
    let dead = catalog
        .entries
        .iter()
        .find(|e| e.topic == topic_dead)
        .expect("catalog lists the dead route (a boot announce)");

    // Hand oracle: the live topic has exactly one producer (P's publisher, seen over
    // the shared SHM root); the dead route has zero — the distinction that matters.
    assert_eq!(
        live.producer_count,
        Some(1),
        "a live 20 Hz topic reports its one producer: {live:?}"
    );
    assert_eq!(
        dead.producer_count,
        Some(0),
        "a registered-but-dead DDS route reports NO producer (the /uslam/cloud_map \
         case the sidebar dims + labels 'no data yet'): {dead:?}"
    );
    // Boot provenance on both (they came from the announce plan) — a sanity cross-check
    // that the dead route is genuinely catalogued, not merely absent.
    assert_eq!(dead.provenance, cerulion_core::CatalogProvenance::Boot);
}

// ===========================================================================
// The catalog serves DATA-FLOW liveness, not just publisher presence.
// ===========================================================================

/// Producer + gateway on ONE shared SHM root and ONE shared `VirtualClock`, so the
/// gateway's liveness observer ages frames against a clock this test sets. (The
/// shared [`make_producer_and_gateway`] gives the gateway a REAL clock, which cannot
/// reach the settle threshold without a wall-clock sleep.)
fn make_clocked_producer_and_gateway(
    tag: &str,
    plan: GatewayPlan,
    produced: &[String],
) -> (
    Arc<TransportManager>,
    Arc<VirtualClock>,
    GatewayRuntime,
    Vec<cerulion_core::CerulionPublisher>,
) {
    let id = unique_id();
    let root = cerulion_core::testing::iceoryx_test_config();
    let clock = Arc::new(VirtualClock::new());
    clock.set(1_000_000_000);
    let p = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("gwlv_p_{tag}_{id}"),
            clock: clock.clone() as Arc<dyn Clock>,
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init producer P");
    let publishers: Vec<_> = produced
        .iter()
        .map(|t| {
            p.create_publisher_simple(t, MaxSliceLen::const_new(256))
                .expect("P publisher")
        })
        .collect();
    let g = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("gwlv_g_{tag}_{id}"),
            clock: clock.clone() as Arc<dyn Clock>,
            network: Some(NetworkConfig {
                robot_identity: Some("gw".to_string()),
                ..NetworkConfig::default()
            }),
            ..Default::default()
        },
        root,
    )
    .expect("init gateway G");
    let gateway = GatewayRuntime::new(g, plan).expect("gateway boot");
    (p, clock, gateway, publishers)
}

/// Drive the gateway once at virtual time `now` with the liveness sweep forced —
/// production self-throttles it to a 200 ms cadence; a test that owns the clock
/// drives it explicitly.
fn drive_with_sweep(gateway: &mut GatewayRuntime, clock: &VirtualClock, now: u64) {
    clock.set(now);
    gateway.force_liveness_sweep_for_test();
    gateway.drive_once().expect("drive");
}

/// THE PRODUCTION PATH: the served catalog distinguishes a topic whose
/// publisher exists and STREAMS from one whose publisher exists and NEVER PUBLISHES.
///
/// This is the `ros2 attach` shape, and it is the one `producer_count` cannot see:
/// on a `cerulion ros2 attach` robot the bridge graph creates a Cerulion publisher
/// for EVERY discovered DDS topic at graph-BUILD time, so BOTH topics here have a
/// live publisher and BOTH report `producer_count: Some(1)` — measured on a real
/// robot as all 75 topics reading `1`. (The test above uses a dead route
/// with NO publisher at all, which is the easy case.) This test asserts that
/// blindness EXPLICITLY, then asserts that the data-flow observation separates the
/// two rows, through the REAL `catalog` serve decision the zenoh callback runs.
#[test]
fn catalog_liveness_separates_a_streaming_topic_from_a_silent_publisher() {
    let id = unique_id();
    let streaming = format!("/lv/streaming/{id}");
    let silent = format!("/lv/silent/{id}");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![streaming.clone(), silent.clone()],
        ingress: vec![],
    };
    // BOTH topics get a real publisher: the `ros2 attach` shape.
    let produced = vec![streaming.clone(), silent.clone()];
    let (_p, clock, mut gateway, mut pubs) =
        make_clocked_producer_and_gateway("lv", plan, &produced);
    let t0 = clock.now_ns();
    const MS: u64 = 1_000_000;

    assert!(
        gateway.liveness_enabled(),
        "observation must be on by default"
    );
    // Pass 1: the observer attaches a tap per announced topic.
    drive_with_sweep(&mut gateway, &clock, t0);
    assert_eq!(
        gateway.liveness_tap_count(),
        2,
        "one observation tap per announced topic (neither is demanded)"
    );

    // Only ONE topic publishes — three frames, each observed 10 ms later. Every
    // one is stamped after its tap attached, so each dates the topic (the
    // retained-history gate discriminates by stamp — see the `liveness` module
    // docs' retained-history section).
    for step in 1..=3u64 {
        let at = t0 + step * 100 * MS;
        publish_one(&mut pubs[0], &clock, at, (step as f64, 0.0, 0.0));
        drive_with_sweep(&mut gateway, &clock, at + 10 * MS);
    }
    // Run virtual time PAST the no-data threshold with the silent topic silent
    // throughout, then let the live one publish once more so the two rows differ
    // in the way the sidebar actually renders: streaming vs dead.
    let last_pub = t0 + (LIVENESS_NO_DATA_MIN_MS + 1_000) * MS;
    publish_one(&mut pubs[0], &clock, last_pub, (4.0, 0.0, 0.0));
    let last_seen = last_pub + 10 * MS;
    drive_with_sweep(&mut gateway, &clock, last_seen);
    let settled = last_seen + 500 * MS;
    drive_with_sweep(&mut gateway, &clock, settled);

    let catalog = gateway.catalog_serve_for_test("gw");
    assert!(catalog.error.is_none());
    let find = |t: &str| {
        catalog
            .entries
            .iter()
            .find(|e| e.topic == t)
            .unwrap_or_else(|| panic!("catalog lists {t}"))
    };
    let live = find(&streaming);
    let dead = find(&silent);

    // (a) THE DEFECT, asserted: the `producer_count` signal reads the SAME on both rows.
    assert_eq!(
        live.producer_count, dead.producer_count,
        "both topics have a registered publisher, so `producer_count` CANNOT \
         distinguish them — this is exactly why data-flow liveness exists: {live:?} vs {dead:?}"
    );
    assert_eq!(live.producer_count, Some(1));

    // (b) The data-flow observation does distinguish them, through the real serve.
    let live_l = live.liveness.expect("the streaming topic carries liveness");
    let dead_l = dead.liveness.expect("the silent topic carries liveness");
    assert_eq!(
        live_l.frames_observed, 4,
        "exactly the four published frames were observed (hand oracle)"
    );
    assert_eq!(
        live_l.last_frame_age_ms,
        Some((settled - last_seen) / MS),
        "the age is the robot's own clock delta since it drained the last frame"
    );
    assert_eq!(
        live_l.state(),
        LivenessState::Streaming,
        "0.5 s since the last frame is inside the 5 s recency window"
    );
    assert_eq!(
        dead_l.frames_observed, 0,
        "the silent publisher produced nothing"
    );
    assert_eq!(dead_l.last_frame_age_ms, None);
    assert!(
        dead_l.observed_for_ms >= LIVENESS_NO_DATA_MIN_MS,
        "precondition: the silent topic really is past the no-data threshold ({} ms)",
        dead_l.observed_for_ms
    );
    assert_eq!(
        dead_l.state(),
        LivenessState::NoData,
        "THE HEADLINE: a registered-but-silent publisher reads `no_data` — the row \
         the sidebar dims — while its producer_count says Some(1): {dead:?}"
    );
    assert_ne!(
        live_l.state(),
        dead_l.state(),
        "the two rows MUST classify differently — that is the whole point"
    );
}

/// The RETIRE adjudication pin — registration is PRESENCE-BASED and
/// the desk-visible surface stays accurate WITHOUT a withdraw primitive. A topic
/// registered at runtime over the control channel (the rmw / dds_bridge shape)
/// whose producer is then DESTROYED (a) stays registered + announced — nothing
/// withdraws it, the documented model in `reg_channel`'s module docs and the
/// `rmw_destroy_publisher` note — and (b) serves a catalog row that tells the
/// truth about the dead route: `producer_count` probes live ports and reads
/// `Some(0)`, and the liveness annotation ages the row out of
/// `streaming` (`Idle`: produced once, no longer fresh) — so a desk dims it
/// rather than rendering a phantom live stream. The Streaming assertion while
/// the producer is alive is the anti-vacuity half: the same row genuinely read
/// as a live stream before the destroy.
///
/// Deleting the runtime-topic liveness hook
/// (`self.liveness.track(canonical)` in
/// `reconcile_egress_topics`) fails this arm at the tap-count precondition —
/// without it a runtime-registered topic is never observed at all and the
/// accuracy half of the model is gone (`liveness: None` on every runtime row).
#[test]
fn a_destroyed_producers_topic_stays_cataloged_with_honest_liveness() {
    let id = unique_id();
    let topic = format!("/dp/gone/{id}");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![],
        ingress: vec![],
    };
    let (p, clock, mut gateway, mut pubs) =
        make_clocked_producer_and_gateway("dp", plan, std::slice::from_ref(&topic));
    let t0 = clock.now_ns();
    const MS: u64 = 1_000_000;
    const HASH: u64 = 0xC154_4C15_44C1_544C;

    // The rmw shape: the producing (network-free) process registers its topic
    // over the control channel; the gateway drains, registers + announces it.
    assert!(
        p.register_dynamic_egress_topic(&topic, HASH)
            .expect("register"),
        "a fresh registration returns Ok(true)"
    );
    for _ in 0..3 {
        p.republish_dynamic_egress_for_test();
        gateway.drive_once().expect("drive");
    }
    assert!(
        gateway.is_runtime_registered(&topic).expect("registered"),
        "the control-channel registration reached the gateway"
    );
    // The runtime topic joined the liveness observer (reconcile_egress_topics'
    // `liveness.track`); the forced sweep attaches its observation tap.
    drive_with_sweep(&mut gateway, &clock, t0 + 50 * MS);
    assert_eq!(
        gateway.liveness_tap_count(),
        1,
        "a runtime-registered topic gets an observation tap like a boot topic"
    );

    // The producer publishes; the observation dates the topic (the first batch
    // is the attach baseline — banked, undated; the later stamps advance).
    let mut last_seen = 0u64;
    for step in 1..=3u64 {
        let at = t0 + step * 100 * MS;
        publish_one(&mut pubs[0], &clock, at, (step as f64, 0.0, 0.0));
        last_seen = at + 10 * MS;
        drive_with_sweep(&mut gateway, &clock, last_seen);
    }

    // Anti-vacuity: while the producer LIVES the served row reads as a live
    // stream — Streaming, one live producer, the advertised reg-channel hash.
    let alive = gateway.catalog_serve_for_test("gw");
    assert!(alive.error.is_none());
    let row = alive
        .entries
        .iter()
        .find(|e| e.topic == topic)
        .expect("the runtime topic is catalogued while alive");
    assert_eq!(
        row.schema_hash,
        Some(HASH),
        "the reg-channel hash is served"
    );
    assert_eq!(row.producer_count, Some(1), "one live producer while alive");
    let live_l = row.liveness.expect("the runtime row carries liveness");
    assert_eq!(
        live_l.frames_observed, 3,
        "exactly the three published frames were observed (hand oracle)"
    );
    assert_eq!(
        live_l.state(),
        LivenessState::Streaming,
        "a producing runtime topic reads streaming: {row:?}"
    );

    // DESTROY the producer — the transport effect of `rmw_destroy_publisher`.
    drop(pubs.remove(0));

    // Age virtual time past the streaming recency window (the sweep drains
    // nothing; the observer's tap holds the data service open, so the live
    // producer probe still answers).
    let aged = last_seen + (LIVENESS_STREAMING_RECENCY_MS + 1_000) * MS;
    drive_with_sweep(&mut gateway, &clock, aged);

    // (a) PRESENCE: nothing withdrew the registration or the announce.
    assert!(
        gateway
            .is_runtime_registered(&topic)
            .expect("still registered"),
        "no withdraw exists — the registration outlives the producer"
    );
    assert!(
        gateway
            .manager()
            .network()
            .expect("network")
            .announced_topics()
            .contains(&topic),
        "the topic stays announced after its producer is destroyed"
    );

    // (b) ACCURACY: the served row says "dead route", never "live stream".
    let after = gateway.catalog_serve_for_test("gw");
    assert!(after.error.is_none());
    let row = after
        .entries
        .iter()
        .find(|e| e.topic == topic)
        .expect("the destroyed producer's topic is STILL catalogued (the model)");
    assert_eq!(
        row.producer_count,
        Some(0),
        "the live-port probe reads ZERO producers once the publisher dropped"
    );
    let dead_l = row.liveness.expect("liveness is still served for the row");
    assert_eq!(
        dead_l.frames_observed, 3,
        "the observation history is retained"
    );
    assert_eq!(
        dead_l.last_frame_age_ms,
        Some(LIVENESS_STREAMING_RECENCY_MS + 1_000),
        "the age is the exact observer-clock delta since the last drained frame"
    );
    assert_eq!(
        dead_l.state(),
        LivenessState::Idle,
        "produced-then-destroyed ages to Idle — never a phantom stream, and \
         never the dimmed no_data (frames were observed): {row:?}"
    );
}

/// Liveness cost contract, through the production drive loop: a DEMANDED topic's
/// liveness is fed by the EGRESS tap, so the gateway never holds two subscriber
/// ports for one topic — and the observation survives the hand-off in both
/// directions.
#[test]
fn demanded_topic_liveness_rides_the_egress_tap_with_no_extra_port() {
    let id = unique_id();
    let topic = format!("/lv/demand/{id}");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic.clone()],
        ingress: vec![],
    };
    let (_p, clock, mut gateway, mut pubs) =
        make_clocked_producer_and_gateway("lvd", plan, std::slice::from_ref(&topic));
    let t0 = clock.now_ns();
    const MS: u64 = 1_000_000;

    // Undemanded: the OBSERVER owns the one tap.
    drive_with_sweep(&mut gateway, &clock, t0);
    assert_eq!(gateway.liveness_tap_count(), 1);
    assert!(gateway.active_tap_topics().is_empty(), "no egress tap yet");

    // Demand arrives → the egress tap takes over and the observer releases its own.
    gateway
        .manager()
        .bridge_manager()
        .enable_bridge(&topic)
        .expect("enable");
    drive_with_sweep(&mut gateway, &clock, t0 + 100 * MS);
    assert_eq!(gateway.active_tap_topics(), vec![topic.clone()]);
    assert_eq!(
        gateway.liveness_tap_count(),
        0,
        "ONE port per topic: the observer must release its tap to the egress tap"
    );

    // Frames the egress tap drains ARE the observation — no second drain, no second
    // port, and the liveness still moves. The egress tap is a freshly-attached
    // port, so its first yield COULD be the publisher's flushed backlog and the
    // gateway cannot tell: that batch is the advancement BASELINE, so it
    // is banked without dating — and the row is already `Idle` (produced), never
    // the dimmed `no_data`.
    let pub_at = t0 + 200 * MS;
    publish_one(&mut pubs[0], &clock, pub_at, (1.0, 2.0, 3.0));
    assert_eq!(drive_until(&mut gateway, 1, &topic), 1, "forwarded");
    let l = gateway.topic_liveness(&topic).expect("observing");
    assert_eq!(
        l.last_frame_age_ms, None,
        "the drainer's first batch is its baseline — the demand plane applies the \
         SAME rule as the observer's own tap"
    );
    assert_eq!(l.frames_observed, 1, "banked as observed");
    assert_eq!(
        l.state(),
        LivenessState::Idle,
        "produced ⇒ Idle, never the dimmed row: {l:?}"
    );
    let pub_at = pub_at + 100 * MS;
    publish_one(&mut pubs[0], &clock, pub_at, (4.0, 5.0, 6.0));
    // `drive_until`'s want is CUMULATIVE (it reads `forwarded_count`), so the
    // second frame is want = 2.
    assert_eq!(drive_until(&mut gateway, 2, &topic), 1, "forwarded");
    clock.set(pub_at + 10 * MS);
    let l = gateway
        .topic_liveness(&topic)
        .expect("the egress tap fed the observation");
    assert_eq!(
        l.frames_observed, 2,
        "the demanded topic's liveness comes free from the drain that already happens"
    );
    assert_eq!(
        l.last_frame_age_ms,
        Some(10),
        "the SECOND batch advances past the baseline stamp, so it dates the topic \
         — drained at `pub_at`, read 10 ms later (hand oracle)"
    );
    assert_eq!(l.state(), LivenessState::Streaming);
    assert_eq!(
        gateway.liveness_tap_count(),
        0,
        "still no second port while demand lasts"
    );

    // Demand ends → the observer takes the topic back on its next sweep.
    gateway
        .manager()
        .bridge_manager()
        .disable_bridge(&topic)
        .expect("disable");
    drive_with_sweep(&mut gateway, &clock, pub_at + 300 * MS);
    assert!(
        gateway.active_tap_topics().is_empty(),
        "egress tap detached"
    );
    assert_eq!(
        gateway.liveness_tap_count(),
        1,
        "the observer re-attached — observation is continuous across the hand-off"
    );
    assert_eq!(
        gateway
            .topic_liveness(&topic)
            .expect("still observed")
            .frames_observed,
        2,
        "history survives the hand-off"
    );
}

/// When a DEMANDED topic's egress tap DIES mid-drain, the observation
/// riding it must end too — the robot must not keep serving a verdict nothing is
/// watching.
///
/// The egress tap IS the topic's observer while demand lasts (that is the
/// one-port-per-topic contract). A drain failure — canonically a producer service
/// torn down and re-created, which a stale tap can never see — makes `drive_once`
/// drop the tap. If the observation interval were left OPEN across that, the
/// topic would keep accruing `observed_for_ms` with nothing attached, harden into
/// `no_data` (or serve an ever-staler `idle` age), and the sidebar would render a
/// confident verdict derived from a dead tap.
///
/// Delete the `set_externally_observed(topic, false)` on the
/// drain-failure arm of `drive_once` and the UNKNOWN assertion below fails — the
/// topic keeps reporting.
#[test]
fn a_dead_egress_tap_ends_the_observation_it_was_feeding() {
    let id = unique_id();
    let topic = format!("/lv/deadtap/{id}");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic.clone()],
        ingress: vec![],
    };
    let (_p, clock, mut gateway, mut pubs) =
        make_clocked_producer_and_gateway("lvdt", plan, std::slice::from_ref(&topic));
    let t0 = clock.now_ns();
    const MS: u64 = 1_000_000;

    // Demand arrives; the egress tap takes over the observation and feeds it.
    // The whole stimulus stays inside ONE sweep interval so the observer's own
    // sweep cannot re-attach behind the assertions and mask the contract.
    gateway
        .manager()
        .bridge_manager()
        .enable_bridge(&topic)
        .expect("enable");
    drive_with_sweep(&mut gateway, &clock, t0 + 10 * MS);
    assert_eq!(gateway.active_tap_topics(), vec![topic.clone()]);
    for i in 0..2u64 {
        let at = t0 + (20 + i * 10) * MS;
        publish_one(&mut pubs[0], &clock, at, (i as f64, 0.0, 0.0));
        // `drive_until`'s want is CUMULATIVE (it reads `forwarded_count`).
        assert_eq!(drive_until(&mut gateway, i + 1, &topic), 1, "forwarded");
    }
    assert_eq!(
        gateway
            .topic_liveness(&topic)
            .expect("observing through the egress tap")
            .state(),
        LivenessState::Streaming,
        "precondition: a real observation is riding the egress tap"
    );

    // The tap dies mid-drain (the production stimulus, injected — a real
    // teardown/recreate race is not deterministically stageable).
    assert!(
        gateway.fail_next_egress_drain_for_test(&topic),
        "precondition: there was an egress tap to poison"
    );
    clock.set(t0 + 40 * MS);
    gateway.drive_once().expect("drive survives a tap death");
    assert!(
        gateway.active_tap_topics().is_empty(),
        "the wedged tap is dropped so the next pass can re-attach a fresh one"
    );
    assert_eq!(
        gateway.topic_liveness(&topic),
        None,
        "THE PIN: nothing is draining this topic now, so its liveness is UNKNOWN — \
         not a frozen `streaming` inherited from a tap that no longer exists"
    );

    // ANTI-TAUTOLOGY: the next pass re-attaches and observation resumes with its
    // banked history, so this is a pause, not a permanent loss.
    clock.set(t0 + 50 * MS);
    gateway.drive_once().expect("drive");
    assert_eq!(gateway.active_tap_topics(), vec![topic.clone()]);
    let resumed = gateway
        .topic_liveness(&topic)
        .expect("observation resumed on the fresh tap");
    assert_eq!(
        resumed.frames_observed, 2,
        "the banked history came back with the re-attach"
    );
}

/// Regression guard: the liveness
/// observer must NEVER cost a topic its ability to EGRESS.
///
/// Both the observer's tap and the egress tap are subscriber ports on the same
/// topic, and a topic provisions only `INTROSPECTION_SUBSCRIBER_HEADROOM` spare
/// slots — shared with `bagd`, `topic echo` and vizd. This test drives the topic to
/// ZERO free slots while the observer holds one, then demands it. Because
/// `drive_once` releases the observer's tap BEFORE attempting the egress attach,
/// egress gets the freed slot. Reverse that order and the egress attach fails on a
/// full topic, `continue`s before the hand-off, and the observer keeps the slot
/// FOREVER — permanent egress starvation for that topic.
#[test]
fn liveness_tap_yields_its_slot_so_egress_never_starves() {
    let id = unique_id();
    let topic = format!("/lv/slots/{id}");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic.clone()],
        ingress: vec![],
    };
    let (p, clock, mut gateway, _pubs) =
        make_clocked_producer_and_gateway("lvs", plan, std::slice::from_ref(&topic));
    let t0 = clock.now_ns();

    // The observer takes one subscriber slot.
    drive_with_sweep(&mut gateway, &clock, t0);
    assert_eq!(gateway.liveness_tap_count(), 1);

    // Consume EVERY remaining subscriber slot with foreign taps (the bagd /
    // `topic echo` / vizd contenders), so the topic is completely full.
    let mut hogs = Vec::new();
    while let Ok(sub) = p.create_data_only_subscriber(&topic) {
        hogs.push(sub);
    }
    assert!(
        !hogs.is_empty(),
        "precondition: the topic had spare introspection slots to fill"
    );
    assert!(
        p.create_data_only_subscriber(&topic).is_err(),
        "precondition: ZERO free subscriber slots remain"
    );

    // Demand arrives on a completely full topic.
    gateway
        .manager()
        .bridge_manager()
        .enable_bridge(&topic)
        .expect("enable");
    drive_with_sweep(&mut gateway, &clock, t0 + 100_000_000);

    assert_eq!(
        gateway.active_tap_topics(),
        vec![topic.clone()],
        "EGRESS MUST WIN THE SLOT: the observer releases its tap before the egress \
         attach, so a topic at capacity still egresses. Reverse the order and this \
         fails — and would keep failing forever, since a failed attach `continue`s \
         before the hand-off"
    );
    assert_eq!(
        gateway.liveness_tap_count(),
        0,
        "and the observer holds nothing — one port per topic"
    );
    assert_eq!(
        gateway.attach_failure_count(&topic),
        0,
        "the egress attach never even failed once"
    );
    drop(hogs);
}

/// The split-flush hole through the production egress
/// path: a retained-history flush deeper than one drive pass's drain budget
/// arrives in TWO batches, and the later chunk's stamps EXCEED the earlier
/// chunk's. It must NOT date the topic.
///
/// The gateway drains ONE `drain_owned(budget)` per drive pass with no
/// drain-to-empty loop (unlike the observer's own tap), so this split is the
/// gateway's normal behaviour rather than a race. Judged naively, the second chunk looks
/// like "the publisher produced something new since the first" and a route that has
/// been dead for hours renders `Streaming` the moment a remote demands it — the
/// false-live hazard, arriving through the demand plane.
///
/// The defence is `DrainObservation::queue_emptied`: the baseline stays OPEN until
/// a pass comes back short, so every chunk of one flush is absorbed into it.
///
/// Hardcode `queue_emptied: true` at the `note_frames` call site in
/// `GatewayRuntime::drive_once` and the undated assertion below fails.
#[test]
fn a_split_retained_history_flush_does_not_date_a_dead_route_over_egress() {
    let id = unique_id();
    let topic = format!("/lv/split/{id}");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic.clone()],
        ingress: vec![],
    };
    // No helper-created publisher: this topic needs a RETAINED HISTORY.
    let (p, clock, mut gateway, _pubs) = make_clocked_producer_and_gateway("lvsp", plan, &[]);
    let t0 = clock.now_ns();
    const MS: u64 = 1_000_000;
    const HISTORY: usize = 8;

    let mut publisher = p
        .create_publisher(&topic, MaxSliceLen::const_new(256), HISTORY)
        .expect("publisher with retained history");

    // The DEAD route's backlog: published long before anything watched, and the
    // publisher never sends again.
    for i in 0..HISTORY {
        publish_one(
            &mut publisher,
            &clock,
            t0 + i as u64 * MS,
            (i as f64, 0.0, 0.0),
        );
    }

    // Demand arrives: the egress tap attaches and takes over the observation.
    let demand_at = t0 + 1_000 * MS;
    clock.set(demand_at);
    gateway
        .manager()
        .bridge_manager()
        .enable_bridge(&topic)
        .expect("enable");
    gateway.drive_once().expect("drive");
    assert_eq!(gateway.active_tap_topics(), vec![topic.clone()]);

    // Nudge the publisher into flushing its retained history into that fresh
    // connection (a listener-full subscriber connects — bagd / `topic echo` /
    // vizd would do exactly this).
    let _late_joiner = p
        .create_subscriber(&topic)
        .expect("a listener-full subscriber connects");
    clock.set(demand_at + 10 * MS);
    publisher.pump_history();

    // Drain it over SEVERAL passes and record how the observation moved after
    // each, so the split is measured rather than assumed.
    let mut banked = Vec::new();
    for pass in 0..6u64 {
        clock.set(demand_at + (20 + pass * 10) * MS);
        gateway.drive_once().expect("drive");
        let l = gateway.topic_liveness(&topic).expect("observing");
        assert_eq!(
            l.last_frame_age_ms, None,
            "pass {pass}: a chunk of the SAME flush must never date a dead route: {l:?}"
        );
        banked.push(l.frames_observed);
    }
    let observed = *banked.last().expect("passes ran");
    assert!(
        observed > 0,
        "precondition: the flush really reached the tap"
    );

    // The SPLIT precondition: the flush arrived across more than one pass, so
    // the second chunk really did have to be judged against the first.
    let passes_with_new_frames = banked
        .iter()
        .scan(0u64, |prev, &n| {
            let grew = n > *prev;
            *prev = n;
            Some(grew)
        })
        .filter(|&grew| grew)
        .count();
    assert!(
        passes_with_new_frames >= 2,
        "precondition: the {HISTORY}-frame flush must span >= 2 drive passes for \
         this test to be probative (banked per pass: {banked:?}). If iceoryx2's \
         borrow budget grows past the history depth this stops being a split and \
         the assertion above degenerates into the plain baseline rule"
    );

    // Settled: produced, undatable, Idle — never dimmed, never fresh.
    clock.set(demand_at + (LIVENESS_NO_DATA_MIN_MS + 5_000) * MS);
    gateway.drive_once().expect("drive");
    let l = gateway.topic_liveness(&topic).expect("observing");
    assert_eq!(l.last_frame_age_ms, None);
    assert_eq!(
        l.state(),
        LivenessState::Idle,
        "the dead route's flushed backlog buys it no freshness, and its banked \
         frames keep it out of the dimmed row: {l:?}"
    );
}

/// The egress drain's `queue_emptied` must be reachable even when
/// a topic's borrow budget is 1 — where "drained fewer frames than I asked for"
/// is arithmetically impossible at a site that already knows a frame arrived.
///
/// `queue_emptied` is the ONLY thing that closes the advancement baseline, so a
/// drain that can never report it leaves the demanded topic permanently undatable
/// — `Idle` with no age while it is visibly streaming to the desk. The gateway
/// takes ONE borrow budget per drive pass by design, and
/// `budget = tap.max_borrowed_samples().max(1)`, so a service created with
/// `subscriber_max_borrowed_samples = 1` (or the `.max(1)` floor) makes
/// `drained < budget` unreachable: a pass that drains anything drains exactly 1.
///
/// The drain OBSERVES emptiness — after a saturated read the tap's port is asked,
/// via the NON-CONSUMING `DataOnlySubscriber::has_samples()` — so this topic dates
/// exactly like any other, and the data plane is untouched (a consuming probe
/// would answer the same question but would also change what each pass forwards).
///
/// Restoring
/// `queue_emptied: drain_err.is_none() && drained < budget as u64` fails the final
/// assertion here with `last_frame_age_ms: None` and `Idle`.
#[test]
fn a_budget_one_egress_drain_can_still_close_its_baseline_and_date() {
    let id = unique_id();
    let topic = format!("/lv/budget1/{id}");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic.clone()],
        ingress: vec![],
    };
    // No helper-created publisher: this topic's SERVICE must be created with a
    // borrow budget of 1, which only an explicit topic config can do.
    let (p, clock, mut gateway, _pubs) = make_clocked_producer_and_gateway("lvb1", plan, &[]);
    let t0 = clock.now_ns();
    const MS: u64 = 1_000_000;

    let mut cfg = p.default_topic_config();
    // `External` provisioning passes the borrow value through verbatim; the
    // owned-topic arms would raise it to the borrow floor of 3.
    cfg.publisher_provisioning = PublisherProvisioning::External;
    cfg.subscriber_max_borrowed_samples = Some(1);
    let mut publisher = p
        .create_publisher_with_topic_config(&topic, MaxSliceLen::const_new(256), 0, cfg)
        .expect("publisher on a borrow-budget-1 service");

    // PRECONDITION, measured: the budget the egress drain will compute really is
    // 1 — the regime where a `drained < budget` inference is unreachable. (The probe is
    // dropped immediately so it never competes for a subscriber slot.)
    {
        let probe = p
            .create_data_only_subscriber(&topic)
            .expect("probe the service's borrow budget");
        assert_eq!(
            probe.max_borrowed_samples().max(1),
            1,
            "precondition: this test is only probative on a budget-1 topic"
        );
    }

    // Demand arrives: the egress tap attaches and takes over the observation.
    let demand_at = t0 + 1_000 * MS;
    clock.set(demand_at);
    gateway
        .manager()
        .bridge_manager()
        .enable_bridge(&topic)
        .expect("enable");
    gateway.drive_once().expect("drive");
    assert_eq!(gateway.active_tap_topics(), vec![topic.clone()]);

    // Batch 1: ONE frame — a read that exactly FILLS the budget and empties the
    // queue. It is the connection's advancement BASELINE (banked, undated), and
    // the pass MUST recognise the queue as empty or nothing later can ever date.
    publish_one(&mut publisher, &clock, demand_at + 10 * MS, (1.0, 0.0, 0.0));
    clock.set(demand_at + 20 * MS);
    gateway.drive_once().expect("drive");
    let l = gateway.topic_liveness(&topic).expect("observing");
    assert_eq!(
        l.frames_observed, 1,
        "precondition: the frame reached the tap"
    );
    assert_eq!(
        l.last_frame_age_ms, None,
        "the first observed batch is the baseline, dated by nothing: {l:?}"
    );

    // Batch 2: a strictly later stamp — the advancement that dates the topic.
    publish_one(&mut publisher, &clock, demand_at + 30 * MS, (2.0, 0.0, 0.0));
    clock.set(demand_at + 40 * MS);
    gateway.drive_once().expect("drive");
    let l = gateway.topic_liveness(&topic).expect("observing");
    assert_eq!(
        l.last_frame_age_ms,
        Some(0),
        "THE PIN — PRE-FIX THIS IS `None` FOREVER: at budget 1 the baseline could \
         never close, so a streaming demanded topic rendered `Idle` with no age \
         for the whole life of the demand: {l:?}"
    );
    assert_eq!(l.state(), LivenessState::Streaming);
}

/// The general case behind the budget-1 hole: a drive pass whose
/// read exactly FILLS its borrow budget proves nothing about the remainder, so
/// emptiness must be OBSERVED (a probe) rather than inferred from
/// `drained < budget`. Otherwise a topic whose backlog happens to land on the
/// budget boundary silently loses its chance to close the baseline.
///
/// Two arms, both against hand oracles, and they guard OPPOSITE mistakes:
///
/// * Arm 1 (discriminating for the observed-emptiness rule): an exactly-full read that did
///   empty the queue closes the baseline, so the next advancing frame dates the
///   topic. Restoring `drained < budget` fails it — that pass reports
///   `queue_emptied: false`, the batch is still absorbed as baseline, and the
///   topic reads `None`.
/// * Arm 2 (the regression guard on that rule): a genuinely over-budget backlog
///   must NOT be closed early — `has_samples()` answering `true` proves the queue
///   is not empty. Replacing that verdict with an
///   unconditional `queue_emptied = true` closes the baseline a pass early, the
///   next chunk of the SAME burst advances, and the age assertion after the
///   drain-the-rest pass reads `Some(0)` instead of the hand oracle. (The
///   pre-existing `a_split_retained_history_flush_..._over_egress` test also
///   catches this, and the two are complementary.) The arm's tail also shows a
///   saturating topic RECOVERING rather than going permanently deaf; that half is
///   behavioural, not discriminating, since the final short read reports `true`
///   under either rule.
#[test]
fn an_exactly_full_egress_drain_closes_its_baseline_so_the_topic_dates() {
    let id = unique_id();
    let topic = format!("/lv/full/{id}");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic.clone()],
        ingress: vec![],
    };
    let produced = vec![topic.clone()];
    let (p, clock, mut gateway, mut pubs) =
        make_clocked_producer_and_gateway("lvfull", plan, &produced);
    let t0 = clock.now_ns();
    const MS: u64 = 1_000_000;

    // The budget the egress drain will compute, read from the live service rather
    // than assumed (iceoryx2's default is 2; a config change must not silently
    // make this test non-probative).
    let budget = {
        let probe = p
            .create_data_only_subscriber(&topic)
            .expect("probe the service's borrow budget");
        probe.max_borrowed_samples().max(1)
    };
    assert!(budget >= 1);

    let demand_at = t0 + 1_000 * MS;
    clock.set(demand_at);
    gateway
        .manager()
        .bridge_manager()
        .enable_bridge(&topic)
        .expect("enable");
    gateway.drive_once().expect("drive");
    assert_eq!(gateway.active_tap_topics(), vec![topic.clone()]);

    // ARM 1: EXACTLY `budget` frames — a saturated read that nonetheless left the
    // queue empty. That is the baseline batch, and the pass must close it.
    for i in 0..budget as u64 {
        publish_one(
            &mut pubs[0],
            &clock,
            demand_at + (10 + i) * MS,
            (i as f64, 0.0, 0.0),
        );
    }
    clock.set(demand_at + 100 * MS);
    gateway.drive_once().expect("drive");
    let l = gateway.topic_liveness(&topic).expect("observing");
    assert_eq!(
        l.frames_observed, budget as u64,
        "precondition: the read really did fill the budget exactly: {l:?}"
    );
    assert_eq!(l.last_frame_age_ms, None, "still the baseline: {l:?}");

    // One advancing frame later the topic is dated — which is only possible if
    // the exactly-full read above was recognised as having emptied the queue.
    publish_one(&mut pubs[0], &clock, demand_at + 200 * MS, (9.0, 0.0, 0.0));
    clock.set(demand_at + 210 * MS);
    gateway.drive_once().expect("drive");
    let l = gateway.topic_liveness(&topic).expect("observing");
    assert_eq!(
        l.last_frame_age_ms,
        Some(0),
        "PRE-FIX the exactly-full read reported `queue_emptied: false`, so this \
         batch was still absorbed as baseline and the topic read `None`: {l:?}"
    );
    assert_eq!(l.state(), LivenessState::Streaming);

    // ARM 2: a genuinely OVER-budget backlog must NOT be mistaken for an empty
    // queue by the probe — the split-flush defence has to survive the emptiness probe. Drop
    // the demand and re-establish it so a FRESH baseline is opened, then stage a
    // backlog deeper than one pass can drain.
    gateway
        .manager()
        .bridge_manager()
        .disable_bridge(&topic)
        .expect("disable");
    clock.set(demand_at + 300 * MS);
    gateway.drive_once().expect("drive");
    gateway
        .manager()
        .bridge_manager()
        .enable_bridge(&topic)
        .expect("re-enable");
    clock.set(demand_at + 310 * MS);
    gateway.drive_once().expect("drive");

    let banked_before = gateway
        .topic_liveness(&topic)
        .expect("observing")
        .frames_observed;
    let backlog = budget as u64 * 3;
    for i in 0..backlog {
        publish_one(
            &mut pubs[0],
            &clock,
            demand_at + (400 + i) * MS,
            (100.0 + i as f64, 0.0, 0.0),
        );
    }
    // The first pass CANNOT have emptied the queue, so it must not close the
    // baseline — a later chunk of one burst must never out-rank an earlier one,
    // and the probe must not mistake a saturated read for an empty queue.
    clock.set(demand_at + 500 * MS);
    gateway.drive_once().expect("drive");
    let l = gateway.topic_liveness(&topic).expect("observing");
    assert!(
        l.frames_observed >= banked_before + budget as u64,
        "precondition: the backlog reached the tap and this pass really did fill \
         its budget (banked before: {banked_before}): {l:?}"
    );
    assert!(
        l.frames_observed < banked_before + backlog,
        "precondition: the backlog is genuinely deeper than one pass can drain, \
         so the split-flush defence is under test here: {l:?}"
    );
    // The age is ARM 1's arrival, still growing — NOT refreshed by this chunk.
    assert_eq!(
        l.last_frame_age_ms,
        Some(290),
        "a chunk of an un-drained burst must not REFRESH the age (hand oracle: \
         the last dating was at demand+210 ms and it is now demand+500 ms): {l:?}"
    );

    // The pass that drains the REST of the burst must not date it either. This is
    // the assertion that kills a probe which claims emptiness unconditionally
    // after a saturated read (`queue_emptied = true` in the
    // probe arm closes the baseline one pass early, so this chunk advances and
    // the age reads `Some(0)` here).
    clock.set(demand_at + 600 * MS);
    gateway.drive_once().expect("drive");
    let l = gateway.topic_liveness(&topic).expect("observing");
    assert_eq!(
        l.last_frame_age_ms,
        Some(390),
        "a later chunk of ONE burst must never out-rank an earlier one — the age \
         is still ARM 1's arrival at demand+210 ms, now demand+600 ms: {l:?}"
    );

    // RECOVERY: drain anything left, then publish twice — the first closes the
    // burst's baseline (the batch that CLOSES it never dates, by design), the
    // second advances past it and dates. A saturating topic is not permanently
    // deaf.
    for pass in 0..6u64 {
        clock.set(demand_at + (610 + pass * 10) * MS);
        gateway.drive_once().expect("drive");
    }
    publish_one(&mut pubs[0], &clock, demand_at + 800 * MS, (7.0, 0.0, 0.0));
    clock.set(demand_at + 810 * MS);
    gateway.drive_once().expect("drive");
    publish_one(&mut pubs[0], &clock, demand_at + 900 * MS, (8.0, 0.0, 0.0));
    clock.set(demand_at + 910 * MS);
    gateway.drive_once().expect("drive");
    let l = gateway.topic_liveness(&topic).expect("observing");
    assert_eq!(
        l.last_frame_age_ms,
        Some(0),
        "a saturating topic dates once it recovers — the baseline closed on a \
         drain that genuinely emptied the queue: {l:?}"
    );
    assert_eq!(l.state(), LivenessState::Streaming);
}

/// A DEMANDED topic's rate estimate rides the EGRESS drain, so the
/// topics a desk is actively streaming are not the only ones without a frequency.
///
/// The demand plane is where this could silently go inert. `sweep()` skips an
/// externally-observed topic entirely, so a demanded topic's whole observation —
/// the rate included — comes from `note_frames` on the gateway's own drive loop.
/// If `drain_egress_tap` failed to read the wire SEQUENCE the way it already
/// reads the stamp, every checked topic would fall to the labelled FLOOR (or to
/// no rate at all) while unchecked ones measured exactly: the inverse of the
/// original complaint, and just as wrong.
///
/// Real `loan_proxy` publishes, so the sequences are the production publisher's
/// own commit counter (gap-free) — nothing here hand-stamps a header.
/// Hand oracle: ten commits across a two-second window is 5 Hz.
#[test]
fn a_demanded_topics_rate_rides_the_egress_drain() {
    let id = unique_id();
    let topic = format!("/rate/demand/{id}");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic.clone()],
        ingress: vec![],
    };
    let (_p, clock, mut gateway, mut pubs) =
        make_clocked_producer_and_gateway("rated", plan, std::slice::from_ref(&topic));
    let t0 = clock.now_ns();
    const MS: u64 = 1_000_000;

    // Demand arrives: the egress tap takes over and becomes the observer.
    drive_with_sweep(&mut gateway, &clock, t0);
    gateway
        .manager()
        .bridge_manager()
        .enable_bridge(&topic)
        .expect("enable");
    drive_with_sweep(&mut gateway, &clock, t0 + 100 * MS);
    assert_eq!(gateway.active_tap_topics(), vec![topic.clone()]);
    assert_eq!(
        gateway.liveness_tap_count(),
        0,
        "precondition: the observer released its own tap, so EVERY observation \
         below — the rate included — comes from the egress drain"
    );

    // One publish every 200 ms, drained as it lands. The first drained batch is
    // this connection's advancement baseline AND the rate window's anchor; the
    // window closes on the drain two seconds later.
    let anchor_at = t0 + 200 * MS;
    for n in 1..=11u64 {
        let at = anchor_at + (n - 1) * 200 * MS;
        publish_one(&mut pubs[0], &clock, at, (n as f64, 0.0, 0.0));
        assert_eq!(
            drive_until(&mut gateway, n, &topic),
            1,
            "forwarded frame {n}"
        );
    }

    let l = gateway
        .topic_liveness(&topic)
        .expect("the egress tap fed the observation");
    assert_eq!(
        l.frames_observed, 11,
        "precondition: every publish was drained by the egress tap"
    );
    assert_eq!(
        l.state(),
        LivenessState::Streaming,
        "precondition: dated, so a rate may be served"
    );
    assert_eq!(
        l.rate_estimate,
        Some(cerulion_core::TopicRateEstimate {
            // Ten commits across the 2 s window between anchor and close.
            millihertz: 5_000,
            is_floor: false,
        }),
        "the demand plane must measure EXACTLY, from the publisher's own commit \
         sequence — a floor here would mean `drain_egress_tap` never read it: {l:?}"
    );
    assert_eq!(
        gateway.liveness_tap_count(),
        0,
        "and it still costs no second port"
    );
}

/// The DEMAND-PLANE twin of the observer-plane headline: a publisher
/// restarts with NO drain-visible lull on a topic a remote consumer is streaming,
/// and the row recovers a dated `Streaming` reading.
///
/// This plane is where the silence gate is HARSHEST and where the arm could most
/// easily go silently inert. `sweep()` skips an externally-observed topic
/// entirely, so every observation — and therefore every scrap of the sustained
/// path's evidence — arrives through `note_frames` from `drain_egress_tap`. Two
/// fields have to survive that call for the path to work at all, and both are read
/// there and nowhere else on this plane: `writers_seen` (the single-writer gate)
/// and `queue_emptied` (the non-consuming `has_samples()` answer, which is the
/// structural burst-killer). A gateway that dropped either would leave every
/// DEMANDED topic — exactly the streams Studio renders — permanently unable to
/// heal from a restart, while undemanded ones healed fine.
///
/// The restart is REAL: run 1's publisher and its manager are dropped, freeing the
/// single-writer slot, and a replacement attaches to the same topic on a FRESH
/// clock two hours below the dead run's last stamp. Every drive drains frames, one
/// sweep interval apart, so `quiet_long_enough` is false throughout.
#[test]
fn a_lull_free_restart_resets_the_epoch_on_the_demand_plane() {
    const MS: u64 = 1_000_000;
    const S: u64 = LIVENESS_SWEEP_INTERVAL_NS;
    const _: () = assert!(
        S < REGRESSION_RESET_MIN_GAP_NS,
        "precondition: consecutive drains are INSIDE the silence gate, so the \
         silence-gated reset path cannot be what heals this topic"
    );
    const RUN1_BASE: u64 = 2 * 60 * 60 * 1_000 * MS;
    const RUN2_BASE: u64 = 5 * MS;
    const _: () = assert!(RUN2_BASE < RUN1_BASE);

    let id = unique_id();
    let topic = format!("/epoch/demand/{id}");
    let root = cerulion_core::testing::iceoryx_test_config();
    let make = |tag: &str, base: u64| {
        let clock = Arc::new(VirtualClock::new());
        clock.set(base);
        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("{tag}_{id}"),
                clock: clock.clone() as Arc<dyn Clock>,
                ..Default::default()
            },
            root.clone(),
        )
        .expect("init manager");
        (mgr, clock)
    };

    // The gateway (the OBSERVER) runs its own clock — the one every duration in
    // this test is measured on.
    let gw_clock = Arc::new(VirtualClock::new());
    let t0 = 1_000_000_000u64;
    gw_clock.set(t0);
    let g = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("g_{id}"),
            clock: gw_clock.clone() as Arc<dyn Clock>,
            network: Some(NetworkConfig {
                robot_identity: Some("restartgw".to_string()),
                ..NetworkConfig::default()
            }),
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init gateway G");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic.clone()],
        ingress: vec![],
    };
    let mut gateway = GatewayRuntime::new(g, plan).expect("gateway boot");

    // ---- Run 1, two hours up, and a remote consumer DEMANDS the topic. ----
    let (run1_mgr, run1_clock) = make("w1", RUN1_BASE);
    let mut run1_pub = run1_mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("run-1 publisher");
    drive_with_sweep(&mut gateway, &gw_clock, t0);
    gateway
        .manager()
        .bridge_manager()
        .enable_bridge(&topic)
        .expect("enable");
    drive_with_sweep(&mut gateway, &gw_clock, t0 + S);
    assert_eq!(gateway.active_tap_topics(), vec![topic.clone()]);
    assert_eq!(
        gateway.liveness_tap_count(),
        0,
        "precondition: the observer released its own tap, so EVERY observation \
         below comes from the egress drain"
    );

    let mut forwarded = 0u64;
    let drive_at = |gateway: &mut GatewayRuntime, now: u64, want: u64| {
        gw_clock.set(now);
        drive_until(gateway, want, &topic);
    };
    // The hand-off re-opened the advancement baseline, so run 1 needs two batches:
    // one to close it and one to date.
    publish_one(&mut run1_pub, &run1_clock, RUN1_BASE, (1.0, 0.0, 0.0));
    forwarded += 1;
    drive_at(&mut gateway, t0 + 2 * S, forwarded);
    publish_one(
        &mut run1_pub,
        &run1_clock,
        RUN1_BASE + 10 * MS,
        (2.0, 0.0, 0.0),
    );
    forwarded += 1;
    let dated_at = t0 + 3 * S;
    drive_at(&mut gateway, dated_at, forwarded);
    let l = gateway.topic_liveness(&topic).expect("observing");
    assert_eq!(
        l.last_frame_age_ms,
        Some(0),
        "precondition: run 1 dates normally on the demand plane: {l:?}"
    );
    assert_eq!(l.state(), LivenessState::Streaming);

    // ---- THE RESTART, with no drive-visible pause. ----
    drop(run1_pub);
    drop(run1_mgr);
    let (run2_mgr, run2_clock) = make("w2", RUN2_BASE);
    let mut run2_pub = run2_mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("run-2 publisher — the restarted worker");

    let regime_start = dated_at + S;
    let confirm_at = regime_start + SUSTAINED_REGRESSION_MIN_SPAN_NS;
    let mut at = regime_start;
    let mut step = 0u64;
    while at <= confirm_at {
        publish_one(
            &mut run2_pub,
            &run2_clock,
            RUN2_BASE + step * MS,
            (10.0 + step as f64, 0.0, 0.0),
        );
        forwarded += 1;
        drive_at(&mut gateway, at, forwarded);
        let l = gateway.topic_liveness(&topic).expect("observing");
        assert_eq!(
            gateway.forwarded_count(&topic),
            forwarded,
            "PRECONDITION (measured): every drive of this regime really drained \
             and forwarded a frame, so no silence ever accumulates (at {at} ns)"
        );
        if at < confirm_at {
            assert_eq!(
                l.last_frame_age_ms,
                Some((at - dated_at) / MS),
                "while the regime builds, the row serves the DEAD run's age, \
                 growing: {l:?}"
            );
        }
        at += S;
        step += 1;
    }
    assert_eq!(
        gateway.liveness_tap_count(),
        0,
        "and the whole thing still costs no second port"
    );

    // ---- ONE advancement past the reset, the row is live again. ----
    publish_one(
        &mut run2_pub,
        &run2_clock,
        RUN2_BASE + step * MS,
        (99.0, 0.0, 0.0),
    );
    forwarded += 1;
    drive_at(&mut gateway, at, forwarded);
    let l = gateway.topic_liveness(&topic).expect("observing");
    assert_eq!(
        l.last_frame_age_ms,
        Some(0),
        "THE PIN — PRE-FIX THIS IS `Some({})` AND GROWING FOREVER on the very \
         plane a vizd-demanded topic rides: {l:?}",
        (at - dated_at) / MS
    );
    assert_eq!(
        l.state(),
        LivenessState::Streaming,
        "the demanded row is live again: {l:?}"
    );
}

// ---------------------------------------------------------------------------
// The zero-demand idle wait.
//
// These arms drive `GatewayRuntime::idle_wait` DIRECTLY, which is the only place
// the lost-wakeup race window is reachable: the seam that opens it
// (`set_idle_wait_gate_for_test`) takes `&mut GatewayRuntime`, and every
// production driver has already moved the gateway onto a thread. The
// zero-demand-CEILING half — "a running gateway really does stop spinning" — is
// the netd e2e (`egress_plane_iox2_test.rs`), which observes a real drive thread.
// ---------------------------------------------------------------------------

/// A generous LIVENESS ceiling for the wake arms. Never a wall in units of
/// `GATEWAY_ZERO_DEMAND_IDLE`: the load-bearing assertion in every arm below is the
/// `demand_wakes` COUNTER, which a timeout cannot increment and load can delay but
/// never fake. This bound only catches a wait that hung.
const WAKE_LIVENESS_CEILING: Duration = Duration::from_secs(30);

/// A gateway with ZERO demand BLOCKS on the demand signal, and a real demand
/// transition — arriving from ANOTHER thread while it is parked — is what ends the
/// wait, not the fallback timeout.
///
/// The oracle is `GatewayDriveStats`: `zero_demand_waits` proves the loop took the
/// blocking arm at all, `demand_wakes` proves the wait ended on a SIGNAL. No
/// elapsed time can distinguish those two, which is why both are counters.
#[test]
fn a_zero_demand_gateway_parks_and_a_demand_transition_wakes_it() {
    let topic = format!("/gw/wake/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic.clone()],
        ingress: vec![],
    };
    let (p, _clock, gateway, _pubs) =
        make_producer_and_gateway("gwwake", plan, std::slice::from_ref(&topic));
    let _ = &p;
    let stats = gateway.drive_stats();
    assert!(
        !gateway.any_demand_on(),
        "precondition: no remote has demanded this topic yet"
    );

    // A demand transition landing while the gateway is PARKED. The flip happens on
    // a separate thread — exactly like the zenoh liveliness callback / the
    // reconciler, neither of which is the drive thread.
    let flipper_bridges = gateway.manager().bridge_manager_arc();
    let flipper_topic = topic.clone();
    let flipper = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(40));
        flipper_bridges
            .enable_bridge(&flipper_topic)
            .expect("enable the demand flag")
    });

    let started = Instant::now();
    gateway.idle_wait();
    let elapsed = started.elapsed();
    assert!(
        flipper.join().expect("flipper thread"),
        "the flip must be a genuine false→true transition (the only thing that bumps)"
    );

    assert_eq!(
        stats.zero_demand_waits(),
        1,
        "with no flag ON the loop must take the BLOCKING arm exactly once"
    );
    assert_eq!(
        stats.demand_wakes(),
        1,
        "the wait must have ended on the demand SIGNAL, not on the fallback timeout \
         (elapsed {elapsed:?}) — a timeout can never increment this counter"
    );
    assert!(
        elapsed < WAKE_LIVENESS_CEILING,
        "liveness: the wait must not hang (got {elapsed:?})"
    );
    println!("demand-woken idle wait returned in {elapsed:?}");
}

/// THE lost-wakeup pin (Principle #6). A demand that lands in the race window —
/// AFTER the loop has scanned the flags and found nothing ON, BEFORE it blocks —
/// must still be served immediately.
///
/// Made deterministic by the `set_idle_wait_gate_for_test` seam, which runs a hook
/// at precisely that instant. This holds ONLY because `idle_wait` snapshots the
/// signal's generation BEFORE the scan; an implementation that snapshots after it
/// blocks for the whole `GATEWAY_ZERO_DEMAND_IDLE` fallback and reports a TIMEOUT,
/// which is the failure this arm exists to catch.
#[test]
fn a_demand_that_lands_in_the_race_window_is_never_missed() {
    let topic = format!("/gw/race/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic.clone()],
        ingress: vec![],
    };
    let (p, _clock, mut gateway, _pubs) =
        make_producer_and_gateway("gwrace", plan, std::slice::from_ref(&topic));
    let _ = &p;
    let stats = gateway.drive_stats();

    // The hook fires INSIDE the window: the scan has already reported "nothing ON",
    // and the blocking wait has not yet been entered.
    let gate_bridges = gateway.manager().bridge_manager_arc();
    let gate_topic = topic.clone();
    let fired = Arc::new(AtomicU64::new(0));
    let fired_hook = Arc::clone(&fired);
    gateway.set_idle_wait_gate_for_test(Arc::new(move || {
        // Fire exactly once — `idle_wait` may be called again by a future edit.
        if fired_hook.fetch_add(1, Ordering::SeqCst) == 0 {
            gate_bridges
                .enable_bridge(&gate_topic)
                .expect("enable the demand flag inside the race window");
        }
    }));

    let started = Instant::now();
    gateway.idle_wait();
    let elapsed = started.elapsed();

    assert_eq!(
        fired.load(Ordering::SeqCst),
        1,
        "anti-tautology: the race-window hook must actually have run"
    );
    assert_eq!(
        stats.zero_demand_waits(),
        1,
        "the loop took the blocking arm (the flags really were all OFF at scan time)"
    );
    assert_eq!(
        stats.demand_wakes(),
        1,
        "A DEMAND THAT LANDED IN THE RACE WINDOW MUST BE SERVED AT ONCE — this is 0, \
         and the wait spends the whole {GATEWAY_ZERO_DEMAND_IDLE:?} fallback, if the \
         generation is snapshotted AFTER the flag scan instead of before it \
         (elapsed {elapsed:?})"
    );
    assert!(
        elapsed < WAKE_LIVENESS_CEILING,
        "liveness: the wait must not hang (got {elapsed:?})"
    );
    // The demand is REAL, not just signalled: the next pass attaches the tap.
    gateway
        .drive_once()
        .expect("drive after the race-window demand");
    assert_eq!(
        gateway.active_tap_topics(),
        vec![topic.clone()],
        "the race-window demand must produce a real tap, not merely a wake"
    );
}

/// The ON path is BYTE-UNCHANGED: while any flag is ON the loop paces at the
/// 1 ms `GATEWAY_IDLE_POLL` and NEVER takes the blocking arm. This is the pin
/// behind the design claim that the liveness observer's `queue_emptied`, its
/// sustained-regression bands and its rate window all keep the cadence they
/// were derived against — they are demand-plane properties, and the demand plane
/// never parks.
///
/// The load-bearing assertion is `zero_demand_waits() == 0` (an exact zero load
/// cannot inflate), not a pass rate.
#[test]
fn with_demand_on_the_loop_never_takes_the_blocking_arm() {
    let topic = format!("/gw/on/{}", unique_id());
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic.clone()],
        ingress: vec![],
    };
    let (p, _clock, mut gateway, _pubs) =
        make_producer_and_gateway("gwon", plan, std::slice::from_ref(&topic));
    let _ = &p;
    let stats = gateway.drive_stats();

    gateway
        .manager()
        .bridge_manager()
        .enable_bridge(&topic)
        .expect("demand arrives");
    assert!(gateway.any_demand_on(), "the flag is ON");

    const IDLE_PASSES: usize = 20;
    let started = Instant::now();
    for _ in 0..IDLE_PASSES {
        gateway.drive_once().expect("drive");
        gateway.idle_wait();
    }
    let elapsed = started.elapsed();

    assert_eq!(
        stats.zero_demand_waits(),
        0,
        "a DEMANDED gateway must never park — the demand plane's cadence is what \
         the liveness observer's rules were derived against"
    );
    assert_eq!(
        stats.demand_wakes(),
        0,
        "and it therefore posts no demand wakes either"
    );
    assert_eq!(
        stats.passes(),
        IDLE_PASSES as u64,
        "one pass per drive_once (Principle #3)"
    );
    // A CEILING only — contention can only make this slower, and the exact-zero
    // assertions above are what carry the claim.
    assert!(
        elapsed < WAKE_LIVENESS_CEILING,
        "liveness: {IDLE_PASSES} demanded passes must not take {elapsed:?}"
    );
    println!(
        "{IDLE_PASSES} demanded idle passes in {elapsed:?} \
         (the unchanged 1 ms tick)"
    );
}

/// A RUNTIME-registered topic of a BUILT-IN type is catalog-NAMED
/// through the built-in corpus's hash→name bindings — the rmw shape: an rmw
/// `std_msgs/String` publisher registers `(topic, String::SCHEMA_HASH)` over the
/// reg-channel with no name, and the desk consumes it only if the catalog carries
/// `schema_name: Some("std_msgs/String")` (it then decodes locally).
///
/// The serving's bindings come from `native_ros2_messages::builtin_hash_bindings`
/// — the ONE corpus source both the CLI's `build_schema_serving` and netd's
/// start-booted standing gateway now hand across — so this arm pins the whole
/// chain a built-in name rides: corpus binding (hash == the generated type's
/// `SCHEMA_HASH`) → reg-channel hash → catalog name. Without the builtin
/// bindings the handed slice would be custom-only, so this row would read
/// `None`; the same gateway's SECOND runtime topic,
/// registered under a hash NO binding carries, must STILL read `None` — the
/// anti-tautology half proving the name comes from the map, not from the
/// gateway inventing one for every runtime topic. One gateway, two rows, hand
/// oracles.
#[test]
fn a_runtime_topic_of_a_builtin_type_is_catalog_named_through_the_corpus_bindings() {
    let id = unique_id();
    let named_topic = format!("/cn/chatter/{id}");
    let unnamed_topic = format!("/cn/mystery/{id}");
    let string_hash = native_ros2_messages::std_msgs::String::SCHEMA_HASH;
    // A hash NO corpus binding carries (the control row).
    const MYSTERY_HASH: u64 = 0x1541_0000_DEAD_0001;
    let robot = "cnrob";

    let bindings = native_ros2_messages::builtin_hash_bindings();
    assert_eq!(
        bindings.len(),
        native_ros2_messages::BUILTIN_MSGS.len(),
        "the corpus bindings are total (one per vendored message)"
    );
    assert!(
        !bindings.iter().any(|b| b.schema_hash == MYSTERY_HASH),
        "the control hash must be absent from the corpus map"
    );
    let serving = cerulion_core::SchemaServing {
        topic_schemas: vec![], // NO topic→name binding: the catalog MUST use the hash map
        schema_docs: vec![],   // built-ins are never served — the desk has them
        schema_hashes: bindings,
    };
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![], // empty boot plan — both topics register at runtime
        ingress: vec![],
    };
    let (p, _clock, mut gateway, _port) =
        make_serving_producer_and_gateway("cn", plan, robot, serving);

    // The rmw shape: a publisher exists, and the process registers ONLY
    // (topic, schema_hash) over the reg-channel — no name.
    let _pub_named = p
        .create_publisher_simple(&named_topic, MaxSliceLen::const_new(256))
        .expect("named-topic publisher");
    let _pub_unnamed = p
        .create_publisher_simple(&unnamed_topic, MaxSliceLen::const_new(256))
        .expect("unnamed-topic publisher");
    assert!(p
        .register_dynamic_egress_topic(&named_topic, string_hash)
        .expect("register the built-in-typed runtime topic"));
    assert!(p
        .register_dynamic_egress_topic(&unnamed_topic, MYSTERY_HASH)
        .expect("register the control runtime topic"));
    let both_registered = |gateway: &GatewayRuntime| {
        gateway
            .is_runtime_registered(&named_topic)
            .expect("reg check")
            && gateway
                .is_runtime_registered(&unnamed_topic)
                .expect("reg check")
    };
    for _ in 0..10 {
        p.republish_dynamic_egress_for_test();
        gateway.drive_once().expect("drive");
        if both_registered(&gateway) {
            break;
        }
    }
    assert!(
        both_registered(&gateway),
        "both runtime topics registered on the gateway"
    );

    let catalog = gateway.catalog_serve_for_test(robot);
    let row = |topic: &str| {
        catalog
            .entries
            .iter()
            .find(|e| e.topic == topic)
            .unwrap_or_else(|| panic!("the catalog must list runtime topic {topic}"))
    };
    // THE PIN: the built-in-typed runtime topic is NAMED — with the exact
    // generated wire hash beside it.
    let named = row(&named_topic);
    assert_eq!(
        named.schema_hash,
        Some(string_hash),
        "the catalog carries the reg-channel hash (== std_msgs::String::SCHEMA_HASH)"
    );
    assert_eq!(
        named.schema_name.as_deref(),
        Some("std_msgs/String"),
        "a runtime topic of a BUILT-IN type is catalog-named through the corpus bindings \
         (without the corpus bindings: None — the desk's SchemaUnavailable)"
    );
    // The CONTROL: a runtime topic under a hash the map does not carry stays
    // unnamed — the name is resolved, never fabricated.
    let unnamed = row(&unnamed_topic);
    assert_eq!(unnamed.schema_hash, Some(MYSTERY_HASH));
    assert_eq!(
        unnamed.schema_name, None,
        "a hash absent from every binding must NOT be named"
    );
}

/// A serving merged into a running gateway reaches the
/// LIVE catalog + schema serve — and the merge is FIRST-WINS, so the boot
/// serving stays authoritative for keys it already holds.
///
/// The defect this pins out: were the boot serving materialized into immutable
/// maps, a gateway booted (e.g. netd's standing start boot, built-in corpus
/// only) would NEVER incorporate a later registration's custom types — they
/// would catalogue `schema_name: None` with zero served docs (the desk's
/// `SchemaUnavailable`) while a gateway handed the same serving AT boot resolves
/// them. Driven through the REAL read paths (`catalog_serve_for_test` /
/// `schema_serve_for_test` — the exact gated decisions the zenoh callbacks run)
/// against hand oracles:
///
/// - a topic under the MERGED custom hash is catalog-NAMED and its doc is served
///   with its verbatim text (without the merge: `None` + not-found);
/// - a topic under the BOOT hash keeps its BOOT name even though the merged
///   serving carries a CONFLICTING rebind of the same hash (first-wins), and the
///   boot doc still serves;
/// - merging the SAME serving twice changes nothing (idempotence).
#[test]
fn a_merged_serving_reaches_the_live_catalog_and_schema_serve_first_wins() {
    let id = unique_id();
    let boot_topic = format!("/cnm/boot/{id}");
    let later_topic = format!("/cnm/later/{id}");
    const BOOT_HASH: u64 = 0x1541_0001_0000_0001;
    const LATER_HASH: u64 = 0x1541_0001_0000_0002;
    let robot = "cnmerge";

    let boot_doc_text = "int32 alpha\n";
    let boot_serving = cerulion_core::SchemaServing {
        topic_schemas: vec![],
        schema_docs: vec![cerulion_core::SchemaDoc {
            qualified: "boot_msgs/Alpha".to_string(),
            encoding: cerulion_core::SchemaEncoding::Msg,
            text: boot_doc_text.to_string(),
            deps: vec![],
        }],
        schema_hashes: vec![cerulion_core::SchemaHashName {
            schema_hash: BOOT_HASH,
            qualified: "boot_msgs/Alpha".to_string(),
        }],
    };
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![],
        ingress: vec![],
    };
    let (p, _clock, mut gateway, _port) =
        make_serving_producer_and_gateway("cnm", plan, robot, boot_serving);

    // Both topics register at runtime (the reg-channel shape — hash, no name).
    assert!(p
        .register_dynamic_egress_topic(&boot_topic, BOOT_HASH)
        .expect("register boot-hash topic"));
    assert!(p
        .register_dynamic_egress_topic(&later_topic, LATER_HASH)
        .expect("register later-hash topic"));
    for _ in 0..10 {
        p.republish_dynamic_egress_for_test();
        gateway.drive_once().expect("drive");
        let both = gateway.is_runtime_registered(&boot_topic).expect("reg")
            && gateway.is_runtime_registered(&later_topic).expect("reg");
        if both {
            break;
        }
    }

    // PREMISE (the pre-merge frozen state): the later hash is unnamed and its
    // type is not served.
    let before = gateway.catalog_serve_for_test(robot);
    let entry = |reply: &cerulion_core::transport::cerulion_q::CatalogReply, t: &str| {
        reply
            .entries
            .iter()
            .find(|e| e.topic == t)
            .unwrap_or_else(|| panic!("catalog must list {t}"))
            .clone()
    };
    assert_eq!(entry(&before, &later_topic).schema_name, None);
    assert!(gateway
        .schema_serve_for_test(robot, "later_msgs/Widget")
        .docs
        .is_empty());

    // THE MERGE — a later registration's serving, carrying the custom binding +
    // doc AND a conflicting rebind of the BOOT hash (which must not take).
    let later_doc_text = "float64 v\nint32 n\n";
    let later_serving = cerulion_core::SchemaServing {
        topic_schemas: vec![],
        schema_docs: vec![cerulion_core::SchemaDoc {
            qualified: "later_msgs/Widget".to_string(),
            encoding: cerulion_core::SchemaEncoding::Msg,
            text: later_doc_text.to_string(),
            deps: vec![],
        }],
        schema_hashes: vec![
            cerulion_core::SchemaHashName {
                schema_hash: LATER_HASH,
                qualified: "later_msgs/Widget".to_string(),
            },
            // The conflicting rebind — first-wins must keep the boot name.
            cerulion_core::SchemaHashName {
                schema_hash: BOOT_HASH,
                qualified: "later_msgs/WrongAlpha".to_string(),
            },
        ],
    };
    let handles = gateway.schema_serving_handles();
    handles.merge(&later_serving);

    for pass in ["merged", "merged twice (idempotent)"] {
        let after = gateway.catalog_serve_for_test(robot);
        assert_eq!(
            entry(&after, &later_topic).schema_name.as_deref(),
            Some("later_msgs/Widget"),
            "{pass}: the merged binding names the runtime topic (without the merge: None forever)"
        );
        assert_eq!(
            entry(&after, &boot_topic).schema_name.as_deref(),
            Some("boot_msgs/Alpha"),
            "{pass}: first-wins — the conflicting rebind must not displace the boot name"
        );
        let widget = gateway.schema_serve_for_test(robot, "later_msgs/Widget");
        assert_eq!(widget.docs.len(), 1, "{pass}: the merged doc serves");
        assert_eq!(widget.docs[0].text, later_doc_text);
        let alpha = gateway.schema_serve_for_test(robot, "boot_msgs/Alpha");
        assert_eq!(alpha.docs.len(), 1, "{pass}: the boot doc still serves");
        assert_eq!(alpha.docs[0].text, boot_doc_text);
        handles.merge(&later_serving);
    }
}

/// The pure accumulator fold
/// `SchemaServing::merge_from` — netd's egress plane folds every registration's
/// serving into one plane-lifetime `SchemaServing` and seeds any gateway
/// (re-)boot from it, so the fold's FIRST-WINS + additive + idempotent contract
/// is what keeps a replacement gateway from forgetting earlier registrations.
/// Hand oracles, no transport.
#[test]
fn merge_from_is_first_wins_additive_and_idempotent() {
    let doc = |q: &str, text: &str| cerulion_core::SchemaDoc {
        qualified: q.to_string(),
        encoding: cerulion_core::SchemaEncoding::Msg,
        text: text.to_string(),
        deps: vec![],
    };
    let binding = |hash: u64, q: &str| cerulion_core::SchemaHashName {
        schema_hash: hash,
        qualified: q.to_string(),
    };
    let topic = |t: &str, q: &str| cerulion_core::TopicSchema {
        topic: t.to_string(),
        schema_name: q.to_string(),
    };
    let mut acc = cerulion_core::SchemaServing {
        topic_schemas: vec![topic("/a", "pkg/A")],
        schema_docs: vec![doc("pkg/A", "int32 a\n")],
        schema_hashes: vec![binding(1, "pkg/A")],
    };
    // A later serving: a NEW type B, plus CONFLICTS on every existing key — a
    // rebind of /a (canonicalized: the bare "a" spelling must still collide), a
    // rebind of hash 1, and a different doc under pkg/A. First-wins keeps A's.
    let later = cerulion_core::SchemaServing {
        topic_schemas: vec![topic("a", "pkg/WrongA"), topic("/b", "pkg/B")],
        schema_docs: vec![doc("pkg/A", "int64 wrong\n"), doc("pkg/B", "int32 b\n")],
        schema_hashes: vec![binding(1, "pkg/WrongA"), binding(2, "pkg/B")],
    };
    acc.merge_from(&later);
    acc.merge_from(&later); // idempotent — the second fold changes nothing
    assert_eq!(acc.topic_schemas.len(), 2, "additive: A + B, no dup of /a");
    assert_eq!(
        acc.topic_schemas[0].schema_name, "pkg/A",
        "first-wins on /a"
    );
    assert_eq!(acc.topic_schemas[1].topic, "/b");
    assert_eq!(acc.schema_hashes.len(), 2);
    assert_eq!(
        acc.schema_hashes[0].qualified, "pkg/A",
        "first-wins on hash 1"
    );
    assert_eq!(acc.schema_hashes[1].qualified, "pkg/B");
    assert_eq!(acc.schema_docs.len(), 2);
    assert_eq!(
        acc.schema_docs[0].text, "int32 a\n",
        "first-wins on the pkg/A doc"
    );
    assert_eq!(acc.schema_docs[1].qualified, "pkg/B");
    // Folding into an EMPTY accumulator takes everything (the first
    // registration on a fresh plane).
    let mut fresh = cerulion_core::SchemaServing::default();
    fresh.merge_from(&acc);
    assert_eq!(fresh, acc, "an empty fold takes the whole serving");
}
