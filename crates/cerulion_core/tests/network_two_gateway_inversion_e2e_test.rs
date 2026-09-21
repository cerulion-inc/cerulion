// SPDX-License-Identifier: AGPL-3.0-only
//! The QUERYABLE INVERSION e2e — demand pulled by a DIALER GET against
//! a PRODUCER's QUERYABLE, the two wire-proven directions of a strict
//! connect-only, listen-less, scouting-off zenoh 1.8 peer link.
//!
//! ```text
//! Machine A (root_a):  P (network-free) + G_prod (LISTEN, announce /t,
//!                      query surface cerulion_q/{prod}/**)
//! Machine B (root_b):  G_cons (CONNECT-only, listen-less, ingress /t,
//!                      demand-GET loop) + local sub
//!
//! G_cons demand GET (cerulion_q/*/demand{/t}) ──▶ G_prod queryable ──▶ enable
//!   ──▶ G_prod tap ──▶ zenoh TCP ──▶ G_cons ingress ──▶ B local sub
//! ```
//!
//! Unlike [`network_two_gateway_reconcile_e2e_test`] (which exercises the
//! liveliness-query BELT), these tests isolate the INVERSION's GET path as the
//! SOLE enabler by SUPPRESSING the demander's `cerulion_lv` DEMAND TOKEN (via
//! `set_suppress_ingress_token_for_test`): with no token, the producer's
//! liveliness subscriber sees no `Put` and its reconciler gathers empty, so the
//! ONLY thing that can flip the egress flag is the demand queryable.
//!
//! - `demand_queryable_get_path_alone_delivers_byte_identical` — the decisive
//!   e2e: the GET path alone completes demand→enable→tap→forward→reinject
//!   byte-identical, with the enable attributed to the queryable (grant/ack
//!   counters) and NOT the reconciler (`reconciler_enabled_count == 0`).
//!   Deleting the `enable_bridge` call in `handle_demand_verb`
//!   (network.rs) fails this test — no enable, no tap, no delivery.
//! - `demand_expiry_disables_after_ttl` — a GET-granted topic is released once
//!   its last GET ages past DEMAND_TTL AND liveliness is absent (driven by the
//!   `run_get_expiry_once(now)` sync seam at a future `now`).
//! - `allowlist_refuses_non_egress_topic_get` — a GET for a non-egress topic
//!   gets the REFUSED ack + no flag flip (the allow-list gate inside the enable).
//! - `identity_harvest_targets_producer` — the demander learns the producer
//!   identity from the announce space (the wildcard-fallback arm is the pure
//!   `demand_get_selectors_wildcard_then_explicit` unit test in network.rs).
//! - `suppressed_producer_refuses_get_without_enabling_or_stamping` — with
//!   the producer's `suppress_live_demand` seam set the
//!   queryable replies REFUSED without enabling or stamping: the demander sees a
//!   REFUSED ack, the producer records a refusal + ZERO grants + no keepalive,
//!   and the egress flag stays OFF (egress `AllowAll`, so suppression — not the
//!   allow-list — is the isolated cause).
//!
//! NOT `#[serial]`: distinct per-test SHM roots, per-run probed ports, bounded
//! retries ⇒ parallel-safe.

use std::net::TcpListener;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::gateway::{
    GatewayEgressPolicy, GatewayIngressEntry, GatewayPlan, GatewayRuntime,
};
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::subscriber::CerulionSubscriber;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use native_ros2_messages::geometry_msgs::Vector3;

static COUNTER: AtomicU64 = AtomicU64::new(0);

const PROD_ROBOT: &str = "inv-prod";
const CONS_ROBOT: &str = "inv-cons";

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

/// (schema_hash, sequence, timestamp_ns, total_size, payload).
type Received = (u64, u32, u64, u32, Vec<u8>);

fn vector3_payload(x: f64, y: f64, z: f64) -> Vec<u8> {
    let mut v = Vec::with_capacity(24);
    v.extend_from_slice(&x.to_le_bytes());
    v.extend_from_slice(&y.to_le_bytes());
    v.extend_from_slice(&z.to_le_bytes());
    v
}

const PLAN: [(u64, (f64, f64, f64)); 3] = [
    (10_000_000, (1.25, -2.5, 3.75)),
    (20_000_000, (4.5, 5.5, -6.5)),
    (30_000_000, (7.5, 8.5, 9.5)),
];

fn oracle() -> Vec<Received> {
    PLAN.iter()
        .enumerate()
        .map(|(i, (ts, (x, y, z)))| {
            (
                Vector3::SCHEMA_HASH,
                i as u32,
                *ts,
                (WireHeader::SIZE + 24) as u32,
                vector3_payload(*x, *y, *z),
            )
        })
        .collect()
}

/// The assembled two-gateway inversion rig (both gateways alive, the topic's
/// SHM service created by P, B's local subscriber ready).
struct Rig {
    a_clock: Arc<VirtualClock>,
    a_pub: CerulionPublisher,
    gateway_prod: GatewayRuntime,
    gateway_cons: GatewayRuntime,
    b_sub: CerulionSubscriber,
    /// G_prod's egress bridge flag for the topic (the observable the queryable
    /// flips + the tap reads).
    flag: Arc<AtomicBool>,
}

/// Build machine A (producer P + gateway G_prod) and machine B (gateway G_cons +
/// local sub). The demander's ingress DEMAND TOKEN is always SUPPRESSED so the
/// queryable is the sole enable path. Bounded retry absorbs the probe→rebind
/// port race. `egress` is G_prod's egress posture.
fn setup_rig(tag: &str, egress: GatewayEgressPolicy) -> Rig {
    let id = unique_id();
    let topic = format!("/inv/{tag}/{id}");
    let root_a = cerulion_core::testing::iceoryx_test_config();

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
                node_name: format!("inv_p_{tag}_{id}_{attempt}"),
                clock: clock_dyn,
                ..Default::default()
            },
            root_a.clone(),
        )
        .expect("init P");
        let g = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("inv_gprod_{tag}_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    robot_identity: Some(PROD_ROBOT.to_string()),
                    ..Default::default()
                }),
                ..Default::default()
            },
            root_a.clone(),
        )
        .expect("init G_prod");
        let plan = GatewayPlan {
            egress_policy: egress.clone(),
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
    let Some((p_transport, a_clock, gateway_prod, port)) = a else {
        panic!("could not establish G_prod's listening session in 3 attempts");
    };
    // P's producer publisher creates the topic's SHM data service (G_prod taps it).
    let a_pub = p_transport
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("P publisher");

    // Machine B: distinct root; G_cons connect-only + listen-less. Suppress its
    // ingress DEMAND TOKEN BEFORE new() so only the queryable path can enable.
    let b_mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("inv_gcons_{tag}_{id}"),
            network: Some(NetworkConfig {
                connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                robot_identity: Some(CONS_ROBOT.to_string()),
                ..Default::default()
            }),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init G_cons manager");
    b_mgr
        .network()
        .expect("G_cons network")
        .set_suppress_ingress_token_for_test(true);
    let b_sub = b_mgr.create_subscriber(&topic).expect("B subscriber");
    let plan_cons = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowList(vec![]), // ingress-only
        announce: vec![],
        ingress: vec![GatewayIngressEntry {
            topic: topic.clone(),
            schema_hash: Vector3::SCHEMA_HASH,
        }],
    };
    let gateway_cons = GatewayRuntime::new(b_mgr, plan_cons).expect("G_cons boot");

    let flag = gateway_prod
        .manager()
        .bridge_manager()
        .register_topic(&topic)
        .expect("G_prod bridge flag handle");

    Rig {
        a_clock,
        a_pub,
        gateway_prod,
        gateway_cons,
        b_sub,
        flag,
    }
}

/// Drive the demander's demand GET (and the producer's tap attach) until the
/// egress flag flips ON or the deadline elapses. Returns whether it flipped.
fn drive_until_enabled(rig: &mut Rig, deadline: Instant) -> bool {
    while !rig.flag.load(Ordering::Relaxed) {
        if Instant::now() >= deadline {
            return false;
        }
        rig.gateway_cons.demand_get_once().expect("demand get pass");
        rig.gateway_prod.drive_once().expect("drive prod");
        std::thread::sleep(Duration::from_millis(20));
    }
    true
}

/// Test (decisive): the demand-queryable GET path ALONE (demander token
/// suppressed, reconciler naturally blind) completes the full chain
/// byte-identical, with the enable attributed to the queryable.
#[test]
fn demand_queryable_get_path_alone_delivers_byte_identical() {
    let mut rig = setup_rig("decisive", GatewayEgressPolicy::AllowAll);

    // Handshake: the GET flips G_prod's flag (bounded 20 s).
    assert!(
        drive_until_enabled(&mut rig, Instant::now() + Duration::from_secs(20)),
        "the demand GET did not flip G_prod's egress flag — the queryable-inversion path is dead"
    );

    // Attach the tap BEFORE publishing (the tap has no history).
    let attach_deadline = Instant::now() + Duration::from_secs(5);
    while rig.gateway_prod.active_tap_topics().is_empty() {
        assert!(Instant::now() < attach_deadline, "tap did not attach");
        rig.gateway_prod.drive_once().expect("drive attach");
        std::thread::sleep(Duration::from_millis(10));
    }

    // Publish the plan on P.
    for (ts, (x, y, z)) in PLAN {
        rig.a_clock.set(ts);
        let mut proxy = rig.a_pub.loan_proxy::<Vector3>().expect("P loan_proxy");
        proxy.x = x;
        proxy.y = y;
        proxy.z = z;
    }

    // Drive G_prod to forward + collect on B (bounded). Keep GETting so the
    // producer keeps egressing.
    let mut got: Vec<Received> = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(10);
    while got.len() < PLAN.len() && Instant::now() < deadline {
        rig.gateway_cons.demand_get_once().expect("demand get pass");
        rig.gateway_prod.drive_once().expect("drive forward");
        rig.b_sub
            .try_receive(|msg| {
                let h = msg.header();
                got.push((
                    h.schema_hash,
                    h.sequence,
                    h.timestamp_ns,
                    h.total_size,
                    msg.payload().to_vec(),
                ));
            })
            .expect("B try_receive");
        if got.len() < PLAN.len() {
            std::thread::sleep(Duration::from_millis(20));
        }
    }

    assert_eq!(
        got,
        oracle(),
        "B must receive P's frames verbatim through the queryable-inversion GET path"
    );
    // Attribution: the enable came from the QUERYABLE (grant + enabled ack), NOT
    // the reconciler (which gathered empty — the demander declared no token).
    assert!(
        rig.gateway_prod.demand_grant_count() >= 1,
        "the enable must be attributed to a demand-queryable grant; got {}",
        rig.gateway_prod.demand_grant_count()
    );
    assert!(
        rig.gateway_cons.demand_enabled_ack_count() >= 1,
        "the demander must have received at least one ENABLED ack; got {}",
        rig.gateway_cons.demand_enabled_ack_count()
    );
    assert_eq!(
        rig.gateway_prod.reconciler_enabled_count(),
        0,
        "the reconciler must NOT have enabled the flag (the demander declared no demand token, \
         so the reconciler gathered empty) — the enable is solely the queryable's"
    );
}

/// Test (expiry): a GET-granted topic is DISABLED once its last GET ages past
/// DEMAND_TTL AND liveliness is absent — driven deterministically via the
/// `run_get_expiry_once(now)` sync seam at a future `now` (simulating the
/// demander having stopped GETting). Attribution: the disable is the expiry's
/// (`demand_expiry_count`), not the reconciler's.
#[test]
fn demand_expiry_disables_after_ttl() {
    let mut rig = setup_rig("expiry", GatewayEgressPolicy::AllowAll);

    // Establish demand: the GET flips the flag + records a keepalive stamp.
    assert!(
        drive_until_enabled(&mut rig, Instant::now() + Duration::from_secs(20)),
        "the demand GET did not flip the flag"
    );
    assert!(
        rig.flag.load(Ordering::Relaxed),
        "flag must be ON after the GET"
    );

    // A same-time expiry sweep must NOT disable (the grant is fresh).
    rig.gateway_prod
        .run_get_expiry_once(Instant::now())
        .expect("expiry pass (fresh)");
    assert!(
        rig.flag.load(Ordering::Relaxed),
        "a fresh GET-granted topic must NOT be expired"
    );

    // Advance `now` past the TTL (the demander stopped GETting) → the topic is
    // expired AND liveliness is absent (no token) → disabled.
    let future = Instant::now() + Duration::from_secs(7) + Duration::from_secs(1);
    rig.gateway_prod
        .run_get_expiry_once(future)
        .expect("expiry pass (aged)");
    assert!(
        !rig.flag.load(Ordering::Relaxed),
        "the topic must be DISABLED once its last GET aged past DEMAND_TTL and liveliness is absent"
    );
    assert!(
        rig.gateway_prod.demand_expiry_count() >= 1,
        "the disable must be attributed to the GET-expiry sweep; got {}",
        rig.gateway_prod.demand_expiry_count()
    );
}

/// Test (allow-list): a GET demanding a topic NOT in the egress allow-list gets
/// the REFUSED ack and never flips the flag — the allow-list gate inside the
/// enable still enforces posture on the inversion path.
#[test]
fn allowlist_refuses_non_egress_topic_get() {
    // G_prod announces the topic but its egress posture is deny-all (empty
    // allow-list), so the demand GET is refused.
    let mut rig = setup_rig("allowlist", GatewayEgressPolicy::AllowList(vec![]));

    // Drive several GET passes; the flag must NEVER flip.
    let deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < deadline && rig.gateway_cons.demand_refused_ack_count() == 0 {
        rig.gateway_cons.demand_get_once().expect("demand get pass");
        assert!(
            !rig.flag.load(Ordering::Relaxed),
            "a non-egress topic's flag must NEVER flip on a demand GET"
        );
        std::thread::sleep(Duration::from_millis(20));
    }

    assert!(
        rig.gateway_cons.demand_refused_ack_count() >= 1,
        "the demander must have received a REFUSED ack for the non-egress topic; got {}",
        rig.gateway_cons.demand_refused_ack_count()
    );
    assert!(
        rig.gateway_prod.demand_refusal_count() >= 1,
        "the producer must have recorded the refusal; got {}",
        rig.gateway_prod.demand_refusal_count()
    );
    assert!(
        !rig.flag.load(Ordering::Relaxed),
        "the non-egress topic's flag must stay false"
    );
    assert_eq!(
        rig.gateway_prod.demand_grant_count(),
        0,
        "a refused topic must record ZERO grants"
    );
}

/// Test (identity harvest): the demander learns the producer's robot identity
/// from the ANNOUNCE space and it lands in the harvested set (which the explicit
/// GET selectors then target). The wildcard-fallback arm (empty identities) is
/// pinned by the pure `demand_get_selectors_wildcard_then_explicit` unit test.
#[test]
fn identity_harvest_targets_producer() {
    let mut rig = setup_rig("identity", GatewayEgressPolicy::AllowAll);

    // Before any GET pass, no identity is harvested (wildcard fallback state).
    assert_eq!(
        rig.gateway_cons.harvested_identity_count(),
        0,
        "no identity should be harvested before the first demand-GET pass"
    );

    // Drive GET passes until the producer's identity is harvested (bounded).
    let deadline = Instant::now() + Duration::from_secs(10);
    while rig.gateway_cons.harvested_identity_count() == 0 && Instant::now() < deadline {
        rig.gateway_cons.demand_get_once().expect("demand get pass");
        std::thread::sleep(Duration::from_millis(20));
    }

    let harvested = rig.gateway_cons.harvested_identities();
    assert!(
        harvested.iter().any(|r| r == PROD_ROBOT),
        "the demander must harvest the producer identity '{PROD_ROBOT}' from the announce space; \
         got {harvested:?}"
    );
    // Sanity: the demand-GET loop actually ran.
    assert!(
        rig.gateway_cons.demand_get_pass_count() >= 1,
        "the demander must have run at least one demand-GET pass"
    );
}

/// Test (suppress seam): with the PRODUCER's
/// `suppress_live_demand` seam set, the demand queryable is a NON-flipping path —
/// it replies REFUSED without `enable_bridge` or a keepalive stamp. Egress is
/// `AllowAll` here, so WITHOUT suppression this topic would be GRANTED; a refusal
/// therefore isolates the suppress seam (not the allow-list) as the cause. The
/// reconcile e2e uses the suppress arm (to make the reconciler
/// the sole flag-flipper) without asserting it DIRECTLY — this pins it.
#[test]
fn suppressed_producer_refuses_get_without_enabling_or_stamping() {
    let mut rig = setup_rig("suppress", GatewayEgressPolicy::AllowAll);

    // Suppress the PRODUCER's queryable enable (a live-read atomic — setting it
    // after `GatewayRuntime::new` declared the queryable still takes effect).
    rig.gateway_prod
        .manager()
        .network()
        .expect("G_prod network")
        .set_suppress_live_demand_for_test(true);

    // Drive demander GET passes until it receives a REFUSED ack (bounded). The
    // egress flag must NEVER flip — asserted every iteration.
    let deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < deadline && rig.gateway_cons.demand_refused_ack_count() == 0 {
        rig.gateway_cons.demand_get_once().expect("demand get pass");
        assert!(
            !rig.flag.load(Ordering::Relaxed),
            "a suppressed producer must NEVER flip the egress flag"
        );
        std::thread::sleep(Duration::from_millis(20));
    }

    // Demander side: it received a REFUSED ack.
    assert!(
        rig.gateway_cons.demand_refused_ack_count() >= 1,
        "the demander must receive a REFUSED ack from the suppressed producer; got {}",
        rig.gateway_cons.demand_refused_ack_count()
    );
    // Producer side: refusal recorded, ZERO grants, NO keepalive stamp.
    assert!(
        rig.gateway_prod.demand_refusal_count() >= 1,
        "the producer must record the refusal; got {}",
        rig.gateway_prod.demand_refusal_count()
    );
    assert_eq!(
        rig.gateway_prod.demand_grant_count(),
        0,
        "a suppressed producer must record ZERO grants"
    );
    assert!(
        rig.gateway_prod.granted_topics().is_empty(),
        "a suppressed producer must hold NO keepalive stamps; got {:?}",
        rig.gateway_prod.granted_topics()
    );
    // The egress bridge flag (which `is_enabled` reads for the ack) stays OFF.
    assert!(
        !rig.flag.load(Ordering::Relaxed),
        "the egress bridge flag must stay disabled under suppression (no egress)"
    );
}
