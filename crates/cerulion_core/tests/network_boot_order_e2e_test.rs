// SPDX-License-Identifier: AGPL-3.0-only
//! Boot-order e2e for two LISTENING gateways: a demanded topic flows whichever
//! side holds the connect endpoint.
//!
//! ```text
//! Machine A (root_a):  P (network-free) + G_prod (LISTEN, announce /t)
//! Machine B (root_b):  G_cons (LISTEN, ingress /t, demand-GET loop) + local sub
//! ```
//!
//! With multicast scouting off, a machine that boots finds the listening
//! machines already up over mDNS and lists them as connect endpoints, so when
//! both machines listen (`CERULION_NETD_LISTEN` on each) either side can be the
//! dialer. These tests pass the connect endpoint by hand (no mDNS):
//!
//! - `producer_boots_second_and_dials_the_running_consumer`, G_prod CONNECTS
//!   to G_cons, so the consumer's demand rides an INBOUND link at the consumer.
//! - `consumer_boots_second_and_dials_the_running_producer`, G_cons CONNECTS
//!   to G_prod, the outbound shape a desk has always used.
//!
//! Each runs the production demand paths (the consumer's demand token and its
//! demand-GET loop, the producer's queryable and reconciler) and asserts P's
//! frames arrive at B byte-identical.
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
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::subscriber::CerulionSubscriber;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use native_ros2_messages::geometry_msgs::Vector3;

static COUNTER: AtomicU64 = AtomicU64::new(0);

const PROD_ROBOT: &str = "order-prod";
const CONS_ROBOT: &str = "order-cons";

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

const PLAN: [(u64, (f64, f64, f64)); 3] = [
    (10_000_000, (1.25, -2.5, 3.75)),
    (20_000_000, (4.5, 5.5, -6.5)),
    (30_000_000, (7.5, 8.5, 9.5)),
];

fn oracle() -> Vec<Received> {
    PLAN.iter()
        .enumerate()
        .map(|(i, (ts, (x, y, z)))| {
            let mut v = Vec::with_capacity(24);
            v.extend_from_slice(&x.to_le_bytes());
            v.extend_from_slice(&y.to_le_bytes());
            v.extend_from_slice(&z.to_le_bytes());
            (
                Vector3::SCHEMA_HASH,
                i as u32,
                *ts,
                (WireHeader::SIZE + 24) as u32,
                v,
            )
        })
        .collect()
}

/// Which machine boots second (and therefore holds the connect endpoint).
#[derive(Clone, Copy)]
enum SecondToBoot {
    Producer,
    Consumer,
}

struct Producer {
    _p: Arc<TransportManager>,
    clock: Arc<VirtualClock>,
    publisher: CerulionPublisher,
    gateway: GatewayRuntime,
}

struct Consumer {
    gateway: GatewayRuntime,
    sub: CerulionSubscriber,
}

/// Boot machine A: P + a LISTENING G_prod that announces `topic`, connecting to
/// `connect` (empty when A boots first). Bounded retry absorbs the probe→bind
/// port race. Returns the producer and its listen port.
fn boot_producer(tag: &str, id: &str, topic: &str, connect: Vec<String>) -> (Producer, u16) {
    let root_a = cerulion_core::testing::iceoryx_test_config();
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let clock = Arc::new(VirtualClock::new());
        let clock_dyn: Arc<dyn Clock> = clock.clone();
        let p = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("order_p_{tag}_{id}_{attempt}"),
                clock: clock_dyn,
                ..Default::default()
            },
            root_a.clone(),
        )
        .expect("init P");
        let g = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("order_gprod_{tag}_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    connect_endpoints: connect.clone(),
                    gossip_scouting: true,
                    robot_identity: Some(PROD_ROBOT.to_string()),
                    ..Default::default()
                }),
                ..Default::default()
            },
            root_a.clone(),
        )
        .expect("init G_prod");
        let plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![topic.to_string()],
            ingress: vec![],
        };
        match GatewayRuntime::new(g, plan) {
            Ok(gateway) => {
                let publisher = p
                    .create_publisher_simple(topic, MaxSliceLen::const_new(256))
                    .expect("P publisher");
                return (
                    Producer {
                        _p: p,
                        clock,
                        publisher,
                        gateway,
                    },
                    port,
                );
            }
            Err(e) => eprintln!("attempt {attempt}: G_prod session failed (port {port}): {e}"),
        }
    }
    panic!("could not establish G_prod's listening session in 3 attempts");
}

/// Boot machine B: a LISTENING G_cons ingressing `topic`, connecting to
/// `connect` (empty when B boots first), plus B's local subscriber.
fn boot_consumer(tag: &str, id: &str, topic: &str, connect: Vec<String>) -> (Consumer, u16) {
    let root_b = cerulion_core::testing::iceoryx_test_config();
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("order_gcons_{tag}_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    connect_endpoints: connect.clone(),
                    gossip_scouting: true,
                    robot_identity: Some(CONS_ROBOT.to_string()),
                    ..Default::default()
                }),
                ..Default::default()
            },
            root_b.clone(),
        )
        .expect("init G_cons manager");
        let sub = mgr.create_subscriber(topic).expect("B subscriber");
        let plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowList(vec![]),
            announce: vec![],
            ingress: vec![GatewayIngressEntry {
                topic: topic.to_string(),
                schema_hash: Vector3::SCHEMA_HASH,
            }],
        };
        match GatewayRuntime::new(mgr, plan) {
            Ok(gateway) => return (Consumer { gateway, sub }, port),
            Err(e) => eprintln!("attempt {attempt}: G_cons session failed (port {port}): {e}"),
        }
    }
    panic!("could not establish G_cons's listening session in 3 attempts");
}

/// Boot both machines in `order`, then demand, publish, and collect at B.
fn run(tag: &str, order: SecondToBoot) -> Vec<Received> {
    let id = unique_id();
    let topic = format!("/order/{tag}/{id}");
    let (mut prod, mut cons) = match order {
        SecondToBoot::Producer => {
            let (cons, cons_port) = boot_consumer(tag, &id, &topic, vec![]);
            let (prod, _) =
                boot_producer(tag, &id, &topic, vec![format!("tcp/127.0.0.1:{cons_port}")]);
            (prod, cons)
        }
        SecondToBoot::Consumer => {
            let (prod, prod_port) = boot_producer(tag, &id, &topic, vec![]);
            let (cons, _) =
                boot_consumer(tag, &id, &topic, vec![format!("tcp/127.0.0.1:{prod_port}")]);
            (prod, cons)
        }
    };
    let flag = prod
        .gateway
        .manager()
        .bridge_manager()
        .register_topic(&topic)
        .expect("G_prod bridge flag handle");

    // Demand: the consumer's GET loop and the producer's drive (queryable +
    // reconciler) until the egress flag is ON.
    let deadline = Instant::now() + Duration::from_secs(20);
    while !flag.load(Ordering::Relaxed) {
        assert!(
            Instant::now() < deadline,
            "{tag}: the demand never enabled G_prod's egress"
        );
        cons.gateway.demand_get_once().expect("demand get pass");
        prod.gateway.drive_once().expect("drive prod");
        std::thread::sleep(Duration::from_millis(20));
    }
    let attach_deadline = Instant::now() + Duration::from_secs(5);
    while prod.gateway.active_tap_topics().is_empty() {
        assert!(
            Instant::now() < attach_deadline,
            "{tag}: tap did not attach"
        );
        prod.gateway.drive_once().expect("drive attach");
        std::thread::sleep(Duration::from_millis(10));
    }

    for (ts, (x, y, z)) in PLAN {
        prod.clock.set(ts);
        let mut proxy = prod
            .publisher
            .loan_proxy::<Vector3>()
            .expect("P loan_proxy");
        proxy.x = x;
        proxy.y = y;
        proxy.z = z;
    }

    let mut got: Vec<Received> = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(10);
    while got.len() < PLAN.len() && Instant::now() < deadline {
        cons.gateway.demand_get_once().expect("demand get pass");
        prod.gateway.drive_once().expect("drive forward");
        cons.sub
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
    got
}

#[test]
fn producer_boots_second_and_dials_the_running_consumer() {
    assert_eq!(
        run("prod-second", SecondToBoot::Producer),
        oracle(),
        "B must receive P's frames verbatim when the producer dials the consumer"
    );
}

#[test]
fn consumer_boots_second_and_dials_the_running_producer() {
    assert_eq!(
        run("cons-second", SecondToBoot::Consumer),
        oracle(),
        "B must receive P's frames verbatim when the consumer dials the producer"
    );
}
