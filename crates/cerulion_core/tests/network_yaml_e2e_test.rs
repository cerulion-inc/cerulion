// SPDX-License-Identifier: AGPL-3.0-only
//! THE acceptance arm — config-only cross-machine delivery.
//!
//! "A user enables cross-machine delivery by adding `network:` blocks to two
//! graph YAMLs — no code." Two graphs are built PURELY from YAML strings:
//! graph A (the "robot": an egress producer, `listen` on a bind-probed
//! 127.0.0.1 port) and graph B (the "workstation": an ingress consumer,
//! `connect` to A's port). The ONLY network inputs are the two `network:`
//! blocks — each side's transport `NetworkConfig` is minted FROM its parsed
//! block via `GraphConfig::network_transport_config()` (exactly what the CLI's
//! `resolve_run_network` does), and all wiring (allow-list, egress watch,
//! ingress bridge) happens inside `GraphRuntime::build`.
//!
//! ```text
//! A: src (external) ──topic:──▶ /nwe2e/cloud/… ──egress──▶ zenoh TCP
//!                                                            │ (B's ingress bridge)
//! B: sink (#[input(trigger)]) ◀── local iceoryx2 ◀── re-inject verbatim
//! ```
//!
//! B's consumer must observe A's EXACT f64 payload (bit-for-bit, hand
//! oracle) — never a self-compare. The deeper full-wire-frame byte identity
//! (header + sequence + timestamp) is `network_ingress_e2e_test`'s job;
//! THIS file pins the yaml-only user story end-to-end through two REAL
//! `GraphRuntime`s.
//!
//! Port-reuse race + retry conventions cribbed from
//! `network_ingress_e2e_test.rs` (bind-probe an ephemeral port, drop the
//! probe listener, bounded 3-attempt rebuild on a stolen port — A's zenoh
//! bind failure surfaces as a `GraphRuntime::build` error through the
//! network hook's `start_network_bridge_watch`).
//!
//! NOT `#[serial]`: distinct per-test SHM roots (`init_for_test`), freshly
//! probed ports, unique topics.

use std::net::TcpListener;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::node::{NodeEntry, NodeInfo};
use cerulion_core::graph::{compute_gateway_plan, parse_graph_raw, GraphRuntime, GraphTopology};
use cerulion_core::prelude::*;
use cerulion_core::transport::gateway::GatewayRuntime;
use cerulion_core::transport::{NetworkPosture, TransportConfig, TransportManager};
use indexmap::IndexMap;
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

/// Bind-probe an ephemeral TCP port (crib: `network_ingress_e2e_test.rs` —
/// the probe→rebind window is absorbed by the caller's bounded retry).
fn probe_ephemeral_port() -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind ephemeral probe");
    let port = listener.local_addr().expect("probe local_addr").port();
    drop(listener);
    port
}

/// The hand oracle payload A publishes and B must observe bit-for-bit.
/// All three values are exact in f64 (no rounding ambiguity).
const ORACLE: (f64, f64, f64) = (42.5, -7.25, 0.001953125);

/// A sentinel bit-pattern no oracle field carries (f64::NAN bits) — "sink
/// has not observed anything yet".
const UNSEEN: u64 = u64::MAX;

// ===========================================================================
// Node types
// ===========================================================================

/// Graph A's producer: external (HostDriven) so the harness fires it
/// deterministically via `trigger_external` + `step`. Every fire publishes
/// the SAME oracle payload (fresh wire sequence each time — B asserts the
/// payload bits, which are fire-count-independent).
#[cerulion_node(external)]
#[derive(Default)]
struct CloudSource {
    #[output]
    cloud: Vector3,
}

#[cerulion_node_impl]
impl CloudSource {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cloud.x = ORACLE.0;
        self.cloud.y = ORACLE.1;
        self.cloud.z = ORACLE.2;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// Graph B's consumer: fires on each delivered frame and records the
/// observed f64s as raw bits into shared atomics (bit-for-bit observable
/// from the harness).
#[cerulion_node]
#[derive(Default)]
struct CloudSink {
    #[input(trigger)]
    cloud: Vector3,
    seen_x: Arc<AtomicU64>,
    seen_y: Arc<AtomicU64>,
    seen_z: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl CloudSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen_x.store(self.cloud.x.to_bits(), Ordering::Relaxed);
        self.seen_y.store(self.cloud.y.to_bits(), Ordering::Relaxed);
        self.seen_z.store(self.cloud.z.to_bits(), Ordering::Relaxed);
        Ok(())
    }
}

// ===========================================================================
// The acceptance test
// ===========================================================================

/// Config-only cross-machine delivery: two YAML strings, two
/// `GraphRuntime::build`s, one bit-identical payload at B's consumer.
#[test]
fn yaml_network_blocks_deliver_across_two_graphs() {
    let id = unique_id();
    let topic = format!("/nwe2e/cloud/{id}");

    // ---- Graph A (egress, listens). The graph build is network-free;
    // A's egress is a GATEWAY computed from A's parsed `network:` block (the
    // CLI-analog: compute_gateway_plan → GatewayRuntime). Bounded retry: a stolen
    // probed port fails the GATEWAY's zenoh bind (its watch start opens the
    // listen session) — retry with a fresh port + manager.
    let mut a_setup: Option<(GraphRuntime, GatewayRuntime, Arc<TransportManager>, u16)> = None;
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let yaml_a = format!(
            r#"
name: nwe2e_robot
prefix: nwe2ea
nodes:
  - id: src
    type: cloud_source
    outputs:
      - name: cloud
        schema: geometry_msgs/Vector3
        topic: {topic}
network:
  mode: peer
  listen:
    - tcp/127.0.0.1:{port}
  egress:
    - {topic}
"#
        );
        let config_a = parse_graph_raw(&yaml_a).expect("graph A YAML parses");
        // THE config-only step: the transport's network comes FROM the
        // parsed block (what the CLI's resolve_run_network does).
        let net_a = config_a
            .network_transport_config()
            .expect("graph A declares an enabled network block");
        let clock_a = Arc::new(VirtualClock::new());
        let mgr_a = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("nwe2e_a_{id}_{attempt}"),
                clock: clock_a.clone(),
                subscriber_buffer_size: 16,
                network: Some(net_a),
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .expect("init manager A");
        let mut factories_a: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        factories_a.insert("src".to_string(), Box::new(CloudSourceEntry::new()));
        // Compute A's gateway plan from the parsed YAML block BEFORE the build
        // moves config/factories (the CLI-analog wiring).
        let infos_a: IndexMap<String, NodeInfo> = factories_a
            .iter()
            .map(|(k, e)| (k.clone(), e.info().expect("node info")))
            .collect();
        let topology_a = GraphTopology::build(&config_a, &infos_a).expect("topology A");
        let plan_a = compute_gateway_plan(&config_a, &topology_a, &infos_a, NetworkPosture::Strict)
            .expect("plan A computes")
            .expect("graph A declares an enabled network block");
        let rt = GraphRuntime::build(config_a, factories_a, &mgr_a, clock_a)
            .expect("graph A build (network-free)");
        // The gateway's watch start opens A's listen session — a stolen port
        // fails HERE.
        match GatewayRuntime::new(Arc::clone(&mgr_a), plan_a) {
            Ok(gateway) => {
                a_setup = Some((rt, gateway, mgr_a, port));
                break;
            }
            Err(e) => {
                eprintln!(
                    "attempt {attempt}: graph A gateway boot failed (probed port {port} likely \
                     stolen in the probe->rebind window): {e}"
                );
            }
        }
    }
    let Some((mut rt_a, mut gateway_a, mgr_a, port)) = a_setup else {
        panic!("could not boot graph A's gateway listener in 3 attempts");
    };

    // ---- Graph B (ingress, connects) — distinct SHM root, its network
    // minted from ITS yaml block.
    let yaml_b = format!(
        r#"
name: nwe2e_workstation
prefix: nwe2eb
nodes:
  - id: sink
    type: cloud_sink
    inputs:
      - name: cloud
        source: {topic}
network:
  mode: peer
  connect:
    - tcp/127.0.0.1:{port}
  ingress:
    - {topic}
"#
    );
    let config_b = parse_graph_raw(&yaml_b).expect("graph B YAML parses");
    let net_b = config_b
        .network_transport_config()
        .expect("graph B declares an enabled network block");
    let clock_b = Arc::new(VirtualClock::new());
    let mgr_b = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("nwe2e_b_{id}"),
            clock: clock_b.clone(),
            subscriber_buffer_size: 16,
            network: Some(net_b),
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init manager B");
    let seen_x = Arc::new(AtomicU64::new(UNSEEN));
    let seen_y = Arc::new(AtomicU64::new(UNSEEN));
    let seen_z = Arc::new(AtomicU64::new(UNSEEN));
    let sink = CloudSink {
        seen_x: Arc::clone(&seen_x),
        seen_y: Arc::clone(&seen_y),
        seen_z: Arc::clone(&seen_z),
        ..Default::default()
    };
    let mut factories_b: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories_b.insert(
        "sink".to_string(),
        Box::new(CloudSinkEntry::with_state(sink)),
    );
    // Compute B's gateway plan from ITS parsed block BEFORE the build moves
    // config/factories (the CLI-analog wiring): the `ingress:` entry carries the
    // expected schema hash resolved from the sink's macro InputMeta.
    let infos_b: IndexMap<String, NodeInfo> = factories_b
        .iter()
        .map(|(k, e)| (k.clone(), e.info().expect("node info")))
        .collect();
    let topology_b = GraphTopology::build(&config_b, &infos_b).expect("topology B");
    let plan_b = compute_gateway_plan(&config_b, &topology_b, &infos_b, NetworkPosture::Strict)
        .expect("plan B computes")
        .expect("graph B declares an enabled network block");
    // The graph build is network-free — B's ingress lives in ITS
    // gateway, booted below.
    let mut rt_b = GraphRuntime::build(config_b, factories_b, &mgr_b, clock_b)
        .expect("graph B build (network-free)");
    // Boot B's GATEWAY after the build (the sink's subscriber has opened the
    // topic's service, so the ingress re-injection publisher attaches to it):
    // this registers the network→local ingress bridge AND declares the DEMAND
    // TopicToken that flips A's egress flag — `register_ingress` declares the
    // data subscriber BEFORE the token (inside `register_ingress_topic`), so
    // A's first forwarded frame routes here (no first-frame race). Ingress
    // needs no drive loop (re-injection rides the zenoh callback), but the
    // handle must stay alive for the test's duration.
    let _gateway_b = GatewayRuntime::new(Arc::clone(&mgr_b), plan_b)
        .expect("graph B gateway boot (its ingress bridge connects to A's live listener)");

    // ---- Liveliness handshake: B's ingress token must flip A's egress flag
    // (through the allow-list — the topic IS declared under A's `egress:`).
    // `register_topic` is idempotent: this handle IS the flag A's gateway reads.
    let flag = mgr_a
        .bridge_manager()
        .register_topic(&topic)
        .expect("A bridge flag handle");
    let deadline = Instant::now() + Duration::from_secs(10);
    while !flag.load(Ordering::Relaxed) {
        assert!(
            Instant::now() < deadline,
            "liveliness handshake did not flip A's bridge flag within 10s — \
             B's ingress TopicToken never reached A's gateway watcher"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    // Attach A's egress tap BEFORE the producer publishes (the tap has no
    // history — it only forwards frames published after it attaches).
    let attach_deadline = Instant::now() + Duration::from_secs(5);
    while gateway_a.active_tap_topics().is_empty() {
        assert!(
            Instant::now() < attach_deadline,
            "A's gateway tap did not attach"
        );
        gateway_a.drive_once().expect("drive A's gateway");
        std::thread::sleep(Duration::from_millis(10));
    }

    // ---- Drive until B's consumer observes the oracle bit-for-bit (bounded).
    // Each iteration: fire A's external producer (one SHM publish), drive A's
    // gateway (tap → zenoh forward), then step B (drains the re-injected frame
    // + fires the data-trigger sink).
    const STEP: Duration = Duration::from_millis(1);
    let oracle_bits = (ORACLE.0.to_bits(), ORACLE.1.to_bits(), ORACLE.2.to_bits());
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut observed = (UNSEEN, UNSEEN, UNSEEN);
    while Instant::now() < deadline {
        rt_a.trigger_external("src").expect("trigger A's producer");
        rt_a.step(STEP);
        gateway_a.drive_once().expect("drive A's gateway forward");
        rt_b.step(STEP);
        observed = (
            seen_x.load(Ordering::Relaxed),
            seen_y.load(Ordering::Relaxed),
            seen_z.load(Ordering::Relaxed),
        );
        if observed == oracle_bits {
            break;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    assert_eq!(
        observed, oracle_bits,
        "B's consumer must observe A's payload BIT-FOR-BIT ({:?}); config-only \
         cross-graph delivery failed within the bounded window",
        ORACLE
    );
}

// ===========================================================================
// A topic published by a process OUTSIDE the graph (`external: true`)
// ===========================================================================

/// The iceoryx2 config file (JSON) the outside publisher child opens, so it
/// shares graph A's shared memory root.
const ENV_OUTSIDE_IOX2_CONFIG: &str = "CER_NWE2E_OUTSIDE_IOX2_CONFIG";
/// The comma-separated topics the outside publisher child registers and publishes.
const ENV_OUTSIDE_TOPICS: &str = "CER_NWE2E_OUTSIDE_TOPICS";
/// Stop-file path: the child publishes until it appears, then exits 0.
const ENV_OUTSIDE_STOP: &str = "CER_NWE2E_OUTSIDE_STOP";

/// Child entrypoint, re-invoked as a subprocess via `current_exe`: the
/// outside publisher. It does what an `rmw_cerulion` publisher does on this
/// machine (registers the topic over the gateway control channel with its
/// schema hash, then publishes on the topic's shared memory service) for every
/// topic in [`ENV_OUTSIDE_TOPICS`], publishing [`ORACLE`] on each every 20 ms
/// with an advancing timestamp. In a normal suite run the env is absent and
/// this is a no-op pass.
///
/// Exits through `process::exit` with a status the parent reads: 0 when the
/// stop file appeared, 2 on any failure.
#[allow(clippy::disallowed_methods)]
#[test]
fn outside_publisher_child_entrypoint() {
    let Ok(config_path) = std::env::var(ENV_OUTSIDE_IOX2_CONFIG) else {
        return; // not the child invocation
    };
    let run = || -> Result<(), Box<dyn std::error::Error>> {
        let topics = std::env::var(ENV_OUTSIDE_TOPICS)?;
        let stop = std::env::var(ENV_OUTSIDE_STOP)?;
        let ix: iceoryx2::config::Config =
            serde_json::from_str(&std::fs::read_to_string(&config_path)?)?;
        let clock = Arc::new(VirtualClock::new());
        let mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("nwe2e_outside_{}", unique_id()),
                clock: clock.clone(),
                ..Default::default()
            },
            ix,
        )?;
        let mut publishers = Vec::new();
        for topic in topics.split(',') {
            let publisher = mgr.create_publisher_typed::<Vector3>(topic, None)?;
            let _newly = mgr.register_dynamic_egress_topic(
                topic,
                <Vector3 as cerulion_core::message::ShmMessage>::SCHEMA_HASH,
            )?;
            publishers.push(publisher);
        }
        // Bounded: about 60 s, then fail loudly.
        for step in 1..=3000u64 {
            if std::path::Path::new(&stop).exists() {
                return Ok(());
            }
            clock.set(step * 1_000_000);
            for publisher in &mut publishers {
                let mut proxy = publisher.loan_proxy::<Vector3>()?;
                proxy.x = ORACLE.0;
                proxy.y = ORACLE.1;
                proxy.z = ORACLE.2;
            }
            std::thread::sleep(Duration::from_millis(20));
        }
        Err("stop file never appeared within the bound".into())
    };
    match run() {
        Ok(()) => std::process::exit(0),
        Err(e) => {
            eprintln!("[nwe2e outside publisher] FAILED: {e}");
            std::process::exit(2);
        }
    }
}

/// Kills and reaps the outside publisher child on drop, so a parent panic
/// never leaves it publishing.
struct OutsideChildGuard {
    child: std::process::Child,
    reaped: bool,
}

impl Drop for OutsideChildGuard {
    fn drop(&mut self) {
        if !self.reaped {
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
    }
}

/// Two processes, one Strict `network:` block: graph A lists a topic that a
/// SEPARATE OS process publishes (the outside publisher child, standing in for
/// a ROS 2 program on `rmw_cerulion`) as `external: true`, and graph B on a
/// distinct shared memory root ingresses it over a 127.0.0.1 TCP hop. B's
/// consumer observes the child's payload bit-for-bit. The child also
/// publishes and registers a second topic A does NOT list; B ingresses that
/// one too, and A's bridge never enables it, forwards none of its frames, and
/// B's second consumer never fires (only listed topics leave).
#[test]
fn yaml_network_external_egress_carries_an_outside_publishers_topic() {
    let id = unique_id();
    let own = format!("/nwe2e/outside/{id}/own");
    let listed = format!("/nwe2e/outside/{id}/chatter");
    let unlisted = format!("/nwe2e/outside/{id}/unlisted");

    // ---- Graph A: one in-graph producer of its own topic, and the outside
    // publisher's topic listed under `egress:` as `external: true`.
    let ix_a = cerulion_core::testing::iceoryx_test_config();
    let mut a_setup: Option<(GraphRuntime, GatewayRuntime, Arc<TransportManager>, u16)> = None;
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let yaml_a = format!(
            r#"
name: nwe2e_outside_robot
prefix: nwe2eoa
nodes:
  - id: src
    type: cloud_source
    outputs:
      - name: cloud
        schema: geometry_msgs/Vector3
        topic: {own}
network:
  mode: peer
  listen:
    - tcp/127.0.0.1:{port}
  egress:
    - {own}
    - topic: {listed}
      external: true
"#
        );
        let config_a = parse_graph_raw(&yaml_a).expect("graph A YAML parses");
        cerulion_core::graph::validate_graph(&config_a)
            .expect("an external egress topic needs no in-graph producer");
        let net_a = config_a
            .network_transport_config()
            .expect("graph A declares an enabled network block");
        let clock_a = Arc::new(VirtualClock::new());
        let mgr_a = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("nwe2e_oa_{id}_{attempt}"),
                clock: clock_a.clone(),
                subscriber_buffer_size: 16,
                network: Some(net_a),
            },
            ix_a.clone(),
        )
        .expect("init manager A");
        let mut factories_a: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        factories_a.insert("src".to_string(), Box::new(CloudSourceEntry::new()));
        let infos_a: IndexMap<String, NodeInfo> = factories_a
            .iter()
            .map(|(k, e)| (k.clone(), e.info().expect("node info")))
            .collect();
        let topology_a = GraphTopology::build(&config_a, &infos_a).expect("topology A");
        let plan_a = compute_gateway_plan(&config_a, &topology_a, &infos_a, NetworkPosture::Strict)
            .expect("plan A computes")
            .expect("graph A declares an enabled network block");
        assert!(
            plan_a.announce.contains(&listed) && !plan_a.announce.contains(&unlisted),
            "A's plan announces the external topic and not the unlisted one: {:?}",
            plan_a.announce
        );
        let rt = GraphRuntime::build(config_a, factories_a, &mgr_a, clock_a)
            .expect("graph A build (network-free)");
        match GatewayRuntime::new(Arc::clone(&mgr_a), plan_a) {
            Ok(gateway) => {
                a_setup = Some((rt, gateway, mgr_a, port));
                break;
            }
            Err(e) => {
                eprintln!(
                    "attempt {attempt}: graph A gateway boot failed (probed port {port} likely \
                     stolen in the probe->rebind window): {e}"
                );
            }
        }
    }
    let Some((_rt_a, mut gateway_a, mgr_a, port)) = a_setup else {
        panic!("could not boot graph A's gateway listener in 3 attempts");
    };

    // ---- The outside publisher: a separate OS process on A's root.
    let dir = std::env::temp_dir().join(format!("cer_nwe2e_outside_{id}"));
    std::fs::create_dir_all(&dir).expect("make temp dir");
    let config_path = dir.join("ix_config.json");
    std::fs::write(
        &config_path,
        serde_json::to_string(&ix_a).expect("serialize iceoryx2 config"),
    )
    .expect("write iceoryx2 config");
    let stop_path = dir.join("stop");
    let mut child = OutsideChildGuard {
        child: std::process::Command::new(std::env::current_exe().expect("current_exe"))
            .args([
                "--exact",
                "outside_publisher_child_entrypoint",
                "--nocapture",
            ])
            .env(ENV_OUTSIDE_IOX2_CONFIG, &config_path)
            .env(ENV_OUTSIDE_TOPICS, format!("{listed},{unlisted}"))
            .env(ENV_OUTSIDE_STOP, &stop_path)
            .spawn()
            .expect("spawn the outside publisher"),
        reaped: false,
    };

    // Wait until A's gateway has registered the unlisted topic from the
    // child's control-channel record (the topic is not on A's egress list).
    let deadline = Instant::now() + Duration::from_secs(20);
    while !gateway_a
        .is_runtime_registered(&unlisted)
        .expect("is_runtime_registered")
    {
        assert!(
            Instant::now() < deadline,
            "the outside publisher's registration of {unlisted} never reached A's gateway"
        );
        gateway_a.drive_once().expect("drive A's gateway");
        std::thread::sleep(Duration::from_millis(20));
    }

    // ---- Graph B: ingresses BOTH topics, one consumer each.
    let yaml_b = format!(
        r#"
name: nwe2e_outside_workstation
prefix: nwe2eob
nodes:
  - id: sink_listed
    type: cloud_sink
    inputs:
      - name: cloud
        source: {listed}
  - id: sink_unlisted
    type: cloud_sink
    inputs:
      - name: cloud
        source: {unlisted}
network:
  mode: peer
  connect:
    - tcp/127.0.0.1:{port}
  ingress:
    - {listed}
    - {unlisted}
"#
    );
    let config_b = parse_graph_raw(&yaml_b).expect("graph B YAML parses");
    let net_b = config_b
        .network_transport_config()
        .expect("graph B declares an enabled network block");
    let clock_b = Arc::new(VirtualClock::new());
    let mgr_b = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("nwe2e_ob_{id}"),
            clock: clock_b.clone(),
            subscriber_buffer_size: 16,
            network: Some(net_b),
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init manager B");
    let seen = |sink: &CloudSink| {
        (
            sink.seen_x.load(Ordering::Relaxed),
            sink.seen_y.load(Ordering::Relaxed),
            sink.seen_z.load(Ordering::Relaxed),
        )
    };
    let new_sink = || CloudSink {
        seen_x: Arc::new(AtomicU64::new(UNSEEN)),
        seen_y: Arc::new(AtomicU64::new(UNSEEN)),
        seen_z: Arc::new(AtomicU64::new(UNSEEN)),
        ..Default::default()
    };
    let probe_listed = new_sink();
    let probe_unlisted = new_sink();
    let mut factories_b: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for (node, probe) in [
        ("sink_listed", &probe_listed),
        ("sink_unlisted", &probe_unlisted),
    ] {
        let sink = CloudSink {
            seen_x: Arc::clone(&probe.seen_x),
            seen_y: Arc::clone(&probe.seen_y),
            seen_z: Arc::clone(&probe.seen_z),
            ..Default::default()
        };
        factories_b.insert(node.to_string(), Box::new(CloudSinkEntry::with_state(sink)));
    }
    let infos_b: IndexMap<String, NodeInfo> = factories_b
        .iter()
        .map(|(k, e)| (k.clone(), e.info().expect("node info")))
        .collect();
    let topology_b = GraphTopology::build(&config_b, &infos_b).expect("topology B");
    let plan_b = compute_gateway_plan(&config_b, &topology_b, &infos_b, NetworkPosture::Strict)
        .expect("plan B computes")
        .expect("graph B declares an enabled network block");
    let mut rt_b = GraphRuntime::build(config_b, factories_b, &mgr_b, clock_b)
        .expect("graph B build (network-free)");
    let _gateway_b = GatewayRuntime::new(Arc::clone(&mgr_b), plan_b)
        .expect("graph B gateway boot (its ingress bridge connects to A's live listener)");

    // ---- B's demand for the listed topic flips A's flag through the
    // allow-list; drive A until its tap on the child's service attaches.
    let flag = mgr_a
        .bridge_manager()
        .register_topic(&listed)
        .expect("A bridge flag handle");
    let deadline = Instant::now() + Duration::from_secs(10);
    while !flag.load(Ordering::Relaxed) {
        assert!(
            Instant::now() < deadline,
            "B's demand for {listed} never flipped A's bridge flag within 10s"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    let deadline = Instant::now() + Duration::from_secs(10);
    while !gateway_a.active_tap_topics().contains(&listed) {
        assert!(
            Instant::now() < deadline,
            "A's gateway tap on the outside publisher's service did not attach"
        );
        gateway_a.drive_once().expect("drive A's gateway");
        std::thread::sleep(Duration::from_millis(10));
    }

    // ---- Drive A's gateway and step B until the listed consumer observes the
    // child's payload bit-for-bit (bounded).
    const STEP: Duration = Duration::from_millis(1);
    let oracle_bits = (ORACLE.0.to_bits(), ORACLE.1.to_bits(), ORACLE.2.to_bits());
    let deadline = Instant::now() + Duration::from_secs(10);
    while Instant::now() < deadline && seen(&probe_listed) != oracle_bits {
        gateway_a.drive_once().expect("drive A's gateway forward");
        rt_b.step(STEP);
        std::thread::sleep(Duration::from_millis(10));
    }
    assert_eq!(
        seen(&probe_listed),
        oracle_bits,
        "B's consumer of the external egress topic must observe the outside publisher's \
         payload bit-for-bit ({ORACLE:?})"
    );
    assert!(
        gateway_a.forwarded_count(&listed) > 0,
        "A's gateway forwarded the listed topic's frames"
    );

    // ---- Strict scope holds. B's demand for the unlisted topic reaches A and
    // A's bridge refuses it; then both sides run one more second and the
    // unlisted topic never egresses.
    let deadline = Instant::now() + Duration::from_secs(10);
    while !mgr_a
        .bridge_manager()
        .was_egress_refused(&unlisted)
        .expect("egress gate readable")
    {
        assert!(
            Instant::now() < deadline,
            "B's demand for {unlisted} never reached A's bridge scoping filter within 10s"
        );
        gateway_a.drive_once().expect("drive A's gateway");
        std::thread::sleep(Duration::from_millis(20));
    }
    assert!(
        !mgr_a
            .bridge_manager()
            .was_egress_refused(&listed)
            .expect("egress gate readable"),
        "the listed external topic is never refused"
    );
    let settle = Instant::now() + Duration::from_secs(1);
    while Instant::now() < settle {
        gateway_a.drive_once().expect("drive A's gateway");
        rt_b.step(STEP);
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(
        !mgr_a
            .bridge_manager()
            .is_enabled(&unlisted)
            .expect("is_enabled unlisted"),
        "the unlisted topic's bridge must never enable under the Strict block"
    );
    assert_eq!(
        gateway_a.forwarded_count(&unlisted),
        0,
        "no frame of the unlisted topic leaves A"
    );
    assert_eq!(
        seen(&probe_unlisted),
        (UNSEEN, UNSEEN, UNSEEN),
        "B's consumer of the unlisted topic never fires"
    );

    // ---- Release the child; it exits 0 (registered and published throughout).
    std::fs::write(&stop_path, b"done").expect("write stop file");
    let status = child.child.wait().expect("wait for the outside publisher");
    child.reaped = true;
    assert!(status.success(), "outside publisher exited {status:?}");
    let _ = std::fs::remove_dir_all(&dir);
}
