// SPDX-License-Identifier: AGPL-3.0-only
//! The CLIENT-driven egress e2e over the PRODUCTION plane + REAL
//! iceoryx2 + zenoh — proof that a desk graph pushing its egress plan into netd (over
//! the `register_egress` UDS verb) actually EGRESSES the topic (the "no inert
//! shipping" rule, composed through the daemon + client rather than the plane
//! directly).
//!
//! Shape (crib of `egress_plane_iox2_test.rs` + `client_e2e_test.rs`): a local
//! producer P + the desk daemon's SHARED network manager G (the ONE session netd
//! owns) live on the SAME per-test SHM root; an in-process `cerulion-netd` daemon is
//! started with the PRODUCTION [`GatewayEgressPlane`] over G. A [`NetdClient`]
//! CONNECTS over the real UDS and `register_egress`es the topic's plan — which drives
//! the daemon → the production plane → boots the embedded gateway + pushes the topic.
//! A remote consumer B (a DISTINCT SHM root reaching G ONLY over a real 127.0.0.1 TCP
//! hop) then demands the topic and reads P's frames BYTE-IDENTICAL to a HAND oracle
//! (each frame recomputed from its OWN wire `sequence` — never a self-compare,
//! Principle #7).
//!
//! This composes `egress_plane_iox2_test` (plane driven directly) THROUGH the client
//! and daemon UDS seam — the "desk run registers with netd and the topic egresses"
//! proof at the netd layer (hermetic, no robot). The graph_cmd decision that a desk
//! run under the flag returns the netd path (no gateway child) is pinned separately in
//! the `cerulion_cli_engine` graph_cmd tests; together they cover the requirement.
//!
//! Parallel-safe (per-test SHM roots + probed ports + isolated scouting-off sessions +
//! a unique UDS socket per test), so NOT `#[serial]`.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{GatewayEgressPolicy, GatewayPlan, SchemaServing};

use cerulion_netd::client::{ClientError, NetdClient};
use cerulion_netd::daemon::{self, NetdConfig, RunningNetd};
use cerulion_netd::egress::{EgressPlane, GatewayEgressPlane};
use cerulion_netd::mirror::{MirrorError, MirrorPlane, MirrorRelease};
use cerulion_netd::registry::TopicKey;

/// The wire schema hash the egress frames carry — B's ingress bridge validates every
/// inbound frame against it before re-injecting (a mismatch is silently dropped), so
/// P's frames + B's `register_ingress_topic` MUST agree.
const EGRESS_HASH: u64 = 0x0837_C5B0_E9E5_0001;

/// A unique id so parallel tests never collide on topic / SHM / socket names.
fn unique_id() -> String {
    static N: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}",
        std::process::id(),
        N.fetch_add(1, Ordering::Relaxed)
    )
}

/// Probe a free ephemeral TCP port on loopback (the gateway binds it; a probe→rebind
/// race is absorbed by the caller's retry).
fn probe_ephemeral_port() -> u16 {
    std::net::TcpListener::bind("127.0.0.1:0")
        .expect("bind ephemeral")
        .local_addr()
        .expect("local_addr")
        .port()
}

/// Hand-build the raw wire frame P publishes for logical sequence `seq` — the oracle.
/// Payload = three little-endian f64s `(seq, 2*seq, 3*seq)`; the header's `sequence`
/// is `seq` and `timestamp_ns` a deterministic function of it.
fn oracle_frame(seq: u32) -> Vec<u8> {
    let mut payload = Vec::with_capacity(24);
    payload.extend_from_slice(&(seq as f64).to_le_bytes());
    payload.extend_from_slice(&(2.0 * seq as f64).to_le_bytes());
    payload.extend_from_slice(&(3.0 * seq as f64).to_le_bytes());
    let header = WireHeader {
        schema_hash: EGRESS_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: 2_000 + seq as u64 * 11,
    };
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

/// A minimal spy mirror plane — never demanded in this egress-only test (ensuring a
/// mirror errs loudly; releasing is a no-op). The daemon requires a mirror plane; this
/// keeps the ingress half inert while the egress half is the real production plane.
struct UnusedMirrorPlane;
impl MirrorPlane for UnusedMirrorPlane {
    fn ensure_mirror(&self, key: &TopicKey, _schema_hash: u64) -> Result<(), MirrorError> {
        Err(MirrorError::Register {
            key: key.clone(),
            source: Box::new(cerulion_core::TransportError::Internal {
                reason: "this egress-only test never demands ingress".to_string(),
            }),
        })
    }
    fn release_mirror(&self, _key: &TopicKey) -> MirrorRelease {
        MirrorRelease::Retired
    }
}

/// Machine A: a network-free producer P + the shared network manager G + an in-process
/// netd daemon whose PRODUCTION egress plane wraps G. A `NetdClient` register_egress's
/// the topic over the real UDS (which boots the embedded gateway, binding G's port).
/// Established with a bounded port retry (the probe→rebind steal race).
struct MachineA {
    producer: Arc<TransportManager>,
    _netd: RunningNetd,
    _client: NetdClient,
    dir: std::path::PathBuf,
    port: u16,
    /// The shared gateway G's serialized iceoryx2 namespace — the
    /// MATCHING forward, and the base a mismatch test mutates the prefix of.
    gateway_config_json: String,
}

fn establish_machine_a(tag: &str, id: &str, topic: &str) -> MachineA {
    let root = iceoryx_test_config();
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let producer = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("cegp_p_{tag}_{id}_{attempt}"),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("init producer P");
        let g = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("cegp_g_{tag}_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    robot_identity: Some("cegpdesk".to_string()),
                    ..Default::default()
                }),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("init shared gateway G");
        // Capture G's namespace BEFORE it moves into the plane — the
        // MATCHING forward (a same-machine run resolves this identical config).
        let gateway_config_json =
            serde_json::to_string(&g.iox_config()).expect("serialize G config");

        // The in-process netd daemon over the PRODUCTION egress plane wrapping G.
        let dir = std::env::temp_dir().join(format!("cer_cegp_{tag}_{id}_{attempt}"));
        std::fs::create_dir_all(&dir).expect("mk tempdir");
        let sock = dir.join("netd.sock");
        // G is ROBOT-shaped (identity + a `tcp/` listen endpoint), so
        // the beacon decides `Advertise` and the production advertiser would put
        // a real, resolvable `_cerulion._tcp` record (`cegpdesk`) on whatever LAN
        // this test runs on — where a concurrent `cerulion topic list` renders it
        // as a live ROBOT and caches a dead ephemeral port in `peers.json`. This
        // file asserts nothing about the beacon, so the suppression is free.
        let egress_plane: Arc<dyn EgressPlane> =
            Arc::new(GatewayEgressPlane::new_without_mdns_for_test(g));
        let mirror_plane: Arc<dyn MirrorPlane> = Arc::new(UnusedMirrorPlane);
        let netd = daemon::start_with_egress(
            sock.clone(),
            mirror_plane,
            egress_plane,
            NetdConfig {
                idle_grace: Duration::from_secs(3600),
                idle_watch_poll: Duration::from_millis(25),
                ..NetdConfig::default()
            },
        )
        .expect("daemon start (production egress plane)");

        // The desk graph's CLIENT: connect over UDS + push the egress plan. This drives
        // the daemon → the production plane → boots the embedded gateway (binds G's
        // port). A bind steal surfaces HERE as a client error → retry a fresh port.
        let mut client = NetdClient::connect_or_spawn_at(sock).expect("connect to netd over UDS");
        let plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![topic.to_string()],
            ingress: vec![],
        };
        // Register config-LESS (the monolith shape) — the shared gateway shares
        // netd's namespace, so there is nothing to verify.
        match client.register_egress(&plan, &SchemaServing::default(), None) {
            Ok(resp) => {
                assert!(
                    resp.gateway_started,
                    "the first egress plan boots the shared gateway"
                );
                assert_eq!(resp.registered_topics, 1);
                return MachineA {
                    producer,
                    _netd: netd,
                    _client: client,
                    dir,
                    port,
                    gateway_config_json,
                };
            }
            Err(e) => {
                eprintln!("attempt {attempt}: client register_egress failed (port {port}): {e}");
                drop(client);
                drop(netd);
                let _ = std::fs::remove_dir_all(&dir);
            }
        }
    }
    panic!("could not register the egress plan + boot the listening gateway in 3 attempts");
}

/// Spawn a background thread on P that publishes `oracle_frame(seq)` for seq=0,1,2,…
/// over `publish_raw` until `stop` flips — the continuous producer the async
/// demand-driven tap needs (the tap has no history).
fn spawn_producer(
    producer: &Arc<TransportManager>,
    topic: &str,
    stop: Arc<AtomicBool>,
) -> JoinHandle<()> {
    let mut pubr = producer
        .create_publisher_simple(topic, MaxSliceLen::const_new(256))
        .expect("P publisher");
    std::thread::spawn(move || {
        let mut seq: u32 = 0;
        while !stop.load(Ordering::Relaxed) {
            let frame = oracle_frame(seq);
            let _ = pubr.publish_raw(&frame);
            seq = seq.wrapping_add(1);
            std::thread::sleep(Duration::from_millis(5));
        }
    })
}

/// Machine B: a DISTINCT root reaching the gateway ONLY over the TCP hop on `port`. B
/// declares the demand token (`register_ingress_topic`) that drives the gateway's
/// egress ON, then collects the re-injected frames and asserts DELIVERY (> 0) +
/// BYTE-IDENTITY to each frame's own-sequence oracle.
fn receive_and_verify_over_hop(port: u16, topic: &str, node_suffix: &str) {
    let b = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("cegp_b_{node_suffix}"),
            network: Some(NetworkConfig {
                connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                ..Default::default()
            }),
            ..Default::default()
        },
        iceoryx_test_config(),
    )
    .expect("init B");
    // Create the reader FIRST (the consumer-first order), then the demand.
    let b_sub = b.create_subscriber(topic).expect("B subscriber");
    b.register_ingress_topic(topic, EGRESS_HASH, MaxSliceLen::const_new(256))
        .expect("B register_ingress_topic (the demand token)");

    const WANT: usize = 5;
    let mut got: Vec<(u32, Vec<u8>)> = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(20);
    while got.len() < WANT && Instant::now() < deadline {
        b_sub
            .try_receive(|msg| {
                let h = msg.header();
                let payload = msg.payload().to_vec();
                let mut full = vec![0u8; WireHeader::SIZE + payload.len()];
                h.write_to_buf(&mut full[..WireHeader::SIZE]);
                full[WireHeader::SIZE..].copy_from_slice(&payload);
                got.push((h.sequence, full));
            })
            .expect("B try_receive");
        if got.len() < WANT {
            std::thread::sleep(Duration::from_millis(20));
        }
    }
    assert!(
        got.len() >= WANT,
        "the client-driven production egress plane forwarded {}/{WANT} frames over zenoh for \
         '{topic}' (got {}); the path is inert if 0",
        got.len(),
        got.len()
    );
    for (seq, frame) in &got {
        assert_eq!(
            frame,
            &oracle_frame(*seq),
            "frame seq {seq} on '{topic}' must arrive byte-identical to its own-sequence oracle"
        );
    }
}

/// The headline pin: a desk graph's `register_egress` over the netd CLIENT + daemon
/// drives the PRODUCTION egress plane so a local producer's frames are observable over
/// the REAL zenoh hop by a remote consumer B, BYTE-IDENTICAL to the hand oracle.
#[test]
fn client_register_egress_forwards_producer_frames_over_zenoh_to_a_remote_consumer() {
    let id = unique_id();
    let topic = format!("/cegp/data/{id}");
    let a = establish_machine_a("fwd", &id, &topic);

    let stop = Arc::new(AtomicBool::new(false));
    let producer_handle = spawn_producer(&a.producer, &topic, Arc::clone(&stop));

    receive_and_verify_over_hop(a.port, &topic, &format!("{id}_b"));

    stop.store(true, Ordering::Relaxed);
    let _ = producer_handle.join();
    // Teardown: dropping A drops the client (releases the egress registration) then the
    // daemon (stops + joins the gateway drive thread — no hang).
    let dir = a.dir.clone();
    drop(a);
    let _ = std::fs::remove_dir_all(&dir);
}

/// The WHOLE chain refuses a MISMATCHING forwarded namespace over the
/// wire — a client `register_egress` carrying a config with a DIFFERENT prefix than
/// netd's shared session surfaces as a LOUD `ClientError::Netd` naming the mismatch.
/// This is the exact signal the CLI's `establish_network_egress` degrades on: a
/// mismatch → child gateway on the run's own namespace (never a silent no-egress).
#[test]
fn client_register_egress_with_a_mismatching_config_is_refused_over_the_wire() {
    let id = unique_id();
    let topic = format!("/cegp/cfgmis/{id}");
    // Boots the shared gateway via the first client (config-less).
    let a = establish_machine_a("cfgmis", &id, &topic);

    // A SECOND client forwarding a DIFFERENT namespace (flip only the prefix on the
    // real config JSON — FileName serializes as a plain string, so it re-parses).
    let mut v: serde_json::Value = serde_json::from_str(&a.gateway_config_json).unwrap();
    v["global"]["prefix"] = serde_json::json!("cer_wiremis_");
    let mismatch = serde_json::to_string(&v).unwrap();

    let sock = a.dir.join("netd.sock");
    let mut client2 = NetdClient::connect_or_spawn_at(sock).expect("second client connects");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![format!("/cegp/cfgmis2/{id}")],
        ingress: vec![],
    };
    let err = client2
        .register_egress(&plan, &SchemaServing::default(), Some(&mismatch))
        .expect_err("a mismatching forwarded config is refused over the wire");
    match err {
        ClientError::Netd { error, .. } => {
            assert!(
                error.contains("does NOT match"),
                "the LOUD namespace-mismatch refusal crosses the wire to the client: {error}"
            );
            assert!(
                error.contains("cer_wiremis_"),
                "the refusal names the run's forwarded prefix: {error}"
            );
        }
        other => panic!("expected ClientError::Netd (a refusal), got {other:?}"),
    }

    let dir = a.dir.clone();
    drop(client2);
    drop(a);
    let _ = std::fs::remove_dir_all(&dir);
}

/// The WHOLE chain refuses a run whose forwarded
/// config shares netd's `(root_path, prefix)` but resolves a DIFFERENT
/// `service.directory` — a `(root_path, prefix)`-only identity would wrongly
/// MATCH here, so netd would tap its own empty service directory and the run
/// would silently egress nothing. The extended discovery identity (incl. the
/// service directory) makes the refusal cross the wire as a LOUD `ClientError::Netd`.
#[test]
fn client_register_egress_with_a_divergent_service_directory_is_refused_over_the_wire() {
    let id = unique_id();
    let topic = format!("/cegp/svcdirmis/{id}");
    // Boots the shared gateway via the first client (config-less).
    let a = establish_machine_a("svcdirmis", &id, &topic);

    // A SECOND client forwarding a config with (root_path, prefix) IDENTICAL to
    // netd's but a DIFFERENT service directory (Path serializes as a plain string).
    // The sanity assert on the default value ALSO validates the JSON key path.
    let mut v: serde_json::Value = serde_json::from_str(&a.gateway_config_json).unwrap();
    assert_eq!(
        v["global"]["service"]["directory"], "services",
        "sanity: default service directory (JSON key path)"
    );
    v["global"]["service"]["directory"] = serde_json::json!("cer_wire_other_services");
    let mismatch = serde_json::to_string(&v).unwrap();

    let sock = a.dir.join("netd.sock");
    let mut client2 = NetdClient::connect_or_spawn_at(sock).expect("second client connects");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![format!("/cegp/svcdirmis2/{id}")],
        ingress: vec![],
    };
    let err = client2
        .register_egress(&plan, &SchemaServing::default(), Some(&mismatch))
        .expect_err("a divergent service.directory is refused over the wire");
    match err {
        ClientError::Netd { error, .. } => {
            assert!(
                error.contains("does NOT match"),
                "the namespace-mismatch refusal crosses the wire: {error}"
            );
            assert!(
                error.contains("cer_wire_other_services"),
                "the refusal names the run's divergent service directory: {error}"
            );
        }
        other => panic!("expected ClientError::Netd (a refusal), got {other:?}"),
    }

    let dir = a.dir.clone();
    drop(client2);
    drop(a);
    let _ = std::fs::remove_dir_all(&dir);
}
