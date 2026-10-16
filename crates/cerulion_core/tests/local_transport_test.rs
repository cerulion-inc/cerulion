// SPDX-License-Identifier: AGPL-3.0-only
//! Local-transport integration coverage.
//!
//! Run with:
//! `cargo build -p test_node_macro_cdylib -p test_node_cdylib`, then
//! `cargo test -p cerulion_core --features local-transport --test local_transport_test -- --test-threads=1`
//!
//! The fixtures are built WITHOUT the feature, so the loader must refuse them.
//!
//! Device role prerequisites: set `HOME`, `TMPDIR`, and `CERULION_SHM_DIR`
//! beneath `/data/local/tmp` before running `serve_role` on Android.

#![cfg(feature = "local-transport")]

use std::net::TcpListener;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Barrier};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::gateway::{
    GatewayEgressPolicy, GatewayPlan, GatewayRuntime, SchemaHashName, SchemaServing, TopicSchema,
};
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{TransportConfig, TransportKind, TransportManager, TRANSPORT_KIND};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::TransportError;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use tracing_test::traced_test;

static COUNTER: AtomicU64 = AtomicU64::new(0);

const PLAN: [(u64, (f64, f64, f64)); 3] = [
    (1_000_000, (1.5, -2.5, 3.5)),
    (2_000_000, (4.5, 5.5, -6.5)),
    (3_000_000, (7.5, 8.5, 9.5)),
];

fn unique_id() -> String {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    let count = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{nanos}_{count}")
}

fn vector3_payload(x: f64, y: f64, z: f64) -> Vec<u8> {
    let mut payload = Vec::with_capacity(24);
    payload.extend_from_slice(&x.to_le_bytes());
    payload.extend_from_slice(&y.to_le_bytes());
    payload.extend_from_slice(&z.to_le_bytes());
    payload
}

fn oracle_frame(seq: u32, timestamp_ns: u64, payload: &[u8]) -> Vec<u8> {
    let size = (WireHeader::SIZE + payload.len()) as u32;
    let header = WireHeader {
        schema_hash: Vector3::SCHEMA_HASH,
        total_size: size,
        offset_table_offset: size,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns,
    };
    let mut frame = vec![0; size as usize];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(payload);
    frame
}

fn oracle_frames() -> Vec<Vec<u8>> {
    PLAN.iter()
        .enumerate()
        .map(|(index, (timestamp_ns, (x, y, z)))| {
            oracle_frame(index as u32, *timestamp_ns, &vector3_payload(*x, *y, *z))
        })
        .collect()
}

fn received_frame(message: cerulion_core::ReceivedMessage<'_>) -> Vec<u8> {
    let header = message.header();
    let payload = message.payload();
    let mut frame = vec![0; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(payload);
    frame
}

fn publish_one(
    publisher: &mut cerulion_core::CerulionPublisher,
    clock: &VirtualClock,
    timestamp_ns: u64,
    (x, y, z): (f64, f64, f64),
) {
    clock.set(timestamp_ns);
    let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan Vector3");
    proxy.x = x;
    proxy.y = y;
    proxy.z = z;
}

#[test]
fn transport_kind_is_in_process() {
    assert_eq!(TRANSPORT_KIND, TransportKind::InProcess);
}

fn run_thread_delivery(tag: &str) -> Vec<Vec<u8>> {
    let id = unique_id();
    let topic = format!("/local_transport/{tag}/{id}");
    let clock = Arc::new(VirtualClock::new());
    let clock_dyn: Arc<dyn Clock> = clock.clone();
    let transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("local_delivery_{tag}_{id}"),
            clock: clock_dyn,
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init local transport");
    let mut publisher = transport
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let subscriber = transport
        .create_subscriber(&topic)
        .expect("create subscriber");
    let start = Arc::new(Barrier::new(2));

    let subscriber_start = start.clone();
    let subscriber_thread = thread::spawn(move || {
        subscriber_start.wait();
        let mut frames = Vec::with_capacity(PLAN.len());
        let deadline = Instant::now() + Duration::from_secs(5);
        while frames.len() < PLAN.len() && Instant::now() < deadline {
            subscriber
                .try_receive(|message| frames.push(received_frame(message)))
                .expect("receive local frames");
            if frames.len() < PLAN.len() {
                thread::sleep(Duration::from_millis(2));
            }
        }
        frames
    });

    let publisher_start = start;
    let publisher_clock = clock.clone();
    let publisher_thread = thread::spawn(move || {
        publisher_start.wait();
        for (timestamp_ns, values) in PLAN {
            publish_one(&mut publisher, &publisher_clock, timestamp_ns, values);
            thread::sleep(Duration::from_millis(5));
        }
    });
    publisher_thread.join().expect("publisher thread");
    subscriber_thread.join().expect("subscriber thread")
}

#[test]
fn local_delivery_between_threads_matches_oracle_deterministically() {
    let expected = oracle_frames();
    let first = run_thread_delivery("first");
    let second = run_thread_delivery("second");

    assert_eq!(first, expected, "first run must deliver every oracle frame");
    assert_eq!(
        second, expected,
        "second run must deliver every oracle frame"
    );
    assert_eq!(first, second, "repeated runs must be byte-identical");
}

fn probe_ephemeral_port() -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind port probe");
    let port = listener.local_addr().expect("probe local address").port();
    drop(listener);
    port
}

fn run_gateway_delivery() -> (Vec<Vec<u8>>, u64) {
    let id = unique_id();
    let topic = format!("/local_transport/gateway/{id}");
    let root_a = cerulion_core::testing::iceoryx_test_config();
    let mut setup = None;

    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let clock = Arc::new(VirtualClock::new());
        let clock_dyn: Arc<dyn Clock> = clock.clone();
        let producer = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("local_gateway_p_{id}_{attempt}"),
                clock: clock_dyn,
                ..Default::default()
            },
            root_a.clone(),
        )
        .expect("init producer");
        let gateway_manager = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("local_gateway_g_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    robot_identity: Some("local_transport_test".to_string()),
                    ..Default::default()
                }),
                ..Default::default()
            },
            root_a.clone(),
        )
        .expect("init gateway");
        let plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![topic.clone()],
            ingress: vec![],
        };
        match GatewayRuntime::new(gateway_manager, plan) {
            Ok(gateway) => {
                setup = Some((producer, clock, gateway, port));
                break;
            }
            Err(error) => eprintln!("gateway listen attempt {attempt} failed: {error}"),
        }
    }

    let Some((producer, clock, mut gateway, port)) = setup else {
        panic!("could not bind the gateway listener after three attempts");
    };
    let mut publisher = producer
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create producer publisher");

    let reader = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("local_gateway_b_{id}"),
            network: Some(NetworkConfig {
                connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                ..Default::default()
            }),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init remote reader");
    let subscriber = reader
        .create_subscriber(&topic)
        .expect("create remote local subscriber");
    reader
        .register_ingress_topic(&topic, Vector3::SCHEMA_HASH, MaxSliceLen::const_new(256))
        .expect("declare reader demand");

    let demanded = gateway
        .manager()
        .bridge_manager()
        .register_topic(&topic)
        .expect("gateway demand flag");
    let demand_deadline = Instant::now() + Duration::from_secs(10);
    while !demanded.load(Ordering::Relaxed) {
        assert!(
            Instant::now() < demand_deadline,
            "reader demand did not reach the gateway"
        );
        thread::sleep(Duration::from_millis(20));
    }

    let attach_deadline = Instant::now() + Duration::from_secs(5);
    while gateway.active_tap_topics().is_empty() {
        assert!(
            Instant::now() < attach_deadline,
            "gateway tap did not attach"
        );
        gateway.drive_once().expect("drive tap attach");
        thread::sleep(Duration::from_millis(10));
    }

    for (timestamp_ns, values) in PLAN {
        publish_one(&mut publisher, &clock, timestamp_ns, values);
    }

    let mut frames = Vec::with_capacity(PLAN.len());
    let receive_deadline = Instant::now() + Duration::from_secs(10);
    while frames.len() < PLAN.len() && Instant::now() < receive_deadline {
        gateway.drive_once().expect("forward local frame");
        subscriber
            .try_receive(|message| frames.push(received_frame(message)))
            .expect("receive forwarded frame");
        if frames.len() < PLAN.len() {
            thread::sleep(Duration::from_millis(20));
        }
    }
    // The gateway's own forwarded counter is an observation independent of
    // local delivery: it increments only in `forward_frame`'s success arm on
    // the network egress path, so delivery alone cannot fake it.
    let forwarded = gateway.forwarded_count(&topic);
    (frames, forwarded)
}

#[test]
fn gateway_forwards_oracle_frames_to_remote_reader_in_process() {
    let (actual, forwarded) = run_gateway_delivery();
    assert_eq!(
        actual,
        oracle_frames(),
        "reader must receive the exact hand-built wire frames"
    );
    assert_eq!(
        forwarded,
        PLAN.len() as u64,
        "the gateway's forwarded counter must prove every frame went over the network"
    );
}

#[traced_test]
#[test]
fn local_publisher_creation_skips_the_shm_pool_advice_scan() {
    // The local transport maps no shared-memory pools, so the
    // MADV_NOHUGEPAGE advice scan must not run at publisher creation: its
    // empty result is the normal state here, not the publisher-site anomaly
    // that warns "our /proc/self/maps scan is broken".
    // The oracle is the scan counter, not the absence of warnings: an
    // earlier publisher test can consume the process-wide warn-once token
    // before these logs are checked.
    let scans_before = cerulion_core::transport::shm_pool_advice_scans_run();
    let id = unique_id();
    let topic = format!("/local_transport/advise/{id}");
    let producer = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("local_advise_{id}"),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init producer");
    let _publisher = producer
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");

    assert_eq!(
        cerulion_core::transport::shm_pool_advice_scans_run(),
        scans_before,
        "local transport must not run the MADV_NOHUGEPAGE pool scan"
    );
    logs_assert(|lines: &[&str]| {
        assert!(
            !lines.iter().any(|line| line.contains("scan is broken")
                || line.contains("MADV_NOHUGEPAGE mitigation found NO")),
            "local transport must not emit the broken-scan warning: {lines:?}"
        );
        Ok(())
    });
}

#[traced_test]
#[test]
fn local_manager_creation_skips_the_shm_safety_warnings() {
    // The local transport maps no shared-memory pools, so manager creation
    // must not run `check_and_warn_shm_safety`: THP shmem, RLIMIT_AS and
    // max_map_count warnings all describe a hazard no local port can hit.
    // The oracle is the probe counter, not the absence of warnings, so the
    // test fails even on a host whose settings would warn nothing.
    let probes_before = cerulion_core::transport::shm_safety_checks_run();
    let id = unique_id();
    let _manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("local_safety_{id}"),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init manager");
    assert_eq!(
        cerulion_core::transport::shm_safety_checks_run(),
        probes_before,
        "local-transport init ran the shared-memory safety probes"
    );

    logs_assert(|lines: &[&str]| {
        assert!(
            !lines
                .iter()
                .any(|line| line.contains("Transparent Huge Pages")
                    || line.contains("RLIMIT_AS")
                    || line.contains("max_map_count")),
            "local transport must not emit shared-memory safety warnings: {lines:?}"
        );
        Ok(())
    });
}

#[test]
#[serial]
fn local_transport_refuses_cross_process_initialization_and_graphs() {
    let error = match TransportManager::init_with_config(
        TransportConfig::default(),
        cerulion_core::testing::iceoryx_test_config(),
    ) {
        Err(error) => error,
        Ok(_) => panic!("local-transport must refuse init_with_config"),
    };
    assert!(
        matches!(error, TransportError::NodeCreation { .. }),
        "initialization refusal must be NodeCreation: {error}"
    );
    let error_message = error.to_string();
    assert!(
        error_message.contains("local-transport"),
        "initialization refusal must name local-transport: {error}"
    );
    assert!(
        error_message.contains(
            "joining another process's transport namespace \
             (TransportManager::init_with_config)"
        ),
        "initialization refusal must explain the cross-process join: {error}"
    );
    assert!(
        TransportManager::get().is_err(),
        "refused initialization must leave the singleton unset"
    );

    let error = match TransportManager::detached_with_config(
        TransportConfig::default(),
        cerulion_core::testing::iceoryx_test_config(),
    ) {
        Err(error) => error,
        Ok(_) => panic!("local-transport must refuse detached pre-creation"),
    };
    assert!(
        matches!(error, TransportError::NodeCreation { .. }),
        "detached pre-creation refusal must be NodeCreation: {error}"
    );
    let error_message = error.to_string();
    assert!(
        error_message.contains("local-transport"),
        "detached pre-creation refusal must name local-transport: {error}"
    );
    assert!(
        error_message.contains(
            "pre-creating services for other processes \
             (TransportManager::detached_with_config)"
        ),
        "detached pre-creation refusal must explain the cross-process services: {error}"
    );
    assert!(
        TransportManager::get().is_err(),
        "refused detached pre-creation must leave the singleton unset"
    );

    let yaml = r#"
prefix: local
nodes:
  - id: node
    type: test
process_groups:
  worker:
    - node
"#;
    let graph = cerulion_core::graph::parse_graph(yaml).expect("parse graph YAML");
    let error = cerulion_core::graph::validate_graph(&graph)
        .expect_err("local-transport must refuse process groups");
    assert!(
        matches!(error, TransportError::GraphError { .. }),
        "graph refusal must be GraphError: {error}"
    );
    assert!(
        format!("{error}").contains("local-transport"),
        "graph refusal must name local-transport: {error}"
    );
}

fn assert_loader_refuses_cdylib_under_local_transport(fixture: &str) {
    let path = cerulion_core::testing::find_fixture_cdylib(fixture);
    let error = match cerulion_core::graph::node::DylibNodeEntry::load(&path) {
        Err(error) => error,
        Ok(_) => panic!("{fixture} is built without local-transport and must be refused"),
    };
    assert!(
        matches!(error, TransportError::NodeError { .. }),
        "{fixture} refusal must be NodeError: {error}"
    );
    assert!(
        error
            .to_string()
            .contains("native cdylib nodes are not supported under `local-transport`"),
        "{fixture} refusal must name the local-transport limitation: {error}"
    );
}

#[test]
fn loader_refuses_a_macro_cdylib_built_without_local_transport() {
    assert_loader_refuses_cdylib_under_local_transport("test_node_macro_cdylib");
}

#[test]
fn loader_refuses_a_legacy_cross_process_cdylib() {
    assert_loader_refuses_cdylib_under_local_transport("test_node_cdylib");
}

/// A cdylib built WITH `local-transport` (a matching transport id) is refused
/// the same way: its process-local registry is a different image's registry,
/// so its frames cannot reach the host (issue #301). Needs
/// `cargo build -p test_node_macro_cdylib --features local-transport` so the
/// on-disk fixture carries the matching tag.
#[test]
#[ignore = "requires a local-transport build of the fixture"]
fn loader_refuses_a_matching_local_transport_cdylib() {
    assert_loader_refuses_cdylib_under_local_transport("test_node_macro_cdylib");
}

#[test]
#[ignore = "device role for Android local-transport end-to-end checks"]
#[serial]
fn serve_role() {
    let listen = std::env::var("CERULION_LOCAL_SERVE_LISTEN")
        .unwrap_or_else(|_| "tcp/0.0.0.0:7683".to_string());
    let topic = std::env::var("CERULION_LOCAL_SERVE_TOPIC")
        .unwrap_or_else(|_| "/local/example".to_string());
    let seconds = std::env::var("CERULION_LOCAL_SERVE_SECS")
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .unwrap_or(60);
    let clock = Arc::new(VirtualClock::new());
    let clock_dyn: Arc<dyn Clock> = clock.clone();
    let manager = TransportManager::init(TransportConfig {
        node_name: format!("local_serve_{}", std::process::id()),
        clock: clock_dyn,
        network: Some(NetworkConfig {
            listen_endpoints: vec![listen],
            robot_identity: Some("local_example".to_string()),
            ..Default::default()
        }),
        ..Default::default()
    })
    .expect("initialize local serving manager");
    let mut publisher = manager
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create local publisher");
    let subscriber = manager
        .create_subscriber(&topic)
        .expect("create local subscriber");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic.clone()],
        ingress: vec![],
    };
    let serving = SchemaServing {
        topic_schemas: vec![TopicSchema {
            topic: topic.clone(),
            schema_name: "geometry_msgs/Vector3".to_string(),
        }],
        schema_docs: vec![],
        schema_hashes: vec![SchemaHashName {
            schema_hash: Vector3::SCHEMA_HASH,
            qualified: "geometry_msgs/Vector3".to_string(),
        }],
    };
    let mut gateway =
        GatewayRuntime::new_with_schema_serving(manager, plan, serving).expect("start gateway");

    let stop = Arc::new(AtomicBool::new(false));
    let published = Arc::new(AtomicU64::new(0));
    let received = Arc::new(AtomicU64::new(0));
    let last_sequence = Arc::new(AtomicU64::new(0));

    let publisher_stop = stop.clone();
    let publisher_count = published.clone();
    let publisher_clock = clock.clone();
    let publisher_thread = thread::spawn(move || {
        let mut timestamp_ns = 0u64;
        while !publisher_stop.load(Ordering::Relaxed) {
            timestamp_ns = timestamp_ns.saturating_add(100_000_000);
            publish_one(
                &mut publisher,
                &publisher_clock,
                timestamp_ns,
                (timestamp_ns as f64 / 1_000_000_000.0, 2.0, 3.0),
            );
            publisher_count.fetch_add(1, Ordering::Relaxed);
            thread::sleep(Duration::from_millis(100));
        }
    });

    let subscriber_stop = stop.clone();
    let subscriber_count = received.clone();
    let subscriber_sequence = last_sequence.clone();
    let subscriber_thread = thread::spawn(move || {
        let mut report_at = Instant::now();
        while !subscriber_stop.load(Ordering::Relaxed) {
            subscriber
                .try_receive(|message| {
                    subscriber_count.fetch_add(1, Ordering::Relaxed);
                    subscriber_sequence.store(message.header().sequence as u64, Ordering::Relaxed);
                })
                .expect("receive local serving frame");
            if report_at.elapsed() >= Duration::from_secs(1) {
                println!(
                    "local received={} last_seq={}",
                    subscriber_count.load(Ordering::Relaxed),
                    subscriber_sequence.load(Ordering::Relaxed)
                );
                report_at = Instant::now();
            }
            thread::sleep(Duration::from_millis(5));
        }
    });

    println!("READY");
    let deadline = Instant::now() + Duration::from_secs(seconds);
    while Instant::now() < deadline {
        gateway.drive_once().expect("drive local gateway");
        thread::sleep(Duration::from_millis(10));
    }
    stop.store(true, Ordering::Relaxed);
    publisher_thread.join().expect("publisher role thread");
    subscriber_thread.join().expect("subscriber role thread");
    println!(
        "published={} local_received={} forwarded={}",
        published.load(Ordering::Relaxed),
        received.load(Ordering::Relaxed),
        gateway.forwarded_count(&topic)
    );
}
