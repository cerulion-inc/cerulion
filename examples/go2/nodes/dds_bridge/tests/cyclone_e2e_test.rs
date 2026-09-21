// SPDX-License-Identifier: AGPL-3.0-only
//! LIVE-DDS e2e (needs ROS 2 Humble + CycloneDDS): a real CycloneDDS peer → the DDS pump → the
//! bridge node → a Cerulion subscriber, asserting the known-cloud byte oracle.
//!
//! `#[ignore]`'d (the rclpy_xproc convention): CI has no ROS distro. Run on a
//! machine with ROS 2 Humble + CycloneDDS, a robot-LAN (or any multicast-capable)
//! interface, and the peer publishing the KNOWN cloud:
//!
//! ```bash
//! # Terminal A (the peer — Humble + CycloneDDS):
//! source /opt/ros/humble/setup.bash
//! export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
//! export ROS_DOMAIN_ID=0
//! ros2 topic pub /probe_cloud sensor_msgs/msg/PointCloud2 \
//! '{height: 1, width: 2, is_bigendian: false, point_step: 12, row_step: 24, is_dense: true,
//!   fields: [{name: x, offset: 0, datatype: 7, count: 1},
//!            {name: y, offset: 4, datatype: 7, count: 1},
//!            {name: z, offset: 8, datatype: 7, count: 1}],
//!   data: [0,0,128,63, 0,0,0,64, 0,0,64,64,  0,0,128,64, 0,0,160,64, 0,0,192,64]}' -r 2
//!
//! # Terminal B (this test — GO2_IFACE = this machine's primary interface IP;
//! # REQUIRED: without with_only_networks a multi-homed host fragments
//! # SPDP/SEDP and CycloneDDS drops discovery — the failure this test pins):
//! GO2_IFACE=192.168.x.y cargo test -p dds_bridge --test cyclone_e2e_test -- --ignored --nocapture
//! ```
//!
//! # Harness choice (justification)
//!
//! The node runs in a real `GraphRuntime` (the polled seam — trigger + step,
//! which never queries `external_source`) while the test drives the
//! PRODUCTION `BridgePump` directly on a thread sharing the node's queue —
//! i.e. exactly the production dataflow (real rustdds subscription → typed
//! `cerulion_go2_dds` decode → latest-wins queue → tick → schema write → iceoryx2) minus
//! only the doorbell/WaitSet wiring, which is platform-pinned by
//! `cerulion_core/tests/cdylib_blocking_doorbell_test.rs`. Driving the full
//! live loop would need `run_live` inside a test (its polled/test seams are
//! cerulion_core-internal); this harness exercises every line of THIS crate's
//! production path.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::transport::{TransportConfig, TransportManager};
use indexmap::IndexMap;
use native_ros2_messages::sensor_msgs::PointCloud2;

use dds_bridge::config::BridgeConfig;
use dds_bridge::pump::BridgePump;
use dds_bridge::queue::SampleQueue;
use dds_bridge::{DdsBridge, DdsBridgeEntry};

/// The known point bytes the peer recipe publishes: (1,2,3)/(4,5,6) LE f32.
const KNOWN_DATA: [u8; 24] = [
    0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64, // (1.0, 2.0, 3.0)
    0, 0, 128, 64, 0, 0, 160, 64, 0, 0, 192, 64, // (4.0, 5.0, 6.0)
];

/// Precondition check (loud panic with the recipe — the rclpy_xproc pattern):
/// `ros2` reachable (evidence a ROS 2 env is sourced for the PEER half) and
/// `GO2_IFACE` set (the with_only_networks constraint — see the module docs).
fn preconditions() -> Vec<std::net::IpAddr> {
    if std::process::Command::new("ros2")
        .arg("--help")
        .output()
        .is_err()
    {
        panic!(
            "PRECONDITION: `ros2` not runnable — source a ROS 2 Humble env \
             (the peer half of this test needs `ros2 topic pub` under \
             RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; recipe in this file's header)"
        );
    }
    let raw = std::env::var("GO2_IFACE").unwrap_or_else(|_| {
        panic!(
            "PRECONDITION: GO2_IFACE is not set — set it to this machine's primary \
             interface IP (e.g. GO2_IFACE=192.168.123.99). REQUIRED: without \
             DomainParticipantBuilder::with_only_networks a multi-homed host \
             bloats SPDP/SEDP past ~1.4 KB and CycloneDDS DROPS the fragmented \
             builtin data, so discovery never completes (see the \
             cerulion_go2_dds crate docs, 'Interop constraints')"
        )
    });
    let ips = cerulion_go2_dds::participant::parse_iface_list(&raw);
    assert!(
        !ips.is_empty(),
        "PRECONDITION: GO2_IFACE={raw:?} parsed to no IP addresses"
    );
    ips
}

#[test]
#[ignore] // needs a live CycloneDDS peer (recipe in the header)
fn cyclone_peer_cloud_reaches_cerulion_subscriber_byte_exact() {
    // Without a subscriber, node-side tracing (INCLUDING a tick error that
    // kills the whole body — the all-four-ports failure below) is INVISIBLE.
    // Install a minimal stderr subscriber so failures PRINT. RUST_LOG
    // overrides; default info. try_init: idempotent if the harness ever
    // grows one.
    let _ = tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .try_init();

    let only_networks = preconditions();
    let domain: u16 = std::env::var("CER_DDS_E2E_DOMAIN")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);

    // The bridge graph on an isolated SHM root.
    let clock = Arc::new(VirtualClock::new());
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "dds_bridge_cyclone_e2e".to_string(),
            clock: clock.clone(),
            subscriber_buffer_size: 16,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated transport");

    let queue: Arc<Mutex<SampleQueue>> = Arc::new(Mutex::new(SampleQueue::default()));
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "dds_bridge_cyclone_e2e".to_string(),
        prefix: "cyc".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "bridge".to_string(),
            node_type: "dds_bridge".to_string(),
            inputs: vec![],
            // ALL FOUR ports MUST be wired even though this e2e maps only the
            // cloud: macro ports are compile-time, and the generated tick
            // wrapper resolves EVERY output proxy before the user body runs —
            // a missing port errs the tick ("zero-copy tick: missing
            // publisher for output `...`", cerulion_macros impl_macro.rs), so
            // the drain never executes. The signature of that failure:
            // a cloud-only NodeDef gives pump[samples_pushed=
            // 172] queue[drained=0] — reception fine, node body never ran.
            // Unwired-but-unmapped ports are harmless: they never publish
            // (the output discard gate).
            outputs: [
                ("cloud", "sensor_msgs/PointCloud2", Some(1024 * 1024)),
                ("odom", "nav_msgs/Odometry", None),
                ("twist", "geometry_msgs/TwistStamped", None),
                ("request_json", "std_msgs/String", None),
            ]
            .into_iter()
            .map(|(name, schema, max_slice_len)| OutputDef {
                name: name.to_string(),
                schema: schema.to_string(),
                max_slice_len,
                history_size: 0,
                topic: None,
            })
            .collect(),
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "bridge".to_string(),
        Box::new(DdsBridgeEntry::with_state(DdsBridge::with_shared_queue(
            Arc::clone(&queue),
        ))),
    );
    let mut rt = GraphRuntime::build(config, factories, &mgr, clock)
        .expect("bridge graph builds (all four ports wired)");
    let mut obs = mgr
        .create_subscriber("/cyc/bridge/cloud")
        .expect("cloud observer");

    // The PRODUCTION pump, driven on a test thread sharing the node's queue
    // (the harness-choice note in the module docs).
    let bridge_cfg = BridgeConfig::from_yaml(
        &format!(
            "domain_id: {domain}\nonly_networks: [{}]\nmappings:\n  - dds_topic: /probe_cloud\n    ros_type: sensor_msgs/PointCloud2\n    cerulion_topic: /cyc/bridge/cloud\n    qos: best_effort\n",
            only_networks
                .iter()
                .map(|ip| format!("\"{ip}\""))
                .collect::<Vec<_>>()
                .join(", ")
        ),
        "cyclone_e2e inline",
    )
    .expect("inline e2e config validates");

    let stop = Arc::new(AtomicBool::new(false));
    let pump_stop = Arc::clone(&stop);
    let pump_queue = Arc::clone(&queue);
    // Construct the pump on the test thread ONLY to grab the shared stats
    // handle (Principle #3 — the failure message below localizes the dead
    // hop); all DDS objects are created on the pump's own drain thread.
    let mut pump = BridgePump::new(bridge_cfg, pump_queue);
    let pump_stats = pump.stats();
    let pump_thread = std::thread::spawn(move || {
        // The PRODUCTION helper step verbatim (iterate + paced sleep) — the
        // exact code the node's Blocking closure runs.
        while !pump_stop.load(Ordering::SeqCst) {
            pump.run_helper_iteration();
        }
    });

    // Poll: fire the node + look for the known cloud (bounded 30 s — covers
    // discovery + matching against the 2 Hz peer).
    let deadline = Instant::now() + Duration::from_secs(30);
    let mut seen = None;
    while Instant::now() < deadline {
        rt.trigger_external("bridge").expect("trigger bridge");
        rt.step(Duration::from_millis(20));
        let got = obs
            .try_view::<PointCloud2, _>(|v| {
                (
                    v.width,
                    v.height,
                    v.point_step,
                    v.row_step,
                    v.data().to_vec(),
                    v.fields_bytes().to_vec(),
                )
            })
            .expect("try_view cloud");
        if let Some(g) = got {
            seen = Some(g);
            break;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    stop.store(true, Ordering::SeqCst);
    pump_thread.join().expect("pump thread joins");

    let (width, height, point_step, row_step, data, fields_blob) = seen.unwrap_or_else(|| {
        // The hop counters localize the dead hop on sight:
        //   samples_pushed == 0                  -> DDS->queue dead (discovery/
        //                                           matching/stream — check the
        //                                           streams_started + peer);
        //   samples_pushed > 0, drained == 0     -> queue->tick dead (fire/
        //                                           trigger wiring);
        //   drained > 0                          -> tick->publish->observe dead
        //                                           (schema write / subscriber).
        let (drained, evicted) = {
            let q = queue
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            (q.lifetime_drained_total(), q.lifetime_dropped_total())
        };
        panic!(
            "no cloud arrived from the CycloneDDS peer within 30 s.\n\
             HOP COUNTERS: pump[{}] queue[drained={drained} evicted={evicted}]\n\
             Checklist: is the peer publishing (recipe in this file's header)? \
             Same domain (CER_DDS_E2E_DOMAIN / ROS_DOMAIN_ID)? Multicast OK on \
             GO2_IFACE? If CycloneDDS prints 'fragmented builtin data not yet \
             supported', re-check GO2_IFACE (the with_only_networks constraint)",
            pump_stats.render(),
        )
    });

    // The byte oracle: the peer's known cloud, byte-exact through the bridge.
    assert_eq!(width, 2);
    assert_eq!(height, 1);
    assert_eq!(point_step, 12);
    assert_eq!(row_step, 24);
    assert_eq!(data, KNOWN_DATA, "point bytes must cross byte-exact");
    // The packed fields blob for x/y/z float32 at 0/4/8 (the cerulion_viz layout).
    let mut oracle_blob = Vec::new();
    for (name, off) in [("x", 0u32), ("y", 4), ("z", 8)] {
        oracle_blob.extend_from_slice(&(name.len() as u32).to_le_bytes());
        oracle_blob.extend_from_slice(name.as_bytes());
        oracle_blob.extend_from_slice(&off.to_le_bytes());
        oracle_blob.push(7u8);
        oracle_blob.extend_from_slice(&1u32.to_le_bytes());
    }
    assert_eq!(
        fields_blob, oracle_blob,
        "fields blob must match the packed layout"
    );

    // Decode the delivered points as the sink would: (1,2,3)/(4,5,6).
    let p = |i: usize, off: usize| {
        f32::from_le_bytes(data[i * 12 + off..i * 12 + off + 4].try_into().unwrap())
    };
    assert_eq!(
        [[p(0, 0), p(0, 4), p(0, 8)], [p(1, 0), p(1, 4), p(1, 8)]],
        [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
    );
}

/// LIVE-DDS e2e: a CycloneDDS peer publishing a type OUTSIDE the hand
/// registry (`sensor_msgs/Imu`) flows through the RAW-generic path
/// (raw-CDR reader → generic codec → dynamic ingress publisher) into a
/// Cerulion subscriber, byte-field-verified via the schema-driven
/// `FrameWalker`. ZERO per-type code anywhere on this path — the
/// acceptance arm.
///
/// No graph, no node: raw routes bypass the node's ports entirely (frames
/// publish straight from the pump's drain thread), so the harness is the
/// pump + an observing subscriber on an isolated transport.
///
/// ```bash
/// # Terminal A (the peer — Humble + CycloneDDS). ALL values dyadic-exact so
/// # YAML→f64 parsing is bit-deterministic:
/// source /opt/ros/humble/setup.bash
/// export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
/// export ROS_DOMAIN_ID=0
/// ros2 topic pub /probe_imu sensor_msgs/msg/Imu \
/// '{orientation: {x: 0.5, y: -0.25, z: 0.125, w: 0.8125},
///   angular_velocity: {x: 1.5, y: -2.5, z: 3.25},
///   linear_acceleration: {x: 9.5, y: 0.5, z: -0.25}}' -r 2
///
/// # Terminal B (this test):
/// GO2_IFACE=192.168.x.y cargo test -p dds_bridge --test cyclone_e2e_test \
///   cyclone_peer_imu -- --ignored --nocapture
/// ```
#[test]
#[ignore] // needs a live CycloneDDS peer (recipe above)
fn cyclone_peer_imu_reaches_cerulion_via_the_raw_generic_route() {
    use cerulion_core::codegen::{parse_rosmsg, FrameValueKind, FrameWalker};
    use cerulion_core::message::ShmMessage;
    use cerulion_core::wire::WireHeader;

    let _ = tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .try_init();

    let only_networks = preconditions();
    let domain: u16 = std::env::var("CER_DDS_E2E_DOMAIN")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);

    // Isolated transport with the PRODUCTION RealClock: raw wire stamps are
    // drain-time transport-clock reads (lib.rs raw-path docs), so a nonzero
    // timestamp is assertable below.
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "dds_bridge_cyclone_imu_e2e".to_string(),
            clock: Arc::new(cerulion_core::clock::RealClock),
            subscriber_buffer_size: 16,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated transport");

    // ONE raw mapping, validated through the schema-resolvable relaxation (Imu is
    // outside the 4-type registry but schema-resolvable).
    let bridge_cfg = BridgeConfig::from_yaml(
        &format!(
            "domain_id: {domain}\nonly_networks: [{}]\nmappings:\n  - dds_topic: /probe_imu\n    ros_type: sensor_msgs/Imu\n    cerulion_topic: /cyc/raw/imu\n    qos: best_effort\n",
            only_networks
                .iter()
                .map(|ip| format!("\"{ip}\""))
                .collect::<Vec<_>>()
                .join(", ")
        ),
        "cyclone_imu_e2e inline",
    )
    .expect("raw e2e config validates (the schema-resolvable relaxation)");

    let obs = mgr.create_subscriber("/cyc/raw/imu").expect("imu observer");

    // The PRODUCTION pump construction (with_transport IS the
    // production path — the node hands its context-carried manager the same
    // way; this test hands the isolated init_for_test manager directly).
    let stop = Arc::new(AtomicBool::new(false));
    let pump_stop = Arc::clone(&stop);
    let mut pump = BridgePump::with_transport(
        bridge_cfg,
        Arc::new(Mutex::new(SampleQueue::default())), // unused by raw routes
        Arc::clone(&mgr),
    );
    let pump_stats = pump.stats();
    let pump_thread = std::thread::spawn(move || {
        while !pump_stop.load(Ordering::SeqCst) {
            pump.run_helper_iteration();
        }
    });

    // Poll for the first raw frame (bounded 30 s — discovery + the 2 Hz peer).
    let deadline = Instant::now() + Duration::from_secs(30);
    let mut first: Option<Vec<u8>> = None;
    while Instant::now() < deadline && first.is_none() {
        obs.try_receive(|msg| {
            if first.is_none() {
                let mut f = vec![0u8; WireHeader::SIZE];
                msg.header().write_to_buf(&mut f);
                f.extend_from_slice(msg.payload());
                first = Some(f);
            }
        })
        .expect("try_receive imu");
        std::thread::sleep(Duration::from_millis(20));
    }
    stop.store(true, Ordering::SeqCst);
    pump_thread.join().expect("pump thread joins");

    let frame = first.unwrap_or_else(|| {
        // The raw hop counters localize the dead hop on sight:
        //   raw_streams_started == 0            -> reader/route init dead
        //                                          (check the set_failed log);
        //   raw_published == 0, failures == 0   -> DDS side quiet (peer/
        //                                          domain/GO2_IFACE);
        //   raw_route_failures > 0              -> frames arrive but fail
        //                                          transcode (schema drift?).
        panic!(
            "no Imu frame arrived from the CycloneDDS peer within 30 s.\n\
             HOP COUNTERS: pump[{}]\n\
             Checklist: is the peer publishing (recipe in this test's doc)? \
             Same domain (CER_DDS_E2E_DOMAIN / ROS_DOMAIN_ID)? Multicast OK \
             on GO2_IFACE?",
            pump_stats.render(),
        )
    });

    // Header pins: the GENERATED type's schema hash routes the frame; the
    // wire stamp is a real drain-time clock read; the first publish is seq 0.
    let hdr = WireHeader::read_from_buf(&frame).expect("header parses");
    assert_eq!(
        hdr.schema_hash,
        <native_ros2_messages::sensor_msgs::Imu as ShmMessage>::SCHEMA_HASH,
        "codec hash must equal the generated type's SCHEMA_HASH"
    );
    assert!(hdr.timestamp_ns > 0, "drain-time transport-clock stamp");
    assert_eq!(hdr.sequence, 0, "the route's first publish carries seq 0");

    // Byte-level field verification via the schema-driven walker (built over
    // the SAME embedded registry the codec uses; walk_by_hash also pins the
    // hash → schema routing).
    let schemas: Vec<_> = native_ros2_messages::BUILTIN_MSGS
        .iter()
        .map(|&(pkg, name, text)| parse_rosmsg(text, name, Some(pkg)).expect("builtin parses"))
        .collect();
    let (walker, _) = FrameWalker::new(schemas);
    let fv = walker.walk_by_hash(&frame).expect("walker decodes by hash");

    let nested_f64 = |field: &str, sub: &str| -> f64 {
        match fv.field(field) {
            Some(FrameValueKind::Nested(n)) => match n.field(sub) {
                Some(FrameValueKind::F64(v)) => *v,
                other => panic!("{field}.{sub}: expected F64, got {other:?}"),
            },
            other => panic!("{field}: expected Nested, got {other:?}"),
        }
    };
    // The recipe's dyadic-exact values, byte-exact through the whole path.
    assert_eq!(nested_f64("orientation", "x"), 0.5);
    assert_eq!(nested_f64("orientation", "y"), -0.25);
    assert_eq!(nested_f64("orientation", "z"), 0.125);
    assert_eq!(nested_f64("orientation", "w"), 0.8125);
    assert_eq!(nested_f64("angular_velocity", "x"), 1.5);
    assert_eq!(nested_f64("angular_velocity", "y"), -2.5);
    assert_eq!(nested_f64("angular_velocity", "z"), 3.25);
    assert_eq!(nested_f64("linear_acceleration", "x"), 9.5);
    assert_eq!(nested_f64("linear_acceleration", "y"), 0.5);
    assert_eq!(nested_f64("linear_acceleration", "z"), -0.25);
    // Covariance arrays: 9 f64 zeros each (the recipe leaves them default).
    for cov in [
        "orientation_covariance",
        "angular_velocity_covariance",
        "linear_acceleration_covariance",
    ] {
        match fv.field(cov) {
            Some(FrameValueKind::PrimArray(pa)) => {
                let vals: Vec<f64> = pa.iter_f64().collect();
                assert_eq!(vals, vec![0.0; 9], "{cov}");
            }
            other => panic!("{cov}: expected PrimArray, got {other:?}"),
        }
    }
}
