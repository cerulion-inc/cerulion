// SPDX-License-Identifier: AGPL-3.0-only
//! The rmw graph registry `/__cerulion/rmw_graph` over real shared memory.
//!
//! Each arm builds its own managers on an isolated iceoryx2 root, so two
//! managers stand in for two processes on one host. Hostile and blocking
//! peers are raw iceoryx2 ports opened with the module's public constants.
//! The killed-writer arm runs a real child process and kills it, because a
//! `Dead` owner node exists only after its process is gone.
//!
//! Expected maps are written by hand in each arm. Every arm is `#[serial]`.
//! Root tags stay short: on macOS a doorbell listener is a socket whose path
//! holds the root, and a long root makes the listener fail to open.
//!
//! ```text
//! cargo test -p cerulion_core --test rmw_graph_registry_iox2_test -- --test-threads=1
//! ```

use std::collections::{BTreeMap, BTreeSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::testing::{child_iceoryx_config, kill_child_holding_ports, IsolatedRoot};
use cerulion_core::transport::dead_node_sweep::sweep_node_config;
use cerulion_core::transport::rmw_graph_registry::{
    encode_snapshot, GatherCompleteness, RmwGraphFrameError, RmwGraphGather,
    MAX_RMW_GRAPH_TOPIC_LEN, RMW_GRAPH_DOORBELL_SERVICE_NAME, RMW_GRAPH_FRAME_LEN,
    RMW_GRAPH_MAX_NODES, RMW_GRAPH_MAX_READERS, RMW_GRAPH_MAX_WRITERS, RMW_GRAPH_QUEUE_DEPTH,
    RMW_GRAPH_REGISTRY_SERVICE_NAME,
};
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::TransportError;
use iceoryx2::prelude::{CallbackProgression, NodeBuilder, ServiceName};
use iceoryx2::service::ipc_threadsafe::Service as Ipc;
use iceoryx2::service::port_factory::PortFactory as _;
use serial_test::serial;
use tracing_test::traced_test;

/// A ceiling, never a wait: a settled gather returns as soon as it has heard
/// every live writer.
const SETTLE_WINDOW: Duration = Duration::from_secs(5);

/// The window for an arm whose gather is expected to end `Incomplete`.
const SHORT_WINDOW: Duration = Duration::from_millis(300);

const CHILD_READY: &str = "CHILD_RMW_GRAPH_WRITER_READY";
const CHILD_TOPIC: &str = "/child/joint_states";
const CHILD_TYPE: &str = "sensor_msgs/msg/JointState";

fn manager(tag: &str, config: &iceoryx2::config::Config) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("rmwg_{tag}"),
            ..Default::default()
        },
        config.clone(),
    )
    .expect("manager on the isolated root")
}

/// A raw iceoryx2 node on `config` that never reaps another node.
fn raw_node(config: &iceoryx2::config::Config) -> iceoryx2::node::Node<Ipc> {
    NodeBuilder::new()
        .config(&sweep_node_config(config))
        .create::<Ipc>()
        .expect("raw node")
}

/// The registry service opened the way the module opens it.
fn raw_registry(
    node: &iceoryx2::node::Node<Ipc>,
) -> iceoryx2::service::port_factory::publish_subscribe::PortFactory<Ipc, [u8], ()> {
    let name: ServiceName = RMW_GRAPH_REGISTRY_SERVICE_NAME
        .try_into()
        .expect("service name");
    node.service_builder(&name)
        .publish_subscribe::<[u8]>()
        .subscriber_max_buffer_size(RMW_GRAPH_QUEUE_DEPTH)
        .max_subscribers(RMW_GRAPH_MAX_READERS)
        .max_publishers(RMW_GRAPH_MAX_WRITERS)
        .max_nodes(RMW_GRAPH_MAX_NODES)
        .history_size(0)
        .enable_safe_overflow(true)
        .open_or_create()
        .expect("raw registry service")
}

fn map(rows: &[(&str, &[&str])]) -> BTreeMap<String, BTreeSet<String>> {
    rows.iter()
        .map(|&(topic, types)| {
            (
                topic.to_string(),
                types.iter().map(|t| t.to_string()).collect(),
            )
        })
        .collect()
}

fn settled(rows: &[(&str, &[&str])]) -> RmwGraphGather {
    RmwGraphGather {
        topics: map(rows),
        completeness: GatherCompleteness::Settled,
    }
}

fn gather(reader: &TransportManager, own: Option<u128>, window: Duration) -> RmwGraphGather {
    reader.gather_rmw_graph(own, window).expect("gather")
}

#[test]
#[serial]
fn registry_services_never_list_as_topics_while_a_writer_is_live() {
    let root = IsolatedRoot::mint("rg_a");
    let tm = manager("names_w", &root.config());
    let writer = tm.open_rmw_graph_writer();
    writer
        .add("/chatter", "std_msgs/msg/String")
        .expect("valid pair");
    assert!(writer.writer_id().is_some(), "the writer is live");

    let mut services = BTreeSet::new();
    <Ipc as iceoryx2::service::Service>::list(&root.config(), |details| {
        services.insert(details.static_details.name().as_str().to_string());
        CallbackProgression::Continue
    })
    .expect("list services");
    assert!(
        services.contains(RMW_GRAPH_REGISTRY_SERVICE_NAME)
            && services.contains(RMW_GRAPH_DOORBELL_SERVICE_NAME),
        "both registry services exist while the writer is live: {services:?}"
    );

    let topics = tm.list_topics().expect("list topics");
    assert!(
        topics
            .iter()
            .all(|t| !t.starts_with("/__cerulion/rmw_graph")),
        "a registry service listed as a topic: {topics:?}"
    );
}

#[test]
#[serial]
fn two_writers_gather_to_the_hand_written_map_and_settle_the_same_way_twice() {
    let root = IsolatedRoot::mint("rg_b");
    let a = manager("two_a", &root.config());
    let b = manager("two_b", &root.config());
    let reader = manager("two_r", &root.config());
    let wa = a.open_rmw_graph_writer();
    let wb = b.open_rmw_graph_writer();
    wa.add("/joint_states", "sensor_msgs/msg/JointState")
        .expect("pair");
    wa.add("/rosout", "rcl_interfaces/msg/Log").expect("pair");
    wb.add("/cmd_vel", "geometry_msgs/msg/Twist").expect("pair");
    wb.add("/rosout", "rcl_interfaces/msg/Log").expect("pair");

    let expected = settled(&[
        ("/cmd_vel", &["geometry_msgs/msg/Twist"]),
        ("/joint_states", &["sensor_msgs/msg/JointState"]),
        ("/rosout", &["rcl_interfaces/msg/Log"]),
    ]);
    let first = gather(&reader, None, SETTLE_WINDOW);
    let second = gather(&reader, None, SETTLE_WINDOW);
    assert_eq!(first, expected);
    assert_eq!(second, expected);

    // A writer does not wait for itself, and its own pairs are not returned.
    assert_eq!(
        gather(&a, wa.writer_id(), SETTLE_WINDOW),
        settled(&[
            ("/cmd_vel", &["geometry_msgs/msg/Twist"]),
            ("/rosout", &["rcl_interfaces/msg/Log"]),
        ])
    );
}

#[test]
#[serial]
fn refcounts_keep_a_pair_until_its_last_endpoint_is_removed() {
    let root = IsolatedRoot::mint("rg_c");
    let tm = manager("refs_w", &root.config());
    let reader = manager("refs_r", &root.config());
    let writer = tm.open_rmw_graph_writer();
    writer
        .add("/scan", "sensor_msgs/msg/LaserScan")
        .expect("pair");
    writer
        .add("/scan", "sensor_msgs/msg/LaserScan")
        .expect("pair");

    writer.remove("/scan", "sensor_msgs/msg/LaserScan");
    assert_eq!(
        gather(&reader, None, SETTLE_WINDOW),
        settled(&[("/scan", &["sensor_msgs/msg/LaserScan"])])
    );

    // An unknown pair, and a known topic with an unknown type, change nothing.
    writer.remove("/never_added", "std_msgs/msg/Empty");
    writer.remove("/scan", "sensor_msgs/msg/PointCloud2");
    assert_eq!(
        gather(&reader, None, SETTLE_WINDOW),
        settled(&[("/scan", &["sensor_msgs/msg/LaserScan"])])
    );

    // The last endpoint goes: the writer is still heard, with nothing to list.
    writer.remove("/scan", "sensor_msgs/msg/LaserScan");
    assert_eq!(gather(&reader, None, SETTLE_WINDOW), settled(&[]));
}

#[test]
#[serial]
fn a_snapshot_larger_than_one_frame_arrives_whole() {
    let root = IsolatedRoot::mint("rg_d");
    let tm = manager("big_w", &root.config());
    let reader = manager("big_r", &root.config());
    let writer = tm.open_rmw_graph_writer();
    // 50 entries of 50 bytes each do not fit one 2048-byte frame.
    let topics: Vec<String> = (0..50)
        .map(|i| format!("/robot/topic_num_{i:03}"))
        .collect();
    for topic in &topics {
        writer
            .add(topic, "sensor_msgs/msg/JointState")
            .expect("pair");
    }
    let frames = encode_snapshot(
        1,
        1,
        topics
            .iter()
            .map(|t| (t.as_str(), "sensor_msgs/msg/JointState")),
    )
    .expect("encode");
    assert_eq!(
        frames.len(),
        2,
        "precondition: the snapshot needs two parts"
    );

    let got = gather(&reader, None, SETTLE_WINDOW);
    assert_eq!(got.completeness, GatherCompleteness::Settled);
    assert_eq!(got.topics.len(), 50);
    assert_eq!(
        got.topics.keys().next().map(String::as_str),
        Some("/robot/topic_num_000")
    );
    assert_eq!(
        got.topics.keys().next_back().map(String::as_str),
        Some("/robot/topic_num_049")
    );
    assert!(got
        .topics
        .values()
        .all(|types| types == &BTreeSet::from(["sensor_msgs/msg/JointState".to_string()])));
}

#[test]
#[serial]
fn two_types_on_one_topic_come_back_sorted() {
    let root = IsolatedRoot::mint("rg_e");
    let a = manager("types_a", &root.config());
    let b = manager("types_b", &root.config());
    let reader = manager("types_r", &root.config());
    let wa = a.open_rmw_graph_writer();
    let wb = b.open_rmw_graph_writer();
    wa.add("/odom", "nav_msgs/msg/Odometry").expect("pair");
    wb.add("/odom", "geometry_msgs/msg/PoseStamped")
        .expect("pair");

    let got = gather(&reader, None, SETTLE_WINDOW);
    assert_eq!(
        got,
        settled(&[(
            "/odom",
            &["geometry_msgs/msg/PoseStamped", "nav_msgs/msg/Odometry"]
        )])
    );
    let types: Vec<&str> = got.topics["/odom"].iter().map(String::as_str).collect();
    assert_eq!(
        types,
        vec!["geometry_msgs/msg/PoseStamped", "nav_msgs/msg/Odometry"]
    );
}

#[test]
#[serial]
fn a_gather_with_every_reader_slot_taken_is_an_error_until_one_frees() {
    let root = IsolatedRoot::mint("rg_f");
    let tm = manager("slots_w", &root.config());
    let reader = manager("slots_r", &root.config());
    let writer = tm.open_rmw_graph_writer();
    writer.add("/imu", "sensor_msgs/msg/Imu").expect("pair");

    let node = raw_node(&root.config());
    let service = raw_registry(&node);
    let held: Vec<_> = (0..RMW_GRAPH_MAX_READERS)
        .map(|_| service.subscriber_builder().create().expect("hold a slot"))
        .collect();

    match reader.gather_rmw_graph(None, SETTLE_WINDOW) {
        Err(TransportError::Internal { reason }) => assert!(
            reason.contains("could not create the gather subscriber"),
            "unexpected reason: {reason}"
        ),
        other => panic!("a gather with no reader slot must fail, got {other:?}"),
    }

    drop(held);
    assert_eq!(
        gather(&reader, None, SETTLE_WINDOW),
        settled(&[("/imu", &["sensor_msgs/msg/Imu"])])
    );
}

#[test]
#[serial]
fn a_lone_writer_settles_at_once_and_a_ring_is_answered_when_one_is_needed() {
    let root = IsolatedRoot::mint("rg_g");
    let tm = manager("lone_w", &root.config());
    let writer = tm.open_rmw_graph_writer();
    writer.add("/tf", "tf2_msgs/msg/TFMessage").expect("pair");

    assert_eq!(gather(&tm, writer.writer_id(), SETTLE_WINDOW), settled(&[]));
    assert_eq!(
        writer.doorbell_rings_answered(),
        0,
        "a gather with nobody to hear must not ring"
    );

    // Without its own id the writer is required, so the gather rings and the
    // pump answers.
    assert_eq!(
        gather(&tm, None, SETTLE_WINDOW),
        settled(&[("/tf", &["tf2_msgs/msg/TFMessage"])])
    );
    let deadline = Instant::now() + SETTLE_WINDOW;
    while writer.doorbell_rings_answered() == 0 && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(
        writer.doorbell_rings_answered() > 0,
        "the ring was answered"
    );
}

#[test]
#[serial]
fn a_dropped_writer_leaves_the_gather() {
    let root = IsolatedRoot::mint("rg_h");
    let tm = manager("drop_w", &root.config());
    let reader = manager("drop_r", &root.config());
    let writer = tm.open_rmw_graph_writer();
    writer
        .add("/map", "nav_msgs/msg/OccupancyGrid")
        .expect("pair");
    assert_eq!(
        gather(&reader, None, SETTLE_WINDOW),
        settled(&[("/map", &["nav_msgs/msg/OccupancyGrid"])])
    );

    drop(writer);
    assert_eq!(gather(&reader, None, SETTLE_WINDOW), settled(&[]));
}

#[test]
#[serial]
fn an_invalid_name_is_refused_and_records_nothing() {
    let root = IsolatedRoot::mint("rg_i");
    let tm = manager("bad_w", &root.config());
    let reader = manager("bad_r", &root.config());
    let writer = tm.open_rmw_graph_writer();
    let long_topic = format!("/{}", "a".repeat(MAX_RMW_GRAPH_TOPIC_LEN));

    assert_eq!(
        writer.add("", "std_msgs/msg/String"),
        Err(RmwGraphFrameError::EmptyTopic)
    );
    assert_eq!(writer.add("/a", ""), Err(RmwGraphFrameError::EmptyType));
    assert_eq!(
        writer.add(&long_topic, "std_msgs/msg/String"),
        Err(RmwGraphFrameError::TopicTooLong {
            len: MAX_RMW_GRAPH_TOPIC_LEN + 1
        })
    );
    assert_eq!(
        writer.writer_id(),
        None,
        "a refused pair opens no port and publishes nothing"
    );
    assert_eq!(gather(&reader, None, SETTLE_WINDOW), settled(&[]));

    writer.add("/ok", "std_msgs/msg/String").expect("pair");
    assert_eq!(
        gather(&reader, None, SETTLE_WINDOW),
        settled(&[("/ok", &["std_msgs/msg/String"])])
    );
}

#[test]
#[serial]
fn a_failed_open_keeps_the_counts_and_opens_on_the_next_call() {
    let root = IsolatedRoot::mint("rg_j");
    let tm = manager("open_w", &root.config());
    let reader = manager("open_r", &root.config());

    // A service of the same name with fewer publisher slots: every open with
    // the module's constants is refused while it exists.
    let node = raw_node(&root.config());
    let name: ServiceName = RMW_GRAPH_REGISTRY_SERVICE_NAME
        .try_into()
        .expect("service name");
    let blocker = node
        .service_builder(&name)
        .publish_subscribe::<[u8]>()
        .max_publishers(1)
        .create()
        .expect("incompatible service");

    let writer = tm.open_rmw_graph_writer();
    writer
        .add("/lidar", "sensor_msgs/msg/PointCloud2")
        .expect("pair");
    writer
        .add("/lidar", "sensor_msgs/msg/PointCloud2")
        .expect("pair");
    assert_eq!(writer.writer_id(), None, "the open was refused");
    assert!(
        reader.gather_rmw_graph(None, SETTLE_WINDOW).is_err(),
        "a gather cannot open the service either"
    );

    drop(blocker);
    drop(node);
    // One of the two endpoints goes; the open runs again on this call.
    writer.remove("/lidar", "sensor_msgs/msg/PointCloud2");
    assert!(writer.writer_id().is_some(), "the open ran again");
    assert_eq!(
        gather(&reader, None, SETTLE_WINDOW),
        settled(&[("/lidar", &["sensor_msgs/msg/PointCloud2"])])
    );
}

#[test]
#[serial]
#[traced_test]
fn hostile_frames_are_skipped_and_the_frames_after_them_still_count() {
    let root = IsolatedRoot::mint("rg_k");
    let tm = manager("hostile_w", &root.config());
    let reader = manager("hostile_r", &root.config());
    let writer = tm.open_rmw_graph_writer();
    writer.add("/good", "pkg/msg/Good").expect("pair");

    let node = raw_node(&root.config());
    let service = raw_registry(&node);
    let raw = service
        .publisher_builder()
        .initial_max_slice_len(RMW_GRAPH_FRAME_LEN)
        .create()
        .expect("raw publisher");
    let raw_id = raw.id().value();
    let mut bad_magic =
        encode_snapshot(raw_id, 1, [("/x", "pkg/msg/X")]).expect("encode")[0].clone();
    bad_magic[0..2].copy_from_slice(&[0xCE, 0x37]);
    let hostile: Vec<Vec<u8>> = vec![
        vec![0xCE, 0x52, 0x01],
        bad_magic,
        // Header for part 0 of 1 claiming 65535 entries, with no entry bytes.
        vec![
            0xCE, 0x52, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
            0xFF, 0xFF,
        ],
    ];
    let valid = encode_snapshot(raw_id, 1, [("/raw/ok", "pkg/msg/Ok")]).expect("encode");

    // The gather's subscriber exists only while it runs, so the raw peer
    // must keep sending through the gather. The thread is joined before any
    // assertion.
    let stop = Arc::new(AtomicBool::new(false));
    let sender = {
        let stop = Arc::clone(&stop);
        std::thread::spawn(move || {
            while !stop.load(Ordering::Relaxed) {
                for frame in hostile.iter().chain(valid.iter()) {
                    let sample = raw.loan_slice_uninit(frame.len()).expect("loan");
                    sample.write_from_slice(frame).send().expect("send");
                }
                std::thread::sleep(Duration::from_millis(5));
            }
        })
    };
    let got = reader.gather_rmw_graph(None, SETTLE_WINDOW);
    stop.store(true, Ordering::Relaxed);
    sender.join().expect("sender thread");

    assert_eq!(
        got.expect("gather"),
        settled(&[("/good", &["pkg/msg/Good"]), ("/raw/ok", &["pkg/msg/Ok"])])
    );
    assert!(logs_contain("rmw graph registry degraded"));
    assert!(logs_contain("what=\"decode\""));
}

#[test]
#[serial]
fn a_silent_peer_makes_the_gather_incomplete_and_keeps_what_was_heard() {
    let root = IsolatedRoot::mint("rg_l");
    let tm = manager("silent_w", &root.config());
    let reader = manager("silent_r", &root.config());
    let writer = tm.open_rmw_graph_writer();
    writer.add("/heard", "pkg/msg/Heard").expect("pair");

    let node = raw_node(&root.config());
    let service = raw_registry(&node);
    let _silent = service.publisher_builder().create().expect("raw publisher");

    assert_eq!(
        gather(&reader, None, SHORT_WINDOW),
        RmwGraphGather {
            topics: map(&[("/heard", &["pkg/msg/Heard"])]),
            completeness: GatherCompleteness::Incomplete {
                live_writers: 2,
                writers_heard: 1,
            },
        }
    );
}

/// The child half of the killed-writer arm: a writer with one pair on the
/// parent's root, held until the parent kills this process.
#[test]
#[ignore = "child process entry point, driven by the parent tests in this file"]
fn subprocess_child_holds_a_graph_writer() {
    let Some(config) = child_iceoryx_config() else {
        return;
    };
    let tm = manager("child_w", &config);
    let writer = tm.open_rmw_graph_writer();
    writer.add(CHILD_TOPIC, CHILD_TYPE).expect("pair");
    assert!(writer.writer_id().is_some(), "child writer is live");
    println!("{CHILD_READY}");
    use std::io::Write;
    std::io::stdout().flush().expect("flush the ready marker");
    for _ in 0..600 {
        std::thread::sleep(Duration::from_millis(100));
    }
}

#[test]
#[serial]
fn a_killed_writer_is_not_waited_for_and_its_pairs_are_gone() {
    let root = IsolatedRoot::mint("rg_m");
    let reader = manager("kill_r", &root.config());
    let live = manager("kill_w", &root.config());
    let writer = live.open_rmw_graph_writer();
    writer
        .add("/parent/odom", "nav_msgs/msg/Odometry")
        .expect("pair");

    kill_child_holding_ports(
        "subprocess_child_holds_a_graph_writer",
        &root,
        CHILD_READY,
        Duration::from_millis(200),
    );

    let node = raw_node(&root.config());
    let service = raw_registry(&node);
    let mut publishers = 0usize;
    service.dynamic_config().list_publishers(|_| {
        publishers += 1;
        CallbackProgression::Continue
    });
    assert_eq!(
        publishers, 2,
        "precondition: the killed child's publisher is still listed beside the live one"
    );

    assert_eq!(
        gather(&reader, None, SETTLE_WINDOW),
        settled(&[("/parent/odom", &["nav_msgs/msg/Odometry"])])
    );
}
