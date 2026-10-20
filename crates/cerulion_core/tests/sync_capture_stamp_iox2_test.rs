// SPDX-License-Identifier: AGPL-3.0-only
//! Capture-time (`header.stamp`) Sync matching over real iceoryx2.
//!
//! Each frame carries two clocks: its WIRE stamp (the virtual clock at loan
//! time, set by hand before each publish) and its `header.stamp` (written into
//! the message). The two are deliberately far apart, so an oracle written in
//! capture stamps can only pass if the node matched on capture time, and a
//! publish-time control on the very same frames fuses something different.
//!
//! The stamp locations come from the real schema resolver over the built-in
//! corpus, and the frames from the real generated `PoseStamped` writer, so the
//! test also pins that the resolved location matches what producers write.
//!
//! `#[serial]`: real iceoryx2 over the process-global SHM singleton.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::codegen::layout::LayoutResolver;
use cerulion_core::codegen::parse_rosmsg;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef};
use cerulion_core::graph::node::{
    BackpressurePolicy, ClosureNodeEntry, InputMeta, MacroPolicy, NodeEntry, NodeInfo,
};
use cerulion_core::graph::validation::resolve_capture_sync_stamps;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::message::ShmMessage;
use cerulion_core::prelude::*;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::PoseStamped;
use serial_test::serial;

const TOPIC_CAM: &str = "/cap/cam";
const TOPIC_LIDAR: &str = "/cap/lidar";
const MS: u64 = 1_000_000;

/// Every `(cam, lidar)` pair a fire read, as the CAPTURE milliseconds each
/// frame carries in `pose.position.x`.
type Sets = Arc<Mutex<Vec<(f64, f64)>>>;

fn builtin_resolver() -> LayoutResolver {
    let mut schemas = Vec::new();
    for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
        if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
            schemas.push(s);
        }
    }
    LayoutResolver::new(schemas).0
}

/// A 2-trigger `sync_window_ms = 10` fusion node over `PoseStamped` inputs
/// `cam` and `lidar`. `capture` picks the matching clock; `locations`
/// overrides the resolved stamp map (to test the build refusal).
fn fusion_graph(
    prefix: &str,
    sets: Sets,
    capture: bool,
    locations: Option<IndexMap<String, cerulion_core::scheduler::sync_stamp::HeaderStampLocation>>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let meta = |name: &str| InputMeta {
        name: name.to_string(),
        schema_hash: <PoseStamped as ShmMessage>::SCHEMA_HASH,
        trigger: true,
        depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
        backpressure: BackpressurePolicy::DropOldest,
        expect_within_ms: None,
    };
    let mut info = NodeInfo::with_meta(vec![meta("cam"), meta("lidar")], vec![])
        .with_policy(MacroPolicy::Sync { window_ms: 10 });
    if capture {
        let resolved = locations.unwrap_or_else(|| {
            let found = resolve_capture_sync_stamps(
                "fuse",
                &[
                    ("cam", "geometry_msgs/PoseStamped"),
                    ("lidar", "geometry_msgs/PoseStamped"),
                ],
                &mut builtin_resolver(),
            )
            .expect("PoseStamped carries header.stamp");
            ["cam", "lidar"]
                .into_iter()
                .map(String::from)
                .zip(found)
                .collect()
        });
        info = info.with_sync_capture_stamps(resolved);
    }
    let entry = ClosureNodeEntry::new(info, move |ctx| {
        let mut read = |input: &str| {
            ctx.subscriber_mut(input).and_then(|s| {
                s.try_view::<PoseStamped, _>(|view| view.pose.position.x)
                    .ok()
                    .flatten()
            })
        };
        if let (Some(cam), Some(lidar)) = (read("cam"), read("lidar")) {
            sets.lock().expect("sets sink poisoned").push((cam, lidar));
        }
        Ok(())
    })
    .with_label("capture_fuse");

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("capture_{prefix}"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "fuse".to_string(),
            node_type: "capture_fuse".to_string(),
            inputs: vec![
                InputDef {
                    name: "cam".to_string(),
                    source: TOPIC_CAM.to_string(),
                },
                InputDef {
                    name: "lidar".to_string(),
                    source: TOPIC_LIDAR.to_string(),
                },
            ],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fuse".to_string(), Box::new(entry));
    (config, factories)
}

/// Publish one `PoseStamped` at WIRE time `wire_ms`, captured at `capture_ms`
/// (written into `header.stamp`), tagged with `capture_ms` in
/// `pose.position.x` so a fused set names its own frames.
fn publish(
    publisher: &mut cerulion_core::CerulionPublisher,
    clock: &VirtualClock,
    wire_ms: u64,
    capture_ms: u64,
) {
    clock.set(wire_ms * MS);
    let mut p = publisher.loan_proxy::<PoseStamped>().expect("loan");
    p.pose.position.x = capture_ms as f64;
    let ns = capture_ms * MS;
    p.with_header(|h| {
        h.set_frame_id("base_link")?;
        h.stamp.sec = (ns / 1_000_000_000) as i32;
        h.stamp.nanosec = (ns % 1_000_000_000) as u32;
        Ok(())
    })
    .expect("write header");
}

/// Build the graph, publish `frames` (`(topic, wire_ms, capture_ms)`) in
/// order, run one step, and return the fused sets plus the node handle.
fn run(
    prefix: &str,
    capture: bool,
    frames: &[(&str, u64, u64)],
) -> (Vec<(f64, f64)>, cerulion_core::scheduler::NodeHandle) {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = fusion_graph(prefix, Arc::clone(&sets), capture, None);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build capture fusion graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut cam = mgr
        .create_publisher(TOPIC_CAM, MaxSliceLen::const_new(128), 0)
        .expect("publisher on /cap/cam");
    let mut lidar = mgr
        .create_publisher(TOPIC_LIDAR, MaxSliceLen::const_new(128), 0)
        .expect("publisher on /cap/lidar");
    for (topic, wire_ms, capture_ms) in frames {
        let publisher = if *topic == TOPIC_CAM {
            &mut cam
        } else {
            &mut lidar
        };
        publish(publisher, &clock, *wire_ms, *capture_ms);
    }
    clock.set(0);
    runtime.step(Duration::from_millis(1));
    let handle = runtime.node_handle("fuse").expect("node handle");
    let fused = sets.lock().expect("sets sink poisoned").clone();
    (fused, handle.clone())
}

/// The camera publishes 40 ms after capture, the lidar 5 ms after: on publish
/// time the partners are 35 ms apart, past the 10 ms window.
const LATE_CAMERA: [(&str, u64, u64); 4] = [
    (TOPIC_LIDAR, 105, 100),
    (TOPIC_CAM, 140, 100),
    (TOPIC_LIDAR, 205, 200),
    (TOPIC_CAM, 240, 200),
];

#[test]
#[serial]
fn capture_time_matching_fuses_frames_captured_together() {
    let (fused, handle) = run("capok", true, &LATE_CAMERA);
    assert_eq!(fused, vec![(100.0, 100.0), (200.0, 200.0)]);
    assert_eq!(handle.sync_capture_backwards_drops(TOPIC_CAM), 0);
    assert_eq!(handle.sync_capture_decode_failures(TOPIC_CAM), 0);
}

#[test]
#[serial]
fn publish_time_matching_on_the_same_frames_fuses_nothing() {
    // The control: without the opt-in, every partner is 35 ms away on wire
    // stamps and dies on the 10 ms window. This is what makes the capture-time
    // oracle above discriminating.
    let (fused, _) = run("cappub", false, &LATE_CAMERA);
    assert_eq!(fused, vec![]);
}

#[test]
#[serial]
fn a_camera_stamp_that_goes_backwards_is_dropped_counted_and_never_fused() {
    let frames = [
        (TOPIC_CAM, 101, 100),
        (TOPIC_LIDAR, 103, 100),
        (TOPIC_CAM, 104, 95),
        (TOPIC_CAM, 201, 200),
        (TOPIC_LIDAR, 203, 200),
    ];
    let (fused, handle) = run("capback", true, &frames);
    assert_eq!(fused, vec![(100.0, 100.0), (200.0, 200.0)]);
    assert_eq!(handle.sync_capture_backwards_drops(TOPIC_CAM), 1);
    assert_eq!(handle.sync_capture_backwards_drops(TOPIC_LIDAR), 0);
}

#[test]
#[serial]
fn two_runs_on_the_same_frames_fuse_identical_sets() {
    let frames = [
        (TOPIC_CAM, 130, 100),
        (TOPIC_LIDAR, 101, 98),
        (TOPIC_LIDAR, 131, 128),
        (TOPIC_CAM, 160, 130),
        (TOPIC_CAM, 161, 125),
        (TOPIC_LIDAR, 201, 202),
        (TOPIC_CAM, 230, 200),
    ];
    let (first, h1) = run("capdet1", true, &frames);
    let (second, h2) = run("capdet2", true, &frames);
    assert_eq!(first, vec![(100.0, 98.0), (130.0, 128.0), (200.0, 202.0)]);
    assert_eq!(first, second);
    assert_eq!(h1.sync_capture_backwards_drops(TOPIC_CAM), 1);
    assert_eq!(
        h1.sync_capture_backwards_drops(TOPIC_CAM),
        h2.sync_capture_backwards_drops(TOPIC_CAM)
    );
}

#[test]
#[serial]
fn an_opted_in_node_missing_a_trigger_location_is_refused_at_build() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let only_cam: IndexMap<_, _> = [(
        "cam".to_string(),
        cerulion_core::scheduler::sync_stamp::HeaderStampLocation::Table {
            fixed_size: 56,
            index: 0,
            stamp_offset: 0,
        },
    )]
    .into_iter()
    .collect();
    let (config, factories) = fusion_graph("caprefuse", sets, true, Some(only_cam));
    let clock = Arc::new(VirtualClock::new());
    match GraphRuntime::build_for_test(config, factories, clock, 16) {
        Err(TransportError::SyncCaptureStamp { node_id, input, .. }) => {
            assert_eq!((node_id.as_str(), input.as_str()), ("fuse", "lidar"));
        }
        Err(other) => panic!("expected SyncCaptureStamp, got {other}"),
        Ok(_) => panic!("a node missing a trigger location must not build"),
    }
}
