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
use tracing_test::traced_test;

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

/// A capture stamp no `builtin_interfaces/Time` can hold (`nanosec` of one
/// second), as a `capture_ms` value: [`publish`] writes it raw.
const BAD_STAMP: u64 = u64::MAX;

/// Publish one `PoseStamped` at WIRE time `wire_ms`, captured at `capture_ms`
/// (written into `header.stamp`), tagged with `capture_ms` in
/// `pose.position.x` so a fused set names its own frames. [`BAD_STAMP`]
/// writes `sec = 0, nanosec = 1_000_000_000`.
fn publish(
    publisher: &mut cerulion_core::CerulionPublisher,
    clock: &VirtualClock,
    wire_ms: u64,
    capture_ms: u64,
) {
    clock.set(wire_ms * MS);
    let mut p = publisher.loan_proxy::<PoseStamped>().expect("loan");
    p.pose.position.x = capture_ms as f64;
    let (sec, nanosec) = if capture_ms == BAD_STAMP {
        (0, 1_000_000_000)
    } else {
        let ns = capture_ms * MS;
        ((ns / 1_000_000_000) as i32, (ns % 1_000_000_000) as u32)
    };
    p.with_header(|h| {
        h.set_frame_id("base_link")?;
        h.stamp.sec = sec;
        h.stamp.nanosec = nanosec;
        Ok(())
    })
    .expect("write header");
}

/// Publishers on both topics of a built fusion graph.
struct Feed {
    cam: cerulion_core::CerulionPublisher,
    lidar: cerulion_core::CerulionPublisher,
}

impl Feed {
    fn new(runtime: &GraphRuntime) -> Self {
        let mgr = runtime.test_transport().expect("test transport parked");
        Self {
            cam: mgr
                .create_publisher(TOPIC_CAM, MaxSliceLen::const_new(128), 0)
                .expect("publisher on /cap/cam"),
            lidar: mgr
                .create_publisher(TOPIC_LIDAR, MaxSliceLen::const_new(128), 0)
                .expect("publisher on /cap/lidar"),
        }
    }

    fn send(&mut self, clock: &VirtualClock, frames: &[(&str, u64, u64)]) {
        for (topic, wire_ms, capture_ms) in frames {
            let publisher = if *topic == TOPIC_CAM {
                &mut self.cam
            } else {
                &mut self.lidar
            };
            publish(publisher, clock, *wire_ms, *capture_ms);
        }
    }
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
    // The feed outlives the step: a dropped publisher takes its frames along.
    let mut feed = Feed::new(&runtime);
    feed.send(&clock, frames);
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

/// How many captured log lines contain `needle`.
fn count_lines(lines: &[&str], needle: &str) -> usize {
    lines.iter().filter(|l| l.contains(needle)).count()
}

#[test]
#[serial]
#[traced_test]
fn a_camera_stamp_that_goes_backwards_is_dropped_counted_and_never_fused() {
    // Two backwards frames, each discarded by a different head op: one regime,
    // so one warning, a suppressed repeat, and a recovery line once the camera
    // contributes an in-order head again.
    let frames = [
        (TOPIC_CAM, 101, 100),
        (TOPIC_LIDAR, 103, 100),
        (TOPIC_CAM, 104, 95),
        (TOPIC_CAM, 201, 200),
        (TOPIC_LIDAR, 203, 200),
        (TOPIC_CAM, 204, 195),
        (TOPIC_CAM, 301, 300),
        (TOPIC_LIDAR, 303, 300),
        (TOPIC_CAM, 401, 400),
        (TOPIC_LIDAR, 403, 400),
    ];
    let (fused, handle) = run("capback", true, &frames);
    assert_eq!(
        fused,
        vec![
            (100.0, 100.0),
            (200.0, 200.0),
            (300.0, 300.0),
            (400.0, 400.0)
        ]
    );
    assert_eq!(handle.sync_capture_backwards_drops(TOPIC_CAM), 2);
    assert_eq!(handle.sync_capture_backwards_drops(TOPIC_LIDAR), 0);
    assert_eq!(handle.sync_capture_decode_failures(TOPIC_CAM), 0);
    logs_assert(|lines: &[&str]| {
        let loud = count_lines(lines, "older than one this input already contributed");
        let recovered = count_lines(lines, "in order again on this input");
        if (loud, recovered) == (1, 1) {
            Ok(())
        } else {
            Err(format!(
                "expected one warning and one recovery line, got {loud} and {recovered}"
            ))
        }
    });
}

#[test]
#[serial]
#[traced_test]
fn a_frame_whose_stamp_cannot_be_decoded_is_in_no_set_and_counted() {
    // `nanosec = 1_000_000_000` is no valid time. Each bad frame is
    // discarded by a different head op, the good frames around them still
    // fuse, and the regime logs one warning and one recovery line.
    let frames = [
        (TOPIC_CAM, 101, 100),
        (TOPIC_LIDAR, 103, 100),
        (TOPIC_CAM, 104, BAD_STAMP),
        (TOPIC_CAM, 201, 200),
        (TOPIC_LIDAR, 203, 200),
        (TOPIC_CAM, 204, BAD_STAMP),
        (TOPIC_CAM, 301, 300),
        (TOPIC_LIDAR, 303, 300),
        (TOPIC_CAM, 401, 400),
        (TOPIC_LIDAR, 403, 400),
    ];
    let (fused, handle) = run("capbad", true, &frames);
    assert_eq!(
        fused,
        vec![
            (100.0, 100.0),
            (200.0, 200.0),
            (300.0, 300.0),
            (400.0, 400.0)
        ]
    );
    assert_eq!(handle.sync_capture_decode_failures(TOPIC_CAM), 2);
    assert_eq!(handle.sync_capture_decode_failures(TOPIC_LIDAR), 0);
    assert_eq!(handle.sync_capture_backwards_drops(TOPIC_CAM), 0);
    logs_assert(|lines: &[&str]| {
        let loud = count_lines(lines, "could not be decoded");
        let recovered = count_lines(lines, "decode again on this input");
        if (loud, recovered) == (1, 1) {
            Ok(())
        } else {
            Err(format!(
                "expected one warning and one recovery line, got {loud} and {recovered}"
            ))
        }
    });
}

/// After the camera's 100 ms frame arrived, and before the lidar's: the
/// checkpoint both runs below cross. Then a backwards camera frame (95), a
/// lidar frame (103) it would pair with, and an ordinary pair at 200.
const BEFORE_CHECKPOINT: [(&str, u64, u64); 1] = [(TOPIC_CAM, 101, 100)];
const AFTER_CHECKPOINT: [(&str, u64, u64); 5] = [
    (TOPIC_CAM, 104, 95),
    (TOPIC_LIDAR, 105, 100),
    (TOPIC_LIDAR, 106, 103),
    (TOPIC_CAM, 201, 200),
    (TOPIC_LIDAR, 203, 200),
];

/// Build the capture fusion graph, optionally restore `restored` before the
/// first step, then run one step per frame batch. Returns the fused sets and
/// the node handle.
fn run_batches(
    prefix: &str,
    restored: Option<std::collections::BTreeMap<String, u64>>,
    batches: &[&[(&str, u64, u64)]],
) -> (Vec<(f64, f64)>, cerulion_core::scheduler::NodeHandle) {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = fusion_graph(prefix, Arc::clone(&sets), true, None);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build capture fusion graph");
    if let Some(heads) = restored {
        runtime.restore_sync_input_timestamps(&[("fuse".to_string(), heads)].into());
    }
    let mut feed = Feed::new(&runtime);
    for batch in batches {
        feed.send(&clock, batch);
        clock.set(0);
        runtime.step(Duration::from_millis(1));
    }
    let handle = runtime.node_handle("fuse").expect("node handle").clone();
    let fused = sets.lock().expect("sets sink poisoned").clone();
    (fused, handle)
}

#[test]
#[serial]
fn a_resumed_run_drops_the_same_backwards_frame_as_the_live_run() {
    // Live: the camera's 100 is held at the checkpoint, so 95 is backwards
    // and dropped, 103 finds no partner and 200 pairs with 200.
    let (live, live_handle) =
        run_batches("caplive", None, &[&BEFORE_CHECKPOINT, &AFTER_CHECKPOINT]);
    assert_eq!(live, vec![(100.0, 100.0), (200.0, 200.0)]);
    assert_eq!(live_handle.sync_capture_backwards_drops(TOPIC_CAM), 1);

    // Resumed from the checkpoint: the camera's head is restored as a stamp
    // (no frame behind it, so its set's body reads nothing). The restored
    // stamp seeds the high-water, so 95 is dropped exactly as live; without
    // the seed it would pair with 103 and add a set the live run never had.
    let restored = [(TOPIC_CAM.to_string(), 100 * MS)].into();
    let (resumed, resumed_handle) = run_batches("capresume", Some(restored), &[&AFTER_CHECKPOINT]);
    assert_eq!(
        resumed,
        vec![(200.0, 200.0)],
        "the live sets after the checkpoint"
    );
    assert_eq!(resumed_handle.sync_capture_backwards_drops(TOPIC_CAM), 1);
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
            data_floor: 64,
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

/// A fired pair, then a camera frame 5 ms behind the fired camera stamp
/// (backwards on a 10 ms window) and a lidar frame it would pair with, then an
/// ordinary pair at 200.
const FIRED_PAIR: [(&str, u64, u64); 2] = [(TOPIC_CAM, 101, 100), (TOPIC_LIDAR, 102, 100)];
const AFTER_THE_FIRE: [(&str, u64, u64); 4] = [
    (TOPIC_CAM, 104, 95),
    (TOPIC_LIDAR, 105, 97),
    (TOPIC_CAM, 201, 200),
    (TOPIC_LIDAR, 203, 200),
];

/// Build the capture fusion graph, hand it to `before_first_step` (a
/// restore), then run one step per batch, handing the runtime to
/// `after_first_step` once the first step ran (a checkpoint read).
fn run_with_checkpoint(
    prefix: &str,
    before_first_step: impl FnOnce(&mut GraphRuntime),
    batches: &[&[(&str, u64, u64)]],
    mut after_first_step: impl FnMut(&GraphRuntime),
) -> (Vec<(f64, f64)>, cerulion_core::scheduler::NodeHandle) {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = fusion_graph(prefix, Arc::clone(&sets), true, None);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build capture fusion graph");
    before_first_step(&mut runtime);
    let mut feed = Feed::new(&runtime);
    for (i, batch) in batches.iter().enumerate() {
        feed.send(&clock, batch);
        clock.set(0);
        runtime.step(Duration::from_millis(1));
        if i == 0 {
            after_first_step(&runtime);
        }
    }
    let handle = runtime.node_handle("fuse").expect("node handle").clone();
    let fused = sets.lock().expect("sets sink poisoned").clone();
    (fused, handle)
}

#[test]
#[serial]
fn a_run_resumed_right_after_a_fire_drops_the_same_backwards_frame_as_the_live_run() {
    use cerulion_core::state::VecSink;
    use cerulion_core::state_restore::NodeFrameworkState;

    // Live, with the checkpoint taken right after the 100 ms pair fired: the
    // camera's 95 is backwards and dropped, the lidar's 97 dies alone, and
    // 200 pairs with 200.
    let mut section = None;
    let (live, live_handle) = run_with_checkpoint(
        "capfirelive",
        |_| {},
        &[&FIRED_PAIR, &AFTER_THE_FIRE],
        |runtime| section = runtime.node_framework_state("fuse"),
    );
    assert_eq!(live, vec![(100.0, 100.0), (200.0, 200.0)]);
    assert_eq!(live_handle.sync_capture_backwards_drops(TOPIC_CAM), 1);

    // The checkpoint holds no head (both fired) but still states each
    // input's high-water, and the bytes carry it.
    let section = section.expect("the fusion node has a section");
    assert!(section.sync_input_timestamps.is_empty(), "both heads fired");
    assert_eq!(
        section.sync_capture_high_water,
        [
            (TOPIC_CAM.to_string(), 100 * MS),
            (TOPIC_LIDAR.to_string(), 100 * MS)
        ]
        .into()
    );
    let mut sink = VecSink::new();
    section.encode(&mut sink).expect("fits");
    let read = NodeFrameworkState::decode("fuse", &sink.into_inner()).expect("well-formed");
    // (This read seam leaves the transport's service table to the anchor
    // carrier, so only the two Sync tables are compared.)
    assert_eq!(
        (&read.sync_input_timestamps, &read.sync_capture_high_water),
        (
            &section.sync_input_timestamps,
            &section.sync_capture_high_water
        ),
        "the Sync tables round-trip"
    );

    // Resumed from those bytes: 95 is dropped exactly as live.
    let restore = |runtime: &mut GraphRuntime| {
        let per_node =
            |m: &std::collections::BTreeMap<String, u64>| [("fuse".to_string(), m.clone())].into();
        runtime.restore_sync_input_timestamps(&per_node(&read.sync_input_timestamps));
        runtime.restore_sync_capture_high_water(&per_node(&read.sync_capture_high_water));
    };
    let (resumed, resumed_handle) =
        run_with_checkpoint("capfireresume", restore, &[&AFTER_THE_FIRE], |_| {});
    assert_eq!(
        resumed,
        vec![(200.0, 200.0)],
        "the live sets after the fire"
    );
    assert_eq!(resumed_handle.sync_capture_backwards_drops(TOPIC_CAM), 1);

    // The control: resumed without the high-water, 95 pairs with 97, a set
    // the live run never fused. This is what makes the arm above
    // discriminating.
    let (unrestored, _) = run_with_checkpoint("capfirebare", |_| {}, &[&AFTER_THE_FIRE], |_| {});
    assert_eq!(unrestored, vec![(95.0, 97.0), (200.0, 200.0)]);
}

#[test]
#[serial]
fn a_capture_clock_restart_re_bases_the_partner_so_its_first_new_frame_fuses() {
    // 10 ms window. 100 and 95 fuse. Then the camera's clock restarts at 80
    // (20 ms back: a new epoch, not a late frame), and the lidar's 88 is the
    // first frame of the new epoch on its side. 88 is within a window of the
    // lidar's old 95, but the epoch reset forgot that high-water, so 88 is
    // not backwards and pairs with 80.
    let frames: [&[(&str, u64, u64)]; 3] = [
        &[(TOPIC_CAM, 101, 100), (TOPIC_LIDAR, 102, 95)],
        &[(TOPIC_CAM, 201, 80), (TOPIC_LIDAR, 202, 88)],
        &[],
    ];
    let (fused, handle) = run_with_checkpoint("capepoch", |_| {}, &frames, |_| {});
    assert_eq!(fused, vec![(100.0, 95.0), (80.0, 88.0)]);
    assert_eq!(handle.sync_capture_backwards_drops(TOPIC_LIDAR), 0);
    assert_eq!(handle.sync_capture_backwards_drops(TOPIC_CAM), 0);
}

#[test]
#[serial]
fn a_capture_clock_restart_right_after_a_resume_re_bases_the_node_as_the_live_run_does() {
    // 10 ms window. 100 and 95 fuse, and the lidar's 112 (too far from 100 to
    // join that set) stays held: the checkpoint is taken there. Then the
    // camera's clock restarts at 80 and the lidar's 88 is the first frame of
    // the new epoch on its side.
    //
    // Live: the camera's high-water is 100, so 80 is a restart and the
    // node voids the held 112 and fuses 80 with 88.
    let before: &[(&str, u64, u64)] = &[
        (TOPIC_CAM, 101, 100),
        (TOPIC_LIDAR, 102, 95),
        (TOPIC_LIDAR, 103, 112),
    ];
    let after: &[(&str, u64, u64)] = &[(TOPIC_CAM, 201, 80), (TOPIC_LIDAR, 202, 88)];
    let mut section = None;
    let (live, live_handle) = run_with_checkpoint(
        "caprestartlive",
        |_| {},
        &[before, after, &[]],
        |runtime| section = runtime.node_framework_state("fuse"),
    );
    assert_eq!(live, vec![(100.0, 95.0), (80.0, 88.0)]);
    assert_eq!(live_handle.sync_epoch_reset_discards(TOPIC_LIDAR), 1);

    let section = section.expect("the fusion node has a section");
    assert_eq!(
        section.sync_input_timestamps,
        [(TOPIC_LIDAR.to_string(), 112 * MS)].into(),
        "the lidar's 112 is held at the checkpoint"
    );
    assert_eq!(
        section.sync_capture_high_water,
        [
            (TOPIC_CAM.to_string(), 100 * MS),
            (TOPIC_LIDAR.to_string(), 112 * MS)
        ]
        .into()
    );

    // Resumed: the restored camera high-water must reach the scheduler's
    // epoch check too. Restored only into the capture layer, 80 is filled as
    // an ordinary head, the held 112 is kept, 80 dies as unmatchable and the
    // node never fuses again.
    let restore = |runtime: &mut GraphRuntime| {
        let per_node =
            |m: &std::collections::BTreeMap<String, u64>| [("fuse".to_string(), m.clone())].into();
        runtime.restore_sync_input_timestamps(&per_node(&section.sync_input_timestamps));
        runtime.restore_sync_capture_high_water(&per_node(&section.sync_capture_high_water));
    };
    let (resumed, resumed_handle) =
        run_with_checkpoint("caprestartresume", restore, &[after, &[]], |_| {});
    assert_eq!(
        resumed,
        vec![(80.0, 88.0)],
        "the live sets after the checkpoint"
    );
    assert_eq!(resumed_handle.sync_epoch_reset_discards(TOPIC_LIDAR), 1);
}
