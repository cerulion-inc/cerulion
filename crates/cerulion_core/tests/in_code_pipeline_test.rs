// SPDX-License-Identifier: AGPL-3.0-only
//! In-code multi-node pipelines, kept as TEST code.
//!
//! These two programs used to live under `crates/cerulion_core/examples/` as
//! `basic_timer.rs` and `perception_pipeline.rs`, and another test ran them
//! with `cargo run --example`. They are not how an application is written:
//! several node types share one file, the graph is a YAML string assembled in
//! code, and the runtime is built and stepped by hand. That shape belongs
//! under `tests/` only. The user-facing version of the two-node pipeline is
//! the workspace `examples/basic_timer/` (one crate per node type, a graph
//! file, run through the `cerulion` verbs), and the multi-rate shape of the
//! four-node pipeline is the `imu` node of `examples/perception/`.
//!
//! What this file keeps is the regression value the examples carried:
//!
//! 1. A macro-defined pipeline builds and steps to completion over the real
//!    shared-memory transport.
//! 2. `GraphRuntime::step(delta)` advances the clock exactly once per step.
//!    The old examples called `clock.advance(delta)` after `step(delta)`, which
//!    double counted time; the oracles below are computed from a single
//!    advance and fail if the clock moves twice.
//! 3. The camera and detector pair publishes fully initialised frames and the
//!    detector counts exactly the lit pixels.
//!
//! Every oracle is written out by hand from the period arithmetic, never
//! compared against a second run.
//!
//! Each test builds its runtime with `GraphRuntime::build_for_test`, which
//! roots the transport at a unique shared-memory prefix, so the file needs
//! neither `#[serial]` nor `--test-threads=1`.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::{parse_graph, validate_graph, GraphRuntime};
use cerulion_core::prelude::*;
use cerulion_core::testing::TestTransport;
use indexmap::IndexMap;
use native_ros2_messages::sensor_msgs::Image;
use native_ros2_messages::std_msgs::Int32;

// ---- Two-node pipeline: timer -> printer -----------------------------------

/// Values the two-node sink received, in arrival order.
static TIMER_SINK: Mutex<Vec<i32>> = Mutex::new(Vec::new());

#[cerulion_node(period_ms = 100)]
struct Timer {
    tick_count: i32,
    #[output]
    count: Int32,
}

#[cerulion_node_impl]
impl Timer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.tick_count += 1;
        self.count.data = self.tick_count;
        Ok(())
    }
}

#[cerulion_node]
struct TimerSink {
    #[input(trigger)]
    count: Int32,
}

#[cerulion_node_impl]
impl TimerSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let value = self.count.data;
        TIMER_SINK.lock().unwrap().push(value);
        Ok(())
    }
}

#[test]
fn two_node_pipeline_steps_to_completion_with_a_single_clock_advance() {
    let yaml = r#"
prefix: test/in_code/basic_timer
nodes:
  - id: timer
    type: timer
    outputs:
      - name: count
        schema: std_msgs/Int32
  - id: printer
    type: timer_sink
    inputs:
      - name: count
        source: timer/count
"#;
    let config = parse_graph(yaml).expect("parse");
    validate_graph(&config).expect("validate");

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("timer".to_string(), Box::new(TimerEntry::new()));
    factories.insert("printer".to_string(), Box::new(TimerSinkEntry::new()));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock.clone(), 8).expect("build");

    TIMER_SINK.lock().unwrap().clear();
    const STEPS: u64 = 5;
    for _ in 0..STEPS {
        // `step` advances the clock itself. Advancing it again here is the
        // double count this test exists to catch.
        runtime.step(Duration::from_millis(100));
    }

    // Single advance: 5 steps of 100 ms is 500 ms of graph time, exactly.
    assert_eq!(
        clock.now_ns(),
        STEPS * 100_000_000,
        "the clock must advance once per step"
    );
    // A 100 ms period fires once per 100 ms step.
    assert_eq!(
        runtime.node_handle("timer").expect("timer").fire_count(),
        STEPS
    );
    // The sink sits one DAG level below the timer, so each publish reaches it
    // inside the same step: five values, in order, none skipped.
    assert_eq!(
        runtime
            .node_handle("printer")
            .expect("printer")
            .fire_count(),
        STEPS
    );
    assert_eq!(*TIMER_SINK.lock().unwrap(), vec![1, 2, 3, 4, 5]);

    runtime.shutdown();
}

// ---- Four-node pipeline: two sources at different rates ---------------------

/// Bright-pixel counts the four-node sink received, in arrival order.
static PIXEL_SINK: Mutex<Vec<i32>> = Mutex::new(Vec::new());

#[cerulion_node(period_ms = 33)]
struct Camera {
    frame: u32,
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl Camera {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.frame = self.frame.wrapping_add(1);
        // A valid, small mono8 frame. Every byte of the loan is written: its
        // declared length is published in full.
        self.image.height = 64;
        self.image.width = 64;
        self.image.step = 64;
        self.image.is_bigendian = 0;
        self.image.header.frame_id = "camera";
        self.image.encoding = "mono8";
        let pixels = self.image.loan_data(64 * 64)?;
        pixels.fill(0);
        let lit = self.frame as usize % (64 * 64 + 1);
        pixels[..lit].fill(255);
        Ok(())
    }
}

#[cerulion_node(period_ms = 10)]
struct Imu {
    sample: u32,
    #[output]
    reading: Int32,
}

#[cerulion_node_impl]
impl Imu {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.sample += 1;
        self.reading.data = self.sample as i32;
        Ok(())
    }
}

#[cerulion_node]
struct Detector {
    #[input(trigger)]
    image: Image,
    #[output]
    count: Int32,
}

#[cerulion_node_impl]
impl Detector {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Count bright pixels through the borrowed view: no image copy.
        let bright = self
            .image
            .data()
            .iter()
            .filter(|&&pixel| pixel >= 128)
            .count() as i32;
        self.count.data = bright;
        Ok(())
    }
}

#[cerulion_node]
struct PixelSink {
    #[input(trigger)]
    count: Int32,
}

#[cerulion_node_impl]
impl PixelSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let value = self.count.data;
        PIXEL_SINK.lock().unwrap().push(value);
        Ok(())
    }
}

#[test]
fn four_node_multi_rate_pipeline_steps_to_completion() {
    let yaml = r#"
prefix: test/in_code/perception
nodes:
  - id: camera
    type: camera
    outputs:
      - name: image
        schema: sensor_msgs/Image
        max_slice_len: 8192
  - id: imu
    type: imu
    outputs:
      - name: reading
        schema: std_msgs/Int32
  - id: detector
    type: detector
    inputs:
      - name: image
        source: camera/image
    outputs:
      - name: count
        schema: std_msgs/Int32
  - id: printer
    type: pixel_sink
    inputs:
      - name: count
        source: detector/count
"#;
    let config = parse_graph(yaml).expect("parse");
    validate_graph(&config).expect("validate");

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("camera".to_string(), Box::new(CameraEntry::new()));
    factories.insert("imu".to_string(), Box::new(ImuEntry::new()));
    factories.insert("detector".to_string(), Box::new(DetectorEntry::new()));
    factories.insert("printer".to_string(), Box::new(PixelSinkEntry::new()));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock.clone(), 8).expect("build");

    PIXEL_SINK.lock().unwrap().clear();
    const STEPS: u64 = 30;
    for _ in 0..STEPS {
        runtime.step(Duration::from_millis(10));
    }

    // 30 steps of 10 ms is 300 ms of graph time, advanced once per step.
    assert_eq!(clock.now_ns(), STEPS * 10_000_000);
    // The 10 ms source fires on every step; the 33 ms source fires at 33, 66,
    // ... 297 ms, which is floor(300 / 33) = 9 times. Two rates, one graph.
    assert_eq!(runtime.node_handle("imu").expect("imu").fire_count(), 30);
    assert_eq!(
        runtime.node_handle("camera").expect("camera").fire_count(),
        9
    );
    // The detector and the sink are data triggered down the chain, so each
    // fires once per camera frame, inside the step that published it.
    assert_eq!(
        runtime
            .node_handle("detector")
            .expect("detector")
            .fire_count(),
        9
    );
    // Frame n lights its first n pixels, so the counts are 1 through 9.
    assert_eq!(*PIXEL_SINK.lock().unwrap(), (1..=9).collect::<Vec<i32>>());

    runtime.shutdown();
}

// ---- The camera and detector pair, frame by frame ---------------------------

#[test]
fn synthetic_frames_are_fully_initialized_and_deliver_pixel_counts() {
    let transport = TestTransport::with_buffer_size(4);
    let image_pub = transport.publisher("image", MaxSliceLen::const_new(8192), 0);
    let image_input = transport.subscriber("image");
    let mut image_observer = transport.subscriber("image");
    let count_pub = transport.publisher("count", MaxSliceLen::const_new(256), 0);
    let mut count_observer = transport.subscriber("count");
    let mut camera = CameraEntry::new();
    camera
        .init(NodeContext::for_tests(
            [("image".into(), AnyPublisher::Ipc(image_pub))]
                .into_iter()
                .collect(),
            Default::default(),
        ))
        .unwrap();
    let mut detector = DetectorEntry::new();
    detector
        .init(NodeContext::for_tests(
            [("count".into(), AnyPublisher::Ipc(count_pub))]
                .into_iter()
                .collect(),
            [("image".into(), AnySubscriber::Ipc(image_input))]
                .into_iter()
                .collect(),
        ))
        .unwrap();
    // (frame counter before the tick, lit pixels after it). 4096 wraps to 0.
    for (before, expected) in [(0, 1), (1, 2), (4095, 4096), (4096, 0), (0, 1)] {
        camera.inner.frame = before;
        camera.tick().unwrap();
        assert_eq!(
            image_observer
                .try_view::<Image, _>(|image| {
                    assert_eq!((image.width, image.height, image.step), (64, 64, 64));
                    assert_eq!(image.encoding().unwrap(), "mono8");
                    assert_eq!(image.data().len(), 4096);
                    assert!(image.data()[..expected].iter().all(|&v| v == 255));
                    assert!(image.data()[expected..].iter().all(|&v| v == 0));
                    assert!(image.header_bytes().ends_with(b"camera"));
                })
                .unwrap(),
            Some(())
        );
        detector.tick().unwrap();
        assert_eq!(
            count_observer
                .try_view::<Int32, _>(|count| count.data)
                .unwrap(),
            Some(expected as i32)
        );
    }
    camera.shutdown().unwrap();
    detector.shutdown().unwrap();
}
