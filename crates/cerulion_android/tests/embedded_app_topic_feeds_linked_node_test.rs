// SPDX-License-Identifier: AGPL-3.0-only
//! An app-published topic feeds a linked graph node through an absolute
//! `source:`.
//!
//! The runtime creates the app's `publish` topics before it builds the graph,
//! so the node attaches to a topic the app already owns, and creates each one
//! with a queue as deep as the deepest graph input that reads it. The steps:
//!
//! - a trigger input gets every frame, one tick per frame;
//! - a latest-value input of a period node holds each frame;
//! - a node that reports an input deeper than 64 frames fails the start with
//!   the depth limit named;
//! - a trigger input 64 frames deep, the deepest an input can declare, gets a
//!   burst of 64 frames that piles up in its queue, with none lost.
//!
//! It runs under both transports: the default cross-process one, and the
//! in-process one an app uses (`--features local-transport`). The node
//! registry and the transport are process state, so this file is its own
//! binary and holds ONE test that runs its steps in order.

use std::time::{Duration, Instant};

use cerulion_android::{codes, EmbedError, EmbeddedRuntime, RuntimeState};
use cerulion_core::graph::{NodeContext, NodeEntry, NodeInfo};
use cerulion_core::transport::TransportManager;
use cerulion_core::TransportResult;
use serde_json::json;
use tempfile::tempdir;
use test_node_macro_relay_rlib::{hold_deep_relay, DeepRelayNodeEntry};

cerulion_android::embed_nodes! {
    "test_node_macro_relay_rlib" => test_node_macro_relay_rlib::RelayNodeEntry,
    "test_node_macro_deep_relay" => DeepRelayNodeEntry,
    "test_node_macro_period_input_cdylib" => test_node_macro_period_input_cdylib::PeriodInputNodeEntry,
    "too_deep_relay" => TooDeepRelayEntry,
}

/// The deep relay, reporting its trigger input one frame deeper than any
/// input may declare. The node macro refuses such a declaration at compile
/// time, so only a node entry written by hand can report it.
struct TooDeepRelayEntry(DeepRelayNodeEntry);

impl TooDeepRelayEntry {
    fn new() -> Self {
        Self(DeepRelayNodeEntry::new())
    }
}

impl NodeEntry for TooDeepRelayEntry {
    fn info(&self) -> TransportResult<NodeInfo> {
        let info = self.0.info()?;
        let inputs = info
            .input_meta()
            .iter()
            .cloned()
            .map(|mut meta| {
                meta.depth = 65;
                meta
            })
            .collect();
        Ok(NodeInfo::with_meta(inputs, info.output_meta().to_vec()))
    }

    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        self.0.init(context)
    }

    fn tick(&mut self) -> TransportResult<()> {
        self.0.tick()
    }
}

const JNI_VERSION_1_6: i32 = 0x0001_0006;
const VECTOR3: &str = "geometry_msgs/Vector3";

/// A graph of one node whose input `input` reads the app topic `source`
/// through an absolute reference, and whose output is `output`.
fn graph_yaml(prefix: &str, node_type: &str, input: &str, source: &str, output: &str) -> String {
    format!(
        "prefix: {prefix}
nodes:
  - id: logic
    type: {node_type}
    inputs:
      - name: {input}
        source: {source}
    outputs:
      - name: {output}
        schema: {VECTOR3}
"
    )
}

fn start_config(graph_yaml: &str, app_topic: &str) -> String {
    json!({
        "graph_yaml": graph_yaml,
        "publish": [{"topic": app_topic, "schema": VECTOR3}],
    })
    .to_string()
}

fn publish_x(runtime: &mut EmbeddedRuntime, topic: &str, x: f64) {
    runtime
        .publish(topic, &json!({"x": x, "y": 0.0, "z": 0.0}).to_string())
        .unwrap_or_else(|error| panic!("publish x = {x} on {topic}: {error}"));
}

/// `(x, z)` of one Vector3 frame.
fn x_and_z(payload: &[u8]) -> (f64, f64) {
    let x = f64::from_le_bytes(payload[0..8].try_into().expect("x bytes"));
    let z = f64::from_le_bytes(payload[16..24].try_into().expect("z bytes"));
    (x, z)
}

fn assert_running_one_node(runtime: &EmbeddedRuntime) {
    let status = runtime.status_handle().lock().unwrap().clone();
    assert_eq!(status.state, RuntimeState::Running, "{status:?}");
    assert_eq!(status.nodes, 1, "{status:?}");
}

/// The relay ticks once per frame on its trigger input: it copies `x` and
/// writes its tick count into `z`. A burst of app frames must come out as one
/// echo per frame, in publish order, and no more.
fn trigger_input_gets_every_app_frame_in_order(runtime: &mut EmbeddedRuntime, pid: u32) {
    let app_topic = format!("/app_feed_trigger_{pid}/cmd");
    let prefix = format!("app_feed_trigger_graph_{pid}");
    let echo_topic = format!("/{prefix}/logic/echo");
    let graph = graph_yaml(
        &prefix,
        "test_node_macro_relay_rlib",
        "cmd",
        &app_topic,
        "echo",
    );
    runtime
        .start(&start_config(&graph, &app_topic))
        .expect("a graph that reads an app topic on a trigger input starts");
    assert_running_one_node(runtime);

    let subscriber = TransportManager::get()
        .expect("transport manager")
        .create_subscriber(&echo_topic)
        .expect("subscribe to the relay output");
    for x in [10.0, 20.0, 30.0, 40.0, 50.0] {
        publish_x(runtime, &app_topic, x);
    }
    let mut echoes = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(10);
    while echoes.len() < 5 && Instant::now() < deadline {
        subscriber
            .wait_for_message(Duration::from_millis(200), |message| {
                echoes.push(x_and_z(message.payload()));
            })
            .expect("wait for relay output");
    }
    // One tick per frame: nothing more arrives once the five were served.
    let quiet_until = Instant::now() + Duration::from_millis(300);
    while Instant::now() < quiet_until {
        subscriber
            .wait_for_message(Duration::from_millis(50), |message| {
                echoes.push(x_and_z(message.payload()));
            })
            .expect("wait for relay output");
    }
    assert_eq!(
        echoes,
        [
            (10.0, 1.0),
            (20.0, 2.0),
            (30.0, 3.0),
            (40.0, 4.0),
            (50.0, 5.0)
        ]
    );
    runtime.stop().expect("stop the trigger graph");
}

/// The period node copies its latest-value input's `x` into every output
/// frame. Until the app publishes, the node produces nothing; after each
/// publish its output holds the new value. Each value is published once the
/// node has shown the previous one on two ticks (a second frame with no new
/// publish is the hold), so the node must show every value, in order, and
/// never a value the app did not send.
fn latest_value_input_holds_each_app_frame_in_order(runtime: &mut EmbeddedRuntime, pid: u32) {
    let app_topic = format!("/app_feed_latest_{pid}/cmd");
    let prefix = format!("app_feed_latest_graph_{pid}");
    let out_topic = format!("/{prefix}/logic/out");
    let graph = graph_yaml(
        &prefix,
        "test_node_macro_period_input_cdylib",
        "inp",
        &app_topic,
        "out",
    );
    runtime
        .start(&start_config(&graph, &app_topic))
        .expect("a graph that reads an app topic on a latest-value input starts");
    assert_running_one_node(runtime);

    let subscriber = TransportManager::get()
        .expect("transport manager")
        .create_subscriber(&out_topic)
        .expect("subscribe to the period node output");
    // About 20 period ticks with nothing published: the node must produce
    // nothing, never a default value it was not sent.
    let mut early = 0;
    let quiet_until = Instant::now() + Duration::from_millis(200);
    while Instant::now() < quiet_until {
        subscriber
            .wait_for_message(Duration::from_millis(50), |_| early += 1)
            .expect("wait for period node output");
    }
    assert_eq!(early, 0, "the period node sent output before any publish");
    // `(x, frames)`: each value the node showed, and on how many frames.
    let mut shown: Vec<(f64, usize)> = Vec::new();
    for x in [1.5, 2.5, 3.5, 4.5, 5.5] {
        publish_x(runtime, &app_topic, x);
        let deadline = Instant::now() + Duration::from_secs(10);
        while shown
            .last()
            .is_none_or(|&(value, frames)| value != x || frames < 2)
            && Instant::now() < deadline
        {
            subscriber
                .wait_for_message(Duration::from_millis(200), |message| {
                    let (value, _) = x_and_z(message.payload());
                    match shown.last_mut() {
                        Some((last, frames)) if *last == value => *frames += 1,
                        _ => shown.push((value, 1)),
                    }
                })
                .expect("wait for period node output");
        }
    }
    let values: Vec<f64> = shown.iter().map(|&(value, _)| value).collect();
    assert_eq!(values, [1.5, 2.5, 3.5, 4.5, 5.5], "{shown:?}");
    assert!(
        shown.iter().all(|&(_, frames)| frames >= 2),
        "each value must be held over at least two ticks: {shown:?}"
    );
    runtime.stop().expect("stop the latest-value graph");
}

/// The deep relay's trigger input queues 64 frames. Its ticks are held while
/// the app publishes a burst of 64, so at least 63 frames wait in the queue
/// at once (a 16-frame queue would drop the oldest of them). Frame `i`
/// carries `x = i`, and the relay writes its tick count into `z`, so a lost
/// frame leaves every later echo with `x` above `z`. The test subscriber's
/// own queue is shallower than the burst and can drop old echoes, but it
/// always keeps the newest one, `(64, 64)`.
fn deep_trigger_input_gets_a_burst_of_64_app_frames(runtime: &mut EmbeddedRuntime, pid: u32) {
    let app_topic = format!("/app_feed_deep_{pid}/cmd");
    let prefix = format!("app_feed_deep_graph_{pid}");
    let echo_topic = format!("/{prefix}/logic/echo");
    let graph = graph_yaml(
        &prefix,
        "test_node_macro_deep_relay",
        "cmd",
        &app_topic,
        "echo",
    );
    let hold = hold_deep_relay();
    runtime
        .start(&start_config(&graph, &app_topic))
        .expect("a graph that reads an app topic on a 64-deep trigger input starts");
    assert_running_one_node(runtime);

    let subscriber = TransportManager::get()
        .expect("transport manager")
        .create_subscriber(&echo_topic)
        .expect("subscribe to the deep relay output");
    for i in 1..=64 {
        publish_x(runtime, &app_topic, f64::from(i));
    }
    drop(hold);
    let mut echoes = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(10);
    while echoes.last() != Some(&(64.0, 64.0)) && Instant::now() < deadline {
        subscriber
            .wait_for_message(Duration::from_millis(200), |message| {
                echoes.push(x_and_z(message.payload()));
            })
            .expect("wait for deep relay output");
    }
    // One tick per frame: nothing more arrives once the 64 were served.
    let quiet_until = Instant::now() + Duration::from_millis(300);
    while Instant::now() < quiet_until {
        subscriber
            .wait_for_message(Duration::from_millis(50), |message| {
                echoes.push(x_and_z(message.payload()));
            })
            .expect("wait for deep relay output");
    }
    assert_eq!(echoes.last(), Some(&(64.0, 64.0)), "{echoes:?}");
    assert!(
        echoes.iter().all(|&(x, z)| x == z),
        "an echo with x != z means a frame was lost or reordered: {echoes:?}"
    );
    assert!(
        echoes.windows(2).all(|pair| pair[0].1 < pair[1].1),
        "echoes arrive in tick order: {echoes:?}"
    );
    runtime.stop().expect("stop the deep trigger graph");
}

/// An app topic read by an input deeper than 64 frames fails the start with
/// a graph error that names the input, its depth and the limit, before the
/// app creates the topic.
fn an_input_deeper_than_64_fails_the_start_with_the_limit_named(
    runtime: &mut EmbeddedRuntime,
    pid: u32,
) {
    let app_topic = format!("/app_feed_too_deep_{pid}/cmd");
    let prefix = format!("app_feed_too_deep_graph_{pid}");
    let graph = graph_yaml(&prefix, "too_deep_relay", "cmd", &app_topic, "echo");
    let error = runtime
        .start(&start_config(&graph, &app_topic))
        .expect_err("an input deeper than 64 frames cannot start");
    let EmbedError::Graph(message) = &error else {
        panic!("expected the depth refusal, got: {error}");
    };
    assert!(
        message.contains(&format!(
            "input 'logic.cmd' on topic '{app_topic}' declares depth = 65; the maximum queue \
             depth is 64"
        )),
        "expected the depth refusal for 'logic.cmd', got: {message}"
    );
    let status = runtime.status_handle().lock().unwrap().clone();
    assert_eq!(status.state, RuntimeState::Failed, "{status:?}");
    let last_error = status
        .last_error
        .as_ref()
        .expect("a failed start records its error");
    assert_eq!(last_error.code, codes::GRAPH, "{status:?}");
    assert_eq!(&last_error.message, message, "{status:?}");
}

#[test]
fn an_app_topic_feeds_a_linked_node_through_an_absolute_source() {
    assert_eq!(
        JNI_OnLoad(std::ptr::null_mut(), std::ptr::null_mut()),
        JNI_VERSION_1_6
    );
    let temp = tempdir().unwrap();
    let mut runtime = EmbeddedRuntime::new(temp.path().to_path_buf(), "android_app_feed".into())
        .expect("runtime");
    let pid = std::process::id();
    trigger_input_gets_every_app_frame_in_order(&mut runtime, pid);
    latest_value_input_holds_each_app_frame_in_order(&mut runtime, pid);
    an_input_deeper_than_64_fails_the_start_with_the_limit_named(&mut runtime, pid);
    deep_trigger_input_gets_a_burst_of_64_app_frames(&mut runtime, pid);
}
