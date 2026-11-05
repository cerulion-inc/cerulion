// SPDX-License-Identifier: AGPL-3.0-only
//! An app reads a graph output and an app topic through `subscribe` entries
//! and `poll`.
//!
//! The graph is one relay node that reads the app topic `cmd` on a trigger
//! input and writes `echo`, a graph output: each frame copies `x` and writes
//! the node's tick count into `z`. The app subscribes to both topics, so one
//! publish shows up once on each. A second graph holds one node that sends a
//! single frame from its `init` and never again, which reaches a subscription
//! only through the output's `history_size`. It runs under both transports:
//! the default cross-process one, and the in-process one an app uses
//! (`--features local-transport`). The node registry and the transport are
//! process state, so this file is its own binary and holds ONE test that runs
//! its steps in order.

use std::time::{Duration, Instant};

use cerulion_android::codes;
use cerulion_android::{EmbeddedRuntime, RuntimeState};
use cerulion_core::error::TransportResult;
use cerulion_core::graph::node::{NodeContext, NodeEntry, NodeInfo};
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::MacroPolicy;
use serde_json::json;
use tempfile::tempdir;

cerulion_android::embed_nodes! {
    "test_node_macro_relay_rlib" => test_node_macro_relay_rlib::RelayNodeEntry,
    "init_sender" => InitSender,
}

const JNI_VERSION_1_6: i32 = 0x0001_0006;
const VECTOR3: &str = "geometry_msgs/Vector3";

/// A node that sends one frame on `state` from its `init` and nothing
/// after: its period is an hour, and its tick sends nothing. It keeps its
/// context, so the publisher outlives the frame, and it forwards the
/// runtime's history pump to that context, as a macro node does.
#[derive(Default)]
struct InitSender {
    context: Option<NodeContext>,
}

impl InitSender {
    fn new() -> Self {
        Self::default()
    }
}

impl NodeEntry for InitSender {
    fn info(&self) -> TransportResult<NodeInfo> {
        Ok(
            NodeInfo::from_names(Vec::new(), vec!["state".to_string()]).with_policy(
                MacroPolicy::Period {
                    period_ms: 3_600_000,
                },
            ),
        )
    }

    fn init(&mut self, mut context: NodeContext) -> TransportResult<()> {
        let frame = cerulion_android::encode::encode_json(
            &schemas_layout(VECTOR3),
            r#"{"x":7.0}"#,
            64,
            0,
            0,
        )
        .expect("a Vector3 frame");
        context
            .publisher_mut("state")
            .expect("the state output")
            .publish_raw(&frame)?;
        self.context = Some(context);
        Ok(())
    }

    fn tick(&mut self) -> TransportResult<()> {
        Ok(())
    }

    fn pump_history(&mut self) {
        if let Some(context) = self.context.as_mut() {
            context.pump_history();
        }
    }

    fn pump_history_at(&mut self, now: Instant) {
        if let Some(context) = self.context.as_mut() {
            context.pump_history_at(now);
        }
    }
}

struct Topics {
    cmd: String,
    echo: String,
    graph: String,
}

fn topics(pid: u32) -> Topics {
    let prefix = format!("poll_graph_{pid}");
    let cmd = format!("/poll_{pid}/cmd");
    let graph = format!(
        "prefix: {prefix}
nodes:
  - id: logic
    type: test_node_macro_relay_rlib
    inputs:
      - name: cmd
        source: {cmd}
    outputs:
      - name: echo
        schema: {VECTOR3}
"
    );
    Topics {
        echo: format!("/{prefix}/logic/echo"),
        cmd,
        graph,
    }
}

fn start_config(topics: &Topics, depth: usize) -> String {
    json!({
        "graph_yaml": topics.graph,
        "publish": [{"topic": topics.cmd, "schema": VECTOR3}],
        "subscribe": [
            {"topic": topics.cmd, "schema": VECTOR3, "depth": depth},
            {"topic": topics.echo, "schema": VECTOR3, "depth": depth},
        ],
    })
    .to_string()
}

fn publish_x(runtime: &mut EmbeddedRuntime, topic: &str, x: f64) {
    runtime
        .publish(topic, &json!({"x": x, "y": 0.0, "z": 0.0}).to_string())
        .unwrap_or_else(|error| panic!("publish x = {x} on {topic}: {error}"));
}

fn poll(runtime: &mut EmbeddedRuntime, topic: &str) -> Option<String> {
    runtime
        .poll(topic)
        .unwrap_or_else(|error| panic!("poll {topic}: {error}"))
        .map(str::to_owned)
}

/// Poll `topic` until `count` messages arrived; the graph runs on its own
/// thread, so its output lands after the publish returns.
fn poll_n(runtime: &mut EmbeddedRuntime, topic: &str, count: usize) -> Vec<String> {
    let mut messages = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(10);
    while messages.len() < count {
        assert!(
            Instant::now() < deadline,
            "{topic}: {} of {count} messages within 10 s: {messages:?}",
            messages.len()
        );
        match poll(runtime, topic) {
            Some(message) => messages.push(message),
            None => std::thread::sleep(Duration::from_millis(5)),
        }
    }
    messages
}

/// `(polled, dropped, undecodable)` of a subscription in status.
fn counts(runtime: &EmbeddedRuntime, topic: &str) -> (u64, u64, u64) {
    let status = runtime.status_handle().lock().unwrap().clone();
    let entry = status
        .subscribed
        .get(topic)
        .unwrap_or_else(|| panic!("{topic} missing from status: {status:?}"));
    (entry.polled, entry.dropped, entry.undecodable)
}

fn vector(x: f64, z: f64) -> String {
    format!("{{\"x\":{x:?},\"y\":0.0,\"z\":{z:?}}}")
}

/// Each publish comes back once on the app topic and once, through the
/// node, on the graph output, in publish order. An empty queue polls as
/// nothing.
fn polls_every_message_in_order(runtime: &mut EmbeddedRuntime, topics: &Topics) {
    runtime
        .start(&start_config(topics, 4))
        .expect("a start with subscriptions");
    assert_eq!(
        runtime.status_handle().lock().unwrap().state,
        RuntimeState::Running
    );
    assert_eq!(poll(runtime, &topics.cmd), None);
    assert_eq!(poll(runtime, &topics.echo), None);
    assert_eq!(counts(runtime, &topics.cmd), (0, 0, 0));

    for x in [10.0, 20.0, 30.0] {
        publish_x(runtime, &topics.cmd, x);
    }
    assert_eq!(
        poll_n(runtime, &topics.cmd, 3),
        [
            r#"{"x":10.0,"y":0.0,"z":0.0}"#,
            r#"{"x":20.0,"y":0.0,"z":0.0}"#,
            r#"{"x":30.0,"y":0.0,"z":0.0}"#,
        ]
    );
    assert_eq!(poll(runtime, &topics.cmd), None);
    assert_eq!(
        poll_n(runtime, &topics.echo, 3),
        [
            r#"{"x":10.0,"y":0.0,"z":1.0}"#,
            r#"{"x":20.0,"y":0.0,"z":2.0}"#,
            r#"{"x":30.0,"y":0.0,"z":3.0}"#,
        ]
    );
    assert_eq!(counts(runtime, &topics.cmd), (3, 0, 0));
    assert_eq!(counts(runtime, &topics.echo), (3, 0, 0));
}

/// A burst larger than the depth keeps the newest `depth` messages and
/// counts every older one as dropped. On the graph output the poll races
/// the node, so there every frame is either polled, in order, or dropped.
fn overflow_drops_the_oldest_and_counts_them(runtime: &mut EmbeddedRuntime, topics: &Topics) {
    for x in 1..=7 {
        publish_x(runtime, &topics.cmd, f64::from(x));
    }
    assert_eq!(
        poll_n(runtime, &topics.cmd, 4),
        [
            vector(4.0, 0.0),
            vector(5.0, 0.0),
            vector(6.0, 0.0),
            vector(7.0, 0.0)
        ]
    );
    assert_eq!(poll(runtime, &topics.cmd), None);
    assert_eq!(counts(runtime, &topics.cmd), (7, 3, 0));

    // The node ticks once per frame, so its last echo carries z = 10.
    let mut echoes = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(10);
    while echoes.last() != Some(&vector(7.0, 10.0)) {
        assert!(Instant::now() < deadline, "echoes within 10 s: {echoes:?}");
        match poll(runtime, &topics.echo) {
            Some(message) => echoes.push(message),
            None => std::thread::sleep(Duration::from_millis(5)),
        }
    }
    let expected: Vec<String> = (1..=7)
        .map(|x| vector(f64::from(x), f64::from(x + 3)))
        .collect();
    // A poll can land between frames, so the echoes are an ordered
    // subsequence of the expected frames, ending with the last one.
    let mut rest = expected.iter();
    assert!(
        echoes
            .iter()
            .all(|echo| rest.by_ref().any(|frame| frame == echo)),
        "echoes must be expected frames, in order: {echoes:?}"
    );
    let (polled, dropped, undecodable) = counts(runtime, &topics.echo);
    assert_eq!(polled, 3 + echoes.len() as u64);
    assert_eq!(polled - 3 + dropped, 7, "every echo is polled or dropped");
    assert_eq!(undecodable, 0);
}

/// A frame of another schema on a subscribed topic is consumed, refused
/// with DECODE and counted; the next poll reads the frame after it.
fn a_frame_of_another_schema_is_consumed_and_counted(runtime: &mut EmbeddedRuntime, pid: u32) {
    let topic = format!("/poll_{pid}/foreign");
    let config = json!({
        "subscribe": [{"topic": topic, "schema": "std_msgs/String"}],
    })
    .to_string();
    runtime.start(&config).expect("a subscription-only start");
    let vector = schemas_layout(VECTOR3);
    let frame = cerulion_android::encode::encode_json(&vector, r#"{"x":1.0}"#, 64, 0, 0)
        .expect("a Vector3 frame");
    let slot = MaxSliceLen::try_new((WireHeader::SIZE + 64) as u32).expect("slot");
    let mut foreign = TransportManager::get()
        .expect("transport manager")
        .create_publisher(&topic, slot, 0)
        .expect("a foreign publisher");
    let string = schemas_layout("std_msgs/String");
    let after = cerulion_android::encode::encode_json(&string, r#"{"data":"after"}"#, 64, 0, 1)
        .expect("a String frame");
    foreign
        .publish_raw(&frame)
        .expect("publish the Vector3 frame");
    foreign
        .publish_raw(&after)
        .expect("publish the String frame");
    let error = runtime
        .poll(&topic)
        .expect_err("a Vector3 frame on a String topic");
    assert_eq!(error.code(), codes::DECODE);
    assert_eq!(
        poll(runtime, &topic).as_deref(),
        Some(r#"{"data":"after"}"#)
    );
    assert_eq!(poll(runtime, &topic), None);
    assert_eq!(counts(runtime, &topic), (1, 0, 1));
    drop(foreign);
    runtime.stop().expect("stop the subscription-only run");
}

/// The graph of one `init_sender` node; `history_size` goes under its
/// output when given.
fn init_sender_graph(prefix: &str, history_size: Option<usize>) -> String {
    let history = history_size
        .map(|size| format!("        history_size: {size}\n"))
        .unwrap_or_default();
    format!(
        "prefix: {prefix}
nodes:
  - id: boot
    type: init_sender
    outputs:
      - name: state
        schema: {VECTOR3}
{history}"
    )
}

/// A subscription attaches after the graph build, which ran the node's
/// `init`. So the frame the node sends from `init` is not in the queue when
/// the output keeps no history, and it arrives, once, when the output
/// declares `history_size`: the node's publisher delivers it to the late
/// joiner at the next loop pass.
fn an_init_frame_arrives_only_through_history(runtime: &mut EmbeddedRuntime, pid: u32) {
    let prefix = format!("poll_init_{pid}");
    let state = format!("/{prefix}/boot/state");
    let config = json!({
        "graph_yaml": init_sender_graph(&prefix, None),
        "subscribe": [{"topic": state, "schema": VECTOR3}],
    })
    .to_string();
    runtime
        .start(&config)
        .expect("a start whose node sends from init");
    assert_eq!(poll(runtime, &state), None);
    std::thread::sleep(Duration::from_millis(200));
    assert_eq!(poll(runtime, &state), None, "no history, no init frame");
    assert_eq!(counts(runtime, &state), (0, 0, 0));
    runtime.stop().expect("stop the init-send run");

    let prefix = format!("poll_init_history_{pid}");
    let state = format!("/{prefix}/boot/state");
    let config = json!({
        "graph_yaml": init_sender_graph(&prefix, Some(1)),
        "subscribe": [{"topic": state, "schema": VECTOR3}],
    })
    .to_string();
    runtime
        .start(&config)
        .expect("a start whose node sends from init, with history");
    assert_eq!(poll_n(runtime, &state, 1), [r#"{"x":7.0,"y":0.0,"z":0.0}"#]);
    assert_eq!(poll(runtime, &state), None);
    assert_eq!(counts(runtime, &state), (1, 0, 0));
    runtime.stop().expect("stop the init-send run with history");
}

/// The queue holds `depth` messages from each publisher of the topic, not
/// from all of them together: at depth 1, one frame from each of two
/// publishers both reach the app.
fn a_depth_holds_that_many_per_publisher(runtime: &mut EmbeddedRuntime, pid: u32) {
    let topic = format!("/poll_{pid}/two_publishers");
    let config = json!({
        "subscribe": [{"topic": topic, "schema": VECTOR3, "depth": 1}],
    })
    .to_string();
    runtime.start(&config).expect("a subscription-only start");
    let layout = schemas_layout(VECTOR3);
    let slot = MaxSliceLen::try_new((WireHeader::SIZE + 64) as u32).expect("slot");
    let manager = TransportManager::get().expect("transport manager");
    let mut first = manager
        .create_publisher(&topic, slot, 0)
        .expect("a first publisher");
    let mut second = manager
        .create_publisher(&topic, slot, 0)
        .expect("a second publisher");
    for (publisher, x) in [(&mut first, 1.0), (&mut second, 2.0)] {
        let frame =
            cerulion_android::encode::encode_json(&layout, &json!({"x": x}).to_string(), 64, 0, 0)
                .expect("a Vector3 frame");
        publisher.publish_raw(&frame).expect("publish a frame");
    }
    let mut polled = [poll(runtime, &topic), poll(runtime, &topic)];
    polled.sort();
    assert_eq!(
        polled,
        [Some(vector(1.0, 0.0)), Some(vector(2.0, 0.0))],
        "one frame from each publisher"
    );
    assert_eq!(poll(runtime, &topic), None);
    assert_eq!(counts(runtime, &topic), (2, 0, 0));
    drop((first, second));
    runtime.stop().expect("stop the two-publisher run");
}

fn schemas_layout(name: &str) -> cerulion_core::dynamic::WireLayout {
    let (schemas, _) = cerulion_core::dynamic::SchemaSet::from_schemas(
        cerulion_cli_engine::schema_cmd::parse_builtin_schemas(),
    )
    .expect("built-in schemas");
    schemas.layout(name).expect("a built-in schema").clone()
}

/// A start refuses a subscription it could never serve.
fn refuses_a_subscription_it_cannot_serve(runtime: &mut EmbeddedRuntime, topics: &Topics) {
    let refusals = [
        (
            json!({"subscribe": [{"topic": "/poll/deep", "schema": VECTOR3, "depth": 17}]}),
            "depth 17 for '/poll/deep' exceeds the subscription queue limit 16".to_string(),
        ),
        (
            // A subscription to a graph output attaches after the graph
            // build, and the refusal keeps its own code and text there.
            json!({
                "graph_yaml": topics.graph,
                "publish": [{"topic": topics.cmd, "schema": VECTOR3}],
                "subscribe": [{"topic": topics.echo, "schema": VECTOR3, "depth": 17}],
            }),
            format!(
                "depth 17 for '{}' exceeds the subscription queue limit 16",
                topics.echo
            ),
        ),
        (
            json!({"subscribe": [{"topic": "/poll/pose", "schema": "geometry_msgs/Pose"}]}),
            "subscribe schema 'geometry_msgs/Pose' field position has type Nested".to_string(),
        ),
        (
            json!({"subscribe": [{"topic": "/poll/none", "schema": "no_msgs/Missing"}]}),
            "unknown subscribe schema 'no_msgs/Missing'".to_string(),
        ),
        (
            json!({
                "publish": [{"topic": topics.cmd, "schema": VECTOR3}],
                "subscribe": [{"topic": topics.cmd, "schema": "std_msgs/String"}],
            }),
            format!(
                "subscribe topic '{}' has schema 'std_msgs/String', but the app publishes '{VECTOR3}'",
                topics.cmd
            ),
        ),
        (
            json!({
                "graph_yaml": topics.graph,
                "publish": [{"topic": topics.cmd, "schema": VECTOR3}],
                "subscribe": [{"topic": topics.echo, "schema": "std_msgs/String"}],
            }),
            format!(
                "subscribe topic '{}' has schema 'std_msgs/String', but a graph output declares '{VECTOR3}'",
                topics.echo
            ),
        ),
        (
            // The graph text names String; the node itself sends Vector3.
            json!({
                "graph_yaml": topics.graph.replace(
                    &format!("schema: {VECTOR3}"),
                    "schema: std_msgs/String",
                ),
                "publish": [{"topic": topics.cmd, "schema": VECTOR3}],
                "subscribe": [{"topic": topics.echo, "schema": "std_msgs/String"}],
            }),
            format!(
                "subscribe topic '{}' has schema 'std_msgs/String', but node 'logic' sends output 'echo' with another schema",
                topics.echo
            ),
        ),
    ];
    for (config, message) in refusals {
        let error = runtime
            .start(&config.to_string())
            .expect_err("the subscription must be refused");
        assert_eq!(error.code(), codes::INVALID_ARGUMENT, "{config}: {error}");
        assert!(error.to_string().starts_with(&message), "{config}: {error}");
    }
}

#[test]
fn an_app_polls_a_graph_output_and_an_app_topic() {
    assert_eq!(
        JNI_OnLoad(std::ptr::null_mut(), std::ptr::null_mut()),
        JNI_VERSION_1_6
    );
    let temp = tempdir().unwrap();
    let mut runtime =
        EmbeddedRuntime::new(temp.path().to_path_buf(), "android_poll".into()).expect("runtime");
    let pid = std::process::id();
    let topics = topics(pid);

    let error = runtime.poll(&topics.cmd).expect_err("no run yet");
    assert_eq!(error.code(), codes::BAD_STATE);

    polls_every_message_in_order(&mut runtime, &topics);
    overflow_drops_the_oldest_and_counts_them(&mut runtime, &topics);
    let error = runtime.poll("/poll/missing").expect_err("not subscribed");
    assert_eq!(error.code(), codes::UNKNOWN_TOPIC);
    runtime.stop().expect("stop");
    let error = runtime.poll(&topics.cmd).expect_err("stopped");
    assert_eq!(error.code(), codes::BAD_STATE);

    // A restart gets fresh queues and fresh counts, and the same messages
    // decode to the same text.
    polls_every_message_in_order(&mut runtime, &topics);
    runtime.stop().expect("stop the second run");

    a_frame_of_another_schema_is_consumed_and_counted(&mut runtime, pid);
    an_init_frame_arrives_only_through_history(&mut runtime, pid);
    a_depth_holds_that_many_per_publisher(&mut runtime, pid);
    refuses_a_subscription_it_cannot_serve(&mut runtime, &topics);
}
