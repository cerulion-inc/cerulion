// SPDX-License-Identifier: AGPL-3.0-only
//! An app reads a graph output and an app topic through `subscribe` entries
//! and `poll`.
//!
//! The graph is one relay node that reads the app topic `cmd` on a trigger
//! input and writes `echo`, a graph output: each frame copies `x` and writes
//! the node's tick count into `z`. The app subscribes to both topics, so one
//! publish shows up once on each. It runs under both transports: the default
//! cross-process one, and the in-process one an app uses
//! (`--features local-transport`). The node registry and the transport are
//! process state, so this file is its own binary and holds ONE test that runs
//! its steps in order.

use std::time::{Duration, Instant};

use cerulion_android::codes;
use cerulion_android::{EmbeddedRuntime, RuntimeState};
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use serde_json::json;
use tempfile::tempdir;

cerulion_android::embed_nodes! {
    "test_node_macro_relay_rlib" => test_node_macro_relay_rlib::RelayNodeEntry,
}

const JNI_VERSION_1_6: i32 = 0x0001_0006;
const VECTOR3: &str = "geometry_msgs/Vector3";

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
/// the node, so there every frame is either polled or dropped.
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
    assert!(
        expected.ends_with(&echoes),
        "echoes must be the newest frames, in order: {echoes:?}"
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
            "depth 17 for '/poll/deep' exceeds the subscription queue limit 16",
        ),
        (
            json!({"subscribe": [{"topic": "/poll/pose", "schema": "geometry_msgs/Pose"}]}),
            "subscribe schema 'geometry_msgs/Pose' field position has type",
        ),
        (
            json!({"subscribe": [{"topic": "/poll/none", "schema": "no_msgs/Missing"}]}),
            "unknown subscribe schema 'no_msgs/Missing'",
        ),
        (
            json!({
                "publish": [{"topic": topics.cmd, "schema": VECTOR3}],
                "subscribe": [{"topic": topics.cmd, "schema": "std_msgs/String"}],
            }),
            "subscribe topic '",
        ),
        (
            json!({
                "graph_yaml": topics.graph,
                "publish": [{"topic": topics.cmd, "schema": VECTOR3}],
                "subscribe": [{"topic": topics.echo, "schema": "std_msgs/String"}],
            }),
            "subscribe topic '",
        ),
    ];
    for (config, message) in refusals {
        let error = runtime
            .start(&config.to_string())
            .expect_err("the subscription must be refused");
        assert_eq!(error.code(), codes::INVALID_ARGUMENT, "{config}");
        assert!(error.to_string().starts_with(message), "{config}: {error}");
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
    refuses_a_subscription_it_cannot_serve(&mut runtime, &topics);
}
