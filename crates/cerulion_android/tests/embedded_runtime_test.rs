// SPDX-License-Identifier: AGPL-3.0-only
//! Requires `cargo build -p test_node_macro_period_cdylib` before running.

use std::path::{Path, PathBuf};
use std::time::Duration;

use cerulion_android::EmbeddedRuntime;
use cerulion_bag::BagReader;
use cerulion_core::{transport::TransportManager, wire::WireHeader};
use serde_json::json;
use tempfile::tempdir;

fn fixture_dir() -> PathBuf {
    let target_dir = std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target"));
    target_dir.join("debug")
}

fn config(node_lib_dir: &Path, prefix: &str, file: &str, topic: &str) -> String {
    let graph_yaml = format!(
        "prefix: {prefix}\nnodes:\n  - id: ticker\n    type: test_node_macro_period_cdylib\n    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n"
    );
    json!({
        "graph_yaml": graph_yaml,
        "node_lib_dir": node_lib_dir,
        "record": {"file": file},
        "publish": [{"topic": topic, "schema": "std_msgs/String"}]
    })
    .to_string()
}

fn string_payloads(path: &Path, topic: &str) -> Vec<String> {
    let reader = BagReader::open(path).expect("open finalized bag");
    let channels = reader.channels().expect("read channels");
    let graph_topic = channels
        .iter()
        .find(|channel| channel.topic.contains("/ticker/cmd"))
        .expect("graph output channel");
    assert_eq!(graph_topic.schema_name, "geometry_msgs/Vector3");
    let string_channel = channels
        .iter()
        .find(|channel| channel.topic == topic)
        .expect("publish channel");
    assert_eq!(string_channel.schema_name, "std_msgs/String");

    reader
        .user_message_index()
        .expect("index messages")
        .get(topic)
        .expect("String messages")
        .iter()
        .map(|span| {
            let frame = reader.frame(span);
            let body = &frame[WireHeader::SIZE..];
            let offset = u32::from_le_bytes(body[..4].try_into().unwrap()) as usize;
            let length = u32::from_le_bytes(body[4..8].try_into().unwrap()) as usize;
            let start = WireHeader::SIZE + offset;
            std::str::from_utf8(&frame[start..start + length])
                .expect("String payload UTF-8")
                .to_string()
        })
        .collect()
}

#[test]
fn graph_publish_record_lifecycle_and_errors() {
    let temp = tempdir().unwrap();
    let files_dir = temp.path();
    let node_lib_dir = fixture_dir();
    let library = node_lib_dir.join("libtest_node_macro_period_cdylib.so");
    assert!(
        library.is_file(),
        "build the fixture first: cargo build -p test_node_macro_period_cdylib"
    );
    let prefix = format!("embedded_{}", std::process::id());
    let topic = format!("/{prefix}/heartbeat");
    let first_path = files_dir.join("bags/first.mcap");
    let mut runtime = EmbeddedRuntime::new(files_dir.to_path_buf(), "android_test".into()).unwrap();

    assert!(matches!(
        runtime.stop(),
        Err(cerulion_android::EmbedError::BadState(_))
    ));
    assert!(runtime.publish(&topic, r#"{"data":"before"}"#).is_err());
    assert!(matches!(
        runtime.start(
            &json!({
                "publish": [{"topic": topic, "schema": "missing/Message"}]
            })
            .to_string()
        ),
        Err(cerulion_android::EmbedError::InvalidArgument(_))
    ));
    let first_config = config(&node_lib_dir, &prefix, "first.mcap", &topic);
    let mut invalid_record: serde_json::Value =
        serde_json::from_str(&first_config).expect("config JSON");
    invalid_record["record"]["topics"] = json!(["/missing"]);
    assert!(matches!(
        runtime.start(&invalid_record.to_string()),
        Err(cerulion_android::EmbedError::InvalidArgument(_))
    ));
    runtime.start(&first_config).unwrap();
    assert!(runtime.start(&first_config).is_err());
    assert!(runtime.publish("/unknown", r#"{"data":"x"}"#).is_err());
    assert!(matches!(
        runtime.publish(&topic, "[]"),
        Err(cerulion_android::EmbedError::Encode(_))
    ));
    let graph_topic = format!("/{prefix}/ticker/cmd");
    let manager = TransportManager::get().expect("transport manager");
    let subscriber = manager
        .create_subscriber(&graph_topic)
        .expect("subscribe to graph output");
    assert!(
        subscriber
            .wait_for_message(Duration::from_secs(2), |_| {})
            .expect("wait for graph output")
            > 0
    );
    drop(subscriber);
    for value in ["a", "b", "c"] {
        runtime
            .publish(&topic, &json!({"data": value}).to_string())
            .unwrap();
    }
    let first_summary = runtime.stop().unwrap().unwrap();
    assert!(first_summary.messages >= 3);
    assert!(first_summary.channels >= 2);
    assert!(
        first_summary
            .per_topic
            .get(&graph_topic)
            .copied()
            .unwrap_or(0)
            > 0
    );
    let status_handle = runtime.status_handle();
    let status = status_handle.lock().unwrap();
    assert!(
        status
            .recorder
            .as_ref()
            .and_then(|summary| summary.per_topic.get(&graph_topic))
            .copied()
            .unwrap_or(0)
            > 0
    );
    drop(status);
    assert!(runtime.publish(&topic, r#"{"data":"after"}"#).is_err());
    assert_eq!(string_payloads(&first_path, &topic), ["a", "b", "c"]);

    let second_path = files_dir.join("bags/second.mcap");
    let second_config = config(&node_lib_dir, &prefix, "second.mcap", &topic);
    runtime.start(&second_config).unwrap();
    for value in ["a", "b", "c"] {
        runtime
            .publish(&topic, &json!({"data": value}).to_string())
            .unwrap();
    }
    runtime.stop().unwrap();
    assert_eq!(string_payloads(&second_path, &topic), ["a", "b", "c"]);

    let different_listen = json!({"listen":["tcp/127.0.0.1:7683"]}).to_string();
    assert!(runtime.start(&different_listen).is_err());

    let missing_node_lib = json!({
        "graph_yaml": "nodes:\n  - id: ticker\n    type: test_node_macro_period_cdylib\n    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n"
    })
    .to_string();
    assert!(matches!(
        runtime.start(&missing_node_lib),
        Err(cerulion_android::EmbedError::Graph(_))
    ));
    let missing_library = json!({
        "graph_yaml": "nodes:\n  - id: ticker\n    type: missing_cdylib\n",
        "node_lib_dir": node_lib_dir
    })
    .to_string();
    assert!(matches!(
        runtime.start(&missing_library),
        Err(cerulion_android::EmbedError::Graph(_))
    ));
    assert!(matches!(
        runtime.start(r#"{"graph_yaml":"nodes: ["}"#),
        Err(cerulion_android::EmbedError::Graph(_))
    ));
}
