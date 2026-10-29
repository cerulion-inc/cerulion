// SPDX-License-Identifier: AGPL-3.0-only
//! Requires `cargo build -p test_node_macro_period_cdylib -p test_node_hang_cdylib`
//! before running.

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

use cerulion_android::EmbeddedRuntime;
use cerulion_bag::BagReader;
use cerulion_core::{transport::TransportManager, wire::WireHeader, LanAccess};
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

fn assert_channels(path: &Path, prefix: &str, topic: &str, expect_graph_channel: bool) {
    let reader = BagReader::open(path).expect("open finalized bag");
    let channels = reader.channels().expect("read channels");
    if expect_graph_channel {
        // Compare the FULL prefixed graph topic: a contains() pass hides a
        // resolution that wrote the wrong namespace.
        let graph_topic = channels
            .iter()
            .find(|channel| channel.topic == format!("/{prefix}/ticker/cmd"))
            .expect("graph output channel");
        assert_eq!(graph_topic.schema_name, "geometry_msgs/Vector3");
    }
    let string_channel = channels
        .iter()
        .find(|channel| channel.topic == topic)
        .expect("publish channel");
    assert_eq!(string_channel.schema_name, "std_msgs/String");
}

fn string_payloads(path: &Path, topic: &str) -> Vec<String> {
    let reader = BagReader::open(path).expect("open finalized bag");
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
    assert_channels(&first_path, &prefix, &topic, true);
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

#[test]
fn burst_publishes_are_recorded_in_order() {
    let temp = tempdir().unwrap();
    let files_dir = temp.path();
    let prefix = format!("burst_{}", std::process::id());
    let topic = format!("/{prefix}/heartbeat");
    let bag_path = files_dir.join("bags/burst.mcap");
    let mut runtime = EmbeddedRuntime::new(files_dir.to_path_buf(), "android_test".into()).unwrap();

    runtime
        .start(
            &json!({
                "record": {"file": "burst.mcap"},
                "publish": [{"topic": topic, "schema": "std_msgs/String"}]
            })
            .to_string(),
        )
        .unwrap();
    let expected: Vec<String> = (0..256).map(|index| format!("frame-{index}")).collect();
    for value in &expected {
        runtime
            .publish(&topic, &json!({"data": value}).to_string())
            .unwrap();
    }
    runtime.stop().unwrap();

    assert_channels(&bag_path, &prefix, &topic, false);
    assert_eq!(string_payloads(&bag_path, &topic), expected);
}

#[test]
fn rejects_publish_topics_that_are_graph_outputs() {
    let temp = tempdir().unwrap();
    let node_lib_dir = fixture_dir();
    assert!(
        node_lib_dir
            .join("libtest_node_macro_period_cdylib.so")
            .is_file(),
        "build the fixture first: cargo build -p test_node_macro_period_cdylib"
    );
    let prefix = format!("collision_{}", std::process::id());
    let topic = format!("/{prefix}/ticker/cmd");
    let graph_yaml = format!(
        "prefix: {prefix}\nnodes:\n  - id: ticker\n    type: test_node_macro_period_cdylib\n    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n"
    );
    let mut runtime =
        EmbeddedRuntime::new(temp.path().to_path_buf(), "android_test".into()).unwrap();

    let error = runtime
        .start(
            &json!({
                "graph_yaml": graph_yaml,
                "node_lib_dir": node_lib_dir,
                "publish": [{"topic": topic, "schema": "geometry_msgs/Vector3"}]
            })
            .to_string(),
        )
        .expect_err("a graph output cannot also be an app publisher");
    let message =
        format!("publish topic '{topic}' is a graph output; graph topics are single-writer");
    assert!(matches!(
        error,
        cerulion_android::EmbedError::InvalidArgument(ref actual) if actual == &message
    ));
    let status = runtime.status_handle();
    let status = status.lock().unwrap();
    assert_eq!(status.state, cerulion_android::RuntimeState::Failed);
    assert_eq!(
        status.last_error.as_ref().map(|error| error.code),
        Some(cerulion_android::codes::INVALID_ARGUMENT)
    );
    assert_eq!(
        status
            .last_error
            .as_ref()
            .map(|error| error.message.as_str()),
        Some(message.as_str())
    );
}

/// A graph output is a topic the start serves, so `egress` may list it; a
/// start without `listen` builds no gateway and the list has no effect.
#[test]
fn an_egress_list_may_name_a_graph_output() {
    let temp = tempdir().unwrap();
    let node_lib_dir = fixture_dir();
    assert!(
        node_lib_dir
            .join("libtest_node_macro_period_cdylib.so")
            .is_file(),
        "build the fixture first: cargo build -p test_node_macro_period_cdylib"
    );
    let prefix = format!("egress_{}", std::process::id());
    let topic = format!("/{prefix}/ticker/cmd");
    let graph_yaml = format!(
        "prefix: {prefix}\nnodes:\n  - id: ticker\n    type: test_node_macro_period_cdylib\n    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n"
    );
    let mut runtime =
        EmbeddedRuntime::new(temp.path().to_path_buf(), "android_test".into()).unwrap();
    runtime
        .start(
            &json!({
                "graph_yaml": graph_yaml,
                "node_lib_dir": node_lib_dir,
                "egress": [topic]
            })
            .to_string(),
        )
        .expect("an egress list may name a graph output");
    {
        let status = runtime.status_handle();
        let status = status.lock().unwrap();
        assert_eq!(status.state, cerulion_android::RuntimeState::Running);
        assert!(status.listen.is_empty());
        assert_eq!(status.lan_access, None, "no listen builds no gateway");
    }
    runtime.stop().unwrap();
}

#[test]
fn dropping_a_running_runtime_finalizes_its_recording() {
    let temp = tempdir().unwrap();
    let files_dir = temp.path();
    let node_lib_dir = fixture_dir();
    assert!(
        node_lib_dir
            .join("libtest_node_macro_period_cdylib.so")
            .is_file(),
        "build the fixture first: cargo build -p test_node_macro_period_cdylib"
    );
    let prefix = format!("drop_{}", std::process::id());
    let topic = format!("/{prefix}/heartbeat");
    let bag_path = files_dir.join("bags/dropped.mcap");
    let mut runtime = EmbeddedRuntime::new(files_dir.to_path_buf(), "android_test".into()).unwrap();

    runtime
        .start(&config(&node_lib_dir, &prefix, "dropped.mcap", &topic))
        .unwrap();
    for value in ["a", "b", "c"] {
        runtime
            .publish(&topic, &json!({"data": value}).to_string())
            .unwrap();
    }
    let status_handle = runtime.status_handle();
    drop(runtime);

    let status = status_handle.lock().unwrap();
    assert!(matches!(
        status.state,
        cerulion_android::RuntimeState::Stopped
    ));
    let summary = status.recorder.as_ref().expect("recorder summary on drop");
    assert_eq!(summary.per_topic.get(&topic).copied(), Some(3));
    drop(status);
    assert_channels(&bag_path, &prefix, &topic, true);
    assert_eq!(string_payloads(&bag_path, &topic), ["a", "b", "c"]);
}

#[test]
fn dropping_a_never_started_runtime_is_a_no_op() {
    let temp = tempdir().unwrap();
    let runtime = EmbeddedRuntime::new(temp.path().to_path_buf(), "android_test".into()).unwrap();
    drop(runtime);
}

#[test]
fn an_existing_recording_file_is_refused_untouched() {
    let temp = tempdir().unwrap();
    let files_dir = temp.path();
    let prefix = format!("exists_{}", std::process::id());
    let topic = format!("/{prefix}/heartbeat");
    let bags_dir = files_dir.join("bags");
    std::fs::create_dir_all(&bags_dir).unwrap();
    let bag_path = bags_dir.join("taken.mcap");
    let original = b"sentinel bytes that must survive";
    std::fs::write(&bag_path, original).unwrap();
    let mut runtime = EmbeddedRuntime::new(files_dir.to_path_buf(), "android_test".into()).unwrap();

    assert!(matches!(
        runtime.start(
            &json!({
                "record": {"file": "taken.mcap"},
                "publish": [{"topic": topic, "schema": "std_msgs/String"}]
            })
            .to_string(),
        ),
        Err(cerulion_android::EmbedError::Recorder(_))
    ));
    assert_eq!(std::fs::read(&bag_path).unwrap(), original);
    assert_eq!(
        runtime
            .status_handle()
            .lock()
            .unwrap()
            .last_error
            .as_ref()
            .map(|error| error.code),
        Some(cerulion_android::codes::RECORDER)
    );
}

#[test]
fn a_refused_recording_config_leaves_its_file_name_free() {
    let temp = tempdir().unwrap();
    let files_dir = temp.path();
    let prefix = format!("refused_{}", std::process::id());
    let topic = format!("/{prefix}/heartbeat");
    let bag_path = files_dir.join("bags/retry.mcap");
    let mut runtime = EmbeddedRuntime::new(files_dir.to_path_buf(), "android_test".into()).unwrap();

    let error = runtime
        .start(
            &json!({
                "record": {"file": "retry.mcap", "topics": []},
                "publish": [{"topic": topic, "schema": "std_msgs/String"}]
            })
            .to_string(),
        )
        .expect_err("empty recording selection is refused");
    assert!(matches!(
        error,
        cerulion_android::EmbedError::InvalidArgument(_)
    ));
    assert_eq!(
        error.to_string(),
        "recording requires at least one graph output or publish topic"
    );
    assert!(!bag_path.exists());

    runtime
        .start(
            &json!({
                "record": {"file": "retry.mcap"},
                "publish": [{"topic": topic, "schema": "std_msgs/String"}]
            })
            .to_string(),
        )
        .unwrap();
    runtime.stop().unwrap();
    assert!(bag_path.is_file());
}

#[test]
fn a_restart_must_name_a_new_recording_file() {
    let temp = tempdir().unwrap();
    let files_dir = temp.path();
    let prefix = format!("rename_{}", std::process::id());
    let topic = format!("/{prefix}/heartbeat");
    let mut runtime = EmbeddedRuntime::new(files_dir.to_path_buf(), "android_test".into()).unwrap();

    let first = json!({
        "record": {"file": "first.mcap"},
        "publish": [{"topic": topic, "schema": "std_msgs/String"}]
    })
    .to_string();
    runtime.start(&first).unwrap();
    runtime.stop().unwrap();
    assert!(matches!(
        runtime.start(&first),
        Err(cerulion_android::EmbedError::Recorder(_))
    ));

    runtime
        .start(
            &json!({
                "record": {"file": "second.mcap"},
                "publish": [{"topic": topic, "schema": "std_msgs/String"}]
            })
            .to_string(),
        )
        .unwrap();
    runtime.stop().unwrap();
}

const BLOCKED_TICK_CHILD_ENV: &str = "CERULION_ANDROID_BLOCKED_TICK_CHILD";

#[test]
fn stopping_runtime_closes_lan_before_a_blocked_tick_join() {
    if std::env::var_os(BLOCKED_TICK_CHILD_ENV).is_some() {
        run_blocked_tick_shutdown_child();
        return;
    }

    let mut child = Command::new(std::env::current_exe().expect("current test executable"))
        .args([
            "--exact",
            "stopping_runtime_closes_lan_before_a_blocked_tick_join",
            "--test-threads=1",
            "--nocapture",
        ])
        .env(BLOCKED_TICK_CHILD_ENV, "1")
        .env("CER_HANG_MODE", "1")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn blocked tick child");
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        if let Some(status) = child.try_wait().expect("poll blocked tick child") {
            let output = child.wait_with_output().expect("collect child output");
            assert!(
                status.success(),
                "blocked tick child failed with {status}\nstdout:\n{}\nstderr:\n{}",
                String::from_utf8_lossy(&output.stdout),
                String::from_utf8_lossy(&output.stderr)
            );
            return;
        }
        if Instant::now() >= deadline {
            let _ = child.kill();
            let output = child
                .wait_with_output()
                .expect("collect timed out child output");
            panic!(
                "blocked tick child did not exit\nstdout:\n{}\nstderr:\n{}",
                String::from_utf8_lossy(&output.stdout),
                String::from_utf8_lossy(&output.stderr)
            );
        }
        thread::sleep(Duration::from_millis(25));
    }
}

fn run_blocked_tick_shutdown_child() {
    let node_lib_dir = fixture_dir();
    assert!(
        node_lib_dir.join("libtest_node_hang_cdylib.so").is_file(),
        "build the fixture first: cargo build -p test_node_hang_cdylib"
    );
    let temp = tempdir().expect("temporary runtime directory");
    let port = std::net::TcpListener::bind("127.0.0.1:0")
        .expect("bind ephemeral port")
        .local_addr()
        .expect("ephemeral port address")
        .port();
    let graph_yaml = "nodes:\n  - id: hung\n    type: test_node_hang_cdylib\n    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n";
    let config = json!({
        "graph_yaml": graph_yaml,
        "node_lib_dir": node_lib_dir,
        "listen": [format!("tcp/127.0.0.1:{port}")],
        "lan_access": "open",
        "mdns": false
    })
    .to_string();
    let mut runtime =
        EmbeddedRuntime::new(temp.path().to_path_buf(), "android_test".into()).unwrap();
    runtime
        .start(&config)
        .expect("start open runtime with hanging node");
    let manager = TransportManager::get().expect("initialized app transport manager");
    let network = manager.network().expect("app gateway network");
    assert_eq!(network.lan_access(), LanAccess::Open);
    thread::sleep(Duration::from_secs(1));

    let (done_tx, done_rx) = mpsc::channel();
    let _stop_thread = thread::spawn(move || {
        runtime
            .stop()
            .expect("stop runtime with blocked graph tick");
        let _ = done_tx.send(());
    });
    let deadline = Instant::now() + Duration::from_secs(5);
    while network.lan_access() != LanAccess::Paired && Instant::now() < deadline {
        thread::sleep(Duration::from_millis(10));
    }
    assert_eq!(
        network.lan_access(),
        LanAccess::Paired,
        "stop must revoke LAN access before joining the blocked graph thread"
    );
    assert!(
        matches!(
            done_rx.recv_timeout(Duration::from_millis(100)),
            Err(mpsc::RecvTimeoutError::Timeout)
        ),
        "stop must remain blocked while the node is parked in tick"
    );
}

#[test]
fn a_custom_schema_under_files_dir_schemas_publishes() {
    // `files_dir/schemas` is the workspace schema store: a publisher declared
    // with a schema that only exists there must resolve and publish. This
    // pinned `files_dir/schemas/schemas` lookup (the workspace loader takes
    // the root and appends `schemas/` itself) once reported
    // `unknown publish schema` for exactly this case.
    let temp = tempdir().unwrap();
    let files_dir = temp.path();
    std::fs::create_dir_all(files_dir.join("schemas")).unwrap();
    std::fs::write(
        files_dir.join("schemas/tick.yaml"),
        "schemas:\n  custom/Tick:\n    fields:\n      int64 value: {}\n",
    )
    .unwrap();
    let topic = format!("/custom_schema/{}", std::process::id());
    let mut runtime = EmbeddedRuntime::new(files_dir.to_path_buf(), "android_test".into()).unwrap();
    runtime
        .start(
            &json!({
                "publish": [{"topic": topic, "schema": "custom/Tick"}]
            })
            .to_string(),
        )
        .unwrap();
    runtime.publish(&topic, r#"{"value":7}"#).unwrap();
    runtime.stop().unwrap();
}
