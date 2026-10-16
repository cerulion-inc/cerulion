// SPDX-License-Identifier: AGPL-3.0-only
//! Requires `cargo build -p test_node_macro_period_cdylib` before running.

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use cerulion_android::{EmbedError, EmbeddedRuntime, RuntimeState};
use cerulion_bag::BagReader;
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

#[test]
fn graph_shutdown_finishes_and_can_be_stopped_and_restarted() {
    std::env::set_var("CER_PERIOD_REQUEST_SHUTDOWN", "1");
    let temp = tempdir().expect("app files directory");
    let node_lib_dir = fixture_dir();
    let library = node_lib_dir.join(cerulion_cli_engine::graph_cmd::cdylib_name(
        "test_node_macro_period_cdylib",
    ));
    assert!(
        library.is_file(),
        "build the fixture first: cargo build -p test_node_macro_period_cdylib"
    );
    let prefix = format!("embedded_finish_{}", std::process::id());
    let topic = format!("/{prefix}/heartbeat");
    let first_path = temp.path().join("bags/finished.mcap");
    let mut runtime = EmbeddedRuntime::new(temp.path().to_path_buf(), "android_finish_test".into())
        .expect("runtime");

    runtime
        .start(&config(&node_lib_dir, &prefix, "finished.mcap", &topic))
        .expect("start graph");
    let status_handle = runtime.status_handle();
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let state = status_handle.lock().expect("status lock").state;
        if state == RuntimeState::Finished {
            break;
        }
        assert_ne!(state, RuntimeState::Failed, "graph failed before finishing");
        assert!(
            Instant::now() < deadline,
            "graph did not finish within 10 seconds"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(matches!(
        runtime.publish(&topic, r#"{"data":"too late"}"#),
        Err(EmbedError::BadState(_))
    ));

    let summary = runtime
        .stop()
        .expect("stop finished graph")
        .expect("recording summary");
    assert_eq!(
        status_handle.lock().expect("status lock").state,
        RuntimeState::Stopped
    );
    assert!(summary.channels > 0);
    assert!(status_handle
        .lock()
        .expect("status lock")
        .recorder
        .is_some());
    let reader = BagReader::open(&first_path).expect("open finalized recording");
    let graph_channel = reader
        .channels()
        .expect("read recording channels")
        .into_iter()
        .find(|channel| channel.topic == format!("/{prefix}/ticker/cmd"))
        .expect("recorded graph channel");
    assert_eq!(graph_channel.schema_name, "geometry_msgs/Vector3");

    std::env::remove_var("CER_PERIOD_REQUEST_SHUTDOWN");
    runtime
        .start(&config(&node_lib_dir, &prefix, "restarted.mcap", &topic))
        .expect("restart after finished graph");
    // The restarted graph must actually run, not merely accept the config:
    // wait for a post-restart output before stopping. `start` sets Running
    // once the worker entered its closure, so the publish proves the
    // restarted runtime's send path really works.
    runtime
        .publish(&topic, r#"{"data":"after-restart"}"#)
        .expect("publish on the restarted runtime");
    {
        let status = runtime.status_handle();
        let status = status.lock().unwrap();
        assert!(
            matches!(status.state, cerulion_android::RuntimeState::Running),
            "the restarted graph is not Running: {status:?}"
        );
        assert_eq!(
            status.published.get(&topic),
            Some(&1),
            "the post-restart publish was not recorded"
        );
    }
    runtime.stop().expect("stop restarted runtime");
    // The restarted recording carries the post-restart output: the graph
    // produced work after restart, not just a Running flag.
    let reader = BagReader::open(temp.path().join("bags/restarted.mcap"))
        .expect("open restarted recording");
    assert!(
        reader
            .channels()
            .expect("read restarted channels")
            .into_iter()
            .any(|channel| channel.topic == format!("/{prefix}/ticker/cmd")),
        "the restarted graph produced no recorded output"
    );
}
