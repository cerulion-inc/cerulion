// SPDX-License-Identifier: AGPL-3.0-only
//! A graph refused at launch, in its OWN binary: the embedded runtime latches
//! process-wide state (the transport identity, the iceoryx2 root under the
//! first start's files dir) on the first start, so sharing a binary with
//! another start-driving test makes both order-dependent.
//!
//! Requires `cargo build -p test_node_macro_external_cdylib` before running.

use std::path::{Path, PathBuf};

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

#[test]
fn a_graph_launch_refusal_fails_the_start_instead_of_reporting_running() {
    // A host-driven external node is provably inert under the live loop, so
    // the graph worker's launch checks refuse it. That refusal happens on the
    // worker thread after the graph was built and AFTER the recorder started;
    // start must wait for it and return the failure, never report Running
    // for a graph that refused to launch. The bound is deterministic: start
    // returns only once the worker passed its launch checks or exited.
    let temp = tempdir().expect("app files directory");
    let node_lib_dir = fixture_dir();
    let library = node_lib_dir.join(cerulion_cli_engine::graph_cmd::cdylib_name(
        "test_node_macro_external_cdylib",
    ));
    assert!(
        library.is_file(),
        "build the fixture first: cargo build -p test_node_macro_external_cdylib"
    );
    let prefix = format!("embedded_refusal_{}", std::process::id());
    let topic = format!("/{prefix}/heartbeat");
    let graph_yaml = format!(
        "prefix: {prefix}\nnodes:\n  - id: inert\n    type: test_node_macro_external_cdylib\n    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n"
    );
    let mut runtime =
        EmbeddedRuntime::new(temp.path().to_path_buf(), "android_refusal_test".into())
            .expect("runtime");
    for attempt in ["first", "second"] {
        let error = runtime
            .start(
                &json!({
                    "graph_yaml": graph_yaml,
                    "node_lib_dir": node_lib_dir,
                    "record": {"file": format!("{attempt}.mcap")},
                    "publish": [{"topic": topic, "schema": "std_msgs/String"}]
                })
                .to_string(),
            )
            .expect_err("a graph refused at launch cannot report start success");
        // Exactly the launch refusal for the inert node: an earlier graph
        // parse, load or build failure would also be `Graph`, and must not
        // satisfy this test.
        let EmbedError::Graph(message) = &error else {
            panic!("expected the graph launch refusal, got: {error}");
        };
        assert!(
            message.contains("provably inert at launch") && message.contains("inert ("),
            "expected the inert-at-launch refusal for node 'inert', got: {message}"
        );
        let status = runtime.status_handle();
        let status = status.lock().expect("status lock");
        assert_eq!(status.state, RuntimeState::Failed, "{status:?}");
        drop(status);
        // The recorder was live when the graph refused: the failed start's
        // teardown stopped it and finalized its bag.
        let bag = temp.path().join(format!("bags/{attempt}.mcap"));
        let completeness = BagReader::open(&bag)
            .expect("open the failed start's recording")
            .completeness()
            .expect("scan the failed start's recording");
        assert!(
            completeness.is_finalized(),
            "the failed start's recording must carry its footer, got: {completeness:?}"
        );
        // The failed start tore its components down: nothing is left to
        // publish through, and the next attempt starts from a clean slate.
        assert!(matches!(
            runtime.publish(&topic, r#"{"data":"x"}"#),
            Err(EmbedError::BadState(_))
        ));
    }
}
