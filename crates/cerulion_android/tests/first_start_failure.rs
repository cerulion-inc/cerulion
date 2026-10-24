// SPDX-License-Identifier: AGPL-3.0-only
//! A failed FIRST gateway start, in its OWN binary: the query surface is
//! latched once per process, so only the first start of a process creates
//! it. The surface outlives the failed start; it must serve no schema.
//!
//! Requires `cargo build -p test_node_macro_external_cdylib` before running.

use std::path::{Path, PathBuf};

use cerulion_android::runtime::gateway_owned_for_test;
use cerulion_android::{EmbedError, EmbeddedRuntime};
use cerulion_core::transport::TransportManager;
use cerulion_core::LanAccess;
use serde_json::json;
use tempfile::tempdir;

fn fixture_dir() -> PathBuf {
    let target_dir = std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target"));
    target_dir.join("debug")
}

#[test]
fn a_failed_first_start_serves_no_schema() {
    let node_lib_dir = fixture_dir();
    let library = node_lib_dir.join(cerulion_cli_engine::graph_cmd::cdylib_name(
        "test_node_macro_external_cdylib",
    ));
    assert!(
        library.is_file(),
        "build the fixture first: cargo build -p test_node_macro_external_cdylib"
    );
    let robot = "first_start_failure";
    let prefix = format!("first_start_{}", std::process::id());
    // The graph builds and the gateway latches the query surface with this
    // run's serving; then the graph worker refuses the inert node at launch.
    let graph_yaml = format!(
        "prefix: {prefix}\nnodes:\n  - id: inert\n    type: test_node_macro_external_cdylib\n    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n"
    );
    let temp = tempdir().expect("app files directory");
    let mut runtime =
        EmbeddedRuntime::new(temp.path().to_path_buf(), robot.into()).expect("runtime");
    assert!(
        TransportManager::get().is_err(),
        "this binary's first start must create the process manager"
    );

    let error = runtime
        .start(
            &json!({
                "graph_yaml": graph_yaml,
                "node_lib_dir": node_lib_dir,
                "listen": ["tcp/127.0.0.1:0"],
                "lan_access": "open",
                "mdns": false,
                "publish": [{"topic": format!("/{prefix}/app"), "schema": "std_msgs/String"}]
            })
            .to_string(),
        )
        .expect_err("a graph refused at launch fails the start");

    let EmbedError::Graph(message) = &error else {
        panic!("expected the graph launch refusal, got: {error}");
    };
    assert!(message.contains("provably inert at launch"), "{message}");
    let manager = TransportManager::get().expect("the failed start created the manager");
    let network = manager.network().expect("network");
    let live = network
        .query_surface_serving_handles()
        .expect("the failed start's gateway latched the query surface");
    assert!(
        live.is_serving_nothing_for_test(),
        "the latched surface must not keep serving the failed run's schema: {:?}",
        live.served_topics_for_test()
    );
    assert_eq!(network.lan_access(), LanAccess::Paired);
    assert!(network.announced_topics().is_empty());
    assert_eq!(
        manager
            .bridge_manager()
            .registered_topic_count()
            .expect("count"),
        0
    );
    assert!(!gateway_owned_for_test());
}
