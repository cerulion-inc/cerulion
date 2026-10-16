// SPDX-License-Identifier: AGPL-3.0-only
use std::path::PathBuf;
use std::time::Duration;

use cerulion_android::EmbeddedRuntime;
use cerulion_core::transport::TransportManager;
use serde_json::json;
use tempfile::tempdir;

const ROBOT: &str = "android_schema_restart_test";

fn query_catalog() -> cerulion_core::transport::cerulion_q::CatalogReply {
    let manager = TransportManager::get().expect("transport manager");
    let network = manager.network().expect("network transport");
    cerulion_core::transport::discovery::query_robot_catalog(
        network.session().expect("network session"),
        ROBOT,
        Duration::from_secs(1),
    )
    .expect("runtime must answer its catalog query")
}

#[test]
fn restarting_runtime_replaces_the_latched_catalog_schema() {
    let files = tempdir().expect("app files directory");
    let topic = format!("/{ROBOT}/restart_topic");
    let listen = "tcp/127.0.0.1:0";
    let mut runtime =
        EmbeddedRuntime::new(PathBuf::from(files.path()), ROBOT.to_string()).expect("runtime");
    let config = |schema: &str| {
        json!({
            "listen": [listen],
            "publish": [{"topic": topic.clone(), "schema": schema}]
        })
        .to_string()
    };
    let catalog_schema = |catalog: &cerulion_core::transport::cerulion_q::CatalogReply| {
        catalog
            .entries
            .iter()
            .find(|entry| entry.topic == topic)
            .unwrap_or_else(|| panic!("catalog must list {topic}"))
            .schema_name
            .clone()
    };

    runtime
        .start(&config("std_msgs/String"))
        .expect("start first runtime");
    assert_eq!(
        catalog_schema(&query_catalog()).as_deref(),
        Some("std_msgs/String")
    );
    runtime.stop().expect("stop first runtime");

    runtime
        .start(&config("geometry_msgs/Vector3"))
        .expect("restart runtime with a changed topic schema");
    let restarted_schema = catalog_schema(&query_catalog());
    assert_eq!(restarted_schema.as_deref(), Some("geometry_msgs/Vector3"));
    assert_ne!(restarted_schema.as_deref(), Some("std_msgs/String"));
    runtime.stop().expect("stop restarted runtime");
}
