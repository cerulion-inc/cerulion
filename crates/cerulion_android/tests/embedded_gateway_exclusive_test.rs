// SPDX-License-Identifier: AGPL-3.0-only
use std::path::PathBuf;

use cerulion_android::{EmbedError, EmbeddedRuntime, RuntimeState};
use cerulion_core::transport::TransportManager;
use serde_json::json;
use tempfile::tempdir;

const ROBOT: &str = "android_gateway_exclusive_test";

fn config(topic: &str) -> String {
    json!({
        "listen": ["tcp/127.0.0.1:0"],
        "publish": [{"topic": topic, "schema": "std_msgs/String"}]
    })
    .to_string()
}

fn exported(manager: &TransportManager) -> (Vec<String>, Vec<String>) {
    let network = manager.network().expect("network transport");
    let mut registered: Vec<String> = manager
        .bridge_manager()
        .registered_topics()
        .expect("bridge registry")
        .into_iter()
        .map(|(topic, _flag)| topic)
        .collect();
    registered.sort();
    (network.announced_topics(), registered)
}

/// Two runtimes with one process identity share one transport manager. A
/// second start with listen endpoints while the first runtime's gateway is
/// live must be refused before it touches that manager, so the running
/// runtime keeps its announcements and bridge flags. Hand oracle: the active
/// runtime exports `/<robot>/active` before and after the refused start, and
/// the second runtime starts once the first has stopped.
#[test]
fn an_overlapping_start_leaves_the_active_gateway_topics_exported() {
    let files = tempdir().expect("app files directory");
    let active_topic = format!("/{ROBOT}/active");
    let second_topic = format!("/{ROBOT}/second");
    let mut active =
        EmbeddedRuntime::new(PathBuf::from(files.path()), ROBOT.to_string()).expect("runtime");
    active.start(&config(&active_topic)).expect("start active");
    let manager = TransportManager::get().expect("transport manager");
    let before = exported(&manager);
    assert!(
        before.0.contains(&active_topic),
        "announced: {:?}",
        before.0
    );
    assert!(
        before.1.contains(&active_topic),
        "registered: {:?}",
        before.1
    );

    let mut second =
        EmbeddedRuntime::new(PathBuf::from(files.path()), ROBOT.to_string()).expect("runtime");
    let refused = second
        .start(&config(&second_topic))
        .expect_err("an overlapping gateway start must be refused");
    assert!(matches!(refused, EmbedError::BadState(_)), "{refused:?}");

    assert_eq!(exported(&manager), before);
    assert_eq!(
        active.status_handle().lock().expect("status").state,
        RuntimeState::Running
    );
    active
        .publish(&active_topic, r#"{"data":"still running"}"#)
        .expect("the active runtime still publishes");

    active.stop().expect("stop active");
    second
        .start(&config(&second_topic))
        .expect("the gateway is free once the active runtime stopped");
    second.stop().expect("stop second");
}
