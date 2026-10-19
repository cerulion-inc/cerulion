// SPDX-License-Identifier: AGPL-3.0-only
//! A restart-after-component-failure round-trip in its OWN binary: the
//! embedded runtime's listen endpoints are latched process-lifetime by the
//! first start, and other tests in the shared binary start with `listen: []` -
//! whichever runs first pins the process settings, so co-locating this test
//! makes it order-dependent.

use cerulion_android::EmbeddedRuntime;
use cerulion_bag::BagReader;
use cerulion_core::transport::TransportManager;
use cerulion_core::LanAccess;
use serde_json::json;
use std::path::{Path, PathBuf};
use tempfile::tempdir;

fn fixture_dir() -> PathBuf {
    let target_dir = std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target"));
    target_dir.join("debug")
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
            let body = &frame[cerulion_core::wire::WireHeader::SIZE..];
            let offset = u32::from_le_bytes(body[..4].try_into().unwrap()) as usize;
            let length = u32::from_le_bytes(body[4..8].try_into().unwrap()) as usize;
            let start = cerulion_core::wire::WireHeader::SIZE + offset;
            std::str::from_utf8(&frame[start..start + length])
                .expect("String payload UTF-8")
                .to_string()
        })
        .collect()
}

#[test]
fn restart_after_component_failure_finalizes_both_bags() {
    let temp = tempdir().unwrap();
    let files_dir = temp.path();
    let prefix = format!("restart_{}", std::process::id());
    let topic = format!("/{prefix}/heartbeat");
    let mut runtime = EmbeddedRuntime::new(files_dir.to_path_buf(), "android_test".into()).unwrap();

    // A REAL component failure, not an assigned state: hold the port the
    // gateway is about to listen on so its session bind fails for real.
    // (If the start path ever swallowed the error instead of recording it,
    // the status asserts below fail.)
    let occupied = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let gateway_port = occupied.local_addr().unwrap().port();

    // The node init failure is synchronous: it lands during graph
    // construction, before the recorder spawns, so no `first.mcap`
    // precondition is possible - what the test proves instead is that the
    // failure is recorded as Failed and every partial component is torn
    // down before the retry below.
    let node_lib_dir = fixture_dir();
    let failing_lib = node_lib_dir.join(cerulion_cli_engine::graph_cmd::cdylib_name(
        "test_node_failing_cdylib",
    ));
    assert!(
        failing_lib.is_file(),
        "build the fixture first: cargo build -p test_node_failing_cdylib"
    );
    std::env::set_var("CER_FAIL_MODE", "init_fail");
    let graph_yaml = format!(
        "prefix: {prefix}\nnodes:\n  - id: failing\n    type: test_node_failing_cdylib\n    outputs:\n      - name: out\n        schema: geometry_msgs/Vector3\n"
    );
    let error = runtime
        .start(
            &json!({
                "record": {"file": "first.mcap"},
                "listen": [format!("tcp/127.0.0.1:{gateway_port}")],
                "lan_access": "open",
                "node_lib_dir": node_lib_dir,
                "graph_yaml": graph_yaml,
                "publish": [{"topic": topic, "schema": "std_msgs/String"}]
            })
            .to_string(),
        )
        .expect_err("a failed component cannot report start success");
    std::env::remove_var("CER_FAIL_MODE");
    // The occupied port fails the gateway bind (Network) and the node's
    // init_fail fails the graph worker (Graph); whichever surfaces first
    // ends the start, so both variants are a real component failure.
    assert!(
        matches!(
            error,
            cerulion_android::EmbedError::Network(_) | cerulion_android::EmbedError::Graph(_)
        ),
        "{error}"
    );
    {
        let status = runtime.status_handle();
        let status = status.lock().unwrap();
        // The failed start must not leave the runtime advertising a healthy
        // or starting state: a retry on top of it is what item 4's teardown
        // ordering protects.
        assert!(
            matches!(status.state, cerulion_android::RuntimeState::Failed),
            "a refused start must record Failed, got: {status:?}"
        );
    }
    drop(occupied);

    // A retry after the failure starts cleanly: the failed start's surviving
    // components were torn down before the new config was accepted. The
    // process-lifetime listen latch means the retry MUST reuse the same
    // endpoint - which is now free.
    runtime
        .start(
            &json!({
                "record": {"file": "second.mcap"},
                "listen": [format!("tcp/127.0.0.1:{gateway_port}")],
                "publish": [{"topic": topic, "schema": "std_msgs/String"}]
            })
            .to_string(),
        )
        .unwrap();
    {
        let status = runtime.status_handle();
        let status = status.lock().unwrap();
        assert!(
            matches!(status.state, cerulion_android::RuntimeState::Running),
            "the retry must reach Running, got: {status:?}"
        );
    }
    runtime.publish(&topic, r#"{"data":"after"}"#).unwrap();
    runtime.stop().unwrap();

    let second_path = files_dir.join("bags/second.mcap");
    assert_eq!(string_payloads(&second_path, &topic), ["after"]);

    // LAN posture rollback after a failure that lands AFTER the gateway
    // spawned (so teardown itself closes LAN access first). Each prior
    // posture differs from - or equals - the applied `open`, so a skipped
    // rollback and a rollback that compares against the applied value both
    // fail here.
    let manager = TransportManager::get().expect("the retry created the process manager");
    let network = manager.network().expect("gateway network");
    for prior in [LanAccess::Paired, LanAccess::Open] {
        network.set_lan_access(prior);
        // A failure BEFORE the gateway is built (the node's init fails during
        // graph construction): teardown does not close LAN access here, so
        // only the rollback can undo the applied `open`.
        std::env::set_var("CER_FAIL_MODE", "init_fail");
        let error = runtime
            .start(
                &json!({
                    "listen": [format!("tcp/127.0.0.1:{gateway_port}")],
                    "lan_access": "open",
                    "node_lib_dir": node_lib_dir,
                    "graph_yaml": graph_yaml,
                })
                .to_string(),
            )
            .expect_err("a node init failure fails the start");
        std::env::remove_var("CER_FAIL_MODE");
        assert!(
            matches!(error, cerulion_android::EmbedError::Graph(_)),
            "{error}"
        );
        assert_eq!(
            network.lan_access(),
            prior,
            "a start that failed before its gateway must restore the prior LAN posture ({prior:?})"
        );
        // A `bags` FILE makes the recorder's directory creation fail after
        // the gateway worker is already running.
        let blocked = tempdir().unwrap();
        std::fs::write(blocked.path().join("bags"), b"not a directory").unwrap();
        let mut failing =
            EmbeddedRuntime::new(blocked.path().to_path_buf(), "android_test".into()).unwrap();
        let error = failing
            .start(
                &json!({
                    "record": {"file": "blocked.mcap"},
                    "listen": [format!("tcp/127.0.0.1:{gateway_port}")],
                    "lan_access": "open",
                    "publish": [{"topic": topic, "schema": "std_msgs/String"}]
                })
                .to_string(),
            )
            .expect_err("an uncreatable bags directory fails the start");
        assert!(
            matches!(error, cerulion_android::EmbedError::Recorder(_)),
            "{error}"
        );
        assert_eq!(
            network.lan_access(),
            prior,
            "a failed start must restore the prior LAN posture ({prior:?})"
        );
    }
}
