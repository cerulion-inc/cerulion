// SPDX-License-Identifier: AGPL-3.0-only
//! A restart-after-component-failure round-trip in its OWN binary: the
//! embedded runtime's listen endpoints are latched process-lifetime by the
//! first start, and other tests in the shared binary start with `listen: []` -
//! whichever runs first pins the process settings, so co-locating this test
//! makes it order-dependent.

use cerulion_android::EmbeddedRuntime;
use cerulion_bag::BagReader;
use serde_json::json;
use std::path::Path;
use tempfile::tempdir;

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

    let error = runtime
        .start(
            &json!({
                "record": {"file": "first.mcap"},
                "listen": [format!("tcp/127.0.0.1:{gateway_port}")],
                "publish": [{"topic": topic, "schema": "std_msgs/String"}]
            })
            .to_string(),
        )
        .expect_err("a listen on an occupied port cannot open a gateway session");
    assert!(
        matches!(error, cerulion_android::EmbedError::Network(_)),
        "{error}"
    );
    {
        let status = runtime.status_handle();
        let status = status.lock().unwrap();
        // The failed start must not leave the runtime advertising a healthy
        // or starting state: a retry on top of it is what item 4's teardown
        // ordering protects.
        assert!(
            !matches!(
                status.state,
                cerulion_android::RuntimeState::Running | cerulion_android::RuntimeState::Starting
            ),
            "a refused start left a live state behind: {status:?}"
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
    runtime.publish(&topic, r#"{"data":"after"}"#).unwrap();
    runtime.stop().unwrap();

    let second_path = files_dir.join("bags/second.mcap");
    assert_eq!(string_payloads(&second_path, &topic), ["after"]);
}
