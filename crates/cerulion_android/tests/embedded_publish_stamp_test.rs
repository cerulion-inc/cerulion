// SPDX-License-Identifier: AGPL-3.0-only
//! App frames take their wire stamp from the runtime's clock. Each test runs
//! the runtime on a `VirtualClock`, records the app topic and reads the bag
//! back: the stamp bytes in the frame header and the record's log time are
//! checked against hand-written values.

use std::path::Path;
use std::sync::Arc;

use cerulion_android::EmbeddedRuntime;
use cerulion_bag::BagReader;
use cerulion_core::clock::VirtualClock;
use serde_json::json;
use tempfile::tempdir;

/// Byte offset of `timestamp_ns` in the wire header.
const STAMP_OFFSET: usize = 24;

/// One recorded frame on `topic`: its wire stamp bytes and its log time.
fn recorded_stamps(path: &Path, topic: &str) -> Vec<([u8; 8], u64)> {
    let reader = BagReader::open(path).expect("open finalized bag");
    reader
        .messages()
        .expect("read messages")
        .map(|message| message.expect("a readable message"))
        .filter(|message| message.topic == topic)
        .map(|message| {
            let stamp: [u8; 8] = message.data[STAMP_OFFSET..STAMP_OFFSET + 8]
                .try_into()
                .unwrap();
            (stamp, message.log_time)
        })
        .collect()
}

fn recording_config(file: &str, topic: &str) -> String {
    json!({
        "record": {"file": file},
        "publish": [{"topic": topic, "schema": "std_msgs/String"}]
    })
    .to_string()
}

#[test]
fn a_published_frame_carries_the_runtime_clock_stamp() {
    let temp = tempdir().unwrap();
    let files_dir = temp.path();
    let topic = format!("/stamp_clock_{}/heartbeat", std::process::id());
    let clock = Arc::new(VirtualClock::new());
    clock.advance(0x0102_0304_0506_0708);
    let mut runtime = EmbeddedRuntime::new(files_dir.to_path_buf(), "android_test".into()).unwrap();
    runtime.set_clock_for_test(clock.clone());

    runtime
        .start(&recording_config("clock.mcap", &topic))
        .unwrap();
    runtime.publish(&topic, r#"{"data":"first"}"#).unwrap();
    clock.advance(5_000_000);
    runtime.publish(&topic, r#"{"data":"second"}"#).unwrap();
    runtime.stop().unwrap();

    assert_eq!(
        recorded_stamps(&files_dir.join("bags/clock.mcap"), &topic),
        vec![
            (
                [0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01],
                72_623_859_790_382_856
            ),
            (
                [0x48, 0x52, 0x52, 0x05, 0x04, 0x03, 0x02, 0x01],
                72_623_859_795_382_856
            ),
        ]
    );
}
