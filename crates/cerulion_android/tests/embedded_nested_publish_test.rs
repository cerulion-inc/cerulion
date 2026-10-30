// SPDX-License-Identifier: AGPL-3.0-only
//! An app publishes nested messages from JSON. The recorder subscribes to
//! every app topic, so the frames it records are the frames a subscriber
//! reads; each is checked byte for byte against a frame written out by hand
//! from the schema's wire layout.
//!
//! The runtime takes the process transport manager: run it with
//! `--test-threads=1`.

use std::path::Path;

use cerulion_android::{codes, EmbedError, EmbeddedRuntime};
use cerulion_bag::BagReader;
use cerulion_cli_engine::schema_cmd::parse_builtin_schemas;
use cerulion_core::dynamic::SchemaSet;
use serde_json::json;
use tempfile::tempdir;

/// The frames recorded on `topic`, in order.
fn recorded_frames(path: &Path, topic: &str) -> Vec<Vec<u8>> {
    let reader = BagReader::open(path).expect("open finalized bag");
    reader
        .messages()
        .expect("read messages")
        .map(|message| message.expect("a readable message"))
        .filter(|message| message.topic == topic)
        .map(|message| message.data)
        .collect()
}

fn schema_hash(name: &str) -> u64 {
    let (schemas, _) = SchemaSet::from_schemas(parse_builtin_schemas()).expect("built-in schemas");
    schemas.schema_hash(name).expect("a built-in schema")
}

/// The wire header of a frame `total` bytes long with a fixed section of
/// `fixed` bytes and `count` variable fields, numbered `sequence` and
/// stamped `stamp`.
fn header(hash: u64, total: u32, fixed: u32, count: u32, sequence: u32, stamp: u64) -> Vec<u8> {
    let mut bytes = hash.to_le_bytes().to_vec();
    for value in [total, 40 + fixed, count, sequence] {
        bytes.extend_from_slice(&value.to_le_bytes());
    }
    bytes.extend_from_slice(&stamp.to_le_bytes());
    bytes.extend_from_slice(&[0; 8]);
    bytes
}

const F64_ZERO: [u8; 8] = [0; 8];
const F64_ONE: [u8; 8] = [0, 0, 0, 0, 0, 0, 0xF0, 0x3F];
const F64_ONE_AND_HALF: [u8; 8] = [0, 0, 0, 0, 0, 0, 0xF8, 0x3F];
const F64_MINUS_TWO_AND_HALF: [u8; 8] = [0, 0, 0, 0, 0, 0, 0x04, 0xC0];
const F64_HALF: [u8; 8] = [0, 0, 0, 0, 0, 0, 0xE0, 0x3F];

#[test]
fn nested_messages_published_from_json_reach_a_subscriber_byte_for_byte() {
    let temp = tempdir().unwrap();
    let files_dir = temp.path();
    let pid = std::process::id();
    let pose_topic = format!("/nested_{pid}/pose");
    let twist_topic = format!("/nested_{pid}/twist");
    let mut runtime = EmbeddedRuntime::new(files_dir.to_path_buf(), "android_test".into()).unwrap();
    runtime
        .start(
            &json!({
                "record": {"file": "nested.mcap"},
                "publish": [
                    {"topic": pose_topic, "schema": "geometry_msgs/PoseStamped"},
                    {"topic": twist_topic, "schema": "geometry_msgs/Twist"}
                ]
            })
            .to_string(),
        )
        .unwrap();

    // Stamps far in the past are taken as given, so every frame is known.
    let pose = r#"{"header":{"stamp":{"sec":5,"nanosec":7},"frame_id":"map"},"pose":{"position":{"x":1.5},"orientation":{"w":1.0}}}"#;
    assert_eq!(
        runtime.publish_stamped(&pose_topic, pose, 1_000).unwrap(),
        0
    );
    let twist = r#"{"linear":{"x":1.0,"y":-2.5},"angular":{"z":0.5}}"#;
    assert_eq!(
        runtime.publish_stamped(&twist_topic, twist, 2_000).unwrap(),
        0
    );
    assert_eq!(
        runtime.publish_stamped(&twist_topic, "{}", 3_000).unwrap(),
        1
    );

    // A bad nested field is refused with its path, and sends nothing.
    let error = runtime
        .publish(&twist_topic, r#"{"angular":{"z":"fast"}}"#)
        .unwrap_err();
    assert!(matches!(error, EmbedError::Encode(_)), "{error:?}");
    assert_eq!(error.code(), codes::ENCODE);
    assert_eq!(error.to_string(), "angular.z: expected float64, got string");
    let published = runtime
        .status_handle()
        .lock()
        .unwrap()
        .published
        .get(&twist_topic)
        .copied();
    assert_eq!(published, Some(2));
    runtime.stop().unwrap();

    // PoseStamped: the fixed `pose` (seven float64s) is the fixed section,
    // and `header` is its one variable field, a headerless body of its own:
    // `stamp` (two 32-bit integers), an offset table row for `frame_id`,
    // then the text.
    let mut want_pose = header(
        schema_hash("geometry_msgs/PoseStamped"),
        123,
        56,
        1,
        0,
        1_000,
    );
    want_pose.extend_from_slice(&F64_ONE_AND_HALF); // position.x
    for _ in 0..5 {
        want_pose.extend_from_slice(&F64_ZERO); // position.y, z, orientation.x, y, z
    }
    want_pose.extend_from_slice(&F64_ONE); // orientation.w
    want_pose.extend_from_slice(&[64, 0, 0, 0, 19, 0, 0, 0]); // header @64, 19 B
    want_pose.extend_from_slice(&[5, 0, 0, 0, 7, 0, 0, 0]); // stamp
    want_pose.extend_from_slice(&[16, 0, 0, 0, 3, 0, 0, 0]); // frame_id @16, 3 B
    want_pose.extend_from_slice(b"map");

    // Twist: two fixed Vector3s, six float64s and no offset table.
    let twist_hash = schema_hash("geometry_msgs/Twist");
    let mut want_twist = header(twist_hash, 88, 48, 0, 0, 2_000);
    for value in [
        F64_ONE,
        F64_MINUS_TWO_AND_HALF,
        F64_ZERO,
        F64_ZERO,
        F64_ZERO,
        F64_HALF,
    ] {
        want_twist.extend_from_slice(&value);
    }
    let mut want_zero_twist = header(twist_hash, 88, 48, 0, 1, 3_000);
    want_zero_twist.extend_from_slice(&[0; 48]);

    let bag = files_dir.join("bags/nested.mcap");
    assert_eq!(recorded_frames(&bag, &pose_topic), vec![want_pose]);
    assert_eq!(
        recorded_frames(&bag, &twist_topic),
        vec![want_twist, want_zero_twist]
    );
}
