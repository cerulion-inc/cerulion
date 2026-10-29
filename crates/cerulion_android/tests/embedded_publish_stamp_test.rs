// SPDX-License-Identifier: AGPL-3.0-only
//! App frames take their wire stamp from the transport clock, the clock that
//! stamps graph node frames. Each test runs the runtime on a `VirtualClock`,
//! records the app topic and reads the bag back: the stamp bytes in the frame
//! header and the record's log time are checked against hand-written values.
//!
//! The process transport manager keeps the clock of the start that created
//! it, so every runtime in this binary shares one clock and each test sets
//! its time: run it with `--test-threads=1`.

use std::path::Path;
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant};

use cerulion_android::{codes, EmbedError, EmbeddedRuntime};
use cerulion_bag::BagReader;
use cerulion_core::clock::VirtualClock;
use serde_json::json;
use tempfile::tempdir;

cerulion_android::embed_nodes! {
    "test_node_macro_relay_rlib" => test_node_macro_relay_rlib::RelayNodeEntry,
}

const JNI_VERSION_1_6: i32 = 0x0001_0006;

/// The one clock of this binary, set to `time_ns`.
fn clock_at(time_ns: u64) -> Arc<VirtualClock> {
    static CLOCK: OnceLock<Arc<VirtualClock>> = OnceLock::new();
    let clock = Arc::clone(CLOCK.get_or_init(|| Arc::new(VirtualClock::new())));
    clock.set(time_ns);
    clock
}

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
    let clock = clock_at(0x0102_0304_0506_0708);
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

#[test]
fn a_caller_stamp_in_bound_is_recorded_as_given() {
    let temp = tempdir().unwrap();
    let files_dir = temp.path();
    let topic = format!("/stamp_caller_{}/heartbeat", std::process::id());
    let clock = clock_at(10_000_000_000);
    let mut runtime = EmbeddedRuntime::new(files_dir.to_path_buf(), "android_test".into()).unwrap();
    runtime.set_clock_for_test(clock.clone());

    runtime
        .start(&recording_config("caller.mcap", &topic))
        .unwrap();
    // A stamp four seconds old, then one exactly one second ahead of the
    // clock: the largest lead the runtime takes.
    assert_eq!(
        runtime
            .publish_stamped(&topic, r#"{"data":"old"}"#, 4_000_000_000)
            .unwrap(),
        0
    );
    assert_eq!(
        runtime
            .publish_stamped(&topic, r#"{"data":"ahead"}"#, 11_000_000_000)
            .unwrap(),
        1
    );
    runtime.stop().unwrap();

    assert_eq!(
        recorded_stamps(&files_dir.join("bags/caller.mcap"), &topic),
        vec![
            (
                [0x00, 0x28, 0x6b, 0xee, 0x00, 0x00, 0x00, 0x00],
                4_000_000_000
            ),
            (
                [0x00, 0xae, 0xa6, 0x8f, 0x02, 0x00, 0x00, 0x00],
                11_000_000_000
            ),
        ]
    );
}

#[test]
fn a_caller_stamp_past_the_bound_is_refused_and_sends_nothing() {
    let temp = tempdir().unwrap();
    let files_dir = temp.path();
    let topic = format!("/stamp_refused_{}/heartbeat", std::process::id());
    let clock = clock_at(10_000_000_000);
    let mut runtime = EmbeddedRuntime::new(files_dir.to_path_buf(), "android_test".into()).unwrap();
    runtime.set_clock_for_test(clock.clone());

    runtime
        .start(&recording_config("refused.mcap", &topic))
        .unwrap();
    for stamp in [11_000_000_001, u64::MAX] {
        let error = runtime
            .publish_stamped(&topic, r#"{"data":"future"}"#, stamp)
            .unwrap_err();
        assert!(matches!(error, EmbedError::InvalidArgument(_)), "{error:?}");
        assert_eq!(error.code(), codes::INVALID_ARGUMENT);
    }
    let error = runtime
        .publish_stamped(&topic, r#"{"data":"future"}"#, 11_000_000_001)
        .unwrap_err();
    assert_eq!(
        error.to_string(),
        "publish stamp 11000000001 ns is more than 1000000000 ns ahead of the runtime clock (10000000000 ns)"
    );
    // A refused publish takes no sequence number: the next frame is 0.
    assert_eq!(runtime.publish(&topic, r#"{"data":"now"}"#).unwrap(), 0);
    let published = runtime
        .status_handle()
        .lock()
        .unwrap()
        .published
        .get(&topic)
        .copied();
    assert_eq!(published, Some(1));
    runtime.stop().unwrap();

    assert_eq!(
        recorded_stamps(&files_dir.join("bags/refused.mcap"), &topic),
        vec![(
            [0x00, 0xe4, 0x0b, 0x54, 0x02, 0x00, 0x00, 0x00],
            10_000_000_000
        )]
    );
}

#[test]
fn two_runs_on_the_same_clock_record_identical_frames() {
    let run = |name: &str| -> Vec<Vec<u8>> {
        let temp = tempdir().unwrap();
        let files_dir = temp.path();
        let topic = format!("/stamp_twice_{}/heartbeat", std::process::id());
        let clock = clock_at(7_000_000_000);
        let mut runtime =
            EmbeddedRuntime::new(files_dir.to_path_buf(), "android_test".into()).unwrap();
        runtime.set_clock_for_test(clock.clone());
        runtime.start(&recording_config(name, &topic)).unwrap();
        runtime.publish(&topic, r#"{"data":"a"}"#).unwrap();
        clock.advance(20_000_000);
        runtime
            .publish_stamped(&topic, r#"{"data":"b"}"#, 6_500_000_000)
            .unwrap();
        runtime.stop().unwrap();
        let reader = BagReader::open(files_dir.join("bags").join(name)).unwrap();
        reader
            .messages()
            .unwrap()
            .map(|message| message.unwrap())
            .filter(|message| message.topic == topic)
            .map(|message| message.data)
            .collect()
    };
    let first = run("first.mcap");
    assert_eq!(first.len(), 2);
    assert_eq!(first, run("second.mcap"));
}

/// Graph node frames take their stamp from the same clock: a linked relay
/// reads the app topic, and its echo frame carries the injected clock's
/// time, set far past any real monotonic reading (over two years of uptime).
#[test]
fn a_graph_node_stamps_its_frames_from_the_runtime_clock() {
    assert_eq!(
        JNI_OnLoad(std::ptr::null_mut(), std::ptr::null_mut()),
        JNI_VERSION_1_6
    );
    let temp = tempdir().unwrap();
    let files_dir = temp.path();
    let pid = std::process::id();
    let app_topic = format!("/stamp_graph_{pid}/cmd");
    let prefix = format!("stamp_graph_{pid}");
    let echo_topic = format!("/{prefix}/logic/echo");
    let graph_yaml = format!(
        "prefix: {prefix}
nodes:
  - id: logic
    type: test_node_macro_relay_rlib
    inputs:
      - name: cmd
        source: {app_topic}
    outputs:
      - name: echo
        schema: geometry_msgs/Vector3
"
    );
    const BASE: u64 = 0x0100_0000_0000_0000;
    let clock = clock_at(BASE);
    let mut runtime = EmbeddedRuntime::new(files_dir.to_path_buf(), "android_test".into()).unwrap();
    runtime.set_clock_for_test(clock.clone());
    runtime
        .start(
            &json!({
                "graph_yaml": graph_yaml,
                "record": {"file": "graph.mcap"},
                "publish": [{"topic": app_topic, "schema": "geometry_msgs/Vector3"}],
                "subscribe": [{"topic": echo_topic, "schema": "geometry_msgs/Vector3"}]
            })
            .to_string(),
        )
        .unwrap();
    runtime
        .publish(&app_topic, r#"{"x":1.0,"y":0.0,"z":0.0}"#)
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    while runtime.poll(&echo_topic).unwrap().is_none() {
        assert!(Instant::now() < deadline, "no echo from the relay");
        std::thread::sleep(Duration::from_millis(10));
    }
    runtime.stop().unwrap();

    let echoes = recorded_stamps(&files_dir.join("bags/graph.mcap"), &echo_topic);
    assert_eq!(echoes.len(), 1, "{echoes:?}");
    assert_eq!(
        echoes,
        vec![(
            [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01],
            72_057_594_037_927_936
        )]
    );
}
