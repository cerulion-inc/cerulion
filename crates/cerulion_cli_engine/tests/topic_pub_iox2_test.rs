// SPDX-License-Identifier: AGPL-3.0-only
//! `topic pub` over iceoryx2: the frames reach a subscriber on the topic
//! with the encoded values, consecutive sequence numbers and the type's
//! schema hash, and the report counts the frames and the subscriber; and
//! `topic snapshot` waits out its timeout for a topic that never appears.

use std::sync::atomic::AtomicBool;
use std::time::Duration;

use cerulion_cli_engine::topic_cmd::publish::{topic_pub, PubRequest};
use cerulion_cli_engine::topic_cmd::TopicScope;
use serial_test::serial;

#[test]
#[serial]
fn published_frames_reach_a_local_subscriber_with_the_given_values() {
    let topic = "/c1_test/pub/cmd_vel";
    let transport = cerulion_core::TransportManager::get_or_init().expect("transport");
    let subscriber = transport.create_subscriber(topic).expect("subscriber");
    // The reader runs while the verb publishes: a subscriber reads a frame
    // while its publisher is open.
    let reader = std::thread::spawn(move || {
        let mut seen = Vec::new();
        for _ in 0..100 {
            subscriber
                .wait_for_message(Duration::from_millis(100), |msg| {
                    seen.push((
                        msg.header().sequence,
                        msg.header().schema_hash,
                        msg.payload().to_vec(),
                    ));
                })
                .expect("receive");
            if seen.len() >= 3 {
                break;
            }
        }
        seen
    });
    let running = AtomicBool::new(true);
    let req = PubRequest {
        topic,
        type_name: "geometry_msgs/msg/Twist",
        message: r#"{"linear": {"x": 0.25}, "angular": {"z": -1}}"#,
        times: 3,
        rate_hz: Some(20.0),
        wait_subscribers: Duration::from_secs(5),
        scope: TopicScope::Local,
    };
    let report = topic_pub(&req, None, &running).expect("publish");
    assert_eq!(
        report.line(),
        format!("published: topic={topic} type=geometry_msgs/Twist frames=3 subscribers=1")
    );

    let mut want_body = Vec::new();
    for v in [0.25f64, 0.0, 0.0, 0.0, 0.0, -1.0] {
        want_body.extend_from_slice(&v.to_le_bytes());
    }
    let (walker, _) = cerulion_core::codegen::FrameWalker::new(
        cerulion_cli_engine::schema_cmd::parse_builtin_schemas(),
    );
    let twist_hash = walker.schema_hash_for("geometry_msgs/Twist").unwrap();
    let seen = reader.join().expect("reader thread");
    assert_eq!(
        seen,
        (1..=3)
            .map(|seq| (seq, twist_hash, want_body.clone()))
            .collect::<Vec<_>>()
    );
}

#[test]
#[serial]
fn a_bad_value_publishes_nothing_and_is_an_encode_error() {
    let running = AtomicBool::new(true);
    let req = PubRequest {
        topic: "/c1_test/pub/bad",
        type_name: "std_msgs/UInt8",
        message: "data: 256",
        times: 1,
        rate_hz: None,
        wait_subscribers: Duration::ZERO,
        scope: TopicScope::Local,
    };
    let err = topic_pub(&req, None, &running).unwrap_err();
    assert_eq!(
        err.exit_code(),
        cerulion_cli_engine::message_text::EXIT_ENCODE,
        "{err}"
    );
    let transport = cerulion_core::TransportManager::get_or_init().expect("transport");
    assert_eq!(transport.topic_publisher_count("/c1_test/pub/bad"), 0);
}

#[test]
#[serial]
fn a_snapshot_of_a_topic_that_never_appears_is_unavailable_after_the_timeout() {
    use cerulion_cli_engine::topic_cmd::snapshot::{topic_snapshot, SnapshotRequest};
    let running = AtomicBool::new(true);
    let req = SnapshotRequest {
        topic: "/c1_test/snapshot/no_camera",
        output: None,
        timeout: Duration::from_millis(500),
        quality: 80,
        scope: TopicScope::Local,
    };
    let started = std::time::Instant::now();
    let err = topic_snapshot(&req, None, &running).unwrap_err();
    assert_eq!(
        err.exit_code(),
        cerulion_cli_engine::message_text::EXIT_UNAVAILABLE,
        "{err}"
    );
    assert!(started.elapsed() >= Duration::from_millis(500));
}
