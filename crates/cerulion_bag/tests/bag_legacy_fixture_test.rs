// SPDX-License-Identifier: AGPL-3.0-only
//! Bags written by the release before the 40-byte wire header, read by this
//! build. The fixtures under `tests/fixtures/legacy_v1_*.mcap` are that
//! release's own byte-pin fixtures, byte for byte (the writer of commit
//! af0541f056): version 1 schema descriptors, and user payloads that are
//! short opaque bytes rather than wire frames. Every expectation is the
//! literal the old fixture's writer was given.

use std::path::Path;

use cerulion_bag::BagReader;

fn open(name: &str) -> BagReader {
    BagReader::open(
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures")
            .join(format!("legacy_v1_{name}.mcap")),
    )
    .expect("open the legacy fixture")
}

/// The two-topic fixture: both user topics are named as version 1, their
/// payloads (too short to be wire frames) come back exactly as recorded and
/// are counted as passed through, and the reserved channel and the
/// attachment are untouched.
#[test]
fn a_bag_from_the_32_byte_release_reads_with_its_legacy_topics_named() {
    let r = open("two_topic_determinism");
    assert!(r.has_readable_footer());
    assert_eq!(
        r.legacy_wire_header_topics().unwrap(),
        vec!["/camera".to_string(), "/imu".to_string()]
    );
    for c in r.channels().unwrap() {
        let d = c.descriptor.expect("a cerulion descriptor");
        assert_eq!(
            (d.descriptor_version(), d.wire_header_version),
            (1, 1),
            "{}",
            c.topic
        );
    }
    let msgs: Vec<_> = r.messages().unwrap().map(|m| m.unwrap()).collect();
    let user: Vec<(&str, &[u8])> = msgs
        .iter()
        .filter(|m| !m.topic.starts_with("__cerulion/"))
        .map(|m| (m.topic.as_str(), m.data.as_slice()))
        .collect();
    assert_eq!(
        user,
        vec![("/camera", &[0x11u8; 20][..]), ("/imu", &[0x22u8; 12][..])]
    );
    assert_eq!(r.legacy_passthrough_count(), 2);
    let trace = r.scheduler_trace().unwrap();
    assert_eq!(trace.len(), 1);
    assert_eq!((trace[0].step, trace[0].fire_time_ns), (7, 42));
    let att = r.attachment("graph.yaml").unwrap().expect("the attachment");
    assert_eq!(att.data, b"g: 1\n");

    // The frame walk serves the same bytes from the map.
    let index = r.user_message_index().unwrap();
    assert_eq!(r.frame(&index["/camera"][0]), &[0x11u8; 20][..]);
    assert_eq!(r.frame(&index["/imu"][0]), &[0x22u8; 12][..]);
}

/// The six-message fixture: three topics, one message assembled from two
/// parts, all served as recorded.
#[test]
fn the_widest_legacy_fixture_serves_every_payload_as_recorded() {
    let r = open("six_message_roundtrip");
    assert_eq!(
        r.legacy_wire_header_topics().unwrap(),
        vec![
            "/camera/image".to_string(),
            "/imu".to_string(),
            "/pose".to_string()
        ]
    );
    let index = r.user_message_index().unwrap();
    let served: Vec<Vec<u8>> = index["/camera/image"]
        .iter()
        .map(|s| r.frame(s).to_vec())
        .collect();
    assert_eq!(served, vec![vec![0xAA; 16], vec![0xCC; 16]]);
    assert_eq!(
        r.frame(&index["/imu"][0]),
        &[0x01, 0x02, 0x03, 0x04, 0x05][..]
    );
    assert_eq!(r.frame(&index["/pose"][0]), &[0xBB; 8][..]);
}

/// The empty fixture: reserved channels only, so nothing is legacy.
#[test]
fn a_legacy_bag_with_no_user_topic_names_none() {
    let r = open("empty_topic_list");
    assert!(r.legacy_wire_header_topics().unwrap().is_empty());
    assert_eq!(r.user_message_index().unwrap().len(), 0);
}
