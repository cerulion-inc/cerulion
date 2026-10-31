// SPDX-License-Identifier: AGPL-3.0-only
//! A bag recorded with the version 1 wire header (a version 1 schema
//! descriptor, 32-byte frame headers) reads back with every user frame in the
//! current 40-byte layout, through every accessor: the owned message streams,
//! the frame walk and its resumption, and the retained span index. Reserved
//! `__cerulion/*` payloads are not wire frames and are never touched.
//!
//! The legacy bag is written here by the upstream `mcap` crate's writer (an
//! independent container writer), with hand-written descriptor and frame bytes;
//! every expectation is a hand-written byte vector.

use cerulion_bag::{BagReader, BagWriter, BagWriterConfig, TopicSchema};

/// A version 1 descriptor: version 1, recipe 3, the given hash, fixed size 24.
fn v1_descriptor(schema_hash: u64) -> Vec<u8> {
    let mut d = vec![0x01, 0x00, 0x03, 0x00, 0x00, 0x00];
    d.extend_from_slice(&schema_hash.to_le_bytes());
    d.extend_from_slice(&[0x18, 0x00, 0x00, 0x00]);
    assert_eq!(d.len(), 18);
    d
}

const HASH: u64 = 0x0102_0304_0506_0708;

/// A version 1 `Vector3`-shaped frame: 32-byte header (total 56, offset table
/// at 56, no entries) then three f64.
fn v1_frame(seq: u8, x: f64) -> Vec<u8> {
    let mut f = Vec::new();
    f.extend_from_slice(&HASH.to_le_bytes());
    f.extend_from_slice(&[56, 0, 0, 0]); // total_size
    f.extend_from_slice(&[56, 0, 0, 0]); // offset_table_offset
    f.extend_from_slice(&[0, 0, 0, 0]); // offset_table_count
    f.extend_from_slice(&[seq, 0, 0, 0]); // sequence
    f.extend_from_slice(&[0xE8, 0x03, 0, 0, 0, 0, 0, 0]); // timestamp_ns = 1000
    f.extend_from_slice(&x.to_le_bytes());
    f.extend_from_slice(&0.0f64.to_le_bytes());
    f.extend_from_slice(&(-x).to_le_bytes());
    assert_eq!(f.len(), 56);
    f
}

/// The same frame in the 40-byte layout: total and offset table grow by 8,
/// an untagged step tag at byte 32, the body unchanged.
fn v2_frame(seq: u8, x: f64) -> Vec<u8> {
    let mut f = Vec::new();
    f.extend_from_slice(&HASH.to_le_bytes());
    f.extend_from_slice(&[64, 0, 0, 0]);
    f.extend_from_slice(&[64, 0, 0, 0]);
    f.extend_from_slice(&[0, 0, 0, 0]);
    f.extend_from_slice(&[seq, 0, 0, 0]);
    f.extend_from_slice(&[0xE8, 0x03, 0, 0, 0, 0, 0, 0]);
    f.extend_from_slice(&[0; 8]); // step_tag: untagged
    f.extend_from_slice(&x.to_le_bytes());
    f.extend_from_slice(&0.0f64.to_le_bytes());
    f.extend_from_slice(&(-x).to_le_bytes());
    assert_eq!(f.len(), 64);
    f
}

/// A 40-byte reserved trace payload, recognisable and NOT a wire frame.
fn trace_payload() -> Vec<u8> {
    (0u8..40).collect()
}

/// Write a version 1 bag: one user channel (`/v3`, three frames) and the
/// scheduler-trace reserved channel (one record), both under version 1
/// descriptors, interleaved.
fn legacy_bag(frames: &[Vec<u8>]) -> Vec<u8> {
    let mut buf = std::io::Cursor::new(Vec::new());
    {
        let mut w = mcap::WriteOptions::new()
            .compression(None)
            .create(&mut buf)
            .unwrap();
        let user_schema = w
            .add_schema("geometry_msgs/Vector3", "cerulion", &v1_descriptor(HASH))
            .unwrap();
        let user = w
            .add_channel(user_schema, "/v3", "cerulion", &Default::default())
            .unwrap();
        let trace_schema = w
            .add_schema("cerulion.SchedulerTrace", "cerulion", &v1_descriptor(0))
            .unwrap();
        let trace = w
            .add_channel(
                trace_schema,
                "__cerulion/scheduler_trace",
                "cerulion",
                &Default::default(),
            )
            .unwrap();
        for (i, f) in frames.iter().enumerate() {
            w.write_to_known_channel(
                &mcap::records::MessageHeader {
                    channel_id: user,
                    sequence: i as u32,
                    log_time: 10 + i as u64,
                    publish_time: 10 + i as u64,
                },
                f,
            )
            .unwrap();
            if i == 0 {
                w.write_to_known_channel(
                    &mcap::records::MessageHeader {
                        channel_id: trace,
                        sequence: 0,
                        log_time: 10,
                        publish_time: 10,
                    },
                    &trace_payload(),
                )
                .unwrap();
            }
        }
        w.finish().unwrap();
    }
    buf.into_inner()
}

fn three_frames() -> Vec<Vec<u8>> {
    vec![v1_frame(0, 1.5), v1_frame(1, 2.5), v1_frame(2, 3.5)]
}

#[test]
fn a_version_1_bag_names_its_legacy_topics_and_its_descriptors_say_version_1() {
    let r = BagReader::from_bytes(legacy_bag(&three_frames()));
    assert_eq!(
        r.legacy_wire_header_topics().unwrap(),
        vec!["/v3".to_string()]
    );
    for c in r.channels().unwrap() {
        let d = c.descriptor.expect("cerulion descriptor");
        assert_eq!(d.descriptor_version(), 1, "{}", c.topic);
        assert_eq!(d.wire_header_version, 1, "{}", c.topic);
    }
}

#[test]
fn every_message_stream_upgrades_user_frames_and_leaves_reserved_payloads_alone() {
    let r = BagReader::from_bytes(legacy_bag(&three_frames()));
    let want = [v2_frame(0, 1.5), v2_frame(1, 2.5), v2_frame(2, 3.5)];

    let streamed: Vec<_> = r.messages().unwrap().map(|m| m.unwrap()).collect();
    let (recovered, completeness) = r.recover_messages().unwrap();
    assert!(completeness.is_finalized());
    for msgs in [&streamed, &recovered] {
        let user: Vec<&[u8]> = msgs
            .iter()
            .filter(|m| m.topic == "/v3")
            .map(|m| m.data.as_slice())
            .collect();
        assert_eq!(user, want.iter().map(|v| v.as_slice()).collect::<Vec<_>>());
        let trace: Vec<&[u8]> = msgs
            .iter()
            .filter(|m| m.topic == "__cerulion/scheduler_trace")
            .map(|m| m.data.as_slice())
            .collect();
        assert_eq!(trace, vec![trace_payload().as_slice()]);
    }
    let on_topic: Vec<Vec<u8>> = r
        .recover_messages_on_topic("/v3")
        .unwrap()
        .map(|m| m.unwrap().data)
        .collect();
    assert_eq!(on_topic, want.to_vec());
}

#[test]
fn the_frame_walk_its_resumption_and_the_span_index_serve_upgraded_frames() {
    let r = BagReader::from_bytes(legacy_bag(&three_frames()));
    let want = [v2_frame(0, 1.5), v2_frame(1, 2.5), v2_frame(2, 3.5)];

    let mut walk = r.user_frames().unwrap();
    let (_, first) = walk.next_user_frame().unwrap().expect("first frame");
    assert_eq!(first.len, 64);
    assert_eq!(r.frame(&first), want[0].as_slice());
    let position = walk.into_position();

    let mut resumed = r.resume_user_frames(position).unwrap();
    let mut rest = Vec::new();
    while let Some((_, span)) = resumed.next_user_frame().unwrap() {
        rest.push(r.frame(&span).to_vec());
    }
    assert_eq!(rest, want[1..].to_vec());

    let index = r.user_message_index().unwrap();
    assert_eq!(index.len(), 1, "the reserved channel is not a user frame");
    let spans = &index["/v3"];
    let served: Vec<&[u8]> = spans.iter().map(|s| r.frame(s)).collect();
    assert_eq!(
        served,
        want.iter().map(|v| v.as_slice()).collect::<Vec<_>>()
    );

    // A second walk serves the same upgraded bytes (the copy is made once).
    let mut again = r.user_frames().unwrap();
    let (_, s) = again.next_user_frame().unwrap().unwrap();
    assert_eq!(r.frame(&s), want[0].as_slice());
}

/// A version 1 frame that is not one whole frame (its `total_size` disagrees
/// with its length) is served UNCHANGED, as a headerless frame is: by the
/// message streams and by the frame walk, counted, and the bag stays usable
/// (its finalized probe and every other frame are unaffected).
#[test]
fn a_version_1_frame_that_is_not_whole_is_served_unchanged_and_counted() {
    // The second frame declares 57 bytes but carries 56.
    let mut frames = three_frames();
    frames[1][8] = 57;
    let broken = frames[1].clone();
    let r = BagReader::from_bytes(legacy_bag(&frames));
    assert!(r.has_readable_footer(), "the bag is intact");

    let user: Vec<Vec<u8>> = r
        .messages()
        .unwrap()
        .map(|m| m.unwrap())
        .filter(|m| m.topic == "/v3")
        .map(|m| m.data)
        .collect();
    assert_eq!(
        user,
        vec![v2_frame(0, 1.5), broken.clone(), v2_frame(2, 3.5)]
    );
    assert_eq!(r.legacy_passthrough_count(), 1);

    let mut walk = r.user_frames().unwrap();
    let mut walked = Vec::new();
    while let Some((_, span)) = walk.next_user_frame().unwrap() {
        walked.push(r.frame(&span).to_vec());
    }
    assert_eq!(walked, vec![v2_frame(0, 1.5), broken, v2_frame(2, 3.5)]);
    assert_eq!(
        r.legacy_passthrough_count(),
        2,
        "once per accessor that served it"
    );
}

/// A user channel stating a wire header version other than 1 or this build's
/// is refused by every accessor, naming the version.
#[test]
fn a_channel_at_an_unknown_wire_header_version_is_refused_naming_it() {
    let mut buf = std::io::Cursor::new(Vec::new());
    {
        let mut w = mcap::WriteOptions::new()
            .compression(None)
            .create(&mut buf)
            .unwrap();
        // A version 2 descriptor stating wire header version 7.
        let mut d = vec![0x02, 0x00, 0x03, 0x00, 0x00, 0x00];
        d.extend_from_slice(&HASH.to_le_bytes());
        d.extend_from_slice(&[0x18, 0x00, 0x00, 0x00, 0x07, 0x00]);
        let schema = w
            .add_schema("geometry_msgs/Vector3", "cerulion", &d)
            .unwrap();
        let ch = w
            .add_channel(schema, "/v7", "cerulion", &Default::default())
            .unwrap();
        w.write_to_known_channel(
            &mcap::records::MessageHeader {
                channel_id: ch,
                sequence: 0,
                log_time: 1,
                publish_time: 1,
            },
            &v2_frame(0, 1.0),
        )
        .unwrap();
        w.finish().unwrap();
    }
    let r = BagReader::from_bytes(buf.into_inner());
    for err in [
        r.legacy_wire_header_topics().unwrap_err(),
        r.user_frames().err().expect("the walk refuses"),
        r.messages()
            .unwrap()
            .find_map(Result::err)
            .expect("the stream refuses"),
    ] {
        assert!(
            matches!(
                err,
                cerulion_bag::BagError::UnsupportedWireHeader { version: 7, .. }
            ),
            "{err}"
        );
        assert!(err.to_string().contains("wire header version 7"), "{err}");
    }
}

#[test]
fn a_current_bag_is_served_byte_for_byte_from_the_map() {
    let topics = [TopicSchema {
        topic: "/v3".to_string(),
        schema_name: "geometry_msgs/Vector3".to_string(),
        schema_hash: HASH,
        wire_fixed_size: 24,
    }];
    let path = std::env::temp_dir().join(format!(
        "legacy_wire_current_{}_{}.mcap",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    let mut w = BagWriter::create(&path, BagWriterConfig::default(), &topics).unwrap();
    let frame = v2_frame(0, 1.5);
    w.write_message("/v3", 0, 10, 10, &[&frame]).unwrap();
    w.finalize().unwrap();

    let r = BagReader::open(&path).unwrap();
    assert!(r.legacy_wire_header_topics().unwrap().is_empty());
    let mut walk = r.user_frames().unwrap();
    let (_, span) = walk.next_user_frame().unwrap().unwrap();
    assert!(span.offset < r.bytes().len(), "served from the map");
    assert_eq!(r.frame(&span), frame.as_slice());
    let msgs: Vec<_> = r.messages().unwrap().map(|m| m.unwrap()).collect();
    let user: Vec<_> = msgs.iter().filter(|m| m.topic == "/v3").collect();
    assert_eq!(user[0].data, frame);
    std::fs::remove_file(&path).ok();
}

/// A version 1 frame carrying bytes after its declared `total_size` is
/// upgraded: the frame is the declared prefix, the trailing bytes follow the
/// upgraded frame unchanged, and nothing is counted as passed through, in
/// the message stream and the frame walk alike.
#[test]
fn a_version_1_frame_with_trailing_bytes_is_upgraded_and_keeps_them() {
    let mut frames = three_frames();
    frames[1].extend_from_slice(&[0xB1, 0xB2, 0xB3]); // declares 56, holds 59
    let mut want = v2_frame(1, 2.5);
    want.extend_from_slice(&[0xB1, 0xB2, 0xB3]);
    let r = BagReader::from_bytes(legacy_bag(&frames));

    let user: Vec<Vec<u8>> = r
        .messages()
        .unwrap()
        .map(|m| m.unwrap())
        .filter(|m| m.topic == "/v3")
        .map(|m| m.data)
        .collect();
    assert_eq!(user, vec![v2_frame(0, 1.5), want.clone(), v2_frame(2, 3.5)]);

    let mut walk = r.user_frames().unwrap();
    let mut walked = Vec::new();
    while let Some((_, span)) = walk.next_user_frame().unwrap() {
        walked.push(r.frame(&span).to_vec());
    }
    assert_eq!(walked, vec![v2_frame(0, 1.5), want, v2_frame(2, 3.5)]);
    assert_eq!(r.legacy_passthrough_count(), 0);
}

/// A version 1 bag whose upgraded frames exceed the limit is refused by the
/// frame walk naming `cerulion bag migrate`, while the as-recorded walk still
/// serves its frames in the 32-byte layout and the message stream still
/// upgrades them one at a time.
#[test]
fn a_version_1_bag_past_the_upgrade_limit_is_refused_by_the_walk_only() {
    let r = BagReader::from_bytes(legacy_bag(&three_frames()));
    // Three upgraded frames are 3 * 64 = 192 bytes; allow 191.
    r.set_legacy_upgrade_cap_for_test(191);
    match r.user_frames() {
        Err(cerulion_bag::BagError::LegacyUpgrade { reason }) => {
            assert!(reason.contains("192 bytes"), "{reason}");
            assert!(reason.contains("cerulion bag migrate"), "{reason}");
        }
        Err(other) => panic!("expected the upgrade limit refusal, got {other}"),
        Ok(_) => panic!("expected the upgrade limit refusal, got a walk"),
    }
    assert!(r.has_readable_footer(), "the bag is intact, not torn");

    let mut walk = r.user_frames_as_recorded().unwrap();
    let mut recorded = Vec::new();
    while let Some((_, span)) = walk.next_user_frame().unwrap() {
        recorded.push(r.frame(&span).to_vec());
    }
    assert_eq!(recorded, three_frames());

    let streamed: Vec<Vec<u8>> = r
        .messages()
        .unwrap()
        .map(|m| m.unwrap())
        .filter(|m| m.topic == "/v3")
        .map(|m| m.data)
        .collect();
    assert_eq!(
        streamed,
        vec![v2_frame(0, 1.5), v2_frame(1, 2.5), v2_frame(2, 3.5)]
    );

    // At exactly the limit the walk is served.
    let r = BagReader::from_bytes(legacy_bag(&three_frames()));
    r.set_legacy_upgrade_cap_for_test(192);
    assert!(r.user_frames().is_ok());
}
