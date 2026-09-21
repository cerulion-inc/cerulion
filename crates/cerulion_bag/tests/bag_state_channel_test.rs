// SPDX-License-Identifier: AGPL-3.0-only
//! The THIRD hardcoded reserved channel,
//! `__cerulion/state`, and the record-length gate that keeps it fixed-size.
//!
//! The channel is registered UNCONDITIONALLY, exactly like its two siblings, so
//! the channel-id assignment stays a function of the registration list alone
//! rather than of whether a particular run happened to checkpoint. The arms
//! here pin the three things a reader depends on and one thing a WRITER must be
//! refused:
//!
//! - the SLASH-LESS topic spelling. `UserFrameWalk::next_user_frame` tests
//!   `RESERVED_PREFIX` (`__cerulion/`, no leading slash), so a `/__cerulion/state`
//!   channel would NOT be skipped — it would reach replay's `classify_topics` as
//!   a topic the graph neither produces nor consumes and make every checkpointed
//!   bag fail `BagGraphMismatch` (exit 2, "corrupt or hand-edited recording").
//!   Asserted through the production reader's own skip walk, not by comparing
//!   the constant to a literal;
//! - the descriptor (`schema_hash` 0, `wire_fixed_size` = `STATE_RECORD_SIZE`) —
//!   hash 0 for the same reason its siblings carry it, the payload being a
//!   framework record with no recipe-3 schema to look up;
//! - the records round-trip BYTE-IDENTICALLY, against records built by
//!   `cerulion_core`'s own `encode_record` / `encode_skip_record` and compared to
//!   a HAND-BUILT expectation (never a self-compare against a second run of the
//!   same writer);
//! - a payload that is NOT exactly one record is REFUSED, on `write_message`
//!   itself, because that is the entry point the recorder splices ring SHM spans
//!   through — a gate on a convenience wrapper alone would be reachable by no
//!   production caller.
//!
//! Parallel-safe: pure file I/O into per-test temp paths, no transport.

#![cfg(unix)]

use std::path::PathBuf;

use cerulion_bag::{
    BagError, BagReader, BagWriter, BagWriterConfig, TopicSchema, RESERVED_PREFIX,
    STATE_RECORD_SIZE, STATE_SCHEMA, STATE_TOPIC,
};
use cerulion_core::state_ring::{
    encode_record, encode_skip_record, SkipCause, StateRecordHeader, RECORD_KIND_CHUNK,
    RECORD_KIND_FINAL, STATE_RECORD_PAYLOAD,
};

fn tmp(tag: &str) -> PathBuf {
    let mut p = std::env::temp_dir();
    p.push(format!(
        "state_chan_{tag}_{}_{}.mcap",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    p
}

fn user_topic(t: &str) -> TopicSchema {
    TopicSchema {
        topic: t.into(),
        schema_name: "sensor_msgs/Image".into(),
        schema_hash: 0x1111_2222_3333_4444,
        wire_fixed_size: 64,
    }
}

/// A CHUNK record whose payload is `fill` repeated over the full payload region,
/// and a FINAL record carrying `tail`. Hand-built: the header fields are chosen
/// here, so the read-back comparison below is against a value this test decided.
fn anchor_records(run_id: u64, step: u64, node_idx: u32, fill: u8, tail: &[u8]) -> Vec<Vec<u8>> {
    let body = vec![fill; STATE_RECORD_PAYLOAD];
    let chunk = encode_record(
        &StateRecordHeader {
            run_id,
            step,
            node_idx,
            part: 0,
            kind: RECORD_KIND_CHUNK,
            len: STATE_RECORD_PAYLOAD as u32,
        },
        &body,
    );
    let final_rec = encode_record(
        &StateRecordHeader {
            run_id,
            step,
            node_idx,
            part: 1,
            kind: RECORD_KIND_FINAL,
            len: tail.len() as u32,
        },
        tail,
    );
    vec![chunk.to_vec(), final_rec.to_vec()]
}

/// THE round trip: state records land on the reserved channel byte-for-byte and
/// come back in write order, while the reader's USER-frame walk skips them.
#[test]
fn state_records_round_trip_on_the_reserved_channel_and_stay_out_of_the_user_walk() {
    let p = tmp("roundtrip");
    let mut w = BagWriter::create(
        &p,
        BagWriterConfig::default(),
        &[user_topic("/camera/image")],
    )
    .unwrap();

    // Resolve ONCE, like the recorder does — the id path is what production
    // uses, so it is the path under test.
    let state_id = w.state_channel_id();
    let expected = anchor_records(0xDEAD_BEEF_CAFE_F00D, 41, 7, 0xA5, b"tail bytes");
    let skip = encode_skip_record(
        0xDEAD_BEEF_CAFE_F00D,
        42,
        7,
        SkipCause::Contended,
        "node mutex held at the pre-fork probe",
    );

    w.write_chunk(|s| {
        // One user frame, so the bag is not state-only and the walk below has
        // something it MUST still yield.
        s.write_message("/camera/image", 0, 1_000, 1_000, &[&[9u8; 64]])?;
        for (i, rec) in expected.iter().enumerate() {
            s.write_message(
                state_id,
                i as u32,
                2_000 + i as u64,
                2_000 + i as u64,
                &[rec],
            )?;
        }
        s.write_message(state_id, 2, 2_002, 2_002, &[&skip])?;
        Ok(())
    })
    .unwrap();
    w.finalize().unwrap();

    let r = BagReader::open(&p).unwrap();

    // (a) the channel is there, with the descriptor a reader keys on.
    let channels = r.channels().unwrap();
    let state = channels
        .iter()
        .find(|c| c.topic == STATE_TOPIC)
        .expect("the state channel is registered in every bag");
    assert_eq!(state.schema_name, STATE_SCHEMA);
    let d = state.descriptor.expect("a cerulion descriptor");
    assert_eq!(d.schema_hash, 0, "a framework record has no user schema");
    assert_eq!(d.wire_fixed_size, STATE_RECORD_SIZE);

    // (b) the records come back BYTE-IDENTICAL, in write order, against the
    //     hand-built expectation.
    let got: Vec<_> = r
        .messages()
        .unwrap()
        .collect::<Result<Vec<_>, _>>()
        .unwrap()
        .into_iter()
        .filter(|m| m.topic == STATE_TOPIC)
        .collect();
    assert_eq!(got.len(), 3, "two anchor records + one SKIP");
    assert_eq!(got[0].data, expected[0]);
    assert_eq!(got[1].data, expected[1]);
    assert_eq!(got[2].data, skip.to_vec());
    for m in &got {
        assert_eq!(
            m.data.len(),
            STATE_RECORD_SIZE as usize,
            "every state record is exactly one fixed-size record"
        );
    }

    // (c) the USER walk skips them — the slash-less spelling, proven through the
    //     production reader rather than by comparing the constant to a literal.
    assert!(
        STATE_TOPIC.starts_with(RESERVED_PREFIX),
        "the reserved prefix is slash-less, so the topic must be too"
    );
    let channel_topic: std::collections::HashMap<u16, String> =
        channels.iter().map(|c| (c.id, c.topic.clone())).collect();
    let mut walk = r.user_frames().unwrap();
    let mut user_topics: Vec<String> = Vec::new();
    while let Some((channel_id, _span)) = walk.next_user_frame().unwrap() {
        user_topics.push(channel_topic[&channel_id].clone());
    }
    assert_eq!(
        user_topics,
        vec!["/camera/image".to_string()],
        "the user walk must yield ONLY the user frame — a state record reaching \
         it is what makes replay refuse a checkpointed bag"
    );

    std::fs::remove_file(&p).ok();
}

/// The LENGTH GATE, on the entry point the recorder actually uses.
///
/// Both directions: one byte short and one byte long, plus the exact size as the
/// anti-tautology control (a gate that refused everything would pass the two
/// negative arms on its own).
#[test]
fn a_payload_that_is_not_exactly_one_record_is_refused_on_the_state_channel() {
    let p = tmp("lengate");
    let mut w = BagWriter::create(&p, BagWriterConfig::default(), &[user_topic("/a")]).unwrap();
    let state_id = w.state_channel_id();

    let short = vec![0u8; STATE_RECORD_SIZE as usize - 1];
    let long = vec![0u8; STATE_RECORD_SIZE as usize + 1];
    let exact = vec![0u8; STATE_RECORD_SIZE as usize];

    let err = w
        .write_message(state_id, 0, 1, 1, &[&short])
        .expect_err("a short payload must be refused");
    match err {
        BagError::BadStateRecordLen {
            channel,
            actual,
            expected,
        } => {
            assert_eq!(channel, STATE_TOPIC);
            assert_eq!(actual, STATE_RECORD_SIZE as usize - 1);
            assert_eq!(expected, STATE_RECORD_SIZE as usize);
        }
        other => panic!("expected BadStateRecordLen, got {other:?}"),
    }
    assert!(
        matches!(
            w.write_message(state_id, 0, 1, 1, &[&long]),
            Err(BagError::BadStateRecordLen { .. })
        ),
        "a long payload must be refused too"
    );
    // The control: the exact size is ACCEPTED, so the two arms above are about
    // the length and not about the channel being write-protected.
    w.write_message(state_id, 0, 1, 1, &[&exact])
        .expect("exactly one record must be accepted");

    // And the gate composes over SPLIT parts, which is how the recorder writes
    // (a record spliced out of ring SHM can arrive as more than one slice):
    // the sum is what is judged.
    let (a, b) = exact.split_at(100);
    w.write_message(state_id, 1, 2, 2, &[a, b])
        .expect("two parts summing to one record must be accepted");

    w.finalize().unwrap();
    std::fs::remove_file(&p).ok();
}

/// Byte-determinism, over records: the SAME call sequence produces byte-identical
/// files. `cerulion_bag` reads no clock, so this is a property of the writer and
/// the new channel must not have introduced one.
#[test]
fn two_runs_of_one_state_record_sequence_are_byte_identical() {
    let write_one = |p: &std::path::Path| {
        let mut w = BagWriter::create(p, BagWriterConfig::default(), &[user_topic("/a")]).unwrap();
        let state_id = w.state_channel_id();
        let recs = anchor_records(7, 3, 1, 0x5A, b"xyz");
        w.write_chunk(|s| {
            for (i, rec) in recs.iter().enumerate() {
                s.write_message(state_id, i as u32, 10 + i as u64, 10 + i as u64, &[rec])?;
            }
            Ok(())
        })
        .unwrap();
        w.finalize().unwrap();
        std::fs::read(p).unwrap()
    };
    let p1 = tmp("det1");
    let p2 = tmp("det2");
    let a = write_one(&p1);
    let b = write_one(&p2);
    assert_eq!(
        a, b,
        "two identical state-record call sequences must produce identical bytes"
    );
    assert!(!a.is_empty());
    std::fs::remove_file(&p1).ok();
    std::fs::remove_file(&p2).ok();
}
