// SPDX-License-Identifier: AGPL-3.0-only
//! Validation + edge cases: reserved-prefix rejection, unknown-topic write
//! rejection, duplicate-topic rejection, the channel-id-space cap (pinned on
//! both sides), bad scheduler-trace length, the
//! producer-label length gate + its round trip, the
//! empty-topic-list + zero-message finalize (both produce valid oracle-readable
//! bags), attachment round-trip, and the descriptor encode/decode byte oracle.

use std::path::PathBuf;

use cerulion_bag::{
    BagError, BagReader, BagWriter, BagWriterConfig, ProducerAttribution, ProducerRecord,
    SchemaDescriptor, TopicSchema, NONDETERMINISM_TOPIC, PRODUCER_RECORD_SIZE,
    SCHEDULER_TRACE_TOPIC,
};

/// A unique scratch path — process id + a monotonic counter, no clock (see
/// `bag_late_channel_test::tmp`).
fn tmp() -> PathBuf {
    static C: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let n = C.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let p = std::env::temp_dir().join(format!("cerulion_bag_val_{}_{}", std::process::id(), n));
    // A REUSED pid meeting a leftover artifact from an interrupted earlier run
    // must not leak into this run's assertions (a rejection-path test never
    // truncates the path it refuses to create), so the path is cleared at
    // issuance — deterministic, and still clock-free.
    let _ = std::fs::remove_file(&p);
    p
}

fn topic(t: &str) -> TopicSchema {
    TopicSchema {
        topic: t.into(),
        schema_name: "std_msgs/UInt8".into(),
        schema_hash: 0x1,
        wire_fixed_size: 1,
    }
}

#[test]
fn reserved_prefix_user_topic_is_rejected() {
    let p = tmp();
    let err = match BagWriter::create(
        &p,
        BagWriterConfig::default(),
        &[topic("__cerulion/scheduler_trace")],
    ) {
        Err(e) => e,
        Ok(_) => panic!("reserved prefix must be rejected"),
    };
    assert!(
        matches!(err, BagError::ReservedTopicPrefix { .. }),
        "got {err:?}"
    );
    // Validation runs before the file is opened, so nothing should exist.
    assert!(!p.exists(), "an invalid config must not leave a stray file");
    std::fs::remove_file(&p).ok();
}

#[test]
fn duplicate_topic_is_rejected() {
    let p = tmp();
    let err = match BagWriter::create(&p, BagWriterConfig::default(), &[topic("/a"), topic("/a")]) {
        Err(e) => e,
        Ok(_) => panic!("duplicate topic must be rejected"),
    };
    assert!(
        matches!(err, BagError::DuplicateTopic { .. }),
        "got {err:?}"
    );
    std::fs::remove_file(&p).ok();
}

/// The 16-bit channel-id space is a FAIL-CLOSED boundary,
/// pinned on BOTH sides in one body.
///
/// A channel id is a `u16` and both id-assignment sites derive it from an index
/// with `as u16`, so one channel past the space wraps to id 0 and two channels
/// silently share an id. The registration total is `user topics + reserved`,
/// which is why the fourth reserved channel moved the boundary. So the
/// arms sit at `CAP - RESERVED` (accepted) and one past it (refused), and the
/// accepted arm's oracle is the id of the LAST reserved channel: `u16::MAX`
/// exactly, i.e. the top of the space is reachable and nothing wrapped.
///
/// The refused arm is FIRST so the accepted arm's size is checked against the
/// error's own `reserved`/`cap` before it is built — a fifth reserved channel
/// then fails HERE, attributably, instead of silently making the accepted arm
/// one topic short of the boundary it claims to sit on.
#[test]
fn the_channel_id_space_is_a_fail_closed_boundary_pinned_on_both_sides() {
    const CAP: usize = u16::MAX as usize + 1;
    /// The reserved channels `BagWriter` auto-registers — the count
    /// `empty_topic_list_still_registers_reserved_channels` pins independently.
    const RESERVED: usize = 4;

    let names: Vec<String> = (0..CAP - RESERVED + 1)
        .map(|i| format!("/t{i:05}"))
        .collect();
    let mut topics: Vec<TopicSchema> = names.iter().map(|n| topic(n)).collect();

    // --- one past the cap: refused, naming both addends, leaving no file ---
    let p = tmp();
    let err = match BagWriter::create(&p, BagWriterConfig::default(), &topics) {
        Err(e) => e,
        Ok(_) => panic!("a registration past the channel-id space must be refused"),
    };
    assert!(
        matches!(
            err,
            BagError::TooManyChannels { topics: t, reserved, total, cap }
                if t == CAP - RESERVED + 1
                    && reserved == RESERVED
                    && total == CAP + 1
                    && cap == CAP
        ),
        "got {err:?}"
    );
    assert!(
        !p.exists(),
        "an over-cap registration must not leave a stray file"
    );
    std::fs::remove_file(&p).ok();

    // --- exactly at the cap: accepted, and the top id is reachable ---
    topics.pop();
    assert_eq!(topics.len(), CAP - RESERVED);
    let p = tmp();
    let w = BagWriter::create(&p, BagWriterConfig::default(), &topics)
        .expect("a registration that exactly fills the channel-id space is legal");
    // User topics take ids 0..CAP-RESERVED, the reserved channels the rest — so
    // the last reserved channel by name sits on `u16::MAX`. A wrap would put a
    // channel on id 0, colliding with the canonically-first user topic.
    assert_eq!(
        w.channel_id(cerulion_bag::FRAME_PRODUCERS_TOPIC),
        Some((CAP - RESERVED) as u16)
    );
    assert_eq!(w.channel_id(cerulion_bag::STATE_TOPIC), Some(u16::MAX));
    assert_eq!(w.channel_id("/t00000"), Some(0));
    w.finalize().unwrap();
    std::fs::remove_file(&p).ok();
}

/// The SCHEMA-id space is a SECOND, one-SMALLER fail-closed
/// boundary — REFUSAL side.
///
/// Schema ids are assigned `(i + 1) as u16` over the DISTINCT `(schema name,
/// descriptor)` identities, because MCAP reserves id 0 for "no schema" — so the
/// usable space is `u16::MAX`, one smaller than the channel space, and the one
/// shape that overruns it is a registration that exactly FILLS the channel space
/// with all-distinct schemas: `65,532` user topics each carrying its own type
/// plus the 4 reserved channels = `65,536` identities against `65,535` ids.
/// That is what this builds, and it is what makes the arm a discriminator rather
/// than a restatement: the channel total sits exactly AT its own cap, so the
/// channel guard passes and only the schema guard can refuse it.
///
/// REFUSAL side only, and the cost is MEASURED rather than estimated: an
/// accepted-at-cap arm would have to get PAST validation and into `with_sink`'s
/// dedup, which is an O(n^2) `Vec::contains` scan over all-distinct identities.
/// Deleting the guard reaches exactly that walk:
/// **17.4 s** in a debug build, against **0.07 s** for this arm and **0.25 s**
/// for its channel-cap sibling — so an accepted arm would cost ~70x the whole
/// rest of this binary (0.22 s) on every CI run of the package. The refusal is
/// reachable in bounded time precisely BECAUSE the guard runs before that walk.
/// The accepted side is not left unpinned by that choice: the channel-cap test's
/// own accepted arm registers `65,532` topics sharing ONE schema and succeeds,
/// which is the control proving this guard does not blanket-refuse a
/// registration of that size.
#[test]
fn the_schema_id_space_is_a_fail_closed_boundary_one_smaller_than_the_channel_space() {
    const CHANNEL_CAP: usize = u16::MAX as usize + 1;
    const SCHEMA_CAP: usize = u16::MAX as usize;
    /// The reserved channels, each carrying its own distinct schema — the count
    /// `empty_topic_list_still_registers_reserved_channels` pins independently.
    const RESERVED: usize = 4;

    // Exactly fills the CHANNEL space, every user topic carrying its OWN schema
    // identity.
    let topics: Vec<TopicSchema> = (0..CHANNEL_CAP - RESERVED)
        .map(|i| TopicSchema {
            topic: format!("/t{i:05}"),
            schema_name: format!("pkg/S{i:05}"),
            schema_hash: 0x1,
            wire_fixed_size: 1,
        })
        .collect();

    let p = tmp();
    let err = match BagWriter::create(&p, BagWriterConfig::default(), &topics) {
        Err(e) => e,
        Ok(_) => panic!("a registration past the schema-id space must be refused"),
    };
    assert!(
        matches!(
            err,
            BagError::TooManySchemas { schemas, cap }
                if schemas == CHANNEL_CAP && cap == SCHEMA_CAP
        ),
        "got {err:?}"
    );
    // The channel total is exactly AT its cap, so this refusal is the SCHEMA
    // guard's and not the channel guard's spilling over.
    assert_eq!(topics.len() + RESERVED, CHANNEL_CAP);
    assert!(
        !p.exists(),
        "an over-cap registration must not leave a stray file"
    );
    std::fs::remove_file(&p).ok();
}

#[test]
fn unknown_topic_and_channel_write_is_rejected() {
    let p = tmp();
    let mut w = BagWriter::create(&p, BagWriterConfig::default(), &[topic("/a")]).unwrap();
    let payload = [0u8; 1];
    let e1 = w
        .write_chunk(|c| c.write_message("/nope", 0, 1, 1, &[&payload[..]]))
        .expect_err("unknown topic");
    assert!(matches!(e1, BagError::UnknownTarget { .. }), "got {e1:?}");
    // Out-of-range channel id (there are 5 channels: /a + 4 reserved -> ids 0..4).
    let e2 = w
        .write_chunk(|c| c.write_message(99u16, 0, 1, 1, &[&payload[..]]))
        .expect_err("unknown channel id");
    assert!(matches!(e2, BagError::UnknownTarget { .. }), "got {e2:?}");
    drop(w.finalize());
    std::fs::remove_file(&p).ok();
}

#[test]
fn wrong_length_scheduler_trace_payload_is_rejected() {
    let p = tmp();
    let mut w = BagWriter::create(&p, BagWriterConfig::default(), &[topic("/a")]).unwrap();
    let tid = w.scheduler_trace_channel_id();
    // 10 bytes on the scheduler-trace channel (must be exactly 40).
    let junk = [0u8; 10];
    let err = w
        .write_chunk(|c| c.write_message(tid, 0, 1, 1, &[&junk[..]]))
        .expect_err("bad trace length");
    assert!(
        matches!(err, BagError::BadTraceRecordLen { actual: 10, .. }),
        "got {err:?}"
    );
    drop(w.finalize());
    std::fs::remove_file(&p).ok();
}

/// The producer-label channel takes EXACTLY one fixed-size record.
///
/// A hand-built `FrameLabel` written through the ordinary `write_message` entry
/// point (the one the recorder uses — it splices ring spans, so a gate on a
/// convenience wrapper would be reachable by no production caller) lands
/// byte-for-byte and decodes back to what was encoded; anything of another
/// length is REFUSED rather than written as a label no reader can frame.
///
/// The ACCEPTING half is in this body deliberately: without it, a blanket
/// "refuse everything on this channel" variant passes the refusal assertion.
#[test]
fn producer_label_length_is_gated_and_a_valid_record_round_trips() {
    let p = tmp();
    let mut w = BagWriter::create(&p, BagWriterConfig::default(), &[topic("/a")]).unwrap();
    let pid = w.frame_producers_channel_id();

    // Refused: one byte short of a record.
    let short = [0u8; PRODUCER_RECORD_SIZE - 1];
    let err = w
        .write_chunk(|c| c.write_message(pid, 0, 1, 1, &[&short[..]]))
        .expect_err("a short producer label must be refused");
    assert!(
        matches!(
            err,
            BagError::BadProducerRecordLen {
                actual,
                expected,
                ..
            } if actual == PRODUCER_RECORD_SIZE - 1 && expected == PRODUCER_RECORD_SIZE
        ),
        "got {err:?}"
    );

    // Refused: two records in one message (the framing a reader relies on is
    // one record per message, so a doubled payload is not "more labels").
    let doubled = [0u8; PRODUCER_RECORD_SIZE * 2];
    let err = w
        .write_chunk(|c| c.write_message(pid, 0, 1, 1, &[&doubled[..]]))
        .expect_err("a doubled producer label must be refused");
    assert!(
        matches!(err, BagError::BadProducerRecordLen { .. }),
        "got {err:?}"
    );

    // Accepted: a real record, written as the recorder would.
    let label = ProducerRecord {
        attribution: ProducerAttribution::FrameLabel { frame_index: 3 },
        channel_id: 0,
        publisher_id: 0x1122_3344_5566_7788_99AA_BBCC_DDEE_FF00,
    };
    let bytes = label.encode();
    w.write_chunk(|c| c.write_message(pid, 0, 500, 500, &[&bytes[..]]))
        .unwrap();
    w.finalize().unwrap();

    // Oracle: exactly one message, on the producer-label topic, decoding back
    // to the record that was encoded.
    let data = std::fs::read(&p).unwrap();
    let msgs: Vec<mcap::Message> = mcap::MessageStream::new(&data)
        .unwrap()
        .collect::<Result<_, _>>()
        .unwrap();
    assert_eq!(msgs.len(), 1, "only the accepted label is in the bag");
    assert_eq!(msgs[0].channel.topic, cerulion_bag::FRAME_PRODUCERS_TOPIC);
    assert_eq!(&msgs[0].data[..], &bytes[..], "payload bytes");
    assert_eq!(ProducerRecord::decode(&msgs[0].data).unwrap(), label);
    std::fs::remove_file(&p).ok();
}

#[test]
fn error_in_scope_discards_pending_chunk_and_writer_stays_usable() {
    let p = tmp();
    let mut w = BagWriter::create(&p, BagWriterConfig::default(), &[topic("/a")]).unwrap();

    // Scope 1: two good messages recorded, then the closure errors → the WHOLE
    // pending chunk is discarded (fail-loud: write_chunk returns the error and
    // the messages are NOT in the bag).
    let doomed = [0xAAu8; 8];
    let err = w
        .write_chunk(|c| {
            c.write_message("/a", 0, 100, 100, &[&doomed[..]])?;
            c.write_message("/a", 1, 101, 101, &[&doomed[..]])?;
            c.write_message("/nope", 2, 102, 102, &[&doomed[..]]) // errors
        })
        .expect_err("closure error must surface");
    assert!(matches!(err, BagError::UnknownTarget { .. }), "got {err:?}");

    // Scope 2: the writer is still usable; only THIS scope's message lands.
    let kept = [0xBBu8; 8];
    w.write_chunk(|c| c.write_message("/a", 7, 200, 200, &[&kept[..]]))
        .unwrap();
    w.finalize().unwrap();

    // Oracle: exactly ONE message (seq 7), and Statistics agree — the
    // discarded messages must not inflate counts or time bounds.
    let data = std::fs::read(&p).unwrap();
    let msgs: Vec<mcap::Message> = mcap::MessageStream::new(&data)
        .unwrap()
        .collect::<Result<_, _>>()
        .unwrap();
    assert_eq!(msgs.len(), 1);
    assert_eq!(msgs[0].sequence, 7);
    assert_eq!(&msgs[0].data[..], &kept[..]);
    let stats = mcap::Summary::read(&data)
        .unwrap()
        .unwrap()
        .stats
        .expect("stats");
    assert_eq!(stats.message_count, 1, "discarded messages must not count");
    assert_eq!(
        stats.message_start_time, 200,
        "discarded times must not fold"
    );
    assert_eq!(stats.message_end_time, 200);
    assert_eq!(stats.chunk_count, 1);
    std::fs::remove_file(&p).ok();
}

#[test]
fn panic_in_scope_discards_pending_chunk_and_writer_survives() {
    let p = tmp();
    let mut w = BagWriter::create(&p, BagWriterConfig::default(), &[topic("/a")]).unwrap();

    // A closure panic unwinds through write_chunk; ChunkScope's drop guard
    // must discard the pending chunk (no stale payload pointer survives).
    let doomed = [0xEEu8; 8];
    let unwound = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let _ = w.write_chunk(|c| {
            c.write_message("/a", 0, 100, 100, &[&doomed[..]])?;
            panic!("simulated recorder panic mid-scope");
        });
    }));
    assert!(unwound.is_err(), "the panic must propagate");

    // The writer is still usable after catch_unwind, and the doomed message is
    // gone: a later flush must not writev the discarded (dangling-in-spirit)
    // plan entries.
    let kept = [0x42u8; 8];
    w.write_chunk(|c| c.write_message("/a", 9, 300, 300, &[&kept[..]]))
        .unwrap();
    w.finalize().unwrap();

    let data = std::fs::read(&p).unwrap();
    let msgs: Vec<mcap::Message> = mcap::MessageStream::new(&data)
        .unwrap()
        .collect::<Result<_, _>>()
        .unwrap();
    assert_eq!(msgs.len(), 1, "only the post-panic message is in the bag");
    assert_eq!(msgs[0].sequence, 9);
    assert_eq!(&msgs[0].data[..], &kept[..]);
    let stats = mcap::Summary::read(&data)
        .unwrap()
        .unwrap()
        .stats
        .expect("stats");
    assert_eq!(stats.message_count, 1);
    assert_eq!(stats.message_start_time, 300);
    std::fs::remove_file(&p).ok();
}

#[test]
fn trace_between_scopes_joins_the_next_chunk() {
    let p = tmp();
    let rec = cerulion_bag::TraceRingRecord {
        step: 3,
        fire_time_ns: 50,
        duration_ns: 1,
        node_idx: 0,
        global_level: 0,
        record_type: 1, // RECORD_TYPE_FIRE
        reserved: 0,
    };
    let mut w = BagWriter::create(&p, BagWriterConfig::default(), &[topic("/a")]).unwrap();
    // Trace record written OUTSIDE any scope (arena-copied, no lifetime
    // hazard) — it must join the next chunk's plan and flush with it.
    w.write_scheduler_trace(0, 50, 50, &rec).unwrap();
    let payload = [0x5Au8; 8];
    w.write_chunk(|c| c.write_message("/a", 1, 60, 60, &[&payload[..]]))
        .unwrap();
    w.finalize().unwrap();

    let data = std::fs::read(&p).unwrap();
    let summary = mcap::Summary::read(&data).unwrap().unwrap();
    assert_eq!(
        summary.chunk_indexes.len(),
        1,
        "the between-scopes trace record and the scope's message share ONE chunk"
    );
    let msgs: Vec<mcap::Message> = mcap::MessageStream::new(&data)
        .unwrap()
        .collect::<Result<_, _>>()
        .unwrap();
    assert_eq!(msgs.len(), 2);
    let trace = BagReader::from_bytes(data).scheduler_trace().unwrap();
    assert_eq!(trace, vec![rec]);
    std::fs::remove_file(&p).ok();
}

#[test]
fn empty_topic_list_still_registers_reserved_channels() {
    let p = tmp();
    let w = BagWriter::create(&p, BagWriterConfig::default(), &[]).unwrap();
    // Only the FOUR reserved channels exist, in sorted-name order (ids 0..3).
    // Of the two added later, `__cerulion/state` sorts last and
    // `__cerulion/frame_producers` sorts first.
    assert_eq!(w.channel_id(cerulion_bag::FRAME_PRODUCERS_TOPIC), Some(0));
    assert_eq!(w.channel_id(NONDETERMINISM_TOPIC), Some(1));
    assert_eq!(w.channel_id(SCHEDULER_TRACE_TOPIC), Some(2));
    assert_eq!(w.channel_id(cerulion_bag::STATE_TOPIC), Some(3));
    w.finalize().unwrap();

    // The bag is valid + readable by the mcap oracle: 4 channels, 0 messages.
    let data = std::fs::read(&p).unwrap();
    let summary = mcap::Summary::read(&data).unwrap().expect("valid summary");
    assert_eq!(summary.channels.len(), 4);
    let msgs: Vec<_> = mcap::MessageStream::new(&data)
        .unwrap()
        .collect::<Result<_, _>>()
        .unwrap();
    assert!(msgs.is_empty());
    std::fs::remove_file(&p).ok();
}

#[test]
fn zero_message_finalize_is_a_valid_bag() {
    let p = tmp();
    let w = BagWriter::create(&p, BagWriterConfig::default(), &[topic("/a")]).unwrap();
    w.finalize().unwrap();

    let r = BagReader::open(&p).unwrap();
    assert_eq!(r.channels().unwrap().len(), 5); // /a + 4 reserved
    assert_eq!(r.messages().unwrap().count(), 0);
    assert!(r.scheduler_trace().unwrap().is_empty());
    std::fs::remove_file(&p).ok();
}

#[test]
fn attachment_roundtrips_on_a_message_free_bag() {
    let p = tmp();
    let blob = b"{\"seed\":1234}".to_vec();
    let mut w = BagWriter::create(&p, BagWriterConfig::default(), &[topic("/a")]).unwrap();
    w.write_attachment("env.json", "application/json", 500, 499, &blob)
        .unwrap();
    w.finalize().unwrap();

    let r = BagReader::open(&p).unwrap();
    let a = r
        .attachment("env.json")
        .unwrap()
        .expect("attachment present");
    assert_eq!(a.data, blob);
    assert_eq!(a.media_type, "application/json");
    assert_eq!(a.log_time, 500);
    assert_eq!(a.create_time, 499);
    assert!(r.attachment("missing").unwrap().is_none());
    std::fs::remove_file(&p).ok();
}

#[test]
fn descriptor_encode_decode_byte_oracle() {
    let d = SchemaDescriptor::new(0x0102_0304_0506_0708, 0x1112_1314);
    let bytes = d.encode();
    // 18-byte little-endian layout: version(2) recipe(4) hash(8) size(4).
    let expected: [u8; 18] = [
        0x01, 0x00, // version = 1
        0x03, 0x00, 0x00, 0x00, // recipe = 3
        0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, // hash LE
        0x14, 0x13, 0x12, 0x11, // size LE
    ];
    assert_eq!(bytes, expected);
    assert_eq!(SchemaDescriptor::decode(&bytes).unwrap(), d);
}

/// #4: the `write_chunk` doc promise — chunks auto-flushed BEFORE a later
/// closure error stay durable; only the pending chunk is discarded.
/// chunk_max_bytes = 100 with 60-byte message bodies (31-byte frame + 29-byte
/// payload) auto-flushes after every 2nd message: msgs 0+1 -> chunk 1,
/// msgs 2+3 -> chunk 2, msg 4 pending when the closure errors.
#[test]
fn auto_flushed_chunks_survive_a_later_scope_error() {
    let p = tmp();
    let cfg = BagWriterConfig {
        chunk_max_bytes: 100,
        ..Default::default()
    };
    let mut w = BagWriter::create(&p, cfg, &[topic("/a")]).unwrap();

    let payloads: Vec<Vec<u8>> = (0..5).map(|i| vec![i as u8; 29]).collect();
    let err = w
        .write_chunk(|c| {
            for (i, pl) in payloads.iter().enumerate() {
                c.write_message("/a", i as u32, 1000 + i as u64, 1000 + i as u64, &[&pl[..]])?;
            }
            // Two auto-flushes have happened (after msgs 1 and 3); msg 4 is
            // pending. Now fail the scope.
            Err(cerulion_bag::BagError::Malformed {
                reason: "simulated recorder failure mid-scope".into(),
            })
        })
        .expect_err("the closure error must surface");
    assert!(matches!(err, cerulion_bag::BagError::Malformed { .. }));
    w.finalize().unwrap();

    // Hand oracle: msgs 0-3 durable (two auto-flushed chunks), msg 4 gone.
    let data = std::fs::read(&p).unwrap();
    let msgs: Vec<mcap::Message> = mcap::MessageStream::new(&data)
        .unwrap()
        .collect::<Result<_, _>>()
        .unwrap();
    assert_eq!(msgs.len(), 4, "the two auto-flushed chunks are durable");
    for (i, m) in msgs.iter().enumerate() {
        assert_eq!(m.sequence, i as u32);
        assert_eq!(&m.data[..], &payloads[i][..]);
    }
    // Statistics + indexes reflect ONLY the durable chunks.
    let stats = mcap::Summary::read(&data)
        .unwrap()
        .unwrap()
        .stats
        .expect("stats");
    assert_eq!(stats.message_count, 4);
    assert_eq!(stats.chunk_count, 2);
    assert_eq!(stats.message_start_time, 1000);
    assert_eq!(
        stats.message_end_time, 1003,
        "the discarded msg 4 (ts 1004) must not fold into the time bounds"
    );
    std::fs::remove_file(&p).ok();
}

/// Two topics sharing one (schema name, hash, size)
/// identity produce exactly ONE Schema record, referenced by both Channels.
#[test]
fn bagd_schema_registry_no_duplication_for_same_schema_hash() {
    let p = tmp();
    let shared = |t: &str| TopicSchema {
        topic: t.into(),
        schema_name: "sensor_msgs/Image".into(),
        schema_hash: 0xFEED_BEEF,
        wire_fixed_size: 64,
    };
    let w = BagWriter::create(
        &p,
        BagWriterConfig::default(),
        &[shared("/cam_left"), shared("/cam_right")],
    )
    .unwrap();
    w.finalize().unwrap();

    let data = std::fs::read(&p).unwrap();
    let summary = mcap::Summary::read(&data).unwrap().unwrap();
    // 1 shared user schema + 4 reserved schemas = 5 (NOT 6).
    assert_eq!(
        summary.schemas.len(),
        5,
        "the shared schema must be registered once"
    );
    let by_topic: std::collections::HashMap<String, u16> = summary
        .channels
        .values()
        .map(|c| {
            (
                c.topic.clone(),
                c.schema.as_ref().expect("channel has a schema").id,
            )
        })
        .collect();
    assert_eq!(
        by_topic["/cam_left"], by_topic["/cam_right"],
        "both channels must reference the SAME schema id"
    );
    // And that schema decodes to the shared identity.
    let schema = summary
        .schemas
        .get(&by_topic["/cam_left"])
        .expect("shared schema present");
    assert_eq!(schema.name, "sensor_msgs/Image");
    let d = cerulion_bag::SchemaDescriptor::decode(&schema.data).unwrap();
    assert_eq!(d.schema_hash, 0xFEED_BEEF);
    assert_eq!(d.wire_fixed_size, 64);
    std::fs::remove_file(&p).ok();
}

// ===========================================================================
// Reader-side wrong-length trace payload: the locating index
// ===========================================================================

/// Build a FINALIZED, uncompressed, chunked bag via the `mcap` crate's own
/// writer (our `BagWriter` refuses a wrong-length trace payload at write time,
/// so a foreign/corrupt bag is modeled with the foreign writer) carrying the
/// given payloads on a `__cerulion/scheduler_trace` channel.
fn foreign_bag_with_trace_payloads(payloads: &[&[u8]]) -> Vec<u8> {
    let mut buf = std::io::Cursor::new(Vec::new());
    {
        let mut w = mcap::WriteOptions::new()
            .compression(None)
            .create(&mut buf)
            .unwrap();
        let chan = w
            .add_channel(0, SCHEDULER_TRACE_TOPIC, "cerulion", &Default::default())
            .unwrap();
        for (i, p) in payloads.iter().enumerate() {
            w.write_to_known_channel(
                &mcap::records::MessageHeader {
                    channel_id: chan,
                    sequence: i as u32,
                    log_time: 100 + i as u64,
                    publish_time: 100 + i as u64,
                },
                p,
            )
            .unwrap();
        }
        w.finish().unwrap();
    }
    buf.into_inner()
}

/// A 39-byte payload on the trace channel must be a loud `Malformed` naming
/// the CHANNEL and the record's file-order index — on BOTH the strict
/// (streaming) and the crash-recovery (stock-stream) paths.
#[test]
fn wrong_length_trace_payload_read_back_names_the_record_index() {
    let good = [0u8; 40];
    let bad = [0u8; 39];

    // Arm 1: the bad record is the FIRST trace record → "trace record 0".
    let r = BagReader::from_bytes(foreign_bag_with_trace_payloads(&[&bad]));
    for err in [
        r.scheduler_trace().expect_err("strict must refuse"),
        r.recover_scheduler_trace()
            .expect_err("recover must refuse"),
    ] {
        let msg = err.to_string();
        assert!(
            msg.contains("trace record 0") && msg.contains("39 bytes"),
            "error names the index + actual length: {msg}"
        );
        assert!(
            msg.contains(SCHEDULER_TRACE_TOPIC),
            "error names the channel: {msg}"
        );
    }

    // Arm 2: a valid 40-byte record FIRST, the bad one second → "trace record 1"
    // (the index is the trace-channel ordinal, proving it advances).
    let r = BagReader::from_bytes(foreign_bag_with_trace_payloads(&[&good, &bad]));
    for err in [
        r.scheduler_trace().expect_err("strict must refuse"),
        r.recover_scheduler_trace()
            .expect_err("recover must refuse"),
    ] {
        let msg = err.to_string();
        assert!(
            msg.contains("trace record 1") && msg.contains("39 bytes"),
            "the second record is ordinal 1: {msg}"
        );
    }
}

// ===========================================================================
// `register_topic` guards + the writer-wide poison latch
// ===========================================================================

/// `register_topic` refuses exactly what `create` refuses, and the
/// refusal leaves the writer untouched.
///
/// The cheap guards run in one body; the ID-SPACE half is its own test below,
/// because reaching that boundary costs a 65k-topic construction.
#[test]
fn register_topic_refuses_a_duplicate_and_a_reserved_prefix() {
    let p = tmp();
    let mut w = BagWriter::create(&p, BagWriterConfig::default(), &[topic("/a")]).unwrap();
    let before = w.channel_id("/a");
    let count_before = w.registered_topics().len();

    // (a) A reserved-prefix topic — the namespace the writer auto-registers.
    let err = w
        .register_topic(&topic("__cerulion/state"))
        .expect_err("a reserved prefix must be refused");
    assert!(
        matches!(err, BagError::ReservedTopicPrefix { ref topic } if topic == "__cerulion/state"),
        "got {err:?}"
    );

    // (b) A topic already in the CONSTRUCTION set. A second channel per NAME is
    // forbidden outright — every consumer keys by name, so two channels sharing
    // one would make "the messages on /a" unanswerable.
    let err = w
        .register_topic(&topic("/a"))
        .expect_err("a construction-set duplicate must be refused");
    assert!(
        matches!(err, BagError::DuplicateTopic { ref topic } if topic == "/a"),
        "got {err:?}"
    );

    // (c) A topic already registered LATE — the same guard over the other half
    // of the table, which a lookup that only saw the construction set would miss.
    w.register_topic(&topic("/late")).unwrap();
    let err = w
        .register_topic(&topic("/late"))
        .expect_err("a late duplicate must be refused too");
    assert!(
        matches!(err, BagError::DuplicateTopic { ref topic } if topic == "/late"),
        "got {err:?}"
    );

    // A refused registration changed NOTHING.
    assert_eq!(w.channel_id("/a"), before);
    assert_eq!(w.channel_id("__cerulion/state"), Some(4));
    assert_eq!(
        w.registered_topics().len(),
        count_before + 1,
        "only the ONE accepted registration is in the table"
    );
    w.finalize().unwrap();
    std::fs::remove_file(&p).ok();
}

/// The channel-id space is fail-closed for `register_topic` too,
/// pinned on BOTH sides — the same 16-bit `as u16` wrap hazard
/// `the_channel_id_space_is_a_fail_closed_boundary_pinned_on_both_sides` pins
/// for the constructor, one entry point over.
///
/// The construction set is deliberately ONE short of the space, so exactly one
/// late registration fits: it must succeed and take `u16::MAX` (the top of the
/// space is reachable, nothing wrapped), and the next must be refused with the
/// same two addends `validate_registration` reports, leaving the table
/// unchanged. A missing guard hands the second registration id 0, colliding
/// with the canonically-first user topic.
#[test]
fn register_topic_refuses_a_registration_past_the_channel_id_space() {
    const CAP: usize = u16::MAX as usize + 1;
    /// Pinned independently by `empty_topic_list_still_registers_reserved_channels`.
    const RESERVED: usize = 4;

    // One slot short of the cap, so the FIRST late registration exactly fills it.
    let names: Vec<String> = (0..CAP - RESERVED - 1)
        .map(|i| format!("/t{i:05}"))
        .collect();
    let topics: Vec<TopicSchema> = names.iter().map(|n| topic(n)).collect();

    let p = tmp();
    let mut w = BagWriter::create(&p, BagWriterConfig::default(), &topics)
        .expect("one short of the cap is legal");

    let id = w
        .register_topic(&topic("/fits"))
        .expect("the last free channel id must be usable");
    assert_eq!(
        id,
        u16::MAX,
        "the top of the id space is reachable — a wrap would land on 0"
    );
    assert_eq!(w.channel_id("/t00000"), Some(0), "and nothing collided");

    let err = w
        .register_topic(&topic("/overflows"))
        .expect_err("one past the space must be refused");
    assert!(
        matches!(
            err,
            BagError::TooManyChannels { topics: t, reserved, total, cap }
                if t == CAP - RESERVED && reserved == RESERVED && total == CAP + 1 && cap == CAP
        ),
        "got {err:?}"
    );
    assert_eq!(
        w.channel_id("/overflows"),
        None,
        "a refused registration must not enter the table"
    );
    assert_eq!(
        w.channel_id("/fits"),
        Some(u16::MAX),
        "and must not disturb it"
    );
    w.finalize().unwrap();
    std::fs::remove_file(&p).ok();
}

/// The SCHEMA-id space is `register_topic`'s SECOND, one-smaller
/// fail-closed boundary — and it is charged only for a NEW identity.
///
/// Three steps, in an order chosen so each guard is ISOLATED rather than
/// restated:
///
/// 1. Fill to exactly `USABLE_SCHEMA_IDS` distinct identities, which leaves the
///    CHANNEL table one slot short of its own (one larger) cap.
/// 2. A NEW identity is refused `TooManySchemas`. A channel slot is still free,
///    so only the schema guard can be refusing it — the same discriminator the
///    constructor's sibling test uses.
/// 3. An EXISTING identity SUCCEEDS at that same moment, taking `u16::MAX`.
///    That is the load-bearing half: a guard charged per REGISTRATION rather
///    than per new identity would refuse it, and a bag could then never add a
///    channel for a type it already carries. It also consumes the last channel
///    slot, so the next registration hits `TooManyChannels` — the two caps
///    demonstrated to be different quantities against different bounds.
///
/// Reached by REGISTERING rather than constructing, which is why it is
/// affordable: `register_topic` dedups through a `HashMap` (O(1) per call),
/// while the constructor's dedup is an O(n^2) `Vec::contains` scan. MEASURED,
/// because the constructor route is what the sibling `create` test declined to
/// pay for: **207 ms** here against **16.55 s** for a 65,535-distinct-schema
/// construction — an 80x saving for a STRONGER oracle (the construction route
/// cannot show step 3 at all).
#[test]
fn register_topic_charges_the_schema_id_space_only_for_a_new_identity() {
    use cerulion_bag::test_sink::ScriptedSink;

    const CHANNEL_CAP: usize = u16::MAX as usize + 1;
    const SCHEMA_CAP: usize = u16::MAX as usize;
    /// Pinned independently by `empty_topic_list_still_registers_reserved_channels`.
    const RESERVED: usize = 4;

    // Start from the reserved channels alone (4 channels, 4 identities) and
    // register distinct identities until the schema space is exactly full.
    let mut w =
        BagWriter::with_sink(ScriptedSink::single_call(), BagWriterConfig::default(), &[]).unwrap();
    for i in 0..SCHEMA_CAP - RESERVED {
        w.register_topic(&TopicSchema {
            topic: format!("/t{i:05}"),
            schema_name: format!("pkg/S{i:05}"),
            schema_hash: 0x1,
            wire_fixed_size: 1,
        })
        .expect("every registration up to the schema cap must be accepted");
    }
    // Structural precondition: the schema space is FULL and the channel space
    // has exactly one slot left, so the two guards are separable here.
    assert_eq!(w.registered_topics().len(), SCHEMA_CAP - RESERVED);
    assert_eq!(
        w.channel_id(&format!("/t{:05}", SCHEMA_CAP - RESERVED - 1)),
        Some((SCHEMA_CAP - 1) as u16),
        "the channel table holds SCHEMA_CAP entries, one short of the channel cap"
    );

    // (2) A NEW identity is refused — by the SCHEMA guard, with a channel slot
    //     still free.
    let err = w
        .register_topic(&TopicSchema {
            topic: "/new_identity".into(),
            schema_name: "pkg/BRAND_NEW".into(),
            schema_hash: 0x9,
            wire_fixed_size: 1,
        })
        .expect_err("a new identity past the schema space must be refused");
    assert!(
        matches!(
            err,
            BagError::TooManySchemas { schemas, cap }
                if schemas == CHANNEL_CAP && cap == SCHEMA_CAP
        ),
        "got {err:?}"
    );
    assert_eq!(
        w.channel_id("/new_identity"),
        None,
        "a refused registration must not enter the channel table"
    );

    // (3) An EXISTING identity still fits — the guard is per new identity, not
    //     per registration — and takes the top of the channel space.
    let shared = w
        .register_topic(&TopicSchema {
            topic: "/shares_an_identity".into(),
            schema_name: "pkg/S00000".into(),
            schema_hash: 0x1,
            wire_fixed_size: 1,
        })
        .expect("a topic reusing an existing schema identity costs no schema id");
    assert_eq!(
        shared,
        u16::MAX,
        "and it takes the last channel id — the top of that space is reachable too"
    );

    // Which now exhausts the CHANNEL space: a different cap, a different
    // quantity, a different error.
    let err = w
        .register_topic(&TopicSchema {
            topic: "/one_too_many".into(),
            schema_name: "pkg/S00000".into(),
            schema_hash: 0x1,
            wire_fixed_size: 1,
        })
        .expect_err("the channel space is now full");
    assert!(
        matches!(err, BagError::TooManyChannels { cap, .. } if cap == CHANNEL_CAP),
        "got {err:?}"
    );
}

/// A POISONED writer refuses every later call.
///
/// The poison is produced the way production would: a cold-path `write_bytes`
/// that accepts part of its buffer and then fails, leaving the sink's byte
/// stream longer than the writer's `pos`. There is no retry, so each of the
/// EIGHT guarded entry points is asserted SEPARATELY — they are eight call
/// sites, and a check missing from any one of them lets that path write at an
/// offset that does not exist. The set is held complete by
/// `every_public_writer_entry_point_is_classified_for_the_poison_latch`, which
/// fails until a new `pub fn` is classified.
///
/// Three of the eight would return `Err` even unguarded, because they end in
/// (or delegate to) a guarded `flush_chunk`/`write_attachment` — so for those
/// the oracle is not the return value but the SIDE EFFECT the guard prevents:
/// `write_chunk` must not invoke the caller's closure, and
/// `write_schema_catalog` is driven with an EMPTY catalog, whose early return,
/// placed before the guard, would answer `Ok(())` on a dead writer. `write_scheduler_trace` — the one
/// most easily left unguarded, being the only entry point that appends to the arena
/// without going through `write_message` — is pinned on both the refusal AND
/// the arena staying empty.
#[test]
fn a_poisoned_writer_refuses_every_later_call() {
    use cerulion_bag::test_sink::{BytesAction, ScriptedSink};

    // The prelude rides the first `write_bytes`; the attachment's is the one we
    // tear in half.
    let sink = ScriptedSink::single_call().with_byte_script([
        BytesAction::Full,
        BytesAction::ShortThenFail {
            n: 3,
            errno: libc::EIO,
        },
    ]);
    let mut w = BagWriter::with_sink(sink, BagWriterConfig::default(), &[topic("/a")]).unwrap();

    let durable_prefix = w.sink().bytes().len();
    w.write_attachment("graph.yaml", "application/yaml", 0, 0, b"g: 1\n")
        .expect_err("the scripted cold-path failure must surface");
    // The fixture really did make partial progress — without this the arm would
    // be proving nothing about a torn file.
    assert_eq!(w.sink().bytes().len(), durable_prefix + 3);

    let payload = [0u8; 1];
    assert!(
        matches!(
            w.write_message("/a", 0, 1, 1, &[&payload[..]]),
            Err(BagError::Poisoned { .. })
        ),
        "write_message must refuse"
    );
    assert!(
        matches!(
            w.write_attachment("env.json", "application/json", 0, 0, b"{}"),
            Err(BagError::Poisoned { .. })
        ),
        "write_attachment must refuse"
    );
    assert!(
        matches!(
            w.register_topic(&topic("/late")),
            Err(BagError::Poisoned { .. })
        ),
        "register_topic must refuse"
    );
    assert!(
        matches!(w.flush_chunk(), Err(BagError::Poisoned { .. })),
        "flush_chunk must refuse"
    );

    // The SIXTH entry point, and the easiest to leave unguarded: it is the only
    // path that appends to the chunk arena without going through
    // `write_message`. Unguarded it returns Ok and grows the arena by a frame
    // the writer can never emit — so the arena is asserted too, which is the
    // part a return-value-only oracle would miss.
    let trace = cerulion_bag::TraceRingRecord {
        step: 1,
        fire_time_ns: 10,
        duration_ns: 5,
        node_idx: 0,
        global_level: 0,
        record_type: 1,
        reserved: 0,
    };
    assert!(
        matches!(
            w.write_scheduler_trace(0, 10, 10, &trace),
            Err(BagError::Poisoned { .. })
        ),
        "write_scheduler_trace must refuse"
    );
    assert_eq!(
        w.open_chunk_messages(),
        0,
        "a refused trace record must not reach the arena"
    );

    // `write_chunk` ends in the guarded `flush_chunk`, so its return value is
    // Err either way. What its OWN check buys is that the caller's closure is
    // never invoked on a dead writer.
    let mut closure_ran = false;
    assert!(
        matches!(
            w.write_chunk(|_| {
                closure_ran = true;
                Ok(())
            }),
            Err(BagError::Poisoned { .. })
        ),
        "write_chunk must refuse"
    );
    assert!(
        !closure_ran,
        "a poisoned writer must not run the caller's chunk closure"
    );

    // `write_schema_catalog` delegates to the guarded `write_attachment` — but
    // ONLY for a non-empty catalog. An EMPTY one returned Ok(()) before its own
    // check, which is a success answer from a writer that will never emit
    // another byte, so the empty catalog is the discriminating input.
    assert!(
        matches!(
            w.write_schema_catalog(&cerulion_bag::BagSchemaCatalog::empty()),
            Err(BagError::Poisoned { .. })
        ),
        "write_schema_catalog must refuse even an empty catalog"
    );

    // The error names the operation that poisoned the writer, so one log line
    // says which shape tore the file.
    let err = w.write_message("/a", 1, 2, 2, &[&payload[..]]).unwrap_err();
    let msg = err.to_string();
    assert!(
        msg.contains("poisoned") && msg.contains("framed-record write"),
        "the refusal must name the operation that poisoned the writer: {msg}"
    );

    // None of the refusals wrote anything: the stream is still the durable
    // prefix plus the torn fragment.
    let captured = w.sink().bytes().to_vec();
    assert_eq!(captured.len(), durable_prefix + 3);
    assert!(
        matches!(w.finalize(), Err(BagError::Poisoned { .. })),
        "finalize must refuse — a summary over a file whose offsets are unknown is worse than none"
    );

    // And what IS on disk is exactly the durable prefix the writer wrote before
    // the tear: a crash-recoverable bag, not a corrupt one.
    let r = BagReader::from_bytes(captured);
    let (msgs, completeness) = r.recover_messages().unwrap();
    assert!(msgs.is_empty());
    assert!(!completeness.is_finalized());
}

/// Every `pub fn` in `writer.rs`, with its poison-latch classification.
///
/// `Guarded` = calls `check_poisoned()?` as its FIRST statement, and has its own
/// arm in `a_poisoned_writer_refuses_every_later_call`. Everything else states
/// WHY it needs no guard. Duplicates are intentional: `create`,
/// `write_message` and `write_scheduler_trace` each appear on two impls, and
/// keeping both rows means a third copy on a new impl fails this test.
const WRITER_PUBLIC_SURFACE: &[(&str, &str)] = &[
    // --- Guarded: reaches the sink, refuses once poisoned (8) ---------------
    ("register_topic", "Guarded"),
    ("write_message", "Guarded — BagWriter"),
    ("write_scheduler_trace", "Guarded — BagWriter"),
    ("write_chunk", "Guarded"),
    ("flush_chunk", "Guarded"),
    ("write_attachment", "Guarded"),
    ("write_schema_catalog", "Guarded"),
    ("finalize", "Guarded"),
    // --- Cannot reach the sink (4) ------------------------------------------
    (
        "discard_pending_chunk",
        "clears pending state only — never appends, never writes; ChunkScope's \
         Drop calls it where no error can be returned",
    ),
    (
        "write_message",
        "ChunkScope — its single statement delegates to the guarded BagWriter twin",
    ),
    (
        "write_scheduler_trace",
        "ChunkScope — its single statement delegates to the guarded BagWriter twin",
    ),
    (
        "fault_inject_flush_failures_for_test",
        "test seam behind #[cfg] — sets two counters, touches no sink/arena/table",
    ),
    // --- Constructors: no writer exists yet to poison (4) --------------------
    ("create", "BagWriter constructor"),
    ("with_sink", "BagWriter constructor"),
    ("create", "FileSink constructor — a sink, not a writer"),
    ("from_file", "FileSink constructor — a sink, not a writer"),
    // --- Read-only (11) ------------------------------------------------------
    ("channel_id", "read-only"),
    ("registered_topics", "read-only"),
    ("scheduler_trace_channel_id", "read-only"),
    ("state_channel_id", "read-only"),
    ("frame_producers_channel_id", "read-only"),
    ("bytes_written", "read-only"),
    ("messages_persisted", "read-only"),
    ("chunks_flushed", "read-only"),
    ("open_chunk_bytes", "read-only"),
    ("open_chunk_messages", "read-only"),
    ("sink", "read-only borrow (test seam)"),
    // --- Crate-internal (1) ---------------------------------------------------
    (
        "flush_iovecs",
        "pub(crate) writev loop — reached only from the guarded flush_chunk",
    ),
];

/// The name of the function a line declares, if that line declares a function
/// with ANY non-private visibility.
///
/// Parses the general shape rather than a list of spellings —
/// `Visibility? Qualifier* fn NAME` — where `Visibility` is `pub`, `pub(crate)`,
/// `pub(super)`, `pub(self)` or `pub(in path)`, and the qualifiers are `unsafe`,
/// `async`, `const` and `extern "ABI"` in any order. A visibility is mandatory:
/// a private `fn` is unreachable from outside the crate and cannot be a new
/// entry point onto the writer.
///
/// Returns `None` for anything that is not a function declaration, including
/// `pub struct` / `pub const` / `pub use` / `pub mod`, an identifier that merely
/// starts with `pub`, and any commented-out line (a comment's trimmed text
/// starts with `/`, which is not a visibility).
fn public_fn_name(line: &str) -> Option<String> {
    let mut s = line.trim_start().strip_prefix("pub")?;

    if let Some(rest) = s.strip_prefix('(') {
        // A visibility restriction holds a path, never a nested paren, so the
        // first `)` closes it: `pub(crate)`, `pub(super)`, `pub(self)`,
        // `pub(in crate::writer)`.
        s = &rest[rest.find(')')? + 1..];
    } else if !s.starts_with(char::is_whitespace) {
        // `pub` must be a whole token — `pubfish` is an identifier.
        return None;
    }

    // Qualifiers, in any order, until `fn`.
    loop {
        s = s.trim_start();
        let Some(rest) = ["unsafe", "async", "const", "extern"].iter().find_map(|q| {
            s.strip_prefix(q)
                .filter(|r| r.starts_with(|c: char| c.is_whitespace() || c == '"'))
        }) else {
            break;
        };
        s = rest.trim_start();
        // `extern "C"` — step over the ABI string.
        if let Some(r) = s.strip_prefix('"') {
            s = &r[r.find('"')? + 1..];
        }
    }

    let rest = s.strip_prefix("fn")?;
    if !rest.starts_with(char::is_whitespace) {
        return None;
    }
    let name: String = rest
        .trim_start()
        .chars()
        .take_while(|c| c.is_alphanumeric() || *c == '_')
        .collect();
    (!name.is_empty()).then_some(name)
}

/// Every non-private function name declared in `src`, in source order.
fn scan_public_fns(src: &str) -> Vec<String> {
    src.lines().filter_map(public_fn_name).collect()
}

/// The inventory walk's matcher recognises every visibility and
/// qualifier form, not the four spellings `writer.rs` happens to use.
///
/// Driven against SYNTHETIC sources rather than by editing `writer.rs`, because
/// the property is about forms the file does not currently contain — which is
/// exactly where a spelling-list matcher fails: a `pub(crate) unsafe fn` added
/// tomorrow would reach the sink and dodge the enumeration, reintroducing the
/// stale-list hole one level down.
#[test]
fn the_matcher_recognises_every_visibility_and_qualifier_form() {
    // (line, the name it declares)
    let positives: &[(&str, &str)] = &[
        ("pub fn a() {}", "a"),
        ("pub(crate) fn b() {}", "b"),
        // THE form the four-spelling matcher missed.
        (
            "pub(crate) unsafe fn mutant_entry(&mut self) {}",
            "mutant_entry",
        ),
        ("pub(super) fn c() {}", "c"),
        ("pub(self) fn d() {}", "d"),
        ("pub(in crate::writer) fn e() {}", "e"),
        ("pub unsafe fn f() {}", "f"),
        ("pub async fn g() {}", "g"),
        ("pub const fn h() {}", "h"),
        ("pub unsafe extern \"C\" fn i() {}", "i"),
        ("pub async unsafe fn j() {}", "j"),
        ("    pub fn indented() {}", "indented"),
        ("pub fn generic<'a, T: Copy>(x: T) {}", "generic"),
        ("pub(crate)fn tight() {}", "tight"),
    ];
    for (line, want) in positives {
        assert_eq!(
            public_fn_name(line).as_deref(),
            Some(*want),
            "must recognise `{line}`"
        );
    }

    let negatives: &[&str] = &[
        "fn private() {}",
        "pub struct S;",
        "pub const CAP: u16 = 1;",
        "pub use crate::x;",
        "pub mod m;",
        "pub type T = u8;",
        "// pub fn commented() {}",
        "/// pub fn doc_commented() {}",
        "pubfish fn nope() {}",
        "let pub_thing = 1;",
        "pub fn",
        "",
    ];
    for line in negatives {
        assert_eq!(
            public_fn_name(line),
            None,
            "must NOT recognise `{line}` as a function declaration"
        );
    }

    // And it composes over a whole source, in order.
    let src = "pub struct W;\n\
               impl W {\n\
                   pub(crate) unsafe fn mutant_entry(&mut self) {}\n\
                   fn private(&self) {}\n\
                   pub fn ok(&self) {}\n\
               }\n";
    assert_eq!(scan_public_fns(src), vec!["mutant_entry", "ok"]);
}

/// No `pub fn` in `writer.rs` is unclassified for the poison latch.
///
/// The behavioural test above proves the eight guarded entry points refuse. It
/// cannot prove the set is COMPLETE, and completeness is where a latch goes wrong:
/// `write_scheduler_trace` is easy to leave unguarded because it is the sixth member of
/// a set a reader would count as five, and a hand-maintained prose list is
/// exactly the artefact that goes stale. So the set is derived from the source
/// on every run, and a new entry point fails here until someone writes down
/// which side of the line it falls on.
///
/// # Why a line walk rather than a comment stripper
///
/// The match is on lines whose TRIMMED text starts with a `pub … fn ` form, so
/// a `//`-comment can never satisfy it. A block comment could — `writer.rs` has
/// none today, and this test passing is that fact's only proof — but the
/// failure direction is safe: a commented-out signature ADDS a phantom name and
/// fails LOUDLY. It cannot HIDE one, because real code is never inside a
/// comment. That asymmetry is what makes the cheap walk sound here, where
/// `bag_clock_free_test`'s "no token anywhere" property genuinely needed the
/// depth-tracked stripper.
///
/// What the walk must NOT do is enumerate a few spellings: a matcher that knew
/// only `pub fn` / `pub(crate) fn` / `pub unsafe fn` / `pub async fn` would let a
/// `pub(crate) unsafe fn` or a `pub(in crate::…) fn` sink-reaching entry point
/// walk straight past the enumeration — the very stale-list failure this test
/// exists to prevent, reintroduced one level down. [`public_fn_name`] therefore
/// parses the general form (any visibility, any order of qualifiers), and
/// `the_matcher_recognises_every_visibility_and_qualifier_form` drives it
/// against synthetic sources so the coverage does not depend on which spellings
/// `writer.rs` happens to contain today.
#[test]
fn every_public_writer_entry_point_is_classified_for_the_poison_latch() {
    let src = std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/writer.rs"),
    )
    .expect("writer.rs must be readable — the walk cannot pass by finding nothing");

    let found_ref = scan_public_fns(&src);
    let mut found = found_ref;

    // Anti-tautology: a walk that matched nothing (a renamed file, a broken
    // matcher) would "agree" with an empty inventory and pass forever.
    assert!(
        found.len() >= 20,
        "the walk found only {} signatures — it is not reading writer.rs properly",
        found.len()
    );
    for known in ["write_scheduler_trace", "flush_chunk", "finalize"] {
        assert!(
            found.iter().any(|f| f == known),
            "the walk must find the known entry point `{known}`"
        );
    }

    let mut declared: Vec<String> = WRITER_PUBLIC_SURFACE
        .iter()
        .map(|(n, _)| (*n).to_string())
        .collect();
    found.sort();
    declared.sort();

    if found != declared {
        let extra: Vec<_> = found.iter().filter(|f| !declared.contains(f)).collect();
        let missing: Vec<_> = declared.iter().filter(|d| !found.contains(d)).collect();
        panic!(
            "writer.rs's public surface no longer matches WRITER_PUBLIC_SURFACE.\n\
             In the source but NOT classified: {extra:?}\n\
             Classified but NOT in the source: {missing:?}\n\
             If you added an entry point that can reach the sink, give it \
             `self.check_poisoned()?` as its FIRST statement, add an arm to \
             `a_poisoned_writer_refuses_every_later_call`, and add a `Guarded` \
             row here. Otherwise add a row saying why it needs no guard."
        );
    }
}
