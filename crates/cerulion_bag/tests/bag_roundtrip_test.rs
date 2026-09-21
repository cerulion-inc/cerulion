// SPDX-License-Identifier: AGPL-3.0-only
//! Round-trip: write a bag with hand-built payloads, then read it back BOTH via
//! the `mcap` crate directly (the independent oracle) AND via our `BagReader`,
//! asserting channels / schemas / message bytes / attachment bytes / the typed
//! scheduler-trace decode against hand-built expected values.

use std::path::PathBuf;

use cerulion_bag::{
    BagReader, BagWriter, BagWriterConfig, SchemaDescriptor, TopicSchema, FRAME_PRODUCERS_TOPIC,
    NONDETERMINISM_TOPIC, SCHEDULER_TRACE_TOPIC, SCHEMA_ENCODING,
};
use cerulion_core::trace_ring::{TraceRingRecord, RECORD_TYPE_DEPARTURE, RECORD_TYPE_FIRE};

fn tempdir() -> PathBuf {
    static C: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let n = C.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let name = format!(
        "cerulion_bag_rt_{}_{}_{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos(),
        n
    );
    let p = std::env::temp_dir().join(name);
    std::fs::create_dir_all(&p).unwrap();
    p
}

fn topics() -> Vec<TopicSchema> {
    vec![
        TopicSchema {
            topic: "/camera/image".into(),
            schema_name: "sensor_msgs/Image".into(),
            schema_hash: 0x1111_2222_3333_4444,
            wire_fixed_size: 64,
        },
        TopicSchema {
            topic: "/imu".into(),
            schema_name: "sensor_msgs/Imu".into(),
            schema_hash: 0x5555_6666_7777_8888,
            wire_fixed_size: 320,
        },
        TopicSchema {
            topic: "/pose".into(),
            schema_name: "geometry_msgs/Pose".into(),
            schema_hash: 0x9999_AAAA_BBBB_CCCC,
            wire_fixed_size: 56,
        },
    ]
}

#[test]
fn roundtrip_via_mcap_oracle_and_bagreader() {
    let dir = tempdir();
    let path = dir.join("rt.mcap");

    // Payload buffers MUST outlive finalize() (zero-copy pointer contract).
    let img0: Vec<u8> = vec![0xAA; 16];
    let imu_a: Vec<u8> = vec![0x01, 0x02, 0x03];
    let imu_b: Vec<u8> = vec![0x04, 0x05];
    let pose0: Vec<u8> = vec![0xBB; 8];
    let img1: Vec<u8> = vec![0xCC; 16];
    let graph_yaml = b"nodes:\n  - camera\n  - detector\n".to_vec();
    let env_json = br#"{"RUST_LOG":"info"}"#.to_vec();

    let fire = TraceRingRecord {
        step: 1,
        fire_time_ns: 1004,
        duration_ns: 500,
        node_idx: 2,
        global_level: 1,
        record_type: RECORD_TYPE_FIRE,
        reserved: 0,
    };
    let departure = TraceRingRecord {
        step: 2,
        fire_time_ns: 1005,
        duration_ns: 0,
        node_idx: 3,
        global_level: 2,
        record_type: RECORD_TYPE_DEPARTURE,
        reserved: 0,
    };

    let mut w = BagWriter::create(&path, BagWriterConfig::default(), &topics()).unwrap();

    // Deterministic channel ids: user topics sorted by name, reserved after.
    assert_eq!(w.channel_id("/camera/image"), Some(0));
    assert_eq!(w.channel_id("/imu"), Some(1));
    assert_eq!(w.channel_id("/pose"), Some(2));
    // Reserved ids follow the SORTED reserved names, so its
    // `frame_producers` takes the lowest of them and its siblings shift up.
    assert_eq!(w.channel_id(FRAME_PRODUCERS_TOPIC), Some(3));
    assert_eq!(w.channel_id(NONDETERMINISM_TOPIC), Some(4));
    assert_eq!(w.channel_id(SCHEDULER_TRACE_TOPIC), Some(5));
    assert_eq!(w.scheduler_trace_channel_id(), 5);
    assert_eq!(w.frame_producers_channel_id(), 3);

    // All borrowed payloads go through ONE chunk scope (`'buf` = the buffers
    // above, which outlive the whole write_chunk call). Trace records
    // interleave in the same chunk via the scope.
    w.write_chunk(|c| {
        c.write_message("/camera/image", 0, 1000, 1000, &[&img0[..]])?;
        // A two-part payload exercises multi-iovec assembly for one message.
        c.write_message("/imu", 0, 1001, 1001, &[&imu_a[..], &imu_b[..]])?;
        c.write_message("/pose", 0, 1002, 1002, &[&pose0[..]])?;
        c.write_message("/camera/image", 1, 1003, 1003, &[&img1[..]])?;
        c.write_scheduler_trace(1, 1004, 1004, &fire)?;
        c.write_scheduler_trace(2, 1005, 1005, &departure)?;
        Ok(())
    })
    .unwrap();
    w.write_attachment("graph.yaml", "application/yaml", 2000, 1999, &graph_yaml)
        .unwrap();
    w.write_attachment("env.json", "application/json", 2001, 2000, &env_json)
        .unwrap();
    w.finalize().unwrap();

    // Hand oracle for the 6 messages (topic, seq, log_time, publish_time, data).
    let expected: Vec<(&str, u32, u64, u64, Vec<u8>)> = vec![
        ("/camera/image", 0, 1000, 1000, img0.clone()),
        (
            "/imu",
            0,
            1001,
            1001,
            [imu_a.clone(), imu_b.clone()].concat(),
        ),
        ("/pose", 0, 1002, 1002, pose0.clone()),
        ("/camera/image", 1, 1003, 1003, img1.clone()),
        (
            SCHEDULER_TRACE_TOPIC,
            1,
            1004,
            1004,
            fire.as_bytes().to_vec(),
        ),
        (
            SCHEDULER_TRACE_TOPIC,
            2,
            1005,
            1005,
            departure.as_bytes().to_vec(),
        ),
    ];

    // -------- (A) read via the mcap crate directly (independent oracle) --------
    let data = std::fs::read(&path).unwrap();
    let summary = mcap::Summary::read(&data)
        .unwrap()
        .expect("bag has a summary");
    assert_eq!(summary.channels.len(), 7, "3 user + 4 reserved channels");
    assert_eq!(summary.schemas.len(), 7, "7 distinct schemas");
    // Every schema uses the `cerulion` encoding.
    for s in summary.schemas.values() {
        assert_eq!(s.encoding, SCHEMA_ENCODING);
        assert_eq!(s.data.len(), cerulion_bag::DESCRIPTOR_LEN);
    }
    // Stats agree with what we wrote.
    let stats = summary.stats.as_ref().expect("stats");
    assert_eq!(stats.message_count, 6);
    assert_eq!(stats.channel_count, 7);
    assert_eq!(stats.schema_count, 7);
    assert_eq!(stats.attachment_count, 2);

    let msgs: Vec<mcap::Message> = mcap::MessageStream::new(&data)
        .unwrap()
        .collect::<Result<_, _>>()
        .expect("all messages parse under the mcap oracle");
    assert_eq!(msgs.len(), expected.len());
    for (got, exp) in msgs.iter().zip(&expected) {
        assert_eq!(got.channel.topic, exp.0, "topic");
        assert_eq!(got.sequence, exp.1, "sequence");
        assert_eq!(got.log_time, exp.2, "log_time");
        assert_eq!(got.publish_time, exp.3, "publish_time");
        assert_eq!(&got.data[..], &exp.4[..], "payload bytes");
    }

    // Attachment bytes via the mcap oracle.
    assert_eq!(summary.attachment_indexes.len(), 2);
    let mut att: std::collections::HashMap<String, Vec<u8>> = std::collections::HashMap::new();
    for idx in &summary.attachment_indexes {
        let a = mcap::read::attachment(&data, idx).unwrap();
        att.insert(a.name.clone(), a.data.into_owned());
    }
    assert_eq!(att.get("graph.yaml").unwrap(), &graph_yaml);
    assert_eq!(att.get("env.json").unwrap(), &env_json);

    // -------- (B) read via our BagReader --------
    let r = BagReader::open(&path).unwrap();

    let channels = r.channels().unwrap();
    assert_eq!(channels.len(), 7);
    let by_topic: std::collections::HashMap<&str, &cerulion_bag::BagChannel> =
        channels.iter().map(|c| (c.topic.as_str(), c)).collect();
    // The /camera/image descriptor decodes to the hash + size we registered.
    let cam = by_topic["/camera/image"];
    let d: SchemaDescriptor = cam.descriptor.unwrap();
    assert_eq!(d.schema_hash, 0x1111_2222_3333_4444);
    assert_eq!(d.wire_fixed_size, 64);
    assert_eq!(cam.schema_name, "sensor_msgs/Image");
    // The scheduler-trace descriptor: hash 0, fixed size 40.
    let st = by_topic[SCHEDULER_TRACE_TOPIC];
    let sd = st.descriptor.unwrap();
    assert_eq!(sd.schema_hash, 0);
    assert_eq!(sd.wire_fixed_size, 40);
    // The THIRD reserved channel is registered in EVERY bag,
    // whether or not the run checkpointed, and carries the state record's own
    // fixed size — asserted here beside its sibling because both come out of
    // the same unconditional registration.
    let state = by_topic[cerulion_bag::STATE_TOPIC];
    let std_ = state.descriptor.unwrap();
    assert_eq!(std_.schema_hash, 0);
    assert_eq!(std_.wire_fixed_size, cerulion_bag::STATE_RECORD_SIZE);
    assert_eq!(state.schema_name, cerulion_bag::STATE_SCHEMA);

    // Messages via BagReader match the same oracle.
    let got: Vec<_> = r.messages().unwrap().collect::<Result<_, _>>().unwrap();
    assert_eq!(got.len(), expected.len());
    for (g, exp) in got.iter().zip(&expected) {
        assert_eq!(g.topic, exp.0);
        assert_eq!(g.sequence, exp.1);
        assert_eq!(g.log_time, exp.2);
        assert_eq!(g.publish_time, exp.3);
        assert_eq!(g.data, exp.4);
    }

    // Typed scheduler-trace decode == the exact records we wrote.
    let trace = r.scheduler_trace().unwrap();
    assert_eq!(trace, vec![fire, departure]);

    // Attachment round-trip via BagReader.
    assert_eq!(
        r.attachment("graph.yaml").unwrap().unwrap().data,
        graph_yaml
    );
    let env = r.attachment("env.json").unwrap().unwrap();
    assert_eq!(env.data, env_json);
    assert_eq!(env.media_type, "application/json");

    std::fs::remove_dir_all(&dir).ok();
}

/// Arena-regrowth regression pin: the chunk arena (`chunk_frames`) is a growing
/// `Vec<u8>`; plan entries must reference it by OFFSET, never by pointer — a
/// pointer captured before a reallocation would dangle and the flushed bytes
/// would be garbage. Force many reallocations within ONE chunk (hundreds of
/// arena-copied records + borrowed payloads) and assert every read-back payload
/// equals the hand oracle.
#[test]
fn arena_regrowth_within_one_chunk_flushes_correct_bytes() {
    let dir = tempdir();
    let path = dir.join("regrow.mcap");

    // 400 trace records × (31-byte frame + 40-byte arena copy) ≈ 28 KiB of
    // arena — the Vec reallocates many times from its empty start. Interleave
    // borrowed-payload messages so both entry kinds cross reallocation points.
    const N: usize = 400;
    let borrowed: Vec<Vec<u8>> = (0..N).map(|i| vec![(i % 251) as u8; 24]).collect();

    let mut w = BagWriter::create(&path, BagWriterConfig::default(), &topics()).unwrap();
    w.write_chunk(|c| {
        for (i, b) in borrowed.iter().enumerate() {
            let rec = TraceRingRecord {
                step: i as u64,
                fire_time_ns: 10_000 + i as u64,
                duration_ns: 7,
                node_idx: (i % 5) as u32,
                global_level: 1,
                record_type: RECORD_TYPE_FIRE,
                reserved: 0,
            };
            c.write_scheduler_trace(i as u32, 10_000 + i as u64, 10_000 + i as u64, &rec)?;
            c.write_message(
                "/imu",
                i as u32,
                20_000 + i as u64,
                20_000 + i as u64,
                &[&b[..]],
            )?;
        }
        Ok(())
    })
    .unwrap();
    w.finalize().unwrap();

    // Single chunk (default 4 MiB cap never crossed), all payloads == oracle.
    let data = std::fs::read(&path).unwrap();
    let summary = mcap::Summary::read(&data).unwrap().unwrap();
    assert_eq!(
        summary.chunk_indexes.len(),
        1,
        "one chunk, many reallocations"
    );

    let r = BagReader::from_bytes(data);
    let trace = r.scheduler_trace().unwrap();
    assert_eq!(trace.len(), N);
    for (i, t) in trace.iter().enumerate() {
        assert_eq!(t.step, i as u64, "trace record {i} intact after regrowth");
        assert_eq!(t.fire_time_ns, 10_000 + i as u64);
    }
    let imu: Vec<_> = r
        .messages()
        .unwrap()
        .collect::<Result<Vec<_>, _>>()
        .unwrap()
        .into_iter()
        .filter(|m| m.topic == "/imu")
        .collect();
    assert_eq!(imu.len(), N);
    for (i, m) in imu.iter().enumerate() {
        assert_eq!(m.data, borrowed[i], "borrowed payload {i} intact");
    }

    std::fs::remove_dir_all(&dir).ok();
}

/// Memory fix: `user_message_index` (the zero-copy replay foundation)
/// must produce, per USER topic, spans whose resolved bytes are BYTE-IDENTICAL
/// to the `mcap` crate's `MessageStream` (the independent oracle) — in FILE
/// order — and must EXCLUDE the reserved `__cerulion/*` channels. A small chunk
/// cap forces MANY chunks so the multi-chunk framing walk + cross-chunk topic
/// interleave are exercised.
#[test]
fn user_message_index_frames_byte_match_mcap_oracle_across_chunks() {
    let dir = tempdir();
    let path = dir.join("index.mcap");

    // Tiny chunk cap ⇒ almost every message flushes its own chunk (many chunks).
    let cfg = BagWriterConfig {
        chunk_max_bytes: 64,
        ..Default::default()
    };
    const N: usize = 50;
    // Distinct per-frame payloads so a mis-ordered/mis-sized span is caught.
    let cam: Vec<Vec<u8>> = (0..N).map(|i| vec![(i % 253) as u8; 8 + i % 5]).collect();
    let imu: Vec<Vec<u8>> = (0..N).map(|i| vec![(i % 251) as u8; 3 + i % 7]).collect();

    let mut w = BagWriter::create(&path, cfg, &topics()).unwrap();
    w.write_chunk(|c| {
        for i in 0..N {
            // Interleave user topics AND a reserved trace record per iteration —
            // the trace channel must NOT appear in the index.
            c.write_message(
                "/camera/image",
                i as u32,
                1000 + i as u64,
                1000 + i as u64,
                &[&cam[i][..]],
            )?;
            let rec = TraceRingRecord {
                step: i as u64,
                fire_time_ns: 5000 + i as u64,
                duration_ns: 1,
                node_idx: 0,
                global_level: 0,
                record_type: RECORD_TYPE_FIRE,
                reserved: 0,
            };
            c.write_scheduler_trace(i as u32, 5000 + i as u64, 5000 + i as u64, &rec)?;
            c.write_message(
                "/imu",
                i as u32,
                2000 + i as u64,
                2000 + i as u64,
                &[&imu[i][..]],
            )?;
        }
        Ok(())
    })
    .unwrap();
    w.finalize().unwrap();

    // Oracle: every USER frame via the mcap crate, in file order, per topic.
    let data = std::fs::read(&path).unwrap();
    assert!(
        mcap::Summary::read(&data)
            .unwrap()
            .unwrap()
            .chunk_indexes
            .len()
            > 1,
        "the tiny cap must produce multiple chunks"
    );
    let mut oracle: std::collections::HashMap<String, Vec<Vec<u8>>> =
        std::collections::HashMap::new();
    for m in mcap::MessageStream::new(&data).unwrap() {
        let m = m.unwrap();
        if m.channel.topic.starts_with("__cerulion/") {
            continue;
        }
        oracle
            .entry(m.channel.topic.clone())
            .or_default()
            .push(m.data.into_owned());
    }

    let r = BagReader::open(&path).unwrap();
    let index = r.user_message_index().unwrap();

    // Reserved channel is absent; both user topics present.
    assert!(!index.keys().any(|k| k.starts_with("__cerulion/")));
    assert_eq!(index.len(), oracle.len(), "same set of user topics");
    for (topic, spans) in &index {
        let exp = &oracle[topic];
        assert_eq!(spans.len(), exp.len(), "frame count for {topic}");
        for (i, span) in spans.iter().enumerate() {
            assert_eq!(r.frame(span), &exp[i][..], "topic {topic} frame {i} bytes");
        }
    }

    std::fs::remove_dir_all(&dir).ok();
}

/// Write a small valid finalized bag and return its raw bytes plus the byte
/// offset of the footer's `summary_start` field. Footer tail layout (fixed):
/// `[0x02][len=20 u64][summary_start u64][summary_offset_start u64]
/// [summary_crc u32][MAGIC 8]` — so `summary_start` sits at
/// `len - 8(magic) - 4(crc) - 8(summary_offset_start) - 8(summary_start)
/// = len - 28`.
fn valid_bag_bytes_and_summary_start_offset() -> (Vec<u8>, usize) {
    let dir = tempdir();
    let path = dir.join("crafted.mcap");
    let payload = [0xEEu8; 16];
    let mut w = BagWriter::create(&path, BagWriterConfig::default(), &topics()).unwrap();
    w.write_chunk(|c| {
        c.write_message("/imu", 0, 1000, 1000, &[&payload[..]])?;
        c.write_message("/imu", 1, 1001, 1001, &[&payload[..]])?;
        Ok(())
    })
    .unwrap();
    w.finalize().unwrap();
    let bytes = std::fs::read(&path).unwrap();
    std::fs::remove_dir_all(&dir).ok();
    let off = bytes.len() - 28;
    // Self-check the offset arithmetic: the field currently decodes to a sane
    // in-range summary_start (a wrong offset would read garbage here and the
    // corruption below would be meaningless).
    let current = u64::from_le_bytes(bytes[off..off + 8].try_into().unwrap()) as usize;
    assert!(
        current >= 8 && current < bytes.len(),
        "summary_start offset arithmetic drifted: decoded {current} from a {}-byte bag",
        bytes.len()
    );
    (bytes, off)
}

/// A footer `summary_start` corrupted into
/// `[0, MAGIC.len())` must be a LOUD `Malformed` from `user_message_index` —
/// never a slice-index panic. The finalization gate validates only the footer
/// FINGERPRINT (frame + magics), never the summary_start VALUE, so values
/// 1..=7 reach the index step; without the range guard, `&bytes[8..data_end]` panics
/// ("slice index starts at 8 but ends at 4").
#[test]
fn corrupt_footer_summary_start_below_magic_is_malformed_not_panic() {
    let (bytes, off) = valid_bag_bytes_and_summary_start_offset();
    // 0 (the one low value a zero-only guard covers), the panic reproducer 4, and both
    // boundary values of the corrupt window [1, 7].
    for bad in [0u64, 1, 4, 7] {
        let mut corrupt = bytes.clone();
        corrupt[off..off + 8].copy_from_slice(&bad.to_le_bytes());
        let r = BagReader::from_bytes(corrupt);
        let err = r
            .user_message_index()
            .expect_err("summary_start={bad} must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains("summary_start"),
            "error must name the corrupt field (summary_start={bad}): {msg}"
        );
    }
    // Control: MAGIC.len() itself (an empty data section) is in-range — the
    // guard is exclusive below 8, not below 9.
    let mut empty_section = bytes.clone();
    empty_section[off..off + 8].copy_from_slice(&8u64.to_le_bytes());
    let r = BagReader::from_bytes(empty_section);
    let index = r.user_message_index().expect("empty data section is legal");
    assert!(index.is_empty(), "an empty data section indexes no topics");
}

/// Residue bytes after the last complete record in the
/// data section (too short to hold a record frame) must be a LOUD `Malformed`
/// naming the residue count — never silently dropped (an under-indexed topic
/// would mislabel the replay diff verdict). Crafted by pulling `summary_start`
/// back 10 bytes: the closing DataEnd record (13 bytes) truncates to 3 residue
/// bytes at the section tail.
#[test]
fn residue_bytes_in_data_section_are_malformed_not_silently_dropped() {
    let (bytes, off) = valid_bag_bytes_and_summary_start_offset();
    let real_start = u64::from_le_bytes(bytes[off..off + 8].try_into().unwrap());
    let mut corrupt = bytes.clone();
    corrupt[off..off + 8].copy_from_slice(&(real_start - 10).to_le_bytes());
    let r = BagReader::from_bytes(corrupt);
    let err = r
        .user_message_index()
        .expect_err("a truncated data section must be rejected");
    let msg = err.to_string();
    assert!(
        msg.contains("residue") && msg.contains("data section"),
        "error must name the residue + section: {msg}"
    );
}

/// The streaming `trace_records` pinned against TWO independent
/// oracles on a FINALIZED MULTI-CHUNK bag with trace records in the 2nd+ chunk
/// interleaved with user messages:
///
/// 1. `recover_scheduler_trace().0` — the stock `mcap::MessageStream` path (a
///    completely different decode pipeline, the same way
///    `user_message_index` is pinned against the stock stream), and
/// 2. the HAND-BUILT record vector that was written.
///
/// `scheduler_trace()` (the collect of `trace_records()`) is asserted equal
/// too, closing the three-way equality.
#[test]
fn trace_records_stream_matches_recover_oracle_across_chunks() {
    let dir = tempdir();
    let path = dir.join("multi_chunk_trace.mcap");

    // Hand-built trace records with distinct field values (the written oracle).
    let expected: Vec<TraceRingRecord> = (0..7u64)
        .map(|i| TraceRingRecord {
            step: i,
            fire_time_ns: 1_000 + i,
            duration_ns: 10 * i,
            node_idx: (i % 3) as u32,
            global_level: (i % 2) as u32,
            record_type: RECORD_TYPE_FIRE,
            reserved: 0,
        })
        .collect();

    let user_payload = [0xABu8; 24];
    let mut w = BagWriter::create(&path, BagWriterConfig::default(), &topics()).unwrap();
    // Chunk 1: user messages + the first 2 trace records.
    w.write_chunk(|c| {
        c.write_message("/imu", 0, 10, 10, &[&user_payload[..]])?;
        c.write_scheduler_trace(0, 1_000, 1_000, &expected[0])?;
        c.write_message("/camera/image", 0, 11, 11, &[&user_payload[..]])?;
        c.write_scheduler_trace(1, 1_001, 1_001, &expected[1])
    })
    .unwrap();
    // Chunk 2: trace records INTERLEAVED with user messages (the 2nd+ chunk
    // arm — a regression here would under-decode without a loud error).
    w.write_chunk(|c| {
        c.write_scheduler_trace(2, 1_002, 1_002, &expected[2])?;
        c.write_message("/imu", 1, 12, 12, &[&user_payload[..]])?;
        c.write_scheduler_trace(3, 1_003, 1_003, &expected[3])?;
        c.write_scheduler_trace(4, 1_004, 1_004, &expected[4])?;
        c.write_message("/pose", 0, 13, 13, &[&user_payload[..]])
    })
    .unwrap();
    // Chunk 3: trace-only tail.
    w.write_chunk(|c| {
        c.write_scheduler_trace(5, 1_005, 1_005, &expected[5])?;
        c.write_scheduler_trace(6, 1_006, 1_006, &expected[6])
    })
    .unwrap();
    w.finalize().unwrap();

    let r = BagReader::open(&path).unwrap();
    // The streaming iterator, collected.
    let streamed: Vec<TraceRingRecord> = r
        .trace_records()
        .unwrap()
        .collect::<Result<_, _>>()
        .unwrap();
    // Oracle 1: the hand-built written records.
    assert_eq!(streamed, expected, "streamed == written (hand oracle)");
    // Oracle 2: the stock-mcap-stream recover path (independent pipeline).
    let (recovered, completeness) = r.recover_scheduler_trace().unwrap();
    assert!(completeness.is_finalized());
    assert_eq!(streamed, recovered, "streamed == stock-stream recover");
    // And the materializing collector is exactly the collect of the stream.
    assert_eq!(r.scheduler_trace().unwrap(), streamed);

    std::fs::remove_dir_all(&dir).ok();
}
