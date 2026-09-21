// SPDX-License-Identifier: AGPL-3.0-only
//! Crash recovery: a truncated bag (torn tail / no summary) recovers exactly the
//! complete chunks' messages via the `mcap` crate (verdict: NATIVE, no fallback
//! needed), and a corrupted chunk body is DETECTED (never silently returned).

use std::path::PathBuf;

use cerulion_bag::{BagReader, BagWriter, BagWriterConfig, TopicSchema};

/// A unique scratch path — process id + a monotonic counter, no clock (see
/// `bag_late_channel_test::tmp`).
fn tmp() -> PathBuf {
    static C: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let n = C.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let p = std::env::temp_dir().join(format!("cerulion_bag_crash_{}_{}", std::process::id(), n));
    // A REUSED pid meeting a leftover artifact from an interrupted earlier run
    // must not leak into this run's assertions (a rejection-path test never
    // truncates the path it refuses to create), so the path is cleared at
    // issuance — deterministic, and still clock-free.
    let _ = std::fs::remove_file(&p);
    p
}

const N: usize = 6;

/// Write `N` messages, one per chunk (chunk_max_bytes = 1), then finalize.
/// Returns (file bytes, chunk (start_offset, length) pairs sorted ascending).
fn write_multichunk(path: &std::path::Path) -> (Vec<u8>, Vec<(u64, u64)>) {
    let payloads: Vec<Vec<u8>> = (0..N).map(|i| vec![i as u8; 8]).collect();
    let cfg = BagWriterConfig {
        chunk_max_bytes: 1,
        ..Default::default()
    };
    let mut w = BagWriter::create(
        path,
        cfg,
        &[TopicSchema {
            topic: "/a".into(),
            schema_name: "std_msgs/UInt8".into(),
            schema_hash: 0x1,
            wire_fixed_size: 8,
        }],
    )
    .unwrap();
    // chunk_max_bytes = 1 → the auto-flush inside the scope emits one chunk
    // per message (sound: all payloads outlive the scope).
    w.write_chunk(|c| {
        for (i, p) in payloads.iter().enumerate() {
            c.write_message("/a", i as u32, 1000 + i as u64, 1000 + i as u64, &[&p[..]])?;
        }
        Ok(())
    })
    .unwrap();
    w.finalize().unwrap();

    let data = std::fs::read(path).unwrap();
    let summary = mcap::Summary::read(&data).unwrap().unwrap();
    let mut chunks: Vec<(u64, u64)> = summary
        .chunk_indexes
        .iter()
        .map(|c| (c.chunk_start_offset, c.chunk_length))
        .collect();
    chunks.sort();
    assert_eq!(chunks.len(), N, "one chunk per message");
    (data, chunks)
}

fn recover(bytes: &[u8]) -> (Vec<u32>, cerulion_bag::BagCompleteness) {
    let r = BagReader::from_bytes(bytes.to_vec());
    let (msgs, completeness) = r.recover_messages().unwrap();
    (msgs.iter().map(|m| m.sequence).collect(), completeness)
}

#[test]
fn truncation_recovers_exactly_the_complete_chunks_and_reports_how_it_ended() {
    use cerulion_bag::BagCompleteness as C;
    let path = tmp();
    let (data, chunks) = write_multichunk(&path);

    // Control: the UNtruncated bag is Finalized (all messages, no tear).
    let (all, completeness) = recover(&data);
    assert_eq!(all, (0..N as u32).collect::<Vec<_>>());
    assert!(
        completeness.is_finalized(),
        "the finalized bag must report Finalized; got {completeness:?}"
    );

    // (a) MID-final-chunk: cut inside the last chunk's body -> the last chunk
    //     is torn, messages 0..N-2 recover, and the tear IS reported (a
    //     mid-record cut can never look Finalized or boundary-clean).
    let (last_start, _last_len) = chunks[N - 1];
    let mid_final = &data[..(last_start as usize + 20)];
    let (seqs, completeness) = recover(mid_final);
    assert_eq!(
        seqs,
        (0..(N as u32 - 1)).collect::<Vec<_>>(),
        "mid-final-chunk truncation recovers all but the torn last message"
    );
    assert!(
        matches!(completeness, C::TornTail(_)),
        "a mid-chunk cut must report TornTail; got {completeness:?}"
    );

    // (b) EXACTLY at a chunk boundary: cut at the start of chunk 3 -> chunks
    //     0,1,2 recover (3 messages) and the ending is the DISTINGUISHING arm:
    //     the stream ends cleanly (no terminal error), so only the missing
    //     finalization epilogue separates this crash from a finalized bag.
    let (chunk3_start, _) = chunks[3];
    let at_boundary = &data[..chunk3_start as usize];
    let (seqs, completeness) = recover(at_boundary);
    assert_eq!(
        seqs,
        vec![0, 1, 2],
        "cut at a chunk boundary recovers exactly the complete chunks before it"
    );
    assert!(
        matches!(completeness, C::TruncatedAtChunkBoundary),
        "a boundary cut must report TruncatedAtChunkBoundary, NOT Finalized; got {completeness:?}"
    );
    assert!(!completeness.is_finalized());

    // (c) INSIDE the summary: the data section is intact, only the summary is
    //     torn -> ALL N messages recover, and the bag still must NOT read as
    //     Finalized (the cut lands mid-record -> TornTail).
    let summary_start = mcap::read::footer(&data).unwrap().summary_start;
    let in_summary = &data[..(summary_start as usize + 4)];
    let (seqs, completeness) = recover(in_summary);
    assert_eq!(
        seqs,
        (0..N as u32).collect::<Vec<_>>(),
        "a torn summary loses no data-section messages"
    );
    assert!(
        matches!(completeness, C::TornTail(_)),
        "a crash during finalize must not read as Finalized; got {completeness:?}"
    );

    std::fs::remove_file(&path).ok();
}

/// The fixture the mid-file-channel torn-tail arms share.
///
/// `/early` in chunk 1; a mid-file `register_topic("/late")` between the
/// chunks; `/late` seq 0 in chunk 2 and `/late` seq 1 in chunk 3. THREE chunks,
/// because the interesting cut is one that tears a chunk while a COMPLETE
/// late-channel message already stands before it — impossible to place with the
/// two-chunk shape this started as.
///
/// Returns the finalized bytes, the id `register_topic` handed back, the write
/// position immediately after the registration, and the chunk start offsets
/// ascending.
fn write_late_channel_bag(path: &std::path::Path) -> (Vec<u8>, u16, usize, Vec<usize>) {
    use cerulion_bag::{BagWriterConfig, TopicSchema};

    let early = TopicSchema {
        topic: "/early".into(),
        schema_name: "sensor_msgs/Imu".into(),
        schema_hash: 0xAAAA,
        wire_fixed_size: 8,
    };
    let late = TopicSchema {
        topic: "/late".into(),
        schema_name: "sensor_msgs/Image".into(),
        schema_hash: 0xBBBB,
        wire_fixed_size: 8,
    };

    let payload = [0x9Au8; 8];
    let mut w = BagWriter::create(path, BagWriterConfig::default(), &[early]).unwrap();
    w.write_message("/early", 0, 100, 100, &[&payload[..]])
        .unwrap();
    w.flush_chunk().unwrap();
    // The write position the moment BEFORE the registration, and the moment
    // after: the record-boundary cut lands between them, i.e. after the Channel
    // record and before the chunk that carries its first message.
    let before_registration = w.bytes_written() as usize;
    let late_id = w.register_topic(&late).unwrap();
    let after_registration = w.bytes_written() as usize;
    w.write_message("/late", 0, 200, 200, &[&payload[..]])
        .unwrap();
    w.flush_chunk().unwrap();
    // The third chunk: what a cut can tear while chunk 2's late message stays
    // whole.
    w.write_message("/late", 1, 300, 300, &[&payload[..]])
        .unwrap();
    w.flush_chunk().unwrap();
    w.finalize().unwrap();

    let data = std::fs::read(path).unwrap();
    let summary = mcap::Summary::read(&data).unwrap().unwrap();
    let mut chunks: Vec<usize> = summary
        .chunk_indexes
        .iter()
        .map(|c| c.chunk_start_offset as usize)
        .collect();
    chunks.sort_unstable();
    assert_eq!(
        chunks.len(),
        3,
        "structural precondition: the fixture must produce three chunks"
    );
    assert!(
        before_registration < after_registration && after_registration <= chunks[1],
        "structural precondition: the registration sits between chunk 1 and chunk 2 \
         ({before_registration} < {after_registration} <= {})",
        chunks[1]
    );

    (data, late_id, after_registration, chunks)
}

/// A COMPLETE message on a mid-file channel survives a tear in a
/// LATER chunk.
///
/// Neither sibling cut can see this. `a_late_channel_survives_a_torn_tail`'s
/// mid-chunk cut lands in the FIRST late chunk, so no late message is ever
/// complete there and its oracle only names `/early`; its other cut removes the
/// closing magic, which tears no chunk at all and leaves every chunk complete.
/// The intersection — a complete late-channel message that PRECEDES a torn
/// chunk — is covered by neither, so a recovery bug that drops it would pass them.
///
/// Truncating the recovered set to its
/// first message on the tear path leaves `a_late_channel_survives_a_torn_tail`
/// green and fails this arm, which is exactly the hole it covers.
#[test]
fn a_complete_late_message_survives_a_torn_later_chunk() {
    use cerulion_bag::BagCompleteness as C;

    let path = tmp();
    let (data, late_id, _after_registration, chunks) = write_late_channel_bag(&path);

    // Inside the THIRD chunk: chunks 1 and 2 are complete, chunk 3 is torn.
    let cut = &data[..chunks[2] + 20];
    let r = BagReader::from_bytes(cut.to_vec());
    let (msgs, completeness) = r.recover_messages().unwrap();

    assert!(
        matches!(completeness, C::TornTail(_)),
        "the cut lands mid-chunk, so completeness must report the tear; got {completeness:?}"
    );
    assert_eq!(
        msgs.iter()
            .map(|m| (m.topic.as_str(), m.sequence))
            .collect::<Vec<_>>(),
        vec![("/early", 0u32), ("/late", 0u32)],
        "both complete chunks recover — the torn third contributes nothing, and the complete \
         late-channel message standing before it is not collateral"
    );
    let late = msgs
        .iter()
        .find(|m| m.topic == "/late")
        .expect("asserted present above");
    assert_eq!(
        late.channel_id, late_id,
        "and it resolves on the id the mid-file registration returned"
    );

    std::fs::remove_file(&path).ok();
}

/// A topic registered MID-FILE survives a torn tail, and the cut
/// either side of it says the right thing.
///
/// The registration is a TOP-LEVEL record, so it is durable the moment
/// `register_topic` returns and cannot be lost with a discarded or torn chunk.
/// That is exactly what the in-arena placement would break, and it is why this
/// arm cuts in two places rather than one:
///
/// - **(a) inside the FIRST chunk after the registration**: the cut lands in
///   that chunk's header, so no late message is complete and only `/early`
///   recovers. Cutting further in — past every chunk, at the closing magic —
///   then recovers BOTH late messages on the late TOPIC and CHANNEL ID; the
///   in-arena placement loses the Channel record with its chunk, so they cannot
///   resolve a topic at all.
/// - **(b) between the Channel record and its first chunk**: no message carries
///   the late topic (none was ever flushed), the ending is clean, and every
///   recovered topic is a prelude one.
///
/// Neither cut can see a complete late message that PRECEDES a torn chunk —
/// (a) tears the only chunk that could hold one, and the closing-magic cut
/// tears no chunk at all. That intersection is
/// [`a_complete_late_message_survives_a_torn_later_chunk`].
///
/// BOTH cuts additionally assert `channels()` is `Err`. That is deliberate:
/// `BagReader::channels()` reads the SUMMARY, and a killed recorder's bag has
/// none — `Summary::read` fails `BadMagic` before any `summary_start == 0`
/// fallback can run. Asserting it keeps that fallback from quietly becoming
/// load-bearing here, and it is why the recovery oracle is `recover_messages`.
#[test]
fn a_late_channel_survives_a_torn_tail() {
    use cerulion_bag::BagCompleteness as C;

    let path = tmp();
    let (data, late_id, after_registration, chunks) = write_late_channel_bag(&path);
    let first_late_chunk = chunks[1];

    // --- (a) cut INSIDE the FIRST chunk after the registration ---
    // Deliberately the first late chunk, not the last: this arm's oracle is
    // that NO late message is complete, which is what makes the in-arena placement
    // (the registration lost with its chunk) visible. The complete-late-message
    // case is the separate arm below.
    let cut_a = &data[..first_late_chunk + 20];
    let r = BagReader::from_bytes(cut_a.to_vec());
    let (msgs, completeness) = r.recover_messages().unwrap();
    assert!(
        matches!(completeness, C::TornTail(_)),
        "a mid-chunk cut must report TornTail; got {completeness:?}"
    );
    assert!(
        r.channels().is_err(),
        "a killed recorder's bag has no summary, so channels() must fail rather than serve a \
         fallback nothing tested"
    );
    // The late chunk is torn, so its message is gone — but the prelude message
    // survives, which is what proves the cut recovered anything at all.
    assert_eq!(
        msgs.iter().map(|m| m.topic.as_str()).collect::<Vec<_>>(),
        vec!["/early"],
        "the torn chunk's message is lost; the earlier one is not"
    );

    // The registration itself is DURABLE even in this torn bag: cutting further
    // in, at the very end, recovers the late message ON the late channel.
    let r = BagReader::from_bytes(data[..data.len() - 1].to_vec());
    let (msgs, completeness) = r.recover_messages().unwrap();
    assert!(
        !completeness.is_finalized(),
        "a bag missing its closing magic is not Finalized; got {completeness:?}"
    );
    let late_msgs: Vec<_> = msgs.iter().filter(|m| m.topic == "/late").collect();
    assert_eq!(
        late_msgs.len(),
        2,
        "both late-channel messages must recover, resolved through the TOP-LEVEL Channel record \
         written before their chunks"
    );
    assert!(
        late_msgs.iter().all(|m| m.channel_id == late_id),
        "and on the id the registration returned"
    );

    // --- (b) cut BETWEEN the Channel record and its first chunk ---
    let cut_b = &data[..after_registration];
    let r = BagReader::from_bytes(cut_b.to_vec());
    let (msgs, completeness) = r.recover_messages().unwrap();
    assert!(
        matches!(completeness, C::TruncatedAtChunkBoundary),
        "a cut at a RECORD boundary — here, right after a channel registration — must report a \
         clean ending; got {completeness:?}"
    );
    assert!(r.channels().is_err(), "still no summary to read");
    assert!(
        !msgs.iter().any(|m| m.topic == "/late"),
        "no message on the late channel was ever flushed, so none may be recovered"
    );
    assert_eq!(
        msgs.iter().map(|m| m.topic.as_str()).collect::<Vec<_>>(),
        vec!["/early"],
        "every recovered topic is a prelude topic"
    );

    std::fs::remove_file(&path).ok();
}

#[test]
fn corrupt_chunk_body_is_detected_not_silently_returned() {
    let path = tmp();
    let (mut data, chunks) = write_multichunk(&path);

    // Flip one byte well inside chunk 2's BODY (start + opcode/reclen 9 + chunk
    // header 40 + 15). The chunk's uncompressed_crc must now mismatch.
    let (start, _len) = chunks[2];
    let pos = start as usize + 9 + 40 + 15;
    data[pos] ^= 0xFF;

    // Strict read: the corruption surfaces as an error (chunk CRC mismatch),
    // never a silently-returned garbage message.
    let r = BagReader::from_bytes(data.clone());
    let strict: Result<Vec<_>, _> = r.messages().unwrap().collect();
    assert!(
        strict.is_err(),
        "a corrupt chunk must make a strict read fail"
    );

    // Lenient recovery also REPORTS the corruption as the TornTail ending —
    // even though the file still carries a (now-lying) finalization epilogue,
    // the tear takes precedence.
    let (_recovered, completeness) = r.recover_messages().unwrap();
    let err = match completeness {
        cerulion_bag::BagCompleteness::TornTail(e) => e,
        other => panic!("corruption must be reported as TornTail, got {other:?}"),
    };
    let msg = err.to_string();
    assert!(
        msg.contains("Chunk CRC") || msg.to_lowercase().contains("crc"),
        "terminal error should name the CRC mismatch; got: {msg}"
    );

    std::fs::remove_file(&path).ok();
}

/// #3: the crash-tolerant scheduler-trace path — trace records from COMPLETE
/// chunks are readable after a recorder crash, with the truncation reported
/// (the strict `scheduler_trace()` would refuse the whole bag).
#[test]
fn recover_scheduler_trace_reads_complete_chunks_of_a_truncated_bag() {
    use cerulion_bag::BagCompleteness as C;
    use cerulion_core::trace_ring::{TraceRingRecord, RECORD_TYPE_FIRE};

    let path = tmp();
    let recs: Vec<TraceRingRecord> = (0..4)
        .map(|i| TraceRingRecord {
            step: i as u64,
            fire_time_ns: 1000 + i as u64,
            duration_ns: 5,
            node_idx: i as u32,
            global_level: 0,
            record_type: RECORD_TYPE_FIRE,
            reserved: 0,
        })
        .collect();

    // chunk_max_bytes = 1 -> one chunk per trace record.
    let cfg = BagWriterConfig {
        chunk_max_bytes: 1,
        ..Default::default()
    };
    let mut w = BagWriter::create(&path, cfg, &[]).unwrap();
    for (i, rec) in recs.iter().enumerate() {
        w.write_scheduler_trace(i as u32, 1000 + i as u64, 1000 + i as u64, rec)
            .unwrap();
    }
    w.finalize().unwrap();

    let data = std::fs::read(&path).unwrap();
    let summary = mcap::Summary::read(&data).unwrap().unwrap();
    let mut starts: Vec<u64> = summary
        .chunk_indexes
        .iter()
        .map(|c| c.chunk_start_offset)
        .collect();
    starts.sort();
    assert_eq!(starts.len(), 4);

    // Truncate at the start of chunk 3 (a boundary crash): records 0..=2
    // recover; the strict path refuses the torn bag entirely.
    let truncated = data[..starts[3] as usize].to_vec();
    let r = BagReader::from_bytes(truncated);
    assert!(
        r.scheduler_trace().is_err(),
        "the strict trace read must refuse a truncated bag"
    );
    let (trace, completeness) = r.recover_scheduler_trace().unwrap();
    assert_eq!(trace, recs[..3], "complete chunks' trace records recover");
    assert!(
        matches!(completeness, C::TruncatedAtChunkBoundary),
        "the truncation must be reported; got {completeness:?}"
    );

    // Control: the intact bag recovers everything and reads Finalized.
    let r = BagReader::from_bytes(data);
    let (trace, completeness) = r.recover_scheduler_trace().unwrap();
    assert_eq!(trace, recs);
    assert!(completeness.is_finalized());

    std::fs::remove_file(&path).ok();
}

/// Epilogue-truncation classification, committed as tests
/// (tests are the detection layer). From
/// ONE realistic finalized bag (user messages + BOTH reserved channels
/// populated + an attachment, so the epilogue offsets are realistic), cut the
/// tail at structurally-computed offsets and assert the ONE binding invariant
/// for every cut: completeness is NEVER `Finalized` (the catastrophic
/// misclassification) and all complete-chunk messages are still returned.
#[test]
fn epilogue_truncation_is_never_classified_finalized() {
    use cerulion_bag::BagCompleteness as C;
    use cerulion_bag::NONDETERMINISM_TOPIC;
    use cerulion_core::trace_ring::{TraceRingRecord, RECORD_TYPE_FIRE};

    let path = tmp();
    let mut w = BagWriter::create(
        &path,
        BagWriterConfig::default(),
        &[TopicSchema {
            topic: "/a".into(),
            schema_name: "std_msgs/UInt8".into(),
            schema_hash: 0x1,
            wire_fixed_size: 8,
        }],
    )
    .unwrap();
    let p0 = [0x10u8; 8];
    let p1 = [0x11u8; 8];
    let nondet = [0x77u8; 4];
    let rec = TraceRingRecord {
        step: 1,
        fire_time_ns: 1002,
        duration_ns: 3,
        node_idx: 0,
        global_level: 0,
        record_type: RECORD_TYPE_FIRE,
        reserved: 0,
    };
    w.write_chunk(|c| {
        c.write_message("/a", 0, 1000, 1000, &[&p0[..]])?;
        c.write_message("/a", 1, 1001, 1001, &[&p1[..]])?;
        c.write_scheduler_trace(2, 1002, 1002, &rec)?;
        c.write_message(NONDETERMINISM_TOPIC, 3, 1003, 1003, &[&nondet[..]])
    })
    .unwrap();
    w.write_attachment("graph.yaml", "application/yaml", 2000, 1999, b"g: 1\n")
        .unwrap();
    w.finalize().unwrap();
    let data = std::fs::read(&path).unwrap();
    const TOTAL_MSGS: usize = 4;

    // --- structural offsets (never hardcoded byte numbers) ---
    let summary_start = mcap::read::footer(&data).unwrap().summary_start as usize;
    // DataEnd record (opcode + u64 len + u32 crc = 13 B) precedes the summary.
    let dataend_start = summary_start - 13;
    assert_eq!(data[dataend_start], 0x0F, "DataEnd precedes the summary");
    // Footer record frame (opcode + u64 len = 9 B) + 20 field bytes + magic(8).
    let footer_frame_start = data.len() - 8 - 20 - 9;
    assert_eq!(data[footer_frame_start], 0x02, "Footer frame located");
    // First summary record's end = a mid-summary record boundary.
    let first_summary_len = u64::from_le_bytes(
        data[summary_start + 1..summary_start + 9]
            .try_into()
            .unwrap(),
    );
    let after_first_summary_record = summary_start + 9 + first_summary_len as usize;
    assert!(after_first_summary_record < footer_frame_start);

    // (cut, expected classification arm, label)
    type ExpectedArm = fn(&C) -> bool;
    let cuts: Vec<(usize, ExpectedArm, &str)> = vec![
        // (a) right after DataEnd, before any summary record: clean record
        //     boundary, epilogue missing.
        (
            summary_start,
            |c| matches!(c, C::TruncatedAtChunkBoundary),
            "after DataEnd",
        ),
        // (d) at a summary-record boundary between summary start and footer.
        (
            after_first_summary_record,
            |c| matches!(c, C::TruncatedAtChunkBoundary),
            "after first summary record",
        ),
        // (b) mid-Footer record: torn record.
        (
            footer_frame_start + 5,
            |c| matches!(c, C::TornTail(_)),
            "mid-Footer record",
        ),
        // (c) mid-closing-magic: the stream ends cleanly after the footer
        //     record, but the epilogue fingerprint (trailing magic) fails.
        (
            data.len() - 4,
            |c| matches!(c, C::TruncatedAtChunkBoundary),
            "mid-closing-magic",
        ),
    ];

    for (cut, expected_arm, label) in cuts {
        let truncated = data[..cut].to_vec();
        let r = BagReader::from_bytes(truncated);
        let (msgs, completeness) = r.recover_messages().unwrap();
        // The binding assertion for EVERY cut: never Finalized...
        assert!(
            !completeness.is_finalized(),
            "cut '{label}' (at {cut}) must NEVER be classified Finalized; got {completeness:?}"
        );
        // ...and the data section's messages are all still returned.
        assert_eq!(
            msgs.len(),
            TOTAL_MSGS,
            "cut '{label}': complete-chunk messages must all be recovered"
        );
        // Documented classification per arm.
        assert!(
            expected_arm(&completeness),
            "cut '{label}': unexpected classification {completeness:?}"
        );
    }

    // Positive control (anti-tautology): the INTACT bag reports Finalized with
    // the same message set + the attachment readable.
    let r = BagReader::from_bytes(data);
    let (msgs, completeness) = r.recover_messages().unwrap();
    assert!(completeness.is_finalized());
    assert_eq!(msgs.len(), TOTAL_MSGS);
    assert_eq!(
        r.attachment("graph.yaml").unwrap().unwrap().data,
        b"g: 1\n".to_vec()
    );
    let (trace, _) = r.recover_scheduler_trace().unwrap();
    assert_eq!(trace, vec![rec], "the reserved trace channel is populated");

    std::fs::remove_file(&path).ok();
}
