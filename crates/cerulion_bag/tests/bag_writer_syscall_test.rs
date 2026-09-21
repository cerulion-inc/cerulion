// SPDX-License-Identifier: AGPL-3.0-only
//! Drives the writer's `writev` batch loop through the scripted in-memory sink:
//! IOV_MAX batching, partial-write resumption at adversarial boundaries, EINTR
//! retry, and the no-progress (`writev` returns 0) hard-error path. Every
//! success path asserts byte-for-byte identity with the single-`writev`
//! expectation.
//!
//! A chunk flush is exactly TWO iovecs — `[chunk header][body]` —
//! because message framing and message payloads are appended to ONE contiguous
//! arena at record time. The tests in this file pin that property (never
//! one iovec per frame and per payload part with the payload iovecs
//! POINTER-IDENTICAL to the caller's buffers), including the behavioural one that a
//! pointer-stashing regression cannot pass: a caller's buffer may be mutated
//! the instant `write_message` returns.

use cerulion_bag::test_sink::{ScriptedSink, SinkAction};
use cerulion_bag::{BagError, BagWriter, BagWriterConfig, TopicSchema};

fn one_topic() -> Vec<TopicSchema> {
    vec![TopicSchema {
        topic: "/a".into(),
        schema_name: "std_msgs/UInt8".into(),
        schema_hash: 0xABCD,
        wire_fixed_size: 1,
    }]
}

/// Write the payloads as single-part messages on `/a` inside ONE chunk scope
/// (the scope-end flush is the writev under test). Returns the sink's recorded
/// bytes + the writev call count.
fn write_and_flush(sink: ScriptedSink, payloads: &[Vec<u8>]) -> (Vec<u8>, usize) {
    let mut w = BagWriter::with_sink(sink, BagWriterConfig::default(), &one_topic()).unwrap();
    w.write_chunk(|c| {
        for (i, p) in payloads.iter().enumerate() {
            c.write_message("/a", i as u32, 1000 + i as u64, 1000 + i as u64, &[&p[..]])?;
        }
        Ok(())
    })
    .unwrap();
    (w.sink().bytes().to_vec(), w.sink().writev_calls())
}

/// Like `write_and_flush` but returns the scope-end error (for the failure-mode
/// scripts).
fn write_expect_err(sink: ScriptedSink, payloads: &[Vec<u8>]) -> BagError {
    let mut w = BagWriter::with_sink(sink, BagWriterConfig::default(), &one_topic()).unwrap();
    w.write_chunk(|c| {
        for (i, p) in payloads.iter().enumerate() {
            c.write_message("/a", i as u32, 1000 + i as u64, 1000, &[&p[..]])?;
        }
        Ok(())
    })
    .expect_err("the scripted sink failure must surface from write_chunk")
}

fn payloads(n: usize) -> Vec<Vec<u8>> {
    (0..n).map(|i| vec![i as u8; 3 + (i % 5)]).collect()
}

#[test]
fn single_call_is_the_oracle_and_iov_max_batching_matches_it() {
    let pls = payloads(10);
    // Oracle: one writev per flush (huge iov_max, all Full).
    let (oracle, oracle_calls) = write_and_flush(ScriptedSink::single_call(), &pls);
    assert_eq!(
        oracle_calls, 1,
        "single_call flushes the chunk in ONE writev"
    );

    // A chunk is `[header][body]` = 2 iovecs however many messages it
    // holds, so `iov_max = 1` is what forces the batching loop to split now
    // (a 21-iovec per-frame plan would need 4). The loop is therefore
    // UNREACHABLE in production — POSIX guarantees `IOV_MAX >= 16` — which is
    // precisely why it is worth pinning here rather than trusting it.
    let (batched, calls) = write_and_flush(ScriptedSink::new(1, std::iter::empty()), &pls);
    assert_eq!(
        calls, 2,
        "iov_max=1 must split the 2-iovec flush into exactly 2 writev calls (got {calls})"
    );
    assert_eq!(batched, oracle, "batched byte stream == single-call oracle");
}

#[test]
fn partial_writes_at_adversarial_boundaries_resume_identically() {
    let pls = payloads(6);
    let (oracle, _) = write_and_flush(ScriptedSink::single_call(), &pls);

    // Short(1) everywhere forces resumption at EVERY byte boundary (mid-frame
    // AND mid-payload AND iovec boundaries all covered), interleaved with EINTR.
    let script = (0..4096).flat_map(|i| {
        if i % 7 == 0 {
            vec![SinkAction::Interrupted, SinkAction::Short(1)]
        } else {
            vec![SinkAction::Short(1)]
        }
    });
    let (got, _) = write_and_flush(ScriptedSink::new(3, script), &pls);
    assert_eq!(
        got, oracle,
        "byte-for-byte identical under 1-byte partial writes"
    );
}

#[test]
fn explicit_mid_frame_and_mid_payload_and_boundary_cuts_resume() {
    // One 10-byte message: chunk header (49) + frame (31) + payload (10) = 90.
    let pls = vec![vec![0x77u8; 10]];
    let (oracle, _) = write_and_flush(ScriptedSink::single_call(), &pls);

    // Cut MID-FRAME (54 = header 49 + 5 of the 31-byte frame), then MID-PAYLOAD
    // (29 = frame rest 26 + 3 of the 10-byte payload), then the remaining 7.
    let script = [
        SinkAction::Short(54),
        SinkAction::Short(29),
        SinkAction::Full,
    ];
    let (got, _) = write_and_flush(ScriptedSink::new(8, script), &pls);
    assert_eq!(
        got, oracle,
        "explicit boundary cuts reassemble the same bytes"
    );
}

/// The `ScriptedSink` CALL LOG is faithful — it records every call,
/// in order, with the right kind and size.
///
/// The log is new test apparatus, and it is the sole oracle for
/// `bag_late_channel_test::a_late_registration_is_one_write_and_no_extra_chunk_boundary`
/// (the two older probes cannot see a cold-path write: `writev_calls()` counts
/// chunk flushes only, and `iov_captures()` is never touched by `write_bytes`).
/// An apparatus that mis-recorded would make that oracle vacuous rather than
/// wrong, so it is pinned HERE, in the file that owns the writer's syscall
/// shape, against facts this file already establishes independently.
///
/// Every claim is cross-checked against a probe that existed before the log:
/// the `Writev` entries must agree with `writev_calls()` and their iovec counts
/// with `iov_captures()`, and the recorded `WriteBytes` lengths must sum with
/// the writev bytes to exactly the byte stream the sink accepted.
#[test]
fn the_call_log_records_every_sink_call_in_order_with_its_kind_and_size() {
    use cerulion_bag::test_sink::SinkCall;

    let pls = payloads(3);
    let mut w = BagWriter::with_sink(
        ScriptedSink::single_call(),
        BagWriterConfig::default(),
        &one_topic(),
    )
    .unwrap();
    w.write_chunk(|c| {
        for (i, p) in pls.iter().enumerate() {
            c.write_message("/a", i as u32, 1000 + i as u64, 1000 + i as u64, &[&p[..]])?;
        }
        Ok(())
    })
    .unwrap();

    let calls: Vec<SinkCall> = w.sink().calls().to_vec();

    // The SHAPE, written out by hand: the prelude is one cold-path write, the
    // chunk flush is one 2-iovec writev (header + arena), and its
    // single channel's MessageIndex is one more cold-path write.
    assert!(
        matches!(
            calls.as_slice(),
            [
                SinkCall::WriteBytes(_),
                SinkCall::Writev(2),
                SinkCall::WriteBytes(_)
            ]
        ),
        "expected [prelude write][chunk writev(2)][MessageIndex write]; got {calls:?}"
    );

    // Cross-check 1: the log's writev entries agree with the older counter.
    assert_eq!(
        calls
            .iter()
            .filter(|c| matches!(c, SinkCall::Writev(_)))
            .count(),
        w.sink().writev_calls(),
        "the log must not miss or invent a writev"
    );

    // Cross-check 2: the iovec counts agree with `iov_captures()`.
    let logged_iovs: Vec<usize> = calls
        .iter()
        .filter_map(|c| match c {
            SinkCall::Writev(n) => Some(*n),
            _ => None,
        })
        .collect();
    let captured_iovs: Vec<usize> = w.sink().iov_captures().iter().map(Vec::len).collect();
    assert_eq!(logged_iovs, captured_iovs);

    // Cross-check 3: the SIZES are real. Every call in this fixture is accepted
    // in full, so the logged cold-path lengths plus the writev payloads must sum
    // to exactly the byte stream the sink holds.
    let logged_bytes: usize = calls
        .iter()
        .map(|c| match c {
            SinkCall::WriteBytes(n) => *n,
            SinkCall::Writev(_) => 0,
        })
        .sum();
    let writev_bytes: usize = w
        .sink()
        .iov_captures()
        .iter()
        .flat_map(|iovs| iovs.iter().map(|(_, len)| *len))
        .sum();
    assert_eq!(
        logged_bytes + writev_bytes,
        w.sink().bytes().len(),
        "the logged sizes must account for the whole accepted stream"
    );

    drop(w.finalize());
}

#[test]
fn eintr_is_retried_and_yields_identical_bytes() {
    let pls = payloads(4);
    let (oracle, _) = write_and_flush(ScriptedSink::single_call(), &pls);

    // Two EINTRs before any progress, then normal completion.
    let script = [
        SinkAction::Interrupted,
        SinkAction::Interrupted,
        SinkAction::Full,
        SinkAction::Full,
        SinkAction::Full,
    ];
    let (got, calls) = write_and_flush(ScriptedSink::new(4, script), &pls);
    assert_eq!(got, oracle, "EINTR-retried stream is identical");
    assert!(
        calls >= 3,
        "the two interrupts were retried (got {calls} calls)"
    );
}

#[test]
fn persistent_zero_write_is_a_hard_error_not_silent_truncation() {
    let err = write_expect_err(ScriptedSink::new(4, [SinkAction::Zero]), &payloads(3));
    assert!(
        matches!(err, BagError::WritevNoProgress { .. }),
        "expected WritevNoProgress, got {err:?}"
    );
}

#[test]
fn writev_failure_surfaces_errno_and_offset() {
    let err = write_expect_err(
        ScriptedSink::new(4, [SinkAction::Fail(libc::ENOSPC)]),
        &payloads(3),
    );
    match err {
        BagError::Writev { errno, .. } => assert_eq!(errno, libc::ENOSPC),
        other => panic!("expected Writev errno error, got {other:?}"),
    }
}

/// Copy-into-arena, the STRUCTURAL half: a chunk flush is `[header][body]` and NEITHER
/// iovec aliases the caller's payload buffer.
///
/// This replaces `payload_iovec_is_pointer_identical_to_caller_buffer`, whose
/// assertion was the exact inverse (one iovec per part, pointer-identical to
/// the caller's memory). Both directions are worth pinning structurally for the
/// same reason that test gave: byte equality alone cannot tell a copy from an
/// alias, so a revert to pointer-stashing would pass every byte-level test in
/// this file. The negative assertion here is what a revert fails.
#[test]
fn a_chunk_flush_is_two_iovecs_and_neither_aliases_the_callers_payload() {
    let payload = [0xC3u8; 64];
    let mut w = BagWriter::with_sink(
        ScriptedSink::single_call(),
        BagWriterConfig::default(),
        &one_topic(),
    )
    .unwrap();
    w.write_chunk(|c| c.write_message("/a", 0, 1, 1, &[&payload[..]]))
        .unwrap();

    let captures = w.sink().iov_captures();
    assert_eq!(captures.len(), 1, "one chunk flush = one writev call");
    let call = &captures[0];
    assert_eq!(
        call.len(),
        2,
        "expected exactly [chunk header][body]; got {call:?}"
    );

    let base = payload.as_ptr() as usize;
    let payload_end = base + payload.len();
    // NEGATIVE (the revert-sensitive half): no iovec may overlap the caller's
    // buffer at all — the bytes handed to the kernel are writer-owned.
    for (p, l) in call.iter() {
        let iov_end = p + l;
        assert!(
            iov_end <= base || *p >= payload_end,
            "iovec [{p:#x}, {iov_end:#x}) overlaps the caller's payload buffer \
             [{base:#x}, {payload_end:#x}) — payloads must be COPIED into the arena"
        );
    }
    // POSITIVE anti-tautology: the payload bytes really are in the body (a
    // writer that dropped them entirely would also satisfy the negative arm).
    let body_len = call[1].1;
    assert!(
        body_len >= payload.len(),
        "the body iovec ({body_len} B) must carry the {} B payload",
        payload.len()
    );
    assert!(
        w.sink()
            .bytes()
            .windows(payload.len())
            .any(|win| win == payload),
        "the payload bytes must appear verbatim in the written stream"
    );
}

/// The multi-part variant: both parts are concatenated into the body, in
/// order, adjacent (replaces `multi_part_payload_iovecs_are_pointer_identical_in_order`).
#[test]
fn multi_part_payloads_are_concatenated_into_the_body_in_order() {
    let part_a = [0x11u8; 16];
    let part_b = [0x22u8; 48];
    let mut w = BagWriter::with_sink(
        ScriptedSink::single_call(),
        BagWriterConfig::default(),
        &one_topic(),
    )
    .unwrap();
    w.write_chunk(|c| c.write_message("/a", 0, 1, 1, &[&part_a[..], &part_b[..]]))
        .unwrap();

    let call = &w.sink().iov_captures()[0];
    assert_eq!(call.len(), 2, "[chunk header][body]");

    // Hand oracle: the two parts land back-to-back, a-then-b, exactly once.
    let mut joined = Vec::new();
    joined.extend_from_slice(&part_a);
    joined.extend_from_slice(&part_b);
    let stream = w.sink().bytes();
    let occurrences = stream
        .windows(joined.len())
        .filter(|win| *win == joined.as_slice())
        .count();
    assert_eq!(
        occurrences, 1,
        "part 0 then part 1, adjacent and in order, exactly once"
    );
}

/// Copy-into-arena, the BEHAVIOURAL half — and the one a pointer-stashing regression
/// cannot pass: the caller's buffer is free the instant `write_message`
/// returns.
///
/// This is the property the whole change exists for. The recorder drains a
/// shared-memory sample, records it, and RELEASES the borrow immediately; if
/// the writer still referenced that memory at flush time the released slot's
/// new contents would land in the bag. Mutating the buffer between the record
/// and the flush models exactly that, with a hand oracle (the ORIGINAL bytes
/// must be in the bag; the overwritten bytes must not be).
#[test]
fn a_payload_mutated_after_write_message_does_not_change_the_bag() {
    const ORIGINAL: u8 = 0xA5;
    const OVERWRITE: u8 = 0x5A;
    let mut payload = [ORIGINAL; 64];
    let mut w = BagWriter::with_sink(
        ScriptedSink::single_call(),
        BagWriterConfig::default(),
        &one_topic(),
    )
    .unwrap();

    w.write_message("/a", 0, 1, 1, &[&payload[..]]).unwrap();
    // The caller reuses/releases its buffer BEFORE the chunk is flushed.
    payload.fill(OVERWRITE);
    w.flush_chunk().unwrap();

    let stream = w.sink().bytes();
    assert!(
        stream
            .windows(64)
            .any(|win| win.iter().all(|b| *b == ORIGINAL)),
        "the bag must carry the bytes as they were AT RECORD TIME"
    );
    assert!(
        !stream
            .windows(64)
            .any(|win| win.iter().all(|b| *b == OVERWRITE)),
        "the post-record overwrite must NOT reach the bag — the payload was copied"
    );
}

/// Revert-sensitive: a FAILED chunk writev must not pollute the
/// running data-section CRC. Were the chunk's framed bytes folded into
/// `data_crc` BEFORE the writev, then after a writev Err + discard a later
/// finalize() would stamp a DataEnd `data_section_crc` covering ghost bytes that
/// never hit the sink — a silently-invalid CRC on an otherwise-valid bag.
///
/// THE BITING ORACLE is our own recomputation: the mcap crate's MessageStream
/// path validates CHUNK CRCs only (`validate_data_section_crc` defaults off),
/// so a strict mcap read passes even on the corrupt-CRC file. We therefore
/// recompute CRC32 over the file's data section (every byte before the DataEnd
/// record) and compare with the stamped value.
mod failed_chunk_flush {
    use super::*;
    use cerulion_bag::{BagReader, WritevOutcome, WritevSink};
    use std::cell::RefCell;
    use std::io::{self, IoSlice};
    use std::rc::Rc;

    /// Delegating sink so the test keeps a handle to the recorded bytes after
    /// `finalize()` consumes the writer.
    struct SharedSink(Rc<RefCell<ScriptedSink>>);
    impl WritevSink for SharedSink {
        fn writev_once(&mut self, iovs: &[IoSlice<'_>]) -> WritevOutcome {
            self.0.borrow_mut().writev_once(iovs)
        }
        fn iov_max(&self) -> usize {
            self.0.borrow().iov_max()
        }
        fn write_bytes(&mut self, buf: &[u8]) -> io::Result<()> {
            self.0.borrow_mut().write_bytes(buf)
        }
        fn sync_all(&mut self) -> io::Result<()> {
            self.0.borrow_mut().sync_all()
        }
    }

    #[test]
    fn failed_chunk_writev_does_not_pollute_data_section_crc() {
        // First writev (chunk 1's flush) fails; everything after succeeds.
        let inner = Rc::new(RefCell::new(ScriptedSink::new(
            usize::MAX,
            [SinkAction::Fail(libc::EIO)],
        )));
        let mut w = BagWriter::with_sink(
            SharedSink(Rc::clone(&inner)),
            BagWriterConfig::default(),
            &one_topic(),
        )
        .unwrap();

        let doomed = [0xDDu8; 16];
        let err = w
            .write_chunk(|c| c.write_message("/a", 0, 100, 100, &[&doomed[..]]))
            .expect_err("the scripted writev failure must surface");
        assert!(matches!(err, BagError::Writev { .. }), "got {err:?}");

        // The writer continues: a second chunk succeeds, then finalize.
        let kept = [0x99u8; 16];
        w.write_chunk(|c| c.write_message("/a", 1, 200, 200, &[&kept[..]]))
            .unwrap();
        w.finalize().unwrap();
        let data = inner.borrow().bytes().to_vec();

        // Strict mcap-crate read passes; only the surviving message is present.
        let msgs: Vec<mcap::Message> = mcap::MessageStream::new(&data)
            .unwrap()
            .collect::<Result<_, _>>()
            .unwrap();
        assert_eq!(msgs.len(), 1);
        assert_eq!(msgs[0].sequence, 1);
        assert_eq!(&msgs[0].data[..], &kept[..]);

        // Our reader agrees and reports Finalized.
        let r = BagReader::from_bytes(data.clone());
        let (recovered, completeness) = r.recover_messages().unwrap();
        assert_eq!(recovered.len(), 1);
        assert!(completeness.is_finalized());

        // The biting assertion: recompute the data-section CRC. The data
        // section is every byte before the DataEnd record, which sits
        // immediately before summary_start (13 bytes: opcode + u64 len + u32).
        let summary_start = mcap::read::footer(&data).unwrap().summary_start as usize;
        let dataend_start = summary_start - 13;
        assert_eq!(
            data[dataend_start], 0x0F,
            "structural precondition: DataEnd record precedes the summary"
        );
        let stamped = u32::from_le_bytes(
            data[dataend_start + 9..dataend_start + 13]
                .try_into()
                .unwrap(),
        );
        let recomputed = crc32fast::hash(&data[..dataend_start]);
        assert_eq!(
            stamped, recomputed,
            "data_section_crc must cover exactly the bytes durably written — \
             a failed chunk flush must not fold its ghost bytes into the running CRC"
        );
    }

    /// A failed `flush_chunk` leaves the chunk RETRYABLE — including its message
    /// TIME BOUNDS.
    ///
    /// A `flush_chunk` that `take()`s `chunk_msg_start`/`chunk_msg_end` before
    /// issuing the `writev` loses those bounds on the error path while
    /// the chunk itself survives, so a caller that RETRIES the flush (rather
    /// than discarding the chunk) stamps `0..0` into both the Chunk header and
    /// the ChunkIndex — and, through `file_msg_start`/`file_msg_end`, into the
    /// file Statistics — while every message inside kept its real log time. An
    /// index that disagrees with its own content, produced on exactly the path a
    /// mid-run write error takes.
    ///
    /// The oracle is deliberately the SUMMARY, not the messages: the message
    /// stream is right either way, so only the summary can show it.
    ///
    /// Restoring `.take()` fails this at `0` on all three bounds.
    #[test]
    fn a_retried_chunk_flush_keeps_its_real_message_time_bounds() {
        const LOG_TIME: u64 = 123_456_789;

        // First writev (the chunk flush) fails; everything after succeeds. The
        // prelude rides `write_bytes`, so the flush really is the first writev.
        let inner = Rc::new(RefCell::new(ScriptedSink::new(
            usize::MAX,
            [SinkAction::Fail(libc::EIO)],
        )));
        let mut w = BagWriter::with_sink(
            SharedSink(Rc::clone(&inner)),
            BagWriterConfig::default(),
            &one_topic(),
        )
        .unwrap();

        let payload = [0x5Au8; 16];
        w.write_message("/a", 0, LOG_TIME, LOG_TIME, &[&payload[..]])
            .unwrap();
        let err = w
            .flush_chunk()
            .expect_err("the scripted writev failure must surface");
        assert!(matches!(err, BagError::Writev { .. }), "got {err:?}");

        // RETRY the same chunk — the caller did NOT discard it. This is the
        // shape under test; `discard_pending_chunk` clears the bounds itself and
        // so can never show the bug.
        w.flush_chunk().expect("the retry must succeed");
        w.finalize().unwrap();
        let data = inner.borrow().bytes().to_vec();

        // The message is present, with its real log time (this half always
        // passed).
        let msgs: Vec<mcap::Message> = mcap::MessageStream::new(&data)
            .unwrap()
            .collect::<Result<_, _>>()
            .unwrap();
        assert_eq!(
            msgs.len(),
            1,
            "the retried chunk's message must be in the bag"
        );
        assert_eq!(msgs[0].log_time, LOG_TIME);

        // THE BITING ORACLE: the summary must agree with the content it indexes.
        let summary = mcap::Summary::read(&data).unwrap().expect("valid summary");
        let ci = summary
            .chunk_indexes
            .first()
            .expect("one chunk index for the retried chunk");
        assert_eq!(
            (ci.message_start_time, ci.message_end_time),
            (LOG_TIME, LOG_TIME),
            "the retried chunk's INDEX must carry the bounds of the messages it holds"
        );
        let stats = summary.stats.expect("statistics record");
        assert_eq!(
            (stats.message_start_time, stats.message_end_time),
            (LOG_TIME, LOG_TIME),
            "the file's Statistics fold the chunk's bounds, so they drift with it"
        );
    }
}

/// `bytes_written` counts only bytes DURABLY handed to the
/// sink — a PENDING (unflushed) chunk is excluded, and after every flush the
/// counter equals the sink's recorded stream length exactly.
///
/// `write_scheduler_trace` is the probe for the pending-exclusion half: unlike
/// `write_chunk` (whose scope-end always flushes), it queues into the pending
/// chunk WITHOUT flushing below the size threshold, so the pending state is
/// observable from outside.
#[test]
fn bytes_written_tracks_durable_sink_bytes_and_excludes_pending_chunk() {
    let mut w = BagWriter::with_sink(
        ScriptedSink::single_call(),
        BagWriterConfig::default(),
        &one_topic(),
    )
    .unwrap();

    // After create: magic + Header + Schemas + Channels are durable.
    let prelude = w.bytes_written();
    assert!(prelude > 0, "the prelude must already be counted");
    assert_eq!(
        prelude as usize,
        w.sink().bytes().len(),
        "bytes_written == the sink's recorded stream after create"
    );

    // Queue a trace record into the PENDING chunk — nothing flushed yet, so
    // bytes_written must NOT move (pending exclusion) and the sink is unchanged.
    let rec = cerulion_bag::TraceRingRecord {
        step: 1,
        fire_time_ns: 10,
        duration_ns: 5,
        node_idx: 0,
        global_level: 0,
        record_type: 1,
        reserved: 0,
    };
    w.write_scheduler_trace(0, 10, 10, &rec).unwrap();
    assert_eq!(
        w.bytes_written(),
        prelude,
        "a pending (unflushed) chunk must NOT be counted"
    );
    // The durable-prefix getters share the pending-chunk
    // exclusion — a queued-but-unflushed message counts NOTHING.
    assert_eq!(
        w.messages_persisted(),
        0,
        "pending messages are not persisted"
    );
    assert_eq!(w.chunks_flushed(), 0, "no chunk flushed yet");
    assert_eq!(
        w.sink().bytes().len(),
        prelude as usize,
        "nothing reached the sink yet"
    );

    // Flush: the counter grows by exactly the flushed chunk + MessageIndex
    // bytes and equals the sink stream length again.
    w.flush_chunk().unwrap();
    let after_flush = w.bytes_written();
    assert!(after_flush > prelude, "the flushed chunk must be counted");
    assert_eq!(
        w.messages_persisted(),
        1,
        "the flushed trace record is persisted"
    );
    assert_eq!(w.chunks_flushed(), 1, "exactly one durable chunk");
    assert_eq!(
        after_flush as usize,
        w.sink().bytes().len(),
        "bytes_written == the sink's recorded stream after the flush"
    );

    // The zero-copy chunk path (write_chunk scope-end flush) holds the same
    // equality: the delta is exactly what the sink accepted.
    let payload = [0x7Au8; 64];
    w.write_chunk(|c| c.write_message("/a", 1, 20, 20, &[&payload[..]]))
        .unwrap();
    assert_eq!(
        w.bytes_written() as usize,
        w.sink().bytes().len(),
        "bytes_written == sink stream after a write_chunk flush"
    );
    assert!(w.bytes_written() > after_flush);
}
