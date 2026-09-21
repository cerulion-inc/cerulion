// SPDX-License-Identifier: AGPL-3.0-only
//! — the state ring over REAL POSIX SHM.
//!
//! The PURE halves (the record byte layout, the chunk-count arithmetic, every
//! reassembly verdict) are in-module unit tests in `state_ring.rs`; this file is the
//! behavioural half — the seams a pure test structurally cannot see:
//!
//! * a whole anchor really crossing a `MAP_SHARED` segment and coming back
//!   byte-identical, with `node_idx` resolving through the ring MANIFEST and the
//!   producer RANK riding the ring HEADER (rank does not ride the record);
//! * the ring really selecting [`OverrunPolicy::Backpressure`], and a consumer
//!   REFUSING one that did not — the "no inert shipping" proof that the mode is not
//!   a comment;
//! * a slow recorder making the writer WAIT with ZERO loss (the backpressure gate) and a
//!   WEDGED one degrading to a LOUD lap rather than silence;
//! * a real `fork(2)` child streaming an anchor and the parent resyncing — this
//!   composition, which is the shipping shape and cannot be modelled by a thread.
//!
//! Every cross-thread and cross-process wait here is bounded by a GENEROUS liveness
//! deadline: the work it bounds is milliseconds, so seconds only decide how long a
//! WEDGE takes to report itself. An unbounded wait would turn a real failure into a
//! hang, and a hung test burns a CI job's whole timeout with no attributable red.
//!
//! Per-test ring tags (name + pid) ⇒ parallel-safe; no `#[serial]`, no iceoryx2.

#![cfg(unix)]

use std::time::{Duration, Instant};

use cerulion_core::shm_ring::{OverrunPolicy, ShmRingOwner};
use cerulion_core::state_ring::{
    encode_record, scan_state_ring_ranks, state_ring_shm_name, state_ring_tag,
    unlink_stale_state_rings, StateAnchorEvent, StateAssembler, StateRecordHeader,
    StateRingConsumer, StateRingError, StateRingOwner, TornCause, RECORD_KIND_CHUNK,
    RECORD_KIND_FINAL, STATE_RECORD_PAYLOAD, STATE_RECORD_SIZE, STATE_RING_PROBE_GAP_TOLERANCE,
};
use cerulion_core::trace_ring::encode_manifest;

// ===========================================================================
// Shared helpers
// ===========================================================================

/// Unique ring tag per test (name + pid) so parallel tests / re-runs never collide on
/// a `/dev/shm` object.
fn tag(name: &str) -> String {
    format!("state_{name}_{}", std::process::id())
}

/// GENEROUS liveness ceiling for every rendezvous in this file. Never a wall stated
/// in units of the thing under test.
const DEADLINE: Duration = Duration::from_secs(30);

/// A blob that is its OWN oracle: byte `i` is `(i * 7 + 3) % 251`, so a reassembly
/// that dropped, duplicated or reordered a 480-byte chunk cannot look right.
fn blob(n: usize) -> Vec<u8> {
    (0..n).map(|i| ((i * 7 + 3) % 251) as u8).collect()
}

/// Drain everything currently available and return the events, asserting no lap.
fn drain_all(c: &mut StateRingConsumer, asm: &mut StateAssembler) -> Vec<StateAnchorEvent> {
    let mut out = Vec::new();
    c.drain(asm, &mut out).expect("no lap");
    out
}

// ===========================================================================
// 1. A whole anchor crosses the segment byte-identically
// ===========================================================================

/// The headline round trip: a 2.5-record anchor written through the sink, drained by
/// a consumer that opened its OWN mapping, and reassembled byte-for-byte against a
/// hand-built oracle — never a self-compare (the blob is generated from its own
/// indices, so the assertion is against arithmetic, not against what was written).
///
/// It also pins the two identity halves kept deliberately separate: `node_idx` is an
/// INDEX resolved through the ring MANIFEST (a fixed-width name field would collide
/// two nodes sharing a prefix), and `rank` rides the ring HEADER, not the record.
#[test]
fn a_multi_record_anchor_crosses_the_segment_and_reassembles_byte_identically() {
    const RUN: u64 = 0xDEAD_BEEF_0000_0001;
    const STEP: u64 = 4_242;
    let want = blob(1_100); // 3 records: 480 + 480 + 140

    let mut owner = StateRingOwner::create(
        &tag("roundtrip"),
        1_024,
        3, // rank
        RUN,
        &["camera", "planner", "costmap"],
    )
    .expect("create");
    let name = owner.name().to_string();
    assert_eq!(
        owner.overrun_policy(),
        OverrunPolicy::Backpressure,
        "a state ring is ALWAYS backpressure — an anchor through a wait-free ring is \
         lost, not slow"
    );
    let mut producer = owner.producer().expect("producer");

    let mut sink = producer.sink(STEP, 1);
    // Write in awkward slices so the chunker's fill/flush boundaries are exercised
    // rather than aligned with the caller's writes.
    for piece in want.chunks(37) {
        sink.append(piece);
    }
    let parts = sink.finish();
    assert_eq!(parts, 3);
    assert_eq!(producer.pushed(), 3);

    let mut consumer = StateRingConsumer::open(&name).expect("open");
    assert_eq!(consumer.rank(), 3, "rank rides the ring HEADER");
    assert_eq!(consumer.record_size(), STATE_RECORD_SIZE);
    assert_eq!(consumer.node_ids(), ["camera", "planner", "costmap"]);

    let mut asm = StateAssembler::passthrough();
    let events = drain_all(&mut consumer, &mut asm);
    assert_eq!(
        events,
        vec![StateAnchorEvent::Complete {
            run_id: RUN,
            step: STEP,
            node_idx: 1,
            parts: 3,
            bytes: want,
        }]
    );
    // The manifest is what turns the index back into an identity.
    assert_eq!(consumer.node_id(1), Some("planner"));
    assert_eq!(consumer.node_id(99), None);
    assert!(asm.finish().is_empty(), "nothing left open");
    drop(owner);
}

/// The two chunk-count boundaries over the real ring, in ONE body so the +1 is a
/// DIFFERENCE rather than two independent readings: a blob that exactly fills its
/// records emits no trailing empty one, and a blob one byte longer emits exactly one
/// more.
#[test]
fn a_boundary_exact_anchor_and_the_one_byte_over_twin_differ_by_exactly_one_record() {
    let exact = blob(STATE_RECORD_PAYLOAD * 3);
    let over = blob(STATE_RECORD_PAYLOAD * 3 + 1);

    let mut owner = StateRingOwner::create(&tag("boundary"), 1_024, 0, 7, &["n"]).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");

    let mut sink = producer.sink(1, 0);
    sink.append(&exact);
    assert_eq!(sink.finish(), 3, "3 * 480 becomes exactly 3 records");
    let after_exact = producer.pushed();
    assert_eq!(after_exact, 3);

    let mut sink = producer.sink(2, 0);
    sink.append(&over);
    assert_eq!(
        sink.finish(),
        4,
        "one byte over costs exactly one more record"
    );
    assert_eq!(producer.pushed() - after_exact, 4);

    let mut consumer = StateRingConsumer::open(&name).expect("open");
    let mut asm = StateAssembler::passthrough();
    let events = drain_all(&mut consumer, &mut asm);
    assert_eq!(
        events,
        vec![
            StateAnchorEvent::Complete {
                run_id: 7,
                step: 1,
                node_idx: 0,
                parts: 3,
                bytes: exact,
            },
            StateAnchorEvent::Complete {
                run_id: 7,
                step: 2,
                node_idx: 0,
                parts: 4,
                bytes: over,
            },
        ]
    );
    drop(owner);
}

/// Two nodes' anchors written INTERLEAVED record for record must reassemble
/// independently. `node_idx` is part of the reassembly key; drop it and these two
/// streams concatenate into two wrong blobs of the right total length.
///
/// The interleave is produced by writing both sinks in lockstep through the REAL
/// producer, which is the shape the bag presents when several ranks' records merge.
#[test]
fn two_nodes_written_interleaved_reassemble_independently() {
    let a = blob(1_100);
    let b: Vec<u8> = blob(1_100).iter().map(|x| x ^ 0xFF).collect();

    let mut owner =
        StateRingOwner::create(&tag("interleave"), 1_024, 0, 11, &["a", "b"]).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");

    // A sink owns the producer for one anchor's lifetime, so a genuine record-for-
    // record interleave is written through `push_record` — the same seam the sink
    // itself writes through, with the same encoder.
    let a_pieces: Vec<&[u8]> = a.chunks(STATE_RECORD_PAYLOAD).collect();
    let b_pieces: Vec<&[u8]> = b.chunks(STATE_RECORD_PAYLOAD).collect();
    assert_eq!(a_pieces.len(), 3);
    assert_eq!(b_pieces.len(), 3);
    for i in 0..3 {
        for (node_idx, piece) in [(0u32, a_pieces[i]), (1u32, b_pieces[i])] {
            let header = StateRecordHeader {
                run_id: 11,
                step: 5,
                node_idx,
                part: i as u32,
                kind: if i == 2 {
                    RECORD_KIND_FINAL
                } else {
                    RECORD_KIND_CHUNK
                },
                len: piece.len() as u32,
            };
            producer.push_record(&encode_record(&header, piece));
        }
    }

    let mut consumer = StateRingConsumer::open(&name).expect("open");
    let mut asm = StateAssembler::passthrough();
    let events = drain_all(&mut consumer, &mut asm);
    assert_eq!(events.len(), 2, "two anchors, two Complete events");
    assert_eq!(
        events[0],
        StateAnchorEvent::Complete {
            run_id: 11,
            step: 5,
            node_idx: 0,
            parts: 3,
            bytes: a,
        }
    );
    assert_eq!(
        events[1],
        StateAnchorEvent::Complete {
            run_id: 11,
            step: 5,
            node_idx: 1,
            parts: 3,
            bytes: b,
        }
    );
    drop(owner);
}

/// A record LOST from the middle of an anchor must be reported TORN, and NO short
/// blob may be served for it — the rule that makes an incomplete checkpoint
/// detectable instead of silently wrong.
///
/// The gap is produced over the real ring by pushing hand-built records and skipping
/// one, which is exactly what a lost record looks like to the reader.
#[test]
fn an_omitted_middle_record_is_torn_over_the_real_ring_and_serves_no_short_blob() {
    let want = blob(STATE_RECORD_PAYLOAD * 4);
    let mut owner = StateRingOwner::create(&tag("torn"), 1_024, 0, 1, &["n"]).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");

    for (i, piece) in want.chunks(STATE_RECORD_PAYLOAD).enumerate() {
        if i == 2 {
            continue; // the lost record
        }
        let header = StateRecordHeader {
            run_id: 1,
            step: 1,
            node_idx: 0,
            part: i as u32,
            kind: if i == 3 {
                RECORD_KIND_FINAL
            } else {
                RECORD_KIND_CHUNK
            },
            len: piece.len() as u32,
        };
        producer.push_record(&encode_record(&header, piece));
    }

    let mut consumer = StateRingConsumer::open(&name).expect("open");
    let mut asm = StateAssembler::passthrough();
    let mut events = drain_all(&mut consumer, &mut asm);
    events.extend(asm.finish());
    assert_eq!(
        events,
        vec![StateAnchorEvent::Torn {
            run_id: 1,
            step: 1,
            node_idx: 0,
            cause: TornCause::PartOutOfOrder {
                expected: 2,
                got: 3
            },
        }]
    );
    assert!(
        !events
            .iter()
            .any(|e| matches!(e, StateAnchorEvent::Complete { .. })),
        "a torn anchor must NEVER be served as a Complete short blob"
    );
    drop(owner);
}

// ===========================================================================
// 2. The policy is real, and a mis-created ring is refused at the first read
// ===========================================================================

/// A ring created under the WAIT-FREE policy is refused by a state consumer, loudly
/// and at OPEN.
///
/// A state ring under `FailLoud` does not merely go slower: the writer never waits,
/// so a 1.09 M-record anchor laps a fixed ring and the anchor is LOST. Refusing at
/// open turns that into one error instead of a bag full of torn anchors.
///
/// Its ANTI-TAUTOLOGY half is the headline round trip above, which opens a
/// backpressure ring through the same code path and succeeds.
#[test]
fn a_state_consumer_refuses_a_ring_that_is_not_backpressure() {
    let manifest = encode_manifest(&["n"]).expect("manifest");
    let owner = ShmRingOwner::create(&tag("failloud"), STATE_RECORD_SIZE, 16, 0, &manifest)
        .expect("create");
    let name = owner.name().to_string();
    match StateRingConsumer::open(&name) {
        Err(StateRingError::OverrunPolicyMismatch {
            actual, expected, ..
        }) => {
            assert_eq!(actual, OverrunPolicy::FailLoud.as_wire());
            assert_eq!(expected, OverrunPolicy::Backpressure.as_wire());
        }
        other => panic!("expected OverrunPolicyMismatch, got {other:?}"),
    }
    drop(owner);
}

/// A ring whose records are not state records is refused too — the sibling guard, so
/// a trace ring handed to a state consumer by mistake fails at open rather than
/// decoding scheduler records as checkpoint headers.
#[test]
fn a_state_consumer_refuses_a_ring_whose_records_are_the_wrong_size() {
    let manifest = encode_manifest(&["n"]).expect("manifest");
    let owner = ShmRingOwner::create_with_policy(
        &tag("wrongsize"),
        40, // a trace record
        16,
        0,
        &manifest,
        OverrunPolicy::Backpressure,
    )
    .expect("create");
    let name = owner.name().to_string();
    match StateRingConsumer::open(&name) {
        Err(StateRingError::RecordSizeMismatch { actual, expected }) => {
            assert_eq!(actual, 40);
            assert_eq!(expected, STATE_RECORD_SIZE);
        }
        other => panic!("expected RecordSizeMismatch, got {other:?}"),
    }
    drop(owner);
}

// ===========================================================================
// 3. The backpressure gate: a slow recorder makes the writer WAIT, with ZERO loss
// ===========================================================================

/// THE gate. An anchor far larger than the ring, against a recorder deliberately
/// slower than the writer, must arrive WHOLE — which under the wait-free policy is
/// impossible by construction.
///
/// Three assertions carry it and each covers what the others cannot: the byte oracle
/// proves nothing was lost or reordered; `backpressure_waits() > 0` proves the ring
/// really was driven into the state under test (a run where the consumer kept up
/// would otherwise pass while proving nothing); and `wait_timeouts() == 0` states
/// that every wait was RELEASED by a commit rather than expiring into the lapping
/// fallback.
#[test]
fn a_slow_recorder_makes_the_state_writer_wait_and_the_whole_anchor_survives() {
    const CAP: u32 = 8;
    /// 64 records through an 8-record ring — the writer must block repeatedly.
    const BLOB_BYTES: usize = STATE_RECORD_PAYLOAD * 64;
    /// The recorder pauses between batches, so the writer is GUARANTEED to fill the
    /// ring: it pushes 8 records in microseconds while the recorder takes this long
    /// to free them. Load can only widen that gap.
    const RECORDER_PAUSE: Duration = Duration::from_micros(200);

    let want = blob(BLOB_BYTES);
    let mut owner =
        StateRingOwner::create(&tag("bp_gate"), CAP, 0, 77, &["giant"]).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");

    // Open the consumer BEFORE any push: its published cursor is what the producer
    // waits on, so attaching up front removes a startup race from the arm entirely.
    let mut consumer = StateRingConsumer::open(&name).expect("open");

    let expected = want.clone();
    let recorder = std::thread::spawn(move || {
        let mut asm = StateAssembler::passthrough();
        let mut events: Vec<StateAnchorEvent> = Vec::new();
        let start = Instant::now();
        while events.is_empty() {
            assert!(
                start.elapsed() < DEADLINE,
                "the anchor never completed within {DEADLINE:?} — read_cursor={}, \
                 write_cursor={}, available={}, open_anchors={} (this is a FAILURE, \
                 not a hang)",
                consumer.read_cursor(),
                consumer.write_cursor(),
                consumer.available(),
                asm.open_anchors(),
            );
            let n = consumer
                .drain(&mut asm, &mut events)
                .expect("a backpressure writer must never lap this recorder");
            if n == 0 {
                std::thread::yield_now();
                continue;
            }
            std::thread::sleep(RECORDER_PAUSE);
        }
        (events, asm.torn_drains())
    });

    let mut sink = producer.sink(9, 0);
    sink.append(&want);
    let parts = sink.finish();
    assert_eq!(parts, 64);

    let (events, torn_drains) = recorder.join().expect("recorder thread");
    assert_eq!(torn_drains, 0, "nothing was lapped mid-read");
    assert_eq!(
        events,
        vec![StateAnchorEvent::Complete {
            run_id: 77,
            step: 9,
            node_idx: 0,
            parts: 64,
            bytes: expected,
        }],
        "backpressure delivered a {}-record anchor through a {CAP}-record ring",
        parts
    );
    assert!(
        producer.backpressure_waits() > 0,
        "ANTI-VACUITY: the ring must really have filled — a run where the recorder \
         kept up proves nothing about waiting (waits={}, wait_nanos={})",
        producer.backpressure_waits(),
        producer.backpressure_wait_nanos(),
    );
    assert_eq!(
        producer.backpressure_wait_timeouts(),
        0,
        "every wait was RELEASED by a commit, never expired into the lapping fallback"
    );
    drop(owner);
}

/// The other side of the same contract: a WEDGED recorder is a LOUD lap, never
/// silence.
///
/// A wait that expires writes anyway (`shm_ring` contract 6 — dropping the record
/// would be silent, while lapping is caught by the consumer), so the degradation is
/// counted on the writer AND reported on the reader. Without this arm, "backpressure
/// never loses anything" would be an unbounded promise the design does not make.
#[test]
fn a_wedged_recorder_degrades_to_a_loud_lap_never_to_silence() {
    const CAP: u32 = 8;
    const RECORDS: usize = 12;
    /// Short on purpose: this arm bounds a WEDGE, and the shipped 5 s ceiling would
    /// make it a 20 s test for no extra property.
    const CEILING: Duration = Duration::from_millis(50);

    let mut owner = StateRingOwner::create(&tag("wedged"), CAP, 0, 1, &["n"]).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    producer.set_backpressure_wait_timeout(CEILING);

    // Nobody ever drains: the published read cursor stays at 0.
    let mut sink = producer.sink(1, 0);
    sink.append(&blob(STATE_RECORD_PAYLOAD * RECORDS));
    sink.finish();

    assert_eq!(
        producer.backpressure_wait_timeouts(),
        (RECORDS - CAP as usize) as u64,
        "every push past the ring's capacity waited its whole ceiling and lapped"
    );

    let mut consumer = StateRingConsumer::open(&name).expect("open");
    let mut asm = StateAssembler::passthrough();
    let mut out = Vec::new();
    match consumer.drain(&mut asm, &mut out) {
        Err(StateRingError::Ring(e)) => {
            let msg = e.to_string();
            assert!(
                msg.contains("overrun"),
                "the lap must be reported LOUDLY: {msg}"
            );
        }
        other => panic!("expected a loud Overrun, got {other:?}"),
    }
    assert!(out.is_empty(), "a torn drain hands on no events");
    assert_eq!(
        asm.torn_drains(),
        1,
        "the assembler records that its reassembly has a hole the record stream \
         itself cannot show"
    );
    assert_eq!(asm.open_anchors(), 0, "every anchor in flight was dropped");
    drop(owner);
}

// ===========================================================================
// 4. The mid-run attach
// ===========================================================================

/// A recorder attaching MID-ANCHOR lands inside a blob whose head it will never see.
/// An armed assembler DISCARDS that partial head instead of reporting corruption, and
/// assembles the NEXT anchor whole.
#[test]
fn a_mid_run_attach_discards_the_partial_head_anchor_and_takes_the_next_whole() {
    let head = blob(STATE_RECORD_PAYLOAD * 4);
    let next = blob(700);

    let mut owner = StateRingOwner::create(&tag("midrun"), 1_024, 0, 3, &["n"]).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");

    // Two records of the head anchor land BEFORE the recorder exists.
    let mut sink = producer.sink(1, 0);
    sink.append(&head[..STATE_RECORD_PAYLOAD * 2]);
    assert_eq!(sink.parts_emitted(), 1);

    let mut consumer = StateRingConsumer::open_at_live(&name).expect("open_at_live");
    // …and the rest arrives after it attached.
    sink.append(&head[STATE_RECORD_PAYLOAD * 2..]);
    sink.finish();
    let mut sink = producer.sink(2, 0);
    sink.append(&next);
    sink.finish();

    let mut asm = StateAssembler::armed();
    let mut events = drain_all(&mut consumer, &mut asm);
    events.extend(asm.finish());
    assert!(
        asm.discarded() > 0,
        "ANTI-VACUITY: the attach must really have landed mid-anchor"
    );
    assert!(!asm.is_armed(), "the first part-0 record opens it for good");
    assert_eq!(
        events,
        vec![StateAnchorEvent::Complete {
            run_id: 3,
            step: 2,
            node_idx: 0,
            parts: 2,
            bytes: next,
        }],
        "the headless anchor is discarded, not reported torn, and the next is whole"
    );
    drop(owner);
}

// ===========================================================================
// 5. resync_after_fork — the shipping shape, and only a REAL fork can model it
// ===========================================================================

/// BOUNDED `waitpid`, so a wedged child is an attributable red rather than a hang.
/// SIGKILLs and reaps on expiry.
fn reap_bounded(pid: libc::pid_t) -> libc::c_int {
    let start = Instant::now();
    loop {
        let mut status: libc::c_int = 0;
        // SAFETY: FFI waitpid on a child this test forked; WNOHANG never blocks.
        let r = unsafe { libc::waitpid(pid, &mut status, libc::WNOHANG) };
        if r == pid {
            return status;
        }
        assert!(
            r >= 0,
            "waitpid({pid}): {}",
            std::io::Error::last_os_error()
        );
        if start.elapsed() >= DEADLINE {
            // SAFETY: kill + blocking reap of our own child; no orphan is left.
            unsafe {
                libc::kill(pid, libc::SIGKILL);
                let mut s: libc::c_int = 0;
                libc::waitpid(pid, &mut s, 0);
            }
            panic!("the fork child did not exit within {DEADLINE:?} — it was killed and reaped");
        }
        std::thread::sleep(Duration::from_millis(2));
    }
}

/// THE COMPOSITION: the checkpoint CHILD takes the producer role, streams a
/// whole anchor straight into the `MAP_SHARED` segment, and `_exit`s; the parent
/// resyncs on reap and writes its own anchor where the child left off. Both come back
/// whole from one consumer.
///
/// A thread cannot model this. `fork` duplicates the producer's LOCAL write cursor
/// while the ring pages are SHARED, so without
/// [`resync_after_fork`](cerulion_core::state_ring::StateRingProducer::resync_after_fork)
/// the parent's next push writes at a stale index — over the child's records — and
/// `Release`-stores a cursor that goes BACKWARDS. (`shm_ring`'s own test file
/// reproduces that corruption; here the property under test is that a whole ANCHOR
/// composes across the seam.)
///
/// The child's body is sink writes and `_exit` and nothing else — no allocation, no
/// lock, no destructor — which is both what makes it sound after `fork` and exactly
/// the shape a fork-safe child must have. `_exit` (not `exit`, not a return) runs no `atexit`
/// handler and no `Drop`, so the child never `shm_unlink`s the parent's ring.
#[test]
fn a_forked_child_streams_a_whole_anchor_and_the_parent_resyncs_before_its_own() {
    let child_state = blob(1_500); // 4 records
    let parent_state = blob(600); // 2 records

    let mut owner = StateRingOwner::create(
        &tag("fork"),
        1_024,
        0,
        0xC0FFEE,
        &["child_node", "parent_node"],
    )
    .expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");

    // SAFETY: the child branch below touches only mapped memory and `_exit`s.
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork: {}", std::io::Error::last_os_error());
    if pid == 0 {
        let mut sink = producer.sink(100, 0);
        for piece in child_state.chunks(97) {
            sink.append(piece);
        }
        sink.finish();
        // SAFETY: leave immediately — no unwinding into the parent's frames, no
        // `Drop` for the inherited owner (which would unlink the parent's ring).
        unsafe { libc::_exit(0) };
    }

    let status = reap_bounded(pid);
    assert!(
        libc::WIFEXITED(status) && libc::WEXITSTATUS(status) == 0,
        "the fork child must exit 0 (status {status})"
    );

    // THE HAZARD, stated: the parent's LOCAL cursor did not move while the SHARED one
    // did. A push from here would write at a stale index.
    assert_eq!(
        producer.pushed(),
        0,
        "fork duplicated the local cursor; the child's pushes are invisible to it"
    );
    let resynced = producer.resync_after_fork();
    assert_eq!(resynced, 4, "resync reloads the cursor the CHILD published");

    let mut sink = producer.sink(100, 1);
    sink.append(&parent_state);
    sink.finish();

    let mut consumer = StateRingConsumer::open(&name).expect("open");
    let mut asm = StateAssembler::passthrough();
    let events = drain_all(&mut consumer, &mut asm);
    assert_eq!(
        events,
        vec![
            StateAnchorEvent::Complete {
                run_id: 0xC0FFEE,
                step: 100,
                node_idx: 0,
                parts: 4,
                bytes: child_state,
            },
            StateAnchorEvent::Complete {
                run_id: 0xC0FFEE,
                step: 100,
                node_idx: 1,
                parts: 2,
                bytes: parent_state,
            },
        ],
        "one gap-free stream across the fork: the child's anchor, then the parent's \
         written where the child left off"
    );
    drop(owner);
}

// ===========================================================================
// Taking over an arm tag clears the rings a CRASHED
// earlier run left under it.
// ===========================================================================

/// The recorder's own probe, applied to one arm tag's rank space — i.e. exactly
/// what a recorder handed `arm_tag` could ADOPT.
fn adoptable_ranks(arm_tag: &str) -> Vec<u32> {
    scan_state_ring_ranks(|rank| {
        state_ring_shm_name(arm_tag, rank)
            .map(|n| cerulion_core::shm_ring::shm_object_exists(&n))
            .unwrap_or(false)
    })
}

/// Plant a ring the way a CRASH leaves one: create it, then leak the owner so its
/// `Drop` never `shm_unlink`s the name.
fn plant_orphan_ring(arm_tag: &str, rank: u32) {
    let ring_tag = state_ring_tag(arm_tag, rank).expect("nameable rank");
    let owner = StateRingOwner::create(&ring_tag, 8, rank, 0xDEAD_BEEF, &["n0"])
        .expect("plant the orphan ring");
    std::mem::forget(owner);
}

/// Remove a planted orphan the sweep is not expected to reach, so the test leaves
/// no `/dev/shm` object behind: creating over the name is unlink-first, and this
/// owner's `Drop` unlinks it for real.
fn remove_orphan_ring(arm_tag: &str, rank: u32) {
    let ring_tag = state_ring_tag(arm_tag, rank).expect("nameable rank");
    drop(StateRingOwner::create(&ring_tag, 8, rank, 0, &["n0"]).expect("re-create to clean up"));
}

/// A leaked ring is a DEAD run's node state wearing this
/// run's identity, and taking over the tag must remove it.**
///
/// `MappedStateArm::create_owned` is unlink-first, so arming under a tag already
/// asserts ownership of it — it destroys whatever word was there. The per-rank ring
/// names under that tag are the rest of the namespace and the half that carries
/// DATA, and they were left standing.
///
/// The two planted ranks are the two shapes that actually happen under a reused
/// explicit `CERULION_STATE_ARM_TAG`:
///
/// * rank 0 — a rank the next run HAS, whose ring creation it may nonetheless
///   refuse (the per-worker RAM gate); the deployment is meant to report that
///   as a HOLE, and instead the hole is filled with a dead run's records;
/// * rank 3 — a rank the next run does not have at all (the earlier run was wider,
///   or multi-process where this one is a monolith).
///
/// Nothing downstream can reject either: `state_ring_run_id` is derived from the
/// TAG, so both carry a `run_id` IDENTICAL to the new run's.
#[test]
fn taking_over_an_arm_tag_removes_the_rings_a_crashed_run_left_under_it() {
    let t = tag("stale_takeover");
    plant_orphan_ring(&t, 0);
    plant_orphan_ring(&t, 3);

    // ANTI-TAUTOLOGY: without this, "the sweep left nothing adoptable" is satisfied
    // by a planting helper that planted nothing.
    assert_eq!(
        adoptable_ranks(&t),
        vec![0, 3],
        "precondition: a crashed run really does leave rings a recorder can adopt"
    );

    let sweep = unlink_stale_state_rings(&t);
    assert_eq!(sweep.removed, vec![0, 3], "both leaks reported as removed");
    assert!(sweep.refused.is_empty(), "{:?}", sweep.refused);
    assert!(sweep.found_anything());

    // THE ORACLE: a recorder handed this tag can now adopt NOTHING.
    assert!(
        adoptable_ranks(&t).is_empty(),
        "a recorder armed with `{t}` must find no ring this run did not create"
    );

    // IDEMPOTENT, and the ordinary case: the second take-over finds nothing and
    // says so, rather than reporting phantom removals.
    let again = unlink_stale_state_rings(&t);
    assert_eq!(again, cerulion_core::state_ring::StaleRingSweep::default());
    assert!(!again.found_anything());
}

/// The sweep clears exactly the set the RECORDER can reach — including the blind
/// spot, which both halves share.
///
/// `scan_state_ring_ranks` stops after `STATE_RING_PROBE_GAP_TOLERANCE` consecutive
/// misses, so a leak that far above the last live rank is invisible to the recorder
/// too. Pinned rather than left implicit: the sweep must never be WEAKER than the
/// reader it protects, and "equal" is the claim being made.
#[test]
fn the_sweep_clears_exactly_what_a_recorder_could_reach() {
    let t = tag("stale_reach");
    let unreachable = STATE_RING_PROBE_GAP_TOLERANCE + 1;
    plant_orphan_ring(&t, 0);
    plant_orphan_ring(&t, unreachable);

    // The reader's own reach, measured rather than assumed: rank `unreachable`
    // exists and the recorder still cannot see it.
    assert_eq!(
        adoptable_ranks(&t),
        vec![0],
        "precondition: the recorder's sweep itself stops before rank {unreachable}"
    );

    let sweep = unlink_stale_state_rings(&t);
    assert_eq!(
        sweep.removed,
        vec![0],
        "the sweep removes what the recorder could have adopted, and stops where it does"
    );
    assert!(
        cerulion_core::shm_ring::shm_object_exists(
            &state_ring_shm_name(&t, unreachable).expect("nameable")
        ),
        "…and the shared blind spot is stated, not papered over"
    );
    remove_orphan_ring(&t, unreachable);
}
