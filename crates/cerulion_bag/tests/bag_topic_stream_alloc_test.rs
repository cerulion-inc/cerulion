// SPDX-License-Identifier: AGPL-3.0-only
//! `BagReader::recover_messages_on_topic`
//! reads ONE channel out of a bag without paying for the rest of it.
//!
//! # Why the oracle is ALLOCATION
//!
//! `recover_messages_on_topic` is **output-equivalent** to
//! `recover_messages().filter(|m| m.topic == t)`. Every behavioural assertion a
//! caller could make — which messages, in what order, with what bytes — is
//! satisfied by both, which is not a guess: with the topic filter deleted
//! (`m.channel.topic == topic || true`) the ENTIRE 151-test
//! `replay_engine_test` suite still passes. So a behavioural test cannot see
//! the property this function exists for, and the usable oracle is what it
//! COSTS.
//!
//! What it costs is a copy: `BagMessage` owns its payload
//! (`m.data.into_owned()`), so collecting first and filtering after copies every
//! byte of every channel. `read_bag_anchors` reads a few hundred kilobytes of
//! `__cerulion/state` records out of recordings that are routinely gigabytes of
//! camera frames, so on a collect-then-filter path a mid-run replay's peak memory is the
//! size of the bag before one frame of it is needed.
//!
//! PEAK LIVE bytes, and the distinction is the whole measurement. A probe
//! that counts bytes REQUESTED reports the two paths as
//! near-identical (3,207,857 B vs 3,188,753 B) — a true number about the wrong
//! quantity. `mcap::MessageStream` materialises one CHUNK at a time and frees
//! it before the next, so cumulative demand is ~the bag either way; what
//! separates them is what is live AT ONCE. `recover_messages` holds every
//! message simultaneously (peak = the bag), while a filtered stream holds one
//! chunk plus what the caller keeps (peak = a chunk). The hazard is
//! "can exhaust memory", so peak is the quantity, and the probe subtracts on
//! `dealloc` and tracks a high-water mark.
//!
//! The fixture writes MANY chunks, because that is what a real recording is
//! (a chunk closes at 4 MiB or 1000 ms, so a gigabyte bag carries
//! hundreds) and because with one chunk the two paths genuinely do cost the
//! same — a single-chunk fixture would have made this arm assert a property
//! the code does not have.
//!
//! The CONTROL arm is what makes the filtered arm mean anything: it drives
//! `recover_messages` over the same bag and requires its peak to reach the
//! off-topic payload total, proving the probe SEES payload retention at all. A
//! broken probe would report a small number for both and pass a filtered-only
//! assertion.
//!
//! Own binary: `#[global_allocator]` is process-wide. Exactly ONE test here
//! opens a measurement window and the counter is thread-scoped, so its sibling
//! cannot perturb it — no `#[serial]` is owed, and adding a dev-dep for one
//! would be the wrong trade. Parallel-safe against other binaries (per-test
//! temp path, no transport).

#![cfg(unix)]

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicI64, Ordering};

use cerulion_bag::{BagReader, BagWriter, BagWriterConfig, TopicSchema, STATE_TOPIC};
use cerulion_core::state_ring::{encode_record, StateRecordHeader, RECORD_KIND_FINAL};

thread_local! {
    static MEASURED_THREAD: Cell<bool> = const { Cell::new(false) };
}

/// Tracks the PEAK heap bytes live on the measured thread while a window is
/// open: `alloc` adds, `dealloc` subtracts, and a high-water mark is kept.
///
/// `live` is SIGNED because a buffer allocated before the window can be freed
/// inside it, which drives the running total below zero. That only ever
/// understates the peak, so it is safe for a lower-bound control and for a
/// ceiling on the filtered walk; a saturating unsigned counter would instead pin the
/// total at zero and silently flatten the measurement.
///
/// Thread-scoped for the reason documented in `shm_ring_zero_alloc_test`:
/// libtest machinery and platform lazy-init allocate on other threads at times
/// this test does not control.
struct CountingAllocator {
    inner: System,
    live: AtomicI64,
    peak: AtomicI64,
    enabled: AtomicBool,
}

impl CountingAllocator {
    const fn new() -> Self {
        Self {
            inner: System,
            live: AtomicI64::new(0),
            peak: AtomicI64::new(0),
            enabled: AtomicBool::new(false),
        }
    }

    fn enable(&self) {
        MEASURED_THREAD.set(true);
        self.live.store(0, Ordering::SeqCst);
        self.peak.store(0, Ordering::SeqCst);
        self.enabled.store(true, Ordering::SeqCst);
    }

    fn disable(&self) -> i64 {
        self.enabled.store(false, Ordering::SeqCst);
        MEASURED_THREAD.set(false);
        self.peak.load(Ordering::SeqCst)
    }

    #[inline]
    fn note(&self, delta: i64) {
        if self.enabled.load(Ordering::Relaxed) {
            // `try_with`, never `with`: during thread teardown TLS is
            // inaccessible and `with` would panic INSIDE the allocator.
            if MEASURED_THREAD.try_with(Cell::get).unwrap_or(false) {
                let now = self.live.fetch_add(delta, Ordering::Relaxed) + delta;
                self.peak.fetch_max(now, Ordering::Relaxed);
            }
        }
    }
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        self.note(layout.size() as i64);
        unsafe { self.inner.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        self.note(-(layout.size() as i64));
        unsafe { self.inner.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        self.note(new_size as i64 - layout.size() as i64);
        unsafe { self.inner.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator::new();

/// One off-topic frame. Big enough that the payload copies dwarf every fixed
/// cost of a walk (topic strings, iterator state), small enough to stay instant.
const FRAME_BYTES: usize = 32 * 1024;
/// Off-topic frames. `FRAMES * FRAME_BYTES` = 3 MiB of payload the anchor
/// reader has no use for — the bytes a collect-then-filter walk copies.
const FRAMES: usize = 96;
/// The state records the reader actually wants. Three, so the arm is a genuine
/// filter rather than a "find one message" special case.
const STATE_RECORDS: usize = 3;
/// Force MANY chunks out of a 3 MiB fixture. A real recording is chunked (4 MiB
/// or 1000 ms) and a streaming reader's peak is ONE chunk; a fixture
/// that fitted in the 4 MiB default would have exactly one, where the two paths
/// legitimately cost the same.
const CHUNK_BYTES: usize = 128 * 1024;

const USER_TOPIC: &str = "/camera/image";
const USER_TOTAL: u64 = (FRAMES * FRAME_BYTES) as u64;

fn tmp(tag: &str) -> PathBuf {
    let mut p = std::env::temp_dir();
    p.push(format!(
        "topic_stream_alloc_{tag}_{}_{}.mcap",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    p
}

/// A bag carrying `FRAMES` large user frames and `STATE_RECORDS` small anchors
/// on the reserved state channel — the shape of every real checkpoint bag.
fn write_mixed_bag(path: &std::path::Path) {
    let mut w = BagWriter::create(
        path,
        BagWriterConfig {
            chunk_max_bytes: CHUNK_BYTES,
            ..BagWriterConfig::default()
        },
        &[TopicSchema {
            topic: USER_TOPIC.into(),
            schema_name: "sensor_msgs/Image".into(),
            schema_hash: 0x1111_2222_3333_4444,
            wire_fixed_size: FRAME_BYTES as u32,
        }],
    )
    .unwrap();
    let state_id = w.state_channel_id();
    let frame = vec![7u8; FRAME_BYTES];
    let records: Vec<Vec<u8>> = (0..STATE_RECORDS)
        .map(|i| {
            encode_record(
                &StateRecordHeader {
                    run_id: 0xDEAD_BEEF,
                    step: 40 + i as u64,
                    node_idx: 0,
                    part: 0,
                    kind: RECORD_KIND_FINAL,
                    len: 8,
                },
                &(i as u64).to_le_bytes(),
            )
            .to_vec()
        })
        .collect();

    // INTERLEAVED, so the state records are spread across chunks rather than
    // sitting in a prefix a reader could reach without walking the user frames.
    let stride = FRAMES / STATE_RECORDS;
    w.write_chunk(|s| {
        for i in 0..FRAMES {
            s.write_message(
                USER_TOPIC,
                i as u32,
                1_000 + i as u64,
                1_000 + i as u64,
                &[&frame],
            )?;
            if i % stride == 0 {
                let rec = &records[i / stride];
                s.write_message(
                    state_id,
                    i as u32,
                    2_000 + i as u64,
                    2_000 + i as u64,
                    &[rec],
                )?;
            }
        }
        Ok(())
    })
    .unwrap();
    w.finalize().unwrap();
}

/// THE arm: reading one channel must not cost the whole bag.
///
/// Both halves run over the SAME reader, so the file bytes are already resident
/// and out of both windows — what is measured is purely what each API asks the
/// allocator for on top of them.
#[test]
fn reading_one_channel_does_not_pay_for_the_rest_of_the_bag() {
    let p = tmp("stream");
    write_mixed_bag(&p);
    let r = BagReader::open(&p).unwrap();

    // CONTROL — collect-then-filter. Requests at least every payload byte,
    // because `BagMessage` owns its data. Without this arm a broken probe that
    // measured nothing would pass the filtered-walk assertion below.
    ALLOCATOR.enable();
    let (all, _completeness) = r.recover_messages().unwrap();
    let collected = all.iter().filter(|m| m.topic == STATE_TOPIC).count();
    let control_peak = ALLOCATOR.disable();

    // FIX — filtered inside the reader, on the borrowed message.
    ALLOCATOR.enable();
    let streamed: Vec<_> = r
        .recover_messages_on_topic(STATE_TOPIC)
        .unwrap()
        .collect::<Result<Vec<_>, _>>()
        .unwrap();
    let stream_peak = ALLOCATOR.disable();

    println!(
        "peak live: recover_messages {control_peak} B; recover_messages_on_topic \
         {stream_peak} B; off-topic payload total {USER_TOTAL} B; chunk {CHUNK_BYTES} B"
    );

    // Same answer both ways — the filtered walk is a cost change, not a behaviour change.
    assert_eq!(
        collected, STATE_RECORDS,
        "the fixture writes 3 state records"
    );
    assert_eq!(
        streamed.len(),
        STATE_RECORDS,
        "the streaming reader must yield exactly the state channel's messages"
    );
    for m in &streamed {
        assert_eq!(
            m.topic, STATE_TOPIC,
            "an off-topic message reached the caller — the filter is inside the reader"
        );
    }

    assert!(
        control_peak >= USER_TOTAL as i64,
        "the probe must SEE payload retention: collect-then-filter peaked at {control_peak} B, \
         under the {USER_TOTAL} B of payload it holds live by construction"
    );
    // The claim is "peak does not scale with the bag, it scales with a CHUNK".
    // The bound is stated in chunks rather than as a fraction of the bag, since
    // that is the shape of the guarantee: a bag ten times larger moves
    // USER_TOTAL and leaves this ceiling where it is.
    let ceiling = 4 * CHUNK_BYTES as i64;
    assert!(
        stream_peak < ceiling,
        "reading one channel peaked at {stream_peak} B against a {ceiling} B ceiling (4 chunks) \
         and {USER_TOTAL} B of off-topic payload — it is holding channels it filtered out"
    );
}

/// A torn tail still yields what precedes it, then reports the tear — the
/// recovery half of `recover_messages`, preserved by the streaming twin.
///
/// # The oracle is the SIBLING API over the same truncated bytes
///
/// `recover_messages_on_topic` is output-equivalent to
/// `recover_messages().filter(..)` (see the module docs), and a tear must not
/// change that: the filtered stream owes the SAME state-channel prefix, byte
/// for byte, and then the SAME tear. Both open one `mcap::MessageStream` with
/// the same options, so they fault at the same record — which is what makes the
/// control an exact expectation rather than an approximation, and what keeps
/// the arm from hard-coding a count that moves with the fixture's size or the
/// truncation ratio.
///
/// A `yielded > 0 || tore` disjunction (a weaker oracle) passes for a
/// reader that yields any prefix and then STOPS CLEANLY, swallowing the
/// corruption. That is the reading `read_bag_anchors` acts on, so it is pinned
/// here rather than left to the caller.
#[test]
fn a_torn_bag_yields_its_readable_prefix_then_reports_the_tear() {
    let p = tmp("torn");
    write_mixed_bag(&p);
    let mut bytes = std::fs::read(&p).unwrap();
    // Lop off the finalization epilogue AND part of the last chunk, so the
    // stream ends mid-record rather than at a clean boundary.
    bytes.truncate(bytes.len() * 3 / 4);
    let r = BagReader::from_bytes(bytes);

    let (control, completeness) = r.recover_messages().expect("the control stream opens");
    assert!(
        matches!(completeness, cerulion_bag::BagCompleteness::TornTail(_)),
        "fixture precondition: the truncation must land MID-RECORD, so the bag really is torn \
         rather than cut at a chunk boundary — got {completeness:?}"
    );
    let expected: Vec<&cerulion_bag::BagMessage> =
        control.iter().filter(|m| m.topic == STATE_TOPIC).collect();
    assert!(
        !expected.is_empty(),
        "fixture precondition: the readable prefix must carry state records, or 'yields its \
         readable prefix' is a claim about an empty set"
    );

    let mut yielded: Vec<cerulion_bag::BagMessage> = Vec::new();
    let mut tore = false;
    for item in r.recover_messages_on_topic(STATE_TOPIC).unwrap() {
        match item {
            Ok(m) => {
                assert_eq!(m.topic, STATE_TOPIC);
                yielded.push(m);
            }
            Err(_) => {
                tore = true;
                break;
            }
        }
    }

    assert_eq!(
        yielded.len(),
        expected.len(),
        "the filtered stream must yield EXACTLY the state-channel prefix the unfiltered reader \
         recovers from the same torn bytes — a short read is a silently swallowed message"
    );
    for (got, want) in yielded.iter().zip(&expected) {
        assert_eq!(got.sequence, want.sequence, "prefix order must match");
        assert_eq!(
            got.data, want.data,
            "prefix payloads must match byte for byte"
        );
    }
    assert!(
        tore,
        "the tear must be REPORTED to the caller — a reader that stops cleanly after corruption \
         hands `read_bag_anchors` a truncated anchor set that reads as 'this bag has none'"
    );
}
