// SPDX-License-Identifier: AGPL-3.0-only
//! The SHM ring's `BACKPRESSURE` overrun mode and
//! [`ShmRingProducer::resync_after_fork`].
//!
//! Two changes to a SHIPPED primitive, so the arms here are shaped around what
//! must NOT move as much as what must:
//!
//! * **`BACKPRESSURE`** — a checkpoint anchor is ~1.09 M chunked records for
//!   500 MB of node state, so it cannot fit a fixed ring, and a lapped anchor is a
//!   LOST anchor. The writer is a short-lived `fork` child, not a hot loop, so it
//!   can afford to wait. The arms prove the producer WAITS rather than lapping, that
//!   a drain-side commit RELEASES it, that the wait observables count exactly the
//!   pushes that waited, and — the other half — that a `FailLoud` (i.e. every trace)
//!   ring is untouched: its policy word stays 0, its read-cursor word is never
//!   written, and it still laps with the same exact `records_lost` arithmetic.
//! * **`resync_after_fork`** — `fork` duplicates the producer's LOCAL write
//!   cursor while the ring pages are `MAP_SHARED` and are not copied, so a parent
//!   that pushes after a child did overwrites the child's records and publishes a
//!   cursor that goes BACKWARDS. The arms use a REAL `fork(2)`, which is the only
//!   way to reproduce that: a spawned process has no copy of the parent's producer.
//!   The child's whole body is `push` + `_exit`, which is exactly the "no alloc, no
//!   lock, no syscall" shape the ring's own design contract promises for post-fork
//!   code.
//!
//! Real POSIX SHM on macOS AND Linux, no iceoryx2, unique per-test ring tags — so
//! the file is parallel-safe and needs no `#[serial]` / `--test-threads=1`. The pure
//! decision oracles (the wait predicate at both sides of its threshold, the policy
//! wire round-trip, the header offsets) live as in-module unit tests in
//! `shm_ring.rs`; this file is the behavioural half.
//!
//! Every cross-thread and cross-process wait here is bounded by a GENEROUS liveness
//! deadline: the work it bounds is milliseconds, so seconds only decide how long a
//! WEDGE takes to report itself. An unbounded wait would turn a real failure into a
//! hang, and a hung test burns a CI job's whole timeout with no attributable red.

#![cfg(unix)]

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::shm_ring::{
    OverrunPolicy, ShmRingConsumer, ShmRingError, ShmRingOwner, ShmRingProducer,
    HEADER_OFF_OVERRUN_POLICY, HEADER_OFF_READ_CURSOR,
};
use cerulion_core::trace_ring::{
    TraceRingConsumer, TraceRingOwner, TraceRingRecord, RECORD_TYPE_FIRE,
};

// ===========================================================================
// Shared helpers
// ===========================================================================

/// Unique ring tag per test (name + pid) so parallel tests / re-runs never collide
/// on a `/dev/shm` object.
fn tag(name: &str) -> String {
    format!("{name}_{}", std::process::id())
}

/// GENEROUS liveness ceiling for every cross-thread / cross-process rendezvous in
/// this file. Never a wall stated in units of the thing under test.
const DEADLINE: Duration = Duration::from_secs(30);

/// Push one 8-byte little-endian sequence number. The record body IS its own
/// oracle: a drained value must equal its ring index, so a corrupted or reordered
/// stream is visible without a side table.
fn push_seq(p: &mut ShmRingProducer, s: u64) {
    p.push(&s.to_le_bytes());
}

/// Drain an 8-byte-record ring into a `Vec<u64>` and commit.
fn drain_seqs(c: &mut ShmRingConsumer) -> Result<Vec<u64>, ShmRingError> {
    let v = {
        let (a, b) = c.drain_slices()?;
        let mut v = Vec::new();
        for chunk in a.as_chunks::<8>().0.iter().chain(b.as_chunks::<8>().0) {
            v.push(u64::from_le_bytes(*chunk));
        }
        v
    };
    c.commit(v.len() as u64)?;
    Ok(v)
}

/// A raw read-only view of a live ring segment, for reading header words the typed
/// API deliberately does not expose on every layer (the trace consumer has no
/// `published_read_cursor`). `Drop` `munmap`s.
struct RawMap {
    ptr: *mut u8,
    len: usize,
}
impl Drop for RawMap {
    fn drop(&mut self) {
        // SAFETY: unmap exactly the region `open_raw` mapped.
        unsafe {
            libc::munmap(self.ptr as *mut libc::c_void, self.len);
        }
    }
}
impl RawMap {
    fn u32_at(&self, off: usize) -> u32 {
        assert!(off + 4 <= self.len);
        // SAFETY: `off + 4 <= len` and the mapping is readable.
        unsafe { std::ptr::read_unaligned(self.ptr.add(off) as *const u32) }
    }
    fn u64_at(&self, off: usize) -> u64 {
        assert!(off + 8 <= self.len);
        // SAFETY: `off + 8 <= len` and the mapping is readable.
        unsafe { std::ptr::read_unaligned(self.ptr.add(off) as *const u64) }
    }
}

fn open_raw(name: &str) -> RawMap {
    let cname = std::ffi::CString::new(name).unwrap();
    // SAFETY: FFI open of an existing named SHM object; `cname` is valid.
    let fd = unsafe { libc::shm_open(cname.as_ptr(), libc::O_RDONLY, 0) };
    assert!(
        fd >= 0,
        "open_raw shm_open({name}): {}",
        std::io::Error::last_os_error()
    );
    // SAFETY: zeroed stat filled by fstat on our fd.
    let mut st: libc::stat = unsafe { std::mem::zeroed() };
    assert_eq!(unsafe { libc::fstat(fd, &mut st) }, 0);
    let len = st.st_size as usize;
    // SAFETY: map the object read-only.
    let addr = unsafe {
        libc::mmap(
            std::ptr::null_mut(),
            len,
            libc::PROT_READ,
            libc::MAP_SHARED,
            fd,
            0,
        )
    };
    // SAFETY: `fd` is mapped now (or the map failed); either way we own it.
    unsafe {
        libc::close(fd);
    }
    assert_ne!(addr, libc::MAP_FAILED, "open_raw mmap({name})");
    RawMap {
        ptr: addr as *mut u8,
        len,
    }
}

/// Spin on `flag` until set, or panic ATTRIBUTABLY at the deadline naming `what`.
fn await_flag(flag: &AtomicBool, what: &str) {
    let start = Instant::now();
    while !flag.load(Ordering::SeqCst) {
        assert!(
            start.elapsed() < DEADLINE,
            "{what} never happened within {DEADLINE:?} — the peer is gone or wedged \
             (this is a FAILURE, not a hang)"
        );
        std::thread::yield_now();
    }
}

/// A hand-built trace record whose payload is its own index.
fn trace_record(i: u64) -> TraceRingRecord {
    TraceRingRecord {
        step: i,
        fire_time_ns: 1000 + i * 7,
        duration_ns: 3 * i,
        node_idx: (i % 3) as u32,
        global_level: (i % 2) as u32,
        record_type: RECORD_TYPE_FIRE,
        reserved: 0,
    }
}

// ===========================================================================
// 1. BACKPRESSURE: the producer waits instead of lapping, and NOTHING is lost
// ===========================================================================

/// The headline. A producer streaming `N` records through a ring that holds EIGHT,
/// against a consumer that is deliberately slower than it, must deliver every
/// record in order — which under `FailLoud` is impossible by construction (the
/// sibling control below laps by `N - capacity`).
///
/// Two assertions carry it, and each covers what the other cannot: the exact
/// sequence oracle proves nothing was lost or reordered, and
/// `backpressure_waits() > 0` proves the ring really was driven into the state
/// under test — without it a run where the consumer happened to keep up would pass
/// while proving nothing. `wait_timeouts() == 0` states the third fact: it waited
/// and was RELEASED, never expired into the lapping fallback.
#[test]
fn a_backpressure_producer_waits_instead_of_lapping_and_the_stream_is_gap_free() {
    const N: u64 = 1_000;
    const CAP: u32 = 8;
    /// The consumer pauses between batches, so the producer is GUARANTEED to fill
    /// the 8-slot ring and block: it pushes 8 records in nanoseconds while the
    /// consumer takes this long to free them. Load can only widen that gap.
    const CONSUMER_PAUSE: Duration = Duration::from_micros(200);

    let mut owner = ShmRingOwner::create_with_policy(
        &tag("bp_gapfree"),
        8,
        CAP,
        0,
        &[],
        OverrunPolicy::Backpressure,
    )
    .expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");

    // Open the consumer BEFORE any push: its published cursor is what the producer
    // waits on, so attaching it up front removes a startup race from the arm
    // entirely (the mid-stream attach is its own test below).
    let mut consumer = ShmRingConsumer::open(&name).expect("open");

    let ch = std::thread::spawn(move || {
        let mut got: Vec<u64> = Vec::with_capacity(N as usize);
        let start = Instant::now();
        while (got.len() as u64) < N {
            assert!(
                start.elapsed() < DEADLINE,
                "only {} of {N} records arrived within {DEADLINE:?} — read_cursor={}, \
                 write_cursor={}, available={}",
                got.len(),
                c_state(&consumer).0,
                c_state(&consumer).1,
                c_state(&consumer).2,
            );
            let batch = drain_seqs(&mut consumer)
                .expect("a backpressure producer must never lap this consumer");
            if batch.is_empty() {
                std::thread::yield_now();
                continue;
            }
            got.extend(batch);
            std::thread::sleep(CONSUMER_PAUSE);
        }
        got
    });

    for s in 0..N {
        push_seq(&mut producer, s);
    }
    let got = ch.join().expect("consumer thread");

    let expected: Vec<u64> = (0..N).collect();
    assert_eq!(
        got,
        expected,
        "backpressure delivered every record in order through a ring {} times smaller \
         than the stream",
        N / CAP as u64
    );
    assert!(
        producer.backpressure_waits() > 0,
        "ANTI-VACUITY: the ring must really have filled — a run where the consumer \
         kept up proves nothing about waiting (waits={}, wait_nanos={})",
        producer.backpressure_waits(),
        producer.backpressure_wait_nanos()
    );
    assert_eq!(
        producer.backpressure_wait_timeouts(),
        0,
        "every wait was RELEASED by a commit, never expired into the lapping fallback"
    );
    drop(owner);
}

/// Read a consumer's cursors without borrowing it mutably (for panic messages).
fn c_state(c: &ShmRingConsumer) -> (u64, u64, u64) {
    (c.read_cursor(), c.write_cursor(), c.available())
}

/// The CONTROL for the headline, and the behavioural half of "the default policy is
/// unchanged": the SAME script — a tiny ring, far more records than it holds — under
/// `FailLoud` laps with the exact earlier arithmetic, and its producer waits
/// ZERO times. Without this, "backpressure did not lap" could be satisfied by a ring
/// that was simply never full.
#[test]
fn a_fail_loud_producer_on_the_same_script_laps_with_exact_records_lost() {
    const CAP: u32 = 8;
    const N: u64 = 20;

    let mut owner = ShmRingOwner::create(&tag("fl_control"), 8, CAP, 0, &[]).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    assert_eq!(
        producer.overrun_policy(),
        OverrunPolicy::FailLoud,
        "`create` selects the wait-free contract — the trace ring's path"
    );

    let mut consumer = ShmRingConsumer::open(&name).expect("open");
    for s in 0..N {
        push_seq(&mut producer, s);
    }

    let err = consumer
        .drain_slices()
        .expect_err("a fail-loud producer laps a consumer that never drained");
    match err {
        ShmRingError::Overrun {
            records_lost,
            read_cursor,
            write_cursor,
            capacity,
        } => assert_eq!(
            (records_lost, read_cursor, write_cursor, capacity),
            (N - CAP as u64, 0, N, CAP as u64),
            "the earlier overrun arithmetic, unchanged"
        ),
        other => panic!("expected Overrun, got {other:?}"),
    }
    assert_eq!(
        producer.backpressure_waits(),
        0,
        "a fail-loud producer never reads consumer state, so it can never wait"
    );
    assert_eq!(producer.backpressure_wait_timeouts(), 0);
    assert_eq!(producer.backpressure_wait_nanos(), 0);
    drop(owner);
}

// ===========================================================================
// 2. The wait observables, against a hand oracle
// ===========================================================================

/// Deterministic and single-threaded: exactly which pushes wait, and what the
/// counters read afterwards.
///
/// The three phases are chosen so the counters can only be right for the right
/// reason: a ring filled EXACTLY to capacity must not wait (a predicate off by one
/// in the stalling direction fails here), a commit must restore room without any
/// wait at all, and only the push that would overwrite an uncommitted record waits.
/// That last one is driven against a SHORT ceiling so the expiry arm is reachable in
/// a test — and the expiry must still LAP, because a silent drop is the one outcome
/// this design refuses.
#[test]
fn the_wait_observables_count_only_the_pushes_that_had_to_wait() {
    const CAP: u32 = 4;
    /// Short on purpose: the arm needs a wait to EXPIRE, and a 5 s default would
    /// make that a 5 s test. It bounds the one push that has nowhere to go.
    const SHORT_CEILING: Duration = Duration::from_millis(40);

    let mut owner = ShmRingOwner::create_with_policy(
        &tag("bp_counters"),
        8,
        CAP,
        0,
        &[],
        OverrunPolicy::Backpressure,
    )
    .expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    let mut consumer = ShmRingConsumer::open(&name).expect("open");

    // First phase — fill EXACTLY to capacity. Every one of these has room by the
    // threshold rule (`local_write - read < capacity`), so none may wait.
    for s in 0..CAP as u64 {
        push_seq(&mut producer, s);
    }
    assert_eq!(
        producer.backpressure_waits(),
        0,
        "filling a ring to exactly its capacity never waits"
    );
    assert_eq!(producer.backpressure_wait_nanos(), 0);

    // Second phase — a commit frees the whole ring, and the next capacity-worth of
    // pushes must again go through with no wait.
    assert_eq!(
        drain_seqs(&mut consumer).expect("drain"),
        vec![0, 1, 2, 3],
        "the first batch is the whole ring"
    );
    assert_eq!(
        consumer.published_read_cursor(),
        CAP as u64,
        "the commit PUBLISHED the new cursor — that word is what the producer waits on"
    );
    for s in CAP as u64..2 * CAP as u64 {
        push_seq(&mut producer, s);
    }
    assert_eq!(
        producer.backpressure_waits(),
        0,
        "a committed ring has room again"
    );

    // Third phase — one more push with the ring full and nothing committed. It must
    // WAIT, and with nobody draining it must EXPIRE and lap.
    producer.set_backpressure_wait_timeout(SHORT_CEILING);
    assert_eq!(producer.backpressure_wait_timeout(), SHORT_CEILING);
    let before = Instant::now();
    push_seq(&mut producer, 2 * CAP as u64);
    let waited = before.elapsed();

    assert_eq!(
        producer.backpressure_waits(),
        1,
        "exactly ONE push had to wait across all three phases"
    );
    assert_eq!(
        producer.backpressure_wait_timeouts(),
        1,
        "with nothing draining, that wait EXPIRED"
    );
    assert!(
        waited >= SHORT_CEILING,
        "the wait really spent its ceiling (a LOWER bound — load can only lengthen \
         it): waited {waited:?} against {SHORT_CEILING:?}"
    );
    assert!(
        producer.backpressure_wait_nanos() >= SHORT_CEILING.as_nanos() as u64,
        "the counter records the time actually spent waiting: {} ns",
        producer.backpressure_wait_nanos()
    );

    // …and the expired wait LAPPED rather than dropping the record: the consumer
    // reports the loss loudly, which a silent drop never would.
    let err = consumer
        .drain_slices()
        .expect_err("an expired wait falls back to lapping, which the consumer detects");
    assert!(
        matches!(
            err,
            ShmRingError::Overrun {
                records_lost: 1,
                ..
            }
        ),
        "exactly the one record the expired wait wrote over: {err:?}"
    );
    drop(owner);
}

/// The RELEASE path: a blocked producer is freed by the drain side committing, and
/// the record it was holding lands with nothing lost.
///
/// The producer runs on the TEST thread so its counters are readable directly; the
/// consumer is the one on a thread. `wait_nanos >= RELEASE_DELAY` is a LOWER bound
/// (the producer cannot be released before the commit happens), so load can lengthen
/// it but never invert it.
#[test]
fn a_commit_releases_a_blocked_producer_without_spending_its_ceiling() {
    const CAP: u32 = 4;
    const RELEASE_DELAY: Duration = Duration::from_millis(100);

    let mut owner = ShmRingOwner::create_with_policy(
        &tag("bp_release"),
        8,
        CAP,
        0,
        &[],
        OverrunPolicy::Backpressure,
    )
    .expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    let mut consumer = ShmRingConsumer::open(&name).expect("open");

    for s in 0..CAP as u64 {
        push_seq(&mut producer, s);
    }
    assert_eq!(producer.backpressure_waits(), 0);

    let filled = Arc::new(AtomicBool::new(false));
    let filled_c = Arc::clone(&filled);
    let ch = std::thread::spawn(move || {
        await_flag(&filled_c, "the producer's fill");
        // Hold the producer blocked for a measurable, bounded stretch, then release
        // it by COMMITTING — the only act that moves the published cursor.
        std::thread::sleep(RELEASE_DELAY);
        let first = drain_seqs(&mut consumer).expect("no lap: the producer waited");
        // Then drain the record the release let through.
        let start = Instant::now();
        let mut rest = Vec::new();
        while rest.is_empty() {
            assert!(
                start.elapsed() < DEADLINE,
                "the released push never landed within {DEADLINE:?}"
            );
            rest = drain_seqs(&mut consumer).expect("no lap");
            if rest.is_empty() {
                std::thread::yield_now();
            }
        }
        let mut all = first;
        all.extend(rest);
        all
    });

    filled.store(true, Ordering::SeqCst);
    push_seq(&mut producer, CAP as u64); // blocks until the consumer commits
    let got = ch.join().expect("consumer thread");

    assert_eq!(
        producer.backpressure_waits(),
        1,
        "the one push with no room waited"
    );
    assert_eq!(
        producer.backpressure_wait_timeouts(),
        0,
        "it was RELEASED by the commit — it did not expire (its ceiling is the {:?} \
         default, far beyond the {RELEASE_DELAY:?} it actually waited)",
        producer.backpressure_wait_timeout()
    );
    assert!(
        producer.backpressure_wait_nanos() >= RELEASE_DELAY.as_nanos() as u64,
        "the producer cannot have been released before the commit that released it: \
         {} ns against {RELEASE_DELAY:?}",
        producer.backpressure_wait_nanos()
    );
    assert_eq!(
        got,
        (0..=CAP as u64).collect::<Vec<_>>(),
        "nothing was lost across the block"
    );
    drop(owner);
}

/// A consumer attaching MID-STREAM publishes its jumped cursor, which is what
/// releases a producer that filled the ring before anyone was listening.
///
/// This is the one release path a commit cannot cover: `open_at_live` sets the
/// cursor without ever committing, so a version of it that skipped publication
/// would leave the producer blocked until its ceiling expired — and then lap the
/// consumer that had just attached.
///
/// # Why the consumer must not touch `commit` before the release
///
/// The first version of this arm drained in a loop, and `commit(0)` on an EMPTY
/// batch publishes the cursor just as a real commit does — so the drain loop itself
/// released the producer and an `open_at_live` that published NOTHING passed. Found
/// The consumer now waits on `available()`, which reads the
/// producer's cursor and writes nothing, so the ONLY thing that can release the
/// producer here is the attach. The header word is also asserted directly, so the
/// claim is pinned twice over and each half is independently testable.
#[test]
fn a_live_attach_publishes_its_jumped_cursor_and_releases_a_blocked_producer() {
    const CAP: u32 = 4;
    const ATTACH_DELAY: Duration = Duration::from_millis(100);

    let mut owner = ShmRingOwner::create_with_policy(
        &tag("bp_live"),
        8,
        CAP,
        0,
        &[],
        OverrunPolicy::Backpressure,
    )
    .expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");

    // NOBODY is attached, so the published cursor is 0 and the ring fills.
    for s in 0..CAP as u64 {
        push_seq(&mut producer, s);
    }
    assert_eq!(producer.backpressure_waits(), 0);

    let filled = Arc::new(AtomicBool::new(false));
    let filled_c = Arc::clone(&filled);
    let ch = std::thread::spawn(move || {
        await_flag(&filled_c, "the producer's fill");
        std::thread::sleep(ATTACH_DELAY);
        let mut c = ShmRingConsumer::open_at_live(&name).expect("live attach");
        let attach = c.read_cursor();
        // Read the header word the producer waits on, BEFORE anything else this
        // consumer does could write it.
        let published = c.published_read_cursor();
        // Wait for the released record WITHOUT committing: `available()` reads the
        // producer's cursor and writes nothing, so the attach is the only thing that
        // can have freed the producer.
        let start = Instant::now();
        while c.available() == 0 {
            assert!(
                start.elapsed() < DEADLINE,
                "the released push never reached the live consumer within {DEADLINE:?} \
                 — attach={attach}, published={published}, write_cursor={}",
                c.write_cursor()
            );
            std::thread::yield_now();
        }
        let got = drain_seqs(&mut c).expect("no lap");
        (attach, published, got)
    });

    filled.store(true, Ordering::SeqCst);
    push_seq(&mut producer, CAP as u64); // blocks until the live attach publishes
    let (attach, published, got) = ch.join().expect("consumer thread");

    assert_eq!(
        attach, CAP as u64,
        "the live attach lands at the producer's write cursor"
    );
    assert_eq!(
        published, attach,
        "the attach PUBLISHED its jumped cursor — this is the word the producer \
         waits on, and it is asserted directly because a behavioural release alone \
         can be supplied by any later commit"
    );
    assert_eq!(
        producer.backpressure_waits(),
        1,
        "the push with a full ring and no reader waited"
    );
    assert_eq!(
        producer.backpressure_wait_timeouts(),
        0,
        "the ATTACH released it — a live attach that published nothing would have \
         left it to expire and lap"
    );
    assert_eq!(
        got,
        vec![CAP as u64],
        "a mid-stream attach serves the suffix, and the released record is in it"
    );
    drop(owner);
}

// ===========================================================================
// 3. The trace ring is byte- and behaviour-unchanged
// ===========================================================================

/// The mode is per-ring and the TRACE ring did not get it.
///
/// Read off the mapped bytes rather than through an accessor, because the claim is
/// about the SEGMENT: the policy word must still be 0, and the read-cursor word —
/// which took over reserved padding — must still be 0 after a real drain-and-commit
/// cycle. A backpressure ring beside it, driven identically, moves both words, which
/// is what makes the trace ring's zeroes evidence rather than an untested default.
#[test]
fn the_trace_ring_selects_fail_loud_and_never_publishes_a_read_cursor() {
    const N: u64 = 12;

    // ---- the TRACE ring: FailLoud, and nothing writes the new word ----
    let mut trace = TraceRingOwner::create(&tag("trace_unchanged"), 1024, 3, &["a", "b"])
        .expect("create trace ring");
    let trace_name = trace.name().to_string();
    let mut tp = trace.producer().expect("producer");
    {
        let raw = open_raw(&trace_name);
        assert_eq!(
            raw.u32_at(HEADER_OFF_OVERRUN_POLICY),
            OverrunPolicy::FailLoud.as_wire(),
            "a trace ring is created FAIL-LOUD — collapsing the mode selection so it \
             gets BACKPRESSURE changes this word"
        );
        assert_eq!(
            raw.u64_at(HEADER_OFF_READ_CURSOR),
            0,
            "the read-cursor word starts where the old padding was: zero"
        );
    }

    for i in 0..N {
        tp.push(&trace_record(i));
    }
    let mut tc = TraceRingConsumer::open(&trace_name).expect("open trace consumer");
    let mut out = Vec::new();
    assert_eq!(tc.drain(&mut out).expect("drain"), N as usize);
    assert_eq!(
        out,
        (0..N).map(trace_record).collect::<Vec<_>>(),
        "the trace round trip is unchanged"
    );
    assert_eq!(
        tc.read_cursor(),
        N,
        "the consumer really advanced — the assertion below is about PUBLICATION, \
         not about a drain that never happened"
    );
    {
        let raw = open_raw(&trace_name);
        assert_eq!(
            raw.u64_at(HEADER_OFF_READ_CURSOR),
            0,
            "a FAIL-LOUD consumer never publishes its cursor: the word is still the \
             zero-filled padding it always was, so the trace ring's segment bytes are \
             what they were before this change"
        );
    }
    drop(tp);
    drop(trace);

    // ---- a BACKPRESSURE ring, driven identically, moves both words ----
    let mut owner = ShmRingOwner::create_with_policy(
        &tag("bp_publishes"),
        8,
        1024,
        3,
        &[],
        OverrunPolicy::Backpressure,
    )
    .expect("create");
    let name = owner.name().to_string();
    let mut p = owner.producer().expect("producer");
    let mut c = ShmRingConsumer::open(&name).expect("open");
    for s in 0..N {
        push_seq(&mut p, s);
    }
    assert_eq!(
        drain_seqs(&mut c).expect("drain"),
        (0..N).collect::<Vec<_>>()
    );
    {
        let raw = open_raw(&name);
        assert_eq!(
            raw.u32_at(HEADER_OFF_OVERRUN_POLICY),
            OverrunPolicy::Backpressure.as_wire(),
            "the mode a ring was created under is on the wire"
        );
        assert_eq!(
            raw.u64_at(HEADER_OFF_READ_CURSOR),
            N,
            "ANTI-TAUTOLOGY: the same drive DOES move the word on a backpressure \
             ring, so the trace ring's zero is a real refusal to publish"
        );
    }
    // …and a cross-process reader can tell which contract it is under.
    assert_eq!(
        OverrunPolicy::from_wire(c.overrun_policy()),
        Some(OverrunPolicy::Backpressure),
        "the consumer reads the policy back out of the header"
    );
    assert_eq!(c.published_read_cursor(), c.read_cursor());
    drop(owner);
}

// ===========================================================================
// 4. resync_after_fork — a REAL fork(2), which is the only faithful harness
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

/// `true` iff the child exited normally with status 0.
fn exited_clean(status: libc::c_int) -> bool {
    libc::WIFEXITED(status) && libc::WEXITSTATUS(status) == 0
}

/// THE `resync_after_fork` arm: a `fork` child takes the producer role, and the parent's copy is
/// stale until it resyncs.
///
/// The child's body is `push` + `_exit` and nothing else — no allocation, no lock,
/// no destructor — which is both what makes it sound after `fork` and exactly the
/// shape the ring's design contract promises for post-fork writers. `_exit` (not
/// `exit`, not a return) is load-bearing twice over: it runs no `atexit` handler and
/// no `Drop`, so the child never `shm_unlink`s the parent's ring.
///
/// The ring is BACKPRESSURE because that is the mode the real caller uses, so the
/// child's push exercises the room check too; capacity is far larger than the
/// stream, so nothing here waits.
#[test]
fn a_forked_child_pushes_and_the_parent_resyncs_before_its_next_push() {
    const PARENT_BEFORE: u64 = 4;
    const CHILD: u64 = 16;
    const PARENT_AFTER: u64 = 3;
    const TOTAL: u64 = PARENT_BEFORE + CHILD + PARENT_AFTER;

    let mut owner = ShmRingOwner::create_with_policy(
        &tag("fork_resync"),
        8,
        1024,
        0,
        &[],
        OverrunPolicy::Backpressure,
    )
    .expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");

    for s in 0..PARENT_BEFORE {
        push_seq(&mut producer, s);
    }

    // SAFETY: the child branch below touches only mapped memory and `_exit`.
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork: {}", std::io::Error::last_os_error());
    if pid == 0 {
        for s in PARENT_BEFORE..PARENT_BEFORE + CHILD {
            push_seq(&mut producer, s);
        }
        // SAFETY: leave immediately — no unwinding into the parent's frames, no
        // `Drop` for the inherited owner (which would unlink the parent's ring).
        unsafe { libc::_exit(0) };
    }

    let status = reap_bounded(pid);
    assert!(
        exited_clean(status),
        "the fork child must exit 0 (status {status})"
    );

    // THE HAZARD, stated: the parent's LOCAL cursor did not move, while the SHARED
    // one did. A push from here would write at a stale index.
    assert_eq!(
        producer.pushed(),
        PARENT_BEFORE,
        "fork duplicated the local cursor; the child's pushes are invisible to it"
    );

    let resynced = producer.resync_after_fork();
    assert_eq!(
        resynced,
        PARENT_BEFORE + CHILD,
        "resync reloads the cursor the CHILD published"
    );
    assert_eq!(producer.pushed(), PARENT_BEFORE + CHILD);

    for s in PARENT_BEFORE + CHILD..TOTAL {
        push_seq(&mut producer, s);
    }

    let mut consumer = ShmRingConsumer::open(&name).expect("open");
    let got = drain_seqs(&mut consumer).expect("capacity >> stream ⇒ no lap");
    assert_eq!(
        got,
        (0..TOTAL).collect::<Vec<_>>(),
        "one gap-free stream across the fork: the parent's prefix, the child's body, \
         and the parent's tail written where the child left off"
    );
    drop(owner);
}

/// This arm shows the hazard `resync_after_fork` exists to prevent — the anti-tautology
/// for the arm above.
///
/// Without it, "the resynced run produced the right stream" could be satisfied by a
/// ring on which the stale cursor happened to be harmless. Here the parent
/// deliberately does NOT resync, and the corruption is asserted EXACTLY: the
/// published write cursor goes BACKWARDS, the child's records are overwritten in
/// place, and what a consumer can see is a short, wrong stream.
///
/// This arm is expected to keep passing if `resync_after_fork` is deleted — it never
/// calls it. Its job is to prove the hazard is real, which is what makes the sibling
/// arm's oracle discriminating.
#[test]
fn without_resync_a_stale_parent_cursor_overwrites_the_childs_records() {
    const PARENT_BEFORE: u64 = 4;
    const CHILD: u64 = 16;
    const PARENT_AFTER: u64 = 3;

    let mut owner = ShmRingOwner::create_with_policy(
        &tag("fork_hazard"),
        8,
        1024,
        0,
        &[],
        OverrunPolicy::Backpressure,
    )
    .expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");

    for s in 0..PARENT_BEFORE {
        push_seq(&mut producer, s);
    }

    // SAFETY: as above — the child only pushes and `_exit`s.
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork: {}", std::io::Error::last_os_error());
    if pid == 0 {
        for s in PARENT_BEFORE..PARENT_BEFORE + CHILD {
            push_seq(&mut producer, s);
        }
        // SAFETY: leave without unwinding or running the inherited owner's `Drop`.
        unsafe { libc::_exit(0) };
    }
    assert!(
        exited_clean(reap_bounded(pid)),
        "the fork child must exit 0"
    );

    let mut consumer = ShmRingConsumer::open(&name).expect("open");
    assert_eq!(
        consumer.write_cursor(),
        PARENT_BEFORE + CHILD,
        "the child's pushes are published — this is what the parent is about to lose"
    );

    // NO resync. The parent writes from its stale cursor.
    for s in PARENT_BEFORE + CHILD..PARENT_BEFORE + CHILD + PARENT_AFTER {
        push_seq(&mut producer, s);
    }

    assert_eq!(
        consumer.write_cursor(),
        PARENT_BEFORE + PARENT_AFTER,
        "the published write cursor went BACKWARDS ({} → {}) — the ring now claims \
         fewer records exist than were committed",
        PARENT_BEFORE + CHILD,
        PARENT_BEFORE + PARENT_AFTER
    );

    let got = drain_seqs(&mut consumer).expect("no lap — capacity is huge");
    assert_eq!(
        got,
        vec![0, 1, 2, 3, 20, 21, 22],
        "the parent's tail landed ON TOP of the child's first records and the rest of \
         the child's work is unreachable: the stream is short AND wrong"
    );
    drop(owner);
}

/// `resync_after_fork` must be a RELOAD, not a jump: on a ring nobody forked, the
/// producer's own cursor already equals the published one, so the call changes
/// nothing and the stream continues seamlessly.
///
/// This is what stops the repair being written as "reset to zero" or "skip ahead" —
/// either would pass the fork arm's `Err`-free path while destroying an ordinary
/// producer that happened to call it.
#[test]
fn resync_on_a_ring_nobody_forked_is_a_no_op() {
    const A: u64 = 5;
    const B: u64 = 3;

    let mut owner = ShmRingOwner::create_with_policy(
        &tag("resync_noop"),
        8,
        1024,
        0,
        &[],
        OverrunPolicy::Backpressure,
    )
    .expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");

    for s in 0..A {
        push_seq(&mut producer, s);
    }
    assert_eq!(
        producer.resync_after_fork(),
        A,
        "an unforked producer's local cursor already IS the published one"
    );
    assert_eq!(producer.pushed(), A);
    // Idempotent: a second call cannot drift either.
    assert_eq!(producer.resync_after_fork(), A);

    for s in A..A + B {
        push_seq(&mut producer, s);
    }
    let mut consumer = ShmRingConsumer::open(&name).expect("open");
    assert_eq!(
        drain_seqs(&mut consumer).expect("drain"),
        (0..A + B).collect::<Vec<_>>(),
        "the stream continues exactly where it left off"
    );
    drop(owner);
}

// ===========================================================================
// The NON-BLOCKING push (`try_push`)
// ===========================================================================

/// A `BACKPRESSURE` push that has no room WAITS — up to five seconds, then laps.
/// Some callers run on a robot's node thread at a step boundary and must be able to
/// DECLINE a record instead, so `try_push` refuses rather than waits.
///
/// The oracle is the wait COUNTER, never a wall. A wall assertion tight enough to
/// separate "returned at once" from "waited" is also tight enough for a loaded
/// runner to invert; `backpressure_waits()` counts exactly the pushes that entered
/// `await_room`, and load cannot move it. The anti-tautology half is in the same
/// body: the blocking `push` on the SAME full ring must move that counter, or
/// "unchanged" would prove nothing.
#[test]
fn try_push_refuses_a_full_backpressure_ring_instead_of_waiting_for_it() {
    const CAP: u32 = 4;
    /// Short, because this arm DELIBERATELY drives one blocking push to its
    /// ceiling. The default is five seconds and nothing here needs to prove the
    /// duration — only that the wait happened at all.
    const SHORT_WAIT: Duration = Duration::from_millis(50);

    let mut owner = ShmRingOwner::create_with_policy(
        &tag("bp_trypush"),
        8,
        CAP,
        0,
        &[],
        OverrunPolicy::Backpressure,
    )
    .expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    producer.set_backpressure_wait_timeout(SHORT_WAIT);

    // A consumer that never drains: its published read cursor stays at 0, so the
    // ring genuinely fills. Opened BEFORE any push (the fixture above's reasoning).
    let mut consumer = ShmRingConsumer::open(&name).expect("open");

    // Room to spare: `try_push` behaves exactly as `push`.
    for i in 0..CAP as u64 {
        assert_eq!(
            producer.free_records(),
            Some(CAP as u64 - i),
            "a BACKPRESSURE ring reports its free records"
        );
        assert!(
            producer.try_push(&i.to_le_bytes()),
            "a push with room must be accepted"
        );
    }

    // THE ARM. The ring is full and nothing has drained it.
    assert_eq!(producer.free_records(), Some(0), "the ring is full");
    let waits_before = producer.backpressure_waits();
    assert!(
        !producer.try_push(&99u64.to_le_bytes()),
        "try_push must REFUSE a full backpressure ring"
    );
    assert_eq!(
        producer.backpressure_waits(),
        waits_before,
        "a refusal must not have entered the wait at all — this is the whole point: \
         the caller declines instead of paying up to the wait ceiling per record"
    );
    assert_eq!(
        producer.pushed(),
        CAP as u64,
        "a refused record must not be published"
    );

    // Draining RE-ARMS the acceptance: the refusal is transient, never a latch.
    // Checked BEFORE the deliberate lap below, so this drain reads a gap-free
    // stream and an Overrun here would be a real failure rather than the lap the
    // next block causes on purpose.
    let drained = drain_seqs(&mut consumer).expect("drain");
    assert_eq!(
        drained,
        (0..CAP as u64).collect::<Vec<_>>(),
        "the consumer must see exactly the published records, gap-free"
    );
    assert!(
        producer.free_records().is_some_and(|f| f > 0),
        "a drained ring has room again"
    );
    assert!(
        producer.try_push(&123u64.to_le_bytes()),
        "once the recorder drains, refusal records are published again"
    );
    assert_eq!(
        producer.backpressure_waits(),
        waits_before,
        "and none of that entered a wait either"
    );

    // ANTI-TAUTOLOGY: the blocking push on a full ring really does wait, so
    // "the counter did not move" above is a fact about `try_push`, not about a
    // ring that was never full. Refill first — the drain above freed room.
    let mut refilled = 0u64;
    while producer.try_push(&(200 + refilled).to_le_bytes()) {
        refilled += 1;
        assert!(
            refilled <= CAP as u64,
            "try_push must stop accepting at capacity"
        );
    }
    assert_eq!(producer.free_records(), Some(0), "full again");
    producer.push(&99u64.to_le_bytes());
    assert!(
        producer.backpressure_waits() > waits_before,
        "the blocking push must have waited — otherwise the refusal proves nothing"
    );
    assert_eq!(
        producer.backpressure_wait_timeouts(),
        1,
        "and it must have hit its ceiling and LAPPED, which is the behaviour the \
         refusal exists to avoid on a node thread"
    );

    drop(owner);
}

/// A `FailLoud` (every trace) ring never waits by contract, so it has nothing to
/// refuse: `try_push` is `push` and always succeeds — INCLUDING the push that laps.
///
/// The distinction the helper draws is "would this push WAIT?", never "will this
/// push lap?". A `try_push` that refused on a lapping FailLoud ring would silently
/// turn the wait-free trace path into a lossy one.
#[test]
fn try_push_never_refuses_a_fail_loud_ring_even_when_it_laps() {
    const CAP: u32 = 4;
    let mut owner = ShmRingOwner::create(&tag("fl_trypush"), 8, CAP, 0, &[]).expect("create");
    let mut producer = owner.producer().expect("producer");

    // Twice the capacity, with no consumer at all: every push must be accepted.
    for i in 0..(CAP as u64 * 2) {
        assert!(
            producer.try_push(&i.to_le_bytes()),
            "a FailLoud ring never waits, so try_push can never refuse (push {i})"
        );
    }
    assert_eq!(producer.pushed(), CAP as u64 * 2);
    assert_eq!(
        producer.free_records(),
        None,
        "and it answers no free-records question at all — there is no consumer cursor"
    );
    assert_eq!(producer.backpressure_waits(), 0, "it never entered a wait");
    drop(owner);
}

/// A malformed record PANICS even when the ring has no room.
///
/// `try_push`'s documented `# Panics` contract is "as `push`: a wrong-length record is
/// a caller bug". With the room check ahead of the length assertion that contract was
/// conditional on ring state: a malformed record fed to a FULL backpressure ring
/// returned `false` — indistinguishable from ordinary backpressure — and the same bug
/// only began panicking once the recorder had drained enough for the record to reach
/// `push`. A caller bug that surfaces only under drain pressure is a caller bug that
/// surfaces in production and nowhere else.
#[test]
#[should_panic(expected = "record length must equal the ring record_size")]
fn try_push_panics_on_a_malformed_record_even_when_the_ring_is_full() {
    const CAP: u32 = 2;
    let mut owner = ShmRingOwner::create_with_policy(
        &tag("bp_trymalformed"),
        8,
        CAP,
        0,
        &[],
        OverrunPolicy::Backpressure,
    )
    .expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    // A consumer that never drains, so the ring genuinely fills.
    let _consumer = ShmRingConsumer::open(&name).expect("open");
    for i in 0..CAP as u64 {
        assert!(producer.try_push(&i.to_le_bytes()), "room for {i}");
    }
    assert_eq!(
        producer.free_records(),
        Some(0),
        "the ring must be FULL, or this arm proves nothing about the ordering"
    );
    // 4 bytes into an 8-byte ring: a caller bug, and the ring's fullness must not
    // decide whether it is reported.
    producer.try_push(&[1u8, 2, 3, 4]);
}

/// The same malformed record is refused the same way when the ring HAS room — the
/// anti-tautology half, proving the panic above is about the record and not about a
/// full ring having some other failure mode.
#[test]
#[should_panic(expected = "record length must equal the ring record_size")]
fn try_push_panics_on_a_malformed_record_when_the_ring_has_room() {
    let mut owner = ShmRingOwner::create_with_policy(
        &tag("bp_trymalformed_room"),
        8,
        4,
        0,
        &[],
        OverrunPolicy::Backpressure,
    )
    .expect("create");
    let mut producer = owner.producer().expect("producer");
    assert_eq!(producer.free_records(), Some(4), "the ring has room");
    producer.try_push(&[1u8, 2, 3, 4]);
}
