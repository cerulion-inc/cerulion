// SPDX-License-Identifier: AGPL-3.0-only
//! Integration tests for the SPSC POSIX-SHM trace ring:
//! [`shm_ring`](cerulion_core::shm_ring) (generic) + [`trace_ring`](cerulion_core::trace_ring)
//! (trace-specific).
//!
//! These tests use REAL POSIX shared memory (`shm_open`/`mmap`) on macOS AND Linux
//! — they do NOT touch iceoryx2, so no `#[serial]` / `--test-threads=1` is needed.
//! Each test derives a UNIQUE ring tag from the test name + pid so the whole file
//! is parallel-safe. The pure byte-layout / manifest-codec oracles live as in-module
//! unit tests in `shm_ring.rs` / `trace_ring.rs`; this file exercises the ring
//! end-to-end over real SHM (round-trip, wrap, overrun, drop-unlink, generation,
//! cross-process, concurrent SPSC).
//!
//! The zero-alloc push proof lives in its OWN binary (`shm_ring_zero_alloc_test.rs`)
//! because `#[global_allocator]` is process-wide.

#![cfg(unix)]

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::shm_ring::{
    ring_shm_name, ShmRingConsumer, ShmRingError, ShmRingOwner, HEADER_OFF_CAPACITY,
    HEADER_OFF_MANIFEST_LEN, HEADER_OFF_RECORD_SIZE, MANIFEST_CAPACITY,
};
use cerulion_core::trace_ring::{
    default_capacity_records, encode_manifest, HeadStepGate, TraceRingConsumer, TraceRingError,
    TraceRingOwner, TraceRingRecord, RECORD_TYPE_DEPARTURE, RECORD_TYPE_FIRE,
    RECORD_TYPE_STEP_BOUNDARY,
};

// ===========================================================================
// Shared helpers
// ===========================================================================

/// Unique ring tag per test (name + pid) so parallel tests / re-runs never
/// collide on a `/dev/shm` object.
fn tag(name: &str) -> String {
    format!("{name}_{}", std::process::id())
}

/// The DETERMINISTIC oracle record for index `i` — computed IDENTICALLY by the
/// parent and (cross-process) the child, so a comparison of drained records
/// against this is a genuine oracle, not a self-compare. Every 5th record is a
/// DEPARTURE (duration 0, node_idx carries a "rank"); the rest are FIRE.
fn oracle_record(i: u64) -> TraceRingRecord {
    if i.is_multiple_of(5) {
        TraceRingRecord {
            step: i,
            fire_time_ns: 1000 + i * 7,
            duration_ns: 0,
            node_idx: (i % 4) as u32, // "departed rank"
            global_level: 0,
            record_type: RECORD_TYPE_DEPARTURE,
            reserved: 0,
        }
    } else {
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
}

/// Push one 8-byte little-endian sequence number onto a generic-ring producer.
fn push_seq(p: &mut cerulion_core::shm_ring::ShmRingProducer, s: u64) {
    p.push(&s.to_le_bytes());
}

/// Drain a generic 8-byte-record ring into a `Vec<u64>` and commit.
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

/// A raw writable `mmap` of an existing SHM object — for CRAFTING corrupt headers
/// in the hostile-input tests. `Drop` `munmap`s.
struct RawMap {
    ptr: *mut u8,
    len: usize,
}
impl Drop for RawMap {
    fn drop(&mut self) {
        // SAFETY: unmap the region we mapped in `open_writable`.
        unsafe {
            libc::munmap(self.ptr as *mut libc::c_void, self.len);
        }
    }
}
fn open_writable(name: &str) -> RawMap {
    let cname = std::ffi::CString::new(name).unwrap();
    // SAFETY: FFI open of an existing named SHM object; `cname` is valid.
    let fd = unsafe { libc::shm_open(cname.as_ptr(), libc::O_RDWR, 0) };
    assert!(
        fd >= 0,
        "open_writable shm_open: {}",
        std::io::Error::last_os_error()
    );
    // SAFETY: zeroed stat filled by fstat on our fd.
    let mut st: libc::stat = unsafe { std::mem::zeroed() };
    assert_eq!(unsafe { libc::fstat(fd, &mut st) }, 0);
    let len = st.st_size as usize;
    // SAFETY: map the object read/write.
    let addr = unsafe {
        libc::mmap(
            std::ptr::null_mut(),
            len,
            libc::PROT_READ | libc::PROT_WRITE,
            libc::MAP_SHARED,
            fd,
            0,
        )
    };
    // SAFETY: fd is mapped now.
    unsafe {
        libc::close(fd);
    }
    assert_ne!(addr, libc::MAP_FAILED);
    RawMap {
        ptr: addr as *mut u8,
        len,
    }
}

// ===========================================================================
// 1. Round-trip oracle (trace ring, cross-mapping in-process)
// ===========================================================================

#[test]
fn test_round_trip_matches_hand_built_oracle() {
    const N: u64 = 32;
    let node_ids = ["camera", "imu", "lidar"];
    let mut owner = TraceRingOwner::create(&tag("roundtrip"), 1024, 7, &node_ids).expect("create");
    let name = owner.name().to_string();
    let gen = owner.generation();
    let mut producer = owner.producer().expect("producer minted once");

    for i in 0..N {
        producer.push(&oracle_record(i));
    }

    // Consumer opens a SEPARATE mapping of the same object by name.
    let mut consumer = TraceRingConsumer::open(&name).expect("open");
    assert_eq!(consumer.rank(), 7, "rank travels through the header");
    assert_eq!(
        consumer.generation(),
        gen,
        "generation travels through the header"
    );
    assert_eq!(
        consumer.node_ids(),
        &["camera".to_string(), "imu".to_string(), "lidar".to_string()],
        "manifest node-id table travels through the header"
    );

    let mut out = Vec::new();
    let n = consumer.drain(&mut out).expect("drain");
    assert_eq!(n, N as usize);
    let expected: Vec<TraceRingRecord> = (0..N).map(oracle_record).collect();
    assert_eq!(out, expected, "drained records == hand-built oracle");
}

// ===========================================================================
// 2. Wrap-around (generic ring, capacity 8, incremental drains vs oracle)
// ===========================================================================

#[test]
fn test_wrap_around_incremental_drain_matches_staged_oracle() {
    // capacity 8 records × 8 bytes; push 20 across 3 staged drains that WRAP.
    let mut owner = ShmRingOwner::create(&tag("wrap"), 8, 8, 0, &[]).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    let mut consumer = ShmRingConsumer::open(&name).expect("open");

    // Stage 1: push 0..5, drain → exactly [0,1,2,3,4].
    for s in 0..5u64 {
        push_seq(&mut producer, s);
    }
    assert_eq!(drain_seqs(&mut consumer).unwrap(), vec![0, 1, 2, 3, 4]);

    // Stage 2: push 5..13 (8 records, unread == capacity, boundary OK), drain →
    // [5..13). These wrap the ring (indices 5,6,7,0,1,2,3,4).
    for s in 5..13u64 {
        push_seq(&mut producer, s);
    }
    assert_eq!(
        drain_seqs(&mut consumer).unwrap(),
        vec![5, 6, 7, 8, 9, 10, 11, 12]
    );

    // Stage 3: push 13..20, drain → [13..20).
    for s in 13..20u64 {
        push_seq(&mut producer, s);
    }
    assert_eq!(
        drain_seqs(&mut consumer).unwrap(),
        vec![13, 14, 15, 16, 17, 18, 19]
    );

    // Nothing left.
    assert_eq!(consumer.available(), 0);
    assert_eq!(consumer.read_cursor(), 20);
}

// ===========================================================================
// 3. Overrun up-front (push past capacity without draining)
// ===========================================================================

#[test]
fn test_overrun_up_front_reports_exact_records_lost() {
    let mut owner = ShmRingOwner::create(&tag("overrun_up"), 8, 8, 0, &[]).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    let mut consumer = ShmRingConsumer::open(&name).expect("open");

    // Push 9 into an 8-slot ring with NO drain → lapped by exactly 1.
    for s in 0..9u64 {
        push_seq(&mut producer, s);
    }
    let err = consumer.drain_slices().expect_err("must overrun");
    match err {
        ShmRingError::Overrun {
            records_lost,
            read_cursor,
            write_cursor,
            capacity,
        } => {
            assert_eq!(records_lost, 1, "9 pushed - 8 capacity = 1 lost");
            assert_eq!(read_cursor, 0);
            assert_eq!(write_cursor, 9);
            assert_eq!(capacity, 8);
        }
        other => panic!("expected Overrun, got {other:?}"),
    }
}

// ===========================================================================
// 4. Overrun torn-drain (lap DURING drain) + negative control
// ===========================================================================

#[test]
fn test_torn_drain_commit_detects_lap_and_negative_control_commits() {
    let mut owner = ShmRingOwner::create(&tag("torn"), 8, 8, 0, &[]).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    let mut consumer = ShmRingConsumer::open(&name).expect("open");

    // --- torn drain: drain_slices at write=5, producer laps to write=14 before commit ---
    for s in 0..5u64 {
        push_seq(&mut producer, s);
    }
    {
        // Hold the drained slices (modelling bagd reading them) while the producer
        // overwrites the region.
        let _slices = consumer.drain_slices().expect("drain ok at write=5");
        for s in 5..14u64 {
            push_seq(&mut producer, s);
        }
    } // _slices dropped
    let err = consumer
        .commit(5)
        .expect_err("torn drain must be detected at commit");
    match err {
        ShmRingError::Overrun {
            records_lost,
            read_cursor,
            write_cursor,
            capacity,
        } => {
            assert_eq!(read_cursor, 0, "commit did NOT advance the read cursor");
            assert_eq!(write_cursor, 14);
            assert_eq!(capacity, 8);
            assert_eq!(records_lost, 6, "14 - 8 = 6 records lost");
        }
        other => panic!("expected Overrun, got {other:?}"),
    }
    // The read cursor stayed at 0 (loud failure, no silent advance).
    assert_eq!(consumer.read_cursor(), 0);

    // --- negative control: a non-lapping push between drain and commit → commit OK ---
    let mut owner2 = ShmRingOwner::create(&tag("torn_ok"), 8, 8, 0, &[]).expect("create");
    let name2 = owner2.name().to_string();
    let mut prod2 = owner2.producer().expect("producer");
    let mut cons2 = ShmRingConsumer::open(&name2).expect("open");
    for s in 0..3u64 {
        push_seq(&mut prod2, s);
    }
    {
        let _slices = cons2.drain_slices().expect("drain ok at write=3");
        // Push 2 more — write=5, unread=5 <= capacity 8, NOT a lap.
        for s in 3..5u64 {
            push_seq(&mut prod2, s);
        }
    }
    cons2.commit(3).expect("non-lapping commit must succeed");
    assert_eq!(
        cons2.read_cursor(),
        3,
        "commit advanced by exactly the drained count"
    );
}

// ===========================================================================
// 7. Validation / hostile input
// ===========================================================================

#[test]
fn test_open_missing_name_errors_and_creates_nothing() {
    // A name that certainly does not exist.
    let name = format!(
        "/cer_rg_{:016x}",
        0xdead_beef_dead_0000u64.wrapping_add(std::process::id() as u64)
    );
    let err = ShmRingConsumer::open(&name).expect_err("missing name must error");
    assert!(matches!(err, ShmRingError::Open { .. }), "got {err:?}");
    // Strict open never creates: a second open still fails.
    assert!(ShmRingConsumer::open(&name).is_err());
}

#[test]
fn test_bad_magic_is_rejected() {
    let mut owner = ShmRingOwner::create(&tag("badmagic"), 8, 8, 0, &[]).expect("create");
    let name = owner.name().to_string();
    let _p = owner.producer();
    {
        // Zero the magic (offset 0..8) in the live shared segment.
        let rm = open_writable(&name);
        // SAFETY: rm maps ≥ 8 bytes; zero the magic field.
        unsafe {
            std::ptr::write_bytes(rm.ptr, 0u8, 8);
        }
    }
    let err = ShmRingConsumer::open(&name).expect_err("bad magic must be rejected");
    assert!(
        matches!(err, ShmRingError::Validation { .. }),
        "got {err:?}"
    );
    assert!(err.to_string().contains("magic"));
}

#[test]
fn test_bad_version_is_rejected() {
    let mut owner = ShmRingOwner::create(&tag("badver"), 8, 8, 0, &[]).expect("create");
    let name = owner.name().to_string();
    let _p = owner.producer();
    {
        // Overwrite version (offset 8..12) with 999, leaving magic valid.
        let rm = open_writable(&name);
        let bad = 999u32.to_ne_bytes();
        // SAFETY: rm maps well past offset 12; write the 4-byte version field.
        unsafe {
            std::ptr::copy_nonoverlapping(bad.as_ptr(), rm.ptr.add(8), 4);
        }
    }
    let err = ShmRingConsumer::open(&name).expect_err("bad version must be rejected");
    assert!(
        matches!(err, ShmRingError::Validation { .. }),
        "got {err:?}"
    );
    assert!(err.to_string().contains("version"));
}

#[test]
fn test_trace_consumer_rejects_wrong_record_size() {
    // A generic ring with record_size 16 (not the trace 40) — the trace consumer
    // must reject it.
    let mut owner = ShmRingOwner::create(&tag("recsize"), 16, 8, 0, &[]).expect("create");
    let name = owner.name().to_string();
    let _p = owner.producer();
    // Generic consumer opens fine (record_size is whatever the header says).
    let g = ShmRingConsumer::open(&name).expect("generic open");
    assert_eq!(g.record_size(), 16);
    // Trace consumer rejects it — but the manifest is empty here, so it fails at
    // the record-size gate first (which is what we want to pin).
    let err = TraceRingConsumer::open(&name).expect_err("trace consumer rejects rs != 40");
    match err {
        TraceRingError::RecordSizeMismatch { actual, expected } => {
            assert_eq!(actual, 16);
            assert_eq!(expected, 40);
        }
        other => panic!("expected RecordSizeMismatch, got {other:?}"),
    }
}

#[test]
fn test_capacity_not_power_of_two_is_rejected() {
    let err = ShmRingOwner::create(&tag("cap6"), 8, 6, 0, &[]).expect_err("cap 6 must be rejected");
    assert!(matches!(err, ShmRingError::Create { .. }), "got {err:?}");
    assert!(err.to_string().contains("power of two"));
}

#[test]
fn test_zero_capacity_and_zero_record_size_are_rejected() {
    assert!(matches!(
        ShmRingOwner::create(&tag("cap0"), 8, 0, 0, &[]),
        Err(ShmRingError::Create { .. })
    ));
    assert!(matches!(
        ShmRingOwner::create(&tag("rs0"), 0, 8, 0, &[]),
        Err(ShmRingError::Create { .. })
    ));
}

#[test]
fn test_oversize_manifest_at_create_is_rejected() {
    // A manifest larger than MANIFEST_CAPACITY handed to the GENERIC create.
    let big = vec![0u8; MANIFEST_CAPACITY + 1];
    let err = ShmRingOwner::create(&tag("bigman"), 8, 8, 0, &big).expect_err("oversize manifest");
    assert!(matches!(err, ShmRingError::Create { .. }), "got {err:?}");
    assert!(err.to_string().contains("MANIFEST_CAPACITY"));
}

/// The length guard is a real `assert_eq!`: a release-mode
/// `debug_assert` would let a short `record` feed `copy_nonoverlapping` a length
/// past the caller's slice — an OOB read (UB) reachable from a safe pub fn.
/// Ungated from `cfg(debug_assertions)`: pins the documented panic contract in
/// BOTH debug AND release (run under `--release` to prove the release half).
#[test]
#[should_panic(expected = "record length must equal the ring record_size")]
fn test_push_wrong_length_record_panics_in_all_build_modes() {
    let mut owner = ShmRingOwner::create(&tag("wronglen"), 8, 8, 0, &[]).expect("create");
    let mut producer = owner.producer().expect("producer");
    // Push a 4-byte record into an 8-byte-record ring — the assert fires.
    producer.push(&[1, 2, 3, 4]);
}

// ===========================================================================
// 8. Owner drop unlinks; a consumer opened BEFORE the drop keeps draining
// ===========================================================================

#[test]
fn test_owner_drop_unlinks_but_prior_consumer_survives() {
    const N: u64 = 6;
    let mut owner = TraceRingOwner::create(&tag("unlink"), 1024, 0, &["a", "b"]).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    for i in 0..N {
        producer.push(&oracle_record(i));
    }

    // Consumer opens (its own mapping) BEFORE the owner drops.
    let mut consumer = TraceRingConsumer::open(&name).expect("open before drop");

    // Drop the producer + owner → owner Drop shm_unlinks the NAME.
    drop(producer);
    drop(owner);

    // A FRESH open by name now fails (name unlinked).
    assert!(
        TraceRingConsumer::open(&name).is_err(),
        "after owner drop the name is unlinked; fresh open fails"
    );

    // But the pre-drop consumer's mapping is still valid (unlink != unmap) — it
    // still drains the records that were pushed (munmap-safety).
    let mut out = Vec::new();
    consumer
        .drain(&mut out)
        .expect("prior consumer still drains");
    let expected: Vec<TraceRingRecord> = (0..N).map(oracle_record).collect();
    assert_eq!(out, expected, "prior consumer reads the pushed records");
}

// ===========================================================================
// 9. Generation: recreate with the same tag → new generation observed
// ===========================================================================

#[test]
fn test_recreate_same_tag_bumps_generation() {
    let t = tag("regen");
    let owner1 = ShmRingOwner::create(&t, 8, 8, 0, &[]).expect("create 1");
    let gen1 = owner1.generation();
    drop(owner1);
    std::thread::sleep(Duration::from_millis(2));
    let owner2 = ShmRingOwner::create(&t, 8, 8, 0, &[]).expect("create 2");
    let gen2 = owner2.generation();
    assert!(
        gen2 > gen1,
        "recreate observes a strictly greater generation: {gen1} -> {gen2}"
    );
    // The consumer sees the NEW generation.
    let c = ShmRingConsumer::open(owner2.name()).expect("open 2");
    assert_eq!(c.generation(), gen2);
}

// ===========================================================================
// 12. Concurrent SPSC stress — clean (no overrun) + small-cap (overrun detected)
// ===========================================================================

#[test]
fn test_concurrent_spsc_100k_sequence_exact_no_overrun() {
    const N: u64 = 100_000;
    // capacity 2^17 = 131072 > N, so even if the consumer lags, nothing is lost.
    let mut owner = ShmRingOwner::create(&tag("spsc_clean"), 8, 131_072, 0, &[]).expect("create");
    let name = owner.name().to_string();
    let producer = owner.producer().expect("producer");

    let ph = std::thread::spawn(move || {
        let mut p = producer;
        for i in 0..N {
            p.push(&i.to_le_bytes());
        }
    });

    let ch = std::thread::spawn(move || {
        let mut c = ShmRingConsumer::open(&name).expect("open");
        let mut got: Vec<u64> = Vec::with_capacity(N as usize);
        let start = Instant::now();
        while (got.len() as u64) < N {
            // A PROGRESS wait, not a flag wait — so it carries its own deadline
            // and reports the ring state (see `await_flag`'s note). Unbounded,
            // a dead producer would HANG this test instead of failing it.
            assert!(
                start.elapsed() < CROSS_THREAD_DEADLINE,
                "only {} of {N} records arrived within {CROSS_THREAD_DEADLINE:?} — \
                 read_cursor={}, write_cursor={}, available={}",
                got.len(),
                c.read_cursor(),
                c.write_cursor(),
                c.available(),
            );
            let batch = drain_seqs(&mut c).expect("no overrun with adequate capacity");
            if batch.is_empty() {
                std::thread::yield_now();
                continue;
            }
            got.extend(batch);
        }
        got
    });

    ph.join().expect("producer thread");
    let got = ch.join().expect("consumer thread");
    let expected: Vec<u64> = (0..N).collect();
    assert_eq!(
        got, expected,
        "SPSC delivered 0..100k in exact order with no loss"
    );
    // Keep the owner alive until both threads are done.
    drop(owner);
}

#[test]
fn test_concurrent_spsc_small_capacity_overrun_is_detected() {
    const N: u64 = 100_000;
    // capacity 16 << N: a slow consumer WILL be lapped and must detect the overrun.
    let mut owner = ShmRingOwner::create(&tag("spsc_over"), 8, 16, 0, &[]).expect("create");
    let name = owner.name().to_string();
    let producer = owner.producer().expect("producer");
    let done = Arc::new(AtomicBool::new(false));
    let done_p = Arc::clone(&done);

    let ph = std::thread::spawn(move || {
        let mut p = producer;
        for i in 0..N {
            p.push(&i.to_le_bytes());
        }
        done_p.store(true, Ordering::SeqCst);
    });

    let ch = std::thread::spawn(move || {
        let mut c = ShmRingConsumer::open(&name).expect("open");
        // Wait until the producer has pushed all N into a 16-slot ring: the
        // consumer is now provably lapped (100k >> 16).
        await_flag(&done, CROSS_THREAD_DEADLINE, "the producer's 100k push run");
        // `.err()` consumes the Result (dropping any borrowed Ok slices), leaving an
        // owned Option<ShmRingError> with no borrow of `c`.
        let err = c.drain_slices().err();
        matches!(err, Some(ShmRingError::Overrun { .. }))
    });

    ph.join().expect("producer thread");
    let overrun_detected = ch.join().expect("consumer thread");
    assert!(
        overrun_detected,
        "a lapped small-capacity consumer must detect the overrun"
    );
    drop(owner);
}

// ===========================================================================
// 11. Cross-PROCESS: parent creates + pushes; a child process opens by name,
//     drains, and verifies against the SAME deterministic oracle.
// ===========================================================================

const XPROC_ENV_NAME: &str = "CER_SHMRING_CHILD_NAME";
const XPROC_ENV_N: &str = "CER_SHMRING_CHILD_N";
const XPROC_ENV_GEN: &str = "CER_SHMRING_CHILD_GEN";
const XPROC_ENV_RANK: &str = "CER_SHMRING_CHILD_RANK";

/// The child body: open the parent's ring by name, verify metadata + drained
/// records against the deterministic `oracle_record` (recomputed here — NOT a
/// serialized copy), returning a descriptive `Err` on any mismatch.
fn run_xproc_child(name: &str, n: u64, gen: u64, rank: u32) -> Result<(), String> {
    let mut c = TraceRingConsumer::open(name).map_err(|e| format!("open: {e}"))?;
    if c.rank() != rank {
        return Err(format!("rank mismatch: got {} want {rank}", c.rank()));
    }
    if c.generation() != gen {
        return Err(format!(
            "generation mismatch: got {} want {gen}",
            c.generation()
        ));
    }
    if c.node_ids() != ["camera".to_string(), "imu".to_string(), "lidar".to_string()] {
        return Err(format!("manifest mismatch: {:?}", c.node_ids()));
    }
    let mut out = Vec::new();
    c.drain(&mut out).map_err(|e| format!("drain: {e}"))?;
    if out.len() as u64 != n {
        return Err(format!("count mismatch: got {} want {n}", out.len()));
    }
    for (i, rec) in out.iter().enumerate() {
        let expect = oracle_record(i as u64);
        if *rec != expect {
            return Err(format!("record {i} mismatch: got {rec:?} want {expect:?}"));
        }
    }
    Ok(())
}

/// The child entrypoint. A NORMAL `cargo test` run (env unset) is a no-op pass;
/// when the supervisor re-invokes this binary with `--exact
/// cross_process_child_entrypoint` + the env, it runs the child body and
/// `exit(0)` on success / `exit(2)` on failure. NOT `#[ignore]`d — our SHM is REAL
/// on macOS too (unlike the barrier stub), so this cross-process test runs on
/// every OS.
#[test]
// P12 exemption, scoped to this fn rather than the file: this is the body of a
// SELF-RE-EXEC CHILD process — a process entrypoint by construction, whose exit
// code IS the channel the parent reads its verdict from. The ban stays armed for
// every other line in this binary, which is the half a file-wide allow gave up.
#[allow(clippy::disallowed_methods)]
fn cross_process_child_entrypoint() {
    let name = match std::env::var(XPROC_ENV_NAME) {
        Ok(v) => v,
        Err(_) => return, // normal run — not the child invocation.
    };
    let n: u64 = std::env::var(XPROC_ENV_N).unwrap().parse().unwrap();
    let gen: u64 = std::env::var(XPROC_ENV_GEN).unwrap().parse().unwrap();
    let rank: u32 = std::env::var(XPROC_ENV_RANK).unwrap().parse().unwrap();
    match run_xproc_child(&name, n, gen, rank) {
        Ok(()) => std::process::exit(0),
        Err(e) => {
            eprintln!("[shm_ring xproc child] FAILED: {e}");
            std::process::exit(2);
        }
    }
}

/// BOUNDED wait for a child to exit (SIGKILL + reap on timeout so a hung child
/// never hangs the suite).
fn wait_bounded(
    child: &mut std::process::Child,
    deadline: Duration,
) -> Option<std::process::ExitStatus> {
    let start = Instant::now();
    loop {
        match child.try_wait() {
            Ok(Some(status)) => return Some(status),
            Ok(None) => {
                if start.elapsed() >= deadline {
                    let _ = child.kill();
                    let _ = child.wait();
                    return None;
                }
                std::thread::sleep(Duration::from_millis(20));
            }
            Err(_) => {
                let _ = child.kill();
                let _ = child.wait();
                return None;
            }
        }
    }
}

#[test]
fn test_cross_process_consumer_drains_parent_oracle() {
    const N: u64 = 32;
    let node_ids = ["camera", "imu", "lidar"];
    let mut owner = TraceRingOwner::create(&tag("xproc"), 1024, 7, &node_ids).expect("create");
    let name = owner.name().to_string();
    let gen = owner.generation();
    let mut producer = owner.producer().expect("producer");
    for i in 0..N {
        producer.push(&oracle_record(i));
    }

    // Re-invoke THIS binary running ONLY the child entrypoint, over the same name.
    let exe = std::env::current_exe().expect("current_exe");
    let mut child = std::process::Command::new(exe)
        .args([
            "--exact",
            "cross_process_child_entrypoint",
            "--test-threads=1",
            "--nocapture",
        ])
        .env(XPROC_ENV_NAME, &name)
        .env(XPROC_ENV_N, N.to_string())
        .env(XPROC_ENV_GEN, gen.to_string())
        .env(XPROC_ENV_RANK, "7")
        .stdout(std::process::Stdio::inherit())
        .stderr(std::process::Stdio::inherit())
        .spawn()
        .expect("spawn child");

    let status = wait_bounded(&mut child, Duration::from_secs(5))
        .expect("child must exit within 5s (not hang)");
    // Keep the owner + producer alive until the child has finished draining (the
    // name + segment must persist through the child's open).
    drop(producer);
    drop(owner);
    assert!(
        status.success(),
        "cross-process child failed (exit {status:?}) — see its stderr above"
    );
}

// ===========================================================================
// The PRODUCTION default-size ring actually creates/maps/works
// ===========================================================================

/// The 64 MiB-budget default (2^20 records × 40 B ≈ 40 MiB + 64 KiB header) is
/// never exercised by the small-ring tests — this pins that the production
/// default segment really creates, maps, round-trips, and drains on macOS AND
/// Linux (encoding the coordinator's raw-syscall smoke test into CI).
#[test]
fn test_default_size_ring_creates_and_round_trips() {
    const N: u64 = 16;
    let mut owner = TraceRingOwner::create(
        &tag("default_size"),
        default_capacity_records(),
        3,
        &["camera", "imu"],
    )
    .expect("default-size create must succeed");
    assert_eq!(owner.capacity(), 1 << 20);
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    for i in 0..N {
        producer.push(&oracle_record(i));
    }

    let mut consumer = TraceRingConsumer::open(&name).expect("open default-size ring");
    assert_eq!(consumer.rank(), 3);
    let mut out = Vec::new();
    let n = consumer.drain(&mut out).expect("drain");
    assert_eq!(n, N as usize);
    let expected: Vec<TraceRingRecord> = (0..N).map(oracle_record).collect();
    assert_eq!(out, expected, "default-size ring round-trips the oracle");
}

// ===========================================================================
// Consumer validation branches over CRAFTED corrupt segments.
// Each starts from a VALID ring (kept alive) and corrupts ONE header field
// in-place via a raw writable mapping (same crafting mechanism as the
// bad-magic/bad-version tests — raw libc mmap writes at the offsets the
// `const _` asserts pin), then asserts the strict open rejects it with the
// matching Validation reason.
// ===========================================================================

/// Overwrite 4 bytes at header offset `off` in the live segment named `name`.
fn corrupt_u32(name: &str, off: usize, value: u32) {
    let rm = open_writable(name);
    let bytes = value.to_ne_bytes();
    // SAFETY: rm maps the whole segment (≥ header); `off + 4` is within the
    // 64-byte control area. Call sites pass the exported `HEADER_OFF_*`
    // constants, which are const-assert-pinned in `shm_ring.rs`.
    unsafe {
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), rm.ptr.add(off), 4);
    }
}

#[test]
fn test_zero_record_size_in_header_is_rejected_at_open() {
    let mut owner = ShmRingOwner::create(&tag("hdr_rs0"), 8, 8, 0, &[]).expect("create");
    let _p = owner.producer();
    corrupt_u32(owner.name(), HEADER_OFF_RECORD_SIZE, 0); // record_size := 0
    let err = ShmRingConsumer::open(owner.name()).expect_err("rs=0 must be rejected");
    assert!(
        matches!(err, ShmRingError::Validation { .. }),
        "got {err:?}"
    );
    assert!(
        err.to_string().contains("record_size is 0"),
        "reason names the zero record_size: {err}"
    );
}

#[test]
fn test_non_power_of_two_capacity_in_header_is_rejected_at_open() {
    let mut owner = ShmRingOwner::create(&tag("hdr_cap3"), 8, 8, 0, &[]).expect("create");
    let _p = owner.producer();
    corrupt_u32(owner.name(), HEADER_OFF_CAPACITY, 3); // capacity := 3 (non-power-of-two)
    let err = ShmRingConsumer::open(owner.name()).expect_err("cap=3 must be rejected");
    assert!(
        matches!(err, ShmRingError::Validation { .. }),
        "got {err:?}"
    );
    assert!(
        err.to_string().contains("not a non-zero power of two"),
        "reason names the bad capacity: {err}"
    );
}

#[test]
fn test_header_declares_more_than_segment_size_rejected_at_open() {
    let mut owner = ShmRingOwner::create(&tag("hdr_huge"), 8, 8, 0, &[]).expect("create");
    let _p = owner.producer();
    // capacity := 2^30 (a power of two, so it passes the pow2 gate) — the declared
    // data region (8 B × 2^30 = 8 GiB) dwarfs the actual fstat'd segment.
    corrupt_u32(owner.name(), HEADER_OFF_CAPACITY, 1 << 30);
    let err = ShmRingConsumer::open(owner.name()).expect_err("oversized declaration rejected");
    assert!(
        matches!(err, ShmRingError::Validation { .. }),
        "got {err:?}"
    );
    assert!(
        err.to_string().contains("< expected"),
        "reason names the segment-too-small mismatch: {err}"
    );
}

#[test]
fn test_manifest_len_beyond_capacity_in_header_is_rejected_at_open() {
    let mut owner = ShmRingOwner::create(&tag("hdr_man"), 8, 8, 0, &[]).expect("create");
    let _p = owner.producer();
    corrupt_u32(
        owner.name(),
        HEADER_OFF_MANIFEST_LEN,
        (MANIFEST_CAPACITY + 1) as u32,
    ); // manifest_len
    let err = ShmRingConsumer::open(owner.name()).expect_err("manifest_len overflow rejected");
    assert!(
        matches!(err, ShmRingError::Validation { .. }),
        "got {err:?}"
    );
    assert!(
        err.to_string().contains("exceeds MANIFEST_CAPACITY"),
        "reason names the manifest_len overflow: {err}"
    );
}

// ===========================================================================
// Single-producer double-mint returns None (SPSC guard)
// ===========================================================================

#[test]
fn test_generic_producer_double_mint_is_none() {
    let mut owner = ShmRingOwner::create(&tag("dblmint_g"), 8, 8, 0, &[]).expect("create");
    let first = owner.producer();
    assert!(first.is_some(), "first mint yields the producer");
    assert!(
        owner.producer().is_none(),
        "second mint must be None — a second producer would corrupt slots silently"
    );
}

#[test]
fn test_trace_producer_double_mint_is_none() {
    let mut owner = TraceRingOwner::create(&tag("dblmint_t"), 64, 0, &["a"]).expect("create");
    let first = owner.producer();
    assert!(first.is_some(), "first mint yields the producer");
    assert!(owner.producer().is_none(), "second mint must be None");
}

// ===========================================================================
// A single over-long node id → NodeIdTooLong (not ManifestTooLarge)
// ===========================================================================

#[test]
fn test_node_id_longer_than_u16_is_rejected_with_index_and_len() {
    let long = "x".repeat(70_000); // > u16::MAX (65535)
    let err = encode_manifest(&["ok", &long]).expect_err("70k-byte id must be rejected");
    match err {
        TraceRingError::NodeIdTooLong { index, len } => {
            assert_eq!(index, 1, "the offending id's index");
            assert_eq!(len, 70_000, "the offending id's byte length");
        }
        TraceRingError::ManifestTooLarge { .. } => {
            panic!("must be NodeIdTooLong (per-id gate fires first), not ManifestTooLarge")
        }
        other => panic!("expected NodeIdTooLong, got {other:?}"),
    }
}

// ===========================================================================
// Crash-recovery — a planted raw orphan is unlinked by create
// ===========================================================================

#[test]
fn test_create_over_planted_orphan_succeeds_and_round_trips() {
    let t = tag("orphan");
    // Plant an orphan DIRECTLY via libc (simulating a crashed prior run that never
    // unlinked): same name the owner will derive, sized arbitrarily small.
    let cname = std::ffi::CString::new(ring_shm_name(&t)).unwrap();
    // SAFETY: FFI create of the orphan object; mode 0o600.
    let fd = unsafe {
        libc::shm_open(
            cname.as_ptr(),
            libc::O_CREAT | libc::O_RDWR,
            0o600 as libc::c_uint,
        )
    };
    assert!(
        fd >= 0,
        "planting the orphan failed: {}",
        std::io::Error::last_os_error()
    );
    // SAFETY: size the fresh orphan (once) then close — it stays linked (orphaned).
    unsafe {
        libc::ftruncate(fd, 128);
        libc::close(fd);
    }

    // The unlink-first create path must clear the orphan and build a fresh ring.
    let mut owner =
        ShmRingOwner::create(&t, 8, 8, 0, &[]).expect("create over a planted orphan succeeds");
    let mut producer = owner.producer().expect("producer");
    let mut consumer = ShmRingConsumer::open(owner.name()).expect("open");
    for s in 0..3u64 {
        push_seq(&mut producer, s);
    }
    assert_eq!(
        drain_seqs(&mut consumer).unwrap(),
        vec![0, 1, 2],
        "the fresh ring over the orphan's name round-trips"
    );
}

// ===========================================================================
// Boundary + accessor pins
// ===========================================================================

#[test]
fn test_manifest_of_exactly_capacity_bytes_is_accepted() {
    // Off-by-one guard on the `>` check: EXACTLY MANIFEST_CAPACITY is legal.
    let exact = vec![0xABu8; MANIFEST_CAPACITY];
    let owner =
        ShmRingOwner::create(&tag("man_exact"), 8, 8, 0, &exact).expect("exact-capacity manifest");
    let consumer = ShmRingConsumer::open(owner.name()).expect("open");
    assert_eq!(
        consumer.manifest(),
        exact.as_slice(),
        "all 64 KiB travel intact"
    );
}

#[test]
fn test_pushed_tracks_push_count_and_overrun_policy_is_zero() {
    let mut owner = ShmRingOwner::create(&tag("pushed"), 8, 8, 0, &[]).expect("create");
    let mut producer = owner.producer().expect("producer");
    assert_eq!(producer.pushed(), 0, "no pushes yet");
    for s in 0..5u64 {
        push_seq(&mut producer, s);
    }
    assert_eq!(producer.pushed(), 5, "pushed() tracks the push count");

    let consumer = ShmRingConsumer::open(owner.name()).expect("open");
    assert_eq!(consumer.overrun_policy(), 0, "reserved field reads back 0");
}

/// Regression pin: over-commit is a HARD error in every build
/// mode (not a debug_assert) — the cursor must NOT advance, and a subsequent
/// exact commit succeeds.
#[test]
fn test_commit_beyond_available_errs_without_advancing_cursor() {
    let mut owner = ShmRingOwner::create(&tag("overcommit"), 8, 8, 0, &[]).expect("create");
    let mut producer = owner.producer().expect("producer");
    let mut consumer = ShmRingConsumer::open(owner.name()).expect("open");
    for s in 0..5u64 {
        push_seq(&mut producer, s);
    }
    {
        let _slices = consumer.drain_slices().expect("drain 5");
    }
    let err = consumer
        .commit(6)
        .expect_err("commit(6) with 5 available must err");
    match err {
        ShmRingError::CommitBeyondAvailable {
            requested,
            available,
        } => {
            assert_eq!(requested, 6);
            assert_eq!(available, 5);
        }
        other => panic!("expected CommitBeyondAvailable, got {other:?}"),
    }
    assert_eq!(
        consumer.read_cursor(),
        0,
        "failed commit did NOT advance the cursor"
    );
    consumer
        .commit(5)
        .expect("exact commit still succeeds after the failed one");
    assert_eq!(consumer.read_cursor(), 5);
}

/// Regression pin: a torn-drain `drain()` leaves `out` UNCHANGED
/// (no garbage records from the overwritten region leak to the caller).
#[test]
fn test_trace_drain_leaves_out_unchanged_on_torn_drain() {
    let mut owner = TraceRingOwner::create(&tag("torn_out"), 8, 0, &["a"]).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    let mut consumer = TraceRingConsumer::open(&name).expect("open");

    // Seed `out` with a sentinel the failed drain must preserve verbatim.
    let sentinel = oracle_record(99);
    let mut out = vec![sentinel];

    // Lap the ring before the drain: 14 pushes into an 8-slot ring (write=14,
    // read=0, 14-0 > 8) so `drain()`'s internal `drain_slices` errs up-front.
    // (The OTHER Err path — a lap landing after `drain_slices` succeeded but
    // before the internal commit, i.e. the truncate branch — is pinned
    // DETERMINISTICALLY by `test_trace_drain_truncates_decoded_records_on_torn_commit`
    // via the pre-commit hook seam.)
    for i in 0..14u64 {
        producer.push(&oracle_record(i));
    }
    let err = consumer.drain(&mut out).expect_err("lapped drain must err");
    assert!(
        matches!(err, TraceRingError::Ring(ShmRingError::Overrun { .. })),
        "expected Ring(Overrun), got {err:?}"
    );
    assert_eq!(
        out,
        vec![sentinel],
        "on Err, out is unchanged (no garbage records appended)"
    );
}

/// The torn-drain pin's concurrent arm: under a RACING producer, whenever `drain()`
/// errs — whichever internal branch it took (already-lapped up-front, or the
/// lap-during-decode truncate branch) — `out` must hold EXACTLY the records of
/// the successful drains (a verifiable prefix oracle: sequential steps 0..k),
/// with nothing appended by the failed call. Branch-agnostic, so it is
/// race-robust (no flake): the INVARIANT holds on every interleaving. The
/// DETERMINISTIC pin of the truncate branch lives in
/// `test_trace_drain_truncates_decoded_records_on_torn_commit` (the pre-commit
/// hook seam); this test's unique value is a REAL cross-thread interleaving the
/// seam cannot reproduce.
#[test]
fn test_trace_drain_out_invariant_holds_under_racing_producer() {
    const N: u64 = 50_000;
    let mut owner = TraceRingOwner::create(&tag("torn_race"), 64, 0, &["a"]).expect("create");
    let name = owner.name().to_string();
    let producer = owner.producer().expect("producer");

    let ph = std::thread::spawn(move || {
        let mut p = producer;
        for i in 0..N {
            p.push(&oracle_record(i));
        }
    });

    let mut consumer = TraceRingConsumer::open(&name).expect("open");
    let mut out: Vec<TraceRingRecord> = Vec::new();
    // Drain until the producer laps us (with cap 64 vs 50k pushes it always
    // will) or, in the (theoretical) never-lapped case, until all N arrived.
    let racing_start = Instant::now();
    let lapped = loop {
        // Same PROGRESS-wait discipline as the 100k arm: bounded, and the panic
        // names the ring state rather than hanging on a dead producer.
        assert!(
            racing_start.elapsed() < CROSS_THREAD_DEADLINE,
            "the racing drain neither lapped nor reached {N} within \
             {CROSS_THREAD_DEADLINE:?} — drained={}, read_cursor={}, available={}",
            out.len(),
            consumer.read_cursor(),
            consumer.available(),
        );
        let len_before = out.len();
        match consumer.drain(&mut out) {
            Ok(_) => {
                if out.len() as u64 >= N {
                    break false;
                }
                std::thread::yield_now();
            }
            Err(e) => {
                assert!(
                    matches!(e, TraceRingError::Ring(ShmRingError::Overrun { .. })),
                    "racing drain errs only with Ring(Overrun), got {e:?}"
                );
                assert_eq!(
                    out.len(),
                    len_before,
                    "the failed drain appended nothing (the torn-drain invariant)"
                );
                break true;
            }
        }
    };
    // Every record delivered before the failure is the exact sequential prefix
    // oracle — no torn/garbage record ever leaked into `out`.
    for (k, rec) in out.iter().enumerate() {
        assert_eq!(
            *rec,
            oracle_record(k as u64),
            "record {k} must match the sequential oracle (lapped={lapped})"
        );
    }
    ph.join().expect("producer thread");
}

/// The deterministic pin of `TraceRingConsumer::drain`'s
/// truncate-on-torn-commit branch (drain_slices SUCCEEDS → 4 records decoded and
/// pushed into `out` → the pre-commit hook laps the ring → the internal `commit`
/// returns Overrun → `out.truncate(original_len)` must discard the 4 decoded,
/// now-possibly-garbage records). The lap-in-that-window cannot be forced
/// single-threaded through the public API, so the cfg-gated
/// `drain_with_pre_commit_hook_for_test` seam stands in for the racing producer.
/// Reverting the truncate makes THIS test fail deterministically (`out` would
/// keep the 4 decoded records after the sentinels).
#[test]
fn test_trace_drain_truncates_decoded_records_on_torn_commit() {
    let mut owner = TraceRingOwner::create(&tag("torn_hook"), 8, 0, &["a"]).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    let mut consumer = TraceRingConsumer::open(&name).expect("open");

    // Two hand-built sentinels: "unchanged" is a CONTENT compare, not just a len
    // compare, so a truncate to the right len with corrupted content also fails.
    let sentinels = [oracle_record(1001), oracle_record(1002)];
    let mut out: Vec<TraceRingRecord> = sentinels.to_vec();

    for i in 0..4u64 {
        producer.push(&oracle_record(i));
    }
    // Hook: AFTER the 4 records were decoded into `out`, push 16 more — at the
    // internal commit, write - read = 20 - 0 > capacity 8 ⇒ torn-drain Overrun.
    let err = consumer
        .drain_with_pre_commit_hook_for_test(&mut out, || {
            for i in 4..20u64 {
                producer.push(&oracle_record(i));
            }
        })
        .expect_err("the hook-forced lap must surface as a torn-drain Err");
    assert!(
        matches!(err, TraceRingError::Ring(ShmRingError::Overrun { .. })),
        "expected Ring(Overrun), got {err:?}"
    );
    assert_eq!(
        out,
        sentinels.to_vec(),
        "the truncate branch discarded the 4 decoded records — out is exactly the sentinels"
    );
    assert_eq!(
        consumer.read_cursor(),
        0,
        "failed drain did not advance the cursor"
    );
}

/// Control arm (anti-tautology for the seam test above): the SAME shape but the
/// hook pushes only 2 records — no lap (write=6 ≤ capacity 8) — so the drain
/// SUCCEEDS and `out` gains exactly the 4 pre-hook records after the sentinels
/// (hand oracle). Proves the hook apparatus itself doesn't fail the drain.
#[test]
fn test_trace_drain_with_hook_no_lap_succeeds_with_oracle() {
    let mut owner = TraceRingOwner::create(&tag("torn_hook_ok"), 8, 0, &["a"]).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    let mut consumer = TraceRingConsumer::open(&name).expect("open");

    let sentinels = [oracle_record(1001), oracle_record(1002)];
    let mut out: Vec<TraceRingRecord> = sentinels.to_vec();

    for i in 0..4u64 {
        producer.push(&oracle_record(i));
    }
    let n = consumer
        .drain_with_pre_commit_hook_for_test(&mut out, || {
            for i in 4..6u64 {
                producer.push(&oracle_record(i));
            }
        })
        .expect("no lap → drain succeeds");
    assert_eq!(n, 4, "exactly the 4 pre-hook records were drained");
    let expected: Vec<TraceRingRecord> = sentinels
        .iter()
        .copied()
        .chain((0..4u64).map(oracle_record))
        .collect();
    assert_eq!(
        out, expected,
        "out == sentinels + the 4 drained records in order"
    );
    assert_eq!(consumer.read_cursor(), 4, "successful drain committed 4");
    // The 2 hook-pushed records are still pending for the next drain.
    assert_eq!(consumer.available(), 2);
}

// ===========================================================================
// The LIVE (mid-run) attach seam + the partial-head-step rule.
//
// `open` starts at record 0, and a ring that has already lapped therefore
// cannot be opened-and-drained at all: the first drain is a hard `Overrun`.
// A recorder attaching to a long-running graph is exactly that case. These
// arms pin what `open_at_live` changes (the window BEGINS at the attach
// instant) and — just as load-bearing — everything it does NOT change:
// overrun accounting from the attach point, torn-drain detection, and node
// identity through the manifest.
//
// Hand oracles throughout: the generic-ring arms push `i` as the record body
// so a drained value is checkable against the cursor observable, and the
// trace-ring arms build their step stream from named helpers.
// ===========================================================================

/// Node table for the trace-stream arms (also the manifest oracle).
const C1_NODE_IDS: [&str; 3] = ["camera", "imu", "lidar"];

/// A `STEP_BOUNDARY` record for `step` — the head of a step, carrying the
/// step's gating-clock value.
fn c1_boundary(step: u64) -> TraceRingRecord {
    TraceRingRecord {
        step,
        fire_time_ns: 1_000_000 * step,
        duration_ns: 0,
        node_idx: 0,
        global_level: 0,
        record_type: RECORD_TYPE_STEP_BOUNDARY,
        reserved: 0,
    }
}

/// A `FIRE` record for `node_idx` within `step`.
fn c1_fire(step: u64, node_idx: u32) -> TraceRingRecord {
    TraceRingRecord {
        step,
        fire_time_ns: 1_000_000 * step,
        duration_ns: 500 + node_idx as u64,
        node_idx,
        global_level: node_idx,
        record_type: RECORD_TYPE_FIRE,
        reserved: 0,
    }
}

/// One COMPLETE step as the scheduler emits it: the boundary, then one fire per
/// node in the manifest. 4 records per step for a 3-node graph.
fn c1_step(step: u64) -> Vec<TraceRingRecord> {
    let mut v = vec![c1_boundary(step)];
    for n in 0..C1_NODE_IDS.len() as u32 {
        v.push(c1_fire(step, n));
    }
    v
}

/// **C1-T1** — the headline: a ring the producer has already lapped is
/// UNOPENABLE-and-drainable at record 0 (the CONTROL proves it, in the same
/// body), while a live attach drains cleanly and serves EXACTLY the records
/// committed after it.
#[test]
fn a_live_attach_to_a_lapped_ring_drains_only_the_records_that_follow_it() {
    const CAP: u32 = 8;
    const PRE: u64 = 20; // 2.5 laps of an 8-slot ring
    let mut owner = ShmRingOwner::create(&tag("live_lapped"), 8, CAP, 0, &[]).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    for s in 0..PRE {
        push_seq(&mut producer, s);
    }

    // CONTROL — an at-zero consumer on this SAME ring is refused, with the exact
    // loss arithmetic. Without this arm "the live attach did not overrun" is
    // satisfied by a ring that never lapped in the first place.
    let mut at_zero = ShmRingConsumer::open(&name).expect("open at zero");
    match at_zero
        .drain_slices()
        .expect_err("a lapped at-zero consumer must Overrun")
    {
        ShmRingError::Overrun {
            records_lost,
            read_cursor,
            write_cursor,
            capacity,
        } => {
            assert_eq!(
                records_lost,
                PRE - CAP as u64,
                "20 pushed - 8 capacity = 12"
            );
            assert_eq!((read_cursor, write_cursor, capacity), (0, PRE, CAP as u64));
        }
        other => panic!("expected Overrun, got {other:?}"),
    }

    // The live attach: cursor lands ON the producer's write cursor.
    let mut live = ShmRingConsumer::open_at_live(&name).expect("open_at_live");
    assert_eq!(
        live.read_cursor(),
        PRE,
        "the live cursor IS the producer's write cursor at open"
    );
    assert_eq!(
        live.available(),
        0,
        "nothing is pending at the attach instant"
    );
    assert_eq!(
        drain_seqs(&mut live).expect("a live attach to a lapped ring must NOT overrun"),
        Vec::<u64>::new(),
        "the pre-attach records are outside this consumer's window entirely"
    );

    // Everything committed AFTER the attach is served, in order, exactly once.
    for s in PRE..PRE + 5 {
        push_seq(&mut producer, s);
    }
    assert_eq!(
        drain_seqs(&mut live).expect("post-attach drain"),
        vec![20, 21, 22, 23, 24],
        "the drained suffix is the hand oracle 20..25"
    );
    assert_eq!(live.read_cursor(), PRE + 5);
    drop(owner);
}

/// A live attach to a ring NOTHING has been written to is not a special case:
/// cursor 0, an empty drain, no error — and it then behaves like an at-zero
/// consumer.
#[test]
fn a_live_attach_to_a_ring_with_nothing_written_starts_at_zero_and_drains_clean() {
    let mut owner = ShmRingOwner::create(&tag("live_empty"), 8, 8, 0, &[]).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");

    let mut live = ShmRingConsumer::open_at_live(&name).expect("open_at_live on an empty ring");
    assert_eq!(live.read_cursor(), 0, "no records ⇒ the live cursor is 0");
    assert_eq!(live.available(), 0);
    assert_eq!(
        drain_seqs(&mut live).expect("empty drain"),
        Vec::<u64>::new()
    );

    for s in 0..3u64 {
        push_seq(&mut producer, s);
    }
    assert_eq!(
        drain_seqs(&mut live).expect("drain after the first pushes"),
        vec![0, 1, 2],
        "an empty-ring live attach sees the run from its very first record"
    );
    drop(owner);
}

/// Overrun accounting is measured FROM THE ATTACH POINT, not from record 0: a
/// producer that laps past the live cursor before the first drain still fails
/// loudly, and the reported cursors name the attach.
#[test]
fn a_producer_that_laps_past_the_live_cursor_is_reported_from_the_attach_point() {
    const CAP: u32 = 8;
    const PRE: u64 = 20;
    let mut owner = ShmRingOwner::create(&tag("live_relap"), 8, CAP, 0, &[]).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    for s in 0..PRE {
        push_seq(&mut producer, s);
    }

    let mut live = ShmRingConsumer::open_at_live(&name).expect("open_at_live");
    assert_eq!(live.read_cursor(), PRE);

    // 9 more into an 8-slot ring with no drain ⇒ lapped by exactly 1, counted
    // from the attach cursor (a from-zero count would read 21 lost).
    for s in PRE..PRE + 9 {
        push_seq(&mut producer, s);
    }
    match live
        .drain_slices()
        .expect_err("lapping past the live cursor must still Overrun")
    {
        ShmRingError::Overrun {
            records_lost,
            read_cursor,
            write_cursor,
            capacity,
        } => {
            assert_eq!(records_lost, 1, "9 post-attach records - 8 capacity = 1");
            assert_eq!(
                read_cursor, PRE,
                "the loss is measured from the ATTACH cursor, not from 0"
            );
            assert_eq!((write_cursor, capacity), (PRE + 9, CAP as u64));
        }
        other => panic!("expected Overrun, got {other:?}"),
    }
    drop(owner);
}

/// The design's node-identity claim, pinned rather than restated: the manifest
/// is written ONCE at create and read at open, so a consumer attaching after the
/// ring has lapped several times still resolves `node_idx` — and rank +
/// generation — exactly as an at-zero consumer would.
#[test]
fn a_late_attacher_resolves_node_identity_through_the_manifest_written_at_create() {
    const CAP: u32 = 8;
    let mut owner =
        TraceRingOwner::create(&tag("live_manifest"), CAP, 7, &C1_NODE_IDS).expect("create");
    let name = owner.name().to_string();
    let gen = owner.generation();
    let mut producer = owner.producer().expect("producer");
    // 20 records into an 8-slot ring — the manifest is long behind the lap point.
    for i in 0..20u64 {
        producer.push(&c1_fire(i, (i % 3) as u32));
    }

    let mut live = TraceRingConsumer::open_at_live(&name).expect("open_at_live");
    assert_eq!(
        live.node_ids(),
        &["camera".to_string(), "imu".to_string(), "lidar".to_string()],
        "the node-id table survives the lap — it lives in the header, not the ring"
    );
    assert_eq!(live.rank(), 7, "rank travels through the header");
    assert_eq!(
        live.generation(),
        gen,
        "generation travels through the header"
    );

    // A post-attach FIRE resolves through that table.
    producer.push(&c1_fire(99, 2));
    let mut out = Vec::new();
    assert_eq!(live.drain(&mut out).expect("drain"), 1);
    assert_eq!(
        out,
        vec![c1_fire(99, 2)],
        "the drained record is the oracle"
    );
    assert_eq!(
        live.node_ids()[out[0].node_idx as usize],
        "lidar",
        "node_idx 2 resolves to the third manifest entry"
    );
    drop(owner);
}

/// **C1-T2** — the mid-step attach: the live cursor lands inside step 3, so the
/// first records drained are FIREs whose STEP_BOUNDARY was committed before the
/// attach. The gate discards exactly those, and the recorded trace begins at the
/// next COMPLETE step.
#[test]
fn a_mid_step_live_attach_discards_the_headless_fires_and_records_from_the_next_step() {
    let mut owner =
        TraceRingOwner::create(&tag("live_midstep"), 1024, 0, &C1_NODE_IDS).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");

    // Steps 0..=2 complete (12 records), then the HEAD of step 3 (boundary + its
    // first fire) — the producer is now mid-step 3.
    for s in 0..3u64 {
        for r in c1_step(s) {
            producer.push(&r);
        }
    }
    producer.push(&c1_boundary(3));
    producer.push(&c1_fire(3, 0));

    let mut live = TraceRingConsumer::open_at_live(&name).expect("open_at_live");
    assert_eq!(live.read_cursor(), 14, "12 complete + 2 of step 3");

    // The TAIL of step 3 — headless from this consumer's point of view — then a
    // whole step 4.
    producer.push(&c1_fire(3, 1));
    producer.push(&c1_fire(3, 2));
    for r in c1_step(4) {
        producer.push(&r);
    }

    let mut gate = HeadStepGate::armed();
    let mut out = Vec::new();
    let admitted = live.drain_gated(&mut gate, &mut out).expect("gated drain");

    assert_eq!(admitted, 4, "only step 4's four records are admitted");
    assert_eq!(
        out,
        c1_step(4),
        "the recorded trace begins at a BOUNDARY — no headless fire survives"
    );
    assert_eq!(gate.discarded(), 2, "exactly step 3's two orphaned fires");
    assert_eq!(
        gate.first_step_recorded(),
        Some(4),
        "the first COMPLETE step is 4 — step 3 is partial and unrecoverable"
    );
    assert_eq!(
        live.read_cursor(),
        20,
        "all 6 records were CONSUMED (the discarded head is committed, not re-served)"
    );
    assert_eq!(live.available(), 0);
    drop(owner);
}

/// The boundary case of the boundary rule: a live cursor landing exactly ON a
/// step head discards nothing. Without this arm, "the gate discards the partial
/// head" is indistinguishable from "the gate always eats a record or two".
#[test]
fn a_live_attach_landing_exactly_on_a_step_boundary_discards_nothing() {
    let mut owner =
        TraceRingOwner::create(&tag("live_onboundary"), 1024, 0, &C1_NODE_IDS).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");

    for s in 0..3u64 {
        for r in c1_step(s) {
            producer.push(&r);
        }
    }

    let mut live = TraceRingConsumer::open_at_live(&name).expect("open_at_live");
    assert_eq!(live.read_cursor(), 12, "three complete steps");

    for r in c1_step(3) {
        producer.push(&r);
    }

    let mut gate = HeadStepGate::armed();
    let mut out = Vec::new();
    let admitted = live.drain_gated(&mut gate, &mut out).expect("gated drain");
    assert_eq!(admitted, 4);
    assert_eq!(out, c1_step(3), "the whole step is recorded");
    assert_eq!(gate.discarded(), 0, "nothing to discard at a step head");
    assert_eq!(gate.first_step_recorded(), Some(3));
    drop(owner);
}

/// A torn drain rolls the gate BACK alongside the records it discards: a gate
/// must not come away believing it saw a boundary in bytes the producer may have
/// overwritten mid-read. Driven through the cfg-gated pre-commit hook (a lap in
/// that window cannot be forced single-threaded through the public API).
#[test]
fn a_torn_gated_drain_rolls_the_head_step_gate_back_with_the_records_it_discards() {
    let mut owner =
        TraceRingOwner::create(&tag("live_torn_gate"), 8, 0, &C1_NODE_IDS).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    let mut consumer = TraceRingConsumer::open(&name).expect("open");

    // Two headless fires, then a complete-enough step 1 head: an armed gate
    // WOULD discard 2 and admit 2.
    producer.push(&c1_fire(0, 1));
    producer.push(&c1_fire(0, 2));
    producer.push(&c1_boundary(1));
    producer.push(&c1_fire(1, 0));

    let mut gate = HeadStepGate::armed();
    let mut out = Vec::new();
    let err = consumer
        .drain_gated_with_pre_commit_hook_for_test(&mut gate, &mut out, || {
            // 16 more ⇒ write - read = 20 > capacity 8 at the internal commit.
            for i in 0..16u64 {
                producer.push(&c1_fire(9, (i % 3) as u32));
            }
        })
        .expect_err("the hook-forced lap must surface as a torn-drain Err");
    assert!(
        matches!(err, TraceRingError::Ring(ShmRingError::Overrun { .. })),
        "expected Ring(Overrun), got {err:?}"
    );
    assert!(out.is_empty(), "the decoded records were truncated away");
    assert_eq!(
        gate,
        HeadStepGate::armed(),
        "the gate is rolled back to its pre-call state — still closed, nothing \
         discarded, no step claimed complete"
    );
    assert_eq!(consumer.read_cursor(), 0, "a failed drain advances nothing");
    drop(owner);
}

/// Anti-tautology for the arm above: the SAME shape with a hook that does not
/// lap COMMITS, so the gate really does move when nothing goes wrong (otherwise
/// "the gate was rolled back" would be satisfied by a gate that never advances).
#[test]
fn the_gated_hook_without_a_lap_commits_and_advances_the_gate() {
    let mut owner =
        TraceRingOwner::create(&tag("live_torn_gate_ok"), 8, 0, &C1_NODE_IDS).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    let mut consumer = TraceRingConsumer::open(&name).expect("open");

    producer.push(&c1_fire(0, 1));
    producer.push(&c1_fire(0, 2));
    producer.push(&c1_boundary(1));
    producer.push(&c1_fire(1, 0));

    let mut gate = HeadStepGate::armed();
    let mut out = Vec::new();
    let admitted = consumer
        .drain_gated_with_pre_commit_hook_for_test(&mut gate, &mut out, || {
            // 2 more: write = 6 ≤ capacity 8 ⇒ no lap.
            producer.push(&c1_fire(1, 1));
            producer.push(&c1_fire(1, 2));
        })
        .expect("no lap ⇒ the gated drain succeeds");
    assert_eq!(admitted, 2, "the boundary and the fire that follows it");
    assert_eq!(out, vec![c1_boundary(1), c1_fire(1, 0)]);
    assert_eq!(gate.discarded(), 2, "step 0's two headless fires");
    assert_eq!(gate.first_step_recorded(), Some(1));
    assert!(gate.is_open());
    assert_eq!(
        consumer.read_cursor(),
        4,
        "all 4 drained records were committed, discarded ones included"
    );
    assert_eq!(consumer.available(), 2, "the hook's 2 are still pending");
    drop(owner);
}

/// The GENEROUS liveness ceiling every cross-thread wait in this file is bounded
/// by. It bounds work measured in MILLISECONDS, so seconds here only decide how
/// long a WEDGE takes to report itself — it is never a wall stated in units of
/// the thing under test.
const CROSS_THREAD_DEADLINE: Duration = Duration::from_secs(30);

/// Spin on `flag` until it is set, or panic ATTRIBUTABLY at `deadline` naming
/// `what`.
///
/// EVERY cross-thread flag rendezvous in this file goes through here, and the
/// THREE progress loops that wait on RECORDS rather than a flag
/// (`test_concurrent_spsc_100k_...`, `test_trace_drain_out_invariant_...` and
/// the mid-stream arm) carry their own deadline inline — so no
/// unbounded cross-thread spin remains in this file at all. An unbounded spin
/// turns a real failure into a HANG, and
/// a hung test on CI burns the whole job's timeout with NO attributable red —
/// indistinguishable from the known timeout-cancel class.
///
/// What each panic can carry differs, and the difference is structural: THIS
/// helper names only `what` never happened and how long it waited, because a
/// flag wait has no ring state to report and the producer's wait for
/// `attached` runs BEFORE any attach cursor exists. The DRAIN loops name their
/// progress plus the ring's cursors (and, in the mid-stream arm, the attach
/// cursor), because at that point those exist and are what a reader needs.
fn await_flag(flag: &AtomicBool, deadline: Duration, what: &str) {
    let start = Instant::now();
    while !flag.load(Ordering::SeqCst) {
        assert!(
            start.elapsed() < deadline,
            "{what} never happened within {deadline:?} — the peer thread is gone \
             or wedged (this is a FAILURE, not a hang: see the sibling thread's panic)"
        );
        std::thread::yield_now();
    }
}

/// Cross-thread: a producer streaming at full speed while a consumer attaches
/// LIVE mid-stream must see a GAP-FREE SUFFIX. The oracle is exact and needs no
/// wall assertion — the record body IS its sequence number, so the first drained
/// value must equal the attach cursor the consumer itself reports, and the run
/// to the last record must be contiguous.
///
/// # Why the attach point is a HANDSHAKE and not a flag race
///
/// Bounding the attach from BELOW alone (a pre-fill flag) is not enough, and the
/// gap is not theoretical: with the producer free to run to `N` the moment it
/// sets the flag, a consumer that loses one scheduling slice attaches at `N`,
/// `available()` is 0 forever, and the drain loop below can never terminate —
/// there is no EOF in a ring, so an unbounded loop HANGS instead of failing.
/// Without the handshake the hang reproduces at about 1 run in 10.
///
/// So the ceiling is STRUCTURAL: the producer pre-fills `PRE` records, flags
/// them, and then WAITS for the consumer's own `attached` flag before pushing a
/// single record of the tail. The attach cursor is therefore EXACTLY `PRE` on
/// every run — asserted as an equality, not a band — and the tail streams at
/// full speed against a consumer that is already attached, which is the
/// concurrency this arm exists to exercise. Both waits and the drain loop are
/// deadline-bounded so any wedge is an attributable red.
///
/// Capacity exceeds `N`, so no legitimate run can overrun.
#[test]
fn a_live_attach_mid_stream_drains_a_gap_free_suffix() {
    const N: u64 = 100_000;
    const PRE: u64 = 20_000;
    /// Generous liveness ceiling for each cross-thread rendezvous. The work it
    /// bounds is milliseconds of pushing; seconds here only decide how long a
    /// WEDGE takes to report itself.
    const RENDEZVOUS_DEADLINE: Duration = Duration::from_secs(30);

    let mut owner = ShmRingOwner::create(&tag("live_stream"), 8, 131_072, 0, &[]).expect("create");
    let name = owner.name().to_string();
    let producer = owner.producer().expect("producer");
    let prefilled = Arc::new(AtomicBool::new(false));
    let attached = Arc::new(AtomicBool::new(false));
    let prefilled_p = Arc::clone(&prefilled);
    let attached_p = Arc::clone(&attached);

    let ph = std::thread::spawn(move || {
        let mut p = producer;
        for i in 0..PRE {
            p.push(&i.to_le_bytes());
        }
        prefilled_p.store(true, Ordering::SeqCst);
        // THE CEILING: not one record of the tail exists until the consumer has
        // attached, so the attach cursor cannot outrun the pre-fill.
        await_flag(
            &attached_p,
            RENDEZVOUS_DEADLINE,
            "the consumer's live attach",
        );
        for i in PRE..N {
            p.push(&i.to_le_bytes());
        }
    });

    let ch = std::thread::spawn(move || {
        await_flag(&prefilled, RENDEZVOUS_DEADLINE, "the producer's pre-fill");
        let mut c = ShmRingConsumer::open_at_live(&name).expect("open_at_live mid-stream");
        let attach = c.read_cursor();
        attached.store(true, Ordering::SeqCst);

        let mut got: Vec<u64> = Vec::new();
        let start = Instant::now();
        while got.last().copied() != Some(N - 1) {
            assert!(
                start.elapsed() < RENDEZVOUS_DEADLINE,
                "the drain never reached record {} within {RENDEZVOUS_DEADLINE:?} — \
                 attach={attach}, drained={}, last={:?}, read_cursor={}, \
                 write_cursor={}, available={}",
                N - 1,
                got.len(),
                got.last(),
                c.read_cursor(),
                c.write_cursor(),
                c.available(),
            );
            let batch = drain_seqs(&mut c).expect("capacity > N ⇒ no legitimate overrun");
            if batch.is_empty() {
                std::thread::yield_now();
                continue;
            }
            got.extend(batch);
        }
        (attach, got)
    });

    ph.join().expect("producer thread");
    let (attach, got) = ch.join().expect("consumer thread");

    assert_eq!(
        attach, PRE,
        "the handshake makes the attach cursor EXACTLY the pre-fill — no record \
         of the tail exists until the consumer has attached"
    );
    assert_eq!(
        got.first().copied(),
        Some(attach),
        "the first record served is exactly the one at the attach cursor — the \
         ring position and the payload agree"
    );
    let expected: Vec<u64> = (attach..N).collect();
    assert_eq!(
        got, expected,
        "the drained window is the gap-free suffix [attach, N)"
    );
    drop(owner);
}

/// A gate is state carried ACROSS drains, and a real recorder loop calls
/// `drain_gated` on a schedule — so the second call must not re-arm.
///
/// Every other gated arm makes exactly ONE `drain_gated` call per gate, which is
/// the one shape that cannot see a gate re-armed at drain entry: if the gate
/// re-armed there, a looping recorder would discard every batch's pre-boundary
/// records forever and `first_step_recorded` would revert to `None` after each
/// drain. Here drain 1 opens the gate on a boundary, and drain 2 carries that
/// SAME step's BOUNDARY-FREE tail — which an un-carried gate would discard in full.
#[test]
fn a_gate_carried_across_two_drains_keeps_its_verdict_and_does_not_re_arm() {
    let mut owner =
        TraceRingOwner::create(&tag("live_carry"), 1024, 0, &C1_NODE_IDS).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    let mut consumer = TraceRingConsumer::open(&name).expect("open");

    let mut gate = HeadStepGate::armed();
    let mut out = Vec::new();

    // Drain 1: two headless fires of step 3, then step 4's head.
    producer.push(&c1_fire(3, 1));
    producer.push(&c1_fire(3, 2));
    producer.push(&c1_boundary(4));
    producer.push(&c1_fire(4, 0));
    assert_eq!(
        consumer.drain_gated(&mut gate, &mut out).expect("drain 1"),
        2,
        "drain 1 admits step 4's head"
    );
    assert_eq!(out, vec![c1_boundary(4), c1_fire(4, 0)]);
    assert_eq!(gate.discarded(), 2);
    assert_eq!(gate.first_step_recorded(), Some(4));
    assert!(gate.is_open());

    // An IDLE drain between the two — the shape a POLLING recorder produces on
    // every quiet tick (drain(n) → drain(0) → drain(n)). A gate re-armed on an
    // empty pass looks identical to a healthy one until the NEXT batch arrives,
    // so the idle call is asserted to change NOTHING.
    let before_idle = gate;
    assert_eq!(
        consumer
            .drain_gated(&mut gate, &mut out)
            .expect("idle drain"),
        0,
        "nothing was published between the drains"
    );
    assert_eq!(
        gate, before_idle,
        "an idle drain leaves the gate EXACTLY as it was — it does not re-arm"
    );

    // Drain 2: the REST of step 4 — no boundary in this batch at all.
    producer.push(&c1_fire(4, 1));
    producer.push(&c1_fire(4, 2));
    assert_eq!(
        consumer
            .drain_gated(&mut gate, &mut out)
            .expect("drain 2 on the SAME gate"),
        2,
        "an open gate admits a boundary-free batch in full — it does not re-arm"
    );
    assert_eq!(
        out,
        vec![c1_boundary(4), c1_fire(4, 0), c1_fire(4, 1), c1_fire(4, 2)],
        "the recorded trace is the whole of step 4, across both drains"
    );
    assert_eq!(
        gate.discarded(),
        2,
        "nothing further was discarded — the count is carried, not recomputed"
    );
    assert_eq!(
        gate.first_step_recorded(),
        Some(4),
        "the first complete step survives the second drain"
    );
    drop(owner);
}

/// The torn-drain rollback restores the EXACT pre-call gate, which is only
/// observable from a NON-PRISTINE one.
///
/// The sibling rollback arm starts from a fresh `armed()` gate, where "roll back"
/// and "re-arm" are the same value — so it cannot see a rollback that resets
/// instead of restoring. Here the gate has already opened, discarded, and dated a
/// step before the torn drain, so only a true restore passes.
#[test]
fn a_torn_drain_restores_a_non_pristine_gate_field_for_field() {
    let mut owner =
        TraceRingOwner::create(&tag("live_torn_nonpristine"), 8, 0, &C1_NODE_IDS).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    let mut consumer = TraceRingConsumer::open(&name).expect("open");

    // Bring the gate to a NON-pristine state through a normal, successful drain.
    let mut gate = HeadStepGate::armed();
    let mut out = Vec::new();
    producer.push(&c1_fire(0, 1));
    producer.push(&c1_boundary(1));
    producer.push(&c1_fire(1, 0));
    assert_eq!(consumer.drain_gated(&mut gate, &mut out).expect("drain"), 2);
    let before = gate;
    assert_ne!(
        before,
        HeadStepGate::armed(),
        "PRECONDITION: the pre-call gate must be non-pristine, or a rollback that \
         merely re-arms would pass"
    );
    assert_eq!(
        (
            before.is_open(),
            before.discarded(),
            before.first_step_recorded()
        ),
        (true, 1, Some(1)),
        "the state the rollback must restore, stated explicitly"
    );
    assert_eq!(consumer.read_cursor(), 3);

    // Now a torn drain from that state: 2 more records drained, then the hook
    // laps the ring (write - read = 14 - 3 = 11 > capacity 8).
    producer.push(&c1_fire(1, 1));
    producer.push(&c1_fire(1, 2));
    let err = consumer
        .drain_gated_with_pre_commit_hook_for_test(&mut gate, &mut out, || {
            for i in 0..9u64 {
                producer.push(&c1_fire(9, (i % 3) as u32));
            }
        })
        .expect_err("the hook-forced lap must surface as a torn-drain Err");
    assert!(
        matches!(err, TraceRingError::Ring(ShmRingError::Overrun { .. })),
        "expected Ring(Overrun), got {err:?}"
    );
    assert_eq!(
        gate, before,
        "the gate is restored to its EXACT pre-call state — not reset to armed()"
    );
    assert_eq!(consumer.read_cursor(), 3, "a failed drain advances nothing");
    drop(owner);
}

/// The ORDINARY first drain of a mid-run attach admits NOTHING — every record is
/// pre-boundary — and it must still commit what it CONSUMED, or those records are
/// served again on the next drain and discarded a second time.
///
/// The sibling commit-count arm drives a MIXED pass (some discarded, some
/// admitted), where committing the admitted count still advances the cursor and
/// only the arithmetic is wrong. This is the all-discarded pass, where the
/// admitted count is 0 and the cursor would not move at all.
#[test]
fn a_drain_that_admits_nothing_still_commits_what_it_consumed() {
    const HEADLESS: u64 = 3;
    let mut owner =
        TraceRingOwner::create(&tag("live_all_discarded"), 1024, 0, &C1_NODE_IDS).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    let mut consumer = TraceRingConsumer::open(&name).expect("open");

    // Drain 1: the tail of a step whose boundary was lapped away — nothing to admit.
    for n in 0..HEADLESS as u32 {
        producer.push(&c1_fire(7, n));
    }
    let mut gate = HeadStepGate::armed();
    let mut out = Vec::new();
    assert_eq!(
        consumer.drain_gated(&mut gate, &mut out).expect("drain 1"),
        0,
        "no boundary yet ⇒ nothing admitted"
    );
    assert!(out.is_empty());
    assert_eq!(gate.discarded(), HEADLESS);
    assert_eq!(gate.first_step_recorded(), None);
    assert_eq!(
        consumer.read_cursor(),
        HEADLESS,
        "the discarded records were CONSUMED and committed — they were really read"
    );
    assert_eq!(consumer.available(), 0, "nothing is pending for a re-drain");

    // Drain 2: step 8 arrives. If drain 1 had committed its ADMITTED count (0),
    // the three headless fires would be re-served and discarded a second time.
    producer.push(&c1_boundary(8));
    producer.push(&c1_fire(8, 0));
    assert_eq!(
        consumer.drain_gated(&mut gate, &mut out).expect("drain 2"),
        2
    );
    assert_eq!(out, vec![c1_boundary(8), c1_fire(8, 0)]);
    assert_eq!(
        gate.discarded(),
        HEADLESS,
        "the head was discarded ONCE — a re-served batch would double this"
    );
    assert_eq!(gate.first_step_recorded(), Some(8));
    drop(owner);
}

/// `open` and `open_at_live` share ONE validation body, and this is what makes
/// that a checkable fact rather than a comment: a ring that is not a trace ring
/// must be refused IDENTICALLY by both, and so must a name that does not exist.
///
/// Without it, an `open_at_live` written as its own inline open — skipping the
/// record-size check — passes every other arm in this file, since they all open
/// rings the trace layer created.
#[test]
fn open_at_live_validates_exactly_like_open() {
    // A GENERIC ring: right magic, right version, WRONG record size for a trace.
    let owner = ShmRingOwner::create(&tag("live_parity"), 8, 8, 0, &[]).expect("create");
    let name = owner.name().to_string();

    let at_zero = TraceRingConsumer::open(&name).expect_err("open rejects rs != 40");
    let at_live = TraceRingConsumer::open_at_live(&name).expect_err("open_at_live must too");
    for (label, err) in [("open", at_zero), ("open_at_live", at_live)] {
        match err {
            TraceRingError::RecordSizeMismatch { actual, expected } => {
                assert_eq!((actual, expected), (8, 40), "{label} reports the sizes");
            }
            other => panic!("{label}: expected RecordSizeMismatch, got {other:?}"),
        }
    }

    // A name that was never created: both refuse, neither mints anything.
    let missing = ring_shm_name(&tag("live_parity_absent"));
    assert!(TraceRingConsumer::open(&missing).is_err());
    assert!(
        TraceRingConsumer::open_at_live(&missing).is_err(),
        "a live attach never creates the ring it cannot find"
    );
    assert!(
        ShmRingConsumer::open(&missing).is_err(),
        "and the generic layer left no phantom segment behind"
    );
    drop(owner);
}

// ===========================================================================
// The FailLoud MULTI-READER contract
// ===========================================================================

/// **The contract:** a `FailLoud` ring is ONE PRODUCER, N INDEPENDENT
/// READERS — each with a LOCAL cursor, each seeing the WHOLE stream.
///
/// This is the property γ's always-on trace rings rest on: a run's standing
/// window recorder drains the ring while a mid-run `cerulion bag record --run`
/// attaches to the SAME ring, and neither is a party to the other. Before
/// this contract was settled the module doc called the ring "SPSC" flatly and bagd's own
/// comments called a second reader a hazard, so the shape γ depends on was
/// documented as the shape not to build. The mechanics were already right; what
/// was missing was a test that says so.
///
/// # Two INDEPENDENTLY mutation-sensitive oracles, deliberately
///
/// 1. **Both readers drain the identical hand-built vector.** Killed by a
///    SHARED-cursor regression — one that seeds `open`'s cursor from the header
///    word AND publishes on commit, so reader A's progress advances reader B.
///    B then drains a truncated (or empty) vector. The oracle is
///    `oracle_record(i)` recomputed here, never A's output compared to B's: a
///    cross-compare passes when BOTH are wrong.
/// 2. **The header's read-cursor word stays at its create-time zero.** Killed
///    by the narrower half — dropping the policy gate in `publish_read_cursor`
///    so a `FailLoud` consumer stores its cursor. That alone changes no verdict
///    in oracle 1 (a `FailLoud` producer never LOADS the word), which is exactly
///    why it needs its own assertion: an unpublished word is what makes the
///    multi-reader property hold no matter how many readers there are, and it is
///    observable through `published_read_cursor` (Principle #3).
///
/// The interleaving is deliberate: B opens and drains only AFTER A has drained
/// AND committed, so a stolen cursor would be visible rather than raced past.
#[test]
fn two_failloud_consumers_each_see_the_whole_stream() {
    const N: u64 = 48;
    let node_ids = ["camera", "imu"];
    let mut owner =
        TraceRingOwner::create(&tag("two_readers"), 1024, 3, &node_ids).expect("create");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer minted once");
    for i in 0..N {
        producer.push(&oracle_record(i));
    }

    // The HAND oracle both readers are measured against.
    let expected: Vec<TraceRingRecord> = (0..N).map(oracle_record).collect();

    // Reader A — the standing window recorder's role. Drains AND commits, so a
    // shared cursor would have been published by the time B opens.
    let mut a = TraceRingConsumer::open(&name).expect("reader A opens");
    let mut out_a = Vec::new();
    assert_eq!(a.drain(&mut out_a).expect("A drains"), N as usize);
    assert_eq!(out_a, expected, "reader A sees the whole stream");
    assert_eq!(a.available(), 0, "A consumed everything it saw");

    // Reader B — the mid-run `bag record --run` attach, opened AFTER A finished.
    let mut b = TraceRingConsumer::open(&name).expect("reader B opens");
    let mut out_b = Vec::new();
    assert_eq!(
        b.drain(&mut out_b).expect("B drains"),
        N as usize,
        "B's cursor is its OWN: A's commit must not have advanced it"
    );
    assert_eq!(
        out_b, expected,
        "reader B sees the whole stream too — the SAME records, not the remainder"
    );

    // Both resolved node identity from the one manifest, independently.
    assert_eq!(a.node_ids(), b.node_ids(), "one manifest, two readers");
    assert_eq!(a.node_ids(), &["camera".to_string(), "imu".to_string()]);

    // Oracle 2: nothing was ever published into the shared word. `open` publishes
    // 0 and `commit` publishes the advanced cursor — BOTH behind the
    // `Backpressure` gate, so on this `FailLoud` ring the word is untouched at
    // its create-time zero even though two consumers opened, drained and
    // committed. (`published_read_cursor` reads the HEADER; `read_cursor` reads
    // the local field — the divergence is the observable.)
    assert_eq!(
        a.published_read_cursor(),
        0,
        "a FailLoud consumer publishes NOTHING — the header word stays at create-time zero"
    );
    assert_eq!(
        b.published_read_cursor(),
        0,
        "and B publishes nothing either"
    );
    assert_eq!(
        (a.read_cursor(), b.read_cursor()),
        (N, N),
        "…while both LOCAL cursors advanced: local-vs-published is the divergence"
    );

    // A third reader, opened last, still gets everything — N is not two.
    let mut c = TraceRingConsumer::open(&name).expect("reader C opens");
    let mut out_c = Vec::new();
    assert_eq!(c.drain(&mut out_c).expect("C drains"), N as usize);
    assert_eq!(out_c, expected, "the contract is N readers, not two");

    // ANTI-TAUTOLOGY: the apparatus can tell a truncated view from a full one.
    // A reader opened AT LIVE sees nothing of the backlog — same ring, same
    // moment, different cursor — so "both saw 48" above is a real measurement
    // rather than a drain that always answers 48.
    let mut live = TraceRingConsumer::open_at_live(&name).expect("reader D opens at live");
    let mut out_live = Vec::new();
    assert_eq!(
        live.drain(&mut out_live).expect("D drains"),
        0,
        "an at-live reader starts past the backlog — the probe distinguishes windows"
    );
    assert!(out_live.is_empty());

    drop(owner);
}
