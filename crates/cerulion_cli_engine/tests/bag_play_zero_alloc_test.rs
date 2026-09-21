// SPDX-License-Identifier: AGPL-3.0-only
//! The `bag play` hot loop allocates NOTHING per frame.
//!
//! # Why this test exists
//!
//! `bag play` is the robot-substitute data source, so it runs for as long as an
//! operator is working — hours, looping. A per-frame heap allocation would be
//! invisible in a 3-frame test and ruinous at 1 kHz for an afternoon. The frame
//! path is deliberately `mmap slice → publish_raw → loaned SHM slot`: ONE
//! memcpy (the true count — the bytes must cross from a file mapping into the
//! SHM segment) and ZERO allocations.
//!
//! # Why a DELTA, not an absolute count
//!
//! The hot loop is inside `bag_play_with_manager`, below the public seam, so it
//! cannot be bracketed by `enable()`/`disable()` from outside. Setup (open,
//! scan, publisher creation, the banner) legitimately allocates. So the probe
//! measures two plays of bags with the SAME shape and a DIFFERENT frame count,
//! and takes the difference: everything that scales with frame count shows up
//! in the delta, and everything that does not cancels.
//!
//! `allocs(2N) - allocs(N)` divided by `N` is therefore the per-frame
//! allocation count, and it must be 0.
//!
//! Own binary: `#[global_allocator]` is process-wide.

#![cfg(unix)]

use std::alloc::{GlobalAlloc, Layout, System};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::Arc;

use cerulion_bag::{BagWriter, BagWriterConfig, TopicSchema};
use cerulion_cli_engine::bag_cmd::{self, PlayOptions};
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::WireHeader;

/// Counts allocations on the MEASURED thread only, while armed.
///
/// Thread-scoped for the same reason `shm_ring_zero_alloc_test` is: background
/// threads (libtest, the transport's own workers) allocate for reasons that
/// have nothing to do with the loop under test, and the contract being pinned
/// is "the play loop allocates nothing ON ITS OWN THREAD".
struct CountingAllocator;

static ALLOCS: AtomicUsize = AtomicUsize::new(0);
static ARMED: AtomicBool = AtomicBool::new(false);
thread_local! {
    static MEASURED: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if ARMED.load(Ordering::Relaxed) {
            // `try_with` so a thread tearing its TLS down never panics inside
            // the allocator.
            let _ = MEASURED.try_with(|m| {
                if m.get() {
                    ALLOCS.fetch_add(1, Ordering::Relaxed);
                }
            });
        }
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        if ARMED.load(Ordering::Relaxed) {
            let _ = MEASURED.try_with(|m| {
                if m.get() {
                    ALLOCS.fetch_add(1, Ordering::Relaxed);
                }
            });
        }
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static ALLOC: CountingAllocator = CountingAllocator;

const HASH: u64 = 0x5A5A_1234_9876_ABCD;

fn unique() -> u64 {
    static N: AtomicU64 = AtomicU64::new(0);
    (std::process::id() as u64) << 20 | N.fetch_add(1, Ordering::Relaxed)
}

fn frame(seq: u32, payload: &[u8]) -> Vec<u8> {
    let total = WireHeader::SIZE + payload.len();
    let header = WireHeader {
        schema_hash: HASH,
        total_size: total as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence: seq,
        // All stamps identical: `pace_step` then returns a zero delay for every
        // frame, so the loop runs flat out and the measurement is not dominated
        // by sleeping.
        //
        // SCOPE LIMIT: identical stamps mean `advance_ns` is 0 on
        // every frame, so this fixture never enters the catch-up rate-bound
        // branch. The allocation claim it pins is therefore about the ordinary
        // publish path only. That is deliberate — stamping real deltas would
        // make the probe sleep, and a sleeping loop cannot be measured for
        // per-frame allocations at this resolution — but it means a future
        // allocation added inside the rate-bound branch would NOT be caught
        // here.
        timestamp_ns: 1_000,
    };
    let mut buf = vec![0u8; total];
    header.write_to_buf(&mut buf[..WireHeader::SIZE]);
    buf[WireHeader::SIZE..].copy_from_slice(payload);
    buf
}

fn write_bag(dir: &Path, name: &str, topic: &str, frames: usize) -> PathBuf {
    let path = dir.join(name);
    let schemas = vec![TopicSchema {
        topic: topic.to_string(),
        schema_name: "geometry_msgs/Vector3".to_string(),
        schema_hash: HASH,
        wire_fixed_size: 24,
    }];
    let bodies: Vec<Vec<u8>> = (0..frames)
        .map(|i| frame(i as u32, &[i as u8; 32]))
        .collect();
    let mut w = BagWriter::create(&path, BagWriterConfig::default(), &schemas).expect("create");
    w.write_chunk(|scope| {
        for (i, b) in bodies.iter().enumerate() {
            scope.write_message(topic, i as u32, 1_000, 1_000, &[&b[..]])?;
        }
        Ok(())
    })
    .expect("chunk");
    w.finalize().expect("finalize");
    path
}

/// The MINIMUM allocation count over [`REPS`] repetitions — a variance
/// REDUCER, not the thing that makes the gate sound.
///
/// A tempting justification would be that allocation
/// noise is one-sided ("a run can never do less than the real cost"), so the
/// minimum converges on the truth. That model is refuted by this probe's own
/// output: the observed setup counts move in BOTH directions (36 190 vs
/// 36 146), because `bag_play_with_manager` builds a `FrameWalker` over the 254
/// built-in schemas — about 36 000 allocations whose exact count depends on
/// container growth and ordering, not on a monotone "extra work" term.
///
/// So the minimum is kept only because it narrows the spread cheaply. What
/// actually carries the gate is the SIGNED MAGNITUDE band in the assertion
/// below, sized against a frame count large enough that a real per-frame
/// allocation dwarfs the residual jitter.
const REPS: usize = 3;

fn min_over_reps(mut f: impl FnMut() -> usize) -> usize {
    (0..REPS).map(|_| f()).min().expect("REPS > 0")
}

/// Play one bag on the CURRENT thread with the counter armed, returning the
/// allocation count for the whole call.
fn measured_play_once(bag: &Path) -> usize {
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("play_alloc_{}", unique()),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init_for_test");

    // Warm the path once on a throwaway run so one-time lazy statics (log
    // filters, format machinery) are not attributed to the measured window.
    let mut sink = Vec::with_capacity(4096);
    let _ = bag_cmd::bag_play_with_manager(
        &mgr,
        bag,
        PlayOptions {
            rate: 1_000_000.0,
            ..Default::default()
        },
        Arc::new(AtomicBool::new(true)),
        &mut sink,
    );

    let mgr2 = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("play_alloc_{}", unique()),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init_for_test");
    let mut sink2 = Vec::with_capacity(4096);

    MEASURED.with(|m| m.set(true));
    ALLOCS.store(0, Ordering::Relaxed);
    ARMED.store(true, Ordering::Relaxed);
    let out = bag_cmd::bag_play_with_manager(
        &mgr2,
        bag,
        PlayOptions {
            rate: 1_000_000.0,
            ..Default::default()
        },
        Arc::new(AtomicBool::new(true)),
        &mut sink2,
    );
    ARMED.store(false, Ordering::Relaxed);
    let n = ALLOCS.load(Ordering::Relaxed);
    MEASURED.with(|m| m.set(false));
    let summary = out.expect("play");
    assert!(summary.total_injected() > 0, "the probe must publish");
    n
}

/// Scan one bag with the counter armed (the pre-pass `bag play` runs before it
/// publishes anything), returning the allocation count.
fn measured_scan_once(bag: &Path) -> usize {
    let reader = cerulion_bag::BagReader::open(bag).expect("open");
    // Warm once so lazily-initialised statics are not attributed.
    let _ = bag_cmd::scan_bag(&reader, bag).expect("scan");

    MEASURED.with(|m| m.set(true));
    ALLOCS.store(0, Ordering::Relaxed);
    ARMED.store(true, Ordering::Relaxed);
    let scan = bag_cmd::scan_bag(&reader, bag);
    ARMED.store(false, Ordering::Relaxed);
    let n = ALLOCS.load(Ordering::Relaxed);
    MEASURED.with(|m| m.set(false));
    assert!(scan.expect("scan").total_frames > 0);
    n
}

/// THE pin: the per-frame allocation count of the play loop is ZERO — plus the
/// anti-tautology probe that the counter SEES allocations at all.
///
/// # Why ONE `#[test]` body and not two
///
/// `ALLOCS` and `ARMED` are process-global; only the
/// measured-thread flag is thread-local. Two `#[test]`s in one binary run
/// CONCURRENTLY under libtest's default thread pool, so they race BOTH ways: a
/// sibling's `ALLOCS.store(0)` landing mid-window makes the measurement fail
/// spuriously (MEASURED: 4 times in 20 runs on an idle machine), and — far worse —
/// the same store landing inside the measured window can ZERO a real
/// per-frame allocation and make the gate vacuously pass.
///
/// `#[serial]` alone does not fix it: it serialises the bodies, but the WAITING
/// test's thread still allocates lock-map entries inside the running test's
/// window. The remedy (see the `shm_ring_zero_alloc_test` module docs) is to fold
/// the measurements into ONE `#[test]` body so no sibling test thread exists
/// during a window, which is what this test does.
#[test]
fn the_play_loop_allocates_nothing_per_frame() {
    // ---- Step 1: the probe must SEE allocations on the measured thread. ----
    // Without this the zero below would be vacuous — a broken arming flag or a
    // mis-scoped thread-local would make every "0 allocations" claim pass by
    // measuring nothing at all.
    MEASURED.with(|m| m.set(true));
    ALLOCS.store(0, Ordering::Relaxed);
    ARMED.store(true, Ordering::Relaxed);
    let mut sink = 0usize;
    for i in 0..10usize {
        let v: Vec<u8> = Vec::with_capacity(64 + i);
        sink += v.capacity();
        std::hint::black_box(&v);
    }
    ARMED.store(false, Ordering::Relaxed);
    let seen = ALLOCS.load(Ordering::Relaxed);
    MEASURED.with(|m| m.set(false));
    assert!(sink > 0);
    assert!(
        seen >= 10,
        "the probe must observe the 10 deliberate allocations, saw {seen}"
    );

    // ---- Step 2: and NOTHING while disarmed. ------------------------------
    // ATTRIBUTION LIMIT: this disarms BOTH gates at once — `ARMED` is
    // already false from step 1 and `MEASURED` was just cleared — so a zero
    // here does not distinguish which gate did the work. It is a smoke test that
    // the counter stops, not a proof of which flag stopped it. Splitting them
    // would need a third step per flag; the pairing is sound because step 1
    // already proved BOTH are required to count.
    ALLOCS.store(0, Ordering::Relaxed);
    let v: Vec<u8> = Vec::with_capacity(4096);
    std::hint::black_box(&v);
    assert_eq!(
        ALLOCS.load(Ordering::Relaxed),
        0,
        "a disarmed probe must count nothing"
    );

    // ---- Step 3: the real gate. ------------------------------------------
    // N is large enough that the real signal DWARFS the fixed-setup jitter. The
    // play path builds a FrameWalker over the 254 built-in schemas (~36 000
    // allocations) whose count varies by up to ~70 between runs; at N = 500 a
    // genuine per-frame allocation (500) is only 7x that jitter, so the gate
    // is unreliable. At N = 5000 it is ~70x, and the +/-N/25 band (200) comfortably
    // clears the jitter while still failing a per-frame regression by 25x.
    const N: usize = 5_000;
    let dir = tempfile::tempdir().expect("tempdir");
    let topic = format!("/play/alloc/{}", unique());
    let small = write_bag(dir.path(), "small.mcap", &topic, N);
    let large = write_bag(dir.path(), "large.mcap", &topic, N * 2);

    // Attribute the two frame-count-scaling phases separately, so a failure
    // says WHICH one regressed instead of only that something did.
    let s_small = min_over_reps(|| measured_scan_once(&small));
    let s_large = min_over_reps(|| measured_scan_once(&large));
    let a_small = min_over_reps(|| measured_play_once(&small));
    let a_large = min_over_reps(|| measured_play_once(&large));
    eprintln!("alloc probe: scan {s_small} -> {s_large}; play {a_small} -> {a_large} (N = {N})");
    assert!(s_large >= s_small, "scan measurement inverted");
    assert_eq!(
        s_large - s_small,
        0,
        "the bag PRE-SCAN must allocate nothing per frame; {N} extra frames cost {} allocation(s)",
        s_large - s_small
    );

    // The contract is "the allocation count does not SCALE with frame count", so
    // the test is on the delta's MAGNITUDE, signed.
    //
    // A signed comparison matters here. Since the play path builds a
    // `FrameWalker` over the 254 built-in schemas to resolve each channel's wire
    // hash — about 36 000 allocations of SETUP that jitters by a couple between
    // runs — so the true per-frame signal (2 across 500 extra frames) sits well
    // inside that jitter and the delta legitimately comes out NEGATIVE some
    // runs. MEASURED: 36 181 vs 36 179. Demanding monotonicity fails roughly 2
    // runs in 20 on an idle machine, which is an unreliable gate, not a signal.
    //
    // The discriminator is unchanged in strength: a genuine per-frame
    // allocation costs N (500) here, so requiring |delta| under 4% of N fails
    // that by 25x while absorbing the setup jitter. The scan half above is
    // asserted at EXACTLY zero because it builds no walker and so has no jitter.
    let delta = a_large as i64 - a_small as i64;
    let scaled_bound = (N / 25) as i64;
    assert!(
        delta.abs() <= scaled_bound,
        "the play loop must allocate NOTHING PER FRAME: {N} extra frames moved the count by \
         {delta} allocation(s) ({a_small} for {N} frames, {a_large} for {}), which exceeds the \
         +/-{scaled_bound} noise band. A per-frame allocation would cost about {N}.",
        N * 2
    );
}
