// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation proof for the wait-free SHM ring push path.
//!
//! Design requirement 1: `push` must do NO alloc (also no lock / no syscall / no
//! clock read — but heap allocs are what a `#[global_allocator]` can observe). This
//! is the permanent regression guard: any future change that sneaks a heap
//! allocation onto the push path (a `Vec`, a `format!`, a boxed error) fails here.
//!
//! # Why a separate binary?
//!
//! `#[global_allocator]` is process-wide; we flip an `AtomicBool` so only the
//! measured steady-state push window is counted (setup — the `create` syscalls,
//! `Arc`/`String`/`Vec` allocations — is excluded).
//!
//! # Why ONE `#[serial]` test body?
//!
//! The `CountingAllocator` `enable()`/`disable()` window is process-global. Two
//! separate `#[test]`s here proved doubly racy: unserialized, one test's
//! `enable()` (which zeroes the count) could reset the other's window (silent
//! pass on broken code); serialized with `#[serial]`, the WAITING test's thread
//! still allocated one-time serial_test lock-map entries inside the running
//! test's window (intermittent spurious "got 5"). Both measurements therefore
//! live in one `#[test]` body (the fold-ordered-assertions pattern) —
//! no sibling test thread can exist during a measured window. `#[serial]` is
//! kept so any FUTURE test added to this binary serializes against it.
//!
//! # Why the counter is ADDITIONALLY thread-scoped
//!
//! The fold killed the sibling-TEST thread source, but the Linux CI runner
//! still failed ("got 2" in the phase-1 window): some NON-test background
//! thread (libtest harness machinery / platform TLS / lazy runtime init)
//! allocated during the measured window — sources outside this test's control
//! that differ per platform. The contract under test is "push performs no heap
//! allocation ON THE CALLING THREAD", so `alloc()` now counts only when the
//! window is enabled AND the calling thread is the one that enabled it
//! (a const-init `thread_local!` flag). A real allocation sneaked into `push`
//! still fails — it happens on the measured thread. The third phase is the sanity
//! control proving the thread-scoped counter still bites.

#![cfg(unix)]

use serial_test::serial;
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use cerulion_core::shm_ring::ShmRingOwner;
use cerulion_core::trace_ring::{TraceRingOwner, TraceRingRecord, RECORD_TYPE_FIRE};

// ---------------------------------------------------------------------------
// Counting allocator (thread-scoped variant of the step_zero_alloc_test.rs
// pattern)
// ---------------------------------------------------------------------------

thread_local! {
    /// `true` only on the thread that called [`CountingAllocator::enable`].
    ///
    /// MUST be `const`-init with a non-`Drop` payload (`Cell<bool>`): a lazy
    /// (allocating) TLS init or a registered destructor touched from inside
    /// `GlobalAlloc::alloc` would RECURSE into the allocator. Const-init TLS of
    /// a plain `Cell<bool>` performs no allocation and registers no destructor
    /// on first touch.
    static MEASURED_THREAD: Cell<bool> = const { Cell::new(false) };
}

/// Counts heap allocations made by the MEASURED THREAD while a window is open.
///
/// THREAD-SCOPED on purpose: the contract under test is "push performs no heap
/// allocation ON THE CALLING THREAD". Background threads — libtest machinery,
/// platform TLS/lazy runtime init, a sibling test blocked on the `#[serial]`
/// lock — allocate at times outside this test's control (the source of both the macOS
/// "got 5" flake and the Linux CI "got 2" failure) and are OUT of contract. A
/// real allocation introduced into `push()` still fails: it happens on the
/// measured thread.
struct CountingAllocator {
    inner: System,
    count: AtomicU64,
    enabled: AtomicBool,
}

impl CountingAllocator {
    const fn new() -> Self {
        Self {
            inner: System,
            count: AtomicU64::new(0),
            enabled: AtomicBool::new(false),
        }
    }
    /// Open a measurement window: zero the count, mark THIS thread as the
    /// measured one, and enable counting.
    fn enable(&self) {
        MEASURED_THREAD.set(true);
        self.count.store(0, Ordering::SeqCst);
        self.enabled.store(true, Ordering::SeqCst);
    }
    /// Close the window (clearing both the global flag and this thread's
    /// measured mark) and return the count.
    fn disable(&self) -> u64 {
        self.enabled.store(false, Ordering::SeqCst);
        MEASURED_THREAD.set(false);
        self.count.load(Ordering::SeqCst)
    }
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if self.enabled.load(Ordering::Relaxed) {
            // `try_with`, never `with`: during thread teardown TLS is
            // inaccessible and `with` would panic INSIDE the allocator. An
            // inaccessible TLS means "not the measured thread" — don't count.
            let measured = MEASURED_THREAD.try_with(Cell::get).unwrap_or(false);
            if measured {
                self.count.fetch_add(1, Ordering::Relaxed);
            }
        }
        unsafe { self.inner.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { self.inner.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator::new();

fn tag(name: &str) -> String {
    format!("ring_za_{name}_{}", std::process::id())
}

/// BOTH push paths (trace layer + generic ring) measured in ONE test body,
/// plus the counter-bites sanity control.
///
/// One body, not two `#[serial]` tests: `#[serial]` serializes bodies but the
/// waiting test's THREAD still contends on serial_test's lazy lock-map during
/// the running test's measured window (the count-POLLUTION flip side of the
/// count-reset race `#[serial]` fixes). A single body is the
/// fold-ordered-assertions pattern. The counter is ADDITIONALLY thread-scoped
/// (see the allocator doc) because Linux CI proved non-test background threads
/// allocate during the window too.
#[test]
#[serial]
fn test_push_paths_are_zero_alloc_at_steady_state() {
    // --- First phase: trace-layer producer (encode + push) ---
    let mut owner =
        TraceRingOwner::create(&tag("trace_za"), 1024, 0, &["a", "b", "c"]).expect("create");
    let mut producer = owner.producer().expect("producer");

    // Warm up (excluded from the measurement) — push a handful so the ring is at
    // steady state (nothing about push is lazy, but this mirrors the executor gate).
    for i in 0..16u64 {
        producer.push(&record(i));
    }

    ALLOCATOR.enable();
    for i in 0..10_000u64 {
        producer.push(&record(i));
    }
    let allocs = ALLOCATOR.disable();

    assert_eq!(
        allocs, 0,
        "TraceRingProducer::push must not heap-allocate at steady state (got {allocs})"
    );

    // --- Second phase: generic ring producer (raw byte-record push) ---
    let mut owner = ShmRingOwner::create(&tag("generic_za"), 40, 1024, 0, &[]).expect("create");
    let mut producer = owner.producer().expect("producer");
    let buf = [0xABu8; 40];

    for _ in 0..16 {
        producer.push(&buf);
    }

    ALLOCATOR.enable();
    for _ in 0..10_000 {
        producer.push(&buf);
    }
    let allocs = ALLOCATOR.disable();

    assert_eq!(
        allocs, 0,
        "ShmRingProducer::push must not heap-allocate at steady state (got {allocs})"
    );

    // --- Third phase: sanity control — the thread-scoped counter still BITES ---
    // Kills the "thread-scoping accidentally made the counter count nothing"
    // false-green: one deliberate heap allocation on the measured thread must
    // read back as EXACTLY 1, and a fresh empty window as 0.
    ALLOCATOR.enable();
    let v: Vec<u8> = Vec::with_capacity(1);
    std::hint::black_box(v);
    let allocs = ALLOCATOR.disable();
    assert_eq!(
        allocs, 1,
        "a deliberate measured-thread allocation must count exactly 1 (got {allocs})"
    );

    ALLOCATOR.enable();
    let allocs = ALLOCATOR.disable();
    assert_eq!(
        allocs, 0,
        "an empty fresh window must count 0 (got {allocs})"
    );
}

fn record(i: u64) -> TraceRingRecord {
    TraceRingRecord {
        step: i,
        fire_time_ns: 1000 + i,
        duration_ns: i,
        node_idx: (i % 3) as u32,
        global_level: 0,
        record_type: RECORD_TYPE_FIRE,
        reserved: 0,
    }
}
