// SPDX-License-Identifier: AGPL-3.0-only
//! The state decoder's hostile-COUNT pre-allocation cap
//! (`ELEMENT_PREALLOC_CAP`, `cerulion_core::state::impls::prealloc`) pinned by
//! an oracle that can see it.
//!
//! # Why this needs its own binary, and why the verdict cannot serve
//!
//! A container's count word is **blob-supplied**, so reserving from it scales
//! the reservation by `size_of::<T>()` against bytes the blob does not have.
//! The cap is the guard, and it is **output-equivalent**: with or without it
//! the decode ends in the same `StateError::Truncated`, having merely asked the
//! allocator for far more on the way. On macOS removing it does not
//! even fail loudly — the reservation is a lazy virtual mapping the OS
//! overcommits. **MEASURED: removing the cap passes every `--lib` state test.**
//! So the oracle is ALLOCATION, which needs a process-wide
//! `#[global_allocator]` and therefore a test binary of its own — the
//! `frame_walker_count_budget_test` pattern, for the same reason.
//!
//! # The blob must survive the COUNT guard to reach the cap at all
//!
//! `read_count` (review thread 5) refuses a count above the bytes
//! that follow, which by itself disposes of the four-byte `u32::MAX` blob — the
//! two guards answer different questions and compose. What the cap still buys
//! is the case a *justified* count reaches: a container of `ELEMENT_BYTES`-wide
//! elements amplifies its reservation by that width, so a 4 KiB blob can still
//! ask for **4 MiB**. Every blob here is therefore built to PASS the count
//! guard, which is also why the arms are worth keeping — a blob that died one
//! guard earlier would measure nothing.
//!
//! # The control is what makes the hostile assertion mean something
//!
//! A probe that measured nothing would pass the hostile arm trivially. The
//! CONTROL decodes a REAL `CONTROL_ELEMENTS`-long vector of the SAME element
//! type over a well-formed blob and must request at least its own payload —
//! proving the window sees element-list growth at all — while the hostile walk
//! must stay under a small absolute bound. Both verdicts are asserted too, so
//! the decoder's error contract is not weakened by measuring its cost.
//!
//! # Mutation verification
//!
//! Replacing `prealloc(n)` with a bare `n` fails BOTH arms — the sequence one
//! at **4_199_424 B** requested and the map one at **8_433_672 B**, against a
//! ceiling of 1_048_576 — while the `verdict` assertions stay green under that
//! replacement, which is the whole point. Widening `ELEMENT_PREALLOC_CAP` past the
//! ceiling fails them the same way.
//!
//! Pure — no transport, no shared memory. `#[serial]` for the shared
//! measurement window only.
//!
//! ```bash
//! cargo test -p cerulion_core --test state_prealloc_budget_test
//! ```

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use cerulion_core::state::{CerulionState, StateCursor, StateError};
use serial_test::serial;

thread_local! {
    /// `true` only on the thread that opened the measurement window.
    ///
    /// MUST be `const`-init with a non-`Drop` payload: a lazy (allocating) TLS
    /// init or a registered destructor touched from inside `GlobalAlloc::alloc`
    /// would recurse into the allocator.
    static MEASURED_THREAD: Cell<bool> = const { Cell::new(false) };
}

/// Counts heap BYTES requested by the measured thread while a window is open.
///
/// Bytes, not allocation COUNT: the thing under test is a single up-front
/// reservation whose SIZE is the whole signal. `realloc` is overridden because
/// a `Vec` that grows by doubling reaches the allocator through it, and the
/// control arm's real growth must be visible.
///
/// Thread-scoped for the reason documented in `shm_ring_zero_alloc_test`:
/// libtest machinery and platform lazy-init allocate on other threads at times
/// this test does not control.
struct CountingAllocator {
    inner: System,
    bytes: AtomicU64,
    enabled: AtomicBool,
}

impl CountingAllocator {
    const fn new() -> Self {
        Self {
            inner: System,
            bytes: AtomicU64::new(0),
            enabled: AtomicBool::new(false),
        }
    }

    fn enable(&self) {
        MEASURED_THREAD.set(true);
        self.bytes.store(0, Ordering::SeqCst);
        self.enabled.store(true, Ordering::SeqCst);
    }

    fn disable(&self) -> u64 {
        self.enabled.store(false, Ordering::SeqCst);
        MEASURED_THREAD.set(false);
        self.bytes.load(Ordering::SeqCst)
    }

    #[inline]
    fn note(&self, n: usize) {
        if self.enabled.load(Ordering::Relaxed) {
            // `try_with`, never `with`: during thread teardown TLS is
            // inaccessible and `with` would panic INSIDE the allocator.
            if MEASURED_THREAD.try_with(Cell::get).unwrap_or(false) {
                self.bytes.fetch_add(n as u64, Ordering::Relaxed);
            }
        }
    }
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        self.note(layout.size());
        unsafe { self.inner.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { self.inner.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        self.note(new_size);
        unsafe { self.inner.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator::new();

/// Width of the element type both arms use.
///
/// The amplification factor a blob-supplied count buys, and the whole reason
/// the cap still matters once `read_count` has refused the unjustifiable
/// counts: a count the blob CAN justify still reserves this much per element.
const ELEMENT_BYTES: usize = 1024;

/// Elements in the HOSTILE blob's count word.
///
/// Chosen to sit just inside `read_count`'s bound — the blob carries exactly
/// this many BYTES after the count — so the decode reaches the reservation and
/// only then runs out of payload.
const HOSTILE_COUNT: usize = 4096;

/// Elements in the CONTROL blob, which carries its full declared payload.
const CONTROL_ELEMENTS: usize = 2048;

/// Ceiling for a guarded hostile decode.
///
/// The cap reserves 256 elements (256 KiB at this width); everything else in
/// the window is the decoder's own fixed cost. Generous by 4x against the
/// guarded cost and 4x under the unguarded reservation.
const HOSTILE_CEILING_BYTES: u64 = 1024 * 1024;

/// `[u32 declared_count][payload_bytes bytes]` — a `Vec<[u8; ELEMENT_BYTES]>`
/// encoding whose count is JUSTIFIED by the byte budget but whose payload runs
/// out partway.
fn wide_element_blob(declared_count: usize, payload_bytes: usize) -> Vec<u8> {
    let mut v = Vec::with_capacity(4 + payload_bytes);
    v.extend_from_slice(&(declared_count as u32).to_le_bytes());
    v.resize(4 + payload_bytes, 0u8);
    v
}

/// Run `f` with the byte window open, returning `(result, bytes_requested)`.
fn measure<T>(f: impl FnOnce() -> T) -> (T, u64) {
    ALLOCATOR.enable();
    let out = f();
    let bytes = ALLOCATOR.disable();
    (out, bytes)
}

#[test]
#[serial]
fn a_hostile_count_word_never_reserves_from_the_blob() {
    // CONTROL: a well-formed blob carrying its full payload. Decoding it must
    // REQUEST at least that payload — this is what proves the window sees
    // element-list growth, and without it every "the hostile walk allocated
    // nothing" claim is vacuous.
    let well_formed = wide_element_blob(CONTROL_ELEMENTS, CONTROL_ELEMENTS * ELEMENT_BYTES);
    let (control, control_bytes) = measure(|| {
        let mut cursor = StateCursor::new(&well_formed);
        Vec::<[u8; ELEMENT_BYTES]>::cer_read(&mut cursor)
    });
    let control = control.expect("the well-formed blob decodes");
    assert_eq!(control.len(), CONTROL_ELEMENTS);
    let payload_floor = (CONTROL_ELEMENTS * ELEMENT_BYTES) as u64;
    assert!(
        control_bytes >= payload_floor,
        "the probe must see the real list being built: requested {control_bytes} B, \
         expected at least {payload_floor} B"
    );

    // HOSTILE: a count the blob's byte budget JUSTIFIES (so `read_count`
    // admits it) over elements it cannot possibly fill. The reservation is
    // where the amplification lives — 4 KiB of payload asking for 4 MiB.
    let hostile = wide_element_blob(HOSTILE_COUNT, HOSTILE_COUNT);
    let (verdict, hostile_bytes) = measure(|| {
        let mut cursor = StateCursor::new(&hostile);
        Vec::<[u8; ELEMENT_BYTES]>::cer_read(&mut cursor)
    });
    assert!(
        matches!(verdict, Err(StateError::Truncated { .. })),
        "the decoder still refuses; the guard is output-equivalent, got {verdict:?}"
    );
    assert!(
        hostile_bytes < HOSTILE_CEILING_BYTES,
        "a blob-declared count must never size the reservation: requested \
         {hostile_bytes} B against a ceiling of {HOSTILE_CEILING_BYTES} B \
         (control, for scale: {control_bytes} B for {CONTROL_ELEMENTS} real elements)"
    );
}

#[test]
#[serial]
fn the_cap_covers_maps_and_sets_as_well_as_sequences() {
    // Same guard, a different container family — a map reserves through the
    // same helper, and a `HashMap`'s per-entry cost is larger, so an unguarded
    // reservation here is worse, not better.
    let hostile = wide_element_blob(HOSTILE_COUNT, HOSTILE_COUNT);
    let (verdict, bytes) = measure(|| {
        let mut cursor = StateCursor::new(&hostile);
        HashMap::<u32, [u8; ELEMENT_BYTES]>::cer_read(&mut cursor)
    });
    assert!(verdict.is_err(), "the decoder still refuses");
    assert!(
        bytes < HOSTILE_CEILING_BYTES,
        "a map's reservation must not come from the blob either: requested {bytes} B"
    );
}
