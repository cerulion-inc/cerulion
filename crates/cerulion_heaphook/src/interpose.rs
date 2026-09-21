// SPDX-License-Identifier: AGPL-3.0-only
//! The `malloc`-family interposers (Linux/GNU). Each exported symbol wins
//! first-in-load-order resolution when this `.so` is `LD_PRELOAD`ed, so every
//! `std::vector`/`std::string` allocation in the process routes through here.
//!
//! Every entry point:
//!   1. holds the thread re-entrancy guard ([`state::with_guard`]) — a
//!      re-entrant call (our own bookkeeping, or `dlsym` calling `calloc`) skips
//!      our logic and goes straight to the real allocator / bootstrap arena;
//!   2. while a borrow window is armed on the thread, bump-allocates into the
//!      loan slot (an overflow latches the escape and falls back to the real
//!      allocator so the fill never faults);
//!   3. classifies `free`/`realloc`/`malloc_usable_size` pointers by the EXACT
//!      pure [`crate::classify::classify_ptr`] test (via the `state` helpers) — a
//!      registered SHM range is routed to the release callback, never to glibc
//!      `free`.

use core::ffi::{c_int, c_void};

use crate::state;
use crate::state::{FreeClass, UsableOutcome};
use crate::window::BumpOutcome;

/// `alignof(max_align_t)` on x86_64 and aarch64 glibc — what plain
/// `malloc`/`calloc`/`realloc` guarantee.
const MAX_ALIGN: usize = 16;

// ── real-or-bootstrap fallbacks ─────────────────────────────────────────────

/// The bootstrap arena, setting `errno = ENOMEM` on exhaustion (a NULL return)
/// so a caller inspecting `errno` sees the real failure, not a stale value.
fn bootstrap_or_enomem(size: usize, align: usize, zero: bool) -> *mut c_void {
    let p = state::bootstrap_alloc(size, align, zero);
    if p.is_null() {
        set_enomem();
    }
    p
}

fn real_malloc_or_bootstrap(size: usize) -> *mut c_void {
    state::ensure_resolved();
    match state::real_malloc() {
        // SAFETY: `f` is glibc's malloc.
        Some(f) => unsafe { f(size) },
        None => bootstrap_or_enomem(size, MAX_ALIGN, false),
    }
}

fn real_calloc_or_bootstrap(nmemb: usize, size: usize) -> *mut c_void {
    state::ensure_resolved();
    match state::real_calloc() {
        // SAFETY: `f` is glibc's calloc.
        Some(f) => unsafe { f(nmemb, size) },
        None => match nmemb.checked_mul(size) {
            Some(total) => bootstrap_or_enomem(total, MAX_ALIGN, true),
            None => {
                set_enomem();
                core::ptr::null_mut()
            }
        },
    }
}

/// Allocate `size` bytes at `align` from the best available aligned real
/// allocator, else the bootstrap arena.
fn real_aligned_or_bootstrap(align: usize, size: usize) -> *mut c_void {
    state::ensure_resolved();
    // SAFETY: each resolved fn is glibc's same-named allocator.
    unsafe {
        if let Some(f) = state::real_memalign() {
            return f(align, size);
        }
        if let Some(f) = state::real_aligned_alloc() {
            // aligned_alloc requires size to be a multiple of align.
            let sz = crate::window::align_up(size, align).unwrap_or(size);
            return f(align, sz);
        }
        if let Some(f) = state::real_posix_memalign() {
            let mut out: *mut c_void = core::ptr::null_mut();
            if f(&mut out, align, size) == 0 {
                return out;
            }
            return core::ptr::null_mut();
        }
    }
    bootstrap_or_enomem(size, align, false)
}

fn real_realloc_or_bootstrap(ptr: *mut c_void, size: usize) -> *mut c_void {
    let addr = ptr as usize;
    // SHM_FREE_BYPASS fix: during the prepare-lock interval a hook-managed
    // pointer must not reach glibc realloc (which would free the SHM address
    // or read past the extent). Fail safe — NULL + ENOMEM, old block valid,
    // no bytes moved; the interval is microseconds and the caller (a foreign
    // prepare handler) must tolerate realloc failure by the C contract.
    if state::atfork_interval_classify(addr)
        .map(|a| a != state::IntervalPtrAction::Unmanaged)
        .unwrap_or(false)
    {
        set_enomem();
        return core::ptr::null_mut();
    }
    if state::bootstrap_contains(addr) {
        // Copy EXACTLY the old allocation's bytes (from its size header) — never
        // adjacent arena allocations. `move_bytes` further clamps to the new
        // size for a shrink.
        // SAFETY: `addr` is a live bootstrap allocation, so its header is valid.
        let old = unsafe { state::bootstrap_size(addr) };
        return move_bytes(ptr, size, old);
    }
    state::ensure_resolved();
    match state::real_realloc() {
        // SAFETY: `f` is glibc's realloc and `ptr` is a real-heap pointer.
        Some(f) => unsafe { f(ptr, size) },
        // Pre-resolution real-heap realloc (another interposer allocated `ptr`
        // before resolution completed): the real size is unknown, so it cannot safely
        // move it. Fail the realloc — the C contract leaves the old block valid.
        None => {
            set_enomem();
            core::ptr::null_mut()
        }
    }
}

fn real_free_or_noop(ptr: *mut c_void) {
    if state::bootstrap_contains(ptr as usize) {
        return; // a bootstrap allocation is never reclaimed.
    }
    // SHM_FREE_BYPASS fix: during the `atfork` prepare-lock interval, a foreign
    // prepare handler's free reaches this reentrant arm (the re-entrancy guard is
    // set) — a hook-MANAGED address must be handled here (counted no-op / leak)
    // instead of handed to glibc. The classification is PURE
    // (`interval_managed_action`) and the counting lives INSIDE this gated
    // block, so skipping the verdict here also skips the count (and the
    // SHM address then reaches glibc) — which is exactly what the exact-delta
    // counter oracle detects. `NoopWindow` counts nothing (bump arena).
    if let Some(action) = state::interval_managed_action(ptr as usize) {
        use state::IntervalPtrAction as A;
        match action {
            A::NoopWindow => {}
            A::NoopQuarantine => state::note_quarantine_noop_free(),
            A::NoopTombstone => state::note_tombstone_hit(),
            // Leak-and-count (kind 5): registration kept, callback NOT fired
            // (a consumer callback from inside a foreign prepare handler, with
            // both spin locks held, is its own deadlock class).
            A::LeakRegistered => state::note_atfork_interval_leak(),
            // The widened gate boundary (gate armed, locks not provably
            // held): the tables are unreadable, so the pointer cannot be
            // proven unmanaged — leak-and-count (kind 5), never glibc. A leak
            // in a few-instruction signal-only window is bounded and counted;
            // a glibc free of a possibly-SHM address corrupts the heap.
            A::LeakUnclassified => state::note_atfork_interval_leak(),
            A::Unmanaged => unreachable!("interval_managed_action filters Unmanaged"),
        }
        return;
    }
    if let Some(f) = state::real_free() {
        // SAFETY: `f` is glibc's free and `ptr` is a real-heap pointer.
        unsafe { f(ptr) };
    } else {
        // Pre-resolution real-heap free — nothing to free with; leak, counted.
        state::note_pre_resolution_leak();
    }
}

/// Allocate `new_size` real bytes and copy `copy_len` bytes out of `src`, moving
/// a bootstrap/window/segment/quarantine allocation to the real heap. `copy_len`
/// is ALWAYS bounded by the caller to the true extent of `src` (the ledger size,
/// the segment/quarantine bytes remaining, or the bootstrap-arena remaining).
fn move_bytes(src: *mut c_void, new_size: usize, copy_len: usize) -> *mut c_void {
    let dst = real_malloc_or_bootstrap(new_size);
    if !dst.is_null() && !src.is_null() {
        let n = copy_len.min(new_size);
        // SAFETY: dst is a fresh new_size allocation; src holds at least
        // copy_len valid bytes (the caller bounds copy_len to src's real
        // extent). Ranges do not overlap.
        unsafe { core::ptr::copy_nonoverlapping(src as *const u8, dst as *mut u8, n) };
    }
    dst
}

// ── window bump helper ──────────────────────────────────────────────────────

/// Try the armed window first; on overflow or no window, run `fallback`.
fn window_alloc(
    size: usize,
    align: usize,
    zero: bool,
    fallback: impl FnOnce() -> *mut c_void,
) -> *mut c_void {
    let bumped = state::with_window_mut(
        |w| {
            // TEST SEAM (compiled out of shipping builds): the deterministic
            // model of a signal-handler fork poisoning this thread while THIS
            // closure is in flight.
            #[cfg(test)]
            state::mid_bump_poison_hook_for_test();
            Some(w.bump(size, align))
        },
        || None,
    );
    match bumped {
        Some(BumpOutcome::InSlot(addr)) => {
            // CRITICAL re-check: `with_window_mut`'s entry gate runs
            // BEFORE the closure, so a bump IN FLIGHT when a signal-handler
            // fork poisoned this thread resumes in the CHILD and lands here
            // with an in-slot result. Re-check AFTER the closure and BEFORE any
            // use of the address — exposing it (or running the `zero` memset
            // below) would hand the child a pointer into parent-owned
            // MAP_SHARED slot memory. A poisoned thread's bump result is
            // DISCARDED (the leftover window is inert garbage; its advanced
            // cursor/ledger are never consulted again) and the allocation
            // routes to the real allocator — calloc zeroing is preserved by
            // the fallback itself.
            if state::window_poisoned() {
                return fallback();
            }
            let p = addr as *mut c_void;
            if zero {
                // SAFETY: bump returned a live in-slot range of `size` bytes.
                unsafe { core::ptr::write_bytes(p as *mut u8, 0, size) };
            }
            p
        }
        // Overflow (escape already latched inside `bump`) or no window.
        _ => fallback(),
    }
}

/// Route a registered-segment pointer to the release callback (never glibc). If
/// no callback is registered the range is LEAKED (never wrongly freed) —
/// counted, so the misconfiguration is observable. MUST be called with the
/// re-entrancy guard NOT held, so the callback's own allocator calls are
/// classified normally (a callback that frees another registered range routes to
/// release, not glibc).
/// Returns whether the callback actually RAN — the release hand-off closes
/// only then: with no callback the range is LEAKED, and closing the hand-off
/// would strand the leaked range with no coverage, so a second free of it
/// would reach glibc (the very class the hand-off exists to stop). A leaked
/// range keeps its quarantine coverage forever — bounded, counted, and exactly
/// what the atfork child (which also never runs the callback) gets.
fn release_segment(ptr: *mut c_void, cookie: usize) -> bool {
    if let Some(cb) = state::release_callback() {
        // Suspend the caller's window across the
        // callback — its allocations must go to the real allocator rather than
        // bump long-lived bookkeeping into a transient loan slot (and latch
        // spurious escapes), while its frees still classify normally (the
        // guard is NOT held, by design; an in-slot free classifies as a
        // quarantine no-op because the slot is quarantined from arm time).
        state::with_window_suspended(|| {
            // SAFETY: cb is the consumer-registered release callback;
            // ptr/cookie came from a live registration.
            unsafe { cb(ptr, cookie) }
        });
        true
    } else {
        state::note_release_without_callback();
        false
    }
}

// ── the interposed symbols ──────────────────────────────────────────────────

/// # Safety
/// Standard C `malloc` contract.
#[no_mangle]
pub unsafe extern "C" fn malloc(size: usize) -> *mut c_void {
    state::with_guard(
        || window_alloc(size, MAX_ALIGN, false, || real_malloc_or_bootstrap(size)),
        || real_malloc_or_bootstrap(size),
    )
}

/// # Safety
/// Standard C `calloc` contract.
#[no_mangle]
pub unsafe extern "C" fn calloc(nmemb: usize, size: usize) -> *mut c_void {
    state::with_guard(
        || match nmemb.checked_mul(size) {
            None => {
                set_enomem();
                core::ptr::null_mut()
            }
            Some(total) => window_alloc(total, MAX_ALIGN, true, || {
                real_calloc_or_bootstrap(nmemb, size)
            }),
        },
        || real_calloc_or_bootstrap(nmemb, size),
    )
}

/// The plan a guarded `realloc` returns: the pointer to hand back, plus an
/// optional `(ptr, cookie)` to release AFTER the guard is dropped.
struct ReallocPlan {
    dst: *mut c_void,
    release: Option<state::ReleaseTicket>,
}

/// # Safety
/// Standard C `realloc` contract.
#[no_mangle]
pub unsafe extern "C" fn realloc(ptr: *mut c_void, size: usize) -> *mut c_void {
    if ptr.is_null() {
        return malloc(size);
    }
    // glibc's `realloc(p, 0)` FREES `p` and returns
    // NULL. Diverging for hook-managed pointers meant a window pointer took
    // the whole-message ESCAPE path (a spurious `Reallocated` latch plus a
    // leaked `malloc(0)` move) and a quarantined pointer answered NULL +
    // ENOMEM — a fabricated failure for what glibc calls success. Routing
    // size-0 through our own `free` gives every class its free semantics
    // (window/quarantine no-op, segment RELEASE, real heap really freed),
    // then NULL — exactly glibc's contract.
    if size == 0 {
        // SAFETY: forwarding the caller's live pointer to our interposed
        // `free`, per the C contract for `realloc(p, 0)`.
        unsafe { free(ptr) };
        return core::ptr::null_mut();
    }
    let plan = state::with_guard(
        || plan_realloc(ptr, size),
        || ReallocPlan {
            dst: real_realloc_or_bootstrap(ptr, size),
            release: None,
        },
    );
    // The release callback runs OUTSIDE the guard so its own allocator calls are
    // classified normally (M2 fix).
    if let Some(t) = plan.release {
        // Hand-off close: after the callback, and only if it ran (see `free`).
        if release_segment(t.addr as *mut c_void, t.cookie) {
            if let Some(start) = t.handoff {
                state::end_release_handoff(start);
            }
        }
    }
    plan.dst
}

/// The guarded `realloc` planning. A window/segment/quarantine pointer cannot be
/// `realloc`d in place (shared memory, not heap): its bytes are moved to the
/// real heap. A window pointer latches an escape; a segment/quarantine range's
/// bytes are copied UNDER the registry lock (so a concurrent unregister/release
/// cannot free the SHM mid-copy), and a segment's sample is released ONLY if the
/// move succeeded (a failed realloc leaves the old block valid AND registered).
fn plan_realloc(ptr: *mut c_void, size: usize) -> ReallocPlan {
    let addr = ptr as usize;
    if state::bootstrap_contains(addr) {
        return ReallocPlan {
            dst: real_realloc_or_bootstrap(ptr, size),
            release: None,
        };
    }
    // Window (thread-local): move the ledgered bytes off-slot, escape latched.
    if let Some(old) = state::window_realloc_escape(addr) {
        return ReallocPlan {
            dst: move_bytes(ptr, size, old),
            release: None,
        };
    }
    // Segment / quarantine: the copy runs under the lock, pinning the range.
    if let Some(mv) = state::realloc_move_registered(addr, |avail| move_bytes(ptr, size, avail)) {
        return ReallocPlan {
            dst: mv.dst,
            release: mv.release,
        };
    }
    ReallocPlan {
        dst: real_realloc_or_bootstrap(ptr, size),
        release: None,
    }
}

/// Set `errno = ENOMEM` (POSIX requires it on an allocation failure; glibc
/// callers that inspect `errno` would otherwise see a stale value).
fn set_enomem() {
    // SAFETY: __errno_location returns a valid per-thread int pointer.
    unsafe { *libc::__errno_location() = libc::ENOMEM };
}

/// # Safety
/// Standard glibc `reallocarray` contract (`realloc` with an overflow-checked
/// `nmemb * size`). libsystemd / OpenSSL-style code uses it, so it must be
/// interposed too or a `reallocarray` of a registered SHM range would reach
/// glibc `realloc` unclassified.
#[no_mangle]
pub unsafe extern "C" fn reallocarray(ptr: *mut c_void, nmemb: usize, size: usize) -> *mut c_void {
    match nmemb.checked_mul(size) {
        Some(total) => realloc(ptr, total),
        None => {
            set_enomem();
            core::ptr::null_mut()
        }
    }
}

/// # Safety
/// Standard C `free` contract.
#[no_mangle]
pub unsafe extern "C" fn free(ptr: *mut c_void) {
    if ptr.is_null() {
        return;
    }
    // Classify + remove under the guard; release OUTSIDE it (M2 fix).
    let release = state::with_guard(
        || plan_free(ptr),
        || {
            real_free_or_noop(ptr);
            None
        },
    );
    if let Some(t) = release {
        // Close the release hand-off only AFTER the callback returned — the
        // transient quarantine coverage is what a fork child inherits if it
        // lands mid-release — and only if it RAN (a no-callback leak keeps its
        // coverage; see `release_segment`).
        if release_segment(t.addr as *mut c_void, t.cookie) {
            if let Some(start) = t.handoff {
                state::end_release_handoff(start);
            }
        }
    }
}

/// The guarded `free` planning; returns the release ticket iff a release
/// callback must fire outside the guard.
fn plan_free(ptr: *mut c_void) -> Option<state::ReleaseTicket> {
    let addr = ptr as usize;
    if state::bootstrap_contains(addr) {
        return None; // bootstrap arena: no-op.
    }
    match state::classify_and_remove_for_free(addr) {
        // A live armed-window slot on this thread: no-op (bump arena).
        FreeClass::WindowNoop => None,
        // A LIVE disarmed / other-thread quarantine extent: no-op (never
        // glibc), counted so a forgotten `retire_slot` is observable.
        FreeClass::QuarantineNoop => {
            state::note_quarantine_noop_free();
            None
        }
        // A TOMBSTONED (retired) slot extent: no-op (never glibc), counted as a
        // tombstone hit — tombstone-on-retire: a late in-slot free after retire stays a
        // no-op instead of falling through to glibc.
        FreeClass::TombstoneNoop => {
            state::note_tombstone_hit();
            None
        }
        // A forged/registered SHM range: release OUTSIDE the guard, never
        // glibc; its extent is parked in the quarantine until the hand-off
        // closes.
        FreeClass::Release { cookie, handoff } => Some(state::ReleaseTicket {
            addr,
            cookie,
            handoff,
        }),
        FreeClass::RealHeap => {
            real_free_or_noop(ptr);
            None
        }
    }
}

/// # Safety
/// Standard POSIX `posix_memalign` contract. `memptr` must be a valid `void**`.
#[no_mangle]
pub unsafe extern "C" fn posix_memalign(
    memptr: *mut *mut c_void,
    align: usize,
    size: usize,
) -> c_int {
    // EINVAL unless align is a power of two multiple of sizeof(void*).
    if !align.is_power_of_two() || !align.is_multiple_of(core::mem::size_of::<*mut c_void>()) {
        return libc::EINVAL;
    }
    state::with_guard(
        || {
            let p = window_alloc(size, align, false, || {
                real_aligned_or_bootstrap(align, size)
            });
            if p.is_null() {
                return libc::ENOMEM;
            }
            // SAFETY: memptr is a valid out-pointer per the C contract.
            unsafe { *memptr = p };
            0
        },
        || {
            match state::real_posix_memalign() {
                // SAFETY: glibc's posix_memalign with a valid out-pointer.
                Some(f) => unsafe { f(memptr, align, size) },
                None => {
                    let p = real_aligned_or_bootstrap(align, size);
                    if p.is_null() {
                        libc::ENOMEM
                    } else {
                        // SAFETY: valid out-pointer.
                        unsafe { *memptr = p };
                        0
                    }
                }
            }
        },
    )
}

/// # Safety
/// Standard C11 `aligned_alloc` contract.
#[no_mangle]
pub unsafe extern "C" fn aligned_alloc(align: usize, size: usize) -> *mut c_void {
    state::with_guard(
        || {
            window_alloc(size, align, false, || {
                real_aligned_or_bootstrap(align, size)
            })
        },
        || real_aligned_or_bootstrap(align, size),
    )
}

/// # Safety
/// Standard (obsolete) glibc `memalign` contract.
#[no_mangle]
pub unsafe extern "C" fn memalign(align: usize, size: usize) -> *mut c_void {
    state::with_guard(
        || {
            window_alloc(size, align, false, || {
                real_aligned_or_bootstrap(align, size)
            })
        },
        || real_aligned_or_bootstrap(align, size),
    )
}

/// # Safety
/// Standard glibc `malloc_usable_size` contract.
#[no_mangle]
pub unsafe extern "C" fn malloc_usable_size(ptr: *mut c_void) -> usize {
    if ptr.is_null() {
        return 0;
    }
    state::with_guard(|| window_usable_size(ptr), || real_usable_or_zero(ptr))
}

fn window_usable_size(ptr: *mut c_void) -> usize {
    let addr = ptr as usize;
    if state::bootstrap_contains(addr) {
        // EXACT bootstrap allocation size, from its header.
        // SAFETY: `addr` is a live bootstrap allocation.
        return unsafe { state::bootstrap_size(addr) };
    }
    // EXACT allocation size — never larger, or the caller could overwrite the
    // slot past its allocation (a quarantine extent reports 0, same reason).
    match state::classify_usable(addr) {
        UsableOutcome::Exact(n) => n,
        UsableOutcome::RealHeap => real_usable_or_zero(ptr),
    }
}

fn real_usable_or_zero(ptr: *mut c_void) -> usize {
    // SHM_FREE_BYPASS fix: a managed pointer during the prepare interval gets
    // the conservative 0 (matching `classify_usable`'s managed answer), never
    // glibc's walk of a non-heap address.
    if state::atfork_interval_classify(ptr as usize)
        .map(|a| a != state::IntervalPtrAction::Unmanaged)
        .unwrap_or(false)
    {
        return 0;
    }
    if state::bootstrap_contains(ptr as usize) {
        // SAFETY: live bootstrap allocation.
        return unsafe { state::bootstrap_size(ptr as usize) };
    }
    state::ensure_resolved();
    match state::real_malloc_usable_size() {
        // SAFETY: glibc's malloc_usable_size on a real-heap pointer.
        Some(f) => unsafe { f(ptr) },
        None => 0,
    }
}

#[cfg(test)]
mod tests {
    use crate::state;
    use libc::c_void;

    // Poisoning cannot stop a window operation ALREADY IN
    // FLIGHT — a signal-handler fork landing after `with_window_mut` has
    // entered its bump closure resumes the closure in the CHILD, and without a
    // post-closure re-check `window_alloc` exposes (and for calloc, WRITES) an
    // in-slot pointer into parent-owned MAP_SHARED memory. The seam
    // (`arm_mid_bump_poison_for_test`) sets the poison from INSIDE the next
    // bump closure — the deterministic same-thread model of that interleaving
    // (entered unpoisoned, poisoned mid-flight, resumed). In this
    // self-interposed test binary `libc::malloc`/`libc::calloc` ARE our
    // interposers, so the calls below drive the production path.
    // Dropping the post-closure re-check in `window_alloc` makes
    // the seam-phase CALLOC return the in-slot pointer → the off-slot assert
    // fails (its memset also zeroes sentinel bytes); a re-check moved AFTER
    // the zero memset keeps the return off-slot but zeroes sentinel bytes →
    // the sentinel sweep at the end goes red.
    #[test]
    fn an_in_flight_bump_poisoned_mid_closure_never_exposes_an_in_slot_pointer() {
        // Read side of the global-tables guard (state doc) —
        // taken BEFORE CB_LOCK (one consistent order, no cycle).
        let _t = state::GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        // Serialize against the counter-asserting tests:
        // this test's cleanup frees a RETIRED slot, incrementing the global
        // kind-4 counter, which would race `a_post_retire_free...`'s exact
        // delta assertions. CB_LOCK guards ALL process-global interpose state
        // (window / callback / counters), so every test touching it serializes.
        let _l = CB_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // A REAL buffer as the slot: a failing
        // assert below panics, and the panic machinery ALLOCATES — with an
        // armed-unpoisoned window over a FAKE base that allocation would bump
        // into unmapped memory, turning a red assert into a binary-killing
        // SIGSEGV. Real memory keeps failure paths failing red. The whole slot
        // is sentinel-filled so ANY write through the poisoned route (e.g. a
        // calloc memset reordered before the re-check) is detected exactly.
        let slot = unsafe { libc::malloc(0x1000) } as *mut u8;
        assert!(!slot.is_null());
        let base = slot as usize;
        let tail = base + 0x1000;
        for i in 0..0x1000 {
            // SAFETY: slot is a live 0x1000-byte buffer we own.
            unsafe { *slot.add(i) = 0xEE };
        }
        assert_eq!(
            state::arm_window(base, tail),
            state::ArmOutcome::Armed,
            "precondition: a window armed on this thread"
        );
        // CONTROL (anti-tautology): un-poisoned, the window really bumps.
        let ctl = unsafe { libc::malloc(64) } as usize;
        assert!(
            (base..tail).contains(&ctl),
            "control: an armed, un-poisoned window serves in-slot pointers"
        );
        unsafe { libc::free(ctl as *mut c_void) }; // in-window → WindowNoop

        // THE PIN: the poison lands mid-closure; the in-flight bump's result
        // must be DISCARDED and the allocation routed to the real allocator.
        // Driven through CALLOC (zero = true) so BOTH exposure hazards are
        // observable: a dropped re-check RETURNS the in-slot pointer (the
        // off-slot assert), and a re-check reordered AFTER the zero memset
        // WRITES slot bytes (the sentinel sweep below). With malloc here the
        // reordered memset would be invisible — the later calloc runs
        // fully poisoned, entry-gated, and never reaches the InSlot arm.
        state::arm_mid_bump_poison_for_test();
        let z = unsafe { libc::calloc(1, 48) } as *mut u8;
        assert!(
            !z.is_null(),
            "the allocation still succeeds via the real allocator"
        );
        let zu = z as usize;
        assert!(
            !(base..tail).contains(&zu),
            "an in-flight bump poisoned mid-closure must never expose an in-slot pointer"
        );
        for i in 0..48 {
            // SAFETY: z is a live 48-byte real-heap allocation.
            assert_eq!(
                unsafe { *z.add(i) },
                0,
                "calloc zeroing preserved on the poisoned route"
            );
        }
        unsafe { libc::free(z as *mut c_void) };

        // Post-poison, the ENTRY gate serves malloc off-slot too.
        let p = unsafe { libc::malloc(64) } as usize;
        assert!(p != 0 && !(base..tail).contains(&p));
        unsafe { libc::free(p as *mut c_void) };

        // The poisoned route never wrote a single slot byte: the control's
        // bump and the seam calloc's DISCARDED bump reserved (but never wrote)
        // their ranges, and every post-poison allocation was served off-slot —
        // a `zero` memset reordered before the poison re-check would have
        // zeroed the discarded bump's sentinel bytes right here.
        for i in 0..0x1000 {
            // SAFETY: slot is still our live buffer.
            let b = unsafe { *slot.add(i) };
            assert_eq!(
                b, 0xEE,
                "slot byte {i} was written through the poisoned window"
            );
        }

        // cleanup: the arm quarantined [base, tail) globally — retire it, then
        // really free the slot (the armed-but-poisoned window on this thread
        // is never consulted again and drops at thread teardown).
        assert!(state::retire_quarantine(base));
        unsafe { libc::free(slot as *mut c_void) };
    }

    /// Serializes the tests that touch the PROCESS-GLOBAL release callback.
    static CB_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    static CB_ALLOC_ADDR: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    static CB_CALLS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

    unsafe extern "C" fn allocating_release_cb(_p: *mut c_void, _cookie: usize) {
        CB_CALLS.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        // The callback allocates — the hazard: without the window suspension
        // this bumps into the caller's armed window.
        let q = unsafe { libc::malloc(64) } as usize;
        CB_ALLOC_ADDR.store(q, std::sync::atomic::Ordering::SeqCst);
        unsafe { libc::free(q as *mut c_void) };
    }

    // The release callback runs OUTSIDE the guard by
    // design (its frees must classify), but its ALLOCATIONS must not bump the
    // caller's armed window — long-lived callback bookkeeping in a transient
    // loan slot, plus spurious escapes. Dropping the
    // `with_window_suspended` wrap in `release_segment` lands the callback's
    // malloc in-slot → the off-slot assert fails (and the ledger grows).
    #[test]
    fn a_release_callbacks_allocation_never_bumps_the_callers_window() {
        // Read side of the global-tables guard (state doc) —
        // taken BEFORE CB_LOCK (one consistent order, no cycle).
        let _t = state::GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        // Poison-tolerant lock: a sibling CB test's red must stay ITS red, not
        // cascade here as a PoisonError (each test re-initializes the global
        // callback state it uses).
        let _l = CB_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // REAL-INTERPOSITION ORDERING: on Linux EVERY call below
        // runs through our own hook, so `seg` MUST be allocated and registered
        // BEFORE the window is armed — an armed-window malloc would bump `seg`
        // INTO the slot, its free would classify WindowNoop, and the release
        // path under test would never fire (the vacuous shape a wrong order
        // produces).
        let slot = unsafe { libc::malloc(0x1000) } as usize;
        assert!(slot != 0);
        let tail = slot + 0x1000;
        // A separate REAL-heap buffer registered as a take-side segment
        // (window not armed yet, so this malloc cannot bump).
        let seg = unsafe { libc::malloc(256) } as usize;
        assert!(seg != 0);
        state::register_segment_reserving(seg, 256, 0x51).expect("register");
        state::set_release_callback(Some(allocating_release_cb));
        CB_ALLOC_ADDR.store(0, std::sync::atomic::Ordering::SeqCst);
        assert_eq!(state::arm_window(slot, tail), state::ArmOutcome::Armed);
        let ledger_before = state::with_window(|w| w.ledger().len(), || usize::MAX);
        unsafe { libc::free(seg as *mut c_void) }; // → release path, callback runs
        state::set_release_callback(None);
        let cb_ptr = CB_ALLOC_ADDR.load(std::sync::atomic::Ordering::SeqCst);
        assert!(cb_ptr != 0, "the callback ran and allocated");
        assert!(
            !(slot..tail).contains(&cb_ptr),
            "a callback allocation bumped the caller's armed window"
        );
        let ledger_after = state::with_window(|w| w.ledger().len(), || usize::MAX);
        assert_eq!(
            ledger_after, ledger_before,
            "the window never advanced during the callback"
        );
        // `seg`'s real buffer was RELEASED (never glibc-freed): a test-scoped
        // 256-byte leak, by the release contract. Cleanup the window + slot.
        assert!(state::disarm_window().is_some());
        assert!(state::retire_quarantine(slot));
        unsafe { libc::free(slot as *mut c_void) };
    }

    // Tombstone-on-retire: a `free` of an in-slot pointer
    // AFTER `retire_slot` must stay a counted NO-OP — the retired slot's
    // coverage persisting — instead of falling through to glibc (the
    // delete-on-retire corruption). The freed pointer is the slot BASE (covered
    // by the extent AND a valid glibc allocation), so under a delete-instead-of-tombstone bug
    // the fall-through free does not abort — the counter is the discriminator.
    // `retire_quarantine` deleting instead of tombstoning makes the
    // post-retire free a RealHeap real-free → the tombstone counter never moves
    // → the kind-4 assert goes red. The LIVE half also pins the kind-3 vs kind-4
    // split (a naive single-counter would fail one of the two asserts).
    #[test]
    fn a_post_retire_free_of_an_in_slot_pointer_is_a_counted_no_op() {
        // Read side of the global-tables guard (state doc) —
        // taken BEFORE CB_LOCK (one consistent order, no cycle).
        let _t = state::GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        let _l = CB_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let base = unsafe { libc::malloc(0x1000) } as usize;
        assert!(base != 0);
        let tail = base + 0x1000;
        // Arm then disarm: `base` is now covered by a LIVE quarantine extent
        // (the window is gone, so it is not a WindowNoop).
        assert_eq!(state::arm_window(base, tail), state::ArmOutcome::Armed);
        assert!(state::disarm_window().is_some());

        // LIVE half: a free is a QuarantineNoop (kind 3), NOT a tombstone hit.
        let (c3, c4) = (state::counter(3), state::counter(4));
        unsafe { libc::free(base as *mut c_void) };
        assert_eq!(
            state::counter(3),
            c3 + 1,
            "a live-quarantine free counts kind 3"
        );
        assert_eq!(state::counter(4), c4, "and NOT kind 4 while still live");

        // Retire → TOMBSTONE (does not delete).
        assert!(state::retire_quarantine(base));

        // TOMBSTONE half: the post-retire free is a counted no-op (kind 4),
        // never glibc — tombstone-on-retire. (`base` stays covered, so it is leaked:
        // a bounded, test-scoped leak, exactly the production contract — an
        // in-slot pointer is never returned to glibc.)
        let (c3, c4) = (state::counter(3), state::counter(4));
        unsafe { libc::free(base as *mut c_void) };
        assert_eq!(
            state::counter(4),
            c4 + 1,
            "a post-retire in-slot free is a counted tombstone no-op, never glibc"
        );
        assert_eq!(state::counter(3), c3, "and NOT a live-quarantine no-op");

        // Cleanup: purge the tombstone entry (retire tombstones, so unregister).
        assert!(state::with_quarantine(|q| q.unregister(base)));
    }

    // SHM_FREE_BYPASS: during the `atfork` prepare-lock
    // interval, a foreign prepare handler's free of a hook-MANAGED pointer
    // reaches this reentrant arm (the re-entrancy guard is set) and must be classified
    // — NOT handed to glibc. Drives the REAL interposed free between the REAL
    // `atfork_lock`/`atfork_unlock` (this binary self-interposes on Linux):
    // (a) quarantined → counted no-op kind 3, never glibc; (b) registered →
    // leak-and-count kind 5, registration KEPT, callback NOT fired;
    // (c) ordinary unmanaged traffic still completes (no wedge).
    //
    // EXACT deltas under CB_LOCK — NOT a lower bound: kinds 3/5 are
    // process-global, so a parallel test's kind-3 bump satisfies `> before`
    // even with classification bypassed (a lower bound passes code that
    // skips classification). CB_LOCK serializes every counter-touching test, so `== +1`
    // is reliable, and the pure/side-effect split (`interval_managed_action`
    // classifies, this arm counts) means code that ignores the verdict
    // also skips the count → the delta stays 0 → red.
    #[test]
    fn a_foreign_prepare_handler_free_of_a_managed_pointer_stays_classified() {
        // Read side of the global-tables guard (state doc) —
        // taken BEFORE CB_LOCK (one consistent order, no cycle).
        let _t = state::GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        let _l = CB_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // Managed fixtures BEFORE the interval (normal execution). REAL buffers:
        // each base is a valid glibc pointer, so a classification bypass CORRUPTS (double-frees
        // a live buffer) rather than aborting — the counter delta, not a crash,
        // is the discriminator.
        let slot = unsafe { libc::malloc(256) } as usize;
        assert!(slot != 0);
        // Control guard around the insertion: a
        // quarantine Vec growth under QUARANTINE_LOCK frees its old buffer, and
        // in this self-interposed binary that free re-enters classification and
        // spins on the very lock this thread holds — the guard routes it to the
        // reentrant real-allocator arm instead.
        state::with_control_guard(|| {
            state::with_quarantine(|q| {
                q.insert_or_extend(slot, 256);
            })
        });
        let seg = unsafe { libc::malloc(256) } as usize;
        assert!(seg != 0);
        state::register_segment_reserving(seg, 256, 0x1F).expect("register");

        // THE INTERVAL: both spin locks + guard held, exactly as prepare leaves
        // it. (CB_LOCK is a std Mutex, unrelated to the spin locks; no other
        // counter test holds a spin lock while waiting on CB_LOCK, so no
        // deadlock.)
        state::atfork_lock();
        // (a) quarantined → EXACTLY one kind-3 no-op, never glibc.
        let c3 = state::counter(3);
        unsafe { libc::free(slot as *mut c_void) };
        assert_eq!(
            state::counter(3),
            c3 + 1,
            "a quarantined free during the interval is a counted no-op (never glibc)"
        );
        // (b) registered → EXACTLY one kind-5 leak; registration KEPT.
        let c5 = state::counter(5);
        unsafe { libc::free(seg as *mut c_void) };
        assert_eq!(
            state::counter(5),
            c5 + 1,
            "a registered free during the interval leaks-and-counts (never glibc)"
        );
        // (c) ordinary unmanaged traffic completes (no wedge) —
        // and moves NEITHER managed counter.
        let (c3b, c5b) = (state::counter(3), state::counter(5));
        let p = unsafe { libc::malloc(64) };
        assert!(!p.is_null());
        unsafe { libc::free(p) };
        assert_eq!(
            state::counter(3),
            c3b,
            "unmanaged free is not a kind-3 no-op"
        );
        assert_eq!(
            state::counter(5),
            c5b,
            "unmanaged free is not a kind-5 leak"
        );
        state::atfork_unlock();

        // Post-interval: the registration SURVIVED (a later normal free still
        // routes to Release — the interval leak never destroyed coverage).
        assert!(
            state::with_registry(|r| r.classify(seg + 8)).is_some(),
            "the interval leak keeps the registration intact"
        );
        // Cleanup: purge both managed entries; the leaked buffers are
        // test-scoped by the managed-coverage contract.
        state::with_registry(|r| assert!(r.unregister(seg)));
        assert!(state::with_quarantine(|q| q.unregister(slot)));
    }

    // glibc's `realloc(p, 0)` FREES `p` and returns
    // NULL. Every hook-managed class must follow: window → no-op free with NO
    // `Reallocated` escape; quarantine → no-op free with errno UNTOUCHED (a
    // NULL + ENOMEM answer fabricates a failure); segment → RELEASE fires.
    // Dropping the size-0 branch in `realloc` sends the window pointer down
    // the escape path (non-null return + a latched escape) and the quarantined
    // pointer to NULL + ENOMEM.
    #[test]
    fn realloc_size_zero_frees_by_class_and_returns_null() {
        // Read side of the global-tables guard (state doc) —
        // taken BEFORE CB_LOCK (one consistent order, no cycle).
        let _t = state::GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        // Poison-tolerant (see the sibling CB test).
        let _l = CB_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // Window class.
        let slot = unsafe { libc::malloc(0x1000) } as usize;
        assert!(slot != 0);
        let tail = slot + 0x1000;
        assert_eq!(state::arm_window(slot, tail), state::ArmOutcome::Armed);
        let p = unsafe { libc::malloc(64) } as usize;
        assert!((slot..tail).contains(&p), "precondition: in-slot pointer");
        let r = unsafe { libc::realloc(p as *mut c_void, 0) };
        assert!(
            r.is_null(),
            "realloc(window_ptr, 0) is free semantics: NULL"
        );
        assert_eq!(
            state::disarm_window(),
            Some(None),
            "free semantics latches NO Reallocated escape"
        );
        // Quarantine class: the slot extent is quarantined from arm time.
        unsafe { *libc::__errno_location() = 0 };
        let r2 = unsafe { libc::realloc(p as *mut c_void, 0) };
        assert!(r2.is_null());
        assert_eq!(
            unsafe { *libc::__errno_location() },
            0,
            "a quarantined no-op free is SUCCESS — no fabricated ENOMEM"
        );
        // Segment class: the release fires exactly once.
        let seg = unsafe { libc::malloc(256) } as usize;
        assert!(seg != 0);
        state::register_segment_reserving(seg, 256, 0x77).expect("register");
        state::set_release_callback(Some(allocating_release_cb));
        let calls_before = CB_CALLS.load(std::sync::atomic::Ordering::SeqCst);
        let r3 = unsafe { libc::realloc(seg as *mut c_void, 0) };
        state::set_release_callback(None);
        assert!(
            r3.is_null(),
            "realloc(segment_ptr, 0) releases and returns NULL"
        );
        assert_eq!(
            CB_CALLS.load(std::sync::atomic::Ordering::SeqCst),
            calls_before + 1,
            "the release callback fired exactly once"
        );
        // Real heap: glibc frees and returns NULL (its own contract).
        let q = unsafe { libc::malloc(64) };
        assert!(!q.is_null());
        let r4 = unsafe { libc::realloc(q, 0) };
        assert!(r4.is_null());
        // cleanup (seg's buffer was released → test-scoped leak, as above).
        assert!(state::retire_quarantine(slot));
        unsafe { libc::free(slot as *mut c_void) };
    }
}
