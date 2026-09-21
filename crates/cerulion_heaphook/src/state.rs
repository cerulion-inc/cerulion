// SPDX-License-Identifier: AGPL-3.0-only
//! Process + thread state for the Linux/GNU interposition layer: the resolved
//! real-allocator pointers, the dlsym-bootstrap arena, the per-thread
//! re-entrancy guard and borrow window, and the global segment registry +
//! release callback.
//!
//! Everything here is `cfg`-gated to Linux/GNU (its only `use` site is the
//! interposer); the LOGIC it wraps lives in the pure `#[cfg]`-free modules.

use core::cell::RefCell;
use core::ffi::{c_char, c_int, c_void};
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};

use crate::abi::ReleaseCallback;
use crate::classify::{classify_ptr, PtrClass};
use crate::recursion::ReentryFlag;
use crate::registry::SegmentRegistry;
use crate::window::{align_up, BorrowWindow};

// ── Real allocator function types ───────────────────────────────────────────

pub type MallocFn = unsafe extern "C" fn(usize) -> *mut c_void;
pub type FreeFn = unsafe extern "C" fn(*mut c_void);
pub type CallocFn = unsafe extern "C" fn(usize, usize) -> *mut c_void;
pub type ReallocFn = unsafe extern "C" fn(*mut c_void, usize) -> *mut c_void;
pub type PosixMemalignFn = unsafe extern "C" fn(*mut *mut c_void, usize, usize) -> c_int;
pub type AlignedAllocFn = unsafe extern "C" fn(usize, usize) -> *mut c_void;
pub type MallocUsableSizeFn = unsafe extern "C" fn(*mut c_void) -> usize;

// The resolved real symbols (0 until `ensure_resolved` runs). Stored as `usize`
// and transmuted to the fn types only when non-zero.
static REAL_MALLOC: AtomicUsize = AtomicUsize::new(0);
static REAL_FREE: AtomicUsize = AtomicUsize::new(0);
static REAL_CALLOC: AtomicUsize = AtomicUsize::new(0);
static REAL_REALLOC: AtomicUsize = AtomicUsize::new(0);
static REAL_POSIX_MEMALIGN: AtomicUsize = AtomicUsize::new(0);
static REAL_ALIGNED_ALLOC: AtomicUsize = AtomicUsize::new(0);
static REAL_MEMALIGN: AtomicUsize = AtomicUsize::new(0);
static REAL_MALLOC_USABLE_SIZE: AtomicUsize = AtomicUsize::new(0);

static RESOLVED: AtomicBool = AtomicBool::new(false);
static RESOLVING: AtomicBool = AtomicBool::new(false);

/// Resolve every real allocator symbol via `dlsym(RTLD_NEXT, …)`, once.
///
/// Called eagerly from the load constructor and lazily by any interposer that
/// finds a symbol unresolved. The single-resolver latch ([`RESOLVING`]) is what
/// breaks the classic "dlsym calls `calloc`" recursion: while one thread is
/// resolving, any re-entrant `calloc`/`malloc` sees the symbol still `0` and
/// serves the bootstrap arena instead of recursing into `dlsym`.
pub fn ensure_resolved() {
    if RESOLVED.load(Ordering::Acquire) {
        return;
    }
    if RESOLVING.swap(true, Ordering::AcqRel) {
        // Another thread is resolving; callers serve bootstrap until RESOLVED.
        return;
    }
    // SAFETY: dlsym with RTLD_NEXT and a NUL-terminated name is defined on
    // glibc; each name is a valid C allocator symbol. The results are function
    // pointers we only ever transmute back to the exact matching signature.
    unsafe {
        REAL_MALLOC.store(dlnext(c"malloc"), Ordering::Release);
        REAL_FREE.store(dlnext(c"free"), Ordering::Release);
        REAL_CALLOC.store(dlnext(c"calloc"), Ordering::Release);
        REAL_REALLOC.store(dlnext(c"realloc"), Ordering::Release);
        REAL_POSIX_MEMALIGN.store(dlnext(c"posix_memalign"), Ordering::Release);
        REAL_ALIGNED_ALLOC.store(dlnext(c"aligned_alloc"), Ordering::Release);
        REAL_MEMALIGN.store(dlnext(c"memalign"), Ordering::Release);
        REAL_MALLOC_USABLE_SIZE.store(dlnext(c"malloc_usable_size"), Ordering::Release);
    }
    RESOLVED.store(true, Ordering::Release);
}

/// `dlsym(RTLD_NEXT, name)` as a `usize` (0 if the symbol is absent — e.g.
/// `memalign` on a stripped libc; callers fall back).
///
/// # Safety
/// dlsym with `RTLD_NEXT` and a valid C string is defined on glibc.
unsafe fn dlnext(name: &core::ffi::CStr) -> usize {
    libc::dlsym(libc::RTLD_NEXT, name.as_ptr() as *const c_char) as usize
}

macro_rules! real_getter {
    ($vis:vis $name:ident, $slot:ident, $ty:ty) => {
        /// The resolved real symbol, or `None` if unresolved/absent.
        $vis fn $name() -> Option<$ty> {
            let p = $slot.load(Ordering::Acquire);
            if p == 0 {
                None
            } else {
                // SAFETY: a non-zero slot holds a pointer dlsym returned for
                // exactly this signature.
                Some(unsafe { core::mem::transmute::<usize, $ty>(p) })
            }
        }
    };
}

real_getter!(pub real_malloc, REAL_MALLOC, MallocFn);
real_getter!(pub real_free, REAL_FREE, FreeFn);
real_getter!(pub real_calloc, REAL_CALLOC, CallocFn);
real_getter!(pub real_realloc, REAL_REALLOC, ReallocFn);
real_getter!(pub real_posix_memalign, REAL_POSIX_MEMALIGN, PosixMemalignFn);
real_getter!(pub real_aligned_alloc, REAL_ALIGNED_ALLOC, AlignedAllocFn);
// `memalign` shares `aligned_alloc`'s `(align, size) -> ptr` shape.
real_getter!(pub real_memalign, REAL_MEMALIGN, AlignedAllocFn);
real_getter!(pub real_malloc_usable_size, REAL_MALLOC_USABLE_SIZE, MallocUsableSizeFn);

// ── dlsym-bootstrap arena ───────────────────────────────────────────────────
//
// Allocations that arrive before `ensure_resolved` has stored the real symbols
// (another preloaded library's constructor, or the `dlsym` resolution itself
// calling `calloc`) are served from this fixed BSS arena. A `free` of a
// bootstrap pointer is a no-op; a `realloc` copies out to the real heap. After
// resolution the arena is never touched again.

const BOOTSTRAP_CAP: usize = 1 << 20; // 1 MiB, zero-init BSS (no on-disk cost)

#[repr(align(16))]
struct Bootstrap(core::cell::UnsafeCell<[u8; BOOTSTRAP_CAP]>);
// SAFETY: access is via atomic bump of BOOTSTRAP_CURSOR; each returned range is
// disjoint, and a bootstrap pointer is never handed to the real allocator.
unsafe impl Sync for Bootstrap {}

static BOOTSTRAP: Bootstrap = Bootstrap(core::cell::UnsafeCell::new([0u8; BOOTSTRAP_CAP]));
static BOOTSTRAP_CURSOR: AtomicUsize = AtomicUsize::new(0);

#[inline]
fn bootstrap_base() -> usize {
    BOOTSTRAP.0.get() as usize
}

/// Whether `ptr` is a live bootstrap allocation. The lower bound is
/// `base + BOOTSTRAP_HDR`, not `base`: every allocation's user pointer sits at
/// least a header past the arena start, so a pointer in `[base, base + HDR)` is
/// never one we handed out — excluding it keeps [`bootstrap_size`]'s `*(ptr - 8)`
/// header read inside the arena (guarding the C-contract-UB precondition of a
/// bogus pointer that merely happens to land in the arena range).
#[inline]
pub fn bootstrap_contains(ptr: usize) -> bool {
    let base = bootstrap_base();
    ptr >= base + BOOTSTRAP_HDR && ptr < base + BOOTSTRAP_CAP
}

/// Bytes reserved before each bootstrap allocation for its size header. 16 (not
/// 8) so the returned pointer keeps 16-byte alignment while an 8-byte size sits
/// in the header word immediately before it.
const BOOTSTRAP_HDR: usize = 16;

/// The EXACT size of a bootstrap allocation, read from its size header (the 8
/// bytes immediately before `ptr`). Used by `realloc`/`malloc_usable_size` so a
/// growing realloc copies only the real old bytes — never adjacent arena
/// allocations (an information leak).
///
/// # Safety
/// `ptr` must be a live bootstrap allocation (`bootstrap_contains(ptr)`), so its
/// header word is a valid, initialized `usize`.
pub unsafe fn bootstrap_size(ptr: usize) -> usize {
    *((ptr - 8) as *const usize)
}

/// Bump `size` bytes at `align` from the arena, zeroing if `zero`. Each
/// allocation is preceded by an 8-byte size header (see [`bootstrap_size`]).
/// Returns null on exhaustion (the arena is generous; exhaustion means an
/// unusual pre-resolution allocation storm and is counted).
pub fn bootstrap_alloc(size: usize, align: usize, zero: bool) -> *mut c_void {
    let align = align.max(16);
    loop {
        let cur = BOOTSTRAP_CURSOR.load(Ordering::Acquire);
        let base = bootstrap_base();
        // Reserve the header, then align the USER pointer; the 8-byte size word
        // lives at `user - 8`, within the reserved (disjoint) span.
        let after_hdr = match (base + cur).checked_add(BOOTSTRAP_HDR) {
            Some(a) => a,
            None => {
                note_bootstrap_exhausted();
                return core::ptr::null_mut();
            }
        };
        let user_abs = match align_up(after_hdr, align) {
            Some(a) => a,
            None => {
                note_bootstrap_exhausted();
                return core::ptr::null_mut();
            }
        };
        // Reserve at least ONE byte (even for a size-0 request) so `end` is
        // strictly greater than the user offset: a size-0 allocation whose
        // aligned user address is exactly `base + BOOTSTRAP_CAP` would otherwise
        // pass the `end > CAP` gate (`end == CAP`) and hand back a ONE-PAST-the-
        // arena pointer that `bootstrap_contains` rejects — a later free of it
        // would then reach glibc. With `reserve >= 1`, `end == CAP` only for a
        // user address strictly inside the arena, and `end == CAP + 1` (the
        // one-past case) is rejected here.
        let reserve = size.max(1);
        let end = match (user_abs - base).checked_add(reserve) {
            Some(e) => e,
            None => {
                note_bootstrap_exhausted();
                return core::ptr::null_mut();
            }
        };
        if end > BOOTSTRAP_CAP {
            note_bootstrap_exhausted();
            return core::ptr::null_mut();
        }
        if BOOTSTRAP_CURSOR
            .compare_exchange(cur, end, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
        {
            let p = user_abs as *mut u8;
            // SAFETY: [user_abs-8, user_abs+size) is a freshly claimed, disjoint
            // sub-range of the arena; the header word is 8-aligned.
            unsafe {
                *((user_abs - 8) as *mut usize) = size;
                if zero {
                    core::ptr::write_bytes(p, 0, size);
                }
            }
            return p as *mut c_void;
        }
    }
}

// ── Thread-local guard + borrow window ──────────────────────────────────────

thread_local! {
    static GUARD: ReentryFlag = const { ReentryFlag::new() };
    static WINDOW: RefCell<Option<BorrowWindow>> = const { RefCell::new(None) };
    /// See [`window_poisoned`].
    static WINDOW_POISONED: core::cell::Cell<bool> = const { core::cell::Cell::new(false) };
    /// Whether OUR `atfork` prepare handler set the re-entrancy guard on this
    /// thread — the parent handler must clear only a
    /// guard it owns, never one an enclosing interposer frame holds.
    static ATFORK_GUARD_HELD: core::cell::Cell<bool> = const { core::cell::Cell::new(false) };
    /// Whether the `atfork` prepare-lock INTERVAL GATE is armed on this thread.
    /// Published by `atfork_lock` FIRST — before
    /// the re-entrancy guard is set, and hence before the locks — and cleared
    /// LAST on both exits (`atfork_unlock` and the child reset: after the
    /// releases AND after the guard clears) — deliberately WIDER than both
    /// the locks-held span and the guard's atfork span, so a signal landing
    /// between ANY two of those steps can never observe "not in the interval"
    /// while the guard is held or a lock is (or is about to be) held. The
    /// guard containment is load-bearing: the guard is what routes a free to
    /// the REENTRANT arm, whose only defense is this gate — see the INVARIANT
    /// on `atfork_lock`. Widening is the SAFE direction: a managed pointer
    /// freed in the widened boundary is leaked-and-counted (kind 5,
    /// `IntervalPtrAction::LeakUnclassified`) instead of falling through to
    /// glibc `free` of an SHM address (heap corruption). See
    /// `atfork_interval_classify`.
    static ATFORK_INTERVAL_ARMED: core::cell::Cell<bool> = const { core::cell::Cell::new(false) };
    /// Whether THIS thread actually holds BOTH global spin locks (set by
    /// `atfork_lock` AFTER acquiring, cleared by `atfork_unlock` BEFORE
    /// releasing — and by the child reset FIRST, before its own table
    /// mutations: the fold mutates the tables with the locks force-open, so
    /// the child must never classify under this flag).
    /// While set, the reentrant arm may
    /// CLASSIFY hook-managed addresses by reading the guarded tables directly
    /// — sound precisely because no other thread can be mutating them (they
    /// spin on the locks we hold) and this thread cannot re-enter `with_*`
    /// (the guard routes everything to the reentrant arm). NOT the interval
    /// gate (that is `ATFORK_INTERVAL_ARMED` above): this flag only answers
    /// "are the tables readable", and keeps its acquire-side ordering because
    /// mis-reading it FALSE while the locks are held merely degrades to the
    /// conservative leak arm — the safe direction — while setting it before
    /// the locks would re-open the lock-free-read race the SAFETY comment in
    /// `atfork_interval_classify` depends on. See `atfork_interval_classify`
    /// (SHM_FREE_BYPASS fix).
    static ATFORK_LOCKS_HELD: core::cell::Cell<bool> = const { core::cell::Cell::new(false) };
    /// Depth of ACTIVE release-callback frames on this thread — see
    /// [`with_window_suspended`].
    static WINDOW_SUSPENDED: core::cell::Cell<u32> = const { core::cell::Cell::new(0) };
}

/// Run `f` with this thread's window SUSPENDED. The
/// release callback runs OUTSIDE the re-entrancy guard by design — its frees
/// must classify (a callback freeing another registered range routes to
/// release, not glibc) — but that also let its ALLOCATIONS bump into any window
/// armed on the calling thread: long-lived callback bookkeeping written into a
/// transient loan slot, plus spurious escapes. While suspended,
/// `with_window`/`with_window_mut` report absent, so callback allocations go to
/// the real allocator while its in-slot frees still classify as quarantine
/// no-ops (the slot is quarantined from arm time). A DEPTH, not a flag: a
/// callback's free of another registered range fires a NESTED release.
pub(crate) fn with_window_suspended<R>(f: impl FnOnce() -> R) -> R {
    let entered = WINDOW_SUSPENDED
        .try_with(|d| d.set(d.get().saturating_add(1)))
        .is_ok();
    let out = f();
    if entered {
        let _ = WINDOW_SUSPENDED.try_with(|d| d.set(d.get().saturating_sub(1)));
    }
    out
}

/// Whether a release callback is active on this thread (window suspended).
fn window_suspended() -> bool {
    WINDOW_SUSPENDED.try_with(|d| d.get() > 0).unwrap_or(false)
}

/// Whether THIS thread's window machinery is POISONED — set only by the
/// `atfork` child handler when the inherited `WINDOW` cell was mid-borrow and
/// could not be disarmed ([`clear_window_forget_ledger`]).
/// While set, every window entry point on this thread takes its `absent` arm,
/// so the child can never allocate from (or classify against) the parent-owned
/// MAP_SHARED slot through the leftover window — without the latch, the
/// child's next malloc re-entered `with_window_mut` on the inherited window
/// and bumped parent-owned slot memory (cross-process corruption, the exact
/// class this crate exists to prevent). THREAD-local by design: the stuck
/// window lives only on the FORKING thread's TLS (a new child thread starts
/// with a `None` window, so nothing else needs the latch), and a
/// process-global flag would let one poisoned thread disable the window
/// machinery process-wide. Never cleared: the stuck cell stays un-disarmable
/// for the life of the (child) process, so its thread stays poisoned.
///
/// Checked at ENTRY by every window entry point, AND RE-CHECKED by
/// `window_alloc` after its bump closure returns: the
/// entry gate cannot stop an operation already IN FLIGHT when the fork landed
/// — the interrupted closure resumes in the child and completes its bump — so
/// the EXPOSURE boundary re-checks before returning (or zeroing) an in-slot
/// address and discards a poisoned thread's bump result.
pub(crate) fn window_poisoned() -> bool {
    WINDOW_POISONED.try_with(|p| p.get()).unwrap_or(false)
}

#[cfg(test)]
thread_local! {
    /// TEST SEAM (unit-test builds only): a ONE-SHOT flag that makes
    /// the next window-bump closure set `WINDOW_POISONED` from INSIDE itself —
    /// the deterministic same-thread model of a signal-handler `fork()` that
    /// poisons this thread while a `with_window_mut` operation is IN FLIGHT
    /// (entered before the fork, resumed in the child after the handler).
    static MID_BUMP_POISON_ARMED: core::cell::Cell<bool> = const { core::cell::Cell::new(false) };
}

/// Arm the one-shot mid-bump poison (see [`MID_BUMP_POISON_ARMED`]).
#[cfg(test)]
pub fn arm_mid_bump_poison_for_test() {
    MID_BUMP_POISON_ARMED.with(|f| f.set(true));
}

/// TEST ACCESSOR: whether the re-entrancy guard is set on this
/// thread — the observable for the atfork guard-discipline pins.
#[cfg(test)]
pub fn guard_active_for_test() -> bool {
    GUARD.try_with(|g| g.is_active()).unwrap_or(false)
}

/// TEST ACCESSOR: force the guard SET, modelling a fork issued
/// from a signal handler that interrupted an interposer frame (the stuck-guard
/// shape the child clear exists for).
#[cfg(test)]
pub fn set_guard_for_test() {
    let _ = GUARD.try_with(|g| {
        let _ = g.try_set();
    });
}

/// Called by `window_alloc`'s bump closure in test builds; fires at most once
/// per arming, poisoning this thread mid-flight.
#[cfg(test)]
pub fn mid_bump_poison_hook_for_test() {
    MID_BUMP_POISON_ARMED.with(|f| {
        if f.get() {
            f.set(false);
            WINDOW_POISONED.with(|p| p.set(true));
        }
    });
}

/// Run `f` with the re-entrancy guard held (our logic active). If the guard was
/// already held on this thread (a re-entrant allocator call), OR the guard
/// thread-local has been destroyed (late thread teardown), runs `reentrant`
/// instead — which must go straight to the real allocator. `try_with` is
/// load-bearing: allocator calls arrive during thread teardown (glibc/libstdc++
/// pthread-key destructors free memory after Rust's thread-locals are gone), and
/// a `with` there would PANIC and abort the process out of `extern "C" free`.
#[inline]
pub fn with_guard<R>(f: impl FnOnce() -> R, reentrant: impl FnOnce() -> R) -> R {
    // Set the flag (if clear) inside try_with; run f OUTSIDE so the flag stays
    // set for the duration — a re-entrant allocator call during f() re-enters
    // here, sees the flag set, and takes the real-allocator branch.
    match GUARD.try_with(|g| g.try_set()) {
        Ok(true) => {
            let out = f();
            let _ = GUARD.try_with(|g| g.clear());
            out
        }
        Ok(false) => reentrant(), // re-entrant → real allocator (the caller is us)
        // TLS DESTROYED (late thread teardown): still run `f`, i.e. still
        // CLASSIFY. The re-entrancy guard is a thread-local, but the segment
        // registry and quarantine are GLOBAL — a destructor freeing a
        // quarantined/registered SHM pointer must NOT bypass classification and
        // reach glibc `free`. Running `f` unguarded is safe: the window
        // thread-local reads absent (`try_with` → `absent`), and any allocation
        // `f` triggers re-enters here, hits this same Err arm, and its window
        // bump falls through to the real allocator — no recursion, no bump.
        Err(_) => f(),
    }
}

/// Run a control-API body with the guard held — acquiring it if clear, or
/// running under the already-held guard if re-entrant. Either way `f` runs with
/// the guard active (so a bookkeeping allocation bypasses an armed window). A
/// destroyed guard TLS simply runs `f` directly.
#[inline]
pub fn with_control_guard<R>(f: impl FnOnce() -> R) -> R {
    match GUARD.try_with(|g| g.try_set()) {
        Ok(acquired) => {
            let out = f();
            if acquired {
                let _ = GUARD.try_with(|g| g.clear());
            }
            out
        }
        Err(_) => f(),
    }
}

/// The outcome of an [`arm_window`] call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmOutcome {
    /// A window was armed.
    Armed,
    /// A window is already armed on this thread (nested arm).
    AlreadyArmed,
    /// The window thread-local is being torn down — an arm is impossible.
    Unavailable,
}

/// Arm a window on this thread over `[base, tail_limit)`.
///
/// On success it ALSO registers the whole slot `[base, tail_limit)` into the
/// GLOBAL quarantine (keyed by `base`). This is what makes the borrow window
/// sound beyond its own thread: the window itself is thread-local, so a
/// `free`/`realloc` of an in-slot pointer on ANOTHER thread, or after the
/// filling thread exits with the window still armed, or in a fork child, would
/// otherwise consult an absent window and route the shared-memory address to
/// glibc `free`. With the slot in the global quarantine from ARM time, every
/// such path classifies it as a no-op instead. The owning thread still gets
/// bump/escape semantics because the thread-local window is checked FIRST.
/// `retire_slot(base)` removes the quarantine entry when the sample is reclaimed.
pub fn arm_window(base: usize, tail_limit: usize) -> ArmOutcome {
    // A poisoned thread's cell may be stuck mid-borrow — even the
    // armability READ below could panic; refuse before touching it.
    if window_poisoned() {
        return ArmOutcome::Unavailable;
    }
    // Decide armability WITHOUT publishing the window yet.
    match WINDOW.try_with(|w| w.borrow().is_none()) {
        Ok(false) => ArmOutcome::AlreadyArmed,
        Err(_) => ArmOutcome::Unavailable,
        Ok(true) => {
            // COVERAGE BEFORE EXPOSURE: put the slot into the GLOBAL quarantine
            // BEFORE the thread-local window becomes visible, so there is no gap
            // in which another thread could free an in-slot pointer and miss both
            // the (this-thread-only) window and the (not-yet-inserted) quarantine
            // → glibc free of SHM. (No in-slot allocation exists yet at arm time,
            // but ordering coverage-first is the sound invariant regardless.)
            //
            // CHILD-FOLD INVARIANT (continuous): at EVERY
            // instant a fork could occur, `quarantine.spare_capacity() >=
            // registry.len()`. The reserve therefore runs BEFORE the insert that
            // consumes a slot, inside ONE `with_both` scope: `atfork`'s prepare
            // handler must acquire both locks, so no fork can snapshot the
            // intermediate state (an earlier form re-reserved AFTER the insert,
            // leaving a fork-visible under-reserved window). `+ 1` covers this
            // insert's own possible push. The caller holds the control guard
            // (the `guarded()` export), so the reserve's own malloc/free take
            // the reentrant real-heap arm and cannot re-enter these locks.
            if tail_limit > base {
                // The reserve may GROW the quarantine
                // `Vec`, FREEING its old buffer — and a free while BOTH locks
                // are held must take the reentrant arm, or it classifies into
                // `with_both` and spins on the locks this thread already
                // holds. The `guarded()` export already holds the control
                // guard, but acquiring it HERE too (idempotent nesting) makes
                // the fn safe for DIRECT callers — unit tests included, whose
                // self-interposed binary routes that free through our own
                // hook.
                with_control_guard(|| {
                    with_both(|reg, quar| {
                        quar.reserve(reg.len() + 1);
                        quar.insert_or_extend(base, tail_limit - base);
                    })
                });
            }
            // Now publish the window. Same-thread, so no TOCTOU with the check
            // above; a window cannot appear between the check and here.
            WINDOW
                .try_with(|w| *w.borrow_mut() = Some(BorrowWindow::new(base, tail_limit)))
                .map(|()| ArmOutcome::Armed)
                .unwrap_or(ArmOutcome::Unavailable)
        }
    }
}

/// Disarm this thread's window and return its latched escape (if a window was
/// armed). The slot's `[base, tail_limit)` extent is ALREADY in the quarantine
/// (registered at [`arm_window`]), so nothing is folded here — the entry simply
/// outlives the window until `retire_slot(base)`. Taking the window with
/// `.take()` (slot already `None`) means the ledger `Vec`'s drop-time `free`
/// cannot be misclassified as a window pointer.
pub fn disarm_window() -> Option<Option<crate::window::EscapeKind>> {
    if window_poisoned() {
        return None; // The stuck cell must never be touched.
    }
    let taken = WINDOW.try_with(|w| w.borrow_mut().take()).ok().flatten();
    taken.map(|w| w.escape())
}

/// Run `f` against this thread's armed window (mutable), or `absent` if none (or
/// the TLS is destroyed — see [`with_guard`] for why `try_with` is required).
pub fn with_window_mut<R>(f: impl FnOnce(&mut BorrowWindow) -> R, absent: impl FnOnce() -> R) -> R {
    // A poisoned cell may be stuck mid-borrow — never touch it.
    // A release callback's allocations must not bump
    // the caller's window — suspended reads as absent.
    if window_poisoned() || window_suspended() {
        return absent();
    }
    match WINDOW.try_with(|w| w.borrow_mut().as_mut().map(f)) {
        Ok(Some(r)) => r,
        _ => absent(),
    }
}

/// Run `f` against this thread's armed window (shared), or `absent` if none (or
/// the TLS is destroyed).
pub fn with_window<R>(f: impl FnOnce(&BorrowWindow) -> R, absent: impl FnOnce() -> R) -> R {
    // A stuck MUT borrow would panic even `borrow()`.
    // Suspended (release callback active) reads as absent.
    if window_poisoned() || window_suspended() {
        return absent();
    }
    match WINDOW.try_with(|w| w.borrow().as_ref().map(f)) {
        Ok(Some(r)) => r,
        _ => absent(),
    }
}

/// Disarm this thread's window WITHOUT freeing its ledger `Vec` — for the
/// `atfork` CHILD handler ONLY. Dropping the window frees the ledger, and
/// `free` is not async-signal-safe (it could deadlock on an allocator lock a
/// vanished parent thread held across `fork()`), so the child handler must not
/// drop it. `mem::forget` disarms the window (the slot goes `None`, so the
/// child's own allocations never bump the inherited loan slot) while leaking
/// the ledger buffer — bounded (one per forking thread that held a window) and
/// moot once the child `exec`s; a fork-without-exec child pays that one-time
/// leak to stay async-signal-safe. NEVER use on the normal disarm path.
///
/// Uses `.take()` — NOT `*slot = None` — so the old `BorrowWindow` leaves the
/// `RefCell` borrow before it is forgotten (belt-and-braces: even the `forget`
/// path must not re-enter the borrow; `disarm_window` relies on the same
/// discipline where it DOES drop-and-free).
///
/// `try_borrow_mut`, NEVER `borrow_mut`: if `fork()` was called
/// from a signal handler that interrupted THIS thread inside a
/// `with_window`/`with_window_mut` closure, the child inherits the `RefCell`
/// in its BORROWED state, and a panicking borrow here would abort the child
/// from inside the atfork handler (the panic hook also allocates — the same
/// class as the panic-hook deadlock). On the `Err` arm the window is left
/// ARMED instead: the interrupted frame's references stay valid when the
/// signal handler returns into it in the child, and nothing panics. That arm
/// is reachable ONLY under a caller POSIX violation (`fork` is not
/// async-signal-safe — the same caller-constraint class as `atfork_prepare`'s
/// documented signal window), where the child must confine itself to
/// async-signal-safe calls until `exec` anyway — a left-armed window is the
/// safe degrade; a child abort is not. The leftover window is additionally
/// POISONED ([`window_poisoned`]) so it is INERT: no
/// allocation ever bumps it, no classification ever consults it.
fn clear_window_forget_ledger() {
    let _ = WINDOW.try_with(|w| match w.try_borrow_mut() {
        Ok(mut slot) => {
            if let Some(win) = slot.take() {
                core::mem::forget(win);
            }
        }
        Err(_) => {
            // Inherited MID-BORROW (a signal-handler fork interrupted this
            // thread inside a window closure): the cell cannot be disarmed
            // without invalidating the interrupted frame's references — so
            // POISON this thread's window machinery instead.
            // Every window entry point bypasses to the real
            // allocator while the latch is set, so the child can never bump
            // the parent-owned MAP_SHARED slot through the leftover window.
            // A plain const-init TLS `Cell` store: no allocation, no locks —
            // this module's TLS block is already live on this thread (we are
            // inside `WINDOW.try_with`).
            let _ = WINDOW_POISONED.try_with(|p| p.set(true));
        }
    });
}

// ── Global segment registry + quarantine (spin-locked) + release callback ────

struct Registry(core::cell::UnsafeCell<SegmentRegistry>);
// SAFETY: every access holds the matching lock.
unsafe impl Sync for Registry {}

static REGISTRY: Registry = Registry(core::cell::UnsafeCell::new(SegmentRegistry::new()));
static REGISTRY_LOCK: AtomicBool = AtomicBool::new(false);

/// Disarmed slot extents awaiting `retire_slot` (see [`crate::classify`]).
static QUARANTINE: Registry = Registry(core::cell::UnsafeCell::new(SegmentRegistry::new()));
static QUARANTINE_LOCK: AtomicBool = AtomicBool::new(false);

// KNOWN LIMITATIONS of these bare-CAS spin locks (documented, not yet fixed —
// a follow-on if real-time guarantees are wanted):
//   * A full `memcpy` runs under the lock on the segment/quarantine realloc
//     path, so a low-priority thread preempted mid-copy blocks a high-priority
//     `realloc` — UNBOUNDED PRIORITY INVERSION under `SCHED_FIFO`. A
//     priority-inheriting mutex or a lock-free structure would fix it.
//   * `atfork_prepare` SETS the re-entrancy guard before acquiring these locks,
//     so a foreign prepare handler running after ours
//     (POSIX runs prepare handlers in REVERSE registration order, and our
//     `.init_array` ctor registers late) — or an allocating signal handler
//     firing between `prepare` and `fork` — takes the reentrant real-allocator
//     arm instead of classifying into a spin lock this very thread holds.
//     Residual: a fork during THREAD TEARDOWN (guard TLS destroyed) cannot set
//     the flag and keeps the old wedgeable shape.
//   * A PANIC while a lock is held is survivable only as an ABORT, never a
//     wedge — and only because three layers line up on every SHIPPED path:
//     (1) every shipped entry into these locks (the interposers via
//     `with_guard`, the ABI exports via `with_control_guard`) holds the
//     re-entry flag, so the panic HOOK's own allocator traffic (capture
//     buffers, temp frees — the hook runs BEFORE unwinding) takes the
//     reentrant real-heap fallback instead of re-entering classification;
//     (2) the RAII `LockGuard` releases the locks as the unwind passes; and
//     (3) the panic aborts at the `extern "C"` boundary. A DIRECT caller
//     without the flag (a unit test, a future non-FFI path) gets no layer 1:
//     its panic hook re-enters `with_both` and spins forever on the same
//     thread BEFORE any landing pad runs — box-observed as a full-suite
//     hang. So under-lock code must stay panic-free and allocation-free
//     (the allocating quarantine fold was removed for exactly this reason),
//     and defects must surface as red asserts, never as panics under a lock.
//     Owner-tracking abort-on-reentry was considered and skipped: it adds a
//     thread-identity read per acquisition (TLS — hazardous in an interposer
//     during teardown, the TLS-teardown class) to defend a state unreachable by
//     construction on shipped paths.
fn lock(l: &AtomicBool) {
    while l
        .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        core::hint::spin_loop();
    }
}

/// RAII: releases a spin lock as an unwind passes. This is layer 2 of the
/// three-layer panic story in the KNOWN LIMITATIONS block above — it is NOT
/// sufficient on its own: a panic's hook runs BEFORE the unwind reaches this
/// drop, and only the re-entry flag (layer 1) keeps that hook's allocator
/// traffic from spinning on the still-held lock.
struct LockGuard(&'static AtomicBool);
impl Drop for LockGuard {
    fn drop(&mut self) {
        self.0.store(false, Ordering::Release);
    }
}

/// Run `f` with the global segment registry locked.
pub fn with_registry<R>(f: impl FnOnce(&mut SegmentRegistry) -> R) -> R {
    lock(&REGISTRY_LOCK);
    let _g = LockGuard(&REGISTRY_LOCK);
    // SAFETY: the lock is held for the whole borrow (until `_g` drops).
    f(unsafe { &mut *REGISTRY.0.get() })
}

/// Run `f` with the global quarantine locked.
pub fn with_quarantine<R>(f: impl FnOnce(&mut SegmentRegistry) -> R) -> R {
    lock(&QUARANTINE_LOCK);
    let _g = LockGuard(&QUARANTINE_LOCK);
    // SAFETY: the lock is held for the whole borrow.
    f(unsafe { &mut *QUARANTINE.0.get() })
}

/// Run `f` with BOTH global locks held, always registry-before-quarantine so the
/// order is consistent with [`atfork_lock`] (no lock-order deadlock).
fn with_both<R>(f: impl FnOnce(&mut SegmentRegistry, &mut SegmentRegistry) -> R) -> R {
    lock(&REGISTRY_LOCK);
    let _r = LockGuard(&REGISTRY_LOCK);
    lock(&QUARANTINE_LOCK);
    let _q = LockGuard(&QUARANTINE_LOCK);
    // SAFETY: both locks held for the whole borrow.
    f(unsafe { &mut *REGISTRY.0.get() }, unsafe {
        &mut *QUARANTINE.0.get()
    })
}

/// Register `[start, start + len)` with `cookie`, maintaining the CHILD-FOLD
/// INVARIANT: at EVERY instant a fork could occur,
/// `quarantine.spare_capacity() >= registry.len()`, so the `atfork` child's
/// registry→quarantine fold never ALLOCATES. malloc is not async-signal-safe:
/// a child fold that reallocated its `Vec` could deadlock on an allocator lock
/// a vanished parent thread held across `fork()` (reproducible
/// with an allocator-lock shim; the reserve below keeps the
/// invariant CONTINUOUS).
///
/// The reserve runs BEFORE the registration that raises the requirement,
/// inside ONE `with_both` scope: `atfork`'s prepare handler must acquire both
/// locks, so no fork can snapshot an under-reserved intermediate state (a
/// reserve issued AFTER the mutation leaves exactly that window). `+ 1` covers
/// the registration about to land; a failed registration leaves a harmless
/// over-reserve. The sibling maintaining site is [`arm_window`], which
/// reserves before its quarantine insert consumes a slot. The fold itself
/// pushes at most `registry.len()` NEW bases (every armed slot is already
/// quarantined from [`arm_window`]) — pinned by the `debug_assert` in
/// [`atfork_child_reset`] and the pure `SegmentRegistry` reservation tests.
///
/// SELF-GUARDING: the body holds the control
/// re-entrancy guard itself, so the `reserve`'s own malloc/free take
/// `with_guard`'s reentrant arm (the real allocator) and cannot re-enter these
/// spin locks even though `with_both` holds them — safe for direct callers
/// (unit tests in the self-interposed binary included); the `guarded()` export
/// nests idempotently. Reserving at register/arm time (normal execution)
/// rather than in the `atfork` PREPARE handler is deliberate: a reserve inside
/// prepare would call the real allocator while another prepare handler might
/// hold ITS lock, moving the very deadlock into the parent.
pub fn register_segment_reserving(
    start: usize,
    len: usize,
    cookie: usize,
) -> Result<(), crate::registry::RegisterError> {
    with_control_guard(|| {
        with_both(|reg, quar| {
            quar.reserve(reg.len() + 1);
            reg.register(start, len, cookie)?;
            Ok(())
        })
    })
}

// ── classification for free / realloc / usable_size (routes through the pure
//    `classify_ptr` so the ordering is the single source of truth) ────────────

/// Where a `free` pointer must go, resolved atomically (a Segment is removed
/// from the registry under the same lock so the release callback fires at most
/// once, and a later real-heap allocation recycling that address frees to glibc
/// correctly). A double-free of the SAME released SHM pointer is NOT re-caught —
/// it reaches glibc as caller UB; catching it would need an allocating
/// quarantine fold under the lock, which is the teardown deadlock (see
/// `classify_free_locked` / `realloc_move_registered`).
pub enum FreeClass {
    /// A live armed-window slot on THIS thread: do nothing (bump arena).
    WindowNoop,
    /// A LIVE disarmed/other-thread quarantine extent (not yet retired): do
    /// nothing (never glibc, never callback) — counted, so a consumer that
    /// forgets `retire_slot` is observable.
    QuarantineNoop,
    /// A TOMBSTONED (retired-but-remembered) slot extent: do nothing
    /// (tombstone-on-retire) — a late free of an in-slot pointer that outlived
    /// the sample; counted separately from the live no-op.
    TombstoneNoop,
    /// Ordinary private heap: the real allocator.
    RealHeap,
    /// A registered SHM range — removed from the registry, its extent parked in
    /// the quarantine as a refcounted HAND-OFF: fire the release callback with
    /// this cookie OUTSIDE the guard, then close the hand-off
    /// ([`end_release_handoff`]) iff `handoff` is `Some` (i.e. the callback
    /// actually ran — a no-callback leak keeps its ref and its coverage). Every
    /// real release owns exactly one ref; overlapping same-base releases each
    /// own one, and only the LAST close removes the shared entry (the ABA
    /// fix).
    Release {
        cookie: usize,
        handoff: Option<usize>,
    },
}

/// Classify `addr` for `free`, removing a matched segment atomically. `addr`
/// must already be known non-bootstrap (the interposer checks that first).
pub fn classify_and_remove_for_free(addr: usize) -> FreeClass {
    with_window(
        |w| classify_free_locked(addr, Some(w)),
        || classify_free_locked(addr, None),
    )
}

fn classify_free_locked(addr: usize, window: Option<&BorrowWindow>) -> FreeClass {
    with_both(|reg, quar| match classify_ptr(addr, window, reg, quar) {
        PtrClass::WindowSlot => FreeClass::WindowNoop,
        // A quarantine hit is a LIVE no-op or a TOMBSTONE hit — same
        // no-op action, different counter; the covering entry's flag decides.
        PtrClass::Quarantine => {
            if quar.classify(addr).map(|s| s.tombstoned()).unwrap_or(false) {
                FreeClass::TombstoneNoop
            } else {
                FreeClass::QuarantineNoop
            }
        }
        PtrClass::RealHeap => FreeClass::RealHeap,
        PtrClass::Segment { cookie } => {
            // The released range must stay COVERED
            // across the unregister→callback gap. The callback fires OUTSIDE
            // these locks, so a fork landing in that gap would otherwise
            // leave the CHILD with the range in NEITHER registry NOR
            // quarantine — its free of the MAP_SHARED pointer would go to glibc.
            // Insert the range into the quarantine FIRST (coverage before
            // removal), then remove the registration; the CALLER closes the
            // hand-off after the callback returns in the parent (a fork child
            // never runs the callback and simply keeps the coverage).
            //
            // ALLOCATION DISCIPLINE (a standing rule, upheld here): nothing under
            // `with_both` may allocate — on the TLS-teardown path a re-entrant
            // allocator call re-enters classification and spins on these
            // non-reentrant locks. The insert below is allocation-FREE by the
            // child-fold invariant (quarantine spare >= registry.len() >= 1
            // while this segment is registered — debug-asserted); the
            // unregister is an in-place `Vec::remove`. Side effect: a
            // concurrent double-free during the callback window now
            // classifies QuarantineNoop instead of reaching glibc.
            let mut handoff = None;
            if let Some(seg) = reg.classify(addr) {
                let (start, len) = (seg.start(), seg.len());
                debug_assert!(
                    quar.spare_capacity() >= 1,
                    "child-fold invariant broken: no reserved spare for the release hand-off"
                );
                // ABA-SAFE close: take a refcounted hand-off,
                // not a Created/Extended flag. If a SAME-BASE segment is
                // registered and released while this callback is still running,
                // both share this one base-keyed entry, and only the LAST close
                // removes it — so an overlapping later release is never uncovered
                // by an earlier release returning. `open_handoff` returns whether
                // a close is OWED (`false` iff it merely extended PRE-EXISTING
                // arm-window/tombstone coverage that must outlive this release).
                if quar.open_handoff(start, len) {
                    handoff = Some(start);
                }
                reg.unregister(start);
            }
            FreeClass::Release { cookie, handoff }
        }
    })
}

/// Close a release HAND-OFF in the PARENT now that the callback has returned:
/// decrement the entry's pending-release refcount and remove the transient
/// quarantine coverage ONLY when the last overlapping release lets go (the
/// ABA fix). If a SAME-BASE segment was registered and released
/// while this callback ran, its ref keeps the shared entry alive past this
/// close. (A fork child never runs the callback, never closes the hand-off, and
/// keeps the coverage — exactly the point.)
pub fn end_release_handoff(start: usize) {
    with_quarantine(|q| {
        let _ = q.close_handoff(start);
    });
}

/// If `addr` is in this thread's armed window, latch the reallocation escape and
/// return the pointer's ledger size (the exact bytes to move to the real heap);
/// otherwise `None`. Thread-local, so no cross-thread race.
pub fn window_realloc_escape(addr: usize) -> Option<usize> {
    with_window_mut(
        |w| {
            if w.owns(addr) {
                w.note_window_realloc();
                Some(w.ledger_size(addr).unwrap_or(0))
            } else {
                None
            }
        },
        || None,
    )
}

/// A scheduled release: the freed/moved address, the sample cookie captured
/// under the lock, and the hand-off to close after
/// the callback returns (`Some(start)` iff the transient quarantine entry was
/// CREATED for this release).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReleaseTicket {
    /// The address being released (handed to the callback).
    pub addr: usize,
    /// The sample cookie — the release IDENTITY (see `realloc_move_registered`).
    pub cookie: usize,
    /// `Some(start)` iff [`end_release_handoff`] must run after the callback.
    pub handoff: Option<usize>,
}

/// The result of a registered/quarantined realloc move, done UNDER the lock.
pub struct ReallocMove {
    /// The new (real-heap) pointer, or null if the allocation failed.
    pub dst: *mut c_void,
    /// The release to fire OUTSIDE the guard — present only for a segment whose
    /// move SUCCEEDED (a failed move leaves the old block valid AND registered;
    /// a quarantine move never releases).
    pub release: Option<ReleaseTicket>,
}

/// Handle a `realloc` of a REGISTERED or QUARANTINED pointer by running the
/// caller's `copy(avail)` move WHILE the range is pinned under the lock — so no
/// concurrent `free`/`realloc` of the same range on another thread can
/// unregister it and fire the release callback (freeing the shared memory) in
/// the middle of the copy. Returns `None` if `addr` is neither (the caller then
/// handles the window / real-heap cases). For a segment, the registration is
/// removed only after a SUCCESSFUL move.
///
/// `copy(avail)` must allocate the new buffer and copy at most `avail` bytes
/// from the source; it calls only the REAL allocator (never our interposer), so
/// running it under the spin lock cannot deadlock.
///
/// NOTE ON WHAT IS PROVEN: the copy-under-lock closes the hook-internal race by
/// CODE STRUCTURE (the copy runs inside the locked section, so no concurrent
/// `free`/`realloc` of the same range can unregister + release the sample
/// mid-copy). `copy` calls only the REAL allocator (glibc, never our hook) and
/// `reg.unregister` is an in-place `Vec::remove` — so NOTHING under this lock
/// allocates through our interposer, which is what keeps the TLS-teardown path
/// (`with_guard`'s Err arm) from re-entering here and deadlocking. The unit
/// tests pin the SEQUENTIAL semantics — bounded copy, release-on-success-only,
/// atomic removal — not the concurrency property, which rests on the lock scope.
///
/// COOKIE IS THE RELEASE IDENTITY: the scheduled release is
/// `(addr, cookie)` where `cookie` is captured HERE, under the lock, as the OLD
/// sample's handle — and [`release_callback`] releases by COOKIE, not by
/// address. So even if the consumer registers a NEW segment at the SAME `addr`
/// after this returns but before the release fires (the callback runs OUTSIDE
/// the lock — firing it under the lock would re-introduce
/// the TLS-teardown deadlock), the OLD cookie still releases the OLD sample, and
/// the NEW registration stays intact so a later free of the replacement routes
/// to the NEW cookie. Suppressing the old release when a replacement appears
/// (as an external reproduction proposed) would LEAK the old sample — the
/// opposite of correct. Re-using a slot address before its prior sample's
/// release completes is a consumer LIFECYCLE violation regardless; the crate
/// stays correct through it. Pinned by
/// `a_same_address_reregistration_does_not_corrupt_the_old_release_or_new_routing`.
pub fn realloc_move_registered(
    addr: usize,
    copy: impl FnOnce(usize) -> *mut c_void,
) -> Option<ReallocMove> {
    with_both(|reg, quar| {
        if let Some(seg) = reg.classify(addr) {
            // A registered segment is an EXACT per-field range (the consumer
            // contract), so `seg.end() - addr` is the true allocation size — the
            // copy discloses nothing beyond it.
            let (start, len, cookie, avail) =
                (seg.start(), seg.len(), seg.cookie(), seg.end() - addr);
            let dst = copy(avail); // copied while the segment is pinned under the lock
            if dst.is_null() {
                // Failed: leave the old block valid AND still registered.
                return Some(ReallocMove { dst, release: None });
            }
            // The release hand-off, exactly as in `classify_free_locked`:
            // coverage before removal, allocation-free by the child-fold
            // invariant (quarantine spare >= registry.len() >= 1 here). ABA-safe
            // refcounted close — `handoff` is always `Some`.
            debug_assert!(
                quar.spare_capacity() >= 1,
                "child-fold invariant broken: no reserved spare for the release hand-off"
            );
            let handoff = if quar.open_handoff(start, len) {
                Some(start)
            } else {
                None
            };
            reg.unregister(start);
            return Some(ReallocMove {
                dst,
                release: Some(ReleaseTicket {
                    addr,
                    cookie,
                    handoff,
                }),
            });
        }
        if quar.classify(addr).is_some() {
            // A quarantined slot pointer has NO per-allocation extent — the whole
            // slot `[base, tail_limit)` was quarantined at arm time and the
            // ledger was dropped at disarm, so `tail_limit - addr` would copy
            // LATER allocations / unrelated slot bytes into the caller's buffer
            // (memory disclosure). Fail the realloc SAFELY instead: null +
            // ENOMEM, no copy. The C contract leaves the old block valid, and
            // this matches the crate's fail-safe-over-corruption direction.
            // (Reallocating published slot-backed storage is a misuse; the normal
            // take-side path reallocs a registered EXACT segment, handled above.)
            // SAFETY: __errno_location returns a valid per-thread int pointer.
            unsafe { *libc::__errno_location() = libc::ENOMEM };
            return Some(ReallocMove {
                dst: core::ptr::null_mut(),
                release: None,
            });
        }
        None
    })
}

/// The EXACT usable size of a hook-managed pointer, or [`UsableOutcome::RealHeap`]
/// for a private-heap pointer. Never larger than the real allocation.
pub enum UsableOutcome {
    /// A hook-managed pointer of exactly `n` usable bytes.
    Exact(usize),
    /// A private-heap pointer: ask the real allocator.
    RealHeap,
}

/// Classify `addr` for `malloc_usable_size`. `addr` must already be known
/// non-null and non-bootstrap (the interposer reports 0 for a bootstrap
/// pointer).
pub fn classify_usable(addr: usize) -> UsableOutcome {
    if let Some(sz) = with_window(
        |w| {
            if w.owns(addr) {
                Some(w.ledger_size(addr).unwrap_or(0))
            } else {
                None
            }
        },
        || None,
    ) {
        return UsableOutcome::Exact(sz);
    }
    with_both(|reg, quar| {
        // A registered segment is a whole SHM sample (or a per-field sub-range);
        // the allocation AT `addr` has an unknown exact size, and `seg.end() -
        // addr` is the bytes to the end of the REGISTERED RANGE, not the
        // allocation — reporting that would let a glibc-idiomatic caller grow
        // into the slack and write past the forged capacity into sibling bytes
        // of the same sample. Report 0 (conservative — forces a realloc → escape
        // → copy, which is sound). The window path stays exact (the ledger has
        // the true size).
        if reg.classify(addr).is_some() || quar.classify(addr).is_some() {
            return UsableOutcome::Exact(0);
        }
        UsableOutcome::RealHeap
    })
}

/// Retire a quarantined slot extent by its start address (the consumer calls
/// this once the backing sample is reclaimed) — TOMBSTONE-on-retire: the
/// entry is marked dead-but-remembered, NOT deleted, so a late `free` of an
/// incidental in-slot pointer (which can outlive the sample) stays a counted
/// no-op forever instead of reaching glibc. A pool that recycles the slot
/// address REVIVES the tombstone on the next `arm_window` (bounded by the
/// pool's distinct addresses). Returns whether an entry started at `start`.
pub fn retire_quarantine(start: usize) -> bool {
    with_quarantine(|q| q.tombstone(start))
}

/// Acquire both global locks (the `atfork` prepare handler) so no mutator holds
/// one across the fork — with the interval gate ARMED FIRST (the
/// ordering below) and the re-entrancy guard set BEFORE the
/// locks. pthread_atfork PREPARE handlers run in
/// REVERSE registration order, and our `.init_array` constructor typically
/// registers late, so handlers registered EARLIER run AFTER ours — while this
/// thread holds these spin locks. A foreign prepare handler that frees
/// re-enters our interposer; without the guard that free CLASSIFIES →
/// `with_both` → spins forever on the locks this very thread holds, wedging
/// every fork. With the guard held, such calls take the reentrant arm — which
/// CONSULTS the interval gate (`real_free_or_noop`'s
/// `interval_managed_action`, and the reentrant heads of realloc/usable-size)
/// — and never touch the locks.
///
/// # INVARIANT (the signal-window class)
///
/// At EVERY instant where a `free` could reach glibc, one of two things is
/// true: FULL classification runs (guard not held → the interposers' main
/// arm → `classify_and_remove_for_free`), or the interval gate is VISIBLE
/// AND CONSULTED (guard held → the reentrant arm → `interval_managed_action`;
/// the realloc/usable-size reentrant heads consult it the same way). The
/// gate's armed span therefore strictly CONTAINS the guard's atfork span AND
/// the locks' span: armed BEFORE the guard is set here, cleared AFTER the
/// guard is cleared on BOTH exits (`atfork_unlock`,
/// `atfork_child_release_guard`) — so no signal can observe "guard held,
/// gate unarmed" anywhere inside the atfork path. The corollary the CHILD
/// adds: a phase that READS the tables must never coexist with a
/// table MUTATION — `atfork_child_reset` mutates the tables with the locks
/// force-open, so it clears the locks-held flag FIRST and runs its whole
/// body under the never-reads phase (see its child phase timeline). The
/// residual OUTSIDE atfork — a signal handler freeing a managed pointer
/// while interrupted inside an ORDINARY interposer frame (guard held, no
/// fork in flight) — is the pre-existing reentrancy class the gate
/// deliberately does not cover: arming it for every interposer frame would
/// leak every reentrant real-heap free.
///
/// `ATFORK_GUARD_HELD` remembers whether WE set the flag: a fork issued from
/// INSIDE an interposer frame arrives with it already set, and the parent
/// handler must not clear a guard it does not own. A destroyed guard TLS
/// (fork during thread teardown) degrades to not-acquired — that corner keeps
/// the old wedgeable shape, documented in KNOWN LIMITATIONS.
pub fn atfork_lock() {
    // PUBLISH the interval gate FIRST — before the guard, before the locks:
    // the guard is what routes a free to the
    // REENTRANT arm, and that arm's only defense is the gate — so a signal
    // interrupting after guard-set but before a gate-arm would take the
    // reentrant arm, read "no interval", and hand a managed SHM pointer to
    // glibc. Widening is the SAFE direction both times: in the gate-armed
    // pre-guard window a free still takes FULL classification (guard not
    // held), with only the real-heap tails degrading to leak-and-count
    // (kind 5); in the gate-armed pre-lock window the reentrant arm
    // classifies conservatively (kind 5), never reading the tables. A
    // destroyed TLS degrades to not-armed — that teardown-fork corner keeps
    // the glibc fall-through, same caveat as the guard.
    let _ = ATFORK_INTERVAL_ARMED.try_with(|h| h.set(true));
    #[cfg(test)]
    atfork_boundary_probe_for_test();
    let acquired = GUARD.try_with(|g| g.try_set()).unwrap_or(false);
    let _ = ATFORK_GUARD_HELD.try_with(|h| h.set(acquired));
    #[cfg(test)]
    atfork_boundary_probe_for_test();
    lock(&REGISTRY_LOCK);
    lock(&QUARANTINE_LOCK);
    // Only now are the tables readable (SHM_FREE_BYPASS fix): with both locks
    // actually held, the reentrant arm classifies against them lock-free. The
    // signal window between the `lock` calls and this store degrades to the
    // conservative leak arm (gate armed, tables not yet readable) — safe.
    let _ = ATFORK_LOCKS_HELD.try_with(|h| h.set(true));
}
/// Release both global locks (the `atfork` parent handler), then the
/// re-entrancy guard (only if OUR prepare handler set it), then — LAST — the
/// interval gate: the exact reverse of [`atfork_lock`], preserving its
/// INVARIANT (the gate outlives the guard on the way out, so a signal between
/// guard-clear steps still finds the reentrant arm's gate armed).
pub fn atfork_unlock() {
    // Table reads stop BEFORE releasing the locks — once another thread can
    // mutate the tables, the reentrant arm must not read them.
    let _ = ATFORK_LOCKS_HELD.try_with(|h| h.set(false));
    QUARANTINE_LOCK.store(false, Ordering::Release);
    REGISTRY_LOCK.store(false, Ordering::Release);
    let held = ATFORK_GUARD_HELD
        .try_with(|h| h.replace(false))
        .unwrap_or(false);
    if held {
        let _ = GUARD.try_with(|g| g.clear());
    }
    #[cfg(test)]
    atfork_boundary_probe_for_test();
    // The interval gate clears LAST — after the releases AND after the guard
    // clear (the mirror of the prepare-side ordering): a
    // signal landing between any of the steps above must still classify
    // conservatively — leaked-and-counted, never glibc.
    let _ = ATFORK_INTERVAL_ARMED.try_with(|h| h.set(false));
}

#[cfg(test)]
thread_local! {
    /// TEST SEAM: when armed with a probe address,
    /// the boundary hooks classify it exactly where a signal handler's free
    /// would land, recording `(verdict, guard_active)` at each boundary —
    /// prepare: after gate-arm before the guard, then after the guard before
    /// the first lock; parent: after the releases AND the guard clear, before
    /// the gate clears; child: after the guard clear, before the gate clears.
    /// The guard bit is what makes the gate-vs-guard ORDER observable: a
    /// probe-recorded instant with the gate armed and the guard NOT held
    /// exists only if the gate was published first.
    static ATFORK_BOUNDARY_PROBE: RefCell<BoundaryProbe> = const { RefCell::new(None) };
}

/// The armed probe: the address plus each boundary's
/// `(interval verdict, guard active)` observation.
#[cfg(test)]
type BoundaryProbe = Option<(usize, Vec<(Option<IntervalPtrAction>, bool)>)>;

/// Arm the boundary probe for `addr` (see [`ATFORK_BOUNDARY_PROBE`]). The
/// result vector is pre-reserved so the boundary pushes never allocate inside
/// the handlers.
#[cfg(test)]
pub fn arm_atfork_boundary_probe_for_test(addr: usize) {
    ATFORK_BOUNDARY_PROBE.with(|p| *p.borrow_mut() = Some((addr, Vec::with_capacity(4))));
}

/// Take the recorded boundary observations, disarming the probe.
#[cfg(test)]
pub fn take_atfork_boundary_probe_for_test() -> Vec<(Option<IntervalPtrAction>, bool)> {
    ATFORK_BOUNDARY_PROBE.with(|p| p.borrow_mut().take().map(|(_, v)| v).unwrap_or_default())
}

/// Force the interval gate armed on this thread — models the TLS state a fork
/// child inherits from the prepare handler, for the child-exit ordering pin.
#[cfg(test)]
pub fn arm_interval_gate_for_test() {
    let _ = ATFORK_INTERVAL_ARMED.try_with(|h| h.set(true));
}

/// Force the locks-held flag set on this thread — the other half of the
/// inherited-from-prepare TLS state, for the child-reset phase pin
/// (the reset must CLEAR it before its table mutations).
#[cfg(test)]
pub fn set_atfork_locks_held_for_test() {
    let _ = ATFORK_LOCKS_HELD.try_with(|h| h.set(true));
}

/// TEST SEAM: serializes tests that touch the
/// PROCESS-GLOBAL tables / spin locks against the one test that drives the
/// full [`atfork_child_reset`] — which force-opens the spin locks and clears
/// the registry, so it must be EXCLUSIVE (`.write()`) while every other
/// global-table test holds the READ side (staying parallel among themselves,
/// exactly as before). `interpose::tests`' CB_LOCK remains their
/// intra-module serializer, acquired AFTER this read guard (one consistent
/// order — no cycle).
#[cfg(test)]
pub(crate) static GLOBAL_TABLES_TEST_LOCK: std::sync::RwLock<()> = std::sync::RwLock::new(());

/// Called by the boundary hooks in test builds; a no-op unless armed.
#[cfg(test)]
fn atfork_boundary_probe_for_test() {
    ATFORK_BOUNDARY_PROBE.with(|p| {
        if let Some((addr, results)) = p.borrow_mut().as_mut() {
            results.push((interval_managed_action(*addr), guard_active_for_test()));
        }
    });
}

// The interval decision vocabulary + pure decision table live in the
// `#[cfg]`-free [`crate::classify`] module so they are
// oracle-testable on every platform; re-exported here because the interposers
// consume them through `state::`.
pub use crate::classify::IntervalPtrAction;
use crate::classify::{interval_ptr_action, AtforkIntervalPhase, IntervalTableVerdict};

/// Classify `addr` for the REENTRANT arm during the prepare-lock interval, or
/// `None` when this thread is NOT in (or around) the interval (the common
/// case — the caller then applies the ordinary reentrant action). The
/// SHM_FREE_BYPASS hazard: with the re-entrancy guard set before the locks, a
/// foreign prepare handler's free of a hook-MANAGED pointer would take the
/// reentrant arm straight to glibc — an SHM address on the real heap's free
/// list. The decision itself is the pure
/// [`crate::classify::interval_ptr_action`]; this wiring reads the two
/// thread-local phase flags (the widened gate `ATFORK_INTERVAL_ARMED`
/// versus the tables-readable `ATFORK_LOCKS_HELD`), the window TLS (via
/// `try_borrow` — a stuck borrow from a signal-handler fork mid-closure
/// degrades to unclassified rather than panicking), and hands the pure
/// decision a table lookup it consults ONLY under `ArmedLocksHeld`. In the
/// widened boundary (gate armed, locks not provably held — the signal spans
/// beside the lock operations) the answer is the conservative
/// [`IntervalPtrAction::LeakUnclassified`]: leaked-and-counted, never glibc,
/// and never a table read (another thread may be mutating the tables under
/// the locks). Allocation-free and lock-free throughout.
pub fn atfork_interval_classify(addr: usize) -> Option<IntervalPtrAction> {
    let armed = ATFORK_INTERVAL_ARMED.try_with(|h| h.get()).unwrap_or(false);
    if !armed {
        return None;
    }
    let phase = if ATFORK_LOCKS_HELD.try_with(|h| h.get()).unwrap_or(false) {
        AtforkIntervalPhase::ArmedLocksHeld
    } else {
        AtforkIntervalPhase::ArmedLocksNotHeld
    };
    // Window first (mirrors `classify_ptr`'s order): this thread's TLS — safe
    // to read in EVERY armed phase.
    let window_hit = WINDOW
        .try_with(|w| {
            w.try_borrow()
                .ok()
                .map(|slot| slot.as_ref().map(|win| win.owns(addr)).unwrap_or(false))
                .unwrap_or(false)
        })
        .unwrap_or(false)
        && !window_poisoned();
    interval_ptr_action(phase, window_hit, || {
        // SAFETY: this closure runs only under `ArmedLocksHeld`, and
        // ATFORK_LOCKS_HELD is set only between acquire and release on THIS
        // thread, so both locks are held here — no other thread can hold a
        // `&mut`, and this thread cannot (the guard keeps it on the reentrant
        // arm).
        let (reg, quar) = unsafe { (&*REGISTRY.0.get(), &*QUARANTINE.0.get()) };
        if reg.classify(addr).is_some() {
            IntervalTableVerdict::Registered
        } else if let Some(seg) = quar.classify(addr) {
            if seg.tombstoned() {
                IntervalTableVerdict::QuarantineTombstoned
            } else {
                IntervalTableVerdict::QuarantineLive
            }
        } else {
            IntervalTableVerdict::NotFound
        }
    })
}

/// The reentrant free arm's interval gate, split PURE from its counting action:
/// returns `Some(action)` ONLY for a hook-MANAGED pointer
/// during the prepare interval — `None` for unmanaged OR not-in-interval, so
/// the caller falls through to the real allocator. NO side effect: the counting
/// lives in the caller's gated block, so a caller that IGNORES this verdict
/// also skips the count — which is what makes the count a sound
/// oracle for "the SHM address was kept off glibc".
pub fn interval_managed_action(addr: usize) -> Option<IntervalPtrAction> {
    match atfork_interval_classify(addr) {
        Some(IntervalPtrAction::Unmanaged) | None => None,
        managed => managed,
    }
}

/// The child half of the guard hand-off — called at the end of
/// [`atfork_child_reset`], and clearing UNCONDITIONALLY:
/// besides the guard our own prepare set, a fork issued from a signal
/// handler that interrupted an interposer frame inherits the guard SET, and if
/// the child `siglongjmp`s out of the handler the interrupted frame never
/// resumes and never clears it — every later allocator call in the child would
/// take the reentrant arm, with ZERO classification, sending quarantined SHM
/// pointers to glibc `free`. Clearing restores classification for the child's
/// life. If the interrupted frame DOES resume, its own eventual `clear()` is a
/// harmless double-clear (a `Cell` store); the cost is re-entrancy protection
/// for the remainder of that one resumed frame — transient, and reachable only
/// under the same caller POSIX violation (fork in a signal handler) as the
/// window poison.
pub(crate) fn atfork_child_release_guard() {
    // Both interval flags too: the child force-opened the locks at the TOP of
    // `atfork_child_reset`, so this runs release-both → guard-clear →
    // gate-clear — the same exit ordering as `atfork_unlock`
    // ([`atfork_lock`]'s INVARIANT: the gate outlives the guard on every
    // exit), and it cannot re-open the acquisition gap (the child ends with
    // the gate CLEARED, after the force-open). The tables-readable flag first
    // — an idempotent BACKSTOP: the reset already cleared it as
    // its step 1, BEFORE its table mutations (see the child phase timeline
    // there); kept here because this fn is also the direct exit seam the
    // unit pins drive — then the guard, then the widened gate LAST. A signal
    // landing between any of these still finds the reentrant arm's gate
    // armed and classifies conservatively (leaked, never glibc; the
    // single-threaded child makes this span benign anyway).
    let _ = ATFORK_LOCKS_HELD.try_with(|h| h.set(false));
    let _ = ATFORK_GUARD_HELD.try_with(|h| h.set(false));
    let _ = GUARD.try_with(|g| g.clear());
    #[cfg(test)]
    atfork_boundary_probe_for_test();
    let _ = ATFORK_INTERVAL_ARMED.try_with(|h| h.set(false));
}
/// The `atfork` child handler. The child is a fresh single-threaded process that
/// inherited the parent's MAP_SHARED slots and C++ objects (forged vectors still
/// point at the SAME SHM addresses). Clearing the registry to empty would make
/// those inherited frees fall through to glibc — so instead every registered
/// range is FOLDED INTO THE QUARANTINE (free → no-op, never the callback:
/// releasing here would double-release the parent's sample), the callback is
/// cleared, and `RESOLVING` is reset (a fork mid-resolution must be able to
/// resolve in the child).
///
/// # Async-signal-safety
/// This handler runs in the child of a possibly-multithreaded `fork()`, so it
/// may call ONLY async-signal-safe operations — no `malloc`/`free`, which could
/// deadlock on an allocator lock a vanished parent thread held across the fork.
/// Three handler-body hazards are neutralised:
///   * the fold's `Vec::push` — the parent keeps `quarantine.spare_capacity()
///     >= registry.len()` CONTINUOUSLY (reserved BEFORE the mutation at both
///     maintaining sites, [`register_segment_reserving`] and [`arm_window`]),
///     so the pushes stay within capacity and never reallocate (the
///     `debug_assert` pins it);
///   * the window disarm's ledger `free` — [`clear_window_forget_ledger`] takes
///     the window WITHOUT dropping (a bounded leak, not a `free`);
///   * the window disarm's `RefCell` borrow — `try_borrow_mut` tolerates a
///     cell inherited MID-BORROW (a signal-handler `fork()` that interrupted
///     this thread inside a window closure): the degrade is a
///     left-armed-but-POISONED window (`window_poisoned` makes every window
///     entry point bypass it, so the child can never bump parent-owned slot
///     memory through it; an operation already IN FLIGHT
///     at the fork is invalidated at the exposure boundary instead, by
///     `window_alloc`'s post-closure re-check), never a
///     child-aborting `BorrowMutError` panic.
///
/// Everything else here is a plain atomic store. `reg.clear()` and
/// `quar.insert_or_extend` (within capacity) touch no allocator.
pub fn atfork_child_reset() {
    // CHILD PHASE TIMELINE (the child mirror of the
    // prepare/parent timelines on `atfork_lock`/`atfork_unlock`). The TLS
    // inherited from prepare through `fork()` is: gate ARMED, LOCKS_HELD
    // true, guard held.
    //
    //   1. LOCKS_HELD := false — FIRST, before ANY mutation below. This very
    //      handler MUTATES the guarded tables (the registry→quarantine fold
    //      + `reg.clear()`) with the locks force-open, so a signal-side
    //      reentrant free classifying under `ArmedLocksHeld` would
    //      lock-free-read Vecs THIS handler is mid-mutating — the corruption
    //      class all over again. Under `ArmedLocksNotHeld` the reentrant
    //      classification NEVER reads the tables (LeakUnclassified —
    //      leak-and-count, bounded, counted): table mutations only ever
    //      happen under a phase that never reads them.
    //   2. Force both locks open (any holder was another parent thread
    //      absent in the child) + clear the callback and resolver latches.
    //   3. The window clear + the fold — every mutation runs under the
    //      never-reads phase from step 1.
    //   4. The exit (`atfork_child_release_guard`): guard clear, then the
    //      gate LAST — the established exit ordering, unchanged.
    let _ = ATFORK_LOCKS_HELD.try_with(|h| h.set(false));
    REGISTRY_LOCK.store(false, Ordering::Release);
    QUARANTINE_LOCK.store(false, Ordering::Release);
    RESOLVING.store(false, Ordering::Release);
    RELEASE_CB.store(0, Ordering::Release);
    #[cfg(test)]
    atfork_boundary_probe_for_test();
    // Disarm the inherited window WITHOUT freeing its ledger (free is not
    // async-signal-safe here) — so the child's own allocations never bump the
    // inherited loan slot, and the fold below cannot bump it either.
    clear_window_forget_ledger();
    // SAFETY: single-threaded in the child immediately after fork; the locks are
    // open and no other thread can contend.
    unsafe {
        let reg = &mut *REGISTRY.0.get();
        let quar = &mut *QUARANTINE.0.get();
        // Fold every inherited registered range into the quarantine (base-keyed,
        // never dropped). Other threads' ARMED-window slots are already in the
        // quarantine from `arm_window`, so the child inherits their coverage too
        // — a fork-without-exec child freeing an inherited in-slot pointer hits
        // the quarantine, not glibc. The parent reserved capacity for exactly
        // this many pushes, so it does not ALLOCATE (async-signal-safe).
        let cap_before = quar.capacity();
        for seg in reg.segments() {
            quar.insert_or_extend(seg.start(), seg.len());
        }
        debug_assert_eq!(
            quar.capacity(),
            cap_before,
            "atfork child fold reallocated — the parent's capacity reserve was insufficient"
        );
        // Mid-mutation probe instant: the fold has grown the quarantine and
        // the registry is not yet cleared — the phase here MUST be the
        // never-reads one (pinned by the child-reset phase test).
        #[cfg(test)]
        atfork_boundary_probe_for_test();
        reg.clear();
    }
    // Last: release the re-entrancy guard (see `atfork_child_release_guard` for
    // why this is UNCONDITIONAL), then the gate. Everything above classifies
    // conservatively under the step-1 phase; last keeps the handler body
    // symmetric with `atfork_unlock`.
    atfork_child_release_guard();
}

static RELEASE_CB: AtomicUsize = AtomicUsize::new(0);

/// Set (or clear with `None`) the release callback.
pub fn set_release_callback(cb: ReleaseCallback) {
    let v = match cb {
        Some(f) => f as usize,
        None => 0,
    };
    RELEASE_CB.store(v, Ordering::Release);
}

/// The registered release callback, if any.
pub fn release_callback() -> ReleaseCallback {
    let p = RELEASE_CB.load(Ordering::Acquire);
    if p == 0 {
        None
    } else {
        // SAFETY: a non-zero slot holds a ReleaseCallback fn pointer.
        Some(unsafe { core::mem::transmute::<usize, unsafe extern "C" fn(*mut c_void, usize)>(p) })
    }
}

// ── Principle #3 observability: counters + first-of-regime breadcrumbs ───────
//
// The counters survive filtering (Principle #3). But the ONLY reader is an
// rmw borrow consumer that is not built yet, and a stock ROS 2 process installs
// no `tracing` subscriber — so a failure would otherwise be invisible on the
// robot. Each failure path therefore also emits a ONE-LINE, allocation-free
// (`libc::write`) breadcrumb the FIRST time it fires (latched, flood-latch
// discipline — never a flood), gated on `CERULION_HEAPHOOK_DEBUG`. The
// won-malloc DEGRADE is louder still — see `set_status`.

static RELEASE_WITHOUT_CALLBACK: AtomicUsize = AtomicUsize::new(0);
static BOOTSTRAP_EXHAUSTED: AtomicUsize = AtomicUsize::new(0);
static PRE_RESOLUTION_LEAK: AtomicUsize = AtomicUsize::new(0);
static QUARANTINE_NOOP_FREES: AtomicUsize = AtomicUsize::new(0);
static TOMBSTONE_HITS: AtomicUsize = AtomicUsize::new(0);
static ATFORK_INTERVAL_LEAKS: AtomicUsize = AtomicUsize::new(0);

/// `CERULION_HEAPHOOK_DEBUG` truthiness, cached at load (a `getenv` per failure
/// would be wasteful and the value never changes).
static DEBUG_ENABLED: AtomicBool = AtomicBool::new(false);
/// Set the cached debug flag (the load constructor reads the env once).
pub fn set_debug_enabled(on: bool) {
    DEBUG_ENABLED.store(on, Ordering::Release);
}
/// Whether the debug breadcrumbs are enabled.
pub fn debug_enabled() -> bool {
    DEBUG_ENABLED.load(Ordering::Acquire)
}

/// Write `msg` to stderr (fd 2) ONCE — the first time `latch` is armed — iff
/// debug breadcrumbs are enabled. Allocation-free and flood-free.
fn breadcrumb_once(latch: &AtomicBool, msg: &[u8]) {
    if !debug_enabled() || latch.swap(true, Ordering::AcqRel) {
        return;
    }
    // SAFETY: writing a fixed byte slice to stderr; no allocation.
    unsafe {
        libc::write(2, msg.as_ptr() as *const c_void, msg.len());
    }
}

static WARNED_RELEASE_NO_CB: AtomicBool = AtomicBool::new(false);
static WARNED_BOOTSTRAP: AtomicBool = AtomicBool::new(false);
static WARNED_PRE_RES: AtomicBool = AtomicBool::new(false);

/// A registered range was freed/realloced but no release callback was set — the
/// range is leaked rather than wrongly freed.
pub fn note_release_without_callback() {
    RELEASE_WITHOUT_CALLBACK.fetch_add(1, Ordering::Relaxed);
    breadcrumb_once(
        &WARNED_RELEASE_NO_CB,
        b"cerulion heap hook: SHM range freed with no release callback set (leaked)\n",
    );
}
/// The bootstrap arena was exhausted (returned NULL).
pub fn note_bootstrap_exhausted() {
    BOOTSTRAP_EXHAUSTED.fetch_add(1, Ordering::Relaxed);
    breadcrumb_once(
        &WARNED_BOOTSTRAP,
        b"cerulion heap hook: bootstrap arena exhausted (pre-resolution)\n",
    );
}
/// A real-heap pointer was freed before the real allocator resolved — leaked.
pub fn note_pre_resolution_leak() {
    PRE_RESOLUTION_LEAK.fetch_add(1, Ordering::Relaxed);
    breadcrumb_once(
        &WARNED_PRE_RES,
        b"cerulion heap hook: real-heap free before the allocator resolved (leaked)\n",
    );
}
/// A LIVE-quarantined (disarmed, not-yet-retired / other-thread slot) pointer
/// was freed — a no-op. Not a failure, but counted so a consumer that never
/// calls `retire_slot` (an unbounded LIVE quarantine) is observable.
pub fn note_quarantine_noop_free() {
    QUARANTINE_NOOP_FREES.fetch_add(1, Ordering::Relaxed);
}
/// A TOMBSTONED (retired-but-remembered) slot pointer was freed — a no-op
/// (tombstone-on-retire). The retired slot's coverage doing its job: a late free
/// of an incidental in-slot pointer that outlived the sample. Counted
/// separately from the live no-op so an operator can tell "coverage after
/// retire" from "consumer never retired"; sustained growth means a slot's tail
/// is still being freed long after retire (benign, but visible).
pub fn note_tombstone_hit() {
    TOMBSTONE_HITS.fetch_add(1, Ordering::Relaxed);
}
/// A REGISTERED range was freed by a foreign prepare handler DURING the
/// `atfork` prepare-lock interval (SHM_FREE_BYPASS fix): the sample is LEAKED
/// for the interval — registration kept, callback deliberately not fired (see
/// [`IntervalPtrAction::LeakRegistered`]) — and counted here (kind 5). Also
/// counts a free landing in the WIDENED gate boundary
/// ([`IntervalPtrAction::LeakUnclassified`]): the tables are unreadable there,
/// so the pointer cannot be proven unmanaged and is leaked the same way.
pub fn note_atfork_interval_leak() {
    ATFORK_INTERVAL_LEAKS.fetch_add(1, Ordering::Relaxed);
}

/// Read one of the diagnostic counters by index (the FFI getter): 0 =
/// releases with no callback (leaked ranges), 1 = bootstrap exhaustions,
/// 2 = pre-resolution real-heap-free leaks, 3 = LIVE quarantine no-op frees,
/// 4 = tombstone hits (frees of a RETIRED slot's in-slot pointers), 5 =
/// atfork-interval registered-range leaks (a foreign prepare handler freed a
/// registered SHM range during the prepare-lock interval — leaked, not
/// released).
pub fn counter(kind: u32) -> u64 {
    let v = match kind {
        0 => &RELEASE_WITHOUT_CALLBACK,
        1 => &BOOTSTRAP_EXHAUSTED,
        2 => &PRE_RESOLUTION_LEAK,
        3 => &QUARANTINE_NOOP_FREES,
        4 => &TOMBSTONE_HITS,
        5 => &ATFORK_INTERVAL_LEAKS,
        _ => return u64::MAX,
    };
    v.load(Ordering::Relaxed) as u64
}

// ── won-malloc status (set at load) ─────────────────────────────────────────

static STATUS: AtomicU32 = AtomicU32::new(0);

/// Record the load-time status bitset (see `abi::HEAPHOOK_STATUS_*`).
///
/// If the hook did NOT win `malloc` (another interposer — jemalloc/tcmalloc/ASan
/// — is ahead of it), the whole feature silently degrades to the copy path. That
/// degrade gets an ALWAYS-ON (not debug-gated) one-line stderr breadcrumb: the
/// launcher auto-injects this hook by default, so a robot whose process already
/// carries a foreign allocator would otherwise get zero output about why
/// zero-copy is off.
pub fn set_status(bits: u32) {
    STATUS.store(bits, Ordering::Release);
    if bits & crate::abi::HEAPHOOK_STATUS_WON_MALLOC == 0 {
        // SAFETY: allocation-free write of a fixed slice to stderr.
        let msg = b"cerulion heap hook: another allocator won malloc; zero-copy disabled\n";
        unsafe {
            libc::write(2, msg.as_ptr() as *const c_void, msg.len());
        }
    }
}

/// The current status bitset.
pub fn status() -> u32 {
    STATUS.load(Ordering::Acquire)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bootstrap_size_round_trips_the_exact_request() {
        // Each allocation carries an 8-byte size header, so a realloc copies the
        // real old size — never adjacent arena bytes (the P1(3) info leak).
        let a = bootstrap_alloc(64, 16, false) as usize;
        let b = bootstrap_alloc(100, 16, false) as usize;
        assert_ne!(a, 0);
        assert_ne!(b, 0);
        assert_ne!(a, b, "distinct allocations get distinct addresses");
        // SAFETY: a and b are live bootstrap allocations.
        unsafe {
            assert_eq!(bootstrap_size(a), 64);
            assert_eq!(bootstrap_size(b), 100);
        }
        assert!(bootstrap_contains(a) && bootstrap_contains(b));
        // 16-byte alignment of the user pointer holds despite the header.
        assert_eq!(a % 16, 0);
        assert_eq!(b % 16, 0);
    }

    // Fake, mutually-disjoint high addresses so the process-global registry
    // cannot collide across parallel tests (no real memory is touched — the
    // copy closure is a stand-in that never dereferences the pointers).
    #[test]
    fn realloc_move_registered_copies_under_lock_then_removes_and_releases() {
        // Read side of the global-tables guard (see its doc).
        let _t = GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        let base = 0x6000_0000_0000usize;
        // The production registration path — it maintains the child-fold
        // invariant (quarantine spare >= registry.len()) the release
        // hand-off insert below relies on for its no-allocation-under-lock
        // guarantee (a raw `register` would bypass the reserve).
        register_segment_reserving(base, 256, 0xABC).expect("register");
        let mv = realloc_move_registered(base + 8, |avail| {
            assert_eq!(avail, 248, "copy is bounded to the segment bytes remaining");
            0x1234 as *mut c_void
        })
        .expect("classified as a segment");
        assert_eq!(mv.dst as usize, 0x1234);
        assert_eq!(
            mv.release,
            Some(ReleaseTicket {
                addr: base + 8,
                cookie: 0xABC,
                handoff: Some(base),
            }),
            "release scheduled on success, with the release hand-off CREATED"
        );
        assert!(
            with_registry(|r| r.classify(base + 8)).is_none(),
            "a successful move removes the segment atomically"
        );
        // The released range stays quarantine-COVERED across the
        // unregister→callback gap (what a fork child inherits), then the
        // caller closes the hand-off after the callback returns.
        assert!(
            with_quarantine(|q| q.classify(base + 8)).is_some(),
            "the hand-off keeps the released range covered during the gap"
        );
        end_release_handoff(base);
        assert!(
            with_quarantine(|q| q.classify(base + 8)).is_none(),
            "closing the hand-off removes the transient coverage in the parent"
        );
    }

    // The atfork PREPARE handler must hold the
    // re-entrancy guard while it holds both spin locks — POSIX runs prepare
    // handlers in REVERSE registration order and our ctor registers late, so
    // FOREIGN handlers run AFTER ours, while the locks are held; one that
    // frees would classify → with_both → spin forever without the guard. The
    // behavioral half drives a real free through the interposer between
    // atfork_lock and atfork_unlock: with the guard it completes (reentrant →
    // real allocator). Drop the guard-set in atfork_lock → the first
    // assert goes red (and the free would spin); drop the parent's clear → the
    // last assert goes red.
    #[test]
    fn atfork_prepare_holds_the_guard_so_a_foreign_handlers_free_completes() {
        // Read side of the global-tables guard (see its doc).
        let _t = GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        atfork_lock();
        assert!(
            guard_active_for_test(),
            "prepare must set the re-entrancy guard before taking the locks"
        );
        // The foreign-handler shape: allocator traffic on this thread while
        // both locks are held. Reentrant arm → real allocator → completes.
        let p = unsafe { libc::malloc(64) };
        assert!(!p.is_null());
        unsafe { libc::free(p) };
        atfork_unlock();
        assert!(
            !guard_active_for_test(),
            "the parent handler releases the guard it set"
        );
    }

    // The signal-window class, all boundaries:
    // the interval gate must be OBSERVED armed at THREE probe instants —
    // (P1) in `atfork_lock` BEFORE the re-entrancy guard is set (the
    // guard gap: the guard routes a free to the reentrant arm, whose
    // only defense is the gate), (P2) after the guard but BEFORE the first
    // lock acquisition (the pre-lock gap), and (P3) in `atfork_unlock` AFTER the
    // releases AND the guard clear, before the gate clears. Each probe
    // records `(verdict, guard_active)`: the verdict must be the SAFE
    // managed arm (LeakUnclassified — leak-and-count, tables never read),
    // NEVER `None` (the glibc fall-through), and the guard BIT is what makes
    // the gate-vs-guard ORDER observable — P1's `false` exists only if the
    // gate was published before the guard, P3's `false` only if it outlives
    // the guard's clear. Reverting the prepare to guard-before-gate
    // makes P1 record `(_, true)` (statement swap) or `(None, true)` (gate
    // moved past the probe) → red either way; publish-after-the-locks reds
    // P2's verdict; clear-before-release (or before the guard clear) reds
    // P3.
    #[test]
    fn the_interval_gate_covers_the_lock_acquisition_and_release_boundaries() {
        // Read side of the global-tables guard (see its doc).
        let _t = GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        // An address no test registers: the probe verdicts then discriminate
        // purely on PHASE (boundary ⇒ conservative leak; inside ⇒ precise
        // Unmanaged ⇒ `None` from interval_managed_action; outside ⇒ `None`).
        const PROBE: usize = 0xF2F2_0000_0001;
        arm_atfork_boundary_probe_for_test(PROBE);
        atfork_lock();
        // Observed inside; ASSERTED after unlock — a red assert here would
        // panic while this thread holds both spin locks and wedge the suite.
        let inside = interval_managed_action(PROBE);
        atfork_unlock();
        assert_eq!(
            inside, None,
            "inside the interval proper the tables are readable and an \
             unmanaged address classifies precisely (the managed arms are the \
             interpose SHM_FREE_BYPASS test's)"
        );
        let leak = Some(IntervalPtrAction::LeakUnclassified);
        assert_eq!(
            take_atfork_boundary_probe_for_test(),
            vec![
                (leak, false), // P1: gate armed BEFORE the guard is set
                (leak, true),  // P2: gate armed, guard held, locks not yet
                (leak, false), // P3: gate still armed AFTER the guard cleared
            ],
            "every boundary must classify to the safe leak-and-count arm — a \
             `None` verdict is the glibc fall-through, and a wrong guard bit \
             means the gate does not contain the guard's span"
        );
        assert_eq!(
            interval_managed_action(PROBE),
            None,
            "after the parent handler returns the interval is fully disarmed"
        );
    }

    // The CHILD exit: `atfork_child_release_guard` must
    // clear the gate LAST — after the guard clears — mirroring
    // `atfork_unlock`. The child inherits the prepare-armed gate + a set
    // guard through `fork()`; its exit probe (after the guard clear, before
    // the gate clear) must still observe the safe arm with the guard already
    // released. Moving the gate clear above the guard clear records
    // `(None, true)` → red on both components.
    #[test]
    fn the_child_exit_keeps_the_gate_armed_until_after_the_guard_clears() {
        const PROBE: usize = 0xF2F2_0000_0002;
        // The TLS state a fork child inherits from our prepare handler.
        set_guard_for_test();
        arm_interval_gate_for_test();
        arm_atfork_boundary_probe_for_test(PROBE);
        atfork_child_release_guard();
        assert!(
            !guard_active_for_test(),
            "precondition of the pin: the child released the guard"
        );
        assert_eq!(
            take_atfork_boundary_probe_for_test(),
            vec![(Some(IntervalPtrAction::LeakUnclassified), false)],
            "the child-exit boundary (guard cleared, gate not yet) must \
             classify to the safe leak-and-count arm"
        );
        assert_eq!(
            interval_managed_action(PROBE),
            None,
            "after the child exit the interval is fully disarmed"
        );
    }

    // The child fold phase: `atfork_child_reset` MUTATES
    // the global tables (the registry→quarantine fold + `reg.clear()`) on a
    // thread whose TLS inherits ArmedLocksHeld from prepare — with the locks
    // force-open. A signal-side reentrant free during those mutations must
    // NOT classify under ArmedLocksHeld (a lock-free read of Vecs this very
    // handler is mid-mutating — the corruption class again): the reset
    // clears ATFORK_LOCKS_HELD as its step 1, so its ENTIRE body runs under
    // the never-reads phase (LeakUnclassified — leak-and-count). Probes:
    // P-c1 after the force-open, before the window clear + fold; P-c2
    // INSIDE the mutation span (quarantine grown by the fold, registry not
    // yet cleared); P-c3 at the exit (guard cleared, gate still armed).
    // Deferring the LOCKS_HELD clear to the exit
    // classifies P-c1/P-c2 under ArmedLocksHeld — the unmanaged
    // probe address then READS the tables and answers Unmanaged, recorded
    // as `(None, true)` → red on both entries.
    #[test]
    fn the_child_reset_mutates_the_tables_under_the_never_reads_phase() {
        // EXCLUSIVE side of the global-tables guard: this drives the FULL child
        // reset, which force-opens the spin locks and clears the
        // process-global registry — every other global-table test holds the
        // READ side.
        let _x = GLOBAL_TABLES_TEST_LOCK
            .write()
            .unwrap_or_else(|e| e.into_inner());
        const PROBE: usize = 0xF2F2_0000_0003;
        const FOLD_BASE: usize = 0x6C00_0000_0000;
        // One registered range, so the fold genuinely mutates BOTH tables
        // between P-c1 and P-c2 (a fake base far from any heap address — a
        // leftover quarantine entry could otherwise collide with a later
        // malloc reuse; purged below anyway). Registered BEFORE the
        // inherited-TLS trio below, as in reality (registration is normal
        // pre-fork execution): with armed+locks-held fabricated first, the
        // registration's own reserve — whose reentrant free runs INSIDE
        // `with_both`'s `&mut` scope — would classify ArmedLocksHeld and
        // shared-read the very tables that scope mutably borrows.
        register_segment_reserving(FOLD_BASE, 256, 0xC4).expect("register");
        // The TLS state a fork child inherits from our prepare handler.
        set_guard_for_test();
        arm_interval_gate_for_test();
        set_atfork_locks_held_for_test();
        arm_atfork_boundary_probe_for_test(PROBE);
        atfork_child_reset();
        let leak = Some(IntervalPtrAction::LeakUnclassified);
        assert_eq!(
            take_atfork_boundary_probe_for_test(),
            vec![
                (leak, true),  // P-c1: locks-held phase OFF before any mutation
                (leak, true),  // P-c2: mid-fold — still the never-reads phase
                (leak, false), // P-c3: guard cleared, gate still armed
            ],
            "the child's table mutations must run under the never-reads \
             phase — a `None` verdict means a boundary classified under \
             ArmedLocksHeld and read the tables this handler was mutating"
        );
        // Behavioral: the fold moved the registration into quarantine
        // coverage, and the reset ends fully disarmed.
        assert!(
            with_registry(|r| r.classify(FOLD_BASE + 8)).is_none(),
            "the registry was cleared"
        );
        assert!(
            with_quarantine(|q| q.classify(FOLD_BASE + 8)).is_some(),
            "the folded range keeps quarantine coverage in the child"
        );
        assert_eq!(
            interval_managed_action(PROBE),
            None,
            "after the reset the interval is fully disarmed"
        );
        assert!(!guard_active_for_test(), "the reset released the guard");
        // Cleanup: purge the folded coverage.
        assert!(with_quarantine(|q| q.unregister(FOLD_BASE)));
    }

    // The SHM_FREE_BYPASS pin lives in `interpose::tests`
    // (`a_foreign_prepare_handler_free_of_a_managed_pointer_stays_classified`):
    // it must take `CB_LOCK` to serialize the process-global counter deltas
    // (a known flake class — a lower bound is satisfied by a parallel
    // test's kind-3 bump even with classification bypassed, so it would pass a
    // classification bypass), and that lock lives beside the other
    // counter-asserting tests there.

    // The child handler releases a guard left SET by a
    // fork from a signal handler that interrupted an interposer frame (the
    // siglongjmp shape: the frame never resumes, nothing else ever clears it,
    // and every later allocator call would take the reentrant arm — ZERO
    // classification, quarantined SHM pointers to glibc free). Drives the
    // extracted `atfork_child_release_guard` directly — the full
    // `atfork_child_reset` clears the PROCESS-GLOBAL registry, which parallel
    // tests own pieces of (the wiring is covered cross-process: with the
    // prepare-set guard, a child that failed to clear would run
    // `probe_atfork`'s quarantined free on the reentrant arm → glibc abort).
    #[test]
    fn the_child_reset_releases_a_stuck_reentrancy_guard() {
        set_guard_for_test();
        assert!(guard_active_for_test(), "precondition: guard stuck");
        atfork_child_release_guard();
        assert!(
            !guard_active_for_test(),
            "the child must not live its whole life on the reentrant arm"
        );
    }

    // The release hand-off, free path: between the atomic unregister and the
    // outside-guard callback, the range must classify as quarantine (a fork
    // child landing in that gap inherits the coverage; a concurrent double
    // free hits a no-op instead of glibc).
    #[test]
    fn a_release_hand_off_keeps_the_range_covered_until_closed() {
        // Read side of the global-tables guard (see its doc).
        let _t = GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        let base = 0x6900_0000_0000usize;
        register_segment_reserving(base, 256, 7).expect("register");
        match classify_and_remove_for_free(base + 8) {
            FreeClass::Release { cookie, handoff } => {
                assert_eq!(cookie, 7);
                assert_eq!(handoff, Some(base), "a fresh hand-off entry was CREATED");
            }
            _ => panic!("expected Release"),
        }
        assert!(
            with_quarantine(|q| q.classify(base + 8)).is_some(),
            "the released range is covered during the callback gap"
        );
        end_release_handoff(base);
        assert!(
            with_quarantine(|q| q.classify(base + 8)).is_none(),
            "the parent closes the hand-off after the callback"
        );
    }

    // The release hand-off, Extended arm: a release whose range merges into
    // PRE-EXISTING quarantine coverage must not schedule a close — closing
    // would delete coverage that predates (and must outlive) this release.
    #[test]
    fn an_extended_hand_off_never_schedules_removal_of_pre_existing_coverage() {
        // Read side of the global-tables guard (see its doc).
        let _t = GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        let base = 0x6a00_0000_0000usize;
        // Guarded insertion (same TEST_LOCK_REENTRY class as the interpose CB
        // tests): a Vec growth's old-buffer free must not re-enter the lock.
        with_control_guard(|| with_quarantine(|q| q.insert_or_extend(base, 512))); // pre-existing
        register_segment_reserving(base, 256, 8).expect("register");
        match classify_and_remove_for_free(base + 8) {
            FreeClass::Release { cookie, handoff } => {
                assert_eq!(cookie, 8);
                assert_eq!(handoff, None, "an EXTENDED insert schedules no close");
            }
            _ => panic!("expected Release"),
        }
        assert!(
            with_quarantine(|q| q.classify(base + 8)).is_some(),
            "the pre-existing coverage survives the release"
        );
        assert!(retire_quarantine(base));
    }

    // ABA hand-off, WIRING: the deterministic overlapping-
    // release interleave through the production seam (classify_free_locked's
    // open_handoff + end_release_handoff's close_handoff). A registers + releases
    // at base and A's callback is "still running" (we hold its ticket, unclosed);
    // B registers + releases the SAME base (overlap); A returns (close) — the
    // range MUST stay covered because B's ref holds it; then B returns (close) —
    // now removed. The bug (unconditional close) would uncover B at A's close.
    // A close_handoff that removes unconditionally reds the
    // "covered after A's close" assert.
    #[test]
    fn overlapping_same_base_releases_stay_covered_until_the_last_close() {
        // Read side of the global-tables guard (see its doc).
        let _t = GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        let base = 0x6b00_0000_0000usize;
        // A: register + release (A's callback conceptually still running).
        register_segment_reserving(base, 256, 0xA).expect("register A");
        let a = match classify_and_remove_for_free(base + 8) {
            FreeClass::Release { cookie, handoff } => {
                assert_eq!(cookie, 0xA);
                assert_eq!(handoff, Some(base), "A creates the hand-off, owes a close");
                handoff
            }
            _ => panic!("expected Release for A"),
        };
        // B: same base re-registered + released while A is open (overlap).
        register_segment_reserving(base, 256, 0xB).expect("register B");
        let b = match classify_and_remove_for_free(base + 8) {
            FreeClass::Release { cookie, handoff } => {
                assert_eq!(cookie, 0xB);
                assert_eq!(handoff, Some(base), "B overlaps, owns its own ref");
                handoff
            }
            _ => panic!("expected Release for B"),
        };
        // A returns first — coverage MUST persist (B still needs it: the ABA point).
        end_release_handoff(a.unwrap());
        assert!(
            with_quarantine(|q| q.classify(base + 8)).is_some(),
            "ABA: A's close must not uncover the range B still depends on"
        );
        // B returns last — now the entry is removed.
        end_release_handoff(b.unwrap());
        assert!(
            with_quarantine(|q| q.classify(base + 8)).is_none(),
            "the last overlapping release removes the shared entry"
        );
    }

    // The inverse interleave through
    // the production seam. A release hand-off opens FIRST (creating the entry);
    // while its callback is out, the SAME BASE is RE-ARMED (the arm-window
    // insert path — with_control_guard + reserve + insert_or_extend, exactly
    // what `arm_window` runs); the hand-off's close (refs 1→0) must KEEP the
    // entry — unregistering there discards the re-armed coverage, and a
    // later SHM free classifies RealHeap → libc. A close that ignores the
    // persistent marker reds the covered-after-close assert.
    #[test]
    fn a_re_arm_during_an_open_handoff_survives_the_close_at_the_seam() {
        // Read side of the global-tables guard (see its doc).
        let _t = GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        let base = 0x6c00_0000_0000usize;
        // A: register + release — the hand-off creates the entry, close owed.
        register_segment_reserving(base, 256, 0xC).expect("register");
        let ticket = match classify_and_remove_for_free(base + 8) {
            FreeClass::Release { cookie, handoff } => {
                assert_eq!(cookie, 0xC);
                assert_eq!(handoff, Some(base), "the hand-off owns a close");
                handoff.unwrap()
            }
            _ => panic!("expected Release"),
        };
        // While A's callback is out: the pool recycles the base and the slot is
        // RE-ARMED — the exact arm-window quarantine insert (fake span; no
        // allocation may occur between "arm" and the close below on this
        // thread, mirroring `arm_window_quarantines_the_slot...`'s note).
        with_control_guard(|| {
            with_both(|reg, quar| {
                quar.reserve(reg.len() + 1);
                quar.insert_or_extend(base, 256);
            })
        });
        // A's callback returns; the close must NOT discard the re-armed slot.
        end_release_handoff(ticket);
        assert!(
            with_quarantine(|q| q.classify(base + 8)).is_some(),
            "INVERSE ABA: the re-armed slot's coverage survives the hand-off close"
        );
        // The re-armed slot lives its normal life: retire tombstones it.
        assert!(retire_quarantine(base));
        assert!(
            with_quarantine(|q| q.classify(base + 8)).is_some(),
            "still covered as a tombstone"
        );
        // Cleanup: full purge of the process-global entry.
        assert!(with_quarantine(|q| q.unregister(base)));
    }

    #[test]
    fn realloc_move_of_a_quarantined_pointer_fails_safe_without_copying() {
        // Read side of the global-tables guard (see its doc).
        let _t = GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        // A quarantined slot has no per-allocation extent, so a realloc of a
        // pointer into it must copy NOTHING and fail (null) rather than
        // disclose later slot bytes.
        //
        // KILL SHAPE: the closure RECORDS and returns instead of
        // panicking, and cleanup runs BEFORE the asserts. In this test binary
        // the interposers are LIVE (the lib's `#[no_mangle]` malloc family
        // preempts glibc's inside the test executable), and this test calls
        // `realloc_move_registered` DIRECTLY — no interposer frame, so the
        // re-entry flag is NOT set. A panic raised while `with_both` holds the
        // spin locks therefore deadlocks in the panic HOOK: the hook's own
        // allocator traffic (libtest capture-buffer realloc, temp frees)
        // re-enters classification and spins on the held lock on the SAME
        // thread, BEFORE any unwind landing pad (the RAII `LockGuard`) can
        // run. A defect must surface as a red assert, never a suite hang.
        let base = 0x6150_0000_0000usize;
        // Guarded insertion (TEST_LOCK_REENTRY class — see the interpose pin).
        with_control_guard(|| with_quarantine(|q| q.insert_or_extend(base, 256)));
        let copied = core::cell::Cell::new(false);
        let mv = realloc_move_registered(base + 8, |_avail| {
            copied.set(true);
            core::ptr::null_mut()
        })
        .expect("classified as quarantined");
        // Cleanup BEFORE the asserts so a red kill cannot leak the entry into
        // the process-global quarantine other tests share.
        assert!(retire_quarantine(base));
        assert!(
            !copied.get(),
            "a quarantined-pointer realloc must never run the copy closure"
        );
        assert!(mv.dst.is_null(), "fails safe with a null result");
        assert!(mv.release.is_none(), "nothing is released on a fail-safe");
    }

    #[test]
    fn a_failed_realloc_move_leaves_the_segment_registered_and_schedules_no_release() {
        // Read side of the global-tables guard (see its doc).
        let _t = GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        let base = 0x6100_0000_0000usize;
        with_registry(|r| r.register(base, 256, 0xDEF).expect("register"));
        let mv = realloc_move_registered(base + 8, |_avail| core::ptr::null_mut())
            .expect("classified as a segment");
        assert!(mv.dst.is_null());
        assert_eq!(mv.release, None, "a failed move never releases the sample");
        assert!(
            with_registry(|r| r.classify(base + 8)).is_some(),
            "the old block stays registered so its later free routes correctly"
        );
        with_registry(|r| r.unregister(base));
    }

    #[test]
    fn realloc_move_registered_of_an_unrelated_pointer_is_none() {
        // Read side of the global-tables guard (see its doc).
        let _t = GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        assert!(
            realloc_move_registered(0x6200_0000_0000usize, |_| core::ptr::null_mut()).is_none()
        );
    }

    // Refutation pin: a successful move schedules the OLD
    // cookie for an outside-lock release; if the consumer then registers a NEW
    // segment at the SAME address before that release fires, the OLD sample must
    // STILL be released (suppressing it would LEAK) via the OLD cookie (release
    // is cookie-identified, not address-identified), and the NEW registration
    // must route a later free to the NEW cookie. This is exactly the sequence an
    // external reproduction called a bug; it is correct behaviour.
    #[test]
    fn a_same_address_reregistration_does_not_corrupt_the_old_release_or_new_routing() {
        // Read side of the global-tables guard (see its doc).
        let _t = GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        let base = 0x6700_0000_0000usize;
        const OLD_COOKIE: usize = 0xAA;
        const NEW_COOKIE: usize = 0xBB;
        register_segment_reserving(base, 256, OLD_COOKIE).expect("register old");

        // The move captures + schedules the OLD cookie under the lock and removes
        // the old registration atomically (parking the release hand-off).
        let mv = realloc_move_registered(base + 8, |_avail| 0x4000 as *mut c_void)
            .expect("classified as a segment");
        assert_eq!(
            mv.release,
            Some(ReleaseTicket {
                addr: base + 8,
                cookie: OLD_COOKIE,
                handoff: Some(base),
            }),
            "the OLD sample is scheduled for release by its OLD cookie — never suppressed"
        );
        assert!(
            with_registry(|r| r.classify(base + 8)).is_none(),
            "the old registration was removed atomically under the lock"
        );

        // The consumer now re-uses the address (a lifecycle violation, but the
        // crate stays correct): register a NEW segment at the same base BEFORE
        // the OLD release fires (its hand-off entry is still parked).
        register_segment_reserving(base, 256, NEW_COOKIE).expect("register new");

        // A later free of the replacement routes to the NEW cookie — the stale
        // OLD-cookie release cannot touch it (cookie identity: registry is
        // classified BEFORE quarantine), and the NEW registration is intact.
        // Its own hand-off is an OVERLAPPING release on the still-parked entry
        // (the first's ref is still open), so it OWNS A REF and owes its own
        // close (the ABA fix) — `Some(base)`, not `None`.
        match classify_and_remove_for_free(base + 8) {
            FreeClass::Release { cookie, handoff } => {
                assert_eq!(
                    cookie, NEW_COOKIE,
                    "the replacement frees by its OWN cookie, never the released old one"
                );
                assert_eq!(
                    handoff,
                    Some(base),
                    "an overlapping same-base release owns its own ref (ABA-safe)"
                );
            }
            FreeClass::WindowNoop => panic!("expected Release, got WindowNoop"),
            FreeClass::QuarantineNoop => panic!("expected Release, got QuarantineNoop"),
            FreeClass::TombstoneNoop => panic!("expected Release, got TombstoneNoop"),
            FreeClass::RealHeap => panic!("expected Release, got RealHeap"),
        }
        // Cleanup: the FIRST release's hand-off was never closed in this test
        // (no interposer drove it) — fully PURGE the parked entry (retire now
        // tombstones rather than removes, so use the internal unregister).
        assert!(with_quarantine(|q| q.unregister(base)));
    }

    // Wiring note: `register_segment_reserving` and
    // `arm_window` keep `quarantine.spare_capacity() >= registry.len()`
    // CONTINUOUSLY (reserve BEFORE the mutation, in one lock scope) so the
    // child fold never reallocates. The no-realloc PRIMITIVE is pinned deterministically by
    // `registry::tests::reserving_for_n_new_bases_makes_a_fold_of_them_allocation_free`
    // (isolated `SegmentRegistry` instances — no global-state race); at a real
    // `fork()` the `debug_assert` in `atfork_child_reset` + a
    // multithreaded-fork reproduction verify the wired path. A state-level unit
    // test is deliberately NOT added: the invariant is over the PROCESS-GLOBAL
    // registry/quarantine, which other parallel unit tests grow without
    // reserving, so any assertion tying spare capacity to `registry.len()` here
    // would flake under `--test-threads > 1`.

    // If `fork()` is called from a signal handler that interrupted
    // THIS thread inside a window closure, the child inherits the WINDOW
    // `RefCell` in its BORROWED state. The child's disarm must tolerate that —
    // degrade (window left armed), never a `BorrowMutError` panic, which would
    // abort the child from inside the atfork handler. Same-thread model of the
    // inherited state: hold a live borrow and run the child disarm under it.
    // Reverting `try_borrow_mut` to `borrow_mut` panics here.
    #[test]
    fn the_child_window_disarm_tolerates_an_inherited_mid_borrow_cell() {
        WINDOW.with(|w| {
            *w.borrow_mut() = Some(BorrowWindow::new(0x7100_0000_0000, 0x7100_0000_1000));
            let held = w.borrow(); // the inherited mid-borrow state
            clear_window_forget_ledger(); // must NOT panic
            assert!(held.is_some(), "the interrupted frame's view stays valid");
        });
        // The degrade left the window ARMED (the documented residual under the
        // caller-UB precondition)...
        assert!(
            WINDOW.with(|w| w.borrow().is_some()),
            "on a mid-borrow cell the disarm degrades to a left-armed window"
        );
        // ...and once the borrow is gone (the interrupted frame completed), the
        // same disarm takes the window — the normal arm still works.
        clear_window_forget_ledger();
        assert!(
            WINDOW.with(|w| w.borrow().is_none()),
            "an unborrowed cell is taken exactly as before"
        );
    }

    // When the child's disarm CANNOT clear an inherited
    // mid-borrow window, that thread's window machinery must be POISONED — the
    // next malloc must succeed via the REAL allocator and provably NOT advance
    // the leftover window. In this self-interposed test binary `libc::malloc`
    // IS our interposer, so the calls below drive the production path.
    // Each of these fails deterministically: drop the poison SET or the
    // `with_window_mut` gate → the first malloc panics (BorrowMutError on the
    // held borrow); drop the `with_window` gate → the free in the stuck-MUT
    // phase panics; drop the `arm_window` / `disarm_window` gate → the
    // Unavailable / None asserts fail.
    #[test]
    fn a_poisoned_child_window_is_bypassed_never_bumped() {
        // Read side of the global-tables guard (see its doc).
        let _t = GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        let base = 0x7200_0000_0000usize;
        let tail = base + 0x1000;
        WINDOW.with(|w| {
            *w.borrow_mut() = Some(BorrowWindow::new(base, tail));
            let held = w.borrow(); // the inherited mid-borrow state
            clear_window_forget_ledger(); // Err arm → poisons this thread
                                          // The child's next malloc WHILE the interrupted frame is live:
                                          // bypasses to the real allocator — no panic, no bump.
            let p = unsafe { libc::malloc(64) } as usize;
            assert!(p != 0, "allocation still succeeds via the real allocator");
            assert!(
                !(base..tail).contains(&p),
                "a poisoned window is never bumped (in-slot pointer returned)"
            );
            unsafe { libc::free(p as *mut c_void) };
            assert!(held.is_some(), "the interrupted frame's view stays valid");
        });
        // The interrupted frame completed (borrow released); the window
        // survived ARMED but the poison still bypasses every path.
        let p2 = unsafe { libc::malloc(64) } as usize;
        assert!(p2 != 0 && !(base..tail).contains(&p2));
        unsafe { libc::free(p2 as *mut c_void) };
        // The stuck-MUT shape (a fork interrupting `with_window_mut`): a free
        // must also bypass — `with_window`'s shared `borrow()` would panic on
        // a stuck MUT borrow without its gate.
        WINDOW.with(|w| {
            let _held_mut = w.borrow_mut();
            let p3 = unsafe { libc::malloc(32) } as usize;
            assert!(p3 != 0 && !(base..tail).contains(&p3));
            unsafe { libc::free(p3 as *mut c_void) };
        });
        // The leftover window's counters are FROZEN — nothing ever advanced it.
        WINDOW.with(|w| {
            let slot = w.borrow();
            let win = slot
                .as_ref()
                .expect("window left armed (the documented degrade)");
            assert_eq!(
                win.ledger().len(),
                0,
                "no allocation ever bumped the window"
            );
        });
        // Arm/disarm are inert on a poisoned thread (both would touch the cell).
        assert_eq!(arm_window(base, tail), ArmOutcome::Unavailable);
        assert!(disarm_window().is_none());
    }

    #[test]
    fn classify_usable_of_a_segment_interior_is_conservative_zero() {
        // Read side of the global-tables guard (see its doc).
        let _t = GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        // An interior pointer must NOT report bytes-to-end-of-range (the
        // over-report that would let a caller write past the forged capacity).
        let base = 0x6300_0000_0000usize;
        with_registry(|r| r.register(base, 256, 1).expect("register"));
        match classify_usable(base + 8) {
            UsableOutcome::Exact(n) => assert_eq!(n, 0, "segment usable is conservative 0"),
            UsableOutcome::RealHeap => panic!("a registered segment is hook-managed"),
        }
        with_registry(|r| r.unregister(base));
    }

    #[test]
    fn classify_usable_of_an_unrelated_pointer_is_real_heap() {
        // Read side of the global-tables guard (see its doc).
        let _t = GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        assert!(matches!(
            classify_usable(0x6400_0000_0000usize),
            UsableOutcome::RealHeap
        ));
    }

    #[test]
    fn a_zero_size_bootstrap_alloc_is_a_distinct_in_arena_pointer() {
        // reserve >= 1 means a size-0 allocation still advances the cursor, so
        // two of them are DISTINCT and BOTH are strictly inside the arena (never
        // a one-past-the-end pointer that `bootstrap_contains` would reject).
        let a = bootstrap_alloc(0, 16, false) as usize;
        let b = bootstrap_alloc(0, 16, false) as usize;
        assert_ne!(a, 0);
        assert_ne!(b, 0);
        assert_ne!(a, b, "size-0 allocations get distinct addresses");
        assert!(bootstrap_contains(a) && bootstrap_contains(b));
        // SAFETY: both are live bootstrap allocations.
        unsafe {
            assert_eq!(bootstrap_size(a), 0, "the recorded size is the true 0");
            assert_eq!(bootstrap_size(b), 0);
        }
    }

    #[test]
    fn arm_window_quarantines_the_slot_before_publishing_the_window() {
        // Read side of the global-tables guard (see its doc).
        let _t = GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        // NOTE: no allocation may occur between arm and disarm (a real window is
        // armed on THIS thread over a FAKE span — a malloc would bump into it).
        let base = 0x6500_0000_0000usize;
        let tail = base + 4096;
        let outcome = arm_window(base, tail);
        let in_quar_while_armed = with_quarantine(|q| q.classify(base + 8).is_some());
        let _ = disarm_window();
        // Now safe to allocate (window is gone).
        let in_quar_after_disarm = with_quarantine(|q| q.classify(base + 8).is_some());
        let live_before_retire =
            with_quarantine(|q| q.classify(base + 8).map(|s| s.tombstoned()).unwrap_or(true));

        // Tombstone-on-retire: retire TOMBSTONES (does not delete) — the
        // extent still COVERS its range so a late in-slot free stays a no-op.
        let retired = retire_quarantine(base);
        let covered_after_retire = with_quarantine(|q| q.classify(base + 8).is_some());
        let tombstoned_after_retire = with_quarantine(|q| {
            q.classify(base + 8)
                .map(|s| s.tombstoned())
                .unwrap_or(false)
        });

        // A re-arm at the same base REVIVES the tombstone (recycled slot) — the
        // entry is live coverage again, not a duplicate.
        let re_armed = arm_window(base, tail);
        let revived = with_quarantine(|q| {
            q.classify(base + 8)
                .map(|s| !s.tombstoned())
                .unwrap_or(false)
        });
        let _ = disarm_window();

        assert_eq!(outcome, ArmOutcome::Armed);
        assert!(
            in_quar_while_armed,
            "the slot is in the GLOBAL quarantine from arm time (coverage before exposure)"
        );
        assert!(
            in_quar_after_disarm,
            "the quarantine extent outlives the window until retire_slot"
        );
        assert!(
            !live_before_retire,
            "the extent is LIVE (not tombstoned) before retire"
        );
        assert!(retired, "retire_slot finds and tombstones the extent");
        assert!(
            covered_after_retire,
            "retire TOMBSTONES — the extent still covers its range (a late in-slot free stays a no-op)"
        );
        assert!(tombstoned_after_retire, "the extent is now a tombstone");
        assert_eq!(re_armed, ArmOutcome::Armed, "a recycled slot re-arms");
        assert!(
            revived,
            "re-arm REVIVES the tombstone (recycled, not duplicated)"
        );

        // Cleanup: fully purge the (now-revived-then-disarmed) entry.
        assert!(with_quarantine(|q| q.unregister(base)));
    }
}
