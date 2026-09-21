// SPDX-License-Identifier: AGPL-3.0-only
//! `heaphook_probe` — the heaphook e2e probe.
//!
//! A standalone binary the Linux e2e test `LD_PRELOAD`s
//! `libcerulion_heaphook.so` onto. It DELIBERATELY does not depend on the
//! `cerulion_heaphook` crate: the interposers must reach it ONLY through the
//! preload, never a static link, so a run with no preload is a clean control and
//! `dlsym("cerulion_heaphook_abi")` genuinely misses when the hook is absent.
//!
//! It is also the reference rmw borrow CONSUMER: it handshakes by `dlsym`, arms a
//! borrow window over a stand-in "loan slot", drives the malloc family into it,
//! range-tests the result, reads escapes, and exercises the segment-release
//! path — exactly what the rmw will do in Rust/C.
//!
//! Dispatched by `CER_HEAPHOOK_PROBE`; prints raw FACT lines (`key=value`) plus a
//! final `PROBE_DONE` marker. The oracles live in the parent test.

use std::ffi::CStr;
use std::sync::atomic::{AtomicUsize, Ordering};

use libc::{c_char, c_int, c_void};

// ── exported-symbol fn types (must match cerulion_heaphook::exports) ─────────

type VersionFn = unsafe extern "C" fn() -> u32;
type StatusFn = unsafe extern "C" fn() -> u32;
type ArmFn = unsafe extern "C" fn(*mut c_void, *mut c_void) -> c_int;
type DisarmFn = unsafe extern "C" fn() -> c_int;
type EscapeFn = unsafe extern "C" fn() -> c_int;
type RangeTestFn = unsafe extern "C" fn(*const c_void, usize) -> c_int;
type RegisterFn = unsafe extern "C" fn(*mut c_void, usize, usize) -> c_int;
type UnregisterFn = unsafe extern "C" fn(*mut c_void) -> c_int;
type ReleaseCb = unsafe extern "C" fn(*mut c_void, usize);
type SetReleaseFn = unsafe extern "C" fn(Option<ReleaseCb>) -> c_int;
type RetireFn = unsafe extern "C" fn(*mut c_void) -> c_int;
type CounterFn = unsafe extern "C" fn(u32) -> u64;

/// `dlsym(RTLD_DEFAULT, name)` — the preloaded hook's export, or null if absent.
fn sym(name: &CStr) -> *mut c_void {
    // SAFETY: RTLD_DEFAULT + a valid C string.
    unsafe { libc::dlsym(libc::RTLD_DEFAULT, name.as_ptr() as *const c_char) }
}

/// Transmute a resolved symbol to a fn type (panics if absent — every probe but
/// `handshake` requires the hook preloaded).
fn need<T: Copy>(name: &CStr) -> T {
    let p = sym(name);
    assert!(
        !p.is_null(),
        "symbol {name:?} not found — is the hook preloaded?"
    );
    assert_eq!(
        std::mem::size_of::<T>(),
        std::mem::size_of::<*mut c_void>(),
        "fn-ptr size mismatch"
    );
    // SAFETY: T is a fn-pointer type matching the export's signature.
    unsafe { std::mem::transmute_copy::<*mut c_void, T>(&p) }
}

// ── the segment-release recorder (the take-side stand-in for the rmw) ────────

static RELEASED_PTR: AtomicUsize = AtomicUsize::new(0);
static RELEASED_COOKIE: AtomicUsize = AtomicUsize::new(usize::MAX);
static RELEASE_CALLS: AtomicUsize = AtomicUsize::new(0);

unsafe extern "C" fn on_release(ptr: *mut c_void, cookie: usize) {
    RELEASED_PTR.store(ptr as usize, Ordering::SeqCst);
    RELEASED_COOKIE.store(cookie, Ordering::SeqCst);
    RELEASE_CALLS.fetch_add(1, Ordering::SeqCst);
}

fn main() {
    let probe = std::env::var("CER_HEAPHOOK_PROBE").unwrap_or_default();
    match probe.as_str() {
        "handshake" => probe_handshake(),
        "window" => probe_window(),
        "escape" => probe_escape(),
        "segment" => probe_segment(),
        "loop" => probe_loop(),
        "atfork" => probe_atfork(),
        "armed_fork" => probe_armed_fork(),
        // `memalign`/`reallocarray`/`malloc_usable_size` are glibc-only.
        #[cfg(all(target_os = "linux", target_env = "gnu"))]
        "aligned" => probe_aligned(),
        #[cfg(all(target_os = "linux", target_env = "gnu"))]
        "reallocarray" => probe_reallocarray(),
        "guard_stress" => probe_guard_stress(),
        "quarantine" => probe_quarantine(),
        "segment_realloc" => probe_segment_realloc(),
        // `__errno_location` is glibc-only; the disclosure e2e is Linux-only.
        #[cfg(all(target_os = "linux", target_env = "gnu"))]
        "disarmed_realloc" => probe_disarmed_realloc(),
        "realloc_escape" => probe_realloc_escape(),
        "calloc_zero" => probe_calloc_zero(),
        "counter" => probe_counter(),
        other => panic!("unknown probe {other:?}"),
    }
    println!("PROBE_DONE");
}

/// Handshake: version + won-malloc status (the only probe that may run WITHOUT
/// the hook — then the symbol is absent and `present=0`).
fn probe_handshake() {
    let abi = sym(c"cerulion_heaphook_abi");
    let present = !abi.is_null();
    if !present {
        println!("present=0");
        return;
    }
    let version: VersionFn = need(c"cerulion_heaphook_version");
    let status: StatusFn = need(c"cerulion_heaphook_status");
    // SAFETY: resolved exports.
    let (v, s) = unsafe { (version(), status()) };
    println!("present=1");
    println!("version={v}");
    println!("status_won={}", s & 1);
}

/// A stand-in "loan slot": a real-heap buffer allocated BEFORE any window is
/// armed (so it comes from the real allocator, not the window).
fn alloc_slot(len: usize) -> *mut u8 {
    // SAFETY: plain malloc; no window armed yet.
    let p = unsafe { libc::malloc(len) } as *mut u8;
    assert!(!p.is_null(), "slot alloc failed");
    p
}

fn probe_window() {
    let arm: ArmFn = need(c"cerulion_heaphook_arm_window");
    let disarm: DisarmFn = need(c"cerulion_heaphook_disarm_window");
    let escape: EscapeFn = need(c"cerulion_heaphook_window_escape");
    let range: RangeTestFn = need(c"cerulion_heaphook_window_range_test");
    let retire: RetireFn = need(c"cerulion_heaphook_retire_slot");

    let slot_len = 4096usize;
    let slot = alloc_slot(slot_len);
    let base = slot as usize;
    let tail = base + slot_len;

    // Tight armed section — no Rust allocation between arm and disarm.
    // SAFETY: the fns are resolved exports; `slot` is a live buffer.
    let (in_slot, rt, esc, disarm_rc) = unsafe {
        arm(base as *mut c_void, tail as *mut c_void);
        let p = libc::malloc(64) as usize;
        let in_slot = (p >= base && p < tail) as i32;
        let rt = range(p as *const c_void, 64);
        let esc = escape();
        let disarm_rc = disarm();
        (in_slot, rt, esc, disarm_rc)
    };
    // The slot's extent is QUARANTINED (from arm), so `free(slot)` would be a
    // no-op until retired — retire first so the real buffer is actually freed.
    // SAFETY: after retire, `slot` classifies RealHeap → the real allocator.
    unsafe {
        retire(base as *mut c_void);
        libc::free(slot as *mut c_void);
    }

    println!("in_slot={in_slot}");
    println!("range_test={rt}");
    println!("escape={esc}");
    println!("disarm_rc={disarm_rc}");
}

fn probe_escape() {
    let arm: ArmFn = need(c"cerulion_heaphook_arm_window");
    let disarm: DisarmFn = need(c"cerulion_heaphook_disarm_window");
    let escape: EscapeFn = need(c"cerulion_heaphook_window_escape");
    let retire: RetireFn = need(c"cerulion_heaphook_retire_slot");

    let slot_len = 64usize;
    let slot = alloc_slot(slot_len);
    let base = slot as usize;
    let tail = base + slot_len;

    // SAFETY: resolved exports; live slot. A 128-byte bump overflows the 64-byte
    // tail → GrowthPastTail escape, and the interposer serves real heap.
    let (in_slot, esc, disarm_rc, spilled) = unsafe {
        arm(base as *mut c_void, tail as *mut c_void);
        let p = libc::malloc(128) as usize;
        let in_slot = (p >= base && p < tail) as i32;
        let esc = escape();
        let disarm_rc = disarm();
        (in_slot, esc, disarm_rc, p)
    };
    // `spilled` came from the real allocator (overflow spill) → a real free. The
    // slot is quarantined from arm → retire before freeing it for real.
    // SAFETY: spilled is real-heap; slot is RealHeap after retire.
    unsafe {
        libc::free(spilled as *mut c_void);
        retire(base as *mut c_void);
        libc::free(slot as *mut c_void);
    }

    println!("in_slot={in_slot}");
    println!("escape={esc}");
    println!("disarm_rc={disarm_rc}");
}

fn probe_segment() {
    let register: RegisterFn = need(c"cerulion_heaphook_register_segment");
    let set_release: SetReleaseFn = need(c"cerulion_heaphook_set_release_callback");

    let seg = alloc_slot(256);
    let seg_addr = seg as usize;
    let interior = seg_addr + 8;
    let cookie = 0xC0FFEEusize;

    let retire: RetireFn = need(c"cerulion_heaphook_retire_slot");

    // SAFETY: resolved exports; `seg` is a live buffer standing in for an SHM
    // range. Freeing an interior pointer of a registered range must route to
    // `on_release` (recording it) exactly ONCE, NEVER to glibc free. The callback
    // is cleared BEFORE the snapshot so a stray later callback cannot slip in.
    let reregister_rc = unsafe {
        set_release(Some(on_release));
        register(seg as *mut c_void, 256, cookie);
        libc::free(interior as *mut c_void); // → release + unregister + quarantine
        set_release(None);
        // ANTI-VACUITY: the free must have UNREGISTERED the range. Re-registering
        // the SAME bytes now succeeds (rc 0) only if it was removed from the
        // registry — a broken unregister would leave it registered and this
        // register would overlap-reject.
        register(seg as *mut c_void, 256, cookie)
    };

    let calls = RELEASE_CALLS.load(Ordering::SeqCst);
    let ptr_match = (RELEASED_PTR.load(Ordering::SeqCst) == interior) as i32;
    let cookie_match = (RELEASED_COOKIE.load(Ordering::SeqCst) == cookie) as i32;

    // Clean up: unregister the re-registration, retire the quarantine entry the
    // first free folded in, then free the real buffer.
    // SAFETY: after unregister + retire, `seg` classifies RealHeap.
    unsafe {
        let unregister: UnregisterFn = need(c"cerulion_heaphook_unregister_segment");
        unregister(seg as *mut c_void);
        retire(seg as *mut c_void);
        libc::free(seg as *mut c_void);
    }

    println!("release_calls={calls}");
    println!("released_ptr_match={ptr_match}");
    println!("released_cookie_match={cookie_match}");
    println!("segment_reregistered_ok={}", (reregister_rc == 0) as i32);
}

/// A malloc-heavy loop with NO window armed. Its checksum is over WRITTEN/READ
/// values (never addresses), so the hook — which forwards to the real allocator
/// when no window is armed — must produce a byte-identical result to a run with
/// no preload at all.
fn probe_loop() {
    let mut sum: u64 = 0;
    const FNV: u64 = 1099511628211;
    for i in 0..2000u64 {
        let sz = 16 + (i % 200) as usize;
        // SAFETY: standard malloc/write/read/realloc/free on owned buffers.
        unsafe {
            let p = libc::malloc(sz) as *mut u8;
            if p.is_null() {
                continue;
            }
            for j in 0..sz {
                *p.add(j) = ((i.wrapping_add(j as u64)) & 0xff) as u8;
            }
            for j in 0..sz {
                sum = sum.wrapping_add(*p.add(j) as u64).wrapping_mul(FNV);
            }
            let sz2 = sz + 32;
            let p2 = libc::realloc(p as *mut c_void, sz2) as *mut u8;
            if p2.is_null() {
                libc::free(p as *mut c_void);
                continue;
            }
            for j in sz..sz2 {
                *p2.add(j) = 0xAA;
            }
            for j in 0..sz2 {
                sum = sum.wrapping_add(*p2.add(j) as u64);
            }
            libc::free(p2 as *mut c_void);
        }
    }
    // calloc must be zeroed both ways.
    // SAFETY: calloc then read-back then free of an owned buffer.
    unsafe {
        let c = libc::calloc(64, 4) as *mut u8;
        if !c.is_null() {
            for j in 0..256 {
                sum = sum.wrapping_add(*c.add(j) as u64);
            }
            libc::free(c as *mut c_void);
        }
    }
    println!("checksum={sum:016x}");
    // Whether the hook is actually LOADED: ld.so treats a failed
    // LD_PRELOAD as a warning and continues, so the e2e byte-identity oracle
    // must be able to tell a hooked run from a silently-unhooked one.
    println!(
        "hook_present={}",
        (!sym(c"cerulion_heaphook_abi").is_null()) as i32
    );
}

fn probe_atfork() {
    let register: RegisterFn = need(c"cerulion_heaphook_register_segment");
    let unregister: UnregisterFn = need(c"cerulion_heaphook_unregister_segment");

    let seg = alloc_slot(64);
    // SAFETY: register [seg, seg+64) in the PARENT, then fork.
    let (parent_register_ok, cleared) = unsafe {
        // Anti-vacuity: the parent registration must SUCCEED, else the fork child
        // would inherit nothing to fold and the survival below proves nothing.
        let parent_rc = register(seg as *mut c_void, 64, 1);
        let pid = libc::fork();
        // A failed fork() must be a PROBE failure — waiting on pid -1 would
        // evaluate the initialized status and report child success with no
        // child (and no atfork handler) ever having run.
        assert!(pid >= 0, "fork failed in probe_atfork");
        if pid == 0 {
            // Child: (1) re-register a SMALLER range FIRST — success proves the
            // registry was CLEARED (the inherited [seg,seg+64) was folded into
            // the quarantine, not left registered). (2) Free an address OUTSIDE
            // the new [seg,seg+32) but INSIDE the inherited quarantine
            // [seg,seg+64): a NO-OP if the fold happened, a glibc free of an
            // interior pointer → ABORT if the child merely cleared the registry.
            let rc = register(seg as *mut c_void, 32, 2);
            libc::free((seg as usize + 48) as *mut c_void);
            libc::_exit(if rc == 0 { 0 } else { 1 });
        }
        // Parent: reap. exit 0 ⇒ the fold worked (no abort) AND the registry was
        // cleared in the child. The parent's own registration is untouched.
        let mut status: c_int = 0;
        // Checked waitpid: a failure would read the zeroed
        // status as success with no child ever having run.
        let rc = libc::waitpid(pid, &mut status, 0);
        assert!(rc == pid, "waitpid failed in probe_atfork");
        let cleared = (libc::WIFEXITED(status) && libc::WEXITSTATUS(status) == 0) as i32;
        unregister(seg as *mut c_void);
        libc::free(seg as *mut c_void); // now RealHeap → real free
        ((parent_rc == 0) as i32, cleared)
    };
    println!("atfork_parent_register_ok={parent_register_ok}");
    println!("atfork_child_cleared={cleared}");
}

/// A window ARMED across a `fork` — the slot's extent is in the quarantine from
/// arm time, so the child (which inherits the quarantine) freeing an in-slot
/// pointer is a NO-OP, not a glibc free of shared memory. Pins that the borrow
/// window is sound beyond its own thread / process.
fn probe_armed_fork() {
    let arm: ArmFn = need(c"cerulion_heaphook_arm_window");
    let disarm: DisarmFn = need(c"cerulion_heaphook_disarm_window");
    let retire: RetireFn = need(c"cerulion_heaphook_retire_slot");
    let range: RangeTestFn = need(c"cerulion_heaphook_window_range_test");

    let slot = alloc_slot(4096);
    let base = slot as usize;
    let tail = base + 4096;

    // SAFETY: arm, bump TWO in-slot allocations, then fork WHILE armed.
    let (p_in_window, p2_interior, survived) = unsafe {
        arm(base as *mut c_void, tail as *mut c_void);
        let p = libc::malloc(64) as usize; // first bump — lands AT the slot base
        let p2 = libc::malloc(64) as usize; // second bump — strictly INTERIOR
                                            // Anti-vacuity: the FIRST bump equals the slot base,
                                            // which is itself a valid glibc pointer (`alloc_slot`'s malloc) — a
                                            // MISROUTED free of it would not abort, so it proves nothing. The
                                            // child's oracle rides `p2`, an interior address whose glibc free
                                            // aborts loudly if quarantine routing is broken.
        let p_in_window = range(p as *const c_void, 64);
        let p2_interior = ((range(p2 as *const c_void, 64) == 1) && p2 != base) as i32;
        let pid = libc::fork();
        // A failed fork() must be a PROBE failure, never a vacuous pass.
        assert!(pid >= 0, "fork failed in probe_armed_fork");
        if pid == 0 {
            // DISCRIMINATOR: the child's window TLS must be CLEARED by the atfork
            // child handler. A fresh allocation must therefore land on the REAL
            // heap, NOT inside the parent's window span — if the TLS were not
            // cleared, the child would inherit the armed window and the fresh
            // malloc would bump into [base, tail). Exit 2 flags that failure.
            let fresh = libc::malloc(64) as usize;
            if fresh == 0 {
                // A NULL child allocation is a PROBE failure — 0 < base would
                // otherwise masquerade as "TLS cleared". Exit 3 flags it.
                libc::_exit(3);
            }
            let tls_cleared = fresh < base || fresh >= tail;
            // The INTERIOR in-slot pointer is the discriminating free — a
            // quarantine no-op if routing works, a loud glibc interior-pointer
            // abort if it does not. `p` (== slot base) frees too, and `fresh`
            // is real heap so it frees for real.
            libc::free(p2 as *mut c_void);
            libc::free(p as *mut c_void);
            libc::free(fresh as *mut c_void);
            libc::_exit(if tls_cleared { 0 } else { 2 });
        }
        let mut status: c_int = 0;
        // An unchecked waitpid failure would evaluate the zero-initialized
        // status as a clean exit — a vacuous pass.
        let rc = libc::waitpid(pid, &mut status, 0);
        assert!(rc == pid, "waitpid failed in probe_armed_fork");
        // exit 0 ⇒ no abort (in-slot frees were no-ops) AND the child TLS cleared.
        let ok = (libc::WIFEXITED(status) && libc::WEXITSTATUS(status) == 0) as i32;
        disarm();
        retire(base as *mut c_void);
        libc::free(slot as *mut c_void); // retired → RealHeap → real free
        (p_in_window, p2_interior, ok)
    };
    println!("armed_fork_p_in_window={p_in_window}");
    println!("armed_fork_p2_interior={p2_interior}");
    println!("armed_fork_survived={survived}");
}

/// The aligned allocators (`posix_memalign`/`aligned_alloc`/`memalign`) bump into
/// an armed window with correct alignment, and `malloc_usable_size` of a window
/// pointer reports the EXACT ledger size (never the over-report hazard).
/// glibc-only: `memalign`/`malloc_usable_size` are not in macOS libc.
#[cfg(all(target_os = "linux", target_env = "gnu"))]
fn probe_aligned() {
    let arm: ArmFn = need(c"cerulion_heaphook_arm_window");
    let disarm: DisarmFn = need(c"cerulion_heaphook_disarm_window");
    let retire: RetireFn = need(c"cerulion_heaphook_retire_slot");

    let slot = alloc_slot(1 << 16);
    let base = slot as usize;
    let tail = base + (1 << 16);

    // SAFETY: resolved exports; live slot.
    let (pm_ok, aa_ok, ma_ok, usable) = unsafe {
        arm(base as *mut c_void, tail as *mut c_void);
        let mut pm: *mut c_void = core::ptr::null_mut();
        let rc = libc::posix_memalign(&mut pm, 64, 128);
        let pm_ok = ((rc == 0)
            && (pm as usize) >= base
            && (pm as usize) < tail
            && (pm as usize).is_multiple_of(64)) as i32;
        let aa = libc::aligned_alloc(32, 64) as usize;
        let aa_ok = ((aa >= base) && (aa < tail) && aa.is_multiple_of(32)) as i32;
        let ma = libc::memalign(16, 48) as usize;
        let ma_ok = ((ma >= base) && (ma < tail) && ma.is_multiple_of(16)) as i32;
        // usable of a window pointer is the EXACT request (100), not the slack.
        let mp = libc::malloc(100);
        let usable = libc::malloc_usable_size(mp);
        disarm();
        retire(base as *mut c_void);
        libc::free(slot as *mut c_void); // retired → real free
        (pm_ok, aa_ok, ma_ok, usable)
    };
    println!("posix_memalign_in_slot_aligned={pm_ok}");
    println!("aligned_alloc_in_slot_aligned={aa_ok}");
    println!("memalign_in_slot_aligned={ma_ok}");
    println!("window_usable_exact={}", (usable == 100) as i32);
}

/// `reallocarray` of a pointer into a registered segment routes like `realloc`
/// (overflow-checked `nmemb * size`), releasing the sample once. glibc-only.
#[cfg(all(target_os = "linux", target_env = "gnu"))]
fn probe_reallocarray() {
    let register: RegisterFn = need(c"cerulion_heaphook_register_segment");
    let set_release: SetReleaseFn = need(c"cerulion_heaphook_set_release_callback");
    let retire: RetireFn = need(c"cerulion_heaphook_retire_slot");

    let seg = alloc_slot(256);
    let interior = seg as usize + 8;
    // SAFETY: register, then reallocarray an interior pointer (4 * 64 = 256).
    let (calls, off_seg, non_null, usable_ok, data_ok) = unsafe {
        // Pattern-fill the segment so the move's data copy is checkable.
        for i in 0..256usize {
            *seg.add(i) = (i as u8) ^ 0xA5;
        }
        set_release(Some(on_release));
        register(seg as *mut c_void, 256, 0x1234);
        let q = libc::reallocarray(interior as *mut c_void, 4, 64) as usize;
        let calls = RELEASE_CALLS.load(Ordering::SeqCst);
        // A NULL result is a FAILURE — require non-null AND out-of-segment, else
        // a failing reallocarray that still released the sample would pass
        // (`free(NULL)` no-ops, `q < seg` would be true for NULL).
        let non_null = (q != 0) as i32;
        let off = (q != 0 && ((q < seg as usize) || (q >= seg as usize + 256))) as i32;
        // The FULL `nmemb * size` request must reach the allocator: an
        // implementation that ignored `size` (e.g. `realloc(ptr, nmemb)`)
        // would get a minimum-chunk usable size far below 256.
        let usable_ok = (q != 0 && libc::malloc_usable_size(q as *mut c_void) >= 256) as i32;
        // The moved bytes are the segment tail from `interior` (248 bytes),
        // copied verbatim.
        let mut data_ok = (q != 0) as i32;
        if q != 0 {
            for i in 0..248usize {
                if *((q + i) as *const u8) != (((i + 8) as u8) ^ 0xA5) {
                    data_ok = 0;
                }
            }
        }
        set_release(None);
        libc::free(q as *mut c_void);
        retire(seg as *mut c_void);
        libc::free(seg as *mut c_void);
        (calls, off, non_null, usable_ok, data_ok)
    };
    println!("reallocarray_release_calls={calls}");
    println!("reallocarray_non_null={non_null}");
    println!("reallocarray_moved_off_segment={off_seg}");
    println!("reallocarray_usable_ge_request={usable_ok}");
    println!("reallocarray_data_ok={data_ok}");
}

/// A `realloc` of a pointer INTO a registered segment moves the bytes to the
/// real heap (copied UNDER the registry lock), releases the sample once, and
/// unregisters the range.
fn probe_segment_realloc() {
    let register: RegisterFn = need(c"cerulion_heaphook_register_segment");
    let set_release: SetReleaseFn = need(c"cerulion_heaphook_set_release_callback");
    let retire: RetireFn = need(c"cerulion_heaphook_retire_slot");

    let seg = alloc_slot(256);
    let interior = seg as usize + 8;
    let cookie = 0xBEEF_usize;

    // SAFETY: fill the segment, register it, then realloc an interior pointer.
    let (data_ok, calls, ptr_match, cookie_match, off_seg) = unsafe {
        for i in 0..256usize {
            *(seg.add(i)) = (i as u8) ^ 0x33;
        }
        set_release(Some(on_release));
        register(seg as *mut c_void, 256, cookie);
        let q = libc::realloc(interior as *mut c_void, 512) as *mut u8; // moves off-segment
                                                                        // The bytes from the interior pointer (248 of them) survived the move.
        let mut ok = 1i32;
        for i in 0..248usize {
            if *q.add(i) != (((i + 8) as u8) ^ 0x33) {
                ok = 0;
            }
        }
        let off = (((q as usize) < seg as usize) || ((q as usize) >= seg as usize + 256)) as i32;
        let calls = RELEASE_CALLS.load(Ordering::SeqCst);
        let pm = (RELEASED_PTR.load(Ordering::SeqCst) == interior) as i32;
        let cm = (RELEASED_COOKIE.load(Ordering::SeqCst) == cookie) as i32;
        libc::free(q as *mut c_void); // q is real heap
        set_release(None);
        retire(seg as *mut c_void); // retire TOMBSTONES; the free below is a no-op (leaks — fixture-scoped)
        libc::free(seg as *mut c_void);
        (ok, calls, pm, cm, off)
    };
    println!("realloc_data_ok={data_ok}");
    println!("release_calls={calls}");
    println!("released_ptr_match={ptr_match}");
    println!("released_cookie_match={cookie_match}");
    println!("moved_off_segment={off_seg}");
}

/// Stress the re-entrancy guard: MANY bumps grow the window's ledger `Vec`, and
/// each growth is a re-entrant real `malloc` the guard must route past our logic
/// — without it this recurses forever. All bumps land in a large slot.
fn probe_guard_stress() {
    let arm: ArmFn = need(c"cerulion_heaphook_arm_window");
    let disarm: DisarmFn = need(c"cerulion_heaphook_disarm_window");
    let retire: RetireFn = need(c"cerulion_heaphook_retire_slot");

    let slot_len = 1 << 16; // 64 KiB
    let slot = alloc_slot(slot_len);
    let base = slot as usize;
    let tail = base + slot_len;
    const N: usize = 1000;

    // SAFETY: resolved exports; live slot. 1000 * (16 aligned) < 64 KiB fits.
    let (in_slot, disarm_rc) = unsafe {
        arm(base as *mut c_void, tail as *mut c_void);
        let mut in_slot = 0usize;
        for _ in 0..N {
            let p = libc::malloc(16) as usize;
            if p >= base && p < tail {
                in_slot += 1;
            }
        }
        let disarm_rc = disarm();
        (in_slot, disarm_rc)
    };
    // Slot is quarantined from arm → retire before the real free.
    // SAFETY: after retire, `slot` classifies RealHeap.
    unsafe {
        retire(base as *mut c_void);
        libc::free(slot as *mut c_void);
    }

    println!("stress_total={N}");
    println!("stress_in_slot={in_slot}");
    println!("disarm_rc={disarm_rc}");
}

/// An INCIDENTAL in-slot allocation freed AFTER disarm must be a no-op (routed
/// through the quarantine), NEVER glibc `free` of a shared-memory address (which
/// aborts). The probe simply has to SURVIVE: a mis-routed free crashes it.
fn probe_quarantine() {
    let arm: ArmFn = need(c"cerulion_heaphook_arm_window");
    let disarm: DisarmFn = need(c"cerulion_heaphook_disarm_window");
    let retire: RetireFn = need(c"cerulion_heaphook_retire_slot");

    let slot_len = 4096usize;
    let slot = alloc_slot(slot_len);
    let base = slot as usize;
    let tail = base + slot_len;

    // SAFETY: resolved exports; live slot.
    let (in_slot, retire_rc) = unsafe {
        arm(base as *mut c_void, tail as *mut c_void);
        let p = libc::malloc(64) as usize; // an incidental in-slot allocation
        let in_slot = (p >= base && p < tail) as i32;
        disarm(); // the slot extent has been quarantined since ARM time
        libc::free(p as *mut c_void); // MUST be a no-op, not glibc free of SHM
        let retire_rc = retire(base as *mut c_void); // retire TOMBSTONES the extent (RC_OK)
                                                     // The slot stays covered; the free below is a tombstone no-op (leaks — fixture-scoped).
        libc::free(slot as *mut c_void);
        (in_slot, retire_rc)
    };
    println!("in_slot={in_slot}");
    println!("retire_rc={retire_rc}"); // 0 = a quarantine extent was retired
    println!("survived=1"); // reached here ⇒ the in-slot free did not abort
}

/// DISCLOSURE regression: a `realloc` of a pointer into a
/// DISARMED slot must fail SAFELY (null) rather than copy `tail_limit - addr`
/// bytes — which would splice a LATER allocation (or unrelated slot bytes) into
/// the caller's buffer. Arm a slot, make TWO in-window allocations A then B with
/// distinct fills, disarm (drops the per-allocation ledger; the whole slot stays
/// quarantined from arm), then `realloc(A)`. The scenario is arranged so a naive
/// `tail_limit - A` copy WOULD span B — so a null return is the disclosure proof,
/// and A stays valid (the C contract for a failed realloc).
#[cfg(all(target_os = "linux", target_env = "gnu"))]
fn probe_disarmed_realloc() {
    let arm: ArmFn = need(c"cerulion_heaphook_arm_window");
    let disarm: DisarmFn = need(c"cerulion_heaphook_disarm_window");
    let retire: RetireFn = need(c"cerulion_heaphook_retire_slot");

    let slot_len = 4096usize;
    let slot = alloc_slot(slot_len);
    let base = slot as usize;
    let tail = base + slot_len;

    const A_SECRET: u8 = 0xAA;
    const B_SECRET: u8 = 0xBB;

    // SAFETY: resolved exports; live slot. No Rust allocation in the armed span.
    let (a_in_slot, b_in_slot, b_after_a, span_covers_b, realloc_null, a_intact, errno_enomem) = unsafe {
        arm(base as *mut c_void, tail as *mut c_void);
        let a = libc::malloc(64) as *mut u8;
        for i in 0..64usize {
            *a.add(i) = A_SECRET;
        }
        let b = libc::malloc(64) as *mut u8;
        for i in 0..64usize {
            *b.add(i) = B_SECRET; // the "secret" a tail-copy must never disclose
        }
        let (av, bv) = (a as usize, b as usize);
        let a_in = (av >= base && av < tail) as i32;
        let b_in = (bv >= base && bv < tail) as i32;
        let b_after = (bv > av) as i32;
        // A naive copy of `tail - A` bytes reaches through B entirely.
        let covers = ((tail - av) >= (bv - av) + 64) as i32;

        disarm(); // drops A's/B's ledger extents; slot stays quarantined
        *libc::__errno_location() = 0;
        let q = libc::realloc(a as *mut c_void, 128); // quarantined ptr → fail-safe
        let q_null = q.is_null() as i32;
        let errno_is_enomem = (*libc::__errno_location() == libc::ENOMEM) as i32;
        // The failed realloc left A valid AND unchanged (the C realloc contract).
        let mut intact = 1i32;
        for i in 0..64usize {
            if *a.add(i) != A_SECRET {
                intact = 0;
            }
        }
        // Retire the slot (quarantined from arm), then the real buffer frees.
        retire(base as *mut c_void);
        libc::free(slot as *mut c_void);
        (a_in, b_in, b_after, covers, q_null, intact, errno_is_enomem)
    };
    println!("a_in_slot={a_in_slot}");
    println!("b_in_slot={b_in_slot}");
    println!("b_after_a={b_after_a}");
    println!("span_covers_b={span_covers_b}");
    println!("realloc_null={realloc_null}");
    println!("a_intact={a_intact}");
    println!("errno_enomem={errno_enomem}");
}

/// A `realloc` of a window pointer is an escape (a vector grew past its bump):
/// the bytes move to the real heap, the window latches Reallocated (code 2), and
/// the old data is preserved.
fn probe_realloc_escape() {
    let arm: ArmFn = need(c"cerulion_heaphook_arm_window");
    let disarm: DisarmFn = need(c"cerulion_heaphook_disarm_window");
    let escape: EscapeFn = need(c"cerulion_heaphook_window_escape");
    let retire: RetireFn = need(c"cerulion_heaphook_retire_slot");

    let slot_len = 4096usize;
    let slot = alloc_slot(slot_len);
    let base = slot as usize;
    let tail = base + slot_len;

    // SAFETY: resolved exports; live slot.
    let (data_ok, esc, moved_off_slot, disarm_rc) = unsafe {
        arm(base as *mut c_void, tail as *mut c_void);
        let p = libc::malloc(64) as *mut u8;
        for i in 0..64u8 {
            *p.add(i as usize) = i ^ 0x5a;
        }
        let q = libc::realloc(p as *mut c_void, 128) as *mut u8; // escape: moves off-slot
        let esc = escape(); // 2 = Reallocated
                            // The first 64 bytes survived the move.
        let mut ok = 1i32;
        for i in 0..64u8 {
            if *q.add(i as usize) != (i ^ 0x5a) {
                ok = 0;
            }
        }
        let moved = ((q as usize) < base || (q as usize) >= tail) as i32;
        let disarm_rc = disarm();
        libc::free(q as *mut c_void); // q is real heap (moved off) → real free
        retire(base as *mut c_void); // retire TOMBSTONES; the free below is a no-op (slot leaks — fixture-scoped)
        libc::free(slot as *mut c_void);
        (ok, esc, moved, disarm_rc)
    };
    println!("realloc_data_ok={data_ok}");
    println!("escape={esc}");
    println!("moved_off_slot={moved_off_slot}");
    println!("disarm_rc={disarm_rc}");
}

/// `calloc` into an armed window must ZERO the slot bytes even when the slot
/// arrived dirty (a loan slot is uninitialized).
fn probe_calloc_zero() {
    let arm: ArmFn = need(c"cerulion_heaphook_arm_window");
    let disarm: DisarmFn = need(c"cerulion_heaphook_disarm_window");
    let retire: RetireFn = need(c"cerulion_heaphook_retire_slot");

    let slot_len = 4096usize;
    let slot = alloc_slot(slot_len);
    let base = slot as usize;
    let tail = base + slot_len;

    // SAFETY: dirty the slot, then calloc into it and check zeros.
    let (all_zero, in_slot) = unsafe {
        for i in 0..slot_len {
            *(slot.add(i)) = 0xFF;
        }
        arm(base as *mut c_void, tail as *mut c_void);
        let c = libc::calloc(1, 64) as *mut u8;
        let in_slot = ((c as usize) >= base && (c as usize) < tail) as i32;
        let mut z = 1i32;
        for i in 0..64usize {
            if *c.add(i) != 0 {
                z = 0;
            }
        }
        disarm();
        retire(base as *mut c_void); // retire TOMBSTONES; the free below is a no-op (slot leaks — fixture-scoped)
        libc::free(slot as *mut c_void);
        (z, in_slot)
    };
    println!("in_slot={in_slot}");
    println!("all_zero={all_zero}");
}

/// A registered range freed with NO release callback set is LEAKED, not wrongly
/// freed — and the leak is COUNTED (Principle #3 observability).
fn probe_counter() {
    let register: RegisterFn = need(c"cerulion_heaphook_register_segment");
    let set_release: SetReleaseFn = need(c"cerulion_heaphook_set_release_callback");
    let counter: CounterFn = need(c"cerulion_heaphook_counter");
    let retire: RetireFn = need(c"cerulion_heaphook_retire_slot");

    let seg = alloc_slot(256);
    // SAFETY: no callback set → a free of the range counts a leak and folds the
    // range into the quarantine (index 0 = release-without-callback).
    let (before, after) = unsafe {
        set_release(None);
        let before = counter(0);
        register(seg as *mut c_void, 256, 7);
        libc::free((seg as usize + 8) as *mut c_void); // release (no cb) → counted + quarantined
        let after = counter(0);
        retire(seg as *mut c_void); // TOMBSTONES the hand-off entry the release parked
        libc::free(seg as *mut c_void); // tombstone no-op now (leaks — fixture-scoped)
        (before, after)
    };
    println!("counter_increased={}", (after > before) as i32);
}
