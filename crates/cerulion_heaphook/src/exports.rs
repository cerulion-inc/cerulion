// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion_heaphook_*` C exports (Linux/GNU): the versioned handshake
//! vtable the rmw `dlsym`s at init, the borrow-window + segment-registry
//! control API, and the load constructor (real-symbol resolution, won-malloc
//! detection, `atfork` registration, the `debug!`-once breadcrumb).
//!
//! # The rmw borrow consumer API
//!
//! At init: `dlsym(RTLD_DEFAULT, "cerulion_heaphook_abi")`, call it, feed the
//! `version` + `status()` through [`crate::abi::decide_handshake`]. If
//! [`crate::abi::HandshakeVerdict::Active`], drive publish-side zero-copy:
//!
//! 1. obtain a loan slot; `arm_window(base, tail_limit)` over its tail on the
//!    filling thread (nested arm ⇒ [`crate::abi::RC_ERR_ALREADY_ARMED`]);
//! 2. let the stock node fill the message (`resize`/`assign` bump into the
//!    slot);
//! 3. before publish, for each variable field call `window_range_test(ptr, len)`
//!    to confirm it landed in the slot, and read `window_escape()`; a non-zero
//!    escape (or a failed range test) ⇒ copy that field the old way, loudly;
//! 4. `disarm_window()` — the slot's WHOLE `[base, tail_limit)` extent was
//!    already quarantined AT ARM TIME (coverage before exposure; disarm folds
//!    nothing further), so a later `free` of any in-slot
//!    pointer (the adopted field storage, OR an incidental buffer the fill
//!    allocated) is a NO-OP, never glibc `free` of a shared-memory address.
//!    When the backing sample is finally reclaimed, `retire_slot(base)`
//!    TOMBSTONES that quarantine extent (see the RETIRE CONTRACT below).
//!
//! The quarantine is what makes step 4 sound WITHOUT the consumer having to
//! account for every allocation the stock fill made: an armed window bumps
//! EVERY allocation on its thread into the slot, and any of those pointers may
//! be freed after disarm or on another thread. Do NOT `free`/`realloc` an
//! in-slot pointer as if it were private heap; retire the slot instead.
//!
//! Retire contract — `retire_slot` is tombstone-on-retire (by
//! design). Retiring marks the extent dead-but-remembered, it does not delete
//! the coverage: an INCIDENTAL in-slot allocation can outlive the sample (stock
//! user code holds it arbitrarily long), and a `free` of such a pointer AFTER
//! retire stays a counted NO-OP (diagnostic counter kind 4, tombstone hits)
//! forever — never glibc. So the consumer retires FREELY the moment the sample
//! is reclaimed; it need not track incidental-allocation lifetimes. Bounded:
//! when the pool RECYCLES a slot address, the next `arm_window` at that base
//! REVIVES the tombstone (one entry per distinct slot address, not per
//! message). This is the same "coverage is never forgotten" guarantee a
//! fixed-VA-partition range test gives, without reserving virtual address
//! space or a kernel module — Jetson-viable. (Consumer note for the take-side
//! `register_segment` path: a `free` of a REGISTERED range still removes the
//! registration and fires the release callback — tombstoning is the
//! QUARANTINE/arm-window path only.)
//!
//! Take side (the residual): `register_segment(sample_bytes, len,
//! sample_handle)` for a forged vector's storage and `set_release_callback` once
//! at init — a foreign `free`/`realloc` of that range is routed to the callback
//! (and the range is removed atomically), never to glibc. The release callback
//! runs WITHOUT the hook's re-entrancy guard held, so it may itself allocate;
//! its `free` of ANOTHER registered range routes to release, not glibc.
//!
//! # Registration contract (segment vs quarantine)
//!
//! Register a segment as the EXACT byte range of the take-side allocation being
//! adopted (a per-field sub-range), NOT the whole enclosing sample. Classify
//! ranks a registered Segment ABOVE a Quarantine extent covering the same bytes,
//! so if a publish slot is BOTH quarantined (from `arm_window`) and registered as
//! a whole sample, the first in-slot free would release the sample prematurely
//! while adopted storage is still subscriber-visible. In practice take-side
//! samples and publish-side loan slots are distinct memory; keeping segment
//! registrations to exact adopted sub-ranges keeps the two planes disjoint.
//!
//! # Observability (depends on the rmw consumer)
//!
//! The diagnostic counters (`counter`) have no in-process reader until the
//! rmw borrow consumer reads them; a stock ROS 2 process installs no `tracing`
//! subscriber. So each failure path ALSO writes a one-line, allocation-free
//! stderr breadcrumb the first time it fires, gated on `CERULION_HEAPHOOK_DEBUG`
//! (latched, never a flood), and the won-malloc DEGRADE writes an ALWAYS-ON line
//! — because the launcher auto-injects this hook, and a process carrying a
//! foreign allocator must not degrade silently.

use core::ffi::{c_char, c_int, c_void};
use core::sync::atomic::{AtomicBool, Ordering};

use crate::abi::{
    HeaphookAbi, ReleaseCallback, HEAPHOOK_ABI_VERSION, HEAPHOOK_STATUS_WON_MALLOC,
    RC_ERR_ALREADY_ARMED, RC_ERR_BAD_ARG, RC_ERR_NOT_ARMED, RC_ERR_OVERLAP, RC_ERR_UNAVAILABLE,
    RC_ERR_UNKNOWN, RC_OK,
};
use crate::registry::RegisterError;
use crate::state;
use crate::state::ArmOutcome;

// ── the handshake vtable ────────────────────────────────────────────────────

static ABI: HeaphookAbi = HeaphookAbi {
    version: HEAPHOOK_ABI_VERSION,
    status: cerulion_heaphook_status,
    arm_window: cerulion_heaphook_arm_window,
    disarm_window: cerulion_heaphook_disarm_window,
    window_escape: cerulion_heaphook_window_escape,
    window_range_test: cerulion_heaphook_window_range_test,
    register_segment: cerulion_heaphook_register_segment,
    unregister_segment: cerulion_heaphook_unregister_segment,
    set_release_callback: cerulion_heaphook_set_release_callback,
    retire_slot: cerulion_heaphook_retire_slot,
    counter: cerulion_heaphook_counter,
};

/// The ONE symbol the consumer `dlsym`s: returns the versioned vtable.
///
/// # Safety
/// Returns a pointer to a `'static` vtable; always valid to read.
#[no_mangle]
pub unsafe extern "C" fn cerulion_heaphook_abi() -> *const HeaphookAbi {
    &ABI
}

/// The ABI version (the vtable's `version`, exported standalone so a consumer
/// can handshake with two `dlsym`s — `version` + `status` — and never needs the
/// vtable struct layout).
///
/// # Safety
/// Returns a constant; always sound.
#[no_mangle]
pub unsafe extern "C" fn cerulion_heaphook_version() -> u32 {
    HEAPHOOK_ABI_VERSION
}

/// The load-time status bitset (see [`HEAPHOOK_STATUS_WON_MALLOC`]).
///
/// # Safety
/// Reads an atomic; always sound.
#[no_mangle]
pub unsafe extern "C" fn cerulion_heaphook_status() -> u32 {
    state::status()
}

/// Arm a thread-local borrow window over `[base, tail_limit)`.
///
/// # Safety
/// `base`/`tail_limit` describe a loan slot the caller owns for the window's
/// lifetime; only their addresses are used.
#[no_mangle]
pub unsafe extern "C" fn cerulion_heaphook_arm_window(
    base: *mut c_void,
    tail_limit: *mut c_void,
) -> c_int {
    // Validate like `register_segment`. A NULL base
    // would arm a window over `[0, len)` — its arm-time quarantine insert then
    // classifies near-NULL addresses as no-op frees PROCESS-WIDE, and the next
    // malloc on this thread would hand back a near-NULL "in-slot" pointer. A
    // tail BELOW base is a reversed/overflowed ask — genuine garbage, refused
    // loudly. `tail_limit == base` is NOT garbage: it is the
    // documented ZERO-CAPACITY window the rmw's minimum slice ceiling mints
    // (`windowed_borrow_geometry` deliberately admits `payload_len ==
    // tail_off`: "a zero-capacity window is sound — every fill escapes ⇒
    // copies"), and every layer below already serves it — `state::arm_window`
    // skips the quarantine insert for the empty extent (nothing can point
    // into `[base, base)`), `BorrowWindow::bump` reserves at least one byte
    // so EVERY fill allocation (a zero-size request included) overflows to
    // the real allocator with GrowthPastTail latched, and the consumer's
    // cursor bisection recovers `cursor == base`. A `<=` refusal here
    // would make the caller degrade WINDOWLESS on every borrow of a min-ceiling
    // topic — a spurious per-borrow WindowUnavailable where the documented
    // behavior is an armed window whose bumps safely escape.
    if base.is_null() || (tail_limit as usize) < (base as usize) {
        return RC_ERR_BAD_ARG;
    }
    guarded(
        || match state::arm_window(base as usize, tail_limit as usize) {
            ArmOutcome::Armed => RC_OK,
            ArmOutcome::AlreadyArmed => RC_ERR_ALREADY_ARMED,
            ArmOutcome::Unavailable => RC_ERR_UNAVAILABLE,
        },
    )
}

/// Disarm this thread's window; returns the latched escape code (0 = none) or
/// [`RC_ERR_NOT_ARMED`].
///
/// # Safety
/// Sound to call at any time.
#[no_mangle]
pub unsafe extern "C" fn cerulion_heaphook_disarm_window() -> c_int {
    guarded(|| match state::disarm_window() {
        None => RC_ERR_NOT_ARMED,
        Some(escape) => escape.map(|e| e.as_code()).unwrap_or(0),
    })
}

/// The armed window's latched escape code (0 = none) or [`RC_ERR_NOT_ARMED`].
///
/// # Safety
/// Sound to call at any time.
#[no_mangle]
pub unsafe extern "C" fn cerulion_heaphook_window_escape() -> c_int {
    guarded(|| {
        state::with_window(
            |w| w.escape().map(|e| e.as_code()).unwrap_or(0),
            || RC_ERR_NOT_ARMED,
        )
    })
}

/// 1 iff `[ptr, ptr + len)` landed fully inside the armed window's bumped
/// region (the adopt test); 0 if not; [`RC_ERR_NOT_ARMED`] with no window.
///
/// # Safety
/// Only the address `ptr` is read, never dereferenced.
#[no_mangle]
pub unsafe extern "C" fn cerulion_heaphook_window_range_test(
    ptr: *const c_void,
    len: usize,
) -> c_int {
    guarded(|| {
        state::with_window(
            |w| {
                if w.range_adopted(ptr as usize, len) {
                    1
                } else {
                    0
                }
            },
            || RC_ERR_NOT_ARMED,
        )
    })
}

/// Register a shared-memory range `[start, start + len)` with an opaque
/// `cookie` (a sample/slot handle) for the release callback.
///
/// # Safety
/// `start`/`len` describe a range the caller keeps alive until unregister; only
/// the address is used.
#[no_mangle]
pub unsafe extern "C" fn cerulion_heaphook_register_segment(
    start: *mut c_void,
    len: usize,
    cookie: usize,
) -> c_int {
    if start.is_null() {
        return RC_ERR_BAD_ARG;
    }
    guarded(|| {
        // The atfork-child-fold reserve happens INSIDE the same lock scope as
        // the registration: the capacity invariant must
        // hold at every fork-possible instant, so there is no post-register
        // re-reserve window here (guard held → the reserve bypasses the locks).
        match state::register_segment_reserving(start as usize, len, cookie) {
            Ok(()) => RC_OK,
            Err(RegisterError::ZeroLength) => RC_ERR_BAD_ARG,
            Err(RegisterError::Overlaps(_)) => RC_ERR_OVERLAP,
        }
    })
}

/// Unregister the range starting at `start`.
///
/// # Safety
/// Only the address is used.
#[no_mangle]
pub unsafe extern "C" fn cerulion_heaphook_unregister_segment(start: *mut c_void) -> c_int {
    guarded(|| {
        state::with_registry(|reg| {
            if reg.unregister(start as usize) {
                RC_OK
            } else {
                RC_ERR_UNKNOWN
            }
        })
    })
}

/// Set (or clear with a null pointer) the release callback.
///
/// # Safety
/// `cb`, if non-null, must be a valid `extern "C"` function for the range's
/// lifetime.
#[no_mangle]
pub unsafe extern "C" fn cerulion_heaphook_set_release_callback(cb: ReleaseCallback) -> c_int {
    guarded(|| {
        state::set_release_callback(cb);
        RC_OK
    })
}

/// Retire a disarmed slot's quarantine extent by its `base` address — the
/// consumer calls this once the backing sample is reclaimed. TOMBSTONE
/// on-retire: the extent is marked dead-but-remembered (a late in-slot
/// free stays a counted no-op), not deleted; a re-arm at the same base revives
/// it. Returns [`RC_OK`] if an entry started at `base`, [`RC_ERR_UNKNOWN`]
/// otherwise. See the RETIRE CONTRACT in this module's header.
///
/// # Safety
/// Only the address is used.
#[no_mangle]
pub unsafe extern "C" fn cerulion_heaphook_retire_slot(base: *mut c_void) -> c_int {
    guarded(|| {
        if state::retire_quarantine(base as usize) {
            RC_OK
        } else {
            RC_ERR_UNKNOWN
        }
    })
}

/// Read a diagnostic counter by index (see [`HeaphookAbi::counter`]).
///
/// # Safety
/// Reads an atomic; always sound.
#[no_mangle]
pub unsafe extern "C" fn cerulion_heaphook_counter(kind: u32) -> u64 {
    state::counter(kind)
}

/// Run a control-API body with the re-entrancy guard held, so any bookkeeping
/// allocation it triggers (a registry `Vec` growth) bypasses to the real
/// allocator instead of bumping into an armed window.
fn guarded(f: impl FnOnce() -> c_int) -> c_int {
    state::with_control_guard(f)
}

// ── load constructor ────────────────────────────────────────────────────────

/// The ELF `.init_array` constructor — runs at load, before `main`.
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
static HEAPHOOK_CTOR: extern "C" fn() = heaphook_ctor;

extern "C" fn heaphook_ctor() {
    on_load();
}

fn on_load() {
    state::ensure_resolved();

    // Cache the debug flag once (failure-path breadcrumbs read it, not getenv).
    let debug = env_truthy(c"CERULION_HEAPHOOK_DEBUG");
    state::set_debug_enabled(debug);

    let won = won_malloc();
    // set_status emits the ALWAYS-ON won-malloc degrade line when !won.
    state::set_status(if won { HEAPHOOK_STATUS_WON_MALLOC } else { 0 });

    // atfork: keep the registry lock consistent and reset the child.
    // SAFETY: standard pthread_atfork registration with static handlers.
    let atfork_rc = unsafe {
        libc::pthread_atfork(
            Some(atfork_prepare),
            Some(atfork_parent),
            Some(atfork_child),
        )
    };
    if atfork_rc != 0 {
        // ENOMEM: children fork WITHOUT our lock/quarantine reset — a real,
        // if rare, degrade. Say so once (always-on, like the malloc degrade).
        let msg =
            b"cerulion heap hook: pthread_atfork registration failed; fork children unprotected\n";
        // SAFETY: allocation-free write of a fixed slice to stderr.
        unsafe {
            libc::write(2, msg.as_ptr() as *const c_void, msg.len());
        }
    }

    breadcrumb(won, debug);
}

extern "C" fn atfork_prepare() {
    state::atfork_lock();
}
extern "C" fn atfork_parent() {
    state::atfork_unlock();
}
extern "C" fn atfork_child() {
    state::atfork_child_reset();
}

/// Did this library resolve as the process's `malloc` (first-in-load-order)?
///
/// It compares the MODULE the process-resolved `malloc` lives in against OUR
/// module. An ADDRESS compare against our own `malloc` symbol would be WRONG:
/// taking the address of an EXPORTED symbol goes through the GOT and is itself
/// preemptible, so under `LD_PRELOAD=libjemalloc.so:libcerulion_heaphook.so`
/// BOTH our reference and the resolved symbol would be jemalloc's `malloc` and
/// compare EQUAL — reporting "won" for exactly the foreign-allocator case the
/// status bit exists to catch. Module identity via `dladdr` is preemption-proof:
/// `heaphook_ctor` is a PRIVATE (non-exported, `dso_local`) symbol, so `dladdr`
/// on its address always yields OUR module's base.
fn won_malloc() -> bool {
    // SAFETY: RTLD_DEFAULT + a valid C string is defined on glibc.
    let resolved = unsafe { libc::dlsym(libc::RTLD_DEFAULT, c"malloc".as_ptr() as *const c_char) };
    if resolved.is_null() {
        return false;
    }
    let mut ours: libc::Dl_info = unsafe { core::mem::zeroed() };
    let mut theirs: libc::Dl_info = unsafe { core::mem::zeroed() };
    let ours_addr = heaphook_ctor as *const () as *const c_void;
    // SAFETY: dladdr with a valid code address and a Dl_info out-pointer.
    let ok_ours = unsafe { libc::dladdr(ours_addr, &mut ours) };
    let ok_theirs = unsafe { libc::dladdr(resolved, &mut theirs) };
    ok_ours != 0 && ok_theirs != 0 && ours.dli_fbase == theirs.dli_fbase
}

/// The `debug!`-once load breadcrumb naming the ABI version. Also written raw to
/// stderr (allocation-free) when `CERULION_HEAPHOOK_DEBUG` is truthy — a stock
/// ROS 2 process installs no `tracing` subscriber, so the raw line is the only
/// channel the container tests can observe the load through.
fn breadcrumb(won: bool, debug: bool) {
    static ONCE: AtomicBool = AtomicBool::new(false);
    if ONCE.swap(true, Ordering::AcqRel) {
        return;
    }
    tracing::debug!(
        abi_version = HEAPHOOK_ABI_VERSION,
        won_malloc = won,
        "cerulion heap hook loaded"
    );
    if debug {
        raw_breadcrumb(HEAPHOOK_ABI_VERSION, won);
    }
}

/// True iff the environment variable `name` is set to a non-empty value other
/// than the EXACT string `"0"`. So `"0debug"` is truthy while `"0"` and `""`
/// are not (matching only `"0"` by its first byte would falsely disable on any
/// value that merely starts with `0`). Allocation-free (`getenv`, no `std::env`).
fn env_truthy(name: &core::ffi::CStr) -> bool {
    // SAFETY: getenv with a valid C string returns a pointer into environ.
    let p = unsafe { libc::getenv(name.as_ptr() as *const c_char) };
    if p.is_null() {
        return false;
    }
    // SAFETY: getenv's result points at a NUL-terminated C string.
    let first = unsafe { *p } as u8;
    if first == 0 {
        return false; // empty
    }
    // The EXACT string "0" (a '0' followed immediately by NUL) is false.
    // SAFETY: `first` was non-NUL, so the byte after it is still within the
    // NUL-terminated string (at worst the terminator itself).
    if first == b'0' && unsafe { *p.add(1) } == 0 {
        return false;
    }
    true
}

/// Write `"cerulion heap hook loaded: abi v<N> won_malloc=<0|1>\n"` to fd 2 with
/// no heap allocation (a fixed stack buffer + one `write(2)`).
fn raw_breadcrumb(version: u32, won: bool) {
    let mut buf = [0u8; 96];
    let mut n = 0usize;
    n += append(&mut buf, n, b"cerulion heap hook loaded: abi v");
    n += write_u32_dec(&mut buf[n..], version);
    n += append(&mut buf, n, b" won_malloc=");
    if n < buf.len() {
        buf[n] = if won { b'1' } else { b'0' };
        n += 1;
    }
    if n < buf.len() {
        buf[n] = b'\n';
        n += 1;
    }
    // SAFETY: writing n valid bytes of a stack buffer to stderr.
    unsafe {
        libc::write(2, buf.as_ptr() as *const c_void, n);
    }
}

/// Append `src` into `buf` at `at`, returning the number of bytes written
/// (clamped to the remaining capacity).
fn append(buf: &mut [u8; 96], at: usize, src: &[u8]) -> usize {
    let mut i = 0;
    while i < src.len() && at + i < buf.len() {
        buf[at + i] = src[i];
        i += 1;
    }
    i
}

/// Write `val` in decimal into `out`, returning the byte count.
fn write_u32_dec(out: &mut [u8], val: u32) -> usize {
    if out.is_empty() {
        return 0;
    }
    if val == 0 {
        out[0] = b'0';
        return 1;
    }
    let mut tmp = [0u8; 10];
    let mut v = val;
    let mut len = 0;
    while v > 0 {
        tmp[len] = b'0' + (v % 10) as u8;
        v /= 10;
        len += 1;
    }
    let mut n = 0;
    while n < len && n < out.len() {
        out[n] = tmp[len - 1 - n];
        n += 1;
    }
    n
}

#[cfg(test)]
mod tests {
    use super::*;

    // The zero-capacity boundary: `base == tail_limit` is
    // the documented ZERO-CAPACITY window — the rmw's minimum-slice-ceiling
    // shape, `windowed_borrow_geometry` deliberately admitting `payload_len
    // == tail_off` — and must ARM: a `<=` refusal would make the caller
    // degrade windowless (a spurious per-borrow WindowUnavailable) on every
    // borrow of a min-ceiling topic. One past the boundary (reversed bounds)
    // and NULL stay refused — the genuine garbage classes. Reverting
    // the comparison to `<=` fails the first assert with RC_ERR_BAD_ARG
    // where RC_OK is required.
    #[test]
    fn a_zero_capacity_window_arms_while_reversed_and_null_stay_refused() {
        // Read side of the global-tables guard: the quarantine
        // classify below reads the process-global table.
        let _t = state::GLOBAL_TABLES_TEST_LOCK
            .read()
            .unwrap_or_else(|e| e.into_inner());
        let slot = [0u8; 8];
        let base = slot.as_ptr() as *mut c_void;
        // SAFETY: addresses are only stored/compared, never dereferenced —
        // and a zero-capacity window can never mint an in-slot pointer (the
        // bump's at-least-one-byte reserve overflows every request).
        unsafe {
            assert_eq!(
                cerulion_heaphook_arm_window(base, base),
                RC_OK,
                "base == tail_limit is the documented zero-capacity window"
            );
            // Genuinely ARMED, not silently ignored: a nested arm refuses.
            assert_eq!(
                cerulion_heaphook_arm_window(base, base),
                RC_ERR_ALREADY_ARMED,
                "the zero-capacity window is really armed on this thread"
            );
            // The empty extent quarantined NOTHING (state::arm_window's
            // `tail_limit > base` gate) — no process-global entry to purge.
            assert!(
                state::with_quarantine(|q| q.classify(base as usize).is_none()),
                "an empty extent inserts no quarantine coverage"
            );
            // Cleanup disarm. 0 (no escape) on a quiet thread; a positive
            // escape code is also acceptable — any allocation this thread
            // made mid-window legitimately overflowed and latched — but a
            // negative rc would mean no window was armed at all.
            let rc = cerulion_heaphook_disarm_window();
            assert!(rc >= 0, "disarm found the armed window (got {rc})");
            // One past the boundary: reversed bounds stay refused…
            assert_eq!(
                cerulion_heaphook_arm_window((base as usize + 1) as *mut c_void, base),
                RC_ERR_BAD_ARG
            );
            // …and so does NULL.
            assert_eq!(
                cerulion_heaphook_arm_window(core::ptr::null_mut(), base),
                RC_ERR_BAD_ARG
            );
        }
    }

    #[test]
    fn env_truthy_matches_only_the_exact_string_zero_as_false() {
        // A dedicated var name so this cannot collide with a real setting.
        let name = c"CER_HEAPHOOK_ENV_TRUTHY_TEST";
        let key = "CER_HEAPHOOK_ENV_TRUTHY_TEST";
        std::env::remove_var(key);
        assert!(!env_truthy(name), "unset is false");
        std::env::set_var(key, "");
        assert!(!env_truthy(name), "empty is false");
        std::env::set_var(key, "0");
        assert!(!env_truthy(name), "the exact string 0 is false");
        std::env::set_var(key, "0debug");
        assert!(
            env_truthy(name),
            "0debug is truthy (not the exact string 0)"
        );
        std::env::set_var(key, "00");
        assert!(env_truthy(name), "00 is truthy");
        std::env::set_var(key, "1");
        assert!(env_truthy(name));
        std::env::remove_var(key);
    }

    /// Every vtable slot must point at the export it is named for. A named-field
    /// struct literal is type-checked, but a FIELD REORDER (a C consumer reading
    /// by offset) type-checks and passes the size/align pin in `abi.rs` — so pin
    /// each slot's function IDENTITY through the `cerulion_heaphook_abi` entry the
    /// consumer actually `dlsym`s.
    #[test]
    fn the_vtable_slots_point_at_their_named_exports() {
        // SAFETY: cerulion_heaphook_abi returns a 'static vtable.
        let abi = unsafe { &*cerulion_heaphook_abi() };
        assert_eq!(abi.version, HEAPHOOK_ABI_VERSION);
        // Compare each slot (a fn pointer) to its named export (a fn item) as
        // thin raw pointers — a FIELD REORDER points a slot at the wrong fn.
        assert_eq!(
            abi.status as *const (),
            cerulion_heaphook_status as *const ()
        );
        assert_eq!(
            abi.arm_window as *const (),
            cerulion_heaphook_arm_window as *const ()
        );
        assert_eq!(
            abi.disarm_window as *const (),
            cerulion_heaphook_disarm_window as *const ()
        );
        assert_eq!(
            abi.window_escape as *const (),
            cerulion_heaphook_window_escape as *const ()
        );
        assert_eq!(
            abi.window_range_test as *const (),
            cerulion_heaphook_window_range_test as *const ()
        );
        assert_eq!(
            abi.register_segment as *const (),
            cerulion_heaphook_register_segment as *const ()
        );
        assert_eq!(
            abi.unregister_segment as *const (),
            cerulion_heaphook_unregister_segment as *const ()
        );
        assert_eq!(
            abi.set_release_callback as *const (),
            cerulion_heaphook_set_release_callback as *const ()
        );
        assert_eq!(
            abi.retire_slot as *const (),
            cerulion_heaphook_retire_slot as *const ()
        );
        assert_eq!(
            abi.counter as *const (),
            cerulion_heaphook_counter as *const ()
        );
    }
}
