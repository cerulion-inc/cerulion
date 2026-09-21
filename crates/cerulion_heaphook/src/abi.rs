// SPDX-License-Identifier: AGPL-3.0-only
//! The versioned handshake ABI — the ONE symbol (`cerulion_heaphook_abi`) the
//! rmw `dlsym`s at init, plus the pure decision logic that gates the whole
//! feature on it.
//!
//! # Contract (the rmw borrow consumer API)
//!
//! At `rmw` init the consumer does:
//!
//! ```c
//! const HeaphookAbi *(*get)() = dlsym(RTLD_DEFAULT, "cerulion_heaphook_abi");
//! ```
//!
//! and feeds the outcome through [`decide_handshake`]:
//!
//! * symbol ABSENT (`get == NULL`) → no hook is preloaded → today's copy path;
//! * `abi->version != HEAPHOOK_ABI_VERSION` → a skewed hook → copy path, loudly
//!   ONCE;
//! * `abi->status()` without [`HEAPHOOK_STATUS_WON_MALLOC`] → another malloc
//!   interposer (jemalloc/tcmalloc/ASan) won resolution → copy path, loudly
//!   ONCE;
//! * otherwise the hook is [`HandshakeVerdict::Active`] and the consumer uses
//!   the vtable's `arm_window`/`disarm_window`/`register_segment`/… entries.
//!
//! Degrade is never a failed publish — it is exactly the behaviour without a hook,
//! reported once. This decision logic lives here (pure, `#[cfg]`-free) so it is
//! unit-tested on every platform even though the rmw borrow consumer is not built
//! yet — the "no inert shipping" rule: the gate is proven where it is defined.

use core::ffi::c_void;

/// The ABI version this library implements. Bumped on any change to the
/// [`HeaphookAbi`] layout or the semantics of its entries.
///
/// v2: appended `retire_slot` + `counter` to the `repr(C)` vtable. Appending to
/// a struct a consumer indexes by offset IS a layout change, so an rmw consumer
/// compiled against v1 must reject a v2 hook (and vice versa) at the `dlsym`
/// handshake — that rejection, via [`decide_handshake`], is exactly the point.
pub const HEAPHOOK_ABI_VERSION: u32 = 2;

/// [`HeaphookAbi::status`] bit: the hook resolved as the process's `malloc`
/// (it won first-in-load-order). Clear means another interposer is ahead of us
/// and the borrow window would never bump — the consumer must degrade.
pub const HEAPHOOK_STATUS_WON_MALLOC: u32 = 1 << 0;

/// A `free`/`realloc`-style release the consumer registers: the hook calls it
/// (never glibc `free`) for a pointer inside a registered SHM range, handing
/// back the range's cookie. `None` clears the registration.
pub type ReleaseCallback = Option<unsafe extern "C" fn(ptr: *mut c_void, cookie: usize)>;

/// The handshake vtable `cerulion_heaphook_abi` returns a pointer to. `repr(C)`
/// and versioned — the consumer reads `version` first and uses nothing else on
/// a mismatch. Every entry is also exported as a standalone `#[no_mangle]`
/// symbol for tests that `dlsym` them directly.
#[repr(C)]
pub struct HeaphookAbi {
    /// Layout/semantics version; compare against [`HEAPHOOK_ABI_VERSION`].
    pub version: u32,
    /// Returns the status bitset (see [`HEAPHOOK_STATUS_WON_MALLOC`]).
    pub status: unsafe extern "C" fn() -> u32,
    /// Arm a thread-local borrow window over `[base, tail_limit)`.
    pub arm_window: unsafe extern "C" fn(base: *mut c_void, tail_limit: *mut c_void) -> i32,
    /// Disarm the thread's window; returns the latched escape code (0 = none,
    /// see [`crate::window::EscapeKind::as_code`]) or [`RC_ERR_NOT_ARMED`].
    pub disarm_window: unsafe extern "C" fn() -> i32,
    /// The armed window's latched escape code (0 = none) or [`RC_ERR_NOT_ARMED`].
    pub window_escape: unsafe extern "C" fn() -> i32,
    /// 1 iff `[ptr, ptr + len)` is fully inside the armed window's bumped region
    /// (the adopt test); 0 if not; [`RC_ERR_NOT_ARMED`] if no window is armed.
    pub window_range_test: unsafe extern "C" fn(ptr: *const c_void, len: usize) -> i32,
    /// Register a SHM range `[start, start + len)` with an opaque `cookie`.
    pub register_segment:
        unsafe extern "C" fn(start: *mut c_void, len: usize, cookie: usize) -> i32,
    /// Unregister the range starting at `start`.
    pub unregister_segment: unsafe extern "C" fn(start: *mut c_void) -> i32,
    /// Set (or clear, with `None`) the release callback.
    pub set_release_callback: unsafe extern "C" fn(cb: ReleaseCallback) -> i32,
    /// Retire a disarmed slot's quarantine extent by its `base` address (the
    /// consumer calls this once the backing sample is reclaimed) — TOMBSTONE
    /// on-retire: the extent is marked dead-but-remembered, NOT
    /// deleted, so a late `free` of an incidental in-slot pointer that outlived
    /// the sample stays a counted no-op (kind 4) forever instead of reaching
    /// glibc; a pool that recycles the slot address REVIVES the tombstone on
    /// the next `arm_window`. Returns [`RC_OK`] if an entry started at `base`,
    /// [`RC_ERR_UNKNOWN`] otherwise. See [`crate::classify`].
    pub retire_slot: unsafe extern "C" fn(base: *mut c_void) -> i32,
    /// Read a Principle-#3 diagnostic counter by index: 0 = releases with no
    /// callback registered (leaked ranges), 1 = bootstrap-arena exhaustions,
    /// 2 = pre-resolution real-heap-free leaks, 3 = LIVE quarantine no-op frees
    /// (an in-slot pointer freed after disarm, before retire), 4 = tombstone
    /// hits (an in-slot pointer freed AFTER `retire_slot` — the retired slot's
    /// coverage working as designed; sustained growth just means a slot tail is
    /// still being freed long after retire, benign), 5 = atfork-interval
    /// registered-range leaks (a foreign prepare handler freed a registered SHM
    /// range during the prepare-lock interval — leaked, not released). An
    /// unknown index returns `u64::MAX`.
    pub counter: unsafe extern "C" fn(kind: u32) -> u64,
}

// ── Shared i32 return codes (FFI + consumer + tests) ────────────────────────

/// Success.
pub const RC_OK: i32 = 0;
/// `arm_window` while a window is already armed on this thread (nested arm).
pub const RC_ERR_ALREADY_ARMED: i32 = -1;
/// A window query/disarm with no window armed on this thread.
pub const RC_ERR_NOT_ARMED: i32 = -2;
/// A bad argument (null/zero-length registration).
pub const RC_ERR_BAD_ARG: i32 = -3;
/// `register_segment` for a range overlapping a live one.
pub const RC_ERR_OVERLAP: i32 = -4;
/// `unregister_segment` for a range that was never registered.
pub const RC_ERR_UNKNOWN: i32 = -5;
/// A thread-local resource (the window) is unavailable — the thread is being
/// torn down. Distinct from `RC_ERR_ALREADY_ARMED` (a real nested arm).
pub const RC_ERR_UNAVAILABLE: i32 = -6;

/// The consumer-side handshake outcome.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HandshakeVerdict {
    /// Use the hook: arm windows, register segments, forge in place.
    Active,
    /// No `cerulion_heaphook_abi` symbol — no hook preloaded. Copy path.
    DegradeAbsent,
    /// The hook's version differs from what the consumer implements. Copy path.
    DegradeVersionMismatch,
    /// Another malloc interposer won resolution. Copy path.
    DegradeForeignAllocator,
}

impl HandshakeVerdict {
    /// Whether the hook may be used.
    pub fn is_active(self) -> bool {
        matches!(self, HandshakeVerdict::Active)
    }
}

/// The pure handshake decision the rmw borrow consumer runs at init.
///
/// `present` — did `dlsym` find `cerulion_heaphook_abi`; `hook_version` — the
/// vtable's `version`; `expected_version` — [`HEAPHOOK_ABI_VERSION`] as the
/// consumer was compiled against; `status` — the vtable's `status()` bitset.
///
/// Order is deliberate: absence first (nothing else is readable), then version
/// (a skewed vtable's `status` layout is untrusted), then the won-malloc bit.
pub fn decide_handshake(
    present: bool,
    hook_version: u32,
    expected_version: u32,
    status: u32,
) -> HandshakeVerdict {
    if !present {
        return HandshakeVerdict::DegradeAbsent;
    }
    if hook_version != expected_version {
        return HandshakeVerdict::DegradeVersionMismatch;
    }
    if status & HEAPHOOK_STATUS_WON_MALLOC == 0 {
        return HandshakeVerdict::DegradeForeignAllocator;
    }
    HandshakeVerdict::Active
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_absent_symbol_degrades_regardless_of_the_other_fields() {
        assert_eq!(
            decide_handshake(
                false,
                HEAPHOOK_ABI_VERSION,
                HEAPHOOK_ABI_VERSION,
                HEAPHOOK_STATUS_WON_MALLOC
            ),
            HandshakeVerdict::DegradeAbsent
        );
    }

    #[test]
    fn a_version_mismatch_degrades_before_status_is_trusted() {
        assert_eq!(
            decide_handshake(true, 2, 1, HEAPHOOK_STATUS_WON_MALLOC),
            HandshakeVerdict::DegradeVersionMismatch
        );
        // Even with the won-malloc bit set, a wrong version does not activate.
        assert_eq!(
            decide_handshake(true, 0, 1, HEAPHOOK_STATUS_WON_MALLOC),
            HandshakeVerdict::DegradeVersionMismatch
        );
    }

    #[test]
    fn a_matching_hook_that_did_not_win_malloc_degrades_foreign() {
        assert_eq!(
            decide_handshake(true, 1, 1, 0),
            HandshakeVerdict::DegradeForeignAllocator,
            "another interposer is ahead of us → copy path"
        );
    }

    #[test]
    fn a_present_matching_won_malloc_hook_is_active() {
        let v = decide_handshake(true, 1, 1, HEAPHOOK_STATUS_WON_MALLOC);
        assert_eq!(v, HandshakeVerdict::Active);
        assert!(v.is_active());
    }

    #[test]
    fn extra_status_bits_do_not_disturb_the_won_malloc_test() {
        // Future status bits must not flip an active verdict.
        let status = HEAPHOOK_STATUS_WON_MALLOC | 0b1010;
        assert_eq!(
            decide_handshake(true, 1, 1, status),
            HandshakeVerdict::Active
        );
    }

    #[test]
    fn only_active_reports_is_active() {
        assert!(!HandshakeVerdict::DegradeAbsent.is_active());
        assert!(!HandshakeVerdict::DegradeVersionMismatch.is_active());
        assert!(!HandshakeVerdict::DegradeForeignAllocator.is_active());
        assert!(HandshakeVerdict::Active.is_active());
    }

    #[test]
    fn the_vtable_is_repr_c_and_pointer_word_sized() {
        // Layout sanity: a u32 + ten fn pointers, u32 padded to a pointer word.
        // (Exact size is target-dependent for fn pointers; assert it is at
        // least the ten pointers + the padded version word.)
        let min = core::mem::size_of::<usize>() * 10 + core::mem::size_of::<usize>();
        assert!(core::mem::size_of::<HeaphookAbi>() >= min);
        assert_eq!(
            core::mem::align_of::<HeaphookAbi>(),
            core::mem::align_of::<usize>()
        );
    }

    #[test]
    fn return_codes_are_distinct() {
        let codes = [
            RC_OK,
            RC_ERR_ALREADY_ARMED,
            RC_ERR_NOT_ARMED,
            RC_ERR_BAD_ARG,
            RC_ERR_OVERLAP,
            RC_ERR_UNKNOWN,
            RC_ERR_UNAVAILABLE,
        ];
        for (i, a) in codes.iter().enumerate() {
            for b in &codes[i + 1..] {
                assert_ne!(a, b, "return codes must be distinct");
            }
        }
    }
}
