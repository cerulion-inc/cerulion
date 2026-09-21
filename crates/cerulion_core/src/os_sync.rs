// SPDX-License-Identifier: AGPL-3.0-only
//! The shared Apple `os_sync_*` FFI backend (macOS ≥ 14.4).
//!
//! The `os_sync_wait_on_address_with_timeout` / `os_sync_wake_by_address_all`
//! family is Apple's public futex-shaped kernel wait (macOS ≥ 14.4,
//! `<os/os_sync_wait_on_address.h>`). It is bound at RUNTIME via `dlsym` so an
//! OLDER macOS degrades to a sleep fallback instead of dyld-aborting at launch
//! — never a direct `extern` reference.
//!
//! TWO consumers share this ONE backend (the no-two-copies rule):
//!
//! - [`crate::barrier`]: the barrier boundary wait + the step-start
//!   park wake word — cross-process SHM words, `OS_SYNC_*_SHARED`, woken by a
//!   peer's `os_sync_wake_by_address_all`.
//! - [`crate::monitor_wait`]: the live-loop park's degraded-tier
//!   NAP — a process-local scratch word with NO waker, where the timed wait is
//!   used purely as a tighter-slop replacement for `thread::sleep` (measured
//!   on the M3 Max: ~+52% p50 overshoot for `nanosleep` vs ~+26% for an
//!   os_sync timeout, at the park's exact 100µs recheck).
//!
//! The [`OS_SYNC_DISABLED`] latch is deliberately ONE latch for the whole
//! family: an `EINVAL`/`ENOTSUP` from the kernel means the PRIMITIVE is
//! unusable on this host, not one call shape — so every consumer degrades
//! together. Each consumer keeps its OWN kill switch (the barrier's
//! `CERULION_BARRIER_OS_SYNC`, the park nap's `CERULION_PARK_OS_SYNC`): the
//! env names are consumer-facing surface, and one switch silently disabling an
//! unrelated tier is the misleading-name class this repo rejects. The pure
//! `=0`/`=1`/garbage grammar ([`parse_os_sync_kill_switch`]) IS shared, so the
//! two switches cannot drift in what they accept, and
//! that grammar LIVES in [`crate::kill_switch`], platform-neutral, with the
//! credit plane's two switches sharing it as well. This module re-exports it
//! under its own name; it owns the FFI backend and the latch, not the grammar.
//!
//! Whole module `#[cfg(target_os = "macos")]` (registered as such in
//! `lib.rs`): no other target compiles any of this.

use core::ffi::{c_int, c_void};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::OnceLock;

/// The two Apple `os_sync_*` symbols the macOS kernel-wait tiers bind at
/// runtime. Typed fn pointers whose signatures MATCH libc 0.2.186's declared
/// `os_sync_wait_on_address_with_timeout` / `os_sync_wake_by_address_all`
/// (`size_t == usize`, `os_clockid_t == u32`, `*_flags_t == u32`), so the
/// `dlsym` void* → fn-pointer transmute is sound. Two fn pointers are
/// `Copy + Send + Sync`, so the struct needs no `unsafe impl` to sit in the
/// process-global [`OnceLock`].
pub(crate) struct OsSyncFns {
    /// `int os_sync_wait_on_address_with_timeout(void *addr, uint64_t value,
    /// size_t size, uint32_t flags, os_clockid_t clockid, uint64_t timeout_ns)`.
    pub(crate) wait: unsafe extern "C" fn(*mut c_void, u64, usize, u32, u32, u64) -> c_int,
    /// `int os_sync_wake_by_address_all(void *addr, size_t size, uint32_t flags)`.
    pub(crate) wake: unsafe extern "C" fn(*mut c_void, usize, u32) -> c_int,
}

/// Process-global "the os_sync primitive is UNUSABLE" latch. Set the first
/// time ANY consumer sees an UNRECOVERABLE errno (`EINVAL`/`ENOTSUP`) from the
/// kernel wait; once set, every consumer's activity gate reads false and takes
/// its sleep fallback. NEVER cleared (a primitive that `EINVAL`s once
/// `EINVAL`s always). `Relaxed`: a benign monotone latch — eventual
/// cross-thread visibility suffices.
static OS_SYNC_DISABLED: AtomicBool = AtomicBool::new(false);

/// `true` iff the os_sync family has been latched UNUSABLE by an
/// unrecoverable errno (see [`latch_os_sync_disabled`]). Consumers fold this
/// into their own activity gates.
pub(crate) fn os_sync_latched() -> bool {
    OS_SYNC_DISABLED.load(Ordering::Relaxed)
}

/// Latch the os_sync family UNUSABLE process-wide. Returns `true` iff THIS
/// call flipped the latch (the caller's warn-once gate — the first site to
/// observe the unrecoverable errno logs it; every later site degrades
/// silently).
pub(crate) fn latch_os_sync_disabled() -> bool {
    latch_flip(family_latch())
}

/// The ONE process-global family latch, exposed by reference so a consumer's
/// error arm can be written against an INJECTED `&AtomicBool` (the
/// `latch_flip` seam) while its production wrapper passes THIS — the park
/// nap's `park_nap_with_backend` does exactly that (the
/// EINVAL/ENOTSUP arm is unreachable on a healthy kernel, so its test drives
/// a fake backend against a LOCAL latch; really flipping this global would
/// degrade every sibling test that reads an os_sync activity gate).
pub(crate) fn family_latch() -> &'static AtomicBool {
    &OS_SYNC_DISABLED
}

/// The pure latch-flip over an INJECTED flag — the testable half of
/// [`latch_os_sync_disabled`], split out so the flip-exactly-once contract is
/// pinnable WITHOUT mutating the process-global [`OS_SYNC_DISABLED`] (which
/// sibling lib tests in this binary read through their activity gates — a
/// test that really latched it would race/degrade them under parallel
/// libtest, the CountingAllocator process-global-state class).
pub(crate) fn latch_flip(flag: &AtomicBool) -> bool {
    !flag.swap(true, Ordering::Relaxed)
}

/// Resolve a symbol from the default set of loaded images (`RTLD_DEFAULT` →
/// libSystem) via `dlsym`. `None` when the symbol is absent (macOS < 14.4
/// lacks the `os_sync_*` family). Safe to callers: `name` is a static
/// NUL-terminated symbol string, so no UB is reachable.
fn resolve_symbol(name: &[u8]) -> Option<std::ptr::NonNull<c_void>> {
    debug_assert_eq!(name.last(), Some(&0), "symbol name must be NUL-terminated");
    // SAFETY: `RTLD_DEFAULT` is a documented dlsym pseudo-handle; `name` is a
    // static NUL-terminated C string. `dlsym` reads no caller memory beyond it.
    let sym = unsafe { libc::dlsym(libc::RTLD_DEFAULT, name.as_ptr() as *const libc::c_char) };
    std::ptr::NonNull::new(sym)
}

/// The resolved Apple os_sync backend, or `None` on a host lacking the
/// symbols (macOS < 14.4). Resolved ONCE via `dlsym(RTLD_DEFAULT, …)` — NOT a
/// direct extern reference, which would make dyld abort at LAUNCH on an older
/// macOS — and cached process-wide in a `OnceLock`. Emits ONE
/// `tracing::debug!` naming the resolution outcome the first time it runs.
pub(crate) fn os_sync_backend() -> Option<&'static OsSyncFns> {
    static BACKEND: OnceLock<Option<OsSyncFns>> = OnceLock::new();
    BACKEND
        .get_or_init(|| {
            let wait = resolve_symbol(b"os_sync_wait_on_address_with_timeout\0");
            let wake = resolve_symbol(b"os_sync_wake_by_address_all\0");
            match (wait, wake) {
                (Some(wait), Some(wake)) => {
                    // SAFETY: both symbols resolved from a loaded image; each is
                    // transmuted to the fn-pointer type whose signature matches
                    // libc 0.2.186's declaration of that exact symbol (see the
                    // `OsSyncFns` field docs). `dlsym` yields a code pointer,
                    // pointer-sized like the fn pointer, so the transmute is sound.
                    let wait: unsafe extern "C" fn(
                        *mut c_void,
                        u64,
                        usize,
                        u32,
                        u32,
                        u64,
                    ) -> c_int = unsafe { std::mem::transmute(wait.as_ptr()) };
                    let wake: unsafe extern "C" fn(*mut c_void, usize, u32) -> c_int =
                        unsafe { std::mem::transmute(wake.as_ptr()) };
                    tracing::debug!(
                        tier = "macos-os_sync",
                        "Apple os_sync_wait/wake_by_address resolved (macOS >= 14.4) — shared by the barrier wake tier and the park nap tier"
                    );
                    Some(OsSyncFns { wait, wake })
                }
                _ => {
                    tracing::debug!(
                        tier = "macos-recheck-fallback",
                        "os_sync_* symbols absent (macOS < 14.4) — barrier wake + park nap tiers = chunked sleep-recheck"
                    );
                    None
                }
            }
        })
        .as_ref()
}

/// Classify a failing `os_sync_wait_on_address` errno as BENIGN — an EXPECTED
/// wait-loop return that re-loops silently: `ETIMEDOUT` (the slice expired)
/// or `EINTR` (a signal). A VALUE-MISMATCH is deliberately NOT here: it
/// returns a NON-NEGATIVE rc (verified on macOS: rc 0, no errno), so it is
/// handled by the caller's re-poll, never this classifier. Pure —
/// oracle-testable without a syscall seam.
pub(crate) fn os_sync_errno_is_benign(errno: i32) -> bool {
    matches!(errno, libc::ETIMEDOUT | libc::EINTR)
}

/// Classify a failing `os_sync_wait_on_address` errno as UNRECOVERABLE — the
/// kernel primitive itself is unusable, so the observing consumer latches
/// [`OS_SYNC_DISABLED`] (via [`latch_os_sync_disabled`]) and degrades to its
/// sleep fallback: `EINVAL` (bad args / unsupported operand size) or
/// `ENOTSUP` (the primitive is not supported here). An errno that is NEITHER
/// benign NOR unrecoverable takes the warn-once + bounded-sleep middle arm.
/// Pure.
pub(crate) fn os_sync_errno_is_unrecoverable(errno: i32) -> bool {
    matches!(errno, libc::EINVAL | libc::ENOTSUP)
}

/// Is the shared os_sync BACKEND usable on this host — resolved,
/// and not latched off by an unrecoverable errno?
///
/// This is the FACT half of every consumer's gate, with NO consumer's kill
/// switch folded in. It exists because folding one was a real bug: the credit
/// plane used to ask [`crate::barrier`] the availability question, and the
/// barrier's `os_sync_active()` ANDs in `CERULION_BARRIER_OS_SYNC` — so
/// disabling the BARRIER's macOS tier silently disabled the CREDIT plane's too,
/// while the docs promised the two switches were independent.
///
/// The split this restores is the one `parse_kill_switch`'s module docs state:
/// a shared FACT is read from ONE place, and a per-consumer DECISION is made
/// per consumer. A consumer asks this, then applies its OWN switch. Both gates
/// are cached atomic loads.
pub(crate) fn os_sync_backend_usable() -> bool {
    os_sync_backend().is_some() && !os_sync_latched()
}

/// The shared kill-switch grammar, under the name this module's two macOS
/// consumers already call it — accurate there, since `CERULION_BARRIER_OS_SYNC`
/// and `CERULION_PARK_OS_SYNC` really are os_sync switches.
///
/// The DEFINITION moved to [`crate::kill_switch`]: the
/// grammar is pure and platform-neutral, and the credit plane's
/// `CERULION_CREDIT_WAKE` is a cross-platform consumer, so a copy gated behind
/// `#[cfg(target_os = "macos")]` simply did not exist on Linux. Re-exported
/// rather than re-imported at each call site so `barrier.rs` and
/// `monitor_wait.rs` keep their current imports — one definition, no drift.
pub(crate) use crate::kill_switch::parse_kill_switch as parse_os_sync_kill_switch;

#[cfg(test)]
mod tests {
    use super::*;

    /// The latch-flip contract — the flip is reported EXACTLY once
    /// (the caller's warn-once gate) and never un-reported. Driven over an
    /// INJECTED local flag ([`latch_flip`], the seam split out for exactly
    /// this): really flipping the process-global [`OS_SYNC_DISABLED`] in the
    /// shared lib binary would race/degrade every sibling test that reads an
    /// os_sync activity gate (the barrier's `park_wait_activity` arms run in
    /// this same binary under parallel libtest).
    #[test]
    fn latch_flip_reports_the_flip_exactly_once_and_never_clears() {
        let flag = AtomicBool::new(false);
        assert!(
            latch_flip(&flag),
            "the FIRST latch call must report the flip (the warn-once gate)"
        );
        assert!(flag.load(Ordering::Relaxed), "the latch must read back set");
        assert!(
            !latch_flip(&flag),
            "a SECOND latch call must NOT report a flip (later sites degrade silently)"
        );
        assert!(
            flag.load(Ordering::Relaxed),
            "the latch stays set — never cleared"
        );
    }

    /// Print-only backend probe — whether the os_sync symbols
    /// resolved on THIS host, so CI logs show whether the real primitive ran.
    /// Never asserts a value (host-dependent: macOS ≥ 14.4 resolves, < 14.4
    /// does not). The barrier keeps its own tier probe
    /// (`os_sync_backend_probe_prints_availability`).
    #[test]
    fn backend_probe_prints_availability() {
        let present = os_sync_backend().is_some();
        eprintln!("os_sync backend available on this host: {present}");
    }

    /// WHEN the backend resolved on this host, a timed wait on a
    /// never-woken process-local word really blocks for at least the timeout
    /// and returns the BENIGN `ETIMEDOUT` — the dlsym-resolution + real-kernel
    /// pin for the park-nap call shape (LOCAL flags, 8-byte word,
    /// `OS_CLOCK_MACH_ABSOLUTE_TIME`, relative ns). On macOS < 14.4 this
    /// prints a skip (the availability itself is host truth, not a failure).
    /// The lower bound is load-SAFE (contention only lengthens a timed wait);
    /// no upper wall is asserted (the class a loaded runner inverts).
    #[test]
    fn a_timed_wait_on_a_never_woken_word_times_out_benignly() {
        let Some(backend) = os_sync_backend() else {
            eprintln!("skipping: os_sync backend absent on this host (macOS < 14.4)");
            return;
        };
        let word: u64 = 0;
        let timeout = std::time::Duration::from_micros(500);
        let started = std::time::Instant::now();
        // SAFETY: `word` is a live, 8-byte-aligned local alive across the call;
        // `backend.wait` is the dlsym-resolved, signature-checked
        // `os_sync_wait_on_address_with_timeout`. LOCAL flags — the word is
        // process-private. The passed `value` equals the word's value, so the
        // kernel compare matches and the wait really blocks until the timeout.
        let rc = unsafe {
            (backend.wait)(
                &word as *const u64 as *mut c_void,
                0,
                8,
                libc::OS_SYNC_WAIT_ON_ADDRESS_NONE,
                libc::OS_CLOCK_MACH_ABSOLUTE_TIME,
                timeout.as_nanos() as u64,
            )
        };
        let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
        assert!(
            rc < 0,
            "a never-woken timed wait must return rc < 0 (timeout)"
        );
        assert!(
            os_sync_errno_is_benign(errno),
            "the timeout errno must classify BENIGN (got errno {errno})"
        );
        assert!(
            started.elapsed() >= timeout,
            "the timed wait must block at least its timeout (got {:?})",
            started.elapsed()
        );
    }

    /// Moved with the fn: the os_sync errno classifier, the
    /// load-bearing half of the unexpected-errno degrade (misclassifying
    /// `ETIMEDOUT` as unrecoverable would latch the primitive off on every
    /// slice/nap expiry; misclassifying `EINVAL` as benign would busy-re-loop a
    /// broken park). Oracle vector, all three arms: BENIGN {`ETIMEDOUT`,
    /// `EINTR`}, UNRECOVERABLE {`EINVAL`, `ENOTSUP`}, and NEITHER (the
    /// warn-once + bounded-sleep middle arm).
    #[test]
    fn errno_classification_oracle() {
        for errno in [libc::ETIMEDOUT, libc::EINTR] {
            assert!(
                os_sync_errno_is_benign(errno),
                "errno {errno} is an expected os_sync return and must re-loop hot"
            );
            assert!(
                !os_sync_errno_is_unrecoverable(errno),
                "benign errno {errno} must not be classified unrecoverable"
            );
        }
        for errno in [libc::EINVAL, libc::ENOTSUP] {
            assert!(
                os_sync_errno_is_unrecoverable(errno),
                "errno {errno} means the primitive is unusable → disable + fallback"
            );
            assert!(
                !os_sync_errno_is_benign(errno),
                "unrecoverable errno {errno} must not be classified benign"
            );
        }
        // NEITHER — the warn-once + bounded-sleep middle arm.
        for errno in [libc::EFAULT, libc::EAGAIN, libc::EPERM, 0] {
            assert!(
                !os_sync_errno_is_benign(errno),
                "errno {errno} is neither benign …"
            );
            assert!(
                !os_sync_errno_is_unrecoverable(errno),
                "… nor unrecoverable (takes the warn+sleep middle arm)"
            );
        }
    }
}
