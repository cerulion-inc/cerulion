// SPDX-License-Identifier: AGPL-3.0-only
//! Fire-once thread-local fault-injection
//! hook for `<Name>Shm::spill_to_overflow`'s allocation step.
//!
//! Codegen-emitted `spill_to_overflow` calls
//! [`armed_and_consume`] AT THE TOP of the function (gated behind
//! `#[cfg(any(test, feature = "test-helpers"))]` so production
//! builds skip the check entirely). When the thread-local is armed,
//! the next call from THIS thread returns
//! `TransportError::AllocationFailed` and clears the flag.
//! Subsequent calls succeed normally.
//!
//! # Why this module is always-on (NOT cfg-gated)
//!
//! The codegen-emitted reference to `armed_and_consume()` lives in
//! `<Name>Shm::spill_to_overflow` inside any crate that uses
//! cerulion codegen (e.g. `native_ros2_messages`). That crate's
//! build does NOT inherit cerulion_core's `test-helpers` feature
//! automatically, so a `#[cfg(feature = "test-helpers")]`-gated
//! module in `cerulion_core` would be invisible to the caller's
//! codegen-emitted code — manifesting as
//! `error[E0433]: cannot find 'testing' in 'cerulion_core'` even
//! during the caller's own `cargo test` runs (the CI canary that
//! caught this pre-merge). Keeping the module
//! always-on makes the symbol resolution work in every build.
//!
//! Production cost is zero on the hot path: `spill_to_overflow` is
//! `#[cold] #[inline(never)]` and the codegen-emitted call site
//! is itself gated behind `#[cfg(any(test, feature = "test-helpers"))]`
//! so a non-test build of the *calling* crate omits the call.
//!
//! # Why thread-local
//!
//! Each `cargo test` worker owns its own thread. The hook never
//! crosses thread boundaries and never persists across tests on
//! the same thread (fire-once auto-clears). Two tests on different
//! threads can both arm + consume without seeing each other.
//!
//! # Distinct from publisher-level fault injection
//!
//! [`crate::transport::publisher::CerulionPublisher`]
//! exposes
//! `fault_inject_send_overflow_frame_after`, which triggers a
//! failure in `Drop`'s re-loan path AFTER an in-tick spill
//! succeeded. THIS module triggers the IN-TICK allocation failure
//! itself, surfacing `AllocationFailed` to the setter caller.
//! Different failure semantics; both need direct test coverage.

use std::cell::Cell;

thread_local! {
    /// `true` once `arm()` has been called and the next
    /// `spill_to_overflow` will fail. Reset to `false` after the
    /// next consume (fire-once).
    static FORCE_SPILL_OOM: Cell<bool> = const { Cell::new(false) };
}

/// Arm the next `spill_to_overflow` allocation on THIS thread to
/// return `AllocationFailed`. Fire-once: cleared after consume.
///
/// Tests only — production code has no reason to call this.
/// Kept always-public (not feature-gated) so test crates that
/// don't enable `cerulion_core/test-helpers` can still arm the
/// fault. Calling this in production is harmless: the next spill
/// returns one `AllocationFailed`, then the flag clears.
///
/// `#[doc(hidden)]` — keeps the symbol out of rustdoc's "public API"
/// surface so users don't discover it via the generated docs. The
/// symbol must remain callable from test crates (which is why it
/// can't be `pub(crate)`), but `cerulion_core`'s public API
/// documentation should not advertise it.
#[doc(hidden)]
pub fn arm() {
    FORCE_SPILL_OOM.with(|cell| cell.set(true));
}

/// Check + consume the armed flag. Returns `true` if the caller
/// (codegen-emitted `spill_to_overflow`) should immediately
/// return `AllocationFailed`. After this call returns `true` the
/// flag is cleared; subsequent `spill_to_overflow` calls on the
/// same thread succeed unless `arm()` is called again.
///
/// Called from codegen-emitted `<Name>Shm::spill_to_overflow`
/// behind a `#[cfg(any(test, feature = "test-helpers"))]` gate at
/// the call site — production builds of the *calling* crate
/// skip the call entirely.
///
/// `#[doc(hidden)]` — codegen call site only; not part of the
/// user-facing API.
#[doc(hidden)]
pub fn armed_and_consume() -> bool {
    FORCE_SPILL_OOM.with(|cell| {
        if cell.get() {
            cell.set(false);
            true
        } else {
            false
        }
    })
}
