// SPDX-License-Identifier: AGPL-3.0-only
//! The thread-local re-entrancy guard, as a pure `#[cfg]`-free state machine.
//!
//! Every interposed allocator entry point (`interpose`, Linux/GNU) does its own
//! bookkeeping through the REAL allocator (`dlsym(RTLD_NEXT, …)`), and that
//! bookkeeping — a `Vec` push into the window ledger, a `dlsym` that itself
//! `calloc`s during dynamic-linker bootstrap — re-enters `malloc`/`free`. The
//! guard makes those re-entrant calls skip our logic and go straight to the
//! real allocator, so an interposer can never recurse into itself.
//!
//! The type lives here, free of any `cfg`, so its arm/clear logic is unit-
//! tested on every platform (the mutation target: a guard that fails to arm,
//! fails to clear, or grants a nested entry deadlocks or infinitely recurses a
//! real process). The Linux FFI holds one in
//! `thread_local! { static GUARD: ReentryFlag }` and drives it through the
//! MANUAL [`ReentryFlag::try_set`] / [`ReentryFlag::clear`] pair — deliberately
//! NOT an RAII token: the flag must be set inside a `LocalKey::try_with`
//! closure while the guarded body runs OUTSIDE that closure (`with_guard`'s
//! teardown-safety pattern), and a token borrowing the flag cannot escape the
//! closure that created it — the reason an earlier,
//! caller-less `enter()` RAII API was deleted.

use core::cell::Cell;

/// A per-thread re-entrancy flag. `!Sync` by construction (`Cell`) — it is only
/// ever a thread-local, never shared.
#[derive(Debug, Default)]
pub struct ReentryFlag {
    active: Cell<bool>,
}

impl ReentryFlag {
    /// A cleared flag. `const` so it can seed a `thread_local!`.
    pub const fn new() -> Self {
        Self {
            active: Cell::new(false),
        }
    }

    /// Whether our logic is currently active on this thread.
    pub fn is_active(&self) -> bool {
        self.active.get()
    }

    /// Try to set the flag: returns `true` if it was clear (now set — the
    /// caller must later [`Self::clear`]), `false` if already set (a re-entrant
    /// call that must bypass to the real allocator). Deliberately a manual
    /// pair, not an RAII token: the flag is set inside a `LocalKey::try_with`
    /// closure while the guarded body runs OUTSIDE that closure (the FFI's
    /// teardown-safety pattern), and a token borrowing the flag could not
    /// escape the closure that created it.
    pub fn try_set(&self) -> bool {
        if self.active.get() {
            false
        } else {
            self.active.set(true);
            true
        }
    }

    /// Clear the flag (pairs with a `true` [`Self::try_set`]).
    pub fn clear(&self) {
        self.active.set(false);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fresh_flag_is_clear() {
        let flag = ReentryFlag::new();
        assert!(!flag.is_active());
    }

    #[test]
    fn try_set_and_clear_are_the_manual_pair() {
        let flag = ReentryFlag::new();
        assert!(flag.try_set(), "first try_set acquires");
        assert!(flag.is_active());
        assert!(!flag.try_set(), "a second try_set is refused (re-entrant)");
        flag.clear();
        assert!(!flag.is_active());
        assert!(flag.try_set(), "after clear, try_set acquires again");
        flag.clear();
    }

    #[test]
    fn sequential_acquisitions_each_re_arm() {
        let flag = ReentryFlag::new();
        for _ in 0..3 {
            assert!(
                flag.try_set(),
                "each round must re-arm after the prior clear"
            );
            assert!(flag.is_active());
            flag.clear();
            assert!(!flag.is_active());
        }
    }
}
