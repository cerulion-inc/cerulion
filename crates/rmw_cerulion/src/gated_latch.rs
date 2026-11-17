// SPDX-License-Identifier: AGPL-3.0-only
//! A flood latch behind a mutex, with an open-regime mark beside it that a
//! success reads with one relaxed load.
//!
//! A per-frame success report (`on_success` of a healthy latch) changes nothing
//! and logs nothing, so [`GatedLatch::on_success`] skips the lock unless a
//! failure was recorded since the last success. A failure marks the regime open
//! while it holds the lock, and a success clears the mark while it holds the
//! lock, so the mark is written in the latch's own lock order. A stale relaxed
//! read can only delay one closing success by one report. The take paths report
//! under the subscription's, server's or client's own mutex, and a wait set's
//! drain latch is reported by one thread at a time (the `rcl_wait` contract),
//! ordered across threads by the caller that hands the wait set over.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Mutex, MutexGuard};

/// A latch `L` behind a mutex, plus the open-regime mark (see the module docs).
#[derive(Debug, Default)]
pub struct GatedLatch<L> {
    open: AtomicBool,
    latch: Mutex<L>,
}

/// Read access to a [`GatedLatch`]'s latch: its counters, never a transition.
pub struct LatchRead<'a, L>(MutexGuard<'a, L>);

impl<L> std::ops::Deref for LatchRead<'_, L> {
    type Target = L;

    fn deref(&self) -> &L {
        &self.0
    }
}

impl<L> GatedLatch<L> {
    /// A latch with no regime open.
    pub const fn new(latch: L) -> Self {
        Self {
            open: AtomicBool::new(false),
            latch: Mutex::new(latch),
        }
    }

    /// The latch, locked to record a failure. The regime is marked open while
    /// the lock is held, so the next [`Self::on_success`] takes the lock.
    pub fn lock_failure(&self) -> MutexGuard<'_, L> {
        let guard = self.locked();
        self.open.store(true, Ordering::Relaxed);
        guard
    }

    /// Run `success` on the locked latch when a failure was recorded since the
    /// last success, and clear the mark under the same lock. Otherwise return
    /// `None` without taking the lock: `success` must be the latch's success
    /// transition, which leaves a latch with no open regime unchanged.
    pub fn on_success<R>(&self, success: impl FnOnce(&mut L) -> R) -> Option<R> {
        if !self.open.load(Ordering::Relaxed) {
            return None;
        }
        let mut guard = self.locked();
        let out = success(&mut guard);
        self.open.store(false, Ordering::Relaxed);
        Some(out)
    }

    /// The latch, locked for reading its counters.
    pub fn read(&self) -> LatchRead<'_, L> {
        LatchRead(self.locked())
    }

    fn locked(&self) -> MutexGuard<'_, L> {
        self.latch.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Test only: hold the latch's mutex, so a call that would lock it blocks.
    #[cfg(test)]
    fn hold_for_test(&self) -> MutexGuard<'_, L> {
        self.locked()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};

    /// A success on a latch with no failure recorded runs nothing and takes no
    /// lock: it returns while another holder keeps the mutex locked.
    #[test]
    fn a_success_with_no_open_regime_takes_no_lock() {
        let latch = GatedLatch::new(FailureRegimeLatch::new());
        let _held = latch.hold_for_test();
        let mut ran = false;
        assert_eq!(latch.on_success(|_| ran = true), None);
        assert!(!ran, "no success transition runs without an open regime");
    }

    /// The regime cycle against a hand oracle: a failure opens it, the next
    /// success closes it under the lock and reports the suppressed count, and
    /// the success after that runs nothing. A failure after recovery is loud
    /// again.
    #[test]
    fn a_failure_opens_the_regime_and_one_success_closes_it() {
        let latch = GatedLatch::new(FailureRegimeLatch::new());
        assert_eq!(latch.lock_failure().on_failure(), RegimeDecision::Loud);
        assert!(matches!(
            latch.lock_failure().on_failure(),
            RegimeDecision::Suppressed { suppressed: 1 }
        ));
        assert_eq!(latch.on_success(|l| l.on_success()), Some(Some(1)));
        assert!(!latch.read().is_failing());
        assert_eq!(latch.on_success(|l| l.on_success()), None);
        assert_eq!(latch.lock_failure().on_failure(), RegimeDecision::Loud);
        assert_eq!(latch.read().total_failures(), 3);
    }
}
