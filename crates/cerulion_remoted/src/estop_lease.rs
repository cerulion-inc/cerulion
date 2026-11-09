// SPDX-License-Identifier: AGPL-3.0-only
//! [`EstopLease`]: the robot's control lease shared with a count of the
//! e-stop engages that wait for its lock.
//!
//! The lease is a [`std::sync::Mutex`], which does not queue its waiters in
//! order: a thread that unlocks it and locks it again at once can take it
//! ahead of a thread that is already waiting. The `topic-publish` and
//! `service-call` act verbs send under that lock ([`crate::act_verbs`]).
//! [`EstopLease::engage`] counts the engage in before it waits for the lock
//! and out after it has engaged the latch, and the act verbs send only when
//! they read the count at zero under the lock, so an engage waits for at most
//! the one send that holds the lock when it starts waiting.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, LockResult, Mutex, MutexGuard, PoisonError, TryLockResult};

use cerud::lease::{ControlLease, SafeFrame};

/// The robot's control lease and the count of e-stop engages waiting for its
/// lock or holding it. Clones share both, so the e-stop verb and the act
/// verbs that share a lease are built from clones of one `EstopLease`.
#[derive(Clone, Debug)]
pub struct EstopLease {
    lease: Arc<Mutex<ControlLease>>,
    engaging: Arc<AtomicUsize>,
}

impl EstopLease {
    /// `lease` with a count of its own, at zero. An engage through another
    /// `EstopLease` over the same lease is not counted on this one.
    pub fn new(lease: Arc<Mutex<ControlLease>>) -> Self {
        EstopLease {
            lease,
            engaging: Arc::new(AtomicUsize::new(0)),
        }
    }

    /// The control lease.
    pub fn lease(&self) -> &Arc<Mutex<ControlLease>> {
        &self.lease
    }

    /// Lock the control lease. An engage through this lock is not counted;
    /// [`EstopLease::engage`] is.
    pub fn lock(&self) -> LockResult<MutexGuard<'_, ControlLease>> {
        self.lease.lock()
    }

    /// Lock the control lease if no other thread holds it.
    pub fn try_lock(&self) -> TryLockResult<MutexGuard<'_, ControlLease>> {
        self.lease.try_lock()
    }

    /// Engage the e-stop for `session`: count the engage in, wait for the
    /// lease's lock (a poisoned lock is taken as it is), engage the latch,
    /// release the lock, and count the engage out, also on a panic. Returns
    /// the safe frame now in force.
    pub fn engage(&self, session: &str) -> SafeFrame {
        let _counted = CountedIn::new(&self.engaging);
        let mut lease = self.lease.lock().unwrap_or_else(PoisonError::into_inner);
        lease.engage_estop(session)
    }

    /// True from when an engage counts itself in until it counts itself out,
    /// after it has engaged the latch and released the lease's lock.
    pub fn engage_pending(&self) -> bool {
        self.engaging.load(Ordering::SeqCst) > 0
    }
}

impl From<Arc<Mutex<ControlLease>>> for EstopLease {
    fn from(lease: Arc<Mutex<ControlLease>>) -> Self {
        EstopLease::new(lease)
    }
}

/// Adds one to a count when made and takes it off when dropped.
struct CountedIn<'a>(&'a AtomicUsize);

impl<'a> CountedIn<'a> {
    fn new(count: &'a AtomicUsize) -> Self {
        count.fetch_add(1, Ordering::SeqCst);
        CountedIn(count)
    }
}

impl Drop for CountedIn<'_> {
    fn drop(&mut self) {
        self.0.fetch_sub(1, Ordering::SeqCst);
    }
}
