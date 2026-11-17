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
//!
//! [`EstopLease::holds`] is the e-stop read that the act verbs' refusal,
//! `restart`'s gate and the rcp plane's Act check share; the act verbs'
//! watcher and send path read the latch themselves. Its lock wait is
//! bounded by the same one send.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, LockResult, Mutex, MutexGuard, PoisonError, TryLockResult};

use cerud::lease::{ControlLease, EstopState, SafeFrame};

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

    /// Whether the e-stop holds: an engage is counted ([`EstopLease::engage_pending`],
    /// read without the lease's lock and again under it), or the latch reads
    /// engaged under the lock, which is held for those reads only. A poisoned
    /// lock reads engaged.
    pub fn holds(&self) -> bool {
        self.engage_pending() || self.holds_under_lock()
    }

    /// [`EstopLease::holds`] past its lock-free read: under the lease's lock,
    /// an engage is counted or the latch reads engaged. A poisoned lock reads
    /// engaged.
    fn holds_under_lock(&self) -> bool {
        match self.lease.lock() {
            Ok(latch) => self.engage_pending() || !matches!(latch.estop(), EstopState::Clear),
            Err(_) => true,
        }
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;
    use std::time::{Duration, Instant};

    fn lease() -> EstopLease {
        EstopLease::new(Arc::new(Mutex::new(ControlLease::new(1_000_000_000))))
    }

    #[test]
    fn a_clear_latch_with_no_engage_counted_does_not_hold() {
        assert!(!lease().holds());
    }

    #[test]
    fn an_engaged_latch_holds() {
        let l = lease();
        l.lock().unwrap().engage_estop("test");
        assert!(!l.engage_pending());
        assert!(l.holds());
    }

    #[test]
    fn a_poisoned_lock_holds() {
        let l = lease();
        let poisoning = l.clone();
        let _ = std::thread::spawn(move || {
            let _held = poisoning.lock().unwrap();
            panic!("poison the lease's lock");
        })
        .join();
        assert!(l.lock().is_err(), "the lock is not poisoned");
        assert!(!l.engage_pending());
        assert!(l.holds());
    }

    #[test]
    fn a_counted_engage_holds_without_the_lease_lock() {
        let l = lease();
        let held = l.lock().unwrap();
        let engaging = l.clone();
        let engager = std::thread::spawn(move || {
            let _ = engaging.engage("test");
        });
        let until = Instant::now() + Duration::from_secs(10);
        while !l.engage_pending() && Instant::now() < until {
            std::thread::sleep(Duration::from_millis(1));
        }
        let counted = l.engage_pending();
        let (tx, rx) = mpsc::channel();
        let reading = l.clone();
        let reader = std::thread::spawn(move || {
            let _ = tx.send(reading.holds());
        });
        let read = rx.recv_timeout(Duration::from_secs(10));
        let latch_clear = matches!(held.estop(), EstopState::Clear);
        drop(held);
        engager.join().unwrap();
        reader.join().unwrap();
        assert!(counted, "the engage was not counted while it waited");
        assert!(
            latch_clear,
            "the latch engaged while the test held its lock"
        );
        assert_eq!(
            read,
            Ok(true),
            "holds() waited for the lease's lock instead of reading the counted engage"
        );
        assert!(l.holds(), "the engage did not engage the latch");
    }

    #[test]
    fn an_engage_counted_while_holds_waits_for_the_lease_lock_holds() {
        // The lock-free read has already passed when the engage counts in:
        // `holds_under_lock` is the part of `holds` that waits for the lock,
        // so the count must be read again once the lock is taken.
        let l = lease();
        let held = l.lock().unwrap();
        let started = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let (tx, rx) = mpsc::channel();
        let reader = {
            let (l, started) = (l.clone(), started.clone());
            std::thread::spawn(move || {
                started.store(true, Ordering::SeqCst);
                let _ = tx.send(l.holds_under_lock());
            })
        };
        let until = Instant::now() + Duration::from_secs(10);
        while !started.load(Ordering::SeqCst) && Instant::now() < until {
            std::thread::yield_now();
        }
        let reached = started.load(Ordering::SeqCst);
        // The reader cannot return before the lock is released.
        let early = rx.try_recv().is_ok();
        let counted = CountedIn::new(&l.engaging);
        drop(held);
        let read = rx.recv_timeout(Duration::from_secs(10));
        drop(counted);
        reader.join().unwrap();
        assert!(reached, "the reader thread did not start");
        assert!(!early, "the reader returned while the test held the lock");
        assert_eq!(
            read,
            Ok(true),
            "an engage counted while holds() waited for the lock was missed"
        );
    }
}
