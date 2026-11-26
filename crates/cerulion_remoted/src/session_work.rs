// SPDX-License-Identifier: AGPL-3.0-only
//! Track session work until it actually finishes, even when its waiter is dropped.

use std::io;
use std::sync::{Arc, Mutex};

use tokio::sync::Notify;
use tokio::task::JoinHandle;

/// A session's shared registry of outstanding work and its first failure.
///
/// Keep a ticket while a producer can register more work. After the session's
/// driver has joined, [`Self::drain`] waits for every ticket and closes admission.
/// Dropping a task's join handle does not remove its work from this registry.
#[derive(Clone, Default)]
pub struct SessionWork(Arc<Inner>);

#[derive(Default)]
struct Inner {
    work: Mutex<Work>,
    changed: Notify,
}

#[derive(Default)]
struct Work {
    pending: usize,
    sealed: bool,
    failure: Option<String>,
}

/// An owned piece of work. Dropping it without completion records a failure.
/// Completion records the result; the ticket remains outstanding until dropped.
#[must_use = "retain the ticket until the work and its child registrations finish"]
pub struct CriticalTicket {
    work: SessionWork,
    label: &'static str,
    completed: bool,
}

impl SessionWork {
    /// Register work before starting it. Refuse new work after drain has sealed.
    pub fn ticket(&self, label: &'static str) -> io::Result<CriticalTicket> {
        let mut work = self
            .0
            .work
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        if work.sealed {
            return Err(io::Error::other("session work already drained"));
        }
        work.pending = work
            .pending
            .checked_add(1)
            .ok_or_else(|| io::Error::other("too many outstanding session tasks"))?;
        Ok(CriticalTicket {
            work: self.clone(),
            label,
            completed: false,
        })
    }

    /// Run a blocking operation while retaining its real completion in the registry.
    /// The Tokio runtime must support blocking tasks. Errors and panics remain
    /// visible to drain even if the returned handle is dropped.
    pub fn spawn_blocking<T, F>(
        &self,
        label: &'static str,
        callback: F,
    ) -> io::Result<JoinHandle<io::Result<T>>>
    where
        T: Send + 'static,
        F: FnOnce() -> io::Result<T> + Send + 'static,
    {
        let mut ticket = self.ticket(label)?;
        Ok(tokio::task::spawn_blocking(move || {
            let result = callback();
            ticket.complete(&result);
            result
        }))
    }

    /// Whether drain sealed the registry. This does not mean the work succeeded.
    pub fn settled(&self) -> bool {
        self.0
            .work
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .sealed
    }

    /// Wait for registered work, close admission and report its first failure.
    ///
    /// Call only after the driver has joined. Every nested producer must retain
    /// its parent ticket until its successor ticket has been registered. This
    /// wait never cancels the work; callers own any shutdown deadline.
    pub async fn drain(&self) -> io::Result<()> {
        loop {
            let changed = self.0.changed.notified();
            tokio::pin!(changed);
            changed.as_mut().enable();
            {
                let mut work = self
                    .0
                    .work
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner());
                if work.pending == 0 {
                    work.sealed = true;
                    return match &work.failure {
                        Some(message) => Err(io::Error::other(message.clone())),
                        None => Ok(()),
                    };
                }
            }
            changed.await;
        }
    }
}

impl CriticalTicket {
    /// Record completion. A failure remains recorded even if later work succeeds.
    pub fn complete<T>(&mut self, result: &io::Result<T>) {
        if let Err(error) = result {
            let mut work = self
                .work
                .0
                .work
                .lock()
                .unwrap_or_else(|poison| poison.into_inner());
            work.failure
                .get_or_insert_with(|| format!("{}: {error}", self.label));
        }
        self.completed = true;
    }
}

impl Drop for CriticalTicket {
    fn drop(&mut self) {
        let mut work = self
            .work
            .0
            .work
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        if !self.completed {
            work.failure
                .get_or_insert_with(|| format!("{} did not complete", self.label));
        }
        work.pending -= 1;
        drop(work);
        self.work.0.changed.notify_waiters();
    }
}

#[cfg(test)]
mod tests {
    use std::future::Future;
    use std::task::Poll;

    use super::*;

    const DEADLINE: std::time::Duration = std::time::Duration::from_secs(15);

    async fn bounded<T>(future: impl Future<Output = T>) -> T {
        tokio::time::timeout(DEADLINE, future)
            .await
            .expect("session work test stalled")
    }

    #[tokio::test]
    async fn empty_drain_closes_admission_and_repeated_drain_is_successful() {
        for _ in 0..2 {
            let work = SessionWork::default();
            assert!(!work.settled());
            bounded(work.drain()).await.unwrap();
            assert!(work.settled());
            assert_eq!(
                work.ticket("late work").err().unwrap().to_string(),
                "session work already drained"
            );
            assert!(work
                .spawn_blocking("late callback", || -> io::Result<()> {
                    panic!("sealed registry ran a callback")
                })
                .is_err());
            bounded(work.drain()).await.unwrap();
        }
    }

    #[tokio::test]
    async fn nested_work_remains_pending_after_its_parent_finishes() {
        let work = SessionWork::default();
        let mut parent = work.ticket("parent").unwrap();
        let mut child = work.ticket("child").unwrap();
        parent.complete(&Ok::<_, io::Error>(()));
        drop(parent);
        let drain = work.drain();
        tokio::pin!(drain);
        std::future::poll_fn(|cx| {
            assert!(drain.as_mut().poll(cx).is_pending());
            Poll::Ready(())
        })
        .await;
        assert!(!work.settled());
        child.complete(&Ok::<_, io::Error>(()));
        drop(child);
        bounded(drain).await.unwrap();
        assert!(work.settled());
    }

    #[tokio::test]
    async fn unfinished_and_panicked_work_remains_a_cleanup_failure() {
        let work = SessionWork::default();
        let task_work = work.clone();
        let failed = tokio::spawn(async move {
            let _ticket = task_work.ticket("panicked worker").unwrap();
            panic!("worker fault");
        });
        assert!(bounded(failed).await.unwrap_err().is_panic());
        assert_eq!(
            bounded(work.drain()).await.unwrap_err().to_string(),
            "panicked worker did not complete"
        );
        assert!(work.settled());
    }

    #[tokio::test]
    async fn the_first_error_survives_later_success_and_repeated_drains() {
        let work = SessionWork::default();
        let mut first = work.ticket("first").unwrap();
        let mut second = work.ticket("second").unwrap();
        first.complete(&Err::<(), _>(io::Error::other("write failed")));
        first.complete(&Ok::<_, io::Error>(()));
        second.complete(&Err::<(), _>(io::Error::other("sync failed")));
        drop((first, second));
        for _ in 0..2 {
            assert_eq!(
                bounded(work.drain()).await.unwrap_err().to_string(),
                "first: write failed"
            );
        }
    }

    #[tokio::test]
    async fn every_registered_drain_wakes_when_the_final_ticket_is_dropped() {
        let work = SessionWork::default();
        let mut ticket = work.ticket("last").unwrap();
        let first = work.drain();
        let second = work.drain();
        tokio::pin!(first, second);
        std::future::poll_fn(|cx| {
            assert!(first.as_mut().poll(cx).is_pending());
            assert!(second.as_mut().poll(cx).is_pending());
            Poll::Ready(())
        })
        .await;
        ticket.complete(&Ok::<_, io::Error>(()));
        drop(ticket);
        bounded(first).await.unwrap();
        bounded(second).await.unwrap();
    }

    #[test]
    fn a_full_pending_counter_refuses_before_registration() {
        let work = SessionWork::default();
        work.0.work.lock().unwrap().pending = usize::MAX;
        assert_eq!(
            work.ticket("overflow").err().unwrap().to_string(),
            "too many outstanding session tasks"
        );
        assert_eq!(work.0.work.lock().unwrap().pending, usize::MAX);
    }

    #[tokio::test]
    async fn dropped_blocking_wait_keeps_the_real_write_owned_until_completion() {
        let directory = tempfile::tempdir().unwrap();
        let output = directory.path().join("completed");
        let target = output.clone();
        let work = SessionWork::default();
        let (entered, started) = tokio::sync::oneshot::channel();
        let (release, held) = std::sync::mpsc::channel();
        let blocking = work
            .spawn_blocking("held real write", move || {
                let _ = entered.send(());
                held.recv_timeout(DEADLINE).map_err(io::Error::other)?;
                std::fs::write(target, b"complete")
            })
            .unwrap();
        bounded(started).await.unwrap();
        drop(blocking);
        let drain = work.drain();
        tokio::pin!(drain);
        std::future::poll_fn(|cx| {
            assert!(drain.as_mut().poll(cx).is_pending());
            Poll::Ready(())
        })
        .await;
        assert!(!output.exists());
        release.send(()).unwrap();
        bounded(drain).await.unwrap();
        assert_eq!(std::fs::read(output).unwrap(), b"complete");
        assert!(work.settled());
    }
}
