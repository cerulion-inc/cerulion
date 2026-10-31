// SPDX-License-Identifier: AGPL-3.0-only
//! Multi-threaded executor detection.
//!
//! A replay serves one process's deliveries in the order ONE executor thread
//! consumed them. A multi-threaded executor runs callbacks concurrently, so no
//! recorded order describes it, and it is refused by name rather than replayed
//! wrong. Each executor kind leaves its own signature at the rmw:
//!
//! - **rclcpp `MultiThreadedExecutor`**: its threads take turns calling
//!   `rmw_wait` on the ONE wait set the executor owns, so a second thread
//!   waiting on a wait set another thread waited on is the mark.
//! - **rclpy `MultiThreadedExecutor`**: one thread waits and the callbacks run
//!   on a thread pool, so the takes come from a thread that never called
//!   `rmw_wait` while a waiting thread exists.
//!
//! Separate single-threaded executors on separate threads each own their own
//! wait set and take on their own thread, so neither mark fires for them.

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

/// The per-process detector state.
#[derive(Default)]
pub(crate) struct Detector {
    /// Some thread has called `rmw_wait`.
    any_waiter: AtomicBool,
}

impl Detector {
    /// A `rmw_wait` call on the wait set whose owner slot is `owner` (0 =
    /// never waited on; otherwise the waiting thread's index + 1), from the
    /// thread with index `thread`. Returns the detail when this call is a
    /// second thread waiting on one wait set.
    pub(crate) fn on_wait(&self, owner: &AtomicU32, thread: u32) -> Option<String> {
        self.any_waiter.store(true, Ordering::Relaxed);
        let me = thread.saturating_add(1);
        match owner.compare_exchange(0, me, Ordering::AcqRel, Ordering::Acquire) {
            Ok(_) => None,
            Err(prev) if prev == me => None,
            Err(prev) => Some(format!(
                "threads {} and {} both wait on one wait set (an rclcpp \
                 MultiThreadedExecutor shares its wait set between its threads)",
                prev - 1,
                thread
            )),
        }
    }

    /// A take from the thread with index `thread`; `has_waited` says whether
    /// that thread has ever called `rmw_wait`. Returns the detail when the take
    /// comes from a thread that never waited while another thread waits.
    pub(crate) fn on_take(&self, thread: u32, has_waited: bool) -> Option<String> {
        if has_waited || !self.any_waiter.load(Ordering::Relaxed) {
            return None;
        }
        Some(format!(
            "thread {thread} takes deliveries but never waits, while another thread waits \
             (an rclpy MultiThreadedExecutor runs its callbacks on a thread pool)"
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_thread_per_wait_set_is_never_flagged() {
        let d = Detector::default();
        let a = AtomicU32::new(0);
        let b = AtomicU32::new(0);
        for _ in 0..3 {
            assert_eq!(d.on_wait(&a, 0), None);
            assert_eq!(
                d.on_wait(&b, 1),
                None,
                "a second executor owns its own wait set"
            );
        }
        assert_eq!(d.on_take(0, true), None);
        assert_eq!(d.on_take(1, true), None);
    }

    #[test]
    fn two_threads_on_one_wait_set_is_the_rclcpp_mark() {
        let d = Detector::default();
        let ws = AtomicU32::new(0);
        assert_eq!(d.on_wait(&ws, 0), None);
        let detail = d.on_wait(&ws, 3).expect("second waiter flagged");
        assert!(detail.contains("rclcpp MultiThreadedExecutor"), "{detail}");
        assert!(detail.contains("threads 0 and 3"), "{detail}");
    }

    #[test]
    fn a_take_from_a_thread_that_never_waits_is_the_rclpy_mark() {
        let d = Detector::default();
        // Before anything waits, a take (construction-time) is not flagged.
        assert_eq!(d.on_take(5, false), None);
        let ws = AtomicU32::new(0);
        d.on_wait(&ws, 0);
        let detail = d.on_take(5, false).expect("pool-thread take flagged");
        assert!(detail.contains("rclpy MultiThreadedExecutor"), "{detail}");
        assert_eq!(
            d.on_take(0, true),
            None,
            "the waiting thread's own take is fine"
        );
    }
}
