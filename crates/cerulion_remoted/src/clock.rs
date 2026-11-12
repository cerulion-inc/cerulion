// SPDX-License-Identifier: AGPL-3.0-only
//! The robot's TRUSTED clock for pairing verification.
//!
//! Certificate validity and the anti-rollback high-water mark are both
//! evaluated against `now_ns`. That time MUST come from the robot's own
//! trusted clock — NEVER from client-controlled request arguments (a client that
//! could set `now_ns` would trivially bypass anti-rollback and cert expiry). This
//! type is the clock seam: production reads the wall clock; tests inject a fixed,
//! advanceable clock so the pairing matrices are deterministic (Principle
//! #7) without ever exposing the time to the wire.
//!
//! [`MonotonicClock`] is the seam for windows a step of the wall clock must
//! neither stretch nor cut, such as an armed pairing code's: production reads
//! the process's monotonic clock, tests inject a fixed, advanceable instant.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, OnceLock};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

/// A trusted time source. Cloneable; a [`RemotedClock::Fixed`] clone shares the
/// same advanceable instant (so a test can move the clock forward mid-ceremony).
#[derive(Clone)]
pub enum RemotedClock {
    /// The robot wall clock (nanoseconds since the Unix epoch). Production.
    Wall,
    /// A fixed, advanceable clock (tests only) — the shared instant behind an
    /// `Arc<AtomicU64>` so a test can [`RemotedClock::set`] it forward.
    Fixed(Arc<AtomicU64>),
}

impl std::fmt::Debug for RemotedClock {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RemotedClock::Wall => f.write_str("RemotedClock::Wall"),
            RemotedClock::Fixed(a) => {
                write!(f, "RemotedClock::Fixed({})", a.load(Ordering::SeqCst))
            }
        }
    }
}

impl RemotedClock {
    /// The production wall clock.
    pub fn wall() -> Self {
        RemotedClock::Wall
    }

    /// A fixed clock pinned at `now_ns` (tests). Advance it with [`RemotedClock::set`].
    pub fn fixed(now_ns: u64) -> Self {
        RemotedClock::Fixed(Arc::new(AtomicU64::new(now_ns)))
    }

    /// The current trusted time (nanoseconds since the Unix epoch). A pre-epoch
    /// wall clock saturates to 0.
    pub fn now_ns(&self) -> u64 {
        match self {
            RemotedClock::Wall => SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_nanos() as u64)
                .unwrap_or(0),
            RemotedClock::Fixed(a) => a.load(Ordering::SeqCst),
        }
    }

    /// Set a [`RemotedClock::Fixed`] clock to `now_ns` (tests). A no-op on
    /// [`RemotedClock::Wall`] — the wall clock cannot be set.
    pub fn set(&self, now_ns: u64) {
        if let RemotedClock::Fixed(a) = self {
            a.store(now_ns, Ordering::SeqCst);
        }
    }
}

/// A monotonic time source in nanoseconds. Cloneable; a
/// [`MonotonicClock::Fixed`] clone shares the same advanceable instant.
#[derive(Clone)]
pub enum MonotonicClock {
    /// The process's monotonic clock ([`Instant`]), counted from its first
    /// reading in this process. A step of the wall clock does not move it.
    /// On Linux and macOS it does not count time the system spends suspended
    /// (std's `Instant`), so a suspend lengthens a window measured on it by
    /// the time suspended. Production.
    Process,
    /// A fixed, advanceable instant (tests only), set with
    /// [`MonotonicClock::set`].
    Fixed(Arc<AtomicU64>),
}

impl std::fmt::Debug for MonotonicClock {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MonotonicClock::Process => f.write_str("MonotonicClock::Process"),
            MonotonicClock::Fixed(a) => {
                write!(f, "MonotonicClock::Fixed({})", a.load(Ordering::SeqCst))
            }
        }
    }
}

impl MonotonicClock {
    /// A fixed clock pinned at `now_ns` (tests). Advance it with
    /// [`MonotonicClock::set`].
    pub fn fixed(now_ns: u64) -> Self {
        MonotonicClock::Fixed(Arc::new(AtomicU64::new(now_ns)))
    }

    /// The current monotonic time in nanoseconds.
    pub fn now_ns(&self) -> u64 {
        match self {
            MonotonicClock::Process => {
                static ORIGIN: OnceLock<Instant> = OnceLock::new();
                let origin = *ORIGIN.get_or_init(Instant::now);
                Instant::now().saturating_duration_since(origin).as_nanos() as u64
            }
            MonotonicClock::Fixed(a) => a.load(Ordering::SeqCst),
        }
    }

    /// Set a [`MonotonicClock::Fixed`] clock to `now_ns` (tests). A no-op on
    /// [`MonotonicClock::Process`].
    pub fn set(&self, now_ns: u64) {
        if let MonotonicClock::Fixed(a) = self {
            a.store(now_ns, Ordering::SeqCst);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_process_clock_never_goes_back_and_a_fixed_one_reads_what_it_was_set_to() {
        let process = MonotonicClock::Process;
        let first = process.now_ns();
        let second = process.now_ns();
        assert!(second >= first, "{first} then {second}");
        process.set(0);
        assert!(
            process.now_ns() >= second,
            "set is a no-op on the process clock"
        );

        let fixed = MonotonicClock::fixed(7);
        let shared = fixed.clone();
        assert_eq!(fixed.now_ns(), 7);
        shared.set(9);
        assert_eq!(fixed.now_ns(), 9, "a clone shares the instant");
        assert_eq!(format!("{fixed:?}"), "MonotonicClock::Fixed(9)");
        assert_eq!(format!("{process:?}"), "MonotonicClock::Process");
    }
}
