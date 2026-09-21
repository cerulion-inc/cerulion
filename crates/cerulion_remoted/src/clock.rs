// SPDX-License-Identifier: AGPL-3.0-only
//! The robot's TRUSTED clock for pairing verification.
//!
//! Certificate validity, the anti-rollback high-water mark, and the CPace TTL are
//! all evaluated against `now_ns`. That time MUST come from the robot's own
//! trusted clock — NEVER from client-controlled request arguments (a client that
//! could set `now_ns` would trivially bypass anti-rollback and cert expiry). This
//! type is the clock seam: production reads the wall clock; tests inject a fixed,
//! advanceable clock so the pairing + TTL matrices are deterministic (Principle
//! #7) without ever exposing the time to the wire.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

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
