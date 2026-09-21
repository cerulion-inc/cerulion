// SPDX-License-Identifier: AGPL-3.0-only
//! Warn-suppression latch for the pre-step drain hot path.
//!
//! Extracted from `runtime.rs`'s `DataTriggerBinding::warn_suppressed`
//! bool into a pure, transport-free
//! state machine so the suppression policy is unit-testable without
//! any subscriber mock.
//!
//! # State machine
//!
//! ```text
//!            on_failure() -> Warn
//!   Healthy ───────────────────────► Failing
//!      ▲                                │
//!      │ on_success() -> true           │ on_failure() -> Debug
//!      │   (caller logs recovery        │   (repeat failures are
//!      │    at info)                    │    downgraded)
//!      └────────────────────────────────┘
//!
//!   Healthy + on_success() -> false  (steady state, no log)
//! ```
//!
//! The FIRST failure in a run logs at `warn` (operators always see the
//! diagnostic); repeated identical failures downgrade to `debug` so a
//! persistently-broken transport at a 1 kHz schedule doesn't flood
//! stdout/journald. A successful drain clears the latch — the caller
//! logs the recovery at `info` exactly once — and the NEXT failure
//! warns again.

/// Log level the caller should use for a drain failure, as decided by
/// [`DrainWarnLatch::on_failure`].
///
/// A two-variant enum (rather than a bare bool) so call sites read as
/// `DrainFailureLevel::Warn` / `Debug` instead of an easily-inverted
/// `already_suppressed` flag — the earlier bool-parameter shape is
/// exactly the inversion-regression surface this module exists to pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DrainFailureLevel {
    /// First failure of a run: log at `tracing::warn!`.
    Warn,
    /// Repeated failure while the latch is set: log at `tracing::debug!`.
    Debug,
}

/// Pure suppression-policy state machine for per-binding drain
/// failures. One latch per `DataTriggerBinding`.
///
/// Holds no transport state and emits no logs itself — callers map
/// the returned decisions onto `tracing` events (see
/// `runtime.rs::log_drain_failure` / `log_drain_recovery`).
#[derive(Debug, Default)]
pub struct DrainWarnLatch {
    /// True while in the Failing state (≥1 failure since the last
    /// successful drain).
    failing: bool,
}

impl DrainWarnLatch {
    /// Construct a latch in the Healthy state.
    pub const fn new() -> Self {
        Self { failing: false }
    }

    /// Record a drain failure; returns the level the caller should log
    /// this failure at. First failure of a run → [`DrainFailureLevel::Warn`]
    /// and the latch sets; repeats → [`DrainFailureLevel::Debug`].
    pub fn on_failure(&mut self) -> DrainFailureLevel {
        if self.failing {
            DrainFailureLevel::Debug
        } else {
            self.failing = true;
            DrainFailureLevel::Warn
        }
    }

    /// Record a successful drain; returns `true` exactly when this
    /// success ends a failure run (the caller should log the recovery
    /// at `info` once). Steady-state successes return `false` so the
    /// hot path stays log-free.
    pub fn on_success(&mut self) -> bool {
        std::mem::replace(&mut self.failing, false)
    }

    /// Returns `true` while the latch is in the Failing state
    /// (observable state — Principle #3).
    pub fn is_failing(&self) -> bool {
        self.failing
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Happy path: first failure warns, repeats downgrade to debug,
    /// recovery is reported once, next failure warns again.
    #[test]
    fn full_cycle_warn_debug_recover_warn() {
        let mut latch = DrainWarnLatch::new();
        assert!(!latch.is_failing());

        assert_eq!(latch.on_failure(), DrainFailureLevel::Warn);
        assert!(latch.is_failing());
        assert_eq!(latch.on_failure(), DrainFailureLevel::Debug);
        assert_eq!(latch.on_failure(), DrainFailureLevel::Debug);

        assert!(
            latch.on_success(),
            "first success after failures = recovery"
        );
        assert!(!latch.is_failing());

        // Latch cleared: the next failure must WARN again (catches the
        // no-recovery / never-warn-again inversion).
        assert_eq!(latch.on_failure(), DrainFailureLevel::Warn);
    }

    /// Inversion regression: the FIRST failure must be Warn, never
    /// Debug (catches a never-warn implementation).
    #[test]
    fn first_failure_is_warn_not_debug() {
        let mut latch = DrainWarnLatch::new();
        assert_eq!(latch.on_failure(), DrainFailureLevel::Warn);
    }

    /// Inversion regression: the SECOND consecutive failure must be
    /// Debug, never Warn (catches an always-warn implementation that
    /// would flood logs at 1 kHz).
    #[test]
    fn repeat_failure_is_debug_not_warn() {
        let mut latch = DrainWarnLatch::new();
        let _ = latch.on_failure();
        assert_eq!(latch.on_failure(), DrainFailureLevel::Debug);
    }

    /// Steady-state successes are silent: no recovery report when
    /// there was nothing to recover from.
    #[test]
    fn success_while_healthy_reports_no_recovery() {
        let mut latch = DrainWarnLatch::new();
        assert!(!latch.on_success());
        assert!(!latch.on_success());
        assert!(!latch.is_failing());
    }

    /// Recovery is reported exactly ONCE per failure run — a second
    /// success immediately after returns false.
    #[test]
    fn recovery_reports_once_per_failure_run() {
        let mut latch = DrainWarnLatch::new();
        let _ = latch.on_failure();
        assert!(latch.on_success());
        assert!(!latch.on_success(), "second success must not re-report");
    }

    /// Default and `new()` agree (the runtime constructs via `new()`;
    /// `#[derive(Default)]` exists for struct-update ergonomics).
    #[test]
    fn default_is_healthy() {
        assert!(!DrainWarnLatch::default().is_failing());
    }
}
