// SPDX-License-Identifier: AGPL-3.0-only
//! Flood-suppression latch for per-node tick-failure logging.
//!
//! The `DrainWarnLatch` sibling (see [`super::drain_latch`]): a pure,
//! transport-free state machine deciding the log level for each tick
//! failure, so a permanently-broken node at a 1 kHz schedule (observed:
//! ~2,660 identical ERROR lines in a 3 s replay of a poisoned cdylib)
//! floods neither stdout nor journald, while every DISTINCT failure
//! regime still surfaces loudly.
//!
//! # State machine (per node — the latch lives in the node's tick callback)
//!
//! ```text
//! occurrence 1 of a reason R          → FirstOfRegime   (error!, full)
//! occurrence 2 of the SAME reason R   → AnnounceSuppression (error!,
//!                                        "suppressing further ... (N so far)")
//! occurrence ≥3 of the SAME reason R  → Suppressed      (debug!, count)
//! reason CHANGES (R → R')             → FirstOfRegime   (error!, full, carries
//!                                        the prior regime's suppressed count)
//! tick succeeds after ≥1 failure      → recovery        (info!, once; latch
//!                                        resets — the NEXT failure errors again)
//! ```
//!
//! At most TWO `error!` lines per `(node, reason)` regime — the first (the
//! diagnostic) and the second (the suppression announcement, so an operator
//! reading the log knows why the stream went quiet). The regime key is the
//! rendered reason string: a cdylib panic (FFI code 2, "panic caught by
//! catch_unwind") transitioning to the poisoned-dead state (FFI code 3,
//! "NODES mutex poisoned") is a reason CHANGE and re-fires `error!` once.
//!
//! Holds no transport state and emits no logs itself — the caller maps the
//! returned [`TickFailureLog`] onto `tracing` events (see
//! `runtime.rs::log_tick_failure` / `log_tick_failure_recovery`, pinned by
//! `#[traced_test]` there — the `DrainWarnLatch` precedent).

/// Log decision for one tick failure, as decided by
/// [`TickFailureLatch::on_failure`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TickFailureLog {
    /// First failure of a (new) reason regime: log at `tracing::error!` in
    /// full. `prior_suppressed` is the number of repeats suppressed in the
    /// PREVIOUS regime (0 when this is the node's first failure ever, or the
    /// prior regime never repeated) — rendered so the regime change also
    /// accounts for what went quiet before it.
    FirstOfRegime { prior_suppressed: u64 },
    /// Second failure of the SAME regime: log at `tracing::error!` once,
    /// announcing that further identical failures are suppressed to debug.
    AnnounceSuppression,
    /// Third-or-later failure of the same regime: log at `tracing::debug!`
    /// with the running suppressed count (`occurrences - 1`).
    Suppressed { suppressed_so_far: u64 },
}

/// Pure flood-suppression state machine for per-node tick failures. One
/// latch per node, owned by the node's tick callback closure.
#[derive(Debug, Default)]
pub struct TickFailureLatch {
    /// The active failure-regime key (`None` = healthy / recovered).
    last_reason: Option<String>,
    /// Failures observed in the active regime (1 = just the first).
    occurrences: u64,
}

impl TickFailureLatch {
    /// Construct a latch in the healthy state.
    pub const fn new() -> Self {
        Self {
            last_reason: None,
            occurrences: 0,
        }
    }

    /// Record a tick failure with the given rendered `reason`; returns the
    /// log decision for THIS occurrence.
    pub fn on_failure(&mut self, reason: &str) -> TickFailureLog {
        if self.last_reason.as_deref() != Some(reason) {
            // New regime (first failure ever, a reason change, or the first
            // failure after a recovery reset).
            let prior_suppressed = self.occurrences.saturating_sub(1);
            // hot-path-alloc-ok: cold in a sustained regime: this arm runs only when the failure
            // REASON changes (or after a recovery reset); a repeating failure re-uses the stored
            // reason and allocates nothing
            self.last_reason = Some(reason.to_string());
            self.occurrences = 1;
            return TickFailureLog::FirstOfRegime { prior_suppressed };
        }
        self.occurrences += 1;
        if self.occurrences == 2 {
            TickFailureLog::AnnounceSuppression
        } else {
            TickFailureLog::Suppressed {
                suppressed_so_far: self.occurrences - 1,
            }
        }
    }

    /// Record a successful tick. Returns `Some(total_suppressed)` exactly
    /// when the node RECOVERED from an active failure regime (the caller
    /// logs the recovery at `info!` once); `None` at steady healthy state.
    /// Resets the latch — the next failure is a fresh `FirstOfRegime`.
    pub fn on_success(&mut self) -> Option<u64> {
        // take() both TESTS the active regime and RESETS it (the healthy
        // steady state returns None without touching `occurrences`).
        self.last_reason.take()?;
        let suppressed = self.occurrences.saturating_sub(1);
        self.occurrences = 0;
        Some(suppressed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Oracle vector: the exact log-decision sequence for the observed
    /// production flood shape — a cdylib panic (regime A) followed by the
    /// endless poisoned-dead state (regime B). Pre-latch this was one
    /// `error!` PER TICK (~2,660 lines in 3 s); the oracle pins ≤2 errors
    /// per regime.
    #[test]
    fn panic_then_poisoned_flood_collapses_to_two_errors_per_regime() {
        let mut latch = TickFailureLatch::new();
        let panic_r = "panic caught by catch_unwind";
        let poison_r = "NODES mutex poisoned";
        let got: Vec<TickFailureLog> = [panic_r, poison_r, poison_r, poison_r, poison_r, poison_r]
            .iter()
            .map(|r| latch.on_failure(r))
            .collect();
        assert_eq!(
            got,
            vec![
                TickFailureLog::FirstOfRegime {
                    prior_suppressed: 0
                },
                // Reason change on occurrence 2 → a fresh regime, error! again.
                TickFailureLog::FirstOfRegime {
                    prior_suppressed: 0
                },
                TickFailureLog::AnnounceSuppression,
                TickFailureLog::Suppressed {
                    suppressed_so_far: 2
                },
                TickFailureLog::Suppressed {
                    suppressed_so_far: 3
                },
                TickFailureLog::Suppressed {
                    suppressed_so_far: 4
                },
            ]
        );
    }

    /// A steady identical failure stream: exactly one FirstOfRegime + one
    /// AnnounceSuppression, then debug-only forever (the flood kill).
    #[test]
    fn identical_failures_log_at_most_two_errors() {
        let mut latch = TickFailureLatch::new();
        let r = "tick failed: sensor offline";
        assert_eq!(
            latch.on_failure(r),
            TickFailureLog::FirstOfRegime {
                prior_suppressed: 0
            }
        );
        assert_eq!(latch.on_failure(r), TickFailureLog::AnnounceSuppression);
        for k in 3..100u64 {
            assert_eq!(
                latch.on_failure(r),
                TickFailureLog::Suppressed {
                    suppressed_so_far: k - 1
                },
                "occurrence {k} must stay suppressed"
            );
        }
    }

    /// Recovery resets the latch: Ok after failures reports the suppressed
    /// total ONCE, and the next failure (even with the SAME reason) is a
    /// fresh loud FirstOfRegime — the DrainWarnLatch warn→debug→info→warn
    /// cycle, at error level.
    #[test]
    fn recovery_resets_and_rearms_the_error() {
        let mut latch = TickFailureLatch::new();
        let r = "transient failure";
        latch.on_failure(r);
        latch.on_failure(r);
        latch.on_failure(r); // 3 occurrences → 2 suppressed
        assert_eq!(latch.on_success(), Some(2), "recovery reports the total");
        assert_eq!(latch.on_success(), None, "steady healthy state is silent");
        assert_eq!(
            latch.on_failure(r),
            TickFailureLog::FirstOfRegime {
                prior_suppressed: 0
            },
            "post-recovery failure re-fires error! even for the same reason"
        );
    }

    /// A regime change AFTER suppression carries the prior regime's
    /// suppressed count into the new FirstOfRegime (the log accounts for
    /// what went quiet).
    #[test]
    fn regime_change_reports_prior_suppressed_count() {
        let mut latch = TickFailureLatch::new();
        for _ in 0..5 {
            latch.on_failure("reason A"); // 5 occurrences → 4 suppressed
        }
        assert_eq!(
            latch.on_failure("reason B"),
            TickFailureLog::FirstOfRegime {
                prior_suppressed: 4
            }
        );
    }

    /// Healthy-state success never reports recovery (no log).
    #[test]
    fn healthy_success_is_silent() {
        let mut latch = TickFailureLatch::new();
        assert_eq!(latch.on_success(), None);
        assert_eq!(latch.on_success(), None);
    }

    /// Inversion regressions: an always-error latch (never suppresses) or a
    /// never-error latch (suppresses the first) both fail these pins.
    #[test]
    fn first_is_loud_and_third_is_quiet() {
        let mut latch = TickFailureLatch::new();
        let r = "r";
        // Never-error inversion: the FIRST must be FirstOfRegime.
        assert!(matches!(
            latch.on_failure(r),
            TickFailureLog::FirstOfRegime { .. }
        ));
        latch.on_failure(r);
        // Always-error inversion: the THIRD must be Suppressed.
        assert!(matches!(
            latch.on_failure(r),
            TickFailureLog::Suppressed { .. }
        ));
    }
}
