// SPDX-License-Identifier: AGPL-3.0-only
//! Flood-suppression latch for the `OutputProxy` discard-error path.
//!
//! Model + rationale mirror the pre-step-drain `DrainWarnLatch`
//! (`graph/drain_latch.rs`): a pure, transport-free state machine so
//! the suppression policy is unit-testable without any publisher / SHM mock.
//!
//! # Why
//!
//! `OutputProxy::Drop` emits a loud `tracing::error!` when a tick releases its
//! loan without writing all declared variable fields (or a staged nested-field
//! flush fails). That first signal is loud by design, so a silently
//! broken dylib node is visible. But a PERSISTENTLY broken node re-emits it on
//! EVERY publish: a measured 875 errors/s/graph (~148 MB of log per leg on a
//! long-running broken node). This latch keeps the FIRST occurrence loud
//! (`error!`, same message + structured fields: the loud-first-signal contract),
//! downgrades SUSTAINED occurrences to `debug!` carrying a running suppressed
//! count, and reports RECOVERY (a subsequent COMPLETE publish on the port) once
//! at `info!` before re-arming, so a fresh breakage is loud again. A
//! lone-discard regime (a single loud `error!` that heals on the very next
//! publish) re-arms SILENTLY — there was no flood to announce the end of, and a
//! recovery `info!` there would just double the log volume of a node flapping
//! every other tick.
//!
//! # State machine
//!
//! ```text
//!             on_discard() -> Error
//!   Healthy ─────────────────────────► Failing (suppressed = 0)
//!      ▲                                  │  on_discard() -> Debug { suppressed += 1 }
//!      │ on_complete() -> recovery        │    (repeat discards downgraded)
//!      │   Some(total) iff total > 0      │
//!      │   (caller logs recovery @info),  │
//!      │   else None (silent re-arm)      │
//!      └──────────────────────────────────┘
//!
//!   Healthy + on_complete() -> None   (steady state: branch-only, no log)
//! ```
//!
//! One latch per `(node_id, output port)` — it lives on the per-port
//! [`CerulionPublisher`](crate::transport::publisher::CerulionPublisher), so
//! there is no key lookup and no lock on the publish path. The happy-path check
//! [`OutputDiscardLatch::on_complete`] on a healthy port is a single
//! predictable branch returning `None` (no alloc, no lock — hot-path
//! discipline). The `error!`/`debug!`/`info!` events themselves are constructed
//! only on the cold discard / recovery transitions.

/// Level the caller should log an output-discard at, as decided by
/// [`OutputDiscardLatch::on_discard`].
///
/// A typed enum (rather than a bare bool) so call sites read as
/// `DiscardLogLevel::Error` / `Debug { .. }` instead of an easily-inverted
/// `already_suppressed` flag — the same inversion-regression surface the
/// sibling [`DrainFailureLevel`](crate::graph::drain_latch::DrainFailureLevel)
/// exists to pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiscardLogLevel {
    /// First discard of a regime: log at `tracing::error!` — the loud,
    /// by-design first signal (same message + structured fields).
    Error,
    /// A repeated discard while the latch is set: log at `tracing::debug!`,
    /// carrying the count of discards SUPPRESSED (downgraded) so far in this
    /// regime (the loud first one is not counted — it was not suppressed).
    Debug {
        /// Number of discards downgraded to `debug!` since the regime opened.
        suppressed: u64,
    },
}

/// Pure flood-suppression state machine for per-port `OutputProxy` discards.
/// One latch per `(node_id, output port)` (see module docs).
///
/// Holds no transport state and emits no logs itself — callers map the returned
/// decisions onto `tracing` events (see `OutputProxy::drop`).
#[derive(Debug, Default)]
pub struct OutputDiscardLatch {
    /// True while in the Failing state (≥1 discard since the last complete
    /// publish).
    failing: bool,
    /// Count of discards downgraded to `debug!` since the regime opened. The
    /// FIRST (loud `error!`) discard is NOT counted — it was not suppressed.
    /// Reset to 0 at each regime open and at recovery.
    suppressed: u64,
    /// UNCONDITIONAL running total of discards on this port across ALL regimes
    /// — bumped on EVERY [`on_discard`](Self::on_discard) regardless of state,
    /// NEVER reset on recovery. This is the Principle #3 queryability signal: a
    /// persistently-broken node whose loud head `error!` has scrolled away and
    /// whose sustained discards are `debug!`-suppressed is otherwise invisible
    /// and uncountable. Surfaced through
    /// [`CerulionPublisher::output_discard_count`](crate::transport::publisher::CerulionPublisher::output_discard_count)
    /// (the node's own tick code, via `AnyPublisher`) AND — for an off-thread
    /// OPERATOR (Principle #3) — through the per-output
    /// `NodeHandle::output_discard_count` accessor, which the graph runtime wires
    /// to the SAME count. This makes it symmetric with the
    /// unconditional `backpressure_{drop_oldest,block_fires_deferred,sampled}_count`
    /// counters this same branch keeps. A `u64` increment on the (cold) discard
    /// path only — the happy/complete path pays nothing new.
    total_discards: u64,
}

impl OutputDiscardLatch {
    /// Construct a latch in the Healthy state.
    pub const fn new() -> Self {
        Self {
            failing: false,
            suppressed: 0,
            total_discards: 0,
        }
    }

    /// Record an output discard; returns the level the caller should log it at.
    /// First discard of a regime → [`DiscardLogLevel::Error`] (the latch sets);
    /// repeats → [`DiscardLogLevel::Debug`] carrying the running suppressed
    /// count.
    #[inline]
    pub fn on_discard(&mut self) -> DiscardLogLevel {
        // UNCONDITIONAL: every discard bumps the queryable total, independent of
        // the log-level regime (Principle #3 — a `debug!`-suppressed sustained
        // discard is still counted here). Cold path only.
        self.total_discards += 1;
        if self.failing {
            self.suppressed += 1;
            DiscardLogLevel::Debug {
                suppressed: self.suppressed,
            }
        } else {
            self.failing = true;
            self.suppressed = 0;
            DiscardLogLevel::Error
        }
    }

    /// Record a COMPLETE publish; always re-arms the loud path (a subsequent
    /// discard is `Error` again). Returns `Some(total_suppressed)` — the caller
    /// logs recovery at `info!` once — ONLY when the closed regime actually
    /// SUPPRESSED at least one discard (`suppressed > 0`).
    ///
    /// A lone-discard regime (one `error!`, immediate recovery, `suppressed ==
    /// 0`) re-arms SILENTLY (returns `None`): the single error already told the
    /// whole story and there was no flood to announce the end of — reporting
    /// recovery there would DOUBLE the log volume of a node that flaps every
    /// other tick (error + info per cycle). Steady-state healthy publishes also
    /// return `None`, so the hot path stays a single predictable branch (no
    /// alloc, no lock).
    #[inline]
    pub fn on_complete(&mut self) -> Option<u64> {
        if self.failing {
            self.failing = false;
            let total = self.suppressed;
            self.suppressed = 0;
            // Report recovery only when the regime actually suppressed
            // (flood-)downgraded discards; a lone-error regime re-arms silently.
            (total > 0).then_some(total)
        } else {
            None
        }
    }

    /// Returns `true` while the latch is in the Failing state.
    ///
    /// A test-oracle accessor for this pure state machine (used by the
    /// oracle-vector unit + integration tests to assert the latch transitions
    /// independently of the decision stream). Production does NOT surface this
    /// — the `OutputProxy` discard/recovery path drives the latch purely
    /// through [`on_discard`](Self::on_discard) / [`on_complete`](Self::on_complete).
    pub fn is_failing(&self) -> bool {
        self.failing
    }

    /// UNCONDITIONAL running total of discards recorded on this port across all
    /// regimes (never reset on recovery). See the `total_discards` field docs —
    /// the queryable Principle #3 signal for a persistently-broken node whose
    /// sustained discards are `debug!`-suppressed. Surfaced through
    /// [`CerulionPublisher::output_discard_count`](crate::transport::publisher::CerulionPublisher::output_discard_count)
    /// (node-local) and the per-output `NodeHandle::output_discard_count`
    /// off-thread operator accessor.
    pub fn total_discards(&self) -> u64 {
        self.total_discards
    }
}

// ---------------------------------------------------------------------------
// This module's half of the ABI LAYOUT PIN (see `crate::abi_layout`).
//
// `abi_pin_struct!` expands to an exhaustive destructuring pattern with no `..`
// rest pattern, so adding or removing a field of one of these structs is a
// COMPILE ERROR naming the struct and the field; it also measures
// size/align/`offset_of!`, which `crate::abi_layout` compares against the
// snapshot table keyed to `CERULION_ABI_VERSION`. `abi_pin_enum!` does the
// same for a variant set (an enum carries no stable field offsets).
// ---------------------------------------------------------------------------
#[cfg(test)]
pub(crate) fn abi_layout_pins() -> Vec<crate::abi_layout::MeasuredStruct> {
    use crate::abi_layout::abi_pin_struct;
    vec![abi_pin_struct!(OutputDiscardLatch {
        failing,
        suppressed,
        total_discards
    })]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Happy path: first discard errors, repeats downgrade to debug with a
    /// running count, recovery reports the total suppressed once, next discard
    /// errors again.
    #[test]
    fn full_cycle_error_debug_recover_error() {
        let mut latch = OutputDiscardLatch::new();
        assert!(!latch.is_failing());

        assert_eq!(latch.on_discard(), DiscardLogLevel::Error);
        assert!(latch.is_failing());
        assert_eq!(latch.on_discard(), DiscardLogLevel::Debug { suppressed: 1 });
        assert_eq!(latch.on_discard(), DiscardLogLevel::Debug { suppressed: 2 });

        // Recovery reports the two suppressed (debug-downgraded) discards.
        assert_eq!(latch.on_complete(), Some(2));
        assert!(!latch.is_failing());

        // Latch cleared: the next discard must ERROR again (catches the
        // never-error-again / no-rearm inversion).
        assert_eq!(latch.on_discard(), DiscardLogLevel::Error);
    }

    /// Inversion regression: the FIRST discard must be Error, never Debug
    /// (catches a never-error implementation).
    #[test]
    fn first_discard_is_error_not_debug() {
        let mut latch = OutputDiscardLatch::new();
        assert_eq!(latch.on_discard(), DiscardLogLevel::Error);
    }

    /// Inversion regression: the SECOND consecutive discard must be Debug,
    /// never Error (catches an always-error implementation that floods).
    #[test]
    fn repeat_discard_is_debug_not_error() {
        let mut latch = OutputDiscardLatch::new();
        let _ = latch.on_discard();
        assert_eq!(latch.on_discard(), DiscardLogLevel::Debug { suppressed: 1 });
    }

    /// Steady-state complete publishes are silent: no recovery when there was
    /// nothing to recover from.
    #[test]
    fn complete_while_healthy_reports_no_recovery() {
        let mut latch = OutputDiscardLatch::new();
        assert_eq!(latch.on_complete(), None);
        assert_eq!(latch.on_complete(), None);
        assert!(!latch.is_failing());
    }

    /// Recovery reports exactly ONCE per multi-discard regime — a second
    /// complete publish immediately after returns None.
    #[test]
    fn recovery_reports_once_per_regime() {
        let mut latch = OutputDiscardLatch::new();
        let _ = latch.on_discard(); // Error
        let _ = latch.on_discard(); // Debug{1} — suppressed > 0 so recovery reports
        assert_eq!(latch.on_complete(), Some(1));
        assert_eq!(
            latch.on_complete(),
            None,
            "second complete must not re-report"
        );
    }

    /// Flapping amplification: a LONE-discard regime (one loud error,
    /// zero suppressed) recovers SILENTLY (`None`) — the single error already
    /// told the story — but STILL re-arms, so the next discard errors again.
    #[test]
    fn single_discard_regime_recovers_silently_but_rearms() {
        let mut latch = OutputDiscardLatch::new();
        assert_eq!(latch.on_discard(), DiscardLogLevel::Error);
        assert_eq!(
            latch.on_complete(),
            None,
            "a lone-error regime re-arms without a recovery info! (no flood to close)"
        );
        assert!(
            !latch.is_failing(),
            "the latch cleared even though it stayed silent"
        );
        // Re-armed: the next discard must ERROR (not stay latched at Debug).
        assert_eq!(latch.on_discard(), DiscardLogLevel::Error);
    }

    /// The suppressed count resets across regimes (a new regime after recovery
    /// starts its debug count at 1, not continuing the prior regime's total).
    #[test]
    fn suppressed_count_resets_across_regimes() {
        let mut latch = OutputDiscardLatch::new();
        let _ = latch.on_discard(); // Error, regime 1
        let _ = latch.on_discard(); // Debug{1}
        let _ = latch.on_discard(); // Debug{2}
        assert_eq!(latch.on_complete(), Some(2));

        let _ = latch.on_discard(); // Error, regime 2
        assert_eq!(
            latch.on_discard(),
            DiscardLogLevel::Debug { suppressed: 1 },
            "a fresh regime restarts the suppressed count at 1"
        );
    }

    /// Default and `new()` agree (the publisher constructs via `new()`;
    /// `#[derive(Default)]` exists for struct-update ergonomics).
    #[test]
    fn default_is_healthy() {
        assert!(!OutputDiscardLatch::default().is_failing());
    }

    /// Principle #3 queryability: the unconditional `total_discards`
    /// counter bumps on EVERY discard regardless of the log-level regime
    /// (error head + debug-suppressed sustained), NEVER resets on recovery, and
    /// is untouched by complete publishes.
    #[test]
    fn total_discards_counts_every_discard_across_regimes_and_never_resets() {
        let mut latch = OutputDiscardLatch::new();
        assert_eq!(latch.total_discards(), 0);

        // Complete publishes on a healthy port do not bump the total.
        assert_eq!(latch.on_complete(), None);
        assert_eq!(latch.total_discards(), 0);

        // Regime 1: 3 discards (Error + 2 Debug). Counter == 3.
        let _ = latch.on_discard();
        let _ = latch.on_discard();
        let _ = latch.on_discard();
        assert_eq!(latch.total_discards(), 3);

        // Recovery must NOT reset the total (it only resets `suppressed`).
        assert_eq!(latch.on_complete(), Some(2));
        assert_eq!(
            latch.total_discards(),
            3,
            "recovery resets the per-regime suppressed count, NOT the lifetime total"
        );

        // Regime 2: 2 more discards. Counter accumulates to 5.
        let _ = latch.on_discard();
        let _ = latch.on_discard();
        assert_eq!(latch.total_discards(), 5);
    }
}
