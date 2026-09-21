// SPDX-License-Identifier: AGPL-3.0-only
//! Oracle-vector tests for the `OutputProxy` discard flood-suppression
//! latch.
//!
//! The policy lives in the pure `OutputDiscardLatch` state machine
//! (`cerulion_core/src/transport/output_discard_latch.rs`), extracted so it is
//! testable without any publisher / iceoryx2 mock — the sibling of its
//! `DrainWarnLatch`. Contract:
//!
//! - FIRST discard of a regime → `error!` (the loud-by-design first
//!   signal; operators always see the diagnostic)
//! - repeated discards → downgrade to `debug!` carrying a running SUPPRESSED
//!   count (no flooding — an unlatched broken node emits ~875 discard errors/s/graph,
//!   ~148 MB of log per leg)
//! - recovery (first COMPLETE publish after discards) → the latch clears and,
//!   ONLY when the regime actually suppressed ≥1 discard, reports the total
//!   suppressed count once at `info!`; a lone-error regime re-arms silently
//!   (no flood to close, so no doubled log volume on an every-other-tick
//!   flapper)
//! - next discard after recovery → `error!` again (re-armed) whether or not the
//!   recovery logged
//!
//! The mapping of these decisions onto actual `tracing` events (error/debug at
//! the two `OutputProxy::drop` discard sites, info at the send-success site) is
//! pinned end-to-end by the cdylib-local stderr subprocess test
//! `cdylib_tracing_stopgap_test.rs` (the loud-first + debug-suppressed arms).
//! RECOVERY cannot be exercised there (the fixture's `CER_DISCARD_MODE` is fixed
//! per run — it never heals), so the recovery + re-arm contract is pinned HERE
//! against hand-written oracle vectors (NOT a two-run self-compare, which
//! would be tautological).
//!
//! No iceoryx2 dependency — parallel-safe, no `--test-threads=1` requirement.

use cerulion_core::transport::output_discard_latch::{DiscardLogLevel, OutputDiscardLatch};

/// Scripted per-publish outcome for the oracle-vector tests.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Event {
    /// The tick released its loan without a complete output (missing variable
    /// field / failed staged flush) — a discard.
    Discard,
    /// A complete publish reached `send()`.
    Complete,
}

/// Observable decision the latch hands the logging layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Decision {
    /// First discard of a regime → loud `error!`.
    Error,
    /// Repeated discard → `debug!` carrying the running suppressed count.
    Debug(u64),
    /// A complete publish that ended a discard regime → `info!` with the total
    /// suppressed count.
    Recovered(u64),
    /// A steady-state complete publish (healthy — no log).
    Silent,
}

/// Drive a scripted event sequence through one latch and collect the decisions.
/// Compared against hand-written oracle vectors below.
fn run_script(events: &[Event]) -> Vec<Decision> {
    let mut latch = OutputDiscardLatch::new();
    events
        .iter()
        .map(|e| match e {
            Event::Discard => match latch.on_discard() {
                DiscardLogLevel::Error => Decision::Error,
                DiscardLogLevel::Debug { suppressed } => Decision::Debug(suppressed),
            },
            Event::Complete => match latch.on_complete() {
                Some(total) => Decision::Recovered(total),
                None => Decision::Silent,
            },
        })
        .collect()
}

/// The canonical cycle: error → debug(1) → debug(2) → recovery(2) → error.
#[test]
fn test_latch_canonical_cycle_matches_oracle() {
    use Decision::*;
    use Event::*;
    let got = run_script(&[Discard, Discard, Discard, Complete, Discard]);
    assert_eq!(
        got,
        vec![Error, Debug(1), Debug(2), Recovered(2), Error],
        "error → debug(running count) → recovery(total suppressed) → error(re-armed)"
    );
}

/// Healthy steady state: complete publishes never produce a log decision.
/// Catches a latch that spuriously reports recovery while healthy.
#[test]
fn test_latch_healthy_steady_state_is_silent() {
    use Decision::*;
    use Event::*;
    let got = run_script(&[Complete, Complete, Complete]);
    assert_eq!(got, vec![Silent, Silent, Silent]);
}

/// Inversion regression: an always-error implementation would emit `Error` at
/// index 1; a never-error implementation would emit `Debug` at index 0. Both
/// diverge from the oracle.
#[test]
fn test_latch_not_always_error_and_not_never_error() {
    use Decision::*;
    use Event::*;
    let got = run_script(&[Discard, Discard]);
    assert_eq!(got[0], Error, "first discard must error (never-error bug)");
    assert_eq!(
        got[1],
        Debug(1),
        "second discard must downgrade to debug (always-error flood bug)"
    );
}

/// A lone-error regime recovers SILENTLY (re-arms without a recovery
/// `info!` — no flood to close) but STILL re-arms, so the trailing discard
/// errors again. A no-recovery implementation that kept the latch set would
/// degrade that trailing discard to `Debug`.
#[test]
fn test_latch_lone_error_regime_rearms_silently() {
    use Decision::*;
    use Event::*;
    let got = run_script(&[Discard, Complete, Complete, Discard]);
    assert_eq!(
        got,
        vec![Error, Silent, Silent, Error],
        "lone-error recovery is silent (0 suppressed) yet still re-arms the error path"
    );
}

/// Alternating discard/complete: every discard errors (each starts a fresh
/// lone-error regime) and every complete re-arms SILENTLY (0 suppressed → no
/// recovery `info!`). A sticky latch would degrade later discards to
/// Debug; a latch without the silent re-arm would emit `Recovered(0)` on each complete.
#[test]
fn test_latch_alternating_pattern_matches_oracle() {
    use Decision::*;
    use Event::*;
    let got = run_script(&[Discard, Complete, Discard, Complete, Discard]);
    assert_eq!(got, vec![Error, Silent, Error, Silent, Error]);
}

/// Long discard run: exactly ONE error at the head, all repeats debug with a
/// monotonically increasing suppressed count — the flood-prevention property at
/// a broken-node publish rate.
#[test]
fn test_latch_long_discard_run_errors_exactly_once() {
    let events = vec![Event::Discard; 1000];
    let got = run_script(&events);
    let errors = got.iter().filter(|d| matches!(d, Decision::Error)).count();
    let debugs = got
        .iter()
        .filter(|d| matches!(d, Decision::Debug(_)))
        .count();
    assert_eq!(errors, 1, "exactly one error per discard regime");
    assert_eq!(debugs, 999, "all repeats downgraded to debug");
    // The last debug carries suppressed == 999 (running count, first not
    // counted).
    assert_eq!(
        *got.last().unwrap(),
        Decision::Debug(999),
        "the running suppressed count reaches N-1 on a 1000-discard regime"
    );
}

/// Recovery after a long regime reports the exact total suppressed count.
#[test]
fn test_latch_recovery_reports_exact_total_suppressed() {
    use Event::*;
    let mut events = vec![Discard; 500]; // 1 error + 499 debug (suppressed 1..=499)
    events.push(Complete); // recovery: total suppressed == 499
    let got = run_script(&events);
    assert_eq!(
        *got.last().unwrap(),
        Decision::Recovered(499),
        "recovery reports the total suppressed (downgraded) count, not the total discards"
    );
}

/// The suppressed count resets across regimes: a fresh regime after recovery
/// restarts its debug count at 1 (not continuing the prior total).
#[test]
fn test_latch_suppressed_count_resets_across_regimes() {
    use Decision::*;
    use Event::*;
    let got = run_script(&[
        Discard, Discard, Discard,  // regime 1: Error, Debug(1), Debug(2)
        Complete, // recovery(2)
        Discard, Discard, // regime 2: Error, Debug(1)
    ]);
    assert_eq!(
        got,
        vec![Error, Debug(1), Debug(2), Recovered(2), Error, Debug(1)]
    );
}

/// `is_failing` exposes the latch state independently of the decision stream
/// (Principle #3: observable state).
#[test]
fn test_latch_is_failing_tracks_state() {
    let mut latch = OutputDiscardLatch::new();
    assert!(!latch.is_failing());
    let _ = latch.on_discard();
    assert!(latch.is_failing());
    let _ = latch.on_discard();
    assert!(latch.is_failing(), "repeat discards keep the latch set");
    let _ = latch.on_complete();
    assert!(!latch.is_failing(), "a complete publish clears the latch");
}

/// Drive a scripted event sequence and return the `total_discards` counter
/// AFTER EACH event — the oracle surface for the unconditional counter.
fn total_discards_trace(events: &[Event]) -> Vec<u64> {
    let mut latch = OutputDiscardLatch::new();
    events
        .iter()
        .map(|e| {
            match e {
                Event::Discard => {
                    let _ = latch.on_discard();
                }
                Event::Complete => {
                    let _ = latch.on_complete();
                }
            }
            latch.total_discards()
        })
        .collect()
}

/// Principle #3 queryability: the unconditional `total_discards`
/// counter bumps on EVERY discard independent of the log-level regime (error
/// head vs debug-suppressed), is untouched by complete publishes, and — the
/// key pin — NEVER resets on recovery. Asymmetric with `suppressed`, which the
/// canonical-cycle test above proves resets each regime.
#[test]
fn test_latch_total_discards_counts_all_discards_never_resets() {
    use Event::*;
    // Two discard regimes separated by a recovery, bracketed by healthy
    // completes. The counter is monotonic non-decreasing, bumps ONLY on
    // Discard, and reaches the total discard count (5) regardless of the
    // per-regime suppressed resets.
    let got = total_discards_trace(&[
        Complete, // 0 — healthy complete does not bump
        Discard, Discard, Discard,  // regime 1: 1,2,3
        Complete, // 3 — recovery does NOT reset the lifetime total
        Discard, Discard,  // regime 2: 4,5
        Complete, // 5 — recovery again, no reset
    ]);
    assert_eq!(
        got,
        vec![0, 1, 2, 3, 3, 4, 5, 5],
        "total_discards bumps once per discard, never on a complete, and never \
         resets at recovery — the queryable signal for a persistently-broken node"
    );
}

/// A long sustained-discard regime (all after the first are `debug!`-suppressed)
/// still counts EVERY discard in `total_discards`: the counter is independent of
/// the flood-latch downgrade, so a broken node whose head `error!` scrolled away
/// remains fully countable.
#[test]
fn test_latch_total_discards_counts_debug_suppressed_discards() {
    let events = vec![Event::Discard; 1000];
    let got = total_discards_trace(&events);
    assert_eq!(
        *got.last().unwrap(),
        1000,
        "all 1000 discards counted (1 error + 999 debug-suppressed)"
    );
    // Monotonic +1 per discard (i.e. the k-th event reads k).
    assert!(
        got.iter().enumerate().all(|(i, &t)| t == (i as u64) + 1),
        "the counter advances by exactly 1 per discard regardless of log level"
    );
}
