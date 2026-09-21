// SPDX-License-Identifier: AGPL-3.0-only
//! Tests for the pre-step-drain warn-suppression latch.
//!
//! The policy lives in the pure `DrainWarnLatch` state machine
//! (`cerulion_core/src/graph/drain_latch.rs`), kept apart from
//! `DataTriggerBinding` so it is testable without any
//! subscriber mock or transport. Contract:
//!
//! - FIRST failure of a run → log at `warn` (operators always see it)
//! - repeated identical failures → downgrade to `debug` (no flooding
//!   at 1 kHz with a persistently-broken transport)
//! - recovery (first success after failures) → latch clears, one-shot
//!   `info`
//! - next failure after recovery → `warn` again
//!
//! The mapping of these decisions onto actual `tracing` events is
//! pinned by inline tracing-capture tests in `graph/runtime.rs`
//! (`drain_log_cycle_emits_warn_then_debug_then_recovery_info` /
//! `drain_log_failure_after_recovery_warns_again`).
//!
//! No iceoryx2 dependency — parallel-safe, no `--test-threads=1`
//! requirement.

use cerulion_core::graph::drain_latch::{DrainFailureLevel, DrainWarnLatch};

/// Scripted drain outcome for the oracle-vector tests.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Event {
    Failure,
    Success,
}

/// Observable decision the latch hands the logging layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Decision {
    Warn,
    Debug,
    /// Success that ended a failure run (caller logs info once).
    Recovered,
    /// Steady-state success (no log).
    Silent,
}

/// Drive a scripted event sequence through one latch and collect the
/// decisions. Compared against hand-written oracle vectors below
/// (NOT against a second run of the same code — see the
/// anti-tautological-test note in the project docs).
fn run_script(events: &[Event]) -> Vec<Decision> {
    let mut latch = DrainWarnLatch::new();
    events
        .iter()
        .map(|e| match e {
            Event::Failure => match latch.on_failure() {
                DrainFailureLevel::Warn => Decision::Warn,
                DrainFailureLevel::Debug => Decision::Debug,
            },
            Event::Success => {
                if latch.on_success() {
                    Decision::Recovered
                } else {
                    Decision::Silent
                }
            }
        })
        .collect()
}

/// The canonical cycle from the issue text:
/// warn → debug → info(recovery) → warn(cleared).
#[test]
fn test_latch_canonical_cycle_matches_oracle() {
    use Decision::*;
    use Event::*;
    let got = run_script(&[Failure, Failure, Failure, Success, Failure]);
    assert_eq!(
        got,
        vec![Warn, Debug, Debug, Recovered, Warn],
        "canonical warn → debug → recovery → warn cycle"
    );
}

/// Healthy steady state: successes never produce a log decision.
/// Catches a latch that spuriously reports recovery while healthy.
#[test]
fn test_latch_healthy_steady_state_is_silent() {
    use Decision::*;
    use Event::*;
    let got = run_script(&[Success, Success, Success]);
    assert_eq!(got, vec![Silent, Silent, Silent]);
}

/// Inversion regression: an always-warn implementation would emit
/// `Warn` at index 1; a never-warn implementation would emit `Debug`
/// at index 0. Both diverge from the oracle.
#[test]
fn test_latch_not_always_warn_and_not_never_warn() {
    use Decision::*;
    use Event::*;
    let got = run_script(&[Failure, Failure]);
    assert_eq!(got[0], Warn, "first failure must warn (never-warn bug)");
    assert_eq!(
        got[1], Debug,
        "second failure must downgrade (always-warn bug)"
    );
}

/// Inversion regression: a no-recovery implementation either keeps the
/// latch set across successes (post-recovery failure stays `Debug`) or
/// never reports `Recovered`. The oracle pins both.
#[test]
fn test_latch_recovery_clears_and_reports_once() {
    use Decision::*;
    use Event::*;
    let got = run_script(&[Failure, Success, Success, Failure]);
    assert_eq!(
        got,
        vec![Warn, Recovered, Silent, Warn],
        "recovery must report exactly once, then re-arm the warn"
    );
}

/// Alternating failure/success: every failure warns (each starts a new
/// run), every success recovers. A sticky latch would degrade later
/// failures to Debug.
#[test]
fn test_latch_alternating_pattern_matches_oracle() {
    use Decision::*;
    use Event::*;
    let got = run_script(&[Failure, Success, Failure, Success, Failure]);
    assert_eq!(got, vec![Warn, Recovered, Warn, Recovered, Warn]);
}

/// Long failure run: exactly ONE warn at the head, all repeats debug —
/// the flood-prevention property at 1 kHz schedules.
#[test]
fn test_latch_long_failure_run_warns_exactly_once() {
    let events = vec![Event::Failure; 1000];
    let got = run_script(&events);
    let warns = got.iter().filter(|d| **d == Decision::Warn).count();
    let debugs = got.iter().filter(|d| **d == Decision::Debug).count();
    assert_eq!(warns, 1, "exactly one warn per failure run");
    assert_eq!(debugs, 999, "all repeats downgraded to debug");
}

/// `is_failing` exposes the latch state independently of the decision
/// stream (Principle #3: observable state).
#[test]
fn test_latch_is_failing_tracks_state() {
    let mut latch = DrainWarnLatch::new();
    assert!(!latch.is_failing());
    let _ = latch.on_failure();
    assert!(latch.is_failing());
    let _ = latch.on_failure();
    assert!(latch.is_failing(), "repeat failures keep the latch set");
    let _ = latch.on_success();
    assert!(!latch.is_failing(), "success clears the latch");
}
