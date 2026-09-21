// SPDX-License-Identifier: AGPL-3.0-only
//! Boundary contract for the live-loop WaitSet attachment
//! capacity guard (`check_waitset_attachment_capacity`).
//!
//! The guard fails a graph build fast if the live loop would attach more
//! iceoryx2 WaitSet notifications (one per `DataTrigger` trigger input + one
//! per `Sync` input + one per external fd source) than the FD-set bound
//! [`WAITSET_MAX_ATTACHMENTS`]. Past the cap, `attach_notification` silently
//! drops sources and those nodes lose their prompt event-driven wakeup
//! (degrading to the 250 ms liveliness cadence — a latency regression, not data
//! loss).
//!
//! These tests pin the PURE boundary helper directly (no transport, no 1025
//! real nodes): the inclusive cap, the additive split across the THREE terms
//! (data-trigger + sync + external fd sources), `saturating_add`
//! panic-safety on pathological inputs, the actionable error string, and the
//! cap-value oracle. The guard is wired into every build path at
//! `GraphRuntime::build_with_scheduler` (the single choke point), so pinning the
//! helper pins the build contract.

use cerulion_core::graph::{check_waitset_attachment_capacity, WAITSET_MAX_ATTACHMENTS};
use cerulion_core::TransportError;

/// The cap value is a documented contract (FD_SETSIZE-conservative). Oracle-pin
/// it so a silent bump is a deliberate, reviewed change.
#[test]
fn cap_is_fd_setsize_conservative_1024() {
    assert_eq!(
        WAITSET_MAX_ATTACHMENTS, 1024,
        "WAITSET_MAX_ATTACHMENTS is the FD_SETSIZE-conservative WaitSet bound; \
         changing it is a deliberate cross-platform decision"
    );
}

/// Happy path: realistic graph sizes (and the empty graph) are accepted,
/// including graphs carrying a nonzero external fd-source term.
#[test]
fn typical_counts_are_accepted() {
    for (dt, sync, ext) in [
        (0, 0, 0),
        (1, 0, 1),
        (0, 1, 2),
        (8, 4, 4),
        (100, 100, 100),
        (340, 340, 340),
    ] {
        assert!(
            check_waitset_attachment_capacity(dt, sync, ext).is_ok(),
            "({dt}, {sync}, {ext}) totals {} ≤ cap — must be accepted",
            dt + sync + ext
        );
    }
}

/// The cap is INCLUSIVE: exactly `WAITSET_MAX_ATTACHMENTS` total attachments is
/// the last accepted count; one more is rejected. Pinned across every split of
/// the boundary total between the two binding terms (kills a `>=`-vs-`>`
/// off-by-one and a term-swap regression). The external fd-source term is
/// exercised separately in [`external_term_participates_in_the_cap`].
#[test]
fn cap_boundary_is_inclusive_across_both_terms() {
    let cap = WAITSET_MAX_ATTACHMENTS;
    // Exactly at the cap — accepted, every split.
    for dt in [0, 1, cap / 2, cap - 1, cap] {
        let sync = cap - dt;
        assert!(
            check_waitset_attachment_capacity(dt, sync, 0).is_ok(),
            "({dt} + {sync} = {cap}) is exactly the cap — must be accepted"
        );
    }
    // One over the cap — rejected, every split.
    for dt in [0, 1, cap / 2, cap, cap + 1] {
        let sync = (cap + 1).saturating_sub(dt);
        assert!(
            check_waitset_attachment_capacity(dt, sync, 0).is_err(),
            "({dt} + {sync} = {}) exceeds the cap — must be rejected",
            dt.saturating_add(sync)
        );
    }
}

/// The external fd-source term is additive with the two binding terms
/// and participates in the SAME inclusive cap. The boundary total is split three
/// ways so the external term can push an otherwise-fitting graph over the cap.
#[test]
fn external_term_participates_in_the_cap() {
    let cap = WAITSET_MAX_ATTACHMENTS;
    // Exactly at the cap, distributed across all three terms — accepted.
    for ext in [0, 1, cap / 3, cap] {
        let dt = (cap - ext) / 2;
        let sync = cap - ext - dt; // dt + sync + ext == cap exactly
        assert_eq!(
            dt + sync + ext,
            cap,
            "test setup: three terms sum to the cap"
        );
        assert!(
            check_waitset_attachment_capacity(dt, sync, ext).is_ok(),
            "({dt} + {sync} + {ext} = {cap}) is exactly the cap — must be accepted"
        );
    }
    // The external term alone pushing one over the cap — rejected.
    for ext in [1, cap / 3, cap] {
        let dt = (cap + 1 - ext) / 2;
        let sync = (cap + 1) - ext - dt; // dt + sync + ext == cap + 1
        assert!(
            check_waitset_attachment_capacity(dt, sync, ext).is_err(),
            "({dt} + {sync} + {ext} = {}) exceeds the cap — must be rejected",
            dt + sync + ext
        );
    }
}

/// Any term alone can blow the cap; the others being zero must not mask it.
#[test]
fn each_term_alone_can_exceed_the_cap() {
    assert!(check_waitset_attachment_capacity(WAITSET_MAX_ATTACHMENTS + 1, 0, 0).is_err());
    assert!(check_waitset_attachment_capacity(0, WAITSET_MAX_ATTACHMENTS + 1, 0).is_err());
    assert!(check_waitset_attachment_capacity(0, 0, WAITSET_MAX_ATTACHMENTS + 1).is_err());
    // and at the cap with the others zero — accepted.
    assert!(check_waitset_attachment_capacity(WAITSET_MAX_ATTACHMENTS, 0, 0).is_ok());
    assert!(check_waitset_attachment_capacity(0, WAITSET_MAX_ATTACHMENTS, 0).is_ok());
    assert!(check_waitset_attachment_capacity(0, 0, WAITSET_MAX_ATTACHMENTS).is_ok());
}

/// Adversarial: pathological counts must REJECT via `saturating_add`, never
/// panic on overflow. `usize::MAX + anything` saturates to `usize::MAX > cap`.
#[test]
fn pathological_counts_saturate_and_reject_without_panic() {
    assert!(check_waitset_attachment_capacity(usize::MAX, 0, 0).is_err());
    assert!(check_waitset_attachment_capacity(0, usize::MAX, 0).is_err());
    assert!(check_waitset_attachment_capacity(0, 0, usize::MAX).is_err());
    assert!(check_waitset_attachment_capacity(usize::MAX, usize::MAX, usize::MAX).is_err());
    assert!(check_waitset_attachment_capacity(usize::MAX, 1, 1).is_err());
}

/// The rejection message must be actionable: it names ALL THREE per-term counts,
/// the total, the cap, the FD_SETSIZE cause, and the split-across-processes
/// remedy. (User-facing build-error surface is the contract — pin its content.)
#[test]
fn rejection_message_is_actionable() {
    let dt = 700;
    let sync = 400;
    let ext = 50; // total 1150 > 1024
    let err = check_waitset_attachment_capacity(dt, sync, ext).expect_err("1150 > cap must reject");
    let TransportError::GraphError { reason } = err else {
        panic!("expected GraphError, got {err:?}");
    };
    for needle in [
        "1150",                             // total
        "700",                              // data-trigger term
        "400",                              // sync term
        "50",                               // external fd-source term
        "external fd sources",              // the term's label
        "1024",                             // the cap
        "FD-set",                           // the cause
        "split the graph across processes", // the remedy
    ] {
        assert!(
            reason.contains(needle),
            "rejection message missing {needle:?}: {reason}"
        );
    }
}

/// Determinism: the same inputs yield a byte-identical error string (the helper
/// is pure; pin it so a future format change can't silently smuggle
/// nondeterminism into a build-error path).
#[test]
fn rejection_message_is_deterministic() {
    let render = || match check_waitset_attachment_capacity(2000, 100, 50) {
        Err(TransportError::GraphError { reason }) => reason,
        other => panic!("expected GraphError, got {other:?}"),
    };
    assert_eq!(render(), render());
}
