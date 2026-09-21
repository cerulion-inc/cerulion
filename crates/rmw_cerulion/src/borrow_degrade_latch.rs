// SPDX-License-Identifier: AGPL-3.0-only
//! The flood-suppression reporter of the windowed-borrow
//! PUBLISH path — the publish-side sibling of [`crate::loan_refusal_latch`],
//! built on `cerulion_core`'s one shared `FailureRegimeLatch` (never a
//! hand-rolled copy).
//!
//! A windowed borrow that could not publish fully zero-copy is a DEGRADE,
//! not a loss: the frame still ships, correct, through a copy — the same
//! bytes a plain copying publish would have shipped. What the operator learns
//! is that the zero-copy contract is not being met and WHY, once per
//! regime instead of once per frame (a 30 Hz camera whose fill escapes the
//! window on every frame is 30 lines/s forever — the disk-fill class the
//! shared latch exists for). Every degrade kind shares ONE regime (they
//! are all "this publish paid a copy") but carries its OWN headline and
//! remedy, exactly like the take side's refusal reporter.
//!
//! The policy, as every latch consumer applies it: loud head (`warn!` —
//! a degrade, not a loss) with full context + the remedy, `debug!` repeats
//! carrying the suppressed count, a loud re-announcement at each decade of
//! the running total (the counters sit behind the standardized rmw C ABI,
//! so the log is a ROS user's only window), ONE `info!` recovery when a
//! fully-adopted publish follows, and an unconditional total riding every
//! line as `total_failures=`. Keys: `topic=`, `kind=`,
//! `total_failures=`, `suppressed_count=`.
//!
//! A wrong-thread loan RETURN is NOT a publish degrade — it publishes and
//! copies nothing (the slot is simply held for the owner thread's next
//! borrow) — so it rides its OWN latch + reporter here
//! (`report_wrong_thread_return`, kind `wrong_thread_return`,
//! recovered by the owner-thread self-heal sweep) rather than falsely
//! inflating the publish-degrade regime and its
//! `borrow_degrade_count()` total. `BorrowDegrade::WrongThread` stays
//! the PUBLISH-path token (that one genuinely copies a frame).

use std::sync::Mutex;

use cerulion_core::transport::failure_regime_latch::{
    lock_regime_latch, FailureRegimeLatch, RegimeDecision,
};

/// Why a windowed-borrow publish paid a copy — the `kind=` token and the
/// headline selector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BorrowDegrade {
    /// The borrow could not arm a window (this thread already had one — a
    /// second outstanding borrow — or the hook's thread state was torn
    /// down). The fill went to the heap and the publish copied.
    WindowUnavailable,
    /// One or more forgeable members' storage was not adopted (another
    /// thread's fill, growth past the tail, a foreign allocator, an
    /// element-misaligned landing) — those members copied; the rest of the
    /// frame may still have adopted.
    Escaped,
    /// The adopted layout would have shipped more dead gap bytes than the
    /// budget — a tight copy frame was cheaper than the slack.
    GapExcess,
    /// The copied members did not fit the loaned slot past the fill's
    /// bump cursor — republished through an exact-size copy loan.
    Overflow,
    /// The publish arrived on a different thread than the borrow: the
    /// borrow thread's window cannot be disarmed from here, so the frame
    /// copied and the original slot is HELD until that thread's next
    /// borrow (an armed window over a recycled slot would corrupt a later
    /// frame).
    WrongThread,
}

impl BorrowDegrade {
    /// The `kind=` token.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::WindowUnavailable => "window_unavailable",
            Self::Escaped => "escaped",
            Self::GapExcess => "gap_excess",
            Self::Overflow => "overflow",
            Self::WrongThread => "wrong_thread",
        }
    }
}

fn observe_failure(latch: &Mutex<FailureRegimeLatch>) -> (RegimeDecision, u64) {
    let mut latch = lock_regime_latch(latch);
    let decision = latch.on_failure();
    (decision, latch.total_failures())
}

fn observe_success(latch: &Mutex<FailureRegimeLatch>) -> (Option<u64>, u64) {
    let mut latch = lock_regime_latch(latch);
    let recovered = latch.on_success();
    (recovered, latch.total_failures())
}

/// The one headline per kind: what happened and what cures it. `&'static`
/// so every regime arm logs the same text for a kind.
fn headline(kind: BorrowDegrade) -> &'static str {
    match kind {
        BorrowDegrade::WindowUnavailable => {
            "windowed borrow could not arm a window (another outstanding borrow \
             already holds this thread's window, or the thread is tearing down) — \
             the fill went to the heap and this publish COPIED; publish or return \
             the other loan first to restore zero-copy"
        }
        BorrowDegrade::Escaped => {
            "windowed publish COPIED escaped storage — a forgeable member's fill \
             did not land in the borrow window (another thread's fill, growth past \
             the reserved tail, or a foreign allocator); the frame is correct, at \
             the cost of a copy"
        }
        BorrowDegrade::GapExcess => {
            "windowed publish fell back to a tight COPY frame — the adopted layout \
             would have shipped more dead gap bytes than the budget allows (an \
             abandoned earlier fill left large holes); the frame is correct"
        }
        BorrowDegrade::Overflow => {
            "windowed publish fell back to an exact-size COPY loan — the copied \
             members did not fit the borrow slot past the fill's cursor; the frame \
             is correct"
        }
        BorrowDegrade::WrongThread => {
            "loaned publish arrived on a different thread than the borrow — the \
             borrow thread's window cannot be disarmed from here, so this publish \
             COPIED and the original slot is held until that thread borrows again; \
             publish loans on the thread that borrowed them to restore zero-copy"
        }
    }
}

/// Record one degraded windowed publish and log it per the regime policy.
/// `escaped`/`adopted` are the member counts of THIS frame (how much of it
/// still went zero-copy).
pub(crate) fn report_borrow_degraded(
    latch: &Mutex<FailureRegimeLatch>,
    topic: &str,
    kind: BorrowDegrade,
    escaped: usize,
    adopted: usize,
) {
    let (decision, total) = observe_failure(latch);
    let msg = headline(kind);
    match decision {
        // The per-kind headline rides the structured `reason=` field
        // (greppable key, no interpolation — the repo's structured-fields
        // rule); the message text stays static.
        RegimeDecision::Loud => tracing::warn!(
            topic = %topic,
            kind = %kind.as_str(),
            escaped,
            adopted,
            total_failures = total,
            reason = %msg,
            "windowed publish degraded to a copy (repeats are logged at debug \
             until a fully zero-copy publish)"
        ),
        RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
            topic = %topic,
            kind = %kind.as_str(),
            escaped,
            adopted,
            total_failures = total,
            suppressed_count = suppressed,
            reason = %msg,
            "windowed publish STILL degrading to copies"
        ),
        RegimeDecision::Suppressed { suppressed } => tracing::debug!(
            topic = %topic,
            kind = %kind.as_str(),
            escaped,
            adopted,
            total_failures = total,
            suppressed_count = suppressed,
            "windowed publish degraded to a copy (suppressed repeat)"
        ),
    }
}

/// Record a FULLY zero-copy windowed publish: closes an open degrade
/// regime, logging the recovery once (only when something was suppressed)
/// and re-arming the loud head.
pub(crate) fn report_borrow_adopted(latch: &Mutex<FailureRegimeLatch>, topic: &str) {
    let (recovered, total) = observe_success(latch);
    if let Some(suppressed) = recovered {
        tracing::info!(
            topic = %topic,
            suppressed_count = suppressed,
            total_failures = total,
            "windowed publish recovered — frames adopt their fill zero-copy again"
        );
    }
}

/// Record a wrong-thread loan RETURN — a slot-hold lifecycle event, NOT a
/// publish degrade (nothing published, nothing copied): its own latch,
/// its own kind token, the same regime policy.
pub(crate) fn report_wrong_thread_return(latch: &Mutex<FailureRegimeLatch>, topic: &str) {
    let (decision, total) = observe_failure(latch);
    let msg = "loaned RETURN arrived on a different thread than the borrow — no \
               frame was published or copied; the borrow thread's window cannot \
               be disarmed from here, so the slot is HELD until that thread \
               borrows again (released by its self-heal sweep); return loans on \
               the thread that borrowed them";
    match decision {
        RegimeDecision::Loud => tracing::warn!(
            topic = %topic,
            kind = "wrong_thread_return",
            total_failures = total,
            reason = %msg,
            "loaned return held a slot for its borrowing thread (repeats are \
             logged at debug until the owner thread's sweep releases one)"
        ),
        RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
            topic = %topic,
            kind = "wrong_thread_return",
            total_failures = total,
            suppressed_count = suppressed,
            reason = %msg,
            "loaned returns STILL arriving on non-borrowing threads"
        ),
        RegimeDecision::Suppressed { suppressed } => tracing::debug!(
            topic = %topic,
            kind = "wrong_thread_return",
            total_failures = total,
            suppressed_count = suppressed,
            "loaned return held a slot for its borrowing thread (suppressed repeat)"
        ),
    }
}

/// Close a wrong-thread-return regime: the owner thread's self-heal sweep
/// released the held slot(s).
pub(crate) fn report_wrong_thread_return_healed(latch: &Mutex<FailureRegimeLatch>, topic: &str) {
    let (recovered, total) = observe_success(latch);
    if let Some(suppressed) = recovered {
        tracing::info!(
            topic = %topic,
            suppressed_count = suppressed,
            total_failures = total,
            "held slot(s) released — the borrowing thread's sweep self-healed"
        );
    }
}
