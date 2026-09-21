// SPDX-License-Identifier: AGPL-3.0-only
//! The flood-suppression reporters of the forged loaned
//! take — the take-side siblings of [`crate::decode_failure_latch`] and
//! [`crate::publish_reject_latch`], built on `cerulion_core`'s one shared
//! `FailureRegimeLatch` (never a hand-rolled fifth copy). Two CONDITIONS,
//! two latches, because their remedies differ:
//!
//! # A REFUSED loaned take (`report_loan_refused`)
//!
//! `rmw_take_loaned_message` holds an iceoryx2 sample across the C ABI for
//! every loan the consumer has not yet returned, and (for a forged type) one
//! rmw-owned shadow message per outstanding loan. Both are BUDGETED — the
//! service's `subscriber_max_borrowed_samples` (provisioned at
//! `RMW_TAKE_LOAN_BORROW_BUDGET`) and the shadow pool sized to the same
//! number — and a consumer that RETAINS loans past that budget is refused on
//! every following take until it returns one. A BARE `error!` per
//! attempt would flood: rclcpp's executor retries the take at the arrival
//! rate, so on a 30 Hz camera topic a stuck consumer is 30 lines/s forever —
//! the disk-fill class `FailureRegimeLatch` exists for. Every refusal kind
//! ([`LoanRefusal`]) shares the ONE regime (they are all "the take was
//! refused, no frame consumed") but carries its OWN headline and remedy: a
//! pool exhausted by retained loans is cured by returning one, a failed heap
//! allocation is memory pressure with NOTHING to return, and a transport
//! receive failure carries the transport's own reason (`ExceedsMaxBorrows`
//! is the budget; anything else is not).
//!
//! The ADOPT path's budget refusal has its own reporter
//! (`report_adopt_borrow_refused`) on the SAME latch and regime, because an
//! adopt-armed type is always `can_loan_take` too: outstanding LOANS and
//! retained ADOPTED messages consume one budget between them, so the line
//! must carry both counts and name whichever actually holds it
//! ([`BorrowHolders`]).
//!
//! # A FORGE that had to fall back to copying (`report_forge_fallback`)
//!
//! A forgeable entry whose offset lies BELOW the frame's data floor (inside
//! the fixed section or the offset table — the placement the `FrameWalker`
//! refuses outright) is never forged: aliasing header/table bytes as a
//! `std::vector`'s elements would be silent wrong data. The take still
//! succeeds — that member is COPIED into the shadow exactly as the copying
//! `rmw_take` would serve it — so the consumer sees the same bytes either
//! way, but the frame paid a copy and its producer emits a layout the wire
//! rule forbids. Loud once per regime (`warn!` — a degrade, not a loss), a
//! recovery line when a clean frame follows, an unconditional total.
//!
//! # The policy, as every latch consumer applies it
//!
//! Loud head (full context + the remedy), `debug!` repeats carrying the
//! suppressed count AND the kind's own remedy (a repeat IS the normal retry
//! path — a generic repeat line would hide the remedy exactly where an
//! operator tailing at debug is looking), a loud
//! re-announcement at each decade of the running
//! total (the counters sit behind the standardized rmw C ABI, so the log is
//! a ROS user's only window), ONE recovery line naming what was suppressed,
//! and an unconditional total that rides EVERY line as `total_failures=` —
//! head, repeat, re-announcement and recovery alike, exactly as the sibling
//! reporters do — so a regime that never reaches a decade still exposes its
//! running count. Keys are the repo's: `topic=`, `total_failures=`,
//! `suppressed_count=`, and `kind=` names which budget bound.

use std::sync::Mutex;

use cerulion_core::transport::failure_regime_latch::{
    lock_regime_latch, FailureRegimeLatch, RegimeDecision,
};

/// Why a loaned take was refused — the `kind=` field on every line, and the
/// selector for the headline + remedy the line carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoanRefusal {
    /// Every shadow in the subscription's pool is loaned out — the consumer
    /// is RETAINING loans (forged types only; checked BEFORE the receive, so
    /// no frame is consumed). Remedy: return one.
    ShadowPoolExhausted,
    /// The pool was under capacity but the shadow could not be built: the
    /// heap allocation failed. Memory pressure or a bug — NOT a retained
    /// loan; there is nothing to return. Also refused before the receive.
    ShadowAllocationFailed,
    /// The transport refused the receive; its reason rides `error=`
    /// verbatim. iceoryx2's `ExceedsMaxBorrows` is the borrow budget (the
    /// subscription holds it whole — return a loan); any other reason is the
    /// transport's own condition, not a loan the consumer can return.
    ReceiveFailed,
}

impl LoanRefusal {
    /// The `kind=` token.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::ShadowPoolExhausted => "shadow_pool_exhausted",
            Self::ShadowAllocationFailed => "shadow_allocation_failed",
            Self::ReceiveFailed => "receive_failed",
        }
    }
}

/// WHO is holding the service's borrow budget when an
/// adopted take's receive is refused with `ExceedsMaxBorrows`.
///
/// The refusal itself is always correct — the port really has no borrow
/// left — but its REMEDY is not one thing. An adopt-armed type is always
/// `can_loan_take` too, so `rmw_take_loaned_message`'s outstanding loans
/// (`pending_takes`) consume the SAME `subscriber_max_borrowed_samples`
/// budget as adopted samples do. Without this classification the arm would always
/// say "free adopted messages", advice that does exactly nothing for a
/// consumer whose borrows are all LOANS — and would leave an operator grepping
/// `kind=adopted_budget_exhausted` on a line that is not about adoption.
///
/// One CONDITION, so one regime and one latch (the take was refused and no
/// frame was consumed); what varies is the noun and the remedy, which is
/// what this enum carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BorrowHolders {
    /// Adopted messages the app has not freed hold the budget, and this
    /// subscription has no outstanding loan. SELF-HEALS when the app frees
    /// any adopted message (its `fini`/destructor releases the sample).
    AdoptedOnly,
    /// Outstanding LOANED takes hold the budget and no adopted sample does.
    /// Freeing adopted messages cannot help — there are none.
    LoanedOnly,
    /// Both paths hold borrows. Either kind of return releases a unit, so
    /// the line names both rather than guessing which the operator meant.
    Both,
    /// The counters say nobody and the port refused anyway.
    ///
    /// The dominant cause is a RACE, not a misconfiguration, so the line
    /// must not prescribe one: `AdoptedSample`'s `Drop` lowers
    /// `outstanding` BEFORE its `sample` field drops and returns the SHM
    /// borrow, and the take reads that counter `Relaxed` from another
    /// thread — so a release landing between the refused receive and this
    /// read shows as `(0, 0)` on a condition that heals by itself. Telling
    /// an operator to restart a robot over that would be the very defect
    /// this enum exists to remove.
    ///
    /// (The obvious other explanation — a pre-existing service whose
    /// ceiling is too small — is NOT reachable here: a ceiling below
    /// `crate::adopt_take::MIN_ADOPT_EFFECTIVE_BUDGET` does not arm
    /// adoption at all, so `take_adopted` never runs on one.)
    Unaccounted,
}

impl BorrowHolders {
    /// The PURE decision: who holds the budget,
    /// from the two counts the take path already has —
    /// `AdoptStats::outstanding` and `SubscriptionInner::pending_takes`.
    /// Total over both counts, allocation-free, dependent on nothing else.
    pub fn classify(adopted_outstanding: usize, loaned_outstanding: usize) -> Self {
        match (adopted_outstanding > 0, loaned_outstanding > 0) {
            (true, false) => Self::AdoptedOnly,
            (false, true) => Self::LoanedOnly,
            (true, true) => Self::Both,
            (false, false) => Self::Unaccounted,
        }
    }

    /// The `kind=` token. `adopted_budget_exhausted` names the
    /// adopted-only case ALONE: an operator's grep for it
    /// means exactly what it says.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::AdoptedOnly => "adopted_budget_exhausted",
            Self::LoanedOnly => "loaned_borrows_exhausted",
            Self::Both => "mixed_borrows_exhausted",
            Self::Unaccounted => "borrow_budget_unaccounted",
        }
    }

    /// The remedy sentence — the half of the line that depends on who
    /// holds the budget.
    pub fn remedy(self) -> &'static str {
        match self {
            Self::AdoptedOnly => {
                "free adopted messages (their fini/destructor releases the samples), or \
                 raise CERULION_RMW_ADOPT_TAKE_BUDGET — a CREATE-time floor that cannot \
                 widen an already-existing smaller service"
            }
            Self::LoanedOnly => {
                "return outstanding loaned messages via \
                 rmw_return_loaned_message_from_subscription; this subscription holds NO \
                 adopted message, so freeing adopted messages cannot release anything"
            }
            Self::Both => {
                "both paths hold borrows on this subscription — return outstanding loaned \
                 messages via rmw_return_loaned_message_from_subscription AND/OR free \
                 adopted messages (their fini/destructor releases the samples); either \
                 releases a unit"
            }
            Self::Unaccounted => {
                "this subscription's own counters show NO borrow held, so it has nothing \
                 to return — most likely a release that landed between the refused receive \
                 and this reading (the adopted counter drops before the SHM borrow does), \
                 in which case the next take succeeds; if it persists, the bound is the \
                 SERVICE's own capacity (budget=)"
            }
        }
    }
}

/// Record one adopted take refused because the service's
/// borrow budget is exhausted, attributing it to whoever actually holds the
/// borrows.
///
/// Shares the loaned path's ONE refusal latch and regime — the condition is
/// the same ("the take was refused, no frame consumed", `taken = false`, the
/// queued frame is NOT consumed, never `RMW_RET_ERROR`) — and carries BOTH
/// counts on every line, because the operator's next move depends on which
/// of them is nonzero and a single `outstanding_loans=` could not say.
pub(crate) fn report_adopt_borrow_refused(
    latch: &Mutex<FailureRegimeLatch>,
    topic: &str,
    adopted_outstanding: usize,
    loaned_outstanding: usize,
    budget: usize,
    error: &str,
) {
    // The classification is derived HERE, from the counts the line is about
    // to print, so the `kind=` token and the numbers beside it cannot
    // disagree: a caller cannot hand in a holder that contradicts its own
    // evidence, because it cannot hand one in at all.
    let holders = BorrowHolders::classify(adopted_outstanding, loaned_outstanding);
    let (decision, total) = observe_failure(latch);
    let remedy = holders.remedy();
    match decision {
        RegimeDecision::Loud => tracing::warn!(
            topic = %topic,
            kind = %holders.as_str(),
            adopted_outstanding,
            loaned_outstanding,
            budget,
            error = %error,
            total_failures = total,
            remedy = %remedy,
            "adopted take refused — the service's EFFECTIVE borrow budget is exhausted \
             (budget= is the service's real capacity; taken=false, the queued frame is not \
             consumed); the release that recovers it is in remedy= (repeats are logged at \
             debug until a take succeeds)"
        ),
        RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
            topic = %topic,
            kind = %holders.as_str(),
            adopted_outstanding,
            loaned_outstanding,
            budget,
            error = %error,
            total_failures = total,
            suppressed_count = suppressed,
            remedy = %remedy,
            "adopted take STILL refused — the regime is open and every attempt is refused \
             (the whole borrow budget is still held); the release that recovers it is in \
             remedy="
        ),
        RegimeDecision::Suppressed { suppressed } => tracing::debug!(
            topic = %topic,
            kind = %holders.as_str(),
            adopted_outstanding,
            loaned_outstanding,
            budget,
            error = %error,
            total_failures = total,
            suppressed_count = suppressed,
            remedy = %remedy,
            "adopted take refused (suppressed repeat) — the release that recovers it is in \
             remedy="
        ),
    }
}

/// One failure observation: the regime decision plus the running total AFTER
/// it was counted, read under one lock so the two cannot disagree.
fn observe_failure(latch: &Mutex<FailureRegimeLatch>) -> (RegimeDecision, u64) {
    let mut latch = lock_regime_latch(latch);
    let decision = latch.on_failure();
    (decision, latch.total_failures())
}

/// One success observation: the recovery (if any) plus the running total.
fn observe_success(latch: &Mutex<FailureRegimeLatch>) -> (Option<u64>, u64) {
    let mut latch = lock_regime_latch(latch);
    let recovered = latch.on_success();
    (recovered, latch.total_failures())
}

/// Record one refused loaned take and log it per the regime policy, with
/// the headline and remedy that are TRUE for `kind`.
///
/// `outstanding_loans` and `budget` are the two numbers the operator needs
/// to see the shape (how many are held vs. how many may be); `error` is the
/// transport's own reason for a [`LoanRefusal::ReceiveFailed`] (empty for
/// the shadow-side kinds, which have no transport error to carry).
pub(crate) fn report_loan_refused(
    latch: &Mutex<FailureRegimeLatch>,
    topic: &str,
    kind: LoanRefusal,
    outstanding_loans: usize,
    budget: usize,
    error: &str,
) {
    let (decision, total) = observe_failure(latch);
    match decision {
        RegimeDecision::Loud => match kind {
            LoanRefusal::ShadowPoolExhausted => tracing::error!(
                topic = %topic,
                kind = %kind.as_str(),
                outstanding_loans,
                budget,
                total_failures = total,
                "loaned take refused — every shadow in the pool is loaned out (the consumer \
                 holds its whole loan budget): return outstanding loans via \
                 rmw_return_loaned_message_from_subscription; the queued frame is not \
                 consumed (repeats are logged at debug until a take succeeds)"
            ),
            LoanRefusal::ShadowAllocationFailed => tracing::error!(
                topic = %topic,
                kind = %kind.as_str(),
                outstanding_loans,
                budget,
                total_failures = total,
                "loaned take refused — could not allocate a shadow message (the heap \
                 allocation failed: memory pressure or a bug, NOT a retained loan, so there \
                 is nothing to return); the queued frame is not consumed (repeats are logged \
                 at debug until a take succeeds)"
            ),
            LoanRefusal::ReceiveFailed => tracing::error!(
                topic = %topic,
                kind = %kind.as_str(),
                outstanding_loans,
                budget,
                error = %error,
                total_failures = total,
                "loaned take refused — the transport refused the receive (see error=): \
                 ExceedsMaxBorrows means the subscription holds its whole borrow budget, \
                 return outstanding loans via rmw_return_loaned_message_from_subscription; \
                 any other reason is the transport's own condition; the queued frame is not \
                 consumed (repeats are logged at debug until a take succeeds)"
            ),
        },
        RegimeDecision::StillFailing { total, suppressed } => match kind {
            LoanRefusal::ShadowPoolExhausted => tracing::error!(
                topic = %topic,
                kind = %kind.as_str(),
                outstanding_loans,
                budget,
                total_failures = total,
                suppressed_count = suppressed,
                "loaned take STILL refused — the regime is open and every attempt is refused \
                 (all shadows loaned out): return outstanding loans via \
                 rmw_return_loaned_message_from_subscription"
            ),
            LoanRefusal::ShadowAllocationFailed => tracing::error!(
                topic = %topic,
                kind = %kind.as_str(),
                outstanding_loans,
                budget,
                total_failures = total,
                suppressed_count = suppressed,
                "loaned take STILL refused — the regime is open and every attempt is refused \
                 (shadow allocation keeps failing: memory pressure or a bug; nothing to return)"
            ),
            LoanRefusal::ReceiveFailed => tracing::error!(
                topic = %topic,
                kind = %kind.as_str(),
                outstanding_loans,
                budget,
                error = %error,
                total_failures = total,
                suppressed_count = suppressed,
                "loaned take STILL refused — the regime is open and every attempt is refused \
                 (the transport keeps refusing the receive, see error=; ExceedsMaxBorrows ⇒ \
                 return outstanding loans via rmw_return_loaned_message_from_subscription)"
            ),
        },
        RegimeDecision::Suppressed { suppressed } => match kind {
            LoanRefusal::ShadowPoolExhausted
            | LoanRefusal::ShadowAllocationFailed
            | LoanRefusal::ReceiveFailed => tracing::debug!(
                topic = %topic,
                kind = %kind.as_str(),
                outstanding_loans,
                budget,
                error = %error,
                total_failures = total,
                suppressed_count = suppressed,
                "loaned take refused (suppressed repeat)"
            ),
        },
    }
}

/// Record an ADOPTED take whose receive failed for a reason that is
/// NOT the borrow budget — a transport fault, reported with the adopted
/// path's own words.
///
/// It rides its OWN latch, not the borrow-refusal one, and that is the whole
/// point of the reporter rather than a detail of it.
///
/// Two reasons, both load-bearing. (1) WORDING: routing it through
/// [`LoanRefusal::ReceiveFailed`] would bound the log volume and then tell an
/// operator "loaned take refused — return outstanding loans", on a call that
/// took no loan and returns `RMW_RET_ERROR`. A remedy the reader cannot
/// perform is the same defect [`BorrowHolders`] exists to remove. (2) And
/// the one that actually matters: `FailureRegimeLatch` is kind-AGNOSTIC and
/// is closed only by a SUCCESS. A consumer retaining adopted messages
/// refuses on every take and returns `RMW_RET_OK`, so the borrow regime
/// never closes — and a genuine hard transport fault arriving during it
/// would land on `Suppressed`, i.e. `debug!`, i.e. invisible under the
/// shipped `rmw_cerulion=warn` filter, while still failing the call. One
/// open regime must never swallow another condition's loud head;
/// a recoverable refusal and a transport fault are different conditions with
/// different remedies, so they get different latches.
pub(crate) fn report_adopt_receive_failed(
    latch: &Mutex<FailureRegimeLatch>,
    topic: &str,
    error: &str,
) {
    let (decision, total) = observe_failure(latch);
    match decision {
        RegimeDecision::Loud => tracing::error!(
            topic = %topic,
            kind = "adopt_receive_failed",
            error = %error,
            total_failures = total,
            "adopted take FAILED — the transport refused the receive for a reason that is \
             NOT the borrow budget (see error=), so nothing this consumer holds can be \
             released to recover: the call returns an error and no frame is served \
             (repeats are logged at debug until a take succeeds)"
        ),
        RegimeDecision::StillFailing { total, suppressed } => tracing::error!(
            topic = %topic,
            kind = "adopt_receive_failed",
            error = %error,
            total_failures = total,
            suppressed_count = suppressed,
            "adopted take STILL FAILING — the regime is open and the transport keeps \
             refusing the receive (see error=); this is the transport's own condition"
        ),
        RegimeDecision::Suppressed { suppressed } => tracing::debug!(
            topic = %topic,
            kind = "adopt_receive_failed",
            error = %error,
            total_failures = total,
            suppressed_count = suppressed,
            "adopted take failed (suppressed repeat) — the transport's own condition, \
             see error="
        ),
    }
}

/// Close the adopted receive-failure regime.
///
/// A latch with no success side is worse than no latch at all: if
/// `report_adopt_receive_failed` opened
/// `AdoptTakeState::receive_failures` and nothing ever closed it, ONE
/// transport fault would silence every later fault on that subscription for the
/// life of the process — the same silence class the separate latch
/// exists to prevent, made permanent.
///
/// The recovery condition is a RECEIVE that succeeded, not a frame that was
/// served: this latch tracks whether the transport accepted the call. An
/// empty queue is a successful receive and closes it, which is both truthful
/// and the promptest recovery available — a subscription that starts polling
/// an idle topic after a fault has demonstrated exactly what the latch
/// doubts.
///
/// One recovery line, only when something was actually suppressed (the
/// module's standing rule: a lone loud failure re-arms silently rather than
/// doubling the log volume on a flapper), and the running total is NOT reset
/// — Principle #3.
pub(crate) fn report_adopt_receive_recovered(latch: &Mutex<FailureRegimeLatch>, topic: &str) {
    let (recovered, total) = observe_success(latch);
    if let Some(suppressed) = recovered {
        tracing::info!(
            topic = %topic,
            suppressed_count = suppressed,
            total_failures = total,
            "adopted take recovered — the transport is accepting receives again; the errors \
             suppressed while the regime was open are counted in suppressed_count"
        );
    }
}

/// WHICH take closed the refusal regime. Both take paths share one
/// subscription latch, so the recovery line must name the operation that
/// actually succeeded: an operator who saw `kind=adopted_budget_exhausted`
/// and is then told the LOAN budget recovered has been pointed at the
/// wrong API (the caller declares this rather
/// than the reporter guessing, which is what keeps the recovery keyed to a
/// real condition).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum LoanServed {
    /// `rmw_take_loaned_message*` served a shadow message.
    Loaned,
    /// A plain take served a frame by ADOPTION (adopt-take): ZERO
    /// payload copies were paid. Usually the sample is held; an
    /// all-empty forged frame holds nothing because there was nothing to
    /// hold, and is still `Adopted` — it copied nothing either. That is
    /// the same question `AdoptStats::adopted_takes` counts, and the
    /// two must never disagree.
    Adopted,
    /// The adoption branch served the frame by COPY: a registration
    /// failure fell back, or nothing in the frame was adoptable. These
    /// are exactly the takes counted in `AdoptStats::fallbacks`.
    /// Labelling them `Adopted` would tell an operator the zero-copy path was
    /// working on frames that paid a full copy.
    Copied,
}

impl LoanServed {
    /// The `served=` field value. Rendered through `Display` like the
    /// sibling `kind=`, so it greps as a bare token rather than a quoted
    /// string.
    pub(crate) fn as_str(self) -> &'static str {
        match self {
            LoanServed::Loaned => "loaned",
            LoanServed::Adopted => "adopted",
            LoanServed::Copied => "copied",
        }
    }
}

/// Record a SERVED take: closes an open refusal regime, logging the
/// recovery once (only when something was suppressed) and re-arming the
/// loud head. The steady-state healthy call is one predictable branch.
///
/// The wording describes what SUCCEEDED, never what is assumed to have
/// failed: one latch can carry both a loaned refusal and an adopted-budget
/// one, so a recovery cannot claim which of them ended without keeping
/// per-kind state the latch deliberately does not hold.
pub(crate) fn report_loan_served(
    latch: &Mutex<FailureRegimeLatch>,
    topic: &str,
    served: LoanServed,
) {
    let (recovered, total) = observe_success(latch);
    let Some(suppressed) = recovered else {
        return;
    };
    match served {
        LoanServed::Loaned => tracing::info!(
            topic = %topic,
            served = %served.as_str(),
            suppressed_count = suppressed,
            total_failures = total,
            "loaned take recovered — the loan budget is available again"
        ),
        LoanServed::Copied => tracing::info!(
            topic = %topic,
            served = %served.as_str(),
            suppressed_count = suppressed,
            total_failures = total,
            "adoption-path take recovered — the frame was served by COPY (nothing was \
             adopted on it), so the refusal regime is closed but the zero-copy path is \
             not what served this take"
        ),
        LoanServed::Adopted => tracing::info!(
            topic = %topic,
            served = %served.as_str(),
            suppressed_count = suppressed,
            total_failures = total,
            "adopted take recovered — an adopted take succeeded, so the borrow budget is \
             available again (whichever holder released: adopted messages freed, loans \
             returned, or the service's own capacity freed up — the kind= on the open \
             regime says which it was)"
        ),
    }
}

/// Record a forged take in which `below_floor` of the type's `forgeable`
/// entries sat below the frame's data floor (`data_floor`, payload-relative)
/// and were COPIED into the shadow instead of forged. The take succeeded and
/// served the same bytes the copying take would; what the operator learns is
/// that a producer emits entries the wire's placement rule forbids, and that
/// this topic is paying a copy for them.
pub(crate) fn report_forge_fallback(
    latch: &Mutex<FailureRegimeLatch>,
    topic: &str,
    below_floor: usize,
    forgeable: usize,
    data_floor: usize,
) {
    let (decision, total) = observe_failure(latch);
    match decision {
        RegimeDecision::Loud => tracing::warn!(
            topic = %topic,
            below_floor,
            forgeable,
            data_floor,
            total_failures = total,
            "forged loaned take fell back to COPYING a sequence: its offset-table entry sits \
             below the frame's data floor (inside the fixed section or the table) — the \
             producer emits a placement the wire forbids; the message was served with the \
             same bytes the copying take would serve, at the cost of a copy (repeats are \
             logged at debug until a well-placed frame arrives)"
        ),
        RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
            topic = %topic,
            below_floor,
            forgeable,
            data_floor,
            total_failures = total,
            suppressed_count = suppressed,
            "forged loaned take STILL falling back to copying — every frame from this \
             producer places a sequence entry below the data floor"
        ),
        RegimeDecision::Suppressed { suppressed } => tracing::debug!(
            topic = %topic,
            below_floor,
            forgeable,
            data_floor,
            total_failures = total,
            suppressed_count = suppressed,
            "forged loaned take fell back to copying (suppressed repeat)"
        ),
    }
}

/// Adopt-take: record an adopted take whose segment REGISTRATION
/// failed — the take was rolled back to the copy path (siblings
/// unregistered, the message un-forged and the masked members copied, the
/// sample dropped) and SUCCEEDED, so this is a degrade line, not a
/// refusal. Its own latch on `AdoptTakeState::registration_failures`: the
/// condition ("should be never" — an overlap means a stale leaked
/// registration; a bad-arg means a malformed range) is neither a retained
/// loan nor a producer-side placement, and one regime must not mask
/// another's loud head.
pub(crate) fn report_adopt_registration_fallback(
    latch: &Mutex<FailureRegimeLatch>,
    topic: &str,
    rc: i32,
    registered_before_failure: usize,
    ranges_total: usize,
) {
    let (decision, total) = observe_failure(latch);
    match decision {
        RegimeDecision::Loud => tracing::warn!(
            topic = %topic,
            rc,
            registered_before_failure,
            ranges_total,
            total_failures = total,
            "adopt-take segment registration FAILED — the take was served by the copy \
             path instead (siblings unregistered, sample released; rc −3 = bad range, \
             −4 = overlap with a live registration, i.e. a stale leaked registration); \
             repeats are logged at debug until a registration succeeds"
        ),
        RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
            topic = %topic,
            rc,
            registered_before_failure,
            ranges_total,
            total_failures = total,
            suppressed_count = suppressed,
            "adopt-take segment registration STILL failing — every adopted take on this \
             topic is being served by the copy path"
        ),
        RegimeDecision::Suppressed { suppressed } => tracing::debug!(
            topic = %topic,
            rc,
            registered_before_failure,
            ranges_total,
            total_failures = total,
            suppressed_count = suppressed,
            "adopt-take segment registration failed (suppressed repeat)"
        ),
    }
}

/// Adopt-take: record an adopted take whose registrations all
/// succeeded — closes an open registration-failure regime (one recovery
/// line iff something was suppressed) and re-arms the loud head.
///
/// The caller must have registered at least one range:
/// a frame whose forged entries are all empty registers
/// nothing, so it has demonstrated nothing about the hook's willingness to
/// register and must not close a regime it never tested.
pub(crate) fn report_adopt_registration_clean(latch: &Mutex<FailureRegimeLatch>, topic: &str) {
    let (recovered, total) = observe_success(latch);
    if let Some(suppressed) = recovered {
        tracing::info!(
            topic = %topic,
            suppressed_count = suppressed,
            total_failures = total,
            "adopt-take segment registration recovered"
        );
    }
}

/// Record a forged take in which every forgeable entry was forged: closes an
/// open fallback regime (one recovery line if anything was suppressed) and
/// re-arms.
pub(crate) fn report_forge_clean(latch: &Mutex<FailureRegimeLatch>, topic: &str) {
    let (recovered, total) = observe_success(latch);
    if let Some(suppressed) = recovered {
        tracing::info!(
            topic = %topic,
            suppressed_count = suppressed,
            total_failures = total,
            "forged loaned take recovered — frames place their sequences at or above the data \
             floor again"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// WHO holds the borrow budget, against a hand-written
    /// table. Every row is a shape a real subscription reaches: an adopt-armed
    /// type is always loan-capable too, so both counters are live at once.
    #[test]
    fn the_borrow_holder_classification_matches_its_oracle_table() {
        let oracle: &[(usize, usize, BorrowHolders, &str)] = &[
            (
                1,
                0,
                BorrowHolders::AdoptedOnly,
                "one retained adopted message and no loan",
            ),
            (
                16,
                0,
                BorrowHolders::AdoptedOnly,
                "a whole budget of adopted messages",
            ),
            (
                0,
                1,
                BorrowHolders::LoanedOnly,
                "one outstanding loan and no adopted message",
            ),
            (
                0,
                4,
                BorrowHolders::LoanedOnly,
                "the whole loan budget outstanding",
            ),
            (
                1,
                1,
                BorrowHolders::Both,
                "one of each — the mixed case the single-kind line could not describe",
            ),
            (3, 2, BorrowHolders::Both, "several of each"),
            (
                0,
                0,
                BorrowHolders::Unaccounted,
                "the counters say nobody — a release that landed between the refusal and \
                 this read, not a misconfiguration",
            ),
        ];
        for &(adopted, loaned, want, why) in oracle {
            assert_eq!(
                BorrowHolders::classify(adopted, loaned),
                want,
                "adopted={adopted} loaned={loaned}: {why}"
            );
        }
    }

    /// The classification only matters if the LINE changes with it: a `kind=`
    /// token and a remedy that were the same for every holder would leave the
    /// operator exactly where the un-classified line left them. All four
    /// tokens and all four remedies must be distinct — and the adopted-only
    /// token must be `adopted_budget_exhausted`, so an operator's standing
    /// grep keeps working.
    #[test]
    fn every_holder_carries_its_own_kind_token_and_remedy() {
        let all = [
            BorrowHolders::AdoptedOnly,
            BorrowHolders::LoanedOnly,
            BorrowHolders::Both,
            BorrowHolders::Unaccounted,
        ];
        for (i, a) in all.iter().enumerate() {
            for b in &all[i + 1..] {
                assert_ne!(a.as_str(), b.as_str(), "{a:?} and {b:?} share a kind token");
                assert_ne!(a.remedy(), b.remedy(), "{a:?} and {b:?} share a remedy");
            }
        }
        assert_eq!(
            BorrowHolders::AdoptedOnly.as_str(),
            "adopted_budget_exhausted",
            "the adopted-only token is the one operators already grep for"
        );
    }

    /// The remedies must not tell an operator to do something that cannot
    /// help: only a holder that actually HOLDS adopted messages may be told
    /// to free them, and only one holding loans may be told to return them.
    /// This is the whole defect — advice that does nothing.
    #[test]
    fn a_remedy_never_prescribes_a_release_the_holder_cannot_make() {
        let adopted_advice = |h: BorrowHolders| h.remedy().contains("free adopted messages");
        let loaned_advice = |h: BorrowHolders| h.remedy().contains("rmw_return_loaned_message");

        assert!(adopted_advice(BorrowHolders::AdoptedOnly));
        assert!(!loaned_advice(BorrowHolders::AdoptedOnly));

        assert!(loaned_advice(BorrowHolders::LoanedOnly));
        assert!(
            !adopted_advice(BorrowHolders::LoanedOnly),
            "a consumer whose borrows are ALL loans must not be told to free adopted messages \
             — that was the defect: {}",
            BorrowHolders::LoanedOnly.remedy()
        );

        assert!(adopted_advice(BorrowHolders::Both));
        assert!(loaned_advice(BorrowHolders::Both));

        assert!(
            !adopted_advice(BorrowHolders::Unaccounted)
                && !loaned_advice(BorrowHolders::Unaccounted),
            "with nothing held there is nothing to return: {}",
            BorrowHolders::Unaccounted.remedy()
        );
        // And it must not prescribe a RESTART either. The `(0, 0)` reading is
        // dominated by a release landing between the refused receive and the
        // counter read — a condition that heals by itself — so telling an
        // operator to stop a process over it is the same defect in a
        // different direction.
        for forbidden in ["restart", "stop the earlier creator"] {
            assert!(
                !BorrowHolders::Unaccounted.remedy().contains(forbidden),
                "the unaccounted remedy must not prescribe `{forbidden}` for what is \
                 usually a transient release race: {}",
                BorrowHolders::Unaccounted.remedy()
            );
        }
    }

    /// The adopted receive-failure latch RECOVERS and
    /// RE-ARMS — the half a failure-only latch lacks.
    ///
    /// A latch with no success side is worse than no latch at all. It opens
    /// on the first fault and nothing closes it, so the SECOND fault (and
    /// every fault after it, for the life of the subscription) is a
    /// `Suppressed` repeat at `debug!` — below the shipped
    /// `rmw_cerulion=warn` filter — while still failing the call. Bounding a
    /// flood would turn into a permanent silence after exactly one fault.
    ///
    /// The oracle is the sequence an operator actually lives through:
    /// fault, fault, recovery, fault. It asserts the recovery carries what
    /// was MISSED rather than the running total, that the total is NOT reset
    /// by recovery (Principle #3), and — the discriminating assertion — that
    /// the fault AFTER the success is `Loud` again. Without the close that
    /// last one is `Suppressed`, which is precisely the defect.
    #[test]
    fn the_adopted_receive_latch_recovers_and_re_arms() {
        let latch = Mutex::new(FailureRegimeLatch::new());
        // Two faults: the first is loud, the second suppressed.
        assert!(matches!(observe_failure(&latch).0, RegimeDecision::Loud));
        assert!(matches!(
            observe_failure(&latch).0,
            RegimeDecision::Suppressed { suppressed: 1 }
        ));
        // A successful receive closes the regime and reports what was missed.
        let (recovered, total) = observe_success(&latch);
        assert_eq!(
            recovered,
            Some(1),
            "the recovery reports the SUPPRESSED count — what the operator missed — not the \
             running total"
        );
        assert_eq!(total, 2, "...and recovery never resets the running total");
        // THE pin: the next fault must be loud again.
        let (decision, total_after) = observe_failure(&latch);
        assert!(
            matches!(decision, RegimeDecision::Loud),
            "a fault AFTER a successful receive must be LOUD again — a `Suppressed` here is \
             the permanent silence this close exists to prevent: got {decision:?}"
        );
        assert_eq!(total_after, 3, "and it still counts unconditionally");
        // A second success with nothing suppressed re-arms SILENTLY, so a
        // flapping transport does not double its own log volume.
        assert_eq!(
            observe_success(&latch).0,
            None,
            "a lone loud failure re-arms without a recovery line"
        );
    }

    /// The adopted path's TRANSPORT failure must be
    /// reported on its OWN latch, and that is a structural property no
    /// behavioural arm in this crate can reach.
    ///
    /// Why it matters is worth restating where the guard lives, because the
    /// bug it prevents is a SILENCE rather than a noise:
    /// `FailureRegimeLatch` is kind-agnostic and closes only on a SUCCESS. A
    /// consumer retaining adopted messages refuses on every take and returns
    /// `RMW_RET_OK`, so the borrow regime never closes — and a genuine hard
    /// transport fault arriving during that window, routed onto the same
    /// latch, lands on `Suppressed`, i.e. `debug!`, i.e. invisible under the
    /// shipped `rmw_cerulion=warn` filter, while still failing the call.
    /// Bounding the log volume would trade a flood for a silence.
    ///
    /// Driving that interleaving end to end needs a real transport fault on
    /// a saturated subscription, which the fake-hook harness cannot mint, so
    /// the property is pinned where it is decided: at the call site. The
    /// walk reads a COMMENT-STRIPPED view, because the comment three lines
    /// above the call names the very latch it must not use.
    #[test]
    fn the_adopted_receive_failure_is_reported_on_its_own_latch() {
        let src =
            std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/src/api/pubsub.rs"))
                .expect("pubsub.rs is readable");
        let code = strip_comments(&src);
        let call = code
            .find("report_adopt_receive_failed(")
            .expect("the adopted receive-failure reporter must be called from pubsub.rs");
        // The argument list ends at the first `)` that closes the call; the
        // latch is its first argument, so a short window is enough.
        let args = &code[call..(call + 200).min(code.len())];
        assert!(
            args.contains("adopt.receive_failures"),
            "the adopted receive failure must be reported on its OWN latch: {args}"
        );
        assert!(
            !args.contains("loan_refusals"),
            "...and NEVER on the shared borrow-refusal latch, whose open regime would \
             suppress a genuine transport fault to debug!: {args}"
        );
        // ...and the SUCCESS side must be wired on the same latch. A latch
        // whose failure reporter is called and whose recovery reporter is
        // not opens a regime nothing can close: one fault would then
        // silence every later fault for the
        // life of the subscription.
        let close = code
            .find("report_adopt_receive_recovered(")
            .expect("the receive-failure regime must be CLOSED somewhere in pubsub.rs");
        let close_args = &code[close..(close + 200).min(code.len())];
        assert!(
            close_args.contains("adopt.receive_failures"),
            "the close must be on the SAME latch the failure opens: {close_args}"
        );
        // Anti-tautology: the stripped view must still contain the code it
        // DOES have, or every absence assertion above is vacuous.
        assert!(
            code.contains("report_adopt_borrow_refused("),
            "the comment stripper must not have eaten the file"
        );
    }

    /// Line and block comments, block nesting included — the same shape the
    /// repo's other confinement walks use, because a walk over raw source
    /// would be satisfied (or defeated) by prose.
    fn strip_comments(src: &str) -> String {
        let mut out = String::with_capacity(src.len());
        let b = src.as_bytes();
        let (mut i, mut depth) = (0usize, 0usize);
        while i < b.len() {
            if depth == 0 && b[i..].starts_with(b"//") {
                while i < b.len() && b[i] != b'\n' {
                    i += 1;
                }
            } else if b[i..].starts_with(b"/*") {
                depth += 1;
                i += 2;
            } else if depth > 0 && b[i..].starts_with(b"*/") {
                depth -= 1;
                i += 2;
            } else {
                if depth == 0 {
                    out.push(b[i] as char);
                }
                i += 1;
            }
        }
        out
    }

    /// Every remedy reaches an operator VERBATIM on three regime arms, so its
    /// whitespace is part of the contract. This pin exists because a lost
    /// line continuation in one of these strings puts runs of
    /// eighteen literal spaces into the middle of the sentence — invisible to
    /// a `contains` assertion, and the first thing a reader would see.
    #[test]
    fn no_remedy_carries_a_run_of_literal_whitespace() {
        for h in [
            BorrowHolders::AdoptedOnly,
            BorrowHolders::LoanedOnly,
            BorrowHolders::Both,
            BorrowHolders::Unaccounted,
        ] {
            let remedy = h.remedy();
            assert!(
                !remedy.contains("  "),
                "{h:?}'s remedy carries a run of spaces (a lost `\\` continuation): {remedy}"
            );
            assert!(
                !remedy.contains('\n') && !remedy.contains('\t'),
                "{h:?}'s remedy must be one flat line for the log: {remedy}"
            );
        }
    }
}
