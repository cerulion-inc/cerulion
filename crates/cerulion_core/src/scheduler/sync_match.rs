//! The per-set Sync MATCHER — the pure verdict engine that replaces
//! [`Scheduler::check_sync`](super::Scheduler).
//!
//! # What changed, and why a matcher rather than a predicate
//!
//! `check_sync` answered ONE question — "are all trigger inputs present and
//! within the window?" — and the scheduler then CLEARED every stamp on the
//! fire. A burst holding several aligned sets therefore collapsed to ONE fire
//! reading the freshest members, silently discarding the rest: the same class
//! already fixed for data triggers, surviving on Sync only because it was
//! documented.
//!
//! Per-set Sync fires once per COMPLETE aligned set, in set order, each trigger
//! input's message consumed by at most one set — the `fifo` semantic applied to
//! aligned sets (by design: `consume = fifo | batched | latest`). That
//! needs more than a predicate, because choosing a set's MEMBERS may require
//! transport work (a non-consuming "is there another frame?" probe, a pop into
//! held storage to learn a stamp, a pop-past to skip a frame). So the matcher is
//! a pure function that DEMANDS those facts through verdicts and is re-run after
//! each one, which keeps it oracle-testable with hand-built inputs and makes
//! every FFI crossing demand-driven.
//!
//! # The precedence order
//!
//! * **(P1) In-order** — per input, delivered member stamps are non-decreasing
//!   and consumption is FIFO. INVIOLABLE.
//! * **(P2) Arrived-set preservation** — a set that is complete and in-window
//!   among ARRIVED frames is never destroyed. INVIOLABLE.
//! * **(P3) Spread** — span minimisation (`max − min` over the tuple, the same
//!   quantity `sync_window_ms` bounds) is best-effort WITHIN P1+P2, realised as
//!   single-step strict-improvement descent on the argmin.
//!
//! P2 is what [`SyncStep::NeedNext`]'s gate exists for: descent is PERMITTED
//! only when at least one trigger input has NO second arrived frame. While every
//! input still holds two or more, a complete LATER tuple exists among arrived
//! frames and consuming an extra frame from any input might be consuming that
//! set's member — so the matcher refuses to descend and serves the backlog in
//! order. In a k-set backlog that means sets 1..k−1 serve greedily and only the
//! TAIL — where a partner has genuinely run out — may be descent-tightened.
//!
//! # R-Fail: a failed probe is never evidence that ENABLES descent
//!
//! Every transport op can fail (a poisoned cdylib `NODES` mutex, an iceoryx2
//! error). The mapping is POSITION-AWARE and FAIL-CLOSED, which is why
//! [`SyncStep::NeedNext`] carries a [`ProbeSite`]: at the ARGMIN, `None` means
//! "nothing to descend to" and therefore Fire (sound greedy); at the GATE,
//! `None` is the PASS witness, so a failed gate probe must resolve to
//! [`NextInfo::Present`] — that input REFUSES the gate — or a probe failure
//! could vouch for scarcity on an unverified input and let descent run into an
//! unprobed backlog, destroying arrived complete in-window sets (a P2
//! violation). The driver owns that mapping; the matcher's contribution is
//! telling it WHICH SITE asked.

use std::time::Duration;

/// Per-trigger-input head state: the frame this input contributes to the set
/// currently being formed.
///
/// Replaces the bare `u64` stamp of `sync_input_timestamps`. Two things the
/// stamp alone could not say:
///
/// * **`Emitted`** — the head was consumed by a fire and the input owes the
///   next set a fresh frame. The earlier code expressed this by CLEARING
///   the whole map on every fire, which is exactly what destroyed the rest of a
///   burst.
/// * **`Filled { backed: false }`** — a head restored from a bag names a stamp
///   but no live frame (see [`crate::state_restore`]). Descent is DISABLED for
///   the whole boundary when any head is unbacked: there is no frame to probe
///   past, and stealing a live queued frame would break the post-restore FIFO
///   rule that a queued frame belongs to the NEXT set.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeadState {
    /// A head is present. `backed` is `true` when a live transport frame sits
    /// behind it (the ordinary case), `false` for a head restored from a bag.
    Filled {
        /// Whether a live frame backs this head.
        backed: bool,
    },
    /// The head was consumed by a fire. A TOMBSTONE, not an absence: it is what
    /// tells the next align pass this input must be re-drained, and it is
    /// deliberately NOT captured into the framework section (see
    /// `Scheduler::node_framework_state`).
    Emitted,
}

/// One trigger input's contribution to the set under construction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SyncHead {
    /// The head frame's wire timestamp.
    pub ts: u64,
    /// Whether the head is available, and whether a live frame backs it.
    pub state: HeadState,
}

impl SyncHead {
    /// A live head backed by a transport frame.
    #[must_use]
    pub fn filled(ts: u64) -> Self {
        Self {
            ts,
            state: HeadState::Filled { backed: true },
        }
    }

    /// A head restored from a bag: a real stamp, no live frame behind it.
    #[must_use]
    pub fn restored(ts: u64) -> Self {
        Self {
            ts,
            state: HeadState::Filled { backed: false },
        }
    }

    /// The tombstone a fire leaves behind.
    #[must_use]
    pub fn emitted(ts: u64) -> Self {
        Self {
            ts,
            state: HeadState::Emitted,
        }
    }

    /// Is this head available as a set member?
    #[must_use]
    pub fn is_filled(&self) -> bool {
        matches!(self.state, HeadState::Filled { .. })
    }

    /// Is this head a restored one, naming a stamp with no live frame?
    #[must_use]
    pub fn is_unbacked(&self) -> bool {
        matches!(self.state, HeadState::Filled { backed: false })
    }
}

/// What the driver knows about the frame BEHIND one input's head.
///
/// Scratch, per align pass, never persisted: it describes the queue at that
/// input's PROBE INSTANT and is deliberately sampled once per input per pass
/// (continuous re-probing would make the verdict a race against arrivals WITHIN
/// the loop). The one exception is structural: the walking argmin's entry resets
/// to [`NextInfo::Unknown`] after each [`SyncStep::Advance`], because the staged
/// frame just became the head and whether ANOTHER sits behind it is genuinely
/// unknown.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NextInfo {
    /// Not probed yet this pass.
    Unknown,
    /// Probed: this input has NO second arrived frame.
    ///
    /// At the ARGMIN this means "nothing to descend to"; at the GATE it is the
    /// PASS witness, and R-Fail forbids a FAILED probe from ever writing it
    /// there.
    None,
    /// Probed: a second frame exists, stamp not yet known.
    Present,
    /// The second frame has been popped into held storage and its stamp read.
    Stamp(u64),
}

/// WHICH position asked for a probe — the R-Fail discriminator.
///
/// The driver cannot re-derive this without a second copy of the matcher's
/// argmin scan, and a `Failed` probe maps to the OPPOSITE answer at the two
/// sites, so the site rides the verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProbeSite {
    /// The walking argmin's own probe. A failure resolves to [`NextInfo::None`]
    /// ⇒ Fire (greedy membership, sound).
    Argmin,
    /// A gate probe on a non-argmin input. A failure resolves to
    /// [`NextInfo::Present`] ⇒ this input REFUSES the gate; the scan continues,
    /// so another input's GENUINE `None` can still pass it on real evidence.
    Gate,
}

/// The matcher's verdict: fire, wait, or a transport op the driver must perform
/// before re-running.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SyncStep {
    /// The set is not complete (or the pass cannot proceed). Do not fire.
    Wait,
    /// The current head tuple IS the set. Fire it.
    Fire,
    /// Window death: the tuple's span exceeds the window, so the listed inputs'
    /// heads are provably in NO set. Pop past each, count UNMATCHABLE, refill,
    /// re-run.
    ///
    /// Carries the whole lo tie-set because every tied-at-minimum head is
    /// equally unmatchable, and discarding only one would re-derive the same
    /// verdict on the next pass.
    DiscardTie(Vec<usize>),
    /// Ask the NON-CONSUMING "is there another arrived frame?" probe for input
    /// `.0`, then re-run. `.1` is the R-Fail site.
    NeedNext(usize, ProbeSite),
    /// Pop input `.0`'s next frame into held storage and report its stamp, then
    /// re-run. Issued only AFTER the gate has passed.
    NeedStamp(usize),
    /// Pop past input `.0`'s head (counted PASSED-OVER), promote its staged
    /// next, and re-run.
    Advance(usize),
}

/// The per-set Sync matcher — pure over
/// `(heads, next, window)`.
///
/// `heads` and `next` are indexed by DECLARATION position (never by the
/// `IndexMap`'s iteration order), which is what makes the gate's short-circuit
/// scan and the tie-break deterministic. `heads[i] == None` means that input has
/// no head at all.
///
/// # Termination
///
/// Every non-terminal verdict either consumes a frame ([`SyncStep::Advance`],
/// [`SyncStep::DiscardTie`]), resolves one [`NextInfo::Unknown`] to a
/// non-`Unknown` value ([`SyncStep::NeedNext`], [`SyncStep::NeedStamp`]), or the
/// driver terminates the pass on a failed op. All three are finite, and the
/// driver additionally clamps advances per input per boundary.
#[must_use]
pub fn next_sync_step(
    heads: &[Option<SyncHead>],
    next: &[NextInfo],
    window: Option<Duration>,
) -> SyncStep {
    // 0. LENGTH GUARD. A desync between the declared trigger
    //    set and the scratch is a wiring bug, and a matcher that guessed which
    //    slice to trust could fabricate a fire on a partial tuple. Waiting is
    //    the answer that cannot invent a set.
    if heads.is_empty() || heads.len() != next.len() {
        return SyncStep::Wait;
    }

    // 1. COMPLETENESS — presence only, never a timer and never a quorum. One
    //    pass also collects the span, so the scan is single-pass.
    let mut lo = u64::MAX;
    let mut hi = 0u64;
    let mut any_unbacked = false;
    for head in heads {
        let Some(head) = head else {
            return SyncStep::Wait;
        };
        match head.state {
            HeadState::Filled { backed } => {
                if !backed {
                    any_unbacked = true;
                }
            }
            HeadState::Emitted => return SyncStep::Wait,
        }
        lo = lo.min(head.ts);
        hi = hi.max(head.ts);
    }

    // 2. DEATH, before descent (bounded mode only). A provably unmatchable frame
    //    must be classified as UNMATCHABLE — "something is wrong" — rather than
    //    as a descent skip, which is "the feature working". The two counters are
    //    the operator's whole diagnosis, so the ORDER is the taxonomy.
    //
    //    `>` (not `>=`) mirrors `check_sync`'s `(max − min) <= window_ns`.
    if let Some(window) = window {
        let window_ns = window.as_nanos() as u64;
        if hi - lo > window_ns {
            return SyncStep::DiscardTie(lo_tie_set(heads, lo));
        }
    }

    // 3. DESCENT, gated.

    // A restore boundary disables descent entirely: an unbacked head names no
    // frame to probe past, and a live queued frame belongs to the NEXT set.
    // Costs exactly one boundary of greedy membership after a restore.
    if any_unbacked {
        return SyncStep::Fire;
    }

    // The tie at lo IS the guard: advancing one of several inputs tied at the
    // minimum cannot reduce the span (the others still pin it), so there is one
    // semantics rather than a separate tie-break rule.
    let mut argmin = 0usize;
    let mut lo_count = 0usize;
    for (i, head) in heads.iter().enumerate() {
        let ts = head.expect("completeness proved every head is Some").ts;
        if ts == lo {
            if lo_count == 0 {
                argmin = i;
            }
            lo_count += 1;
        }
    }
    if lo_count > 1 {
        return SyncStep::Fire;
    }
    let i = argmin;

    // The argmin's OWN probe first: with no second frame there is nothing to
    // descend to, whatever the other inputs hold. This ordering is also what
    // makes a TOTAL op-failure regime cheap — the first probe fails, resolves to
    // `None` under R-Fail, and the pass fires greedy without ever scanning the
    // gate.
    match next[i] {
        NextInfo::Unknown => return SyncStep::NeedNext(i, ProbeSite::Argmin),
        NextInfo::None => return SyncStep::Fire,
        NextInfo::Present | NextInfo::Stamp(_) => {}
    }

    // THE GATE, short-circuit, DECLARATION order. It passes the
    // moment one input provably has no second arrived frame; every input still
    // holding one is a reason to refuse, because a complete LATER tuple then
    // exists among arrived frames.
    let mut gate_passed = false;
    for (j, info) in next.iter().enumerate() {
        if j == i {
            continue;
        }
        match info {
            // GENUINE evidence of scarcity — R-Fail forbids a failed gate probe
            // from writing this.
            NextInfo::None => {
                gate_passed = true;
                break;
            }
            NextInfo::Unknown => return SyncStep::NeedNext(j, ProbeSite::Gate),
            NextInfo::Present | NextInfo::Stamp(_) => {}
        }
    }
    if !gate_passed {
        // Serve the backlog in order. This is the P2 arm.
        return SyncStep::Fire;
    }

    // Only now is a POP justified.
    let NextInfo::Stamp(p) = next[i] else {
        return SyncStep::NeedStamp(i);
    };

    // Judge the overshoot on its REAL value: advancing the argmin past `p` can
    // move the maximum (if `p` overshoots `hi`) and moves the minimum to
    // `min(lo2, p)`, where `lo2` is the second-smallest head — the new minimum
    // once `i`'s old head is gone.
    let lo2 = second_smallest(heads, i);
    let new_span = hi.max(p) - lo2.min(p);
    if new_span < hi - lo {
        // Strict: a plateau stops the walk. This is the earliest-min tie-break
        // the matcher applies.
        return SyncStep::Advance(i);
    }
    SyncStep::Fire
}

/// Every index whose head sits at the minimum stamp.
fn lo_tie_set(heads: &[Option<SyncHead>], lo: u64) -> Vec<usize> {
    // Plain prose, NOT an annotation: `heads` allocates nothing, so a marker here
    // arms nothing and only lands in the lint's unmatched-annotation banner. The
    // allocating line is the `.collect()` at the end of this chain, and it carries
    // the marker itself.
    //
    // The DEATH path only. `DiscardTie` is reached when a partner has run more
    // than a window past this tuple, i.e. UNMATCHABLE — rate 0 on a healthy graph,
    // by the same accounting that makes it worth a loud regime latch. A
    // steady-state alignment never reaches here; the descent's own tie handling
    // counts in place (`lo_count`) and allocates nothing.
    heads
        .iter()
        .enumerate()
        .filter(|(_, head)| head.is_some_and(|h| h.ts == lo))
        .map(|(i, _)| i)
        // hot-path-alloc-ok: the DEATH path only — see the note at the top of this fn
        .collect()
}

/// The smallest head stamp EXCLUDING index `skip` — the minimum the tuple would
/// have if `skip`'s head were removed.
///
/// Reached only after the gate has passed, which requires at least one input
/// other than the argmin, so the iterator is never empty in production; the
/// `unwrap_or` keeps a desynced caller on a defined (and maximally
/// descent-refusing) answer instead of a panic.
fn second_smallest(heads: &[Option<SyncHead>], skip: usize) -> u64 {
    heads
        .iter()
        .enumerate()
        .filter(|(i, _)| *i != skip)
        .filter_map(|(_, head)| head.as_ref().map(|h| h.ts))
        .min()
        .unwrap_or(u64::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1_000_000;

    fn w(ms: u64) -> Option<Duration> {
        Some(Duration::from_millis(ms))
    }

    fn filled(stamps: &[u64]) -> Vec<Option<SyncHead>> {
        stamps
            .iter()
            .map(|&ts| Some(SyncHead::filled(ts)))
            .collect()
    }

    // ---- completeness + length guard -------------------------------------

    #[test]
    fn a_missing_head_waits() {
        let heads = vec![Some(SyncHead::filled(0)), None];
        let next = vec![NextInfo::Unknown; 2];
        assert_eq!(next_sync_step(&heads, &next, w(50)), SyncStep::Wait);
    }

    #[test]
    fn an_emitted_head_waits_it_is_a_tombstone_not_a_member() {
        let heads = vec![Some(SyncHead::filled(0)), Some(SyncHead::emitted(10 * MS))];
        let next = vec![NextInfo::Unknown; 2];
        assert_eq!(next_sync_step(&heads, &next, w(50)), SyncStep::Wait);
    }

    #[test]
    fn a_scratch_length_desync_waits_rather_than_guessing() {
        let heads = filled(&[0, 10 * MS]);
        assert_eq!(
            next_sync_step(&heads, &[NextInfo::None], w(50)),
            SyncStep::Wait
        );
        assert_eq!(next_sync_step(&[], &[], w(50)), SyncStep::Wait);
    }

    // ---- death, and its precedence over descent ---------------------------

    #[test]
    fn a_span_past_the_window_dies_before_any_descent() {
        // A@0 is 210 ms from B@210 with a 50 ms window. Even though A has a
        // next frame and B does not (the descent-permitting shape), the verdict
        // must be DEATH — the classification, not the skip.
        let heads = filled(&[0, 210 * MS]);
        let next = vec![NextInfo::Stamp(210 * MS), NextInfo::None];
        assert_eq!(
            next_sync_step(&heads, &next, w(50)),
            SyncStep::DiscardTie(vec![0])
        );
    }

    #[test]
    fn the_window_boundary_is_inclusive_exactly_as_check_sync_was() {
        let heads = filled(&[0, 50 * MS]);
        let next = vec![NextInfo::None, NextInfo::None];
        // span == window: alive.
        assert_eq!(next_sync_step(&heads, &next, w(50)), SyncStep::Fire);
        // one nanosecond over: dead.
        let heads = filled(&[0, 50 * MS + 1]);
        assert_eq!(
            next_sync_step(&heads, &next, w(50)),
            SyncStep::DiscardTie(vec![0])
        );
    }

    #[test]
    fn death_discards_the_whole_lo_tie_set() {
        let heads = filled(&[0, 0, 500 * MS]);
        let next = vec![NextInfo::Unknown; 3];
        assert_eq!(
            next_sync_step(&heads, &next, w(50)),
            SyncStep::DiscardTie(vec![0, 1])
        );
    }

    #[test]
    fn unbounded_sync_never_dies() {
        let heads = filled(&[0, 3_600 * 1_000 * MS]);
        let next = vec![NextInfo::None, NextInfo::None];
        assert_eq!(next_sync_step(&heads, &next, None), SyncStep::Fire);
    }

    // ---- the gate ---------------------------------------------------------

    #[test]
    fn the_gate_refuses_while_every_input_holds_a_second_arrived_frame() {
        // The backlog shape: A=[1,2,3], B=[10,20,30]. Both inputs have a
        // next, so a complete LATER tuple exists among arrived frames and the
        // matcher must serve the head tuple in order (P2).
        let heads = filled(&[1, 10]);
        let next = vec![NextInfo::Present, NextInfo::Present];
        assert_eq!(next_sync_step(&heads, &next, w(50)), SyncStep::Fire);
    }

    #[test]
    fn the_gate_passes_on_a_genuinely_scarce_partner_and_asks_for_the_stamp() {
        let heads = filled(&[0, 10 * MS]);
        let next = vec![NextInfo::Present, NextInfo::None];
        assert_eq!(next_sync_step(&heads, &next, w(50)), SyncStep::NeedStamp(0));
    }

    #[test]
    fn the_argmins_own_probe_is_asked_first_and_carries_the_argmin_site() {
        let heads = filled(&[0, 10 * MS]);
        let next = vec![NextInfo::Unknown, NextInfo::Unknown];
        assert_eq!(
            next_sync_step(&heads, &next, w(50)),
            SyncStep::NeedNext(0, ProbeSite::Argmin)
        );
    }

    #[test]
    fn a_gate_probe_carries_the_gate_site_and_the_scan_is_declaration_ordered() {
        // Argmin is imu (index 2); the gate scans cam (0) then lidar (1) and
        // must ask for the FIRST unresolved one in declaration order.
        let heads = filled(&[45 * MS, 50 * MS, 0]);
        let next = vec![NextInfo::Unknown, NextInfo::Unknown, NextInfo::Present];
        assert_eq!(
            next_sync_step(&heads, &next, w(50)),
            SyncStep::NeedNext(0, ProbeSite::Gate)
        );
    }

    #[test]
    fn the_gate_short_circuits_at_the_first_scarce_input() {
        // cam has no next ⇒ the gate passes without ever consulting lidar,
        // whose entry stays Unknown. If the scan probed everything, this would
        // return NeedNext(1, Gate) instead.
        let heads = filled(&[45 * MS, 50 * MS, 0]);
        let next = vec![NextInfo::None, NextInfo::Unknown, NextInfo::Present];
        assert_eq!(next_sync_step(&heads, &next, w(50)), SyncStep::NeedStamp(2));
    }

    #[test]
    fn an_argmin_with_no_successor_fires_without_scanning_the_gate() {
        let heads = filled(&[0, 10 * MS]);
        let next = vec![NextInfo::None, NextInfo::Present];
        assert_eq!(next_sync_step(&heads, &next, w(50)), SyncStep::Fire);
    }

    // ---- descent arithmetic ----------------------------------------------

    #[test]
    fn a_strictly_improving_step_advances() {
        // This example: heads (45, 50, 0), imu's next is @10 ⇒
        // new_span = max(50,10) − min(45,10) = 40 < 50.
        let heads = filled(&[45 * MS, 50 * MS, 0]);
        let next = vec![NextInfo::None, NextInfo::Present, NextInfo::Stamp(10 * MS)];
        assert_eq!(next_sync_step(&heads, &next, w(50)), SyncStep::Advance(2));
    }

    #[test]
    fn a_plateau_stops_the_walk_which_is_the_ruled_tie_break() {
        // A=[0,20], B=[10]: advancing A@0 to A@20 gives span 10 — EQUAL, not
        // better. Strict descent refuses, so the earliest frame is served.
        let heads = filled(&[0, 10 * MS]);
        let next = vec![NextInfo::Stamp(20 * MS), NextInfo::None];
        assert_eq!(next_sync_step(&heads, &next, w(50)), SyncStep::Fire);
    }

    #[test]
    fn an_overshoot_is_judged_on_its_real_value_not_assumed_better() {
        // A=[0,60], B=[10]: advancing A gives span |60−10| = 50 > 10. Refuse.
        let heads = filled(&[0, 10 * MS]);
        let next = vec![NextInfo::Stamp(60 * MS), NextInfo::None];
        assert_eq!(next_sync_step(&heads, &next, None), SyncStep::Fire);
    }

    #[test]
    fn a_tie_at_the_minimum_never_descends() {
        let heads = filled(&[0, 0, 10 * MS]);
        let next = vec![NextInfo::Present; 3];
        assert_eq!(next_sync_step(&heads, &next, w(50)), SyncStep::Fire);
    }

    #[test]
    fn the_second_smallest_is_the_new_minimum_after_the_argmin_leaves() {
        // heads (0, 30, 40); argmin 0's next is @35. Removing head 0 leaves
        // lo2 = 30, so new_span = max(40,35) − min(30,35) = 10 < 40.
        let heads = filled(&[0, 30, 40]);
        let next = vec![NextInfo::Stamp(35), NextInfo::Present, NextInfo::None];
        assert_eq!(next_sync_step(&heads, &next, None), SyncStep::Advance(0));
    }

    // ---- restore ----------------------------------------------------------

    #[test]
    fn an_unbacked_head_disables_descent_for_the_whole_boundary() {
        // Every condition for a descent is met EXCEPT that one head is a
        // restored one. The matcher must fire greedily and issue no op.
        let heads = vec![Some(SyncHead::restored(0)), Some(SyncHead::filled(10 * MS))];
        let next = vec![NextInfo::Stamp(9 * MS), NextInfo::None];
        assert_eq!(next_sync_step(&heads, &next, w(50)), SyncStep::Fire);
    }

    #[test]
    fn an_unbacked_head_does_not_suppress_death() {
        // Descent is disabled, but the window still governs — a restored head
        // whose partner is far away is still provably unmatchable.
        let heads = vec![
            Some(SyncHead::restored(0)),
            Some(SyncHead::filled(500 * MS)),
        ];
        let next = vec![NextInfo::Unknown; 2];
        assert_eq!(
            next_sync_step(&heads, &next, w(50)),
            SyncStep::DiscardTie(vec![0])
        );
    }

    // ---- degenerate shapes -------------------------------------------------

    #[test]
    fn a_single_input_sync_fires_without_a_gate_to_pass() {
        // Degenerate-but-functional (the wiring guard warns rather than
        // refuses). There is no j != i, so the gate cannot pass and the matcher
        // must never reach `second_smallest`.
        let heads = filled(&[7]);
        for info in [
            NextInfo::Present,
            NextInfo::Stamp(8),
            NextInfo::Unknown,
            NextInfo::None,
        ] {
            let step = next_sync_step(&heads, &[info], w(50));
            assert!(
                matches!(
                    step,
                    SyncStep::Fire | SyncStep::NeedNext(0, ProbeSite::Argmin)
                ),
                "single-input sync must fire or probe its own argmin, got {step:?}"
            );
        }
    }

    #[test]
    fn identical_stamps_everywhere_fire_at_span_zero() {
        let heads = filled(&[5, 5, 5]);
        let next = vec![NextInfo::Present; 3];
        assert_eq!(next_sync_step(&heads, &next, w(0)), SyncStep::Fire);
    }

    // ---- the agreed 3-topic walk, driven end to end -----------------------

    /// A hand-driven transcription of the normative table for this walk: the driver's
    /// loop with the transport replaced by a hand-written queue oracle. Asserts
    /// the WHOLE op sequence, not just the outcome, so a matcher that reached
    /// the same set by different ops fails.
    #[test]
    fn the_ruled_three_topic_walk_reproduces_the_signed_off_op_sequence() {
        // cam = [45], lidar = [50], imu = [0, 10, 20, 30, 40] (ms).
        let queues: [Vec<u64>; 3] = [vec![], vec![], vec![10, 20, 30, 40]];
        let mut queues = queues.map(|q| q.into_iter().map(|ms| ms * MS).collect::<Vec<_>>());
        let mut heads = filled(&[45 * MS, 50 * MS, 0]);
        let mut next = vec![NextInfo::Unknown; 3];
        let mut ops: Vec<SyncStep> = Vec::new();
        let mut skips = [0u32; 3];

        let verdict = loop {
            let step = next_sync_step(&heads, &next, w(50));
            ops.push(step.clone());
            match step {
                SyncStep::NeedNext(i, _) => {
                    next[i] = if queues[i].is_empty() {
                        NextInfo::None
                    } else {
                        NextInfo::Present
                    };
                }
                SyncStep::NeedStamp(i) => {
                    next[i] = match queues[i].first() {
                        Some(&ts) => NextInfo::Stamp(ts),
                        None => NextInfo::None,
                    };
                }
                SyncStep::Advance(i) => {
                    skips[i] += 1;
                    let promoted = queues[i].remove(0);
                    heads[i] = Some(SyncHead::filled(promoted));
                    next[i] = NextInfo::Unknown;
                }
                other => break other,
            }
            assert!(ops.len() < 64, "walk did not terminate: {ops:?}");
        };

        assert_eq!(verdict, SyncStep::Fire);
        assert_eq!(
            heads,
            filled(&[45 * MS, 50 * MS, 40 * MS]),
            "the required set is (cam=45, lidar=50, imu=40), span 10"
        );
        assert_eq!(skips, [0, 0, 4], "imu skipped 0, 10, 20 and 30");
        assert_eq!(
            ops,
            vec![
                // 1: argmin imu, unknown.
                SyncStep::NeedNext(2, ProbeSite::Argmin),
                // 2: gate scans cam first and short-circuits on its `None`.
                SyncStep::NeedNext(0, ProbeSite::Gate),
                // 3-4: pop imu@10, span 50 -> 40.
                SyncStep::NeedStamp(2),
                SyncStep::Advance(2),
                // 5-7: three more strictly-improving steps, each re-probing
                // ONLY the walking argmin (lidar is never probed at all).
                SyncStep::NeedNext(2, ProbeSite::Argmin),
                SyncStep::NeedStamp(2),
                SyncStep::Advance(2),
                SyncStep::NeedNext(2, ProbeSite::Argmin),
                SyncStep::NeedStamp(2),
                SyncStep::Advance(2),
                SyncStep::NeedNext(2, ProbeSite::Argmin),
                SyncStep::NeedStamp(2),
                SyncStep::Advance(2),
                // 8: imu's queue is exhausted — nothing to descend to.
                SyncStep::NeedNext(2, ProbeSite::Argmin),
                SyncStep::Fire,
            ],
            "the op sequence must match the signed-off table"
        );
        assert_eq!(
            next[1],
            NextInfo::Unknown,
            "lidar is never probed — the gate short-circuits at cam"
        );
    }
}
