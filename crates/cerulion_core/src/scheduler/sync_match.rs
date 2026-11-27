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

    // ---- 30 Hz frame loss, period-minus-skew bound, and recovery ----------

    /// Minimal hand-driven two-input driver over an arrival schedule: each
    /// arrival pushes a frame to input 0 or 1, fills empty/emitted heads, and
    /// runs `next_sync_step` to quiescence (`Wait`). Records every non-Wait
    /// step, emitted `(a_ts, b_ts)` pair, discard count, and skip count.
    struct TwoInputTrace {
        steps: Vec<SyncStep>,
        emitted: Vec<(u64, u64)>,
        unmatched_discards: usize,
        closer_skips: usize,
    }

    fn drive_two_input_arrivals(
        arrivals: &[(usize, u64)],
        window: Option<Duration>,
    ) -> TwoInputTrace {
        let mut queues: [Vec<u64>; 2] = [Vec::new(), Vec::new()];
        let mut heads: [Option<SyncHead>; 2] = [None, None];
        let mut steps = Vec::new();
        let mut emitted = Vec::new();
        let mut unmatched_discards = 0usize;
        let mut closer_skips = 0usize;

        for &(idx, ts) in arrivals {
            queues[idx].push(ts);
            loop {
                for i in 0..2 {
                    let needs_refill = matches!(
                        heads[i],
                        None | Some(SyncHead {
                            state: HeadState::Emitted,
                            ..
                        })
                    );
                    if needs_refill && !queues[i].is_empty() {
                        heads[i] = Some(SyncHead::filled(queues[i].remove(0)));
                    }
                }
                let mut next = [NextInfo::Unknown; 2];
                let terminal = loop {
                    let step = next_sync_step(&heads, &next, window);
                    if step != SyncStep::Wait {
                        steps.push(step.clone());
                    }
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
                                Some(&p) => NextInfo::Stamp(p),
                                None => NextInfo::None,
                            };
                        }
                        SyncStep::Advance(i) => {
                            closer_skips += 1;
                            let promoted = queues[i].remove(0);
                            heads[i] = Some(SyncHead::filled(promoted));
                            next[i] = NextInfo::Unknown;
                        }
                        other => break other,
                    }
                };
                match terminal {
                    SyncStep::Wait => break,
                    SyncStep::Fire => {
                        let a_ts = heads[0].expect("fire requires filled head 0").ts;
                        let b_ts = heads[1].expect("fire requires filled head 1").ts;
                        emitted.push((a_ts, b_ts));
                        heads[0] = Some(SyncHead::emitted(a_ts));
                        heads[1] = Some(SyncHead::emitted(b_ts));
                    }
                    SyncStep::DiscardTie(indices) => {
                        for i in indices {
                            unmatched_discards += 1;
                            heads[i] = None;
                        }
                    }
                    SyncStep::NeedNext(..) | SyncStep::NeedStamp(_) | SyncStep::Advance(_) => {
                        unreachable!("handled in inner loop");
                    }
                }
            }
        }

        TwoInputTrace {
            steps,
            emitted,
            unmatched_discards,
            closer_skips,
        }
    }

    const PERIOD_30HZ_NS: u64 = 33_333_333;
    const SKEW_5MS_NS: u64 = 5_000_000;

    #[test]
    fn thirty_hz_loss_with_window_below_period_minus_skew_discards_orphan_and_recovers() {
        // Two 30 Hz streams where A (image) arrives 5 ms before B (capture time).
        // A_1 is dropped. With window = 20 ms < 33.333 ms - 5 ms = 28.333 ms,
        // (A_2, B_1) exceeds the window, discards orphan B_1 once, and recovers.
        let t = PERIOD_30HZ_NS;
        let s = SKEW_5MS_NS;
        let arrivals = [
            (0, 0),
            (1, s),
            // A_1 dropped; only B_1 arrives.
            (1, t + s),
            (0, 2 * t),
            (1, 2 * t + s),
            (0, 3 * t),
            (1, 3 * t + s),
        ];

        let trace = drive_two_input_arrivals(&arrivals, w(20));

        assert_eq!(
            trace.emitted,
            vec![(0, s), (2 * t, 2 * t + s), (3 * t, 3 * t + s)],
            "after discarding orphan B_1, frames 2 and 3 must pair cleanly"
        );
        assert_eq!(
            trace.unmatched_discards, 1,
            "only the orphaned B_1 must be discarded as UNMATCHABLE"
        );
        assert_eq!(trace.closer_skips, 0);
        assert!(
            trace.steps.contains(&SyncStep::DiscardTie(vec![1])),
            "orphan B_1 at index 1 must die via DiscardTie([1]): {:?}",
            trace.steps
        );
    }

    #[test]
    fn thirty_hz_loss_at_exact_period_minus_skew_boundary_pairs_wrong_sample() {
        // Span of (A_2, B_1) is exactly period - skew = 28_333_333 ns.
        let t = PERIOD_30HZ_NS;
        let s = SKEW_5MS_NS;
        let boundary_ns = t - s;

        let arrivals = [
            (0, 0),
            (1, s),
            (1, t + s),
            (0, 2 * t),
            (1, 2 * t + s),
            (0, 3 * t),
            (1, 3 * t + s),
        ];

        // Inclusive window == period - skew fires (A_2, B_1) and locks (A_3, B_2)
        // with zero discards and zero skips.
        let locked = drive_two_input_arrivals(&arrivals, Some(Duration::from_nanos(boundary_ns)));
        assert_eq!(
            locked.emitted,
            vec![(0, s), (2 * t, t + s), (3 * t, 2 * t + s)],
            "at exact window == period - skew, every pair after the loss shifts by one period"
        );
        assert_eq!(locked.unmatched_discards, 0);
        assert_eq!(locked.closer_skips, 0);

        // One nanosecond below the boundary discards orphan B_1 and recovers.
        let recovered =
            drive_two_input_arrivals(&arrivals, Some(Duration::from_nanos(boundary_ns - 1)));
        assert_eq!(
            recovered.emitted,
            vec![(0, s), (2 * t, 2 * t + s), (3 * t, 3 * t + s)],
            "one nanosecond below period - skew must discard B_1 and recover"
        );
        assert_eq!(recovered.unmatched_discards, 1);
        assert_eq!(recovered.closer_skips, 0);
    }

    #[test]
    fn thirty_hz_loss_under_unbounded_sync_phase_locks_every_subsequent_pair() {
        // Under unbounded_sync (window = None), one lost A_1 causes every
        // subsequent pair to lock onto (A_{k+1}, B_k) with zero discards/skips.
        let t = PERIOD_30HZ_NS;
        let s = SKEW_5MS_NS;
        let arrivals = [
            (0, 0),
            (1, s),
            (1, t + s),
            (0, 2 * t),
            (1, 2 * t + s),
            (0, 3 * t),
            (1, 3 * t + s),
            (0, 4 * t),
            (1, 4 * t + s),
        ];

        let trace = drive_two_input_arrivals(&arrivals, None);

        assert_eq!(
            trace.emitted,
            vec![
                (0, s),
                (2 * t, t + s),
                (3 * t, 2 * t + s),
                (4 * t, 3 * t + s),
            ],
            "unbounded_sync must phase-lock all three post-loss pairs onto (A_{{k+1}}, B_k)"
        );
        assert_eq!(trace.unmatched_discards, 0);
        assert_eq!(trace.closer_skips, 0);
    }

    #[test]
    fn thirty_hz_loss_with_reversed_skew_recovers_via_single_advance() {
        // B arrives 5 ms BEFORE A: B_k = k * T, A_k = k * T + skew.
        // When A_1 is dropped, B_2 arrives while A is still waiting, so when
        // A_2 lands B holds [B_1, B_2] and A has no next: the matcher advances
        // past B_1 once and fires the aligned (A_2, B_2) pair.
        let t = PERIOD_30HZ_NS;
        let s = SKEW_5MS_NS;
        let arrivals = [
            (1, 0),
            (0, s),
            (1, t),
            // A_1 dropped; B_2 lands before A_2.
            (1, 2 * t),
            (0, 2 * t + s),
            (1, 3 * t),
            (0, 3 * t + s),
        ];

        let trace = drive_two_input_arrivals(&arrivals, None);

        assert_eq!(
            trace.emitted,
            vec![(s, 0), (2 * t + s, 2 * t), (3 * t + s, 3 * t)],
            "reversed skew must recover via one Advance on B and pair (A_2, B_2)"
        );
        assert_eq!(trace.unmatched_discards, 0);
        assert_eq!(trace.closer_skips, 1);
        assert!(
            trace.steps.contains(&SyncStep::Advance(1)),
            "recovery must advance index 1 (B): {:?}",
            trace.steps
        );
    }

    #[test]
    fn thirty_hz_recovery_sequence_is_bit_identical_across_two_runs() {
        let t = PERIOD_30HZ_NS;
        let s = SKEW_5MS_NS;
        let arrivals = [
            (0, 0),
            (1, s),
            (1, t + s),
            (0, 2 * t),
            (1, 2 * t + s),
            (0, 3 * t),
            (1, 3 * t + s),
        ];

        let run1 = drive_two_input_arrivals(&arrivals, w(20));
        let run2 = drive_two_input_arrivals(&arrivals, w(20));

        let expected_emitted = vec![(0, s), (2 * t, 2 * t + s), (3 * t, 3 * t + s)];
        let expected_steps = vec![
            // (A_0, B_0)
            SyncStep::NeedNext(0, ProbeSite::Argmin),
            SyncStep::Fire,
            // (A_2, B_1): span 28.333 ms > 20 ms -> discard B_1
            SyncStep::DiscardTie(vec![1]),
            // (A_2, B_2)
            SyncStep::NeedNext(0, ProbeSite::Argmin),
            SyncStep::Fire,
            // (A_3, B_3)
            SyncStep::NeedNext(0, ProbeSite::Argmin),
            SyncStep::Fire,
        ];

        assert_eq!(run1.emitted, expected_emitted);
        assert_eq!(run1.steps, expected_steps);
        assert_eq!(run1.emitted, run2.emitted);
        assert_eq!(run1.steps, run2.steps);
        assert_eq!(run1.unmatched_discards, run2.unmatched_discards);
        assert_eq!(run1.closer_skips, run2.closer_skips);
    }

    // ---- sample(N) gates: admitted frames from different producer ticks ----
    //
    // A `sample(N)` gate admits a frame iff its wire stamp is at least N after
    // the gate's last admitted frame, and each input's gate keeps its own last
    // admitted stamp. The arrivals below are the frames the gates ADMIT from two
    // 30 ms producers (A on ticks 0, 30, 60, ..., B 5 ms later), walked by hand.

    /// Admitted frames after A's 60 ms frame is lost, `sample(31)` on both
    /// inputs. Gate A admits 0, refuses 30, admits 90 (90 - 0 >= 31), refuses
    /// 120, admits 150, refuses 180, admits 210. Gate B admits 5, 65, 125, 185.
    /// The gates now sit on different producer ticks, so no same-tick pair is
    /// ever admitted again.
    const SAMPLED_AFTER_DROP: [(usize, u64); 8] = [
        (0, 0),
        (1, 5 * MS),
        (1, 65 * MS),
        (0, 90 * MS),
        (1, 125 * MS),
        (0, 150 * MS),
        (1, 185 * MS),
        (0, 210 * MS),
    ];

    #[test]
    fn sampled_inputs_after_a_drop_pair_the_previous_tick_with_no_count() {
        // Window 32 ms, above the 30 ms producer period but below the 60 ms
        // admitted spacing: every pair after the drop is one producer tick
        // apart (25 ms span), and neither counter moves.
        let trace = drive_two_input_arrivals(&SAMPLED_AFTER_DROP, w(32));
        assert_eq!(
            trace.emitted,
            vec![
                (0, 5 * MS),
                (90 * MS, 65 * MS),
                (150 * MS, 125 * MS),
                (210 * MS, 185 * MS),
            ],
            "after the drop every fire pairs A with B from the previous tick"
        );
        assert_eq!(trace.unmatched_discards, 0);
        assert_eq!(trace.closer_skips, 0);
    }

    #[test]
    fn sampled_inputs_in_phase_pair_the_same_tick() {
        // Discriminator: the same gates with no drop admit A on 0, 60, 120,
        // 180 and B on 5, 65, 125, 185, so every pair is from one tick.
        let arrivals = [
            (0, 0),
            (1, 5 * MS),
            (0, 60 * MS),
            (1, 65 * MS),
            (0, 120 * MS),
            (1, 125 * MS),
            (0, 180 * MS),
            (1, 185 * MS),
        ];
        let trace = drive_two_input_arrivals(&arrivals, w(32));
        assert_eq!(
            trace.emitted,
            vec![
                (0, 5 * MS),
                (60 * MS, 65 * MS),
                (120 * MS, 125 * MS),
                (180 * MS, 185 * MS),
            ]
        );
        assert_eq!(trace.unmatched_discards, 0);
        assert_eq!(trace.closer_skips, 0);
    }

    #[test]
    fn sampled_inputs_after_a_drop_below_period_minus_skew_discard_loudly() {
        // Window 20 ms, below 30 ms - 5 ms: the out-of-phase frames are 25 or
        // 35 ms apart, so each one is discarded as UNMATCHABLE and counted.
        let trace = drive_two_input_arrivals(&SAMPLED_AFTER_DROP, w(20));
        assert_eq!(trace.emitted, vec![(0, 5 * MS)]);
        assert_eq!(
            trace.unmatched_discards, 5,
            "B65, A90, B125, A150 and B185 are each discarded once"
        );
        assert_eq!(trace.closer_skips, 0);
    }

    /// `sample(31)` on A only, no drop. Gate A admits 0, 60, 120; B delivers
    /// every 30 ms frame 5, 35, 65, 95, 125.
    const SAMPLED_A_PLAIN_B: [(usize, u64); 8] = [
        (0, 0),
        (1, 5 * MS),
        (1, 35 * MS),
        (0, 60 * MS),
        (1, 65 * MS),
        (1, 95 * MS),
        (0, 120 * MS),
        (1, 125 * MS),
    ];

    #[test]
    fn one_sampled_input_pairs_the_previous_tick_with_no_drop() {
        // Window 32 ms: A60 meets the leftover B35 (25 ms span) before B65
        // arrives, and A120 meets B95 after B65 is discarded.
        let trace = drive_two_input_arrivals(&SAMPLED_A_PLAIN_B, w(32));
        assert_eq!(
            trace.emitted,
            vec![(0, 5 * MS), (60 * MS, 35 * MS), (120 * MS, 95 * MS)]
        );
        assert_eq!(trace.unmatched_discards, 1, "only B65 is discarded");
        assert_eq!(trace.closer_skips, 0);
    }

    #[test]
    fn one_sampled_input_below_period_minus_skew_pairs_the_same_tick() {
        // Window 20 ms: the leftover B frame is discarded and A pairs with B
        // from its own tick.
        let trace = drive_two_input_arrivals(&SAMPLED_A_PLAIN_B, w(20));
        assert_eq!(
            trace.emitted,
            vec![(0, 5 * MS), (60 * MS, 65 * MS), (120 * MS, 125 * MS)]
        );
        assert_eq!(trace.unmatched_discards, 2, "B35 and B95 are discarded");
        assert_eq!(trace.closer_skips, 0);
    }
}
