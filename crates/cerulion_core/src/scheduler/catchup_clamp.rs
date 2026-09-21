// SPDX-License-Identifier: AGPL-3.0-only
//! The `max_catchup` CLAMP a state-armed run applies to
//! its `Period` nodes, and the SEAM the scheduler reads it through.
//!
//! # Why a clamp exists at all
//!
//! On the three WALL-GATED shipping paths (`graph run` monolith, `graph
//! profile`, `graph run --record --single-process`) the gating clock advances
//! by the loop's WALL elapsed, so anything that stalls the loop advances
//! logical time by the same amount — and a `Period` node's catch-up burst is
//! bounded by `max_catchup.unwrap_or(u32::MAX)`, i.e. by NOTHING. A 5 ms stall
//! therefore fires a 1 kHz `Period` node **five extra times in that one step**:
//! a changed fire set, a changed trace, changed published frames. That is
//! logic perturbation, not jitter.
//!
//! A checkpoint anchor is exactly such a stall. The rule
//! is therefore: **while a state arm is attached, clamp
//! `max_catchup` to a small default unless the node set one explicitly** — from
//! the FIRST ship, not when the fork carrier lands.
//!
//! That list is the set of paths where an unbounded burst is POSSIBLE, not the
//! set that is armed. `cerulion_cli_engine::state_arm_attach` owns the wiring
//! and states which runs actually carry an arm: the `graph run` monolith, each
//! `graph run-worker`, and `graph run --record`. `graph profile` is deliberately
//! NOT armed — it is a MEASUREMENT verb whose output feeds the auto-partitioner,
//! and a profile taken while a recorder is anchoring the graph is already an
//! invalid measurement of the unrecorded robot, so clamping it would change the
//! costs rather than protect anything.
//!
//! # The onset is `armed && step >= first_anchor_step`, NOT bare `is_armed`
//!
//! A multi-process run is many processes stepping in barrier lockstep, and a
//! recorder ARMS the word at an instant none of them share. Keying on
//! `is_armed()` alone would let rank 0 observe the flip at step `N` and rank 1
//! at step `N+1`, so for one step the two ranks would run DIFFERENT catch-up
//! caps — a divergence the barrier cannot repair and replay cannot reproduce.
//!
//! `first_anchor_step` is the agreed onset: `crate::state_arm::StateArmWord::arm`
//! publishes it (and the cadence) BEFORE the `armed` flag with `Release`, so a
//! reader that `Acquire`-loads `armed` first observes the very same step number
//! every peer does. Gating on `step >= first_anchor_step` therefore flips every
//! rank at ONE step by construction, whatever wall instant each of them noticed
//! the arm at. It is the same `>=` boundary `crate::state_arm::cadence_due`
//! uses, for the same reason.
//!
//! (Every `crate::state_arm` mention in this file is deliberately PROSE in
//! backticks rather than an intra-doc link: this module is portable and that
//! one is `#[cfg(unix)]`, so a link is unresolvable on a non-unix target and
//! fails CI's Documentation job with `rustdoc::broken_intra_doc_links` —
//! pinned by `cfg_audit_test`.)
//!
//! # An explicit `max_catchup` always wins
//!
//! The clamp replaces a DEFAULT (`None` ⇒ unbounded), never a declaration. A
//! node that says `max_catchup = Some(N)` has stated its own burst policy, and
//! silently overriding it while a recorder happens to be attached would make
//! the recorder change the robot's behaviour in the one way this whole module
//! exists to prevent.
//!
//! # What this module is NOT
//!
//! It carries no clock, no SHM and no `#[cfg]`: the decision is two pure
//! functions over three integers, and the ARM is reached through the
//! [`CatchupArm`] trait so the scheduler compiles (and is oracle-testable)
//! everywhere while the only production implementor —
//! `crate::state_arm::MappedStateArm` — is `#[cfg(unix)]` like the rest of
//! the checkpoint substrate. Which is also why that name is prose and not a
//! link: see the note above.

/// One ORDERED observation of the arm word's two onset facts.
///
/// Read together or not at all: `armed` is the `Acquire` edge that publishes
/// `first_anchor_step` (see the module docs), so a reader that took them as two
/// independent loads could pair a fresh `armed` with a PREVIOUS attach's step
/// number.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArmOnset {
    /// Whether a recorder is currently attached.
    pub armed: bool,
    /// The step at which this attach's first anchor is due — the agreed onset
    /// every rank flips at.
    pub first_anchor_step: u64,
}

impl ArmOnset {
    /// The reading of a run with NO arm attached: nothing is clamped.
    pub const DETACHED: Self = Self {
        armed: false,
        first_anchor_step: 0,
    };
}

/// The seam the scheduler reads the arm through, ONCE per step.
///
/// A trait rather than a concrete handle for two reasons. (1) The production
/// arm is a `MAP_SHARED` POSIX-SHM mapping and therefore `#[cfg(unix)]`, while
/// the scheduler is not — threading the concrete type would `#[cfg]`-gate a
/// scheduler field and a `begin_step` branch. (2) The once-per-step READ
/// DISCIPLINE is only checkable by a source that can COUNT its readings, which
/// a real mapping cannot do.
pub trait CatchupArm: Send + Sync {
    /// Read the arm's onset facts as one ordered observation.
    fn onset(&self) -> ArmOnset;
}

/// An arm whose reading is FIXED: what a REPLAY installs.
///
/// # Why replay needs one at all
///
/// The clamp shapes LIVE fire decisions, and without this arm a replay
/// reconstructs none of it: `effective_max_catchup(declared, None)` returns
/// `u32::MAX` and a replay runs the FULL catch-up burst on exactly the stalled
/// steps a recording clamped to four. Since the Flashback plane is always-on,
/// essentially every recorded live run is clamped and an arm-less replay of it
/// is not — a divergence at precisely the stall steps an incident capture
/// exists to hold.
///
/// # Why a fixed reading is the CORRECT reading, not a simplification
///
/// A live arm is a `MAP_SHARED` word a recorder can flip at any instant. A
/// replay has no recorder and no word; what it has is the recording's own
/// statement of the plane its anchors were taken under
/// (`__cerulion/state_coverage.json`'s `armed` block, which carries exactly
/// `first_anchor_step`). Reading that once and holding it is not an
/// approximation of the live behaviour — it is the only construction that is
/// deterministic, and Principle #7 forbids a fire set that depends on when a
/// wall instant fell.
///
/// A recording that names no armed plane installs NOTHING and replays exactly as
/// it did before this existed.
#[derive(Debug, Clone, Copy)]
pub struct StaticCatchupArm {
    onset: ArmOnset,
}

impl StaticCatchupArm {
    /// An arm reporting `onset` on every reading.
    pub fn new(onset: ArmOnset) -> Self {
        Self { onset }
    }

    /// The arm a recording made under a plane armed from `first_anchor_step`.
    pub fn armed_from(first_anchor_step: u64) -> Self {
        Self::new(ArmOnset {
            armed: true,
            first_anchor_step,
        })
    }

    /// What it reports — the Principle #3 observable.
    pub fn onset(&self) -> ArmOnset {
        self.onset
    }
}

impl CatchupArm for StaticCatchupArm {
    fn onset(&self) -> ArmOnset {
        self.onset
    }
}

/// The `max_catchup` a state-armed run applies to a `Period` node that declared
/// none.
///
/// 4 is the recommended value ("clamp `max_catchup` to a small
/// default (say 4)"). It is a BURST bound, not a rate bound: a node still fires
/// at its declared period on every ordinary step, and the cap only bites on the
/// step AFTER a stall, where it trades a few dropped catch-up fires for a fire
/// set that does not depend on how long the recorder took.
pub const ARMED_MAX_CATCHUP_DEFAULT: u32 = 4;

/// PURE: the per-step cap override, derived from ONE arm reading and the step
/// being executed.
///
/// `None` = no clamp this step (no arm, disarmed, or the agreed onset has not
/// been reached). `Some(cap)` = clamp every `Period` node that declared no
/// `max_catchup` of its own.
pub fn derive_catchup_cap_override(onset: ArmOnset, step: u64) -> Option<u32> {
    // `>=`, matching `cadence_due`: the onset step is the FIRST clamped step,
    // because it is the step whose boundary carries the anchor that stalls it.
    if onset.armed && step >= onset.first_anchor_step {
        Some(ARMED_MAX_CATCHUP_DEFAULT)
    } else {
        None
    }
}

/// PURE: the catch-up cap one `Period` node runs under this step.
///
/// The precedence is the whole of the clamp's behavioural contract:
///
/// | declared | override | effective | why |
/// |---|---|---|---|
/// | `Some(n)` | anything | `n` | an EXPLICIT policy is never overridden |
/// | `None` | `Some(c)` | `c` | the clamp replaces the unbounded DEFAULT |
/// | `None` | `None` | `u32::MAX` | today's shipped default, byte-unchanged |
pub fn effective_max_catchup(declared: Option<u32>, override_cap: Option<u32>) -> u32 {
    match declared {
        Some(n) => n,
        None => override_cap.unwrap_or(u32::MAX),
    }
}

/// One step of a `Period` node's schedule: how many times it fires now, from
/// which instant, and where its deadline lands afterwards.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PeriodAdvance {
    /// How many times the node fires at this step (0 = it does not).
    pub fire_count: u32,
    /// The earliest un-fired interval — the first fire's gating instant.
    /// Meaningless when `fire_count == 0`.
    pub first_fire_ns: u64,
    /// The node's deadline AFTER this step.
    pub next_fire_ns: u64,
}

/// Advance one `Period` node's deadline to `now_ns` and report the burst — the
/// ONE implementation of that rule.
///
/// # Why it is here and not written four times
///
/// It was written twice when this was extracted: the live
/// `Scheduler::decide_node` `Period` arm advanced with a `while` LOOP, and
/// `cerulion_cli_engine::replay_rederive::period_step` re-derived the same
/// schedule in O(1) so replay could ask "would this burst have happened?". Two
/// spellings of one rule is the two-copies class, and those two DIVERGED at the
/// edges — which matters more than usual, because the whole job of the replay
/// copy is to answer identically to the live one.
///
/// There were two MORE: `Scheduler::reset_node` and the forward arm
/// of `Scheduler::restore_period_schedule` each carried their own advance, both
/// of them the `cap == 0` case ("carry the deadline strictly past `now`, mint no
/// fires"), and `reset_node`'s was still the `while` loop with both of its
/// non-terminating shapes intact. All four call this now. The `cap == 0` reading
/// is not a special case bolted on for them: `fire_count = min(uncapped, cap)`
/// is 0 at `cap == 0` while `next_fire_ns` still lands past `now`, which is the
/// same rule the loop's own skip-ahead arm implements once its budget is spent.
///
/// # The O(1) form is adopted, and it is not merely faster
///
/// With `uncapped = floor((now - next_fire) / interval) + 1` (the number of
/// whole intervals that carries the deadline strictly past `now`):
///
/// * the loop's `fire_count` is `min(uncapped, cap)`;
/// * the loop leaves `next_fire` at `next_fire + uncapped * interval` in EVERY
///   case — when `fire_count < cap` the loop itself got there, and when
///   `fire_count >= cap` (including `cap == 0`, where the `>=` is satisfied at
///   zero fires) the skip-ahead loop finishes the job.
///
/// So the two agree wherever the loop TERMINATES, and the O(1) form fixes the
/// two places it does not:
///
/// * `interval_ns == 0` made the loop spin forever (the deadline never moves
///   past `now`). Here it is 0 fires and an unmoved deadline — a `period_ms`
///   of 0 is refused upstream, so this is a defensive answer rather than a
///   behaviour change on any reachable path, but a diagnostic reading numbers
///   out of a bag must not hang.
/// * a WALL-FAITHFUL clock (~1.7e18 ns) against a 1 ms interval is ~1.7e12 loop
///   iterations, which is the hang `Scheduler::restore_period_schedule`'s own
///   doc records. A from-start free-run resim anchored on the first boundary is
///   exactly that shape.
///
/// SATURATING throughout, where the live arm used a plain `+=`: this now runs
/// over numbers read out of a file as well as over the scheduler's own, and a
/// clamp is the only answer that neither wraps nor panics.
#[must_use]
pub fn period_advance(next_fire_ns: u64, now_ns: u64, interval_ns: u64, cap: u32) -> PeriodAdvance {
    if interval_ns == 0 || next_fire_ns > now_ns {
        return PeriodAdvance {
            fire_count: 0,
            first_fire_ns: 0,
            next_fire_ns,
        };
    }
    let behind = now_ns - next_fire_ns; // `<=` above ⇒ never underflows.
    let uncapped = (behind / interval_ns).saturating_add(1);
    PeriodAdvance {
        fire_count: uncapped.min(u64::from(cap)) as u32,
        first_fire_ns: next_fire_ns,
        next_fire_ns: next_fire_ns.saturating_add(uncapped.saturating_mul(interval_ns)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The O(1) advance against a per-period LOOP, over a hand-chosen grid
    /// of shapes — the anti-drift oracle for the closed form.
    ///
    /// The loop below is a FROZEN TRANSLITERATION, and that is what it is FOR:
    /// no production loop exists to call. All four advance sites delegate to
    /// the closed form, so this copy is the reference behaviour — the evidence
    /// that the closed form answers what a per-period loop answers.
    /// It must never be "kept in sync" with anything: a change here
    /// would silently re-bless whatever the O(1) form does next. Only shapes
    /// where the loop TERMINATES are compared (`interval > 0`, and a bounded
    /// `behind`); the two non-terminating shapes the O(1) form fixes are
    /// asserted directly below.
    #[test]
    fn the_o1_advance_answers_exactly_what_the_loop_did() {
        fn loop_form(mut next_fire: u64, now: u64, interval: u64, cap: u32) -> PeriodAdvance {
            let cap = cap as usize;
            let mut fire_count: u32 = 0;
            let mut first_fire_ns: u64 = 0;
            if next_fire <= now {
                first_fire_ns = next_fire;
            }
            while next_fire <= now && (fire_count as usize) < cap {
                next_fire += interval;
                fire_count += 1;
            }
            if (fire_count as usize) >= cap {
                while next_fire <= now {
                    next_fire += interval;
                }
            }
            PeriodAdvance {
                fire_count,
                // The live arm leaves `first_fire_ns` at 0 when nothing fired
                // and the caller discards it; normalise so the compare is over
                // the fields that are read.
                first_fire_ns: if fire_count == 0 { 0 } else { first_fire_ns },
                next_fire_ns: next_fire,
            }
        }

        // (next_fire, now, interval, cap)
        let grid = [
            // Not yet due.
            (100u64, 99u64, 10u64, 5u32),
            // Exactly due — one fire.
            (100, 100, 10, 5),
            // One interval late.
            (100, 110, 10, 5),
            // A catch-up burst inside the cap.
            (100, 135, 10, 5),
            // A burst EXACTLY at the cap.
            (100, 140, 10, 5),
            // Over the cap: fires clamp, the deadline still skips ahead.
            (100, 1_000, 10, 5),
            // cap == 0: no fires, deadline still skips ahead. NOT a curiosity
            // — it is the reading `Scheduler::reset_node` and the forward arm of
            // `Scheduler::restore_period_schedule` both call this with, so this
            // row is a production shape.
            (100, 1_000, 10, 0),
            // cap == 1, far behind.
            (0, 10_000, 7, 1),
            // A wall-clock-looking anchor a few intervals in.
            (
                1_700_000_000_000_000_000,
                1_700_000_000_003_000_000,
                1_000_000,
                8,
            ),
        ];
        for (next_fire, now, interval, cap) in grid {
            let mut got = period_advance(next_fire, now, interval, cap);
            if got.fire_count == 0 {
                got.first_fire_ns = 0;
            }
            assert_eq!(
                got,
                loop_form(next_fire, now, interval, cap),
                "period_advance({next_fire}, {now}, {interval}, {cap})"
            );
        }
    }

    /// The two shapes a naive loop cannot answer at all.
    #[test]
    fn a_zero_interval_and_a_wall_honest_anchor_are_answered_rather_than_spun_on() {
        // A zero interval would spin a naive loop forever; here it is a no-op.
        assert_eq!(
            period_advance(100, 1_000, 0, 5),
            PeriodAdvance {
                fire_count: 0,
                first_fire_ns: 0,
                next_fire_ns: 100,
            }
        );
        // ~1.7e12 intervals behind — the from-start free-run resim shape. The
        // burst clamps to the cap and the deadline lands one interval past
        // `now`, computed without iterating.
        let got = period_advance(0, 1_700_000_000_000_000_000, 1_000_000, 4);
        assert_eq!(got.fire_count, 4);
        assert_eq!(got.first_fire_ns, 0);
        assert_eq!(got.next_fire_ns, 1_700_000_000_001_000_000);
        // And the saturating arm: an advance that would overflow clamps rather
        // than wrapping or panicking.
        let got = period_advance(0, u64::MAX - 1, 1, u32::MAX);
        assert_eq!(got.next_fire_ns, u64::MAX);
    }

    /// The derivation table, driven at every corner of both inputs against a
    /// HAND-written expectation — armed/unarmed x before/at/after the onset.
    ///
    /// The `at` row is the one a `>` variant of the boundary fails: it is the
    /// step that carries the first anchor, so a run that clamped only from
    /// `onset + 1` would smear precisely the anchor window the clamp exists for.
    #[test]
    fn the_override_engages_only_from_the_agreed_onset_of_an_armed_run() {
        let cases: &[(bool, u64, u64, Option<u32>)] = &[
            // (armed, first_anchor_step, step, expected override)
            (false, 0, 0, None),
            (false, 0, 9_999, None),
            (false, 100, 100, None),
            (true, 100, 99, None),
            (true, 100, 100, Some(ARMED_MAX_CATCHUP_DEFAULT)),
            (true, 100, 101, Some(ARMED_MAX_CATCHUP_DEFAULT)),
            (true, 0, 0, Some(ARMED_MAX_CATCHUP_DEFAULT)),
            (true, u64::MAX, u64::MAX - 1, None),
            (true, u64::MAX, u64::MAX, Some(ARMED_MAX_CATCHUP_DEFAULT)),
        ];
        for &(armed, first_anchor_step, step, expected) in cases {
            let onset = ArmOnset {
                armed,
                first_anchor_step,
            };
            assert_eq!(
                derive_catchup_cap_override(onset, step),
                expected,
                "armed={armed} first_anchor_step={first_anchor_step} step={step}"
            );
        }
    }

    /// A DISARM takes effect immediately — the onset gates only the ARMING
    /// direction. A recorder that detaches must hand the robot its declared
    /// behaviour back on the very next step, and `first_anchor_step` is stale
    /// the moment `armed` goes false.
    #[test]
    fn a_disarmed_word_clamps_nothing_however_far_past_its_old_onset() {
        let stale = ArmOnset {
            armed: false,
            first_anchor_step: 10,
        };
        for step in [0, 10, 11, 1_000_000] {
            assert_eq!(
                derive_catchup_cap_override(stale, step),
                None,
                "step={step}"
            );
        }
        assert_eq!(derive_catchup_cap_override(ArmOnset::DETACHED, 42), None);
    }

    /// The precedence table, hand-written: an explicit declaration outranks the
    /// clamp in BOTH directions (a value below the clamp AND one above it), and
    /// an unclamped run is byte-identical to the shipped `unwrap_or(u32::MAX)`.
    #[test]
    fn an_explicit_max_catchup_outranks_the_clamp_in_both_directions() {
        let clamp = Some(ARMED_MAX_CATCHUP_DEFAULT);
        // Declared BELOW the clamp — the node is stricter than the clamp.
        assert_eq!(effective_max_catchup(Some(1), clamp), 1);
        assert_eq!(effective_max_catchup(Some(1), None), 1);
        // Declared ABOVE the clamp — the node deliberately wants a long burst,
        // and an attached recorder does not get to shorten it.
        assert_eq!(effective_max_catchup(Some(64), clamp), 64);
        assert_eq!(effective_max_catchup(Some(64), None), 64);
        // Declared EQUAL to the clamp — no observable difference, stated so the
        // table has no gap.
        assert_eq!(
            effective_max_catchup(Some(ARMED_MAX_CATCHUP_DEFAULT), clamp),
            ARMED_MAX_CATCHUP_DEFAULT
        );
        // Undeclared: clamped while armed, unbounded otherwise.
        assert_eq!(
            effective_max_catchup(None, clamp),
            ARMED_MAX_CATCHUP_DEFAULT
        );
        assert_eq!(effective_max_catchup(None, None), u32::MAX);
        // The zero corner: a declared 0 is a real (degenerate) policy and must
        // NOT be read as "unset" and silently replaced by the clamp.
        assert_eq!(effective_max_catchup(Some(0), clamp), 0);
    }

    /// The clamp is a BOUND, not a floor: it can only ever REDUCE the number of
    /// catch-up fires an undeclared node performs, never raise it. Stated as a
    /// property over the whole table so a future default cannot invert it.
    #[test]
    fn the_clamp_never_raises_a_nodes_effective_cap() {
        for declared in [None, Some(0), Some(1), Some(4), Some(64), Some(u32::MAX)] {
            let unclamped = effective_max_catchup(declared, None);
            let clamped = effective_max_catchup(declared, Some(ARMED_MAX_CATCHUP_DEFAULT));
            assert!(
                clamped <= unclamped,
                "declared={declared:?}: clamped {clamped} > unclamped {unclamped}"
            );
        }
    }

    /// Drift guard on the shipped constant. `ARMED_MAX_CATCHUP_DEFAULT` is a
    /// BEHAVIOURAL constant of an armed robot, so a change to it is a change to
    /// what a recording run does and must be a deliberate edit here.
    #[test]
    fn the_armed_default_is_the_ruling_41_value() {
        assert_eq!(
            ARMED_MAX_CATCHUP_DEFAULT, 4,
            "the armed default is fixed at 4; changing it changes \
             what every armed robot's Period nodes do after a stall"
        );
        const _: () = assert!(
            ARMED_MAX_CATCHUP_DEFAULT >= 1,
            "a clamp of 0 would stop every undeclared Period node from firing at all"
        );
    }

    /// The REPLAY arm reports one fixed reading, and it feeds the same
    /// two pure functions the live arm does — so a replayed step lands on the
    /// same cap the recorded step ran under.
    ///
    /// Driven THROUGH `derive_catchup_cap_override` rather than by reading the
    /// field back: the property is that the clamp applies, not that a struct
    /// stores a number.
    #[test]
    fn a_static_replay_arm_clamps_from_its_recorded_onset() {
        let arm = StaticCatchupArm::armed_from(7);
        // Read TWICE — a replay reads it once per step, and an arm whose answer
        // moved between steps would make the fire set depend on the reading.
        assert_eq!(arm.onset(), CatchupArm::onset(&arm));

        // Below the onset: unclamped, exactly as the live run was.
        assert_eq!(
            derive_catchup_cap_override(CatchupArm::onset(&arm), 6),
            None
        );
        // AT the onset (the `>=` boundary) and above: clamped.
        assert_eq!(
            derive_catchup_cap_override(CatchupArm::onset(&arm), 7),
            Some(ARMED_MAX_CATCHUP_DEFAULT)
        );
        assert_eq!(
            derive_catchup_cap_override(CatchupArm::onset(&arm), 9_000),
            Some(ARMED_MAX_CATCHUP_DEFAULT)
        );
        // …and the node's OWN declaration still outranks it, which is the half a
        // replay must not break: a graph that declared `max_catchup` replays
        // under its declaration whether or not the recording was armed.
        assert_eq!(
            effective_max_catchup(Some(2), derive_catchup_cap_override(arm.onset(), 9_000)),
            2
        );
    }

    /// The ANTI-TAUTOLOGY half: an arm built from a DETACHED reading clamps
    /// nothing, so a recording that names no armed plane replays byte-identically
    /// to how it replayed before the clamp existed.
    #[test]
    fn a_detached_static_arm_leaves_every_step_unclamped() {
        let arm = StaticCatchupArm::new(ArmOnset::DETACHED);
        for step in [0u64, 1, 7, 9_000] {
            assert_eq!(derive_catchup_cap_override(arm.onset(), step), None);
            assert_eq!(
                effective_max_catchup(None, derive_catchup_cap_override(arm.onset(), step)),
                u32::MAX,
                "an unarmed recording keeps the unbounded default"
            );
        }
    }
}
