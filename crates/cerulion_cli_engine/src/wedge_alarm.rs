// SPDX-License-Identifier: AGPL-3.0-only
//! Flow mode: the ZERO-CONFIG wedge alarm's two PURE halves — the per-node
//! threshold LADDER and the dwell RULE the supervisor decides against.
//!
//! # What is being watched, and why nothing else watches it
//!
//! `tick_within_ms` times only ticks that RETURN — the scheduler reads
//! `elapsed()` after the callback, so a tick that never comes back increments no
//! counter and emits no line. The one thing in the system that noticed a
//! never-returning tick was the level barrier's boundary timeout, and the flow-mode
//! arc deletes it. This module is the replacement's half that can
//! be driven by an oracle vector: a THRESHOLD resolved once at plan time, and a
//! rule that folds one observation into a verdict.
//!
//! # Clock-free by construction
//!
//! [`WedgeWatch::observe`] takes the elapsed time since the caller's PREVIOUS
//! observation rather than reading a clock, exactly as
//! `cerulion_core::state_carrier::StallWatch` does and for the same two reasons
//! (prose, NOT an intra-doc link: this module is portable and that one is
//! `#[cfg(unix)]`, so a link breaks the non-unix docs gate — the same
//! rule `Scheduler::catchup_arm` states for its own field).
//! **The reaper thread owns the clock; the rule owns the decision** — which here
//! is not only tidiness but correctness, because the observer is a DIFFERENT
//! PROCESS from the one being observed: under the multi-process default each
//! worker runs on its own gating clock starting near zero while the supervisor is
//! on wall time, so a stamp written by a worker and compared against the
//! supervisor's clock compares two unrelated number lines (the liveness
//! clock-domain lesson, which cost that feature an inert gate on the shipping
//! deployment before anybody noticed). The evidence that crosses the process
//! boundary is therefore a SEQ PAIR carrying no time at all
//! (`cerulion_core::wedge_page`); all the time arithmetic happens here, on the
//! observer's own clock, against its own previous observation.
//!
//! It also keeps the whole rule testable at exact boundaries instead of by
//! asserting a wall, which on this repo's macOS CI is the class (a nominal
//! 150 ms charged as 1100–1696 ms under background QoS).

/// Multiplier applied to a DECLARED budget (`tick_within_ms`, `period_ms`).
///
/// A declared budget is a statement about what the node's tick is SUPPOSED to
/// cost, so overshooting it by 4× is not slowness, it is a node that is not coming
/// back. The number is deliberately small: the declaration is the strongest
/// evidence available about this node, and a large multiplier would throw most of
/// it away.
pub const WEDGE_DECLARED_BUDGET_MULTIPLIER: u64 = 4;

/// Multiplier applied to a PROFILED p50.
///
/// Five times looser than the declared multiplier because a p50 is a MEDIAN, not
/// a bound: half of that node's real ticks are slower than it by construction, and
/// tail ticks (a page fault, a first-touch allocation, a DVFS wake) are legitimate
/// rather than evidence of a wedge. A tight multiplier over a median is how a
/// zero-config alarm becomes an alarm operators mute.
pub const WEDGE_PROFILED_P50_MULTIPLIER: u64 = 20;

/// The floor EVERY derived threshold is raised to.
///
/// Five seconds is the number this codebase already uses when it decides a peer is
/// not coming back: `cerulion_core::barrier::BARRIER_BOUNDARY_TIMEOUT` (the exact
/// timeout this module replaces) and
/// `cerulion_core::state_carrier::STATE_STALL_TIMEOUT_NS` (the capture child's
/// watchdog, which argues the constant must be FIXED rather than derived from
/// anything the watched work does — every adaptive formulation re-creates a
/// cost-derived refusal).
///
/// It is a FLOOR rather than a default, which matters for the rungs above: a
/// `period_ms = 1` node derives 4 ms, and a 4 ms alarm on a real robot fires on
/// every scheduler preemption. The floor is what makes the ladder safe to apply
/// to a node whose declaration is tight.
pub const WEDGE_THRESHOLD_FLOOR_NS: u64 = 5_000_000_000;

/// The floor really IS the same 5 s the capture child's watchdog uses, asserted at
/// COMPILE TIME rather than in a test body.
///
/// A runtime `assert_eq!` in a `#[cfg(test)]` module states the relationship only
/// where tests run; a `const _` fails the BUILD, which is what "two 5 s numbers
/// cannot drift apart" has to mean if it is to be true of a release binary.
/// `state_carrier` is `#[cfg(unix)]`, so the assertion is too — on a non-Unix build
/// there is no second constant to disagree with.
///
/// A profiled median genuinely needs more headroom than a declared bound, so the
/// two multipliers' ordering is fixed here as well.
#[cfg(unix)]
const _: () = assert!(
    WEDGE_THRESHOLD_FLOOR_NS == cerulion_core::state_carrier::STATE_STALL_TIMEOUT_NS,
    "the wedge floor and the state-carrier stall timeout are the SAME \
     5 s decision about a peer that is not coming back — moving one must move the \
     other, not leave two numbers silently disagreeing"
);
const _: () = assert!(WEDGE_PROFILED_P50_MULTIPLIER > WEDGE_DECLARED_BUDGET_MULTIPLIER);

/// Which rung of the ladder produced a node's threshold.
///
/// Reported in the alarm so an operator can tell "your node blew the budget you
/// declared" from "your node blew a multiple of a number a profiler measured
/// once" — two findings with different remedies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThresholdSource {
    /// The node declared `#[cerulion_node(tick_within_ms = N)]`.
    DeclaredTickWithin,
    /// The node declared `#[cerulion_node(period_ms = N)]`.
    DeclaredPeriod,
    /// A `graphs/<name>.costs.yaml` snapshot carried a p50 for this node.
    ProfiledP50,
    /// The node declared nothing and no profile covered it — the bare floor.
    ///
    /// This arm is the ZERO-CONFIG property: a node with no rung is still
    /// watched, at [`WEDGE_THRESHOLD_FLOOR_NS`]. It is never silent for lack of
    /// declarations.
    Floor,
}

impl ThresholdSource {
    /// The operator-facing noun, logged as `threshold_source=`.
    pub fn label(self) -> &'static str {
        match self {
            Self::DeclaredTickWithin => "declared tick_within_ms",
            Self::DeclaredPeriod => "declared period_ms",
            Self::ProfiledP50 => "profiled p50",
            Self::Floor => "floor (nothing declared, nothing profiled)",
        }
    }
}

/// One node's resolved wedge threshold.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WedgeThreshold {
    /// The dwell a node must exceed, in nanoseconds. Never below
    /// [`WEDGE_THRESHOLD_FLOOR_NS`].
    pub ns: u64,
    /// Which rung produced it.
    pub source: ThresholdSource,
    /// Whether the FLOOR raised it above what the rung derived.
    ///
    /// Reported separately from [`source`](Self::source) rather than collapsed
    /// into a `Floor` verdict, because the two facts answer different questions: a
    /// `period_ms = 1` node really did derive its threshold from its declaration
    /// (so "make the declaration accurate" is not the remedy), AND the number in
    /// force is the floor's. Collapsing them would tell an operator their
    /// declaration was ignored, which is the misleading-surface class this repo
    /// rejects. Always `false` for [`ThresholdSource::Floor`], where there is no
    /// derived value for the floor to raise.
    pub floored: bool,
}

/// Resolve one node's wedge threshold from the ladder.
///
/// `tick_within_ms` → `period_ms` → profiled p50 → the bare floor, and whatever it
/// produces is raised to [`WEDGE_THRESHOLD_FLOOR_NS`].
///
/// # Every input is `Option`, and a ZERO is treated as absent
///
/// A declared `0` is not a legal budget the macros can emit, and a profiled p50 of
/// 0 means the profiler measured nothing rather than measuring instantaneous work
/// — so both would multiply to 0 and then be raised to the floor anyway. Falling
/// THROUGH instead means the node's threshold names the rung that actually
/// described it, which is what the report is for.
///
/// # Saturating, deliberately
///
/// A hand-edited `tick_within_ms` of `u64::MAX / 2` must not wrap into a
/// microsecond threshold that alarms on every tick. Saturation gives an
/// effectively-never threshold instead, which is the safe direction: the operator
/// asked for something absurd and gets no alarm rather than a false one.
pub fn resolve_wedge_threshold(
    tick_within_ms: Option<u64>,
    period_ms: Option<u64>,
    profiled_p50_ns: Option<u64>,
) -> WedgeThreshold {
    let nonzero = |v: Option<u64>| v.filter(|n| *n > 0);
    let (derived, source) = if let Some(ms) = nonzero(tick_within_ms) {
        (
            ms.saturating_mul(1_000_000)
                .saturating_mul(WEDGE_DECLARED_BUDGET_MULTIPLIER),
            ThresholdSource::DeclaredTickWithin,
        )
    } else if let Some(ms) = nonzero(period_ms) {
        (
            ms.saturating_mul(1_000_000)
                .saturating_mul(WEDGE_DECLARED_BUDGET_MULTIPLIER),
            ThresholdSource::DeclaredPeriod,
        )
    } else if let Some(p50) = nonzero(profiled_p50_ns) {
        (
            p50.saturating_mul(WEDGE_PROFILED_P50_MULTIPLIER),
            ThresholdSource::ProfiledP50,
        )
    } else {
        // No rung at all: the floor IS the threshold, and there is nothing for it
        // to have raised.
        return WedgeThreshold {
            ns: WEDGE_THRESHOLD_FLOOR_NS,
            source: ThresholdSource::Floor,
            floored: false,
        };
    };
    WedgeThreshold {
        ns: derived.max(WEDGE_THRESHOLD_FLOOR_NS),
        source,
        floored: derived < WEDGE_THRESHOLD_FLOOR_NS,
    }
}

/// What one observation of a node's seq pair says.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WedgeVerdict {
    /// The node is between ticks. Nothing to do; the dwell accumulator is reset.
    NotInTick,
    /// The node is inside a tick, but a DIFFERENT one from the last observation —
    /// it entered and left at least once in between, so it is executing normally.
    /// The dwell accumulator is reset.
    Progressing,
    /// The node has been inside the SAME tick since the last observation, but not
    /// yet for its threshold.
    Dwelling {
        /// Nanoseconds accumulated inside this one tick.
        dwell_ns: u64,
    },
    /// The node has been inside ONE tick past its threshold. The tick is not
    /// coming back.
    Wedged {
        /// Nanoseconds accumulated inside this one tick.
        dwell_ns: u64,
    },
}

impl WedgeVerdict {
    /// Whether this verdict should open (or keep open) the node's alarm regime.
    pub fn is_wedged(self) -> bool {
        matches!(self, Self::Wedged { .. })
    }
}

/// The observer-side dwell accumulator: one per watched node.
///
/// Constructed when the supervisor opens a rank's page, driven by the JOIN loop on
/// its existing 20 ms pass, dropped when the rank is reaped.
#[derive(Debug, Clone, Copy, Default)]
pub struct WedgeWatch {
    /// The pair observed last pass, or `None` before the first observation.
    last: Option<(u64, u64)>,
    /// Nanoseconds accumulated inside the pair currently held in `last`.
    dwell_ns: u64,
}

impl WedgeWatch {
    /// A watch for a node nobody has observed yet.
    pub fn new() -> Self {
        Self {
            last: None,
            dwell_ns: 0,
        }
    }

    /// Fold one observation in and decide.
    ///
    /// `elapsed_ns` is the time since the caller's PREVIOUS observation; the
    /// caller owns the clock (see the module docs). `threshold_ns` is the node's
    /// resolved [`WedgeThreshold::ns`].
    ///
    /// # A CHANGED pair is progress, whichever way it changed
    ///
    /// The two counters are monotone `fetch_add`s on one thread, so any change
    /// between two observations proves a fire boundary was crossed — the node
    /// entered a tick, left one, or both. Requiring the pair to be UNCHANGED is
    /// therefore the whole discrimination between a wedged node and a busy one, and
    /// it needs no notion of how fast "busy" is.
    ///
    /// # The FIRST observation of a pair never wedges
    ///
    /// A fresh watch banks its reading and returns without accumulating, because
    /// the elapsed time it was handed covers a period in which this pair may not
    /// have been the one held. Dwell is only ever accumulated across two
    /// observations that saw the SAME pair — which also means a supervisor that
    /// starts watching a node already deep inside a hung tick reports it one
    /// threshold later rather than immediately. That is the correct bound: the
    /// observer genuinely does not know how long a tick it has never seen begin
    /// has been running.
    ///
    /// # A pair the writer cannot produce is NOT a tick
    ///
    /// `exited > entered` is unreachable for a single writer that bumps entry
    /// first, so it can only be a torn read across a fire boundary or a page that
    /// is not the one this node writes. Both are answered "not in a tick", which
    /// costs at most one observation of delay and can never fabricate a wedge —
    /// the same conservative direction
    /// `cerulion_core::state_carrier::StallWatch` takes for a regressing progress
    /// counter.
    pub fn observe(
        &mut self,
        entered: u64,
        exited: u64,
        elapsed_ns: u64,
        threshold_ns: u64,
    ) -> WedgeVerdict {
        let pair = (entered, exited);
        let in_tick = entered > exited;
        if !in_tick {
            self.last = Some(pair);
            self.dwell_ns = 0;
            return WedgeVerdict::NotInTick;
        }
        if self.last != Some(pair) {
            self.last = Some(pair);
            self.dwell_ns = 0;
            return WedgeVerdict::Progressing;
        }
        self.dwell_ns = self.dwell_ns.saturating_add(elapsed_ns);
        if self.dwell_ns < threshold_ns {
            WedgeVerdict::Dwelling {
                dwell_ns: self.dwell_ns,
            }
        } else {
            WedgeVerdict::Wedged {
                dwell_ns: self.dwell_ns,
            }
        }
    }

    /// The dwell currently accumulated against the held pair.
    ///
    /// Reported alongside a terminal verdict so the alarm can say how long, and
    /// exposed (Principle #3) so the accumulator is observable rather than
    /// inferred from a verdict.
    pub fn dwell_ns(&self) -> u64 {
        self.dwell_ns
    }
}

/// The observer-side accumulator for a RANK's step-progress word — the
/// companion for a worker wedged OUTSIDE any tick.
///
/// Deliberately its own type rather than a `WedgeWatch` over `(progress, 0)`: the
/// per-node rule's whole content is "the pair did not change AND the node is
/// inside a tick", and a rank is never "inside" anything. Folding them would make
/// the composed type carry a meaningless second counter and a condition that is
/// always true.
#[derive(Debug, Clone, Copy, Default)]
pub struct StepProgressWatch {
    /// The highest step count observed. A WATERMARK, not the previous reading:
    /// the counter is monotone by construction, so a lower reading is a corrupt
    /// or wrong page, and treating "something changed" as liveness would let a
    /// scribbled page keep a wedged rank alive forever — the defeat-by-memory-
    /// ordering `StallWatch` documents.
    watermark: u64,
    /// Nanoseconds accumulated since the watermark last moved.
    stalled_for_ns: u64,
    /// Whether anything has been observed yet.
    seen: bool,
}

impl StepProgressWatch {
    /// A watch for a rank nobody has observed yet.
    pub fn new() -> Self {
        Self::default()
    }

    /// Fold one reading of the rank's step counter in and decide.
    ///
    /// `threshold_ns` is the rank's threshold, which the supervisor derives as the
    /// MAX over its nodes': a step legitimately contains its nodes' ticks, so a
    /// rank holding a node allowed to dwell `T` must itself be allowed to not
    /// advance for `T` — a tighter rank threshold would alarm on a node that is
    /// merely at its own limit, reporting a rank-level fault for a node-level
    /// condition the per-node half already covers.
    pub fn observe(&mut self, progress: u64, elapsed_ns: u64, threshold_ns: u64) -> WedgeVerdict {
        if !self.seen || progress > self.watermark {
            self.seen = true;
            self.watermark = progress;
            self.stalled_for_ns = 0;
            return WedgeVerdict::Progressing;
        }
        self.stalled_for_ns = self.stalled_for_ns.saturating_add(elapsed_ns);
        if self.stalled_for_ns < threshold_ns {
            WedgeVerdict::Dwelling {
                dwell_ns: self.stalled_for_ns,
            }
        } else {
            WedgeVerdict::Wedged {
                dwell_ns: self.stalled_for_ns,
            }
        }
    }

    /// The highest step count this rank has been observed to reach.
    pub fn watermark(&self) -> u64 {
        self.watermark
    }
}

#[cfg(test)]
mod threshold_tests {
    use super::*;

    const MS: u64 = 1_000_000;

    #[test]
    fn tick_within_ms_is_the_first_rung_and_beats_every_other_input() {
        // 2000 ms x 4 = 8 s, comfortably past the floor, so the value proves the
        // multiplier rather than the floor. Both lower rungs are supplied and must
        // be IGNORED.
        let t = resolve_wedge_threshold(Some(2_000), Some(50), Some(9_999_999));
        assert_eq!(t.ns, 8_000 * MS);
        assert_eq!(t.source, ThresholdSource::DeclaredTickWithin);
        assert!(!t.floored);
    }

    #[test]
    fn period_ms_is_the_second_rung_and_beats_a_profile() {
        let t = resolve_wedge_threshold(None, Some(2_000), Some(9_999_999));
        assert_eq!(t.ns, 8_000 * MS);
        assert_eq!(t.source, ThresholdSource::DeclaredPeriod);
        assert!(!t.floored);
    }

    #[test]
    fn a_profiled_p50_is_the_third_rung_at_the_looser_multiplier() {
        // 400 ms p50 x 20 = 8 s. The SAME 8 s the two rungs above reach from a
        // 2000 ms declaration, which is the point: a median needs 5x the headroom
        // a declared bound does.
        let t = resolve_wedge_threshold(None, None, Some(400 * MS));
        assert_eq!(t.ns, 8_000 * MS);
        assert_eq!(t.source, ThresholdSource::ProfiledP50);
        assert!(!t.floored);
    }

    #[test]
    fn a_node_with_no_rung_at_all_is_still_watched_at_the_bare_floor() {
        // THE zero-config property: nothing declared, nothing profiled, still
        // watched. A `None` here would be the feature silently opting a node out
        // for the crime of not being annotated.
        let t = resolve_wedge_threshold(None, None, None);
        assert_eq!(t.ns, WEDGE_THRESHOLD_FLOOR_NS);
        assert_eq!(t.source, ThresholdSource::Floor);
        assert!(
            !t.floored,
            "there is no derived value for the floor to have raised"
        );
    }

    #[test]
    fn a_tight_declaration_is_raised_to_the_floor_and_says_so_without_disowning_the_rung() {
        // The shape a 1 kHz control node produces: 1 ms x 4 = 4 ms, which as a
        // threshold would fire on every scheduler preemption.
        let t = resolve_wedge_threshold(None, Some(1), None);
        assert_eq!(t.ns, WEDGE_THRESHOLD_FLOOR_NS);
        assert!(t.floored, "the floor raised it — the operator must be told");
        assert_eq!(
            t.source,
            ThresholdSource::DeclaredPeriod,
            "the rung that DERIVED it is still period_ms; reporting `Floor` here \
             would tell the operator their declaration was ignored"
        );
    }

    #[test]
    fn the_floor_boundary_is_pinned_on_both_sides() {
        // Exactly AT the floor: derived == floor, so nothing was raised.
        let at = resolve_wedge_threshold(Some(WEDGE_THRESHOLD_FLOOR_NS / MS / 4), None, None);
        assert_eq!(at.ns, WEDGE_THRESHOLD_FLOOR_NS);
        assert!(!at.floored, "`floored` is `<`, not `<=`");
        // One millisecond of declaration above it.
        let above =
            resolve_wedge_threshold(Some(WEDGE_THRESHOLD_FLOOR_NS / MS / 4 + 1), None, None);
        assert_eq!(above.ns, WEDGE_THRESHOLD_FLOOR_NS + 4 * MS);
        assert!(!above.floored);
    }

    #[test]
    fn a_zero_declaration_falls_through_rather_than_multiplying_to_nothing() {
        // A zero would multiply to 0 and then be raised to the floor anyway, so
        // the OUTCOME is the same — what falling through buys is that the report
        // names the rung that actually described the node.
        let t = resolve_wedge_threshold(Some(0), Some(0), Some(400 * MS));
        assert_eq!(t.source, ThresholdSource::ProfiledP50);
        assert_eq!(t.ns, 8_000 * MS);
        let none = resolve_wedge_threshold(Some(0), Some(0), Some(0));
        assert_eq!(none.source, ThresholdSource::Floor);
    }

    #[test]
    fn an_absurd_declaration_saturates_to_never_rather_than_wrapping_to_always() {
        let t = resolve_wedge_threshold(Some(u64::MAX), None, None);
        assert_eq!(t.ns, u64::MAX, "saturating, not wrapping");
        assert!(
            t.ns > WEDGE_THRESHOLD_FLOOR_NS,
            "a wrap would land BELOW the floor and alarm on every tick"
        );
        let p = resolve_wedge_threshold(None, None, Some(u64::MAX));
        assert_eq!(p.ns, u64::MAX);
    }

    #[test]
    fn every_source_renders_a_distinct_operator_facing_noun() {
        let all = [
            ThresholdSource::DeclaredTickWithin,
            ThresholdSource::DeclaredPeriod,
            ThresholdSource::ProfiledP50,
            ThresholdSource::Floor,
        ];
        for (i, a) in all.iter().enumerate() {
            for b in &all[i + 1..] {
                assert_ne!(a.label(), b.label(), "{a:?} vs {b:?}");
            }
        }
    }

    #[test]
    fn the_constants_are_the_ones_the_design_names() {
        assert_eq!(WEDGE_DECLARED_BUDGET_MULTIPLIER, 4);
        assert_eq!(WEDGE_PROFILED_P50_MULTIPLIER, 20);
        assert_eq!(WEDGE_THRESHOLD_FLOOR_NS, 5_000_000_000);
        // The tie to `state_carrier::STATE_STALL_TIMEOUT_NS` and the multipliers'
        // ordering are `const _: () = assert!(..)` at module scope, NOT here: a
        // runtime assertion in a `#[cfg(test)]` body states the relationship only
        // where tests run, and the claim being made ("two 5 s numbers cannot drift
        // apart") has to hold of a release build. Restating it here would be a
        // second, weaker copy of a constraint the compiler already enforces.
    }
}

#[cfg(test)]
mod dwell_tests {
    use super::*;

    const S: u64 = 1_000_000_000;
    /// The supervisor's real JOIN-loop pass.
    const PASS: u64 = 20_000_000;

    #[test]
    fn a_node_between_ticks_never_dwells_however_long_it_sits_there() {
        let mut w = WedgeWatch::new();
        for _ in 0..1_000 {
            assert_eq!(w.observe(7, 7, PASS, 5 * S), WedgeVerdict::NotInTick);
        }
        assert_eq!(w.dwell_ns(), 0);
    }

    #[test]
    fn a_busy_node_resets_on_every_changed_pair() {
        let mut w = WedgeWatch::new();
        // Caught mid-tick every pass, but a DIFFERENT tick every pass — the shape
        // a node firing faster than the observer polls produces.
        for n in 1..=1_000u64 {
            let v = w.observe(n, n - 1, PASS, 5 * S);
            assert!(
                matches!(v, WedgeVerdict::Progressing),
                "pass {n} gave {v:?} — a changed pair proves a fire boundary was \
                 crossed, so it can never be a wedge"
            );
        }
        assert_eq!(w.dwell_ns(), 0);
    }

    #[test]
    fn one_unchanging_tick_dwells_then_wedges_exactly_at_the_threshold() {
        let mut w = WedgeWatch::new();
        // Observation 1 BANKS the pair and accumulates nothing.
        assert_eq!(w.observe(3, 2, PASS, 5 * S), WedgeVerdict::Progressing);
        assert_eq!(w.dwell_ns(), 0);
        // 249 further passes of 20 ms = 4.98 s: dwelling, not wedged.
        for _ in 0..249 {
            assert!(matches!(
                w.observe(3, 2, PASS, 5 * S),
                WedgeVerdict::Dwelling { .. }
            ));
        }
        assert_eq!(w.dwell_ns(), 4_980_000_000);
        // The 250th takes it to exactly 5.00 s — the boundary is `>=`.
        assert_eq!(
            w.observe(3, 2, PASS, 5 * S),
            WedgeVerdict::Wedged { dwell_ns: 5 * S }
        );
        // And it stays wedged while the pair stays put.
        assert!(w.observe(3, 2, PASS, 5 * S).is_wedged());
    }

    #[test]
    fn the_threshold_boundary_is_pinned_on_both_sides_in_one_body() {
        let mut below = WedgeWatch::new();
        below.observe(1, 0, 0, 5 * S);
        assert_eq!(
            below.observe(1, 0, 5 * S - 1, 5 * S),
            WedgeVerdict::Dwelling {
                dwell_ns: 5 * S - 1
            }
        );
        let mut at = WedgeWatch::new();
        at.observe(1, 0, 0, 5 * S);
        assert_eq!(
            at.observe(1, 0, 5 * S, 5 * S),
            WedgeVerdict::Wedged { dwell_ns: 5 * S }
        );
    }

    #[test]
    fn a_tick_that_finally_returns_clears_the_dwell_rather_than_carrying_it() {
        let mut w = WedgeWatch::new();
        w.observe(1, 0, 0, 5 * S);
        assert!(w.observe(1, 0, 10 * S, 5 * S).is_wedged());
        // It comes back.
        assert_eq!(w.observe(1, 1, PASS, 5 * S), WedgeVerdict::NotInTick);
        assert_eq!(w.dwell_ns(), 0, "the recovery must not carry stale dwell");
        // The NEXT tick starts from zero, so one slow-but-returning tick cannot
        // inherit the previous one's dwell and wedge immediately.
        w.observe(2, 1, PASS, 5 * S);
        assert!(matches!(
            w.observe(2, 1, PASS, 5 * S),
            WedgeVerdict::Dwelling { dwell_ns } if dwell_ns == PASS
        ));
    }

    #[test]
    fn the_first_observation_of_a_pair_banks_it_without_accumulating() {
        // The correct bound: an observer that starts watching a node already deep
        // inside a hung tick reports it one threshold later, because it cannot
        // know how long a tick it never saw begin has been running. A first
        // observation that ACCUMULATED would fabricate that knowledge from
        // whatever `elapsed_ns` the caller happened to pass.
        let mut w = WedgeWatch::new();
        assert_eq!(
            w.observe(1, 0, 3_600 * S, 5 * S),
            WedgeVerdict::Progressing,
            "an hour of elapsed time before the first sighting proves nothing \
             about this tick"
        );
        assert_eq!(w.dwell_ns(), 0);
    }

    #[test]
    fn a_pair_no_writer_can_produce_reads_as_not_in_a_tick() {
        let mut w = WedgeWatch::new();
        for _ in 0..1_000 {
            assert_eq!(w.observe(2, 9, PASS, 5 * S), WedgeVerdict::NotInTick);
        }
        assert_eq!(w.dwell_ns(), 0);
    }

    #[test]
    fn dwell_saturates_rather_than_wrapping_back_under_the_threshold() {
        let mut w = WedgeWatch::new();
        w.observe(1, 0, 0, u64::MAX);
        w.observe(1, 0, u64::MAX, u64::MAX);
        assert_eq!(w.dwell_ns(), u64::MAX);
        // A wrap here would drop the dwell to ~0 and un-wedge a wedged node.
        assert!(w.observe(1, 0, u64::MAX, u64::MAX).is_wedged());
    }

    #[test]
    fn a_ranks_step_word_stalls_then_wedges_and_a_single_advance_clears_it() {
        let mut w = StepProgressWatch::new();
        assert_eq!(w.observe(0, PASS, 5 * S), WedgeVerdict::Progressing);
        for _ in 0..249 {
            assert!(matches!(
                w.observe(0, PASS, 5 * S),
                WedgeVerdict::Dwelling { .. }
            ));
        }
        assert_eq!(
            w.observe(0, PASS, 5 * S),
            WedgeVerdict::Wedged { dwell_ns: 5 * S }
        );
        assert_eq!(w.observe(1, PASS, 5 * S), WedgeVerdict::Progressing);
        assert_eq!(w.watermark(), 1);
    }

    #[test]
    fn a_ranks_first_reading_is_progress_even_at_zero() {
        // A worker that has not opened its first step yet reads 0, and 0 is also
        // the watermark's initial value — so without the `seen` flag the rank
        // would start accumulating stall from its very first observation and
        // alarm one threshold into a perfectly healthy startup.
        let mut w = StepProgressWatch::new();
        assert_eq!(w.observe(0, 3_600 * S, 5 * S), WedgeVerdict::Progressing);
        assert_eq!(w.watermark(), 0);
    }

    #[test]
    fn a_regressing_step_word_is_not_treated_as_liveness() {
        let mut w = StepProgressWatch::new();
        w.observe(100, PASS, 5 * S);
        // A page that is not this rank's, or a corrupted one, flapping below the
        // watermark. Permissively reading "something changed, so it is alive"
        // would defeat the watchdog by a memory ordering rather than by a missing
        // rule.
        for _ in 0..250 {
            w.observe(5, PASS, 5 * S);
        }
        assert!(w.observe(5, PASS, 5 * S).is_wedged());
        assert_eq!(w.watermark(), 100, "the watermark never regresses");
    }
}
