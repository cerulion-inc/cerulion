//! The per-topic plot-sample rate gate — how often a PLOT-classified
//! topic is allowed to contribute samples, keyed on the publisher's WIRE
//! timestamp.
//!
//! # The problem
//!
//! `Scalars` is deliberately not a COALESCING archetype
//! ([`crate::sink::coalesces`]): a plot is a stream of samples, not a whole-state
//! snapshot, so keeping only the newest frame of a poll tick would throw away
//! real data. The consequence is that a plot topic's render cost scales with the
//! PUBLISHER's rate multiplied by its series count, and nothing bounded the
//! product. A ~500 Hz telemetry firehose whose harvest is 349 series is ~175 000
//! `Scalars` logs per second into the viewer; the viewer was measured at 10 to 15 s of
//! lag at ~6 000 logs/s. The topic technically renders and is practically
//! unusable, which is worse than an explicit refusal.
//!
//! # The contract — and EXACTLY what it covers
//!
//! **A plot topic emits at most [`MAX_PLOT_SAMPLES_PER_SEC`] SCALAR SAMPLES per
//! second of WIRE time, plus ONE unmetered frame when the topic first renders.**
//! (That first frame is what teaches the gate the topic's width — see
//! [`PlotRateGate`] — so it is a one-off surcharge per topic, never a per-second
//! one.)
//!
//! **Scalar samples, and nothing else.** Three things a render arm can emit are
//! deliberately NOT metered, and each for its own reason:
//!
//! - **Geometry** — a pose, an attitude, a point, a cloud. Under rerun's latest-at
//!   semantics these are WHOLE STATE, so withholding one leaves the robot drawn at
//!   a stale position. A `Transform3DWithScalars` topic therefore keeps drawing its
//!   transform on every frame while its sibling telemetry thins.
//! - **Everything a non-plot archetype draws** — images, markers, video, TF.
//!
//! So "at most 2 000 logs/s per topic" is a claim about the SERIES half.
//!
//! **The `ScalarsWithText` structured dump is the one exemption that
//! was WITHDRAWN, because the class it protected changed under it.**
//! Logging the dump before the gate, on the grounds that metering it freezes the
//! document at the first admitted frame, which on a stopped publisher clock is
//! the whole run, is right for a class where a topic earns
//! `ScalarsWithText` only by declaring TEXT, i.e. a low-rate status message.
//! Classification also elects the same archetype for a topic whose CURATION withheld numeric
//! fields — the `/lowstate` firehose this module exists to bound — so the
//! exemption would have put a MEASURED 2 150-byte document × ~500 Hz ≈ 1 MB/s
//! into the viewer on precisely that topic. The dump now rides the SAME verdict
//! as the series ([`DumpRefreshGate`]), and the anti-freeze requirement — the text
//! must never freeze at frame 1 — is kept by a frame-count floor
//! ([`DUMP_REFRESH_FRAME_FLOOR`]) rather than by an exemption. A topic whose
//! series clear their budget every frame still dumps every frame.
//!
//! Every arm that emits curated series routes through ONE choke point
//! (`crate::sink::log_curated_series`), so the six render arms that log the same
//! harvest — `Scalars`/`ScalarsWithText`, the two spatial `WithScalars` pairs,
//! `Imu`, `Odometry`, `SportModeState` — share the one budget and a seventh
//! inherits it by construction. Before the choke point, the gate sat inside the
//! `Scalars` arm alone, so the composite shape it was built for — a wide bank
//! sitting beside a pose — was completely ungated.
//!
//! The gate spends the budget on whole frames: a topic
//! whose last rendered frame carried `S` series must advance its wire timestamp
//! by at least `S / MAX_PLOT_SAMPLES_PER_SEC` seconds before another frame is
//! plotted. So the cap is SELF-TUNING and needs no per-topic configuration —
//! a 6-series `/cmd_vel` is admitted up to ~333 Hz (i.e. never decimated in
//! practice), while a 240-series joint bank is admitted at ~8 Hz. Both land on
//! the same bounded budget.
//!
//! # Why WIRE time, never wall time
//!
//! Two independent reasons, both load-bearing:
//!
//! - **Determinism.** The admitted set must be a pure function of the frame
//!   stream, so a replay of the same frames renders the same plot. A wall-clock
//!   gate would admit a different subset on every run and on every machine — and
//!   per-tick coalescing, the other rate mechanism in this crate, is exactly that
//!   kind of wall-dependent decision (it depends on how the poll loop happened to
//!   batch). This gate is not.
//! - **It is the plot's own X axis.** [`crate::archetype::set_robot_time`] drives
//!   the rerun timeline from the wire stamp, so a frame whose stamp has not
//!   advanced plots AT THE SAME X as the frame before it. Decimating on stamp
//!   advancement therefore withholds only samples the plot could not have
//!   separated anyway.
//!
//! Wire stamps are a PER-PUBLISHER clock domain and are never compared across
//! producers: the gate compares one topic's stamps only against earlier stamps of
//! that same topic. A stamp that goes BACKWARDS is read as a publisher restart
//! (a new clock epoch), which re-anchors the gate and admits the frame rather
//! than stalling the topic until the old epoch's clock is overtaken.
//!
//! # What a refused frame COSTS — the full account
//!
//! A refused frame never pays the harvest walk or the logs: the choke point
//! consults the gate first. What it may still pay is the CLASSIFICATION, and that
//! depends on the topic's archetype, because
//! `crate::sink::infer_archetype_with_stability` marks every answer below the
//! element rung `ElementsUndecided` — those topics re-run the whole ladder on
//! every frame, and the ladder's plot rung is itself two full harvest walks.
//!
//! - A **remembered `Scalars` topic** (the target class, `/lowstate`
//!   included) is dropped on its HEADER, before the walk: `classify_and_route`
//!   consults the gate against `crate::sink::SinkState::plot_kind_hint`. Cost of a
//!   refused frame ≈ a 32-byte header parse. The FIRST frame of such a topic still
//!   pays a full classification — that is what records the hint.
//! - A **`ScalarsWithText` topic** is dropped on its header too, but
//!   only on a frame BOTH gates refuse: its dump rides [`DumpRefreshGate`], so a
//!   frame the floor elects to re-render must still be walked. Exempting this class
//!   from the drop entirely would pay the ladder on every frame; that
//!   matters once `/lowstate` joins it.
//! - A **spatial / `Imu` / `Odometry` / `SportModeState` topic** classifies
//!   `Stable`, so its archetype is memoized and per-frame classification is a map
//!   lookup; the gate saves the sibling harvest and the logs.
//!
//! # What it costs, stated plainly
//!
//! A transient between two admitted samples is not plotted. On a wide topic the
//! admitted rate is single-digit Hz, which is enough to WATCH a joint bank and
//! not enough to catch a 2 ms transient in it — the raw values stay on the wire
//! (`cerulion topic echo`) and a recording keeps every frame. That is the trade
//! this gate makes deliberately, against a pre-gate alternative that rendered
//! nothing usable at all.
//!
//! A publisher whose wire stamps never advance (a stopped or unset clock) is
//! admitted ONCE and then held. That is a real behaviour change, and it is the
//! right one: every one of those frames would plot at the identical X, so the
//! plot after the tenth is the plot after the first. It gets its OWN operator line
//! ([`RefusalCause::StalledClock`]) rather than the rate-limit one, because the
//! two conditions have different remedies.
//!
//! One further consequence of the header-only drop, stated because it is a real
//! semantic: a remembered `Scalars` topic whose SHAPE later changes (an element
//! array that was empty starts carrying elements, so the ladder would now answer
//! `Path3D`) re-classifies on its next ADMITTED frame rather than its next frame.
//!
//! On a healthy publisher that is one gate interval — single-digit milliseconds
//! for an ordinary topic, ~120 ms for a 240-series bank. **On a publisher whose
//! wire clock has STOPPED it is unbounded**, because there is no next admitted
//! frame at all: the topic was admitted once and is held forever, so its
//! classification is held with it. That is the same condition
//! [`RefusalCause::StalledClock`] warns about, and the correct reading of the warn
//! is "this topic's plot AND its layout are both frozen until the publisher's
//! clock moves". The stability memo is untouched and still evicts every frame;
//! this is the rate gate, not the memo.

/// The per-topic plot budget, in scalar samples per second of WIRE time.
///
/// The one number that answers "how much can a single plot topic's SERIES cost
/// the viewer". With [`crate::archetype::MAX_TOTAL_SERIES`] bounding a topic at
/// 256 series, this bounds the same topic at 2 000 scalar logs/s however fast its
/// publisher runs — a hard ceiling that holds without knowing the publisher's
/// rate.
///
/// It bounds the SERIES half only. Geometry and a `ScalarsWithText` dump are
/// deliberately outside it — see the module docs for why each one is.
///
/// 2 000 is chosen against the two measured points that bracket it: one measurement
/// recorded 10–15 s of viewer lag at ~6 000 logs/s (so the budget must sit well
/// under that), while a plot panel a human reads is ~1 000 px wide, so a
/// 240-series bank at the ~8 Hz this yields still fills a 2-minute window with
/// more points than the panel can draw. Ordinary control topics are unaffected:
/// a 6-series `/cmd_vel` would have to publish above ~333 Hz to be gated at all.
pub const MAX_PLOT_SAMPLES_PER_SEC: u64 = 2_000;

/// How many frames a `ScalarsWithText` topic may go without
/// re-rendering its field dump — the ANTI-FREEZE floor, and the only reason the
/// dump ever renders on a frame the series gate refused.
///
/// The dump's primary cadence is the SERIES gate's: it is a snapshot of the same
/// message the plot lines come from, so refreshing it when the plot refreshes
/// keeps one story on the screen and needs no second budget. That alone would
/// re-open the stalled-clock freeze, though: a publisher whose wire clock never advances is
/// admitted ONCE and refused forever, which would freeze its text at frame 1 for
/// the whole run, and the text is exactly what an operator reads when the plot
/// has stopped moving. This floor is the answer: after this many refused frames
/// the dump renders regardless.
///
/// A FRAME COUNT, not a duration, because the only clock this crate may key
/// decimation on is the publisher's wire stamp (determinism, Principle #7 — see
/// "Why WIRE time, never wall time" above), and in the case this floor exists for
/// that clock is precisely what has stopped. A frame count is still a pure
/// function of the frame stream, so a replay renders the same dumps.
///
/// **64 is calibrated to be INERT on a healthy topic.** The measured live shape —
/// a ~500 Hz bank curated to 60 series — earns a 30 ms gate interval, i.e. ~15
/// frames between admissions at 500 Hz, well inside the floor, so on that topic
/// the floor never fires and the dump tracks the plot exactly. It bites only when
/// the series gate has gone quiet for a long run of frames, which is either a
/// stalled clock or a topic far wider/faster than the measured one — and there
/// its cost is bounded by the publisher's rate (at 500 Hz, ~8 dumps/s ≈ 17 KB/s
/// against a MEASURED 2 150-byte dump for that bank).
///
/// The staleness it admits is stated plainly: on a stalled-clock publisher the
/// text pane can be up to this many frames old, which is ~0.13 s at 500 Hz and
/// ~6 s at 10 Hz. Such a topic also gets its own [`RefusalCause::StalledClock`]
/// operator line, so the condition is never silent.
pub const DUMP_REFRESH_FRAME_FLOOR: u64 = 64;

/// One `ScalarsWithText` topic's field-dump refresh gate — a pure state
/// machine over the SERIES gate's verdicts, with no clock of its own.
///
/// # Why the dump needs a gate at all
///
/// Originally the class was low-rate by construction: a topic earned
/// `ScalarsWithText` only by declaring TEXT, which in practice is a
/// vendor status / response / diagnostic message. The class is now also elected on a
/// second condition — the curation WITHHELD numeric fields — which is exactly the
/// `/lowstate`-shaped firehose this module was written to bound. Logging its dump on
/// every frame, as the earlier unmetered arm did, is a MEASURED 2 150 bytes ×
/// ~500 Hz ≈ 1 MB/s of markdown per topic, into the viewer, on the one topic the
/// curation exists to make usable. So the dump is metered — and the exact
/// version of the module's "the budget bounds its plot lines, not its document
/// count" is now: the document count rides the series cadence, with a floor.
///
/// # What it does NOT change
///
/// A topic whose series are admitted every frame — every text-declaring topic
/// small enough or slow enough to clear its own budget, which is any topic under
/// [`MAX_PLOT_SAMPLES_PER_SEC`] samples per wire second — dumps on every frame,
/// exactly as before.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DumpRefreshGate {
    /// Frames refused since the last RENDERED dump — the floor's counter.
    since_render: u64,
    /// How many dump renders this gate has withheld (Principle #3: observable
    /// without reading a log line).
    suppressed: u64,
}

impl DumpRefreshGate {
    /// Decide whether this frame renders its field dump, and record the decision.
    ///
    /// `series_admitted` is the SERIES gate's verdict for the SAME frame — the
    /// caller must consult that gate exactly once and hand the answer here, never
    /// re-consult it (it mutates).
    pub fn admit(&mut self, series_admitted: bool) -> bool {
        if series_admitted || self.since_render >= DUMP_REFRESH_FRAME_FLOOR {
            self.since_render = 0;
            return true;
        }
        self.since_render += 1;
        self.suppressed += 1;
        false
    }

    /// How many dump renders this gate has withheld so far.
    pub const fn suppressed(&self) -> u64 {
        self.suppressed
    }

    /// Frames refused since the last rendered dump (test / observability seam).
    pub const fn since_render(&self) -> u64 {
        self.since_render
    }
}

/// WHY a frame was refused — the two conditions have DIFFERENT remedies, so the
/// operator report must not collapse them.
///
/// "Over budget" is the gate working as designed: the publisher is faster than
/// its series count can afford, the plot is thinned, nothing is wrong. "Stalled
/// clock" is a producer whose wire stamps never advance — the plot holds ONE
/// sample for the run, and no amount of waiting or tuning changes that, because
/// the timeline IS the wire stamp. Told the first story, an operator tunes a rate
/// that is not the problem.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RefusalCause {
    /// The stamp advanced, but by less than this topic's series count earns.
    OverBudget,
    /// The stamp did not advance AT ALL since the last admitted frame.
    StalledClock,
}

/// One PLOT topic's rate gate — a pure state machine over that topic's wire
/// timestamps.
///
/// The series count is LEARNED from the last rendered frame rather than measured
/// up front, because the count is only known after the harvest and harvesting a
/// frame the gate is about to refuse is the cost this exists to avoid. The first
/// frame of a topic is therefore always admitted (there is nothing to learn from
/// yet) and sets the budget for the ones after it. That keeps the gate a pure
/// function of the frame stream — the count comes from an earlier frame of the
/// same stream, never from a clock or a measurement.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PlotRateGate {
    /// The wire timestamp of the last ADMITTED frame; `None` before the first.
    last_admitted_ns: Option<u64>,
    /// How many series the last admitted frame's harvest yielded.
    series: usize,
    /// How many frames this gate has refused (Principle #3: the decimation is
    /// observable independently of whether anyone read the log line).
    suppressed: u64,
    /// Whether the once-per-topic operator report has already been emitted.
    reported: bool,
    /// Why the FIRST refusal happened — what [`PlotRateGate::claim_report`] hands
    /// the caller so the two conditions get different operator lines.
    first_cause: Option<RefusalCause>,
}

impl PlotRateGate {
    /// The minimum wire-time gap between two admitted frames for a topic whose
    /// frames carry `series` plot series.
    ///
    /// Zero for a topic that has plotted nothing yet (or plots nothing at all) —
    /// a frame costing no samples cannot exceed a sample budget, so it is never
    /// refused.
    pub const fn min_interval_ns(series: usize) -> u64 {
        if series == 0 {
            return 0;
        }
        (series as u64) * 1_000_000_000 / MAX_PLOT_SAMPLES_PER_SEC
    }

    /// Decide whether the frame stamped `timestamp_ns` may be plotted, and record
    /// the decision. Returns `true` when the frame is admitted.
    ///
    /// Three admitting cases, in the order they are checked:
    ///
    /// 1. **First frame** — nothing to compare against, and the frame is what
    ///    teaches the gate the topic's width.
    /// 2. **Epoch reset** — the stamp is BELOW the last admitted one. Within one
    ///    publisher run stamps are non-decreasing (one writer, one clock), so a
    ///    regression is structural evidence of a restarted publisher on a fresh
    ///    clock. Admitting and re-anchoring recovers immediately; treating it as
    ///    "not advanced enough" would stall the topic for as long as the previous
    ///    run lasted.
    /// 3. **Budget met** — the stamp advanced by at least the interval the last
    ///    admitted frame's width earns.
    pub fn admit(&mut self, timestamp_ns: u64) -> bool {
        let Some(last) = self.last_admitted_ns else {
            self.last_admitted_ns = Some(timestamp_ns);
            return true;
        };
        let advanced_enough = timestamp_ns
            .checked_sub(last)
            .is_some_and(|delta| delta >= Self::min_interval_ns(self.series));
        // `checked_sub` returning None IS the epoch reset: the stamp is below the
        // anchor, so the publisher's clock restarted.
        if advanced_enough || timestamp_ns < last {
            self.last_admitted_ns = Some(timestamp_ns);
            return true;
        }
        self.suppressed += 1;
        if self.first_cause.is_none() {
            self.first_cause = Some(if timestamp_ns == last {
                RefusalCause::StalledClock
            } else {
                RefusalCause::OverBudget
            });
        }
        false
    }

    /// Record how many series the frame just rendered actually yielded — the
    /// budget the NEXT [`admit`](Self::admit) call spends.
    pub fn note_series(&mut self, series: usize) {
        self.series = series;
    }

    /// How many frames this gate has refused so far.
    pub const fn suppressed(&self) -> u64 {
        self.suppressed
    }

    /// The series count learned from the last admitted frame.
    pub const fn series(&self) -> usize {
        self.series
    }

    /// The effective minimum wire-time gap this topic is currently held to.
    pub const fn current_interval_ns(&self) -> u64 {
        Self::min_interval_ns(self.series)
    }

    /// Claim the once-per-topic operator report, returning the refusal CAUSE
    /// exactly once — on the first call after the gate has actually refused
    /// something. A gate that never refuses never reports, so a normal-rate topic
    /// stays silent.
    ///
    /// The cause is the FIRST refusal's, not the latest: the report is emitted at
    /// that moment and describes it. A topic whose clock later stalls after a
    /// genuine over-budget regime keeps the over-budget line — an accepted, stated
    /// limit of a once-per-topic report (the alternative is a second line per
    /// topic per cause, which re-opens the flood this report exists inside of).
    pub fn claim_report(&mut self) -> Option<RefusalCause> {
        if self.reported || self.suppressed == 0 {
            return None;
        }
        self.reported = true;
        self.first_cause
    }

    /// The cause of this gate's first refusal, if it has refused anything
    /// (Principle #3 — observable without reading a log line).
    pub const fn first_refusal_cause(&self) -> Option<RefusalCause> {
        self.first_cause
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The budget arithmetic, against hand-computed intervals — the numbers the
    /// module doc quotes, so the doc cannot rot silently.
    #[test]
    fn the_interval_is_the_series_count_divided_by_the_budget() {
        assert_eq!(PlotRateGate::min_interval_ns(0), 0, "nothing to plot");
        // 2 000 samples/s ⇒ one series may resample every 0.5 ms.
        assert_eq!(PlotRateGate::min_interval_ns(1), 500_000);
        // A 6-series /cmd_vel: 3 ms ⇒ ~333 Hz, above any real control topic.
        assert_eq!(PlotRateGate::min_interval_ns(6), 3_000_000);
        // A 240-series joint bank: 120 ms ⇒ ~8.3 Hz.
        assert_eq!(PlotRateGate::min_interval_ns(240), 120_000_000);
        // At the per-topic series cap the budget is exactly met: 256 × (1/2000) s.
        assert_eq!(PlotRateGate::min_interval_ns(256), 128_000_000);
    }

    /// The CEILING claim, derived rather than asserted: admitted frames × their
    /// series count never exceeds the budget per second of wire time, whatever
    /// the publisher's rate. Driven with a 500 Hz stream of 240-series frames
    /// (the measured live shape) over ten wire seconds.
    #[test]
    fn a_firehose_is_held_to_the_budget_per_wire_second() {
        let mut gate = PlotRateGate::default();
        const SERIES: usize = 240;
        const PERIOD_NS: u64 = 2_000_000; // 500 Hz
        const FRAMES: u64 = 5_000; // 10 wire seconds
        let mut admitted = 0u64;
        for i in 0..FRAMES {
            if gate.admit(i * PERIOD_NS) {
                admitted += 1;
                gate.note_series(SERIES);
            }
        }
        let wire_secs = (FRAMES * PERIOD_NS) as f64 / 1e9;
        let samples = (admitted * SERIES as u64) as f64;
        // The exact ceiling, INCLUDING the documented one-off: the first frame of
        // a topic is admitted unmetered (it is the frame that teaches the gate the
        // topic's width), so the bound is the budget over the window plus ONE
        // frame's worth — never a second unmetered frame.
        let ceiling = MAX_PLOT_SAMPLES_PER_SEC as f64 * wire_secs + SERIES as f64;
        assert!(
            samples <= ceiling,
            "admitted {admitted} frames × {SERIES} series = {samples} samples over \
             {wire_secs}s, past the {ceiling}-sample ceiling"
        );
        // …and the STEADY state (everything after that first frame) is inside the
        // budget with nothing added, which is the claim the module doc makes.
        let steady = ((admitted - 1) * SERIES as u64) as f64 / wire_secs;
        assert!(
            steady <= MAX_PLOT_SAMPLES_PER_SEC as f64,
            "steady-state {steady}/s is over the {MAX_PLOT_SAMPLES_PER_SEC} budget"
        );
        // Anti-vacuity: the gate did not simply stop the topic. The first frame
        // is free (nothing learned yet), so the steady-state count is one per
        // 120 ms of the 10 s window.
        assert_eq!(
            admitted,
            1 + 10_000 / 120,
            "hand oracle: 1 + floor(10s/120ms)"
        );
        assert_eq!(gate.suppressed(), FRAMES - admitted);
        // A genuine rate overrun is reported AS one — the complement of the
        // stalled-clock arm below. Without this pin a reporter that hardcoded
        // either cause would pass (only one of the two was asserted).
        assert_eq!(gate.first_refusal_cause(), Some(RefusalCause::OverBudget));
        assert_eq!(gate.claim_report(), Some(RefusalCause::OverBudget));
    }

    /// An ordinary control topic is NOT decimated — the gate must be invisible
    /// where it buys nothing. A 6-series topic at 100 Hz admits every frame and
    /// never claims a report. Without this the budget could be tuned down to
    /// "decimate everything" and the ceiling test above would still pass.
    #[test]
    fn a_normal_rate_control_topic_is_never_decimated() {
        let mut gate = PlotRateGate::default();
        for i in 0..500u64 {
            assert!(gate.admit(i * 10_000_000), "100 Hz frame {i} must plot");
            gate.note_series(6);
        }
        assert_eq!(gate.suppressed(), 0);
        assert!(
            gate.claim_report().is_none(),
            "a gate that refuses nothing stays silent"
        );
    }

    /// The threshold, pinned on both sides at one nanosecond. A frame exactly AT
    /// the interval is admitted; one nanosecond short is refused. A `>` / `>=`
    /// slip fails one side, and both sides are needed: the ceiling test alone
    /// passes a gate that is one nanosecond too strict.
    #[test]
    fn the_interval_is_a_threshold_pinned_on_both_sides() {
        let interval = PlotRateGate::min_interval_ns(10);

        let mut short = PlotRateGate::default();
        assert!(short.admit(1_000));
        short.note_series(10);
        assert!(
            !short.admit(1_000 + interval - 1),
            "one nanosecond short is refused"
        );
        // The refusal did NOT move the anchor: the next frame is measured from
        // the last ADMITTED stamp, so a refused frame cannot ratchet the gate.
        assert!(short.admit(1_000 + interval));

        let mut exact = PlotRateGate::default();
        assert!(exact.admit(1_000));
        exact.note_series(10);
        assert!(
            exact.admit(1_000 + interval),
            "exactly at the interval plots"
        );
    }

    /// A stamp BELOW the anchor is a publisher restart, not a stalled clock: it
    /// is admitted, and the gate re-anchors on the NEW epoch so the restarted
    /// topic resumes at its own budget immediately. A gate that only ever moved
    /// its anchor forwards would refuse a restarted publisher for as long as the
    /// previous run lasted — the exact inversion this rule exists to prevent.
    #[test]
    fn a_stamp_regression_is_an_epoch_reset_not_a_stall() {
        let mut gate = PlotRateGate::default();
        // Two hours of uptime on the old epoch.
        let old = 7_200 * 1_000_000_000u64;
        assert!(gate.admit(old));
        gate.note_series(10);
        // The worker restarts; its clock begins again near zero.
        assert!(
            gate.admit(1_000),
            "a restarted publisher is admitted at once"
        );
        gate.note_series(10);
        // …and the budget now runs against the NEW epoch, not the old one.
        assert!(!gate.admit(1_001), "too soon on the new clock");
        assert!(gate.admit(1_000 + PlotRateGate::min_interval_ns(10)));
        assert_eq!(gate.suppressed(), 1);
    }

    /// Identical successive stamps never advance, so after the first they are
    /// refused — the documented consequence of gating on the plot's own X axis.
    /// The refusals are COUNTED and the report is claimable, so the topic is
    /// never silently held.
    #[test]
    fn a_publisher_whose_clock_never_advances_plots_once_and_says_so() {
        let mut gate = PlotRateGate::default();
        assert!(gate.admit(42), "the first frame always plots");
        gate.note_series(4);
        for _ in 0..100 {
            assert!(!gate.admit(42));
        }
        assert_eq!(gate.suppressed(), 100);
        // A STOPPED clock is reported as such, never as a rate overrun.
        assert_eq!(
            gate.claim_report(),
            Some(RefusalCause::StalledClock),
            "the operator is told, once, WHICH condition this is"
        );
        assert_eq!(gate.claim_report(), None, "…and only once");
    }

    /// A topic whose harvest yields NO series is never refused: a frame costing
    /// zero samples cannot exceed a sample budget. This is the arm that keeps a
    /// declared-but-momentarily-empty plot topic (the frame-dependent-layout shape)
    /// from being held back by a gate it does not load.
    #[test]
    fn a_zero_series_frame_is_never_refused() {
        let mut gate = PlotRateGate::default();
        for i in 0..10u64 {
            assert!(gate.admit(i));
            gate.note_series(0);
        }
        assert_eq!(gate.suppressed(), 0);
    }

    /// DETERMINISM: the admitted set is a pure function of the stamp stream, so
    /// two independent gates driven by the same stamps admit exactly the same
    /// frames. Compared against a hand-built expectation as well, so it is not a
    /// self-compare: at 240 series (120 ms) over 1 kHz stamps, frame 0 is free
    /// and every 120th frame after it is admitted.
    #[test]
    fn the_admitted_set_is_a_pure_function_of_the_stamp_stream() {
        let run = || {
            let mut gate = PlotRateGate::default();
            let mut admitted = Vec::new();
            for i in 0..600u64 {
                if gate.admit(i * 1_000_000) {
                    admitted.push(i);
                    gate.note_series(240);
                }
            }
            admitted
        };
        let first = run();
        assert_eq!(first, run(), "two runs admit the same frames");
        let want: Vec<u64> = std::iter::once(0).chain((120..600).step_by(120)).collect();
        assert_eq!(first, want, "hand oracle: frame 0, then every 120 ms");
    }

    /// The width is learned from the LAST ADMITTED frame, so a topic that
    /// genuinely narrows (a dynamic array that shrinks) relaxes its own gate
    /// instead of staying held at the old width forever.
    #[test]
    fn the_gate_follows_the_width_of_the_last_admitted_frame() {
        let mut gate = PlotRateGate::default();
        assert!(gate.admit(0));
        gate.note_series(200); // 100 ms
        assert_eq!(gate.current_interval_ns(), 100_000_000);
        assert!(!gate.admit(50_000_000));
        assert!(gate.admit(100_000_000));
        gate.note_series(2); // 1 ms
        assert_eq!(gate.current_interval_ns(), 1_000_000);
        assert!(gate.admit(101_000_000), "the narrowed topic plots sooner");
    }

    /// The dump gate follows the SERIES verdict exactly — no second
    /// cadence, no clock of its own — as long as the series keep being admitted.
    /// That is what keeps one story on the screen and leaves every topic small
    /// enough to clear its own budget rendering exactly as it did before.
    #[test]
    fn the_dump_rides_the_series_verdict_frame_for_frame() {
        let mut gate = DumpRefreshGate::default();
        // Hand oracle: an admitted series always brings its document.
        for _ in 0..DUMP_REFRESH_FRAME_FLOOR * 3 {
            assert!(gate.admit(true));
        }
        assert_eq!(gate.suppressed(), 0, "nothing was withheld");
        assert_eq!(gate.since_render(), 0);
    }

    /// The ANTI-FREEZE floor, pinned as a threshold on BOTH sides — the
    /// requirement the dump metering keeps while withdrawing the exemption.
    ///
    /// A stalled publisher clock refuses every series after the first, so without
    /// the floor the text would show frame 1 for the life of the run. Exactly
    /// `DUMP_REFRESH_FRAME_FLOOR` refusals must NOT re-render (the pane is not yet
    /// stale enough) and the next one must — a hand-written verdict vector, so an
    /// off-by-one in either direction fails.
    #[test]
    fn the_anti_freeze_floor_is_a_threshold_pinned_on_both_sides() {
        let mut gate = DumpRefreshGate::default();
        // Frame 1: the series were admitted, so the document renders.
        assert!(gate.admit(true));
        // The clock then stalls. The floor counts REFUSED frames since that
        // render: the first `FLOOR` of them are withheld …
        for i in 0..DUMP_REFRESH_FRAME_FLOOR {
            assert!(
                !gate.admit(false),
                "refusal {i} is inside the floor and must stay withheld"
            );
        }
        assert_eq!(gate.suppressed(), DUMP_REFRESH_FRAME_FLOOR);
        // … and the NEXT one re-renders, resetting the counter.
        assert!(gate.admit(false), "the floor re-renders a frozen document");
        assert_eq!(gate.since_render(), 0, "the floor re-anchors on a render");
        assert_eq!(
            gate.suppressed(),
            DUMP_REFRESH_FRAME_FLOOR,
            "a re-render is not a suppression"
        );
        // The cycle repeats: the floor is a standing backstop, not a one-shot.
        for _ in 0..DUMP_REFRESH_FRAME_FLOOR {
            assert!(!gate.admit(false));
        }
        assert!(gate.admit(false));
    }

    /// The floor is calibrated to be INERT on a healthy topic — the claim its
    /// constant's doc makes, asserted rather than asserted-in-prose.
    ///
    /// The measured live shape: a ~500 Hz bank curated to 60 series earns a 30 ms
    /// interval, i.e. ~15 frames between admissions. Driven through BOTH real
    /// gates together, the dump must render exactly as often as the series and the
    /// floor must never fire — so on that topic the document count is the series
    /// frame count, not a second stream.
    #[test]
    fn on_the_measured_live_shape_the_floor_never_fires() {
        let mut series = PlotRateGate::default();
        let mut dump = DumpRefreshGate::default();
        const SERIES: usize = 60;
        const PERIOD_NS: u64 = 2_000_000; // 500 Hz
        let mut rendered_series = 0u64;
        let mut rendered_dumps = 0u64;
        for i in 0..2_500u64 {
            // 5 wire seconds
            let admitted = series.admit(i * PERIOD_NS);
            if admitted {
                rendered_series += 1;
                series.note_series(SERIES);
            }
            if dump.admit(admitted) {
                rendered_dumps += 1;
            }
        }
        assert_eq!(
            rendered_dumps, rendered_series,
            "the dump tracks the plot exactly — the floor contributed nothing"
        );
        // The gate interval is 30 ms, so 5 wire seconds admit 1 + 5000/30 frames.
        assert_eq!(rendered_series, 1 + 166);
        // And the measured cost claim: ~33 documents per wire second, against the
        // ~500 the earlier unmetered arm would have logged.
        assert!(
            (30..40).contains(&(rendered_dumps / 5)),
            "~33 documents per wire second, got {}",
            rendered_dumps / 5
        );
    }
}
