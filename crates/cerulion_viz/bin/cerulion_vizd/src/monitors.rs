// SPDX-License-Identifier: AGPL-3.0-only
//! Monitors — the DAEMON-side plane around
//! [`cerulion_viz::monitor`]'s pure engine.
//!
//! The engine is a pure function of the samples it is handed:
//! no clock, no transport, no I/O, and — deliberately — no caller of its own. This
//! module is the state that makes it reachable, and nothing more. It owns exactly
//! four things the engine cannot:
//!
//! 1. **ONE monotonic origin.** The engine's every duration is a difference of two
//!    sample timestamps, so the origin is arbitrary as long as it is the SAME
//!    origin for that engine's whole life. Holding it here is what guarantees
//!    that: the sampler and the serving handler both date against
//!    [`MonitorPlane::now_ns`], so there is one number line and no way to grow a
//!    second one.
//! 2. **The bounded alert ring** ([`MONITOR_ALERT_RING`]). Retention policy, not
//!    engine policy: the engine MINTS transitions and forgets them.
//! 3. **The per-`(row, condition)` flood latches.** See
//!    `MonitorPlane::report_open_regimes` for what actually floods and why the
//!    latch is fed per SAMPLE rather than per transition.
//! 4. **The pass outcome** ([`SamplePassOutcome`]) the daemon folds into its
//!    counted observables.
//!
//! # Why the ring is bounded and the row table is not
//!
//! A row is STICKY and always readable (Principle #3) — that is the contract
//! `monitors` serves, and a row the daemon dropped would read as "never watched
//! it" rather than "watched it and it is fine". The rows are bounded instead by
//! their LIFECYCLE: a row is created by a sample and destroyed by
//! [`MonitorPlane::forget`], which the daemon calls where the observation it was
//! derived from ends (a `detach`). Alerts have no such lifecycle — a wedged robot
//! can mint them indefinitely — so they ride a ring and the agent dedupes on `seq`.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::time::Instant;

use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};
use cerulion_viz::monitor::{
    Alert, MonitorCondition, MonitorEngine, MonitorRow, MonitorSample, MONITOR_ALERT_RING,
    MONITOR_SAMPLE_INTERVAL_NS,
};

/// The identity of one DISCOVERY-plane row: `(robot, topic)`.
///
/// Robot FIRST, unlike the engine's own `RowKey` (which sorts by topic so its
/// output reads as a topic list). This set is only ever asked robot-scoped
/// questions — "which of go2's rows did this gather stop naming?" — and a
/// robot-major order makes that a contiguous range rather than a full scan.
///
/// A catalog row always names its robot, which is what makes a plain `String`
/// correct here where the engine needs an `Option`: an entry arrives inside a
/// [`CatalogReply`](cerulion_core::transport::cerulion_q::CatalogReply) that
/// self-attributes, so a discovery row can never be the `robot: None` LOCAL row.
type DiscoveryRowKey = (String, String);

/// A timestamp on this plane's ONE number line that has been LATCHED MONOTONE —
/// the only thing an engine WRITE accepts.
///
/// # Why this is a type rather than a `u64`
///
/// With exactly one writer (the drain loop's sampler), "the plane's
/// clock never goes backwards" is true by construction. The discovery plane adds a SECOND
/// writer on a CONTROLLER thread, and that breaks it: a `discover` handler
/// captures its instant, spends a netd round trip (150–600 ms, and unbounded on a
/// slow LAN) gathering, and only then reaches the engine — by which time the drain
/// loop has fed several passes at LATER instants. Two controller threads racing
/// two `discover` calls produce the same inversion without any gather at all.
///
/// A backwards timestamp is not a cosmetic problem, and it is not even
/// symmetrically conservative. [`ConditionTracker`] anchors
/// `qualifying_since_ns` at the FIRST qualifying sample of a run and confirms when
/// `now - anchor >= MONITOR_CONFIRM_MIN_SPAN_NS`. Anchor that run on a stale
/// instant and the span reads LONGER than the observation really was — so a
/// condition confirms INSIDE the window a lull-free restart is still
/// healing in, which is the one thing [`MONITOR_CONFIRM_MIN_SPAN_NS`] exists to
/// prevent. That is a false `stalled` on a topic that is fine.
///
/// So the write clock is latched, and the latch is not a convention anyone has to
/// remember: the field is private to this module, [`MonitorPlane::advance_to`] is
/// its only constructor, and the write entry points take this type. A daemon that
/// tried to write with [`MonitorPlane::now_ns`] — the READ clock, which must NOT
/// latch, or a serving handler would advance the writers' number line — does not
/// compile.
///
/// [`ConditionTracker`]: cerulion_viz::monitor::ConditionTracker
/// [`MONITOR_CONFIRM_MIN_SPAN_NS`]: cerulion_viz::monitor::MONITOR_CONFIRM_MIN_SPAN_NS
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct WriteStamp(u64);

impl WriteStamp {
    /// The latched nanoseconds, for building the samples of the pass it stamps.
    pub const fn as_ns(self) -> u64 {
        self.0
    }

    /// A stamp at a HAND-CHOSEN instant, for tests that drive the engine on a
    /// sample clock of their own.
    ///
    /// Test-only, and it does not weaken the guarantee: what [`WriteStamp`] buys is
    /// that PRODUCTION code cannot reach a write clock other than
    /// [`MonitorPlane::advance_to`], and a `#[cfg(test)]` constructor is not in
    /// production. The latch itself is driven through `advance_to` by
    /// `the_write_clock_never_runs_backwards_when_two_writers_race`.
    #[cfg(test)]
    pub(crate) const fn for_test(ns: u64) -> Self {
        Self(ns)
    }
}

/// What ONE sampling pass did — the deltas the daemon folds into its counted
/// observables, returned rather than written so the plane stays free of atomics
/// and the counter bump lands at the ONE site that knows the pass really ran.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SamplePassOutcome {
    /// Samples HANDED TO the engine on this pass — one per watched row.
    ///
    /// Deliberately counts samples the engine then DISCARDED (inside a row's
    /// settle window, or carrying no usable evidence), because the question this
    /// answers is "is the sampler running?", not "what did it learn?". The
    /// evidence count is [`MonitorRow::samples`], which is per row and counts only
    /// what was admitted. Conflating the two would leave the sampler's own
    /// liveness unobservable for the first [`MONITOR_SETTLE_NS`] of every row.
    ///
    /// [`MONITOR_SETTLE_NS`]: cerulion_viz::monitor::MONITOR_SETTLE_NS
    pub observed: u64,
    /// Conditions RAISED on this pass.
    pub raised: u64,
    /// Conditions CLEARED on this pass.
    pub cleared: u64,
    /// How many rows are reporting at least one withheld condition AS OF this
    /// pass — a GAUGE, not a running total.
    ///
    /// The distinction matters and is why this one field is not a delta: "12 rows
    /// are being judged on nothing" is a statement about NOW that an operator can
    /// act on, while a monotonic sum over passes would climb forever on a healthy
    /// desk with one permanently-ineligible `/tf` and mean nothing at all.
    pub rows_ineligible: u64,
    /// Rows RETIRED on this pass — always 0 on the attached plane, which has no
    /// retirement of its own (a tap's row dies with its tap).
    ///
    /// Counted because the retirement is otherwise observable only by ABSENCE, and
    /// "the row is gone" and "the row was never opened" render identically. An
    /// operator watching a churning LAN needs to be able to tell a plane that is
    /// keeping up from one that has quietly stopped feeding.
    pub retired: u64,
}

/// The daemon's monitor state: the engine, its clock origin, the alert ring, and
/// the flood latches.
///
/// Lives in the daemon's EXISTING state, deliberately. The monitors make
/// a testable cost claim that includes "no new lock, no new lock ordering", and a
/// plane reachable from both the poll thread and a control handler needs
/// synchronisation of some kind — so it borrows the lock the poll thread already
/// takes every pass rather than introducing a second one to get wrong.
#[derive(Debug)]
pub struct MonitorPlane {
    /// The ONE monotonic origin (see the module docs).
    origin: Instant,
    /// The highest stamp ever handed to a WRITER — see [`WriteStamp`] for why two
    /// writers on two threads make this necessary and what a regression costs.
    last_write_ns: u64,
    /// The newest gather GENERATION whose evidence was applied, or `None` before
    /// the first pass. See [`MonitorPlane::catalog_superseded`].
    ///
    /// Advanced only on an APPLIED pass: a throttled or superseded pass leaves no
    /// mark, so it can neither delay the next legitimate feed nor raise the bar
    /// against a gather whose answer was never used.
    last_catalog_gen: Option<u64>,
    /// When the CATALOG plane last fed, or `None` before its first pass.
    ///
    /// The attached sampler's cadence is owned by `poll_loop`, which holds a
    /// pass-local mark. The catalog plane has no owning thread — it rides whatever
    /// `discover` calls a controller makes — so its mark lives here, and the gate
    /// is enforced by [`Self::observe_catalog_pass`] itself rather than by a call
    /// site that could forget it.
    last_catalog_feed_ns: Option<u64>,
    engine: MonitorEngine,
    /// Rows this plane opened from a CATALOG, `(robot, topic)`.
    ///
    /// Tracked because catalog rows have no lifecycle of their own. An ATTACHED
    /// row is bounded by its tap (a `detach` forgets it); a catalog row is minted
    /// by whatever a robot happened to announce, so a long-lived desk watching a
    /// churning LAN would accumulate rows for topics — and robots — that no longer
    /// exist and report them UNKNOWN with a forever-growing age. This set is what
    /// lets a SETTLED gather retire them, and it holds ONLY discovery-owned keys
    /// so the retirement can never reach a row a tap owns.
    discovery_rows: BTreeSet<DiscoveryRowKey>,
    /// Recent transitions, oldest first, bounded by [`MONITOR_ALERT_RING`].
    alerts: VecDeque<Alert>,
    /// One latch per `(robot, topic, condition)` — the keying rule ("one
    /// per observing entity AND condition"), so a row's open `stalled` regime can
    /// never swallow the loud head of its own `rate_deviation`.
    ///
    /// Created LAZILY, only when a condition is actually raised, so a healthy desk
    /// carries none. Removed with the row by [`Self::forget`].
    latches: BTreeMap<(Option<String>, String, MonitorCondition), FailureRegimeLatch>,
    /// The baseline the RAISE was judged against, per `(robot, topic, condition)`.
    ///
    /// [`MonitorRow::baseline_mhz`] and the verdict's own basis are two
    /// different numbers: a `stalled` confirmed after a baseline wipe is judged on the
    /// row's LAST FROZEN baseline, which rides the ALERT while the row's field
    /// stays absent (it is the live RATE baseline, and that really is re-learning).
    /// [`Self::report_condition`] reads the row, so without this the loud line for
    /// exactly the class the monitors exist to page would carry no baseline at all —
    /// Principle #3 silence on the one number an operator judges the verdict from.
    ///
    /// Written from the RAISE, so it is the number the verdict rested on rather
    /// than whatever the row holds when a line is emitted; it must therefore serve
    /// the DECADE re-announcement too, not only the head, which is why it outlives
    /// the pass. Overwritten by the next raise of the same regime (never merged),
    /// REMOVED when a raise carries no baseline at all — an absent basis must not
    /// be reported as an earlier regime's number — and dropped with the row beside
    /// the latches.
    raise_basis: BTreeMap<(Option<String>, String, MonitorCondition), u64>,
}

impl Default for MonitorPlane {
    fn default() -> Self {
        Self {
            origin: Instant::now(),
            last_write_ns: 0,
            last_catalog_feed_ns: None,
            last_catalog_gen: None,
            engine: MonitorEngine::new(),
            discovery_rows: BTreeSet::new(),
            alerts: VecDeque::new(),
            latches: BTreeMap::new(),
            raise_basis: BTreeMap::new(),
        }
    }
}

impl MonitorPlane {
    /// Date an [`Instant`] against this plane's ONE origin.
    ///
    /// `SATURATING`, so an instant captured before the plane existed reads 0 rather
    /// than wrapping. That is reachable in the ordinary way: a handler captures
    /// `now` before taking the lock, and on the very first call the plane is
    /// constructed inside it.
    pub fn now_ns(&self, at: Instant) -> u64 {
        at.saturating_duration_since(self.origin).as_nanos() as u64
    }

    /// Date an [`Instant`] for a WRITE, latching the plane's number line so it can
    /// never run backwards.
    ///
    /// The only constructor of a [`WriteStamp`] — see that type for why a
    /// regression is an unsafe-direction bug rather than a cosmetic one, and why
    /// the discovery plane's second writer is what makes it reachable.
    ///
    /// Latching is the correct reading, not merely the safe one. What the stamp
    /// dates is the moment an observation is HANDED to the engine, which is what
    /// [`MonitorRow::last_sample_age_ms`] reports (the freshness of the FEED) and
    /// what the confirmation span measures. A catalog gathered a round trip ago is
    /// being handed over NOW, so dating it now overstates nothing: it can only
    /// SHORTEN a confirmation span, never inflate one.
    pub fn advance_to(&mut self, at: Instant) -> WriteStamp {
        self.last_write_ns = self.now_ns(at).max(self.last_write_ns);
        WriteStamp(self.last_write_ns)
    }

    /// Feed ONE ATTACHED-plane pass's samples through the engine, bank every
    /// transition in the ring, report the open regimes, and answer with the pass's
    /// deltas.
    ///
    /// The whole pass is driven from ONE [`WriteStamp`] so every row on it is dated
    /// from the same instant — the same reason `tap_liveness_snapshot` is a
    /// snapshot.
    pub fn observe_pass(&mut self, samples: &[MonitorSample], at: WriteStamp) -> SamplePassOutcome {
        let mut outcome = self.feed_samples(samples);
        outcome.rows_ineligible = self.report_table(at);
        outcome
    }

    /// Feed ONE DISCOVERY-plane pass — the catalog rows a `discover` gather
    /// carried — and RETIRE the rows a settled gather proves are gone.
    ///
    /// The key requirement lands here: these rows are topics NOBODY has
    /// attached, so this is the only path on which a verdict for an unchecked topic
    /// can exist at all.
    ///
    /// Three things this does that [`Self::observe_pass`] does not:
    ///
    /// * **It is THROTTLED, and the throttle is not optional.** This plane's
    ///   cadence is whatever a controller polls `discover` at, and the whole reason
    ///   the attached sampler is gated on `MONITOR_SAMPLE_INTERVAL_NS` — stated on
    ///   its call site, and pinned by
    ///   `monitors_v1_the_sampler_is_gated_on_the_derived_interval_not_every_pass` —
    ///   is that sampling faster re-reads ONE robot-side observation over and over.
    ///   The robot's own observer advances on a 200 ms sweep grid, so a sidebar
    ///   refreshing at 20 Hz would learn a baseline from eight copies of one
    ///   observation and count one piece of evidence eight times. The span half of
    ///   the confirmation still holds (a fast poller cannot raise early — that is
    ///   why the design demands BOTH halves), but the baseline would be a median of
    ///   one sample, so the gate is enforced HERE rather than at a call site that
    ///   could forget it.
    /// * **It TRACKS what it opened**, so the rows can be retired.
    /// * **It RETIRES rows a SETTLED gather stopped naming.** Only settled: the
    ///   `may_drop` rule `fold_streaming_mirrors_into_robots` already applies to
    ///   exactly this question — an unconverged catalog is not evidence of absence
    ///   so retiring on one would forget a live robot's rows
    ///   every time netd restarted.
    ///
    /// `attached` names the rows an attached tap owns, which this pass must have
    /// SKIPPED (the caller filters; the two sample sources are split by
    /// attachment so one row can never be fed by two cadences with two rate
    /// windows). They are excluded from the retirement for a sharper reason than
    /// tidiness: [`Self::forget`] drops a row whoever owns it, so retiring a key
    /// that is merely absent-from-the-catalog-because-Studio-attached-it would
    /// destroy the ATTACHED plane's learned baseline and settle state — silently,
    /// on the one row a user is actually looking at.
    ///
    /// The exclusion is the ONLY mechanism, deliberately. An attached key could
    /// instead be dropped from `discovery_rows` outright ("ownership transferred"),
    /// and that also works — but it is a SECOND mechanism for one rule, and it
    /// leaves the plane unable to retire the row later without the tap's `detach`
    /// having run first. Keeping the key tracked and merely excluded means a row
    /// that a tap released AND the catalog stopped naming is retired by the next
    /// settled gather, with no ordering requirement between the two events at all.
    /// `gather_gen` orders this pass against every other gather (see
    /// `MonitorPlane::catalog_superseded` — private, so it is named rather than
    /// linked: an intra-doc link from a `pub` item to a private one is a hard
    /// error under the docs gate's `-D warnings`); a pass carrying a generation
    /// below the newest already applied is a STALE ANSWER and is dropped whole.
    ///
    /// `None` means the pass was DROPPED — throttled, or superseded — and it is an
    /// `Option` rather than a zeroed outcome because one of the fields is a GAUGE.
    /// A dropped pass that reported `SamplePassOutcome::default()` would hand the
    /// daemon `rows_ineligible: 0` — a positive "nothing is being withheld from the
    /// agent right now" — on a pass that looked at nothing. The deltas would be
    /// harmless (adding zero), which is exactly why the gauge is easy to miss: the
    /// wrong value is indistinguishable from a healthy desk.
    pub fn observe_catalog_pass(
        &mut self,
        samples: &[MonitorSample],
        attached: &BTreeSet<DiscoveryRowKey>,
        settled: bool,
        at: WriteStamp,
        gather_gen: u64,
    ) -> Option<SamplePassOutcome> {
        // SUPERSEDED first, and THROTTLED second, because neither gate may leave a
        // mark behind. A stale pass that consumed the throttle would delay the next
        // legitimate feed by a whole interval on the strength of an answer it was
        // about to discard.
        if self.catalog_superseded(gather_gen) || !self.catalog_due(at) {
            return None;
        }
        self.last_catalog_feed_ns = Some(at.as_ns());
        self.last_catalog_gen = Some(gather_gen);

        let mut seen: BTreeSet<DiscoveryRowKey> = BTreeSet::new();
        for sample in samples {
            // A sample with no robot cannot be a catalog row (a `CatalogReply`
            // self-attributes), so it is not tracked — and therefore never
            // retired. Feeding it is still correct; owning it is not.
            if let Some(robot) = sample.robot.clone() {
                seen.insert((robot, sample.topic.clone()));
            }
        }
        let mut outcome = self.feed_samples(samples);
        self.discovery_rows.extend(seen.iter().cloned());

        if settled {
            for (robot, topic) in stale_discovery_rows(&self.discovery_rows, &seen, attached) {
                // Count what was really REMOVED, not what was un-tracked. The two
                // differ on an ordinary sequence: a tap takes a catalog row over,
                // `detach` forgets it, and this plane's ownership claim outlives
                // the row by one gather — so the tidy-up that follows is a
                // bookkeeping write, and reporting it as a retirement would tell
                // an operator watching LAN churn about an event that did not
                // happen.
                if self.forget(Some(&robot), &topic) {
                    outcome.retired += 1;
                }
                self.discovery_rows.remove(&(robot, topic));
            }
        }
        // AFTER the retirement, deliberately: the gauge and the regime report both
        // describe the table, and a row this pass removed is not in it. See
        // `report_table`.
        outcome.rows_ineligible = self.report_table(at);
        Some(outcome)
    }

    /// Whether a NEWER gather's answer has already been applied, making this one a
    /// stale answer to the same question.
    ///
    /// The hazard this closes is ordering by ARRIVAL. Two `discover` handlers can
    /// be in flight at once, netd gather latency varies by seconds, and the gather
    /// that started FIRST can finish LAST — arriving with a later instant and an
    /// older answer. [`WriteStamp`] cannot see it: the instant it latches is
    /// captured after the gather returns, so the stale pass looks perfectly
    /// current. What lands if it is applied is not merely stale liveness on a few
    /// rows (bad enough — `last_sample_age_ms` would read FRESH for it): its
    /// SETTLED verdict retires every row the newer catalog added, destroying
    /// learned baselines and settle state that a re-open cannot restore.
    ///
    /// Strictly `<`, because generations are unique
    /// ([`fetch_add`](std::sync::atomic::AtomicU64::fetch_add)) — an equal
    /// generation cannot arrive twice, so there is nothing for `<=` to catch.
    /// `None` (no pass applied yet) supersedes nothing.
    fn catalog_superseded(&self, gather_gen: u64) -> bool {
        self.last_catalog_gen
            .is_some_and(|applied| gather_gen < applied)
    }

    /// Whether the catalog plane's throttle has elapsed. `true` before its first
    /// pass, so a fresh daemon's first `discover` feeds immediately.
    fn catalog_due(&self, at: WriteStamp) -> bool {
        self.last_catalog_feed_ns
            .is_none_or(|prev| at.as_ns().saturating_sub(prev) >= MONITOR_SAMPLE_INTERVAL_NS)
    }

    /// The body both planes share: feed, bank, report, and answer with the deltas.
    fn feed_samples(&mut self, samples: &[MonitorSample]) -> SamplePassOutcome {
        let mut outcome = SamplePassOutcome::default();
        for sample in samples {
            outcome.observed += 1;
            for alert in self.engine.observe(sample) {
                if alert.cleared_at_ms.is_some() {
                    outcome.cleared += 1;
                } else {
                    outcome.raised += 1;
                    // Bank the basis the verdict rested on, for the regime's log
                    // lines. Absent means absent: a raise with no
                    // baseline REMOVES any stale entry rather than leaving an
                    // earlier regime's number to be reported as this one's.
                    let key = (alert.robot.clone(), alert.topic.clone(), alert.condition);
                    match alert.baseline_mhz {
                        Some(mhz) => {
                            self.raise_basis.insert(key, mhz);
                        }
                        None => {
                            self.raise_basis.remove(&key);
                        }
                    }
                }
                self.push_alert(alert);
            }
        }
        outcome
    }

    /// Describe the table as it FINALLY stands: the ineligible GAUGE, and the
    /// open condition regimes.
    ///
    /// Split from [`Self::feed_samples`] because the catalog plane RETIRES rows
    /// BETWEEN the two, and both of these are statements about the table rather
    /// than about the samples. Run before the retirement, they describe a table
    /// that no longer exists by the time the pass returns:
    ///
    /// * the GAUGE would publish rows the SAME pass then removed — and it is a
    ///   `store`, so that wrong value stands until another pass repairs it, on an
    ///   observable whose whole contract is "as of now";
    /// * the regime report would log a condition RAISED on a row that is about to
    ///   be forgotten, sending an operator to look at something that no longer
    ///   exists — and `forget` drops the latch, so the recovery line that would
    ///   normally close the regime never comes.
    fn report_table(&mut self, at: WriteStamp) -> u64 {
        let rows = self.engine.rows(at.as_ns());
        let ineligible = rows.iter().filter(|r| !r.ineligible.is_empty()).count() as u64;
        self.report_open_regimes(&rows);
        ineligible
    }

    /// Stop watching one row and discard everything learned about it, its flood
    /// latches included.
    ///
    /// Called where the OBSERVATION the row was derived from ends. Alerts the row
    /// already minted stay in the ring: they record something that really happened,
    /// and an agent that has not polled since must still be able to learn about it.
    pub fn forget(&mut self, robot: Option<&str>, topic: &str) -> bool {
        self.latches
            .retain(|(r, t, _), _| (r.as_deref(), t.as_str()) != (robot, topic));
        // The banked raise basis goes with them: it describes a producer this
        // plane has stopped watching, and a re-shown topic is a new row that must
        // earn its own (the engine's `forget` drops the row's basis for the same
        // reason).
        self.raise_basis
            .retain(|(r, t, _), _| (r.as_deref(), t.as_str()) != (robot, topic));
        self.engine.forget(robot, topic)
    }

    /// Every watched row's current state, dated at `now_ns`.
    pub fn rows(&self, now_ns: u64) -> Vec<MonitorRow> {
        self.engine.rows(now_ns)
    }

    /// The alert ring, oldest first, each entry STAMPED with its serve-time age.
    ///
    /// The stamping is the whole reason this is a method rather than a field read.
    /// `raised_at_ms` / `cleared_at_ms` belong to THIS daemon's monotonic clock,
    /// which no reader shares; [`Alert::with_age`] converts a transition into the
    /// one quantity a reader can render, measured from the transition the entry
    /// records. Doing it here — at SERVE time, in one place — is what keeps a
    /// second answer from existing.
    pub fn alerts_at(&self, now_ms: u64) -> Vec<Alert> {
        self.alerts
            .iter()
            .cloned()
            .map(|a| a.with_age(now_ms))
            .collect()
    }

    /// The next `seq` this plane will assign — the ring's high-water mark.
    pub fn next_seq(&self) -> u64 {
        self.engine.next_seq()
    }

    /// How many rows are being watched.
    pub fn watched(&self) -> usize {
        self.engine.watched()
    }

    /// Bank one transition, evicting the oldest when the ring is full.
    fn push_alert(&mut self, alert: Alert) {
        while self.alerts.len() >= MONITOR_ALERT_RING {
            self.alerts.pop_front();
        }
        self.alerts.push_back(alert);
    }

    /// Log the currently-OPEN condition regimes, flood-suppressed.
    ///
    /// # What actually floods, and why the latch is fed per SAMPLE
    ///
    /// A raise is a one-shot: once a [`ConditionTracker`] is raised it returns no
    /// further transition until it clears, so a wedged topic mints EXACTLY ONE
    /// alert and a latch fed on transitions alone would be inert — it would never
    /// see a second failure to suppress. That is the wrong place to look. What a
    /// long-running fault produces is a standing condition nobody is being
    /// reminded of, and the operator-facing failure is the opposite of a flood:
    /// one line at 03:00 and silence for the next six hours.
    ///
    /// So the latch is fed once per SAMPLING PASS for every condition currently
    /// raised, which is the shape [`FailureRegimeLatch`] was built for: a loud
    /// head, `debug!` repeats carrying the suppressed count, and a LOUD
    /// re-announcement at each DECADE of the running total — bounded by
    /// `log10(total)`, so a topic stalled for a week costs a handful of lines
    /// rather than one every 400 ms or one in total. A condition that clears calls
    /// [`FailureRegimeLatch::on_success`], which reports once IFF something was
    /// suppressed and re-arms the head.
    ///
    /// [`ConditionTracker`]: cerulion_viz::monitor::ConditionTracker
    fn report_open_regimes(&mut self, rows: &[MonitorRow]) {
        for row in rows {
            for condition in MonitorCondition::ALL {
                let key = (row.robot.clone(), row.topic.clone(), condition);
                if row.conditions.contains(&condition) {
                    // The row's own baseline where it has one, else the basis the
                    // RAISE was judged against — the two differ exactly
                    // on a stall confirmed after a baseline wipe.
                    let basis = row
                        .baseline_mhz
                        .or_else(|| self.raise_basis.get(&key).copied());
                    let latch = self.latches.entry(key).or_default();
                    report_condition(latch, row, condition, basis);
                } else if let Some(latch) = self.latches.get_mut(&key) {
                    if let Some(suppressed) = latch.on_success() {
                        tracing::info!(
                            topic = %row.topic,
                            robot = row.robot.as_deref().unwrap_or("<local>"),
                            condition = condition.as_wire(),
                            suppressed_count = suppressed,
                            total_failures = latch.total_failures(),
                            "vizd monitors: condition CLEARED — the row is being judged again \
                             and the loud head is re-armed (the running total is NOT reset: it \
                             is the run's history, not the regime's)"
                        );
                    }
                }
            }
        }
    }
}

/// PURE: which DISCOVERY-owned rows a gather proves are gone.
///
/// A row is stale when this plane opened it, this gather did NOT name it, and no
/// attached tap has taken it over. All three clauses are load-bearing:
///
/// * **`tracked`** scopes the answer to rows the DISCOVERY plane minted. An
///   attached row that a catalog never mentions is not stale, it is simply not this
///   plane's business, and retiring it would delete a live tap's baseline.
/// * **`seen`** is this gather's answer. The caller only invokes this on a SETTLED
///   gather, which is the one state that licenses an absence claim at all (the
///   `may_drop` rule `fold_streaming_mirrors_into_robots`
///   already applies to the same question).
/// * **`attached`** is the overlap window that would otherwise be a silent data
///   loss: a topic the catalog still serves but that Studio has ATTACHED is
///   deliberately absent from `seen` (the two sample sources are split by
///   attachment), so without this clause the very act of looking at a topic in
///   Studio would retire its monitor row.
///
/// Returned rather than applied so the decision is oracle-testable apart from the
/// mutation it drives.
fn stale_discovery_rows(
    tracked: &BTreeSet<DiscoveryRowKey>,
    seen: &BTreeSet<DiscoveryRowKey>,
    attached: &BTreeSet<DiscoveryRowKey>,
) -> Vec<DiscoveryRowKey> {
    tracked
        .iter()
        .filter(|key| !seen.contains(*key) && !attached.contains(*key))
        .cloned()
        .collect()
}

/// One open regime's log line, at the level the shared latch decides.
///
/// Split out so the three arms sit beside each other and the LEVEL of each is
/// visible at a glance — the discipline (`Loud` and `StillFailing` share a
/// level; only `Suppressed` is downgraded).
///
/// `baseline_mhz` is passed IN rather than read off the row, because the
/// row's field and the verdict's basis are two different numbers on
/// exactly the class this reports: a `stalled` confirmed after a baseline wipe was
/// judged on the last frozen baseline, while the row's own field is the live RATE
/// baseline and is legitimately absent. Reading the row here would drop the
/// number from the one line an operator has to judge that verdict from.
fn report_condition(
    latch: &mut FailureRegimeLatch,
    row: &MonitorRow,
    condition: MonitorCondition,
    baseline_mhz: Option<u64>,
) {
    let robot = row.robot.as_deref().unwrap_or("<local>");
    let condition = condition.as_wire();
    match latch.on_failure() {
        RegimeDecision::Loud => tracing::warn!(
            topic = %row.topic,
            robot,
            condition,
            baseline_mhz,
            observed_mhz = row.observed_mhz,
            samples = row.samples,
            total_failures = latch.total_failures(),
            "vizd monitors: a watched topic RAISED a condition — the alert is on the \
             `monitors` verb with its evidence; repeats log at debug until it clears, with \
             a loud re-announcement at each decade of the running total"
        ),
        RegimeDecision::Suppressed { suppressed } => tracing::debug!(
            topic = %row.topic,
            robot,
            condition,
            suppressed,
            total_failures = latch.total_failures(),
            "vizd monitors: condition still raised (warn suppressed)"
        ),
        RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
            topic = %row.topic,
            robot,
            condition,
            suppressed,
            baseline_mhz,
            observed_mhz = row.observed_mhz,
            total_failures = total,
            "vizd monitors: condition is STILL raised — the regime has not closed"
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::transport::liveness::LIVENESS_NO_DATA_MIN_MS;
    use cerulion_core::{LivenessState, TopicLiveness, TopicRateEstimate};
    use cerulion_viz::monitor::{
        MonitorState, MONITOR_CLEAR_SAMPLES, MONITOR_CONFIRM_MIN_SPAN_NS, MONITOR_CONFIRM_SAMPLES,
        MONITOR_SETTLE_NS,
    };
    use std::time::Duration;
    use tracing_test::traced_test;

    /// The sampler's cadence, as the plane's tests drive it.
    const STEP_NS: u64 = cerulion_viz::monitor::MONITOR_SAMPLE_INTERVAL_NS;

    /// A write stamp at a hand-chosen instant on the tests' own sample clock.
    ///
    /// These arms drive the engine's BEHAVIOUR, which is a function of the sample
    /// timestamps and nothing else, so they hand it the clock they want rather than
    /// a real [`Instant`]. The LATCH those stamps bypass is driven through the
    /// production `advance_to` by the `write_clock_tests` module below.
    const fn at(ns: u64) -> WriteStamp {
        WriteStamp::for_test(ns)
    }

    /// A liveness payload that classifies `NoData` — `frames_observed == 0` past
    /// the substrate's own no-data floor. The `silent` condition's stimulus.
    ///
    /// `observed_for_ms` is the sample clock PLUS [`LIVENESS_NO_DATA_MIN_MS`],
    /// which is the ordinary shape rather than a convenience: the robot's observer
    /// outlives this desk's sampler, so a topic it has been watching since before
    /// the daemon booted arrives already past the floor. Getting this wrong is not
    /// hypothetical — a fixture that sits just UNDER the floor makes
    /// every stimulus classify `Unknown`, every sample non-evidential, and
    /// the tests fail for a reason that has nothing to do with the plane.
    /// [`asserting_state`] is why that cannot recur silently.
    fn no_data(at_ns: u64) -> TopicLiveness {
        asserting_state(
            TopicLiveness {
                last_frame_age_ms: None,
                observed_for_ms: LIVENESS_NO_DATA_MIN_MS + at_ns / 1_000_000,
                frames_observed: 0,
                rate_estimate: None,
            },
            LivenessState::NoData,
        )
    }

    /// A liveness payload that classifies `Idle` — PRODUCED (so it can never be
    /// `silent`) with an unknown freshness, which is the undatable class: a
    /// flushed backlog, a `/tf_static` one-shot.
    ///
    /// Used as the NON-qualifying stimulus in arms that need to advance the clock
    /// without advancing a condition's streak, because it is the one evidential
    /// classification that qualifies for nothing on a row that has never streamed.
    fn idle(at_ns: u64) -> TopicLiveness {
        asserting_state(
            TopicLiveness {
                last_frame_age_ms: None,
                observed_for_ms: LIVENESS_NO_DATA_MIN_MS + at_ns / 1_000_000,
                frames_observed: 7,
                rate_estimate: None,
            },
            LivenessState::Idle,
        )
    }

    /// A liveness payload that classifies `Streaming`, carrying a trustworthy rate.
    fn streaming(at_ns: u64, mhz: u64) -> TopicLiveness {
        asserting_state(
            TopicLiveness {
                last_frame_age_ms: Some(0),
                observed_for_ms: LIVENESS_NO_DATA_MIN_MS + at_ns / 1_000_000,
                frames_observed: 100,
                rate_estimate: Some(TopicRateEstimate {
                    millihertz: mhz,
                    is_floor: false,
                }),
            },
            LivenessState::Streaming,
        )
    }

    /// Every fixture payload goes through here: a stimulus that stopped
    /// classifying the way its name claims must fail LOUDLY, not quietly stop
    /// being a stimulus.
    ///
    /// The engine's absolute rules make a mis-built payload INERT rather than
    /// wrong — an `Unknown` classification raises nothing, by design — so a
    /// substrate threshold moving under these fixtures would turn every "raises
    /// exactly once" oracle into "raises zero times", which reads as a bug in the
    /// plane. It is asserted against the substrate's OWN classifier, so the
    /// fixtures cannot drift from the thresholds the production path uses.
    fn asserting_state(liveness: TopicLiveness, expected: LivenessState) -> TopicLiveness {
        assert_eq!(
            liveness.state(),
            expected,
            "fixture no longer classifies {expected:?}: {liveness:?}"
        );
        liveness
    }

    fn sample(topic: &str, at_ns: u64, liveness: TopicLiveness) -> MonitorSample {
        MonitorSample::new(topic, None, at_ns, Some(liveness), true)
    }

    /// Drive ONE row to a confirmed `silent`, returning the plane and the sample
    /// clock it reached. Hand-computed: past the settle window, then
    /// `MONITOR_CONFIRM_SAMPLES` qualifying samples spanning at least
    /// `MONITOR_CONFIRM_MIN_SPAN_NS`.
    fn plane_with_one_raised_silent(topic: &str) -> (MonitorPlane, u64) {
        let mut plane = MonitorPlane::default();
        // t=0 opens the row; the settle window discards everything before it.
        let mut t = 0u64;
        plane.observe_pass(&[sample(topic, t, no_data(0))], at(t));
        t = MONITOR_SETTLE_NS;
        // Enough samples AND enough span: step by the real sampler cadence until
        // both halves of the confirmation are satisfied.
        let need = (MONITOR_CONFIRM_MIN_SPAN_NS / STEP_NS + 1).max(MONITOR_CONFIRM_SAMPLES as u64);
        for _ in 0..=need {
            plane.observe_pass(&[sample(topic, t, no_data(t))], at(t));
            t += STEP_NS;
        }
        (plane, t)
    }

    /// The plane really does drive the engine, and one confirmed condition lands in
    /// the ring EXACTLY once with the pass deltas to match.
    ///
    /// Hand oracle: one row, one condition, one raise. `observed` counts every
    /// sample handed over — settle-window ones included — which is the sampler's
    /// own liveness signal and is asserted separately from the alert count.
    #[test]
    fn a_confirmed_condition_lands_in_the_ring_once_with_matching_pass_deltas() {
        let mut plane = MonitorPlane::default();
        let mut t = 0u64;
        let mut totals = (0u64, 0u64, 0u64);
        plane.observe_pass(&[sample("/dead", t, no_data(0))], at(t));
        t = MONITOR_SETTLE_NS;
        for _ in 0..12 {
            let out = plane.observe_pass(&[sample("/dead", t, no_data(t))], at(t));
            totals.0 += out.observed;
            totals.1 += out.raised;
            totals.2 += out.cleared;
            t += STEP_NS;
        }
        assert_eq!(totals.0, 12, "one sample handed over per pass");
        assert_eq!(
            totals.1, 1,
            "a wedged row raises EXACTLY once, not per sample"
        );
        assert_eq!(totals.2, 0, "and never clears while it stays wedged");

        let alerts = plane.alerts_at(t / 1_000_000);
        assert_eq!(
            alerts.len(),
            1,
            "one transition, one ring entry: {alerts:?}"
        );
        assert_eq!(alerts[0].topic, "/dead");
        assert_eq!(alerts[0].condition, MonitorCondition::Silent);
        assert_eq!(alerts[0].seq, 0, "the first alert of this plane's life");
        assert_eq!(plane.next_seq(), 1);
    }

    /// A RAISE and its CLEAR are counted APART, and a clear is a NEW ring entry
    /// with a NEW `seq` carrying `cleared_at_ms` — never a mutation of the raise.
    ///
    /// Both halves guard the same slip, and it is a one-line one: classifying
    /// every transition as a raise. The wire stays plausible (two entries, two
    /// seqs, the clear's `cleared_at_ms` still set by the engine), so nothing
    /// about the payload looks wrong — while `monitor_alerts_cleared` is pinned
    /// at zero forever and `monitor_alerts_raised` double-counts every flap. That
    /// is precisely the observable the two counters are kept apart FOR: a row
    /// that raised and cleared ten times is a FLAPPER, and a net figure describes
    /// it identically to a desk on which nothing ever happened.
    ///
    /// This arm exists because a variant that reports a net figure survives the
    /// rest of the suite. Every other arm here either never clears, or asserts a FLOOR.
    #[test]
    fn a_raise_and_its_clear_are_counted_apart_and_ride_two_ring_entries() {
        let (mut plane, mut t) = plane_with_one_raised_silent("/dead");
        let raised_seq = plane.alerts.back().expect("the raise").seq;

        // MONITOR_CLEAR_SAMPLES consecutive non-qualifying evidential samples
        // retract it. `Streaming` is the realistic stimulus: the route came back.
        let mut totals = (0u64, 0u64);
        for _ in 0..MONITOR_CLEAR_SAMPLES {
            let out = plane.observe_pass(&[sample("/dead", t, streaming(t, 20_000))], at(t));
            totals.0 += out.raised;
            totals.1 += out.cleared;
            t += STEP_NS;
        }
        assert_eq!(
            totals,
            (0, 1),
            "the retraction is a CLEAR — counting it as a raise leaves \
             `monitor_alerts_cleared` pinned at zero forever"
        );

        let alerts = plane.alerts_at(t / 1_000_000);
        assert_eq!(alerts.len(), 2, "two entries, not one mutated: {alerts:?}");
        assert_eq!(alerts[0].seq, raised_seq);
        assert_eq!(alerts[0].cleared_at_ms, None, "the raise is untouched");
        assert_eq!(
            alerts[1].seq,
            raised_seq + 1,
            "an agent dedupes on `seq`, so a clear it can learn about needs a NEW one"
        );
        assert!(
            alerts[1].cleared_at_ms.is_some(),
            "and `cleared_at_ms` is the discriminator between the two kinds"
        );
        // The clear's serve-time age is measured from the CLEAR, not the raise —
        // the two entries are at different instants, so one shared answer would
        // be wrong for at least one of them.
        assert!(
            alerts[1].age_ms < alerts[0].age_ms,
            "the clear happened LATER, so it is younger: {alerts:?}"
        );
    }

    /// SERVE-TIME AGE: the ring's stamps are this daemon's monotonic clock, and
    /// what a reader gets is the age of the TRANSITION, measured at the instant the
    /// response is built.
    ///
    /// The oracle is hand-computed from the raise stamp the alert itself carries,
    /// and the SAME entry is served twice at two different instants — so an
    /// implementation that stamped the age once at mint time (or never) fails on
    /// the second serve.
    #[test]
    fn the_ring_stamps_a_serve_time_age_on_every_entry_at_every_serve() {
        let (plane, _) = plane_with_one_raised_silent("/dead");
        let raised_at_ms = plane.alerts.front().expect("one alert").raised_at_ms;
        assert_eq!(
            plane.alerts.front().expect("one alert").age_ms,
            None,
            "the engine MINTS an alert with no age — the age is a serve-time fact"
        );

        let first = plane.alerts_at(raised_at_ms + 1_000);
        assert_eq!(first[0].age_ms, Some(1_000), "one second after the raise");
        let second = plane.alerts_at(raised_at_ms + 40_000);
        assert_eq!(second[0].age_ms, Some(40_000), "forty seconds after it");
        // The stamps themselves are untouched by serving — a client dedupes on
        // `seq` and orders on these, so a serve that mutated them would corrupt
        // both.
        assert_eq!(first[0].raised_at_ms, raised_at_ms);
        assert_eq!(second[0].raised_at_ms, raised_at_ms);
        assert_eq!(first[0].seq, second[0].seq);
    }

    /// A serve instant BELOW the transition stamp saturates to zero rather than
    /// wrapping. Reachable in the ordinary way (a `now` captured before the lock,
    /// a transition banked inside it), and `u64::MAX` milliseconds on screen is a
    /// worse answer than "just now".
    #[test]
    fn a_serve_instant_below_the_transition_stamp_reads_zero_not_a_wrapped_age() {
        let (plane, _) = plane_with_one_raised_silent("/dead");
        let raised_at_ms = plane.alerts.front().expect("one alert").raised_at_ms;
        assert!(raised_at_ms > 0, "the fixture must raise past t=0");
        assert_eq!(plane.alerts_at(raised_at_ms - 1)[0].age_ms, Some(0));
    }

    /// The ring is BOUNDED and evicts the OLDEST — with the `seq` continuing to
    /// climb, which is what makes the loss detectable by an agent that dedupes on
    /// it.
    ///
    /// Hand oracle: `MONITOR_ALERT_RING + 3` transitions in, exactly
    /// `MONITOR_ALERT_RING` retained, the first retained `seq` is 3, and the last
    /// is `MONITOR_ALERT_RING + 2`.
    #[test]
    fn the_alert_ring_is_bounded_and_evicts_the_oldest_while_seq_keeps_climbing() {
        let mut plane = MonitorPlane::default();
        let overflow = MONITOR_ALERT_RING as u64 + 3;
        for seq in 0..overflow {
            plane.push_alert(Alert {
                seq,
                topic: format!("/t{seq}"),
                robot: None,
                condition: MonitorCondition::Silent,
                raised_at_ms: seq,
                cleared_at_ms: None,
                age_ms: None,
                observed_mhz: None,
                baseline_mhz: None,
                liveness_state: Some(LivenessState::NoData),
            });
        }
        assert_eq!(plane.alerts.len(), MONITOR_ALERT_RING);
        assert_eq!(plane.alerts.front().expect("full ring").seq, 3);
        assert_eq!(
            plane.alerts.back().expect("full ring").seq,
            MONITOR_ALERT_RING as u64 + 2
        );
    }

    /// `forget` drops the row AND its latches, and a re-shown row starts over —
    /// settle window included, so nothing learned about the old producer is
    /// carried onto the new one.
    ///
    /// The alert the old row minted STAYS in the ring: it records something that
    /// really happened, and an agent that has not polled since must still learn
    /// about it.
    #[test]
    fn forget_drops_the_row_and_its_latches_but_never_the_history_it_already_minted() {
        let (mut plane, t) = plane_with_one_raised_silent("/dead");
        assert_eq!(plane.watched(), 1);
        assert_eq!(plane.latches.len(), 1, "one open regime, one latch");

        assert!(plane.forget(None, "/dead"), "the row existed");
        assert_eq!(plane.watched(), 0);
        assert!(plane.latches.is_empty(), "the latches went with the row");
        assert_eq!(plane.alerts.len(), 1, "the HISTORY is not rewritten");
        assert!(!plane.forget(None, "/dead"), "and it is gone for good");

        // Re-shown: a brand-new row, inside a brand-new settle window, so the
        // first sample teaches it nothing.
        plane.observe_pass(&[sample("/dead", t, no_data(t))], at(t));
        let rows = plane.rows(t);
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].samples, 0, "the settle window discarded it");
        assert_eq!(rows[0].state, MonitorState::Unknown);
        assert!(rows[0].conditions.is_empty(), "and nothing is raised");
    }

    /// `forget` is keyed by `(robot, topic)`: releasing one robot's row leaves the
    /// same topic on another robot — and the LOCAL row — untouched.
    #[test]
    fn forget_releases_one_robots_row_and_leaves_its_namesakes_alone() {
        let mut plane = MonitorPlane::default();
        let s = |robot: Option<&str>| {
            MonitorSample::new(
                "/lowstate",
                robot.map(str::to_string),
                0,
                Some(streaming(0, 20_000)),
                true,
            )
        };
        plane.observe_pass(&[s(None), s(Some("go2")), s(Some("spot"))], at(0));
        assert_eq!(plane.watched(), 3, "three data sources, three rows");

        assert!(plane.forget(Some("go2"), "/lowstate"));
        let rows = plane.rows(0);
        assert_eq!(rows.len(), 2);
        let robots: Vec<Option<&str>> = rows.iter().map(|r| r.robot.as_deref()).collect();
        assert_eq!(robots, vec![None, Some("spot")], "local first, then spot");
    }

    /// The GAUGE is a statement about NOW: it reports how many rows are currently
    /// withholding a condition, and it goes DOWN again when they stop.
    ///
    /// Hand oracle: a row with no evidence at all withholds all three conditions
    /// (`no_liveness_evidence`), so one row ⇒ 1; once it is streaming with a
    /// trustworthy rate nothing is permanently withheld ⇒ 0. A running total could
    /// not fall and would fail the second half.
    #[test]
    fn the_ineligible_gauge_reports_now_and_falls_again() {
        let mut plane = MonitorPlane::default();
        // A blind sample past the settle window: admitted by the sampler, learned
        // from by nothing.
        let blind = MonitorSample::new("/x", None, 0, None, true);
        plane.observe_pass(std::slice::from_ref(&blind), at(0));
        let t = MONITOR_SETTLE_NS;
        let out = plane.observe_pass(&[MonitorSample::new("/x", None, t, None, true)], at(t));
        assert_eq!(
            out.rows_ineligible, 1,
            "nothing is being judged on this row"
        );

        let mut t = t;
        for _ in 0..3 {
            t += STEP_NS;
            plane.observe_pass(&[sample("/x", t, streaming(t, 20_000))], at(t));
        }
        let out = plane.observe_pass(&[sample("/x", t, streaming(t, 20_000))], at(t));
        assert_eq!(
            out.rows_ineligible, 0,
            "evidence is flowing — nothing is permanently withheld"
        );
    }

    /// DETERMINISM (Principle #7): the same sample vector through two fresh planes
    /// yields byte-identical alerts, `seq` numbering included.
    ///
    /// Both are compared against each other AND against a hand oracle for the one
    /// fact that fixes the whole sequence, so this is not a self-compare.
    #[test]
    fn the_same_sample_vector_yields_byte_identical_alerts_twice() {
        let run = || {
            let (plane, t) = plane_with_one_raised_silent("/dead");
            plane.alerts_at(t / 1_000_000)
        };
        let a = run();
        let b = run();
        assert_eq!(
            serde_json::to_string(&a).expect("serializes"),
            serde_json::to_string(&b).expect("serializes")
        );
        assert_eq!(a.len(), 1);
        assert_eq!(a[0].condition, MonitorCondition::Silent);
        assert_eq!(a[0].seq, 0);
    }

    // ── the LATCHED write clock ────────────────────────────────────────

    /// The plane's WRITE clock never runs backwards, however the two writers
    /// interleave.
    ///
    /// The attached plane alone cannot produce an inversion — one writer, one thread. The
    /// discovery plane can, two ways: a `discover` handler captures its instant and then spends a
    /// netd round trip before it reaches the engine, while the drain loop keeps
    /// feeding; and two controller threads racing two `discover` calls invert
    /// without any gather at all.
    ///
    /// Driven through the PRODUCTION `advance_to` with hand-built instants (no
    /// sleeping — [`Instant`] arithmetic is exact and a real elapsed wall would be
    /// the load-sensitive class).
    #[test]
    fn the_write_clock_never_runs_backwards_when_two_writers_race() {
        let mut plane = MonitorPlane::default();
        let base = Instant::now();
        let late = plane.advance_to(base + Duration::from_secs(7));
        let stale = plane.advance_to(base + Duration::from_secs(6));
        assert_eq!(
            stale, late,
            "a writer that captured its instant BEFORE a round trip must not drag \
             the number line back under the writer that banked while it waited"
        );
        // …and the latch is a FLOOR, not a freeze: a genuinely later write still
        // advances, or the clock would stop at the first stamp.
        let later = plane.advance_to(base + Duration::from_secs(8));
        assert!(
            later > late,
            "the latch must not freeze the clock: {later:?} vs {late:?}"
        );
    }

    /// The READ clock never advances the WRITERS' number line.
    ///
    /// `now_ns` is what the `monitors` handler dates a response with, and it is
    /// deliberately NOT latched: a serving handler that moved the write clock would
    /// let a client's polling cadence decide where the next confirmation span is
    /// anchored — the same "cadence depends on who is asking" defect the sampler's
    /// gate exists to prevent, arriving through the read path instead.
    ///
    /// The mark is not directly readable, so it is probed the only available way: a
    /// later `advance_to` must return ITS OWN instant. If the read had latched, the
    /// write would have been dragged up to the far-future read and this would come
    /// back at 90 s instead of 8 s.
    #[test]
    fn the_read_clock_never_advances_the_writers_number_line() {
        let mut plane = MonitorPlane::default();
        let base = Instant::now();
        plane.advance_to(base + Duration::from_secs(7));
        let far = plane.now_ns(base + Duration::from_secs(90));
        let next = plane.advance_to(base + Duration::from_secs(8));
        assert!(
            next.as_ns() < far,
            "a READ at {far} ns must leave the write clock alone, so the next write \
             lands at its own instant: {next:?}"
        );
    }

    /// THE HEADLINE HAZARD: a stale write cannot anchor a qualifying run and
    /// confirm a condition EARLY.
    ///
    /// [`ConditionTracker`](cerulion_viz::monitor::ConditionTracker) anchors
    /// `qualifying_since_ns` at the FIRST qualifying sample of a run and confirms
    /// when `now - anchor >= MONITOR_CONFIRM_MIN_SPAN_NS`. A controller thread that
    /// captured its instant a round trip ago anchors the run in the PAST, so the
    /// span reads longer than the observation really was and the condition confirms
    /// inside the window a lull-free restart is still healing in, a false
    /// `stalled`/`silent` on a topic that is fine. That is the whole reason
    /// [`MONITOR_CONFIRM_MIN_SPAN_NS`](cerulion_viz::monitor::MONITOR_CONFIRM_MIN_SPAN_NS)
    /// exists, and an unlatched clock hands it back.
    ///
    /// The stimulus is built so BOTH answers are reachable and they DIFFER: the
    /// stale capture is 1 s behind the high-water mark, and the four qualifying
    /// samples span only 300 ms of real time. Latched ⇒ span 300 ms ⇒ no raise;
    /// unlatched ⇒ span 1.3 s ⇒ raise. The second half then drives a REAL
    /// `MONITOR_CONFIRM_MIN_SPAN_NS` of observation and requires the raise to
    /// arrive, so this is not a test that passes by never raising.
    #[test]
    fn a_stale_write_cannot_anchor_a_run_and_confirm_a_condition_early() {
        let mut plane = MonitorPlane::default();
        let base = Instant::now();
        let feed = |plane: &mut MonitorPlane, at: Instant, dead: bool| {
            let at = plane.advance_to(at);
            let ns = at.as_ns();
            let liveness = if dead { no_data(ns) } else { idle(ns) };
            plane.observe_pass(&[sample("/uslam/cloud_map", ns, liveness)], at)
        };

        // Open the row, then clear its settle window with a NON-qualifying
        // (undatable-`Idle`) observation. The second one also parks the high-water
        // mark a full second ahead of the stale capture below.
        feed(&mut plane, base, false);
        feed(&mut plane, base + Duration::from_secs(6), false);
        feed(&mut plane, base + Duration::from_secs(7), false);

        // The STALE write: a `discover` that captured `base + 6s` and only now, a
        // round trip later, reaches the engine. It is the run's ANCHOR.
        let mut raised = 0u64;
        raised += feed(&mut plane, base + Duration::from_secs(6), true).raised;
        for ms in [7_100u64, 7_200, 7_300] {
            raised += feed(&mut plane, base + Duration::from_millis(ms), true).raised;
        }
        assert_eq!(
            raised, 0,
            "four qualifying samples spanning 300 ms must NOT confirm — an \
             unlatched clock anchors this run at the stale capture and measures a \
             1.3 s span it never observed"
        );
        assert!(plane.alerts.is_empty(), "and nothing reaches the ring");

        // ANTI-TAUTOLOGY: once a REAL confirmation span has been observed, it
        // raises. (7.0 s is where the latch anchored the run; 8.3 s is 1.3 s later,
        // past MONITOR_CONFIRM_MIN_SPAN_NS.)
        raised += feed(&mut plane, base + Duration::from_millis(8_300), true).raised;
        assert_eq!(raised, 1, "a genuinely long-enough run still raises");
        assert_eq!(plane.alerts.len(), 1);
        assert_eq!(plane.alerts[0].condition, MonitorCondition::Silent);
    }

    // ── the DISCOVERY plane ────────────────────────────────────────────

    /// One catalog-plane sample: a topic on a robot.
    fn catalog_sample(robot: &str, topic: &str, at: WriteStamp, settled: bool) -> MonitorSample {
        MonitorSample::new(
            topic,
            Some(robot.to_string()),
            at.as_ns(),
            Some(no_data(at.as_ns())),
            settled,
        )
    }

    fn keys(pairs: &[(&str, &str)]) -> BTreeSet<DiscoveryRowKey> {
        pairs
            .iter()
            .map(|(r, t)| ((*r).to_string(), (*t).to_string()))
            .collect()
    }

    /// The retirement predicate answers its hand-written vectors, and EACH of its
    /// three clauses is probed on its own.
    #[test]
    fn stale_discovery_rows_answers_its_hand_written_vectors() {
        let tracked = keys(&[("go2", "/a"), ("go2", "/b"), ("spot", "/c")]);

        // Nothing tracked is stale when the gather still names it all.
        assert!(stale_discovery_rows(&tracked, &tracked, &BTreeSet::new()).is_empty());

        // A row the gather stopped naming IS stale — and only that row.
        assert_eq!(
            stale_discovery_rows(
                &tracked,
                &keys(&[("go2", "/a"), ("spot", "/c")]),
                &BTreeSet::new()
            ),
            vec![("go2".to_string(), "/b".to_string())]
        );

        // A whole robot going quiet retires ITS rows and leaves the others.
        assert_eq!(
            stale_discovery_rows(&tracked, &keys(&[("spot", "/c")]), &BTreeSet::new()),
            vec![
                ("go2".to_string(), "/a".to_string()),
                ("go2".to_string(), "/b".to_string())
            ]
        );

        // THE ATTACHED CLAUSE: a topic Studio attached is deliberately absent from
        // `seen` (the two sample sources are split by attachment), so without this
        // the act of LOOKING at a topic would retire its monitor row — and
        // `forget` drops a row whoever owns it, so the loss would be the ATTACHED
        // plane's baseline, on the one row a user is watching.
        assert!(stale_discovery_rows(
            &tracked,
            &keys(&[("go2", "/a"), ("spot", "/c")]),
            &keys(&[("go2", "/b")])
        )
        .is_empty());

        // A row that was never tracked is never retired, however absent it is —
        // this is what keeps the predicate off the attached plane's rows entirely.
        assert!(
            stale_discovery_rows(&BTreeSet::new(), &BTreeSet::new(), &BTreeSet::new()).is_empty()
        );
    }

    /// A SETTLED catalog pass retires the rows it stopped naming; an UNCONVERGED
    /// one retires nothing.
    ///
    /// Hand oracle throughout. The unconverged half is the absence rule — an empty
    /// or short catalog proves nothing then, so retiring on one would drop a live
    /// robot's rows every time netd restarted — and it is checked on the SAME
    /// stimulus that retires when settled, so the two cannot pass for the same
    /// reason.
    #[test]
    fn a_settled_catalog_pass_retires_the_rows_it_stopped_naming_and_an_unsettled_one_does_not() {
        let step = MONITOR_SAMPLE_INTERVAL_NS;
        let none = BTreeSet::new();

        // Both rows opened, by a settled gather.
        let mut plane = MonitorPlane::default();
        let t0 = at(0);
        plane.observe_catalog_pass(
            &[
                catalog_sample("go2", "/a", t0, true),
                catalog_sample("go2", "/b", t0, true),
            ],
            &none,
            true,
            t0,
            0,
        );
        assert_eq!(plane.watched(), 2, "two catalog rows opened");

        // An UNCONVERGED gather that names only `/a` must retire nothing.
        let t1 = at(step);
        plane.observe_catalog_pass(
            &[catalog_sample("go2", "/a", t1, false)],
            &none,
            false,
            t1,
            1,
        );
        assert_eq!(
            plane.watched(),
            2,
            "an unconverged catalog is not evidence of absence"
        );

        // The SAME shortfall, SETTLED, retires `/b`.
        let t2 = at(2 * step);
        plane.observe_catalog_pass(&[catalog_sample("go2", "/a", t2, true)], &none, true, t2, 2);
        let rows = plane.rows(t2.as_ns());
        assert_eq!(rows.len(), 1, "…and a settled one does: {rows:?}");
        assert_eq!(rows[0].topic, "/a");
    }

    /// A row an ATTACHED tap has taken over survives a settled gather that no
    /// longer lists it on this plane — because the tap owns it now.
    ///
    /// This is the overlap window every remote attach passes through, and getting
    /// it wrong is silent: [`MonitorPlane::forget`] drops a row whoever owns it, so
    /// the retirement would delete the baseline and settle state of the one row a
    /// user is actually looking at, on the very next `discover` the sidebar makes.
    #[test]
    fn a_row_an_attached_tap_took_over_is_not_retired_by_the_catalog_plane() {
        let step = MONITOR_SAMPLE_INTERVAL_NS;
        let mut plane = MonitorPlane::default();
        let t0 = at(0);
        plane.observe_catalog_pass(
            &[
                catalog_sample("go2", "/a", t0, true),
                catalog_sample("go2", "/b", t0, true),
            ],
            &BTreeSet::new(),
            true,
            t0,
            0,
        );
        assert_eq!(plane.watched(), 2);

        // Studio attaches `/b`: the caller now SKIPS it (the attached sampler feeds
        // it instead) and declares it attached.
        let attached = keys(&[("go2", "/b")]);
        let t1 = at(step);
        plane.observe_catalog_pass(
            &[catalog_sample("go2", "/a", t1, true)],
            &attached,
            true,
            t1,
            1,
        );
        let rows = plane.rows(t1.as_ns());
        let watched: Vec<&str> = rows.iter().map(|r| r.topic.as_str()).collect();
        assert_eq!(
            watched,
            vec!["/a", "/b"],
            "the attached row survives: {rows:?}"
        );

        // …and once the tap goes away and the catalog still does not name it, it
        // retires normally. (`detach` is what forgets it in production; here the
        // point is that the plane stops PROTECTING it, so the row set is not
        // permanently pinned by a stale attachment claim.)
        let t2 = at(2 * step);
        plane.observe_catalog_pass(
            &[catalog_sample("go2", "/a", t2, true)],
            &BTreeSet::new(),
            true,
            t2,
            2,
        );
        assert_eq!(
            plane.watched(),
            1,
            "…and is retired once the tap releases it"
        );
    }

    /// The catalog plane is THROTTLED to the derived interval, and the throttle is
    /// enforced by the plane rather than by a call site.
    ///
    /// This plane's cadence is whatever a controller polls `discover` at, so
    /// without the gate a sidebar refreshing at 20 Hz would count ONE robot-side
    /// observation (the observer's sweep grid is 200 ms) eight times and learn a
    /// "baseline" that is a median of one sample. The span half of the confirmation
    /// still holds — a fast poller cannot RAISE early, which is why the design
    /// demands both halves — so the damage is silent, and no alert oracle would
    /// see it.
    ///
    /// Hand oracle on `observed`, which counts samples HANDED to the engine: a
    /// dropped pass hands over none. The boundary is pinned on BOTH sides.
    #[test]
    fn the_catalog_plane_is_throttled_to_the_derived_interval() {
        let step = MONITOR_SAMPLE_INTERVAL_NS;
        let none = BTreeSet::new();
        let mut plane = MonitorPlane::default();

        // Each feed is a SUCCESSIVE gather, so its generation rises with it — `ns`
        // is strictly increasing across the drives below and serves as the ticket.
        // This arm is about the THROTTLE, so nothing here may be superseded.
        let feed = |plane: &mut MonitorPlane, ns: u64| {
            let t = at(ns);
            plane
                .observe_catalog_pass(&[catalog_sample("go2", "/a", t, true)], &none, true, t, ns)
                .map_or(0, |o| o.observed)
        };

        assert_eq!(feed(&mut plane, 0), 1, "the first pass always feeds");
        assert_eq!(feed(&mut plane, 1), 0, "…and an immediate re-poll does not");
        assert_eq!(
            feed(&mut plane, step - 1),
            0,
            "one nanosecond under the interval is still too soon"
        );
        assert_eq!(
            feed(&mut plane, step),
            1,
            "…and exactly AT the interval it feeds again"
        );
        // The mark advances to the pass that RAN, not to the one that was dropped:
        // a mark advanced on a dropped pass would let a fast poller push the next
        // feed out forever.
        assert_eq!(feed(&mut plane, 2 * step - 1), 0);
        assert_eq!(feed(&mut plane, 2 * step), 1);
    }

    /// A DROPPED pass is a no-op in every respect — it must not retire a row on the
    /// strength of a gather it never looked at.
    ///
    /// The throttle returns before the retirement, and that ordering is the whole
    /// arm: a throttle placed AFTER it would let a fast poller retire every row
    /// whose robot happened to be missing from one gather, while the `observed`
    /// oracle above stayed green.
    #[test]
    fn a_throttled_catalog_pass_retires_nothing() {
        let mut plane = MonitorPlane::default();
        let t0 = at(0);
        plane.observe_catalog_pass(
            &[
                catalog_sample("go2", "/a", t0, true),
                catalog_sample("go2", "/b", t0, true),
            ],
            &BTreeSet::new(),
            true,
            t0,
            0,
        );
        assert_eq!(plane.watched(), 2);

        // A settled gather naming NOTHING, arriving inside the throttle window.
        let t1 = at(1);
        let out = plane.observe_catalog_pass(&[], &BTreeSet::new(), true, t1, 1);
        assert_eq!(
            out, None,
            "a dropped pass reports NOTHING — a zeroed outcome would hand the \
             daemon `rows_ineligible: 0`, a positive claim that nothing is being \
             withheld, made by a pass that looked at nothing"
        );
        assert_eq!(
            plane.watched(),
            2,
            "…including retiring rows on the strength of a gather it skipped"
        );
    }

    /// A SLOW gather that hands over LAST carrying an OLDER answer is dropped
    /// whole — its evidence AND its retirements.
    ///
    /// The shape is ordinary: two `discover` handlers in flight (a Studio sidebar
    /// and a `cerulion viz`, or one controller refreshing while another attaches),
    /// netd gather latency varying by seconds, and the gather that started FIRST
    /// finishing LAST. [`WriteStamp`] cannot see it — the instant it latches is
    /// captured AFTER the gather returns, so the stale pass carries a perfectly
    /// current one and the clock latch is a no-op on it.
    ///
    /// Both halves of the damage are driven here, and the second is the sharp one:
    /// a stale SETTLED verdict names fewer topics, so it RETIRES the rows the
    /// newer catalog added — destroying learned baselines and settle state that
    /// re-opening cannot restore, on rows that are perfectly healthy.
    #[test]
    fn a_slow_gather_that_hands_over_last_cannot_apply_its_older_answer() {
        let mut plane = MonitorPlane::default();
        let none = BTreeSet::new();

        // The NEWER gather (generation 7) lands first — three topics.
        let t0 = at(0);
        let newer = [
            catalog_sample("go2", "/a", t0, true),
            catalog_sample("go2", "/b", t0, true),
            catalog_sample("go2", "/c", t0, true),
        ];
        let out = plane
            .observe_catalog_pass(&newer, &none, true, t0, 7)
            .expect("the newer gather is applied");
        assert_eq!(out.observed, 3);
        assert_eq!(plane.watched(), 3, "three rows opened");

        // The OLDER gather (generation 3) was started first and is only NOW
        // handing over. Its instant is LATER (the throttle is satisfied) and its
        // catalog is SHORT — it never saw `/c`.
        let t1 = at(MONITOR_SAMPLE_INTERVAL_NS * 4);
        let older = [
            catalog_sample("go2", "/a", t1, true),
            catalog_sample("go2", "/b", t1, true),
        ];
        assert!(
            plane.catalog_due(t1),
            "precondition: the THROTTLE would let this pass through, so the \
             generation is the only thing that can stop it"
        );
        let out = plane.observe_catalog_pass(&older, &none, true, t1, 3);
        assert_eq!(out, None, "a superseded gather is dropped whole");
        assert_eq!(
            plane.watched(),
            3,
            "…and `/c` SURVIVES: a stale SETTLED verdict must not retire a row the \
             newer catalog still contains: {:?}",
            plane.rows(t1.as_ns())
        );

        // The drop leaves NO MARK: a stale pass must neither consume the throttle
        // nor raise the generation bar, or it would penalise the next legitimate
        // gather for arriving behind it.
        let t2 = at(MONITOR_SAMPLE_INTERVAL_NS * 5);
        let out = plane
            .observe_catalog_pass(&newer, &none, true, t2, 8)
            .expect("the next legitimate gather is unaffected");
        assert_eq!(out.observed, 3);
    }

    /// The gauge a retiring pass publishes describes the table it LEAVES, not the
    /// one it found.
    ///
    /// `rows_ineligible` is a GAUGE and it is `store`d, so a value counted before
    /// the same pass's retirements is not merely early — it STANDS until another
    /// pass happens to repair it, on the one observable whose entire contract is
    /// "as of now". The shape is the ordinary one: a discovery row that has never
    /// been given evidence withholds all three conditions, and a robot that stops
    /// announcing it is exactly what retires it.
    ///
    /// The oracle separates the two answers: BEFORE the retiring pass the row is
    /// legitimately counted (1), and the retiring pass itself must report 0 while
    /// also reporting the retirement — so a test that merely read 0 at the end
    /// could not tell a fixed ordering from a row that was never counted at all.
    #[test]
    fn a_retiring_pass_publishes_the_gauge_of_the_table_it_leaves() {
        let mut plane = MonitorPlane::default();
        let none = BTreeSet::new();
        let blind = |ns: u64| {
            [MonitorSample::new(
                "/uslam/cloud_map",
                Some("go2".to_string()),
                at(ns).as_ns(),
                None,
                true,
            )]
        };

        // Open the row, clear its settle window, and confirm it IS ineligible —
        // nothing is being judged on it, which is what the gauge counts.
        plane
            .observe_catalog_pass(&blind(0), &none, true, at(0), 0)
            .expect("applied");
        let settled_at = MONITOR_SETTLE_NS;
        let out = plane
            .observe_catalog_pass(&blind(settled_at), &none, true, at(settled_at), 1)
            .expect("applied");
        assert_eq!(
            out.rows_ineligible, 1,
            "precondition: a row with no evidence withholds every condition, so \
             the gauge counts it while it exists"
        );

        // The robot stops announcing it. THE PIN: the same pass retires the row,
        // so the gauge it publishes must describe a table without it.
        let after = settled_at + MONITOR_SAMPLE_INTERVAL_NS;
        let out = plane
            .observe_catalog_pass(&[], &none, true, at(after), 2)
            .expect("applied");
        assert_eq!(out.retired, 1, "precondition: the row really was retired");
        assert_eq!(
            out.rows_ineligible, 0,
            "the GAUGE is a store, so counting a row this pass then REMOVED leaves \
             the wrong value standing until some later pass repairs it — on the \
             one observable whose contract is `as of now`"
        );
        assert_eq!(plane.watched(), 0, "…and the table really is empty");
    }

    /// A condition RAISED on a row the same pass RETIRES is not reported as an
    /// open regime.
    ///
    /// The other half of the ordering, and the operator-facing one. `forget` drops
    /// the latch with the row, so a regime line emitted for a row that is about to
    /// be retired can NEVER be closed — the recovery line that would normally end
    /// it belongs to a latch that no longer exists. The operator is left with a
    /// standing fault on a topic that is not on the verb.
    ///
    /// Driven to stop at the RAISE, so the hand oracle is exactly ONE regime line
    /// for this row's whole life: the loud head. Reporting before the retirement
    /// adds a second (the suppressed repeat), which is the discriminator.
    #[traced_test]
    #[test]
    fn a_condition_on_a_row_the_pass_retires_is_not_reported_as_open() {
        let mut plane = MonitorPlane::default();
        let none = BTreeSet::new();
        let feed = |plane: &mut MonitorPlane, t: u64, gen: u64| {
            let stamp = at(t);
            plane.observe_catalog_pass(
                &[MonitorSample::new(
                    "/dead",
                    Some("go2".to_string()),
                    stamp.as_ns(),
                    Some(no_data(t)),
                    true,
                )],
                &none,
                true,
                stamp,
                gen,
            )
        };

        // Open the row, then feed until the condition CONFIRMS — and stop there.
        feed(&mut plane, 0, 0).expect("applied");
        let mut t = MONITOR_SETTLE_NS;
        let mut gen = 0u64;
        let mut raised = false;
        for _ in 0..64 {
            gen += 1;
            if feed(&mut plane, t, gen).expect("applied").raised == 1 {
                raised = true;
                break;
            }
            t += STEP_NS;
        }
        assert!(raised, "precondition: the condition confirmed");
        assert_eq!(plane.latches.len(), 1, "one open regime, one latch");

        // THE RETIRING PASS: the robot stops announcing it.
        t += STEP_NS;
        let out = plane
            .observe_catalog_pass(&[], &none, true, at(t), gen + 1)
            .expect("applied");
        assert_eq!(out.retired, 1, "precondition: the row really was retired");
        assert!(
            plane.latches.is_empty(),
            "the retirement took the latch with the row"
        );

        logs_assert(|lines: &[&str]| {
            let regime = lines
                .iter()
                .filter(|l| {
                    l.contains("RAISED a condition")
                        || l.contains("still raised")
                        || l.contains("STILL raised")
                })
                .count();
            if regime == 1 {
                Ok(())
            } else {
                Err(format!(
                    "expected exactly ONE regime line (the loud head) — reporting \
                     the table BEFORE the retirement adds a repeat for a row this \
                     pass removed, and `forget` takes the latch, so nothing can \
                     ever close it. Got {regime}:\n{lines:#?}"
                ))
            }
        });
    }

    /// The generation gate is a THRESHOLD, pinned on both sides, and it is the
    /// APPLIED generation that sets the bar — not the newest one merely seen.
    #[test]
    fn only_a_gather_older_than_the_applied_one_is_superseded() {
        let mut plane = MonitorPlane::default();
        let none = BTreeSet::new();
        let s = |ns: u64| [catalog_sample("go2", "/a", at(ns), true)];
        let step = MONITOR_SAMPLE_INTERVAL_NS;

        assert!(
            !plane.catalog_superseded(0),
            "nothing is superseded before a pass has ever been applied"
        );
        plane
            .observe_catalog_pass(&s(0), &none, true, at(0), 5)
            .expect("applied");
        assert!(plane.catalog_superseded(4), "older: superseded");
        assert!(!plane.catalog_superseded(5), "the SAME generation is not");
        assert!(!plane.catalog_superseded(6), "newer: not");

        // A pass the THROTTLE dropped never becomes the bar — its answer was not
        // used, so a gather older than it but newer than the applied one must
        // still be allowed to land.
        let dropped = plane.observe_catalog_pass(&s(1), &none, true, at(1), 9);
        assert_eq!(dropped, None, "precondition: throttled");
        assert!(
            !plane.catalog_superseded(6),
            "a THROTTLED generation must not raise the bar — its evidence was \
             never applied, so nothing newer than the APPLIED pass is stale"
        );
        plane
            .observe_catalog_pass(&s(step), &none, true, at(step), 6)
            .expect("generation 6 still lands");
    }

    /// A retirement is counted only when a row was really REMOVED — an ownership
    /// claim that outlived its row is tidied up silently.
    ///
    /// The sequence is ordinary, not contrived: a catalog opens a row, a tap takes
    /// it over, `detach` forgets it, and this plane's tracking entry survives until
    /// the next settled gather stops naming it. Counting that tidy-up would report
    /// a retirement to an operator watching LAN churn for an event that did not
    /// happen — and the row set is the thing they are being told about.
    #[test]
    fn an_ownership_claim_that_outlived_its_row_is_tidied_up_without_being_counted() {
        let mut plane = MonitorPlane::default();
        let t0 = at(0);
        let out = plane.observe_catalog_pass(
            &[catalog_sample("go2", "/a", t0, true)],
            &BTreeSet::new(),
            true,
            t0,
            0,
        );
        assert_eq!(out.expect("the pass ran").retired, 0);
        assert_eq!(plane.watched(), 1);

        // Something else releases the row — in production a `detach`, whose tap had
        // taken this catalog row over.
        assert!(plane.forget(Some("go2"), "/a"), "the row existed");
        assert_eq!(plane.watched(), 0);

        // The gather stops naming it. The tracking entry goes, and NOTHING is
        // reported: there was no row left to retire.
        let t1 = at(MONITOR_SAMPLE_INTERVAL_NS);
        let out = plane
            .observe_catalog_pass(&[], &BTreeSet::new(), true, t1, 1)
            .expect("the pass ran");
        assert_eq!(
            out.retired, 0,
            "the claim was tidied up, but no row was retired"
        );
        assert!(
            plane.discovery_rows.is_empty(),
            "…and the claim really is gone: {:?}",
            plane.discovery_rows
        );

        // ANTI-TAUTOLOGY: a row that IS still there is retired, and counted.
        let t2 = at(2 * MONITOR_SAMPLE_INTERVAL_NS);
        plane.observe_catalog_pass(
            &[catalog_sample("go2", "/b", t2, true)],
            &BTreeSet::new(),
            true,
            t2,
            2,
        );
        let t3 = at(3 * MONITOR_SAMPLE_INTERVAL_NS);
        let out = plane
            .observe_catalog_pass(&[], &BTreeSet::new(), true, t3, 3)
            .expect("the pass ran");
        assert_eq!(out.retired, 1, "a real retirement still counts");
    }

    /// The two planes share ONE row table, and the discovery plane only ever claims
    /// ownership of rows that name a robot.
    ///
    /// A `MonitorSample` with `robot: None` is a genuine LOCAL producer, which no
    /// catalog can describe (a `CatalogReply` self-attributes). Such a sample is
    /// still FED — the engine's row table is one table — but it is never TRACKED,
    /// so a settled gather can never retire a local row it was never able to see.
    #[test]
    fn the_discovery_plane_never_claims_ownership_of_a_local_row() {
        let mut plane = MonitorPlane::default();
        let t0 = at(0);
        let local = MonitorSample::new("/local", None, t0.as_ns(), Some(no_data(0)), true);
        plane.observe_catalog_pass(
            &[local, catalog_sample("go2", "/a", t0, true)],
            &BTreeSet::new(),
            true,
            t0,
            0,
        );
        assert_eq!(plane.watched(), 2, "both rows were fed");
        assert_eq!(
            plane.discovery_rows,
            keys(&[("go2", "/a")]),
            "…but only the catalog row is OWNED by this plane"
        );

        // A settled gather naming nothing retires the catalog row and leaves the
        // local one alone.
        let t1 = at(MONITOR_SAMPLE_INTERVAL_NS);
        plane.observe_catalog_pass(&[], &BTreeSet::new(), true, t1, 1);
        let rows = plane.rows(t1.as_ns());
        assert_eq!(rows.len(), 1);
        assert_eq!(
            (rows[0].topic.as_str(), rows[0].robot.as_deref()),
            ("/local", None)
        );
    }
}

/// Monitors: the FLOOD LATCHES around the standing conditions.
///
/// The wire cannot storm — the alert ring is bounded at [`MONITOR_ALERT_RING`] and
/// a raise is a one-shot — so what these arms are about is the LOG, which is the
/// only surface an operator watching a robot at 03:00 actually sees.
///
/// Every predicate matches the LEVEL TOKEN as well as the message. A text-only
/// filter passes the headline variant: the suppressed arm emitted at `warn!` keeps
/// every message and every count intact while the suppression itself is entirely
/// ineffective (a text-only filter lets exactly that variant survive a
/// whole suite).
#[cfg(test)]
mod latch_tests {
    use super::*;
    use cerulion_core::transport::liveness::LIVENESS_NO_DATA_MIN_MS;
    use cerulion_core::{LivenessState, TopicLiveness, TopicRateEstimate};
    use cerulion_viz::monitor::{
        MonitorState, MONITOR_CLEAR_SAMPLES, MONITOR_CONFIRM_MIN_SPAN_NS, MONITOR_CONFIRM_SAMPLES,
        MONITOR_LEARN_SAMPLES, MONITOR_SETTLE_NS,
    };
    use tracing_test::traced_test;

    /// The sampler's cadence, as these arms drive it.
    const STEP_NS: u64 = cerulion_viz::monitor::MONITOR_SAMPLE_INTERVAL_NS;

    /// A write stamp at a hand-chosen instant on the tests' own sample clock.
    const fn at(ns: u64) -> WriteStamp {
        WriteStamp::for_test(ns)
    }

    fn sample(topic: &str, at_ns: u64, liveness: TopicLiveness) -> MonitorSample {
        MonitorSample::new(topic, None, at_ns, Some(liveness), true)
    }

    /// A payload that classifies `Streaming` carrying a trustworthy rate.
    ///
    /// Asserted against the substrate's OWN classifier for the reason the sibling
    /// module states: a threshold moving under the fixture would make it INERT
    /// rather than wrong, and an "expected exactly one line" oracle reads a silent
    /// engine as a broken reporter.
    fn streaming(at_ns: u64, mhz: u64) -> TopicLiveness {
        let l = TopicLiveness {
            last_frame_age_ms: Some(0),
            observed_for_ms: LIVENESS_NO_DATA_MIN_MS + at_ns / 1_000_000,
            frames_observed: 100,
            rate_estimate: Some(TopicRateEstimate {
                millihertz: mhz,
                is_floor: false,
            }),
        };
        assert_eq!(l.state(), LivenessState::Streaming, "fixture: {l:?}");
        l
    }

    /// A payload that classifies `Idle` with a DATED age — a topic that streamed
    /// and stopped, serving no rate.
    fn idle_dated(at_ns: u64) -> TopicLiveness {
        let l = TopicLiveness {
            last_frame_age_ms: Some(60_000),
            observed_for_ms: LIVENESS_NO_DATA_MIN_MS + at_ns / 1_000_000,
            frames_observed: 100,
            rate_estimate: None,
        };
        assert_eq!(l.state(), LivenessState::Idle, "fixture: {l:?}");
        l
    }

    /// One row, raised on `conditions`.
    fn raised_row(topic: &str, conditions: Vec<MonitorCondition>) -> MonitorRow {
        MonitorRow {
            topic: topic.to_string(),
            robot: Some("go2".to_string()),
            state: if conditions.is_empty() {
                MonitorState::Healthy
            } else {
                MonitorState::Alerting
            },
            conditions,
            baseline_mhz: Some(20_000),
            observed_mhz: Some(1_000),
            observed_is_floor: Some(false),
            liveness_state: Some(LivenessState::Idle),
            samples: 42,
            last_sample_age_ms: 0,
            ineligible: Vec::new(),
        }
    }

    /// Lines carrying `needle` AND the whole whitespace token `level`.
    ///
    /// Both halves are read as whole tokens rather than substrings: `tracing-test`
    /// renders the SPAN NAME — the test function's own name — into every line, so
    /// a bare `contains("WARN")` can be satisfied by something that is not the
    /// level at all.
    fn count_at(lines: &[&str], level: &str, needle: &str) -> usize {
        lines
            .iter()
            .filter(|l| l.contains(needle) && l.split_whitespace().any(|t| t == level))
            .count()
    }

    const RAISED: &str = "RAISED a condition";
    const STILL: &str = "still raised (warn suppressed)";
    const DECADE: &str = "is STILL raised";
    const CLEARED: &str = "condition CLEARED";

    /// A wedged row is LOUD once, then quiet, then loud again at the DECADE —
    /// bounded by `log10(total)` rather than by the sampling cadence.
    ///
    /// The stimulus is what makes this the right shape: a raise is a ONE-SHOT (the
    /// engine's tracker returns no further transition until it clears), so a latch
    /// fed on transitions would never see a second failure and would be inert.
    /// What a long-running fault really produces is a standing condition nobody is
    /// reminded of, and the operator-facing failure is the OPPOSITE of a flood —
    /// one line at 03:00 and silence for the next six hours. So the latch is fed
    /// once per SAMPLING PASS while the condition is raised.
    ///
    /// Hand oracle for 10 passes: 1 loud head + 8 debug repeats + 1 loud decade
    /// re-announcement carrying `total_failures=10`.
    #[traced_test]
    #[test]
    fn a_standing_condition_is_loud_once_then_quiet_then_loud_again_at_the_decade() {
        let mut plane = MonitorPlane::default();
        let rows = vec![raised_row("/dead", vec![MonitorCondition::Stalled])];
        for _ in 0..10 {
            plane.report_open_regimes(&rows);
        }
        logs_assert(|lines: &[&str]| {
            let head = count_at(lines, "WARN", RAISED);
            let quiet = count_at(lines, "DEBUG", STILL);
            let decade = count_at(lines, "WARN", DECADE);
            if (head, quiet, decade) != (1, 8, 1) {
                return Err(format!(
                    "expected (1 WARN head, 8 DEBUG repeats, 1 WARN decade), got \
                     ({head}, {quiet}, {decade}):\n{lines:#?}"
                ));
            }
            // The re-announcement exists FOR the operator who missed the head, so
            // it must carry the running total and the row's identity.
            let re = lines
                .iter()
                .find(|l| l.contains(DECADE))
                .expect("the decade line");
            for field in ["total_failures=10", "topic=/dead", "condition=\"stalled\""] {
                if !re.split_whitespace().any(|t| t == field) {
                    return Err(format!("the decade line must carry {field}: {re}"));
                }
            }
            Ok(())
        });
    }

    /// A condition that CLEARS reports recovery ONCE, at `INFO`, carrying the
    /// SUPPRESSED count — and then re-arms, so the next regime is loud again.
    ///
    /// Hand oracle: 3 raised passes (1 head + 2 suppressed), then a healthy pass
    /// ⇒ exactly one recovery carrying `suppressed_count=2`; then 1 raised pass ⇒
    /// a SECOND loud head.
    #[traced_test]
    #[test]
    fn a_cleared_condition_reports_recovery_once_and_re_arms_the_loud_head() {
        let mut plane = MonitorPlane::default();
        let raised = vec![raised_row("/dead", vec![MonitorCondition::Stalled])];
        let healthy = vec![raised_row("/dead", Vec::new())];
        for _ in 0..3 {
            plane.report_open_regimes(&raised);
        }
        plane.report_open_regimes(&healthy);
        // A SECOND healthy pass must stay silent: recovery is reported once per
        // regime, not once per pass on a healthy desk.
        plane.report_open_regimes(&healthy);
        plane.report_open_regimes(&raised);

        logs_assert(|lines: &[&str]| {
            let heads = count_at(lines, "WARN", RAISED);
            let recovered = count_at(lines, "INFO", CLEARED);
            if (heads, recovered) != (2, 1) {
                return Err(format!(
                    "expected 2 loud heads (the regime re-armed) and 1 recovery, got \
                     ({heads}, {recovered}):\n{lines:#?}"
                ));
            }
            let line = lines
                .iter()
                .find(|l| l.contains(CLEARED))
                .expect("the recovery line");
            // The SUPPRESSED count, not the total: it is the number of lines the
            // operator did not see.
            if !line.split_whitespace().any(|t| t == "suppressed_count=2") {
                return Err(format!("recovery must report what was missed: {line}"));
            }
            if !line.split_whitespace().any(|t| t == "total_failures=3") {
                return Err(format!("and must NOT reset the running total: {line}"));
            }
            Ok(())
        });
    }

    /// SEPARATENESS: one row's open `stalled` regime must not swallow the loud
    /// head of its own `rate_deviation`.
    ///
    /// The keying rule ("one per observing entity AND condition") applied
    /// here: they are different faults with different remedies, and an operator
    /// who has already been told about the stall still needs to be told the rate
    /// collapsed. Driven in the order that catches a merged latch — the stall
    /// regime is opened FIRST, so with one latch per row the rate's head would
    /// arrive already suppressed.
    #[traced_test]
    #[test]
    fn an_open_regime_does_not_swallow_a_sibling_conditions_loud_head() {
        let mut plane = MonitorPlane::default();
        plane.report_open_regimes(&[raised_row("/tf", vec![MonitorCondition::Stalled])]);
        plane.report_open_regimes(&[raised_row("/tf", vec![MonitorCondition::Stalled])]);
        plane.report_open_regimes(&[raised_row(
            "/tf",
            vec![MonitorCondition::Stalled, MonitorCondition::RateDeviation],
        )]);
        assert_eq!(plane.latches.len(), 2, "one latch per (row, condition)");

        logs_assert(|lines: &[&str]| {
            let heads: Vec<&&str> = lines
                .iter()
                .filter(|l| l.contains(RAISED) && l.split_whitespace().any(|t| t == "WARN"))
                .collect();
            if heads.len() != 2 {
                return Err(format!(
                    "each condition owns its loud head; got {}:\n{lines:#?}",
                    heads.len()
                ));
            }
            let conditions: Vec<&str> = heads
                .iter()
                .filter_map(|l| l.split_whitespace().find(|t| t.starts_with("condition=")))
                .collect();
            if conditions != vec!["condition=\"stalled\"", "condition=\"rate_deviation\""] {
                return Err(format!(
                    "the two heads name their own conditions: {conditions:?}"
                ));
            }
            Ok(())
        });
    }

    /// ANTI-TAUTOLOGY: a healthy desk logs NOTHING and creates NO latch.
    ///
    /// Without this the "exactly N" arms above would pass a reporter that also
    /// fired on rows with nothing raised — and the latch map would grow one entry
    /// per watched row per condition on a robot where nothing is ever wrong.
    #[traced_test]
    #[test]
    fn a_healthy_row_logs_nothing_and_allocates_no_latch() {
        let mut plane = MonitorPlane::default();
        for _ in 0..5 {
            plane.report_open_regimes(&[raised_row("/fine", Vec::new())]);
        }
        assert!(
            plane.latches.is_empty(),
            "latches are created lazily, only for a condition that really raised"
        );
        logs_assert(|lines: &[&str]| {
            let noise: Vec<&&str> = lines
                .iter()
                .filter(|l| {
                    [RAISED, STILL, DECADE, CLEARED]
                        .iter()
                        .any(|n| l.contains(n))
                })
                .collect();
            if noise.is_empty() {
                Ok(())
            } else {
                Err(format!("a healthy row must say nothing: {noise:#?}"))
            }
        });
    }

    /// The loud line for a stall confirmed after a baseline wipe carries the
    /// basis the VERDICT rested on, not the row's absent rate baseline.
    ///
    /// The two are different numbers on exactly this class, and the row's is the
    /// one that is missing: a `rate_deviation` clearing (which is what a dying
    /// publisher's absent rate does) wipes `MonitorRow::baseline_mhz` while the
    /// engine keeps its last FROZEN baseline as the stall basis. A reporter that
    /// reads the row therefore drops the number from the very line an operator has
    /// to judge the verdict from — MEASURED: a row-reading reporter carries no
    /// `baseline_mhz` field at all, because `tracing` omits a `None`.
    ///
    /// Driven through the REAL engine (`observe_pass`), because the whole point is
    /// that the raise and the row DISAGREE, and a hand-built `MonitorRow` cannot
    /// produce that disagreement. Both halves are asserted in the same body: the
    /// served row really does carry no baseline, and the WARN really does carry
    /// one — either alone reads as the other's bug.
    #[traced_test]
    #[test]
    fn a_stall_after_a_wipe_logs_the_basis_the_verdict_rested_on() {
        const TOPIC: &str = "/dying";
        const NOMINAL_MHZ: u64 = 100_000;
        const DEVIATING_MHZ: u64 = NOMINAL_MHZ / 2 - 1;

        let mut plane = MonitorPlane::default();
        // Open the row, then step past the settle window.
        plane.observe_pass(&[sample(TOPIC, 0, streaming(0, NOMINAL_MHZ))], at(0));
        let mut t = MONITOR_SETTLE_NS;
        let confirm =
            (MONITOR_CONFIRM_MIN_SPAN_NS / STEP_NS + 1).max(u64::from(MONITOR_CONFIRM_SAMPLES));

        // Learn a baseline…
        for _ in 0..=u64::from(MONITOR_LEARN_SAMPLES) {
            plane.observe_pass(&[sample(TOPIC, t, streaming(t, NOMINAL_MHZ))], at(t));
            t += STEP_NS;
        }
        // …degrade until `rate_deviation` confirms…
        for _ in 0..confirm {
            plane.observe_pass(&[sample(TOPIC, t, streaming(t, DEVIATING_MHZ))], at(t));
            t += STEP_NS;
        }
        // …then DIE: the rate goes absent, which clears the deviation (wiping the
        // baseline) and then confirms the stall against the retained basis.
        for _ in 0..(confirm + u64::from(MONITOR_CLEAR_SAMPLES)) {
            plane.observe_pass(&[sample(TOPIC, t, idle_dated(t))], at(t));
            t += STEP_NS;
        }

        let row = plane
            .rows(t)
            .into_iter()
            .find(|r| r.topic == TOPIC)
            .expect("the row");
        assert_eq!(
            row.conditions,
            vec![MonitorCondition::Stalled],
            "precondition: the death really confirmed a stall: {row:?}"
        );
        assert_eq!(
            row.baseline_mhz, None,
            "…and the ROW carries no baseline, which is what makes this arm an \
             oracle rather than a coincidence: {row:?}"
        );

        logs_assert(|lines: &[&str]| {
            let head = lines
                .iter()
                .find(|l| {
                    l.contains(RAISED)
                        && l.split_whitespace().any(|t| t == "WARN")
                        && l.split_whitespace().any(|t| t == "condition=\"stalled\"")
                })
                .ok_or_else(|| format!("no loud head for the stall:\n{lines:#?}"))?;
            if !head
                .split_whitespace()
                .any(|t| t == format!("baseline_mhz={NOMINAL_MHZ}"))
            {
                return Err(format!(
                    "the stall's loud line must carry the basis it was judged \
                     against ({NOMINAL_MHZ} mHz), not the row's wiped field: {head}"
                ));
            }
            Ok(())
        });
    }
}
