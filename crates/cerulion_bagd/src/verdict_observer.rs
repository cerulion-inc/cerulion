// SPDX-License-Identifier: AGPL-3.0-only
//! Flashback trigger 3 — the MONITORS-VERDICT OBSERVER, robot-side.
//!
//! Flashback ships three triggers. The manual verb and the
//! process-fault publisher are wired; this is the third. It watches the same
//! per-topic verdicts Studio's sidebar renders and, when one CONFIRMS, asks the
//! recorder it lives inside to keep the moment.
//!
//! # Why this runs on the ROBOT, inside the recorder
//!
//! The monitors engine and its plane run on the DESK today
//! (`cerulion_viz::monitor`, `cerulion_vizd::monitors`), so the obvious shape
//! is a desk-side observer publishing a request. It does not work, for three
//! reasons and one that decides it:
//!
//! * The robot gateway's whole request/response vocabulary is four verbs —
//!   `Catalog`, `Schema`, `Runs`, `Demand` (`QueryVerb`). Three are
//!   read-only GETs; `Demand` carries nothing but a topic name. **There is no
//!   desk→robot command path on the LAN.**
//! * The trigger channel is deliberately barred from the network:
//!   `/__cerulion/flashback` sits under the reserved prefix and is refused
//!   egress at three independent guards, one of which calls serving it "leaking
//!   the control plane onto the network". That is a standing refusal, not a gap.
//! * A WAN command path exists (`WireRequest::SyncEpoch`, the iroh OPS plane's
//!   `restart` / `engage-estop`) but is pairing-gated, so a plain LAN
//!   desk/robot pair has none at all — the trigger would be silently absent on
//!   the commonest deployment.
//!
//! And the deciding one is not plumbing. Flashback is, by design, an
//! **always-on black box**; a trigger that fires only while a desk is attached
//! and looking is not always-on, and the incident nobody was watching is the one
//! a black box exists for.
//!
//! This is not the `rerun_sink` mistake in a new coat. That decision is
//! about VISUALIZATION — putting a renderer on the robot. Nothing here renders,
//! decodes or converts an archetype. The recorder has always been robot-side,
//! and deciding **when to keep a recording** is that process's existing job.
//!
//! # Zero new ports, and why that is structural rather than a promise
//!
//! The recorder already holds a listener-less tap on every live topic and
//! already drains it. What it does NOT have is any notion of age, rate or
//! liveness, and it must not grow one: the liveness observer's
//! stamp-advancement dating rule, its two-guard epoch reset and its
//! sequence-basis rate windows are exactly the subtle logic that drifts when it
//! exists in two copies, and a second implementation of them would disagree
//! with the desk's about whether a robot is healthy.
//!
//! So this reuses the substrate's own record book in the gateway's proven
//! EXTERNAL-OBSERVER mode: every topic is marked
//! [`set_externally_observed`](TopicLivenessObserver::set_externally_observed),
//! which makes the observer drop any tap of its own, and the recorder's existing
//! drain feeds [`note_frames`](TopicLivenessObserver::note_frames). `sweep()` is
//! never called, so no tap is ever attached. The observer becomes a pure record
//! book: no subscriber, no listener, no thread.
//!
//! `queue_emptied` is OBSERVED, never inferred — the one thing that closes the
//! advancement baseline, and a drainer that can never report `true` leaves its
//! topic permanently undatable. The recorder's drain is the SAFE shape for this:
//! it loops to a short read, so the loop exiting on `n == 0` is direct evidence
//! the queue is empty, while an exit on the staging budget is direct evidence it
//! is not. Neither is arithmetic.
//!
//! # The regime, and why "not spammy" is inherited rather than rebuilt
//!
//! Every anti-spam rule already lives in
//! [`FlashbackTriggerGate`](cerulion_core::flashback::trigger::FlashbackTriggerGate)
//! and is already oracle-tested there. This module's whole contribution to it is
//! choosing the SUBJECT — the regime key — and calling `recover` on the clearing
//! edge. With those two right, the behaviour falls out:
//!
//! * a topic stalls ⇒ ONE capture; it stays stalled ⇒ nothing further is asked
//!   for at all;
//! * it recovers ⇒ the regime closes;
//! * it stalls AGAIN ⇒ a new regime, a new capture;
//! * three topics stall together ⇒ the first captures and the rest COALESCE into
//!   that same bag as extra causes. One incident, one bag.
//!
//! **TWO layers do that suppressing, and which one does the work here is worth
//! stating, because the first version of this module got it backwards.** The
//! engine's own raise is a ONE-SHOT — `ConditionTracker` reports a transition and
//! then nothing until the condition CLEARS — so on a steadily-stalled topic no
//! second request is ever constructed and the gate is never asked. MEASURED: a
//! stall driven for six further sampling passes yields exactly one
//! `TriggerDecision` and `TriggerStats::requests == 1`. The gate's per-cause
//! latch is therefore the SECOND line, and what it covers is the shape the
//! engine cannot see — a duplicate observer, or a raise whose clearing edge never
//! reached the gate. Both layers are load-bearing and neither is redundant, but a
//! reader tracing "why did this not spam" will find the answer in the engine, not
//! in the latch. Its practical consequence is that `recover` normally reports
//! `Some(0)`: the regime was genuinely open, and it swallowed nothing because
//! nothing was re-asked.
//!
//! A monitor verdict is the one trigger kind with a recovery EDGE
//! ([`TriggerKind::has_recovery_edge`]), which is why its regime is deliberately
//! not RE-ARMED by the refractory floor: a permanently bad monitor must not
//! capture once a minute forever, and its clearing edge is the real signal that
//! the next occurrence is news.
//!
//! **That is not the same as being exempt from the floor, and the distinction is
//! MEASURED rather than assumed.** The floor is stamped at CAPTURE time and
//! deliberately SURVIVES a recovery (the gate's own flapper rule), so a topic
//! that heals and re-stalls INSIDE
//! [`DEFAULT_FLASHBACK_REFRACTORY_MS`](cerulion_core::flashback::trigger::DEFAULT_FLASHBACK_REFRACTORY_MS)
//! is refused as `Refractory` — with the retry time — rather than captured. That
//! is the behaviour a bag-per-oscillation would otherwise produce, and it is what
//! `a_flapper_re_firing_inside_the_refractory_floor_is_refused_and_told_when_to_retry`
//! pins; its twin waits the floor out and captures. Both sides of that boundary
//! are asserted, because either alone reads as the other's bug.
//!
//! **The re-arm IS immediate, and the spam bound is the floor rather than a
//! learning delay.** This used to say the opposite, and the opposite
//! was a bug: the wipe discards the RATE baseline on any clear, and the stall
//! gate read that same baseline, so a recovered topic was `stalled`-ineligible
//! until it re-learned — which a topic that never recovered (a publisher that
//! DIED, whose absent rate is itself what cleared the deviation) could not do at
//! all. The engine now keeps the LAST FROZEN baseline as its stall basis, so
//! eligibility survives the clear and a death reaching this module is a
//! transition like any other. Nothing here changed to make that true; what
//! changed is that the transition now arrives.
//!
//! What bounds the spam is therefore stated where it actually lives: the
//! engine's raise is a ONE-SHOT (a steadily-stalled topic constructs no second
//! request at all), the gate's refractory floor is stamped at CAPTURE time and
//! deliberately survives a recovery, and the rolling-hour cap is above both. A
//! topic that heals and immediately re-stalls now REACHES the gate and is
//! refused `Refractory` with a retry time — which is a bounded, attributable
//! answer, where the old behaviour was to never ask.
//!
//! # Two clock domains, named
//!
//! Liveness records and engine samples ride the manager's clock — the same one
//! `note_frames` stamps with, so `read_liveness` must be called with it or every
//! age is nonsense. The GATE rides the recorder's own drive-loop origin. Each is
//! used consistently inside its own subsystem and neither is ever compared to the
//! other.
//!
//! # What is deliberately NOT here
//!
//! No new CLI flag, YAML key or env var. `CERULION_FLASHBACK=off` already kills
//! the plane and therefore this trigger; `CERULION_FLASHBACK_MAX_PER_HOUR`
//! already throttles it; `CERULION_TOPIC_LIVENESS=off` already disables the
//! record book, and an operator who turned liveness off getting no verdicts is
//! the same answer the desk gives.

use std::collections::BTreeSet;
use std::sync::Arc;

use cerulion_core::flashback::switch::{TriggerPosture, TriggerSwitch};
use cerulion_core::flashback::trigger::{CaptureRequest, TriggerKind};
use cerulion_core::monitor::{
    Alert, MonitorEngine, MonitorRow, MonitorSample, MONITOR_SAMPLE_INTERVAL_NS,
};
use cerulion_core::transport::liveness::{
    read_liveness, DrainObservation, TopicLiveness, TopicLivenessObserver,
};
use cerulion_core::transport::TransportManager;

/// The trigger kind everything this module mints belongs to.
///
/// Exported so the caller's
/// [`recover`](cerulion_core::flashback::trigger::FlashbackTriggerGate::recover)
/// call cannot name a different one from the requests it is re-arming.
/// `recover(kind, subject)` looks the regime up by BOTH halves, so a mismatched
/// kind fails silently — it finds no regime, returns `None`, and leaves the real
/// one open forever, swallowing every later occurrence.
pub const VERDICT_KIND: TriggerKind = TriggerKind::MonitorVerdict;

/// What one confirmed monitor transition asks the recorder to do.
///
/// A transition is either a RAISE or a CLEAR and the two drive opposite halves of
/// the gate, so they are separate variants rather than one struct with a flag: a
/// caller that forgot to branch would silently turn every recovery into a capture
/// request, which is the spam the gate exists to prevent.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum VerdictAction {
    /// A condition CONFIRMED — ask the gate to capture the moment.
    Capture(CaptureRequest),
    /// A condition CLEARED — re-arm its regime, so a later recurrence is news.
    ///
    /// Carries the subject rather than the whole request because
    /// [`FlashbackTriggerGate::recover`](cerulion_core::flashback::trigger::FlashbackTriggerGate::recover)
    /// takes `(kind, subject)`, and the kind is fixed for everything this module
    /// mints.
    Recover {
        /// The regime key — see [`verdict_subject`].
        subject: String,
    },
}

/// PURE: the regime key one alert belongs to.
///
/// [`CaptureRequest::subject`] states the responsibility this discharges: two
/// requests sharing a subject are "the same condition happening again" and the
/// second is suppressed, so a subject that is too coarse loses captures and one
/// that is too fine floods. A monitor alert is identified by all three of
/// condition, origin and topic — the engine's own example is that "a stall on
/// `/a` and a stall on `/b` are different conditions", and one desk watches many
/// robots, so `/lowstate` on two of them must not share a regime either.
///
/// A LOCAL producer (`robot: None`, the attribution convention) is spelled `local`
/// rather than left empty, so the three fields are always positionally readable
/// in a log line and an empty robot name cannot collide with a missing one.
///
/// Not length-clamped. The channel encoder refuses a subject over
/// [`MAX_FLASHBACK_SUBJECT_LEN`](cerulion_core::flashback::channel::MAX_FLASHBACK_SUBJECT_LEN)
/// (256), but this path never encodes one — it hands the request to the gate
/// in-process. Truncating to fit would MERGE two regimes into one and silently
/// lose a capture, which is strictly worse than a long key; iceoryx2's own name
/// limits keep real topics far below the ceiling, and
/// `a_realistic_worst_case_subject_still_fits_the_channel_encoder` pins that.
pub fn verdict_subject(alert: &Alert) -> String {
    format!(
        "{}:{}:{}",
        alert.condition.as_wire(),
        alert.robot.as_deref().unwrap_or("local"),
        alert.topic
    )
}

/// PURE: the human detail carried into the capture's own record.
///
/// [`CaptureCause`](cerulion_core::flashback::trigger::CaptureCause) keeps the
/// detail of the request that FIRST contributed a cause, and it is what a reader
/// of a finished flashback sees. So it names the numbers the verdict rested on
/// rather than restating the subject: a bag saying only
/// `monitor_verdict / stalled:go2:/lowstate` would tell an operator what fired
/// and nothing about why.
///
/// Every optional field is omitted when absent rather than rendered as a zero —
/// the absent-is-not-zero rule the whole liveness surface is built on. A missing
/// rate means "no trustworthy measurement", and printing `0 mHz` for it would be
/// an affirmatively wrong claim about a topic that may be streaming at 500 Hz
/// behind an untrusted basis.
pub fn verdict_detail(alert: &Alert) -> String {
    let mut detail = format!(
        "monitor verdict {} on {}",
        alert.condition.as_wire(),
        alert.topic
    );
    if let Some(robot) = alert.robot.as_deref() {
        detail.push_str(&format!(" (robot {robot})"));
    }
    if let Some(state) = alert.liveness_state {
        detail.push_str(&format!("; liveness={}", state.as_wire()));
    }
    if let Some(age) = alert.age_ms {
        detail.push_str(&format!("; last frame {age} ms ago"));
    }
    if let Some(observed) = alert.observed_mhz {
        detail.push_str(&format!("; observed={observed} mHz"));
    }
    if let Some(baseline) = alert.baseline_mhz {
        detail.push_str(&format!("; baseline={baseline} mHz"));
    }
    detail
}

/// PURE: the action one alert asks for.
///
/// The discriminator is [`Alert::cleared_at_ms`], which the engine sets on a
/// CLEAR and leaves `None` on a RAISE. Reading it here — rather than having the
/// caller infer intent from a row's state — keeps the raise/clear decision in one
/// place, with the engine's own field as its only input.
pub fn action_for(alert: &Alert) -> VerdictAction {
    let subject = verdict_subject(alert);
    if alert.cleared_at_ms.is_some() {
        VerdictAction::Recover { subject }
    } else {
        VerdictAction::Capture(CaptureRequest::monitor_verdict(
            subject,
            verdict_detail(alert),
        ))
    }
}

/// PURE: [`action_for`], filtered by this robot's per-condition POSTURE.
///
/// # Why the CONDITION's switch is applied here and not at the gate
///
/// All three monitor conditions share [`TriggerKind::MonitorVerdict`], so the gate
/// cannot tell them apart without parsing the subject's first chunk — a string
/// grammar this module owns and must stay free to change. This function holds the
/// `Alert`, so it knows the condition first-hand. See
/// [`switch`](cerulion_core::flashback::switch)'s module docs for the whole split.
///
/// # A RECOVERY is never filtered, and the asymmetry is deliberate
///
/// Dropping a switched-off condition's CAPTURE is the whole of the `silent` demotion. But
/// dropping its RECOVERY would be unsafe in the one case it could matter: the gate
/// has no kind-level switch for `MonitorVerdict`, so a SECOND process publishing a
/// `monitor_verdict` request onto the channel can open a regime this observer never
/// minted — and a regime with no recovery edge delivered stays open for the rest of
/// the run, silently swallowing every later occurrence. A recovery for a regime
/// that does not exist is a no-op that returns `None`; a recovery withheld from one
/// that does is a lost capture. The cheap direction is to deliver it.
pub fn admit_action(alert: &Alert, posture: TriggerPosture) -> Option<VerdictAction> {
    let action = action_for(alert);
    if matches!(action, VerdictAction::Capture(_))
        && !posture.is_on(TriggerSwitch::for_condition(alert.condition))
    {
        return None;
    }
    Some(action)
}

/// The recorder-side watchdog: a liveness record book fed by the recorder's own
/// drains, and the shared monitors engine reading it.
pub struct VerdictObserver {
    /// The manager whose clock stamps the record book. Held so the observer can
    /// read that clock ITSELF — see [`VerdictObserver::now_ns`].
    manager: Arc<TransportManager>,
    /// The substrate's record book, held in EXTERNAL-OBSERVER mode throughout —
    /// see the module docs. `sweep()` is never called on it, which is what makes
    /// the zero-ports claim structural.
    liveness: TopicLivenessObserver,
    /// The SHARED engine (`cerulion_core::monitor`), the same one vizd runs.
    engine: MonitorEngine,
    /// Topics already marked externally observed.
    ///
    /// The mark is idempotent, but it also RE-OPENS the advancement baseline
    /// every time it flips false→true, so calling it per drain would hold the
    /// baseline permanently open and no topic would ever date. Tracking it here
    /// makes the mark exactly once-per-topic.
    marked: BTreeSet<String>,
    /// When the last engine pass ran, on the liveness clock. `None` until the
    /// first, so the first pass is never throttled out.
    last_sample_ns: Option<u64>,
    /// Engine passes run — the "is the sampler alive?" observable (Principle #3).
    passes: u64,
    /// Actions minted, ever. Unconditional; nothing resets it.
    actions: u64,
    /// This robot's per-condition POSTURE (decisions 110-B + 112-G).
    posture: TriggerPosture,
    /// Captures WITHHELD by that posture, ever. Unconditional; nothing resets it.
    ///
    /// Its own counter rather than a share of `actions`, because a robot whose
    /// `silent` rows are firing steadily and being withheld reads, on every other
    /// observable, exactly like one where nothing is happening — and the whole
    /// point of demoting a condition rather than deleting it is that an operator
    /// can find out it would have fired (Principle #3).
    withheld: u64,
}

impl VerdictObserver {
    /// Open a record book over the recorder's own transport manager.
    ///
    /// Constructing the observer attaches nothing: taps are only opened by
    /// `sweep()`, which this module never calls.
    pub fn new(manager: Arc<TransportManager>) -> Self {
        Self::with_posture(manager, TriggerPosture::default())
    }

    /// Open a record book with an explicit per-condition posture.
    ///
    /// The production caller resolves one from the environment ONCE and hands it
    /// here; a test hands one built by hand, which is what keeps the posture arms
    /// free of `set_var` and therefore parallel-safe.
    pub fn with_posture(manager: Arc<TransportManager>, posture: TriggerPosture) -> Self {
        Self {
            liveness: TopicLivenessObserver::new(Arc::clone(&manager)),
            manager,
            engine: MonitorEngine::new(),
            marked: BTreeSet::new(),
            last_sample_ns: None,
            passes: 0,
            actions: 0,
            posture,
            withheld: 0,
        }
    }

    /// Feed ONE tap drain into the record book.
    ///
    /// Called from the recorder's drain, with what that drain already holds — the
    /// wire headers it parsed anyway, the publisher count its port can answer, and
    /// whether the loop ran the queue dry. Nothing here re-reads the data plane.
    ///
    /// A zero-frame drain is SKIPPED rather than reported, on the substrate's own
    /// advice: the interval is already open from the mark, an empty drain
    /// deliberately does not close the baseline, and skipping keeps an idle topic
    /// off the table lock and the clock. A topic that stops publishing still ages
    /// into `Idle` and then `NoData`, because the age is computed at READ time
    /// from the last dated frame.
    pub fn note_drain(&mut self, topic: &str, obs: DrainObservation) {
        if !self.marked.contains(topic) {
            // hot-path-alloc-ok: cold — once per topic, at its first drain.
            self.liveness.set_externally_observed(topic, true);
            self.marked.insert(topic.to_string());
        }
        if obs.frames == 0 {
            return;
        }
        self.liveness.note_frames(topic, obs);
    }

    /// The LIVENESS clock — the one `note_frames` stamps its records with.
    ///
    /// **This is the whole of the clock-domain fix, and it is why `sample` takes
    /// no instant.** `read_liveness` computes every age as
    /// `now_ns - last_frame_at_ns`, and `last_frame_at_ns` is stamped by the
    /// record book from `manager.clock()` — so `now_ns` MUST come from that same
    /// clock or the subtraction compares two unrelated number lines.
    ///
    /// The recorder's own drive-loop origin is `start.elapsed()` (nanoseconds
    /// since the RECORDER PROCESS started) while the manager's `RealClock` is
    /// `real_ns()` (system uptime). On a robot up for six hours whose recorder
    /// started a minute ago the recorder's instant is ~60e9 and the stamp
    /// ~21600e9, so `saturating_sub` yields **0** — every topic reads
    /// `last_frame_age_ms: 0`, classifies `Streaming` forever, and `stalled` can
    /// NEVER fire. The watchdog would be silently inert on exactly the condition
    /// it exists to catch, on every long-uptime robot, with no error anywhere.
    ///
    /// Reading the clock HERE rather than accepting one makes that unrepresentable:
    /// the caller has no instant to get wrong. The GATE keeps the recorder's own
    /// origin, which is correct — it only ever compares recorder instants to each
    /// other — so the two domains stay separate and neither is ever subtracted
    /// from the other.
    fn now_ns(&self) -> u64 {
        self.manager.clock().now_ns()
    }

    /// Whether an engine pass is due, on the liveness clock.
    ///
    /// [`MONITOR_SAMPLE_INTERVAL_NS`] is twice the substrate's own sweep interval
    /// — sampling at or under it would re-read one observation, and doubling
    /// guarantees a fresh one even when the two grids beat. The recorder's drive
    /// loop runs orders of magnitude faster, so this is a clock compare on almost
    /// every pass.
    pub fn due(&self) -> bool {
        self.due_at(self.now_ns())
    }

    /// The throttle predicate over an EXPLICIT instant.
    ///
    /// Private, and it exists only so [`observe_samples`](Self::observe_samples) —
    /// the transport-free seam, whose origin is the caller's own — can share ONE
    /// implementation with the production path instead of restating the interval
    /// arithmetic. Nothing outside this module can reach it, so the API a caller
    /// sees still admits no instant.
    fn due_at(&self, now_ns: u64) -> bool {
        match self.last_sample_ns {
            None => true,
            Some(last) => now_ns.saturating_sub(last) >= MONITOR_SAMPLE_INTERVAL_NS,
        }
    }

    /// Run ONE engine pass over the recorder's tapped topics and return what the
    /// confirmed transitions ask for.
    ///
    /// `topics` is handed in rather than held, so the watched universe is exactly
    /// the recorder's own tap set: a verdict about a topic no tap holds could
    /// never be backed by frames in the resulting bag.
    ///
    /// **Takes NO instant** — see this type's `now_ns` for why the caller must
    /// not be able to supply one.
    ///
    /// `discovery_converged: true` is accurate HERE and would not be on the desk. The
    /// flag exists so an empty or short catalog is never read as "the topic is
    /// dead"; that class cannot arise on this plane, because a row exists only
    /// because the recorder is holding a tap on the topic — which is direct
    /// evidence the topic is real. Same reasoning, same answer, as the monitors
    /// ATTACHED plane, whose evidence is likewise first-hand.
    ///
    /// The due check is repeated here purely to avoid READING the liveness table
    /// on a pass whose result would be discarded;
    /// [`observe_samples`](Self::observe_samples) remains the authority.
    pub fn sample<'a, I>(&mut self, topics: I) -> Vec<VerdictAction>
    where
        I: IntoIterator<Item = &'a str>,
    {
        // ONE clock read for the whole pass: the throttle, the liveness ages and
        // the engine's sample stamps must all be dated from the SAME instant, or a
        // row's age and the span its confirmation is measured over disagree.
        let now_ns = self.now_ns();
        if !self.due_at(now_ns) {
            return Vec::new();
        }
        let table = self.liveness.table();
        let samples: Vec<MonitorSample> = topics
            .into_iter()
            .map(|topic| {
                let liveness = read_liveness(&table, topic, now_ns);
                // `robot: None` — every topic a recorder taps is on its OWN
                // machine. A mirrored remote topic is re-injected into local SHM
                // by netd and is a local producer from here; attributing it to a
                // robot would need the provenance registry, and a wrong
                // attribution would split one regime in two.
                MonitorSample::new(topic, None, now_ns, liveness, true)
            })
            .collect();
        self.observe_samples(&samples, now_ns)
    }

    /// Run ONE engine pass over samples the caller already holds.
    ///
    /// **This is the drivable seam** — the whole of this module's behaviour above
    /// the liveness record book is a pure function of the samples handed in, so a
    /// test builds a `MonitorSample` vector by hand and gets the real engine, the
    /// real confirmation windows and the real action mapping with no transport
    /// and no waiting. Without it the only way to reach a verdict would be to
    /// publish through real iceoryx2 and then wait out
    /// `MONITOR_CONFIRM_MIN_SPAN_NS` (1.2 s) for EVERY arm, turning the
    /// adversarial cases — a flapper at the refractory boundary, a rate cap, two
    /// interleaved regimes — into minute-long load-sensitive tests of what is
    /// pure arithmetic.
    ///
    /// **The throttle is enforced HERE**, not at the call site, on the rule the
    /// monitors plane already states: a cadence a caller could forget silently
    /// becomes "whatever the drive loop runs at", and the engine confirms on a
    /// COUNT of samples as well as a span — so a caller sampling far too fast
    /// would confirm a condition inside the window a lull-free restart is
    /// still healing in, which is a false `stalled` on a topic that is fine. An
    /// un-due pass returns empty and advances nothing.
    pub fn observe_samples(
        &mut self,
        samples: &[MonitorSample],
        now_ns: u64,
    ) -> Vec<VerdictAction> {
        if !self.due_at(now_ns) {
            return Vec::new();
        }
        self.last_sample_ns = Some(now_ns);
        self.passes += 1;
        let mut actions = Vec::new();
        for sample in samples {
            for alert in self.engine.observe(sample) {
                match admit_action(&alert, self.posture) {
                    Some(action) => actions.push(action),
                    // WITHHELD by this robot's posture. Counted, never silent —
                    // see the field.
                    None => self.withheld += 1,
                }
            }
        }
        self.actions += actions.len() as u64;
        actions
    }

    /// The posture in force.
    pub fn posture(&self) -> TriggerPosture {
        self.posture
    }

    /// Captures withheld by the posture since this observer opened. Never reset.
    pub fn withheld(&self) -> u64 {
        self.withheld
    }

    /// Engine passes run since this observer opened.
    pub fn passes(&self) -> u64 {
        self.passes
    }

    /// Actions minted since this observer opened. Never reset.
    pub fn actions(&self) -> u64 {
        self.actions
    }

    /// Rows the engine is watching.
    pub fn watched(&self) -> usize {
        self.engine.watched()
    }

    /// The engine's own view of every row it watches (Principle #3).
    ///
    /// Exists because the counted observables ([`watched`](Self::watched),
    /// [`passes`](Self::passes)) answer only "did a pass run?" — they are
    /// satisfied by a sampler that handed the engine a vector of BLIND samples,
    /// which is the one regression that would make the whole watchdog inert while
    /// every counter climbed. A row carries what was actually CONSUMED
    /// (`liveness_state`, `observed_mhz`, `samples`), so an oracle on those is an
    /// oracle on the observation really reaching the engine.
    pub fn rows(&self) -> Vec<MonitorRow> {
        self.engine.rows(self.now_ns())
    }

    /// What the record book currently says about one topic (Principle #3).
    ///
    /// The observation is otherwise reachable only through a verdict, which by
    /// design takes a settle window plus a confirmation span to appear — so
    /// without this there is no way to tell "the feed is working and the topic is
    /// healthy" from "the feed is dead", which are the two states a silent
    /// watchdog is in. `None` means UNKNOWN, never dead.
    pub fn liveness(&self, topic: &str) -> Option<TopicLiveness> {
        read_liveness(&self.liveness.table(), topic, self.now_ns())
    }

    /// Whether the record book is enabled at all (`CERULION_TOPIC_LIVENESS`).
    ///
    /// A disabled book records nothing, so every sample is blind and no verdict can
    /// ever fire. Exposed so the recorder can say so ONCE at startup rather than
    /// leaving an operator to wonder why a stalled topic never captured.
    pub fn is_enabled(&self) -> bool {
        self.liveness.is_enabled()
    }
}

impl std::fmt::Debug for VerdictObserver {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("VerdictObserver")
            .field("watched", &self.engine.watched())
            .field("marked", &self.marked.len())
            .field("passes", &self.passes)
            .field("actions", &self.actions)
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::flashback::channel::MAX_FLASHBACK_SUBJECT_LEN;
    use cerulion_core::monitor::MonitorCondition;
    use cerulion_core::LivenessState;

    /// A RAISED alert, hand-built — the engine's own shape, minted here so these
    /// arms are oracles rather than a re-run of the engine's tests.
    fn raised(topic: &str, robot: Option<&str>, condition: MonitorCondition) -> Alert {
        Alert {
            seq: 1,
            topic: topic.to_string(),
            robot: robot.map(str::to_string),
            condition,
            raised_at_ms: 1_000,
            cleared_at_ms: None,
            age_ms: Some(7_500),
            observed_mhz: None,
            baseline_mhz: Some(100_000),
            liveness_state: Some(LivenessState::Idle),
        }
    }

    #[test]
    fn the_subject_separates_condition_origin_and_topic() {
        // All three fields are load-bearing: change ONE and the regime must differ,
        // or a stall on one topic would suppress a stall on another.
        let a = verdict_subject(&raised("/lowstate", Some("go2"), MonitorCondition::Stalled));
        let b = verdict_subject(&raised("/odom", Some("go2"), MonitorCondition::Stalled));
        let c = verdict_subject(&raised(
            "/lowstate",
            Some("spot"),
            MonitorCondition::Stalled,
        ));
        let d = verdict_subject(&raised("/lowstate", Some("go2"), MonitorCondition::Silent));
        assert_eq!(a, "stalled:go2:/lowstate");
        assert_eq!(b, "stalled:go2:/odom");
        assert_eq!(c, "stalled:spot:/lowstate");
        assert_eq!(d, "silent:go2:/lowstate");
        let all = BTreeSet::from([a, b, c, d]);
        assert_eq!(all.len(), 4, "every field must key its own regime");
    }

    #[test]
    fn a_local_producer_is_spelled_local_and_cannot_collide_with_a_named_robot() {
        let local = verdict_subject(&raised("/scan", None, MonitorCondition::Stalled));
        assert_eq!(local, "stalled:local:/scan");
        // The one collision that would matter: a robot genuinely CALLED "local"
        // still lands on the same key. Recorded rather than defended — this plane
        // always passes `robot: None`, so the named arm is unreachable here, and
        // defending it would mean an escape syntax nothing can currently produce.
        let named = verdict_subject(&raised("/scan", Some("local"), MonitorCondition::Stalled));
        assert_eq!(named, local);
    }

    #[test]
    fn a_raise_asks_for_a_capture_and_a_clear_asks_for_a_recover() {
        let mut alert = raised("/lowstate", Some("go2"), MonitorCondition::Stalled);
        match action_for(&alert) {
            VerdictAction::Capture(req) => {
                assert_eq!(req.kind, TriggerKind::MonitorVerdict);
                assert_eq!(req.subject, "stalled:go2:/lowstate");
                assert!(!req.pin, "a verdict capture is not pinned by default");
            }
            other => panic!("a raise must ask for a capture, got {other:?}"),
        }
        alert.cleared_at_ms = Some(9_000);
        assert_eq!(
            action_for(&alert),
            VerdictAction::Recover {
                subject: "stalled:go2:/lowstate".to_string()
            },
            "a clear must re-arm the regime, never capture again"
        );
    }

    #[test]
    fn the_raise_and_its_clear_agree_on_the_subject() {
        // The whole recovery edge rests on this: `recover(kind, subject)` finds the
        // regime by key, so a clear that spelled its subject differently would
        // leave the regime open forever and the next occurrence would be silently
        // swallowed.
        let mut alert = raised("/imu/data", None, MonitorCondition::RateDeviation);
        let VerdictAction::Capture(req) = action_for(&alert) else {
            panic!("expected a capture");
        };
        alert.cleared_at_ms = Some(9_000);
        let VerdictAction::Recover { subject } = action_for(&alert) else {
            panic!("expected a recover");
        };
        assert_eq!(req.subject, subject);
    }

    #[test]
    fn the_detail_names_the_numbers_the_verdict_rested_on() {
        let alert = raised("/lowstate", Some("go2"), MonitorCondition::Stalled);
        let detail = verdict_detail(&alert);
        assert!(detail.contains("stalled"), "{detail}");
        assert!(detail.contains("/lowstate"), "{detail}");
        assert!(detail.contains("robot go2"), "{detail}");
        assert!(detail.contains("idle"), "{detail}");
        assert!(detail.contains("7500 ms ago"), "{detail}");
        assert!(detail.contains("baseline=100000 mHz"), "{detail}");
        // ABSENT is not zero: this alert carries no observed rate, and rendering
        // one as `0 mHz` would be an affirmatively wrong claim about a topic that
        // may be streaming behind an untrusted basis.
        assert!(
            !detail.contains("observed="),
            "an absent rate must be omitted, not rendered as zero: {detail}"
        );
    }

    #[test]
    fn the_detail_omits_every_field_the_alert_does_not_carry() {
        let bare = Alert {
            seq: 2,
            topic: "/scan".to_string(),
            robot: None,
            condition: MonitorCondition::Silent,
            raised_at_ms: 5,
            cleared_at_ms: None,
            age_ms: None,
            observed_mhz: None,
            baseline_mhz: None,
            liveness_state: None,
        };
        let detail = verdict_detail(&bare);
        assert_eq!(detail, "monitor verdict silent on /scan");
    }

    #[test]
    fn a_realistic_worst_case_subject_still_fits_the_channel_encoder() {
        // `verdict_subject` does not clamp, because clamping would MERGE regimes.
        // This is the check that says the un-clamped key is nonetheless safe on the
        // shapes that can actually occur: a long ROS-style topic on a long robot
        // name under the longest condition word.
        let alert = raised(
            "/sensors/front_left/depth_camera/points_registered_filtered",
            Some("go2-warehouse-north-dock-07"),
            MonitorCondition::RateDeviation,
        );
        let subject = verdict_subject(&alert);
        assert!(
            subject.len() <= MAX_FLASHBACK_SUBJECT_LEN,
            "a realistic worst case must fit the channel encoder's {MAX_FLASHBACK_SUBJECT_LEN} \
             bytes, in case this ever routes over it: {} bytes",
            subject.len()
        );
    }

    // ---------------------------------------------------------------------
    // The `silent` DEMOTION, applied at the
    // MINT SITE (see `admit_action` for why not at the gate).
    // ---------------------------------------------------------------------

    /// A CLEARED alert, hand-built — `action_for`'s discriminator is
    /// `cleared_at_ms`, so a recovery differs from a raise in exactly that field.
    fn cleared(topic: &str, robot: Option<&str>, condition: MonitorCondition) -> Alert {
        Alert {
            cleared_at_ms: Some(9_000),
            ..raised(topic, robot, condition)
        }
    }

    /// THE DEMOTION: under the shipped default posture a `silent` verdict mints
    /// NOTHING, while its two siblings still do.
    ///
    /// All three conditions are driven in ONE body because either half alone
    /// reads as the other's bug: "silent is withheld" without the siblings is
    /// indistinguishable from an observer that mints nothing at all, and
    /// "stalled still captures" without silent is indistinguishable from no
    /// demotion having shipped.
    #[test]
    fn the_default_posture_withholds_silent_and_keeps_its_two_siblings() {
        let posture = TriggerPosture::default();
        for condition in [MonitorCondition::Stalled, MonitorCondition::RateDeviation] {
            let alert = raised("/lowstate", Some("go2"), condition);
            match admit_action(&alert, posture) {
                Some(VerdictAction::Capture(req)) => {
                    assert_eq!(req.kind, TriggerKind::MonitorVerdict, "{condition:?}");
                    assert_eq!(req.subject, verdict_subject(&alert), "{condition:?}");
                }
                other => panic!("{condition:?} must still capture, got {other:?}"),
            }
        }
        assert_eq!(
            admit_action(
                &raised("/lowstate", Some("go2"), MonitorCondition::Silent),
                posture
            ),
            None,
            "project rule: `silent` is demoted to available-OFF"
        );
    }

    /// The demotion is a DEMOTION, not a deletion — a robot for which
    /// never-produced routes ARE the interesting ones can opt back in.
    #[test]
    fn a_robot_that_opts_silent_back_in_gets_its_captures() {
        let posture = TriggerPosture::default().with(TriggerSwitch::Silent, true);
        let alert = raised("/uslam/cloud_map", Some("go2"), MonitorCondition::Silent);
        match admit_action(&alert, posture) {
            Some(VerdictAction::Capture(req)) => {
                assert_eq!(req.subject, "silent:go2:/uslam/cloud_map")
            }
            other => panic!("an opted-in silent verdict must capture, got {other:?}"),
        }
        // …and the switch really is per-condition: turning `silent` ON does not
        // turn a switched-off sibling on with it.
        let one_off = posture.with(TriggerSwitch::Stalled, false);
        assert_eq!(
            admit_action(&raised("/a", None, MonitorCondition::Stalled), one_off),
            None
        );
        assert!(admit_action(&raised("/a", None, MonitorCondition::Silent), one_off).is_some());
    }

    /// A RECOVERY is NEVER withheld, whatever the posture says.
    ///
    /// The asymmetry is the point and it is a safety argument, not a symmetry
    /// one: the gate has no kind-level switch for `MonitorVerdict`, so a SECOND
    /// process publishing a `monitor_verdict` request onto the channel can open a
    /// regime this observer never minted — and a regime with no recovery
    /// delivered stays open for the rest of the run, silently swallowing every
    /// later occurrence. A recovery for a regime that does not exist is a no-op
    /// returning `None`; a recovery withheld from one that does is a lost
    /// capture.
    #[test]
    fn a_recovery_is_delivered_even_for_a_condition_whose_captures_are_withheld() {
        let posture = TriggerPosture::default();
        let alert = cleared("/lowstate", Some("go2"), MonitorCondition::Silent);
        match admit_action(&alert, posture) {
            Some(VerdictAction::Recover { subject }) => {
                assert_eq!(subject, "silent:go2:/lowstate")
            }
            other => panic!("a recovery must never be withheld, got {other:?}"),
        }
        // …and with EVERY switch off, so the answer cannot be an accident of the
        // defaults.
        let all_off = TriggerSwitch::ALL
            .into_iter()
            .fold(posture, |p, s| p.with(s, false));
        for condition in MonitorCondition::ALL {
            assert!(
                matches!(
                    admit_action(&cleared("/a", None, condition), all_off),
                    Some(VerdictAction::Recover { .. })
                ),
                "{condition:?} recovery must be delivered"
            );
        }
    }

    /// Every monitor condition has a switch, and the mapping is the documented
    /// one.
    ///
    /// A hand table over `MonitorCondition::ALL` rather than a sample: the way a
    /// condition ships un-postured is by being ADDED, which a per-variant list
    /// cannot see (it passes by not mentioning the new one).
    #[test]
    fn every_monitor_condition_maps_to_the_switch_that_governs_it() {
        let oracle: &[(MonitorCondition, TriggerSwitch)] = &[
            (MonitorCondition::Stalled, TriggerSwitch::Stalled),
            (MonitorCondition::Silent, TriggerSwitch::Silent),
            (
                MonitorCondition::RateDeviation,
                TriggerSwitch::RateDeviation,
            ),
        ];
        assert_eq!(
            oracle.len(),
            MonitorCondition::ALL.len(),
            "a monitor condition was added without a posture switch"
        );
        for (condition, switch) in oracle {
            assert_eq!(TriggerSwitch::for_condition(*condition), *switch);
        }
    }
}
