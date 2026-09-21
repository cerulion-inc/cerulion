// SPDX-License-Identifier: AGPL-3.0-only
//! The **rule→action seam** — what turns "something happened" into
//! "write a flashback", and what stops a robot writing ten thousand of them.
//!
//! Everything here is PURE: it takes a request and a clock reading and returns a
//! decision. No transport, no filesystem, no UI, and no clock of its own — so the
//! whole policy is oracle-testable against hand-written vectors, and the same
//! engine serves all three of v1's triggers without any of them learning about
//! the others.
//!
//! # Why ONE seam for three triggers
//!
//! Flashback v1 ships a manual CLI verb, a process-fault auto-trigger and a
//! monitors-verdict auto-trigger. They differ ONLY in who observes; what they
//! produce is one thing — a request to capture the moment. Giving each its own
//! policy would mean three copies of the anti-spam rules, which is the
//! two-copies class with a disk-filling failure mode attached. So the observers
//! construct a [`CaptureRequest`] and this gate decides, and the monitors-UI work
//! reuses it by building a cause rather than by learning what a capture is.
//!
//! # Why this is NOT a [`FailureRegimeLatch`](crate::transport::failure_regime_latch)
//!
//! That machine is the repo's shared flood suppressor and the temptation to
//! newtype it here is real — the not-spammy requirement names its philosophy.
//! But its output is a LOG-VOLUME decision (`Loud` / `Suppressed` /
//! `StillFailing`), and its most distinctive feature is the DECADE LADDER: an open
//! regime re-announces itself at 10, 100, 1000 failures, because the rmw counters
//! sit behind a standardized C ABI and the log is a ROS user's only window.
//!
//! Transplanted here that reads: write the 10th bag, the 100th bag, the 1000th
//! bag. A line is cheap and a bag is ~155 MB — so the one behaviour that machine
//! exists to guarantee is the exact behaviour this one exists to prevent. What IS
//! carried over is its philosophy and its two hard-won rules, both visible below:
//! an UNCONDITIONAL counter that recovery never resets (Principle #3), and a
//! re-arm on recovery.
//!
//! # The four mechanisms, and the order they are asked in
//!
//! | mechanism | rule |
//! |---|---|
//! | overlap coalescing | a request landing inside an active capture EXTENDS it — ONE bag, causes recorded as a list |
//! | per-cause regime latch | the FIRST request of a cause-regime captures; repeats while it is open are COUNTED, never captured |
//! | refractory floor | a cause with no natural recovery edge re-arms after [`DEFAULT_FLASHBACK_REFRACTORY_MS`] |
//! | global rate cap | at most [`DEFAULT_FLASHBACK_MAX_PER_HOUR`] captures per rolling hour; the next is loudly suppressed AND counted |
//!
//! Coalescing is asked FIRST, and that is not arrangement — it is correctness. A
//! burst is ONE moment (three nodes dying together is one incident), so a member
//! of a burst must join the capture rather than be suppressed by it: suppressing
//! it would drop its cause from the bag's own list, and would leave that cause's
//! regime CLOSED, so the same fault would capture again the moment the bag
//! finished. Extending records the cause and opens its regime, which is what makes
//! the burst settle.
//!
//! The three suppression arms are then ordered most-specific-first, on the same
//! rule [`arm_verdict`](super::arm_verdict) states for its two: report the one the
//! operator can ACT on. "This condition is still open" names a condition; "you are
//! over the hourly cap" names a budget and says nothing about what is wrong.
//!
//! # What is deliberately not a trigger
//!
//! Declared QoS deadlines — `#[input(expect_within_ms = N)]` and
//! `#[output(promise_within_ms = N)]` — are REJECTED as capture triggers for now,
//! and the reason is this module's own [`TriggerKind::has_recovery_edge`] split.
//! The flashback channel carries REQUESTS only; regime recovery is gate-internal,
//! so a graph-process observer can never deliver a clearing edge. That leaves a
//! deadline observer with two possible postures and both are wrong: as a
//! no-recovery kind it re-captures a persistently-late input every
//! [`DEFAULT_FLASHBACK_REFRACTORY_MS`] until the rolling-hour cap is spent, and as
//! a recovery kind its regime stays open for the rest of the run and silently
//! swallows the next genuine occurrence. Misses also accrue per SILENCE WINDOW
//! rather than per arrival, so a mis-declared budget on a 1 kHz input is a
//! sustained regime rather than an event.
//!
//! Revisit through the monitors plane — a new learned verdict kind, which keeps the
//! one rule→action seam — or after the channel grows a recovery verb. A fourth
//! trigger wire would be the two-detectors class.

use std::collections::{BTreeMap, VecDeque};

use super::switch::{TriggerPosture, TriggerSwitch};

/// The default refractory floor for a cause that has no recovery edge, in
/// milliseconds.
///
/// A monitors verdict CLEARS — the engine reports a healthy row and the regime
/// re-arms on evidence. A process fault does not: nothing ever says "the node
/// that panicked is fine now", so without a floor its regime would stay open for
/// the rest of the run and a second, genuinely new panic would go uncaptured.
///
/// 60 s is the chosen number. Against the ~155 MB-class capture it is also a
/// disk-rate statement: one cause can cost at most a capture a minute, which the
/// retention caps then bound absolutely.
pub const DEFAULT_FLASHBACK_REFRACTORY_MS: u64 = 60_000;

/// The default global rate cap: captures per rolling hour, across ALL causes.
///
/// The BACKSTOP, deliberately loose enough that it is not the mechanism doing the
/// work — the latch and the refractory floor are. It exists for the shape neither
/// of those can see: many DIFFERENT causes firing once each, which is a robot
/// coming apart rather than one condition repeating, and is exactly when an
/// operator least wants their disk churned.
pub const DEFAULT_FLASHBACK_MAX_PER_HOUR: u32 = 20;

/// Slots of [`DEFAULT_FLASHBACK_MAX_PER_HOUR`] the AUTOMATIC kinds may not spend
/// (the manual reserve).
///
/// # The lockout this closes
///
/// The cap originally gave the manual verb and the automatic triggers one undifferentiated
/// budget, which has a failure mode the automatic triggers cannot see: a robot
/// coming apart mints capture-moments, and the operator watching it come apart
/// reaches for `cerulion flashback` to capture the one moment they actually care
/// about — and is refused, by the flood the event itself is producing. The verb
/// is locked out by exactly the condition it exists for.
///
/// So the last two slots of every rolling hour belong to Manual. What that buys is
/// narrow and stated: the operator gets TWO captures, not an exemption. A scripted
/// manual flood still hits the full cap and is still refused loudly, which is
/// the loud refusal preserved rather than weakened.
///
/// Two, not one: the first manual capture of an incident is routinely the wrong
/// moment (an operator captures, watches, and captures again once they know what
/// they are looking at), and a reserve of one makes that second attempt the
/// refusal.
pub const FLASHBACK_MANUAL_RESERVE: u32 = 2;

/// Override [`DEFAULT_FLASHBACK_MAX_PER_HOUR`].
///
/// The knob every rate-cap refusal NAMES. A refusal that quotes a number an
/// operator cannot change is only half a message — the per-verb remedy rule
/// applied to an env var rather than a flag.
pub const FLASHBACK_MAX_PER_HOUR_ENV: &str = "CERULION_FLASHBACK_MAX_PER_HOUR";

/// PURE: the rolling-hour cap in force, from [`FLASHBACK_MAX_PER_HOUR_ENV`].
///
/// Resolved through the SAME shared parser as every other Flashback knob, so a
/// zero is refused with the default quoted rather than silently meaning "capture
/// nothing" — which the gate WOULD honour literally, and which an operator
/// typing `0` almost never means (they mean the kill switch).
pub fn resolve_max_per_hour(raw: Option<&str>) -> (u32, Option<String>) {
    let (value, complaint) =
        super::resolve_positive_override(raw, &DEFAULT_FLASHBACK_MAX_PER_HOUR, "captures per hour");
    let cap = value
        .map(|v| u32::try_from(v).unwrap_or(u32::MAX))
        .unwrap_or(DEFAULT_FLASHBACK_MAX_PER_HOUR);
    (cap, complaint)
}

/// How long a capture keeps recording after its trigger, in milliseconds.
///
/// The v1 `T+15s` post window. The pre half is not here because it is not a policy this
/// gate can enforce: it is however much the rolling window happens to be holding
/// when the trigger lands (see [`super::DEFAULT_FLASHBACK_CADENCE_MS`] for the
/// arithmetic that sizes it).
pub const DEFAULT_FLASHBACK_POST_WINDOW_MS: u64 = 15_000;

/// The longest a single capture may be extended by coalescing, in milliseconds.
///
/// Coalescing is unbounded by nature — every new cause pushes the end out — so a
/// robot faulting steadily would write ONE bag that never finalizes, which is the
/// same outcome as writing none and is harder to notice. At the ceiling the
/// capture finalizes on schedule and the next request starts a new one, so a
/// sustained fault becomes a SERIES of bounded bags that the rate cap and the
/// retention caps then bound.
///
/// 120 s = the post window (15 s) plus enough room for a genuine multi-cause
/// incident to play out, and eight times the shipped anchor cadence, so a capture
/// at the ceiling still spans several anchors.
pub const DEFAULT_FLASHBACK_MAX_CAPTURE_SPAN_MS: u64 = 120_000;

/// The most DISTINCT causes one capture will record.
///
/// Coalescing dedupes by `(kind, subject)`, which bounds the list by the number
/// of distinct CONDITIONS rather than by the number of requests — and that is not
/// a bound at all on the shape this exists for: a monitors adapter keys on the
/// row, so a robot whose LAN goes down raises one cause per topic and a Go2-scale
/// graph has ~100 of them. The capture is one bag; its cause list is a summary a
/// human reads.
///
/// Reaching it never suppresses the request — the capture still EXTENDS — so the
/// cost of the bound is a name in a list, and it is REPORTED
/// ([`CauseRecord::Dropped`], [`FinishedCapture::causes_dropped`]) rather than
/// silently absorbed.
pub const DEFAULT_FLASHBACK_MAX_CAUSES: usize = 32;

/// The most DROPPED cause identities one capture remembers.
///
/// Recognising a repeat of an omitted cause needs its identity kept, and a set
/// of identities is exactly the unbounded growth
/// [`DEFAULT_FLASHBACK_MAX_CAUSES`] exists to prevent — a robot whose LAN drops
/// raises one distinct cause per topic, and nothing stops an adversarial or
/// merely unlucky subject space from being larger still. So the memory is capped
/// too, and past the cap the distinct-omitted count may overstate again
/// ([`FinishedCapture::causes_dropped_exact`] says when it might).
///
/// 128 = 4x the cause cap: generous enough that the Go2-scale shape it is sized
/// for (~100 topics) is counted exactly, small enough that the worst case is a
/// bounded list of short strings. A capture reaching it is already one whose
/// summary is dominated by the count rather than the names.
pub const DEFAULT_FLASHBACK_MAX_DROPPED_TRACKED: usize = 128;

/// Milliseconds in the rate cap's rolling window.
const RATE_WINDOW_MS: u64 = 60 * 60 * 1000;

/// How far BEFORE the caller's zero this gate's internal clock line begins.
///
/// # The seed cannot land on a line that starts at the caller's zero
///
/// [`FlashbackTriggerGate::seed_from_history`] places a capture written 10 s ago
/// at `now_ns - 10s`, and the recorder's own monotonic origin is its process
/// start — so at construction `now_ns` is ~0 and every seeded instant saturates
/// to zero. The floor then reads `elapsed == 0` for a cause that captured an hour
/// ago, and the rolling window expires an hour after THIS PROCESS started rather
/// than an hour after the captures were written. That direction is safe (it can
/// only withhold budget) but it is not what the seed claims to do, and a
/// mechanism whose docs overstate it is one nobody can reason about later.
///
/// So the gate's INTERNAL line is the caller's, shifted forward by one rolling
/// window: every instant a caller supplies has this added before use, and every
/// instant handed back has it removed, so the external contract is byte-identical
/// while the whole of the seedable past is representable.
///
/// One window exactly, because that is the entire history the gate can act on —
/// a capture older than the window seeds nothing at all (see `seed_from_history`),
/// so there is nothing further back to represent.
const GATE_EPOCH_NS: u64 = RATE_WINDOW_MS * NS_PER_MS;

/// One millisecond in nanoseconds — this module's clock unit is NANOSECONDS
/// (matching every other monotonic reading in the codebase) while its knobs are
/// stated in MILLISECONDS (matching what an operator types).
const NS_PER_MS: u64 = 1_000_000;

/// WHO observed the thing that wants capturing.
///
/// Deliberately a closed set of variants and not one string: the KIND decides
/// whether the anti-spam rules apply at all (see [`TriggerKind::is_automatic`]),
/// and a reader of a finished bag must be able to tell "an operator asked for
/// this" from "the robot decided on its own" without parsing prose.
///
/// # Adding a variant is ADDITIVE on the wire, and the direction matters
///
/// [`channel`](super::channel) encodes the kind as ONE byte and answers an
/// unknown one with `FlashbackRecordError::UnknownKind`, so a NEW producer talking
/// to an OLD recorder is REFUSED loudly at decode rather than silently mis-read as
/// some other kind — the frame carries no other field a wrong guess could be
/// caught by. An OLD producer talking to a NEW recorder is unaffected. Both halves
/// ship in one process image on a robot today; the refusal is what makes a mixed
/// deployment loud rather than wrong.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum TriggerKind {
    /// `cerulion flashback` — an operator or an agent asked, explicitly.
    Manual,
    /// A worker PROCESS died — the supervisor observed a child exit it did not
    /// ask for.
    ///
    /// Deliberately NOT "a node panicked": a contained panic leaves the process
    /// alive and is [`PanicDisable`](Self::PanicDisable)'s business. The two were
    /// one variant until the later kinds were added, and the merge was a documented
    /// lie — the kind's own doc named a node panic while no code path fired it.
    ProcessFault,
    /// A monitors verdict fired — `stalled`, `rate_deviation` or `silent`.
    ///
    /// The CONDITION rides the subject's first chunk
    /// (`{condition}:{robot|local}:{topic}`), not the kind: three conditions
    /// sharing one detector, one record book and one recovery edge are one KIND
    /// with three regimes, and splitting them into wire variants would put the
    /// monitors engine's own vocabulary into a frame format that has to outlive
    /// it. Per-condition POSTURE is therefore a
    /// [`TriggerSwitch`] decided at the mint site —
    /// see that module for the split.
    MonitorVerdict,
    /// A node was DISABLED after `MAX_CONSECUTIVE_PANICS` consecutive panics, or
    /// its entry mutex was POISONED by a panic — the process is alive and one
    /// node has stopped.
    ///
    /// Fires once per DEATH TRANSITION, not once per node per run. Both mint
    /// sites latch on the transition rather than on the state (see
    /// [`NodeDeathLedger`](crate::scheduler::NodeDeathLedger)), and a node can
    /// die twice: `Scheduler::reset_node` re-enables a disabled node and clears
    /// its panic run, so three further panics disable it again — a second, real
    /// death, pinned by
    /// `scheduler_test::a_reset_and_three_more_panics_is_a_second_node_death`.
    /// (This doc previously claimed "at most once per node per run" on the
    /// grounds that the scheduler "never re-enables"; `reset_node` is exactly
    /// that. What bounds the repeats instead is TWO things at different scales:
    /// the ledger's per-`(node, cause)` dedup within ONE drain window — `take`
    /// clears `deaths`, and the dedup memory lives there — and the mint sites'
    /// own latches across windows.)
    ///
    /// It is an automatic kind under the floor, because a graph that disables a
    /// dozen nodes in a minute is one incident and should be one bag.
    PanicDisable,
    /// The RUN this recorder is bound to VANISHED — a monolith hard crash
    /// (SIGSEGV / OOM / abort) that announced nothing.
    ///
    /// The recorder's own run watcher is the observer, so this is the one kind
    /// minted by the process that also decides on it.
    RunVanished,
    /// A human engaged the E-STOP over the Cerulion ops plane.
    ///
    /// The highest specificity per capture in the whole trigger space: somebody
    /// declared an incident. Scope is narrow — it sees a
    /// `cerud`-mediated e-stop and nothing else, so a hardwired Cat-0 stop (motor
    /// power cut, compute alive) announces nowhere in the middleware and fires
    /// nothing here.
    EStop,
    /// A robot-side process DECLARED an incident on `/__cerulion/flashback`.
    ///
    /// This is by design: the tier-2 physical-event contract is the channel
    /// that is already open, not a payload-inference plane inside the recorder.
    /// A bumper, a safety PLC, a domain e-stop topic or a grasp-failure detector
    /// publishes one of these when ITS signal fires; see
    /// [`CaptureRequest::declared`].
    ///
    /// Its OWN kind rather than riding [`Manual`](Self::Manual) or
    /// [`ProcessFault`](Self::ProcessFault), for two reasons that pull the same
    /// way. A reader of a finished bag must be able to tell "the robot's own
    /// safety logic fired" from "an operator pressed the verb" and from "the
    /// framework faulted" — three different next steps. And riding `Manual` would
    /// inherit its latch and floor EXEMPTIONS, which are justified by a human
    /// watching; a detector wired to a bumper at 10 Hz is exactly what the floor
    /// exists for.
    Declared,
}

impl TriggerKind {
    /// PURE: does the anti-spam machinery apply to this kind?
    ///
    /// # Manual is exempt from the LATCH and the FLOOR, and that is the whole
    /// distinction
    ///
    /// The requirement is that the automatic triggers must not be spammy. A
    /// manual request is a deliberate act by somebody who is watching, so latching
    /// it would make the second `cerulion flashback` in a minute silently do
    /// nothing — a verb that lies about having run, which is the class this repo
    /// refuses.
    ///
    /// It is NOT exempt from the rate cap: that arm is the disk backstop, it is
    /// reported loudly rather than silently, and it is counted. An operator who
    /// hits it is told the number and the knob.
    /// # Every kind added after the first three is AUTOMATIC, including the two
    /// a human is behind
    ///
    /// [`EStop`](Self::EStop) and [`Declared`](Self::Declared) both originate in a
    /// human decision, so the temptation to give them Manual's exemptions is real
    /// and it is wrong. The exemption is justified by a human WATCHING THIS
    /// RECORDER — somebody who typed `cerulion flashback` and is standing by to
    /// read the answer — not by a human being upstream somewhere. An e-stop
    /// arrives over the ops plane from a possibly-remote operator, and a declared
    /// trigger arrives from a detector that human wired up months ago; neither is
    /// watching the verdict, and both can repeat at a rate no person could
    /// produce (an e-stop re-engaged by an automated safety loop, a bumper
    /// bouncing).
    pub fn is_automatic(self) -> bool {
        match self {
            Self::Manual => false,
            Self::ProcessFault
            | Self::MonitorVerdict
            | Self::PanicDisable
            | Self::RunVanished
            | Self::EStop
            | Self::Declared => true,
        }
    }

    /// PURE: will anything ever tell this gate the condition CLEARED?
    ///
    /// # The distinction the refractory floor turns on
    ///
    /// A monitors verdict has a recovery EDGE: the monitors engine reports the
    /// row healthy again and the observer calls [`FlashbackTriggerGate::recover`].
    /// A process fault has none — nothing ever says "the node that panicked is
    /// fine now", nothing says "the worker that died is back" — so its regime,
    /// once open, is open for the rest of the run unless something else re-arms
    /// it.
    ///
    /// That is why the floor is a RE-ARM here and not merely a rate limit: a
    /// SECOND, genuinely new panic an hour later is news, and a regime with no
    /// recovery edge would have swallowed it silently. A cause that CAN recover
    /// is deliberately NOT floor-re-armed — a monitor condition that stays bad
    /// would then capture once a minute forever, which is exactly the spam the
    /// not-spammy requirement forbids, and its recovery edge is the reliable signal
    /// that the next occurrence is new.
    ///
    /// [`Manual`](Self::Manual) answers `false`, and the answer is UNUSED: a
    /// manual request is never latched (see [`Self::is_automatic`]), so nothing
    /// consults this for it. `false` is nonetheless the correct reading — nothing
    /// will ever report that an operator's request "recovered".
    ///
    /// # `false` is the SAFE default for a new kind, and that is why every kind
    /// added after the first three answers it
    ///
    /// The two mistakes are not symmetric. A kind wrongly marked `false` re-arms
    /// on the floor, so its worst case is one redundant capture per
    /// [`DEFAULT_FLASHBACK_REFRACTORY_MS`] — bounded, visible, and further bounded
    /// by the rolling-hour cap. A kind wrongly marked `true` waits for a recovery
    /// edge nobody publishes, so its regime stays open for the rest of the run and
    /// every later occurrence is SILENTLY swallowed. `MonitorVerdict` is `true`
    /// only because `verdict_observer` really does drive [`FlashbackTriggerGate::recover`] on the
    /// engine's clear transition; nothing else in the tree drives one.
    ///
    /// Per kind: a disabled node is never re-enabled, a vanished run never comes
    /// back, and nothing publishes "the e-stop was released" or "the bumper is
    /// clear" onto a channel that carries requests only.
    pub fn has_recovery_edge(self) -> bool {
        match self {
            Self::MonitorVerdict => true,
            Self::ProcessFault
            | Self::Manual
            | Self::PanicDisable
            | Self::RunVanished
            | Self::EStop
            | Self::Declared => false,
        }
    }

    /// The word this kind uses in logs, in the bag's own coverage, and on the
    /// wire — ONE spelling, so a `tracing` field and a JSON value cannot drift
    /// (the two-copies rule, applied to a vocabulary rather than a function).
    pub fn as_wire(self) -> &'static str {
        match self {
            Self::Manual => "manual",
            Self::ProcessFault => "process_fault",
            Self::MonitorVerdict => "monitor_verdict",
            Self::PanicDisable => "panic_disable",
            Self::RunVanished => "run_vanished",
            Self::EStop => "estop",
            Self::Declared => "declared",
        }
    }

    /// Every kind, in wire-byte order.
    ///
    /// Exists so a test can walk the vocabulary rather than restate it: a hand
    /// list is exactly what lets a NEW variant ship with no wire byte, no switch
    /// and no spam posture, which is the whole class this constant closes.
    pub const ALL: [TriggerKind; 7] = [
        Self::Manual,
        Self::ProcessFault,
        Self::MonitorVerdict,
        Self::PanicDisable,
        Self::RunVanished,
        Self::EStop,
        Self::Declared,
    ];
}

/// A request to capture the moment.
///
/// `subject` is the REGIME KEY, and choosing it is the caller's one real
/// responsibility: two requests with the same `(kind, subject)` are "the same
/// condition happening again" and the second is suppressed. So a monitors adapter
/// keys on `(condition, topic, robot)` — a stall on `/a` and a stall on `/b` are
/// different conditions — and a process-fault adapter keys on the node or the
/// rank. A subject that is too coarse loses captures; one that is too fine floods.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CaptureRequest {
    /// Who observed.
    pub kind: TriggerKind,
    /// The stable identity of the condition — see the type docs.
    pub subject: String,
    /// Human-readable detail, carried into the capture's own record. Never used
    /// as an identity: two spellings of one condition must not become two regimes.
    pub detail: String,
    /// Ask for this capture to be excluded from retention eviction.
    pub pin: bool,
}

impl CaptureRequest {
    /// A manual request, the shape `cerulion flashback` builds.
    pub fn manual(detail: impl Into<String>) -> Self {
        Self {
            kind: TriggerKind::Manual,
            // Manual requests share ONE regime key because the kind is exempt
            // from the latch anyway — giving each its own key would suggest a
            // per-request regime that nothing ever consults.
            subject: String::new(),
            detail: detail.into(),
            pin: false,
        }
    }

    /// A process-fault request keyed on what faulted.
    pub fn process_fault(subject: impl Into<String>, detail: impl Into<String>) -> Self {
        Self {
            kind: TriggerKind::ProcessFault,
            subject: subject.into(),
            detail: detail.into(),
            pin: false,
        }
    }

    /// A monitors-verdict request keyed on the condition AND the row it fired on.
    pub fn monitor_verdict(subject: impl Into<String>, detail: impl Into<String>) -> Self {
        Self {
            kind: TriggerKind::MonitorVerdict,
            subject: subject.into(),
            detail: detail.into(),
            pin: false,
        }
    }

    /// A node-disable request keyed on the node that was disabled.
    pub fn panic_disable(subject: impl Into<String>, detail: impl Into<String>) -> Self {
        Self {
            kind: TriggerKind::PanicDisable,
            subject: subject.into(),
            detail: detail.into(),
            pin: false,
        }
    }

    /// A vanished-run request keyed on the run that vanished.
    pub fn run_vanished(subject: impl Into<String>, detail: impl Into<String>) -> Self {
        Self {
            kind: TriggerKind::RunVanished,
            subject: subject.into(),
            detail: detail.into(),
            pin: false,
        }
    }

    /// An e-stop request.
    ///
    /// One subject for the whole robot, deliberately: an e-stop is a machine-wide
    /// safety event and there is nothing finer for it to be keyed on. That also
    /// means two engagements inside the floor coalesce or are refused rather than
    /// writing two bags of one incident.
    pub fn estop(detail: impl Into<String>) -> Self {
        Self {
            kind: TriggerKind::EStop,
            subject: "estop".to_string(),
            detail: detail.into(),
            pin: false,
        }
    }

    /// A declared request — the tier-2 contract.
    ///
    /// # The one thing a caller has to get right
    ///
    /// `subject` is the REGIME KEY, so it names the CONDITION and not the moment:
    /// `bumper:front` rather than `bumper:front:1712...`. A subject carrying a
    /// timestamp, a counter or a UUID gives every occurrence its own regime, which
    /// defeats the latch and the floor outright and turns a bouncing detector into
    /// a bag a minute until the rolling-hour cap stops it — see
    /// [`CaptureRequest::subject`].
    pub fn declared(subject: impl Into<String>, detail: impl Into<String>) -> Self {
        Self {
            kind: TriggerKind::Declared,
            subject: subject.into(),
            detail: detail.into(),
            pin: false,
        }
    }

    /// Ask for the resulting capture to be pinned.
    #[must_use]
    pub fn pinned(mut self) -> Self {
        self.pin = true;
        self
    }

    /// The regime this request belongs to.
    fn regime_key(&self) -> RegimeKey {
        (self.kind, self.subject.clone())
    }
}

/// `(kind, subject)` — see [`CaptureRequest::subject`].
type RegimeKey = (TriggerKind, String);

/// ONE cause a capture is recording — the identity AND the human detail.
///
/// The DETAIL is what a reader of a finished flashback sees, and carrying it
/// here is the whole point: [`CaptureRequest::detail`] promises to reach the
/// capture's record, and a gate that kept only the regime key
/// (`(kind, subject)`) silently discarded it — from the request that STARTED the
/// capture as well as from every coalesced one — so a bag could say
/// `process_fault / rank:1` and never "worker exited with SIGSEGV".
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CaptureCause {
    /// Who observed.
    pub kind: TriggerKind,
    /// The regime key's subject — the stable identity of the condition.
    pub subject: String,
    /// The human detail from the request that first contributed this cause.
    ///
    /// FIRST, not last: a repeat carries no new information about the event
    /// (it is the same condition, still true), and overwriting would make the
    /// record depend on how many times a monitor happened to re-evaluate before
    /// the capture closed.
    pub detail: String,
}

/// What a coalesced request did to the capture it joined.
///
/// A named outcome rather than a `bool`, because there are THREE answers and the
/// third is the one an operator needs: a request can be recorded, be a repeat of
/// a cause already recorded, or be DROPPED because the capture is already
/// carrying [`TriggerPolicy::max_causes`]. A `bool` collapses the last two, so a
/// bag that silently stopped recording causes reads exactly like one whose
/// causes all repeated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CauseRecord {
    /// A cause the capture was not yet carrying. Recorded.
    Added,
    /// The capture has already SEEN this `(kind, subject)` — either recorded in
    /// its cause list, or omitted from it because the list was full. Not new
    /// information about the event either way.
    ///
    /// The two are deliberately ONE answer: what a reader needs is "this is the
    /// same condition again", and splitting it would put the cause list's
    /// capacity — an internal bound — into the vocabulary every observer reads.
    Repeated,
    /// The cause list is full — see [`TriggerPolicy::max_causes`]. The request
    /// still EXTENDED the capture in time; only its cause was dropped.
    Dropped,
}

/// Why a request did not become a capture.
///
/// Each variant carries the number the operator needs, because a suppression they
/// cannot quantify is indistinguishable from the feature being broken.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SuppressReason {
    /// This cause's regime is already open — the condition never cleared.
    RegimeOpen {
        /// How many requests of this regime have now been suppressed, INCLUDING
        /// this one. Unconditional; recovery reports it and does not reset the
        /// lifetime total.
        suppressed: u64,
    },
    /// This cause captured recently and has no recovery edge, so it is inside its
    /// refractory floor.
    Refractory {
        /// Nanoseconds until this cause may capture again.
        retry_in_ns: u64,
    },
    /// The global rolling-hour cap is spent.
    RateCapped {
        /// Captures inside the rolling window.
        captures_in_window: u32,
        /// The cap in force FOR THIS KIND — the number that actually bound, which
        /// for an automatic kind is the policy cap minus
        /// [`TriggerPolicy::manual_reserve`].
        ///
        /// Reporting the EFFECTIVE cap rather than the policy one is the whole
        /// point of carrying a number: an automatic trigger refused at 18 of 20
        /// would otherwise render as "18 captures in the window, cap 20", which
        /// reads as a bug in the gate rather than as the reserve working.
        cap: u32,
        /// How many of the policy cap are held back for Manual, so a refusal can
        /// say WHY its cap is lower than the knob. Zero for a manual request and
        /// whenever the reserve is not in force.
        reserved_for_manual: u32,
    },
    /// This kind's posture switch is OFF — see [`TriggerSwitch`].
    ///
    /// Its own arm rather than a silent drop, because a disabled trigger and a
    /// broken one are indistinguishable from the outside otherwise: an operator
    /// who turned `CERULION_FLASHBACK_ON_ESTOP=off` months ago and forgot needs to
    /// be told which switch is answering, not left to conclude the hook never
    /// shipped.
    Disabled {
        /// The switch that refused.
        ///
        /// The typed switch rather than its spelling, so the refusal and the
        /// variable an operator has to change cannot drift apart — the
        /// one-spelling rule, with [`TriggerSwitch::env_suffix`] as the one place
        /// the string lives.
        switch: TriggerSwitch,
    },
}

/// What the gate decided.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TriggerDecision {
    /// Start a capture. The caller writes a bag covering everything the window
    /// holds, plus the post window.
    Capture {
        /// Monotonic across this gate's life, over captures only.
        seq: u64,
        /// When this capture stops recording, unless a later request extends it.
        ends_at_ns: u64,
        /// Whether the capture is excluded from retention eviction.
        pinned: bool,
    },
    /// A capture is already open and this request joined it.
    Extend {
        /// The capture that absorbed it.
        seq: u64,
        /// Its new end, after the extension (possibly unchanged — see
        /// [`DEFAULT_FLASHBACK_MAX_CAPTURE_SPAN_MS`]).
        ends_at_ns: u64,
        /// What happened to this request's CAUSE — see [`CauseRecord`].
        cause: CauseRecord,
    },
    /// Nothing was captured, and why.
    Suppressed(SuppressReason),
}

/// The knobs, resolved once by the caller and then fixed for the process's life.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TriggerPolicy {
    /// See [`DEFAULT_FLASHBACK_POST_WINDOW_MS`].
    pub post_window_ns: u64,
    /// See [`DEFAULT_FLASHBACK_REFRACTORY_MS`].
    pub refractory_ns: u64,
    /// See [`DEFAULT_FLASHBACK_MAX_PER_HOUR`]. Zero means "capture nothing", which
    /// is a legitimate way to leave the plane running while writing no bags.
    pub max_per_hour: u32,
    /// See [`DEFAULT_FLASHBACK_MAX_CAPTURE_SPAN_MS`].
    pub max_capture_span_ns: u64,
    /// See [`DEFAULT_FLASHBACK_MAX_CAUSES`].
    pub max_causes: usize,
    /// See [`DEFAULT_FLASHBACK_MAX_DROPPED_TRACKED`].
    pub max_dropped_tracked: usize,
    /// See [`FLASHBACK_MANUAL_RESERVE`].
    pub manual_reserve: u32,
}

impl Default for TriggerPolicy {
    fn default() -> Self {
        Self {
            post_window_ns: DEFAULT_FLASHBACK_POST_WINDOW_MS * NS_PER_MS,
            refractory_ns: DEFAULT_FLASHBACK_REFRACTORY_MS * NS_PER_MS,
            max_per_hour: DEFAULT_FLASHBACK_MAX_PER_HOUR,
            max_capture_span_ns: DEFAULT_FLASHBACK_MAX_CAPTURE_SPAN_MS * NS_PER_MS,
            max_causes: DEFAULT_FLASHBACK_MAX_CAUSES,
            max_dropped_tracked: DEFAULT_FLASHBACK_MAX_DROPPED_TRACKED,
            manual_reserve: FLASHBACK_MANUAL_RESERVE,
        }
    }
}

impl TriggerPolicy {
    /// PURE: the rolling-hour cap in force for `kind`, and how much of the policy
    /// cap was held back to get there.
    ///
    /// # The reserve is CLAMPED so it can never disable the triggers it protects
    ///
    /// A reserve subtracted flat would make `max_per_hour = 2` mean "no automatic
    /// captures at all, ever" — an operator tightening the disk budget silently
    /// switching the whole watchdog off, which is the inversion this codebase
    /// refuses. So the reserve is capped at `max_per_hour - 1`: at a cap of 1 it is
    /// zero (the two share the single slot, first come), at 2 it is one, and from 3
    /// upward it is the full [`FLASHBACK_MANUAL_RESERVE`].
    ///
    /// The automatic budget is therefore ALWAYS at least one slot whenever the
    /// policy admits any capture at all. A `max_per_hour` of ZERO is honoured
    /// literally on both sides — it means "capture nothing", and reserving a slot
    /// out of nothing would invent one.
    pub fn effective_cap(&self, kind: TriggerKind) -> (u32, u32) {
        if !kind.is_automatic() {
            return (self.max_per_hour, 0);
        }
        let reserve = self.manual_reserve.min(self.max_per_hour.saturating_sub(1));
        (self.max_per_hour - reserve, reserve)
    }
}

/// The unconditional counters (Principle #3).
///
/// Every one of these is a LIFETIME total that no recovery, re-arm or capture ever
/// resets — the rule `FailureRegimeLatch` learned and the reason its `total_failures`
/// survives recovery. A reader asking "how much has this robot been capturing?" gets
/// an answer that does not depend on when they asked.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct TriggerStats {
    /// Every request the gate was asked to decide.
    pub requests: u64,
    /// Requests that STARTED a capture.
    pub captures: u64,
    /// Requests that joined an open capture.
    pub coalesced: u64,
    /// Requests suppressed, for any reason.
    pub suppressed: u64,
    /// Requests suppressed specifically by the rate cap — kept apart because it is
    /// the one arm that says nothing about the robot's health, and netting it into
    /// the total would make a churning disk read like a faulting robot.
    pub rate_capped: u64,
    /// Requests refused because their trigger's posture switch is OFF.
    ///
    /// Kept apart for the same reason, one step further: this arm says nothing
    /// about the robot AND nothing about the disk — it reports a CONFIGURATION.
    /// A robot whose e-stop hook was switched off a year ago would otherwise read,
    /// on every suppression counter an operator can see, exactly like one whose
    /// gate is throttling a real flood.
    pub disabled: u64,
}

/// One cause's regime.
#[derive(Debug, Clone, Copy, Default)]
struct Regime {
    /// Open = this cause captured and has not recovered.
    open: bool,
    /// Requests suppressed while open, lifetime.
    suppressed: u64,
    /// When this cause last captured, for the refractory floor.
    last_capture_ns: Option<u64>,
}

/// A capture that has stopped recording — what the caller needs to name the bag
/// and to record what it was about.
///
/// A named struct rather than a tuple because its three members are two `u64`-ish
/// scalars and a list: a `(u64, bool, Vec<..>)` return reads identically whichever
/// order the fields are in, and a swap at a call site would compile.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FinishedCapture {
    /// The capture's sequence number, monotonic across this gate's life.
    pub seq: u64,
    /// Whether it is excluded from retention eviction.
    pub pinned: bool,
    /// Every cause it recorded, WITH its detail, in arrival order.
    pub causes: Vec<CaptureCause>,
    /// How many further DISTINCT causes reached this capture after its list was
    /// full (see [`TriggerPolicy::max_causes`]). Zero on every ordinary capture.
    ///
    /// Carried rather than inferred: a full list and a list that happens to hold
    /// exactly `max_causes` causes are different facts, and only this one tells a
    /// reader the summary is incomplete.
    ///
    /// DISTINCT is the claim, and it holds while
    /// [`Self::causes_dropped_exact`] does — see there.
    pub causes_dropped: u32,
    /// Whether [`Self::causes_dropped`] really is a count of DISTINCT causes.
    ///
    /// `true` on every capture that did not exhaust
    /// [`TriggerPolicy::max_dropped_tracked`], which is every capture any shipping
    /// shape produces. `false` means the identity memory filled, so a cause
    /// dropped and later repeated could not be recognised and may be counted more
    /// than once — the count becomes an UPPER BOUND on the distinct total.
    ///
    /// Reported rather than silently tolerated, on the same rule as
    /// [`RetentionPlan::pinned_over_cap`](super::retention::RetentionPlan::pinned_over_cap):
    /// a bound that has been reached is a fact about the answer, and a reader who
    /// cannot see it has no way to know the number weakened.
    pub causes_dropped_exact: bool,
}

/// ONE capture already on disk, as [`FlashbackTriggerGate::seed_from_history`]
/// reads it.
///
/// Deliberately NOT [`super::retention::CaptureEntry`]: that type is the
/// retention policy's input and carries a size and a pin, neither of which the
/// gate has any business reading, while the gate needs the FULL cause list and
/// retention needs only the primary class. One type serving both would make each
/// side's fields look meaningful to the other.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CaptureHistory {
    /// When it was captured, on the DURABLE wall-derived line — the same reading
    /// [`CaptureEntry::created_ns`](super::retention::CaptureEntry::created_ns)
    /// carries, and NEVER a monotonic one.
    pub created_wall_ns: u64,
    /// Every cause it recorded, as `(kind, subject)` regime keys.
    ///
    /// ALL of them, not just the primary: each cause that reached a capture had
    /// its regime opened and its floor stamped, so restoring only the first would
    /// leave every coalesced cause free to re-capture immediately after a
    /// relaunch — which is the flood, one layer down.
    pub causes: Vec<(TriggerKind, String)>,
}

/// The capture currently recording.
#[derive(Debug, Clone)]
struct ActiveCapture {
    seq: u64,
    started_ns: u64,
    ends_at_ns: u64,
    pinned: bool,
    causes: Vec<CaptureCause>,
    causes_dropped: u32,
    /// Identities of causes this capture DROPPED, so their repeats are
    /// recognised rather than re-counted. Bounded by
    /// [`TriggerPolicy::max_dropped_tracked`].
    dropped: Vec<RegimeKey>,
    /// The bound above was reached, so `causes_dropped` may now overstate.
    dropped_identities_saturated: bool,
}

/// The gate. See the module docs.
#[derive(Debug)]
pub struct FlashbackTriggerGate {
    policy: TriggerPolicy,
    posture: TriggerPosture,
    regimes: BTreeMap<RegimeKey, Regime>,
    active: Option<ActiveCapture>,
    /// Capture instants inside the rolling hour, oldest first. Bounded by
    /// `max_per_hour` because it is pruned before every read.
    recent: VecDeque<u64>,
    next_seq: u64,
    stats: TriggerStats,
}

impl FlashbackTriggerGate {
    /// A gate under `policy`, with every trigger at its documented default
    /// posture.
    pub fn new(policy: TriggerPolicy) -> Self {
        Self::with_posture(policy, TriggerPosture::default())
    }

    /// A gate under `policy` and an explicit per-trigger `posture`.
    ///
    /// The posture is fixed at construction for the same reason the policy is: a
    /// suppression that depended on when it was asked would be unexplainable from
    /// the environment a reader can inspect afterwards.
    pub fn with_posture(policy: TriggerPolicy, posture: TriggerPosture) -> Self {
        Self {
            policy,
            posture,
            regimes: BTreeMap::new(),
            active: None,
            recent: VecDeque::new(),
            next_seq: 0,
            stats: TriggerStats::default(),
        }
    }

    /// SEED the gate's rate-and-floor state from captures already on disk
    /// (gate state that survives a recorder restart).
    ///
    /// # The flood the shipped gate could not see
    ///
    /// Every anti-spam mechanism here lives in ONE process's memory, and a
    /// recorder is per-run. So a robot that relaunches its graph every ninety
    /// seconds — a systemd restart loop, the shape a fault produces — mints a
    /// FRESH rolling-hour budget on every launch: the cap never engages at all,
    /// the refractory floor re-arms on every boot, and the retention directory
    /// rotates in minutes while the one capture worth keeping ages out. The
    /// time-domain machinery is complete and per-process, which on that shape is
    /// the same as absent.
    ///
    /// # What is restored, and what deliberately is NOT
    ///
    /// Restored: the ROLLING-HOUR window (so the cap is a property of the ROBOT's
    /// last hour rather than this process's) and each cause's REFRACTORY stamp (so
    /// a fault that captured 10 s before the relaunch is still inside its floor).
    /// Both are RATE statements — claims about what already happened, which a
    /// directory listing is direct evidence of.
    ///
    /// NOT restored: the regime `open` bit. That is a claim that a condition is
    /// STILL FAILING, and a process that has just started has observed nothing —
    /// asserting it would suppress the first genuine occurrence after every
    /// restart, silently, which is strictly worse than the flood this closes.
    /// Restoring it is also unnecessary: the floor covers exactly the window in
    /// which a repeat would be noise.
    ///
    /// # Two clocks, and the mapping between them is the whole of the arithmetic
    ///
    /// A capture's `created_ns` is WALL-derived (it must outlive a reboot — see
    /// [`CaptureEntry::created_ns`](super::retention::CaptureEntry::created_ns))
    /// while this gate is MONOTONIC-only. Handing a wall stamp straight in would
    /// compare two unrelated number lines: the cross-clock defect, in a module
    /// that documents its clock discipline. So the caller supplies BOTH
    /// readings of NOW, and each capture's age is computed on the wall line and
    /// then subtracted from the monotonic one.
    ///
    /// Every subtraction saturates, and the direction matters: a capture stamped
    /// in the FUTURE (a clock stepped backwards since it was written) has age zero
    /// and lands at `now_ns`, i.e. is treated as maximally RECENT. That is the
    /// conservative reading — it can only withhold budget, never invent it — and
    /// the alternative would hand back rate-cap slots nobody earned.
    ///
    /// Captures older than the rolling window contribute nothing at all, so a
    /// directory of ancient bags seeds an empty gate.
    ///
    /// # Idempotence is the caller's business, not this function's
    ///
    /// Seeding twice would double-count the window, so this is documented as a
    /// CONSTRUCTION-time call. It is not defended against here, because a gate
    /// that silently ignored a second seed would hide a caller that lost track of
    /// which gate it was holding.
    pub fn seed_from_history(&mut self, history: &[CaptureHistory], now_ns: u64, now_wall_ns: u64) {
        let horizon = RATE_WINDOW_MS * NS_PER_MS;
        // The INTERNAL line — see `GATE_EPOCH_NS`. Without the shift every seeded
        // instant saturates to zero at construction time, which is exactly when
        // this is called.
        let now_ns = Self::internal(now_ns);
        // Sorted so the rolling window's deque stays oldest-first, which
        // `prune_rate_window` relies on to stop at the first live entry.
        let mut seeds: Vec<(u64, &CaptureHistory)> = history
            .iter()
            .map(|h| {
                let age = now_wall_ns.saturating_sub(h.created_wall_ns);
                (now_ns.saturating_sub(age), h)
            })
            .collect();
        seeds.sort_by_key(|(mono, _)| *mono);

        for (mono, entry) in seeds {
            // A capture the window can no longer see spends no budget — but its
            // refractory stamp is equally stale, so the whole entry is skipped.
            if now_ns.saturating_sub(mono) >= horizon {
                continue;
            }
            self.recent.push_back(mono);
            for (kind, subject) in &entry.causes {
                let regime = self.regimes.entry((*kind, subject.clone())).or_default();
                // The LATEST capture of a cause is the one the floor is measured
                // from, and the seeds are walked oldest-first, so a plain
                // assignment lands on the newest. `max` rather than a bare write
                // so a caller handing an unsorted history cannot invert it.
                regime.last_capture_ns =
                    Some(regime.last_capture_ns.map_or(mono, |prev| prev.max(mono)));
            }
        }
    }

    /// The lifetime counters.
    pub fn stats(&self) -> TriggerStats {
        self.stats
    }

    /// The policy in force.
    pub fn policy(&self) -> TriggerPolicy {
        self.policy
    }

    /// The per-trigger posture in force.
    pub fn posture(&self) -> TriggerPosture {
        self.posture
    }

    /// The capture currently recording, if any: `(seq, ends_at_ns)`.
    pub fn active_capture(&self) -> Option<(u64, u64)> {
        self.active
            .as_ref()
            .map(|a| (a.seq, Self::external(a.ends_at_ns)))
    }

    /// The causes the active capture is recording, in the order they arrived.
    pub fn active_causes(&self) -> Vec<CaptureCause> {
        self.active
            .as_ref()
            .map(|a| a.causes.clone())
            .unwrap_or_default()
    }

    /// PURE: decide what to do about `request` at `now_ns`.
    ///
    /// The ORDER of the arms is load-bearing — see the module docs.
    pub fn decide(&mut self, request: &CaptureRequest, now_ns: u64) -> TriggerDecision {
        let now_ns = Self::internal(now_ns);
        self.stats.requests += 1;
        let key = request.regime_key();

        // (0) POSTURE. Asked BEFORE coalescing, and that ordering is the same
        // correctness argument the coalescing/suppression order rests on, run the
        // other way: a switched-off trigger must not EXTEND an open capture
        // either, because extending would record its cause in the bag and open its
        // regime — so a trigger an operator turned off would still appear in the
        // evidence and would still latch, which is neither of the two states the
        // switch admits.
        //
        // Only KIND-level switches are asked here; a monitor verdict's condition
        // switch is applied at the mint site. See `switch`'s module docs for the
        // split and `TriggerSwitch::for_kind` for why `None` means ALLOW.
        if let Some(switch) = self.posture.refusing_switch(request.kind) {
            self.stats.suppressed += 1;
            self.stats.disabled += 1;
            return TriggerDecision::Suppressed(SuppressReason::Disabled { switch });
        }

        // (1) COALESCE. A burst is one moment, so a request landing inside an open
        // capture joins it rather than being judged. This precedes every
        // suppression arm deliberately: see the module docs.
        if let Some(active) = self.active.as_mut() {
            if now_ns < active.ends_at_ns {
                let already_recorded = active
                    .causes
                    .iter()
                    .any(|c| c.kind == key.0 && c.subject == key.1);
                // A cause the capture already DROPPED is still a cause it has
                // SEEN. Without this, a previously-dropped condition repeating
                // was counted again on every repeat, so `causes_dropped` reported
                // OCCURRENCES while documenting DISTINCT omitted causes — and a
                // single condition repeating at frame rate could report hundreds.
                let already_dropped = active.dropped.iter().any(|k| k.0 == key.0 && k.1 == key.1);
                let cause = if already_recorded || already_dropped {
                    CauseRecord::Repeated
                } else if active.causes.len() >= self.policy.max_causes {
                    // The list is FULL. The request still extends the capture —
                    // suppressing a burst member would be the ordering defect the
                    // module docs describe — but its cause is reported dropped
                    // rather than silently absorbed.
                    active.causes_dropped = active.causes_dropped.saturating_add(1);
                    // Remember the IDENTITY so its repeats are recognised — under
                    // its own bound, or this set is the unbounded growth the cause
                    // cap exists to prevent, one layer down. Past the bound the
                    // count can overstate again, and the capture SAYS SO
                    // (`causes_dropped_exact`) rather than quietly reverting to
                    // the behaviour this fix removed.
                    if active.dropped.len() < self.policy.max_dropped_tracked {
                        active.dropped.push(key.clone());
                    } else {
                        active.dropped_identities_saturated = true;
                    }
                    CauseRecord::Dropped
                } else {
                    active.causes.push(CaptureCause {
                        kind: key.0,
                        subject: key.1.clone(),
                        detail: request.detail.clone(),
                    });
                    CauseRecord::Added
                };
                // The capture keeps going for a fresh post window from THIS
                // request — Live Photos: the moment lasts as long as things keep
                // happening — but never past the span ceiling, or a steadily
                // faulting robot writes one bag that never finalizes.
                let ceiling = active
                    .started_ns
                    .saturating_add(self.policy.max_capture_span_ns);
                let wanted = now_ns.saturating_add(self.policy.post_window_ns);
                active.ends_at_ns = active.ends_at_ns.max(wanted.min(ceiling));
                if request.pin {
                    active.pinned = true;
                }
                // A cause that reached a capture has FIRED, so its regime opens
                // here exactly as it would have on a capture of its own.
                // Otherwise the burst's members would all still be un-latched when
                // the bag finalized, and each would immediately capture again.
                let regime = self.regimes.entry(key).or_default();
                regime.open = true;
                regime.last_capture_ns = Some(now_ns);
                self.stats.coalesced += 1;
                return TriggerDecision::Extend {
                    seq: active.seq,
                    ends_at_ns: Self::external(active.ends_at_ns),
                    cause,
                };
            }
        }

        // (2) The per-cause LATCH, and (3) the refractory floor. Both are
        // AUTOMATIC-only: a manual request is a deliberate act and must never be
        // silently swallowed (see `TriggerKind::is_automatic`).
        if request.kind.is_automatic() {
            let policy = self.policy;
            let has_recovery_edge = request.kind.has_recovery_edge();
            let regime = self.regimes.entry(key.clone()).or_default();
            if regime.open {
                // THE FLOOR RE-ARMS A REGIME NOTHING ELSE EVER WILL.
                //
                // Without this, an open regime is asked about BEFORE the floor is
                // consulted, so a cause with no recovery edge (a process fault —
                // nothing ever reports that a panicked node is fine now) stayed
                // suppressed for the REST OF THE RUN: a second, genuinely new
                // panic an hour later captured nothing and was counted as a
                // repeat of the first. Explicit `recover` still works and is
                // still the ONLY thing that re-arms a cause that CAN recover.
                let floor_expired = regime
                    .last_capture_ns
                    .is_some_and(|last| now_ns.saturating_sub(last) >= policy.refractory_ns);
                if has_recovery_edge || !floor_expired {
                    regime.suppressed += 1;
                    let suppressed = regime.suppressed;
                    self.stats.suppressed += 1;
                    return TriggerDecision::Suppressed(SuppressReason::RegimeOpen { suppressed });
                }
                // Re-armed by the floor. The suppression COUNT is deliberately
                // NOT reset — it is a lifetime total (Principle #3), exactly as
                // `recover` leaves it.
                regime.open = false;
            }
            if let Some(last) = regime.last_capture_ns {
                let elapsed = now_ns.saturating_sub(last);
                if elapsed < policy.refractory_ns {
                    regime.suppressed += 1;
                    self.stats.suppressed += 1;
                    return TriggerDecision::Suppressed(SuppressReason::Refractory {
                        retry_in_ns: policy.refractory_ns - elapsed,
                    });
                }
            }
        }

        // (4) The global rate cap — the disk backstop, applied to EVERY kind, at
        // the cap that kind is entitled to. An automatic kind stops at
        // `max_per_hour - manual_reserve` so the operator's verb still has
        // headroom during the flood the automatic triggers are producing (decision
        // 112-F); Manual sees the whole cap and is still refused at it.
        self.prune_rate_window(now_ns);
        let in_window = self.recent.len() as u32;
        let (cap, reserved_for_manual) = self.policy.effective_cap(request.kind);
        if in_window >= cap {
            self.stats.suppressed += 1;
            self.stats.rate_capped += 1;
            return TriggerDecision::Suppressed(SuppressReason::RateCapped {
                captures_in_window: in_window,
                cap,
                reserved_for_manual,
            });
        }

        // CAPTURE.
        let seq = self.next_seq;
        self.next_seq += 1;
        let ends_at_ns = now_ns.saturating_add(self.policy.post_window_ns);
        self.active = Some(ActiveCapture {
            seq,
            started_ns: now_ns,
            ends_at_ns,
            pinned: request.pin,
            // The STARTING request's detail is recorded too — it was being
            // discarded, which is the half of the loss no coalescing test could
            // see (every capture has exactly one of these).
            causes: vec![CaptureCause {
                kind: key.0,
                subject: key.1.clone(),
                detail: request.detail.clone(),
            }],
            causes_dropped: 0,
            dropped: Vec::new(),
            dropped_identities_saturated: false,
        });
        self.recent.push_back(now_ns);
        let regime = self.regimes.entry(key).or_default();
        regime.open = true;
        regime.last_capture_ns = Some(now_ns);
        self.stats.captures += 1;
        TriggerDecision::Capture {
            seq,
            ends_at_ns: Self::external(ends_at_ns),
            pinned: request.pin,
        }
    }

    /// Close the active capture, or `None` when nothing was open.
    ///
    /// Separate from [`Self::decide`] because the gate holds no clock and cannot
    /// know when the caller actually finished writing — a finalize can outlast
    /// `ends_at_ns` by however long the writer took, and inventing an end here
    /// would let a request coalesce into a capture whose bag is already closed.
    pub fn finish_capture(&mut self) -> Option<FinishedCapture> {
        self.active.take().map(|a| FinishedCapture {
            seq: a.seq,
            pinned: a.pinned,
            causes: a.causes,
            causes_dropped: a.causes_dropped,
            causes_dropped_exact: !a.dropped_identities_saturated,
        })
    }

    /// Is the active capture due to finish at `now_ns`?
    pub fn capture_due_to_end(&self, now_ns: u64) -> bool {
        let now_ns = Self::internal(now_ns);
        self.active.as_ref().is_some_and(|a| now_ns >= a.ends_at_ns)
    }

    /// The caller's instant on this gate's INTERNAL line — see [`GATE_EPOCH_NS`].
    fn internal(now_ns: u64) -> u64 {
        now_ns.saturating_add(GATE_EPOCH_NS)
    }

    /// [`Self::internal`]'s inverse, for every instant handed BACK to a caller.
    fn external(internal_ns: u64) -> u64 {
        internal_ns.saturating_sub(GATE_EPOCH_NS)
    }

    /// RE-ARM a cause: its condition recovered, so the next occurrence is news
    /// again.
    ///
    /// Returns how many requests were suppressed while the regime was open — the
    /// number a recovery line reports, and the reason recovery does NOT reset the
    /// lifetime total (Principle #3, the `FailureRegimeLatch` rule).
    ///
    /// The refractory stamp deliberately SURVIVES a recovery. A cause that
    /// recovers and re-fires within the floor is a FLAPPER, and a flapper is the
    /// shape that most needs the floor: without this, a condition oscillating every
    /// two seconds would capture on every rising edge.
    pub fn recover(&mut self, kind: TriggerKind, subject: &str) -> Option<u64> {
        let key = (kind, subject.to_string());
        let regime = self.regimes.get_mut(&key)?;
        if !regime.open {
            return None;
        }
        regime.open = false;
        Some(regime.suppressed)
    }

    /// Drop rate-window entries older than one rolling hour.
    fn prune_rate_window(&mut self, now_ns: u64) {
        let horizon = RATE_WINDOW_MS * NS_PER_MS;
        while let Some(&front) = self.recent.front() {
            // SATURATING, and the direction matters: a `now_ns` BELOW a recorded
            // stamp (a caller handing back a stale reading) must not make the
            // subtraction wrap into an enormous age and evict a capture that
            // really is inside the window — that would silently hand back rate-cap
            // budget nobody earned.
            if now_ns.saturating_sub(front) >= horizon {
                self.recent.pop_front();
            } else {
                break;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = NS_PER_MS;

    fn gate() -> FlashbackTriggerGate {
        FlashbackTriggerGate::new(TriggerPolicy::default())
    }

    /// The vocabulary is a cross-process contract: a log field, a JSON value and a
    /// bag's coverage all spell it, so one rename in one place is a surface that
    /// silently stops matching.
    ///
    /// The oracle is a HAND-WRITTEN table over `TriggerKind::ALL`, walked rather
    /// than sampled: the way a kind ships broken is by being ADDED and then not
    /// reaching one of the three decisions, which a per-variant assertion list
    /// cannot see (it passes by not mentioning the new one).
    #[test]
    fn the_kind_vocabulary_is_the_documented_one() {
        // (kind, wire, is_automatic, has_recovery_edge)
        let oracle: &[(TriggerKind, &str, bool, bool)] = &[
            (TriggerKind::Manual, "manual", false, false),
            (TriggerKind::ProcessFault, "process_fault", true, false),
            (TriggerKind::MonitorVerdict, "monitor_verdict", true, true),
            (TriggerKind::PanicDisable, "panic_disable", true, false),
            (TriggerKind::RunVanished, "run_vanished", true, false),
            (TriggerKind::EStop, "estop", true, false),
            (TriggerKind::Declared, "declared", true, false),
        ];
        assert_eq!(
            oracle.len(),
            TriggerKind::ALL.len(),
            "a kind was added without an oracle row — every kind needs a wire \
             spelling, a spam posture and a recovery answer"
        );
        for (kind, wire, automatic, recovers) in oracle {
            assert_eq!(kind.as_wire(), *wire, "{kind:?} wire spelling");
            assert_eq!(kind.is_automatic(), *automatic, "{kind:?} is_automatic");
            assert_eq!(
                kind.has_recovery_edge(),
                *recovers,
                "{kind:?} has_recovery_edge"
            );
        }
        // MONITOR VERDICT IS THE ONLY RECOVERING KIND, and that is the load-bearing
        // half: `has_recovery_edge == true` means the gate WAITS for a driver, and
        // only `verdict_observer` drives one. A second `true` would open a regime
        // nothing ever closes and silently swallow every later occurrence of that
        // cause for the rest of the run.
        let recovering: Vec<TriggerKind> = TriggerKind::ALL
            .into_iter()
            .filter(|k| k.has_recovery_edge())
            .collect();
        assert_eq!(recovering, vec![TriggerKind::MonitorVerdict]);
        // MANUAL IS THE ONLY EXEMPT KIND. A second `false` would give a trigger a
        // human is not watching the exemptions justified by one who is.
        let exempt: Vec<TriggerKind> = TriggerKind::ALL
            .into_iter()
            .filter(|k| !k.is_automatic())
            .collect();
        assert_eq!(exempt, vec![TriggerKind::Manual]);
    }

    /// Every kind's constructor stamps the kind it is named for.
    ///
    /// Cheap and worth having because the constructors are copy-paste siblings:
    /// a `declared` helper stamping `EStop` would compile, would route through the
    /// right gate arms, and would only ever show up as the wrong word in a bag
    /// somebody reads six months later.
    #[test]
    fn every_constructor_stamps_the_kind_it_is_named_for() {
        assert_eq!(CaptureRequest::manual("d").kind, TriggerKind::Manual);
        assert_eq!(
            CaptureRequest::process_fault("s", "d").kind,
            TriggerKind::ProcessFault
        );
        assert_eq!(
            CaptureRequest::monitor_verdict("s", "d").kind,
            TriggerKind::MonitorVerdict
        );
        assert_eq!(
            CaptureRequest::panic_disable("s", "d").kind,
            TriggerKind::PanicDisable
        );
        assert_eq!(
            CaptureRequest::run_vanished("s", "d").kind,
            TriggerKind::RunVanished
        );
        assert_eq!(CaptureRequest::estop("d").kind, TriggerKind::EStop);
        assert_eq!(
            CaptureRequest::declared("s", "d").kind,
            TriggerKind::Declared
        );
        // An e-stop keys on ONE subject for the whole robot — see the constructor.
        assert_eq!(CaptureRequest::estop("d").subject, "estop");
        // …and every constructor carries the caller's detail through, which is what
        // a reader of the finished bag actually sees.
        assert_eq!(
            CaptureRequest::estop("engaged by k9").detail,
            "engaged by k9"
        );
        assert_eq!(
            CaptureRequest::declared("bumper:front", "hit").detail,
            "hit"
        );
    }

    /// The defaults are numbers other people's arithmetic depends on.
    #[test]
    fn the_rate_cap_knob_resolves_through_the_shared_parser() {
        assert_eq!(
            resolve_max_per_hour(None),
            (DEFAULT_FLASHBACK_MAX_PER_HOUR, None)
        );
        assert_eq!(resolve_max_per_hour(Some("5")).0, 5);
        // A ZERO is refused with the default quoted. The gate would honour it
        // literally ("capture nothing"), which is what makes refusing it here
        // the correct reading of what an operator typing 0 wants.
        let (cap, complaint) = resolve_max_per_hour(Some("0"));
        assert_eq!(cap, DEFAULT_FLASHBACK_MAX_PER_HOUR);
        assert!(complaint.is_some_and(|c| c.contains("20")));
        // The NAME is a cross-process contract: an operator sets it and the
        // refusal message names it, so a rename in one place is a knob that
        // silently stops working.
        assert_eq!(
            FLASHBACK_MAX_PER_HOUR_ENV,
            "CERULION_FLASHBACK_MAX_PER_HOUR"
        );
    }

    #[test]
    fn the_shipped_policy_is_the_documented_one() {
        let p = TriggerPolicy::default();
        assert_eq!(p.post_window_ns, 15_000 * MS);
        assert_eq!(p.refractory_ns, 60_000 * MS);
        assert_eq!(p.max_per_hour, 20);
        assert_eq!(p.max_capture_span_ns, 120_000 * MS);
        assert_eq!(p.max_causes, 32);
        assert_eq!(p.max_dropped_tracked, 128);
        // The identity memory must outsize the cause list, or a capture would
        // start forgetting omissions before it has finished recording them.
        assert!(p.max_dropped_tracked > p.max_causes);
        // The span ceiling must exceed the post window, or EVERY capture would be
        // truncated at its own start — the ceiling would stop being a ceiling and
        // become the length.
        assert!(p.max_capture_span_ns > p.post_window_ns);
    }

    /// The happy path: a fault captures, and the capture covers the post window.
    #[test]
    fn a_first_fault_captures_and_runs_for_the_post_window() {
        let mut g = gate();
        let req = CaptureRequest::process_fault("node:planner", "panicked");
        let d = g.decide(&req, 1_000 * MS);
        assert_eq!(
            d,
            TriggerDecision::Capture {
                seq: 0,
                ends_at_ns: 16_000 * MS,
                pinned: false
            }
        );
        assert!(!g.capture_due_to_end(15_999 * MS));
        assert!(g.capture_due_to_end(16_000 * MS));
        assert_eq!(g.active_capture(), Some((0, 16_000 * MS)));
        let stats = g.stats();
        assert_eq!(
            (stats.requests, stats.captures, stats.suppressed),
            (1, 1, 0)
        );
    }

    /// THE anti-spam headline: a condition that stays bad is ONE capture and a
    /// running count, not a capture per evaluation tick.
    ///
    /// The drive is deliberately spread past the post window, so the repeats are
    /// judged by the LATCH rather than absorbed by coalescing — otherwise this
    /// test would pass against a gate with no latch at all.
    #[test]
    fn a_condition_that_stays_bad_captures_once_and_counts_the_rest() {
        let mut g = gate();
        let req = CaptureRequest::monitor_verdict("stalled:/lowstate", "no frames");
        assert!(matches!(
            g.decide(&req, 0),
            TriggerDecision::Capture { seq: 0, .. }
        ));
        // Nine more evaluations, each well past the previous capture's end.
        for i in 1..=9u64 {
            let now = i * 20_000 * MS;
            match g.decide(&req, now) {
                TriggerDecision::Suppressed(SuppressReason::RegimeOpen { suppressed }) => {
                    assert_eq!(suppressed, i, "the count is a running total");
                }
                other => panic!("evaluation {i} must be suppressed by the latch, got {other:?}"),
            }
        }
        let stats = g.stats();
        assert_eq!(
            (stats.requests, stats.captures, stats.suppressed),
            (10, 1, 9)
        );
        // …and RECOVERY re-arms, reporting what was missed.
        assert_eq!(
            g.recover(TriggerKind::MonitorVerdict, "stalled:/lowstate"),
            Some(9)
        );
        // A second recovery reports nothing — the regime is already closed, and a
        // recovery line per evaluation tick is the flood this machine prevents.
        assert_eq!(
            g.recover(TriggerKind::MonitorVerdict, "stalled:/lowstate"),
            None
        );
        // The lifetime totals are UNCONDITIONAL: recovery did not reset them.
        assert_eq!(g.stats().suppressed, 9);
    }

    /// A recovered cause re-fires INSIDE the floor and is still held — the flapper
    /// shape, which is exactly what the floor is for and which the latch alone
    /// cannot see (the regime is legitimately closed).
    #[test]
    fn a_flapping_cause_is_held_by_the_refractory_floor_after_it_recovers() {
        let mut g = gate();
        let req = CaptureRequest::monitor_verdict("stalled:/imu", "no frames");
        assert!(matches!(g.decide(&req, 0), TriggerDecision::Capture { .. }));
        assert_eq!(
            g.recover(TriggerKind::MonitorVerdict, "stalled:/imu"),
            Some(0)
        );
        // Re-fires 20 s later: past the post window (so not coalesced), regime
        // CLOSED (so not latched) — only the floor can hold it.
        match g.decide(&req, 20_000 * MS) {
            TriggerDecision::Suppressed(SuppressReason::Refractory { retry_in_ns }) => {
                assert_eq!(retry_in_ns, 40_000 * MS);
            }
            other => panic!("a flapper inside the floor must be held, got {other:?}"),
        }
        // …and at the floor it captures again. BOTH sides of the boundary, because
        // a floor that never expires is a permanent mute.
        assert!(matches!(
            g.decide(&req, 59_999 * MS),
            TriggerDecision::Suppressed(SuppressReason::Refractory { .. })
        ));
        g.recover(TriggerKind::MonitorVerdict, "stalled:/imu");
        assert!(matches!(
            g.decide(&req, 60_000 * MS),
            TriggerDecision::Capture { seq: 1, .. }
        ));
    }

    /// Coalescing: a burst is ONE bag whose causes are a list.
    #[test]
    fn a_burst_is_one_capture_carrying_every_cause() {
        let mut g = gate();
        assert!(matches!(
            g.decide(&CaptureRequest::process_fault("rank:1", "SIGSEGV"), 0),
            TriggerDecision::Capture { seq: 0, .. }
        ));
        // A DIFFERENT cause, 100 ms later — inside the post window.
        match g.decide(
            &CaptureRequest::process_fault("rank:2", "exited: 101"),
            100 * MS,
        ) {
            TriggerDecision::Extend {
                seq,
                ends_at_ns,
                cause,
            } => {
                assert_eq!(seq, 0, "one bag");
                assert_eq!(cause, CauseRecord::Added, "a new cause is new information");
                assert_eq!(ends_at_ns, 15_100 * MS, "the moment keeps going");
            }
            other => panic!("a burst member must join the capture, got {other:?}"),
        }
        // The SAME cause again: still an extension in time, but not new
        // information — and its detail must NOT overwrite the first one's.
        match g.decide(
            &CaptureRequest::process_fault("rank:2", "a later, vaguer line"),
            200 * MS,
        ) {
            TriggerDecision::Extend { cause, .. } => assert_eq!(cause, CauseRecord::Repeated),
            other => panic!("a repeat inside the window extends, got {other:?}"),
        }
        // THE DETAILS REACH THE CAPTURE — from the request that STARTED it as
        // well as from the coalesced one. `CaptureRequest::detail` promises this,
        // and a gate keeping only the regime key discarded both.
        assert_eq!(
            g.active_causes(),
            vec![
                CaptureCause {
                    kind: TriggerKind::ProcessFault,
                    subject: "rank:1".to_string(),
                    detail: "SIGSEGV".to_string(),
                },
                CaptureCause {
                    kind: TriggerKind::ProcessFault,
                    subject: "rank:2".to_string(),
                    detail: "exited: 101".to_string(),
                },
            ]
        );
        assert_eq!(g.stats().captures, 1);
        assert_eq!(g.stats().coalesced, 2);
    }

    /// THE reason coalescing runs before the latch: a burst member must come out
    /// of the capture with its regime OPEN, or it re-captures the moment the bag
    /// finalizes.
    ///
    /// This is the arm that fails if the two are reordered — the ordering is
    /// otherwise invisible, since both orders produce one bag.
    #[test]
    fn a_coalesced_cause_leaves_the_capture_latched() {
        let mut g = gate();
        g.decide(&CaptureRequest::process_fault("rank:1", "died"), 0);
        g.decide(&CaptureRequest::process_fault("rank:2", "died"), 100 * MS);
        g.finish_capture();
        // Well past the (finished) capture and past nothing else: rank:2 fired
        // INSIDE the bag, so it is a known condition, not news.
        match g.decide(
            &CaptureRequest::process_fault("rank:2", "died"),
            20_000 * MS,
        ) {
            TriggerDecision::Suppressed(SuppressReason::RegimeOpen { suppressed }) => {
                assert_eq!(suppressed, 1);
            }
            other => panic!("a cause that rode a capture must be latched by it, got {other:?}"),
        }
    }

    /// The span ceiling: a steadily faulting robot gets a SERIES of bounded bags,
    /// never one that will not finalize.
    #[test]
    fn coalescing_cannot_extend_a_capture_past_the_span_ceiling() {
        let mut g = gate();
        g.decide(&CaptureRequest::process_fault("a", "x"), 0);
        // Push every 10 s for 5 minutes. Each request is inside the previous end
        // (post window 15 s), so every one coalesces.
        let mut last_end = 15_000 * MS;
        for i in 1..=30u64 {
            let now = i * 10_000 * MS;
            match g.decide(&CaptureRequest::process_fault("a", "x"), now) {
                TriggerDecision::Extend { ends_at_ns, .. } => {
                    assert!(ends_at_ns >= last_end, "the end never moves backwards");
                    last_end = ends_at_ns;
                }
                // Once the ceiling is reached the capture's end stops moving, so a
                // later request falls OUTSIDE it and is judged normally. That is
                // the ceiling working, and the point at which it stops being one
                // bag.
                TriggerDecision::Suppressed(_) | TriggerDecision::Capture { .. } => break,
            }
        }
        assert_eq!(
            last_end,
            120_000 * MS,
            "the capture is clamped at exactly the span ceiling"
        );
    }

    /// Manual bypasses the LATCH and the FLOOR: a verb that ran must not silently
    /// do nothing.
    #[test]
    fn a_manual_request_is_never_latched_or_held_by_the_floor() {
        let mut g = gate();
        // Ten manual requests, each past the previous capture's window so none is
        // absorbed by coalescing.
        for i in 0..10u64 {
            let now = i * 20_000 * MS;
            match g.decide(&CaptureRequest::manual("operator asked"), now) {
                TriggerDecision::Capture { seq, .. } => assert_eq!(seq, i),
                other => panic!("manual request {i} must capture, got {other:?}"),
            }
        }
        assert_eq!(g.stats().captures, 10);
        assert_eq!(g.stats().suppressed, 0);
    }

    /// …but manual is NOT exempt from the disk backstop, and the suppression is
    /// quantified.
    #[test]
    fn the_rate_cap_applies_to_every_kind_and_reports_its_numbers() {
        let mut g = gate();
        for i in 0..20u64 {
            assert!(
                matches!(
                    g.decide(&CaptureRequest::manual("x"), i * 20_000 * MS),
                    TriggerDecision::Capture { .. }
                ),
                "capture {i} is inside the cap"
            );
        }
        match g.decide(&CaptureRequest::manual("x"), 20 * 20_000 * MS) {
            TriggerDecision::Suppressed(SuppressReason::RateCapped {
                captures_in_window,
                cap,
                reserved_for_manual,
            }) => {
                // MANUAL sees the WHOLE cap and no reserve — the reserve is held
                // back from the automatic kinds, never from the verb it protects.
                assert_eq!((captures_in_window, cap, reserved_for_manual), (20, 20, 0));
            }
            other => panic!("the 21st must be rate-capped, got {other:?}"),
        }
        // The rate-capped count is kept APART from the total, because it says
        // nothing about the robot's health.
        let stats = g.stats();
        assert_eq!((stats.suppressed, stats.rate_capped), (1, 1));
        // …and the window ROLLS: an hour after the first capture, budget returns.
        // The first capture was at t=0, so at t = 1 h it is exactly at the horizon
        // and is evicted (`>=`).
        assert!(matches!(
            g.decide(&CaptureRequest::manual("x"), RATE_WINDOW_MS * MS),
            TriggerDecision::Capture { .. }
        ));
    }

    /// By design, a manual request at the cap must fail loudly, never a silent
    /// no-op.
    ///
    /// "Loudly" is a property of the SEAM as well as of whatever logs it, so this
    /// asserts what the gate returns rather than what a caller does with it:
    /// the decision must be a `Suppressed` a caller cannot mistake for success,
    /// it must be the RATE-CAP arm specifically (naming the budget an operator
    /// can raise, not a regime a manual request has no way to recover), and it
    /// must carry BOTH numbers, so the refusal is actionable rather than a bare
    /// verdict. `cerulion flashback`'s own non-zero exit is the CLI half; this is
    /// the half that makes it possible.
    ///
    /// The pin that matters most is the negative one: at the cap the gate must
    /// NOT return `Capture`, because a verb that reports success and writes no
    /// bag is the silent no-op the decision forbids.
    #[test]
    fn a_manual_request_at_the_rate_cap_is_refused_loudly_never_silently() {
        let mut g = FlashbackTriggerGate::new(TriggerPolicy {
            max_per_hour: 2,
            ..TriggerPolicy::default()
        });
        for i in 0..2u64 {
            assert!(matches!(
                g.decide(&CaptureRequest::manual("operator asked"), i * 20_000 * MS),
                TriggerDecision::Capture { .. }
            ));
        }
        // Past the cap, and past every window that could absorb it: the previous
        // capture's post window closed 5 s ago, so this is judged, not coalesced.
        let decision = g.decide(&CaptureRequest::manual("operator asked"), 40_000 * MS);
        match &decision {
            TriggerDecision::Suppressed(SuppressReason::RateCapped {
                captures_in_window,
                cap,
                reserved_for_manual,
            }) => {
                // ACTIONABLE: the number in force and the number reached.
                assert_eq!((*captures_in_window, *cap), (2, 2));
                assert_eq!(*reserved_for_manual, 0, "manual is never charged a reserve");
            }
            other => panic!("a manual request at the cap must be REFUSED, got {other:?}"),
        }
        // …and refused in a way no caller can read as success.
        assert!(
            !matches!(decision, TriggerDecision::Capture { .. }),
            "a refused manual request must never look like a capture"
        );
        // …and it opened NO NEW capture. The previous one is still the gate's
        // active entry — `finish_capture` is the caller's call, deliberately, so
        // the assertion is that `seq` did not advance rather than that nothing is
        // held.
        assert_eq!(
            g.active_capture().map(|(seq, _)| seq),
            Some(1),
            "a refused request must not start a capture"
        );
        // The refusal is COUNTED, unconditionally (Principle #3) — an operator
        // who missed the line can still see it happened.
        let stats = g.stats();
        assert_eq!(
            (stats.captures, stats.suppressed, stats.rate_capped),
            (2, 1, 1)
        );
        // A manual request is NEVER refused by the latch or the floor — those are
        // the arms that would make it a silent no-op, since nothing ever
        // "recovers" an operator's request. Pinned by exclusion, so a future
        // reordering that routes manual through them fails here.
        assert!(
            !matches!(
                decision,
                TriggerDecision::Suppressed(SuppressReason::RegimeOpen { .. })
                    | TriggerDecision::Suppressed(SuppressReason::Refractory { .. })
            ),
            "manual must be exempt from the latch and the floor: {decision:?}"
        );
    }

    /// THREAD 4: a regime with NO recovery edge re-arms at the floor — otherwise
    /// a process fault is suppressed for the REST OF THE RUN.
    ///
    /// The latch is asked before the floor, so an open regime short-circuited
    /// every later request and `last_capture_ns` was never reached. Nothing ever
    /// calls `recover` for a panicked node, so the second genuinely new panic of
    /// a run captured nothing.
    ///
    /// Both sides of the boundary, because a floor that never expires is the bug
    /// and a floor that expires early is spam.
    #[test]
    fn an_open_process_fault_regime_re_arms_at_the_refractory_floor() {
        let mut g = gate();
        let req = CaptureRequest::process_fault("node:planner", "panicked");
        assert!(matches!(
            g.decide(&req, 0),
            TriggerDecision::Capture { seq: 0, .. }
        ));
        // Inside the floor: still latched, still counted.
        match g.decide(&req, 59_999 * MS) {
            TriggerDecision::Suppressed(SuppressReason::RegimeOpen { suppressed }) => {
                assert_eq!(suppressed, 1);
            }
            other => panic!("inside the floor a repeat must be latched, got {other:?}"),
        }
        // AT the floor, with NOTHING having called `recover`: a new panic is news.
        assert!(
            matches!(
                g.decide(&req, 60_000 * MS),
                TriggerDecision::Capture { seq: 1, .. }
            ),
            "a fault regime nothing can recover must re-arm at the floor"
        );
        // The lifetime suppression total SURVIVES the re-arm (Principle #3),
        // exactly as it survives an explicit `recover`.
        assert_eq!(g.stats().suppressed, 1);
        assert_eq!(g.stats().captures, 2);
    }

    /// …and its CONTROL: a cause that CAN recover is NOT floor-re-armed.
    ///
    /// Without this, floor re-arming becomes "every open regime captures once a
    /// minute", which is the spam the not-spammy requirement forbids: a monitor
    /// condition that stays bad for an hour would write 60 bags. Its recovery
    /// edge is the reliable signal, and `recover` is the only thing that re-arms it.
    #[test]
    fn a_monitor_regime_is_not_re_armed_by_the_floor_because_it_can_recover() {
        let mut g = gate();
        let req = CaptureRequest::monitor_verdict("stalled:/lowstate", "no frames");
        assert!(matches!(
            g.decide(&req, 0),
            TriggerDecision::Capture { seq: 0, .. }
        ));
        // Ten minutes of a condition that stays bad — ten floors' worth.
        for i in 1..=10u64 {
            let now = i * 60_000 * MS;
            match g.decide(&req, now) {
                TriggerDecision::Suppressed(SuppressReason::RegimeOpen { suppressed }) => {
                    assert_eq!(suppressed, i);
                }
                other => panic!("a recoverable regime must stay latched at {now}: {other:?}"),
            }
        }
        assert_eq!(g.stats().captures, 1, "one bag, not eleven");
        // And its recovery edge still works.
        assert_eq!(
            g.recover(TriggerKind::MonitorVerdict, "stalled:/lowstate"),
            Some(10)
        );
        assert!(matches!(
            g.decide(&req, 700_000 * MS),
            TriggerDecision::Capture { seq: 1, .. }
        ));
    }

    /// THREAD 3: the cause list is BOUNDED, and reaching the bound is REPORTED.
    ///
    /// Coalescing dedupes by `(kind, subject)`, so the list is bounded by distinct
    /// CONDITIONS — which is no bound at all on the shape this exists for (one
    /// cause per topic when a robot's LAN drops). The request still extends the
    /// capture; only its cause is dropped, and the drop is counted.
    #[test]
    fn a_capture_bounds_its_cause_list_and_reports_what_it_dropped() {
        let mut g = FlashbackTriggerGate::new(TriggerPolicy {
            max_causes: 3,
            ..TriggerPolicy::default()
        });
        assert!(matches!(
            g.decide(&CaptureRequest::monitor_verdict("stalled:/a", "a"), 0),
            TriggerDecision::Capture { .. }
        ));
        for (i, subject) in ["stalled:/b", "stalled:/c"].iter().enumerate() {
            let now = (i as u64 + 1) * 10 * MS;
            assert_eq!(
                g.decide(&CaptureRequest::monitor_verdict(*subject, "x"), now),
                TriggerDecision::Extend {
                    seq: 0,
                    ends_at_ns: now + 15_000 * MS,
                    cause: CauseRecord::Added,
                }
            );
        }
        // The fourth DISTINCT cause: the capture still extends, the cause does not
        // fit, and the request is told so.
        match g.decide(&CaptureRequest::monitor_verdict("stalled:/d", "x"), 30 * MS) {
            TriggerDecision::Extend {
                cause, ends_at_ns, ..
            } => {
                assert_eq!(cause, CauseRecord::Dropped);
                assert_eq!(
                    ends_at_ns,
                    15_030 * MS,
                    "a dropped cause still extends the moment"
                );
            }
            other => panic!("a full list must still extend, got {other:?}"),
        }
        let finished = g.finish_capture().expect("a capture is open");
        assert_eq!(finished.causes.len(), 3, "the list is capped");
        assert_eq!(
            finished.causes_dropped, 1,
            "and the capture says its summary is incomplete"
        );
        assert!(
            finished.causes_dropped_exact,
            "one identity fits easily inside the memory bound"
        );
        // A capture that fit everything reports ZERO — without this the field is
        // satisfied by one that always counts.
        let mut g = gate();
        g.decide(&CaptureRequest::manual("op"), 0);
        let finished = g.finish_capture().expect("open");
        assert_eq!(finished.causes_dropped, 0);
        assert!(finished.causes_dropped_exact);
    }

    /// `causes_dropped` counts DISTINCT omitted causes, which is what its own
    /// documentation claims — so a cause that was dropped and then REPEATS is
    /// recognised, not counted a second time.
    ///
    /// Only RETAINED causes were checked for repeats, and a dropped cause is by
    /// definition never retained, so every repeat of one re-incremented the
    /// counter and returned `Dropped` again. The summary then overstated the
    /// distinct total without bound: ONE condition repeating at frame rate
    /// reported hundreds of "further distinct causes".
    #[test]
    fn a_repeat_of_a_dropped_cause_is_a_repeat_not_a_second_drop() {
        let mut g = FlashbackTriggerGate::new(TriggerPolicy {
            max_causes: 1,
            ..TriggerPolicy::default()
        });
        assert!(matches!(
            g.decide(&CaptureRequest::monitor_verdict("stalled:/a", "x"), 0),
            TriggerDecision::Capture { .. }
        ));
        // `/b` does not fit: dropped, and counted ONCE.
        assert!(matches!(
            g.decide(&CaptureRequest::monitor_verdict("stalled:/b", "x"), 10 * MS),
            TriggerDecision::Extend {
                cause: CauseRecord::Dropped,
                ..
            }
        ));
        // The SAME cause again, five more times. Each still EXTENDS the capture —
        // dropping a cause never suppresses the request — but none is a fresh
        // omission.
        for i in 1..=5u64 {
            let now = (10 + i * 10) * MS;
            match g.decide(&CaptureRequest::monitor_verdict("stalled:/b", "x"), now) {
                TriggerDecision::Extend {
                    cause, ends_at_ns, ..
                } => {
                    assert_eq!(cause, CauseRecord::Repeated, "repeat {i}");
                    assert_eq!(ends_at_ns, now + 15_000 * MS, "it still extends");
                }
                other => panic!("repeat {i} must extend, got {other:?}"),
            }
        }
        // A DIFFERENT cause is a genuinely new omission and IS counted.
        assert!(matches!(
            g.decide(&CaptureRequest::monitor_verdict("stalled:/c", "x"), 70 * MS),
            TriggerDecision::Extend {
                cause: CauseRecord::Dropped,
                ..
            }
        ));
        let finished = g.finish_capture().expect("a capture is open");
        assert_eq!(
            finished.causes_dropped, 2,
            "two DISTINCT causes were omitted, however often they repeated"
        );
        assert!(
            finished.causes_dropped_exact,
            "well inside the memory bound"
        );
        assert_eq!(finished.causes.len(), 1, "the retained cap is untouched");
    }

    /// The identity memory is BOUNDED, and past the bound the capture says its
    /// distinct count may overstate rather than quietly reverting.
    ///
    /// Both sides: at the bound every repeat is still recognised and the count
    /// stays exact; ONE identity past it, a repeat can no longer be recognised
    /// and `causes_dropped_exact` goes false. Without the second half the bound
    /// would be an unreported silent degradation — the thing this repo refuses —
    /// and without the first it would be satisfied by a gate that never claims
    /// exactness at all.
    #[test]
    fn the_dropped_identity_memory_is_bounded_and_says_when_it_saturates() {
        const TRACKED: usize = 3;
        let policy = TriggerPolicy {
            max_causes: 1,
            max_dropped_tracked: TRACKED,
            ..TriggerPolicy::default()
        };

        // AT the bound: three distinct causes dropped, each remembered.
        let mut g = FlashbackTriggerGate::new(policy);
        g.decide(&CaptureRequest::monitor_verdict("retained", "x"), 0);
        for i in 0..TRACKED {
            let req = CaptureRequest::monitor_verdict(format!("drop:{i}"), "x");
            assert!(matches!(
                g.decide(&req, 10 * MS),
                TriggerDecision::Extend {
                    cause: CauseRecord::Dropped,
                    ..
                }
            ));
            // …and its repeat is RECOGNISED.
            assert!(matches!(
                g.decide(&req, 20 * MS),
                TriggerDecision::Extend {
                    cause: CauseRecord::Repeated,
                    ..
                }
            ));
        }
        let finished = g.finish_capture().expect("open");
        assert_eq!(finished.causes_dropped, TRACKED as u32);
        assert!(
            finished.causes_dropped_exact,
            "exactly at the bound, nothing was forgotten"
        );

        // ONE past it: the fourth identity does not fit.
        let mut g = FlashbackTriggerGate::new(policy);
        g.decide(&CaptureRequest::monitor_verdict("retained", "x"), 0);
        for i in 0..=TRACKED {
            g.decide(
                &CaptureRequest::monitor_verdict(format!("drop:{i}"), "x"),
                10 * MS,
            );
        }
        // The UNREMEMBERED cause's repeat is counted again — the accepted cost.
        assert!(matches!(
            g.decide(
                &CaptureRequest::monitor_verdict(format!("drop:{TRACKED}"), "x"),
                20 * MS
            ),
            TriggerDecision::Extend {
                cause: CauseRecord::Dropped,
                ..
            }
        ));
        // …while a cause remembered BEFORE saturation is STILL recognised, so
        // the degradation is confined to what could not be stored rather than
        // disabling the whole mechanism.
        assert!(matches!(
            g.decide(&CaptureRequest::monitor_verdict("drop:0", "x"), 30 * MS),
            TriggerDecision::Extend {
                cause: CauseRecord::Repeated,
                ..
            }
        ));
        let finished = g.finish_capture().expect("open");
        assert!(
            !finished.causes_dropped_exact,
            "past the bound the count is an UPPER BOUND, and the capture says so"
        );
        assert_eq!(
            finished.causes_dropped,
            TRACKED as u32 + 2,
            "the unremembered identity was counted twice — the accepted overstatement"
        );
    }

    /// The suppression arms are ordered most-specific-first: a request that trips
    /// BOTH the latch and the rate cap reports the LATCH, because that is the one
    /// naming a condition an operator can act on.
    #[test]
    fn a_request_that_trips_two_arms_reports_the_actionable_one() {
        let mut g = FlashbackTriggerGate::new(TriggerPolicy {
            max_per_hour: 1,
            ..TriggerPolicy::default()
        });
        let req = CaptureRequest::monitor_verdict("stalled:/a", "x");
        assert!(matches!(g.decide(&req, 0), TriggerDecision::Capture { .. }));
        // The cap is spent AND the regime is open. The regime wins.
        match g.decide(&req, 20_000 * MS) {
            TriggerDecision::Suppressed(SuppressReason::RegimeOpen { .. }) => {}
            other => panic!("the actionable arm must be reported, got {other:?}"),
        }
        // A DIFFERENT cause, whose regime is closed, then reports the cap.
        match g.decide(
            &CaptureRequest::monitor_verdict("stalled:/b", "x"),
            20_000 * MS,
        ) {
            TriggerDecision::Suppressed(SuppressReason::RateCapped { .. }) => {}
            other => panic!("a fresh cause must reach the cap arm, got {other:?}"),
        }
    }

    /// A zero cap captures NOTHING — a legitimate way to leave the plane running
    /// while writing no bags, and the boundary that proves the comparison is
    /// `>=` rather than `>` (at `max_per_hour == 0` an empty window must already
    /// be full).
    #[test]
    fn a_zero_rate_cap_captures_nothing() {
        let mut g = FlashbackTriggerGate::new(TriggerPolicy {
            max_per_hour: 0,
            ..TriggerPolicy::default()
        });
        assert!(matches!(
            g.decide(&CaptureRequest::manual("x"), 0),
            TriggerDecision::Suppressed(SuppressReason::RateCapped {
                captures_in_window: 0,
                cap: 0,
                // A cap of ZERO reserves NOTHING: holding a slot back out of a
                // budget that admits no captures would invent one.
                reserved_for_manual: 0
            })
        ));
        assert_eq!(g.stats().captures, 0);
    }

    /// Pinning rides the request through to the decision, and a pin arriving on a
    /// COALESCED request still pins the bag it joined — the operator pinned the
    /// moment, not their own request.
    #[test]
    fn a_pin_reaches_the_capture_even_when_it_arrives_by_coalescing() {
        let mut g = gate();
        assert_eq!(
            g.decide(&CaptureRequest::process_fault("a", "x"), 0),
            TriggerDecision::Capture {
                seq: 0,
                ends_at_ns: 15_000 * MS,
                pinned: false
            }
        );
        g.decide(&CaptureRequest::manual("pin it").pinned(), 100 * MS);
        let finished = g.finish_capture().expect("a capture is open");
        assert_eq!(finished.seq, 0);
        assert!(
            finished.pinned,
            "the pin must reach the bag the operator meant"
        );
        assert_eq!(finished.causes.len(), 2);
        assert!(
            g.finish_capture().is_none(),
            "closing twice is not a capture"
        );
    }

    /// A stale `now_ns` must not hand back rate-cap budget nobody earned.
    #[test]
    fn a_clock_reading_below_a_recorded_capture_evicts_nothing() {
        let mut g = FlashbackTriggerGate::new(TriggerPolicy {
            max_per_hour: 1,
            ..TriggerPolicy::default()
        });
        assert!(matches!(
            g.decide(&CaptureRequest::manual("x"), 10_000_000 * MS),
            TriggerDecision::Capture { .. }
        ));
        // Close it, or the second request would COALESCE (a stale reading is
        // trivially inside an open capture's window) and never reach the rate
        // window at all — which is the precondition this arm needs, not the
        // property it is about.
        g.finish_capture();
        // A reading BELOW the recorded stamp. A wrapping subtraction would compute
        // an enormous age, evict the entry and grant a fresh capture.
        assert!(matches!(
            g.decide(&CaptureRequest::manual("x"), 1_000 * MS),
            TriggerDecision::Suppressed(SuppressReason::RateCapped { .. })
        ));
    }

    /// Determinism (Principle #7): the same request sequence yields the same
    /// decision sequence, against a HAND-written oracle rather than a second run
    /// of the same code.
    #[test]
    fn one_request_sequence_yields_one_hand_written_decision_sequence() {
        let script: Vec<(CaptureRequest, u64)> = vec![
            (CaptureRequest::process_fault("a", "x"), 0),
            (CaptureRequest::process_fault("b", "x"), 1_000 * MS),
            (CaptureRequest::process_fault("a", "x"), 30_000 * MS),
            (CaptureRequest::manual("op"), 31_000 * MS),
        ];
        let oracle = vec![
            TriggerDecision::Capture {
                seq: 0,
                ends_at_ns: 15_000 * MS,
                pinned: false,
            },
            TriggerDecision::Extend {
                seq: 0,
                ends_at_ns: 16_000 * MS,
                cause: CauseRecord::Added,
            },
            // `a` rode the capture, so its regime is open: latched, not captured.
            TriggerDecision::Suppressed(SuppressReason::RegimeOpen { suppressed: 1 }),
            // Manual is exempt from the latch and the floor.
            TriggerDecision::Capture {
                seq: 1,
                ends_at_ns: 46_000 * MS,
                pinned: false,
            },
        ];
        for run in 0..2 {
            let mut g = gate();
            let got: Vec<TriggerDecision> = script
                .iter()
                .map(|(req, now)| g.decide(req, *now))
                .collect();
            assert_eq!(got, oracle, "run {run}");
        }
    }

    /// The anti-tautology control: a healthy robot's gate decides nothing, counts
    /// nothing and holds no state. Without it, every "exactly N" arm above is
    /// satisfied by a gate that also fires on nothing at all.
    #[test]
    fn a_gate_nobody_asks_captures_nothing_and_counts_nothing() {
        let g = gate();
        assert_eq!(g.stats(), TriggerStats::default());
        assert_eq!(g.active_capture(), None);
        assert!(g.active_causes().is_empty());
        assert!(!g.capture_due_to_end(u64::MAX));
    }

    // ---------------------------------------------------------------------
    // The manual reserve.
    // ---------------------------------------------------------------------

    /// THE HEADLINE: an automatic flood cannot lock the operator's verb out.
    ///
    /// Driven as the event actually arrives — a robot coming apart mints
    /// automatic requests until it is refused, and only THEN does the operator
    /// reach for the verb. Both halves are asserted in one body because either
    /// alone reads as the other's bug: "automatic stops at 18" without the manual
    /// capture is indistinguishable from an off-by-two in the cap, and "manual
    /// captures" without the automatic refusal is indistinguishable from no
    /// reserve at all.
    #[test]
    fn an_automatic_flood_cannot_lock_the_operator_out_of_the_verb() {
        let mut g = gate();
        // Distinct subjects, so the LATCH and the FLOOR never fire and the only
        // thing that can refuse these is the rate cap.
        for i in 0..18u64 {
            match g.decide(
                &CaptureRequest::process_fault(format!("graph/p{i}"), "worker died"),
                i * 20_000 * MS,
            ) {
                TriggerDecision::Capture { .. } => {}
                other => panic!("automatic capture {i} is inside the reserved cap, got {other:?}"),
            }
        }
        // The 19th automatic request is refused — at 18, not 20.
        match g.decide(
            &CaptureRequest::process_fault("graph/p18", "worker died"),
            18 * 20_000 * MS,
        ) {
            TriggerDecision::Suppressed(SuppressReason::RateCapped {
                captures_in_window,
                cap,
                reserved_for_manual,
            }) => {
                // The EFFECTIVE cap, and the reason it is lower than the knob.
                assert_eq!((captures_in_window, cap, reserved_for_manual), (18, 18, 2));
            }
            other => panic!("the 19th automatic request must be reserve-capped, got {other:?}"),
        }
        // …and the operator, watching the flood, still gets BOTH reserved slots.
        for i in 0..2u64 {
            match g.decide(
                &CaptureRequest::manual("the moment I actually care about"),
                (19 + i) * 20_000 * MS,
            ) {
                TriggerDecision::Capture { .. } => {}
                other => panic!("reserved manual capture {i} must succeed, got {other:?}"),
            }
        }
        // The loud refusal is preserved, not weakened: at the full cap a manual request is
        // still refused, loudly, with the numbers.
        match g.decide(&CaptureRequest::manual("one too many"), 21 * 20_000 * MS) {
            TriggerDecision::Suppressed(SuppressReason::RateCapped {
                captures_in_window,
                cap,
                ..
            }) => assert_eq!((captures_in_window, cap), (20, 20)),
            other => panic!("a manual request at the FULL cap must be refused, got {other:?}"),
        }
    }

    /// The clamp: a reserve may never disable the automatic triggers outright.
    ///
    /// Pinned as ARITHMETIC over the whole small-cap range rather than at one
    /// point, because the hazard is a flat subtraction — which is correct at every
    /// large cap and silently switches the watchdog off at the small ones an
    /// operator reaches for when tightening a disk budget.
    #[test]
    fn the_manual_reserve_never_starves_the_automatic_triggers_it_protects() {
        // (max_per_hour, automatic cap, reserve withheld)
        let oracle: &[(u32, u32, u32)] = &[
            // ZERO means "capture nothing" and is honoured literally on both
            // sides — reserving a slot out of nothing would invent one.
            (0, 0, 0),
            (1, 1, 0),
            (2, 1, 1),
            (3, 1, 2),
            (4, 2, 2),
            (20, 18, 2),
            (u32::MAX, u32::MAX - 2, 2),
        ];
        for (max_per_hour, expect_cap, expect_reserve) in oracle {
            let policy = TriggerPolicy {
                max_per_hour: *max_per_hour,
                ..TriggerPolicy::default()
            };
            assert_eq!(
                policy.effective_cap(TriggerKind::ProcessFault),
                (*expect_cap, *expect_reserve),
                "automatic cap at max_per_hour={max_per_hour}"
            );
            // Manual always sees the whole thing.
            assert_eq!(
                policy.effective_cap(TriggerKind::Manual),
                (*max_per_hour, 0),
                "manual cap at max_per_hour={max_per_hour}"
            );
            // …and the invariant the clamp exists for.
            if *max_per_hour > 0 {
                assert!(
                    *expect_cap >= 1,
                    "a nonzero cap must always leave the automatic triggers a slot"
                );
            }
        }
    }

    /// A cap of 1 is the boundary the clamp is written for: the reserve collapses
    /// to zero and the two kinds SHARE the single slot, first come.
    #[test]
    fn at_a_cap_of_one_the_reserve_collapses_and_the_slot_is_shared() {
        let mut g = FlashbackTriggerGate::new(TriggerPolicy {
            max_per_hour: 1,
            ..TriggerPolicy::default()
        });
        assert!(matches!(
            g.decide(&CaptureRequest::process_fault("graph/p0", "died"), 0),
            TriggerDecision::Capture { .. }
        ));
        // The automatic request took the only slot, and Manual is refused at it —
        // the correct outcome, and the reason the clamp does not go further.
        match g.decide(&CaptureRequest::manual("x"), 40_000 * MS) {
            TriggerDecision::Suppressed(SuppressReason::RateCapped { cap, .. }) => {
                assert_eq!(cap, 1)
            }
            other => panic!("the shared slot is spent, got {other:?}"),
        }
    }

    // ---------------------------------------------------------------------
    // The per-trigger posture.
    // ---------------------------------------------------------------------

    /// A switched-off KIND is refused at the gate, naming the switch.
    ///
    /// The gate is the backstop for the kinds a SECOND PROCESS publishes — a
    /// supervisor, a `cerud` handler, a user's own detector — none of which need
    /// share this recorder's environment.
    #[test]
    fn a_switched_off_kind_is_refused_at_the_gate_and_the_refusal_names_the_switch() {
        let mut g = FlashbackTriggerGate::with_posture(
            TriggerPolicy::default(),
            TriggerPosture::default().with(TriggerSwitch::EStop, false),
        );
        match g.decide(&CaptureRequest::estop("engaged by k9"), 0) {
            TriggerDecision::Suppressed(SuppressReason::Disabled { switch }) => {
                assert_eq!(switch, TriggerSwitch::EStop);
                // The refusal names the variable an operator types, not a prose
                // description of it.
                assert_eq!(switch.env_suffix(), "ESTOP");
                assert_eq!(switch.env_var(), "CERULION_FLASHBACK_ON_ESTOP");
            }
            other => panic!("a switched-off e-stop must be refused, got {other:?}"),
        }
        // It is COUNTED apart: a configuration refusal must not read like a flood.
        let stats = g.stats();
        assert_eq!(
            (stats.suppressed, stats.disabled, stats.rate_capped),
            (1, 1, 0)
        );
        // …and it spent NOTHING. No capture, no regime, no rate-cap slot — so the
        // sibling kinds are untouched and a later re-enable starts clean.
        assert_eq!(stats.captures, 0);
        assert!(g.active_capture().is_none());
        assert!(matches!(
            g.decide(&CaptureRequest::process_fault("graph/p0", "died"), MS),
            TriggerDecision::Capture { .. }
        ));
    }

    /// A switched-off trigger cannot join an OPEN capture either.
    ///
    /// This is the arm the arm ORDER exists for. Coalescing runs before every
    /// suppression arm deliberately, so a posture check placed after it would let
    /// a disabled trigger EXTEND the capture — recording its cause in the bag and
    /// opening its regime — which is neither of the two states the switch admits.
    #[test]
    fn a_switched_off_trigger_cannot_extend_an_open_capture() {
        let mut g = FlashbackTriggerGate::with_posture(
            TriggerPolicy::default(),
            TriggerPosture::default().with(TriggerSwitch::Declared, false),
        );
        assert!(matches!(
            g.decide(&CaptureRequest::process_fault("graph/p0", "died"), 0),
            TriggerDecision::Capture { .. }
        ));
        // Inside the open capture's post window.
        match g.decide(&CaptureRequest::declared("bumper:front", "hit"), 1_000 * MS) {
            TriggerDecision::Suppressed(SuppressReason::Disabled { switch }) => {
                assert_eq!(switch, TriggerSwitch::Declared)
            }
            other => panic!("a disabled trigger must not extend a capture, got {other:?}"),
        }
        // The capture carries ONE cause — the disabled trigger left no trace in
        // the evidence.
        assert_eq!(g.active_causes().len(), 1);
        assert_eq!(g.active_causes()[0].kind, TriggerKind::ProcessFault);
    }

    /// The two kinds with NO kind-level switch are never refused by the gate.
    ///
    /// The direction is the whole assertion: reading "no switch" as "refuse" would
    /// silently disable `cerulion flashback` and every monitor verdict on every
    /// robot. Driven with EVERY switch off, so the answer cannot be an accident of
    /// the defaults.
    #[test]
    fn the_gate_never_refuses_a_kind_whose_posture_it_does_not_own() {
        let posture = TriggerSwitch::ALL
            .into_iter()
            .fold(TriggerPosture::default(), |p, s| p.with(s, false));
        let mut g = FlashbackTriggerGate::with_posture(TriggerPolicy::default(), posture);
        assert!(matches!(
            g.decide(&CaptureRequest::manual("operator asked"), 0),
            TriggerDecision::Capture { .. }
        ));
        // A monitor verdict lands inside the manual capture's window, so the
        // assertion is that it JOINED rather than that it was refused.
        match g.decide(
            &CaptureRequest::monitor_verdict("stalled:local:/a", "no frames"),
            MS,
        ) {
            TriggerDecision::Extend { .. } => {}
            other => panic!("a monitor verdict must not be gate-refused, got {other:?}"),
        }
        assert_eq!(g.stats().disabled, 0);
    }

    /// The DEFAULT posture is the shipped one, driven through the gate.
    ///
    /// The pure defaults are pinned in `switch`; what this adds is that the gate
    /// built with `new()` really carries them — i.e. that a recorder constructed
    /// the ordinary way admits the shipped default set.
    #[test]
    fn a_default_gate_admits_every_default_on_kind() {
        for (i, kind) in TriggerKind::ALL.into_iter().enumerate() {
            let mut g = gate();
            let request = CaptureRequest {
                kind,
                subject: format!("s{i}"),
                detail: "d".to_string(),
                pin: false,
            };
            assert!(
                matches!(g.decide(&request, 0), TriggerDecision::Capture { .. }),
                "{kind:?} must be admitted by a default gate"
            );
            assert_eq!(g.stats().disabled, 0, "{kind:?}");
        }
    }

    // ---------------------------------------------------------------------
    // Gate state that survives
    // a recorder restart.
    // ---------------------------------------------------------------------

    /// One hour on the wall line, in nanoseconds — the rolling window's horizon.
    const HOUR_NS: u64 = RATE_WINDOW_MS * NS_PER_MS;

    fn history(created_wall_ns: u64, causes: &[(TriggerKind, &str)]) -> CaptureHistory {
        CaptureHistory {
            created_wall_ns,
            causes: causes.iter().map(|(k, s)| (*k, (*s).to_string())).collect(),
        }
    }

    /// THE HEADLINE: a relaunch loop no longer mints a fresh hourly budget.
    ///
    /// Driven as the flood actually arrives. The pre-seed behaviour is asserted
    /// in the SAME body against an unseeded gate under the identical policy, so
    /// the arm cannot pass by the cap being small — it pins the DIFFERENCE the
    /// seed makes.
    #[test]
    fn a_relaunched_recorder_inherits_the_robots_own_rolling_hour() {
        // Eighteen captures in the last hour — the automatic budget, spent by
        // previous lives of this recorder.
        let now_wall = 10 * HOUR_NS;
        let disk: Vec<CaptureHistory> = (0..18)
            .map(|i| {
                history(
                    now_wall - (i + 1) * 60 * 1_000 * NS_PER_MS,
                    &[(TriggerKind::ProcessFault, &format!("graph/p{i}"))],
                )
            })
            .collect();

        // WITHOUT the seed: a fresh process happily captures again. This is the
        // shipped defect, asserted so the arm below is a difference and not a
        // restatement of the cap.
        let mut unseeded = gate();
        assert!(matches!(
            unseeded.decide(&CaptureRequest::process_fault("graph/new", "died"), 0),
            TriggerDecision::Capture { .. }
        ));

        // WITH it: the automatic budget is already spent, and the refusal names
        // the numbers.
        let mut seeded = gate();
        seeded.seed_from_history(&disk, 0, now_wall);
        match seeded.decide(&CaptureRequest::process_fault("graph/new", "died"), 0) {
            TriggerDecision::Suppressed(SuppressReason::RateCapped {
                captures_in_window,
                cap,
                reserved_for_manual,
            }) => assert_eq!((captures_in_window, cap, reserved_for_manual), (18, 18, 2)),
            other => panic!("the inherited window must bind, got {other:?}"),
        }
        // …and the manual reserve still holds across the restart: the operator's two
        // reserved slots survive a flood that predates this process entirely.
        assert!(matches!(
            seeded.decide(&CaptureRequest::manual("the moment I care about"), 0),
            TriggerDecision::Capture { .. }
        ));
    }

    /// A cause that captured just before the relaunch is still inside its floor.
    ///
    /// The refractory half, driven on BOTH sides of the boundary in one body: a
    /// seed that restored a stamp but placed it wrongly on the monotonic line
    /// would pass a one-sided check at whichever end it happened to land.
    #[test]
    fn a_cause_that_captured_before_the_restart_is_still_inside_its_floor() {
        let now_wall = HOUR_NS;
        let floor_ns = TriggerPolicy::default().refractory_ns;
        // Captured 10 s before this process started.
        let ten_s = 10_000 * NS_PER_MS;
        let disk = vec![history(
            now_wall - ten_s,
            &[(TriggerKind::ProcessFault, "graph/p0")],
        )];

        let mut g = gate();
        g.seed_from_history(&disk, 0, now_wall);
        // Inside the floor: refused, and told when to retry — the retry time is
        // the arithmetic that proves the stamp landed on the right instant.
        match g.decide(&CaptureRequest::process_fault("graph/p0", "died"), 0) {
            TriggerDecision::Suppressed(SuppressReason::Refractory { retry_in_ns }) => {
                assert_eq!(retry_in_ns, floor_ns - ten_s)
            }
            other => panic!("a cause inside its inherited floor must be held, got {other:?}"),
        }
        // A DIFFERENT cause is untouched — the seed restores per-cause stamps, not
        // a blanket quiet period.
        assert!(matches!(
            g.decide(&CaptureRequest::process_fault("graph/p1", "died"), 0),
            TriggerDecision::Capture { .. }
        ));
        // …and past the floor the same cause captures again. Driven from a fresh
        // gate so the capture above cannot be what re-armed it.
        let mut g = gate();
        g.seed_from_history(&disk, 0, now_wall);
        assert!(matches!(
            g.decide(
                &CaptureRequest::process_fault("graph/p0", "died"),
                floor_ns - ten_s
            ),
            TriggerDecision::Capture { .. }
        ));
    }

    /// EVERY cause of a coalesced capture seeds a floor, not just the primary.
    ///
    /// Each cause that reached a capture had its regime opened and its floor
    /// stamped, so restoring only the first would leave every coalesced cause
    /// free to re-capture the instant the recorder came back — which is the flood
    /// this closes, one layer down.
    #[test]
    fn every_cause_of_a_coalesced_capture_seeds_its_own_floor() {
        let now_wall = HOUR_NS;
        let disk = vec![history(
            now_wall - 1_000 * NS_PER_MS,
            &[
                (TriggerKind::ProcessFault, "graph/p0"),
                (TriggerKind::MonitorVerdict, "stalled:local:/a"),
                (TriggerKind::MonitorVerdict, "stalled:local:/b"),
            ],
        )];
        let mut g = gate();
        g.seed_from_history(&disk, 0, now_wall);
        for (kind, subject) in [
            (TriggerKind::ProcessFault, "graph/p0"),
            (TriggerKind::MonitorVerdict, "stalled:local:/a"),
            (TriggerKind::MonitorVerdict, "stalled:local:/b"),
        ] {
            let request = CaptureRequest {
                kind,
                subject: subject.to_string(),
                detail: "again".into(),
                pin: false,
            };
            assert!(
                matches!(
                    g.decide(&request, 0),
                    TriggerDecision::Suppressed(SuppressReason::Refractory { .. })
                ),
                "{kind:?}/{subject} must inherit its floor"
            );
        }
    }

    /// The regime `open` bit is NOT restored, and that is the load-bearing
    /// omission.
    ///
    /// A seeded stamp is a RATE statement; an open regime is a claim the
    /// condition is still failing, which a process that has observed nothing
    /// cannot make. If it were restored, the first genuine occurrence after every
    /// restart would be suppressed as `RegimeOpen` — silently, and forever, since
    /// only a recovery edge closes it and a `ProcessFault` has none.
    ///
    /// Pinned by driving the cause PAST its inherited floor: a restored stamp
    /// yields `Capture`, a restored `open` bit yields `RegimeOpen` no matter how
    /// much time passes.
    #[test]
    fn a_seeded_cause_is_floored_but_never_latched() {
        let now_wall = HOUR_NS;
        let disk = vec![history(
            now_wall - 1_000 * NS_PER_MS,
            &[(TriggerKind::MonitorVerdict, "stalled:local:/a")],
        )];
        let mut g = gate();
        g.seed_from_history(&disk, 0, now_wall);
        // A MonitorVerdict has a recovery edge, so an `open` regime is NEVER
        // re-armed by the floor — which makes it the sharpest kind to test with:
        // a restored `open` bit would suppress this forever.
        let request = CaptureRequest::monitor_verdict("stalled:local:/a", "no frames");
        let well_past = TriggerPolicy::default().refractory_ns * 4;
        assert!(
            matches!(
                g.decide(&request, well_past),
                TriggerDecision::Capture { .. }
            ),
            "a seeded cause must be FLOORED, never LATCHED — a restored open \
             regime would swallow this occurrence and every later one"
        );
    }

    /// Captures older than the rolling window seed NOTHING.
    ///
    /// Both sides of the horizon in one body, because either alone reads as the
    /// other's bug: "old captures are ignored" is satisfied by a seed that does
    /// nothing at all.
    #[test]
    fn captures_past_the_rolling_horizon_seed_nothing() {
        let now_wall = 10 * HOUR_NS;
        let policy = TriggerPolicy {
            max_per_hour: 2,
            ..TriggerPolicy::default()
        };

        // AT the horizon — evicted, on the same `>=` the live window uses.
        let mut at = FlashbackTriggerGate::new(policy);
        at.seed_from_history(
            &[history(now_wall - HOUR_NS, &[(TriggerKind::Manual, "")])],
            0,
            now_wall,
        );
        assert!(matches!(
            at.decide(&CaptureRequest::manual("x"), 0),
            TriggerDecision::Capture { .. }
        ));

        // INSIDE it by a millisecond — counted.
        let mut inside = FlashbackTriggerGate::new(policy);
        inside.seed_from_history(
            &[
                history(now_wall - HOUR_NS + NS_PER_MS, &[(TriggerKind::Manual, "")]),
                history(now_wall - HOUR_NS + NS_PER_MS, &[(TriggerKind::Manual, "")]),
            ],
            0,
            now_wall,
        );
        match inside.decide(&CaptureRequest::manual("x"), 0) {
            TriggerDecision::Suppressed(SuppressReason::RateCapped {
                captures_in_window, ..
            }) => assert_eq!(captures_in_window, 2),
            other => panic!("two captures inside the window must bind the cap, got {other:?}"),
        }
    }

    /// A capture stamped in the FUTURE is treated as maximally RECENT.
    ///
    /// A clock stepped backwards since the bag was written makes its age
    /// negative, and the saturating subtraction is what decides which way that
    /// fails. Withholding budget is the conservative direction; the alternative
    /// wraps the age to something enormous, drops the entry, and hands back
    /// rate-cap slots nobody earned — the same reasoning `prune_rate_window`
    /// already carries for its own subtraction.
    #[test]
    fn a_capture_stamped_in_the_future_withholds_budget_rather_than_inventing_it() {
        let now_wall = HOUR_NS;
        let mut g = FlashbackTriggerGate::new(TriggerPolicy {
            max_per_hour: 1,
            ..TriggerPolicy::default()
        });
        g.seed_from_history(
            &[history(now_wall + HOUR_NS, &[(TriggerKind::Manual, "")])],
            0,
            now_wall,
        );
        assert!(
            matches!(
                g.decide(&CaptureRequest::manual("x"), 0),
                TriggerDecision::Suppressed(SuppressReason::RateCapped { .. })
            ),
            "a future-stamped capture must still SPEND its slot"
        );
    }

    /// An unsorted history seeds the LATEST stamp per cause, not the last one
    /// handed in.
    ///
    /// The floor is measured from a cause's most recent capture, and a caller
    /// reading a directory has no ordering guarantee at all — so a plain
    /// assignment would leave the floor keyed on whichever entry `read_dir`
    /// happened to yield last, which is a shorter floor about half the time and
    /// undetectable from the outside.
    #[test]
    fn an_unsorted_history_seeds_the_latest_stamp_per_cause() {
        let now_wall = HOUR_NS;
        let floor_ns = TriggerPolicy::default().refractory_ns;
        let recent = 1_000 * NS_PER_MS;
        let old = 50_000 * NS_PER_MS;
        // NEWEST first — the order that breaks a naive last-write-wins seed.
        let disk = vec![
            history(
                now_wall - recent,
                &[(TriggerKind::ProcessFault, "graph/p0")],
            ),
            history(now_wall - old, &[(TriggerKind::ProcessFault, "graph/p0")]),
        ];
        let mut g = gate();
        g.seed_from_history(&disk, 0, now_wall);
        match g.decide(&CaptureRequest::process_fault("graph/p0", "died"), 0) {
            TriggerDecision::Suppressed(SuppressReason::Refractory { retry_in_ns }) => {
                assert_eq!(
                    retry_in_ns,
                    floor_ns - recent,
                    "the floor must run from the NEWEST capture of this cause"
                )
            }
            other => panic!("expected the inherited floor, got {other:?}"),
        }
    }

    /// An empty history is a no-op — an unreadable directory must leave a robot
    /// capturing exactly as it did before.
    #[test]
    fn an_empty_history_leaves_the_gate_untouched() {
        let mut g = gate();
        g.seed_from_history(&[], 0, HOUR_NS);
        assert_eq!(g.stats(), TriggerStats::default());
        assert!(matches!(
            g.decide(&CaptureRequest::process_fault("graph/p0", "died"), 0),
            TriggerDecision::Capture { seq: 0, .. }
        ));
    }
}
