// SPDX-License-Identifier: AGPL-3.0-only
//! The per-trigger posture: which of the default-on
//! triggers a particular robot may turn off.
//!
//! PURE: it maps a name to an environment variable and a lookup to a decision. No
//! `std::env` read, no clock, no transport — the caller supplies the lookup, so
//! every arm is oracle-testable without mutating process state.
//!
//! # This REVERSES a documented stance, and the reversal is the point
//!
//! `verdict_observer`'s "What is deliberately NOT here" says, in as many words:
//! no new CLI flag, YAML key or env var. That was the right call while the
//! monitors triggers were the only automatic ones and the kill switch plus the
//! rate cap were the whole posture surface. The per-trigger posture changes it, for a
//! reason the earlier stance could not have: `silent` is now demoted from
//! default-ON, and a demotion with no way back is a REMOVAL. A robot whose
//! never-produced routes really are the thing worth capturing — the shape `silent`
//! was built for — would have no way to say so.
//!
//! So the surface is the smallest one that can express "this row, on this robot":
//! one boolean environment variable per switchable row, read through the SAME
//! [`parse_plane_switch`] the kill switch uses, so an
//! unrecognised value is REPORTED and the row keeps its documented default rather
//! than being silently guessed at. A list-valued knob was the alternative and is
//! worse on exactly that axis: a typo inside a comma-separated list is silently
//! inert, and "silently inert" is the failure mode this whole plane is built to
//! avoid.
//!
//! # WHERE a switch is enforced, and why it is not one place
//!
//! Four of the seven rows are a whole [`TriggerKind`], and three of them can arrive
//! at the recorder OVER THE CHANNEL from another process — a supervisor publishing
//! a worker death, a `cerud` handler publishing an e-stop, a user's own detector
//! publishing a declared incident. Those processes have their own environments and
//! may not even be able to read this robot's; so a kind-level switch is enforced by
//! the GATE ([`FlashbackTriggerGate::decide`](super::trigger::FlashbackTriggerGate::decide)),
//! which is the one place every request passes through.
//!
//! The other three rows are monitor CONDITIONS. They share
//! [`TriggerKind::MonitorVerdict`] — three conditions, one detector, one record
//! book — so the gate cannot tell them apart without parsing the subject's first
//! chunk, and building a switch on a string grammar the monitors engine owns would
//! couple the gate to a vocabulary that has to be free to move. They are therefore
//! enforced at the MINT SITE (`verdict_observer`), which holds the `Alert` and
//! knows the condition first-hand. That path is in-process and single-caller, so
//! there is no second producer for the gate to backstop.
//!
//! Stated as a rule: **the gate refuses what can arrive from elsewhere; the mint
//! site refuses what only it can name.** [`TriggerSwitch::for_kind`] is the whole
//! of the first half and returns `None` for exactly the two kinds the gate must not
//! judge.
//!
//! # Manual is NOT switchable, deliberately
//!
//! `cerulion flashback` is a verb an operator types and waits on. An environment
//! variable that made it silently do nothing would be a verb that lies about
//! having run — the class
//! [`TriggerKind::is_automatic`](super::trigger::TriggerKind::is_automatic) already
//! refuses for the latch and the floor, applied to posture. An operator who wants
//! no captures at all has [`FLASHBACK_ENV`](super::FLASHBACK_ENV)`=off`, which
//! turns the plane off outright and says so.

use super::trigger::TriggerKind;
use super::{parse_plane_switch, PlaneSwitch};
use crate::monitor::MonitorCondition;

/// The prefix every per-trigger switch shares.
pub const FLASHBACK_ON_PREFIX: &str = "CERULION_FLASHBACK_ON_";

/// One switchable trigger row.
///
/// Rows, not [`TriggerKind`]s: the three monitor conditions share a kind and need
/// three switches, while `Manual` is a kind with no switch at all. The two
/// vocabularies are deliberately different shapes and folding them would force one
/// of those cases to be wrong.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum TriggerSwitch {
    /// A worker process died — [`TriggerKind::ProcessFault`].
    WorkerDeath,
    /// A node was disabled after consecutive panics — [`TriggerKind::PanicDisable`].
    PanicDisable,
    /// The bound run vanished — [`TriggerKind::RunVanished`].
    RunVanished,
    /// A human engaged the e-stop — [`TriggerKind::EStop`].
    EStop,
    /// A robot-side process declared an incident — [`TriggerKind::Declared`].
    Declared,
    /// Monitor condition `stalled` — process alive, data dead.
    Stalled,
    /// Monitor condition `rate_deviation` — a rate collapse inside the window.
    RateDeviation,
    /// Monitor condition `silent` — a registered route that never produced.
    ///
    /// The one row that ships off, by design. See
    /// [`TriggerSwitch::default_on`].
    Silent,
}

impl TriggerSwitch {
    /// Every switch, in declaration order.
    ///
    /// Walked rather than restated by the tests, so a new row cannot ship without
    /// a suffix, a default and a posture entry.
    pub const ALL: [TriggerSwitch; 8] = [
        Self::WorkerDeath,
        Self::PanicDisable,
        Self::RunVanished,
        Self::EStop,
        Self::Declared,
        Self::Stalled,
        Self::RateDeviation,
        Self::Silent,
    ];

    /// The suffix this switch's environment variable carries after
    /// [`FLASHBACK_ON_PREFIX`].
    ///
    /// Also the token a [`SuppressReason::Disabled`](super::trigger::SuppressReason::Disabled)
    /// refusal names, so the thing an operator reads in a log line is the thing
    /// they type — the one-spelling rule applied to a knob.
    pub fn env_suffix(self) -> &'static str {
        match self {
            Self::WorkerDeath => "WORKER_DEATH",
            Self::PanicDisable => "PANIC_DISABLE",
            Self::RunVanished => "RUN_VANISHED",
            Self::EStop => "ESTOP",
            Self::Declared => "DECLARED",
            Self::Stalled => "STALL",
            Self::RateDeviation => "RATE",
            Self::Silent => "SILENT",
        }
    }

    /// This switch's full environment variable name.
    pub fn env_var(self) -> String {
        format!("{FLASHBACK_ON_PREFIX}{}", self.env_suffix())
    }

    /// This switch's wire byte, for the `Disabled` suppression verdict.
    ///
    /// Bytes are APPENDED and never re-used, for the same reason
    /// [`TriggerKind`]'s are: a re-used byte silently renames a refusal, and the
    /// refusal's whole job is to name the switch an operator must go and change.
    pub fn as_wire_byte(self) -> u8 {
        match self {
            Self::WorkerDeath => 1,
            Self::PanicDisable => 2,
            Self::RunVanished => 3,
            Self::EStop => 4,
            Self::Declared => 5,
            Self::Stalled => 6,
            Self::RateDeviation => 7,
            Self::Silent => 8,
        }
    }

    /// [`Self::as_wire_byte`]'s exact inverse; `None` for a byte this build does
    /// not know.
    pub fn from_wire_byte(byte: u8) -> Option<Self> {
        Self::ALL.into_iter().find(|s| s.as_wire_byte() == byte)
    }

    /// Whether this row is ON when nobody says otherwise.
    ///
    /// # `Silent` is the one `false`, and it is a change to SHIPPED behaviour
    ///
    /// `silent` fires when a registered route never produced
    /// this run, which on an attach-shaped graph is a description of the graph
    /// rather than of an incident: that graph shape registers a publisher per
    /// discovered route at build time, so a route that legitimately never carries
    /// data reaches `NoData` and confirms — one ~155 MB capture per healthy run,
    /// roughly twelve seconds after launch, documenting an ABSENCE whose cause
    /// predates the window it captured.
    ///
    /// The cost of the demotion is real and is not hidden: never-produced routes
    /// and topics below `MONITOR_STALL_MIN_MHZ` leave the capture plane entirely,
    /// because `stalled` structurally cannot see them (it watches a transition FROM
    /// `Streaming`, which they never reach). They keep the STATUS plane, which is
    /// where an absence belongs — `cerulion topic list` and the monitors surface
    /// both still report them.
    ///
    /// It is a DEMOTION rather than a deletion precisely so a robot for which those
    /// routes are the interesting ones can say `CERULION_FLASHBACK_ON_SILENT=on`.
    pub fn default_on(self) -> bool {
        match self {
            Self::WorkerDeath
            | Self::PanicDisable
            | Self::RunVanished
            | Self::EStop
            | Self::Declared
            | Self::Stalled
            | Self::RateDeviation => true,
            Self::Silent => false,
        }
    }

    /// PURE: the switch a whole [`TriggerKind`] answers to, if any.
    ///
    /// `None` for exactly two kinds, for two different reasons, and neither is an
    /// oversight:
    ///
    /// - [`TriggerKind::Manual`] has no switch — see the module docs.
    /// - [`TriggerKind::MonitorVerdict`] carries THREE switchable rows inside one
    ///   kind, so answering with any one of them would refuse the other two along
    ///   with it. Its posture is applied at the mint site, which knows the
    ///   condition.
    ///
    /// A caller that treats `None` as "refuse" would silently disable the manual
    /// verb and every monitor verdict; a caller that treats it as "allow" is
    /// correct, which is why the gate's arm is written that way round.
    pub fn for_kind(kind: TriggerKind) -> Option<Self> {
        match kind {
            TriggerKind::Manual | TriggerKind::MonitorVerdict => None,
            TriggerKind::ProcessFault => Some(Self::WorkerDeath),
            TriggerKind::PanicDisable => Some(Self::PanicDisable),
            TriggerKind::RunVanished => Some(Self::RunVanished),
            TriggerKind::EStop => Some(Self::EStop),
            TriggerKind::Declared => Some(Self::Declared),
        }
    }

    /// PURE: the switch one monitor CONDITION answers to.
    ///
    /// TOTAL — every condition has a switch, which is what makes the mint-site
    /// half of the split exhaustive: a new monitor condition cannot ship with no
    /// posture, because this match would not compile.
    pub fn for_condition(condition: MonitorCondition) -> Self {
        match condition {
            MonitorCondition::Stalled => Self::Stalled,
            MonitorCondition::Silent => Self::Silent,
            MonitorCondition::RateDeviation => Self::RateDeviation,
        }
    }
}

/// Which triggers are ON, resolved once and then fixed for the process's life.
///
/// Resolved ONCE for the same reason [`TriggerPolicy`](super::trigger::TriggerPolicy)
/// is: a posture that could change mid-run would make a suppression depend on when
/// it was asked, and a capture that happened would be unexplainable from the
/// environment a reader can see afterwards.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TriggerPosture {
    /// Indexed by [`TriggerSwitch::ALL`]'s order.
    on: [bool; TriggerSwitch::ALL.len()],
}

impl Default for TriggerPosture {
    /// Every row at its documented default — the posture a robot with none of these
    /// variables set is in.
    fn default() -> Self {
        let mut on = [true; TriggerSwitch::ALL.len()];
        for (slot, switch) in on.iter_mut().zip(TriggerSwitch::ALL) {
            *slot = switch.default_on();
        }
        Self { on }
    }
}

impl TriggerPosture {
    /// PURE: resolve every switch through `lookup`, plus whatever it could not
    /// honour.
    ///
    /// `lookup` takes the FULL variable name and answers what the environment
    /// holds. Taking a lookup rather than reading `std::env` here is what keeps
    /// this testable in parallel: `set_var` is process-global and would make every
    /// arm `#[serial]`, which is how a posture test ends up racing an unrelated
    /// one.
    ///
    /// A complaint is a LINE, never a failure. An unreadable posture must not stop
    /// a robot capturing, and must not be silently ignored either — the same
    /// contract [`resolve_cadence_ms`](super::resolve_cadence_ms) has, through the
    /// same parser.
    ///
    /// # SILENCE means two DIFFERENT things one layer apart, and conflating them
    /// was a shipped defect
    ///
    /// [`parse_plane_switch`] is a KILL-SWITCH parser: its subject is
    /// [`FLASHBACK_ENV`](super::FLASHBACK_ENV), one variable governing whether a
    /// safety feature runs at all, and for that subject `None` correctly means
    /// [`On`](PlaneSwitch::On) — the feature SHIPS on, so an operator who has never
    /// heard of the variable gets it. Absence there is "nobody reached for the kill
    /// switch".
    ///
    /// A PER-ROW switch is a PREFERENCE, and its default is written down per row by
    /// [`TriggerSwitch::default_on`] — currently `on` for seven rows and `off` for
    /// `Silent`. Absence here is "the operator expressed no
    /// preference for THIS row", which must resolve to that row's own default, NOT
    /// to the kill switch's ships-on answer.
    ///
    /// So absence and an empty value are handled HERE, before the parser is
    /// consulted, and only a value an operator really typed reaches it. Routing an
    /// unset variable through `parse_plane_switch` is exactly the regression this
    /// paragraph exists to prevent: every row came back `On`, the `Self::default()`
    /// seed below became dead code, and the `silent` demotion was inert in
    /// production (a deployment with no Flashback variables set fired an 82 MB capture
    /// eleven seconds after boot on 32 `silent:local:*` verdicts).
    ///
    /// `parse_plane_switch` itself is deliberately UNCHANGED — its absent arm is
    /// correct for the subject it was written for, and re-pointing it would silently
    /// disable Flashback on every robot that has never set `CERULION_FLASHBACK`.
    pub fn resolve<F>(mut lookup: F) -> (Self, Vec<String>)
    where
        F: FnMut(&str) -> Option<String>,
    {
        let mut posture = Self::default();
        let mut complaints = Vec::new();
        for (idx, switch) in TriggerSwitch::ALL.into_iter().enumerate() {
            let var = switch.env_var();
            let raw = lookup(&var);
            // The operator said NOTHING about this row — unset, or set to what a
            // shell script that computed no value leaves behind. Keep the seeded
            // default and consult nothing. (See the divergence note above.)
            let said_something = raw.as_deref().is_some_and(|v| !v.trim().is_empty());
            if !said_something {
                continue;
            }
            match parse_plane_switch(raw.as_deref()) {
                PlaneSwitch::On => posture.on[idx] = true,
                PlaneSwitch::Off => posture.on[idx] = false,
                PlaneSwitch::Unrecognized(value) => {
                    // The row KEEPS ITS DEFAULT rather than taking either
                    // reading. Defaulting an unreadable value to ON would arm a
                    // trigger an operator believes they disabled; defaulting it to
                    // OFF would disable one they believe is armed. Neither is
                    // defensible, so the answer is "this said nothing" plus a line
                    // that quotes what they typed.
                    complaints.push(format!(
                        "{var}={value:?} is not a recognised on/off value, so the \
                         {} trigger keeps its default ({}). Use `on` or `off`",
                        switch.env_suffix(),
                        if switch.default_on() { "on" } else { "off" },
                    ));
                }
            }
        }
        (posture, complaints)
    }

    /// PURE: resolve from the process environment.
    ///
    /// The one impure entry point, kept to a single line so everything above it
    /// stays oracle-testable.
    pub fn from_env() -> (Self, Vec<String>) {
        Self::resolve(|var| std::env::var(var).ok())
    }

    /// Is this row ON?
    pub fn is_on(&self, switch: TriggerSwitch) -> bool {
        let idx = TriggerSwitch::ALL
            .iter()
            .position(|s| *s == switch)
            .expect("every switch is in ALL");
        self.on[idx]
    }

    /// PURE: the switch REFUSING a request of this kind, if one is.
    ///
    /// The gate's arm and [`Self::kind_allowed`] are the same question asked for
    /// two different answers — a refusal payload and a boolean — so they are ONE
    /// function with two readings rather than two copies of
    /// `for_kind(..)` + `is_on(..)`. That is the two-copies rule applied to
    /// a predicate, and it is not hypothetical here: written twice, a mutation of
    /// either copy leaves the other's tests green, so half the vocabulary's
    /// coverage would be pinning a function production never calls.
    ///
    /// `None` — "nothing refuses it" — for the two kinds with no kind-level
    /// switch; see [`TriggerSwitch::for_kind`] for why that is the correct
    /// direction.
    pub fn refusing_switch(&self, kind: TriggerKind) -> Option<TriggerSwitch> {
        TriggerSwitch::for_kind(kind).filter(|s| !self.is_on(*s))
    }

    /// PURE: may a request of this KIND be decided at all?
    pub fn kind_allowed(&self, kind: TriggerKind) -> bool {
        self.refusing_switch(kind).is_none()
    }

    /// Turn one row off. Test seam and programmatic override; production posture
    /// comes from [`Self::from_env`].
    #[must_use]
    pub fn with(mut self, switch: TriggerSwitch, on: bool) -> Self {
        let idx = TriggerSwitch::ALL
            .iter()
            .position(|s| *s == switch)
            .expect("every switch is in ALL");
        self.on[idx] = on;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The whole vocabulary against a HAND-WRITTEN table.
    ///
    /// Restating the mapping is the point: a table derived from the code under test
    /// would agree with any rename, and these strings are an operator-facing
    /// surface (they are typed into a shell and read out of a log line).
    #[test]
    fn every_switch_carries_the_env_name_and_default_it_documents() {
        let oracle: &[(TriggerSwitch, &str, bool)] = &[
            (TriggerSwitch::WorkerDeath, "WORKER_DEATH", true),
            (TriggerSwitch::PanicDisable, "PANIC_DISABLE", true),
            (TriggerSwitch::RunVanished, "RUN_VANISHED", true),
            (TriggerSwitch::EStop, "ESTOP", true),
            (TriggerSwitch::Declared, "DECLARED", true),
            (TriggerSwitch::Stalled, "STALL", true),
            (TriggerSwitch::RateDeviation, "RATE", true),
            // The one demotion, and the reason this test
            // spells every default out rather than asserting "all true".
            (TriggerSwitch::Silent, "SILENT", false),
        ];
        assert_eq!(
            oracle.len(),
            TriggerSwitch::ALL.len(),
            "a switch was added without an oracle row"
        );
        for (switch, suffix, default_on) in oracle {
            assert_eq!(switch.env_suffix(), *suffix, "{switch:?} suffix");
            assert_eq!(
                switch.env_var(),
                format!("CERULION_FLASHBACK_ON_{suffix}"),
                "{switch:?} variable"
            );
            assert_eq!(switch.default_on(), *default_on, "{switch:?} default");
        }
    }

    /// The default posture IS the documented default set — {manual, worker death,
    /// stalled, rate_deviation} plus the three later additions, with `silent`
    /// demoted.
    #[test]
    fn the_default_posture_is_ruling_110_as_set() {
        let posture = TriggerPosture::default();
        for switch in TriggerSwitch::ALL {
            assert_eq!(
                posture.is_on(switch),
                switch.default_on(),
                "{switch:?} default posture"
            );
        }
        assert!(!posture.is_on(TriggerSwitch::Silent), "silent is demoted");
        assert!(posture.is_on(TriggerSwitch::Stalled), "stalled is kept");
        assert!(
            posture.is_on(TriggerSwitch::RateDeviation),
            "rate_deviation is kept"
        );
    }

    /// Every kind maps to the switch that governs it, and the two that must map to
    /// NOTHING do.
    #[test]
    fn only_the_kinds_a_second_process_can_publish_carry_a_kind_level_switch() {
        assert_eq!(TriggerSwitch::for_kind(TriggerKind::Manual), None);
        assert_eq!(TriggerSwitch::for_kind(TriggerKind::MonitorVerdict), None);
        assert_eq!(
            TriggerSwitch::for_kind(TriggerKind::ProcessFault),
            Some(TriggerSwitch::WorkerDeath)
        );
        assert_eq!(
            TriggerSwitch::for_kind(TriggerKind::PanicDisable),
            Some(TriggerSwitch::PanicDisable)
        );
        assert_eq!(
            TriggerSwitch::for_kind(TriggerKind::RunVanished),
            Some(TriggerSwitch::RunVanished)
        );
        assert_eq!(
            TriggerSwitch::for_kind(TriggerKind::EStop),
            Some(TriggerSwitch::EStop)
        );
        assert_eq!(
            TriggerSwitch::for_kind(TriggerKind::Declared),
            Some(TriggerSwitch::Declared)
        );
    }

    /// A kind with NO switch is ALLOWED, never refused.
    ///
    /// The direction is the whole assertion: reading `None` as "refuse" would
    /// silently disable `cerulion flashback` and every monitor verdict on every
    /// robot, which is the single worst regression this vocabulary could ship.
    #[test]
    fn a_kind_with_no_switch_is_allowed_rather_than_refused() {
        let posture = TriggerPosture::default();
        assert!(posture.kind_allowed(TriggerKind::Manual));
        assert!(posture.kind_allowed(TriggerKind::MonitorVerdict));
        // …and turning every switch OFF does not change that, so the answer cannot
        // be an accident of the defaults.
        let all_off = TriggerSwitch::ALL
            .into_iter()
            .fold(posture, |p, s| p.with(s, false));
        assert!(all_off.kind_allowed(TriggerKind::Manual));
        assert!(all_off.kind_allowed(TriggerKind::MonitorVerdict));
        assert!(!all_off.kind_allowed(TriggerKind::EStop));
    }

    /// `off` disables, `on` enables, and BOTH directions are reachable — the
    /// demoted row can be turned back on and a default-on row can be turned off.
    #[test]
    fn a_switch_moves_its_row_in_both_directions() {
        let (posture, complaints) = TriggerPosture::resolve(|var| match var {
            "CERULION_FLASHBACK_ON_SILENT" => Some("on".to_string()),
            "CERULION_FLASHBACK_ON_STALL" => Some("off".to_string()),
            _ => None,
        });
        assert!(complaints.is_empty(), "{complaints:?}");
        assert!(posture.is_on(TriggerSwitch::Silent), "silent opted back in");
        assert!(!posture.is_on(TriggerSwitch::Stalled), "stalled opted out");
        // Untouched rows keep their documented defaults.
        assert!(posture.is_on(TriggerSwitch::RateDeviation));
        assert!(posture.is_on(TriggerSwitch::EStop));
    }

    /// An unrecognised value keeps the DEFAULT and complains, naming the offender.
    ///
    /// Both halves matter: taking the value as OFF would disable a trigger nobody
    /// asked to disable, and taking it as ON would arm one an operator believes
    /// they turned off. Neither is the answer, so the row does not move.
    #[test]
    fn an_unrecognised_value_keeps_the_default_and_says_so() {
        let (posture, complaints) = TriggerPosture::resolve(|var| match var {
            // A default-ON row, so an "unrecognised means off" bug is visible.
            "CERULION_FLASHBACK_ON_ESTOP" => Some("of".to_string()),
            // A default-OFF row, so an "unrecognised means on" bug is visible too.
            "CERULION_FLASHBACK_ON_SILENT" => Some("yes-please".to_string()),
            _ => None,
        });
        assert!(
            posture.is_on(TriggerSwitch::EStop),
            "estop keeps its ON default"
        );
        assert!(
            !posture.is_on(TriggerSwitch::Silent),
            "silent keeps its OFF default"
        );
        assert_eq!(complaints.len(), 2, "{complaints:?}");
        let estop = complaints
            .iter()
            .find(|c| c.contains("ESTOP"))
            .expect("the estop complaint");
        assert!(estop.contains("\"of\""), "quotes what was typed: {estop}");
        assert!(
            estop.contains("(on)"),
            "names the default in force: {estop}"
        );
        let silent = complaints
            .iter()
            .find(|c| c.contains("SILENT"))
            .expect("the silent complaint");
        assert!(
            silent.contains("(off)"),
            "names the default in force: {silent}"
        );
    }

    /// An EMPTY ENVIRONMENT is the shipped defaults, row by row.
    ///
    /// This is the arm the regression walked straight through. `resolve` seeded
    /// [`Self::default`] and then handed EVERY variable — set or not — to
    /// [`parse_plane_switch`], whose absent arm answers `On` because it is a KILL
    /// SWITCH parser. So every row was forced ON, the seed was dead code, and
    /// the `silent` demotion was inert in production: a deployment
    /// with no Flashback variables set fired an 82 MB capture eleven seconds after
    /// boot on 32 `silent:local:*` verdicts.
    ///
    /// Asserted PER ROW rather than by struct equality, so a failure names the row
    /// that moved instead of printing two opaque bit arrays.
    #[test]
    fn an_empty_environment_resolves_to_the_shipped_defaults() {
        let (posture, complaints) = TriggerPosture::resolve(|_| None);
        assert!(complaints.is_empty(), "{complaints:?}");
        for switch in TriggerSwitch::ALL {
            assert_eq!(
                posture.is_on(switch),
                switch.default_on(),
                "{switch:?} must keep its documented default when nobody sets its \
                 variable"
            );
        }
        assert_eq!(
            posture,
            TriggerPosture::default(),
            "an unset environment IS the default posture"
        );
    }

    /// An EMPTY value is not an instruction — it is what a shell script that
    /// computed nothing leaves behind, so the row keeps its own default.
    ///
    /// This arm's oracle was CORRECTED. It used to record that an empty value
    /// read as ON (`parse_plane_switch`'s kill-switch answer) and asserted only the
    /// already-on row, which made it agree with the regression: a `silent` forced ON
    /// by an empty string was the documented behaviour here. Empty and ABSENT are
    /// the same statement — the operator said nothing — so both keep the default.
    #[test]
    fn an_empty_value_is_not_an_instruction() {
        let (posture, complaints) = TriggerPosture::resolve(|var| match var {
            "CERULION_FLASHBACK_ON_SILENT" => Some(String::new()),
            "CERULION_FLASHBACK_ON_STALL" => Some("   ".to_string()),
            "CERULION_FLASHBACK_ON_ESTOP" => Some("\t\n".to_string()),
            _ => None,
        });
        assert!(complaints.is_empty(), "{complaints:?}");
        assert!(
            !posture.is_on(TriggerSwitch::Silent),
            "an empty string is not an instruction to arm a demoted row"
        );
        assert!(posture.is_on(TriggerSwitch::Stalled));
        assert!(posture.is_on(TriggerSwitch::EStop));
        assert_eq!(
            posture,
            TriggerPosture::default(),
            "whitespace-only values leave the whole posture at its defaults"
        );
    }

    /// An explicit `on` ARMS the one demoted row and touches nothing else.
    ///
    /// The escape hatch the `silent` demotion exists for: a robot whose
    /// never-produced routes really are the interesting ones says
    /// `CERULION_FLASHBACK_ON_SILENT=on`. The every-other-row half is what stops a
    /// "set them all" regression passing.
    #[test]
    fn an_explicit_on_arms_a_demoted_row() {
        let (posture, complaints) = TriggerPosture::resolve(|var| {
            (var == "CERULION_FLASHBACK_ON_SILENT").then(|| "on".to_string())
        });
        assert!(complaints.is_empty(), "{complaints:?}");
        assert!(posture.is_on(TriggerSwitch::Silent), "silent opted back in");
        for switch in TriggerSwitch::ALL {
            if switch == TriggerSwitch::Silent {
                continue;
            }
            assert_eq!(
                posture.is_on(switch),
                switch.default_on(),
                "{switch:?} must be untouched by another row's variable"
            );
        }
    }

    /// An explicit `off` DISARMS a default-on row without arming the demoted one.
    #[test]
    fn an_explicit_off_disarms_a_default_on_row() {
        let (posture, complaints) = TriggerPosture::resolve(|var| {
            (var == "CERULION_FLASHBACK_ON_STALL").then(|| "off".to_string())
        });
        assert!(complaints.is_empty(), "{complaints:?}");
        assert!(!posture.is_on(TriggerSwitch::Stalled), "stalled opted out");
        assert!(
            !posture.is_on(TriggerSwitch::Silent),
            "silent stays demoted — another row's `off` says nothing about it"
        );
        assert!(posture.is_on(TriggerSwitch::RateDeviation));
    }
}
