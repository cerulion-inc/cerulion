// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion bag play --resim` **surface** over the replay
//! engine.
//!
//! # What this module is, and what it is not
//!
//! It is a **verb layer**. Every byte of re-execution, injection, capture and
//! byte-comparison still happens in [`crate::replay_cmd::run_replay`] and
//! [`crate::replay_engine`], which this surface does not touch. What moves here is
//! the *surface*: which flags are legal together, what the exit code means, and
//! what the operator is told when no verdict was asked for.
//!
//! # The two modes
//!
//! `--resim all` alone is **NEUTRAL**: it re-executes the bagged graph against
//! the CURRENT workspace build and reports what happened, but it makes **no
//! claim about whether the result matches the recording**. Divergence is the
//! product, not a failure — the agent-iteration loop (edit → build → resim →
//! score) consumes exactly this — so a completed re-execution is always exit 0.
//!
//! `--verify` turns it into the verifier: the byte-comparison verdict
//! renders and the stable 0/1/2/3/4/5/6 exit taxonomy applies unchanged.
//!
//! **Neutral is not silent, and it is not forgiving of failure.** "No verdict"
//! scopes exactly the two *comparison* outcomes — data violations (exit 1) and
//! the structural trace divergence (exit 6). Everything that means the
//! re-execution could not be performed keeps its loud exit: an unreadable or
//! not-replay-grade bag (2), a cdylib that would not load (3), a candidate node
//! that PANICKED mid-run (3), an internal error (5). A neutral resim that
//! swallowed those would report "exit 0" for a run that never happened, which
//! is the silent-failure class this repo has a rule against.
//!
//! # Reserved, deliberately not foreclosed
//!
//! `--record-out PATH` (capture the re-executed run's own output to a fresh,
//! chainable bag) and `--score-topic TOPIC[:FIELD]` (a scalar trajectory over a
//! re-executed topic) are RESERVED fork-mode flags. They compose on the NEUTRAL
//! dial — they are the answer to "what did a no-verdict resim produce" — and
//! neither name, short flag, nor value shape is taken by anything here.
//! `--resim <node,...>` (a partial cut) is likewise reserved and refused by name
//! today; see [`ResimSelection::parse`].
//!
//! # Platform gating — deliberately NOT inherited from the callee
//!
//! Almost everything here is platform-independent: parsing a selection, judging
//! a flag combination, classifying an exit code, rendering a summary. Only
//! [`run_resim`] and [`run_play_resim`] reach [`crate::replay_cmd`], which is
//! `#[cfg(unix)]` because the bag reader is — so ONLY those two carry the cfg.
//!
//! Inheriting `#[cfg(unix)]` wholesale from the callee
//! would break the non-Unix build outright (E0433): `main` names
//! [`resolve_play_mode`], [`PlayFlags`] and [`EXIT_USAGE`] UNCONDITIONALLY, to
//! refuse a malformed `--resim` invocation above the login gate, and nothing
//! about judging a flag combination is Unix-only. Hence [`ResimOptions`], a
//! platform-independent mirror of `replay_cmd::ReplayOptions` converted only
//! inside the gated half, and the exit-code constants re-declared here with a
//! `#[cfg(unix)]` compile-time equality assertion against `replay_cmd`'s so
//! they cannot drift.
//!
//! (The pre-auth usage refusal is the ungated reference that carries the
//! invariant. The pin does not care which
//! item does — it compares the two sides' `cfg`s, whatever they name.)
//!
//! `crates/cerulion_cli/src/cfg_symmetry_tests.rs` pins the split structurally: an
//! item that grows a `#[cfg(unix)]` here without `main` gaining a matching gate
//! fails a test, because a Unix host never compiles the non-Unix target.

use std::path::{Path, PathBuf};

#[cfg(unix)]
use crate::replay_cmd;
#[cfg(unix)]
use crate::replay_engine::ReplayOutcome;

/// Exit code 0 — the re-execution completed (and, under `--verify`, matched).
pub const EXIT_PASS: u8 = 0;
/// Exit code 1 — a data violation. `--verify` only.
pub const EXIT_VIOLATION: u8 = 1;
/// Exit code 3 — a node failed: cdylib load, or a PANIC-class execution
/// failure. Reported in BOTH modes.
pub const EXIT_NODE_FAILURE: u8 = 3;
/// Exit code 6 — the re-executed fire schedule diverged. `--verify` only.
pub const EXIT_TRACE_DIVERGENCE: u8 = 6;

// The codes above are re-declared rather than re-exported because this module
// is platform-independent while `replay_cmd` is `#[cfg(unix)]` (see the module
// docs). Re-declaring risks drift, so on the platform where BOTH exist the
// compiler proves they agree — a const assertion, evaluated at compile time,
// with no runtime cost and no way to silence it.
#[cfg(unix)]
const _: () = {
    assert!(EXIT_PASS == replay_cmd::EXIT_PASS);
    assert!(EXIT_VIOLATION == replay_cmd::EXIT_VIOLATION);
    assert!(EXIT_NODE_FAILURE == replay_cmd::EXIT_NODE_FAILURE);
    assert!(EXIT_TRACE_DIVERGENCE == replay_cmd::EXIT_TRACE_DIVERGENCE);
};

/// The knobs a resim threads into the engine — a platform-independent mirror of
/// `replay_cmd::ReplayOptions`, converted into it inside [`run_resim`] (the one
/// `#[cfg(unix)]` seam). Without the mirror the whole flag resolver would
/// inherit the bag reader's cfg, and with it the removal notice.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct ResimOptions {
    /// `--duration D` in NANOSECONDS of bag time — stop re-executing once a
    /// recorded boundary's clock target passes the run's bag-time ORIGIN plus
    /// this. The origin is the MINIMUM first-boundary target across the ranks,
    /// so one bound names one wall interval for all of them rather than each
    /// rank's own first D seconds.
    pub duration_bound_ns: Option<u64>,
    /// `--report PATH` (requires `--verify`).
    pub report_path: Option<PathBuf>,
    /// `--tolerance PATH` (requires `--verify`).
    pub tolerance_path: Option<PathBuf>,
    /// `--strict-state` — refuse the re-execution unless every executed node's
    /// state restored (carried across from `cerulion replay`).
    pub strict_state: bool,
}

/// Which nodes re-execute.
///
/// Only [`ResimSelection::All`] is buildable today. A partial cut is not
/// supported (it needs the three-bucket topic partition and the soundness
/// preflight), so a subset spelling is REFUSED by name rather than silently
/// widened to `all` — a resim that quietly re-executed the whole graph when you
/// asked for one node would report a verdict about code you did not select.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ResimSelection {
    /// Every node in the bagged graph re-executes — the whole-graph
    /// re-execution, reached through the new spelling.
    All,
}

impl ResimSelection {
    /// Parse the `--resim` value. `all` (exactly, lowercase) is the only
    /// accepted spelling; anything else is a loud refusal naming the gap and
    /// the value that IS accepted.
    pub fn parse(raw: &str) -> Result<Self, String> {
        if raw == "all" {
            return Ok(Self::All);
        }
        Err(format!(
            "`--resim {raw}` selects a SUBSET of the graph's nodes, which is not built \
             yet: only the whole-graph re-execution ships today. Use `--resim all` to \
             re-execute every node in the bag's graph."
        ))
    }
}

/// The raw `bag play` flag surface, as clap parsed it, before any legality
/// check. Presence — not value — is what the refusals key on, which is why
/// `rate` is an `Option` rather than a defaulted `f64`: clap's `default_value_t`
/// erases the difference between "the user asked for 1.0" and "the user asked
/// for nothing".
#[derive(Debug, Clone, Default)]
pub struct PlayFlags {
    /// `--resim <NODES|all>`.
    pub resim: Option<String>,
    /// `--verify`.
    pub verify: bool,
    /// `--rate N` — playback only.
    pub rate: Option<f64>,
    /// `--loop` — playback only.
    pub repeat: bool,
    /// `--topics TOPIC` — playback only.
    pub topics: Vec<String>,
    /// `--duration D` (SECONDS of bag time) — legal in BOTH halves: it bounds
    /// what EXECUTES / what is republished, never a verdict.
    pub duration: Option<f64>,
    /// `--start-offset S` (SECONDS of bag time) — playback only.
    pub start_offset: Option<f64>,
    /// `--report PATH` — resim + `--verify` only.
    pub report: Option<PathBuf>,
    /// `--tolerance PATH` — resim + `--verify` only.
    pub tolerance: Option<PathBuf>,
    /// `--strict-state` — resim only, in EITHER mode.
    pub strict_state: bool,
}

/// What a legal `bag play` invocation resolved to.
#[derive(Debug, Clone, PartialEq)]
pub enum PlayMode {
    /// Verbatim frame playback: the existing `bag play` verb, unchanged.
    Playback {
        /// The pace multiplier, defaulted here rather than by clap so presence
        /// stays observable at the refusal seam.
        rate: f64,
        /// `--loop`.
        repeat: bool,
        /// `--topics`.
        topics: Vec<String>,
        /// `--duration D` in NANOSECONDS of BAG TIME. Playback
        /// stops republishing once a channel has advanced that far through its
        /// OWN recorded timeline. `None` plays the whole bag.
        duration_bound_ns: Option<u64>,
        /// `--start-offset S` in NANOSECONDS of BAG TIME — skip
        /// the leading frames of each channel's own timeline. `None` starts at
        /// the beginning.
        start_offset_ns: Option<u64>,
    },
    /// Re-execution of the bagged graph.
    Resim {
        /// Which nodes re-execute.
        selection: ResimSelection,
        /// `--verify` — byte-compare and apply the exit taxonomy.
        verify: bool,
        /// Knobs threaded into the engine.
        options: ResimOptions,
    },
}

/// The playback-only flags, with the reason each is meaningless under a resim.
/// Named once so the refusal text and the docs cannot drift apart.
const PLAYBACK_ONLY: &[(&str, &str)] = &[
    (
        "--rate",
        "a resim runs on the recording's own gating clock (that is what makes it \
         reproducible), so there is no wall pace to multiply",
    ),
    (
        "--loop",
        "a resim is ONE deterministic re-execution of the bag; running it twice in one \
         process would not re-execute the same starting state",
    ),
    (
        "--topics",
        "a resim's topic set is decided by which nodes re-execute, not by a filter — \
         narrowing by node is `--resim <nodes>`, which is not built yet",
    ),
    (
        "--start-offset",
        "re-executing from the MIDDLE of a recording needs a per-rank resume ANCHOR \
         (the graph state each worker held at that instant), and a free-run recording \
         carries none — which is the same reason a mid-run free-run bag is refused at \
         the replay entry. Per-rank anchors are not supported yet; until \
         they land, a resim starts where the recording does",
    ),
];

/// The resim-only flags, with the reason each is meaningless under playback.
const RESIM_ONLY: &[(&str, &str)] = &[
    (
        "--verify",
        "there is nothing to verify: `bag play` without `--resim` re-publishes the \
         recorded frames verbatim, so they match the recording by construction",
    ),
    (
        "--report",
        "the report is a re-execution verdict; plain playback produces none",
    ),
    (
        "--tolerance",
        "a tolerance RELAXES a byte-comparison, and plain playback performs none",
    ),
    (
        "--strict-state",
        "there is no node state to restore: playback re-publishes recorded frames \
         and executes nothing",
    ),
];

/// Resolve a `bag play` invocation, refusing every illegal flag combination
/// LOUDLY and by name.
///
/// The refusals are grouped by the question they answer, and each names the
/// remedy rather than only the conflict:
///
/// 1. a playback-only flag under `--resim`,
/// 2. a resim-only flag with no `--resim`,
/// 3. a verify-only flag under a NEUTRAL resim.
///
/// Group 3 is the one worth arguing. `--tolerance` and `--report` both describe
/// a *verdict*: a tolerance widens the byte-comparison's accept band, and the
/// report's own `passed` / `violations` fields ARE the taxonomy neutral mode
/// declines to claim. Writing that JSON from a neutral run would hand the caller
/// the verdict through a field name while the exit code said no verdict was
/// made. `--duration` is different and is allowed in BOTH modes: it bounds how
/// much of the bag executes (or is republished), which is a question about the
/// run, not about the comparison. `--start-offset` is NOT: a resim would need a
/// per-rank resume ANCHOR to start in the middle, and no recording carries one
/// (the same reason a mid-run free-run bag is refused at the replay entry).
pub fn resolve_play_mode(flags: PlayFlags) -> Result<PlayMode, String> {
    let PlayFlags {
        resim,
        verify,
        rate,
        repeat,
        topics,
        duration,
        start_offset,
        report,
        tolerance,
        strict_state,
    } = flags;
    // Both bounds are BAG-TIME SECONDS, so both are validated the same way and
    // before anything else: a bound nobody can act on is a usage error, not a
    // silently-ignored flag.
    let duration_bound_ns = parse_bag_seconds("--duration", duration)?;
    let start_offset_ns = parse_bag_seconds("--start-offset", start_offset)?;

    let Some(raw) = resim else {
        // Playback. Refuse every resim-only flag, in declaration order.
        let present: [bool; 4] = [verify, report.is_some(), tolerance.is_some(), strict_state];
        if let Some((flag, why)) = RESIM_ONLY
            .iter()
            .zip(present)
            .find_map(|(entry, given)| given.then_some(*entry))
        {
            return Err(format!(
                "`{flag}` needs `--resim` — {why}. To RE-EXECUTE the bag's graph against your \
                 current build, run `cerulion bag play <bag> --resim all` (add `--verify` to \
                 byte-compare it against the recording)."
            ));
        }
        return Ok(PlayMode::Playback {
            rate: rate.unwrap_or(DEFAULT_RATE),
            repeat,
            topics,
            duration_bound_ns,
            start_offset_ns,
        });
    };

    let selection = ResimSelection::parse(&raw)?;

    // Playback-only flags under a resim.
    let present: [bool; 4] = [
        rate.is_some(),
        repeat,
        !topics.is_empty(),
        start_offset.is_some(),
    ];
    if let Some((flag, why)) = PLAYBACK_ONLY
        .iter()
        .zip(present)
        .find_map(|(entry, given)| given.then_some(*entry))
    {
        return Err(format!(
            "`{flag}` is a PLAYBACK flag and cannot be combined with `--resim` — {why}. Drop \
             `{flag}` to re-execute the graph, or drop `--resim` to play the recorded frames \
             back verbatim."
        ));
    }

    // Verify-only flags under a NEUTRAL resim.
    if !verify {
        for (flag, given) in [
            ("--report", report.is_some()),
            ("--tolerance", tolerance.is_some()),
        ] {
            if given {
                return Err(format!(
                    "`{flag}` needs `--verify` — a bare `--resim all` re-executes the graph and \
                     makes NO claim about whether the result matches the recording, so there is \
                     no verdict for `{flag}` to shape or persist. Add `--verify` to byte-compare \
                     against the recording."
                ));
            }
        }
    }

    Ok(PlayMode::Resim {
        selection,
        verify,
        options: ResimOptions {
            duration_bound_ns,
            report_path: report,
            tolerance_path: tolerance,
            strict_state,
        },
    })
}

/// `bag play`'s pace default, applied in [`resolve_play_mode`] rather than by
/// clap so the flag's PRESENCE survives to the refusal seam.
pub const DEFAULT_RATE: f64 = 1.0;

/// Parse a BAG-TIME bound given in SECONDS into nanoseconds.
///
/// Seconds because that is the unit a bag's own duration is quoted in
/// everywhere else an operator meets it (`bag info`, the play summary), and
/// FRACTIONAL because sub-second bounds are the useful ones on a 1 kHz graph.
///
/// A value that cannot be a duration is a USAGE error and is refused by name
/// rather than clamped: silently reading `--duration -1` as "the whole bag"
/// would make the flag lie about what it did.
fn parse_bag_seconds(flag: &str, secs: Option<f64>) -> Result<Option<u64>, String> {
    let Some(v) = secs else { return Ok(None) };
    if !v.is_finite() || v < 0.0 {
        return Err(format!(
            "`{flag} {v}` is not a duration — it must be a finite, non-negative number of \
             SECONDS of bag time (fractional accepted, e.g. `{flag} 12.5`). Omit it to cover \
             the whole recording."
        ));
    }
    // `f64` holds every nanosecond value a real bag can carry exactly enough
    // for a bound (a 2^53 ns bag is ~104 days), and the saturating cast keeps
    // an absurd ask from wrapping into a tiny one.
    Ok(Some((v * 1e9) as u64))
}

/// The projection of a [`ReplayOutcome`] the resim surface reads.
///
/// A thin struct rather than the outcome itself, for one reason: the exit
/// classifier and the neutral renderer are the two places this surface actually
/// decides anything, and they must be oracle-testable against hand-written
/// inputs. Building a whole `ReplayOutcome` in a test would couple this
/// module's oracles to the engine's field list — the engine that this surface
/// deliberately does not touch.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResimReport {
    /// Every candidate node that suffered a PANIC-CLASS failure, as
    /// `(node_id, reason)` — exit 3 in BOTH modes, and rendered in both, since
    /// a crash is the one thing a neutral resim must never report silently.
    pub node_failures: Vec<(String, String)>,
    /// The re-executed fire schedule diverged from the recording.
    pub trace_diverged: bool,
    /// Every data violation the engine recorded, as `(topic, detail)`.
    ///
    /// Carried because the topic TALLY cannot express all of them, and the
    /// neutral renderer must never make a claim wider than its evidence. The
    /// shipping instance is the engine's rule 5e: when a bag's `record_health.json`
    /// is lossy but no PER-TOPIC violation attributed the loss, the engine adds
    /// one global `RecordSideTapLoss` violation under the sentinel topic
    /// `(recording)` — and it does so ONLY on the branch where `violations`,
    /// `trace_divergence` and `node_failures` are all empty
    /// (the global catch-all in `replay_engine.rs`). That
    /// precondition is exactly `topics_passed == topics_checked`, so the
    /// violation can never move `topics_checked - topics_passed`: reading the
    /// tally alone, a lossy recording is INDISTINGUISHABLE from a perfect
    /// match, and the summary claimed the latter every single time the
    /// catch-all fired.
    pub violations: Vec<(String, String)>,
    /// The engine's own pass flag (no violations, no divergence, no failure).
    pub passed: bool,
    /// Recorded steps actually re-executed.
    pub ticks_replayed: usize,
    /// Graph-produced topics the diff considered.
    pub topics_checked: usize,
    /// How many of those matched byte-for-byte.
    pub topics_passed: usize,
    /// The range a Flashback capture DECLARED its resume covers, as
    /// `(through_ns, frames beyond it)`.
    ///
    /// `None` for every `--record` bag. Carried into the neutral projection
    /// because a bare `re-executed N step(s)` line cannot say that the bag held
    /// frames the re-execution deliberately did not reach — and a neutral resim
    /// makes no verdict, so an unreported exclusion would leave the operator with
    /// no signal at all that the range stopped short of the window.
    ///
    /// A TOTAL rather than the per-topic map: the neutral summary's job is to
    /// say that a bounded remainder exists and where the bound is, and the
    /// per-topic breakdown is in the `--report` JSON for a reader who wants it.
    pub covered_range: Option<(u64, usize)>,
    /// The COORDINATION PROVENANCE line, already
    /// rendered.
    ///
    /// The rule is that the contract a replay APPLIED is always visible, and
    /// a neutral resim is a surface like any other — it makes no verdict, but
    /// "which contract did you re-execute this under" is not a verdict, it is
    /// what makes the re-execution readable at all. The verify-mode verdict
    /// prints it (`replay_engine::render_verdict`), and the neutral summary
    /// prints it too.
    ///
    /// Carried as the RENDERED line rather than as the two fields: the text is
    /// the canonical vocabulary and lives in exactly one function
    /// (`replay_engine::render_coordination_line`), which is Unix-only, while
    /// everything that consumes a `ResimReport` is not. `None` only for a
    /// hand-built report; [`ResimReport::from_outcome`] always fills it.
    pub coordination: Option<String>,
    /// Every bag-fed injection ANOMALY, as
    /// `(rank, topic, detail)`.
    ///
    /// Carried into the neutral projection for the reason the covered range is:
    /// a neutral resim makes no verdict, so the ONE thing it can still do wrong
    /// is let the operator believe the re-execution saw what the recording saw.
    /// An anomaly says it did not — the harness and the recording disagree about
    /// a topic's frame stream — and reporting it only under `--verify` would
    /// leave a bare `--resim` reporting a difference count with no way to know
    /// the inputs differed first.
    ///
    /// The `detail` sentence carries the kind's own wording, so the projection
    /// does not also have to carry the kind.
    pub injection_anomalies: Vec<(u32, String, String)>,
    /// Every node the harness UNDER-FED, as
    /// `(rank, node_id, empty refills)`.
    ///
    /// The sibling of [`Self::injection_anomalies`] one layer down: the frames
    /// reached the topic but not the node's burst, so the node fired on its held
    /// head. Same reason for carrying it into the neutral mode — it is the cause
    /// an observed difference would otherwise be silently attributed to the
    /// candidate.
    pub input_shortfalls: Vec<(u32, String, u64)>,
}

impl ResimReport {
    /// Project an engine outcome onto the fields the surface reads.
    ///
    /// `#[cfg(unix)]` with its argument type — `ReplayOutcome` lives in the
    /// Unix-only engine. Everything that CONSUMES a `ResimReport` stays
    /// platform-independent, which is why the projection is a separate step.
    #[cfg(unix)]
    pub fn from_outcome(o: &ReplayOutcome) -> Self {
        Self {
            node_failures: o
                .node_failures
                .iter()
                .map(|f| (f.node_id.clone(), f.reason.clone()))
                .collect(),
            trace_diverged: o.trace_divergence.is_some(),
            violations: o
                .violations
                .iter()
                .map(|v| (v.topic.clone(), v.detail.clone()))
                .collect(),
            passed: o.passed,
            ticks_replayed: o.ticks_replayed,
            topics_checked: o.topics_checked,
            topics_passed: o.topics_passed,
            covered_range: o
                .covered_range
                .as_ref()
                .map(|c| (c.through_ns, c.trailing_frames.values().sum())),
            coordination: Some(crate::replay_engine::render_coordination_line(
                &o.coordination,
            )),
            injection_anomalies: o
                .injection_anomalies
                .iter()
                .map(|a| (a.rank, a.topic.clone(), a.detail.clone()))
                .collect(),
            input_shortfalls: o
                .replay_input_shortfalls
                .iter()
                .map(|s| (s.rank, s.node_id.clone(), s.count))
                .collect(),
        }
    }
}

/// Pick the exit code for a completed re-execution.
///
/// A NEUTRAL resim declines exactly the two COMPARISON outcomes: byte
/// violations (exit 1) and the structural trace divergence (exit 6). Both are
/// answers to "does this match the recording", a question a bare `--resim` did
/// not ask. Everything else keeps its code in both modes — see the module docs.
pub fn resim_exit_code(report: &ResimReport, verify: bool) -> u8 {
    // A panic-class node failure is NOT a comparison result: the candidate
    // crashed, so the re-execution the neutral mode promised did not happen.
    // It preempts everything in both modes, exactly as `cerulion replay` does.
    if !report.node_failures.is_empty() {
        return EXIT_NODE_FAILURE;
    }
    if !verify {
        return EXIT_PASS;
    }
    if report.trace_diverged {
        return EXIT_TRACE_DIVERGENCE;
    }
    if report.passed {
        EXIT_PASS
    } else {
        EXIT_VIOLATION
    }
}

/// Push the COORDINATION PROVENANCE line, if the report carries one.
///
/// A helper rather than an inline block because it is emitted on EVERY path of
/// [`render_resim_summary`] — the crash path included. The verify verdict
/// (`replay_engine::render_verdict`) prints it FIRST, "above everything else, on
/// the pass path and the fail path alike"; the neutral renderer's crash arm
/// returns EARLY, so before this helper existed the one path that exits NON-ZERO
/// was also the only one that never said which contract it re-executed under.
///
/// It sits directly under the header on both paths, so the two arms read the
/// same way; the line's TEXT is `replay_engine::render_coordination_line`'s, so
/// the neutral summary and the verify verdict speak ONE vocabulary.
///
/// An ABSENT provenance renders NOTHING — never a default. `None` is reachable
/// only from a hand-built [`ResimReport`] ([`ResimReport::from_outcome`] always
/// fills it), and inventing a contract for a report that carries none would be
/// exactly the positive claim the `Option` exists to prevent.
fn push_coordination(s: &mut String, report: &ResimReport) {
    if let Some(line) = &report.coordination {
        s.push_str(&format!("  {line}\n"));
    }
}

/// Render what a NEUTRAL resim did, WITHOUT a pass/fail verdict.
///
/// The verify-mode renderer (`replay_engine::render_verdict`) opens with a
/// `PASS` / `FAIL` line and enumerates violations; printing that under an exit 0
/// would be worse than printing nothing. This one reports the facts a neutral
/// re-execution actually established — how much of the bag ran, how many
/// produced topics it recomputed, and whether the recomputed bytes happened to
/// differ from the recording — and then says plainly that no verdict was made
/// and how to ask for one.
///
/// The difference count is reported as an OBSERVATION, never as a failure: it is
/// the neutral mode's product. It is also the right thing to show, because the
/// engine computed it either way and hiding it would leave the operator with no
/// signal at all from a re-execution they asked for.
pub fn render_resim_summary(report: &ResimReport, bag: &Path) -> String {
    let mut s = String::new();

    // A crashed candidate is NOT an observation, so it does not get the
    // no-verdict framing: the re-execution this mode promised did not happen,
    // the exit code is 3 in both modes, and the operator needs to be told WHY.
    // Reporting a node failure only under `--verify` would leave a neutral run
    // exiting non-zero with nothing on stderr explaining it — which is exactly
    // the silent failure the neutral mode is otherwise careful to avoid.
    if !report.node_failures.is_empty() {
        s.push_str(&format!("resim FAILED: {}\n", bag.display()));
        push_coordination(&mut s, report);
        for (node_id, reason) in &report.node_failures {
            s.push_str(&format!("  NODE FAILURE: '{node_id}' — {reason}\n"));
        }
        s.push_str(
            "  the re-execution did not complete, so nothing about the recording was \
             observed. Fix the crash and re-run.\n",
        );
        return s;
    }

    s.push_str(&format!("resim (no verdict): {}\n", bag.display()));
    push_coordination(&mut s, report);
    s.push_str(&format!(
        "  re-executed {} step(s), {} produced topic(s)\n",
        report.ticks_replayed, report.topics_checked
    ));

    // A Flashback capture's COVERED RANGE, when the bag holds frames
    // beyond it.
    //
    // The line is the neutral mode's half of that rule: the bag keeps
    // every frame, the claim names the covered prefix, and a resume of such a bag
    // completes CLEANLY and SAYS SO, rather than exiting 2 and calling the
    // recording corrupt. Reported as a fact about the bag, never as a shortfall:
    // the frames are present and readable, and the reason the range stops is a
    // property of how a capture is closed, not of anything that went wrong.
    //
    // Gated on a non-empty remainder, so a capture that covered everything it
    // holds prints nothing extra.
    if let Some((through_ns, beyond)) = report.covered_range {
        if beyond > 0 {
            s.push_str(&format!(
                "  resim covered this recording through {through_ns} ns; {beyond} recorded \
                 frame(s) published after that instant were outside the covered range (they are \
                 IN the bag and readable — a capture's frame window and its scheduler-trace \
                 window are closed independently)\n"
            ));
        }
    }

    // The two HARNESS-side reports, ABOVE the observation they
    // qualify. A neutral resim makes no verdict, so the one thing it can still
    // get wrong is letting the operator read its difference count as a fact
    // about the candidate when the replay's own INPUT stream differed from the
    // recording's first. Both reached the `--report` JSON and nothing else.
    for (rank, topic, detail) in &report.injection_anomalies {
        s.push_str(&format!(
            "  injection anomaly: rank {rank} topic {topic}: {detail}\n"
        ));
    }
    for (rank, node_id, count) in &report.input_shortfalls {
        s.push_str(&format!(
            "  under-fed node: rank {rank} '{node_id}' asked its refill hook for the next \
             recorded frame and got nothing {count} time(s), so it fired on its held head\n"
        ));
    }

    let differing = report.topics_checked.saturating_sub(report.topics_passed);

    // The byte-for-byte claim is the ONE positive statement this renderer
    // makes, so it carries the strictest precondition: every checked topic
    // matched, the schedule matched, the engine recorded NO violation at all,
    // and at least one topic was actually compared.
    //
    // The two extra conjuncts are each load-bearing:
    //
    // * `violations.is_empty()` — a violation the topic tally cannot express is
    //   not a hypothetical. The engine's rule 5e adds a global `(recording)`
    //   violation for a lossy recording precisely when nothing else diverged,
    //   which is precisely when `differing == 0`, so a two-conjunct guard
    //   would announce a byte-for-byte match on EVERY lossy bag — while the very
    //   violation it ignores says byte-exact verification was IMPOSSIBLE
    //   because the RECORDING lost frames. Neutral mode is allowed to make no
    //   verdict; it is not allowed to make a false observation.
    //
    // * `topics_checked > 0` — with nothing compared, "the frames match" is
    //   vacuously true and reads as a positive result (`--duration 0`, or a
    //   run where no produced topic carried a frame).
    let matched_everything = differing == 0
        && !report.trace_diverged
        && report.violations.is_empty()
        && report.topics_checked > 0;

    if matched_everything {
        s.push_str("  observation: the re-executed frames match the recording byte-for-byte\n");
    } else {
        if differing > 0 {
            s.push_str(&format!(
                "  observation: {differing} of {} topic(s) differ from the recording\n",
                report.topics_checked
            ));
        }
        // Violations NOT accounted for by the tally above. Rendered only when
        // `differing == 0`, because a differing topic's own violation is
        // already reported by the count — printing both would double-report one
        // divergence. What survives that filter is the unattributed kind, whose
        // `detail` names the real cause (the rule-5e text names the dropped
        // frame counts and the topics they came from).
        if differing == 0 {
            for (topic, detail) in &report.violations {
                s.push_str(&format!("  observation: [{topic}] {detail}\n"));
            }
        }
        if report.trace_diverged {
            s.push_str("  observation: the re-executed fire schedule differs from the recording\n");
        }
        if differing == 0 && report.violations.is_empty() && !report.trace_diverged {
            // Reached only via `topics_checked == 0`.
            s.push_str(
                "  observation: no produced topic carried a frame, so NOTHING was compared \
                 against the recording\n",
            );
        }
    }

    s.push_str(
        "  no verdict was made — divergence is the OUTPUT of a bare `--resim`, not a failure. \
         Add `--verify` to byte-compare against the recording and get the pass/fail exit \
         contract.\n",
    );
    s
}

/// Run a resolved resim and return its exit code, rendering the mode's own
/// report to `stderr` (the verdict under `--verify`, the neutral summary
/// otherwise) — the same stream `cerulion replay` used, so a CI caller's stdout
/// redirection is unaffected.
///
/// `#[cfg(unix)]`: this is one of the two seams that reach the Unix-only bag
/// reader. Everything else in this module is platform-independent by design —
/// see the module docs.
#[cfg(unix)]
// Logging-rule exception: this is a CLI verb's rendered VERDICT, written to stderr so a
// piped stdout stays clean. It is the command's answer to the user, not a log
// line, and must not be suppressible by RUST_LOG.
#[allow(clippy::print_stderr)]
pub fn run_resim(
    bag: &Path,
    selection: &ResimSelection,
    verify: bool,
    options: ResimOptions,
) -> u8 {
    let ResimSelection::All = selection;
    match replay_cmd::run_replay(bag, engine_options(options)) {
        Ok(outcome) => {
            let report = ResimReport::from_outcome(&outcome);
            if verify {
                eprint!("{}", replay_cmd::render_verdict(&outcome, bag));
            } else {
                eprint!("{}", render_resim_summary(&report, bag));
            }
            resim_exit_code(&report, verify)
        }
        // The typed-error half of the contract is identical in both modes:
        // every `ReplayError` means the re-execution could not be performed (or
        // the bag could not be trusted), which no mode reports as success.
        Err(e) => {
            eprintln!("Error: {e}");
            e.exit_code()
        }
    }
}

/// Convert the surface's platform-independent knobs into the engine's.
///
/// Its own function, and unit-tested field by field, because it is the ONE seam
/// where a knob can be silently dropped on the way to the engine and NOTHING
/// downstream would notice: `--strict-state` in particular is inert on a
/// from-start bag, so an e2e over any bag this repo's CLI tests can build would
/// pass with it discarded.
#[cfg(unix)]
fn engine_options(o: ResimOptions) -> replay_cmd::ReplayOptions {
    let ResimOptions {
        duration_bound_ns,
        report_path,
        tolerance_path,
        strict_state,
    } = o;
    replay_cmd::ReplayOptions {
        duration_bound_ns,
        report_path,
        tolerance_path,
        strict_state,
    }
}

/// The whole `cerulion bag play --resim` exit policy, in ONE place.
///
/// Resolve the flags, refuse an illegal combination with [`EXIT_USAGE`], and
/// otherwise run the resim and return its code. The CLI's only job is to
/// destructure clap's parse into [`PlayFlags`] and hand it here — so the codes
/// a misuse, a refusal and a verdict produce cannot drift apart across crates.
///
/// `#[cfg(unix)]` with [`run_resim`], its only callee that touches the engine.
#[cfg(unix)]
// Logging-rule exception: same as `run_resim` — a CLI verb's user-facing error line,
// paired with the exit code it returns.
#[allow(clippy::print_stderr)]
pub fn run_play_resim(bag: &Path, flags: PlayFlags) -> u8 {
    match resolve_play_mode(flags) {
        Ok(PlayMode::Resim {
            selection,
            verify,
            options,
        }) => run_resim(bag, &selection, verify, options),
        // Unreachable from the CLI caller, which routes here only when a
        // resim-family flag is present — and `resolve_play_mode` refuses every
        // one of those without `--resim`. Answered rather than
        // `unreachable!`d, so a future refactor of either side degrades to a
        // true message instead of a panic.
        Ok(PlayMode::Playback { .. }) => {
            eprintln!("Error: no `--resim` given — `cerulion bag play <bag>` plays the recorded frames back verbatim; add `--resim all` to re-execute the bag's graph instead.");
            EXIT_USAGE
        }
        Err(e) => {
            eprintln!("Error: {e}");
            EXIT_USAGE
        }
    }
}

/// Exit code for a MISUSE of the resim surface — an illegal flag combination
/// ([`resolve_play_mode`]) or an unbuildable selection ([`ResimSelection::parse`]).
///
/// 2 because that is what clap already returns for every other usage error in
/// this binary (an unknown flag, a missing argument), and a flag-legality
/// refusal is exactly that class. What matters more is the code it must NOT be:
/// **1**, which under `--resim --verify` means "your code diverged from the
/// recording". A malformed invocation reported as a data violation would make a
/// CI job announce a regression that does not exist — the worst reading
/// available. 2 is at least never a claim about the candidate's bytes.
pub const EXIT_USAGE: u8 = 2;

#[cfg(test)]
mod tests {
    use super::*;

    fn resim(raw: &str) -> PlayFlags {
        PlayFlags {
            resim: Some(raw.to_string()),
            ..Default::default()
        }
    }

    #[test]
    fn resim_all_is_the_one_accepted_selection() {
        assert_eq!(ResimSelection::parse("all"), Ok(ResimSelection::All));
    }

    #[test]
    fn a_node_subset_is_refused_by_name_and_names_the_gap() {
        // Hand oracle: the message must carry the offending value, the fact
        // that the narrowing is unbuilt, and the spelling that DOES work — a bare
        // "unsupported" would leave the operator guessing all three.
        let err = ResimSelection::parse("planner,controller").unwrap_err();
        assert!(err.contains("planner,controller"), "{err}");
        assert!(err.contains("not built yet"), "{err}");
        assert!(err.contains("--resim all"), "{err}");
    }

    #[test]
    fn selection_matching_is_exact_never_widened_to_all() {
        // Every near-miss is a refusal, not a silent whole-graph re-execution:
        // "ALL" is not "all", and an empty value is not "everything".
        for raw in ["ALL", "All", " all", "all ", "", "*"] {
            assert!(
                ResimSelection::parse(raw).is_err(),
                "`--resim {raw}` must be refused, not read as `all`"
            );
        }
    }

    #[test]
    fn plain_playback_keeps_its_flags_and_defaults_the_rate() {
        let mode = resolve_play_mode(PlayFlags {
            repeat: true,
            topics: vec!["/a".into()],
            ..Default::default()
        })
        .unwrap();
        assert_eq!(
            mode,
            PlayMode::Playback {
                rate: DEFAULT_RATE,
                repeat: true,
                topics: vec!["/a".into()],
                duration_bound_ns: None,
                start_offset_ns: None,
            }
        );
    }

    /// The two BAG-TIME bounds parse from SECONDS into
    /// nanoseconds and reach BOTH halves — `--duration` reaches each, and
    /// `--start-offset` reaches playback (a resim refuses it, which is
    /// `every_playback_flag_is_refused_under_resim_with_its_own_reason`).
    ///
    /// The values are chosen so a unit slip is visible rather than plausible:
    /// `12.5` is FRACTIONAL (a seconds→ns conversion that truncated to whole
    /// seconds gives 12e9, and one that forgot to scale gives 12), and the two
    /// flags carry DIFFERENT values so a mapping that crossed them fails.
    #[test]
    fn the_bag_time_bounds_parse_from_seconds_and_reach_both_halves() {
        let PlayMode::Playback {
            duration_bound_ns,
            start_offset_ns,
            ..
        } = resolve_play_mode(PlayFlags {
            duration: Some(12.5),
            start_offset: Some(0.25),
            ..Default::default()
        })
        .unwrap()
        else {
            panic!("expected playback");
        };
        assert_eq!(duration_bound_ns, Some(12_500_000_000));
        assert_eq!(start_offset_ns, Some(250_000_000));

        let PlayMode::Resim { options, .. } = resolve_play_mode(PlayFlags {
            duration: Some(12.5),
            ..resim("all")
        })
        .unwrap() else {
            panic!("expected a resim");
        };
        assert_eq!(options.duration_bound_ns, Some(12_500_000_000));

        // The negative: a bound nobody gave is not invented.
        let PlayMode::Playback {
            duration_bound_ns,
            start_offset_ns,
            ..
        } = resolve_play_mode(PlayFlags::default()).unwrap()
        else {
            panic!("expected playback");
        };
        assert_eq!(duration_bound_ns, None);
        assert_eq!(start_offset_ns, None);
    }

    /// A value that cannot be a duration is a USAGE error named by FLAG, never
    /// clamped — silently reading `--duration -1` as "the whole bag" would make
    /// the flag lie about what it did.
    #[test]
    fn a_bound_that_is_not_a_duration_is_refused_by_name() {
        for (flag, flags) in [
            (
                "--duration",
                PlayFlags {
                    duration: Some(-1.0),
                    ..Default::default()
                },
            ),
            (
                "--duration",
                PlayFlags {
                    duration: Some(f64::NAN),
                    ..Default::default()
                },
            ),
            (
                "--start-offset",
                PlayFlags {
                    start_offset: Some(f64::INFINITY),
                    ..Default::default()
                },
            ),
        ] {
            let err = resolve_play_mode(flags).unwrap_err();
            assert!(err.contains(flag), "the refusal names {flag}: {err}");
            assert!(
                err.contains("SECONDS of bag time"),
                "…and says what the unit is: {err}"
            );
        }
        // ANTI-TAUTOLOGY: zero is a LEGAL ask (the half-open window covers
        // nothing), so the guard must not refuse everything it is handed.
        let PlayMode::Playback {
            duration_bound_ns, ..
        } = resolve_play_mode(PlayFlags {
            duration: Some(0.0),
            ..Default::default()
        })
        .unwrap()
        else {
            panic!("expected playback");
        };
        assert_eq!(duration_bound_ns, Some(0));
    }

    /// The two NUMERIC edges of `parse_bag_seconds` —
    /// negative zero, and the saturating cast its own comment promises.
    ///
    /// Both are reachable from a command line: `--duration=-0` parses as
    /// `-0.0_f64`, and `--duration=1e30` is a number a user can type.
    #[test]
    fn the_bag_time_parser_answers_its_numeric_edges() {
        let bound = |v: f64| {
            resolve_play_mode(PlayFlags {
                duration: Some(v),
                ..Default::default()
            })
        };

        // NEGATIVE ZERO is a legal, finite, non-negative zero — `-0.0 < 0.0` is
        // FALSE in IEEE-754, so the guard admits it and it must mean the same
        // thing `0.0` does (a window covering nothing), never a refusal and
        // never a wrapped enormous bound.
        let PlayMode::Playback {
            duration_bound_ns, ..
        } = bound(-0.0).expect("negative zero is a legal zero")
        else {
            panic!("expected playback");
        };
        assert_eq!(
            duration_bound_ns,
            Some(0),
            "`-0.0` is zero, and `(-0.0 * 1e9) as u64` must be 0"
        );

        // The SATURATING cast. `1e30` seconds is ~1e39 ns, far past `u64::MAX`;
        // Rust's `as` saturates rather than wrapping, so an absurd ask becomes
        // an effectively unbounded one instead of a tiny one. The hand oracle
        // is `u64::MAX` — a WRAPPING cast would give a small number here and a
        // bound of a few nanoseconds would silently replay almost nothing.
        let PlayMode::Playback {
            duration_bound_ns, ..
        } = bound(1e30).expect("an enormous but finite ask is legal")
        else {
            panic!("expected playback");
        };
        assert_eq!(
            duration_bound_ns,
            Some(u64::MAX),
            "an over-range ask saturates, so it covers everything"
        );

        // And the boundary just BELOW the cast's ceiling still lands exactly:
        // one nanosecond is representable and must not be rounded to zero.
        let PlayMode::Playback {
            duration_bound_ns, ..
        } = bound(1e-9).expect("one nanosecond is legal")
        else {
            panic!("expected playback");
        };
        assert_eq!(duration_bound_ns, Some(1));
    }

    #[test]
    fn an_explicit_rate_survives_to_playback() {
        let mode = resolve_play_mode(PlayFlags {
            rate: Some(2.5),
            ..Default::default()
        })
        .unwrap();
        assert_eq!(
            mode,
            PlayMode::Playback {
                rate: 2.5,
                repeat: false,
                topics: vec![],
                duration_bound_ns: None,
                start_offset_ns: None,
            }
        );
    }

    #[test]
    fn bare_resim_all_is_neutral_and_carries_no_verify() {
        let mode = resolve_play_mode(resim("all")).unwrap();
        assert_eq!(
            mode,
            PlayMode::Resim {
                selection: ResimSelection::All,
                verify: false,
                options: ResimOptions::default(),
            }
        );
    }

    #[test]
    fn verify_threads_the_diff_knobs_through() {
        let mode = resolve_play_mode(PlayFlags {
            resim: Some("all".into()),
            verify: true,
            duration: Some(12.0),
            report: Some(PathBuf::from("/tmp/r.json")),
            tolerance: Some(PathBuf::from("/tmp/t.yaml")),
            strict_state: true,
            ..Default::default()
        })
        .unwrap();
        assert_eq!(
            mode,
            PlayMode::Resim {
                selection: ResimSelection::All,
                verify: true,
                options: ResimOptions {
                    duration_bound_ns: Some(12_000_000_000),
                    report_path: Some(PathBuf::from("/tmp/r.json")),
                    tolerance_path: Some(PathBuf::from("/tmp/t.yaml")),
                    strict_state: true,
                },
            }
        );
    }

    #[test]
    fn run_shaping_flags_are_legal_in_both_resim_modes() {
        // The line between the two flag classes: `--duration` and
        // `--strict-state` shape what EXECUTES, so neutral mode keeps them;
        // `--report` and `--tolerance` shape the VERDICT, so they need
        // `--verify` (the arm below).
        //
        // `--strict-state` is the one worth arguing, and the restore path's own
        // code decides it: `enforce_strict_state` is a PRECONDITION evaluated
        // before the first step, and its failure is a `ReplayError::StateRestore`
        // on the exit-2 arm — the typed-error path neutral mode already keeps
        // loud. Gating it behind `--verify` would let a neutral resim silently
        // run nodes from their constructors after the operator asked for full
        // restoration.
        for verify in [false, true] {
            let mode = resolve_play_mode(PlayFlags {
                resim: Some("all".into()),
                verify,
                duration: Some(3.0),
                strict_state: true,
                ..Default::default()
            })
            .unwrap();
            let PlayMode::Resim { options, .. } = mode else {
                panic!("expected a resim");
            };
            assert_eq!(options.duration_bound_ns, Some(3_000_000_000));
            assert!(
                options.strict_state,
                "`--strict-state` must reach the engine in {} mode",
                if verify { "verify" } else { "neutral" }
            );
        }
    }

    #[test]
    fn every_playback_flag_is_refused_under_resim_with_its_own_reason() {
        let cases: [(&str, PlayFlags); 4] = [
            (
                "--rate",
                PlayFlags {
                    rate: Some(2.0),
                    ..resim("all")
                },
            ),
            (
                "--loop",
                PlayFlags {
                    repeat: true,
                    ..resim("all")
                },
            ),
            (
                "--topics",
                PlayFlags {
                    topics: vec!["/a".into()],
                    ..resim("all")
                },
            ),
            // `--start-offset` is playback-only because
            // re-executing from the middle needs a per-rank resume ANCHOR, and
            // no recording carries one. Its sibling `--duration` is legal in
            // both halves and is therefore deliberately NOT in this table —
            // `run_shaping_flags_are_legal_in_both_resim_modes` is its arm.
            (
                "--start-offset",
                PlayFlags {
                    start_offset: Some(1.0),
                    ..resim("all")
                },
            ),
        ];
        for (flag, flags) in cases {
            let err = resolve_play_mode(flags).unwrap_err();
            assert!(err.contains(flag), "refusal must name {flag}: {err}");
            assert!(
                err.contains("PLAYBACK flag"),
                "refusal must say WHICH half {flag} belongs to: {err}"
            );
            // Anti-tautology: a refusal that only names the conflict leaves the
            // operator with two commands and no idea which to drop.
            assert!(
                err.contains("Drop") && err.contains("--resim"),
                "refusal must name both remedies: {err}"
            );
        }
    }

    #[test]
    fn every_resim_flag_is_refused_without_resim_with_its_own_reason() {
        let cases: [(&str, PlayFlags); 4] = [
            (
                "--verify",
                PlayFlags {
                    verify: true,
                    ..Default::default()
                },
            ),
            (
                "--strict-state",
                PlayFlags {
                    strict_state: true,
                    ..Default::default()
                },
            ),
            (
                "--report",
                PlayFlags {
                    report: Some(PathBuf::from("/tmp/r.json")),
                    ..Default::default()
                },
            ),
            (
                "--tolerance",
                PlayFlags {
                    tolerance: Some(PathBuf::from("/tmp/t.yaml")),
                    ..Default::default()
                },
            ),
        ];
        for (flag, flags) in cases {
            let err = resolve_play_mode(flags).unwrap_err();
            assert!(err.contains(flag), "refusal must name {flag}: {err}");
            assert!(
                err.contains("--resim all"),
                "refusal must name the spelling that works: {err}"
            );
        }
    }

    #[test]
    fn report_and_tolerance_need_verify_not_merely_resim() {
        for (flag, flags) in [
            (
                "--report",
                PlayFlags {
                    report: Some(PathBuf::from("/tmp/r.json")),
                    ..resim("all")
                },
            ),
            (
                "--tolerance",
                PlayFlags {
                    tolerance: Some(PathBuf::from("/tmp/t.yaml")),
                    ..resim("all")
                },
            ),
        ] {
            let err = resolve_play_mode(flags).unwrap_err();
            assert!(err.contains(flag), "{err}");
            assert!(err.contains("--verify"), "must name the remedy: {err}");
            assert!(
                err.contains("NO claim"),
                "must say WHY a neutral resim has no verdict to shape: {err}"
            );
        }
    }

    #[test]
    fn a_subset_selection_is_refused_before_any_flag_legality_check() {
        // Order matters: an operator who typed both a bad selection and a bad
        // flag should be told the selection is not built, not sent chasing a
        // flag conflict that will still be there afterwards.
        let err = resolve_play_mode(PlayFlags {
            rate: Some(2.0),
            ..resim("planner")
        })
        .unwrap_err();
        assert!(err.contains("not built yet"), "{err}");
        assert!(!err.contains("PLAYBACK flag"), "{err}");
    }

    /// A report in the shape the exit classifier and renderer read. `3` topics
    /// checked, `violations` of them differing.
    fn report(passed: bool, violations: usize, diverged: bool) -> ResimReport {
        ResimReport {
            node_failures: vec![],
            trace_diverged: diverged,
            // One per DIFFERING topic, so the tally and the violation list
            // agree the way a real outcome's do.
            violations: (0..violations)
                .map(|i| (format!("/t{i}"), format!("frame 0 differs on /t{i}")))
                .collect(),
            passed,
            ticks_replayed: 10,
            topics_checked: 3,
            topics_passed: 3 - violations,
            // The ORDINARY shape — a `--record` bag declares no range — so the
            // arms that are not about the covered range keep rendering what they always
            // did, and the covered-range line stays absent unless an arm asks
            // for it.
            covered_range: None,
            // The provenance line, as
            // `replay_engine::render_coordination_line` renders it for the
            // ordinary shape — a bag carrying an explicit lockstep stamp.
            coordination: Some("coordination: lockstep".to_string()),
            // The two harness-side reports are additive and the
            // arms that are not about them keep rendering what they always did.
            injection_anomalies: vec![],
            input_shortfalls: vec![],
        }
    }

    /// The rule is that the contract a re-execution
    /// APPLIED is always visible, on every surface that reports one — and a
    /// NEUTRAL resim is one. It makes no verdict, but "which contract
    /// did you re-execute this under" is not a verdict: without it a free-run
    /// bag and an older, unstamped bag produce the same summary, and the operator
    /// cannot tell an EXPLICIT lockstep stamp from an INFERRED one.
    ///
    /// The line is rendered upstream by
    /// `replay_engine::render_coordination_line` and printed verbatim here, so
    /// the neutral summary and the verify verdict speak ONE vocabulary.
    #[test]
    fn the_neutral_summary_states_the_coordination_provenance() {
        let bag = Path::new("/tmp/x.mcap");

        let mut lockstep = report(true, 0, false);
        lockstep.coordination = Some("coordination: lockstep".to_string());
        let out = render_resim_summary(&lockstep, bag);
        assert!(
            out.contains("coordination: lockstep"),
            "the neutral summary must state which contract it re-executed under: {out}"
        );

        // The INFERRED suffix is what separates "the recorder said lockstep"
        // from "nothing said, so lockstep was assumed" — the distinction the whole
        // `Option<CoordinationMode>` stamp exists to preserve.
        let mut inferred = report(true, 0, false);
        inferred.coordination =
            Some("coordination: lockstep (inferred: no coordination stamp)".to_string());
        let out = render_resim_summary(&inferred, bag);
        assert!(
            out.contains("(inferred: no coordination stamp)"),
            "an inferred contract must be rendered as inferred: {out}"
        );

        // A hand-built report with no provenance prints no line at all rather
        // than inventing one — absence is never rendered as a claim.
        let mut none = report(true, 0, false);
        none.coordination = None;
        let out = render_resim_summary(&none, bag);
        assert!(
            !out.contains("coordination:"),
            "an absent provenance must render NOTHING, never a default: {out}"
        );
    }

    /// A resim of a Flashback capture SAYS how far it covered, and
    /// says nothing when the range covered everything.
    ///
    /// A neutral resim makes no verdict, so an unreported exclusion leaves the
    /// operator with no signal at all that the re-execution stopped short of the
    /// window — which, for a bag whose whole purpose is an incident, is the
    /// silent half of the very confident-false the range fixes.
    ///
    /// Three shapes in one body, because what matters is which of them SPEAK:
    ///
    /// * a range with frames beyond it — the tail-race capture — names both the
    ///   instant and the count, AND says the frames are still in the bag (a line
    ///   that only said "N frames were not compared" reads as data loss);
    /// * a range with NOTHING beyond it — the two windows closed together — is
    ///   SILENT, or an operator learns to ignore the line on the runs where it
    ///   means something;
    /// * no range at all — every `--record` bag — renders byte-identically to
    ///   the output without the line, which is the anti-regression half.
    #[test]
    fn a_neutral_resim_reports_the_range_it_covered_only_when_something_fell_outside() {
        let bag = Path::new("/tmp/capture.mcap");

        let mut with_tail = report(true, 0, false);
        with_tail.covered_range = Some((16_102_963_042, 7));
        let text = render_resim_summary(&with_tail, bag);
        assert!(
            text.contains("through 16102963042 ns"),
            "the covered range's end is named: {text}"
        );
        assert!(
            text.contains("7 recorded frame(s)"),
            "…and how much fell outside it: {text}"
        );
        assert!(
            text.contains("IN the bag and readable"),
            "…and that those frames are PRESENT — the bag keeps every frame, so a line \
             reporting only the exclusion would read as loss: {text}"
        );

        let mut covered_everything = report(true, 0, false);
        covered_everything.covered_range = Some((16_102_963_042, 0));
        let quiet = render_resim_summary(&covered_everything, bag);
        assert!(
            !quiet.contains("outside the covered range"),
            "a capture that covered everything it holds announces no limitation: {quiet}"
        );

        // …and a `--record` bag is byte-identical to a capture that covered
        // everything, which is the strongest form of "nothing changed for it".
        assert_eq!(render_resim_summary(&report(true, 0, false), bag), quiet);
    }

    #[test]
    fn neutral_mode_reports_exit_zero_on_the_exact_outcome_verify_calls_a_violation() {
        // THE headline discriminator, at the classifier: one outcome, two
        // modes, two answers. Byte divergence and schedule divergence are the
        // only two things `--verify` adds.
        let diverged = report(false, 2, false);
        assert_eq!(resim_exit_code(&diverged, false), EXIT_PASS);
        assert_eq!(resim_exit_code(&diverged, true), EXIT_VIOLATION);

        let schedule = report(false, 0, true);
        assert_eq!(resim_exit_code(&schedule, false), EXIT_PASS);
        assert_eq!(resim_exit_code(&schedule, true), EXIT_TRACE_DIVERGENCE);
    }

    #[test]
    fn a_clean_re_execution_is_exit_zero_in_both_modes() {
        // Anti-tautology for the arm above: if neutral mode returned 0
        // unconditionally the discriminator would still pass, so the clean
        // case must agree with verify mode rather than differ from it.
        let clean = report(true, 0, false);
        assert_eq!(resim_exit_code(&clean, false), EXIT_PASS);
        assert_eq!(resim_exit_code(&clean, true), EXIT_PASS);
    }

    #[test]
    fn a_panicking_node_fails_loudly_in_neutral_mode_too() {
        // "Neutral" scopes the two COMPARISON outcomes and nothing else: a
        // candidate that crashed did not re-execute, so exit 0 would be a lie.
        let mut crashed = report(false, 0, false);
        crashed.node_failures = vec![("planner".into(), "tick PANICKED".into())];
        assert_eq!(resim_exit_code(&crashed, false), EXIT_NODE_FAILURE);
        assert_eq!(resim_exit_code(&crashed, true), EXIT_NODE_FAILURE);
    }

    #[test]
    fn a_node_failure_preempts_a_divergence_in_verify_mode() {
        // Root cause over symptoms — the precedence 3 > 6 > 1, kept
        // byte-for-byte by the new surface.
        let mut both = report(false, 2, true);
        both.node_failures = vec![("planner".into(), "tick PANICKED".into())];
        assert_eq!(resim_exit_code(&both, true), EXIT_NODE_FAILURE);
    }

    #[test]
    fn the_neutral_summary_reports_divergence_without_claiming_a_verdict() {
        let s = render_resim_summary(&report(false, 2, false), Path::new("/tmp/run.mcap"));
        assert!(s.contains("no verdict"), "{s}");
        assert!(s.contains("2 of 3 topic(s) differ"), "{s}");
        assert!(s.contains("--verify"), "must name how to ask for one: {s}");
        // The verify renderer's vocabulary must NOT leak into a mode that made
        // no claim — a "FAIL" line above an exit 0 is worse than silence.
        assert!(!s.contains("FAIL"), "{s}");
        assert!(!s.contains("PASS"), "{s}");
    }

    #[test]
    fn the_neutral_summary_says_so_when_nothing_diverged() {
        let s = render_resim_summary(&report(true, 0, false), Path::new("/tmp/run.mcap"));
        assert!(s.contains("match the recording byte-for-byte"), "{s}");
        assert!(s.contains("no verdict"), "{s}");
    }

    /// A lossy recording must never be reported
    /// as a byte-for-byte match.
    ///
    /// This is the exact shape the engine's rule 5e produces, and it is not a corner
    /// case — the catch-all is added ONLY when `violations`, `trace_divergence`
    /// and `node_failures` are all empty, i.e. only when every checked topic
    /// passed. So `topics_passed == topics_checked` is its PRECONDITION, and a
    /// renderer keying on the tally alone announces a perfect match on EVERY
    /// lossy bag — the strongest possible form of the defect, not the weakest.
    ///
    /// The exit code stays 0 and the "no verdict" framing stays: a neutral
    /// resim genuinely made no verdict. What it may not do is state an
    /// observation its evidence contradicts.
    #[test]
    fn a_lossy_recording_is_never_reported_as_a_byte_for_byte_match() {
        let mut lossy = report(false, 0, false);
        lossy.violations = vec![(
            "(recording)".into(),
            "the RECORDING lost frames: 12 frame(s) dropped before the writer".into(),
        )];

        let s = render_resim_summary(&lossy, Path::new("/tmp/run.mcap"));

        // THE pin: the false claim must be gone...
        assert!(
            !s.contains("match the recording byte-for-byte"),
            "a lossy recording was reported as a byte-for-byte match:\n{s}"
        );
        // ...and replaced by the real cause, not merely omitted. Silence would
        // leave the operator with a clean-looking summary for a lossy bag.
        assert!(s.contains("(recording)"), "{s}");
        assert!(s.contains("the RECORDING lost frames"), "{s}");
        assert!(s.contains("12 frame(s) dropped"), "{s}");
        // The mode's own framing is unchanged — still no verdict, still no
        // verify-mode vocabulary.
        assert!(s.contains("no verdict"), "{s}");
        assert!(!s.contains("FAIL"), "{s}");
        assert!(!s.contains("PASS"), "{s}");
        // And a lossy recording is still exit 0 in neutral mode: the fix is
        // vocabulary, not the exit contract.
        assert_eq!(resim_exit_code(&lossy, false), EXIT_PASS);
        assert_eq!(resim_exit_code(&lossy, true), EXIT_VIOLATION);
    }

    /// With NOTHING compared, "the frames match" is vacuously true and reads as
    /// a positive result. Reachable via `--duration 0`, or a run in which no
    /// produced topic carried a frame.
    #[test]
    fn a_run_that_compared_no_topic_does_not_claim_a_match() {
        let mut nothing = report(true, 0, false);
        nothing.topics_checked = 0;
        nothing.topics_passed = 0;

        let s = render_resim_summary(&nothing, Path::new("/tmp/run.mcap"));
        assert!(
            !s.contains("match the recording byte-for-byte"),
            "claimed a match having compared nothing:\n{s}"
        );
        assert!(s.contains("NOTHING was compared"), "{s}");
        assert!(s.contains("no verdict"), "{s}");
    }

    /// ANTI-TAUTOLOGY for the two arms above: a genuinely clean re-execution
    /// MUST still make the positive claim. Without this, deleting the claim
    /// outright would satisfy both of them.
    #[test]
    fn a_genuinely_clean_re_execution_still_claims_the_match() {
        let clean = report(true, 0, false);
        assert!(clean.violations.is_empty(), "helper precondition");
        let s = render_resim_summary(&clean, Path::new("/tmp/run.mcap"));
        assert!(s.contains("match the recording byte-for-byte"), "{s}");
        assert!(!s.contains("NOTHING was compared"), "{s}");
    }

    /// A DIFFERING topic's violation is reported by the count, not twice. This
    /// pins that the unattributed-violation arm did not turn every ordinary
    /// divergence into a second, duplicated line.
    #[test]
    fn a_differing_topic_is_reported_once_not_twice() {
        let s = render_resim_summary(&report(false, 2, false), Path::new("/tmp/run.mcap"));
        assert!(s.contains("2 of 3 topic(s) differ"), "{s}");
        assert_eq!(
            s.matches("observation:").count(),
            1,
            "one divergence, one observation line:\n{s}"
        );
    }

    #[test]
    fn the_neutral_summary_names_a_schedule_divergence_separately() {
        let s = render_resim_summary(&report(false, 0, true), Path::new("/tmp/run.mcap"));
        assert!(s.contains("fire schedule differs"), "{s}");
        assert!(!s.contains("topic(s) differ"), "{s}");
    }

    #[test]
    fn the_neutral_summary_reports_a_crash_instead_of_a_no_verdict_observation() {
        // The hole a real-binary run found: neutral mode exited 3 on a
        // panicking candidate while printing only its ordinary summary, so the
        // operator saw a non-zero exit with nothing explaining it. A crash is
        // not an observation — it REPLACES the no-verdict framing.
        let mut crashed = report(false, 0, false);
        crashed.node_failures = vec![("planner".into(), "tick PANICKED".into())];
        let s = render_resim_summary(&crashed, Path::new("/tmp/run.mcap"));
        assert!(s.contains("NODE FAILURE"), "{s}");
        assert!(s.contains("planner"), "must name the node: {s}");
        assert!(s.contains("tick PANICKED"), "must name the reason: {s}");
        assert!(
            !s.contains("no verdict"),
            "a crash is not a no-verdict observation: {s}"
        );
        assert!(
            !s.contains("observation:"),
            "nothing was observed — the run did not complete: {s}"
        );
    }

    /// The provenance line rides EVERY path of the
    /// neutral renderer, the CRASH path included.
    ///
    /// The verify verdict prints it FIRST — `replay_engine::render_verdict`'s
    /// own comment says "above everything else, on the pass path and the fail
    /// path alike", and its `NODE FAILURE` block renders BELOW it — while the
    /// neutral renderer's crash arm returns EARLY, so without the helper the one
    /// path that exits NON-ZERO is also the only one that never says which
    /// contract it re-executed under. That is the reading an operator debugging
    /// a panicking candidate most needs: without the line a free-run bag, an
    /// older unstamped bag and an explicitly-stamped one produce the same crash
    /// report.
    ///
    /// Three claims, each catching its own broken variant: the line is PRESENT
    /// on this path; it sits under the header and ABOVE the failure block (the
    /// shape both arms share); and an ABSENT provenance renders NOTHING rather
    /// than a fabricated default.
    #[test]
    fn a_crashed_re_execution_still_states_its_coordination_provenance() {
        let bag = Path::new("/tmp/crash.mcap");

        let mut crashed = report(false, 0, false);
        crashed.node_failures = vec![("planner".into(), "tick PANICKED".into())];
        // The INFERRED suffix deliberately, not the bare lockstep line: it is
        // the distinction the whole `Option<CoordinationMode>` stamp exists to
        // preserve, and a renderer that hardcoded "coordination: lockstep"
        // would satisfy a bare `contains("coordination:")`.
        crashed.coordination =
            Some("coordination: lockstep (inferred: no coordination stamp)".to_string());
        let out = render_resim_summary(&crashed, bag);

        assert!(
            out.contains("NODE FAILURE"),
            "precondition: this really is the crash path: {out}"
        );
        assert!(
            out.contains("(inferred: no coordination stamp)"),
            "a crashed re-execution must still state the contract it ran under: {out}"
        );
        let header = out.find("resim FAILED").expect("the crash header");
        let coord = out.find("coordination:").expect("the provenance line");
        let failure = out.find("NODE FAILURE").expect("the failure block");
        assert!(
            header < coord && coord < failure,
            "the provenance sits under the header and above the failure block, exactly as \
             it does on the no-verdict path: {out}"
        );

        // ANTI-TAUTOLOGY: an absent provenance renders NOTHING on this path
        // either. Without it, a renderer that unconditionally printed a
        // lockstep line would pass every assertion above.
        let mut none = crashed.clone();
        none.coordination = None;
        let out = render_resim_summary(&none, bag);
        assert!(out.contains("NODE FAILURE"), "still the crash path: {out}");
        assert!(
            !out.contains("coordination:"),
            "an absent provenance must render NOTHING, never a default: {out}"
        );
    }

    #[cfg(unix)]
    #[test]
    fn every_knob_survives_the_conversion_into_the_engines_options() {
        // The drop-a-knob guard. Asserted field by field rather than by struct
        // equality because `ReplayOptions` deliberately derives no `PartialEq`.
        let got = engine_options(ResimOptions {
            duration_bound_ns: Some(7_000_000_000),
            report_path: Some(PathBuf::from("/tmp/r.json")),
            tolerance_path: Some(PathBuf::from("/tmp/t.yaml")),
            strict_state: true,
        });
        assert_eq!(got.duration_bound_ns, Some(7_000_000_000));
        assert_eq!(got.report_path.as_deref(), Some(Path::new("/tmp/r.json")));
        assert_eq!(
            got.tolerance_path.as_deref(),
            Some(Path::new("/tmp/t.yaml"))
        );
        assert!(
            got.strict_state,
            "`--strict-state` must reach the engine — it is INERT on a \
             from-start bag, so no e2e this repo can cheaply build would catch \
             it being dropped here"
        );

        // And the negative: a knob nobody set must not be invented.
        let none = engine_options(ResimOptions::default());
        assert_eq!(none.duration_bound_ns, None);
        assert!(none.report_path.is_none());
        assert!(none.tolerance_path.is_none());
        assert!(!none.strict_state);
    }

    #[test]
    fn a_misuse_never_exits_as_a_data_violation() {
        // THE constraint on this code, and the reason it is named rather than
        // spelled `ExitCode::FAILURE` at the call site: under `--verify`, 1
        // means "your code diverged". A malformed invocation reported as 1
        // makes CI announce a regression that does not exist.
        assert_ne!(EXIT_USAGE, EXIT_PASS);
        assert_ne!(EXIT_USAGE, EXIT_VIOLATION);
    }
}
