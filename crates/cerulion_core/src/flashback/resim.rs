// SPDX-License-Identifier: AGPL-3.0-only
//! **May this capture claim to be resimmable?** That is judged by
//! `bag play --resim`'s OWN refusal rules, in one place, so the two answers
//! cannot drift.
//!
//! # Why a shared judge exists at all
//!
//! A Flashback capture's manifest carries a `resimmable` field. Until this
//! module it was computed as `trace_records > 0` — a claim about whether the bag
//! held a scheduler trace, and nothing else. That is one of **six** things resim
//! checks, and the other five are exactly the ones a capture is most likely to
//! fail: a fault-triggered capture carries a departure boundary, a multi-process
//! run declares several state rings, and an anchor that does not cover every
//! executed node is refused outright.
//!
//! A capture stamped `resimmable: true` that `bag play --resim` then refuses is
//! worse than one that never claimed it: the operator reads the manifest,
//! believes they can resume the incident, and finds out at the moment they most
//! need it not to be true. So the rule is written ONCE, here, in the crate both
//! readers already depend on, and the recorder judges its own capture with the
//! same predicates the replayer will use on it.
//!
//! # What this module is NOT
//!
//! It does not read a bag, does not open a ring and does not know what a
//! `BagReader` is — it takes already-extracted FACTS and returns a verdict. That
//! is the same rule [`crate::state_restore`] follows, and for the same reason:
//! this crate is portable, while every reader-side helper in the replay path is
//! `#[cfg(unix)]`. It also means every branch is oracle-testable, including the
//! ones a healthy robot never reaches.
//!
//! # The order is the precedence, and it is not cosmetic
//!
//! The arms are asked in the order `run_replay` asks them, so a capture reports
//! the SAME gap resim would report first. Reporting a later one would send an
//! operator to fix something that is not what stops them — the "report the arm
//! an operator can ACT on" rule the trigger gate already follows.
//!
//! Within the trace-record arm that order is RECORD ORDER, not a precedence
//! between KINDS of record. `run_replay` walks the trace once and refuses per
//! record — the departure gate, then the four record-level faults — so the gap
//! it names is the one belonging to the FIRST refusing record in file order.
//! This judge therefore asks ONE question there
//! ([`crate::trace_ring::first_trace_record_refusal`]) instead of testing an
//! aggregate departure COUNT ahead of the record walk: a trace carrying a
//! malformed record BEFORE a departure advertised fault replay while resim
//! refused it at the earlier record under a different error entirely, which is
//! precisely the drift a shared verdict exists to prevent.

use crate::state_restore::{plan_restore, AnchorFact, RestoreRequest};

/// The facts a verdict is computed from.
///
/// Every field is something the recorder can measure about the bag it is about
/// to write, and something the replayer re-measures about the bag it is about to
/// replay. Nothing here is a guess.
#[derive(Debug, Clone)]
pub struct ResimFacts<'a> {
    /// Scheduler-trace records the bag carries, AFTER any trim.
    ///
    /// A COUNT rather than a flag, so the manifest can report it and a reader
    /// can tell "a trimmed trace of 4 records" from "no trace at all".
    pub trace_records: usize,
    /// Trace records that are DEPARTURE (fault) boundaries — see
    /// [`crate::trace_ring::TraceRingRecord::is_departure_boundary`].
    ///
    /// A COUNT, for the REFUSAL'S MESSAGE. Whether a departure refuses at all is
    /// decided by [`first_record_refusal`](Self::first_record_refusal), because
    /// the gate refuses at a RECORD and this number cannot say where in the file
    /// that record sits. Measured over the SAME record set the walk runs over
    /// (the trimmed trace), so the two cannot disagree about whether a departure
    /// is present — the caller measures both in one pass.
    pub departure_records: usize,
    /// The step of the FIRST step-boundary record in the trace, which is what
    /// `resolve_resume` derives the resume point from.
    ///
    /// `None` means the trace carries no boundary at all. That is NOT the same
    /// as an empty trace (a trace of FIRE records with no boundary is possible
    /// on a torn head) and resim refuses it separately, so the two are kept
    /// apart here too.
    pub first_recorded_step: Option<u64>,
    /// Whether the bag carries a `graph.yaml` attachment `run_replay` can
    /// actually USE — present, UTF-8, and parsing as a graph (its gate 4,
    /// immediately after the trace-presence gate).
    ///
    /// PRESENCE alone is not the gate replay applies, and asking the weaker
    /// question here put the verdict and the gate back out of step — the exact
    /// defect the field was added to fix. `run_replay` decodes the attachment
    /// (`BagInvalidAttachment` on non-UTF-8) and parses it (`graph YAML did not
    /// parse`) before anything runs, so an attachment NAMED `graph.yaml` that is
    /// a truncated write, a stray binary, or malformed YAML is refused there
    /// while a capture asking only "is a thing with that name in the bag?"
    /// published `resimmable: true`. The caller supplies the parsed answer.
    ///
    /// A `graph run --record` recorder is always handed one, but `cerulion
    /// bagd` takes `--attach` and `--ring` INDEPENDENTLY, so a standalone
    /// recorder with trace rings and no attachments is an ordinary invocation
    /// that produced a capture claiming `resimmable: true` against a bag resim
    /// refuses at `BagMissingAttachment`.
    pub graph_attachment_present: bool,
    /// A hole the RECORDER observed in its own read of the trace ring, when
    /// there is one — see [`TraceGap`].
    ///
    /// `None` is "this recorder observed no hole", which is the ordinary answer
    /// and the only one a recorder that never drained a ring can give. A
    /// recorder that cannot observe holes at all (it was handed no ring) reaches
    /// [`ResimGap::NoTrace`] first, so the two absences never have to be told
    /// apart here.
    pub trace_gap: Option<TraceGap>,
    /// The LOWEST and HIGHEST step among the records this capture carries, after
    /// any trim.
    ///
    /// Read only by the [`TraceGap`] arm, and read as a PAIR: the question there
    /// is whether the carried stream straddles a hole, which needs both edges.
    /// `None` on an empty trace, which cannot straddle anything and is refused
    /// by [`ResimGap::NoTrace`] before this arm is reached anyway.
    ///
    /// Kept apart from [`first_recorded_step`](Self::first_recorded_step), which
    /// is the first STEP-BOUNDARY record's step — the value `resolve_resume`
    /// derives a resume point from. These two are over ALL records including
    /// fires, because a hole is a hole in the record stream whatever kind of
    /// record sits either side of it.
    pub min_recorded_step: Option<u64>,
    /// See [`min_recorded_step`](Self::min_recorded_step).
    pub max_recorded_step: Option<u64>,
    /// The WORKER trace-manifest ranks this capture will attach, SORTED,
    /// DEDUPED, sentinel EXCLUDED — the exact slice
    /// [`crate::trace_ring::first_rank_manifest_gap`] takes.
    ///
    /// A ROSTER rather than a pre-computed gap, so the judge asks the shared
    /// predicate itself rather than trusting a caller to have asked it — which
    /// is what keeps the two sides one rule instead of two.
    ///
    /// Empty means "this capture attaches no worker manifest". That is NOT
    /// judged here: it lands on the trace-presence arm above (a capture with no
    /// manifest carries no trace records either), and `run_replay` refuses it
    /// separately as `BagMissingAttachment`.
    pub worker_manifest_ranks: &'a [u32],
    /// Whether [`required_nodes`](Self::required_nodes) is a KNOWN set.
    ///
    /// `false` means the capture could not resolve one — a multi-trace-ring
    /// recorder cannot map a FIRE record's `node_idx` to a node without the
    /// record's own rank, which it does not demux, so it declines and hands over
    /// an EMPTY list. Empty and UNKNOWN are then the same value with opposite
    /// meanings: empty says "no node executed, so no anchor is required" and
    /// satisfies the coverage gate vacuously, while unknown says "the nodes the
    /// replay will execute are not enumerable here". Reading the second as the
    /// first let a multi-rank capture claim `resimmable` with no coverage check
    /// performed at all, against a replay that goes on to execute the trace's
    /// nodes.
    pub required_nodes_known: bool,
    /// The FIRST record the replay gate would refuse, in FILE ORDER — of EITHER
    /// kind.
    ///
    /// `run_replay` walks every trace record once and refuses on five things: a
    /// departure/fault boundary, then a foreign rank stamp, a FIRE `node_idx`
    /// past its rank's manifest, a zeroed `record_type` 0, and a
    /// reserved/unknown kind (`replay_cmd.rs` 628-722). The judge counted
    /// records and derived nodes but asked NONE of them, so a malformed or
    /// future-written trace was stamped `resimmable: true` and then refused at
    /// the gate.
    ///
    /// Carries the KIND as well as the fault
    /// ([`crate::trace_ring::TraceRecordRefusal`]) because the two kinds are not
    /// ranked against each other — the record that comes FIRST is the one the
    /// gate reaches first, so a fault preceding a departure is a
    /// `TraceRecordRejected`, and a departure preceding a fault is fault replay.
    ///
    /// Supplied by [`crate::trace_ring::first_trace_record_refusal`], which is
    /// the SAME predicate in the SAME order the gate runs — the point of the
    /// shared function being that a refusal added to one cannot go missing from
    /// the other.
    pub first_record_refusal: Option<crate::trace_ring::TraceRecordRefusal>,
    /// **Scheduler-trace rings this run DECLARED to the recorder**.
    ///
    /// Reported beside [`trace_rings_unreadable`](Self::trace_rings_unreadable)
    /// so a refusal can state the shortfall — "1 of 2" — rather than merely that
    /// one exists.
    pub trace_rings_declared: usize,
    /// **Declared trace rings this recorder has NO READER for**.
    ///
    /// A recorder-only fact, like [`trace_gap`](Self::trace_gap) and for the same
    /// reason: the replayer re-measures every other fact off the bag, and a ring
    /// that was never read leaves NOTHING in the bag to measure. What lands is a
    /// trace that looks whole — every record it carries is well formed, the
    /// boundaries line up, the manifests it does carry are contiguous — while an
    /// entire rank's fires are simply absent. Nothing a reader can compute
    /// distinguishes that from a run where those nodes never fired.
    ///
    /// Two ways to get one, both already reported elsewhere by the recorder and
    /// neither previously reaching this verdict: a DECLARED ring that could not
    /// be opened at all (`bagd`'s attach path warns and continues, so one
    /// vanished ring costs its trace rather than the whole recording —
    /// `BagdSummary::rings_unavailable`), and a ring LOST mid-run under the drain
    /// (a decode/open failure that is not a lap, so there is no live cursor to
    /// re-attach to — `TraceDrainState::rings_retired`).
    ///
    /// A LAP is NOT one of them: the drain re-opens at the live cursor and keeps
    /// reading, which is a HOLE ([`trace_gap`](Self::trace_gap)) with its own
    /// arm and its own refusal scope: only the capture straddling it is
    /// refused.
    pub trace_rings_unreadable: usize,
    /// State rings the bag's own coverage manifest declares.
    pub state_rings_declared: usize,
    /// The run ids that have an anchor at the anchor step.
    ///
    /// More than one is `AmbiguousRun`: a bag holding two runs' anchors at one
    /// step cannot say which run a record belongs to.
    pub anchor_run_ids_at_anchor_step: usize,
    /// The run the anchor belongs to, as `resolve_run_at` would resolve it.
    pub anchor_run_id: u64,
    /// The nodes the replayed suffix EXECUTES — resim's `fires.executed`.
    ///
    /// Not the graph's node list: an anchor only has to cover what actually
    /// fires after the resume point, and a recorder can read
    /// exactly that set off the trimmed trace's own FIRE records.
    pub required_nodes: &'a [String],
    /// What the embedded checkpoint covers, one fact per node.
    pub anchor_facts: &'a [AnchorFact],
}

/// A HOLE a recorder observed in its OWN read of a run's scheduler-trace ring.
///
/// # Why this fact can only come from the recorder
///
/// Every other field of [`ResimFacts`] is re-measurable from the bag: the
/// replayer opens the same recording and counts the same records. This one is
/// not. A trace ring is `OverrunPolicy::FailLoud`, so when a reader falls far
/// enough behind that the producer laps it, the records in between are gone —
/// and what lands in the bag afterwards is a stream that LOOKS continuous. The
/// steps either side of the hole are ordinary boundaries, the fires hang off
/// them correctly, and nothing a reader can compute distinguishes "the graph did
/// not fire for 300 steps" from "this recorder was not there for 300 steps".
///
/// So the recorder that watched the hole open is the only party that can report
/// it, and if it does not, the bag's verdict is confident and false. That is the
/// same class [`ResimFacts::graph_attachment_present`] closes from the other
/// direction, and it is why the judge takes this as a FACT rather than deriving
/// it.
///
/// # It is a gap in the READER, not in the run
///
/// The producer lost nothing: the graph kept stepping and kept pushing. What was
/// lost is one reader's view of it, which is why the remedy is about the
/// recorder (a stall it must not have) and never about the graph.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TraceGap {
    /// The last step this recorder ADMITTED before the hole.
    ///
    /// Always known: a hole is observed by a reader that had already read
    /// something, and every trace record carries a step. A reader that was
    /// lapped before its FIRST read has no "before" at all — that is a late
    /// ATTACH, not a gap, and it is reported as one.
    pub last_step_before: u64,
    /// The first step it admitted after re-attaching at the live cursor.
    ///
    /// `None` while nothing has arrived since — which is not a missing value but
    /// a real state, and one with a consequence: a capture cannot hold a record
    /// on the far side of a hole nothing has yet crossed, so an unclosed gap
    /// spans nothing. See [`spans`](Self::spans).
    pub first_step_after: Option<u64>,
}

impl TraceGap {
    /// Does this hole fall INSIDE a record stream whose steps run
    /// `min_step..=max_step`?
    ///
    /// The question is deliberately about the records a capture CARRIES rather
    /// than about the window it covers or the instant the hole opened, and that
    /// choice is what makes the answer exact instead of merely conservative. A
    /// capture holds a hole iff it holds records on BOTH sides of it: one at or
    /// before [`last_step_before`](Self::last_step_before) and one at or after
    /// [`first_step_after`](Self::first_step_after). A capture whose whole
    /// window sits after the re-attach holds only the far side and is
    /// continuous — which is the property that keeps ONE recorder stall from
    /// condemning every later capture of the run (the rule: a hole is
    /// refused, a run is not).
    ///
    /// `None` on either bound is "this capture carries no records", and an
    /// unclosed gap ([`first_step_after`](Self::first_step_after) `== None`)
    /// spans nothing for the reason given there. Both answer `false`, and in
    /// both cases the capture is refused by an arm that is actually about it.
    pub fn spans(&self, min_step: Option<u64>, max_step: Option<u64>) -> bool {
        let (Some(min), Some(max), Some(after)) = (min_step, max_step, self.first_step_after)
        else {
            return false;
        };
        min <= self.last_step_before && max >= after
    }
}

/// WHY a capture cannot be resimmed.
///
/// Each variant names the resim refusal it corresponds to, because the operator
/// reading a manifest and the operator reading a refused `bag play --resim` are
/// the same person and should not have to translate.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ResimGap {
    /// No scheduler trace — resim's `BagNoSchedulerTrace`.
    NoTrace,
    /// A trace record the replay gate refuses outright.
    ///
    /// Carries the rendered detail rather than the fault, so the gap stays a
    /// plain reason string like its six siblings.
    TraceRecordRejected {
        /// The fault, already rendered.
        detail: String,
    },
    /// The capture cannot enumerate the nodes its trace executes.
    ///
    /// A multi-trace-ring recorder does not demux `node_idx` by rank, so it can
    /// neither name those nodes nor check an anchor covers them.
    AmbiguousNodeMap,
    /// No usable `graph.yaml` attachment — resim's `BagMissingAttachment`.
    ///
    /// The bag cannot say WHAT to re-execute, so nothing after this matters.
    NoGraph,
    /// The recorder watched a HOLE open in its own read of the trace ring, and
    /// this capture carries records on both sides of it.
    ///
    /// It has no `run_replay` counterpart, because the replayer cannot see it —
    /// see [`TraceGap`] for why. The numbers are the hole's own edges.
    TraceLapped {
        /// The last step the recorder admitted before the hole.
        last_step_before: u64,
        /// The first step it admitted after re-attaching.
        first_step_after: Option<u64>,
    },
    /// A trace ring this run DECLARED that the recorder never read.
    ///
    /// Like [`TraceLapped`](Self::TraceLapped) it has no `run_replay`
    /// counterpart, and for the same reason: what is missing left no trace of
    /// itself in the bag. See
    /// [`ResimFacts::trace_rings_unreadable`] for the two ways one arises.
    TracePartial {
        /// Trace rings this run declared to the recorder.
        declared: usize,
        /// How many of them it has no reader for.
        unreadable: usize,
    },
    /// The worker trace manifests are not contiguous `0..=max` — resim's
    /// `MultiRankManifestGap`.
    ///
    /// Carries the FIRST missing rank, which is what that error names.
    RankManifestGap {
        /// The first worker rank with no manifest.
        missing: u32,
    },
    /// The trace carries a departure/fault boundary — resim's
    /// `DegradedRecordingDeparture`.
    FaultReplay {
        /// How many departure records the trace carries.
        departures: usize,
    },
    /// The trace carries records but no step boundary — resim's
    /// `BagNoStepBoundaries`.
    NoBoundary,
    /// Several state rings, so a record's `node_idx` names a different node in
    /// each — resim's `MultiRingAmbiguous`.
    MultiRing {
        /// How many rings the coverage manifest declares.
        rings: usize,
    },
    /// Two runs' anchors at the resume step — resim's `AmbiguousRun`.
    AmbiguousRun {
        /// How many runs have an anchor there.
        runs: usize,
    },
    /// The anchor does not cover every executed node — resim's
    /// `AnchorIncomplete`, judged by the SAME [`plan_restore`] resim calls.
    AnchorIncomplete {
        /// `plan_restore`'s own refusal, rendered.
        detail: String,
    },
}

impl ResimGap {
    /// The manifest's `resimmable_reason`, and the sentence an operator reads.
    ///
    /// Every arm names what is missing AND what would have to change, because a
    /// verdict an operator cannot act on is a verdict that reads as the feature
    /// being broken.
    pub fn reason(&self) -> String {
        match self {
            Self::NoTrace => "no scheduler trace in this capture: a resume derives its step \
                              from the trace, and this capture carries no trace records. That \
                              is either a recorder that held no trace ring or a ring that \
                              yielded nothing — the two are indistinguishable from the bag, so \
                              this says only what is true of both. For a `graph run` capture \
                              the run's `run.json` records which (a run that declined its \
                              rings, one that could not have them, or a single-process shape \
                              that mints none); a standalone `cerulion bagd` capture writes no \
                              such manifest and has no further answer to give. The state is in \
                              the bag and readable; `bag play --resim` cannot use it"
                .to_string(),
            Self::TraceRecordRejected { detail } => format!(
                "{detail}. `cerulion bag play --resim` refuses a trace on that record before it \
                 loads anything, so this capture cannot be re-executed. The frames, the state \
                 and the trace are all in the bag and readable"
            ),
            Self::AmbiguousNodeMap => "this capture's recorder held SEVERAL trace rings and does \
                                       not demux a record's rank, so it cannot say which nodes \
                                       its trace executes — and therefore cannot check that its \
                                       checkpoint covers them. The frames, the state and the \
                                       trace are all in the bag and readable"
                .to_string(),
            Self::NoGraph => {
                "this capture carries no usable `graph.yaml` attachment, so nothing says \
                              which graph to re-execute — a resume needs the graph the frames \
                              were produced by. A `cerulion graph run --record` recorder is \
                              always handed one; a standalone `cerulion bagd` is only handed \
                              what its `--attach` names. The frames and the state are in the bag \
                              and readable"
                    .to_string()
            }
            Self::TraceLapped {
                last_step_before,
                first_step_after,
            } => {
                let far = first_step_after
                    .map(|s| s.to_string())
                    .unwrap_or_else(|| "?".to_string());
                format!(
                    "this capture's scheduler trace has a HOLE in it: the recorder fell far \
                     enough behind this run's trace ring that the producer lapped it, losing \
                     every record between step {last_step_before} and step {far}, and this \
                     capture carries records on both sides of that hole. A resume re-executes \
                     from a step skeleton, so a skeleton with a gap in it would replay a \
                     different run — the frames, the state and the trace it does have are all in \
                     the bag and readable. Later captures of this run, whose window begins after \
                     the recorder re-attached, are unaffected"
                )
            }
            Self::TracePartial {
                declared,
                unreadable,
            } => format!(
                "this capture's recorder was handed {declared} scheduler-trace ring(s) and has no \
                 reader for {unreadable} of them — either the ring could not be opened (the run \
                 may have been exiting: a ring's SHM name is unlinked when its owner drops) or it \
                 was lost mid-run. Every record those rings carried is therefore missing from this \
                 capture, and nothing in the bag says so: what it holds looks continuous. A resume \
                 re-executes from a step skeleton, so a skeleton missing a worker's fires would \
                 replay a different run. The frames, the state and the trace it does have are all \
                 in the bag and readable; the recorder's log names each unreadable ring and why"
            ),
            Self::RankManifestGap { missing } => format!(
                "this capture's worker trace manifests are not contiguous — rank {missing} has \
                 none, and `cerulion bag play --resim` refuses a bag with a hole in its worker \
                 roster before it loads anything. The frames, the state and the trace are all in \
                 the bag and readable"
            ),
            Self::FaultReplay { departures } => format!(
                "this capture's trace carries {departures} departure (fault) boundary record(s) \
                 — a peer worker was lost mid-run. Replaying a fault-degraded recording is not \
                 supported yet; the frames and the state are in the bag and readable"
            ),
            Self::NoBoundary => "this capture's trace carries no step-boundary record, so nothing \
                                 says which step a resume would begin at. The frames and the \
                                 state are in the bag and readable"
                .to_string(),
            Self::MultiRing { rings } => format!(
                "this capture drained {rings} state rings, and a state record carries a node \
                 index but no rank — every ring numbers its own nodes from 0, so nothing in the \
                 bag says which ring a record came from. Capture a single-process run \
                 to get a resimmable bag"
            ),
            Self::AmbiguousRun { runs } => format!(
                "{runs} runs have an anchor at this capture's resume step, and nothing in the bag \
                 says which run a record belongs to"
            ),
            Self::AnchorIncomplete { detail } => format!(
                "this capture's anchor does not cover every node the resumed window executes: \
                 {detail}"
            ),
        }
    }
}

/// The sentence a RESIMMABLE capture carries.
///
/// Its own constant rather than a literal at the render site, so the positive
/// claim is as reviewable as the six negative ones.
pub const RESIMMABLE_FROM_START_REASON: &str =
    "this capture's window reaches its run's own step 0, so it carries a scheduler trace and \
     needs no checkpoint: `cerulion bag play <bag> --resim all` re-executes it from the start";

/// The sentence a RESIMMABLE capture carries when it resumes FROM AN ANCHOR.
pub const RESIMMABLE_REASON: &str =
    "this capture carries a scheduler trace trimmed to its anchor and a complete checkpoint: \
     `cerulion bag play <bag> --resim all` resumes from it";

/// The clause a resimmable capture appends to name its COVERED RANGE.
///
/// A capture's frame window and its trace window are closed by two different
/// threads with no rendezvous between them (see
/// `cerulion_bagd::trace_window::TrimmedTrace::last_boundary_target_ns`), so the
/// frames routinely run a little past the last boundary the bag carries. The
/// decision is that the bag keeps every frame and the claim names the
/// prefix a resume can actually cover — so the positive verdict says how far it
/// reaches, and says plainly that the remainder is present and simply outside
/// the range.
///
/// Its own constant so the sentence is as reviewable as the seven refusals, and
/// composed by [`resimmable_reason`] so no render site can state the verdict
/// without it.
pub const RESIM_COVERED_THROUGH_CLAUSE: &str =
    "; that resume covers this recording through gating-clock instant";

/// The reason a RESIMMABLE capture carries, with its covered range named.
///
/// One function rather than two constants at the render site, because the range
/// clause has to attach to BOTH positive sentences and a renderer that appended
/// it to one of them would publish a verdict whose scope depends on which arm it
/// took.
///
/// `covered_through_ns` is `None` when the capture carries no
/// authoritative-rank step boundary to end a range at. That capture is refused
/// separately (`ResimGap::NoBoundary`), so the arm is reachable only from a
/// from-start claim whose trace the trim kept whole — and the base sentence
/// alone is then the accurate one: it promises re-execution and states no range
/// it did not measure.
pub fn resimmable_reason(from_start: bool, covered_through_ns: Option<u64>) -> String {
    let base = if from_start {
        RESIMMABLE_FROM_START_REASON
    } else {
        RESIMMABLE_REASON
    };
    match covered_through_ns {
        Some(ns) => format!(
            "{base}{RESIM_COVERED_THROUGH_CLAUSE} {ns} ns (the last step boundary this capture's \
             trace carries). Frames published after that instant are IN the bag and readable — \
             they are outside the range a resume re-executes, because the capture's frame window \
             and its trace window are closed independently"
        ),
        None => base.to_string(),
    }
}

/// PURE: may this capture claim `resimmable`?
///
/// `Ok(())` means yes. The arms are asked in `run_replay`'s own order — see the
/// module docs on why that is a correctness property rather than a style.
///
/// # The step-0 arm is a genuine YES, not an oversight
///
/// `resolve_resume` returns `FromStart` when the first recorded boundary is step
/// 0: the constructor's state IS the state, so no anchor is needed and none is
/// looked for. A capture whose window happens to reach the run's own first step
/// is therefore resimmable with no checkpoint at all, and refusing it for a
/// missing anchor would be refusing a bag resim would accept.
pub fn judge_resimmable(facts: &ResimFacts<'_>) -> Result<(), ResimGap> {
    // 1. `BagNoSchedulerTrace` — the first thing `run_replay` refuses on.
    if facts.trace_records == 0 {
        return Err(ResimGap::NoTrace);
    }
    // 2. `BagMissingAttachment` for `graph.yaml` — `run_replay`'s gate 4, which
    //    sits between the trace-presence gate above and the record-type walk
    //    below. Asked in that position so a capture reports the gap resim would
    //    report FIRST.
    if !facts.graph_attachment_present {
        return Err(ResimGap::NoGraph);
    }
    // 2b. `MultiRankManifestGap` — `run_replay`'s gate 5, which loads the trace
    //     manifests AFTER the graph gate and BEFORE it walks a single record
    //     (`replay_cmd.rs`: the graph gate at 4, `load_trace_manifests` at 5,
    //     the record walk after it). So a bag with a hole in its worker roster
    //     is refused there before the walk can reach any record-level fault, and
    //     the judge must reach it at the same point or report a gap resim never
    //     gets to.
    //
    //     Asked through the SHARED predicate rather than re-derived, because the
    //     capture side's own rank table ZERO-FILLS a gap by design: every record
    //     resolves against it, the record walk finds nothing to refuse, and a
    //     `{0, 7}` capture stamped `resimmable: true` against a bag resim
    //     refuses outright. See `trace_ring::first_rank_manifest_gap`.
    if let Some(missing) = crate::trace_ring::first_rank_manifest_gap(facts.worker_manifest_ranks) {
        return Err(ResimGap::RankManifestGap { missing });
    }
    // 2c. `TraceLapped` — the recorder's OWN observation, which has
    //     no `run_replay` counterpart at all: the replayer re-measures every
    //     other fact here off the bag, and a lapped trace is precisely the one
    //     it cannot (see [`TraceGap`]).
    //
    //     Asked HERE, and the position is the whole of the argument. Gates 1,
    //     2 and 2b are about the bag's INPUTS — is there a trace, is there a
    //     graph, is the worker roster whole — and every one of them is a
    //     question resim itself asks and answers before it walks a record, so
    //     each keeps its place and a capture still reports the gap resim would
    //     report first. Everything BELOW reasons over the record stream: the
    //     walk classifies records, the boundary arm resolves a resume from
    //     them, the node map is derived from their FIRE entries and the anchor
    //     is judged against that map. A stream with a hole in it makes each of
    //     those answers UNSOUND rather than merely incomplete — the walk finds
    //     nothing to refuse, a boundary is present, the map resolves — so a
    //     hole must be named before the first arm that would be computed from
    //     it, or the capture reports a downstream gap that is an artifact of
    //     the hole, or none at all.
    if let Some(gap) = facts.trace_gap {
        // Only a capture that CARRIES both sides of the hole holds it. The
        // bounds come from the trimmed trace's own records, so a capture whose
        // window begins after the recorder re-attached is continuous and is not
        // condemned by a stall it was not there for (a hole is refused, a run is not).
        if gap.spans(facts.min_recorded_step, facts.max_recorded_step) {
            return Err(ResimGap::TraceLapped {
                last_step_before: gap.last_step_before,
                first_step_after: gap.first_step_after,
            });
        }
    }
    // 2d. `TracePartial` — the other fact only the
    //     recorder can report, and the one the lap arm's neighbours made easy to
    //     miss: a ring it never read at all.
    //
    //     A lap is bounded — the drain re-attaches and the hole has two edges,
    //     so the lap arm refuses exactly the capture that straddles it. A ring
    //     that was never opened, or that was lost with no live cursor to
    //     re-attach to, has no edges: its records are missing from EVERY capture
    //     the recorder takes, and the stream that does land looks continuous.
    //
    //     BESIDE the lap arm rather than up with gates 1-2b, for the lap arm's
    //     own reason. Those gates mirror `run_replay`'s order so a capture
    //     reports the gap resim would report first; this fact resim cannot see
    //     at all. Everything BELOW reasons over the record stream — the walk,
    //     the boundary, the node map, the anchor — and a stream missing a whole
    //     rank makes each of those answers unsound rather than incomplete, so it
    //     must be named before the first arm computed from it.
    //
    //     AFTER the lap: both are refusals, and a hole has EDGES an operator can
    //     act on, so when a recorder managed both it names the sharper one.
    if facts.trace_rings_unreadable > 0 {
        return Err(ResimGap::TracePartial {
            declared: facts.trace_rings_declared,
            unreadable: facts.trace_rings_unreadable,
        });
    }
    // 3. The RECORD WALK — `DegradedRecordingDeparture` and the four
    //    record-level faults, in ONE arm because `run_replay` reaches them in
    //    ONE walk. Before the boundary check, exactly as `run_replay` walks the
    //    records before it resolves a resume: a fault recording is refused
    //    whether or not its trace is otherwise well formed.
    //
    //    The gap named here belongs to the FIRST refusing record in file order,
    //    which is the record the gate reaches first — so it is the gap the gate
    //    names, not merely also-a-refusal. Two arms ranked by KIND (departure
    //    count first, then the fault) would answer differently on a trace
    //    carrying both, and did: see the module docs.
    if let Some(refusal) = &facts.first_record_refusal {
        use crate::trace_ring::TraceRecordRefusal;
        return Err(match refusal {
            // The COUNT is for the message only; the walk already proved at
            // least one departure exists, so a zero here is a caller whose two
            // measurements disagree and must not render as "0 departure(s)".
            TraceRecordRefusal::Departure => ResimGap::FaultReplay {
                departures: facts.departure_records.max(1),
            },
            TraceRecordRefusal::Fault(fault) => ResimGap::TraceRecordRejected {
                detail: fault.detail(),
            },
        });
    }
    // 3b. FAIL-CLOSED backstop for facts whose two halves disagree — a caller
    //     that measured departures but supplied no walk. Unreachable when both
    //     are measured over one record set (which is the only way the recorder
    //     builds them), and BELOW the record-ordered arm on purpose: above it,
    //     it would restore exactly the aggregate-first precedence that was
    //     removed here. Never silently resimmable.
    if facts.departure_records > 0 {
        return Err(ResimGap::FaultReplay {
            departures: facts.departure_records,
        });
    }
    // 4. Is there a resume point at all?
    let Some(first_recorded_step) = facts.first_recorded_step else {
        return Err(ResimGap::NoBoundary);
    };
    if first_recorded_step == 0 {
        // FromStart — see the doc comment. No anchor is consulted.
        return Ok(());
    }
    // 5. `MultiRingAmbiguous`. A property of the MANIFEST, so it is asked before
    //    anything about a particular step's records — the same ordering
    //    `read_bag_anchors` states for itself.
    if facts.state_rings_declared > 1 {
        return Err(ResimGap::MultiRing {
            rings: facts.state_rings_declared,
        });
    }
    // 5b. The node map, and it belongs BELOW the ring count — this is the
    //     third time this arm has moved, so the reasoning is worth stating in
    //     full rather than by reference.
    //
    //     `run_replay` has NO node-map gate of its own: it demuxes by rank
    //     because it HAS the rank tables (`load_rank_tables`), so the question
    //     this arm answers is a RECORDER limitation, not a replay one. That
    //     makes its position a pure question of "what does resim refuse FIRST on
    //     a bag that trips both?" — and the answer is `read_bag_anchors`, which
    //     refuses `rings_declared > 1` at its first statement
    //     (`replay_state.rs`), reached from `resolve_resume` BEFORE
    //     `resolve_run_at` or `plan_restore` (`replay_engine.rs`).
    //
    //     A multi-PROCESS capture trips both at once — several trace rings leave
    //     the node map unresolvable AND several state rings are declared — so
    //     this is the ordinary shape rather than a corner: the judge reported
    //     `AmbiguousNodeMap` while the replay it predicts refuses
    //     `MultiRingAmbiguous`, two different first refusals for one bag, which
    //     is the divergence this shared judge exists to prevent.
    //
    //     Its previous home (above the ring count) was argued on the grounds
    //     that "the node map is what the anchor-coverage questions are phrased
    //     in, so it sits with the other manifest properties that gate them". The
    //     ring count is one of those same manifest properties and resim asks it
    //     first, so that argument places this arm here just as well — the two
    //     are adjacent either way, and only one order matches the replay.
    if !facts.required_nodes_known {
        return Err(ResimGap::AmbiguousNodeMap);
    }
    // 6. `AmbiguousRun`.
    if facts.anchor_run_ids_at_anchor_step > 1 {
        return Err(ResimGap::AmbiguousRun {
            runs: facts.anchor_run_ids_at_anchor_step,
        });
    }
    // 7. `AnchorIncomplete` — judged by resim's OWN function, not by a second
    //    implementation of its rule. This is the whole point of the module.
    plan_restore(&RestoreRequest {
        run_id: facts.anchor_run_id,
        required_nodes: facts.required_nodes,
        first_recorded_step,
        facts: facts.anchor_facts,
    })
    .map(|_| ())
    .map_err(|refusal| ResimGap::AnchorIncomplete {
        detail: refusal.to_string(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state_restore::AnchorOutcome;
    use crate::trace_ring::{TraceRecordFault, TraceRecordRefusal};

    const RUN: u64 = 0xC1E5;

    /// A record-level fault the walk would report — spelled once so the arms
    /// read as "the first refusing record was a fault" rather than as a type
    /// path.
    fn fault(record_type: u32) -> Option<TraceRecordRefusal> {
        Some(TraceRecordRefusal::Fault(
            TraceRecordFault::UnsupportedRecordType { record_type },
        ))
    }

    fn fact(node: &str, step: u64, outcome: AnchorOutcome) -> AnchorFact {
        AnchorFact {
            run_id: RUN,
            step,
            node: node.to_string(),
            outcome,
        }
    }

    /// A capture that passes every arm.
    fn healthy<'a>(nodes: &'a [String], anchors: &'a [AnchorFact]) -> ResimFacts<'a> {
        ResimFacts {
            // A contiguous single-worker roster — the ordinary shape,
            // so the new gate is inert unless an arm says otherwise.
            worker_manifest_ranks: &[0],
            trace_records: 12,
            departure_records: 0,
            first_recorded_step: Some(41),
            // One declared ring, and this recorder is reading it — so
            // the partial-trace gate is inert unless an arm says otherwise.
            trace_rings_declared: 1,
            trace_rings_unreadable: 0,
            state_rings_declared: 1,
            anchor_run_ids_at_anchor_step: 1,
            anchor_run_id: RUN,
            required_nodes: nodes,
            anchor_facts: anchors,
            graph_attachment_present: true,
            required_nodes_known: true,
            first_record_refusal: None,
            // A CONTINUOUS trace — no recorder stall — so the lap gate
            // is inert unless an arm says otherwise. The bounds bracket
            // `first_recorded_step` because a real trace's boundary sits inside
            // the record stream, never outside it.
            trace_gap: None,
            min_recorded_step: Some(41),
            max_recorded_step: Some(45),
        }
    }

    /// The positive sentence names the COVERED RANGE, in both shapes,
    /// and says nothing about a range it was not given.
    ///
    /// One composer rather than two constants at the render site, so this arm is
    /// what stops the claim's SCOPE depending on which arm a renderer took: a
    /// from-start capture and an anchored one make different promises about
    /// state, and the identical promise about how far the promise reaches.
    #[test]
    fn the_resimmable_sentence_names_the_range_it_covers_in_both_shapes() {
        for from_start in [true, false] {
            let with = resimmable_reason(from_start, Some(16_102_963_042));
            assert!(
                with.contains("16102963042 ns"),
                "from_start={from_start}: the range is stated: {with}"
            );
            assert!(
                with.contains("outside the range"),
                "from_start={from_start}: …and the remainder is named as PRESENT but outside \
                 it — a claim that only said where it stopped would read as data loss: {with}"
            );
            // The base sentence SURVIVES: the clause qualifies the promise, it
            // does not replace it. Each shape keeps its own — the from-start one
            // must never tell a reader to inspect a checkpoint it does not hold.
            let base = if from_start {
                RESIMMABLE_FROM_START_REASON
            } else {
                RESIMMABLE_REASON
            };
            assert!(
                with.starts_with(base),
                "from_start={from_start}: the base sentence is kept verbatim: {with}"
            );

            // …and with NO range measured, the sentence claims none. Reachable
            // only for a capture whose trace carries no authoritative-rank
            // boundary — which `NoBoundary` refuses — so the correct rendering is
            // the base sentence alone rather than a fabricated endpoint.
            let without = resimmable_reason(from_start, None);
            assert_eq!(without, base);
            assert!(!without.contains(RESIM_COVERED_THROUGH_CLAUSE));
        }
    }

    /// THE positive arm — without it every refusal test is satisfied by a judge
    /// that refuses everything.
    #[test]
    fn a_trimmed_trace_with_a_complete_anchor_is_resimmable() {
        let nodes = vec!["a".to_string(), "b".to_string()];
        let anchors = vec![
            fact("a", 40, AnchorOutcome::Complete),
            fact("b", 40, AnchorOutcome::Complete),
        ];
        assert_eq!(judge_resimmable(&healthy(&nodes, &anchors)), Ok(()));
    }

    /// The earlier claim, still true and still first: no trace, no resume.
    #[test]
    fn a_capture_with_no_trace_is_refused_before_anything_else_is_examined() {
        let nodes = vec!["a".to_string()];
        // Every OTHER fact is also broken, so this pins the ORDER as well as the
        // arm: a judge that asked them in any other sequence would report a
        // different gap.
        let facts = ResimFacts {
            // A contiguous single-worker roster — the ordinary shape,
            // so the new gate is inert unless an arm says otherwise.
            worker_manifest_ranks: &[0],
            trace_records: 0,
            departure_records: 3,
            first_recorded_step: None,
            // A ring nobody read, too — so this also pins that the
            // partial-trace gate does not preempt the no-trace one.
            trace_rings_declared: 2,
            trace_rings_unreadable: 1,
            state_rings_declared: 4,
            anchor_run_ids_at_anchor_step: 2,
            anchor_run_id: RUN,
            required_nodes: &nodes,
            anchor_facts: &[],
            // Broken too, and deliberately: the graph gate is asked immediately
            // AFTER this one, so it is the nearest thing to a false positive
            // this arm can carry — and the node-map gate is right behind it.
            graph_attachment_present: false,
            required_nodes_known: false,
            // A zeroed record FIRST, three departures behind it — a consistent
            // pair, since the walk stops at the first refusal while the count
            // spans the whole trace.
            first_record_refusal: Some(TraceRecordRefusal::Fault(TraceRecordFault::ZeroedRecord)),
            // Broken too. A hole is a fact about a record stream, and
            // a capture with NO records cannot be holding one — so a judge that
            // asked the lap gate before the trace gate would name a hole in an
            // empty bag.
            trace_gap: Some(TraceGap {
                last_step_before: 40,
                first_step_after: Some(97),
            }),
            min_recorded_step: Some(38),
            max_recorded_step: Some(99),
        };
        assert_eq!(judge_resimmable(&facts), Err(ResimGap::NoTrace));
    }

    /// The graph gate sits between the trace gate and the record walk — asserted
    /// as a POSITION, not merely as a reachable arm.
    ///
    /// A review added `NoGraph` "in `run_replay`'s own gate order (after the
    /// trace, before the record walk)", because a capture should report the gap
    /// resim would report FIRST; a capture naming a later gap sends its reader
    /// to fix something that was never the blocker. The e2e arm proves the gap is
    /// REACHED. This one proves WHERE: a bag with a graph attachment and no
    /// trace is `NoTrace` (the arm above), and one with a trace, no graph, and
    /// every LATER fact broken is `NoGraph` rather than any of them.
    #[test]
    fn a_capture_with_no_graph_is_refused_before_every_gate_below_it() {
        let nodes = vec!["a".to_string()];
        let facts = ResimFacts {
            // A contiguous single-worker roster — the ordinary shape,
            // so the new gate is inert unless an arm says otherwise.
            worker_manifest_ranks: &[0],
            trace_records: 12,
            graph_attachment_present: false,
            // Everything the judge asks AFTER the graph gate, broken.
            // Including a ring nobody read.
            trace_rings_declared: 2,
            trace_rings_unreadable: 1,
            required_nodes_known: false,
            first_record_refusal: Some(TraceRecordRefusal::Fault(TraceRecordFault::ZeroedRecord)),
            departure_records: 3,
            first_recorded_step: None,
            state_rings_declared: 4,
            anchor_run_ids_at_anchor_step: 2,
            anchor_run_id: RUN,
            required_nodes: &nodes,
            anchor_facts: &[],
            // And the lap gate, which sits below the graph gate — a
            // capture missing its graph reports THAT, because it is the gap
            // `run_replay` itself would reach first and the one an operator can
            // act on.
            trace_gap: Some(TraceGap {
                last_step_before: 40,
                first_step_after: Some(97),
            }),
            min_recorded_step: Some(38),
            max_recorded_step: Some(99),
        };
        assert_eq!(judge_resimmable(&facts), Err(ResimGap::NoGraph));
    }

    /// A capture that carries records on BOTH sides of a hole the
    /// recorder watched open is refused — and one taken after the re-attach is
    /// NOT, which is the half the rule turns on.
    ///
    /// Two verdicts off ONE gap, so this is not merely "the arm is reachable":
    /// the difference between them is the carried step range and nothing else,
    /// which is exactly the rule. A judge that refused on the mere PRESENCE of a
    /// lap would condemn every later capture of a run that stalled once — the
    /// permanent-retire behaviour the design rejects — and would pass the first
    /// assertion here while failing the second.
    #[test]
    fn a_capture_carrying_both_sides_of_a_hole_is_refused_and_a_later_one_is_not() {
        let nodes = vec!["a".to_string()];
        let gap = TraceGap {
            last_step_before: 40,
            first_step_after: Some(97),
        };
        // Straddling: records at 38 and at 99, with 41..=96 missing. `healthy`'s
        // resume is step 41 off an anchor at 40, which is the ordinary shape.
        let straddling = vec![fact("a", 40, AnchorOutcome::Complete)];
        assert_eq!(
            judge_resimmable(&ResimFacts {
                trace_gap: Some(gap),
                min_recorded_step: Some(38),
                max_recorded_step: Some(99),
                ..healthy(&nodes, &straddling)
            }),
            Err(ResimGap::TraceLapped {
                last_step_before: 40,
                first_step_after: Some(97),
            })
        );
        // The SAME run, the SAME hole, a capture whose window begins after the
        // recorder re-attached — resuming at 97 off an anchor at 96, so the ONLY
        // difference from the arm above is the range it carries.
        let later = vec![fact("a", 96, AnchorOutcome::Complete)];
        assert_eq!(
            judge_resimmable(&ResimFacts {
                trace_gap: Some(gap),
                min_recorded_step: Some(97),
                max_recorded_step: Some(140),
                first_recorded_step: Some(97),
                ..healthy(&nodes, &later)
            }),
            Ok(())
        );
    }

    /// An UNCLOSED hole refuses nothing.
    ///
    /// The state every hole is in for at least one drain pass: the recorder has
    /// re-attached and nothing has yet crossed the far side. A capture taken in
    /// that window carries only the near side, so it holds no hole — and a judge
    /// that treated `None` as "unknown, therefore refuse" would fail every
    /// capture taken in the seconds after a stall on a run that was fine.
    #[test]
    fn an_unclosed_hole_refuses_nothing() {
        let nodes = vec!["a".to_string()];
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        assert_eq!(
            judge_resimmable(&ResimFacts {
                trace_gap: Some(TraceGap {
                    last_step_before: 40,
                    first_step_after: None,
                }),
                // A range that WOULD straddle the hole if the far edge were
                // known — so this arm is about the `None`, not about the bounds.
                min_recorded_step: Some(38),
                max_recorded_step: Some(99),
                ..healthy(&nodes, &anchors)
            }),
            Ok(())
        );
    }

    /// The lap gate's POSITION — below the roster gate, above the
    /// record walk.
    ///
    /// Asserted as a position rather than as a reachable arm, for the reason
    /// every other precedence arm in this file is. The rule the position encodes:
    /// gates 1, 2 and 2b ask about the bag's INPUTS and are gates `run_replay`
    /// itself reaches, so they keep their places and a capture still reports the
    /// gap resim would report first; everything BELOW is computed FROM the record
    /// stream, and a stream with a hole in it makes those answers unsound rather
    /// than merely incomplete.
    #[test]
    fn the_lap_gate_sits_below_the_roster_gate_and_above_the_record_walk() {
        let nodes = vec!["a".to_string()];
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let gap = TraceGap {
            last_step_before: 40,
            first_step_after: Some(97),
        };
        // A hole AND a hole in the worker roster: the roster wins, because resim
        // refuses on it before it walks a record.
        assert_eq!(
            judge_resimmable(&ResimFacts {
                worker_manifest_ranks: &[0, 2],
                trace_gap: Some(gap),
                min_recorded_step: Some(38),
                max_recorded_step: Some(99),
                ..healthy(&nodes, &anchors)
            }),
            Err(ResimGap::RankManifestGap { missing: 1 })
        );
        // A hole AND a refusable record: the hole wins, because the walk's answer
        // is derived from the very stream that has the hole in it.
        assert_eq!(
            judge_resimmable(&ResimFacts {
                trace_gap: Some(gap),
                min_recorded_step: Some(38),
                max_recorded_step: Some(99),
                first_record_refusal: Some(TraceRecordRefusal::Fault(
                    TraceRecordFault::ZeroedRecord
                )),
                ..healthy(&nodes, &anchors)
            }),
            Err(ResimGap::TraceLapped {
                last_step_before: 40,
                first_step_after: Some(97),
            })
        );
    }

    /// A DECLARED trace ring the recorder never read
    /// makes every capture non-resimmable, with the cause named.
    ///
    /// The confident-false shape this closes: `bagd`'s attach path warns and
    /// CONTINUES past a declared ring it cannot open (one vanished ring must
    /// cost its trace, not the frames of every topic in the bag), and the drain
    /// RETIRES a ring lost mid-run — and the verdict then judged only the
    /// records and identities that SURVIVED. A capture whose recorder read one
    /// of a run's two ranks therefore carried a trace that looks whole: every
    /// record well formed, the boundaries in order, the ONE manifest it carries
    /// contiguous. Resim would re-execute a multi-process run's rank 0 alone and
    /// call it a faithful replay.
    ///
    /// Three claims. The ANTI-TAUTOLOGY one is the third: with every declared
    /// ring read, the same facts pass — so the arm is about the SHORTFALL and
    /// not about there being several rings.
    #[test]
    fn a_declared_trace_ring_the_recorder_never_read_refuses_the_capture() {
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let nodes = vec!["a".to_string()];
        let partial = ResimFacts {
            trace_rings_declared: 2,
            trace_rings_unreadable: 1,
            ..healthy(&nodes, &anchors)
        };
        assert_eq!(
            judge_resimmable(&partial),
            Err(ResimGap::TracePartial {
                declared: 2,
                unreadable: 1,
            }),
            "a capture missing a whole ring's records must never be stamped resimmable"
        );
        // The SHORTFALL is what is reported, not the declared count: a recorder
        // that read NONE of three rings says so.
        assert_eq!(
            judge_resimmable(&ResimFacts {
                trace_rings_declared: 3,
                trace_rings_unreadable: 3,
                ..healthy(&nodes, &anchors)
            }),
            Err(ResimGap::TracePartial {
                declared: 3,
                unreadable: 3,
            })
        );
        assert_eq!(
            judge_resimmable(&ResimFacts {
                trace_rings_declared: 2,
                trace_rings_unreadable: 0,
                ..healthy(&nodes, &anchors)
            }),
            Ok(()),
            "ANTI-TAUTOLOGY: two rings BOTH read is not refused here, so the arm above is about \
             the ring nobody read"
        );
    }

    /// The partial-trace gate sits BESIDE the lap
    /// gate — after it, and before every arm computed from the record stream.
    ///
    /// A POSITION, so it is asserted from both sides. A recorder that both lost
    /// a ring and was lapped on another names the LAP, because a hole has edges
    /// an operator can act on; and a capture that is partial AND would trip a
    /// downstream arm (here: a node map several rings leave unresolvable) names
    /// the partial trace, because every downstream answer is computed from the
    /// very stream that is missing a rank.
    #[test]
    fn the_partial_trace_gate_sits_after_the_lap_and_before_the_record_arms() {
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let nodes = vec!["a".to_string()];
        let gap = TraceGap {
            last_step_before: 40,
            first_step_after: Some(97),
        };
        assert_eq!(
            judge_resimmable(&ResimFacts {
                trace_gap: Some(gap),
                min_recorded_step: Some(38),
                max_recorded_step: Some(99),
                trace_rings_declared: 2,
                trace_rings_unreadable: 1,
                ..healthy(&nodes, &anchors)
            }),
            Err(ResimGap::TraceLapped {
                last_step_before: 40,
                first_step_after: Some(97),
            }),
            "a hole has EDGES, so when a recorder managed both it names the sharper refusal"
        );
        assert_eq!(
            judge_resimmable(&ResimFacts {
                trace_rings_declared: 2,
                trace_rings_unreadable: 1,
                required_nodes_known: false,
                ..healthy(&nodes, &anchors)
            }),
            Err(ResimGap::TracePartial {
                declared: 2,
                unreadable: 1,
            }),
            "a stream missing a whole rank makes every arm derived from it unsound rather than \
             merely incomplete, so it is named first"
        );
    }

    /// The partial-trace refusal states the
    /// SHORTFALL, the two causes an operator can check, and what is still in the
    /// bag.
    #[test]
    fn the_partial_trace_refusal_names_the_shortfall_and_where_to_look() {
        let reason = ResimGap::TracePartial {
            declared: 3,
            unreadable: 2,
        }
        .reason();
        assert!(reason.contains("3 scheduler-trace ring(s)"), "{reason}");
        assert!(reason.contains("no reader for 2"), "{reason}");
        // The two ways one arises, so the operator knows what to look for.
        assert!(reason.contains("could not be opened"), "{reason}");
        assert!(reason.contains("lost mid-run"), "{reason}");
        // …and the other half of the rule: the bag is still evidence.
        assert!(
            reason.contains("in the bag and readable"),
            "a refusal must not read as 'the black box is broken': {reason}"
        );
    }

    /// The lap refusal names the hole's EDGES and says later captures are fine.
    ///
    /// Both halves matter to the operator reading it: the numbers are what makes
    /// the gap locatable in the run, and the last sentence is what stops one
    /// refused capture being read as "the black box is broken".
    #[test]
    fn the_lap_refusal_names_the_hole_and_scopes_the_damage() {
        let closed = ResimGap::TraceLapped {
            last_step_before: 40,
            first_step_after: Some(97),
        }
        .reason();
        assert!(closed.contains("step 40"), "{closed}");
        assert!(closed.contains("step 97"), "{closed}");
        assert!(closed.contains("Later captures"), "{closed}");
        // An unclosed hole cannot be refused ON (the arm above), but the sentence
        // still has to be renderable — and must not fabricate a far edge.
        let open = ResimGap::TraceLapped {
            last_step_before: 40,
            first_step_after: None,
        }
        .reason();
        assert!(open.contains("step ?"), "{open}");
        assert!(!open.contains("step 0"), "{open}");
    }

    /// A multi-trace-ring capture cannot enumerate the nodes its trace executes,
    /// and an EMPTY node list must not be read as "no node executed".
    ///
    /// `trim_node_ids` resolves the `node_idx -> node` table from the ONE ring
    /// that supplied it and hands back an empty list for any other count, so
    /// before this gate a multi-rank capture reached the coverage check with
    /// nothing to check and passed it vacuously — publishing `resimmable: true`
    /// against a replay that goes on to execute those very nodes.
    ///
    /// Positioned like the ring-count gate: a property of the RECORDER's
    /// manifests, asked before any anchor question, because every anchor
    /// question is phrased in terms of the node list this capture lacks. So the
    /// arm carries a COMPLETE anchor and a healthy run — the shape that used to
    /// answer `Ok(())`.
    #[test]
    fn an_unknown_node_map_is_refused_rather_than_read_as_an_empty_one() {
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let facts = ResimFacts {
            // A contiguous single-worker roster — the ordinary shape,
            // so the new gate is inert unless an arm says otherwise.
            worker_manifest_ranks: &[0],
            required_nodes_known: false,
            // Nothing else is wrong: without this the arm would pass on some
            // other gap and prove nothing about the node map.
            required_nodes: &[],
            anchor_facts: &anchors,
            ..healthy(&[], &anchors)
        };
        assert_eq!(judge_resimmable(&facts), Err(ResimGap::AmbiguousNodeMap));
        // The anti-tautology half: the SAME facts with a KNOWN (and genuinely
        // empty) map are resimmable, so the refusal is about the map being
        // unknown rather than about it being empty.
        assert_eq!(
            judge_resimmable(&ResimFacts {
                // A contiguous single-worker roster — the ordinary shape,
                // so the new gate is inert unless an arm says otherwise.
                worker_manifest_ranks: &[0],
                required_nodes_known: true,
                ..facts
            }),
            Ok(())
        );
    }

    /// **A record the replay gate refuses is refused here too, and
    /// which gap is named is decided by RECORD ORDER — not by KIND.**
    ///
    /// `run_replay` walks the trace once and refuses per record: the departure
    /// gate, then four record-level faults (`replay_cmd.rs` 628-722). The judge
    /// counted records and derived nodes but asked none of them, so a malformed
    /// or future-written trace was stamped `resimmable: true` and then refused.
    ///
    /// The order matters as much as the arm, and a review corrected which order it
    /// is. The judge used to test an aggregate departure COUNT ahead of the
    /// record walk, so a trace whose fault record came BEFORE its departure
    /// advertised fault replay while the gate refused it earlier under
    /// `TraceRecordUnsupported` — two named gaps for one bag, which is the drift
    /// the shared verdict exists to prevent. Both orders are driven here from the
    /// same pair of records, so an implementation that always answers one kind
    /// fails whichever direction it got wrong.
    #[test]
    fn the_gap_named_is_the_first_refusing_records_in_file_order() {
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let faulted = ResimFacts {
            // A contiguous single-worker roster — the ordinary shape,
            // so the new gate is inert unless an arm says otherwise.
            worker_manifest_ranks: &[0],
            first_record_refusal: fault(7),
            ..healthy(&[], &anchors)
        };
        let gap = judge_resimmable(&faulted).expect_err("a refused record is not resimmable");
        match &gap {
            ResimGap::TraceRecordRejected { detail } => {
                assert!(
                    detail.contains("record_type 7"),
                    "the reason must name the record kind an operator has to act on: {detail}"
                );
            }
            other => panic!("expected TraceRecordRejected, got {other:?}"),
        }
        // The key pin: the fault record comes first and three departures
        // follow it, so the gate refuses at the fault — the count behind it
        // changes nothing.
        assert_eq!(
            judge_resimmable(&ResimFacts {
                // A contiguous single-worker roster — the ordinary shape,
                // so the new gate is inert unless an arm says otherwise.
                worker_manifest_ranks: &[0],
                departure_records: 3,
                ..faulted.clone()
            }),
            Err(ResimGap::TraceRecordRejected {
                detail: crate::trace_ring::TraceRecordFault::UnsupportedRecordType {
                    record_type: 7
                }
                .detail(),
            }),
            "a fault BEFORE a departure is the gap the gate names"
        );
        // …and the other direction, which is what stops this arm from passing a
        // judge that simply never reports a departure: the SAME trace with the
        // departure first is fault replay, carrying the measured count.
        assert_eq!(
            judge_resimmable(&ResimFacts {
                // A contiguous single-worker roster — the ordinary shape,
                // so the new gate is inert unless an arm says otherwise.
                worker_manifest_ranks: &[0],
                departure_records: 3,
                first_record_refusal: Some(TraceRecordRefusal::Departure),
                ..faulted.clone()
            }),
            Err(ResimGap::FaultReplay { departures: 3 })
        );
        // …and the anti-tautology half: the SAME facts with no refusing record
        // are resimmable, so the refusal is about the record and nothing else.
        assert_eq!(
            judge_resimmable(&ResimFacts {
                // A contiguous single-worker roster — the ordinary shape,
                // so the new gate is inert unless an arm says otherwise.
                worker_manifest_ranks: &[0],
                first_record_refusal: None,
                ..faulted
            }),
            Ok(())
        );
    }

    /// The FAIL-CLOSED backstop: facts whose two halves disagree — departures
    /// measured, no walk supplied — are refused rather than resimmable.
    ///
    /// Unreachable when both are measured over one record set, which is the only
    /// way the recorder builds them. It is pinned anyway because `ResimFacts` is
    /// public and the wrong answer here is the confident-false one: a capture
    /// stamped `resimmable: true` against a bag `bag play --resim` refuses at
    /// `DegradedRecordingDeparture`.
    ///
    /// Its position is also asserted, in the same body: it sits BELOW the
    /// record-ordered arm, so a fault-first trace still reports its fault with
    /// departures present. Above the walk it would BE the aggregate-first
    /// precedence a review removed.
    #[test]
    fn departures_with_no_walk_are_refused_rather_than_read_as_resimmable() {
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let inconsistent = ResimFacts {
            // A contiguous single-worker roster — the ordinary shape,
            // so the new gate is inert unless an arm says otherwise.
            worker_manifest_ranks: &[0],
            departure_records: 2,
            first_record_refusal: None,
            ..healthy(&[], &anchors)
        };
        assert_eq!(
            judge_resimmable(&inconsistent),
            Err(ResimGap::FaultReplay { departures: 2 })
        );
        // The backstop is BELOW the walk: with a fault reported first, the fault
        // is still the gap.
        assert!(matches!(
            judge_resimmable(&ResimFacts {
                // A contiguous single-worker roster — the ordinary shape,
                // so the new gate is inert unless an arm says otherwise.
                worker_manifest_ranks: &[0],
                first_record_refusal: fault(9),
                ..inconsistent
            }),
            Err(ResimGap::TraceRecordRejected { .. })
        ));
        // The mirror inconsistency — a departure walked, none counted — must not
        // render as "0 departure (fault) boundary record(s)".
        assert_eq!(
            judge_resimmable(&ResimFacts {
                // A contiguous single-worker roster — the ordinary shape,
                // so the new gate is inert unless an arm says otherwise.
                worker_manifest_ranks: &[0],
                departure_records: 0,
                first_record_refusal: Some(TraceRecordRefusal::Departure),
                ..healthy(&[], &anchors)
            }),
            Err(ResimGap::FaultReplay { departures: 1 })
        );
    }

    /// A capture that is BOTH multi-ring and departure-carrying reports the
    /// DEPARTURE — the refusal `run_replay` reaches first.
    ///
    /// `run_replay` has no node-map gate at all; it scans record types for a
    /// departure boundary right after the graph gate. So when both faults are
    /// present the only answer that matches the gate is `FaultReplay`, and this
    /// arm is the one that fails if the node-map check drifts back above it.
    ///
    /// The pair is asserted BOTH ways round the departure, so it pins an ORDER
    /// rather than a preference: with a departure the verdict is `FaultReplay`
    /// whatever the node map says, and with the departure removed the same facts
    /// fall through to `AmbiguousNodeMap` — which is what stops this arm from
    /// passing against a judge that simply never reports the node map.
    #[test]
    fn a_departure_outranks_an_unknown_node_map() {
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let both = ResimFacts {
            // A contiguous single-worker roster — the ordinary shape,
            // so the new gate is inert unless an arm says otherwise.
            worker_manifest_ranks: &[0],
            required_nodes_known: false,
            departure_records: 2,
            first_record_refusal: Some(TraceRecordRefusal::Departure),
            required_nodes: &[],
            anchor_facts: &anchors,
            ..healthy(&[], &anchors)
        };
        assert_eq!(
            judge_resimmable(&both),
            Err(ResimGap::FaultReplay { departures: 2 })
        );
        assert_eq!(
            judge_resimmable(&ResimFacts {
                // A contiguous single-worker roster — the ordinary shape,
                // so the new gate is inert unless an arm says otherwise.
                worker_manifest_ranks: &[0],
                departure_records: 0,
                first_record_refusal: None,
                ..both
            }),
            Err(ResimGap::AmbiguousNodeMap)
        );
    }

    /// A HOLE in the worker roster is refused, and the
    /// judge reaches it where `run_replay` does.
    ///
    /// The capture side's own rank table (`bagd::rank_node_counts`) ZERO-FILLS a
    /// gap rank, on purpose and correctly for its own use — so on manifests
    /// `{0, 2}` every record resolves, the record walk finds nothing to refuse,
    /// and the judge had nothing to say. `run_replay` meanwhile refuses the bag
    /// at `load_trace_manifests` before it loads a single node. A capture
    /// stamped `resimmable: true` that resim refuses outright is exactly the
    /// class this shared judge exists to prevent.
    ///
    /// The CONTROL is the same facts with a contiguous roster, without which the
    /// arm passes against a judge that refuses every multi-rank capture.
    #[test]
    fn a_hole_in_the_worker_roster_is_refused_naming_the_missing_rank() {
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let nodes = vec!["a".to_string()];
        let gapped = ResimFacts {
            worker_manifest_ranks: &[0, 2],
            first_recorded_step: Some(41),
            required_nodes: &nodes,
            anchor_facts: &anchors,
            ..healthy(&nodes, &anchors)
        };
        assert_eq!(
            judge_resimmable(&gapped),
            Err(ResimGap::RankManifestGap { missing: 1 }),
            "a bag `run_replay` refuses at `MultiRankManifestGap` must never be stamped resimmable"
        );
        assert_eq!(
            judge_resimmable(&ResimFacts {
                worker_manifest_ranks: &[0, 1],
                ..gapped.clone()
            }),
            Ok(()),
            "ANTI-TAUTOLOGY: a contiguous roster is not refused, so the arm above is about the \
             HOLE and not about there being several ranks"
        );
    }

    /// The roster gate sits where `run_replay` reaches
    /// it — AFTER the graph gate and BEFORE the record walk.
    ///
    /// `replay_cmd.rs` runs gate 4 (graph), then gate 5
    /// (`load_trace_manifests`), then the record walk. So a bag that is BOTH
    /// missing its graph and holed reports the graph, and one that is holed AND
    /// carries a malformed record reports the hole. Asserted both ways, so it
    /// pins a POSITION rather than a preference.
    #[test]
    fn the_roster_gate_sits_between_the_graph_gate_and_the_record_walk() {
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let nodes = vec!["a".to_string()];
        let holed = ResimFacts {
            worker_manifest_ranks: &[0, 2],
            first_recorded_step: Some(41),
            required_nodes: &nodes,
            anchor_facts: &anchors,
            ..healthy(&nodes, &anchors)
        };
        assert_eq!(
            judge_resimmable(&ResimFacts {
                graph_attachment_present: false,
                ..holed.clone()
            }),
            Err(ResimGap::NoGraph),
            "the graph gate is reached FIRST"
        );
        assert_eq!(
            judge_resimmable(&ResimFacts {
                first_record_refusal: fault(9),
                ..holed.clone()
            }),
            Err(ResimGap::RankManifestGap { missing: 1 }),
            "the manifests load BEFORE any record is walked, so the hole is the first refusal"
        );
    }

    /// Several state rings outrank an unknown node map.
    ///
    /// `run_replay` has NO node-map gate — it demuxes by rank because it HAS the
    /// rank tables — so the only ordering question is what resim refuses first
    /// on a bag that trips both, and that is `read_bag_anchors`, whose FIRST
    /// statement refuses `rings_declared > 1`.
    ///
    /// A multi-PROCESS capture trips both at once (several trace rings leave the
    /// node map unresolvable AND several state rings are declared), so this is
    /// the ordinary shape rather than a corner: the judge reported
    /// `AmbiguousNodeMap` where the replay refuses `MultiRingAmbiguous`.
    ///
    /// Asserted BOTH ways, so it pins the ORDER: with several rings the verdict
    /// is `MultiRing` whatever the node map says, and with ONE ring the same
    /// facts fall through to `AmbiguousNodeMap` — which stops the arm passing
    /// against a judge that never reports the node map at all.
    #[test]
    fn several_state_rings_outrank_an_unknown_node_map() {
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let both = ResimFacts {
            required_nodes_known: false,
            state_rings_declared: 2,
            first_recorded_step: Some(41),
            required_nodes: &[],
            anchor_facts: &anchors,
            ..healthy(&[], &anchors)
        };
        assert_eq!(
            judge_resimmable(&both),
            Err(ResimGap::MultiRing { rings: 2 }),
            "resim refuses the ring count at `read_bag_anchors` and has no node-map gate at all"
        );
        assert_eq!(
            judge_resimmable(&ResimFacts {
                state_rings_declared: 1,
                ..both
            }),
            Err(ResimGap::AmbiguousNodeMap),
            "with ONE ring the node map is what remains, so this judge does still report it"
        );
    }

    /// Decision: a departure-carrying capture never claims resimmable, and
    /// the reason names fault replay.
    #[test]
    fn a_departure_carrying_capture_is_refused_naming_fault_replay() {
        let nodes = vec!["a".to_string()];
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let mut facts = healthy(&nodes, &anchors);
        facts.departure_records = 2;
        facts.first_record_refusal = Some(TraceRecordRefusal::Departure);
        let gap = judge_resimmable(&facts).expect_err("a fault capture is not resimmable");
        assert_eq!(gap, ResimGap::FaultReplay { departures: 2 });
        let reason = gap.reason();
        assert!(reason.contains("not supported yet"), "{reason}");
        assert!(
            reason.contains("fault"),
            "the reason must name FAULT REPLAY as the gap: {reason}"
        );
    }

    /// A departure is refused even when the anchor is perfect and the trace is
    /// otherwise well formed — it is not a tiebreak, it is a refusal.
    #[test]
    fn a_departure_outranks_a_perfectly_good_anchor() {
        let nodes = vec!["a".to_string()];
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let mut facts = healthy(&nodes, &anchors);
        facts.departure_records = 1;
        facts.first_record_refusal = Some(TraceRecordRefusal::Departure);
        // …and the anchor arm would have said YES, which is what makes this
        // arm's precedence observable rather than incidental.
        let mut control = healthy(&nodes, &anchors);
        control.departure_records = 0;
        control.first_record_refusal = None;
        assert_eq!(judge_resimmable(&control), Ok(()));
        assert!(matches!(
            judge_resimmable(&facts),
            Err(ResimGap::FaultReplay { .. })
        ));
    }

    /// A trace with records but no boundary says exactly that, rather than
    /// borrowing the empty-trace message.
    #[test]
    fn a_trace_with_no_boundary_is_its_own_refusal() {
        let nodes = vec!["a".to_string()];
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let mut facts = healthy(&nodes, &anchors);
        facts.first_recorded_step = None;
        assert_eq!(judge_resimmable(&facts), Err(ResimGap::NoBoundary));
    }

    /// The multi-process shape: several state rings ⇒ `node_idx` is ambiguous.
    #[test]
    fn a_multi_ring_capture_is_refused_naming_the_rank_gap() {
        let nodes = vec!["a".to_string()];
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let mut facts = healthy(&nodes, &anchors);
        facts.state_rings_declared = 3;
        let gap = judge_resimmable(&facts).expect_err("multi-ring is not resimmable");
        assert_eq!(gap, ResimGap::MultiRing { rings: 3 });
        assert!(gap.reason().contains("state rings"), "{}", gap.reason());
    }

    /// EXACTLY one ring is fine — the boundary, so a `>=` cannot pass.
    #[test]
    fn one_ring_is_not_ambiguous() {
        let nodes = vec!["a".to_string()];
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let mut facts = healthy(&nodes, &anchors);
        facts.state_rings_declared = 1;
        assert_eq!(judge_resimmable(&facts), Ok(()));
        // …and zero, which is what a frames-only capture reports, must not be
        // read as "many".
        facts.state_rings_declared = 0;
        assert_eq!(judge_resimmable(&facts), Ok(()));
    }

    #[test]
    fn two_runs_at_the_resume_step_are_ambiguous() {
        let nodes = vec!["a".to_string()];
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let mut facts = healthy(&nodes, &anchors);
        facts.anchor_run_ids_at_anchor_step = 2;
        assert_eq!(
            judge_resimmable(&facts),
            Err(ResimGap::AmbiguousRun { runs: 2 })
        );
    }

    /// The anchor arm is `plan_restore`'s OWN answer, so a node the checkpoint
    /// SKIPPED is refused exactly as resim would refuse it — and the refusal
    /// text is `plan_restore`'s, not a second sentence that could drift from it.
    #[test]
    fn an_anchor_missing_an_executed_node_is_refused_in_plan_restores_own_words() {
        let nodes = vec!["a".to_string(), "b".to_string()];
        // `b` fires in the suffix and has NO fact at the anchor step.
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let gap = judge_resimmable(&healthy(&nodes, &anchors))
            .expect_err("an incomplete anchor is not resimmable");
        let ResimGap::AnchorIncomplete { detail } = &gap else {
            panic!("expected AnchorIncomplete, got {gap:?}");
        };
        assert!(
            detail.contains('b'),
            "the offending node is named: {detail}"
        );
        // The independent oracle: `plan_restore` itself, asked the same question.
        let direct = plan_restore(&RestoreRequest {
            run_id: RUN,
            required_nodes: &nodes,
            first_recorded_step: 41,
            facts: &anchors,
        })
        .expect_err("plan_restore refuses it too");
        assert_eq!(
            detail,
            &direct.to_string(),
            "the capture must report resim's OWN refusal, verbatim"
        );
    }

    /// A capture whose window reaches step 0 needs NO anchor: resim resumes from
    /// the start, so refusing it would refuse a bag resim accepts.
    #[test]
    fn a_capture_that_reaches_step_zero_is_resimmable_with_no_anchor_at_all() {
        let mut facts = healthy(&[], &[]);
        facts.first_recorded_step = Some(0);
        // Deliberately hostile on every anchor-side fact — none of them is
        // consulted on this arm.
        facts.state_rings_declared = 0;
        facts.anchor_run_ids_at_anchor_step = 0;
        assert_eq!(judge_resimmable(&facts), Ok(()));
    }

    /// The `first_recorded_step == 0` arm must not swallow a step-1 capture,
    /// which DOES need an anchor (at step 0).
    #[test]
    fn a_capture_resuming_at_step_one_still_needs_its_step_zero_anchor() {
        let nodes = vec!["a".to_string()];
        let mut facts = healthy(&nodes, &[]);
        facts.first_recorded_step = Some(1);
        assert!(matches!(
            judge_resimmable(&facts),
            Err(ResimGap::AnchorIncomplete { .. })
        ));

        let anchors = vec![fact("a", 0, AnchorOutcome::Complete)];
        let mut ok = healthy(&nodes, &anchors);
        ok.first_recorded_step = Some(1);
        assert_eq!(judge_resimmable(&ok), Ok(()));
    }

    /// Every reason is a SENTENCE an operator can act on, and no two arms render
    /// the same one — a reader who cannot tell two gaps apart cannot fix either.
    #[test]
    fn every_gap_renders_a_distinct_actionable_reason() {
        let gaps = [
            ResimGap::NoTrace,
            // The review that added `NoGraph` did not add it here, so its reason was
            // never held to the distinctness rule the other gaps are.
            ResimGap::NoGraph,
            ResimGap::RankManifestGap { missing: 1 },
            ResimGap::AmbiguousNodeMap,
            ResimGap::TraceRecordRejected {
                detail: crate::trace_ring::TraceRecordFault::ZeroedRecord.detail(),
            },
            ResimGap::FaultReplay { departures: 1 },
            ResimGap::NoBoundary,
            ResimGap::MultiRing { rings: 2 },
            ResimGap::AmbiguousRun { runs: 2 },
            ResimGap::AnchorIncomplete {
                detail: "node `a` has no state".to_string(),
            },
        ];
        let mut seen: Vec<String> = Vec::new();
        for gap in &gaps {
            let reason = gap.reason();
            assert!(reason.len() > 40, "a bare label is not a reason: {reason}");
            assert!(
                !seen.contains(&reason),
                "two gaps render the same reason: {reason}"
            );
            seen.push(reason);
        }
        // …and BOTH positive claims are distinct from every gap, and from each
        // other: a from-start capture and an anchor-resume capture say different
        // things, and neither may borrow a refusal's sentence.
        assert!(!seen.iter().any(|r| r == RESIMMABLE_REASON));
        assert!(!seen.iter().any(|r| r == RESIMMABLE_FROM_START_REASON));
        assert_ne!(RESIMMABLE_REASON, RESIMMABLE_FROM_START_REASON);
        assert!(RESIMMABLE_FROM_START_REASON.len() > 40);
    }
}
