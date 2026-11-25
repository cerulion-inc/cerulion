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
//!
//! Four arms read the capture's coordination stamp. A `recorder.json` that
//! states no contract this build can read ([`ResimGap::CoordinationUnreadable`])
//! is refused right after the record walk, where the replay reads that
//! attachment. Three judge free run only. Several worker ranks whose rank 0
//! resumes mid-run ([`ResimGap::FreeRunResumeUnsupported`]) are asked right
//! below the state-ring arm, where `resolve_resume` refuses them. A worker rank
//! with no boundary stream ([`ResimGap::RankWithoutBoundaryStream`]) is asked
//! first on the from-start arm, where the replay's rank plan refuses it. Worker
//! ranks that begin apart ([`ResimGap::MixedStart`]) are asked right below the
//! boundary arm, ahead of the replay's order: when rank 0 begins at step 0 the
//! from-start arm would otherwise accept the capture, and when rank 0 is the
//! late rank the replay refuses the bag later, at its state-ring or free-run
//! resume gate, while the capture names the ranks. A capture that both begins
//! apart and holds a worker rank with no boundary stream names the mixed start,
//! while the replay refuses that rank first; the remedy is the same. The
//! mixed-start sentence gives the remedy the free-run mid-run refusal gives:
//! record the run from its first step.

use std::collections::BTreeMap;

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
    /// Each WORKER rank's first kept step-boundary step, keyed by rank: one
    /// entry per worker rank that kept a boundary, and none for a rank that
    /// kept none or for any rank that is not a worker (the departure ring's
    /// sentinel rank included). Keyed by rank, so a rank has one first step,
    /// and the resume start and the free-run arms read the same one. A
    /// [`worker_manifest_ranks`](Self::worker_manifest_ranks) rank with no entry
    /// is a worker that kept no boundary.
    ///
    /// The resume's START is the AUTHORITATIVE rank's entry, rank 0's, which the
    /// judge reads out of this map: the replay window is rank 0's boundary
    /// stream by construction, because `resolve_resume` derives its resume point
    /// from `first_recorded_boundary`, which walks rank 0 only. The covered
    /// range's END folds the other way, to the earliest last kept boundary
    /// across the ranks that kept one, what the slowest rank can back. Read
    /// rank-blind, the start would be the first boundary in retained order, and
    /// on a capture whose peer carries no anchor that is the peer's step 0: a
    /// from-start claim the reader refuses.
    ///
    /// Under free run every worker rank is read: ranks that begin apart, one at
    /// step 0 and another past it, are [`ResimGap::MixedStart`], and, when every
    /// worker rank that has an entry begins at step 0, a worker rank with no
    /// entry is [`ResimGap::RankWithoutBoundaryStream`].
    ///
    /// No entry for rank 0 means rank 0 kept no boundary. That is NOT the same
    /// as an empty trace (a trace of FIRE records with no boundary is possible
    /// on a torn head) and resim refuses it separately, so the two are kept
    /// apart here too.
    pub rank_first_steps: &'a BTreeMap<u32, u64>,
    /// The coordination contract the capture's `__cerulion/recorder.json`
    /// states, read through [`recorded_coordination`], the stamp `bag play
    /// --resim` reads.
    ///
    /// A [`RecordedCoordination::Unreadable`] stamp is
    /// [`ResimGap::CoordinationUnreadable`], since the replay refuses such a
    /// recording when it reads its `recorder.json`. Three more arms read it, and
    /// only a free-run stamp takes any of them: [`ResimGap::MixedStart`],
    /// [`ResimGap::RankWithoutBoundaryStream`] and
    /// [`ResimGap::FreeRunResumeUnsupported`].
    pub coordination: RecordedCoordination,
    /// The `trace_format` the capture's `__cerulion/recorder.json` stamps,
    /// read through [`recorded_trace_format`] (`1` for a recording with no
    /// stamp). It words an unknown stop cause in the record walk's refusal
    /// ([`crate::trace_ring::TraceRecordFault::detail`]), as `bag play
    /// --resim` words its own from the same stamp.
    pub trace_format: u32,
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
    /// Kept apart from [`rank_first_steps`](Self::rank_first_steps), whose rank
    /// 0 entry is the AUTHORITATIVE rank's first kept STEP-BOUNDARY step, the
    /// value `resolve_resume` derives a resume point from. These two are over ALL
    /// records including
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
    /// **Ranks that carried a step boundary into the capture and keep none**,
    /// ascending, while another rank keeps one.
    ///
    /// A recorder-only fact, like [`trace_gap`](Self::trace_gap): the capture's
    /// end cut removes a rank's final step when no next boundary proves it
    /// finished, and the trim removes its steps before the anchor, so a rank can
    /// be left with no record in the bag at all while the frames its removed
    /// step published are still there. Nothing in the bag says that rank
    /// existed in the trace, and a covered range folded over the ranks that
    /// kept a boundary holds those frames with no boundary of their rank to
    /// match them against.
    ///
    /// Empty when no rank keeps a boundary: that capture is
    /// [`ResimGap::NoBoundary`].
    pub ranks_without_a_kept_step: &'a [u32],
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
    /// **Ranks that exist and published NO state ring at all**, ascending.
    ///
    /// A RECORDER-ONLY fact, on the same terms as
    /// [`trace_gap`](Self::trace_gap) and
    /// [`trace_rings_unreadable`](Self::trace_rings_unreadable): the replayer
    /// re-measures every other field off the bag, and a rank that published
    /// nothing leaves nothing in the bag to measure. A CAPTURE's own
    /// `state_coverage.json` cannot carry it either, and deliberately does not
    /// (`StateCoverage::for_capture` writes an empty `ranks_missing`, because a
    /// capture walked no rank space and can witness neither density nor a
    /// hole). The recorder DID walk it, so the recorder is the one party that
    /// can state it, and stating it here is what lets a capture's stored
    /// `resimmable_reason` answer the very warn that sent an operator to read
    /// it.
    ///
    /// EMPTY is "no hole was observed", which is the ordinary answer and the
    /// only one a recorder that swept no rank space can give. Read ONLY by the
    /// [`ResimGap::MultiRing`] arm, which is where a rank hole and a multi-ring
    /// recording turn out to be one event: on a run of three or more ranks the
    /// gap publishes nothing and the ranks above it each publish a ring.
    pub ranks_missing: &'a [u32],
    /// Whether anything WALKED a rank space in this run.
    ///
    /// A RECORDER-ONLY fact on the same terms as
    /// [`ranks_missing`](Self::ranks_missing), and the one that says which of
    /// the two things an empty roster means. A discovery sweep walks the rank
    /// space and an empty roster is then its report of no hole; a hand picked
    /// `--state-ring` list walks none, so its empty roster is no report at all.
    /// Only the recorder knows which it did: the bag records the ranks it held,
    /// never whether anything looked past them.
    pub rank_space_walked: bool,
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
    /// The first frame a kept read consumed that this capture does not hold,
    /// when there is one: [`first_frame_gap`] over the kept trace and the
    /// held frames.
    ///
    /// A RECORDER fact, like [`trace_gap`](Self::trace_gap): the trace and the
    /// checkpoint come from rings a recorder can read from their first record,
    /// while the frames are what its taps drained from the moment they
    /// attached. A capture whose resume begins before its frames do, or whose
    /// window lost a frame a kept read consumed, carries a trace that names a
    /// frame `bag play --resim --verify` then cannot find (its
    /// `read_identity_frame_missing`). `None` is "every named frame is held".
    pub frame_gap: Option<FrameGap>,
}

/// One frame a KEPT read record consumed, as the read log names it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NamedFrame {
    /// The consumer's step.
    pub step: u64,
    /// The producer token of the frame's writer: the last producer annotation
    /// of the read's site before it.
    pub writer: u64,
    /// The frame's wire sequence.
    pub sequence: u32,
}

/// The frames a capture holds of ONE writer.
///
/// Private fields: [`first_frame_gap`] binary-searches the sequences, so the
/// only way in is [`HeldFrames::new`], which sorts them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HeldFrames {
    /// The topic the writer's frames are on.
    topic: String,
    /// The wire sequences held, ascending and distinct.
    sequences: Vec<u32>,
}

impl HeldFrames {
    /// The frames of one writer on `topic`, at `sequences` in any order.
    #[must_use]
    pub fn new(topic: String, mut sequences: Vec<u32>) -> Self {
        sequences.sort_unstable();
        sequences.dedup();
        Self { topic, sequences }
    }
}

/// A frame a kept read consumed that the capture does not hold.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FrameGap {
    /// The step of the read that names it.
    pub step: u64,
    /// The writer's producer token.
    pub writer: u64,
    /// The missing frame's wire sequence.
    pub sequence: u32,
    /// Where the capture's frames of that writer begin. `None` when it holds
    /// none of them.
    pub held_from: Option<HeldFrom>,
}

/// The first frame a capture holds of one writer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HeldFrom {
    /// The topic the writer's frames are on.
    pub topic: String,
    /// The lowest sequence held.
    pub first_sequence: u32,
}

/// PURE: the first frame in `named` (record order) that `held` (keyed by
/// writer token) does not hold.
///
/// The replay's `read_identity_frame_missing` predicate, asked at capture time:
/// a fed `--verify` replay looks up every frame a read consumed by its
/// `(writer, sequence)` and refuses before its first step when the bag holds
/// none. Keyed by writer alone because a writer token names one publisher port,
/// and a port publishes on one topic.
pub fn first_frame_gap(
    named: &[NamedFrame],
    held: &std::collections::BTreeMap<u64, HeldFrames>,
) -> Option<FrameGap> {
    named.iter().find_map(|frame| {
        let frames = held.get(&frame.writer);
        if frames.is_some_and(|f| f.sequences.binary_search(&frame.sequence).is_ok()) {
            return None;
        }
        Some(FrameGap {
            step: frame.step,
            writer: frame.writer,
            sequence: frame.sequence,
            held_from: frames.and_then(|f| {
                f.sequences.first().map(|&first_sequence| HeldFrom {
                    topic: f.topic.clone(),
                    first_sequence,
                })
            }),
        })
    })
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

/// The coordination contract a recording was taken under, as the `coordination`
/// key of its `__cerulion/recorder.json` stamps it: `"lockstep"` or
/// `"free_run"`. A recorder writes the stamp through this type's `Serialize`
/// and [`recorded_coordination`] reads it back through its `Deserialize`.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize, Hash,
)]
#[serde(rename_all = "snake_case")]
pub enum CoordinationMode {
    /// Barrier lockstep: every rank advances the SAME handed quantum per step,
    /// so the per-rank STEP_BOUNDARY streams carry EQUAL targets on every
    /// shared step and one first boundary anchors a mid-run resume. The
    /// DEFAULT: a recording with no stamp is replayed under it.
    #[default]
    Lockstep,
    /// Free run: each rank keeps its OWN wall-faithful boundary stream sharing
    /// only the GO epoch, so cross-rank boundary equality does not hold and
    /// there is no single first boundary to anchor off.
    FreeRun,
}

/// The coordination contract a recording's `__cerulion/recorder.json` states,
/// as one of three answers: see [`recorded_coordination`] and
/// [`stated_coordination`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RecordedCoordination {
    /// [`CoordinationMode::Lockstep`].
    Lockstep {
        /// Whether the document stamps `"lockstep"` itself. `false` when there
        /// is no document or it carries no `coordination` key: replay reads
        /// such a recording as lockstep, the contract of a recording with no
        /// stamp.
        stamped: bool,
    },
    /// [`CoordinationMode::FreeRun`], stamped `"free_run"`.
    FreeRun,
    /// A document or stamp this build cannot read. Replay refuses it rather
    /// than reading it as lockstep.
    Unreadable(UnreadableCoordination),
}

/// Why a `recorder.json` states no coordination contract this build can read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum UnreadableCoordination {
    /// The document does not parse as JSON.
    NotJson,
    /// The document parses, and its top level is not a JSON object.
    NotAnObject,
    /// The `coordination` key holds a value [`CoordinationMode`] does not
    /// decode, as JSON text: a string keeps its quotes, so `"barrier"` and
    /// `null` read apart.
    UnknownStamp(String),
}

impl std::fmt::Display for UnreadableCoordination {
    /// The stamp as a sentence can quote it: the value itself for an unknown
    /// stamp, else what the document is.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotJson => f.write_str("not JSON"),
            Self::NotAnObject => f.write_str("not a JSON object"),
            Self::UnknownStamp(value) => f.write_str(value),
        }
    }
}

/// PURE: the `trace_format` `recorder_json`, the bytes of a recording's
/// `__cerulion/recorder.json` (`None` when the recording carries none),
/// stamps: `1` when there is no document, the bytes are not JSON, or the
/// document holds no `trace_format` that is a whole number up to `u32::MAX`,
/// as a recording with no stamp reads. `bag play --resim` reads the stamp it
/// words an unknown stop cause from through this function too.
pub fn recorded_trace_format(recorder_json: Option<&[u8]>) -> u32 {
    recorder_json
        .and_then(|bytes| serde_json::from_slice::<serde_json::Value>(bytes).ok())
        .and_then(|doc| doc.get("trace_format").and_then(serde_json::Value::as_u64))
        .and_then(|f| u32::try_from(f).ok())
        .unwrap_or(1)
}

/// PURE: the coordination contract `recorder_json`, the bytes of a recording's
/// `__cerulion/recorder.json` (`None` when the recording carries none), states.
///
/// * no document: [`RecordedCoordination::Lockstep`] with `stamped: false`;
/// * bytes that are not JSON, or a top level that is not an object:
///   [`RecordedCoordination::Unreadable`], saying which;
/// * a JSON object: [`stated_coordination`] of it.
pub fn recorded_coordination(recorder_json: Option<&[u8]>) -> RecordedCoordination {
    let Some(bytes) = recorder_json else {
        return RecordedCoordination::Lockstep { stamped: false };
    };
    let Ok(doc) = serde_json::from_slice::<serde_json::Value>(bytes) else {
        return RecordedCoordination::Unreadable(UnreadableCoordination::NotJson);
    };
    let Some(obj) = doc.as_object() else {
        return RecordedCoordination::Unreadable(UnreadableCoordination::NotAnObject);
    };
    stated_coordination(obj)
}

/// PURE: the coordination contract a parsed `recorder.json` object states.
///
/// * no `coordination` key: [`RecordedCoordination::Lockstep`] with
///   `stamped: false`;
/// * a value [`CoordinationMode`] decodes (`"lockstep"` or `"free_run"`, or
///   serde's one-key map spelling such as `{"free_run": null}`): that
///   contract, stamped;
/// * any other value (`null`, a number, an unknown string):
///   [`RecordedCoordination::Unreadable`] with
///   [`UnreadableCoordination::UnknownStamp`].
///
/// Reads the `coordination` key and nothing else, so a malformed sibling key
/// (an `arch` that is a number) does not make the contract unreadable.
pub fn stated_coordination(
    recorder: &serde_json::Map<String, serde_json::Value>,
) -> RecordedCoordination {
    let Some(value) = recorder.get("coordination") else {
        return RecordedCoordination::Lockstep { stamped: false };
    };
    match <CoordinationMode as serde::Deserialize>::deserialize(value) {
        Ok(CoordinationMode::Lockstep) => RecordedCoordination::Lockstep { stamped: true },
        Ok(CoordinationMode::FreeRun) => RecordedCoordination::FreeRun,
        Err(_) => RecordedCoordination::Unreadable(UnreadableCoordination::UnknownStamp(
            value.to_string(),
        )),
    }
}

/// The remedy a refusal of where a recording begins gives (the free-run
/// refusals and the frames-begin-late form of [`ResimGap::FrameNotHeld`]): a
/// recording taken from the run's first step needs no resume.
const RECORD_FROM_THE_FIRST_STEP: &str =
    "record the run from its first step with `cerulion graph run --record`";

/// The worker ranks of a free-run recording that begin at step 0 and the ones
/// that begin past it, when it holds both. Only [`mixed_start`] builds one, so
/// neither side is empty and each is ascending by rank.
///
/// Each free-run rank keeps its own boundary stream, so where its recording
/// begins is its own. A rank that begins past step 0 would need its own
/// checkpoint to be re-executed, and a rank that begins at step 0 is
/// re-executed from its constructor state; re-executing the two kinds side by
/// side is not supported, so a capture of such a recording is not resimmable
/// ([`ResimGap::MixedStart`]).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MixedStart {
    /// The ranks that begin past step 0, each with its first step.
    past_zero: Vec<(u32, u64)>,
    /// The ranks that begin at step 0.
    at_zero: Vec<u32>,
}

/// PURE: which of `rank_first_steps` begin at step 0 and which past it, or
/// `None` unless there is at least one of each, which takes two ranks.
///
/// `rank_first_steps` holds each worker rank's first recorded step boundary,
/// keyed by rank. Both sides come out in the map's rank order, so the order
/// the ranks were found in cannot change the sentence.
pub fn mixed_start(rank_first_steps: &BTreeMap<u32, u64>) -> Option<MixedStart> {
    let (at_zero, past_zero): (Vec<_>, Vec<_>) = rank_first_steps
        .iter()
        .map(|(&rank, &step)| (rank, step))
        .partition(|&(_, step)| step == 0);
    if at_zero.is_empty() || past_zero.is_empty() {
        return None;
    }
    Some(MixedStart {
        past_zero,
        at_zero: at_zero.into_iter().map(|(rank, _)| rank).collect(),
    })
}

impl MixedStart {
    /// The sentence a capture of this recording carries as its reason: every
    /// rank on each side, each late rank's first step, and the remedy.
    pub fn sentence(&self) -> String {
        let noun = bag_noun(true);
        let past = match self.past_zero.as_slice() {
            [(rank, step)] => {
                format!("worker rank {rank} of this free-run {noun} begins at step {step}")
            }
            ranks => format!(
                "worker ranks {} of this free-run {noun} begin past step 0",
                and_list(
                    ranks
                        .iter()
                        .map(|(rank, step)| format!("{rank} (first step {step})"))
                )
            ),
        };
        let at = match self.at_zero.as_slice() {
            [rank] => format!("rank {rank} begins at step 0"),
            ranks => format!(
                "ranks {} begin at step 0",
                and_list(ranks.iter().map(u32::to_string))
            ),
        };
        format!(
            "{past} while {at}, and re-executing a rank that begins past step 0 beside one \
             that begins at step 0 is not supported: {RECORD_FROM_THE_FIRST_STEP}"
        )
    }
}

/// The noun a free-run refusal calls the bag it read: [`bag_noun`]'s for a
/// Flashback capture, and "bag" for any other recording, the word the replay's
/// `FreeRunResumeUnsupported` refusal prints.
const fn free_run_noun(from_a_capture: bool) -> &'static str {
    if from_a_capture {
        bag_noun(true)
    } else {
        "bag"
    }
}

/// `a`, `a and b`, `a, b and c`.
fn and_list(items: impl Iterator<Item = String>) -> String {
    let items: Vec<String> = items.collect();
    match items.split_last() {
        Some((last, rest)) if !rest.is_empty() => format!("{} and {last}", rest.join(", ")),
        _ => items.concat(),
    }
}

/// The sentence `bag play --resim` refuses a free-run recording of several
/// worker ranks that begins mid-run with, and the reason a capture of one
/// carries ([`ResimGap::FreeRunResumeUnsupported`]).
///
/// `first_step` is rank 0's first recorded step boundary and `ranks` the
/// worker trace manifests the recording holds. `from_a_capture` picks the
/// noun: "capture" for the reason a Flashback capture carries, "bag" for the
/// replay's refusal, which reads any recording.
pub fn free_run_resume_unsupported_reason(
    first_step: u64,
    ranks: usize,
    from_a_capture: bool,
) -> String {
    let noun = free_run_noun(from_a_capture);
    format!(
        "this {noun} is stamped `coordination: free_run`, its recording begins MID-RUN (its \
         first recorded STEP_BOUNDARY is step {first_step}, not step 0), and it holds \
         {ranks} worker ranks. A mid-run resume anchors on the ONE first recorded boundary \
         and places the clock off that single value, which holds for exactly one worker \
         rank, and a free-run recording with several ranks abolishes it: each rank keeps \
         its own boundary stream and its own clock, so there is no single first boundary, \
         {ranks} clocks cannot be placed off one value, and a frame stamp compared against \
         it crosses clock domains. Per-rank resume anchors are not supported yet; \
         {RECORD_FROM_THE_FIRST_STEP}"
    )
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
    /// plain reason string like its siblings.
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
    /// The capture's `__cerulion/recorder.json` states no coordination
    /// contract this build can read: resim refuses such a bag when it reads
    /// that attachment (`BagInvalidAttachment`, or `CoordinationUnknown` for
    /// an unknown stamp).
    CoordinationUnreadable(UnreadableCoordination),
    /// The trace carries records but no step boundary — resim's
    /// `BagNoStepBoundaries`.
    NoBoundary,
    /// Free-run worker ranks that begin apart: one at step 0 and another past
    /// it. See [`MixedStart`]. Asked ahead of `run_replay`'s order: see the
    /// module docs.
    MixedStart(MixedStart),
    /// Free-run worker ranks that attach a trace manifest and keep no step
    /// boundary: resim's `RankWithoutBoundaryStream`, which its free-run rank
    /// plan raises for the lowest worker rank whose manifest has no boundary
    /// stream; the capture names every such rank. Under lockstep the replay
    /// plans rank 0 alone and does not raise this refusal.
    RankWithoutBoundaryStream {
        /// The worker ranks with no step boundary in the capture, ascending.
        ranks: Vec<u32>,
    },
    /// Some rank carried a step boundary into the capture and keeps none, while
    /// another rank keeps one. See [`ResimFacts::ranks_without_a_kept_step`].
    ///
    /// Under free run such a rank keeps no boundary stream, so a free-run arm
    /// asked earlier refuses the capture first ([`ResimGap::MixedStart`] or
    /// [`ResimGap::RankWithoutBoundaryStream`] from start, [`ResimGap::MultiRing`]
    /// or [`ResimGap::FreeRunResumeUnsupported`] resumed): this gap is reached
    /// only under lockstep. The recorder declines it because
    /// the bag carries no record of the rank's removed steps and no reader can
    /// tell the capture is short of that rank; the recorder is the party that
    /// knows it.
    RankWithoutCompleteStep {
        /// The ranks with no complete step in the capture, ascending.
        ranks: Vec<u32>,
    },
    /// Several state rings, so a record's `node_idx` names a different node in
    /// each — resim's `MultiRingAmbiguous`.
    MultiRing {
        /// How many rings the coverage manifest declares.
        rings: usize,
        /// Ranks that exist and published no state ring at all, ascending.
        ///
        /// The RECORDER's own live sweep, which is the very set the operator's
        /// rank warn is computed from, so this sentence answers that warn by
        /// name.
        ///
        /// It is NOT the roster `replay_state`'s `MultiRingAmbiguous` carries,
        /// and the two are not always equal. That one is read off the BAG's
        /// `state_coverage.json`, and `StateCoverage::for_capture` writes an
        /// EMPTY `ranks_missing` on purpose, because a capture walks no rank
        /// space. So on a capture this stored reason names the hole and a resim
        /// of that same capture cannot: the recorder looked and the bag did
        /// not. Both sentences render through one set of words, which keeps the
        /// WORDING from drifting and was mistaken in an earlier round of this
        /// work for keeping the RANKS equal. What closes the gap for a reader
        /// is [`render_rank_hole`], which makes a capture's refusal say that it
        /// could not look, and name this sentence as the authority.
        ///
        /// EMPTY on a multi-ring recording with no hole, where the clause
        /// renders nothing and the sentence is the one it always was.
        ranks_missing: Vec<u32>,
        /// Whether anything WALKED a rank space in this run.
        ///
        /// The roster above is empty in two unrelated cases, exactly as the
        /// refusal's is: a discovery sweep that ran and found no hole, and a
        /// run whose state rings were handed in by name
        /// (`cerulion bagd --state-ring`), where nothing walked a rank space
        /// and `missing_state_ring_ranks_within` can prove no hole. An earlier
        /// round held that this side needed no flag because its roster had
        /// walked by construction; the hand picked list is an ordinary
        /// invocation and it does not walk, so the stored sentence fell as
        /// silent as the refusal used to.
        rank_space_walked: bool,
    },
    /// A free-run capture of several worker ranks whose rank 0 begins past step
    /// 0: resim's `FreeRunResumeUnsupported`, in its own sentence
    /// ([`free_run_resume_unsupported_reason`]).
    FreeRunResumeUnsupported {
        /// Rank 0's first kept step.
        first_step: u64,
        /// How many worker ranks the capture attaches manifests for.
        ranks: usize,
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
    /// A kept read consumed a frame this capture does not hold: a `--verify`
    /// resim's `read_identity_frame_missing`. See [`ResimFacts::frame_gap`].
    FrameNotHeld {
        /// The step the capture's trace resumes at.
        resume_step: u64,
        /// The first missing frame and where the held ones begin.
        gap: FrameGap,
    },
}

/// The one clause that names the ranks that exist and published no state ring.
///
/// ONE renderer for TWO sentences: the stored `resimmable_reason` a capture
/// carries (through [`ResimGap::MultiRing`]) and the refusal a resim of that
/// capture prints (`cerulion_cli_engine`'s
/// `AnchorReadRefusal::MultiRingAmbiguous`). They are the two halves of one
/// operator journey, so the WORDS are written once, here.
///
/// One renderer keeps the wording identical. It does NOT make the two sentences
/// name the same ranks, and reading it as though it did is what an earlier
/// round of this work got wrong: the two sides build their roster from
/// DIFFERENT sources. The stored reason takes the recorder's live rank sweep.
/// The refusal takes the bag's own `state_coverage.json`, which is written with
/// an empty `ranks_missing` for a capture. They agree wherever both sources
/// walked the rank space, and on a capture only one of them did, which is why a
/// capture's refusal must say that it could not look rather than fall silent:
/// see [`render_rank_hole`].
///
/// EMPTY when there is no hole, so a multi-ring recording without one reads
/// exactly as it did before this clause existed. The ranks are rendered in the
/// order they arrive rather than re-sorted here: the recorder already keeps
/// them ascending, and a second sort would be a second rule about an order that
/// has one owner.
pub fn render_ranks_missing(ranks: &[u32]) -> String {
    if ranks.is_empty() {
        return String::new();
    }
    let list = ranks
        .iter()
        .map(u32::to_string)
        .collect::<Vec<_>>()
        .join(", ");
    let noun = if ranks.len() == 1 { "rank" } else { "ranks" };
    format!(" ({noun} {list} published none, which is the hole you were warned about)")
}

/// The clause a refusal appends when the bag it read is a CAPTURE, whose
/// coverage manifest cannot witness a rank hole at all.
///
/// `StateCoverage::for_capture` writes an EMPTY `ranks_missing` on purpose,
/// because a capture walks no rank space. So an empty roster on a capture is
/// not the claim "no rank published nothing": it is no claim at all, and
/// rendering it as silence reads as the first. That silence is what let one
/// operator journey carry two sentences about one recording, the stored reason
/// naming a rank and the refusal naming none, with neither saying which of them
/// had looked.
///
/// It names the AUTHORITY rather than restating the hole, because the refusal
/// genuinely does not know it: the roster that does is the recorder's own
/// sweep, and the capture stores that as prose in the flashback manifest of
/// this same bag.
///
/// # The authority is named as a FIELD IN A FILE, and deliberately not as a command
///
/// Nothing prints `resimmable_reason`. `cerulion bag info`'s flashback block is
/// built from `FlashbackManifest`, which decodes the capture's seq, pin, spans,
/// frames and topic counts and carries neither the verdict nor its reason, so a
/// clause naming that command would send an operator to a block the sentence is
/// not in: the same class of untrue operator text this clause exists to retire.
/// The words used here are the ones `cerulion flashback`'s own NOT-resimmable
/// line already prints (`flashback_cmd.rs`), so the two pointers cannot come to
/// name different places for one field.
pub const CAPTURE_RANK_ROSTER_CLAUSE: &str =
    " (its coverage manifest walks no rank space, so it names no rank here whether or not this \
     run had a hole: the authority on that is `anchor.resimmable_reason` in this same bag's \
     `__cerulion/flashback.json`)";

/// The clause the STORED reason appends when nothing walked a rank space.
///
/// The RECORDER's roster is empty for two unrelated reasons too, and an earlier
/// round of this work said it was not: it argued that the roster reaching
/// [`ResimGap::MultiRing`] needed no flag because it was the recorder's own
/// sweep and had "walked by construction". A hand picked `--state-ring` list is
/// the counterexample and it is an ordinary invocation: nothing walks a rank
/// space, `missing_state_ring_ranks_within` can prove no hole and answers
/// EMPTY, and the stored sentence then fell exactly as silent as the refusal
/// used to, for the second reason while reading as the first.
pub const UNSWEPT_RANK_ROSTER_CLAUSE: &str =
    " (nothing walked a rank space in this run, whose state rings were named rather than \
     discovered, so it names no rank here whether or not one published none)";

/// Where a rank roster CAME FROM, which is the whole of what a reader of an
/// empty one has to know.
///
/// An empty roster is several unrelated facts wearing one shape, and a renderer
/// handed only the vector renders every one of them as the claim "no rank
/// published nothing". So the source is a PARAMETER: each caller states which
/// reader filled the vector, and the silences that are not the same claim stop
/// reading as one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RankRoster {
    /// The RECORDER's own discovery sweep, which walked a rank space.
    ///
    /// An empty roster here IS the claim that no rank published nothing, and it
    /// is the claim the operator's rank warn was computed from.
    RecorderSweep,
    /// The RECORDER's rings, with nothing having walked a rank space: a hand
    /// picked `--state-ring` list, whose ranks come from the rings' headers and
    /// prove nothing about which ranks the deployment has.
    RecorderNoSweep,
    /// A RECORDING's own `state_coverage.json`.
    ///
    /// Renders as [`RecorderSweep`](Self::RecorderSweep) does, and names the one
    /// case this distinction does NOT reach: a recording whose rings were named
    /// rather than discovered writes its empty `ranks_missing` for the
    /// [`RecorderNoSweep`](Self::RecorderNoSweep) reason, and its manifest
    /// carries no marker saying which. Separating those two needs a
    /// `state_coverage.json` KEY rather than a rendering, so it is stated here
    /// rather than read as though it could not arise.
    RecordingManifest,
    /// A CAPTURE's own `state_coverage.json`, which `StateCoverage::for_capture`
    /// writes empty by design.
    CaptureManifest,
}

/// The rank clause for a reader that knows WHERE its roster came from.
///
/// Four answers, and the two middle ones are the correction this exists for:
///
/// * a roster naming ranks renders the hole clause, whatever the source;
/// * an EMPTY roster from a recorder that SWEPT renders nothing, exactly as it
///   always did: that sweep is the one the operator's warn was computed from,
///   so its silence really is the claim that no rank published nothing;
/// * an EMPTY roster from a recorder that swept NOTHING renders
///   [`UNSWEPT_RANK_ROSTER_CLAUSE`], because a run whose rings were named
///   walked no rank space and can witness neither a hole nor its absence;
/// * an EMPTY roster read off a CAPTURE renders
///   [`CAPTURE_RANK_ROSTER_CLAUSE`], because a capture cannot witness a hole
///   and its silence must not read as "there is none".
///
/// A RECORDING's manifest keeps the old reading, and
/// [`RankRoster::RecordingManifest`] states what that costs.
pub fn render_rank_hole(ranks_missing: &[u32], roster: RankRoster) -> String {
    if !ranks_missing.is_empty() {
        return render_ranks_missing(ranks_missing);
    }
    match roster {
        RankRoster::RecorderSweep | RankRoster::RecordingManifest => String::new(),
        RankRoster::RecorderNoSweep => UNSWEPT_RANK_ROSTER_CLAUSE.to_string(),
        RankRoster::CaptureManifest => CAPTURE_RANK_ROSTER_CLAUSE.to_string(),
    }
}

/// The roster source for a sentence built from the RECORDER's own sets.
///
/// One mapping, named once, so the two readers of a recorder roster cannot come
/// to disagree about what its silence means.
pub const fn recorder_roster(rank_space_walked: bool) -> RankRoster {
    if rank_space_walked {
        RankRoster::RecorderSweep
    } else {
        RankRoster::RecorderNoSweep
    }
}

/// The word a sentence calls the bag it read.
///
/// ONE source for both halves of the multi ring sentence. The refusal opened
/// "this recording" whatever it had read, so a capture's refusal called one bag
/// a recording and a CAPTURE inside a single sentence, while the stored reason
/// about the same bag opened "this capture". A reader who met both learned that
/// the tool did not know what it had read.
pub const fn bag_noun(from_a_capture: bool) -> &'static str {
    if from_a_capture {
        "capture"
    } else {
        "recording"
    }
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
            Self::CoordinationUnreadable(why) => {
                let stated = match why {
                    UnreadableCoordination::UnknownStamp(stamp) => {
                        format!(
                            "stamps `coordination` {stamp}, which is not a contract this Cerulion \
                             knows"
                        )
                    }
                    UnreadableCoordination::NotJson | UnreadableCoordination::NotAnObject => {
                        format!("is {why}, so it states no coordination contract")
                    }
                };
                format!(
                    "this capture's `__cerulion/recorder.json` {stated}, and `cerulion bag play \
                     --resim` refuses a recording whose contract it cannot read before it \
                     re-executes anything. The frames, the state and the trace are in the bag \
                     and readable"
                )
            }
            Self::RankWithoutCompleteStep { ranks } => format!(
                "this capture's trace holds no complete step for rank(s) {ranks:?}, though each \
                 carried a step boundary: the capture removed each such rank's last step because \
                 no next boundary proved it finished, with any step tied to its gating-clock \
                 target, and any earlier step it kept preceded the anchor, if there is one. The \
                 frames those steps published are in the bag, and a resume matches a rank's \
                 frames against that rank's own boundaries, so a replay of this capture can \
                 refuse it as inconsistent. The frames, the state and the other ranks' trace are \
                 in the bag and readable"
            ),
            Self::NoBoundary => "this capture's trace carries no step-boundary record, so nothing \
                                 says which step a resume would begin at. The frames and the \
                                 state are in the bag and readable"
                .to_string(),
            Self::MixedStart(start) => start.sentence(),
            Self::RankWithoutBoundaryStream { ranks } => {
                let (subject, has) = match ranks.as_slice() {
                    [rank] => (format!("worker rank {rank}"), "has a trace manifest"),
                    several => (
                        format!(
                            "worker ranks {}",
                            and_list(several.iter().map(u32::to_string))
                        ),
                        "have trace manifests",
                    ),
                };
                format!(
                    "{subject} of this free-run capture {has} and no step boundary, and \
                     `cerulion bag play --resim` refuses, before re-executing anything, a free-run \
                     recording in which a worker rank has no recorded clock to re-advance: \
                     {RECORD_FROM_THE_FIRST_STEP}"
                )
            }
            Self::FreeRunResumeUnsupported { first_step, ranks } => {
                free_run_resume_unsupported_reason(*first_step, *ranks, true)
            }
            Self::MultiRing {
                rings,
                ranks_missing,
                rank_space_walked,
            } => format!(
                "this capture drained {rings} state rings{}. From state record format version 1 \
                 a state record carries its producer's rank and the coverage manifest carries the \
                 ring to rank join, so a bag of that format does say which ring each record came \
                 from. What is rankless is the READER: its index table is keyed by node \
                 index alone, and its assembler groups parts by run, step and node index, and \
                 neither of those keys carries a rank. Every ring numbers its own nodes from 0, \
                 so two producers' first nodes would collide before any join could be consulted. \
                 Capture a single-process run to get a resimmable bag",
                render_rank_hole(ranks_missing, recorder_roster(*rank_space_walked))
            ),
            Self::AmbiguousRun { runs } => format!(
                "{runs} runs have an anchor at this capture's resume step, and nothing in the bag \
                 says which run a record belongs to"
            ),
            Self::AnchorIncomplete { detail } => format!(
                "this capture's anchor does not cover every node the resumed window executes: \
                 {detail}"
            ),
            Self::FrameNotHeld { resume_step, gap } => frame_not_held_reason(*resume_step, gap),
        }
    }
}

/// The first of the two remedies a capture whose frames begin late carries,
/// which holds for a graph with one worker rank: a capture of several is not
/// resumed from their checkpoints. The second, for any graph, is
/// [`RECORD_FROM_THE_FIRST_STEP`].
const FRAMES_BEGIN_LATE_REMEDY: &str =
    "For a graph with one worker rank, a capture taken more than one Flashback window into the run \
     (`CERULION_FLASHBACK_WINDOW_MS`) resumes from a checkpoint its frames cover";

/// [`ResimGap::FrameNotHeld`]'s sentence: the "frames begin late" form when the
/// missing sequence is below the first one held of its writer (or none is
/// held), the "a frame its window lost" form otherwise.
fn frame_not_held_reason(resume_step: u64, gap: &FrameGap) -> String {
    let FrameGap {
        step,
        writer,
        sequence,
        held_from,
    } = gap;
    let readable = "`cerulion bag play --resim all --verify` cannot judge it. The frames, the \
                    state and the trace are in the bag and readable.";
    let read = if *step == resume_step {
        "whose read".to_string()
    } else {
        format!("and its read at step {step}")
    };
    let attached_after = if resume_step == 0 {
        "the run's first step"
    } else {
        "the checkpoint this capture resumes from"
    };
    let late = |lead: String, frames: &str| {
        format!(
            "{lead}, a frame the capture never held: its frames {frames} start later than \
             {attached_after}, because the recorder's taps attached after it, as in a capture \
             taken in the first seconds of a run, or because the window let them go. {readable} \
             {FRAMES_BEGIN_LATE_REMEDY}. For any graph, {RECORD_FROM_THE_FIRST_STEP}"
        )
    };
    match held_from {
        Some(HeldFrom {
            topic,
            first_sequence,
        }) if sequence < first_sequence => late(
            format!(
                "this capture's frames on '{topic}' from writer {writer:016x} begin at sequence \
                 {first_sequence}, but its trace resumes at step {resume_step}, {read} of that \
                 topic names sequence {sequence}"
            ),
            "on that topic",
        ),
        Some(HeldFrom { topic, .. }) => format!(
            "this capture keeps a read at step {step} of '{topic}' that names writer \
             {writer:016x}, sequence {sequence}, a frame its window lost: the window recorder \
             could not keep up with that topic. {readable} `cerulion graph run --record` records \
             every frame of a run"
        ),
        None => late(
            format!(
                "this capture holds no frame from writer {writer:016x}, but its trace resumes at \
                 step {resume_step}, {read} names that writer's sequence {sequence}"
            ),
            "from that writer",
        ),
    }
}

/// The sentence a RESIMMABLE capture carries.
///
/// Its own constant rather than a literal at the render site, so the positive
/// claim is as reviewable as the negative ones.
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
/// Its own constant so the sentence is as reviewable as the refusals, and
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
/// The number the clause names is the recorder's fold, the MINIMUM over the ranks
/// that kept a boundary of each rank's last kept target, so the sentence names it
/// that way. Calling it the last boundary the trace carries was true only while
/// one rank kept one: on a k>1 capture whose peer ended shorter the trace carries
/// a LATER boundary than the instant printed beside the claim, and a from-start
/// capture reaches this renderer with two ranks (`judge_resimmable` returns on
/// the step-0 arm before the multi-ring gate).
///
/// `covered_through_ns` is `None` when NO rank kept a step boundary to end a
/// range at, which is what the writer's fold over the ranks answers. That is the
/// same condition `ResimGap::NoBoundary` refuses on, and it is asked first, so no
/// capture reaches this arm in production and its only callers are this module's
/// own oracle arms. The base sentence alone is the accurate rendering there: it
/// promises re-execution and states no range it did not measure.
pub fn resimmable_reason(from_start: bool, covered_through_ns: Option<u64>) -> String {
    let base = if from_start {
        RESIMMABLE_FROM_START_REASON
    } else {
        RESIMMABLE_REASON
    };
    match covered_through_ns {
        Some(ns) => format!(
            "{base}{RESIM_COVERED_THROUGH_CLAUSE} {ns} ns (the earliest last kept step boundary \
             across the ranks that kept one). Frames published after that instant are IN the bag \
             and readable: they are outside the range a resume re-executes, because the capture's \
             frame window and its trace window are closed independently"
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
/// missing anchor would be refusing a bag resim would accept. The refusals it
/// still takes there are a free-run worker rank with no boundary stream
/// ([`ResimGap::RankWithoutBoundaryStream`]), a rank left with no complete step
/// ([`ResimGap::RankWithoutCompleteStep`]) and a kept read whose frame the
/// capture does not hold ([`ResimGap::FrameNotHeld`]), none of which an anchor
/// would repair. Under free run the arm accepts a capture only when
/// every worker rank begins at step 0: a peer that begins past it is
/// [`ResimGap::MixedStart`], asked before the arm, and a worker rank that kept
/// no boundary is [`ResimGap::RankWithoutBoundaryStream`], asked first on it.
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
                detail: fault.detail(facts.trace_format),
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
    // 3c. The coordination contract `recorder.json` states. `run_replay` reads
    //     that attachment after its record walk and before it resolves a
    //     resume, and refuses a stamp it cannot read there.
    let free_run = match &facts.coordination {
        RecordedCoordination::FreeRun => true,
        RecordedCoordination::Lockstep { .. } => false,
        RecordedCoordination::Unreadable(why) => {
            return Err(ResimGap::CoordinationUnreadable(why.clone()))
        }
    };
    // 4. Is there a resume point at all? The AUTHORITATIVE rank's own first
    //    kept boundary, the step `resolve_resume` resumes at (see the field).
    let Some(&first_recorded_step) = facts
        .rank_first_steps
        .get(&crate::trace_ring::AUTHORITATIVE_TRACE_RANK)
    else {
        return Err(ResimGap::NoBoundary);
    };
    // 4a. Free-run worker ranks that begin apart, the recording's own shape
    //     rather than a fact about any one step. Asked before the from-start
    //     arm, which a rank past step 0 must not take, and before every anchor
    //     arm, which judges only the checkpoints the capture embeds and so never
    //     sees a rank that begins at step 0.
    if free_run {
        if let Some(start) = mixed_start(facts.rank_first_steps) {
            return Err(ResimGap::MixedStart(start));
        }
    }
    // 4b. A rank with no complete step, the recorder's own observation (see
    //     the field). Asked after the boundary arm, so a capture whose
    //     authoritative rank kept no boundary still names `NoBoundary` as the
    //     replay does. A from-start capture asks it here, where it would
    //     otherwise be accepted; a resumed one asks it last (below), after every
    //     gate the replay's anchor read refuses on first.
    let rank_without_a_step = || -> Result<(), ResimGap> {
        if facts.ranks_without_a_kept_step.is_empty() {
            Ok(())
        } else {
            Err(ResimGap::RankWithoutCompleteStep {
                ranks: facts.ranks_without_a_kept_step.to_vec(),
            })
        }
    };
    // 4c. A frame a kept read consumed and the capture does not hold, asked
    //     LAST on both paths: the replay resolves its resume (every gate it
    //     shares with this judge) and only then plans the fed reads that refuse
    //     a missing frame.
    let frames_held = || -> Result<(), ResimGap> {
        match &facts.frame_gap {
            None => Ok(()),
            Some(gap) => Err(ResimGap::FrameNotHeld {
                resume_step: first_recorded_step,
                gap: gap.clone(),
            }),
        }
    };
    if first_recorded_step == 0 {
        // FromStart — see the doc comment. No anchor is consulted.
        //
        // 4d. Under free run the replay plans every worker manifest's boundary
        //     stream and refuses a rank with none (`RankWithoutBoundaryStream`)
        //     before its consistency checks, so it is asked ahead of 4b.
        if free_run {
            let ranks: Vec<u32> = facts
                .worker_manifest_ranks
                .iter()
                .copied()
                .filter(|rank| !facts.rank_first_steps.contains_key(rank))
                .collect();
            if !ranks.is_empty() {
                return Err(ResimGap::RankWithoutBoundaryStream { ranks });
            }
        }
        rank_without_a_step()?;
        return frames_held();
    }
    // 5. `MultiRingAmbiguous`. A property of the MANIFEST, so it is asked before
    //    anything about a particular step's records — the same ordering
    //    `read_bag_anchors` states for itself.
    if facts.state_rings_declared > 1 {
        return Err(ResimGap::MultiRing {
            rings: facts.state_rings_declared,
            ranks_missing: facts.ranks_missing.to_vec(),
            rank_space_walked: facts.rank_space_walked,
        });
    }
    // 5a. `FreeRunResumeUnsupported`: several free-run worker ranks whose rank
    //     0 resumes mid-run (a peer at step 0 was refused at 4a). `resolve_resume`
    //     refuses it right after its state-ring refusal, before it resolves a run
    //     or plans a restore, in the sentence this gap renders.
    if free_run && facts.worker_manifest_ranks.len() > 1 {
        return Err(ResimGap::FreeRunResumeUnsupported {
            first_step: first_recorded_step,
            ranks: facts.worker_manifest_ranks.len(),
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
    .map_err(|refusal| ResimGap::AnchorIncomplete {
        detail: refusal.to_string(),
    })?;
    // 8. The resumed halves of 4b and 4c.
    rank_without_a_step()?;
    frames_held()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state_restore::AnchorOutcome;
    use crate::trace_ring::{TraceRecordFault, TraceRecordRefusal};

    const RUN: u64 = 0xC1E5;

    /// Rank 0 at step 41, the first steps `healthy` carries.
    static RANK_0_AT_41: std::sync::LazyLock<BTreeMap<u32, u64>> =
        std::sync::LazyLock::new(|| BTreeMap::from([(0, 41)]));
    /// No rank kept a boundary.
    static NO_FIRST_STEP: BTreeMap<u32, u64> = BTreeMap::new();

    /// First steps keyed by rank, from `(rank, step)` pairs.
    fn by_rank(pairs: &[(u32, u64)]) -> BTreeMap<u32, u64> {
        pairs.iter().copied().collect()
    }

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

    /// The writer token of the ticker in the coverage arms below.
    const W: u64 = 0x42cb_ecd9_a96b_9f36;

    fn named(step: u64, writer: u64, sequence: u32) -> NamedFrame {
        NamedFrame {
            step,
            writer,
            sequence,
        }
    }

    fn held(topic: &str, sequences: &[u32]) -> HeldFrames {
        HeldFrames::new(topic.to_string(), sequences.to_vec())
    }

    /// The step-41 read of a capture whose ticker frames begin at sequence 43.
    fn late_gap() -> FrameGap {
        FrameGap {
            step: 41,
            writer: W,
            sequence: 41,
            held_from: Some(HeldFrom {
                topic: "/p/ticker/cmd".to_string(),
                first_sequence: 43,
            }),
        }
    }

    /// Each stamp a recorder writes, `CoordinationMode`'s `Serialize`, reads
    /// back as that contract, stamped.
    #[test]
    fn every_written_coordination_stamp_reads_back_as_its_contract() {
        for (mode, text, want) in [
            (
                CoordinationMode::Lockstep,
                "\"lockstep\"",
                RecordedCoordination::Lockstep { stamped: true },
            ),
            (
                CoordinationMode::FreeRun,
                "\"free_run\"",
                RecordedCoordination::FreeRun,
            ),
        ] {
            // Exhaustive: a new contract stops this test compiling, which
            // points at the rows above.
            match mode {
                CoordinationMode::Lockstep | CoordinationMode::FreeRun => {}
            }
            let stamp = serde_json::to_value(mode).expect("a unit variant serializes");
            assert_eq!(stamp.to_string(), text, "{mode:?}");
            let doc = serde_json::Map::from_iter([("coordination".to_string(), stamp)]);
            assert_eq!(stated_coordination(&doc), want, "{mode:?}");
        }
        assert_eq!(CoordinationMode::default(), CoordinationMode::Lockstep);
    }

    /// Every arm of the coordination parse over hand-written documents, each
    /// answer written out rather than derived.
    #[test]
    fn the_coordination_parse_reads_each_document_shape_one_way() {
        use RecordedCoordination::{FreeRun, Lockstep, Unreadable};
        use UnreadableCoordination::{NotAnObject, NotJson, UnknownStamp};
        let unknown = |text: &str| Unreadable(UnknownStamp(text.to_string()));
        let cases: [(Option<&[u8]>, RecordedCoordination); 19] = [
            // No document, and a document without the key: lockstep, unstamped.
            (None, Lockstep { stamped: false }),
            (Some(b"{}"), Lockstep { stamped: false }),
            (
                Some(br#"{"arch":"aarch64","trace_format":10}"#),
                Lockstep { stamped: false },
            ),
            // The two stamps a recorder writes.
            (
                Some(br#"{"coordination":"lockstep"}"#),
                Lockstep { stamped: true },
            ),
            (Some(br#"{"coordination":"free_run"}"#), FreeRun),
            // A malformed SIBLING costs the contract nothing.
            (Some(br#"{"arch":42,"coordination":"free_run"}"#), FreeRun),
            // serde's one-key map spelling of a unit variant.
            (Some(br#"{"coordination":{"free_run":null}}"#), FreeRun),
            (
                Some(br#"{"coordination":{"lockstep":null}}"#),
                Lockstep { stamped: true },
            ),
            // A duplicated key: the last one is the document's value.
            (
                Some(br#"{"coordination":"lockstep","coordination":"free_run"}"#),
                FreeRun,
            ),
            // Every other value is unreadable, quoted as JSON text.
            (
                Some(br#"{"coordination":"quantum_entangled"}"#),
                unknown("\"quantum_entangled\""),
            ),
            (
                Some(br#"{"coordination":"FreeRun"}"#),
                unknown("\"FreeRun\""),
            ),
            (Some(br#"{"coordination":null}"#), unknown("null")),
            (Some(br#"{"coordination":7}"#), unknown("7")),
            (
                Some(br#"{"coordination":["free_run"]}"#),
                unknown("[\"free_run\"]"),
            ),
            // A document that is not an object, or not JSON at all.
            (Some(br#"["free_run"]"#), Unreadable(NotAnObject)),
            (Some(br#""free_run""#), Unreadable(NotAnObject)),
            (Some(b"null"), Unreadable(NotAnObject)),
            (Some(b""), Unreadable(NotJson)),
            (Some(b"{\"coordination\":\"free_run\""), Unreadable(NotJson)),
        ];
        for (bytes, want) in cases {
            let shown = bytes.map(String::from_utf8_lossy);
            assert_eq!(recorded_coordination(bytes), want, "{shown:?}");
        }
        // The text a refusal quotes.
        assert_eq!(
            UnknownStamp("\"barrier\"".to_string()).to_string(),
            "\"barrier\""
        );
        assert_eq!(NotJson.to_string(), "not JSON");
        assert_eq!(NotAnObject.to_string(), "not a JSON object");
    }

    /// The coverage rule over hand vectors: a held frame is no gap, and the
    /// first unheld one in RECORD order is named with where its writer's held
    /// frames begin.
    #[test]
    fn the_first_named_frame_the_capture_does_not_hold_is_the_gap() {
        const V: u64 = 0x0bad_cafe;
        let map = std::collections::BTreeMap::from([
            // Unsorted, with a repeat: the constructor orders them.
            (W, held("/p/ticker/cmd", &[6, 3, 5, 3])),
            (V, held("/p/odom", &[0, 1])),
        ]);
        let gap = |step, writer, sequence, from: Option<(&str, u32)>| FrameGap {
            step,
            writer,
            sequence,
            held_from: from.map(|(topic, first_sequence)| HeldFrom {
                topic: topic.to_string(),
                first_sequence,
            }),
        };
        assert_eq!(first_frame_gap(&[], &map), None, "no read names nothing");
        assert_eq!(
            first_frame_gap(&[named(3, W, 3), named(5, W, 5), named(1, V, 1)], &map),
            None,
            "every named frame is held"
        );
        assert_eq!(
            first_frame_gap(&[named(1, W, 1), named(2, W, 2)], &map),
            Some(gap(1, W, 1, Some(("/p/ticker/cmd", 3)))),
            "below the first held frame"
        );
        assert_eq!(
            first_frame_gap(&[named(3, W, 3), named(4, W, 4)], &map),
            Some(gap(4, W, 4, Some(("/p/ticker/cmd", 3)))),
            "a hole between held frames"
        );
        assert_eq!(
            first_frame_gap(&[named(7, W, 7)], &map),
            Some(gap(7, W, 7, Some(("/p/ticker/cmd", 3)))),
            "past the last held frame"
        );
        assert_eq!(
            first_frame_gap(&[named(1, V, 0), named(2, 0xFEED, 0)], &map),
            Some(gap(2, 0xFEED, 0, None)),
            "a writer the capture holds nothing of, and V's sequence 0 is held"
        );
        assert_eq!(
            first_frame_gap(&[named(9, W, 9), named(1, W, 1)], &map),
            Some(gap(9, W, 9, Some(("/p/ticker/cmd", 3)))),
            "record order, not the lowest step"
        );
        let empty = std::collections::BTreeMap::from([(W, held("/p/ticker/cmd", &[]))]);
        assert_eq!(
            first_frame_gap(&[named(1, W, 1)], &empty),
            Some(gap(1, W, 1, None)),
            "an entry holding no sequence holds nothing"
        );
    }

    /// The coverage gate is asked LAST on both paths, after every arm the
    /// replay's resume resolves, so each existing refusal keeps its place.
    #[test]
    fn the_coverage_gate_is_asked_after_every_other_arm() {
        let nodes = vec!["a".to_string()];
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let missing = [1u32];
        assert_eq!(
            judge_resimmable(&ResimFacts {
                frame_gap: Some(late_gap()),
                ..healthy(&nodes, &anchors)
            }),
            Err(ResimGap::FrameNotHeld {
                resume_step: 41,
                gap: late_gap(),
            })
        );
        assert!(
            matches!(
                judge_resimmable(&ResimFacts {
                    frame_gap: Some(late_gap()),
                    ..healthy(&nodes, &[])
                }),
                Err(ResimGap::AnchorIncomplete { .. })
            ),
            "an incomplete anchor is named first"
        );
        for first in [0, 41] {
            assert_eq!(
                judge_resimmable(&ResimFacts {
                    rank_first_steps: &by_rank(&[(0, first)]),
                    ranks_without_a_kept_step: &missing,
                    frame_gap: Some(late_gap()),
                    ..healthy(&nodes, &anchors)
                }),
                Err(ResimGap::RankWithoutCompleteStep { ranks: vec![1] }),
                "a rank with no step is named first, step {first}"
            );
        }
        // The from-start path asks it too: no anchor repairs a missing frame.
        assert_eq!(
            judge_resimmable(&ResimFacts {
                rank_first_steps: &by_rank(&[(0, 0)]),
                frame_gap: Some(late_gap()),
                ..healthy(&[], &[])
            }),
            Err(ResimGap::FrameNotHeld {
                resume_step: 0,
                gap: late_gap(),
            })
        );
    }

    /// Each lead names the topic, the step, the writer and both sequences, and
    /// says what to do.
    #[test]
    fn the_frame_not_held_reason_names_the_frame_and_the_remedy() {
        let reason = |resume_step, step, sequence, first: Option<u32>| {
            ResimGap::FrameNotHeld {
                resume_step,
                gap: FrameGap {
                    step,
                    writer: W,
                    sequence,
                    held_from: first.map(|first_sequence| HeldFrom {
                        topic: "/p/ticker/cmd".to_string(),
                        first_sequence,
                    }),
                },
            }
            .reason()
        };
        assert_eq!(
            reason(1, 1, 1, Some(3)),
            "this capture's frames on '/p/ticker/cmd' from writer 42cbecd9a96b9f36 begin at \
             sequence 3, but its trace resumes at step 1, whose read of that topic names sequence \
             1, a frame the capture never held: its frames on that topic start later than the \
             checkpoint this capture resumes from, because the recorder's taps attached after it, \
             as in a capture taken in the first seconds of a run, or because the window let them \
             go. `cerulion bag play --resim all --verify` cannot judge it. The frames, the state \
             and the trace are in the bag and readable. For a graph with one worker rank, a capture \
             taken more than one Flashback window into the run (`CERULION_FLASHBACK_WINDOW_MS`) \
             resumes from a checkpoint its frames cover. For any graph, record the run from its \
             first step with `cerulion graph run --record`"
        );
        assert_eq!(
            reason(1, 4, 4, Some(0)),
            "this capture keeps a read at step 4 of '/p/ticker/cmd' that names writer \
             42cbecd9a96b9f36, sequence 4, a frame its window lost: the window recorder could \
             not keep up with that topic. `cerulion bag play --resim all --verify` cannot judge \
             it. The frames, the state and the trace are in the bag and readable. `cerulion graph \
             run --record` records every frame of a run"
        );
        for (text, clause) in [
            // A first missing read after the resume step names both steps.
            (
                reason(1, 4, 2, Some(3)),
                "resumes at step 1, and its read at step 4 of that topic names sequence 2,",
            ),
            // A from-start capture has no checkpoint to name.
            (
                reason(0, 0, 0, Some(3)),
                "start later than the run's first step,",
            ),
            // A writer the capture holds nothing of.
            (
                reason(1, 1, 0, None),
                "this capture holds no frame from writer 42cbecd9a96b9f36, but its trace resumes \
                 at step 1, whose read names that writer's sequence 0, a frame the capture never \
                 held: its frames from that writer start later than the checkpoint",
            ),
        ] {
            assert!(text.contains(clause), "{clause:?} in {text}");
        }
    }

    /// The capture judge words an unknown stop cause from the capture's own
    /// stamp, as the replay does: at the formats this build reads, which
    /// define no other cause, the trace is corrupt; one format past them, a
    /// newer or foreign writer added the cause. The full reason, each side.
    #[test]
    fn an_unknown_stop_cause_is_worded_from_the_captures_stamp() {
        use crate::trace_ring::SUPPORTED_TRACE_FORMAT;
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let reason = |trace_format: u32| {
            judge_resimmable(&ResimFacts {
                trace_format,
                first_record_refusal: Some(TraceRecordRefusal::Fault(
                    TraceRecordFault::UnknownStopCause { rank: 1, code: 9 },
                )),
                ..healthy(&[], &anchors)
            })
            .expect_err("an unknown stop cause is not resimmable")
            .reason()
        };
        assert_eq!(
            reason(SUPPORTED_TRACE_FORMAT),
            format!(
                "a scheduler-trace NODE-STOP record of rank 1 carries stop cause 9, which trace \
                 format {SUPPORTED_TRACE_FORMAT} does not define; the trace is corrupt. `cerulion \
                 bag play --resim` refuses a trace on that record before it loads anything, so \
                 this capture cannot be re-executed. The frames, the state and the trace are all \
                 in the bag and readable"
            )
        );
        assert_eq!(
            reason(SUPPORTED_TRACE_FORMAT + 1),
            "a scheduler-trace NODE-STOP record of rank 1 carries stop cause 9, which this \
             Cerulion version does not define; the trace was written by a newer or foreign \
             writer, so upgrade cerulion to replay it. `cerulion bag play --resim` refuses a \
             trace on that record before it loads anything, so this capture cannot be \
             re-executed. The frames, the state and the trace are all in the bag and readable"
        );
    }

    /// The stamp a capture's `recorder.json` states, against a hand table:
    /// a whole number up to `u32::MAX` as written, and `1` for every document
    /// that states none.
    #[test]
    fn the_recorded_trace_format_is_the_stamp_or_one() {
        for (doc, want) in [
            (Some(r#"{"trace_format":12}"#), 12u32),
            (Some(r#"{"arch":"aarch64","trace_format":13}"#), 13),
            (Some(r#"{"trace_format":4294967295}"#), u32::MAX),
            (Some(r#"{"trace_format":4294967296}"#), 1),
            (Some(r#"{"trace_format":"12"}"#), 1),
            (Some(r#"{"trace_format":-12}"#), 1),
            (Some(r#"{"arch":"aarch64"}"#), 1),
            (Some("[12]"), 1),
            (Some(r#"{"trace_format":"#), 1),
            (None, 1),
        ] {
            assert_eq!(
                recorded_trace_format(doc.map(str::as_bytes)),
                want,
                "{doc:?}"
            );
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
            rank_first_steps: &RANK_0_AT_41,
            // A recording with no stamp, judged as lockstep, so the two
            // free-run arms are inert unless an arm says otherwise.
            coordination: RecordedCoordination::Lockstep { stamped: false },
            trace_format: 1,
            // One declared ring, and this recorder is reading it — so
            // the partial-trace gate is inert unless an arm says otherwise.
            trace_rings_declared: 1,
            trace_rings_unreadable: 0,
            // Every rank kept a step, so the rank arm is inert unless an arm
            // says otherwise.
            ranks_without_a_kept_step: &[],
            state_rings_declared: 1,
            ranks_missing: &[],
            // A run whose rings were DISCOVERED, which is what every
            // multi-process `graph run` provisions, so the unswept clause is
            // inert unless an arm says otherwise.
            rank_space_walked: true,
            anchor_run_ids_at_anchor_step: 1,
            anchor_run_id: RUN,
            required_nodes: nodes,
            anchor_facts: anchors,
            graph_attachment_present: true,
            required_nodes_known: true,
            first_record_refusal: None,
            // A CONTINUOUS trace — no recorder stall — so the lap gate
            // is inert unless an arm says otherwise. The bounds bracket
            // rank 0's first step because a real trace's boundary sits inside
            // the record stream, never outside it.
            trace_gap: None,
            min_recorded_step: Some(41),
            max_recorded_step: Some(45),
            // Every frame the kept reads name is held, so the coverage gate is
            // inert unless an arm says otherwise.
            frame_gap: None,
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
            // only for a capture whose trace carries no step boundary on ANY
            // rank, which `NoBoundary` refuses, so the correct rendering is
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
            rank_first_steps: &NO_FIRST_STEP,
            // Free run, which the two free-run arms read, hostile as well.
            coordination: RecordedCoordination::FreeRun,
            trace_format: 1,
            // A ring nobody read, too — so this also pins that the
            // partial-trace gate does not preempt the no-trace one.
            trace_rings_declared: 2,
            trace_rings_unreadable: 1,
            // A rank left without a step, hostile as well.
            ranks_without_a_kept_step: &[1],
            state_rings_declared: 4,
            ranks_missing: &[],
            // Hostile here too: a run that swept no rank space is the shape
            // whose empty roster says nothing, and the ring arm this would
            // change the wording of is never reached on this fixture.
            rank_space_walked: false,
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
            // A missing frame too, hostile like the rest: the coverage gate is
            // asked last.
            frame_gap: Some(late_gap()),
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
            rank_first_steps: &NO_FIRST_STEP,
            coordination: RecordedCoordination::FreeRun,
            trace_format: 1,
            // A rank left without a step, hostile as well.
            ranks_without_a_kept_step: &[1],
            state_rings_declared: 4,
            ranks_missing: &[],
            // Hostile here too: a run that swept no rank space is the shape
            // whose empty roster says nothing, and the ring arm this would
            // change the wording of is never reached on this fixture.
            rank_space_walked: false,
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
            // A missing frame too, hostile like the rest: the coverage gate is
            // asked last.
            frame_gap: Some(late_gap()),
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
                rank_first_steps: &by_rank(&[(0, 97)]),
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
            first_record_refusal: fault(8),
            ..healthy(&[], &anchors)
        };
        let gap = judge_resimmable(&faulted).expect_err("a refused record is not resimmable");
        match &gap {
            ResimGap::TraceRecordRejected { detail } => {
                assert!(
                    detail.contains("record_type 8"),
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
                    record_type: 8
                }
                .detail(1),
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
            rank_first_steps: &by_rank(&[(0, 41)]),
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
            rank_first_steps: &by_rank(&[(0, 41)]),
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
            ranks_missing: &[],
            rank_first_steps: &by_rank(&[(0, 41)]),
            required_nodes: &[],
            anchor_facts: &anchors,
            ..healthy(&[], &anchors)
        };
        assert_eq!(
            judge_resimmable(&both),
            Err(ResimGap::MultiRing {
                rings: 2,
                ranks_missing: Vec::new(),
                rank_space_walked: true,
            }),
            "resim refuses the ring count at `read_bag_anchors` and has no node-map gate at all"
        );
        assert_eq!(
            judge_resimmable(&ResimFacts {
                state_rings_declared: 1,
                ranks_missing: &[],
                ..both
            }),
            Err(ResimGap::AmbiguousNodeMap),
            "with ONE ring the node map is what remains, so this judge does still report it"
        );
    }

    /// The STORED reason a multi-ring capture carries names the RANK that
    /// published nothing, in the same words the resim refusal uses.
    ///
    /// `bag info` prints this string and a resim of the same capture prints
    /// `AnchorReadRefusal::MultiRingAmbiguous`, and until now only the second
    /// named the rank. An operator warned about a RANK, told by `bag info` about
    /// RINGS, and then told by the resim about a rank again has to work out that
    /// all three are one event. Both sentences render the clause through
    /// [`render_ranks_missing`], so the WORDS are one set of words.
    ///
    /// They are not one ROSTER, and the doc this replaces claimed they were.
    /// The roster here is a RECORDER fact and reaches this judge no other way:
    /// a capture's own `state_coverage.json` carries an empty `ranks_missing`
    /// by design, because a capture walks no rank space. That is exactly the
    /// manifest the replay side reads, so on a capture this sentence names the
    /// hole and the refusal cannot. What the refusal does instead is say that
    /// it did not look and name this sentence as the authority, which is
    /// [`render_rank_hole`] and the arm below it.
    #[test]
    fn the_stored_multi_ring_reason_names_the_rank_that_published_no_ring() {
        let nodes = vec!["a".to_string()];
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let reason_for = |missing: &[u32]| {
            judge_resimmable(&ResimFacts {
                state_rings_declared: 2,
                ranks_missing: missing,
                ..healthy(&nodes, &anchors)
            })
            .expect_err("several state rings are never resimmable")
            .reason()
        };

        let one = reason_for(&[1]);
        assert!(
            one.contains("(rank 1 published none, which is the hole you were warned about)"),
            "the stored reason answers the warn by NAME: {one}"
        );
        let two = reason_for(&[1, 3]);
        assert!(
            two.contains("(ranks 1, 3 published none, which is the hole you were warned about)"),
            "two holes read as a list and the noun agrees: {two}"
        );

        // THE CONTROL, and the half that fails if the clause were unconditional:
        // a multi-ring recording with NO hole renders the sentence it always
        // rendered, with no rank clause and no dangling parenthesis.
        let clean = reason_for(&[]);
        assert!(
            !clean.contains("published none"),
            "no hole, no clause: {clean}"
        );
        assert!(
            clean.contains("drained 2 state rings. From state record format version 1 a state"),
            "and the sentence is otherwise the one it always was: {clean}"
        );
        assert!(
            clean.contains("so a bag of that format"),
            "and the clause names a bag, so this capture's sentence never calls \
             the bag it read a recording: {clean}"
        );

        // THE SECOND CONTROL: the clause is reached only through the RING arm.
        // A capture whose window reaches step 0 is resimmable whatever the ring
        // count and whatever the hole, so it carries no reason to name a rank
        // in. This is the same fact the six operator sentences state first.
        assert_eq!(
            judge_resimmable(&ResimFacts {
                state_rings_declared: 2,
                ranks_missing: &[1],
                rank_first_steps: &by_rank(&[(0, 0)]),
                ..healthy(&nodes, &anchors)
            }),
            Ok(()),
            "step 0 needs no anchor, so neither the ring count nor the hole is consulted"
        );
    }

    /// An empty roster on a CAPTURE and an empty roster on a RECORDING must not
    /// render the same, because they are not the same fact.
    ///
    /// This is the arm that holds the correction the doc above states. A
    /// recording's empty roster is its own sweep reporting no hole. A capture's
    /// empty roster is a manifest that walked no rank space and reports
    /// nothing either way, while the stored reason built from the recorder's
    /// sweep may be naming a rank in the very same bag. Rendering both as
    /// silence is what let those two sentences disagree with nothing to say
    /// which of them had looked.
    #[test]
    fn a_captures_empty_rank_roster_does_not_read_as_no_hole() {
        assert_eq!(
            render_rank_hole(&[], RankRoster::RecordingManifest),
            "",
            "a RECORDING with no hole renders nothing, exactly as it always did"
        );
        assert_eq!(
            render_rank_hole(&[1], RankRoster::RecordingManifest),
            render_ranks_missing(&[1]),
            "and one with a hole renders the hole clause, word for word"
        );

        let capture = render_rank_hole(&[], RankRoster::CaptureManifest);
        assert!(
            capture.contains("walks no rank space"),
            "a capture says that it could not look: {capture}"
        );
        assert!(
            capture.contains("resimmable_reason"),
            "and names the sentence that IS the authority for it: {capture}"
        );
        assert_ne!(
            capture,
            render_rank_hole(&[], RankRoster::RecordingManifest),
            "the two readings of an empty roster must not render as one sentence"
        );

        // THE CONTROL on the ORDER of the two questions: a roster that really
        // does name a rank is evidence whichever manifest carried it, so the
        // hole clause wins and the capture note is not appended over it.
        assert_eq!(
            render_rank_hole(&[1], RankRoster::CaptureManifest),
            render_ranks_missing(&[1]),
            "a roster that names a rank is the answer, capture or not"
        );
    }

    /// A RECORDER that swept nothing does not claim there is no hole either.
    ///
    /// The fourth answer, and the one an earlier round argued was unreachable:
    /// it held that the roster reaching [`ResimGap::MultiRing`] was the
    /// recorder's own sweep and had walked "by construction". A hand picked
    /// `--state-ring` list is the counterexample and it is an ordinary
    /// `cerulion bagd` invocation, so the stored sentence carried the same
    /// unwitnessed silence the refusal was corrected for.
    ///
    /// Driven through `reason()` rather than through the renderer alone,
    /// because what shipped was a call site that never asked the question: an
    /// arm over `render_rank_hole` by itself would pass against a `reason()`
    /// that still hardcodes the swept answer.
    #[test]
    fn a_recorder_that_swept_nothing_does_not_claim_no_hole() {
        let unswept = ResimGap::MultiRing {
            rings: 2,
            ranks_missing: Vec::new(),
            rank_space_walked: false,
        }
        .reason();
        assert!(
            unswept.contains("nothing walked a rank space"),
            "a run whose rings were named says so: {unswept}"
        );
        assert!(
            !unswept.contains("published none, which is the hole"),
            "and it invents no hole it never looked for: {unswept}"
        );

        // THE CONTROL, and the arm is vacuous without it: a recorder that DID
        // sweep keeps the sentence it always had, because its silence is a real
        // report and the operator's rank warn was computed from that same walk.
        let swept = ResimGap::MultiRing {
            rings: 2,
            ranks_missing: Vec::new(),
            rank_space_walked: true,
        }
        .reason();
        assert!(
            !swept.contains("nothing walked a rank space"),
            "a sweep that ran says nothing about not having looked: {swept}"
        );
        assert_ne!(unswept, swept, "the two silences are not one sentence");

        // THE SECOND CONTROL, on the ORDER: a roster that names a rank is
        // evidence, so it is rendered whatever the sweep flag says. A run
        // CANNOT reach this shape (a hole is only provable by a walk), and the
        // rule is asserted anyway, because the renderer is what decides which
        // of the two clauses a future caller gets.
        let holed = ResimGap::MultiRing {
            rings: 2,
            ranks_missing: vec![1],
            rank_space_walked: false,
        }
        .reason();
        assert!(
            holed.contains("rank 1 published none"),
            "a named rank is the answer: {holed}"
        );
        assert!(
            !holed.contains("nothing walked a rank space"),
            "and the two clauses are never both appended: {holed}"
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
        facts.rank_first_steps = &NO_FIRST_STEP;
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
        assert_eq!(
            gap,
            ResimGap::MultiRing {
                rings: 3,
                ranks_missing: Vec::new(),
                rank_space_walked: true,
            }
        );
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
        let firsts_facts = by_rank(&[(0, 0)]);
        facts.rank_first_steps = &firsts_facts;
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
        let firsts_facts = by_rank(&[(0, 1)]);
        facts.rank_first_steps = &firsts_facts;
        assert!(matches!(
            judge_resimmable(&facts),
            Err(ResimGap::AnchorIncomplete { .. })
        ));

        let anchors = vec![fact("a", 0, AnchorOutcome::Complete)];
        let mut ok = healthy(&nodes, &anchors);
        let firsts_ok = by_rank(&[(0, 1)]);
        ok.rank_first_steps = &firsts_ok;
        assert_eq!(judge_resimmable(&ok), Ok(()));
    }

    /// The resume starts at RANK 0's first kept step: a peer that keeps its
    /// step 0 does not make a capture whose rank 0 resumes at step 41 a
    /// from-start one, and a peer's step never stands in for a rank 0 that kept
    /// no boundary.
    #[test]
    fn the_resume_starts_at_rank_0s_own_first_step() {
        let nodes = vec!["a".to_string()];
        // No anchor at all: a from-start read accepts this capture, and a resume
        // at step 41 refuses it for the missing anchor.
        for firsts in [by_rank(&[(1, 0), (0, 41)]), by_rank(&[(0, 41), (2, 0)])] {
            assert!(
                matches!(
                    judge_resimmable(&ResimFacts {
                        rank_first_steps: &firsts,
                        ..healthy(&nodes, &[])
                    }),
                    Err(ResimGap::AnchorIncomplete { .. })
                ),
                "{firsts:?}"
            );
        }
        assert_eq!(
            judge_resimmable(&ResimFacts {
                rank_first_steps: &by_rank(&[(2, 0), (0, 0), (1, 0)]),
                ..healthy(&nodes, &[])
            }),
            Ok(()),
            "every rank from step 0 needs no anchor"
        );
        for firsts in [by_rank(&[(1, 0)]), by_rank(&[(1, 41), (2, 0)])] {
            assert_eq!(
                judge_resimmable(&ResimFacts {
                    rank_first_steps: &firsts,
                    ..healthy(&nodes, &[])
                }),
                Err(ResimGap::NoBoundary),
                "{firsts:?}: no rank 0 entry"
            );
        }
    }

    /// The free-run stamp, two worker ranks and the given first steps, every
    /// other fact `healthy`.
    fn free_run<'a>(
        nodes: &'a [String],
        anchors: &'a [AnchorFact],
        rank_first_steps: &'a BTreeMap<u32, u64>,
    ) -> ResimFacts<'a> {
        ResimFacts {
            coordination: RecordedCoordination::FreeRun,
            worker_manifest_ranks: &[0, 1],
            rank_first_steps,
            ..healthy(nodes, anchors)
        }
    }

    /// A start, written out by hand.
    fn start(past_zero: &[(u32, u64)], at_zero: &[u32]) -> MixedStart {
        MixedStart {
            past_zero: past_zero.to_vec(),
            at_zero: at_zero.to_vec(),
        }
    }

    /// The two lockstep stamps: written, and read off a recording with none.
    fn lockstep() -> [RecordedCoordination; 2] {
        [
            RecordedCoordination::Lockstep { stamped: true },
            RecordedCoordination::Lockstep { stamped: false },
        ]
    }

    /// Every way a `recorder.json` can state no contract this build reads.
    fn unreadable() -> [UnreadableCoordination; 3] {
        [
            UnreadableCoordination::NotJson,
            UnreadableCoordination::NotAnObject,
            UnreadableCoordination::UnknownStamp("\"barrier\"".to_string()),
        ]
    }

    /// `mixed_start` answers only when a rank begins at step 0 AND another past
    /// it, each side in rank order whatever order the ranks were found in.
    #[test]
    fn a_mixed_start_needs_a_rank_at_step_0_and_one_past_it() {
        for firsts in [
            by_rank(&[]),
            by_rank(&[(0, 0)]),
            by_rank(&[(0, 6)]),
            by_rank(&[(0, 0), (1, 0), (2, 0)]),
            by_rank(&[(0, 3), (1, 6)]),
        ] {
            assert_eq!(mixed_start(&firsts), None, "{firsts:?}");
        }
        let mixed = |pairs: &[(u32, u64)]| mixed_start(&by_rank(pairs));
        assert_eq!(mixed(&[(0, 0), (1, 1)]), Some(start(&[(1, 1)], &[0])));
        assert_eq!(mixed(&[(0, 5), (1, 0)]), Some(start(&[(0, 5)], &[1])));
        assert_eq!(
            mixed(&[(0, 0), (1, u64::MAX)]),
            Some(start(&[(1, u64::MAX)], &[0]))
        );
        let want = Some(start(&[(1, 6), (2, 9)], &[0, 3]));
        for pairs in [
            [(0, 0), (1, 6), (2, 9), (3, 0)],
            [(3, 0), (2, 9), (1, 6), (0, 0)],
            [(2, 9), (0, 0), (3, 0), (1, 6)],
            [(1, 6), (3, 0), (0, 0), (2, 9)],
        ] {
            assert_eq!(mixed(&pairs), want, "{pairs:?}");
        }
    }

    /// The list joins one item alone, two with `and`, and three or more with
    /// commas and a final `and`.
    #[test]
    fn a_list_reads_as_one_item_or_ends_with_and() {
        let list = |items: &[&str]| and_list(items.iter().map(|s| s.to_string()));
        assert_eq!(list(&[]), "");
        assert_eq!(list(&["0"]), "0");
        assert_eq!(list(&["0", "3"]), "0 and 3");
        assert_eq!(list(&["0", "1", "2"]), "0, 1 and 2");
    }

    /// The sentence names each side in the one-rank and the several-rank form,
    /// and lists three as `a, b and c`.
    #[test]
    fn the_mixed_start_sentence_names_every_rank_on_each_side() {
        let tail = "and re-executing a rank that begins past step 0 beside one that begins at \
                    step 0 is not supported: record the run from its first step with `cerulion \
                    graph run --record`";
        assert_eq!(
            start(&[(1, 2), (2, 4), (3, 8)], &[0]).sentence(),
            format!(
                "worker ranks 1 (first step 2), 2 (first step 4) and 3 (first step 8) of this \
                 free-run capture begin past step 0 while rank 0 begins at step 0, {tail}"
            )
        );
        assert_eq!(
            start(&[(3, 5)], &[0, 1, 2]).sentence(),
            format!(
                "worker rank 3 of this free-run capture begins at step 5 while ranks 0, 1 and 2 \
                 begin at step 0, {tail}"
            )
        );
        assert_eq!(
            start(&[(1, 6)], &[0]).sentence(),
            format!(
                "worker rank 1 of this free-run capture begins at step 6 while rank 0 begins at \
                 step 0, {tail}"
            )
        );
        // Every order of one set of ranks renders one sentence, twice alike.
        let one = mixed_start(&by_rank(&[(0, 0), (1, 6), (2, 9), (3, 0)]))
            .expect("a mixed start")
            .sentence();
        for pairs in [
            [(3, 0), (2, 9), (1, 6), (0, 0)],
            [(2, 9), (0, 0), (3, 0), (1, 6)],
        ] {
            let start = mixed_start(&by_rank(&pairs)).expect("a mixed start");
            assert_eq!(start.sentence(), one, "{pairs:?}");
            assert_eq!(start.sentence(), start.sentence());
        }
    }

    /// The free-run mid-run refusal names the first step and the rank count,
    /// and gives one remedy.
    #[test]
    fn the_free_run_resume_refusal_names_the_step_the_ranks_and_one_remedy() {
        assert_eq!(
            free_run_resume_unsupported_reason(12, 3, false),
            "this bag is stamped `coordination: free_run`, its recording begins MID-RUN (its \
             first recorded STEP_BOUNDARY is step 12, not step 0), and it holds 3 worker ranks. \
             A mid-run resume anchors on the ONE first recorded boundary and places the clock \
             off that single value, which holds for exactly one worker rank, and a free-run \
             recording with several ranks abolishes it: each rank keeps its own boundary stream \
             and its own clock, so there is no single first boundary, 3 clocks cannot be placed \
             off one value, and a frame stamp compared against it crosses clock domains. \
             Per-rank resume anchors are not supported yet; record the run from its first step \
             with `cerulion graph run --record`"
        );
        // The capture's reason is the same sentence about a capture.
        let capture = ResimGap::FreeRunResumeUnsupported {
            first_step: 12,
            ranks: 3,
        }
        .reason();
        assert_eq!(capture, free_run_resume_unsupported_reason(12, 3, true));
        assert_eq!(
            capture,
            free_run_resume_unsupported_reason(12, 3, false).replacen(
                "this bag is stamped",
                "this capture is stamped",
                1
            ),
            "only the noun differs"
        );
        assert!(capture.starts_with("this capture is stamped `coordination: free_run`"));
    }

    /// Under free run, ranks that begin apart are refused by name in both
    /// directions; under every other stamp the judge answers as it does for a
    /// recording with no stamp.
    #[test]
    fn free_run_ranks_that_begin_apart_are_refused_by_name() {
        let nodes = vec!["a".to_string()];
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let later = by_rank(&[(0, 0), (1, 6)]);
        let reverse = by_rank(&[(1, 0), (0, 41)]);
        assert_eq!(
            judge_resimmable(&free_run(&nodes, &anchors, &later)),
            Err(ResimGap::MixedStart(start(&[(1, 6)], &[0])))
        );
        assert_eq!(
            judge_resimmable(&free_run(&nodes, &anchors, &reverse)),
            Err(ResimGap::MixedStart(start(&[(0, 41)], &[1])))
        );
        for coordination in lockstep() {
            for firsts in [&later, &reverse] {
                assert_eq!(
                    judge_resimmable(&ResimFacts {
                        coordination: coordination.clone(),
                        ..free_run(&nodes, &anchors, firsts)
                    }),
                    Ok(()),
                    "{coordination:?} {firsts:?}"
                );
            }
        }
        for why in unreadable() {
            for firsts in [&later, &reverse] {
                assert_eq!(
                    judge_resimmable(&ResimFacts {
                        coordination: RecordedCoordination::Unreadable(why.clone()),
                        ..free_run(&nodes, &anchors, firsts)
                    }),
                    Err(ResimGap::CoordinationUnreadable(why.clone())),
                    "{why:?} {firsts:?}"
                );
            }
        }
        // Ranks that begin together.
        assert_eq!(
            judge_resimmable(&free_run(&nodes, &anchors, &by_rank(&[(0, 0), (1, 0)]))),
            Ok(())
        );
        // Worker rank 1 has a manifest and no boundary stream.
        assert_eq!(
            judge_resimmable(&free_run(&nodes, &anchors, &by_rank(&[(0, 0)]))),
            Err(ResimGap::RankWithoutBoundaryStream { ranks: vec![1] })
        );
        // Three worker ranks: one late beside two at step 0, two late beside
        // rank 0, and rank 0 the late one.
        let three = |pairs: &[(u32, u64)]| {
            judge_resimmable(&ResimFacts {
                worker_manifest_ranks: &[0, 1, 2],
                ..free_run(&nodes, &anchors, &by_rank(pairs))
            })
        };
        assert_eq!(
            three(&[(0, 0), (1, 6), (2, 0)]),
            Err(ResimGap::MixedStart(start(&[(1, 6)], &[0, 2])))
        );
        assert_eq!(
            three(&[(0, 0), (1, 6), (2, 9)]),
            Err(ResimGap::MixedStart(start(&[(1, 6), (2, 9)], &[0])))
        );
        assert_eq!(
            three(&[(0, 41), (1, 0), (2, 0)]),
            Err(ResimGap::MixedStart(start(&[(0, 41)], &[1, 2])))
        );
        // Two judgements of one capture are one verdict.
        let facts = free_run(&nodes, &anchors, &later);
        assert_eq!(judge_resimmable(&facts), judge_resimmable(&facts.clone()));
    }

    /// The arm sits right below the boundary arm: every gate above it names
    /// itself first, and it names itself before every arm below it.
    #[test]
    fn the_mixed_start_arm_sits_below_the_boundary_arm_and_above_every_anchor_arm() {
        let nodes = vec!["a".to_string()];
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let firsts = by_rank(&[(0, 41), (1, 0)]);
        let mixed = free_run(&nodes, &anchors, &firsts);
        let above = [
            (
                ResimFacts {
                    trace_records: 0,
                    ..mixed.clone()
                },
                ResimGap::NoTrace,
            ),
            (
                ResimFacts {
                    graph_attachment_present: false,
                    ..mixed.clone()
                },
                ResimGap::NoGraph,
            ),
            (
                ResimFacts {
                    worker_manifest_ranks: &[0, 2],
                    ..mixed.clone()
                },
                ResimGap::RankManifestGap { missing: 1 },
            ),
            (
                ResimFacts {
                    trace_gap: Some(TraceGap {
                        last_step_before: 41,
                        first_step_after: Some(44),
                    }),
                    ..mixed.clone()
                },
                ResimGap::TraceLapped {
                    last_step_before: 41,
                    first_step_after: Some(44),
                },
            ),
            (
                ResimFacts {
                    trace_rings_unreadable: 1,
                    ..mixed.clone()
                },
                ResimGap::TracePartial {
                    declared: 1,
                    unreadable: 1,
                },
            ),
            (
                ResimFacts {
                    first_record_refusal: Some(TraceRecordRefusal::Departure),
                    departure_records: 1,
                    ..mixed.clone()
                },
                ResimGap::FaultReplay { departures: 1 },
            ),
            (
                ResimFacts {
                    first_record_refusal: fault(9),
                    ..mixed.clone()
                },
                ResimGap::TraceRecordRejected {
                    detail: TraceRecordFault::UnsupportedRecordType { record_type: 9 }.detail(1),
                },
            ),
            // The backstop: departures counted with no walk to place them.
            (
                ResimFacts {
                    departure_records: 2,
                    ..mixed.clone()
                },
                ResimGap::FaultReplay { departures: 2 },
            ),
            (
                ResimFacts {
                    rank_first_steps: &by_rank(&[(1, 0), (2, 41)]),
                    ..mixed.clone()
                },
                ResimGap::NoBoundary,
            ),
        ];
        for (facts, want) in above {
            assert_eq!(judge_resimmable(&facts), Err(want));
        }
        let missing = [1u32];
        let below = [
            ResimFacts {
                state_rings_declared: 2,
                ..mixed.clone()
            },
            ResimFacts {
                required_nodes_known: false,
                ..mixed.clone()
            },
            ResimFacts {
                anchor_run_ids_at_anchor_step: 2,
                ..mixed.clone()
            },
            free_run(&nodes, &[], &firsts),
            ResimFacts {
                ranks_without_a_kept_step: &missing,
                ..mixed.clone()
            },
            ResimFacts {
                frame_gap: Some(late_gap()),
                ..mixed.clone()
            },
        ];
        for facts in below {
            assert_eq!(
                judge_resimmable(&facts),
                Err(ResimGap::MixedStart(start(&[(0, 41)], &[1])))
            );
        }
    }

    /// A free-run capture of several worker ranks whose rank 0 resumes mid-run
    /// reads the replay's own refusal, after the state-ring arm and before the
    /// node map and the anchor arms; one rank, or any other stamp, does not.
    #[test]
    fn a_free_run_capture_of_several_ranks_resuming_mid_run_reads_the_replays_refusal() {
        let nodes = vec!["a".to_string()];
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let refusal = |ranks| {
            Err(ResimGap::FreeRunResumeUnsupported {
                first_step: 41,
                ranks,
            })
        };
        // Both ranks past step 0, and a peer that kept no boundary.
        for firsts in [by_rank(&[(0, 41), (1, 38)]), by_rank(&[(0, 41)])] {
            assert_eq!(
                judge_resimmable(&free_run(&nodes, &anchors, &firsts)),
                refusal(2),
                "{firsts:?}"
            );
        }
        assert_eq!(
            judge_resimmable(&ResimFacts {
                worker_manifest_ranks: &[0, 1, 2],
                ..free_run(&nodes, &anchors, &by_rank(&[(0, 41), (2, 45)]))
            }),
            refusal(3)
        );
        let firsts = by_rank(&[(0, 41), (1, 38)]);
        let both = free_run(&nodes, &anchors, &firsts);
        assert!(matches!(
            judge_resimmable(&ResimFacts {
                state_rings_declared: 2,
                ..both.clone()
            }),
            Err(ResimGap::MultiRing { .. })
        ));
        let missing = [1u32];
        for facts in [
            ResimFacts {
                required_nodes_known: false,
                ..both.clone()
            },
            ResimFacts {
                anchor_run_ids_at_anchor_step: 2,
                ..both.clone()
            },
            free_run(&nodes, &[], &firsts),
            ResimFacts {
                ranks_without_a_kept_step: &missing,
                ..both.clone()
            },
            ResimFacts {
                frame_gap: Some(late_gap()),
                ..both.clone()
            },
        ] {
            assert_eq!(judge_resimmable(&facts), refusal(2));
        }
        // One worker rank takes the ordinary resume.
        assert_eq!(
            judge_resimmable(&ResimFacts {
                worker_manifest_ranks: &[0],
                ..free_run(&nodes, &anchors, &by_rank(&[(0, 41)]))
            }),
            Ok(())
        );
        for coordination in lockstep() {
            assert_eq!(
                judge_resimmable(&ResimFacts {
                    coordination: coordination.clone(),
                    ..both.clone()
                }),
                Ok(()),
                "{coordination:?}"
            );
        }
        for why in unreadable() {
            assert_eq!(
                judge_resimmable(&ResimFacts {
                    coordination: RecordedCoordination::Unreadable(why.clone()),
                    ..both.clone()
                }),
                Err(ResimGap::CoordinationUnreadable(why.clone())),
                "{why:?}"
            );
        }
    }

    /// A `recorder.json` that states no readable contract refuses the capture
    /// in the replay's position: below every record-walk arm, above the
    /// boundary arm and every arm below it, a from-start capture included.
    #[test]
    fn a_coordination_stamp_this_build_cannot_read_refuses_the_capture() {
        let nodes = vec!["a".to_string()];
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let at_zero = by_rank(&[(0, 0)]);
        let mixed = by_rank(&[(0, 0), (1, 6)]);
        for why in unreadable() {
            let refused = |facts: ResimFacts<'_>| {
                judge_resimmable(&ResimFacts {
                    coordination: RecordedCoordination::Unreadable(why.clone()),
                    ..facts
                })
            };
            let gap = Err(ResimGap::CoordinationUnreadable(why.clone()));
            // A from-start capture and a mixed start, each otherwise accepted
            // or refused below this arm.
            assert_eq!(refused(free_run(&nodes, &[], &at_zero)), gap, "{why:?}");
            assert_eq!(refused(free_run(&nodes, &anchors, &mixed)), gap, "{why:?}");
            // Below it: no boundary, several state rings, an unknown node map, an
            // incomplete anchor.
            for facts in [
                free_run(&nodes, &anchors, &NO_FIRST_STEP),
                ResimFacts {
                    state_rings_declared: 2,
                    ..healthy(&nodes, &anchors)
                },
                ResimFacts {
                    required_nodes_known: false,
                    ..healthy(&nodes, &anchors)
                },
                healthy(&nodes, &[]),
            ] {
                assert_eq!(refused(facts), gap, "{why:?}");
            }
            // Above it: the record walk and its backstop.
            assert_eq!(
                refused(ResimFacts {
                    first_record_refusal: Some(TraceRecordRefusal::Departure),
                    departure_records: 1,
                    ..healthy(&nodes, &anchors)
                }),
                Err(ResimGap::FaultReplay { departures: 1 })
            );
            assert_eq!(
                refused(ResimFacts {
                    departure_records: 2,
                    ..healthy(&nodes, &anchors)
                }),
                Err(ResimGap::FaultReplay { departures: 2 })
            );
            assert!(matches!(
                refused(ResimFacts {
                    first_record_refusal: fault(9),
                    ..healthy(&nodes, &anchors)
                }),
                Err(ResimGap::TraceRecordRejected { .. })
            ));
        }
        // The reason states the fact and the replay's refusal, in each form.
        let refusal = "`cerulion bag play --resim` refuses a recording whose contract it cannot \
                       read before it re-executes anything. The frames, the state and the trace \
                       are in the bag and readable";
        for (why, stated) in [
            (
                UnreadableCoordination::NotJson,
                "is not JSON, so it states no coordination contract",
            ),
            (
                UnreadableCoordination::NotAnObject,
                "is not a JSON object, so it states no coordination contract",
            ),
            (
                UnreadableCoordination::UnknownStamp("\"barrier\"".to_string()),
                "stamps `coordination` \"barrier\", which is not a contract this Cerulion knows",
            ),
        ] {
            assert_eq!(
                ResimGap::CoordinationUnreadable(why).reason(),
                format!("this capture's `__cerulion/recorder.json` {stated}, and {refusal}")
            );
        }
    }

    /// Under free run a worker rank with no boundary stream refuses a
    /// from-start capture, below the mixed start and above the rank with no
    /// complete step and the frame check; under lockstep it does not, as the
    /// replay plans rank 0 alone there.
    #[test]
    fn a_free_run_worker_rank_with_no_boundary_stream_refuses_a_from_start_capture() {
        let nodes = vec!["a".to_string()];
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let at_zero = by_rank(&[(0, 0)]);
        let three = |pairs: &[(u32, u64)]| {
            judge_resimmable(&ResimFacts {
                worker_manifest_ranks: &[0, 1, 2],
                ..free_run(&nodes, &anchors, &by_rank(pairs))
            })
        };
        assert_eq!(
            three(&[(0, 0)]),
            Err(ResimGap::RankWithoutBoundaryStream { ranks: vec![1, 2] })
        );
        assert_eq!(
            three(&[(0, 0), (2, 0)]),
            Err(ResimGap::RankWithoutBoundaryStream { ranks: vec![1] })
        );
        assert_eq!(three(&[(0, 0), (1, 0), (2, 0)]), Ok(()));
        // Two judgements of one capture are one verdict.
        assert_eq!(three(&[(0, 0)]), three(&[(0, 0)]));
        // The mixed start is named first; rank 0 past step 0 is the free-run
        // mid-run refusal, as `resolve_resume` refuses before it plans ranks.
        assert_eq!(
            three(&[(0, 0), (1, 6)]),
            Err(ResimGap::MixedStart(start(&[(1, 6)], &[0])))
        );
        assert_eq!(
            three(&[(0, 41)]),
            Err(ResimGap::FreeRunResumeUnsupported {
                first_step: 41,
                ranks: 3,
            })
        );
        // Above the rank with no complete step and the frame check.
        let missing = [1u32];
        for facts in [
            ResimFacts {
                ranks_without_a_kept_step: &missing,
                ..free_run(&nodes, &anchors, &at_zero)
            },
            ResimFacts {
                frame_gap: Some(late_gap()),
                ..free_run(&nodes, &anchors, &at_zero)
            },
        ] {
            assert_eq!(
                judge_resimmable(&facts),
                Err(ResimGap::RankWithoutBoundaryStream { ranks: vec![1] })
            );
        }
        // Below an unreadable stamp.
        assert_eq!(
            judge_resimmable(&ResimFacts {
                coordination: RecordedCoordination::Unreadable(UnreadableCoordination::NotJson),
                ..free_run(&nodes, &anchors, &at_zero)
            }),
            Err(ResimGap::CoordinationUnreadable(
                UnreadableCoordination::NotJson
            ))
        );
        // Lockstep tolerates the rank.
        for coordination in lockstep() {
            assert_eq!(
                judge_resimmable(&ResimFacts {
                    coordination: coordination.clone(),
                    ..free_run(&nodes, &anchors, &at_zero)
                }),
                Ok(()),
                "{coordination:?}"
            );
        }
        // The sentence, one rank and several.
        let tail = "and no step boundary, and `cerulion bag play --resim` refuses, before \
                    re-executing anything, a free-run recording in which a worker rank has no \
                    recorded clock to re-advance: \
                    record the run from its first step with `cerulion graph run --record`";
        assert_eq!(
            ResimGap::RankWithoutBoundaryStream { ranks: vec![1] }.reason(),
            format!("worker rank 1 of this free-run capture has a trace manifest {tail}")
        );
        assert_eq!(
            ResimGap::RankWithoutBoundaryStream {
                ranks: vec![1, 2, 4]
            }
            .reason(),
            format!("worker ranks 1, 2 and 4 of this free-run capture have trace manifests {tail}")
        );
    }

    /// A rank the capture left with no complete step refuses, from start and
    /// resumed alike, once some boundary exists, and after the anchor gates on
    /// a resumed capture.
    #[test]
    fn a_rank_without_a_complete_step_refuses_after_the_boundary_arm() {
        let nodes = vec!["a".to_string()];
        let anchors = vec![fact("a", 40, AnchorOutcome::Complete)];
        let missing = [1u32];
        for first in [0, 41] {
            let facts = ResimFacts {
                rank_first_steps: &by_rank(&[(0, first)]),
                ranks_without_a_kept_step: &missing,
                ..healthy(&nodes, &anchors)
            };
            assert_eq!(
                judge_resimmable(&facts),
                Err(ResimGap::RankWithoutCompleteStep { ranks: vec![1] }),
                "first recorded step {first}"
            );
            assert_eq!(
                judge_resimmable(&ResimFacts {
                    ranks_without_a_kept_step: &[],
                    ..facts.clone()
                }),
                Ok(()),
                "the control: the same facts with every rank kept, step {first}"
            );
        }
        assert_eq!(
            judge_resimmable(&ResimFacts {
                rank_first_steps: &by_rank(&[]),
                ranks_without_a_kept_step: &missing,
                ..healthy(&nodes, &anchors)
            }),
            Err(ResimGap::NoBoundary),
            "with no authoritative boundary the boundary arm answers first"
        );
        // A resumed capture asks it after the anchor gates the replay's anchor
        // read refuses on first.
        assert!(
            matches!(
                judge_resimmable(&ResimFacts {
                    state_rings_declared: 2,
                    ranks_without_a_kept_step: &missing,
                    ..healthy(&nodes, &anchors)
                }),
                Err(ResimGap::MultiRing { .. })
            ),
            "the ring ambiguity is named first on a resumed capture"
        );
        assert!(
            matches!(
                judge_resimmable(&ResimFacts {
                    ranks_without_a_kept_step: &missing,
                    ..healthy(&nodes, &[])
                }),
                Err(ResimGap::AnchorIncomplete { .. })
            ),
            "an incomplete anchor is named first on a resumed capture"
        );
        let reason = ResimGap::RankWithoutCompleteStep { ranks: vec![1] }.reason();
        assert!(
            reason.contains("rank(s) [1]") && reason.contains("no complete step"),
            "{reason}"
        );
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
                detail: crate::trace_ring::TraceRecordFault::ZeroedRecord.detail(1),
            },
            ResimGap::FaultReplay { departures: 1 },
            ResimGap::NoBoundary,
            ResimGap::MixedStart(start(&[(1, 6)], &[0])),
            ResimGap::MixedStart(start(&[(1, 6), (2, 9)], &[0, 3])),
            ResimGap::MixedStart(start(&[(0, 3)], &[1])),
            ResimGap::FreeRunResumeUnsupported {
                first_step: 6,
                ranks: 2,
            },
            ResimGap::RankWithoutCompleteStep { ranks: vec![1] },
            ResimGap::MultiRing {
                rings: 2,
                ranks_missing: Vec::new(),
                rank_space_walked: true,
            },
            // The SAME gap from a recorder that swept nothing, which must not
            // render the sentence above: the distinctness rule this arm keeps
            // is exactly what an unflagged silence broke.
            ResimGap::MultiRing {
                rings: 2,
                ranks_missing: Vec::new(),
                rank_space_walked: false,
            },
            ResimGap::AmbiguousRun { runs: 2 },
            ResimGap::AnchorIncomplete {
                detail: "node `a` has no state".to_string(),
            },
            // The coverage gate's three leads: frames that begin late, a frame
            // the window lost, and a writer the capture holds nothing of.
            ResimGap::FrameNotHeld {
                resume_step: 41,
                gap: late_gap(),
            },
            ResimGap::FrameNotHeld {
                resume_step: 41,
                gap: FrameGap {
                    sequence: 44,
                    ..late_gap()
                },
            },
            ResimGap::FrameNotHeld {
                resume_step: 41,
                gap: FrameGap {
                    held_from: None,
                    ..late_gap()
                },
            },
            ResimGap::RankWithoutBoundaryStream { ranks: vec![1] },
            ResimGap::RankWithoutBoundaryStream { ranks: vec![1, 2] },
            // A `recorder.json` that states no readable contract, each way.
            ResimGap::CoordinationUnreadable(UnreadableCoordination::NotJson),
            ResimGap::CoordinationUnreadable(UnreadableCoordination::NotAnObject),
            ResimGap::CoordinationUnreadable(UnreadableCoordination::UnknownStamp(
                "\"barrier\"".to_string(),
            )),
        ];
        // The mixed start's three forms, written out: one rank on each side,
        // several, and rank 0 the late one.
        let tail = "and re-executing a rank that begins past step 0 beside one that begins at \
                    step 0 is not supported: record the run from its first step with `cerulion \
                    graph run --record`";
        for (gap, lead) in [
            (
                &gaps[7],
                "worker rank 1 of this free-run capture begins at step 6 while rank 0 begins at \
                 step 0,",
            ),
            (
                &gaps[8],
                "worker ranks 1 (first step 6) and 2 (first step 9) of this free-run capture \
                 begin past step 0 while ranks 0 and 3 begin at step 0,",
            ),
            (
                &gaps[9],
                "worker rank 0 of this free-run capture begins at step 3 while rank 1 begins at \
                 step 0,",
            ),
        ] {
            assert_eq!(gap.reason(), format!("{lead} {tail}"));
        }
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
