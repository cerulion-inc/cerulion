// SPDX-License-Identifier: AGPL-3.0-only
//! The READ-LOG-STEERED injection planner.
//!
//! A free-run bag replays SEQUENTIALLY, one rank at a time. While rank `R`
//! runs, every topic produced by a DIFFERENT rank is not produced at all — it is
//! INJECTED from the bag. The rule:
//!
//! > Cross-rank edges are served from the RECORDED frames, injected per edge and
//! > steered by the consumer's own read log (inject exactly the inter-read
//! > frames each record's `popped` implies).
//!
//! This module is the pure half of that sentence. It answers **which recorded
//! frames are due before which step**, and nothing else: no transport, no bag
//! reader, no clock. The engine drives the answer through the real
//! injector and the accounting drain.
//!
//! # Why the read log steers, and not the wall clock
//!
//! The shipped alternative is [`replay_engine`](crate::replay_engine)'s
//! wall-time `InjectionWindow`: re-publish every recorded frame whose wire
//! timestamp falls inside the band the step advances through. That works under
//! LOCKSTEP, where one gating clock orders every rank's publishes against every
//! rank's steps. Under free-run there is no such clock — each rank advances its
//! own, and a producing rank's stamps are not comparable to a consuming rank's
//! step targets (the cross-clock comparison class). What the bag DOES carry, per
//! consuming edge, is the kind-6 read log: for each read, how many frames it
//! popped off its queue and which one it served. Summing `popped` reconstructs
//! the consumer's queue OCCUPANCY without ever comparing two clocks, which is
//! why the read log is THE replay contract: kind 6 carries NO arrival
//! instant — replay pacing is DERIVED from serve constraints, never
//! recorded.
//!
//! # The core rule
//!
//! For one edge, walk its recorded reads in order and keep a running cursor into
//! the topic's recorded frame stream. A read that popped `n` frames consumed the
//! next `n` frames in file order, so those `n` frames must be on the wire before
//! the step that read happened at. **`popped` is the quantity, never
//! `served_seq`** — a `Latest` drain that popped 4 and served the newest names
//! ONE sequence but consumed FOUR frames, and a `sample(N)`-decimated read
//! serves nothing at all while still consuming everything it popped. Steering on
//! the served sequence under-injects both shapes, which is
//! [pinned shape 1](#what-the-tests-pin).
//!
//! # Fan-out, and the one shape that cannot be steered
//!
//! Two consumers of one topic share ONE publish stream but hold INDEPENDENT
//! queues. A single injected frame therefore lands in both queues at once, and
//! the two read logs may account for it at different steps. The rule is
//! earliest-wins — a frame is injected at the earliest step any consumer
//! consumed it, because a frame injected LATE than a consumer's read is simply
//! absent from a read the recording says served it.
//!
//! Injecting EARLY is free for an [`ConsumeMode::EachFifo`] consumer: it pops
//! one frame per read in arrival order, so a deeper queue serves the same frame
//! in the same order. It is NOT free for a [`ConsumeMode::Latest`] one, whose
//! drain consumes the WHOLE queue and serves the newest: hand it two frames
//! where the recording handed it one and it serves the wrong frame. That case is
//! detected, never served — [`plan_topic_injection`] returns a typed
//! [`StandDown`] and the caller falls back to the wall-time window under a NAMED
//! degrade. Silently serving the newer frame would be an invented edge-read
//! divergence attributed to the candidate, which is precisely the false-alarm
//! risk the bag-fed decision exists to avoid.
//!
//! # What this module refuses rather than guesses
//!
//! Every refusal is DATA ([`StandDownReason`]), never a log line — the caller
//! owns the operator surface and must be able to name the degrade in its own
//! verdict header. The refusals:
//!
//! - an OVERFLOW MARKER ([`ReadOutcomeKind::Truncated`]) in the stream: `k`
//!   records were dropped at the stage rim, each having consumed an unknown
//!   number of frames, so the `popped` sum past that point is a floor rather
//!   than a count;
//! - an unresolvable producer on a `multi_publisher_topics` edge, where a wire
//!   `sequence` is per-publisher and names no frame on its own;
//! - a read log that accounts for more frames than the bag recorded, or whose
//!   served sequence disagrees with the frame its own `popped` sum lands on;
//! - a [`ConsumeMode::Latest`] read the earliest-wins schedule cannot reproduce.
//!
//! # A NAMED RESIDUAL: the promote-serve slot state
//!
//! The cursor this module keeps is a sum of `popped`, and that sum is CORRECT
//! today. What the wire cannot express is a SLOT STATE: under per-set Sync a
//! descent's `sync_peek_next_stamp` pops a frame into `next_head` and records it
//! as a `Peek` with `popped: 1`, and the frame is later served by `try_view`'s
//! R-pop′ promote arm — which records NOTHING, exactly as the frozen-slot serve
//! beside it records nothing. So a reader cannot tell "the peeked frame is still
//! parked" from "the body consumed it".
//!
//! **Nothing is wrong because of it.** The pop is accounted exactly once (at the
//! peek), the promote-serve pops nothing, so `fold_edge`'s monotone `popped`
//! sum and `replay_rederive`'s credit sum are both right. `verify_sync` is out
//! of its blast radius entirely: it admits the peek stamp as a
//! CANDIDATE, which over-approximates by construction. The gap blocks only a
//! claim nobody makes yet — anything that would need to know a queue's
//! occupancy at an instant rather than over a span.
//!
//! **Deliberately NO [`StandDownReason`] for it.** A stand-down must rest on
//! OBSERVABLE evidence, and this shape is by construction unobservable — a
//! variant nothing can ever raise is dead vocabulary, which is exactly what
//! `PerSetSyncDescentUnmodelled` was before the format-5 peek/head mark.
//! The inverse [`StandDownReason::ServedNothingPopped`] exists because ITS shape
//! is detectable; this one is not.
//!
//! The ONE record that would close it — a body-site `Served` at `popped: 0` for
//! the promote-serve, mirroring `sync_discard_head`'s `Drain`-at-`popped: 0` —
//! is not written today: it is a recording-side change, so it would need the
//! same PAIRED-rollout treatment the format-5 peek/head mark got.
//!
//! An edge with NO kind-6 coverage at all (a quarantined node, a pre-annotation
//! rank) is a different thing and is not a refusal: it contributes no
//! constraint, and the surviving covered edges steer. It is reported in
//! [`InjectionSchedule::uncovered`] so the caller can say so. Only when NO edge
//! on the topic PLANS A READ is there nothing to steer FROM, and the topic
//! stands down, which is not the report above's test: an edge whose every record
//! is a hand off plans no read and is still not reported, because each of its
//! records names a frame an earlier record on that edge consumed.
//!
//! # What the tests pin
//!
//! Three wrong planners, each failed by its own oracle arm:
//!
//! 1. schedule keyed on `served_seq` alone, ignoring `popped` — the
//!    `DrainedBatch` and decimation arms under-inject;
//! 2. the fan-out feasibility check deleted — the infeasible arm plans a
//!    schedule that silently mis-serves a `Latest` consumer;
//! 3. the multi-publisher stand-down deleted — an unresolvable token joins a
//!    read to whichever frame the file order happened to land on.

use cerulion_core::read_outcome::{
    DueRead, ReadEdgeCapability, ReadOutcomeKind, ReadPlanRefusal, ReadSiteRole, ReadStageRole,
    StageKey,
};
use std::collections::BTreeMap;
use std::fmt;

// ===========================================================================
// Inputs
// ===========================================================================

/// How one input's drain consumes its queue.
///
/// Mirrors `cerulion_core::transport::subscriber::ConsumeMode`, which is
/// `pub(crate)` there and so unreachable from this crate. The two variants must
/// mean exactly what the subscriber's do, because the feasibility rule below is
/// a statement about the real drain: [`Self::Latest`] is
/// `drain_to_latest_with_accounting` (drain the queue, serve the newest),
/// [`Self::EachFifo`] is `drain_one_with_accounting` (pop exactly one, FIFO).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConsumeMode {
    /// Drain the queue, serve the newest sample, discard the rest.
    Latest,
    /// Pop exactly one sample per read, FIFO order — per-message delivery.
    EachFifo,
}

/// What the manifest publisher tables say about ONE 64-bit producer token,
/// already resolved.
///
/// The planner takes the RESOLVED form rather than the raw token deliberately:
/// resolution is `replay_engine`'s `ProducerResolution` — an FNV-1a over each
/// rank manifest's recorded `UniquePublisherId`s, merged across ranks — and a
/// second copy of that hashing would be a second thing to keep in step with the
/// recorder. This type is the OUTCOME of that lookup, so the planner can neither
/// re-derive it nor disagree with the verifier about it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ResolvedProducer {
    /// Exactly one recorded publisher hashes to the token: `node/output`, the
    /// stable identity across runs.
    Named(String),
    /// Two or more DISTINCT recorded ids hash to the token. Never guess which.
    Collision,
    /// No rank manifest names the token — a bag recorded without them, a manifest that
    /// degraded past its budget, or a cross-graph writer.
    Foreign,
}

/// WHICH of an input's two queues an edge addresses.
///
/// ONE `(node, input)` can carry TWO INDEPENDENT QUEUES: under the Sync and
/// Separate disciplines the runtime owns a trigger/sync DRAIN subscriber beside
/// the node body's own, and `cerulion_core::read_outcome::ReadStageRole`'s doc
/// says they "deliberately share an `input_idx`". They have their own cursors
/// into the same published stream, so the planner must be told which one a
/// stream of reads belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum DrainSite {
    /// The runtime-owned trigger/sync DRAIN subscriber.
    Drain,
    /// The node BODY's own read.
    Body,
}

/// A consuming edge's address: the node, the input port it reads, and WHICH of
/// that input's two queues.
///
/// Named as a triple rather than a joined string because it is what the
/// "edge-read divergence" phrase renders (the phrase names
/// the EDGE, which is the whole reason the class was promoted out of a generic
/// exit 6), and a caller composing that sentence needs the parts.
///
/// # The SITE is a field, not a suffix on the name
///
/// Building the body edge's id as
/// `format!("{input}#body")` would put a name no port has into the `--report`
/// JSON's `injection_stand_downs[].edge` and every rendered sentence, where a
/// reader (or a CI job keying on the input) cannot tell a real port called
/// `x#body` from the body queue of `x`. The suffix is a RENDERING, so it lives
/// in [`fmt::Display`] and the data carries the two halves apart.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct EdgeId {
    /// The consuming node's id, as resolved through its rank's manifest.
    pub node_id: String,
    /// The input port name — the port's OWN name, never decorated.
    pub input: String,
    /// Which of the input's two queues.
    pub site: DrainSite,
}

impl EdgeId {
    /// A DRAIN-site edge (the runtime's trigger/sync subscriber).
    #[must_use]
    pub fn drain(node_id: impl Into<String>, input: impl Into<String>) -> Self {
        Self {
            node_id: node_id.into(),
            input: input.into(),
            site: DrainSite::Drain,
        }
    }

    /// A BODY-site edge (the node's own read).
    #[must_use]
    pub fn body(node_id: impl Into<String>, input: impl Into<String>) -> Self {
        Self {
            node_id: node_id.into(),
            input: input.into(),
            site: DrainSite::Body,
        }
    }
}

impl fmt::Display for EdgeId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // The `#body` suffix an operator reads — rendered HERE, so the data
        // never carries a port name no port has.
        match self.site {
            DrainSite::Drain => write!(f, "{}.{}", self.node_id, self.input),
            DrainSite::Body => write!(f, "{}.{}#body", self.node_id, self.input),
        }
    }
}

/// One recorded kind-6 record, in RECORD ORDER, as demuxed from the bag.
///
/// The two ANNOTATION kinds ([`ReadOutcomeKind::Producer`] and
/// [`ReadOutcomeKind::Truncated`]) are passed through as records rather than
/// pre-folded by the caller: folding an annotation onto the read it annotates is
/// part of reading the log correctly, so it lives here, next to the tests that
/// pin it, rather than being duplicated at every call site.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecordedRead {
    /// The step this record was staged at, on the RECORDING rank's own boundary
    /// stream. Non-decreasing within one edge's stream.
    ///
    /// OUTSIDE the body because every record has one, whatever it says.
    pub step: u64,
    /// What this record IS — see [`RecordedReadBody`].
    pub body: RecordedReadBody,
}

/// The five READ kinds — [`ReadOutcomeKind`] minus its two ANNOTATION kinds,
/// which are [`RecordedReadBody`] variants in their own right.
///
/// A narrower type exists so [`RecordedReadBody::Read`] cannot hold a kind that
/// is not a read: the planner's serving test (`Served | DrainedBatch`) would
/// otherwise be one arm away from a `Producer` record whose fields mean
/// something else entirely.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadKind {
    /// A fresh frame was served.
    Served,
    /// An earlier frame was re-served (the cross-step hold); nothing was popped.
    Held,
    /// Nothing was available.
    NoFrame,
    /// A trigger drain; `served_seq` names the newest frame it took.
    DrainedBatch,
    /// A `sample(N)` gate dropped the frame; `served_seq` names the last
    /// ACCEPTED sequence.
    Decimated,
}

impl ReadKind {
    /// The READ kind a wire kind names, or `None` for the two ANNOTATION kinds
    /// — which are [`RecordedReadBody`] variants and never a `Read`.
    ///
    /// Exhaustive over [`ReadOutcomeKind`] on purpose: a kind added upstream
    /// fails to compile HERE, at the one classifier, instead of falling into
    /// whichever arm a caller's wildcard happened to be.
    #[must_use]
    pub fn from_outcome(kind: ReadOutcomeKind) -> Option<Self> {
        match kind {
            ReadOutcomeKind::Served => Some(Self::Served),
            ReadOutcomeKind::Held => Some(Self::Held),
            ReadOutcomeKind::NoFrame => Some(Self::NoFrame),
            ReadOutcomeKind::DrainedBatch => Some(Self::DrainedBatch),
            ReadOutcomeKind::Decimated => Some(Self::Decimated),
            ReadOutcomeKind::Truncated | ReadOutcomeKind::Producer => None,
        }
    }

    /// Does this read NAME a frame it just consumed?
    ///
    /// `Held` names an EARLIER frame (it popped nothing), `Decimated` names the
    /// last ACCEPTED sequence (which the gate dropped this frame in favour of),
    /// and `NoFrame` names nothing — none of the three can be joined to a
    /// position in this read's own range.
    #[must_use]
    fn serves(self) -> bool {
        matches!(self, Self::Served | Self::DrainedBatch)
    }
}

/// What one recorded kind-6 record IS.
///
/// An enum rather than six flat `pub` fields, three of which would be only conditionally
/// meaningful — `popped` reinterpreted on a `Truncated` marker, `token` only on
/// a `Producer` annotation, and a `hand_off` flag valid ONLY in conjunction
/// with `popped: 0` and a present `served_seq`, which a consumer could do no
/// more than `debug_assert!`. A debug assertion in a REPLAY planner is the
/// wrong instrument twice over: it is absent from the release build a robot
/// runs, and it fires at the consumer rather than refusing the value at
/// construction. The variants carry exactly the fields their kind gives
/// meaning, so every one of those combinations is unrepresentable.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RecordedReadBody {
    /// An ordinary read: it consumed `popped` frames off the queue and served
    /// what `served_seq` names (`None` = the no-frame sentinel).
    Read {
        /// Which read this was.
        kind: ReadKind,
        /// The served frame's wire sequence; `None` is the no-frame sentinel.
        served_seq: Option<u32>,
        /// Frames this read consumed off the queue.
        popped: u32,
    },
    /// A HAND-OFF: it names a frame an EARLIER record on this edge already
    /// consumed, and consumes nothing itself.
    ///
    /// The one producer is the per-set Sync matcher's PROMOTION: a peek pops a
    /// frame off the queue (recording it, with its real `popped`) and parks it
    /// as `next_head`; the promotion then installs that same frame as the head
    /// and records it AGAIN, at the same sequence, with `popped: 0`. The second
    /// record exists so `verify_sync` can tell which frame was the head at fire
    /// time; it moves nothing between the queue and the node.
    ///
    /// The planner must therefore SKIP it: it occupies no position in the frame
    /// stream, so joining it would either land on a frame the cursor has
    /// already passed or refuse the edge outright
    /// ([`StandDownReason::ServedNothingPopped`], whose doc's "not a shape any
    /// production drain site emits" was true until this record existed).
    ///
    /// It carries NO `popped` (a hand-off consumes nothing by definition) and
    /// its `served_seq` is not optional (a promotion NAMES the frame it
    /// installed) — the two `debug_assert!`s this variant replaced. A zero-pop
    /// record that names NOTHING is not a hand-off but the corrupt shape
    /// `ServedNothingPopped` exists for, so [`RecordedRead::read`] refuses to
    /// build one and it is still stood down under every role.
    HandOff {
        /// The wire sequence of the frame this record installed as the head.
        served_seq: u32,
    },
    /// A producer ANNOTATION: it names the publisher of the frame the NEXT read
    /// record on this edge serves. It consumes nothing and occupies no position
    /// in the frame stream.
    Producer {
        /// The resolved token. NOT optional: a `Producer` record whose token
        /// resolved to nothing does not exist — an unrecognised one resolves to
        /// [`ResolvedProducer::Foreign`], which is an answer.
        token: ResolvedProducer,
    },
    /// An OVERFLOW marker: `dropped_records` records are missing here, each
    /// having consumed an unknown number of frames.
    Truncated {
        /// How many records the marker's own merge window dropped. This is the
        /// field the flat model spelled `popped`, which meant something else on
        /// every other kind.
        dropped_records: u32,
    },
}

impl RecordedRead {
    /// An ordinary read, or the HAND-OFF it is when `hand_off` says so AND the
    /// record has a hand-off's SHAPE.
    ///
    /// `hand_off` is DERIVED by the caller, not re-derived here, and only where
    /// the bag's `trace_format` says the roles may be believed — the
    /// module-wide rule that a site claim is read once, at the seam that knows
    /// the format. On an archived bag it is always `false`, so a genuinely
    /// corrupt zero-pop `DrainedBatch` there is refused exactly as it was.
    ///
    /// The SHAPE half is enforced here rather than asked of the caller: a
    /// hand-off consumes nothing, names the frame it installed, AND is a
    /// `DrainedBatch` — the only kind a promotion wears (`sync_discard_head`
    /// stages one; nothing else stages a zero-pop batch that names a frame). A
    /// `hand_off` claim over a record that popped frames, names none, or is
    /// any other kind takes the ordinary read arm — where a zero-pop serving
    /// read meets [`StandDownReason::ServedNothingPopped`], as it must.
    ///
    /// The kind conjunct is HERE, with the other two shape conjuncts, and not
    /// at the call site: a shape rule in this
    /// constructor that drops the kind lets a corrupt zero-pop
    /// `Served` under a `Drain` role classify as a hand-off and be SKIPPED
    /// by `fold_edge` — the exact corruption `ServedNothingPopped` exists to
    /// refuse — instead of standing the topic down.
    #[must_use]
    pub fn read(
        step: u64,
        kind: ReadKind,
        served_seq: Option<u32>,
        popped: u32,
        hand_off: bool,
    ) -> Self {
        let body = match served_seq {
            Some(seq) if hand_off && popped == 0 && kind == ReadKind::DrainedBatch => {
                RecordedReadBody::HandOff { served_seq: seq }
            }
            _ => RecordedReadBody::Read {
                kind,
                served_seq,
                popped,
            },
        };
        Self { step, body }
    }

    /// A producer annotation naming `token`.
    #[must_use]
    pub fn producer(step: u64, token: ResolvedProducer) -> Self {
        Self {
            step,
            body: RecordedReadBody::Producer { token },
        }
    }

    /// An overflow marker for a window that dropped `dropped_records` records.
    #[must_use]
    pub fn truncated(step: u64, dropped_records: u32) -> Self {
        Self {
            step,
            body: RecordedReadBody::Truncated { dropped_records },
        }
    }
}

/// One consuming edge on the replaying rank, with its whole recorded read
/// stream.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConsumerEdge {
    /// Which consumer this is.
    pub id: EdgeId,
    /// How its drain consumes the queue — the feasibility rule keys on this.
    pub mode: ConsumeMode,
    /// The input's provisioned queue depth, when the caller knows it.
    ///
    /// Optional because the planner's core rule does not need it, but supplying
    /// it closes a real hole in earliest-wins: injecting a frame ahead of the
    /// step that consumed it DEEPENS the queue, and a queue driven past its
    /// depth evicts under `drop_oldest` — an eviction the recording never had.
    /// With a depth in hand that is a detected [`StandDownReason::QueueOverflow`];
    /// without one it is an unmodelled residual.
    pub depth: Option<u32>,
    /// The recorded kind-6 stream, in record order, annotations included.
    pub reads: Vec<RecordedRead>,
}

/// One recorded frame of the topic, in the bag's FILE order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecordedFrame {
    /// The frame's wire sequence — the join key the read log names.
    pub sequence: u32,
    /// Which publisher produced it, where the caller can attribute it.
    ///
    /// `None` is the ordinary shape on a single-publisher topic, where the
    /// sequence identifies the frame on its own.
    ///
    /// On a [`TopicReplay::multi_publisher`] topic it is the RECORD-TIME label
    /// the recorder wrote down (its `__cerulion/frame_producers`
    /// sidecar), resolved through the same manifest publisher tables the read
    /// annotation resolves through — so the two sides of the join are stated in
    /// ONE vocabulary and the planner can neither re-derive nor disagree with
    /// the verifier about either.
    ///
    /// `None` on such a topic therefore does not mean "the bag cannot carry
    /// this" — it means THIS bag does not: it predates the sidecar, or its label
    /// stream did not cover the frame. Either way the planner stands the topic
    /// down ([`ProducerGap::UnattributedFrame`]) rather than guessing a join;
    /// the engine that read the sidecar is what says WHICH of the two it was.
    pub producer: Option<ResolvedProducer>,
}

/// One topic's whole planning input: its recorded frames plus every edge on the
/// replaying rank that consumes it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TopicReplay {
    /// The canonical topic name (reporting only).
    pub topic: String,
    /// Whether the topic is `multi_publisher_topics`-listed — the ONLY case
    /// where a wire `sequence` does not identify a frame (it is a per-publisher
    /// counter, so two writers' sequences collide).
    pub multi_publisher: bool,
    /// The topic's recorded frames, in FILE order.
    ///
    /// EVERY recorded frame, not a subset: the schedule addresses frames by
    /// their INDEX in this vector and the caller drives the plan by POPPING that
    /// many off a feed serving the same file order, so a gap here silently
    /// re-keys every later index onto a different frame.
    ///
    /// A HEADERLESS recorded frame therefore cannot appear: [`RecordedFrame`]
    /// carries a `sequence`, and there is none to carry. That is a CALLER
    /// obligation, not something this module can check — the engine refuses such
    /// a topic BEFORE planning it (the `headerless_recorded_frames` injection
    /// stand-down) and drives it from the recorded-clock window instead, which
    /// injects a headerless frame verbatim and in order. Passing one here by
    /// omitting it would produce a plan that looks well-formed and addresses the
    /// wrong frames from that index on.
    pub frames: Vec<RecordedFrame>,
    /// The consuming edges on the rank being replayed.
    pub edges: Vec<ConsumerEdge>,
}

// ===========================================================================
// Outputs
// ===========================================================================

/// One frame the schedule places.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DueFrame {
    /// The frame's 0-based index in the topic's recorded FILE order.
    ///
    /// This, not [`Self::sequence`], is the unambiguous identity: a wire
    /// sequence is per-publisher, so on a multi-publisher topic two distinct
    /// frames can carry the same one. It is also what the caller wants — the
    /// injector walks the frame feed positionally.
    pub frame_index: usize,
    /// The frame's wire sequence, carried so the caller can cross-check the
    /// frame it pulled off the feed without re-parsing the plan.
    ///
    /// That cross-check is ARMED: `Injector::inject_planned`
    /// parses the header of every frame it is about to inject and compares it
    /// against this, counting and REPORTING a disagreement
    /// (`InjectionAnomalyKind::SequenceMismatch`) rather than injecting silently.
    /// Without that cross-check the
    /// positional model and the frame feed could disagree about a topic's stream
    /// and every later injection land on the wrong frame with no trace of it.
    pub sequence: u32,
}

/// Foreign frames the recording placed BETWEEN two fires of a
/// LOCAL producer, and the pause that puts them back there.
///
/// # Why a before-step bucket cannot express this
///
/// Everything a [`StepInjection::frames`] bucket holds lands in the consumer
/// FIFO ahead of EVERY one of that step's local publishes, so a recorded serve
/// order of `[local fire 1, FOREIGN, local fire 2]` on one shared topic is
/// unreproducible by construction — the replayed consumer would see the foreign
/// frame first. This is the sub-step vocabulary that expresses it, and it is the
/// exact mirror of `cerulion_core`'s `IntraStepPause`: the scheduler pauses a
/// node's trace-driven burst after its `after_fire`-th fire and hands control to
/// the engine's injection hook, which owes it these frames.
///
/// # Reachable only on a LABELLED topic
///
/// A slot exists only where the planner can tell a LOCAL frame from a foreign
/// one, which needs [`RecordedFrame::producer`] — and that is populated only on
/// a [`TopicReplay::multi_publisher`] topic (a single-publisher topic's frames
/// carry `None` by construction, and a topic two ranks both write to is
/// multi-publisher by definition). So a bag with no producer labels plans no
/// slots at all, and the schedule it gets is the slot-free one.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SlotInjection {
    /// The LOCAL producer node whose burst pauses — the node an
    /// `IntraStepPause` names.
    pub producer_node: String,
    /// How many of that node's fires must have COMPLETED first. 1-BASED, and
    /// counted WITHIN this step, matching the scheduler seam exactly; `0` is
    /// never emitted (a frame before the step's first local fire folds into
    /// [`StepInjection::frames`], which is what "before any fire" already
    /// means).
    pub after_fire: u32,
    /// The frames, in FILE order.
    pub frames: Vec<DueFrame>,
}

/// The frames due before ONE step, and the frames due INSIDE it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StepInjection {
    /// Inject these BEFORE executing this step, so the step's drain sees them.
    pub before_step: u64,
    /// The frames, in FILE order.
    pub frames: Vec<DueFrame>,
    /// The INTRA-STEP slots, in the order their anchoring local
    /// fires occur — so a caller driving them in order reproduces the recorded
    /// interleave.
    ///
    /// At most ONE entry per `(producer_node, after_fire)` pair: consecutive
    /// foreign frames under one anchor FOLD into that anchor's slot, because
    /// two pauses at one slot is what `Scheduler::set_replay_intra_step_pauses`
    /// refuses ("fold two injections at ONE slot into ONE pause carrying both
    /// frames").
    ///
    /// EMPTY on every topic with no local producer, which is every topic the
    /// shipped rank split injects — see [`SlotInjection`].
    pub slots: Vec<SlotInjection>,
    /// Per LOCAL producer node, how many of ITS frames this step's
    /// recorded stream carries — the ordinal ceiling every slot's `after_fire`
    /// was derived from.
    ///
    /// Exported so the engine can hold it against the recorded FIRE count of
    /// the same `(node, step)`: `after_fire` counts local FRAMES while the
    /// scheduler's pause counts FIRES, and the two diverge on a fire that
    /// published nothing (a replayed discard, a lazy loan the
    /// tick never wrote) or more than one frame on this topic (a node writing
    /// the listed topic through two outputs). See
    /// `replay_engine::AfterFireDisagreement`.
    pub local_fire_ordinals: BTreeMap<String, u32>,
}

/// A steerable topic's plan.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InjectionSchedule {
    /// The topic this plans.
    pub topic: String,
    /// Frames due, keyed by step, ascending. A step with nothing due is OMITTED
    /// rather than carried as an empty entry.
    pub per_step: Vec<StepInjection>,
    /// Edges with NO kind-6 coverage: they constrained nothing, and the caller
    /// must say so rather than implying the schedule was verified against them.
    ///
    /// An edge whose every record is a hand off is NOT here: each of its records
    /// names a frame an earlier record on that edge consumed, so the schedule
    /// leaves no read of it unexplained.
    pub uncovered: Vec<EdgeId>,
    /// Recorded frames NO covered edge accounts for — trailing frames published
    /// after the last recorded read, most often.
    ///
    /// They are deliberately NOT scheduled: the read log is the only evidence of
    /// what the rank's queues held, and injecting a frame it never accounts for
    /// would add occupancy the recording did not have, breaking the very
    /// [`ConsumeMode::Latest`] reproduction the plan exists to guarantee.
    pub unconsumed: Vec<DueFrame>,
    /// Recorded frames a LOCAL producer publishes, which this
    /// schedule therefore does NOT inject.
    ///
    /// Kept apart from [`Self::unconsumed`] rather than folded into it, because
    /// the two are opposite claims: an unconsumed frame is one the read log
    /// never accounts for (a coverage HOLE the caller reports), while a local
    /// frame is one the replaying rank produces itself (the schedule declining
    /// it is the whole point). Folding them would put a phantom coverage hole
    /// on every co-located topic, sized to its own live output.
    ///
    /// EMPTY whenever [`LocalProducers`] is, which is every topic the shipped
    /// rank split injects.
    pub local_frames: Vec<DueFrame>,
}

/// Which of a topic's recorded producers ran on the RANK BEING
/// REPLAYED — i.e. whose frames this pass PRODUCES rather than injects.
///
/// # Why a map, and not a set of labels
///
/// An intra-step pause names a NODE (`cerulion_core`'s `IntraStepPause`), while
/// a producer label is `node/output` — the [`ResolvedProducer::Named`]
/// vocabulary. Splitting the label back apart HERE would be the planner
/// re-deriving a join the engine already made, which is exactly what
/// [`ResolvedProducer`]'s own doc refuses for the token: nothing forbids a `/`
/// in a node id, so a split is a guess. The caller holds both halves (it built
/// the label from the manifest's `[node, output]` pair) and supplies both.
///
/// # Empty is the shipped shape
///
/// With no local producer every recorded frame is foreign, every frame folds
/// into the before-step bucket, no slot is planned, and the schedule is
/// byte-identical to the slot-free one. That is not a fallback — under the shipped
/// rank split an injected topic is one ANOTHER rank produces, so this is empty
/// on every topic the engine plans today.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct LocalProducers {
    /// Producer LABEL (`node/output`) → the producing node's id.
    by_label: BTreeMap<String, String>,
}

impl LocalProducers {
    /// No producer on this topic is local — the shipped shape.
    #[must_use]
    pub fn none() -> Self {
        Self::default()
    }

    /// From `(label, node_id)` pairs. A repeated label keeps the LAST node,
    /// which cannot arise from the engine's own build (the manifest join is a
    /// map) and is not worth a refusal in a type this small.
    #[must_use]
    pub fn from_pairs<I: IntoIterator<Item = (String, String)>>(pairs: I) -> Self {
        Self {
            by_label: pairs.into_iter().collect(),
        }
    }

    /// Does this topic have any local producer at all?
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.by_label.is_empty()
    }

    /// How many labels are claimed as local.
    #[must_use]
    pub fn len(&self) -> usize {
        self.by_label.len()
    }

    /// The local node that produced a frame, or `None` if the frame is foreign
    /// — including every frame this planner cannot ATTRIBUTE.
    ///
    /// An unattributed or unresolved frame is deliberately never claimed as
    /// local: a frame the plan cannot attribute is one it must inject or refuse
    /// (`fold_edge` has already refused a multi-publisher topic whose join
    /// failed), never one it silently assumes some local node will publish —
    /// which would drop a foreign frame from the wire entirely.
    fn node_for(&self, producer: Option<&ResolvedProducer>) -> Option<&str> {
        match producer {
            Some(ResolvedProducer::Named(label)) => self.by_label.get(label).map(String::as_str),
            _ => None,
        }
    }
}

/// Why one topic cannot be steered from its read log.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StandDown {
    /// The topic.
    pub topic: String,
    /// The reason, typed so the caller can render it AND branch on it.
    pub reason: StandDownReason,
}

impl StandDown {
    /// The caller-facing degrade sentence: what could not be steered, on which
    /// edge, why, and what happens instead.
    ///
    /// Rendered here rather than at the call site so the vocabulary is written
    /// once; the caller decides the LEVEL and where it goes (this module logs
    /// nothing — a planner that logged would make the same degrade appear twice
    /// once the caller reported it too).
    pub fn describe(&self) -> String {
        format!(
            "read-log-steered injection unavailable on '{}': {} — \
             falling back to the recorded-clock injection window",
            self.topic, self.reason
        )
    }
}

/// The refusals, each naming the edge that caused it.
///
/// A refusal is TOPIC-scoped even when one edge caused it, because injection
/// is a topic-scoped act: one re-publish stream feeds every consumer, so a
/// schedule that cannot be reconciled against one edge cannot be driven while
/// claiming the others are reproduced.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StandDownReason {
    /// No edge on this rank has any kind-6 read record, so there is nothing to
    /// steer from. Not a defect — a fully quarantined or pre-annotation rank.
    NoCoverage {
        /// Every offered edge that planned no read and carried nothing that
        /// accounts for one. An edge whose records are all hand offs is ABSENT
        /// from this list: it plans nothing to steer from and leaves nothing
        /// unexplained, so the list can be shorter than the topic's edge count.
        edges: Vec<EdgeId>,
    },
    /// An overflow marker: `dropped_records` kind-6 records were dropped at the
    /// stage rim in one merge window. Each dropped record consumed an unknown
    /// number of frames, so every `popped` sum after this point is a floor.
    TruncatedReadLog {
        /// The edge whose stage overflowed.
        edge: EdgeId,
        /// The step the marker was emitted at.
        step: u64,
        /// Records dropped in that window (NOT frames).
        dropped_records: u32,
    },
    /// A read on a multi-publisher edge could not be joined to a frame.
    UnresolvedProducer {
        /// The edge.
        edge: EdgeId,
        /// The step of the read that could not be joined.
        step: u64,
        /// Which half of the join failed.
        detail: ProducerGap,
    },
    /// The read log accounts for more frames than the bag recorded.
    FrameShortfall {
        /// The edge whose sum ran past the end.
        edge: EdgeId,
        /// The step of the read that ran past it.
        step: u64,
        /// Frames the log had consumed by the end of that read.
        needed: usize,
        /// Frames the bag actually holds for this topic.
        recorded: usize,
    },
    /// A serving read's `popped` sum lands on a frame whose sequence is not the
    /// one the record says was served — the read log and the frame stream
    /// disagree about this topic.
    JoinMismatch {
        /// The edge.
        edge: EdgeId,
        /// The step of the disagreeing read.
        step: u64,
        /// What the record says it served (`None` = the no-frame sentinel on a
        /// kind that must name a frame).
        recorded_served: Option<u32>,
        /// The sequence of the frame the `popped` sum lands on.
        frame_at_position: Option<u32>,
    },
    /// A record that SERVED a frame while popping nothing — unrepresentable in
    /// the cursor model, and not a shape any production drain site emits
    /// (`Served` is only ever staged from a drain that produced a sample).
    ServedNothingPopped {
        /// The edge.
        edge: EdgeId,
        /// The step of the record.
        step: u64,
        /// What it claims to have served.
        recorded_served: Option<u32>,
    },
    /// The earliest-wins schedule would hand a consumer a different queue than
    /// the recording did, so its drain would serve a different frame.
    ///
    /// Both modes reach it, for the same reason at opposite ends of the queue:
    /// a [`ConsumeMode::Latest`] drain takes the WHOLE queue and serves the
    /// newest, so any difference in what is pending changes its answer; a
    /// [`ConsumeMode::EachFifo`] drain pops ONE from the head, so it is
    /// insensitive to extra frames BEHIND the one it takes — but an EMPTY read
    /// has no head to be insensitive about, and a schedule that leaves anything
    /// pending turns "the queue was empty" into a pop.
    FanOutInfeasible {
        /// How this edge's drain consumes its queue — which decides both WHICH
        /// frame it would serve and how the refusal reads.
        mode: ConsumeMode,
        /// The edge the schedule cannot reproduce.
        edge: EdgeId,
        /// The step of the read it cannot reproduce.
        step: u64,
        /// What the recording says the read served.
        recorded_served: Option<u32>,
        /// What a drain-to-latest would serve under the schedule.
        would_serve: Option<u32>,
        /// Frames the schedule leaves pending in this edge's queue at that read.
        pending: usize,
        /// Frames the recording says the read consumed.
        recorded_popped: u32,
    },
    /// The schedule drives an edge's queue past its provisioned depth, so
    /// `drop_oldest` would evict a frame the recording delivered.
    QueueOverflow {
        /// The edge.
        edge: EdgeId,
        /// The step at which the queue would be over-deep.
        step: u64,
        /// Frames the schedule leaves pending.
        pending: usize,
        /// The provisioned depth.
        depth: u32,
    },
}

impl StandDownReason {
    /// A stable machine-readable code for the `--report` JSON — deliberately
    /// separate from [`fmt::Display`], which is prose and free to be reworded.
    pub fn code(&self) -> &'static str {
        match self {
            Self::NoCoverage { .. } => "no_read_log_coverage",
            Self::TruncatedReadLog { .. } => "truncated_read_log",
            Self::UnresolvedProducer { .. } => "unresolved_producer",
            Self::FrameShortfall { .. } => "frame_shortfall",
            Self::JoinMismatch { .. } => "join_mismatch",
            Self::ServedNothingPopped { .. } => "served_nothing_popped",
            Self::FanOutInfeasible { .. } => "fan_out_infeasible",
            Self::QueueOverflow { .. } => "queue_overflow",
        }
    }

    /// The edge this refusal names, when one caused it.
    pub fn edge(&self) -> Option<&EdgeId> {
        match self {
            Self::NoCoverage { .. } => None,
            Self::TruncatedReadLog { edge, .. }
            | Self::UnresolvedProducer { edge, .. }
            | Self::FrameShortfall { edge, .. }
            | Self::JoinMismatch { edge, .. }
            | Self::ServedNothingPopped { edge, .. }
            | Self::FanOutInfeasible { edge, .. }
            | Self::QueueOverflow { edge, .. } => Some(edge),
        }
    }
}

impl fmt::Display for StandDownReason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoCoverage { edges } => write!(
                f,
                "no consuming edge on this rank plans a recorded read, \
                 so there is nothing to steer from \
                 ({} edge(s) carry no read record)",
                edges.len()
            ),
            Self::TruncatedReadLog {
                edge,
                step,
                dropped_records,
            } => write!(
                f,
                "{edge}'s read log is truncated at step {step} \
                 ({dropped_records} record(s) dropped at the stage rim), \
                 so the frames consumed past that point are not countable"
            ),
            Self::UnresolvedProducer { edge, step, detail } => write!(
                f,
                "{edge}'s read at step {step} is on a multi-publisher topic and \
                 cannot be joined to a frame: {detail}"
            ),
            Self::FrameShortfall {
                edge,
                step,
                needed,
                recorded,
            } => write!(
                f,
                "{edge}'s read log accounts for {needed} frame(s) by step {step} \
                 but the bag records only {recorded}"
            ),
            Self::JoinMismatch {
                edge,
                step,
                recorded_served,
                frame_at_position,
            } => write!(
                f,
                "{edge}'s read at step {step} says it served sequence {} \
                 but its own popped count lands on {}",
                render_seq(*recorded_served),
                render_seq(*frame_at_position)
            ),
            Self::ServedNothingPopped {
                edge,
                step,
                recorded_served,
            } => write!(
                f,
                "{edge}'s read at step {step} served sequence {} while popping \
                 nothing off the queue",
                render_seq(*recorded_served)
            ),
            Self::FanOutInfeasible {
                mode,
                edge,
                step,
                recorded_served,
                would_serve,
                pending,
                recorded_popped,
            } => write!(
                f,
                "{edge} is a {} consumer whose read at step {step} \
                 cannot be reproduced: the fan-out schedule leaves {pending} frame(s) \
                 pending where the recording consumed {recorded_popped}, so the drain \
                 would serve sequence {} where the recording served {}",
                match mode {
                    ConsumeMode::Latest => "drain-to-latest",
                    ConsumeMode::EachFifo => "per-message FIFO",
                },
                render_seq(*would_serve),
                render_seq(*recorded_served)
            ),
            Self::QueueOverflow {
                edge,
                step,
                pending,
                depth,
            } => write!(
                f,
                "{edge}'s queue would hold {pending} frame(s) at step {step} against \
                 a provisioned depth of {depth}, so the schedule would evict a frame \
                 the recording delivered"
            ),
        }
    }
}

/// Which half of a multi-publisher join failed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProducerGap {
    /// The read carried no preceding [`ReadOutcomeKind::Producer`] annotation,
    /// so the recording never named the frame's publisher — a bag predating the
    /// annotations, or one whose annotation did not reach the file.
    MissingAnnotation,
    /// The annotation's token hashes to two or more distinct publishers.
    AnnotationCollision,
    /// No rank manifest names the annotation's token.
    AnnotationForeign,
    /// The bag's frame carries no producer label, so there is nothing to join
    /// the annotation against: this bag predates the record-time
    /// labels, its label stream did not reach the frame, or the recorder never
    /// ARMED labelling on the topic.
    ///
    /// That last one is narrower than it reads: the recorder builds a labelling
    /// for EVERY tap. Handed the `multi_publisher_topics` declaration it labels
    /// from the topic's first frame; handed none it arms on OBSERVED plurality
    /// and labels from the frame a SECOND writer is first seen on. So an
    /// undeclared tap is not exempt — it arms later, not never — and the run
    /// that carries no label at all is the one that only ever had one writer.
    ///
    /// The planner cannot tell those apart — it never sees the bag — so this
    /// renders every branch and defers the diagnosis to the engine's
    /// `producer_labels` note, which read the channel table and can.
    UnattributedFrame {
        /// The frame's index in file order.
        frame_index: usize,
    },
    /// The frame's own attribution is itself unresolved.
    FrameProducerUnresolved {
        /// The frame's index in file order.
        frame_index: usize,
    },
    /// Both sides resolved and they name DIFFERENT publishers.
    Mismatch {
        /// What the read's annotation named.
        annotated: String,
        /// What the frame the popped sum landed on is attributed to.
        frame: String,
    },
}

impl fmt::Display for ProducerGap {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingAnnotation => write!(
                f,
                "the read carries no producer annotation — this bag predates the read-time \
                 producer annotations, or the annotation did not reach the bag; re-record \
                 with a current build to steer this topic"
            ),
            Self::AnnotationCollision => write!(
                f,
                "the read's producer token hashes to more than one publisher"
            ),
            Self::AnnotationForeign => write!(
                f,
                "no rank manifest names the read's producer token (foreign publisher)"
            ),
            Self::UnattributedFrame { frame_index } => write!(
                f,
                "recorded frame {frame_index} carries no producer label — this bag predates \
                 producer labels, its label stream did not reach the frame, or the recorder \
                 never armed labelling on this topic (only ever ONE writer was observed on it: \
                 a declared tap labels from the first frame and an undeclared one from the \
                 frame a second writer first appears, so neither writes a label for a genuinely \
                 single-writer run). The run's producer-labels note says which; re-recording \
                 helps only in the first two cases"
            ),
            Self::FrameProducerUnresolved { frame_index } => write!(
                f,
                "recorded frame {frame_index}'s producer is itself unresolved"
            ),
            Self::Mismatch { annotated, frame } => write!(
                f,
                "the read names publisher '{annotated}' but the frame is attributed \
                 to '{frame}'"
            ),
        }
    }
}

/// The planner's verdict for one topic.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TopicPlan {
    /// Drive this schedule.
    Steered(InjectionSchedule),
    /// Fall back to the wall-time window and report the named degrade.
    StandDown(StandDown),
}

fn render_seq(seq: Option<u32>) -> String {
    match seq {
        Some(s) => s.to_string(),
        None => "<none>".to_string(),
    }
}

// ===========================================================================
// The planner
// ===========================================================================

/// One read record with its annotations folded away and its frame range
/// resolved — the intermediate the feasibility pass walks.
#[derive(Debug)]
struct PlannedRead {
    step: u64,
    /// Half-open range of FRAME INDICES this read consumed.
    start: usize,
    end: usize,
    served_seq: Option<u32>,
}

/// One covered edge, folded.
#[derive(Debug)]
struct EdgePlan {
    id: EdgeId,
    mode: ConsumeMode,
    depth: Option<u32>,
    reads: Vec<PlannedRead>,
}

/// ONE step's bucket while the derivation walks it — the
/// before-step frames, the intra-step slots, and the anchor state that decides
/// which of the two a foreign frame lands in.
#[derive(Debug, Default)]
struct StepGroup {
    /// Foreign frames placed BEFORE this step's first local fire.
    before: Vec<DueFrame>,
    /// Foreign frames placed after one, in anchor order.
    slots: Vec<SlotInjection>,
    /// This step's fire ordinal PER local producer node. Per node and not a
    /// single running count, because `after_fire` counts the fires of the ONE
    /// node the pause names — two local producers on one topic each have their
    /// own burst.
    fires: BTreeMap<String, u32>,
    /// The most recent local frame's `(node, that node's ordinal in this step)`
    /// — what a foreign frame attaches to. `None` until this step's first local
    /// frame, which is exactly the before-step bucket's condition.
    anchor: Option<(String, u32)>,
}

impl StepGroup {
    /// A LOCAL frame: it is never injected, and it MOVES the anchor.
    fn note_local(&mut self, node: &str) {
        let ordinal = self.fires.entry(node.to_string()).or_insert(0);
        *ordinal = ordinal.saturating_add(1);
        self.anchor = Some((node.to_string(), *ordinal));
    }

    /// A FOREIGN frame: before-step while no local fire has happened yet in
    /// this step, otherwise the current anchor's slot.
    fn push_foreign(&mut self, due: DueFrame) {
        let Some((node, after_fire)) = self.anchor.clone() else {
            self.before.push(due);
            return;
        };
        // FOLDED onto the last slot when the anchor has not moved, because two
        // pauses at one `(node, after_fire)` is what the scheduler refuses. The
        // anchor is strictly new every time it moves (a local frame always
        // increments its node's ordinal), so the LAST slot is the only one that
        // can match and no scan is needed.
        match self.slots.last_mut() {
            Some(slot) if slot.producer_node == node && slot.after_fire == after_fire => {
                slot.frames.push(due);
            }
            _ => self.slots.push(SlotInjection {
                producer_node: node,
                after_fire,
                frames: vec![due],
            }),
        }
    }

    /// Nothing to inject at this step — a group that exists only because a
    /// local frame passed through it.
    fn is_inert(&self) -> bool {
        self.before.is_empty() && self.slots.is_empty()
    }
}

/// Plan the read-log-steered injection for ONE topic on the rank being
/// replayed.
///
/// `local` names the producers that run on THIS rank, whose frames the pass
/// publishes live rather than injecting. Pass
/// [`LocalProducers::none`] for a topic produced entirely off-rank — which is
/// every topic the shipped rank split injects, and the shape whose plan is
/// byte-identical to the slot-free one.
///
/// See the module docs for the rules. Pure and total: every input shape yields
/// either a schedule or a typed stand-down, and nothing is logged.
pub fn plan_topic_injection(topic: &TopicReplay, local: &LocalProducers) -> TopicPlan {
    let mut covered: Vec<EdgePlan> = Vec::new();
    let mut uncovered: Vec<EdgeId> = Vec::new();

    for edge in &topic.edges {
        match fold_edge(edge, topic) {
            Err(reason) => {
                return TopicPlan::StandDown(StandDown {
                    topic: topic.topic.clone(),
                    reason,
                })
            }
            Ok(EdgeFold::Planned(plan)) => covered.push(plan),
            // Covered, nothing to do. Pushing it onto `uncovered` would report a
            // read this schedule was not checked against, and every record on
            // the edge names a frame an earlier record consumed.
            Ok(EdgeFold::Covered) => {}
            Ok(EdgeFold::NoReads) => uncovered.push(edge.id.clone()),
        }
    }

    if covered.is_empty() {
        return TopicPlan::StandDown(StandDown {
            topic: topic.topic.clone(),
            reason: StandDownReason::NoCoverage { edges: uncovered },
        });
    }

    // Earliest-wins: a frame goes on the wire at the earliest step ANY covered
    // consumer accounted for it. Each edge covers a PREFIX of the frame stream
    // (its cursor starts at 0 and only advances) with non-decreasing steps, so
    // the pointwise minimum is itself non-decreasing in the frame index — the
    // property the feasibility pass's `partition_point` relies on.
    let mut earliest: Vec<Option<u64>> = vec![None; topic.frames.len()];
    for plan in &covered {
        for read in &plan.reads {
            for slot in &mut earliest[read.start..read.end] {
                *slot = Some(match *slot {
                    Some(prev) => prev.min(read.step),
                    None => read.step,
                });
            }
        }
    }

    if let Err(reason) = verify_fan_out(&covered, &earliest, topic, local) {
        return TopicPlan::StandDown(StandDown {
            topic: topic.topic.clone(),
            reason,
        });
    }

    // The LABEL-PARTITIONED walk. One pass over the frame stream
    // in FILE order, placing each frame by what produced it.
    //
    // WHY FILE ORDER IS THE INTERLEAVE, and it is an assumption worth stating:
    // one topic is recorded through ONE tap on ONE channel, so its file order
    // is the order the recorder drained its commits — the same order a
    // consumer's queue held them. Restricted to the frames of ONE step that is
    // exactly the recorded serve order, which is what a slot has to reproduce.
    // The read log is what says WHICH step a frame belongs to; the file order
    // is what says where it sits inside it. Neither alone is enough, and
    // nothing else in the bag carries the sub-step position.
    //
    //   LOCAL   → never injected; it ADVANCES its node's fire ordinal and
    //             becomes the anchor every later foreign frame of this step
    //             attaches to.
    //   FOREIGN → the before-step bucket while the step has no anchor yet,
    //             otherwise the anchor's intra-step slot.
    //
    // With no local producer the anchor never arms, so every frame takes the
    // before-step arm and this is the slot-free loop.
    let mut by_step: BTreeMap<u64, StepGroup> = BTreeMap::new();
    let mut unconsumed: Vec<DueFrame> = Vec::new();
    let mut local_frames: Vec<DueFrame> = Vec::new();
    for (idx, slot) in earliest.iter().enumerate() {
        let frame = &topic.frames[idx];
        let due = DueFrame {
            frame_index: idx,
            sequence: frame.sequence,
        };
        match (slot, local.node_for(frame.producer.as_ref())) {
            (step, Some(node)) => {
                // A local frame NO covered edge accounts for still is not an
                // injection gap — it is produced live either way. It anchors
                // nothing, because nothing says which step it fell in.
                if let Some(step) = step {
                    by_step.entry(*step).or_default().note_local(node);
                }
                local_frames.push(due);
            }
            (Some(step), None) => by_step.entry(*step).or_default().push_foreign(due),
            (None, None) => unconsumed.push(due),
        }
    }

    TopicPlan::Steered(InjectionSchedule {
        topic: topic.topic.clone(),
        per_step: by_step
            .into_iter()
            // A step with nothing to inject is OMITTED — a
            // group can exist purely because a local frame passed through
            // it, and an empty entry would make the caller's cursor spend a
            // step's worth of nothing.
            .filter(|(_, group)| !group.is_inert())
            .map(|(before_step, group)| StepInjection {
                before_step,
                frames: group.before,
                slots: group.slots,
                local_fire_ordinals: group.fires,
            })
            .collect(),
        uncovered,
        unconsumed,
        local_frames,
    })
}

/// What folding ONE edge's recorded stream answered.
///
/// THREE answers, not an `Option`'s two. A stream that plans no read splits into
/// two different facts, and one `None` writes both: a stream whose every record
/// is already accounted for, and a stream that accounts for nothing. The first
/// leaves no read unexplained on the edge, the second leaves the whole edge
/// unexplained, and only the second is a coverage gap the caller reports.
enum EdgeFold {
    /// The stream plans this schedule of reads.
    Planned(EdgePlan),
    /// COVERED, nothing to do: the stream carries at least one hand off and no
    /// record that plans a read. A hand off names a frame an earlier record on
    /// this edge already consumed, so it occupies no position in the frame
    /// stream and needs no injection: the edge is explained without a schedule.
    Covered,
    /// The stream plans no read and carries nothing that accounts for one: an
    /// empty stream, or annotations alone, which name the producer of a read
    /// without being a read.
    NoReads,
}

/// Fold one edge's recorded stream: drop the annotations onto the reads they
/// annotate, resolve each read's frame range, and cross-check every serving
/// read against the frame its own `popped` sum lands on.
///
/// The three answers are [`EdgeFold`]'s arms.
fn fold_edge(edge: &ConsumerEdge, topic: &TopicReplay) -> Result<EdgeFold, StandDownReason> {
    let mut pending_token: Option<&ResolvedProducer> = None;
    let mut cursor = 0usize;
    let mut reads: Vec<PlannedRead> = Vec::new();
    // Set at the hand off skip below. A `Truncated` body returns and a
    // `Producer` body continues before that skip, so `HandOff` is the only body
    // that reaches it.
    let mut covered_without_pop = false;

    for record in &edge.reads {
        match &record.body {
            // An overflow marker means RECORDS are missing here, each having
            // consumed an unknown number of frames. Everything after it is a
            // floor, so the correct answer is to stop rather than to plan a
            // schedule that under-injects by an unknown amount.
            RecordedReadBody::Truncated { dropped_records } => {
                return Err(StandDownReason::TruncatedReadLog {
                    edge: edge.id.clone(),
                    step: record.step,
                    dropped_records: *dropped_records,
                })
            }
            // A producer annotation names the publisher of the frame the NEXT
            // read serves. It consumes nothing and occupies no position in the
            // frame stream.
            RecordedReadBody::Producer { token } => {
                pending_token = Some(token);
                continue;
            }
            // Both remaining bodies CONSUME the pending annotation below.
            RecordedReadBody::Read { .. } | RecordedReadBody::HandOff { .. } => {}
        }

        // Taken unconditionally, including by the hand-off that does not join:
        // an annotation belongs to exactly the read that follows it, so leaving
        // it armed would attribute it to a LATER read.
        let token = pending_token.take();

        // A per-set Sync PROMOTION names a frame an earlier peek on
        // this edge already consumed, so it advances no cursor and joins to no
        // position. Skipped AFTER the token is taken, for the reason above.
        // See `RecordedReadBody::HandOff`.
        let RecordedReadBody::Read {
            kind,
            served_seq,
            popped,
        } = &record.body
        else {
            covered_without_pop = true;
            continue;
        };
        let (kind, served_seq) = (*kind, *served_seq);

        let popped = *popped as usize;
        let end = cursor + popped;
        if end > topic.frames.len() {
            return Err(StandDownReason::FrameShortfall {
                edge: edge.id.clone(),
                step: record.step,
                needed: end,
                recorded: topic.frames.len(),
            });
        }

        // Only the two SERVING kinds name a frame this read just consumed —
        // see [`ReadKind::serves`] for why the other three cannot.
        if kind.serves() {
            if popped == 0 {
                return Err(StandDownReason::ServedNothingPopped {
                    edge: edge.id.clone(),
                    step: record.step,
                    recorded_served: served_seq,
                });
            }
            // A `Latest` drain serves the NEWEST of what it popped and an
            // `EachFifo` pop-one serves its only frame: both are the LAST frame
            // of the range, so one rule covers both modes.
            let landed = &topic.frames[end - 1];
            if served_seq != Some(landed.sequence) {
                return Err(StandDownReason::JoinMismatch {
                    edge: edge.id.clone(),
                    step: record.step,
                    recorded_served: served_seq,
                    frame_at_position: Some(landed.sequence),
                });
            }
            if topic.multi_publisher {
                verify_producer(token, landed, end - 1).map_err(|detail| {
                    StandDownReason::UnresolvedProducer {
                        edge: edge.id.clone(),
                        step: record.step,
                        detail,
                    }
                })?;
            }
        }

        reads.push(PlannedRead {
            step: record.step,
            start: cursor,
            end,
            served_seq,
        });
        cursor = end;
    }

    if reads.is_empty() {
        return Ok(if covered_without_pop {
            EdgeFold::Covered
        } else {
            EdgeFold::NoReads
        });
    }
    Ok(EdgeFold::Planned(EdgePlan {
        id: edge.id.clone(),
        mode: edge.mode,
        depth: edge.depth,
        reads,
    }))
}

/// On a multi-publisher topic a wire `sequence` is per-publisher, so the
/// sequence match in `fold_edge` is necessary but not sufficient: the frame the
/// cursor landed on must also be the one the recorded annotation names.
fn verify_producer(
    token: Option<&ResolvedProducer>,
    frame: &RecordedFrame,
    frame_index: usize,
) -> Result<(), ProducerGap> {
    let annotated = match token {
        None => return Err(ProducerGap::MissingAnnotation),
        Some(ResolvedProducer::Collision) => return Err(ProducerGap::AnnotationCollision),
        Some(ResolvedProducer::Foreign) => return Err(ProducerGap::AnnotationForeign),
        Some(ResolvedProducer::Named(label)) => label,
    };
    let frame_label = match &frame.producer {
        None => return Err(ProducerGap::UnattributedFrame { frame_index }),
        Some(ResolvedProducer::Collision) | Some(ResolvedProducer::Foreign) => {
            return Err(ProducerGap::FrameProducerUnresolved { frame_index })
        }
        Some(ResolvedProducer::Named(label)) => label,
    };
    if annotated != frame_label {
        return Err(ProducerGap::Mismatch {
            annotated: annotated.clone(),
            frame: frame_label.clone(),
        });
    }
    Ok(())
}

/// Replay the earliest-wins schedule against every covered edge and require
/// each recorded read to be reproducible under it.
///
/// The whole asymmetry of fan-out lives here. A [`ConsumeMode::Latest`] edge
/// drains the WHOLE queue and serves the newest, so the queue it is handed must
/// hold EXACTLY what the recording handed it. A [`ConsumeMode::EachFifo`] edge
/// pops one frame per read in arrival order, so an early injection only deepens
/// its queue BEHIND the head and it still serves the same frame.
///
/// That last argument has one hole, and it is the empty read: a read that
/// popped NOTHING has no head to be insensitive about, so "the queue was empty"
/// is a claim about the queue's DEPTH and the schedule has to honour it. Both
/// modes are therefore checked; what differs is WHICH frame the replayed drain
/// would serve — the newest pending for `Latest`, the head for `EachFifo`.
fn verify_fan_out(
    covered: &[EdgePlan],
    earliest: &[Option<u64>],
    topic: &TopicReplay,
    local: &LocalProducers,
) -> Result<(), StandDownReason> {
    for plan in covered {
        let mut cursor = 0usize;
        for read in &plan.reads {
            // `earliest` is non-decreasing over its `Some` prefix (see
            // `plan_topic_injection`), so the frames on the wire by this step
            // are exactly a prefix and `partition_point` finds its end.
            let admitted =
                earliest.partition_point(|slot| slot.is_some_and(|step| step <= read.step));
            // The injected frames and
            // a LOCAL producer's live publishes ride the SAME edge but NOT
            // the same iceoryx2 CONNECTION — capacity is enforced per
            // publisher, so `QueueOverflow` may convict only when the
            // DEEPEST single connection exceeds the depth, never the merged
            // total (a valid co-located topic where every connection fits
            // was previously stood down because their SUM did not).
            let occupancy = pending_occupancy(topic, local, cursor.min(admitted)..admitted);
            let pending = occupancy.total();

            if let Some(depth) = plan.depth {
                let worst_connection = occupancy.max_connection();
                if worst_connection > depth as usize {
                    return Err(StandDownReason::QueueOverflow {
                        edge: plan.id.clone(),
                        step: read.step,
                        pending: worst_connection,
                        depth,
                    });
                }
            }

            match plan.mode {
                ConsumeMode::Latest => {
                    if admitted != read.end {
                        let would_serve =
                            (admitted > cursor).then(|| topic.frames[admitted - 1].sequence);
                        return Err(StandDownReason::FanOutInfeasible {
                            mode: plan.mode,
                            edge: plan.id.clone(),
                            step: read.step,
                            recorded_served: read.served_seq,
                            would_serve,
                            pending,
                            recorded_popped: (read.end - read.start) as u32,
                        });
                    }
                }
                ConsumeMode::EachFifo => {
                    // Every frame in this read's own range has an earliest step
                    // of at most THIS edge's step (the minimum is taken over a
                    // set that includes it), so the frames it consumed are
                    // always on the wire by now. FIFO order then preserves which
                    // one it serves. Asserted rather than branched: a `return`
                    // here would be a refusal no input can reach, and a refusal
                    // no test can drive is worse than an invariant that says so.
                    debug_assert!(
                        admitted >= read.end,
                        "earliest-wins cannot place a frame after the read that consumed it \
                         ({} < {} on {})",
                        admitted,
                        read.end,
                        plan.id
                    );
                    // The invariant above is about the
                    // frames this read CONSUMED, and over an EMPTY read it is
                    // vacuous — which is exactly the shape the mode's
                    // insensitivity argument does not cover. A recorded
                    // `NoFrame` says the queue was EMPTY at this step; if the
                    // schedule has admitted anything this edge has not taken
                    // yet, the replayed pop-one finds a head and SERVES it, and
                    // the edge's cursor is off by one for the rest of the
                    // stream.
                    //
                    // It is not a corrupt-bag shape. All reads of a step carry
                    // ONE step number (`merge_read_outcomes` stamps the
                    // level-end merge), so a level-0 FIFO body read that found
                    // nothing and a later-level consumer that took the frame
                    // later in the SAME step are both stamped `S` — the frame
                    // was published between them — and earliest-wins then places
                    // it before the step, in front of the read that recorded
                    // silence. The `Latest` arm already refuses its own version
                    // of this (its `would_serve` is written for the empty-read
                    // case); this is the FIFO half.
                    //
                    // The frame it would serve is the queue HEAD — `cursor`, not
                    // `admitted - 1` — because a FIFO pop takes the oldest.
                    if read.end == read.start && admitted > cursor {
                        return Err(StandDownReason::FanOutInfeasible {
                            mode: plan.mode,
                            edge: plan.id.clone(),
                            step: read.step,
                            recorded_served: read.served_seq,
                            would_serve: Some(topic.frames[cursor].sequence),
                            pending,
                            recorded_popped: 0,
                        });
                    }
                }
            }
            cursor = read.end;
        }
    }
    Ok(())
}

/// Per-connection occupancy of a queue's PENDING span.
///
/// iceoryx2 applies subscriber capacity PER PUBLISHER CONNECTION, not as one
/// shared pool a topic's writers jointly deepen: `drop_oldest` eviction is a
/// property of ONE publisher's own backlog against the subscriber's declared
/// depth. So a co-located topic's LOCAL producer node(s) and the INJECTOR
/// (which carries every frame this rank's own [`LocalProducers`] table does
/// not attribute — every frame another rank, or an out-of-graph publisher,
/// produced) are each their OWN connection, each independently capped at the
/// edge's depth — never summed into one pool, which a two-term
/// return would invite the caller to do.
///
/// Empty (every count zero) whenever `local` is empty, which is every topic
/// the shipped rank split injects.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
struct ConnectionOccupancy {
    /// Local producer node id → frames pending on ITS OWN connection.
    by_local_node: BTreeMap<String, usize>,
    /// Frames pending on the injector's ONE connection.
    injected: usize,
}

impl ConnectionOccupancy {
    /// The MERGED total across every connection — still the right quantity
    /// for a claim about the schedule's overall pending count
    /// ([`StandDownReason::FanOutInfeasible`]'s `pending` field, which
    /// explains a fan-out MISMATCH rather than asserting an eviction), never
    /// for deciding [`StandDownReason::QueueOverflow`] (see
    /// [`Self::max_connection`]).
    fn total(&self) -> usize {
        self.injected + self.by_local_node.values().sum::<usize>()
    }

    /// The DEEPEST single connection — the quantity `drop_oldest` actually
    /// enforces eviction against, and the ONLY one a [`QueueOverflow`]
    /// refusal may convict on. A topic whose local producer(s) and the
    /// injector each sit within the declared depth never evicts, even when
    /// their SUM exceeds it — summing is the defect this pins: it
    /// stood a valid co-located topic down as `QueueOverflow`
    /// on a depth every real connection actually satisfied.
    ///
    /// [`QueueOverflow`]: StandDownReason::QueueOverflow
    fn max_connection(&self) -> usize {
        self.by_local_node
            .values()
            .copied()
            .chain(std::iter::once(self.injected))
            .max()
            .unwrap_or(0)
    }
}

fn pending_occupancy(
    topic: &TopicReplay,
    local: &LocalProducers,
    span: std::ops::Range<usize>,
) -> ConnectionOccupancy {
    let mut occupancy = ConnectionOccupancy::default();
    for f in &topic.frames[span] {
        match local.node_for(f.producer.as_ref()) {
            Some(node) => *occupancy.by_local_node.entry(node.to_string()).or_insert(0) += 1,
            None => occupancy.injected += 1,
        }
    }
    occupancy
}

// ===========================================================================
// The ENFORCEMENT planner
// ===========================================================================

/// Which read-outcome stages one wired input owns, and which of them the core
/// will GATE.
///
/// Existence and gateability are two facts and both are needed. WHICH stage
/// staged a record is decided by existence (a record's site names a stage only
/// where that stage exists), while whether the gate may hold that stage's frames
/// back is a wiring fact the core answers: a per-set Sync trigger input pops
/// outside the two gated bodies, and a multi-publisher edge's wire sequence is
/// per publisher, so neither can be gated. A stage the core refuses is UNGATED
/// for the whole run and takes today's drain byte for byte, which is the answer
/// for every OTHER ungated edge in this engine.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StageRoles {
    /// The node body's own read stage exists.
    pub body: bool,
    /// A runtime-owned trigger/sync drain stage exists.
    pub drain: bool,
    /// The body stage can be gated.
    pub body_gateable: bool,
    /// The drain stage can be gated.
    pub drain_gateable: bool,
}

impl StageRoles {
    /// Can the stage this role names be gated?
    fn gateable(self, role: ReadStageRole) -> bool {
        match role {
            ReadStageRole::Body => self.body_gateable,
            ReadStageRole::Drain => self.drain_gateable,
        }
    }
}

/// The name-to-index table a recorded read's input name resolves through, plus
/// the stages this build wires.
///
/// Both halves are needed and neither can stand in for the other: the RECORDING
/// names an input by the position the recorder stamped
/// ([`crate::replay_engine`] resolves `input_idx` to a name through the bag's
/// manifest), so going back from a name to a `StageKey` inverts that same table,
/// while WHICH stages exist at that index is a fact about the graph this replay
/// built.
#[derive(Debug, Clone, Default)]
pub struct StageTable {
    /// Node id to its ordered recorded input names, the bag's own table.
    inputs: BTreeMap<String, Vec<String>>,
    /// `(node, input_idx)` to the stages this build wired there.
    stages: BTreeMap<(String, u16), StageRoles>,
    /// The core's own reason for each stage it refuses to gate. A `Vec` because
    /// `StageKey` is a hashable identity and not an ordered one, and the list is
    /// one entry per refused stage on one rank.
    refusals: Vec<(StageKey, ReadPlanRefusal)>,
}

impl StageTable {
    /// From the bag's manifest input tables and the core's own stage census.
    #[must_use]
    pub fn new(inputs: &BTreeMap<String, Vec<String>>, census: &[ReadEdgeCapability]) -> Self {
        let mut stages: BTreeMap<(String, u16), StageRoles> = BTreeMap::new();
        for cap in census {
            let roles = stages
                .entry((cap.key.node.clone(), cap.key.input_idx))
                .or_default();
            match cap.key.role {
                ReadStageRole::Body => {
                    roles.body = true;
                    roles.body_gateable = cap.enforceable;
                }
                ReadStageRole::Drain => {
                    roles.drain = true;
                    roles.drain_gateable = cap.enforceable;
                }
            }
        }
        Self {
            inputs: inputs.clone(),
            stages,
            refusals: census
                .iter()
                .filter_map(|c| c.reason.clone().map(|r| (c.key.clone(), r)))
                .collect(),
        }
    }

    /// The core's reason for refusing to gate this stage, when it named one.
    #[must_use]
    pub fn refusal(&self, key: &StageKey) -> Option<&ReadPlanRefusal> {
        self.refusals.iter().find(|(k, _)| k == key).map(|(_, r)| r)
    }

    /// Resolve one consuming input to its recorded index and its wired stages.
    ///
    /// The DUPLICATE arm is scoped to the name being resolved: a node whose
    /// table repeats some OTHER name still resolves this one to exactly one
    /// index, and refusing the run for it would refuse a bag whose ambiguity
    /// touches no gated edge.
    pub fn resolve(
        &self,
        node: &str,
        input: &str,
    ) -> Result<(u16, StageRoles), AdmissionRefusalReason> {
        let unresolved = |detail: String| AdmissionRefusalReason::InputNameUnresolved {
            node: node.to_string(),
            input: input.to_string(),
            detail,
        };
        let Some(names) = self.inputs.get(node) else {
            return Err(unresolved(
                "no rank manifest carries an input table for this node".to_string(),
            ));
        };
        let mut found: Vec<usize> = Vec::new();
        for (idx, name) in names.iter().enumerate() {
            if name == input {
                found.push(idx);
            }
        }
        match found.as_slice() {
            [] => {
                return Err(unresolved(format!(
                    "the node's recorded input table ({}) does not name it",
                    render_names(names)
                )))
            }
            [_one] => {}
            [first, second, ..] => {
                return Err(AdmissionRefusalReason::InputNameDuplicated {
                    node: node.to_string(),
                    name: input.to_string(),
                    first_idx: u16::try_from(*first).unwrap_or(u16::MAX),
                    second_idx: u16::try_from(*second).unwrap_or(u16::MAX),
                })
            }
        }
        let idx = u16::try_from(found[0]).map_err(|_| {
            unresolved("its recorded index is past the 16-bit wire field".to_string())
        })?;
        let Some(roles) = self.stages.get(&(node.to_string(), idx)) else {
            return Err(unresolved(format!(
                "this build wires no read-outcome stage at index {idx} of that node"
            )));
        };
        Ok((idx, *roles))
    }
}

/// Render an input-name table for a refusal sentence.
fn render_names(names: &[String]) -> String {
    if names.is_empty() {
        return "<empty>".to_string();
    }
    names.join(", ")
}

/// One recorded kind-6 record with the read SITE the wire stamped.
///
/// The site rides beside the record rather than inside it because
/// [`RecordedRead`] is the STEERING vocabulary, where the site is already spent
/// splitting one input's stream into two queues. The enforcement planner needs
/// the opposite: one stream per input, in record order, with the site as a
/// per-record label, so a record can be placed on the stage that STAGED it
/// (both sites stage into one `Body` stage under the unified discipline) without
/// the record order being lost to a pre-split.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SitedRead {
    /// The role the wire carries. [`ReadSiteRole::Unstamped`] on every
    /// `trace_format` <= 4 bag, where the placement below takes the pre-roles
    /// kind rule.
    pub role: ReadSiteRole,
    /// The record itself.
    pub read: RecordedRead,
}

/// One consuming input on the replaying rank, with its whole recorded stream.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AdmissionEdge {
    /// The consuming node's id.
    pub node_id: String,
    /// The input port's own name.
    pub input: String,
    /// How its drain consumes the queue; carried for the refusal sentences.
    pub mode: ConsumeMode,
    /// Every kind-6 record of this input, in RECORD ORDER, both sites
    /// interleaved.
    pub reads: Vec<SitedRead>,
}

/// One graph-produced topic's whole enforcement input.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TopicAdmission {
    /// The canonical topic name (reporting only).
    pub topic: String,
    /// Whether the topic is `multi_publisher_topics`-listed. Carried so the
    /// planner can name it in a refusal; the core census is what refuses such an
    /// edge, since a per-publisher wire sequence names no single head.
    pub multi_publisher: bool,
    /// The consuming inputs on the rank being replayed.
    pub edges: Vec<AdmissionEdge>,
    /// The last step this rank's recorded boundary stream carries.
    ///
    /// A stage whose read stream stops below it plans no admission for the
    /// remaining steps, and an armed stage with no admission WITHHOLDS, so the
    /// consumer would read nothing from that step on. That is the
    /// [`AdmissionRefusalReason::StreamEndsEarly`] refusal rather than a replay
    /// whose tail is gated to empty.
    pub last_boundary_step: u64,
}

/// One gated stage's recorded admissions.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StageAdmission {
    /// The stage the core gates.
    pub key: StageKey,
    /// How the edge's drain consumes its queue; carried for the sentences.
    pub mode: ConsumeMode,
    /// Ascending in step. A step the recording holds a read for is ALWAYS
    /// present, including one that admits nothing.
    pub per_step: Vec<StepAdmission>,
}

/// One step's admissions at one stage, in RECORDED ORDER.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StepAdmission {
    /// The recorded step.
    pub step: u64,
    /// This step's reads, in record order. An entry whose pop count is 0 is a
    /// DECLARATION (the recorded none, or the served hand off), never an
    /// omission.
    pub due: Vec<DueRead>,
}

/// What the planner answered for one topic.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AdmissionPlan {
    /// Gate these stages.
    Enforced {
        /// The gated stages. Never empty.
        stages: Vec<StageAdmission>,
        /// Stages of the same topic the core refuses to gate, each with its
        /// reason: they take today's drain, and the report says so rather than
        /// implying the whole topic was gated.
        ungateable: Vec<(StageKey, String)>,
    },
    /// NO stage of this topic's consuming inputs can be gated on WIRING facts,
    /// each with the core's own reason.
    ///
    /// Not a refusal: the stages are ungated, so the replay takes today's drain
    /// on them, exactly as it does for every edge outside a gated rank. It is a
    /// third answer rather than an empty [`Self::Enforced`] because the report
    /// has to say which of the two it was, and an empty plan would read as "the
    /// gate ran and held nothing".
    NotGateable(Vec<(StageKey, String)>),
    /// This recording cannot be enforced on this topic.
    Refused(AdmissionRefusal),
}

/// A refusal, naming the topic it was found on.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AdmissionRefusal {
    /// The topic.
    pub topic: String,
    /// The reason, typed so the caller can render it AND branch on it.
    pub reason: AdmissionRefusalReason,
}

/// A record shape the plan cannot place on a step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecordShape {
    /// It consumed frames while naming none, so the surviving sequence the gate
    /// compares against is unstated.
    PoppedWithoutSequence,
    /// A serving kind that popped nothing and is not the hand off shape
    /// [`RecordedReadBody::HandOff`] carries.
    ServedNothingPopped,
}

impl RecordShape {
    /// The stable token, rendered in the refusal detail.
    #[must_use]
    pub fn code(self) -> &'static str {
        match self {
            Self::PoppedWithoutSequence => "popped_without_sequence",
            Self::ServedNothingPopped => "served_nothing_popped",
        }
    }
}

/// Why this recording's read log cannot be ENFORCED on a graph-produced edge.
///
/// Every arm is a fact about what the RECORDING can support, never about the
/// candidate: the engine maps all of them to one exit-2 refusal, because
/// enforcing what can be enforced and falling back on the rest is the
/// coincidence the enforcement exists to remove.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AdmissionRefusalReason {
    /// The bag carries kind-6 records for this rank, and none of them lands on
    /// a stage of this topic's consuming inputs.
    NoCoverage {
        /// Every stage the topic's edges offered.
        keys: Vec<StageKey>,
    },
    /// An overflow marker: records were dropped at the stage rim, so every
    /// later popped count on that stage is a floor.
    RecordDropped {
        /// The stage that overflowed.
        key: StageKey,
        /// The step the marker was emitted at.
        step: u64,
        /// Records dropped in that window (NOT frames).
        dropped_records: u32,
    },
    /// The stage's read stream stops below the rank's last recorded boundary.
    StreamEndsEarly {
        /// The stage.
        key: StageKey,
        /// The last step the stream holds a read for.
        last_read_step: u64,
        /// The last step the rank's boundary stream carries.
        last_boundary_step: u64,
    },
    /// A record whose shape names no admission.
    UnenforceableRecord {
        /// The stage.
        key: StageKey,
        /// The record's step.
        step: u64,
        /// Which shape it is.
        shape: RecordShape,
    },
    /// A gated input's name does not resolve to exactly one wired stage.
    InputNameUnresolved {
        /// The consuming node.
        node: String,
        /// The input name the record carried.
        input: String,
        /// Which half of the resolution failed.
        detail: String,
    },
    /// A gated input's recorded name appears twice in its node's table, so the
    /// records naming it address two stages.
    InputNameDuplicated {
        /// The consuming node.
        node: String,
        /// The repeated name.
        name: String,
        /// The first index carrying it.
        first_idx: u16,
        /// The second.
        second_idx: u16,
    },
    /// The core's pre-step census refuses to gate the stage.
    EdgeNotGateable {
        /// The stage.
        key: StageKey,
        /// The core's own reason.
        reason: ReadPlanRefusal,
    },
}

impl AdmissionRefusalReason {
    /// A stable machine-readable token, one per cause, kept apart from
    /// [`fmt::Display`], which is prose and free to be reworded.
    #[must_use]
    pub fn code(&self) -> &'static str {
        match self {
            Self::NoCoverage { .. } => "read_log_no_coverage",
            Self::RecordDropped { .. } => "read_log_record_dropped",
            Self::StreamEndsEarly { .. } => "read_log_truncated",
            Self::UnenforceableRecord { .. } => "read_log_unenforceable_record",
            Self::InputNameUnresolved { .. } => "read_log_input_name_unresolved",
            Self::InputNameDuplicated { .. } => "read_log_input_name_duplicated",
            Self::EdgeNotGateable { .. } => "read_log_edge_not_gateable",
        }
    }

    /// The stage this refusal names, when one caused it.
    #[must_use]
    pub fn key(&self) -> Option<&StageKey> {
        match self {
            Self::NoCoverage { .. }
            | Self::InputNameUnresolved { .. }
            | Self::InputNameDuplicated { .. } => None,
            Self::RecordDropped { key, .. }
            | Self::StreamEndsEarly { key, .. }
            | Self::UnenforceableRecord { key, .. }
            | Self::EdgeNotGateable { key, .. } => Some(key),
        }
    }

    /// How the refusal NAMES its edge to an operator: the stage label where a
    /// stage caused it, and `node.input` on the two arms that fire before a
    /// stage is resolved.
    #[must_use]
    pub fn edge_label(&self) -> Option<String> {
        match self {
            Self::NoCoverage { .. } => None,
            Self::InputNameUnresolved { node, input, .. } => Some(format!("{node}.{input}")),
            Self::InputNameDuplicated { node, name, .. } => Some(format!("{node}.{name}")),
            other => other.key().map(StageKey::label),
        }
    }
}

impl fmt::Display for AdmissionRefusalReason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoCoverage { keys } => write!(
                f,
                "this rank's read log carries no record for any of the {} consuming stage(s) of \
                 a topic the rank both produces and consumes ({}), so the recorded reads of \
                 that edge are unknown and a replay would serve whatever its queue happened to \
                 hold. Re-record the bag with this binary",
                keys.len(),
                render_keys(keys)
            ),
            Self::RecordDropped {
                key,
                step,
                dropped_records,
            } => write!(
                f,
                "a stage-overflow marker at step {step} dropped {dropped_records} read \
                 record(s) on stage {}, so every later popped count on this stage is a floor \
                 and the replay cannot admit the recorded frames at their recorded steps. \
                 Re-record the bag with this binary",
                key.label()
            ),
            Self::StreamEndsEarly {
                key,
                last_read_step,
                last_boundary_step,
            } => write!(
                f,
                "stage {}'s read stream ends at step {last_read_step} while this rank's \
                 boundary stream runs to step {last_boundary_step}, so the steps in between \
                 name no recorded read and an armed gate would withhold every frame at them. \
                 Re-record the bag with this binary",
                key.label()
            ),
            Self::UnenforceableRecord { key, step, shape } => write!(
                f,
                "stage {}'s record at step {step} has a shape no admission can be built from \
                 ({}), so the frames it consumed cannot be placed on their recorded step. \
                 Re-record the bag with this binary",
                key.label(),
                shape.code()
            ),
            Self::InputNameUnresolved {
                node,
                input,
                detail,
            } => write!(
                f,
                "the gated input '{input}' of node '{node}' does not resolve to a wired read \
                 stage ({detail}), so its recorded reads cannot be addressed. Re-record the \
                 bag with this binary"
            ),
            Self::InputNameDuplicated {
                node,
                name,
                first_idx,
                second_idx,
            } => write!(
                f,
                "node '{node}' records the input name '{name}' twice (index {first_idx} and \
                 index {second_idx}), so every record naming it addresses two stages and the \
                 gate cannot tell which one read the frame. Re-record the bag with this binary"
            ),
            Self::EdgeNotGateable { key, reason } => write!(
                f,
                "stage {} cannot be gated: {reason}. Re-record the bag with a graph whose \
                 producer and consumer sit on different ranks, or replay it under \
                 CERULION_EXECUTION_MODE=lockstep",
                key.label()
            ),
        }
    }
}

/// Render a stage list for a refusal sentence.
fn render_keys(keys: &[StageKey]) -> String {
    if keys.is_empty() {
        return "<none wired>".to_string();
    }
    keys.iter()
        .map(StageKey::label)
        .collect::<Vec<_>>()
        .join(", ")
}

/// Which stage STAGED a record, from the site the wire carried and the stages
/// the input owns.
///
/// One rule, three inputs. An input that owns ONE stage stages every one of its
/// reads there, whatever the site says: under the unified discipline the
/// boundary drain and the node body both stage into the `Body` stage
/// (`cerulion_core::read_outcome::StagedReadOutcome`). An input that owns TWO
/// splits them, by the wire role where the bag stamps one and by the pre-roles
/// KIND rule where it does not, which is the same rule
/// [`crate::replay_engine`]'s steering demux applies to the same records: a
/// `DrainedBatch` is a drain-site record and every other kind is a body-site
/// one. A `Peek` steers to the drain stage, since its pop really did leave that
/// queue.
fn stage_of_record(role: ReadSiteRole, kind_is_batch: bool, roles: StageRoles) -> ReadStageRole {
    if !roles.drain {
        return ReadStageRole::Body;
    }
    if !roles.body {
        return ReadStageRole::Drain;
    }
    match role {
        ReadSiteRole::Drain | ReadSiteRole::Peek => ReadStageRole::Drain,
        ReadSiteRole::Body => ReadStageRole::Body,
        ReadSiteRole::Unstamped => {
            if kind_is_batch {
                ReadStageRole::Drain
            } else {
                ReadStageRole::Body
            }
        }
    }
}

/// Is this record's kind the batch drain the pre-roles site rule reads as a
/// drain-site record?
fn record_is_batch(body: &RecordedReadBody) -> bool {
    match body {
        RecordedReadBody::Read { kind, .. } => *kind == ReadKind::DrainedBatch,
        // A promotion is staged by the per-set Sync matcher, a drain site.
        RecordedReadBody::HandOff { .. } => true,
        RecordedReadBody::Producer { .. } | RecordedReadBody::Truncated { .. } => false,
    }
}

/// Plan the RECORDED ADMISSION of one graph-produced topic's consuming stages
/// on the rank being replayed.
///
/// [`plan_topic_injection`]'s twin at the other end of the same log: that one
/// places recorded FRAMES onto steps for a topic nothing local produces, this
/// one places recorded READS onto steps for a topic the pass produces itself.
/// It names no frame index because a produced edge has no feed to index into,
/// and it names no sequence range because the record carries none: one pop count
/// and one surviving sequence per read, which is exactly what the core's
/// `DueRead` admits.
///
/// Pure and total: every input shape yields either a plan or a typed refusal,
/// and nothing is logged.
#[must_use]
pub fn plan_edge_admission(topic: &TopicAdmission, stages: &StageTable) -> AdmissionPlan {
    let refused = |reason: AdmissionRefusalReason| {
        AdmissionPlan::Refused(AdmissionRefusal {
            topic: topic.topic.clone(),
            reason,
        })
    };
    let mut planned: Vec<StageAdmission> = Vec::new();
    let mut offered: Vec<StageKey> = Vec::new();
    let mut ungateable: Vec<(StageKey, String)> = Vec::new();
    for edge in &topic.edges {
        let (input_idx, roles) = match stages.resolve(&edge.node_id, &edge.input) {
            Ok(v) => v,
            Err(reason) => return refused(reason),
        };
        let key_of = |role: ReadStageRole| StageKey {
            node: edge.node_id.clone(),
            input_idx,
            role,
        };
        for (exists, role) in [
            (roles.body, ReadStageRole::Body),
            (roles.drain, ReadStageRole::Drain),
        ] {
            if !exists {
                continue;
            }
            if roles.gateable(role) {
                offered.push(key_of(role));
            } else {
                ungateable.push((
                    key_of(role),
                    stages.refusal(&key_of(role)).map_or_else(
                        || "the core refuses to gate this stage".to_string(),
                        |r| r.to_string(),
                    ),
                ));
            }
        }
        // Per stage, step to that step's admissions in RECORD ORDER. A
        // `BTreeMap` orders the STEPS and a `Vec` keeps the records inside one
        // step exactly as the bag holds them: sorting there would state an order
        // the recording does not.
        let mut body: BTreeMap<u64, Vec<DueRead>> = BTreeMap::new();
        let mut drain: BTreeMap<u64, Vec<DueRead>> = BTreeMap::new();
        for sited in &edge.reads {
            let step = sited.read.step;
            let role = stage_of_record(sited.role, record_is_batch(&sited.read.body), roles);
            // A stage the core refuses to gate takes today's drain, so its
            // records constrain nothing and are never READ here: the refusal
            // vocabulary below speaks only about stages the gate will drive, and
            // judging a shape on an ungated stage would refuse a run over an
            // edge this gate never touches.
            if !roles.gateable(role) {
                continue;
            }
            let key = key_of(role);
            let due = match &sited.read.body {
                // Records are missing here, each having consumed an unknown
                // number of frames, so every later popped count on this stage is
                // a floor.
                RecordedReadBody::Truncated { dropped_records } if *dropped_records > 0 => {
                    return refused(AdmissionRefusalReason::RecordDropped {
                        key,
                        step,
                        dropped_records: *dropped_records,
                    })
                }
                // A marker whose window dropped nothing removes no record.
                RecordedReadBody::Truncated { .. } => continue,
                // An annotation names the publisher of the next read and
                // occupies no position in the pop stream. A multi-publisher edge
                // is refused by the core census, which reads the wiring.
                RecordedReadBody::Producer { .. } => continue,
                // The third meaning: the frame named reached the consumer
                // without a dequeue, so the gate walks past this entry rather
                // than withholding at it.
                RecordedReadBody::HandOff { served_seq } => {
                    DueRead::served_without_pop(Some(*served_seq))
                }
                RecordedReadBody::Read {
                    kind,
                    served_seq,
                    popped,
                } => {
                    if *popped > 0 && served_seq.is_none() {
                        return refused(AdmissionRefusalReason::UnenforceableRecord {
                            key,
                            step,
                            shape: RecordShape::PoppedWithoutSequence,
                        });
                    }
                    if kind.serves() && *popped == 0 {
                        return refused(AdmissionRefusalReason::UnenforceableRecord {
                            key,
                            step,
                            shape: RecordShape::ServedNothingPopped,
                        });
                    }
                    match (*kind, *popped) {
                        // A recorded none, or a held read that took no new
                        // frame: the recording's drain ran at this position and
                        // consumed nothing, so the gate withholds at it.
                        (_, 0) => DueRead::nothing(),
                        // A sample gate dropped this read's frame in favour of
                        // an earlier one, so the served sequence names the last
                        // ACCEPTED frame and not what the read popped: the count
                        // is enforced and the identity is not.
                        (ReadKind::Decimated, n) => DueRead::decimated(n, *served_seq),
                        (_, n) => DueRead::pops(n, *served_seq),
                    }
                }
            };
            let per_step = if role == ReadStageRole::Drain {
                &mut drain
            } else {
                &mut body
            };
            per_step.entry(step).or_default().push(due);
        }
        for (role, per_step) in [(ReadStageRole::Body, body), (ReadStageRole::Drain, drain)] {
            if per_step.is_empty() {
                continue;
            }
            let key = key_of(role);
            let last_read_step = per_step.keys().next_back().copied().unwrap_or(0);
            if last_read_step < topic.last_boundary_step {
                return refused(AdmissionRefusalReason::StreamEndsEarly {
                    key,
                    last_read_step,
                    last_boundary_step: topic.last_boundary_step,
                });
            }
            planned.push(StageAdmission {
                key,
                mode: edge.mode,
                per_step: per_step
                    .into_iter()
                    .map(|(step, due)| StepAdmission { step, due })
                    .collect(),
            });
        }
    }
    if planned.is_empty() {
        if offered.is_empty() {
            return AdmissionPlan::NotGateable(ungateable);
        }
        return refused(AdmissionRefusalReason::NoCoverage { keys: offered });
    }
    AdmissionPlan::Enforced {
        stages: planned,
        ungateable,
    }
}

// ===========================================================================
// Oracle tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn edge_id(node: &str, input: &str) -> EdgeId {
        EdgeId::drain(node, input)
    }

    /// The SITE is a field and the `#body` suffix is a RENDERING — so the data
    /// never carries a port name no port has, which is what the `--report`
    /// JSON's `edge` must not hold.
    #[test]
    fn the_body_site_renders_a_suffix_it_does_not_store() {
        let drain = EdgeId::drain("planner", "cmd");
        let body = EdgeId::body("planner", "cmd");
        // Both name the REAL port.
        assert_eq!(drain.input, "cmd");
        assert_eq!(body.input, "cmd");
        // They are distinct identities.
        assert_ne!(drain, body);
        // And only the rendering carries the suffix.
        assert_eq!(drain.to_string(), "planner.cmd");
        assert_eq!(body.to_string(), "planner.cmd#body");
        // A port GENUINELY called `cmd#body` is a different edge from the body
        // queue of `cmd`, and the data says so even though they render alike.
        let literal = EdgeId::drain("planner", "cmd#body");
        assert_eq!(literal.to_string(), body.to_string());
        assert_ne!(literal, body);
        assert_eq!(literal.input, "cmd#body");
    }

    /// Every [`ProducerGap`]'s rendered sentence, pinned.
    ///
    /// These strings ARE the diagnosis an operator gets when a multi-publisher
    /// topic stands down — the planner is file-blind, so this text is the whole
    /// of what it can say — and NOT ONE of them was asserted anywhere. Two had
    /// been rewritten along the way (the `MissingAnnotation` hedge, and
    /// `UnattributedFrame`'s claim about what an undeclared tap does) with no
    /// test to hold either rewrite in place: reverting both to the earlier,
    /// FALSE claims left the whole suite green.
    ///
    /// Each row asserts its own wording AND that it does not carry a sibling's,
    /// because the failure that actually matters here is a read-side sentence
    /// rendered for a frame-side refusal (or the reverse) — which sends the
    /// operator to fix the wrong end of their own bag. `variant_name`'s
    /// exhaustive `match` makes a new variant a COMPILE error rather than a row
    /// nobody wrote.
    #[test]
    fn every_producer_gap_renders_its_own_diagnosis_and_no_siblings() {
        const fn variant_name(gap: &ProducerGap) -> &'static str {
            match gap {
                ProducerGap::MissingAnnotation => "MissingAnnotation",
                ProducerGap::AnnotationCollision => "AnnotationCollision",
                ProducerGap::AnnotationForeign => "AnnotationForeign",
                ProducerGap::UnattributedFrame { .. } => "UnattributedFrame",
                ProducerGap::FrameProducerUnresolved { .. } => "FrameProducerUnresolved",
                ProducerGap::Mismatch { .. } => "Mismatch",
            }
        }

        let rows: Vec<(ProducerGap, Vec<&str>, Vec<&str>)> = vec![
            (
                ProducerGap::MissingAnnotation,
                vec![
                    "the read carries no producer annotation",
                    // THE HEDGE. The earlier text asserted "this bag's recorder
                    // did not annotate reads on multi-publisher topics", which
                    // is one of two causes stated as if it were the only one —
                    // an annotation that simply did not reach the file reads
                    // identically here.
                    "this bag predates the read-time producer annotations",
                    "or the annotation did not reach the bag",
                    "re-record with a current build",
                ],
                // The READ side must never borrow the FRAME side's vocabulary.
                vec!["producer label", "hashes to more than one", "rank manifest"],
            ),
            (
                ProducerGap::AnnotationCollision,
                vec!["the read's producer token hashes to more than one publisher"],
                vec!["re-record", "producer label", "rank manifest"],
            ),
            (
                ProducerGap::AnnotationForeign,
                vec!["no rank manifest names the read's producer token"],
                vec!["re-record", "producer label", "hashes to more than one"],
            ),
            (
                ProducerGap::UnattributedFrame { frame_index: 7 },
                vec![
                    "recorded frame 7 carries no producer label",
                    "never armed labelling on this topic",
                    // THE CORRECTED CLAIM. `cerulion_bagd` builds a
                    // `ProducerLabeling` for EVERY tap: an undeclared one arms
                    // on OBSERVED plurality and labels from the frame a second
                    // writer first appears. The earlier parenthetical named "a
                    // tap it was handed no multi-publisher declaration for" as
                    // a cause of a wholly unlabelled topic, which is false —
                    // the cause is a run that only ever had one writer.
                    "only ever ONE writer was observed on it",
                    "an undeclared one from the frame a second writer first appears",
                    "The run's producer-labels note says which",
                ],
                vec![
                    // The FALSE parenthetical, spelled out so a revert fails
                    // here rather than shipping.
                    "handed no multi-publisher declaration for",
                    "the read carries no producer annotation",
                    "read's producer token",
                ],
            ),
            (
                ProducerGap::FrameProducerUnresolved { frame_index: 3 },
                vec!["recorded frame 3's producer is itself unresolved"],
                vec![
                    "carries no producer label",
                    "rank manifest",
                    "re-record",
                    "producer-labels note",
                ],
            ),
            (
                ProducerGap::Mismatch {
                    annotated: "ext/cam".to_string(),
                    frame: "ghost/cam".to_string(),
                },
                vec![
                    "the read names publisher 'ext/cam'",
                    "the frame is attributed to 'ghost/cam'",
                ],
                vec!["re-record", "rank manifest", "carries no producer label"],
            ),
        ];

        for (gap, wants, forbids) in &rows {
            let name = variant_name(gap);
            let rendered = gap.to_string();
            for want in wants {
                assert!(
                    rendered.contains(want),
                    "{name}: {want:?} missing from {rendered:?}"
                );
            }
            for forbid in forbids {
                assert!(
                    !rendered.contains(forbid),
                    "{name}: carries a sibling's wording {forbid:?}: {rendered:?}"
                );
            }
        }

        // No two variants render the same sentence — a `match` arm copied and
        // not edited would otherwise pass every row above.
        let mut rendered: Vec<String> = rows.iter().map(|(g, ..)| g.to_string()).collect();
        rendered.sort();
        let distinct = rendered.len();
        rendered.dedup();
        assert_eq!(rendered.len(), distinct, "two gaps render alike");
    }

    /// A read that SERVED the frame carrying `served`, having popped `popped`.
    fn served(step: u64, served: u32, popped: u32) -> RecordedRead {
        RecordedRead::read(step, ReadKind::Served, Some(served), popped, false)
    }

    fn batch(step: u64, served: u32, popped: u32) -> RecordedRead {
        RecordedRead::read(step, ReadKind::DrainedBatch, Some(served), popped, false)
    }

    fn decimated(step: u64, last_accepted: Option<u32>, popped: u32) -> RecordedRead {
        RecordedRead::read(step, ReadKind::Decimated, last_accepted, popped, false)
    }

    fn no_frame(step: u64) -> RecordedRead {
        RecordedRead::read(step, ReadKind::NoFrame, None, 0, false)
    }

    fn producer_note(step: u64, label: &str) -> RecordedRead {
        RecordedRead::producer(step, ResolvedProducer::Named(label.to_string()))
    }

    fn truncated(step: u64, dropped: u32) -> RecordedRead {
        RecordedRead::truncated(step, dropped)
    }

    /// Frames 0..n with sequences `0..n`, single publisher.
    fn frames(seqs: &[u32]) -> Vec<RecordedFrame> {
        seqs.iter()
            .map(|s| RecordedFrame {
                sequence: *s,
                producer: None,
            })
            .collect()
    }

    fn edge(id: EdgeId, mode: ConsumeMode, reads: Vec<RecordedRead>) -> ConsumerEdge {
        ConsumerEdge {
            id,
            mode,
            depth: None,
            reads,
        }
    }

    fn topic(multi: bool, frames: Vec<RecordedFrame>, edges: Vec<ConsumerEdge>) -> TopicReplay {
        TopicReplay {
            topic: "/t".to_string(),
            multi_publisher: multi,
            frames,
            edges,
        }
    }

    /// The slot-free planner: no producer on this topic runs on the replaying
    /// rank, which is every topic the shipped rank split injects.
    fn plan_no_local(topic: &TopicReplay) -> TopicPlan {
        plan_topic_injection(topic, &LocalProducers::none())
    }

    /// The schedule as `(step, [sequence, ..])` — the shape the oracles are
    /// hand-written in.
    fn shape(schedule: &InjectionSchedule) -> Vec<(u64, Vec<u32>)> {
        schedule
            .per_step
            .iter()
            .map(|s| (s.before_step, s.frames.iter().map(|f| f.sequence).collect()))
            .collect()
    }

    fn expect_steered(plan: TopicPlan) -> InjectionSchedule {
        match plan {
            TopicPlan::Steered(s) => s,
            TopicPlan::StandDown(sd) => panic!("expected a schedule, got: {}", sd.describe()),
        }
    }

    fn expect_stand_down(plan: TopicPlan) -> StandDown {
        match plan {
            TopicPlan::StandDown(sd) => sd,
            TopicPlan::Steered(s) => {
                panic!("expected a stand-down, got the schedule {:?}", shape(&s))
            }
        }
    }

    /// THE happy path: one `EachFifo` consumer popping one frame per step. The
    /// oracle is written by hand from the read log, never read back off the
    /// planner.
    #[test]
    fn a_single_each_fifo_edge_schedules_one_frame_per_recorded_read() {
        let t = topic(
            false,
            frames(&[10, 11, 12]),
            vec![edge(
                edge_id("fusion", "lidar"),
                ConsumeMode::EachFifo,
                vec![served(4, 10, 1), served(5, 11, 1), served(9, 12, 1)],
            )],
        );

        let s = expect_steered(plan_no_local(&t));

        assert_eq!(
            shape(&s),
            vec![(4, vec![10]), (5, vec![11]), (9, vec![12])],
            "each recorded read's one popped frame is due before its own step"
        );
        assert!(s.uncovered.is_empty());
        assert!(s.unconsumed.is_empty());
        // The frame INDEX is the identity the injector walks by; assert it
        // alongside the sequence so a planner that emitted the right sequences
        // at the wrong positions still fails.
        assert_eq!(
            s.per_step[2].frames,
            vec![DueFrame {
                frame_index: 2,
                sequence: 12
            }]
        );
    }

    /// `popped` is the quantity, not `served_seq`: a batch drain that served the
    /// newest of four consumed FOUR frames, and all four are due at its step.
    /// The oracle for pinned shape 1 in the module docs.
    #[test]
    fn a_drained_batch_makes_every_frame_it_popped_due_at_one_step() {
        let t = topic(
            false,
            frames(&[10, 11, 12, 13, 14]),
            vec![edge(
                edge_id("planner", "scan"),
                ConsumeMode::Latest,
                vec![served(2, 10, 1), batch(7, 14, 4)],
            )],
        );

        let s = expect_steered(plan_no_local(&t));

        assert_eq!(
            shape(&s),
            vec![(2, vec![10]), (7, vec![11, 12, 13, 14])],
            "the batch's four popped frames are all due before step 7, \
             not just the one it served"
        );
    }

    /// A drain-to-latest consumer that let three frames pile up and served the
    /// newest — the ordinary `Latest` shape, with a `NoFrame` step in the middle
    /// proving an empty read consumes nothing.
    #[test]
    fn a_latest_edge_drains_everything_pending_and_serves_the_newest() {
        let t = topic(
            false,
            frames(&[1, 2, 3, 4]),
            vec![edge(
                edge_id("viz", "state"),
                ConsumeMode::Latest,
                vec![served(1, 1, 1), no_frame(2), served(3, 4, 3)],
            )],
        );

        let s = expect_steered(plan_no_local(&t));

        assert_eq!(shape(&s), vec![(1, vec![1]), (3, vec![2, 3, 4])]);
    }

    /// Two consumers of one topic, one FIFO and one drain-to-latest, whose
    /// accounts agree once the frames are placed at the earliest step either of
    /// them consumed them.
    #[test]
    fn a_feasible_fan_out_places_each_frame_at_the_earliest_step_any_consumer_took_it() {
        let a = edge(
            edge_id("a", "in"),
            ConsumeMode::EachFifo,
            vec![served(3, 7, 1), served(4, 8, 1), served(5, 9, 1)],
        );
        // `b` lets all three pile up and drains them in one go at step 5.
        let b = edge(
            edge_id("b", "in"),
            ConsumeMode::Latest,
            vec![batch(5, 9, 3)],
        );
        let t = topic(false, frames(&[7, 8, 9]), vec![a, b]);

        let s = expect_steered(plan_no_local(&t));

        assert_eq!(
            shape(&s),
            vec![(3, vec![7]), (4, vec![8]), (5, vec![9])],
            "the FIFO consumer's earlier steps win; the Latest consumer still \
             finds all three pending at step 5"
        );
    }

    /// The refusal: a FIFO consumer pulls frame 2 forward to step 1, which puts
    /// TWO frames in the drain-to-latest consumer's queue at step 2 where the
    /// recording gave it one. Its drain would serve sequence 2 where the
    /// recording served 1. The oracle for pinned shape 2.
    #[test]
    fn an_infeasible_fan_out_stands_the_topic_down_naming_the_latest_edge() {
        let fast = edge(
            edge_id("fast", "in"),
            ConsumeMode::EachFifo,
            vec![served(1, 1, 1), served(2, 2, 1)],
        );
        let slow = edge(
            edge_id("slow", "in"),
            ConsumeMode::Latest,
            vec![served(2, 1, 1), served(3, 2, 1)],
        );
        let t = topic(false, frames(&[1, 2]), vec![fast, slow]);

        let sd = expect_stand_down(plan_no_local(&t));

        assert_eq!(sd.reason.code(), "fan_out_infeasible");
        assert_eq!(
            sd.reason,
            StandDownReason::FanOutInfeasible {
                mode: ConsumeMode::Latest,
                edge: edge_id("slow", "in"),
                step: 2,
                recorded_served: Some(1),
                would_serve: Some(2),
                pending: 2,
                recorded_popped: 1,
            },
            "the refusal names the edge, the step, and BOTH sequences"
        );
        assert!(
            sd.describe().contains("slow.in"),
            "the degrade sentence names the edge: {}",
            sd.describe()
        );
        assert!(
            sd.describe().contains("drain-to-latest"),
            "…and names the MODE whose rule it broke: {}",
            sd.describe()
        );
    }

    /// An EMPTY read on a FIFO edge is a claim about the
    /// queue's DEPTH, and the schedule has to honour it.
    ///
    /// `EachFifo`'s insensitivity argument is about frames BEHIND the head: a
    /// pop-one serves the same frame however deep the queue gets. A read that
    /// popped NOTHING has no head, so the argument does not reach it — and the
    /// arm's `debug_assert!(admitted >= read.end)` is VACUOUS over an empty
    /// range, so it cannot guard this case.
    ///
    /// The recording below is CONSISTENT, not corrupt. Every kind-6 record of a
    /// step carries ONE step number (`merge_read_outcomes` stamps the level-end
    /// merge), so a level-0 FIFO body read that found nothing at step 1 and a
    /// later-level consumer that took the frame later in that SAME step are
    /// both stamped 1 — the frame was published between them. Earliest-wins
    /// then places it before step 1, in front of the read that recorded
    /// silence, and the replayed pop-one serves it.
    ///
    /// Deleting the empty-read refusal makes this steer, with
    /// `shape == [(1, [7])]` — the frame injected in front of the read that
    /// recorded nothing.
    #[test]
    fn an_empty_fifo_read_the_schedule_would_fill_stands_the_topic_down() {
        // The FIFO edge: nothing at step 1, the frame at step 2.
        let fifo = edge(
            edge_id("fusion", "in"),
            ConsumeMode::EachFifo,
            vec![no_frame(1), served(2, 7, 1)],
        );
        // A later-level consumer that took the SAME frame inside step 1.
        let other = edge(
            edge_id("logger", "in"),
            ConsumeMode::Latest,
            vec![served(1, 7, 1)],
        );
        let t = topic(false, frames(&[7]), vec![fifo, other]);

        let sd = expect_stand_down(plan_no_local(&t));

        assert_eq!(sd.reason.code(), "fan_out_infeasible");
        assert_eq!(
            sd.reason,
            StandDownReason::FanOutInfeasible {
                mode: ConsumeMode::EachFifo,
                edge: edge_id("fusion", "in"),
                step: 1,
                // The recording served NOTHING at that read…
                recorded_served: None,
                // …and the replayed pop-one would serve the queue HEAD. Not
                // `admitted - 1`: a FIFO pop takes the OLDEST, which is what
                // separates this refusal's answer from the `Latest` arm's.
                would_serve: Some(7),
                pending: 1,
                recorded_popped: 0,
            },
            "the refusal names the FIFO edge, its empty read, and the frame it would pop"
        );
        assert!(
            sd.describe().contains("per-message FIFO"),
            "the sentence must not call a FIFO consumer drain-to-latest: {}",
            sd.describe()
        );
    }

    /// ANTI-TAUTOLOGY for the arm above, and the property the new refusal must
    /// NOT break: an empty FIFO read the schedule leaves EMPTY is still
    /// reproducible, and a NON-empty FIFO read is still insensitive to frames
    /// queued behind the one it pops.
    ///
    /// Without this, "an empty read stands the topic down" and "a FIFO edge
    /// with an empty read stands the topic down" are indistinguishable — and
    /// the second is a refusal that would fire on every quiet step of every
    /// data-trigger node in the system.
    #[test]
    fn an_empty_fifo_read_the_schedule_leaves_empty_is_still_reproducible() {
        // Nothing is on the wire at step 1: the sibling takes frame 7 at step 2,
        // so `earliest[7] = 2` and the step-1 read stays empty.
        let fifo = edge(
            edge_id("fusion", "in"),
            ConsumeMode::EachFifo,
            vec![no_frame(1), served(3, 7, 1)],
        );
        let other = edge(
            edge_id("logger", "in"),
            ConsumeMode::Latest,
            vec![served(2, 7, 1)],
        );
        let t = topic(false, frames(&[7]), vec![fifo, other]);
        let s = expect_steered(plan_no_local(&t));
        assert_eq!(
            shape(&s),
            vec![(2, vec![7])],
            "the frame is due at the earliest step ANY consumer took it, which is \
             after the empty read"
        );

        // And the insensitivity the mode really does have: a FIFO edge whose
        // read POPPED one still verifies with two frames pending, because the
        // extra frame sits BEHIND the one it takes.
        let fifo = edge(
            edge_id("fusion", "in"),
            ConsumeMode::EachFifo,
            vec![served(3, 7, 1), served(4, 8, 1)],
        );
        let other = edge(
            edge_id("logger", "in"),
            ConsumeMode::Latest,
            vec![served(2, 8, 2)],
        );
        let t = topic(false, frames(&[7, 8]), vec![fifo, other]);
        let s = expect_steered(plan_no_local(&t));
        assert_eq!(
            shape(&s),
            vec![(2, vec![7, 8])],
            "a FIFO edge still serves its head with a deeper queue behind it"
        );
    }

    /// An edge with no kind-6 records at all constrains nothing; the covered
    /// sibling still steers, and the uncovered edge is reported rather than
    /// implied to have been verified.
    #[test]
    fn an_uncovered_edge_constrains_nothing_and_is_reported_explicitly() {
        let quarantined = edge(edge_id("quarantined", "in"), ConsumeMode::Latest, vec![]);
        let live = edge(
            edge_id("live", "in"),
            ConsumeMode::EachFifo,
            vec![served(1, 5, 1), served(2, 6, 1)],
        );
        let t = topic(false, frames(&[5, 6]), vec![quarantined, live]);

        let s = expect_steered(plan_no_local(&t));

        assert_eq!(shape(&s), vec![(1, vec![5]), (2, vec![6])]);
        assert_eq!(s.uncovered, vec![edge_id("quarantined", "in")]);
    }

    /// With NO covered edge there is nothing to steer FROM, so the topic stands
    /// down — the caller falls back to the wall-time window for the whole topic.
    #[test]
    fn a_topic_whose_every_edge_is_uncovered_stands_down() {
        let t = topic(
            false,
            frames(&[1, 2]),
            vec![
                edge(edge_id("a", "in"), ConsumeMode::Latest, vec![]),
                // Annotations alone are NOT coverage: they describe reads, and
                // this stream describes none.
                edge(
                    edge_id("b", "in"),
                    ConsumeMode::Latest,
                    vec![producer_note(1, "p/out")],
                ),
            ],
        );

        let sd = expect_stand_down(plan_no_local(&t));

        assert_eq!(
            sd.reason,
            StandDownReason::NoCoverage {
                edges: vec![edge_id("a", "in"), edge_id("b", "in")],
            }
        );
    }

    /// The THIRD fold answer: an edge whose every record is a hand off is
    /// COVERED, not a coverage gap.
    ///
    /// A hand off names a frame an earlier record on the same edge already
    /// consumed, so such an edge plans no read AND leaves no read unexplained.
    /// Reporting it in `uncovered` would tell the caller the schedule was never
    /// checked against a read, when every record of it is accounted for.
    ///
    /// The two neighbouring answers are pinned by
    /// `an_uncovered_edge_constrains_nothing_and_is_reported_explicitly` (an
    /// empty stream) and `a_topic_whose_every_edge_is_uncovered_stands_down` (an
    /// annotation-only stream); both still hold, and the second arm here is what
    /// stops this answer from swallowing them.
    #[test]
    fn a_hand_off_only_edge_is_covered_and_not_reported_uncovered() {
        let hand_off_only = |node: &str| {
            edge(
                edge_id(node, "in"),
                ConsumeMode::Latest,
                vec![RecordedRead::read(
                    1,
                    ReadKind::DrainedBatch,
                    Some(5),
                    0,
                    true,
                )],
            )
        };
        let live = || {
            edge(
                edge_id("live", "in"),
                ConsumeMode::EachFifo,
                vec![served(1, 5, 1), served(2, 6, 1)],
            )
        };

        let s = expect_steered(plan_no_local(&topic(
            false,
            frames(&[5, 6]),
            vec![hand_off_only("promoted"), live()],
        )));
        assert_eq!(shape(&s), vec![(1, vec![5]), (2, vec![6])]);
        assert!(
            s.uncovered.is_empty(),
            "a hand off only edge is covered, nothing to do: {:?}",
            s.uncovered
        );

        // An ANNOTATION-only stream stays a gap: it names the producer of a read
        // it does not carry, so nothing on it is accounted for.
        let with_annotation = expect_steered(plan_no_local(&topic(
            false,
            frames(&[5, 6]),
            vec![
                hand_off_only("promoted"),
                edge(
                    edge_id("noted", "in"),
                    ConsumeMode::Latest,
                    vec![producer_note(1, "p/out")],
                ),
                live(),
            ],
        )));
        assert_eq!(with_annotation.uncovered, vec![edge_id("noted", "in")]);

        // With no edge planning a read there is nothing to steer from, so the
        // topic still stands down, and the covered edge is absent from the list
        // the stand down names.
        let sd = expect_stand_down(plan_no_local(&topic(
            false,
            frames(&[5, 6]),
            vec![hand_off_only("promoted")],
        )));
        assert_eq!(sd.reason, StandDownReason::NoCoverage { edges: vec![] });
        // The rendered sentence counts the edges carrying no read record, and
        // says nothing about what this edge's records are: the earlier wording
        // claimed no edge HAS a read log, which this topic contradicts.
        assert_eq!(
            sd.reason.to_string(),
            "no consuming edge on this rank plans a recorded read, \
             so there is nothing to steer from \
             (0 edge(s) carry no read record)"
        );
    }

    /// A `sample(N)` consumer's decimated reads pop frames the gate then drops.
    /// Those frames ENTERED the queue, so they must be injected — the schedule
    /// is keyed on `popped`, and a `Decimated` record's `served_seq` (the last
    /// ACCEPTED sequence, not a frame in this batch) is deliberately not joined.
    #[test]
    fn a_decimated_consumers_skipped_frames_are_still_scheduled() {
        // sample(3): accept 100, decimate 101 and 102, accept 103.
        let t = topic(
            false,
            frames(&[100, 101, 102, 103]),
            vec![edge(
                edge_id("gate", "raw"),
                ConsumeMode::Latest,
                vec![
                    served(1, 100, 1),
                    decimated(2, Some(100), 1),
                    decimated(3, Some(100), 1),
                    served(4, 103, 1),
                ],
            )],
        );

        let s = expect_steered(plan_no_local(&t));

        assert_eq!(
            shape(&s),
            vec![
                (1, vec![100]),
                (2, vec![101]),
                (3, vec![102]),
                (4, vec![103])
            ],
            "the two decimated frames are injected even though nothing served them"
        );
        assert!(s.unconsumed.is_empty());
    }

    /// On a multi-publisher topic the producer annotation joins the read to the
    /// frame; two publishers whose sequences OVERLAP are the shape that needs
    /// it (a sequence is per-publisher, so seq 0 appears twice).
    #[test]
    fn a_multi_publisher_edge_joins_through_the_resolved_producer_token() {
        let mp_frames = vec![
            RecordedFrame {
                sequence: 0,
                producer: Some(ResolvedProducer::Named("left/tf".into())),
            },
            RecordedFrame {
                sequence: 0,
                producer: Some(ResolvedProducer::Named("right/tf".into())),
            },
            RecordedFrame {
                sequence: 1,
                producer: Some(ResolvedProducer::Named("left/tf".into())),
            },
        ];
        let t = topic(
            true,
            mp_frames,
            vec![edge(
                edge_id("tf_sink", "tf"),
                ConsumeMode::EachFifo,
                vec![
                    producer_note(1, "left/tf"),
                    served(1, 0, 1),
                    producer_note(2, "right/tf"),
                    served(2, 0, 1),
                    producer_note(3, "left/tf"),
                    served(3, 1, 1),
                ],
            )],
        );

        let s = expect_steered(plan_no_local(&t));

        assert_eq!(shape(&s), vec![(1, vec![0]), (2, vec![0]), (3, vec![1])]);
        assert_eq!(
            s.per_step
                .iter()
                .map(|p| p.frames[0].frame_index)
                .collect::<Vec<_>>(),
            vec![0, 1, 2],
            "the two seq-0 frames are told apart by FILE POSITION, which is why \
             the schedule carries an index and not only a sequence"
        );
    }

    /// An unresolvable token on a multi-publisher edge is never guessed past.
    /// The oracle for pinned shape 3 — driven for each way the join can fail.
    #[test]
    fn an_unresolvable_producer_on_a_multi_publisher_edge_stands_the_topic_down() {
        let mp_frames = vec![
            RecordedFrame {
                sequence: 0,
                producer: Some(ResolvedProducer::Named("left/tf".into())),
            },
            RecordedFrame {
                sequence: 0,
                producer: Some(ResolvedProducer::Named("right/tf".into())),
            },
        ];
        let read_streams: Vec<(Vec<RecordedRead>, ProducerGap)> = vec![
            // No annotation at all.
            (vec![served(1, 0, 1)], ProducerGap::MissingAnnotation),
            // The token hashed to two publishers.
            (
                vec![
                    RecordedRead::producer(1, ResolvedProducer::Collision),
                    served(1, 0, 1),
                ],
                ProducerGap::AnnotationCollision,
            ),
            // No manifest names the token.
            (
                vec![
                    RecordedRead::producer(1, ResolvedProducer::Foreign),
                    served(1, 0, 1),
                ],
                ProducerGap::AnnotationForeign,
            ),
            // Both resolved, and they disagree: the popped sum lands on frame 0
            // (`left/tf`) while the read names `right/tf`.
            (
                vec![producer_note(1, "right/tf"), served(1, 0, 1)],
                ProducerGap::Mismatch {
                    annotated: "right/tf".into(),
                    frame: "left/tf".into(),
                },
            ),
        ];

        for (reads, want) in read_streams {
            let t = topic(
                true,
                mp_frames.clone(),
                vec![edge(edge_id("tf_sink", "tf"), ConsumeMode::EachFifo, reads)],
            );
            let sd = expect_stand_down(plan_no_local(&t));
            assert_eq!(
                sd.reason,
                StandDownReason::UnresolvedProducer {
                    edge: edge_id("tf_sink", "tf"),
                    step: 1,
                    detail: want.clone(),
                },
                "expected the {want} gap to stand the topic down"
            );
        }
    }

    /// The SAME stream on a topic that is NOT multi-publisher-listed plans
    /// fine — the anti-tautology half of the arm above: the stand-down is the
    /// multi-publisher rule firing, not a stream the planner cannot read.
    #[test]
    fn a_single_publisher_topic_needs_no_producer_annotation() {
        let t = topic(
            false,
            frames(&[0, 1]),
            vec![edge(
                edge_id("sink", "in"),
                ConsumeMode::EachFifo,
                vec![served(1, 0, 1), served(2, 1, 1)],
            )],
        );

        let s = expect_steered(plan_no_local(&t));
        assert_eq!(shape(&s), vec![(1, vec![0]), (2, vec![1])]);
    }

    /// A frame the bag holds but no covered edge ever accounted for is reported,
    /// never scheduled: injecting it would add queue occupancy the recording did
    /// not have.
    #[test]
    fn frames_no_edge_accounted_for_are_reported_and_not_scheduled() {
        let t = topic(
            false,
            frames(&[1, 2, 3]),
            vec![edge(
                edge_id("sink", "in"),
                ConsumeMode::EachFifo,
                vec![served(1, 1, 1)],
            )],
        );

        let s = expect_steered(plan_no_local(&t));

        assert_eq!(shape(&s), vec![(1, vec![1])]);
        assert_eq!(
            s.unconsumed,
            vec![
                DueFrame {
                    frame_index: 1,
                    sequence: 2
                },
                DueFrame {
                    frame_index: 2,
                    sequence: 3
                }
            ]
        );
    }

    /// An overflow marker means an unknown number of consumed frames went
    /// unrecorded, so the `popped` sum past it is a floor and the topic stands
    /// down rather than under-injecting silently.
    #[test]
    fn an_overflow_marker_stands_the_topic_down_naming_the_hole() {
        let t = topic(
            false,
            frames(&[1, 2, 3]),
            vec![edge(
                edge_id("burst", "in"),
                ConsumeMode::EachFifo,
                vec![served(1, 1, 1), truncated(2, 5), served(3, 3, 1)],
            )],
        );

        let sd = expect_stand_down(plan_no_local(&t));

        assert_eq!(
            sd.reason,
            StandDownReason::TruncatedReadLog {
                edge: edge_id("burst", "in"),
                step: 2,
                dropped_records: 5,
            }
        );
        assert_eq!(sd.reason.code(), "truncated_read_log");
    }

    /// [`RecordedRead::read`] classifies a HAND-OFF by its
    /// SHAPE — zero pops, a named frame, AND the `DrainedBatch` kind — and
    /// every other `hand_off` claim is an ordinary read.
    ///
    /// The kind conjunct is the one that matters most: with it gone, a corrupt
    /// zero-pop `Served` under a `Drain` role classifies as a hand-off and is
    /// SKIPPED by `fold_edge` instead of standing the topic down on
    /// `ServedNothingPopped`.
    #[test]
    fn a_hand_off_needs_zero_pops_a_named_frame_and_the_drained_batch_kind() {
        let is_hand_off = |r: &RecordedRead| matches!(r.body, RecordedReadBody::HandOff { .. });
        assert!(
            is_hand_off(&RecordedRead::read(
                1,
                ReadKind::DrainedBatch,
                Some(7),
                0,
                true
            )),
            "the promotion's own shape"
        );
        // Each conjunct removed, one at a time:
        assert!(
            !is_hand_off(&RecordedRead::read(1, ReadKind::Served, Some(7), 0, true)),
            "a zero-pop `Served` that names a frame is NOT a hand-off — it is the \
             corruption `ServedNothingPopped` refuses"
        );
        assert!(
            !is_hand_off(&RecordedRead::read(
                1,
                ReadKind::DrainedBatch,
                Some(7),
                1,
                true
            )),
            "a batch that popped is a read (the peek pops for real)"
        );
        assert!(
            !is_hand_off(&RecordedRead::read(
                1,
                ReadKind::DrainedBatch,
                None,
                0,
                true
            )),
            "a zero-pop batch naming nothing is a read"
        );
        assert!(
            !is_hand_off(&RecordedRead::read(
                1,
                ReadKind::DrainedBatch,
                Some(7),
                0,
                false
            )),
            "and without the caller's claim nothing is a hand-off"
        );
    }

    /// Through the planner — a genuine hand-off (peek then
    /// promotion) folds to ONE consumed frame, while a corrupt zero-pop
    /// `Served` at a hand-off claim stands the topic down.
    #[test]
    fn a_promotion_folds_once_and_a_corrupt_served_hand_off_stands_down() {
        // The promotion: a `Peek` popped seq 1 (recorded as a batch popping 1),
        // then the promotion re-names seq 1 with zero pops, then an ordinary
        // batch consumes seq 2.
        let promoted = topic(
            false,
            frames(&[1, 2]),
            vec![edge(
                edge_id("fusion", "a"),
                ConsumeMode::EachFifo,
                vec![
                    batch(1, 1, 1),
                    RecordedRead::read(1, ReadKind::DrainedBatch, Some(1), 0, true),
                    batch(2, 2, 1),
                ],
            )],
        );
        match plan_no_local(&promoted) {
            TopicPlan::Steered(s) => assert_eq!(
                s.per_step.iter().map(|p| p.frames.len()).sum::<usize>(),
                2,
                "seq 1 is consumed ONCE — the promotion re-names it, it does not pop it: {s:?}"
            ),
            TopicPlan::StandDown(sd) => panic!("a genuine promotion steers: {sd:?}"),
        }

        // The corruption: the SAME positions, but the zero-pop record is a
        // `Served`. `hand_off` is still claimed (the caller's role test passes
        // for any kind), and the ctor must refuse it into the read arm.
        let corrupt = topic(
            false,
            frames(&[1, 2]),
            vec![edge(
                edge_id("fusion", "a"),
                ConsumeMode::EachFifo,
                vec![
                    batch(1, 1, 1),
                    RecordedRead::read(1, ReadKind::Served, Some(1), 0, true),
                    batch(2, 2, 1),
                ],
            )],
        );
        let sd = expect_stand_down(plan_no_local(&corrupt));
        assert_eq!(
            sd.reason.code(),
            "served_nothing_popped",
            "a zero-pop Served is refused, never skipped as a hand-off: {sd:?}"
        );
    }

    /// A read log that consumes more frames than the bag holds is a
    /// conservation failure, named rather than clamped.
    #[test]
    fn a_read_log_that_outruns_the_recorded_frames_stands_down() {
        let t = topic(
            false,
            frames(&[1, 2]),
            vec![edge(
                edge_id("sink", "in"),
                ConsumeMode::Latest,
                vec![batch(1, 2, 2), served(2, 3, 1)],
            )],
        );

        let sd = expect_stand_down(plan_no_local(&t));

        assert_eq!(
            sd.reason,
            StandDownReason::FrameShortfall {
                edge: edge_id("sink", "in"),
                step: 2,
                needed: 3,
                recorded: 2,
            }
        );
    }

    /// The read log and the frame stream disagreeing about which frame a read
    /// served is a stand-down, not a schedule built on the disagreement.
    #[test]
    fn a_served_sequence_that_misses_its_own_popped_position_stands_down() {
        let t = topic(
            false,
            frames(&[10, 11, 12]),
            vec![edge(
                edge_id("sink", "in"),
                ConsumeMode::Latest,
                // Popping two lands on frame 11, but the record claims 12.
                vec![batch(1, 12, 2)],
            )],
        );

        let sd = expect_stand_down(plan_no_local(&t));

        assert_eq!(
            sd.reason,
            StandDownReason::JoinMismatch {
                edge: edge_id("sink", "in"),
                step: 1,
                recorded_served: Some(12),
                frame_at_position: Some(11),
            }
        );
    }

    /// A serving record that popped nothing has no position in the frame stream
    /// — unrepresentable, and no production drain site emits it.
    #[test]
    fn a_serving_record_that_popped_nothing_stands_down() {
        let t = topic(
            false,
            frames(&[10]),
            vec![edge(
                edge_id("sink", "in"),
                ConsumeMode::Latest,
                vec![served(1, 10, 0)],
            )],
        );

        let sd = expect_stand_down(plan_no_local(&t));
        assert_eq!(sd.reason.code(), "served_nothing_popped");
    }

    /// Earliest-wins deepens a queue, and a queue driven past its provisioned
    /// depth would evict under `drop_oldest`. Detected when the caller supplies
    /// the depth.
    #[test]
    fn a_schedule_that_over_deepens_a_known_queue_stands_down() {
        let fast = edge(
            edge_id("fast", "in"),
            ConsumeMode::EachFifo,
            vec![served(1, 1, 1), served(1, 2, 1), served(1, 3, 1)],
        );
        let mut shallow = edge(
            edge_id("shallow", "in"),
            ConsumeMode::EachFifo,
            vec![served(4, 1, 1), served(5, 2, 1), served(6, 3, 1)],
        );
        shallow.depth = Some(2);
        let t = topic(false, frames(&[1, 2, 3]), vec![fast, shallow]);

        let sd = expect_stand_down(plan_no_local(&t));

        assert_eq!(
            sd.reason,
            StandDownReason::QueueOverflow {
                edge: edge_id("shallow", "in"),
                step: 4,
                pending: 3,
                depth: 2,
            }
        );
    }

    /// The same fan-out with the depth ABSENT plans without complaint — the
    /// anti-tautology control proving the arm above is the depth check firing
    /// and not the fan-out rule.
    #[test]
    fn the_same_fan_out_plans_when_no_depth_is_supplied() {
        let fast = edge(
            edge_id("fast", "in"),
            ConsumeMode::EachFifo,
            vec![served(1, 1, 1), served(1, 2, 1), served(1, 3, 1)],
        );
        let slow = edge(
            edge_id("slow", "in"),
            ConsumeMode::EachFifo,
            vec![served(4, 1, 1), served(5, 2, 1), served(6, 3, 1)],
        );
        let t = topic(false, frames(&[1, 2, 3]), vec![fast, slow]);

        let s = expect_steered(plan_no_local(&t));
        assert_eq!(shape(&s), vec![(1, vec![1, 2, 3])]);
    }

    /// A producer annotation belongs to exactly the read that FOLLOWS it. A
    /// stream where an annotation is followed by a non-serving read must not
    /// leak that token onto the next serving read.
    #[test]
    fn a_producer_annotation_does_not_leak_past_the_read_it_annotates() {
        let mp_frames = vec![
            RecordedFrame {
                sequence: 0,
                producer: Some(ResolvedProducer::Named("left/tf".into())),
            },
            RecordedFrame {
                sequence: 0,
                producer: Some(ResolvedProducer::Named("right/tf".into())),
            },
        ];
        // The annotation belongs to the HELD read at step 1 (a held frame has a
        // producer too). The serving read at step 2 has none of its own, so the
        // join must fail rather than borrow the held read's.
        let t = topic(
            true,
            mp_frames,
            vec![edge(
                edge_id("tf_sink", "tf"),
                ConsumeMode::EachFifo,
                vec![
                    producer_note(1, "left/tf"),
                    RecordedRead::read(1, ReadKind::Held, Some(0), 0, false),
                    served(2, 0, 1),
                ],
            )],
        );

        let sd = expect_stand_down(plan_no_local(&t));
        assert_eq!(
            sd.reason,
            StandDownReason::UnresolvedProducer {
                edge: edge_id("tf_sink", "tf"),
                step: 2,
                detail: ProducerGap::MissingAnnotation,
            }
        );
    }

    /// Determinism: the same input plans the same schedule, and the frames
    /// within a step come out in FILE order regardless of which edge placed
    /// them.
    #[test]
    fn the_plan_is_deterministic_and_file_ordered_within_a_step() {
        let build = || {
            topic(
                false,
                frames(&[1, 2, 3, 4]),
                vec![
                    edge(
                        edge_id("z", "in"),
                        ConsumeMode::EachFifo,
                        vec![served(1, 1, 1), served(2, 2, 1), served(2, 3, 1)],
                    ),
                    edge(
                        edge_id("a", "in"),
                        ConsumeMode::Latest,
                        vec![batch(2, 3, 3), served(3, 4, 1)],
                    ),
                ],
            )
        };
        let first = expect_steered(plan_no_local(&build()));
        let second = expect_steered(plan_no_local(&build()));

        assert_eq!(first, second);
        assert_eq!(
            shape(&first),
            vec![(1, vec![1]), (2, vec![2, 3]), (3, vec![4])]
        );
    }
    // =======================================================================
    // The INTRA-STEP slot derivation
    // =======================================================================
    //
    // Every arm below is a CO-LOCATED topic: one producer on the replaying
    // rank, one on another. That shape needs producer LABELS to be told apart,
    // which only a `multi_publisher` topic carries, so each arm is built as
    // one — see `SlotInjection`'s "reachable only on a labelled topic".

    /// A recorded frame attributed to a named publisher.
    fn labelled(sequence: u32, label: &str) -> RecordedFrame {
        RecordedFrame {
            sequence,
            producer: Some(ResolvedProducer::Named(label.to_string())),
        }
    }

    /// `relay/out` runs on the replaying rank; everything else is foreign.
    fn relay_is_local() -> LocalProducers {
        LocalProducers::from_pairs([("relay/out".to_string(), "relay".to_string())])
    }

    /// The schedule's slots as `(step, node, after_fire, [sequence, ..])` — the
    /// shape the slot oracles are hand-written in.
    fn slot_shape(schedule: &InjectionSchedule) -> Vec<(u64, &str, u32, Vec<u32>)> {
        schedule
            .per_step
            .iter()
            .flat_map(|entry| {
                entry.slots.iter().map(move |slot| {
                    (
                        entry.before_step,
                        slot.producer_node.as_str(),
                        slot.after_fire,
                        slot.frames.iter().map(|f| f.sequence).collect(),
                    )
                })
            })
            .collect()
    }

    /// Frame indices the schedule declines to inject because a LOCAL producer
    /// publishes them.
    fn local_shape(schedule: &InjectionSchedule) -> Vec<(usize, u32)> {
        schedule
            .local_frames
            .iter()
            .map(|f| (f.frame_index, f.sequence))
            .collect()
    }

    /// One `EachFifo` read of one frame, with the producer annotation the
    /// multi-publisher join needs.
    fn joined(step: u64, label: &str, sequence: u32) -> Vec<RecordedRead> {
        vec![producer_note(step, label), served(step, sequence, 1)]
    }

    /// THE canonical shape (`[local fire 1, FOREIGN, local fire 2]`): the
    /// foreign frame the recording placed BETWEEN two local fires becomes a
    /// slot after fire 1 — not a before-step frame, which would put it in the
    /// consumer FIFO ahead of BOTH local publishes.
    ///
    /// The `after_fire` value is the whole assertion: it is 1-BASED and counts
    /// COMPLETED fires, so an off-by-one attributes the frame to the wrong side
    /// of the second fire and reproduces the very order the slot exists to fix.
    #[test]
    fn a_foreign_frame_between_two_local_fires_becomes_a_slot_after_the_first() {
        let t = topic(
            true,
            vec![
                labelled(100, "relay/out"),
                labelled(7, "peer/out"),
                labelled(101, "relay/out"),
            ],
            vec![edge(
                edge_id("sink", "in"),
                ConsumeMode::EachFifo,
                [
                    joined(5, "relay/out", 100),
                    joined(5, "peer/out", 7),
                    joined(5, "relay/out", 101),
                ]
                .concat(),
            )],
        );

        let s = expect_steered(plan_topic_injection(&t, &relay_is_local()));

        assert_eq!(
            slot_shape(&s),
            vec![(5, "relay", 1, vec![7])],
            "the foreign frame rides a pause after the relay's FIRST fire of step 5"
        );
        assert_eq!(
            shape(&s),
            vec![(5, Vec::new())],
            "and NOTHING is due before the step — a before-step frame would land \
             ahead of both local publishes"
        );
        assert_eq!(
            local_shape(&s),
            vec![(0, 100), (2, 101)],
            "both local frames are declined: the live producer publishes them"
        );
        assert!(s.unconsumed.is_empty(), "every frame is accounted for");
    }

    /// `[FOREIGN, local fire 1, FOREIGN]`: the frame BEFORE the step's first
    /// local fire folds into the before-step bucket exactly as it always did,
    /// and only the one after it becomes a slot.
    ///
    /// "Before any fire" is what the before-step bucket already means, so
    /// routing that first frame into a slot would both invent an `after_fire`
    /// of 0 (which the scheduler refuses at install) and delay a frame the
    /// recording had on the wire before the step began.
    #[test]
    fn a_foreign_frame_before_the_first_local_fire_stays_in_the_before_step_bucket() {
        let t = topic(
            true,
            vec![
                labelled(7, "peer/out"),
                labelled(100, "relay/out"),
                labelled(8, "peer/out"),
            ],
            vec![edge(
                edge_id("sink", "in"),
                ConsumeMode::EachFifo,
                [
                    joined(5, "peer/out", 7),
                    joined(5, "relay/out", 100),
                    joined(5, "peer/out", 8),
                ]
                .concat(),
            )],
        );

        let s = expect_steered(plan_topic_injection(&t, &relay_is_local()));

        assert_eq!(
            shape(&s),
            vec![(5, vec![7])],
            "the frame ahead of every local fire is due BEFORE the step"
        );
        assert_eq!(
            slot_shape(&s),
            vec![(5, "relay", 1, vec![8])],
            "and only the one after the first local fire is a slot"
        );
        assert_eq!(local_shape(&s), vec![(1, 100)]);
    }

    /// The BACK-COMPAT arm: with no local producer the plan is the slot-free one.
    ///
    /// Asserted against the SAME topic planned both ways, so it cannot pass by
    /// the two happening to agree on a shape neither produces — and the topic
    /// is one whose frames ARE labelled, which is the only way a planner that
    /// keyed on the labels rather than on the local SET could be caught.
    #[test]
    fn an_all_foreign_topic_plans_no_slots_and_matches_the_pre_c3_schedule() {
        let build = || {
            topic(
                true,
                vec![
                    labelled(100, "relay/out"),
                    labelled(7, "peer/out"),
                    labelled(101, "relay/out"),
                ],
                vec![edge(
                    edge_id("sink", "in"),
                    ConsumeMode::EachFifo,
                    [
                        joined(5, "relay/out", 100),
                        joined(5, "peer/out", 7),
                        joined(5, "relay/out", 101),
                    ]
                    .concat(),
                )],
            )
        };

        let s = expect_steered(plan_topic_injection(&build(), &LocalProducers::none()));

        assert_eq!(
            shape(&s),
            vec![(5, vec![100, 7, 101])],
            "every frame is foreign, so every frame is due before the step"
        );
        assert!(slot_shape(&s).is_empty(), "and nothing is placed inside it");
        assert!(s.local_frames.is_empty());
        // The same topic with `relay` local plans something DIFFERENT — without
        // this the arm above would pass on a planner that ignored `local`
        // entirely.
        let steered = expect_steered(plan_topic_injection(&build(), &relay_is_local()));
        assert_ne!(shape(&steered), shape(&s));
    }

    /// A topic ONLY this rank produces injects nothing at all: every frame is
    /// declined, and a step whose group holds only local frames is OMITTED
    /// rather than carried as an empty entry the caller's cursor would spend.
    #[test]
    fn an_all_local_topic_schedules_nothing_and_carries_no_empty_step_entries() {
        let t = topic(
            true,
            vec![labelled(100, "relay/out"), labelled(101, "relay/out")],
            vec![edge(
                edge_id("sink", "in"),
                ConsumeMode::EachFifo,
                [joined(5, "relay/out", 100), joined(6, "relay/out", 101)].concat(),
            )],
        );

        let s = expect_steered(plan_topic_injection(&t, &relay_is_local()));

        assert!(
            s.per_step.is_empty(),
            "no step has anything to inject: {:?}",
            shape(&s)
        );
        assert_eq!(local_shape(&s), vec![(0, 100), (1, 101)]);
        assert!(
            s.unconsumed.is_empty(),
            "a local frame is not a coverage hole"
        );
    }

    /// Two foreign frames under ONE anchor FOLD into ONE slot.
    ///
    /// `Scheduler::set_replay_intra_step_pauses` refuses the same
    /// `(node, after_fire)` twice — "fold two injections at ONE slot into ONE
    /// pause carrying both frames" — so a planner emitting two entries here
    /// would make every such step's install fail.
    #[test]
    fn consecutive_foreign_frames_under_one_anchor_fold_into_one_slot() {
        let t = topic(
            true,
            vec![
                labelled(100, "relay/out"),
                labelled(7, "peer/out"),
                labelled(8, "peer/out"),
                labelled(101, "relay/out"),
            ],
            vec![edge(
                edge_id("sink", "in"),
                ConsumeMode::EachFifo,
                [
                    joined(5, "relay/out", 100),
                    joined(5, "peer/out", 7),
                    joined(5, "peer/out", 8),
                    joined(5, "relay/out", 101),
                ]
                .concat(),
            )],
        );

        let s = expect_steered(plan_topic_injection(&t, &relay_is_local()));

        assert_eq!(
            slot_shape(&s),
            vec![(5, "relay", 1, vec![7, 8])],
            "ONE slot carrying both frames, in file order"
        );
    }

    /// Two DISTINCT slots in one step, one per local fire — and their
    /// `after_fire` values are 1 and 2, the node's own fire ordinals WITHIN the
    /// step.
    #[test]
    fn two_local_fires_in_one_step_carry_two_slots_at_their_own_ordinals() {
        let t = topic(
            true,
            vec![
                labelled(100, "relay/out"),
                labelled(7, "peer/out"),
                labelled(101, "relay/out"),
                labelled(8, "peer/out"),
            ],
            vec![edge(
                edge_id("sink", "in"),
                ConsumeMode::EachFifo,
                [
                    joined(5, "relay/out", 100),
                    joined(5, "peer/out", 7),
                    joined(5, "relay/out", 101),
                    joined(5, "peer/out", 8),
                ]
                .concat(),
            )],
        );

        let s = expect_steered(plan_topic_injection(&t, &relay_is_local()));

        assert_eq!(
            slot_shape(&s),
            vec![(5, "relay", 1, vec![7]), (5, "relay", 2, vec![8])],
        );
    }

    /// The fire ordinal is PER STEP, not per pass: the second step's slot is
    /// after fire 1 again, because the scheduler counts a burst's fires within
    /// the step it installs the pause for.
    #[test]
    fn the_fire_ordinal_restarts_every_step() {
        let t = topic(
            true,
            vec![
                labelled(100, "relay/out"),
                labelled(7, "peer/out"),
                labelled(101, "relay/out"),
                labelled(8, "peer/out"),
            ],
            vec![edge(
                edge_id("sink", "in"),
                ConsumeMode::EachFifo,
                [
                    joined(5, "relay/out", 100),
                    joined(5, "peer/out", 7),
                    joined(6, "relay/out", 101),
                    joined(6, "peer/out", 8),
                ]
                .concat(),
            )],
        );

        let s = expect_steered(plan_topic_injection(&t, &relay_is_local()));

        assert_eq!(
            slot_shape(&s),
            vec![(5, "relay", 1, vec![7]), (6, "relay", 1, vec![8])],
            "each step's burst is counted from 1"
        );
    }

    /// TWO local producers on one topic: a foreign frame anchors on the fire
    /// that most recently PRECEDED it, and each node's ordinal is its own.
    ///
    /// A single running count over all local frames would name `after_fire 2`
    /// on a node that has fired once, and the scheduler would pause at a fire
    /// the burst never reaches.
    #[test]
    fn a_foreign_frame_anchors_on_the_nearest_preceding_local_fire_per_node() {
        let local = LocalProducers::from_pairs([
            ("relay/out".to_string(), "relay".to_string()),
            ("mux/out".to_string(), "mux".to_string()),
        ]);
        let t = topic(
            true,
            vec![
                labelled(100, "relay/out"),
                labelled(200, "mux/out"),
                labelled(7, "peer/out"),
                labelled(101, "relay/out"),
                labelled(8, "peer/out"),
            ],
            vec![edge(
                edge_id("sink", "in"),
                ConsumeMode::EachFifo,
                [
                    joined(5, "relay/out", 100),
                    joined(5, "mux/out", 200),
                    joined(5, "peer/out", 7),
                    joined(5, "relay/out", 101),
                    joined(5, "peer/out", 8),
                ]
                .concat(),
            )],
        );

        let s = expect_steered(plan_topic_injection(&t, &local));

        assert_eq!(
            slot_shape(&s),
            vec![(5, "mux", 1, vec![7]), (5, "relay", 2, vec![8])],
            "seq 7 follows the MUX's first fire; seq 8 follows the RELAY's SECOND"
        );
    }

    /// Fan-out keeps its earliest-wins rule and the slots derive per topic on
    /// top of it: a frame two consumers account for at different steps is
    /// placed at the EARLIER one, and its anchor is read off THAT step's group.
    #[test]
    fn earliest_wins_decides_the_step_and_the_slot_derives_inside_it() {
        let t = topic(
            true,
            vec![labelled(100, "relay/out"), labelled(7, "peer/out")],
            vec![
                edge(
                    edge_id("late", "in"),
                    ConsumeMode::EachFifo,
                    [joined(9, "relay/out", 100), joined(9, "peer/out", 7)].concat(),
                ),
                edge(
                    edge_id("early", "in"),
                    ConsumeMode::EachFifo,
                    [joined(5, "relay/out", 100), joined(5, "peer/out", 7)].concat(),
                ),
            ],
        );

        let s = expect_steered(plan_topic_injection(&t, &relay_is_local()));

        assert_eq!(
            slot_shape(&s),
            vec![(5, "relay", 1, vec![7])],
            "the EARLIER consumer decides the step, and the anchor comes from it"
        );
        assert!(s.per_step.iter().all(|e| e.before_step == 5));
    }

    /// A LOCAL frame no covered edge accounts for is not a coverage hole: it is
    /// produced live either way, so it lands in `local_frames` and NOT in
    /// `unconsumed` — where it would report a phantom hole on every co-located
    /// topic, sized to that topic's own live output.
    #[test]
    fn an_unaccounted_local_frame_is_declined_not_reported_as_a_coverage_hole() {
        let t = topic(
            true,
            vec![
                labelled(100, "relay/out"),
                labelled(7, "peer/out"),
                labelled(101, "relay/out"),
            ],
            vec![edge(
                edge_id("sink", "in"),
                ConsumeMode::EachFifo,
                [joined(5, "relay/out", 100), joined(5, "peer/out", 7)].concat(),
            )],
        );

        let s = expect_steered(plan_topic_injection(&t, &relay_is_local()));

        assert_eq!(local_shape(&s), vec![(0, 100), (2, 101)]);
        assert!(
            s.unconsumed.is_empty(),
            "the trailing LOCAL frame is declined, not an injection gap: {:?}",
            s.unconsumed
        );
        assert_eq!(slot_shape(&s), vec![(5, "relay", 1, vec![7])]);
    }

    /// A SUMMED occupancy over the
    /// depth must NOT stand a topic down when every real CONNECTION is
    /// within it.
    ///
    /// Summing the injected frame and the local producer's own
    /// publishes into ONE pool is not how iceoryx2
    /// enforces `drop_oldest`: capacity is PER PUBLISHER CONNECTION, so
    /// `relay`'s own two live publishes (its OWN connection, depth-2, exactly
    /// AT the ceiling) and `peer`'s one injected frame (a SEPARATE
    /// connection, well under) never evict each other, however deep the
    /// SUM reads. A summing rule stands this exact topic down at depth 2 —
    /// summed occupancy 3 > 2 — even though NEITHER connection individually
    /// overflows; this is the convicting arm (a summed-occupancy
    /// revert fails it, by flipping `expect_steered` back to a stand-down).
    #[test]
    fn a_summed_occupancy_over_depth_still_steers_when_every_connection_fits() {
        let build = |depth: u32| {
            topic(
                true,
                vec![
                    labelled(100, "relay/out"),
                    labelled(101, "relay/out"),
                    labelled(7, "peer/out"),
                ],
                vec![ConsumerEdge {
                    id: edge_id("sink", "in"),
                    mode: ConsumeMode::Latest,
                    depth: Some(depth),
                    reads: vec![producer_note(5, "peer/out"), batch(5, 7, 3)],
                }],
            )
        };

        // relay's own connection = 2 (AT the depth), peer's = 1 — summed is 3
        // (over a depth-2 edge), but NEITHER connection alone overflows.
        let s = expect_steered(plan_topic_injection(&build(2), &relay_is_local()));
        assert_eq!(
            slot_shape(&s),
            vec![(5, "relay", 2, vec![7])],
            "steered, not stood down: the summed occupancy (3) exceeds depth \
             (2), but relay's own connection (2) and peer's (1) each fit"
        );

        // One shallower still steers relay's own connection down to the
        // boundary (1 < 2 <= depth), the anti-vacuity control that this is a
        // real per-connection computation and not a constant-true stub.
        let s = expect_steered(plan_topic_injection(&build(3), &relay_is_local()));
        assert_eq!(slot_shape(&s), vec![(5, "relay", 2, vec![7])]);
    }

    /// The positive complement: a genuine SINGLE-connection overflow — one
    /// local producer's OWN backlog alone exceeds the depth — still stands
    /// the topic down. The CONDITION is narrow (max connection, not the
    /// sum); the check is not disabled.
    #[test]
    fn a_single_connections_own_backlog_over_depth_still_stands_the_topic_down() {
        let t = topic(
            true,
            vec![
                labelled(100, "relay/out"),
                labelled(101, "relay/out"),
                labelled(102, "relay/out"),
                labelled(7, "peer/out"),
            ],
            vec![ConsumerEdge {
                id: edge_id("sink", "in"),
                mode: ConsumeMode::Latest,
                depth: Some(2),
                reads: vec![producer_note(5, "peer/out"), batch(5, 7, 4)],
            }],
        );

        let sd = expect_stand_down(plan_topic_injection(&t, &relay_is_local()));
        assert_eq!(
            sd.reason,
            StandDownReason::QueueOverflow {
                edge: edge_id("sink", "in"),
                step: 5,
                pending: 3,
                depth: 2,
            },
            "relay's OWN connection alone holds 3 pending frames against a \
             depth-2 edge — peer's single injected frame is irrelevant to \
             THIS connection's overflow"
        );
    }

    /// Determinism: the same co-located input plans the same slots, frames and
    /// declines twice over (Principle #7).
    #[test]
    fn the_slot_plan_is_deterministic() {
        let build = || {
            topic(
                true,
                vec![
                    labelled(100, "relay/out"),
                    labelled(7, "peer/out"),
                    labelled(101, "relay/out"),
                    labelled(8, "peer/out"),
                ],
                vec![edge(
                    edge_id("sink", "in"),
                    ConsumeMode::EachFifo,
                    [
                        joined(5, "relay/out", 100),
                        joined(5, "peer/out", 7),
                        joined(6, "relay/out", 101),
                        joined(6, "peer/out", 8),
                    ]
                    .concat(),
                )],
            )
        };

        let first = expect_steered(plan_topic_injection(&build(), &relay_is_local()));
        let second = expect_steered(plan_topic_injection(&build(), &relay_is_local()));
        assert_eq!(first, second);
    }

    /// A frame the planner cannot ATTRIBUTE is never claimed as local.
    ///
    /// Claiming one would drop a foreign frame off the wire entirely — the
    /// planner would assume some local node publishes it — which is the
    /// opposite of every other refusal in this module.
    #[test]
    fn only_a_resolved_named_producer_can_be_local() {
        let local = relay_is_local();
        assert_eq!(
            local.node_for(Some(&ResolvedProducer::Named("relay/out".into()))),
            Some("relay")
        );
        assert_eq!(
            local.node_for(Some(&ResolvedProducer::Named("peer/out".into()))),
            None,
            "a resolved producer that is not on this rank is foreign"
        );
        assert_eq!(local.node_for(None), None, "an unlabelled frame is foreign");
        assert_eq!(
            local.node_for(Some(&ResolvedProducer::Collision)),
            None,
            "a colliding token names no single publisher, so it names no local one"
        );
        assert_eq!(
            local.node_for(Some(&ResolvedProducer::Foreign)),
            None,
            "and an unresolved token is foreign by name as well as by rule"
        );
        assert!(LocalProducers::none().is_empty());
        assert_eq!(local.len(), 1);
    }

    // =======================================================================
    // The ENFORCEMENT planner
    // =======================================================================

    /// A census over one input's stages, all gateable.
    fn census(node: &str, idx: u16, roles: &[ReadStageRole]) -> Vec<ReadEdgeCapability> {
        roles
            .iter()
            .map(|role| ReadEdgeCapability {
                key: StageKey {
                    node: node.to_string(),
                    input_idx: idx,
                    role: *role,
                },
                enforceable: true,
                reason: None,
            })
            .collect()
    }

    fn inputs(node: &str, names: &[&str]) -> BTreeMap<String, Vec<String>> {
        BTreeMap::from([(
            node.to_string(),
            names.iter().map(|n| (*n).to_string()).collect(),
        )])
    }

    fn sited(
        step: u64,
        role: ReadSiteRole,
        kind: ReadKind,
        seq: Option<u32>,
        popped: u32,
    ) -> SitedRead {
        SitedRead {
            role,
            read: RecordedRead::read(step, kind, seq, popped, false),
        }
    }

    fn one_edge(reads: Vec<SitedRead>, last_boundary_step: u64) -> TopicAdmission {
        TopicAdmission {
            topic: "/t".to_string(),
            multi_publisher: false,
            edges: vec![AdmissionEdge {
                node_id: "relay".to_string(),
                input: "inp".to_string(),
                mode: ConsumeMode::Latest,
                reads,
            }],
            last_boundary_step,
        }
    }

    /// The `(pops, served_seq)` derivation, and the three pop-less meanings, over
    /// ONE recorded stream with a hand-written expectation per step.
    #[test]
    fn a_recorded_stream_folds_to_one_admission_per_record_in_record_order() {
        let stages = StageTable::new(
            &inputs("relay", &["inp"]),
            &census("relay", 0, &[ReadStageRole::Body]),
        );
        let topic = one_edge(
            vec![
                // step 0: a batch of 2, the newest carrying seq 5.
                sited(0, ReadSiteRole::Body, ReadKind::DrainedBatch, Some(5), 2),
                // step 1: the recorded none, then a held read that took nothing:
                // two POSITIONS the gate withholds at, not one.
                sited(1, ReadSiteRole::Body, ReadKind::NoFrame, None, 0),
                sited(1, ReadSiteRole::Body, ReadKind::Held, Some(5), 0),
                // step 2: a hand off, which SERVED seq 6 without a dequeue.
                SitedRead {
                    role: ReadSiteRole::Drain,
                    read: RecordedRead::read(2, ReadKind::DrainedBatch, Some(6), 0, true),
                },
                // step 2 again: a decimated read, whose count is enforced and
                // whose identity is not.
                sited(2, ReadSiteRole::Body, ReadKind::Decimated, Some(6), 3),
            ],
            2,
        );
        let AdmissionPlan::Enforced {
            stages: plans,
            ungateable,
        } = plan_edge_admission(&topic, &stages)
        else {
            panic!("the stream plans admissions");
        };
        assert!(ungateable.is_empty());
        assert_eq!(plans.len(), 1, "one input, one stage: {plans:?}");
        assert_eq!(plans[0].key.role, ReadStageRole::Body);
        let per_step: Vec<(u64, Vec<DueRead>)> = plans[0]
            .per_step
            .iter()
            .map(|s| (s.step, s.due.clone()))
            .collect();
        assert_eq!(
            per_step,
            vec![
                (0, vec![DueRead::pops(2, Some(5))]),
                (1, vec![DueRead::nothing(), DueRead::nothing()]),
                (
                    2,
                    vec![
                        DueRead::served_without_pop(Some(6)),
                        DueRead::decimated(3, Some(6)),
                    ]
                ),
            ],
            "one admission per record, steps ascending, records in RECORD order"
        );
    }

    /// A DRAIN-site record folds into the BODY stage when that is the only stage
    /// the input owns, and splits by site when it owns both.
    ///
    /// Under the unified discipline both read paths stage into one stage, so a
    /// plan keyed on the record's SITE would address a stage that does not exist
    /// and every boundary drain would go ungated.
    #[test]
    fn a_drain_site_read_folds_into_the_body_stage_under_the_unified_discipline() {
        let unified = StageTable::new(
            &inputs("relay", &["inp"]),
            &census("relay", 0, &[ReadStageRole::Body]),
        );
        let reads = vec![
            sited(0, ReadSiteRole::Drain, ReadKind::DrainedBatch, Some(0), 1),
            sited(0, ReadSiteRole::Body, ReadKind::Served, Some(0), 1),
        ];
        let AdmissionPlan::Enforced { stages: plans, .. } =
            plan_edge_admission(&one_edge(reads.clone(), 0), &unified)
        else {
            panic!("the unified input plans admissions");
        };
        assert_eq!(plans.len(), 1);
        assert_eq!(plans[0].key.role, ReadStageRole::Body);
        assert_eq!(
            plans[0].per_step[0].due.len(),
            2,
            "BOTH records land on the one stage that staged them: {:?}",
            plans[0].per_step
        );

        let split = StageTable::new(
            &inputs("relay", &["inp"]),
            &census("relay", 0, &[ReadStageRole::Body, ReadStageRole::Drain]),
        );
        let AdmissionPlan::Enforced { stages: plans, .. } =
            plan_edge_admission(&one_edge(reads, 0), &split)
        else {
            panic!("the split input plans admissions");
        };
        let mut roles: Vec<ReadStageRole> = plans.iter().map(|p| p.key.role).collect();
        roles.sort_by_key(|r| r.wire());
        assert_eq!(
            roles,
            vec![ReadStageRole::Body, ReadStageRole::Drain],
            "an input owning two stages splits its stream by site"
        );
        for plan in &plans {
            assert_eq!(plan.per_step[0].due.len(), 1, "one record each: {plan:?}");
        }
    }

    /// An UNSTAMPED record is placed by the pre-roles KIND rule, not refused.
    ///
    /// The steering demux reads a `DrainedBatch` as a drain-site record and every
    /// other kind as a body-site one on every `trace_format` <= 4 bag, and the
    /// gate reads the same rule, so an archived bag's records are placed rather
    /// than making the whole recording unenforceable.
    #[test]
    fn an_unstamped_record_is_placed_by_the_pre_roles_kind_rule() {
        let split = StageTable::new(
            &inputs("relay", &["inp"]),
            &census("relay", 0, &[ReadStageRole::Body, ReadStageRole::Drain]),
        );
        let topic = one_edge(
            vec![
                sited(
                    0,
                    ReadSiteRole::Unstamped,
                    ReadKind::DrainedBatch,
                    Some(0),
                    1,
                ),
                sited(0, ReadSiteRole::Unstamped, ReadKind::Served, Some(1), 1),
            ],
            0,
        );
        let AdmissionPlan::Enforced { stages: plans, .. } = plan_edge_admission(&topic, &split)
        else {
            panic!("an unstamped stream is placeable");
        };
        let drain = plans
            .iter()
            .find(|p| p.key.role == ReadStageRole::Drain)
            .expect("the batch went to the drain stage");
        assert_eq!(drain.per_step[0].due, vec![DueRead::pops(1, Some(0))]);
        let body = plans
            .iter()
            .find(|p| p.key.role == ReadStageRole::Body)
            .expect("the served read went to the body stage");
        assert_eq!(body.per_step[0].due, vec![DueRead::pops(1, Some(1))]);
    }

    /// Every refusal names its cause and, where a stage caused it, that stage.
    #[test]
    fn every_admission_refusal_names_its_cause_and_its_edge() {
        let stages = StageTable::new(
            &inputs("relay", &["inp"]),
            &census("relay", 0, &[ReadStageRole::Body]),
        );
        // No record plans a read: an annotation-only stream.
        let uncovered = one_edge(
            vec![SitedRead {
                role: ReadSiteRole::Body,
                read: RecordedRead::producer(0, ResolvedProducer::Foreign),
            }],
            0,
        );
        let AdmissionPlan::Refused(r) = plan_edge_admission(&uncovered, &stages) else {
            panic!("an annotation-only stream covers nothing");
        };
        assert_eq!(r.reason.code(), "read_log_no_coverage");
        assert!(r.reason.key().is_none() && r.reason.edge_label().is_none());
        assert!(
            r.reason.to_string().contains("relay[0]/body"),
            "the whole-topic arm names every stage it offered: {}",
            r.reason
        );

        // A marker that dropped records.
        let dropped = one_edge(
            vec![SitedRead {
                role: ReadSiteRole::Body,
                read: RecordedRead::truncated(4, 3),
            }],
            4,
        );
        let AdmissionPlan::Refused(r) = plan_edge_admission(&dropped, &stages) else {
            panic!("a marker that dropped records is refused");
        };
        assert_eq!(r.reason.code(), "read_log_record_dropped");
        assert_eq!(r.reason.edge_label().as_deref(), Some("relay[0]/body"));

        // A stream that stops below the rank's last boundary.
        let short = one_edge(
            vec![sited(0, ReadSiteRole::Body, ReadKind::Served, Some(0), 1)],
            7,
        );
        let AdmissionPlan::Refused(r) = plan_edge_admission(&short, &stages) else {
            panic!("a stream that ends early is refused");
        };
        assert_eq!(r.reason.code(), "read_log_truncated");
        assert!(r.reason.to_string().contains("step 7"), "{}", r.reason);

        // A record that consumed frames while naming none.
        let shapeless = one_edge(
            vec![sited(
                0,
                ReadSiteRole::Body,
                ReadKind::DrainedBatch,
                None,
                2,
            )],
            0,
        );
        let AdmissionPlan::Refused(r) = plan_edge_admission(&shapeless, &stages) else {
            panic!("a popped record that names no sequence is refused");
        };
        assert_eq!(r.reason.code(), "read_log_unenforceable_record");
        assert!(
            r.reason.to_string().contains("popped_without_sequence"),
            "{}",
            r.reason
        );

        // A serving read that popped nothing and is not the hand-off shape.
        let served_nothing = one_edge(
            vec![sited(0, ReadSiteRole::Body, ReadKind::Served, Some(3), 0)],
            0,
        );
        let AdmissionPlan::Refused(r) = plan_edge_admission(&served_nothing, &stages) else {
            panic!("a zero-pop Served record is refused");
        };
        assert_eq!(r.reason.code(), "read_log_unenforceable_record");
        assert!(
            r.reason.to_string().contains("served_nothing_popped"),
            "{}",
            r.reason
        );

        // An input name the recorded table repeats.
        let ambiguous = StageTable::new(
            &inputs("relay", &["inp", "inp"]),
            &census("relay", 0, &[ReadStageRole::Body]),
        );
        let AdmissionPlan::Refused(r) = plan_edge_admission(
            &one_edge(
                vec![sited(0, ReadSiteRole::Body, ReadKind::Served, Some(0), 1)],
                0,
            ),
            &ambiguous,
        ) else {
            panic!("a repeated input name is refused");
        };
        assert_eq!(r.reason.code(), "read_log_input_name_duplicated");
        assert_eq!(r.reason.edge_label().as_deref(), Some("relay.inp"));
        let sentence = r.reason.to_string();
        assert!(
            sentence.contains("index 0") && sentence.contains("index 1"),
            "the sentence names BOTH colliding indices: {sentence}"
        );

        // An input name the recorded table does not carry.
        let absent = StageTable::new(
            &inputs("relay", &["other"]),
            &census("relay", 0, &[ReadStageRole::Body]),
        );
        let AdmissionPlan::Refused(r) = plan_edge_admission(
            &one_edge(
                vec![sited(0, ReadSiteRole::Body, ReadKind::Served, Some(0), 1)],
                0,
            ),
            &absent,
        ) else {
            panic!("an unresolvable input name is refused");
        };
        assert_eq!(r.reason.code(), "read_log_input_name_unresolved");
    }

    /// A stage the CORE refuses to gate is left ungated and NOT refused.
    ///
    /// A per-set Sync trigger input pops outside the two gated bodies and a
    /// multi-publisher edge's wire sequence names no single head, so neither can
    /// be gated. Those stages take today's drain, exactly as every edge outside a
    /// gated rank does, and the answer names them so the report never implies
    /// coverage the run did not have.
    #[test]
    fn a_stage_the_core_refuses_to_gate_is_ungated_and_never_a_refusal() {
        let key = StageKey {
            node: "relay".to_string(),
            input_idx: 0,
            role: ReadStageRole::Body,
        };
        let stages = StageTable::new(
            &inputs("relay", &["inp"]),
            &[ReadEdgeCapability {
                key: key.clone(),
                enforceable: false,
                reason: Some(ReadPlanRefusal::PerSetSyncEdge(key.clone())),
            }],
        );
        // A stream that would otherwise be REFUSED for its shape: the refusal
        // vocabulary speaks only about stages the gate will drive.
        let topic = one_edge(
            vec![sited(0, ReadSiteRole::Body, ReadKind::Served, Some(3), 0)],
            9,
        );
        let AdmissionPlan::NotGateable(ungateable) = plan_edge_admission(&topic, &stages) else {
            panic!("no stage of this topic can be gated");
        };
        assert_eq!(ungateable.len(), 1);
        assert_eq!(ungateable[0].0, key);
        assert!(
            ungateable[0].1.contains("per_set_sync_edge"),
            "the core's own reason is carried: {}",
            ungateable[0].1
        );
    }
}
