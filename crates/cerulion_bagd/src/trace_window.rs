// SPDX-License-Identifier: AGPL-3.0-only
//! The rolling SCHEDULER-TRACE retention: what makes a Flashback capture
//! RESUMABLE rather than merely readable.
//!
//! # Why this exists
//!
//! `cerulion bag play --resim` is DRIVEN by recorded step boundaries: no
//! boundaries, zero steps execute, and a trace-empty bag is refused before any
//! resume logic runs. Boundaries live ONLY in the SHM trace ring — the
//! scheduler's own comment is "Boundaries are RING-ONLY: they never enter the
//! in-memory `TraceEntry` trace" — so a capture that carries frames and a
//! checkpoint and no trace is a bag you can look at and cannot resume.
//!
//! The anchor half of that problem is [`crate::anchor_window`]. This is the
//! other half, and the two are deliberately the same shape: a rolling
//! drop-oldest retention with a SPAN promise and a BYTE backstop, fed from the
//! ring's one SPSC consumer, drained into a capture at finalize.
//!
//! # It rides THIS RECORDER's one drain — a per-ARTIFACT rule, not a ban on a
//! second reader of the ring
//!
//! Be precise about what the ring permits and what this recorder must not do,
//! because the two used to be stated as one thing and they are not.
//!
//! What the RING permits: a trace ring is `OverrunPolicy::FailLoud`, where each
//! consumer maps its own view and holds its own LOCAL read cursor and nothing is
//! published into the shared header word (`cerulion_core::shm_ring` module doc
//! contract 7). So a `FailLoud` ring is ONE PRODUCER, N INDEPENDENT READERS:
//! two consumers do not steal each other's records, and each sees the FULL
//! stream from wherever it opened. The mid-run attach depends on exactly that — a run's
//! standing window recorder drains its trace ring while a mid-run
//! `cerulion bag record --run` attaches to the SAME ring, neither a party to the
//! other.
//!
//! What THIS RECORDER must not do: read one ring twice into ONE artifact. On a
//! `--record` run the consumer is the WRITER THREAD (`WriterCore::write_batch`
//! drains, writes the continuous bag, then commits). Minting a SECOND consumer
//! here to feed this window would not "share" that thread's stream — it would be
//! an independent reader of the same records, and every record would land in
//! this recorder's output TWICE, silently: once through the writer's drain, once
//! through ours. The duplication is a property of the artifact, not of the ring,
//! which is why bagd also refuses the same ring declared twice on one command
//! line (`reject_duplicate_ring_names`) while a second RECORDER on the same ring
//! is fine.
//!
//! So the window is fed from INSIDE the one drain the writer thread already
//! does, from the same span, before the same commit — exactly as
//! [`crate::anchor_window::AnchorHarvester`] is fed from inside the state ring's.
//! (The state ring's own rule is stronger and for a different reason: it is
//! `OverrunPolicy::Backpressure`, whose read cursor IS a shared header word a
//! producer waits on, so it is genuinely single-consumer.)
//!
//! # What it holds, and why decoded records are the byte-exact choice
//!
//! [`cerulion_core::trace_ring::TraceRingRecord`] values, not raw bytes.
//! `as_bytes`/`from_bytes` are an explicit little-endian encode with no padding
//! and are exact inverses, and `BagWriter::write_scheduler_trace` takes the
//! struct — so decode-here / encode-there is byte-identical BY CONSTRUCTION,
//! and the trim can read `step` and `record_type` without a second parse.
//!
//! Records are stamped with their ring's RANK on the way in, exactly as the
//! continuous bag's writer stamps them (`rank | (on_ring & TRACE_DISCARD_BIT)`),
//! so a capture's `__cerulion/scheduler_trace` bytes are the same bytes a
//! recording's are for the same records. A reader cannot tell the two apart, and
//! that is the point: resim reads a capture through the path it already reads a
//! recording through.
//!
//! # The clock is the RECORDER's
//!
//! Same rule as [`crate::window`], for the same reason: a record's
//! `fire_time_ns` is the PUBLISHER's gating clock, which on a worker
//! starts at zero. Ordering a retention by it would interleave unrelated number
//! lines. The recorder's own monotonic reading stamps each batch; the gating
//! clock still reaches the bag inside every record, it is simply not what the
//! window is ordered by.

use std::collections::{BTreeMap, VecDeque};

use cerulion_core::trace_ring::{
    TraceRingRecord, AUTHORITATIVE_TRACE_RANK, RECORD_TYPE_FIRE, RECORD_TYPE_STEP_BOUNDARY,
};

/// Bytes one retained record costs.
const RECORD_BYTES: usize = cerulion_core::trace_ring::TRACE_RECORD_SIZE as usize;

/// One drain's worth of trace records, held for the window's span.
pub(crate) struct TraceBatch {
    /// The RECORDER's monotonic reading when this batch was drained.
    pub taken_at_ns: u64,
    /// The records, already rank-stamped as the bag would write them.
    pub records: Vec<TraceRingRecord>,
}

/// What one eviction pass did.
///
/// Split exactly as [`crate::window::EvictionReport`] is, and for the same
/// reason: ageing is the retention working, while the byte ceiling taking
/// records a capture wanted is a capture covering less than it claims.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct TraceEvictionReport {
    /// Records dropped for age.
    pub aged: u64,
    /// Records the BYTE ceiling took while a capture still wanted them.
    pub truncated: u64,
}

impl TraceEvictionReport {
    fn is_empty(&self) -> bool {
        self.aged == 0 && self.truncated == 0
    }
}

/// The rolling trace retention. See the module docs.
pub(crate) struct TraceWindow {
    batches: VecDeque<TraceBatch>,
    bytes: usize,
    records: u64,
    span_ns: u64,
    max_bytes: usize,
    /// Lifetime totals, never reset (Principle #3).
    aged: u64,
    truncated: u64,
}

impl TraceWindow {
    /// A retention spanning `span_ns` and holding at most `max_bytes`.
    pub(crate) fn new(span_ns: u64, max_bytes: u64) -> Self {
        Self {
            batches: VecDeque::new(),
            bytes: 0,
            records: 0,
            span_ns,
            // CLAMPED rather than wrapped, for the reason `FrameWindow::new`
            // states: on a 32-bit target a wrap turns an operator's "no
            // practical ceiling" into "keep almost nothing", which is the one
            // direction that silently destroys the feature.
            max_bytes: usize::try_from(max_bytes).unwrap_or(usize::MAX),
            aged: 0,
            truncated: 0,
        }
    }

    /// Bytes currently held.
    pub(crate) fn bytes(&self) -> usize {
        self.bytes
    }

    /// Records currently held.
    pub(crate) fn records(&self) -> u64 {
        self.records
    }

    /// Lifetime records dropped for age.
    pub(crate) fn aged(&self) -> u64 {
        self.aged
    }

    /// Lifetime records the byte ceiling took while a capture wanted them.
    ///
    /// LIFETIME — a caller rendering ONE capture's manifest must snapshot it and
    /// report the DELTA, exactly as `FlashbackPlane` does for the frame window's
    /// twin. Printing this directly would label every later capture truncated
    /// once any capture ever lost a record.
    pub(crate) fn truncated(&self) -> u64 {
        self.truncated
    }

    /// The oldest batch's stamp, or `None` when empty.
    pub(crate) fn oldest_ns(&self) -> Option<u64> {
        self.batches.front().map(|b| b.taken_at_ns)
    }

    /// Take a drained batch into the retention.
    ///
    /// An EMPTY batch is dropped rather than held, for the reason
    /// [`crate::window::FrameWindow::push`] states: it carries nothing
    /// recoverable and would only give the retention a stamp that makes
    /// `oldest_ns` claim coverage it does not have.
    pub(crate) fn push(&mut self, taken_at_ns: u64, records: Vec<TraceRingRecord>) {
        if records.is_empty() {
            return;
        }
        self.bytes += records.len() * RECORD_BYTES;
        self.records += records.len() as u64;
        self.batches.push_back(TraceBatch {
            taken_at_ns,
            records,
        });
    }

    /// Drop what the retention no longer has to hold.
    ///
    /// `protect_from_ns` is an ACTIVE CAPTURE's floor, with the same asymmetry
    /// the frame window has: exempt from the SPAN rule (the capture already
    /// promised to carry it) and NOT exempt from the BYTE rule (a ceiling a
    /// capture could suspend is not a ceiling).
    pub(crate) fn evict(
        &mut self,
        now_ns: u64,
        protect_from_ns: Option<u64>,
    ) -> TraceEvictionReport {
        let mut report = TraceEvictionReport::default();

        // (1) AGE.
        let horizon = match protect_from_ns {
            Some(floor) => now_ns.saturating_sub(self.span_ns).min(floor),
            None => now_ns.saturating_sub(self.span_ns),
        };
        while let Some(front) = self.batches.front() {
            if front.taken_at_ns >= horizon {
                break;
            }
            report.aged += self.pop_front_counting();
        }

        // (2) BYTES — the backstop.
        while self.bytes > self.max_bytes {
            let wanted_by_capture = self
                .batches
                .front()
                .is_some_and(|b| protect_from_ns.is_some_and(|f| b.taken_at_ns >= f));
            let dropped = self.pop_front_counting();
            if dropped == 0 {
                // Empty and still over the ceiling: the ceiling is below one
                // batch. Breaking is what stops this being an infinite loop.
                break;
            }
            if wanted_by_capture {
                report.truncated += dropped;
            } else {
                report.aged += dropped;
            }
        }

        self.aged += report.aged;
        self.truncated += report.truncated;
        if !report.is_empty() {
            tracing::trace!(
                aged = report.aged,
                truncated = report.truncated,
                held = self.records,
                "flashback: trace retention rolled"
            );
        }
        report
    }

    fn pop_front_counting(&mut self) -> u64 {
        match self.batches.pop_front() {
            Some(b) => {
                let n = b.records.len();
                self.bytes -= n * RECORD_BYTES;
                self.records -= n as u64;
                n as u64
            }
            None => 0,
        }
    }

    /// Every retained record at or after `from_ns`, oldest first.
    pub(crate) fn records_from(&self, from_ns: u64) -> impl Iterator<Item = &TraceRingRecord> {
        self.batches
            .iter()
            .filter(move |b| b.taken_at_ns >= from_ns)
            .flat_map(|b| b.records.iter())
    }

    /// `target(S)` for a step, searched across the WHOLE retention.
    ///
    /// # Why this is not simply read off the trim
    ///
    /// [`trim_to_anchor`] recovers `target(S−1)` from the anchor step's own
    /// boundary record as it walks past it, which works only if that record is
    /// among the ones the walk is OFFERED — and the walk is offered
    /// [`records_from`]`(floor_ns)`, i.e. the capture's FRAME floor.
    ///
    /// Those two are different clocks' worth of the same instant: a trace batch
    /// is stamped when the RECORDER drained it, an anchor when the recorder
    /// drained the STATE ring, and `select_anchor` requires only
    /// `anchor.taken_at_ns >= floor_ns`. So a boundary drained one pass before
    /// its own anchor sits below the floor, is never offered, and the target
    /// reads `None`.
    ///
    /// That is not a remote corner. `AnchorWindow::select` picks the NEWEST
    /// checkpoint at or before the deadline, and at the shipped constants the
    /// eligible band is exactly `[floor, floor + cadence]` — one cadence wide,
    /// with the floor as its lower edge — so the chosen anchor routinely sits
    /// near the floor and the two drains straddling it is an ordinary
    /// interleaving rather than an exotic one.
    ///
    /// The COST of missing it is a silent downgrade: the manifest writes
    /// `anchor_target_ns: null`, `replay_engine::capture_anchor_target_ns` reads
    /// `None`, and the external-frame prefix skip goes away — so the first
    /// replayed step is injected with the capture's whole retained pre-window,
    /// which is precisely the flood the field exists to prevent. Wrong in the
    /// direction of doing nothing, and therefore invisible.
    ///
    /// # Why a SEARCH rather than widening the trim's own offer
    ///
    /// Widening it to `records_from(0)` would also change what the capture
    /// CARRIES (records at or past the resume step but below the frame floor
    /// would start being kept) and what `discarded` counts. The target is a
    /// FACT ABOUT THE RETENTION, not about the capture's frame floor, so it is
    /// asked as its own question and the trim's keep/discard rule is left
    /// exactly as it was.
    ///
    /// RANK-SCOPED, because a boundary record for one step is unique per rank
    /// and the answer a caller wants is the one belonging to the rank whose
    /// anchor it is recovering. An unscoped read answers the first boundary at
    /// that step in retained order, which under free run is whichever rank
    /// happened to be banked first, so a different rank's clock is reported as
    /// this rank's floor.
    pub(crate) fn boundary_target_ns_for(&self, rank: u32, step: u64) -> Option<u64> {
        self.batches
            .iter()
            .flat_map(|b| b.records.iter())
            .find(|r| {
                r.rank() == rank && r.step == step && r.record_type == RECORD_TYPE_STEP_BOUNDARY
            })
            .map(|r| r.fire_time_ns)
    }

    /// Drop everything. Called at teardown, once nothing can want it.
    pub(crate) fn clear(&mut self) {
        self.batches.clear();
        self.bytes = 0;
        self.records = 0;
    }
}

/// A trace trimmed to a capture's own anchor — what the bag carries.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(crate) struct TrimmedTrace {
    /// The records to write, in retained order.
    pub records: Vec<TraceRingRecord>,
    /// Departure (fault) boundaries among them.
    ///
    /// Counted here rather than re-derived at the verdict, so the number the
    /// manifest reports and the number the verdict judges are the same walk.
    pub departures: usize,
    /// The step of the AUTHORITATIVE rank's first kept step-boundary record: what
    /// `resolve_resume` will derive the resume point from.
    ///
    /// The two endpoints this struct carries fold in OPPOSITE directions, and the
    /// difference is which side each one has to be safe for:
    ///
    /// - the covered range's END ([`last_boundary_target_ns`](Self::last_boundary_target_ns))
    ///   is the EARLIEST last kept boundary across the ranks that kept one, because
    ///   a resume covers the graph only as far as its slowest rank can back it;
    /// - the resume's START is rank 0's own first kept boundary, because the replay
    ///   window is rank 0's boundary stream by construction and that is the step a
    ///   resume actually begins at.
    ///
    /// Rank-blind, this would be the first boundary in retained order, which on a
    /// capture whose peer carries no anchor is the peer's step 0 and makes the
    /// verdict claim a from-start resume the reader refuses.
    pub first_recorded_step: Option<u64>,
    /// The gating-clock value at the ANCHOR step's own boundary — `target(S−1)`.
    ///
    /// `None` when that boundary is not in the retention. It is the floor for the
    /// external-frame trim (see [`trim_to_anchor`]), and a capture that cannot
    /// compute it does not trim rather than guessing one.
    pub anchor_target_ns: Option<u64>,
    /// The gating-clock target of the EARLIEST rank's last kept step-boundary
    /// record: the instant a resume of this capture can cover TO.
    ///
    /// The MINIMUM over the ranks that kept a boundary, because a resume covers
    /// the graph only as far as its SLOWEST rank: past that instant one rank has
    /// no recorded boundary for a frame to be matched against. The rule used to
    /// be the AUTHORITATIVE rank's own last boundary, justified by "a peer
    /// rank's boundary is pinned to rank 0's on every shared step", a LOCKSTEP
    /// fact, true only while every rank advances together, which is precisely
    /// the assumption free run removes. For lockstep and k=1 the minimum is over
    /// one value and the number is unchanged.
    ///
    /// Which of those two shapes a run has is set by its execution mode. Under
    /// the `CERULION_EXECUTION_MODE=lockstep` opt-out the replay engine's
    /// `validate_step_boundaries` phase 2 refuses a bag whose peer targets
    /// differ from rank 0's at a shared step, so each rank's targets agree with
    /// rank 0's at every step it shares with rank 0; the minimum is then the
    /// target at the earliest last kept step, which is rank 0's own last target
    /// whenever no rank was trimmed or ended shorter than rank 0, and can still
    /// be that number when one was, because equal consecutive targets are legal.
    /// Under the FREE-RUN default that phase is mode-gated off, each rank's
    /// gating clock wall-follows on its own from the shared epoch, and the
    /// targets differ by design: the minimum is then the only instant every rank
    /// has a boundary at or before, and a peer's records can carry later ones.
    ///
    /// `None` when the kept records carry no such boundary, which is exactly the
    /// state resim refuses separately (`BagNoStepBoundaries`): a capture with no
    /// boundary has no covered range to claim, and reporting one would be the
    /// confident-false class in miniature.
    ///
    /// # Why the capture measures this at all
    ///
    /// A capture has TWO producers on TWO THREADS. Frames are staged by the
    /// recorder's DRIVE LOOP (`Recorder::harvest_window`); the scheduler trace is
    /// drained and banked by the WRITER THREAD (`WriterCore::write_batch` ->
    /// `admit_trace_batch`). `close_capture` runs on the drive loop and reads
    /// both with NO rendezvous between them (the rule is that the drain never
    /// waits on the writer), so a capture's FRAME window routinely ends one
    /// writer cycle AHEAD of its TRACE window. Those tail frames are stamped past
    /// the last boundary the bag carries, and `replay_engine`'s consistency check
    /// requires every graph-produced frame's timestamp to match a recorded
    /// `STEP_BOUNDARY` target — so it refused the bag as "corrupt or
    /// hand-edited". MEASURED: 3 of 6 captures off one `--record` run.
    ///
    /// The recording contract is that the bag keeps every frame
    /// (a black box never discards evidence) and the claim names the
    /// covered PREFIX instead. This is that claim's upper endpoint.
    ///
    /// # Why it is read off the KEPT records rather than published by the writer
    ///
    /// A shared high-water mark the writer thread stamps (an `AtomicU64` of the
    /// newest banked boundary) would answer a NEARBY question — what the
    /// RETENTION holds — and the range a reader needs is what the BAG CARRIES.
    /// The two differ whenever the trim discards records (every mid-run capture
    /// discards its pre-resume prefix) or the retention's byte ceiling truncates
    /// them (`TraceEvictionReport::truncated`). `bag play --resim` walks the
    /// bag's own trace, so a range derived from anything else is a second
    /// derivation of one fact — which is how the verdict and the gate came to
    /// disagree in the first place. Measuring it in the walk that BUILDS
    /// `records` makes them agree by construction, costs one comparison per
    /// record, needs no new shared state and touches neither thread's cadence.
    ///
    /// WITHIN one rank it is LAST-WINS rather than a maximum, deliberately: the
    /// replayer's own `BoundaryCursor` walks the trace in FILE order and ends on
    /// the last boundary it meets, and these records are written to the bag in
    /// exactly this order. On a well-formed trace (targets non-decreasing, as
    /// `validate_step_boundaries` enforces) the two are the same number;
    /// where they could differ, matching the reader is what matters. ACROSS
    /// ranks it is the minimum, for the reason above: last-wins across ranks
    /// would answer whichever rank the writer banked last, which is a race.
    pub last_boundary_target_ns: Option<u64>,
    /// The nodes the kept records FIRE, deduplicated, in first-fire order.
    ///
    /// This is resim's `fires.executed` over the replayed suffix, read off the
    /// same records resim will read it off — so the anchor-completeness question
    /// the capture asks is the question resim asks.
    pub executed_nodes: Vec<String>,
    /// Records the trim discarded as belonging to steps before the resume.
    pub discarded: usize,
    /// The same five facts PER RANK, keyed by the rank on the record.
    ///
    /// # Why the scalars above cannot serve
    ///
    /// Under free run each rank reaches its own step on its own schedule, so one
    /// capture has k cuts rather than one, and each of the facts above is a
    /// different number for each of them. The scalars answer the questions a
    /// whole-graph reader asks and each is folded in the CONSERVATIVE direction;
    /// this map answers the question a per-rank reader asks, which is the one
    /// `bag play --resim` will ask once it stops refusing k>1.
    ///
    /// Exactly one entry for a lockstep or k=1 capture, whose numbers ARE the
    /// scalars.
    pub per_rank: BTreeMap<u32, RankTrim>,
}

/// What ONE rank's half of a trimmed trace reports.
///
/// Every field is the per-rank twin of the identically named field on
/// [`TrimmedTrace`], measured over that rank's own records only. The docs on
/// those fields apply here unchanged; what differs is the scope.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(crate) struct RankTrim {
    /// The step of this rank's FIRST kept step-boundary record.
    pub first_recorded_step: Option<u64>,
    /// The gating-clock value at THIS rank's anchor step's own boundary.
    pub anchor_target_ns: Option<u64>,
    /// The gating-clock target of the LAST step-boundary this rank kept.
    pub last_boundary_target_ns: Option<u64>,
    /// The nodes THIS rank's kept records fire, resolved through THIS rank's own
    /// manifest, deduplicated, in first-fire order.
    pub executed_nodes: Vec<String>,
    /// Records discarded from THIS rank as belonging to steps before its resume.
    pub discarded: usize,
}

/// PURE: trim a retained trace to `[boundary(anchor_step + 1), end]`.
///
/// # Why the resume step and not the anchor step
///
/// `resolve_resume` reads the FIRST step-boundary record in the bag and computes
/// `anchor_step = first.step - 1`. So a trace whose first boundary is
/// `anchor_step + 1` makes resim derive EXACTLY the anchor this capture
/// embedded. Keeping one step more would resume before the checkpoint; keeping
/// one less would resume after it, past state the anchor does not describe.
///
/// # Why a step FILTER rather than a scan to the boundary record
///
/// Both give the same answer on a single-rank trace, where records are pushed in
/// step order. The filter is TOTAL: it needs no ordering assumption, so a
/// multi-rank trace (which a capture may still carry, and which is refused as
/// unresimmable for a different reason) is trimmed correctly rather than
/// arbitrarily.
///
/// # `node_ids` may be short, and that is not an error
///
/// It is the ring's own manifest. A `node_idx` past its end is a corrupt record
/// rather than an unnamed node; it contributes NO required node, because
/// inventing a name would make the anchor-completeness question unanswerable
/// against a node that does not exist.
pub(crate) fn trim_to_anchor(
    records: impl Iterator<Item = TraceRingRecord>,
    anchor_steps: &BTreeMap<u32, u64>,
    node_ids: &BTreeMap<u32, Vec<String>>,
) -> TrimmedTrace {
    walk(records, Some(anchor_steps), node_ids)
}

/// PURE: keep EVERY retained record — what a capture with no anchor carries.
///
/// Its own entry point rather than `trim_to_anchor` with a sentinel step: every
/// `u64` is a real step (0 is the run's first, `u64::MAX` discards everything),
/// so "there is no anchor" cannot be spelled as one without a reader having to
/// decode it. The facts it reports are the same ones the trimmed walk reports,
/// which is what lets the verdict judge both through one predicate.
pub(crate) fn keep_all(
    records: impl Iterator<Item = TraceRingRecord>,
    node_ids: &BTreeMap<u32, Vec<String>>,
) -> TrimmedTrace {
    walk(records, None, node_ids)
}

/// What the walk keeps, decided ONCE before it starts.
///
/// A three-way answer rather than an `Option<u64>` resume step, because there
/// are genuinely three cases and two of them are not a step number:
///
/// * [`All`](Keep::All) — no anchor to trim to,
/// * [`From`](Keep::From) — the ordinary case,
/// * [`Nothing`](Keep::Nothing) — an anchor at `u64::MAX`, which NO step can
///   follow. A `saturating_add` answers `u64::MAX` there and so keeps the anchor
///   step's OWN records, i.e. one step too many; a `wrapping_add` answers 0 and
///   keeps the whole trace. Neither is "there is nothing after the anchor", so
///   the arithmetic is not asked to express it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Keep {
    All,
    From(u64),
    Nothing,
}

impl Keep {
    /// The rule for ONE rank, from that rank's own anchor step.
    fn for_anchor(anchor_step: u64) -> Self {
        match anchor_step.checked_add(1) {
            Some(resume) => Self::From(resume),
            None => Self::Nothing,
        }
    }
}

/// The one walk both entry points share — `None` means keep everything.
///
/// # One walk, k cuts
///
/// `anchor_steps` is keyed by the rank the record carries, so each rank is
/// measured against ITS OWN resume step. Under free run the ranks reach
/// different steps at one capture deadline, and a single global cut would
/// discard one rank's records for not having reached another rank's step.
///
/// A rank the table does NOT name keeps everything. There is no cut to apply to
/// it: either it contributed no anchor to this capture (and its records are
/// evidence a black box keeps) or it is not a worker rank at all, like the
/// DEPARTURE ring, whose `node_idx` is a worker rank rather than a manifest
/// index. Trimming it against some other rank's step would discard records
/// against a number that says nothing about it.
fn walk(
    records: impl Iterator<Item = TraceRingRecord>,
    anchor_steps: Option<&BTreeMap<u32, u64>>,
    node_ids: &BTreeMap<u32, Vec<String>>,
) -> TrimmedTrace {
    let mut out = TrimmedTrace::default();
    for rec in records {
        let rank = rec.rank();
        let anchor_step = anchor_steps.and_then(|m| m.get(&rank).copied());
        let keep = match anchor_step {
            None => Keep::All,
            Some(step) => Keep::for_anchor(step),
        };
        let discard = match keep {
            Keep::All => false,
            Keep::From(resume) => rec.step < resume,
            Keep::Nothing => true,
        };
        if discard {
            // Before the resume. The ANCHOR step's own boundary is still worth
            // reading on the way past: its `fire_time_ns` is `target(S−1)`, the
            // floor the external-frame trim needs. Recorded for the rank the
            // record belongs to AND, for the authoritative rank, into the
            // scalar every reader written before the map still reads.
            if anchor_step == Some(rec.step) && rec.record_type == RECORD_TYPE_STEP_BOUNDARY {
                out.per_rank.entry(rank).or_default().anchor_target_ns = Some(rec.fire_time_ns);
                // The capture-wide scalar takes the AUTHORITATIVE rank's boundary
                // and no other. It is `target(S-1)` for the whole capture, read by
                // the recorder to trim external frames and by the replay to skip
                // the same prefix, so a peer's value makes those two numbers
                // belong to different clocks under free run. Absent rank 0's own
                // boundary the scalar stays `None` and neither side trims.
                if rank == AUTHORITATIVE_TRACE_RANK {
                    out.anchor_target_ns = Some(rec.fire_time_ns);
                }
            }
            out.per_rank.entry(rank).or_default().discarded += 1;
            out.discarded += 1;
            continue;
        }
        if rec.is_departure_boundary() {
            out.departures += 1;
        }
        if rec.record_type == RECORD_TYPE_STEP_BOUNDARY {
            let per = out.per_rank.entry(rank).or_default();
            if per.first_recorded_step.is_none() {
                per.first_recorded_step = Some(rec.step);
            }
            per.last_boundary_target_ns = Some(rec.fire_time_ns);
            // The covered range's upper endpoint is measured PER RANK here and
            // folded into the scalar after the walk. See the fold at the end of
            // this function for why the scalar is the earliest rank's.
            //
            // The old rule read it off the AUTHORITATIVE rank alone, on the
            // reasoning that "a peer rank's boundary is pinned to rank 0's on
            // every shared step". That is a LOCKSTEP fact: it is true only while
            // every rank advances together, and the whole subject of this change
            // is the run where they do not. Under free run rank 0's last
            // boundary says nothing about where a peer rank stopped.
            //
            // Mode does not change the fold, only what the minimum comes out
            // to: under the `CERULION_EXECUTION_MODE=lockstep` opt-out
            // `validate_step_boundaries` phase 2 refuses a bag whose peer
            // targets differ from rank 0's at a shared step, so each rank's
            // targets agree with rank 0's at every step it shares with rank 0;
            // the minimum is then the target at the earliest last kept step,
            // which is rank 0's own last target whenever no rank was trimmed or
            // ended shorter than rank 0, and can still be that number when one
            // was, because equal consecutive targets are legal. Under the
            // free-run default that phase is mode-gated off and the targets
            // differ by design, so it is the slowest rank's.
        }
        if rec.record_type == RECORD_TYPE_FIRE {
            // Resolved through THIS rank's own manifest. A `node_idx` names a
            // different node in each ring, so resolving a peer rank's index
            // through rank 0's table would name the wrong node, which is why
            // the recorder used to decline to resolve any of them for k>1 and
            // reported an empty executed set.
            if let Some(name) = node_ids
                .get(&rank)
                .and_then(|ids| ids.get(rec.node_idx as usize))
            {
                let per = out.per_rank.entry(rank).or_default();
                if !per.executed_nodes.iter().any(|n| n == name) {
                    per.executed_nodes.push(name.clone());
                }
                if !out.executed_nodes.iter().any(|n| n == name) {
                    out.executed_nodes.push(name.clone());
                }
            }
        }
        out.records.push(rec);
    }
    // The scalar covered-range endpoint is the EARLIEST rank's, not rank 0's.
    //
    // A resume covers the graph only as far as its SLOWEST rank's last
    // boundary: past that instant one rank has no recorded boundary for a frame
    // to be matched against, and a claim reaching further is a claim the bag
    // cannot back. Rank 0's own last boundary overstates exactly when rank 0 ran
    // longest, which under free run is an ordinary outcome rather than a rare
    // one.
    //
    // Only ranks that actually kept a boundary vote. A rank with none has no
    // endpoint to be the earliest of, and letting it fold in as "nothing" would
    // turn every capture with one quiet rank into a capture claiming no range at
    // all.
    //
    // For lockstep and k=1 the minimum is over one value and the scalar is
    // exactly what it was, which is the control the arm for this asserts.
    if let Some(earliest) = out
        .per_rank
        .values()
        .filter_map(|r| r.last_boundary_target_ns)
        .min()
    {
        out.last_boundary_target_ns = Some(earliest);
    }
    // The RESUME's start, and the fold here is the opposite shape to the one
    // above: the AUTHORITATIVE rank's own first kept boundary, not the first
    // boundary in retained order and not a minimum over the ranks.
    //
    // The replay window IS rank 0's boundary stream by construction
    // (`resolve_resume` reads `first_recorded_boundary`, which walks
    // `AUTHORITATIVE_TRACE_RANK` only), so rank 0's first kept boundary is the
    // step a resume actually begins at. Read rank-blind, this scalar took
    // whichever rank's boundary the retention happened to bank first: on a
    // capture where rank 0 resumes from a mid-run anchor and a peer carries no
    // anchor at all, the peer's step-0 boundary is retained and lands first, the
    // scalar read 0, and `judge_resimmable` stamped the capture resimmable FROM
    // START while the replay resolved rank 0's later boundary into a mid-run
    // resume and refused it. The verdict promised a resume the reader rejects.
    //
    // `None` when rank 0 kept no boundary, which is the same bag
    // `first_recorded_boundary` answers `None` for and
    // `validate_step_boundaries` refuses as `BagNoStepBoundaries`.
    out.first_recorded_step = out
        .per_rank
        .get(&AUTHORITATIVE_TRACE_RANK)
        .and_then(|r| r.first_recorded_step);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1_000_000;

    fn boundary(step: u64, target_ns: u64) -> TraceRingRecord {
        TraceRingRecord {
            step,
            fire_time_ns: target_ns,
            duration_ns: 0,
            node_idx: 0,
            global_level: 0,
            record_type: RECORD_TYPE_STEP_BOUNDARY,
            reserved: 0,
        }
    }

    fn fire(step: u64, node_idx: u32) -> TraceRingRecord {
        TraceRingRecord {
            step,
            // SATURATING: the saturation arm below drives `u64::MAX` as a step,
            // and a debug build's `*` panics there — the helper must not decide
            // which steps a test may use.
            fire_time_ns: step.saturating_mul(MS),
            duration_ns: 7,
            node_idx,
            global_level: 0,
            record_type: RECORD_TYPE_FIRE,
            reserved: 0,
        }
    }

    /// The rank-0 manifest every single-rank arm below resolves through.
    fn nodes() -> BTreeMap<u32, Vec<String>> {
        BTreeMap::from([(
            AUTHORITATIVE_TRACE_RANK,
            vec!["ticker".to_string(), "relay".to_string()],
        )])
    }

    /// The single-rank cut table: rank 0 alone, at `step`.
    fn cut(step: u64) -> BTreeMap<u32, u64> {
        BTreeMap::from([(AUTHORITATIVE_TRACE_RANK, step)])
    }

    // ---------------------------------------------------------------- window

    #[test]
    fn a_retained_batch_is_counted_in_records_and_bytes() {
        let mut w = TraceWindow::new(30_000 * MS, 1 << 30);
        assert_eq!(w.records(), 0);
        assert_eq!(w.oldest_ns(), None);

        w.push(10 * MS, vec![boundary(1, 1000), fire(1, 0)]);
        assert_eq!(w.records(), 2);
        assert_eq!(w.bytes(), 2 * RECORD_BYTES);
        assert_eq!(w.oldest_ns(), Some(10 * MS));

        // An EMPTY batch is not held — it would give `oldest_ns` a stamp with
        // nothing behind it.
        w.push(20 * MS, Vec::new());
        assert_eq!(w.records(), 2);
        assert_eq!(w.oldest_ns(), Some(10 * MS));
    }

    #[test]
    fn the_span_drops_the_oldest_and_a_capture_floor_holds_it() {
        let mut w = TraceWindow::new(100 * MS, 1 << 30);
        w.push(10 * MS, vec![fire(1, 0)]);
        w.push(60 * MS, vec![fire(2, 0)]);

        // At 150 ms the 10 ms batch is 140 ms old, past the 100 ms span.
        let report = w.evict(150 * MS, None);
        assert_eq!(report.aged, 1);
        assert_eq!(report.truncated, 0);
        assert_eq!(w.records(), 1);

        // …but a capture whose floor reaches back further KEEPS it.
        let mut held = TraceWindow::new(100 * MS, 1 << 30);
        held.push(10 * MS, vec![fire(1, 0)]);
        held.push(60 * MS, vec![fire(2, 0)]);
        let report = held.evict(150 * MS, Some(5 * MS));
        assert_eq!(report, TraceEvictionReport::default());
        assert_eq!(held.records(), 2);
    }

    /// The BYTE backstop is NOT suspended by a capture — and what it takes from
    /// one is reported as a TRUNCATION rather than as ordinary ageing.
    #[test]
    fn the_byte_ceiling_bites_through_a_capture_floor_and_says_so() {
        // A ceiling of two records.
        let mut w = TraceWindow::new(30_000 * MS, (2 * RECORD_BYTES) as u64);
        w.push(10 * MS, vec![fire(1, 0)]);
        w.push(20 * MS, vec![fire(2, 0)]);
        w.push(30 * MS, vec![fire(3, 0)]);

        let report = w.evict(40 * MS, Some(5 * MS));
        assert_eq!(report.truncated, 1, "the capture wanted it: {report:?}");
        assert_eq!(report.aged, 0);
        assert_eq!(w.records(), 2);
        assert_eq!(w.truncated(), 1, "the lifetime total moves too");

        // The same drop with NO capture is ordinary ageing, not a truncation —
        // the anti-tautology half, without which any drop would read as loss.
        let mut plain = TraceWindow::new(30_000 * MS, (2 * RECORD_BYTES) as u64);
        plain.push(10 * MS, vec![fire(1, 0)]);
        plain.push(20 * MS, vec![fire(2, 0)]);
        plain.push(30 * MS, vec![fire(3, 0)]);
        let report = plain.evict(40 * MS, None);
        assert_eq!(report.aged, 1);
        assert_eq!(report.truncated, 0);
    }

    /// A ceiling BELOW one batch must terminate rather than spin.
    #[test]
    fn a_ceiling_below_one_batch_empties_and_stops() {
        let mut w = TraceWindow::new(30_000 * MS, 1);
        w.push(10 * MS, vec![fire(1, 0), fire(1, 1)]);
        let report = w.evict(20 * MS, None);
        assert_eq!(report.aged, 2);
        assert_eq!(w.records(), 0);
        assert_eq!(w.bytes(), 0);
        // Evicting again over an empty, still-over-ceiling window is a no-op.
        assert_eq!(w.evict(30 * MS, None), TraceEvictionReport::default());
    }

    #[test]
    fn records_from_serves_the_capture_floor_oldest_first() {
        let mut w = TraceWindow::new(30_000 * MS, 1 << 30);
        w.push(10 * MS, vec![fire(1, 0)]);
        w.push(20 * MS, vec![fire(2, 0), fire(2, 1)]);
        w.push(30 * MS, vec![fire(3, 0)]);

        let steps: Vec<u64> = w.records_from(20 * MS).map(|r| r.step).collect();
        assert_eq!(steps, vec![2, 2, 3]);
        let all: Vec<u64> = w.records_from(0).map(|r| r.step).collect();
        assert_eq!(all, vec![1, 2, 2, 3]);
    }

    /// `target(S)` is recoverable from the WHOLE
    /// retention, not only from the part the capture's FRAME floor offers.
    ///
    /// The trim reads the anchor step's target off that step's own boundary
    /// record as it walks past it — which requires the record to be among the
    /// ones it is OFFERED, and it is offered `records_from(floor_ns)`. A trace
    /// batch is stamped when the RECORDER drained it while the anchor is
    /// stamped when it drained the STATE ring, so a boundary drained one pass
    /// before its own anchor sits BELOW the floor and the target reads `None` —
    /// which silently disables the external-frame prefix skip and floods the
    /// first replayed step with the capture's whole pre-window.
    #[test]
    fn the_anchor_target_is_recoverable_below_the_capture_floor() {
        let mut w = TraceWindow::new(30_000 * MS, 1 << 30);
        // The anchor step's boundary, drained in a batch BELOW the floor.
        w.push(10 * MS, vec![boundary(3, 3_000), fire(3, 0)]);
        w.push(20 * MS, vec![boundary(4, 4_000), fire(4, 0)]);

        // PRECONDITION: the floor really does hide it, or the arm is not the one
        // under test — the trim below would find it on its way past and the
        // recovery would never be exercised.
        let offered: Vec<u64> = w.records_from(20 * MS).map(|r| r.step).collect();
        assert_eq!(
            offered,
            vec![4, 4],
            "the floor must exclude the anchor step"
        );
        assert_eq!(
            trim_to_anchor(w.records_from(20 * MS).copied(), &cut(3), &nodes()).anchor_target_ns,
            None,
            "PRECONDITION: the floor-filtered trim cannot recover it on its own"
        );

        assert_eq!(
            w.boundary_target_ns_for(AUTHORITATIVE_TRACE_RANK, 3),
            Some(3_000),
            "the retention holds that boundary and must be able to answer for it"
        );
        // …and the ANTI-TAUTOLOGY half: a step the retention does not hold
        // answers `None`, so the recovery is a lookup rather than a fabrication.
        assert_eq!(w.boundary_target_ns_for(AUTHORITATIVE_TRACE_RANK, 9), None);
        // A FIRE record at the anchor step is not a boundary and must not be
        // read as one — its `fire_time_ns` is a fire time, not a step target.
        let mut fires_only = TraceWindow::new(30_000 * MS, 1 << 30);
        fires_only.push(10 * MS, vec![fire(3, 0), fire(3, 1)]);
        assert_eq!(
            fires_only.boundary_target_ns_for(AUTHORITATIVE_TRACE_RANK, 3),
            None
        );
    }

    #[test]
    fn clear_releases_everything() {
        let mut w = TraceWindow::new(30_000 * MS, 1 << 30);
        w.push(10 * MS, vec![fire(1, 0)]);
        w.clear();
        assert_eq!(w.records(), 0);
        assert_eq!(w.bytes(), 0);
        assert_eq!(w.oldest_ns(), None);
    }

    // ------------------------------------------------------------------ trim

    /// THE trim: the kept trace begins at `boundary(anchor + 1)`, so
    /// `resolve_resume` derives EXACTLY the anchor this capture embedded.
    #[test]
    fn the_trim_begins_at_the_boundary_after_the_anchor() {
        let records = vec![
            boundary(3, 3_000),
            fire(3, 0),
            boundary(4, 4_000),
            fire(4, 0),
            fire(4, 1),
            boundary(5, 5_000),
            fire(5, 1),
        ];
        let out = trim_to_anchor(records.into_iter(), &cut(4), &nodes());

        // `anchor_step + 1` == 5: everything below step 5 is discarded.
        assert_eq!(out.first_recorded_step, Some(5));
        assert_eq!(out.records.len(), 2);
        assert!(out.records.iter().all(|r| r.step == 5));
        assert_eq!(out.discarded, 5);
        // …and resim's own derivation lands back on OUR anchor.
        assert_eq!(
            out.first_recorded_step.map(|s| s - 1),
            Some(4),
            "the whole point of the trim"
        );
    }

    /// `target(anchor_step)` is read off the anchor step's OWN boundary, which
    /// the trim discards — so it must be captured on the way past or it is gone.
    #[test]
    fn the_trim_recovers_the_anchor_steps_target_from_the_record_it_discards() {
        let records = vec![
            boundary(3, 3_000),
            boundary(4, 4_321),
            fire(4, 0),
            boundary(5, 5_000),
        ];
        let out = trim_to_anchor(records.into_iter(), &cut(4), &nodes());
        assert_eq!(out.anchor_target_ns, Some(4_321));

        // …and when that boundary is NOT retained, the trim says so rather than
        // substituting a neighbouring step's target.
        let short = vec![fire(4, 0), boundary(5, 5_000)];
        let out = trim_to_anchor(short.into_iter(), &cut(4), &nodes());
        assert_eq!(out.anchor_target_ns, None);
    }

    /// The executed set is what resim will call `fires.executed`, read off the
    /// SAME records — deduplicated, and in first-fire order rather than manifest
    /// order (an order that came from the node table would not be evidence).
    #[test]
    fn the_executed_set_is_the_kept_fires_deduplicated() {
        let records = vec![
            boundary(5, 5_000),
            fire(5, 1),
            fire(5, 0),
            boundary(6, 6_000),
            fire(6, 1),
        ];
        let out = trim_to_anchor(records.into_iter(), &cut(4), &nodes());
        assert_eq!(out.executed_nodes, vec!["relay", "ticker"]);
    }

    /// A node the manifest cannot name contributes NOTHING — inventing a name
    /// would ask the anchor to cover a node that does not exist.
    #[test]
    fn a_fire_whose_node_idx_is_past_the_manifest_names_no_required_node() {
        let records = vec![boundary(5, 5_000), fire(5, 0), fire(5, 99)];
        let out = trim_to_anchor(records.into_iter(), &cut(4), &nodes());
        assert_eq!(out.executed_nodes, vec!["ticker"]);
        // …but the record is still WRITTEN: a black box does not discard
        // evidence it cannot interpret.
        assert_eq!(out.records.len(), 3);
    }

    /// Departures are counted over the KEPT records only. A fault before the
    /// resume point is not in the bag, so it cannot make the bag unresimmable.
    #[test]
    fn departures_are_counted_over_the_kept_records_only() {
        let mut early = fire(3, 0);
        early.record_type = cerulion_core::trace_ring::RECORD_TYPE_DEPARTURE;
        let mut late = fire(6, 0);
        late.record_type = cerulion_core::trace_ring::RECORD_TYPE_DEPARTURE;

        let out = trim_to_anchor(
            vec![early, boundary(5, 5_000), late].into_iter(),
            &cut(4),
            &nodes(),
        );
        assert_eq!(out.departures, 1, "only the one at or after the resume");

        let clean = trim_to_anchor(
            vec![boundary(5, 5_000), fire(5, 0)].into_iter(),
            &cut(4),
            &nodes(),
        );
        assert_eq!(clean.departures, 0);
    }

    /// A departure stamped by the RING RANK (not the record kind) counts too —
    /// the shared predicate's second signal, reached through the trim.
    #[test]
    fn a_sentinel_ranked_record_counts_as_a_departure() {
        let mut stamped = fire(6, 0);
        stamped.reserved = cerulion_core::trace_ring::DEPARTURE_RING_RANK;
        let out = trim_to_anchor(
            vec![boundary(5, 5_000), stamped].into_iter(),
            &cut(4),
            &nodes(),
        );
        assert_eq!(out.departures, 1);
    }

    /// An anchor at step 0 keeps everything from `boundary(1)`, and an anchor at
    /// `u64::MAX` keeps NOTHING — the two ends of the resume-step arithmetic.
    ///
    /// The second half is the one that has three plausible wrong answers, and it
    /// caught two of them while it was being written: `wrapping_add` answers 0
    /// and keeps the whole trace (a capture claiming to resume from an anchor it
    /// is entirely BEFORE), `saturating_add` answers `u64::MAX` and keeps the
    /// anchor step's own records (one step too many — the state a resume is
    /// about to overwrite, replayed on top of itself).
    #[test]
    fn the_resume_step_neither_wraps_nor_saturates_into_the_anchor_step() {
        let out = trim_to_anchor(
            vec![boundary(0, 0), fire(0, 0), boundary(1, 1_000)].into_iter(),
            &cut(0),
            &nodes(),
        );
        assert_eq!(out.first_recorded_step, Some(1));
        assert_eq!(out.records.len(), 1);
        assert_eq!(out.anchor_target_ns, Some(0), "step 0's own target");

        let out = trim_to_anchor(
            vec![boundary(u64::MAX, 9), fire(u64::MAX, 0)].into_iter(),
            &cut(u64::MAX),
            &nodes(),
        );
        assert_eq!(
            out.records.len(),
            0,
            "no step can follow u64::MAX, so nothing is after the anchor"
        );
        assert_eq!(out.discarded, 2);
        assert_eq!(out.first_recorded_step, None);
        // …and the anchor step's own target is still recovered on the way past,
        // exactly as it is for any other anchor.
        assert_eq!(out.anchor_target_ns, Some(9));
    }

    /// A trace with FIRE records and no boundary reports `None` rather than
    /// guessing a resume step — the state resim refuses separately.
    #[test]
    fn a_trace_with_no_boundary_reports_no_first_step() {
        let out = trim_to_anchor(vec![fire(5, 0), fire(6, 1)].into_iter(), &cut(4), &nodes());
        assert_eq!(out.first_recorded_step, None);
        assert_eq!(out.records.len(), 2);
        // And it claims NO covered range. A capture with no boundary
        // is refused separately (`BagNoStepBoundaries`), so a range here would
        // be a claim about a bag nothing can resume — the confident-false class
        // in miniature.
        assert_eq!(out.last_boundary_target_ns, None);
    }

    // ---------------------------------------------------------- covered range

    /// THE covered-range measurement: the upper endpoint is the LAST kept
    /// boundary's target, and it is measured over the records the BAG CARRIES.
    ///
    /// Both halves matter and the test asserts them apart:
    ///
    /// * the endpoint is the LAST kept boundary, not the first and not the
    ///   anchor's — a range ending at the resume step would call every frame
    ///   after the first replayed step "outside the covered range" and skip the
    ///   whole recording;
    /// * the DISCARDED anchor boundary is still read for `anchor_target_ns`, so
    ///   the two endpoints of the range come from two different records and one
    ///   cannot be mistaken for the other.
    #[test]
    fn the_trim_reports_the_last_boundary_it_keeps_as_the_covered_range() {
        let records = vec![
            boundary(4, 4_000),
            fire(4, 0),
            boundary(5, 5_000),
            fire(5, 0),
            boundary(6, 6_000),
            fire(6, 1),
        ];
        let out = trim_to_anchor(records.into_iter(), &cut(4), &nodes());

        assert_eq!(
            out.last_boundary_target_ns,
            Some(6_000),
            "the covered range ends at the LAST boundary the bag carries"
        );
        assert_eq!(
            out.anchor_target_ns,
            Some(4_000),
            "…while the range's LOWER endpoint is still the discarded anchor \
             boundary's target — two records, two endpoints"
        );
        assert_eq!(out.first_recorded_step, Some(5));
    }

    /// A capture with NOTHING after its anchor claims no range at all.
    ///
    /// The trim keeps no record, so there is no boundary to end a range at —
    /// and this is the shape `flashback_trace_e2e_test`'s
    /// `a_trace_with_no_boundary_past_the_anchor_is_trimmed_away_and_reads_not_
    /// resimmable` drives end to end. A capture that reported a range here would
    /// be stating coverage over a trace it does not carry.
    #[test]
    fn a_trim_that_keeps_nothing_claims_no_covered_range() {
        let out = trim_to_anchor(
            vec![boundary(4, 4_000), fire(4, 0)].into_iter(),
            &cut(4),
            &nodes(),
        );
        assert!(out.records.is_empty());
        assert_eq!(out.last_boundary_target_ns, None);
        // …and the anchor's own target is STILL recovered on the way past, so
        // "no range" is not the same answer as "nothing was measured".
        assert_eq!(out.anchor_target_ns, Some(4_000));
    }

    /// The endpoint never runs PAST a peer rank's last boundary.
    ///
    /// The foreign record sits LAST in retained order and carries the HIGHER
    /// target, so an implementation that simply took the newest boundary it saw
    /// answers `9_000` and this arm reads it directly. It is also the arm that
    /// held while the rule was "the authoritative rank's alone", which is why it
    /// is kept: the two rules agree here, and the arm below is where they part.
    ///
    /// A peer's target is not rank 0's under the free-run default: that clock
    /// wall-follows on its own from the shared epoch, and the check that pins
    /// the two together under the `CERULION_EXECUTION_MODE=lockstep` opt-out,
    /// `validate_step_boundaries` phase 2, is mode-gated off. The higher number
    /// here is therefore a shape a real run produces.
    #[test]
    fn a_foreign_ranks_boundary_never_ends_the_covered_range() {
        let mut foreign = boundary(7, 9_000);
        foreign.reserved = 1;
        let out = trim_to_anchor(
            vec![boundary(5, 5_000), fire(5, 0), foreign].into_iter(),
            &cut(4),
            &nodes(),
        );
        assert_eq!(out.last_boundary_target_ns, Some(5_000));
        // …and the foreign record is still WRITTEN: a black box does not discard
        // evidence it declines to measure against.
        assert_eq!(out.records.len(), 3);
    }

    /// A rank-1 boundary and a rank-0 fire record, for the k=2 arms below.
    fn ranked_boundary(rank: u32, step: u64, target_ns: u64) -> TraceRingRecord {
        let mut r = boundary(step, target_ns);
        r.reserved = rank;
        r
    }

    fn ranked_fire(rank: u32, step: u64, node_idx: u32) -> TraceRingRecord {
        let mut r = fire(step, node_idx);
        r.reserved = rank;
        r
    }

    /// The capture wide anchor target stays EMPTY rather than borrow a peer's.
    ///
    /// `anchor_target_ns` is `target(S-1)` for the capture as a whole and BOTH
    /// sides read the same number off it: the recorder trims external frames below
    /// it, and the replay skips the same prefix at the first resumed step. Under
    /// free run each rank advances on its own clock, so a peer's target is a
    /// different instant; taking it would trim the bag against one clock and skip
    /// against another.
    ///
    /// `None` leaves both sides at their untrimmed answer, which agree: the
    /// recorder keeps the whole prefix and the replay reads an empty skip map.
    ///
    /// THE SHAPE: rank 0 is anchored at 41 and its step 41 boundary is NOT in the
    /// records, so there is nothing of its own to recover, while rank 1 is anchored
    /// at 44 and its step 44 boundary IS retained and discarded on the way past.
    /// The peer's 44_100 is therefore available to be borrowed and must not be.
    #[test]
    fn the_capture_wide_anchor_target_never_borrows_a_peers_boundary() {
        let out = trim_to_anchor(
            vec![
                // Rank 1's own anchor boundary, below its resume, so the walk reads
                // it as that rank's floor.
                ranked_boundary(1, 44, 44_100),
                ranked_boundary(1, 45, 45_100),
                // Rank 0 resumes at 42 with NO boundary at its own anchor step 41.
                ranked_boundary(0, 42, 42_000),
                ranked_fire(0, 42, 0),
            ]
            .into_iter(),
            &BTreeMap::from([(AUTHORITATIVE_TRACE_RANK, 41u64), (1, 44u64)]),
            &ranked_nodes(),
        );
        assert_eq!(
            out.per_rank[&1].anchor_target_ns,
            Some(44_100),
            "PRECONDITION: the peer's own floor IS recovered, so there is a value \
             available to be borrowed"
        );
        assert_eq!(
            out.per_rank
                .get(&AUTHORITATIVE_TRACE_RANK)
                .and_then(|r| r.anchor_target_ns),
            None,
            "PRECONDITION: rank 0 has no boundary at its own anchor step"
        );
        assert_eq!(
            out.anchor_target_ns, None,
            "the capture wide floor is rank 0's own or nothing, never the peer's \
             44100: both the trim and the replay's skip read this one number"
        );
    }

    /// A PEER with no anchor does not make the capture claim a from-start resume.
    ///
    /// The shape, which is an ordinary multi rank capture and not a crafted one:
    /// rank 0 resumes from a MID RUN anchor, so its records below the resume are
    /// discarded, while rank 1 carries NO anchor and keeps everything including its
    /// step 0 boundary. Retained order then puts rank 1's step 0 boundary FIRST.
    ///
    /// Read rank-blind, the scalar took that 0 and `judge_resimmable` returned on
    /// its step 0 arm, stamping the capture resimmable FROM START. The replay does
    /// not agree: `resolve_resume` reads `first_recorded_boundary`, which walks
    /// rank 0 only, resolves rank 0's step 42 into a mid run resume, needs an
    /// anchor for it, and for k>1 refuses the recording. The verdict promised a
    /// resume the reader rejects.
    ///
    /// HAND ORACLE. Rank 0 keeps from 42, rank 1 keeps from 0, so the scalar is 42,
    /// which is above 0 and therefore NOT the from-start arm. 0 is asserted absent
    /// from the scalar, which is the number the rank-blind read produced, so a
    /// return to it fails here rather than in a replay.
    #[test]
    fn a_peers_step_zero_boundary_does_not_make_the_resume_claim_from_start() {
        let out = trim_to_anchor(
            vec![
                // Rank 1 has no anchor, so its step 0 boundary is KEPT and, banked
                // first, is the first boundary in retained order.
                ranked_boundary(1, 0, 100),
                ranked_fire(1, 0, 0),
                // Rank 0's own stream resumes from its mid run anchor at 41.
                ranked_boundary(0, 41, 41_000),
                ranked_boundary(0, 42, 42_000),
                ranked_fire(0, 42, 0),
            ]
            .into_iter(),
            // Rank 0 alone carries an anchor; rank 1 is absent from the cut table,
            // which is `Keep::All` for it.
            &BTreeMap::from([(AUTHORITATIVE_TRACE_RANK, 41u64)]),
            &ranked_nodes(),
        );
        assert_eq!(
            out.per_rank[&1].first_recorded_step,
            Some(0),
            "PRECONDITION: rank 1 really did keep its step 0 boundary, which is the \
             whole shape of this arm"
        );
        assert_eq!(
            out.first_recorded_step,
            Some(42),
            "the resume's START is rank 0's own first kept boundary, which is the \
             step `resolve_resume` will reach for"
        );
        assert_ne!(
            out.first_recorded_step,
            Some(0),
            "and never the peer's step 0, which is what made the verdict claim a \
             from-start resume the reader refuses"
        );
    }

    /// The two ranks' own manifests, DIFFERENT lists at the same indices, which
    /// is the whole reason a shared table names the wrong node.
    fn ranked_nodes() -> BTreeMap<u32, Vec<String>> {
        BTreeMap::from([
            (0, vec!["ticker".to_string(), "relay".to_string()]),
            (1, vec!["planner".to_string(), "arm".to_string()]),
        ])
    }

    /// ORACLE 2: each rank is trimmed from ITS OWN cut, and nothing else is.
    ///
    /// Rank 0's cut is 41 and rank 1's is 44, so rank 0 keeps from 42 and rank 1
    /// keeps from 45. Stated record by record rather than by a count, because a
    /// count passes for a trim that discarded the right NUMBER of the wrong
    /// records: a global cut at the minimum would keep rank 1's steps 42 to 44,
    /// and a global cut at the maximum would discard rank 0's 42 to 44.
    #[test]
    fn each_rank_is_trimmed_from_its_own_cut_and_nothing_else_is() {
        let records = vec![
            ranked_boundary(0, 41, 41_000),
            ranked_boundary(1, 41, 41_100),
            ranked_boundary(0, 42, 42_000),
            ranked_boundary(1, 44, 44_100),
            ranked_boundary(0, 43, 43_000),
            ranked_boundary(1, 45, 45_100),
        ];
        let cuts = BTreeMap::from([(0, 41), (1, 44)]);
        let out = trim_to_anchor(records.into_iter(), &cuts, &ranked_nodes());

        let kept: Vec<(u32, u64)> = out.records.iter().map(|r| (r.rank(), r.step)).collect();
        assert_eq!(
            kept,
            vec![(0, 42), (0, 43), (1, 45)],
            "rank 0 from 42, rank 1 from 45, and nothing else"
        );
        assert_eq!(out.discarded, 3, "41 and 41 and 44");
        assert_eq!(out.per_rank[&0].discarded, 1);
        assert_eq!(out.per_rank[&1].discarded, 2);
        // Each rank's own anchor target, read off its own discarded boundary.
        assert_eq!(out.per_rank[&0].anchor_target_ns, Some(41_000));
        assert_eq!(out.per_rank[&1].anchor_target_ns, Some(44_100));
        assert_eq!(out.per_rank[&0].first_recorded_step, Some(42));
        assert_eq!(out.per_rank[&1].first_recorded_step, Some(45));
    }

    /// ORACLE 3: `executed_nodes` stops being empty for k>1, and each rank's
    /// indices resolve through THAT rank's own manifest.
    ///
    /// At the parent this answered an EMPTY list for every k>1 capture, because
    /// the recorder declined to resolve any index rather than resolve a peer
    /// rank's through rank 0's table. The two manifests here name different
    /// nodes at the same indices, so a shared-table implementation answers
    /// `ticker` and `relay` for both ranks and this arm reads the substitution
    /// directly rather than only the emptiness.
    #[test]
    fn each_ranks_executed_nodes_resolve_through_that_ranks_own_manifest() {
        let records = vec![
            ranked_boundary(0, 42, 42_000),
            ranked_fire(0, 42, 0),
            ranked_boundary(1, 45, 45_100),
            ranked_fire(1, 45, 1),
        ];
        let cuts = BTreeMap::from([(0, 41), (1, 44)]);
        let out = trim_to_anchor(records.into_iter(), &cuts, &ranked_nodes());

        assert_eq!(out.per_rank[&0].executed_nodes, vec!["ticker".to_string()]);
        assert_eq!(
            out.per_rank[&1].executed_nodes,
            vec!["arm".to_string()],
            "rank 1's index 1 is `arm`, not rank 0's `relay`"
        );
        // …and the scalar is the UNION, in first-fire order, so a reader that
        // wants the whole graph's executed set still has one.
        assert_eq!(
            out.executed_nodes,
            vec!["ticker".to_string(), "arm".to_string()],
            "no longer empty for k>1, which is the defect this closes"
        );
    }

    /// ORACLE 4: the scalar covered-range endpoint is the EARLIEST rank's, and
    /// the per-rank map keeps both.
    ///
    /// Rank 0 runs LONGEST here, which is exactly the shape where the old rule
    /// (rank 0's own last boundary) OVERSTATES: it would claim coverage through
    /// 43,000 while rank 1 has no recorded boundary past 41,100, so a frame
    /// stamped between them has nothing to be matched against.
    #[test]
    fn the_scalar_covered_range_is_the_earliest_ranks_not_rank_zeros() {
        let records = vec![
            ranked_boundary(0, 42, 42_000),
            ranked_boundary(1, 45, 41_100),
            ranked_boundary(0, 43, 43_000),
        ];
        let cuts = BTreeMap::from([(0, 41), (1, 44)]);
        let out = trim_to_anchor(records.into_iter(), &cuts, &ranked_nodes());

        assert_eq!(out.per_rank[&0].last_boundary_target_ns, Some(43_000));
        assert_eq!(out.per_rank[&1].last_boundary_target_ns, Some(41_100));
        assert_eq!(
            out.last_boundary_target_ns,
            Some(41_100),
            "the slowest rank's endpoint, not rank 0's 43_000"
        );

        // THE CONTROL: under lockstep there is one rank, the minimum is over one
        // value, and the scalar is UNCHANGED from what the parent answered.
        let lockstep = trim_to_anchor(
            vec![boundary(42, 42_000), fire(42, 0), boundary(43, 43_000)].into_iter(),
            &cut(41),
            &nodes(),
        );
        assert_eq!(
            lockstep.last_boundary_target_ns,
            Some(43_000),
            "k=1 is the identity: the scalar is the one rank's own last boundary"
        );
        assert_eq!(lockstep.per_rank.len(), 1);
    }

    /// The boundary recovery answers the NAMED rank's clock and never a peer's.
    ///
    /// `boundary_target_ns_for(rank, step)` exists so the trim can recover
    /// `target(S)` for a step whose boundary record sits below the capture's
    /// frame floor. Under free run two ranks reach one step at two instants, so
    /// the lookup has to be keyed by both: a rank-blind `find` answers whichever
    /// rank's record the retention happens to hold first, and the scalar then
    /// carries another rank's clock as this rank's floor.
    ///
    /// Rank 1's record is pushed FIRST here deliberately. That is the order in
    /// which a rank-blind lookup answers rank 1's clock for BOTH ranks, so the
    /// arm reads the substitution rather than only an absence. Every expected
    /// value is written out by hand.
    ///
    /// The three cases: each rank's own step, and a rank the retention holds
    /// nothing for, so the lookup is a lookup rather than a fabrication.
    #[test]
    fn the_boundary_recovery_answers_the_named_ranks_clock_and_not_a_peers() {
        let mut w = TraceWindow::new(30_000 * MS, 1 << 30);
        w.push(
            10 * MS,
            vec![
                ranked_boundary(1, 3, 31_000),
                ranked_boundary(0, 3, 30_000),
                ranked_boundary(0, 4, 40_000),
            ],
        );

        assert_eq!(
            w.boundary_target_ns_for(0, 3),
            Some(30_000),
            "rank 0's own clock at step 3, not rank 1's 31_000"
        );
        assert_eq!(
            w.boundary_target_ns_for(1, 3),
            Some(31_000),
            "rank 1's own clock at the SAME step"
        );
        assert_eq!(
            w.boundary_target_ns_for(0, 4),
            Some(40_000),
            "a step only rank 0 reached is still rank 0's"
        );
        // A step rank 1 never reached answers NOTHING, rather than borrowing the
        // rank that did reach it.
        assert_eq!(w.boundary_target_ns_for(1, 4), None);
        // …and a rank the retention holds no record for at all.
        assert_eq!(w.boundary_target_ns_for(2, 3), None);
    }

    /// `keep_all` — the NO-ANCHOR capture — measures the same endpoint.
    ///
    /// Its own arm because the two entry points share `walk` but not their
    /// filters, and a capture with no anchor is exactly the shape whose whole
    /// retention is carried: it has the LONGEST frame window and therefore the
    /// most to lose to a tail race.
    #[test]
    fn an_untrimmed_capture_reports_its_covered_range_too() {
        let out = keep_all(
            vec![boundary(0, 0), fire(0, 0), boundary(1, 1_000)].into_iter(),
            &nodes(),
        );
        assert_eq!(out.last_boundary_target_ns, Some(1_000));
        assert_eq!(out.first_recorded_step, Some(0));
    }

    /// The same input trimmed twice is the same output — the retention feeds a
    /// bag, and a bag that differed run to run would not be replay-grade.
    #[test]
    fn the_trim_is_deterministic() {
        let records = vec![
            boundary(4, 4_000),
            fire(4, 0),
            boundary(5, 5_000),
            fire(5, 1),
            fire(5, 0),
        ];
        let a = trim_to_anchor(records.clone().into_iter(), &cut(4), &nodes());
        let b = trim_to_anchor(records.into_iter(), &cut(4), &nodes());
        assert_eq!(a, b);
        // …and against a HAND oracle, so this is not two runs of one closure.
        assert_eq!(a.first_recorded_step, Some(5));
        assert_eq!(a.anchor_target_ns, Some(4_000));
        assert_eq!(a.last_boundary_target_ns, Some(5_000));
        assert_eq!(a.executed_nodes, vec!["relay", "ticker"]);
        assert_eq!(a.discarded, 2);
        assert_eq!(a.departures, 0);
        assert_eq!(
            a.records.iter().map(|r| r.step).collect::<Vec<_>>(),
            vec![5, 5, 5]
        );
    }
}
