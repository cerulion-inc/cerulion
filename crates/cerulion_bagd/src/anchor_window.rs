// SPDX-License-Identifier: AGPL-3.0-only
//! The ROLLING ANCHOR RETENTION — the recorder's own
//! drop-oldest buffer of the NEWEST node-state checkpoints, and the thing that
//! turns a Flashback from a viewable dump into a resumable one.
//!
//! # The inversion this closes
//!
//! A capture plane is armed on every serving `graph run`, and with
//! nothing draining the per-rank state ring the producer's
//! [`free_records()`](cerulion_core::state_ring::StateRingProducer::free_records)
//! precheck declines every anchor after the ring fills ONCE. The ring then
//! retains the run's **oldest** anchors — a five-hour-old robot's checkpoint
//! from its first thirty seconds — and a black box needs the NEWEST.
//!
//! Draining it is the whole answer, and it needs somewhere for the drained anchors
//! to go. That is this module: the recorder holds the last few checkpoints, in
//! its own memory, under its own bounds, exactly as [`crate::window::FrameWindow`]
//! holds the last few seconds of frames.
//!
//! # Why RAW RECORDS and not reassembled blobs
//!
//! A recorded bag carries anchors on the reserved `__cerulion/state` channel as
//! the ring's 512-byte records VERBATIM, and `cerulion bag play --resim` reads
//! them back through the same [`StateAssembler`] a live drain runs. So the
//! retention holds the records, not the blob they assemble into:
//!
//! * a capture's bytes are then **byte-identical** to what a `--record` bag holds
//!   for the same anchor, so the resim reader needs no new path and no new
//!   framing, and
//! * nothing re-encodes. Re-chunking a reassembled blob would reproduce those
//!   records only as long as two copies of the chunker agreed forever — the
//!   two-copies drift class, for no gain.
//!
//! A [`StateAssembler`] still runs, because "which records make ONE whole anchor"
//! is a rule that must have exactly one implementation; its reassembled `bytes`
//! are DISCARDED. The cost is that an anchor is transiently held twice while it
//! is in flight (the assembler's buffer and ours) — inherent to retaining
//! anything at all, since the records are reclaimed the moment the cursor
//! advances, and bounded by [`AnchorHarvester::max_open_bytes`].
//!
//! # The clock is the RECORDER's, for the reason the frame window states
//!
//! An anchor record carries a STEP, never a nanosecond ("a step is not a
//! nanosecond" — `WriterCore::flush_batch`). A step cannot be compared with the
//! frame window's horizon, so a checkpoint is stamped with the recorder's own
//! monotonic reading when its first record was drained. That is an UPPER BOUND
//! on when the anchor was really taken (the drain happens at or after the
//! commit), which is conservative in the direction that matters: an anchor
//! selected as "at or before T−15" was really taken at or before T−15.
//!
//! # Bounds, and which of them is a promise
//!
//! Same split as the frame window. The SPAN is the promise: keep every
//! checkpoint the window still covers, plus the single newest one that fell out
//! of it (the CARRY — see [`AnchorWindow::evict`]). At cadence `C` over a span
//! `W` that is `⌈W/C⌉ + 1` checkpoints, DERIVED from the window rather than from
//! a second copy of the cadence knob the graph process owns.
//!
//! The BYTE ceiling is the backstop, and **the rule is strict about what
//! reaching it costs**. A robot whose checkpoints do not
//! fit does NOT degrade to a frames-only Flashback — still a black box, not a resumable
//! one. The rule is: "STRICTLY NEVER FRAMES-ONLY. Every Flashback
//! capture must be resimmable." A plane that cannot hold one whole generation
//! raises a standing alarm and REFUSES to finalize a capture rather than writing
//! one that cannot resim.
//!
//! Two things follow for this module. The ceiling is no longer a fixed number:
//! `FlashbackPlane` re-splits its budget ANCHOR-FIRST as the demand becomes
//! known and re-points it through [`AnchorWindow::set_max_bytes`], so what is
//! here is the ceiling currently in force rather than the one the operator set.
//! And a capture that loses EVERY rank's anchor to it is not written at all, so
//! the `RetentionCeilingExhausted` reason below is what the REFUSAL is keyed on
//! when it applies to every rank — there it decides whether a bag exists, not
//! merely what a manifest says.
//!
//! # …and STRICTLY NEVER SILENTLY PARTIAL
//!
//! The rule above is about the WHOLE capture and it is unchanged. A k-rank run
//! adds a second shape it never had to answer for: a capture that has one
//! rank's state and not another's. Throwing that away would discard the state
//! of every healthy rank because one rank was unlucky, and the run an operator
//! is trying to understand is exactly the run where one rank went wrong. So
//! such a capture IS written, and the rule that carries the weight instead is
//! that it is never written SILENTLY: [`AnchorSelection`] returns the set and
//! the SHORTFALL together, exhaustive over every ring the retention knows
//! about, and the manifest stamps each absent rank with its number and the
//! reason its own retention gave. A capture with a hole nothing can account for
//! is the one outcome this module exists to make impossible.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::sync::Arc;

use cerulion_core::state_ring::{
    StateAnchorEvent, StateAssembler, StateRecordHeader, STATE_RECORD_HEADER_SIZE,
    STATE_RECORD_SIZE,
};

/// One 512-byte state-ring record.
pub(crate) type StateRecord = [u8; STATE_RECORD_SIZE as usize];

/// What a retained anchor IS — the two shapes a reader can act on.
///
/// A TORN anchor is deliberately absent: it can never be served, so retaining
/// one would spend the byte ceiling on bytes no resim may use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AnchorKind {
    /// Every part arrived and the stream closed on its FINAL record.
    Complete,
    /// The writer VOIDED this node's anchor on purpose and said why. Retained
    /// because a capture that carries the skip tells a reader WHY a node has no
    /// state, where a capture that dropped it says only that one is missing.
    Skipped,
}

/// One node's whole anchor at one step, as the records that carried it.
#[derive(Debug, Clone)]
pub(crate) struct RetainedAnchor {
    /// The SHM name of the ring this came off — which ring's manifest resolves
    /// `node_idx`.
    pub ring: String,
    /// Index into that ring's node-identity table.
    pub node_idx: u32,
    /// The node id `node_idx` resolved to, or `None` when the ring's manifest
    /// has no such index.
    ///
    /// Resolved at HARVEST time and carried, rather than resolved at capture
    /// time: the ring may be gone by then (a run that exited unlinks its SHM
    /// name), and an anchor whose node cannot be named is one a resim cannot
    /// apply — better to know that while the ring is still open.
    pub node: Option<String>,
    /// Complete or Skipped.
    pub kind: AnchorKind,
    /// The raw records in PART order, exactly as they sat in the ring.
    ///
    /// # Why this is an `Arc`
    ///
    /// A capture's close SELECTS a checkpoint — which clones it — and then
    /// copied every record out of it again, so closing a capture memcpy'd the
    /// whole checkpoint TWICE on the drive loop. The anchor-reserve arithmetic
    /// makes that up to ~1.2 GiB per copy on a 25-rank robot, at exactly the
    /// moment the recorder must keep draining taps whose SHM queues are the loss
    /// boundary. Sharing the record vector makes both the selection
    /// clone and the hand-off to the writer O(#anchors) pointer bumps.
    ///
    /// The `Arc` wraps the whole PART VECTOR rather than each record, because
    /// the vector is what is built once by the harvester and never mutated
    /// afterwards — `admit` appends whole `RetainedAnchor`s to a checkpoint, so
    /// no already-shared vector is ever written to and no copy-on-write is
    /// possible.
    pub records: Arc<Vec<StateRecord>>,
}

impl RetainedAnchor {
    /// Bytes this anchor occupies in the retention.
    pub(crate) fn byte_len(&self) -> usize {
        self.records.len() * STATE_RECORD_SIZE as usize
    }

    /// Whether this anchor carries real state rather than a skip marker.
    ///
    /// By design the generation measurement counts only these. A
    /// `Skipped` anchor is one record saying why a node did not checkpoint, so
    /// counting it toward the demand would let a run whose nodes all skipped
    /// report a generation size that no real checkpoint will ever have.
    pub(crate) fn is_complete(&self) -> bool {
        self.kind == AnchorKind::Complete
    }

    /// Stage A: WHY this anchor was skipped, read off its own record.
    ///
    /// `None` for a COMPLETE anchor, and for a `Skipped` one whose records do
    /// not carry a skip — which cannot happen through
    /// [`AnchorHarvester::feed`] (a `Skipped` kind is minted only by a skip
    /// record) but is answered from the records rather than assumed.
    ///
    /// # Why it is read rather than carried
    ///
    /// The cause is IN the record — a skip is self-contained and authoritative,
    /// and `StateAnchorLedger` reads it from exactly these four bytes. Storing a
    /// second copy on the retained anchor would be a second thing to keep in
    /// step with the wire, and the alternative a caller reaches for when the
    /// cause is not available is to INVENT one, which is the fake-data rule this
    /// repo refuses: `plan_restore` renders the cause into the sentence an
    /// operator reads about why their capture cannot be resumed.
    pub(crate) fn skip_cause(&self) -> Option<cerulion_core::state::SkipCause> {
        if self.kind != AnchorKind::Skipped {
            return None;
        }
        let record = self.records.last()?;
        let header = StateRecordHeader::from_bytes(
            record[..STATE_RECORD_HEADER_SIZE]
                .try_into()
                .expect("40-byte header"),
        );
        if header.kind != cerulion_core::state_ring::RECORD_KIND_SKIP_V2 {
            return None;
        }
        // The DECLARED length gates the read. A record is a fixed-size slot, so
        // the four bytes after the header are always ADDRESSABLE — they are just
        // not always the writer's. A skip declaring fewer than four payload
        // bytes has its cause word made up of zero-filled padding, and zeros
        // decode to real, confidently-named causes: a one-byte payload of
        // `0x01` reads `Contended`, which would send an operator looking for a
        // node mutex that was never held.
        //
        // Reported as `Unrecognized` rather than `None`, on the module's own
        // rule that a SKIP is authoritative: the writer said this anchor was
        // voided, and only its REASON is unreadable. `None` would say the
        // opposite — that this is not a skip at all — and drop the fact.
        const CAUSE_LEN: u32 = 4;
        if header.len < CAUSE_LEN {
            return Some(cerulion_core::state::SkipCause::Unrecognized(0));
        }
        let raw = u32::from_le_bytes(
            record[STATE_RECORD_HEADER_SIZE..STATE_RECORD_HEADER_SIZE + CAUSE_LEN as usize]
                .try_into()
                .expect("4-byte slice"),
        );
        Some(cerulion_core::state::SkipCause::from_wire(raw))
    }
}

/// The producer RANK this anchor's records carry, read off the first one.
///
/// Read rather than carried, on exactly the rule
/// [`RetainedAnchor::skip_cause`] states for the skip cause: the rank is IN the
/// record from state record format version 1, so a second copy on the retained
/// anchor would be a second thing to keep in step with the wire. `None` for an
/// anchor holding no record, which is a shape the harvester does not produce and
/// which is answered rather than assumed.
pub(crate) fn producer_rank(records: &[StateRecord]) -> Option<u32> {
    let first = records.first()?;
    Some(
        StateRecordHeader::from_bytes(
            first[..STATE_RECORD_HEADER_SIZE]
                .try_into()
                .expect("a 512-byte record always holds a 40-byte header"),
        )
        .rank,
    )
}

/// PURE: the STATE bytes a whole anchor's records carry, which is the sum of
/// each record's declared `len` and never the record footprint.
///
/// The distinction matters for coverage: `byte_len` is what the anchor
/// costs the RETENTION (footprint, which is what a byte ceiling must bound),
/// while a coverage manifest's `bytes` is how much STATE the node's anchor
/// accounts for. A 4-byte blob rides one 512-byte record, and reporting 512
/// would describe the framing rather than the state.
///
/// A record whose header will not parse contributes nothing rather than a
/// guess; every record here came through the assembler, so that is unreachable
/// and the arm exists only so a malformed one cannot inflate the count.
pub(crate) fn anchor_payload_bytes(records: &[StateRecord]) -> u64 {
    records
        .iter()
        .map(|record| {
            let header = StateRecordHeader::from_bytes(
                record[..STATE_RECORD_HEADER_SIZE]
                    .try_into()
                    .expect("a 512-byte record always holds a 40-byte header"),
            );
            u64::from(header.len)
        })
        .sum()
}

/// Every anchor seen for one `(run_id, ring, step)` — one RANK's half of the
/// unit a resim resumes from.
///
/// A resume needs EVERY executed node's state at ONE step (`plan_restore`), so
/// the checkpoint rather than the individual anchor is what the retention keeps,
/// evicts and selects.
///
/// # Why the RING is part of the key
///
/// A k-worker run drains k state rings and each rank reaches a step on its own
/// schedule, so two ranks at step 41 are two independent facts. Folded into one
/// entry they became "whichever rank admitted first", and that one rank's
/// checkpoint was then served as the whole graph's state — a set with a hole
/// and nothing able to say so. Keying by the ring keeps each rank's checkpoint
/// whole.
///
/// The CROSS-RANK unit — every ring's entry for one `(run_id, step)` — is still
/// the thing eviction takes and the thing a capture selects a SET from; it is
/// derived from the key rather than stored, so there is one place the key is
/// written down.
///
/// The RING rather than the rank, because the ring name is what the harvester
/// stamps onto every anchor and what `RetainedAnchor` already carries. The RANK
/// is read out of the records through [`producer_rank`], on the rule that the
/// rank is IN the record and a second copy would be a second thing to keep in
/// step with the wire.
#[derive(Debug, Clone)]
pub(crate) struct Checkpoint {
    /// The run these anchors belong to.
    pub run_id: u64,
    /// The SHM name of the ring every anchor here came off — the second
    /// component of the key.
    pub ring: String,
    /// The anchor step.
    pub step: u64,
    /// The RECORDER's monotonic reading when this checkpoint's FIRST record was
    /// drained — see the module docs on why this is not the step.
    pub taken_at_ns: u64,
    /// Its anchors, in admission order.
    pub anchors: Vec<RetainedAnchor>,
    bytes: usize,
}

impl Checkpoint {
    /// Bytes this checkpoint occupies.
    pub(crate) fn byte_len(&self) -> usize {
        self.bytes
    }

    /// The bytes its COMPLETE anchors occupy — what a GENERATION costs.
    ///
    /// By design this is the demand the anchor reserve is sized
    /// from. Apart from [`byte_len`](Self::byte_len), which is the retention
    /// FOOTPRINT and includes skip markers: a reserve sized off the footprint of
    /// a mostly-skipped checkpoint would be sized off records no resume can use.
    pub(crate) fn complete_byte_len(&self) -> usize {
        self.anchors
            .iter()
            .filter(|a| a.is_complete())
            .map(|a| a.byte_len())
            .sum()
    }

    /// How many of its anchors are COMPLETE — the ones a resim can apply.
    pub(crate) fn complete_anchors(&self) -> usize {
        self.anchors
            .iter()
            .filter(|a| a.kind == AnchorKind::Complete)
            .count()
    }

    /// Every record it holds, as the SHARED per-anchor groups they are stored
    /// in — admission order, then part order within each group.
    ///
    /// This replaced a flat `records()` that yielded borrows, whose
    /// only caller had to copy every record back out to own them. Handing back
    /// the `Arc`s instead makes a capture's hand-off to the writer thread one
    /// pointer bump per anchor. The flat form was deleted rather than kept
    /// beside this one: it had no other caller, and the dead-code policy is what
    /// stops a second way of reading a checkpoint drifting from the first.
    pub(crate) fn record_groups(&self) -> impl Iterator<Item = &Arc<Vec<StateRecord>>> {
        self.anchors.iter().map(|a| &a.records)
    }

    /// Records it holds, across every anchor.
    ///
    /// NOT the number of anchors and NOT the number of groups: a multi-part
    /// anchor is several records in one group, and the manifest's `records` is
    /// what was written onto the state channel.
    pub(crate) fn record_count(&self) -> usize {
        self.anchors.iter().map(|a| a.records.len()).sum()
    }

    /// The RANK that produced it, read off its first record's header.
    ///
    /// Read rather than carried, for the reason
    /// [`producer_rank`] is read rather than carried: the rank is IN the record,
    /// and a second copy on the checkpoint would be a second thing to keep in
    /// step with the wire. `None` for a checkpoint holding no record at all,
    /// which is a shape the harvester does not produce and which is answered
    /// rather than assumed.
    pub(crate) fn producer_rank(&self) -> Option<u32> {
        self.anchors.iter().find_map(|a| producer_rank(&a.records))
    }
}

/// What one eviction pass did.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct AnchorEvictionReport {
    /// Checkpoints dropped because they aged past the span (and were not the
    /// carry). Ordinary — this is the retention working.
    pub aged: u64,
    /// Checkpoints dropped because the BYTE ceiling bit. Never ordinary: under
    /// the never-frames-only rule a capture that loses its anchor this way is refused
    /// rather than written frames-only, so this counts checkpoints whose loss
    /// costs a whole capture.
    pub truncated: u64,
}

impl AnchorEvictionReport {
    fn is_empty(&self) -> bool {
        self.aged == 0 && self.truncated == 0
    }
}

/// Why a capture carries no anchor. Rendered into the capture's manifest, so a
/// reader is never left to guess.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum NoAnchorReason {
    /// Nothing was ever retained: the run declared no state ring, the plane was
    /// refused at arm time, or no anchor has come due yet.
    NothingRetained,
    /// Checkpoints exist, but every one of them predates the frames this capture
    /// carries — so resuming from it would execute steps whose inputs are not in
    /// the bag.
    AllOlderThanTheFrames,
    /// The retention's BYTE CEILING refused or dropped every checkpoint: state was
    /// captured and the plane simply could not afford to hold it.
    ///
    /// Its own reason, rather than folding into
    /// [`NothingRetained`](Self::NothingRetained), because the two are different
    /// facts with different remedies. "Nothing was retained" says
    /// the run produced no anchors — check the arm gate, check the cadence. This
    /// says the run produced them and the CEILING took them, which is fixed by
    /// giving the plane more room. Reporting both as "no anchor retained" told an
    /// operator to look in exactly the wrong place.
    ///
    /// **This reason does two different jobs, and which one depends on HOW MANY
    /// RANKS it applies to.**
    ///
    /// Applied to EVERY rank it decides whether a bag exists. It is then the one
    /// no-anchor reason a capture is REFUSED for: the plane could not hold a
    /// whole generation for anybody, so any bag would be a dashcam clip wearing
    /// the Flashback name, which the never-frames-only rule forbids. The other
    /// two total absences are still written with an accurate non-resim verdict —
    /// `NothingRetained` covers a run younger than its first cadence, where the
    /// remedy is to wait rather than to change a knob.
    ///
    /// Applied to ONE RANK OF SEVERAL it decides what a manifest SAYS. The
    /// capture is written, because discarding every healthy rank's state over
    /// one unlucky rank would throw away the recording of exactly the run an
    /// operator is trying to understand, and that rank is STAMPED with this
    /// reason in the manifest's missing-rank block. The remedy is the same
    /// sentence in both jobs (give the plane more room); what differs is whether
    /// a bag exists to read it in.
    ///
    /// Misclassifying one reason as another is therefore never merely a wrong
    /// word: at the whole-capture level it refuses a capture that should have
    /// been written or writes one that should have been refused, and at the
    /// per-rank level it sends an operator after the wrong problem for a rank
    /// whose state they no longer have.
    RetentionCeilingExhausted,
}

impl NoAnchorReason {
    /// What an operator should DO about this rank, as one sentence.
    ///
    /// Three reasons, three remedies, and they point in three different
    /// directions: wait, re-cut the window, or give the plane more room. A
    /// missing-rank stamp carrying only the wire word would make a reader look
    /// all three up, and a stamp carrying one remedy for all three would send
    /// two thirds of them to the wrong place.
    ///
    /// Written for a MISSING RANK rather than for a refused capture: the subject
    /// of each sentence is the rank, because that is what the stamp names.
    pub(crate) fn rank_remedy(self) -> &'static str {
        match self {
            Self::NothingRetained => {
                "this rank retained no checkpoint: it published no state ring, its plane was \
                 refused at arm time, or it had not reached its first anchor cadence when the \
                 capture was triggered"
            }
            Self::AllOlderThanTheFrames => {
                "this rank's checkpoints all predate the frames this capture carries, so \
                 resuming from one would execute steps whose inputs are not in the bag: \
                 widen the window span or shorten the anchor cadence"
            }
            Self::RetentionCeilingExhausted => {
                "the retention's byte ceiling took this rank's checkpoints: raise the anchor \
                 ceiling so the plane can hold one whole checkpoint generation"
            }
        }
    }

    /// The wire spelling, and the one an operator reads.
    pub(crate) fn as_wire(self) -> &'static str {
        match self {
            Self::NothingRetained => "no_anchor_retained",
            Self::AllOlderThanTheFrames => "anchors_older_than_the_frames",
            Self::RetentionCeilingExhausted => "anchors_dropped_by_the_byte_ceiling",
        }
    }
}

/// Which checkpoint a capture got, and how it compares with what it asked for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AnchorFit {
    /// At or before the capture's claimed pre-window start: the whole claimed
    /// window is covered by a resume from here.
    CoversTheClaimedWindow,
    /// The only usable checkpoint is NEWER than the claimed pre-window start, so
    /// a resume covers less than the capture's frames do. Served anyway — some
    /// coverage beats none — and SAID, because the difference is the operator's
    /// to judge.
    NewerThanTheClaimedWindow,
}

/// One rank's member of a capture's anchor SET.
#[derive(Debug, Clone)]
pub(crate) struct SelectedAnchor {
    /// That rank's checkpoint, cloned out of the retention.
    pub checkpoint: Checkpoint,
    /// How it compares with what THIS capture claimed. Per rank, because the
    /// ranks' answers genuinely differ: one rank's newest checkpoint can sit
    /// inside the claimed window while another's only candidate is newer than
    /// it.
    pub fit: AnchorFit,
    /// The number of the CAPTURE EVENT this member was selected for.
    ///
    /// Stamped at SELECTION, never at harvest, and the distinction is the whole
    /// reason the field can exist at all: [`AnchorWindow::admit`] pushes a
    /// checkpoint on the recorder's drive loop long before any capture is
    /// triggered, so a checkpoint cannot know which capture would take it, while
    /// `select` is called ONCE per capture with that capture's own frozen
    /// `(floor_ns, deadline_ns)` pair. One retained checkpoint may therefore be
    /// selected by several captures in turn and carry a different number into
    /// each one.
    ///
    /// What it is FOR: a restore point that names its capture lets a reader
    /// refuse a resume assembled from two different captures' anchors, which is
    /// otherwise a silent wrong answer rather than an error.
    pub capture_seq: u64,
}

/// What one capture's selection returned: the SET, and the ranks it could not
/// fill.
///
/// A SET rather than one checkpoint. A k-rank run holds k independent
/// retentions, and each rank's newest complete checkpoint at or before the SAME
/// capture deadline is its own answer. A NON-SIMULTANEOUS cut is ACCEPTED here:
/// the ranks may sit at different steps, and the consistency of cross-rank edges
/// is repaired downstream by positioning each CONSUMING rank's read log, with
/// the clock as the fallback.
///
/// The two maps are exhaustive together over the rings the retention knows
/// about, which is the property the missing-rank stamp rests on: a rank absent
/// from `selected` is NAMED in `shortfall` with the reason its own retention
/// gave, so a capture can never be written with a hole nobody can account for.
#[derive(Debug, Clone, Default)]
pub(crate) struct AnchorSelection {
    /// The chosen checkpoint per ring, keyed by the ring's SHM name.
    pub selected: std::collections::BTreeMap<String, SelectedAnchor>,
    /// Why each ring that contributed NOTHING contributed nothing.
    pub shortfall: std::collections::BTreeMap<String, MissingRank>,
}

/// One ring that contributed nothing to a capture's anchor set: WHICH rank, and
/// WHY.
///
/// Both halves are required for the stamp to be worth anything. The reason
/// alone tells an operator what went wrong without saying to whom, and on a
/// twenty-rank robot that is not actionable; the rank alone tells them who
/// without saying what to do about it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct MissingRank {
    /// The rank, when it can be known.
    ///
    /// `None` only for a ring the retention was never TOLD the rank of and
    /// which holds no record to read one off. Answered rather than defaulted to
    /// 0: a stamp that named rank 0 for an unknown ring would accuse a rank that
    /// may be perfectly healthy, which is worse than saying the rank is unknown.
    pub rank: Option<u32>,
    /// The reason that ring's own retention gave.
    pub reason: NoAnchorReason,
}

impl AnchorSelection {
    /// The one selected member, for a retention holding a single ring.
    ///
    /// TEST-ONLY. Production folds the whole set (a k-rank capture has no "the"
    /// checkpoint), so a production caller reaching for one member would be
    /// publishing one rank's numbers as the capture's — the exact defect the set
    /// exists to remove.
    #[cfg(test)]
    pub(crate) fn sole(&self) -> (&Checkpoint, AnchorFit) {
        assert_eq!(
            self.selected.len(),
            1,
            "sole() on a selection of {} members",
            self.selected.len()
        );
        let picked = self.selected.values().next().expect("one member");
        (&picked.checkpoint, picked.fit)
    }
}

/// The rolling checkpoint retention. See the module docs.
pub(crate) struct AnchorWindow {
    /// Ascending by `taken_at_ns` (the harvest clock is monotone, and a
    /// checkpoint is stamped by its FIRST record).
    checkpoints: VecDeque<Checkpoint>,
    span_ns: u64,
    max_bytes: usize,
    bytes: usize,
    /// Lifetime totals, never reset (Principle #3).
    admitted: u64,
    aged: u64,
    truncated: u64,
    /// Every node the drained rings' MANIFESTS declare, keyed by ring name.
    ///
    /// Recorded when a ring is OPENED, not when one of its anchors lands, and
    /// held HERE rather than read at capture time for the reason
    /// `RetainedAnchor::node` states: by the time a capture closes the ring may
    /// be gone (a run that exited unlinks its SHM name), and a rank that has not
    /// anchored ONCE is exactly the one whose absence a capture must report.
    ///
    /// It is what lets a capture's coverage name a node that was DUE to anchor
    /// and did not — see `crate::build_capture_state_coverage`. A checkpoint is
    /// every anchor seen for one `(run_id, step)`, and "seen" is not "expected":
    /// a sibling whose records are still in flight, or a rank that died, simply
    /// is not there, and a manifest built only from what arrived can never say so.
    declared_nodes: BTreeMap<String, Vec<String>>,
    /// The RANK of every ring the retention has been told about.
    ///
    /// Apart from `declared_nodes` because the two answer different questions
    /// and one of them survives an empty node table: a ring is declared with its
    /// rank even when its node list is empty, and the missing-rank stamp needs
    /// the rank whether or not the nodes are known.
    declared_ranks: BTreeMap<String, u32>,
    /// Times the BYTE CEILING refused or dropped a checkpoint — from this
    /// window's own eviction AND from a harvester abandoning an anchor in flight
    /// (see [`note_ceiling_refusal`](Self::note_ceiling_refusal)).
    ///
    /// Its own axis because `aged`/`truncated` classify by whether a CAPTURE was
    /// hurt, so neither can answer "did the ceiling ever bite" — the question a
    /// REFUSED capture must answer to distinguish a ceiling that took everything from a run
    /// that simply never anchored.
    ceiling_refusals: u64,
    /// The largest anchor-shaped thing the CEILING has
    /// taken, in bytes. See [`note_refused_bytes`](Self::note_refused_bytes).
    max_refused_bytes: usize,
    /// The largest complete generation this run has ever
    /// held, in bytes.
    ///
    /// # STICKY, and that is the safe direction
    ///
    /// It only ever rises. A checkpoint is filled by its ranks one anchor at a
    /// time, so a generation reads SMALLER until its last sibling lands, and a
    /// figure that could fall would let a half-filled checkpoint shrink the
    /// reserve that a full one needs — which is the reserve collapsing exactly
    /// when the state grew. Rising-only means the demand converges upward on the
    /// real figure and the reserve is never sized below something already seen.
    ///
    /// Its cost, stated: state that genuinely SHRINKS for the rest of a run
    /// keeps a reserve sized for the peak. That is bounded (the reserve is
    /// capped by the plane) and it is the direction that cannot lose a
    /// checkpoint, which is what the never-frames-only rule is about.
    max_generation_bytes: usize,
}

impl AnchorWindow {
    /// A retention covering `span_ns` and holding at most `max_bytes`.
    pub(crate) fn new(span_ns: u64, max_bytes: u64) -> Self {
        Self {
            checkpoints: VecDeque::new(),
            span_ns,
            // CLAMPED rather than wrapped, for the reason `FrameWindow::new`
            // states: a wrap turns "keep everything" into "keep almost nothing".
            max_bytes: usize::try_from(max_bytes).unwrap_or(usize::MAX),
            bytes: 0,
            admitted: 0,
            aged: 0,
            truncated: 0,
            declared_nodes: BTreeMap::new(),
            declared_ranks: BTreeMap::new(),
            ceiling_refusals: 0,
            max_refused_bytes: 0,
            max_generation_bytes: 0,
        }
    }

    /// Record every node one ring's manifest declares.
    ///
    /// Called when the ring is OPENED — by the drive loop for a declared ring
    /// and by the discovery sweep for a rank found by name — so a ring that
    /// never closes a single anchor is still known to have been expected. Keyed
    /// by the name `admit_harvested_anchors` stamps onto each anchor, so the two
    /// halves cannot disagree about which ring is which.
    ///
    /// Idempotent by overwrite: a ring name is unique and its manifest is fixed
    /// for the life of the ring, so a re-declaration is the same list.
    pub(crate) fn declare_ring_nodes(&mut self, ring: &str, rank: u32, nodes: &[String]) {
        self.declared_nodes.insert(ring.to_string(), nodes.to_vec());
        self.declared_ranks.insert(ring.to_string(), rank);
    }

    /// The RANK of a ring, from the declaration or from what it is holding.
    ///
    /// Two sources because a ring can be known through either: the drive loop
    /// and the discovery sweep DECLARE a ring when they open it, and a ring the
    /// retention only ever saw records from is known through the rank its own
    /// producer stamped into them. The declaration wins when both answer, since
    /// it is the rank the opener was handed rather than one read back out.
    fn ring_rank(&self, ring: &str) -> Option<u32> {
        self.declared_ranks.get(ring).copied().or_else(|| {
            self.checkpoints
                .iter()
                .filter(|c| c.ring == ring)
                .find_map(Checkpoint::producer_rank)
        })
    }

    /// The declared node table, cloned for a capture to judge against.
    ///
    /// CLONED for the reason `select_anchor` clones its checkpoint: the caller
    /// builds a manifest without holding the lock the drive loop needs. It is a
    /// handful of node ids per rank.
    pub(crate) fn declared_nodes(&self) -> BTreeMap<String, Vec<String>> {
        self.declared_nodes.clone()
    }

    /// Bytes currently held.
    pub(crate) fn bytes(&self) -> usize {
        self.bytes
    }

    /// Checkpoints currently held.
    pub(crate) fn checkpoints(&self) -> usize {
        self.checkpoints.len()
    }

    /// Anchors admitted, lifetime.
    pub(crate) fn admitted(&self) -> u64 {
        self.admitted
    }

    /// Checkpoints aged out, lifetime.
    pub(crate) fn aged(&self) -> u64 {
        self.aged
    }

    /// Times the BYTE CEILING refused or dropped a checkpoint, from either site.
    pub(crate) fn ceiling_refusals(&self) -> u64 {
        self.ceiling_refusals
    }

    /// Record that a HARVESTER refused an anchor in flight for exceeding its
    /// buffer ceiling.
    ///
    /// The other half of the same fact: such an anchor never becomes a checkpoint,
    /// so the window's own eviction can never see it, and a capture would report
    /// "nothing retained" for a run whose state was captured and refused on cost.
    /// The harvest reports it here, so ONE counter answers the question wherever
    /// the ceiling bit.
    ///
    /// The largest complete generation ever held.
    ///
    /// `None` until one has been measured — the caller must not read a zero as
    /// "state costs nothing", which is the one reading that would reserve
    /// nothing and then wonder why captures do not resim.
    pub(crate) fn max_generation_bytes(&self) -> Option<u64> {
        (self.max_generation_bytes > 0).then_some(self.max_generation_bytes as u64)
    }

    /// Re-point the byte ceiling.
    ///
    /// The anchor RESERVE is re-derived as the measured generation grows, so
    /// this ceiling moves during a run — upwards as the demand becomes known.
    /// Like the frame window's, a shrink takes effect on the next
    /// [`evict`](Self::evict) rather than here, so eviction stays the one place
    /// checkpoints are dropped.
    pub(crate) fn set_max_bytes(&mut self, max_bytes: u64) {
        self.max_bytes = usize::try_from(max_bytes).unwrap_or(usize::MAX);
    }

    /// The ceiling in force. TEST-ONLY, and deliberately so: production reads
    /// the split's own `anchor_bytes` (the plane applies it), so a getter here
    /// would be a second answer to one question. What a test needs, and cannot
    /// get any other way, is proof that the split really reached this field.
    #[cfg(test)]
    pub(crate) fn max_bytes_for_test(&self) -> u64 {
        self.max_bytes as u64
    }

    pub(crate) fn note_ceiling_refusal(&mut self, refusals: u64) {
        self.ceiling_refusals = self.ceiling_refusals.saturating_add(refusals);
    }

    /// The largest anchor-shaped thing the CEILING has
    /// taken, in bytes — a FLOOR on what the plane would have to hold.
    ///
    /// Fed from BOTH refusal sites, because they observe different halves of the
    /// same fact: this window's own eviction knows a whole checkpoint's COMPLETE
    /// bytes, and the harvester knows the partial buffer an in-flight anchor had
    /// accumulated. Sticky max over both.
    ///
    /// `0` means NOTHING was observed, never "zero bytes" — the remedy renderer
    /// keys on that to name the knob with NO number rather than print a zero.
    pub(crate) fn note_refused_bytes(&mut self, bytes: u64) {
        let bytes = usize::try_from(bytes).unwrap_or(usize::MAX);
        self.max_refused_bytes = self.max_refused_bytes.max(bytes);
    }

    /// The largest refused anchor-shaped thing seen, or `None` if none was.
    pub(crate) fn refused_bytes_floor(&self) -> Option<u64> {
        (self.max_refused_bytes > 0).then_some(self.max_refused_bytes as u64)
    }

    /// Checkpoints the BYTE ceiling took, lifetime — the degradation
    /// counter.
    pub(crate) fn truncated(&self) -> u64 {
        self.truncated
    }

    /// The oldest held checkpoint's stamp, or `None` when empty — the MINIMUM
    /// over rings.
    ///
    /// Stated as a minimum rather than read off the front, even though the
    /// admission clamp keeps the queue ascending and the two therefore agree
    /// today. With k rings in one queue the reader's question is "how far back
    /// does the retention reach for ANY rank", and a front read answers it only
    /// as long as the clamp holds; the minimum answers it by construction.
    pub(crate) fn oldest_ns(&self) -> Option<u64> {
        self.checkpoints.iter().map(|c| c.taken_at_ns).min()
    }

    /// Take one harvested anchor into its checkpoint.
    ///
    /// `taken_at_ns` stamps a checkpoint only the FIRST time it is seen: the
    /// stamp answers "when was this checkpoint taken", and a later-arriving
    /// sibling anchor from a slower rank must not drag that forward past the
    /// deadline a selection compares it against.
    ///
    /// # The queue is ASCENDING, and that is enforced rather than assumed
    ///
    /// Eviction and selection both walk from the front and stop at the first
    /// checkpoint that is new enough, so an out-of-order stamp would make the
    /// retention evict the wrong thing and serve the wrong anchor — silently,
    /// with every counter still adding up. The stamp is therefore CLAMPED to the
    /// newest already held.
    ///
    /// In production it is a no-op: both drain sites read one monotonic origin
    /// and only ONE of them is ever the ring's consumer (the state ring is
    /// SPSC). The clamp exists so that stops being something a reader has to
    /// verify by tracing callers, and so a future third caller cannot break the
    /// invariant by accident.
    pub(crate) fn admit(&mut self, taken_at_ns: u64, anchor: HarvestedAnchor) {
        let taken_at_ns = match self.checkpoints.back() {
            Some(newest) => taken_at_ns.max(newest.taken_at_ns),
            None => taken_at_ns,
        };
        let HarvestedAnchor {
            run_id,
            step,
            node_idx,
            kind,
            records,
            ring,
            node,
        } = anchor;
        // The checkpoint key's second component. Cloned rather than borrowed
        // because the name moves onto the anchor below, and a ring name is a few
        // dozen bytes against a checkpoint that is measured in megabytes.
        let ring_key = ring.clone();
        let retained = RetainedAnchor {
            ring,
            node_idx,
            node,
            kind,
            // The ONE place a harvested record vector becomes a shared one. The
            // harvester built it and is handing it over, so this is a move into
            // an allocation, never a copy of the records.
            records: Arc::new(records),
        };
        let added = retained.byte_len();
        self.admitted += 1;
        self.bytes += added;
        // Reverse scan: a sibling of the checkpoint being filled right now is at
        // or near the back, and a retention holds a handful of them.
        if let Some(existing) = self
            .checkpoints
            .iter_mut()
            .rev()
            .find(|c| c.run_id == run_id && c.ring == ring_key && c.step == step)
        {
            existing.anchors.push(retained);
            existing.bytes += added;
        } else {
            self.checkpoints.push_back(Checkpoint {
                run_id,
                ring: ring_key,
                step,
                taken_at_ns,
                anchors: vec![retained],
                bytes: added,
            });
        }
        // The demand is the whole CROSS-RANK checkpoint's, summed over every
        // ring's entry for this step, because that is what a capture writes
        // together and therefore what the reserve has to hold. Measured off ONE
        // ring's entry it is the demand divided by k, and a plane sized from it
        // loses k-1 ranks to the ceiling on the first real capture.
        let generation = self.generation_complete_bytes(run_id, step);
        self.note_generation(generation);
    }

    /// The COMPLETE bytes of the whole cross-rank checkpoint at `(run_id, step)`
    /// — the sum over every ring's entry for that step.
    ///
    /// Not the sum over every RETAINED step: the reserve has to hold one
    /// generation at a time, and summing the retention would size it for the
    /// whole rolling buffer.
    fn generation_complete_bytes(&self, run_id: u64, step: u64) -> usize {
        self.checkpoints
            .iter()
            .filter(|c| c.run_id == run_id && c.step == step)
            .map(|c| c.complete_byte_len())
            .sum()
    }

    /// Raise the measured generation demand if this checkpoint is the biggest
    /// yet. See [`max_generation_bytes`](Self::max_generation_bytes).
    fn note_generation(&mut self, generation_bytes: usize) {
        self.max_generation_bytes = self.max_generation_bytes.max(generation_bytes);
    }

    /// Drop what the retention no longer has to hold.
    ///
    /// `protect_from_ns` is an ACTIVE CAPTURE's floor, on exactly the frame
    /// window's terms: exempt from the SPAN rule (the capture already promised
    /// to carry it), never exempt from the BYTE rule.
    ///
    /// # What eviction TAKES: a whole cross-rank checkpoint
    ///
    /// Both arms take every ring's entry for one `(run_id, step)` or none of
    /// them. Taking one ring's entry would leave the rest of that step held and
    /// unusable: a resume needs a member per rank, so a step missing one rank
    /// restores nothing while still costing the retention the other k-1 ranks'
    /// bytes. Whole-step eviction also keeps the two counters meaning what they
    /// say — `aged` and `truncated` count CHECKPOINTS, and a per-entry count
    /// would multiply both by k on the same run.
    ///
    /// # The CARRY
    ///
    /// The newest checkpoint that has aged out is KEPT, and that is the `+1` in
    /// `⌈W/C⌉ + 1`. Without it a checkpoint that ages out microseconds before a
    /// trigger leaves the capture with only checkpoints NEWER than its claimed
    /// pre-window start — so the capture would resume from a later instant than
    /// it could have, purely because of where the eviction pass fell. Keeping one
    /// costs one cadence of memory and removes the phase dependence.
    ///
    /// The carry is retained, never SERVED past the frames: [`select`](Self::select)
    /// refuses any checkpoint below the capture's own floor, because resuming
    /// from one would execute steps whose inputs the bag does not carry.
    pub(crate) fn evict(
        &mut self,
        now_ns: u64,
        protect_from_ns: Option<u64>,
    ) -> AnchorEvictionReport {
        let mut report = AnchorEvictionReport::default();

        // (1) AGE, minus the carry. The horizon is pulled BACK to an active
        // capture's floor exactly as the frame window's is.
        let horizon = match protect_from_ns {
            Some(floor) => now_ns.saturating_sub(self.span_ns).min(floor),
            None => now_ns.saturating_sub(self.span_ns),
        };
        // Every cross-rank checkpoint strictly older than the horizon is a
        // candidate; the NEWEST of those is the carry and stays. So drop while at
        // least TWO are below the horizon.
        while self.aged_below(horizon) >= 2 {
            if !self.pop_front_counting() {
                break;
            }
            report.aged += 1;
        }

        // (2) BYTES. The backstop; the carry has no exemption here either.
        while self.bytes > self.max_bytes {
            let Some(front) = self.checkpoints.front() else {
                // Empty and still over the ceiling: the ceiling is below one
                // checkpoint. Nothing further can go.
                break;
            };
            let oldest = (front.run_id, front.step);
            let wanted_by_capture = protect_from_ns.is_some_and(|f| {
                self.checkpoints
                    .iter()
                    .filter(|c| (c.run_id, c.step) == oldest)
                    .any(|c| c.taken_at_ns >= f)
            });
            // Read the whole cross-rank checkpoint's COMPLETE bytes BEFORE
            // it is popped — the ceiling just proved the plane could not hold a
            // generation this big, which is the evidence the remedy falls back to
            // when no whole generation was ever measured. Summed over the k
            // entries, for the same reason `admit` measures the demand that way:
            // one ring's entry is the figure divided by k.
            let refused_bytes = self.generation_complete_bytes(oldest.0, oldest.1) as u64;
            self.note_refused_bytes(refused_bytes);
            if !self.pop_front_counting() {
                // Nothing went, and the loop would otherwise spin — breaking is
                // what stops this being an infinite loop.
                break;
            }
            // ONE refusal for the whole cross-rank checkpoint, not k. The counter
            // answers "how many CHECKPOINTS did the ceiling take", which is the
            // number the remedy sentence and the refusal verdict are written
            // against; counting entries would report a k-rank run as having lost
            // k times as much as it did.
            //
            // Counted on its OWN axis as well as into the report's two buckets.
            // Those buckets answer "was a CAPTURE hurt", which splits a byte
            // eviction across `truncated` and `aged` depending on whether one was
            // active — so neither can answer "did the CEILING ever take
            // anything", which is what a REFUSED capture has to say
            // for itself.
            self.ceiling_refusals += 1;
            if wanted_by_capture {
                report.truncated += 1;
            } else {
                report.aged += 1;
            }
        }

        self.aged += report.aged;
        self.truncated += report.truncated;
        if !report.is_empty() {
            tracing::trace!(
                aged = report.aged,
                truncated = report.truncated,
                held_bytes = self.bytes,
                held_checkpoints = self.checkpoints.len(),
                "flashback anchors: evicted"
            );
        }
        report
    }

    /// How many whole CROSS-RANK checkpoints sit strictly below `horizon`.
    ///
    /// Distinct `(run_id, step)` pairs, not deque entries: with k rings a
    /// per-entry count reads k times the real number, so the age arm would drop
    /// k-1 generations too many and the carry would be gone.
    ///
    /// A generation counts as below the horizon only when its NEWEST entry is.
    /// The entries of one step are not contiguous in the queue — a slow rank's
    /// step-41 anchor is stamped after a fast rank's step-42 one and lands
    /// behind it — so a step can straddle the horizon, and taking the whole step
    /// off the strength of its OLDEST entry would evict an entry the window
    /// still covers.
    fn aged_below(&self, horizon: u64) -> usize {
        let mut newest: BTreeMap<(u64, u64), u64> = BTreeMap::new();
        for c in &self.checkpoints {
            let seen = newest.entry((c.run_id, c.step)).or_insert(0);
            *seen = (*seen).max(c.taken_at_ns);
        }
        newest.values().filter(|ns| **ns < horizon).count()
    }

    /// Drop the OLDEST whole cross-rank checkpoint — every ring's entry for the
    /// front entry's `(run_id, step)`. `false` when there was none.
    ///
    /// The whole queue is walked rather than only its head, because the entries
    /// of one step are not contiguous (see [`aged_below`](Self::aged_below)).
    /// Order is preserved for the survivors, which is what keeps the queue
    /// ascending for eviction and selection.
    fn pop_front_counting(&mut self) -> bool {
        let Some(front) = self.checkpoints.front() else {
            return false;
        };
        let key = (front.run_id, front.step);
        let mut kept = VecDeque::with_capacity(self.checkpoints.len());
        let mut dropped = false;
        while let Some(c) = self.checkpoints.pop_front() {
            if (c.run_id, c.step) == key {
                self.bytes -= c.bytes;
                dropped = true;
            } else {
                kept.push_back(c);
            }
        }
        self.checkpoints = kept;
        dropped
    }

    /// Choose the SET of checkpoints a capture should carry — one per rank.
    ///
    /// `floor_ns` is the capture's own frame floor and `deadline_ns` is the
    /// instant its CLAIMED pre-window starts (`capture_start − post_window`).
    /// Both are the CAPTURE's, frozen at its trigger, and ONE pair serves every
    /// rank: the deadline is what makes the k answers one cut rather than k
    /// unrelated selections.
    ///
    /// The rule runs PER RING, in the order it is applied:
    ///
    /// 1. A candidate must sit at or after `floor_ns`. A checkpoint older than
    ///    the oldest frame in the bag is UNUSABLE, not merely suboptimal: a
    ///    resume from it executes steps whose inputs were never recorded, and
    ///    the result would be a divergence caused by the recording rather than
    ///    by the code under test.
    /// 2. A candidate must be one
    ///    ([`checkpoint_is_complete`](Self::checkpoint_is_complete)) — see that
    ///    doc for why a partial checkpoint and an all-declined one both are.
    ///    A rank with no candidate contributes NOTHING and is named in
    ///    `shortfall` with its reason, rather than being silently omitted and
    ///    leaving the set with a hole nothing accounts for.
    /// 3. Among candidates, the NEWEST at or before `deadline_ns` — so a resume
    ///    covers the whole window the capture claims, and re-executes no more
    ///    than it has to.
    /// 4. If none reaches the deadline, the OLDEST candidate: the most coverage
    ///    still available, reported as
    ///    [`AnchorFit::NewerThanTheClaimedWindow`] so the shortfall is the
    ///    reader's to see rather than a silent difference.
    ///
    /// # Different steps across ranks are ACCEPTED
    ///
    /// The ranks' answers are compared against one DEADLINE, never against each
    /// other, so a set whose members sit at steps 41 and 44 is a correct set
    /// rather than a refusal. A graph-wide simultaneous cut is not something the
    /// recorder can produce without stopping every rank, and the downstream
    /// repair — positioning each CONSUMING rank's read log, with the clock as
    /// the fallback — is what makes the non-simultaneous cut resumable.
    ///
    /// # The ring UNIVERSE, and why it is not read off the held checkpoints
    ///
    /// A rank whose every checkpoint the ceiling took holds nothing, so a
    /// universe derived from the queue would not know that rank existed and the
    /// capture would be written with an unaccounted hole. The universe is
    /// therefore every ring the retention has been TOLD about
    /// ([`declare_ring_nodes`](Self::declare_ring_nodes), called when a ring is
    /// OPENED) joined with every ring currently holding a checkpoint, so a ring
    /// is missed only if it was never opened and never anchored.
    ///
    /// `Err` only when NO ring contributed, and then with the worst reason the
    /// rings gave, so a capture that would have been refused before this change
    /// is still refused for the same reason.
    ///
    /// # The capture identity
    ///
    /// `capture_seq` is the number of the capture making this call, and every
    /// member of the returned set is stamped with it. It is the caller's because
    /// the retention has no idea a capture exists: see
    /// [`SelectedAnchor::capture_seq`] for why the stamp cannot be applied at
    /// harvest time instead.
    pub(crate) fn select(
        &self,
        floor_ns: u64,
        deadline_ns: u64,
        capture_seq: u64,
    ) -> Result<AnchorSelection, NoAnchorReason> {
        let mut rings: BTreeSet<&str> = self
            .declared_nodes
            .keys()
            .map(String::as_str)
            .collect::<BTreeSet<&str>>();
        rings.extend(self.checkpoints.iter().map(|c| c.ring.as_str()));
        let mut out = AnchorSelection::default();
        for ring in rings {
            match self.select_one_ring(ring, floor_ns, deadline_ns, capture_seq) {
                Ok(picked) => {
                    out.selected.insert(ring.to_string(), picked);
                }
                Err(reason) => {
                    out.shortfall.insert(
                        ring.to_string(),
                        MissingRank {
                            rank: self.ring_rank(ring),
                            reason,
                        },
                    );
                }
            }
        }
        if out.selected.is_empty() {
            return Err(self.no_anchor_reason(&out.shortfall));
        }
        Ok(out)
    }

    /// Whether a checkpoint is a SELECTION CANDIDATE at all.
    ///
    /// A retained checkpoint is one, and the predicate exists to be READ rather
    /// than to filter: the per-rank rule and its two rejected alternatives are
    /// the subject of this doc, and a future change that wants a stricter rule
    /// has to answer the two arms below before it can have one.
    ///
    /// # Why not "an anchor for every node the ring DECLARED"
    ///
    /// Because the retention cannot separate the two shapes such a rule would
    /// have to separate, and the shipped contract is the opposite one. A
    /// checkpoint is filled one node at a time, so a node missing from it is
    /// either still IN FLIGHT or never anchored at all, and nothing the
    /// retention holds tells those apart. The recorder's answer, which predates
    /// this change, is to WRITE the checkpoint and NAME the absent node in
    /// `state_coverage.json` rather than throw the capture away: a partial
    /// checkpoint is evidence, and an operator who can see which node is missing
    /// is better served than one handed a refusal. A declared-table rule turns
    /// every such capture into a total refusal, which is a k=1 behaviour change
    /// the lockstep control below forbids.
    /// [`a_checkpoint_missing_a_declared_node_is_still_selected_and_never_refused`]
    /// is the arm.
    ///
    /// # Why not "at least one COMPLETE anchor"
    ///
    /// Because a rank whose every node VOIDED is exactly the rank an operator
    /// most needs named. Its checkpoint restores no state, but it carries the
    /// skip markers that say WHY, and dropping it from the set drops that ring
    /// from the capture's ring-to-rank map — so the declining rank disappears
    /// from the manifest instead of being reported in it, which is the evidence
    /// the retention keeps skips for.
    /// [`a_rank_whose_every_node_declined_is_still_selected_so_the_skip_is_reported`]
    /// is the arm.
    ///
    /// # What names a rank that contributes nothing
    ///
    /// The SHORTFALL, not this predicate. A rank holding no checkpoint at or
    /// above the capture's floor contributes nothing, and
    /// [`select_one_ring`](Self::select_one_ring) reports it BY RANK with the
    /// reason, so a capture is never written with a hole nothing accounts for.
    /// The ring UNIVERSE is read off the DECLARED table for the same purpose: a
    /// rank the byte ceiling emptied holds nothing and is still named.
    fn checkpoint_is_complete(checkpoint: &Checkpoint) -> bool {
        !checkpoint.anchors.is_empty()
    }

    /// [`select`](Self::select)'s rule over ONE ring.
    fn select_one_ring(
        &self,
        ring: &str,
        floor_ns: u64,
        deadline_ns: u64,
        capture_seq: u64,
    ) -> Result<SelectedAnchor, NoAnchorReason> {
        let mut held = 0usize;
        let mut complete_held = 0usize;
        let mut oldest_candidate: Option<&Checkpoint> = None;
        let mut best: Option<&Checkpoint> = None;
        for c in self.checkpoints.iter().filter(|c| c.ring == ring) {
            held += 1;
            if !Self::checkpoint_is_complete(c) {
                continue;
            }
            complete_held += 1;
            if c.taken_at_ns < floor_ns {
                continue;
            }
            if oldest_candidate.is_none() {
                oldest_candidate = Some(c);
            }
            if c.taken_at_ns <= deadline_ns {
                // Ascending, so a later match is strictly newer.
                best = Some(c);
            }
        }
        if held == 0 || complete_held == 0 {
            // WHICH absence: a rank that never anchored anything a resume could
            // apply, or one whose anchors the ceiling took. Different facts,
            // different remedies, and the Q8 stamp prints them as two different
            // sentences.
            return Err(if self.ceiling_refusals > 0 && held == 0 {
                NoAnchorReason::RetentionCeilingExhausted
            } else {
                NoAnchorReason::NothingRetained
            });
        }
        match (best, oldest_candidate) {
            (Some(c), _) => Ok(SelectedAnchor {
                checkpoint: c.clone(),
                fit: AnchorFit::CoversTheClaimedWindow,
                capture_seq,
            }),
            (None, Some(c)) => Ok(SelectedAnchor {
                checkpoint: c.clone(),
                fit: AnchorFit::NewerThanTheClaimedWindow,
                capture_seq,
            }),
            (None, None) => Err(NoAnchorReason::AllOlderThanTheFrames),
        }
    }

    /// The ONE reason a capture with no anchor for ANY rank reports.
    ///
    /// The worst of the per-ring reasons, on the precedence
    /// `RetentionCeilingExhausted` then `AllOlderThanTheFrames` then
    /// `NothingRetained`. The order is not cosmetic: the ceiling reason is the
    /// one a capture is REFUSED for, so reporting a milder reason for a run the
    /// ceiling emptied would write a bag the never-frames-only rule forbids.
    fn no_anchor_reason(&self, shortfall: &BTreeMap<String, MissingRank>) -> NoAnchorReason {
        if shortfall.is_empty() {
            return if self.ceiling_refusals > 0 {
                NoAnchorReason::RetentionCeilingExhausted
            } else {
                NoAnchorReason::NothingRetained
            };
        }
        if shortfall
            .values()
            .any(|m| m.reason == NoAnchorReason::RetentionCeilingExhausted)
        {
            NoAnchorReason::RetentionCeilingExhausted
        } else if shortfall
            .values()
            .any(|m| m.reason == NoAnchorReason::AllOlderThanTheFrames)
        {
            NoAnchorReason::AllOlderThanTheFrames
        } else {
            NoAnchorReason::NothingRetained
        }
    }

    /// Drop everything. Called at teardown, once nothing can want it.
    pub(crate) fn clear(&mut self) {
        self.checkpoints.clear();
        self.bytes = 0;
    }
}

/// One anchor, reassembled and ready to retain.
#[derive(Debug, Clone)]
pub(crate) struct HarvestedAnchor {
    /// The run it belongs to.
    pub run_id: u64,
    /// The anchor step.
    pub step: u64,
    /// Index into the ring's node-identity table.
    pub node_idx: u32,
    /// The ring's SHM name.
    pub ring: String,
    /// The node id, resolved through the ring's manifest.
    pub node: Option<String>,
    /// Complete or Skipped.
    pub kind: AnchorKind,
    /// The raw records, in part order.
    pub records: Vec<StateRecord>,
}

/// The per-ring half of the harvest: whose records make ONE anchor.
///
/// It TRAVELS WITH ITS RING (a field on `SendStateRing`), which is what makes
/// the harvest impossible for a mode to forget: the recorder drains the ring
/// while it owns it and the writer thread drains it afterwards, and the
/// harvester goes wherever the ring goes rather than being wired up twice.
///
/// The completeness rule is [`StateAssembler`]'s and only ever
/// [`StateAssembler`]'s — this type buffers raw records alongside it and hands
/// them over when the assembler says an anchor closed.
pub(crate) struct AnchorHarvester {
    assembler: StateAssembler,
    /// Records buffered per `(run_id, step, node_idx)`, in arrival order.
    open: BTreeMap<(u64, u64, u32), Vec<StateRecord>>,
    /// Bytes those buffers hold.
    open_bytes: usize,
    /// The ceiling on them. An anchor bigger than this is ABANDONED rather than
    /// held, so a giant-state robot's captures are refused by design
    /// instead of the recorder becoming the process
    /// that runs the machine out of memory.
    max_open_bytes: usize,
    /// Streams the assembler has VOIDED. It never re-opens one, so buffering
    /// their surviving records would grow without bound for no possible result.
    voided: BTreeSet<(u64, u64, u32)>,
    /// Lifetime counters (Principle #3).
    torn: u64,
    malformed: u64,
    ceiling_refused: u64,
    /// The largest PARTIAL buffer a refused anchor had
    /// already accumulated when the ceiling took it, in bytes.
    ///
    /// # What it is FOR, and why it is a floor rather than a size
    ///
    /// The never-frames-only refusal has to name a knob value, and its first source is
    /// the measured generation. On a robot whose anchors never complete there is
    /// no such measurement — and rendering the remedy from an absent one printed
    /// `…ANCHOR_MAX_MB=0`, a confidently wrong number on exactly the robot that
    /// needs the right one. This is the fallback evidence: an anchor that had
    /// buffered N bytes before being refused is at least N bytes, so N is a true
    /// FLOOR on what the plane would have to hold.
    ///
    /// STICKY MAX, and ZERO means "nothing observed" rather than "zero bytes":
    /// the commonest refusal is a new anchor's FIRST record arriving while a
    /// sibling holds the whole global budget, which contributes nothing of its
    /// own. That case is genuinely unmeasured and must reach the arm that names
    /// the knob with NO number — never one that prints a zero.
    max_refused_partial_bytes: usize,
}

impl AnchorHarvester {
    /// A harvester paired with its ring's OPEN policy — `armed` for a mid-run
    /// attach (discard the partial head anchor the live cursor landed inside),
    /// `passthrough` for a stream read from its start.
    ///
    /// Paired rather than chosen, for the reason `StateAnchorLedger::for_open`
    /// states: an assembler judging a mid-run stream as if it began at part 0
    /// cries `PartOutOfOrder` on every attach.
    pub(crate) fn for_open(attached_mid_run: bool, max_open_bytes: usize) -> Self {
        Self {
            assembler: if attached_mid_run {
                StateAssembler::armed()
            } else {
                StateAssembler::passthrough()
            },
            open: BTreeMap::new(),
            open_bytes: 0,
            max_open_bytes,
            voided: BTreeSet::new(),
            torn: 0,
            malformed: 0,
            ceiling_refused: 0,
            max_refused_partial_bytes: 0,
        }
    }

    /// Anchors this harvester reported TORN, lifetime.
    pub(crate) fn torn(&self) -> u64 {
        self.torn
    }

    /// Records this harvester could not key, lifetime.
    pub(crate) fn malformed(&self) -> u64 {
        self.malformed
    }

    /// Anchors the IN-FLIGHT ceiling refused, lifetime.
    ///
    /// Counted ONCE per anchor, whichever way the ceiling bit — see `buffer`.
    /// Not spelled `abandoned_too_large`, and not counting only the anchors
    /// that had bytes to abandon, because that is wrong on both
    /// halves: the commonest refusal is a SMALL anchor whose first record
    /// arrives while a SIBLING holds the whole budget, so it is neither too
    /// large nor abandoned — nothing of it was ever held — and it would go
    /// uncounted while being refused just as permanently.
    pub(crate) fn ceiling_refused(&self) -> u64 {
        self.ceiling_refused
    }

    /// The largest partial buffer a refused anchor held.
    /// `0` means NOTHING was observed — see the field docs.
    pub(crate) fn max_refused_partial_bytes(&self) -> u64 {
        self.max_refused_partial_bytes as u64
    }

    /// Bytes currently buffered for anchors still in flight.
    pub(crate) fn open_bytes(&self) -> usize {
        self.open_bytes
    }

    /// Feed one raw 512-byte record. Returns an anchor when one CLOSED.
    ///
    /// A short slice is refused rather than padded: a record is a fixed-width
    /// format contract, and inventing the tail of one is how a reader serves a
    /// blob nobody wrote.
    pub(crate) fn feed(&mut self, record: &[u8]) -> Option<HarvestedAnchor> {
        let Ok(fixed) = <&StateRecord>::try_from(record) else {
            self.malformed += 1;
            return None;
        };
        let header = StateRecordHeader::from_bytes(
            record[..STATE_RECORD_HEADER_SIZE]
                .try_into()
                .expect("a 512-byte record always holds a 40-byte header"),
        );
        let key = (header.run_id, header.step, header.node_idx);

        // JUDGE FIRST, then buffer only what the assembler ACCEPTED.
        //
        // The order matters. An
        // ARMED harvester — a mid-run attach, whose cursor lands inside somebody
        // else's anchor — DISCARDS every record until the first `part == 0`
        // (`StateAssembler::feed`). Buffering before asking meant those discarded
        // tail records entered `open` under a key that could never complete, so
        // they held their bytes until `finish` and could exhaust the in-flight
        // ceiling, abandoning the FIRST real anchor after the attach — Principle
        // #11, and on exactly the path the arming exists to make safe.
        //
        // Acceptance is OBSERVED, not re-derived: `discarded` is the assembler's
        // own counter, so a change to its arming policy is followed here rather
        // than duplicated (the two-copies drift class). Feeding first is safe
        // for the completing record because the buffer is filled BELOW, before
        // the event is matched.
        let unkeyable = header.validate().is_err();
        let voided = self.voided.contains(&key);
        let discarded_before = self.assembler.discarded();
        let event = self.assembler.feed(record);
        let discarded_here = self.assembler.discarded() != discarded_before;
        if !unkeyable && !voided && !discarded_here {
            self.buffer(key, fixed);
        }
        let event = event?;
        match event {
            StateAnchorEvent::Complete {
                run_id,
                step,
                node_idx,
                ..
            } => {
                let key = (run_id, step, node_idx);
                let records = self.take(&key)?;
                Some(HarvestedAnchor {
                    run_id,
                    step,
                    node_idx,
                    ring: String::new(),
                    node: None,
                    kind: AnchorKind::Complete,
                    records,
                })
            }
            StateAnchorEvent::Skipped {
                run_id,
                step,
                node_idx,
                ..
            } => {
                let key = (run_id, step, node_idx);
                let mut records = self.take(&key)?;
                // ONLY the skip record. A skip is AUTHORITATIVE over anything in
                // flight — `StateAssembler::feed` drops the partial anchor when
                // one arrives — so the chunks buffered before it carry no
                // information a reader may use, and retaining them would spend a
                // checkpoint's ceiling on bytes the reader's own assembler
                // discards a second time (the same class as the armed
                // tail above). The skip is the record just buffered, so it is the
                // LAST one.
                let skip = records.pop();
                records.clear();
                records.extend(skip);
                Some(HarvestedAnchor {
                    run_id,
                    step,
                    node_idx,
                    ring: String::new(),
                    node: None,
                    kind: AnchorKind::Skipped,
                    records,
                })
            }
            StateAnchorEvent::Torn {
                run_id,
                step,
                node_idx,
                ..
            } => {
                self.torn += 1;
                let key = (run_id, step, node_idx);
                self.drop_open(&key);
                // The assembler VOIDS the stream and swallows its survivors, so
                // nothing further for this key can ever complete.
                self.voided.insert(key);
                None
            }
            StateAnchorEvent::SkipAfterComplete {
                run_id,
                step,
                node_idx,
                ..
            } => {
                // The Complete already won and already took its records; this
                // skip's own record is the only thing buffered under the key.
                self.drop_open(&(run_id, step, node_idx));
                None
            }
            StateAnchorEvent::Malformed { .. } => {
                self.malformed += 1;
                None
            }
        }
    }

    /// Close the stream: report what is still open as TORN and free it.
    ///
    /// Called when a ring is retired, so an anchor that was mid-flight when the
    /// producer died is counted rather than silently forgotten.
    pub(crate) fn finish(&mut self) {
        let torn = self.assembler.finish().len() as u64;
        self.torn += torn;
        self.open.clear();
        self.open_bytes = 0;
    }

    /// Buffer one record. `false` when the ceiling refused it.
    fn buffer(&mut self, key: (u64, u64, u32), record: &StateRecord) -> bool {
        const REC: usize = STATE_RECORD_SIZE as usize;
        if self.open_bytes + REC > self.max_open_bytes {
            // ABANDON this stream rather than hold a partial one forever: the
            // assembler will still judge it, and a partial buffer that can never
            // complete is memory spent for nothing.
            //
            // Counted whether or not anything was held. The
            // budget is GLOBAL across the keys in flight, so the commonest
            // refusal is a new anchor's FIRST record arriving while a sibling
            // holds the whole budget: `drop_open` finds nothing to free and
            // answers `false`, yet the key is VOIDED below and can never
            // complete — refused exactly as permanently as one whose partial
            // buffer was dropped. Counting only the second kind left the first
            // invisible on every axis: the ring-retirement log under-reported
            // it, and `AnchorWindow::select` — which reads this counter through
            // `note_ceiling_refusal` to tell "the run never anchored" from "the
            // ceiling took the anchors" — answered `no_anchor_retained`, sending
            // the operator to the arm gate when the answer was
            // `CERULION_FLASHBACK_ANCHOR_MAX_MB`.
            //
            // EXACTLY ONCE per anchor, structurally rather than by arithmetic:
            // the key is voided in the same breath and `feed` does not call this
            // for a voided key, so a multi-record anchor cannot be counted again
            // on its next part.
            // The refused anchor's OWN buffered bytes, read
            // BEFORE `drop_open` frees them — the fallback evidence the remedy
            // falls back to when no whole generation has ever been measured.
            let own_bytes = self.open.get(&key).map_or(0, |records| records.len() * REC);
            self.max_refused_partial_bytes = self.max_refused_partial_bytes.max(own_bytes);
            let held_a_partial_buffer = self.drop_open(&key);
            self.ceiling_refused += 1;
            self.voided.insert(key);
            tracing::debug!(
                open_bytes = self.open_bytes,
                max_open_bytes = self.max_open_bytes,
                step = key.1,
                node_idx = key.2,
                held_a_partial_buffer,
                "flashback anchors: an in-flight anchor was refused by the \
                 retention's buffer ceiling — this capture cannot resim that node and, if \
                 the ceiling took every anchor, will be REFUSED"
            );
            return false;
        }
        self.open_bytes += REC;
        self.open.entry(key).or_default().push(*record);
        true
    }

    /// Take a closed stream's records.
    fn take(&mut self, key: &(u64, u64, u32)) -> Option<Vec<StateRecord>> {
        let records = self.open.remove(key)?;
        self.open_bytes -= records.len() * STATE_RECORD_SIZE as usize;
        Some(records)
    }

    /// Free a stream's buffer. `true` when there was one.
    fn drop_open(&mut self, key: &(u64, u64, u32)) -> bool {
        match self.open.remove(key) {
            Some(records) => {
                self.open_bytes -= records.len() * STATE_RECORD_SIZE as usize;
                true
            }
            None => false,
        }
    }
}

#[cfg(test)]
mod tests {
    /// The capture number every arm below selects under, unless the arm is
    /// ABOUT the number. A non-zero literal so a stamp that was never written
    /// reads as 0 and fails rather than matching by luck.
    const CAPTURE: u64 = 3;

    use super::*;
    use cerulion_core::state::SkipCause;
    use cerulion_core::state_ring::{
        encode_record, StateChunker, RECORD_KIND_SKIP_V2, STATE_RECORD_FORMAT_VERSION,
    };

    const MS: u64 = 1_000_000;
    const REC: usize = STATE_RECORD_SIZE as usize;

    /// The records the production chunker emits for one node's blob — never a
    /// hand-rolled framing, so a change to the record format fails here rather
    /// than producing a retention full of bytes no reader accepts.
    fn anchor_records(run_id: u64, step: u64, node_idx: u32, blob: &[u8]) -> Vec<StateRecord> {
        let mut out = Vec::new();
        let mut chunker = StateChunker::new(run_id, step, node_idx, 0);
        chunker.append(blob, &mut |r| out.push(*r));
        chunker.finish(&mut |r| out.push(*r));
        out
    }

    /// [`anchor_records`] with the producer RANK stamped into every record, for
    /// the oracles that read the rank back out through [`producer_rank`].
    fn anchor_records_ranked(
        run_id: u64,
        step: u64,
        node_idx: u32,
        rank: u32,
        blob: &[u8],
    ) -> Vec<StateRecord> {
        let mut out = Vec::new();
        let mut chunker = StateChunker::new(run_id, step, node_idx, rank);
        chunker.append(blob, &mut |r| out.push(*r));
        chunker.finish(&mut |r| out.push(*r));
        out
    }

    /// One rank's anchor: its own ring, its own rank stamped in the records.
    fn rank_anchor(
        rank: u32,
        run_id: u64,
        step: u64,
        node_idx: u32,
        blob: &[u8],
    ) -> HarvestedAnchor {
        HarvestedAnchor {
            run_id,
            step,
            node_idx,
            ring: format!("rank{rank}"),
            node: Some(format!("r{rank}n{node_idx}")),
            kind: AnchorKind::Complete,
            records: anchor_records_ranked(run_id, step, node_idx, rank, blob),
        }
    }

    /// One rank's VOIDED anchor: its own ring, one skip record, no state.
    fn rank_skip(rank: u32, run_id: u64, step: u64, node_idx: u32) -> HarvestedAnchor {
        HarvestedAnchor {
            run_id,
            step,
            node_idx,
            ring: format!("rank{rank}"),
            node: Some(format!("r{rank}n{node_idx}")),
            kind: AnchorKind::Skipped,
            records: vec![skip_record(run_id, step, node_idx)],
        }
    }

    fn skip_record(run_id: u64, step: u64, node_idx: u32) -> StateRecord {
        let detail = b"the recorder has not drained enough of the state ring";
        let mut payload = Vec::with_capacity(4 + detail.len());
        payload.extend_from_slice(&SkipCause::RecorderBehind.as_wire().to_le_bytes());
        payload.extend_from_slice(detail);
        encode_record(
            &StateRecordHeader {
                run_id,
                step,
                node_idx,
                part: 0,
                kind: RECORD_KIND_SKIP_V2,
                len: payload.len() as u32,
                rank: 0,
                format_version: STATE_RECORD_FORMAT_VERSION,
            },
            &payload,
        )
    }

    fn harvested(run_id: u64, step: u64, node_idx: u32, blob: &[u8]) -> HarvestedAnchor {
        harvested_on(&format!("ring{node_idx}"), run_id, step, node_idx, blob)
    }

    /// [`harvested`] with the RING named, for the cases where the ring is the
    /// point: `harvested` gives each node index its own ring, which makes every
    /// node a separate RANK now that the checkpoint key carries the ring.
    fn harvested_on(
        ring: &str,
        run_id: u64,
        step: u64,
        node_idx: u32,
        blob: &[u8],
    ) -> HarvestedAnchor {
        HarvestedAnchor {
            run_id,
            step,
            node_idx,
            ring: ring.to_string(),
            node: Some(format!("n{node_idx}")),
            kind: AnchorKind::Complete,
            records: anchor_records(run_id, step, node_idx, blob),
        }
    }

    /// THE headline: the retention holds the NEWEST checkpoints, which is the
    /// inversion this module exists to close. A fill-then-skip ring keeps the
    /// oldest; this keeps the last `⌈W/C⌉ + 1`.
    #[test]
    fn the_retention_holds_the_newest_checkpoints_not_the_oldest() {
        // A 30 s span at a 15 s cadence: two checkpoints inside it, plus the
        // carry.
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        for i in 0..8u64 {
            let step = i * 1000;
            w.admit(i * 15_000 * MS, harvested(7, step, 0, &[i as u8; 64]));
            w.evict(i * 15_000 * MS, None);
        }
        // At t = 105 s the horizon is 75 s, so checkpoints at 0/15/30/45/60 s
        // have aged; the newest of those (60 s) is the CARRY.
        let steps: Vec<u64> = w.checkpoints.iter().map(|c| c.step).collect();
        assert_eq!(
            steps,
            vec![4000, 5000, 6000, 7000],
            "the retention must hold the NEWEST checkpoints (75/90/105 s) plus the carry (60 s) \
             — a fill-then-skip ring holds 0/15/30 s instead, which is the defect this closes"
        );
        // …and it is `⌈W/C⌉ + 1`, derived rather than configured.
        assert_eq!(w.checkpoints(), 30_000usize.div_ceil(15_000) + 1 + 1);
        assert_eq!(w.aged(), 4);
        assert_eq!(w.truncated(), 0);
    }

    /// A checkpoint is stamped ONCE, by its first record. A slower rank's
    /// sibling anchor joining later must not drag the stamp forward past the
    /// deadline a selection compares it against.
    #[test]
    fn a_late_sibling_anchor_does_not_drag_its_checkpoints_stamp_forward() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        // ONE ring, two nodes: the sibling is a node of the SAME rank, which is
        // what a late sibling is. Two RINGS at one step are two checkpoints now
        // that the key carries the ring, and that is a different fact.
        w.admit(10_000 * MS, harvested_on("ring0", 1, 500, 0, &[1; 16]));
        w.admit(14_000 * MS, harvested_on("ring0", 1, 500, 1, &[2; 16]));
        assert_eq!(w.checkpoints(), 1, "one ring at one step is one checkpoint");
        let c = w.checkpoints.front().expect("held");
        assert_eq!(c.taken_at_ns, 10_000 * MS);
        assert_eq!(c.anchors.len(), 2);
        assert_eq!(c.complete_anchors(), 2);

        // The discriminator: a deadline BETWEEN the two admissions still selects
        // it. Under a last-writer stamp it would be refused as too new.
        let selection = w.select(0, 12_000 * MS, CAPTURE).expect("selected");
        let (picked, fit) = selection.sole();
        assert_eq!(picked.step, 500);
        assert_eq!(fit, AnchorFit::CoversTheClaimedWindow);
    }

    /// The selection rule, all three arms, against hand-computed instants.
    #[test]
    fn selection_prefers_the_newest_inside_the_claimed_window_and_never_below_the_floor() {
        let mut w = AnchorWindow::new(60_000 * MS, 1 << 30);
        for (t, step) in [(5_000u64, 100u64), (20_000, 200), (40_000, 300)] {
            w.admit(t * MS, harvested(1, step, 0, &[0; 8]));
        }

        // A capture at T = 50 s, post window 15 s ⇒ deadline 35 s, floor 20 s.
        let selection = w
            .select(20_000 * MS, 35_000 * MS, CAPTURE)
            .expect("selected");
        let (picked, fit) = selection.sole();
        assert_eq!(
            picked.step, 200,
            "the NEWEST at or before the deadline — 300 is too new, 100 is below the floor"
        );
        assert_eq!(fit, AnchorFit::CoversTheClaimedWindow);

        // Raise the floor past every checkpoint at or before the deadline: the
        // only candidate left is NEWER than the claimed window, and is served
        // WITH that fact rather than silently.
        let selection = w
            .select(38_000 * MS, 35_000 * MS, CAPTURE)
            .expect("selected");
        let (picked, fit) = selection.sole();
        assert_eq!(picked.step, 300);
        assert_eq!(
            fit,
            AnchorFit::NewerThanTheClaimedWindow,
            "a resume from here covers less than the capture's frames do, and must say so"
        );

        // A floor above everything: no candidate at all. Resuming from a
        // checkpoint whose forward frames are not in the bag would diverge for a
        // reason the recording caused.
        assert_eq!(
            w.select(45_000 * MS, 46_000 * MS, CAPTURE).err(),
            Some(NoAnchorReason::AllOlderThanTheFrames)
        );
        // …and an empty retention is a DIFFERENT reason, because the remedies
        // differ: one is "no anchor was taken", the other "the window was too
        // short to reach one".
        let empty = AnchorWindow::new(30_000 * MS, 1 << 30);
        assert_eq!(
            empty.select(0, 0, CAPTURE).err(),
            Some(NoAnchorReason::NothingRetained)
        );
    }

    /// An active capture's checkpoint is not evicted out from under it — the
    /// anchor twin of the frame window's own pre-window pin.
    #[test]
    fn an_active_captures_checkpoint_survives_its_post_window() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        w.admit(0, harvested(1, 10, 0, &[9; 32]));
        w.admit(20_000 * MS, harvested(1, 20, 0, &[9; 32]));

        // A trigger at t = 30 s: floor 0.
        let floor = 0u64;
        // Fifteen seconds later the bare horizon is 15 s and would have taken the
        // step-10 checkpoint the capture promised.
        let report = w.evict(45_000 * MS, Some(floor));
        assert_eq!(report, AnchorEvictionReport::default());
        assert_eq!(w.oldest_ns(), Some(0));

        // CONTROL, same body: with no capture active the same instant evicts
        // down to the carry. Without it the arm above passes against a retention
        // that never evicts at all.
        let report = w.evict(45_000 * MS, None);
        assert_eq!(
            report.aged, 0,
            "the step-10 checkpoint is the ONLY one past the horizon, so it is the CARRY"
        );
        assert_eq!(w.checkpoints(), 2);

        // …and it goes only once a SECOND checkpoint has aged past the horizon
        // and can carry in its place. At t = 60 s the horizon is 30 s, so both
        // held checkpoints are past it: the older goes, the newer carries.
        let report = w.evict(60_000 * MS, None);
        assert_eq!(report.aged, 1);
        assert_eq!(
            w.oldest_ns(),
            Some(20_000 * MS),
            "the NEWEST of the aged-out checkpoints stays as the carry"
        );
    }

    /// The byte ceiling is a CEILING — a capture cannot suspend it — and what it
    /// takes from a capture is reported as a truncation, and by design a capture
    /// whose ANCHOR is what it took is REFUSED rather than written.
    #[test]
    fn the_byte_ceiling_bites_through_a_capture_and_says_that_it_did() {
        // One 64-byte blob is exactly one record.
        let one = REC as u64;
        let mut w = AnchorWindow::new(30_000 * MS, one * 2);
        w.admit(0, harvested(1, 10, 0, &[1; 64]));
        w.admit(1_000 * MS, harvested(1, 20, 0, &[2; 64]));
        assert_eq!(
            w.evict(1_000 * MS, Some(0)),
            AnchorEvictionReport::default()
        );
        assert_eq!(w.bytes(), REC * 2);

        w.admit(2_000 * MS, harvested(1, 30, 0, &[3; 64]));
        let report = w.evict(2_000 * MS, Some(0));
        assert_eq!(
            report,
            AnchorEvictionReport {
                aged: 0,
                truncated: 1
            },
            "a byte-cap eviction inside a capture's floor is a TRUNCATION — the capture it \
             belonged to has lost coverage it claimed"
        );
        assert_eq!(w.truncated(), 1);

        // ANTI-TAUTOLOGY: with no capture active the same overflow is ordinary
        // ageing, so the two counters are chosen by the floor rather than by
        // which loop dropped the checkpoint.
        w.admit(3_000 * MS, harvested(1, 40, 0, &[4; 64]));
        let report = w.evict(3_000 * MS, None);
        assert_eq!(
            report,
            AnchorEvictionReport {
                aged: 1,
                truncated: 0
            }
        );
    }

    /// A ceiling below a single checkpoint empties the retention rather than
    /// looping — the configuration an operator asked for, honoured, not hung on.
    #[test]
    fn a_ceiling_below_one_checkpoint_empties_the_retention_rather_than_looping() {
        let mut w = AnchorWindow::new(30_000 * MS, 8);
        w.admit(0, harvested(1, 10, 0, &[0; 4096]));
        let report = w.evict(0, None);
        assert_eq!(report.aged, 1);
        assert_eq!(w.checkpoints(), 0);
        assert_eq!(w.bytes(), 0);
        assert_eq!(w.evict(0, None), AnchorEvictionReport::default());
        // And it says WHY it is empty. This arm asserted
        // `NothingRetained` until the ceiling got its own reason — which is the
        // defect that fix closes, since a run whose anchors the ceiling ate reads
        // identically to one that never anchored.
        assert_eq!(
            w.select(0, 0, CAPTURE).err(),
            Some(NoAnchorReason::RetentionCeilingExhausted)
        );
    }

    /// The clock starting below the span must not underflow the horizon into
    /// evicting everything on the recorder's first pass.
    #[test]
    fn a_clock_below_the_span_does_not_underflow_into_evicting_everything() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        w.admit(0, harvested(1, 10, 0, &[0; 8]));
        w.admit(100 * MS, harvested(1, 20, 0, &[0; 8]));
        assert_eq!(w.evict(100 * MS, None), AnchorEvictionReport::default());
        assert_eq!(w.checkpoints(), 2);
    }

    /// Two runs never merge into one checkpoint: `plan_restore` refuses a step
    /// served by two runs, so a retention that keyed on the step alone would
    /// build exactly the bag a resim cannot use.
    #[test]
    fn two_runs_at_the_same_step_are_two_checkpoints() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        w.admit(0, harvested(1, 500, 0, &[1; 8]));
        w.admit(1_000 * MS, harvested(2, 500, 0, &[2; 8]));
        assert_eq!(w.checkpoints(), 2);
        let runs: Vec<u64> = w.checkpoints.iter().map(|c| c.run_id).collect();
        assert_eq!(runs, vec![1, 2]);
    }

    /// The harvester hands over the records the production chunker wrote, in
    /// part order, byte-for-byte — which is what lets a capture's
    /// `__cerulion/state` bytes be identical to a recording's.
    #[test]
    fn the_harvester_yields_a_whole_anchors_records_byte_for_byte() {
        // Three records' worth: 472-byte payloads, so the last is short.
        let blob = vec![0xC5u8; 1000];
        let expected = anchor_records(3, 42, 1, &blob);
        assert_eq!(expected.len(), 3, "precondition: this blob really chunks");

        let mut h = AnchorHarvester::for_open(false, 1 << 20);
        let mut got = None;
        for (i, rec) in expected.iter().enumerate() {
            let out = h.feed(rec);
            if i + 1 < expected.len() {
                assert!(out.is_none(), "an anchor closes on its FINAL record only");
            } else {
                got = out;
            }
        }
        let anchor = got.expect("the final record closes the anchor");
        assert_eq!(anchor.run_id, 3);
        assert_eq!(anchor.step, 42);
        assert_eq!(anchor.node_idx, 1);
        assert_eq!(anchor.kind, AnchorKind::Complete);
        assert_eq!(
            anchor.records, expected,
            "the retained records must be the ring's own, unmodified"
        );
        assert_eq!(h.open_bytes(), 0, "a closed anchor frees its buffer");
        assert_eq!(h.torn(), 0);
    }

    /// A SKIP is retained, because a capture that carries it tells a reader WHY
    /// a node has no state where a capture that dropped it says only that one is
    /// missing.
    #[test]
    fn a_skip_is_retained_as_its_own_one_record_anchor() {
        let mut h = AnchorHarvester::for_open(false, 1 << 20);
        let rec = skip_record(3, 42, 2);
        let anchor = h.feed(&rec).expect("a skip closes immediately");
        assert_eq!(anchor.kind, AnchorKind::Skipped);
        assert_eq!(anchor.records, vec![rec]);
        // …and it is ONE record: a skip is self-contained, so a capture that
        // carries it costs a checkpoint nothing while telling a reader exactly
        // why that node has no state.
        assert_eq!(anchor.records.len(), 1);
    }

    /// A TORN anchor is never retained, and its stream is never buffered again:
    /// the assembler voids it permanently, so holding its survivors would grow
    /// memory for a result that can never arrive.
    #[test]
    fn a_torn_anchor_is_refused_and_its_stream_stops_costing_memory() {
        let records = anchor_records(3, 42, 1, &vec![0xAB; 2000]);
        assert!(records.len() >= 4, "precondition: several parts");

        let mut h = AnchorHarvester::for_open(false, 1 << 20);
        h.feed(&records[0]).expect_none_or_panic();
        // Skip part 1 entirely: part 2 arrives out of order and tears the stream.
        assert!(h.feed(&records[2]).is_none());
        assert_eq!(h.torn(), 1);
        assert_eq!(h.open_bytes(), 0, "a torn stream frees its buffer at once");

        // Its surviving records are swallowed AND cost nothing.
        for rec in &records[3..] {
            assert!(h.feed(rec).is_none());
        }
        assert_eq!(
            h.open_bytes(),
            0,
            "a voided stream must never re-buffer — the assembler can never close it"
        );
    }

    /// An anchor bigger than the retention's in-flight ceiling is ABANDONED, not
    /// held: a giant-state robot has its captures refused by design
    /// rather than making the recorder the process that runs the machine out of
    /// memory.
    #[test]
    fn an_oversized_anchor_is_abandoned_rather_than_held() {
        // Room for two records only.
        let mut h = AnchorHarvester::for_open(false, REC * 2);
        let records = anchor_records(3, 42, 1, &vec![7u8; 2000]);
        assert!(records.len() > 2);
        for rec in &records {
            assert!(h.feed(rec).is_none(), "it must never complete");
        }
        assert_eq!(h.ceiling_refused(), 1);
        assert_eq!(h.open_bytes(), 0);

        // …and the harvester still works afterwards: the abandonment is scoped
        // to the stream that overran, not to the ring.
        let small = anchor_records(3, 43, 1, &[1u8; 8]);
        let anchor = h.feed(&small[0]).expect("a later small anchor still lands");
        assert_eq!(anchor.step, 43);
    }

    /// A NEW anchor refused at a FULL ceiling is counted, even
    /// though it had nothing buffered to abandon.
    ///
    /// The in-flight budget is GLOBAL across the keys in flight, so this is the
    /// commonest way the ceiling bites on a real robot: one node's anchor is
    /// mid-flight and holding the budget when a SIBLING's first record arrives.
    /// `drop_open` finds no partial buffer for the newcomer and answers `false`,
    /// and the old counter incremented only inside that `if` — so the sibling was
    /// voided (it can never complete) and refused (its state is not in any
    /// capture) while every counter said the ceiling had done nothing.
    ///
    /// That is not merely an under-count: `AnchorWindow::select` reads this
    /// number through `note_ceiling_refusal` to tell a run that never anchored
    /// from one whose anchors the ceiling took, so a capture on such a robot
    /// reported `no_anchor_retained` and sent its operator to the arm gate
    /// instead of to `CERULION_FLASHBACK_ANCHOR_MAX_MB`.
    ///
    /// The oracle is hand-written and each step is asserted apart, because the
    /// two refusal kinds land on the same counter and a single end-of-run total
    /// cannot say which of them moved it.
    #[test]
    fn a_new_anchor_refused_at_a_full_ceiling_is_counted_though_it_held_nothing() {
        // Room for two records; node 0's anchor needs three and node 1's two.
        let mut h = AnchorHarvester::for_open(false, REC * 2);
        let a = anchor_records(3, 42, 0, &vec![7u8; 1200]);
        let b = anchor_records(3, 42, 1, &vec![9u8; 600]);
        assert!(
            a.len() >= 3,
            "precondition: node 0 fills the budget and needs more"
        );
        assert!(
            b.len() >= 2,
            "precondition: node 1 is multi-part, so its refusal can repeat"
        );

        // Node 0 fills the budget EXACTLY, and closes nothing.
        assert!(h.feed(&a[0]).is_none());
        assert!(h.feed(&a[1]).is_none());
        assert_eq!(h.open_bytes(), REC * 2, "precondition: the budget is full");
        assert_eq!(h.ceiling_refused(), 0, "nothing has been refused yet");

        // Node 1's FIRST record: refused, with no partial buffer of its own.
        assert!(h.feed(&b[0]).is_none());
        assert_eq!(
            h.ceiling_refused(),
            1,
            "an anchor the ceiling refused is counted whether or not it had bytes to \
             abandon — when this reads 0, a capture that lost this node's \
             state blames a run that never anchored"
        );
        assert_eq!(
            h.open_bytes(),
            REC * 2,
            "the INCUMBENT's buffer is untouched: the refusal drops the newcomer, never \
             the anchor already paying for the budget"
        );

        // Node 1's later records ride the VOIDED guard, so the count is per
        // ANCHOR and not per refused record.
        for rec in &b[1..] {
            assert!(h.feed(rec).is_none());
        }
        assert_eq!(
            h.ceiling_refused(),
            1,
            "counted exactly ONCE per anchor: the key is voided at the first refusal and \
             `feed` never buffers a voided key again"
        );

        // …and the OTHER refusal kind still counts, on its own: node 0's next
        // record overruns the budget it is itself holding, which is the anchor
        // that really is too large.
        assert!(h.feed(&a[2]).is_none());
        assert_eq!(
            h.ceiling_refused(),
            2,
            "the two refusal kinds are both counted, and neither swallows the other"
        );
        assert_eq!(h.open_bytes(), 0, "the abandoned buffer is freed");
    }

    /// A short slice is refused rather than padded — a record is a fixed-width
    /// format contract, and inventing its tail is how a reader serves a blob
    /// nobody wrote.
    #[test]
    fn a_short_record_is_counted_malformed_and_never_padded() {
        let mut h = AnchorHarvester::for_open(false, 1 << 20);
        assert!(h.feed(&[0u8; 8]).is_none());
        assert_eq!(h.malformed(), 1);
        assert_eq!(h.open_bytes(), 0);
    }

    /// The MID-RUN attach: a harvester paired with `open_at_live` discards the
    /// partial head anchor its cursor landed inside, and serves the NEXT one —
    /// rather than reporting a late attach as corruption.
    #[test]
    fn a_mid_run_harvester_discards_the_partial_head_and_serves_the_next() {
        let head = anchor_records(3, 40, 0, &vec![1u8; 2000]);
        let next = anchor_records(3, 41, 0, &[2u8; 8]);
        assert!(head.len() >= 3);

        let mut h = AnchorHarvester::for_open(true, 1 << 20);
        // Land mid-blob: parts 1.. of the head anchor.
        for rec in &head[1..] {
            assert!(h.feed(rec).is_none());
        }
        assert_eq!(h.torn(), 0, "a late attach is NOT corruption");
        // The discarded tail must cost NOTHING. Buffering before
        // asking the assembler put those records in `open` under a key that can
        // never complete, so they held their bytes until `finish` and could
        // exhaust the in-flight ceiling — abandoning the first real anchor after
        // the attach, on the very path the arming exists to make safe.
        assert_eq!(
            h.open_bytes(),
            0,
            "records the assembler DISCARDED must never enter the retention's buffer"
        );
        let anchor = h.feed(&next[0]).expect("the next whole anchor is served");
        assert_eq!(anchor.step, 41);
        assert_eq!(anchor.records, next);

        // ANTI-TAUTOLOGY: a PASSTHROUGH harvester fed the same head reports it
        // torn — so the arm above is the arming, not a harvester that never
        // tears anything.
        let mut p = AnchorHarvester::for_open(false, 1 << 20);
        for rec in &head[1..] {
            p.feed(rec);
        }
        assert_eq!(p.torn(), 1);
    }

    /// At the ceiling: a mid-run attach whose discarded tail is
    /// LARGER than the in-flight ceiling must still serve the next whole anchor.
    ///
    /// This is the arm the leak actually broke. The isolated `open_bytes() == 0`
    /// check above says the bytes are not held; this says what holding them
    /// COSTS — a harvester that buffers the discarded tail hits its ceiling
    /// during records it was always going to throw away, voids the key, and then
    /// abandons the first real anchor after the attach. The recorder would report
    /// a healthy ring, no torn anchors, and no checkpoints at all.
    #[test]
    fn a_discarded_head_tail_larger_than_the_ceiling_cannot_starve_the_next_anchor() {
        let head = anchor_records(3, 40, 0, &vec![1u8; 4000]);
        let next = anchor_records(3, 41, 0, &[2u8; 8]);
        assert!(
            head.len() - 1 > 2,
            "precondition: the discarded tail really exceeds the ceiling"
        );

        let mut h = AnchorHarvester::for_open(true, REC * 2);
        for rec in &head[1..] {
            assert!(h.feed(rec).is_none());
        }
        assert_eq!(
            h.ceiling_refused(),
            0,
            "the ceiling was never consulted — a DISCARDED record is not buffered, so it \
             cannot refuse one"
        );

        let anchor = h
            .feed(&next[0])
            .expect("the first whole anchor after the attach must still be served");
        assert_eq!(anchor.records, next);
        assert_eq!(h.open_bytes(), 0);
    }

    /// Stage A: a SKIP declaring fewer than four payload bytes reports
    /// its cause UNRECOGNISED, never a cause read out of zero padding.
    ///
    /// A record is a fixed-size slot, so the four bytes after the header are
    /// always ADDRESSABLE — they are just not always the writer's. A read
    /// that ignored the length would decode a one-byte payload of `0x01` as `Contended`, which
    /// `plan_restore` renders into the sentence an operator reads about why
    /// their capture cannot be resumed: they would go looking for a node mutex
    /// that was never held.
    ///
    /// The boundary is pinned on BOTH sides in one body, so a fix that simply
    /// refused every short-ish record would fail the four-byte arm.
    #[test]
    fn a_skip_declaring_fewer_than_four_payload_bytes_reports_an_unrecognised_cause() {
        // The payload byte is `0x01` — `SkipCause::Contended`'s own wire value —
        // so a length-blind read produces a confident, WRONG, named cause rather
        // than something obviously broken.
        let short = encode_record(
            &StateRecordHeader {
                run_id: 1,
                step: 2,
                node_idx: 3,
                part: 0,
                kind: RECORD_KIND_SKIP_V2,
                len: 1,
                rank: 0,
                format_version: STATE_RECORD_FORMAT_VERSION,
            },
            &[SkipCause::Contended.as_wire() as u8],
        );
        let below = RetainedAnchor {
            ring: "r0".to_string(),
            node_idx: 3,
            node: Some("n3".to_string()),
            kind: AnchorKind::Skipped,
            records: Arc::new(vec![short]),
        };
        assert_eq!(
            below.skip_cause(),
            Some(SkipCause::Unrecognized(0)),
            "a cause word the writer never wrote must not be read as a cause"
        );

        // EXACTLY four bytes is the smallest LEGAL cause and must still decode —
        // the other side of the boundary.
        let exact = encode_record(
            &StateRecordHeader {
                run_id: 1,
                step: 2,
                node_idx: 3,
                part: 0,
                kind: RECORD_KIND_SKIP_V2,
                len: 4,
                rank: 0,
                format_version: STATE_RECORD_FORMAT_VERSION,
            },
            &SkipCause::Contended.as_wire().to_le_bytes(),
        );
        let at = RetainedAnchor {
            records: Arc::new(vec![exact]),
            ..below.clone()
        };
        assert_eq!(at.skip_cause(), Some(SkipCause::Contended));

        // And the ordinary shape — a cause plus its detail text — is unchanged.
        let full = RetainedAnchor {
            records: Arc::new(vec![skip_record(1, 2, 3)]),
            ..below
        };
        assert_eq!(full.skip_cause(), Some(SkipCause::RecorderBehind));
    }

    /// Second instance: a SKIP that lands on an anchor already IN
    /// FLIGHT keeps ONLY the skip record.
    ///
    /// `StateAssembler::feed` drops the partial anchor when a skip arrives, so
    /// the chunks buffered before it carry nothing a reader may use — a reader's
    /// own assembler discards them a second time. Retaining them would spend a
    /// checkpoint's ceiling, and its manifest's byte count, on bytes that mean
    /// nothing.
    #[test]
    fn a_skip_over_an_in_flight_anchor_keeps_only_the_skip_record() {
        let partial = anchor_records(3, 42, 2, &vec![7u8; 3000]);
        assert!(partial.len() >= 4, "precondition: several chunks in flight");

        let mut h = AnchorHarvester::for_open(false, 1 << 20);
        for rec in &partial[..partial.len() - 1] {
            assert!(h.feed(rec).is_none(), "the anchor is still open");
        }
        assert!(
            h.open_bytes() > REC,
            "precondition: the chunks really are held"
        );

        let skip = skip_record(3, 42, 2);
        let anchor = h.feed(&skip).expect("a skip closes immediately");
        assert_eq!(anchor.kind, AnchorKind::Skipped);
        assert_eq!(
            anchor.records,
            vec![skip],
            "a skipped anchor is ONE record — the voided chunks must not ride with it"
        );
        assert_eq!(h.open_bytes(), 0);
    }

    /// The queue stays ASCENDING even when a caller hands it a stamp that went
    /// backwards. Eviction and selection both walk from the front and stop at
    /// the first checkpoint new enough, so an out-of-order stamp evicts the
    /// wrong checkpoint and serves the wrong anchor — with every counter still
    /// adding up, which is what makes it worth enforcing rather than assuming.
    #[test]
    fn a_stamp_that_went_backwards_cannot_break_the_queues_order() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        w.admit(10_000 * MS, harvested(1, 10, 0, &[1; 8]));
        // A stamp BELOW the newest held: clamped, never inserted out of order.
        w.admit(2_000 * MS, harvested(1, 20, 0, &[2; 8]));
        let stamps: Vec<u64> = w.checkpoints.iter().map(|c| c.taken_at_ns).collect();
        assert_eq!(
            stamps,
            vec![10_000 * MS, 10_000 * MS],
            "the queue must stay ascending — the clamp is what makes the front-walking \
             eviction and selection sound"
        );
        assert!(stamps.windows(2).all(|w| w[0] <= w[1]));

        // …and the clamped checkpoint is still SELECTABLE at its clamped stamp,
        // so the hardening costs no anchor.
        let selection = w.select(0, 10_000 * MS, CAPTURE).expect("selected");
        let (picked, _) = selection.sole();
        assert_eq!(picked.step, 20);
    }

    /// A retention emptied by its BYTE CEILING says so, rather
    /// than reporting the same "nothing retained" as a run that never anchored.
    ///
    /// The two are different facts with opposite remedies — one says check the
    /// arm gate and the cadence, the other says give the plane more room — and
    /// under the never-frames-only rule the second also decides whether the capture is
    /// REFUSED, so collapsing them points an operator at exactly the wrong place
    /// AND changes whether a bag exists.
    #[test]
    fn a_retention_emptied_by_its_byte_ceiling_says_so() {
        // A ceiling below one checkpoint: it is admitted and then evicted, which
        // is how a big-state robot's retention really empties.
        let mut w = AnchorWindow::new(30_000 * MS, 0);
        w.admit(0, harvested(1, 10, 0, &[1; 8]));
        w.evict(0, None);
        assert_eq!(w.checkpoints(), 0);
        assert!(
            w.ceiling_refusals() > 0,
            "precondition: the ceiling really bit"
        );
        assert_eq!(
            w.select(0, u64::MAX, CAPTURE).err(),
            Some(NoAnchorReason::RetentionCeilingExhausted),
            "an emptied-by-the-ceiling retention must not report `no_anchor_retained`"
        );

        // ANTI-TAUTOLOGY: a retention that never held anything still reports
        // NothingRetained, so the new reason tracks the CEILING rather than
        // emptiness.
        let never = AnchorWindow::new(30_000 * MS, 1 << 30);
        assert_eq!(
            never.select(0, u64::MAX, CAPTURE).err(),
            Some(NoAnchorReason::NothingRetained)
        );

        // ...and a retention that AGED its checkpoints out is not the ceiling
        // either: the two eviction rules are reported apart.
        let mut aged = AnchorWindow::new(1_000 * MS, 1 << 30);
        aged.admit(0, harvested(1, 10, 0, &[1; 8]));
        aged.admit(1, harvested(1, 20, 0, &[2; 8]));
        aged.evict(60_000 * MS, None);
        assert_eq!(
            aged.ceiling_refusals(),
            0,
            "ageing is the retention working, not the ceiling biting"
        );
    }

    /// The other half of the same fact: an anchor the HARVESTER refused in flight
    /// never becomes a checkpoint, so the window's own eviction can never see it.
    /// The harvest reports it, and ONE counter answers wherever the ceiling bit.
    #[test]
    fn an_in_flight_refusal_reported_by_the_harvest_also_names_the_ceiling() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        assert_eq!(
            w.select(0, u64::MAX, CAPTURE).err(),
            Some(NoAnchorReason::NothingRetained),
            "precondition: nothing has been refused yet"
        );
        w.note_ceiling_refusal(1);
        assert_eq!(
            w.select(0, u64::MAX, CAPTURE).err(),
            Some(NoAnchorReason::RetentionCeilingExhausted),
            "a big-state robot whose anchors never fit must be told it is the CEILING, not \
             told its run never anchored"
        );
        assert_eq!(w.ceiling_refusals(), 1);
    }

    /// By design the measured generation demand only ever rises,
    /// and it counts only COMPLETE anchors.
    ///
    /// Both halves are the safe direction, and neither is visible from a fixture
    /// that admits one checkpoint. A demand that could FALL would let a
    /// half-filled checkpoint shrink the reserve a full one needs — the reserve
    /// collapsing exactly when the state grew. A demand that counted SKIPS would
    /// be sized off records no resume can use.
    #[test]
    fn the_generation_demand_rises_only_and_counts_only_complete_anchors() {
        let mut w = AnchorWindow::new(30_000 * MS, u64::MAX);
        assert_eq!(w.max_generation_bytes(), None, "nothing measured yet");

        // A BIG checkpoint: four records at step 1.
        let mut big = harvested(1, 1, 0, &vec![0xAB; REC * 3]);
        assert!(
            big.records.len() >= 2,
            "precondition: the blob really chunks"
        );
        let big_bytes = (big.records.len() * REC) as u64;
        big.kind = AnchorKind::Complete;
        w.admit(1000, big);
        assert_eq!(w.max_generation_bytes(), Some(big_bytes));

        // A SMALLER checkpoint at a later step must NOT lower the demand.
        let mut small = harvested(1, 2, 0, &[0xCD]);
        small.kind = AnchorKind::Complete;
        w.admit(2000, small);
        assert_eq!(
            w.max_generation_bytes(),
            Some(big_bytes),
            "the demand is STICKY: a later, smaller checkpoint must not shrink the \
             reserve that the bigger one needs"
        );

        // A SKIP contributes nothing, however big the checkpoint it lands in.
        let mut skipped = harvested(1, 3, 0, &vec![0xEF; REC * 8]);
        skipped.kind = AnchorKind::Skipped;
        w.admit(3000, skipped);
        assert_eq!(
            w.max_generation_bytes(),
            Some(big_bytes),
            "a SKIP says why a node did not checkpoint — sizing the reserve off it \
             would size it off records no resume can use"
        );
    }

    /// Every no-anchor reason has its own wire spelling: a reader that cannot
    /// tell "nothing was captured" from "the window was too short" cannot act on
    /// either.
    #[test]
    fn the_no_anchor_reasons_are_spelled_apart() {
        assert_eq!(
            NoAnchorReason::NothingRetained.as_wire(),
            "no_anchor_retained"
        );
        assert_eq!(
            NoAnchorReason::AllOlderThanTheFrames.as_wire(),
            "anchors_older_than_the_frames"
        );
        assert_eq!(
            NoAnchorReason::RetentionCeilingExhausted.as_wire(),
            "anchors_dropped_by_the_byte_ceiling"
        );
        // All three spellings distinct — the whole point of having three.
        let wires = [
            NoAnchorReason::NothingRetained.as_wire(),
            NoAnchorReason::AllOlderThanTheFrames.as_wire(),
            NoAnchorReason::RetentionCeilingExhausted.as_wire(),
        ];
        let unique: std::collections::BTreeSet<&str> = wires.iter().copied().collect();
        assert_eq!(unique.len(), wires.len(), "{wires:?}");
    }

    /// Each no-anchor reason's REMEDY is its own sentence, written out here
    /// rather than read back off the function.
    ///
    /// The wire arm above proves the three IDENTIFIERS distinct and says nothing
    /// about the three sentences an operator actually reads, so two causes could
    /// render one remedy and the suite would be green. Measured: a mutant that
    /// gave `AllOlderThanTheFrames` the `NothingRetained` text survived the whole
    /// suite, and the sentence it substituted sends an operator to check whether
    /// a rank published a ring when what happened is that the window was too
    /// short for the checkpoints the rank did publish.
    ///
    /// Each expectation is the WHOLE sentence, by hand. A fragment would pass for
    /// a remedy that kept one clause and lost the rest, and the remedies exist to
    /// be read end to end.
    #[test]
    fn the_no_anchor_reasons_remedies_are_spelled_apart() {
        assert_eq!(
            NoAnchorReason::NothingRetained.rank_remedy(),
            "this rank retained no checkpoint: it published no state ring, its plane was \
             refused at arm time, or it had not reached its first anchor cadence when the \
             capture was triggered"
        );
        assert_eq!(
            NoAnchorReason::AllOlderThanTheFrames.rank_remedy(),
            "this rank's checkpoints all predate the frames this capture carries, so \
             resuming from one would execute steps whose inputs are not in the bag: \
             widen the window span or shorten the anchor cadence"
        );
        assert_eq!(
            NoAnchorReason::RetentionCeilingExhausted.rank_remedy(),
            "the retention's byte ceiling took this rank's checkpoints: raise the anchor \
             ceiling so the plane can hold one whole checkpoint generation"
        );
        // All three DISTINCT, which is the property the substitution above broke
        // and the one no single equality can state.
        let remedies = [
            NoAnchorReason::NothingRetained.rank_remedy(),
            NoAnchorReason::AllOlderThanTheFrames.rank_remedy(),
            NoAnchorReason::RetentionCeilingExhausted.rank_remedy(),
        ];
        let unique: std::collections::BTreeSet<&str> = remedies.iter().copied().collect();
        assert_eq!(unique.len(), remedies.len(), "{remedies:?}");
        // …and each remedy names a DIFFERENT lever, so the three are actionable
        // apart rather than merely different strings.
        assert!(NoAnchorReason::NothingRetained
            .rank_remedy()
            .contains("published no state ring"));
        assert!(NoAnchorReason::AllOlderThanTheFrames
            .rank_remedy()
            .contains("widen the window span"));
        assert!(NoAnchorReason::RetentionCeilingExhausted
            .rank_remedy()
            .contains("raise the anchor"));
    }

    trait ExpectNone {
        fn expect_none_or_panic(self);
    }
    impl ExpectNone for Option<HarvestedAnchor> {
        fn expect_none_or_panic(self) {
            assert!(self.is_none(), "expected no anchor yet");
        }
    }

    /// `skip_cause` reads the kind family the encoder MINTS, so the cause survives
    /// into the sentence `plan_restore` renders.
    ///
    /// Left on the previous format's constant this returns `None` on the arm its own
    /// doc calls unreachable, and the operator loses the reason their capture cannot
    /// be resumed. The compiler cannot see it: the test is a `u32` against a `u32`.
    #[test]
    fn a_skipped_anchor_reports_its_cause_from_the_kind_the_encoder_mints() {
        let skipped = RetainedAnchor {
            ring: "r0".to_string(),
            node_idx: 1,
            node: Some("n1".to_string()),
            kind: AnchorKind::Skipped,
            records: Arc::new(vec![cerulion_core::state_ring::encode_skip_record(
                7,
                42,
                1,
                0,
                SkipCause::ChildTimeout,
                "no progress",
            )]),
        };
        assert_eq!(
            skipped.skip_cause(),
            Some(SkipCause::ChildTimeout),
            "the cause the writer put in, not None"
        );

        // THE CONTROL: a record in the PREVIOUS layout carries the v0 SKIP kind at
        // bytes 24 to 28, and this build must NOT read it as a skip. Without it the
        // arm above would also pass on a reader widened to accept both families,
        // which is exactly what the kind change exists to prevent.
        let mut old = [0u8; STATE_RECORD_SIZE as usize];
        old[24..28].copy_from_slice(&3u32.to_le_bytes()); // the v0 SKIP kind
        old[28..32].copy_from_slice(&4u32.to_le_bytes());
        old[32..36].copy_from_slice(&SkipCause::ChildTimeout.as_wire().to_le_bytes());
        let previous_format = RetainedAnchor {
            records: Arc::new(vec![old]),
            ..skipped.clone()
        };
        assert_eq!(
            previous_format.skip_cause(),
            None,
            "a previous-format record is not this build's skip"
        );
    }

    // ----------------------------------------------------------------------
    // The per-rank retention. Every expected value below is written out
    // LITERALLY rather than recomputed from the code under test: a 64-byte blob
    // is ONE 512-byte record (the payload region is 472 bytes), so a one-node
    // anchor costs 512 bytes and a two-rank checkpoint at one step costs 1024.
    // ----------------------------------------------------------------------

    /// ORACLE 1, arm (a): two RANKS at the SAME step are two checkpoints.
    ///
    /// Before the re-key one entry held both ranks' anchors and a selection
    /// returned it as the whole graph's state, so the second rank's step, stamp
    /// and fit had nowhere to be reported.
    #[test]
    fn two_ranks_at_one_step_are_two_checkpoints_and_two_members() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        w.admit(10_000 * MS, rank_anchor(0, 7, 41, 0, &[1; 64]));
        w.admit(10_100 * MS, rank_anchor(1, 7, 41, 0, &[2; 64]));

        assert_eq!(
            w.checkpoints(),
            2,
            "one step on two rings is two checkpoints"
        );
        let selection = w
            .select(0, 20_000 * MS, CAPTURE)
            .expect("both ranks selected");
        assert_eq!(selection.selected.len(), 2);
        assert!(selection.shortfall.is_empty());

        let zero = &selection.selected["rank0"];
        assert_eq!(zero.checkpoint.ring, "rank0");
        assert_eq!(zero.checkpoint.step, 41);
        assert_eq!(zero.checkpoint.taken_at_ns, 10_000 * MS);
        assert_eq!(zero.checkpoint.anchors.len(), 1);
        assert_eq!(zero.checkpoint.byte_len(), 512);
        assert_eq!(zero.fit, AnchorFit::CoversTheClaimedWindow);

        let one = &selection.selected["rank1"];
        assert_eq!(one.checkpoint.ring, "rank1");
        assert_eq!(one.checkpoint.step, 41);
        assert_eq!(one.checkpoint.taken_at_ns, 10_100 * MS);
        assert_eq!(one.checkpoint.anchors.len(), 1);
        assert_eq!(one.checkpoint.byte_len(), 512);
        assert_eq!(one.fit, AnchorFit::CoversTheClaimedWindow);

        // The ranks the members really carry, read out of the records.
        assert_eq!(producer_rank(&zero.checkpoint.anchors[0].records), Some(0));
        assert_eq!(producer_rank(&one.checkpoint.anchors[0].records), Some(1));
    }

    /// ORACLE 1, arm (b), and ORACLE 7: ONE deadline, two ranks, DIFFERENT
    /// steps, and the set is returned rather than refused.
    #[test]
    fn the_anchor_set_is_one_deadline_and_a_non_simultaneous_cut_is_accepted() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        w.admit(10_000 * MS, rank_anchor(0, 7, 41, 0, &[1; 64]));
        w.admit(11_000 * MS, rank_anchor(1, 7, 44, 0, &[2; 64]));

        let selection = w
            .select(0, 12_000 * MS, CAPTURE)
            .expect("one deadline, two ranks, two answers");
        let pairs: Vec<(&str, u64)> = selection
            .selected
            .iter()
            .map(|(ring, m)| (ring.as_str(), m.checkpoint.step))
            .collect();
        assert_eq!(
            pairs,
            vec![("rank0", 41), ("rank1", 44)],
            "each rank's own step, against ONE capture deadline"
        );
        assert!(
            selection.shortfall.is_empty(),
            "a non-simultaneous cut is accepted, never a shortfall"
        );
    }

    /// ORACLE 7, the SHORTFALL half: a rank that contributes nothing is NAMED
    /// with its reason, rather than being silently omitted and leaving the set
    /// with a hole nothing accounts for.
    ///
    /// Rank 1 was DECLARED and anchored nothing, which is the shape a capture
    /// must never render as a two-rank run with one rank's state: the set holds
    /// rank 0 alone and the shortfall says rank 1 and why.
    #[test]
    fn an_incomplete_rank_is_named_in_the_shortfall_rather_than_leaving_a_hole() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        w.declare_ring_nodes("rank0", 0, &["r0n0".to_string(), "r0n1".to_string()]);
        w.declare_ring_nodes("rank1", 1, &["r1n0".to_string(), "r1n1".to_string()]);
        // Rank 0 drained both its nodes at step 41. Rank 1 drained nothing.
        w.admit(10_000 * MS, rank_anchor(0, 7, 41, 0, &[1; 64]));
        w.admit(10_010 * MS, rank_anchor(0, 7, 41, 1, &[1; 64]));

        let selection = w
            .select(0, 12_000 * MS, CAPTURE)
            .expect("rank 0 still answers");
        assert_eq!(
            selection.selected.keys().collect::<Vec<_>>(),
            vec!["rank0"],
            "only the rank that anchored joins the set"
        );
        assert_eq!(
            selection.shortfall.get("rank1"),
            Some(&MissingRank {
                rank: Some(1),
                reason: NoAnchorReason::NothingRetained,
            }),
            "…and the rank that did not is NAMED, BY RANK, with the reason. The ring \
             universe is the DECLARED table, so a rank holding nothing is still known"
        );

        // ANTI-VACUITY: the same retention with one anchor on rank 1 returns
        // BOTH and an EMPTY shortfall, so the arm above is the shortfall rule
        // and not a selection that simply never returns two.
        w.admit(10_020 * MS, rank_anchor(1, 7, 44, 0, &[2; 64]));
        let whole = w
            .select(0, 12_000 * MS, CAPTURE)
            .expect("both ranks selected");
        assert_eq!(whole.selected.len(), 2);
        assert!(whole.shortfall.is_empty());
    }

    /// The predicate's first rejected alternative, pinned: a checkpoint MISSING
    /// one of its ring's declared nodes is still SELECTED, never refused.
    ///
    /// A node absent from a checkpoint is either still in flight or never
    /// anchored, and the retention cannot tell those apart. The recorder writes
    /// the capture and names the absent node in its coverage manifest, and this
    /// arm is what stops a declared-table completeness rule from quietly
    /// converting every such capture into a total refusal.
    #[test]
    fn a_checkpoint_missing_a_declared_node_is_still_selected_and_never_refused() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        w.declare_ring_nodes("rank0", 0, &["r0n0".to_string(), "r0n1".to_string()]);
        // Only node 0 of the two the ring declared.
        w.admit(10_000 * MS, rank_anchor(0, 7, 41, 0, &[1; 64]));

        let selection = w
            .select(0, 12_000 * MS, CAPTURE)
            .expect("a partial checkpoint IS an answer");
        assert_eq!(selection.selected.len(), 1);
        assert_eq!(selection.selected["rank0"].checkpoint.step, 41);
        assert_eq!(
            selection.selected["rank0"].checkpoint.anchors.len(),
            1,
            "one of the ring's two declared nodes, carried rather than refused"
        );
        assert!(selection.shortfall.is_empty());
    }

    /// The predicate's second rejected alternative, pinned: a rank whose every
    /// node DECLINED is still selected, so the skip reaches the manifest.
    ///
    /// Its checkpoint restores no state: `complete_anchors` is 0 and the fold
    /// reports 0, and it carries the markers that say WHY each
    /// node declined. A rule that admitted only checkpoints with a complete
    /// anchor would drop this ring from the capture's ring-to-rank map, so the
    /// declining rank would vanish from the manifest instead of being reported
    /// in it.
    #[test]
    fn a_rank_whose_every_node_declined_is_still_selected_so_the_skip_is_reported() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        w.declare_ring_nodes("rank0", 0, &["r0n0".to_string()]);
        w.declare_ring_nodes("rank1", 1, &["r1n0".to_string()]);
        w.admit(10_000 * MS, rank_anchor(0, 7, 41, 0, &[1; 64]));
        w.admit(10_010 * MS, rank_skip(1, 7, 41, 0));

        let selection = w
            .select(0, 12_000 * MS, CAPTURE)
            .expect("both ranks answer");
        assert_eq!(
            selection.selected.len(),
            2,
            "the declining rank is IN the set, so the capture can report it"
        );
        assert!(
            selection.shortfall.is_empty(),
            "a rank that said why it declined did not fall short: it answered"
        );
        assert_eq!(
            selection.selected["rank1"].checkpoint.complete_anchors(),
            0,
            "…and the set member reports 0 complete anchors, which is what it holds"
        );
    }

    /// ORACLE 11: after an eviction the set is either WHOLE or STAMPED, never
    /// silently short.
    ///
    /// The exhaustiveness property in one line: the ring universe the retention
    /// knows about is EXACTLY the union of the ranks that contributed and the
    /// ranks that were stamped. A rank in neither map is a hole nothing can
    /// account for, which is the outcome the missing-rank stamp exists to make
    /// impossible, and an eviction is the easiest way to produce one by
    /// accident.
    #[test]
    fn an_eviction_leaves_the_set_whole_or_stamped_and_never_silently_short() {
        let mut w = AnchorWindow::new(5_000 * MS, 1 << 30);
        w.declare_ring_nodes("rank0", 0, &["r0n0".to_string()]);
        w.declare_ring_nodes("rank1", 1, &["r1n0".to_string()]);
        // Step 40 is the ONLY generation rank 1 ever reached: it stopped
        // anchoring after it, which is the shape a rank that died produces.
        w.admit(9_000 * MS, rank_anchor(0, 7, 40, 0, &[1; 64]));
        w.admit(9_010 * MS, rank_anchor(1, 7, 40, 0, &[2; 64]));
        w.admit(10_000 * MS, rank_anchor(0, 7, 41, 0, &[1; 64]));
        w.admit(20_000 * MS, rank_anchor(0, 7, 42, 0, &[1; 64]));

        // The horizon (24 s minus the 5 s span) is 19 s, so generations 40 and
        // 41 are both below it; the AGE arm drops whole generations while at
        // least two are below, so 40 goes and 41 is the CARRY. Rank 1's only
        // entry was in 40.
        w.evict(24_000 * MS, None);
        assert_eq!(
            w.checkpoints(),
            2,
            "precondition: generation 40 went whole and rank 0 still holds 41 and 42"
        );

        let selection = w.select(0, 25_000 * MS, CAPTURE).expect("rank 0 answers");
        assert_eq!(selection.selected.keys().collect::<Vec<_>>(), vec!["rank0"]);
        assert_eq!(
            selection.shortfall.get("rank1"),
            Some(&MissingRank {
                rank: Some(1),
                reason: NoAnchorReason::NothingRetained,
            }),
            "the evicted rank is STAMPED, by rank and with a reason"
        );

        // THE EXHAUSTIVENESS ASSERT, written as the union rather than as two
        // separate counts: a rank that fell out of both maps would pass a pair
        // of "at least one" checks and fail this one.
        let mut accounted: Vec<&str> = selection
            .selected
            .keys()
            .chain(selection.shortfall.keys())
            .map(String::as_str)
            .collect();
        accounted.sort_unstable();
        assert_eq!(
            accounted,
            vec!["rank0", "rank1"],
            "every ring the retention knows about is in exactly one of the two maps"
        );
    }

    /// ORACLE 10, THE CONTROL: a k=2 capture whose BOTH ranks lose everything to
    /// the ceiling still answers the whole-capture refusal reason.
    ///
    /// The partial shape is written and stamped; this shape is not written at
    /// all, and the recorder keys that refusal on exactly this `Err`. An
    /// implementation that reported the milder `NothingRetained` here would turn
    /// the never-frames-only refusal into a bag.
    #[test]
    fn both_ranks_taken_by_the_ceiling_still_answers_the_refusal_reason() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        w.declare_ring_nodes("rank0", 0, &["r0n0".to_string()]);
        w.declare_ring_nodes("rank1", 1, &["r1n0".to_string()]);
        w.admit(10_000 * MS, rank_anchor(0, 7, 41, 0, &[1; 64]));
        w.admit(10_010 * MS, rank_anchor(1, 7, 41, 0, &[2; 64]));
        // The ceiling drops to below one entry, so both go and both are counted.
        w.set_max_bytes(0);
        w.evict(11_000 * MS, None);
        assert_eq!(w.checkpoints(), 0, "precondition: the ceiling took both");

        assert_eq!(
            w.select(0, 12_000 * MS, CAPTURE).err(),
            Some(NoAnchorReason::RetentionCeilingExhausted),
            "the reason the whole-capture refusal is keyed on, not the milder one"
        );

        // ANTI-VACUITY: a retention that never held anything answers the MILDER
        // reason, so the arm above is reading the ceiling and not a constant.
        let never = AnchorWindow::new(30_000 * MS, 1 << 30);
        assert_eq!(
            never.select(0, 12_000 * MS, CAPTURE).err(),
            Some(NoAnchorReason::NothingRetained)
        );
    }

    /// ORACLE 13, arms (a) and (b): every member of a capture's set carries THAT
    /// capture's number, and the SAME checkpoint selected again by the next
    /// capture carries the next number.
    ///
    /// The stamp is what a resume reads to refuse a set assembled from two
    /// captures, so a stamp applied at HARVEST would be wrong in exactly the
    /// case the refusal exists for: a checkpoint is admitted on the drive loop
    /// long before any capture is triggered, and one retained checkpoint is
    /// legitimately selected by several captures in turn.
    #[test]
    fn every_member_carries_the_selecting_captures_number_not_the_harvests() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        // Both admitted long before either capture below exists. Rank 1's is
        // the OLDER of the two, which is the member a harvest-time stamp would
        // get most wrong.
        w.admit(10_000 * MS, rank_anchor(1, 7, 41, 0, &[2; 64]));
        w.admit(10_500 * MS, rank_anchor(0, 7, 41, 0, &[1; 64]));

        // ARM (a): capture 9 selects both, and both carry 9.
        let ninth = w.select(0, 12_000 * MS, 9).expect("both ranks selected");
        assert_eq!(ninth.selected.len(), 2);
        assert_eq!(ninth.selected["rank0"].capture_seq, 9);
        assert_eq!(
            ninth.selected["rank1"].capture_seq, 9,
            "the rank admitted first carries the SELECTING capture's number, \
             not a number from its own harvest"
        );

        // ARM (b): capture 10 selects the SAME two checkpoints and they carry 10.
        let tenth = w
            .select(0, 12_000 * MS, 10)
            .expect("both ranks selected again");
        assert_eq!(tenth.selected["rank0"].capture_seq, 10);
        assert_eq!(tenth.selected["rank1"].capture_seq, 10);
        // …and it really is the same checkpoint, so arm (b) is about the STAMP
        // and not about a different member being chosen.
        assert_eq!(
            tenth.selected["rank0"].checkpoint.step,
            ninth.selected["rank0"].checkpoint.step
        );
        assert_eq!(
            tenth.selected["rank0"].checkpoint.taken_at_ns,
            ninth.selected["rank0"].checkpoint.taken_at_ns
        );
    }

    /// ORACLE 12: the FITS may differ across ranks, and each rank reports its own.
    ///
    /// Rank 0's newest candidate sits at or before the capture's deadline, so it
    /// COVERS the claimed window. Rank 1 holds only a checkpoint taken AFTER the
    /// deadline (but at or above the floor), so its best answer is the oldest
    /// candidate, reported NEWER than the claimed window. A selection that
    /// published one fit for the set would tell a reader one of those two facts
    /// about a rank for which it is false.
    #[test]
    fn the_fits_differ_across_ranks_and_each_rank_reports_its_own() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        w.admit(10_000 * MS, rank_anchor(0, 7, 41, 0, &[1; 64]));
        // AFTER the deadline below, so rank 1 has no candidate inside the window.
        w.admit(14_000 * MS, rank_anchor(1, 7, 44, 0, &[2; 64]));

        let selection = w
            .select(0, 12_000 * MS, CAPTURE)
            .expect("both ranks answer");
        assert_eq!(
            selection.selected["rank0"].fit,
            AnchorFit::CoversTheClaimedWindow
        );
        assert_eq!(
            selection.selected["rank1"].fit,
            AnchorFit::NewerThanTheClaimedWindow,
            "rank 1's only candidate is newer than the window this capture claims"
        );

        // THE CONTROL: under lockstep the two ranks sit at one instant inside the
        // window and the two fits are EQUAL, so the arm above is measuring the
        // per-rank answer and not a selection that always disagrees.
        let mut lockstep = AnchorWindow::new(30_000 * MS, 1 << 30);
        lockstep.admit(10_000 * MS, rank_anchor(0, 7, 41, 0, &[1; 64]));
        lockstep.admit(10_010 * MS, rank_anchor(1, 7, 41, 0, &[2; 64]));
        let both = lockstep
            .select(0, 12_000 * MS, CAPTURE)
            .expect("both ranks answer");
        assert_eq!(
            both.selected["rank0"].fit, both.selected["rank1"].fit,
            "a lockstep cut gives one answer to both ranks"
        );
        assert_eq!(
            both.selected["rank0"].fit,
            AnchorFit::CoversTheClaimedWindow
        );
    }

    /// ORACLE 8: the byte reserve is the SUM over the k checkpoints of ONE
    /// step, not the largest single rank's and not the sum over every retained
    /// step.
    #[test]
    fn the_reserve_is_measured_over_one_whole_cross_rank_checkpoint() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        // Step 50: rank 0 one record, rank 1 TWO (a 600-byte blob does not fit
        // one 472-byte payload region).
        w.admit(10_000 * MS, rank_anchor(0, 7, 50, 0, &[1; 64]));
        w.admit(10_010 * MS, rank_anchor(1, 7, 50, 0, &[2; 600]));
        assert_eq!(
            w.max_generation_bytes(),
            Some(1536),
            "512 + 1024: the sum over the k ranks of ONE step, not the largest rank's 1024"
        );

        // Step 51, the same shape. The reserve must NOT grow to 3072: one
        // generation is what has to be held at a time.
        w.admit(20_000 * MS, rank_anchor(0, 7, 51, 0, &[1; 64]));
        w.admit(20_010 * MS, rank_anchor(1, 7, 51, 0, &[2; 600]));
        assert_eq!(
            w.max_generation_bytes(),
            Some(1536),
            "a second retained step does not raise the demand for ONE generation"
        );
        assert_eq!(
            w.bytes(),
            3072,
            "the RETENTION holds both, which is a different number"
        );
    }

    /// ORACLE 9, arm (a) AGE: eviction takes whole cross-rank checkpoints.
    #[test]
    fn the_age_arm_takes_every_rank_of_the_oldest_step_or_none_of_it() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        for (step, at) in [(40u64, 0u64), (41, 20_000 * MS), (42, 40_000 * MS)] {
            w.admit(at, rank_anchor(0, 7, step, 0, &[1; 64]));
            w.admit(at + MS, rank_anchor(1, 7, step, 0, &[2; 64]));
        }
        assert_eq!(w.checkpoints(), 6, "three steps on two rings");

        // Horizon 25 s: steps 40 and 41 are below it, 42 is not. The newest
        // aged-out step (41) is the CARRY and stays whole.
        let report = w.evict(55_000 * MS, None);
        assert_eq!(report.aged, 1, "ONE checkpoint aged, not two deque entries");
        assert_eq!(report.truncated, 0);
        let held: Vec<(u64, &str)> = w
            .checkpoints
            .iter()
            .map(|c| (c.step, c.ring.as_str()))
            .collect();
        assert_eq!(
            held,
            vec![(41, "rank0"), (41, "rank1"), (42, "rank0"), (42, "rank1")],
            "ALL of step 40 went and ALL of step 41 was carried"
        );
        assert_eq!(w.bytes(), 2048, "four 512-byte anchors remain");
        assert_eq!(w.aged(), 1);
    }

    /// ORACLE 9, arm (b) BYTES: the oldest WHOLE step goes, `bytes` falls by the
    /// sum of its k entries, and the ceiling counts ONE refusal for the step.
    #[test]
    fn the_byte_arm_takes_a_whole_step_and_counts_one_refusal_for_it() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        for (step, at) in [(40u64, 0u64), (41, 100 * MS), (42, 200 * MS)] {
            w.admit(at, rank_anchor(0, 7, step, 0, &[1; 64]));
            w.admit(at + MS, rank_anchor(1, 7, step, 0, &[2; 64]));
        }
        assert_eq!(w.bytes(), 3072);
        assert_eq!(w.ceiling_refusals(), 0);

        // A ceiling that leaves room for two of the three cross-rank
        // checkpoints. Nothing has aged (the horizon is 0).
        w.set_max_bytes(2048);
        let report = w.evict(1_000 * MS, None);
        assert_eq!(
            w.bytes(),
            2048,
            "bytes fell by the SUM of step 40's two entries"
        );
        assert_eq!(
            w.ceiling_refusals(),
            1,
            "ONE refusal for the whole cross-rank checkpoint, never k of them"
        );
        assert_eq!(
            report.aged, 1,
            "no capture wanted it, so it lands in the aged bucket"
        );
        assert_eq!(report.truncated, 0);
        let held: Vec<(u64, &str)> = w
            .checkpoints
            .iter()
            .map(|c| (c.step, c.ring.as_str()))
            .collect();
        assert_eq!(
            held,
            vec![(41, "rank0"), (41, "rank1"), (42, "rank0"), (42, "rank1")],
            "the oldest WHOLE step went; no step is left with one rank missing"
        );
        // The refused-bytes floor is the whole generation's, not one rank's.
        assert_eq!(w.refused_bytes_floor(), Some(1024));
    }

    /// ORACLE 6, the RETENTION half: a lockstep (one ring) retention answers
    /// exactly what it answered before the set existed.
    #[test]
    fn a_lockstep_retention_selects_one_member_with_the_parents_numbers() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        w.declare_ring_nodes("rank0", 0, &["r0n0".to_string()]);
        w.admit(10_000 * MS, rank_anchor(0, 7, 41, 0, &[1; 64]));
        w.admit(25_000 * MS, rank_anchor(0, 7, 42, 0, &[1; 64]));

        let selection = w.select(0, 20_000 * MS, CAPTURE).expect("selected");
        assert_eq!(selection.selected.len(), 1, "one ring is one member");
        assert!(selection.shortfall.is_empty());
        let (picked, fit) = selection.sole();
        assert_eq!(picked.step, 41, "the NEWEST at or before the deadline");
        assert_eq!(picked.taken_at_ns, 10_000 * MS);
        assert_eq!(picked.anchors.len(), 1);
        assert_eq!(picked.complete_anchors(), 1);
        assert_eq!(picked.byte_len(), 512);
        assert_eq!(fit, AnchorFit::CoversTheClaimedWindow);
        assert_eq!(w.oldest_ns(), Some(10_000 * MS));
        assert_eq!(w.aged(), 0);
        assert_eq!(w.truncated(), 0);
        assert_eq!(w.ceiling_refusals(), 0);
    }

    /// A rank the ceiling emptied is still NAMED, because the ring universe is
    /// what the retention was TOLD about rather than what it is holding.
    #[test]
    fn a_rank_whose_ring_holds_nothing_is_still_named_in_the_shortfall() {
        let mut w = AnchorWindow::new(30_000 * MS, 1 << 30);
        w.declare_ring_nodes("rank0", 0, &["r0n0".to_string()]);
        w.declare_ring_nodes("rank1", 1, &["r1n0".to_string()]);
        w.admit(10_000 * MS, rank_anchor(0, 7, 41, 0, &[1; 64]));
        w.note_ceiling_refusal(1);

        let selection = w.select(0, 20_000 * MS, CAPTURE).expect("rank 0 answers");
        assert_eq!(selection.selected.keys().collect::<Vec<_>>(), vec!["rank0"]);
        assert_eq!(
            selection.shortfall.get("rank1"),
            Some(&MissingRank {
                rank: Some(1),
                reason: NoAnchorReason::RetentionCeilingExhausted,
            }),
            "the rank that holds nothing while the ceiling has bitten says WHICH absence"
        );
    }
}
