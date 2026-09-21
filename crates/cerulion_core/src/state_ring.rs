// SPDX-License-Identifier: AGPL-3.0-only
//! The node-state-CHECKPOINT layer over the generic [`crate::shm_ring`] SPSC ring
//! and the sibling of [`crate::trace_ring`].
//!
//! Where [`crate::shm_ring`] is a reusable "fixed-size opaque records in, zero-copy
//! byte slices out" ring, THIS module fixes the record TYPE and the reassembly rule
//! for a node-state anchor:
//!
//! - [`StateRecordHeader`] — a 32-byte, endian-pinned, padding-free record header
//!   followed by [`STATE_RECORD_PAYLOAD`] payload bytes, [`STATE_RECORD_SIZE`]
//!   (512 B) in total. Its byte layout is a FORMAT CONTRACT: the record IS the bag's
//!   `__cerulion/state` payload, so it is hand-encoded little-endian (never
//!   transmuted).
//! - [`StateChunker`] — the pure writer half. **Chunking is what makes a size
//!   refusal unnecessary: a bigger state is more records. Arithmetic, not
//!   policy.**
//! - [`StateAssembler`] — the pure reader half, which reassembles
//!   `(run_id, step, node_idx)` streams and reports a TORN one rather than serving a
//!   short blob.
//! - The ring MANIFEST is the node-identity table, and it is the SAME table
//!   [`crate::trace_ring`] already defines — `node_idx` in a record indexes into it.
//!
//! # Why the ring, and not a reserved iceoryx2 topic
//!
//! `MAP_SHARED` is SHARED across `fork`, not copy-on-write, so the checkpoint child
//! writes straight into the segment the parent and `bagd` already see: no pipe, no
//! pump thread, and no extra memcpy of the whole state. A ring is POSIX SHM rather
//! than an iceoryx2 service, so `topic list` and the Studio sidebar never see it and
//! there is nothing to filter.
//!
//! # This ring is ALWAYS [`OverrunPolicy::Backpressure`]
//!
//! [`StateRingOwner::create`] is the only constructor and it selects
//! [`OverrunPolicy::Backpressure`] unconditionally, so the mode cannot drift: a
//! 500 MB anchor is ~1.09 M records through a fixed ring, and a lapped anchor is a
//! LOST anchor. Blocking the writer is harmless because the writer is a short-lived
//! `fork` child, never the hot loop. [`StateRingConsumer::open`] REFUSES a
//! ring created under any other policy, so a mis-created ring is loud at the first
//! read rather than silently lossy.
//!
//! # Nothing here allocates on the writer path
//!
//! [`StateChunker`] holds its partial record in an inline `[u8; 480]`, encodes onto a
//! `[u8; 512]` stack buffer, and hands that to the ring's `push` — no alloc, no lock,
//! no clock read (the backpressure room check reads a clock only on a push that must
//! actually WAIT). That is the constraint that makes it usable in a `fork` child.
//! The READER allocates (it reassembles into a `Vec<u8>`) and runs in `bagd` or on
//! the restore path, never in the child.
//!
//! NOTE: this module is compiled only on Unix — the `#[cfg(unix)]` gate lives on its
//! `pub mod state_ring;` declaration in `lib.rs`.

use std::collections::BTreeMap;

use crate::shm_ring::{
    prev_power_of_two, OverrunPolicy, ShmRingConsumer, ShmRingError, ShmRingOwner, ShmRingProducer,
    ShmRingResult,
};
// The node-identity table is ONE format with ONE implementation (the rule:
// a second copy of a shared codec is how two surfaces drift). It is defined in
// `trace_ring` because that layer needed it first; the dependency is acyclic and
// one-directional, and hoisting the two functions into `shm_ring` would rename two
// `pub` items for no behavioural gain.
use crate::trace_ring::{decode_manifest, encode_manifest, TraceRingError};

// ===========================================================================
// Record format
// ===========================================================================

/// The fixed size of one state record on the ring / in the bag, in bytes.
pub const STATE_RECORD_SIZE: u32 = 512;

/// The record HEADER size in bytes. Everything after it is payload.
pub const STATE_RECORD_HEADER_SIZE: usize = 32;

/// Payload bytes carried by ONE record: [`STATE_RECORD_SIZE`] minus the header.
///
/// This is the number the ring arithmetic is stated in: a 500 MB anchor is
/// ~1.09 M records and ~534 MB of ring traffic, i.e. a **6.7 % framing overhead**
/// (`512 / 480`), and any ring sizing must be read with that multiplier.
pub const STATE_RECORD_PAYLOAD: usize = STATE_RECORD_SIZE as usize - STATE_RECORD_HEADER_SIZE;

const _: () = assert!(
    STATE_RECORD_HEADER_SIZE + STATE_RECORD_PAYLOAD == STATE_RECORD_SIZE as usize,
    "the header and payload must exactly fill a record"
);
const _: () = assert!(STATE_RECORD_PAYLOAD == 480);

/// Record kind: a NON-final payload chunk. Its `len` is always
/// [`STATE_RECORD_PAYLOAD`] — a short non-final chunk is structurally impossible
/// from [`StateChunker`] and is reported [`TornCause::ShortChunk`] by the reader.
pub const RECORD_KIND_CHUNK: u32 = 1;

/// Record kind: the FINAL payload chunk of one node's state at one step. Its `part`
/// is the last index, so the anchor's part count is `part + 1` — which is why the
/// header carries no separate `parts_total` (see [`StateRecordHeader`]).
pub const RECORD_KIND_FINAL: u32 = 2;

/// Record kind: a SKIP record — the rule is "SKIP records naming a voided
/// anchor's cause". Its payload is `cause: u32 (LE)` followed by optional UTF-8
/// detail; its `part` is 0 and it is complete in one record.
pub const RECORD_KIND_SKIP: u32 = 3;
// `kind == 0` is INVALID (a zeroed slot). Values 4+ are RESERVED.

/// A state record's 32-byte header.
///
/// # Why there is no `parts_total`
///
/// A simpler sketch of the header is
/// `run_id | S | node_idx | part | parts_total | len`. A STREAMING writer cannot
/// fill `parts_total`: the fork child emits record 0 long before it knows how many
/// records a 500 MB encode will take, and buffering the blob to find out is exactly
/// what chunking exists to avoid. As specified the field would therefore have to
/// carry a sentinel on every non-final record — a count that is not a count, which
/// is the misleading-name class this repo rejects.
///
/// It is replaced, at the SAME offset and the same width, by [`kind`](Self::kind),
/// which carries the identical completeness information — the final record is
/// marked, and `parts_total == part + 1` there — while giving the SKIP record an
/// explicit discriminant instead of a sentinel smuggled into a count. The header size,
/// the payload size, and every number in the 6.7 % arithmetic are byte-unchanged.
///
/// # `node_idx` is an INDEX, never a name
///
/// Node ids are `String` keys, so a fixed-width name field would silently collide two
/// nodes sharing a prefix. `node_idx` resolves through the ring manifest —
/// the same node-identity table [`crate::trace_ring`] defines — read by
/// [`StateRingConsumer::node_ids`]. `rank` is already a ring-HEADER field, so it does
/// not ride the record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(C)]
pub struct StateRecordHeader {
    /// The run this record belongs to (the run identity, opaque here).
    ///
    /// It rides every record because the bag's `__cerulion/state` channel merges the
    /// records of every worker RANK of a run, and a recorder bound to a run must be
    /// able to check a record really belongs to it without consulting a ring header
    /// it may not have created.
    pub run_id: u64,
    /// The anchor step `S` — the state AFTER step `S` completed, taken at the
    /// boundary that follows it.
    pub step: u64,
    /// Index into the ring manifest's node-identity table.
    pub node_idx: u32,
    /// 0-based chunk index within this `(run_id, step, node_idx)` stream.
    pub part: u32,
    /// One of [`RECORD_KIND_CHUNK`] / [`RECORD_KIND_FINAL`] / [`RECORD_KIND_SKIP`]
    /// (0 invalid, 4+ reserved).
    pub kind: u32,
    /// Valid payload bytes in this record (`<= STATE_RECORD_PAYLOAD`).
    pub len: u32,
}

const _: () = assert!(
    std::mem::size_of::<StateRecordHeader>() == STATE_RECORD_HEADER_SIZE,
    "StateRecordHeader must be exactly 32 bytes"
);
const _: () = assert!(std::mem::align_of::<StateRecordHeader>() == 8);

impl StateRecordHeader {
    /// Encode to the 32-byte little-endian wire form (the format contract). Owned
    /// array, not a reference: the bytes are hand-built (never a transmute), so they
    /// are endian-independent and padding-free.
    pub fn as_bytes(&self) -> [u8; STATE_RECORD_HEADER_SIZE] {
        let mut b = [0u8; STATE_RECORD_HEADER_SIZE];
        b[0..8].copy_from_slice(&self.run_id.to_le_bytes());
        b[8..16].copy_from_slice(&self.step.to_le_bytes());
        b[16..20].copy_from_slice(&self.node_idx.to_le_bytes());
        b[20..24].copy_from_slice(&self.part.to_le_bytes());
        b[24..28].copy_from_slice(&self.kind.to_le_bytes());
        b[28..32].copy_from_slice(&self.len.to_le_bytes());
        b
    }

    /// Decode from the 32-byte little-endian wire form. Total inverse of
    /// [`as_bytes`](Self::as_bytes). Performs NO validation — see
    /// [`validate`](Self::validate).
    pub fn from_bytes(b: &[u8; STATE_RECORD_HEADER_SIZE]) -> Self {
        Self {
            run_id: u64::from_le_bytes(b[0..8].try_into().unwrap()),
            step: u64::from_le_bytes(b[8..16].try_into().unwrap()),
            node_idx: u32::from_le_bytes(b[16..20].try_into().unwrap()),
            part: u32::from_le_bytes(b[20..24].try_into().unwrap()),
            kind: u32::from_le_bytes(b[24..28].try_into().unwrap()),
            len: u32::from_le_bytes(b[28..32].try_into().unwrap()),
        }
    }

    /// Structural validation of an UNTRUSTED header: the kind is one this build
    /// knows, and `len` fits the payload region.
    ///
    /// Deliberately does NOT judge whether a non-final chunk is full — that is a
    /// STREAM rule ([`TornCause::ShortChunk`]), not a property of one header, and
    /// conflating them would let the reader report "malformed record" for what is
    /// really a truncated anchor.
    pub fn validate(&self) -> Result<(), String> {
        if self.len as usize > STATE_RECORD_PAYLOAD {
            return Err(format!(
                "len {} exceeds the {STATE_RECORD_PAYLOAD}-byte payload region",
                self.len
            ));
        }
        match self.kind {
            RECORD_KIND_CHUNK | RECORD_KIND_FINAL | RECORD_KIND_SKIP => Ok(()),
            0 => Err("kind 0 is a zeroed/never-written slot".to_string()),
            other => Err(format!("kind {other} is not a kind this build knows")),
        }
    }
}

/// Build one whole 512-byte record from a header and its payload.
///
/// The unused tail is ZERO-FILLED, which is a determinism requirement rather than
/// tidiness: these bytes land in the bag, so a record whose padding carried whatever
/// the SHM slot held last would make two byte-identical captures produce two
/// different bags (Principle #7).
///
/// # Panics
///
/// If `payload.len() != header.len` or `payload.len() > STATE_RECORD_PAYLOAD` — a
/// caller bug, fail-loud in EVERY build mode (the record is a format contract and a
/// silently mis-lengthed one is undetectable downstream).
pub fn encode_record(
    header: &StateRecordHeader,
    payload: &[u8],
) -> [u8; STATE_RECORD_SIZE as usize] {
    assert!(
        payload.len() <= STATE_RECORD_PAYLOAD,
        "payload {} exceeds the {STATE_RECORD_PAYLOAD}-byte payload region",
        payload.len()
    );
    assert_eq!(
        payload.len(),
        header.len as usize,
        "header.len must equal the payload length"
    );
    let mut out = [0u8; STATE_RECORD_SIZE as usize];
    out[..STATE_RECORD_HEADER_SIZE].copy_from_slice(&header.as_bytes());
    out[STATE_RECORD_HEADER_SIZE..STATE_RECORD_HEADER_SIZE + payload.len()]
        .copy_from_slice(payload);
    out
}

/// How many records a blob of `len` bytes becomes.
///
/// A ZERO-length blob is still ONE record — a node whose state encodes to nothing is
/// a real, complete anchor, and emitting no record at all would make it
/// indistinguishable from a node that was never captured.
pub fn parts_for_len(len: u64) -> u64 {
    if len == 0 {
        1
    } else {
        len.div_ceil(STATE_RECORD_PAYLOAD as u64)
    }
}

// ===========================================================================
// Skip causes ("SKIP records naming a voided anchor's cause")
// ===========================================================================

/// The cause vocabulary a [`RECORD_KIND_SKIP`] record carries.
///
/// DEFINED in [`crate::state`] (portable) and re-exported here so this module's
/// established path, `cerulion_core::state_ring::SkipCause`, keeps resolving for
/// `cerulion_bagd` and the `cerulion_bag` channel tests. It moved because a cause
/// is something a READER decodes out of a bag, and the machine that reads a bag is
/// not necessarily the one that recorded it — see [`crate::state::SkipCause`]'s
/// module doc.
pub use crate::state::SkipCause;

/// Build a SKIP record for `node_idx` at `step`, naming `cause` with optional UTF-8
/// `detail` (truncated on a UTF-8 character boundary to fit one record).
pub fn encode_skip_record(
    run_id: u64,
    step: u64,
    node_idx: u32,
    cause: SkipCause,
    detail: &str,
) -> [u8; STATE_RECORD_SIZE as usize] {
    const DETAIL_CAP: usize = STATE_RECORD_PAYLOAD - 4;
    let mut cut = detail.len().min(DETAIL_CAP);
    while cut > 0 && !detail.is_char_boundary(cut) {
        cut -= 1;
    }
    let detail = &detail.as_bytes()[..cut];
    let mut payload = [0u8; STATE_RECORD_PAYLOAD];
    payload[0..4].copy_from_slice(&cause.as_wire().to_le_bytes());
    payload[4..4 + detail.len()].copy_from_slice(detail);
    let header = StateRecordHeader {
        run_id,
        step,
        node_idx,
        part: 0,
        kind: RECORD_KIND_SKIP,
        len: (4 + detail.len()) as u32,
    };
    encode_record(&header, &payload[..4 + detail.len()])
}

// ===========================================================================
// The writer half: pure chunking
// ===========================================================================

/// The PURE writer: turns an arbitrarily long byte stream into
/// [`STATE_RECORD_SIZE`] records for one `(run_id, step, node_idx)` anchor.
///
/// It is separated from the ring so the chunking arithmetic — the part that has an
/// off-by-one in it — is testable with no SHM at all, and so a caller with a
/// different destination (the bag-side re-encode, a test) reuses one implementation.
///
/// # Lazy flush, and why the boundary case matters
///
/// A full buffer is emitted only when the NEXT byte needs room, so a blob whose
/// length is an exact multiple of [`STATE_RECORD_PAYLOAD`] produces exactly
/// `len / 480` records and never a trailing empty one. That keeps the ring arithmetic
/// literally true (500 MB ⇒ 1_092_267 records, not 1_092_268) and, more importantly,
/// keeps the FINAL marker on the record that carries the last real byte.
///
/// # Dropping a chunker without [`finish`](Self::finish) is a TRUNCATED anchor
///
/// Deliberately: a `Drop` that auto-finished would publish a SHORT blob as COMPLETE
/// on exactly the paths where the writer died mid-encode. The reader reports the
/// truncation ([`TornCause::Truncated`]) instead, which is the "never silently
/// reassembled short" rule this layer exists to hold.
#[derive(Debug)]
pub struct StateChunker {
    run_id: u64,
    step: u64,
    node_idx: u32,
    next_part: u32,
    buf: [u8; STATE_RECORD_PAYLOAD],
    filled: usize,
}

impl StateChunker {
    /// A chunker for one node's state at one anchor step.
    pub fn new(run_id: u64, step: u64, node_idx: u32) -> Self {
        Self {
            run_id,
            step,
            node_idx,
            next_part: 0,
            buf: [0u8; STATE_RECORD_PAYLOAD],
            filled: 0,
        }
    }

    /// Append `bytes`, emitting whole records through `emit` as they fill.
    ///
    /// Allocation-free: the partial record lives inline and each emitted record is a
    /// stack array.
    pub fn append(
        &mut self,
        mut bytes: &[u8],
        emit: &mut impl FnMut(&[u8; STATE_RECORD_SIZE as usize]),
    ) {
        while !bytes.is_empty() {
            if self.filled == STATE_RECORD_PAYLOAD {
                // Only ever reached because MORE bytes follow, so this record is
                // genuinely non-final.
                self.emit_buffered(RECORD_KIND_CHUNK, emit);
            }
            let n = (STATE_RECORD_PAYLOAD - self.filled).min(bytes.len());
            self.buf[self.filled..self.filled + n].copy_from_slice(&bytes[..n]);
            self.filled += n;
            bytes = &bytes[n..];
        }
    }

    /// Emit the tail as the FINAL record and return the anchor's total part count.
    ///
    /// Always emits at least one record, so a zero-byte state is a complete anchor
    /// rather than an absent one.
    pub fn finish(mut self, emit: &mut impl FnMut(&[u8; STATE_RECORD_SIZE as usize])) -> u32 {
        self.emit_buffered(RECORD_KIND_FINAL, emit);
        self.next_part
    }

    /// Records emitted so far (the next record's `part` index).
    pub fn parts_emitted(&self) -> u32 {
        self.next_part
    }

    fn emit_buffered(
        &mut self,
        kind: u32,
        emit: &mut impl FnMut(&[u8; STATE_RECORD_SIZE as usize]),
    ) {
        let header = StateRecordHeader {
            run_id: self.run_id,
            step: self.step,
            node_idx: self.node_idx,
            part: self.next_part,
            kind,
            len: self.filled as u32,
        };
        emit(&encode_record(&header, &self.buf[..self.filled]));
        self.next_part += 1;
        self.filled = 0;
    }
}

// ===========================================================================
// The reader half: pure reassembly
// ===========================================================================

/// Why a reassembled anchor is TORN — never served as a short blob.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TornCause {
    /// A record arrived whose `part` is not the one this stream owed next: a chunk
    /// was lost, duplicated, or reordered.
    PartOutOfOrder {
        /// The part index the stream owed next.
        expected: u32,
        /// The part index that actually arrived.
        got: u32,
    },
    /// A NON-final chunk carried fewer than [`STATE_RECORD_PAYLOAD`] bytes — a shape
    /// [`StateChunker`] cannot produce, so the stream is corrupt.
    ShortChunk {
        /// The offending part index.
        part: u32,
        /// Its declared length.
        len: u32,
    },
    /// The stream ended (the drain finished) with no FINAL record: the writer died
    /// mid-encode, or the records were lost.
    Truncated {
        /// How many parts had arrived when the stream ended.
        parts_seen: u32,
    },
}

/// What one drained record told the reader.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StateAnchorEvent {
    /// One node's whole state at one step, reassembled byte-for-byte.
    Complete {
        /// The run the anchor belongs to.
        run_id: u64,
        /// The anchor step.
        step: u64,
        /// The manifest index of the node.
        node_idx: u32,
        /// How many records carried it.
        parts: u32,
        /// The reassembled state bytes.
        bytes: Vec<u8>,
    },
    /// An anchor that can never be served — reported at the record that broke it, or
    /// at [`StateAssembler::finish`] for one that simply stopped.
    Torn {
        /// The run the anchor belonged to.
        run_id: u64,
        /// The anchor step.
        step: u64,
        /// The manifest index of the node.
        node_idx: u32,
        /// What broke it.
        cause: TornCause,
    },
    /// The writer VOIDED this anchor on purpose and said why.
    Skipped {
        /// The run the anchor belonged to.
        run_id: u64,
        /// The anchor step.
        step: u64,
        /// The manifest index of the node.
        node_idx: u32,
        /// The named cause.
        cause: SkipCause,
        /// Free-form detail the writer attached (possibly empty).
        detail: String,
    },
    /// A SKIP that arrived for an anchor this stream already COMPLETED (completion
    /// precedence).
    ///
    /// A DIAGNOSTIC, not a refusal. The child publishes a node's final record and
    /// THEN bumps the breadcrumb's accounting word; those are two statements, so an
    /// asynchronous kill between them leaves a complete anchor in the ring with the
    /// parent's post-mortem still believing that node uncovered — and the parent duly
    /// emits a SKIP for it. The `fork.rs` accounting docs explain why the write order
    /// cannot be fixed on the parent side (reversing it trades a loud contradiction
    /// for a SILENT ABSENCE) and why the resolution belongs here.
    ///
    /// **The Complete WINS.** It is fully assembled data the writer really published;
    /// the skip is the parent's guess about accounting it could not observe. Applying
    /// the anchor is the only reading that uses what the run actually captured — and
    /// the skip is reported rather than dropped, because it is real evidence that a
    /// child died in that window.
    SkipAfterComplete {
        /// The run the anchor belongs to.
        run_id: u64,
        /// The anchor step, which is also the completed anchor's step.
        step: u64,
        /// The manifest index of the node.
        node_idx: u32,
        /// The cause the writer named on the skip.
        cause: SkipCause,
        /// Free-form detail the writer attached (possibly empty).
        detail: String,
    },
    /// A record this build cannot even key — reported rather than skipped, because a
    /// stream that quietly drops records it does not understand is how a reader
    /// serves a short blob without knowing it.
    Malformed {
        /// What was wrong with it.
        reason: String,
    },
}

/// Per-anchor reassembly state.
#[derive(Debug)]
enum AnchorState {
    Open {
        next_part: u32,
        buf: Vec<u8>,
    },
    /// Already reported TORN; swallow the rest of the stream so one broken anchor
    /// yields one event, not one per surviving record.
    Voided,
}

/// The PURE reader: reassembles state records into whole anchors, and reports a torn
/// one rather than a short one.
///
/// # Interleaving
///
/// Records are keyed by `(run_id, step, node_idx)`, so two nodes' streams interleaved
/// on one ring reassemble independently. In production one child writes one node at a
/// time, but the BAG merges every worker rank's records and a reader must not depend
/// on that ordering.
///
/// # Armed vs passthrough — the mid-run attach
///
/// [`StateRingConsumer::open_at_live`] lands the read cursor MID-BLOB, so the first
/// records a mid-run recorder sees can be the tail of an anchor whose head was
/// committed before the attach. That is a late attach, not corruption, and reporting
/// it as [`TornCause::PartOutOfOrder`] would cry wolf on every mid-run attach. An
/// [`armed`](Self::armed) assembler DISCARDS records until the first `part == 0`
/// (counted by [`discarded`](Self::discarded)) and then behaves exactly like a
/// [`passthrough`](Self::passthrough) one — the same shape
/// [`crate::trace_ring::HeadStepGate`] already established for the trace ring, and
/// monotone for the same reason (it opens once and never re-arms).
#[derive(Debug)]
pub struct StateAssembler {
    open: BTreeMap<(u64, u64, u32), AnchorState>,
    /// Completion precedence: the step of the newest anchor this stream COMPLETED, per
    /// `(run, node)`.
    ///
    /// Keyed by node rather than by anchor so it cannot grow without bound: one entry
    /// per node per run, the same order as `open`, rather than one per anchor ever
    /// seen. That is exact for the condition it decides — a parent's post-mortem skip
    /// names the step its child was capturing, records reach a reader in ring order,
    /// and steps advance — so the contradicting skip always meets the newest entry.
    completed: BTreeMap<(u64, u32), u64>,
    armed: bool,
    discarded: u64,
    torn_drains: u64,
    skips_after_complete: u64,
}

impl Default for StateAssembler {
    fn default() -> Self {
        Self::passthrough()
    }
}

impl StateAssembler {
    /// An assembler for a stream read from its START: every record is judged, and a
    /// headless anchor is TORN.
    pub fn passthrough() -> Self {
        Self {
            open: BTreeMap::new(),
            completed: BTreeMap::new(),
            armed: false,
            discarded: 0,
            torn_drains: 0,
            skips_after_complete: 0,
        }
    }

    /// An assembler for a MID-RUN attach: discards records until the first `part == 0`
    /// so the partial head anchor is dropped rather than misreported as corruption.
    pub fn armed() -> Self {
        Self {
            open: BTreeMap::new(),
            completed: BTreeMap::new(),
            armed: true,
            discarded: 0,
            torn_drains: 0,
            skips_after_complete: 0,
        }
    }

    /// Whether the assembler is still discarding a partial head anchor.
    pub fn is_armed(&self) -> bool {
        self.armed
    }

    /// How many leading records were discarded as the partial head anchor.
    pub fn discarded(&self) -> u64 {
        self.discarded
    }

    /// How many drains hit a LAP — found already lapped, or lapped mid-read. Every
    /// anchor in flight at such a drain is dropped, so a nonzero count means the
    /// reassembly has holes the record stream itself cannot show (Principle #3).
    pub fn torn_drains(&self) -> u64 {
        self.torn_drains
    }

    /// Completion precedence: remember that `(run, node)` completed an anchor at `step`.
    ///
    /// Keeps the NEWEST step. Records reach a reader in ring order and steps advance,
    /// so this is a running maximum in practice; taking the max explicitly means a
    /// reordered stream cannot make an older completion mask a newer one.
    fn note_completed(&mut self, run_id: u64, step: u64, node_idx: u32) {
        let slot = self.completed.entry((run_id, node_idx)).or_insert(step);
        *slot = (*slot).max(step);
    }

    /// Completion precedence: how many SKIPs arrived for anchors this stream had already
    /// completed (Principle #3 — the contradiction is counted, not merely logged).
    ///
    /// Non-zero means at least one capture child was killed between publishing a
    /// node's final record and bumping its accounting word. The anchors were applied;
    /// this is how many times the parent's post-mortem disagreed.
    pub fn skips_after_complete(&self) -> u64 {
        self.skips_after_complete
    }

    /// How many anchors are partially assembled right now.
    pub fn open_anchors(&self) -> usize {
        self.open.len()
    }

    /// Offer one whole record, in ring order. Returns the event it completed, broke,
    /// or announced — at most one per record.
    pub fn feed(&mut self, record: &[u8]) -> Option<StateAnchorEvent> {
        if record.len() < STATE_RECORD_SIZE as usize {
            return Some(StateAnchorEvent::Malformed {
                reason: format!(
                    "record is {} bytes, short of the {STATE_RECORD_SIZE}-byte record size",
                    record.len()
                ),
            });
        }
        let mut hb = [0u8; STATE_RECORD_HEADER_SIZE];
        hb.copy_from_slice(&record[..STATE_RECORD_HEADER_SIZE]);
        let header = StateRecordHeader::from_bytes(&hb);
        if let Err(reason) = header.validate() {
            return Some(StateAnchorEvent::Malformed { reason });
        }
        // The partial head anchor of a mid-run attach: a record whose `part` is not 0
        // belongs to a stream whose head this reader never saw. Monotone — the first `part == 0`
        // record opens the assembler for good.
        if self.armed {
            if header.part != 0 {
                self.discarded += 1;
                return None;
            }
            self.armed = false;
        }
        let payload =
            &record[STATE_RECORD_HEADER_SIZE..STATE_RECORD_HEADER_SIZE + header.len as usize];
        let key = (header.run_id, header.step, header.node_idx);

        if header.kind == RECORD_KIND_SKIP {
            // A skip is self-contained and AUTHORITATIVE over anything IN FLIGHT:
            // whatever was partially assembled for this anchor is void, and the writer
            // just said why.
            self.open.remove(&key);
            let (cause, detail) = decode_skip_payload(payload);
            // ...but NOT over an anchor already COMPLETED. See
            // `StateAnchorEvent::SkipAfterComplete`: the child publishes the final
            // record and then bumps its accounting word, so a kill between the two
            // leaves real data in the ring and a parent that will skip the same key.
            // The data wins; the skip is reported as the diagnostic it is.
            if self.completed.get(&(header.run_id, header.node_idx)) == Some(&header.step) {
                self.skips_after_complete += 1;
                return Some(StateAnchorEvent::SkipAfterComplete {
                    run_id: header.run_id,
                    step: header.step,
                    node_idx: header.node_idx,
                    cause,
                    detail,
                });
            }
            return Some(StateAnchorEvent::Skipped {
                run_id: header.run_id,
                step: header.step,
                node_idx: header.node_idx,
                cause,
                detail,
            });
        }

        let is_final = header.kind == RECORD_KIND_FINAL;
        match self.open.get_mut(&key) {
            None => {
                if header.part != 0 {
                    if !is_final {
                        self.open.insert(key, AnchorState::Voided);
                    }
                    return Some(torn(
                        &header,
                        TornCause::PartOutOfOrder {
                            expected: 0,
                            got: header.part,
                        },
                    ));
                }
                if !is_final && header.len as usize != STATE_RECORD_PAYLOAD {
                    self.open.insert(key, AnchorState::Voided);
                    return Some(torn(
                        &header,
                        TornCause::ShortChunk {
                            part: header.part,
                            len: header.len,
                        },
                    ));
                }
                if is_final {
                    // Completion precedence: remember the completion, so a post-mortem SKIP
                    // for this same anchor is read as the diagnostic it is rather than
                    // as a refusal that voids real data.
                    self.note_completed(header.run_id, header.step, header.node_idx);
                    return Some(StateAnchorEvent::Complete {
                        run_id: header.run_id,
                        step: header.step,
                        node_idx: header.node_idx,
                        parts: 1,
                        bytes: payload.to_vec(),
                    });
                }
                self.open.insert(
                    key,
                    AnchorState::Open {
                        next_part: 1,
                        buf: payload.to_vec(),
                    },
                );
                None
            }
            Some(AnchorState::Voided) => {
                // Already reported. The FINAL record closes the tombstone so a LATER
                // anchor for the same key starts clean.
                if is_final {
                    self.open.remove(&key);
                }
                None
            }
            Some(AnchorState::Open { next_part, buf }) => {
                if header.part != *next_part {
                    let cause = TornCause::PartOutOfOrder {
                        expected: *next_part,
                        got: header.part,
                    };
                    if is_final {
                        self.open.remove(&key);
                    } else {
                        self.open.insert(key, AnchorState::Voided);
                    }
                    return Some(torn(&header, cause));
                }
                if !is_final && header.len as usize != STATE_RECORD_PAYLOAD {
                    let cause = TornCause::ShortChunk {
                        part: header.part,
                        len: header.len,
                    };
                    self.open.insert(key, AnchorState::Voided);
                    return Some(torn(&header, cause));
                }
                buf.extend_from_slice(payload);
                if is_final {
                    let parts = *next_part + 1;
                    let bytes = match self.open.remove(&key) {
                        Some(AnchorState::Open { buf, .. }) => buf,
                        // Unreachable: we just matched `Open` under the same key.
                        _ => Vec::new(),
                    };
                    self.note_completed(header.run_id, header.step, header.node_idx);
                    return Some(StateAnchorEvent::Complete {
                        run_id: header.run_id,
                        step: header.step,
                        node_idx: header.node_idx,
                        parts,
                        bytes,
                    });
                }
                *next_part += 1;
                None
            }
        }
    }

    /// Close the stream: every anchor still open is TRUNCATED and reported, in
    /// deterministic key order. Anchors already reported torn are dropped silently.
    pub fn finish(&mut self) -> Vec<StateAnchorEvent> {
        let mut out = Vec::new();
        for ((run_id, step, node_idx), state) in std::mem::take(&mut self.open) {
            if let AnchorState::Open { next_part, .. } = state {
                out.push(StateAnchorEvent::Torn {
                    run_id,
                    step,
                    node_idx,
                    cause: TornCause::Truncated {
                        parts_seen: next_part,
                    },
                });
            }
        }
        out
    }

    /// A drain hit a LAP: every anchor in flight may have absorbed overwritten bytes
    /// (or be missing records outright), so all of them are dropped and the event is
    /// counted.
    ///
    /// Deliberately not a rollback: rolling back would mean cloning partially
    /// assembled anchors that can be hundreds of megabytes, and a lap already means
    /// the recording has failed loudly (`shm_ring` contract 2).
    fn note_torn_drain(&mut self) {
        self.open.clear();
        self.torn_drains += 1;
    }
}

fn torn(header: &StateRecordHeader, cause: TornCause) -> StateAnchorEvent {
    StateAnchorEvent::Torn {
        run_id: header.run_id,
        step: header.step,
        node_idx: header.node_idx,
        cause,
    }
}

/// Split a SKIP record's payload into its cause code and UTF-8 detail. A payload too
/// short to carry a code decodes as [`SkipCause::Unrecognized(0)`], and invalid UTF-8
/// detail is replaced rather than dropped — a voided anchor must always name
/// something.
fn decode_skip_payload(payload: &[u8]) -> (SkipCause, String) {
    if payload.len() < 4 {
        return (SkipCause::Unrecognized(0), String::new());
    }
    let cause = SkipCause::from_wire(u32::from_le_bytes(payload[0..4].try_into().unwrap()));
    (cause, String::from_utf8_lossy(&payload[4..]).into_owned())
}

// ===========================================================================
// Errors
// ===========================================================================

/// Errors from the state ring layer.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum StateRingError {
    /// An error from the underlying generic ring (create/open/validation/overrun).
    #[error(transparent)]
    Ring(#[from] ShmRingError),
    /// The node-identity manifest (the table shared with [`crate::trace_ring`]) could
    /// not be encoded or decoded.
    #[error("state ring node manifest error: {0}")]
    Manifest(#[from] TraceRingError),
    /// The opened ring's `record_size` is not [`STATE_RECORD_SIZE`] — it is not a
    /// state ring.
    #[error("state ring record_size is {actual} but a state record requires {expected}")]
    RecordSizeMismatch {
        /// The header's actual record size.
        actual: u32,
        /// [`STATE_RECORD_SIZE`].
        expected: u32,
    },
    /// The opened ring was NOT created under [`OverrunPolicy::Backpressure`]. A state
    /// ring under the wait-free policy silently laps a 1.09 M-record anchor, so this
    /// is refused at open rather than discovered as loss later.
    #[error("state ring '{name}' was created under overrun policy {actual} but a state ring requires Backpressure ({expected}) — an anchor through a wait-free ring is lost, not slow")]
    OverrunPolicyMismatch {
        /// The object name.
        name: String,
        /// The header's policy discriminant.
        actual: u32,
        /// [`OverrunPolicy::Backpressure`]'s discriminant.
        expected: u32,
    },
    /// A per-rank state-ring name could not be derived — see
    /// [`state_ring_tag`]'s refusals.
    #[error("cannot derive a per-rank state-ring name: {reason}")]
    UnnameableRing {
        /// Which rule refused, in a sentence an operator can act on.
        reason: String,
    },
}

/// Result alias for state ring operations.
pub type StateRingResult<T> = Result<T, StateRingError>;

// ===========================================================================
// Sizing
// ===========================================================================

/// Default state-ring data-region budget (64 MiB), matching the trace ring's.
///
/// It is a DEFAULT, not a sizing rule: the black-box recorder's retention floor
/// (`max(configured window, since-last-completed-anchor)`) is the real input, and a
/// deployment that anchors a 500 MB state needs a far larger ring — 2 × Σ state_bytes
/// × the 512/480 framing.
pub const DEFAULT_STATE_RING_BYTES: usize = 64 * 1024 * 1024;

/// The largest capacity a `u32` ring-capacity field can express as a power of two.
///
/// `2^32` does not fit, so `2^31` is the ceiling — and saturating AT it is what stops
/// a narrowing cast turning an over-large budget into ZERO (see
/// [`capacity_records_for_bytes`]).
pub const MAX_CAPACITY_RECORDS: u64 = 1 << 31;

/// Records that fit in `bytes`, rounded DOWN to a power of two (the ring capacity
/// must be one), SATURATED at [`MAX_CAPACITY_RECORDS`].
///
/// The saturation is not decoration: `capacity` is a `u32`, so a budget of 2 TiB or
/// more makes `bytes / 512` reach `2^32` and the narrowing cast silently produce
/// **0** — which `StateRingOwner::create` then rejects as "capacity must be > 0", an
/// answer that tells an operator asking for a huge ring nothing about what went
/// wrong. Saturating keeps this function's contract ("the largest power of two that
/// fits") true for every input.
///
/// RESIDUAL, stated rather than papered over: a ring at the ceiling is 1 TiB of
/// segment, so a caller who asks for one still fails — at `ftruncate`/`mmap`, with an
/// error naming the size it could not allocate. That is the truthful complaint; the
/// cast was not.
pub fn capacity_records_for_bytes(bytes: usize) -> u32 {
    prev_power_of_two((bytes / STATE_RECORD_SIZE as usize) as u64).min(MAX_CAPACITY_RECORDS) as u32
}

/// The default capacity in records: [`DEFAULT_STATE_RING_BYTES`] worth, rounded down
/// to a power of two (`2^17` = 131_072).
pub fn default_capacity_records() -> u32 {
    capacity_records_for_bytes(DEFAULT_STATE_RING_BYTES)
}

// ===========================================================================
// (arm tag, rank) → ring name — the multi-process rendezvous
// ===========================================================================

/// The rank NO state ring may ever be created under: the supervisor's
/// DEPARTURE-ring sentinel.
///
/// The supervisor's departure ring is a TRACE ring carrying one record per lost
/// worker and no node manifest at all — there is no node in it whose state could
/// be checkpointed. Re-stated here (rather than imported from `trace_ring`, which
/// does not define it either — `cerulion_cli_engine`'s supervisor mints it and
/// `cerulion_bagd` re-states it) and pinned against the ONE thing all three
/// really share: the value.
pub const STATE_RING_RESERVED_RANK: u32 = u32::MAX;

/// How many ranks a DISCOVERING recorder probes for (`0..STATE_RING_MAX_RANKS`).
///
/// A recorder learns the arm TAG and nothing else — under a `process_groups:` or
/// auto-derived deployment the rank COUNT is decided at run time, after the
/// recorder's argv was fixed — so per-rank ring names cannot be pre-listed and
/// the recorder probes instead. The ceiling bounds that probe: it is a `shm_open`
/// per rank per scan, so an unbounded sweep would be a syscall storm on the
/// recorder's drive loop.
///
/// 1024 is far above any real deployment (auto-partitioning derives one process per NODE in
/// the worst case, and a graph with a thousand nodes is not a shape anyone runs)
/// and far BELOW [`STATE_RING_RESERVED_RANK`], so the sweep can never probe the
/// departure sentinel — the exclusion is structural here, and separately enforced
/// at construction by [`state_ring_tag`].
pub const STATE_RING_MAX_RANKS: u32 = 1024;

// The two rules above must not be able to drift into each other: a ceiling that
// reached the sentinel would make the sweep probe a name a departure ring could
// legitimately own.
const _: () = assert!(
    STATE_RING_MAX_RANKS < STATE_RING_RESERVED_RANK,
    "the discovery sweep must never be able to probe the departure sentinel rank"
);
// And a ceiling BELOW any plausible deployment would silently drop its tail
// ranks — the inert-shipping direction of the same drift.
const _: () = assert!(
    STATE_RING_MAX_RANKS >= 64,
    "the discovery ceiling must be above any plausible deployment's rank count"
);

/// The ring TAG the graph process of `rank` creates its state ring under, for a
/// run armed with `arm_tag`.
///
/// # Why the name is DERIVED rather than handed over
///
/// The arm tag is the one token both halves already share — a recorder mints it
/// and creates [`crate::state_arm::MappedStateArm`] under it; every graph process
/// of the run opens that SAME word by that SAME tag (this is what makes
/// onset agreement arithmetic rather than a protocol). Deriving each rank's ring
/// name from `(tag, rank)` extends exactly that property to the data plane: a
/// recorder that knows the tag can NAME every rank's ring without being told how
/// many ranks there are, and a rank that knows the tag can create its ring without
/// being told where to publish.
///
/// The shape mirrors `cerulion_cli_engine`'s trace-ring discipline
/// (`cer_rec_{graph}_{sup_pid}_r{rank}`): a distinct prefix, the coordinating
/// token, then `_r{rank}` splitting the ranks of one deployment. Collision safety
/// rides on the arm tag, exactly as the trace ring's rides on the supervisor pid —
/// `cer_st_` and `cer_rec_` cannot collide, and two concurrent runs get two tags.
/// The tag is hashed into a fixed-length SHM object name by
/// [`crate::shm_ring::ring_shm_name`], so tag LENGTH is not a constraint here.
///
/// # Refusals
///
/// * a blank tag — an empty arm tag is what a script that computed no tag leaves
///   behind, and every rank would then derive the SAME name from nothing;
/// * [`STATE_RING_RESERVED_RANK`] — see its docs.
pub fn state_ring_tag(arm_tag: &str, rank: u32) -> StateRingResult<String> {
    if arm_tag.trim().is_empty() {
        return Err(StateRingError::UnnameableRing {
            reason: "the checkpoint arm tag is blank, so no per-rank state-ring name can be \
                     derived from it (every rank would derive the same name from nothing)"
                .to_string(),
        });
    }
    if rank == STATE_RING_RESERVED_RANK {
        return Err(StateRingError::UnnameableRing {
            reason: format!(
                "rank {rank} is the supervisor departure-ring sentinel and carries no node \
                 manifest, so it can never own a node-state ring"
            ),
        });
    }
    Ok(format!("cer_st_{arm_tag}_r{rank}"))
}

/// The POSIX SHM object name of the state ring `rank` owns under `arm_tag` — the
/// name a producer creates and a recorder opens. See [`state_ring_tag`].
pub fn state_ring_shm_name(arm_tag: &str, rank: u32) -> StateRingResult<String> {
    Ok(crate::shm_ring::ring_shm_name(&state_ring_tag(
        arm_tag, rank,
    )?))
}

/// The `run_id` every record of a run armed with `arm_tag` carries — FNV-1a 64 of
/// the tag.
///
/// [`StateAssembler`] groups parts by `(run_id, step, node_idx)`, so every RANK of
/// one run must stamp the same value or one graph state reassembles as several.
/// DERIVED from the tag rather than plumbed, for the reason the ring name is: the
/// tag is the token every rank already has, and a recorder can compute the same
/// number from the same tag without being told anything.
///
/// Nothing outside this system authorities the value — the restore path SELECTS by
/// it (`state_restore::RestoreRequest::run_id`) rather than validating it against
/// an external identity — so "agrees across ranks, computable by the reader" is
/// the whole contract, and a hash of the tag satisfies it exactly.
pub fn state_ring_run_id(arm_tag: &str) -> u64 {
    // The SAME fold `ring_shm_name` derives the ring's object name with — one
    // copy, in `shm_map`, since a hash two BINARIES must agree on is exactly
    // where two independently-written copies drift.
    crate::shm_map::fnv1a64(arm_tag.as_bytes())
}

/// How many CONSECUTIVE absent ranks end a discovery sweep.
///
/// Ranks are dense, so the sweep is really "walk up until the run runs out". The
/// tolerance is what keeps a HOLE — a rank that exists and published nothing —
/// from truncating the sweep and hiding every rank above it, which would turn a
/// reportable gap ([`missing_state_ring_ranks`]) into a silently short recording.
/// It is a small constant rather than [`STATE_RING_MAX_RANKS`] because the sweep
/// costs one `shm_open` per probed rank on the recorder's drive loop, and it runs
/// for the whole run.
pub const STATE_RING_PROBE_GAP_TOLERANCE: u32 = 8;

/// PURE: walk the rank space with `probe`, returning every rank that answered.
///
/// `probe(rank)` answers "does a state ring exist for this rank?" — in production
/// a `shm_open` of [`state_ring_shm_name`], in a test a closure over a hand set.
/// Separating the WALK from the probe is what makes the stopping rule testable:
/// it is the only part with a decision in it, and the decision (how long to keep
/// looking after the ranks appear to run out) is exactly the part that can be
/// silently wrong.
///
/// Stops after [`STATE_RING_PROBE_GAP_TOLERANCE`] consecutive misses, or at
/// [`STATE_RING_MAX_RANKS`]. Both bounds are real: without the first the sweep is
/// 1024 syscalls every scan for the whole run; without the second a change to the
/// first could make it unbounded.
pub fn scan_state_ring_ranks(mut probe: impl FnMut(u32) -> bool) -> Vec<u32> {
    let mut found = Vec::new();
    let mut misses = 0u32;
    for rank in 0..STATE_RING_MAX_RANKS {
        if probe(rank) {
            found.push(rank);
            misses = 0;
        } else {
            misses += 1;
            if misses >= STATE_RING_PROBE_GAP_TOLERANCE {
                break;
            }
        }
    }
    found
}

/// What [`unlink_stale_state_rings`] found under one arm tag.
///
/// Both halves are reported because they are different facts with different
/// consequences: a `removed` rank is a hazard this run CLOSED, a `refused` one is a
/// hazard it could not close and a recorder armed with this tag may still adopt.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct StaleRingSweep {
    /// Ranks whose leftover ring NAME this sweep removed.
    pub removed: Vec<u32>,
    /// Ranks whose leftover ring exists and could NOT be unlinked, with the reason.
    pub refused: Vec<(u32, String)>,
}

impl StaleRingSweep {
    /// Did the sweep find anything at all? (`false` is the ordinary case.)
    pub fn found_anything(&self) -> bool {
        !self.removed.is_empty() || !self.refused.is_empty()
    }
}

/// Remove every per-rank state-ring NAME left over under `arm_tag` by an earlier
/// run, and report what was found.
///
/// Call this from the process that CREATES `arm_tag`'s arm word, BEFORE any rank
/// creates its ring — the moment this run takes ownership of the tag.
///
/// # The hazard this closes
///
/// [`MappedStateArm::create_owned`](crate::state_arm::MappedStateArm::create_owned)
/// is UNLINK-FIRST, so arming under a tag already ASSERTS that this run owns that
/// tag: it destroys whatever word was there. The per-rank ring names under the same
/// tag were the one part of that namespace left un-asserted, and they are the part
/// that carries DATA.
///
/// So under a REUSED explicit `CERULION_STATE_ARM_TAG`, a crashed earlier run's
/// `cer_st_{tag}_r{rank}` objects survive in SHM, and every rank this run does NOT
/// create a ring for leaves one of them standing:
///
/// * a rank whose worker REFUSED the plane (the per-worker RAM gate) — the
///   deployment is meant to report that as a HOLE
///   ([`missing_state_ring_ranks`]), and instead the hole is filled with a dead
///   run's records;
/// * a rank this deployment does not HAVE at all — the earlier run had more
///   workers, or was multi-process where this one is a monolith.
///
/// Nothing downstream can reject those records: [`state_ring_run_id`] is derived
/// from the TAG, so a leaked ring carries a `run_id` IDENTICAL to this run's and
/// the assembler groups them together. Previous-run node state in this run's
/// captures is replay not identical to live (Principle #7), which is the one thing
/// this plane exists to make true.
///
/// # Why the walk is [`scan_state_ring_ranks`] and not `0..STATE_RING_MAX_RANKS`
///
/// A recorder can only adopt what its OWN sweep reaches, and that sweep is this
/// function. Clearing exactly the set the recorder can reach is therefore total by
/// construction, and it costs one syscall per probed rank — on the overwhelmingly
/// common DERIVED tag (unique per run, so nothing can be there) that is
/// [`STATE_RING_PROBE_GAP_TOLERANCE`] syscalls, once, at arm time.
///
/// A rank that exists but cannot be unlinked still counts as PRESENT for the walk,
/// so one un-removable name can never truncate the sweep and hide the leaks above
/// it.
///
/// # Never a refusal
///
/// This reports; it does not fail. A leftover name is somebody else's crash, and a
/// graph must not decline to run because of one — the caller logs what was found.
///
/// # Scope: a REFUSED plane sweeps nothing, and does not need to
///
/// The caller runs this only once it has really created the word, so a run whose
/// plane the kill switch or the RAM gate declined leaves every leftover name
/// standing. That is not a hole: such a run hands NO tag to its recorder (both
/// always-on spawn sites read the ARMED binding), so nothing sweeps for those names
/// and nothing can adopt them. They are somebody else's crash residue, still
/// waiting for a run that takes the tag over.
pub fn unlink_stale_state_rings(arm_tag: &str) -> StaleRingSweep {
    let mut sweep = StaleRingSweep::default();
    scan_state_ring_ranks(|rank| {
        let Ok(shm_name) = state_ring_shm_name(arm_tag, rank) else {
            return false;
        };
        let Ok(c_name) = std::ffi::CString::new(shm_name) else {
            return false;
        };
        // SAFETY: FFI unlink of a name this function derived; the pointer is a
        // valid NUL-terminated C string for the duration of the call.
        if unsafe { libc::shm_unlink(c_name.as_ptr()) } == 0 {
            sweep.removed.push(rank);
            return true;
        }
        let err = std::io::Error::last_os_error();
        if err.kind() == std::io::ErrorKind::NotFound {
            // The ordinary case: nothing was there.
            return false;
        }
        sweep.refused.push((rank, err.to_string()));
        true
    });
    sweep
}

/// PURE: the ranks BELOW the highest one discovered that produced no ring.
///
/// A deployment's ranks are DENSE — `cerulion_cli_engine`'s planner numbers
/// workers `0..n` — so a hole in the discovered set is not an absence of
/// information, it is EVIDENCE that a rank which exists failed to publish a ring.
/// That matters because a graph-wide anchor is all-or-nothing across workers:
/// a rank contributing nothing makes every anchor of the run partial, and
/// without this the recorder would simply see fewer rings than there are ranks and
/// have no way to know it.
///
/// Only holes BELOW the maximum are reported, because nothing here can know how
/// many ranks the run has: rank `n+1` being absent is indistinguishable from the
/// deployment having `n+1` ranks. A hole is the one thing that is provable from
/// the recorder's side alone.
///
/// # Why the density argument is scoped to the ADDRESSABLE rank space
///
/// A rank at or above [`STATE_RING_MAX_RANKS`] cannot witness density, because the
/// sweep that establishes density can never probe it: it is a rank whose ring this
/// recorder could not have found even if it existed, so "every rank below it must
/// exist" is a claim about a space nothing walked.
///
/// This is a BOUND, not a nicety. The ranks reaching this function come from ring
/// HEADERS — `StateCoverage::ranks_discovered` is built from `ring.rank()`, read
/// out of a segment ANOTHER process wrote — so the maximum is attacker- (or
/// corruption-) controlled. Taking `u32::MAX` as the density max asks for a
/// 4.29-billion-element `Vec<u32>`: ~17 GB allocated on the recorder's writer
/// thread at finalize, from four bytes of foreign SHM. Ignoring out-of-range ranks
/// caps the result at [`STATE_RING_MAX_RANKS`] entries and can only ever REMOVE
/// fabricated holes — every rank of a real deployment is inside the sweep's
/// ceiling, or the recorder never saw it in the first place.
pub fn missing_state_ring_ranks(found: &[u32]) -> Vec<u32> {
    missing_state_ring_ranks_within(found, found)
}

/// PURE: [`missing_state_ring_ranks`] with the two halves of its argument SEPARATED —
/// which rank space was WALKED, and which ranks are known to have published.
///
/// # Why they are not the same set
///
/// The density argument has two premises, and only one of them is about the ranks
/// this recording drained. "Rank `k` exists" is inferred from having SWEPT past it
/// and found a higher rank — a claim only the discovery walk can make. "Rank `k`
/// published nothing" is the absence of a ring, over EVERY ring the recording holds,
/// however it was obtained.
///
/// Conflating them breaks in both directions, and both are reachable:
///
/// * `--state-ring` takes an explicit, hand-picked list of SHM object names, and the
///   rank comes from each ring's HEADER — the operator never even sees it. Declaring
///   only rank 1's ring then "proves" rank 0 existed and published nothing, and the
///   bag is marked INCOMPLETE for a shape the operator chose. Nothing walked a rank
///   space here, so `swept` is empty and no hole can be claimed.
/// * With discovery ON, a ring ALSO declared explicitly is never re-adopted by its
///   derived name (it is already held), so it is absent from `swept` while being
///   very much present. Counting it as a hole would invent the opposite error.
///
/// So: the maximum comes from `swept`, membership from `known`.
pub fn missing_state_ring_ranks_within(swept: &[u32], known: &[u32]) -> Vec<u32> {
    let Some(max) = swept
        .iter()
        .copied()
        .filter(|r| *r < STATE_RING_MAX_RANKS)
        .max()
    else {
        return Vec::new();
    };
    (0..max).filter(|r| !known.contains(r)).collect()
}

// ===========================================================================
// Owner / producer / sink / consumer
// ===========================================================================

/// The state-ring owner: creates the SHM segment sized for state records, writes the
/// node-id manifest, and mints the single [`StateRingProducer`].
#[derive(Debug)]
#[must_use = "the owner shm_unlinks the ring name on drop — bind it to a named local"]
pub struct StateRingOwner {
    inner: ShmRingOwner,
    node_ids: Vec<String>,
    run_id: u64,
}

impl StateRingOwner {
    /// Create a state ring for `tag` holding `capacity_records` records, tagged with
    /// the producer `rank` and the `run_id` every record will carry, with `node_ids`
    /// as the manifest table.
    ///
    /// ALWAYS [`OverrunPolicy::Backpressure`] — there is deliberately no constructor
    /// that selects anything else (see the module doc).
    ///
    /// # Refusals
    ///
    /// [`STATE_RING_RESERVED_RANK`], for the reason [`state_ring_tag`] refuses it —
    /// and this is the SECOND guard, not a duplicate of it. `state_ring_tag` gates
    /// the NAME; this gates the ring HEADER, and the two are independent arguments
    /// here because `tag` and `rank` are separate parameters: a caller may derive a
    /// perfectly legal name for rank 3 and still stamp `u32::MAX` into the header.
    /// Such a ring is discoverable (its name is rank 3's) but reports a rank the
    /// deployment does not have, and every reader downstream takes the HEADER's word
    /// — `StateCoverage::ranks_discovered` is built from `ring.rank()`, so one such
    /// ring makes the density argument in [`missing_state_ring_ranks`] claim four
    /// billion absent ranks.
    pub fn create(
        tag: &str,
        capacity_records: u32,
        rank: u32,
        run_id: u64,
        node_ids: &[&str],
    ) -> StateRingResult<Self> {
        if rank == STATE_RING_RESERVED_RANK {
            return Err(StateRingError::UnnameableRing {
                reason: format!(
                    "rank {rank} is the supervisor departure-ring sentinel and carries no \
                     node manifest, so it can never own a node-state ring"
                ),
            });
        }
        let manifest = encode_manifest(node_ids)?;
        let inner = ShmRingOwner::create_with_policy(
            tag,
            STATE_RECORD_SIZE,
            capacity_records,
            rank,
            &manifest,
            OverrunPolicy::Backpressure,
        )?;
        Ok(Self {
            inner,
            node_ids: node_ids.iter().map(|s| s.to_string()).collect(),
            run_id,
        })
    }

    /// Mint the single [`StateRingProducer`] (`None` if already minted).
    pub fn producer(&mut self) -> Option<StateRingProducer> {
        let run_id = self.run_id;
        self.inner
            .producer()
            .map(|inner| StateRingProducer { inner, run_id })
    }

    /// The POSIX SHM object name — hand this to the recorder process.
    pub fn name(&self) -> &str {
        self.inner.name()
    }

    /// The producer rank recorded in the ring header.
    pub fn rank(&self) -> u32 {
        self.inner.rank()
    }

    /// The create-generation.
    pub fn generation(&self) -> u64 {
        self.inner.generation()
    }

    /// The capacity in records.
    pub fn capacity(&self) -> u32 {
        self.inner.capacity()
    }

    /// The run every record from this ring carries.
    pub fn run_id(&self) -> u64 {
        self.run_id
    }

    /// The node-id manifest table.
    pub fn node_ids(&self) -> &[String] {
        &self.node_ids
    }

    /// The ring's overrun policy — always [`OverrunPolicy::Backpressure`].
    pub fn overrun_policy(&self) -> OverrunPolicy {
        self.inner.overrun_policy()
    }
}

/// The state-ring producer: the writer role a checkpoint `fork` child takes for its
/// lifetime.
#[derive(Debug)]
#[must_use = "a producer with no pushes records nothing"]
pub struct StateRingProducer {
    inner: ShmRingProducer,
    run_id: u64,
}

impl StateRingProducer {
    /// Open a [`StateRingSink`] for one node's state at one anchor step.
    pub fn sink(&mut self, step: u64, node_idx: u32) -> StateRingSink<'_> {
        let run_id = self.run_id;
        StateRingSink {
            chunker: StateChunker::new(run_id, step, node_idx),
            producer: self,
        }
    }

    /// Publish a SKIP record naming why this anchor (or this node's part of it) was
    /// voided.
    pub fn push_skip(&mut self, step: u64, node_idx: u32, cause: SkipCause, detail: &str) {
        let record = encode_skip_record(self.run_id, step, node_idx, cause, detail);
        self.push_record(&record);
    }

    /// Publish a SKIP record ONLY if it needs no wait — `true` when it landed.
    ///
    /// The refusal path's own record must never be the thing that blocks. A state
    /// ring is always `BACKPRESSURE` (see [`StateRingOwner::create`]), so
    /// [`push_skip`](Self::push_skip) waits up to five seconds per record and then
    /// laps; a boundary that has just DECLINED an anchor because the recorder is
    /// behind would otherwise spend that wait, once per node, on the node thread —
    /// turning "the recorder fell behind" into "the control loop stopped".
    ///
    /// A refusal that cannot be published costs the reader the CAUSE, never the
    /// fact: a node with no record at a cadence is already
    /// `StateCoverage::nodes_without_anchor`, which escalates on its own. Callers
    /// must still report the loss — see the boundary's unpublished-skip counter.
    pub fn try_push_skip(
        &mut self,
        step: u64,
        node_idx: u32,
        cause: SkipCause,
        detail: &str,
    ) -> bool {
        let record = encode_skip_record(self.run_id, step, node_idx, cause, detail);
        self.inner.try_push(&record)
    }

    /// Publish one pre-built record. The seam [`StateRingSink`] writes through, and
    /// the one a caller with its own encoder (or a test building a deliberately
    /// broken stream) uses.
    pub fn push_record(&mut self, record: &[u8; STATE_RECORD_SIZE as usize]) {
        self.inner.push(record);
    }

    /// The run every record from this producer carries.
    pub fn run_id(&self) -> u64 {
        self.run_id
    }

    /// Total records published so far.
    pub fn pushed(&self) -> u64 {
        self.inner.pushed()
    }

    /// How many pushes had to WAIT for room (Principle #3 — the backpressure is
    /// observable without a log, which also keeps the push path safe in a `fork`
    /// child).
    pub fn backpressure_waits(&self) -> u64 {
        self.inner.backpressure_waits()
    }

    /// Total nanoseconds spent waiting for room.
    pub fn backpressure_wait_nanos(&self) -> u64 {
        self.inner.backpressure_wait_nanos()
    }

    /// How many waits EXPIRED and lapped anyway — the degradation from "no loss" to
    /// "loud loss".
    pub fn backpressure_wait_timeouts(&self) -> u64 {
        self.inner.backpressure_wait_timeouts()
    }

    /// How many 512-byte records may be pushed RIGHT NOW without blocking.
    ///
    /// Always `Some` for a state ring — [`StateRingOwner::create`] admits no
    /// policy but `BACKPRESSURE` — but the `Option` is kept rather than
    /// unwrapped so the answer stays the generic layer's, not a claim this
    /// layer makes on its behalf.
    ///
    /// The checkpoint carrier asks this BEFORE its inline walk: a blocking push
    /// on the node thread would stall the robot for the wait timeout, and the
    /// arena's byte bound cannot see a full ring. Converting the answer from
    /// records into "will this anchor fit" is [`parts_for_len`]'s job.
    pub fn free_records(&self) -> Option<u64> {
        self.inner.free_records()
    }

    /// Override the per-push backpressure wait ceiling.
    pub fn set_backpressure_wait_timeout(&mut self, timeout: std::time::Duration) {
        self.inner.set_backpressure_wait_timeout(timeout);
    }

    /// Re-read the LOCAL write cursor from the header — the repair for a producer
    /// copy duplicated by `fork`.
    ///
    /// The contract is the underlying ring's: the producer role is handed to the
    /// child for the child's lifetime, the parent touches the ring not at all between
    /// `fork` and reap, and calls this on reap.
    pub fn resync_after_fork(&mut self) -> u64 {
        self.inner.resync_after_fork()
    }
}

/// A capture sink over the state ring: the destination the FORK-CARRIER path
/// streams a node's encoded state into.
///
/// # Which path this sink serves, and why it is UNBUDGETED
///
/// The design has two sinks feeding ONE encoder and emitting IDENTICAL bytes, so
/// which carrier a node took is invisible in the bag:
///
/// - the INLINE carrier's `BoundedSink` — a hard `CAPTURE_INLINE_BUDGET_BYTES`
///   capacity over a pre-allocated arena, shared across the boundary's nodes, which
///   refuses past its cap;
/// - **this one**, the fork child's, which has NO budget. The ring's whole argument is
///   that chunking deletes the size refusal — "a bigger state is more records.
///   Arithmetic, not policy" — and the ring's `Backpressure` mode means a writer that
///   outruns the recorder WAITS rather than failing. So a write here never returns
///   "full": [`append`](Self::append) is infallible, and the
///   `StateSink` trait impl is one line (`self.append(bytes); Ok(())`) with
///   the trait's own defaults for `remaining_hint` (`None` — unbounded) and `refuse`
///   (a no-op).
///
/// `remaining_hint() == None` is a real cost and it is the design's: a hash-like
/// container's canonical order needs a sort index built before the first byte, and in
/// the child that index is paid in full — the RAM budget covers exactly that
/// (240 MB for a 30 M-entry map). The inline sink refuses early instead; the child
/// pays, which is what it is for.
///
/// Dropping the sink without [`finish`](Self::finish) leaves the anchor TRUNCATED and
/// the reader says so — see [`StateChunker`].
#[derive(Debug)]
#[must_use = "a sink that is never written to and never finished records nothing"]
pub struct StateRingSink<'a> {
    chunker: StateChunker,
    producer: &'a mut StateRingProducer,
}

impl StateRingSink<'_> {
    /// Append `bytes` to this anchor, pushing whole records as they fill.
    ///
    /// Infallible by design (see the type docs): under backpressure the ring WAITS
    /// for room rather than refusing, and a wait that expires laps — which the
    /// consumer reports loudly — instead of silently dropping a record.
    pub fn append(&mut self, bytes: &[u8]) {
        let Self { chunker, producer } = self;
        chunker.append(bytes, &mut |record| producer.push_record(record));
    }

    /// Emit the tail as the FINAL record; returns the anchor's part count.
    pub fn finish(self) -> u32 {
        let Self { chunker, producer } = self;
        chunker.finish(&mut |record| producer.push_record(record))
    }

    /// Records emitted so far.
    pub fn parts_emitted(&self) -> u32 {
        self.chunker.parts_emitted()
    }
}

/// The one-line impl the type docs above predicted.
///
/// Both defaults are taken deliberately, and they are the whole difference
/// between this sink and the inline carrier's `BoundedSink`:
///
/// * `remaining_hint` stays `None` — UNBOUNDED. That is the point of the fork
///   carrier: a hash-like container may build its full sort index here, because
///   the child is the process that is allowed to pay for it (the RAM budget covers it).
///   Returning a bound would make the child refuse exactly the giants it exists
///   to serve.
/// * `refuse` stays a no-op, for the same reason: there is no budget to latch
///   against, and a sink that cannot refuse cannot be asked to charge anyone.
///
/// `write` is infallible because the ring WAITS rather than dropping (the
/// `BACKPRESSURE` mode). That wait is safe HERE — the child is a process nobody
/// is waiting for — and it is precisely why the boundary's inline half must
/// never push through this sink without asking `free_records` first.
impl crate::state::StateSink for StateRingSink<'_> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), crate::state::SinkFull> {
        self.append(bytes);
        Ok(())
    }
}

/// The state-ring consumer — reassembles anchors (for `bagd`, tools and the restore
/// path) and passes through the zero-copy [`drain_slices`](Self::drain_slices) /
/// [`commit`](Self::commit) pair for a `writev` recorder.
#[derive(Debug)]
#[must_use = "a consumer that is never drained reads nothing"]
pub struct StateRingConsumer {
    inner: ShmRingConsumer,
    node_ids: Vec<String>,
}

impl StateRingConsumer {
    /// STRICT-open a state ring by its object name, from its START.
    pub fn open(shm_name: &str) -> StateRingResult<Self> {
        Self::from_inner(ShmRingConsumer::open(shm_name)?)
    }

    /// STRICT-open a state ring at the producer's CURRENT write cursor — the mid-run
    /// attach seam (the trace ring's shape). Pair with [`StateAssembler::armed`], which
    /// discards the partial head anchor this cursor lands inside.
    pub fn open_at_live(shm_name: &str) -> StateRingResult<Self> {
        Self::from_inner(ShmRingConsumer::open_at_live(shm_name)?)
    }

    /// The shared tail of both opens: validate the record size AND the overrun
    /// policy, then decode the node-id manifest. ONE body, so the two entry points
    /// cannot drift in what they validate.
    fn from_inner(inner: ShmRingConsumer) -> StateRingResult<Self> {
        if inner.record_size() != STATE_RECORD_SIZE {
            return Err(StateRingError::RecordSizeMismatch {
                actual: inner.record_size(),
                expected: STATE_RECORD_SIZE,
            });
        }
        let expected = OverrunPolicy::Backpressure.as_wire();
        if inner.overrun_policy() != expected {
            return Err(StateRingError::OverrunPolicyMismatch {
                name: inner.name().to_string(),
                actual: inner.overrun_policy(),
                expected,
            });
        }
        let node_ids = decode_manifest(inner.manifest())?;
        Ok(Self { inner, node_ids })
    }

    /// Drain every currently-available record through `assembler`, appending the
    /// events it produces to `out`, then commit. Returns the number of RECORDS
    /// consumed off the ring (not the number of events — one anchor is many records).
    ///
    /// A LAP is reported on BOTH of the ring's detection points and both mean the
    /// same thing to the reassembly — records were irrecoverably overwritten, so
    /// every anchor in flight has a hole:
    ///
    /// - UP FRONT (`drain_slices` finds the consumer already lapped): nothing is
    ///   read, the assembler drops its open anchors and counts the event;
    /// - MID-READ (`commit` finds the producer lapped into the drained region): the
    ///   events appended by THIS call are truncated away as well, because the bytes
    ///   they were decoded from may have been overwritten.
    ///
    /// Either way [`StateAssembler::torn_drains`] counts it. Handing on a partially
    /// assembled anchor after a lap would be exactly the short-blob-served-as-
    /// complete failure this layer exists to prevent.
    pub fn drain(
        &mut self,
        assembler: &mut StateAssembler,
        out: &mut Vec<StateAnchorEvent>,
    ) -> StateRingResult<usize> {
        const RS: usize = STATE_RECORD_SIZE as usize;
        let original_len = out.len();
        let consumed = match self.inner.drain_slices() {
            Ok((a, b)) => {
                for chunk in a.as_chunks::<RS>().0.iter().chain(b.as_chunks::<RS>().0) {
                    if let Some(ev) = assembler.feed(chunk) {
                        out.push(ev);
                    }
                }
                (a.len() + b.len()) / RS
            }
            Err(e) => {
                assembler.note_torn_drain();
                return Err(e.into());
            }
        };
        if let Err(e) = self.inner.commit(consumed as u64) {
            out.truncate(original_len);
            assembler.note_torn_drain();
            return Err(e.into());
        }
        Ok(consumed)
    }

    /// Zero-copy passthrough: the unread region as up-to-2 raw byte slices into the
    /// mapping (for a `writev` recorder). Pair with [`commit`](Self::commit).
    ///
    /// DELIBERATELY ring-level (`ShmRingResult`) — a 1:1 passthrough with no
    /// state-layer semantics added, exactly like the trace ring's.
    pub fn drain_slices(&mut self) -> ShmRingResult<(&[u8], &[u8])> {
        self.inner.drain_slices()
    }

    /// Zero-copy passthrough: advance the read cursor, re-validating no torn drain.
    pub fn commit(&mut self, n_records: u64) -> ShmRingResult<()> {
        self.inner.commit(n_records)
    }

    /// Unread record count (`> capacity` signals a lap).
    pub fn available(&self) -> u64 {
        self.inner.available()
    }

    /// The consumer's local read cursor.
    pub fn read_cursor(&self) -> u64 {
        self.inner.read_cursor()
    }

    /// The producer's write cursor as this consumer last observed it.
    pub fn write_cursor(&self) -> u64 {
        self.inner.write_cursor()
    }

    /// The decoded node-id manifest table — what a record's `node_idx` indexes.
    pub fn node_ids(&self) -> &[String] {
        &self.node_ids
    }

    /// Resolve a record's `node_idx` through the manifest.
    pub fn node_id(&self, node_idx: u32) -> Option<&str> {
        self.node_ids.get(node_idx as usize).map(|s| s.as_str())
    }

    /// The producer rank from the ring header (rank rides the header, not the
    /// record).
    pub fn rank(&self) -> u32 {
        self.inner.rank()
    }

    /// The create-generation from the header (restart detection).
    pub fn generation(&self) -> u64 {
        self.inner.generation()
    }

    /// The per-record size from the header (always [`STATE_RECORD_SIZE`] here).
    pub fn record_size(&self) -> u32 {
        self.inner.record_size()
    }

    /// The object name this consumer opened.
    pub fn name(&self) -> &str {
        self.inner.name()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Collect the records a chunker emits, so the pure writer half is testable with
    /// no SHM at all.
    fn collect(blob: &[u8], run_id: u64, step: u64, node_idx: u32) -> Vec<Vec<u8>> {
        let mut out: Vec<Vec<u8>> = Vec::new();
        let mut ch = StateChunker::new(run_id, step, node_idx);
        ch.append(blob, &mut |r| out.push(r.to_vec()));
        ch.finish(&mut |r| out.push(r.to_vec()));
        out
    }

    /// A blob whose byte at index `i` is its own low byte — its own oracle.
    fn blob(n: usize) -> Vec<u8> {
        (0..n).map(|i| (i % 251) as u8).collect()
    }

    #[test]
    fn record_geometry_is_a_format_contract() {
        assert_eq!(STATE_RECORD_SIZE, 512);
        assert_eq!(STATE_RECORD_HEADER_SIZE, 32);
        assert_eq!(STATE_RECORD_PAYLOAD, 480);
        assert_eq!(std::mem::size_of::<StateRecordHeader>(), 32);
    }

    #[test]
    fn header_byte_layout_oracle() {
        let h = StateRecordHeader {
            run_id: 0x0102_0304_0506_0708,
            step: 0x1112_1314_1516_1718,
            node_idx: 0x2122_2324,
            part: 0x3132_3334,
            kind: RECORD_KIND_FINAL,
            len: 0x0000_01E0, // 480
        };
        let expected: [u8; 32] = [
            0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, // run_id
            0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11, // step
            0x24, 0x23, 0x22, 0x21, // node_idx
            0x34, 0x33, 0x32, 0x31, // part
            0x02, 0x00, 0x00, 0x00, // kind = FINAL
            0xE0, 0x01, 0x00, 0x00, // len = 480
        ];
        assert_eq!(h.as_bytes(), expected, "header layout is a format contract");
        assert_eq!(
            StateRecordHeader::from_bytes(&expected),
            h,
            "from_bytes is the total inverse of as_bytes"
        );
    }

    #[test]
    fn a_record_zero_fills_its_unused_tail_so_two_captures_are_byte_identical() {
        let h = StateRecordHeader {
            run_id: 7,
            step: 3,
            node_idx: 1,
            part: 0,
            kind: RECORD_KIND_FINAL,
            len: 3,
        };
        let rec = encode_record(&h, &[0xAA, 0xBB, 0xCC]);
        assert_eq!(&rec[32..35], &[0xAA, 0xBB, 0xCC]);
        assert!(
            rec[35..].iter().all(|&b| b == 0),
            "the tail past `len` must be zero — these bytes land in the bag"
        );
    }

    // THE chunk-count arithmetic (a prime mutation target). Hand oracle, boundaries
    // on both sides, plus the number stated for the flagship 500 MB anchor.
    #[test]
    fn parts_for_len_is_exact_at_every_boundary() {
        assert_eq!(parts_for_len(0), 1, "an empty state is still ONE record");
        assert_eq!(parts_for_len(1), 1);
        assert_eq!(parts_for_len(479), 1);
        assert_eq!(parts_for_len(480), 1, "exactly full ⇒ no trailing record");
        assert_eq!(parts_for_len(481), 2);
        assert_eq!(parts_for_len(959), 2);
        assert_eq!(parts_for_len(960), 2);
        assert_eq!(parts_for_len(961), 3);
        // The ring's own arithmetic: a 500 MB anchor is ~1.09 M records.
        assert_eq!(parts_for_len(500 * 1024 * 1024), 1_092_267);
    }

    #[test]
    fn the_chunker_emits_exactly_parts_for_len_records_at_every_boundary() {
        for n in [0usize, 1, 479, 480, 481, 960, 961, 2400, 2401] {
            let recs = collect(&blob(n), 1, 2, 3);
            assert_eq!(
                recs.len() as u64,
                parts_for_len(n as u64),
                "blob of {n} bytes must become parts_for_len({n}) records"
            );
            // Every record but the last is a FULL chunk; the last is FINAL.
            for (i, r) in recs.iter().enumerate() {
                let mut hb = [0u8; 32];
                hb.copy_from_slice(&r[..32]);
                let h = StateRecordHeader::from_bytes(&hb);
                assert_eq!(h.part, i as u32, "parts are 0-based and contiguous");
                if i + 1 == recs.len() {
                    assert_eq!(h.kind, RECORD_KIND_FINAL);
                } else {
                    assert_eq!(h.kind, RECORD_KIND_CHUNK);
                    assert_eq!(
                        h.len as usize, STATE_RECORD_PAYLOAD,
                        "a non-final chunk is always full"
                    );
                }
            }
        }
    }

    /// Feed a record vector to an assembler and return every event, finish included.
    fn assemble(records: &[Vec<u8>], mut asm: StateAssembler) -> Vec<StateAnchorEvent> {
        let mut out = Vec::new();
        for r in records {
            if let Some(ev) = asm.feed(r) {
                out.push(ev);
            }
        }
        out.extend(asm.finish());
        out
    }

    #[test]
    fn a_multi_record_blob_round_trips_byte_identically() {
        // 2.5 records: two full chunks and a short final one.
        let want = blob(1100);
        let recs = collect(&want, 0xABCD, 42, 7);
        assert_eq!(recs.len(), 3);
        let events = assemble(&recs, StateAssembler::passthrough());
        assert_eq!(
            events,
            vec![StateAnchorEvent::Complete {
                run_id: 0xABCD,
                step: 42,
                node_idx: 7,
                parts: 3,
                bytes: want,
            }],
        );
    }

    #[test]
    fn a_boundary_exact_blob_round_trips_in_exactly_its_own_records() {
        let want = blob(960);
        let recs = collect(&want, 1, 1, 1);
        assert_eq!(recs.len(), 2, "960 == 2 * 480, no trailing empty record");
        match assemble(&recs, StateAssembler::passthrough()).as_slice() {
            [StateAnchorEvent::Complete { parts, bytes, .. }] => {
                assert_eq!(*parts, 2);
                assert_eq!(bytes, &want);
            }
            other => panic!("expected one Complete, got {other:?}"),
        }
        // One byte over: the same blob plus a byte becomes one more record.
        let want = blob(961);
        let recs = collect(&want, 1, 1, 1);
        assert_eq!(recs.len(), 3);
        match assemble(&recs, StateAssembler::passthrough()).as_slice() {
            [StateAnchorEvent::Complete { parts, bytes, .. }] => {
                assert_eq!(*parts, 3);
                assert_eq!(bytes, &want);
            }
            other => panic!("expected one Complete, got {other:?}"),
        }
    }

    #[test]
    fn an_empty_state_is_a_complete_anchor_not_an_absent_one() {
        let recs = collect(&[], 5, 9, 2);
        assert_eq!(recs.len(), 1);
        assert_eq!(
            assemble(&recs, StateAssembler::passthrough()),
            vec![StateAnchorEvent::Complete {
                run_id: 5,
                step: 9,
                node_idx: 2,
                parts: 1,
                bytes: Vec::new(),
            }]
        );
    }

    // THE torn-detection arm (a prime mutation target): a missing MIDDLE chunk must
    // be reported, and NO short blob may be served for that anchor.
    #[test]
    fn an_omitted_middle_chunk_is_torn_and_no_short_blob_is_served() {
        let want = blob(1500); // 4 records: 3 full + 1 short final
        let recs = collect(&want, 1, 1, 1);
        assert_eq!(recs.len(), 4);
        let mut kept = recs.clone();
        kept.remove(2); // drop part 2

        let events = assemble(&kept, StateAssembler::passthrough());
        assert_eq!(
            events,
            vec![StateAnchorEvent::Torn {
                run_id: 1,
                step: 1,
                node_idx: 1,
                cause: TornCause::PartOutOfOrder {
                    expected: 2,
                    got: 3
                },
            }],
            "the gap is named AT the gap"
        );
        assert!(
            !events
                .iter()
                .any(|e| matches!(e, StateAnchorEvent::Complete { .. })),
            "a torn anchor must NEVER be served as a Complete short blob"
        );
    }

    #[test]
    fn a_missing_final_record_is_reported_truncated_at_finish() {
        let want = blob(1500);
        let mut recs = collect(&want, 3, 4, 5);
        recs.pop(); // lose the FINAL record
        assert_eq!(
            assemble(&recs, StateAssembler::passthrough()),
            vec![StateAnchorEvent::Torn {
                run_id: 3,
                step: 4,
                node_idx: 5,
                cause: TornCause::Truncated { parts_seen: 3 },
            }]
        );
    }

    /// Build one record by hand — for the streams `StateChunker` cannot produce.
    fn hand_record(step: u64, node_idx: u32, part: u32, kind: u32, len: usize) -> Vec<u8> {
        let header = StateRecordHeader {
            run_id: 1,
            step,
            node_idx,
            part,
            kind,
            len: len as u32,
        };
        encode_record(&header, &blob(len)).to_vec()
    }

    // A non-final chunk that is not FULL is a shape the chunker cannot produce, so the
    // stream is corrupt. BOTH arms are driven — the guard exists twice, once for the
    // record that OPENS an anchor and once for a record arriving into an open one, and
    // a test that feeds only a part-0 record never reaches the second (with
    // only that arm, neutralising the open-anchor guard fails
    // NOTHING).
    #[test]
    fn a_short_non_final_chunk_is_torn_whether_it_opens_the_anchor_or_lands_mid_stream() {
        // ARM 1: the record that opens the anchor.
        let mut asm = StateAssembler::passthrough();
        assert_eq!(
            asm.feed(&hand_record(1, 0, 0, RECORD_KIND_CHUNK, 10)),
            Some(StateAnchorEvent::Torn {
                run_id: 1,
                step: 1,
                node_idx: 0,
                cause: TornCause::ShortChunk { part: 0, len: 10 },
            })
        );

        // ARM 2: a record arriving into an anchor that is already open. Its part index
        // is the one the stream owed, so nothing else can catch it — without the guard
        // the short payload is appended and the anchor completes SHORT.
        let mut asm = StateAssembler::passthrough();
        assert_eq!(
            asm.feed(&hand_record(
                2,
                0,
                0,
                RECORD_KIND_CHUNK,
                STATE_RECORD_PAYLOAD
            )),
            None,
            "the anchor opens normally"
        );
        assert_eq!(
            asm.feed(&hand_record(2, 0, 1, RECORD_KIND_CHUNK, 7)),
            Some(StateAnchorEvent::Torn {
                run_id: 1,
                step: 2,
                node_idx: 0,
                cause: TornCause::ShortChunk { part: 1, len: 7 },
            })
        );
        // …and the anchor is VOIDED, so its own FINAL record cannot resurrect it as a
        // short blob.
        assert_eq!(asm.feed(&hand_record(2, 0, 2, RECORD_KIND_FINAL, 5)), None);
        assert!(asm.finish().is_empty(), "reported once, not twice");
    }

    // A voided anchor's FINAL record CLEARS its tombstone, so a LATER anchor for the
    // same key starts clean. Both void paths are driven, because they clear by
    // different mechanisms and neither is pinned anywhere else: the
    // `events.len() == 1` assertions elsewhere cannot see a tombstone that is never
    // removed (a stale one is dropped silently at `finish`), so the observable is what
    // happens to the NEXT anchor.
    #[test]
    fn a_voided_key_is_cleared_by_its_final_record_so_the_next_anchor_starts_clean() {
        let want = blob(600);

        // PATH 1: the gap arrives on a NON-final record ⇒ a tombstone is inserted, and
        // the anchor's own FINAL record clears it.
        let mut asm = StateAssembler::passthrough();
        assert_eq!(
            asm.feed(&hand_record(
                1,
                0,
                0,
                RECORD_KIND_CHUNK,
                STATE_RECORD_PAYLOAD
            )),
            None
        );
        assert!(matches!(
            asm.feed(&hand_record(
                1,
                0,
                2,
                RECORD_KIND_CHUNK,
                STATE_RECORD_PAYLOAD
            )),
            Some(StateAnchorEvent::Torn { .. })
        ));
        assert_eq!(
            asm.feed(&hand_record(1, 0, 3, RECORD_KIND_FINAL, 4)),
            None,
            "the tombstone swallows the rest of the broken stream"
        );
        let mut events = Vec::new();
        for r in collect(&want, 1, 1, 0) {
            if let Some(e) = asm.feed(&r) {
                events.push(e);
            }
        }
        events.extend(asm.finish());
        assert_eq!(
            events,
            vec![StateAnchorEvent::Complete {
                run_id: 1,
                step: 1,
                node_idx: 0,
                parts: 2,
                bytes: want.clone(),
            }],
            "a re-sent anchor for the SAME key must assemble — a tombstone that is \
             never cleared would swallow it forever"
        );

        // PATH 2: the gap arrives ON the FINAL record ⇒ no tombstone is needed and the
        // key is removed directly.
        let mut asm = StateAssembler::passthrough();
        assert_eq!(
            asm.feed(&hand_record(
                9,
                0,
                0,
                RECORD_KIND_CHUNK,
                STATE_RECORD_PAYLOAD
            )),
            None
        );
        assert!(matches!(
            asm.feed(&hand_record(9, 0, 5, RECORD_KIND_FINAL, 3)),
            Some(StateAnchorEvent::Torn { .. })
        ));
        let mut events = Vec::new();
        for r in collect(&want, 1, 9, 0) {
            if let Some(e) = asm.feed(&r) {
                events.push(e);
            }
        }
        events.extend(asm.finish());
        assert_eq!(
            events,
            vec![StateAnchorEvent::Complete {
                run_id: 1,
                step: 9,
                node_idx: 0,
                parts: 2,
                bytes: want,
            }],
            "the same, by the other void path"
        );
    }

    // THE key arm: `node_idx` is part of the reassembly key. Drop it and these two
    // interleaved streams concatenate into two wrong blobs.
    #[test]
    fn two_interleaved_nodes_reassemble_independently() {
        let a = blob(1100);
        let b: Vec<u8> = blob(1100).iter().map(|x| x ^ 0xFF).collect();
        let ra = collect(&a, 9, 100, 0);
        let rb = collect(&b, 9, 100, 1);
        assert_eq!(ra.len(), 3);
        assert_eq!(rb.len(), 3);
        // Interleave them record for record.
        let mut mixed = Vec::new();
        for i in 0..3 {
            mixed.push(ra[i].clone());
            mixed.push(rb[i].clone());
        }
        let events = assemble(&mixed, StateAssembler::passthrough());
        assert_eq!(events.len(), 2, "two anchors, two Complete events");
        assert_eq!(
            events[0],
            StateAnchorEvent::Complete {
                run_id: 9,
                step: 100,
                node_idx: 0,
                parts: 3,
                bytes: a,
            }
        );
        assert_eq!(
            events[1],
            StateAnchorEvent::Complete {
                run_id: 9,
                step: 100,
                node_idx: 1,
                parts: 3,
                bytes: b,
            }
        );
    }

    #[test]
    fn two_steps_of_one_node_reassemble_independently() {
        let a = blob(600);
        let b = blob(700);
        let mut mixed = collect(&a, 1, 10, 0);
        mixed.extend(collect(&b, 1, 11, 0));
        let events = assemble(&mixed, StateAssembler::passthrough());
        assert_eq!(events.len(), 2);
        assert!(matches!(
            &events[0],
            StateAnchorEvent::Complete { step: 10, bytes, .. } if bytes == &a
        ));
        assert!(matches!(
            &events[1],
            StateAnchorEvent::Complete { step: 11, bytes, .. } if bytes == &b
        ));
    }

    #[test]
    fn a_voided_anchor_reports_once_and_its_survivors_are_swallowed() {
        let want = blob(2400); // 5 full records
        let recs = collect(&want, 1, 1, 1);
        assert_eq!(recs.len(), 5);
        let mut kept = recs.clone();
        kept.remove(1); // drop part 1; parts 2,3,4 survive
        let events = assemble(&kept, StateAssembler::passthrough());
        assert_eq!(
            events.len(),
            1,
            "one broken anchor yields ONE event, not one per surviving record: {events:?}"
        );
    }

    #[test]
    fn a_skip_record_names_its_cause_and_voids_whatever_was_in_flight() {
        let want = blob(1100);
        let mut recs = collect(&want, 2, 5, 3);
        recs.truncate(1); // an anchor that started …
        recs.push(
            encode_skip_record(2, 5, 3, SkipCause::ChildTimeout, "no progress for 5s").to_vec(),
        );
        let events = assemble(&recs, StateAssembler::passthrough());
        assert_eq!(
            events,
            vec![StateAnchorEvent::Skipped {
                run_id: 2,
                step: 5,
                node_idx: 3,
                cause: SkipCause::ChildTimeout,
                detail: "no progress for 5s".to_string(),
            }],
            "the skip is reported and the in-flight anchor is NOT also reported torn"
        );
    }

    #[test]
    fn skip_cause_wire_round_trips_and_preserves_an_unknown_code() {
        for c in [
            SkipCause::Contended,
            SkipCause::LowMemory,
            SkipCause::StillEncoding,
            SkipCause::ForkFailed,
            SkipCause::ChildTimeout,
            SkipCause::CaptureFailed,
        ] {
            assert_eq!(SkipCause::from_wire(c.as_wire()), c);
        }
        assert_eq!(SkipCause::from_wire(9999), SkipCause::Unrecognized(9999));
        assert_eq!(SkipCause::Unrecognized(9999).as_wire(), 9999);
    }

    #[test]
    fn a_long_skip_detail_is_truncated_on_a_char_boundary() {
        let detail = "é".repeat(400); // 800 bytes, past the 476-byte detail cap
        let rec = encode_skip_record(1, 1, 1, SkipCause::Contended, &detail);
        let mut asm = StateAssembler::passthrough();
        match asm.feed(&rec) {
            Some(StateAnchorEvent::Skipped { detail, cause, .. }) => {
                assert_eq!(cause, SkipCause::Contended);
                assert!(detail.len() <= STATE_RECORD_PAYLOAD - 4);
                assert!(
                    detail.chars().all(|c| c == 'é'),
                    "truncation must not split a character: {detail:?}"
                );
            }
            other => panic!("expected Skipped, got {other:?}"),
        }
    }

    #[test]
    fn a_malformed_record_is_reported_never_skipped() {
        // kind 0 — a zeroed slot.
        let zeroed = [0u8; STATE_RECORD_SIZE as usize];
        let mut asm = StateAssembler::passthrough();
        assert!(matches!(
            asm.feed(&zeroed),
            Some(StateAnchorEvent::Malformed { .. })
        ));
        // len past the payload region.
        let mut bad = StateRecordHeader {
            run_id: 1,
            step: 1,
            node_idx: 0,
            part: 0,
            kind: RECORD_KIND_FINAL,
            len: 481,
        }
        .as_bytes()
        .to_vec();
        bad.resize(STATE_RECORD_SIZE as usize, 0);
        assert!(matches!(
            asm.feed(&bad),
            Some(StateAnchorEvent::Malformed { .. })
        ));
        // A short buffer.
        assert!(matches!(
            asm.feed(&[0u8; 8]),
            Some(StateAnchorEvent::Malformed { .. })
        ));
    }

    // The mid-run attach: the partial head anchor is DISCARDED, not misreported as
    // corruption — and the next whole anchor assembles cleanly.
    #[test]
    fn an_armed_assembler_discards_a_headless_anchor_and_assembles_the_next() {
        let head = blob(1500);
        let next = blob(700);
        let mut stream = collect(&head, 1, 1, 0);
        stream.drain(..2); // attach lands after parts 0 and 1
        stream.extend(collect(&next, 1, 2, 0));

        let mut asm = StateAssembler::armed();
        let mut events = Vec::new();
        for r in &stream {
            if let Some(ev) = asm.feed(r) {
                events.push(ev);
            }
        }
        events.extend(asm.finish());
        assert_eq!(asm.discarded(), 2, "parts 2 and 3 of the headless anchor");
        assert!(!asm.is_armed(), "the first part-0 record opens it for good");
        assert_eq!(
            events,
            vec![StateAnchorEvent::Complete {
                run_id: 1,
                step: 2,
                node_idx: 0,
                parts: 2,
                bytes: next,
            }]
        );
    }

    // The anti-tautology for the arm above: a PASSTHROUGH assembler on the SAME
    // headless stream names it torn. Without this, "armed discarded it" could be
    // satisfied by an assembler that ignores headless records in both modes.
    #[test]
    fn a_passthrough_assembler_names_the_same_headless_anchor_torn() {
        let head = blob(1500);
        let mut stream = collect(&head, 1, 1, 0);
        stream.drain(..2);
        let events = assemble(&stream, StateAssembler::passthrough());
        assert_eq!(
            events,
            vec![StateAnchorEvent::Torn {
                run_id: 1,
                step: 1,
                node_idx: 0,
                cause: TornCause::PartOutOfOrder {
                    expected: 0,
                    got: 2
                },
            }]
        );
    }

    #[test]
    fn default_capacity_is_a_power_of_two_worth_of_the_default_budget() {
        assert_eq!(default_capacity_records(), 131_072);
        assert!(default_capacity_records().is_power_of_two());
        assert_eq!(capacity_records_for_bytes(512 * 3), 2, "rounds DOWN");
        assert_eq!(capacity_records_for_bytes(0), 0);
    }

    // A budget past what a u32 capacity can express must SATURATE, not wrap to zero.
    // Both sides of the ceiling in one body, so the saturation is a DIFFERENCE rather
    // than two independent readings.
    #[cfg(target_pointer_width = "64")]
    #[test]
    fn an_over_large_budget_saturates_at_the_largest_representable_capacity() {
        let at_ceiling = MAX_CAPACITY_RECORDS as usize * STATE_RECORD_SIZE as usize; // 1 TiB
        assert_eq!(
            capacity_records_for_bytes(at_ceiling),
            MAX_CAPACITY_RECORDS as u32,
            "exactly the ceiling is representable and must be returned"
        );
        assert_eq!(
            capacity_records_for_bytes(at_ceiling * 2),
            MAX_CAPACITY_RECORDS as u32,
            "a budget whose record count reaches 2^32 must SATURATE — a narrowing \
             cast turns it into 0, which `create` then rejects as 'capacity must be \
             > 0', an answer about the wrong problem"
        );
        assert_eq!(
            capacity_records_for_bytes(usize::MAX),
            MAX_CAPACITY_RECORDS as u32,
            "and so must anything above it"
        );
        assert!(capacity_records_for_bytes(usize::MAX).is_power_of_two());
    }
}

#[cfg(test)]
mod naming_tests {
    use super::*;

    /// The tag RECIPE is a cross-process, cross-BINARY contract: a graph process
    /// (`cerulion_cli_engine`) creates the ring under it and a recorder
    /// (`cerulion_bagd`) probes for it, and the two never speak. A change on one
    /// side alone is a run that checkpoints into a segment nobody drains, with
    /// nothing anywhere reporting it — so the literal is pinned, not derived.
    #[test]
    fn the_tag_recipe_is_the_documented_literal() {
        assert_eq!(
            state_ring_tag("cer_run_42", 0).expect("rank 0"),
            "cer_st_cer_run_42_r0"
        );
        assert_eq!(
            state_ring_tag("cer_run_42", 7).expect("rank 7"),
            "cer_st_cer_run_42_r7"
        );
        // Distinct ranks of one run name distinct rings, and distinct runs at one
        // rank do too — the two axes the name has to separate.
        assert_ne!(
            state_ring_tag("t", 0).unwrap(),
            state_ring_tag("t", 1).unwrap()
        );
        assert_ne!(
            state_ring_tag("a", 0).unwrap(),
            state_ring_tag("b", 0).unwrap()
        );
        // And the SHM name is the shared `ring_shm_name` recipe over that tag —
        // never a second hash, or the two sides would derive different objects
        // from the same agreed tag.
        assert_eq!(
            state_ring_shm_name("cer_run_42", 3).unwrap(),
            crate::shm_ring::ring_shm_name("cer_st_cer_run_42_r3")
        );
    }

    /// `cer_st_` must not be a PREFIX of, or prefixed by, the other SHM families
    /// this repo mints. (The names it derives are hashed to a fixed length by
    /// `ring_shm_name`, so this is about the TAG namespace: a state-ring tag must
    /// never be constructible as a trace-ring tag.)
    #[test]
    fn the_state_ring_tag_namespace_cannot_collide_with_the_trace_ring_family() {
        let state = state_ring_tag("x", 0).unwrap();
        // The supervisor's two trace-ring families, spelled as it mints them.
        for trace in ["cer_rec_g_1234_r0", "cer_rec_g_1234_dep", "cer_rec_g_1234"] {
            assert_ne!(state, trace);
            assert!(
                !state.starts_with(trace) && !trace.starts_with(&state),
                "state tag `{state}` and trace tag `{trace}` must be prefix-free"
            );
        }
    }

    /// The departure sentinel is refused at CONSTRUCTION, not merely kept out of
    /// the discovery sweep's range: a rank number reaching here from a plan is the
    /// bookkeeping bug the guard exists for, and a state ring named as one would
    /// make an anchor's provenance indistinguishable from a worker departure.
    #[test]
    fn the_departure_sentinel_rank_can_never_name_a_state_ring() {
        let err = state_ring_tag("t", STATE_RING_RESERVED_RANK)
            .expect_err("the sentinel rank must be refused");
        let msg = err.to_string();
        assert!(
            msg.contains("departure") && msg.contains(&STATE_RING_RESERVED_RANK.to_string()),
            "the refusal must name the sentinel and why: {msg}"
        );
        assert!(state_ring_shm_name("t", STATE_RING_RESERVED_RANK).is_err());
        // The rank one BELOW it is an ordinary rank — the boundary is exact.
        assert!(state_ring_tag("t", STATE_RING_RESERVED_RANK - 1).is_ok());
    }

    /// The OWNER constructor refuses it too, and that is a second argument rather
    /// than a second copy of the first: `tag` and `rank` are separate parameters,
    /// so a caller can hand a perfectly legal rank-3 NAME and still stamp the
    /// sentinel into the ring HEADER. Every reader downstream believes the header
    /// (`StateCoverage::ranks_discovered` is built from `ring.rank()`), so the name
    /// guard alone leaves the door open.
    ///
    /// Deliberately asserted WITHOUT creating a ring: the refusal must land before
    /// the SHM segment exists, so a rejected create leaves no orphan behind.
    #[test]
    fn the_owner_constructor_refuses_the_departure_sentinel_rank() {
        let tag = format!("owner_sentinel_{}", std::process::id());
        let err = StateRingOwner::create(&tag, 8, STATE_RING_RESERVED_RANK, 1, &["n0"])
            .expect_err("the owner constructor must refuse the sentinel rank");
        let msg = err.to_string();
        assert!(
            msg.contains("departure") && msg.contains(&STATE_RING_RESERVED_RANK.to_string()),
            "the refusal must name the sentinel and why: {msg}"
        );
        // and it refused BEFORE any syscall — no segment was left behind.
        assert!(
            !crate::shm_ring::shm_object_exists(&crate::shm_ring::ring_shm_name(&tag)),
            "a refused create must not leave an SHM object behind"
        );
        // Anti-tautology: the constructor is not simply broken for every rank.
        let ok = StateRingOwner::create(&tag, 8, STATE_RING_RESERVED_RANK - 1, 1, &["n0"])
            .expect("an ordinary rank must still create");
        assert_eq!(ok.rank(), STATE_RING_RESERVED_RANK - 1);
    }

    /// A blank tag is what a launcher that computed no tag leaves behind. Deriving
    /// from it would give every rank of every run the same name.
    #[test]
    fn a_blank_arm_tag_names_no_ring() {
        for blank in ["", "   ", "\t\n"] {
            let err = state_ring_tag(blank, 0).expect_err("a blank tag must be refused");
            assert!(
                err.to_string().contains("blank"),
                "the refusal must say WHICH rule refused: {err}"
            );
        }
        // A tag with interior whitespace is a name, not a blank — the rule is
        // "says nothing", not "is tidy".
        assert!(state_ring_tag(" a b ", 0).is_ok());
    }

    /// The rank-hole oracle. Ranks are dense `0..n`, so a hole is PROOF a rank
    /// that exists published nothing — the one thing a recorder can establish
    /// about a deployment whose rank count it was never told.
    #[test]
    fn a_hole_below_the_highest_discovered_rank_is_reported_and_the_top_is_not() {
        // Hand oracle, every row a different shape.
        let cases: &[(&[u32], &[u32])] = &[
            (&[], &[]),
            (&[0], &[]),
            (&[0, 1, 2], &[]),
            // rank 1 exists (2 does) and published nothing.
            (&[0, 2], &[1]),
            (&[0, 3], &[1, 2]),
            // Order of discovery must not matter — the sweep reports what it
            // found, not when.
            (&[3, 0], &[1, 2]),
            // rank 0 itself can be the hole (a monolith-numbered rank that died).
            (&[1, 2], &[0]),
            // Nothing above the maximum is claimed: `[0]` is a one-rank run, not
            // a run missing ranks 1..
            (&[0, 1], &[]),
        ];
        for (found, want) in cases {
            assert_eq!(
                missing_state_ring_ranks(found),
                want.to_vec(),
                "found {found:?}"
            );
        }
    }

    /// A rank the discovery sweep can never PROBE cannot witness density.
    ///
    /// These ranks arrive from ring HEADERS — a segment another process wrote — so
    /// the density maximum is foreign data. Believing it makes the recorder ask for
    /// one `u32` per rank below it: at `u32::MAX` that is ~17 GB on the writer
    /// thread at finalize, from four bytes of SHM.
    ///
    /// The cheap poison case is FIRST on purpose: it catches a filter-less max
    /// in 1023 elements instead of four billion, so this test stays runnable.
    #[test]
    fn a_rank_outside_the_addressable_space_cannot_drive_the_density_argument() {
        // Hand oracle. `STATE_RING_MAX_RANKS` is the sweep's EXCLUSIVE ceiling.
        let cases: &[(&[u32], &[u32])] = &[
            // Cheap poison: the ceiling itself is out of range, so rank 0 is the
            // whole run and nothing is missing (un-fixed: 1023 fabricated holes).
            (&[0, STATE_RING_MAX_RANKS], &[]),
            // The expensive poison the bound really exists for.
            (&[0, 1, u32::MAX], &[]),
            (&[STATE_RING_RESERVED_RANK], &[]),
            // A REAL hole is still reported with a poison rank alongside it —
            // the bound removes fabricated holes, never true ones.
            (&[0, 2, u32::MAX], &[1]),
        ];
        for (found, want) in cases {
            assert_eq!(
                missing_state_ring_ranks(found),
                want.to_vec(),
                "found {found:?}"
            );
        }
        // The boundary is exact on the INSIDE too: one below the ceiling is an
        // ordinary rank and does witness density.
        let inside = missing_state_ring_ranks(&[STATE_RING_MAX_RANKS - 1]);
        assert_eq!(inside.len(), STATE_RING_MAX_RANKS as usize - 1);
        assert_eq!(inside.first().copied(), Some(0));
        // And the result can never exceed the addressable space.
        assert!(inside.len() < STATE_RING_MAX_RANKS as usize);
    }

    /// Every rank of ONE run must stamp the SAME `run_id` (the assembler groups
    /// by it), and two different runs must not collide into one grouping key.
    /// Both halves fall out of deriving it from the tag — which is the point:
    /// there is no plumbing that could get it wrong per rank.
    #[test]
    fn one_runs_ranks_agree_on_the_grouping_id_and_two_runs_do_not() {
        let a = state_ring_run_id("cer_run_42");
        for rank in [0u32, 1, 7, 63] {
            // The value is a function of the TAG alone — the rank is not an
            // input, so no rank can derive a different one.
            assert_eq!(state_ring_run_id("cer_run_42"), a, "rank {rank}");
        }
        assert_ne!(a, state_ring_run_id("cer_run_43"));
        // It is the SHARED fold, not a second one: the same string through the
        // ring-name recipe carries the same 16 hex digits.
        assert!(crate::shm_ring::ring_shm_name("cer_run_42").ends_with(&format!("{a:016x}")));
    }

    /// The sweep's stopping rule, against hand-built rank sets. The HOLE rows are
    /// the load-bearing ones: a sweep that stopped at the first miss would report
    /// a two-rank run as one-rank whenever rank 1 was slow to arm, and the
    /// resulting recording would look complete.
    #[test]
    fn the_sweep_walks_past_holes_and_stops_when_the_ranks_run_out() {
        fn sweep(present: &[u32]) -> (Vec<u32>, u32) {
            let mut probes = 0u32;
            let found = scan_state_ring_ranks(|r| {
                probes += 1;
                present.contains(&r)
            });
            (found, probes)
        }
        // Dense runs: every rank found, and the walk costs the ranks plus the
        // tolerance — never the whole 1024-rank space.
        for n in [1u32, 2, 5, 19] {
            let present: Vec<u32> = (0..n).collect();
            let (found, probes) = sweep(&present);
            assert_eq!(found, present, "dense run of {n}");
            assert_eq!(
                probes,
                n + STATE_RING_PROBE_GAP_TOLERANCE,
                "the walk must stop a tolerance past the last rank, not sweep the space"
            );
        }
        // A HOLE inside the run does not truncate the sweep.
        assert_eq!(sweep(&[0, 2, 3]).0, vec![0, 2, 3]);
        // …up to the tolerance, pinned on BOTH sides in units of CONSECUTIVE
        // MISSES (which is what the rule counts — a rank present at `k` leaves
        // `k - 1` misses behind it).
        let bridges = STATE_RING_PROBE_GAP_TOLERANCE; // ranks 1..=T-1 absent = T-1 misses
        assert_eq!(
            sweep(&[0, bridges]).0,
            vec![0, bridges],
            "tolerance-1 consecutive misses must still bridge"
        );
        let stops = STATE_RING_PROBE_GAP_TOLERANCE + 1; // T consecutive misses
        assert_eq!(
            sweep(&[0, stops]).0,
            vec![0],
            "the tolerance-th consecutive miss ends the sweep — the bound, stated"
        );
        // Nothing armed at all: bounded, and reports nothing rather than
        // fabricating a rank.
        let (found, probes) = sweep(&[]);
        assert!(found.is_empty());
        assert_eq!(probes, STATE_RING_PROBE_GAP_TOLERANCE);
        // A run whose rank 0 is the hole is still discovered — rank 0 is not
        // special to the walk.
        assert_eq!(sweep(&[1, 2]).0, vec![1, 2]);
    }
}
