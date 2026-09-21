// SPDX-License-Identifier: AGPL-3.0-only
//! The RESTORE side, as pure decisions.
//!
//! The capture half lives elsewhere: a trait that encodes a node's state
//! ([`crate::state::CerulionState`]), a ring that carries it (`state_ring`),
//! and a bag channel it lands in. This module owns
//! everything a reader must DECIDE before a single byte is applied to a node,
//! and it owns it as pure functions over plain inputs so every rule can be
//! driven by a hand-written oracle rather than by a bag.
//!
//! # The five decisions
//!
//! 1. **Which anchor do we resume from?** [`plan_restore`] — the checkpoint/trace
//!    rendezvous, checked rather than assumed.
//! 2. **Do the recorded bytes still describe this node's type?**
//!    [`AnchorBlob`] carries the capture-time `STATE_SHAPE` and
//!    [`classify_shape`] judges it.
//! 3. **Which recorded frames may a restored graph see?** [`plan_backlog_admission`]
//!    — the backlog frame gate.
//! 4. **What sequence does a restored publisher start at?**
//!    [`seed_publisher_sequence`] — the publisher-sequence seed ladder.
//! 5. **Is the recording good enough to re-execute at all?**
//!    [`refuse_lossy_reexecuted_topics`] and [`enforce_strict_state`].
//!
//! # It is PORTABLE, and that is a requirement rather than an accident
//!
//! The capture half is `#[cfg(unix)]` because it needs `fork(2)` and POSIX SHM.
//! Nothing here does: these are decisions about a RECORDED bag, and the machine
//! that replays a bag is not necessarily the one that recorded it — the same
//! desk/robot split every other reader in this repo assumes. So this module
//! names no unix-gated module (the one type it needs from the capture
//! vocabulary, [`crate::state::SkipCause`], lives on the portable side and is
//! re-exported by `state_ring` for its existing callers), and prose that points
//! at a unix-only item does so as plain text rather than as an intra-doc link a
//! non-unix `cargo doc` would report broken.
//!
//! # What this module deliberately does NOT do
//!
//! It never reads a bag, never touches transport, and never applies bytes. The
//! adapter that turns MCAP records into these inputs lives in
//! `cerulion_cli_engine`; the seam that hands a blob to a node lives on
//! [`crate::graph::node::NodeEntry`]. Keeping the rules here means the CLI, a
//! future `bag play --resim`, and the offline `bag derive-anchors` verb all
//! judge a recording the same way, because there is only one judge.
//!
//! # Refusals are terminal, on purpose
//!
//! Every failure here returns a [`RestoreRefusal`], never a degraded default.
//! Restoring a node to `Default` when its recorded state could not be applied
//! fabricates state (Principle #13) and re-creates exactly the class
//! `SnapshotState::Failed` exists to close. Each variant's `Display` names the
//! offender and the fix.

use std::collections::BTreeMap;
use std::fmt;

use thiserror::Error;

use crate::state::{CerulionState, SkipCause, StateError, StateSink};

// ===========================================================================
// The anchor blob framing
// ===========================================================================

/// The magic word every anchor blob opens with: the ASCII bytes `CERSTATE`,
/// little-endian.
///
/// It exists because the shape check has no other wire slot. `STATE_SHAPE` is
/// a compile-time constant on the node's type and nothing else
/// writes it anywhere — not into the record header (which is a fixed 32 bytes
/// of transport bookkeeping), not into `state_coverage.json` (which the
/// recorder builds from headers it never decodes), and not into the ring
/// manifest (which never reaches the bag). Without it a retyped field restores
/// SILENTLY, which is the failure the shape gate exists to make terminal.
///
/// The magic is not decoration. A blob is either framed or it is not, and
/// sniffing — "treat the first eight bytes as a shape if they look like one" —
/// cannot tell a framed blob from an unframed one whose first field happens to
/// hold that value. So the decoder REQUIRES the magic and reports
/// [`RestoreRefusal::UnframedAnchor`] otherwise. That is a loud refusal on a
/// blob it cannot interpret, never a guess at one it can.
pub const ANCHOR_BLOB_MAGIC: u64 = u64::from_le_bytes(*b"CERSTATE");

/// The magic of a blob that also carries a FRAMEWORK SECTION
/// ([`NodeFrameworkState`]): the ASCII bytes `CERSTAT2`, little-endian.
///
/// # Why a second magic rather than a version field
///
/// A version field can only be added where one can be READ, and a v1 blob has
/// no slot for it: its bytes are magic, shape, then payload, so the first byte
/// after the shape belongs to the node's own encoding. The magic is therefore
/// the ONLY in-band discriminant available, and reusing it is what keeps this
/// additive.
///
/// Sniffing — "if the bytes after the shape look like a section, read one" —
/// is the alternative and it is the one [`ANCHOR_BLOB_MAGIC`]'s own doc
/// refuses, for the same reason: a payload whose first four bytes happen to
/// hold `1` is indistinguishable from a section declaring version 1.
///
/// The compatibility story runs both ways and only one direction is silent-
/// capable, which is why it is the one that is preserved:
///
/// | Bag | Reader | Outcome |
/// |---|---|---|
/// | v1 (no section) | reads v2 | v1 arm, `framework` empty — byte-identical to a v1-only reader |
/// | v2 (section) | reads v2 | the section is read |
/// | v2 (section) | v1 only | [`RestoreRefusal::UnframedAnchor`] — LOUD, never a misread |
pub const ANCHOR_BLOB_MAGIC_V2: u64 = u64::from_le_bytes(*b"CERSTAT2");

/// Bytes of framing an anchor blob carries ahead of the encoded state.
pub const ANCHOR_BLOB_HEADER_SIZE: usize = 16;

const _: () = assert!(ANCHOR_BLOB_HEADER_SIZE == 2 * size_of::<u64>());

/// Bytes of framing a [`ANCHOR_BLOB_MAGIC_V2`] blob carries ahead of the
/// framework section: [`ANCHOR_BLOB_HEADER_SIZE`] plus the section's own
/// `u32` length.
pub const ANCHOR_BLOB_V2_HEADER_SIZE: usize = ANCHOR_BLOB_HEADER_SIZE + size_of::<u32>();

/// One node's anchor, split into the identity the reader checks and the bytes
/// the decoder consumes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AnchorBlob<'a> {
    /// The `CerulionState::STATE_SHAPE` of the type that produced the payload.
    pub state_shape: u64,
    /// The framework section's raw bytes, EMPTY when the blob carries
    /// none (every v1 anchor, and every v2 capture of a node the
    /// scheduler had nothing to say about).
    ///
    /// Raw rather than parsed so this type stays `Copy` and `decode` stays one
    /// slice split; [`Self::framework_state`] does the parse, and its refusal
    /// names the node.
    ///
    /// EMPTY is unambiguous: a section always carries at least its own version
    /// word, so a zero-length one is refused at decode rather than reaching
    /// here.
    pub framework: &'a [u8],
    /// The encoded state — exactly what `cer_capture` wrote.
    pub payload: &'a [u8],
}

impl<'a> AnchorBlob<'a> {
    /// Write the framing for a capture of `state_shape` into `out`.
    ///
    /// The capture carrier calls this immediately before the node's own
    /// `cer_capture`, so the blob the ring carries is `header ++ payload`. It
    /// is exposed separately from [`capture_anchor_blob`] because the fork
    /// carrier walks nodes through an ERASED view (a trait object cannot carry
    /// an associated const), so it has the shape in hand as a `u64` rather
    /// than as a type parameter.
    pub fn write_header(state_shape: u64, out: &mut dyn StateSink) -> Result<(), StateError> {
        out.write(&ANCHOR_BLOB_MAGIC.to_le_bytes())?;
        out.write(&state_shape.to_le_bytes())?;
        Ok(())
    }

    /// Write the framing for a capture of `state_shape` that ALSO carries the
    /// framework section.
    ///
    /// The section is the scheduler's own plain-data view of the node at the
    /// anchor boundary — see [`NodeFrameworkState`] — and it is the carrier,
    /// not the node, that supplies it: the node's `cer_capture` encodes the
    /// USER's fields and knows nothing about the scheduler that drives it.
    ///
    /// Emits [`ANCHOR_BLOB_MAGIC_V2`], so a blob written through here is
    /// readable by any build that knows the v2 magic and LOUDLY refused by one that does not.
    pub fn write_header_with_framework(
        state_shape: u64,
        framework: &NodeFrameworkState,
        out: &mut dyn StateSink,
    ) -> Result<(), StateError> {
        out.write(&ANCHOR_BLOB_MAGIC_V2.to_le_bytes())?;
        out.write(&state_shape.to_le_bytes())?;
        let len = framework.encoded_len();
        // A section longer than `u32::MAX` is unreachable — the length is
        // dominated by the sync-input names, one per wired trigger input — but
        // the cast is the format contract, so it is checked rather than
        // assumed.
        let len32 = u32::try_from(len).map_err(|_| StateError::SinkFull)?;
        out.write(&len32.to_le_bytes())?;
        framework.encode(out)
    }

    /// Split a reassembled anchor into its shape, its framework section and
    /// its payload.
    ///
    /// `node` names the offender in the refusal — a reader holding a dozen
    /// anchors needs to know WHICH one it cannot interpret.
    ///
    /// A [`ANCHOR_BLOB_MAGIC`] (v1) blob yields an EMPTY `framework` and a
    /// payload starting at [`ANCHOR_BLOB_HEADER_SIZE`] — byte-for-byte the
    /// v1-only reader's answer, which is what keeps every v1 bag readable.
    pub fn decode(node: &str, bytes: &'a [u8]) -> Result<Self, RestoreRefusal> {
        let unframed = || RestoreRefusal::UnframedAnchor {
            node: node.to_string(),
            bytes: bytes.len(),
        };
        if bytes.len() < ANCHOR_BLOB_HEADER_SIZE {
            return Err(unframed());
        }
        let magic = u64::from_le_bytes(bytes[0..8].try_into().expect("8 bytes"));
        let state_shape = u64::from_le_bytes(bytes[8..16].try_into().expect("8 bytes"));
        if magic == ANCHOR_BLOB_MAGIC {
            return Ok(Self {
                state_shape,
                framework: &[],
                payload: &bytes[ANCHOR_BLOB_HEADER_SIZE..],
            });
        }
        if magic != ANCHOR_BLOB_MAGIC_V2 {
            return Err(unframed());
        }
        if bytes.len() < ANCHOR_BLOB_V2_HEADER_SIZE {
            // The MAGIC was recognised, so this blob IS framed — what is
            // truncated is the section's own length word. Reporting it as
            // unframed would send the reader after the wrong thing.
            return Err(RestoreRefusal::MalformedFrameworkSection {
                node: node.to_string(),
                reason: format!(
                    "the anchor is {} byte(s), too short to carry the {ANCHOR_BLOB_V2_HEADER_SIZE}-byte \
                     framing a section rides behind",
                    bytes.len()
                ),
            });
        }
        let section_len = u32::from_le_bytes(bytes[16..20].try_into().expect("4 bytes")) as usize;
        // A v2 blob DECLARING a section shorter than the version header is
        // structurally malformed, and the floor is checked HERE rather than
        // wherever the section is later parsed because the value `0` never
        // reaches a parser at all: it yields an EMPTY `framework`, which is the
        // signal a v1 blob sets, so the reader would take the v1 arm and
        // fall silently back to trace-derived scheduling. That is the
        // mis-attribution class — a claim the recording DID make, discarded
        // without a word, and then a divergence reported against the candidate.
        //
        // No legitimate writer can produce it: `write_header_with_framework`
        // emits `NodeFrameworkState::encoded_len()`, whose floor is this prefix
        // for EVERY value including `Default` (a section always carries at
        // least its own version word). So the whole range below the floor is
        // unreachable from any v2 capture, and refusing all of it under one
        // rule keeps the boundary a single line rather than a special case for
        // zero and a different message for one-to-twenty-four.
        if section_len < FRAMEWORK_SECTION_V1_PREFIX {
            return Err(RestoreRefusal::MalformedFrameworkSection {
                node: node.to_string(),
                reason: format!(
                    "the anchor declares a {section_len}-byte framework section, but a section is \
                     at least {FRAMEWORK_SECTION_V1_PREFIX} bytes — no capture can write one \
                     smaller, and reading it as ABSENT would silently resume on a schedule this \
                     recording did not state"
                ),
            });
        }
        let section_end = ANCHOR_BLOB_V2_HEADER_SIZE
            .checked_add(section_len)
            .filter(|end| *end <= bytes.len())
            .ok_or_else(|| RestoreRefusal::MalformedFrameworkSection {
                node: node.to_string(),
                reason: format!(
                    "the section declares {section_len} byte(s) but the anchor holds only {} \
                     after its {ANCHOR_BLOB_V2_HEADER_SIZE}-byte framing",
                    bytes.len().saturating_sub(ANCHOR_BLOB_V2_HEADER_SIZE)
                ),
            })?;
        Ok(Self {
            state_shape,
            framework: &bytes[ANCHOR_BLOB_V2_HEADER_SIZE..section_end],
            payload: &bytes[section_end..],
        })
    }

    /// Parse this anchor's framework section, if it carries one.
    ///
    /// `Ok(None)` is the v1 answer and the one every v1 bag
    /// gives; a malformed section is a REFUSAL rather than a `None`, because
    /// "the recording states nothing" and "the recording states something this
    /// build cannot read" are different facts and treating them alike would
    /// silently downgrade the second into the first.
    ///
    /// An EMPTY `framework` therefore means V1 and nothing else: [`Self::decode`]
    /// refuses a v2 blob whose declared section is below the version-header
    /// floor, so no v2 anchor can reach this short-circuit. Without that floor
    /// a `section_len` of zero would arrive here indistinguishable from a v1
    /// blob and take the absent arm.
    pub fn framework_state(
        &self,
        node: &str,
    ) -> Result<Option<NodeFrameworkState>, RestoreRefusal> {
        if self.framework.is_empty() {
            return Ok(None);
        }
        NodeFrameworkState::decode(node, self.framework).map(Some)
    }
}

/// Capture `value` as a framed anchor blob: the shape header, then its state.
///
/// This is the seam the derive and the carriers adopt so a
/// recorded anchor is self-describing. A capture that skips it produces a blob
/// [`AnchorBlob::decode`] refuses, which is the intended direction: an
/// unreadable anchor is a loud restore failure, never a silent misapply.
pub fn capture_anchor_blob<T: CerulionState>(
    value: &T,
    out: &mut dyn StateSink,
) -> Result<(), StateError> {
    AnchorBlob::write_header(T::STATE_SHAPE, out)?;
    value.cer_capture(out)
}

/// Capture `value` as a framed anchor blob CARRYING the framework section
/// ([`NodeFrameworkState`]): the shape header, the scheduler's plain-data view of this
/// node, then its state.
///
/// The two halves come from different owners on purpose — the payload is the
/// USER's fields, encoded by the node's own generated `cer_capture`; the
/// section is the FRAMEWORK's, read off the scheduler by the carrier — and
/// they ride one blob so a reader cannot end up holding one without the other.
pub fn capture_anchor_blob_with_framework<T: CerulionState>(
    value: &T,
    framework: &NodeFrameworkState,
    out: &mut dyn StateSink,
) -> Result<(), StateError> {
    AnchorBlob::write_header_with_framework(T::STATE_SHAPE, framework, out)?;
    value.cer_capture(out)
}

// ===========================================================================
// The framework section
// ===========================================================================

/// The HIGHEST framework-section version this build writes.
///
/// The version a capture actually stamps is CONTENT-dependent, and that is the
/// point: a section carrying an input-service table is a version-2
/// section, and one carrying only the three version-1 fields is still written
/// as version 1, byte-for-byte the form written before version 2 existed. So a
/// `Period`/`Sync` graph's anchors do not grow, and the version MEANS "these
/// bytes are here" rather than "this is the build that wrote them".
///
/// A reader accepts any version at or above [`FRAMEWORK_SECTION_MIN_READABLE`],
/// reads every structure the version declares, and IGNORES whatever follows
/// within the section's declared length — which is what makes a future field an
/// addition rather than a break. It is the section's LENGTH, not this number,
/// that bounds the skip.
pub const FRAMEWORK_SECTION_WRITE_VERSION: u32 = 2;

/// The LOWEST framework-section version this build can read.
///
/// # Why this is separate from [`FRAMEWORK_SECTION_WRITE_VERSION`]
///
/// Were the floor check and the write version ONE constant, bumping it for
/// version 2 would make every version-1 section decode as
/// [`RestoreRefusal::MalformedFrameworkSection`]. That is not a theoretical
/// hazard: every bag recorded before version 2 existed holds version-1 sections, and the reader
/// rules for such a bag (re-inject the whole pre-anchor band, loudly) REQUIRE
/// the section to still be readable.
///
/// So the floor is a FLOOR — the oldest layout this build still has a decoder
/// for — and it moves only when a layout is genuinely dropped, which is a
/// separate and much louder decision than adding a field.
pub const FRAMEWORK_SECTION_MIN_READABLE: u32 = 1;

const _: () = assert!(
    FRAMEWORK_SECTION_MIN_READABLE <= FRAMEWORK_SECTION_WRITE_VERSION,
    "the readable floor cannot sit above the version this build writes"
);

/// The first section version that carries the input-service table.
const FRAMEWORK_SECTION_INPUT_SERVICE_VERSION: u32 = 2;

/// Bytes a version-1 section carries ahead of its sync-input table:
/// `version(u32) ++ next_fire_present(u8) ++ next_fire_ns(u64) ++
/// pending_data_count(u64) ++ sync_count(u32)`.
const FRAMEWORK_SECTION_V1_PREFIX: usize = 4 + 1 + 8 + 8 + 4;

/// The CARRIER's own plain-data view of ONE node at the anchor boundary.
///
/// # Whose view this is
///
/// It is the CAPTURE CARRIER's composite, not one subsystem's. Three of the
/// fields are read off the `Scheduler` (`next_fire_ns`, `pending_data_count`,
/// `sync_input_timestamps` — `Scheduler::node_framework_state` supplies them);
/// [`Self::input_service`] is read off the graph runtime's per-input SERVICE
/// CURSORS, which live on the transport ports. They ride ONE section because a
/// reader cannot usefully hold one without the other: the scheduler's counters
/// say what the node was owed, and the cursors say which frames it had already
/// been served, and only the pair describes the state a resume must re-create.
///
/// Naming it "the scheduler's view" would be
/// a misleading name: a reader who trusted
/// it would look for the cursor in `Scheduler` and find nothing.
///
/// # Why the scheduler's state is captured at all
///
/// `ScheduledNode` is not serializable as a struct: it carries
/// `callback: Box<dyn FnMut() + Send>` and `Arc<AtomicU64>` handles shared with
/// `NodeHandle`. Those are WIRING, rebuilt by `GraphRuntime::build`. What a
/// resumed graph cannot rebuild is the node's TIMING and TRIGGER state — the
/// first three plain-data fields below — and without them a resumed graph
/// executes a different schedule from the one the recording holds while
/// reporting the difference as the candidate's fault:
///
/// * `next_fire_ns` — a `Period` node rebuilt at `clock.now_ns() + interval`
///   with the clock still at its origin has a deadline far below the anchor
///   instant, so its first resumed step bursts (MEASURED:
///   four fires where the recording holds one).
/// * `pending_data_count` — a `Data` node whose fire was deferred at the anchor
///   (a `block`/`throttle_ms` gate, or a tripped circuit breaker) holds
///   signalled-but-unfired arrivals that a rebuilt scheduler has none of.
/// * `sync_input_timestamps` — a worked example: a
///   `sync_window_ms = 50` fusion node checkpointed with `/cam` arrived and
///   `/lidar` not must restore with `/cam` STILL arrived, or it waits for a
///   second `/cam` the recording never sent.
///
/// # It is a VALUE, never an identity
///
/// The rule for the shared counters: what is captured is the value, which
/// restore writes into the REBUILT `Arc`. Nothing here names a handle, a
/// pointer or an index — a section is meaningful in a process that never saw
/// the one that wrote it, which is the same desk/robot split every other
/// reader in this module assumes.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct NodeFrameworkState {
    /// `ScheduledNode::next_fire_ns` — the gating-clock instant this node's
    /// `Period` trigger is next due at. `None` for every non-`Period` trigger,
    /// which carries no timing state.
    pub next_fire_ns: Option<u64>,
    /// `ScheduledNode::pending_data_count` — arrivals signalled into a `Data`
    /// trigger and not yet consumed by a fire.
    pub pending_data_count: u64,
    /// `ScheduledNode::sync_input_timestamps` — per trigger-input TOPIC, the
    /// timestamp of the arrival currently satisfying it.
    ///
    /// A `BTreeMap` rather than the scheduler's insertion-ordered `IndexMap`
    /// because these bytes land in a bag: byte-determinism requires a total order that does
    /// not depend on how a particular run happened to wire its inputs, or two
    /// byte-identical captures would produce two different bags. Nothing reads
    /// the order back — `Scheduler::check_sync` looks each input up by name.
    pub sync_input_timestamps: BTreeMap<String, u64>,
    /// Per PER-MESSAGE-FIFO trigger input (name -> wire `sequence`),
    /// the last frame the node's TICK actually READ before the anchor.
    ///
    /// `None` (the outer option) is the recording saying NOTHING — a version-1
    /// section, i.e. every anchor written before version 2 existed. `Some(map)` is a
    /// positive statement, and an EMPTY map inside it says "this node has no
    /// per-message FIFO input", which is a different claim from silence. A
    /// `None` VALUE inside the map says the input had been served no frame at
    /// all. Read them through [`Self::service_cursor`] rather than by hand, so
    /// the three cases stay distinguishable at every call site.
    ///
    /// # Why a wire SEQUENCE and not a count
    ///
    /// It is an IDENTITY, so it survives everything a count does not: a
    /// recorder tap that attached mid-stream, junk frames a drain skipped, and
    /// `sample(N)` decimation each shift how many frames a topic holds without
    /// shifting which frame the tick last read. `WireHeader::sequence` is
    /// gap-free at commit on both production publisher paths.
    ///
    /// # Why SERVED and not POPPED
    ///
    /// A frame popped into the boundary's frozen slot but never handed to the
    /// tick (a deferred fire, or the nested-read collapse) is RE-OFFERED
    /// at the next boundary — but a rebuilt subscriber has no frozen slot, so a
    /// resume must re-inject that frame. Advancing at serve keeps both runs'
    /// first resumed step reading the same frame; advancing at pop would skip
    /// it and starve the step.
    pub input_service: Option<BTreeMap<String, Option<u32>>>,
}

/// What a recording says about ONE input's service cursor.
///
/// Three-valued on purpose: "the recording states nothing" and "the recording
/// states that nothing was served" lead to OPPOSITE reader rules — the first
/// must re-inject the whole pre-anchor band (and say so), the second is a
/// positive claim that the band is genuinely all unread. Collapsing them into
/// one `Option` is what would make an older bag silently claim its
/// consumers had read nothing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputServiceCursor {
    /// The recording states nothing about this input — a version-1 section, a
    /// node with no section at all, or an input the section's table omits.
    Absent,
    /// The recording states this input had been served NO frame at the anchor.
    NothingServed,
    /// The wire `sequence` of the last frame this input's tick READ.
    Served(u32),
}

impl NodeFrameworkState {
    /// Whether this section states anything at all.
    ///
    /// A carrier uses it to skip the v2 framing entirely for a node the
    /// scheduler has nothing to say about, so a graph of `External` nodes with
    /// no pending data writes exactly the v1 bytes.
    pub fn is_empty(&self) -> bool {
        self.next_fire_ns.is_none()
            && self.pending_data_count == 0
            && self.sync_input_timestamps.is_empty()
            // A table whose every cursor is `None` states nothing a
            // reader can act on — every rule keyed on it answers "re-inject the
            // whole band", which is also what an absent table answers. Treating
            // it as empty keeps EVERY data-trigger node on the version-1 header
            // it has always written until one of its inputs has actually
            // been served something, instead of moving the whole population to
            // a v2 header + section for a table of nothing (which would grow
            // `anchor_framing_bytes` — and with it the RecorderBehind precheck
            // — on every graph in the repo).
            //
            // COST, stated: an input that genuinely served nothing is then
            // indistinguishable from an older recording, so it takes the
            // Absent rule (re-inject the band, loudly) rather than the
            // NothingServed one. Both re-inject the same frames; only the
            // loudness differs, and a mid-run resume of a node that has served
            // nothing is the shape a from-start bag covers anyway.
            && self
                .input_service
                .as_ref()
                .is_none_or(|m| m.values().all(Option::is_none))
    }

    /// What this section says about `input`'s service cursor.
    ///
    /// The ONE reader of [`Self::input_service`], so the three cases cannot be
    /// collapsed at a call site.
    pub fn service_cursor(&self, input: &str) -> InputServiceCursor {
        match self.input_service.as_ref().map(|m| m.get(input)) {
            None | Some(None) => InputServiceCursor::Absent,
            Some(Some(None)) => InputServiceCursor::NothingServed,
            Some(Some(Some(seq))) => InputServiceCursor::Served(*seq),
        }
    }

    /// The section version these bytes are written at — CONTENT-dependent, see
    /// [`FRAMEWORK_SECTION_WRITE_VERSION`].
    fn wire_version(&self) -> u32 {
        if self.input_service.is_some() {
            FRAMEWORK_SECTION_INPUT_SERVICE_VERSION
        } else {
            1
        }
    }

    /// Bytes [`Self::encode`] will write.
    ///
    /// Computed analytically rather than by encoding into a scratch buffer:
    /// the inline carrier runs this on the node thread inside the boundary's
    /// byte budget, where an allocation is exactly what the bounded
    /// attempt exists to avoid.
    pub fn encoded_len(&self) -> usize {
        FRAMEWORK_SECTION_V1_PREFIX
            + self
                .sync_input_timestamps
                .keys()
                .map(|name| 4 + name.len() + 8)
                .sum::<usize>()
            + self
                .input_service
                .as_ref()
                .map(|m| 4 + m.keys().map(|name| 4 + name.len() + 1 + 4).sum::<usize>())
                .unwrap_or(0)
    }

    /// Write the section body — everything INSIDE the length the framing
    /// declares.
    ///
    /// Little-endian, hand-built and padding-free, like every other wire form
    /// in this repo: the bytes are a format contract, so they must not depend
    /// on the recording host's endianness or on a struct layout.
    pub fn encode(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        out.write(&self.wire_version().to_le_bytes())?;
        out.write(&[u8::from(self.next_fire_ns.is_some())])?;
        out.write(&self.next_fire_ns.unwrap_or(0).to_le_bytes())?;
        out.write(&self.pending_data_count.to_le_bytes())?;
        let count =
            u32::try_from(self.sync_input_timestamps.len()).map_err(|_| StateError::SinkFull)?;
        out.write(&count.to_le_bytes())?;
        for (name, ts) in &self.sync_input_timestamps {
            let name_len = u32::try_from(name.len()).map_err(|_| StateError::SinkFull)?;
            out.write(&name_len.to_le_bytes())?;
            out.write(name.as_bytes())?;
            out.write(&ts.to_le_bytes())?;
        }
        // The version-2 tail. Written only when the section STATES a
        // table (`wire_version` above), so a section with nothing to say here
        // is still a version-1 section, byte for byte.
        if let Some(cursors) = &self.input_service {
            let count = u32::try_from(cursors.len()).map_err(|_| StateError::SinkFull)?;
            out.write(&count.to_le_bytes())?;
            for (name, seq) in cursors {
                let name_len = u32::try_from(name.len()).map_err(|_| StateError::SinkFull)?;
                out.write(&name_len.to_le_bytes())?;
                out.write(name.as_bytes())?;
                out.write(&[u8::from(seq.is_some())])?;
                out.write(&seq.unwrap_or(0).to_le_bytes())?;
            }
        }
        Ok(())
    }

    /// Read a section body back.
    ///
    /// Total inverse of [`Self::encode`] for a version-1 section. A section
    /// declaring a HIGHER version is read for its version-1 prefix and its
    /// remainder ignored — the additive contract — while anything this build
    /// cannot make sense of is a refusal naming the node and what was wrong,
    /// never a silently empty section.
    pub fn decode(node: &str, bytes: &[u8]) -> Result<Self, RestoreRefusal> {
        let malformed = |reason: String| RestoreRefusal::MalformedFrameworkSection {
            node: node.to_string(),
            reason,
        };
        if bytes.len() < FRAMEWORK_SECTION_V1_PREFIX {
            return Err(malformed(format!(
                "a framework section is at least {FRAMEWORK_SECTION_V1_PREFIX} bytes, got {}",
                bytes.len()
            )));
        }
        let version = u32::from_le_bytes(bytes[0..4].try_into().expect("4 bytes"));
        if version < FRAMEWORK_SECTION_MIN_READABLE {
            // Version 0 is the zeroed-slot reading, and there is no version
            // below the first one this repo ever wrote — so a lower number is
            // not an OLDER section, it is a section this build has no layout
            // for. Refusing beats reading `next_fire_ns` out of bytes that may
            // mean something else.
            //
            // The floor is MIN_READABLE, never the WRITE version: bumping the
            // write version must not un-read the entire pre-existing bag
            // population (see `FRAMEWORK_SECTION_MIN_READABLE`).
            return Err(malformed(format!(
                "section version {version} is below {FRAMEWORK_SECTION_MIN_READABLE}, the first \
                 version this format ever had"
            )));
        }
        let next_fire_present = bytes[4];
        if next_fire_present > 1 {
            return Err(malformed(format!(
                "the next-fire presence byte is {next_fire_present}, which is neither 0 nor 1"
            )));
        }
        let next_fire_raw = u64::from_le_bytes(bytes[5..13].try_into().expect("8 bytes"));
        let pending_data_count = u64::from_le_bytes(bytes[13..21].try_into().expect("8 bytes"));
        let sync_count = u32::from_le_bytes(bytes[21..25].try_into().expect("4 bytes")) as usize;

        let mut sync_input_timestamps = BTreeMap::new();
        let mut cursor = FRAMEWORK_SECTION_V1_PREFIX;
        for i in 0..sync_count {
            let name_end = cursor
                .checked_add(4)
                .filter(|end| *end <= bytes.len())
                .ok_or_else(|| {
                    malformed(format!(
                        "sync input {i} of {sync_count} has no length prefix — the section ends \
                         after {} byte(s)",
                        bytes.len()
                    ))
                })?;
            let name_len =
                u32::from_le_bytes(bytes[cursor..name_end].try_into().expect("4 bytes")) as usize;
            let ts_start = name_end.checked_add(name_len).ok_or_else(|| {
                malformed(format!(
                    "sync input {i} declares a name of {name_len} bytes"
                ))
            })?;
            let ts_end = ts_start
                .checked_add(8)
                .filter(|end| *end <= bytes.len())
                .ok_or_else(|| {
                    malformed(format!(
                        "sync input {i} of {sync_count} declares a {name_len}-byte name and a \
                         timestamp, which do not fit the section's remaining {} byte(s)",
                        bytes.len().saturating_sub(name_end)
                    ))
                })?;
            let name = std::str::from_utf8(&bytes[name_end..ts_start])
                .map_err(|e| malformed(format!("sync input {i}'s name is not UTF-8: {e}")))?;
            let ts = u64::from_le_bytes(bytes[ts_start..ts_end].try_into().expect("8 bytes"));
            if sync_input_timestamps.insert(name.to_string(), ts).is_some() {
                return Err(malformed(format!(
                    "sync input '{name}' appears twice, so the section states two timestamps for \
                     one input"
                )));
            }
            cursor = ts_end;
        }

        // The version-2 tail. A version-1 section carries NO table
        // and decodes to `None` — the recording states nothing — which is a
        // different fact from a version-2 section declaring a count of zero,
        // and every reader rule keyed on the cursor depends on telling them
        // apart. Bytes AFTER the table are still ignored (the additive
        // contract): the section's declared LENGTH bounds the skip.
        let input_service = if version >= FRAMEWORK_SECTION_INPUT_SERVICE_VERSION {
            let count_end = cursor
                .checked_add(4)
                .filter(|end| *end <= bytes.len())
                .ok_or_else(|| {
                    malformed(format!(
                        "a version-{version} section declares an input-service table, but the \
                         section ends after {} byte(s) with no count word",
                        bytes.len()
                    ))
                })?;
            let count =
                u32::from_le_bytes(bytes[cursor..count_end].try_into().expect("4 bytes")) as usize;
            cursor = count_end;
            let mut cursors: BTreeMap<String, Option<u32>> = BTreeMap::new();
            for i in 0..count {
                let name_end = cursor
                    .checked_add(4)
                    .filter(|end| *end <= bytes.len())
                    .ok_or_else(|| {
                        malformed(format!(
                            "service cursor {i} of {count} has no length prefix — the section \
                             ends after {} byte(s)",
                            bytes.len()
                        ))
                    })?;
                let name_len =
                    u32::from_le_bytes(bytes[cursor..name_end].try_into().expect("4 bytes"))
                        as usize;
                let value_start = name_end.checked_add(name_len).ok_or_else(|| {
                    malformed(format!(
                        "service cursor {i} declares a name of {name_len} bytes"
                    ))
                })?;
                let value_end = value_start
                    .checked_add(5)
                    .filter(|end| *end <= bytes.len())
                    .ok_or_else(|| {
                        malformed(format!(
                            "service cursor {i} of {count} declares a {name_len}-byte name and a \
                             cursor, which do not fit the section's remaining {} byte(s)",
                            bytes.len().saturating_sub(name_end)
                        ))
                    })?;
                let name = std::str::from_utf8(&bytes[name_end..value_start]).map_err(|e| {
                    malformed(format!("service cursor {i}'s name is not UTF-8: {e}"))
                })?;
                let present = bytes[value_start];
                if present > 1 {
                    return Err(malformed(format!(
                        "service cursor {i}'s presence byte is {present}, which is neither 0 nor 1"
                    )));
                }
                let seq = u32::from_le_bytes(
                    bytes[value_start + 1..value_end]
                        .try_into()
                        .expect("4 bytes"),
                );
                if cursors
                    .insert(name.to_string(), (present == 1).then_some(seq))
                    .is_some()
                {
                    return Err(malformed(format!(
                        "input '{name}' appears twice in the service-cursor table, so the section \
                         states two cursors for one input"
                    )));
                }
                cursor = value_end;
            }
            Some(cursors)
        } else {
            None
        };

        Ok(Self {
            next_fire_ns: (next_fire_present == 1).then_some(next_fire_raw),
            pending_data_count,
            sync_input_timestamps,
            input_service,
        })
    }
}

// ===========================================================================
// Anchor facts and the resume selection
// ===========================================================================

/// What became of ONE node's anchor at ONE step.
///
/// This is `state_ring::StateAnchorEvent` with the bytes removed: the
/// selection is a decision about COMPLETENESS, and handing it payloads would
/// make every oracle carry megabytes to assert on a verdict that never reads
/// them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AnchorOutcome {
    /// Every part arrived; the blob is whole.
    Complete,
    /// A part was lost, duplicated, reordered or short — the anchor is void.
    Torn,
    /// The carrier declined to capture this node, naming why.
    Skipped(SkipCause),
}

/// One node's anchor outcome at one step of one run.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AnchorFact {
    /// The run the anchor belongs to — a bag can carry several.
    pub run_id: u64,
    /// The step the anchor is "at": the state AFTER step `step` completed.
    pub step: u64,
    /// The node id the recorder resolved this anchor to.
    pub node: String,
    /// What became of it.
    pub outcome: AnchorOutcome,
}

/// Everything [`plan_restore`] needs, and nothing that would let it read a bag.
#[derive(Debug, Clone)]
pub struct RestoreRequest<'a> {
    /// The run whose anchors are eligible.
    pub run_id: u64,
    /// Every node the replay will EXECUTE. All-or-nothing is judged over
    /// exactly this set, so a node the cut leaves out cannot veto an
    /// anchor it is not part of.
    pub required_nodes: &'a [String],
    /// The step of the first `STEP_BOUNDARY` the bag's trace carries.
    ///
    /// This is the rendezvous' other half. It is an INPUT rather than
    /// something derived here because only the reader that walked the trace
    /// knows it, and having both halves lets the plan CHECK the rendezvous rather than
    /// assume it.
    pub first_recorded_step: u64,
    /// Every anchor outcome the bag carries.
    pub facts: &'a [AnchorFact],
}

/// The anchor a restore resumes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResumeAnchor {
    /// The run it belongs to.
    pub run_id: u64,
    /// `S` — the state is that of the graph AFTER step `S` completed.
    pub step: u64,
}

impl ResumeAnchor {
    /// The first step the replay EXECUTES: `S + 1`.
    ///
    /// A method rather than a field because the rendezvous is arithmetic, and
    /// a stored second copy of it is a second thing that can disagree.
    pub fn first_replay_step(&self) -> u64 {
        self.step.saturating_add(1)
    }
}

/// What a restore does before the first step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RestorePlan {
    /// Nothing: the recording begins at step 0, where every node's state is
    /// exactly what its constructor produces. The empty checkpoint IS
    /// the correct checkpoint, and from-start replay rests on exactly that,
    /// so a from-start bag replays byte-identically
    /// whether or not it also carries anchors.
    FromStart,
    /// Apply an anchor, then execute from `S + 1`.
    FromAnchor(ResumeAnchor),
}

/// Why one node's part of an anchor is unusable.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NodeAnchorProblem {
    /// The bag carries no anchor for this node at this step.
    ///
    /// This is the ordinary reading of a node
    /// that has no capture carrier or no `CerulionState` derive — which
    /// is why such a node narrows which selections an anchor
    /// serves rather than voiding the anchor: it is only fatal when the node
    /// is one the replay must execute.
    Missing,
    /// The anchor's records did not reassemble (lost/reordered/short part).
    Torn,
    /// The carrier declined, naming its cause.
    Skipped(SkipCause),
}

impl fmt::Display for NodeAnchorProblem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Missing => f.write_str("no anchor recorded"),
            Self::Torn => f.write_str("anchor torn (a part was lost, reordered or short)"),
            Self::Skipped(cause) => write!(f, "capture skipped ({cause:?})"),
        }
    }
}

/// The one topic-loss fact the lossy-bag refusal reads.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TopicLoss {
    /// The recorded topic.
    pub topic: String,
    /// `record_health.json`'s per-topic `frames_lost`.
    pub frames_lost: u64,
    /// `record_coverage.json`'s per-topic `prefix_lost`.
    pub prefix_lost: u64,
}

/// Every way a restore refuses, each naming the offender and the fix.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum RestoreRefusal {
    /// The rendezvous anchor does not cover every node the replay must run.
    #[error(
        "the recording's anchor at step {anchor_step} does not cover {} of the {} node(s) this \
         replay executes: {}. A replay that resumes mid-run needs EVERY executed node's state at \
         one step. Fix: record from step 0 (`cerulion graph run --record`), or \
         re-record with a recorder attached long enough for a complete anchor. If this bag is a \
         mid-run attach, run `cerulion bag info` on it first: an attach that DECLINED the run's \
         node-state rings — because a Flashback recorder was already draining them — records no \
         anchors at all, and no attach duration changes that; its anchors, IF that recorder \
         captured any, are in its captures. `bag info` prints nothing on that line for a bag \
         that is not a mid-run attach, which is how you tell the two apart",
        problems.len(),
        problems.len() + complete_nodes,
        render_problems(problems)
    )]
    AnchorIncomplete {
        /// `S`, the step the anchor claims to describe.
        anchor_step: u64,
        /// Each uncovered node and why.
        problems: Vec<(String, NodeAnchorProblem)>,
        /// How many executed nodes DID have a complete anchor — the true
        /// denominator, so "3 of 40" reads differently from "3 of 3".
        complete_nodes: usize,
    },

    /// A topic the replay re-executes is lossy, so its sequence seed cannot be
    /// derived.
    #[error(
        "the recording is lossy on {} re-executed topic(s): {}. Publisher sequences are DERIVED \
         from the bag (the counter advances at commit), so a lost frame permanently \
         offsets the seed and every subsequent frame reads as a byte mismatch. Fix: replay a \
         recording with no loss on these topics, or narrow the replay so they are injected \
         rather than re-executed",
        topics.len(),
        render_losses(topics)
    )]
    LossyTopics {
        /// Each lossy re-executed topic with its counts.
        topics: Vec<TopicLoss>,
    },

    /// The recorded state's type shape differs from the current build's.
    #[error(
        "node '{node}' was recorded with state shape {recorded:#018x} but this build declares \
         {current:#018x}. A field was retyped, added, removed, renamed OR REORDERED, so the \
         recorded bytes no longer describe this type and applying them would restore silently \
         wrong values. SCOPE: this voids EVERY anchor for this node in the \
         recording, not just this one — the shape is a property of the type, so no earlier \
         anchor is any more applicable than this one. TOLERANCE: there is none. The recorded \
         identity is ONE hash over every field name and type in declaration order, and \
         `cer_capture` writes fields POSITIONALLY, so ANY change to a field's name, type, \
         ORDER or count is terminal and anything else restores — a reorder is refused, and an \
         added field is refused rather than defaulted. Fix: check out the node source the \
         recording was made against, or re-record against this build"
    )]
    ShapeDrift {
        /// The node whose shape moved.
        node: String,
        /// The shape the bag carries.
        recorded: u64,
        /// The shape this build declares.
        current: u64,
    },

    /// An anchor blob carries no [`ANCHOR_BLOB_MAGIC`] framing.
    #[error(
        "node '{node}'s anchor is {bytes} byte(s) that do not open with the anchor \
         framing, so its recorded state shape is unknown and it cannot be checked against this \
         build. Fix: re-record with a build whose capture writes the anchor header \
         (`cerulion_core::state_restore::capture_anchor_blob`)"
    )]
    UnframedAnchor {
        /// The node whose anchor could not be framed.
        node: String,
        /// How many bytes the anchor held.
        bytes: usize,
    },

    /// An anchor's framework section could not be read.
    #[error(
        "node '{node}'s anchor carries a framework section this build cannot read: \
         {reason}. The section states the scheduler's own timing and trigger state at the \
         anchor, so a replay that ignored it would resume on a schedule the recording does not \
         describe and report the difference as a node change. Fix: replay with the build the \
         recording was made with, or re-record against this one"
    )]
    MalformedFrameworkSection {
        /// The node whose section could not be read.
        node: String,
        /// What was wrong with it, in the decoder's own words.
        reason: String,
    },

    /// Two consumers of one topic captured different backlog depths.
    #[error(
        "topic '{topic}' was captured with disagreeing undrained backlogs: {}. Re-injecting one \
         depth over-feeds the other consumer and diverges it on its first drain. Fix: replay a recording whose anchor was taken when the consumers agreed, \
         or narrow the replay so this topic has one executed consumer",
        render_claims(claims)
    )]
    BacklogDisagreement {
        /// The topic whose consumers disagree.
        topic: String,
        /// Each consumer and the backlog it captured.
        claims: Vec<(String, u32)>,
    },

    /// The bag holds fewer pre-anchor frames than the captured backlog claims.
    #[error(
        "topic '{topic}' captured an undrained backlog of {claimed} frame(s) at the anchor, but \
         the recording holds only {available} frame(s) at or before it. The restored consumer \
         would start {} frame(s) short of the state it was captured in. Fix: replay a recording that covers the anchor's backlog",
        claimed - available
    )]
    BacklogShortfall {
        /// The topic that is short.
        topic: String,
        /// Frames the anchor says were queued.
        claimed: u32,
        /// Frames the bag actually holds at or before the anchor.
        available: u32,
    },

    /// A re-executed topic has more than one writer, so no single seed can be
    /// right for all of them.
    #[error(
        "topic '{topic}' is listed in the graph's `multi_publisher_topics:`, so its recorded \
         stream INTERLEAVES one wire-sequence counter per publisher — and a frame carries no \
         publisher identity, so the bag cannot say which counter any recorded sequence belonged \
         to. One seed applied to every publisher is therefore wrong for at least one of them, and \
         every frame it emits reads as a byte mismatch that looks like a node change. \
         Fix: narrow the replay so this topic is injected rather than \
         re-executed, or replay from step 0, where every publisher starts at 0 and no seed is \
         needed"
    )]
    MultiPublisherTopicNotSeedable {
        /// The listed topic.
        topic: String,
    },

    /// No rung of the seed ladder could name a re-executed topic's starting
    /// sequence.
    #[error(
        "topic '{topic}' has no recorded frame at or after the anchor and no captured commit \
         counter, so the sequence a restored publisher must start at is unknown. Seeding it \
         wrongly makes every subsequent frame a byte mismatch that reads as a node change. \
         Fix: replay a recording that covers this topic, or narrow \
         the replay so it is not re-executed"
    )]
    SequenceSeedUnavailable {
        /// The topic with no seed evidence.
        topic: String,
    },

    /// `--strict-state` was asked for and some executed node cannot restore.
    #[error(
        "--strict-state requires every executed node to restore its recorded state, but {} \
         node(s) declare none: {}. Fix: drop --strict-state to replay them from their \
         constructors, or add `#[derive(CerulionState)]` to those node types and re-record",
        nodes.len(),
        nodes.join(", ")
    )]
    StrictStateUnsatisfied {
        /// The executed nodes that declare no state.
        nodes: Vec<String>,
    },
}

fn render_problems(problems: &[(String, NodeAnchorProblem)]) -> String {
    problems
        .iter()
        .map(|(node, problem)| format!("{node} ({problem})"))
        .collect::<Vec<_>>()
        .join(", ")
}

fn render_losses(topics: &[TopicLoss]) -> String {
    topics
        .iter()
        .map(|t| {
            format!(
                "{} (frames_lost={}, prefix_lost={})",
                t.topic, t.frames_lost, t.prefix_lost
            )
        })
        .collect::<Vec<_>>()
        .join(", ")
}

fn render_claims(claims: &[(String, u32)]) -> String {
    claims
        .iter()
        .map(|(consumer, pending)| format!("{consumer}={pending}"))
        .collect::<Vec<_>>()
        .join(", ")
}

/// Decide what a replay of this recording must do before its first step.
///
/// # The rendezvous is CHECKED, not searched
///
/// The rendezvous is `checkpoint@S == trace-gate@S+1 == frames-from-S+1`: a recorder
/// attaching mid-run sets the arm word's `first_anchor_step` so the checkpoint
/// and the trace gate land on ONE `S` by construction. So this function does
/// not hunt for "the newest usable anchor" — it derives the step the trace
/// already committed to (`first_recorded_step - 1`) and asks whether the
/// anchor at exactly that step is whole.
///
/// Searching would be strictly worse: picking an EARLIER anchor than the trace
/// begins at leaves steps to execute for which no boundary was recorded, and
/// picking a LATER one silently discards recorded steps. Either produces a
/// replay whose verdict is about a different execution than the one the bag
/// describes.
///
/// A `first_recorded_step` of 0 means the recording began at step 0, where the
/// constructor's state IS the correct state — so the answer is
/// [`RestorePlan::FromStart`] and every existing replay behaviour is
/// byte-unchanged.
pub fn plan_restore(request: &RestoreRequest<'_>) -> Result<RestorePlan, RestoreRefusal> {
    if request.first_recorded_step == 0 {
        return Ok(RestorePlan::FromStart);
    }
    let anchor_step = request.first_recorded_step - 1;

    let mut problems: Vec<(String, NodeAnchorProblem)> = Vec::new();
    let mut complete_nodes = 0usize;

    for node in request.required_nodes {
        let fact = request
            .facts
            .iter()
            .find(|f| f.run_id == request.run_id && f.step == anchor_step && &f.node == node);
        match fact.map(|f| &f.outcome) {
            Some(AnchorOutcome::Complete) => complete_nodes += 1,
            Some(AnchorOutcome::Torn) => {
                problems.push((node.clone(), NodeAnchorProblem::Torn));
            }
            Some(AnchorOutcome::Skipped(cause)) => {
                problems.push((node.clone(), NodeAnchorProblem::Skipped(*cause)));
            }
            None => problems.push((node.clone(), NodeAnchorProblem::Missing)),
        }
    }

    if problems.is_empty() {
        Ok(RestorePlan::FromAnchor(ResumeAnchor {
            run_id: request.run_id,
            step: anchor_step,
        }))
    } else {
        Err(RestoreRefusal::AnchorIncomplete {
            anchor_step,
            problems,
            complete_nodes,
        })
    }
}

/// What a runtime-level restore actually applied.
///
/// Every field is a LIST rather than a count, because each one is something an
/// operator may have to act on and "3 nodes declared no state" does not say
/// which three.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct RestoreReport {
    /// Nodes whose recorded state was decoded and applied, in graph order.
    pub restored: Vec<String>,
    /// Nodes that were offered an anchor but declare no restorable state.
    ///
    /// The default reading is benign (a stateless node, or one whose type has
    /// no `CerulionState` derive), which is why
    /// this is a report rather than an error — and why `--strict-state`
    /// exists for an operator who wants it refused instead.
    pub declared_no_state: Vec<String>,
    /// Anchors for nodes this graph does not contain.
    ///
    /// Normal for a machine-wide recording that carries several runs, and
    /// indistinguishable from a typo'd selection from inside the runtime — so
    /// it is handed back rather than judged here.
    pub anchors_unused: Vec<String>,
}

// ===========================================================================
// The shape gate
// ===========================================================================

/// Judge a recorded anchor's state shape against this build's.
///
/// # What the encoding can and cannot tell you
///
/// A per-FIELD drift table — reorders fine, adds/removes
/// tolerated-and-named, retypes terminal — would require the encoding to be
/// name-keyed per field. The encoding is not that: `cer_capture` writes
/// fields POSITIONALLY and the identity is ONE `u64` per type
/// ([`crate::state::StateShape`]), folded over every field name and type in
/// declaration order. So the reachable table has two rows, and it is stated
/// here rather than in a doc that would drift from it:
///
/// | Edit | Outcome |
/// |---|---|
/// | anything that changes a field's name, type, order or count | **terminal** |
/// | anything that does not | restores |
///
/// That is STRICTER than a per-field table in both directions it moves: a reorder is
/// refused where such a table would allow it, and an added field is refused where
/// it would default it. Strictness is the safe direction — a refusal names
/// the node and both hashes, while the alternative restores bytes under a
/// layout that no longer describes them.
pub fn classify_shape(node: &str, recorded: u64, current: u64) -> Result<(), RestoreRefusal> {
    if recorded == current {
        Ok(())
    } else {
        Err(RestoreRefusal::ShapeDrift {
            node: node.to_string(),
            recorded,
            current,
        })
    }
}

// ===========================================================================
// The lossy-bag refusal
// ===========================================================================

/// Refuse a replay whose RE-EXECUTED topics lost frames.
///
/// A re-executed topic's publisher sequence is derived from the bag rather
/// than stored (the reason is that the counter advances at COMMIT, so a
/// committed-then-lost frame permanently offsets it). The byte diff compares
/// the whole wire header including `sequence`, so a seed off by one turns
/// every subsequent frame into a `ByteMismatch` that reads to an operator as
/// "my node changed".
///
/// Only re-executed topics are judged. An INJECTED topic is replayed
/// verbatim — its recorded bytes carry their own sequences — so a gap there
/// costs fidelity to the original run, not correctness of the diff, and
/// refusing over it would make the feature unusable on any bag with a lossy
/// bystander.
pub fn refuse_lossy_reexecuted_topics(
    reexecuted: &[String],
    losses: &[TopicLoss],
) -> Result<(), RestoreRefusal> {
    let lossy: Vec<TopicLoss> = losses
        .iter()
        .filter(|l| (l.frames_lost > 0 || l.prefix_lost > 0) && reexecuted.contains(&l.topic))
        .cloned()
        .collect();
    if lossy.is_empty() {
        Ok(())
    } else {
        Err(RestoreRefusal::LossyTopics { topics: lossy })
    }
}

// ===========================================================================
// The backlog frame gate
// ===========================================================================

/// One consumer's undrained input backlog at the anchor.
///
/// The value is the scheduler's own `pending_data_count`
/// (`scheduler/mod.rs`), captured in the framework section.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BacklogClaim {
    /// The topic the frames are queued on.
    pub topic: String,
    /// The consumer node holding them.
    pub consumer: String,
    /// How many frames it had queued and undrained when the anchor was taken.
    pub pending: u32,
}

/// How many recorded frames a topic holds at or before the anchor.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TopicFrameCount {
    /// The topic.
    pub topic: String,
    /// Frames recorded with a timestamp at or before the anchor boundary.
    pub frames_at_or_before_anchor: u32,
}

/// Per topic, how many pre-anchor frames a restore must re-inject.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct BacklogAdmission {
    /// Topic -> trailing frame count to admit from at-or-before the anchor.
    /// A topic with no claim is absent, and absent means ZERO: the plain
    /// rule (frames from `S+1` onward) is the correct one for every topic that
    /// was fully drained.
    pub per_topic: BTreeMap<String, u32>,
}

impl BacklogAdmission {
    /// Frames to admit from before the anchor for `topic` — 0 if it had none.
    pub fn admitted(&self, topic: &str) -> u32 {
        self.per_topic.get(topic).copied().unwrap_or(0)
    }

    /// Whether any topic needs pre-anchor frames at all.
    pub fn is_empty(&self) -> bool {
        self.per_topic.is_empty()
    }
}

/// The backlog frame gate: decide which frames from BEFORE the anchor a restored
/// graph must still see.
///
/// # The hole this closes
///
/// The anchor captures `pending_data_count` — the frames a consumer had queued and
/// NOT yet drained when the anchor was taken — while the plain frame gate admits frames
/// from `S+1` only. Those two are inconsistent on their own: a data-trigger consumer with a
/// three-frame backlog at `S` is restored believing it has three frames to
/// read, and the frame walk hands it none, so its first post-restore drains
/// diverge silently. That is a silent divergence
/// manufactured by the restore rather than by the user's code.
///
/// The rule is therefore: per topic, admit the LAST `pending` frames at or
/// before `S`, and require the recording to actually hold them.
///
/// # Two consumers of one topic that disagree is TERMINAL, not a maximum
///
/// Each consumer has its own transport queue, but the replay re-injects into
/// ONE topic, so every consumer of it sees the same frames. If consumer A
/// captured 3 pending and consumer B captured 5, no single admission serves
/// both — 5 over-feeds A by two frames and 3 starves B by two — and either
/// choice diverges a node while reporting success. Taking the maximum would be
/// exactly the silent divergence this gate exists to remove, so the
/// disagreement is refused and named.
///
/// This is mechanism-independent: it holds for any capture mechanism, not only
/// the fork carrier.
pub fn plan_backlog_admission(
    claims: &[BacklogClaim],
    available: &[TopicFrameCount],
) -> Result<BacklogAdmission, RestoreRefusal> {
    let mut by_topic: BTreeMap<&str, Vec<(String, u32)>> = BTreeMap::new();
    for claim in claims {
        by_topic
            .entry(claim.topic.as_str())
            .or_default()
            .push((claim.consumer.clone(), claim.pending));
    }

    let mut per_topic = BTreeMap::new();
    for (topic, mut claimants) in by_topic {
        claimants.sort();
        let first = claimants[0].1;
        if claimants.iter().any(|(_, pending)| *pending != first) {
            return Err(RestoreRefusal::BacklogDisagreement {
                topic: topic.to_string(),
                claims: claimants,
            });
        }
        if first == 0 {
            continue;
        }
        let have = available
            .iter()
            .find(|c| c.topic == topic)
            .map(|c| c.frames_at_or_before_anchor)
            .unwrap_or(0);
        if have < first {
            return Err(RestoreRefusal::BacklogShortfall {
                topic: topic.to_string(),
                claimed: first,
                available: have,
            });
        }
        per_topic.insert(topic.to_string(), first);
    }

    Ok(BacklogAdmission { per_topic })
}

// ===========================================================================
// The publisher-sequence seed ladder
// ===========================================================================

/// Which rung of the ladder produced a seed.
///
/// Recorded so a report can say WHERE a sequence came from: the three rungs
/// carry genuinely different evidential strength, and an operator debugging a
/// byte mismatch on frame 1 needs to know which one answered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeedRung {
    /// (i) The last frame recorded at or before the anchor, plus one. The
    /// ordinary case.
    LastFrameBeforeAnchor,
    /// (ii) The first frame recorded AFTER the anchor, taken verbatim: that
    /// frame IS the next one the re-executed publisher emits, so its recorded
    /// sequence is the seed rather than a predecessor of it.
    FirstFrameAfterAnchor,
    /// (iii) The publisher's own last-committed counter, captured in the
    /// framework section, plus one. Authoritative ONLY here — the bag is the
    /// single source of truth everywhere it can answer, because a second
    /// stored copy of the number every gap detector keys on is a second thing
    /// that can be wrong.
    CapturedCommit,
}

/// How many publishers a re-executed topic has, as DECLARED by the caller.
///
/// It is an enum the caller states rather than a bool it infers, for the reason
/// the publisher's own `NotifyListenerCount` is one (the timing is
/// DECLARED by the caller because inferring it is a silent-inversion
/// hazard). The answer is a property of the GRAPH — a topic listed in
/// `multi_publisher_topics:` — and nothing in a bag can be read to recover it,
/// so a seed derived without it is silently wrong rather than loudly missing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TopicWriters {
    /// Exactly one publisher, which is every graph topic that is not listed in
    /// `multi_publisher_topics:` — i.e. every shipping shape but one.
    SingleWriter,
    /// The topic is listed in the graph's `multi_publisher_topics:` opt-in, so
    /// several publishers write it and each carries its OWN counter.
    MultiPublisher,
}

/// What the bag and the anchor know about one re-executed topic's sequence.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SeedEvidence {
    /// The `sequence` of the last frame recorded at or before the anchor.
    pub last_at_or_before_anchor: Option<u32>,
    /// The `sequence` of the first frame recorded after the anchor.
    pub first_after_anchor: Option<u32>,
    /// The publisher's last-committed sequence, captured with the anchor.
    pub captured_last_commit: Option<u32>,
}

/// A re-executed publisher's starting wire sequence, and where it came from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SequenceSeed {
    /// The topic.
    pub topic: String,
    /// The sequence the restored publisher's FIRST frame must carry.
    pub seed: u32,
    /// Which rung answered.
    pub rung: SeedRung,
}

/// The seed ladder: derive a re-executed publisher's starting sequence.
///
/// # Why a one-line rule is not enough
///
/// The one-line rule is "last recorded sequence before S, plus 1". That is undefined for
/// a topic with ZERO recorded frames at or before `S` — the ordinary shape of
/// a WINDOW bag, or of any deliberate mid-run attach whose first frame on a
/// slow topic lands after the anchor. And the `prefix_lost` refusal that would
/// otherwise catch it is structurally silent exactly there: the recorder stamps
/// that marker only under `armed_before_producers`, which is FALSE for a
/// mid-run `bag record` attach by construction (a constructor handed a tap
/// vector cannot know when those producers started).
///
/// So one silent topic seeds at 0 and manufactures a `ByteMismatch` on every
/// subsequent frame — a confident FAIL against a divergence the restore
/// created.
///
/// # The ladder
///
/// Rungs are tried in order and the first that answers wins:
///
/// 1. [`SeedRung::LastFrameBeforeAnchor`] — `last + 1`.
/// 2. [`SeedRung::FirstFrameAfterAnchor`] — that frame's own sequence,
///    verbatim. Recorded evidence beats a captured counter because the bag is
///    what the diff compares against.
/// 3. [`SeedRung::CapturedCommit`] — `captured + 1`, authoritative only here.
/// 4. Otherwise refuse LOUDLY, naming the topic.
///
/// Arithmetic is `wrapping_add`, because the wire sequence is a `u32` that
/// really does wrap and a saturating seed would silently stall a long run's
/// counter at `u32::MAX`.
///
/// # Every rung assumes ONE writer, so a multi-publisher topic is refused first
///
/// `sequence` is a PER-PUBLISHER commit counter. On a topic listed in the
/// graph's `multi_publisher_topics:` the recorded stream is therefore an
/// INTERLEAVING of several unrelated counters, and a wire frame carries no
/// publisher identity — the header is schema hash, sizes, sequence and
/// timestamp, and nothing in it says who wrote it. So rung (i) and rung (ii)
/// read "the last/first recorded sequence" off a stream whose successive frames
/// may belong to different counters, and neither answer is that of any
/// particular publisher. This is the same wall the topic-rate estimator meets:
/// there `newest_sequence` HOPS between counters and would report a confident
/// 1550 Hz for a ~101 Hz two-writer `/tf`, so it likewise withholds
/// the exact answer rather than compute one from a basis that cannot support
/// it.
///
/// Rung (iii) is the one that could in principle serve, since a captured
/// last-committed counter belongs to the publisher that captured it — but the
/// seed is APPLIED per TOPIC (`TransportManager::set_replay_sequence_seeds`),
/// so per-publisher evidence has nowhere to go. Making both halves
/// per-publisher would change the seed interface, and inventing an attribution
/// the wire cannot support would be the fabricated state this module refuses.
///
/// So `MultiPublisher` is refused BEFORE any evidence is consulted — a topic
/// with three rungs of perfectly good evidence is still refused, because the
/// evidence answers a question about the stream and the seed is a question
/// about one publisher.
pub fn seed_publisher_sequence(
    topic: &str,
    writers: TopicWriters,
    evidence: &SeedEvidence,
) -> Result<SequenceSeed, RestoreRefusal> {
    if writers == TopicWriters::MultiPublisher {
        return Err(RestoreRefusal::MultiPublisherTopicNotSeedable {
            topic: topic.to_string(),
        });
    }
    if let Some(last) = evidence.last_at_or_before_anchor {
        return Ok(SequenceSeed {
            topic: topic.to_string(),
            seed: last.wrapping_add(1),
            rung: SeedRung::LastFrameBeforeAnchor,
        });
    }
    if let Some(first) = evidence.first_after_anchor {
        return Ok(SequenceSeed {
            topic: topic.to_string(),
            seed: first,
            rung: SeedRung::FirstFrameAfterAnchor,
        });
    }
    if let Some(commit) = evidence.captured_last_commit {
        return Ok(SequenceSeed {
            topic: topic.to_string(),
            seed: commit.wrapping_add(1),
            rung: SeedRung::CapturedCommit,
        });
    }
    Err(RestoreRefusal::SequenceSeedUnavailable {
        topic: topic.to_string(),
    })
}

// ===========================================================================
// `--strict-state`
// ===========================================================================

/// Enforce `--strict-state`: every executed node must restore, or refuse.
///
/// # What the flag means
///
/// `--strict-state` does NOT govern shape drift. A per-field drift table, where
/// the default tolerates an added or removed field and a strict mode refuses
/// it, does not exist
/// here (see [`classify_shape`]): every SHAPE difference is already terminal,
/// so the flag would be a no-op if it governed drift.
///
/// The difference it DOES govern is coverage. By default a node that declares
/// no state at all — a stateless node, or one whose type has no
/// `CerulionState` derive — restores nothing and is executed from its
/// constructor, which is correct for the first and a coverage
/// gap for the second. `--strict-state` refuses
/// that second reading: it says "I want every executed node's state, and I
/// want to be told if the recording cannot give it to me" rather than
/// discovering it later in a divergence report.
///
/// `nodes_without_state` is the executed nodes whose entries declare no state.
pub fn enforce_strict_state(
    nodes_without_state: &[String],
    strict: bool,
) -> Result<(), RestoreRefusal> {
    if !strict || nodes_without_state.is_empty() {
        return Ok(());
    }
    Err(RestoreRefusal::StrictStateUnsatisfied {
        nodes: nodes_without_state.to_vec(),
    })
}
