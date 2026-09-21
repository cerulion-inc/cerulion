// SPDX-License-Identifier: AGPL-3.0-only
//! The DESK-SIDE H.264 path — a robot's raw compressed video renders
//! without any robot-side transcoding.
//!
//! # Why this module exists
//!
//! Decision: we support BOTH camera routes. The robot may transcode to
//! JPEG (the efficient path for a robot we control), and the desk must
//! ALSO be able to take the robot's native compressed stream untouched
//! (the GENERALITY path). A robot we have never seen publishes
//! whatever it publishes — very likely a raw H.264 stream and no transcoder —
//! and requiring a Cerulion transcoding node on it before its camera is
//! viewable is exactly the robot-side dependency the project's rule ("no viz on the
//! robot; the desk renders") exists to avoid.
//!
//! Rerun 0.34 ships [`rerun::VideoStream`] with
//! [`rerun::components::VideoCodec::H264`]: the VIEWER decodes. So this module
//! does not decode video. It answers two questions the viewer cannot:
//!
//! 1. **Is this topic carrying H.264?** — [`classify_h264_payload`].
//! 2. **Which sub-stream does this access unit belong to?** — [`VideoDemux`].
//!
//! # 1. Classification: CONTENT first, name only as a tie-breaker
//!
//! The generality bar forbids keying on a schema name: `unitree_go/Go2FrontVideoData`
//! is the instance we have, not the contract. So the gate is the BYTES. An access
//! unit in Annex-B framing is a start-code-delimited NAL sequence
//! ([`scan_annex_b`]), and that shape is structurally checkable without any
//! decoder: every NAL header must carry `forbidden_zero_bit == 0`, a
//! `nal_unit_type` inside the in-band range `1..=23`, and the `nal_ref_idc`
//! polarity the spec REQUIRES for parameter sets / IDR slices / SEI. A message
//! carrying such bytes in a `uint8[]` field is a video topic whatever it or its
//! fields are called.
//!
//! Field NAMES are an ACCELERATOR only ([`VIDEO_FIELD_HINTS`]): when a message
//! has several byte fields they decide which is examined FIRST, never whether a
//! field qualifies. A hinted field whose bytes are not Annex-B is rejected; an
//! unhinted field whose bytes are Annex-B is accepted.
//!
//! # 2. Demux: one child entity per rendition
//!
//! One topic can interleave SEVERAL encoder outputs. A robot can alternate a
//! 360p and a 720p stream on `/frontvideostream`, and a decoder fed alternating
//! resolutions breaks. [`VideoDemux`] therefore splits a topic into per-rendition
//! sub-streams keyed by the resolution its SPS declares, rendered under
//! `<topic entity>/`[`VIDEO_CHILD`]`/<WxH>` — one child for the common
//! single-rendition topic, N for an interleaved one.
//!
//! Attribution runs in this precedence:
//!
//! | # | Evidence | Used when |
//! |---|---|---|
//! | 1 | An **SPS in this access unit** ([`sps_dimensions`]) | every keyframe |
//! | 2 | A **learned sibling discriminator** — an integer field of the SAME message whose value was previously observed alongside an SPS | non-keyframes on an interleaved topic |
//! | 3½ | A **confirmed tag carrying an UNKNOWN value** ⇒ this unit belongs to a rendition that has not opened yet ⇒ WAIT | non-keyframes arriving before their own rendition's first SPS |
//! | 3 | The **only stream seen so far** | non-keyframes on a single-rendition topic |
//!
//! Rung 3½ sits ABOVE rung 3 and exists because rung 3 was a SILENT mis-route
//! without it: between rendition A's first SPS and rendition B's first SPS only A
//! exists, so every B unit fell through to "the only stream" and was fed to A's
//! decoder with no drop, no warn and no counter. A MISS on a tag we trust is
//! positive evidence, not an absence.
//!
//! **Trust is earned by REPETITION** ([`DISCRIMINATOR_CONFIRM_OBSERVATIONS`]): a
//! field speaks at rung 3½ only once the SAME value has been seen at two separate
//! parameter sets. A rendition tag repeats every keyframe and qualifies; a
//! per-frame counter never repeats and never does — which is the whole point,
//! since an unconfirmed field misses on essentially every unit and trusting that
//! would drop a healthy single-rendition topic's entire stream. The exact bound:
//! the hole closes one keyframe interval in, not instantly.
//!
//! Rung 2 is LEARNED, never hardcoded: whenever an access unit carries an SPS,
//! every integer sibling field's current value is recorded against the resolution
//! that SPS declared. A field that takes more than
//! [`MAX_DISCRIMINATOR_VALUES`] distinct values is a counter, not a rendition
//! tag, and is disqualified permanently; so is one that maps a single value to
//! two different resolutions. On the Go2 this converges on `video_height`
//! (360 → 640x360, 720 → 1280x720) and discards `time_frame`, with no
//! Go2-specific code.
//!
//! When rungs 1-3 all fail on a topic that has shown TWO OR MORE renditions, the
//! access unit is DROPPED and counted rather than guessed at: feeding it to the
//! wrong decoder corrupts that stream's picture, which is worse than a gap.
//!
//! # A resolution CHANGE, as distinct from an interleave
//!
//! A camera that switches resolution once is not interleaving. Its new units miss
//! on the confirmed tag, take rung 3½, and WAIT — dropped as "before keyframe",
//! not misdiagnosed as an ambiguity — until the new rendition's first parameter
//! set opens its own sub-stream, after which they flow normally. The OLD
//! sub-stream simply stops receiving samples; its entity keeps whatever it last
//! rendered, which is the faithful depiction of a stream that ended. Nothing is
//! retired eagerly, because a topic that switches back would then have to re-open
//! and re-wait for no benefit.
//!
//! # The IDR gate
//!
//! Rerun's H.264 contract requires a keyframe to carry its SPS, and a decoder
//! cannot start mid-GOP. The gate is STRUCTURAL: a sub-stream comes into
//! existence only from an access unit that carried its SPS, so an SPS-less unit
//! arriving before any parameter set has no sub-stream to belong to and is
//! dropped and counted ([`VideoDemux::dropped_before_keyframe`]).
//!
//! That drop is the ORDINARY attach transient — every mid-GOP subscriber sees a
//! short burst of it — so it is logged at DEBUG and is a strictly separate
//! [`VideoReject`] arm from the genuinely-ambiguous interleaved case, which
//! warns. Collapsing the two would fire an alarming (and false) "this topic
//! interleaves two or more renditions" warn on every healthy camera attach.
//!
//! # Scope (deliberately NOT implemented)
//!
//! * **H.265 / AV1 / VP8 / VP9.** Rerun has the codecs, but each needs its own
//!   parameter-set parser to answer the demux question, and no captured sample
//!   exists to test against (Principle #13).
//!
//!   **H.265 needed an explicit refusal, and the earlier claim that the H.264
//!   header rules already rejected it was FALSE.** HEVC wears the SAME Annex-B
//!   framing and its 2-byte header overlaps H.264's 1-byte one: an HEVC SPS
//!   (`42 01`) reads as a slice-data-partition-A and a TRAIL_R (`02 01`) as a
//!   non-reference slice, both of which pass every rule in
//!   [`scan_annex_b`]. An HEVC discriminator therefore detects the parameter-set /
//!   IRAP signature and refuses those units outright. An access unit containing
//!   ONLY HEVC inter-frames stays genuinely ambiguous: it classifies, opens no
//!   sub-stream (no parseable H.264 SPS will ever arrive), is dropped at the
//!   keyframe gate, and escalates to a loud warn via
//!   [`NEVER_KEYFRAMED_WARN_AFTER`]. So an HEVC topic renders nothing and SAYS so
//!   — it never feeds garbage to a decoder.
//! * **Fragmented / RTP-packetized NAL units** (FU-A, STAP-A). Those are a
//!   TRANSPORT framing that a Cerulion topic does not use: the payload here is a
//!   whole access unit in one message. RTP `nal_unit_type` 24..=31 is
//!   deliberately rejected, so an RTP payload is never mistaken for Annex-B.
//! * **AVCC / length-prefixed samples** (the MP4 in-band form). Distinguishing
//!   AVCC from arbitrary bytes needs the out-of-band `avcC` record, which a bare
//!   `uint8[]` field does not carry.
//! * **B-frames.** Rerun 0.34's `VideoStream` has no decode-timestamp component
//!   (rerun-io/rerun#10090), so a stream with B-frames plays back with the
//!   presentation order rerun infers. Nothing here can fix that.
//!
//! # Known residual: a parameter-set-only message becomes a picture-less sample
//!
//! An encoder that ships SPS/PPS in ONE message and the IDR in the NEXT produces
//! an access unit with no coded slice. [`scan_annex_b`] accepts it (an SPS is
//! enough to classify), so it opens the sub-stream and is forwarded as its own
//! `VideoStream` sample — which carries parameter sets and no picture, where
//! rerun's docs say a sample "must contain enough data for exactly one video
//! frame".
//!
//! It is forwarded rather than dropped ON PURPOSE: those bytes are the only copy
//! of the parameter sets, and withholding them leaves the stream undecodable —
//! strictly worse than one sample that yields no frame. Decoders fed Annex-B
//! normally receive parameter sets ahead of slices, so the expected cost is a
//! no-op sample. The stricter fix (retain the parameter sets and PREPEND them to
//! the next slice-carrying unit, so every emitted sample is exactly one frame) is
//! deliberately not taken here: it would trade the verbatim hand-off — which the
//! tests pin — for extra state, and the keyframe flag is already correct for this
//! shape (see [`VideoDemux::route`]).

use std::collections::BTreeMap;

use cerulion_core::codegen::{FrameValue, FrameValueKind};
use rerun::RecordingStream;

use crate::archetype::set_robot_time;

// ────────────────────────────────────────────────────────────────────────────
// Annex-B scanning (pure — no rerun, no decoder)
// ────────────────────────────────────────────────────────────────────────────

/// The lowest in-band `nal_unit_type`. `0` is "unspecified" and never appears in
/// a conforming Annex-B stream.
const NAL_TYPE_MIN: u8 = 1;
/// The highest in-band `nal_unit_type` (ITU-T H.264 Table 7-1). `24..=31` are
/// unspecified and are used by RTP packetization (STAP/FU), which is a transport
/// framing, not Annex-B — rejecting them is what keeps an RTP payload from being
/// classified as a renderable access unit.
const NAL_TYPE_MAX: u8 = 23;

/// `nal_unit_type` of a non-IDR coded slice.
const NAL_SLICE_NON_IDR: u8 = 1;
/// `nal_unit_type` of an IDR coded slice (a keyframe).
const NAL_SLICE_IDR: u8 = 5;
/// `nal_unit_type` of a Sequence Parameter Set — the NAL that carries the
/// picture dimensions, and the one rerun requires on a keyframe.
const NAL_SPS: u8 = 7;
/// `nal_unit_type` of a Picture Parameter Set.
const NAL_PPS: u8 = 8;
/// `nal_unit_type` of Supplemental Enhancement Information.
const NAL_SEI: u8 = 6;
/// `nal_unit_type` of an Access Unit Delimiter.
const NAL_AUD: u8 = 9;
/// `nal_unit_type` of End Of Sequence.
const NAL_END_OF_SEQ: u8 = 10;
/// `nal_unit_type` of End Of Stream.
const NAL_END_OF_STREAM: u8 = 11;
/// `nal_unit_type` of Filler Data.
const NAL_FILLER: u8 = 12;
/// `nal_unit_type` of a Sequence Parameter Set EXTENSION — a parameter set, so
/// it shares the SPS/PPS `nal_ref_idc` requirement.
const NAL_SPS_EXT: u8 = 13;
/// `nal_unit_type` of a SUBSET Sequence Parameter Set (SVC/MVC) — likewise a
/// parameter set for the `nal_ref_idc` rule.
const NAL_SUBSET_SPS: u8 = 15;

/// One NAL unit located inside an Annex-B buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NalUnit<'a> {
    /// `nal_unit_type` (bits 0..=4 of the header byte).
    pub kind: u8,
    /// `nal_ref_idc` (bits 5..=6 of the header byte).
    pub ref_idc: u8,
    /// The NAL's bytes INCLUDING its one-byte header, EXCLUDING the start code
    /// and any trailing zero padding. This is what
    /// [`h264_reader::nal::RefNal::new`] wants.
    pub bytes: &'a [u8],
}

/// A successfully scanned H.264 Annex-B access unit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AccessUnit<'a> {
    /// Every NAL unit in the buffer, in order.
    pub nals: Vec<NalUnit<'a>>,
}

impl<'a> AccessUnit<'a> {
    /// The FIRST Sequence Parameter Set in this access unit, if any. Its presence
    /// is what opens a sub-stream (see the module's IDR-gate section) and what
    /// feeds [`sps_dimensions`].
    pub fn sps(&self) -> Option<&NalUnit<'a>> {
        self.nals.iter().find(|n| n.kind == NAL_SPS)
    }

    /// Whether this access unit carries an IDR coded slice.
    ///
    /// On its own this is NOT rerun's keyframe condition — see
    /// [`AccessUnit::is_self_contained_keyframe`] for that, and
    /// [`VideoDemux::route`] for the STREAM-AWARE verdict actually reported to
    /// rerun.
    pub fn has_idr(&self) -> bool {
        self.nals.iter().any(|n| n.kind == NAL_SLICE_IDR)
    }

    /// Whether this access unit is a keyframe ON ITS OWN — an IDR coded slice
    /// that carries its own SPS, so a decoder with NO prior state can start here.
    ///
    /// This is rerun's codec contract read strictly ("Key frames (IDR) require
    /// inclusion of a SPS"), and it is the right predicate for a single access
    /// unit in isolation. It is deliberately NOT what the demux reports: a
    /// sub-stream is created only by an access unit whose SPS was parsed and fed,
    /// so by the time any later unit is routed the decoder at that entity ALREADY
    /// has the parameter sets — and an encoder that ships SPS/PPS in one message
    /// and the IDR in the next would otherwise produce a stream where NO sample
    /// is ever flagged a keyframe, leaving the viewer with no start point. See
    /// [`VideoDemux::route`].
    pub fn is_self_contained_keyframe(&self) -> bool {
        self.sps().is_some() && self.has_idr()
    }

    /// Whether this access unit carries a coded picture (a VCL NAL, types
    /// `1..=5`).
    pub fn has_coded_slice(&self) -> bool {
        self.nals
            .iter()
            .any(|n| (NAL_SLICE_NON_IDR..=NAL_SLICE_IDR).contains(&n.kind))
    }
}

/// Whether a NAL header byte is structurally valid H.264.
///
/// Three independent checks, each a hard requirement of ITU-T H.264 §7.4.1 — a
/// blob of arbitrary bytes fails them far more often than not, which is what
/// makes the content gate a classifier rather than a guess:
///
/// * `forbidden_zero_bit` (bit 7) MUST be 0.
/// * `nal_unit_type` MUST be in-band ([`NAL_TYPE_MIN`]..=[`NAL_TYPE_MAX`]).
/// * `nal_ref_idc` polarity, quoting §7.4.1's two hard rules EXACTLY — it "shall
///   not be equal to 0" for an IDR slice (type 5) and for a sequence parameter
///   set, sequence parameter set extension, subset sequence parameter set or
///   picture parameter set (types 7, 13, 15, 8), and "shall be equal to 0" for
///   types 6, 9, 10, 11 and 12 (SEI / AUD / end-of-sequence / end-of-stream /
///   filler).
///
/// **Coded slices (types 1-4) are deliberately unconstrained here, including the
/// data partitions.** §7.4.1 permits `nal_ref_idc == 0` on any of them — that is
/// precisely how a NON-REFERENCE picture is marked — and only requires the value
/// to be CONSISTENT across types 1-4 within one picture, which is a cross-NAL
/// invariant this per-header check cannot see. Demanding non-zero on type 2
/// (slice data partition A), on the theory that a partition is always
/// reference data, would be wrong: it would reject a legal data-partitioned
/// stream's non-reference partition outright. The cost of the
/// correction is one extra accepted header byte out of 256 — nothing measurable
/// against the start-code and whole-buffer requirements.
fn nal_header_is_valid(header: u8) -> bool {
    if header & 0x80 != 0 {
        return false;
    }
    let ref_idc = (header >> 5) & 0x03;
    let kind = header & 0x1F;
    if !(NAL_TYPE_MIN..=NAL_TYPE_MAX).contains(&kind) {
        return false;
    }
    match kind {
        // "shall not be equal to 0": IDR slice, and every parameter-set flavour.
        NAL_SLICE_IDR | NAL_SPS | NAL_PPS | NAL_SPS_EXT | NAL_SUBSET_SPS => ref_idc != 0,
        // "shall be equal to 0": never-referenced data.
        NAL_SEI | NAL_AUD | NAL_END_OF_SEQ | NAL_END_OF_STREAM | NAL_FILLER => ref_idc == 0,
        _ => true,
    }
}

/// Whether this NAL sequence carries an unmistakable H.265 (HEVC) signature.
///
/// HEVC uses the SAME Annex-B byte-stream framing, so [`scan_annex_b`]'s
/// start-code walk is codec-agnostic and only the HEADER rules separate the two —
/// and they overlap. HEVC's header is TWO bytes: `forbidden_zero_bit(1)`,
/// `nal_unit_type(6)`, `nuh_layer_id(6)`, `nuh_temporal_id_plus1(3)`, the last of
/// which is required non-zero. Read as H.264, an HEVC SPS (`42 01`) is a
/// slice-data-partition-A and an HEVC TRAIL_R (`02 01`) a non-reference slice —
/// both legal, so the H.264 rules alone accept them.
///
/// The discriminator used here is the PARAMETER SETS and IRAP pictures, because a
/// decodable HEVC stream MUST carry them periodically: HEVC types 32/33/34
/// (VPS/SPS/PPS) and 16..=23 (the IRAP range) with a valid second header byte.
///
/// **Declared residual:** an access unit containing ONLY HEVC inter-frames is
/// genuinely ambiguous with H.264 data partitions and is NOT caught here. It
/// classifies as video, opens no sub-stream (no parseable H.264 SPS will ever
/// arrive), and is dropped at the keyframe gate — which escalates to a loud warn
/// via [`NEVER_KEYFRAMED_WARN_AFTER`]. So an HEVC topic renders nothing and SAYS
/// so; it never feeds garbage to a decoder.
fn looks_like_hevc(nals: &[NalUnit<'_>]) -> bool {
    nals.iter().any(|n| {
        // Needs the 2-byte header plus at least one payload byte.
        if n.bytes.len() < 3 {
            return false;
        }
        let hevc_type = (n.bytes[0] >> 1) & 0x3F;
        // `nuh_temporal_id_plus1` shall not be 0.
        let tid_plus1 = n.bytes[1] & 0x07;
        // `nuh_layer_id` is 0 for the base layer — every single-layer stream.
        let layer_id = ((n.bytes[0] & 0x01) << 5) | (n.bytes[1] >> 3);
        let is_param_set = (32..=34).contains(&hevc_type);
        let is_irap = (16..=23).contains(&hevc_type);
        tid_plus1 != 0 && layer_id == 0 && (is_param_set || is_irap)
    })
}

/// Index just past a 3- or 4-byte Annex-B start code beginning at `at`, or
/// `None` when `bytes[at..]` does not open with one.
fn start_code_len(bytes: &[u8], at: usize) -> Option<usize> {
    if bytes.len() >= at + 4 && bytes[at..at + 4] == [0, 0, 0, 1] {
        Some(4)
    } else if bytes.len() >= at + 3 && bytes[at..at + 3] == [0, 0, 1] {
        Some(3)
    } else {
        None
    }
}

/// Parse `bytes` as ONE H.264 Annex-B access unit, or `None` when it is not one.
///
/// This is the CONTENT GATE the classifier runs (see the module docs): the
/// buffer must OPEN with a start code and every start-code-delimited NAL in it
/// must carry a structurally valid header (`nal_header_is_valid` — the
/// forbidden-zero-bit, in-band-type and ref-idc-polarity rules), and the whole
/// must contain a coded slice or an SPS. A single stray `00 00 01` sequence
/// inside an unrelated binary blob is not enough — every subsequent unit has to
/// check out too.
///
/// Requiring the start code at offset 0 is deliberate. A Cerulion topic carries
/// ONE access unit per message, so a conforming payload always begins with one;
/// scanning for a start code anywhere in the buffer would turn any sufficiently
/// large blob into a candidate.
pub fn scan_annex_b(bytes: &[u8]) -> Option<AccessUnit<'_>> {
    // Annex-B permits any number of `leading_zero_8bits` before the FIRST start
    // code (§B.1.1), and a conforming writer may emit them — refusing that shape
    // would fail a legal stream, which the generality bar does not allow. Only
    // ZERO bytes are skipped, so this is not the "scan for a start code anywhere"
    // relaxation: a buried start code after ANY non-zero byte is still refused.
    let mut lead = 0usize;
    while bytes.get(lead) == Some(&0) && start_code_len(bytes, lead).is_none() {
        lead += 1;
    }
    let mut pos = lead;
    let first = start_code_len(bytes, pos)?;
    pos += first;

    let mut nals: Vec<NalUnit<'_>> = Vec::new();
    while pos < bytes.len() {
        // Find the next start code; the current NAL runs up to it.
        let mut end = bytes.len();
        let mut next = None;
        let mut probe = pos;
        while probe + 3 <= bytes.len() {
            if bytes[probe] == 0 && bytes[probe + 1] == 0 && bytes[probe + 2] == 1 {
                // A 4-byte start code is `00 00 00 01`; the preceding zero (and
                // any further `trailing_zero_8bits`) is padding that belongs to
                // NEITHER NAL, so trim it off this NAL's tail.
                //
                // This trim is deliberately allowed to be over-eager, because it
                // cannot reach anything that matters. A slice RBSP may legally
                // END in zeros (`cabac_zero_word`), so the trimmed view can be
                // shorter than the true NAL — but the trimmed view is used for
                // exactly two things, header inspection and SPS parsing, and
                // neither is affected: an SPS carries no `cabac_zero_word` (it is
                // slice-only), and a header lives at offset 0. What goes to the
                // decoder is `H264Payload::bytes`, the ORIGINAL buffer verbatim,
                // never a re-assembly of these views — so no byte a decoder needs
                // can be lost here.
                let mut cut = probe;
                while cut > pos && bytes[cut - 1] == 0 {
                    cut -= 1;
                }
                end = cut;
                next = Some(probe + 3);
                break;
            }
            probe += 1;
        }

        let nal = &bytes[pos..end];
        if nal.is_empty() || !nal_header_is_valid(nal[0]) {
            return None;
        }
        // A NAL is normally its header plus at least one payload byte — EXCEPT
        // for end-of-sequence and end-of-stream, whose RBSPs are defined EMPTY
        // (§7.3.2.5 / §7.3.2.6), so a conforming one is exactly its single header
        // byte. A flat `len() < 2` rejected those, and because a bad NAL fails the
        // WHOLE buffer, an encoder that appends either at a GOP or stream boundary
        // lost that entire message to the text dump.
        let min_len = match nal[0] & 0x1F {
            NAL_END_OF_SEQ | NAL_END_OF_STREAM => 1,
            _ => 2,
        };
        if nal.len() < min_len {
            return None;
        }
        nals.push(NalUnit {
            kind: nal[0] & 0x1F,
            ref_idc: (nal[0] >> 5) & 0x03,
            bytes: nal,
        });

        match next {
            Some(n) => pos = n,
            None => break,
        }
    }

    if nals.is_empty() {
        return None;
    }
    // H.265 wears the SAME Annex-B framing, and its 2-byte NAL header collides
    // with H.264's 1-byte one often enough to matter: an HEVC SPS (`42 01`) reads
    // as an H.264 slice-data-partition-A and an HEVC TRAIL_R (`02 01`) as a
    // non-reference one — both pass every rule above. Feeding those to an H.264
    // decoder cannot work, so detect the signature and refuse.
    if looks_like_hevc(&nals) {
        return None;
    }
    let au = AccessUnit { nals };
    // Parameter sets alone are not a renderable sample, and a lone SEI/AUD blob
    // is far more likely to be a coincidence than a video frame. Require the
    // buffer to carry a picture or the parameter set that configures one.
    if !au.has_coded_slice() && au.sps().is_none() {
        return None;
    }
    Some(au)
}

/// Decode a Sequence Parameter Set NAL into `(width, height)` in PIXELS, honouring
/// the SPS's own frame cropping (so a 1280x720-cropped-from-1280x736 stream
/// reports 720, not 736).
///
/// `sps_nal` is the NAL INCLUDING its header byte and still carrying
/// emulation-prevention bytes — exactly [`NalUnit::bytes`]. `h264_reader` strips
/// the emulation prevention while reading the RBSP.
///
/// `None` when the bytes are not a parseable SPS: the caller treats that as "no
/// dimension evidence in this access unit", never as a decode failure worth
/// surfacing per frame.
pub fn sps_dimensions(sps_nal: &[u8]) -> Option<(u32, u32)> {
    use h264_reader::nal::{sps::SeqParameterSet, Nal as _, RefNal};
    let nal = RefNal::new(sps_nal, &[], true);
    let sps = SeqParameterSet::from_bits(nal.rbsp_bits()).ok()?;
    sps.pixel_dimensions().ok()
}

// ────────────────────────────────────────────────────────────────────────────
// Classification: which byte field (if any) carries the access unit
// ────────────────────────────────────────────────────────────────────────────

/// Field-name hints, most specific first — an ACCELERATOR for choosing WHICH
/// byte field of a multi-blob message to examine first, never a requirement.
///
/// The gate is always [`scan_annex_b`] over the field's CONTENT: a field named
/// `video_data` holding JPEG is rejected, and a field named `blob` holding
/// Annex-B is accepted. Ordering matters only when a message carries two byte
/// fields that BOTH scan as Annex-B, which no real message does — and even then
/// the choice is deterministic rather than dependent on declaration order.
pub const VIDEO_FIELD_HINTS: &[&str] = &[
    "video_data",
    "video",
    "h264",
    "avc",
    "nal",
    "nalu",
    "access_unit",
    "au",
    "frame_data",
    "encoded_data",
    "stream",
    "data",
    "payload",
    "buffer",
    "bytes",
];

/// A message field identified as carrying an H.264 Annex-B access unit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct H264Payload<'a> {
    /// The field the access unit came from (for diagnostics + the demux's
    /// discriminator exclusion).
    pub field: String,
    /// The raw access-unit bytes, VERBATIM — this is what is handed to
    /// [`rerun::VideoStream`], with no re-framing.
    pub bytes: &'a [u8],
    /// The scanned NAL structure.
    pub access_unit: AccessUnit<'a>,
}

/// Hint rank of a field name — lower sorts first; unhinted fields sort last.
fn hint_rank(name: &str) -> usize {
    VIDEO_FIELD_HINTS
        .iter()
        .position(|h| *h == name)
        .unwrap_or(VIDEO_FIELD_HINTS.len())
}

/// THE CLASSIFIER: does this decoded frame carry an H.264 Annex-B access unit?
///
/// Byte-array fields are examined in (hint rank, declaration order) order and the
/// first whose CONTENT scans as Annex-B wins. Nothing about the schema NAME is
/// consulted, so a never-seen robot's camera topic classifies on the same
/// evidence the Go2's does.
pub fn classify_h264_payload<'a>(fv: &FrameValue<'a>) -> Option<H264Payload<'a>> {
    let mut candidates: Vec<(usize, usize, &str, &'a [u8])> = fv
        .fields
        .iter()
        .enumerate()
        .filter_map(|(i, f)| match &f.value {
            FrameValueKind::Bytes(b) => Some((hint_rank(f.name.as_str()), i, f.name.as_str(), *b)),
            _ => None,
        })
        .collect();
    candidates.sort_by_key(|(rank, idx, _, _)| (*rank, *idx));

    for (_, _, name, bytes) in candidates {
        if let Some(access_unit) = scan_annex_b(bytes) {
            return Some(H264Payload {
                field: name.to_string(),
                bytes,
                access_unit,
            });
        }
    }
    None
}

// ────────────────────────────────────────────────────────────────────────────
// Demux
// ────────────────────────────────────────────────────────────────────────────

/// The SYNTHETIC child segment every decoded video sub-stream renders under
/// (`<entity>/viz-video/<WxH>`).
///
/// **The `-` is load-bearing** — the same argument as
/// [`crate::archetype::PATH_VERTICES_CHILD`] and [`crate::sink::SWEEP_CHILD`]: a
/// synthesized child shares a namespace with real topics, so a robot publishing
/// both `/camera` and `/camera/video/640x360` would collide on it.
/// `crate::tf::sanitize_segment` emits only `[A-Za-z0-9_]`, so no topic name can
/// ever reach a segment containing `-`.
pub const VIDEO_CHILD: &str = "viz-video";

/// How many DISTINCT values a sibling integer field may take before it is
/// disqualified as a rendition discriminator.
///
/// A rendition tag takes as many values as there are renditions (two, on the
/// Go2). A frame counter or a timestamp takes a new value every message, so
/// without this cap its learned map would grow without bound. Four is comfortably
/// above any plausible rendition count on one topic, and a counter trips it on
/// its FIFTH distinct value (the check is `len() > MAX`).
///
/// The cap bounds MEMORY; it is not what keeps a counter from mis-routing. A
/// counter can hit on a repeated value before it is disqualified, so correctness
/// rests on the attribution step requiring every live discriminator to AGREE (see
/// the module docs, rung 2).
pub const MAX_DISCRIMINATOR_VALUES: usize = 4;

/// How many SEPARATE parameter-set observations must AGREE on one value before
/// that field is trusted as a REAL rendition tag (see [`VideoDemux::route`]'s
/// rung 3½).
///
/// A rendition tag REPEATS — `video_height` reads 360 on every 360p keyframe — so
/// two observations of the same value is cheap evidence that the field names a
/// rendition rather than an instant. A frame counter or timestamp NEVER repeats,
/// so it can never reach this bar, which is exactly the property the confirmation
/// rule needs: only a confirmed field is allowed to say "this unit belongs to a
/// rendition I have not opened yet", and a counter saying that on every P-frame
/// would drop a healthy single-rendition topic's entire stream.
pub const DISCRIMINATOR_CONFIRM_OBSERVATIONS: u32 = 2;

/// How many sub-streams ONE input may track before further renditions are refused.
///
/// The same reasoning as [`MAX_DISCRIMINATOR_VALUES`], applied to the other
/// unbounded map: `streams` is keyed by SPS-declared resolution, and while a real
/// camera publishes one or two, a malformed or hostile producer whose SPS parses
/// to a fresh size every keyframe would grow it without bound. Eight is far above
/// any plausible rendition count (the Go2 uses two) and is a memory bound, not a
/// behavioural one — reaching it is loud, once.
pub const MAX_STREAMS_PER_INPUT: usize = 8;

/// How many access units may be dropped waiting for a FIRST keyframe before the
/// wait is escalated from a debug transient to a loud, once-per-input warn.
///
/// A mid-GOP attach drops a short burst — one keyframe interval, typically a few
/// dozen units at most — and that is normal. A topic that has dropped this many
/// with NO sub-stream ever opened will not render, and the operator has to be
/// told rather than left watching an empty pane.
pub const NEVER_KEYFRAMED_WARN_AFTER: u64 = 300;

/// A sub-stream identity: the picture size its SPS declares.
///
/// Resolution is the discriminator because it is the property a DECODER cannot
/// tolerate changing mid-stream — which is the whole reason the demux exists. Two
/// renditions at the SAME resolution but different bitrates would share a key and
/// be interleaved into one decoder; nothing in an Annex-B access unit
/// distinguishes them, so that case is out of reach here rather than guessed at.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct StreamKey {
    /// Picture width in pixels.
    pub width: u32,
    /// Picture height in pixels.
    pub height: u32,
}

impl StreamKey {
    /// The entity-path segment for this sub-stream (`"640x360"`).
    pub fn segment(&self) -> String {
        format!("{}x{}", self.width, self.height)
    }
}

/// Why an access unit was NOT fed to a sub-stream. Every rejection is countable
/// and nameable — nothing is silently discarded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VideoReject {
    /// The topic has shown two or more renditions and this access unit carried
    /// neither an SPS nor a recognised discriminator value, so attributing it
    /// would be a guess (and a wrong guess corrupts a decoder). Genuinely
    /// actionable — warned once per input.
    Unattributable,
    /// Nothing on this topic has carried a parameter set since the tap opened, so
    /// no decoder could start on this unit. The ORDINARY attach transient (a
    /// mid-GOP subscriber waiting for the next keyframe), not a fault — logged at
    /// debug, never warned.
    BeforeKeyframe,
}

/// What [`VideoDemux::route`] decided for one access unit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum VideoRoute {
    /// Feed it to this sub-stream.
    Feed {
        /// The sub-stream it belongs to.
        key: StreamKey,
        /// Whether it is a decoder-startable keyframe (IDR + SPS).
        is_keyframe: bool,
        /// `true` on the FIRST sample of this sub-stream — the once-per-rendition
        /// "stream opened" breadcrumb, and where its decoder is created.
        first_sample: bool,
    },
    /// Drop it, for this reason.
    Drop(VideoReject),
}

/// A learned sibling-field → rendition mapping (see the module docs, rung 2).
#[derive(Debug, Default, Clone)]
struct Discriminator {
    /// Observed value → (the resolution an SPS declared alongside it, how many
    /// SEPARATE parameter-set observations have agreed on it).
    ///
    /// The count is what separates a real rendition TAG from a per-frame counter:
    /// a tag repeats its value on every keyframe of that rendition, a counter
    /// never repeats. See [`DISCRIMINATOR_CONFIRM_OBSERVATIONS`].
    seen: BTreeMap<i128, (StreamKey, u32)>,
    /// Set once this field proved to be a counter (too many distinct values) or
    /// self-contradictory (one value, two resolutions). Never cleared: a field
    /// that has behaved like a counter once cannot be trusted later, and the
    /// latch is what bounds `seen`.
    disqualified: bool,
}

impl Discriminator {
    /// Whether ANY value of this field has been observed alongside a parameter
    /// set at least [`DISCRIMINATOR_CONFIRM_OBSERVATIONS`] times — i.e. whether it
    /// REPEATS, which is what distinguishes a rendition tag from an instant.
    ///
    /// Only a confirmed field may declare an unopened sub-stream (see
    /// `InputState::attribute`'s rung 3½).
    fn is_confirmed(&self) -> bool {
        !self.disqualified
            && self
                .seen
                .values()
                .any(|(_, n)| *n >= DISCRIMINATOR_CONFIRM_OBSERVATIONS)
    }
}

/// Per-sub-stream state.
///
/// A sub-stream EXISTS only because an access unit carrying its SPS created it
/// (see [`VideoDemux::route`]), so "has this stream been opened by a keyframe?"
/// needs no field — the entry's presence IS the answer.
#[derive(Debug, Default, Clone)]
struct StreamState {
    /// Samples fed to this sub-stream.
    samples: u64,
    /// Whether this sub-stream has been ANNOUNCED (the once-per-rendition
    /// "H.264 stream opened" breadcrumb, and the point at which its decoder is
    /// created).
    ///
    /// The desk-side decoder narrowed this bit. It used to track whether the STATIC
    /// `VideoStream:codec` declaration had reached the CURRENT viewer, which made
    /// it viewer-scoped state that a gRPC reconnect had to re-arm — a fresh server
    /// holds no declaration, and rerun cannot decode H.264 without one. Now that
    /// the desk decodes and logs self-contained `Image`s, nothing the viewer holds
    /// gates rendering, so this is a property of the STREAM alone and survives a
    /// reconnect like the rest of the demux's learned knowledge.
    announced: bool,
}

/// Per-input demux state.
#[derive(Debug, Default, Clone)]
struct InputState {
    /// Candidate rendition discriminators, keyed by field name.
    discriminators: BTreeMap<String, Discriminator>,
    /// Sub-streams seen on this input.
    streams: BTreeMap<StreamKey, StreamState>,
    /// Access units dropped because no sub-stream could be attributed.
    unattributable: u64,
    /// Access units dropped because their sub-stream had not yet been opened by
    /// an SPS.
    before_keyframe: u64,
    /// Warn-once latch for the unattributable regime.
    warned_unattributable: bool,
    /// Access units whose SPS was present but UNPARSEABLE.
    sps_parse_failures: u64,
    /// Warn-once latch for the unparseable-SPS regime.
    warned_sps_parse: bool,
    /// Warn-once latch for the per-input rendition cap.
    warned_stream_cap: bool,
    /// Whether the never-any-keyframe escalation has already fired.
    warned_never_keyframed: bool,
}

/// The per-input H.264 demux — the stateful half of this module, owned by
/// [`crate::sink::SinkState`].
///
/// Deterministic by construction: every decision is a function of the frames seen
/// so far in order (no clock, no hashing iteration order — the maps are
/// `BTreeMap`s), so an identical frame sequence produces identical entity
/// assignments on replay.
#[derive(Debug, Default, Clone)]
pub struct VideoDemux {
    inputs: BTreeMap<String, InputState>,
}

impl VideoDemux {
    /// A fresh demux that has seen nothing.
    pub fn new() -> Self {
        Self::default()
    }

    /// Decide where one access unit goes, LEARNING from it on the way (see the
    /// module docs for the precedence).
    ///
    /// `fv` is the whole decoded message — the demux reads its integer siblings
    /// to learn the rendition discriminator; `payload` is the classified video
    /// field, whose own name is excluded from the sibling scan.
    pub fn route(&mut self, input: &str, fv: &FrameValue, payload: &H264Payload) -> VideoRoute {
        let state = self.inputs.entry(input.to_string()).or_default();
        let sps_dims = payload
            .access_unit
            .sps()
            .and_then(|n| sps_dimensions(n.bytes));

        // An access unit carrying an SPS we could NOT parse is the one shape that
        // is genuinely un-renderable and used to say so only at debug, under a
        // reason ("waiting for a keyframe") that was not the truth. Count it and
        // name it, once per input.
        if sps_dims.is_none() && payload.access_unit.sps().is_some() {
            state.sps_parse_failures += 1;
            if !state.warned_sps_parse {
                state.warned_sps_parse = true;
                let profile_idc = payload
                    .access_unit
                    .sps()
                    .and_then(|n| n.bytes.get(1).copied());
                tracing::warn!(
                    profile_idc = ?profile_idc,
                    "cerulion_viz: this topic's H.264 parameter set could not be parsed, so its \
                     picture size is unknown and no sub-stream can be opened — every access unit \
                     is being dropped. A profile this build's SPS reader does not cover is the \
                     usual cause."
                );
            }
        }

        let key = match sps_dims {
            Some((width, height)) => {
                let key = StreamKey { width, height };
                state.learn(fv, &payload.field, key);
                // Bound the per-input rendition set (see `MAX_STREAMS_PER_INPUT`).
                // A NEW key past the cap is refused; an EXISTING one still routes,
                // so a healthy two-rendition topic is untouched.
                if !state.streams.contains_key(&key) && state.streams.len() >= MAX_STREAMS_PER_INPUT
                {
                    state.unattributable += 1;
                    if !state.warned_stream_cap {
                        state.warned_stream_cap = true;
                        tracing::warn!(
                            streams = state.streams.len(),
                            width,
                            height,
                            "cerulion_viz: this topic has declared more distinct H.264 resolutions \
                             than the per-topic cap allows; further renditions are DROPPED. A \
                             producer whose parameter sets decode to a new size every keyframe is \
                             the usual cause."
                        );
                    }
                    return VideoRoute::Drop(VideoReject::Unattributable);
                }
                key
            }
            None => match state.attribute(fv, &payload.field) {
                Attribution::Key(key) => key,
                Attribution::WaitingForKeyframe => {
                    state.before_keyframe += 1;
                    return VideoRoute::Drop(VideoReject::BeforeKeyframe);
                }
                Attribution::Ambiguous => {
                    state.unattributable += 1;
                    return VideoRoute::Drop(VideoReject::Unattributable);
                }
            },
        };

        // THE KEYFRAME GATE, enforced STRUCTURALLY rather than by a flag: a
        // sub-stream enters `streams` only on the branch above that parsed an SPS
        // out of THIS access unit, and every SPS-less unit is attributed to a key
        // that is already there. So a stream cannot exist un-opened, and there is
        // no reachable state in which an un-startable unit is fed. (An explicit
        // `opened: bool` used to sit here and was DEAD CODE — no input could make
        // it false, so nothing tested it and it protected nothing; a mutation that
        // deleted the check survived the whole suite. Deleting the flag moved the
        // guarantee into the shape of the data, and splitting the reject reason
        // put the real behaviour — a mid-GOP attach is a debug transient, not an
        // ambiguity — under test.)
        // STREAM-AWARE keyframe verdict, not the per-unit one. A sub-stream exists
        // only because an access unit carrying its SPS was parsed AND fed to this
        // entity, so the decoder there already holds the parameter sets and ANY
        // IDR routed here is a genuine start point. Reporting the strict per-unit
        // predicate instead would leave an encoder that ships SPS/PPS and the IDR
        // as SEPARATE messages with a stream in which no sample is EVER flagged a
        // keyframe — the viewer would have bytes and no place to start.
        let is_keyframe = payload.access_unit.has_idr();
        let stream = state.streams.entry(key).or_default();
        let first_sample = !stream.announced;
        stream.announced = true;
        stream.samples += 1;
        // RE-ARM the unattributable latch, matching the house contract every other
        // flood latch here follows (`DrainWarnLatch`, `OutputDiscardLatch`): a
        // successful feed ENDS the regime, so a LATER regime is loud again instead
        // of silent for the rest of the run.
        state.warned_unattributable = false;
        VideoRoute::Feed {
            key,
            is_keyframe,
            first_sample,
        }
    }

    /// Sub-streams currently known for `input`, in resolution order (test /
    /// observability seam).
    pub fn streams(&self, input: &str) -> Vec<StreamKey> {
        self.inputs
            .get(input)
            .map(|s| s.streams.keys().copied().collect())
            .unwrap_or_default()
    }

    /// Samples FED to `(input, key)` so far (test / observability seam).
    pub fn samples(&self, input: &str, key: StreamKey) -> u64 {
        self.inputs
            .get(input)
            .and_then(|s| s.streams.get(&key))
            .map(|s| s.samples)
            .unwrap_or(0)
    }

    /// Access units dropped on `input` because their sub-stream had not yet been
    /// opened by an SPS (the IDR gate).
    pub fn dropped_before_keyframe(&self, input: &str) -> u64 {
        self.inputs
            .get(input)
            .map(|s| s.before_keyframe)
            .unwrap_or(0)
    }

    /// Access units dropped on `input` because no sub-stream could be attributed.
    pub fn dropped_unattributable(&self, input: &str) -> u64 {
        self.inputs
            .get(input)
            .map(|s| s.unattributable)
            .unwrap_or(0)
    }

    /// Every input's rendition SEGMENTS (`"640x360"`), keyed by input name, in
    /// resolution order — the snapshot the layout plane needs to give each
    /// rendition its OWN view instead of overlaying them in one.
    ///
    /// Segments rather than `StreamKey`s so the consumer (the daemon, across a
    /// thread boundary) can build `<entity>/viz-video/<segment>` without
    /// depending on this module's types. Inputs with no sub-stream yet are
    /// omitted: a topic that has never opened one has no renditions to place.
    pub fn rendition_segments(&self) -> BTreeMap<String, Vec<String>> {
        self.inputs
            .iter()
            .filter(|(_, s)| !s.streams.is_empty())
            .map(|(input, s)| {
                (
                    input.clone(),
                    s.streams.keys().map(|k| k.segment()).collect(),
                )
            })
            .collect()
    }

    /// Access units on `input` whose SPS was present but could NOT be parsed, so
    /// no sub-stream could be opened from them.
    pub fn sps_parse_failures(&self, input: &str) -> u64 {
        self.inputs
            .get(input)
            .map(|s| s.sps_parse_failures)
            .unwrap_or(0)
    }

    /// CLAIM the NEVER-KEYFRAMED escalation for `input`: `true` at most once, and
    /// only once enough units have gone by with NOT ONE sub-stream open.
    ///
    /// The ordinary mid-GOP attach transient is a short burst and stays at debug.
    /// A topic that has dropped [`NEVER_KEYFRAMED_WARN_AFTER`] units with nothing
    /// ever opened is not a transient — it is a stream that will never render, and
    /// staying quiet there is the same "silently shows nothing" failure the
    /// loud-not-silent rule exists to prevent.
    pub fn take_never_keyframed_warn(&mut self, input: &str) -> bool {
        let Some(state) = self.inputs.get_mut(input) else {
            return false;
        };
        if state.streams.is_empty()
            && state.before_keyframe >= NEVER_KEYFRAMED_WARN_AFTER
            && !state.warned_never_keyframed
        {
            state.warned_never_keyframed = true;
            return true;
        }
        false
    }

    /// CLAIM the unattributable warn-once latch for `input`: `true` exactly once
    /// per input, on the first drop that has something to report.
    ///
    /// This is the PRODUCTION latch driver — `crate::sink::render_video_sample`
    /// calls it to decide whether to emit the loud warn — and it is `&mut self`
    /// for that reason, not because it is test-only.
    pub fn take_unattributable_warn(&mut self, input: &str) -> bool {
        let Some(state) = self.inputs.get_mut(input) else {
            return false;
        };
        if state.unattributable > 0 && !state.warned_unattributable {
            state.warned_unattributable = true;
            return true;
        }
        false
    }
}

impl InputState {
    /// Record every integer sibling's current value against the resolution an SPS
    /// just declared, disqualifying fields that behave like counters or
    /// contradict themselves.
    fn learn(&mut self, fv: &FrameValue, video_field: &str, key: StreamKey) {
        for (name, value) in integer_siblings(fv, video_field) {
            let d = self.discriminators.entry(name).or_default();
            if d.disqualified {
                continue;
            }
            match d.seen.get_mut(&value) {
                Some((prev, _)) if *prev != key => {
                    // One value, two resolutions — this field does not identify a
                    // rendition.
                    d.disqualified = true;
                    d.seen.clear();
                }
                Some((_, observations)) => {
                    // The SAME value agreeing again is what CONFIRMS a tag: a
                    // counter never gets here.
                    *observations = observations.saturating_add(1);
                }
                None => {
                    d.seen.insert(value, (key, 1));
                    if d.seen.len() > MAX_DISCRIMINATOR_VALUES {
                        // A counter, not a tag.
                        d.disqualified = true;
                        d.seen.clear();
                    }
                }
            }
        }
    }

    /// Attribute an SPS-less access unit: a learned discriminator hit, else the
    /// single known sub-stream, else nothing.
    fn attribute(&self, fv: &FrameValue, video_field: &str) -> Attribution {
        // No sub-stream exists yet, so nothing on this topic has carried a
        // parameter set since the tap opened. That is the ORDINARY attach
        // transient — a mid-GOP subscriber — not an ambiguity, and conflating the
        // two is what would make a perfectly healthy single-rendition camera log
        // the loud "interleaves two or more renditions" warn on every attach.
        if self.streams.is_empty() {
            return Attribution::WaitingForKeyframe;
        }
        // EVERY live discriminator must AGREE. Taking the first one that hits —
        // in declaration order — silently routes a frame to the wrong decoder as
        // soon as a NEAR-discriminator outranks the real tag positionally, and
        // that shape is ordinary, not contrived: two renditions of one capture
        // normally share a frame counter, so on the Go2's own field order
        // (`time_frame` before `video_height`) `time_frame` legitimately learns
        // `1000 → 360p, 1001 → 720p` — two values, two resolutions, never
        // self-contradictory, so nothing disqualifies it — and the next 360p
        // P-frame stamped 1001 was fed to the 720p decoder with no drop, no warn
        // and `unattributable == 0`. That is exactly the corruption the module
        // docs call worse than a gap, arrived at silently.
        //
        // Disagreement is therefore a GUESS, and a guess is refused.
        let mut verdict: Option<StreamKey> = None;
        // A MISS on a CONFIRMED tag is positive evidence, not an absence.
        let mut missed_confirmed = false;
        for (name, value) in integer_siblings(fv, video_field) {
            let Some(d) = self.discriminators.get(&name) else {
                continue;
            };
            if d.disqualified {
                continue;
            }
            let Some((key, _)) = d.seen.get(&value) else {
                if d.is_confirmed() {
                    missed_confirmed = true;
                }
                continue;
            };
            match verdict {
                None => verdict = Some(*key),
                Some(prev) if prev != *key => return Attribution::Ambiguous,
                Some(_) => {}
            }
        }
        if let Some(key) = verdict {
            return Attribution::Key(key);
        }
        // RUNG 3½ — the not-yet-opened rendition. A CONFIRMED tag carrying a value
        // it has never been taught means this unit belongs to a rendition whose
        // first parameter set has not arrived: the correct answer is "waiting for
        // ITS keyframe", NOT the single-stream fallback below.
        //
        // Without this, the window between rendition A's first SPS and rendition
        // B's first SPS is a SILENT mis-route: only A exists, so every B unit fell
        // through to `streams.len() == 1` and was fed to A's decoder with no drop,
        // no warn and no counter. The window is bounded by the OTHER rendition's
        // keyframe interval — nothing this code controls.
        //
        // CONFIRMATION is what makes the rule safe. An unconfirmed field (any
        // per-frame counter) misses on essentially every unit, so trusting an
        // unconfirmed miss would drop a healthy single-rendition topic's whole
        // stream. See `DISCRIMINATOR_CONFIRM_OBSERVATIONS`.
        if missed_confirmed {
            return Attribution::WaitingForKeyframe;
        }
        // A single-rendition topic — the overwhelmingly common case — has exactly
        // one sub-stream, and its SPS-less access units can only belong to it.
        // With two or more and no discriminator, silence is the correct answer.
        if self.streams.len() == 1 {
            if let Some(key) = self.streams.keys().next().copied() {
                return Attribution::Key(key);
            }
        }
        Attribution::Ambiguous
    }
}

/// What [`InputState::attribute`] could conclude about an SPS-less access unit.
///
/// The two failure arms are DELIBERATELY distinct, because they mean opposite
/// things to an operator: `WaitingForKeyframe` is a healthy camera's normal
/// start-up transient (debug), while `Ambiguous` is a real, actionable
/// degradation on a topic that is genuinely interleaving streams this build
/// cannot separate (warn).
enum Attribution {
    /// Feed it to this sub-stream.
    Key(StreamKey),
    /// No sub-stream exists yet — nothing has carried a parameter set since the
    /// tap opened, so no decoder could start on this unit anyway.
    WaitingForKeyframe,
    /// Two or more sub-streams exist and nothing in this message says which.
    Ambiguous,
}

/// Every TOP-LEVEL integer-valued field of `fv` except the video blob itself, in
/// declaration order, widened to `i128` so signed and unsigned values share one
/// key space.
///
/// Floats are excluded on purpose: an exact-equality key over a float is a
/// footgun, and no producer tags a rendition with one.
fn integer_siblings(fv: &FrameValue, video_field: &str) -> Vec<(String, i128)> {
    fv.fields
        .iter()
        .filter(|f| f.name != video_field)
        .filter_map(|f| {
            let v: i128 = match &f.value {
                FrameValueKind::I8(v) => (*v).into(),
                FrameValueKind::U8(v) => (*v).into(),
                FrameValueKind::I16(v) => (*v).into(),
                FrameValueKind::U16(v) => (*v).into(),
                FrameValueKind::I32(v) => (*v).into(),
                FrameValueKind::U32(v) => (*v).into(),
                FrameValueKind::I64(v) => (*v).into(),
                FrameValueKind::U64(v) => (*v).into(),
                _ => return None,
            };
            Some((f.name.to_string(), v))
        })
        .collect()
}

// ────────────────────────────────────────────────────────────────────────────
// Rendering
// ────────────────────────────────────────────────────────────────────────────

/// The entity one sub-stream renders under: `<entity>/`[`VIDEO_CHILD`]`/<WxH>`.
pub fn video_entity(entity: &str, key: StreamKey) -> String {
    format!("{entity}/{VIDEO_CHILD}/{}", key.segment())
}

/// FALLBACK ONLY: log the static per-stream codec declaration.
///
/// Reached only when no Cisco OpenH264 blob is available and this desk therefore
/// CANNOT decode (see [`crate::video_decode`]). In that regime the VIEWER decodes,
/// which is the original path's behaviour: laggy (a measured 567 ms of ffmpeg
/// frame-threading) but functional, and strictly better than a black pane.
///
/// rerun's `VideoStream` docs specify that every component except `sample` is
/// logged statically once per entity, and rerun's H.264 contract REQUIRES this
/// one: a viewer that never received it cannot decode at all.
pub fn log_video_codec(rec: &RecordingStream, entity: &str) {
    let archetype =
        rerun::VideoStream::update_fields().with_codec(rerun::components::VideoCodec::H264);
    if let Err(e) = rec.log_static(entity.to_string(), &archetype) {
        tracing::warn!(error = %e, entity, "Rerun: VideoStream codec log failed");
    }
}

/// FALLBACK ONLY: log ONE H.264 access unit as a video sample at
/// `entity`, for the viewer to decode. See [`log_video_codec`] for when this
/// regime is reached.
///
/// The bytes are handed over VERBATIM: rerun's H.264 contract is Annex-B, which
/// is exactly what the robot published, so there is no re-framing step and no
/// copy beyond the one `VideoSample` requires.
pub fn log_video_sample(
    rec: &RecordingStream,
    entity: &str,
    timestamp_ns: u64,
    sample: &[u8],
    is_keyframe: bool,
) {
    set_robot_time(rec, timestamp_ns);
    let archetype = rerun::VideoStream::update_fields()
        .with_sample(sample.to_vec())
        .with_is_keyframe(is_keyframe);
    if let Err(e) = rec.log(entity.to_string(), &archetype) {
        tracing::warn!(error = %e, entity, "Rerun: VideoStream sample log failed");
    }
}

impl VideoDemux {
    /// Forget which sub-streams have been ANNOUNCED, so the next sample of each
    /// re-announces — and, in the fallback regime, re-logs its STATIC
    /// `VideoStream:codec` declaration to the bounced (empty) server.
    ///
    /// Called after a live gRPC reconnect, beside
    /// [`crate::sink::SinkState::clear_rebroadcast_dedup`] and
    /// [`crate::stream::rearm_after_reconnect`].
    ///
    /// **It is load-bearing ONLY in the fallback regime**, and there it is
    /// critical: rerun's H.264 contract requires the codec component, a fresh
    /// server has none, and without this the latch would never re-fire — leaving
    /// every video topic dead for the rest of the daemon's life. When this desk is
    /// decoding, every logged `Image` is self-contained and the re-arm costs one
    /// repeated breadcrumb, which is why it is unconditional rather than
    /// mode-dependent: a bit that is only sometimes reset is a bit whose state
    /// nobody can reason about.
    ///
    /// Resets ONLY the what-the-viewer-has-seen half. The learned SPS knowledge —
    /// stream keys, discriminators, sample totals — is a property of the ROBOT's
    /// stream, not of any viewer, so it survives: dropping it would re-open the
    /// keyframe gate and stall every rendition until its next IDR.
    pub fn rearm_stream_announcements(&mut self) {
        for input in self.inputs.values_mut() {
            for stream in input.streams.values_mut() {
                stream.announced = false;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The REAL Sequence Parameter Set from a live Unitree Go2 `/frontvideostream`
    /// sample — bytes 0x18..0x32 of
    /// `examples/go2/nodes/dds_bridge/tests/fixtures/go2_frontvideostream_360p_idr.cdr`,
    /// which is itself a verbatim DDS capture taken off the robot with an rclpy
    /// `raw=True` subscription (see that fixture's provenance note in
    /// `frontvideostream_wire_test.rs`). Nothing here is synthesized.
    const GO2_REAL_SPS: &[u8] = &[
        0x67, 0x64, 0x10, 0x28, 0xac, 0x1b, 0x1a, 0xa0, 0xa0, 0x2f, 0xf9, 0x61, 0x00, 0x00, 0x03,
        0x00, 0x01, 0x00, 0x00, 0x03, 0x00, 0x3c, 0x8f, 0x08, 0x84, 0x6a,
    ];

    #[test]
    fn the_real_go2_sps_decodes_to_its_advertised_resolution() {
        // The robot's own `video_height` field reads 360 on this sample, so the
        // SPS parse is cross-checked against an INDEPENDENT fact from the same
        // capture rather than against itself.
        assert_eq!(sps_dimensions(GO2_REAL_SPS), Some((640, 360)));
    }

    #[test]
    fn nal_header_validity_is_the_documented_rule() {
        // forbidden_zero_bit set.
        assert!(!nal_header_is_valid(0xE7));
        // type 0 (unspecified) and type 24 (RTP STAP-A) are out of band.
        assert!(!nal_header_is_valid(0x60));
        assert!(!nal_header_is_valid(0x78));
        // SPS with ref_idc 0 violates the polarity rule.
        assert!(!nal_header_is_valid(0x07));
        // SEI with ref_idc != 0 violates it the other way.
        assert!(!nal_header_is_valid(0x26));
        // The other two parameter-set flavours carry the SAME requirement.
        assert!(!nal_header_is_valid(0x0D)); // SPS extension, ref_idc 0
        assert!(!nal_header_is_valid(0x0F)); // subset SPS, ref_idc 0
        assert!(nal_header_is_valid(0x6D));
        assert!(nal_header_is_valid(0x6F));
        // The real Go2 headers.
        assert!(nal_header_is_valid(0x67)); // SPS, ref_idc 3
        assert!(nal_header_is_valid(0x68)); // PPS, ref_idc 3
        assert!(nal_header_is_valid(0x65)); // IDR, ref_idc 3
        assert!(nal_header_is_valid(0x41)); // non-IDR, ref_idc 2
        assert!(nal_header_is_valid(0x06)); // SEI, ref_idc 0

        // CODED SLICES 1-4 ARE UNCONSTRAINED, ref_idc 0 INCLUDED. §7.4.1 marks a
        // NON-REFERENCE picture exactly that way, so demanding non-zero on any of
        // them rejects legal video. Type 2 (slice data partition A) is the one
        // that was wrong: a partition is not "always reference data".
        for kind in 1u8..=4 {
            assert!(
                nal_header_is_valid(kind),
                "type {kind} with ref_idc 0 is a legal non-reference slice"
            );
            assert!(
                nal_header_is_valid(0x60 | kind),
                "type {kind} with ref_idc 3 is a legal reference slice"
            );
        }
        // And it is reachable through the real gate, not just the predicate.
        assert!(
            scan_annex_b(&[0, 0, 0, 1, 0x02, 0x11, 0x22]).is_some(),
            "a non-reference data-partition-A access unit must classify"
        );
    }

    #[test]
    fn stream_key_segment_is_the_entity_shape() {
        let key = StreamKey {
            width: 1280,
            height: 720,
        };
        assert_eq!(key.segment(), "1280x720");
        assert_eq!(
            video_entity("world/camera", key),
            "world/camera/viz-video/1280x720"
        );
    }
}
