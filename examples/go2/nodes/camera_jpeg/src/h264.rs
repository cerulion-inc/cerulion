// SPDX-License-Identifier: AGPL-3.0-only
//! Pure H.264 access-unit admission gate for the camera node.
//!
//! Unsafe-free, std-only, GStreamer-free — every decision below is a pure
//! function of `(video_height, access_unit_bytes)` plus a two-bit latch, so the
//! whole policy is oracle-testable with hand-built byte fixtures (no decoder,
//! no camera, no transport). [`crate::capture`] composes it; nothing here
//! depends on anything above it.
//!
//! # Why a gate exists at all
//!
//! The Go2 publishes `/frontvideostream` as ONE topic carrying TWO INDEPENDENT
//! H.264 renditions — 640x360 and 1280x720, one of each per `time_frame` (see
//! `examples/go2/schemas/unitree_go/msg/Go2FrontVideoData.msg`, whose layout was
//! established from 60 live captures). They are separate encodes with separate
//! SPS/PPS and separate slice numbering. Feeding both into one decoder is not
//! "two streams at once", it is one corrupt stream: the resolution flips every
//! frame and the decoder is re-initialised (or errors) on every access unit.
//! So the node picks ONE rendition by `video_height` and drops the other.
//!
//! The second gate is the IDR gate. A decoder cannot start mid-GOP: without the
//! sequence parameter set it has no resolution, no profile and no reference
//! frames, so every access unit until the first IDR is either dropped
//! internally or produces an error. Relying on `h264parse` to swallow
//! them is an implicit dependency on an element's private policy, and
//! its complaints are indistinguishable from a genuinely broken stream. So the
//! node gates EXPLICITLY: nothing is pushed until an access unit carrying an SPS
//! (NAL type 7) arrives, which for this stream means an IDR access unit (the
//! `.msg` header records that an IDR access unit carries SPS + PPS + IDR). A
//! byte scan for one NAL type over a ~10 KB buffer is a few microseconds, and
//! it makes "we are still waiting for a keyframe" a first-class, COUNTED state
//! instead of a decoder warning nobody can attribute.
//!
//! # Flood discipline
//!
//! A camera that hands us the wrong rendition does it 15-30 times a second, so
//! every skip class is COUNTED unconditionally and logged at most ONCE per
//! regime, repeats demoted (the house `DrainWarnLatch` shape). The expected
//! sibling rendition is not even a warn — it is the normal case, so it is
//! counted silently and surfaced in the node's teardown summary.

// Pure by construction. (The forbid lives HERE, not at the crate root, because
// the `#[cerulion_node]` macro in lib.rs expands cdylib FFI entry points
// containing `unsafe`.)
#![forbid(unsafe_code)]

/// NAL unit type of a sequence parameter set (H.264 spec Table 7-1). Its
/// presence is what tells us a decoder can start on this access unit.
pub const NAL_TYPE_SPS: u8 = 7;

/// The frame heights the Go2 is KNOWN to publish on `/frontvideostream`
/// (SPS-confirmed 640x360 and 1280x720 — one of each per `time_frame`).
///
/// A height in this set that is not the configured target is the EXPECTED
/// sibling rendition: counted, never warned. A height OUTSIDE it is an
/// unrecognised stream, so it is warned once per regime —
/// silently dropping it would hide a firmware change behind a black screen.
pub const KNOWN_RENDITION_HEIGHTS: [u32; 2] = [360, 720];

/// Default rendition to decode: the 1280x720 one. The 640x360 sibling is the
/// low-bandwidth twin; 720 is what the demo shows.
pub const DEFAULT_TARGET_HEIGHT: u32 = 720;

/// Why [`resolve_target_height`] returned the height it did.
///
/// The DECISION is separated from the LOGGING on purpose: the resolution runs
/// inside the node's `init()`, which is reachable only on a build that has a
/// decoder (i.e. only under the `gstreamer` feature), so a test on a machine
/// without GStreamer can never observe the warns. Returning the note makes the
/// policy oracle-testable everywhere; the caller's job is the one `match` that
/// maps a note to its `tracing::warn!`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetHeightNote {
    /// The configured height is one the Go2 is known to publish — say nothing.
    Known,
    /// The configured value was `0`, which matches NO rendition (it is what an
    /// unset/garbage env var parses to), so [`DEFAULT_TARGET_HEIGHT`] was
    /// substituted. The caller WARNS.
    ZeroFallback,
    /// A non-zero height outside [`KNOWN_RENDITION_HEIGHTS`]. Honoured exactly
    /// as asked — the node will decode it if the robot really publishes it —
    /// but no known rendition has that height, so the caller WARNS rather than
    /// leaving the operator with a black screen and a climbing skip counter.
    UnknownHeight,
}

/// Resolve the configured `CAMERA_TARGET_HEIGHT` into the height the gate will
/// admit, plus what to say about it. Pure — see [`TargetHeightNote`].
pub fn resolve_target_height(configured: u32) -> (u32, TargetHeightNote) {
    if configured == 0 {
        // The default is itself a known height, so this substitution never
        // also warrants an UnknownHeight note.
        return (DEFAULT_TARGET_HEIGHT, TargetHeightNote::ZeroFallback);
    }
    if KNOWN_RENDITION_HEIGHTS.contains(&configured) {
        (configured, TargetHeightNote::Known)
    } else {
        (configured, TargetHeightNote::UnknownHeight)
    }
}

/// True iff `au` contains at least one NAL unit of type `nal_type`.
///
/// Walks Annex-B start codes. Both the 3-byte (`00 00 01`) and 4-byte
/// (`00 00 00 01`) forms end with the same three bytes, so scanning for
/// `00 00 01` finds both; the byte after it is the NAL header, whose low 5
/// bits are `nal_unit_type` and whose top bit (`forbidden_zero_bit`) MUST be 0
/// — a set bit means the byte is not a NAL header, so it is rejected rather
/// than masked into a plausible-looking type.
///
/// Emulation prevention (`00 00 03`) guarantees `00 00 01` cannot occur inside
/// a NAL payload, so no unescaping is needed to scan reliably.
pub fn contains_nal_type(au: &[u8], nal_type: u8) -> bool {
    let mut i = 0usize;
    // Need bytes at i, i+1, i+2 (the start code) and i+3 (the NAL header).
    while i + 3 < au.len() {
        if au[i] == 0 && au[i + 1] == 0 && au[i + 2] == 1 {
            let header = au[i + 3];
            if header & 0x80 == 0 && (header & 0x1F) == nal_type {
                return true;
            }
            // Skip the two positions a start code PROVABLY cannot begin at,
            // and no further. A start code at i+1 needs `au[i+2] == 0` (its
            // middle byte) and one at i+2 needs `au[i+2] == 0` (its first
            // byte) — and we just read `au[i+2] == 1`, so both are excluded.
            // Position i+3 is NOT excluded: that byte is the
            // NAL header we just rejected, and a header byte of `0x00` is a
            // legal value for this scanner to see (nal_unit_type 0,
            // "unspecified"), so `00 00 01 | 00 00 01 67` really does place a
            // second start code at i+3. Skipping to i+4 would step over it and
            // miss the SPS behind it — pinned by
            // `a_start_code_beginning_at_the_previous_nal_header_is_not_skipped`.
            i += 3;
        } else {
            i += 1;
        }
    }
    false
}

/// True iff `au` carries a sequence parameter set — i.e. a decoder can START
/// on this access unit.
pub fn contains_sps(au: &[u8]) -> bool {
    contains_nal_type(au, NAL_TYPE_SPS)
}

/// What [`AuGate::classify`] decided about one access unit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuVerdict {
    /// Feed it to the decoder.
    Push,
    /// Zero-length payload — nothing to decode (counted, latched warn).
    SkipEmpty,
    /// A KNOWN rendition that is not the configured target — the expected
    /// sibling. Counted, never warned (it is half of every frame pair).
    SkipOtherRendition,
    /// A height outside the known renditions (counted, latched warn).
    SkipUnknownRendition,
    /// The target rendition, but no SPS has been seen yet — a decoder cannot
    /// start mid-GOP (counted, latched warn).
    SkipWaitingForSps,
}

/// How the caller should LOG one occurrence of a counted class: the
/// flood-latch decision, carrying the running lifetime total for that class.
///
/// Named for the LATCH, not for the gate: [`FloodCounter`] is used by the
/// node's oversized-JPEG class too, which is a dropped OUTPUT rather than a
/// skipped input, so a `Skip`-prefixed name would misdescribe half its
/// callers.
///
/// `Quiet` is returned for a pushed access unit AND for the expected sibling
/// rendition — the latter is counted but never logged, because at 15-30 fps it
/// would be a line per frame forever.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FloodLog {
    /// Nothing to log.
    Quiet,
    /// First occurrence of this class in the current regime — log LOUDLY.
    First { total: u64 },
    /// A repeat within the same regime — demote to `debug!`.
    Repeat { total: u64 },
}

/// A classification plus its logging decision.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GateOutcome {
    /// What to do with the access unit.
    pub verdict: AuVerdict,
    /// How to log it (already flood-latched).
    pub log: FloodLog,
}

/// One counted, flood-latched loss class — the crate's flood-latch primitive.
///
/// `total` is LIFETIME-monotonic (never reset — it is the observable counter);
/// the latch is per-regime: the first record after arming logs loudly, repeats
/// demote, and [`rearm`](Self::rearm) (a successful push, a pipeline rebuild,
/// a JPEG that fits) opens a fresh regime.
///
/// Public because the node uses it for its own oversized-JPEG class in
/// `lib.rs`: the same counted-always / loud-once-per-regime contract, decided
/// by the same tested code rather than re-implemented beside it.
/// CAPTURED, not `reconstruct`. `total` is the lifetime-monotonic
/// observable this type exists to hold, so reconstructing it would zero
/// exactly the number an operator reads — and both fields are primitives, so
/// capture is free. See the observability rule in `state_derive.rs`: this is
/// the "carried, because it costs nothing" side of it.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, cerulion_core::state::CerulionState)]
pub struct FloodCounter {
    total: u64,
    warned: bool,
}

impl FloodCounter {
    /// Count one occurrence and decide how loudly to log it.
    pub fn record(&mut self) -> FloodLog {
        self.total += 1;
        if self.warned {
            FloodLog::Repeat { total: self.total }
        } else {
            self.warned = true;
            FloodLog::First { total: self.total }
        }
    }

    /// Count one occurrence WITHOUT ever logging it (the expected-sibling
    /// class — at 15-30 fps it would be a line per frame forever).
    pub fn record_quiet(&mut self) -> FloodLog {
        self.total += 1;
        FloodLog::Quiet
    }

    /// Open a fresh warn regime; the lifetime total is untouched.
    pub fn rearm(&mut self) {
        self.warned = false;
    }

    /// The LIFETIME count — bumped on every record regardless of the latch,
    /// so it is a complete total even while the logs are quiet.
    pub fn total(&self) -> u64 {
        self.total
    }
}

/// The admission gate: rendition filter + wait-for-SPS, with per-class
/// counters and flood latches. See the module docs.
#[derive(Debug, Clone, cerulion_core::state::CerulionState)]
pub struct AuGate {
    target_height: u32,
    sps_seen: bool,
    empty: FloodCounter,
    other_rendition: FloodCounter,
    unknown_rendition: FloodCounter,
    waiting_for_sps: FloodCounter,
    admitted: u64,
}

impl AuGate {
    /// A gate admitting only `target_height`, waiting for an SPS first.
    pub fn new(target_height: u32) -> Self {
        Self {
            target_height,
            sps_seen: false,
            empty: FloodCounter::default(),
            other_rendition: FloodCounter::default(),
            unknown_rendition: FloodCounter::default(),
            waiting_for_sps: FloodCounter::default(),
            admitted: 0,
        }
    }

    /// The rendition height this gate admits.
    pub fn target_height(&self) -> u32 {
        self.target_height
    }

    /// True once an admitted access unit carrying an SPS has been seen (i.e.
    /// the decoder has been given a starting point).
    pub fn sps_seen(&self) -> bool {
        self.sps_seen
    }

    /// Access units ADMITTED so far (lifetime) — i.e. classified [`AuVerdict::Push`].
    ///
    /// This is an admission count, NOT a decode count and not even a push
    /// count: the gate decides, the caller pushes, and a push can still be
    /// REFUSED by a dying pipeline (`TranscodeLoop` tears down and rebuilds on
    /// that). The successful-push tally is `TranscodeLoop::aus_pushed`, and
    /// the two are logged side by side in the node's teardown summary — a gap
    /// between them is exactly the refused-push count.
    pub fn admitted(&self) -> u64 {
        self.admitted
    }

    /// Lifetime count of zero-length payloads skipped.
    pub fn skipped_empty(&self) -> u64 {
        self.empty.total
    }

    /// Lifetime count of the expected sibling rendition skipped.
    pub fn skipped_other_rendition(&self) -> u64 {
        self.other_rendition.total
    }

    /// Lifetime count of never-before-seen heights skipped.
    pub fn skipped_unknown_rendition(&self) -> u64 {
        self.unknown_rendition.total
    }

    /// Lifetime count of target-rendition access units dropped because no SPS
    /// had arrived yet.
    pub fn skipped_waiting_for_sps(&self) -> u64 {
        self.waiting_for_sps.total
    }

    /// Re-arm for a FRESH decoder: a rebuilt pipeline has no parameter sets, so
    /// the SPS gate closes again and every warn latch opens a new regime. The
    /// lifetime counters are deliberately NOT reset — they are the observable
    /// totals across the whole run.
    pub fn reset_for_new_pipeline(&mut self) {
        self.sps_seen = false;
        self.rearm_latches();
    }

    fn rearm_latches(&mut self) {
        self.empty.rearm();
        self.other_rendition.rearm();
        self.unknown_rendition.rearm();
        self.waiting_for_sps.rearm();
    }

    /// Classify one access unit off the wire.
    ///
    /// Order is deliberate: an EMPTY payload is nothing at any height; then the
    /// rendition filter (so a wrong-rendition IDR never opens the SPS gate for
    /// a stream that is not being decoded); then the SPS gate. A target-rendition
    /// access unit carrying an SPS opens the gate AND re-arms every warn latch
    /// — the stream is healthy again.
    pub fn classify(&mut self, video_height: u32, au: &[u8]) -> GateOutcome {
        if au.is_empty() {
            return GateOutcome {
                verdict: AuVerdict::SkipEmpty,
                log: self.empty.record(),
            };
        }
        if video_height != self.target_height {
            return if KNOWN_RENDITION_HEIGHTS.contains(&video_height) {
                GateOutcome {
                    verdict: AuVerdict::SkipOtherRendition,
                    log: self.other_rendition.record_quiet(),
                }
            } else {
                GateOutcome {
                    verdict: AuVerdict::SkipUnknownRendition,
                    log: self.unknown_rendition.record(),
                }
            };
        }
        if contains_sps(au) {
            self.sps_seen = true;
        }
        if !self.sps_seen {
            return GateOutcome {
                verdict: AuVerdict::SkipWaitingForSps,
                log: self.waiting_for_sps.record(),
            };
        }
        self.admitted += 1;
        self.rearm_latches();
        GateOutcome {
            verdict: AuVerdict::Push,
            log: FloodLog::Quiet,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build an Annex-B access unit from `(nal_type, payload_len)` pairs using
    /// 4-byte start codes. HAND-BUILT: the framing here is written from the
    /// H.264 spec, never derived from the scanner under test.
    fn au(nals: &[(u8, usize)]) -> Vec<u8> {
        let mut v = Vec::new();
        for (ty, len) in nals {
            v.extend_from_slice(&[0x00, 0x00, 0x00, 0x01]);
            // nal_ref_idc = 3 (0b011 << 5 = 0x60), forbidden_zero_bit = 0.
            v.push(0x60 | (ty & 0x1F));
            // Filler payload that deliberately contains NO start code.
            v.resize(v.len() + *len, 0xAB);
        }
        v
    }

    /// The same, with 3-byte start codes (the other legal Annex-B form).
    fn au_short_start_codes(nals: &[(u8, usize)]) -> Vec<u8> {
        let mut v = Vec::new();
        for (ty, len) in nals {
            v.extend_from_slice(&[0x00, 0x00, 0x01]);
            v.push(0x60 | (ty & 0x1F));
            v.resize(v.len() + *len, 0xAB);
        }
        v
    }

    /// An IDR access unit as the Go2 sends it: SPS(7) + PPS(8) + IDR(5).
    fn idr_au() -> Vec<u8> {
        au(&[(7, 12), (8, 4), (5, 900)])
    }

    /// A plain non-IDR slice access unit (NAL type 1) — no parameter sets.
    fn slice_au() -> Vec<u8> {
        au(&[(1, 700)])
    }

    // ---- contains_nal_type / contains_sps ----

    #[test]
    fn sps_is_found_under_both_start_code_forms() {
        assert!(contains_sps(&idr_au()), "4-byte start codes");
        assert!(
            contains_sps(&au_short_start_codes(&[(7, 12), (8, 4), (5, 900)])),
            "3-byte start codes"
        );
        // Mixed: a 3-byte-prefixed SPS after a 4-byte-prefixed AUD.
        let mut mixed = au(&[(9, 1)]);
        mixed.extend_from_slice(&au_short_start_codes(&[(7, 8)]));
        assert!(contains_sps(&mixed), "mixed start-code widths");
    }

    #[test]
    fn a_slice_only_access_unit_carries_no_sps() {
        assert!(!contains_sps(&slice_au()));
        // Every other NAL type present, SPS absent (PPS 8 is the near miss).
        assert!(!contains_sps(&au(&[(9, 1), (8, 4), (1, 100), (6, 20)])));
    }

    #[test]
    fn a_forbidden_zero_bit_rejects_the_nal_header() {
        // Same bytes as an SPS NAL header except the forbidden_zero_bit is
        // set — not a NAL header, so not an SPS (never masked into one).
        let bytes = vec![0x00, 0x00, 0x00, 0x01, 0x80 | 0x67, 0xAB, 0xAB, 0xAB];
        assert!(!contains_sps(&bytes));
    }

    #[test]
    fn truncated_and_empty_inputs_never_panic_and_never_match() {
        assert!(!contains_sps(&[]));
        assert!(!contains_sps(&[0x00]));
        assert!(!contains_sps(&[0x00, 0x00]));
        assert!(!contains_sps(&[0x00, 0x00, 0x01]), "start code, no header");
        // A start code at the very end with the header byte present matches.
        assert!(contains_sps(&[0xFF, 0x00, 0x00, 0x01, 0x67]));
    }

    #[test]
    fn a_start_code_beginning_at_the_previous_nal_header_is_not_skipped() {
        // THE i+3 fixture. Byte for byte:
        //
        //   [0]=00 [1]=00 [2]=01   start code #1
        //   [3]=00                 its NAL header: type 0 ("unspecified") —
        //                          rejected, and ALSO the first byte of...
        //   [3]=00 [4]=00 [5]=01   ...start code #2, which begins at i+3
        //   [6]=67                 its NAL header: an SPS
        //
        // The scanner matches at i=0, rejects the type-0 header, and must
        // resume at i+3. A `i += 4` skip lands on [4] and never matches again,
        // so the SPS behind the second start code is MISSED — the exact
        // mutation this fixture kills (verified by reverting the skip).
        let mut bytes = vec![0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x67];
        bytes.resize(bytes.len() + 8, 0xAB);
        assert!(
            contains_sps(&bytes),
            "a start code beginning at the PREVIOUS NAL header byte must still be scanned"
        );
        // Same shape, no SPS behind the second start code: still no false
        // positive (the resumed scan reads a real header, it does not guess).
        let mut nope = vec![0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x61];
        nope.resize(nope.len() + 8, 0xAB);
        assert!(!contains_sps(&nope));
    }

    #[test]
    fn a_payload_byte_run_of_zeroes_is_not_a_false_start_code() {
        // `00 00 02` is legal payload (emulation prevention only forbids
        // 00 00 00/01/02/03 unescaped, and 02 is not a start code).
        let bytes = vec![0x00, 0x00, 0x02, 0x67, 0x00, 0x00, 0x00, 0x02, 0x67];
        assert!(!contains_sps(&bytes));
    }

    #[test]
    fn contains_nal_type_discriminates_between_types() {
        let a = au(&[(5, 40)]);
        assert!(contains_nal_type(&a, 5), "IDR slice present");
        assert!(!contains_nal_type(&a, 7), "no SPS in an IDR-only AU");
        assert!(!contains_nal_type(&a, 1), "not a non-IDR slice");
    }

    // ---- FloodCounter (the shared flood-latch primitive) ----
    //
    // Tested DIRECTLY, not only through `AuGate`, because `lib.rs` uses it
    // for the node's oversized-JPEG class too — a loss class whose logging
    // cannot be observed from a test (no subscriber is installed in this
    // crate's suite), so the DECISION is the only thing that can be pinned,
    // and it must be pinned where it is decided.

    #[test]
    fn the_flood_latch_is_loud_once_per_regime_and_counts_unconditionally() {
        let mut c = FloodCounter::default();
        assert_eq!(c.total(), 0);
        // First of a regime: LOUD.
        assert_eq!(c.record(), FloodLog::First { total: 1 });
        // Repeats: demoted — but still counted, every one of them.
        for n in 2..=100u64 {
            assert_eq!(c.record(), FloodLog::Repeat { total: n });
        }
        assert_eq!(c.total(), 100, "the total is unconditional, never latched");
        // Re-arm opens a fresh regime: loud again, total keeps running.
        c.rearm();
        assert_eq!(c.record(), FloodLog::First { total: 101 });
        assert_eq!(c.record(), FloodLog::Repeat { total: 102 });
        assert_eq!(c.total(), 102, "re-arming never resets the lifetime total");
    }

    #[test]
    fn the_flood_latch_quiet_arm_counts_without_ever_logging() {
        // The expected-sibling class: half of every frame pair, so it must
        // never log — but it must still be counted, or the teardown summary
        // would under-report the loss.
        let mut c = FloodCounter::default();
        for _ in 0..5 {
            assert_eq!(c.record_quiet(), FloodLog::Quiet);
        }
        assert_eq!(c.total(), 5);
        // A quiet record does NOT arm the latch, so a later loud record is
        // still the first of its regime.
        assert_eq!(c.record(), FloodLog::First { total: 6 });
    }

    #[test]
    fn re_arming_an_unfired_flood_latch_is_a_no_op() {
        // Idempotence at the boundary the node hits on EVERY good frame: the
        // JPEG-fits path re-arms unconditionally, so re-arming an already-armed
        // counter must not disturb the total or the next verdict.
        let mut c = FloodCounter::default();
        c.rearm();
        c.rearm();
        assert_eq!(c.total(), 0);
        assert_eq!(c.record(), FloodLog::First { total: 1 });
    }

    // ---- resolve_target_height (the CAMERA_TARGET_HEIGHT policy) ----

    #[test]
    fn a_known_configured_height_is_honoured_silently() {
        // Both known renditions resolve to themselves with nothing to say.
        assert_eq!(resolve_target_height(720), (720, TargetHeightNote::Known));
        assert_eq!(resolve_target_height(360), (360, TargetHeightNote::Known));
        // ...and the default is one of them (a drift guard: a default outside
        // the known set would make every launch warn).
        assert_eq!(
            resolve_target_height(DEFAULT_TARGET_HEIGHT),
            (DEFAULT_TARGET_HEIGHT, TargetHeightNote::Known)
        );
    }

    #[test]
    fn a_zero_configured_height_falls_back_to_the_default_and_says_so() {
        // 0 is what an unset or unparseable env var reads as, and it matches
        // NO rendition — so it must SUBSTITUTE the default (not be honoured as
        // a target that skips every access unit) and must be LOUD about it.
        assert_eq!(
            resolve_target_height(0),
            (DEFAULT_TARGET_HEIGHT, TargetHeightNote::ZeroFallback)
        );
    }

    #[test]
    fn an_unknown_configured_height_is_honoured_but_flagged() {
        // Both halves matter: the height is passed through UNCHANGED
        // (the node decodes it if the robot really publishes it — the operator's
        // ask is never silently rewritten), and the note is the one that
        // makes `init` warn. A resolver that clamped to the default would fail
        // the first assert; one that returned `Known` would fail the second.
        assert_eq!(
            resolve_target_height(1080),
            (1080, TargetHeightNote::UnknownHeight)
        );
        assert_eq!(
            resolve_target_height(1),
            (1, TargetHeightNote::UnknownHeight)
        );
        assert_eq!(
            resolve_target_height(u32::MAX),
            (u32::MAX, TargetHeightNote::UnknownHeight)
        );
    }

    // ---- AuGate: the rendition filter ----

    #[test]
    fn the_target_rendition_is_admitted_and_the_sibling_is_dropped_silently() {
        let mut gate = AuGate::new(720);
        // The stream head: a 720 IDR opens the gate.
        assert_eq!(
            gate.classify(720, &idr_au()),
            GateOutcome {
                verdict: AuVerdict::Push,
                log: FloodLog::Quiet
            }
        );
        // The 360 twin of the same time_frame: dropped, counted, NOT logged.
        assert_eq!(
            gate.classify(360, &idr_au()),
            GateOutcome {
                verdict: AuVerdict::SkipOtherRendition,
                log: FloodLog::Quiet
            }
        );
        // A later 720 slice rides the open gate.
        assert_eq!(
            gate.classify(720, &slice_au()),
            GateOutcome {
                verdict: AuVerdict::Push,
                log: FloodLog::Quiet
            }
        );
        assert_eq!(gate.admitted(), 2);
        assert_eq!(gate.skipped_other_rendition(), 1);
        assert_eq!(gate.skipped_unknown_rendition(), 0);
    }

    #[test]
    fn the_filter_is_configurable_and_inverts_cleanly() {
        // With the target flipped to 360, the SAME two access
        // units swap roles. A comparison bug (`==` → `!=`, or a hardcoded 720)
        // fails here in the opposite direction from the test above.
        let mut gate = AuGate::new(360);
        assert_eq!(gate.classify(360, &idr_au()).verdict, AuVerdict::Push);
        assert_eq!(
            gate.classify(720, &idr_au()).verdict,
            AuVerdict::SkipOtherRendition
        );
        assert_eq!(gate.admitted(), 1);
        assert_eq!(gate.skipped_other_rendition(), 1);
        assert_eq!(gate.target_height(), 360);
    }

    #[test]
    fn an_unknown_height_warns_once_per_regime_and_is_always_counted() {
        let mut gate = AuGate::new(720);
        // 1080 is not a rendition this stream carries.
        assert_eq!(
            gate.classify(1080, &idr_au()),
            GateOutcome {
                verdict: AuVerdict::SkipUnknownRendition,
                log: FloodLog::First { total: 1 }
            }
        );
        for n in 2..=5u64 {
            assert_eq!(
                gate.classify(1080, &idr_au()),
                GateOutcome {
                    verdict: AuVerdict::SkipUnknownRendition,
                    log: FloodLog::Repeat { total: n }
                },
                "repeat {n} must demote, and must still COUNT"
            );
        }
        assert_eq!(gate.skipped_unknown_rendition(), 5);
        // A good frame re-arms the regime: the next unknown warns LOUDLY again.
        assert_eq!(gate.classify(720, &idr_au()).verdict, AuVerdict::Push);
        assert_eq!(
            gate.classify(1080, &idr_au()).log,
            FloodLog::First { total: 6 },
            "a successful push opens a new warn regime; the total keeps running"
        );
    }

    #[test]
    fn a_zero_height_is_unknown_not_a_sibling() {
        // The wire field is a u32; 0 is what a mis-decoded sample reads as, so
        // it must be LOUD, not folded into the silent sibling class.
        let mut gate = AuGate::new(720);
        assert_eq!(
            gate.classify(0, &idr_au()),
            GateOutcome {
                verdict: AuVerdict::SkipUnknownRendition,
                log: FloodLog::First { total: 1 }
            }
        );
    }

    // ---- AuGate: the SPS gate ----

    #[test]
    fn nothing_is_pushed_until_the_first_sps_arrives() {
        let mut gate = AuGate::new(720);
        // Mid-GOP join: slices arrive before any parameter set.
        assert_eq!(
            gate.classify(720, &slice_au()),
            GateOutcome {
                verdict: AuVerdict::SkipWaitingForSps,
                log: FloodLog::First { total: 1 }
            }
        );
        assert_eq!(
            gate.classify(720, &slice_au()),
            GateOutcome {
                verdict: AuVerdict::SkipWaitingForSps,
                log: FloodLog::Repeat { total: 2 }
            }
        );
        assert!(!gate.sps_seen());
        assert_eq!(gate.admitted(), 0, "not one byte reached the decoder");

        // The IDR access unit opens the gate — and is itself pushed (it is the
        // keyframe the decoder needs, not merely the signal that one arrived).
        assert_eq!(gate.classify(720, &idr_au()).verdict, AuVerdict::Push);
        assert!(gate.sps_seen());
        // Slices now ride through.
        assert_eq!(gate.classify(720, &slice_au()).verdict, AuVerdict::Push);
        assert_eq!(gate.admitted(), 2);
        assert_eq!(gate.skipped_waiting_for_sps(), 2);
    }

    #[test]
    fn a_sibling_rendition_idr_never_opens_the_gate() {
        // Pins the ORDER of the two filters: if the SPS scan ran
        // before the rendition filter, this 360 IDR would set `sps_seen` and
        // the next 720 slice would be pushed into a decoder that has never
        // seen a 720 parameter set.
        let mut gate = AuGate::new(720);
        assert_eq!(
            gate.classify(360, &idr_au()).verdict,
            AuVerdict::SkipOtherRendition
        );
        assert!(!gate.sps_seen(), "the sibling's SPS is not the target SPS");
        assert_eq!(
            gate.classify(720, &slice_au()).verdict,
            AuVerdict::SkipWaitingForSps
        );
        assert_eq!(gate.admitted(), 0);
    }

    #[test]
    fn an_empty_payload_is_skipped_loudly_at_any_height() {
        let mut gate = AuGate::new(720);
        assert_eq!(
            gate.classify(720, &[]),
            GateOutcome {
                verdict: AuVerdict::SkipEmpty,
                log: FloodLog::First { total: 1 }
            }
        );
        // Even at the sibling height, empty is empty (checked first).
        assert_eq!(
            gate.classify(360, &[]),
            GateOutcome {
                verdict: AuVerdict::SkipEmpty,
                log: FloodLog::Repeat { total: 2 }
            }
        );
        assert_eq!(gate.skipped_empty(), 2);
        assert_eq!(gate.skipped_other_rendition(), 0, "empty is not a sibling");
    }

    // ---- AuGate: pipeline rebuild ----

    #[test]
    fn a_rebuilt_pipeline_closes_the_sps_gate_again() {
        let mut gate = AuGate::new(720);
        assert_eq!(gate.classify(720, &idr_au()).verdict, AuVerdict::Push);
        assert!(gate.sps_seen());

        // The decoder died and was rebuilt: it has no parameter sets, so a
        // mid-GOP slice must NOT be pushed into it.
        gate.reset_for_new_pipeline();
        assert!(!gate.sps_seen());
        assert_eq!(
            gate.classify(720, &slice_au()),
            GateOutcome {
                verdict: AuVerdict::SkipWaitingForSps,
                // A fresh pipeline is a fresh regime — LOUD again, and the
                // lifetime total keeps running (this is the first such skip).
                log: FloodLog::First { total: 1 }
            }
        );
        // The next IDR re-opens it.
        assert_eq!(gate.classify(720, &idr_au()).verdict, AuVerdict::Push);
        assert_eq!(gate.admitted(), 2, "counters are lifetime, never reset");
    }

    #[test]
    fn counters_survive_a_reset_and_only_the_latches_re_arm() {
        let mut gate = AuGate::new(720);
        gate.classify(1080, &idr_au()); // unknown #1 (First)
        gate.classify(1080, &idr_au()); // unknown #2 (Repeat)
        gate.reset_for_new_pipeline();
        assert_eq!(
            gate.classify(1080, &idr_au()).log,
            FloodLog::First { total: 3 },
            "the latch re-armed (loud again) but the total is cumulative"
        );
        assert_eq!(gate.skipped_unknown_rendition(), 3);
    }

    #[test]
    fn a_full_frame_pair_script_matches_a_hand_oracle() {
        // The live shape: per time_frame the robot publishes one 360 and one
        // 720 access unit. Script the first three frame pairs of a stream
        // joined mid-GOP, and compare the whole run to a hand oracle (never to
        // a second run of the gate).
        //
        // The oracle is `(verdict, sps_seen)` per step, NOT verdicts alone.
        // That column is load-bearing, not decoration: the sibling keyframe at
        // step 3 leaves the VERDICT sequence completely unchanged if the SPS
        // scan is hoisted above the rendition filter (a 360 IDR still
        // classifies SkipOtherRendition either way, and every later step is a
        // 720 that was already going to Push). Only `sps_seen` moves — it flips
        // true at step 3 with the scan hoisted instead of step 4 — so WITHOUT
        // this column the inline "must not open the gate" comment below would
        // be a claim this test cannot actually make. With it, a hoisted scan
        // fails here as well as in `a_sibling_rendition_idr_never_opens_the_gate`.
        let mut gate = AuGate::new(DEFAULT_TARGET_HEIGHT);
        let script: Vec<(u32, Vec<u8>)> = vec![
            (360, slice_au()), // sibling
            (720, slice_au()), // mid-GOP: no SPS yet
            (360, idr_au()),   // sibling keyframe — must not open the gate
            (720, idr_au()),   // the target's keyframe
            (360, slice_au()), // sibling
            (720, slice_au()), // rides the open gate
        ];
        let got: Vec<(AuVerdict, bool)> = script
            .iter()
            .map(|(h, au)| (gate.classify(*h, au).verdict, gate.sps_seen()))
            .collect();
        assert_eq!(
            got,
            vec![
                (AuVerdict::SkipOtherRendition, false),
                (AuVerdict::SkipWaitingForSps, false),
                // THE order pin: the sibling's IDR carries an SPS, and the
                // gate must still be CLOSED after it.
                (AuVerdict::SkipOtherRendition, false),
                (AuVerdict::Push, true),
                (AuVerdict::SkipOtherRendition, true),
                (AuVerdict::Push, true),
            ]
        );
        assert_eq!(gate.admitted(), 2);
        assert_eq!(gate.skipped_other_rendition(), 3);
        assert_eq!(gate.skipped_waiting_for_sps(), 1);
        assert_eq!(gate.skipped_empty(), 0);
        assert_eq!(gate.skipped_unknown_rendition(), 0);
    }
}
