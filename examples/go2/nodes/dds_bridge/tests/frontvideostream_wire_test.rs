// SPDX-License-Identifier: AGPL-3.0-only
//! The `/frontvideostream` decode blocker — REAL captured Go2 samples
//! must transcode, and the definition that could not decode them must still be
//! REFUSED for the same reason it was refused live.
//!
//! # Why this file exists
//!
//! On the live robot every `/frontvideostream` sample was dropped with
//!
//! ```text
//! CDR body for 'unitree_go/Go2FrontVideoData' field 'video360p' has a
//! hostile length/count prefix 2170829756 over only 1760 remaining bytes
//! ```
//!
//! so the topic read `no_data` and nothing downstream could work. The byte
//! budget was CORRECT — the length really was nonsense. The **definition** was
//! wrong: the `unitree_go` ament package's four-field
//! `{time_frame, video720p, video360p, video180p}` reconstruction does not
//! describe what the firmware's bare-DDS writer puts on the wire. See
//! `examples/go2/schemas/unitree_go/msg/Go2FrontVideoData.msg` for the full
//! derivation; the corrected layout is
//! `{uint64 time_frame, uint32 video_height, uint8[] video_data}`.
//!
//! # The fixtures are real robot bytes (Principle #13)
//!
//! `tests/fixtures/go2_frontvideostream_*.cdr` are three verbatim DDS payloads
//! (4-byte encapsulation header + CDR body) captured from a Go2 robot with an
//! rclpy `raw=True` subscription — serialized bytes, never deserialized, so
//! nothing in the capture path could have reshaped them. Nothing here is
//! synthesized.
//!
//! # Why these assertions are not a self-compare
//!
//! Every expected value is produced by [`hand_parse`], which walks the raw
//! fixture bytes with an independent, hand-written reader that shares no code
//! with [`CdrCodec`]. The codec's output is then read back through the
//! PRODUCTION reader ([`FrameWalker`] — the same decoder `cerulion_viz` uses)
//! and compared against that oracle. Two independent paths over the same
//! bytes.
//!
//! The strongest structural pin is [`decoded_frame_round_trips_back_to_the_exact_cdr_body`]:
//! `encode(decode(body)) == body` proves the layout accounts for EVERY byte,
//! not merely that it parses without erroring.
//!
//! No transport, no DDS, no iceoryx2 — pure codec over file bytes;
//! parallel-safe.

use std::path::{Path, PathBuf};

use cerulion_core::codegen::{
    parse_rosmsg, CdrCodec, CdrCodecError, CdrEndianness, FrameValueKind, FrameWalker,
    MessageSchema, MAX_CDR_TRAILING_PAD,
};

/// The qualified schema name, as the DDS publisher announces it
/// (`ros2 topic info -v /frontvideostream` → `unitree_go/msg/Go2FrontVideoData`,
/// which the bridge normalizes to `pkg/Type`).
const QNAME: &str = "unitree_go/Go2FrontVideoData";

/// The H.264 Annex-B 4-byte start code that opens every captured access unit.
const ANNEXB_START_CODE: [u8; 4] = [0x00, 0x00, 0x00, 0x01];

/// The largest `/frontvideostream` access unit MEASURED on the live robot:
/// 62,612 bytes, a 720p IDR carrying SPS+PPS+IDR, over 60 consecutive live
/// samples. Provenance: the same capture session the committed fixtures came
/// from; the number is quoted beside `max_slice_len` in
/// `graphs/go2.bridge.yaml` and in the derivation section of
/// `schemas/unitree_go/msg/Go2FrontVideoData.msg`.
///
/// This is deliberately NOT derived from the fixtures. The committed captures
/// top out around 23 KB (a 62 KB fourth capture is not worth carrying in git),
/// so a fixture-derived floor would accept a `max_slice_len` a third of the
/// real peak — green here, every 720p IDR dropped on the robot.
const MEASURED_MAX_ACCESS_UNIT_BYTES: usize = 62_612;

/// The upstream `unitree_go` ament definition — the one that could not decode a
/// real sample. Quoted VERBATIM from
/// `autonomy_stack_go2/install/unitree_go/share/unitree_go/msg/Go2FrontVideoData.msg`
/// on the robot (byte-checked with `cat -A`). This is the regression arm's
/// input, not a strawman.
const UPSTREAM_AMENT_DEFINITION: &str = "\
uint64 time_frame
uint8[] video720p
uint8[] video360p
uint8[] video180p
";

/// A definition that is a strict PREFIX of the shipped one — the shape the
/// wire `schema_hash` gate structurally cannot catch, because a publisher and
/// a subscriber both mint that hash from the same (wrong) local corpus entry.
/// Every length prefix it reads is sane, so nothing blows up; it simply stops
/// early and, without the consumption gate, would publish a frame with the
/// video silently missing.
const SHORT_PREFIX_DEFINITION: &str = "\
uint64 time_frame
uint32 video_height
";

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

/// One real captured sample plus the facts an independent hand parse recovers
/// from it.
struct Fixture {
    /// File stem, for assertion messages.
    name: &'static str,
    /// The verbatim DDS payload: 4-byte encapsulation header + CDR body.
    payload: Vec<u8>,
    /// Whether this capture is an IDR access unit (carries SPS + PPS + IDR).
    /// Declared here rather than sniffed from `name` — `"360p_nonidr"` also
    /// ends in `"idr"`, and an accidental match made the IDR assertion fire on
    /// a non-IDR fixture.
    is_idr: bool,
}

fn fixtures_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
}

fn load(name: &'static str, is_idr: bool) -> Fixture {
    let path = fixtures_dir().join(format!("go2_frontvideostream_{name}.cdr"));
    let payload = std::fs::read(&path).unwrap_or_else(|e| {
        panic!(
            "Fixture missing or unreadable at {}: {e}. These are real \
             captured robot bytes and are committed alongside this test — if the \
             file is gone, restore it rather than regenerating it by hand.",
            path.display()
        )
    });
    Fixture {
        name,
        payload,
        is_idr,
    }
}

/// All three committed captures: both renditions, plus an IDR access unit.
fn all_fixtures() -> Vec<Fixture> {
    vec![
        load("360p_nonidr", false),
        load("720p_nonidr", false),
        load("360p_idr", true),
    ]
}

// ---------------------------------------------------------------------------
// The independent hand oracle
// ---------------------------------------------------------------------------

/// What an independent hand parse recovers from a raw fixture. Shares NO code
/// with `CdrCodec`.
#[derive(Debug, PartialEq, Eq)]
struct HandParsed {
    time_frame: u64,
    video_height: u32,
    /// The H.264 access unit: `body[16 .. 16 + declared_len]`.
    video_data: Vec<u8>,
    /// CDR body length, i.e. `payload.len() - 4`.
    body_len: usize,
    /// Bytes after the access unit — CDR trailing alignment padding.
    trailing_pad: usize,
    /// The padding count the WRITER declared, read out of the encapsulation
    /// options (`payload[3] & 0x3` — the two low bits of the second options
    /// octet, per the RTPS/XTypes encapsulation header). Independent evidence
    /// for what the leftover tail is: see the round-trip test.
    declared_pad: usize,
}

/// Read a fixture's raw bytes by hand under the CORRECTED layout:
/// `u64 @0`, `u32 @8`, `u32 count @12`, `count` bytes `@16`.
///
/// This function is the oracle. It deliberately duplicates the little-endian
/// reads instead of calling anything under test.
fn hand_parse(payload: &[u8]) -> HandParsed {
    assert!(
        payload.len() > 16,
        "a real fixture is always longer than its 4-byte encapsulation header \
         plus the 16-byte fixed prologue"
    );
    // Encapsulation: CDR_LE (rep_id 0x0001). Every Go2 sample observed used it.
    assert_eq!(
        &payload[0..2],
        &[0x00, 0x01],
        "fixture is not CDR_LE — the hand oracle below reads little-endian"
    );
    let body = &payload[4..];

    let time_frame = u64::from_le_bytes(body[0..8].try_into().expect("8 bytes"));
    let video_height = u32::from_le_bytes(body[8..12].try_into().expect("4 bytes"));
    let declared_len = u32::from_le_bytes(body[12..16].try_into().expect("4 bytes")) as usize;
    assert!(
        16 + declared_len <= body.len(),
        "{declared_len} declared bytes do not fit in a {}-byte body — this \
         fixture does not have the layout the oracle assumes",
        body.len()
    );
    let video_data = body[16..16 + declared_len].to_vec();

    HandParsed {
        time_frame,
        video_height,
        video_data,
        body_len: body.len(),
        trailing_pad: body.len() - (16 + declared_len),
        declared_pad: (payload[3] & 0x03) as usize,
    }
}

// ---------------------------------------------------------------------------
// Codecs under test
// ---------------------------------------------------------------------------

/// Parse the SHIPPED `.msg` file — so reverting
/// `examples/go2/schemas/unitree_go/msg/Go2FrontVideoData.msg` fails this test.
fn shipped_schema() -> MessageSchema {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../schemas/unitree_go/msg/Go2FrontVideoData.msg");
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "The workspace-store override is missing at {}: {e}. \
             Without it the bridge falls back to the ament package's \
             four-field definition and /frontvideostream cannot decode.",
            path.display()
        )
    });
    parse_rosmsg(&text, "Go2FrontVideoData", Some("unitree_go"))
        .expect("the shipped Go2FrontVideoData.msg must parse")
}

fn codec_over(schema: MessageSchema) -> CdrCodec {
    let (codec, warnings) = CdrCodec::new(vec![schema]);
    assert!(
        warnings.is_empty(),
        "unexpected schema-resolution warnings: {warnings:?}"
    );
    assert!(codec.knows(QNAME), "codec must know {QNAME}");
    codec
}

fn shipped_codec() -> CdrCodec {
    codec_over(shipped_schema())
}

fn upstream_ament_codec() -> CdrCodec {
    codec_over(
        parse_rosmsg(
            UPSTREAM_AMENT_DEFINITION,
            "Go2FrontVideoData",
            Some("unitree_go"),
        )
        .expect("the upstream ament definition must parse (it is valid .msg)"),
    )
}

fn short_prefix_codec() -> CdrCodec {
    codec_over(
        parse_rosmsg(
            SHORT_PREFIX_DEFINITION,
            "Go2FrontVideoData",
            Some("unitree_go"),
        )
        .expect("the prefix definition must parse (it is valid .msg)"),
    )
}

fn walker_over_shipped() -> FrameWalker {
    let (walker, warnings) = FrameWalker::new(vec![shipped_schema()]);
    assert!(warnings.is_empty(), "walker warnings: {warnings:?}");
    walker
}

// ---------------------------------------------------------------------------
// THE HEADLINE: a real captured frame decodes, and every field matches the
// independent hand oracle.
// ---------------------------------------------------------------------------

#[test]
fn real_captured_samples_decode_and_every_field_matches_the_hand_oracle() {
    let codec = shipped_codec();
    let walker = walker_over_shipped();

    for (i, fx) in all_fixtures().iter().enumerate() {
        let oracle = hand_parse(&fx.payload);

        let frame = codec
            .decode_dds_payload(QNAME, &fx.payload, i as u32, 1_000 + i as u64)
            .unwrap_or_else(|e| {
                panic!(
                    "REGRESSION: the real captured sample '{}' failed to \
                     decode under the shipped definition: {e}",
                    fx.name
                )
            });

        // Read the produced frame back through the PRODUCTION reader.
        let fv = walker
            .walk_by_hash(&frame)
            .unwrap_or_else(|e| panic!("walking the decoded '{}' frame: {e:?}", fx.name));

        assert_eq!(
            fv.schema_name, QNAME,
            "{}: walker resolved the wrong schema",
            fx.name
        );

        match fv.field("time_frame") {
            Some(FrameValueKind::U64(v)) => assert_eq!(
                *v, oracle.time_frame,
                "{}: time_frame disagrees with the hand oracle",
                fx.name
            ),
            other => panic!("{}: time_frame is not a U64: {other:?}", fx.name),
        }

        match fv.field("video_height") {
            Some(FrameValueKind::U32(v)) => assert_eq!(
                *v, oracle.video_height,
                "{}: video_height disagrees with the hand oracle",
                fx.name
            ),
            other => panic!("{}: video_height is not a U32: {other:?}", fx.name),
        }

        match fv.field("video_data") {
            Some(FrameValueKind::Bytes(b)) => {
                assert_eq!(
                    b.len(),
                    oracle.video_data.len(),
                    "{}: video_data length disagrees with the hand oracle",
                    fx.name
                );
                assert_eq!(
                    *b,
                    oracle.video_data.as_slice(),
                    "{}: video_data bytes disagree with the hand oracle",
                    fx.name
                );
            }
            other => panic!("{}: video_data is not Bytes: {other:?}", fx.name),
        }
    }
}

/// The layout claim is TOTAL, not merely parseable: re-encoding the decoded
/// frame reproduces the original CDR body byte-for-byte (up to the trailing
/// alignment pad the encoder legitimately omits — the documented
/// `encode(decode(cdr)) == cdr` inverse).
///
/// A definition that skipped, mis-sized, or invented a field could not
/// round-trip.
#[test]
fn decoded_frame_round_trips_back_to_the_exact_cdr_body() {
    let codec = shipped_codec();

    for fx in all_fixtures() {
        let oracle = hand_parse(&fx.payload);
        let body = &fx.payload[4..];

        let frame = codec
            .decode(QNAME, CdrEndianness::Little, body, 7, 42)
            .unwrap_or_else(|e| panic!("{}: decode: {e}", fx.name));
        let re = codec
            .encode(QNAME, CdrEndianness::Little, &frame)
            .unwrap_or_else(|e| panic!("{}: encode: {e}", fx.name));

        let significant = &body[..oracle.body_len - oracle.trailing_pad];
        assert_eq!(
            re.len(),
            significant.len(),
            "{}: re-encoded body length {} != the original's {} significant \
             bytes — the layout does not account for every byte",
            fx.name,
            re.len(),
            significant.len()
        );
        assert_eq!(
            re, significant,
            "{}: re-encoded CDR body differs from the captured one",
            fx.name
        );

        // The omitted tail really is only CDR alignment padding, and the
        // WRITER says so: the encapsulation options carry the padding count,
        // and all three fixtures declare exactly what is left over (3, 2 and
        // 3 bytes, in `all_fixtures()` order — pinned per fixture by the
        // consumption-gate arm's `KNOWN_PADS`).
        //
        // This is the assertion that discriminates. "fewer than 4 bytes, all
        // zero" cannot tell alignment padding from a trailing `uint8 flags` or
        // `bool is_idr` that happens to be 0 in every capture — which is
        // precisely the hidden-field hazard this block exists to rule out, and
        // a consumer written against the shipped `.msg` would silently never
        // see such a field. An exact match against the declared count does
        // rule it out, using evidence the fixtures already carry.
        assert_eq!(
            oracle.trailing_pad, oracle.declared_pad,
            "{}: {} leftover bytes but the writer declared {} bytes of \
             encapsulation padding — the difference is an UNACCOUNTED-FOR \
             FIELD the shipped .msg does not describe, not alignment",
            fx.name, oracle.trailing_pad, oracle.declared_pad
        );
        assert!(
            body[oracle.body_len - oracle.trailing_pad..]
                .iter()
                .all(|&b| b == 0),
            "{}: trailing bytes are not zero padding",
            fx.name
        );
    }
}

// ---------------------------------------------------------------------------
// THE REGRESSION ARM: the definition that failed live still fails, for the
// SAME reason — the byte budget was never loosened.
// ---------------------------------------------------------------------------

/// The upstream four-field definition must STILL be refused on real bytes,
/// still as `HostileLength`, still naming a `video*p` field.
///
/// This is the anti-tautology twin of the headline test: it proves the headline
/// passes because the DEFINITION was corrected, not because the byte-budget
/// check was weakened. If a future change "fixes" `/frontvideostream` by
/// relaxing `guard_count`, this test goes green-to-red-to-green in the wrong
/// direction — it will start passing the decode and fail here.
#[test]
fn the_upstream_ament_definition_still_refuses_real_bytes_as_hostile_length() {
    let codec = upstream_ament_codec();

    for fx in all_fixtures() {
        let err = codec
            .decode_dds_payload(QNAME, &fx.payload, 0, 0)
            .expect_err(&format!(
                "{}: the upstream four-field definition must NOT decode a real \
                 Go2 sample — if it now does, either the fixture was replaced \
                 or the codec's field walk changed",
                fx.name
            ));

        match &err {
            CdrCodecError::HostileLength {
                schema,
                field,
                claimed,
                available,
            } => {
                assert_eq!(schema, QNAME, "{}: wrong schema in error", fx.name);
                assert!(
                    field == "video360p" || field == "video180p",
                    "{}: expected the failure on a video*p sequence, got '{field}'",
                    fx.name
                );
                // The "hostile" length is H.264 payload misread as a count, so
                // it must be wildly larger than what remains.
                assert!(
                    (*claimed as usize) > *available,
                    "{}: HostileLength must claim more than remains ({claimed} vs {available})",
                    fx.name
                );
            }
            other => panic!(
                "{}: expected HostileLength (the live failure mode), got {other:?}",
                fx.name
            ),
        }
    }
}

// ---------------------------------------------------------------------------
// The not-fully-consumed gate, measured on REAL robot bytes.
// ---------------------------------------------------------------------------

/// Both directions of the consumption gate, on the real captures.
///
/// **ACCEPT** — every committed sample carries CDR trailing padding the field
/// walk never reads, and the WRITER declares exactly that count in the
/// encapsulation options (3, 2 and 3 bytes, in `all_fixtures()` order). So
/// `MAX_CDR_TRAILING_PAD` is not a guess: it is the ceiling real Go2 traffic
/// actually sits at, and a stricter allowance would refuse every
/// `/frontvideostream` sample. This is the anti-false-positive arm — the
/// gate's whole risk is refusing legitimate traffic, and these are the only
/// non-synthesized bytes available to prove it does not.
///
/// **REFUSE** — the same bytes under [`SHORT_PREFIX_DEFINITION`]. The upstream
/// ament definition fails as `HostileLength` only by luck of being MIS-TYPED
/// (see the sibling test); a merely SHORTER definition reads sane lengths
/// throughout and, before the gate existed, published a video-less frame in silence.
///
/// Both splits come from the independent [`hand_parse`] oracle, never a second
/// decode. The synthetic twin of this contract — same shape, hand-built body —
/// lives in `cerulion_core`, which cannot reach these fixtures across the
/// workspace boundary.
///
/// The measured pads are pinned per fixture against the hand-known
/// `KNOWN_PADS`, so the ACCEPT half cannot rot vacuous. A `<=` bound alone is
/// satisfied by a pad of 0 — if these fixtures were ever regenerated from a
/// writer that emitted no padding, the arm would keep passing while proving
/// nothing about the boundary, and "not a guess, it is where real traffic
/// sits" would quietly become false. Two of the three sit exactly AT
/// `MAX_CDR_TRAILING_PAD`, which is the claim that matters: the boundary case
/// is exercised by real bytes, not asserted about them.
#[test]
fn real_bytes_pass_the_consumption_gate_and_a_short_definition_is_refused() {
    let shipped = shipped_codec();
    let short = short_prefix_codec();

    /// The pad each committed capture really carries, in `all_fixtures()`
    /// order — hand-read from the bytes, and independently corroborated by the
    /// writer's own declared count (see
    /// `decoded_frame_round_trips_back_to_the_exact_cdr_body`).
    ///
    /// Pinned per FIXTURE, not as a bag of numbers: the first version of this
    /// pin carried `3, 3, 2` — the multiset, in the wrong order — copied from
    /// prose that had never been checked against the bytes. Naming each
    /// fixture is what turned that into a failing assertion instead of a
    /// second place repeating it.
    const KNOWN_PADS: [(&str, usize); 3] =
        [("360p_nonidr", 3), ("720p_nonidr", 2), ("360p_idr", 3)];

    let mut measured_pads: Vec<(&str, usize)> = Vec::new();

    for fx in all_fixtures() {
        let oracle = hand_parse(&fx.payload);
        measured_pads.push((fx.name, oracle.trailing_pad));

        // ACCEPT: the real pad is within the allowance, and the sample decodes.
        assert!(
            oracle.trailing_pad <= MAX_CDR_TRAILING_PAD,
            "{}: real robot traffic left {} unread trailing bytes, above the \
             {MAX_CDR_TRAILING_PAD} the gate allows — the allowance is too \
             strict for a conformant writer",
            fx.name,
            oracle.trailing_pad
        );
        shipped
            .decode_dds_payload(QNAME, &fx.payload, 0, 0)
            .unwrap_or_else(|e| {
                panic!(
                    "{}: a real captured sample ({} pad bytes) must decode — \
                     the consumption gate must never refuse legitimate \
                     robot traffic. Got {e:?}",
                    fx.name, oracle.trailing_pad
                )
            });

        // REFUSE: the prefix definition walks 12 bytes (u64 + u32) and leaves
        // the length prefix plus the entire access unit unread.
        let err = short
            .decode_dds_payload(QNAME, &fx.payload, 0, 0)
            .expect_err(&format!(
                "{}: a definition missing `video_data` must be REFUSED, not \
                 published with the video silently dropped",
                fx.name
            ));
        assert_eq!(
            err,
            CdrCodecError::TrailingBytes {
                schema: QNAME.to_string(),
                consumed: 12,
                remaining: oracle.body_len - 12,
            },
            "{}: wrong consumed/remaining split",
            fx.name
        );
    }

    // The ACCEPT half's anti-vacuity pin. Exact set, so a regenerated capture
    // with no padding fails loudly here instead of silently turning the `<=`
    // bound above into a tautology.
    assert_eq!(
        measured_pads,
        KNOWN_PADS.to_vec(),
        "the committed captures no longer carry the pads this arm's ACCEPT \
         claim rests on — if fixtures were regenerated, re-read the pads by \
         hand and update KNOWN_PADS (and re-check that at least one still \
         reaches MAX_CDR_TRAILING_PAD, or this arm stops proving the boundary)"
    );
    assert!(
        measured_pads
            .iter()
            .any(|&(_, pad)| pad == MAX_CDR_TRAILING_PAD),
        "no committed capture reaches MAX_CDR_TRAILING_PAD ({MAX_CDR_TRAILING_PAD}), \
         so the allowance's boundary is not exercised by real bytes: {measured_pads:?}"
    );
}

/// The mis-decode is an OFFSET error, not a genuinely hostile robot: the bytes
/// the upstream definition reads as `video360p`'s length prefix sit INSIDE the
/// H.264 access unit that the corrected definition recovers intact.
///
/// This is what makes "our definition was wrong" the diagnosis rather than
/// "the robot sent garbage".
#[test]
fn the_hostile_length_bytes_are_h264_payload_read_at_the_wrong_offset() {
    for fx in all_fixtures() {
        let oracle = hand_parse(&fx.payload);
        let body = &fx.payload[4..];

        // Replay the upstream definition's walk by hand: time_frame @0,
        // then video720p's count @8 — which is really `video_height`.
        let misread_720p_count = oracle.video_height as usize;
        // video720p's "payload" starts at 12; the next count is read at the
        // next 4-aligned offset after it.
        let next = 12 + misread_720p_count;
        // Round UP to the next multiple of 4 (CDR aligns a u32 to 4 bytes).
        let aligned = next.next_multiple_of(4);
        assert!(
            aligned + 4 <= body.len(),
            "{}: the misread offset should still be inside this body",
            fx.name
        );
        let misread_len = u32::from_le_bytes(body[aligned..aligned + 4].try_into().unwrap());

        // That offset is inside the access unit the CORRECT layout recovers.
        assert!(
            aligned >= 16 && aligned + 4 <= 16 + oracle.video_data.len(),
            "{}: misread offset {aligned} should land inside the H.264 access \
             unit (16..{})",
            fx.name,
            16 + oracle.video_data.len()
        );
        // And it is nonsense as a length — which is exactly what the byte
        // budget reported.
        assert!(
            misread_len as usize > body.len(),
            "{}: the misread length {misread_len} should exceed the whole \
             {}-byte body",
            fx.name,
            body.len()
        );
    }
}

// ---------------------------------------------------------------------------
// The payload really is decodable H.264 — the fact the JPEG transcode and the
// (separate) desk-side H.264 decoder both depend on.
// ---------------------------------------------------------------------------

/// Every recovered access unit is Annex-B start-code delimited, and the IDR
/// fixture carries SPS + PPS + IDR (so a decoder can start from it).
#[test]
fn recovered_video_data_is_annexb_h264_and_the_idr_fixture_is_self_starting() {
    let codec = shipped_codec();
    let walker = walker_over_shipped();

    for fx in all_fixtures() {
        let frame = codec
            .decode_dds_payload(QNAME, &fx.payload, 0, 0)
            .unwrap_or_else(|e| panic!("{}: decode: {e}", fx.name));
        let fv = walker.walk_by_hash(&frame).expect("walk");
        let data = match fv.field("video_data") {
            Some(FrameValueKind::Bytes(b)) => *b,
            other => panic!("{}: video_data is not Bytes: {other:?}", fx.name),
        };

        assert!(
            data.len() > 4,
            "{}: an access unit is more than a start code",
            fx.name
        );
        assert_eq!(
            &data[..4],
            &ANNEXB_START_CODE,
            "{}: access unit does not open with the Annex-B start code",
            fx.name
        );

        let nal_types = annexb_nal_types(data);
        assert!(
            !nal_types.is_empty(),
            "{}: no NAL units found in the access unit",
            fx.name
        );

        if fx.is_idr {
            for (ty, label) in [(7u8, "SPS"), (8, "PPS"), (5, "IDR slice")] {
                assert!(
                    nal_types.contains(&ty),
                    "{}: an IDR access unit must carry a {label} (nal_unit_type \
                     {ty}); saw {nal_types:?}",
                    fx.name
                );
            }
        } else {
            assert!(
                nal_types.contains(&1),
                "{}: a non-IDR access unit must carry a non-IDR slice \
                 (nal_unit_type 1); saw {nal_types:?}",
                fx.name
            );
        }
    }
}

/// The `video_height` field agrees with the resolution the H.264 SPS declares —
/// the evidence behind the field's name. Only the IDR fixtures carry an SPS.
#[test]
fn video_height_matches_the_resolution_declared_by_the_h264_sps() {
    let codec = shipped_codec();
    let walker = walker_over_shipped();

    let mut checked = 0usize;
    for fx in all_fixtures() {
        let frame = codec
            .decode_dds_payload(QNAME, &fx.payload, 0, 0)
            .unwrap_or_else(|e| panic!("{}: decode: {e}", fx.name));
        let fv = walker.walk_by_hash(&frame).expect("walk");
        let height = match fv.field("video_height") {
            Some(FrameValueKind::U32(v)) => *v,
            other => panic!("{}: video_height is not a U32: {other:?}", fx.name),
        };
        let data = match fv.field("video_data") {
            Some(FrameValueKind::Bytes(b)) => *b,
            other => panic!("{}: video_data is not Bytes: {other:?}", fx.name),
        };

        if let Some(sps) = first_nal_of_type(data, 7) {
            let sps_height = sps_frame_height(sps).unwrap_or_else(|| {
                panic!("{}: could not read a frame height out of the SPS", fx.name)
            });
            assert_eq!(
                height, sps_height,
                "{}: video_height ({height}) disagrees with the SPS-declared \
                 frame height ({sps_height}) — the field is not the rendition \
                 height after all, and the .msg's field NAME is wrong",
                fx.name
            );
            checked += 1;
        }
    }
    assert!(
        checked > 0,
        "no fixture carried an SPS — this test asserted nothing. Restore the \
         IDR fixture (go2_frontvideostream_360p_idr.cdr)."
    );
}

/// The two renditions are distinguishable on the one topic — the reason
/// `video_height` has to be on the wire at all.
#[test]
fn the_two_renditions_are_told_apart_by_video_height() {
    let codec = shipped_codec();
    let walker = walker_over_shipped();

    let mut heights: Vec<u32> = Vec::new();
    for fx in all_fixtures() {
        let frame = codec
            .decode_dds_payload(QNAME, &fx.payload, 0, 0)
            .expect("decode");
        let fv = walker.walk_by_hash(&frame).expect("walk");
        match fv.field("video_height") {
            Some(FrameValueKind::U32(v)) => heights.push(*v),
            other => panic!("{}: video_height is not a U32: {other:?}", fx.name),
        }
    }
    heights.sort_unstable();
    heights.dedup();
    assert_eq!(
        heights,
        vec![360, 720],
        "the committed fixtures should cover exactly the two observed \
         renditions (360 and 720); got {heights:?}"
    );
}

// ---------------------------------------------------------------------------
// THE SHIPPED CONFIG: graphs/go2.bridge.yaml must actually carry the camera,
// and its OWN codec must decode real bytes.
// ---------------------------------------------------------------------------

/// The end-to-end wiring pin: load the SHIPPED `graphs/go2.bridge.yaml`, build
/// the codec exactly as the pump does (`BridgeConfig::runtime_codec`, seeded
/// from the config's own `msg_dirs`), and decode a REAL captured sample through
/// it.
///
/// This is what closes the loop. The tests above prove the corrected schema
/// decodes when handed directly to a codec; this one proves the shipped config
/// actually reaches that schema — the `msg_dirs` entry is present, it resolves
/// from the config file's directory, and the store copy (not the ament copy)
/// is what the codec ends up with. Deleting `msg_dirs`, moving the store, or
/// dropping the mapping each fails here.
#[test]
fn the_shipped_bridge_config_carries_the_camera_and_its_own_codec_decodes_real_bytes() {
    let cfg_path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../graphs/go2.bridge.yaml");
    let text = std::fs::read_to_string(&cfg_path)
        .unwrap_or_else(|e| panic!("reading {}: {e}", cfg_path.display()));
    let cfg = dds_bridge::config::BridgeConfig::from_yaml(&text, &cfg_path.display().to_string())
        .unwrap_or_else(|e| panic!("the shipped go2.bridge.yaml must validate: {e}"));

    // The camera mapping is present, and points at the topic the JPEG
    // transcode + the desk-side decoder both consume.
    let cam = cfg
        .mappings
        .iter()
        .find(|m| m.dds_topic == "/frontvideostream")
        .expect(
            "graphs/go2.bridge.yaml must map /frontvideostream — it is the only \
             reachable source of Go2 camera imagery",
        );
    assert_eq!(cam.ros_type, "unitree_go/Go2FrontVideoData");
    assert_eq!(cam.cerulion_topic, "/go2/camera/h264");
    // It must ride the RAW generic codec (no typed registry port exists for
    // this type), which is what makes the schema store load-bearing. Asserted
    // through the production accessor the pump itself uses.
    assert!(
        cfg.raw_mappings()
            .iter()
            .any(|m| m.dds_topic == "/frontvideostream"),
        "the camera mapping must be in the config's RAW set — that is the path \
         that consults the schema store"
    );

    // max_slice_len must cover the largest access unit MEASURED on a real
    // robot. An over-cap frame is DROPPED, not truncated
    // (the over-cap loan fails, the raw route's publish returns Err, the sample
    // is discarded and the wire sequence is not burned) — counted in
    // `raw_route_failures` and flood-latched to one `warn!` per regime with
    // repeats at `debug!`. So it is never silent and never a corrupted
    // half-frame; but it IS a lost frame, and for an IDR that costs the
    // desk-side decoder its whole GOP. Loudness is no substitute for headroom.
    //
    // TWO floors, because the committed fixtures are NOT the measured maximum:
    // the largest capture worth committing is ~23 KB, while the largest access
    // unit actually observed is MEASURED_MAX_ACCESS_UNIT_BYTES. Asserting only
    // against the fixtures would let `max_slice_len: 32768` pass while every
    // real 720p IDR was dropped at run time.
    let slice = dds_bridge::config::BridgeConfig::raw_slice_len(cam);
    let largest_fixture = all_fixtures()
        .iter()
        .map(|f| f.payload.len())
        .max()
        .expect("fixtures");
    assert!(
        slice.get() as usize > largest_fixture,
        "max_slice_len ({}) must exceed the largest COMMITTED fixture \
         ({largest_fixture} bytes)",
        slice.get()
    );
    assert!(
        slice.get() as usize > MEASURED_MAX_ACCESS_UNIT_BYTES,
        "max_slice_len ({}) must exceed the largest access unit MEASURED on \
         a real robot ({MEASURED_MAX_ACCESS_UNIT_BYTES} \
         bytes) — a cap below that drops every 720p IDR",
        slice.get()
    );

    // THE pin: the config's own codec — the same one the pump builds — knows
    // the type and decodes real captured bytes.
    let codec = cfg.runtime_codec();
    assert!(
        codec.knows(QNAME),
        "the shipped config's codec cannot resolve {QNAME}. The `msg_dirs` \
         entry pointing at the workspace schema store is missing or does not \
         resolve — without it the raw route cannot transcode and every camera \
         frame is dropped."
    );

    let walker = walker_over_shipped();
    for (i, fx) in all_fixtures().iter().enumerate() {
        let oracle = hand_parse(&fx.payload);
        let frame = codec
            .decode_dds_payload(QNAME, &fx.payload, i as u32, 0)
            .unwrap_or_else(|e| {
                panic!(
                    "{}: the SHIPPED config's codec failed on real captured \
                     bytes: {e}",
                    fx.name
                )
            });
        let fv = walker.walk_by_hash(&frame).expect("walk");
        match fv.field("video_data") {
            Some(FrameValueKind::Bytes(b)) => assert_eq!(
                *b,
                oracle.video_data.as_slice(),
                "{}: video_data through the shipped config disagrees with the \
                 hand oracle",
                fx.name
            ),
            other => panic!("{}: video_data is not Bytes: {other:?}", fx.name),
        }
    }
}

// ---------------------------------------------------------------------------
// Minimal hand-written Annex-B / SPS readers (oracle side — deliberately
// independent of anything under test).
// ---------------------------------------------------------------------------

/// Offsets of each 4-byte Annex-B start code in `d`.
fn annexb_starts(d: &[u8]) -> Vec<usize> {
    let mut out = Vec::new();
    let mut i = 0usize;
    while i + 4 <= d.len() {
        if d[i..i + 4] == ANNEXB_START_CODE {
            out.push(i);
            i += 4;
        } else {
            i += 1;
        }
    }
    out
}

/// `nal_unit_type` of every NAL unit in an Annex-B byte stream.
fn annexb_nal_types(d: &[u8]) -> Vec<u8> {
    annexb_starts(d)
        .into_iter()
        .filter(|s| s + 4 < d.len())
        .map(|s| d[s + 4] & 0x1f)
        .collect()
}

/// The first NAL unit of `want` type, INCLUDING its 1-byte header.
fn first_nal_of_type(d: &[u8], want: u8) -> Option<&[u8]> {
    let starts = annexb_starts(d);
    for (idx, &s) in starts.iter().enumerate() {
        let nal_begin = s + 4;
        if nal_begin >= d.len() {
            continue;
        }
        if d[nal_begin] & 0x1f == want {
            let end = starts.get(idx + 1).copied().unwrap_or(d.len());
            return Some(&d[nal_begin..end]);
        }
    }
    None
}

/// Strip H.264 emulation-prevention bytes (`00 00 03` → `00 00`).
fn unescape_rbsp(b: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(b.len());
    let mut i = 0usize;
    while i < b.len() {
        if i + 2 < b.len() && b[i] == 0 && b[i + 1] == 0 && b[i + 2] == 3 {
            out.push(0);
            out.push(0);
            i += 3;
        } else {
            out.push(b[i]);
            i += 1;
        }
    }
    out
}

/// A minimal MSB-first bit reader for Exp-Golomb SPS fields.
struct BitReader<'a> {
    b: &'a [u8],
    pos: usize,
}

impl<'a> BitReader<'a> {
    fn new(b: &'a [u8]) -> Self {
        Self { b, pos: 0 }
    }
    fn u(&mut self, n: usize) -> Option<u32> {
        let mut v = 0u32;
        for _ in 0..n {
            let byte = *self.b.get(self.pos >> 3)?;
            v = (v << 1) | u32::from((byte >> (7 - (self.pos & 7))) & 1);
            self.pos += 1;
        }
        Some(v)
    }
    fn ue(&mut self) -> Option<u32> {
        let mut zeros = 0usize;
        while self.u(1)? == 0 {
            zeros += 1;
            if zeros > 31 {
                return None;
            }
        }
        if zeros == 0 {
            return Some(0);
        }
        Some((1 << zeros) - 1 + self.u(zeros)?)
    }
    fn se(&mut self) -> Option<i32> {
        let k = self.ue()?;
        // H.264 se(v) mapping: odd codeNum -> +(k+1)/2, even -> -(k/2).
        Some(if k % 2 == 1 {
            k.div_ceil(2) as i32
        } else {
            -((k / 2) as i32)
        })
    }
}

/// Decode `frame_height` from an H.264 SPS NAL (header byte included).
///
/// Deliberately minimal: it handles the Go2's High-profile,
/// `frame_mbs_only_flag = 1`, no-scaling-matrix SPS. Returns `None` on
/// anything it cannot parse exactly, and the caller panics — a silently
/// skipped check would make the test vacuous.
fn sps_frame_height(sps_nal: &[u8]) -> Option<u32> {
    if sps_nal.first().map(|b| b & 0x1f) != Some(7) || sps_nal.len() < 5 {
        return None;
    }
    let rbsp = unescape_rbsp(&sps_nal[1..]);
    let profile_idc = *rbsp.first()? as u32;
    let mut r = BitReader::new(rbsp.get(3..)?);

    r.ue()?; // seq_parameter_set_id
    let chroma_format_idc = if matches!(
        profile_idc,
        100 | 110 | 122 | 244 | 44 | 83 | 86 | 118 | 128 | 138 | 139 | 134 | 135
    ) {
        let cf = r.ue()?;
        if cf == 3 {
            r.u(1)?; // separate_colour_plane_flag
        }
        r.ue()?; // bit_depth_luma_minus8
        r.ue()?; // bit_depth_chroma_minus8
        r.u(1)?; // qpprime_y_zero_transform_bypass_flag
        if r.u(1)? == 1 {
            // seq_scaling_matrix_present_flag — not handled; refuse rather
            // than mis-read.
            return None;
        }
        cf
    } else {
        1
    };

    r.ue()?; // log2_max_frame_num_minus4
    let poc_type = r.ue()?;
    match poc_type {
        0 => {
            r.ue()?;
        }
        1 => {
            r.u(1)?;
            r.se()?;
            r.se()?;
            let n = r.ue()?;
            for _ in 0..n {
                r.se()?;
            }
        }
        _ => {}
    }
    r.ue()?; // max_num_ref_frames
    r.u(1)?; // gaps_in_frame_num_value_allowed_flag
    r.ue()?; // pic_width_in_mbs_minus1
    let height_map_units = r.ue()?.checked_add(1)?;
    let frame_mbs_only = r.u(1)?;
    if frame_mbs_only == 0 {
        r.u(1)?; // mb_adaptive_frame_field_flag
    }
    r.u(1)?; // direct_8x8_inference_flag

    let (mut top, mut bottom) = (0u32, 0u32);
    if r.u(1)? == 1 {
        r.ue()?; // frame_crop_left_offset
        r.ue()?; // frame_crop_right_offset
        top = r.ue()?;
        bottom = r.ue()?;
    }

    let sub_height_c = if chroma_format_idc == 1 { 2 } else { 1 };
    let crop_unit_y = sub_height_c * (2 - frame_mbs_only);
    let raw_height = height_map_units
        .checked_mul(16)?
        .checked_mul(2 - frame_mbs_only)?;
    raw_height.checked_sub(crop_unit_y.checked_mul(top.checked_add(bottom)?)?)
}
