// SPDX-License-Identifier: AGPL-3.0-only
//! The desk-side H.264 path — classification, the SPS-keyed rendition
//! demux, the keyframe gate, and the `rerun::VideoStream` render arm.
//!
//! # What is real here, and what is hand-built
//!
//! [`GO2_REAL_SPS`] / [`GO2_REAL_PPS`] / [`GO2_REAL_IDR_HEAD`] /
//! [`GO2_REAL_720P_SLICE_HEAD`] are VERBATIM bytes lifted from the committed
//! captures in `examples/go2/nodes/dds_bridge/tests/fixtures/` — real DDS payloads
//! taken off a live Unitree Go2 with an rclpy `raw=True` subscription (see the
//! provenance section of `frontvideostream_wire_test.rs`). Their exact source
//! offsets are recorded beside each constant. The IDR and non-IDR slices are
//! TRUNCATED after their first bytes because nothing in this module parses slice
//! payload: the classifier reads NAL headers, the demux reads the SPS. Every
//! header byte and the entire SPS are intact.
//!
//! The second rendition's SPS is BUILT ([`build_sps`]) rather than captured,
//! because the committed 720p capture is a non-IDR access unit and therefore
//! carries no parameter set. It is a real conforming Baseline SPS — `h264_reader`
//! parses it, and the test asserts the dimensions it recovers against the
//! macroblock counts that were encoded, so the builder cannot silently agree with
//! a broken parse.
//!
//! # Why the assertions are not self-compares
//!
//! Dimensions are checked against INDEPENDENT facts: the real SPS against the
//! `video_height` the same captured sample reports in its own field (640x360),
//! the built SPS against the macroblock counts its builder was handed. Routing is
//! checked against hand-written expected entity paths and hand-written per-stream
//! sample counts, never against a second run of the code under test (the one
//! two-run comparison is the determinism pin, which ALSO compares both runs to a
//! hand oracle).
//!
//! No iceoryx2, no transport — pure functions plus a rerun `memory()` sink.
//! Parallel-safe.

use cerulion_core::codegen::layout::{LayoutResolver, WireLayout};
use cerulion_core::codegen::{
    parse_rosmsg, FrameValue, FrameValueKind, FrameWalker, MessageSchema, NamedValue,
};
use cerulion_core::message::ShmMessage;
use cerulion_core::shm_runtime::write_offset_entry;
use cerulion_core::wire::WireHeader;
use cerulion_viz::sink::{classify_frame, dispatch_frame, ArchetypeKind, SinkState};
use cerulion_viz::video::{
    classify_h264_payload, scan_annex_b, sps_dimensions, video_entity, StreamKey, VideoDemux,
    VideoReject, VideoRoute, DISCRIMINATOR_CONFIRM_OBSERVATIONS, MAX_DISCRIMINATOR_VALUES,
    MAX_STREAMS_PER_INPUT, NEVER_KEYFRAMED_WARN_AFTER,
};
use native_ros2_messages::sensor_msgs::CompressedImage;

// ---------------------------------------------------------------------------
// REAL Go2 bytes (provenance in the module docs)
// ---------------------------------------------------------------------------

/// The Sequence Parameter Set NAL from the real 360p IDR capture — bytes
/// `0x18..0x32` of `go2_frontvideostream_360p_idr.cdr` (the access unit begins at
/// file offset 0x14 with a 4-byte start code). Complete and unmodified, emulation
/// prevention bytes included.
const GO2_REAL_SPS: &[u8] = &[
    0x67, 0x64, 0x10, 0x28, 0xac, 0x1b, 0x1a, 0xa0, 0xa0, 0x2f, 0xf9, 0x61, 0x00, 0x00, 0x03, 0x00,
    0x01, 0x00, 0x00, 0x03, 0x00, 0x3c, 0x8f, 0x08, 0x84, 0x6a,
];

/// The Picture Parameter Set NAL from the same capture — bytes `0x36..0x3b`.
/// Complete and unmodified.
const GO2_REAL_PPS: &[u8] = &[0x68, 0xee, 0x31, 0xb2, 0x1b];

/// The first 16 bytes of the IDR slice NAL from the same capture (file offset
/// 0x3f onward). TRUNCATED: slice payload is never parsed here.
const GO2_REAL_IDR_HEAD: &[u8] = &[
    0x65, 0xb8, 0x00, 0x01, 0x40, 0x00, 0x01, 0x3f, 0x73, 0x86, 0xb4, 0x55, 0xf9, 0xf5, 0x02, 0x04,
];

/// The first 20 bytes of the non-IDR slice NAL from the real 720p capture
/// (`go2_frontvideostream_720p_nonidr.cdr`, file offset 0x18 onward — that whole
/// access unit is ONE NAL). TRUNCATED, same reason.
const GO2_REAL_720P_SLICE_HEAD: &[u8] = &[
    0x41, 0xe0, 0x02, 0x80, 0x05, 0x02, 0xff, 0xbd, 0xca, 0x78, 0x48, 0x45, 0x3a, 0x36, 0x25, 0x30,
    0x2f, 0x2d, 0x10, 0xde,
];

/// The `video_height` the real 360p capture reports in its own field — the
/// INDEPENDENT fact the SPS parse is cross-checked against.
const GO2_REAL_360P_REPORTED_HEIGHT: u32 = 360;

/// The four-byte Annex-B start code.
const SC4: &[u8] = &[0, 0, 0, 1];

// ---------------------------------------------------------------------------
// Access-unit assembly + a hand SPS builder
// ---------------------------------------------------------------------------

/// Concatenate NALs into an Annex-B buffer with 4-byte start codes.
fn annex_b(nals: &[&[u8]]) -> Vec<u8> {
    let mut out = Vec::new();
    for n in nals {
        out.extend_from_slice(SC4);
        out.extend_from_slice(n);
    }
    out
}

/// The REAL Go2 360p keyframe access unit (SPS + PPS + truncated IDR slice).
fn go2_real_keyframe_au() -> Vec<u8> {
    annex_b(&[GO2_REAL_SPS, GO2_REAL_PPS, GO2_REAL_IDR_HEAD])
}

/// A minimal MSB-first bit writer, just enough for a Baseline SPS.
struct BitWriter {
    bits: Vec<bool>,
}

impl BitWriter {
    fn new() -> Self {
        Self { bits: Vec::new() }
    }
    fn u(&mut self, value: u32, n: u32) {
        for i in (0..n).rev() {
            self.bits.push((value >> i) & 1 == 1);
        }
    }
    /// Exp-Golomb `ue(v)`: `code = value + 1`, written as `len-1` zeros then the
    /// code's bits (ITU-T H.264 §9.1).
    fn ue(&mut self, value: u32) {
        let code = value + 1;
        let len = 32 - code.leading_zeros();
        self.u(0, len - 1);
        self.u(code, len);
    }
    /// `rbsp_trailing_bits()`: a stop bit then zero padding to a byte boundary.
    fn finish(mut self) -> Vec<u8> {
        self.bits.push(true);
        while !self.bits.len().is_multiple_of(8) {
            self.bits.push(false);
        }
        self.bits
            .chunks(8)
            .map(|c| c.iter().fold(0u8, |acc, b| (acc << 1) | u8::from(*b)))
            .collect()
    }
}

/// Insert emulation-prevention bytes: any `00 00 00|01|02|03` in the RBSP becomes
/// `00 00 03 xx` (ITU-T H.264 §7.4.1.1). Without this a parameter set carrying two
/// consecutive zero bytes could be mistaken for a start code.
fn rbsp_to_ebsp(rbsp: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(rbsp.len());
    let mut zeros = 0usize;
    for &b in rbsp {
        if zeros >= 2 && b <= 3 {
            out.push(3);
            zeros = 0;
        }
        out.push(b);
        zeros = if b == 0 { zeros + 1 } else { 0 };
    }
    out
}

/// Build a conforming Baseline-profile SPS NAL for a macroblock-aligned picture.
///
/// `width_mbs` / `height_mbs` are macroblock counts, so the decoded picture is
/// `16 * width_mbs` by `16 * height_mbs`. Baseline (`profile_idc = 66`) is chosen
/// because its SPS syntax has no chroma-format or scaling-list branch, keeping this
/// builder small enough to audit by eye.
fn build_sps(sps_id: u32, width_mbs: u32, height_mbs: u32) -> Vec<u8> {
    let mut w = BitWriter::new();
    w.u(66, 8); // profile_idc: Baseline
    w.u(0, 8); // constraint_set flags + reserved_zero_2bits
    w.u(30, 8); // level_idc 3.0
    w.ue(sps_id); // seq_parameter_set_id
    w.ue(0); // log2_max_frame_num_minus4
    w.ue(2); // pic_order_cnt_type 2 (no further fields)
    w.ue(1); // max_num_ref_frames
    w.u(0, 1); // gaps_in_frame_num_value_allowed_flag
    w.ue(width_mbs - 1); // pic_width_in_mbs_minus1
    w.ue(height_mbs - 1); // pic_height_in_map_units_minus1
    w.u(1, 1); // frame_mbs_only_flag
    w.u(1, 1); // direct_8x8_inference_flag
    w.u(0, 1); // frame_cropping_flag
    w.u(0, 1); // vui_parameters_present_flag

    let mut nal = vec![0x67u8]; // forbidden 0, nal_ref_idc 3, type 7 (SPS)
    nal.extend_from_slice(&rbsp_to_ebsp(&w.finish()));
    nal
}

/// A hand-built non-IDR coded-slice NAL: header `0x41` (ref_idc 2, type 1) plus
/// opaque payload. Slice content is never parsed, so a marker byte suffices — and
/// it makes each synthetic access unit distinguishable in a byte comparison.
fn slice_nal(marker: u8) -> Vec<u8> {
    vec![0x41, marker, 0x02, 0x80]
}

/// A hand-built IDR coded-slice NAL: header `0x65` (ref_idc 3, type 5).
fn idr_nal(marker: u8) -> Vec<u8> {
    vec![0x65, marker, 0xb8, 0x00]
}

// ---------------------------------------------------------------------------
// FrameValue construction (the classifier + demux take decoded values)
// ---------------------------------------------------------------------------

fn fv<'a>(schema: &str, fields: Vec<(&'a str, FrameValueKind<'a>)>) -> FrameValue<'a> {
    FrameValue {
        schema_name: schema.to_string(),
        fields: fields
            .into_iter()
            .map(|(name, value)| NamedValue {
                name: name.to_string(),
                value,
            })
            .collect(),
    }
}

/// The Go2's corrected `/frontvideostream` message shape:
/// `{uint64 time_frame, uint32 video_height, uint8[] video_data}`.
fn go2_video_fv<'a>(time_frame: u64, video_height: u32, au: &'a [u8]) -> FrameValue<'a> {
    fv(
        "unitree_go/Go2FrontVideoData",
        vec![
            ("time_frame", FrameValueKind::U64(time_frame)),
            ("video_height", FrameValueKind::U32(video_height)),
            ("video_data", FrameValueKind::Bytes(au)),
        ],
    )
}

// ===========================================================================
// 1. The classifier: Annex-B CONTENT is the gate
// ===========================================================================

#[test]
fn a_real_go2_access_unit_classifies_as_video_and_its_sps_gives_the_reported_resolution() {
    let au = go2_real_keyframe_au();
    let frame = go2_video_fv(0x943f_c3bf, GO2_REAL_360P_REPORTED_HEIGHT, &au);

    assert_eq!(
        classify_frame(&frame),
        ArchetypeKind::VideoStream,
        "real Go2 H.264 bytes must classify as video"
    );

    let payload = classify_h264_payload(&frame).expect("classified");
    assert_eq!(payload.field, "video_data");
    assert_eq!(
        payload.bytes,
        &au[..],
        "the payload is handed over verbatim"
    );

    // Three NALs, in the order the robot emitted them.
    let kinds: Vec<u8> = payload.access_unit.nals.iter().map(|n| n.kind).collect();
    assert_eq!(kinds, vec![7, 8, 5], "SPS, PPS, IDR slice");
    assert!(
        payload.access_unit.is_self_contained_keyframe(),
        "an IDR carrying its own SPS is a keyframe by rerun's H.264 definition"
    );

    // Cross-check against the INDEPENDENT fact in the same captured sample: its
    // own `video_height` field reads 360.
    let sps = payload.access_unit.sps().expect("SPS present");
    assert_eq!(
        sps_dimensions(sps.bytes),
        Some((640, GO2_REAL_360P_REPORTED_HEIGHT)),
        "the SPS must decode to the resolution the robot's own field reports"
    );
}

#[test]
fn a_real_go2_non_keyframe_classifies_as_video_but_is_not_a_keyframe() {
    // The OTHER real capture: a 720p access unit that is ONE non-IDR slice with no
    // parameter set. It must classify (it is genuine H.264) yet must NOT be flagged
    // a keyframe — a decoder handed it with no prior state has nothing to configure
    // from, which is exactly why the sub-stream gate exists.
    let au = annex_b(&[GO2_REAL_720P_SLICE_HEAD]);
    let frame = go2_video_fv(0x9441_cc94, 720, &au);
    assert_eq!(classify_frame(&frame), ArchetypeKind::VideoStream);

    let payload = classify_h264_payload(&frame).expect("classified");
    assert_eq!(
        payload
            .access_unit
            .nals
            .iter()
            .map(|n| n.kind)
            .collect::<Vec<_>>(),
        vec![1],
        "one non-IDR coded slice"
    );
    assert!(payload.access_unit.sps().is_none(), "no parameter set");
    assert!(!payload.access_unit.is_self_contained_keyframe());
}

#[test]
fn a_three_byte_start_code_stream_is_accepted() {
    // Annex-B permits `00 00 01` as well as `00 00 00 01`, and encoders mix them
    // (a 4-byte code is a 3-byte code with one leading zero of padding).
    let sps = build_sps(0, 40, 24);
    let mut au = vec![0, 0, 1];
    au.extend_from_slice(&sps);
    au.extend_from_slice(&[0, 0, 1]);
    au.extend_from_slice(&idr_nal(0x11));

    let scan = scan_annex_b(&au).expect("3-byte start codes are Annex-B");
    assert_eq!(
        scan.nals.iter().map(|n| n.kind).collect::<Vec<_>>(),
        vec![7, 5]
    );
    assert!(scan.is_self_contained_keyframe());
}

#[test]
fn mixed_start_code_widths_and_trailing_zero_padding_do_not_corrupt_a_nal() {
    // A 4-byte start code preceded by extra zero padding is legal; those zeros
    // belong to NEITHER NAL. Getting this wrong silently appends `00` bytes to the
    // previous NAL, which would corrupt an SPS handed to the parser.
    let sps = build_sps(0, 80, 45);
    let mut au = Vec::new();
    au.extend_from_slice(SC4);
    au.extend_from_slice(&sps);
    au.extend_from_slice(&[0, 0, 0, 0, 1]); // one EXTRA leading zero
    au.extend_from_slice(&idr_nal(0x22));

    let scan = scan_annex_b(&au).expect("legal Annex-B");
    assert_eq!(scan.nals.len(), 2);
    assert_eq!(
        scan.nals[0].bytes,
        &sps[..],
        "the SPS must come back byte-identical — trailing start-code padding is not \
         part of it"
    );
    assert_eq!(
        sps_dimensions(scan.nals[0].bytes),
        Some((1280, 720)),
        "and it must still parse"
    );
}

#[test]
fn a_pointcloud2_data_blob_is_not_video() {
    // A real cloud payload: packed little-endian f32 xyz triples. Nothing here
    // opens with a start code, and the classifier must say so.
    let points: [[f32; 3]; 4] = [
        [0.0, 0.0, 0.0],
        [1.5, -2.25, 0.75],
        [-0.125, 3.0, 12.5],
        [4.0, 4.0, 4.0],
    ];
    let mut blob = Vec::new();
    for p in points {
        for c in p {
            blob.extend_from_slice(&c.to_le_bytes());
        }
    }
    assert!(
        scan_annex_b(&blob).is_none(),
        "a packed float cloud is not an Annex-B access unit"
    );
    let frame = fv(
        "sensor_msgs/PointCloud2",
        vec![
            ("point_step", FrameValueKind::U32(12)),
            ("width", FrameValueKind::U32(4)),
            ("height", FrameValueKind::U32(1)),
            ("data", FrameValueKind::Bytes(&blob)),
        ],
    );
    assert_eq!(
        classify_frame(&frame),
        ArchetypeKind::Points3D,
        "a cloud must stay a cloud"
    );
}

#[test]
fn a_jpeg_blob_is_not_video_and_still_classifies_as_an_image() {
    // The JPEG SOI + APP0 marker sequence — the exact bytes a `CompressedImage`
    // carries. `FF D8 FF` is not a start code, so the content gate cannot fire.
    let jpeg: &[u8] = &[
        0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, b'J', b'F', b'I', b'F', 0x00, 0x01, 0x01, 0x00,
    ];
    assert!(scan_annex_b(jpeg).is_none());
    let frame = fv(
        "sensor_msgs/CompressedImage",
        vec![
            ("format", FrameValueKind::Str("jpeg")),
            ("data", FrameValueKind::Bytes(jpeg)),
        ],
    );
    assert_eq!(
        classify_frame(&frame),
        ArchetypeKind::Image,
        "the JPEG path must be untouched — H.264 adds a route, it does not steal one"
    );
}

#[test]
fn a_blob_that_merely_starts_with_a_start_code_is_rejected_on_its_nal_headers() {
    // THE sharp false-positive control. The first SEVEN open with a real start
    // code — the cheap half of the gate — and are rejected by the header rules
    // alone; the last two are the degenerate inputs (nothing, and a start code
    // with no NAL after it), which have no header to judge and must not be
    // accepted on that account.
    let cases: &[(&str, &[u8])] = &[
        // forbidden_zero_bit set.
        ("forbidden_zero_bit", &[0, 0, 0, 1, 0xE5, 0x11, 0x22]),
        // nal_unit_type 0: "unspecified", never in a conforming stream.
        ("type 0 unspecified", &[0, 0, 0, 1, 0x60, 0x11, 0x22]),
        // nal_unit_type 28: RTP FU-A fragmentation, a transport framing.
        ("type 28 RTP FU-A", &[0, 0, 0, 1, 0x7C, 0x11, 0x22]),
        // An SPS with nal_ref_idc 0 — the spec forbids it (parameter sets are
        // always reference data).
        ("SPS with ref_idc 0", &[0, 0, 0, 1, 0x07, 0x11, 0x22]),
        // SEI with nal_ref_idc != 0 — forbidden the other way.
        ("SEI with ref_idc 3", &[0, 0, 0, 1, 0x66, 0x11, 0x22]),
        // A single valid-looking header byte with no payload byte after it.
        ("header with no payload", &[0, 0, 0, 1, 0x41]),
        // Only parameter sets, no coded picture and no SPS.
        ("PPS only", &[0, 0, 0, 1, 0x68, 0xee, 0x31]),
        // Empty, and a bare start code.
        ("empty", &[]),
        ("bare start code", &[0, 0, 0, 1]),
    ];
    for (name, bytes) in cases {
        assert!(
            scan_annex_b(bytes).is_none(),
            "{name}: must NOT scan as an access unit"
        );
    }
}

#[test]
fn a_start_code_buried_inside_a_blob_does_not_make_it_an_access_unit() {
    // The gate requires the start code AT OFFSET 0, because a Cerulion topic
    // carries ONE access unit per message. Scanning for a start code ANYWHERE
    // would turn any sufficiently large binary blob into a candidate — and the
    // packed-float and JPEG controls cannot catch that relaxation, because
    // neither happens to contain a `00 00 01` at all. This one does: a real Go2
    // keyframe pasted into the middle of an otherwise unrelated payload.
    let mut buried = vec![0xDE, 0xAD, 0xBE, 0xEF, 0x7F, 0x11];
    buried.extend_from_slice(&go2_real_keyframe_au());
    assert!(
        scan_annex_b(&buried).is_none(),
        "a start code found mid-buffer is not an access unit — the unit must OPEN with one"
    );
    // The anti-tautology half: the very same suffix, alone, DOES scan.
    assert!(scan_annex_b(&buried[6..]).is_some());
}

#[test]
fn a_valid_first_nal_does_not_excuse_a_broken_later_one() {
    // The gate validates EVERY unit, not just the first. A blob that opens
    // convincingly and then contains a `00 00 01` followed by nonsense is not an
    // access unit.
    let mut au = Vec::new();
    au.extend_from_slice(SC4);
    au.extend_from_slice(&build_sps(0, 40, 24));
    au.extend_from_slice(SC4);
    au.extend_from_slice(&[0xF0, 0x11, 0x22]); // forbidden_zero_bit set
    assert!(scan_annex_b(&au).is_none());
}

#[test]
fn the_field_name_is_an_accelerator_not_a_requirement() {
    let au = go2_real_keyframe_au();
    let jpeg: &[u8] = &[0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10];

    // (a) A hinted name holding NON-video bytes is rejected: the name buys nothing.
    let hinted_but_jpeg = fv(
        "acme/Camera",
        vec![("video_data", FrameValueKind::Bytes(jpeg))],
    );
    assert!(
        classify_h264_payload(&hinted_but_jpeg).is_none(),
        "a field CALLED video_data holding JPEG is not video"
    );

    // (b) An UNHINTED name holding Annex-B is accepted: the name is not required.
    let unhinted_but_video = fv(
        "acme/UnseenRobotCamera",
        vec![("blorp", FrameValueKind::Bytes(&au))],
    );
    let p = classify_h264_payload(&unhinted_but_video).expect("content is the gate");
    assert_eq!(p.field, "blorp");
    assert_eq!(
        classify_frame(&unhinted_but_video),
        ArchetypeKind::VideoStream,
        "a never-seen robot's camera renders with no name knowledge at all"
    );

    // (c) With BOTH present, the hinted field is examined first — but only among
    // fields that actually pass the content gate.
    let both = fv(
        "acme/Camera",
        vec![
            ("thumbnail", FrameValueKind::Bytes(jpeg)),
            ("video_data", FrameValueKind::Bytes(&au)),
        ],
    );
    assert_eq!(
        classify_h264_payload(&both).expect("classified").field,
        "video_data"
    );
}

#[test]
fn a_message_with_no_byte_fields_is_never_video() {
    let frame = fv(
        "geometry_msgs/Twist",
        vec![
            ("linear", FrameValueKind::F64(1.0)),
            ("angular", FrameValueKind::F64(2.0)),
        ],
    );
    assert!(classify_h264_payload(&frame).is_none());
    assert_ne!(classify_frame(&frame), ArchetypeKind::VideoStream);
}

// ===========================================================================
// 2. The demux: one child entity per rendition
// ===========================================================================

/// The two renditions the interleave tests use. 640x360 is the REAL Go2 SPS;
/// 1280x720 is built (the committed 720p capture is a non-IDR unit and carries no
/// parameter set).
const KEY_360: StreamKey = StreamKey {
    width: 640,
    height: 360,
};
const KEY_720: StreamKey = StreamKey {
    width: 1280,
    height: 720,
};

#[test]
fn a_built_sps_decodes_to_the_macroblock_geometry_it_was_given() {
    // The builder is only trustworthy if the parser disagrees when it should, so
    // check several geometries rather than one.
    for (w_mbs, h_mbs) in [(40u32, 24u32), (80, 45), (20, 15), (1, 1)] {
        assert_eq!(
            sps_dimensions(&build_sps(0, w_mbs, h_mbs)),
            Some((w_mbs * 16, h_mbs * 16)),
            "built SPS {w_mbs}x{h_mbs} macroblocks"
        );
    }
    // And the sps_id travels without disturbing the geometry.
    assert_eq!(sps_dimensions(&build_sps(3, 80, 45)), Some((1280, 720)));
}

#[test]
fn a_single_rendition_topic_produces_exactly_one_child_and_feeds_every_unit() {
    let mut demux = VideoDemux::new();
    let input = "/camera/h264";

    let key_au = go2_real_keyframe_au();
    let route = demux.route(input, &go2_video_fv(1, 360, &key_au), &pay(&key_au));
    assert_eq!(
        route,
        VideoRoute::Feed {
            key: KEY_360,
            is_keyframe: true,
            first_sample: true
        }
    );

    // Three ordinary non-IDR units follow. They carry no SPS and there is no
    // learned discriminator yet — the single known sub-stream is the only place
    // they can belong.
    for i in 0..3u8 {
        let au = annex_b(&[&slice_nal(i)]);
        assert_eq!(
            demux.route(input, &go2_video_fv(2 + u64::from(i), 360, &au), &pay(&au)),
            VideoRoute::Feed {
                key: KEY_360,
                is_keyframe: false,
                first_sample: false
            },
            "unit {i}"
        );
    }

    assert_eq!(demux.streams(input), vec![KEY_360], "exactly one child");
    assert_eq!(demux.samples(input, KEY_360), 4);
    assert_eq!(demux.dropped_before_keyframe(input), 0);
    assert_eq!(demux.dropped_unattributable(input), 0);
}

#[test]
fn an_interleaved_two_rendition_topic_splits_into_two_children_each_fed_only_its_own_units() {
    // THE headline demux pin: one topic, two encoders,
    // alternating access units, each rendition tagged by the message's own
    // `video_height` — which the demux LEARNS from the keyframes rather than
    // being told.
    let mut demux = VideoDemux::new();
    let input = "/go2/camera/h264";

    let key_360 = go2_real_keyframe_au();
    let key_720 = annex_b(&[&build_sps(0, 80, 45), &idr_nal(0x77)]);

    // Both keyframes first, so both renditions are known and both tags learned.
    assert!(matches!(
        demux.route(input, &go2_video_fv(100, 360, &key_360), &pay(&key_360)),
        VideoRoute::Feed { key, .. } if key == KEY_360
    ));
    assert!(matches!(
        demux.route(input, &go2_video_fv(101, 720, &key_720), &pay(&key_720)),
        VideoRoute::Feed { key, .. } if key == KEY_720
    ));

    // Now alternate SPS-LESS units. Without the learned tag these are
    // indistinguishable, and feeding them to one decoder is what the H.264 route exists
    // to prevent.
    let mut expected_360 = 1u64;
    let mut expected_720 = 1u64;
    for i in 0..6u8 {
        let (height, key) = if i % 2 == 0 {
            (360u32, KEY_360)
        } else {
            (720u32, KEY_720)
        };
        let au = annex_b(&[&slice_nal(i)]);
        assert_eq!(
            demux.route(
                input,
                &go2_video_fv(200 + u64::from(i), height, &au),
                &pay(&au)
            ),
            VideoRoute::Feed {
                key,
                is_keyframe: false,
                first_sample: false
            },
            "alternating unit {i} must land on the {key:?} sub-stream"
        );
        if key == KEY_360 {
            expected_360 += 1;
        } else {
            expected_720 += 1;
        }
    }

    assert_eq!(
        demux.streams(input),
        vec![KEY_360, KEY_720],
        "two children, one per rendition"
    );
    assert_eq!(demux.samples(input, KEY_360), expected_360);
    assert_eq!(demux.samples(input, KEY_720), expected_720);
    assert_eq!(demux.dropped_unattributable(input), 0);

    // The entity paths a viewer sees.
    assert_eq!(
        video_entity("world/go2/camera/h264", KEY_360),
        "world/go2/camera/h264/viz-video/640x360"
    );
    assert_eq!(
        video_entity("world/go2/camera/h264", KEY_720),
        "world/go2/camera/h264/viz-video/1280x720"
    );
}

#[test]
fn an_interleaved_topic_with_no_discriminator_drops_rather_than_guessing() {
    // Same two renditions, but the message carries NOTHING that distinguishes
    // them — every unit reports the same tag. Attributing an SPS-less unit would
    // be a coin flip, and a wrong flip corrupts the picture that decoder is
    // building, so the correct answer is a counted drop.
    let mut demux = VideoDemux::new();
    let input = "/ambiguous/h264";

    let key_360 = go2_real_keyframe_au();
    let key_720 = annex_b(&[&build_sps(0, 80, 45), &idr_nal(0x77)]);
    demux.route(input, &go2_video_fv(1, 0, &key_360), &pay(&key_360));
    demux.route(input, &go2_video_fv(2, 0, &key_720), &pay(&key_720));

    // `video_height` reads 0 on BOTH keyframes, so it maps one value to two
    // resolutions and disqualifies itself.
    for i in 0..4u8 {
        let au = annex_b(&[&slice_nal(i)]);
        assert_eq!(
            demux.route(input, &go2_video_fv(10 + u64::from(i), 0, &au), &pay(&au)),
            VideoRoute::Drop(VideoReject::Unattributable),
            "unit {i}"
        );
    }
    assert_eq!(demux.dropped_unattributable(input), 4);
    assert_eq!(demux.samples(input, KEY_360), 1, "only its own keyframe");
    assert_eq!(demux.samples(input, KEY_720), 1);

    // And the degradation is LOUD exactly once.
    assert!(demux.take_unattributable_warn(input), "first drop warns");
    assert!(
        !demux.take_unattributable_warn(input),
        "the regime does not re-warn"
    );
}

#[test]
fn a_near_discriminator_that_disagrees_refuses_instead_of_feeding_the_wrong_decoder() {
    // THE silent-corruption pin. Two renditions of one capture normally SHARE a
    // frame counter, so on the Go2's own field order (`time_frame` before
    // `video_height`) `time_frame` legitimately learns 1000 -> 360p and
    // 1001 -> 720p: two values, two resolutions, never self-contradictory, so
    // nothing disqualifies it and the MAX_DISCRIMINATOR_VALUES cap is irrelevant.
    //
    // A 360p P-frame stamped 1001 then hits `time_frame` FIRST. Taking that hit
    // routes a 360p frame into the 720p decoder — no drop, no warn,
    // `unattributable == 0` — which is exactly the corruption the module docs call
    // worse than a gap. Requiring AGREEMENT turns it into a counted, warned drop.
    let mut demux = VideoDemux::new();
    let input = "/shared_counter/h264";

    let key_360 = go2_real_keyframe_au();
    let key_720 = annex_b(&[&build_sps(0, 80, 45), &idr_nal(0x77)]);
    demux.route(input, &go2_video_fv(1000, 360, &key_360), &pay(&key_360));
    demux.route(input, &go2_video_fv(1001, 720, &key_720), &pay(&key_720));

    // `time_frame` says 720p, `video_height` says 360p. They disagree.
    let au = annex_b(&[&slice_nal(0x31)]);
    assert_eq!(
        demux.route(input, &go2_video_fv(1001, 360, &au), &pay(&au)),
        VideoRoute::Drop(VideoReject::Unattributable),
        "two live discriminators disagreeing is a GUESS — it must be refused, not \
         resolved by declaration order"
    );
    assert_eq!(demux.dropped_unattributable(input), 1);
    assert_eq!(
        demux.samples(input, KEY_720),
        1,
        "the 720p stream must NOT have been fed a 360p frame"
    );
    assert_eq!(demux.samples(input, KEY_360), 1);
    assert!(demux.take_unattributable_warn(input), "and it is loud");

    // ANTI-TAUTOLOGY: when they AGREE, the frame still flows.
    let au2 = annex_b(&[&slice_nal(0x32)]);
    assert_eq!(
        demux.route(input, &go2_video_fv(1001, 720, &au2), &pay(&au2)),
        VideoRoute::Feed {
            key: KEY_720,
            is_keyframe: false,
            first_sample: false
        },
        "agreement must still route — the rule refuses guesses, not attribution"
    );
}

#[test]
fn a_one_byte_end_of_sequence_nal_does_not_reject_the_access_unit() {
    // `end_of_seq_rbsp()` / `end_of_stream_rbsp()` are defined EMPTY (ITU-T H.264
    // §7.3.2.5 / §7.3.2.6), so a conforming type-10/11 NAL is exactly its one
    // header byte. A flat "header plus at least one payload byte" rule rejected
    // them — and since one bad NAL fails the WHOLE buffer, an encoder that appends
    // either at a GOP boundary lost the entire message to the text dump.
    let base = go2_real_keyframe_au();
    assert!(scan_annex_b(&base).is_some(), "control: the AU alone scans");

    for (name, terminator) in [("end of sequence", 0x0Au8), ("end of stream", 0x0Bu8)] {
        let mut au = base.clone();
        au.extend_from_slice(SC4);
        au.push(terminator);
        let scan = scan_annex_b(&au)
            .unwrap_or_else(|| panic!("{name}: a 1-byte terminator NAL is conforming"));
        assert_eq!(
            scan.nals.iter().map(|n| n.kind).collect::<Vec<_>>(),
            vec![7, 8, 5, terminator & 0x1F],
            "{name}: the terminator must be seen as its own NAL"
        );
        assert!(
            scan.is_self_contained_keyframe(),
            "{name}: still a keyframe"
        );
    }

    // The rule is SCOPED: every other type still needs a payload byte, so the
    // relaxation cannot be a blanket weakening of the gate.
    assert!(
        scan_annex_b(&[0, 0, 0, 1, 0x41]).is_none(),
        "a header-only coded slice is still rejected"
    );
    assert!(
        scan_annex_b(&[0, 0, 0, 1, 0x67]).is_none(),
        "a header-only SPS is still rejected"
    );
}

#[test]
fn a_stream_whose_parameter_sets_arrived_separately_still_flags_its_idr_as_a_keyframe() {
    // An encoder that ships SPS/PPS in ONE message and the IDR in the NEXT. Judged
    // per access unit, NO sample here is ever a keyframe (the parameter-set unit
    // has no slice; the IDR has no SPS) — so the viewer would receive bytes and
    // never a start point. The verdict must be STREAM-AWARE: the sub-stream exists
    // only because the parameter sets were parsed AND fed to this entity, so the
    // decoder already holds them and the following IDR really is startable.
    let mut demux = VideoDemux::new();
    let input = "/split_params/h264";

    let params_only = annex_b(&[GO2_REAL_SPS, GO2_REAL_PPS]);
    let scan = scan_annex_b(&params_only).expect("parameter sets classify");
    assert!(
        !scan.is_self_contained_keyframe(),
        "per-unit it is not a keyframe — it carries no picture"
    );
    assert_eq!(
        demux.route(
            input,
            &go2_video_fv(1, 360, &params_only),
            &pay(&params_only)
        ),
        VideoRoute::Feed {
            key: KEY_360,
            is_keyframe: false,
            first_sample: true
        },
        "the parameter sets are FORWARDED (they are the only copy the decoder \
         gets) and open the stream, but they are not a frame"
    );

    let idr_only = annex_b(&[GO2_REAL_IDR_HEAD]);
    assert!(
        !scan_annex_b(&idr_only)
            .expect("scans")
            .is_self_contained_keyframe(),
        "per-unit the bare IDR is not a keyframe either — no SPS of its own"
    );
    assert_eq!(
        demux.route(input, &go2_video_fv(2, 360, &idr_only), &pay(&idr_only)),
        VideoRoute::Feed {
            key: KEY_360,
            is_keyframe: true,
            first_sample: false
        },
        "but ON THIS STREAM it IS the decoder's start point — this is the flag \
         that was never set before"
    );

    // A following P-frame is still not a keyframe (the flag is not sticky).
    let p = annex_b(&[&slice_nal(0x40)]);
    assert_eq!(
        demux.route(input, &go2_video_fv(3, 360, &p), &pay(&p)),
        VideoRoute::Feed {
            key: KEY_360,
            is_keyframe: false,
            first_sample: false
        }
    );
}

#[test]
fn an_unopened_renditions_units_are_never_fed_to_the_one_open_stream() {
    // BLOCKING. Between rendition A's first SPS and rendition B's first SPS, only
    // A exists — so every SPS-LESS B unit used to fall through to the
    // `streams.len() == 1` rung and be fed to A's DECODER. Silently: no drop, no
    // warn, no counter. The window is bounded by the OTHER rendition's keyframe
    // interval, which nothing here controls.
    //
    // Two 360p keyframes come first because a tag is trusted only once CONFIRMED
    // (the same value seen at DISCRIMINATOR_CONFIRM_OBSERVATIONS parameter sets) —
    // the property that stops a per-frame counter from mass-dropping a healthy
    // single-rendition topic. That is the real bound on this fix: the hole
    // closes one keyframe interval in, not instantly.
    let mut demux = VideoDemux::new();
    let input = "/unopened/h264";
    let key_360 = go2_real_keyframe_au();

    for i in 0..(DISCRIMINATOR_CONFIRM_OBSERVATIONS as u64) {
        assert!(matches!(
            demux.route(input, &go2_video_fv(i, 360, &key_360), &pay(&key_360)),
            VideoRoute::Feed { key, .. } if key == KEY_360
        ));
    }
    assert_eq!(demux.streams(input), vec![KEY_360], "only 360p is open");
    let fed_360 = demux.samples(input, KEY_360);

    // A 720-tagged unit with no SPS. 720 is a value the CONFIRMED `video_height`
    // has never been taught, so it names a rendition that is not open yet.
    let au = annex_b(&[&slice_nal(0x51)]);
    assert_eq!(
        demux.route(input, &go2_video_fv(99, 720, &au), &pay(&au)),
        VideoRoute::Drop(VideoReject::BeforeKeyframe),
        "a unit tagged for an UNOPENED rendition must NOT be fed to the open one"
    );
    assert_eq!(
        demux.samples(input, KEY_360),
        fed_360,
        "the 360p decoder must not have received it"
    );
    assert_eq!(demux.dropped_before_keyframe(input), 1, "and it is COUNTED");
    assert!(
        !demux.take_unattributable_warn(input),
        "it is a wait, not an ambiguity — no false interleaving warn"
    );

    // ANTI-TAUTOLOGY: a 360-tagged unit in the SAME state still flows, so the rule
    // discriminates on the TAG rather than blanket-dropping SPS-less units.
    let ok = annex_b(&[&slice_nal(0x52)]);
    assert_eq!(
        demux.route(input, &go2_video_fv(100, 360, &ok), &pay(&ok)),
        VideoRoute::Feed {
            key: KEY_360,
            is_keyframe: false,
            first_sample: false
        }
    );

    // And once 720p's own keyframe lands, its units flow to their OWN stream.
    let key_720 = annex_b(&[&build_sps(0, 80, 45), &idr_nal(0x77)]);
    assert!(matches!(
        demux.route(input, &go2_video_fv(101, 720, &key_720), &pay(&key_720)),
        VideoRoute::Feed { key, .. } if key == KEY_720
    ));
    let later = annex_b(&[&slice_nal(0x53)]);
    assert_eq!(
        demux.route(input, &go2_video_fv(102, 720, &later), &pay(&later)),
        VideoRoute::Feed {
            key: KEY_720,
            is_keyframe: false,
            first_sample: false
        }
    );
}

#[test]
fn an_unconfirmed_counter_miss_never_drops_a_healthy_single_rendition_stream() {
    // The other half of the confirmation rule, and the reason it exists. On a
    // single-rendition topic `time_frame` misses on EVERY P-frame. If an
    // unconfirmed miss were treated as "an unopened rendition", the whole stream
    // would be dropped — a far worse regression than the hole being closed.
    let mut demux = VideoDemux::new();
    let input = "/single/h264";
    let key_au = go2_real_keyframe_au();
    demux.route(input, &go2_video_fv(1, 360, &key_au), &pay(&key_au));

    for i in 0..20u64 {
        let au = annex_b(&[&slice_nal(i as u8)]);
        assert_eq!(
            demux.route(input, &go2_video_fv(1000 + i, 360, &au), &pay(&au)),
            VideoRoute::Feed {
                key: KEY_360,
                is_keyframe: false,
                first_sample: false
            },
            "P-frame {i} of a healthy single-rendition topic must flow"
        );
    }
    assert_eq!(demux.samples(input, KEY_360), 21);
    assert_eq!(demux.dropped_before_keyframe(input), 0);
    assert_eq!(demux.dropped_unattributable(input), 0);
}

#[test]
fn a_sub_streams_announcement_latch_fires_once_and_survives_a_reconnect() {
    // The H.264 route needed this bit to re-fire on a gRPC reconnect: rerun REQUIRED the
    // static `VideoStream:codec` component to decode H.264, and a bounced server
    // holds none, so a latch that never re-armed left video dead for the daemon's
    // whole life.
    //
    // The DECODE then moved to this desk (the viewer's own ffmpeg backend cost a
    // measured 567 ms of frame-threading lag), so what the viewer now receives is a
    // self-contained `Image` per picture and NOTHING it holds gates rendering. The
    // bit therefore stopped being viewer-scoped state and became a property of the
    // STREAM — the once-per-rendition "opened" breadcrumb, and where its decoder is
    // created. This pins the narrowed contract: it fires exactly once, and a
    // reconnect neither re-fires it nor disturbs the learned half.
    let mut demux = VideoDemux::new();
    let input = "/reconnect/h264";
    let key_au = go2_real_keyframe_au();

    assert_eq!(
        demux.route(input, &go2_video_fv(1, 360, &key_au), &pay(&key_au)),
        VideoRoute::Feed {
            key: KEY_360,
            is_keyframe: true,
            first_sample: true
        },
        "the first sample of a rendition announces it"
    );
    let p = annex_b(&[&slice_nal(1)]);
    assert_eq!(
        demux.route(input, &go2_video_fv(2, 360, &p), &pay(&p)),
        VideoRoute::Feed {
            key: KEY_360,
            is_keyframe: false,
            first_sample: false
        },
        "steady state does not re-announce"
    );

    // THE PIN: nothing re-arms it, so a later sample is still not a first sample —
    // and the LEARNED half is untouched (the stream stays open, so no rendition
    // stalls waiting for a fresh IDR, and the tally is continuous).
    let after = annex_b(&[&slice_nal(2)]);
    assert_eq!(
        demux.route(input, &go2_video_fv(3, 360, &after), &pay(&after)),
        VideoRoute::Feed {
            key: KEY_360,
            is_keyframe: false,
            first_sample: false
        },
        "the announcement is once per rendition, not once per viewer"
    );
    assert_eq!(demux.streams(input), vec![KEY_360]);
    assert_eq!(demux.samples(input, KEY_360), 3);
    assert_eq!(demux.dropped_before_keyframe(input), 0);
}

#[test]
fn the_per_input_rendition_set_is_bounded() {
    // OVERFLOW (a). `streams` was uncapped while the discriminator map was capped
    // for exactly the same reason: a producer whose SPS decodes to a fresh size
    // every keyframe would grow it without bound.
    let mut demux = VideoDemux::new();
    let input = "/many/h264";

    // Distinct macroblock geometries ⇒ distinct resolutions.
    for i in 0..MAX_STREAMS_PER_INPUT {
        let au = annex_b(&[&build_sps(0, 20 + i as u32, 15), &idr_nal(i as u8)]);
        assert!(
            matches!(
                demux.route(input, &go2_video_fv(i as u64, 0, &au), &pay(&au)),
                VideoRoute::Feed { .. }
            ),
            "rendition {i} is within the cap"
        );
    }
    assert_eq!(demux.streams(input).len(), MAX_STREAMS_PER_INPUT);

    // One past the cap is refused, and nothing is created for it.
    let over = annex_b(&[
        &build_sps(0, 20 + MAX_STREAMS_PER_INPUT as u32, 15),
        &idr_nal(0xEE),
    ]);
    assert_eq!(
        demux.route(input, &go2_video_fv(999, 0, &over), &pay(&over)),
        VideoRoute::Drop(VideoReject::Unattributable)
    );
    assert_eq!(
        demux.streams(input).len(),
        MAX_STREAMS_PER_INPUT,
        "the refused rendition must not be recorded"
    );

    // ANTI-TAUTOLOGY: an EXISTING rendition still routes past the cap, so the
    // bound refuses growth rather than freezing the topic.
    let existing = annex_b(&[&build_sps(0, 20, 15), &idr_nal(0x01)]);
    assert!(matches!(
        demux.route(input, &go2_video_fv(1000, 0, &existing), &pay(&existing)),
        VideoRoute::Feed { .. }
    ));
}

#[test]
fn an_hevc_access_unit_is_refused_rather_than_fed_to_the_h264_decoder() {
    // MEDIUM. H.265 wears the SAME Annex-B framing and its 2-byte header collides
    // with H.264's: an HEVC SPS (`42 01`) reads as a slice-data-partition-A and a
    // TRAIL_R (`02 01`) as a non-reference slice, so the H.264 header rules alone
    // ACCEPT them — the scope claim that HEVC "is rejected" was false.
    let vps: &[u8] = &[0x40, 0x01, 0x0c, 0x01];
    let sps: &[u8] = &[0x42, 0x01, 0x01, 0x60];
    let pps: &[u8] = &[0x44, 0x01, 0xc1, 0x73];
    let idr: &[u8] = &[0x26, 0x01, 0xaf, 0x08]; // IDR_W_RADL (type 19)

    for (name, nals) in [
        ("VPS+SPS+PPS+IDR", vec![vps, sps, pps, idr]),
        ("SPS alone", vec![sps]),
        ("IRAP alone", vec![idr]),
    ] {
        let au = annex_b(&nals);
        assert!(
            scan_annex_b(&au).is_none(),
            "{name}: an HEVC access unit must not classify as H.264"
        );
    }

    // ANTI-TAUTOLOGY: the real Go2 H.264 keyframe is UNAFFECTED — the HEVC
    // discriminator must not cost the codec this feature exists for.
    assert!(scan_annex_b(&go2_real_keyframe_au()).is_some());
    assert!(scan_annex_b(&annex_b(&[GO2_REAL_720P_SLICE_HEAD])).is_some());
}

#[test]
fn leading_zero_bytes_before_the_first_start_code_are_accepted() {
    // NIT, but a generality one: Annex-B §B.1.1 permits any number of
    // `leading_zero_8bits` before the first start code, so refusing them fails a
    // legal stream. Only ZERO bytes are skipped — `a_start_code_buried_inside_a_blob`
    // pins that a start code after NON-zero bytes is still refused.
    let base = go2_real_keyframe_au();
    for pad in [1usize, 2, 5] {
        let mut au = vec![0u8; pad];
        au.extend_from_slice(&base);
        let scan = scan_annex_b(&au).unwrap_or_else(|| panic!("{pad} leading zeros is legal"));
        assert_eq!(
            scan.nals.iter().map(|n| n.kind).collect::<Vec<_>>(),
            vec![7, 8, 5],
            "{pad}: the NAL sequence must be unchanged"
        );
        assert_eq!(
            sps_dimensions(scan.nals[0].bytes),
            Some((640, 360)),
            "{pad}: and the SPS must still parse"
        );
    }
}

#[test]
fn the_unattributable_warn_re_arms_after_a_successful_feed() {
    // NIT: every other flood latch in this crate (`DrainWarnLatch`,
    // `OutputDiscardLatch`) re-arms when the regime ends. A latch that never
    // re-arms goes silent for the rest of the run after ONE regime.
    let mut demux = VideoDemux::new();
    let input = "/rearm/h264";
    let key_360 = go2_real_keyframe_au();
    let key_720 = annex_b(&[&build_sps(0, 80, 45), &idr_nal(0x77)]);
    demux.route(input, &go2_video_fv(1, 0, &key_360), &pay(&key_360));
    demux.route(input, &go2_video_fv(2, 0, &key_720), &pay(&key_720));

    // `video_height` reads 0 on both ⇒ disqualified ⇒ ambiguous.
    let a = annex_b(&[&slice_nal(1)]);
    assert_eq!(
        demux.route(input, &go2_video_fv(10, 0, &a), &pay(&a)),
        VideoRoute::Drop(VideoReject::Unattributable)
    );
    assert!(demux.take_unattributable_warn(input), "regime 1 warns");
    assert!(!demux.take_unattributable_warn(input), "and only once");

    // A keyframe feeds successfully, ENDING the regime.
    demux.route(input, &go2_video_fv(20, 0, &key_360), &pay(&key_360));

    // A LATER regime must be loud again.
    let b = annex_b(&[&slice_nal(2)]);
    assert_eq!(
        demux.route(input, &go2_video_fv(30, 0, &b), &pay(&b)),
        VideoRoute::Drop(VideoReject::Unattributable)
    );
    assert!(
        demux.take_unattributable_warn(input),
        "a NEW regime must warn again — a latch that never re-arms goes silent forever"
    );
}

#[test]
fn a_never_keyframed_topic_escalates_from_debug_to_one_loud_warn() {
    // MEDIUM: a short mid-GOP wait is normal and stays at debug, but a topic that
    // never delivers a parameter set will never render, and an empty pane with
    // only debug logs is the silent failure the loud-not-silent rule forbids.
    let mut demux = VideoDemux::new();
    let input = "/never/h264";

    for i in 0..(NEVER_KEYFRAMED_WARN_AFTER - 1) {
        let au = annex_b(&[&slice_nal(i as u8)]);
        demux.route(input, &go2_video_fv(i, 360, &au), &pay(&au));
        assert!(
            !demux.take_never_keyframed_warn(input),
            "unit {i} is still within the ordinary attach transient"
        );
    }
    let au = annex_b(&[&slice_nal(0xFF)]);
    demux.route(input, &go2_video_fv(9999, 360, &au), &pay(&au));
    assert!(
        demux.take_never_keyframed_warn(input),
        "at the threshold it escalates"
    );
    assert!(!demux.take_never_keyframed_warn(input), "exactly once");

    // ANTI-TAUTOLOGY: a topic that DID open a stream never escalates, however many
    // units it later waits on.
    let mut healthy = VideoDemux::new();
    let ok = "/healthy/h264";
    let key_au = go2_real_keyframe_au();
    healthy.route(ok, &go2_video_fv(1, 360, &key_au), &pay(&key_au));
    for i in 0..(NEVER_KEYFRAMED_WARN_AFTER + 10) {
        let au = annex_b(&[&slice_nal(i as u8)]);
        healthy.route(ok, &go2_video_fv(1000 + i, 360, &au), &pay(&au));
    }
    assert!(
        !healthy.take_never_keyframed_warn(ok),
        "a topic with an open stream is not the never-keyframed shape"
    );
}

#[test]
fn an_unparseable_parameter_set_is_counted_rather_than_silently_dropped() {
    // MEDIUM: an SPS this build cannot read used to be permanently AND silently
    // un-renderable, reported only at debug under a reason ("waiting for a
    // keyframe") that was not the truth.
    let mut demux = VideoDemux::new();
    let input = "/badsps/h264";

    // A NAL that passes the H.264 header rules as an SPS but is not a parseable
    // sequence parameter set.
    let broken_sps: &[u8] = &[0x67, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
    let au = annex_b(&[broken_sps, &idr_nal(0x10)]);
    assert!(
        scan_annex_b(&au).is_some(),
        "precondition: it classifies as video"
    );
    assert!(
        sps_dimensions(broken_sps).is_none(),
        "precondition: and its SPS does NOT parse"
    );

    assert_eq!(
        demux.route(input, &go2_video_fv(1, 360, &au), &pay(&au)),
        VideoRoute::Drop(VideoReject::BeforeKeyframe)
    );
    assert_eq!(
        demux.sps_parse_failures(input),
        1,
        "the real cause must be COUNTED, not folded into the keyframe wait"
    );
    assert_eq!(demux.streams(input), Vec::<StreamKey>::new());

    // ANTI-TAUTOLOGY: a parseable SPS never counts as a parse failure.
    let good = go2_real_keyframe_au();
    demux.route(input, &go2_video_fv(2, 360, &good), &pay(&good));
    assert_eq!(demux.sps_parse_failures(input), 1);
}

#[test]
fn a_counter_field_disqualifies_itself_instead_of_growing_without_bound() {
    // `time_frame` takes a new value every message. If it were trusted as a
    // rendition tag its learned map would grow forever and never hit. It must be
    // discarded once it exceeds MAX_DISCRIMINATOR_VALUES distinct values, while
    // the real tag beside it keeps working.
    let mut demux = VideoDemux::new();
    let input = "/counter/h264";

    let key_360 = go2_real_keyframe_au();
    let key_720 = annex_b(&[&build_sps(0, 80, 45), &idr_nal(0x77)]);

    // Enough keyframes to blow the counter's budget, alternating renditions.
    for i in 0..(MAX_DISCRIMINATOR_VALUES as u64 + 3) {
        let (au, height) = if i % 2 == 0 {
            (&key_360, 360u32)
        } else {
            (&key_720, 720u32)
        };
        demux.route(input, &go2_video_fv(1_000 + i, height, au), &pay(au));
    }

    // A non-keyframe whose `time_frame` is one the demux HAS seen before. If
    // `time_frame` were still live it could hit and route by a meaningless
    // coincidence; it must be `video_height` that decides.
    let au = annex_b(&[&slice_nal(9)]);
    assert_eq!(
        demux.route(input, &go2_video_fv(1_000, 720, &au), &pay(&au)),
        VideoRoute::Feed {
            key: KEY_720,
            is_keyframe: false,
            first_sample: false
        },
        "the surviving tag decides, not the stale counter value (which was learned \
         against the 360p stream)"
    );
}

#[test]
fn a_substream_stays_closed_until_its_first_keyframe() {
    // A decoder cannot start mid-GOP, and rerun's H.264 contract requires the SPS
    // on the keyframe. Everything before the first SPS-carrying unit is dropped
    // and counted — never fed, never silently swallowed.
    let mut demux = VideoDemux::new();
    let input = "/late/h264";

    for i in 0..3u8 {
        let au = annex_b(&[&slice_nal(i)]);
        assert_eq!(
            demux.route(input, &go2_video_fv(u64::from(i), 360, &au), &pay(&au)),
            VideoRoute::Drop(VideoReject::BeforeKeyframe),
            "a mid-GOP attach is WaitingForKeyframe, NOT Unattributable — the two \
             mean opposite things to an operator and only the latter is a fault"
        );
    }
    assert_eq!(demux.dropped_before_keyframe(input), 3);
    assert_eq!(
        demux.dropped_unattributable(input),
        0,
        "the healthy attach transient must never be counted as an ambiguity"
    );
    assert_eq!(demux.streams(input), Vec::<StreamKey>::new());
    assert!(
        !demux.take_unattributable_warn(input),
        "and it must never fire the loud interleaved-renditions warn"
    );

    // The keyframe opens the stream, and everything after it flows.
    let key_au = go2_real_keyframe_au();
    assert_eq!(
        demux.route(input, &go2_video_fv(9, 360, &key_au), &pay(&key_au)),
        VideoRoute::Feed {
            key: KEY_360,
            is_keyframe: true,
            first_sample: true
        }
    );
    let au = annex_b(&[&slice_nal(0x44)]);
    assert_eq!(
        demux.route(input, &go2_video_fv(10, 360, &au), &pay(&au)),
        VideoRoute::Feed {
            key: KEY_360,
            is_keyframe: false,
            first_sample: false
        }
    );
    assert_eq!(demux.samples(input, KEY_360), 2);
}

#[test]
fn a_second_renditions_units_wait_for_its_own_keyframe_not_the_first_streams() {
    // The gate is PER SUB-STREAM. A topic whose 360p stream is already running
    // must not feed 720p units into a decoder that has never seen a 720p SPS.
    let mut demux = VideoDemux::new();
    let input = "/staggered/h264";

    let key_360 = go2_real_keyframe_au();
    demux.route(input, &go2_video_fv(1, 360, &key_360), &pay(&key_360));

    // Teach it the 720 tag WITHOUT opening the 720 stream is impossible by
    // construction (learning happens only on an SPS), so drive the real
    // sequence: a 720 keyframe opens it, then a 720 non-keyframe flows.
    let key_720 = annex_b(&[&build_sps(0, 80, 45), &idr_nal(0x77)]);
    assert_eq!(
        demux.route(input, &go2_video_fv(2, 720, &key_720), &pay(&key_720)),
        VideoRoute::Feed {
            key: KEY_720,
            is_keyframe: true,
            first_sample: true
        },
        "the 720 stream's FIRST sample is its own keyframe, not a borrowed one"
    );
    let au = annex_b(&[&slice_nal(0x55)]);
    assert_eq!(
        demux.route(input, &go2_video_fv(3, 720, &au), &pay(&au)),
        VideoRoute::Feed {
            key: KEY_720,
            is_keyframe: false,
            first_sample: false
        }
    );
    assert_eq!(demux.samples(input, KEY_360), 1);
    assert_eq!(demux.samples(input, KEY_720), 2);
    assert_eq!(demux.dropped_before_keyframe(input), 0);
}

#[test]
fn an_sps_carrying_unit_that_is_not_an_idr_still_opens_the_stream_but_is_not_a_keyframe() {
    // A recovery-point stream re-sends its parameter sets ahead of a P-frame. A
    // decoder CAN configure from it, so the sub-stream opens — but rerun's
    // keyframe definition is "IDR + SPS", so the sample is not flagged one.
    let mut demux = VideoDemux::new();
    let input = "/recovery/h264";
    let au = annex_b(&[&build_sps(0, 40, 24), &slice_nal(0x66)]);
    assert_eq!(
        demux.route(input, &go2_video_fv(1, 384, &au), &pay(&au)),
        VideoRoute::Feed {
            key: StreamKey {
                width: 640,
                height: 384
            },
            is_keyframe: false,
            first_sample: true
        }
    );
}

#[test]
fn two_topics_do_not_share_demux_state() {
    let mut demux = VideoDemux::new();
    let a = "/cam_a/h264";
    let b = "/cam_b/h264";

    let key_au = go2_real_keyframe_au();
    demux.route(a, &go2_video_fv(1, 360, &key_au), &pay(&key_au));

    // `b` has seen no keyframe of its own; `a`'s must not open it.
    let au = annex_b(&[&slice_nal(1)]);
    assert_eq!(
        demux.route(b, &go2_video_fv(1, 360, &au), &pay(&au)),
        VideoRoute::Drop(VideoReject::BeforeKeyframe)
    );
    assert_eq!(demux.streams(a), vec![KEY_360]);
    assert_eq!(demux.streams(b), Vec::<StreamKey>::new());
}

#[test]
fn the_demux_is_deterministic_over_an_identical_sequence() {
    // Principle #7: identical inputs, identical entity assignment. Both runs are
    // ALSO checked against a hand-written oracle, so this is not a self-compare.
    let key_360 = go2_real_keyframe_au();
    let key_720 = annex_b(&[&build_sps(0, 80, 45), &idr_nal(0x77)]);
    let heights = [360u32, 720, 360, 720, 720, 360];

    let run = || {
        let mut demux = VideoDemux::new();
        let input = "/det/h264";
        let mut out: Vec<String> = Vec::new();
        demux.route(input, &go2_video_fv(1, 360, &key_360), &pay(&key_360));
        demux.route(input, &go2_video_fv(2, 720, &key_720), &pay(&key_720));
        for (i, h) in heights.iter().enumerate() {
            let au = annex_b(&[&slice_nal(i as u8)]);
            let route = demux.route(input, &go2_video_fv(10 + i as u64, *h, &au), &pay(&au));
            out.push(match route {
                VideoRoute::Feed { key, .. } => video_entity("world/det/h264", key),
                VideoRoute::Drop(r) => format!("drop:{r:?}"),
            });
        }
        out
    };

    let oracle: Vec<String> = heights
        .iter()
        .map(|h| {
            let key = if *h == 360 { KEY_360 } else { KEY_720 };
            video_entity("world/det/h264", key)
        })
        .collect();

    let first = run();
    let second = run();
    assert_eq!(first, oracle, "run 1 matches the hand oracle");
    assert_eq!(second, oracle, "run 2 matches the hand oracle");
    assert_eq!(first, second, "and the two runs are identical");
}

/// Classify `au` as a payload the demux can route (every test above hands the
/// demux the SAME bytes it hands the `FrameValue`).
fn pay(au: &[u8]) -> cerulion_viz::video::H264Payload<'_> {
    let frame = go2_video_fv(0, 0, au);
    // The payload borrows `au`, not the temporary `FrameValue`, so re-classify
    // against a value whose lifetime is the caller's slice.
    let p = classify_h264_payload(&frame).expect("test access units are Annex-B");
    cerulion_viz::video::H264Payload {
        field: p.field,
        bytes: au,
        access_unit: scan_annex_b(au).expect("scanned"),
    }
}

// ===========================================================================
// 3. The render arm, through the real `dispatch_frame` seam
// ===========================================================================

/// The Go2's corrected `/frontvideostream` definition, used to build
/// REAL wire frames the production walker decodes.
const PROBE_MSG: &str = "\
uint64 time_frame
uint32 video_height
uint8[] video_data
";
const PROBE_QNAME: &str = "probe/VideoProbe";

fn probe_schema() -> MessageSchema {
    parse_rosmsg(PROBE_MSG, "VideoProbe", Some("probe")).expect("probe schema parses")
}

fn probe_walker() -> FrameWalker {
    let mut schemas: Vec<MessageSchema> = Vec::new();
    for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
        if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
            schemas.push(s);
        }
    }
    schemas.push(probe_schema());
    let (walker, _) = FrameWalker::new(schemas);
    walker
}

fn probe_layout() -> WireLayout {
    let mut schemas: Vec<MessageSchema> = Vec::new();
    for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
        if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
            schemas.push(s);
        }
    }
    schemas.push(probe_schema());
    let (mut resolver, _) = LayoutResolver::new(schemas);
    resolver.layout_of(PROBE_QNAME).expect("probe layout")
}

/// Build a real `probe/VideoProbe` wire frame carrying `au`.
fn build_probe_frame(time_frame: u64, video_height: u32, au: &[u8], timestamp_ns: u64) -> Vec<u8> {
    let layout = probe_layout();
    assert_eq!(
        layout
            .variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["video_data"],
        "probe variable-field order changed"
    );
    let fixed = layout.fixed_size;
    let table = layout.offset_table_bytes();

    let mut payload = vec![0u8; fixed + table];
    let tf_off = field_offset(&layout, "time_frame");
    let vh_off = field_offset(&layout, "video_height");
    payload[tf_off..tf_off + 8].copy_from_slice(&time_frame.to_le_bytes());
    payload[vh_off..vh_off + 4].copy_from_slice(&video_height.to_le_bytes());
    let data_off = (fixed + table) as u32;
    write_offset_entry(&mut payload, fixed, 0, data_off, au.len() as u32);
    payload.extend_from_slice(au);

    let mut frame = vec![0u8; WireHeader::SIZE];
    let header = WireHeader {
        schema_hash: probe_schema().schema_hash(),
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed) as u32,
        offset_table_count: 1,
        sequence: 0,
        timestamp_ns,
    };
    header.write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

fn field_offset(layout: &WireLayout, field: &str) -> usize {
    layout
        .fixed_fields
        .iter()
        .find(|f| f.name == field)
        .unwrap_or_else(|| panic!("fixed field '{field}' not in the probe layout"))
        .offset
}

fn memory() -> (rerun::RecordingStream, rerun::sink::MemorySinkStorage) {
    rerun::RecordingStreamBuilder::new("test")
        .recording_id("video_h264_test")
        .memory()
        .expect("memory sink")
}

/// Every chunk the sink emitted, decoded back out of the memory sink (crib:
/// `coordinate_frame_test::chunks`). NOTE: `take()` DRAINS, so each call returns
/// only what has been logged since the last one — the drop tests rely on that.
fn chunks(storage: &rerun::sink::MemorySinkStorage) -> Vec<rerun::log::Chunk> {
    storage
        .take()
        .into_iter()
        .filter_map(|msg| match msg {
            rerun::log::LogMsg::ArrowMsg(_, arrow_msg) => {
                Some(rerun::log::Chunk::from_arrow_msg(&arrow_msg).expect("decode chunk"))
            }
            _ => None,
        })
        .collect()
}

/// `(entity path, component descriptor)` for every component of every chunk —
/// read back off the REAL rerun store, so these assertions observe what a viewer
/// would, not what the sink believes it sent.
fn logged_components(storage: &rerun::sink::MemorySinkStorage) -> Vec<(String, String)> {
    let mut out = Vec::new();
    for chunk in chunks(storage) {
        let entity = chunk
            .entity_path()
            .to_string()
            .trim_start_matches('/')
            .to_string();
        for descr in chunk.components().keys() {
            out.push((entity.clone(), descr.as_str().to_string()));
        }
    }
    out
}

/// Whether a DECODED PICTURE was logged at exactly `entity`.
///
/// This used to look for `VideoStream:sample` — an access unit handed to
/// the VIEWER to decode. Rerun 0.34 decodes those in a spawned `ffmpeg` whose
/// default frame threading measured 567 ms of lag, so the desk decodes now and
/// what reaches the viewer is a plain `Image`.
fn logged_image_at(components: &[(String, String)], entity: &str) -> bool {
    components
        .iter()
        .any(|(e, c)| e == entity && c.contains("Image:buffer"))
}

/// Whether ANY `VideoStream` component reached the viewer. Must always be false:
/// see [`logged_image_at`]. The pixel-level pins live in `video_decode_test.rs`;
/// this file's e2e arms pin ROUTING through `dispatch_frame`.
fn logged_any_video_stream(components: &[(String, String)]) -> bool {
    components.iter().any(|(_, c)| c.contains("VideoStream"))
}

/// Eleven CONSECUTIVE access units of a 360p rendition —
/// one keyframe and the ten P-frames after it. Provenance and the AU-split rule
/// are documented on the same constant in `video_decode_test.rs`.
///
/// This REPLACES the lone `go2_frontvideostream_360p_idr_au.h264` keyframe that
/// used to serve the arms below. The truncated `GO2_REAL_*` constants above are
/// enough for the DEMUX (which reads NAL headers and the SPS) and the
/// pure-routing tests keep using them; an arm that drives the full
/// `dispatch_frame` path needs bytes that actually DECODE, and since the GOP-collapse fix it
/// needs at least TWO of them.
///
/// Every arm below that asserts a RENDERED PICTURE needs this rather than the
/// lone keyframe above: since the GOP-collapse fix the decoder runs with openh264's
/// flush-after-decode OFF (its default leaked a picture reference and reset the
/// decoder four pictures into every GOP), so openh264 holds a picture for one
/// call and ONE access unit renders nothing. A second REAL unit is what pushes
/// the first one's picture out — a hand-built slice cannot, because an
/// undecodable unit yields no picture and the wrapper discards anything released
/// on a failing call.
const GO2_REAL_GOP: &[u8] = include_bytes!("fixtures/go2_frontvideostream_360p_gop_11au.h264");

/// The keyframe and the P-frame that follows it, straight out of [`GO2_REAL_GOP`].
///
/// Split by the rule documented in `video_decode_test.rs`: this stream carries no
/// access unit delimiter, and every unit is either a lone non-IDR slice or
/// exactly `SPS, PPS, IDR`, so a unit begins at every type-7 and every type-1
/// NAL. `video_decode_test.rs` asserts that shape on the committed bytes.
fn go2_first_two_units() -> (&'static [u8], &'static [u8]) {
    let mut starts = Vec::new();
    let mut i = 0usize;
    while i + 5 <= GO2_REAL_GOP.len() {
        if GO2_REAL_GOP[i..i + 4] == [0, 0, 0, 1] {
            let kind = GO2_REAL_GOP[i + 4] & 0x1F;
            if kind == 7 || kind == 1 {
                starts.push(i);
            }
            i += 5;
            continue;
        }
        i += 1;
    }
    assert!(starts.len() >= 3, "fixture must carry at least 3 units");
    (
        &GO2_REAL_GOP[starts[0]..starts[1]],
        &GO2_REAL_GOP[starts[1]..starts[2]],
    )
}

#[test]
fn dispatch_frame_logs_a_real_go2_access_unit_under_its_rendition_child() {
    let (rec, storage) = memory();
    let walker = probe_walker();
    let mut state = SinkState::new();

    // TWO real units: openh264 holds the keyframe's picture for one call
    // before releasing it, so a lone unit renders nothing and this arm would be vacuous.
    let (key, p1) = go2_first_two_units();
    for (n, au) in [key, p1].iter().enumerate() {
        let frame = build_probe_frame(n as u64, 360, au, 42_000 + n as u64);
        dispatch_frame(&rec, &walker, "/go2/camera/h264", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");

    let components = logged_components(&storage);
    let expected = "world/go2/camera/h264/viz-video/640x360";
    assert!(
        logged_image_at(&components, expected),
        "expected a decoded picture at {expected}, got {components:?}"
    );
    // No codec declaration any more — the viewer is not decoding, so
    // there is no codec to tell it about, and nothing it holds gates rendering.
    assert!(
        !logged_any_video_stream(&components),
        "the viewer must never be handed a VideoStream to decode: {components:?}"
    );
    // The topic entity itself must carry NO picture — pictures live on the
    // rendition child so a second rendition can never overwrite the first.
    assert!(
        !logged_image_at(&components, "world/go2/camera/h264"),
        "no video may be logged at the bare topic entity"
    );
    assert_eq!(
        state.video().streams("/go2/camera/h264"),
        vec![KEY_360],
        "one rendition seen"
    );
    // Two units fed, both routed to the one rendition (the second is
    // what pushes the first's picture out of openh264's pipeline).
    assert_eq!(state.video().samples("/go2/camera/h264", KEY_360), 2);
}

#[test]
fn dispatch_frame_splits_an_interleaved_topic_across_two_children() {
    let (rec, storage) = memory();
    let walker = probe_walker();
    let mut state = SinkState::new();
    let input = "/go2/camera/h264";

    let key_360 = go2_real_keyframe_au();
    let key_720 = annex_b(&[&build_sps(0, 80, 45), &idr_nal(0x77)]);
    for (ts, h, au) in [(1_000u64, 360u32, &key_360), (2_000, 720, &key_720)] {
        let frame = build_probe_frame(ts, h, au, ts);
        dispatch_frame(&rec, &walker, input, &frame, &mut state);
    }
    for i in 0..4u8 {
        let h = if i % 2 == 0 { 360 } else { 720 };
        let au = annex_b(&[&slice_nal(i)]);
        let frame = build_probe_frame(3_000 + u64::from(i), h, &au, 3_000 + u64::from(i));
        dispatch_frame(&rec, &walker, input, &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");

    // ROUTING is what this arm pins, and the demux counters are where routing is
    // observable: both streams here are hand-built NAL headers over synthetic
    // payload, which the decoder correctly refuses, so no picture reaches
    // the viewer. The pixel-level two-rendition split — two REAL decodable
    // streams, one decoder each, no cross-feed — is
    // `video_decode_test::two_renditions_decode_independently`.
    let components = logged_components(&storage);
    assert!(
        !logged_any_video_stream(&components),
        "the viewer must never be handed a VideoStream to decode: {components:?}"
    );
    assert_eq!(state.video().streams(input), vec![KEY_360, KEY_720]);
    assert_eq!(state.video().samples(input, KEY_360), 3);
    assert_eq!(state.video().samples(input, KEY_720), 3);
}

#[test]
fn dispatch_frame_leaves_a_non_video_topic_on_its_existing_route() {
    // Anti-tautology for the whole feature: the probe schema is UNMAPPED and
    // UNSHAPED, so a frame whose `video_data` is NOT Annex-B must take the
    // ordinary fallback and log at the topic entity, with no video child.
    let (rec, storage) = memory();
    let walker = probe_walker();
    let mut state = SinkState::new();
    let input = "/not/video";

    let junk: &[u8] = &[0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x11, 0x22, 0x33];
    let frame = build_probe_frame(7, 0, junk, 5_000);
    dispatch_frame(&rec, &walker, input, &frame, &mut state);
    rec.flush_blocking().expect("flush");

    let components = logged_components(&storage);
    assert!(
        !components.iter().any(|(e, _)| e.contains("viz-video")),
        "no video child may appear for a non-video payload, got {components:?}"
    );
    assert!(
        components
            .iter()
            .any(|(e, _)| e.starts_with("world/not/video")),
        "the frame must still render somewhere, got {components:?}"
    );
    assert_eq!(state.video().streams(input), Vec::<StreamKey>::new());
}

#[test]
fn dispatch_frame_drops_units_that_precede_the_first_keyframe_and_counts_them() {
    let (rec, storage) = memory();
    let walker = probe_walker();
    let mut state = SinkState::new();
    let input = "/late/h264";

    // Three SPS-less units on a topic with no stream yet: nothing renders.
    for i in 0..3u8 {
        let au = annex_b(&[&slice_nal(i)]);
        let frame = build_probe_frame(u64::from(i), 360, &au, 100 + u64::from(i));
        dispatch_frame(&rec, &walker, input, &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");
    // `logged_components` DRAINS the sink, so this observes exactly the window
    // covered by the three dropped units.
    assert!(
        !logged_components(&storage)
            .iter()
            .any(|(e, _)| e.contains("viz-video")),
        "nothing may be fed before a keyframe"
    );
    assert_eq!(state.video().dropped_before_keyframe(input), 3);
    assert_eq!(
        state.video().dropped_unattributable(input),
        0,
        "a healthy mid-GOP attach is not an ambiguity — this is what keeps the loud \
         interleaved-renditions warn off every normal camera"
    );

    // The keyframe opens it and the stream starts rendering — TWO real units,
    // because openh264 holds the keyframe's picture for one call.
    let (key, p1) = go2_first_two_units();
    for (n, au) in [key, p1].iter().enumerate() {
        let frame = build_probe_frame(9 + n as u64, 360, au, 200 + n as u64);
        dispatch_frame(&rec, &walker, input, &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");
    assert!(
        logged_image_at(
            &logged_components(&storage),
            "world/late/h264/viz-video/640x360"
        ),
        "the keyframe must open the stream and render"
    );
}

// ===========================================================================
// 4. The memo composition — the content rung must survive a WARM memo
// ===========================================================================

/// Build a real `sensor_msgs/CompressedImage` wire frame carrying `blob`.
fn build_compressed_image_frame(format: &[u8], blob: &[u8], timestamp_ns: u64) -> Vec<u8> {
    let mut schemas: Vec<MessageSchema> = Vec::new();
    for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
        if let Ok(sc) = parse_rosmsg(text, name, Some(pkg)) {
            schemas.push(sc);
        }
    }
    let (mut resolver, _) = LayoutResolver::new(schemas);
    let layout = resolver
        .layout_of("sensor_msgs/CompressedImage")
        .expect("built-in schema");
    let fixed = layout.fixed_size;
    let table = layout.offset_table_bytes();
    let mut payload = vec![0u8; fixed + table];
    let format_off = (fixed + table) as u32;
    let data_off = format_off + format.len() as u32;
    write_offset_entry(&mut payload, fixed, 0, 0, 0); // header: empty
    write_offset_entry(&mut payload, fixed, 1, format_off, format.len() as u32);
    write_offset_entry(&mut payload, fixed, 2, data_off, blob.len() as u32);
    payload.extend_from_slice(format);
    payload.extend_from_slice(blob);

    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: <CompressedImage as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed) as u32,
        offset_table_count: 3,
        sequence: 0,
        timestamp_ns,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

#[test]
fn the_video_rung_survives_the_memo_cold_and_warm() {
    // THE merge pin. The memo fix memoizes the SHAPE-inference half of the archetype
    // decision per `(input, schema_hash)`. A video topic's schema is UNMAPPED
    // (`unitree_go/Go2FrontVideoData` is the live one) and its shape is a bag of
    // plottable integers, so if the content rung were inside the memoized half —
    // or after it — the FIRST frame would infer `Scalars`, MEMOIZE it, and serve
    // that for the life of the run. Video dead, no error, on exactly the topic
    // this feature exists for.
    //
    // Both frames go through the REAL `dispatch_frame` -> `SinkState::archetype_for`
    // path: frame 1 with a cold memo, frame 2 with a warm one.
    let (rec, storage) = memory();
    let walker = probe_walker();
    let mut state = SinkState::new();
    let input = "/go2/camera/h264";
    let expected = "world/go2/camera/h264/viz-video/640x360";

    let p_frame = annex_b(&[&slice_nal(0x21)]);

    // COLD — real robot bytes, so this one both routes AND decodes.
    //
    // The cold half's ROUTING is asserted directly (the demux tally),
    // because openh264 holds the keyframe's picture for one call and no image
    // exists yet. The RENDERING pin is kept, one unit later.
    let (key, p1_real) = go2_first_two_units();
    let f1 = build_probe_frame(1, 360, key, 1_000);
    dispatch_frame(&rec, &walker, input, &f1, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        state.video().samples(input, KEY_360),
        1,
        "cold: the first frame must be classified as video and routed to its \
         rendition sub-stream — NOT inferred as Scalars"
    );

    // WARM, and REAL — the memo has now seen this input, and a second frame of
    // the SAME schema is exactly the case `archetype_for` short-circuits. Real
    // robot bytes, so it also pushes the keyframe's picture out and the rendering
    // pin below is reachable.
    let f2 = build_probe_frame(2, 360, p1_real, 2_000);
    dispatch_frame(&rec, &walker, input, &f2, &mut state);
    rec.flush_blocking().expect("flush");
    assert!(
        logged_image_at(&logged_components(&storage), expected),
        "the video rung must end in a decoded picture at {expected}"
    );

    // WARM and UNDECODABLE — the classification probe proper. A hand-built
    // P-frame no decoder can render must STILL be routed to the video sub-stream
    // rather than frozen as `Scalars` by the memo, so the observable is the demux
    // tally alone. It runs LAST because openh264 answers garbage with one of its
    // context-RESETTING error arms, which discards the parameter sets — anything
    // after it could not decode either.
    let f3 = build_probe_frame(3, 360, &p_frame, 3_000);
    dispatch_frame(&rec, &walker, input, &f3, &mut state);
    rec.flush_blocking().expect("flush");

    // The structural half, independent of what rendered: the memo must hold
    // NOTHING for this input, because the content rung answers before the
    // inference the memo caches ever runs.
    assert_eq!(
        state.cached_archetype(input),
        None,
        "a content-classified topic must never enter the shape memo"
    );
    assert_eq!(
        state.inference_runs(),
        0,
        "and the shape ladder must never have run for it at all"
    );

    // The demux agrees it saw all three samples on one sub-stream — so the WARM
    // frames really were routed as video, not merely re-classified. The third,
    // which no decoder can render, is the one that makes this a classification
    // oracle rather than a rendering one.
    assert_eq!(
        state.video().samples(input, KEY_360),
        3,
        "every frame must reach the video sub-stream, decodable or not"
    );
}

#[test]
fn a_name_mapped_video_payload_outranks_the_table_on_a_warm_memo_too() {
    // The other half of the composition: content must beat a CERTAIN name match.
    // The memo is consulted only AFTER a `classify_schema` miss, so a rung placed
    // inside it could never see a `sensor_msgs/CompressedImage` carrying H.264 —
    // the table would answer `Image` first, every frame, warm or cold.
    let (rec, storage) = memory();
    let walker = probe_walker();
    let mut state = SinkState::new();
    let input = "/cam/compressed";

    // Both units are REAL robot bytes now. The second used to be a
    // hand-built slice, which routes but cannot decode — and since openh264 holds
    // the keyframe's picture for one call, nothing would have rendered and the
    // `viz-video` assertion below (this test's whole discriminator against the
    // name table's `Image`) would be unreachable. A real P-frame demonstrates the
    // same "the SECOND frame also takes the video route" property, so the tally
    // oracle is unchanged.
    let (key, p1) = go2_first_two_units();
    let key_au = key.to_vec();
    let p_frame = p1.to_vec();
    for (au, ts) in [(&key_au, 1_000u64), (&p_frame, 2_000)] {
        let frame = build_compressed_image_frame(b"h264", au, ts);
        dispatch_frame(&rec, &walker, input, &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");

    assert!(
        logged_image_at(
            &logged_components(&storage),
            "world/cam/compressed/viz-video/640x360"
        ),
        "an h264-carrying CompressedImage renders as video, beating the name table"
    );
    // BOTH frames took the video route — the second is a hand-built P-frame that
    // no decoder can render, so the demux tally is where that is observable.
    assert_eq!(state.video().samples(input, KEY_360), 2);

    // ANTI-TAUTOLOGY: the ordinary JPEG path on the SAME schema is untouched, and
    // logs no video at all.
    let (rec2, storage2) = memory();
    let mut state2 = SinkState::new();
    let jpeg: &[u8] = &[0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, b'J', b'F', b'I', b'F'];
    for ts in [1_000u64, 2_000] {
        let frame = build_compressed_image_frame(b"jpeg", jpeg, ts);
        dispatch_frame(&rec2, &walker, "/cam/jpeg", &frame, &mut state2);
    }
    rec2.flush_blocking().expect("flush");
    let components = logged_components(&storage2);
    assert!(
        !components.iter().any(|(e, _)| e.contains("viz-video")),
        "a JPEG must never take the video path, got {components:?}"
    );
    assert!(
        components
            .iter()
            .any(|(_, c)| c.starts_with("EncodedImage")),
        "…and must still render as an EncodedImage, got {components:?}"
    );
}
