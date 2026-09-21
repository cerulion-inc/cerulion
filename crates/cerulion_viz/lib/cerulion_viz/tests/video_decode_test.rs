// SPDX-License-Identifier: AGPL-3.0-only
//! The DESK-SIDE H.264 decode + latest-frame presentation path.
//!
//! `video_h264_test.rs` owns the DEMUX (which rendition an access unit belongs
//! to). This file owns what happens after that decision: the access unit is
//! DECODED here and logged as a [`rerun::Image`], instead of being handed to the
//! viewer as a `VideoStream` sample for it to decode.
//!
//! # Why the change, in one measurement
//!
//! Rerun 0.34's native H.264 backend spawns an `ffmpeg` CLI and leaves `-threads`
//! unset, so libavcodec frame-threads sized to the VIEWER's core count and holds
//! roughly one frame per thread before emitting anything. Measured on the test
//! rig against a real x264 stream, one access unit fed per frame period:
//! **18 access units in before frame 0 came out, 567 ms steady-state p50** on a
//! 16-core desk — versus **1 unit in, 0.5 ms** decoding in-process. The full
//! `-threads` sweep is in `src/video_decode.rs`.
//!
//! # The fixtures are REAL encoder / REAL robot output
//!
//! * `fixtures/go2_frontvideostream_360p_idr_au.h264` — the Annex-B access unit
//!   carved out of `examples/go2/nodes/dds_bridge/tests/fixtures/go2_frontvideostream_360p_idr.cdr`
//!   (everything from its first start code on). That `.cdr` is a verbatim DDS
//!   capture taken off a live Unitree Go2 with an rclpy `raw=True` subscription.
//!   **It declares `profile_idc` 100 (High)**, which is exactly why it is pinned
//!   here: OpenH264's ENCODER is Constrained-Baseline-only, so "can the chosen
//!   decoder read what the robot actually sends?" is the question the whole
//!   design rests on, and the direct way to answer it is the robot's own bytes.
//!
//! * `fixtures/grey_ramp_12f_320x240_high.h264` — 12 solid-grey frames, produced
//!   by a real x264 encoder (ffmpeg 8.1.2) from 12 hand-written PGM images whose
//!   luma is `20 + 20*i`:
//!
//!   ```text
//!   ffmpeg -framerate 10 -i f%02d.pgm -c:v libx264 -preset veryfast \
//!          -profile:v high -crf 18 -g 6 -bf 0 -pix_fmt yuv420p \
//!          -x264-params "aud=1:repeat-headers=1" -f h264 grey12.h264
//!   ```
//!
//!   High profile (matching the robot), IDRs at frames 0 and 6, no B-frames, and
//!   an Access Unit Delimiter opening every access unit so the split below is
//!   unambiguous rather than a slice-header guess.
//!
//! * `fixtures/color_ramp_8f_320x240_bt601.h264` — 8 solid SATURATED frames, from
//!   8 hand-written PPM images whose pixels are exactly [`EXPECTED_CENTRE_COLORS`]
//!   (red, green, blue, yellow, cyan, magenta, orange, violet), encoded by the
//!   same ffmpeg 8.1.2:
//!
//!   ```text
//!   ffmpeg -framerate 10 -i c%02d.ppm -c:v libx264 -preset veryfast \
//!          -profile:v high -crf 14 -g 4 -bf 0 -pix_fmt yuv420p \
//!          -color_range tv -colorspace smpte170m -color_primaries smpte170m \
//!          -color_trc smpte170m -x264-params "aud=1:repeat-headers=1" \
//!          -f h264 color8.h264
//!   ```
//!
//!   The three `-color_*` flags are the load-bearing part and are verifiable on
//!   the committed file (`ffprobe` reports `color_range=tv`,
//!   `color_space=smpte170m`): they make it **BT.601 limited range**, which is what
//!   `openh264` 0.9.7 applies unconditionally, so this fixture is the case the
//!   shipped conversion gets RIGHT. The BT.709 twin is this same recipe with
//!   `bt709` in place of `smpte170m`.
//!
//!   A SOLID frame is what makes a hand oracle possible: one number identifies a
//!   whole picture, so "which frame is on screen" is a value comparison rather
//!   than an image diff. [`EXPECTED_CENTRE_RGB`] is the ORIGINAL grey ramp — the
//!   bytes written into the input PGMs, not anything read back out of the code
//!   under test — so every image assertion closes the loop from source picture to
//!   rerun store. The 20-level spacing makes an off-by-one frame unmistakable,
//!   which is precisely the discrimination an off-by-one frame check needs.

use cerulion_core::codegen::layout::{LayoutResolver, WireLayout};
use cerulion_core::codegen::{parse_rosmsg, FrameWalker, MessageSchema};
use cerulion_core::wire::WireHeader;
use cerulion_viz::sink::{dispatch_frame, SinkState};
use cerulion_viz::video::StreamKey;
// `len`/`value` on a logged component's arrow array.
use rerun::external::arrow::array::Array as _;

// ────────────────────────────────────────────────────────────────────────────
// Fixtures + the hand oracle
// ────────────────────────────────────────────────────────────────────────────

const GREY_RAMP: &[u8] = include_bytes!("fixtures/grey_ramp_12f_320x240_high.h264");
const COLOR_RAMP: &[u8] = include_bytes!("fixtures/color_ramp_8f_320x240_bt601.h264");
const GO2_REAL_IDR_AU: &[u8] = include_bytes!("fixtures/go2_frontvideostream_360p_idr_au.h264");

/// ELEVEN CONSECUTIVE access units of a Go2's 360p rendition —
/// one keyframe (SPS + PPS + IDR) and the ten P-frames that follow it, verbatim.
///
/// Captured off the running robot on 2026-07-31. The recipe, so it can be redone
/// against a future firmware: with something already holding the topic's mirror
/// (`cerulion viz --robot <name>`), open a listener-less
/// `TransportManager::create_data_only_subscriber("/go2/camera/h264")` — a
/// READ-ONLY tap that demands nothing and cannot disturb the robot's serve —
/// decode each wire frame with the production `FrameWalker` seeded with the
/// workspace's own `examples/go2/schemas/unitree_go/msg/Go2FrontVideoData.msg`
/// (the hash is not in `BUILTIN_MSGS`), take `classify_h264_payload`'s bytes from
/// the units whose `video_height` reads 360, and concatenate them in
/// wire-sequence order starting at a unit whose first NAL is an SPS.
///
/// The capture this file was cut from was gap-free — 0 missing wire sequences
/// across 400 units — so these eleven are genuinely CONSECUTIVE rather than a
/// filtered selection, which is what makes "the decoder survives a whole GOP" a
/// statement about the robot's real stream.
///
/// **Why a GOP and not the single IDR above.** The defect is invisible on
/// one access unit: the decoder used to render a keyframe and its first two
/// P-frames perfectly and only then collapse, so any fixture shorter than four
/// pictures passes on the broken build. The single-IDR fixture stays because it
/// is what the `.cdr` wire test pins; this one is what makes the failure
/// reachable.
const GO2_REAL_GOP: &[u8] = include_bytes!("fixtures/go2_frontvideostream_360p_gop_11au.h264");

/// The Go2 capture's own picture size, cross-checked against the resolution its
/// `video_height` field reports (360) rather than against our own parse.
const GO2_SIZE: StreamKey = StreamKey {
    width: 640,
    height: 360,
};

const GREY_SIZE: StreamKey = StreamKey {
    width: 320,
    height: 240,
};

/// The grey level of each SOURCE image — the HAND ORACLE.
///
/// This is `20 + 20*i`: literally the byte written into frame `i` of the input
/// PGMs, NOT a value read back out of the code under test. So an image assertion
/// here closes the whole loop — original picture → x264 → Annex-B → wire frame →
/// demux → our decoder → RGB → the rerun store — against a number chosen by hand
/// before any of it ran.
///
/// The pixels arrive as RGB and the source is neutral grey, so R == G == B == this
/// value ([`LoggedImage::is_grey`] asserts the equality separately). Note the
/// round trip is NOT the identity on paper: H.264 carries LIMITED-range luma, so
/// these full-range levels are compressed on the way in (frame 0's luma plane
/// reads 33) and expanded again by the YUV→RGB conversion. Landing back within
/// [`RGB_TOLERANCE`] of the original is the real evidence that the colour
/// conversion is right — a decoder that skipped the range expansion would be
/// wrong by ~13 levels here and fail.
const EXPECTED_CENTRE_RGB: [u8; 12] = [20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240];

/// The neighbour gap in [`EXPECTED_CENTRE_RGB`], in grey levels.
const RGB_STEP: i32 = 20;

/// How many pictures openh264 is still holding when the last access
/// unit of a stream has been fed.
///
/// The decoder runs with openh264's flush-after-decode OFF, because its default
/// forced a picture out down a path that never releases the picture's reference —
/// after three of those the picture buffer (`iNumRefFrames + 2` = 3) was empty,
/// openh264 reported `dsOutOfMemory` and RESET itself, reducing a real stream to
/// three frames per 30-frame GOP. See `video_decode::StreamDecoder` for the
/// chain. Letting openh264 release on its own path costs exactly one picture of
/// pipeline: the first unit after a decoder opens yields no picture, and each one
/// after that yields the PREVIOUS unit's.
///
/// So a run of N access units renders `N - PIPELINE_DEPTH` images, and image `k`
/// is still unit `k`'s picture — the ORDER and CONTENT oracles below are
/// unchanged, only the count is one shorter. Expressing that as one named
/// constant rather than eleven hand-edited numbers is deliberate: it is a single
/// property of the decoder, and a future change to it should fail in one place
/// with a reason attached.
const PIPELINE_DEPTH: usize = 1;

/// How far a decoded centre pixel may sit from its oracle value.
///
/// Absorbs the lossy encode (`crf 18`), the limited-range round trip and the
/// YUV→RGB rounding — measured worst case is 1 level. The const-assert below is
/// what keeps it sound: at `2 * tolerance < RGB_STEP` every frame stays strictly
/// nearer its OWN oracle than to either neighbour's, so a frame served in place of
/// its neighbour cannot pass.
const RGB_TOLERANCE: i32 = 6;

const _: () = assert!(
    RGB_TOLERANCE * 2 < RGB_STEP,
    "tolerance must not blur neighbouring frames"
);

/// Split an Annex-B buffer into access units at every AUD (`nal_unit_type` 9).
///
/// The fixture is encoded with x264 `aud=1`, so every access unit opens with an
/// Access Unit Delimiter and this needs no slice-header parsing.
fn split_access_units(buf: &[u8]) -> Vec<&[u8]> {
    let mut starts = Vec::new();
    let mut i = 0usize;
    while i + 4 < buf.len() {
        if buf[i] == 0 && buf[i + 1] == 0 {
            let (hdr, sc) = if buf[i + 2] == 1 {
                (buf[i + 3], 3usize)
            } else if buf[i + 2] == 0 && buf[i + 3] == 1 {
                (buf[i + 4], 4usize)
            } else {
                i += 1;
                continue;
            };
            if hdr & 0x1F == 9 {
                starts.push(i);
            }
            i += sc;
            continue;
        }
        i += 1;
    }
    starts
        .iter()
        .enumerate()
        .map(|(k, s)| &buf[*s..starts.get(k + 1).copied().unwrap_or(buf.len())])
        .collect()
}

/// Split the Go2 GOP fixture into access units.
///
/// The robot emits NO access unit delimiter, so [`split_access_units`]'s AUD rule
/// does not apply. Its stream has a simpler, verifiable shape instead — measured
/// across the whole 400-unit capture, every unit is either a lone non-IDR slice
/// (NAL type 1) or exactly `SPS, PPS, IDR` (7, 8, 5) — so an access unit begins
/// at every type-7 and every type-1 NAL. `the_go2_gop_fixture_is_the_stream_its_
/// oracle_describes` asserts that shape on the committed bytes, so this rule
/// cannot silently start matching something else.
fn split_go2_access_units(buf: &[u8]) -> Vec<&[u8]> {
    let mut starts = Vec::new();
    let mut i = 0usize;
    while i + 5 <= buf.len() {
        if buf[i] == 0 && buf[i + 1] == 0 && buf[i + 2] == 0 && buf[i + 3] == 1 {
            let kind = buf[i + 4] & 0x1F;
            if kind == 7 || kind == 1 {
                starts.push(i);
            }
            i += 5;
            continue;
        }
        i += 1;
    }
    starts
        .iter()
        .enumerate()
        .map(|(k, s)| &buf[*s..starts.get(k + 1).copied().unwrap_or(buf.len())])
        .collect()
}

/// ANTI-TAUTOLOGY on the Go2 GOP fixture: every oracle that feeds it is
/// meaningless if the committed bytes are not the stream its docs claim.
#[test]
fn the_go2_gop_fixture_is_the_stream_its_oracle_describes() {
    let aus = split_go2_access_units(GO2_REAL_GOP);
    assert_eq!(aus.len(), 11, "fixture must carry 11 access units");

    let shapes: Vec<Vec<u8>> = aus
        .iter()
        .map(|au| {
            cerulion_viz::video::scan_annex_b(au)
                .expect("every unit must scan as Annex-B")
                .nals
                .iter()
                .map(|n| n.kind)
                .collect()
        })
        .collect();
    assert_eq!(
        shapes[0],
        vec![7u8, 8, 5],
        "unit 0 must be the keyframe: SPS, PPS, IDR"
    );
    for (i, s) in shapes.iter().enumerate().skip(1) {
        assert_eq!(s, &vec![1u8], "unit {i} must be a lone non-IDR slice");
    }

    // The keyframe's SPS is what makes this a fixture rather than any
    // High-profile stream: POC type 0 is the shape whose POC steps by 2, which is
    // what openh264's inline-release gate (`iMinPOC - iLastWrittenPOC <= 1`)
    // cannot satisfy. If a future capture lands here with POC type 2, the
    // regression pin below would silently stop reproducing the defect.
    let sps = cerulion_viz::video::scan_annex_b(aus[0])
        .expect("keyframe scans")
        .sps()
        .expect("keyframe carries an SPS")
        .bytes
        .to_vec();
    use h264_reader::nal::{sps::SeqParameterSet, Nal as _, RefNal};
    let parsed = SeqParameterSet::from_bits(RefNal::new(&sps, &[], true).rbsp_bits())
        .expect("the robot's SPS parses");
    assert_eq!(
        parsed.pixel_dimensions().expect("dims"),
        (GO2_SIZE.width, GO2_SIZE.height)
    );
    assert!(
        matches!(
            parsed.pic_order_cnt,
            h264_reader::nal::sps::PicOrderCntType::TypeZero { .. }
        ),
        "the fixture must declare pic_order_cnt_type 0 — the shape the GOP-collapse fix is \
         about; got {:?}",
        parsed.pic_order_cnt
    );
    assert_eq!(
        parsed.max_num_ref_frames, 1,
        "openh264 sizes its picture buffer at max_num_ref_frames + 2, so this is \
         what makes the leak surface on the FOURTH picture"
    );
}

#[test]
fn the_fixture_is_the_twelve_access_unit_ramp_the_oracle_describes() {
    // Anti-tautology guard on the INPUT: every oracle below is meaningless if the
    // committed fixture is not the stream its docs claim. Two IDRs (frames 0 and
    // 6) is what the stall test depends on.
    let aus = split_access_units(GREY_RAMP);
    assert_eq!(aus.len(), 12, "fixture must carry 12 access units");
    assert_eq!(aus.len(), EXPECTED_CENTRE_RGB.len());
    let idrs: Vec<usize> = aus
        .iter()
        .enumerate()
        .filter(|(_, au)| {
            cerulion_viz::video::scan_annex_b(au).is_some_and(|s| s.sps().is_some() && s.has_idr())
        })
        .map(|(i, _)| i)
        .collect();
    assert_eq!(idrs, vec![0, 6], "fixture must open a GOP at 0 and at 6");
}

// ────────────────────────────────────────────────────────────────────────────
// Wire-frame construction (crib: video_h264_test.rs)
// ────────────────────────────────────────────────────────────────────────────

const PROBE_MSG: &str = "uint64 time_frame
uint32 video_height
uint8[] video_data
";
const PROBE_QNAME: &str = "probe/VideoProbe";

fn probe_schema() -> MessageSchema {
    parse_rosmsg(PROBE_MSG, "VideoProbe", Some("probe")).expect("probe schema parses")
}

fn all_schemas() -> Vec<MessageSchema> {
    let mut schemas: Vec<MessageSchema> = Vec::new();
    for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
        if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
            schemas.push(s);
        }
    }
    schemas.push(probe_schema());
    schemas
}

fn probe_walker() -> FrameWalker {
    FrameWalker::new(all_schemas()).0
}

fn probe_layout() -> WireLayout {
    LayoutResolver::new(all_schemas())
        .0
        .layout_of(PROBE_QNAME)
        .expect("probe layout")
}

fn field_offset(layout: &WireLayout, field: &str) -> usize {
    layout
        .fixed_fields
        .iter()
        .find(|f| f.name == field)
        .unwrap_or_else(|| panic!("fixed field '{field}' not in the probe layout"))
        .offset
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
    // One variable field, so entry 0 is `video_data`: [offset u32][length u32].
    payload[fixed..fixed + 4].copy_from_slice(&data_off.to_le_bytes());
    payload[fixed + 4..fixed + 8].copy_from_slice(&(au.len() as u32).to_le_bytes());
    payload.extend_from_slice(au);

    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: probe_schema().schema_hash(),
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed) as u32,
        offset_table_count: 1,
        sequence: 0,
        timestamp_ns,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

// ────────────────────────────────────────────────────────────────────────────
// Rerun memory-sink readback
// ────────────────────────────────────────────────────────────────────────────

fn memory() -> (rerun::RecordingStream, rerun::sink::MemorySinkStorage) {
    rerun::RecordingStreamBuilder::new("test")
        .recording_id("video_decode_test")
        .memory()
        .expect("memory sink")
}

/// One logged image: its entity, resolution and RGB bytes, read back off the REAL
/// rerun store — so these assertions observe what a VIEWER would receive, not
/// what the sink believes it sent.
#[derive(Debug, Clone)]
struct LoggedImage {
    entity: String,
    rgb: Vec<u8>,
    width: u32,
    height: u32,
    /// The value on the `robot_time` timeline this image was logged at, when the
    /// chunk carries one.
    ///
    /// This is what makes the SINK's choice of stamp observable. The
    /// decoder hands back the picture's OWN access unit's timestamp
    /// (`DecodedFrame::timestamp_ns`), and the sink must log THAT rather than the
    /// timestamp of the unit it was just handed — otherwise every rendered frame
    /// is labelled one frame-period newer than it is. Without reading it back
    /// here, that wiring was pinned by nothing: swapping the sink to the caller's
    /// timestamp left the entire suite green.
    time_ns: Option<i64>,
}

impl LoggedImage {
    /// The centre pixel's red channel. A solid grey frame has R == G == B, and
    /// [`LoggedImage::is_grey`] asserts that separately, so one channel identifies
    /// the picture.
    fn centre_luma(&self) -> u8 {
        let idx = ((self.height as usize / 2) * self.width as usize + self.width as usize / 2) * 3;
        self.rgb[idx]
    }

    /// Whether the centre pixel really is a neutral grey (R == G == B within the
    /// chroma round-trip tolerance) — the guard that keeps `centre_luma` from
    /// silently reading one channel of a colour-mangled frame.
    fn is_grey(&self) -> bool {
        let idx = ((self.height as usize / 2) * self.width as usize + self.width as usize / 2) * 3;
        let (r, g, b) = (
            self.rgb[idx] as i32,
            self.rgb[idx + 1] as i32,
            self.rgb[idx + 2] as i32,
        );
        (r - g).abs() <= RGB_TOLERANCE && (g - b).abs() <= RGB_TOLERANCE
    }
}

/// Every chunk the sink emitted. `take()` DRAINS, so each call returns only what
/// has been logged since the last one.
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

/// `(entity, component descriptor)` for every component of every chunk.
fn logged_components(chunks: &[rerun::log::Chunk]) -> Vec<(String, String)> {
    let mut out = Vec::new();
    for chunk in chunks {
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

/// The `<W>x<H>` a `viz-video` entity's last segment declares.
///
/// Reading the resolution off the ENTITY rather than out of the logged
/// `Image:format` component is deliberate: it makes every image assertion a
/// CROSS-CHECK between two independently produced facts — the demux's SPS-derived
/// rendition key (which names the entity) and the decoder's own output size
/// (which sizes the buffer). A decoder that produced the wrong size would satisfy
/// a self-consistent format component and fail here.
fn resolution_from_entity(entity: &str) -> (u32, u32) {
    let seg = entity.rsplit('/').next().expect("a last segment");
    let (w, h) = seg.split_once('x').expect("a <W>x<H> rendition segment");
    (w.parse().expect("width"), h.parse().expect("height"))
}

/// Walk down an arrow array to the `u8` leaf and take its bytes.
///
/// An `Image:buffer` is a rerun `Blob`, so one logged image arrives as a
/// `List<List<u8>>` cell: the outer level is rerun's per-row component list, the
/// inner one the blob itself. Descending until the leaf keeps this readback from
/// depending on how many list levels a given rerun minor wraps it in.
fn flatten_to_u8(arr: rerun::external::arrow::array::ArrayRef) -> Vec<u8> {
    if let Some(u8s) = arr
        .as_any()
        .downcast_ref::<rerun::external::arrow::array::UInt8Array>()
    {
        return u8s.values().to_vec();
    }
    if let Some(inner) = arr
        .as_any()
        .downcast_ref::<rerun::external::arrow::array::ListArray>()
    {
        let mut out = Vec::new();
        for i in 0..inner.len() {
            out.extend(flatten_to_u8(inner.value(i)));
        }
        return out;
    }
    panic!("an image buffer must bottom out in a u8 array, got {arr:?}");
}

/// Every `Image` logged, in log order, with its pixels read back out.
fn logged_images(chunks: &[rerun::log::Chunk]) -> Vec<LoggedImage> {
    let mut out = Vec::new();
    for chunk in chunks {
        let entity = chunk
            .entity_path()
            .to_string()
            .trim_start_matches('/')
            .to_string();
        for (descr, list) in chunk.components().iter() {
            if !descr.as_str().contains("Image:buffer") {
                continue;
            }
            let (width, height) = resolution_from_entity(&entity);
            // The `robot_time` column, row-aligned with the component list.
            let times: Option<&[i64]> = chunk
                .timelines()
                .iter()
                .find(|(name, _)| name.as_str() == "robot_time")
                .map(|(_, col)| col.times_raw());
            for i in 0..list.list_array.len() {
                let rgb = flatten_to_u8(list.list_array.value(i));
                out.push(LoggedImage {
                    entity: entity.clone(),
                    rgb,
                    width,
                    height,
                    time_ns: times.and_then(|t| t.get(i).copied()),
                });
            }
        }
    }
    out
}

/// Drive the whole grey-ramp fixture through the production dispatch path and
/// return everything the viewer would have received.
fn run_ramp(input: &str, aus: &[&[u8]]) -> (Vec<LoggedImage>, Vec<(String, String)>, SinkState) {
    let (rec, storage) = memory();
    let walker = probe_walker();
    let mut state = SinkState::new();
    for (i, au) in aus.iter().enumerate() {
        let frame = build_probe_frame(i as u64, GREY_SIZE.height, au, 1_000 + i as u64 * 100);
        dispatch_frame(&rec, &walker, input, &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");
    let c = chunks(&storage);
    (logged_images(&c), logged_components(&c), state)
}

// ────────────────────────────────────────────────────────────────────────────
// The pins
// ────────────────────────────────────────────────────────────────────────────

/// THE headline: every access unit becomes a decoded picture, at the rendition's
/// entity, with the RIGHT picture in the right order — checked against the hand
/// oracle rather than against a second run of the same code.
#[test]
fn every_access_unit_renders_as_its_own_decoded_image_in_order() {
    let aus = split_access_units(GREY_RAMP);
    let (images, _components, state) = run_ramp("/probe/camera/h264", &aus);

    // One image per access unit, less the one openh264 is still holding when the
    // stream ends (see PIPELINE_DEPTH). Image `k` is still unit `k`'s picture, so
    // the ORDER + CONTENT oracle below is unchanged.
    assert_eq!(
        images.len(),
        EXPECTED_CENTRE_RGB.len() - PIPELINE_DEPTH,
        "one image per access unit, less the pipeline's held picture"
    );
    let expected_entity = "world/probe/camera/h264/viz-video/320x240";
    for (i, img) in images.iter().enumerate() {
        assert_eq!(img.entity, expected_entity, "image {i} at the wrong entity");
        assert_eq!(
            (img.width, img.height),
            (GREY_SIZE.width, GREY_SIZE.height),
            "image {i} has the wrong resolution"
        );
        assert_eq!(
            img.rgb.len(),
            (GREY_SIZE.width * GREY_SIZE.height * 3) as usize,
            "image {i} is not tightly packed RGB8"
        );
        assert!(
            img.is_grey(),
            "image {i} is not neutral grey — colour mangled"
        );
        let got = img.centre_luma() as i32;
        let want = EXPECTED_CENTRE_RGB[i] as i32;
        assert!(
            (got - want).abs() <= RGB_TOLERANCE,
            "image {i}: centre luma {got}, oracle {want} (tolerance {RGB_TOLERANCE})"
        );
    }
    assert_eq!(
        state
            .video_decoders()
            .frames_decoded("/probe/camera/h264", GREY_SIZE),
        (12 - PIPELINE_DEPTH) as u64
    );
    assert_eq!(
        state
            .video_decoders()
            .decode_failures("/probe/camera/h264", GREY_SIZE),
        0,
        "a clean stream must not refuse anything"
    );
}

/// The REGRESSION GUARD for the whole issue: nothing on this path may go back to
/// handing the viewer a `VideoStream` to decode.
///
/// This is the assertion that fails if `render_video_sample` is reverted to
/// the first H.264 route's behaviour, and it is stated on what the rerun store actually holds,
/// so it cannot be satisfied by a sink that merely believes it logged an image.
#[test]
fn nothing_is_logged_as_a_playback_buffered_video_stream() {
    let aus = split_access_units(GREY_RAMP);
    let (images, components, _) = run_ramp("/probe/camera/h264", &aus);

    assert!(
        !components.iter().any(|(_, c)| c.contains("VideoStream")),
        "a VideoStream component reached the viewer — the desk-side decode was \
         bypassed and the viewer's ffmpeg frame-threading lag (measured 567 ms) is \
         back. Components: {components:?}"
    );
    assert!(
        components.iter().any(|(_, c)| c.contains("Image:buffer")),
        "no Image reached the viewer at all: {components:?}"
    );
    assert!(!images.is_empty());
}

/// A mid-stream GAP (the network dropped some access units) must NOT render a
/// corrupt picture, must be COUNTED, and must resume on its own at the next IDR.
///
/// The oracle is the measured behaviour of a decoder with error concealment OFF:
/// units whose references are missing are REFUSED, and the frames from the next
/// keyframe on decode normally.
#[test]
fn a_stall_refuses_undecodable_units_and_resumes_at_the_next_keyframe() {
    let aus = split_access_units(GREY_RAMP);
    // Feed 0,1 (the first GOP's IDR and one P-frame), then SKIP 2..=4 — the gap —
    // then 5 (whose references are now missing) and the whole second GOP, 6..=11.
    let fed: Vec<usize> = vec![0, 1, 5, 6, 7, 8, 9, 10, 11];
    let picked: Vec<&[u8]> = fed.iter().map(|i| aus[*i]).collect();

    let (rec, storage) = memory();
    let walker = probe_walker();
    let mut state = SinkState::new();
    for (n, au) in picked.iter().enumerate() {
        let frame = build_probe_frame(n as u64, GREY_SIZE.height, au, 1_000 + n as u64 * 100);
        dispatch_frame(&rec, &walker, "/probe/gap/h264", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");
    let images = logged_images(&chunks(&storage));

    // 9 units fed, exactly ONE of them undecodable (index 5, orphaned by the gap).
    assert_eq!(
        state
            .video_decoders()
            .decode_failures("/probe/gap/h264", GREY_SIZE),
        1,
        "the orphaned unit must be counted, not silently skipped"
    );
    // 9 units, one refused, and one picture still held by the pipeline.
    assert_eq!(
        images.len(),
        9 - 1 - PIPELINE_DEPTH,
        "pictures from 9 units (one refused, one still in the pipeline)"
    );
    assert_eq!(
        state
            .video_decoders()
            .frames_decoded("/probe/gap/h264", GREY_SIZE),
        (9 - 1 - PIPELINE_DEPTH) as u64
    );

    // Nothing corrupt was rendered for the orphan, and the RESUME lands exactly on
    // the second GOP's own frames — the hand oracle for units 6..=11, less the
    // last one, which is the picture the pipeline is still holding.
    //
    // The two images BEFORE the resume are units 0 and 1: the pipeline held unit
    // 0's picture until unit 1 pushed it out, and it was still holding unit 1's
    // when the orphaned unit was refused. That refusal is NOT one of openh264's
    // two context-RESETTING arms, so the held picture survived it and came out on
    // the keyframe that follows — which is why `StreamDecoder`'s stamp queue
    // clears only on the resetting arms.
    let resumed: Vec<i32> = images[2..].iter().map(|i| i.centre_luma() as i32).collect();
    let want: Vec<i32> = (6..12 - PIPELINE_DEPTH)
        .map(|i| EXPECTED_CENTRE_RGB[i] as i32)
        .collect();
    // And every one of them is logged at ITS OWN unit's time.
    //
    // This is the arm that pins how the stamp queue survives a refusal, and both
    // wrong rules are reachable here. The fed units carry `1_000 + n * 100` at FED
    // index n, and the seven pictures come from fed indices 0, 1, 3, 4, 5, 6, 7 —
    // the orphan at index 2 yields none. CLEARING the queue on a non-resetting
    // refusal mis-stamps the picture that follows it; KEEPING the orphan's own
    // stamp shifts every later picture by one unit for the rest of the run.
    let stamps: Vec<i64> = images
        .iter()
        .map(|i| i.time_ns.expect("every image carries a robot_time"))
        .collect();
    assert_eq!(
        stamps,
        vec![1_000, 1_100, 1_300, 1_400, 1_500, 1_600, 1_700],
        "each picture must carry the time of the unit it came FROM; the orphaned \
         unit contributes no picture and so no stamp"
    );

    for (got, want) in resumed.iter().zip(want.iter()) {
        assert!(
            (got - want).abs() <= RGB_TOLERANCE,
            "after the stall the stream must resume on its own frames: got {resumed:?}, oracle {want:?}"
        );
    }
}

/// A mid-GOP ATTACH — the ordinary case for a desk that connects while the robot
/// is already streaming — renders nothing until the first keyframe, and renders
/// correctly from there.
#[test]
fn a_mid_gop_attach_renders_nothing_until_the_first_keyframe() {
    let aus = split_access_units(GREY_RAMP);
    // Start at unit 3: three P-frames with no parameter set yet, then the IDR at 6.
    let picked: Vec<&[u8]> = (3..9).map(|i| aus[i]).collect();

    let (rec, storage) = memory();
    let walker = probe_walker();
    let mut state = SinkState::new();
    for (n, au) in picked.iter().enumerate() {
        let frame = build_probe_frame(n as u64, GREY_SIZE.height, au, 1_000 + n as u64 * 100);
        dispatch_frame(&rec, &walker, "/probe/midgop/h264", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");
    let images = logged_images(&chunks(&storage));

    // The DEMUX gates the three pre-parameter-set units (no sub-stream exists to
    // route them to), so they never reach a decoder at all — a drop counted as the
    // ordinary attach transient, not as a decode failure.
    assert_eq!(
        state.video().dropped_before_keyframe("/probe/midgop/h264"),
        3,
        "the pre-keyframe units must be dropped at the demux and counted"
    );
    assert_eq!(
        images.len(),
        3 - PIPELINE_DEPTH,
        "units 6,7,8 render once the IDR opens the stream, less the picture the \
         pipeline is still holding"
    );
    for (k, img) in images.iter().enumerate() {
        let want = EXPECTED_CENTRE_RGB[6 + k] as i32;
        let got = img.centre_luma() as i32;
        assert!(
            (got - want).abs() <= RGB_TOLERANCE,
            "post-keyframe image {k}: got {got}, oracle {want}"
        );
    }
}

/// THE decoder-choice pin: the chosen decoder reads what the ROBOT actually
/// sends.
///
/// The Go2's parameter set declares `profile_idc` 100 (High) while OpenH264's
/// ENCODER is Constrained-Baseline-only, so this is not a formality — if the
/// decoder could not read High profile, desk-side decode would be the wrong design
/// and every other test here (which uses our own encoder's output) would still
/// pass. Real captured robot bytes, and the resolution is cross-checked against
/// the capture's own `video_height` field rather than against our SPS parse.
#[test]
fn the_real_go2_high_profile_access_unit_decodes_on_this_desk() {
    let (rec, storage) = memory();
    let walker = probe_walker();
    let mut state = SinkState::new();
    // TWO units: the keyframe, then the P-frame that follows it in the live
    // stream. openh264 holds the keyframe's picture for one call (see
    // PIPELINE_DEPTH), so a single unit renders nothing and this pin would be
    // vacuous — it would assert 0 images and pass on a decoder that reads
    // nothing at all.
    let gop = split_go2_access_units(GO2_REAL_GOP);
    for (n, au) in gop.iter().take(2).enumerate() {
        let frame = build_probe_frame(n as u64, GO2_SIZE.height, au, 42_000 + n as u64);
        dispatch_frame(&rec, &walker, "/go2/camera/h264", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");

    let images = logged_images(&chunks(&storage));
    assert_eq!(
        images.len(),
        2 - PIPELINE_DEPTH,
        "the real Go2 keyframe must render one image"
    );
    let img = &images[0];
    assert_eq!(img.entity, "world/go2/camera/h264/viz-video/640x360");
    assert_eq!((img.width, img.height), (GO2_SIZE.width, GO2_SIZE.height));
    assert_eq!(img.rgb.len(), (640 * 360 * 3) as usize);
    assert_eq!(
        state
            .video_decoders()
            .decode_failures("/go2/camera/h264", GO2_SIZE),
        0,
        "the robot's own High-profile keyframe must not be refused"
    );
    // Not a blank frame: a real camera picture has more than one distinct value.
    let distinct = img
        .rgb
        .chunks(3)
        .map(|p| p[0])
        .collect::<std::collections::BTreeSet<_>>();
    assert!(
        distinct.len() > 8,
        "decoded Go2 frame is nearly uniform ({} distinct luma values) — decoded to garbage?",
        distinct.len()
    );
}

/// **THE GOP-COLLAPSE REGRESSION PIN**: a Go2 GOP decodes ALL THE WAY THROUGH.
///
/// The defect it guards is not "video is slow" — it is that the decoder DESTROYED
/// ITSELF four pictures into every GOP. openh264 buffers each picture on its
/// non-baseline path, tries to release it inline only when
/// `iMinPOC - iLastWrittenPOC <= 1`, and the Go2 steps POC by 2 — so the release
/// fell through to `FlushFrame`, whose
/// `ReleaseBufferedReadyPictureNoReorder(pCtx = NULL, …)` skips the
/// `--pPic->iRefCount` because `m_pPicBuff` is populated only on openh264's
/// THREADED path. Every access unit therefore consumed one slot of a
/// `max_num_ref_frames + 2` = 3 picture buffer permanently; on the fourth,
/// `PrefetchPic` returned NULL, openh264 raised `dsOutOfMemory` and called
/// `ResetDecoder()`, which discards the parameter sets — so every remaining unit
/// of the GOP failed `dsNoParamSets` in microseconds until the next keyframe.
/// Measured on the live robot: 21 of 200 access units rendered.
///
/// This is the arm that FAILS if `StreamDecoder::new` goes back to the crate's
/// default flush-after-decode. MEASURED against reverting to the default: 3 images and 7
/// decode failures, where the shipped decoder yields 10 and 0.
///
/// The oracle is deliberately EXACT on both numbers. "More than three images"
/// would also pass a decoder that stalls later, and `decode_failures == 0` is
/// what says nothing was quietly refused along the way.
#[test]
fn a_live_go2_gop_decodes_every_frame_without_the_decoder_resetting_itself() {
    let aus = split_go2_access_units(GO2_REAL_GOP);
    let (rec, storage) = memory();
    let walker = probe_walker();
    let mut state = SinkState::new();
    for (n, au) in aus.iter().enumerate() {
        let frame = build_probe_frame(n as u64, GO2_SIZE.height, au, 1_000 + n as u64 * 100);
        dispatch_frame(&rec, &walker, "/go2/camera/h264", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");
    let images = logged_images(&chunks(&storage));

    assert_eq!(
        state
            .video_decoders()
            .decode_failures("/go2/camera/h264", GO2_SIZE),
        0,
        "the robot's own GOP must decode end to end — a nonzero count here is the \
         mid-GOP collapse (openh264 reset itself mid-GOP and then refused every \
         unit until the next keyframe)"
    );
    assert_eq!(
        images.len(),
        aus.len() - PIPELINE_DEPTH,
        "every access unit but the one still in the pipeline must render"
    );
    assert_eq!(
        state
            .video_decoders()
            .frames_decoded("/go2/camera/h264", GO2_SIZE),
        (aus.len() - PIPELINE_DEPTH) as u64
    );

    // Real camera pictures, not blanks — the same "did it decode to garbage"
    // guard the single-keyframe pin uses, applied to EVERY rendered frame so a
    // decoder that emits one good picture and then grey cannot pass.
    for (k, img) in images.iter().enumerate() {
        assert_eq!(img.entity, "world/go2/camera/h264/viz-video/640x360");
        assert_eq!((img.width, img.height), (GO2_SIZE.width, GO2_SIZE.height));
        let distinct = img
            .rgb
            .chunks(3)
            .map(|p| p[0])
            .collect::<std::collections::BTreeSet<_>>();
        assert!(
            distinct.len() > 8,
            "image {k} is nearly uniform ({} distinct luma values) — decoded to \
             garbage?",
            distinct.len()
        );
    }
}

/// The picture a decode returns carries ITS OWN access unit's wire
/// timestamp, not the timestamp of the unit that pushed it out.
///
/// openh264 holds a picture for one call, so the naive stamp — the one the caller
/// just handed in — labels every frame one frame-period NEWER than it is. This is
/// a unit-level oracle rather than a rendered-image one because
/// `archetype::set_robot_time` picks its rerun timeline off a process-global
/// `OnceLock`, which would make a chunk-timeline readback order-dependent across
/// this binary.
///
/// The stamps are deliberately far apart and unrelated to any index, so an
/// off-by-one cannot be mistaken for a rounding difference.
#[test]
fn a_decoded_picture_carries_its_own_access_units_timestamp() {
    let aus = split_go2_access_units(GO2_REAL_GOP);
    let stamps: Vec<u64> = (0..aus.len())
        .map(|i| 7_000_000 + i as u64 * 33_333)
        .collect();

    let mut pool = cerulion_viz::video_decode::VideoDecoders::new();
    let mut got: Vec<u64> = Vec::new();
    for (au, stamp) in aus.iter().zip(stamps.iter()) {
        if let cerulion_viz::video_decode::DecodeOutcome::Frame(frame) =
            pool.decode("/go2/ts/h264", GO2_SIZE, au, *stamp)
        {
            got.push(frame.timestamp_ns);
        }
    }

    // Every picture but the last is emitted, each carrying the stamp of the unit
    // it came FROM — i.e. the stamps in order, one short.
    assert_eq!(
        got,
        stamps[..stamps.len() - PIPELINE_DEPTH].to_vec(),
        "a picture must carry its own access unit's stamp; stamping with the \
         caller's would yield {:?}",
        &stamps[PIPELINE_DEPTH..]
    );
}

/// The SINK logs the picture's own stamp, not the stamp of the access
/// unit that pushed it out.
///
/// The sibling above pins the DECODER's carry; this pins the WIRING, and the two
/// are separate for a measured reason — swapping `frame.timestamp_ns` back to the
/// caller's `timestamp_ns` at the `log_raw_image` call site left the entire
/// `cerulion_viz` suite green, because nothing read the timeline back.
///
/// `run_ramp` stamps unit `i` at `1_000 + i * 100`, so a correct sink puts image
/// `k` on `robot_time` `1_000 + k * 100` and an off-by-one variant puts it on
/// `1_100 + k * 100` — distinct on every row.
#[test]
fn the_sink_logs_each_picture_at_its_own_access_units_time() {
    let aus = split_access_units(GREY_RAMP);
    let (images, _, _) = run_ramp("/probe/imgtime/h264", &aus);
    assert!(
        !images.is_empty(),
        "precondition: something must have rendered"
    );

    let got: Vec<i64> = images
        .iter()
        .map(|i| {
            i.time_ns
                .expect("every logged image must carry a robot_time value")
        })
        .collect();
    let want: Vec<i64> = (0..images.len()).map(|k| 1_000 + k as i64 * 100).collect();
    assert_eq!(
        got,
        want,
        "each picture must be logged at ITS OWN access unit's wire time; \
         stamping with the unit that flushed it would give {:?}",
        (0..images.len())
            .map(|k| 1_100 + k as i64 * 100)
            .collect::<Vec<_>>()
    );
}

/// Two renditions on ONE topic get one decoder EACH, and neither is fed the
/// other's frames.
///
/// The Go2 interleaves 360p and 720p on `/frontvideostream`; a single decoder
/// cannot survive that. The demux already separates them — this pins that the
/// DECODERS follow the same split, by observing that each rendition's images carry
/// its own resolution.
#[test]
fn two_renditions_decode_independently() {
    let ramp = split_access_units(GREY_RAMP);
    let (rec, storage) = memory();
    let walker = probe_walker();
    let mut state = SinkState::new();

    // Interleave the grey ramp's first GOP with the real Go2 keyframe, on ONE
    // input: two distinct SPS-declared resolutions, so two sub-streams.
    let plan: Vec<(&[u8], u32)> = vec![
        (ramp[0], GREY_SIZE.height),
        (GO2_REAL_IDR_AU, GO2_SIZE.height),
        (ramp[1], GREY_SIZE.height),
        (ramp[2], GREY_SIZE.height),
    ];
    for (n, (au, h)) in plan.iter().enumerate() {
        let frame = build_probe_frame(n as u64, *h, au, 1_000 + n as u64 * 100);
        dispatch_frame(&rec, &walker, "/probe/multi/h264", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");
    let images = logged_images(&chunks(&storage));

    let mut streams = state.video().streams("/probe/multi/h264");
    streams.sort();
    assert_eq!(streams, vec![GREY_SIZE, GO2_SIZE], "both renditions opened");

    let grey: Vec<&LoggedImage> = images
        .iter()
        .filter(|i| i.entity.ends_with("320x240"))
        .collect();
    let go2: Vec<&LoggedImage> = images
        .iter()
        .filter(|i| i.entity.ends_with("640x360"))
        .collect();
    // Each rendition has its OWN decoder, so each pays its own PIPELINE_DEPTH.
    assert_eq!(
        grey.len(),
        3 - PIPELINE_DEPTH,
        "the grey rendition's three units, less its pipeline's held picture"
    );
    assert_eq!(
        go2.len(),
        1 - PIPELINE_DEPTH,
        "the Go2 rendition's single unit is still IN its pipeline"
    );
    for img in &grey {
        assert_eq!((img.width, img.height), (GREY_SIZE.width, GREY_SIZE.height));
    }
    for img in &go2 {
        assert_eq!((img.width, img.height), (GO2_SIZE.width, GO2_SIZE.height));
    }
    // Each decoder saw only its own rendition's units — the cross-feed guard.
    assert_eq!(
        state
            .video_decoders()
            .frames_decoded("/probe/multi/h264", GREY_SIZE),
        (3 - PIPELINE_DEPTH) as u64
    );
    assert_eq!(
        state
            .video_decoders()
            .frames_decoded("/probe/multi/h264", GO2_SIZE),
        (1 - PIPELINE_DEPTH) as u64
    );
    assert_eq!(
        state
            .video_decoders()
            .decode_failures("/probe/multi/h264", GREY_SIZE)
            + state
                .video_decoders()
                .decode_failures("/probe/multi/h264", GO2_SIZE),
        0,
        "neither decoder was fed the other's frames"
    );
}

/// Two runs over the identical access-unit sequence produce byte-identical
/// pictures (Principle #7). The oracle anchors run 1, so this is not a bare
/// self-compare.
#[test]
fn decoding_is_deterministic_across_runs() {
    let aus = split_access_units(GREY_RAMP);
    let (a, _, _) = run_ramp("/probe/det/h264", &aus);
    let (b, _, _) = run_ramp("/probe/det/h264", &aus);

    // NOT vacuous: a change that renders NOTHING would make two empty runs
    // trivially equal, so the count is pinned to the oracle's length first.
    assert_eq!(
        a.len(),
        EXPECTED_CENTRE_RGB.len() - PIPELINE_DEPTH,
        "run 1 rendered nothing"
    );
    assert_eq!(a.len(), b.len());
    for (i, (x, y)) in a.iter().zip(b.iter()).enumerate() {
        assert_eq!(x.rgb, y.rgb, "image {i} differs between two runs");
        let want = EXPECTED_CENTRE_RGB[i] as i32;
        assert!(
            (x.centre_luma() as i32 - want).abs() <= RGB_TOLERANCE,
            "run 1 image {i} must also match the oracle, not just run 2"
        );
    }
}

/// A decode-failure regime is LOUD ONCE and then suppressed — not one warn per
/// frame at 30 Hz (the disk-fill class), and the counter stays exact
/// whatever the log level.
#[test]
#[tracing_test::traced_test]
fn a_decode_failure_regime_is_loud_once_then_suppressed_and_recovers() {
    let aus = split_access_units(GREY_RAMP);
    // Open the stream on the first GOP's IDR, then feed a run of units the demux
    // ACCEPTS but no decoder can render — a valid non-IDR NAL header (`0x41`) over
    // a payload that is not a coded slice. That is the shape a corrupted or
    // partially-delivered frame takes on the wire: it routes (the demux reads
    // headers, not slice data) and then fails at the decoder, which is exactly the
    // regime this latch exists for. Finally the second GOP's real IDR heals it.
    let garbage: Vec<Vec<u8>> = (0..5u8)
        .map(|i| {
            let mut au = vec![0, 0, 0, 1, 0x41, 0x9A];
            au.extend_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF, i, 0x5A, 0xA5, 0x3C]);
            au
        })
        .collect();
    let mut picked: Vec<&[u8]> = vec![aus[0]];
    picked.extend(garbage.iter().map(|g| g.as_slice()));
    picked.push(aus[6]); // the IDR that heals it

    let (rec, storage) = memory();
    let walker = probe_walker();
    let mut state = SinkState::new();
    for (n, au) in picked.iter().enumerate() {
        let frame = build_probe_frame(n as u64, GREY_SIZE.height, au, 1_000 + n as u64 * 100);
        dispatch_frame(&rec, &walker, "/probe/flood/h264", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");
    let images = logged_images(&chunks(&storage));

    // The counter is the log-level-independent oracle.
    assert_eq!(
        state
            .video_decoders()
            .decode_failures("/probe/flood/h264", GREY_SIZE),
        5,
        "every refused unit must be counted"
    );
    assert_eq!(
        images.len(),
        2 - PIPELINE_DEPTH,
        "only the IDRs rendered, less the one still in the pipeline"
    );

    // Exactly one loud head, four suppressed repeats, one recovery — matched on
    // the LEVEL TOKEN too, so a reporter that emits every line at WARN fails.
    logs_assert(|lines: &[&str]| {
        let loud = lines
            .iter()
            .filter(|l| l.contains("WARN") && l.contains("could not be decoded"))
            .count();
        let quiet = lines
            .iter()
            .filter(|l| l.contains("DEBUG") && l.contains("undecodable (repeat)"))
            .count();
        let healed = lines
            .iter()
            .filter(|l| l.contains("INFO") && l.contains("decoding recovered"))
            .count();
        if loud != 1 {
            return Err(format!("expected exactly 1 loud WARN head, got {loud}"));
        }
        if quiet != 4 {
            return Err(format!("expected 4 DEBUG suppressed repeats, got {quiet}"));
        }
        if healed != 1 {
            return Err(format!("expected exactly 1 INFO recovery, got {healed}"));
        }
        Ok(())
    });
}

/// ANTI-TAUTOLOGY control for the flood test: a healthy stream logs no failure
/// line and counts zero, so the "exactly N" arms above are not satisfied by a
/// reporter that fires on every frame.
#[test]
#[tracing_test::traced_test]
fn a_healthy_stream_reports_no_decode_failure_at_all() {
    let aus = split_access_units(GREY_RAMP);
    let (images, _, state) = run_ramp("/probe/healthy/h264", &aus);
    assert_eq!(images.len(), 12 - PIPELINE_DEPTH);
    assert_eq!(
        state
            .video_decoders()
            .decode_failures("/probe/healthy/h264", GREY_SIZE),
        0
    );
    logs_assert(|lines: &[&str]| {
        let any = lines
            .iter()
            .filter(|l| l.contains("could not be decoded") || l.contains("undecodable"))
            .count();
        if any != 0 {
            return Err(format!("a healthy stream logged {any} failure lines"));
        }
        Ok(())
    });
}

// ────────────────────────────────────────────────────────────────────────────
// Fetch posture: Cisco's binary, and what happens before it is cached
// ────────────────────────────────────────────────────────────────────────────

/// THE FALLBACK PIN: with no decoder on this desk, video must still RENDER — by
/// handing the access unit to the viewer exactly as the first H.264 route did.
///
/// This is the shipping first-run state (Cisco's binary is fetched on first need
/// by the openh264 fetcher, so until it lands there is no local decoder), and it is the one
/// path this crate's own test build cannot reach on its own: the tests enable
/// `decoder-from-source` precisely so they decode hermetically. Hence the seam.
///
/// A black pane would be the worst outcome — worse than the 567 ms of lag this
/// issue is about — so the assertion is that the viewer-decoded path is fully
/// intact: a codec declaration (rerun REQUIRES it to decode H.264 at all) plus a
/// sample per access unit, and NOT an Image.
#[test]
fn with_no_decoder_the_viewer_still_gets_a_decodable_video_stream() {
    let aus = split_access_units(GREY_RAMP);
    let (rec, storage) = memory();
    let walker = probe_walker();
    let mut state = SinkState::new();
    state.set_video_decoders(
        cerulion_viz::video_decode::VideoDecoders::unavailable_for_test(
            "test: no Cisco blob cached",
        ),
    );

    for (i, au) in aus.iter().enumerate() {
        let frame = build_probe_frame(i as u64, GREY_SIZE.height, au, 1_000 + i as u64 * 100);
        dispatch_frame(&rec, &walker, "/probe/nodecoder/h264", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");
    let c = chunks(&storage);
    let components = logged_components(&c);
    let entity = "world/probe/nodecoder/h264/viz-video/320x240";

    // The codec declaration — without it the viewer has bytes it cannot decode,
    // which is the black pane this fallback exists to avoid.
    assert!(
        components
            .iter()
            .any(|(e, comp)| e == entity && comp.contains("VideoStream:codec")),
        "the fallback must declare the codec, got {components:?}"
    );
    // One sample per access unit, all at the rendition child. Counted in ROWS,
    // not chunks: rerun batches many rows into one chunk, so a per-chunk tally
    // would read 3 for a 12-frame stream.
    let samples: usize = c
        .iter()
        .filter(|chunk| chunk.entity_path().to_string().trim_start_matches('/') == entity)
        .flat_map(|chunk| chunk.components().iter())
        .filter(|(d, _)| d.as_str().contains("VideoStream:sample"))
        .map(|(_, list)| list.list_array.len())
        .sum();
    assert_eq!(samples, aus.len(), "one sample per access unit");
    // And NO decoded image — this desk did not decode anything.
    assert!(
        logged_images(&c).is_empty(),
        "a desk with no decoder cannot have produced a picture"
    );
}

/// ANTI-TAUTOLOGY for the arm above: with a decoder present (this build), the
/// SAME stream produces Images and NO `VideoStream` at all.
///
/// Without this the fallback test could be satisfied by a build that had silently
/// stopped decoding altogether.
#[test]
fn the_fallback_is_reached_only_when_the_decoder_is_missing() {
    let aus = split_access_units(GREY_RAMP);
    let (images, components, state) = run_ramp("/probe/haveDecoder/h264", &aus);
    assert!(
        state.video_decoders().is_available(),
        "this test build compiles the decoder in; the fallback must NOT be reached"
    );
    assert_eq!(images.len(), aus.len() - PIPELINE_DEPTH);
    assert!(!components.iter().any(|(_, c)| c.contains("VideoStream")));
}

/// The LICENSING posture is a build fact, so it is pinned as one.
///
/// Cisco's AVC patent grant covers the binaries CISCO distributes. The shipped
/// build therefore must DLOPEN their release (`libloading`) and must NOT compile
/// the decoder in (`source`) — the crate's own DEFAULT feature is `source`, so a
/// dropped `default-features = false` silently makes us the distributor of an AVC
/// decoder. That is a one-word edit with a legal consequence and no runtime
/// symptom, which is exactly the kind of thing a test should hold.
#[test]
fn the_shipped_build_dlopens_ciscos_binary_and_never_compiles_it_in() {
    let manifest = include_str!("../Cargo.toml");
    let dep = manifest
        .lines()
        .find(|l| l.trim_start().starts_with("openh264 = "))
        .expect("an openh264 dependency line");
    assert!(
        dep.contains("default-features = false"),
        "openh264's DEFAULT feature is `source` (compiles the decoder in). The shipped build \
         must opt out: {dep}"
    );
    assert!(
        dep.contains("\"libloading\""),
        "the shipped build must dlopen Cisco's released binary: {dep}"
    );
    assert!(
        !dep.contains("\"source\""),
        "the shipped dependency must not enable `source`: {dep}"
    );
    // The hermetic-test feature exists, and is NOT on by default.
    assert!(
        manifest.contains("decoder-from-source = [\"openh264/source\"]"),
        "the hermetic test feature must exist"
    );
    let features = manifest
        .split("[features]")
        .nth(1)
        .expect("a [features] section");
    let default_line = features
        .lines()
        .find(|l| l.trim_start().starts_with("default = "));
    if let Some(line) = default_line {
        assert!(
            !line.contains("decoder-from-source"),
            "compiling the decoder in must never be a default feature: {line}"
        );
    }
}

// ────────────────────────────────────────────────────────────────────────────
// The chroma the grey fixture structurally cannot bind
// ────────────────────────────────────────────────────────────────────────────

/// The SOURCE colours of `fixtures/color_ramp_8f_320x240_bt601.h264` — the bytes
/// written into the input PPMs, in order.
///
/// Saturated on purpose. Every grey oracle in this file is blind to the entire
/// colour-conversion matrix: a neutral grey has `U == V == 128`, where the chroma
/// terms are multiplied by zero and BT.601, BT.709 and any other matrix agree by
/// construction. Nothing above this line would notice if the conversion were
/// replaced wholesale.
const EXPECTED_CENTRE_COLORS: [[u8; 3]; 8] = [
    [255, 0, 0],
    [0, 255, 0],
    [0, 0, 255],
    [255, 255, 0],
    [0, 255, 255],
    [255, 0, 255],
    [255, 128, 0],
    [128, 0, 255],
];

/// How far a decoded saturated pixel may sit from its source colour.
///
/// MEASURED on this fixture: the worst end-to-end error across all 8 frames is
/// **13** (lossy `crf 14` encode + 4:2:0 chroma subsampling + the YUV→RGB
/// rounding). The headroom to 20 is what a different x264 build might add.
///
/// It is tight enough to be a MATRIX oracle, and that bound is measured too
/// rather than asserted: taking the decoder's OWN centre YUV for these 8 frames
/// and converting it with limited-range BT.709 instead of BT.601 lands within 1
/// of the source for BT.601 and errs by **up to 39** for BT.709 — 24, 39, 24, 39
/// and 27 on five of the eight. So a conversion that silently changed matrix
/// fails here, while the correct round trip passes with room to spare.
const COLOR_TOLERANCE: i32 = 20;

const _: () = assert!(
    COLOR_TOLERANCE >= 13 && COLOR_TOLERANCE < 24,
    "the tolerance must clear the measured round trip (13) and still catch the \
     smallest DISCRIMINATING wrong-matrix error (24). Not the smallest measured one \
     — three of the eight colours err by only 15, 15 and 5 under BT.709 and are not \
     relied on; the five that discriminate err by 24, 39, 24, 39 and 27."
);

/// THE CHROMA PIN: saturated colours survive the decode, so the colour conversion
/// is bound by an oracle at all.
///
/// Deliberately encoded as BT.601 limited range (`smpte170m`), which is what
/// `openh264` 0.9.7 applies UNCONDITIONALLY — it never reads the SPS VUI. So this
/// fixture is the case the shipped conversion gets RIGHT, and the test says so
/// while `crate::video_decode::DecodedFrame`'s docs record the case it gets wrong
/// (BT.709 content, including the Go2's 720p rendition; a known limitation).
/// Encoding a BT.709 fixture here would pin the DEFECT as expected behaviour, so
/// it is not done; such a fixture would
/// fail today and pass once the matrix is read from the SPS VUI.
#[test]
fn saturated_colours_survive_the_decode_so_the_matrix_is_pinned() {
    let aus = split_access_units(COLOR_RAMP);
    assert_eq!(
        aus.len(),
        EXPECTED_CENTRE_COLORS.len(),
        "fixture must carry one access unit per source colour"
    );

    let (rec, storage) = memory();
    let walker = probe_walker();
    let mut state = SinkState::new();
    for (i, au) in aus.iter().enumerate() {
        let frame = build_probe_frame(i as u64, GREY_SIZE.height, au, 1_000 + i as u64 * 100);
        dispatch_frame(&rec, &walker, "/probe/color/h264", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");
    let images = logged_images(&chunks(&storage));

    assert_eq!(
        images.len(),
        EXPECTED_CENTRE_COLORS.len() - PIPELINE_DEPTH,
        "one image per access unit, less the pipeline's held picture"
    );
    for (i, img) in images.iter().enumerate() {
        let want = EXPECTED_CENTRE_COLORS[i];
        let idx = ((img.height as usize / 2) * img.width as usize + img.width as usize / 2) * 3;
        let got = [img.rgb[idx], img.rgb[idx + 1], img.rgb[idx + 2]];
        for c in 0..3 {
            assert!(
                (got[c] as i32 - want[c] as i32).abs() <= COLOR_TOLERANCE,
                "image {i} channel {c}: got {got:?}, oracle {want:?} (tolerance {COLOR_TOLERANCE})"
            );
        }
    }

    // ANTI-TAUTOLOGY: the fixture really is chromatic. Were it neutral, every
    // assertion above would hold under ANY matrix and pin nothing — which is
    // exactly the state the grey-only suite was in.
    let neutral = images.iter().filter(|img| img.is_grey()).count();
    assert_eq!(
        neutral, 0,
        "every frame of this fixture must be saturated; {neutral} decoded neutral, so the \
         chroma terms are multiplied by ~zero and the matrix is unpinned"
    );
}

// ────────────────────────────────────────────────────────────────────────────
// A rendition with no decoder is COUNTABLE, not indistinguishable
// ────────────────────────────────────────────────────────────────────────────

/// A parameter-set-only access unit is accepted and produces NO picture, and that
/// is not a failure.
///
/// The shape an encoder makes when it ships SPS/PPS in one message and the IDR in
/// the next (see `crate::video`'s residual note). It must not be counted as a
/// decode failure — doing so would open a flood regime on a healthy stream.
#[test]
fn a_parameter_set_only_unit_yields_no_picture_and_is_not_a_failure() {
    let aus = split_access_units(GREY_RAMP);
    let (rec, storage) = memory();
    let walker = probe_walker();
    let mut state = SinkState::new();

    // Open the stream, then feed a unit carrying ONLY the parameter sets carved
    // out of the fixture's own IDR access unit — real bytes, no coded slice.
    let idr = cerulion_viz::video::scan_annex_b(aus[0]).expect("fixture IDR scans");
    let mut param_only: Vec<u8> = Vec::new();
    for nal in idr.nals.iter().filter(|n| matches!(n.kind, 7 | 8)) {
        param_only.extend_from_slice(&[0, 0, 0, 1]);
        param_only.extend_from_slice(nal.bytes);
    }
    assert!(
        cerulion_viz::video::scan_annex_b(&param_only)
            .is_some_and(|s| s.sps().is_some() && !s.has_coded_slice()),
        "the crafted unit must be parameter-sets-only"
    );

    for (n, au) in [aus[0], param_only.as_slice(), aus[1]].iter().enumerate() {
        let frame = build_probe_frame(n as u64, GREY_SIZE.height, au, 1_000 + n as u64 * 100);
        dispatch_frame(&rec, &walker, "/probe/paramset/h264", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");
    let images = logged_images(&chunks(&storage));

    assert_eq!(
        state
            .video_decoders()
            .decode_failures("/probe/paramset/h264", GREY_SIZE),
        0,
        "a parameter-set-only unit is not a decode failure"
    );
    // The two picture-carrying units rendered, less the one the pipeline is still
    // holding; the parameter-set-only unit contributed none either way.
    assert_eq!(
        images.len(),
        2 - PIPELINE_DEPTH,
        "only the coded pictures render, less the pipeline's held picture"
    );
    assert_eq!(
        state
            .video_decoders()
            .frames_decoded("/probe/paramset/h264", GREY_SIZE),
        (2 - PIPELINE_DEPTH) as u64
    );
}

/// A rendition that has NO decoder must be distinguishable from a topic carrying
/// no video — through the PUBLIC counters, not through a log line at the start of
/// the run.
///
/// This is the no-decoder rendition shape. It is driven through the pool's own seam
/// rather than the sink, because the create-failure arm needs a pool whose backend
/// cannot build an API, which a build that compiled the decoder in never produces.
#[test]
fn a_rendition_with_no_decoder_reports_its_fallback_units() {
    let mut pool = cerulion_viz::video_decode::VideoDecoders::unavailable_for_test(
        "test: no Cisco blob cached",
    );
    let aus = split_access_units(GREY_RAMP);

    assert!(!pool.is_available(), "the pool under test has no decoder");
    for au in &aus {
        assert_eq!(
            pool.decode("/probe/nodec/h264", GREY_SIZE, au, 0),
            cerulion_viz::video_decode::DecodeOutcome::DecoderUnavailable,
            "every unit must route to the viewer, not to a per-unit failure"
        );
    }

    // Nothing decoded, nothing refused — and the condition is still VISIBLE,
    // which is the whole finding: `frames_decoded == 0 && decode_failures == 0`
    // is exactly what a topic with no video reports.
    assert_eq!(pool.frames_decoded("/probe/nodec/h264", GREY_SIZE), 0);
    assert_eq!(pool.decode_failures("/probe/nodec/h264", GREY_SIZE), 0);
    assert_eq!(
        pool.fallback_units("/probe/nodec/h264", GREY_SIZE),
        aus.len() as u64,
        "every unit handed to the viewer must be counted — `frames_decoded == 0 && \
         decode_failures == 0` is byte-for-byte what a topic with NO video reports, so \
         without this the two are indistinguishable"
    );

    // ANTI-TAUTOLOGY / the discriminator: a rendition NOBODY ever asked about
    // reports zero on all three, so a nonzero count is genuine evidence.
    assert_eq!(
        pool.fallback_units("/probe/never-seen/h264", GREY_SIZE),
        0,
        "an untouched rendition reports nothing"
    );
}

/// THE CARVE-OUT PIN (R2-1): a rendition whose decoder cannot be CREATED still
/// reaches the viewer, and it is a carve-out a test can kill.
///
/// This arm exists because the creation-failure fallback was executed by nothing.
/// `resolve_backend` never yields a blob backend in a build that compiled the
/// decoder in, so those two arms were unreachable and a `panic!()` planted in both
/// left the whole suite green — a carve-out with no arm that dies when it returns.
/// `with_backend_for_test` hands the pool a `Backend::CiscoBlob`, which `api_for`
/// answers with `Err` under `decoder-from-source`, so the arm runs for real.
///
/// Pins all four claims the fallback makes at once.
#[test]
#[tracing_test::traced_test]
fn a_rendition_whose_decoder_cannot_be_created_falls_back_to_the_viewer() {
    let aus = split_access_units(GREY_RAMP);
    let (rec, storage) = memory();
    let walker = probe_walker();
    let mut state = SinkState::new();
    state.set_video_decoders(
        cerulion_viz::video_decode::VideoDecoders::with_backend_for_test(
            cerulion_viz::video_decode::Backend::CiscoBlob("/tmp/r2-unreachable-blob.so".into()),
        ),
    );

    for (i, au) in aus.iter().enumerate() {
        let frame = build_probe_frame(i as u64, GREY_SIZE.height, au, 1_000 + i as u64 * 100);
        dispatch_frame(&rec, &walker, "/probe/createfail/h264", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");
    let c = chunks(&storage);
    let components = logged_components(&c);
    let entity = "world/probe/createfail/h264/viz-video/320x240";

    // CLAIM 2 — the outcome is DecoderUnavailable, not Failed. Observable as the
    // sink taking the viewer-decodes route at all.
    // CLAIM 3 — `first_sample` emitted the codec declaration, WITHOUT which rerun
    // cannot decode H.264 and the fallback is a black pane rather than a slow one.
    assert!(
        components
            .iter()
            .any(|(e, comp)| e == entity && comp.contains("VideoStream:codec")),
        "the fallback must declare the codec, got {components:?}"
    );
    let samples: usize = c
        .iter()
        .filter(|chunk| chunk.entity_path().to_string().trim_start_matches('/') == entity)
        .flat_map(|chunk| chunk.components().iter())
        .filter(|(d, _)| d.as_str().contains("VideoStream:sample"))
        .map(|(_, list)| list.list_array.len())
        .sum();
    assert_eq!(samples, aus.len(), "every unit must reach the viewer");
    assert!(
        logged_images(&c).is_empty(),
        "nothing was decoded here, so no picture may be logged"
    );

    // CLAIM 4 — every unit is COUNTED, so the condition is distinguishable from a
    // topic carrying no video at all.
    assert_eq!(
        state
            .video_decoders()
            .fallback_units("/probe/createfail/h264", GREY_SIZE),
        aus.len() as u64
    );
    assert_eq!(
        state
            .video_decoders()
            .frames_decoded("/probe/createfail/h264", GREY_SIZE),
        0
    );

    // CLAIM 1 — creation is attempted AT MOST ONCE. A retry loop would re-enter
    // `api_for` (and its ~1 MB read + SHA-256, on the render thread) for every
    // access unit; the `create_failed` record is what short-circuits it. The loud
    // creation error is the observable: exactly one, against 12 units.
    logs_assert(|lines: &[&str]| {
        let attempts = lines
            .iter()
            .filter(|l| l.contains("could not be loaded for this rendition"))
            .count();
        if attempts != 1 {
            return Err(format!(
                "decoder creation must be attempted once, not once per unit; saw {attempts} \
                 attempts across {} access units",
                aus.len()
            ));
        }
        Ok(())
    });
}

// ────────────────────────────────────────────────────────────────────────────
// The fetch is only worth anything if a RUNNING pool picks it up
// ────────────────────────────────────────────────────────────────────────────

/// The cache generation is PROCESS-GLOBAL (it is how a completed fetch reaches
/// every pool in the daemon), so the two arms below — one that bumps it, one that
/// asserts it did not move — cannot run concurrently. Every other test in this
/// file only ever READS it, so the lock is scoped to these two.
fn generation_lock() -> std::sync::MutexGuard<'static, ()> {
    static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    LOCK.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// **THE PAYOFF PIN** — a pool that resolved to NO decoder starts
/// decoding once a blob lands, WITHOUT a restart.
///
/// The backend is resolved once per pool on purpose (a per-frame filesystem probe
/// on a desk with no blob costs a syscall per access unit), and that is exactly
/// what would make the whole feature invisible: the fetch completes, the bytes are
/// on disk, and nothing ever looks again — so the desk keeps the 567 ms
/// viewer-decodes path until vizd is restarted. "Restart it" is precisely the
/// experience seen on a live desk, so shipping the fetcher without this would
/// have moved the problem rather than fixed it.
///
/// Driven through the REAL `cache_generation` counter (bumped by the same
/// production accessor a completed fetch bumps) and the REAL `decode` entry point.
/// The test build compiles the decoder in, so the re-resolve finds a usable
/// backend — standing in for the blob having arrived.
///
/// Deleting the `self.adopt_fetched_blob()` call from
/// `VideoDecoders::decode` fails this test — every unit after the "fetch" still
/// reports `DecoderUnavailable`, which is the shipped-without-it behaviour.
#[test]
#[tracing_test::traced_test]
fn a_pool_with_no_decoder_starts_decoding_once_the_blob_arrives() {
    use cerulion_viz::video_decode::{DecodeOutcome, VideoDecoders};
    let _guard = generation_lock();

    let aus = split_access_units(GREY_RAMP);
    assert!(aus.len() >= 4, "the fixture must have units to spare");
    let mut pool =
        VideoDecoders::unavailable_pending_fetch_for_test("test: no Cisco blob cached yet");

    // BEFORE: no decoder, so every unit goes to the viewer.
    assert!(!pool.is_available(), "the pool starts with no decoder");
    for au in aus.iter().take(2) {
        assert_eq!(
            pool.decode("/probe/fetch/h264", GREY_SIZE, au, 0),
            DecodeOutcome::DecoderUnavailable,
            "with no blob, units are handed to the viewer"
        );
    }
    assert_eq!(
        pool.fallback_units("/probe/fetch/h264", GREY_SIZE),
        2,
        "both units were counted against the fallback"
    );
    assert_eq!(pool.frames_decoded("/probe/fetch/h264", GREY_SIZE), 0);

    // THE FETCH LANDS — announced exactly the way a completed fetch announces it.
    cerulion_viz::openh264_fetch::bump_cache_generation_for_test();

    // AFTER: the next unit re-resolves and DECODES. Feeding from the start of the
    // stream because a fresh decoder needs the parameter sets and the IDR.
    let mut decoded = 0usize;
    for au in aus.iter() {
        if let DecodeOutcome::Frame(frame) = pool.decode("/probe/fetch/h264", GREY_SIZE, au, 0) {
            assert_eq!(
                (frame.width, frame.height),
                (GREY_SIZE.width, GREY_SIZE.height),
                "a real picture at the rendition's size"
            );
            assert_eq!(
                frame.rgb.len(),
                (GREY_SIZE.width as usize) * (GREY_SIZE.height as usize) * 3,
                "tightly packed RGB8"
            );
            decoded += 1;
        }
    }
    assert!(
        pool.is_available(),
        "the pool must have adopted the arrived blob"
    );
    // The ANTI-TAUTOLOGY half of the sibling test's `resolves == 0`: the counter
    // really does move, exactly once, when there IS something to adopt.
    assert_eq!(
        pool.resolves_for_test(),
        1,
        "one announced fetch causes exactly one re-resolve, however many access units follow it"
    );
    assert!(
        decoded > 0,
        "after the blob arrives the pool must actually decode; it produced {decoded} pictures"
    );
    assert_eq!(
        pool.frames_decoded("/probe/fetch/h264", GREY_SIZE) as usize,
        decoded,
        "the public counter agrees with what came out"
    );
    // The rendition's "no decoder" record was CLEARED — without that, a rendition
    // already seen before the fetch would stay on the fallback forever while only
    // renditions first seen afterwards decoded.
    assert_eq!(
        pool.fallback_units("/probe/fetch/h264", GREY_SIZE),
        0,
        "the stale no-decoder verdict must not outlive the blob that overturns it"
    );

    // THE RECOVERY LINE. `fallback_units == 0` above cannot discriminate the latch
    // from a bare `create_failed.clear()`: BOTH leave the map empty,
    // so every reader of that map answers identically and the only difference is
    // this log line. Without this assertion the recovery line is pinned by nothing.
    //
    // Matched on the LEVEL TOKEN as well as the message, per the flood-latch
    // convention — a repeat that is secretly loud (or a head secretly quiet) is
    // exactly what an unpinned latch consumer drifts into.
    //
    // Reverting the recovery loop to `create_failed.clear()`
    // fails here with zero recovery lines.
    logs_assert(|lines: &[&str]| {
        let recovery: Vec<&&str> = lines
            .iter()
            .filter(|l| l.contains("stops going to the viewer"))
            .collect();
        if recovery.len() != 1 {
            return Err(format!(
                "exactly one rendition recovered, so exactly one recovery line is due; got {}\n{}",
                recovery.len(),
                lines.join("\n")
            ));
        }
        let line = recovery[0];
        if !line.split_whitespace().take(4).any(|tok| tok == "INFO") {
            return Err(format!("the recovery line must be INFO: {line}"));
        }
        // It carries what the record held BEFORE it was dropped — which is the
        // whole point of reporting rather than clearing.
        for field in ["suppressed_count=", "fallback_units=2"] {
            if !line.contains(field) {
                return Err(format!("the recovery line must carry {field}: {line}"));
            }
        }
        Ok(())
    });
}

/// A pool built AFTER a fetch already landed must not re-resolve on its first
/// unit — the generation is a COUNTER compared against what the pool recorded,
/// not a flag that stays set once anything ever succeeded.
///
/// The observable is that a fresh pool decodes with no generation change at all,
/// and that decoding never moves the counter itself.
#[test]
fn a_pool_built_after_the_fetch_needs_no_generation_change() {
    use cerulion_viz::video_decode::{DecodeOutcome, VideoDecoders};
    let _guard = generation_lock();

    cerulion_viz::openh264_fetch::bump_cache_generation_for_test();
    let before = cerulion_viz::openh264_fetch::cache_generation();

    let aus = split_access_units(GREY_RAMP);
    let mut pool = VideoDecoders::new();
    let mut decoded = 0usize;
    for au in aus.iter() {
        if matches!(
            pool.decode("/probe/afterfetch/h264", GREY_SIZE, au, 0),
            DecodeOutcome::Frame(_)
        ) {
            decoded += 1;
        }
    }
    assert!(
        decoded > 0,
        "a pool built after the fetch decodes straight away"
    );
    assert_eq!(
        cerulion_viz::openh264_fetch::cache_generation(),
        before,
        "decoding must not itself move the cache generation"
    );
    // THE COUNTER-NOT-FLAG PIN. Under `decoder-from-source` a re-resolve returns
    // the identical backend, so no rendering observable can tell "re-resolved"
    // from "did not" — and without this the test passes a `VideoDecoders::new()`
    // that records `Some(0)` and therefore re-resolves on EVERY fresh pool, which
    // is exactly the bug the counter design exists to prevent.
    //
    // Recording `Some(0)` at construction fails here at 1.
    assert_eq!(
        pool.resolves_for_test(),
        0,
        "a pool built AFTER the fetch has nothing to adopt — it must not re-resolve its backend \
         even once"
    );
}
