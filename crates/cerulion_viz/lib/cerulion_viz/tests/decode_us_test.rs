// SPDX-License-Identifier: AGPL-3.0-only
//! A `s3_vizd_render` line's `decode_us` reports THIS
//! frame's decode time or `0` — never a previous frame's, and never another
//! TOPIC's.
//!
//! # The defect this file exists to prevent
//!
//! The decode timer writes `SinkState::probe_decode_us` on EVERY video access unit
//! while the probe is on, but only SAMPLED frames read it. At any stride > 1 — the
//! measurement runbook's own recommendation — `stride - 1` of every `stride`
//! decodes therefore leave a value parked, and ONE `SinkState` is threaded across
//! ALL inputs (`worker.rs::process_batch`). So a sampled `/lowstate` line reported
//! the video's decode time on a topic that has no decoder, and the derived leg
//! `render_us - decode_us` went NEGATIVE. A wrong number is worse than an absent
//! one: it is exactly the class this repo treats as a defect.
//!
//! The fix clears at the WRITER's door (top of `dispatch_or_stage`) instead of the
//! reader's, so a value can only ever belong to the call that emitted the line.
//!
//! # Why the sentinel, and why it is not fake data
//!
//! Arm A/C park a distinctive value through a `#[cfg(test-helpers)]` seam. That is
//! a hand ORACLE, not fabricated measurement: a REAL decode can legitimately take
//! 0 µs on a small unit, which is the same value the bug's absence produces, so an
//! end-to-end-only pin would be silently vacuous whenever the decoder was quick.
//! Arm B then drives the identical property through a REAL unsampled decode (no
//! seam), and asserts its own precondition — that a non-zero value really was
//! parked — so the two arms together cover both the deterministic and the
//! realistic shape.
//!
//! Every oracle is the EMITTED LINE (what an operator reads), with the state behind
//! it asserted alongside.

use std::sync::{Mutex, Once};

use cerulion_core::codegen::layout::{LayoutResolver, WireLayout};
use cerulion_core::codegen::{parse_rosmsg, FrameWalker, MessageSchema};
use cerulion_core::wire::WireHeader;
use cerulion_viz::sink::{dispatch_or_stage, SinkState};
use cerulion_viz::video::StreamKey;
use tracing_test::traced_test;

/// The sampling stride every test in this binary runs under. > 1 is the whole
/// point: at stride 1 every frame clears on read and the defect cannot appear.
const STRIDE: u32 = 4;

/// A wire sequence the stride SAMPLES (a multiple of `STRIDE`).
const SAMPLED_SEQ: u32 = 8;
/// A wire sequence the stride SKIPS — the frame whose decode used to leak.
const UNSAMPLED_SEQ: u32 = 9;

/// A decode time no real decode in this file can produce, so a line carrying it is
/// unambiguously reporting a PREVIOUS frame's value.
const SENTINEL_US: u64 = 424_242;

/// The two input names this file drives (also the `input=` field the line carries).
const CAMERA: &str = "/camera";
const TELEMETRY: &str = "/telemetry";

/// Serializes the tests: they share the process-global `SinkState`-free but
/// `OnceLock`-backed probe stride and a global tracing subscriber.
static ENV_LOCK: Mutex<()> = Mutex::new(());

/// Arm the probe BEFORE anything in this process reads it. `lat_probe`'s stride is
/// resolved once per process, so every test must set the SAME value and must set it
/// before the first `probe_enabled()` — a `Once` at the top of each test does both
/// regardless of which test libtest runs first.
fn arm_probe() {
    static ARM: Once = Once::new();
    ARM.call_once(|| {
        // SAFETY: runs exactly once, before any test body has spawned a thread that
        // reads the environment (the probe's own read happens after this).
        std::env::set_var(cerulion_core::lat_probe::LAT_PROBE_ENV, STRIDE.to_string());
    });
    assert!(
        cerulion_core::lat_probe::probe_enabled(),
        "the probe must be ON for this file to test anything"
    );
    assert!(
        cerulion_core::lat_probe::should_sample(SAMPLED_SEQ),
        "seq {SAMPLED_SEQ} must be sampled at stride {STRIDE}"
    );
    assert!(
        !cerulion_core::lat_probe::should_sample(UNSAMPLED_SEQ),
        "seq {UNSAMPLED_SEQ} must NOT be sampled at stride {STRIDE} — the whole \
         defect lives in the frames the stride skips"
    );
}

// ────────────────────────────────────────────────────────────────────────────
// Fixtures (crib: video_decode_test.rs — the same real x264 grey ramp)
// ────────────────────────────────────────────────────────────────────────────

const GREY_RAMP: &[u8] = include_bytes!("fixtures/grey_ramp_12f_320x240_high.h264");
const GREY_HEIGHT: u32 = 240;
/// The ramp's own picture size — the rendition key its decoder is counted under.
const GREY_KEY: StreamKey = StreamKey {
    width: 320,
    height: 240,
};

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

/// Split the ramp into access units on its Access Unit Delimiters.
fn split_access_units(buf: &[u8]) -> Vec<&[u8]> {
    let mut starts = Vec::new();
    let mut i = 0usize;
    while i + 4 <= buf.len() {
        let sc = if buf[i..].starts_with(&[0, 0, 0, 1]) {
            4
        } else if buf[i..].starts_with(&[0, 0, 1]) {
            3
        } else {
            0
        };
        if sc > 0 {
            if let Some(&hdr) = buf.get(i + sc) {
                if hdr & 0x1F == 9 {
                    starts.push(i);
                }
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

/// Build a real `probe/VideoProbe` wire frame carrying `payload` in
/// `video_data`, stamped with an explicit wire `sequence` (which is what the
/// probe's sampling decision keys on).
fn build_frame(sequence: u32, video_height: u32, payload: &[u8]) -> Vec<u8> {
    let layout = probe_layout();
    let fixed = layout.fixed_size;
    let table = layout.offset_table_bytes();

    let mut body = vec![0u8; fixed + table];
    let tf_off = field_offset(&layout, "time_frame");
    let vh_off = field_offset(&layout, "video_height");
    body[tf_off..tf_off + 8].copy_from_slice(&(sequence as u64).to_le_bytes());
    body[vh_off..vh_off + 4].copy_from_slice(&video_height.to_le_bytes());
    let data_off = (fixed + table) as u32;
    body[fixed..fixed + 4].copy_from_slice(&data_off.to_le_bytes());
    body[fixed + 4..fixed + 8].copy_from_slice(&(payload.len() as u32).to_le_bytes());
    body.extend_from_slice(payload);

    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: probe_schema().schema_hash(),
        total_size: (WireHeader::SIZE + body.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed) as u32,
        offset_table_count: 1,
        sequence,
        timestamp_ns: 1_000 + sequence as u64,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&body);
    frame
}

/// A payload that is definitively NOT H.264 (no Annex-B start code), so the frame
/// classifies as something else and never reaches the decoder.
fn non_video_payload() -> Vec<u8> {
    b"not-annex-b-at-all".to_vec()
}

struct Harness {
    rec: rerun::RecordingStream,
    walker: FrameWalker,
    state: SinkState,
    _storage: rerun::sink::MemorySinkStorage,
}

impl Harness {
    fn new(tag: &str) -> Self {
        let (rec, storage) = rerun::RecordingStreamBuilder::new("decode")
            .recording_id(tag.to_string())
            .memory()
            .expect("memory sink");
        Harness {
            rec,
            walker: probe_walker(),
            state: SinkState::new(),
            _storage: storage,
        }
    }

    /// Drive ONE frame through the production dispatch path.
    fn dispatch(&mut self, input: &str, frame: Vec<u8>) {
        let mut staged: Option<Vec<u8>> = None;
        let mut coalesced = 0u64;
        dispatch_or_stage(
            &self.rec,
            &self.walker,
            input,
            frame,
            &mut self.state,
            &mut staged,
            &mut coalesced,
        );
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Line reading — whole-token field matching (the lesson: a substring
// match on `decode_us=4` is satisfied by `decode_us=424242`).
// ────────────────────────────────────────────────────────────────────────────

/// The `decode_us` value on the ONE `s3_vizd_render` line naming `input`.
fn decode_us_for(lines: &[&str], input: &str) -> u64 {
    let needle = format!("input=\"{input}\"");
    let matching: Vec<&&str> = lines
        .iter()
        .filter(|l| l.contains("s3_vizd_render") && l.contains(&needle))
        .collect();
    assert_eq!(
        matching.len(),
        1,
        "expected exactly ONE s3 line for input {input:?}, got {}:\n{}",
        matching.len(),
        lines.join("\n")
    );
    field_u64(matching[0], "decode_us")
}

/// Read `key=<u64>` as a whole whitespace token off a rendered tracing line.
fn field_u64(line: &str, key: &str) -> u64 {
    let prefix = format!("{key}=");
    line.split_whitespace()
        .find_map(|tok| tok.strip_prefix(&prefix))
        .unwrap_or_else(|| panic!("no `{key}=` token on line:\n{line}"))
        .parse()
        .unwrap_or_else(|e| panic!("`{key}` is not a u64 on line:\n{line}\n{e}"))
}

/// Whether an `s3_vizd_render` line naming `input` was emitted at all.
fn has_s3_line(lines: &[&str], input: &str) -> bool {
    let needle = format!("input=\"{input}\"");
    lines
        .iter()
        .any(|l| l.contains("s3_vizd_render") && l.contains(&needle))
}

// ────────────────────────────────────────────────────────────────────────────
// The pins
// ────────────────────────────────────────────────────────────────────────────

#[traced_test]
#[test]
fn a_sampled_non_video_line_never_carries_a_parked_decode_time() {
    // ARM A — the deterministic regression detector. A previous frame's decode is
    // parked (hand oracle); the next SAMPLED frame is a NON-video topic, which has
    // no decoder at all, so its line must report 0.
    let _g = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    arm_probe();
    let mut h = Harness::new("armA");

    h.state.set_probe_decode_us_for_test(SENTINEL_US);
    assert_eq!(
        h.state.probe_decode_us_for_test(),
        SENTINEL_US,
        "precondition: the sentinel really is parked"
    );

    h.dispatch(TELEMETRY, build_frame(SAMPLED_SEQ, 0, &non_video_payload()));

    logs_assert(|lines: &[&str]| {
        let got = decode_us_for(lines, TELEMETRY);
        if got != 0 {
            return Err(format!(
                "a NON-video topic's line reported decode_us={got} — it has no \
                 decoder, so this is another frame's (or another topic's) time"
            ));
        }
        Ok(())
    });
    assert_eq!(
        h.state.probe_decode_us_for_test(),
        0,
        "the parked value must be gone, not merely unreported"
    );
}

#[traced_test]
#[test]
fn a_real_unsampled_decode_does_not_leak_into_the_next_sampled_line() {
    // ARM B — the same property with NO seam: a real, UNSAMPLED video decode parks
    // a real value (asserted as this arm's own precondition), then the next SAMPLED
    // frame is a different, non-video input. This is the exact shape the runbook's
    // `export CERULION_H264_LAT_PROBE=30` produces on a Studio desk.
    let _g = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    arm_probe();
    let mut h = Harness::new("armB");

    let aus = split_access_units(GREY_RAMP);
    assert!(aus.len() >= 3, "fixture must carry a GOP");

    // TWO UNSAMPLED units: each decodes, writes the timer, emits NO line.
    //
    // The keyframe ALONE no longer yields a picture. The decoder runs
    // with openh264's flush-after-decode OFF (its default leaked a picture
    // reference and reset the decoder mid-GOP — see `video_decode::StreamDecoder`),
    // so openh264 holds each picture for one call and the FIRST unit after a
    // decoder opens returns `NoPicture`. The `frames_decoded > 0` precondition
    // below is the property this arm actually needs, so the fix is to prime the
    // pipeline rather than to weaken it.
    for au in aus.iter().take(2) {
        h.dispatch(CAMERA, build_frame(UNSAMPLED_SEQ, GREY_HEIGHT, au));
    }
    logs_assert(|lines: &[&str]| {
        if has_s3_line(lines, CAMERA) {
            return Err("an UNSAMPLED frame must emit no s3 line".to_string());
        }
        Ok(())
    });

    let parked = h.state.probe_decode_us_for_test();
    assert!(
        h.state.video_decoders().frames_decoded(CAMERA, GREY_KEY) > 0,
        "precondition: the unsampled frame must really have decoded"
    );
    assert!(
        parked > 0,
        "precondition: a real decode must park a NON-ZERO value ({parked}), else \
         this arm cannot discriminate — arm A is the deterministic pin"
    );

    // The next SAMPLED frame is a different topic with no decoder.
    h.dispatch(TELEMETRY, build_frame(SAMPLED_SEQ, 0, &non_video_payload()));

    logs_assert(|lines: &[&str]| {
        let got = decode_us_for(lines, TELEMETRY);
        if got != 0 {
            return Err(format!(
                "the unsampled video decode leaked into a NON-video topic's line \
                 as decode_us={got}"
            ));
        }
        Ok(())
    });
}

#[traced_test]
#[test]
fn a_sampled_video_frame_that_never_reaches_the_decoder_reports_zero() {
    // ARM C — the second occurrence class: the VIDEO topic's own line. A sampled
    // access unit taking a `VideoRoute::Drop` arm (a non-keyframe before any
    // keyframe — the normal startup transient) never reaches the decoder, so it
    // must report 0 rather than the previous unit's time.
    let _g = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    arm_probe();
    let mut h = Harness::new("armC");

    let aus = split_access_units(GREY_RAMP);
    h.state.set_probe_decode_us_for_test(SENTINEL_US);

    // aus[1] is a non-keyframe and NOTHING has opened the stream yet.
    h.dispatch(CAMERA, build_frame(SAMPLED_SEQ, GREY_HEIGHT, aus[1]));

    let demux = h.state.video();
    let dropped = demux.dropped_before_keyframe(CAMERA) + demux.dropped_unattributable(CAMERA);
    assert!(
        dropped > 0,
        "precondition: the unit must really have taken a Drop route \
         (before_keyframe={}, unattributable={})",
        demux.dropped_before_keyframe(CAMERA),
        demux.dropped_unattributable(CAMERA)
    );
    assert_eq!(
        h.state.video_decoders().frames_decoded(CAMERA, GREY_KEY),
        0,
        "precondition: a dropped unit must produce no picture"
    );

    logs_assert(|lines: &[&str]| {
        let got = decode_us_for(lines, CAMERA);
        if got != 0 {
            return Err(format!(
                "a video frame that never reached the decoder reported \
                 decode_us={got} — the previous unit's time"
            ));
        }
        Ok(())
    });
}

#[traced_test]
#[test]
fn a_sampled_frame_that_does_decode_still_reports_its_own_time() {
    // ANTI-TAUTOLOGY. Without this, "always report 0" passes arms A-C. A sampled
    // keyframe DOES decode, so its line must carry a real measurement — and in
    // particular NOT the parked sentinel, which proves the clear did not merely
    // blank the field for everyone.
    let _g = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    arm_probe();
    let mut h = Harness::new("armD");

    let aus = split_access_units(GREY_RAMP);

    // PRIME the pipeline on an UNSAMPLED sequence first. openh264 holds
    // a picture for one call (flush-after-decode is off — see
    // `video_decode::StreamDecoder`), so the keyframe alone produces none. The
    // priming unit is unsampled, so it emits no line and cannot itself satisfy
    // the assertions below; the sentinel is parked AFTER it, so the "did the
    // sentinel survive?" discrimination is untouched.
    h.dispatch(CAMERA, build_frame(UNSAMPLED_SEQ, GREY_HEIGHT, aus[0]));
    h.state.set_probe_decode_us_for_test(SENTINEL_US);

    h.dispatch(CAMERA, build_frame(SAMPLED_SEQ, GREY_HEIGHT, aus[1]));

    assert!(
        h.state.video_decoders().frames_decoded(CAMERA, GREY_KEY) > 0,
        "precondition: a decodable unit must yield a picture"
    );
    logs_assert(|lines: &[&str]| {
        let got = decode_us_for(lines, CAMERA);
        if got == SENTINEL_US {
            return Err(
                "the line reported the PARKED sentinel — the decode timer's own \
                 write is not reaching the line"
                    .to_string(),
            );
        }
        Ok(())
    });
    // And the field now holds THIS frame's decode, not the sentinel.
    assert_ne!(
        h.state.probe_decode_us_for_test(),
        SENTINEL_US,
        "the sentinel must have been cleared and overwritten by this frame's decode"
    );
}
