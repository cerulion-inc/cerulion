// SPDX-License-Identifier: AGPL-3.0-only
//! E2e: the CameraJpeg node's trigger→gate→decode→publish path over a
//! REAL graph runtime + iceoryx2, driven with REAL `unitree_go/Go2FrontVideoData`
//! frames and a SCRIPTED decoder (no camera, no GStreamer).
//!
//! # Seam
//!
//! The node is DATA TRIGGERED: it fires once per access unit that
//! lands on its `h264` input, and everything — the rendition filter, the
//! wait-for-keyframe gate, the decode, the publish — happens inside that one
//! tick. So this test publishes real `Go2FrontVideoData` frames onto an
//! absolute external topic (exactly as the `dds_bridge` raw route does on the
//! robot) and observes the `sensor_msgs/CompressedImage` the node publishes.
//!
//! The GStreamer half is replaced through the node's ONE injection point,
//! [`CameraJpeg::with_transcoder`]: a scripted `JpegTranscoder` that RECORDS
//! every access unit handed to it and emits one JPEG per push, derived from the
//! access unit's filler byte so the mapping input→output is hand-computable.
//! That recording is what makes this an admission-gate test and not just a
//! plumbing test: the assertion is not "some frames came out" but "EXACTLY
//! these bytes reached the decoder and no others". The real gst chain is
//! covered by `tests/loopback_e2e_test.rs`; the loop's failure policy is
//! oracle-tested in `src/capture.rs`; the gate itself in `src/h264.rs`.
//!
//! # What is pinned
//!
//! - **The two-rendition trap (the headline).** The topic interleaves 360 and
//!   720; only the configured rendition reaches the decoder, and the sibling's
//!   IDR does NOT open the keyframe gate.
//! - **The keyframe gate through the transport.** A mid-GOP slice arriving
//!   before any SPS is dropped before the decoder sees it.
//! - **A JPEG is published verbatim** (`format == "jpeg"`, `data ==` what the
//!   decoder produced) and ONLY on the fires that decoded something.
//! - **The stamp schedule.** `header.stamp` equals the EXACT hand-computed
//!   `VirtualClock` schedule (`t0 + k*STEP`): `GraphRuntime::step(d)` advances
//!   the clock BEFORE firing, and the stamp comes from the node clock, never
//!   from the wire `time_frame` or a wall clock. Stamps are asserted RELATIVE
//!   to the post-warm-up `t0` so the oracle is independent of how many warm-up
//!   beats connection establishment consumed. The stamp is parsed from the
//!   nested header's wire bytes: `stamp` is the Header schema's FIRST fixed
//!   field, so `sec` is LE i32 at header-payload [0..4] and `nanosec` LE u32 at
//!   [4..8]. A parsed stamp also proves the header child was WRITTEN (an
//!   unwritten staged child would have discarded the whole frame —
//!   NestedChildIncomplete).
//! - **Determinism** (Principle #7): two isolated runs are byte-identical AND
//!   equal the HAND oracle (two-run equality alone would pass a both-wrong bug).
//!
//! Isolated per-test SHM root (`init_for_test`), so parallel-safe (no
//! `#[serial]`).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{CerulionPublisher, CerulionSubscriber};
use indexmap::IndexMap;
use native_ros2_messages::sensor_msgs::CompressedImage;
use unitree_go::Go2FrontVideoData;

use camera_jpeg::capture::{JpegTranscoder, Pull, TranscoderFactory};
use camera_jpeg::frame::JpegFrame;
use camera_jpeg::{CameraJpeg, CameraJpegEntry};

/// One 20 ms step per beat. The stamp schedule (`t0 + k*STEP`) is
/// hand-computable because the step advances the shared `VirtualClock` before
/// the node fires.
const STEP: Duration = Duration::from_millis(20);
/// STEP in nanoseconds (the stamp-oracle unit).
const STEP_NS: u64 = 20_000_000;
/// The rendition this test decodes (the node's default).
const TARGET_HEIGHT: u32 = 720;
/// The sibling rendition on the same topic.
const SIBLING_HEIGHT: u32 = 360;

// ---------------------------------------------------------------------------
// The scripted decoder (the node's injection seam)
// ---------------------------------------------------------------------------

/// The JPEG length the spy emits by default (see [`DecoderSpy::jpeg_len`]).
const SPY_JPEG_LEN: usize = 16;

/// A decoder model: every pushed access unit is RECORDED and immediately
/// yields one JPEG whose bytes are `[filler; jpeg_len]`, where `filler` is the
/// access unit's last byte. Deterministic, hand-computable, and gst-free.
struct DecoderSpy {
    /// Every access unit that reached the decoder, in order.
    pushed: Vec<Vec<u8>>,
    /// Outputs waiting to be polled.
    ready: std::collections::VecDeque<JpegFrame>,
    /// How many bytes each produced JPEG carries. Configurable so a test can
    /// hand the node a frame that does NOT fit the output's `max_slice_len`.
    jpeg_len: usize,
}

impl Default for DecoderSpy {
    fn default() -> Self {
        Self {
            pushed: Vec::new(),
            ready: std::collections::VecDeque::new(),
            jpeg_len: SPY_JPEG_LEN,
        }
    }
}

struct SpyTranscoder {
    state: Arc<Mutex<DecoderSpy>>,
}

impl JpegTranscoder for SpyTranscoder {
    fn push_au(&mut self, au: &[u8]) -> Result<(), String> {
        let filler = *au
            .last()
            .expect("the gate never pushes an empty access unit");
        let mut s = self.state.lock().unwrap_or_else(|p| p.into_inner());
        let len = s.jpeg_len;
        s.pushed.push(au.to_vec());
        s.ready.push_back(JpegFrame {
            data: vec![filler; len],
            pts_ns: None,
        });
        Ok(())
    }

    fn poll(&mut self) -> Pull {
        let mut s = self.state.lock().unwrap_or_else(|p| p.into_inner());
        match s.ready.pop_front() {
            Some(frame) => Pull::Frame(frame),
            None => Pull::NoFrame,
        }
    }
}

/// The JPEG the spy produces for an access unit whose filler byte is `filler`.
/// HAND-WRITTEN oracle side — never derived by calling the spy.
fn expected_jpeg(filler: u8) -> Vec<u8> {
    vec![filler; SPY_JPEG_LEN]
}

// ---------------------------------------------------------------------------
// Access-unit fixtures (hand-built Annex-B, per the H.264 spec)
// ---------------------------------------------------------------------------

/// Build an Annex-B access unit from `(nal_type, payload_len)` pairs, filled
/// with `filler` so the produced JPEG identifies which access unit made it.
fn au(nals: &[(u8, usize)], filler: u8) -> Vec<u8> {
    let mut v = Vec::new();
    for (ty, len) in nals {
        v.extend_from_slice(&[0x00, 0x00, 0x00, 0x01]);
        // nal_ref_idc = 3, forbidden_zero_bit = 0.
        v.push(0x60 | (ty & 0x1F));
        v.resize(v.len() + *len, filler);
    }
    v
}

/// An IDR access unit as the Go2 sends it: SPS(7) + PPS(8) + IDR(5).
fn idr_au(filler: u8) -> Vec<u8> {
    au(&[(7, 12), (8, 4), (5, 200)], filler)
}

/// A non-IDR slice access unit — needs an already-open keyframe gate.
fn slice_au(filler: u8) -> Vec<u8> {
    au(&[(1, 160)], filler)
}

// ---------------------------------------------------------------------------
// The rig
// ---------------------------------------------------------------------------

struct Rig {
    rt: GraphRuntime,
    /// Publishes `Go2FrontVideoData` onto the node's absolute input topic (the
    /// dds_bridge raw route's stand-in).
    h264_pub: CerulionPublisher,
    clock: Arc<VirtualClock>,
    obs: CerulionSubscriber,
    spy: Arc<Mutex<DecoderSpy>>,
}

/// The graph the node runs in: the PRODUCTION wiring (an absolute `h264`
/// source with no in-graph producer — the bridge's raw route publishes it — and
/// one `CompressedImage` output).
fn graph_config(prefix: &str, h264_topic: &str, max_slice_len: usize) -> GraphConfig {
    GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("cam_e2e_{prefix}"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "camera".to_string(),
            node_type: "camera_jpeg".to_string(),
            inputs: vec![InputDef {
                name: "h264".to_string(),
                source: h264_topic.to_string(),
            }],
            outputs: vec![OutputDef {
                name: "jpeg".to_string(),
                schema: "sensor_msgs/CompressedImage".to_string(),
                max_slice_len: Some(max_slice_len),
                history_size: 0,
                topic: None,
            }],
        }],
    }
}

/// An isolated per-test transport (its own iceoryx2 SHM root).
fn isolated_manager(prefix: &str, clock: Arc<VirtualClock>) -> Arc<TransportManager> {
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("cam_e2e_{prefix}"),
            clock,
            subscriber_buffer_size: 16,
            network: None,
        },
        ix_config,
    )
    .expect("init isolated e2e transport")
}

/// A modest output cap — far bigger than any fixture JPEG below (production
/// graphs size this for a real 720p frame).
const DEFAULT_MAX_SLICE_LEN: usize = 512 * 1024;

fn build_rig(prefix: &str) -> Rig {
    build_rig_with(prefix, DEFAULT_MAX_SLICE_LEN, SPY_JPEG_LEN)
}

/// `build_rig`, with the output's `max_slice_len` and the spy's JPEG size
/// under the caller's control — the two knobs that decide whether a produced
/// frame FITS the wire buffer.
fn build_rig_with(prefix: &str, max_slice_len: usize, jpeg_len: usize) -> Rig {
    let clock = Arc::new(VirtualClock::new());
    let mgr = isolated_manager(prefix, clock.clone());

    let h264_topic = format!("/{prefix}/camera/h264");
    let spy: Arc<Mutex<DecoderSpy>> = Arc::new(Mutex::new(DecoderSpy {
        jpeg_len,
        ..Default::default()
    }));
    let spy_in = Arc::clone(&spy);
    // ONE transcoder for the whole run: the node's loop builds it on the first
    // fed access unit and never rebuilds (nothing here fails).
    let factory: TranscoderFactory = Box::new(move || {
        Ok(Box::new(SpyTranscoder {
            state: Arc::clone(&spy_in),
        }) as Box<dyn JpegTranscoder>)
    });

    let config = graph_config(prefix, &h264_topic, max_slice_len);
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "camera".to_string(),
        Box::new(CameraJpegEntry::with_state(CameraJpeg::with_transcoder(
            factory,
            TARGET_HEIGHT,
        ))),
    );

    // Graph builds FIRST (it creates the absolute external source's service),
    // then the raw publisher attaches — the teleop_mux ordering.
    let rt = GraphRuntime::build(config, factories, &mgr, clock.clone())
        .expect("the camera graph must build (absolute external h264 source, one output)");

    let h264_pub = mgr
        .create_publisher(&h264_topic, MaxSliceLen::const_new(64 * 1024), 0)
        .expect("external h264 publisher attaches");
    let obs = mgr
        .create_subscriber(&format!("/{prefix}/camera/jpeg"))
        .expect("observer subscriber on the camera output");

    Rig {
        rt,
        h264_pub,
        clock,
        obs,
        spy,
    }
}

/// Publish ONE `Go2FrontVideoData` sample — exactly the three fields the robot
/// sends.
fn publish_au(rig: &mut Rig, time_frame: u64, video_height: u32, au: &[u8]) {
    let mut proxy = rig
        .h264_pub
        .loan_proxy::<Go2FrontVideoData>()
        .expect("loan Go2FrontVideoData");
    proxy.time_frame = time_frame;
    proxy.video_height = video_height;
    proxy
        .set_video_data(au)
        .expect("write the access unit into SHM");
    // Drop publishes (direct-path proxy).
}

/// One observed published frame: the payload bytes, the `format` string, and
/// the header stamp in ns RELATIVE to the post-warm-up `t0`.
type Observed = (Vec<u8>, String, u64);

/// Read the (at most one) frame the last fire published.
fn observe(rig: &mut Rig, t0: u64) -> Option<Observed> {
    rig.obs
        .try_view::<CompressedImage, _>(|view| {
            let hb = view.header_bytes();
            assert!(
                hb.len() >= 8,
                "header bytes must carry the 8-byte fixed stamp (got {} bytes)",
                hb.len()
            );
            let sec = i32::from_le_bytes(hb[0..4].try_into().expect("sec is 4 bytes"));
            let nanosec = u32::from_le_bytes(hb[4..8].try_into().expect("nanosec is 4 bytes"));
            // Reconstruct in the SIGNED domain first: `sec` is i32 and a direct
            // `as u64` would two's-complement-wrap a negative value into a huge
            // stamp instead of failing the assert below.
            let stamp_ns_signed = (sec as i64) * 1_000_000_000 + nanosec as i64;
            let stamp_ns = u64::try_from(stamp_ns_signed)
                .expect("a recorded beat's stamp must be non-negative");
            let rel_ns = stamp_ns
                .checked_sub(t0)
                .expect("a recorded beat's stamp must be at/after the post-warm-up clock");
            (
                view.data().to_vec(),
                view.format().map(str::to_string).unwrap_or_default(),
                rel_ns,
            )
        })
        .expect("try_view on the camera output")
}

/// Publish one access unit, step once (the data-trigger node fires), observe.
fn beat(rig: &mut Rig, t0: u64, time_frame: u64, height: u32, au: &[u8]) -> Option<Observed> {
    publish_au(rig, time_frame, height, au);
    rig.rt.step(STEP);
    observe(rig, t0)
}

/// Bounded warm-up: establish the pub→sub connections (the first same-process
/// iceoryx2 delivery can lag a cycle) by feeding keyframes until one JPEG comes
/// back out. Warm-up frames use filler 0xAA and are discarded.
fn warmup(rig: &mut Rig) {
    let mut tries = 0;
    loop {
        if beat(rig, 0, 0, TARGET_HEIGHT, &idr_au(0xAA)).is_some() {
            break;
        }
        tries += 1;
        assert!(
            tries < 200,
            "the camera never produced a frame in 200 warm-up beats"
        );
    }
}

/// One scripted beat's record: label + the observed frame.
type Beat = (&'static str, Option<Observed>);

/// Run the scripted exercise on an isolated transport. Returns the per-beat
/// observed sequence AND the access units that actually reached the decoder.
fn run_script(prefix: &str) -> (Vec<Beat>, Vec<Vec<u8>>) {
    let mut rig = build_rig(prefix);
    warmup(&mut rig);

    // The stamp time base: the clock right after warm-up. Recorded beat k
    // (1-based) steps the clock to exactly t0 + k*STEP before the tick reads
    // now_ns, so its stamp is t0 + k*STEP.
    let t0 = rig.clock.now_ns();
    // Forget the warm-up's pushes so the decoder-side oracle below covers only
    // the scripted beats.
    rig.spy
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .pushed
        .clear();

    // The beats run IN ORDER (each publishes, steps and observes), so this
    // `vec![]` is a sequence of side-effecting calls, not a list of values —
    // Rust evaluates the elements left to right, which is what makes that safe.
    let seq: Vec<Beat> = vec![
        // 1) The SIBLING rendition (an IDR, no less) — dropped before the decoder.
        (
            "sibling_idr",
            beat(&mut rig, t0, 1, SIBLING_HEIGHT, &idr_au(0x11)),
        ),
        // 2) A target-rendition SLICE. The gate is open (warm-up fed a
        // keyframe), so it decodes.
        (
            "target_slice",
            beat(&mut rig, t0, 1, TARGET_HEIGHT, &slice_au(0x22)),
        ),
        // 3) A target-rendition KEYFRAME.
        (
            "target_idr",
            beat(&mut rig, t0, 2, TARGET_HEIGHT, &idr_au(0x33)),
        ),
        // 4) An UNKNOWN height — never seen from this robot, dropped loudly.
        ("unknown_height", beat(&mut rig, t0, 3, 1080, &idr_au(0x44))),
        // 5) An EMPTY payload — nothing to decode.
        ("empty", beat(&mut rig, t0, 4, TARGET_HEIGHT, &[])),
    ];

    let pushed = rig
        .spy
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .pushed
        .clone();
    rig.rt.shutdown();
    (seq, pushed)
}

/// HAND-BUILT oracle (never a re-run of the node): the exact expected per-beat
/// sequence — payloads, format, and the stamp schedule `t0 + k*STEP`.
fn hand_oracle() -> Vec<Beat> {
    vec![
        // The sibling rendition publishes NOTHING.
        ("sibling_idr", None),
        // Beat 2 decodes and publishes at t0 + 2*STEP.
        (
            "target_slice",
            Some((expected_jpeg(0x22), "jpeg".to_string(), 2 * STEP_NS)),
        ),
        // Beat 3 decodes and publishes at t0 + 3*STEP.
        (
            "target_idr",
            Some((expected_jpeg(0x33), "jpeg".to_string(), 3 * STEP_NS)),
        ),
        ("unknown_height", None),
        ("empty", None),
    ]
}

#[test]
fn camera_publish_script_matches_hand_oracle_and_is_deterministic() {
    let oracle = hand_oracle();
    // Only the two target-rendition access units ever reach the decoder.
    let decoder_oracle = vec![slice_au(0x22), idr_au(0x33)];

    let (run_a, pushed_a) = run_script("runa");
    assert_eq!(
        run_a, oracle,
        "run A must match the hand oracle (sibling/unknown/empty publish nothing, the \
         two target access units publish their decoded JPEG at the exact t0 + k*STEP stamp)"
    );
    assert_eq!(
        pushed_a, decoder_oracle,
        "THE rendition pin: exactly the target-rendition access units reached the \
         decoder, byte for byte — the 360 sibling, the 1080 unknown and the empty \
         payload never did"
    );

    // Determinism: an independent isolated run is byte-identical AND equals the
    // oracle (two-run equality alone would pass a both-wrong bug).
    let (run_b, pushed_b) = run_script("runb");
    assert_eq!(run_b, oracle, "run B must match the hand oracle");
    assert_eq!(pushed_b, decoder_oracle, "run B decoder input must match");
    assert_eq!(run_a, run_b, "the two isolated runs must be byte-identical");
}

#[test]
fn a_mid_gop_join_publishes_nothing_until_the_first_keyframe() {
    // The headline SPS-gate pin ACROSS the transport, with NO warm-up keyframe:
    // a graph that joins a running stream mid-GOP must publish nothing (and
    // decode nothing) until an IDR arrives — a decoder cannot start mid-GOP.
    let mut rig = build_rig("midgop");

    for k in 0..8u64 {
        let got = beat(&mut rig, 0, k, TARGET_HEIGHT, &slice_au(0x55));
        assert_eq!(
            got, None,
            "beat {k}: a pre-keyframe slice must publish nothing"
        );
    }
    assert!(
        rig.spy
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .pushed
            .is_empty(),
        "not one pre-keyframe byte reached the decoder"
    );

    // The keyframe opens the gate. Connection establishment may still eat the
    // first delivery, so give it a bounded number of keyframes and assert the
    // decoder saw them.
    let mut published = None;
    for k in 0..20u64 {
        if let Some(obs) = beat(&mut rig, 0, 100 + k, TARGET_HEIGHT, &idr_au(0x66)) {
            published = Some(obs);
            break;
        }
    }
    let (data, format, _) = published.expect("a keyframe must eventually publish a JPEG");
    assert_eq!(data, expected_jpeg(0x66), "the decoded keyframe's JPEG");
    assert_eq!(format, "jpeg");
    let pushed = rig
        .spy
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .pushed
        .clone();
    assert!(
        pushed.iter().all(|au| *au == idr_au(0x66)),
        "ONLY keyframes reached the decoder; the 8 earlier slices never did"
    );
    rig.rt.shutdown();
}

/// Set the spy's produced-JPEG size mid-run (the sizer arm needs a payload
/// SPIKE after the publisher's adaptive window has warmed on small frames).
fn set_spy_jpeg_len(rig: &Rig, len: usize) {
    rig.spy.lock().unwrap_or_else(|p| p.into_inner()).jpeg_len = len;
}

#[test]
fn a_payload_spike_past_the_adaptive_loan_still_publishes() {
    // This test is the reason the payload write is a
    // whole-field `set_data` rather than a `fill_from`.
    //
    // The publisher's loan is ADAPTIVE: once its sliding window is warm
    // (16 sends) it loans about 1.5x the recent maximum frame, NOT
    // `max_slice_len`. A gate that compares the JPEG against the space
    // remaining in THAT loan refuses any frame past ~1.5x recent — an
    // ordinary scene change on a content-driven encoder. And the refusal is
    // SELF-SUSTAINING: the sizer records only sizes that were actually SENT,
    // so a dropped frame never widens the window, the recent maximum never
    // rises, and the camera stalls FOREVER while advising the operator to
    // raise a ceiling that was never the constraint.
    //
    // So: warm the window on small frames, then spike far past 1.5x recent
    // while staying comfortably INSIDE `max_slice_len`. That frame MUST
    // publish (via the spill), and the stream must keep running.
    const CEILING: usize = 512 * 1024;
    const SMALL: usize = 512;
    // ~128x the warm recent max — no adaptive loan can accommodate this
    // without spilling, and it is still an eighth of the ceiling.
    const SPIKE: usize = 64 * 1024;
    // The publisher's window warms at 16 sends; overshoot it comfortably.
    const WARM_SENDS: usize = 40;

    let mut rig = build_rig_with("sizer", CEILING, SMALL);
    warmup(&mut rig);

    // Warm the ADAPTIVE SIZER on small frames.
    let mut small_published = 0usize;
    for k in 0..WARM_SENDS as u64 {
        if beat(&mut rig, 0, k, TARGET_HEIGHT, &idr_au(0x11)).is_some() {
            small_published += 1;
        }
    }
    assert!(
        small_published >= 16,
        "the adaptive window needs >= 16 SENDS to warm; only {small_published} published"
    );

    // THE SPIKE. Bounded retry only for iceoryx2 delivery latency — the
    // assertion is that it publishes AT ALL (a loan-space gate would refuse
    // it on every attempt, forever).
    set_spy_jpeg_len(&rig, SPIKE);
    let mut spike = None;
    for k in 0..20u64 {
        if let Some(obs) = beat(&mut rig, 0, 1_000 + k, TARGET_HEIGHT, &idr_au(0x22)) {
            spike = Some(obs);
            break;
        }
    }
    let (data, format, _) = spike.expect(
        "a payload SPIKE that fits max_slice_len MUST publish — a gate on the adaptive loan \
         refuses it and, because a dropped frame never enters the sizer window, the camera \
         stalls indefinitely",
    );
    assert_eq!(
        data.len(),
        SPIKE,
        "the spike must arrive WHOLE — not truncated, not clipped to the previous loan"
    );
    assert_eq!(
        data,
        vec![0x22u8; SPIKE],
        "and byte-for-byte what was decoded"
    );
    assert_eq!(format, "jpeg");

    // ...and the stream KEEPS RUNNING at the new size (the self-sustaining
    // stall would show up here as the next spike being refused again).
    let mut more = 0usize;
    for k in 0..10u64 {
        if beat(&mut rig, 0, 2_000 + k, TARGET_HEIGHT, &idr_au(0x33)).is_some() {
            more += 1;
        }
    }
    assert!(
        more > 0,
        "after a spike the camera must keep publishing at the new size, not wedge"
    );
    rig.rt.shutdown();
}

#[test]
fn an_oversized_jpeg_is_dropped_never_truncated_onto_the_wire() {
    // The CONTRACT for a GENUINELY over-ceiling frame: it never reaches the
    // wire. This is the ONLY refusal — a frame merely larger than the
    // current adaptive loan spills and publishes (see
    // `a_payload_spike_past_the_adaptive_loan_still_publishes`), which is the
    // whole point of the spill.
    //
    // The two arms share EVERYTHING except the produced frame's size, which is
    // what makes the "nothing published" arm non-vacuous — an absence caused
    // by connection lag or a mis-sized ceiling would show up in the control
    // too.
    //
    // SCOPE. A truncate-and-publish path looks safe on the reasoning that
    // truncation consumes all remaining buffer and the staged `header` flush
    // (which happens at `OutputProxy::Drop`, after `data`) would then fail
    // and discard the frame. That holds only on a COLD publisher, where the
    // loan IS the ceiling — which is exactly the configuration this test
    // uses, and why the claim would look unconditional here. On a WARM
    // publisher the loan is ~1.5x the recent maximum, so a frame between
    // that and `max_slice_len` truncated to the loan leaves room for the
    // header flush and PUBLISHES — corrupt bytes on the wire, in the
    // production configuration. The spill path cannot reach that state at
    // all: any frame within the ceiling spills and publishes whole.
    //
    // The latch/counter half of this class is pinned where it is decided, by
    // `h264::tests::the_flood_latch_*`.
    const CEILING: usize = 4096;
    const FITS: usize = 64;
    const OVERSIZED: usize = 8 * 1024;

    // CONTROL — the same ceiling, a frame that fits: it publishes, verbatim.
    let mut fit_rig = build_rig_with("fit", CEILING, FITS);
    let mut beats_to_first_publish = 0usize;
    let mut published = None;
    for k in 0..200u64 {
        beats_to_first_publish = k as usize + 1;
        if let Some(obs) = beat(&mut fit_rig, 0, k, TARGET_HEIGHT, &idr_au(0x77)) {
            published = Some(obs);
            break;
        }
    }
    let (data, format, _) =
        published.expect("a JPEG that FITS the ceiling must publish (the control arm)");
    assert_eq!(
        data,
        vec![0x77u8; FITS],
        "the control publishes the decoder's bytes verbatim"
    );
    assert_eq!(format, "jpeg");
    fit_rig.rt.shutdown();

    // THE PIN — the same ceiling, a frame that does not fit: nothing is ever
    // published, across many more beats than the control needed.
    let mut big_rig = build_rig_with("big", CEILING, OVERSIZED);
    let beats = (beats_to_first_publish * 4).max(40);
    for k in 0..beats as u64 {
        assert_eq!(
            beat(&mut big_rig, 0, k, TARGET_HEIGHT, &idr_au(0x88)),
            None,
            "beat {k}: an oversized JPEG must be DROPPED — a truncated JPEG is a \
             corrupt JPEG, and nothing may reach the wire"
        );
    }
    // ...and the access units really did reach the decoder, so the absence
    // above is the WRITE dropping an oversized frame, not the admission gate
    // skipping the input (which would make the whole test vacuous).
    let pushed = big_rig
        .spy
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .pushed
        .clone();
    assert_eq!(
        pushed.len(),
        beats,
        "every access unit was admitted and decoded; only the PUBLISH was dropped"
    );
    big_rig.rt.shutdown();
}

// ---------------------------------------------------------------------------
// The no-capture refusal
//
// Every test above drives the node through `CameraJpeg::with_transcoder`, and
// that seam SHORT-CIRCUITS `init()` before the capture check — deliberately (a
// test that supplies its own decoder is by definition not a capture-less
// build), but it also means none of them exercise the refusal.
// These two do, through the REAL `GraphRuntime::build` path, using
// `CameraJpegEntry::new()` — the default entry, which is what `graph run`
// constructs.
//
// They are a feature-gated PAIR, because the refusal is a property of the
// build, not of the input:
//   * WITHOUT `gstreamer` (the default, decoder-less build) the graph build
//     must FAIL, naming the node and the remedy;
//   * WITH it (the capture build a companion runs) the very same graph must BUILD — the
//     anti-tautology control proving the refusal is conditional on having no
//     decoder rather than on being a default-constructed node.
// ---------------------------------------------------------------------------

/// Build the camera graph with the DEFAULT (non-injected) node entry.
fn build_default_entry_graph(prefix: &str) -> Result<GraphRuntime, cerulion_core::TransportError> {
    let clock = Arc::new(VirtualClock::new());
    let mgr = isolated_manager(prefix, clock.clone());
    let h264_topic = format!("/{prefix}/camera/h264");
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("camera".to_string(), Box::new(CameraJpegEntry::new()));
    GraphRuntime::build(
        graph_config(prefix, &h264_topic, DEFAULT_MAX_SLICE_LEN),
        factories,
        &mgr,
        clock,
    )
}

#[cfg(not(feature = "gstreamer"))]
#[test]
fn a_build_without_capture_fails_the_graph_build_naming_the_node() {
    // THE capture-less-build pin. An `external` node gets this guarantee from
    // its shape (`ExternalNodesInertAtLaunch` at launch); a
    // data-triggered node has no launch-time external-source check, so if
    // `init()` did not refuse, this graph would build happily, fire on every
    // access unit, and publish nothing FOREVER — the exact silent-inert
    // failure this pin rules out.
    //
    // Replacing the `if !GSTREAMER_ENABLED { return Err(..) }` body with
    // `Ok(())` makes this test fail (the build succeeds).
    // (`GraphRuntime` is not `Debug`, so `expect_err` is unavailable — match.)
    let err = match build_default_entry_graph("nogst") {
        Ok(_) => panic!("a capture-less build MUST fail the graph build, not run inert"),
        Err(e) => e,
    };
    // SCOPE of the node-name assertion, measured rather than assumed.
    // The framework's structured attribution for a failing user `init` is the
    // literal `node_id: "user_init"` — the macro's boundary wrap, NOT the node
    // type (only the double-init / not-initialized paths carry the snake_case
    // label). So NOTHING outside this node's own message text tells the operator which
    // node refused, which makes the `contains("camera_jpeg")` assertion below
    // LOAD-BEARING rather than tautological: it is the only attribution there
    // is, and deleting the node name from the message would leave a multi-node
    // robot graph failing with an anonymous `user_init` error.
    match &err {
        cerulion_core::TransportError::NodeError { node_id, .. } => {
            assert_eq!(
                node_id, "user_init",
                "drift guard: if the framework starts attributing user-init failures to the \
                 NODE, say so here and the message-text assert below can be relaxed"
            );
        }
        other => panic!("expected a NodeError from the failing init, got: {other}"),
    }
    let msg = err.to_string();
    // ...so the message itself must carry the node name...
    assert!(
        msg.contains("camera_jpeg"),
        "the refusal message is the ONLY node attribution — it must name the node; got: {msg}"
    );
    // ...say WHY...
    assert!(
        msg.contains("gstreamer"),
        "the refusal must name the missing feature; got: {msg}"
    );
    // ...and state the FIX, in the form an operator can paste.
    assert!(
        msg.contains("cerulion node build camera_jpeg"),
        "the refusal must carry the remediation command; got: {msg}"
    );
}

#[cfg(feature = "gstreamer")]
#[test]
fn a_build_with_capture_is_not_refused() {
    // The anti-tautology half: with a decoder compiled in, the SAME graph and
    // the SAME default entry build cleanly. (Nothing is stepped, so no
    // GStreamer pipeline is ever constructed — the factory is lazy, built on
    // the first fed access unit.)
    let rt = build_default_entry_graph("gst")
        .expect("with the gstreamer feature the camera graph must build");
    rt.shutdown();
}
