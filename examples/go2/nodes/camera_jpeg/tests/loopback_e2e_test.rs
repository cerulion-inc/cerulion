// SPDX-License-Identifier: AGPL-3.0-only
//! Loopback e2e: REAL H.264 access units → the camera's `appsrc`
//! transcode pipeline → valid JPEG.
//!
//! NEEDS GSTREAMER. This test requires GStreamer + the H.264/JPEG elements at RUNTIME
//! and is compiled ONLY under the `gstreamer` feature (the whole rig is
//! `#[cfg(feature = "gstreamer")]`).
//!
//! **GStreamer decides whether this binary runs.** `examples/go2` is built and
//! tested by the `demos-go2` job in `.github/workflows/ci.yml`, which runs
//! `cargo build --workspace --tests` and then most of the workspace's suite.
//! That job does not install GStreamer, so THIS binary is the one target it
//! compiles (as the featureless shell below) but cannot run; the job names
//! that exclusion explicitly and runs `--lib` and `node_publish_test`
//! normally.
//!
//! Run it on any machine with GStreamer installed:
//!
//! ```bash
//! cargo test -p camera_jpeg --features gstreamer --test loopback_e2e_test -- --nocapture
//! ```
//!
//! # What it does
//!
//! A [`LoopbackH264Source`] (`videotestsrc ! x264enc !
//! video/x-h264,profile=constrained-baseline ! h264parse config-interval=-1 !
//! appsink` — the NVDEC-compatible shape, see `pipeline.rs`) GENERATES a
//! bounded stream of real Annex-B access units. Each one is handed to the
//! PRODUCTION [`TranscodeLoop`] exactly as the node's tick hands it a wire
//! frame, and the JPEGs that come back out are asserted to be real JPEGs
//! (SOI/EOI markers) at the expected dimensions (parsed from the SOF marker).
//!
//! There is no UDP socket between the two halves (`udpsink` → `udpsrc`): the
//! camera's source is an `appsrc`, so there is no port to probe, no bind race
//! to retry around, and — the point — the test drives the SAME entry point
//! production does, gate included.
//!
//! # Loud precondition (never a silent pass)
//!
//! `preflight_loopback()` runs `gst::init()` and verifies every needed element
//! up front (variant-scoped: the software chain's `avdec_h264`/`videoconvert`/
//! `jpegenc` are required only when the probe selects the software pipeline);
//! if GStreamer or any element is absent, the test PANICS with an attributable
//! message rather than silently passing.
//!
//! ABSENT the `gstreamer` feature the rig cannot compile at all — but "cannot
//! compile" must not read as "passed". `gstreamer` became non-default,
//! and a `cargo test --test loopback_e2e_test` without `--features gstreamer`
//! then produced `0 passed; 0 failed` — a GREEN line for a smoke gate that
//! never ran, which is exactly the silent pass this file's whole design
//! refuses. So the featureless build is NOT empty: it carries one test that
//! FAILS and names the correct invocation
//! ([`the_loopback_smoke_was_built_without_the_gstreamer_feature`]).

/// THE anti-silent-pass guard. Without `--features gstreamer` this
/// binary holds no rig — so it holds this instead, and FAILS.
///
/// `libtest` reports an empty binary as `test result: ok. 0 passed`, which a
/// runbook reader (and a CI log grep) reads as "the smoke gate passed". It did
/// not run. The panic below names the exact invocation, so the failure is
/// self-correcting rather than a puzzle.
///
/// Costs nothing when the feature IS on: this item is compiled out and the
/// real rig below is compiled in.
#[cfg(not(feature = "gstreamer"))]
#[test]
fn the_loopback_smoke_was_built_without_the_gstreamer_feature() {
    panic!(
        "this smoke was built WITHOUT --features gstreamer — you are running the \
         empty shell, and an empty test binary reports `0 passed` (a GREEN line \
         for a gate that never ran). `gstreamer` is a NON-default \
         feature so the rest of examples/go2 builds on a machine with no GStreamer, \
         so the smoke must ask for it explicitly.\n\
         Run: cargo test -p camera_jpeg --features gstreamer --test loopback_e2e_test -- --nocapture"
    );
}

#[cfg(feature = "gstreamer")]
mod rig {
    use std::time::Duration;

    use camera_jpeg::capture::{JpegTranscoder, TranscodeLoop, TranscoderFactory};
    use camera_jpeg::frame::{is_jpeg, jpeg_dimensions, JpegFrame};
    use camera_jpeg::h264::{contains_sps, DEFAULT_TARGET_HEIGHT};
    use camera_jpeg::pipeline::{
        preflight_loopback, AuPull, GstCamera, LoopbackH264Source, PipelineConfig,
    };

    /// Test-pattern dimensions (both multiples of 16, so no encoder macroblock
    /// padding shifts the decoded size). The HEIGHT deliberately equals the
    /// node's default target rendition, so the production gate admits the
    /// generated stream without any test-only configuration.
    const WIDTH: u16 = 1280;
    const HEIGHT: u16 = 720;
    /// The generator emits this many frames — comfortably more than the JPEGs
    /// the test needs, so the decoder's pipeline latency never races the end
    /// of the stream.
    const NUM_BUFFERS: i32 = 60;
    /// Nominal 30 fps spacing, used as the loop's NODE-clock reading. Only the
    /// rebuild backoff consults it, and nothing here fails, so any monotonic
    /// series works; a realistic one keeps the log timestamps sane.
    const FRAME_PERIOD_NS: u64 = 33_000_000;

    /// The production transcoder factory (the exact one `lib.rs` installs).
    fn production_factory() -> TranscoderFactory {
        Box::new(|| {
            let cfg = PipelineConfig::production();
            GstCamera::start(&cfg)
                .map(|cam| Box::new(cam) as Box<dyn JpegTranscoder>)
                .map_err(|e| e.to_string())
        })
    }

    /// Start the generator, LOUDLY (never a silent skip).
    fn start_generator() -> LoopbackH264Source {
        let cfg = PipelineConfig::production();
        LoopbackH264Source::start(&cfg, NUM_BUFFERS, WIDTH, HEIGHT).unwrap_or_else(|e| {
            panic!(
                "the loopback H.264 generator failed to start: {e} — it needs \
                 videotestsrc + x264enc + h264parse + appsink (see BRINGUP.md)"
            )
        })
    }

    /// Run the generator into `feed_height`, collecting JPEGs until `want` have
    /// arrived or the generator ends (its tail is then drained). Returns the
    /// JPEGs and how many access units the generator produced.
    fn run(loopa: &mut TranscodeLoop, want: usize, feed_height: u32) -> (Vec<JpegFrame>, usize) {
        let mut generator = start_generator();
        let mut jpegs: Vec<JpegFrame> = Vec::new();
        let mut aus = 0usize;
        let mut now_ns = 0u64;
        let mut saw_sps = false;

        // Step 1: generate + feed.
        while jpegs.len() < want && aus < NUM_BUFFERS as usize {
            match generator.pull_au(Duration::from_millis(500)) {
                AuPull::Au(au) => {
                    aus += 1;
                    saw_sps |= contains_sps(&au);
                    now_ns += FRAME_PERIOD_NS;
                    if let Some(frame) = loopa.feed(now_ns, feed_height, &au) {
                        jpegs.push(frame);
                    }
                }
                AuPull::NoAu => continue,
                AuPull::Eos => break,
                AuPull::Fatal(msg) => panic!(
                    "the loopback H.264 GENERATOR died (bus Error): {msg} — this is the \
                     test's own source, not the code under test"
                ),
            }
        }
        // Step 2: the decoder's TAIL — the last access units fed are still
        // inside the pipeline when the generator ends.
        for _ in 0..40 {
            if jpegs.len() >= want {
                break;
            }
            now_ns += FRAME_PERIOD_NS;
            match loopa.drain(now_ns) {
                Some(frame) => jpegs.push(frame),
                None => std::thread::sleep(Duration::from_millis(20)),
            }
        }

        assert!(
            aus > 0,
            "the generator produced NO access units — x264enc/h264parse are present \
             (preflight passed) but nothing came out of the appsink"
        );
        assert!(
            saw_sps,
            "no generated access unit carried an SPS — `h264parse config-interval=-1` \
             is supposed to repeat SPS/PPS with every IDR, and without one the \
             camera's keyframe gate can never open (this would make the test below \
             fail for the WRONG reason)"
        );
        (jpegs, aus)
    }

    #[test]
    fn loopback_h264_decodes_to_valid_jpeg_frames() {
        // LOUD precondition: GStreamer + the required elements must be present.
        let kind = match preflight_loopback() {
            Ok(k) => k,
            Err(e) => panic!(
                "LOOPBACK PRECONDITION FAILED: {e}. This e2e requires \
                 GStreamer 1.14+ and the H.264/JPEG elements (see BRINGUP.md); it \
                 does not run on GitHub CI."
            ),
        };
        eprintln!("loopback e2e: transcode pipeline = {}", kind.label());
        assert_eq!(
            HEIGHT as u32, DEFAULT_TARGET_HEIGHT,
            "the generated stream's height must be the node's DEFAULT target, so \
             this test exercises the production gate configuration"
        );

        let mut transcode = TranscodeLoop::new(production_factory(), HEIGHT as u32);
        let (jpegs, aus) = run(&mut transcode, 3, HEIGHT as u32);
        eprintln!(
            "loopback e2e: {} access units fed, {} JPEGs out (stale dropped {}, \
             map failures {}, builds {})",
            aus,
            jpegs.len(),
            transcode.stale_jpegs_dropped(),
            transcode.map_failure_count(),
            transcode.pipeline_builds()
        );

        assert!(
            jpegs.len() >= 3,
            "expected >= 3 valid JPEG frames from the loopback (got {} from {aus} \
             access units) — check this machine's GStreamer H.264/JPEG plugins (see \
             BRINGUP.md)",
            jpegs.len()
        );
        for (i, frame) in jpegs.iter().enumerate() {
            assert!(
                is_jpeg(&frame.data),
                "pulled buffer #{i} is not a JPEG (SOI/EOI markers missing, {} bytes)",
                frame.data.len()
            );
            assert_eq!(
                jpeg_dimensions(&frame.data),
                Some((WIDTH, HEIGHT)),
                "decoded JPEG #{i} must be {WIDTH}x{HEIGHT}"
            );
        }
        assert_eq!(
            transcode.pipeline_builds(),
            1,
            "the pipeline must have been built ONCE — a higher count means it died \
             mid-stream and recovered, which on a Jetson is the NVDEC-compatibility \
             class the constrained-baseline generator exists to prevent"
        );
        assert!(
            transcode.gate().sps_seen(),
            "the keyframe gate must have opened"
        );
        assert_eq!(
            transcode.gate().skipped_other_rendition(),
            0,
            "everything generated was the target rendition"
        );
    }

    #[test]
    fn a_non_target_rendition_is_never_decoded() {
        // THE two-rendition pin over the REAL pipeline: label the SAME generated
        // access units with the sibling height and nothing may be decoded. This
        // is the failure the node exists to prevent — feeding both renditions of
        // `/go2/camera/h264` into one decoder — proven against a real x264enc
        // stream rather than a hand-built fixture.
        if let Err(e) = preflight_loopback() {
            panic!("LOOPBACK PRECONDITION FAILED: {e} (see BRINGUP.md)");
        }
        const SIBLING_HEIGHT: u32 = 360;
        assert_ne!(SIBLING_HEIGHT, HEIGHT as u32);

        let mut transcode = TranscodeLoop::new(production_factory(), HEIGHT as u32);
        // `want` is unreachable on purpose: the run stops at the generator's end.
        let (jpegs, aus) = run(&mut transcode, usize::MAX, SIBLING_HEIGHT);

        assert!(aus > 0, "the generator must have produced access units");
        assert!(
            jpegs.is_empty(),
            "a rendition the node was not asked for must NEVER reach the decoder \
             (got {} JPEGs from {aus} sibling-labelled access units)",
            jpegs.len()
        );
        assert_eq!(
            transcode.gate().skipped_other_rendition() as usize,
            aus,
            "every generated access unit was counted as the sibling rendition"
        );
        assert_eq!(transcode.gate().admitted(), 0);
        assert!(
            !transcode.gate().sps_seen(),
            "a sibling-rendition keyframe must NOT open OUR gate"
        );
    }
}
