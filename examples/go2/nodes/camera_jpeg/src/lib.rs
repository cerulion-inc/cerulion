// SPDX-License-Identifier: AGPL-3.0-only
//! Go2 front-camera H.264 → JPEG transcode node.
//!
//! The Go2's front camera is a 720p H.264 stream that reaches Cerulion as the
//! DDS topic `/frontvideostream`, republished by the `dds_bridge` as
//! `/go2/camera/h264` (`unitree_go/Go2FrontVideoData`). This node DECODES it on
//! the companion computer and re-encodes to JPEG, publishing `sensor_msgs/CompressedImage`
//! (`format = "jpeg"`) on `/go2/camera/jpeg` for the desk-side viewer.
//!
//! # Shape: a DATA-TRIGGERED transcoder
//!
//! The node does NOT own its source. An `#[cerulion_node(external)]` ingress
//! node could own one, a `udpsrc` joined to the vendor's 230.1.1.1:1720 multicast
//! group, pumped by a helper thread that rang the node's doorbell per JPEG. Two
//! things rule that shape out:
//!
//! 1. **The multicast route is dead on the robot.** `ip route get 230.1.1.1`
//!    resolves to `wlan0` (home WiFi), not `eth0` (the robot LAN), so the path
//!    needs a per-robot route/interface fix before one frame can arrive.
//! 2. **The same video is already a topic**, and "the camera is a topic" is
//!    what generalizes to an unfamiliar robot — no socket, no group, no
//!    interface, and it rides the transport that is already recorded and replayed.
//!
//! So the node is `#[input(trigger)]`-driven: one fire per access unit that
//! lands on `/go2/camera/h264`. Each fire feeds that access unit to a GStreamer
//! `appsrc` and publishes whatever JPEG the decoder has ready — see
//! [`capture::TranscodeLoop`] for the (gst-free, oracle-tested) admission /
//! rebuild / drain policy and the `pipeline` module (feature-gated, so not
//! linkable from here) for the gst seam itself. There is no helper thread, no
//! cross-thread queue and no doorbell.
//!
//! # The two-rendition trap
//!
//! `/go2/camera/h264` carries TWO INDEPENDENT H.264 renditions interleaved on
//! ONE topic — 640x360 and 1280x720, one of each per `time_frame`. They are
//! separate encodes with separate parameter sets, so feeding both to one
//! decoder is not "two streams at once", it is one corrupt stream. The node
//! decodes exactly ONE, selected by `video_height` (default
//! [`h264::DEFAULT_TARGET_HEIGHT`], overridable per-run with the
//! [`TARGET_HEIGHT_ENV`] environment variable, read from the node's FROZEN env
//! snapshot so it stays replay-deterministic). The filter, and the
//! wait-for-a-keyframe gate that pairs with it, are pure functions in
//! [`h264`].
//!
//! # Frame loss is bounded and counted, never silent
//!
//! Everything the NODE discards is COUNTED, flood-latch-logged at the boundary
//! where it happens, and summarised at teardown: the sibling rendition,
//! pre-keyframe access units, unknown frame heights, empty payloads, access
//! units that ARRIVED while the pipeline was down (both renditions — see
//! [`capture::TranscodeLoop::aus_dropped_while_down`]), access units a dying
//! pipeline REFUSED (the gap between `aus_admitted` and `aus_pushed`), JPEGs
//! superseded by a newer one in the same tick, and JPEGs that exceed the
//! output's `max_slice_len` CEILING (dropped, never truncated — a truncated
//! JPEG is a corrupt JPEG). A frame merely larger than the publisher's current
//! adaptive loan is NOT a loss: it spills and publishes whole (see
//! `write_jpeg_frame`).
//!
//! Two loss channels are NOT the node's to count, and both are named here
//! rather than left to be discovered:
//!
//! 1. **Transport eviction on the input.** The `h264` input is a normal
//!    `drop_oldest` Cerulion input, so if the node falls behind the OLDEST
//!    access units are evicted by the RUNTIME before the node ever sees them.
//!    The runtime counts them —
//!    `NodeHandle::backpressure_drop_oldest_count("h264")`, an off-thread
//!    operator surface — but the node cannot, so they do not appear in the
//!    teardown summary. See the overload signature below for why this matters
//!    more than a dropped frame usually would.
//! 2. **`appsink max-buffers=2 drop=true`.** The appsink evicts its oldest
//!    JPEG inside GStreamer and the application is never told. This is the ONE
//!    accepted uncounted slot in the node, on two grounds: it is BOUNDED AT
//!    TWO, and what it drops is by construction stale — the `TranscodeLoop`
//!    drain applies the identical latest-wins rule one layer out and DOES
//!    count there (`stale_jpegs_dropped`).
//!
//! # The overload signature (read this before blaming the decoder)
//!
//! Sustained overload on this node does not degrade gracefully into "a few
//! dropped frames". It has a specific, self-reinforcing shape, and knowing it
//! is what lets an operator attribute a stuttering picture correctly:
//!
//! 1. The node falls behind (a slow decode, a stalled tick, a burst).
//! 2. The input's `drop_oldest` queue overflows and the runtime EVICTS the
//!    oldest access units — see channel 1 above; the node is handed a stream
//!    with a HOLE in it and has no way to know.
//! 3. A mid-GOP gap is not something a decoder can absorb. It errors.
//! 4. The bus error surfaces as `capture::Pull::Fatal`; the pipeline is torn
//!    down and rebuilt after [`capture::REBUILD_BACKOFF_NS`] (1 s).
//! 5. A rebuilt decoder has NO parameter sets, so the keyframe gate re-closes
//!    ([`h264::AuGate::reset_for_new_pipeline`]) and nothing is decoded until
//!    the next IDR.
//!
//! So one overflow costs roughly `1 s + (time to the next keyframe)` of video,
//! not one frame — and if the overload persists, the cycle repeats. The
//! teardown summary makes it unmistakable: `pipeline_builds` > 1 together with
//! a climbing `skipped_waiting_for_keyframe`, alongside repeated
//! `camera transcode pipeline FATAL` errors and `WAITING FOR A KEYFRAME` warns
//! in the log. Cross-check the runtime's `drop_oldest` counter on the `h264`
//! input to confirm the eviction is the trigger rather than a genuinely
//! corrupt stream.
//!
//! **Why the input depth is left at the default.** A deeper `#[input(depth =
//! N)]` would let the queue absorb a longer stall — sized to a GOP it would
//! turn "gap" into "delay". It is deliberately NOT set, for two reasons worth
//! recording: (a) the GOP length of the LIVE stream is unmeasured (the
//! loopback generator pins `key-int-max=30`, the robot's encoder is its own),
//! so any depth would be a guess at the number that matters; and (b) an
//! explicit depth makes the service-provisioning order on `/go2/camera/h264`
//! load-bearing — the graph and the bridge's raw-route publisher both
//! land on the default ceiling so either create order is safe, whereas a
//! raised requirement fails the graph build outright if the bridge created the
//! service first. Measure the live keyframe interval and the drop_oldest
//! counter under load first; that measurement is what should choose the
//! number.
//!
//! # Timestamping (Principle #7)
//!
//! `header.stamp` is derived from the NODE clock (`self.now_ns()` in the tick)
//! via `builtin_interfaces::Time::from_ns`, NOT from the gst buffer
//! PTS, NOT from the wire `time_frame`, and NOT from any wall clock — so the
//! published stamp is a pure function of the scheduler's clock. The rebuild
//! backoff in [`capture::TranscodeLoop`] runs on that same node clock for the
//! same reason (and because `#[cerulion_node_impl]` DENIES `Instant::now` in a
//! tick body outright). The gst PTS rides the internal
//! [`frame::JpegFrame`] as diagnostic-only metadata; it lives in the pipeline
//! clock's epoch and is NOT subtractable from `now_ns` (see BRINGUP.md for the
//! latency-measurement plan).
//!
//! ## Scope: the SCHEDULE is deterministic, the PAYLOAD is not
//!
//! This node is **not replay-grade on its output payload**, and that is a
//! property of what it does, not a defect to be fixed. Re-execution
//! re-executes the current cdylibs and BYTE-DIFFS the frames they produce
//! against the recording; the bytes this node produces are whatever a
//! GStreamer decode + JPEG encode emitted at that moment, from a pipeline that
//! is not in the recording, on hardware whose encoder (`nvjpegenc` vs
//! `jpegenc`, and either one's version) is not pinned by anything Cerulion
//! records. A replay of a bag containing `/go2/camera/jpeg` is therefore
//! expected to report a byte mismatch on that topic — as a true statement
//! about the world, not a regression.
//!
//! What IS deterministic, and what a replay legitimately checks:
//!
//! - the fire schedule (one fire per access unit on the `h264` input);
//! - `header.stamp`, a pure function of the scheduler clock;
//! - `header.frame_id` and `format`, constants;
//! - every admission decision in [`h264`] and every lifecycle decision in
//!   [`capture`] — both pure, both oracle-tested for two-run byte identity.
//!
//! So the useful verification on the robot is a rate + accounting check
//! (`cerulion topic hz` on both topics, plus the teardown summary), not a
//! byte-exact replay of the JPEG stream. BRINGUP.md carries the commands.
//!
//! # Transport and codec copy boundaries
//!
//! The H.264 input borrows bytes directly from shared memory. This example has
//! three application-level payload copies around the asynchronous codec:
//! H.264 SHM → owned `gst::Buffer`, JPEG appsink buffer → owned `JpegFrame`,
//! and `JpegFrame` → the output SHM loan. Decode/encode and any driver-internal
//! copies are additional codec work. An adaptive-loan spill can add another
//! output copy when a frame grows beyond the recent size window.
//!
//! The input copy lets GStreamer retain an access unit after this tick ends.
//! The owned JPEG releases the GStreamer pool buffer before publication and
//! keeps the transcoder test seam independent of GStreamer. The output uses
//! a size-aware setter so scene changes can grow the loan without truncation.
//! Local Cerulion subscribers then borrow the published frame without copying.

// NOTE: no crate-level `#![forbid(unsafe_code)]` — the `#[cerulion_node]` macro
// expands cdylib FFI entry points containing `unsafe`. The PURE `frame`,
// `h264` and `capture` modules keep the forbid at module scope.

// P12 (the project logging rule): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod capture;
pub mod frame;
pub mod h264;
#[cfg(feature = "gstreamer")]
pub mod pipeline;
pub mod pipeline_desc;

use cerulion_core::prelude::*;
use native_ros2_messages::builtin_interfaces::Time;
use native_ros2_messages::sensor_msgs::CompressedImage;
use unitree_go::Go2FrontVideoData;

use crate::capture::{TranscodeLoop, TranscoderFactory};
use crate::h264::{
    resolve_target_height, FloodCounter, FloodLog, TargetHeightNote, DEFAULT_TARGET_HEIGHT,
};

/// The `format` field value for every published frame — this node always emits
/// JPEG.
const JPEG_FORMAT: &str = "jpeg";

/// The `header.frame_id` for every published frame: the Go2 front camera's TF
/// frame. (The frame_id is a fixed constant here — make
/// it a field if per-instance configurability is ever needed.)
const CAMERA_FRAME_ID: &str = "front_camera";

/// Environment variable selecting WHICH of the topic's two interleaved
/// renditions to decode, by frame height. Read ONCE at `init` from the node's
/// frozen env snapshot (`NodeContext::env`), so it is replay-deterministic and
/// cannot change under a running graph.
///
/// Graph YAML carries no per-node config block (node config is a node
/// concern), so an env var read through the snapshot is the house mechanism for
/// this, the same one `dds_bridge` uses for `DDS_BRIDGE_CONFIG`.
pub const TARGET_HEIGHT_ENV: &str = "CAMERA_TARGET_HEIGHT";

/// Whether this build can actually decode. `false` means the `gstreamer`
/// feature was off at compile time, which `init` refuses LOUDLY rather than
/// running a node that can never publish a frame.
const GSTREAMER_ENABLED: bool = cfg!(feature = "gstreamer");

/// Build the production transcoder factory: a fresh GStreamer decode→JPEG
/// pipeline per call (the loop calls it once at start and once per rebuild).
#[cfg(feature = "gstreamer")]
fn production_factory() -> TranscoderFactory {
    Box::new(|| {
        let cfg = pipeline::PipelineConfig::production();
        pipeline::GstCamera::start(&cfg)
            .map(|cam| Box::new(cam) as Box<dyn capture::JpegTranscoder>)
            .map_err(|e| e.to_string())
    })
}

/// Without the `gstreamer` feature there is no pipeline to build. This factory
/// is never reached in practice — [`CameraJpeg::init`] refuses the launch
/// first — but it must exist for the type to be constructible, and if it ever
/// IS reached it fails loudly rather than pretending to have a decoder.
#[cfg(not(feature = "gstreamer"))]
fn production_factory() -> TranscoderFactory {
    Box::new(|| {
        Err(
            "camera_jpeg was built WITHOUT the `gstreamer` feature — there is no \
             decoder to build (rebuild with `cerulion node build camera_jpeg`, which \
             probes for GStreamer, or `cargo build -p camera_jpeg --features gstreamer`)"
                .to_string(),
        )
    })
}

/// The Go2 front-camera H.264→JPEG transcode node. See the module docs.
///
/// Data-triggered: fires once per access unit delivered on the `h264` input.
// Field notes (plain comments — the port fields carry only the macro attrs):
// - h264: the Go2FrontVideoData trigger input (one H.264 Annex-B access unit
//   per frame; `video_height` distinguishes the two interleaved renditions).
// - jpeg: the CompressedImage output (format="jpeg"; data = the encoded frame;
//   header.stamp = node clock at publish; header.frame_id = the camera frame).
// - transcode: the gst-free loop owning the pipeline lifecycle + admission gate.
// - published: frames published (observability, summarised at teardown).
// - oversized: JPEGs too large for the output's max_slice_len — dropped (never
//   truncated), counted, warn flood-latched on the SAME tested `FloodCounter`
//   the admission gate's skip classes use.
// - injected_transcoder: test seam marker — see `with_transcoder`.
#[cerulion_node]
#[derive(Default)]
pub struct CameraJpeg {
    #[input(trigger)]
    h264: Go2FrontVideoData,
    #[output]
    jpeg: CompressedImage,
    transcode: TranscodeLoop,
    published: u64,
    oversized: FloodCounter,
    injected_transcoder: bool,
}

impl CameraJpeg {
    /// Construct the node over a caller-provided transcoder factory — the test
    /// seam `tests/node_publish_test.rs` uses to drive the whole
    /// trigger→decode→publish path with a SCRIPTED decoder (no GStreamer, no
    /// camera), and the only way `init` is allowed to run in a build without
    /// the `gstreamer` feature.
    pub fn with_transcoder(factory: TranscoderFactory, target_height: u32) -> Self {
        Self {
            transcode: TranscodeLoop::new(factory, target_height),
            injected_transcoder: true,
            ..Default::default()
        }
    }
}

#[cerulion_node_impl]
impl CameraJpeg {
    /// Resolve the target rendition from the frozen env snapshot and refuse the
    /// launch outright if this build cannot decode.
    ///
    /// The refusal is the replacement for the old
    /// `ExternalSource::HostDriven` → `ExternalNodesInertAtLaunch` path: a
    /// data-triggered node has no launch-time external-source check, so
    /// WITHOUT this it would load happily, fire on every frame, and publish
    /// nothing forever. `init` returning `Err` fails the graph BUILD, naming
    /// the node — loud, at launch, and before any frame is lost.
    fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
        if self.injected_transcoder {
            tracing::info!(
                target_height = self.transcode.gate().target_height(),
                "camera_jpeg: driving an INJECTED transcoder (test seam) — no GStreamer \
                 pipeline will be built"
            );
            return Ok(());
        }
        if !GSTREAMER_ENABLED {
            return Err(NodeError::Fatal(
                "camera_jpeg was built WITHOUT the `gstreamer` feature, so it has no \
                 decoder and could never publish a frame. Build it with capture — \
                 `cerulion node build camera_jpeg --release` (it PROBES for GStreamer \
                 and says exactly what is missing), or `cargo build --release \
                 -p camera_jpeg --features gstreamer` — and note that a later plain \
                 `cargo build --release` OVERWRITES that cdylib with a capture-less one \
                 (see the camera crate's Cargo.toml)."
                    .to_string(),
            ));
        }

        // The DECISION is a pure function (oracle-tested in `h264`); this
        // `match` is the whole of the logging half. Splitting them is what
        // makes the policy testable at all — this line is reachable only on a
        // build that HAS a decoder, so a machine without GStreamer can never
        // execute it.
        let configured: u32 = ctx.env(TARGET_HEIGHT_ENV, DEFAULT_TARGET_HEIGHT);
        let (target_height, note) = resolve_target_height(configured);
        match note {
            TargetHeightNote::Known => {}
            TargetHeightNote::ZeroFallback => tracing::warn!(
                env_var = TARGET_HEIGHT_ENV,
                default_height = DEFAULT_TARGET_HEIGHT,
                "camera_jpeg: a target height of 0 matches no rendition — falling back to \
                 the default"
            ),
            // Loud inference, never silent: the node will happily decode this
            // height if the robot really publishes it, but no known rendition
            // has that height, so say so once at launch instead of leaving the
            // operator with a black screen and a climbing skip counter.
            TargetHeightNote::UnknownHeight => tracing::warn!(
                target_height,
                known = ?h264::KNOWN_RENDITION_HEIGHTS,
                env_var = TARGET_HEIGHT_ENV,
                "camera_jpeg: the configured target height is not one the Go2 is known to \
                 publish — every access unit will be skipped unless the firmware really \
                 changed"
            ),
        }
        self.transcode = TranscodeLoop::new(production_factory(), target_height);
        tracing::info!(
            target_height,
            input_topic_hint = "/go2/camera/h264",
            "camera_jpeg: decoding the {target_height}-line rendition; the sibling \
             rendition on the same topic is skipped"
        );
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        let now = self.now_ns();

        // Port-field reads are hoisted into locals: the `#[cerulion_node_impl]`
        // rewriter does not descend into macro token streams, so a
        // `self.h264.*` inside `tracing::trace!` is a compile error by design.
        // `time_frame` is the CAMERA's own monotonic frame stamp — the two
        // renditions of one capture share a value, which is how the pairing was
        // established.
        let time_frame = self.h264.time_frame;
        let video_height = self.h264.video_height;
        let au = self.h264.video_data();
        let au_len = au.len();
        tracing::trace!(
            time_frame,
            video_height,
            au_len,
            node_ns = now,
            "camera access unit received"
        );

        // Feed the access unit (the loop applies the rendition filter + the
        // wait-for-keyframe gate + the pipeline lifecycle) and take back the
        // NEWEST JPEG the decoder has ready. The read borrows straight out of
        // SHM; the codec ownership boundaries are described in the module docs.
        let Some(frame) = self.transcode.feed(now, video_height, au) else {
            // Nothing to publish this fire: the access unit was skipped, or the
            // decoder has not produced output for it yet (one frame of decode
            // latency is normal, and the very first frames wait for a
            // keyframe). Touch NO port field and return Ok — under the
            // lazy loan an output that is never written is never LOANED, so
            // there is nothing to publish and nothing to discard. (This is a
            // silent, costless non-event, NOT the discard path: that one is
            // reached only when a frame is loaned and then left incomplete.)
            return Ok(());
        };

        let jpeg_len = frame.data.len();
        if let Err(e) = self.write_jpeg_frame(now, &frame.data) {
            // The ONLY way to reach here is a payload that does not fit the
            // output's `max_slice_len` (`PayloadTooLarge`), or an allocation
            // failure inside the spill. Both mean the frame cannot be
            // published; neither means the node is broken. Counted
            // unconditionally, logged once per regime — the same contract, and
            // the same tested type, as every skip class in the admission gate.
            //
            // `e` carries the real numbers and the real remedy
            // (`PayloadTooLarge` names the topic, the requested size, the
            // ceiling, and says to raise `max_slice_len`), so the advice here
            // is TRUE — which it was not while this gate measured a loan.
            let log = self.oversized.record();
            let total = self.oversized.total();
            if matches!(log, FloodLog::First { .. }) {
                tracing::warn!(
                    jpeg_len,
                    total,
                    error = %e,
                    "camera JPEG DROPPED — it exceeds the output's max_slice_len ceiling, and a \
                     truncated JPEG is a corrupt JPEG. Raise max_slice_len on the camera output \
                     in the graph YAML. Repeats demoted to debug until the next frame that fits"
                );
            } else {
                tracing::debug!(
                    jpeg_len,
                    total,
                    error = %e,
                    "camera JPEG dropped: exceeds the output's max_slice_len (suppressed repeat)"
                );
            }
            // Some fields of an already-loaned output are now unwritten, so
            // the runtime discards this frame (itself a once-per-regime
            // error). Returning Ok keeps the fire healthy — the frame is lost,
            // the node is not.
            return Ok(());
        }
        // A frame that fits re-arms the latch: the next oversized one is loud
        // again.
        self.oversized.rearm();

        self.published += 1;
        Ok(())
    }

    /// Write every field of one output frame.
    ///
    /// This is a HELPER rather than inline tick code for one reason: port-field
    /// writes are FALLIBLE and `#[cerulion_node_impl]` bakes a `?` into every
    /// one of them, so a write performed in the tick body can only ever become
    /// a TICK ERROR. Behind a `Result`-returning method the caller can CATCH
    /// the failure and turn it into the counted, flood-latched frame drop the
    /// rest of this node's loss classes use.
    ///
    /// # Why `self.jpeg.data = jpeg` and not `fill_from`
    ///
    /// `fill_from` is the 0-copy form. It is nonetheless the
    /// WRONG form for this payload, because of what it can and cannot see: the
    /// producer closure is handed the space remaining in the CURRENT LOAN, and
    /// the publisher's loan is ADAPTIVE — once its sliding window is warm it
    /// loans roughly `1.5 ×` the recent maximum frame, not `max_slice_len`. A
    /// gate written against that slice therefore rejects any frame larger than
    /// ~1.5 × recent, which on a content-driven encoder is an ordinary scene
    /// change. Worse, it is SELF-SUSTAINING: the sizer only records sizes that
    /// were actually SENT, so a dropped frame never widens the window, the
    /// recent maximum never rises, and the camera stalls indefinitely — while
    /// advising the operator to raise a ceiling that was never the constraint.
    ///
    /// The whole-field write routes through the generated `set_data`, which
    /// calls `ensure_capacity_for(bytes_needed, …)` and SPILLS to the overflow
    /// buffer when the frame outgrows the loan (the `OutputProxy` re-loans to
    /// fit at Drop). So ANY frame within `max_slice_len` publishes, the sizer
    /// sees it, and the window widens on its own; only a genuinely
    /// over-CEILING frame fails — with `PayloadTooLarge`, where "raise
    /// max_slice_len" is true advice.
    ///
    /// This setter copies the JPEG into the output storage. A spill adds a
    /// copy when the publisher re-loans the SHM sample at Drop. The other
    /// codec boundary copies are listed in the module docs.
    ///
    /// `data` is written LAST: on a genuine over-ceiling refusal it is then the
    /// ONLY unwritten field, so the runtime's discard names the field that
    /// actually did not fit rather than whichever sibling the early return
    /// skipped.
    fn write_jpeg_frame(&mut self, now_ns: u64, jpeg: &[u8]) -> Result<(), NodeError> {
        // format = "jpeg" (variable string — a small copy into SHM).
        self.jpeg.format = JPEG_FORMAT;

        // Header: frame_id (variable leaf) + stamp (whole fixed nested) via the
        // staged header view — writing frame_id satisfies the header's child
        // gate. The stamp is the built-in ns→Time split from the NODE clock
        // (deterministic — Principle #7). The gst PTS on `frame` is diagnostic
        // only and never reaches the wire.
        self.jpeg.header.frame_id = CAMERA_FRAME_ID;
        self.jpeg.header.stamp = Time::from_ns(now_ns);

        // The JPEG bytes — spill-capable, see the method docs.
        self.jpeg.data = jpeg;
        Ok(())
    }

    /// Teardown summary — the one place the whole run's skip accounting is
    /// surfaced as a unit (the per-class counters are otherwise only visible
    /// through their flood-latched warns, which by design go quiet).
    fn shutdown(&mut self) -> Result<(), NodeError> {
        let gate = self.transcode.gate();
        tracing::info!(
            published = self.published,
            target_height = gate.target_height(),
            // Two counters, not one: the gate ADMITS (a decision), the loop
            // PUSHES (an outcome). They differ exactly by the access units a
            // dying pipeline refused, so a single counter would claim the
            // outcome while measuring the decision.
            aus_admitted = gate.admitted(),
            aus_pushed = self.transcode.aus_pushed(),
            skipped_other_rendition = gate.skipped_other_rendition(),
            skipped_unknown_rendition = gate.skipped_unknown_rendition(),
            skipped_waiting_for_keyframe = gate.skipped_waiting_for_sps(),
            skipped_empty = gate.skipped_empty(),
            // Named ARRIVED, not "lost": this is pre-gate, so on a
            // two-rendition topic it is ~2x the decodable loss.
            aus_arrived_while_pipeline_down = self.transcode.aus_dropped_while_down(),
            stale_jpegs_dropped = self.transcode.stale_jpegs_dropped(),
            jpegs_dropped_oversized = self.oversized.total(),
            map_failures = self.transcode.map_failure_count(),
            pipeline_builds = self.transcode.pipeline_builds(),
            "camera_jpeg teardown summary"
        );
        Ok(())
    }
}
