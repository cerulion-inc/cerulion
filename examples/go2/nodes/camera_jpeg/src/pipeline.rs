// SPDX-License-Identifier: AGPL-3.0-only
//! GStreamer H.264 → JPEG transcode seam for the Go2 camera node.
//!
//! Compiled ONLY under the `gstreamer` feature (see Cargo.toml). Everything
//! that TOUCHES a gst symbol lives here so the pure [`crate::frame`] /
//! [`crate::h264`] layers and the [`crate::capture`] state machine stay
//! testable with injected buffers and the crate builds without GStreamer for
//! those tests. [`GstCamera`] is the real [`crate::capture::JpegTranscoder`]:
//! the node's tick drives it through the gst-free
//! [`crate::capture::TranscodeLoop`], which owns the admission gate / rebuild /
//! failure policy (all oracle-tested without gst).
//!
//! The pipeline-description STRINGS and their config are gst-free, so they do
//! not live here: they are [`crate::pipeline_desc`], which is NOT feature-gated
//! and carries the hand-written string oracles — otherwise those oracles would
//! compile only on a machine with GStreamer and never run in CI. They are re-exported
//! below, so `pipeline::PipelineConfig` (etc.) still resolves.
//!
//! # The source is a TOPIC, not a socket
//!
//! The node takes its H.264 from a Cerulion `#[input(trigger)]` and feeds it
//! to an `appsrc`. It opens no socket, so nothing here needs a route, an
//! interface or a port.
//!
//! Reading the vendor's 230.1.1.1:1720 H.264 multicast group directly would
//! need a per-robot route fix: on a stock Go2 `ip route get 230.1.1.1`
//! resolves to `wlan0` (home WiFi) rather than `eth0` (the robot LAN). The
//! same video is already on a plain DDS topic (`/frontvideostream`) that the
//! `dds_bridge` republishes as `/go2/camera/h264`, and a camera that is a
//! topic GENERALIZES to a robot this node has never seen.
//!
//! # Pipeline selection (runtime element probe)
//!
//! At [`GstCamera::start`] the node PROBES for the NVIDIA hardware elements and picks:
//! - **Jetson / NVIDIA:** `appsrc ! h264parse ! nvv4l2decoder ! nvjpegenc ! appsink`
//!   (hardware NVDEC + hardware JPEG encode — the Jetson path).
//! - **Software fallback:** `appsrc ! h264parse ! avdec_h264 ! videoconvert ! jpegenc ! appsink`
//!   (any machine without the nv elements — development machines, CI runners).
//!
//! The choice is logged LOUDLY (`tracing::info!` naming the chosen pipeline).
//!
//! # GStreamer 1.16 compatibility
//!
//! The Go2 ships GStreamer **1.16.3** and no newer package exists in its apt
//! repos, so the crate enables NO `v1_XX` cargo feature and every API used here
//! must exist in 1.16. The call sites are all GStreamer-1.0-era:
//! `AppSrc::push_buffer`
//! (`gst_app_src_push_buffer`), `gst::Buffer::from_slice`
//! (`gst_buffer_new_wrapped_full`), `AppSink::try_pull_sample`
//! (`gst_app_sink_try_pull_sample`) and the `appsrc`
//! properties set in the launch string (`is-live`, `format`, `do-timestamp`,
//! `max-bytes`, `caps` — all present since 0.10/1.0). Deliberately NOT used:
//! `appsrc leaky-type`, which is **1.20+** and would silently not exist on the
//! robot.
//!
//! # Bus monitoring
//!
//! Both [`GstCamera::push_au`](crate::capture::JpegTranscoder::push_au) and
//! [`poll`](crate::capture::JpegTranscoder::poll) drain the pipeline's
//! GStreamer bus (non-blocking [`gst::Bus::pop`]): a bus `Error` message — a
//! mid-stream fatal (e.g. the decoder dying on a corrupt IDR) or an async
//! PLAYING failure — surfaces as [`Pull::Fatal`] with the message source +
//! error + debug text, and the `TranscodeLoop` tears down + rebuilds on
//! backoff. Without this the pipeline would sit silent-dead while the appsink
//! returned "no frame" forever.

use std::fmt;
use std::time::Duration;

use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app::{AppSink, AppSrc};

use crate::capture::{JpegTranscoder, Pull};
use crate::frame::JpegFrame;

// The PURE half of this seam — the description strings, the config that
// parameterises them, and the chain-kind enum — lives in `pipeline_desc`,
// which is NOT feature-gated, so its hand-written string oracles run on a machine
// with no GStreamer (i.e. in CI). Re-exported here so every gst-side caller
// and the loopback e2e keep resolving `pipeline::PipelineConfig` etc.
pub use crate::pipeline_desc::{
    build_description, PipelineConfig, PipelineKind, DEFAULT_APPSRC_MAX_BYTES, DEFAULT_H264_CAPS,
};

/// Probe the running GStreamer registry: the hardware path is chosen ONLY if
/// BOTH nv elements are present; otherwise the software fallback. `gst::init()`
/// must have run first (it has, at [`GstCamera::start`]).
pub fn probe_pipeline_kind() -> PipelineKind {
    let have_nvdec = gst::ElementFactory::find("nvv4l2decoder").is_some();
    let have_nvjpeg = gst::ElementFactory::find("nvjpegenc").is_some();
    if have_nvdec && have_nvjpeg {
        PipelineKind::NvidiaJetson
    } else {
        PipelineKind::Software
    }
}

/// Preflight the machine for the loopback e2e / manual bring-up: run `gst::init()`,
/// verify every element the loopback source + camera need, and confirm a
/// decode→encode path exists. Returns the [`PipelineKind`] that will be used, or
/// a LOUD [`CameraError::MissingElements`] listing exactly what is absent — so a
/// missing GStreamer is an attributable failure, NEVER a silent skip.
pub fn preflight_loopback() -> Result<PipelineKind, CameraError> {
    gst::init().map_err(|e| CameraError::Init(e.to_string()))?;
    // Elements the loopback source + camera transport/parse need on EVERY machine
    // (both pipeline variants). Decode/encode elements are variant-scoped
    // below — `videoconvert` is deliberately NOT here: only the SOFTWARE
    // chain uses it (avdec_h264 ! videoconvert ! jpegenc), so a hardware machine
    // without it must not fail preflight when its chosen NVDEC chain works.
    // `udpsrc`/`udpsink` are gone with the multicast path; `appsrc`
    // took their place as the camera's source.
    let required = ["appsrc", "h264parse", "appsink", "videotestsrc", "x264enc"];
    let mut missing: Vec<String> = required
        .iter()
        .filter(|n| gst::ElementFactory::find(n).is_none())
        .map(|n| (*n).to_string())
        .collect();
    // Variant-scoped decode+encode requirements. The probe picks NVIDIA only
    // when BOTH nv elements exist (nothing further to check on that arm); the
    // software arm needs its full chain, each absent element named.
    let kind = probe_pipeline_kind();
    if kind == PipelineKind::Software {
        for name in ["avdec_h264", "videoconvert", "jpegenc"] {
            if gst::ElementFactory::find(name).is_none() {
                missing.push(format!("{name} (software decode/encode chain)"));
            }
        }
    }
    if missing.is_empty() {
        Ok(kind)
    } else {
        Err(CameraError::MissingElements(missing))
    }
}

/// Drain `pipeline`'s bus without blocking and return the FIRST fatal `Error`
/// message formatted as `source: error (debug)`, or `None`.
///
/// Other message types are pipeline chatter here — state-change failures
/// surface as `Error` messages too. A bus `Eos` is not returned: the appsink
/// observes EOS on its own pad, which is where it is handled.
fn first_bus_error(pipeline: &gst::Pipeline) -> Option<String> {
    let bus = pipeline.bus()?;
    while let Some(msg) = bus.pop() {
        if let gst::MessageView::Error(err) = msg.view() {
            let source = msg
                .src()
                .map(|s| s.path_string().to_string())
                .unwrap_or_else(|| "<unknown element>".to_string());
            let debug = err
                .debug()
                .map(|d| d.to_string())
                .unwrap_or_else(|| "<no debug info>".to_string());
            return Some(format!("{source}: {} ({debug})", err.error()));
        }
    }
    None
}

/// A running transcode pipeline. Owns the GStreamer pipeline (PLAYING), its
/// `appsrc` (H.264 in) and its `appsink` (JPEG out). `Send` (the underlying
/// GStreamer objects are thread-safe), so the node can be moved across threads
/// by the runtime. Dropping it tears the pipeline down (sets it to NULL).
///
/// This is the real [`JpegTranscoder`]: `push_au` hands one access unit to the
/// decoder, `poll` takes at most one encoded JPEG back, and both check the BUS
/// first so a fatal error is reported instead of masquerading as "no frame".
pub struct GstCamera {
    pipeline: gst::Pipeline,
    appsrc: AppSrc,
    appsink: AppSink,
}

impl GstCamera {
    /// Initialise GStreamer, probe the element chain, build + PLAY the pipeline,
    /// and return a handle. Logs the chosen pipeline loudly. Fails loudly (never
    /// silently) on any synchronous gst error; ASYNC failures (an element that
    /// dies after reaching PLAYING) surface on the bus and are caught per-push /
    /// per-poll.
    pub fn start(cfg: &PipelineConfig) -> Result<Self, CameraError> {
        gst::init().map_err(|e| CameraError::Init(e.to_string()))?;
        let kind = probe_pipeline_kind();
        let description = build_description(kind, cfg);
        tracing::info!(
            pipeline = kind.label(),
            %description,
            "camera transcode pipeline selected"
        );
        let element =
            gst::parse::launch(&description).map_err(|e| CameraError::Parse(e.to_string()))?;
        let pipeline = element
            .downcast::<gst::Pipeline>()
            .map_err(|_| CameraError::NotPipeline)?;
        let appsrc = pipeline
            .by_name("src")
            .ok_or(CameraError::NoAppsrc)?
            .downcast::<AppSrc>()
            .map_err(|_| CameraError::NoAppsrc)?;
        let appsink = pipeline
            .by_name("sink")
            .ok_or(CameraError::NoAppsink)?
            .downcast::<AppSink>()
            .map_err(|_| CameraError::NoAppsink)?;
        // A live source reports NO-PREROLL here rather than SUCCESS; both are
        // `Ok` — only a real state-change failure is an `Err`.
        pipeline
            .set_state(gst::State::Playing)
            .map_err(|e| CameraError::StateChange(e.to_string()))?;
        Ok(Self {
            pipeline,
            appsrc,
            appsink,
        })
    }
}

impl JpegTranscoder for GstCamera {
    /// Hand ONE Annex-B access unit to the decoder.
    ///
    /// The bus is checked FIRST so a pipeline that already died is reported as
    /// itself rather than as an opaque `Flushing` push failure. The `to_vec` is
    /// the single unavoidable copy: `gst::Buffer` must OWN its memory (it
    /// outlives this call — the decoder holds it), and the source bytes live in
    /// a Cerulion SHM sample that is released when the tick returns.
    fn push_au(&mut self, au: &[u8]) -> Result<(), String> {
        if let Some(fatal) = first_bus_error(&self.pipeline) {
            return Err(fatal);
        }
        let buffer = gst::Buffer::from_slice(au.to_vec());
        self.appsrc
            .push_buffer(buffer)
            .map(|_| ())
            .map_err(|e| format!("appsrc rejected the buffer: {e}"))
    }

    /// Take at most ONE encoded JPEG back, without blocking.
    ///
    /// Bus first (a fatal wins over queued data — a dead decoder's stale sample
    /// must not mask the death), then a ZERO-timeout appsink pull. Maps the gst
    /// buffer's bytes into an owned [`JpegFrame`] (one copy out of the gst pool
    /// — the price of decoupling the publish path from gst's buffer lifetime)
    /// and carries the buffer PTS as [`JpegFrame::pts_ns`] (diagnostic only).
    /// A sample without a readable buffer is [`Pull::MapFailure`] (counted +
    /// rate-limit-warned by the loop — never folded silently into "no frame").
    ///
    /// EOS is only reported when the pull came back empty, so a bounded stream's
    /// LAST buffers are delivered before its EOS is.
    fn poll(&mut self) -> Pull {
        if let Some(fatal) = first_bus_error(&self.pipeline) {
            return Pull::Fatal(fatal);
        }
        match self.appsink.try_pull_sample(gst::ClockTime::ZERO) {
            Some(sample) => {
                let Some(buffer) = sample.buffer() else {
                    return Pull::MapFailure;
                };
                let pts_ns = buffer.pts().map(|c| c.nseconds());
                match buffer.map_readable() {
                    Ok(map) => Pull::Frame(JpegFrame {
                        data: map.as_slice().to_vec(),
                        pts_ns,
                    }),
                    Err(_) => Pull::MapFailure,
                }
            }
            None => {
                // Nothing ready — distinguish a dead/finished pipeline from a
                // decoder that simply has not produced this frame yet, so the
                // caller can rebuild instead of polling a corpse forever.
                if let Some(fatal) = first_bus_error(&self.pipeline) {
                    Pull::Fatal(fatal)
                } else if self.appsink.is_eos() {
                    Pull::Eos
                } else {
                    Pull::NoFrame
                }
            }
        }
    }
}

impl Drop for GstCamera {
    fn drop(&mut self) {
        // Best-effort teardown; ignore the state-change result on the way out.
        let _ = self.pipeline.set_state(gst::State::Null);
    }
}

/// What one [`LoopbackH264Source::pull_au`] observed.
#[derive(Debug, Clone, PartialEq)]
pub enum AuPull {
    /// One complete Annex-B access unit.
    Au(Vec<u8>),
    /// Nothing within the timeout — poll again.
    NoAu,
    /// The generator finished (`num-buffers` exhausted).
    Eos,
    /// The generator pipeline died — formatted `source: error (debug)`.
    Fatal(String),
}

/// A synthetic H.264 Annex-B generator (`videotestsrc ! x264enc !
/// video/x-h264,profile=constrained-baseline ! h264parse config-interval=-1 !
/// appsink`) for the loopback e2e + manual bring-up.
///
/// It produces a bounded number of real, NVDEC-decodable H.264 access units
/// (see [`Self::start`] for the profile / SPS-PPS constraints) which the test
/// feeds into a [`GstCamera`] exactly as the node feeds it live wire frames.
/// The appsink pull keeps a socket out of the chain: there is no port to probe,
/// no bind race to retry around, and the test exercises the SAME entry point
/// production uses.
/// Dropping it tears the generator down.
pub struct LoopbackH264Source {
    pipeline: gst::Pipeline,
    appsink: AppSink,
}

impl LoopbackH264Source {
    /// Start generating `num_buffers` frames of a `width`x`height` test pattern
    /// as H.264 access units. Uses `x264enc tune=zerolatency` so frames flow
    /// immediately.
    ///
    /// # NVDEC compatibility
    ///
    /// `nvv4l2decoder` on a Jetson REJECTS unconstrained x264enc output
    /// ("video_parser_parse Unsupported Codec"): with no downstream
    /// profile constraint, x264enc is free to negotiate a 4:4:4/high-class
    /// profile NVDEC cannot parse. The stream is therefore pinned to the most
    /// conservative decodable shape (each property verified against the
    /// gstreamer.freedesktop.org element docs):
    ///
    /// - raw caps `format=I420` — 4:2:0 input (in x264enc's sink template), so
    ///   the encoder never picks a 4:4:4 profile to match the source;
    /// - `video/x-h264,profile=constrained-baseline` capsfilter AFTER the
    ///   encoder — "the recommended way to set a profile is to set it in the
    ///   downstream caps" (x264enc docs; `constrained-baseline` is in its src
    ///   template and decodable by any baseline/main/high decoder);
    /// - `x264enc key-int-max=30` — "maximal distance between two key-frames"
    ///   = an IDR at least once per second at 30 fps;
    /// - `h264parse config-interval=-1` — "-1 = send with every IDR frame":
    ///   SPS/PPS repeat IN-BAND, so the camera's SPS gate opens on the FIRST
    ///   access unit rather than only on the stream head.
    ///
    /// `videotestsrc` is deliberately NOT `is-live=true`: nothing downstream of
    /// this appsink runs on a clock, so throttling the generator to real time
    /// would only make the test slower.
    pub fn start(
        cfg: &PipelineConfig,
        num_buffers: i32,
        width: u16,
        height: u16,
    ) -> Result<Self, CameraError> {
        gst::init().map_err(|e| CameraError::Init(e.to_string()))?;
        // Bare (unquoted) capsfilters — the profile pin after x264enc and the
        // byte-stream/au `{caps}` after h264parse (matching the camera appsrc
        // caps so the loopback is self-consistent).
        let description = format!(
            "videotestsrc num-buffers={num_buffers} ! \
             video/x-raw,format=I420,width={width},height={height},framerate=30/1 ! \
             x264enc tune=zerolatency key-int-max=30 ! \
             video/x-h264,profile=constrained-baseline ! \
             h264parse config-interval=-1 ! {caps} ! \
             appsink name=out sync=false",
            caps = cfg.h264_caps,
        );
        tracing::info!(%description, "loopback H.264 generator starting");
        let element =
            gst::parse::launch(&description).map_err(|e| CameraError::Parse(e.to_string()))?;
        let pipeline = element
            .downcast::<gst::Pipeline>()
            .map_err(|_| CameraError::NotPipeline)?;
        let appsink = pipeline
            .by_name("out")
            .ok_or(CameraError::NoAppsink)?
            .downcast::<AppSink>()
            .map_err(|_| CameraError::NoAppsink)?;
        pipeline
            .set_state(gst::State::Playing)
            .map_err(|e| CameraError::StateChange(e.to_string()))?;
        Ok(Self { pipeline, appsink })
    }

    /// Block up to `timeout` for the next generated access unit.
    pub fn pull_au(&mut self, timeout: Duration) -> AuPull {
        if let Some(fatal) = first_bus_error(&self.pipeline) {
            return AuPull::Fatal(fatal);
        }
        let timeout_ms = timeout.as_millis().min(u64::MAX as u128) as u64;
        match self
            .appsink
            .try_pull_sample(gst::ClockTime::from_mseconds(timeout_ms))
        {
            Some(sample) => {
                // Same shape as `GstCamera::poll`'s read (let-else + match), so
                // both sides of the loopback exercise one proven idiom. A sample
                // with no readable buffer is NOT a stream end — report "nothing
                // this time" and let the caller poll again.
                let Some(buffer) = sample.buffer() else {
                    return AuPull::NoAu;
                };
                match buffer.map_readable() {
                    Ok(map) => AuPull::Au(map.as_slice().to_vec()),
                    Err(_) => AuPull::NoAu,
                }
            }
            None => {
                if let Some(fatal) = first_bus_error(&self.pipeline) {
                    AuPull::Fatal(fatal)
                } else if self.appsink.is_eos() {
                    AuPull::Eos
                } else {
                    AuPull::NoAu
                }
            }
        }
    }
}

impl Drop for LoopbackH264Source {
    fn drop(&mut self) {
        let _ = self.pipeline.set_state(gst::State::Null);
    }
}

/// A GStreamer capture error — every arm carries context so the failure is
/// loud, never silent.
#[derive(Debug)]
pub enum CameraError {
    /// `gst::init()` failed.
    Init(String),
    /// The pipeline description failed to parse/link.
    Parse(String),
    /// The parsed top-level element was not a `Pipeline`.
    NotPipeline,
    /// No `appsrc` named `src` (or it was not an `AppSrc`).
    NoAppsrc,
    /// No `appsink` named `sink`/`out` (or it was not an `AppSink`).
    NoAppsink,
    /// The pipeline refused to reach the PLAYING state.
    StateChange(String),
    /// Required GStreamer elements are absent (preflight) — the listed names.
    MissingElements(Vec<String>),
}

impl fmt::Display for CameraError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CameraError::Init(e) => write!(f, "GStreamer init failed: {e}"),
            CameraError::Parse(e) => write!(f, "pipeline parse/link failed: {e}"),
            CameraError::NotPipeline => write!(f, "parsed element was not a Pipeline"),
            CameraError::NoAppsrc => write!(f, "pipeline has no appsrc named 'src'"),
            CameraError::NoAppsink => write!(f, "pipeline has no appsink under the expected name"),
            CameraError::StateChange(e) => write!(f, "pipeline failed to reach PLAYING: {e}"),
            CameraError::MissingElements(names) => {
                write!(f, "missing GStreamer elements: {}", names.join(", "))
            }
        }
    }
}

impl std::error::Error for CameraError {}
