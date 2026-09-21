// SPDX-License-Identifier: AGPL-3.0-only
//! The PURE half of the camera's GStreamer seam: the pipeline-description
//! strings and the config that parameterises them.
//!
//! # Why this is its own module
//!
//! These are plain string builders over plain data — no `gstreamer` symbol
//! appears below, and none of it needs a GStreamer runtime. Inside
//! `crate::pipeline`, which is `#[cfg(feature = "gstreamer")]`, their
//! oracle tests would compile only on a machine that has GStreamer's dev headers — i.e.
//! never in the `demos-go2` CI job, which deliberately installs no GStreamer.
//! Keeping the pure half separate is what lets those oracles run on every PR: the
//! strings that decide whether the robot decodes anything are the single most
//! fragile thing in the crate, and a hand-written oracle for them is worthless
//! if nothing runs it.
//!
//! `crate::pipeline` re-exports everything here, so `pipeline::PipelineConfig`
//! (etc.) keeps resolving for the gst-gated callers and the loopback e2e.

// Pure by construction — same module-scope forbid as `frame` / `h264` /
// `capture` (the crate root cannot carry it: the `#[cerulion_node]` macro
// expands cdylib FFI entry points containing `unsafe`).
#![forbid(unsafe_code)]

/// Caps declared on the `appsrc`: raw H.264 Annex-B, one access unit per
/// buffer — EXACTLY the shape `unitree_go/Go2FrontVideoData.video_data`
/// carries (start-code delimited, `00 00 00 01`; an IDR access unit carries
/// SPS + PPS + IDR). No depayloading and no re-containerizing is involved, so
/// there is no raw-vs-RTP question to resolve on
/// the robot: the bridge hands this node elementary-stream access units by
/// construction.
///
/// Shared by the camera's `appsrc` and the loopback test source's final
/// capsfilter so the two are self-consistent.
pub const DEFAULT_H264_CAPS: &str = "video/x-h264,stream-format=byte-stream,alignment=au";

/// Default `appsrc max-bytes=`: 8 MiB of queued, not-yet-decoded H.264.
///
/// # What this does and does NOT bound
///
/// On the GStreamer the Go2 ships (**1.16.3**) this is **advisory, not a
/// ceiling**. `appsrc` enforces `max-bytes` only through `block=true` (which
/// would stall `push_buffer`, and therefore the node's whole graph step — not
/// acceptable here) or through `leaky-type`, which is **1.20+** and so cannot
/// be used at all (see the crate's Cargo.toml note). With `block` left at its
/// default `false` and no leaky-type, reaching `max-bytes` emits `enough-data`
/// and makes the appsrc report itself full — but `push_buffer` still ACCEPTS
/// the buffer and the internal queue keeps growing. 8 MiB is therefore NOT
/// a bound a stopped decoder cannot grow past.
///
/// What actually bounds the backlog, in the order it bites:
///
/// 1. **The node can only push once per fire.** The `h264` input is a normal
///    `drop_oldest` Cerulion input, so a node that falls behind has the OLDEST
///    access units evicted by the TRANSPORT (counted there) rather than being
///    handed an unbounded backlog to shovel at the decoder. One tick pushes at
///    most one access unit.
/// 2. **A decoder that DIES is torn down, appsrc queue included.** A bus
///    `Error` surfaces as `capture::Pull::Fatal`, the whole pipeline is
///    dropped to NULL and rebuilt on the backoff — the queue dies with it.
///
/// The residual this leaves, stated plainly: a decoder that WEDGES without
/// ever posting a bus error would keep accepting pushes and grow the queue
/// without bound. At the measured worst case (~62 KB for a 720p IDR) and ~15
/// target-rendition access units per second that is ~1 MB/s.
///
/// And nothing in this node OBSERVES the crossing: `enough-data` is a GObject
/// signal this node does not connect, and `appsrc`'s fill level is never queried, so
/// reaching 8 MiB produces no log line, no counter and no state change here.
/// The accurate description of this constant is therefore: a value that
/// would become a real ceiling the moment `leaky-type` can be enabled (i.e. a
/// GStreamer >= 1.20 floor), kept at a size that costs nothing meanwhile. It
/// is NOT a mitigation and should not be read as one.
pub const DEFAULT_APPSRC_MAX_BYTES: u64 = 8 * 1024 * 1024;

/// Which decode/encode element chain the running machine supports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PipelineKind {
    /// NVIDIA hardware path: `nvv4l2decoder` + `nvjpegenc` (Jetson Orin).
    NvidiaJetson,
    /// Portable software path: `avdec_h264` + `videoconvert` + `jpegenc`.
    Software,
}

impl PipelineKind {
    /// A short human label for the selection log line.
    pub fn label(self) -> &'static str {
        match self {
            PipelineKind::NvidiaJetson => "NVIDIA Jetson (nvv4l2decoder -> nvjpegenc)",
            PipelineKind::Software => "software (avdec_h264 -> jpegenc)",
        }
    }
}

/// How the camera pipeline's `appsrc` is typed and bounded.
///
/// This was emptied of everything network-shaped (`address`, `port`,
/// `multicast_iface`, `udp_buffer_bytes`): the H.264 now arrives on a Cerulion
/// topic, so there is nothing to bind, join or route.
#[derive(Debug, Clone)]
pub struct PipelineConfig {
    /// `appsrc` caps for the H.264 payload (see [`DEFAULT_H264_CAPS`]).
    pub h264_caps: String,
    /// `appsrc max-bytes=` — the queue high-water mark in bytes (see
    /// [`DEFAULT_APPSRC_MAX_BYTES`], and read its note on what this does and
    /// does not bound on GStreamer 1.16).
    pub appsrc_max_bytes: u64,
}

impl PipelineConfig {
    /// The Go2 front-camera stream as the bridge republishes it: raw Annex-B
    /// access units, one per buffer.
    pub fn production() -> Self {
        Self {
            h264_caps: DEFAULT_H264_CAPS.to_string(),
            appsrc_max_bytes: DEFAULT_APPSRC_MAX_BYTES,
        }
    }
}

impl Default for PipelineConfig {
    fn default() -> Self {
        Self::production()
    }
}

/// Build the gst-launch pipeline description for `kind` reading per `cfg`.
pub fn build_description(kind: PipelineKind, cfg: &PipelineConfig) -> String {
    // `appsrc` caps must be quoted (the caps string contains commas).
    //
    // `is-live=true` — the frames arrive in real time from a camera; telling
    // gst so is what makes the pipeline treat it as a live source rather than
    // trying to preroll one.
    // `format=time` + `do-timestamp=true` — stamp each pushed buffer with the
    // pipeline's running time at push. The wire `time_frame` is the CAMERA's
    // own monotonic counter in an epoch that cannot be mapped onto the pipeline clock,
    // so deriving a PTS from it would be a guess; `do-timestamp` is the
    // 1.16-safe way to give the decoder a monotonic, correctly-spaced clock.
    // (The PUBLISHED `header.stamp` is unaffected — it comes from the node
    // clock in the tick, so it stays replay-deterministic.)
    // `max-bytes` — see DEFAULT_APPSRC_MAX_BYTES. `block` is left
    // default-false: a push must never stall the graph step.
    let src = format!(
        "appsrc name=src is-live=true format=time do-timestamp=true \
         max-bytes={mb} caps=\"{caps}\"",
        mb = cfg.appsrc_max_bytes,
        caps = cfg.h264_caps,
    );
    // `sync=false` (emit as decoded, do not clock-throttle), `max-buffers=2
    // drop=true` (appsink-side drop-oldest: never let stale JPEG back-pressure
    // the decoder — the same latest-wins policy the TranscodeLoop's drain
    // applies on the other side of it).
    //
    // `drop=true` IS a loss channel, and the ONE in this node that is not
    // counted: gst drops the buffer inside the appsink and the application
    // never learns of it. It is accepted because it is BOUNDED AT TWO (the
    // appsink holds at most `max-buffers` and evicts the oldest) and because
    // what it drops is by construction stale — the `TranscodeLoop` drain
    // applies the identical latest-wins rule one layer out and DOES count
    // there (`stale_jpegs_dropped`). See the loss-accounting section of the
    // crate docs.
    let tail = "appsink name=sink sync=false max-buffers=2 drop=true";
    match kind {
        PipelineKind::NvidiaJetson => {
            format!("{src} ! h264parse ! nvv4l2decoder ! nvjpegenc ! {tail}")
        }
        PipelineKind::Software => {
            format!("{src} ! h264parse ! avdec_h264 ! videoconvert ! jpegenc ! {tail}")
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // String-oracle pins for the pure pipeline-string
    // builders. Hand-written full pipeline strings (never rebuilt from the
    // same format! parts — that would be a tautology).
    //
    // These live HERE rather than in `crate::pipeline` precisely so they run
    // without GStreamer — i.e. in the `demos-go2` CI job, on every PR.

    // 1 — the Jetson hardware chain over the production config.
    #[test]
    fn description_jetson_production_matches_hand_oracle() {
        let cfg = PipelineConfig::production();
        assert_eq!(
            build_description(PipelineKind::NvidiaJetson, &cfg),
            "appsrc name=src is-live=true format=time do-timestamp=true \
             max-bytes=8388608 caps=\"video/x-h264,stream-format=byte-stream,alignment=au\" ! \
             h264parse ! nvv4l2decoder ! nvjpegenc ! \
             appsink name=sink sync=false max-buffers=2 drop=true"
        );
    }

    // 2 — the software fallback chain.
    #[test]
    fn description_software_matches_hand_oracle() {
        let cfg = PipelineConfig::production();
        assert_eq!(
            build_description(PipelineKind::Software, &cfg),
            "appsrc name=src is-live=true format=time do-timestamp=true \
             max-bytes=8388608 caps=\"video/x-h264,stream-format=byte-stream,alignment=au\" ! \
             h264parse ! avdec_h264 ! videoconvert ! jpegenc ! \
             appsink name=sink sync=false max-buffers=2 drop=true"
        );
    }

    // 3 — a caps override is quoted verbatim, and a custom max-bytes lands in
    // the property (the two knobs that remain configurable later).
    #[test]
    fn description_caps_and_max_bytes_overrides() {
        let cfg = PipelineConfig {
            h264_caps: "video/x-h264,stream-format=avc".to_string(),
            appsrc_max_bytes: 4096,
        };
        assert_eq!(
            build_description(PipelineKind::Software, &cfg),
            "appsrc name=src is-live=true format=time do-timestamp=true \
             max-bytes=4096 caps=\"video/x-h264,stream-format=avc\" ! \
             h264parse ! avdec_h264 ! videoconvert ! jpegenc ! \
             appsink name=sink sync=false max-buffers=2 drop=true"
        );
    }

    // 4 — the config constructor (field oracle, incl. the 8 MiB default) and
    // that `Default` is the production config, not a zeroed struct.
    #[test]
    fn config_constructors_match_hand_oracle() {
        let p = PipelineConfig::production();
        assert_eq!(
            p.h264_caps,
            "video/x-h264,stream-format=byte-stream,alignment=au"
        );
        assert_eq!(p.appsrc_max_bytes, 8 * 1024 * 1024);
        let d = PipelineConfig::default();
        assert_eq!(d.h264_caps, p.h264_caps);
        assert_eq!(d.appsrc_max_bytes, p.appsrc_max_bytes);
    }

    // 5 — the selection-log labels.
    #[test]
    fn kind_labels_match_hand_oracle() {
        assert_eq!(
            PipelineKind::NvidiaJetson.label(),
            "NVIDIA Jetson (nvv4l2decoder -> nvjpegenc)"
        );
        assert_eq!(
            PipelineKind::Software.label(),
            "software (avdec_h264 -> jpegenc)"
        );
    }
}
