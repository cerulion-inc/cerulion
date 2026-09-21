// SPDX-License-Identifier: AGPL-3.0-only
//! Pure frame types for the Go2 camera JPEG node.
//!
//! This module is unsafe-free, std-only, and holds NO GStreamer types, so
//! everything here is oracle-testable with INJECTED byte buffers — no camera,
//! no gst, no transport. The `pipeline` module (the gst seam), `capture` (the
//! transcode loop) and `lib.rs` (the node) depend on the types here; nothing
//! here depends on them.
//!
//! Two concerns live here:
//!
//! 1. [`JpegFrame`] — one encoded JPEG handed from the decoder to the node's
//!    publish path.
//! 2. [`is_jpeg`] / [`jpeg_dimensions`] — a minimal structural JPEG reader used
//!    by the loopback e2e to verify the pipeline emitted a real, decodable JPEG
//!    (SOI/EOI markers + SOF frame dimensions), not opaque bytes.
//!
//! There is no frame queue here: with the camera fed by a Cerulion
//! `#[input(trigger)]` there is no helper thread and no cross-thread hand-off —
//! decode and publish happen in the same tick. The newest-wins-plus-count
//! drop-oldest policy applies where the frames actually are, in
//! [`crate::capture::TranscodeLoop`]'s bounded drain of the encoder
//! (`stale_jpegs_dropped`), and is oracle-tested there.
//!
//! The `header.stamp` split (node-clock ns → ROS `Time { sec, nanosec }`) is
//! NOT hand-rolled here: `lib.rs` stamps via the built-in
//! `native_ros2_messages::builtin_interfaces::Time::from_ns` helper, which is
//! deterministic (fed the node clock, not a wall clock) so the published stamp
//! is a pure function of the scheduler's clock (Principle #7 — see the node
//! docs).

// Pure by construction. (The forbid lives HERE, not at the crate root, because
// the `#[cerulion_node]` macro in lib.rs expands cdylib FFI entry points
// containing `unsafe`.)
#![forbid(unsafe_code)]

/// One encoded JPEG frame pulled out of the transcode pipeline.
///
/// `data` is the JPEG byte payload (owned — the gst-mapped buffer is copied out
/// ONCE, at the `appsink`, so nothing downstream holds a live gst pool buffer;
/// this is what keeps the publish path pure + injectable). `pts_ns` is the gst
/// buffer presentation timestamp (nanoseconds, pipeline-clock domain) when
/// known — it is DIAGNOSTIC only (the wire stamp is derived from the node clock
/// at publish via `Time::from_ns`, see the node docs).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct JpegFrame {
    /// The encoded JPEG bytes.
    pub data: Vec<u8>,
    /// The gst buffer PTS in ns (pipeline clock), if the buffer carried one.
    /// Diagnostic only — never written to the wire.
    pub pts_ns: Option<u64>,
}

// The node-clock ns → ROS `Time { sec, nanosec }` split is the built-in
// `native_ros2_messages::builtin_interfaces::Time::from_ns` helper, called
// straight into `self.jpeg.header.stamp` in `lib.rs`. The stamp is
// derived from the NODE clock (`self.now_ns()` in the tick), NOT the gst buffer
// PTS or any wall clock, so the published stamp is a pure function of the
// scheduler's clock (Principle #7); the gst PTS→node-clock DELTA stays out of
// the wire (available as [`JpegFrame::pts_ns`] for bring-up latency only).

/// True iff `data` looks like a complete JPEG: it starts with the SOI marker
/// `FF D8` and ends with the EOI marker `FF D9`. A cheap structural gate — NOT
/// a full decode.
pub fn is_jpeg(data: &[u8]) -> bool {
    data.len() >= 4
        && data[0] == 0xFF
        && data[1] == 0xD8
        && data[data.len() - 2] == 0xFF
        && data[data.len() - 1] == 0xD9
}

/// Parse a JPEG's `(width, height)` from its first Start-Of-Frame (SOF) marker,
/// or `None` if `data` is not a JPEG or carries no SOF before it ends.
///
/// Walks the marker segments from the SOI, skipping each length-prefixed
/// segment until an SOF marker (`FF C0..=CF`, excluding the non-SOF `C4` DHT /
/// `C8` JPG / `CC` DAC), whose payload is `precision(1) height(2, big-endian)
/// width(2, big-endian) components(1)`. Used by the loopback e2e to confirm the
/// pipeline decoded + re-encoded the expected frame size.
pub fn jpeg_dimensions(data: &[u8]) -> Option<(u16, u16)> {
    if data.len() < 4 || data[0] != 0xFF || data[1] != 0xD8 {
        return None;
    }
    let mut i = 2usize;
    // Each iteration needs at least a 2-byte marker + 2-byte length.
    while i + 4 <= data.len() {
        if data[i] != 0xFF {
            // Not aligned on a marker — malformed for our purposes.
            return None;
        }
        let marker = data[i + 1];
        let is_sof =
            (0xC0..=0xCF).contains(&marker) && marker != 0xC4 && marker != 0xC8 && marker != 0xCC;
        if is_sof {
            // SOF payload: [Lf hi][Lf lo][precision][H hi][H lo][W hi][W lo]...
            // height at i+5..=i+6, width at i+7..=i+8 (i+2,i+3 = segment length).
            if i + 9 > data.len() {
                return None;
            }
            let height = u16::from_be_bytes([data[i + 5], data[i + 6]]);
            let width = u16::from_be_bytes([data[i + 7], data[i + 8]]);
            return Some((width, height));
        }
        // Non-SOF: skip this segment by its declared length (which includes the
        // 2 length bytes but not the 2 marker bytes).
        let seg_len = u16::from_be_bytes([data[i + 2], data[i + 3]]) as usize;
        if seg_len < 2 {
            return None;
        }
        i += 2 + seg_len;
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- is_jpeg / jpeg_dimensions ----

    /// Minimal but STRUCTURALLY REAL JPEG: SOI, APP0/JFIF, SOF0 (precision 8,
    /// height H, width W, 3 components), then EOI. Enough for the structural
    /// readers under test (not a decodable image — that is the loopback e2e's
    /// job with a real encoder).
    fn synthetic_jpeg(width: u16, height: u16) -> Vec<u8> {
        let mut v = vec![0xFF, 0xD8]; // SOI
                                      // APP0 (JFIF) segment: FF E0, length 16, "JFIF\0", version, units, etc.
        v.extend_from_slice(&[0xFF, 0xE0, 0x00, 0x10]);
        v.extend_from_slice(b"JFIF\0");
        v.extend_from_slice(&[0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00]);
        // SOF0: FF C0, length 17, precision 8, H(2 BE), W(2 BE), 3 components.
        v.extend_from_slice(&[0xFF, 0xC0, 0x00, 0x11, 0x08]);
        v.extend_from_slice(&height.to_be_bytes());
        v.extend_from_slice(&width.to_be_bytes());
        v.extend_from_slice(&[0x03]); // component count
        v.extend_from_slice(&[0x01, 0x22, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01]);
        v.extend_from_slice(&[0xFF, 0xD9]); // EOI
        v
    }

    // 10 — is_jpeg accepts a well-formed SOI..EOI and rejects non-JPEG.
    #[test]
    fn is_jpeg_accepts_soi_eoi_rejects_others() {
        assert!(is_jpeg(&synthetic_jpeg(1280, 720)));
        assert!(!is_jpeg(&[0x00, 0x01, 0x02, 0x03]), "no SOI");
        assert!(!is_jpeg(&[0xFF, 0xD8, 0x00, 0x00]), "no EOI");
        assert!(!is_jpeg(&[0xFF, 0xD8]), "too short");
        assert!(!is_jpeg(&[]), "empty");
    }

    // 11 — jpeg_dimensions reads (width, height) from the SOF marker.
    #[test]
    fn jpeg_dimensions_reads_sof() {
        assert_eq!(
            jpeg_dimensions(&synthetic_jpeg(1280, 720)),
            Some((1280, 720)),
            "720p"
        );
        assert_eq!(jpeg_dimensions(&synthetic_jpeg(64, 48)), Some((64, 48)));
    }

    // 12 — jpeg_dimensions returns None on non-JPEG / no-SOF input.
    #[test]
    fn jpeg_dimensions_none_on_garbage() {
        assert_eq!(jpeg_dimensions(&[0x00; 32]), None, "no SOI");
        assert_eq!(
            jpeg_dimensions(&[0xFF, 0xD8, 0xFF, 0xD9]),
            None,
            "SOI+EOI but no SOF segment"
        );
        // SOI + a single length-prefixed non-SOF segment that runs off the end
        // has no SOF → None (also proves the length-skip does not panic).
        assert_eq!(jpeg_dimensions(&[0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10]), None);
    }
}
