// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion topic snapshot`: the newest frame of a camera topic as a JPEG
//! file.
//!
//! The topic opens exactly as `topic echo` opens it (a local topic, or a
//! remote robot's topic demanded through `cerulion-netd`, unless `--local`);
//! the frame becomes a JPEG through [`cerulion_msgio::snapshot`].

use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

pub use cerulion_msgio::snapshot::{
    default_file_name, frame_to_jpeg, jpeg_dimensions, take_newest_frame, to_jpeg, to_rgb,
    RawImage, SnapshotReport, DEFAULT_JPEG_QUALITY, DEFAULT_SNAPSHOT_TIMEOUT,
};

use super::{ensure_topic_available, local_walker_from_workspace, TopicScope, TopicSource};
use crate::message_text::VerbError;

/// How long the verb waits before it tries again to open a topic that could
/// not be opened.
const REOPEN_DELAY: Duration = Duration::from_millis(200);

/// What `topic snapshot` was asked to do.
#[derive(Debug, Clone)]
pub struct SnapshotRequest<'a> {
    /// The camera topic.
    pub topic: &'a str,
    /// Where to write the JPEG; `None` is a file in the system temp directory.
    pub output: Option<&'a Path>,
    /// How long to wait for a frame.
    pub timeout: Duration,
    /// JPEG quality, 1 to 100 (unused for a `CompressedImage` JPEG).
    pub quality: u8,
    /// Local only, or local plus the LAN.
    pub scope: TopicScope,
}

/// Take the newest frame of `req.topic` and write it as a JPEG.
///
/// The topic is opened again every 200 ms until it opens or
/// `req.timeout` passes.
///
/// Errors: a topic that does not open, or sends no frame, within
/// `req.timeout` (or an interrupt) is [`VerbError::Unavailable`]; a frame of
/// another type, an unsupported encoding or a frame shorter than its size
/// fields is [`VerbError::Encode`]; a file that cannot be written is
/// [`VerbError::Internal`].
pub fn topic_snapshot(
    req: &SnapshotRequest<'_>,
    schemas_dir: Option<&Path>,
    running: &AtomicBool,
) -> Result<SnapshotReport, VerbError> {
    if !(1..=100).contains(&req.quality) {
        return Err(VerbError::Usage(format!(
            "--quality must be 1 to 100, got {}",
            req.quality
        )));
    }
    let deadline = Instant::now() + req.timeout;
    let (_transport, subscriber, source) = loop {
        match ensure_topic_available(req.topic, schemas_dir, Some(running), req.scope) {
            Ok(opened) => break opened,
            Err(e) if Instant::now() >= deadline || !running.load(Ordering::Relaxed) => {
                return Err(VerbError::Unavailable(format!(
                    "'{}' could not be opened within {} s: {}",
                    req.topic,
                    req.timeout.as_secs_f64(),
                    crate::error::render_user_error(&e)
                )))
            }
            Err(_) => std::thread::sleep(REOPEN_DELAY), // ALLOW: a one-shot CLI retry while the camera topic appears, bounded by --timeout
        }
    };
    let walker = match source {
        TopicSource::Remote { walker, .. } => walker,
        TopicSource::Local => local_walker_from_workspace(schemas_dir),
    };

    let Some(frame) = take_newest_frame(&subscriber, deadline, running)? else {
        return Err(VerbError::Unavailable(format!(
            "no frame on '{}' within {} s",
            req.topic,
            req.timeout.as_secs_f64()
        )));
    };
    let (jpeg, width, height, encoding, stamp_ns) = frame_to_jpeg(&walker, &frame, req.quality)?;
    let path = match req.output {
        Some(p) => p.to_path_buf(),
        None => std::env::temp_dir().join(default_file_name(req.topic, stamp_ns)),
    };
    std::fs::write(&path, &jpeg)?;
    Ok(SnapshotReport {
        path,
        width,
        height,
        encoding,
        stamp_ns,
        bytes: jpeg.len(),
    })
}
