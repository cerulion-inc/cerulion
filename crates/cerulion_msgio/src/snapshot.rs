// SPDX-License-Identifier: AGPL-3.0-only
//! A camera frame as JPEG, and the newest frame of a topic: the engine of
//! `cerulion topic snapshot` and the robot-side `topic-snapshot` and
//! `topic-echo-once` verbs.
//!
//! A `sensor_msgs/Image` frame in `yuv422_yuy2` (also spelled `yuyv`),
//! `rgb8`, `bgr8`, `rgba8`, `bgra8` or `mono8` is converted to RGB (YUYV with
//! the BT.601 limited-range matrix) and encoded; a
//! `sensor_msgs/CompressedImage` frame holding a JPEG is written as it came.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::codegen::{FrameValue, FrameValueKind, FrameWalker};
use cerulion_core::wire::WireHeader;

use cerulion_core::{CerulionSubscriber, TransportManager};

use crate::text::VerbError;

/// The default `--timeout`: how long the verb waits for a frame.
pub const DEFAULT_SNAPSHOT_TIMEOUT: Duration = Duration::from_secs(5);

/// The default `--quality` of the JPEG encoder (1 to 100).
pub const DEFAULT_JPEG_QUALITY: u8 = 80;

/// How long the verb waits before it tries again to open a topic that could
/// not be opened.
const REOPEN_DELAY: Duration = Duration::from_millis(200);

/// The longest single wait for a frame, so Ctrl-C is seen promptly.
const WAIT_SLICE: Duration = Duration::from_millis(100);

/// What `topic snapshot` wrote.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SnapshotReport {
    /// The JPEG file.
    pub path: PathBuf,
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// The frame's own encoding (`yuv422_yuy2`, `rgb8`, ...) or, for a
    /// `CompressedImage`, its `format` text.
    pub encoding: String,
    /// The frame's `header.stamp` in nanoseconds.
    pub stamp_ns: i64,
    /// Size of the JPEG file in bytes.
    pub bytes: usize,
}

impl SnapshotReport {
    /// The one JSON line `topic snapshot` prints.
    pub fn json_line(&self) -> String {
        serde_json::json!({
            "path": self.path.to_string_lossy(),
            "width": self.width,
            "height": self.height,
            "encoding": self.encoding,
            "stamp_ns": self.stamp_ns,
            "bytes": self.bytes,
        })
        .to_string()
    }
}

/// A raw `sensor_msgs/Image` frame's pixel fields.
#[derive(Debug, Clone, Copy)]
pub struct RawImage<'a> {
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels.
    pub height: u32,
    /// Bytes per row.
    pub step: u32,
    /// The ROS encoding name.
    pub encoding: &'a str,
    /// The pixel bytes.
    pub data: &'a [u8],
}

/// Open the local shared-memory topic `topic` for reading, trying again
/// every 200 ms until it opens, `deadline` passes or `running`
/// clears ([`VerbError::Unavailable`] naming the last open error).
pub fn open_local_topic(
    topic: &str,
    deadline: Instant,
    running: &AtomicBool,
) -> Result<(Arc<TransportManager>, CerulionSubscriber), VerbError> {
    let transport = TransportManager::get_or_init()?;
    loop {
        match transport.create_subscriber_open_only(topic) {
            Ok(subscriber) => return Ok((transport, subscriber)),
            Err(e) if Instant::now() >= deadline || !running.load(Ordering::Relaxed) => {
                return Err(VerbError::Unavailable(format!(
                    "'{}' could not be opened on this machine: {e}",
                    topic.escape_debug()
                )))
            }
            Err(_) => std::thread::sleep(REOPEN_DELAY), // ALLOW: a one-shot retry while the topic appears, bounded by the caller's deadline
        }
    }
}

/// The newest frame (header included) `subscriber` receives before
/// `deadline`: the last of the first batch that arrives. `None` when none
/// arrives in time or `running` clears.
pub fn take_newest_frame(
    subscriber: &CerulionSubscriber,
    deadline: Instant,
    running: &AtomicBool,
) -> Result<Option<Vec<u8>>, VerbError> {
    let mut newest: Option<Vec<u8>> = None;
    while newest.is_none() && running.load(Ordering::Relaxed) {
        let now = Instant::now();
        if now >= deadline {
            break;
        }
        subscriber.wait_for_message((deadline - now).min(WAIT_SLICE), |msg| {
            let mut frame = vec![0u8; WireHeader::SIZE];
            msg.header().write_to_buf(&mut frame);
            frame.extend_from_slice(msg.payload());
            newest = Some(frame);
        })?;
    }
    Ok(newest)
}

/// `cerulion-snapshot-<topic>-<stamp>.jpg`, the topic's slashes and other
/// non-alphanumeric characters as `_`.
pub fn default_file_name(topic: &str, stamp_ns: i64) -> String {
    let name: String = topic
        .trim_matches('/')
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
        .collect();
    format!("cerulion-snapshot-{name}-{stamp_ns}.jpg")
}

/// Decode one wire frame (header included) and return the JPEG bytes,
/// width, height, encoding text and `header.stamp` in nanoseconds.
pub fn frame_to_jpeg(
    walker: &FrameWalker,
    frame: &[u8],
    quality: u8,
) -> Result<(Vec<u8>, u32, u32, String, i64), VerbError> {
    let value = walker.walk_by_hash(frame).map_err(|e| {
        VerbError::Encode(format!(
            "the frame is not a sensor_msgs/Image or sensor_msgs/CompressedImage this machine \
             can decode: {e}"
        ))
    })?;
    let stamp_ns = stamp_ns(&value);
    match value.schema_name.as_str() {
        "sensor_msgs/Image" => {
            let image = RawImage {
                width: u32_field(&value, "width")?,
                height: u32_field(&value, "height")?,
                step: u32_field(&value, "step")?,
                encoding: str_field(&value, "encoding")?,
                data: bytes_field(&value, "data")?,
            };
            let jpeg = to_jpeg(&image, quality)?;
            Ok((
                jpeg,
                image.width,
                image.height,
                image.encoding.to_string(),
                stamp_ns,
            ))
        }
        "sensor_msgs/CompressedImage" => {
            let format = str_field(&value, "format")?;
            let data = bytes_field(&value, "data")?;
            let (width, height) = jpeg_dimensions(data).ok_or_else(|| {
                VerbError::Encode(format!(
                    "the CompressedImage (format {format:?}) does not hold a JPEG; only JPEG is \
                     supported"
                ))
            })?;
            Ok((data.to_vec(), width, height, format.to_string(), stamp_ns))
        }
        other => Err(VerbError::Encode(format!(
            "the topic carries {other}, not sensor_msgs/Image or sensor_msgs/CompressedImage"
        ))),
    }
}

fn field<'v, 'a>(
    value: &'v FrameValue<'a>,
    name: &str,
) -> Result<&'v FrameValueKind<'a>, VerbError> {
    value
        .field(name)
        .ok_or_else(|| VerbError::Encode(format!("the frame has no `{name}` field")))
}

fn u32_field(value: &FrameValue<'_>, name: &str) -> Result<u32, VerbError> {
    match field(value, name)? {
        FrameValueKind::U32(v) => Ok(*v),
        other => Err(VerbError::Encode(format!(
            "`{name}` is {other:?}, not a uint32"
        ))),
    }
}

fn str_field<'a>(value: &FrameValue<'a>, name: &str) -> Result<&'a str, VerbError> {
    match field(value, name)? {
        FrameValueKind::Str(s) => Ok(s),
        _ => Err(VerbError::Encode(format!("`{name}` is not a UTF-8 string"))),
    }
}

fn bytes_field<'a>(value: &FrameValue<'a>, name: &str) -> Result<&'a [u8], VerbError> {
    match field(value, name)? {
        FrameValueKind::Bytes(b) => Ok(b),
        _ => Err(VerbError::Encode(format!("`{name}` is not a byte array"))),
    }
}

/// `header.stamp` as nanoseconds; 0 when the frame has no such field.
fn stamp_ns(value: &FrameValue<'_>) -> i64 {
    let Some(FrameValueKind::Nested(header)) = value.field("header") else {
        return 0;
    };
    let Some(FrameValueKind::Nested(stamp)) = header.field("stamp") else {
        return 0;
    };
    let sec = match stamp.field("sec") {
        Some(FrameValueKind::I32(s)) => i64::from(*s),
        _ => 0,
    };
    let nanosec = match stamp.field("nanosec") {
        Some(FrameValueKind::U32(n)) => i64::from(*n),
        _ => 0,
    };
    sec * 1_000_000_000 + nanosec
}

/// `bytes` as standard base64 text (with padding).
pub fn base64_encode(bytes: &[u8]) -> String {
    use base64::Engine as _;
    base64::engine::general_purpose::STANDARD.encode(bytes)
}

/// The bytes of standard base64 text; `None` for text that is not base64.
pub fn base64_decode(text: &str) -> Option<Vec<u8>> {
    use base64::Engine as _;
    base64::engine::general_purpose::STANDARD.decode(text).ok()
}

/// The width and height a JPEG's header declares; `None` for bytes that are
/// not a JPEG.
pub fn jpeg_dimensions(data: &[u8]) -> Option<(u32, u32)> {
    use image::ImageDecoder;
    if !data.starts_with(&[0xFF, 0xD8]) {
        return None;
    }
    image::codecs::jpeg::JpegDecoder::new(std::io::Cursor::new(data))
        .ok()
        .map(|d| d.dimensions())
}

fn clamp(v: i32) -> u8 {
    v.clamp(0, 255) as u8
}

/// BT.601 limited-range YUV to RGB, integer form.
fn yuv_to_rgb(y: u8, u: u8, v: u8) -> [u8; 3] {
    let c = i32::from(y) - 16;
    let d = i32::from(u) - 128;
    let e = i32::from(v) - 128;
    [
        clamp((298 * c + 409 * e + 128) >> 8),
        clamp((298 * c - 100 * d - 208 * e + 128) >> 8),
        clamp((298 * c + 516 * d + 128) >> 8),
    ]
}

/// The image as tightly packed RGB8.
pub fn to_rgb(image: &RawImage<'_>) -> Result<Vec<u8>, VerbError> {
    let (w, h, step) = (
        image.width as usize,
        image.height as usize,
        image.step as usize,
    );
    let bpp = match image.encoding {
        "yuv422_yuy2" | "yuyv" => 2,
        "rgb8" | "bgr8" => 3,
        "rgba8" | "bgra8" => 4,
        "mono8" => 1,
        other => {
            return Err(VerbError::Encode(format!(
                "unsupported image encoding {other:?}; supported: yuv422_yuy2, yuyv, rgb8, \
                 bgr8, rgba8, bgra8, mono8"
            )))
        }
    };
    let row = w.checked_mul(bpp);
    let needed = row.and_then(|r| {
        h.checked_sub(1)
            .map_or(Some(0), |rows| step.checked_mul(rows)?.checked_add(r))
    });
    let (Some(row), Some(needed)) = (row, needed) else {
        return Err(short(image, usize::MAX));
    };
    if step < row || image.data.len() < needed {
        return Err(short(image, needed));
    }
    let mut out = Vec::with_capacity(w * h * 3);
    for r in 0..h {
        let line = &image.data[r * step..r * step + row];
        match image.encoding {
            "yuv422_yuy2" | "yuyv" => {
                for x in 0..w {
                    let pair = (x / 2) * 4;
                    let y = line[x * 2];
                    let (u, v) = (line[pair + 1], *line.get(pair + 3).unwrap_or(&128));
                    out.extend_from_slice(&yuv_to_rgb(y, u, v));
                }
            }
            "rgb8" => out.extend_from_slice(line),
            "bgr8" => {
                for px in line.as_chunks::<3>().0 {
                    out.extend_from_slice(&[px[2], px[1], px[0]]);
                }
            }
            "rgba8" => {
                for px in line.as_chunks::<4>().0 {
                    out.extend_from_slice(&px[..3]);
                }
            }
            "bgra8" => {
                for px in line.as_chunks::<4>().0 {
                    out.extend_from_slice(&[px[2], px[1], px[0]]);
                }
            }
            _ => {
                for &g in line {
                    out.extend_from_slice(&[g, g, g]);
                }
            }
        }
    }
    Ok(out)
}

fn short(image: &RawImage<'_>, needed: usize) -> VerbError {
    VerbError::Encode(format!(
        "the frame's {} data bytes are fewer than its size fields need ({needed}: {}x{}, step {})",
        image.data.len(),
        image.width,
        image.height,
        image.step
    ))
}

/// The image as JPEG bytes at `quality` (1 to 100).
pub fn to_jpeg(image: &RawImage<'_>, quality: u8) -> Result<Vec<u8>, VerbError> {
    let rgb = to_rgb(image)?;
    let mut out = Vec::new();
    let mut encoder = image::codecs::jpeg::JpegEncoder::new_with_quality(&mut out, quality);
    encoder
        .encode(
            &rgb,
            image.width,
            image.height,
            image::ExtendedColorType::Rgb8,
        )
        .map_err(|e| VerbError::Encode(format!("JPEG encode failed: {e}")))?;
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::text::{message_frame, EXIT_ENCODE};

    fn image<'a>(encoding: &'a str, w: u32, h: u32, bpp: u32, data: &'a [u8]) -> RawImage<'a> {
        RawImage {
            width: w,
            height: h,
            step: w * bpp,
            encoding,
            data,
        }
    }

    #[test]
    fn yuyv_converts_with_the_bt601_matrix() {
        // Hand-computed: Y=16 U=V=128 is black, Y=235 U=V=128 is white,
        // Y=81 U=90 V=240 is BT.601 red (255, 0, 0 after clamping).
        let data = [16, 128, 235, 128, 81, 90, 81, 240];
        let rgb = to_rgb(&image("yuv422_yuy2", 4, 1, 2, &data)).unwrap();
        assert_eq!(&rgb[0..3], &[0, 0, 0]);
        assert_eq!(&rgb[3..6], &[255, 255, 255]);
        assert_eq!(&rgb[6..9], &[255, 0, 0]);
        assert_eq!(rgb, to_rgb(&image("yuyv", 4, 1, 2, &data)).unwrap());
    }

    #[test]
    fn channel_orders_and_grey_convert_to_rgb() {
        assert_eq!(
            to_rgb(&image("bgr8", 1, 1, 3, &[1, 2, 3])).unwrap(),
            vec![3, 2, 1]
        );
        assert_eq!(
            to_rgb(&image("rgba8", 1, 1, 4, &[1, 2, 3, 9])).unwrap(),
            vec![1, 2, 3]
        );
        assert_eq!(
            to_rgb(&image("bgra8", 1, 1, 4, &[1, 2, 3, 9])).unwrap(),
            vec![3, 2, 1]
        );
        assert_eq!(
            to_rgb(&image("mono8", 2, 1, 1, &[7, 8])).unwrap(),
            vec![7, 7, 7, 8, 8, 8]
        );
    }

    #[test]
    fn a_padded_step_skips_the_row_padding() {
        let data = [1, 2, 3, 0xEE, 4, 5, 6, 0xEE];
        let img = RawImage {
            width: 1,
            height: 2,
            step: 4,
            encoding: "rgb8",
            data: &data,
        };
        assert_eq!(to_rgb(&img).unwrap(), vec![1, 2, 3, 4, 5, 6]);
    }

    #[test]
    fn short_frames_and_unknown_encodings_are_encode_errors() {
        let err = to_rgb(&image("rgb8", 2, 2, 3, &[0; 11])).unwrap_err();
        assert_eq!(err.exit_code(), EXIT_ENCODE);
        let err = to_rgb(&image("bayer_rggb8", 2, 2, 1, &[0; 4])).unwrap_err();
        assert_eq!(err.exit_code(), EXIT_ENCODE);
        assert!(err.to_string().contains("bayer_rggb8"));
        let huge = RawImage {
            width: u32::MAX,
            height: u32::MAX,
            step: u32::MAX,
            encoding: "rgb8",
            data: &[],
        };
        assert_eq!(to_rgb(&huge).unwrap_err().exit_code(), EXIT_ENCODE);
    }

    #[test]
    fn an_image_frame_encodes_a_jpeg_of_its_size_and_reports_its_stamp() {
        let (walker, _) = FrameWalker::new(crate::builtin_schemas());
        let data: Vec<u8> = (0..4 * 2 * 2).map(|i| (i * 13) as u8).collect();
        let text = format!(
            "{{\"header\": {{\"stamp\": {{\"sec\": 3, \"nanosec\": 4}}}}, \"height\": 2, \
             \"width\": 4, \"encoding\": \"yuv422_yuy2\", \"step\": 8, \"data\": {data:?}}}"
        );
        let frame = message_frame(&walker, "sensor_msgs/Image", &text, 0).unwrap();
        let (jpeg, w, h, enc, stamp) = frame_to_jpeg(&walker, &frame, 80).unwrap();
        assert_eq!(
            (w, h, enc.as_str(), stamp),
            (4, 2, "yuv422_yuy2", 3_000_000_004)
        );
        assert_eq!(jpeg_dimensions(&jpeg), Some((4, 2)));
    }

    #[test]
    fn a_compressed_jpeg_passes_through_unchanged() {
        let (walker, _) = FrameWalker::new(crate::builtin_schemas());
        let jpeg = to_jpeg(&image("mono8", 3, 5, 1, &[100; 15]), 90).unwrap();
        let text = format!("{{\"format\": \"jpeg\", \"data\": {jpeg:?}}}");
        let frame = message_frame(&walker, "sensor_msgs/CompressedImage", &text, 0).unwrap();
        let (out, w, h, enc, _) = frame_to_jpeg(&walker, &frame, 80).unwrap();
        assert_eq!(out, jpeg);
        assert_eq!((w, h, enc.as_str()), (3, 5, "jpeg"));
        let png = message_frame(
            &walker,
            "sensor_msgs/CompressedImage",
            "{\"format\": \"png\", \"data\": [137, 80, 78, 71]}",
            0,
        )
        .unwrap();
        assert_eq!(
            frame_to_jpeg(&walker, &png, 80).unwrap_err().exit_code(),
            EXIT_ENCODE
        );
    }

    #[test]
    fn a_frame_of_another_type_is_an_encode_error() {
        let (walker, _) = FrameWalker::new(crate::builtin_schemas());
        let frame = message_frame(&walker, "std_msgs/String", "{}", 0).unwrap();
        let err = frame_to_jpeg(&walker, &frame, 80).unwrap_err();
        assert_eq!(err.exit_code(), EXIT_ENCODE);
        assert!(err.to_string().contains("std_msgs/String"), "{err}");
    }

    #[test]
    fn the_default_file_name_and_report_line_are_stable() {
        assert_eq!(
            default_file_name("/wrist/image_raw", 12),
            "cerulion-snapshot-wrist_image_raw-12.jpg"
        );
        let report = SnapshotReport {
            path: PathBuf::from("/tmp/a.jpg"),
            width: 640,
            height: 480,
            encoding: "yuyv".to_string(),
            stamp_ns: 5,
            bytes: 100,
        };
        let v: serde_json::Value = serde_json::from_str(&report.json_line()).unwrap();
        assert_eq!(
            v,
            serde_json::json!({"path": "/tmp/a.jpg", "width": 640, "height": 480,
                "encoding": "yuyv", "stamp_ns": 5, "bytes": 100})
        );
    }
}
