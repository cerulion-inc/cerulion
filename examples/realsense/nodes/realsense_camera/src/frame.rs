//! Pure helpers: the stream shape this example requests, the two published
//! encodings, plane validation, and the latest-wins hand-over slot between the
//! SDK's helper thread and `tick`. Nothing here touches the SDK, so all of it
//! runs in the unit tests with or without librealsense2 installed.
//!
//! The depth unit and the slot's `offer` are called only by the SDK face;
//! without the `realsense` feature the unit tests are their only caller, so
//! the dead-code lint is quiet there.
#![cfg_attr(not(feature = "realsense"), allow(dead_code))]

use cerulion_core::prelude::NodeError;

/// The stream shape requested from the camera. Every D400-series camera
/// offers 640x480 at 30 frames per second for both streams.
pub const WIDTH: u32 = 640;
pub const HEIGHT: u32 = 480;
pub const FPS: u32 = 30;

/// Color is requested as packed 8-bit RGB: three bytes per pixel, published
/// with the ROS encoding `rgb8`.
pub const COLOR_ENCODING: &str = "rgb8";
pub const COLOR_BYTES_PER_PIXEL: u32 = 3;
/// Depth is requested as Z16: one little-endian `u16` per pixel, published
/// with the ROS encoding `16UC1`. Each count is one depth unit.
pub const DEPTH_ENCODING: &str = "16UC1";
pub const DEPTH_BYTES_PER_PIXEL: u32 = 2;
/// The depth unit this example publishes: one count is one millimetre. The
/// driver reads the camera's configured unit at start and refuses any other,
/// so `16UC1` frames on the topic are always millimetres.
pub const DEPTH_UNIT_M: f32 = 0.001;

/// `header.frame_id` of the two streams, the names realsense-ros uses.
pub const COLOR_FRAME_ID: &str = "camera_color_optical_frame";
pub const DEPTH_FRAME_ID: &str = "camera_depth_optical_frame";

/// One image plane as the SDK hands it over: dimensions, row stride, and the
/// pixel bytes borrowed from the SDK's own frame buffer.
#[derive(Debug, Clone, Copy)]
pub struct Plane<'a> {
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    pub bytes: &'a [u8],
}

/// Refuse a plane whose shape does not match what was requested, or whose
/// byte count disagrees with its own stride and height. The published image
/// must be self-consistent before a consumer indexes into it.
pub fn check_plane(plane: &Plane<'_>, bytes_per_pixel: u32) -> Result<(), NodeError> {
    let expected_len = u64::from(plane.stride) * u64::from(plane.height);
    if plane.width != WIDTH
        || plane.height != HEIGHT
        || plane.stride < plane.width * bytes_per_pixel
        || u64::try_from(plane.bytes.len()) != Ok(expected_len)
    {
        return Err(NodeError::Logic(format!(
            "frame is {}x{} with stride {} and {} bytes; expected {}x{} at {} bytes per pixel",
            plane.width,
            plane.height,
            plane.stride,
            plane.bytes.len(),
            WIDTH,
            HEIGHT,
            bytes_per_pixel
        )));
    }
    Ok(())
}

/// The hand-over slot: the SDK's helper thread offers each frameset, `tick`
/// takes the latest one. A frameset offered before the previous one was taken
/// replaces it, and the replacement is counted, so a consumer that falls
/// behind sees the newest frame and an exact tally rather than a backlog.
pub struct Slot<T> {
    latest: Option<T>,
    overwritten: u64,
}

impl<T> Default for Slot<T> {
    fn default() -> Self {
        Self {
            latest: None,
            overwritten: 0,
        }
    }
}

impl<T> Slot<T> {
    /// Store `value`, replacing (and counting) an untaken predecessor.
    pub fn offer(&mut self, value: T) {
        if self.latest.replace(value).is_some() {
            self.overwritten += 1;
        }
    }

    /// Take the latest value, leaving the slot empty.
    pub fn take(&mut self) -> Option<T> {
        self.latest.take()
    }

    /// How many offered values were replaced before they were taken.
    pub fn overwritten(&self) -> u64 {
        self.overwritten
    }
}
