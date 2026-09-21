// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;
use native_ros2_messages::sensor_msgs::Range;

/// The only encoding this node understands: one little-endian `u16` per
/// pixel, one count per millimetre, as the `realsense_camera` node publishes.
const ENCODING: &str = "16UC1";
/// Depth counts per metre: one count is one millimetre. Dividing by this is
/// correctly rounded in `f32`; multiplying by an inexact `0.001` is not.
const COUNTS_PER_METRE: f32 = 1000.0;
/// The cone is the central tenth of the image in each axis (64x48 pixels of a
/// 640x480 frame).
const CONE_FRACTION: f32 = 0.1;
/// Nominal horizontal field of view of a D400-series depth stream, in degrees.
/// Not read from the device: the camera node publishes no intrinsics, so the
/// cone's angular width below is this figure times `CONE_FRACTION`.
const DEPTH_HFOV_DEG: f32 = 87.0;
/// Nominal D435 usable depth range, metres. Adjust for another model.
const MIN_RANGE_M: f32 = 0.28;
const MAX_RANGE_M: f32 = 10.0;
/// `sensor_msgs/Range` radiation type: RealSense depth is infrared stereo.
const INFRARED: u8 = 1;

// Data-triggered consumer: fires once per depth frame and publishes the
// nearest valid depth inside the central cone as a `sensor_msgs/Range`, the
// message a single-beam distance sensor would publish. Hold a hand in front of
// the camera and the range drops; take it away and it grows back.
#[cerulion_node]
#[derive(Default)]
struct NearestObstacleNode {
    #[input(trigger)]
    depth: Image,
    #[output]
    obstacle: Range,
    frames: u64,
}

#[cerulion_node_impl]
impl NearestObstacleNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // The depth counts are read in place from the borrowed shared-memory
        // sample; nothing is copied. Variable fields read through accessors.
        let encoding = self
            .depth
            .encoding()
            .map_err(|error| NodeError::InvalidInput {
                input: "depth".into(),
                reason: error.to_string(),
            })?;
        let nearest = nearest_in_cone(
            self.depth.data(),
            self.depth.width,
            self.depth.height,
            self.depth.step,
            encoding,
        )?;
        self.frames += 1;

        // Keep the depth frame's stamp and frame id. This small header copy is
        // the only copy: the depth payload itself was inspected in place.
        self.obstacle.header = self.depth.header_bytes();
        // `Range`'s remaining fields are fixed: each assignment lands straight
        // in the loaned shared-memory slot. No return in the cone publishes
        // an infinite range, the convention for "nothing detected".
        self.obstacle.radiation_type = INFRARED;
        self.obstacle.field_of_view = (DEPTH_HFOV_DEG * CONE_FRACTION).to_radians();
        self.obstacle.min_range = MIN_RANGE_M;
        self.obstacle.max_range = MAX_RANGE_M;
        self.obstacle.range = nearest.unwrap_or(f32::INFINITY);
        self.obstacle.variance = 0.0;

        if self.frames.is_multiple_of(30) {
            let frames = self.frames;
            let range_m = nearest.unwrap_or(f32::INFINITY);
            tracing::info!(range_m, frames, "nearest obstacle in the central cone");
        }
        Ok(())
    }
}

/// The smallest nonzero depth inside the central cone, in metres, or `None`
/// when every pixel there is zero (no return). Zero is the SDK's "no depth"
/// marker, never a measurement.
fn nearest_in_cone(
    data: &[u8],
    width: u32,
    height: u32,
    step: u32,
    encoding: &str,
) -> Result<Option<f32>, NodeError> {
    let (width, height, step) = (width as usize, height as usize, step as usize);
    if encoding != ENCODING
        || width == 0
        || height == 0
        || width.checked_mul(2).is_none_or(|row| row > step)
        || height.checked_mul(step) != Some(data.len())
    {
        return Err(NodeError::Logic(
            "expected a nonempty 16UC1 image with consistent stride and data length".into(),
        ));
    }
    let (x0, x1) = cone_span(width);
    let (y0, y1) = cone_span(height);
    let nearest = data
        .chunks_exact(step)
        .take(y1)
        .skip(y0)
        .flat_map(|row| {
            row[x0 * 2..x1 * 2]
                .as_chunks::<2>()
                .0
                .iter()
                .map(|pair| u16::from_le_bytes(*pair))
        })
        .filter(|&count| count != 0)
        .min();
    Ok(nearest.map(|count| f32::from(count) / COUNTS_PER_METRE))
}

/// The pixel span `[start, end)` of the central `CONE_FRACTION` of an axis,
/// always at least one pixel wide.
fn cone_span(len: usize) -> (usize, usize) {
    let span = ((len as f32 * CONE_FRACTION).round() as usize).clamp(1, len);
    let start = (len - span) / 2;
    (start, start + span)
}

#[cfg(test)]
mod tests;
