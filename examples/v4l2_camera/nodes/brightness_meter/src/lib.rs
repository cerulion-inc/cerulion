// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;
use native_ros2_messages::std_msgs::Float32;

/// The only encoding this meter understands: packed YUYV, the format the
/// `v4l2_camera` node publishes.
const ENCODING: &str = "yuv422_yuy2";

// Data-triggered consumer: fires once per published camera frame and publishes
// the frame's mean luma (0 dark, 255 bright). Cover the lens and the number
// drops; that is the whole demonstration.
#[cerulion_node]
#[derive(Default)]
struct BrightnessMeterNode {
    #[input(trigger)]
    image: Image,
    #[output]
    brightness: Float32,
    frames: u64,
}

#[cerulion_node_impl]
impl BrightnessMeterNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // The pixel bytes are read in place from the borrowed shared-memory
        // sample; nothing is copied. Variable fields read through accessors.
        let encoding = self
            .image
            .encoding()
            .map_err(|error| NodeError::InvalidInput {
                input: "image".into(),
                reason: error.to_string(),
            })?;
        let mean = mean_luma(
            self.image.data(),
            self.image.width,
            self.image.height,
            self.image.step,
            encoding,
        )?;
        self.frames += 1;
        // `Float32` is a fixed-only schema: this assignment writes straight
        // into the loaned shared-memory slot, and the frame publishes when
        // `tick` returns `Ok`.
        self.brightness.data = mean;
        if self.frames.is_multiple_of(30) {
            let frames = self.frames;
            tracing::info!(mean_luma = mean, frames, "brightness measured");
        }
        Ok(())
    }
}

/// Mean of the Y (luma) samples of a packed YUYV image. Every second byte of
/// a row is a Y sample; the U and V bytes between them are skipped, and so is
/// any padding past `width * 2` bytes in a row.
fn mean_luma(
    data: &[u8],
    width: u32,
    height: u32,
    step: u32,
    encoding: &str,
) -> Result<f32, NodeError> {
    let (width, height, step) = (width as usize, height as usize, step as usize);
    if encoding != ENCODING
        || width == 0
        || height == 0
        || width.checked_mul(2).is_none_or(|row| row > step)
        || height.checked_mul(step) != Some(data.len())
    {
        return Err(NodeError::Logic(
            "expected a nonempty yuv422_yuy2 image with consistent stride and data length".into(),
        ));
    }
    let sum: u64 = data
        .chunks_exact(step)
        .flat_map(|row| row[..width * 2].iter().step_by(2))
        .map(|&y| u64::from(y))
        .sum();
    Ok(sum as f32 / (width * height) as f32)
}

#[cfg(test)]
mod tests;
