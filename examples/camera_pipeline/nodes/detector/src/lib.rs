// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::PoseStamped;
use native_ros2_messages::sensor_msgs::Image;

// A deterministic brightness detector, triggered by each borrowed image.
#[cerulion_node]
#[derive(Default)]
struct DetectorNode {
    #[input(trigger)]
    image: Image,
    #[output]
    detection: PoseStamped,
}

#[cerulion_node_impl]
impl DetectorNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let center = bright_center(
            self.image.data(),
            self.image.width,
            self.image.height,
            self.image.step,
            self.image
                .encoding()
                .map_err(|error| NodeError::InvalidInput {
                    input: "image".into(),
                    reason: error.to_string(),
                })?,
        )?;
        // Nothing bright in this frame: leave the output untouched. An output
        // a tick never writes is never loaned, so nothing is published.
        let Some((x, y)) = center else {
            return Ok(());
        };
        // Preserve the acquisition metadata. This small header copy is distinct
        // from the image payload, which is inspected directly in shared memory.
        self.detection.header = self.image.header_bytes();
        // `pose` is a fixed nested message, so each leaf assignment lands
        // straight in the loaned shared-memory slot. Leaves that are never
        // assigned (the orientation's x, y and z) stay zero.
        self.detection.pose.position.x = x;
        self.detection.pose.position.y = y;
        self.detection.pose.position.z = 1.0;
        self.detection.pose.orientation.w = 1.0;
        Ok(())
    }
}

// Synthetic camera model: fx = width, fy = height, principal point at the
// image center, and a front-facing plane at z = 1 m. No physical calibration
// or depth inference is implied. RGB mean intensity >= 128 counts as bright.
fn bright_center(
    data: &[u8],
    width: u32,
    height: u32,
    step: u32,
    encoding: &str,
) -> Result<Option<(f64, f64)>, NodeError> {
    let (width, height, step) = (width as usize, height as usize, step as usize);
    if encoding != "rgb8"
        || width == 0
        || height == 0
        || width.checked_mul(3).is_none_or(|row| row > step)
        || height.checked_mul(step) != Some(data.len())
    {
        return Err(NodeError::Logic(
            "expected a nonempty rgb8 image with consistent stride and data length".into(),
        ));
    }
    let mut count = 0usize;
    let (mut sum_x, mut sum_y) = (0.0, 0.0);
    for (y, row) in data.chunks_exact(step).enumerate() {
        for (x, rgb) in row[..width * 3].as_chunks::<3>().0.iter().enumerate() {
            if u16::from(rgb[0]) + u16::from(rgb[1]) + u16::from(rgb[2]) >= 3 * 128 {
                count += 1;
                sum_x += x as f64;
                sum_y += y as f64;
            }
        }
    }
    Ok((count > 0).then(|| {
        (
            (sum_x / count as f64 - (width - 1) as f64 / 2.0) / width as f64,
            (sum_y / count as f64 - (height - 1) as f64 / 2.0) / height as f64,
        )
    }))
}

#[cfg(test)]
mod tests;
