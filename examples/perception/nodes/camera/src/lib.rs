// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

// Deterministic ~30 Hz synthetic source. Three 20×20 rectangles move with the
// tick counter. No wall clock or randomness affects the recorded payload.
#[cerulion_node(period_ms = 33)]
#[derive(Default)]
struct CameraNode {
    #[output]
    image_raw: Image,
    frame: u32,
}

#[cerulion_node_impl]
impl CameraNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.frame = self.frame.wrapping_add(1);
        self.image_raw.height = 128;
        self.image_raw.width = 128;
        self.image_raw.step = 128;
        self.image_raw.is_bigendian = 0;
        self.image_raw.header.frame_id = "camera";
        self.image_raw.encoding = "mono8";
        let pixels = self.image_raw.loan_data(128 * 128)?;
        pixels.fill(0);
        for (i, shade) in [230, 204, 179].into_iter().enumerate() {
            let left = 10 + 40 * i + (self.frame % 7) as usize;
            let top = 10 + 40 * i + (self.frame % 5) as usize;
            for y in top..top + 20 {
                pixels[y * 128 + left..y * 128 + left + 20].fill(shade);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
