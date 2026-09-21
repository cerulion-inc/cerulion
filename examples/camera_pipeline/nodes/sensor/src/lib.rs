// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

// Periodic publisher: emits a `sensor_msgs/Image` every 33 ms. A source-only
// node has no input to fire it, so it must declare a non-data policy.
#[cerulion_node(period_ms = 33)]
#[derive(Default)]
struct SensorNode {
    // Bare `#[output]`: ports need no field list. Every
    // `self.image.<field> = ...` in tick() is rewritten to a fallible write
    // into the loaned shared-memory slot; the generated schema code resolves
    // fixed-vs-variable at compile time.
    #[output]
    image: Image,

    frame_count: u32,
}

#[cerulion_node_impl]
impl SensorNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.frame_count = self.frame_count.wrapping_add(1);
        // `height` and `width` are fixed fields: written straight to shared memory.
        self.image.height = 480;
        self.image.width = 640;
        self.image.step = 640 * 3;
        self.image.is_bigendian = 0;
        // A published frame must write EVERY variable-length field of its
        // schema each tick (for `Image` that is `header`, `encoding`, and
        // `data`), or the frame is discarded with an error. `frame_id` is
        // `header`'s lone variable field; nested-leaf sugar writes it in place.
        self.image.header.frame_id = "camera";
        self.image.encoding = "rgb8";
        // Generate a synthetic solid-color frame directly in the loaned buffer.
        // No temporary pixel vector or payload copy is needed.
        let shade = (self.frame_count % 256) as u8;
        self.image.loan_data(480 * 640 * 3)?.fill(shade);
        Ok(())
    }
}

#[cfg(test)]
mod tests;
