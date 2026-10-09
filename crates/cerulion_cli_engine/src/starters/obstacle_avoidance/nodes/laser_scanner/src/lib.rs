// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::LaserScan;

// Periodic laser range-finder source. A real driver would read the scan off
// the hardware (e.g. via `self.scan.ranges.fill_from(|buf| driver.read(buf))`);
// here we synthesize a simple forward-facing sweep so the example runs with no
// hardware attached. The nearest reading oscillates so the downstream safety
// controller alternates between "clear" and "obstacle" states.
#[cerulion_node(period_ms = 20)]
#[derive(Default)]
struct LaserScannerNode {
    // Bare `#[output]`: ports need no field list. Plain assignment writes
    // both fixed and variable-length fields into the loaned shared-memory
    // slot.
    #[output]
    scan: LaserScan,

    tick_count: u32,
}

#[cerulion_node_impl]
impl LaserScannerNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.tick_count = self.tick_count.wrapping_add(1);

        // Fixed fields: written straight to shared memory.
        self.scan.angle_min = -std::f32::consts::FRAC_PI_2;
        self.scan.angle_max = std::f32::consts::FRAC_PI_2;
        self.scan.angle_increment = std::f32::consts::PI / 179.0;
        self.scan.scan_time = 0.02;
        self.scan.time_increment = 0.0;
        self.scan.range_min = 0.1;
        self.scan.range_max = 10.0;

        // A published frame must write EVERY variable-length field of its
        // schema each tick (for `LaserScan` that is `header`, `ranges`, and
        // `intensities`), or the frame is discarded with an error. `frame_id`
        // is `header`'s lone variable field; `intensities` is optional by ROS
        // convention, so an empty write satisfies the gate.
        self.scan.header.frame_id = "laser";

        // Synthesize a 180-beam sweep. Every ~2 s the obstacle moves
        // inside the 0.5 m safety threshold and back out again.
        let close = self.tick_count % 200 < 100;
        let nearest = if close { 0.3 } else { 5.0 };
        self.scan.loan_ranges(180)?.fill(nearest);
        self.scan.intensities = &[][..];

        Ok(())
    }
}

#[cfg(test)]
mod tests;
