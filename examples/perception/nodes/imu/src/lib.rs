// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Imu;

// Deterministic 100 Hz synthetic IMU. It shares the graph with the ~30 Hz
// camera: two periodic sources at different rates, each with its own
// `period_ms` on its own macro, and nothing in the graph file says how fast
// either runs. Every value is a pure function of the sample counter (no wall
// clock, no randomness), so a recording of it replays byte for byte.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct ImuNode {
    #[output]
    reading: Imu,
    sample: u32,
}

#[cerulion_node_impl]
impl ImuNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.sample = self.sample.wrapping_add(1);
        // A published frame must write EVERY variable-length field of its
        // schema each tick. `Imu` has one, `header`, whose own variable field
        // is `frame_id`.
        self.reading.header.frame_id = "imu";
        // The rest of `Imu` is fixed nested messages and fixed arrays, so each
        // leaf assignment lands straight in the loaned shared-memory slot.
        // Leaves never assigned stay zero; an all-zero covariance means
        // "unknown" by ROS convention.
        self.reading.orientation.w = 1.0;
        self.reading.angular_velocity.z = 0.5;
        // A 1 Hz sawtooth on the x axis so the stream visibly changes.
        self.reading.linear_acceleration.x = f64::from(self.sample % 100) * 0.01;
        self.reading.linear_acceleration.z = 9.81;
        Ok(())
    }
}
