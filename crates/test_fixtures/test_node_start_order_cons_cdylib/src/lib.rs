// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
pub struct ConsNode {
    tick_count: u32,
    #[input]
    inp: Vector3,
}

#[cerulion_node_impl]
impl ConsNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.tick_count += 1;
        Ok(())
    }
}
