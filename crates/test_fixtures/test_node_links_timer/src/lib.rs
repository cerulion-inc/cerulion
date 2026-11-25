// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::std_msgs::Int32;

#[cerulion_node(period_ms = 50)]
#[derive(Default)]
pub struct TestNodeLinksTimerNode {
    #[output]
    count: Int32,
    tick_count: u32,
}

#[cerulion_node_impl]
impl TestNodeLinksTimerNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.tick_count += 1;
        // TODO: populate count (Int32)
        Ok(())
    }
}
