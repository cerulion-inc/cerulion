// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::std_msgs::Int32;

// Periodic source: fires every 100 ms and publishes an incrementing counter.
// A source-only node has no input to fire it, so its trigger policy is the
// `period_ms` on the macro. The graph file carries no policy at all.
#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct TimerNode {
    #[output]
    count: Int32,

    tick_count: i32,
}

#[cerulion_node_impl]
impl TimerNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.tick_count = self.tick_count.wrapping_add(1);
        // `Int32` is a fixed-only schema, so this assignment writes straight
        // into the loaned shared-memory slot. The frame is published when
        // `tick` returns `Ok`; a tick that never touches `count` publishes
        // nothing.
        self.count.data = self.tick_count;
        Ok(())
    }
}
