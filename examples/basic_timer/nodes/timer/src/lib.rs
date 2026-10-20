// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::std_msgs::Int32;

// Periodic source: fires every 100 ms and publishes a counter that goes up by
// `step`. A source-only node has no input to fire it, so its trigger policy is
// the `period_ms` on the macro. The graph file carries no policy at all; it
// carries the VALUE of `step` under the node's `params:`.
#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct TimerNode {
    #[output]
    count: Int32,

    // A typed setting: the field type is the param type, the graph file sets
    // it per node (`params: {step: 2}`), and an absent value keeps the default.
    // A value outside 1..=1000 or of another type fails the graph at load.
    #[param(default = 1, min = 1, max = 1000, description = "how much count goes up per tick")]
    step: i32,

    tick_count: i32,
}

#[cerulion_node_impl]
impl TimerNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.tick_count = self.tick_count.wrapping_add(self.step);
        // `Int32` is a fixed-only schema, so this assignment writes straight
        // into the loaned shared-memory slot. The frame is published when
        // `tick` returns `Ok`; a tick that never touches `count` publishes
        // nothing.
        self.count.data = self.tick_count;
        Ok(())
    }
}
