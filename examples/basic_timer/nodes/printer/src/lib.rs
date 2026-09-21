// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::std_msgs::Int32;

// Data-triggered sink: `#[input(trigger)]` fires this node once per published
// count, so there is no `period_ms` here. A sink declares no `#[output]`.
#[cerulion_node]
#[derive(Default)]
struct PrinterNode {
    #[input(trigger)]
    count: Int32,

    received: u32,
}

#[cerulion_node_impl]
impl PrinterNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.received = self.received.wrapping_add(1);
        // A fixed field reads straight out of the borrowed shared-memory
        // sample: no copy, no allocation. Bind it to a local first, because
        // the macro does not rewrite port reads inside another macro's
        // arguments.
        let value = self.count.data;
        let received = self.received;
        tracing::info!(value, received, "count received");
        Ok(())
    }
}
