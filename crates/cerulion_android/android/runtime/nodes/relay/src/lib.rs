// SPDX-License-Identifier: AGPL-3.0-only
// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::std_msgs::Int32;

// Relay: `#[input(trigger)]` fires this node once per frame on `count`, and
// each tick copies the value to `echo`. A value on `echo` therefore proves a
// frame entered this node through its input.
#[cerulion_node]
#[derive(Default)]
pub struct RelayNode {
    #[input(trigger)]
    count: Int32,

    #[output]
    echo: Int32,
}

#[cerulion_node_impl]
impl RelayNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let value = self.count.data;
        self.echo.data = value;
        Ok(())
    }
}
