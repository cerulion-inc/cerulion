// SPDX-License-Identifier: AGPL-3.0-only
//! Relay node that runs only when linked into another library.
//!
//! It has one trigger input with the same schema as the output of
//! `test_node_macro_period_cdylib`, so a graph can wire the period node into
//! it. Each tick copies the input's `x` and writes a tick count into `z`
//! (1, 2, 3, ...). A tick runs only when a frame arrives on the trigger
//! input, so a subscriber that sees `z` count up proves frames entered this
//! node through its input.

#![deny(unused_imports)]
// Library code never prints; scoped `not(test)` as in the other fixtures.
// Pinned by `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node]
pub struct RelayNode {
    #[input(trigger)]
    cmd: Vector3,

    #[output]
    echo: Vector3,

    ticks: u64,
}

#[cerulion_node_impl]
impl RelayNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.ticks += 1;
        self.echo.x = self.cmd.x;
        self.echo.z = self.ticks as f64;
        Ok(())
    }
}
