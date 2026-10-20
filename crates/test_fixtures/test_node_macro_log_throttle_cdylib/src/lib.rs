// SPDX-License-Identifier: AGPL-3.0-only
//! A fast periodic node that logs on every tick through a throttle and a
//! once site, then asks the graph to stop.
//!
//! Driven by `cerulion_cli/tests/log_throttle_e2e_test.rs` through
//! `cerulion graph run`. It ticks every 10 ms of graph time and stops itself
//! on its 200th tick, so a run covers ticks 0 to 1990 ms after the first.
//! Read off this source, the stderr oracle is:
//!
//! - `log fixture first tick`: once, on the first tick;
//! - `log fixture throttled`: on the first tick and then every 250 ms of
//!   graph time, so at 0, 250, .., 1750 ms after the first tick: 8 lines.
//!
//! Each line carries `tick_ns`, the graph time it was emitted at, so two
//! runs can be compared line for line.

#![deny(unused_imports)]
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use std::time::Duration;

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// The tick on which the node asks the graph to stop.
const LAST_TICK: u32 = 200;

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct LogThrottleNode {
    #[output]
    cmd: Vector3,
    ticks: u32,
}

#[cerulion_node_impl]
impl LogThrottleNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let tick_ns = self.now_ns();
        warn_throttle!(
            self,
            Duration::from_millis(250),
            tick_ns,
            "log fixture throttled"
        );
        info_once!(self, tick_ns, "log fixture first tick");
        self.ticks += 1;
        if self.ticks == LAST_TICK {
            self.request_shutdown();
        }
        self.cmd.x = f64::from(self.ticks);
        Ok(())
    }
}
