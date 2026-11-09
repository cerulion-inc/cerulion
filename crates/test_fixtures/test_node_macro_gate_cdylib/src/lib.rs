// SPDX-License-Identifier: AGPL-3.0-only
//! A period node whose tick can be held open from outside, for the
//! per-instance lock pin: two instances of this one library, loaded into one
//! process, must tick at the same time.
//!
//! While the file named by `hold_path` exists, `tick` waits in 1 ms sleeps
//! (at most 20 s). It then writes `cmd.x = tag` and returns. An empty
//! `hold_path`, the default, never waits.
//!
//! Loaded by `cdylib_instance_lock_test`.

#![deny(unused_imports)]
// P12 (the AGENTS.md logging convention): library code never prints.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use std::time::Duration;

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// The most 1 ms waits a tick makes on its hold file (about 20 s).
const HOLD_CEILING_WAITS: u32 = 20_000;

#[cerulion_node(period_ms = 20)]
struct GateNode {
    #[output]
    cmd: Vector3,
    #[param(default = "", description = "tick waits while this file exists")]
    hold_path: String,
    #[param(default = 0.0, description = "written to cmd.x")]
    tag: f64,
}

#[cerulion_node_impl]
impl GateNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        if !self.hold_path.is_empty() {
            let mut waits = 0;
            while std::path::Path::new(&self.hold_path).exists() && waits < HOLD_CEILING_WAITS {
                std::thread::sleep(Duration::from_millis(1));
                waits += 1;
            }
        }
        self.cmd.x = self.tag;
        Ok(())
    }
}
