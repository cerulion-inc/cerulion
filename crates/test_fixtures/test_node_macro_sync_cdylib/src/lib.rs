// SPDX-License-Identifier: AGPL-3.0-only
//! F6 macro→runtime policy round-trip oracle for Sync.
//!
//! Two trigger inputs so this fixture has a non-empty input
//! set when macro_policy_to_trigger threads input names through.

#![deny(unused_imports)]
// P12 (the AGENTS.md logging rule): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(sync_window_ms = 25)]
struct SyncNode {
    #[input(trigger, depth = 1)]
    cam: Vector3,

    #[input(trigger, depth = 1)]
    imu: Vector3,

    #[output]
    fused: Vector3,
}

#[cerulion_node_impl]
impl SyncNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.fused.x = self.cam.x + self.imu.x;
        Ok(())
    }
}
