// SPDX-License-Identifier: AGPL-3.0-only
//! Params round-trip oracle: a period node whose output is a function of its
//! four `#[param]` fields, so a graph that stages this type twice with
//! different `params:` publishes observably different frames per instance.
//!
//! `cmd.x` = `gain`, `cmd.y` = `offset`, `cmd.z` = the byte length of
//! `frame_id` when `enabled`, else `-1.0`. With the defaults that is
//! `(1.0, 0.0, 9.0)` (`"base_link"` is nine bytes).
//!
//! Loaded by `cdylib_params_test` (the optional
//! `cerulion_node_init_with_params` symbol) and staged by the CLI's
//! `graph_params_e2e_test`.

#![deny(unused_imports)]
// P12 (the AGENTS.md logging convention): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 20)]
struct ParamNode {
    #[output]
    cmd: Vector3,
    #[param(
        default = 1.0,
        min = 0.0,
        max = 10.0,
        description = "multiplier written to cmd.x"
    )]
    gain: f64,
    #[param(default = 0, min = -100, max = 100, description = "written to cmd.y")]
    offset: i32,
    #[param(
        default = "base_link",
        description = "its byte length is written to cmd.z"
    )]
    frame_id: String,
    #[param(default = true, description = "false writes -1.0 to cmd.z")]
    enabled: bool,
}

#[cerulion_node_impl]
impl ParamNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.x = self.gain;
        self.cmd.y = f64::from(self.offset);
        self.cmd.z = if self.enabled {
            self.frame_id.len() as f64
        } else {
            -1.0
        };
        Ok(())
    }
}
