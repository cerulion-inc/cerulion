// SPDX-License-Identifier: AGPL-3.0-only
//! The `test_node_macro_param_cdylib` node rebuilt with one more param:
//! `scale`, default `3.0`. It stands in for "rebuild the node with a new
//! param" without an in-test cargo build.
//!
//! Everything a resume reads besides the params is identical to the original:
//! the same `#[cerulion_node(period_ms = 20)]` trigger, the same `ParamNode`
//! type name, the same single `cmd: Vector3` output port and no field that is
//! node state. A param is not in the state shape, so an anchor the original
//! captured restores into this build. The four original params keep their
//! names, types, defaults and bounds, so a recording of the original names a
//! value for each of them and for nothing this build does not declare. Only
//! `scale` differs: the recording names no value for it.
//!
//! `cmd.x` = `scale`, `cmd.y` = `gain + offset`, `cmd.z` = the byte length of
//! `frame_id` when `enabled`, else `-1.0`. With the defaults that is
//! `(3.0, 1.0, 9.0)`.
//!
//! Staged by the CLI's `graph_params_e2e_test` in place of the original
//! between a capture and its resim.

#![deny(unused_imports)]
// P12 (the AGENTS.md logging convention): library code never prints; it logs through
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
        description = "added to offset, written to cmd.y"
    )]
    gain: f64,
    #[param(default = 0, min = -100, max = 100, description = "added to gain")]
    offset: i32,
    #[param(
        default = "base_link",
        description = "its byte length is written to cmd.z"
    )]
    frame_id: String,
    #[param(default = true, description = "false writes -1.0 to cmd.z")]
    enabled: bool,
    #[param(default = 3.0, min = 0.0, max = 10.0, description = "written to cmd.x")]
    scale: f64,
}

#[cerulion_node_impl]
impl ParamNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.x = self.scale;
        self.cmd.y = self.gain + f64::from(self.offset);
        self.cmd.z = if self.enabled {
            self.frame_id.len() as f64
        } else {
            -1.0
        };
        Ok(())
    }
}
