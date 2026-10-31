// SPDX-License-Identifier: AGPL-3.0-only
//! Real-bag e2e: the PERTURBED twin of `test_node_macro_period_cdylib`
//! — same fire schedule, different payload bytes; exists so the real-bag e2e
//! can prove exit 1 without an in-test cargo build (the CI stand-in for
//! "rebuild the node with a changed constant").
//!
//! The node SHAPE is byte-for-byte identical to the original fixture: the same
//! `#[cerulion_node(period_ms = 50)]` trigger, the same `PeriodNode` type name,
//! the same single `cmd: Vector3` output port + schema, and the same
//! `request_shutdown` reconstruct field (a state field is part of the state
//! shape hash, so the twin must carry it too). Only the tick's
//! published payload constant differs (`cmd.x = 1000.0` vs the original's
//! `0.0`). So a bag recorded with the ORIGINAL node replays byte-EXACT against
//! the original candidate (exit 0) but diverges on the payload — never the
//! schedule — against THIS candidate (exit 1, a data violation, not an exit-6
//! structural trace divergence).

#![deny(unused_imports)]
// P12 (the logging convention in AGENTS.md): library code never prints. It
// logs through `tracing`. Scoped `not(test)` so unit tests keep printing
// diagnostics, and applied at the crate root rather than in `[workspace.lints]`
// because that table cannot distinguish a lib target from a test binary.
// Pinned by `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 50)]
struct PeriodNode {
    #[output]
    cmd: Vector3,
    /// Mirrors the original fixture's field so the two state shapes match.
    #[cerulion(reconstruct)]
    request_shutdown: bool,
}

#[cerulion_node_impl]
impl PeriodNode {
    fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
        self.request_shutdown = ctx.env_opt("CER_PERIOD_REQUEST_SHUTDOWN").is_some();
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        // The ONLY delta vs test_node_macro_period_cdylib (which writes 0.0):
        // a different published constant → different payload bytes, identical
        // fire schedule.
        self.cmd.x = 1000.0;
        if self.request_shutdown {
            self.request_shutdown();
        }
        Ok(())
    }
}
