#![allow(unexpected_cfgs)]
//! The motivating case — a FIXED-field READ of an `#[input]` port inside
//! `tracing::debug!` field syntax (`linear_x = self.cmd_vel.x`). Legal
//! everywhere else, but inside a foreign macro's tokens the rewriter's
//! `self.<port>` -> `__cer_<port>` leaf rewrite never fires (macro tokens
//! are opaque to syn), so the access would reach rustc as a field read on
//! the zero-sized port MARKER — a baffling E0609. The guard must intercept
//! it with the hoist-it compile_error instead.
//!
//! Uses the `cerulion_core::tracing` re-export so the fixture needs no
//! extra dependency; the guard renders the FULL macro path
//! (`cerulion_core::tracing::debug!`) in the diagnostic.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node]
#[derive(Default)]
struct InputReadInMacroNode {
    #[input(trigger)]
    cmd_vel: Vector3,
}

#[cerulion_node_impl]
impl InputReadInMacroNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Should produce the hoist-it compile_error — NOT an E0609
        // on the Vector3 marker. tracing's `key = value` field syntax
        // parses as an `Expr::Assign`, which the guard descends into.
        cerulion_core::tracing::debug!(linear_x = self.cmd_vel.x);
        Ok(())
    }
}

fn main() {}
