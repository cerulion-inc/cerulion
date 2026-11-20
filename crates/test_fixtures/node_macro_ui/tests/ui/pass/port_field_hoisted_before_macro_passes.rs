#![allow(unexpected_cfgs)]
//! Pass case: the fix the guard's hoist-it diagnostic
//! prescribes MUST compile — read the port field into a local BEFORE the
//! macro, pass the local. The `let v = self.cmd_vel.x;` statement sits
//! outside any macro token stream, so the rewriter's leaf rewrite serves
//! it normally (`__cer_cmd_vel.x` via the InputView Deref chain), and the
//! macro argument (`linear_x = v`) contains no port access for the guard
//! to intercept.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node]
#[derive(Default)]
struct HoistedReadNode {
    #[input(trigger)]
    cmd_vel: Vector3,
}

#[cerulion_node_impl]
impl HoistedReadNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // The hoist: bind first…
        let v = self.cmd_vel.x;
        // …then log the local. The guard scans the macro args, finds no
        // `self.<port>.<field>`, and leaves the macro untouched.
        cerulion_core::tracing::debug!(linear_x = v);
        Ok(())
    }
}

fn main() {
    // Reference the generated entry so the expansion is exercised.
    let _entry = HoistedReadNodeEntry::new();
}
