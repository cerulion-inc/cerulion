#![allow(unexpected_cfgs)]
//! `#[on_event]` attached to a RESERVED lifecycle
//! method (`init` here) must fail to compile with a clean, actionable error.
//! Without the guard the method is collected as a handler AND rewritten by the
//! lifecycle branch, so the generated dispatch calls `self.init(ev)` — a method
//! the rewrite already consumed — yielding a confusing post-expansion E0599.
//!
//! `init` (not `tick`) is used so the node still has a valid `tick` for
//! `#[cerulion_node_impl]`, keeping the PRIMARY error the new diagnostic.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct ReservedMethodNode {
    #[input(backpressure = sample(15))]
    inp: Vector3,
    seen: f64,
}

#[cerulion_node_impl]
impl ReservedMethodNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen = self.inp.x;
        Ok(())
    }

    // `#[on_event]` on the reserved `init` lifecycle method — rejected.
    #[on_event(input = "inp")]
    fn init(&mut self, _event: BackpressureEvent) {}
}

fn main() {}
