#![allow(unexpected_cfgs)]
//! A single method carrying TWO `#[on_event(...)]`
//! attributes must fail to compile. Without the guard the second attribute
//! silently overwrites the first (registering only the last port's handler),
//! so the duplicate is rejected at expansion time instead.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct DupAttrNode {
    #[input(backpressure = sample(15))]
    a: Vector3,
    #[input(backpressure = sample(15))]
    b: Vector3,
    seen: f64,
}

#[cerulion_node_impl]
impl DupAttrNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen = self.a.x + self.b.x;
        Ok(())
    }

    // Two attributes on one method — the first would be silently dropped.
    #[on_event(input = "a")]
    #[on_event(input = "b")]
    fn on_pressure(&mut self, _event: BackpressureEvent) {}
}

fn main() {}
