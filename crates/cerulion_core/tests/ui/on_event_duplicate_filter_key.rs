#![allow(unexpected_cfgs)]
//! A SINGLE `#[on_event(...)]` attribute that sets
//! the SAME filter key twice (`input = "a", input = "b"`) must fail to compile.
//! Without the guard the second value silently overwrites the first (the
//! handler would then react to an arbitrary one of the two ports), so the
//! repeated key is rejected at expansion time.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct DupFilterKeyNode {
    #[input(backpressure = sample(15))]
    a: Vector3,
    #[input(backpressure = sample(15))]
    b: Vector3,
    seen: f64,
}

#[cerulion_node_impl]
impl DupFilterKeyNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen = self.a.x + self.b.x;
        Ok(())
    }

    // `input` set twice on ONE attribute — the second would silently last-win.
    #[on_event(input = "a", input = "b")]
    fn on_pressure(&mut self, _event: BackpressureEvent) {}
}

fn main() {}
