#![allow(unexpected_cfgs)]
//! A single `#[on_event(...)]` attribute carrying
//! BOTH `input = "..."` AND `output = "..."` must fail to compile — the
//! attribute accepts exactly one filter, not both. Pins the `(Some, Some)`
//! "not both" branch of the filter parser.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct BothFiltersNode {
    #[input(backpressure = sample(15))]
    x: Vector3,
    #[output]
    y: Vector3,
    seen: f64,
}

#[cerulion_node_impl]
impl BothFiltersNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen = self.x.x;
        self.y.x = self.seen;
        Ok(())
    }

    // Both `input` and `output` on one attribute — exactly one is allowed.
    #[on_event(input = "x", output = "y")]
    fn on_evt(&mut self, _event: BackpressureEvent) {}
}

fn main() {}
