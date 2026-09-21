#![allow(unexpected_cfgs)]
//! `#[on_event]` with no `input`/`output` filter must be
//! rejected with a clear compile error directing the user at the required form
//! (exactly one of `input = "..."` / `output = "..."`).

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct NoInputArgNode {
    #[input(backpressure = sample(15))]
    inp: Vector3,
    seen: f64,
}

#[cerulion_node_impl]
impl NoInputArgNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen = self.inp.x;
        Ok(())
    }

    // Missing the required `input = "..."` / `output = "..."` filter.
    #[on_event()]
    fn on_pressure(&mut self, _event: BackpressureEvent) {}
}

fn main() {}
