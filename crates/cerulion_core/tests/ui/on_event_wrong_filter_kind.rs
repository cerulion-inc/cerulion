#![allow(unexpected_cfgs)]
//! A `BackpressureEvent` handler (input-scoped) declared
//! with an `output = "..."` filter must fail to compile — the filter scope is
//! validated against the routed event kind, so an output filter on an
//! input-scoped event is rejected with the scope-mismatch error.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct WrongFilterKindNode {
    #[input(backpressure = sample(15))]
    inp: Vector3,
    #[output]
    out: Vector3,
    seen: f64,
}

#[cerulion_node_impl]
impl WrongFilterKindNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen = self.inp.x;
        self.out.x = self.seen;
        Ok(())
    }

    // `BackpressureEvent` is input-scoped — using `output =` is a scope error.
    #[on_event(output = "out")]
    fn on_pressure(&mut self, _ev: BackpressureEvent) {}
}

fn main() {}
