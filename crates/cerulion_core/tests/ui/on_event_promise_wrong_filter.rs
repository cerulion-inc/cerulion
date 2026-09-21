#![allow(unexpected_cfgs)]
//! A `PromiseWithinEvent` handler (output-scoped) declared
//! with an `input = "..."` filter must fail to compile — the filter scope is
//! validated against the routed event kind, so an input filter on an
//! output-scoped event is rejected with the scope-mismatch error.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct PromiseWrongFilterNode {
    #[input]
    inp: Vector3,
    #[output(promise_within_ms = 30)]
    out: Vector3,
    seen: f64,
}

#[cerulion_node_impl]
impl PromiseWrongFilterNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen = self.inp.x;
        self.out.x = self.seen;
        Ok(())
    }

    // `PromiseWithinEvent` is output-scoped — using `input =` is a scope error.
    #[on_event(input = "inp")]
    fn on_late(&mut self, _ev: PromiseWithinEvent) {}
}

fn main() {}
