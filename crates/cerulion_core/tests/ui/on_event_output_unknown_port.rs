#![allow(unexpected_cfgs)]
//! Macro diagnostic (R-family / port consistency): an
//! `#[on_event(output = "...")]` handler that references a port NOT declared as
//! an `#[output]` on the sibling `#[cerulion_node]` struct must be rejected at
//! macro-expansion time with the "references an unknown output" diagnostic.
//! (Output-port twin of the existing input-port `on_event_liveliness_unknown_port`
//! case — the input arm and output arm are separate code branches.)

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct OnEventUnknownOutputNode {
    #[output(promise_within_ms = 5)]
    out: Vector3,
    seen: f64,
}

#[cerulion_node_impl]
impl OnEventUnknownOutputNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen += 1.0;
        Ok(())
    }

    // `nope` is not a declared `#[output]` on the struct above.
    #[on_event(output = "nope")]
    fn on_promise(&mut self, _event: PromiseWithinEvent) {}
}

fn main() {}
