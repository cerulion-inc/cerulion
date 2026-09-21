#![allow(unexpected_cfgs)]
//! A `LivelinessEvent` handler (input-scoped) whose
//! `input = "..."` filter references an input NOT declared on the sibling
//! `#[cerulion_node]` struct must fail to compile — the referenced port is
//! validated against the declared `#[input]` fields.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct LivelinessUnknownPortNode {
    #[input]
    inp: Vector3,
    seen: f64,
}

#[cerulion_node_impl]
impl LivelinessUnknownPortNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen = self.inp.x;
        Ok(())
    }

    // `nope` is not a declared `#[input]` field — unknown-port error.
    #[on_event(input = "nope")]
    fn on_liveliness(&mut self, _ev: LivelinessEvent) {}
}

fn main() {}
