#![allow(unexpected_cfgs)]
//! An `#[on_event]` handler whose event parameter type is
//! NOT one of the three reactable events (`BackpressureEvent`,
//! `ExpectWithinEvent`, `PromiseWithinEvent`) must fail to compile — the macro
//! routes the event kind from the param TYPE, so an unrecognized type is
//! rejected at expansion time naming the three valid types.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

struct SomeBogusType;

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct UnknownEventTypeNode {
    #[input(backpressure = sample(15))]
    inp: Vector3,
    seen: f64,
}

#[cerulion_node_impl]
impl UnknownEventTypeNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen = self.inp.x;
        Ok(())
    }

    // Unrecognized event type — only the three reactable events route.
    #[on_event(input = "inp")]
    fn on_pressure(&mut self, _ev: SomeBogusType) {}
}

fn main() {}
