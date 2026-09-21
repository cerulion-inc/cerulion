#![allow(unexpected_cfgs)]
//! An `#[on_event(input = "...")]` handler with the wrong
//! signature (missing the event parameter) must fail to compile — the macro
//! routes the event kind from the param TYPE, so a zero-event-param handler is
//! rejected at expansion time with the "exactly one event parameter" error.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct WrongSigNode {
    #[input(backpressure = sample(15))]
    inp: Vector3,
    seen: f64,
}

#[cerulion_node_impl]
impl WrongSigNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen = self.inp.x;
        Ok(())
    }

    // Wrong signature: the handler must take exactly one event parameter
    // after `&mut self`, e.g. `event: BackpressureEvent`.
    #[on_event(input = "inp")]
    fn on_pressure(&mut self) {}
}

fn main() {}
