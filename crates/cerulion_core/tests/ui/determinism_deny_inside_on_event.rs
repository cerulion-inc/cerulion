#![allow(unexpected_cfgs)]
//! The determinism lint covers EVERY method body, including
//! `#[on_event(...)]` handlers (they run user code too). A DENY symbol
//! (`Instant::now()`) inside an `#[on_event]` handler body must fail to
//! compile — the lint walks handler bodies before the `#[on_event]` rewrite.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct OnEventTimeNode {
    #[input(backpressure = sample(15))]
    inp: Vector3,
    seen: f64,
}

#[cerulion_node_impl]
impl OnEventTimeNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen = self.inp.x;
        Ok(())
    }

    #[on_event(input = "inp")]
    fn on_pressure(&mut self, _ev: BackpressureEvent) {
        // Banned: reads the live clock inside an event handler body.
        let _t = std::time::Instant::now();
    }
}

fn main() {}
