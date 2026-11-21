#![allow(unexpected_cfgs)]
//! The CROSS-METHOD `(port, event-kind)` collision:
//! two SEPARATE methods both `#[on_event(input = "a")]` with the SAME event
//! type (`BackpressureEvent`) — must fail to compile. The dispatch drains a
//! single pending event per `(port, kind)` accessor per tick, so a second
//! handler for the same pair could never fire; it is rejected at expansion.
//!
//! (The same-method duplicate-ATTRIBUTE case is pinned separately by
//! `on_event_duplicate_on_method.rs`; this case is the distinct cross-method
//! dedup-key path.)

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct DupPortKindNode {
    #[input(backpressure = sample(15))]
    a: Vector3,
    seen: f64,
}

#[cerulion_node_impl]
impl DupPortKindNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen = self.a.x;
        Ok(())
    }

    // Two handlers, SAME (port = "a", kind = Backpressure) — duplicate.
    #[on_event(input = "a")]
    fn on_pressure_one(&mut self, _event: BackpressureEvent) {}

    #[on_event(input = "a")]
    fn on_pressure_two(&mut self, _event: BackpressureEvent) {}
}

fn main() {}
