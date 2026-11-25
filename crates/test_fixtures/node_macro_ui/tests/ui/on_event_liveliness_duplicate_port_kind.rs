#![allow(unexpected_cfgs)]
//! The CROSS-METHOD `(port, event-kind)` collision for the
//! NEW `LivelinessEvent` kind — two SEPARATE methods both
//! `#[on_event(input = "a")]` with the SAME event type (`LivelinessEvent`) —
//! must fail to compile. The dispatch drains a single pending event per
//! `(port, kind)` accessor per tick, so a second handler for the same pair
//! could never fire; it is rejected at expansion.
//!
//! Mirrors `on_event_duplicate_port_kind.rs` (the `BackpressureEvent` twin):
//! this case proves the dedup key covers the `Liveliness` kind too.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct DupLivelinessPortKindNode {
    #[input]
    a: Vector3,
    seen: f64,
}

#[cerulion_node_impl]
impl DupLivelinessPortKindNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen = self.a.x;
        Ok(())
    }

    // Two handlers, SAME (port = "a", kind = Liveliness) — duplicate.
    #[on_event(input = "a")]
    fn on_live_one(&mut self, _event: LivelinessEvent) {}

    #[on_event(input = "a")]
    fn on_live_two(&mut self, _event: LivelinessEvent) {}
}

fn main() {}
