#![allow(unexpected_cfgs)]
//! The "at most one #[on_event]" rule must fire even when an
//! EARLIER attribute on the same method is itself malformed. The duplicate
//! must be detected even though the first attr fails to parse (it never sets
//! `parsed_filter`), so a malformed-first + valid-second pair would otherwise
//! mask the structural duplicate behind the lexical error. The macro counts
//! occurrences unconditionally and accumulates diagnostics, so BOTH the
//! unknown-key error and the duplicate error surface in one pass.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct DupAttrMalformedFirstNode {
    #[input(backpressure = sample(15))]
    a: Vector3,
    seen: f64,
}

#[cerulion_node_impl]
impl DupAttrMalformedFirstNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen = self.a.x;
        Ok(())
    }

    // First attr malformed (unknown key `bogus`), second valid — the
    // duplicate must STILL be reported, not masked by the unknown-key error.
    #[on_event(bogus = 3)]
    #[on_event(input = "a")]
    fn on_pressure(&mut self, _event: BackpressureEvent) {}
}

fn main() {}
