use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// Macro diagnostic (R-family / port-consistency): `promise_within_ms = N`
/// may appear at most once inside a single `#[output(...)]`. Two competing
/// deadlines on the same output are ambiguous and must be rejected with the
/// "duplicate `promise_within_ms` in #[output]" diagnostic at macro-expansion
/// time.
#[cerulion_node(period_ms = 100)]
struct DuplicatePromiseWithinNode {
    #[output(promise_within_ms = 5, promise_within_ms = 10)]
    out: Vector3,
}

fn main() {}
