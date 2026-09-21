use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// Macro diagnostic (C-family): `#[input(backpressure = ...)]` accepts only
/// `drop_oldest`, `block`, or `sample(ms)`. An unknown policy keyword must be
/// rejected at macro-expansion time with the actionable list of valid
/// policies — not silently dropped (which would fall back to the default and
/// hide the user's typo).
#[cerulion_node(period_ms = 100)]
struct UnknownBackpressurePolicyNode {
    #[input(backpressure = drop_newest)]
    data: Vector3,
}

fn main() {}
