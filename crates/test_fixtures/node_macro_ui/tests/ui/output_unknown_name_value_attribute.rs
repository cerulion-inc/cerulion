use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// Inside `#[output(...)]` the only
/// `name = value` item is `promise_within_ms = N`. Any other name-value key
/// must be rejected with the "expected `promise_within_ms = N`" diagnostic at
/// macro-expansion time — not silently ignored.
#[cerulion_node(period_ms = 100)]
struct UnknownOutputNameValueNode {
    #[output(expect_within_ms = 5)]
    out: Vector3,
}

fn main() {}
