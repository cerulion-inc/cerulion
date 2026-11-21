use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// The `#[input(...)]` attribute is a
/// LIST form, not a name-value form. Writing `#[input = "..."]` must be
/// rejected at macro-expansion time with the "expected `#[input]` or
/// `#[input(...)]`" diagnostic — not silently ignored.
#[cerulion_node(period_ms = 100)]
struct InputNameValueNode {
    #[input = "trigger"]
    data: Vector3,
}

fn main() {}
