use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// The `#[output(...)]` attribute is a
/// LIST form, not a name-value form. Writing `#[output = "..."]` must be
/// rejected at macro-expansion time with the "expected `#[output]` or
/// `#[output(...)]`" diagnostic — not silently ignored.
#[cerulion_node(period_ms = 100)]
struct OutputNameValueNode {
    #[output = "data"]
    out: Vector3,
}

fn main() {}
