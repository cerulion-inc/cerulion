use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// An unrecognized key inside `#[input(...)]` must be rejected — never
/// silently ignored — naming the offending ident and listing the accepted
/// set (`trigger`, `depth`, `backpressure`, `expect_within_ms`).
#[cerulion_node(period_ms = 100)]
struct UnknownInputAttrNode {
    #[input(bogus_key = 5)]
    data: Vector3,
}

fn main() {}
