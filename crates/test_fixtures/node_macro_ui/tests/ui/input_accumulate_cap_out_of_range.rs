use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// `#[input(accumulate, cap = 65)]`: a cap above the queue depth ceiling (64) is refused, naming the range and the accepted set.
#[cerulion_node(period_ms = 100)]
struct AccumulateNode {
    #[input(accumulate, cap = 65)]
    tf: Vector3,
}

fn main() {}
