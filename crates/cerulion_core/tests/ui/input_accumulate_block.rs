use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// `#[input(accumulate, backpressure = block)]`: `block` paces the producer instead of dropping at the cap, so the combination is refused, naming the accepted set.
#[cerulion_node(period_ms = 100)]
struct AccumulateNode {
    #[input(accumulate, backpressure = block)]
    tf: Vector3,
}

fn main() {}
