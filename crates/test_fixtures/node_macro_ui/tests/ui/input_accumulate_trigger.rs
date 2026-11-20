use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// `#[input(accumulate, trigger)]`: a trigger input already serves every frame, so an accumulate trigger is refused, naming the accepted set.
#[cerulion_node(period_ms = 100)]
struct AccumulateNode {
    #[input(accumulate, trigger)]
    tf: Vector3,
}

fn main() {}
