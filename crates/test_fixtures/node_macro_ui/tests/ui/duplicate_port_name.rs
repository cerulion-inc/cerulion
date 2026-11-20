use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 100)]
struct DuplicatePortNode {
    #[input]
    data: Vector3,
    #[output]
    data: Vector3,
}

fn main() {}
