use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// `consume = lifo` is not a consumption mode: the refusal names the two accepted values.
#[cerulion_node]
struct ConsumeNode {
    #[input(trigger, consume = lifo)]
    pose: Vector3,
}

fn main() {}
