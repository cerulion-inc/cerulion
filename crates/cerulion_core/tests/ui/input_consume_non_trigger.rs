use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// `consume` on a non-trigger input: a latest-value read never fires the node, so it has no consumption order, and the refusal says so.
#[cerulion_node(period_ms = 100)]
struct ConsumeNode {
    #[input(consume = latest)]
    pose: Vector3,
}

fn main() {}
