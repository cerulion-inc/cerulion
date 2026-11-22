use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// An accumulate input alone never fires a node, so a node with no `period_ms`, `external` or trigger input is refused, saying so.
#[cerulion_node]
struct AccumulateNode {
    #[input(accumulate)]
    tf: Vector3,
}

fn main() {}
