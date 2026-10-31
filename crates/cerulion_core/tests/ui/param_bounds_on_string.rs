use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// `min`/`max` apply to integer and float params only; on a `String` param
/// they are refused naming the type.
#[cerulion_node(period_ms = 100)]
struct ParamBoundsOnStringNode {
    #[output]
    cmd: Vector3,
    #[param(default = "base_link", min = 0)]
    frame_id: String,
}

fn main() {}
