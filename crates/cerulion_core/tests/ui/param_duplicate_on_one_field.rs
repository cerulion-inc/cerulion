use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// A field is ONE param: a second `#[param]` on it would declare the same
/// name twice, which the host refuses when it reads the node's info. The
/// macro refuses it at expansion instead.
#[cerulion_node(period_ms = 100)]
struct ParamDuplicateNode {
    #[output]
    cmd: Vector3,
    #[param(default = 1.0)]
    #[param(default = 2.0)]
    gain: f64,
}

fn main() {}
