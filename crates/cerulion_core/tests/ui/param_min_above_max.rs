use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// `min` above `max` is an empty range no value could satisfy; refused at
/// expansion, naming both bounds.
#[cerulion_node(period_ms = 100)]
struct ParamMinAboveMaxNode {
    #[output]
    cmd: Vector3,
    #[param(default = 1.0, min = 5.0, max = 0.0)]
    gain: f64,
}

fn main() {}
