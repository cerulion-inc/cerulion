use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// A default outside its own `min`/`max` would make a graph that sets nothing
/// fail its own range check; refused at expansion instead.
#[cerulion_node(period_ms = 100)]
struct ParamDefaultOutsideBoundsNode {
    #[output]
    cmd: Vector3,
    #[param(default = 9.0, min = 0.0, max = 5.0)]
    gain: f64,
}

fn main() {}
