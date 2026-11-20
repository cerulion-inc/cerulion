use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// Every param is declared, delivered and applied in every build, so a
/// param field the compiler may remove is refused at its `#[cfg]`.
#[cerulion_node(period_ms = 100)]
struct ParamCfgFieldNode {
    #[output]
    cmd: Vector3,
    #[cfg(any())]
    #[param(default = 1.0)]
    gain: f64,
}

fn main() {}
