use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// The field type IS the param type, with no coercion: an `f64` param's
/// `default` must be a float literal. `2` is refused naming the fix (`2.0`).
#[cerulion_node(period_ms = 100)]
struct ParamDefaultWrongTypeNode {
    #[output]
    cmd: Vector3,
    #[param(default = 2)]
    gain: f64,
}

fn main() {}
