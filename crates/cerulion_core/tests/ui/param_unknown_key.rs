use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// An unrecognized key inside `#[param(...)]` must be rejected, never silently
/// ignored, naming the offending ident and listing the accepted set
/// (`default`, `min`, `max`, `description`).
#[cerulion_node(period_ms = 100)]
struct UnknownParamKeyNode {
    #[output]
    cmd: Vector3,
    #[param(default = 1.0, step = 0.1)]
    gain: f64,
}

fn main() {}
