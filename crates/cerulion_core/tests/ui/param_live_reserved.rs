use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// `#[param(live)]` is reserved for the live tier (runtime set, persisted
/// restore, param events) and is refused with a "not yet supported" message
/// rather than parsed and ignored.
#[cerulion_node(period_ms = 100)]
struct ParamLiveReservedNode {
    #[output]
    cmd: Vector3,
    #[param(default = 1.0, live)]
    gain: f64,
}

fn main() {}
