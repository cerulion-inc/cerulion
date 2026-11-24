use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// Macro diagnostic (D-family / policy bounds): `sync_window_ms = 0` would
/// reject every fan-in (a zero-width pairing window can never contain two
/// arrivals), silently starving the node. The macro must reject it at
/// expansion time with the "`sync_window_ms` must be > 0" diagnostic.
#[cerulion_node(sync_window_ms = 0)]
struct SyncWindowZeroNode {
    #[input(trigger)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
}

fn main() {}
