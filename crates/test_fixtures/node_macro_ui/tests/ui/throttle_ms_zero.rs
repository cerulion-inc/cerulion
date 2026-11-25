use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// Macro diagnostic (D-family / policy bounds): `throttle_ms = 0` (a
/// zero-ms producer rate cap) would defer every fire forever — `now -
/// last_fire < 0` is never true, but a zero cap is a degenerate no-op the user
/// almost certainly did not intend. The macro must reject it at expansion time
/// with the "`throttle_ms` must be > 0" diagnostic.
#[cerulion_node(throttle_ms = 0)]
struct ThrottleZeroNode {
    #[input(trigger)]
    inp: Vector3,
    #[output]
    out: u32,
}

fn main() {}
