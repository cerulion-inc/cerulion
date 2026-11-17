use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// `consume = latest` on a Sync node's trigger input: a Sync node fires once per aligned set and takes no per-input consumption mode.
#[cerulion_node(sync_window_ms = 20)]
struct ConsumeNode {
    #[input(trigger, consume = latest)]
    left: Vector3,
    #[input(trigger)]
    right: Vector3,
}

fn main() {}
