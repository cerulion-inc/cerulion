use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// `consume = latest` on the one trigger input of a `sync_window_ms` node: the node keeps the Sync policy, which takes no per-input consumption mode, so this is refused too.
#[cerulion_node(sync_window_ms = 20)]
struct ConsumeNode {
    #[input(trigger, consume = latest)]
    left: Vector3,
}

fn main() {}
