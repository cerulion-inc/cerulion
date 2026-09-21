use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// A BARE-IDENT unknown key inside `#[input(...)]` (no `= value`) must be
/// rejected the same way a `name = value` one is — the match keys on the
/// ident before any value is consumed. The diagnostic names the offending
/// ident and lists the accepted set.
#[cerulion_node]
struct UnknownBareInputAttrNode {
    #[input(trigger, newest_first)]
    data: Vector3,
}

fn main() {}
