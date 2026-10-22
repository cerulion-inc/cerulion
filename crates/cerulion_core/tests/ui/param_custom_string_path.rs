use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

mod custom {
    #[derive(Default)]
    pub struct String;
}

/// A path that only ENDS in a supported name is another type: refused with
/// the accepted list, not as a trait-bound error in the generated code.
#[cerulion_node(period_ms = 100)]
struct ParamCustomStringNode {
    #[output]
    cmd: Vector3,
    #[param(default = "base_link")]
    frame_id: custom::String,
}

fn main() {}
