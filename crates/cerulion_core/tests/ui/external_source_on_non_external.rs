use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// The policy-mismatch contract: `external_source` is only valid
/// on an `#[cerulion_node(external)]` node. Declaring it on a non-external node
/// (here `period_ms`) is rejected at expansion time with OUR own clear
/// diagnostic — the method has no meaning without the `external` trigger policy.
#[cerulion_node(period_ms = 10)]
struct NotExternalWithSource {
    #[output]
    cmd: Vector3,
}

#[cerulion_node_impl]
impl NotExternalWithSource {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.x = 0.0;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

fn main() {}
