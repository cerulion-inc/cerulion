use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// The signature contract: `external_source` must be declared
/// exactly `fn external_source(&mut self) -> ExternalSource`. A `&self` receiver
/// (the most likely user mistake — the method may open a device, so it needs
/// `&mut self`) is rejected at expansion time with OUR own clear diagnostic.
#[cerulion_node(external)]
struct ExternalWrongSig {
    #[output]
    cmd: Vector3,
}

#[cerulion_node_impl]
impl ExternalWrongSig {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.x = 0.0;
        Ok(())
    }

    fn external_source(&self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

fn main() {}
