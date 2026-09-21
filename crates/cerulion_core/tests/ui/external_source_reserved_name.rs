use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// The namespace-collision contract: the `__cer_*` method
/// namespace is reserved by `#[cerulion_node_impl]` for macro-injected shims
/// (including `__cer_user_external_source`). A user method named
/// `__cer_external_source` hits the existing prefix guard and is rejected with a
/// clear "reserved by `#[cerulion_node_impl]`" diagnostic pointing at the user
/// method — NOT a confusing "duplicate definition" cascade after expansion.
#[cerulion_node(external)]
struct ExternalReservedName {
    #[output]
    cmd: Vector3,
}

#[cerulion_node_impl]
impl ExternalReservedName {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.x = 0.0;
        Ok(())
    }

    fn __cer_external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

fn main() {}
