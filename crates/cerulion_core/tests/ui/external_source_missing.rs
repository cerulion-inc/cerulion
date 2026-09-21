use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// The required-method contract: an `#[cerulion_node(external)]`
/// node MUST define an `external_source` method on its `#[cerulion_node_impl]`
/// block. Omitting it is rejected at expansion time with OUR own clear
/// diagnostic (naming the node type + the exact fix) — not a cryptic
/// method-not-found cascade. The impl macro injects a `__cer_user_external_source`
/// stub in this error case so the codegen wrapper's override call still resolves
/// and the user sees ONLY the actionable message.
#[cerulion_node(external)]
struct ExternalMissingSource {
    #[output]
    cmd: Vector3,
}

#[cerulion_node_impl]
impl ExternalMissingSource {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.x = 0.0;
        Ok(())
    }
}

fn main() {}
