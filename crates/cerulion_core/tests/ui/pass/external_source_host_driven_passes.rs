use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// The happy path: an `#[cerulion_node(external)]` node WITH a
/// proper `external_source` returning `ExternalSource::HostDriven` compiles
/// cleanly — the required-method contract accepts the canonical migration form.
#[cerulion_node(external)]
struct ExternalHostDriven {
    #[output]
    cmd: Vector3,
}

#[cerulion_node_impl]
impl ExternalHostDriven {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.x = 0.0;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

fn main() {
    // Prove the generated wrapper surfaces the source through NodeEntry.
    use cerulion_core::graph::node::{ExternalSource, NodeEntry};
    let mut node = ExternalHostDrivenEntry::new();
    assert!(matches!(
        node.external_source(),
        Some(ExternalSource::HostDriven)
    ));
}
