#![allow(unexpected_cfgs)]
//! The macro-argument guard catches a DEEP (3-segment) port path too.
//! `self.imu.orientation.x` inside a foreign macro's tokens is just as
//! broken as a 2-segment read — the rewriter never descends into macro
//! token streams, so neither the leaf rewrite nor the nested
//! `__cer_with_nested_*` rewrite fires and the access would reach rustc as
//! a field read on the zero-sized port MARKER. The finder
//! (`PortFieldFinder`) descends recursively and matches at the innermost
//! 2-segment prefix (`self.imu.orientation`), so the hoist-it
//! compile_error fires. This pins that deep paths are caught even though
//! the finder's structural matcher is still the single-segment shape.

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Imu;

#[cerulion_node]
#[derive(Default)]
struct NestedInputReadInMacroNode {
    #[input(trigger)]
    imu: Imu,
}

#[cerulion_node_impl]
impl NestedInputReadInMacroNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Deep read inside a foreign macro: guarded with the hoist-it
        // compile_error (naming the innermost port prefix), NOT an E0609 on
        // the Imu marker.
        cerulion_core::tracing::debug!(qx = self.imu.orientation.x);
        Ok(())
    }
}

fn main() {}
