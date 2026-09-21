#![allow(unexpected_cfgs)]
//! The unit-return diagnostic must ALSO fire on a
//! NESTED port write. `self.imu.orientation.x = 1.0` (a 3-segment fixed
//! path) now rewrites to the fallible `__cer_imu.__cer_with_nested_orientation(
//! |__cer_v| __cer_v.__cer_assign_x(…))?` chain — so a `()` helper that
//! syntactically cannot use `?` must get OUR targeted compile_error naming
//! the method + the FULL dotted field (`orientation.x`), not rustc's generic
//! E0277 pointing at macro-generated code. Pins that `assign_rewrites`
//! counts nested writes and `first_assign` carries the dotted path.

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Imu;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct NestedUnitHelperWriteNode {
    #[output]
    imu: Imu,
}

#[cerulion_node_impl]
impl NestedUnitHelperWriteNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.set_orientation();
        Ok(())
    }

    // No return arrow — can never use `?`, but the nested fixed-field write
    // below is rewritten to the fallible `__cer_with_nested_orientation(…)?`
    // chain.
    fn set_orientation(&mut self) {
        self.imu.orientation.x = 1.0;
    }
}

fn main() {}
