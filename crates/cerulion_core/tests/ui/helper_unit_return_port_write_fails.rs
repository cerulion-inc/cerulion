#![allow(unexpected_cfgs)]
//! A helper method with NO return type writes a port field. Every
//! rewritten assignment expands to a fallible `__cer_assign_<field>(…)?`
//! call, and a `()` method can syntactically never use `?` — so without
//! the targeted diagnostic the user would get rustc's generic E0277 ("the
//! `?` operator can only be used in a method that returns `Result`…
//! consider `Box<dyn Error>`") pointing at macro-generated code. The
//! rewriter must instead emit OUR compile_error naming the method and the
//! offending port field, prescribing a `Result` return.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct UnitHelperWriteNode {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl UnitHelperWriteNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.set_speed();
        Ok(())
    }

    // No return arrow — can never use `?`, but the fixed-field write below
    // is rewritten to the fallible `__cer_assign_x(…)?` shim call.
    fn set_speed(&mut self) {
        self.out.x = 1.0;
    }
}

fn main() {}
