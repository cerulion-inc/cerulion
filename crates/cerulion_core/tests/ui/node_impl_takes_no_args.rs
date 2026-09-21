use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// Macro diagnostic (R-family / impl-block contract): `#[cerulion_node_impl]`
/// takes NO arguments — ports are auto-discovered from the sibling
/// `#[cerulion_node]` struct. A legacy `inputs(...)`/`outputs(...)`-style arg
/// list on the impl macro must be rejected at expansion time with the "takes no
/// arguments" diagnostic.
///
/// The struct uses a single `#[input(trigger)]` (a valid DataTrigger node, no
/// conflicting node-level policy) so the struct REGISTERS cleanly and the impl
/// macro reaches the arg-rejection check rather than failing the lookup first.
#[cerulion_node]
#[derive(Default)]
struct ImplArgsNode {
    #[input(trigger)]
    inp: Vector3,
    seen: f64,
}

#[cerulion_node_impl(inputs(inp))]
impl ImplArgsNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen = self.inp.x;
        Ok(())
    }
}

fn main() {}
