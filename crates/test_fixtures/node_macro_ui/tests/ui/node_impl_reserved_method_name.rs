use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// Macro diagnostic (R-family / namespace collision): the `__cer_*` method
/// namespace is reserved for macro-injected shims. A user method whose name
/// starts with `__cer_` must be rejected at expansion time with a clear
/// "reserved by `#[cerulion_node_impl]`" diagnostic pointing at the user
/// method — NOT a confusing "duplicate definition" cascade after expansion.
///
/// The struct uses a single `#[input(trigger)]` (a valid DataTrigger node) so
/// it REGISTERS cleanly and the impl macro reaches the reserved-name check.
#[cerulion_node]
#[derive(Default)]
struct ReservedMethodNode {
    #[input(trigger)]
    inp: Vector3,
    seen: f64,
}

#[cerulion_node_impl]
impl ReservedMethodNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen = self.__cer_helper();
        Ok(())
    }

    fn __cer_helper(&self) -> f64 {
        self.inp.x
    }
}

fn main() {}
