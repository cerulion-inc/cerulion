use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// Macro diagnostic (R-family / impl-block contract): every
/// `#[cerulion_node_impl]` block must define a `tick` method — that is the
/// node's per-fire entry point. An impl block with no `tick` must be rejected
/// at expansion time with the "requires a `tick` method" diagnostic.
///
/// The struct uses a single `#[input(trigger)]` (a valid DataTrigger node) so
/// it REGISTERS cleanly and the impl macro reaches the missing-tick check.
#[cerulion_node]
#[derive(Default)]
struct MissingTickNode {
    #[input(trigger)]
    inp: Vector3,
    seen: f64,
}

#[cerulion_node_impl]
impl MissingTickNode {
    // No `tick` method — only a helper.
    fn helper(&self) -> f64 {
        self.inp.x
    }
}

fn main() {}
