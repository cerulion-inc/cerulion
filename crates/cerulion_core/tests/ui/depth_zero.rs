use cerulion_core::prelude::*;

/// Macro diagnostic (C-family / port bounds): `#[input(depth = 0)]` is a
/// queue of length zero — it could never hold a message. The macro must reject
/// it at expansion time with the "depth must be >= 1" diagnostic, mirroring
/// the runtime gate in `GraphTopology::validate`. (Lower-bound companion of
/// the existing `depth_above_max` upper-bound case.)
#[cerulion_node(period_ms = 100)]
struct DepthZeroNode {
    #[input(depth = 0)]
    image: u32,
}

fn main() {}
