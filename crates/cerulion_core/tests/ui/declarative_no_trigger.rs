use cerulion_core::prelude::*;

/// Declarative node with inputs but no trigger policy — should fail.
#[cerulion_node]
struct NoTriggerNode {
    #[input]
    scan: u32,
    #[output]
    cmd: u32,
}

fn main() {}
