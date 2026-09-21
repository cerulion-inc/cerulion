use cerulion_core::prelude::*;

/// Trigger input + period_ms — conflicting policies, should fail.
#[cerulion_node(period_ms = 100)]
struct ConflictNode {
    #[input(trigger)]
    scan: u32,
    #[output]
    cmd: u32,
}

fn main() {}
