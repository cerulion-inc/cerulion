use cerulion_core::prelude::*;

/// `unbounded_sync` without any `#[input(trigger)]` fields — sync (bounded
/// or unbounded) requires ≥2 trigger inputs to be meaningful.
#[cerulion_node(unbounded_sync)]
struct UnboundedSyncNoTriggers {
    #[input]
    a: u32,
    #[input]
    b: u32,
}

fn main() {}
