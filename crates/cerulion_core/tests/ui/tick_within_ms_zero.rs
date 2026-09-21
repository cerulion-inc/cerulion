use cerulion_core::prelude::*;

/// Zero-value rejection:
/// `#[cerulion_node(tick_within_ms = 0)]` must be rejected at compile time.
#[cerulion_node(period_ms = 100, tick_within_ms = 0)]
struct ZeroTickDeadline {
    #[output]
    out: u32,
}

fn main() {}
