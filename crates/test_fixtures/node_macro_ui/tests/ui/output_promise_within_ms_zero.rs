use cerulion_core::prelude::*;

/// Zero-value rejection:
/// `#[output(promise_within_ms = 0)]` must be rejected at compile time.
#[cerulion_node(period_ms = 100)]
struct ZeroOutputDeadline {
    #[output(promise_within_ms = 0)]
    out: u32,
}

fn main() {}
