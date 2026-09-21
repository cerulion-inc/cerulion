use cerulion_core::prelude::*;

/// Every `#[cerulion_node]` must declare at least
/// one `#[input]` or `#[output]` field attribute.
#[cerulion_node]
struct NoPortsNode {
    counter: u32,
}

fn main() {}
