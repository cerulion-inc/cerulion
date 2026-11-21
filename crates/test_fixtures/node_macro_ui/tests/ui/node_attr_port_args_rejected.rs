use cerulion_core::prelude::*;

/// Ports are not macro arguments: `#[cerulion_node(inputs(a), outputs(b))]`
/// must be rejected, pointing at the field-level `#[input]` / `#[output]`
/// attributes that declare them instead.
#[cerulion_node(inputs(a), outputs(b))]
struct PortArgsNode;

fn main() {}
