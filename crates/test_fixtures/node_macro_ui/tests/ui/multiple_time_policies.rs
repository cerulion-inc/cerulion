use cerulion_core::prelude::*;

/// Macro diagnostic (D-family / trigger inference): a node may carry at most
/// ONE node-level time policy. Specifying both `period_ms` and `external`
/// (with zero trigger inputs) is contradictory — is the node clock-driven or
/// host-driven? The macro must reject it at expansion time with the "only one
/// of `period_ms`, `sync_window_ms`, `unbounded_sync`, or `external`"
/// diagnostic.
#[cerulion_node(period_ms = 100, external)]
struct MultipleTimePoliciesNode {
    #[output]
    out: u32,
}

fn main() {}
