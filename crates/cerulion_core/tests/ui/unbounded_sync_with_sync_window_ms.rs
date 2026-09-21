use cerulion_core::prelude::*;

/// `unbounded_sync` and `sync_window_ms` are mutually exclusive — both
/// imply Sync semantics but disagree on whether there's a timing
/// window. Picking both is incoherent; validator rejects.
#[cerulion_node(sync_window_ms = 50, unbounded_sync)]
struct ConflictingSyncNode {
    #[input(trigger)]
    a: u32,
    #[input(trigger)]
    b: u32,
}

fn main() {}
