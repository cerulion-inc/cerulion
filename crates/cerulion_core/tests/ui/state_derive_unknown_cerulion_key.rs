// A typo'd `#[cerulion(...)]` key is REFUSED.
//
// The no-silent-inference rule applied literally: if `#[cerulion(recontsruct)]` silently
// meant "capture it", the node would restore a stale handle and nothing would
// say so. This is OUR `compile_error!`, so its text is stable across
// toolchains and the fixture belongs in the BLOCKING group.

use cerulion_core::state::CerulionState;

struct Handle;

#[derive(CerulionState)]
struct Node {
    #[cerulion(recontsruct)]
    handle: Handle,
}

fn main() {}
