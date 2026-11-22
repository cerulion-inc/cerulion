use cerulion_core::prelude::*;

/// A node does not name its own type: `type_name = "..."` must be rejected,
/// pointing at the folder name (`nodes/<type>/`), which is what the graph
/// resolves against at load time.
#[cerulion_node(type_name = "camera")]
struct CameraNode {
    #[output]
    image: u32,
}

fn main() {}
