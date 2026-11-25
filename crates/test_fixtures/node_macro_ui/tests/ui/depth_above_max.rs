use cerulion_core::prelude::*;

/// `#[input(depth = N)]` must respect
/// `MAX_CONSUMER_DEPTH` (64) at compile time. Every unit of depth
/// commits a full max_slice_len-sized SHM slot on the topic's
/// publisher pool, so an unbounded depth lets one input pin gigabytes
/// of shared memory. Mirrors the runtime gate in
/// `cerulion_core::graph::topology::GraphTopology::validate`.
#[cerulion_node(period_ms = 100)]
struct DepthAboveMax {
    #[input(depth = 65)]
    image: u32,
}

fn main() {}
