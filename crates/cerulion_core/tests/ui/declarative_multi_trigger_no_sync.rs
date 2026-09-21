use cerulion_core::prelude::*;

/// Multiple trigger inputs without sync_window_ms — should fail.
#[cerulion_node]
struct MultiTriggerNode {
    #[input(trigger)]
    imu: u32,
    #[input(trigger)]
    camera: u32,
    #[output]
    pose: u32,
}

fn main() {}
