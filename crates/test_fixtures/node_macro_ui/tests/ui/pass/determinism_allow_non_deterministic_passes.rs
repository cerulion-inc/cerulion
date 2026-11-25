#![allow(unexpected_cfgs)]
//! Pass case: `#[cerulion_node(allow_non_deterministic)]`
//! suppresses EVERY determinism deny — a node calling `Instant::now()`
//! compiles when the blanket opt-out is set.

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;
use std::time::Instant;

#[cerulion_node(period_ms = 16, allow_non_deterministic)]
#[derive(Default)]
struct OptedOutNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl OptedOutNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Allowed by the blanket opt-out.
        let _t = Instant::now();
        self.image.height = 1;
        Ok(())
    }
}

fn main() {
    // Reference the generated entry so the expansion is exercised.
    let _entry = OptedOutNodeEntry::new();
}
