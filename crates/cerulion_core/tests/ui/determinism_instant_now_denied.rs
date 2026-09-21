#![allow(unexpected_cfgs)]
//! Calling `Instant::now()` in a tick body must fail to compile with an
//! actionable message pointing at `self.now_ns()` / `self.real_ns()`.

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;
use std::time::Instant;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct TimeReadingNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl TimeReadingNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Banned: reads the live clock, voids replay determinism.
        let _t = Instant::now();
        self.image.height = 1;
        Ok(())
    }
}

fn main() {}
