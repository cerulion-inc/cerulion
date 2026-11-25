#![allow(unexpected_cfgs)]
//! `#[cerulion_node(uses_live_io)]` suppresses ONLY IO-class rows
//! (today: the `fs::read_dir` warn). It must NOT suppress a non-IO DENY —
//! `Instant::now()` still fails to compile even with `uses_live_io` set. Only
//! the blanket `allow_non_deterministic` silences a deny.

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;
use std::time::Instant;

#[cerulion_node(period_ms = 16, uses_live_io)]
#[derive(Default)]
struct LiveIoButStillTimeNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl LiveIoButStillTimeNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // `uses_live_io` does not cover time/thread denies — this still fails.
        let _t = Instant::now();
        self.image.height = 1;
        Ok(())
    }
}

fn main() {}
