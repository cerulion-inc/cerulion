#![allow(unexpected_cfgs)]
//! `SystemTime::now` reads the live system clock and is a DENY —
//! it must fail to compile (the second of the time-killer deny rows, proving
//! the visitor's path-tail match covers `SystemTime::now` end-to-end, not just
//! `Instant::now`).

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;
use std::time::SystemTime;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct SystemTimeNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl SystemTimeNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Banned: live system clock read.
        let _t = SystemTime::now();
        self.image.height = 1;
        Ok(())
    }
}

fn main() {}
