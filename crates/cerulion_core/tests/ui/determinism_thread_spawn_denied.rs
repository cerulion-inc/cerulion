#![allow(unexpected_cfgs)]
//! An unmanaged `thread::spawn` in a helper method (not just
//! `tick`) must fail to compile — the lint covers every method body.

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct SpawningNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl SpawningNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.image.height = 1;
        self.do_work();
        Ok(())
    }

    fn do_work(&self) {
        // Banned: unmanaged thread, non-reproducible interleaving.
        std::thread::spawn(|| {});
    }
}

fn main() {}
