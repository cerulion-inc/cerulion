#![allow(unexpected_cfgs)]
//! The visitor descends into closures, so a DENY symbol inside a
//! closure body (`|| Instant::now()`) must still fail to compile. (The default
//! `syn::visit::Visit` walk recurses through `Expr::Closure` bodies — this
//! pins that the determinism lint does too.)

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct ClosureTimeNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl ClosureTimeNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Banned: `Instant::now()` inside a closure body.
        let _f = || std::time::Instant::now();
        self.image.height = 1;
        Ok(())
    }
}

fn main() {}
