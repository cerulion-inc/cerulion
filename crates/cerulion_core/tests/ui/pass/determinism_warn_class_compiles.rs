#![allow(unexpected_cfgs)]
//! Pass case: WARN-class symbols (here `std::env::var`) do NOT
//! produce a compile error — the macro half emits deny errors only and stays
//! silent for warn-class symbols (stable Rust has no proc-macro warn API). The
//! node compiles; warn detection + surfacing is the deferred core half. (No
//! opt-out attribute is set; the node is still fully deny-linted.)

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct EnvReadingNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl EnvReadingNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Warn-class: not surfaced by the macro half (deny-only), NOT a
        // compile error.
        let _ = std::env::var("HOME");
        self.image.height = 1;
        Ok(())
    }
}

fn main() {
    let _entry = EnvReadingNodeEntry::new();
}
