#![allow(unexpected_cfgs)]
//! Pass case: `#[cerulion_node(uses_live_io)]` records the live-IO
//! declaration (the suppression flag the deferred CLI half pairs with replay)
//! — a node calling `fs::read_dir` (the IO-class warn) compiles. (A non-IO
//! time/thread DENY would still fire even with this opt-out; covered by the
//! `determinism_uses_live_io_does_not_suppress_deny` fixture. `read_dir` also
//! compiles WITHOUT this opt-out — it is a warn, not a deny, and the macro half
//! is deny-only — see `determinism_fs_read_dir_warns.rs`. `uses_live_io` is the
//! declaration the deferred core half uses to silence the warn at graph load.)

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 16, uses_live_io)]
#[derive(Default)]
struct LiveIoNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl LiveIoNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Allowed by the IO-class opt-out.
        let _ = std::fs::read_dir(".");
        self.image.height = 1;
        Ok(())
    }
}

fn main() {
    let _entry = LiveIoNodeEntry::new();
}
