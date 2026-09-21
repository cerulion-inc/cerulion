#![allow(unexpected_cfgs)]
//! Pass case: `fs::read_dir` is a WARN (IO-class), NOT a deny.
//! Calling it in a tick body WITHOUT any opt-out attribute must COMPILE — the
//! macro half emits deny errors only and stays silent for warn-class symbols
//! (stable Rust has no proc-macro warn API; warn detection + surfacing is the
//! deferred core half, which re-walks node source at the CLI).
//!
//! Live-filesystem-IO determinism is owned by the `uses_live_io` declaration
//! + replay verification, not by a syntactic fs blocklist, so `read_dir` is a
//! nudge to declare `uses_live_io`, not a hard compile error. (The deny set
//! is the four unambiguous time/thread killers — see the deny fixtures.)

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct DirReadingNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl DirReadingNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Warn-class (IO): not surfaced by the macro half (deny-only), so NOT
        // a compile error, even with no opt-out attribute set.
        let _ = std::fs::read_dir(".");
        self.image.height = 1;
        Ok(())
    }
}

fn main() {
    let _entry = DirReadingNodeEntry::new();
}
