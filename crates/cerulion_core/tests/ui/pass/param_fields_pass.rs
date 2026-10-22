#![allow(unexpected_cfgs)]
//! Pass case: `#[param]` on plain fields of every supported shape (a bounded
//! float, a bounded signed integer, an unsigned integer, a bool, a `String`)
//! compiles beside a port, and the wrapper constructed with `new()` already
//! holds every declared default, so a node run with no `params:` reads its
//! defaults from the first tick.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct ParamNode {
    #[output]
    cmd: Vector3,
    #[param(default = 2.0, min = 0.0, max = 5.0, description = "loop gain")]
    gain: f64,
    #[param(default = -3, min = -10, max = 10)]
    offset: i32,
    #[param(default = 7)]
    level: u8,
    #[param(default = true)]
    enabled: bool,
    #[param(default = "base_link", description = "frame the output is expressed in")]
    frame_id: String,
}

#[cerulion_node_impl]
impl ParamNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.x = self.gain;
        self.cmd.y = f64::from(self.offset);
        self.cmd.z = if self.enabled { f64::from(self.level) } else { -1.0 };
        Ok(())
    }
}

fn main() {
    let entry = ParamNodeEntry::new();
    assert_eq!(entry.inner.gain, 2.0);
    assert_eq!(entry.inner.offset, -3);
    assert_eq!(entry.inner.level, 7);
    assert!(entry.inner.enabled);
    assert_eq!(entry.inner.frame_id, "base_link");
}
