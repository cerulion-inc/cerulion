//! J / A user-written method named
//! `real_ns` on the same struct collides with the macro-injected clock
//! shim (`now_ns` / `real_ns` / `virt_ns` / `ext_ns`). Since both end up
//! as inherent impls, rustc rejects the duplicate. Snapshot the
//! diagnostic so the collision behaviour is pinned.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 100)]
struct UserWallNs {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl UserWallNs {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 0.0;
        Ok(())
    }
}

// Second inherent impl block providing a `real_ns` method that collides
// with the macro-injected shim. Both are `pub fn real_ns(&self) -> u64`.
impl UserWallNs {
    pub fn real_ns(&self) -> u64 {
        0
    }
}

fn main() {}
