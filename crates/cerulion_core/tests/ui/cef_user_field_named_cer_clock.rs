//! A user field literally named `__cer_rt` collides
//! with the macro-injected hidden runtime-context field. The macro emits
//! both, so rustc sees a duplicate-field-name error. Snapshot the
//! diagnostic so the collision behaviour is pinned (and so we notice if
//! it ever stops being caught at all).
//!
//! Pre-architecture-refactor this test probed the old `__cer_clock`
//! field name (when hidden state was two fields). After the refactor to
//! a single bundled `__cer_rt: CerNodeRuntimeFields` field, the
//! collision target moved — same idea, single point of contact.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 100)]
struct CollidingHiddenField {
    #[output]
    out: Vector3,
    /// Same name as the macro-injected hidden runtime field. Should fail.
    __cer_rt: u32,
}

#[cerulion_node_impl]
impl CollidingHiddenField {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 0.0;
        Ok(())
    }
}

fn main() {}
