#![allow(unexpected_cfgs)]
//! Pass case: the determinism lint matches FREE/ASSOCIATED path
//! calls (`Instant::now()`), NOT method calls on a value receiver. A user
//! struct field whose `.now()` method is invoked as `self.timer.now()` is a
//! `MethodCall` on a value, not the banned `Instant::now` path call — it must
//! COMPILE (no false positive). This pins that the visitor never consults the
//! path-tail table for `Expr::MethodCall` receivers.

use cerulion_core::prelude::*;
use cerulion_core::state::CerulionState;
use native_ros2_messages::sensor_msgs::Image;

/// A user type with a `.now()` method that has nothing to do with the live
/// clock — calling it must NOT trip the determinism deny lint.
///
/// `CerulionState` is derived because the state impl was folded
/// into `#[cerulion_node]`: the node below holds a `UserTimer` as an ordinary
/// non-port field, and its `counter` GATES the value the node publishes, so it
/// is state and must be capturable. This is the diagnostic's own third remedy
/// ("if this is real state, make it capturable"), taken deliberately over
/// `#[cerulion(reconstruct)]` — reconstructing would zero a counter the tick
/// reads back.
#[derive(Default, CerulionState)]
struct UserTimer {
    counter: u64,
}

impl UserTimer {
    /// Deterministic, user-owned `.now()` — returns a monotonically increasing
    /// counter, not a wall-clock read.
    fn now(&mut self) -> u64 {
        self.counter += 1;
        self.counter
    }
}

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct MethodReceiverNode {
    #[output]
    image: Image,
    timer: UserTimer,
}

#[cerulion_node_impl]
impl MethodReceiverNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // `self.timer.now()` is a method call on a user value, NOT the banned
        // `Instant::now()` path call — this compiles cleanly.
        let n = self.timer.now();
        self.image.height = n as u32;
        Ok(())
    }
}

fn main() {
    let _entry = MethodReceiverNodeEntry::new();
}
