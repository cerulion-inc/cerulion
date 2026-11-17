// SPDX-License-Identifier: AGPL-3.0-only
//! Typed delivery fixture using host-owned input maintenance.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]
use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;
use std::sync::atomic::{AtomicU64, Ordering};
static OBSERVED: AtomicU64 = AtomicU64::new(0);
#[no_mangle]
pub extern "C" fn cerulion_test_observed_input() -> u64 {
    OBSERVED.load(Ordering::Relaxed)
}

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
pub struct SnapshotTrap {
    #[input]
    inp: Vector3,
}
#[cerulion_node_impl]
impl SnapshotTrap {
    fn tick(&mut self) -> Result<(), NodeError> {
        OBSERVED.store(self.inp.x as u64, Ordering::Relaxed);
        Ok(())
    }
}
