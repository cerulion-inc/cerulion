// SPDX-License-Identifier: AGPL-3.0-only
//! Typed aligned-set delivery fixture using host-owned sync maintenance.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]
use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;
use std::sync::atomic::{AtomicU64, Ordering};
static FIRES: AtomicU64 = AtomicU64::new(0);
static A: AtomicU64 = AtomicU64::new(0);
static B: AtomicU64 = AtomicU64::new(0);
#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
pub struct SyncTrap {
    #[input(trigger)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
}
#[cerulion_node_impl]
impl SyncTrap {
    fn tick(&mut self) -> Result<(), NodeError> {
        A.store(self.a.x.to_bits(), Ordering::Relaxed);
        B.store(self.b.x.to_bits(), Ordering::Relaxed);
        FIRES.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}
/// # Safety
/// Each pointer is aligned exclusive writable scalar storage for this call.
#[no_mangle]
pub unsafe extern "C" fn sync_probe(fires: *mut u64, a: *mut u64, b: *mut u64) {
    // SAFETY: test callers lend three distinct live scalar outputs synchronously.
    unsafe {
        fires.write(FIRES.load(Ordering::Relaxed));
        a.write(A.load(Ordering::Relaxed));
        b.write(B.load(Ordering::Relaxed));
    }
}
#[no_mangle]
pub extern "C" fn sync_probe_reset() {
    for row in [&FIRES, &A, &B] {
        row.store(0, Ordering::Relaxed);
    }
}
