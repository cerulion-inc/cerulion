// SPDX-License-Identifier: AGPL-3.0-only
//! Det-clock-accessors fixture: a cdylib node exercising
//! `self.now_ns()` / `self.virt_ns()` / `self.ext_ns()` — the macro shim
//! dispatching through the `Arc<dyn Clock>` cloned from the host
//! `NodeContext` (a TRAIT-OBJECT VTABLE POINTER crossing the cdylib FFI
//! boundary — the `Arc<dyn Trait>`-across-cdylib hazard class documented in
//! `docs/internals/core-transport.md`). `real_ns()` + `request_shutdown()`
//! already have cdylib coverage; these three reads did not.
//!
//! Cribs `cerulion_core/tests/macro_shim_clock_sources_test.rs` (the
//! in-process proof of the same contract). Stamps every read into a
//! `Twist` output's `linear`/`angular` fields so the host can observe them
//! without any FFI accessor beyond the normal wire delivery. Consumed by
//! `cerulion_core/tests/cdylib_clock_accessors_test.rs`.
//!
//! # The shared memory address probe
//!
//! Under `CER_ZC_ADDR_PROBE` this fixture ALSO records the raw virtual address
//! of the output slot it writes through, and stamps `PROBE_MARKER_BITS` into
//! `angular.z`. `cerulion_core/tests/cdylib_shm_write_address_test.rs` reads the
//! address back through [`cerulion_test_get_shm_write_addr`] and asks the OS
//! which mapping contains it: a zero copy plugin write lands in the publisher's
//! shared memory segment, a marshalled one lands on the heap. Byte parity cannot
//! tell those apart, which is why the address is the observable.
//!
//! The probe is OFF unless the variable is set, so `cdylib_clock_accessors_test`
//! sees the exact tick it saw before: `angular.z` stays 0.0 and the static stays
//! at its initial 0.

#![deny(unused_imports)]
// Principle 12 (logging; see the Logging convention in `AGENTS.md`): library
// code never prints. It logs through `tracing`. Scoped `not(test)` so unit
// tests keep printing diagnostics, and applied at the crate root rather than
// in `[workspace.lints]` because that table cannot distinguish a lib target
// from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use core::sync::atomic::{AtomicUsize, Ordering};
use native_ros2_messages::geometry_msgs::Twist;

/// The bit pattern the probed tick stamps into `angular.z`. A hand chosen
/// constant no code re derives, so a host side compare against the same literal
/// is an oracle for the delivered bytes rather than a self compare.
const PROBE_MARKER_BITS: u64 = 0x5A5A_C0FF_EE00_1234;

/// Virtual address of the `angular.z` slot the last probed tick wrote through.
///
/// Process global, so every `dlopen` of this object shares it: the host reads it
/// through its own `libloading` handle of the same path while the graph that
/// produced it is still alive, which keeps the mapping it names valid.
static SHM_WRITE_ADDR: AtomicUsize = AtomicUsize::new(0);

/// Whether this process asked for the address probe.
fn addr_probe_enabled() -> bool {
    std::env::var_os("CER_ZC_ADDR_PROBE").is_some()
}

#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct ClockProbe {
    #[output]
    out: Twist,
}

#[cerulion_node_impl]
impl ClockProbe {
    fn tick(&mut self) -> Result<(), NodeError> {
        let now = self.now_ns();
        self.out.linear.x = now as f64;

        let virt = self.virt_ns();
        self.out.linear.y = if virt.is_some() { 1.0 } else { 0.0 };
        self.out.linear.z = virt.unwrap_or(0) as f64;

        let ext = self.ext_ns();
        self.out.angular.x = if ext.is_some() { 1.0 } else { 0.0 };
        self.out.angular.y = ext.unwrap_or(0) as f64;

        if addr_probe_enabled() {
            self.out.angular.z = f64::from_bits(PROBE_MARKER_BITS);
            // The macro rewrites `self.out` to the loaned `OutputProxy`, which
            // derefs to the `#[repr(C)]` `TwistShm` overlaid on the iceoryx2
            // payload, so this is the address of the payload slot the line above
            // wrote. Taken LAST in the tick: the borrow ends with the statement.
            SHM_WRITE_ADDR.store(
                std::ptr::from_ref(&self.out.angular.z) as usize,
                Ordering::Relaxed,
            );
        }
        Ok(())
    }
}

// ===========================================================================
// Probe accessors (read/reset the process global address slot).
// ===========================================================================

/// Read the address the last probed tick wrote its output payload through, or 0
/// when no probed tick has run. `handle` is accepted for ABI shape but ignored:
/// the slot is process global, shared across every `dlopen` of this object, so
/// the host reads it through its own `libloading` handle of the same path.
#[no_mangle]
pub extern "C" fn cerulion_test_get_shm_write_addr(_handle: u64) -> u64 {
    SHM_WRITE_ADDR.load(Ordering::Relaxed) as u64
}

/// Clear the address slot, so an arm that must observe a FRESH write cannot pass
/// on the address a previous arm in the same process left behind.
#[no_mangle]
pub extern "C" fn cerulion_test_reset_shm_write_addr() {
    SHM_WRITE_ADDR.store(0, Ordering::Relaxed);
}
