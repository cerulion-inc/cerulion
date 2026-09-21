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

#![deny(unused_imports)]
// Principle 12 (logging; see the Logging convention in `AGENTS.md`): library
// code never prints. It logs through `tracing`. Scoped `not(test)` so unit
// tests keep printing diagnostics, and applied at the crate root rather than
// in `[workspace.lints]` because that table cannot distinguish a lib target
// from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Twist;

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
        Ok(())
    }
}
