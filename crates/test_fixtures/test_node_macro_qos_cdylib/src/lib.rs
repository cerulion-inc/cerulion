// SPDX-License-Identifier: AGPL-3.0-only
//! Cdylib QoS round-trip oracle fixture.
//!
//! Declares ALL FOUR QoS knobs via the `#[cerulion_node]` macro
//! so the integration test `cdylib_qos_ffi_test` can load this `.so`/
//! `.dylib` through `DylibNodeEntry`, call `info()`, and assert each
//! knob round-trips across the v6 info-JSON FFI:
//!
//!   - node-level `tick_within_ms = 500` → `NodeInfo::tick_within_ms()`
//!     (500, raised from 10: the budget is a WALL-CLOCK gate, and a routine
//!     ≥10 ms scheduler preemption of the tick on a loaded CI runner blew
//!     the old budget — the flake class, observed on Linux main CI.
//!     The quiet-arm bug classes `cdylib_qos_behavioral_test` pins — a
//!     counter false-firing on µs-class ticks (FFI wiring/units bugs: the
//!     budget crossing as 0, ns-vs-ms confusion make EVERY µs-class tick
//!     blow ANY budget) and dead-vs-wired counter plumbing — are
//!     budget-independent, so the raise loses no detection power.)
//!   - node-level `throttle_ms   = 5`    → `NodeInfo::throttle_ms()`
//!   - input  `expect_within_ms  = 20`   → `InputMeta::expect_within_ms`
//!   - output `promise_within_ms = 30`   → `OutputMeta::promise_within_ms`
//!
//! `throttle_ms` is mutually exclusive with `period_ms` (rejected at
//! macro-expansion), so this node uses a `#[input(trigger)]` field for
//! its firing policy (a `MacroPolicy::DataTrigger`) — orthogonal to the
//! `throttle_ms` rate cap and `tick_within_ms` budget, both of which
//! STACK with any non-`period_ms` trigger.
//!
//! A regression to either side of the v6 FFI (macro emission in
//! `gen_cdylib` OR host parsing in `parse_info_json`) fires the test.

#![deny(unused_imports)]
// Principle #12 (logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(tick_within_ms = 500, throttle_ms = 5)]
struct QosNode {
    // Trigger input carrying a per-input deadline watchdog. The trigger
    // flag makes this a DataTrigger node (so it can fire); the
    // `expect_within_ms` is the input watchdog QoS, plumbed
    // across the FFI.
    #[input(trigger, expect_within_ms = 20)]
    velocity_in: Vector3,

    // Output carrying a per-output promise (publish-side deadline QoS).
    #[output(promise_within_ms = 30)]
    cmd_out: Vector3,
}

#[cerulion_node_impl]
impl QosNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // SHM-to-SHM copy so the macro's port rewrite keeps both ports live.
        self.cmd_out.x = self.velocity_in.x;
        self.cmd_out.y = self.velocity_in.y;
        self.cmd_out.z = self.velocity_in.z;
        Ok(())
    }
}
