// SPDX-License-Identifier: AGPL-3.0-only
//! Exit-3 widening: the PANICKING twin of
//! `test_node_macro_period_cdylib` — same node shape (the same
//! `#[cerulion_node(period_ms = 50)]` trigger, the same `PeriodNode` type
//! name, the same single `cmd: Vector3` output), but the tick PANICS on its
//! THIRD fire (ticks 1–2 succeed, publishing the original's `cmd.x = 0.0`).
//! Exists so the real-bag e2e can prove exit 3 — "node failure: execution
//! (panic-class)" — without an in-test cargo build: the CI stand-in for
//! "the candidate crashes mid-replay".
//!
//! The panic unwinds inside the macro-generated `cerulion_node_tick` FFI
//! wrapper: the first panicking call returns FFI code 2 ("panic caught by
//! catch_unwind") and poisons the cdylib's process-global `NODES` mutex, so
//! every later call returns code 3 — the host maps BOTH codes to the
//! structural `TransportError::NodeTickPanicked` (never string-matched), the
//! runtime records the node's first panic-class reason, and `cerulion
//! replay` maps it to exit 3 (preempting the downstream missing-frame data
//! violations and the thinner replayed schedule).

#![deny(unused_imports)]
// P12 (the repo's logging policy): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 50)]
struct PeriodNode {
    #[output]
    cmd: Vector3,
    ticks: u32,
}

#[cerulion_node_impl]
impl PeriodNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.ticks += 1;
        if self.ticks >= 3 {
            panic!(
                "injected candidate crash at tick {} (exit-3 twin)",
                self.ticks
            );
        }
        // Ticks 1-2: byte-identical to the original fixture's payload.
        self.cmd.x = 0.0;
        Ok(())
    }
}
