// SPDX-License-Identifier: AGPL-3.0-only
//! A `period_ms` cdylib node carrying a plain (NON-trigger) `#[input]`.
//!
//! Loaded by two tests:
//!
//! - `tests/rayon_fire_cdylib_serial_test.rs`: pins the
//!   serial routing of a REAL `DylibNodeEntry`. A `Period` node with a
//!   latest-value input lands in `snapshot_input_names`, and because a cdylib
//!   reports `performs_input_snapshot() == false`, the level executor routes it
//!   onto the serial fire path (`serial_fire_node_ids` set A). The test proves
//!   the trace stays byte-identical THREADS=4 vs THREADS=1 with this cdylib in
//!   the level — the existing `rayon_fire_iox2_test` Test 6 covers only the
//!   CLOSURE case.
//! - `tests/cdylib_non_trigger_hold_test.rs`: pins the cross-step HOLD
//!   of the non-trigger `inp` over the FFI. Built with the macro this
//!   fixture exports the optional `cerulion_node_{set_,}snapshot_inputs` symbols,
//!   so `DylibNodeEntry::snapshot_inputs` forwards the per-step freeze and the
//!   node HOLDS `inp` across silent steps (`holds_input_snapshot() == true`)
//!   while still firing serially (`performs_input_snapshot() == false`).
//!
//! Why a NEW fixture: no existing macro cdylib fixture is a `period_ms` node
//! with a plain (non-trigger) `#[input]` — `test_node_macro_period_cdylib` is
//! output-only, `test_node_macro_cdylib` has an `#[input(trigger)]` (Data
//! trigger), and the rest are trigger / external / sync.

#![deny(unused_imports)]
// Principle #12 (logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 10)]
struct PeriodInputNode {
    /// Plain (non-trigger) latest-value input → the node is classified into
    /// `snapshot_input_names`. It is routed serial under within-level parallel
    /// fire (`performs_input_snapshot() == false`), and the host makes its
    /// cross-step HOLD real via the FFI snapshot forward.
    #[input]
    inp: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl PeriodInputNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // A real read of the (frozen/live) input slot + a real publish — no
        // fake data (Principle #13). The trace records only fire order/time, so
        // the body's value is incidental, but the read/publish exercise the
        // wired ports.
        self.out.x = self.inp.x;
        Ok(())
    }
}
