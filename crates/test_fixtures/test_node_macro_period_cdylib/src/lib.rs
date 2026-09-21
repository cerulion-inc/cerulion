// SPDX-License-Identifier: AGPL-3.0-only
//! F6 macro→runtime policy round-trip oracle.
//!
//! Test cdylib that declares `period_ms = 50` via the `#[cerulion_node]`
//! macro. The integration test `macro_cdylib_policy_round_trip_test`
//! loads this cdylib, calls `info()`, and asserts the returned
//! `NodeInfo::policy` is `Some(MacroPolicy::Period { period_ms: 50 })`.
//!
//! Locks the contract: macro-declared `period_ms` round-trips through
//! the cdylib's `cerulion_node_info()` JSON serialization and the host
//! loader's `parse_info_json` deserialization. A regression to either
//! side (macro emission OR host parsing) fires the test.

#![deny(unused_imports)]
// P12 (the AGENTS.md logging convention): library code never prints — it logs through
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
}

#[cerulion_node_impl]
impl PeriodNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Touch the output so the macro's port-rewrite doesn't elide it.
        self.cmd.x = 0.0;
        Ok(())
    }
}
