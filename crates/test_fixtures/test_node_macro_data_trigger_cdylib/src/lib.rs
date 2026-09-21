// SPDX-License-Identifier: AGPL-3.0-only
//! Macro-emitted `data_trigger` policy round-trip oracle.
//!
//! Test cdylib that declares a single `#[input(trigger)]` field and no
//! node-level policy attribute — the canonical pattern that the macro
//! validator (`cerulion_macros::validate::validate_trigger_inference`)
//! rules valid as "single trigger input → DataTrigger, no additional
//! attr needed", and that pre-(a) silently never fired at runtime.
//!
//! After A2 the cdylib's `cerulion_node_info()` JSON carries
//! `"policy":{"data_trigger":{"input_name":"trigger_in"}}`. The
//! integration test (`macro_cdylib_policy_round_trip_test`) loads this
//! cdylib, calls `info()`, and asserts the returned `NodeInfo::policy`
//! is `Some(MacroPolicy::DataTrigger { input_name: "trigger_in" })`.
//!
//! Locks the contract end-to-end: macro emission → cdylib JSON →
//! `parse_info_json` deserialization → `MacroPolicy::DataTrigger`.

#![deny(unused_imports)]
// Principle 12 (logging): library code never prints. It logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node]
struct DataTriggerNode {
    #[input(trigger)]
    trigger_in: Vector3,

    #[output]
    cmd: Vector3,
}

#[cerulion_node_impl]
impl DataTriggerNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Mirror the input's x into cmd so the macro's port-rewrite
        // doesn't elide the input or output and downstream tests can
        // observe data flow if they wire a publisher upstream.
        self.cmd.x = self.trigger_in.x;
        Ok(())
    }
}
