// SPDX-License-Identifier: AGPL-3.0-only
//! Macro→runtime policy round-trip + fire oracle for **UnboundedSync**
//! across the cdylib FFI.
//!
//! Sibling of `test_node_macro_sync_cdylib` (bounded Sync). Previously the
//! macro's `gen_cdylib` `policy_json` chain had NO `unbounded_sync` arm, so a
//! `#[cerulion_node(unbounded_sync)]` cdylib emitted `cerulion_node_info()`
//! JSON with no `"policy"` key. The host parsed `NodeInfo::policy() == None`
//! and the runtime defaulted the node to `TriggerPolicy::Data` (fires on ANY
//! single input arrival) — silently dropping the all-inputs UnboundedSync
//! contract a fusion node depends on.
//!
//! Consumed by:
//!   - `macro_cdylib_policy_round_trip_test` — the parse pin (policy round-trip).
//!   - `cdylib_unbounded_sync_fire_test` — the behavioral fire + no-lost-data
//!     oracle over real iceoryx2.
//!
//! Two `#[input(trigger)]` fields because `unbounded_sync` requires ≥2 trigger
//! inputs (macro-enforced by `validate.rs::validate_trigger_inference`).

#![deny(unused_imports)]
// Principle #12 (logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(unbounded_sync)]
struct UnboundedSyncNode {
    #[input(trigger)]
    a: Vector3,

    #[input(trigger)]
    b: Vector3,

    #[output]
    fused: Vector3,
}

#[cerulion_node_impl]
impl UnboundedSyncNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Sum the two synced inputs straight into the loaned SHM slot — the
        // hand oracle for cdylib_unbounded_sync_fire_test is `a.x + b.x`.
        self.fused.x = self.a.x + self.b.x;
        Ok(())
    }
}
