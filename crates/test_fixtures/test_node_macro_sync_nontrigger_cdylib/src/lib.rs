// SPDX-License-Identifier: AGPL-3.0-only
//! The FFI-parity fixture for **trigger-scoped Sync** — a
//! bounded-sync cdylib with 2 `#[input(trigger)]` ports AND a plain
//! `#[input]` latest-value context port.
//!
//! Sibling of `test_node_macro_sync_cdylib` (all-trigger bounded Sync). This
//! fixture is the production-surface proof of the flip ACROSS the
//! cdylib FFI: the node must fire when `cam` + `lidar` align in the 25 ms
//! window REGARDLESS of `config`'s arrival state (`config` is a non-trigger
//! latest-value read, held across steps via the snapshot FFI
//! symbols, and its trigger-absence must survive the ABI-v9 info-JSON carry).
//!
//! The tick surfaces what it observed through the fixed output (the
//! counters-via-output pattern): `fused.x` = the trigger sum (alignment
//! proof), `fused.y` = the `config` value the tick READ (hold proof). A
//! never-delivered `config` collapses the tick (the pre-first-delivery
//! WAIT) — the scheduler fire happens, but nothing publishes.
//!
//! Consumed by `cdylib_sync_nontrigger_test.rs` (fire + hold + parity +
//! determinism e2e) — build first:
//!
//! ```bash
//! cargo build -p test_node_macro_sync_nontrigger_cdylib
//! ```

#![deny(unused_imports)]
// P12 (the project logging rule): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(sync_window_ms = 25)]
struct SyncNonTriggerNode {
    #[input(trigger)]
    cam: Vector3,

    #[input(trigger)]
    lidar: Vector3,

    /// Plain (non-trigger) latest-value context — never gates the fire
    /// (trigger-scoped Sync), holds across steps (over the snapshot FFI).
    #[input]
    config: Vector3,

    #[output]
    fused: Vector3,
}

#[cerulion_node_impl]
impl SyncNonTriggerNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Trigger-alignment proof: the sum of the two synced triggers.
        self.fused.x = self.cam.x + self.lidar.x;
        // Hold proof: the config value THIS tick observed (latest delivered,
        // or the held replay on a config-silent step).
        self.fused.y = self.config.x;
        Ok(())
    }
}
