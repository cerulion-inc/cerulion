// SPDX-License-Identifier: AGPL-3.0-only
//! Cdylib parity: the **`sample(N)`** half of `#[input(backpressure =
//! ...)]` on a PER-SET Sync TRIGGER input, on the surface every `graph run`
//! node actually deploys on.
//!
//! An EXACT declaration mirror of `sync_per_set_backpressure_iox2_test.rs`'s
//! in-process `SampledPairFuse` — `#[cerulion_node(sync_window_ms = 50)]`,
//! trigger `a` under `backpressure = sample(5), depth = 16`, trigger `b` on the
//! default `drop_oldest`, and an `#[on_event(input = "a")]` handler — plus the
//! OUTPUT a cdylib needs to be observable at all.
//!
//! # A SEPARATE crate from the `block` fixture, and why
//!
//! The `#[cerulion_node]` cdylib FFI exports FIXED symbol names
//! (`cerulion_node_info`, `cerulion_node_tick`, …), so a cdylib carries exactly
//! ONE macro node and the two policies cannot share a crate. They could not
//! share a NODE either: `block` requires an in-graph producer
//! (`GraphTopology::validate`), while the `sample(N)` membership oracle
//! requires hand-stamped publishes onto an absolute external topic — a source
//! shape `block` refuses. Two fixtures is the shape the rules force.
//!
//! # Observability
//!
//! Same counters-via-output pattern as the sibling: `fused.x` = the `a` member
//! this fire read (the payload IS its own wire stamp, so a recorded set names
//! exactly the frames it observed), `fused.y` = the `b` member, `fused.z` = how
//! many `Sample(5)` events this node's own `#[on_event]` handler has drained
//! (one fire of publish lag — dispatch runs after the body).
//!
//! The handler's guard matches the DECLARED window, so it counts nothing unless
//! the FFI really carried `sample(5)`: a stripped fixture mints `DropOldest`
//! events (or none at all) and publishes `fused.z == 0` forever.
//!
//! Consumed by `cdylib_sync_backpressure_parity_test.rs` — build first:
//!
//! ```bash
//! cargo build -p test_node_macro_sync_sample_cdylib
//! ```

#![deny(unused_imports)]
// Principle #12 (structured logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct SyncSamplePairNode {
    /// The GATED edge. The gate lives inside the one pop primitive every
    /// per-set path uses, so a decimated frame can never become a head and
    /// never join a set: `N` and the sync window `W` compose as a PIPELINE,
    /// never as a joint predicate.
    #[input(trigger, backpressure = sample(5), depth = 16)]
    a: Vector3,

    /// The partner, on the DEFAULT `drop_oldest` policy — the per-INPUT rule.
    #[input(trigger)]
    b: Vector3,

    #[output]
    fused: Vector3,

    /// Read by `tick` (published as `fused.z`), so `dead_code = deny` is
    /// satisfied without an annotation.
    sample_events: u32,
}

#[cerulion_node_impl]
impl SyncSamplePairNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.fused.x = self.a.x;
        self.fused.y = self.b.x;
        self.fused.z = f64::from(self.sample_events);
        Ok(())
    }

    /// The decimation regime reaching the node's own handler, INSIDE the
    /// cdylib. The guard names the declared window, so it can only move if the
    /// FFI carried `sample(5)` — a stripped input mints `DropOldest`.
    #[on_event(input = "a")]
    fn on_a_pressure(&mut self, event: BackpressureEvent) {
        if matches!(event.policy, BackpressurePolicy::Sample(5)) {
            self.sample_events += 1;
        }
    }
}
