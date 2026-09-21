// SPDX-License-Identifier: AGPL-3.0-only
//! Cdylib parity: the **`block`** half of `#[input(backpressure =
//! ...)]` on a PER-SET Sync TRIGGER input, on the surface every `graph run`
//! node actually deploys on.
//!
//! An EXACT declaration mirror of `sync_per_set_backpressure_iox2_test.rs`'s
//! in-process `BlockPairFuse` — `#[cerulion_node(sync_window_ms = 50)]`,
//! trigger `a` under `backpressure = block, depth = 4`, trigger `b` on the
//! default `drop_oldest`, and an `#[on_event(input = "a")]` handler — plus the
//! one thing a cdylib needs that an in-process node does not: an OUTPUT.
//!
//! # Why an output, and why `fused.z`
//!
//! A cdylib's Rust state is opaque across the FFI, so the in-process suites'
//! `with_state`-injected `Arc<Mutex<Vec<_>>>` sink cannot be used here. The
//! tick instead publishes what it OBSERVED into the fixed `Vector3` output and
//! a host-side closure sink reads it back off SHM — the counters-via-output
//! pattern (`test_node_macro_onevent_cdylib`). The values crossing the SHM
//! boundary ARE the proof the behaviour happened inside the cdylib
//! (Principle #13 — no fake data):
//!
//! * `fused.x` = the `a` member this fire read (its publish ordinal, so the
//!   recorded sequence names exactly which frames were served — the LOSSLESS
//!   oracle is contiguity over these values);
//! * `fused.y` = the `b` member this fire read;
//! * `fused.z` = how many `Block` backpressure events this node's own
//!   `#[on_event]` handler has drained. Dispatch runs AFTER the body on the
//!   tick-Ok path, so this lags the true count by one fire; the driving test
//!   takes the MAX, which is monotone-safe.
//!
//! Only two things could make `fused.z` move: the input really is declared
//! `block` (a `drop_oldest` input mints `DropOldest` events, which the guard
//! below refuses), and the FFI carried that declaration. `depth = 4` is the
//! other half — it is what makes the 1 ms producer in the driving test defer
//! at all.
//!
//! Consumed by `cdylib_sync_backpressure_parity_test.rs` — build first:
//!
//! ```bash
//! cargo build -p test_node_macro_sync_block_cdylib
//! ```

#![deny(unused_imports)]
// P12 (Logging, see AGENTS.md): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// What `block_events` is bumped by when a `Block` event reports a DROP.
///
/// `block` is lossless, so such an event is the contract broken — and the
/// counter must not report it as ZERO, which is indistinguishable from "no
/// Block events at all" and would fail with a message asserting the opposite
/// diagnosis. The in-process reference (`BlockPairFuse`) hard-`assert!`s this;
/// a cdylib cannot, because a panicking handler crosses the FFI as an opaque
/// code, so it is encoded in the value instead and the driving test asserts
/// `max_events < LOSSY_BLOCK_SENTINEL` beside `>= 1`.
///
/// Mirrored by `LOSSY_BLOCK_SENTINEL` in
/// `cerulion_core/tests/cdylib_sync_backpressure_parity_test.rs` — change both.
const LOSSY_BLOCK_SENTINEL: u32 = 1_000_000;

#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct SyncBlockPairNode {
    /// The LOSSLESS edge. `depth = 4` is the producer's whole budget: at most
    /// four unserved frames on this edge at any instant, counting the member
    /// the per-set matcher is holding in its frozen head.
    #[input(trigger, backpressure = block, depth = 4)]
    a: Vector3,

    /// The partner, deliberately on the DEFAULT `drop_oldest` policy so the
    /// per-INPUT rule is exercised rather than a whole-node one.
    #[input(trigger)]
    b: Vector3,

    #[output]
    fused: Vector3,

    /// Read by `tick` (published as `fused.z`), so `dead_code = deny` is
    /// satisfied without an annotation.
    block_events: u32,
}

#[cerulion_node_impl]
impl SyncBlockPairNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.fused.x = self.a.x;
        self.fused.y = self.b.x;
        self.fused.z = f64::from(self.block_events);
        Ok(())
    }

    /// The CONSUMER side of `block`: "your queue — including the head the
    /// matcher is holding for you — was at the defer line when this drain ran".
    ///
    /// The POLICY guard is what makes the counter a signal: a `drop_oldest`
    /// input mints `BackpressurePolicy::DropOldest`, which this refuses to
    /// count, so a fixture whose `backpressure = block` was stripped publishes
    /// `fused.z == 0` forever whatever else it does.
    ///
    /// A Block event reporting a DROP takes the sentinel rather than being
    /// silently ignored — see [`LOSSY_BLOCK_SENTINEL`]. Ignoring it would map
    /// the losslessness contract being BROKEN onto the same 0 as "the gate
    /// never engaged", which are opposite diagnoses.
    #[on_event(input = "a")]
    fn on_a_pressure(&mut self, event: BackpressureEvent) {
        if matches!(event.policy, BackpressurePolicy::Block) {
            self.block_events += if event.dropped == 0 {
                1
            } else {
                LOSSY_BLOCK_SENTINEL
            };
        }
    }
}
