// SPDX-License-Identifier: AGPL-3.0-only
//! Hold-block-excluded carrier: ports `BlockHoldConsumer`
//! (`non_trigger_hold_iox2_test.rs`) through the cdylib FFI.
//!
//! The in-process original records `self.inp.x` straight into a
//! host-injected `Arc<AtomicU64>` inside `tick` — a channel a cdylib-loaded
//! node cannot receive (a `DylibNodeEntry` constructs its own state via the
//! FFI `new()`/`init()` entry points; the only channel across the boundary
//! is the declared ports). This fixture adds an `#[output]` so a downstream
//! in-process drain can observe "block reads live, never replays a held
//! value" from the HOST side (`cdylib_hold_block_excluded_test.rs`).

#![deny(unused_imports)]
// P12 (the AGENTS.md logging rule): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(external)]
#[derive(Default)]
struct HoldBlockProbe {
    /// Non-trigger `block` input — EXCLUDED from the step-boundary
    /// snapshot/hold (`build_snapshot_input_names` in `graph/runtime.rs`), so
    /// it reads LIVE every tick instead of replaying the last delivered
    /// value on a silent step.
    #[input(backpressure = block, depth = 4)]
    inp: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl HoldBlockProbe {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.inp.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}
