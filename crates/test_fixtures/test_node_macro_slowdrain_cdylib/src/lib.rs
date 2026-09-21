// SPDX-License-Identifier: AGPL-3.0-only
//! Bp-drop-oldest carrier: a deliberately SLOW (50ms) macro cdylib
//! consumer whose bare non-trigger `#[input]` is left at the default
//! backpressure (`drop_oldest`) and default depth
//! (`cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH`).
//!
//! Why a NEW fixture (see `cdylib_qos_behavioral_test.rs`'s SKIPPED section):
//! every existing macro cdylib `period_ms` fixture drains fast enough to keep
//! pace with a fast producer — `test_node_macro_period_input_cdylib` (10ms
//! period, default depth) drains exactly depth-per-window under a 1ms flood
//! (0 evictions). This fixture's period is 5x that (50ms), so a 1ms-period
//! producer publishes roughly 50 samples between two of this node's own
//! ticks, guaranteeing real `drop_oldest` evictions against the small
//! default queue depth — a reliable overflow carrier for
//! `cdylib_dropoldest_overflow_test.rs`.

#![deny(unused_imports)]
// Principle 12 (logging): library code never prints. It logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 50)]
#[derive(Default)]
struct SlowDrainConsumer {
    /// Bare non-trigger input — default backpressure (`drop_oldest`) at the
    /// default depth. Left un-overflowed by every existing macro cdylib
    /// fixture (the gap this fixture's slow 50ms period closes).
    #[input]
    inp: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl SlowDrainConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.inp.x;
        Ok(())
    }
}
