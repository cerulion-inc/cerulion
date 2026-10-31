// SPDX-License-Identifier: AGPL-3.0-only
//! A `period_ms` cdylib node with an `#[input(accumulate, cap = 8)]` input.
//!
//! Loaded by `cerulion_core/tests/cdylib_accumulate_parity_test.rs`, which runs
//! it beside an in-process twin of the same source and checks that both
//! receive the same frames in the same order. A cdylib's state is opaque
//! across the FFI, so each fire publishes what it received into `out`:
//!
//! - `out.x` = how many frames the fire received;
//! - `out.y` = `sum((i + 1) * x_i)` over the frames in serve order, which
//!   changes when the order changes;
//! - `out.z` = `dropped()` for the fire.

#![deny(unused_imports)]
// Library code never prints; it logs through `tracing`. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 30)]
struct AccumulateNode {
    #[input(accumulate, cap = 8)]
    tf: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl AccumulateNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let mut count = 0.0;
        let mut weighted = 0.0;
        for frame in self.tf.iter() {
            count += 1.0;
            weighted += count * frame.x;
        }
        let dropped = self.tf.dropped() as f64;
        self.out.x = count;
        self.out.y = weighted;
        self.out.z = dropped;
        Ok(())
    }
}
