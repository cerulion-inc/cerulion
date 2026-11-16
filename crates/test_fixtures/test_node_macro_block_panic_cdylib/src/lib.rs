// SPDX-License-Identifier: AGPL-3.0-only
//! A data-triggered node whose trigger input is `backpressure = block` at
//! depth 2, throttled to one fire per 20 ms. Its tick panics on the frame
//! `x = 3` and on every frame from `x = 1000` on, and otherwise mirrors `x`
//! to its output.
//!
//! Loaded by `cerulion_core/tests/block_slot_release_iox2_test.rs`: a node
//! library catches a tick panic and keeps running, so the consumer's `block`
//! slot stays armed through the panic and its producer stays paced.

#![deny(unused_imports)]
// Library code never prints; it logs through `tracing`. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// The one frame that panics in a run whose producer counts up from 1.
const PANIC_ONCE_AT: f64 = 3.0;
/// From this frame on, every frame panics.
const PANIC_ALWAYS_FROM: f64 = 1000.0;

#[cerulion_node(throttle_ms = 20)]
struct BlockPanicNode {
    #[input(trigger, backpressure = block, depth = 2)]
    inp: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl BlockPanicNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let x = self.inp.x;
        if (x - PANIC_ONCE_AT).abs() < f64::EPSILON || x >= PANIC_ALWAYS_FROM {
            panic!("block panic fixture: frame x = {x}");
        }
        self.out.x = x;
        Ok(())
    }
}
