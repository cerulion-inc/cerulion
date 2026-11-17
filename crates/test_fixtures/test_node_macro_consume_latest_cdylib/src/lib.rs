// SPDX-License-Identifier: AGPL-3.0-only
//! `#[input(trigger, consume = latest)]` through the cdylib FFI.
//!
//! One `consume = latest` trigger input mirrored to one output, so every fire
//! publishes the frame its tick read. The integration test
//! (`consume_latest_iox2_test`) queues a burst on the input and reads the
//! output stream: a latest input fires once per step on the newest frame,
//! where the `fifo` fixture (`test_node_macro_data_trigger_cdylib`) forwards
//! the whole burst in order. That difference crosses the FFI only as the
//! info JSON's `"consume":"latest"` key, so this fixture is what pins the key
//! end to end.

#![deny(unused_imports)]
// Library code never prints; it logs through `tracing`. Scoped `not(test)`
// so unit tests keep printing diagnostics. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node]
struct ConsumeLatestNode {
    #[input(trigger, consume = latest)]
    trigger_in: Vector3,

    #[output]
    cmd: Vector3,
}

#[cerulion_node_impl]
impl ConsumeLatestNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.x = self.trigger_in.x;
        Ok(())
    }
}
