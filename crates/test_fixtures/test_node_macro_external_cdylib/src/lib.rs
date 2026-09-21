// SPDX-License-Identifier: AGPL-3.0-only
//! F6 macro→runtime policy round-trip oracle for External.

#![deny(unused_imports)]
// P12 (Logging, see AGENTS.md): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(external)]
struct ExternalNode {
    #[output]
    cmd: Vector3,
}

#[cerulion_node_impl]
impl ExternalNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.x = 0.0;
        Ok(())
    }

    // External nodes must declare their ExternalSource. This
    // fixture is host-driven (fired via `trigger_external`), so the generated
    // cdylib `cerulion_node_external_source` export returns kind 3 (HostDriven).
    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}
