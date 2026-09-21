// SPDX-License-Identifier: AGPL-3.0-only
//! A DATA-TRIGGERED node whose trigger input declares
//! `backpressure = block` — the fixture the multi-process co-location e2e
//! needs and that no shipped fixture could provide.
//!
//! Both pre-existing `block` fixtures (`test_node_macro_holdblock_cdylib`,
//! `test_node_macro_depth_cdylib`) are `#[cerulion_node(external)]` returning
//! `ExternalSource::HostDriven`, which at launch on the live
//! path — so neither can be staged into a graph that a real `cerulion graph
//! run` executes. This one fires on data, exactly like
//! `test_node_macro_data_trigger_cdylib`, and differs from it in precisely one
//! attribute: the `block` policy on its trigger input.
//!
//! Its purpose is the auto-partition default. `block` defers the
//! producer's tick through a mirror the consumer's subscriber decrements, and
//! that mirror is a PROCESS-LOCAL `Arc<AtomicU64>` — so a partition that puts
//! the producer in another worker does not degrade the policy, it refuses to
//! build (`GraphTopology::validate` sees a `Block` consumer on a topic whose
//! producer was filtered out of this worker's subgraph). Staged as the `sink`
//! of the D5 chain, this node is what makes that failure — and the
//! co-location that fixes it — observable over the real binary.
//!
//! `depth = 4` is deliberately shallow: it keeps the defer reachable at the
//! e2e's tick rate rather than needing a long run to fill a default queue.

#![deny(unused_imports)]
// P12 (the AGENTS.md logging convention): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node]
struct TriggerBlockNode {
    #[input(trigger, backpressure = block, depth = 4)]
    trigger_in: Vector3,

    #[output]
    cmd: Vector3,
}

#[cerulion_node_impl]
impl TriggerBlockNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Mirror the input so neither port is elided by the macro's
        // port-rewrite and downstream consumers observe real data flow.
        self.cmd.x = self.trigger_in.x;
        Ok(())
    }
}
