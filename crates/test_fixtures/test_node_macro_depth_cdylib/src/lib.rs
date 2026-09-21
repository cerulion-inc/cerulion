// SPDX-License-Identifier: AGPL-3.0-only
//! ABI v8 fixture: a cdylib node declaring the input-QoS surface
//! that must cross the FFI —
//!
//! - `inp`: `#[input(backpressure = block, depth = 32)]` — depth 32 is
//!   DOUBLE the global transport default (16) and 3.2×
//!   `DEFAULT_CONSUMER_DEPTH` (10), so every oracle distinguishes "the
//!   declared value crossed" from BOTH pre-v8 failure modes (the hardcoded
//!   default and the transport default). `block` pre-A2 silently degraded
//!   to `DropOldest` on the dylib path — the Principle-#6-adjacent bug.
//! - `aux`: `#[input(backpressure = sample(7))]` — the third policy's
//!   wire shape (`{"sample":7}`), on a SEPARATE input so the block topic
//!   stays all-`block` (a mixed topic would degrade its block consumers).
//!
//! Consumed by `cerulion_core/tests/cdylib_depth_ffi_test.rs`:
//! - the parse pins (`info().input_meta()`: depth 32 + Block on `inp`,
//!   Sample(7) on `aux`),
//! - the in-process-vs-dylib parity pins, and
//! - the e2e oracles over real iceoryx2: topic-ceiling provisioning at 32
//!   AND the block plateau (producer fire_count stops at exactly 32
//!   against this stalled consumer — only possible because BOTH depth and
//!   `block` arrived across the FFI).
//!
//! `external` + `HostDriven`: the node never self-fires (a stalled
//! consumer by construction — the plateau oracle needs it to never drain).

#![deny(unused_imports)]
// Principle 12 (logging; see the Logging convention in `AGENTS.md`): library
// code never prints. It logs through `tracing`. Scoped `not(test)` so unit
// tests keep printing diagnostics, and applied at the crate root rather than
// in `[workspace.lints]` because that table cannot distinguish a lib target
// from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(external)]
#[derive(Default)]
struct DepthProbe {
    /// The load-bearing declarations: depth 32 + block must arrive
    /// host-side via the v8 info-JSON `"depth":32` +
    /// `"backpressure":"block"` keys.
    #[input(backpressure = block, depth = 32)]
    inp: Vector3,

    /// Sample-policy carrier: `"backpressure":{"sample":7}` on the wire.
    #[input(backpressure = sample(7))]
    aux: Vector3,
}

#[cerulion_node_impl]
impl DepthProbe {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        let _ = self.aux.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}
