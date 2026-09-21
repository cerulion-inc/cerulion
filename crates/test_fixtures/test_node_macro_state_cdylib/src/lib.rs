// SPDX-License-Identifier: AGPL-3.0-only
//! The keystone state fixture: a macro cdylib with REAL state, for driving the
//! capture/restore FFI pair over a real `dlopen`.
//!
//! Four non-port fields, chosen so one round trip exercises every shape the
//! encoding documents and one behavioural step can observe all four:
//!
//! | field | encoding | why it is here |
//! |---|---|---|
//! | `counter: u64` | 8 bytes LE | the fixed-width scalar case |
//! | `label: String` | `u32` LE len + UTF-8 | a variable-length field |
//! | `history: Vec<f64>` | `u32` LE count + elements | a container of a non-integer |
//! | `tags: HashMap<u32, u8>` | `u32` LE count + sorted `(key, value)` | the HASH-LIKE case, which needs a sort index built BEFORE the first byte — the one shape `capacity_hint` exists for |
//! | `handle: u64` (`#[cerulion(reconstruct)]`) | **nothing** | the escape: a restore must leave it ALONE |
//!
//! The `#[output]` port is deliberately a real one: a port's declared type is a
//! zero-sized SHM marker with no `CerulionState` impl, so a capture that walked
//! it would not compile — and one that *skipped the escape* would silently
//! clobber a handle the node re-derived. Both facts are asserted by the test
//! rather than assumed here.
//!
//! `tick` republishes all four through one `Quaternion`, so the behavioural arm
//! can prove a restored value reaches the node's own body rather than merely
//! sitting in its fields.

#![deny(unused_imports)]
// Principle 12 (logging; see the Logging convention in `AGENTS.md`): library
// code never prints. It logs through `tracing`. Scoped `not(test)` so unit
// tests keep printing diagnostics, and applied at the crate root rather than
// in `[workspace.lints]` because that table cannot distinguish a lib target
// from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use std::collections::HashMap;

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Quaternion;

#[cerulion_node(period_ms = 10)]
struct StateProbeNode {
    #[output]
    out: Quaternion,
    counter: u64,
    label: String,
    history: Vec<f64>,
    tags: HashMap<u32, u8>,
    /// Stands in for a handle the node re-derives rather than restores. `init`
    /// sets it, a recording carries nothing for it, and a restore must not
    /// touch it.
    #[cerulion(reconstruct)]
    handle: u64,
}

#[cerulion_node_impl]
impl StateProbeNode {
    fn init(&mut self, _ctx: &mut NodeContext) -> Result<(), NodeError> {
        // The "re-derived handle": a value that exists only because `init` ran,
        // so a restore that clobbered it would read as 0 downstream.
        self.handle = 7;
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.counter as f64;
        self.out.y = self.history.len() as f64;
        self.out.z = self.label.len() as f64;
        self.out.w = (self.handle + self.tags.len() as u64) as f64;
        self.counter += 1;
        Ok(())
    }
}
