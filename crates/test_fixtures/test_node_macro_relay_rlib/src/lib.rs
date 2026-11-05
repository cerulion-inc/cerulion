// SPDX-License-Identifier: AGPL-3.0-only
//! Relay node that runs only when linked into another library.
//!
//! It has one trigger input with the same schema as the output of
//! `test_node_macro_period_cdylib`, so a graph can wire the period node into
//! it. Each tick copies the input's `x` and writes a tick count into `z`
//! (1, 2, 3, ...). A tick runs only when a frame arrives on the trigger
//! input, so a subscriber that sees `z` count up proves frames entered this
//! node through its input. `DeepRelayNode` is the same relay with the
//! deepest trigger queue an input can declare, and a hold that keeps its
//! ticks waiting so a burst piles up in that queue.

#![deny(unused_imports)]
// Library code never prints; scoped `not(test)` as in the other fixtures.
// Pinned by `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

static DEEP_RELAY_HOLD: AtomicBool = AtomicBool::new(false);

/// Hold [`DeepRelayNode`]'s ticks until the returned guard drops. A held
/// tick waits before it writes its echo, so the frames that arrive meanwhile
/// wait in the input queue. A test that panics while it holds the guard
/// releases the ticks as it unwinds, so a runtime stop does not wait on a
/// held tick.
pub fn hold_deep_relay() -> DeepRelayHold {
    DEEP_RELAY_HOLD.store(true, Ordering::SeqCst);
    DeepRelayHold(())
}

/// Releases [`DeepRelayNode`]'s ticks when dropped. Made by
/// [`hold_deep_relay`].
#[must_use = "the ticks are released when the guard drops"]
pub struct DeepRelayHold(());

impl Drop for DeepRelayHold {
    fn drop(&mut self) {
        DEEP_RELAY_HOLD.store(false, Ordering::SeqCst);
    }
}

#[cerulion_node]
pub struct RelayNode {
    #[input(trigger)]
    cmd: Vector3,

    #[output]
    echo: Vector3,

    ticks: u64,
}

#[cerulion_node_impl]
impl RelayNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.ticks += 1;
        self.echo.x = self.cmd.x;
        self.echo.z = self.ticks as f64;
        Ok(())
    }
}

/// [`RelayNode`] with a 64-frame trigger queue, the deepest an input can
/// declare: a burst of up to 64 frames waits in the queue while a tick is
/// held ([`hold_deep_relay`]), so every frame gets its own tick.
#[cerulion_node]
pub struct DeepRelayNode {
    #[input(trigger, depth = 64)]
    cmd: Vector3,

    #[output]
    echo: Vector3,

    ticks: u64,
}

#[cerulion_node_impl]
impl DeepRelayNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        while DEEP_RELAY_HOLD.load(Ordering::SeqCst) {
            std::thread::sleep(Duration::from_millis(1));
        }
        self.ticks += 1;
        self.echo.x = self.cmd.x;
        self.echo.z = self.ticks as f64;
        Ok(())
    }
}
