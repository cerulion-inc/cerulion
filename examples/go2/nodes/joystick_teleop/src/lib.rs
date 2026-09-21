// SPDX-License-Identifier: AGPL-3.0-only
//! Go2 joystick teleop node.
//!
//! An `#[cerulion_node(external)]` ingress that reads a
//! Bluetooth gamepad on the companion computer and publishes `geometry_msgs/Twist`
//! on `/go2/cmd_vel/joystick`. The safety mux arbitrates this
//! against the keyboard source; the sport driver turns Twist into
//! Go2 sport `Move` requests.
//!
//! # Architecture (the injectable event-source seam)
//!
//! The WHOLE mapping policy lives in the pure [`mapping`] module (deadzone,
//! saturation, sign conventions, deadman gate — oracle-tested, no I/O), and
//! the WHOLE pump decision policy (connected seeding, keepalive cadence, the
//! liveness watchdog) in the pure [`pump_policy`] module (oracle-tested, no
//! I/O). This file is the transport wrapper; [`pump`] is the thin hardware
//! `gilrs` shell around `pump_policy`.
//!
//! - **Live/hardware**: [`JoystickTeleop::external_source`] returns an
//!   [`ExternalSource::Blocking`] closure (via [`pump::spawn_gilrs_blocking`])
//!   driven on a helper thread. The helper owns ALL wall-clock pacing —
//!   startup ring, event rings, the 20 Hz deadman keepalive, the 2 s liveness
//!   watchdog — and pushes [`mapping::PadEvent`]s into the shared `inbox`,
//!   ringing the doorbell when (and only when) a publish is wanted.
//! - **Test/replay**: `external_source()` is never queried on the polled
//!   `step()` path (Principle #7). Tests construct the node via
//!   `JoystickTeleopEntry::with_state(..)` with a shared `inbox`, push scripted
//!   `PadEvent`s, and drive `trigger_external` + `step` — no hardware, fully
//!   deterministic.
//!
//! # Determinism (precise claim)
//!
//! (a) The node BODY ([`JoystickTeleop::tick`]) is a pure deterministic
//! function of its drained `inbox`: it folds the events into `self.state` and
//! writes `self.state.command()`, reading no clock (the mapping is
//! timing-free; keepalive/disconnect-zero cadence are the helper's RING
//! decisions, not body decisions). (b) Platform replay (`bag play --resim`)
//! verifies an ingress node like this one by byte-comparing its RECORDED
//! published frames against the bag (Principle #7) — it does NOT re-drive the
//! inbox, which is a co-equal input alongside the fire schedule. Nothing
//! stronger is claimed.
//!
//! # Startup-zero + no-idle-publish contract (binding requirement)
//!
//! Because a macro node that returns `Ok(())` publishes ALL outputs on EVERY
//! tick, tick cadence == publish cadence. The helper therefore rings
//! EXACTLY ONCE at startup (device present or not) — the node's first tick
//! drains an empty inbox, the default state is un-armed, so it emits a single
//! zero Twist — and then rings ONLY on real input activity (deadman-gated),
//! the keepalive-while-held, or the single liveness-watchdog zero (which is
//! itself a disarm, so it cannot repeat). Steady zeros while idle are NEVER published: a
//! fresh zero stream would permanently override the keyboard in the mux's
//! freshness arbitration (the single startup zero goes stale within 250 ms and
//! can never mute the keyboard). The no-idle guarantee is STRUCTURAL — the
//! helper simply does not ring while idle. Both behaviors are pinned in
//! `tests/joystick_e2e_test.rs`.
//!
//! # Safety
//!
//! No motion is emitted without the deadman (RB) held AND a pad connected
//! (see [`mapping::PadState::command`]). Releasing RB publishes one zero then
//! goes quiet; a Bluetooth drop publishes one zero + a loud warn and keeps
//! scanning for reconnect (the node never exits). A SILENT link with the
//! deadman latched trips the [`pump_policy::LIVENESS_TIMEOUT_NS`] watchdog
//! (one zero + disarm + loud warn), bounding how long a
//! BT-stalled radio can keep republishing the last command. The mux's
//! staleness gate is the defense-in-depth layer above all of this.

// NOTE: no crate-level `#![forbid(unsafe_code)]` — the `#[cerulion_node]`
// macro expands cdylib FFI entry points containing `unsafe`. The PURE mapping
// + pump_policy modules keep the forbid at module scope.

// P12 (the project logging rule): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod mapping;
mod pump;
pub mod pump_policy;

// Re-export the saturation constants at the crate root so the cross-crate
// constant-consistency test (and any downstream) has a stable path. These are
// the CANONICAL teleop saturation limits; the keyboard node duplicates them
// and `tests/const_consistency_test.rs` pins the two in lockstep.
pub use mapping::{DEADZONE, MAX_VX, MAX_VY, MAX_VYAW};

use std::sync::{Arc, Mutex};

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Twist;

use crate::mapping::{PadEvent, PadState};

/// The joystick teleop node. See the module docs for the full contract.
///
/// `external`-policy: fired by the `gilrs` helper's doorbell rings on the live
/// path, or by `trigger_external` on the polled/test path. One output only.
// Field notes (plain comments — port fields carry only the macro attrs):
// - inbox: the shared event queue between the hardware pump (live) or the test
//   script and the node body. Drained + folded every tick.
// - state: the folded gamepad state (latched axes + deadman + connected).
// - cmd: the arbitrated velocity command written every tick (fixed-POD Twist,
//   zero-copy assignment).
#[cerulion_node(external)]
#[derive(Default)]
pub struct JoystickTeleop {
    inbox: Arc<Mutex<Vec<PadEvent>>>,
    state: PadState,
    #[output]
    cmd: Twist,
}

impl JoystickTeleop {
    /// Construct a node sharing `inbox` with a caller (a test script, or the
    /// hardware pump via [`Self::external_source`]). The macro injects a
    /// hidden runtime field with its own `Default`, so `..Default::default()`
    /// initialises everything else; this constructor lives IN-CRATE so the
    /// private `inbox`/`state`/`cmd` fields are reachable (an integration test
    /// in a separate crate cannot construct the struct directly). Inject via
    /// `JoystickTeleopEntry::with_state(JoystickTeleop::with_inbox(inbox))`.
    pub fn with_inbox(inbox: Arc<Mutex<Vec<PadEvent>>>) -> Self {
        Self {
            inbox,
            ..Default::default()
        }
    }

    /// The shared event inbox — lets a test or the pump push [`PadEvent`]s
    /// that the next tick drains.
    pub fn inbox(&self) -> Arc<Mutex<Vec<PadEvent>>> {
        Arc::clone(&self.inbox)
    }
}

#[cerulion_node_impl]
impl JoystickTeleop {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Drain the shared inbox (events pushed by the hardware pump on the
        // live path, or by the test script). Take-and-release so the pump
        // thread is never blocked on the fold. Poison-RECOVERING lock: the
        // POD-Vec contents can never be torn, and delivering safety events
        // (Disconnected / Deadman(false)) after a panic strictly beats
        // dropping them — see the rationale comment in `pump.rs::apply`.
        let drained: Vec<PadEvent> = std::mem::take(
            &mut *self
                .inbox
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner),
        );
        for event in drained {
            self.state.apply(event);
        }

        // Output is a pure function of the folded state (no node-clock read —
        // the joystick mapping is timing-free; see the module docs). Write all
        // six Twist components explicitly (fixed-POD → zero-copy assignment).
        let cmd = self.state.command();
        self.cmd.linear.x = cmd.vx;
        self.cmd.linear.y = cmd.vy;
        self.cmd.linear.z = 0.0;
        self.cmd.angular.x = 0.0;
        self.cmd.angular.y = 0.0;
        self.cmd.angular.z = cmd.vyaw;

        Ok(())
    }

    /// The node's external-ingress source (queried ONCE on the live path,
    /// never under polled `step()` / replay). Hands the runtime the hardware
    /// `gilrs` pump, sharing this node's `inbox`.
    fn external_source(&mut self) -> ExternalSource {
        pump::spawn_gilrs_blocking(Arc::clone(&self.inbox))
    }
}
