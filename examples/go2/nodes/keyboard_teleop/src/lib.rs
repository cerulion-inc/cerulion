// SPDX-License-Identifier: AGPL-3.0-only
//! Go2 keyboard teleop node.
//!
//! An `#[cerulion_node(external)]` ingress that reads the terminal (via
//! `crossterm`) and publishes `geometry_msgs/Twist` on `/go2/cmd_vel/keyboard`.
//! On the desk (`graphs/teleop_desk.yaml`) the topic is served over the network;
//! the companion's `graphs/teleop_remote.yaml` imports it (`ingress:`) and the
//! safety mux arbitrates it against the joystick. It also runs directly on the
//! companion for bench tests (crossterm is portable).
//!
//! # Architecture (the injectable event-source seam)
//!
//! The WHOLE key→Twist policy lives in the pure [`keymap`] module (latch,
//! presets, saturation, the timed auto-zero — oracle-tested, no I/O). This file
//! is the transport wrapper; [`pump`] is the hardware `crossterm` pump + raw
//! mode.
//!
//! - **Live/hardware**: [`KeyboardTeleop::external_source`] returns an
//!   [`ExternalSource::Blocking`] closure (via [`pump::spawn_crossterm_blocking`])
//!   on a helper thread. The helper owns ALL wall-clock pacing — startup ring,
//!   key rings, the 10 Hz keepalive, the auto-zero ring — and pushes
//!   [`keymap::KeyOp`]s into the shared `inbox`, ringing the doorbell only when
//!   a publish is wanted.
//! - **Test/replay**: `external_source()` is never queried on the polled
//!   `step()` path (Principle #7). Tests construct the node via
//!   `KeyboardTeleopEntry::with_state(..)` with a shared `inbox`, push scripted
//!   `KeyOp`s, and drive `trigger_external` + `step` — no terminal, fully
//!   deterministic.
//!
//! # Determinism (precise claim)
//!
//! (a) The node BODY ([`KeyboardTeleop::tick`]) is a pure deterministic
//! function of (node clock, drained inbox): it reads `self.now_ns()` ONCE,
//! folds the drained `KeyOp`s into `self.state` stamped at that time, and
//! writes `self.state.resolve(now)` — which applies the timed auto-zero from
//! the node clock. No wall clock is read in the body; the pump's (wall-clock)
//! ring timing decides WHEN the node ticks, the node decides WHAT it
//! publishes. (b) Platform replay (`bag play --resim`) verifies an ingress
//! node like this one by byte-comparing its RECORDED published frames against
//! the bag (Principle #7) — it does NOT re-drive the inbox, which is a
//! co-equal input alongside the fire schedule and the clock. Nothing stronger
//! is claimed.
//!
//! # Startup-zero + no-idle-publish contract (binding requirement)
//!
//! Because a macro node that returns `Ok(())` publishes ALL outputs on EVERY
//! tick, tick cadence == publish cadence. The helper rings EXACTLY
//! ONCE at startup (before any keypress) — the first tick drains an empty
//! inbox, the default state has no activity, so it emits one zero Twist — then
//! rings ONLY on real key activity, the keepalive while a non-zero command is
//! latched, and the single auto-zero transition. Steady zeros while idle are
//! NEVER published: fresh zeros from the keyboard would claim the mux
//! arbitration slot whenever the joystick is stale (the mux's own 5 Hz
//! keepalive owns the both-stale state). The no-idle guarantee is STRUCTURAL —
//! the helper does not ring while a latched ZERO is idle. Both behaviors are
//! pinned in `tests/keyboard_e2e_test.rs`.

// NOTE: no crate-level `#![forbid(unsafe_code)]` — the `#[cerulion_node]`
// macro expands cdylib FFI entry points containing `unsafe`. The PURE keymap
// module keeps the forbid at module scope (see `keymap.rs`).

// Principle #12 (structured logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod keymap;
mod pump;

// Re-export the saturation constants at the crate root so the cross-crate
// constant-consistency test (in `joystick_teleop`) has a stable path. These
// DUPLICATE the joystick node's canonical constants (a shared crate under
// `nodes/*` would be treated as a node crate by the workspace glob).
pub use keymap::{MAX_VX, MAX_VY, MAX_VYAW};

use std::sync::{Arc, Mutex};

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Twist;

use crate::keymap::{KeyOp, KeyState};

/// The keyboard teleop node. See the module docs for the full contract.
///
/// `external`-policy: fired by the `crossterm` helper's doorbell rings on the
/// live path, or by `trigger_external` on the polled/test path. One output.
// Field notes (plain comments — port fields carry only the macro attrs):
// - inbox: the shared key-op queue between the hardware pump (live) or the
//   test script and the node body. Drained + folded every tick.
// - state: the latched keyboard state (directions + preset scale + last-key
//   node-clock stamp driving the auto-zero).
// - cmd: the velocity command written every tick (fixed-POD Twist, zero-copy).
#[cerulion_node(external)]
#[derive(Default)]
pub struct KeyboardTeleop {
    inbox: Arc<Mutex<Vec<KeyOp>>>,
    state: KeyState,
    #[output]
    cmd: Twist,
}

impl KeyboardTeleop {
    /// Construct a node sharing `inbox` with a caller (a test script, or the
    /// hardware pump via [`Self::external_source`]). Lives IN-CRATE so the
    /// private fields are reachable (an integration test in a separate crate
    /// cannot construct the struct directly). Inject via
    /// `KeyboardTeleopEntry::with_state(KeyboardTeleop::with_inbox(inbox))`.
    pub fn with_inbox(inbox: Arc<Mutex<Vec<KeyOp>>>) -> Self {
        Self {
            inbox,
            ..Default::default()
        }
    }

    /// The shared key-op inbox — lets a test or the pump push [`KeyOp`]s the
    /// next tick drains.
    pub fn inbox(&self) -> Arc<Mutex<Vec<KeyOp>>> {
        Arc::clone(&self.inbox)
    }
}

#[cerulion_node_impl]
impl KeyboardTeleop {
    fn tick(&mut self) -> Result<(), NodeError> {
        // The node clock — read ONCE; both the fold-stamp and the auto-zero
        // derive from it, so the emitted command is a pure function of
        // (now, drained ops). (See the module docs' Determinism section for
        // the precise replay claim.)
        let now = self.now_ns();

        // Poison-RECOVERING drain: the POD-Vec contents can never be torn,
        // and delivering safety key-ops (Stop) after a panic strictly beats
        // dropping them — see the rationale comment at the key-event push
        // site in `pump.rs`. (The auto-zero additionally backstops any
        // latched motion within its window regardless.)
        let drained: Vec<KeyOp> = std::mem::take(
            &mut *self
                .inbox
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner),
        );
        for op in drained {
            self.state.apply(op, now);
        }

        // Resolve the command (applies the timed auto-zero). Write all six
        // Twist components explicitly (fixed-POD → zero-copy assignment).
        let cmd = self.state.resolve(now);
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
    /// `crossterm` pump, sharing this node's `inbox`.
    fn external_source(&mut self) -> ExternalSource {
        pump::spawn_crossterm_blocking(Arc::clone(&self.inbox))
    }

    /// Clean-shutdown terminal restore — layer 1 of the three-layer raw-mode
    /// restore (see `pump.rs`'s "Raw-mode lifecycle"). The
    /// RAII guard lives inside the DETACHED helper thread's closure, whose
    /// `Drop` races process exit on clean shutdown; this deterministic
    /// `shutdown()` call removes that race. Idempotent and best-effort — the
    /// guard's `Drop` and the panic hook remain as backstops, and running any
    /// subset in any order is safe. Harmless on the polled/test path (no raw
    /// mode was ever entered; `disable_raw_mode` on a cooked terminal — or a
    /// no-TTY stderr — is a no-op-class termios call whose error is ignored).
    fn shutdown(&mut self) -> Result<(), NodeError> {
        pump::restore_terminal_best_effort();
        Ok(())
    }
}
