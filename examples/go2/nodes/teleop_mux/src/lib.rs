// SPDX-License-Identifier: AGPL-3.0-only
//! Go2 teleop safety mux.
//!
//! A 50 Hz (`period_ms = 20`) safety arbiter between two velocity-command
//! sources — a local joystick and a (possibly remote) keyboard — that never
//! lets a stale source keep the robot moving. The WHOLE policy lives in the
//! pure [`arbitrate`](arbitrate::arbitrate) module (20 oracle-vector unit
//! tests); this file is only the transport wrapper.
//!
//! # Arbitration (see `arbitrate.rs` for the full contract)
//!
//! | joystick (age < 250 ms) | keyboard (age < 750 ms) | output |
//! |---|---|---|
//! | usable | any | joystick's command (a fresh CENTERED stick is a real zero command — it MUTES the keyboard) |
//! | stale/never/garbage | usable | keyboard's command |
//! | stale/never/garbage | stale/never/garbage | all-zero command, `ActiveSource::None` |
//!
//! "Usable" = fresh AND finite: a command with any non-finite component
//! (NaN/±Inf) makes its source ABSENT for that arbitration — the mux never
//! forwards non-finite values (see `arbitrate.rs`'s non-finite contract).
//! Freshness is `now - wire_timestamp_ns < window`, EXCLUSIVE at the
//! boundary; the age saturates, so a future-stamped frame (delivered just
//! ahead of the local clock) reads age 0 = fresh. The stamps come from
//! `InputView::wire_timestamp_ns()` — for a HELD latest-value input this is
//! the held frame's ORIGINAL publish stamp, so a source that goes silent
//! ages out (and flips the arbitration) even while its held VALUE is still
//! readable. That interplay — the hold serves the value, the staleness
//! check retires it — is the point of this node, and is pinned by the
//! headline e2e in `tests/mux_e2e_test.rs`.
//!
//! # AND-gate + startup-zero contract
//!
//! Both inputs are non-trigger latest-value context reads, so per the hold contract
//! the tick is a structural NO-OP until BOTH sources have delivered at
//! least once (the macro collapses the tick while any non-trigger input is
//! pre-first-delivery — nothing publishes, nothing is fabricated). The mux
//! therefore emits NOTHING until the gate opens. The SOURCE nodes
//! (the joystick driver and the keyboard driver) carry the matching
//! contract: each MUST publish an all-zero command at startup, so the gate
//! opens immediately and the mux begins emitting the safety zero. Nothing
//! in this node needs to (or can) work around the gate — it is the
//! platform's no-fabricated-defaults principle doing its job.
//!
//! # E-stop ladder note
//!
//! This mux is the SOFT layer of the stop ladder: on input staleness it
//! degrades the command stream to sustained zeros. The Go2 driver node maps
//! a sustained-zero command stream to its `StopMove` API; the hardware
//! e-stop stays above both. The mux never claims to be the hard stop.
//!
//! # Determinism (Principle #7)
//!
//! The tick is a pure function of (node clock `self.now_ns()`, the two
//! inputs' held frames + wire stamps, two plain state fields). No wall
//! clock is read anywhere; `last_published_zero_ns` / `prev_was_nonzero`
//! derive purely from node-clock time and prior emitted decisions, so a
//! replay reproduces the exact output sequence bit-for-bit.
//!
//! # Platform gaps (the exact wiring stops are named)
//!
//! ## `mux_state` output — schema wiring gap
//!
//! The design calls for a second output carrying
//! [`arbitrate::MuxState`] as the workspace YAML schema
//! `examples/go2/schemas/mux_state.yaml` (written — the declared contract). It
//! is NOT wired here because a node crate cannot consume a workspace YAML
//! schema; the wiring stops at:
//! - `cerulion_cli_engine/src/node_cmd.rs:630-663` — `node_build` is a
//!   plain `cargo build -p <type>`; no schema-codegen step runs;
//! - `cerulion_core/src/codegen/mod.rs:13-16` — core codegen exports
//!   `generate_schema` (Rust emitter) + `parse_rosmsg` (.msg) but NO YAML
//!   parser; YAML→`MessageSchema` lives only in
//!   `cerulion_cli_engine/src/schema_cmd.rs:198` (`parse_message_schemas`),
//!   a CLI introspection surface — the CLI engine is not a build-time
//!   dependency a node crate can sanely take;
//! - `cerulion_cli_engine/src/templates.rs:2406-2411` — a generated node
//!   lib.rs keeps a tier-3 workspace schema as a bare ident
//!   (`use MuxState;`) that nothing resolves.
//!
//! This node does NOT abuse a native ROS2 type for mux state and does
//! NOT add a custom .msg to native_ros2_messages. With YAML→Rust codegen
//! for node crates, the port would be `#[output] mux_state: MuxState`
//! publishing `decision.state` (encoding pinned in the YAML: active 0=none /
//! 1=joystick / 2=keyboard).
//!
//! ## Publish cadence — no conditional publish on the macro path
//!
//! `arbitrate()` computes a `publish` flag (nonzero → every tick;
//! nonzero→zero transition → immediately; steady zero → 5 Hz keepalive).
//! The macro path cannot honor it: the generated tick tail ARMS every
//! declared output on ANY `Ok` tick outcome
//! (`cerulion_macros/src/impl_macro.rs:2497-2531` — `__cer_arm_publish` on
//! `is_ok()` / `Ok(Ok(true))`, which also clobbers any body-side
//! `__cer_defer_publish`), and an unwritten fixed-schema output would
//! publish its zero-initialized loan anyway
//! (`cerulion_core/src/transport/publisher.rs:885-912` — the fixed prefix
//! is zero-initialized at loan;
//! `cerulion_core/src/transport/output_proxy.rs:87-111` — publish-on-success
//! inversion, discard only on Err/collapse legs). An `Err` return would
//! suppress the publish but floods the runtime's tick-failed diagnostics —
//! the wrong tool. So this node WRITES `decision.cmd` EVERY tick: steady
//! zeros go out at 50 Hz instead of the 5 Hz keepalive — a strict superset
//! of the keepalive contract (every downstream liveness observation still
//! holds; the transition-to-zero still lands on its exact tick), safe and
//! byte-deterministic. The keepalive STATE (`last_published_zero_ns`) is
//! still threaded per the contract — keyed on `decision.publish`, i.e. the
//! 5 Hz cadence, via the pure `arbitrate::state_after` helper (oracle-tested
//! transport-free) — so with a conditional-publish surface,
//! honoring `decision.publish` is a one-line change with no state migration.

// NOTE: no crate-level `#![forbid(unsafe_code)]` — the `#[cerulion_node]`
// macro expands cdylib FFI entry points containing `unsafe` (as every
// Cerulion node crate does). The PURE arbitration module keeps the forbid
// at module scope (see `arbitrate.rs`).

// P12 (the AGENTS.md logging convention): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod arbitrate;

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Twist;

use crate::arbitrate::{arbitrate, state_after, TwistCmd};

/// The teleop safety mux node. See the module docs for the full contract.
///
/// 50 Hz periodic; both inputs are non-trigger latest-value context reads
/// (held across steps), so the mux keeps arbitrating — and
/// keeps emitting — while either source is silent.
// Field notes (plain comments — port fields carry only the macro attrs):
// - joy_cmd: joystick velocity command (local, low-latency; 250 ms freshness).
// - key_cmd: keyboard velocity command (network headroom; 750 ms freshness).
// - cmd: the arbitrated velocity command (written every tick — see the
//   module docs' publish-cadence note).
// - last_published_zero_ns: node-clock time of the last keepalive-cadence
//   zero (the pure contract's 5 Hz bookkeeping).
// - prev_was_nonzero: whether the previously emitted command was nonzero
//   (drives the immediate nonzero→zero transition publish).
#[cerulion_node(period_ms = 20)]
#[derive(Default)]
pub struct TeleopMux {
    #[input]
    joy_cmd: Twist,
    #[input]
    key_cmd: Twist,
    #[output]
    cmd: Twist,
    last_published_zero_ns: Option<u64>,
    prev_was_nonzero: bool,
}

#[cerulion_node_impl]
impl TeleopMux {
    fn tick(&mut self) -> Result<(), NodeError> {
        let now = self.now_ns();

        // Both views are guaranteed served here: if either non-trigger
        // input had never delivered, the macro would have collapsed this
        // whole tick to a no-op (the AND-gate — see module docs),
        // so the `None` arms of `arbitrate` are unreachable from this
        // wrapper (they remain for the pure function's totality + tests).
        // The wire stamp is the HELD frame's ORIGINAL publish stamp, so a
        // silent source ages out even while its value is still readable.
        let joy = Some((
            self.joy_cmd.wire_timestamp_ns(),
            TwistCmd {
                linear: [
                    self.joy_cmd.linear.x,
                    self.joy_cmd.linear.y,
                    self.joy_cmd.linear.z,
                ],
                angular: [
                    self.joy_cmd.angular.x,
                    self.joy_cmd.angular.y,
                    self.joy_cmd.angular.z,
                ],
            },
        ));
        let key = Some((
            self.key_cmd.wire_timestamp_ns(),
            TwistCmd {
                linear: [
                    self.key_cmd.linear.x,
                    self.key_cmd.linear.y,
                    self.key_cmd.linear.z,
                ],
                angular: [
                    self.key_cmd.angular.x,
                    self.key_cmd.angular.y,
                    self.key_cmd.angular.z,
                ],
            },
        ));

        let decision = arbitrate(
            now,
            joy,
            key,
            self.last_published_zero_ns,
            self.prev_was_nonzero,
        );

        // Write the arbitrated command into the loaned SHM slot. EVERY
        // tick — the macro path publishes on any Ok outcome regardless
        // (see the module docs' publish-cadence note), so writing
        // explicitly is the accurate form (an unwritten output would publish
        // a zero-init frame with identical bytes on exactly the ticks the
        // contract would have suppressed).
        self.cmd.linear.x = decision.cmd.linear[0];
        self.cmd.linear.y = decision.cmd.linear[1];
        self.cmd.linear.z = decision.cmd.linear[2];
        self.cmd.angular.x = decision.cmd.angular[0];
        self.cmd.angular.y = decision.cmd.angular[1];
        self.cmd.angular.z = decision.cmd.angular[2];

        // Contract-faithful keepalive bookkeeping via the PURE threading
        // policy (`state_after` — oracle-tested in arbitrate.rs): keyed on
        // the contract's publish decision (the 5 Hz cadence), NOT on the
        // wire's current always-publish behavior — so the state machine
        // evolves exactly as a conditional publish would drive it
        // (the dormant contract).
        let (last_zero, prev_nonzero) = state_after(decision, now, self.last_published_zero_ns);
        self.last_published_zero_ns = last_zero;
        self.prev_was_nonzero = prev_nonzero;

        Ok(())
    }
}
