// SPDX-License-Identifier: AGPL-3.0-only
//! Pure axis→Twist mapping core for the Go2 joystick teleop node.
//!
//! Everything in this module is a pure function of its arguments — NO I/O, no
//! wall clock, no `gilrs` types — so the whole teleop policy is deterministic
//! and oracle-testable in isolation. The hardware `gilrs` pump
//! ([`crate::pump`]) translates raw gamepad events into the [`PadEvent`]s this
//! module folds; the node body ([`crate::JoystickTeleop`]) folds them into a
//! [`PadState`] and calls [`PadState::command`] to produce the emitted Twist.
//!
//! Determinism scope: this purity claim is about the
//! FUNCTION — same folded state in, same command out. Platform replay
//! verifies the NODE by byte-comparing its recorded published frames
//! (Principle #7); it does not re-drive gamepad input. See the `lib.rs`
//! Determinism section for the precise node-level claim.
//!
//! # Mapping (constants below, all documented)
//!
//! | Physical input | gilrs axis/button | Output DOF | Sign |
//! |---|---|---|---|
//! | left stick up/down | `LeftStickY` (up = +1) | `linear.x` (forward) | up ⇒ +x (forward) |
//! | left stick left/right | `LeftStickX` (right = +1) | `linear.y` (strafe) | left ⇒ +y (strafe left) |
//! | right stick left/right | `RightStickX` (right = +1) | `angular.z` (yaw) | left ⇒ +z (yaw-left) |
//! | RB shoulder | `RightTrigger` | deadman | held ⇒ armed |
//!
//! The Go2 `Move` frame is **x forward, y left, z yaw-left**. gilrs reports
//! stick X positive to the RIGHT, so both strafe and yaw NEGATE the raw axis
//! (right stick ⇒ negative y / negative z ⇒ move/turn right). These sign
//! conventions are pinned by [`tests`].
//!
//! # Deadman (safety contract)
//!
//! No motion is EVER emitted unless the deadman (RB) is held AND a gamepad is
//! connected. [`PadState::command`] returns [`Cmd::ZERO`] whenever
//! `deadman && connected` is false — releasing the deadman, or a Bluetooth
//! drop, both zero the command. This is the contract that makes an armed
//! robot safe: no held deadman ⇒ no motion, ever.
//!
//! # Per-axis deadzone + saturation
//!
//! Each raw axis in `[-1, 1]` passes through [`apply_deadzone`]: values with
//! magnitude below [`DEADZONE`] read as exactly `0.0`; above it, the response
//! is RESCALED so the deadzone edge maps to 0 and full deflection maps to 1
//! (no jump at the edge), then clamped to `[-1, 1]` (saturation — an
//! out-of-range reading can never exceed the configured max). A non-finite
//! axis (NaN/±Inf — defensive; gilrs reports finite floats) reads as `0.0`.

// The pure module is unsafe-free by construction. (The forbid lives HERE,
// not at the crate root, because the `#[cerulion_node]` macro in lib.rs
// expands cdylib FFI entry points containing `unsafe`.)
#![forbid(unsafe_code)]

/// Max forward/back speed (m/s) at full stick deflection. Conservative —
/// well under the Go2's sport-mode limits.
pub const MAX_VX: f64 = 0.6;
/// Max strafe speed (m/s) at full stick deflection.
pub const MAX_VY: f64 = 0.4;
/// Max yaw rate (rad/s) at full stick deflection.
pub const MAX_VYAW: f64 = 1.0;
/// Stick deadzone: an axis magnitude strictly below this reads as `0.0`.
/// EXCLUSIVE at the boundary — a reading of exactly `DEADZONE` maps to `0.0`
/// (the rescale numerator is zero there); see [`apply_deadzone`].
pub const DEADZONE: f64 = 0.1;

/// A 3-DOF holonomic velocity command (the DOFs a Go2 `Move` accepts). Maps
/// onto `geometry_msgs/Twist` as `linear.x = vx`, `linear.y = vy`,
/// `angular.z = vyaw`, with every other Twist component `0.0`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cmd {
    /// Forward velocity (m/s), +x = forward.
    pub vx: f64,
    /// Strafe velocity (m/s), +y = left.
    pub vy: f64,
    /// Yaw rate (rad/s), +z = yaw-left.
    pub vyaw: f64,
}

impl Cmd {
    /// The all-zero command (robot stop).
    pub const ZERO: Self = Self {
        vx: 0.0,
        vy: 0.0,
        vyaw: 0.0,
    };

    /// True iff all three DOFs are exactly `0.0` (a deliberate, exact float
    /// equality — `-0.0` compares equal to `0.0`). Used only by tests /
    /// diagnostics; the wire always carries the value regardless.
    #[allow(clippy::float_cmp)]
    #[inline]
    pub fn is_zero(&self) -> bool {
        self.vx == 0.0 && self.vy == 0.0 && self.vyaw == 0.0
    }
}

/// Which analog stick axis a [`PadEvent::Axis`] carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq, cerulion_core::state::CerulionState)]
pub enum PadAxis {
    /// Left stick vertical (gilrs `LeftStickY`, up = +1) → forward velocity.
    LeftY,
    /// Left stick horizontal (gilrs `LeftStickX`, right = +1) → strafe.
    LeftX,
    /// Right stick horizontal (gilrs `RightStickX`, right = +1) → yaw.
    RightX,
}

/// A hardware-agnostic gamepad event: the seam between the wall-clock `gilrs`
/// pump (or a scripted test) and the pure node body. Carries NO timestamp —
/// all timing lives in the pump's ring cadence (live) or the node clock
/// (never needed for the joystick's output, which is a pure function of the
/// folded [`PadState`]).
#[derive(Debug, Clone, Copy, PartialEq, cerulion_core::state::CerulionState)]
pub enum PadEvent {
    /// A stick axis moved to `value` (raw, in `[-1, 1]`).
    Axis(PadAxis, f64),
    /// The deadman (RB shoulder) was pressed (`true`) or released (`false`).
    Deadman(bool),
    /// A gamepad connected.
    Connected,
    /// A gamepad disconnected (Bluetooth drop / unplug).
    Disconnected,
}

/// The folded gamepad state — the "drained shared state" the node body reads.
/// [`PadState::apply`] folds each [`PadEvent`]; [`PadState::command`] maps the
/// current state to a [`Cmd`]. Both are pure.
///
/// `connected` starts `true`: a fresh node with no `Connected` event yet still
/// emits its startup zero (deadman not held ⇒ [`Cmd::ZERO`] regardless), and
/// the pump pushes an explicit `Disconnected` the moment a device drops. This
/// avoids a spurious "never connected" state that would make the deadman
/// gate's `connected` term redundant with `deadman` at startup.
#[derive(Debug, Clone, Copy, PartialEq, cerulion_core::state::CerulionState)]
pub struct PadState {
    /// Raw left-stick-Y reading (forward axis), `[-1, 1]`.
    pub left_y: f64,
    /// Raw left-stick-X reading (strafe axis), `[-1, 1]`.
    pub left_x: f64,
    /// Raw right-stick-X reading (yaw axis), `[-1, 1]`.
    pub right_x: f64,
    /// Whether the deadman (RB) is currently held.
    pub deadman: bool,
    /// Whether a gamepad is currently connected.
    pub connected: bool,
}

impl Default for PadState {
    fn default() -> Self {
        Self {
            left_y: 0.0,
            left_x: 0.0,
            right_x: 0.0,
            deadman: false,
            connected: true,
        }
    }
}

impl PadState {
    /// Fold one [`PadEvent`] into the state. Pure — no I/O, no clock.
    ///
    /// A `Disconnected` also DISARMS the deadman: on reconnect the physical
    /// button state is unknown, so the operator must re-press RB to arm
    /// (fail-safe — a held button across a BT drop must not silently rearm).
    pub fn apply(&mut self, event: PadEvent) {
        match event {
            PadEvent::Axis(PadAxis::LeftY, v) => self.left_y = v,
            PadEvent::Axis(PadAxis::LeftX, v) => self.left_x = v,
            PadEvent::Axis(PadAxis::RightX, v) => self.right_x = v,
            PadEvent::Deadman(held) => self.deadman = held,
            PadEvent::Connected => self.connected = true,
            PadEvent::Disconnected => {
                self.connected = false;
                self.deadman = false;
            }
        }
    }

    /// Map the current state to a velocity command. Returns [`Cmd::ZERO`]
    /// unless the deadman is held AND a gamepad is connected (the safety
    /// gate); otherwise applies the deadzone + saturation + sign conventions
    /// documented at the module level.
    pub fn command(&self) -> Cmd {
        if !(self.deadman && self.connected) {
            return Cmd::ZERO;
        }
        Cmd {
            vx: apply_deadzone(self.left_y) * MAX_VX,
            // gilrs stick-right is +x; Go2 +y is LEFT ⇒ negate.
            vy: -apply_deadzone(self.left_x) * MAX_VY,
            // gilrs stick-right is +x; Go2 +z is yaw-LEFT ⇒ negate.
            vyaw: -apply_deadzone(self.right_x) * MAX_VYAW,
        }
    }
}

/// Apply the per-axis deadzone + rescale + saturation to a raw axis reading.
///
/// - Non-finite input (NaN/±Inf) ⇒ `0.0` (fail-safe: never propagate garbage).
/// - `|v| < DEADZONE` ⇒ `0.0`.
/// - Otherwise the magnitude is rescaled so `DEADZONE` maps to `0.0` and
///   `1.0` maps to `1.0`, clamped to `[0, 1]` (saturation for `|v| > 1`), and
///   the original sign is reapplied.
///
/// The result is in `[-1, 1]`; the caller multiplies by the axis max.
#[inline]
pub fn apply_deadzone(v: f64) -> f64 {
    if !v.is_finite() {
        return 0.0;
    }
    let mag = v.abs();
    if mag < DEADZONE {
        return 0.0;
    }
    let scaled = ((mag - DEADZONE) / (1.0 - DEADZONE)).min(1.0);
    v.signum() * scaled
}

#[cfg(test)]
mod tests {
    // These tests deliberately assert EXACT float values that are exact by
    // construction (single multiplies by 0.0 / ±1.0 / a shared const), so a
    // direct `==` is correct here.
    #![allow(clippy::float_cmp)]

    use super::*;

    /// Tight tolerance for the CONTINUOUS mapping arms (float rescale). The
    /// zero/full-scale/sign arms below use EXACT equality where the result is
    /// a single multiply by 0.0 / ±1.0 (bit-exact).
    const EPS: f64 = 1e-12;

    fn armed(left_y: f64, left_x: f64, right_x: f64) -> PadState {
        PadState {
            left_y,
            left_x,
            right_x,
            deadman: true,
            connected: true,
        }
    }

    // 1 — deadman NOT held ⇒ zero regardless of stick position.
    #[test]
    fn deadman_released_zeroes_all_axes() {
        let mut s = armed(1.0, 1.0, 1.0);
        s.deadman = false;
        assert_eq!(s.command(), Cmd::ZERO);
    }

    // 2 — disconnected ⇒ zero even with deadman held (defense in depth).
    #[test]
    fn disconnected_zeroes_even_with_deadman() {
        let mut s = armed(1.0, 0.0, 0.0);
        s.connected = false;
        assert_eq!(s.command(), Cmd::ZERO);
    }

    // 3 — armed + centered sticks ⇒ zero.
    #[test]
    fn armed_centered_is_zero() {
        assert_eq!(armed(0.0, 0.0, 0.0).command(), Cmd::ZERO);
    }

    // 4 — full forward: LeftStickY = +1 ⇒ vx = +MAX_VX (exact — ×1.0).
    #[test]
    fn full_forward_is_max_vx() {
        let c = armed(1.0, 0.0, 0.0).command();
        assert_eq!(c.vx, MAX_VX);
        assert_eq!(c.vy, 0.0);
        assert_eq!(c.vyaw, 0.0);
    }

    // 5 — full back: LeftStickY = -1 ⇒ vx = -MAX_VX.
    #[test]
    fn full_back_is_neg_max_vx() {
        assert_eq!(armed(-1.0, 0.0, 0.0).command().vx, -MAX_VX);
    }

    // 6 — strafe sign: stick RIGHT (LeftStickX = +1) ⇒ strafe RIGHT (vy = -MAX_VY);
    // stick LEFT ⇒ strafe LEFT (vy = +MAX_VY). Pins the Go2 +y = left convention.
    #[test]
    fn strafe_sign_matches_go2_left_positive() {
        assert_eq!(armed(0.0, 1.0, 0.0).command().vy, -MAX_VY);
        assert_eq!(armed(0.0, -1.0, 0.0).command().vy, MAX_VY);
    }

    // 7 — yaw sign: stick RIGHT (RightStickX = +1) ⇒ yaw RIGHT (vyaw = -MAX_VYAW);
    // stick LEFT ⇒ yaw LEFT (vyaw = +MAX_VYAW). Pins +z = yaw-left.
    #[test]
    fn yaw_sign_matches_go2_yaw_left_positive() {
        assert_eq!(armed(0.0, 0.0, 1.0).command().vyaw, -MAX_VYAW);
        assert_eq!(armed(0.0, 0.0, -1.0).command().vyaw, MAX_VYAW);
    }

    // 8 — deadzone boundary is EXCLUSIVE: exactly DEADZONE and just under ⇒ 0.
    #[test]
    fn deadzone_boundary_exclusive() {
        assert_eq!(apply_deadzone(DEADZONE), 0.0);
        assert_eq!(apply_deadzone(-DEADZONE), 0.0);
        assert_eq!(apply_deadzone(0.05), 0.0);
        assert_eq!(apply_deadzone(-0.099), 0.0);
    }

    // 9 — deadzone RESCALE against HAND-COMPUTED LITERALS (an oracle that
    // recomputes the implementation's formula is a shared-bug tautology).
    // By hand: input 0.55, deadzone 0.1 ⇒ (0.55 − 0.1)/0.9 =
    // 0.45/0.9 = 0.5 exactly. Through vx: 0.5 × 0.6 m/s = 0.3 m/s.
    #[test]
    fn deadzone_rescale_midpoint() {
        assert!((apply_deadzone(0.55) - 0.5).abs() < EPS);
        // Odd symmetry: −0.55 ⇒ −0.5.
        assert!((apply_deadzone(-0.55) + 0.5).abs() < EPS);
        // Mapped through vx: hand literal 0.3 m/s.
        let c = armed(0.55, 0.0, 0.0).command();
        assert!((c.vx - 0.3).abs() < EPS);
    }

    // 10 — saturation: an over-range axis (|v| > 1, defensive) clamps to ±max.
    #[test]
    fn saturation_clamps_over_range_axis() {
        assert_eq!(apply_deadzone(1.5), 1.0);
        assert_eq!(apply_deadzone(-2.0), -1.0);
        assert_eq!(armed(1.5, 0.0, 0.0).command().vx, MAX_VX);
    }

    // 11 — non-finite axis ⇒ 0 (fail-safe; a NaN must never reach the robot).
    #[test]
    fn non_finite_axis_is_zeroed() {
        assert_eq!(apply_deadzone(f64::NAN), 0.0);
        assert_eq!(apply_deadzone(f64::INFINITY), 0.0);
        assert_eq!(apply_deadzone(f64::NEG_INFINITY), 0.0);
        let c = armed(f64::NAN, f64::INFINITY, 0.0).command();
        assert_eq!(c.vx, 0.0);
        assert_eq!(c.vy, 0.0);
    }

    // 12 — apply(): the fold updates the right field; Disconnected disarms.
    #[test]
    fn apply_folds_events_and_disconnect_disarms() {
        let mut s = PadState::default();
        s.apply(PadEvent::Deadman(true));
        s.apply(PadEvent::Axis(PadAxis::LeftY, 1.0));
        assert!(s.deadman && s.connected);
        assert_eq!(s.command().vx, MAX_VX);
        // A BT drop disarms AND zeroes.
        s.apply(PadEvent::Disconnected);
        assert!(!s.connected && !s.deadman);
        assert_eq!(s.command(), Cmd::ZERO);
        // Reconnect alone does NOT rearm (deadman still false).
        s.apply(PadEvent::Connected);
        assert_eq!(s.command(), Cmd::ZERO);
        // Re-press RB rearms.
        s.apply(PadEvent::Deadman(true));
        assert_eq!(s.command().vx, MAX_VX);
    }

    // 13 — combined 3-axis full deflection with the documented signs.
    #[test]
    fn combined_full_deflection_all_axes() {
        // Forward + strafe-left + yaw-left.
        let c = armed(1.0, -1.0, -1.0).command();
        assert_eq!(c.vx, MAX_VX);
        assert_eq!(c.vy, MAX_VY);
        assert_eq!(c.vyaw, MAX_VYAW);
    }

    // 14 — DETERMINISM: folding an identical event script twice yields
    // bit-identical commands (the pure core is a function; this guards against
    // hidden nondeterminism creeping into apply/command). Axis values chosen
    // so every expected output is a clean HAND LITERAL (with
    // no formula recompute): |0.55| rescales to 0.5 exactly ((0.55−0.1)/0.9).
    #[test]
    fn folding_is_deterministic() {
        let script = [
            PadEvent::Connected,
            PadEvent::Deadman(true),
            PadEvent::Axis(PadAxis::LeftY, 0.55),
            PadEvent::Axis(PadAxis::LeftX, -0.55),
            PadEvent::Axis(PadAxis::RightX, 0.55),
        ];
        let run = || {
            let mut s = PadState::default();
            let mut out = Vec::new();
            for &e in &script {
                s.apply(e);
                out.push(s.command());
            }
            out
        };
        let a = run();
        let b = run();
        assert_eq!(
            a, b,
            "identical scripts must fold to bit-identical commands"
        );
        // The final command equals HAND-COMPUTED literals:
        //   vx   = 0.5 × 0.6            = 0.3  (forward)
        //   vy   = −(−0.5) × 0.4        = 0.2  (stick LEFT ⇒ strafe LEFT ⇒ +y)
        //   vyaw = −(0.5) × 1.0         = −0.5 (stick RIGHT ⇒ yaw RIGHT ⇒ −z)
        let last = *a.last().expect("non-empty");
        assert!((last.vx - 0.3).abs() < EPS);
        assert!((last.vy - 0.2).abs() < EPS);
        assert!((last.vyaw + 0.5).abs() < EPS);
    }
}
